#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <driver/uart.h>
#include <Adafruit_NeoPixel.h>


// Configuration values for the DMX sender
namespace Config {
constexpr uart_port_t DmxUart = UART_NUM_2;
constexpr int DmxRxPin = 16;
constexpr int DmxTxPin = 17;
constexpr int Rs485DePin = 4;
constexpr int StatusLedPin = 5;
constexpr uint8_t StatusLedBrightness = 32;
constexpr uint32_t DmxBaud = 250000;
constexpr uint16_t MaxDmxChannels = 512;
constexpr uint16_t MaxDmxBytes = MaxDmxChannels + 1;
constexpr uint8_t UniverseId = 0;
constexpr uint8_t ProtocolVersion = 1;
constexpr uint8_t WifiChannel = 6;
constexpr uint16_t EspNowPayloadLimit = 250;
constexpr uint8_t BroadcastAddress[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
}

struct __attribute__((packed)) DmxPacketHeader {
  uint8_t version;
  uint8_t universe;
  uint32_t sequence;
  uint8_t fragment;
  uint8_t fragmentCount;
  uint16_t totalLength;
  uint16_t dataLength;
};

constexpr size_t FragmentDataSize = Config::EspNowPayloadLimit - sizeof(DmxPacketHeader);
constexpr uint8_t FragmentCount = (Config::MaxDmxBytes + FragmentDataSize - 1) / FragmentDataSize;

struct DmxFrame {
  uint16_t length;
  uint8_t data[Config::MaxDmxBytes];
};

QueueHandle_t completeFrames;
QueueHandle_t uartEvents;
uint32_t nextSequence = 0;
uint32_t receivedFrames = 0;
uint32_t invalidFrames = 0;
uint32_t sentPackets = 0;
uint32_t sendFailures = 0;
Adafruit_NeoPixel statusLed(1, Config::StatusLedPin, NEO_GRB + NEO_KHZ800);

enum class LedStatus : uint8_t { Unconnected, Receiving, Error };

void updateStatusLed() {
  static uint32_t lastInvalidFrames = 0;
  static uint32_t lastSendFailures = 0;
  static uint32_t lastErrorAt = 0;
  static bool errorActive = false;
  static uint32_t lastReceivedFrames = 0;
  static uint32_t lastDmxAt = 0;
  static uint32_t lastBlinkAt = 0;
  static bool blinkOn = false;
  static uint32_t lastColor = 0xffffffff;
  const uint32_t now = millis();

  if (invalidFrames != lastInvalidFrames || sendFailures != lastSendFailures) {
    lastInvalidFrames = invalidFrames;
    lastSendFailures = sendFailures;
    lastErrorAt = now;
    errorActive = true;
  }
  if (receivedFrames != lastReceivedFrames) {
    lastReceivedFrames = receivedFrames;
    lastDmxAt = now;
  }

  const LedStatus status = (errorActive && now - lastErrorAt < 2000) ? LedStatus::Error :
                           (now - lastDmxAt < 1000) ? LedStatus::Receiving :
                                                      LedStatus::Unconnected;
  if (status == LedStatus::Receiving && now - lastBlinkAt >= 250) {
    lastBlinkAt = now;
    blinkOn = !blinkOn;
  }

  uint32_t color = status == LedStatus::Error ? statusLed.Color(255, 180, 0) :
                   status == LedStatus::Receiving && blinkOn ? statusLed.Color(0, 255, 0) :
                   status == LedStatus::Unconnected ? statusLed.Color(255, 0, 0) :
                                                       statusLed.Color(0, 0, 0);
  if (color != lastColor) {
    lastColor = color;
    statusLed.setPixelColor(0, color);
    statusLed.show();
  }
}

void onSend(const uint8_t *, esp_now_send_status_t status) {
  // prevent ESP-NOW callbacks from calling serial IOs
  if (status != ESP_NOW_SEND_SUCCESS) sendFailures++;
}

void sendFrame(const DmxFrame &frame) {
  if (frame.length == 0 || frame.length > Config::MaxDmxBytes) return;
  const uint32_t sequence = nextSequence++;
  const uint8_t fragmentCount = (frame.length + FragmentDataSize - 1) / FragmentDataSize;
  for (uint8_t fragment = 0; fragment < fragmentCount; ++fragment) {
    const size_t offset = fragment * FragmentDataSize;
    const size_t dataLength = min(FragmentDataSize, static_cast<size_t>(frame.length) - offset);
    uint8_t packet[Config::EspNowPayloadLimit] = {};
    auto *header = reinterpret_cast<DmxPacketHeader *>(packet);
    header->version = Config::ProtocolVersion;
    header->universe = Config::UniverseId;
    header->sequence = sequence;
    header->fragment = fragment;
    header->fragmentCount = fragmentCount;
    header->totalLength = frame.length;
    header->dataLength = dataLength;
    memcpy(packet + sizeof(DmxPacketHeader), frame.data + offset, dataLength);
    const esp_err_t result = esp_now_send(Config::BroadcastAddress, packet,
                                          sizeof(DmxPacketHeader) + dataLength);
    if (result == ESP_OK) sentPackets++;
    else sendFailures++;
  }
}

void setupDmxInput() {
  uart_config_t config = {};
  config.baud_rate = Config::DmxBaud;
  config.data_bits = UART_DATA_8_BITS;
  config.parity = UART_PARITY_DISABLE;
  config.stop_bits = UART_STOP_BITS_2;
  config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  config.source_clk = UART_SCLK_APB;
  ESP_ERROR_CHECK(uart_param_config(Config::DmxUart, &config));
  ESP_ERROR_CHECK(uart_set_pin(Config::DmxUart, Config::DmxTxPin, Config::DmxRxPin,
                               UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  ESP_ERROR_CHECK(uart_driver_install(Config::DmxUart, 2048, 0, 20, &uartEvents, 0));
  // flush a short byte tail into UART_DATA, completion still requires UART_BREAK
  ESP_ERROR_CHECK(uart_set_rx_timeout(Config::DmxUart, 2));
}

void setupEspNow() {
  WiFi.mode(WIFI_STA);
  ESP_ERROR_CHECK(esp_wifi_set_channel(Config::WifiChannel, WIFI_SECOND_CHAN_NONE));
  if (esp_now_init() != ESP_OK) {
    while (true) delay(1000);
  }
  esp_now_register_send_cb(onSend);
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, Config::BroadcastAddress, 6);
  peer.channel = Config::WifiChannel;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  ESP_ERROR_CHECK(esp_now_add_peer(&peer));
}

void setup() {
  statusLed.begin();
  statusLed.setBrightness(Config::StatusLedBrightness);
  statusLed.clear();
  statusLed.show();
  pinMode(Config::Rs485DePin, OUTPUT);
  digitalWrite(Config::Rs485DePin, LOW);
  completeFrames = xQueueCreate(2, sizeof(DmxFrame));
  setupDmxInput();
  setupEspNow();
}

void loop() {
  static DmxFrame assembling = {};
  uart_event_t event;
  if (xQueueReceive(uartEvents, &event, 0) == pdTRUE) {
    if (event.type == UART_BREAK) {
      // A break is the DMX frame boundary, ignore inter byte idle time
      if (assembling.length > 0) {
        if (assembling.data[0] == 0 && xQueueSend(completeFrames, &assembling, 0) == pdTRUE) {
          receivedFrames++;
        } else {
          invalidFrames++;
        }
      }
      assembling.length = 0;
    } else if (event.type == UART_DATA) {
      uint8_t bytes[128];
      size_t remaining = event.size;
      while (remaining > 0) {
        const size_t count = uart_read_bytes(Config::DmxUart, bytes, min(remaining, sizeof(bytes)), 0);
        if (count == 0) break;
        remaining -= count;
        const size_t available = Config::MaxDmxBytes - assembling.length;
        const size_t copyCount = min(count, available);
        memcpy(assembling.data + assembling.length, bytes, copyCount);
        assembling.length += copyCount;
      }
    } else if (event.type == UART_FIFO_OVF || event.type == UART_BUFFER_FULL ||
               event.type == UART_PARITY_ERR || event.type == UART_FRAME_ERR) {
      uart_flush_input(Config::DmxUart);
      xQueueReset(uartEvents);
      invalidFrames++;
      assembling.length = 0;
    }
  }
  DmxFrame frame;
  if (xQueueReceive(completeFrames, &frame, 0) == pdTRUE) sendFrame(frame);
  updateStatusLed();
}