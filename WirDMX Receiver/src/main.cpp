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

struct WirelessFragment {
  uint16_t length;
  uint8_t data[Config::EspNowPayloadLimit];
};

struct PendingFrame {
  bool active;
  uint32_t sequence;
  uint16_t totalLength;
  uint8_t fragmentCount;
  uint8_t receivedMask;
  uint8_t data[Config::MaxDmxBytes];
};

QueueHandle_t wirelessFragments;
uint8_t outputFrame[Config::MaxDmxBytes] = {0};
uint32_t currentSequence = 0;
bool haveSequence = false;
uint32_t packetsDiscarded = 0;
uint32_t packetsLost = 0;
uint32_t acceptedFrames = 0;
uint32_t lastOutput = 0;
Adafruit_NeoPixel statusLed(1, Config::StatusLedPin, NEO_GRB + NEO_KHZ800);

enum class LedStatus : uint8_t { Unconnected, Receiving, Error };

void updateStatusLed() {
  static uint32_t lastDiscarded = 0;
  static uint32_t lastLost = 0;
  static uint32_t lastErrorAt = 0;
  static bool errorActive = false;
  static uint32_t lastAccepted = 0;
  static uint32_t lastFrameAt = 0;
  static uint32_t lastBlinkAt = 0;
  static bool blinkOn = false;
  static uint32_t lastColor = 0xffffffff;
  const uint32_t now = millis();

  if (packetsDiscarded != lastDiscarded || packetsLost != lastLost) {
    lastDiscarded = packetsDiscarded;
    lastLost = packetsLost;
    lastErrorAt = now;
    errorActive = true;
  }
  if (acceptedFrames != lastAccepted) {
    lastAccepted = acceptedFrames;
    lastFrameAt = now;
  }

  const LedStatus status = (errorActive && now - lastErrorAt < 2000) ? LedStatus::Error :
                           (now - lastFrameAt < 1000) ? LedStatus::Receiving :
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

bool sequenceIsNewer(uint32_t candidate, uint32_t reference) {
  return static_cast<int32_t>(candidate - reference) > 0;
}

void onReceive(const uint8_t *, const uint8_t *data, int length) {
  // the ESP-NOW buffer is only valid during this callback
  if (length <= 0 || length > Config::EspNowPayloadLimit) return;
  WirelessFragment fragment = {};
  fragment.length = length;
  memcpy(fragment.data, data, length);
  if (xQueueSend(wirelessFragments, &fragment, 0) != pdTRUE) packetsDiscarded++;
}

void setupDmxOutput() {
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
  ESP_ERROR_CHECK(uart_driver_install(Config::DmxUart, 2048, 2048, 0, nullptr, 0));
  pinMode(Config::Rs485DePin, OUTPUT);
  digitalWrite(Config::Rs485DePin, HIGH);
}

void setupEspNow() {
  WiFi.mode(WIFI_STA);
  ESP_ERROR_CHECK(esp_wifi_set_channel(Config::WifiChannel, WIFI_SECOND_CHAN_NONE));
  if (esp_now_init() != ESP_OK) {
    while (true) delay(1000);
  }
  esp_now_register_recv_cb(onReceive);
}

void processFragment(const WirelessFragment &fragment) {
  if (fragment.length < sizeof(DmxPacketHeader)) { packetsDiscarded++; return; }
  const auto *header = reinterpret_cast<const DmxPacketHeader *>(fragment.data);
    if (header->version != Config::ProtocolVersion || header->universe != Config::UniverseId ||
      header->fragmentCount == 0 || header->fragmentCount > FragmentCount ||
      header->fragment >= header->fragmentCount || header->totalLength == 0 ||
      header->totalLength > Config::MaxDmxBytes ||
      header->dataLength != fragment.length - sizeof(DmxPacketHeader) ||
      header->dataLength > FragmentDataSize) { packetsDiscarded++; return; }
  const size_t offset = header->fragment * FragmentDataSize;
  if (offset + header->dataLength > header->totalLength) { packetsDiscarded++; return; }
  const size_t expectedLength = min(FragmentDataSize,
                                    static_cast<size_t>(header->totalLength) - offset);
  if (header->dataLength != expectedLength) { packetsDiscarded++; return; }
  if (haveSequence && !sequenceIsNewer(header->sequence, currentSequence)) {
    packetsDiscarded++;
    return;
  }

  static PendingFrame pending[3] = {};
  PendingFrame *slot = nullptr;
  for (PendingFrame &candidate : pending) {
    if (candidate.active && candidate.sequence == header->sequence) {
      slot = &candidate;
      break;
    }
  }
  if (slot == nullptr) {
    for (PendingFrame &candidate : pending) {
      if (!candidate.active) { slot = &candidate; break; }
    }
  }
  if (slot == nullptr) {
    slot = &pending[0];
    for (PendingFrame &candidate : pending) {
      if (sequenceIsNewer(slot->sequence, candidate.sequence)) slot = &candidate;
    }
  }
  if (!slot->active || slot->sequence != header->sequence) {
    slot->active = true;
    slot->sequence = header->sequence;
    slot->totalLength = header->totalLength;
    slot->fragmentCount = header->fragmentCount;
    slot->receivedMask = 0;
  }
  memcpy(slot->data + offset, fragment.data + sizeof(DmxPacketHeader), header->dataLength);
  slot->receivedMask |= 1U << header->fragment;
  if (slot->receivedMask != ((1U << slot->fragmentCount) - 1U)) return;
  if (haveSequence && !sequenceIsNewer(slot->sequence, currentSequence)) {
    slot->active = false;
    packetsDiscarded++;
    return;
  }
  if (haveSequence) packetsLost += slot->sequence - currentSequence - 1;
  memcpy(outputFrame, slot->data, slot->totalLength);
  if (slot->totalLength < Config::MaxDmxBytes) {
    memset(outputFrame + slot->totalLength, 0, Config::MaxDmxBytes - slot->totalLength);
  }
  currentSequence = slot->sequence;
  haveSequence = true;
  acceptedFrames++;
  slot->active = false;
}

void outputDmxFrame() {
  digitalWrite(Config::Rs485DePin, HIGH);
  uart_write_bytes_with_break(Config::DmxUart, outputFrame + 1, Config::MaxDmxChannels, 25);
}

void setup() {
  statusLed.begin();
  statusLed.setBrightness(Config::StatusLedBrightness);
  statusLed.clear();
  statusLed.show();
  wirelessFragments = xQueueCreate(8, sizeof(WirelessFragment));
  setupDmxOutput();
  setupEspNow();
}

void loop() {
  WirelessFragment fragment;
  while (xQueueReceive(wirelessFragments, &fragment, 0) == pdTRUE) {
    processFragment(fragment);
  }
  if (millis() - lastOutput >= 25) {
    lastOutput = millis();
    outputDmxFrame();
  }
  updateStatusLed();
}