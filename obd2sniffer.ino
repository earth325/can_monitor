#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "driver/twai.h"
#include <math.h>
#include <string.h>

// Toyota Prius Gen 3 (2010-2015) generic OBD-II dashboard.
// Uses ESP32's built-in TWAI controller plus an external CAN transceiver.
// It actively polls standard OBD-II Mode 01 PIDs from the engine ECU.

// Keep the pin assignment from the existing setup.
static constexpr int CAN_RX_PIN = 4;  // transceiver RXD -> ESP32 RX
static constexpr int CAN_TX_PIN = 5;  // ESP32 TX -> transceiver TXD

static constexpr uint32_t OBD_RESPONSE_ID = 0x7E8;
static constexpr uint32_t HYBRID_RESPONSE_ID = 0x7EA;

const char *AP_SSID = "Prius-OBD-Dashboard";
const char *AP_PASSWORD = "PriusDash2015"; // 8+ characters; change if desired

AsyncWebServer server(80);
AsyncEventSource events("/events");

// Basic standard OBD-II readings. Values remain NaN until received.
float engineRpm = NAN;
float speedKmh = NAN;
float coolantC = NAN;
float engineLoadPct = NAN;
float intakeC = NAN;
float throttlePct = NAN;
float ecuVoltage = NAN;  // ECU supply voltage, NOT high-voltage traction battery voltage
float hybridSocPct = NAN; // hybrid battery SOC, when the hybrid ECU supports PID 01-5B
float hybridBatteryCurrentA = NAN; // Toyota-specific PID 21-98: positive = discharge, negative = charge
float batteryAmpsIn = NAN;          // current flowing INTO the HV battery (charging)
float batteryAmpsOut = NAN;         // current flowing OUT OF the HV battery (discharging)

// ISO-TP reassembly for Toyota hybrid ECU PID 21-98. The Prius Gen 3 response
// is multi-frame; a normal one-frame OBD parser will miss it.
static constexpr size_t HYBRID_ISOTP_BUFFER_SIZE = 32;
uint8_t hybridIsoTpBuffer[HYBRID_ISOTP_BUFFER_SIZE] = {};
size_t hybridIsoTpExpected = 0;
size_t hybridIsoTpReceived = 0;
uint8_t hybridIsoTpNextSequence = 1;
bool hybridIsoTpActive = false;
uint32_t hybridIsoTpStartedMs = 0;

uint32_t canRxCount = 0;
uint32_t obdRxCount = 0;
uint32_t lastObdResponseMs = 0;
uint32_t lastPublishMs = 0;
uint32_t lastRequestMs = 0;
size_t nextPidIndex = 0;
bool canReady = false;

// Poll OBD-II Mode 01 PIDs plus Prius Gen-3 Toyota Mode 21 PID 98 on 7E2/7EA.
struct ObdQuery { uint32_t requestId; uint32_t responseId; uint8_t mode; uint8_t pid; };
const ObdQuery QUERIES[] = {
  {0x7E0, 0x7E8, 0x01, 0x0C}, // engine RPM
  {0x7E0, 0x7E8, 0x01, 0x0D}, // vehicle speed
  {0x7E0, 0x7E8, 0x01, 0x05}, // coolant temperature
  {0x7E0, 0x7E8, 0x01, 0x04}, // calculated engine load
  {0x7E0, 0x7E8, 0x01, 0x0F}, // intake air temperature
  {0x7E0, 0x7E8, 0x01, 0x11}, // throttle position
  {0x7E0, 0x7E8, 0x01, 0x42}, // ECU supply voltage
  {0x7E2, 0x7EA, 0x01, 0x5B}, // hybrid battery state of charge (SOC)
  {0x7E2, 0x7EA, 0x21, 0x98}  // Toyota Gen-3 hybrid ECU: HV battery current + limits
};
const size_t QUERY_COUNT = sizeof(QUERIES) / sizeof(QUERIES[0]);
static constexpr uint32_t REQUEST_INTERVAL_MS = 100;
static constexpr uint32_t PUBLISH_INTERVAL_MS = 250;

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Prius OBD-II</title>
<style>
:root{color-scheme:dark;--bg:#0b1220;--card:#151f32;--muted:#a5b4cc;--text:#f2f6ff;--accent:#63e6be;}
*{box-sizing:border-box}body{margin:0;background:linear-gradient(160deg,#111d31,#080d17);color:var(--text);font-family:system-ui,-apple-system,Segoe UI,sans-serif;min-height:100vh}
main{max-width:920px;margin:auto;padding:24px 16px 36px}h1{font-size:clamp(1.7rem,4vw,2.5rem);margin:0 0 6px}.sub{color:var(--muted);margin:0 0 20px}
.status{padding:12px 14px;background:#102d2b;border:1px solid #235d52;border-radius:12px;margin-bottom:16px;color:#b9ffea;font-size:.92rem}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(190px,1fr));gap:12px}.card{background:var(--card);border:1px solid #2b3a52;border-radius:16px;padding:18px;min-height:126px;box-shadow:0 8px 24px #0002}
.label{font-size:.88rem;color:var(--muted);letter-spacing:.04em}.value{font-size:clamp(2rem,6vw,3rem);font-weight:750;letter-spacing:-.04em;margin-top:12px;line-height:1.1}.unit{font-size:.9rem;color:var(--muted);font-weight:500;letter-spacing:0}
.foot{color:var(--muted);font-size:.85rem;line-height:1.55;margin-top:18px}.good{color:var(--accent)}
</style>
</head>
<body><main>
<h1>2013 Toyota Prius</h1><p class="sub">ESP32 live OBD-II dashboard</p>
<div class="status" id="status">Connecting to ESP32 telemetry…</div>
<div class="grid">
 <section class="card"><div class="label">ENGINE SPEED</div><div class="value"><span id="rpm">--</span> <span class="unit">RPM</span></div></section>
 <section class="card"><div class="label">VEHICLE SPEED</div><div class="value"><span id="speed">--</span> <span class="unit">mph</span></div></section>
 <section class="card"><div class="label">COOLANT TEMPERATURE</div><div class="value"><span id="coolant">--</span><span class="unit"> °F</span></div></section>
 <section class="card"><div class="label">ENGINE LOAD</div><div class="value"><span id="load">--</span><span class="unit"> %</span></div></section>
 <section class="card"><div class="label">INTAKE AIR TEMPERATURE</div><div class="value"><span id="intake">--</span><span class="unit"> °F</span></div></section>
 <section class="card"><div class="label">THROTTLE POSITION</div><div class="value"><span id="throttle">--</span><span class="unit"> %</span></div></section>
 <section class="card"><div class="label">ECU SUPPLY VOLTAGE</div><div class="value"><span id="voltage">--</span><span class="unit"> V</span></div></section>
 <section class="card"><div class="label">HYBRID BATTERY SOC</div><div class="value"><span id="soc">--</span><span class="unit"> %</span></div></section>
 <section class="card"><div class="label">HV BATTERY CURRENT</div><div class="value"><span id="batteryCurrent">--</span><span class="unit"> A</span></div></section>
 <section class="card"><div class="label">BATTERY AMPS IN · CHARGING</div><div class="value"><span id="ampsIn">--</span><span class="unit"> A</span></div></section>
 <section class="card"><div class="label">BATTERY AMPS OUT · DISCHARGING</div><div class="value"><span id="ampsOut">--</span><span class="unit"> A</span></div></section>
</div>
<p class="foot">ECU supply voltage is the low-voltage electrical system reading, not the high-voltage traction battery. HV current uses Toyota-specific Mode 21 PID 98 from the hybrid ECU. On this Gen 3 Prius, negative signed current means charging (amps in), and positive signed current means discharging (amps out). The two directional gauges are split from the same signed current reading.</p>
</main>
<script>
const $ = id => document.getElementById(id);
const show = (id, v, digits=0) => { $(id).textContent = (v === null || !Number.isFinite(Number(v))) ? '--' : Number(v).toFixed(digits); };
const source = new EventSource('/events');
source.addEventListener('open', () => { $('status').textContent = 'Dashboard connected; waiting for OBD-II data…'; });
source.addEventListener('error', () => { $('status').textContent = 'Telemetry connection interrupted; reconnecting…'; });
source.addEventListener('telemetry', e => {
  const d = JSON.parse(e.data);
  show('rpm', d.rpm, 0);
  show('speed', d.speedKmh === null ? null : d.speedKmh * 0.621371, 0);
  show('coolant', d.coolantC === null ? null : d.coolantC * 9 / 5 + 32, 0);
  show('load', d.loadPct, 0);
  show('intake', d.intakeC === null ? null : d.intakeC * 9 / 5 + 32, 0);
  show('throttle', d.throttlePct, 0);
  show('voltage', d.ecuVoltage, 1);
  show('soc', d.socPct, 0);
  show('batteryCurrent', d.batteryCurrentA, 1);
  show('ampsIn', d.ampsIn, 1);
  show('ampsOut', d.ampsOut, 1);
  if (d.obdRx > 0) {
    const age = d.obdAgeMs < 0 ? 'unknown' : (d.obdAgeMs / 1000).toFixed(1) + ' s ago';
    $('status').innerHTML = '<span class="good">OBD-II replies received</span> · CAN frames: ' + d.canRx + ' · OBD replies: ' + d.obdRx + ' · Last reply: ' + age;
  } else if (d.canRx > 0) {
    $('status').textContent = 'CAN traffic received (' + d.canRx + ' frames), but no 0x7E8/0x7EA OBD-II replies yet. Check READY/ignition, CAN wiring, and transceiver TX.';
  } else {
    $('status').textContent = 'Waiting for CAN traffic. Check the OBD-II CAN connection and transceiver.';
  }
});
</script>
</body></html>
)rawliteral";

String jsonNumber(float value, unsigned int decimals = 1) {
  if (!isfinite(value)) return "null";
  return String(value, decimals);
}

void publishTelemetry() {
  String json;
  json.reserve(300);
  json  = "{\"rpm\":";        json += jsonNumber(engineRpm, 0);
  json += ",\"speedKmh\":";   json += jsonNumber(speedKmh, 1);
  json += ",\"coolantC\":";   json += jsonNumber(coolantC, 1);
  json += ",\"loadPct\":";    json += jsonNumber(engineLoadPct, 1);
  json += ",\"intakeC\":";    json += jsonNumber(intakeC, 1);
  json += ",\"throttlePct\":"; json += jsonNumber(throttlePct, 1);
  json += ",\"ecuVoltage\":"; json += jsonNumber(ecuVoltage, 2);
  json += ",\"socPct\":";     json += jsonNumber(hybridSocPct, 1);
  json += ",\"batteryCurrentA\":"; json += jsonNumber(hybridBatteryCurrentA, 2);
  json += ",\"ampsIn\":";      json += jsonNumber(batteryAmpsIn, 2);
  json += ",\"ampsOut\":";     json += jsonNumber(batteryAmpsOut, 2);
  json += ",\"canRx\":";      json += String(canRxCount);
  json += ",\"obdRx\":";      json += String(obdRxCount);
  json += ",\"obdAgeMs\":";
  json += ((lastObdResponseMs == 0) ? String(-1) : String(millis() - lastObdResponseMs));
  json += "}";
  events.send(json.c_str(), "telemetry", millis());
}

void sendObdRequest(const ObdQuery &query) {
  // Standard 11-bit CAN diagnostic request to the specified Prius ECU.
  twai_message_t request = {};
  request.identifier = query.requestId;
  request.data_length_code = 8;
  request.data[0] = 0x02;  // two payload bytes follow
  request.data[1] = query.mode; // 01 = standard OBD-II; 21 = Toyota-enhanced data
  request.data[2] = query.pid;
  // Remaining bytes are zero because request was initialized with {}.

  esp_err_t result = twai_transmit(&request, pdMS_TO_TICKS(10));
  if (result != ESP_OK) {
    Serial.printf("OBD request ECU 0x%03lX mode %02X PID %02X failed: %d\n", (unsigned long)query.requestId, query.mode, query.pid, (int)result);
  }
}

void sendHybridFlowControl() {
  // Allow the hybrid ECU to send the remaining ISO-TP response frames.
  twai_message_t flowControl = {};
  flowControl.identifier = 0x7E2; // tester-to-hybrid-ECU request ID
  flowControl.data_length_code = 8;
  flowControl.data[0] = 0x30; // ISO-TP Flow Control: Continue To Send
  flowControl.data[1] = 0x00; // block size 0 = no block limit
  flowControl.data[2] = 0x00; // minimum separation time 0 ms
  esp_err_t result = twai_transmit(&flowControl, pdMS_TO_TICKS(10));
  if (result != ESP_OK) {
    Serial.printf("Hybrid ISO-TP flow-control transmit failed: %d\n", (int)result);
  }
}

void decodeHybridBatteryCurrentPayload(const uint8_t *payload, size_t length) {
  // Reassembled payload is: 61 98 [8 data bytes].
  // Prius Gen 3 PID 21-98: first data word is unsigned big-endian,
  // scaled by 0.01 A and offset by -327.68 A.
  if (length < 10 || payload[0] != 0x61 || payload[1] != 0x98) return;

  const uint16_t raw = ((uint16_t)payload[2] << 8) | payload[3];
  hybridBatteryCurrentA = raw / 100.0f - 327.68f;
  // PriusCAN's documented sign convention: negative current = charging,
  // positive current = discharging.
  batteryAmpsIn = hybridBatteryCurrentA < 0.0f ? -hybridBatteryCurrentA : 0.0f;
  batteryAmpsOut = hybridBatteryCurrentA > 0.0f ? hybridBatteryCurrentA : 0.0f;

  ++obdRxCount;
  lastObdResponseMs = millis();
  Serial.printf("Hybrid battery current (PID 21-98): %.2f A | in/charge %.2f A | out/discharge %.2f A\n",
                hybridBatteryCurrentA, batteryAmpsIn, batteryAmpsOut);
}

bool handleHybridBatteryIsoTp(const twai_message_t &message) {
  if (message.identifier != HYBRID_RESPONSE_ID || message.rtr || message.data_length_code < 1) return false;

  const uint8_t frameType = message.data[0] >> 4;
  if (frameType == 0x1 && message.data_length_code >= 4) {
    // First Frame: only claim the transfer if the response is 61 98.
    if (message.data[2] != 0x61 || message.data[3] != 0x98) return false;
    const size_t totalLength = ((size_t)(message.data[0] & 0x0F) << 8) | message.data[1];
    if (totalLength < 10 || totalLength > HYBRID_ISOTP_BUFFER_SIZE) {
      Serial.printf("Unexpected PID 21-98 ISO-TP length: %u\n", (unsigned)totalLength);
      hybridIsoTpActive = false;
      return true;
    }

    hybridIsoTpExpected = totalLength;
    hybridIsoTpReceived = message.data_length_code - 2;
    if (hybridIsoTpReceived > hybridIsoTpExpected) hybridIsoTpReceived = hybridIsoTpExpected;
    memcpy(hybridIsoTpBuffer, message.data + 2, hybridIsoTpReceived);
    hybridIsoTpNextSequence = 1;
    hybridIsoTpActive = true;
    hybridIsoTpStartedMs = millis();
    sendHybridFlowControl();

    if (hybridIsoTpReceived >= hybridIsoTpExpected) {
      decodeHybridBatteryCurrentPayload(hybridIsoTpBuffer, hybridIsoTpReceived);
      hybridIsoTpActive = false;
    }
    return true;
  }

  if (frameType == 0x2 && hybridIsoTpActive) {
    const uint8_t sequence = message.data[0] & 0x0F;
    if (sequence != hybridIsoTpNextSequence) {
      Serial.printf("Hybrid PID 21-98 ISO-TP sequence mismatch: expected %u, got %u\n",
                    hybridIsoTpNextSequence, sequence);
      hybridIsoTpActive = false;
      return true;
    }
    size_t bytesToCopy = message.data_length_code > 1 ? message.data_length_code - 1 : 0;
    const size_t remaining = hybridIsoTpExpected - hybridIsoTpReceived;
    if (bytesToCopy > remaining) bytesToCopy = remaining;
    memcpy(hybridIsoTpBuffer + hybridIsoTpReceived, message.data + 1, bytesToCopy);
    hybridIsoTpReceived += bytesToCopy;
    hybridIsoTpNextSequence = (hybridIsoTpNextSequence + 1) & 0x0F;

    if (hybridIsoTpReceived >= hybridIsoTpExpected) {
      decodeHybridBatteryCurrentPayload(hybridIsoTpBuffer, hybridIsoTpReceived);
      hybridIsoTpActive = false;
    }
    return true;
  }
  return false;
}

void handleObdResponse(const twai_message_t &message) {
  if (message.data_length_code < 4) return;
  if (message.identifier != OBD_RESPONSE_ID && message.identifier != HYBRID_RESPONSE_ID) return;
  if (message.data[1] != 0x41) return; // standard Mode 01 positive response only

  const uint8_t pid = message.data[2];
  const uint8_t a = message.data[3];

  bool isExpectedReply = false;
  for (size_t i = 0; i < QUERY_COUNT; ++i) {
    if (QUERIES[i].responseId == message.identifier && QUERIES[i].mode == 0x01 && QUERIES[i].pid == pid) {
      isExpectedReply = true;
      break;
    }
  }
  if (!isExpectedReply) return;

  // Standard Mode 01 hybrid-battery SOC is queried from the Prius hybrid ECU.
  if (message.identifier == HYBRID_RESPONSE_ID && pid == 0x5B) {
    hybridSocPct = (float)a * 100.0f / 255.0f;
  } else if (message.identifier == OBD_RESPONSE_ID) {
    switch (pid) {
      case 0x0C: // Engine RPM = ((A * 256) + B) / 4
        if (message.data_length_code >= 5) {
          engineRpm = (((uint16_t)a << 8) | message.data[4]) / 4.0f;
        } else return;
        break;
      case 0x0D: // Vehicle speed, km/h
        speedKmh = a;
        break;
      case 0x05: // Engine coolant temperature, degrees C
        coolantC = (float)a - 40.0f;
        break;
      case 0x04: // Calculated engine load, percent
        engineLoadPct = (float)a * 100.0f / 255.0f;
        break;
      case 0x0F: // Intake air temperature, degrees C
        intakeC = (float)a - 40.0f;
        break;
      case 0x11: // Throttle position, percent
        throttlePct = (float)a * 100.0f / 255.0f;
        break;
      case 0x42: // Control module voltage = ((A * 256) + B) / 1000 V
        if (message.data_length_code >= 5) {
          ecuVoltage = (((uint16_t)a << 8) | message.data[4]) / 1000.0f;
        } else return;
        break;
      default:
        return; // ignore unrelated Mode 01 PIDs
    }
  } else {
    return;
  }

  ++obdRxCount;
  lastObdResponseMs = millis();
  Serial.printf("OBD response ECU 0x%03lX PID 01-%02X: ", (unsigned long)message.identifier, pid);
  for (int i = 0; i < message.data_length_code; ++i) {
    Serial.printf("%02X ", message.data[i]);
  }
  Serial.println();
}

bool startCan() {
  twai_general_config_t generalConfig = TWAI_GENERAL_CONFIG_DEFAULT(
      (gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
  generalConfig.rx_queue_len = 64;
  generalConfig.tx_queue_len = 10;

  twai_timing_config_t timingConfig = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t filterConfig = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t result = twai_driver_install(&generalConfig, &timingConfig, &filterConfig);
  if (result != ESP_OK) {
    Serial.printf("TWAI driver install failed: %d\n", (int)result);
    return false;
  }
  result = twai_start();
  if (result != ESP_OK) {
    Serial.printf("TWAI start failed: %d\n", (int)result);
    return false;
  }
  Serial.println("TWAI started at 500 kbps.");
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n2013 Prius OBD-II dashboard starting...");

  canReady = startCan();

  WiFi.mode(WIFI_AP);
  bool apStarted = WiFi.softAP(AP_SSID, AP_PASSWORD);
  if (apStarted) {
    Serial.print("Join Wi-Fi: "); Serial.println(AP_SSID);
    Serial.print("Password: "); Serial.println(AP_PASSWORD);
    Serial.print("Open dashboard: http://"); Serial.println(WiFi.softAPIP());
  } else {
    Serial.println("Wi-Fi access point failed to start.");
  }

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", INDEX_HTML);
  });
  events.onConnect([](AsyncEventSourceClient *client) {
    client->send("{}", "telemetry", millis(), 1000);
  });
  server.addHandler(&events);
  server.begin();
  Serial.println("Web server started.");

  if (!canReady) {
    Serial.println("CAN is not running; check Serial output and ESP32 board support.");
  }
}

void loop() {
  if (!canReady) {
    delay(100);
    return;
  }

  // Drain a bounded batch each pass so heavy broadcast traffic does not block polling.
  for (int i = 0; i < 40; ++i) {
    twai_message_t message = {};
    if (twai_receive(&message, 0) != ESP_OK) break;
    ++canRxCount;
    if (!handleHybridBatteryIsoTp(message)) handleObdResponse(message);
  }

  const uint32_t now = millis();
  if (hybridIsoTpActive && now - hybridIsoTpStartedMs > 100) {
    Serial.println("Hybrid PID 21-98 ISO-TP response timed out; will retry on next poll.");
    hybridIsoTpActive = false;
  }
  if (!hybridIsoTpActive && now - lastRequestMs >= REQUEST_INTERVAL_MS) {
    lastRequestMs = now;
    sendObdRequest(QUERIES[nextPidIndex]);
    nextPidIndex = (nextPidIndex + 1) % QUERY_COUNT;
  }

  if (now - lastPublishMs >= PUBLISH_INTERVAL_MS) {
    lastPublishMs = now;
    publishTelemetry();
  }
}
