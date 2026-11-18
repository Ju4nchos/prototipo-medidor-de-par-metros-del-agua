/*
 * ============================================================================
 *  SISTEMA IoT DE MONITOREO DE LA CALIDAD DEL AGUA - Nodo sensor ESP32
 *  Proyecto Integrador IoT - Universidad Cooperativa de Colombia, Campus Neiva
 * ============================================================================
 *  Hardware (según el documento del proyecto):
 *    - ESP32 DevKit V1
 *    - Sensor de pH analógico PH-4502C (salida Po, alimentado a 5 V)
 *    - Sensor de turbidez analógico (salida AO, alimentado a 5 V)
 *    - Sensor de conductividad/TDS analógico (salida A, alimentado a 3.3 V)
 *    - Sonda DS18B20 impermeable (1-Wire)
 *    - Alerta local: LED + buzzer
 *    - Resistencias: 2x10k + 2x15k (divisores pH/turbidez), 4.7k (pull-up
 *      DS18B20), 220 ohm (LED), 100 ohm (buzzer)
 *
 *  Conexiones (todas las entradas analógicas usan ADC1, porque ADC2 queda
 *  bloqueado mientras el Wi-Fi está activo):
 *    GPIO34 (D34) <- pH Po        a través del divisor 10k / 15k
 *    GPIO35 (D35) <- Turbidez AO  a través del divisor 10k / 15k
 *    GPIO32 (D32) <- TDS A        directo (salida máx. 2.3 V)
 *    GPIO4  (D4)  <-> DS18B20 DQ  con pull-up 4.7k a 3.3 V
 *    GPIO23 (D23) -> LED de alerta    a través de 220 ohm
 *    GPIO19 (D19) -> Buzzer de alerta a través de 100 ohm
 *  Las señales analógicas van por el lado izquierdo de la placa y las digitales
 *  (LED, buzzer, 1-Wire) por el derecho: el cableado de señal queda separado de
 *  los elementos que generan interferencia (mitigación de ruido del documento).
 *
 *  Arquitectura del firmware (dos tareas en paralelo):
 *    - loop() en el núcleo 1: lee sensores cada 1 s, convierte a unidades,
 *      evalúa rangos (Res. 2115/2007) y maneja LED/buzzer. NUNCA espera a la
 *      red, así la alerta local responde en <= 3 s (RF-05) aunque no haya Wi-Fi.
 *    - tareaRed() en el núcleo 0: mantiene el Wi-Fi (reconexión automática,
 *      RNF-01) y envía por HTTP POST a Supabase las lecturas que el loop deja
 *      en una cola. Si no hay red, la cola actúa como buffer de las últimas
 *      N lecturas y se reenvían al reconectar (plan de mitigación de riesgos).
 * ============================================================================
 */

#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <time.h>

// ----------------------------------------------------------------------------
// 1) MODO DE OPERACIÓN
// ----------------------------------------------------------------------------
// 1 = simulación en Wokwi (los potenciómetros emulan pH, turbidez y TDS)
// 0 = ensamble real
#define MODO_SIMULACION 0

// Tipo de buzzer: 0 = pasivo (necesita una frecuencia, es el de Wokwi)
//                 1 = activo  (suena solo con nivel alto)
#define BUZZER_ACTIVO 0

// ----------------------------------------------------------------------------
// 2) CREDENCIALES (cámbialas para el ensamble real)
// ----------------------------------------------------------------------------
#if MODO_SIMULACION
const char* WIFI_SSID  = "Wokwi-GUEST";   // punto de acceso virtual de Wokwi
const char* WIFI_PASS  = "";
const int   WIFI_CANAL = 6;               // evita el escaneo (conecta ~4 s antes)
#else
const char* WIFI_SSID  = "S25 de Luis";
const char* WIFI_PASS  = "123456788";
const int   WIFI_CANAL = 0;               // 0 = escanear todos los canales
#endif

// Supabase: URL del proyecto y API key "anon" (Settings > API).
// Mientras la URL contenga "TU-PROYECTO" el envío se SIMULA (solo se imprime
// el JSON en el monitor serie), así la simulación funciona sin cuenta.
const char* SUPABASE_URL   = "https://iletadtfsljzucuqythk.supabase.co";
const char* SUPABASE_KEY   = "sb_publishable_QfBKUR5zwDcx6NS24tegkQ_v-tIdZF6";
const char* SUPABASE_TABLA = "lecturas";
const char* UBICACION      = "Tanque 1";

// ----------------------------------------------------------------------------
// 3) PINES
// ----------------------------------------------------------------------------
const uint8_t PIN_PH      = 34;  // ADC1_CH6 (solo entrada)
const uint8_t PIN_TURB    = 35;  // ADC1_CH7 (solo entrada)
const uint8_t PIN_TDS     = 32;  // ADC1_CH4
const uint8_t PIN_ONEWIRE = 4;   // DS18B20
const uint8_t PIN_LED     = 23;  // LED de alerta
const uint8_t PIN_BUZZER  = 19;  // Buzzer de alerta

// ----------------------------------------------------------------------------
// 4) CONSTANTES ELÉCTRICAS
// ----------------------------------------------------------------------------
// Divisor de voltaje para pH y turbidez (módulos alimentados a 5 V):
//   sensor --[R_SUP 10k]--+--[R_INF 15k]-- GND
//                         +--> pin ADC
//   V_adc = V_sensor * R_INF / (R_SUP + R_INF) = 0.6 * V_sensor
//   => 5.0 V del sensor se convierten en 3.0 V en el pin (máx. ESP32: 3.3 V)
const float R_SUP          = 10000.0f;
const float R_INF          = 15000.0f;
const float FACTOR_DIVISOR = (R_SUP + R_INF) / R_INF;   // 1.6667

const float VREF_ADC = 3.3f;     // escala de analogRead() a 11 dB (simulación)
const int   ADC_MAX  = 4095;     // ADC de 12 bits
const int   MUESTRAS = 15;       // muestras por lectura (filtro de mediana)

// ----------------------------------------------------------------------------
// 5) CALIBRACIÓN (ajustar en el ensamble real con soluciones buffer)
// ----------------------------------------------------------------------------
// pH-4502C: voltaje de Po (lado del sensor, ANTES del divisor) en buffer pH 7
// y pH 4. Típicamente ~2.50 V y ~3.04 V (≈ -0.18 V por unidad de pH).
float CAL_PH7_V = 2.50f;
float CAL_PH4_V = 3.04f;

// Turbidez: voltaje AO (lado del sensor) medido con agua destilada/limpia.
// La curva de referencia asume 4.20 V para agua clara; la diferencia se usa
// como corrección de offset.
float TURB_V_AGUA_LIMPIA = 2.37f;   // <- poner el V_turb medido con el sensor en agua limpia

// Estabilizacion de la turbidez (la curva del sensor es muy empinada: 30 mV ~ 100 NTU)
const float TURB_ZONA_MUERTA_V = 0.03f;  // variacion de hasta 30 mV bajo el agua limpia = 0 NTU
const float TURB_ALFA          = 0.1f;   // suavizado: mas bajo = mas estable pero mas lento

// TDS: factor K de calibración con solución patrón (1.0 = sin corrección).
float TDS_K = 1.0f;

// ----------------------------------------------------------------------------
// 6) UMBRALES DE ALERTA
// ----------------------------------------------------------------------------
// Resolución 2115 de 2007 (Colombia):
const float PH_MIN    = 6.0f;     // Art. 4: pH entre 6.5 y 9.0
const float PH_MAX    = 9.0f;
const float TURB_MAX  = 2.0f;     // Art. 2: turbiedad máx. 2 UNT (NTU)
// Art. 3: conductividad máx. 1000 uS/cm -> TDS ≈ 0.5 * EC = 500 ppm
const float TDS_MAX   = 500.0f;
// La Res. 2115 NO fija temperatura: rango operativo definido por el proyecto
const float TEMP_MIN  = 10.0f;
const float TEMP_MAX  = 33.0f;

// ----------------------------------------------------------------------------
// 7) TIEMPOS
// ----------------------------------------------------------------------------
const uint32_t INTERVALO_LECTURA_MS = 1000;   // evaluación de alertas (RF-05)
const uint32_t INTERVALO_ENVIO_MS   = 30000;  // envío a la nube (RF-01: 30-60 s)
const uint32_t REINTENTO_WIFI_MS    = 10000;  // reconexión (RNF-01: <= 30 s)
const uint32_t TIMEOUT_HTTP_MS      = 8000;

// ----------------------------------------------------------------------------
// ESTRUCTURAS Y VARIABLES GLOBALES
// ----------------------------------------------------------------------------
struct Lectura {
  float  ph;
  float  turbidez;
  float  tds;
  float  temperatura;
  bool   alerta;
  bool   errorSensor;
  time_t epoch;          // 0 si aún no hay hora NTP
};

const int     TAM_BUFFER = 20;   // últimas N lecturas guardadas sin red
QueueHandle_t colaEnvio;

OneWire           oneWire(PIN_ONEWIRE);
DallasTemperature ds18b20(&oneWire);

Lectura  ultima;
uint32_t tUltimaLectura = 0;
uint32_t tUltimoEnvio   = 0;
uint32_t tUltimaImpresion = 0;
bool     alertaActiva   = false;
float    vPh = 0, vTurb = 0, vTds = 0;   // voltajes del lado del sensor
float    vTurbSuave = -1;                // voltaje de turbidez suavizado (-1 = sin iniciar)

// ============================================================================
//  LECTURA ANALÓGICA FILTRADA
// ============================================================================
// Toma MUESTRAS lecturas y devuelve la mediana en voltios en el PIN del ESP32.
// La mediana elimina picos de ruido mejor que el promedio (riesgo "ruido
// eléctrico en lecturas analógicas" del documento).
float leerVoltajePin(uint8_t pin) {
  float v[MUESTRAS];
  for (int i = 0; i < MUESTRAS; i++) {
#if MODO_SIMULACION
    v[i] = analogRead(pin) * VREF_ADC / ADC_MAX;      // Wokwi: ADC lineal
#else
    v[i] = analogReadMilliVolts(pin) / 1000.0f;       // real: usa calibración eFuse
#endif
    delayMicroseconds(200);
  }
  // ordenamiento por inserción (15 elementos: suficiente y simple)
  for (int i = 1; i < MUESTRAS; i++) {
    float x = v[i];
    int j = i - 1;
    while (j >= 0 && v[j] > x) { v[j + 1] = v[j]; j--; }
    v[j + 1] = x;
  }
  return v[MUESTRAS / 2];
}

// ============================================================================
//  CONVERSIÓN A UNIDADES REALES
// ============================================================================
float calcularPH(float vSensor) {
  // Recta por dos puntos (pH 7 y pH 4). Pendiente en pH/V (negativa).
  float pendiente = (7.0f - 4.0f) / (CAL_PH7_V - CAL_PH4_V);
  float ph = 7.0f + pendiente * (vSensor - CAL_PH7_V);
  return constrain(ph, 0.0f, 14.0f);
}

float calcularTurbidez(float vSensor) {
  // Curva característica del sensor (fabricante, alimentación 5 V):
  //   NTU = -1120.4 V^2 + 5742.3 V - 4352.9   válida entre 2.5 V y 4.2 V
  float v = vSensor + (4.20f - TURB_V_AGUA_LIMPIA);   // corrección de offset
  if (v > 4.20f - TURB_ZONA_MUERTA_V) return 0.0f;      // zona muerta: igual al agua limpia
  if (v < 2.5f) return 3000.0f;                        // límite de la curva
  float ntu = -1120.4f * v * v + 5742.3f * v - 4352.9f;
  if (ntu < 0.0f) ntu = 0.0f;                          // agua clara
  return ntu;
}

float calcularTDS(float vSensor, float tempC) {
  // Compensación por temperatura a 25 °C (2 % por °C)
  float t = (tempC > -50.0f && tempC < 100.0f) ? tempC : 25.0f;
  float coef = 1.0f + 0.02f * (t - 25.0f);
  float vc = vSensor / coef;
  // Curva del fabricante del módulo TDS -> ppm
  float tds = (133.42f * vc * vc * vc - 255.86f * vc * vc + 857.39f * vc) * 0.5f;
  tds *= TDS_K;
  if (tds < 0.0f) tds = 0.0f;
  return tds;
}

// ============================================================================
//  ALERTA LOCAL (LED + BUZZER), NO BLOQUEANTE
// ============================================================================
void buzzerOn() {
#if BUZZER_ACTIVO
  digitalWrite(PIN_BUZZER, HIGH);
#else
  tone(PIN_BUZZER, 2000);        // 2 kHz: zona de máxima sensibilidad del oído
#endif
}

void buzzerOff() {
#if BUZZER_ACTIVO
  digitalWrite(PIN_BUZZER, LOW);
#else
  noTone(PIN_BUZZER);
#endif
}

void actualizarActuadores() {
  static bool     buzzerSonando = false;
  static uint32_t tCambio = 0;
  uint32_t ahora = millis();

  if (!alertaActiva) {
    digitalWrite(PIN_LED, LOW);
    if (buzzerSonando) { buzzerOff(); buzzerSonando = false; }
    return;
  }

  // LED: fijo si es parámetro fuera de rango; parpadeo rápido si falla un sensor
  if (ultima.errorSensor) digitalWrite(PIN_LED, (ahora / 150) % 2);
  else                    digitalWrite(PIN_LED, HIGH);

  // Buzzer intermitente: 200 ms sonando / 800 ms en silencio
  uint32_t periodo = buzzerSonando ? 200 : 800;
  if (ahora - tCambio >= periodo) {
    tCambio = ahora;
    buzzerSonando = !buzzerSonando;
    if (buzzerSonando) buzzerOn(); else buzzerOff();
  }
}

// ============================================================================
//  ADQUISICIÓN Y EVALUACIÓN
// ============================================================================
void leerSensores() {
  // DS18B20: se lee la conversión pedida en el ciclo anterior (hace 1 s,
  // la conversión de 12 bits tarda 750 ms) y se pide la siguiente. Así nunca
  // se bloquea el programa esperando al sensor.
  float t = ds18b20.getTempCByIndex(0);
  ds18b20.requestTemperatures();

  vPh   = leerVoltajePin(PIN_PH)   * FACTOR_DIVISOR;   // deshace el divisor
  vTurb = leerVoltajePin(PIN_TURB) * FACTOR_DIVISOR;   // deshace el divisor
  vTurbSuave = (vTurbSuave < 0) ? vTurb : vTurbSuave + TURB_ALFA * (vTurb - vTurbSuave);
  vTds  = leerVoltajePin(PIN_TDS);                     // sin divisor

  bool tempOk = (t != DEVICE_DISCONNECTED_C) && (t > -55.0f) && (t < 125.0f);

  ultima.temperatura = tempOk ? t : NAN;
  ultima.ph          = calcularPH(vPh);
  ultima.turbidez    = calcularTurbidez(vTurbSuave);
  ultima.tds         = calcularTDS(vTds, tempOk ? t : 25.0f);
  ultima.errorSensor = !tempOk;

  bool fueraRango =
      ultima.ph < PH_MIN || ultima.ph > PH_MAX ||
      ultima.turbidez > TURB_MAX ||
      ultima.tds > TDS_MAX ||
      (tempOk && (t < TEMP_MIN || t > TEMP_MAX));

  ultima.alerta = fueraRango || ultima.errorSensor;
  time_t ahora; time(&ahora);
  ultima.epoch = (ahora > 1700000000) ? ahora : 0;   // solo si NTP ya sincronizó
  alertaActiva = ultima.alerta;
}

const char* textoEstado(const Lectura& l) {
  if (l.errorSensor) return "Error sensor";
  return l.alerta ? "Revisar" : "Normal";
}

void imprimirLectura() {
  Serial.printf("[%6lu s] pH=%5.2f | Turb=%7.1f NTU | TDS=%6.0f ppm | T=",
                millis() / 1000, ultima.ph, ultima.turbidez, ultima.tds);
  if (isnan(ultima.temperatura)) Serial.print("  ERR ");
  else Serial.printf("%5.1f", ultima.temperatura);
  Serial.printf(" C | %s", textoEstado(ultima));
  if (ultima.alerta) {
    Serial.print("  <-");
    if (ultima.ph < PH_MIN || ultima.ph > PH_MAX) Serial.print(" pH");
    if (ultima.turbidez > TURB_MAX)               Serial.print(" TURBIDEZ");
    if (ultima.tds > TDS_MAX)                     Serial.print(" TDS");
    if (!isnan(ultima.temperatura) &&
        (ultima.temperatura < TEMP_MIN || ultima.temperatura > TEMP_MAX))
                                                  Serial.print(" TEMPERATURA");
    if (ultima.errorSensor)                       Serial.print(" DS18B20");
  }
  Serial.println();
  // Voltajes del lado del sensor: útiles para calibrar el ensamble real
  Serial.printf("           V_pH=%.3f V  V_turb=%.3f V  V_tds=%.3f V  WiFi=%s\n",
                vPh, vTurb, vTds,
                WiFi.status() == WL_CONNECTED ? "OK" : "sin conexion");
}

// Encola la lectura para envío. Si el buffer está lleno (red caída mucho
// tiempo) se descarta la lectura más antigua para conservar las recientes.
void encolarLectura(const Lectura& l) {
  if (xQueueSend(colaEnvio, &l, 0) != pdTRUE) {
    Lectura descartada;
    xQueueReceive(colaEnvio, &descartada, 0);
    xQueueSend(colaEnvio, &l, 0);
    Serial.println("[BUFFER] Lleno: se descartó la lectura más antigua");
  }
}

// ============================================================================
//  RED: Wi-Fi + HTTP POST a Supabase (núcleo 0)
// ============================================================================
String construirJSON(const Lectura& l) {
  String j = "{";
  j += "\"ubicacion\":\"" + String(UBICACION) + "\",";
  if (l.epoch > 0) {
    char fecha[32];
    struct tm tmLocal;
    localtime_r(&l.epoch, &tmLocal);
    strftime(fecha, sizeof(fecha), "%Y-%m-%dT%H:%M:%S-05:00", &tmLocal);
    j += "\"fecha_hora\":\"" + String(fecha) + "\",";
  }
  j += "\"ph\":" + String(l.ph, 2) + ",";
  j += "\"turbidez\":" + String(l.turbidez, 1) + ",";
  j += "\"tds\":" + String(l.tds, 0) + ",";
  j += "\"temperatura\":" + (isnan(l.temperatura) ? String("null") : String(l.temperatura, 1)) + ",";
  j += "\"estado\":\"" + String(textoEstado(l)) + "\",";
  j += "\"alerta\":" + String(l.alerta ? "true" : "false");
  j += "}";
  return j;
}

bool enviarSupabase(const Lectura& l) {
  String json = construirJSON(l);

  if (strstr(SUPABASE_URL, "TU-PROYECTO") != nullptr) {
    Serial.println("[HTTP] (envío simulado) POST " + json);
    return true;
  }

  WiFiClientSecure cliente;
  cliente.setInsecure();   // prototipo: en producción cargar el certificado raíz
  HTTPClient http;
  String url = String(SUPABASE_URL) + "/rest/v1/" + SUPABASE_TABLA;
  if (!http.begin(cliente, url)) return false;
  http.setTimeout(TIMEOUT_HTTP_MS);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("apikey", SUPABASE_KEY);
  http.addHeader("Prefer", "return=minimal");
  int codigo = http.POST(json);
  http.end();

  Serial.printf("[HTTP] POST -> %d %s\n", codigo,
                (codigo >= 200 && codigo < 300) ? "OK" : "ERROR");
  return codigo >= 200 && codigo < 300;
}

void conectarWiFi() {
  Serial.printf("[WiFi] Conectando a \"%s\"...\n", WIFI_SSID);
  WiFi.disconnect();
  if (WIFI_CANAL > 0) WiFi.begin(WIFI_SSID, WIFI_PASS, WIFI_CANAL);
  else                WiFi.begin(WIFI_SSID, WIFI_PASS);
}

void tareaRed(void* parametro) {
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  conectarWiFi();
  uint32_t tUltimoIntento = millis();
  bool estabaConectado = false;
  bool horaConfigurada = false;

  for (;;) {
    bool conectado = (WiFi.status() == WL_CONNECTED);

    if (conectado && !estabaConectado) {
      Serial.printf("[WiFi] Conectado. IP: %s\n", WiFi.localIP().toString().c_str());
      if (!horaConfigurada) {
        configTime(-5 * 3600, 0, "pool.ntp.org", "time.google.com"); // Colombia UTC-5
        horaConfigurada = true;
      }
    }
    if (!conectado && estabaConectado) {
      Serial.println("[WiFi] Conexión perdida. Las lecturas quedan en el buffer.");
      tUltimoIntento = millis();
    }
    estabaConectado = conectado;

    if (!conectado) {
      if (millis() - tUltimoIntento >= REINTENTO_WIFI_MS) {
        tUltimoIntento = millis();
        conectarWiFi();
      }
    } else {
      // Envía lo pendiente en orden. Solo se retira de la cola si el POST
      // fue exitoso (2xx): así ninguna lectura se pierde por un fallo puntual.
      Lectura l;
      while (WiFi.status() == WL_CONNECTED && xQueuePeek(colaEnvio, &l, 0) == pdTRUE) {
        if (enviarSupabase(l)) {
          xQueueReceive(colaEnvio, &l, 0);
        } else {
          vTaskDelay(pdMS_TO_TICKS(3000));   // reintento más tarde
          break;
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(250));
  }
}

// ============================================================================
//  SETUP Y LOOP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("=== Monitor de calidad del agua - ESP32 ===");
#if MODO_SIMULACION
  Serial.println("Modo: SIMULACION (Wokwi). Gire los potenciometros pH / Turbidez / TDS");
  Serial.println("y haga clic en el DS18B20 para cambiar la temperatura.");
#else
  Serial.println("Modo: ENSAMBLE REAL");
#endif

  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  buzzerOff();

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);   // rango de entrada hasta ~3.1-3.3 V

  ds18b20.begin();
  ds18b20.setResolution(12);
  ds18b20.setWaitForConversion(false);   // lectura no bloqueante
  ds18b20.requestTemperatures();
  delay(800);                            // primera conversión

  // Autoprueba de la alerta local: LED y buzzer 300 ms
  digitalWrite(PIN_LED, HIGH); buzzerOn(); delay(300);
  digitalWrite(PIN_LED, LOW);  buzzerOff();

  colaEnvio = xQueueCreate(TAM_BUFFER, sizeof(Lectura));
  xTaskCreatePinnedToCore(tareaRed, "tareaRed", 8192, nullptr, 1, nullptr, 0);

  tUltimoEnvio = millis() - INTERVALO_ENVIO_MS + 5000;   // primer envío a los ~5 s
}

void loop() {
  uint32_t ahora = millis();

  if (ahora - tUltimaLectura >= INTERVALO_LECTURA_MS) {
    tUltimaLectura = ahora;
    leerSensores();
    if (ahora - tUltimaImpresion >= 2000) {
      tUltimaImpresion = ahora;
      imprimirLectura();
    }
  }

  if (ahora - tUltimoEnvio >= INTERVALO_ENVIO_MS) {
    tUltimoEnvio = ahora;
    encolarLectura(ultima);
  }

  actualizarActuadores();
  delay(10);
}
