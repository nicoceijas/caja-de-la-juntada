// sketch.ino — Caja de la juntada · Sprint 2 · firmware v0.6.4 en UN SOLO ARCHIVO
//   v0.6.4 (24/11): S1.7 falla de sensor visible en todos los estados + WiFi no bloqueante
//                 + Telegram desde el ESP32 (tarea aparte con cola y reintentos)
//                 + caída del WiFi simulable con el pulsador "Caida WiFi" (GPIO 26).
// (versión para el plan gratuito de Wokwi, que no permite subir archivos).
// Es el mismo código que la versión en módulos (COD-S1-002), pegado en este orden:
//   0. Tipos  1. config.h  2. Declaraciones (.h)  3. sensores  4. actuadores
//   5. estados  6. red  7. prueba_circuito  8. setup() y loop()
// Librerías (Library Manager): Adafruit SSD1306, Adafruit GFX Library, Adafruit NeoPixel, ESP32Servo
// WiFi, WiFiClientSecure y HTTPClient vienen con el core del ESP32 (no se agregan).

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_NeoPixel.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>

// =================================================================
// 0. Tipos (van primero para que el IDE pueda generar los prototipos)
// =================================================================
enum class EstadoCasillero : uint8_t { LIBRE, OCUPADO, FALLA };
enum class EstadoCaja : uint8_t { ESPERA, LISTA, JUNTADA, ALERTA, FIN };
enum class EstadoBandera : uint8_t { BAJA, SUBIENDO, CONFIRMADA, FALLA };

// =================================================================
// 1. Configuración  (config.h)
// =================================================================
// config.h — Caja de la juntada
// Todos los parámetros ajustables del firmware en un solo lugar.
// Pines según DOC-01 / DOC-05; umbrales según DEC-015 (PRB-S0-001).

// ---------------------------------------------------------------
// Modo de trabajo
// 1 = prueba de circuito (Sprint 1, pasos 1 a 3): muestra lecturas y prueba
//     cada componente. 0 = firmware normal (máquina de estados, desde S1.4).
// ---------------------------------------------------------------
#define MODO_PRUEBA_CIRCUITO 0

// Versión: se imprime al arrancar para saber qué código está cargado en Wokwi.
#define FIRMWARE_VERSION "v0.6.4 (S1.7 + WiFi + Telegram)"

// ---------------------------------------------------------------
// Casilleros y reglas de la juntada
// ---------------------------------------------------------------
constexpr uint8_t  CANT_CASILLEROS = 4;
constexpr bool     CASILLERO_ASIGNADO[CANT_CASILLEROS] = { true, true, true, true };
constexpr uint32_t DURACION_MINIMA_MS = 30UL * 60UL * 1000UL;   // 30 minutos (se informa en el resumen)
constexpr uint8_t  MIN_PARTICIPANTES  = 2;      // celulares mínimos para poder iniciar
constexpr uint8_t  MAX_RETIROS        = 20;     // retiros que se guardan por juntada
// Nombre de cada casillero en los mensajes de Telegram.
// Se puede poner el nombre de cada uno, p. ej. { "Jugador 1", "Jugador 2", "Jugador 3", "Jugador 4" }.
constexpr const char* NOMBRE_CASILLERO[CANT_CASILLEROS] = { "C1", "C2", "C3", "C4" };

// ---------------------------------------------------------------
// Máquina de estados
// ---------------------------------------------------------------
constexpr uint32_t ALERTA_SONIDO_MS     = 10000;  // la alerta suena hasta 10 s; después queda solo la luz
constexpr uint32_t MENSAJE_TEMPORAL_MS  = 2500;   // avisos cortos en el OLED
constexpr uint32_t PARPADEO_MS          = 400;    // medio período del parpadeo de los LED

// ---------------------------------------------------------------
// Pines (placa board-esp32-devkit-c-v4 en Wokwi)
// ---------------------------------------------------------------
constexpr uint8_t PIN_LDR[CANT_CASILLEROS]  = { 32, 33, 34, 35 };  // ADC1, pull-down 1 MΩ
constexpr uint8_t PIN_OLED_SDA     = 21;
constexpr uint8_t PIN_OLED_SCL     = 22;
constexpr uint8_t PIN_NEOPIXEL     = 16;
constexpr uint8_t PIN_SERVO        = 18;
constexpr uint8_t PIN_BUZZER       = 19;
constexpr uint8_t PIN_BOTON_INICIO = 4;    // INPUT_PULLUP
constexpr uint8_t PIN_FIN_CARRERA  = 23;   // INPUT_PULLUP; LOW = bandera arriba (confirmada)
constexpr uint8_t PIN_BOTON_WIFI   = 26;   // INPUT_PULLUP; simula la caída del WiFi (uso digital: el conflicto ADC2/WiFi es solo analógico)

// ---------------------------------------------------------------
// Sensores LDR (DEC-015). Lectura ALTA = casillero tapado = ocupado.
// ---------------------------------------------------------------
constexpr uint8_t  LDR_MUESTRAS           = 8;     // promedio móvil de 8 muestras
constexpr uint32_t LDR_PERIODO_MUESTRA_MS = 20;    // 8 x 20 ms = ventana de 160 ms
constexpr int      UMBRAL_OCUPADO         = 3700;  // por encima: ocupado
constexpr int      UMBRAL_LIBRE           = 3200;  // por debajo: libre (entre ambos: histéresis)
constexpr int      UMBRAL_INVALIDO        = 10;    // por debajo: lectura inválida
constexpr uint32_t CONFIRMACION_MS        = 1000;  // cambio OCUPADO/LIBRE sostenido
constexpr uint32_t FALLA_SENSOR_MS        = 2000;  // inválido sostenido → FALLA

// ---------------------------------------------------------------
// Pulsadores
// ---------------------------------------------------------------
constexpr uint32_t ANTIRREBOTE_MS     = 30;
constexpr uint32_t PULSACION_LARGA_MS = 2000;      // fin de la juntada

// ---------------------------------------------------------------
// Bandera (servo + fin de carrera) — DEC-020 (18/11): reemplaza a la traba de la tapa.
// El servo levanta una bandera al iniciar la juntada: es una señal visible de que el
// "juego" empezó, no un impedimento. No hay tapa ni traba.
// ---------------------------------------------------------------
constexpr int      SERVO_ANGULO_BAJA    = 90;       // bandera acostada
constexpr int      SERVO_ANGULO_ALZADA  = 0;      // bandera vertical
constexpr uint32_t BANDERA_TIMEOUT_MS   = 2000;    // sin confirmación → falla de actuador
constexpr uint8_t  PIXEL_FALLA_BANDERA  = 7;       // píxel de la tira que se pone magenta si la bandera falla

// ---------------------------------------------------------------
// NeoPixel: 0–3 un píxel por casillero, 4–7 luz ambiente
// ---------------------------------------------------------------
constexpr uint8_t NEOPIXEL_CANT           = 8;
constexpr uint8_t NEOPIXEL_PRIMER_AMBIENTE = 4;
constexpr uint8_t NEOPIXEL_BRILLO         = 60;    // 0–255

// ---------------------------------------------------------------
// OLED SSD1306
// ---------------------------------------------------------------
constexpr uint8_t  OLED_ANCHO       = 128;
constexpr uint8_t  OLED_ALTO        = 64;
constexpr uint8_t  OLED_DIRECCION   = 0x3C;
constexpr uint32_t OLED_REFRESCO_MS = 200;

// ---------------------------------------------------------------
// Serial
// ---------------------------------------------------------------
constexpr uint32_t SERIAL_BAUDIOS = 115200;

// ---------------------------------------------------------------
// Red: WiFi y Telegram (Sprint 2; DEC-017: Telegram directo desde el ESP32)
// ---------------------------------------------------------------
#define WIFI_SSID "Wokwi-GUEST"
#define WIFI_PASS ""
constexpr int      WIFI_CANAL        = 6;        // canal fijo: Wokwi conecta más rápido
constexpr uint32_t WIFI_REINTENTO_MS = 15000;    // sin conexión: nuevo intento cada 15 s (sin bloquear)

// ATENCIÓN: completar SOLO en tu copia de Wokwi. No subir ni compartir el proyecto con estos
// valores (el gateway público de Wokwi monitorea el tráfico). Con los valores de ejemplo,
// los mensajes no se envían: se muestran por Serial para poder probar sin token.
#define TELEGRAM_TOKEN   "TU_TOKEN"
#define TELEGRAM_CHAT_ID "TU_CHAT_ID"

constexpr uint8_t  TELEGRAM_COLA              = 12;    // mensajes que esperan envío (RAM)
constexpr uint16_t TELEGRAM_TEXTO_MAX         = 600;   // largo máximo de un mensaje
constexpr uint8_t  TELEGRAM_REINTENTOS        = 3;     // errores HTTP (4xx/5xx) antes de descartar
constexpr uint32_t TELEGRAM_ESPERA_REINTENTO_MS = 3000;

// Escenario "caída del WiFi": el pulsador "Caida WiFi" (GPIO 26) corta y restablece el WiFi.
// (Los 4 pulsadores de voto se quitaron del circuito: el modo votación está postergado, DEC-020.)

// =================================================================
// 2a. Declaraciones de sensores  (sensores.h)
// =================================================================
// sensores.h — Entradas de la caja: LDR de los casilleros, pulsadores y fin de carrera.


const char* nombreEstadoCasillero(EstadoCasillero e);

// ---------- LDR ----------
void            sensoresIniciar();
void            sensoresActualizar(uint32_t ahora);   // llamar en cada vuelta de loop()
int             ldrPromedio(uint8_t casillero);       // 0–4095, promedio móvil
EstadoCasillero estadoCasillero(uint8_t casillero);  // estado confirmado
bool            cambioPendiente(uint8_t casillero);  // hay un cambio esperando confirmación
uint32_t        estadoDesde(uint8_t casillero);      // millis() en que EMPEZÓ el último cambio confirmado
                                                     // (≈ 1 s antes de la confirmación; sirve para fechar retiros)
bool            casilleroCambio(uint8_t casillero);  // true una vez después de cada cambio confirmado

// ---------- Pulsadores (antirrebote + pulsación larga) ----------
class Boton {
public:
  void iniciar(uint8_t pin);
  void actualizar(uint32_t ahora);
  bool presionado() const { return estable_; }        // nivel actual (ya filtrado)
  bool flancoPresion();                                // true una vez al apretar
  bool flancoSoltadoCorto();                           // true una vez al soltar antes de la pulsación larga
  bool pulsacionLarga();                               // true una vez al cumplir PULSACION_LARGA_MS

private:
  uint8_t  pin_ = 0;
  bool     crudoAnterior_ = false;
  bool     estable_ = false;
  uint32_t ultimoCambio_ = 0;
  uint32_t inicioPresion_ = 0;
  bool     flancoPresion_ = false;
  bool     flancoSoltadoCorto_ = false;
  bool     largaAvisada_ = false;
  bool     largaPendiente_ = false;
};

extern Boton botonInicio;
extern Boton botonWifi;

// ---------- Fin de carrera de la bandera ----------
bool finDeCarreraCerrado();   // true = la bandera llegó arriba

// =================================================================
// 2b. Declaraciones de actuadores  (actuadores.h)
// =================================================================
// actuadores.h — Salidas de la caja: NeoPixel, OLED, buzzer y servo de la bandera.

// Colores 0xRRGGBB
constexpr uint32_t rgb(uint8_t r, uint8_t g, uint8_t b) {
  return (uint32_t(r) << 16) | (uint32_t(g) << 8) | b;
}
constexpr uint32_t COLOR_APAGADO   = rgb(0, 0, 0);
constexpr uint32_t COLOR_LIBRE     = rgb(0, 0, 40);      // azul tenue
constexpr uint32_t COLOR_OCUPADO   = rgb(0, 150, 0);     // verde
constexpr uint32_t COLOR_ALERTA    = rgb(255, 0, 0);     // rojo
constexpr uint32_t COLOR_FALLA     = rgb(255, 0, 255);   // magenta: "sin sensor" / falla de la bandera
constexpr uint32_t COLOR_AMBIENTE  = rgb(255, 120, 30);  // cálido
constexpr uint32_t COLOR_DUDOSO    = rgb(150, 150, 0);   // amarillo (cambio esperando confirmación, solo prueba)
constexpr uint32_t COLOR_LISTA     = rgb(0, 80, 0);      // ambiente verde: todos listos para empezar
constexpr uint32_t COLOR_FIN       = rgb(60, 60, 120);   // ambiente lavanda: resumen de la juntada

void actuadoresIniciar();
void actuadoresActualizar(uint32_t ahora);   // llamar en cada vuelta de loop()

// ---------- NeoPixel ----------
void ledCasillero(uint8_t casillero, uint32_t color);
void ledsAmbiente(uint32_t color);
void ledsApagar();
void ledPixel(uint8_t indice, uint32_t color);   // cualquier píxel 0–7 (p. ej. el 7 como LED de falla de la bandera)

// ---------- OLED ----------
extern Adafruit_SSD1306 oled;
bool oledTocaRefrescar(uint32_t ahora);      // true cada OLED_REFRESCO_MS

// ---------- Buzzer (no bloqueante) ----------
struct Nota { uint16_t frecuenciaHz; uint16_t duracionMs; };   // frecuencia 0 = silencio
void buzzerBeep(uint16_t frecuenciaHz, uint16_t duracionMs);
void buzzerSecuencia(const Nota* notas, uint8_t cantidad);
bool buzzerOcupado();
void buzzerApagar();

// ---------- Servo de la bandera (DEC-020) ----------
void banderaLevantar();
void banderaBajar();
bool banderaArriba();                         // último comando enviado (no confirma posición)

// =================================================================
// 2c. Declaraciones de la máquina de estados  (estados.h)
// =================================================================
// estados.h — Máquina de estados de la caja (S1.4).
//
//   ESPERA ──(todos los asignados OCUPADO)──> LISTA
//   ESPERA/LISTA ──(toque corto del botón, con >= MIN_PARTICIPANTES)──> JUNTADA
//   LISTA ──(se libera un casillero)──> ESPERA
//   JUNTADA ──(un participante retira el celular)──> ALERTA
//   ALERTA ──(todos los retirados lo vuelven a poner)──> JUNTADA
//   JUNTADA/ALERTA ──(pulsación larga)──> FIN
//   FIN ──(toque corto)──> ESPERA
//
// Bandera (S1.6, DEC-020), en paralelo con JUNTADA/ALERTA: al iniciar, el servo levanta
// la bandera y se espera hasta BANDERA_TIMEOUT_MS a que el fin de carrera cierre. Sin
// confirmación → falla de actuador: aviso, píxel 7 en magenta, servo a reposo y la
// juntada sigue sin bandera. La caja no impide sacar ningún celular.
//
// La juntada nunca arranca sola: siempre hace falta el botón (DEC-019).


// Bandera (S1.6, DEC-020; antes "traba de la tapa"): el servo la levanta al iniciar y el
// fin de carrera tiene que confirmarlo dentro de BANDERA_TIMEOUT_MS.

const char* nombreEstadoCaja(EstadoCaja e);

// Un retiro de celular durante la juntada.
struct Retiro {
  uint8_t  casillero;       // 0..CANT_CASILLEROS-1
  uint32_t msDesdeInicio;   // momento del retiro, relativo al inicio
};

// Datos de la juntada en curso o de la última terminada (base para el resumen
// y, desde el Sprint 2, para publicar en Firestore y Telegram).
struct Juntada {
  bool     participante[CANT_CASILLEROS];  // casilleros ocupados al iniciar
  uint8_t  cantParticipantes;
  uint32_t inicioMs;
  uint32_t duracionMs;                     // se completa al terminar
  int8_t   primero;                        // primer casillero en retirar el celular (-1 = nadie)
  uint8_t  retirosPorCasillero[CANT_CASILLEROS];
  Retiro   retiros[MAX_RETIROS];
  uint8_t  cantRetiros;                    // retiros guardados (hasta MAX_RETIROS)
  uint16_t totalRetiros;                   // retiros contados (puede superar MAX_RETIROS)
  bool     sinSensor[CANT_CASILLEROS];     // participantes cuyo sensor falló durante la juntada
  bool     fallaAlIniciar[CANT_CASILLEROS];// asignados con el sensor en falla al iniciar (quedan afuera)
  EstadoBandera bandera;                   // resultado de la bandera en esta juntada
  uint16_t numero;                         // número de juntada desde que se encendió la caja
};

void           estadosIniciar();
void           estadosActualizar(uint32_t ahora);   // llamar en cada vuelta de loop()
EstadoCaja     estadoActual();
const Juntada& juntadaActual();
void           avisoOled(const char* texto);        // aviso corto en la última línea del OLED

// =================================================================
// 2d. Declaraciones de red  (red.h)
// =================================================================
// red.h — WiFi y Telegram (Sprint 2). Firestore pasa al Sprint 3 (decisión del 24/11).
// La caja funciona igual sin internet: la red nunca bloquea el loop().

struct MensajeTelegram { char texto[TELEGRAM_TEXTO_MAX]; };

void redIniciar();
void redActualizar(uint32_t ahora);
bool redConectada();
void telegramEnviar(const char* texto);   // no bloquea: deja el mensaje en la cola

// =================================================================
// 2e. Declaraciones del modo prueba  (prueba_circuito.h)
// =================================================================
// prueba_circuito.h — Modo de prueba del circuito (Sprint 1, paso 1).
// Se activa con MODO_PRUEBA_CIRCUITO 1 en config.h.

void pruebaIniciar();
void pruebaActualizar(uint32_t ahora);

// =================================================================
// 3. Sensores  (sensores.cpp)
// =================================================================
// sensores.cpp — Implementación de las entradas. Sin delay(): todo con millis().

Boton botonInicio;
Boton botonWifi;

const char* nombreEstadoCasillero(EstadoCasillero e) {
  switch (e) {
    case EstadoCasillero::LIBRE:   return "LIBRE";
    case EstadoCasillero::OCUPADO: return "OCUPADO";
    case EstadoCasillero::FALLA:   return "FALLA";
  }
  return "?";
}

// =================================================================
// LDR: muestreo periódico + promedio móvil + clasificación con confirmación
//
// Cada LDR_PERIODO_MUESTRA_MS se toma una muestra por casillero y se recalcula
// el promedio móvil. Con ese promedio se calcula un estado "candidato":
//   promedio < UMBRAL_INVALIDO              → FALLA
//   promedio > UMBRAL_OCUPADO               → OCUPADO
//   promedio < UMBRAL_LIBRE                 → LIBRE
//   entre UMBRAL_LIBRE y UMBRAL_OCUPADO     → el estado actual (histéresis)
// El estado confirmado cambia recién cuando el candidato se sostiene
// CONFIRMACION_MS (OCUPADO/LIBRE) o FALLA_SENSOR_MS (FALLA). Si el candidato
// vuelve al estado actual antes de ese tiempo, el cambio se descarta.
// =================================================================
struct Casillero {
  uint16_t        muestras[LDR_MUESTRAS];
  EstadoCasillero confirmado;     // estado vigente
  EstadoCasillero pendiente;      // candidato distinto del vigente (si hayPendiente)
  bool            hayPendiente;
  uint32_t        pendienteDesde; // millis() en que apareció el candidato
  uint32_t        confirmadoDesde;// millis() en que empezó el último cambio confirmado
  bool            cambio;         // aviso de cambio para quien lo consuma
};

static Casillero cas[CANT_CASILLEROS];
static uint8_t   indiceMuestra = 0;
static uint32_t  ultimaMuestra = 0;

void sensoresIniciar() {
  analogReadResolution(12);                 // 0–4095
  uint32_t ahora = millis();
  for (uint8_t c = 0; c < CANT_CASILLEROS; c++) {
    // GPIO 34 y 35 son solo entrada y no tienen pull internos: el pull-down es el de 1 MΩ externo.
    pinMode(PIN_LDR[c], INPUT);
    uint16_t primera = analogRead(PIN_LDR[c]);
    for (uint8_t m = 0; m < LDR_MUESTRAS; m++) cas[c].muestras[m] = primera;  // arranca lleno
    // Estado inicial sin esperar confirmación (OCUPADO o LIBRE). FALLA siempre pasa
    // por la espera de FALLA_SENSOR_MS, también al arrancar.
    cas[c].confirmado      = (primera > UMBRAL_OCUPADO) ? EstadoCasillero::OCUPADO : EstadoCasillero::LIBRE;
    cas[c].hayPendiente    = false;
    cas[c].pendiente       = cas[c].confirmado;
    cas[c].pendienteDesde  = ahora;
    cas[c].confirmadoDesde = ahora;
    cas[c].cambio          = false;
  }

  botonInicio.iniciar(PIN_BOTON_INICIO);
  botonWifi.iniciar(PIN_BOTON_WIFI);
  pinMode(PIN_FIN_CARRERA, INPUT_PULLUP);
}

int ldrPromedio(uint8_t c) {
  uint32_t suma = 0;
  for (uint8_t m = 0; m < LDR_MUESTRAS; m++) suma += cas[c].muestras[m];
  return suma / LDR_MUESTRAS;
}

static EstadoCasillero candidato(uint8_t c) {
  int v = ldrPromedio(c);
  if (v < UMBRAL_INVALIDO) return EstadoCasillero::FALLA;
  if (v > UMBRAL_OCUPADO)  return EstadoCasillero::OCUPADO;
  if (v < UMBRAL_LIBRE)    return EstadoCasillero::LIBRE;
  return cas[c].confirmado;                 // zona de histéresis: no hay cambio
}

static void clasificar(uint8_t c, uint32_t ahora) {
  Casillero& k = cas[c];
  EstadoCasillero cand = candidato(c);

  if (cand == k.confirmado) {               // nada que cambiar: se descarta cualquier pendiente
    k.hayPendiente = false;
    return;
  }
  if (!k.hayPendiente || cand != k.pendiente) {   // aparece (o cambia) el candidato: arranca la espera
    k.hayPendiente = true;
    k.pendiente = cand;
    k.pendienteDesde = ahora;
    return;
  }
  uint32_t espera = (cand == EstadoCasillero::FALLA) ? FALLA_SENSOR_MS : CONFIRMACION_MS;
  if (ahora - k.pendienteDesde >= espera) {
    Serial.printf("[%lu ms] C%u: %s -> %s (lectura %d, sostenido %lu ms)\n",
                  (unsigned long)ahora, c + 1,
                  nombreEstadoCasillero(k.confirmado), nombreEstadoCasillero(cand),
                  ldrPromedio(c), (unsigned long)(ahora - k.pendienteDesde));
    k.confirmado = cand;
    k.confirmadoDesde = k.pendienteDesde;   // el cambio empezó cuando apareció el candidato, no al confirmarse
    k.hayPendiente = false;
    k.cambio = true;
  }
}

EstadoCasillero estadoCasillero(uint8_t c) { return cas[c].confirmado; }
bool            cambioPendiente(uint8_t c) { return cas[c].hayPendiente; }
uint32_t        estadoDesde(uint8_t c)     { return cas[c].confirmadoDesde; }
bool            casilleroCambio(uint8_t c) { bool r = cas[c].cambio; cas[c].cambio = false; return r; }

void sensoresActualizar(uint32_t ahora) {
  if (ahora - ultimaMuestra >= LDR_PERIODO_MUESTRA_MS) {
    ultimaMuestra = ahora;
    for (uint8_t c = 0; c < CANT_CASILLEROS; c++) cas[c].muestras[indiceMuestra] = analogRead(PIN_LDR[c]);
    indiceMuestra = (indiceMuestra + 1) % LDR_MUESTRAS;
    for (uint8_t c = 0; c < CANT_CASILLEROS; c++) clasificar(c, ahora);
  }

  botonInicio.actualizar(ahora);
  botonWifi.actualizar(ahora);
}

// =================================================================
// Fin de carrera (interruptor deslizante entre GPIO 23 y GND)
// =================================================================
bool finDeCarreraCerrado() {
  return digitalRead(PIN_FIN_CARRERA) == LOW;
}

// =================================================================
// Boton: antirrebote por tiempo + detección de pulsación larga
// =================================================================
void Boton::iniciar(uint8_t pin) {
  pin_ = pin;
  pinMode(pin_, INPUT_PULLUP);
  crudoAnterior_ = estable_ = (digitalRead(pin_) == LOW);
  ultimoCambio_ = millis();
}

void Boton::actualizar(uint32_t ahora) {
  bool crudo = (digitalRead(pin_) == LOW);          // apretado = LOW (pull-up)
  if (crudo != crudoAnterior_) {
    crudoAnterior_ = crudo;
    ultimoCambio_ = ahora;
  }
  if (crudo != estable_ && (ahora - ultimoCambio_) >= ANTIRREBOTE_MS) {
    estable_ = crudo;
    if (estable_) {                                 // se apretó
      inicioPresion_ = ahora;
      flancoPresion_ = true;
      largaAvisada_ = false;
    } else if (!largaAvisada_) {                    // se soltó antes de la pulsación larga
      flancoSoltadoCorto_ = true;
    }
  }
  if (estable_ && !largaAvisada_ && (ahora - inicioPresion_) >= PULSACION_LARGA_MS) {
    largaAvisada_ = true;
    largaPendiente_ = true;
  }
}

bool Boton::flancoPresion()      { bool r = flancoPresion_;      flancoPresion_ = false;      return r; }
bool Boton::flancoSoltadoCorto() { bool r = flancoSoltadoCorto_; flancoSoltadoCorto_ = false; return r; }
bool Boton::pulsacionLarga()     { bool r = largaPendiente_;     largaPendiente_ = false;     return r; }

// =================================================================
// 4. Actuadores  (actuadores.cpp)
// =================================================================
// actuadores.cpp — Implementación de las salidas. Sin delay(): todo con millis().

Adafruit_SSD1306 oled(OLED_ANCHO, OLED_ALTO, &Wire, -1);
static Adafruit_NeoPixel tira(NEOPIXEL_CANT, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
static Servo servoBandera;

static bool     tiraCambio = true;     // hay que mandar los colores a la tira
static uint32_t ultimoRefrescoOled = 0;
static bool     buzzerSonando = false;   // hay un tono o una secuencia en curso
static uint32_t buzzerFin = 0;           // fin de la nota actual
static const Nota* secuencia = nullptr;  // secuencia en curso (nullptr = beep suelto)
static uint8_t  secuenciaLargo = 0;
static uint8_t  secuenciaIndice = 0;
static bool     alzada = false;

void actuadoresIniciar() {
  // NeoPixel
  tira.begin();
  tira.setBrightness(NEOPIXEL_BRILLO);
  tira.clear();
  tira.show();

  // OLED
  Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);
  Wire.setClock(400000);
  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_DIRECCION)) {
    Serial.println(F("[ERROR] OLED no responde (revisar SDA 21 / SCL 22)"));
  }
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.display();

  // Buzzer
  pinMode(PIN_BUZZER, OUTPUT);
  noTone(PIN_BUZZER);

  // Servo (SG90: 50 Hz, pulso 500–2400 µs)
  servoBandera.setPeriodHertz(50);
  servoBandera.attach(PIN_SERVO, 500, 2400);
  banderaBajar();
}

void actuadoresActualizar(uint32_t ahora) {
  if (tiraCambio) {
    tira.show();
    tiraCambio = false;
  }
  if (buzzerSonando && (int32_t)(ahora - buzzerFin) >= 0) {
    if (secuencia != nullptr && secuenciaIndice + 1 < secuenciaLargo) {
      secuenciaIndice++;
      const Nota& n = secuencia[secuenciaIndice];
      if (n.frecuenciaHz > 0) tone(PIN_BUZZER, n.frecuenciaHz); else noTone(PIN_BUZZER);
      buzzerFin = ahora + n.duracionMs;
    } else {
      buzzerApagar();
    }
  }
}

// ---------- NeoPixel ----------
void ledCasillero(uint8_t c, uint32_t color) {
  if (c >= CANT_CASILLEROS) return;
  if (tira.getPixelColor(c) != color) { tira.setPixelColor(c, color); tiraCambio = true; }
}

void ledsAmbiente(uint32_t color) {
  for (uint8_t i = NEOPIXEL_PRIMER_AMBIENTE; i < NEOPIXEL_CANT; i++) {
    if (tira.getPixelColor(i) != color) { tira.setPixelColor(i, color); tiraCambio = true; }
  }
}

void ledPixel(uint8_t i, uint32_t color) {
  if (i >= NEOPIXEL_CANT) return;
  if (tira.getPixelColor(i) != color) { tira.setPixelColor(i, color); tiraCambio = true; }
}

void ledsApagar() {
  tira.clear();
  tiraCambio = true;
}

// ---------- OLED ----------
bool oledTocaRefrescar(uint32_t ahora) {
  if (ahora - ultimoRefrescoOled >= OLED_REFRESCO_MS) {
    ultimoRefrescoOled = ahora;
    return true;
  }
  return false;
}

// ---------- Buzzer ----------
// El buzzer de Wokwi es pasivo: necesita una frecuencia (tone), no un nivel fijo.
void buzzerBeep(uint16_t frecuenciaHz, uint16_t duracionMs) {
  secuencia = nullptr;
  tone(PIN_BUZZER, frecuenciaHz);
  buzzerSonando = true;
  buzzerFin = millis() + duracionMs;
}

// Reproduce una secuencia de notas sin bloquear (frecuencia 0 = silencio).
// El arreglo tiene que seguir existiendo mientras suena (usar arreglos static/const).
void buzzerSecuencia(const Nota* notas, uint8_t cantidad) {
  if (cantidad == 0) return;
  secuencia = notas;
  secuenciaLargo = cantidad;
  secuenciaIndice = 0;
  if (notas[0].frecuenciaHz > 0) tone(PIN_BUZZER, notas[0].frecuenciaHz); else noTone(PIN_BUZZER);
  buzzerSonando = true;
  buzzerFin = millis() + notas[0].duracionMs;
}

bool buzzerOcupado() { return buzzerSonando; }

void buzzerApagar() {
  noTone(PIN_BUZZER);
  buzzerSonando = false;
  secuencia = nullptr;
}

// ---------- Servo ----------
// ---------- Servo de la bandera ----------
void banderaLevantar() { servoBandera.write(SERVO_ANGULO_ALZADA); alzada = true; }
void banderaBajar()    { servoBandera.write(SERVO_ANGULO_BAJA);   alzada = false; }
bool banderaArriba()   { return alzada; }

// =================================================================
// 5. Máquina de estados  (estados.cpp)
// =================================================================
// estados.cpp — Máquina de estados de la caja (S1.4). Ver el diagrama en estados.h.
//
// Reglas:
//  - Participantes: casilleros asignados que están OCUPADO al apretar el botón de inicio.
//  - Retiro: un participante pasa de OCUPADO a LIBRE (estado ya confirmado por sensores.cpp).
//    Se registra casillero y momento (relativo al inicio); el primero queda marcado.
//  - Un participante cuyo sensor pasa a FALLA queda "sin sensor" y la juntada sigue con
//    los demás (base de S1.7; el aviso completo en el OLED se termina en S1.7).
//  - Casilleros no participantes se ignoran durante la juntada.
//  - Bandera (S1.6, DEC-020; reemplaza a la traba): ver revisarBandera(). El fin de
//    carrera solo se mira durante los BANDERA_TIMEOUT_MS posteriores a la orden de levantar (en Wokwi el interruptor no está
//    atado al servo y queda donde se lo deja).
//
//  - Falla de sensor (S1.7, v0.6): en CUALQUIER estado, cuando un casillero asignado pasa a
//    FALLA se avisa por Serial, OLED (aviso + línea fija "Sin sensor"), buzzer, LED magenta y
//    Telegram. Un participante que falla queda "sin sensor"; un casillero en falla al iniciar
//    queda fuera de la juntada. En ambos casos la juntada sigue con los demás.
//  - Telegram (v0.6): inicio, cada retiro, fallas de sensor y de bandera, y resumen final.

// ---------------------------------------------------------------
// Estado interno
// ---------------------------------------------------------------
static EstadoCaja      estado = EstadoCaja::ESPERA;
static uint32_t        entradaEstado = 0;
static Juntada         jun;
static EstadoCasillero previo[CANT_CASILLEROS];   // estado de cada casillero en la vuelta anterior
static bool            afuera[CANT_CASILLEROS];   // participante con el celular afuera ahora
static uint32_t        alertaDesde = 0;           // último retiro (para limitar el sonido)
static uint32_t        banderaDesde = 0;          // momento de la orden de levantar la bandera
static char            mensaje[32] = "";
static uint32_t        mensajeHasta = 0;
static uint16_t        numeroJuntada = 0;         // juntadas desde que se encendió la caja

// ---------------------------------------------------------------
// Sonidos (frecuencia 0 = silencio)
// ---------------------------------------------------------------
static const Nota SONIDO_LISTA[]    = { {1320, 100}, {0, 80}, {1320, 100} };
static const Nota SONIDO_INICIO[]   = { {880, 120}, {0, 60}, {1320, 220} };
static const Nota SONIDO_ALERTA[]   = { {2000, 180}, {0, 120}, {2000, 180}, {0, 400} };
static const Nota SONIDO_REPUESTO[] = { {1320, 80} };
static const Nota SONIDO_FIN[]      = { {1320, 150}, {0, 50}, {990, 150}, {0, 50}, {660, 350} };
static const Nota SONIDO_ERROR[]    = { {300, 300} };
#define LARGO(a) (uint8_t)(sizeof(a) / sizeof(a[0]))

// ---------------------------------------------------------------
// Utilidades
// ---------------------------------------------------------------
const char* nombreEstadoCaja(EstadoCaja e) {
  switch (e) {
    case EstadoCaja::ESPERA:  return "ESPERA";
    case EstadoCaja::LISTA:   return "LISTA";
    case EstadoCaja::JUNTADA: return "JUNTADA";
    case EstadoCaja::ALERTA:  return "ALERTA";
    case EstadoCaja::FIN:     return "FIN";
  }
  return "?";
}

EstadoCaja     estadoActual()  { return estado; }
const Juntada& juntadaActual() { return jun; }

// hh:mm:ss
static void formatoTiempo(uint32_t ms, char* buf, size_t largo) {
  uint32_t s = ms / 1000;
  snprintf(buf, largo, "%02lu:%02lu:%02lu",
           (unsigned long)(s / 3600), (unsigned long)((s / 60) % 60), (unsigned long)(s % 60));
}

static void mostrarMensaje(const char* texto, uint32_t ahora) {
  strncpy(mensaje, texto, sizeof(mensaje) - 1);
  mensaje[sizeof(mensaje) - 1] = '\0';
  mensajeHasta = ahora + MENSAJE_TEMPORAL_MS;
}

static bool hayMensaje(uint32_t ahora) { return (int32_t)(mensajeHasta - ahora) > 0; }

void avisoOled(const char* texto) { mostrarMensaje(texto, millis()); }

// Agrega texto con formato al final de buf (para armar los mensajes de Telegram).
static void agregar(char* buf, size_t largo, const char* formato, ...) {
  size_t usado = strlen(buf);
  if (usado + 1 >= largo) return;
  va_list args;
  va_start(args, formato);
  vsnprintf(buf + usado, largo - usado, formato, args);
  va_end(args);
}

// Hay algún casillero asignado con el sensor en falla (para las líneas fijas del OLED).
static bool haySensorEnFalla() {
  for (uint8_t c = 0; c < CANT_CASILLEROS; c++)
    if (CASILLERO_ASIGNADO[c] && estadoCasillero(c) == EstadoCasillero::FALLA) return true;
  return false;
}

static void oledListaSinSensor() {
  oled.print(F("Sin sensor:"));
  for (uint8_t c = 0; c < CANT_CASILLEROS; c++)
    if (CASILLERO_ASIGNADO[c] && estadoCasillero(c) == EstadoCasillero::FALLA) oled.printf(" C%u", c + 1);
  oled.println();
}

static void cambiarEstado(EstadoCaja nuevo, uint32_t ahora) {
  Serial.printf("[%lu ms] Estado: %s -> %s\n", (unsigned long)ahora,
                nombreEstadoCaja(estado), nombreEstadoCaja(nuevo));
  estado = nuevo;
  entradaEstado = ahora;
  mensajeHasta = ahora;   // un aviso del estado anterior no se arrastra al nuevo
}

// Casilleros asignados con celular.
static uint8_t ocupadosAsignados() {
  uint8_t n = 0;
  for (uint8_t c = 0; c < CANT_CASILLEROS; c++)
    if (CASILLERO_ASIGNADO[c] && estadoCasillero(c) == EstadoCasillero::OCUPADO) n++;
  return n;
}

// Todos los asignados con sensor válido están ocupados (los que están en FALLA no se esperan).
static bool todosListos() {
  uint8_t esperados = 0;
  for (uint8_t c = 0; c < CANT_CASILLEROS; c++)
    if (CASILLERO_ASIGNADO[c] && estadoCasillero(c) != EstadoCasillero::FALLA) esperados++;
  uint8_t ocupados = ocupadosAsignados();
  return esperados > 0 && ocupados == esperados && ocupados >= MIN_PARTICIPANTES;
}

static uint8_t participantesAfuera() {
  uint8_t n = 0;
  for (uint8_t c = 0; c < CANT_CASILLEROS; c++) if (afuera[c]) n++;
  return n;
}

// ---------------------------------------------------------------
// Acciones de la juntada
// ---------------------------------------------------------------
static void iniciarJuntada(uint32_t ahora) {
  memset(&jun, 0, sizeof(jun));
  jun.primero = -1;
  jun.inicioMs = ahora;
  Serial.print(F("[INICIO] Participantes:"));
  for (uint8_t c = 0; c < CANT_CASILLEROS; c++) {
    jun.participante[c] = CASILLERO_ASIGNADO[c] && estadoCasillero(c) == EstadoCasillero::OCUPADO;
    afuera[c] = false;
    if (jun.participante[c]) { jun.cantParticipantes++; Serial.printf(" C%u", c + 1); }
  }
  Serial.println();

  // Casilleros asignados con el sensor en falla al iniciar: quedan afuera de esta juntada.
  bool algunaFalla = false;
  for (uint8_t c = 0; c < CANT_CASILLEROS; c++) {
    jun.fallaAlIniciar[c] = CASILLERO_ASIGNADO[c] && estadoCasillero(c) == EstadoCasillero::FALLA;
    if (jun.fallaAlIniciar[c]) {
      algunaFalla = true;
      Serial.printf("[FALLA] C%u con el sensor en falla: queda fuera de esta juntada\n", c + 1);
    }
  }
  jun.numero = ++numeroJuntada;

  banderaLevantar();
  jun.bandera = EstadoBandera::SUBIENDO;
  banderaDesde = ahora;
  Serial.println(F("[BANDERA] Levantando: esperando el fin de carrera"));
  buzzerSecuencia(SONIDO_INICIO, LARGO(SONIDO_INICIO));
  cambiarEstado(EstadoCaja::JUNTADA, ahora);

  // Telegram
  char txt[TELEGRAM_TEXTO_MAX] = "";
  agregar(txt, sizeof(txt), "🚩 Arrancó la juntada #%u\n👥 Participantes:", jun.numero);
  bool primeroDeLista = true;
  for (uint8_t c = 0; c < CANT_CASILLEROS; c++) {
    if (!jun.participante[c]) continue;
    agregar(txt, sizeof(txt), "%s %s", primeroDeLista ? "" : ",", NOMBRE_CASILLERO[c]);
    primeroDeLista = false;
  }
  if (algunaFalla) {
    agregar(txt, sizeof(txt), "\n⚠️ Sin sensor (quedan afuera):");
    for (uint8_t c = 0; c < CANT_CASILLEROS; c++)
      if (jun.fallaAlIniciar[c]) agregar(txt, sizeof(txt), " %s", NOMBRE_CASILLERO[c]);
  }
  agregar(txt, sizeof(txt), "\nEl primero que levante el celular queda registrado 👀");
  telegramEnviar(txt);
}

static void registrarRetiro(uint8_t c, uint32_t ahora) {
  // Momento del retiro: cuando empezó el cambio, no cuando se confirmó (≈ 1 s después).
  // Si empezó antes de iniciar la juntada, se cuenta como 0.
  uint32_t desde = estadoDesde(c);
  if ((int32_t)(desde - jun.inicioMs) < 0) desde = jun.inicioMs;
  (void)ahora;
  uint32_t t = desde - jun.inicioMs;
  afuera[c] = true;
  jun.retirosPorCasillero[c]++;
  jun.totalRetiros++;
  if (jun.cantRetiros < MAX_RETIROS) jun.retiros[jun.cantRetiros++] = { c, t };

  bool esPrimero = (jun.primero < 0);
  if (esPrimero) jun.primero = c;

  char tiempo[12];
  formatoTiempo(t, tiempo, sizeof(tiempo));
  Serial.printf("[RETIRO] C%u a los %s%s\n", c + 1, tiempo, esPrimero ? " (primero en levantarlo)" : "");

  char txt[TELEGRAM_TEXTO_MAX] = "";
  agregar(txt, sizeof(txt), "📵 %s levantó el celular a los %s", NOMBRE_CASILLERO[c], tiempo);
  if (esPrimero) agregar(txt, sizeof(txt), "\n¡Es el primero en levantarlo!");
  agregar(txt, sizeof(txt), "\nRetiros en esta juntada: %u", jun.totalRetiros);
  telegramEnviar(txt);
}

// Revisa los participantes: retiros, celulares repuestos y sensores en falla.
// Devuelve cuántos retiros nuevos hubo en esta vuelta.
static uint8_t revisarParticipantes(uint32_t ahora) {
  uint8_t nuevos = 0;
  for (uint8_t c = 0; c < CANT_CASILLEROS; c++) {
    if (!jun.participante[c] || jun.sinSensor[c]) continue;
    EstadoCasillero e = estadoCasillero(c);

    if (e == EstadoCasillero::FALLA) {
      jun.sinSensor[c] = true;   // el aviso lo da revisarFallasSensor()
      afuera[c] = false;
    } else if (previo[c] == EstadoCasillero::OCUPADO && e == EstadoCasillero::LIBRE) {
      registrarRetiro(c, ahora);
      nuevos++;
    } else if (afuera[c] && e == EstadoCasillero::OCUPADO) {
      afuera[c] = false;
      Serial.printf("[REPUESTO] C%u volvio a la caja\n", c + 1);
    }
  }
  return nuevos;
}

// Falla de sensor (S1.7): avisa cuando un casillero asignado entra en FALLA o se recupera,
// en cualquier estado de la caja. La reacción de la juntada la maneja revisarParticipantes().
static void revisarFallasSensor(uint32_t ahora) {
  bool enJuntada = (estado == EstadoCaja::JUNTADA || estado == EstadoCaja::ALERTA);
  for (uint8_t c = 0; c < CANT_CASILLEROS; c++) {
    if (!CASILLERO_ASIGNADO[c]) continue;
    EstadoCasillero e = estadoCasillero(c);

    if (e == EstadoCasillero::FALLA && previo[c] != EstadoCasillero::FALLA) {
      bool participa = enJuntada && jun.participante[c];
      Serial.printf("[FALLA] C%u: sensor sin lectura valida (promedio %d). %s\n", c + 1, ldrPromedio(c),
                    participa ? "La juntada sigue con los demas" : "El casillero no puede participar");
      char aviso[24];
      snprintf(aviso, sizeof(aviso), "FALLA SENSOR C%u", c + 1);
      mostrarMensaje(aviso, ahora);
      if (estado != EstadoCaja::ALERTA) buzzerSecuencia(SONIDO_ERROR, LARGO(SONIDO_ERROR));

      char txt[TELEGRAM_TEXTO_MAX] = "";
      agregar(txt, sizeof(txt), "⚠️ Falla de sensor en %s: sin lectura válida (¿sensor desconectado?).\n%s",
              NOMBRE_CASILLERO[c],
              participa ? "La juntada sigue con los demás." : "Ese casillero no puede participar hasta revisarlo.");
      telegramEnviar(txt);
    } else if (e != EstadoCasillero::FALLA && previo[c] == EstadoCasillero::FALLA) {
      Serial.printf("[SENSOR] C%u volvio a leer (%s)\n", c + 1, nombreEstadoCasillero(e));
    }
  }
}

// Falla de actuador (S1.6): el servo recibió la orden de levantar la bandera pero el fin
// de carrera no confirmó que llegó arriba dentro de BANDERA_TIMEOUT_MS.
static void revisarBandera(uint32_t ahora) {
  if (jun.bandera != EstadoBandera::SUBIENDO) return;
  if (finDeCarreraCerrado()) {
    jun.bandera = EstadoBandera::CONFIRMADA;
    Serial.printf("[BANDERA] Arriba, confirmada por el fin de carrera (%lu ms)\n", (unsigned long)(ahora - banderaDesde));
  } else if (ahora - banderaDesde >= BANDERA_TIMEOUT_MS) {
    jun.bandera = EstadoBandera::FALLA;
    banderaBajar();   // a reposo: no se deja el servo forzando contra algo que lo frena
    buzzerSecuencia(SONIDO_ERROR, LARGO(SONIDO_ERROR));
    mostrarMensaje("FALLA BANDERA", ahora);
    Serial.printf("[FALLA] Bandera sin confirmar en %lu ms: la juntada sigue sin bandera\n",
                  (unsigned long)BANDERA_TIMEOUT_MS);
    telegramEnviar("⚠️ Falla de actuador: la bandera no subió (el fin de carrera no confirmó en 2 s).\n"
                   "La juntada sigue sin bandera.");
  }
}

static const char* textoBandera(EstadoBandera e) {
  switch (e) {
    case EstadoBandera::CONFIRMADA: return "confirmada";
    case EstadoBandera::FALLA:      return "FALLA (no llegó arriba)";
    case EstadoBandera::SUBIENDO:   return "sin confirmar (fin antes de 2 s)";
    default:                      return "-";
  }
}

static void imprimirResumen() {
  char tiempo[12];
  formatoTiempo(jun.duracionMs, tiempo, sizeof(tiempo));
  char minimo[12];
  formatoTiempo(DURACION_MINIMA_MS, minimo, sizeof(minimo));
  Serial.println(F("========== RESUMEN DE LA JUNTADA =========="));
  Serial.printf("Duracion: %s (minimo %s: %s)\n", tiempo, minimo,
                jun.duracionMs >= DURACION_MINIMA_MS ? "cumplido" : "no cumplido");
  Serial.print(F("Participantes:"));
  for (uint8_t c = 0; c < CANT_CASILLEROS; c++) if (jun.participante[c]) Serial.printf(" C%u", c + 1);
  Serial.println();
  if (jun.primero >= 0) Serial.printf("Primero en levantarlo: C%u\n", jun.primero + 1);
  else                  Serial.println(F("Primero en levantarlo: nadie!"));
  Serial.printf("Retiros: %u (", jun.totalRetiros);
  bool primeroDeLista = true;
  for (uint8_t c = 0; c < CANT_CASILLEROS; c++) {
    if (!jun.participante[c]) continue;
    Serial.printf("%sC%u:%u", primeroDeLista ? "" : " ", c + 1, jun.retirosPorCasillero[c]);
    primeroDeLista = false;
  }
  Serial.println(F(")"));
  for (uint8_t i = 0; i < jun.cantRetiros; i++) {
    formatoTiempo(jun.retiros[i].msDesdeInicio, tiempo, sizeof(tiempo));
    Serial.printf("  - C%u a los %s\n", jun.retiros[i].casillero + 1, tiempo);
  }
  Serial.print(F("Sensores en falla:"));
  bool alguno = false;
  for (uint8_t c = 0; c < CANT_CASILLEROS; c++) {
    if (jun.sinSensor[c])           { Serial.printf(" C%u (durante la juntada)", c + 1); alguno = true; }
    else if (jun.fallaAlIniciar[c]) { Serial.printf(" C%u (desde el inicio)", c + 1);    alguno = true; }
  }
  Serial.println(alguno ? "" : " ninguno");
  Serial.printf("Bandera: %s\n", textoBandera(jun.bandera));
  Serial.println(F("==========================================="));
}

// Resumen al grupo de Telegram (feature 9).
static void enviarResumenTelegram() {
  char txt[TELEGRAM_TEXTO_MAX] = "";
  char tiempo[12], minimo[12];
  formatoTiempo(jun.duracionMs, tiempo, sizeof(tiempo));
  formatoTiempo(DURACION_MINIMA_MS, minimo, sizeof(minimo));
  agregar(txt, sizeof(txt), "🏁 Terminó la juntada #%u\n", jun.numero);
  agregar(txt, sizeof(txt), "⏱ Duración: %s (mínimo %s: %s)\n", tiempo, minimo,
          jun.duracionMs >= DURACION_MINIMA_MS ? "cumplido ✅" : "no cumplido ❌");
  agregar(txt, sizeof(txt), "👥 Participantes:");
  bool primeroDeLista = true;
  for (uint8_t c = 0; c < CANT_CASILLEROS; c++) {
    if (!jun.participante[c]) continue;
    agregar(txt, sizeof(txt), "%s %s", primeroDeLista ? "" : ",", NOMBRE_CASILLERO[c]);
    primeroDeLista = false;
  }
  if (jun.primero >= 0) agregar(txt, sizeof(txt), "\n📵 Primero en levantarlo: %s", NOMBRE_CASILLERO[jun.primero]);
  else                  agregar(txt, sizeof(txt), "\n🏆 ¡Nadie levantó el celular!");
  agregar(txt, sizeof(txt), "\n🔁 Retiros: %u", jun.totalRetiros);
  if (jun.totalRetiros > 0) {
    agregar(txt, sizeof(txt), " (");
    primeroDeLista = true;
    for (uint8_t c = 0; c < CANT_CASILLEROS; c++) {
      if (!jun.participante[c]) continue;
      agregar(txt, sizeof(txt), "%s%s: %u", primeroDeLista ? "" : ", ", NOMBRE_CASILLERO[c], jun.retirosPorCasillero[c]);
      primeroDeLista = false;
    }
    agregar(txt, sizeof(txt), ")");
  }
  bool alguno = false;
  for (uint8_t c = 0; c < CANT_CASILLEROS; c++) {
    if (!jun.sinSensor[c] && !jun.fallaAlIniciar[c]) continue;
    agregar(txt, sizeof(txt), alguno ? ", %s" : "\n⚠️ Sensores en falla: %s", NOMBRE_CASILLERO[c]);
    alguno = true;
  }
  agregar(txt, sizeof(txt), "\n🚩 Bandera: %s", textoBandera(jun.bandera));
  telegramEnviar(txt);
}

static void terminarJuntada(uint32_t ahora) {
  jun.duracionMs = ahora - jun.inicioMs;
  for (uint8_t c = 0; c < CANT_CASILLEROS; c++) afuera[c] = false;
  banderaBajar();
  buzzerSecuencia(SONIDO_FIN, LARGO(SONIDO_FIN));
  cambiarEstado(EstadoCaja::FIN, ahora);
  imprimirResumen();
  enviarResumenTelegram();
}

// ---------------------------------------------------------------
// Salidas por estado (LED y OLED)
// ---------------------------------------------------------------
static void actualizarLeds(uint32_t ahora) {
  bool parpadeo = ((ahora / PARPADEO_MS) % 2) == 0;
  switch (estado) {
    case EstadoCaja::ESPERA:
    case EstadoCaja::LISTA:
      for (uint8_t c = 0; c < CANT_CASILLEROS; c++) {
        EstadoCasillero e = estadoCasillero(c);
        ledCasillero(c, !CASILLERO_ASIGNADO[c]          ? COLOR_APAGADO
                      : e == EstadoCasillero::FALLA      ? COLOR_FALLA
                      : e == EstadoCasillero::OCUPADO    ? COLOR_OCUPADO
                                                         : COLOR_LIBRE);
      }
      ledsAmbiente(estado == EstadoCaja::LISTA ? COLOR_LISTA : COLOR_APAGADO);
      break;

    case EstadoCaja::JUNTADA:
    case EstadoCaja::ALERTA:
      for (uint8_t c = 0; c < CANT_CASILLEROS; c++) {
        uint32_t color = COLOR_APAGADO;
        if (jun.participante[c]) {
          if (jun.sinSensor[c])  color = COLOR_FALLA;
          else if (afuera[c])    color = parpadeo ? COLOR_ALERTA : COLOR_APAGADO;
          else                   color = COLOR_OCUPADO;
        } else if (CASILLERO_ASIGNADO[c] && estadoCasillero(c) == EstadoCasillero::FALLA) {
          color = COLOR_FALLA;   // no participa porque su sensor está en falla: se sigue viendo
        }
        ledCasillero(c, color);
      }
      if (estado == EstadoCaja::ALERTA) ledsAmbiente(parpadeo ? COLOR_ALERTA : COLOR_APAGADO);
      else                              ledsAmbiente(COLOR_AMBIENTE);
      if (jun.bandera == EstadoBandera::FALLA) ledPixel(PIXEL_FALLA_BANDERA, COLOR_FALLA);
      break;

    case EstadoCaja::FIN:
      for (uint8_t c = 0; c < CANT_CASILLEROS; c++)
        ledCasillero(c, (jun.primero == (int8_t)c) ? COLOR_ALERTA : COLOR_APAGADO);
      ledsAmbiente(COLOR_FIN);
      if (jun.bandera == EstadoBandera::FALLA) ledPixel(PIXEL_FALLA_BANDERA, COLOR_FALLA);
      break;
  }
}

static const char* etiquetaCasillero(uint8_t c) {
  if (!CASILLERO_ASIGNADO[c]) return "  ";
  switch (estadoCasillero(c)) {
    case EstadoCasillero::OCUPADO: return "OK";
    case EstadoCasillero::FALLA:   return "FALLA";
    default:                       return "--";
  }
}

// El OLED no tiene tildes en su fuente: textos sin acentos.
static void actualizarOled(uint32_t ahora) {
  if (!oledTocaRefrescar(ahora)) return;
  char tiempo[12];
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setCursor(0, 0);

  switch (estado) {
    case EstadoCaja::ESPERA: {
      oled.println(F("CAJA DE LA JUNTADA"));
      oled.println(F("Deja tu celular"));
      oled.println();
      oled.printf("C1:%-5s C2:%s\n", etiquetaCasillero(0), etiquetaCasillero(1));
      oled.printf("C3:%-5s C4:%s\n", etiquetaCasillero(2), etiquetaCasillero(3));
      if (haySensorEnFalla()) oledListaSinSensor();   // S1.7: aviso fijo mientras dure la falla
      else                    oled.println();
      uint8_t n = ocupadosAsignados();
      oled.printf("Celulares: %u\n", n);
      if (!hayMensaje(ahora) && n >= MIN_PARTICIPANTES) oled.print(F("Boton: empezar"));
      break;
    }
    case EstadoCaja::LISTA:
      oled.setTextSize(2);
      oled.println(F("LISTOS!"));
      oled.setTextSize(1);
      oled.printf("\n%u celulares\n\n", ocupadosAsignados());
      oled.println(F("Apreta el boton"));
      oled.print(F("para empezar"));
      break;

    case EstadoCaja::JUNTADA:
      oled.printf("JUNTADA (%u)", jun.cantParticipantes);
      if (jun.bandera == EstadoBandera::FALLA)         oled.print(F(" SIN BAND."));
      else if (jun.bandera == EstadoBandera::SUBIENDO) oled.print(F(" bandera"));
      oled.println();
      oled.setCursor(0, 16);
      oled.setTextSize(2);
      formatoTiempo(ahora - jun.inicioMs, tiempo, sizeof(tiempo));
      oled.print(tiempo);
      oled.setTextSize(1);
      if (haySensorEnFalla()) { oled.setCursor(0, 32); oledListaSinSensor(); }   // S1.7
      oled.setCursor(0, 40);
      if (jun.primero >= 0) oled.printf("Primero: C%u\n", jun.primero + 1);
      else                  oled.println(F("Nadie lo levanto"));
      if (!hayMensaje(ahora)) oled.print(F("Fin: mantener boton"));
      break;

    case EstadoCaja::ALERTA:
      oled.setTextSize(2);
      oled.println(F("ALERTA!"));
      oled.setTextSize(1);
      oled.setCursor(0, 20);
      oled.print(F("Afuera:"));
      for (uint8_t c = 0; c < CANT_CASILLEROS; c++) if (afuera[c]) oled.printf(" C%u", c + 1);
      oled.println();
      if (jun.primero >= 0) oled.printf("Primero: C%u\n", jun.primero + 1);
      oled.setCursor(0, 42);
      formatoTiempo(ahora - jun.inicioMs, tiempo, sizeof(tiempo));
      oled.printf("Juntada %s", tiempo);
      if (jun.bandera == EstadoBandera::FALLA) { oled.setCursor(0, 56); oled.print(F("SIN BANDERA")); }
      break;

    case EstadoCaja::FIN: {
      oled.println(F("FIN - RESUMEN"));
      formatoTiempo(jun.duracionMs, tiempo, sizeof(tiempo));
      oled.printf("Duracion %s\n", tiempo);
      if (jun.primero >= 0) oled.printf("Primero: C%u\n", jun.primero + 1);
      else                  oled.println(F("Nadie lo levanto!"));
      oled.printf("Retiros: %u\n", jun.totalRetiros);
      oled.printf("Minimo: %s\n", jun.duracionMs >= DURACION_MINIMA_MS ? "cumplido" : "no cumplido");
      oled.printf("Bandera: %s\n", jun.bandera == EstadoBandera::CONFIRMADA ? "OK"
                                 : jun.bandera == EstadoBandera::FALLA      ? "FALLA" : "-");
      bool alguno = false;
      for (uint8_t c = 0; c < CANT_CASILLEROS; c++) if (jun.sinSensor[c] || jun.fallaAlIniciar[c]) alguno = true;
      if (alguno) {
        oled.print(F("Sin sensor:"));
        for (uint8_t c = 0; c < CANT_CASILLEROS; c++)
          if (jun.sinSensor[c] || jun.fallaAlIniciar[c]) oled.printf(" C%u", c + 1);
        oled.println();
      }
      if (!hayMensaje(ahora)) { oled.setCursor(0, 56); oled.print(F("Boton: nueva juntada")); }
      break;
    }
  }

  if (hayMensaje(ahora)) {           // aviso corto en la última línea
    oled.setTextSize(1);
    oled.fillRect(0, 56, OLED_ANCHO, 8, SSD1306_BLACK);
    oled.setCursor(0, 56);
    oled.print(mensaje);
  }
  oled.display();
}

// ---------------------------------------------------------------
// Máquina de estados
// ---------------------------------------------------------------
void estadosIniciar() {
  estado = EstadoCaja::ESPERA;
  entradaEstado = millis();
  memset(&jun, 0, sizeof(jun));
  jun.primero = -1;
  for (uint8_t c = 0; c < CANT_CASILLEROS; c++) { previo[c] = estadoCasillero(c); afuera[c] = false; }
  Serial.println(F("Estado inicial: ESPERA"));
}

void estadosActualizar(uint32_t ahora) {
  // Los eventos del botón se leen SIEMPRE, para que no queden guardados y se
  // apliquen tarde en otro estado.
  bool toqueCorto = botonInicio.flancoSoltadoCorto();
  bool pulsLarga  = botonInicio.pulsacionLarga();
  botonInicio.flancoPresion();   // no se usa: se descarta

  revisarFallasSensor(ahora);    // S1.7: en cualquier estado

  switch (estado) {
    case EstadoCaja::ESPERA:
      if (toqueCorto) {
        if (ocupadosAsignados() >= MIN_PARTICIPANTES) {
          iniciarJuntada(ahora);
        } else {
          buzzerSecuencia(SONIDO_ERROR, LARGO(SONIDO_ERROR));
          char txt[24];
          snprintf(txt, sizeof(txt), "Minimo %u celulares", MIN_PARTICIPANTES);
          mostrarMensaje(txt, ahora);
          Serial.println(F("[AVISO] Inicio rechazado: faltan celulares"));
        }
      } else if (todosListos()) {
        buzzerSecuencia(SONIDO_LISTA, LARGO(SONIDO_LISTA));
        cambiarEstado(EstadoCaja::LISTA, ahora);
      }
      break;

    case EstadoCaja::LISTA:
      if (toqueCorto)          iniciarJuntada(ahora);
      else if (!todosListos()) cambiarEstado(EstadoCaja::ESPERA, ahora);
      break;

    case EstadoCaja::JUNTADA:
      revisarBandera(ahora);
      if (pulsLarga) {
        terminarJuntada(ahora);
      } else if (revisarParticipantes(ahora) > 0) {
        alertaDesde = ahora;
        buzzerSecuencia(SONIDO_ALERTA, LARGO(SONIDO_ALERTA));
        cambiarEstado(EstadoCaja::ALERTA, ahora);
      }
      break;

    case EstadoCaja::ALERTA:
      revisarBandera(ahora);
      if (pulsLarga) {
        terminarJuntada(ahora);
        break;
      }
      if (revisarParticipantes(ahora) > 0) alertaDesde = ahora;   // otro retiro: vuelve a sonar
      if (participantesAfuera() == 0) {
        buzzerSecuencia(SONIDO_REPUESTO, LARGO(SONIDO_REPUESTO));
        cambiarEstado(EstadoCaja::JUNTADA, ahora);
      } else if (ahora - alertaDesde < ALERTA_SONIDO_MS && !buzzerOcupado()) {
        buzzerSecuencia(SONIDO_ALERTA, LARGO(SONIDO_ALERTA));
      }
      break;

    case EstadoCaja::FIN:
      if (toqueCorto) cambiarEstado(EstadoCaja::ESPERA, ahora);
      break;
  }

  for (uint8_t c = 0; c < CANT_CASILLEROS; c++) previo[c] = estadoCasillero(c);
  (void)entradaEstado;

  actualizarLeds(ahora);
  actualizarOled(ahora);
}

// =================================================================
// 6. Red  (red.cpp)
// =================================================================
// red.cpp — WiFi no bloqueante y Telegram (Sprint 2, v0.6).
//
// Diseño:
//  - WiFi: WiFi.begin() una vez y después solo se mira WiFi.status() en cada vuelta. Sin
//    conexión, se reintenta cada WIFI_REINTENTO_MS. La caja sigue funcionando igual.
//  - Telegram: el loop() nunca espera a internet. telegramEnviar() deja el mensaje en una
//    cola de FreeRTOS y vuelve enseguida; una tarea aparte (núcleo 0) lo manda por HTTPS a
//    la API de bots (DEC-017). Un envío tarda 1–3 s: si lo hiciera el loop(), la caja se
//    congelaría (sin sonido, LED ni lecturas) en plena alerta.
//  - Sin WiFi o con error de red, el mensaje espera en la cola y se reintenta: es la base
//    del escenario "caída del WiFi". Si la API responde un error (p. ej. token inválido),
//    se reintenta TELEGRAM_REINTENTOS veces y se descarta, para no trabar la cola.
//  - Simplificación de la simulación: setInsecure() no valida el certificado de Telegram.
//    En producción va validación de certificado (limitación documentada).

static QueueHandle_t colaTelegram = nullptr;
static bool     telegramActivo = false;
static bool     wifiAntes = false;
static bool     caidaSimulada = false;
static uint32_t ultimoIntentoWifi = 0;

static bool telegramConfigurado() {
  return strcmp(TELEGRAM_TOKEN, "TU_TOKEN") != 0 && strcmp(TELEGRAM_CHAT_ID, "TU_CHAT_ID") != 0;
}

bool redConectada() { return WiFi.status() == WL_CONNECTED; }

// Escapa el texto para meterlo en un JSON (comillas, barras y saltos de línea).
static void escaparJson(const char* entrada, char* salida, size_t largo) {
  size_t j = 0;
  for (size_t i = 0; entrada[i] != '\0' && j + 3 < largo; i++) {
    unsigned char ch = (unsigned char)entrada[i];
    if (ch == '"' || ch == '\\') { salida[j++] = '\\'; salida[j++] = (char)ch; }
    else if (ch == '\n')         { salida[j++] = '\\'; salida[j++] = 'n'; }
    else if (ch == '\t')         { salida[j++] = '\\'; salida[j++] = 't'; }
    else if (ch < 0x20)          { /* otros caracteres de control: se omiten */ }
    else                         { salida[j++] = (char)ch; }   // UTF-8 (tildes, emojis) pasa tal cual
  }
  salida[j] = '\0';
}

// POST a sendMessage. Devuelve el código HTTP (200 = OK) o un número negativo si falló la conexión.
// Solo la llama la tarea de Telegram (los búferes estáticos no se comparten).
static int telegramPost(const char* texto) {
  static char escapado[TELEGRAM_TEXTO_MAX * 2];
  static char cuerpo[TELEGRAM_TEXTO_MAX * 2 + 96];
  escaparJson(texto, escapado, sizeof(escapado));
  snprintf(cuerpo, sizeof(cuerpo), "{\"chat_id\":\"%s\",\"text\":\"%s\"}", TELEGRAM_CHAT_ID, escapado);

  WiFiClientSecure cliente;
  cliente.setInsecure();                       // solo simulación (ver arriba)
  HTTPClient http;
  http.setTimeout(10000);
  if (!http.begin(cliente, "https://api.telegram.org/bot" TELEGRAM_TOKEN "/sendMessage")) return -100;
  http.addHeader("Content-Type", "application/json");
  int codigo = http.POST((uint8_t*)cuerpo, strlen(cuerpo));
  http.end();
  return codigo;
}

// Tarea de FreeRTOS: saca los mensajes de la cola de a uno y los manda, en orden.
static void tareaTelegram(void* parametro) {
  (void)parametro;
  static MensajeTelegram m;
  for (;;) {
    if (xQueueReceive(colaTelegram, &m, portMAX_DELAY) != pdTRUE) continue;
    uint8_t erroresApi = 0;
    for (;;) {
      if (WiFi.status() != WL_CONNECTED) {     // sin red: el mensaje espera, no se pierde
        vTaskDelay(pdMS_TO_TICKS(1000));
        continue;
      }
      int codigo = telegramPost(m.texto);
      if (codigo == 200) {
        Serial.printf("[TELEGRAM] Enviado (quedan %u en cola)\n", (unsigned)uxQueueMessagesWaiting(colaTelegram));
        break;
      }
      if (codigo < 0 || codigo == 429 || codigo >= 500) {   // red o servidor: se reintenta sin límite
        Serial.printf("[TELEGRAM] Error de red/servidor (%d): reintento en %lu ms\n",
                      codigo, (unsigned long)TELEGRAM_ESPERA_REINTENTO_MS);
      } else if (++erroresApi >= TELEGRAM_REINTENTOS) {     // 4xx: token o chat_id mal
        Serial.printf("[TELEGRAM] Error HTTP %d: mensaje descartado (revisar TOKEN y CHAT_ID)\n", codigo);
        break;
      } else {
        Serial.printf("[TELEGRAM] Error HTTP %d (intento %u de %u)\n", codigo, erroresApi, TELEGRAM_REINTENTOS);
      }
      vTaskDelay(pdMS_TO_TICKS(TELEGRAM_ESPERA_REINTENTO_MS));
    }
    vTaskDelay(pdMS_TO_TICKS(300));            // no saturar la API con ráfagas
  }
}

void telegramEnviar(const char* texto) {
  if (!telegramActivo) {
    Serial.printf("[TELEGRAM] (sin token, no se envia)\n%s\n", texto);
    return;
  }
  static MensajeTelegram m;                    // solo lo usa el loop()
  strncpy(m.texto, texto, sizeof(m.texto) - 1);
  m.texto[sizeof(m.texto) - 1] = '\0';
  if (xQueueSend(colaTelegram, &m, 0) == pdTRUE) {
    Serial.printf("[TELEGRAM] En cola (%u)%s\n", (unsigned)uxQueueMessagesWaiting(colaTelegram),
                  redConectada() ? "" : " - sin WiFi: se envia al reconectar");
  } else {
    Serial.println(F("[TELEGRAM] Cola llena: mensaje descartado"));
  }
}

void redIniciar() {
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS, WIFI_CANAL);   // no espera: el estado se revisa en redActualizar()
  ultimoIntentoWifi = millis();
  Serial.println(F("[WIFI] Conectando a " WIFI_SSID "..."));

  telegramActivo = telegramConfigurado();
  if (telegramActivo) {
    colaTelegram = xQueueCreate(TELEGRAM_COLA, sizeof(MensajeTelegram));
    xTaskCreatePinnedToCore(tareaTelegram, "telegram", 12288, nullptr, 1, nullptr, 0);
    Serial.println(F("[TELEGRAM] Activo"));
  } else {
    Serial.println(F("[TELEGRAM] Sin TOKEN/CHAT_ID en config: los mensajes se muestran solo por Serial"));
  }
  Serial.println(F("[WIFI] Simular caida: apretar el boton Caida WiFi"));
}

// Escenario "caída del WiFi": cada vez que se aprieta el pulsador "Caida WiFi" (GPIO 26)
// se corta o se restablece la conexión.
static void fijarCaidaWifi(bool caida, uint32_t ahora) {
  caidaSimulada = caida;
  if (caida) {
    WiFi.setAutoReconnect(false);   // que el ESP32 no se vuelva a conectar solo
    WiFi.disconnect();              // se desconecta de la red (Wokwi-GUEST)
    Serial.println(F("[WIFI] Caida SIMULADA: la caja sigue; Telegram queda en cola"));
    avisoOled("WIFI CAIDO (simulado)");
  } else {
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASS, WIFI_CANAL);   // vuelve a pedir la conexión
    ultimoIntentoWifi = ahora;
    Serial.println(F("[WIFI] Fin de la caida simulada: reconectando"));
    avisoOled("WIFI: reconectando");
  }
}

void redActualizar(uint32_t ahora) {
  if (botonWifi.flancoPresion())
    fijarCaidaWifi(!caidaSimulada, ahora);

  bool conectado = redConectada();
  if (conectado != wifiAntes) {
    wifiAntes = conectado;
    if (conectado) {
      Serial.printf("[WIFI] Conectado (IP %s)\n", WiFi.localIP().toString().c_str());
      avisoOled("WIFI conectado");
    } else {
      Serial.println(F("[WIFI] Sin conexion: la caja sigue funcionando"));
      if (!caidaSimulada) avisoOled("SIN WIFI");
      ultimoIntentoWifi = ahora;
    }
  }

  if (!conectado && !caidaSimulada && ahora - ultimoIntentoWifi >= WIFI_REINTENTO_MS) {
    ultimoIntentoWifi = ahora;
    Serial.println(F("[WIFI] Reintentando conexion..."));
    WiFi.begin(WIFI_SSID, WIFI_PASS, WIFI_CANAL);
  }
}

// =================================================================
// 7. Modo prueba de circuito  (prueba_circuito.cpp)
// =================================================================
// prueba_circuito.cpp — Verifica cada componente del circuito y sirve para medir la curva de los LDR.
//
// Qué hace:
//  - Cada 500 ms imprime por Serial las 4 lecturas promediadas y el estado de las entradas.
//  - Píxeles 0–3: estado CONFIRMADO de cada casillero (verde ocupado, azul libre,
//    magenta falla/sin sensor) y amarillo mientras un cambio espera confirmación
//    (1 s para ocupado/libre, 2 s para falla). Píxeles 4–7: ambiente cálido.
//  - Cada cambio confirmado se informa por Serial (lo imprime sensores.cpp).
//  - Botón Inicio/Fin: alterna la bandera entre baja y alzada, con un pitido.
//  - Botón "Caida WiFi": pitido y aviso por Serial al apretarlo.
//  - Fin de carrera: avisa por Serial cada vez que cambia.
//  - OLED: lecturas y entradas.

static constexpr uint32_t PERIODO_REPORTE_MS = 500;
static uint32_t ultimoReporte = 0;
static bool     finCarreraAnterior = false;

static uint32_t colorSegunEstado(uint8_t c) {
  if (cambioPendiente(c)) return COLOR_DUDOSO;
  switch (estadoCasillero(c)) {
    case EstadoCasillero::OCUPADO: return COLOR_OCUPADO;
    case EstadoCasillero::FALLA:   return COLOR_FALLA;
    default:                       return COLOR_LIBRE;
  }
}

static char letraEstado(uint8_t c) {   // O = ocupado, L = libre, F = falla; minúscula = cambio pendiente
  char l = estadoCasillero(c) == EstadoCasillero::OCUPADO ? 'O'
         : estadoCasillero(c) == EstadoCasillero::FALLA   ? 'F' : 'L';
  return cambioPendiente(c) ? (char)(l + 32) : l;
}

void pruebaIniciar() {
  Serial.println(F("=== MODO PRUEBA DE CIRCUITO ==="));
  Serial.println(F("Columnas: tiempo | C1..C4 lectura (0-4095) y estado | inicio | boton wifi | fin de carrera | servo"));
  Serial.println(F("Estado: O ocupado, L libre, F falla; en minuscula = cambio esperando confirmacion"));
  ledsAmbiente(COLOR_AMBIENTE);
  finCarreraAnterior = finDeCarreraCerrado();
}

void pruebaActualizar(uint32_t ahora) {
  // --- Píxeles según el estado de cada casillero ---
  for (uint8_t c = 0; c < CANT_CASILLEROS; c++) {
    ledCasillero(c, colorSegunEstado(c));
    casilleroCambio(c);   // en el modo prueba solo se informa (sensores.cpp ya lo imprime)
  }

  // --- Botón de inicio: alterna la bandera ---
  if (botonInicio.flancoPresion()) {
    if (banderaArriba()) banderaBajar(); else banderaLevantar();
    buzzerBeep(1000, 100);
    Serial.printf(">> Inicio/Fin presionado: servo %s\n", banderaArriba() ? "BANDERA ARRIBA (90)" : "BANDERA BAJA (0)");
  }
  if (botonInicio.pulsacionLarga()) {
    buzzerBeep(600, 300);
    Serial.println(F(">> Inicio/Fin: pulsacion larga detectada"));
  }

  // --- Botón de caída del WiFi ---
  if (botonWifi.flancoPresion()) {
    buzzerBeep(1500, 80);
    Serial.println(F(">> Boton Caida WiFi presionado"));
  }

  // --- Fin de carrera ---
  bool fc = finDeCarreraCerrado();
  if (fc != finCarreraAnterior) {
    finCarreraAnterior = fc;
    Serial.printf(">> Fin de carrera: %s\n", fc ? "CERRADO (bandera arriba)" : "ABIERTO");
  }

  // --- Reporte periódico por Serial ---
  if (ahora - ultimoReporte >= PERIODO_REPORTE_MS) {
    ultimoReporte = ahora;
    Serial.printf("t=%6.1fs | C1=%4d %c C2=%4d %c C3=%4d %c C4=%4d %c | inicio=%d | wifi=%d | fc=%s | servo=%s\n",
                  ahora / 1000.0,
                  ldrPromedio(0), letraEstado(0), ldrPromedio(1), letraEstado(1),
                  ldrPromedio(2), letraEstado(2), ldrPromedio(3), letraEstado(3),
                  botonInicio.presionado(),
                  botonWifi.presionado(),
                  fc ? "CERRADO" : "ABIERTO",
                  banderaArriba() ? "ARRIBA" : "BAJA");
  }

  // --- OLED ---
  if (oledTocaRefrescar(ahora)) {
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setCursor(0, 0);
    oled.println(F("PRUEBA DE CIRCUITO"));
    oled.printf("C1 %4d %c  C2 %4d %c\n", ldrPromedio(0), letraEstado(0), ldrPromedio(1), letraEstado(1));
    oled.printf("C3 %4d %c  C4 %4d %c\n", ldrPromedio(2), letraEstado(2), ldrPromedio(3), letraEstado(3));
    oled.printf("Boton WiFi: %s\n", botonWifi.presionado() ? "apretado" : "-");
    oled.printf("Inicio: %s\n", botonInicio.presionado() ? "apretado" : "-");
    oled.printf("Fin carrera: %s\n", fc ? "CERRADO" : "ABIERTO");
    oled.printf("Bandera: %s\n", banderaArriba() ? "ARRIBA" : "BAJA");
    oled.display();
  }
}

// =================================================================
// 8. setup() y loop()
// Regla: nada de delay(). Cada módulo tiene su actualizar(ahora).
// =================================================================
void setup() {
  Serial.begin(SERIAL_BAUDIOS);
  Serial.println();
  Serial.println(F("Caja de la juntada - Sprint 2 - firmware " FIRMWARE_VERSION));

  sensoresIniciar();
  actuadoresIniciar();

#if MODO_PRUEBA_CIRCUITO
  pruebaIniciar();
#else
  estadosIniciar();
  redIniciar();
#endif
}

void loop() {
  uint32_t ahora = millis();

  sensoresActualizar(ahora);

#if MODO_PRUEBA_CIRCUITO
  pruebaActualizar(ahora);
#else
  estadosActualizar(ahora);
  redActualizar(ahora);
#endif

  actuadoresActualizar(ahora);
}