#include <U8g2lib.h>
#include <Wire.h>

// Pantalla OLED 1.3" I2C (SDA = GPIO 21, SCK = GPIO 22, VCC = 3V3, GND = GND)
U8G2_SH1106_128X64_NONAME_F_HW_I2C pantalla(U8G2_R0, U8X8_PIN_NONE);
// U8G2_SSD1306_128X64_NONAME_F_HW_I2C pantalla(U8G2_R0, U8X8_PIN_NONE);   // si tu chip es SSD1306

#define PIN_LED_VERDE 4
#define PIN_LED_ROJO  5
#define PIN_LED_ENLACE 2

const unsigned long BAUDIOS = 115200;
const unsigned long TIMEOUT_MS = 3000;
const unsigned long PERIODO_RAPIDO_MS = 150;
const unsigned long PERIODO_LENTO_MS = 500;
const unsigned long PERIODO_PANTALLA_MS = 150;

int estado = 0, velocidad = 0, limite = 30, lluvia = 0;
unsigned long ultimoMensaje = 0, ultimaPantalla = 0;
bool hayEnlace = false;
char linea[40];
int n = 0;

void procesarLinea() {
  linea[n] = 0;
  int e, v, l, r;
  int k = sscanf(linea, "%d,%d,%d,%d", &e, &v, &l, &r);
  if (k >= 1 && e >= 0 && e <= 3) {
    estado = e;
    if (k >= 2) velocidad = v;
    if (k >= 3) limite = l;
    if (k >= 4) lluvia = r;
    ultimoMensaje = millis();
    hayEnlace = true;
  }
}

void dibujar() {
  pantalla.clearBuffer();
  if (!hayEnlace) {
    pantalla.setFont(u8g2_font_helvB12_tr);
    pantalla.drawStr(8, 28, "SIN ENLACE");
    pantalla.setFont(u8g2_font_6x12_tr);
    pantalla.drawStr(8, 48, "Conecta el PC (USB)");
    pantalla.sendBuffer();
    return;
  }
  const char *titulo = "VIA NORMAL";
  if (estado == 1) titulo = "PEATON EN ZONA";
  else if (estado == 2) titulo = "EXCESO VELOCIDAD";
  else if (estado == 3) titulo = "PELIGRO!";

  if (estado >= 2) {
    pantalla.drawBox(0, 0, 128, 16);
    pantalla.setDrawColor(0);
  }
  pantalla.setFont(u8g2_font_helvB10_tr);
  int w = pantalla.getStrWidth(titulo);
  pantalla.drawStr((128 - w) / 2, 13, titulo);
  pantalla.setDrawColor(1);

  char buf[16];
  pantalla.setFont(u8g2_font_logisoso28_tn);
  snprintf(buf, sizeof(buf), "%d", velocidad);
  pantalla.drawStr(4, 52, buf);
  pantalla.setFont(u8g2_font_6x12_tr);
  pantalla.drawStr(80, 36, "km/h");
  snprintf(buf, sizeof(buf), "Lim %d", limite);
  pantalla.drawStr(80, 50, buf);
  if (lluvia) pantalla.drawStr(80, 63, "LLUVIA");
  pantalla.sendBuffer();
}

void setup() {
  Serial.begin(BAUDIOS);
  pinMode(PIN_LED_VERDE, OUTPUT);
  pinMode(PIN_LED_ROJO, OUTPUT);
  pinMode(PIN_LED_ENLACE, OUTPUT);
  Wire.begin(21, 22);
  pantalla.begin();
  dibujar();
}

void loop() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') { procesarLinea(); n = 0; }
    else if (c != '\r' && n < (int)sizeof(linea) - 1) linea[n++] = c;
  }
  if (hayEnlace && millis() - ultimoMensaje > TIMEOUT_MS) {
    hayEnlace = false;
    estado = 0; velocidad = 0;
  }

  unsigned long t = millis();
  bool rapido = ((t / PERIODO_RAPIDO_MS) % 2) == 0;
  bool lento  = ((t / PERIODO_LENTO_MS) % 2) == 0;
  bool verde = false, rojo = false;
  if (hayEnlace) {
    if (estado == 0) verde = true;
    else if (estado == 1) verde = lento;
    else if (estado == 2) rojo = rapido;
    else rojo = true;
  }
  digitalWrite(PIN_LED_VERDE, verde);
  digitalWrite(PIN_LED_ROJO, rojo);
  digitalWrite(PIN_LED_ENLACE, hayEnlace ? HIGH : LOW);

  if (t - ultimaPantalla >= PERIODO_PANTALLA_MS) {
    ultimaPantalla = t;
    dibujar();
  }
}
