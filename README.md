# Sistema de alerta temprana bidireccional para la seguridad peatonal y el control de velocidad en zonas universitarias

Proyecto Integrador II · Ingeniería de Datos e Inteligencia Artificial · Universidad Santo Tomás, Seccional Tunja

**Estudiantes:** Miguel Patiño, Camilo Hernández, Julián Andrés Hernández, Nicolás Valbuena  
**Docentes:** Zalma Valentina Moreno Galeano, Fernando Jiménez Díaz, Yashin Alberto Rueda Matos

## Objetivo general
Desarrollar un sistema de alerta temprana para la seguridad peatonal y el control de velocidad, dirigido a peatones y conductores frente a la USTA Tunja, mediante visión por computador, homografía y un ESP32, hasta un prototipo probado en la vía, evaluado por su latencia, el error de velocidad y la detección de peatones.

## ¿Cómo funciona?
1. Una cámara (webcam USB o celular con IP Webcam) envía video al PC.
2. El PC detecta peatones y vehículos con **YOLOv8n** y los sigue con **ByteTrack**.
3. Una **homografía** convierte la posición de cada vehículo de píxeles a metros sobre la vía y calcula la velocidad: `v = (Δd / Δt) × 3,6` km/h.
4. El límite se ajusta con el clima (IDEAM por datos.gov.co, con respaldo de **Open-Meteo**): 30 km/h en seco y 20 km/h con lluvia.
5. El PC muestra los avisos en pantalla y envía el estado al **ESP32** por USB, que enciende los LED y la pantalla OLED.

| Estado | Significado | LED |
|---|---|---|
| 0 | Vía normal | Verde fijo |
| 1 | Peatón en la zona | Verde intermitente lento |
| 2 | Exceso de velocidad | Rojo intermitente rápido |
| 3 | Peligro (peatón + exceso) | Rojo fijo |

Si se pierde el enlace con el PC por más de 3 s, el ESP32 apaga las alertas y muestra "SIN ENLACE".

## Archivos
| Archivo | Descripción |
|---|---|
| `sistema_interfaz.py` | Programa del PC: video, detección, velocidad, clima, interfaz y envío al ESP32 |
| `firmware_esp32.ino` | Firmware del ESP32: LED y pantalla OLED SH1106 1,3" (I2C) |

## Uso
```bash
pip install opencv-python numpy requests ultralytics pyserial
python sistema_interfaz.py camaras    # ver qué número tiene la cámara
python sistema_interfaz.py calibrar   # clic en las 4 esquinas del rectángulo medido en el piso
python sistema_interfaz.py clima      # probar la consulta del clima en Tunja
python sistema_interfaz.py            # ejecutar el sistema
```
Teclas: `ESC` salir · `R` girar imagen · `S` sonido · `L` lluvia/seco/automático.

### ESP32
- Librería: **U8g2** (Arduino IDE).
- Conexiones: OLED SDA → GPIO 21, SCK → GPIO 22, VCC → 3V3, GND → GND; LED verde → GPIO 4, LED rojo → GPIO 5, LED de enlace → GPIO 2.
- Protocolo serial (115200 baudios): `estado,velocidad,limite,lluvia` (ej. `2,43,30,0`).

> El prototipo no sustituye la señalización vial oficial.
