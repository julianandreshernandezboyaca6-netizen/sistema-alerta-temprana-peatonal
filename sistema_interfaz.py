"""
sistema_interfaz.py - Medidor de velocidad con INTERFAZ EN EL COMPUTADOR (sin depender de LEDs)

Muestra el video con recuadros, un panel lateral con contadores de peatones y vehiculos,
un cartel de aviso (atencion / exceso de velocidad / peligro), una lista de avisos recientes
y un pitido del computador. El ESP32 (LED) es opcional: USAR_ESP32 = False por defecto.

Uso:    python sistema_interfaz.py
        python sistema_interfaz.py calibrar      (clic en las 4 esquinas del rectangulo medido)
        python sistema_interfaz.py clima         (prueba la consulta del clima en Tunja)
Teclas: ESC = salir   R = girar la imagen 90 grados   S = sonido on/off   L = forzar lluvia/seco/auto
Requiere: pip install opencv-python numpy requests ultralytics   (y pyserial solo si USAR_ESP32)
"""
import csv
import math
import queue
import sys
import threading
import time
from collections import defaultdict, deque

import cv2
import numpy as np
import requests

try:
    import winsound                      # pitidos del computador (solo Windows)
except ImportError:
    winsound = None

# =====================================================================
#                          1. PARAMETROS
# =====================================================================
# ---- 1.1 Red ----
CAM_URL = 2   # webcam USB conectada al PC. Si sale negra o es la camara del PC, prueba 1.
# Camara conectada por cable al PC (webcam USB):
# pon un NUMERO en vez de la direccion:  CAM_URL = 0   (1, 2... si el 0 no es la correcta).
# Para ver cual numero es cual:  python sistema_interfaz.py camaras

# ---- 1.2 ESP32 opcional ----
USAR_ESP32 = True        # True = ademas de los avisos en pantalla, manda la alerta al ESP32 por USB
PUERTO_ESP32 = "COM3"   # None = lo busca solo. Si no lo encuentra: "COM3"
BAUDIOS = 115200

# ---- 1.3 Fisica / calibracion ----
LIMITE_KMH = 30          # velocidad maxima permitida para vehiculos
PTS_IMG_EJEMPLO = np.float32([[300, 200], [500, 200], [620, 420], [100, 420]])   # valor de ejemplo (NO calibrado)
PTS_IMG = np.float32([[300, 200], [500, 200], [620, 420], [100, 420]])   # se obtiene con: python sistema_interfaz.py calibrar
PTS_REAL = np.float32([[0, 0], [3, 0], [3, 10], [0, 10]])                # metros reales (cinta)

# ---- 1.4 Deteccion (YOLO) ----
MODELO = "yolov8n.pt"    # modelo YOLO. n = nano (rapido). Mas preciso y mas lento: "yolov8s.pt" o "yolov8m.pt"
CLASES = [0, 2, 3, 5, 7]  # 0=persona 2=carro 3=moto 5=bus 7=camion
NOMBRES = {0: "PERSONA", 2: "CARRO", 3: "MOTO", 5: "BUS", 7: "CAMION"}
CONF_VEHICULO = 0.30     # confianza minima para CARROS/MOTOS/BUSES. Baja = detecta mas (y se equivoca mas). Rango util 0.15 - 0.5
CONF_PERSONA = 0.35      # confianza minima para PERSONAS (mas alta = menos falsos peatones)
CONF_MIN = min(CONF_VEHICULO, CONF_PERSONA)   # el piso que se le pasa a YOLO y al seguimiento (no lo cambies)
IMGSZ = 480              # tamano de analisis. 320 rapido; 480 o 640 mas preciso y lento
MIN_ALTO_PX = 30         # se ignoran recuadros mas bajos que esto (carros muy lejanos: miden mal y llenan la pantalla)
ANCHO = 640              # los cuadros se reducen a este ancho. Si lo cambias, vuelve a calibrar
ROTAR = 0                # 0, 90, 180 o 270 grados (sentido horario). Tambien se cambia con la tecla R.
                         # Si giras la imagen, hay que volver a calibrar.

# ---- 1.5 Medicion de velocidad ----
VENTANA_POS = 15         # posiciones usadas para v = dr/dt. Mas = mas estable, reacciona mas lento
MIN_MUESTRAS = 5         # no mide hasta tener al menos estas posiciones
MIN_DT = 0.2             # s. Intervalo minimo para medir
VENTANA_MEDIANA = 5      # se reporta la mediana de las ultimas N velocidades
UMBRAL_QUIETO_KMH = 2    # por debajo de esto se muestra 0
SOLO_DENTRO_DEL_AREA = True    # True = solo mide objetos dentro del poligono calibrado
OLVIDAR_TRACK_S = 3.0    # s. Borra de memoria lo que ya no se ve

# ---- 1.6 Zona de cruce y avisos ----
USAR_AREA_COMO_ZONA = True     # True  = la zona de cruce es el poligono calibrado (usar en el proyecto final)
                               # False = toda la imagen es la zona (solo para probar sin calibrar)
PERMITIR_SIN_CALIBRAR = False  # False = mientras PTS_IMG sea el de ejemplo, NO se muestran velocidades ni hay exceso/peligro
VEL_PELIGRO_KMH = 10     # un vehiculo en la zona a mas de esta velocidad + un peaton en la zona = PELIGRO
CONFIRMAR_CUADROS = 3    # cuadros seguidos para activar un aviso (evita falsas alarmas)
MANTENER_ALERTA_S = 2.0  # s. El aviso sigue visible este tiempo despues de la ultima vez
LATIDO_S = 0.5           # s. Latido al ESP32 (solo si USAR_ESP32)
SONIDO = True            # pitido del computador en avisos de exceso y peligro (tecla S lo activa/desactiva)

# ---- 1.7 Clima (datos del IDEAM en Tunja) ----
USAR_CLIMA = True              # consulta el clima y baja el limite si esta lloviendo
LIMITE_LLUVIA_KMH = 20         # con lluvia el limite baja a este valor (si es menor que LIMITE_KMH)
CLIMA_CADA_S = 300             # s. Cada cuanto se vuelve a consultar el clima
LLUVIA_MM_MIN = 0.1            # mm. Una observacion con al menos esta precipitacion cuenta como "llueve"
EDAD_MAX_MIN = 120             # min. Si el dato del IDEAM es mas viejo que esto, no se usa (pasa al respaldo)
# IDEAM por datos abiertos de Colombia (datos.gov.co, conjunto "Precipitacion"):
IDEAM_DATASET = "s54a-sgyg"
IDEAM_MUNICIPIO = "TUNJA"      # se buscan las estaciones de este municipio
IDEAM_ESTACION = None          # codigo de una estacion concreta (ej. "24035010"). None = la mas reciente de Tunja.
                               # Para ver las estaciones y sus codigos:  python sistema_interfaz.py clima
COL_MUNICIPIO, COL_ESTACION, COL_NOMBRE = "municipio", "codigoestacion", "nombreestacion"
COL_FECHA, COL_VALOR = "fechaobservacion", "valorobservado"   # nombres de columna (verificar con el comando clima)
USAR_RESPALDO_OPENMETEO = True  # si el IDEAM no responde o su dato es viejo, usa Open-Meteo (sin clave)
TUNJA_LAT, TUNJA_LON = 5.5353, -73.3678

# ---- 1.8 Pantalla y registro ----
PANEL_ANCHO = 340
MOSTRAR_DETALLE = True   # muestra dr y dt de cada objeto
GUARDAR_CSV = True
ARCHIVO_CSV = "mediciones.csv"

# =====================================================================
STOP = False

# colores (BGR)
VERDE = (90, 210, 90)
AZUL = (255, 200, 0)
ROJO = (60, 60, 255)
AMBAR = (0, 190, 255)
GRIS = (150, 150, 150)
BLANCO = (240, 240, 240)
FONDO = (36, 28, 22)


# =====================================================================
#                      2. CAMARA (webcam USB conectada al PC)
# =====================================================================
class MJPEGCam:
    """Lee el video de la camara en un hilo aparte y siempre guarda el cuadro MAS RECIENTE."""

    def __init__(self, url):
        self.url = url
        self.frame = None
        self.frame_id = 0
        self.last_id = 0
        self.stop = False
        threading.Thread(target=self._run_webcam if isinstance(url, int) else self._run, daemon=True).start()

    def _run_webcam(self):
        """Camara conectada al PC (indice 0, 1, 2...). En Windows usa DirectShow, que es el que ven las apps de iPhone."""
        cap = None
        while not self.stop:
            if cap is None or not cap.isOpened():
                cap = cv2.VideoCapture(self.url, cv2.CAP_DSHOW) if sys.platform.startswith("win") else cv2.VideoCapture(self.url)
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                if not cap.isOpened():
                    print(f"[aviso] No se pudo abrir la camara {self.url}. Revisa que la webcam este conectada y que ninguna otra app (Camara de Windows) la este usando, o prueba otro numero (0, 1, 2).")
                    time.sleep(2)
                    continue
            ok, img = cap.read()
            if not ok or img is None:
                time.sleep(0.05)
                continue
            h, w = img.shape[:2]
            if w > ANCHO:
                img = cv2.resize(img, (ANCHO, int(h * ANCHO / w)))
            self.frame = img
            self.frame_id += 1
        if cap is not None:
            cap.release()

    def _run(self):
        while not self.stop:
            try:
                r = requests.get(self.url, stream=True, timeout=5)
                buf = b""
                for chunk in r.iter_content(chunk_size=65536):
                    if self.stop:
                        break
                    buf += chunk
                    b = buf.rfind(b"\xff\xd9")            # fin de la ultima imagen JPEG completa
                    if b == -1:
                        continue
                    a = buf.rfind(b"\xff\xd8", 0, b)      # inicio de esa imagen
                    if a != -1:
                        img = cv2.imdecode(np.frombuffer(buf[a:b + 2], np.uint8), cv2.IMREAD_COLOR)
                        if img is not None:
                            h, w = img.shape[:2]
                            if w > ANCHO:                  # reduce el tamano para ir mas rapido
                                img = cv2.resize(img, (ANCHO, int(h * ANCHO / w)))
                            self.frame = img
                            self.frame_id += 1
                    buf = buf[b + 2:]
            except requests.RequestException:
                time.sleep(1)                              # si se cae la red, reintenta

    def read(self):
        """Espera un cuadro NUEVO (maximo 5 s). Asi nunca procesa el mismo cuadro dos veces."""
        t0 = time.time()
        while self.frame_id == self.last_id and time.time() - t0 < 5:
            time.sleep(0.005)
        self.last_id = self.frame_id
        return self.frame is not None, self.frame

    def release(self):
        self.stop = True

# =====================================================================
#                      3. ENVIO AL ESP32 (opcional)
# =====================================================================
def buscar_puerto_esp32():
    """Busca un puerto serial que parezca un ESP32 (chips CP210x, CH340, FTDI o USB nativo)."""
    from serial.tools import list_ports
    claves = ("CP210", "CH340", "CH910", "USB SERIAL", "USB-SERIAL", "FTDI", "ESP32", "USB JTAG")
    for p in list_ports.comports():
        if any(c in (p.description or "").upper() for c in claves):
            return p.device
    return None


class Notificador:
    """Manda la orden al ESP32 por cable USB (serial), en un hilo aparte para no frenar el video.
    Protocolo: una linea  estado,velocidad,limite,lluvia  (ej. 2,43,30,0). estado: 0 normal, 1 peaton en zona,
    2 exceso de velocidad, 3 peligro (peaton + exceso). El ESP32 la usa para los LED y la pantalla OLED."""

    def __init__(self, puerto, baudios):
        self.ser = None
        self.q = queue.Queue(maxsize=1)
        try:
            import serial                                   # paquete pyserial
            puerto = puerto or buscar_puerto_esp32()
            if puerto is None:
                raise RuntimeError("no se encontro ningun ESP32 conectado por USB")
            self.ser = serial.Serial(puerto, baudios, timeout=1)
            time.sleep(2)                                   # el ESP32 se reinicia al abrir el puerto
            print(f"[ok] ESP32 conectado en {puerto}")
            threading.Thread(target=self._run, daemon=True).start()
        except Exception as e:
            print(f"[aviso] Sin ESP32 ({e}). El programa sigue sin alertas fisicas.")

    def enviar(self, vel):
        if self.ser is None:
            return
        try:
            self.q.get_nowait()          # si habia un mensaje pendiente, lo reemplaza por el nuevo
        except queue.Empty:
            pass
        self.q.put(vel)

    def _run(self):
        while True:
            vel = self.q.get()
            try:
                self.ser.write(f"{vel}\n".encode())
            except Exception:
                pass


# =====================================================================
#                      4. FUNCIONES DE APOYO
# =====================================================================
def a_metros(H, px, py):
    """Homografia: convierte un punto de la imagen (pixeles) a posicion en el piso (metros)."""
    p = cv2.perspectiveTransform(np.float32([[[px, py]]]), H)[0][0]
    return float(p[0]), float(p[1])


def consultar_ideam():
    """Ultima precipitacion de las estaciones del IDEAM en Tunja (datos.gov.co). Devuelve (llueve, fuente, detalle) o None."""
    from datetime import datetime
    where = f"upper({COL_MUNICIPIO}) like '%{IDEAM_MUNICIPIO.upper()}%'"
    if IDEAM_ESTACION:
        where += f" AND {COL_ESTACION}='{IDEAM_ESTACION}'"
    r = requests.get(f"https://www.datos.gov.co/resource/{IDEAM_DATASET}.json",
                     params={"$where": where, "$order": f"{COL_FECHA} DESC", "$limit": 30}, timeout=10)
    r.raise_for_status()
    filas = r.json()
    if not filas:
        return None
    ultima = filas[0][COL_FECHA]
    recientes = [f for f in filas if f[COL_FECHA] == ultima]            # observaciones de la fecha mas nueva
    fecha = datetime.fromisoformat(ultima.split(".")[0])
    edad_min = (datetime.now() - fecha).total_seconds() / 60            # se asume hora de Colombia en ambos
    if edad_min > EDAD_MAX_MIN:
        return None                                                     # dato viejo: no sirve para saber si llueve ahora
    valor = max(float(f.get(COL_VALOR, 0) or 0) for f in recientes)
    nombre = recientes[0].get(COL_NOMBRE, "estacion")
    return valor >= LLUVIA_MM_MIN, "IDEAM", f"{nombre}: {valor} mm ({ultima[:16].replace('T', ' ')})"


def consultar_openmeteo():
    """Respaldo: precipitacion actual en Tunja segun Open-Meteo (modelo, no es una estacion del IDEAM)."""
    r = requests.get("https://api.open-meteo.com/v1/forecast",
                     params={"latitude": TUNJA_LAT, "longitude": TUNJA_LON, "current": "precipitation"}, timeout=10)
    r.raise_for_status()
    p = float(r.json()["current"]["precipitation"])
    return p >= LLUVIA_MM_MIN, "Open-Meteo", f"{p} mm ahora"


class Clima:
    """Consulta el clima en un hilo aparte. llueve: True / False / None (sin datos)."""

    def __init__(self):
        self.llueve, self.fuente, self.detalle = None, "sin datos", ""
        self.manual = None                      # None = automatico, True = forzar lluvia, False = forzar seco (tecla L)
        if USAR_CLIMA:
            threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while True:
            self.actualizar()
            time.sleep(CLIMA_CADA_S)

    def actualizar(self):
        for nombre, f in (("IDEAM", consultar_ideam), ("Open-Meteo", consultar_openmeteo)):
            if nombre == "Open-Meteo" and not USAR_RESPALDO_OPENMETEO:
                break
            try:
                r = f()
            except Exception as e:
                print(f"[clima] {nombre} fallo: {type(e).__name__}: {e}")
                continue
            if r:
                self.llueve, self.fuente, self.detalle = r
                print(f"[clima] {self.fuente}: {'LLUEVE' if self.llueve else 'sin lluvia'} - {self.detalle}")
                return
        self.llueve, self.fuente, self.detalle = None, "sin datos", ""

    def lluvia(self):
        return self.manual if self.manual is not None else bool(self.llueve)

    def limite(self):
        return min(LIMITE_KMH, LIMITE_LLUVIA_KMH) if self.lluvia() else LIMITE_KMH

    def texto(self):
        if self.manual is not None:
            return ("LLUVIA (manual)" if self.manual else "SECO (manual)")
        if self.llueve is None:
            return "sin datos"
        return f"{'LLUVIA' if self.llueve else 'sin lluvia'} ({self.fuente})"


def diagnostico_clima():
    """python sistema_interfaz.py clima : muestra lo que devuelve el IDEAM para tu municipio."""
    print(f"Consultando datos.gov.co/{IDEAM_DATASET} para municipio {IDEAM_MUNICIPIO} ...")
    try:
        r = requests.get(f"https://www.datos.gov.co/resource/{IDEAM_DATASET}.json",
                         params={"$where": f"upper({COL_MUNICIPIO}) like '%{IDEAM_MUNICIPIO.upper()}%'",
                                 "$order": f"{COL_FECHA} DESC", "$limit": 10}, timeout=15)
        print("HTTP", r.status_code)
        filas = r.json()
        if isinstance(filas, list) and filas:
            print("Columnas del dato:", list(filas[0].keys()))
            for f in filas:
                print({k: f.get(k) for k in (COL_ESTACION, COL_NOMBRE, COL_FECHA, COL_VALOR)})
        else:
            print("Respuesta sin filas:", str(filas)[:300])
    except Exception as e:
        print("Fallo la consulta:", type(e).__name__, e)
    print("\nDecision del programa:")
    c = Clima.__new__(Clima)
    c.llueve, c.fuente, c.detalle, c.manual = None, "sin datos", "", None
    c.actualizar()
    print("->", c.texto(), "| limite activo:", c.limite(), "km/h")


def preparar(frame, grados):
    """Gira la imagen (si hace falta) y la deja con un ancho maximo de ANCHO."""
    if grados == 90:
        frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    elif grados == 180:
        frame = cv2.rotate(frame, cv2.ROTATE_180)
    elif grados == 270:
        frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    h, w = frame.shape[:2]
    if w > ANCHO:
        frame = cv2.resize(frame, (ANCHO, int(h * ANCHO / w)))
    return frame


TRACKER_YAML = "bytetrack_sensible.yaml"


def crear_tracker():
    """ByteTrack por defecto solo empieza a seguir objetos con confianza >= 0.25. Este archivo baja ese piso a CONF_MIN
    para que los carros poco seguros tambien reciban un ID. Si algo falla, usa el bytetrack.yaml normal."""
    try:
        with open(TRACKER_YAML, "w", encoding="utf-8") as f:
            f.write("tracker_type: bytetrack\n"
                    f"track_high_thresh: {CONF_MIN}\n"
                    "track_low_thresh: 0.1\n"
                    f"new_track_thresh: {CONF_MIN}\n"
                    "track_buffer: 30\n"
                    "match_thresh: 0.8\n"
                    "fuse_score: True\n")
        return TRACKER_YAML
    except OSError:
        return "bytetrack.yaml"


class Confirmador:
    """Activa un aviso tras N cuadros seguidos y lo mantiene un tiempo (evita parpadeos y falsas alarmas)."""

    def __init__(self, cuadros, mantener):
        self.cuadros, self.mantener = cuadros, mantener
        self.n, self.hasta = 0, 0.0

    def actualizar(self, condicion, t):
        self.n = self.n + 1 if condicion else 0
        if self.n >= self.cuadros:
            self.hasta = t + self.mantener
        return t < self.hasta


_pitando = False


def pitar(nivel):
    """Pitido del computador en un hilo aparte. nivel 3 = peligro (3 pitidos agudos), 2 = exceso (1 pitido)."""
    global _pitando
    if _pitando:
        return
    _pitando = True

    def _run():
        global _pitando
        try:
            if winsound is not None:
                if nivel >= 3:
                    for _ in range(3):
                        winsound.Beep(1900, 160)
                else:
                    winsound.Beep(1300, 350)
            else:
                print("\a", end="", flush=True)
        finally:
            time.sleep(0.5)
            _pitando = False

    threading.Thread(target=_run, daemon=True).start()


def texto(img, s, org, escala=0.6, color=BLANCO, grosor=1, sombra=False):
    """Texto; con sombra=True lleva borde negro para leerse sobre el video."""
    if sombra:
        cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, escala, (0, 0, 0), grosor + 2)
    cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, escala, color, grosor)


def calibrar():
    """Muestra un cuadro; haces clic en las 4 esquinas del rectangulo medido en el piso."""
    cam = MJPEGCam(CAM_URL)
    ok, frame = cam.read()
    cam.release()
    if not ok:
        raise SystemExit("No llegan imagenes de la camara.")
    frame = preparar(frame, ROTAR)
    puntos = []
    nombres = ["arriba-izquierda", "arriba-derecha", "abajo-derecha", "abajo-izquierda"]

    def click(ev, x, y, flags, param):
        if ev == cv2.EVENT_LBUTTONDOWN and len(puntos) < 4:
            puntos.append((x, y))
            print(f"punto {len(puntos)} ({nombres[len(puntos) - 1]}): [{x}, {y}]")

    cv2.namedWindow("Calibracion")
    cv2.setMouseCallback("Calibracion", click)
    while True:
        vista = frame.copy()
        for i, (x, y) in enumerate(puntos):
            cv2.circle(vista, (x, y), 5, (0, 255, 255), -1)
            cv2.putText(vista, str(i + 1), (x + 8, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        msg = f"Clic en: {nombres[len(puntos)]}" if len(puntos) < 4 else "Listo: presiona ESC"
        cv2.putText(vista, msg, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("Calibracion", vista)
        if cv2.waitKey(30) == 27:
            break
    cv2.destroyAllWindows()
    if len(puntos) == 4:
        print("\nPega esto en PTS_IMG:")
        print("PTS_IMG = np.float32(" + str([list(p) for p in puntos]) + ")")


# =====================================================================
#                      5. INTERFAZ (video + panel de avisos)
# =====================================================================
NIVELES = {
    0: ("SIN AVISOS", VERDE, "Zona despejada"),
    1: ("ATENCION", AMBAR, "Peaton en la zona de cruce"),
    2: ("EXCESO DE VELOCIDAD", ROJO, ""),
    3: ("PELIGRO", ROJO, "Vehiculo rapido y peaton en la zona"),
}


def armar_vista(frame, nivel, t, info):
    """Pone el video a la izquierda y el panel de avisos a la derecha. Devuelve la imagen final."""
    h, w = frame.shape[:2]
    alto = max(h, 600)
    vista = np.full((alto, w + PANEL_ANCHO, 3), FONDO, np.uint8)
    vista[:h, :w] = frame

    # ---- cartel sobre el video ----
    if nivel > 0:
        titulo, color, _ = NIVELES[nivel]
        if nivel == 2:
            msg = f"EXCESO DE VELOCIDAD: {info['vexc']:.0f} km/h (limite {info['limite']})"
        elif nivel == 3:
            msg = f"PELIGRO: VEHICULO A {info['vrap']:.0f} km/h Y PEATON EN LA ZONA"
        else:
            msg = "ATENCION: PEATON EN LA ZONA DE CRUCE"
        if nivel >= 2 and int(t * 4) % 2 == 1:
            color = (0, 0, 150)                              # parpadeo del cartel rojo
        sobre = vista.copy()
        cv2.rectangle(sobre, (0, 0), (w, 44), color, -1)
        vista[:44, :w] = cv2.addWeighted(sobre[:44, :w], 0.82, vista[:44, :w], 0.18, 0)
        texto(vista, msg, (10, 29), 0.62, (255, 255, 255), 2, sombra=True)

    if info['sin_cal']:                                      # aviso grande: velocidades sin valor
        sobre = vista.copy()
        cv2.rectangle(sobre, (0, h - 56), (w, h), (0, 0, 170), -1)
        vista[h - 56:h, :w] = cv2.addWeighted(sobre[h - 56:h, :w], 0.85, vista[h - 56:h, :w], 0.15, 0)
        texto(vista, "SIN CALIBRAR: velocidades NO reales", (10, h - 32), 0.62, (255, 255, 255), 2, sombra=True)
        texto(vista, "python sistema_interfaz.py calibrar", (10, h - 10), 0.5, (255, 255, 255), 1, sombra=True)

    # ---- panel ----
    x0 = w + 16
    texto(vista, "MEDIDOR DE VELOCIDAD", (x0, 30), 0.62, BLANCO, 2)
    texto(vista, f"{info['fps']:.1f} FPS   YOLO: {MODELO}", (x0, 54), 0.5, GRIS)

    titulo, color, sub = NIVELES[nivel]
    if nivel >= 2 and int(t * 4) % 2 == 1:
        color = (0, 0, 150)
    cv2.rectangle(vista, (x0 - 6, 68), (w + PANEL_ANCHO - 10, 138), color, -1)
    texto(vista, titulo, (x0 + 4, 100), 0.78 if len(titulo) < 14 else 0.62, (255, 255, 255), 2)
    sub = sub or f"{info['vexc']:.0f} km/h > {info['limite']} km/h"
    texto(vista, sub[:38], (x0 + 4, 125), 0.45, (255, 255, 255), 1)

    texto(vista, f"PEATONES:   {info['pers_tot']}   (en zona: {info['pers_zona']})", (x0, 168), 0.55, VERDE, 1)
    texto(vista, f"VEHICULOS:  {info['veh_tot']}   (en zona: {info['veh_zona']})", (x0, 192), 0.55, AZUL, 1)
    texto(vista, f"Vel. maxima vehiculo: {info['vmax']:.0f} km/h", (x0, 216), 0.5, BLANCO, 1)

    colc = AMBAR if info['lluvia'] else BLANCO
    texto(vista, f"CLIMA: {info['clima']}"[:38], (x0, 240), 0.5, colc, 1)
    texto(vista, f"Limite activo: {info['limite']} km/h", (x0, 262), 0.55, colc, 2 if info['lluvia'] else 1)

    texto(vista, "EN PANTALLA", (x0, 290), 0.5, GRIS, 1)
    cv2.line(vista, (x0, 296), (w + PANEL_ANCHO - 12, 296), (80, 70, 60), 1)
    for i, (nombre, tid, kmh, col, vtxt) in enumerate(info['lista'][:6]):
        texto(vista, f"{nombre} #{tid}   {vtxt}"[:38], (x0, 316 + 20 * i), 0.5, col, 1)
    if not info['lista']:
        texto(vista, "(nada detectado)", (x0, 316), 0.5, GRIS, 1)

    texto(vista, "ULTIMOS AVISOS", (x0, 448), 0.5, GRIS, 1)
    cv2.line(vista, (x0, 454), (w + PANEL_ANCHO - 12, 454), (80, 70, 60), 1)
    for i, (hora, msg, col) in enumerate(list(info['avisos'])[::-1][:5]):
        texto(vista, f"{hora}  {msg}"[:40], (x0, 474 + 19 * i), 0.43, col, 1)
    if not info['avisos']:
        texto(vista, "(sin avisos todavia)", (x0, 474), 0.45, GRIS, 1)

    snd = "ON" if info['sonido'] else "OFF"
    texto(vista, f"ESC salir  R girar({info['rot']})  S sonido {snd}  L lluvia", (x0, alto - 12), 0.42, GRIS, 1)
    return vista


# =====================================================================
#                      6. PROGRAMA PRINCIPAL
# =====================================================================
def listar_camaras():
    """Prueba los numeros de camara 0..5 y muestra una ventana con cada una que abra (cualquier tecla = siguiente)."""
    for i in range(6):
        cap = cv2.VideoCapture(i, cv2.CAP_DSHOW) if sys.platform.startswith("win") else cv2.VideoCapture(i)
        ok, img = (cap.read() if cap.isOpened() else (False, None))
        if ok and img is not None:
            print(f"Camara {i}: abre, {img.shape[1]}x{img.shape[0]}")
            cv2.imshow(f"Camara {i} (cualquier tecla = siguiente)", img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        else:
            print(f"Camara {i}: no abre")
        cap.release()
    print("Pon el numero de la correcta en CAM_URL, por ejemplo: CAM_URL = 1")


def main():
    global SONIDO
    from ultralytics import YOLO

    rot = ROTAR
    calibrado = not np.allclose(PTS_IMG, PTS_IMG_EJEMPLO)
    sin_cal = (not calibrado) and (not PERMITIR_SIN_CALIBRAR)
    if not calibrado:
        print("[AVISO] PTS_IMG es el de EJEMPLO: las velocidades no son reales. Corre: python sistema_interfaz.py calibrar")
    H = cv2.getPerspectiveTransform(PTS_IMG, PTS_REAL)     # homografia: pixeles -> metros
    area = PTS_IMG.astype(np.int32)                        # poligono calibrado
    model = YOLO(MODELO)                                   # YOLO preentrenado (se descarga solo la primera vez)
    print(f"[ok] YOLO cargado: {MODELO}  (clases: {[NOMBRES.get(c, c) for c in CLASES]})")
    clima = Clima()
    tracker = crear_tracker()
    cam = MJPEGCam(CAM_URL)
    esp = Notificador(PUERTO_ESP32, BAUDIOS) if USAR_ESP32 else None

    ok, _ = cam.read()
    if not ok:
        raise SystemExit("No llegan imagenes de la camara. Revisa que la webcam este conectada por USB y que ninguna otra app la use, o cambia CAM_URL a 1.")

    hist = defaultdict(lambda: deque(maxlen=VENTANA_POS))
    vel_hist = defaultdict(lambda: deque(maxlen=VENTANA_MEDIANA))
    visto, detalle = {}, {}

    conf_peaton = Confirmador(CONFIRMAR_CUADROS, MANTENER_ALERTA_S)
    conf_exceso = Confirmador(CONFIRMAR_CUADROS, MANTENER_ALERTA_S)
    conf_peligro = Confirmador(CONFIRMAR_CUADROS, MANTENER_ALERTA_S)
    activos_prev = set()
    avisos = deque(maxlen=20)
    ultimo_envio, ultimo_estado = 0.0, None
    fps = 0.0

    archivo = writer = None
    if GUARDAR_CSV:
        archivo = open(ARCHIVO_CSV, "w", newline="", encoding="utf-8")
        writer = csv.writer(archivo)
        writer.writerow(["tiempo_s", "id", "clase", "X_m", "Y_m", "dr_m", "dt_s", "v_kmh", "limite_kmh"])
    t_inicio = time.time()
    print("Interfaz lista. ESC = salir, R = girar imagen, S = sonido on/off")

    try:
        while not STOP:
            ok, frame = cam.read()
            if not ok:
                continue
            t = time.time()                                # marca de tiempo REAL del cuadro
            frame = preparar(frame, rot)
            limite = clima.limite()                        # 30 km/h normal, 20 km/h si llueve

            try:
                res = model.track(frame, persist=True, classes=CLASES, tracker=tracker,
                                  conf=CONF_MIN, imgsz=IMGSZ, verbose=False)[0]
            except Exception as e:
                if tracker == "bytetrack.yaml":
                    raise
                print(f"[aviso] el seguimiento sensible fallo ({type(e).__name__}: {e}); uso bytetrack.yaml normal")
                tracker = "bytetrack.yaml"
                continue

            pers_tot = pers_zona = veh_tot = veh_zona = 0
            vmax = vexc = vrap = 0.0
            hay_exceso = veh_rapido_zona = False
            lista = []

            if res.boxes.id is not None:
                for box, tid, cls, conf in zip(res.boxes.xyxy.cpu().numpy(),
                                               res.boxes.id.int().cpu().numpy(),
                                               res.boxes.cls.int().cpu().numpy(),
                                               res.boxes.conf.cpu().numpy()):
                    if conf < (CONF_PERSONA if int(cls) == 0 else CONF_VEHICULO):
                        continue                           # no alcanza la confianza minima de su clase
                    x1, y1, x2, y2 = box
                    if (y2 - y1) < MIN_ALTO_PX:
                        continue                           # demasiado pequeno / lejano para medirlo bien
                    cx, cy = (x1 + x2) / 2, y2             # punto de apoyo: centro del borde inferior
                    visto[tid] = t
                    dentro = cv2.pointPolygonTest(area, (float(cx), float(cy)), False) >= 0

                    if SOLO_DENTRO_DEL_AREA and not dentro:
                        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), GRIS, 1)
                        hist[tid].clear()
                        continue

                    en_zona = dentro if USAR_AREA_COMO_ZONA else True
                    xm, ym = a_metros(H, cx, cy)
                    hist[tid].append((t, xm, ym))

                    kmh = 0.0
                    if len(hist[tid]) >= MIN_MUESTRAS:
                        t0, x0_, y0_ = hist[tid][0]
                        t1, xf, yf = hist[tid][-1]
                        dt = t1 - t0
                        if dt > MIN_DT:
                            dr = math.hypot(xf - x0_, yf - y0_)    # distancia recorrida (m)
                            v = dr / dt * 3.6                      # v = dr/dt  ->  km/h
                            vel_hist[tid].append(v)
                            kmh = float(np.median(vel_hist[tid]))
                            detalle[tid] = (dr, dt)
                            if writer:
                                writer.writerow([round(t - t_inicio, 3), int(tid), NOMBRES.get(int(cls), str(cls)),
                                                 round(xm, 3), round(ym, 3), round(dr, 3), round(dt, 3),
                                                 round(kmh, 2), limite])
                    if kmh < UMBRAL_QUIETO_KMH or sin_cal:
                        kmh = 0.0
                    vtxt = "-- km/h" if sin_cal else f"{kmh:.0f} km/h"

                    nombre = NOMBRES.get(int(cls), "OBJETO")
                    if int(cls) == 0:                                   # ---- peaton
                        pers_tot += 1
                        pers_zona += 1 if en_zona else 0
                        color = VERDE
                    else:                                               # ---- vehiculo
                        veh_tot += 1
                        veh_zona += 1 if en_zona else 0
                        vmax = max(vmax, kmh)
                        color = AZUL
                        if kmh > limite:
                            hay_exceso = True
                            vexc = max(vexc, kmh)
                            color = ROJO
                        if en_zona and kmh >= VEL_PELIGRO_KMH:
                            veh_rapido_zona = True
                            vrap = max(vrap, kmh)
                    lista.append((nombre, int(tid), kmh, color, vtxt))

                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
                    texto(frame, f"{nombre} #{int(tid)} {conf * 100:.0f}% {vtxt}", (int(x1), max(18, int(y1) - 8)), 0.5, color, 2, sombra=True)
                    if MOSTRAR_DETALLE and tid in detalle and kmh > 0 and not sin_cal:
                        dr, dt = detalle[tid]
                        texto(frame, f"dr={dr:.2f} m  dt={dt:.2f} s", (int(x1), int(y2) + 18), 0.45, color, 1, sombra=True)

            # ---- avisos: cada uno se confirma varios cuadros y se mantiene un rato ----
            a_peaton = conf_peaton.actualizar(pers_zona > 0, t)
            a_exceso = conf_exceso.actualizar(hay_exceso, t)
            a_peligro = conf_peligro.actualizar(pers_zona > 0 and veh_rapido_zona, t)
            nivel = 3 if a_peligro else 2 if a_exceso else 1 if a_peaton else 0

            activos = {n for n, a in ((1, a_peaton), (2, a_exceso), (3, a_peligro)) if a}
            hora = time.strftime("%H:%M:%S")
            for n in sorted(activos - activos_prev):               # avisos nuevos desde el cuadro anterior
                if n == 1:
                    avisos.append((hora, f"Peaton en zona ({pers_zona})", AMBAR))
                elif n == 2:
                    avisos.append((hora, f"Exceso {vexc:.0f} km/h (lim {limite})", ROJO))
                else:
                    avisos.append((hora, f"PELIGRO: vehiculo {vrap:.0f} km/h", ROJO))
                if SONIDO and n >= 2:
                    pitar(n)
            activos_prev = activos

            if esp is not None:                                    # LED del ESP32 (opcional)
                alerta = (1 if a_peaton else 0) | (2 if nivel >= 2 else 0)   # 0 normal, 1 peaton, 2 exceso, 3 peligro (peaton + exceso)
                if alerta != ultimo_estado or t - ultimo_envio > LATIDO_S:
                    # mensaje: estado,velocidad maxima (km/h),limite vigente (km/h),lluvia (0/1)  -> lo usan los LED y la pantalla OLED
                    esp.enviar(f"{alerta},{int(round(vmax))},{int(limite)},{1 if clima.lluvia() else 0}")
                    ultimo_estado, ultimo_envio = alerta, t

            for k in [k for k, v in visto.items() if t - v > OLVIDAR_TRACK_S]:
                visto.pop(k, None); hist.pop(k, None); vel_hist.pop(k, None); detalle.pop(k, None)

            if USAR_AREA_COMO_ZONA or SOLO_DENTRO_DEL_AREA:
                cv2.polylines(frame, [area], True, (255, 255, 0), 2)
            fps = 0.9 * fps + 0.1 / max(time.time() - t, 1e-3) if fps else 1 / max(time.time() - t, 1e-3)
            lista.sort(key=lambda r: -r[2])
            info = dict(fps=fps, pers_tot=pers_tot, pers_zona=pers_zona, veh_tot=veh_tot, veh_zona=veh_zona,
                        vmax=vmax, vexc=vexc, vrap=vrap, lista=lista, avisos=avisos, sonido=SONIDO, rot=rot,
                        limite=limite, lluvia=clima.lluvia(), clima=clima.texto(), sin_cal=sin_cal)
            cv2.imshow("Medidor de velocidad", armar_vista(frame, nivel, t, info))

            tecla = cv2.waitKey(1) & 0xFF
            if tecla == 27:                                        # ESC
                break
            elif tecla in (ord("r"), ord("R")):                    # girar 90 grados
                rot = (rot + 90) % 360
                hist.clear(); vel_hist.clear(); detalle.clear()
                print(f"Imagen girada a {rot} grados. Si usas el area calibrada, vuelve a calibrar con ROTAR = {rot}.")
            elif tecla in (ord("s"), ord("S")):
                SONIDO = not SONIDO
            elif tecla in (ord("l"), ord("L")):                    # automatico -> lluvia -> seco -> automatico
                clima.manual = True if clima.manual is None else False if clima.manual else None
                print(f"Clima: {clima.texto()}")
    finally:
        cam.release()
        if esp is not None:
            esp.enviar(False)
            time.sleep(0.3)
        if archivo:
            archivo.close()
            print(f"Mediciones guardadas en {ARCHIVO_CSV}")
        cv2.destroyAllWindows()


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "calibrar":
        calibrar()
    elif len(sys.argv) > 1 and sys.argv[1] == "camaras":
        listar_camaras()
    elif len(sys.argv) > 1 and sys.argv[1] == "clima":
        diagnostico_clima()
    else:
        main()