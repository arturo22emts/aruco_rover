"""
Sistema Autónomo de Telemetría y Control de Tracción por Visión Artificial (Versión Headless)
Autor: Arturo Emiliano Montes Sarmiento (Adaptado del GCS de UNAM SPACE)

DESCRIPCIÓN:
Este script opera como un servicio de fondo (demonio) en una Raspberry Pi a bordo de un rover. 
Utiliza visión por computadora (OpenCV) para procesar el feed de video en tiempo real, 
calculando la pose y el ID de los marcadores ArUco para traducirlos directamente en 
señales eléctricas de control (GPIO):

- ArUco ID 0  ->  Activa el PIN_FORWARD (17) para tracción delantera.
- ArUco ID 1  ->  Activa el PIN_BACKWARD (27) para tracción en reversa.
- Sin marcador -> Ambos pines se apagan tras un margen de seguridad (SIGNAL_TIMEOUT).

CARACTERÍSTICAS DE PRODUCCIÓN:
- Rendimiento Headless: Eliminación total de GUI y renderizado para máxima eficiencia en CPU/RAM.
- Anti-tartamudeo (Memoria de estado): Tolera pérdida de frames por vibraciones o iluminación.
- Interlock Inteligente: Bloqueo por hardware instantáneo para evitar cortocircuitos si se ven ambos IDs.
- Reloj Monotónico: Temporizadores inmunes a saltos de red/NTP para una respuesta predecible.
- Watchdog Integrado: Aborta de forma segura si la cámara se desconecta, permitiendo que systemd lo reinicie.
"""

import cv2
import numpy as np
import time
import sys
import signal

# ------------------------- CONFIGURACIÓN -----------------------------
CAMERA_INDEX    = 0       # /dev/video0
CAMERA_WIDTH    = 640
CAMERA_HEIGHT   = 480

PIN_FORWARD     = 17      # BCM
PIN_BACKWARD    = 27      # BCM

MARKER_SIZE     = 0.05    # metros (5 cm) - debe coincidir con el impreso
MIN_STREAK      = 2       # frames consecutivos para validar un ID
MAX_Z           = 5.0     # ignora marcadores a más de 5 m
LOG_INTERVAL    = 0.5     # segundos entre logs de telemetría
FPS_LOG_INTERVAL = 10.0   # segundos entre logs de diagnóstico FPS

# Anti-tartamudeo: mantiene la señal activa X segundos tras la última detección.


CAMERA_FAIL_MAX = 30      # frames perdidos consecutivos antes de abortar
LOOP_SLEEP      = 0.001   # pequeño respiro por iteración (protege CPU)
# ---------------------------------------------------------------------

# ------------------------- GPIO --------------------------------------
try:
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(PIN_FORWARD,  GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(PIN_BACKWARD, GPIO.OUT, initial=GPIO.LOW)
    HAS_GPIO = True
    print(f"[HW] GPIO listo. FORWARD=BCM{PIN_FORWARD}  BACKWARD=BCM{PIN_BACKWARD}")
except ImportError:
    HAS_GPIO = False
    print("[!] RPi.GPIO no encontrado. Corriendo en modo simulación.")


# ------------------------- CIERRE LIMPIO -----------------------------
_shutdown_done = False

def shutdown(sig=None, frame=None):
    global _shutdown_done
    if _shutdown_done:
        return
    _shutdown_done = True

    print("\n[INFO] Deteniendo sistema...")
    try:
        cap.release()
    except Exception:
        pass
    if HAS_GPIO:
        try:
            GPIO.output(PIN_FORWARD,  GPIO.LOW)
            GPIO.output(PIN_BACKWARD, GPIO.LOW)
            GPIO.cleanup()
        except Exception as e:
            print(f"[!] Error al limpiar GPIO: {e}")
    sys.exit(0)

signal.signal(signal.SIGINT,  shutdown)
signal.signal(signal.SIGTERM, shutdown)


# ------------------------- CÁMARA ------------------------------------
cap = cv2.VideoCapture(CAMERA_INDEX)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAMERA_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)

if not cap.isOpened():
    print(f"[!] No se pudo abrir la cámara {CAMERA_INDEX}")
    sys.exit(1)

aw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
ah = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f"[INFO] Cámara abierta a {aw}x{ah}")


# ------------------------- MATRIZ INTRÍNSECA -------------------------
focal = aw * 1.25
cx = aw / 2.0
cy = ah / 2.0
camera_matrix = np.array([[focal, 0, cx],
                          [0, focal, cy],
                          [0, 0, 1]], dtype=np.float32)
dist_coeffs = np.zeros((4, 1))

half = MARKER_SIZE / 2.0
obj_points = np.array([[-half,  half, 0],
                       [ half,  half, 0],
                       [ half, -half, 0],
                       [-half, -half, 0]], dtype=np.float32)


# ------------------------- ARUCO -------------------------------------
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
params = cv2.aruco.DetectorParameters()
params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
detector = cv2.aruco.ArucoDetector(aruco_dict, params)


# ------------------------- ESTADO ------------------------------------
id_streak       = {}
last_log_time   = 0.0
last_fps_log    = time.monotonic()
fps_frame_count = 0

prev_gpio_fwd   = None
prev_gpio_bwd   = None

last_seen_fwd   = 0.0
last_seen_bwd   = 0.0

camera_fail_count = 0


# ------------------------- HELPERS -----------------------------------
def set_gpio(fwd, bwd):
    """Escribe en GPIO solo si el estado cambió."""
    global prev_gpio_fwd, prev_gpio_bwd
    if not HAS_GPIO:
        return
    if fwd != prev_gpio_fwd:
        GPIO.output(PIN_FORWARD,  GPIO.HIGH if fwd else GPIO.LOW)
        prev_gpio_fwd = fwd
    if bwd != prev_gpio_bwd:
        GPIO.output(PIN_BACKWARD, GPIO.HIGH if bwd else GPIO.LOW)
        prev_gpio_bwd = bwd


def log_detections(detections):
    """Log rate-limitado de detecciones."""
    global last_log_time
    now = time.monotonic()
    if now - last_log_time < LOG_INTERVAL:
        return
    last_log_time = now
    for mid, x, y, z in detections:
        print(f"ArUco Id: {mid:02d} | X - {x:+.2f} | Y - {y:+.2f} | Z - {z:.2f}")


def log_fps(now):
    """Log de FPS cada FPS_LOG_INTERVAL segundos."""
    global last_fps_log, fps_frame_count
    fps_frame_count += 1
    elapsed = now - last_fps_log
    if elapsed >= FPS_LOG_INTERVAL:
        fps = fps_frame_count / elapsed
        print(f"[INFO] FPS promedio: {fps:.1f}")
        last_fps_log = now
        fps_frame_count = 0


# ------------------------- LOOP PRINCIPAL ----------------------------
print("[INFO] Sistema corriendo. CTRL+C para salir.")

try:
    while True:
        ret, frame = cap.read()

        if not ret:
            camera_fail_count += 1
            if camera_fail_count > CAMERA_FAIL_MAX:
                print("[!] Cámara perdida. Saliendo para que systemd reinicie.")
                break
            time.sleep(0.05)
            continue
        camera_fail_count = 0

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        try:
            corners, ids, _ = detector.detectMarkers(gray)
        except Exception as e:
            print(f"[!] Error en detección: {e}")
            corners, ids = None, None

        detections     = []
        ids_this_frame = set()

        if ids is not None:
            for i in range(len(ids)):
                pts = np.squeeze(corners[i])

                ok, rvec, tvec = cv2.solvePnP(
                    obj_points, pts, camera_matrix, dist_coeffs
                )
                if not ok:
                    continue

                x, y, z = tvec.flatten()
                if z <= 0.05 or z > MAX_Z:
                    continue

                raw_id = ids[i]
                mid = int(raw_id[0] if hasattr(raw_id, "__len__") else raw_id)

                ids_this_frame.add(mid)

                id_streak[mid] = id_streak.get(mid, 0) + 1
                if id_streak[mid] < MIN_STREAK:
                    continue

                detections.append((mid, x, y, z))

        for mid in list(id_streak.keys()):
            if mid not in ids_this_frame:
                del id_streak[mid]

        # ------------------- LÓGICA GPIO -------------------
        stable_ids = [d[0] for d in detections]
        now = time.monotonic() 

        if 0 in stable_ids:
            last_seen_fwd = now
        if 1 in stable_ids:
            last_seen_bwd = now

        go_forward  = (now - last_seen_fwd) < SIGNAL_TIMEOUT
        go_backward = (now - last_seen_bwd) < SIGNAL_TIMEOUT

        if go_forward and go_backward:
            go_forward = go_backward = False

        set_gpio(go_forward, go_backward)

        # ------------------- LOGS --------------------------
        if detections:
            log_detections(detections)
        log_fps(now)

        time.sleep(LOOP_SLEEP)

except Exception as e:
    print(f"[!] Error fatal: {e}")
finally:
    shutdown()
