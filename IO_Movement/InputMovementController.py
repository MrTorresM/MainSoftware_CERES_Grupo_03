import pygame
import sys
import requests
import time

# ---------- Configuración de conexión con el ESP32 ----------
ESP32_IP     = "172.20.10.2"   # IP del ESP32 que imprime el JSON y recibe POST
URL_POST     = f"http://{ESP32_IP}/post"
URL_SENSORS  = f"http://{ESP32_IP}/sensors"
HTTP_TIMEOUT = 0.3            # Segundos de timeout en POST
SENSOR_FREQ  = 1.0            # Intervalo (s) entre lecturas de sensores

def enviar_comando(cmd: str):
    """
    Envía un POST al ESP32 con el cuerpo siendo el comando (texto plano).
    """
    try:
        requests.post(URL_POST, data=cmd.encode("utf-8"), timeout=HTTP_TIMEOUT)
    except:
        pass  # Ignora errores de conexión

def leer_sensores():
    """
    Realiza GET a /sensors y devuelve un diccionario con las lecturas.
    Si falla, devuelve None.
    """
    try:
        resp = requests.get(URL_SENSORS, timeout=1.0)
        if resp.status_code == 200:
            return resp.json()
    except:
        pass
    return None

# ---------- Inicialización de Pygame y joystick ----------
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("No se detectó ningún joystick. Conéctalo y vuelve a ejecutar.")
    pygame.quit()
    sys.exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"Joystick detectado: {joystick.get_name()}")
print("Ejes disponibles:")
print("  Izquierdo  → axis 0 (horizontal), axis 1 (vertical)")
print("  Derecho    → axis 2 (horizontal), axis 3 (vertical)")
print("Pulsa ESC o cierra la ventana para salir.\n")

# ---------- Parámetros de umbral y estado ----------
THRESHOLD = 0.5
estado_anterior = { "lx":0, "ly":0, "rx":0, "ry":0 }
clock = pygame.time.Clock()
last_sensors = time.time() - SENSOR_FREQ

def obtener_estado(val):
    if val < -THRESHOLD:
        return -1
    elif val > THRESHOLD:
        return +1
    else:
        return 0

while True:
    # 1) Manejamos eventos Pygame (salir si se cierra ventana o ESC)
    for evento in pygame.event.get():
        if evento.type == pygame.QUIT:
            enviar_comando("L:parar")
            pygame.quit()
            sys.exit()
        elif evento.type == pygame.KEYDOWN and evento.key == pygame.K_ESCAPE:
            enviar_comando("L:parar")
            pygame.quit()
            sys.exit()

    # 2) Leemos ejes del joystick
    lx = joystick.get_axis(0)
    ly = joystick.get_axis(1)
    rx = joystick.get_axis(2)
    ry = joystick.get_axis(3)

    lx_state = obtener_estado(lx)
    ly_state = obtener_estado(ly)
    rx_state = obtener_estado(rx)
    ry_state = obtener_estado(ry)

    # 3) Stick izquierdo (L:) controla motores
    if ly_state != estado_anterior["ly"]:
        if ly_state == -1:
            print("Joystick Izquierdo → arriba")
            enviar_comando("L:adelante")
        elif ly_state == +1:
            print("Joystick Izquierdo → abajo")
            enviar_comando("L:atras")
        else:  # ly_state == 0
            print("Joystick Izquierdo → centrado (vertical)")
            enviar_comando("L:parar")
        estado_anterior["ly"] = ly_state
        estado_anterior["lx"] = 0
        # Saltamos lectura horizontal de este ciclo
        continue

    if lx_state != estado_anterior["lx"]:
        if lx_state == -1:
            print("Joystick Izquierdo → izquierda")
            enviar_comando("L:izquierda")
        elif lx_state == +1:
            print("Joystick Izquierdo → derecha")
            enviar_comando("L:derecha")
        else:  # lx_state == 0
            print("Joystick Izquierdo → centrado (horizontal)")
            enviar_comando("L:parar")
        estado_anterior["lx"] = lx_state
        continue

    # 4) Si el stick izquierdo está en (0,0), usamos stick derecho (R:) para servos
    if lx_state == 0 and ly_state == 0:
        if ry_state != estado_anterior["ry"]:
            if ry_state == -1:
                print("Joystick Derecho → arriba")
                enviar_comando("R:adelante")
            elif ry_state == +1:
                print("Joystick Derecho → abajo")
                enviar_comando("R:atras")
            else:  # ry_state == 0
                print("Joystick Derecho → centrado (vertical)")
                enviar_comando("R:parar")
            estado_anterior["ry"] = ry_state
            estado_anterior["rx"] = 0
            continue

        if rx_state != estado_anterior["rx"]:
            if rx_state == -1:
                print("Joystick Derecho → izquierda")
                enviar_comando("R:izquierda")
            elif rx_state == +1:
                print("Joystick Derecho → derecha")
                enviar_comando("R:derecha")
            else:  # rx_state == 0
                print("Joystick Derecho → centrado (horizontal)")
                enviar_comando("R:parar")
            estado_anterior["rx"] = rx_state
            continue

    # 5) Sin cambios, no enviamos nada

    # 6) Cada SENSOR_FREQ segundos, pedimos datos de sensores y los imprimimos
    t = time.time()
    if t - last_sensors >= SENSOR_FREQ:
        sensor_data = leer_sensores()
        if sensor_data:
            # Imprimimos en consola Python
            color       = sensor_data["color"]
            distancia   = sensor_data["distancia_cm"]
            voltaje     = sensor_data["voltaje"]
            magnetometro = sensor_data["magnetometro"]
            print("---- Lectura Sensores ----")
            print(f" Color → R:{color['R']} G:{color['G']} B:{color['B']}  Clasif: {color['clasificacion']}")
            print(f" Distancia → {distancia} cm")
            print(f" Voltaje → {voltaje} V")
            print(f" Magnetómetro → X:{magnetometro['rawX']} Y:{magnetometro['rawY']} Z:{magnetometro['rawZ']} Heading: {magnetometro['heading°']}°")
            print("----------------------------\n")
        last_sensors = t

    # 7) Control de tasa de refresco para joystick (~120 FPS)
    clock.tick(120)
