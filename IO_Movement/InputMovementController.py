import pygame
import sys
import requests
import time

# ---------- Configuración de conexión con el ESP32 ----------
ESP32_IP     = "172.20.10.2"   
URL_POST     = f"http://{ESP32_IP}/post"
URL_SENSORS  = f"http://{ESP32_IP}/sensors"
HTTP_TIMEOUT = 0.3            
SENSOR_FREQ  = 1.0            

def enviar_comando(cmd: str):
    try:
        requests.post(URL_POST, data=cmd.encode("utf-8"), timeout=HTTP_TIMEOUT)
    except:
        pass

def leer_sensores():
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

joystick = None

def detectar_joystick():
    global joystick
    if pygame.joystick.get_count() > 0:
        # Si no había uno o cambió
        if joystick is None:
            joystick = pygame.joystick.Joystick(0)
            joystick.init()
            print(f"[+] Joystick conectado: {joystick.get_name()}")
    else:
        if joystick is not None:
            print("[!] Joystick desconectado")
        joystick = None

detectar_joystick()

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
    for evento in pygame.event.get():
        # Gestión de cerrar ventana o ESC
        if evento.type == pygame.QUIT:
            enviar_comando("L:parar")
            pygame.quit()
            sys.exit()
        elif evento.type == pygame.KEYDOWN and evento.key == pygame.K_ESCAPE:
            enviar_comando("L:parar")
            pygame.quit()
            sys.exit()
        # Detección dinámica de joystick
        elif evento.type == pygame.JOYDEVICEADDED or evento.type == pygame.JOYDEVICEREMOVED:
            detectar_joystick()

    # Si hay joystick, leemos ejes y enviamos comandos
    if joystick is not None:
        try:
            lx = joystick.get_axis(0)
            ly = joystick.get_axis(1)
            rx = joystick.get_axis(2)
            ry = joystick.get_axis(3)
        except pygame.error:
            # En caso de error al leer, marcamos desconexión
            print("[!] Error leyendo joystick, asumiendo desconexión")
            joystick = None
            lx = ly = rx = ry = 0.0

        lx_state = obtener_estado(lx)
        ly_state = obtener_estado(ly)
        rx_state = obtener_estado(rx)
        ry_state = obtener_estado(ry)

        # Control stick izquierdo (L:)
        if ly_state != estado_anterior["ly"]:
            if ly_state == -1:
                print("Joystick Izquierdo → arriba")
                enviar_comando("L:adelante")
            elif ly_state == +1:
                print("Joystick Izquierdo → abajo")
                enviar_comando("L:atras")
            else:
                print("Joystick Izquierdo → centrado (vertical)")
                enviar_comando("L:parar")
            estado_anterior["ly"] = ly_state
            estado_anterior["lx"] = 0
            continue

        if lx_state != estado_anterior["lx"]:
            if lx_state == -1:
                print("Joystick Izquierdo → izquierda")
                enviar_comando("L:izquierda")
            elif lx_state == +1:
                print("Joystick Izquierdo → derecha")
                enviar_comando("L:derecha")
            else:
                print("Joystick Izquierdo → centrado (horizontal)")
                enviar_comando("L:parar")
            estado_anterior["lx"] = lx_state
            continue

        # Control stick derecho (R:) solo si izquierdo está en centro
        if lx_state == 0 and ly_state == 0:
            if ry_state != estado_anterior["ry"]:
                if ry_state == -1:
                    print("Joystick Derecho → arriba")
                    enviar_comando("R:adelante")
                elif ry_state == +1:
                    print("Joystick Derecho → abajo")
                    enviar_comando("R:atras")
                else:
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
                else:
                    print("Joystick Derecho → centrado (horizontal)")
                    enviar_comando("R:parar")
                estado_anterior["rx"] = rx_state
                continue

    # Cada SENSOR_FREQ segundos, leemos sensores aunque no haya joystick
    t = time.time()
    if t - last_sensors >= SENSOR_FREQ:
        sensor_data = leer_sensores()
        if sensor_data:
            color       = sensor_data["color"]
            distancia   = sensor_data["distancia_cm"]
            voltaje     = sensor_data["voltaje"]
            magnetom     = sensor_data["magnetometro"]
            print("---- Lectura Sensores ----")
            print(f" Color → R:{color['R']} G:{color['G']} B:{color['B']}  Clasif: {color['clasificacion']}")
            print(f" Distancia → {distancia} cm")
            print(f" Voltaje → {voltaje} V")
            print(f" Magnetómetro → X:{magnetom['rawX']} Y:{magnetom['rawY']} Z:{magnetom['rawZ']} Heading: {magnetom['heading°']}°")
            print("----------------------------\n")
        last_sensors = t

    clock.tick(120)
