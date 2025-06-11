# runInterface.py

import subprocess
import webbrowser
import os
import time
import threading
import http.server
import socketserver
import signal
import sys

# Puerto del servidor HTTP
HTTP_PORT = 8000

# Ruta base del proyecto donde está la carpeta GUI
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
INTERFACE_DIR = os.path.join(BASE_DIR, "GUI")

# Ruta relativa al archivo HTML principal desde INTERFACE_DIR
HTML_PATH = "HTML/index.html"

# Objeto del servidor para poder cerrarlo
httpd = None

# Handler personalizado para manejar ruta de cierre
class CustomHandler(http.server.SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/cerrar":
            self.send_response(200)
            self.end_headers()
            self.wfile.write(b"[Sistema] Servidor cerrado correctamente.")
            print("[Sistema] Solicitud de cierre recibida. Apagando servidor...")
            threading.Thread(target=detener_servidor).start()
        else:
            super().do_GET()

class CustomTCPServer(socketserver.TCPServer):
    allow_reuse_address = True

def iniciar_http_server():
    global httpd
    os.chdir(INTERFACE_DIR)
    handler = CustomHandler
    httpd = CustomTCPServer(("", HTTP_PORT), handler)
    print(f"[Sistema] Servidor HTTP corriendo en http://localhost:{HTTP_PORT} ...")
    httpd.serve_forever()

def detener_servidor():
    global httpd
    if httpd:
        httpd.shutdown()
        print("[Sistema] Servidor detenido.")

def abrir_interfaz():
    time.sleep(2)  # Espera para que el servidor esté activo
    url = f"http://localhost:{HTTP_PORT}/{HTML_PATH}"
    print(f"[Sistema] Abriendo interfaz en navegador: {url}")
    webbrowser.open(url)

if __name__ == "__main__":
    print("[Sistema] Iniciando servidor HTTP e interfaz...")
    servidor_thread = threading.Thread(target=iniciar_http_server, daemon=True)
    servidor_thread.start()
    abrir_interfaz()

    try:
        while servidor_thread.is_alive():
            time.sleep(1)
    except KeyboardInterrupt:
        print("[Sistema] Interrupción manual detectada. Cerrando servidor...")
        detener_servidor()
        sys.exit(0)