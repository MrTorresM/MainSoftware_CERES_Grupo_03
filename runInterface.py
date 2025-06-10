import subprocess
import webbrowser
import os
import time

# Ruta relativa al archivo HTML principal
HTML_PATH = "GUI/HTML/index.html"

# Puerto del servidor HTTP
HTTP_PORT = 8000

def iniciar_http_server():
    """Levanta un servidor HTTP local para servir la interfaz web"""
    try:
        print(f"[Sistema] Iniciando servidor HTTP en http://localhost:{HTTP_PORT} ...")
        os.chdir(".")
        subprocess.Popen(["python", "-m", "http.server", str(HTTP_PORT)])
    except subprocess.CalledProcessError as e:
        print(f"[Error] Falló el servidor HTTP: {e}")

def abrir_interfaz():
    """Abre la interfaz web en el navegador"""
    time.sleep(2)  # Espera para asegurar que el servidor HTTP esté listo
    url = f"http://localhost:{HTTP_PORT}/{HTML_PATH}"
    print(f"[Sistema] Abriendo interfaz en navegador: {url}")
    webbrowser.open(url)

if __name__ == "__main__":
    print("[Sistema] Iniciando servidor HTTP e interfaz...")
    iniciar_http_server()
    abrir_interfaz()