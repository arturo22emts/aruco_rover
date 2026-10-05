Comandos de Despliegue en Raspberry Pi (systemd)
Ejecuta estos comandos en la terminal SSH de tu Raspberry Pi. (Nota: El código superior fue limpiado de caracteres invisibles \xa0 generados al copiar desde la web, por lo que no lanzará errores de sintaxis al ejecutarse).

1. Instalar requerimientos (si no están instalados)

Bash
sudo apt update
sudo apt install python3-opencv
pip3 install RPi.GPIO opencv-python
2. Crear el archivo del servicio de arranque automático

Bash
sudo nano /etc/systemd/system/rover.service
3. Pegar la configuración del servicio
(Asegúrate de cambiar /home/pi/aruco_headless.py por la ruta real donde guardaste el archivo).

Ini, TOML
[Unit]
Description=Control de Rover ArUco (Headless)
After=multi-user.target

[Service]
Type=simple
User=pi
ExecStart=/usr/bin/python3 /home/pi/aruco_headless.py
WorkingDirectory=/home/pi/
StandardOutput=journal
StandardError=journal
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
(Guarda con CTRL+O, Enter y sal con CTRL+X).

4. Habilitar y arrancar el demonio

Bash
sudo systemctl daemon-reload
sudo systemctl enable rover.service
sudo systemctl start rover.service
5. Monitorear telemetría y FPS en tiempo real

Bash
journalctl -u rover.service -f
