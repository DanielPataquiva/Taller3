import sys
import math
import numpy as np
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QMessageBox, QVBoxLayout,
    QWidget, QLabel, QLineEdit, QPushButton, QComboBox
)
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from time import sleep

# =========================
# CONFIGURACIÓN SERVOS GPIO
# =========================
try:
    from gpiozero import Servo
    from gpiozero.pins.pigpio import PiGPIOFactory
    factory = PiGPIOFactory()
    servo_enabled = True
    print("✅ Servos habilitados (modo físico)")
except ImportError:
    print("⚠️ gpiozero no disponible: el control físico se desactiva (solo simulación).")
    servo_enabled = False

# =========================
# PARÁMETROS DEL ROBOT
# =========================
L1 = 10.0  # cm
L2 = 10.0  # cm

# Offset físico de los servos (ajústalos hasta que coincida la posición real con la simulada)
OFFSET_THETA1 = 90   # Grado donde el brazo 1 apunta al eje X positivo
OFFSET_THETA2 = 90   # Grado donde el brazo 2 está extendido en línea con L1

# Invertir servos si giran al revés
INVERT_SERVO1 = False
INVERT_SERVO2 = True

if servo_enabled:
    servo1 = Servo(17, pin_factory=factory, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000)
    servo2 = Servo(18, pin_factory=factory, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000)
else:
    servo1 = servo2 = None


def map_angle(deg):
    """Convierte un ángulo en grados (0–180) a rango -1 a 1 para gpiozero."""
    deg = max(0, min(180, deg))
    return (deg - 90) / 90.0


def mover_servos(theta1_deg, theta2_deg):
    """Mueve los servos físicamente (si gpiozero está disponible)."""
    if not servo_enabled:
        return

    # Aplicar offsets físicos
    ang1 = OFFSET_THETA1 + (theta1_deg if not INVERT_SERVO1 else -theta1_deg)
    ang2 = OFFSET_THETA2 + (theta2_deg if not INVERT_SERVO2 else -theta2_deg)

    ang1 = max(0, min(180, ang1))
    ang2 = max(0, min(180, ang2))

    servo1.value = map_angle(ang1)
    servo2.value = map_angle(ang2)
    sleep(0.02)


# =========================
# CLASE PARA DIBUJAR ROBOT
# =========================
class RobotCanvas(FigureCanvas):
    def __init__(self):
        self.fig = Figure()
        super().__init__(self.fig)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_xlim(-25, 25)
        self.ax.set_ylim(-5, 25)
        self.ax.set_aspect('equal', 'box')
        self.ax.grid(True)
        self.link1, = self.ax.plot([], [], 'o-', lw=4, color='b')
        self.link2, = self.ax.plot([], [], 'o-', lw=4, color='g')
        self.target, = self.ax.plot([], [], 'rx', ms=10, mew=2)

    def dibujar_robot(self, theta1, theta2, x_target=None, y_target=None):
        x1 = L1 * math.cos(theta1)
        y1 = L1 * math.sin(theta1)
        x2 = x1 + L2 * math.cos(theta1 + theta2)
        y2 = y1 + L2 * math.sin(theta1 + theta2)

        self.link1.set_data([0, x1], [0, y1])
        self.link2.set_data([x1, x2], [y1, y2])

        if x_target is not None and y_target is not None:
            self.target.set_data([x_target], [y_target])

        self.ax.figure.canvas.draw()


# =========================
# CLASE PRINCIPAL INTERFAZ
# =========================
class RobotApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Simulación Robot 2R")
        self.resize(800, 600)

        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        # Área de simulación
        self.canvas = RobotCanvas()
        layout.addWidget(self.canvas)

        # Controles
        control_layout = QtWidgets.QFormLayout()
        self.x_input = QLineEdit()
        self.y_input = QLineEdit()
        self.elbow_combo = QComboBox()
        self.elbow_combo.addItems(["Codo arriba", "Codo abajo"])
        self.steps_input = QLineEdit("40")
        self.move_button = QPushButton("Mover robot")

        self.theta1_label = QLabel("θ1 = 0°")
        self.theta2_label = QLabel("θ2 = 0°")

        control_layout.addRow("X (cm):", self.x_input)
        control_layout.addRow("Y (cm):", self.y_input)
        control_layout.addRow("Codo:", self.elbow_combo)
        control_layout.addRow("Pasos animación:", self.steps_input)
        control_layout.addRow(self.move_button)
        control_layout.addRow(self.theta1_label)
        control_layout.addRow(self.theta2_label)

        layout.addLayout(control_layout)

        # Conectar
        self.move_button.clicked.connect(self.mover_robot)

        # Estado inicial
        self.theta1 = 0
        self.theta2 = 0
        self.canvas.dibujar_robot(0, 0)

    def mover_robot(self):
        try:
            x = float(self.x_input.text())
            y = float(self.y_input.text())
            steps = int(self.steps_input.text())
            elbow = self.elbow_combo.currentText()
        except ValueError:
            QMessageBox.warning(self, "Error", "Ingrese valores numéricos válidos.")
            return

        # Verificar si está dentro del área de trabajo
        if math.hypot(x, y) > (L1 + L2) or math.hypot(x, y) < abs(L1 - L2):
            QMessageBox.warning(self, "Fuera de alcance", "El punto está fuera del área de trabajo.")
            return

        # Cinemática inversa
        cos_t2 = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
        sin_t2 = math.sqrt(1 - cos_t2**2)
        if elbow == "Codo abajo":
            sin_t2 = -sin_t2

        theta2 = math.atan2(sin_t2, cos_t2)
        k1 = L1 + L2 * cos_t2
        k2 = L2 * sin_t2
        theta1 = math.atan2(y, x) - math.atan2(k2, k1)

        theta1_deg = math.degrees(theta1)
        theta2_deg = math.degrees(theta2)

        self.theta1_label.setText(f"θ1 = {theta1_deg:.1f}°")
        self.theta2_label.setText(f"θ2 = {theta2_deg:.1f}°")

        # Animación + movimiento físico
        t1_ini = self.theta1
        t2_ini = self.theta2
        for i in range(steps + 1):
            t1_i = t1_ini + (theta1 - t1_ini) * (i / steps)
            t2_i = t2_ini + (theta2 - t2_ini) * (i / steps)
            self.canvas.dibujar_robot(t1_i, t2_i, x, y)
            mover_servos(math.degrees(t1_i), math.degrees(t2_i))
            QtWidgets.QApplication.processEvents()
            sleep(0.02)

        self.theta1 = theta1
        self.theta2 = theta2


# =========================
# MAIN
# =========================
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RobotApp()
    window.show()
    sys.exit(app.exec_())
