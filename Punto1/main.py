import sys
import numpy as np
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox, QWidget, QVBoxLayout
from robot_2r_ui import Ui_Form
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from time import sleep

try:
    from gpiozero import Servo
    PHYSICAL_MODE = True
except ImportError:
    print("⚠️ gpiozero no disponible: el control físico se desactiva (solo simulación).")
    PHYSICAL_MODE = False


class MplCanvas(FigureCanvas):
    def __init__(self, parent=None, width=4, height=3, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        self.ax = self.fig.add_subplot(111)
        super().__init__(self.fig)


class RobotApp(QMainWindow, Ui_Form):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

        self.L1 = 10
        self.L2 = 11

        # 🧩 Creamos un contenedor donde irá la simulación
        self.sim_container = QWidget(self)
        self.layout_sim = QVBoxLayout(self.sim_container)
        self.setCentralWidget(self.sim_container)

        # Agregar la gráfica
        self.canvas = MplCanvas(self, width=5, height=4, dpi=100)
        self.layout_sim.addWidget(self.canvas)
        self.ax = self.canvas.ax
        self.ax.set_xlim(-25, 25)
        self.ax.set_ylim(-25, 25)
        self.ax.set_aspect('equal')
        self.ax.grid(True)

        # Crear controles (si no están en el UI)
        self.controls = QWidget(self)
        control_layout = QtWidgets.QFormLayout(self.controls)

        self.inputX = QtWidgets.QLineEdit()
        self.inputY = QtWidgets.QLineEdit()
        self.btnMover = QtWidgets.QPushButton("Mover")
        self.lblTheta1 = QtWidgets.QLabel("θ1 = 0°")
        self.lblTheta2 = QtWidgets.QLabel("θ2 = 0°")

        control_layout.addRow("X (cm):", self.inputX)
        control_layout.addRow("Y (cm):", self.inputY)
        control_layout.addRow(self.btnMover)
        control_layout.addRow(self.lblTheta1)
        control_layout.addRow(self.lblTheta2)

        self.layout_sim.addWidget(self.controls)

        # Conectar botón
        self.btnMover.clicked.connect(self.mover_robot)

        # Servos
        if PHYSICAL_MODE:
            self.servo1 = Servo(17)
            self.servo2 = Servo(18)
        else:
            self.servo1 = None
            self.servo2 = None

        self.theta1 = 0
        self.theta2 = 0
        self.actualizar_grafica()

    def cinematica_inversa(self, x, y):
        r = np.sqrt(x**2 + y**2)
        if r > (self.L1 + self.L2) or r < abs(self.L1 - self.L2):
            raise ValueError("El punto está fuera del área de trabajo.")

        cos_theta2 = (x**2 + y**2 - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
        sin_theta2 = np.sqrt(1 - cos_theta2**2)
        theta2 = np.arctan2(sin_theta2, cos_theta2)

        k1 = self.L1 + self.L2 * cos_theta2
        k2 = self.L2 * sin_theta2
        theta1 = np.arctan2(y, x) - np.arctan2(k2, k1)
        return theta1, theta2

    def cinematica_directa(self, theta1, theta2):
        x1 = self.L1 * np.cos(theta1)
        y1 = self.L1 * np.sin(theta1)
        x2 = x1 + self.L2 * np.cos(theta1 + theta2)
        y2 = y1 + self.L2 * np.sin(theta1 + theta2)
        return (x1, y1), (x2, y2)

    def actualizar_grafica(self):
        self.ax.clear()
        self.ax.set_xlim(-25, 25)
        self.ax.set_ylim(-25, 25)
        self.ax.set_aspect('equal')
        self.ax.grid(True)

        (x1, y1), (x2, y2) = self.cinematica_directa(self.theta1, self.theta2)
        self.ax.plot([0, x1], [0, y1], 'b-', linewidth=4)
        self.ax.plot([x1, x2], [y1, y2], 'g-', linewidth=4)
        self.ax.plot([0, x1, x2], [0, y1, y2], 'ko', markersize=6)
        self.canvas.draw()

    def mover_robot(self):
        try:
            x = float(self.inputX.text())
            y = float(self.inputY.text())

            theta1, theta2 = self.cinematica_inversa(x, y)

            self.lblTheta1.setText(f"θ1 = {np.degrees(theta1):.2f}°")
            self.lblTheta2.setText(f"θ2 = {np.degrees(theta2):.2f}°")

            pasos = 40
            for i in range(1, pasos + 1):
                t1 = self.theta1 + (theta1 - self.theta1) * i / pasos
                t2 = self.theta2 + (theta2 - self.theta2) * i / pasos
                self.theta1, self.theta2 = t1, t2
                self.actualizar_grafica()
                QApplication.processEvents()
                sleep(0.02)

            if PHYSICAL_MODE:
                self.servo1.value = theta1 / np.pi
                self.servo2.value = theta2 / np.pi

        except ValueError as e:
            QMessageBox.warning(self, "Error", str(e))
        except Exception as e:
            QMessageBox.critical(self, "Error inesperado", str(e))


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RobotApp()
    window.show()
    sys.exit(app.exec_())
