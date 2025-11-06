# main.py — versión robusta y corregida
import sys
import math
import time
import traceback
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QApplication, QMessageBox, QVBoxLayout
import matplotlib
matplotlib.use("Qt5Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

# ---------- Intento cargar control físico (AngularServo) ----------
try:
    from gpiozero import AngularServo
    from gpiozero.pins.pigpio import PiGPIOFactory
    factory = PiGPIOFactory()
    SERVO_ENABLED = True
    print("✅ gpiozero AngularServo OK (modo físico)")
except Exception as e:
    SERVO_ENABLED = False
    print("⚠️ gpiozero no disponible: modo simulación.", e)

# ---------- Parámetros del robot ----------
L1 = 10.0
L2 = 11.0

# offsets / direcciones (ajústalos según calibración real)
OFFSET_THETA1 = 90.0
OFFSET_THETA2 = 90.0
DIR_THETA1 = 1.0
DIR_THETA2 = 1.0

# ---------- Inicializar servos ----------
servo1 = servo2 = None
if SERVO_ENABLED:
    try:
        servo1 = AngularServo(17, min_angle=0, max_angle=180,
                              min_pulse_width=0.5/1000, max_pulse_width=2.5/1000,
                              pin_factory=factory)
        servo2 = AngularServo(18, min_angle=0, max_angle=180,
                              min_pulse_width=0.5/1000, max_pulse_width=2.5/1000,
                              pin_factory=factory)
    except Exception as e:
        print("⚠️ Error iniciando servos:", e)
        SERVO_ENABLED = False

# ---------- Utilidad para buscar widgets ----------
def find_first_attr(obj, names):
    for n in names:
        if hasattr(obj, n):
            return getattr(obj, n)
    return None

# ---------- Canvas de matplotlib ----------
class RobotCanvas(FigureCanvas):
    def __init__(self, parent=None):
        self.fig, self.ax = plt.subplots(figsize=(5, 4), dpi=100)
        super().__init__(self.fig)
        self.ax.set_aspect('equal')
        self.ax.grid(True)
        self.line, = self.ax.plot([], [], 'o-', lw=3)

    def update(self, theta1_deg, theta2_deg, target_xy=None):
        t1 = math.radians(theta1_deg)
        t2 = math.radians(theta2_deg)
        x1 = L1 * math.cos(t1)
        y1 = L1 * math.sin(t1)
        x2 = x1 + L2 * math.cos(t1 + t2)
        y2 = y1 + L2 * math.sin(t1 + t2)
        self.ax.clear()
        self.ax.plot([0, x1, x2], [0, y1, y2], 'o-', lw=3)
        if target_xy:
            self.ax.plot([target_xy[0]], [target_xy[1]], 'rx')
        rng = L1 + L2 + 2
        self.ax.set_xlim(-rng, rng)
        self.ax.set_ylim(-rng, rng)
        self.ax.set_xlabel("X (cm)")
        self.ax.set_ylabel("Y (cm)")
        self.ax.grid(True)
        self.draw()

# ---------- Cinemática inversa ----------
def inverse_kinematics(x, y, elbow_down=True):
    r = math.hypot(x, y)
    if r > (L1 + L2) or r < abs(L1 - L2):
        return None
    cos_t2 = (x*x + y*y - L1*L1 - L2*L2) / (2*L1*L2)
    cos_t2 = max(-1.0, min(1.0, cos_t2))
    t2 = -math.acos(cos_t2) if elbow_down else math.acos(cos_t2)
    k1 = L1 + L2*math.cos(t2)
    k2 = L2*math.sin(t2)
    t1 = math.atan2(y, x) - math.atan2(k2, k1)
    return math.degrees(t1), math.degrees(t2)

# ---------- Mover servos físicos ----------
def move_physical_servos(theta1_deg, theta2_deg, debug=False):
    if not SERVO_ENABLED:
        if debug:
            print("[SIM] mover servos:", theta1_deg, theta2_deg)
        return
    ang1 = OFFSET_THETA1 + DIR_THETA1 * theta1_deg
    ang2 = OFFSET_THETA2 + DIR_THETA2 * theta2_deg
    ang1 = max(0.0, min(180.0, ang1))
    ang2 = max(0.0, min(180.0, ang2))
    try:
        servo1.angle = ang1
        servo2.angle = ang2
    except Exception as e:
        print("⚠️ error asignando ángulos:", e)
    if debug:
        print(f"[HARD] servo1={ang1:.1f}°, servo2={ang2:.1f}°")
    time.sleep(0.02)

# ---------- Aplicación principal ----------
class RobotApp(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        try:
            from robot_2r_ui import Ui_Form
        except Exception:
            QMessageBox.critical(None, "Error", "No se encontró robot_2r_ui.py")
            sys.exit(1)

        self.ui = Ui_Form()
        self.ui.setupUi(self)

        # Buscar contenedor para la gráfica
        container = find_first_attr(self.ui, ["widgetGrafica", "plot_widget", "frameGrafica", "centralwidget"])
        if not container:
            container = QtWidgets.QWidget(self)
            layout = QVBoxLayout(self)
            layout.addWidget(container)
            self.setLayout(layout)
            print("⚠️ No se encontró contenedor, se creó uno nuevo.")

        self.canvas = RobotCanvas()
        layout = container.layout()
        if layout is None:
            layout = QVBoxLayout(container)
        layout.addWidget(self.canvas)

        # Buscar controles
        self.x_widget = find_first_attr(self.ui, ["x_input", "txtX", "lineEditX"])
        self.y_widget = find_first_attr(self.ui, ["y_input", "txtY", "lineEditY"])
        self.move_button = find_first_attr(self.ui, ["move_button", "btnMover", "pushButton"])
        self.elbow_combo = find_first_attr(self.ui, ["elbow_combo", "comboCodo"])
        self.theta1_label = find_first_attr(self.ui, ["theta1_label", "lblTheta1"])
        self.theta2_label = find_first_attr(self.ui, ["theta2_label", "lblTheta2"])

        if self.move_button:
            self.move_button.clicked.connect(self.on_move_clicked)

        self.canvas.update(0, 0)

    def on_move_clicked(self):
        try:
            x = float(self.x_widget.text())
            y = float(self.y_widget.text())
        except Exception:
            QMessageBox.warning(self, "Entrada inválida", "X e Y deben ser numéricos.")
            return

        elbow_down = True
        if self.elbow_combo and "arriba" in self.elbow_combo.currentText().lower():
            elbow_down = False

        result = inverse_kinematics(x, y, elbow_down)
        if result is None:
            QMessageBox.warning(self, "Fuera de alcance", "Punto fuera del espacio de trabajo.")
            return

        theta1, theta2 = result
        self.theta1_label.setText(f"θ1 = {theta1:.1f}°")
        self.theta2_label.setText(f"θ2 = {theta2:.1f}°")

        self.canvas.update(theta1, theta2, target_xy=(x, y))
        move_physical_servos(theta1, theta2, debug=True)

# ---------- main ----------
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RobotApp()
    window.show()
    sys.exit(app.exec_())
