import sys, math, time
from PyQt5 import QtWidgets, uic
from PyQt5.QtWidgets import QMessageBox
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

# ====== Intentar cargar GPIO físico ======
try:
    from gpiozero import AngularServo
    from gpiozero.pins.pigpio import PiGPIOFactory
    factory = PiGPIOFactory()
    servo_enabled = True
    print("✅ gpiozero AngularServo OK (modo físico)")
except Exception as e:
    print("⚠️ gpiozero/AngularServo no disponible:", e)
    servo_enabled = False

# ====== Configuración física de los servos ======
OFFSET_THETA1 = 90    # ángulo donde el brazo está recto (base)
OFFSET_THETA2 = 90    # ángulo donde el segundo brazo está recto
DIR_THETA1 = 1        # invierte a -1 si el servo gira al revés
DIR_THETA2 = 1

if servo_enabled:
    try:
        servo1 = AngularServo(
            17, min_angle=0, max_angle=180,
            min_pulse_width=0.5/1000, max_pulse_width=2.5/1000,
            pin_factory=factory
        )
        servo2 = AngularServo(
            18, min_angle=0, max_angle=180,
            min_pulse_width=0.5/1000, max_pulse_width=2.5/1000,
            pin_factory=factory
        )
    except Exception as e:
        print("⚠️ Error al inicializar servos:", e)
        servo_enabled = False
else:
    servo1 = servo2 = None


# ====== Cinemática Inversa ======
L1 = 10.0
L2 = 11.0

def calcular_angulos(x, y):
    try:
        d = math.sqrt(x**2 + y**2)
        if d > (L1 + L2):
            raise ValueError("Posición fuera del alcance del robot")

        cos_t2 = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
        sin_t2 = math.sqrt(1 - cos_t2**2)
        theta2 = math.atan2(sin_t2, cos_t2)

        k1 = L1 + L2 * cos_t2
        k2 = L2 * sin_t2
        theta1 = math.atan2(y, x) - math.atan2(k2, k1)

        return math.degrees(theta1), math.degrees(theta2)
    except Exception as e:
        print("❌ Error en cinemática:", e)
        return None, None


# ====== Movimiento de Servos ======
def mover_servos(theta1_deg, theta2_deg, debug=False):
    if not servo_enabled:
        if debug:
            print(f"[SIM] θ1={theta1_deg:.2f}, θ2={theta2_deg:.2f}")
        return

    ang1 = OFFSET_THETA1 + DIR_THETA1 * theta1_deg
    ang2 = OFFSET_THETA2 + DIR_THETA2 * theta2_deg

    ang1 = max(0.0, min(180.0, ang1))
    ang2 = max(0.0, min(180.0, ang2))

    try:
        servo1.angle = ang1
        servo2.angle = ang2
    except Exception as e:
        print("⚠️ Error al mover servos:", e)

    if debug:
        print(f"[HARD] servo1={ang1:.1f}°, servo2={ang2:.1f}°")
    time.sleep(0.02)


# ====== Ventana principal ======
class RobotApp(QtWidgets.QMainWindow):
    def __init__(self):
        super(RobotApp, self).__init__()
        from robot_2r_ui import Ui_Form
        self.ui = Ui_Form()
        self.ui.setupUi(self)

        # Crear la figura de simulación
        self.fig, self.ax = plt.subplots()
        self.canvas = FigureCanvas(self.fig)
        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self.canvas)
        self.ui.widgetGrafica.setLayout(layout)

        # Conexiones de botones
        self.ui.btnMover.clicked.connect(self.mover_robot)

    def mover_robot(self):
        try:
            x = float(self.ui.txtX.text())
            y = float(self.ui.txtY.text())
        except ValueError:
            QMessageBox.warning(self, "Error", "Ingrese valores válidos para X y Y")
            return

        theta1, theta2 = calcular_angulos(x, y)
        if theta1 is None:
            QMessageBox.warning(self, "Error", "Posición fuera de alcance")
            return

        # Actualizar etiquetas
        self.ui.lblTheta1.setText(f"{theta1:.1f}°")
        self.ui.lblTheta2.setText(f"{theta2:.1f}°")

        # Actualizar gráfico
        self.actualizar_grafico(theta1, theta2)

        # Mover servos físicos
        mover_servos(theta1, theta2, debug=True)

    def actualizar_grafico(self, t1_deg, t2_deg):
        t1 = math.radians(t1_deg)
        t2 = math.radians(t2_deg)

        x1 = L1 * math.cos(t1)
        y1 = L1 * math.sin(t1)
        x2 = x1 + L2 * math.cos(t1 + t2)
        y2 = y1 + L2 * math.sin(t1 + t2)

        self.ax.clear()
        self.ax.plot([0, x1, x2], [0, y1, y2], 'o-', lw=2)
        self.ax.set_xlim(-L1 - L2, L1 + L2)
        self.ax.set_ylim(-L1 - L2, L1 + L2)
        self.ax.set_xlabel("X (cm)")
        self.ax.set_ylabel("Y (cm)")
        self.ax.set_title("Simulación Robot 2R")
        self.ax.grid(True)
        self.canvas.draw()


# ====== Ejecución principal ======
if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = RobotApp()
    window.show()
    sys.exit(app.exec_())
