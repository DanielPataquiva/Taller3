# main.py — CORREGIDO (renombrado método de dibujo + manejo seguro de layouts)
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

# ---------- Canvas de matplotlib (NO usar nombre 'update') ----------
class RobotCanvas(FigureCanvas):
    def __init__(self, parent=None):
        self.fig, self.ax = plt.subplots(figsize=(5, 4), dpi=100)
        super().__init__(self.fig)
        self.ax.set_aspect('equal')
        self.ax.grid(True)
        self.line, = self.ax.plot([], [], 'o-', lw=3)

    def draw_robot(self, theta1_deg, theta2_deg, target_xy=None):
        """Dibuja el robot — recibe ángulos en grados."""
        t1 = math.radians(theta1_deg)
        t2 = math.radians(theta2_deg)
        x1 = L1 * math.cos(t1)
        y1 = L1 * math.sin(t1)
        x2 = x1 + L2 * math.cos(t1 + t2)
        y2 = y1 + L2 * math.sin(t1 + t2)
        self.ax.clear()
        self.ax.plot([0, x1, x2], [0, y1, y2], 'o-', lw=3)
        if target_xy:
            self.ax.plot([target_xy[0]], [target_xy[1]], 'rx', ms=8)
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
        # Cargar UI generado por pyuic5 (robot_2r_ui.py)
        try:
            from robot_2r_ui import Ui_Form
        except Exception as e:
            QMessageBox.critical(None, "Error", "No se encontró robot_2r_ui.py. Genera el .py desde el .ui con pyuic5.")
            raise e

        self.ui = Ui_Form()
        self.ui.setupUi(self)

        # Buscar contenedor gráfico en el UI (varios nombres comunes)
        container = find_first_attr(self.ui, ["widgetGrafica", "plot_widget", "plotWidget", "graphicsWidget", "frameGrafica", "centralWidget"])
        if container is None:
            # Intenta insertar en un layout raíz si existe
            main_layout = find_first_attr(self.ui, ["verticalLayout", "horizontalLayout", "gridLayout"])
            if main_layout is not None:
                container = QtWidgets.QWidget()
                # Añadir el nuevo contenedor al layout principal
                try:
                    main_layout.addWidget(container)
                    print("✅ Se creó contenedor gráfico y se insertó en el layout principal.")
                except Exception as e:
                    print("⚠️ No se pudo añadir al layout principal:", e)
            else:
                # Fallback: crear widget y establecer como centralWidget del QMainWindow
                container = QtWidgets.QWidget(self)
                self.setCentralWidget(container)
                print("⚠️ Fallback: se creó un contenedor gráfico como centralWidget.")

        # Insertar canvas en el contenedor sin romper su layout
        self.canvas = RobotCanvas()
        existing_layout = container.layout()
        if existing_layout is None:
            new_layout = QVBoxLayout(container)  # esto asigna el layout al widget contenedor
            new_layout.addWidget(self.canvas)
            print("✅ Se creó un layout nuevo en el contenedor y se agregó el canvas.")
        else:
            existing_layout.addWidget(self.canvas)
            print("✅ Se agregó el canvas al layout existente del contenedor.")

        # Buscar controles (varios nombres posibles)
        self.x_widget = find_first_attr(self.ui, ["x_input", "txtX", "inputX", "lineEditX", "txt_x"])
        self.y_widget = find_first_attr(self.ui, ["y_input", "txtY", "inputY", "lineEditY", "txt_y"])
        self.move_button = find_first_attr(self.ui, ["move_button", "btnMover", "pushButton", "moveButton"])
        self.elbow_combo = find_first_attr(self.ui, ["elbow_combo", "comboCodo", "codo_combo"])
        self.steps_widget = find_first_attr(self.ui, ["steps_input", "txtSteps", "stepsLine"])
        self.theta1_label = find_first_attr(self.ui, ["theta1_label", "lblTheta1", "labelTheta1"])
        self.theta2_label = find_first_attr(self.ui, ["theta2_label", "lblTheta2", "labelTheta2"])

        # Si faltan controles, crear controles mínimos (no sobrescriben UI original)
        if not (self.x_widget and self.y_widget and self.move_button):
            print("⚠️ No se encontraron todos los controles esperados; se crearán controles mínimos en pantalla.")
            # crear controles mínimos y añadir al final del contenedor layout
            from PyQt5.QtWidgets import QFormLayout, QLineEdit, QPushButton, QComboBox, QLabel
            controls = QtWidgets.QWidget()
            f = QFormLayout(controls)
            if not self.x_widget:
                self.x_widget = QLineEdit()
                f.addRow("X (cm):", self.x_widget)
            if not self.y_widget:
                self.y_widget = QLineEdit()
                f.addRow("Y (cm):", self.y_widget)
            if not self.elbow_combo:
                self.elbow_combo = QComboBox()
                self.elbow_combo.addItems(["Codo arriba", "Codo abajo"])
                f.addRow("Codo:", self.elbow_combo)
            if not self.steps_widget:
                self.steps_widget = QLineEdit("40")
                f.addRow("Pasos:", self.steps_widget)
            if not self.move_button:
                self.move_button = QPushButton("Mover robot")
                f.addRow(self.move_button)
            if not self.theta1_label:
                self.theta1_label = QLabel("θ1 = 0°")
                f.addRow(self.theta1_label)
            if not self.theta2_label:
                self.theta2_label = QLabel("θ2 = 0°")
                f.addRow(self.theta2_label)
            # añadir controles al layout del contenedor (ya creado)
            cont_layout = container.layout()
            if cont_layout is None:
                cont_layout = QVBoxLayout(container)
            cont_layout.addWidget(controls)

        # conectar botón de forma segura
        try:
            self.move_button.clicked.connect(self.on_move_clicked)
        except Exception as e:
            print("⚠️ No se pudo conectar el botón:", e)

        # estado inicial
        self.theta1 = 0.0
        self.theta2 = 0.0
        # dibujar posición inicial usando el método no conflictivo
        self.canvas.draw_robot(0.0, 0.0)

    def on_move_clicked(self):
        # leer entradas robustamente
        try:
            x_text = self.x_widget.text() if hasattr(self.x_widget, "text") else str(self.x_widget)
            y_text = self.y_widget.text() if hasattr(self.y_widget, "text") else str(self.y_widget)
            x = float(x_text)
            y = float(y_text)
        except Exception as e:
            QMessageBox.warning(self, "Entrada inválida", "X e Y deben ser números. " + str(e))
            return

        # elegir configuración de codo
        elbow_txt = self.elbow_combo.currentText() if hasattr(self.elbow_combo, "currentText") else ""
        elbow_down = True
        if "arriba" in elbow_txt.lower() or "up" in elbow_txt.lower():
            elbow_down = False

        # calcular ángulos
        res = inverse_kinematics(x, y, elbow_down=elbow_down)
        if res is None:
            QMessageBox.warning(self, "Fuera de alcance", "Punto fuera del área de trabajo.")
            return
        theta1_deg, theta2_deg = res

        # mostrar ángulos en labels si existen
        try:
            if self.theta1_label is not None and hasattr(self.theta1_label, "setText"):
                self.theta1_label.setText(f"θ1 = {theta1_deg:.2f}°")
            if self.theta2_label is not None and hasattr(self.theta2_label, "setText"):
                self.theta2_label.setText(f"θ2 = {theta2_deg:.2f}°")
        except Exception:
            pass

        # actualizar simulación
        try:
            self.canvas.draw_robot(theta1_deg, theta2_deg, target_xy=(x, y))
        except Exception as e:
            print("⚠️ Error al dibujar en canvas:", e)

        # mover servos físicos (debug True para ver info en consola)
        move_physical_servos(theta1_deg, theta2_deg, debug=True)

# ---------- main ----------
if __name__ == "__main__":
    app = QApplication(sys.argv)
    try:
        win = RobotApp()
        win.show()
        sys.exit(app.exec_())
    except Exception as e:
        print("ERROR crítico:", e)
        traceback.print_exc()
        sys.exit(1)
