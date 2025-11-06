# main.py — versión robusta que detecta nombres de widgets del UI
import sys
import math
import time
import traceback
from PyQt5 import QtWidgets, uic
from PyQt5.QtWidgets import QApplication, QMessageBox, QVBoxLayout, QWidget
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
    print("✅ AngularServo disponible (modo físico).")
except Exception as e:
    SERVO_ENABLED = False
    print("⚠️ AngularServo no disponible, modo simulación. (", e, ")")

# ---------- Parámetros del robot ----------
L1 = 10.0
L2 = 11.0

# offsets / direcciones (ajústalos al calibrar)
OFFSET_THETA1 = 90.0
OFFSET_THETA2 = 90.0
DIR_THETA1 = 1.0
DIR_THETA2 = 1.0

# ---------- Inicializar servos si es posible ----------
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
        print("⚠️ Falló inicializar servos:", e)
        SERVO_ENABLED = False
        servo1 = servo2 = None

# ---------- Utilities para buscar widgets con nombres alternativos ----------
def find_first_attr(obj, names):
    """Devuelve el primer atributo que exista en obj de la lista names, o None."""
    for n in names:
        if hasattr(obj, n):
            return getattr(obj, n)
    return None

# ---------- Canvas de matplotlib ----------
class RobotCanvas(FigureCanvas):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        self.fig, self.ax = plt.subplots(figsize=(width, height), dpi=dpi)
        super().__init__(self.fig)
        self.ax.set_aspect('equal')
        self.ax.grid(True)
        self.line, = self.ax.plot([], [], 'o-', lw=3)

    def update(self, theta1_deg, theta2_deg, target_xy=None):
        # recibe grados; la simulación usa radianes internamente
        t1 = math.radians(theta1_deg)
        t2 = math.radians(theta2_deg)
        x1 = L1 * math.cos(t1)
        y1 = L1 * math.sin(t1)
        x2 = x1 + L2 * math.cos(t1 + t2)
        y2 = y1 + L2 * math.sin(t1 + t2)
        self.ax.clear()
        self.ax.plot([0, x1, x2], [0, y1, y2], 'o-', lw=3)
        if target_xy is not None:
            self.ax.plot([target_xy[0]], [target_xy[1]], 'rx')
        rng = L1 + L2 + 2
        self.ax.set_xlim(-rng, rng)
        self.ax.set_ylim(-rng, rng)
        self.ax.set_xlabel("X (cm)")
        self.ax.set_ylabel("Y (cm)")
        self.ax.grid(True)
        self.draw()

# ---------- Funciones de cinemática ----------
def inverse_kinematics(x, y, elbow_down=True):
    r = math.hypot(x, y)
    if r > (L1 + L2) or r < abs(L1 - L2):
        return None
    cos_t2 = (x*x + y*y - L1*L1 - L2*L2) / (2*L1*L2)
    cos_t2 = max(-1.0, min(1.0, cos_t2))
    if elbow_down:
        t2 = -math.acos(cos_t2)
    else:
        t2 = math.acos(cos_t2)
    k1 = L1 + L2*math.cos(t2)
    k2 = L2*math.sin(t2)
    t1 = math.atan2(y, x) - math.atan2(k2, k1)
    return math.degrees(t1), math.degrees(t2)

# ---------- Mover servos físicos (con offsets y dirección) ----------
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
        print("⚠️ error asignando angulos a servos:", e)
    if debug:
        print(f"[HARD] servo angles set -> base: {ang1:.1f}°, codo: {ang2:.1f}°")
    time.sleep(0.02)

# ---------- Interfaz principal, robusta ----------
class RobotApp(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        # carga UI generado con pyuic5 (archivo .py)
        try:
            from robot_2r_ui import Ui_Form
        except Exception:
            QMessageBox.critical(None, "Error", "No se encontró robot_2r_ui.py. Genéralo con pyuic5.")
            sys.exit(1)

        self.ui = Ui_Form()
        self.ui.setupUi(self)

        # --- Buscar contenedor gráfico entre varias posibilidades ---
        candidate_graph_names = [
            "widgetGrafica", "plot_widget", "plotWidget", "graphicsWidget",
            "centralWidget", "frameGrafica", "frame_plot"
        ]
        graph_container = find_first_attr(self.ui, candidate_graph_names)
        if graph_container is None:
            # si no existe, intentamos buscar un layout en el widget principal y añadir al final
            graph_container = None
            print("⚠️ No se encontró contenedor gráfico por nombre. Se creará uno nuevo dentro del form.")
        else:
            print(f"✅ Contenedor gráfico encontrado: {type(graph_container)} (nombre escogido)")

        # Si se encontró, usamos ese; si no, creamos un widget nuevo y lo insertamos en el form's layout
        if graph_container is None:
            # intentar obtener layout principal del UI y añadir un nuevo widget al final
            main_layout = getattr(self.ui, "verticalLayout", None) or getattr(self.ui, "horizontalLayout", None) or None
            if main_layout is not None:
                # crear widget temporal
                graph_container = QtWidgets.QWidget()
                # si main_layout es QLayout (ya aplicado a un widget), usamos addWidget
                try:
                    main_layout.addWidget(graph_container)
                    print("✅ Se creó e insertó un contenedor gráfico en el layout principal del UI.")
                except Exception as e:
                    print("⚠️ No se pudo añadir al layout principal:", e)
            else:
                # fallback: usar self as container -> pero no sobreescribir layout existente
                graph_container = QtWidgets.QWidget(self)
                self.setCentralWidget(graph_container)
                print("⚠️ Fallback: se creó widgetGrafica como centralWidget.")

        # --- Inserta canvas sin romper layout existente ---
        self.canvas = RobotCanvas()
        existing_layout = graph_container.layout()
        if existing_layout is None:
            # crear layout y asignar
            new_layout = QVBoxLayout(graph_container)
            new_layout.addWidget(self.canvas)
            # no llamamos a graph_container.setLayout(new_layout) porque ya lo hace el constructor de QVBoxLayout(widget)
            print("✅ Layout creado e canvas agregado al contenedor gráfico.")
        else:
            # sólo añadimos el widget al layout existente
            existing_layout.addWidget(self.canvas)
            print("✅ Canvas agregado al layout ya existente del contenedor gráfico.")

        # --- Buscar widgets de control (varios nombres posibles) ---
        x_widget = find_first_attr(self.ui, ["x_input", "txtX", "inputX", "lineEditX", "leX", "xLineEdit"])
        y_widget = find_first_attr(self.ui, ["y_input", "txtY", "inputY", "lineEditY", "leY"])
        move_button = find_first_attr(self.ui, ["move_button", "btnMover", "pushButton", "moveButton"])
        elbow_combo = find_first_attr(self.ui, ["elbow_combo", "comboCodo", "codo_combo"])
        steps_widget = find_first_attr(self.ui, ["steps_input", "txtSteps", "stepsLine", "spinSteps"])
        theta1_label = find_first_attr(self.ui, ["theta1_label", "lblTheta1", "labelTheta1", "theta1"])
        theta2_label = find_first_attr(self.ui, ["theta2_label", "lblTheta2", "labelTheta2", "theta2"])

        # Informar qué encontró
        print("Detector de widgets -> x:", bool(x_widget), "y:", bool(y_widget),
              "move:", bool(move_button), "elbow:", bool(elbow_combo),
              "steps:", bool(steps_widget), "t1:", bool(theta1_label), "t2:", bool(theta2_label))

        # Si no se encontraron controles, construir controles mínimos en el UI (al final)
        if not all([x_widget, y_widget, move_button, elbow_combo, steps_widget, theta1_label, theta2_label]):
            print("⚠️ No se encontraron todos los widgets de control en el UI. Se crearán controles mínimos en pantalla.")
            # crear controles simples
            controls_widget = QtWidgets.QWidget()
            controls_layout = QtWidgets.QFormLayout(controls_widget)
            if not x_widget:
                x_widget = QtWidgets.QLineEdit()
                controls_layout.addRow("X (cm):", x_widget)
            if not y_widget:
                y_widget = QtWidgets.QLineEdit()
                controls_layout.addRow("Y (cm):", y_widget)
            if not elbow_combo:
                elbow_combo = QtWidgets.QComboBox()
                elbow_combo.addItems(["Codo arriba", "Codo abajo"])
                controls_layout.addRow("Codo:", elbow_combo)
            if not steps_widget:
                steps_widget = QtWidgets.QLineEdit("40")
                controls_layout.addRow("Pasos:", steps_widget)
            if not move_button:
                move_button = QtWidgets.QPushButton("Mover robot")
                controls_layout.addRow(move_button)
            if not theta1_label:
                theta1_label = QtWidgets.QLabel("θ1 = 0°")
                controls_layout.addRow(theta1_label)
            if not theta2_label:
                theta2_label = QtWidgets.QLabel("θ2 = 0°")
                controls_layout.addRow(theta2_label)
            # añadir controles justo después del gráfico
            if graph_container.layout() is not None:
                graph_container.layout().addWidget(controls_widget)
            else:
                # fallback: añadir a la canvas parent
                graph_container.layout().addWidget(controls_widget)

        # ahora guardamos referencias en self para usar luego
        self.x_widget = x_widget
        self.y_widget = y_widget
        self.move_button = move_button
        self.elbow_combo = elbow_combo
        self.steps_widget = steps_widget
        self.t1_label = theta1_label
        self.t2_label = theta2_label

        # conectar
        try:
            self.move_button.clicked.connect(self.on_move_clicked)
        except Exception as e:
            print("⚠️ No se pudo conectar el botón move:", e)

        # estado inicial
        self.theta1 = 0.0
        self.theta2 = 0.0
        self.canvas.update(0.0, 0.0)

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

        # elbow choice
        elbow_txt = self.elbow_combo.currentText() if hasattr(self.elbow_combo, "currentText") else ""
        elbow_down = True
        if "arriba" in elbow_txt.lower() or "up" in elbow_txt.lower():
            elbow_down = False
        # calcular angulos
        res = inverse_kinematics := None
        try:
            angles = inverse_kinematics(x, y, elbow_down=elbow_down)
            if angles is None:
                QMessageBox.warning(self, "Fuera de alcance", "Punto fuera del workspace.")
                return
            theta1_deg, theta2_deg = angles
        except Exception as e:
            QMessageBox.critical(self, "Error IK", str(e))
            return

        # mostrar etiquetas si existen
        if hasattr(self, "t1_label"):
            try:
                self.t1_label.setText(f"θ1 = {theta1_deg:.2f}°")
            except Exception:
                pass
        if hasattr(self, "t2_label"):
            try:
                self.t2_label.setText(f"θ2 = {theta2_deg:.2f}°")
            except Exception:
                pass

        # actualizar simulación
        try:
            self.canvas.update(theta1_deg, theta2_deg, target_xy=(x, y))
        except Exception as e:
            print("⚠️ Error actualizando canvas:", e)

        # mover servos físicos (con debug)
        move_physical_servos(theta1_deg, theta2_deg, debug=True)

# ---------- inicio ----------
if __name__ == "__main__":
    app = QApplication(sys.argv)
    try:
        window = RobotApp()
        window.show()
        sys.exit(app.exec_())
    except Exception as e:
        print("ERROR crítico:", e)
        traceback.print_exc()
        sys.exit(1)
