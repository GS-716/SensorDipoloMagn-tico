import serial
import time
import re
import matplotlib.pyplot as plt

PUERTO = "COM4"
BAUD = 115200
NOMBRE_ARCHIVO = "mediciones_sensores.csv"

X_MIN, X_MAX = 100, 300
Y_MIN, Y_MAX = 0, 100

# ===== tu control por Python =====
ANGULO_INI = 0
ANGULO_FIN = 14
STEP_DEG = 2                 # micro-salto (1 o 2 grados)
TIEMPO_TOTAL_MS = 50       # <-- tiempo total del barrido con micro saltos
PAUSA_ENTRE_BARRIDOS = 5.0
REPETICIONES = 1
# =================================

def extraer_dist_y_B(linea: str):
    nums = re.findall(r'[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?', linea)
    if len(nums) < 2: return None
    return float(nums[0]), float(nums[-1])

def enviar(ser, cmd):
    ser.write((cmd + "\n").encode("utf-8"))
    ser.flush()
    print(">>", cmd)

def leer_hasta_done(ser, f, distancias, Bs, line_plot, ax, timeout_s=2.0):
    t0 = time.time()
    while True:
        if time.time() - t0 > timeout_s:
            print("!! Timeout esperando #MOVE_DONE")
            return

        linea = ser.readline().decode("utf-8", errors="ignore").strip()
        if not linea:
            plt.pause(0.001)
            continue

        print(linea)

        if linea.startswith("#MOVE_DONE"):
            return

        if linea.startswith("#"):
            plt.pause(0.001)
            continue

        f.write(linea + "\n"); f.flush()
        res = extraer_dist_y_B(linea)
        if res:
            d, b = res
            distancias.append(d); Bs.append(b)
            line_plot.set_data(distancias, Bs)
            ax.set_xlim(X_MIN, X_MAX); ax.set_ylim(Y_MIN, Y_MAX)
            plt.pause(0.001)

def generar_trayectoria(a0, a1, step):
    if a1 >= a0:
        return list(range(a0, a1 + 1, step))
    else:
        return list(range(a0, a1 - 1, -step))

def main():
    traj = generar_trayectoria(ANGULO_INI, ANGULO_FIN, STEP_DEG)
    n_moves = len(traj) - 1
    if n_moves <= 0:
        print("Trayectoria vacía.")
        return

    # repartir TIEMPO_TOTAL_MS entre micro MOVEs
    tiempo_por_move = max(20, int(TIEMPO_TOTAL_MS / n_moves))  
    # 20ms mínimo para que el servo realmente reciba el PWM

    with serial.Serial(PUERTO, BAUD, timeout=0.05) as ser, \
         open(NOMBRE_ARCHIVO, "w", encoding="utf-8") as f:

        time.sleep(2)
        ser.reset_input_buffer()
        f.write("raw_line\n"); f.flush()

        plt.ion()
        fig, ax = plt.subplots(figsize=(7, 5))
        ax.set_title("|B| vs distancia")
        ax.set_xlabel("Distancia [mm]")
        ax.set_ylabel("|B| [uT]")
        ax.grid(True)
        ax.set_xlim(X_MIN, X_MAX); ax.set_ylim(Y_MIN, Y_MAX)
        line_plot, = ax.plot([], [], marker="o", ls="-", ms=3)

        distancias, Bs = [], []

        for r in range(REPETICIONES):
            print(f"\n===== BARRIDO {r+1}/{REPETICIONES} =====")
            enviar(ser, "ON")

            for ang in traj[1:]:
                enviar(ser, f"MOVE {ang} {tiempo_por_move}")
                # timeout algo mayor al tiempo pedido
                leer_hasta_done(
                    ser, f, distancias, Bs, line_plot, ax,
                    timeout_s=(tiempo_por_move/1000)+1.0
                )

            enviar(ser, "OFF")
            time.sleep(PAUSA_ENTRE_BARRIDOS)

        plt.ioff()
        plt.show()

if __name__ == "__main__":
    main()
