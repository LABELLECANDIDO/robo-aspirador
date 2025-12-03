import pickle
import numpy as np
import matplotlib.pyplot as plt
import os

MAP_FILENAME = "mapa.pkl"
TRAJ_FILENAME = "trajetoria.pkl"

MAP_METERS = 10.0        # mesmo valor do robo.py
MAP_SIZE = 120

def plotar_mapa_e_trajetoria(mapa, trajetoria, nome_arquivo):
    plt.figure(figsize=(6, 6))
    plt.imshow(mapa.T, cmap="Greys", origin="lower")

    xs = [p[0] for p in trajetoria]
    ys = [p[1] for p in trajetoria]

    # Conversão de coordenadas reais para índices do mapa
    xs_pix = [int(((x + MAP_METERS) / (2 * MAP_METERS)) * MAP_SIZE) for x in xs]
    ys_pix = [int(((y + MAP_METERS) / (2 * MAP_METERS)) * MAP_SIZE) for y in ys]

    plt.plot(xs_pix, ys_pix, color="red", linewidth=2)

    # seta indicando direção inicial
    if len(xs_pix) > 2:
        plt.arrow(xs_pix[1], ys_pix[1],
                  xs_pix[2] - xs_pix[1],
                  ys_pix[2] - ys_pix[1],
                  head_width=3, head_length=4,
                  color="yellow")

    plt.title("Mapa + Trajetória")
    plt.xlabel("X (células)")
    plt.ylabel("Y (células)")
    plt.grid(False)

    plt.savefig(nome_arquivo, dpi=300)
    plt.close()
    print(f"[OK] Arquivo gerado: {nome_arquivo}")


# ------------------------------
# Carregar arquivos
# ------------------------------
if not os.path.exists(MAP_FILENAME):
    print("[ERRO] mapa.pkl não encontrado.")
    exit()

if not os.path.exists(TRAJ_FILENAME):
    print("[ERRO] trajetoria.pkl não encontrado.")
    exit()

with open(MAP_FILENAME, "rb") as f:
    mapa = pickle.load(f)

with open(TRAJ_FILENAME, "rb") as f:
    trajetoria = pickle.load(f)


# ------------------------------
# Salvar figura com numeração automática
# ------------------------------
execucao = 1
while os.path.exists(f"mapa_execucao_{execucao}.png"):
    execucao += 1

nome_figura = f"mapa_execucao_{execucao}.png"
plotar_mapa_e_trajetoria(mapa, trajetoria, nome_figura)

print("\n✔ Figura gerada com sucesso!")
