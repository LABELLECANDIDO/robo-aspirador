import pickle
import numpy as np
import matplotlib.pyplot as plt
import os

MAP_FILENAME = "mapa.pkl"
TRAJ_FILENAME = "trajetoria.pkl"

def plotar_mapa_e_trajetoria(mapa, trajetoria, nome_arquivo):
    plt.figure(figsize=(6, 6))
    plt.imshow(mapa.T, cmap="Greys", origin="lower")

    xs = [p[0] for p in trajetoria]
    ys = [p[1] for p in trajetoria]

    xs_pix = [(x + 5) / 10 * mapa.shape[0] for x in xs]
    ys_pix = [(y + 5) / 10 * mapa.shape[1] for y in ys]

    plt.plot(xs_pix, ys_pix, color="red", linewidth=2)

    if len(xs_pix) > 2:
        plt.arrow(xs_pix[1], ys_pix[1],
                  xs_pix[2] - xs_pix[1], ys_pix[2] - ys_pix[1],
                  head_width=3, head_length=4, color="yellow")

    plt.title("Mapa + Trajetória")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.grid(False)

    plt.savefig(nome_arquivo, dpi=300)
    plt.close()
    print(f"[OK] Arquivo gerado: {nome_arquivo}")


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

execucao = 1
while os.path.exists(f"mapa_execucao_{execucao}.png"):
    execucao += 1

nome_figura = f"mapa_execucao_{execucao}.png"
plotar_mapa_e_trajetoria(mapa, trajetoria, nome_figura)

print("\n✔ Figura gerada com sucesso!")
