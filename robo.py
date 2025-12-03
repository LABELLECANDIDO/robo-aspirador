"""
robo.py - Simulação: pista reta com obstáculos no caminho, faixa, lombada e detecção de colisão sem falsos positivos.

Como usar:
    pip install pybullet numpy paho-mqtt
    python3 robo.py
"""

import pybullet as p
import pybullet_data
import numpy as np
import time
import pickle
import json
import os
import math
from collections import deque

# MQTT opcional
try:
    import paho.mqtt.client as mqtt
    MQTT_AVAILABLE = True
except:
    MQTT_AVAILABLE = False

# --------------------------
# Configurações
# --------------------------
MAP_SIZE = 100
MAP_METERS = 8.0        # usamos área maior para certificar espaço na reta
MAP_FILENAME = "mapa.pkl"
TRAJ_FILENAME = "trajetoria.pkl"

MQTT_BROKER = "localhost"
MQTT_PORT = 1883
MQTT_TOPIC = "aspirador/log"

SIM_STEP = 1.0 / 240.0

SENSOR_ANGLES = [-math.pi/6, 0.0, math.pi/6]  # esquerda, frente, direita
SENSOR_MAX_RANGE = 3.0
OBSTACLE_THRESHOLD = 0.35

BASE_SPEED = 4.0
TURN_SPEED = 2.0

# --------------------------
# Utilitárias (mapa / I/O)
# --------------------------
def world_to_cell(pos):
    x, y = pos[0], pos[1]
    i = int(((x + MAP_METERS) / (2 * MAP_METERS)) * MAP_SIZE)
    j = int(((y + MAP_METERS) / (2 * MAP_METERS)) * MAP_SIZE)
    return clamp_cell(i, j)

def clamp_cell(i, j):
    return max(0, min(MAP_SIZE - 1, i)), max(0, min(MAP_SIZE - 1, j))

def carregar_mapa():
    if os.path.exists(MAP_FILENAME):
        try:
            with open(MAP_FILENAME, "rb") as f:
                return pickle.load(f)
        except:
            pass
    return np.zeros((MAP_SIZE, MAP_SIZE), dtype=np.int32)

def salvar_mapa(m):
    with open(MAP_FILENAME, "wb") as f:
        pickle.dump(m, f)

def carregar_trajetoria():
    if os.path.exists(TRAJ_FILENAME):
        try:
            with open(TRAJ_FILENAME, "rb") as f:
                return pickle.load(f)
        except:
            pass
    return []

def salvar_trajetoria(t):
    with open(TRAJ_FILENAME, "wb") as f:
        pickle.dump(t, f)

# --------------------------
# Sensores (raycasts)
# --------------------------
def sensor_distances(robot_id):
    pos, orn = p.getBasePositionAndOrientation(robot_id)
    yaw = p.getEulerFromQuaternion(orn)[2]
    origins = []
    targets = []
    for a in SENSOR_ANGLES:
        ang = yaw + a
        ox, oy, oz = pos[0], pos[1], pos[2] + 0.2
        tx = ox + math.cos(ang) * SENSOR_MAX_RANGE
        ty = oy + math.sin(ang) * SENSOR_MAX_RANGE
        tz = oz
        origins.append((ox, oy, oz))
        targets.append((tx, ty, tz))
    results = p.rayTestBatch(origins, targets)
    dists = []
    for r in results:
        hf = r[2]
        d = hf * SENSOR_MAX_RANGE if hf < 1.0 else SENSOR_MAX_RANGE
        dists.append(d)
    return dists

# --------------------------
# Logger MQTT (opcional)
# --------------------------
class LoggerMQTT:
    def __init__(self):
        self.available = False
        if MQTT_AVAILABLE:
            try:
                self.client = mqtt.Client()
                self.client.connect(MQTT_BROKER, MQTT_PORT, 60)
                self.available = True
            except Exception as e:
                print("[MQTT] erro conectar broker:", e)
                self.available = False
        else:
            print("[MQTT] paho-mqtt não disponível. Logs vão pro console.")

    def publish(self, payload):
        txt = json.dumps(payload)
        if self.available:
            try:
                self.client.publish(MQTT_TOPIC, txt)
            except Exception as e:
                print("[MQTT] falha publicar:", e)
        else:
            print("[LOG]", txt)

# --------------------------
# Detecção de colisão (ignorando IDs seguros)
# --------------------------
def detectar_colisao(robot_id, ignorar_ids):
    contatos = p.getContactPoints(bodyA=robot_id)
    for c in contatos:
        other = c[2]  # bodyUniqueId do outro objeto
        # ignorar contatos com o próprio robô (caso apareça) e com ids na lista
        if other is None:
            continue
        if other not in ignorar_ids and other != robot_id:
            return True
    return False

# --------------------------
# Controle diferencial simples
# --------------------------
def controle_diferencial(distances):
    esq, frente, dir = distances
    if frente < OBSTACLE_THRESHOLD:
        return (-TURN_SPEED, TURN_SPEED) if esq > dir else (TURN_SPEED, -TURN_SPEED)
    erro = esq - dir
    kp = 1.8
    delta = kp * erro
    vl = BASE_SPEED - delta
    vr = BASE_SPEED + delta
    vl = max(-BASE_SPEED, min(BASE_SPEED, vl))
    vr = max(-BASE_SPEED, min(BASE_SPEED, vr))
    return vl, vr

# --------------------------
# Gera pista reta com obstáculos no centro da pista
# --------------------------
def criar_pista_reta_e_obstaculos():
    # retorna dicionário com ids dos objetos criados para registrar ignorados
    objs = {}
    wall_h = 0.4
    wall_t = 0.1

    # paredes laterais (reta longa centrada em y=0, x 6m)
    long_wall = p.createCollisionShape(p.GEOM_BOX, halfExtents=[4.5, wall_t, wall_h])
    left_wall = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=long_wall, basePosition=[0, -2.5, wall_h])
    right_wall = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=long_wall, basePosition=[0, 2.5, wall_h])

    objs['left_wall'] = left_wall
    objs['right_wall'] = right_wall

    # faixa central (linha contínua branca, apenas visual)
    faixa_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[4.5, 0.02, 0.01], rgbaColor=[1,1,1,1])
    faixa_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[4.5, 0.02, 0.01])
    faixa = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=faixa_col, baseVisualShapeIndex=faixa_vis, basePosition=[0, 0, 0.01])
    objs['faixa'] = faixa
    objs['faixa_col'] = faixa_col

    # lombada no começo da reta (leve)
    lomb_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.6, 0.4, 0.05])
    lomb_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.6, 0.4, 0.05], rgbaColor=[0.9,0.6,0.1,1])
    lomb = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=lomb_col, baseVisualShapeIndex=lomb_vis, basePosition=[-3.5, 0, 0.05])
    objs['lombada'] = lomb
    objs['lomb_col'] = lomb_col

    # criar uma série de obstáculos ao longo do centro da pista (vários blocos)
    obst_ids = []
    x_start = -2.5
    spacing = 1.2
    scales = [0.4, 0.3, 0.35, 0.45, 0.3]  # diferentes tamanhos
    colors = [[1,0,0,1],[0,0,1,1],[1,1,0,1],[0.2,0.8,0.2,1],[0.6,0.2,0.9,1]]
    for i, s in enumerate(scales):
        x = x_start + i * spacing
        obs = p.loadURDF("cube.urdf", basePosition=[x, 0.0, s/2.0], globalScaling=s)
        p.changeVisualShape(obs, -1, rgbaColor=colors[i % len(colors)])
        obst_ids.append(obs)
    objs['obst'] = obst_ids

    # obstáculos laterais adicionais (para dificultar desvio)
    side_obs = []
    side_positions = [[-1.2, 1.0, 0.25], [0.8, -1.2, 0.25]]
    for sp in side_positions:
        o = p.loadURDF("cube.urdf", basePosition=sp, globalScaling=0.5)
        p.changeVisualShape(o, -1, rgbaColor=[0.9, 0.4, 0.1,1])
        side_obs.append(o)
    objs['side_obs'] = side_obs

    return objs

# --------------------------
# MAIN
# --------------------------
def main():
    # conectar (forçar opengl2 compatível com WSL sem aceleração)
    p.connect(p.GUI, options="--opengl2")
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0,0,-9.8)
    p.setRealTimeSimulation(0)

    # chão
    plane = p.loadURDF("plane.urdf")

    # criar pista reta + obstáculos
    pista_objs = criar_pista_reta_e_obstaculos()

    # carregar robô em posição segura ANTES do primeiro obstáculo
    SAFE_START = [-5.5, 0.0, 0.12]   # longe na esquerda da reta (x negativo)
    robot = p.loadURDF("husky/husky.urdf", basePosition=SAFE_START)
    wheel_indices = [2,3,4,5]

    # montar lista de objetos a ignorar nas colisões (chão, paredes, faixa, lombada)
    objetos_ignorar = set()
    objetos_ignorar.add(plane)
    # acrescentar paredes
    objetos_ignorar.add(pista_objs['left_wall'])
    objetos_ignorar.add(pista_objs['right_wall'])
    # faixa e lombada (usar os bodyUniqueId retornados)
    objetos_ignorar.add(pista_objs['faixa'])
    objetos_ignorar.add(pista_objs['lombada'])
    # podem-se também ignorar a colisão com os próprios obstáculos? NÃO — queremos detectar colisão com eles.
    # se quiser tratar alguns obstáculos como "não gerar alerta", adicione aqui.

    # carregar mapa/trajetoria
    mapa = carregar_mapa()
    trajetoria = carregar_trajetoria()
    logger = LoggerMQTT()

    # spawn safe: ignorar colisões nos primeiros 0.5s para evitar micro-contactos
    ignore_colisao_ate = time.time() + 0.5

    passos = 0
    recent_dists = deque(maxlen=50)
    print("[SIM] Iniciando simulação - pista reta com obstáculos. Ctrl+C para parar.")

    try:
        while True:
            dists = sensor_distances(robot)
            recent_dists.append(dists)

            # detecção de colisão apenas após o período seguro
            if time.time() > ignore_colisao_ate:
                if detectar_colisao(robot, objetos_ignorar):
                    # apenas alerta quando realmente colidiu com obstáculos (que não estão em objetos_ignorar)
                    print("🚨 ALERTA: colisão REAL detectada!")
                    print("\a")  # beep terminal
                    logger.publish({"event":"colisao_detectada", "pos": p.getBasePositionAndOrientation(robot)[0]})

            # controle e movimento
            vl, vr = controle_diferencial(dists)
            # aplicar velocidades: husky tem 4 rodas indices 2..5
            p.setJointMotorControl2(robot, 2, p.VELOCITY_CONTROL, targetVelocity=vl)
            p.setJointMotorControl2(robot, 3, p.VELOCITY_CONTROL, targetVelocity=vr)
            p.setJointMotorControl2(robot, 4, p.VELOCITY_CONTROL, targetVelocity=vl)
            p.setJointMotorControl2(robot, 5, p.VELOCITY_CONTROL, targetVelocity=vr)

            p.stepSimulation()
            time.sleep(SIM_STEP)

            pos, _ = p.getBasePositionAndOrientation(robot)
            i,j = world_to_cell(pos)
            # atualizar mapa somente se célula válida
            mapa[i,j] += 1
            trajetoria.append((pos[0], pos[1]))

            passos += 1
            # log periódico
            if passos % 120 == 0:
                payload = {
                    "steps": passos,
                    "pos": {"x": pos[0], "y": pos[1]},
                    "dists": dists,
                    "area_pct": float(np.sum(mapa > 0)) / (MAP_SIZE*MAP_SIZE) * 100.0,
                    "recent_dists": list(recent_dists)[-5:]
                }
                logger.publish(payload)

    except KeyboardInterrupt:
        print("\n[SIM] Interrompido pelo usuário. Salvando dados...")
        salvar_mapa(mapa)
        salvar_trajetoria(trajetoria)
        print("Resumo:")
        print(" - passos:", passos)
        print(" - distância total (aprox):", sum(math.hypot(trajetoria[i][0]-trajetoria[i-1][0],
                                                           trajetoria[i][1]-trajetoria[i-1][1]) for i in range(1,len(trajetoria))) if len(trajetoria)>1 else 0.0)
        logger.publish({"final_summary": {"steps": passos}})
    finally:
        p.disconnect()

if __name__ == "__main__":
    main()
