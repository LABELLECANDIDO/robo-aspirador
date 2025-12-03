"""
robo.py - Simulação robusta:
 - pista reta com obstáculos
 - sensores por raycasts com marcação de obstáculos no mapa
 - mapeamento em grid (MAP_SIZE x MAP_SIZE)
 - registro de trajetórias
 - seleção/uso de melhor trajetória anterior como "roteiro aprendido"
 - controle diferencial para seguir waypoints (se houver trajetória aprendida)
 - evita colisões e reduz falsos positivos ignorando objetos listados
 - não fecha mais por exceção; encerra quando janela GUI é fechada
 - melhorias: exploração aleatória, detecção de "stuck", critério de custo (comprimento+energia)
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
import matplotlib.pyplot as plt
import numpy as np

def plot_map_and_traj(mapa, trajetoria):
    h, w = mapa.shape

    # gera imagem normalizada
    img = mapa.astype(np.float32)
    img = img / img.max() if img.max() > 0 else img

    plt.figure(figsize=(8,8))
    plt.imshow(img.T, origin='lower', cmap='gray')

    # converter posições reais (metros) para índices de matriz
    xs = [int(((x + MAP_METERS) / (2 * MAP_METERS)) * w) for x,y in trajetoria]
    ys = [int(((y + MAP_METERS) / (2 * MAP_METERS)) * h) for x,y in trajetoria]

    plt.plot(xs, ys, 'r', linewidth=2)
    plt.title("Mapa + Trajetória")
    plt.xlabel("X (células)")
    plt.ylabel("Y (células)")
    plt.grid(False)
    plt.show()


# MQTT opcional
try:
    import paho.mqtt.client as mqtt
    MQTT_AVAILABLE = True
except:
    MQTT_AVAILABLE = False

# --------------------------
# Configurações
# --------------------------
MAP_SIZE = 120
MAP_METERS = 10.0        # área considerada (x,y) ∈ [-MAP_METERS, MAP_METERS]
MAP_FILENAME = "mapa.pkl"
TRAJ_FILENAME = "trajetoria.pkl"      # armazena lista de trajetórias (lista de listas)
BEST_TRAJ_FILENAME = "best_traj.pkl"

MQTT_BROKER = "localhost"
MQTT_PORT = 1883
MQTT_TOPIC = "aspirador/log"

SIM_STEP = 1.0 / 240.0

SENSOR_ANGLES = [-math.pi/6, 0.0, math.pi/6]  # esquerda, frente, direita
SENSOR_MAX_RANGE = 3.0
OBSTACLE_THRESHOLD = 0.4   # distancia considerada obstáculo

BASE_SPEED = 4.0
TURN_SPEED = 2.0

WAYPOINT_REACH_TOL = 0.25  # quando considerar que alcançou um waypoint
WAYPOINT_FOLLOW_GAIN = 1.6

# stuck detection / escape
STUCK_MOVE_TOL = 0.03     # se não se moveu mais que isso
STUCK_CHECK_INTERVAL = 1.2
STUCK_MAX_COUNT = 3
ESCAPE_REVERSE_TIME = 0.8
ESCAPE_ROTATE_TIME = 1.0

# critérios de custo ao salvar best traj
ENERGY_WEIGHT = 0.08   # peso relativo da energia no custo combinado

# margem extra para reação imediata quando sensores detectam proximidade
REACTIVE_MARGIN = 0.35
# limiares para reação em camadas
DESVIO_MARGIN = 0.12   # desvio leve quando moderadamente próximo
ESCAPE_MARGIN = 0.02   # escape quando criticamente perto

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
                m = pickle.load(f)
                # Se o mapa salvo tem shape compatível, adapta/normaliza
                if isinstance(m, np.ndarray):
                    if m.shape == (MAP_SIZE, MAP_SIZE):
                        return m.astype(np.int32)
                    # tentar converter mapas de dimensões diferentes (ex.: anteriores)
                    try:
                        resized = np.zeros((MAP_SIZE, MAP_SIZE), dtype=np.int32)
                        min_i = min(m.shape[0], MAP_SIZE)
                        min_j = min(m.shape[1], MAP_SIZE)
                        resized[:min_i, :min_j] = m[:min_i, :min_j].astype(np.int32)
                        return resized
                    except Exception:
                        return np.zeros((MAP_SIZE, MAP_SIZE), dtype=np.int32)
        except Exception:
            pass
    return np.zeros((MAP_SIZE, MAP_SIZE), dtype=np.int32)

def salvar_mapa(m):
    try:
        with open(MAP_FILENAME, "wb") as f:
            pickle.dump(m, f)
    except Exception as e:
        print("[ERRO] ao salvar mapa:", e)

def carregar_trajetorias():
    if os.path.exists(TRAJ_FILENAME):
        try:
            with open(TRAJ_FILENAME, "rb") as f:
                ts = pickle.load(f)
                if isinstance(ts, list):
                    cleaned = []
                    for t in ts:
                        if isinstance(t, list):
                            pts = []
                            for pnt in t:
                                if (isinstance(pnt, (list, tuple)) and len(pnt) >= 2 and
                                        isinstance(pnt[0], (int, float)) and isinstance(pnt[1], (int, float))):
                                    pts.append((float(pnt[0]), float(pnt[1])))
                            if pts:
                                cleaned.append(pts)
                    return cleaned
        except Exception:
            pass
    return []  # lista vazia por padrão

def salvar_trajetorias(ts):
    try:
        with open(TRAJ_FILENAME, "wb") as f:
            pickle.dump(ts, f)
    except Exception as e:
        print("[ERRO] ao salvar trajetorias:", e)

def carregar_best_traj():
    if os.path.exists(BEST_TRAJ_FILENAME):
        try:
            with open(BEST_TRAJ_FILENAME, "rb") as f:
                t = pickle.load(f)
                if isinstance(t, list) and len(t) > 1:
                    pts = []
                    for pnt in t:
                        if (isinstance(pnt, (list, tuple)) and len(pnt) >= 2 and
                                isinstance(pnt[0], (int, float)) and isinstance(pnt[1], (int, float))):
                            pts.append((float(pnt[0]), float(pnt[1])))
                    if pts:
                        return pts
        except Exception:
            pass
    return None

def salvar_best_traj(t):
    try:
        with open(BEST_TRAJ_FILENAME, "wb") as f:
            pickle.dump(t, f)
    except Exception as e:
        print("[ERRO] ao salvar best_traj:", e)

# --------------------------
# Sensores (raycasts)
# --------------------------
def sensor_distances_and_hits(robot_id):
    pos, orn = p.getBasePositionAndOrientation(robot_id)
    yaw = p.getEulerFromQuaternion(orn)[2]

    SENSOR_HEIGHT = 0.25   # altura compatível com obstáculos do cenário

    origins = []
    targets = []

    for a in SENSOR_ANGLES:
        ang = yaw + a

        ox = pos[0]
        oy = pos[1]
        oz = pos[2] + SENSOR_HEIGHT

        tx = ox + math.cos(ang) * SENSOR_MAX_RANGE
        ty = oy + math.sin(ang) * SENSOR_MAX_RANGE
        tz = oz

        origins.append([ox, oy, oz])
        targets.append([tx, ty, tz])

    results = p.rayTestBatch(origins, targets)

    dists = []
    hits = []

    for i, r in enumerate(results):
        hit_fraction = r[2]

        if hit_fraction < 1.0:
            ox, oy, oz = origins[i]
            tx, ty, tz = targets[i]

            hx = ox + (tx - ox) * hit_fraction
            hy = oy + (ty - oy) * hit_fraction
            hz = oz + (tz - oz) * hit_fraction

            dists.append(hit_fraction * SENSOR_MAX_RANGE)
            hits.append((hx, hy, hz))
        else:
            dists.append(SENSOR_MAX_RANGE)
            hits.append(None)

    return dists, hits

    pos, orn = p.getBasePositionAndOrientation(robot_id)
    yaw = p.getEulerFromQuaternion(orn)[2]

    origins = []
    targets = []

    SENSOR_HEIGHT = 0.12  # antes 0.25 — ESTAVA MUITO ALTO

    for a in SENSOR_ANGLES:
        ang = yaw + a

        ox = pos[0]
        oy = pos[1]
        oz = pos[2] + SENSOR_HEIGHT

        tx = ox + math.cos(ang) * SENSOR_MAX_RANGE
        ty = oy + math.sin(ang) * SENSOR_MAX_RANGE
        tz = oz  # raycast horizontal

        origins.append((ox, oy, oz))
        targets.append((tx, ty, tz))

    results = p.rayTestBatch(origins, targets)

    dists = []
    hits = []

    for idx, r in enumerate(results):
        hf = r[2]

        if hf < 1.0:
            ox, oy, oz = origins[idx]
            tx, ty, tz = targets[idx]
            hitx = ox + (tx - ox) * hf
            hity = oy + (ty - oy) * hf
            hitz = oz + (tz - oz) * hf
            d = hf * SENSOR_MAX_RANGE
            hits.append((hitx, hity, hitz))
            dists.append(d)
        else:
            hits.append(None)
            dists.append(SENSOR_MAX_RANGE)

    return dists, hits


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
            pass

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
# agora considera força normal para reduzir falsos positivos
# --------------------------
def detectar_colisao(robot_id, ignorar_ids, min_normal_force=10.0):
    try:
        contatos = p.getContactPoints(bodyA=robot_id)
    except Exception:
        return False
    for c in contatos:
        other = c[2] if len(c) > 2 else None
        normal_force = c[9] if len(c) > 9 else 0.0
        if other is None:
            continue
        if other not in ignorar_ids and other != robot_id and normal_force > min_normal_force:
            return True
    return False

# --------------------------
# Controle diferencial simples
# --------------------------
def controle_diferencial(distances):
    esq, frente, dir = distances

    # --- REGRA 1: se há obstáculo na frente, GIRA IMEDIATAMENTE ---
    if frente < OBSTACLE_THRESHOLD:
        # escolhe lado com maior espaço
        if esq > dir:
            return -TURN_SPEED, TURN_SPEED   # gira para a esquerda
        else:
            return TURN_SPEED, -TURN_SPEED   # gira para a direita

    # --- REGRA 2: seguir em linha reta e fazer micro-ajustes ---
    erro = esq - dir
    kp = 4.0   # antes 1.8 → insuficiente
    delta = kp * erro

    vl = BASE_SPEED - delta
    vr = BASE_SPEED + delta

    # saturação para evitar velocidades altas
    max_speed = BASE_SPEED
    vl = max(-max_speed, min(max_speed, vl))
    vr = max(-max_speed, min(max_speed, vr))

    return vl, vr


# --------------------------
# Controle para seguir waypoints (PD simples em orientação)
# --------------------------
def follow_waypoint_control(robot_id, waypoint):
    pos, orn = p.getBasePositionAndOrientation(robot_id)
    yaw = p.getEulerFromQuaternion(orn)[2]
    dx = waypoint[0] - pos[0]
    dy = waypoint[1] - pos[1]
    dist = math.hypot(dx, dy)
    desired_yaw = math.atan2(dy, dx)
    err = (desired_yaw - yaw + math.pi) % (2*math.pi) - math.pi
    forward = BASE_SPEED * max(0.4, 1.0 - abs(err)*0.8)
    turn = WAYPOINT_FOLLOW_GAIN * err
    vl = forward - turn
    vr = forward + turn
    vl = max(-BASE_SPEED, min(BASE_SPEED, vl))
    vr = max(-BASE_SPEED, min(BASE_SPEED, vr))
    return vl, vr, dist


def exploration_control_using_map(robot_id, mapa, dists):
    """Escolhe velocidade com base no mapa: prefere células com menor contagem
    e penaliza células muito marcadas (prováveis obstáculos). Retorna (vl, vr).
    """
    pos, orn = p.getBasePositionAndOrientation(robot_id)
    yaw = p.getEulerFromQuaternion(orn)[2]

    # offsets in meters (forward, forward-left, forward-right)
    offsets = [ (0.6, 0.0), (0.45, 0.35), (0.45, -0.35) ]
    angles = [0.0, math.pi/6, -math.pi/6]
    candidates = []

    for (dx, dy), ang in zip(offsets, angles):
        gx = pos[0] + (math.cos(yaw + ang) * dx - math.sin(yaw + ang) * dy)
        gy = pos[1] + (math.sin(yaw + ang) * dx + math.cos(yaw + ang) * dy)
        ci, cj = world_to_cell((gx, gy, 0))
        # penalizar células com valores altos (obstáculos ou muito visitadas)
        val = int(mapa[ci, cj])
        if val == -1:
            # penaliza apenas células marcadas explicitamente como obstáculo
            candidates.append(1e6)
        else:
            candidates.append(val)

    min_idx = int(np.argmin(candidates))

    # comportamento simples baseado no candidato escolhido
    if min_idx == 0:
        # preferir seguir reto; usar micro-ajuste proporcional à diferença dos sensores
        erro = dists[0] - dists[2]
        kp = 1.6
        delta = kp * erro
        vl = BASE_SPEED - delta
        vr = BASE_SPEED + delta
    elif min_idx == 1:
        vl = BASE_SPEED * 0.35
        vr = BASE_SPEED * 1.0
    else:
        vl = BASE_SPEED * 1.0
        vr = BASE_SPEED * 0.35

    # saturação
    vl = max(-BASE_SPEED, min(BASE_SPEED, vl))
    vr = max(-BASE_SPEED, min(BASE_SPEED, vr))
    return vl, vr

# --------------------------
# Gera pista reta com obstáculos no centro da pista
# --------------------------
def criar_pista_reta_e_obstaculos():
    objs = {}
    wall_h = 0.4
    wall_t = 0.1

    long_wall_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[4.5, wall_t, wall_h])
    left_wall = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=long_wall_col, basePosition=[0, -2.5, wall_h])
    right_wall = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=long_wall_col, basePosition=[0, 2.5, wall_h])

    objs['left_wall'] = left_wall
    objs['right_wall'] = right_wall

    faixa_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[4.5, 0.02, 0.01], rgbaColor=[1,1,1,1])
    faixa_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[4.5, 0.02, 0.01])
    faixa = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=faixa_col, baseVisualShapeIndex=faixa_vis, basePosition=[0, 0, 0.01])
    objs['faixa'] = faixa
    objs['faixa_col'] = faixa_col

    # lombada (plataforma amarela) removida conforme solicitado

    obst_ids = []
    x_start = -2.5
    spacing = 1.2
    scales = [0.4, 0.3, 0.35, 0.45, 0.3]
    colors = [[1,0,0,1],[0,0,1,1],[1,1,0,1],[0.2,0.8,0.2,1],[0.6,0.2,0.9,1]]
    for i, s in enumerate(scales):
        x = x_start + i * spacing
        obs = p.loadURDF("cube.urdf", basePosition=[x, 0.0, s/2.0], globalScaling=s)
        p.changeVisualShape(obs, -1, rgbaColor=colors[i % len(colors)])
        obst_ids.append(obs)
    objs['obst'] = obst_ids

    side_obs = []
    side_positions = [[-1.2, 1.0, 0.25], [0.8, -1.2, 0.25]]
    for sp in side_positions:
        o = p.loadURDF("cube.urdf", basePosition=sp, globalScaling=0.5)
        p.changeVisualShape(o, -1, rgbaColor=[0.9, 0.4, 0.1,1])
        side_obs.append(o)
    objs['side_obs'] = side_obs

    return objs

# --------------------------
# Métricas trajetórias / utilitários (protegidos contra entradas inválidas)
# --------------------------
def traj_length(traj):
    if not isinstance(traj, list) or len(traj) < 2:
        return float('inf')
    s = 0.0
    for i in range(1, len(traj)):
        try:
            x1, y1 = traj[i-1]
            x2, y2 = traj[i]
            s += math.hypot(x2 - x1, y2 - y1)
        except Exception:
            return float('inf')
    return s

def choose_best_traj(trajetorias):
    if not trajetorias:
        return None
    validas = []
    for t in trajetorias:
        if isinstance(t, list) and len(t) > 1:
            ok = True
            for pnt in t:
                if not (isinstance(pnt, (list, tuple)) and len(pnt) >= 2 and
                        isinstance(pnt[0], (int, float)) and isinstance(pnt[1], (int, float))):
                    ok = False
                    break
            if ok:
                validas.append([(float(pt[0]), float(pt[1])) for pt in t])
    if not validas:
        return None
    try:
        return min(validas, key=lambda t: traj_length(t))
    except Exception:
        return None

# --------------------------
# MAIN
# --------------------------
def main():
    p.connect(p.GUI, options="--opengl2")
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0,0,-9.8)
    p.setRealTimeSimulation(0)

    plane = p.loadURDF("plane.urdf")
    pista_objs = criar_pista_reta_e_obstaculos()

    SAFE_START = [-5.5, 0.0, 0.12]
    robot = p.loadURDF("husky/husky.urdf", basePosition=SAFE_START, baseOrientation=p.getQuaternionFromEuler([0,0,0]))
    wheel_indices = [2,3,4,5]

    objetos_ignorar = set()
    objetos_ignorar.add(plane)
    objetos_ignorar.add(pista_objs['left_wall'])
    objetos_ignorar.add(pista_objs['right_wall'])
    objetos_ignorar.add(pista_objs['faixa'])
    # lombada removida — não há objeto 'lombada' em pista_objs agora

    mapa = np.zeros((MAP_SIZE, MAP_SIZE), dtype=np.int32)
    trajetorias_anteriores = carregar_trajetorias()
    best_prev = carregar_best_traj()
    logger = LoggerMQTT()

    if best_prev is None:
        best_prev = choose_best_traj(trajetorias_anteriores)

    learned_waypoints = None
    if best_prev:
        if len(best_prev) > 30:
            step = max(1, len(best_prev)//30)
            learned_waypoints = [best_prev[i] for i in range(0, len(best_prev), step)]
        else:
            learned_waypoints = list(best_prev)
        print("[LEARN] Carregada trajetória aprendida com", len(learned_waypoints), "waypoints.")

    trajetoria_atual = []
    ignore_colisao_ate = time.time() + 0.5
    passos = 0
    recent_dists = deque(maxlen=50)

    waypoint_idx = 0
    using_waypoints = bool(learned_waypoints)

    print("[SIM] Iniciando simulação — feche a janela do PyBullet para encerrar corretamente.")
    start_time = time.time()
    energy = 0.0

    # stuck detection vars
    last_pos = None
    last_check_time = time.time()
    stuck_count = 0
    escaping_until = 0.0

    # exploration heading when no waypoints
    next_heading_time = 0.0
    current_heading = None

    try:
        while True:
            try:
                conn_info = p.getConnectionInfo()
                if conn_info is None:
                    print("[SIM] conexão PyBullet perdida (getConnectionInfo retornou None). Encerrando.")
                    break
                if not p.isConnected():
                    print("[SIM] p.isConnected() == False. Encerrando.")
                    break
            except Exception:
                print("[SIM] exceção ao checar conexão GUI. Encerrando.")
                break

            try:
                dists, hits = sensor_distances_and_hits(robot)
            except Exception as e:
                print("[SIM] erro ao ler sensores:", e)
                break

            recent_dists.append(dists)

            # marcar obstáculos no mapa usando hits (saturar valores)
            for h in hits:
                if h is not None:
                    i,j = world_to_cell(h)
                    # marcação leve por detecção por ray (não tratar como obstáculo)
                    mapa[i,j] = min(mapa[i,j] + 5, 255)


            # Reação em camadas (tiered): aproximação permitida; se estiver
            # moderadamente perto -> desvio suave; se estiver criticamente perto -> escape.
            try:
                min_dist = min(dists)
            except Exception:
                min_dist = SENSOR_MAX_RANGE

            # inicializa flags de controle reativo desta iteração
            force_diff = False
            diff_vl = diff_vr = None

            # escape crítico
            if min_dist < (OBSTACLE_THRESHOLD + ESCAPE_MARGIN):
                now_t = time.time()
                if now_t >= escaping_until:
                    escaping_until = now_t + ESCAPE_REVERSE_TIME + ESCAPE_ROTATE_TIME
                    # comando imediato para ré mais forte
                    try:
                        p.setJointMotorControl2(robot, 2, p.VELOCITY_CONTROL, targetVelocity=-2.5)
                        p.setJointMotorControl2(robot, 3, p.VELOCITY_CONTROL, targetVelocity=-2.5)
                        p.setJointMotorControl2(robot, 4, p.VELOCITY_CONTROL, targetVelocity=-2.5)
                        p.setJointMotorControl2(robot, 5, p.VELOCITY_CONTROL, targetVelocity=-2.5)
                    except Exception:
                        pass
                    try:
                        logger.publish({"event": "proximity_escape", "min_dist": float(min_dist)})
                    except Exception:
                        pass
            # desvio leve
            elif min_dist < (OBSTACLE_THRESHOLD + DESVIO_MARGIN):
                # sinaliza para usar controle diferencial mais adiante
                force_diff = True
                try:
                    diff_vl, diff_vr = controle_diferencial(dists)
                except Exception:
                    force_diff = False

            # colisão real após período seguro
            if time.time() > ignore_colisao_ate:
                if detectar_colisao(robot, objetos_ignorar, min_normal_force=8.0):
                    print("🚨 ALERTA: colisão REAL detectada!")
                    print("\a")
                    logger.publish({"event":"colisao_detectada", "pos": p.getBasePositionAndOrientation(robot)[0]})
                    # marcar no mapa as posições de contato como obstáculos (-1)
                    try:
                        contatos = p.getContactPoints(bodyA=robot)
                        for c in contatos:
                            other = c[2] if len(c) > 2 else None
                            normal_force = c[9] if len(c) > 9 else 0.0
                            if other is None:
                                continue
                            if other in objetos_ignorar or other == robot:
                                continue
                            if normal_force <= 8.0:
                                continue
                            # posição do contato (usar positionOnA se disponível)
                            pos_cont = None
                            try:
                                pos_cont = c[5] if len(c) > 5 else None
                            except Exception:
                                pos_cont = None
                            if pos_cont:
                                oi, oj = world_to_cell(pos_cont)
                                oi, oj = clamp_cell(oi, oj)
                                mapa[oi, oj] = -1
                    except Exception:
                        pass

            # se estiver em manobra de escape, sobrescreve comandos
            now = time.time()
            if now < escaping_until:
                # durante escape: ré por um tempo curto + rotação
                # aplicar velocidades de escape: retornar e girar
                if now < escaping_until - ESCAPE_ROTATE_TIME:
                    vl = -1.6
                    vr = -1.6
                else:
                    vl = -TURN_SPEED
                    vr = TURN_SPEED
            else:
                # controle normal (waypoints ou exploração)
                if using_waypoints and learned_waypoints and waypoint_idx < len(learned_waypoints):
                    wp = learned_waypoints[waypoint_idx]
                    vl, vr, dist_wp = follow_waypoint_control(robot, wp)
                    if dist_wp < WAYPOINT_REACH_TOL:
                        waypoint_idx += 1
                        if waypoint_idx >= len(learned_waypoints):
                            using_waypoints = False
                            print("[LEARN] Waypoints completados. Voltando à exploração livre.")
                    if dists[1] < OBSTACLE_THRESHOLD + 0.1:
                        vlf, vrf = controle_diferencial(dists)
                        vl, vr = vlf, vrf
                else:
                    # exploração livre: usa mapa para preferir células menos visitadas
                    try:
                        # se sensor frontal muito próximo, usar controle reativo
                        if dists[1] < OBSTACLE_THRESHOLD + 0.08:
                            vl, vr = controle_diferencial(dists)
                        else:
                            vl, vr = exploration_control_using_map(robot, mapa, dists)
                    except Exception:
                        # fallback reativo
                        vl, vr = controle_diferencial(dists)

            # aplicar velocidades nas rodas
            try:
                p.setJointMotorControl2(robot, 2, p.VELOCITY_CONTROL, targetVelocity=vl)
                p.setJointMotorControl2(robot, 3, p.VELOCITY_CONTROL, targetVelocity=vr)
                p.setJointMotorControl2(robot, 4, p.VELOCITY_CONTROL, targetVelocity=vl)
                p.setJointMotorControl2(robot, 5, p.VELOCITY_CONTROL, targetVelocity=vr)
            except Exception:
                print("[SIM] erro ao definir motores (possível desconexão). Encerrando.")
                break

            p.stepSimulation()
            time.sleep(SIM_STEP)

            # ler posição (protegido)
            try:
                pos, _ = p.getBasePositionAndOrientation(robot)
            except Exception:
                print("[SIM] erro ao ler posição do robô. Encerrando.")
                break

            # atualizar mapa (posição do robô)
            i,j = world_to_cell(pos)
            mapa[i,j] = min(255, mapa[i,j] + 1)
            trajetoria_atual.append((pos[0], pos[1]))

            energy += (abs(vl) + abs(vr)) * SIM_STEP
            passos += 1

            # stuck detection periodicamente
            if last_pos is None:
                last_pos = pos
                last_check_time = now
            elif now - last_check_time > STUCK_CHECK_INTERVAL:
                moved = math.hypot(pos[0] - last_pos[0], pos[1] - last_pos[1])
                last_check_time = now
                last_pos = pos
                if moved < STUCK_MOVE_TOL:
                    stuck_count += 1
                    if stuck_count >= STUCK_MAX_COUNT:
                        # iniciar escape: ré + rotação
                        escaping_until = now + ESCAPE_REVERSE_TIME + ESCAPE_ROTATE_TIME
                        stuck_count = 0
                        print("[SIM] Detected stuck -> executando manobra de escape")
                else:
                    stuck_count = 0

            # publicar status a cada 120 passos
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
        print("\n[SIM] Interrompido via KeyboardInterrupt pelo usuário.")
    except Exception as e:
        print("\n[SIM] Exceção inesperada:", e)
    finally:
        total_time = time.time() - start_time
    print("[SIM] Encerrando, salvando dados...")

    # ---------------------------------------------------
    # 1. Salvar mapa da execução com timestamp único
    # ---------------------------------------------------
    try:
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        mapa_filename = f"mapa_execucao_{timestamp}.pkl"
        with open(mapa_filename, "wb") as f:
            pickle.dump(mapa, f)
        print(f"[IO] mapa salvo em {mapa_filename}")
    except Exception as e:
        print("[ERRO] salvando mapa:", e)

    # ---------------------------------------------------
    # 2. Salvar trajetória da execução
    # ---------------------------------------------------
    try:
        if trajetoria_atual:
            trajetorias_anteriores.append(trajetoria_atual)
        salvar_trajetorias(trajetorias_anteriores)
    except Exception as e:
        print("[ERRO] salvando trajetórias:", e)

    # ---------------------------------------------------
    # 3. Avaliação da execução / aprendizado
    # ---------------------------------------------------
    try:
        curr_len = traj_length(trajetoria_atual) if trajetoria_atual else float('inf')
        prev_len = traj_length(best_prev) if best_prev else float('inf')
        print("Resumo:")
        print(" - passos:", passos)
        print(" - tempo (s):", round(total_time,2))

        if curr_len != float('inf'):
            print(" - comprimento traj atual (m):", round(curr_len,3))
        else:
            print(" - comprimento traj atual: inválido")

        print(" - energia (simulada):", round(energy,3))

        curr_cost = (curr_len if curr_len!=float('inf') else 1e9) + ENERGY_WEIGHT * energy
        prev_cost = (prev_len if prev_len!=float('inf') else 1e9)

        if curr_cost + 1e-6 < prev_cost:
            salvar_best_traj(trajetoria_atual)
            print(" -> Trajetória atual é melhor. Atualizada como best_traj.")
        else:
            print(" -> Best_traj mantida.")

        logger.publish({
            "final_summary": {
                "steps": passos,
                "time": total_time,
                "traj_len": (curr_len if curr_len!=float('inf') else None),
                "energy": energy
            }
        })

    except Exception as e:
        print("[ERRO] ao avaliar trajeto:", e)

    # ---------------------------------------------------
    # 4. Encerrar PyBullet
    # ---------------------------------------------------
    try:
        p.disconnect()
    except:
        pass
