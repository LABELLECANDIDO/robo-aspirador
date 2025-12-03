# 🧹🤖 Robô Aspirador Inteligente (Simulação + Mapeamento + Node-RED)

Projeto que simula um robô aspirador em PyBullet com mapeamento, detecção de colisões, aprendizado entre execuções, geração de mapas PNG e telemetria via MQTT integrada a um dashboard Node-RED.

## Funcionalidades principais
- Mapeamento do ambiente durante a simulação
- Detecção e envio de colisões reais por MQTT
- Aprendizado entre execuções para evitar áreas já limpas
- Geração de mapas PNG com trajetória
- Telemetria em tempo real para dashboards Node-RED

## Inteligência do robô
- Primeira execução: exploração completa do ambiente  
- Execuções seguintes: evita regiões repetidas, reduz sobreposição e melhora eficiência

## Telemetria (exemplos)
Evento de colisão:
```json
{
    "event": "colisao_detectada",
    "pos": [-3.19, -0.00061, 0.098]
}
```

Pacotes periódicos:
```json
{
    "steps": 960,
    "pos": {"x": -3.181, "y": -0.00061},
    "area_pct": 2.88
}
```

## Supervisório Node-RED
Inclui dashboards para:
- Trajetória percorrida
- Área coberta
- Eficiência (m² por energia)
- Comparação entre execuções
- Alertas de colisão

## Estrutura do projeto
```
.
├── sim/
│   ├── robo.py
│   ├── sensores.py
│   ├── controle.py
│   ├── mapa.pkl
│   └── trajetoria.pkl
│
├── analise/
│   ├── gerar_mapa.py
│   ├── mapa_execucao_1.png
│   ├── mapa_execucao_2.png
│
├── supervisao/
│   ├── node-red-flow.json
│   └── mqtt-config.txt
│
├── README.md
└── requirements.txt
```

## Instalação
1. Instalar dependências Python:
```bash
pip install -r requirements.txt
```

2. Instalar Node-RED e Dashboard:
```bash
sudo npm install -g --unsafe-perm node-red
cd ~/.node-red
npm install @flowfuse/node-red-dashboard
```

3. Instalar e iniciar Mosquitto MQTT:
```bash
sudo apt install mosquitto mosquitto-clients
sudo systemctl enable mosquitto
sudo systemctl start mosquitto
```

Testar o tópico MQTT:
```bash
mosquitto_sub -t aspirador/log
```

## Execução
- Rodar simulação:
```bash
python sim/robo.py
```

- Gerar imagens do mapa e trajetória:
```bash
python analise/gerar_mapa.py
```

- Iniciar Node-RED:
```bash
node-red
```
Acessar:
- http://localhost:1880
- http://localhost:1880/ui

## Mapas gerados
Cada execução produz arquivos como:
- mapa_execucao_1.png
- mapa_execucao_2.png
- mapa_execucao_3.png

Eles mostram: mapa em escala de cinza, trajetória em vermelho e uma seta indicando a direção inicial.

## Como funciona o aprendizado
O robô começa sem conhecimento. Durante cada execução salva:
- mapa.pkl
- trajetoria.pkl

Em execuções futuras utiliza esses dados para:
- Evitar áreas já limpas
- Aproximar-se de rotas ótimas
- Reduzir colisões


