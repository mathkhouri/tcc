"""robotic_arm controller."""

# Import some classes of the controller module and OSC communication
from controller import (
    Robot, Motor, DistanceSensor, PositionSensor, TouchSensor, Speaker
)
from pythonosc.udp_client import SimpleUDPClient

from multiprocessing import Process
from threading import Thread
from time import perf_counter
from typing import List
from time import sleep

IP = '127.0.0.1'
PORT = 9000
MAX_VELOCITY = 3.14  # Em rad/s
PI = 3.14

BPM = 115
TIME_SIGNATURE = 4, 4

SEMI_BREVE = 1
SEMI_BREVE_PONTUADA = SEMI_BREVE - (SEMI_BREVE - 0) / 2
PAUSA_SEMI_BREVE = 10

MINIMA = 2
MINIMA_PONTUADA = MINIMA - (MINIMA - SEMI_BREVE) / 2
PAUSA_MINIMA = 20

SEMINIMA = 4
SEMINIMA_PONTUADA = SEMINIMA - (SEMINIMA - MINIMA) / 2
PAUSA_SEMINIMA = 40

COLCHEIA = 8
COLCHEIA_PONTUADA = COLCHEIA - (COLCHEIA - SEMINIMA) / 2
PAUSA_COLCHEIA = 80

SEMICOLCHEIA = 16
SEMICOLCHEIA_PONTUADA = SEMICOLCHEIA - (SEMICOLCHEIA - COLCHEIA) / 2
PAUSA_SEMICOLCHEIA = 160

FUSA = 32
FUSA_PONTUADA = FUSA - (FUSA - SEMICOLCHEIA) / 2
PAUSA_FUSA = 320

SEMIFUSA = 64
SEMIFUSA_PONTUADA = SEMIFUSA - (SEMIFUSA - FUSA) / 2
PAUSA_SEMIFUSA = 640

COMPASS_1 = [COLCHEIA, SEMINIMA_PONTUADA] * 2
COMPASS_2 = [SEMICOLCHEIA] * 8 + [COLCHEIA, SEMINIMA_PONTUADA]
COMPASS_3 = [COLCHEIA_PONTUADA, SEMICOLCHEIA, MINIMA, COLCHEIA, COLCHEIA]
COMPASS_4 = [SEMINIMA, COLCHEIA, COLCHEIA, COLCHEIA, COLCHEIA, SEMINIMA_PONTUADA, SEMICOLCHEIA]
COMPASS_5 = [PAUSA_SEMI_BREVE]
COMPASS_6 = [COLCHEIA_PONTUADA, SEMI_BREVE]
COMPASS_7 = [COLCHEIA_PONTUADA, SEMI_BREVE]
COMPASS_8 = [PAUSA_SEMINIMA] + [SEMICOLCHEIA]
COMPASS_9 = [SEMICOLCHEIA] * 12 + [SEMI_BREVE]

FULL_SONG = COMPASS_1 + COMPASS_2 + COMPASS_3 + COMPASS_4 + COMPASS_5 + COMPASS_6 + COMPASS_7 + COMPASS_8 + COMPASS_9

STATUS = {
    0: "down command",
    1: "going down",
    2: "up command",
    3: "going up",
    4: 'waiting pause',
}

UP_POSITION = -1.5  # Maxima posicao do braco, em rad
DOWN_POSITION = 0.0  # Minima posicao do braco, em rad


def wait_pause(time_delta):
    """Espera o tempo de pausa da nota a partir do parâmetro recebido. Tem como função parar o movimento do braço.

    :param float time_delta: tempo de pausa a ser esperado
    """
    initial_time = perf_counter()
    while True:
        if perf_counter() - initial_time >= time_delta:
            break


def thread_pause(notes, notes_counter, notes_durations):
    """Executada por uma thread, chama a função responsável pela pausa para N pausas seguidas, retornando apenas quando
    todas as pausas tiverem terminado.

    :param list notes: lista contendo todas as notas da música, para que N sucessivas pausas sejam respeitadas
    :param int notes_counter: contador indicando qual é a nota atual da lista `notes`
    :param list notes_durations: lista contendo as durações, em segundos, de todas as notas da música
    """
    try:
        while notes[notes_counter] % 10 == 0:
            notes.pop(notes_counter)
            wait_pause(notes_durations.pop(notes_counter))
    except IndexError:
        return


def get_time_signature_duration() -> float:
    """Retorna a duração em segundos de cada batida, de acordo com o BPM definido para a música

    :return: float representando a duração em segundos de cada unidade de tempo de um compasso
    """
    return 60 / BPM


def check_pause(note) -> int:
    """Checa se a nota recebida pertence ao padrão de uma pausa ou de uma nota a ser tocada. Retorna seu valor absoluto

    :param int note: nota para ser analizada
    :return: valor absoluto da nota, independente se for uma representação de pausa ou de nota a ser tocada
    """
    return int(note / 10) if note % 10 == 0 else note


def calculate_note_duration(note) -> float:
    """Calcula a duração, em segundos, da nota recebida. Utiliza o BPM e a fórmula de compasso definidos.

    :param int note: código da nota a ser calculado o tempo de duração
    :return: tempo de duração real da nota, calculado em segundos
    """
    return get_time_signature_duration() * (TIME_SIGNATURE[1] / check_pause(note))


def get_notes_duration(*notes) -> List[float]:
    """De acordo com o BPM escolhido, calcula (em segundos) quanto tempo irá durar cada nota, de acordo com seu tipo.

    :param float time_signature_duration: duração em segundos de cada unidade de tempo de um compasso
    :param int notes: lista contendo o valor representativo de cada nota a ser tocada (em última instância, da música)
    :return: lista contendo todos os tempos, em segundos, que irão durar cada a nota a ser tocada
    """
    return [calculate_note_duration(n) for n in notes]


def calculate_note_vs_distance_factor(note) -> float:
    """Calcula o fator de correção normalizado entre a nota recebida e a distância angular de movimetnação do braço.

    Para que a máxima distância angular do braço se mantenha limitada pelo valor definido em `UP_POSITION`, o máximo
    valor retornado é 1. Dessa forma, notas que duram mais que a unidade de tempo não irão considerar uma excursão
    do braço maior que o próprio `UP_POSITION`, pois irão compensar a duração da nota diminuindo a velocidade do braço.
    Para notas que possuem duração menor do que a unidade de tempo, será feita uma porcentagem para que o braço se
    movimente proporcionalmente à duração da nota. Ou seja, se a nota recebida possuir metade da duração da unidade de
    tempo, a excursão do braço será metade da total e assim por diante.

    Portanto, considerando um cenário de fórmula de compasso 4, 4, temos os seguintes exemplos:
        - Nota recebida: 4  ->  valor retornado: 1
        - Nota recebida: 2  ->  valor retornado: 1
        - Nota recebida: 8  ->  valor retornado: 0.5

    :param int note: nota a ser tocada
    :return: fator de propoção normalizado do movimento do braço
    """
    return min((TIME_SIGNATURE[1] / note), 1.0)


def calculate_angular_distance(initial_position, final_position, note_vs_distance_factor, half_note_duration) -> float:
    """Calcula a distância angular necessária para que o movimento do braço respeite o tempo da nota e a velocidade max.

    Realia dois cálculos distintos para a distância angular:
        - Excursão máxima do braço multiplicada pela fator de correção nota X distância angular;
        - Setando a velocidade do motor a 3.14 rad/s;

    Dessa forma, é utilizado o menor valor encontrado dentre as duas opções, pois caso o primeiro método resulte no
    maior valor, significa que a máxima velocidade do motor seria desrespeitada. Entretanto, se o segundo método possuir
    o maior valor, significa que não é necessário utilizar a máxima velocidade do motor, pois é possível atingir o mesmo
    tempo apenas reduzindo a velocidade e aumentando a distância.

    :param float initial_position: posição inicial do braço
    :param float final_position: posição final do braço
    :param float note_vs_distance_factor: fator de correção nota X distância angular
    :param float half_note_duration: metade do tempo total de duração da nota
    :return: distância angular a ser percorrida pelo braço, respeitando o tempo de duração da nota e a velocidade max
    """
    return min((abs(initial_position - final_position)) * note_vs_distance_factor, MAX_VELOCITY * half_note_duration)


def calculate_angular_velocity(angular_distance, half_note_duration) -> float:
    """Calcula a velocidade angular à partir do tempo de duração da nota e da distância angular informada.

    :param float angular_distance: distância angular a ser percorrida pelo braço
    :param float half_note_duration: metade do tempo total de duração da nota
    :return: velocidade angular a ser setada no motor para que a nota seja tocada corretamente
    """
    return angular_distance / half_note_duration


def get_angular_velocity(total_note_duration, note, sensor_position) -> tuple:
    """Através de funções auxiliares, calcula a velocidade e posição angulares do motor para a nota recebida

    :param float total_note_duration: tempo total de duração da nota
    :param int note: nota a ser tocada
    :param float sensor_position: valor atual do sensor de posição do motor, em rad
    :return: velocidade e posição angulares do motor para a nota recebida
    """
    note_vs_distance_factor = calculate_note_vs_distance_factor(note)
    angular_distance = calculate_angular_distance(sensor_position, UP_POSITION, note_vs_distance_factor,
                                                  total_note_duration / 2)
    speed = calculate_angular_velocity(angular_distance, total_note_duration / 2)
    up_position = -1 * angular_distance
    return speed, up_position


def init_robot():
    """Inicializa o robô, o timestep para o mundo atual e o client de comunicação via OSC Message

    :return: objetos do robô, timestep do robô e client de comunicação via OSC Message
    """
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    client = SimpleUDPClient(IP, PORT)
    return robot, timestep, client


def init_motors(robot):
    """Inicializa os motores do robô recebido.

    :param robot: objeto do robô
    :return: tupla de motores da mão e das juntas do braço
    """
    hand_motors = [
        robot.getDevice("finger_1_joint_1"),
        robot.getDevice("finger_2_joint_1"),
        robot.getDevice("finger_middle_joint_1")
    ]

    # Motores das juntas do braço, iniciando a contagem de baixo (mais longe da mão) para cima (mais prox da mão)
    ur_motors = [
        robot.getDevice("shoulder_lift_joint"),  # Primeira junta - Rotação Vertical
        robot.getDevice("elbow_joint"),  # Segunda junta - Rotação Vertical
        robot.getDevice("wrist_1_joint"),  # Terceira junta - Rotação Vertical
        robot.getDevice("wrist_2_joint")  # Quarta junta (mão) - Rotação horizontal
    ]
    return hand_motors, ur_motors


def init_sensors(robot, timestep):
    """Inicializa os sensores do robô, sendo o de posição especificamente no motor do 'ombro'

    :param robot: objeto do robô
    :param timestep: timestep para o mundo atual
    :return: tupla de sensores do robô
    """
    distance_sensor = robot.getDevice("distance sensor")
    DistanceSensor.enable(distance_sensor, timestep)

    touch_sensor = robot.getDevice("force")
    TouchSensor.enable(touch_sensor, timestep)

    position_sensor = robot.getDevice("shoulder_lift_joint_sensor")
    PositionSensor.enable(position_sensor, timestep)

    return distance_sensor, touch_sensor, position_sensor


def play_notes(robot, timestep, client, hand_motors, ur_motors, distance_sensor, touch_sensor, position_sensor):
    """Após todas as inicializações, entra no loop principal de execução das notas.

    Através dos 5 estados definidos em `status`, utiliza um loop para alternar entre eles e tocar todas as notas.
    As condições para trocar de estado sempre dizem respeito à posição angular do motor, cujo limite superior varia
    de nota para nota. Além disso, devido à imprecisão imposta pelo processo mecânico simulado, as verificações de
    posição não são exatas, sempre havendo uma margem de +-0.1 na posição alvo VS posição atual do motor.

    O comando principal para realização de todas as notas sempre é dado no estado `up command`, onde são sempre
    calculadas e setadas novas posições alvo e velocidades para o motor. Já o estado `down command` verifica se a nota
    seguinte será uma pausa. Caso seja, mudará o estado para `waiting pause`, onde permanecerá até que chegue uma nota
    a ser tocada. Os demais estados funcionam como transição e verificação de posição ou de toque, não possuindo,
    portanto, nenhuma ação direta no braço do robô.

    :param robot: objeto do robô
    :param timestep: timestep para o mundo atual
    :param client: objeto do client de comunicação via OSC Message
    :param hand_motors: lista contendo todos os motores das mãos
    :param ur_motors: lista contendo todos os motores das juntas
    :param distance_sensor: sensor de distância
    :param touch_sensor: sensor de toque
    :param position_sensor: sensor de posição referente ao motor do 'ombro'
    """
    current_status = 0
    notes_counter = 0
    notes = FULL_SONG.copy()
    status_dict = STATUS.copy()
    notes_durations = get_notes_duration(*notes)
    speed, up_position = get_angular_velocity(get_time_signature_duration(), TIME_SIGNATURE[1], DOWN_POSITION)
    Motor.setVelocity(ur_motors[0], speed)
    Motor.setPosition(ur_motors[0], up_position)
    while robot.step(timestep) != -1:
        if status_dict.get(current_status) == "down command":
            if PositionSensor.getValue(position_sensor) < up_position + 0.1:
                print(status_dict.get(current_status))
                if notes_counter >= len(notes_durations):
                    break
                elif notes[notes_counter] % 10 == 0:
                    current_status = 4
                else:
                    Motor.setPosition(ur_motors[0], DOWN_POSITION)
                    current_status = 1

        elif status_dict.get(current_status) == "going down":
            if PositionSensor.getValue(position_sensor) > DOWN_POSITION - 0.1:
                print(status_dict.get(current_status))
                touch_value = TouchSensor.getValue(touch_sensor)
                print("Detecting a collision of:", round(touch_value, 2), "N")
                client.send_message("/osc_signal_receiver/right_arm", touch_value)
                current_status = 2

        elif status_dict.get(current_status) == "up command":
            print(status_dict.get(current_status))
            speed, up_position = get_angular_velocity(notes_durations[notes_counter], notes[notes_counter],
                                                      PositionSensor.getValue(position_sensor))
            Motor.setVelocity(ur_motors[0], speed)
            Motor.setPosition(ur_motors[0], up_position)
            current_status = 3

        elif status_dict.get(current_status) == "going up":
            if PositionSensor.getValue(position_sensor) < up_position + 0.1:
                print(status_dict.get(current_status))
                current_status = 0
                notes_counter += 1

        elif status_dict.get(current_status) == 'waiting pause':
            thread = Thread(target=thread_pause, args=[notes, notes_counter, notes_durations])
            thread.start()
            thread.join()
            Motor.setVelocity(ur_motors[0], speed)
            current_status = 0


def start():
    """Inicializa os objetos do robô, timestep, client e sensores e chama a função para tocar as notas
    """
    robot, timestep, client = init_robot()
    hand_motors, ur_motors = init_motors(robot)
    distance_sensor, touch_sensor, position_sensor = init_sensors(robot, timestep)
    play_notes(robot, timestep, client, hand_motors, ur_motors, distance_sensor, touch_sensor, position_sensor)


if __name__ == '__main__':
    start()
