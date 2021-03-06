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
    """Espera o tempo de pausa da nota a partir do par??metro recebido. Tem como fun????o parar o movimento do bra??o.

    :param float time_delta: tempo de pausa a ser esperado
    """
    initial_time = perf_counter()
    while True:
        if perf_counter() - initial_time >= time_delta:
            break


def thread_pause(notes, notes_counter, notes_durations):
    """Executada por uma thread, chama a fun????o respons??vel pela pausa para N pausas seguidas, retornando apenas quando
    todas as pausas tiverem terminado.

    :param list notes: lista contendo todas as notas da m??sica, para que N sucessivas pausas sejam respeitadas
    :param int notes_counter: contador indicando qual ?? a nota atual da lista `notes`
    :param list notes_durations: lista contendo as dura????es, em segundos, de todas as notas da m??sica
    """
    try:
        while notes[notes_counter] % 10 == 0:
            notes.pop(notes_counter)
            wait_pause(notes_durations.pop(notes_counter))
    except IndexError:
        return


def get_time_signature_duration() -> float:
    """Retorna a dura????o em segundos de cada batida, de acordo com o BPM definido para a m??sica

    :return: float representando a dura????o em segundos de cada unidade de tempo de um compasso
    """
    return 60 / BPM


def check_pause(note) -> int:
    """Checa se a nota recebida pertence ao padr??o de uma pausa ou de uma nota a ser tocada. Retorna seu valor absoluto

    :param int note: nota para ser analizada
    :return: valor absoluto da nota, independente se for uma representa????o de pausa ou de nota a ser tocada
    """
    return int(note / 10) if note % 10 == 0 else note


def calculate_note_duration(note) -> float:
    """Calcula a dura????o, em segundos, da nota recebida. Utiliza o BPM e a f??rmula de compasso definidos.

    :param int note: c??digo da nota a ser calculado o tempo de dura????o
    :return: tempo de dura????o real da nota, calculado em segundos
    """
    return get_time_signature_duration() * (TIME_SIGNATURE[1] / check_pause(note))


def get_notes_duration(*notes) -> List[float]:
    """De acordo com o BPM escolhido, calcula (em segundos) quanto tempo ir?? durar cada nota, de acordo com seu tipo.

    :param float time_signature_duration: dura????o em segundos de cada unidade de tempo de um compasso
    :param int notes: lista contendo o valor representativo de cada nota a ser tocada (em ??ltima inst??ncia, da m??sica)
    :return: lista contendo todos os tempos, em segundos, que ir??o durar cada a nota a ser tocada
    """
    return [calculate_note_duration(n) for n in notes]


def calculate_note_vs_distance_factor(note) -> float:
    """Calcula o fator de corre????o normalizado entre a nota recebida e a dist??ncia angular de movimetna????o do bra??o.

    Para que a m??xima dist??ncia angular do bra??o se mantenha limitada pelo valor definido em `UP_POSITION`, o m??ximo
    valor retornado ?? 1. Dessa forma, notas que duram mais que a unidade de tempo n??o ir??o considerar uma excurs??o
    do bra??o maior que o pr??prio `UP_POSITION`, pois ir??o compensar a dura????o da nota diminuindo a velocidade do bra??o.
    Para notas que possuem dura????o menor do que a unidade de tempo, ser?? feita uma porcentagem para que o bra??o se
    movimente proporcionalmente ?? dura????o da nota. Ou seja, se a nota recebida possuir metade da dura????o da unidade de
    tempo, a excurs??o do bra??o ser?? metade da total e assim por diante.

    Portanto, considerando um cen??rio de f??rmula de compasso 4, 4, temos os seguintes exemplos:
        - Nota recebida: 4  ->  valor retornado: 1
        - Nota recebida: 2  ->  valor retornado: 1
        - Nota recebida: 8  ->  valor retornado: 0.5

    :param int note: nota a ser tocada
    :return: fator de propo????o normalizado do movimento do bra??o
    """
    return min((TIME_SIGNATURE[1] / note), 1.0)


def calculate_angular_distance(initial_position, final_position, note_vs_distance_factor, half_note_duration) -> float:
    """Calcula a dist??ncia angular necess??ria para que o movimento do bra??o respeite o tempo da nota e a velocidade max.

    Realia dois c??lculos distintos para a dist??ncia angular:
        - Excurs??o m??xima do bra??o multiplicada pela fator de corre????o nota X dist??ncia angular;
        - Setando a velocidade do motor a 3.14 rad/s;

    Dessa forma, ?? utilizado o menor valor encontrado dentre as duas op????es, pois caso o primeiro m??todo resulte no
    maior valor, significa que a m??xima velocidade do motor seria desrespeitada. Entretanto, se o segundo m??todo possuir
    o maior valor, significa que n??o ?? necess??rio utilizar a m??xima velocidade do motor, pois ?? poss??vel atingir o mesmo
    tempo apenas reduzindo a velocidade e aumentando a dist??ncia.

    :param float initial_position: posi????o inicial do bra??o
    :param float final_position: posi????o final do bra??o
    :param float note_vs_distance_factor: fator de corre????o nota X dist??ncia angular
    :param float half_note_duration: metade do tempo total de dura????o da nota
    :return: dist??ncia angular a ser percorrida pelo bra??o, respeitando o tempo de dura????o da nota e a velocidade max
    """
    return min((abs(initial_position - final_position)) * note_vs_distance_factor, MAX_VELOCITY * half_note_duration)


def calculate_angular_velocity(angular_distance, half_note_duration) -> float:
    """Calcula a velocidade angular ?? partir do tempo de dura????o da nota e da dist??ncia angular informada.

    :param float angular_distance: dist??ncia angular a ser percorrida pelo bra??o
    :param float half_note_duration: metade do tempo total de dura????o da nota
    :return: velocidade angular a ser setada no motor para que a nota seja tocada corretamente
    """
    return angular_distance / half_note_duration


def get_angular_velocity(total_note_duration, note, sensor_position) -> tuple:
    """Atrav??s de fun????es auxiliares, calcula a velocidade e posi????o angulares do motor para a nota recebida

    :param float total_note_duration: tempo total de dura????o da nota
    :param int note: nota a ser tocada
    :param float sensor_position: valor atual do sensor de posi????o do motor, em rad
    :return: velocidade e posi????o angulares do motor para a nota recebida
    """
    note_vs_distance_factor = calculate_note_vs_distance_factor(note)
    angular_distance = calculate_angular_distance(sensor_position, UP_POSITION, note_vs_distance_factor,
                                                  total_note_duration / 2)
    speed = calculate_angular_velocity(angular_distance, total_note_duration / 2)
    up_position = -1 * angular_distance
    return speed, up_position


def init_robot():
    """Inicializa o rob??, o timestep para o mundo atual e o client de comunica????o via OSC Message

    :return: objetos do rob??, timestep do rob?? e client de comunica????o via OSC Message
    """
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    client = SimpleUDPClient(IP, PORT)
    return robot, timestep, client


def init_motors(robot):
    """Inicializa os motores do rob?? recebido.

    :param robot: objeto do rob??
    :return: tupla de motores da m??o e das juntas do bra??o
    """
    hand_motors = [
        robot.getDevice("finger_1_joint_1"),
        robot.getDevice("finger_2_joint_1"),
        robot.getDevice("finger_middle_joint_1")
    ]

    # Motores das juntas do bra??o, iniciando a contagem de baixo (mais longe da m??o) para cima (mais prox da m??o)
    ur_motors = [
        robot.getDevice("shoulder_lift_joint"),  # Primeira junta - Rota????o Vertical
        robot.getDevice("elbow_joint"),  # Segunda junta - Rota????o Vertical
        robot.getDevice("wrist_1_joint"),  # Terceira junta - Rota????o Vertical
        robot.getDevice("wrist_2_joint")  # Quarta junta (m??o) - Rota????o horizontal
    ]
    return hand_motors, ur_motors


def init_sensors(robot, timestep):
    """Inicializa os sensores do rob??, sendo o de posi????o especificamente no motor do 'ombro'

    :param robot: objeto do rob??
    :param timestep: timestep para o mundo atual
    :return: tupla de sensores do rob??
    """
    distance_sensor = robot.getDevice("distance sensor")
    DistanceSensor.enable(distance_sensor, timestep)

    touch_sensor = robot.getDevice("force")
    TouchSensor.enable(touch_sensor, timestep)

    position_sensor = robot.getDevice("shoulder_lift_joint_sensor")
    PositionSensor.enable(position_sensor, timestep)

    return distance_sensor, touch_sensor, position_sensor


def play_notes(robot, timestep, client, hand_motors, ur_motors, distance_sensor, touch_sensor, position_sensor):
    """Ap??s todas as inicializa????es, entra no loop principal de execu????o das notas.

    Atrav??s dos 5 estados definidos em `status`, utiliza um loop para alternar entre eles e tocar todas as notas.
    As condi????es para trocar de estado sempre dizem respeito ?? posi????o angular do motor, cujo limite superior varia
    de nota para nota. Al??m disso, devido ?? imprecis??o imposta pelo processo mec??nico simulado, as verifica????es de
    posi????o n??o s??o exatas, sempre havendo uma margem de +-0.1 na posi????o alvo VS posi????o atual do motor.

    O comando principal para realiza????o de todas as notas sempre ?? dado no estado `up command`, onde s??o sempre
    calculadas e setadas novas posi????es alvo e velocidades para o motor. J?? o estado `down command` verifica se a nota
    seguinte ser?? uma pausa. Caso seja, mudar?? o estado para `waiting pause`, onde permanecer?? at?? que chegue uma nota
    a ser tocada. Os demais estados funcionam como transi????o e verifica????o de posi????o ou de toque, n??o possuindo,
    portanto, nenhuma a????o direta no bra??o do rob??.

    :param robot: objeto do rob??
    :param timestep: timestep para o mundo atual
    :param client: objeto do client de comunica????o via OSC Message
    :param hand_motors: lista contendo todos os motores das m??os
    :param ur_motors: lista contendo todos os motores das juntas
    :param distance_sensor: sensor de dist??ncia
    :param touch_sensor: sensor de toque
    :param position_sensor: sensor de posi????o referente ao motor do 'ombro'
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
    """Inicializa os objetos do rob??, timestep, client e sensores e chama a fun????o para tocar as notas
    """
    robot, timestep, client = init_robot()
    hand_motors, ur_motors = init_motors(robot)
    distance_sensor, touch_sensor, position_sensor = init_sensors(robot, timestep)
    play_notes(robot, timestep, client, hand_motors, ur_motors, distance_sensor, touch_sensor, position_sensor)


if __name__ == '__main__':
    start()
