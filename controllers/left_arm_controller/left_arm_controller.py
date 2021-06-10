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
ARM_LENGTH = 0.5  # Em metros
MAX_VELOCITY = 3.14
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
COMPASS_2 = [PAUSA_SEMICOLCHEIA] + [SEMICOLCHEIA] * 7 + [COLCHEIA_PONTUADA, SEMINIMA_PONTUADA]
COMPASS_3 = [COLCHEIA_PONTUADA, PAUSA_FUSA, MINIMA, COLCHEIA, COLCHEIA, SEMINIMA]
COMPASS_4 = [COLCHEIA, COLCHEIA, COLCHEIA, COLCHEIA, COLCHEIA, COLCHEIA_PONTUADA, MINIMA]
COMPASS_5 = []
COMPASS_6 = [PAUSA_FUSA, SEMI_BREVE, PAUSA_SEMINIMA]
COMPASS_7 = [SEMI_BREVE]
COMPASS_8 = [SEMICOLCHEIA]
COMPASS_9 = [PAUSA_SEMICOLCHEIA] + [SEMICOLCHEIA] * 8 + [SEMI_BREVE]

FULL_SONG = COMPASS_1 + COMPASS_2 + COMPASS_3 + COMPASS_4 + COMPASS_5 + COMPASS_6 + COMPASS_7 + COMPASS_8 + COMPASS_9

STATUS = {
    0: "down command",
    1: "going down",
    2: "up command",
    3: "going up",
    4: 'waiting pause',
}

UP_POSITION = -1.5
DOWN_POSITION = 0.0


def wait_pause(time_delta):
    initial_time = perf_counter()
    while True:
        if perf_counter() - initial_time >= time_delta:
            break


def thread_pause(notes, notes_counter, notes_durations):
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
    return int(note / 10) if note % 10 == 0 else note


def calculate_note_duration(note):
    return get_time_signature_duration() * (TIME_SIGNATURE[1] / check_pause(note))


def get_notes_duration(*notes) -> List[float]:
    """De acordo com o BPM escolhido, calcula (em segundos) quanto tempo irá durar cada nota, de acordo com seu tipo.

    :param float time_signature_duration: duração em segundos de cada unidade de tempo de um compasso
    :param int notes: lista contendo o valor representativo de cada nota a ser tocada (em última instância, da música)
    :return: lista contendo todos os tempos, em segundos, que irão durar cada a nota a ser tocada
    """
    return [calculate_note_duration(n) for n in notes]


def calculate_note_vs_distance_factor(note) -> float:
    return min((TIME_SIGNATURE[1] / note), 1.0)


def calculate_linear_distance(initial_position, final_position, note_vs_distance_factor, half_note_duration) -> float:
    return min((2 * PI * ARM_LENGTH * (abs(initial_position - final_position) / 6.28)) * note_vs_distance_factor,
               MAX_VELOCITY * ARM_LENGTH * half_note_duration)


def calculate_angular_velocity(linear_distance, half_note_duration) -> float:
    return (linear_distance / half_note_duration) / ARM_LENGTH


def get_angular_velocity(total_note_duration, note, sensor_position) -> tuple:
    note_vs_distance_factor = calculate_note_vs_distance_factor(note)
    linear_distance = calculate_linear_distance(sensor_position, UP_POSITION, note_vs_distance_factor,
                                                total_note_duration / 2)
    speed = calculate_angular_velocity(linear_distance, total_note_duration / 2)
    up_position = -1 * (linear_distance / ARM_LENGTH)
    return speed, up_position


def init_robot():
    robot = Robot()
    # set the time step of the current world.
    timestep = int(robot.getBasicTimeStep())
    # set the ip and port to send OSC Messages to SuperCollider
    client = SimpleUDPClient(IP, PORT)
    # speaker = Speaker("speaker")
    return robot, timestep, client


def init_motors(robot):
    # Insert a getDevice-like function in order to get the
    # instance of a device of the robot.
    hand_motors = [
        robot.getDevice("finger_1_joint_1"),
        robot.getDevice("finger_2_joint_1"),
        robot.getDevice("finger_middle_joint_1")
    ]

    # Rotação vertical (RV) das juntas, iniciando a contagem de baixo para cima
    ur_motors = [
        robot.getDevice("shoulder_lift_joint"),  # Primeira junta - RV
        robot.getDevice("elbow_joint"),  # Segunda junta - RV
        robot.getDevice("wrist_1_joint"),  # Terceira junta - RV
        robot.getDevice("wrist_2_joint")  # Quarta junta (mão) - Rotação horizontal
    ]
    return hand_motors, ur_motors


def init_sensors(robot, timestep):
    distance_sensor = robot.getDevice("distance sensor")
    DistanceSensor.enable(distance_sensor, timestep)

    touch_sensor = robot.getDevice("force")
    TouchSensor.enable(touch_sensor, timestep)

    position_sensor = robot.getDevice("shoulder_lift_joint_sensor")
    PositionSensor.enable(position_sensor, timestep)

    return distance_sensor, touch_sensor, position_sensor


def play_notes(robot, timestep, client, hand_motors, ur_motors, distance_sensor, touch_sensor, position_sensor):
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
                client.send_message("/osc_signal_receiver/left_arm", touch_value)
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
    robot, timestep, client = init_robot()
    hand_motors, ur_motors = init_motors(robot)
    distance_sensor, touch_sensor, position_sensor = init_sensors(robot, timestep)
    play_notes(robot, timestep, client, hand_motors, ur_motors, distance_sensor, touch_sensor, position_sensor)


if __name__ == '__main__':
    start()
