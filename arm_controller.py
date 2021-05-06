"""robotic_arm controller."""

# Import some classes of the controller module and OSC communication
from controller import (
    Robot, Motor, DistanceSensor, PositionSensor, TouchSensor, Speaker
)
from pythonosc.udp_client import SimpleUDPClient

from time import sleep
from typing import List

IP = '127.0.0.1'
PORT = 57120
SPEED = 1.0
ARM_LENGTH = 0.5 / 3  # Em metros
PI = 3.14

BPM = 120
TIME_SIGNATURE = 4, 4

SEMI_BREVE = 1
MINIMA = 2
SEMINIMA = 4
COLCHEIA = 8
SEMICOLCHEIA = 16
FUSA = 32
SEMIFUSA = 64

INITIAL_NOTES = [4, 4, 8, 8, 4]

STATUS = {
    0: "down command",
    1: "going down",
    2: "up command",
    3: "going up",
    4: 'waiting pause',
}

UP_POSITION = -1.5
DOWN_POSITION = -2.8


def set_velocity(ur_motors):
    for motor in ur_motors:
        Motor.setVelocity(motor, SPEED)


def define_time_signature_duration() -> float:
    """Retorna a duração em segundos de cada batida, de acordo com o BPM definido para a música

    :return: float representando a duração em segundos de cada unidade de tempo de um compasso
    """
    return 60 / BPM


def get_notes_duration(time_signature_duration, *notes) -> List[float]:
    """De acordo com o BPM escolhido, calcula (em segundos) quanto tempo irá durar cada nota, de acordo com seu tipo.

    :param float time_signature_duration: duração em segundos de cada unidade de tempo de um compasso
    :param int notes: lista contendo o valor representativo de cada nota a ser tocada (em última instância, da música)
    :return: lista contendo todos os tempos, em segundos, que irão durar cada a nota a ser tocada
    """
    return [time_signature_duration * (TIME_SIGNATURE[1] / n) for n in notes]


def calculate_velocity(total_duration):
    return (2 * PI * ARM_LENGTH / 4) / (total_duration / 2)


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
    time_signature_duration = define_time_signature_duration()
    notes = get_notes_duration(time_signature_duration, *INITIAL_NOTES)
    status = 0
    status_dict = STATUS.copy()
    notes_counter = 0
    # Move o braço para posição inicial - 90º
    Motor.setPosition(ur_motors[0], UP_POSITION)
    while robot.step(timestep) != -1:
        if status_dict.get(status) == "down command":
            if PositionSensor.getValue(position_sensor) > UP_POSITION + 0.1:
                print(status_dict.get(status))
                if notes_counter >= len(notes):
                    break
                total_duration = notes[notes_counter]
                speed = calculate_velocity(total_duration)
                # for ur_motor, position in zip(ur_motors, target_position):
                Motor.setVelocity(ur_motors[0], speed)
                Motor.setPosition(ur_motors[0], DOWN_POSITION)
                status = 1

        elif status_dict.get(status) == "going down":
            if PositionSensor.getValue(position_sensor) < DOWN_POSITION + 0.1:
                print(status_dict.get(status))
                touch_value = TouchSensor.getValue(touch_sensor)
                print("Detecting a collision of:", round(touch_value, 2), "N")
                client.send_message("/controllers/robotic_arm", touch_value)
                status = 2

        elif status_dict.get(status) == "up command":
            print(status_dict.get(status))
            Motor.setPosition(ur_motors[0], UP_POSITION + 0.2)
            status = 3

        elif status_dict.get(status) == "going up":
            if PositionSensor.getValue(position_sensor) > UP_POSITION + 0.1:
                print(status_dict.get(status))
                status = 0
                notes_counter += 1


def start():
    robot, timestep, client = init_robot()
    hand_motors, ur_motors = init_motors(robot)
    distance_sensor, touch_sensor, position_sensor = init_sensors(robot, timestep)
    play_notes(robot, timestep, client, hand_motors, ur_motors, distance_sensor, touch_sensor, position_sensor)


if __name__ == '__main__':
    start()
