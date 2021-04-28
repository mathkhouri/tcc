"""robotic_arm controller."""

# Import some classes of the controller module and OSC communication
# from controller import (
#     Robot, Motor, DistanceSensor, PositionSensor, TouchSensor, Speaker
# )
# from pythonosc.udp_client import SimpleUDPClient

from time import sleep

IP = '127.0.0.1'
PORT = 57120
SPEED = 1.0
ARM_LENGTH = 0.5/3  # Em metros
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


def set_velocity(ur_motors):
    for motor in ur_motors:
        Motor.setVelocity(motor, SPEED)


def set_initial_positions(ur_motors):
    Motor.setVelocity(ur_motors[0], SPEED)
    Motor.setPosition(ur_motors[0], -6.28/4)
    while PositionSensor.getValue(position_sensor) > -6.28 / 4:
        print(PositionSensor.getValue(position_sensor))


def switch_case(argument):
    switcher = {
        0: "WAITING",
        1: "ROTATING",
        2: "COLLISION",
        3: "ROTATING_BACK"
    }
    return switcher.get(argument, "nothing")


def define_time_signature_duration():
    return 60 / BPM


def get_notes_duration(time_signature_duration, *notes):
    return [time_signature_duration * (TIME_SIGNATURE[1] / n) for n in notes]


def init_robot():
    robot = Robot()
    # set the time step of the current world.
    # timestep = int(robot.getBasicTimeStep())
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

    position_sensor = robot.getDevice("wrist_1_joint_sensor")
    PositionSensor.enable(position_sensor, timestep)

    return distance_sensor, touch_sensor, position_sensor


def run(robot, timestep, client, hand_motors, ur_motors,
        distance_sensor, touch_sensor, position_sensor):
    argument = 0
    counter = 0
    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
        if counter <= 0:
            if switch_case(argument) == "WAITING":
                counter = 8
                for ur_motor, position in zip(ur_motors, TARGET_POSITIONS):
                    Motor.setPosition(ur_motor, position)
                print("Rotating arm \n")
                argument = 1

            elif switch_case(argument) == "ROTATING":
                if PositionSensor.getValue(position_sensor) < -1.7:
                    print("Collision \n")
                    argument = 2
                    touch_value = TouchSensor.getValue(touch_sensor)
                    print(
                        "Detecting a collision of:", round(touch_value, 2),
                        "N"
                    )
                    client.send_message(
                        "/controllers/robotic_arm", touch_value
                    )
            #                Speaker.playSound(
            #                    speaker, speaker,
            #                    "sounds/bateria.wav", 1.0, 1.0, 0, True
            #                )

            elif switch_case(argument) == "COLLISION":
                for ur_motor in ur_motors:
                    Motor.setPosition(ur_motor, 0.0)
                print("Rotating arm back \n")
                argument = 3

            elif switch_case(argument) == "ROTATING_BACK":
                if PositionSensor.getValue(position_sensor) > -0.7:
                    argument = 0
                    print("Waiting \n")

        counter -= 1


def play_notes(robot, timestep, client, hand_motors, ur_motors, distance_sensor, touch_sensor, position_sensor):
    set_initial_positions(ur_motors)
    time_signature_duration = define_time_signature_duration()
    notes = [(0, -6.28 / 4, note) for note in get_notes_duration(time_signature_duration, *INITIAL_NOTES)]
    for target_position, final_position, total_duration in notes:
        speed = (2 * PI * ARM_LENGTH / 4) / (total_duration / 2)
        Motor.setVelocity(ur_motors[0], speed)
        Motor.setPosition(ur_motors[0], target_position)
        while PositionSensor.getValue(position_sensor) < 0:
            print(PositionSensor.getValue(position_sensor))
        Motor.setPosition(ur_motors[0], final_position)
        while PositionSensor.getValue(position_sensor) > -6.28 / 4:
            print(PositionSensor.getValue(position_sensor))


def start():
    robot, timestep, client = init_robot()
    hand_motors, ur_motors = init_motors(robot)
    set_velocity(ur_motors)
    # time_signature_duration = define_time_signature_duration()
    # notes = [(0, -6.28 / 4, note) for note in get_notes_duration(time_signature_duration, *INITIAL_NOTES)]
    # for target_position, final_position, total_duration in notes:
    #     speed = (2 * PI * ARM_LENGTH / 4) / (total_duration / 2)
    distance_sensor, touch_sensor, position_sensor = init_sensors(robot, timestep)
    play_notes(robot, timestep, client, hand_motors, ur_motors, distance_sensor, touch_sensor, position_sensor)
    # run(
    #     robot, timestep, client, hand_motors, ur_motors,
    #     distance_sensor, touch_sensor, position_sensor,
    # )


if __name__ == '__main__':
    start()
