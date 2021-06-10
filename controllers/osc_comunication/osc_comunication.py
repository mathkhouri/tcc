"""osc_comunication controller."""

# Import some classes of the controller module.
from controller import Robot, TouchSensor, Motor
from pythonosc.udp_client import SimpleUDPClient

# create the Robot instance and set max velocity.
robot = Robot()
MAX_SPEED = 6.28
movement_counter = 0

# Osc Message to send to SuperCollider
ip = "127.0.0.1"
port = 57120
client = SimpleUDPClient(ip, port)

# set the time step of the world.
TIME_STEP = 64

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
force = robot.getDevice("force")
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')

force.enable(TIME_STEP)
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

# set up the motor speeds.
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(TIME_STEP) != -1:
    # when the touch sensor detects something, start the avoidance movement:
    force_value = TouchSensor.getValue(force)

    if(force_value > 0.01):
        print("Detecting a collision of:", round(force_value, 2), "N")
        client.send_message("/controllers/osc_comunication", force_value)
        movement_counter = 15
    
    # We use the movement_counter to manage the movements of the robot. When
    # the value is 0 we move straight, then when there is another value this
    # means that we are avoiding an obstacle. For avoiding we first move
    # backward for some cycles and then we turn on ourself.
    if(movement_counter == 0):
        # initialize motor speeds at 50% of MAX_SPEED.
        leftSpeed  = 0.5 * MAX_SPEED
        rightSpeed = 0.5 * MAX_SPEED
    elif(movement_counter >= 7):
        # turn right
        leftSpeed = 0.5 * MAX_SPEED
        rightSpeed = -0.5 * MAX_SPEED
        movement_counter = 0
    else:
        # turn left
        leftSpeed = -0.5 * MAX_SPEED
        rightSpeed = 0.5 * MAX_SPEED
        movement_counter = 0
        
    # write actuators inputs
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
    pass