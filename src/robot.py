from geometry_msgs.msg import Twist
import time

class Robot:
    def __init__(self, width = 1, height = 1, nameSpace = 'ground'):
        self.width = width
        self.height = height
        self.radius = int(max(width, height) / 2)
        self.x = 0
        self.y = 0
        self.quaternion = [0,0,0,0]
        self.eularAngles = [0,0,0]
        self.speed = 1
        self.twist = Twist()
        self.currentState = ['calculate', 'turn', 'forward']
        self.currentStateCounter = 0
        self.clockWise = False
        self.goalTheta = 0
        self.originalTheta = 0
        self.forwardDistance = 0
        self.manageTurn = False
        self.manageStuck = False
        self.recoveryStamp = time.clock() - 100.0
        self.nameSpace = nameSpace



    def getPosition():
        return (self.x, self.y)

    def getOrientation():
        return (self.quaternion)