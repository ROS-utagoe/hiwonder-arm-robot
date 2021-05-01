import os
import sys
import time
sys.path.append(os.path.dirname(__file__) + "../armpi")
import kinematics as kin
import LeArm


class Gripper:
    def __init__(self, id, open_pos=700):
        self.id = id
        self.open_pos = open_pos

    def open(self):
        LeArm.setServo(self.id, self.open_pos, 500)

    def close(self, pos=1500):
        LeArm.setServo(self.id, pos, 500)



if __name__ == '__main__':
    gripper = Gripper(1)
    gripper.open()
    kin.ki_move(1500, 1000, 300, 2000)
    time.sleep(1)
    gripper.close()
    time.sleep(1)
    kin.ki_move(0, 2000, 200, 2000)
