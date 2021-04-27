import os
import sys
import time
sys.path.append(os.path.dirname(__file__) + "../armpi")
import kinematics as kin
import LeArm


if __name__ == '__main__':
    LeArm.runActionGroup('rest', 1)
    kin.ki_move(0, 2000, 500, 2000)
    time.sleep(2)
    kin.ki_move(0, 1500, 200, 2000)
    time.sleep(2)
    kin.ki_move(0, 2000, 2000, 2000)
