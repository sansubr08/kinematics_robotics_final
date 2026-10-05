"""
Description: UR5 movement test
    Uses six joints from CoppeliaSim's UR5 model
    Commands joint position directly from Python

Inputs:
    CoppeliaSim scene with UR5 mounted to OmniPlatform

Outputs:
    UR5 joint motion

NOTES:
    UR5's a 6 DOF articulated robot, with a revolute joint for each DOF
    CoppeliaSim's original UR5 script takes six joint objects and passes them to moveToConfig
"""



#### IMPORTS
import math
from coppeliasim_zmqremoteapi_client import RemoteAPIClient



#### INITS
client = RemoteAPIClient()      # connection to CoppeliaSim
sim    = client.require("sim")  # simulation API

ur5    = sim.getObject("/OmniPlatform/body/forceSensor/UR5")    # UR5 model
joints = sim.getObjectsInTree(ur5, sim.object_joint_type, 0)    # all UR5 joints



#### MOVE STUFF
sim.setStepping(True)   # let python control sim steps
sim.startSimulation()   # start sim

start  = sim.getJointPosition(joints[0])        # joint 1, starting angle
target = start + math.radians(30)               # joint 1, move by 30 deg
sim.setJointTargetPosition(joints[0], target)   # joint 1, command movement

for _ in range(200): sim.step() # run 200 steps

sim.stopSimulation()    # stop sim