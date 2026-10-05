"""
Description: OmniPlatform movement test
    Uses four regularRotation joints from CoppeliaSim's OmniPlatform model
    Commands wheel velocity directly from Python
    
Inputs:
    CoppeliaSim scene with OmniPlatform
    
Outputs:
    OmniPlatform wheel motion

NOTES:
The OmniPlatform's a robot with 4 omni wheels. It can move in any direction without turning
Its body frame is defined as follows (needed to figure out where wheels are located):
    body +X = world +Z  # vertical
    body +Y = world +Y  # horizontal
    body +Z = world -X  # horizontal
"""



#### IMPORTS
import math
from coppeliasim_zmqremoteapi_client import RemoteAPIClient



#### INITS
client = RemoteAPIClient()      # connection to CoppeliaSim
sim    = client.require("sim")  # simulation API

v = 80 * 2.398795 * math.pi / 180   # from default script
# "2.398795 is a factor needed to obtain the right pad rotation velocity"

wheels = [
    sim.getObject("/OmniPlatform/link[0]/regularRotation"), # back right
    sim.getObject("/OmniPlatform/link[1]/regularRotation"), # front right
    sim.getObject("/OmniPlatform/link[2]/regularRotation"), # front left
    sim.getObject("/OmniPlatform/link[3]/regularRotation"), # back left
    ]



#### MOVE STUFF
sim.setStepping(True)   # let python control sim steps
sim.startSimulation()   # start sim

sim.setJointTargetVelocity(wheels[0], -v)   # default move from lua script
sim.setJointTargetVelocity(wheels[1], -v)   # default move from lua script
sim.setJointTargetVelocity(wheels[2],  v)   # default move from lua script
sim.setJointTargetVelocity(wheels[3],  v)   # default move from lua script

for _ in range(200): sim.step()                             # run 200 steps
for wheel in wheels: sim.setJointTargetVelocity(wheel, 0)   # park the car

sim.stopSimulation()    # stop sim