import math
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

def main():
    client = RemoteAPIClient()
    sim = client.require('sim')
    simIK = client.require('simIK')

    # Fetch Scene Handles
    target_handle = sim.getObject('/GrabPoint_Barrel')
    tip_handle = sim.getObject('/GrabPoint_UR5')
    base_handle = sim.getObject('/OmniPlatform/body/forceSensor/UR5')

    joint_paths = [
        '/OmniPlatform/body/forceSensor/UR5/joint',
        '/OmniPlatform/body/forceSensor/UR5/joint/link/joint',
        '/OmniPlatform/body/forceSensor/UR5/joint/link/joint/link/joint',
        '/OmniPlatform/body/forceSensor/UR5/joint/link/joint/link/joint/link/joint',
        '/OmniPlatform/body/forceSensor/UR5/joint/link/joint/link/joint/link/joint/link/joint',
        '/OmniPlatform/body/forceSensor/UR5/joint/link/joint/link/joint/link/joint/link/joint/link/joint'
    ]
    scene_joints = [sim.getObject(p) for p in joint_paths]

    # Setup IK Environment & Group
    ik_env = simIK.createEnvironment()
    ik_group = simIK.createGroup(ik_env)

    simIK.setGroupCalculation(ik_env, ik_group, simIK.method_damped_least_squares, 0, 100)

    # addElementFromScene populates ik_env with internal objects base->tip
    ik_element = simIK.addElementFromScene(
        ik_env, ik_group, base_handle, tip_handle, target_handle, simIK.constraint_position
    )

    # Retrieve internal IK joint handles auto-generated for this IK group
    ik_joints = simIK.getGroupJoints(ik_env, ik_group)

    # Solve IK inside ik_env
    result, _, _ = simIK.handleGroup(ik_env, ik_group, {'syncFromSim': True})

    if result == simIK.result_success:
        print("IK Solution Found Successfully!\n")
        
        # Read position for each internal IK joint and apply to scene joints
        for i, (scene_j, ik_j) in enumerate(zip(scene_joints, ik_joints), 1):
            angle_rad = simIK.getJointPosition(ik_env, ik_j)
            
            # Drive main scene joints to calculated angles
            sim.setJointPosition(scene_j, angle_rad)
            sim.setJointTargetPosition(scene_j, angle_rad)
            
            print(f"  Joint {i}: {math.degrees(angle_rad):.2f}°")
    else:
        print("IK Solver failed. Target position may be out of reach.")

    # Clean up environment
    simIK.eraseEnvironment(ik_env)

if __name__ == '__main__':
    main()