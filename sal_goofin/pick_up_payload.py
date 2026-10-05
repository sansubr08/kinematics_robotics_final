import math
import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

def solve_and_move(sim, simIK, ik_env, ik_group, joints):
    """Solves IK and updates joint positions in the scene."""
    result, _, _ = simIK.handleGroup(ik_env, ik_group, {'syncFromSim': True})

    if result == simIK.result_success:
        ik_joints = simIK.getGroupJoints(ik_env, ik_group)
        for scene_j, ik_j in zip(joints, ik_joints):
            angle_rad = simIK.getJointPosition(ik_env, ik_j)
            sim.setJointTargetPosition(scene_j, angle_rad)
            sim.setJointPosition(scene_j, angle_rad)
        return True
    else:
        print("IK Solve Failed: Target out of reach.")
        return False

def attach_payload(sim, payload_handle, end_effector_handle):
    """Parents the barrel payload to the end-effector dummy."""
    sim.setObjectParent(payload_handle, end_effector_handle, True)
    print("Payload Attached.")

def detach_payload(sim, payload_handle):
    """Detaches the barrel payload back to scene root."""
    sim.setObjectParent(payload_handle, -1, True)
    print("Payload Detached.")

def main():
    client = RemoteAPIClient()
    sim = client.require('sim')
    simIK = client.require('simIK')

    barrel_handle = sim.getObject('/GrabPoint_Barrel')
    tip_handle = sim.getObject('/GrabPoint_UR5')
    base_handle = sim.getObject('/OmniPlatform/body/forceSensor/UR5')
    drop_target_handle = sim.getObject('/PointB')

    joint_paths = [
        '/OmniPlatform/body/forceSensor/UR5/joint',
        '/OmniPlatform/body/forceSensor/UR5/joint/link/joint',
        '/OmniPlatform/body/forceSensor/UR5/joint/link/joint/link/joint',
        '/OmniPlatform/body/forceSensor/UR5/joint/link/joint/link/joint/link/joint',
        '/OmniPlatform/body/forceSensor/UR5/joint/link/joint/link/joint/link/joint/link/joint',
        '/OmniPlatform/body/forceSensor/UR5/joint/link/joint/link/joint/link/joint/link/joint/link/joint'
    ]
    joints = [sim.getObject(p) for p in joint_paths]

    # --- Move to Pick Location ---
    print("--- Step 1: Moving to Pick Location ---")
    ik_env = simIK.createEnvironment()
    ik_group = simIK.createGroup(ik_env)
    simIK.setGroupCalculation(ik_env, ik_group, simIK.method_damped_least_squares, 0, 100)
    simIK.addElementFromScene(ik_env, ik_group, base_handle, tip_handle, barrel_handle, simIK.constraint_position)

    if solve_and_move(sim, simIK, ik_env, ik_group, joints):
        time.sleep(1.0)

        print("--- Step 2: Grabbing Barrel ---")
        attach_payload(sim, barrel_handle, tip_handle)
        time.sleep(0.5)

        # Reset environment cleanly before retargeting
        simIK.eraseEnvironment(ik_env)

        # --- Move to Drop Location ---
        print("--- Step 3: Retargeting IK to Second Platform ---")
        ik_env = simIK.createEnvironment()
        ik_group = simIK.createGroup(ik_env)
        simIK.setGroupCalculation(ik_env, ik_group, simIK.method_damped_least_squares, 0, 100)
        simIK.addElementFromScene(ik_env, ik_group, base_handle, tip_handle, drop_target_handle, simIK.constraint_position)

        if solve_and_move(sim, simIK, ik_env, ik_group, joints):
            time.sleep(1.0)

            print("--- Step 4: Releasing Barrel ---")
            detach_payload(sim, barrel_handle)
            print("\nTransfer Complete!")

        simIK.eraseEnvironment(ik_env)
    else:
        simIK.eraseEnvironment(ik_env)

if __name__ == '__main__':
    main()