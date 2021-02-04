import pybullet as p
import pybullet_data
from pybullet_object_models import ycb_objects

import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)



import time
import math as m


# ------------------------ #
# --- Setup simulation --- #
# ------------------------ #

# Create pybullet GUI
physics_client_id = p.connect(p.GUI)
p.resetDebugVisualizerCamera(1.8, 120, -50, [0.0, -0.0, -0.0])
p.resetSimulation()
p.setPhysicsEngineParameter(numSolverIterations=150)
sim_timestep = 1.0/240
p.setTimeStep(sim_timestep)

# Load plane contained in pybullet_data
p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"))

# Set gravity for simulation. For easy movement without restriction
p.setGravity(0, 0, 0)



#robot = iCubHandsEnv(physics_client_id, use_IK=1, control_arm='r')
urdf_path = r"Poppy_Humanoid.urdf"
robotID = p.loadURDF(urdf_path, useFixedBase=True, basePosition=[0, 0, 0.5])


position, orientation = p.getBasePositionAndOrientation(robotID)

number_of_joints = p.getNumJoints(robotID)
for joint_number in range(number_of_joints):
    info = p.getJointInfo(robotID, joint_number)
    print(info[0], ": ", info[1])

# LEGS
hip_x = p.addUserDebugParameter('hip_x', -m.pi/2, m.pi/2, 0)
hip_z = p.addUserDebugParameter('hip_z', -m.pi/2, m.pi/2, 0)
hip_y = p.addUserDebugParameter('hip_y', -m.pi/2, m.pi/2, 0)
knee_y = p.addUserDebugParameter('knee_y', -m.pi/2, m.pi/2, 0)
ankle_y = p.addUserDebugParameter('ankle_y', -m.pi/2, m.pi/2, 0)

# ARMS
shoulder_y = p.addUserDebugParameter('shoulder_y', -m.pi/2, m.pi/2, 0)
shoulder_x = p.addUserDebugParameter('shoulder_x', -m.pi/2, m.pi/2, 0)
arm_z = p.addUserDebugParameter('arm_z', -m.pi/2, m.pi/2, 0)
elbow_y = p.addUserDebugParameter('elbow_y', -m.pi/2, m.pi/2, 0)

abs_y = p.addUserDebugParameter('abs_y', -m.pi/2, m.pi/2, 0)
abs_x = p.addUserDebugParameter('abs_x', -m.pi/2, m.pi/2, 0)
abs_z = p.addUserDebugParameter('abs_z', -m.pi/2, m.pi/2, 0)
bust_y = p.addUserDebugParameter('bust_y', -m.pi/2, m.pi/2, 0)
bust_x = p.addUserDebugParameter('bust_x', -m.pi/2, m.pi/2, 0)
head_z = p.addUserDebugParameter('head_z', -m.pi/2, m.pi/2, 0)
head_y = p.addUserDebugParameter('head_y', -m.pi/2, m.pi/2, 0)

# Groups of joints.
hip_x_index = [0, 5]
hip_z_index = [1, 6]
hip_y_index = [2, 7]
knee_y_index = [3, 8]
ankle_y_index = [4, 9]

shoulder_y_index = [17, 21]
shoulder_x_index = [18, 22]
arm_z_index = [19, 23]
elbow_y_index = [20, 24]


while True:
    user_hip_x = p.readUserDebugParameter(hip_x)
    user_hip_z = p.readUserDebugParameter(hip_z)
    user_hip_y = p.readUserDebugParameter(hip_y)
    user_knee_y = p.readUserDebugParameter(knee_y)
    user_ankle_y = p.readUserDebugParameter(ankle_y)

    user_shoulder_y = p.readUserDebugParameter(shoulder_y)
    user_shoulder_x = p.readUserDebugParameter(shoulder_x)
    user_arm_z = p.readUserDebugParameter(arm_z)
    user_elbow_y = p.readUserDebugParameter(elbow_y)

    user_abs_y = p.readUserDebugParameter(abs_y)
    user_abs_x = p.readUserDebugParameter(abs_x)
    user_abs_z = p.readUserDebugParameter(abs_z)
    user_bust_y = p.readUserDebugParameter(bust_y)
    user_bust_x = p.readUserDebugParameter(bust_x)
    user_head_z = p.readUserDebugParameter(head_z)
    user_head_y = p.readUserDebugParameter(head_y)

    for joint_index in hip_x_index:
        p.setJointMotorControl2(robotID, joint_index,
                                p.POSITION_CONTROL,
                                targetPosition=user_hip_x)

    for joint_index in hip_z_index:
        p.setJointMotorControl2(robotID, joint_index,
                                p.POSITION_CONTROL,
                                targetPosition=user_hip_z)

    for joint_index in hip_y_index:
        p.setJointMotorControl2(robotID, joint_index,
                                p.POSITION_CONTROL,
                                targetPosition=user_hip_y)

    for joint_index in knee_y_index:
        p.setJointMotorControl2(robotID, joint_index,
                                p.POSITION_CONTROL,
                                targetPosition=user_knee_y)

    for joint_index in ankle_y_index:
        p.setJointMotorControl2(robotID, joint_index,
                                p.POSITION_CONTROL,
                                targetPosition=user_ankle_y)

    for joint_index in shoulder_y_index:
        p.setJointMotorControl2(robotID, joint_index,
                                p.POSITION_CONTROL,
                                targetPosition=user_shoulder_y)

    for joint_index in shoulder_x_index:
        p.setJointMotorControl2(robotID, joint_index,
                                p.POSITION_CONTROL,
                                targetPosition=user_shoulder_x)

    for joint_index in arm_z_index:
        p.setJointMotorControl2(robotID, joint_index,
                                p.POSITION_CONTROL,
                                targetPosition=user_arm_z)

    for joint_index in elbow_y_index:
        p.setJointMotorControl2(robotID, joint_index,
                                p.POSITION_CONTROL,
                                targetPosition=user_elbow_y)
    # Index of abs_y is 10
    p.setJointMotorControl2(robotID, 10,
                            p.POSITION_CONTROL,
                            targetPosition=user_abs_y)

    # Index of abs_y is 11
    p.setJointMotorControl2(robotID, 11,
                            p.POSITION_CONTROL,
                            targetPosition=user_abs_x)

    # Index of abs_y is 12
    p.setJointMotorControl2(robotID, 12,
                            p.POSITION_CONTROL,
                            targetPosition=user_abs_z)

    # Index of bust_y is 13
    p.setJointMotorControl2(robotID, 13,
                            p.POSITION_CONTROL,
                            targetPosition=user_bust_y)

    # Index of bust_x is 14
    p.setJointMotorControl2(robotID, 14,
                            p.POSITION_CONTROL,
                            targetPosition=user_bust_x)

    # Index of head_z is 15
    p.setJointMotorControl2(robotID, 15,
                            p.POSITION_CONTROL,
                            targetPosition=user_bust_x)

    # Index of head_y is 16
    p.setJointMotorControl2(robotID, 16,
                            p.POSITION_CONTROL,
                            targetPosition=user_bust_x)


    p.stepSimulation()
