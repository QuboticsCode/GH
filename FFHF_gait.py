import pybullet as p
import pybullet_data
import time
import math

# Connect to physics server
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# Load plane and robot
planeId = p.loadURDF("plane.urdf")
p.changeDynamics(planeId, -1, lateralFriction=1.0)
startPos = [0, 0, 0.3]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
robot_id = p.loadURDF("GH.urdf", startPos, startOrientation)

# Joint dictionary
joint_ids = {
    "fr_hip": 0, "fr_knee": 1,
    "fl_hip": 2, "fl_knee": 3,
    "hr_hip": 4, "hr_knee": 5,
    "hl_hip": 6, "hl_knee": 7
}

# --- Smoother interpolation function ---
def interpolate_angles(start_angles, end_angles, duration, steps=30):
    for t in range(steps + 1):
        alpha = t / steps
        smoothed = {joint: (1 - alpha) * start_angles[joint] + alpha * end_angles[joint]
                    for joint in start_angles}
        apply_joint_angles(smoothed)
        time.sleep(duration / steps)

# --- Direct joint control (no interpolation) ---
def apply_joint_angles(angles):
    for joint, angle in angles.items():
        p.setJointMotorControl2(
            bodyIndex=robot_id,
            jointIndex=joint_ids[joint],
            controlMode=p.POSITION_CONTROL,
            targetPosition=angle,
            force=20.0
        )
    p.stepSimulation()

# --- Rest position ---
rest_angles = {
    "fr_hip":  -0.4, "fr_knee": 0.7,
    "fl_hip":  -0.4, "fl_knee": 0.7,
    "hr_hip": -0.4, "hr_knee":  0.7,
    "hl_hip": -0.4, "hl_knee":  0.7
}
print("Setting rest position...")
interpolate_angles(rest_angles, rest_angles, duration=1.0)

# --- Define gait phases ---
phase1 = {
    "hr_hip": math.radians(-25), "hr_knee": 0.7,
    "hl_hip": math.radians(-25), "hl_knee": 0.7,
    "fr_hip": math.radians(25),  "fr_knee": 0.7,
    "fl_hip": math.radians(25),  "fl_knee": 0.7,
}
phase2 = {
    "hr_hip": math.radians(25), "hr_knee": 0.7,
    "hl_hip": math.radians(25), "hl_knee": 0.7,
    "fr_hip": 0.0,  "fr_knee": 0.7,
    "fl_hip": 0.0,  "fl_knee": 0.7,
}
phase3 = {
    "hr_hip": math.radians(45), "hr_knee": 0.7,
    "hl_hip": math.radians(45), "hl_knee": 0.7,
    "fr_hip": math.radians(-45),  "fr_knee": 0.7,
    "fl_hip": math.radians(-45),  "fl_knee": 0.7,
}
phase4 = {
    "hr_hip": math.radians(25), "hr_knee": 0.7,
    "hl_hip": math.radians(25), "hl_knee": 0.7,
    "fr_hip": 0.0,  "fr_knee": 0.7,
    "fl_hip": 0.0,  "fl_knee": 0.7,
}
phase5 = {
    "hr_hip": math.radians(-25), "hr_knee": 0.7,
    "hl_hip": math.radians(-25), "hl_knee": 0.7,
    "fr_hip": math.radians(25),  "fr_knee": 0.7,
    "fl_hip": math.radians(25),  "fl_knee": 0.7,
}
phase6 = {
    "hr_hip": math.radians(-45), "hr_knee": 0.7,
    "hl_hip": math.radians(-45), "hl_knee": 0.7,
    "fr_hip": math.radians(45),  "fr_knee": 0.7,
    "fl_hip": math.radians(45),  "fl_knee": 0.7,
}

# --- Gait cycle ---
gait_phases = [phase1, phase2, phase3, phase4, phase5, phase6]

# --- Loop gait with smoothing ---
def run_gait_loop(phases, cycles=3, duration_per_phase=0.2):
    for _ in range(cycles):
        for i in range(len(phases)):
            current = phases[i]
            nxt = phases[(i + 1) % len(phases)]  # Loop back to start
            interpolate_angles(current, nxt, duration_per_phase)

# Run loop
print("Starting bounding loop...")
run_gait_loop(gait_phases, cycles=5, duration_per_phase=0.4)

# Hold final pose
print("Done.")
time.sleep(3)
p.disconnect()