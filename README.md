# Robot Arm Control Team Tasks

### Main Tasks

**1.** Forward Kinematics (The easiest one, we just need to know where we are)

**2.** Inverse Kinematics (This one requires more consideration)

**3.** Motion Planning (This is related with Inverse Kinematics, you need to consider motor constraints and physical constraints)

**4.** Simulate Robot Arm (Whatever you guys chose to use)

**5.** Consider how to implement the controller (remote with IMU and mapping the position of the end effector to the remote in space)

### Requirements

**1.** It needs to run on an STM 32 so you will likely have to implement these yourself

**2.** Stable movement (it should not tear itself to pieces and trajectories should be as smooth and precise as possible)

**3.** Real time IK (we need to be able to move the end effector around in real time)

### Resources:

**1.** There are many online

**2.** https://robotics.caltech.edu/~jwb/courses/ME115/handouts/damped.pdf (for damped jacobian pseudo inverse for IK)

Good Luck 👍


### Addendum 

**Denavit-Hartenberg Parameters**: https://youtu.be/rA9tm0gTln8?si=4x_G1ldNQupkrWpk

**The Paper**: https://www.researchgate.net/publication/360442615_Forward_Inverse_Kinematics_Solution_of_6-DOF_Robots_Those_Have_Offset_Spherical_Wrists