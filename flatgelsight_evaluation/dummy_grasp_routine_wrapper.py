### An example wrapper that takes in a Simulation object and implements methods that allow controlling
### the gripper modularly

# Available gripper control sub-routines:
### - opening/closing the gripper
### - moving the gripper to a specific point in space
### - shaking the gripper

# The grasping routine leverages these primitives, starting with the gripper positioned above the table.
# The routine involves lowering the gripper to the surface, grasping the object, lifting, and then shaking to test the grasp's stability.

import numpy as np
import time
import argparse

from simulation import Simulation
from paths import *
from utils import *
import pybullet as p

GRASP_TIMEOUT = 5 

class Dummy_Grasp_Routine_Wrapper:

    def __init__(self, sim):
        # Provided sim must be reset to any desired state before passing it to here
        self.sim = sim
        self.step_to_sec = 1/240
        self.torques = [0]*self.sim.num_joints

        # No debug sliders (unlike GUI) so no need to keep track of currnt location, camera info etc
        self.sim.p.resetDebugVisualizerCamera(cameraDistance=0.5, cameraYaw=90, cameraPitch=187, cameraTargetPosition=[0,0,0.158])
        return


    def advance_time(self, seconds):
        """
   Advances the simulation by a specified number of seconds.
   
   Parameters:
   - seconds (float): Number of seconds to advance the simulation by.
   """
        assert(seconds >= self.step_to_sec)
        num_steps = int((1 / (self.step_to_sec))*seconds)
        
        for i in range(num_steps): 
            self.apply_torques()
            self.sim.p.stepSimulation()
            time.sleep(self.step_to_sec)
        return

    def apply_torques(self):
        """
    Applies the set torques to the gripper's joints in the simulation.
    """
        self.sim.p.setJointMotorControlArray(bodyIndex=self.sim.arm_id, 
                                    jointIndices=list(range(self.sim.num_joints)[4:]), 
                                    controlMode=self.sim.p.VELOCITY_CONTROL, 
                                    forces=([0]*self.sim.num_joints)[4:])

        self.sim.p.setJointMotorControlArray(bodyIndex=self.sim.arm_id,
                                    jointIndices=list(range(self.sim.num_joints)),
                                    controlMode=self.sim.p.TORQUE_CONTROL,
                                    forces=self.torques)
        return

    #### Setters change the sim wrapper state but do not move time
    def set_gripper_open(self, force):
        """
   Sets the gripper to an open state using a specified force.
   
   Parameters:
   - force (float): The force to be applied while opening the gripper.
   """
        self.torques[self.sim.gripper_joint_indices[0]] = -force
        self.torques[self.sim.gripper_joint_indices[1]] = force
        return

    def set_gripper_close(self, force=100):
        """
   Sets the gripper to a closed state using a specified force.
   
   Parameters:
   - force (float): The force to be applied while closing the gripper. Default is 100.
   """
        self.torques[self.sim.gripper_joint_indices[0]] = force
        self.torques[self.sim.gripper_joint_indices[1]] = -force
        return


    #### sub-soutines:
    #### see R_grasp_object for an example of a routine that uses sub-routines as primitives
    def R_open_gripper(self):
        self.set_gripper_open(force=100)
        self.advance_time(0.05)
        pass

    def R_close_gripper(self):
        force = 40
        for i in np.arange(0,force,1):
            self.set_gripper_close(force=i)
            self.advance_time(self.step_to_sec)
        return

    def R_force_move_to_xyz(self, dst_xyz, timeout=None):
        """
    Routine to move the gripper to a specified xyz coordinate in the simulation.
    
    Parameters:
    - dst_xyz (list): The target xyz coordinates.
    - timeout (float, optional): Maximum time to attempt the move before raising a TimeoutError.
    """
        start_time = time.time()

        self.sim.p.setJointMotorControlArray(bodyIndex=self.sim.arm_id,
                                    jointIndices=[0,1,2],
                                    controlMode=self.sim.p.POSITION_CONTROL,
                                    targetPositions=np.array(dst_xyz)-np.array([0,0,self.sim.gripper_susp_h]),
                                    targetVelocities=[1e-8]*3)

        while True: 
            self.advance_time(self.step_to_sec)
            if np.allclose(self.get_live_position_gripper(), dst_xyz, rtol=1e-3, atol=1e-3): break
            elif time.time()-start_time > GRASP_TIMEOUT: raise TimeoutError
        return

    def R_shake_gripper(self, amp=0.5):
        """
    Routine to shake the gripper to test the stability of the grasp.
    
    Parameters:
    - amp (float): Amplitude of the shaking motion. Default is 0.3.
    """

        for i in range(200):
            dirn = 1 if (i%40<10 or (i%40>=30 and i%40<40)) else -1
            self.sim.p.setJointMotorControlArray(bodyIndex=self.sim.arm_id,
                                                 jointIndices=[0,1,2],
                                                 controlMode=self.sim.p.VELOCITY_CONTROL,
                                                 targetVelocities=[0,dirn*amp,0])
            self.advance_time(self.step_to_sec)

        self.sim.p.setJointMotorControlArray(bodyIndex=self.sim.arm_id,
                                                 jointIndices=[0,1,2],
                                                 controlMode=self.sim.p.VELOCITY_CONTROL,
                                                 targetVelocities=[0,0,0])
        self.advance_time(self.step_to_sec)
        return


    #### Example routine
    #### Here is an example of a routine (uses sub-routines)
    def R_grasp_object(self):
        """
   A sample routine that demonstrates the process of grasping an object in the simulation.
   It includes opening the gripper, moving to a target location, closing the gripper, 
   lifting and shaking the gripper.
   """
        self.advance_time(1)
        self.R_open_gripper()
        self.R_force_move_to_xyz(dst_xyz=[0.0,0.0,0.14])
        self.R_close_gripper()
        self.R_force_move_to_xyz(dst_xyz=[0,0,0.250])
        self.R_shake_gripper()
        return

    ##### Helpers
    def get_live_position_gripper(self):
        """
   Gets the current xyz position of the gripper in the simulation.
   
   Returns:
   - list: Current xyz coordinates of the gripper.
   """
        return [self.sim.p.getLinkState(bodyUniqueId=self.sim.arm_id, linkIndex=i)[0][i] for i in range(3)]


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Run a basic routine')
    parser.add_argument('--obj_path', type=str, required=True)
    args = parser.parse_args()

    # A new urdf file for whichever object was picked at args.obj_path
    object_urdf_path = edit_object_name_in_urdf_file(local_object_path=args.obj_path, urdf_path=ORIGINAL_OBJECTS_URDF_PATH)

    # Create a fresh sim, wrap it in the GUI wrapper and launch the wrapper

    sim = Simulation(p, debug=True)

    sim.reset_state(include=['object', 'gripper', 'plane'], object_urdf_path=object_urdf_path)
    # Wrap then launch wrapper
    routine_wrapper = Dummy_Grasp_Routine_Wrapper(sim=sim)
   
    routine_wrapper.R_grasp_object()