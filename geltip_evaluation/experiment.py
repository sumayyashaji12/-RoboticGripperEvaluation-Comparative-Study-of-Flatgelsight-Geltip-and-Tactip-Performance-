### An example wrapper that takes in a Simulation object and implements methods that allow controlling
### the gripper modularly
### We provide primitives for performing the following sub-routines:
### - opening/closing the gripper
### - moving the gripper to a specific point in space
### - shaking the gripper

### We combine those primitives into a grasping routine where the gripper starts high above the table,
#### Then it is lowered till it reached the surface, closes on the object and goes back up, before shaking to test the grasp goodness

### Notw:
### We also show, in the __main__ section, how to stabilize any loaded object at the center of the table
### The strategy used is to load the object and record the pos and orn at which it stabililizes
### Then exploit this info in a relaunch
import os
import numpy as np
import time
import argparse
import csv
from simulation import Simulation
from paths import *
from utils import *

GRASP_TIMEOUT = 5 

class Dummy_Grasp_Routine_Wrapper:

    def __init__(self, sim):
        # Provided sim must be reset to any desired state before passing it to here
        self.sim = sim
        self.step_to_sec = 1/240
        self.torques = [0]*self.sim.num_joints
        self.success_count = 0  # Initialize success_count attribute
        self.total_attempts = 0  # Initialize total_attempts attribute
        # No debug sliders (unlike GUI) so no need to keep track of currnt location, camera info etc
        self.sim.p.resetDebugVisualizerCamera(cameraDistance=0.5, cameraYaw=90, cameraPitch=187, cameraTargetPosition=[0,0,0.158])
        return


    def advance_time(self, seconds):
        assert(seconds >= self.step_to_sec)
        num_steps = int((1 / (self.step_to_sec))*seconds)
        
        for i in range(num_steps): 
            self.apply_torques()
            self.sim.p.stepSimulation()
            time.sleep(self.step_to_sec)
        return

    def apply_torques(self):
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
        self.torques[self.sim.gripper_joint_indices[0]] = -force
        self.torques[self.sim.gripper_joint_indices[1]] = force
        return

    def set_gripper_close(self, force=100):
        self.torques[self.sim.gripper_joint_indices[0]] = force
        self.torques[self.sim.gripper_joint_indices[1]] = -force
        return


    #### sub-soutines:
    #### see R_grasp_object for an example of a routine that uses sub-routines as primitives
    def R_open_gripper(self):
        self.set_gripper_open(force=100)
        self.advance_time(0.05)
        pass

    def R_close_gripper(self,force=40):
       
        for i in np.arange(0,force,1):
            self.set_gripper_close(force=i)
            self.advance_time(self.step_to_sec)
        return

    def R_force_move_to_xyz(self, dst_xyz, timeout=None):
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
    def R_grasp_object(self, close_gripper_force, shake_gripper_amp, force_move_xyz):
        self.advance_time(1)
        self.R_open_gripper()
        self.R_force_move_to_xyz(dst_xyz=force_move_xyz)
        self.R_close_gripper(force=close_gripper_force)
        self.R_force_move_to_xyz(dst_xyz=[0,0,0.250])
        self.R_shake_gripper(amp=shake_gripper_amp)
        # Evaluate success of the grasp
        success = self.check_if_object_held()
        self.success_count += int(success)
        self.total_attempts += 1
        
        
        return
    def check_if_object_held(self):
       # Get the live position of the object
       object_position = self.get_live_position_object()

       # Check if the object position matches the expected position
       expected_position = [0,0,0.140]
       tolerance = 0.1  # Increased tolerance
       object_held = np.allclose(object_position, expected_position, atol=tolerance)

      
      
       return object_held
    def get_live_position_object(self):
        # Get the live position of the object using the object_id from the Simulation class
        object_position = self.sim.p.getBasePositionAndOrientation(self.sim.object_id)[0]
        return object_position

    def get_success_rate(self):
        if self.total_attempts == 0:
         return 0
        rate = self.success_count / self.total_attempts * 100
        
        return rate
       
    

    ##### Helpers
    def get_live_position_gripper(self):
        return [self.sim.p.getLinkState(bodyUniqueId=self.sim.arm_id, linkIndex=i)[0][i] for i in range(3)]


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Run a basic routine')
    parser.add_argument('--obj_path', type=str, required=True)
    args = parser.parse_args()

    # A new urdf file for whichever object was picked at args.obj_path
    object_urdf_path = edit_object_name_in_urdf_file(local_object_path=args.obj_path, urdf_path=ORIGINAL_OBJECTS_URDF_PATH)

    # Create a fresh sim, wrap it in the GUI wrapper and launch the wrapper
    import pybullet as p
    sim = Simulation(p, debug=False)

    ### Stabilize the object being loaded by loading it in an unstable pos and orn first, letting it stabilize, 
    ### then read off the stable pos and orn and use that to reset the sim
    
    # Unstable
    sim.reset_state(include=['object', 'gripper', 'plane'], object_urdf_path=object_urdf_path)
    # Step for 3s to let stabilize and read off stable pos and orn 
    for i in range(int(3/(1/240))): sim.p.stepSimulation()
    pos, orn = sim.p.getBasePositionAndOrientation(bodyUniqueId=sim.object_id)
    # Start at origin, and right on the plane surface
    stable_pos = [0,0,pos[1]]
    upright_orn = sim.p.getQuaternionFromEuler([ 0,  np.pi / 2,   np.pi / 2])
    # Start at last measured stable orn
    #stable_orn = orn
    # Stable
    sim.reset_state(include=['object', 'gripper', 'plane'], object_urdf_path=object_urdf_path, object_pos=stable_pos, object_orn=upright_orn, gripper_h=0.5)
    # Wrap then launch wrapper
    routine_wrapper = Dummy_Grasp_Routine_Wrapper(sim=sim)
    gripper_name = 'geltip_gripper'
    object_name = 'cube'
    close_gripper_forces = range(30, 40, 50 )
    shake_gripper_amplitudes = [0.1, 0.3, 0.5]
    force_move_xyz_values = [[-0.01,0.02,0.149],[-0.01,0.01,0.147]]  #for x in np.arange(-0.05, 0.051, 0.01)
                         #for y in np.arange(-0.05, 0.051, 0.01)
                         #for z in np.arange(0.130, 0.161, 0.01)]
    # Check if the file is empty
    file_is_empty = not os.path.isfile('experiment_results.csv') or os.path.getsize('experiment_results.csv') == 0

    with open('experiment_results.csv', mode='a', newline='') as csv_file:
        writer = csv.writer(csv_file)
        # If the file is empty, write the header row
        if file_is_empty:
            writer.writerow(['Gripper', 'Object', 'Close Gripper Force', 'Shake Gripper Amplitude', 'Force Move XYZ', 'Success_Count'])
        for force in close_gripper_forces:
          for amp in shake_gripper_amplitudes:
             for xyz in force_move_xyz_values:
                for _ in range(5):
                    routine_wrapper.R_grasp_object(close_gripper_force=force, shake_gripper_amp=amp, force_move_xyz=xyz)
                    # Reset the simulation state if needed
                    #sim.reset_state(include=['object', 'gripper', 'plane'], object_urdf_path=object_urdf_path)
                    sim.reset_state(include=['object', 'gripper', 'plane'], object_urdf_path=object_urdf_path, object_pos=stable_pos, object_orn=upright_orn, gripper_h=0.5)
                    sim.p.resetDebugVisualizerCamera(cameraDistance=0.5, cameraYaw=90, cameraPitch=187, cameraTargetPosition=[0,0,0.158])
                success_count = routine_wrapper.success_count
                print(f"Total successes for force {force}, amplitude {amp}, XYZ {xyz}: {success_count} out of 10 trials")
                # Write the result to the CSV file, including the gripper and object name
                writer.writerow([gripper_name, object_name, force, amp, xyz, success_count])
                   
                routine_wrapper.success_count = 0
                routine_wrapper.total_attempts = 0