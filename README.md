# -RoboticGripperEvaluation-Comparative-Study-of-Flatgelsight-Geltip-and-Tactip-Performance-Gripper Experiments
This repository consists of experiments for three different grippers:

- FlatGelSight Evaluation
- TacTip Evaluation
- GelTip Evaluation
- 
Each folder contains:

Gripper mesh and URDF files
Object URDF files

The dummy_grasp_routine wrapper has been used to experiment with various settings, including:

- Different objects
- Shaking amplitude
- Gripping force
- Grasp position
- Object size
To view the simulation of each gripper : Opening/closing the gripper, Moving the gripper to a specific point in space, Shaking the gripper
Example command: python dummy_grasp_routine_wrapper.py --obj_path ./sample_raw_meshes/cracker_box.obj
  
Machine Learning Analysis:
The data collected from the three gripper experiments are combined to perform a machine learning analysis. A Random Forest algorithm is employed to predict the best-suited gripper for a specific object.

Credits
The dummy_grasp_routine was initially taken from another GitHub repository. Although modifications were made for this project, we'd like to give credits to the original repository (https://github.com/jaks19/parallel_gripper_simulation_pybullet.git).

Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

License
MIT







