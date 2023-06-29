# bio_ik_experiment

Bio IK experiment with Look at goal + Min/Max distance + Balance Robot + Center Joints.

## Demo

- Modify xarm7's kinematics.yaml for Bio IK like this:

  ```
  xarm7:
    kinematics_solver: bio_ik/BioIKKinematicsPlugin
    kinematics_solver_search_resolution: 0.005
    kinematics_solver_timeout: 0.02
    kinematics_solver_attempts: 1
  ```

- Run the demo:

  ```bash
  $ roslaunch xarm7_moveit_config demo.launch
  ```

- Run the solver:

  ```bash
  $ rosrun bio_ik_experiment bio_ik_experiment
  ```


## Video

https://www.youtube.com/watch?v=RKVN-GoH-l0

[![Watch the video](https://img.youtube.com/vi/RKVN-GoH-l0/maxresdefault.jpg)](https://youtu.be/RKVN-GoH-l0)