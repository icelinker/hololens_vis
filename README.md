# hololens_vis
This is a package for integrating the Microsoft Hololens with ROS. The connection is made using rosbridge, this functionality was borrowed from https://github.com/soliagabriel/holoROS . 


optimiseTranforms.py allows synchronisation of the Hololens reported position and a marker tracked using an external system, in our case a retro-reflective motion capture system. This node performs an optimisation to estimate jointly the world offset for the Hololens and the offset from the Hololens centre and the tracked marker. This node should run continuously to account for the shift in the optical odometry. A rough calibration is performed using code borrowed from Nghia Ho at  http://nghiaho.com/uploads/code/rigid_transform_3D.py_ .

The world transform for the Hololens is published as a pose message. Applying this to the locations of objects tracked by the motion capture will place them in the correct place in the Hololens view to make them align correctly. 

This is a work in progress, the accompanying Unity project will be forthcoming. 
