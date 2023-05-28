# Antropomorfo
Anthropomorphic robot with 6 degrees of freedom through processing

Implementation of a university project for the subject Robotics. The track was as follows:

Using Processing, draw an ANTHROPOMORPHIC robot operated in REVERSE KINEMATICS. The desired value (xd,yd,zd) for the gripper position and its orientation
must be editable from the keyboard. In particular, as regards the position of the gripper, the xd, yd and zd coordinates can be modified as follows:
with lowercase x you decrease the xd coordinate, with uppercase X you increase it. Similarly, the letters y and z can be used for the coordinates yd and zd.
As regards the desired orientation of the gripper, i.e. the Re matrix, proceed as follows. Locate the z6 axis of the gripper using the azimuth α and elevation angles
β with respect to the basis system (x0,y0,z0) (refer to this figure), and define x6 and y6 by following a rotation of an angle θ about the z6 axis.
This can be shown to correspond to a parametrization of type ZYZ of the desired rotation matrix Re, with angles (α,90o-β,θ), thus coinciding with the expression
of the matrix R36 of the spherical pulse in which it is necessary to substitute α in place of θ4, 90o-β in place of θ5 and θ in place of θ6. To change the orientation of the gripper will then
it is sufficient to act on the variables α, β and θ by pressing, for example, the following keys: lowercase a to decrease α and uppercase A to increase it, lowercase b to decrease β and uppercase B
to increase it, lowercase t to decrease θ and uppercase T to increase it.
Write the sketch also taking into account the following specifications: 
1-During the entire execution of the program, the value of the desired coordinates (xd,yd,zd) for the position of the gripper and that (in degrees) of the angles (α,β,θ) must be shown on the screen
  which define the orientation of the gripper. The coordinates of the gripper must be written BY CHOOSING THE ONE CONSIDERED IN THE LESSON AS THE BASIC REFERENCE TRIO (x0,y0,z0)
  (and not the one used by Processing).
  
  2-Write on the screen also the desired Re matrix with columns of THREE DIFFERENT COLORS.
  
  3-DRAW both the TREAD (x0,y0,z0) of the BASE and that (x6,y6,z6) of the CLAMP using for the three axes x, y and z the SAME COLORS used for the columns of Re
  (i.e. the x-axis should be drawn the same color as the first column of D, the y-axis the same color as the second column, and the z-axis as the third column).
  For simplicity, the axes can be drawn without arrows.
  
  4-NEGLECTING for simplicity the problem of COLLISIONS between the various links of the robot.
  
  5-For simplicity the CLAMP can be represented as a simple PARALLELEPIPED.
   
  6-Envisage the possibility from the keyboard (for example with the '+' and '-' keys) to pass from the HIGH ELBOW solution to the LOW ELBOW solution. Instead, fix the solution arbitrarily for the spherical pulse.
  
  7-Include the features (already implemented in the various sketches seen in class) that allow:
  1) to change the height of the view,
  2) to modify the value of the constant Kp of the control law,
  3) to move the base of the robot with a click of the mouse.
  
  Note: Our implementation adds a check on alpha beta and theta rotations. When the robot performs one of these rotations, and leaves the workspace, the robot will
  will stop and the red message "Out of workspace!" will appear.
