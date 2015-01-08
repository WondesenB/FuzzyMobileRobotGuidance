# To run the simulation, execute the following commands in the given order

  ----- compile the c codes ------
  - mex Dis_Dir.c
  - mex fuzzy_ctrl.c

  ---- load data and open model ----
  - load('Obs.mat')
  - load('path.mat')
  - open('FuzzyMobileRobotGuidance')
  - Go to subblock 'Robot' and double click the 'Robot' 3D animation block
  - Run the simulation

# Configuration
 
  - During the simuation the target location and obstacle condition can be varied using the given GUI application

