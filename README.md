# To run the simulation, execute the following commands in the given order

##  compile the c codes 
  - mex Dis_Dir.c
  - mex fuzzy_ctrl.c

## To load data and open the SIMULINK model, run the matlab script " loadDataAndOpenModel.m" or use the following commands
  - load('Obs.mat')
  - load('path.mat')
  - open('FuzzyMobileRobotGuidance.mdl')
## To run 
  - Go to subblock 'Robot' and double click the 'Robot' 3D animation block
  - Run the simulation

## Configuration
 
  - During the simuation the target location and obstacle condition can be varied using the given GUI application

# System Block Diagram
![image](misc/Bdgm.png)

# Fuzzy implementation hints

## Block diagram
![image](misc/fuzzysystem.png)

## Fuzzification
![image](misc/fuzzification.png)

## Rule base
![image](misc/rulebase.jpg)

## Inference
![image](misc/inference.png)

## Defuzzification
![image](misc/defuzzification.jpg)

# Simulation Test Result
[![video](misc/thumnail.png)](https://youtu.be/69Ffs5SMVuA)
