# pheromone_map

#### How to Run
rosrun pheromone_controller wrapper_pheromone_controller.py


#### Parameters Essential (also what they are currently set to)


- how often we want to update robot's local stigmergy (integer in Hz)
```
publisher_rate = 2
```

- resolution of the cells in the stigmergy map and  (in meters)
```
map_resolution = 0.25
```
- the radius of the trail of a single robot leaves on the pheromone map (in meters)
```
trail_radius = 0.8
```
- the value of the robot's trail in the matrix (integer)
```
robot_trail_value = 75
```
- the value of the obstacles trail in the matrix (integer)
```
wall_trail_value = 255
```

- These are diffusion parameters (Sigma is for gaussian blurr)
```
diffusion_sigma = 0.75
diffusion_rate = 10
```


# TESTING DATA



#### Data Coming In


- Should read all the name spaces of all robots in the environmentand print these to the screen as it's setting up.

- Should listen to the following topics
```
'/ground/localPheromone/<robotNameSpace>'
```
- 

#### Internal Behaviour

- Pheromones left by robots should be much less than those left by the walls
- Pheromones left by robots should stack over time
- Pheromones left by robots should be in a separate array from those left by the walls



#### Data Going Out

- Should Publish to the following topics `<robot_name>/pheromoneHeading`


