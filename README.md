# Robotics-2D-Occupancy-Grid-Mapping
Implementation of Occupancy Grid Mapping algorithm for a 2D floor map. It incorporates range sensor readings and known poses at each measurement time to build a map.
Mapping is essential for SLAM.
Prerequisites are the knowledge of coordinate system transformations. Map coordinate frame ​vs.​body coordinate frame
Important parameters include; Map resolution ​- The resolution that determines how finely your map will be discretized, Map size in pixels, the origin of your range scanner, log_odd_occ: This parameter is for updating the occupied cell measurements, log_odd_free: This parameter is for updating the free empty cell measurements, log_odd_max: The maximum value for log_odd of your map, log_odd_min: The minimum value for log_odd of your map. 

The reason for constructing the map is because range sensors are not 100% accurate and there can be deviations on ranges on the same object. If we take log_odd_occu = 0.9 and log_odd_free = -0.7. Then, obviously, the higher the value of a cell state, the more it indicates that the cell is occupied, and the lower the number of the other way, the more it means that the grid is idle. In the results, darker colors indicate that the raster is free, and lighter colors indicate that they are occupied. Here is the plot output. Inputs to the algorithm are the x and y coordinates of the robot and its heading angle in the world coordinate frame, map parameters ie resolution, map size, and distance and scan angles of the range sensor.
 
![occupancy grid map](https://github.com/chumoyot/Robotics-2D-Occupancy-Grid-Mapping/assets/135506318/3a0e8664-6bb1-4a6d-aa8e-1a02f0336516)
