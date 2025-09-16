# Navigation README

Within this subdirectory is a python wheel which, upon installation, gives access to a python library for pathfinding, 
as well as the source code used to construct the library.

## Installation:
To install the package, run the command: 
#### pip install <path to the .whl file>

## Importing:
To use the library once it is installed, import the library into your python code with
#### from go_pathfinder import navigation
## Classes:
There are 2 classes within the navigation library:
### EntityMover:
This class is responsible for taking information about the entity's environment and providing movements to the entity 
to allow it to navigate to the end position
#### Constructor:
Parameters:

    end_relative_x (int): the x position of the goal relative to the entity's start position
    end_relative_y (int): the y position of the goal relative to the entity's start position
    obstacle_radius (int): the radius around an added point that is considered an obstacle
    max_straight_length (int, optional): the maximum magnitude of a provided movement; default -1

#### update_map:
Updates the entity position and obstacles on the map

Parameters:

    entity_x (int): the current x position of the entity
    entity_y (int): the current y position of the entity
    added_obstacles (list[tuple[int, int]]): a list of the positions of newly added obstacles
    removed_obstacles (list[tuple[int, int]]): a list of the positions of newly removed obstacles

#### get_entity_position:
Returns:

    tuple(int, int): the current position of the entity

#### get_next_movement:
Returns:

    EntityMovement: the next movement for the entity

Parameters:

    entity_x (int): the current x position of the entity
    entity_y (int): the current y position of the entity
    angle_adjustment (double, optional): the angle_adjustment required for the entity to be facing upwards on the map; default 0.0
Returns:

    EntityMovement: the movement containing angle and magnitude

### EntityMovement:
This class holds the angle and magnitude of a movement

#### Constructor 1:
Parameters:

            angle (double): the angle of the movement
            magnitude (double): the magnitude of the movement

#### Constructor 2:
Parameters:

    start_position (int): the start position of the movement
    end_position (int): the end position of the movement
    angle_adjustment (double, optional): the angle_adjustment required for the entity to be facing upwards on the map; default 0.0

#### get_angle:
Returns:

    double: the angle of the movement

#### get_magnitude:
Returns:

    double: the magnitude of the movement

## Intended Usage:
The go_pathfinder navigation library is intended to be used in conjunction with a system that will provide data 
regarding the environment of the entity

### Setting Up:
Before beginning, some decisions must be made. For example, the navigation library works using a coordinate system, 
however, these can represent practically any distance within the entity's environment. The system interacting with the navigation system must decide on the real world size of a coordinate unit is, and apply conversion when interacting with the navigation system.

Upon initialization of the EntityMover object, of which only one is required for a singular entity, 4 variables can be passed, with the first 3 being required.

The first 2 are the x (ascending goes to the right) and y (ascending goes forwards) coordinates of the finish position relative to the start position of the entity. For the navigation of the maze, while the exact placement of the finish doesn't need top be precise, as the pathfinding program will find the nearest exit regardless, it is more efficient if it is in the right direction and as close as possible to the outside of the actual end position of the maze.

Next is the obstacle radius. Internally, the entity is only as big as a unit, so padding is added to obstacles to ensure the entity stays an adequate distance from obstacles. This should be set to at least half of the longest straight measurement of the entity.

Next is the max_straight_length. This determines the maximum length of movements that will be provided. This is an option so that a movement can be limited to only go as far as is desired, perhaps to stay within the range that the entity was able to scan before moving. 

Once the EntityMover is initialized, the get_entity_position method should be used to determine the entity's starting coordinates within the navigation system to allow for the potential differences in internal coordinates between the environmental monitoring system and the navigation system.

Before beginning navigation, it is advisable to update the map with any obstacles observed by the environment monitoring system, using the update_map method, which takes the entity coordinates and lists of tuples containing 2 ints, representing the x and y coordinates for both added obstacles and removed obstacles.

### Navigating:
Once the EntityMover is set up, the get_next_movement method, which again takes the x and y position of the entity should be called. An optional parameter is the angle_adjustment, which represents the offset required for the entity to be facing upward on the internal navigation map. Using this will ensure that the direction is correct relative to the entity.

The get_next_movement method will return an EntityMovement object, which is immutable and contains the magnitude and angle of the next movement the entity should make. The direction is measured in degrees with 0 degrees being straight forward, angles anticlockwise of 0 being positive and angles clockwise of 0 being negative.

Once the entity has moved, newly detected added or removed obstacles should be passed to the update_map method along with the entity position, and the get_next_movement method called to determine the next movement.


