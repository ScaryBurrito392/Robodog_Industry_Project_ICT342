## This directory contains classes relating the the dog's lidar and navigation

## The following are the classes intended to be accessed:

# Classes:

## DataHolder:

### This class is intended to hold data obtained from the dog such as lidar data. Data can be place into it from inside asyncio subprocess that handles receiving data, and data can be retrieved from it from outside the subprocess.
### It has the advantage of being thread safe as well as blocking when the holder is empty and an attempt to get data is made .

## PositionProcessor:

### This class is intended to handle the passing of data between the asyncio subprocess and the main process. Two data holders, one for the dog's pose and one for the lidar data are passed into static variables, so are accessible both inside and outside the asyncio subprocess without needing to be directly passed.
### update_message is called to update the data within the position processor to the latest data provided by the dog, then the data within can be altered in multiple ways and retrieved.

## Point accumulator:

### This class attempts to accumulate points and to form a global map. It does this by using the dog's estimated position of the lidar frame to place the points into their positions in an underlying grid.
### Each time the accumulator is fed a new frame, it takes the points, only keeps previous points in the area of the frame if they could be blocked by a point in the latest frame, and adds all the points from the latest frame. This is in an attempt to only use the latest frames but to not throw out points that are just blocked.

## Controller:

### This class allows for programmatically controlling the dog. It attempts to allow for movement requests to be made from outside an asyncio subprocess by just launching a movement queue listener within the asyncio subprocess, and then movements can be passed to it from outside. It also allows for blocking until the provided movement is completed using queue logic.
### It also can listen to key presses and make corresponding moves (standard wasd and q for anti-clockwise, e for clockwise)

## Movement:

### This just represents either a forward movent or a rotation, and is what the controller queue accepts

## PrecisionMover:

### This tries to overcome any inaccuracies in the dogs movements by making small corrections after the dog moves to move it to where the movement should have taken it. Still experimental

## SimpleNavigator:

### This tries to navigate an enclosed single-direction corridor by checking the distances to the dog's left, right and in front. It then tries to make navigation decisions based on this information. Still a prototype and an attempt to conceptualise how the navigation could be done without the use of a global consistent map.