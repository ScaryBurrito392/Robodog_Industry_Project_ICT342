""" Inspired by lidar_stream.py and plot_lidar_stream.py by @legion1581 and @MrRobotoW at The RoboVerse Discord """
""" Authored by Jude from the UniSC industry project team """
# Really sorry about the massive file. Didn't want to risk breaking anything and not having the dog to be able to test
# if it all still works when modularized

import json
import queue
import threading
import keyboard
from go2_webrtc_driver.constants import RTC_TOPIC, SPORT_CMD
import asyncio
import logging
import numpy as np
from bitarray import bitarray
from go2_webrtc_driver.webrtc_driver import Go2WebRTCConnection, WebRTCConnectionMethod
from scipy.spatial.transform import Rotation as R
import pygame

reconnect_interval = 5  # Time (seconds) before retrying connection
MAX_RETRY_ATTEMPTS = 3
LIDAR_X_MAX = 129
LIDAR_Y_MAX = 129
LIDAR_Z_MAX = 39
RESOLUTION = 0.05



class ThreeDBitArray:
    """
    A 3d bit array that uses a 1d bit array as the underlying data structure
    """

    def __init__(self, x_size: int, y_size: int, z_size: int):
        """
        Create an instance of ThreeDBitArray

        Args:
            x_size (int): the length of the x-axis
            y_size (int): the length of the y-axis
            z_size(int): the length of the z-axis
        """
        self.x_size = x_size
        self.y_size = y_size
        self.z_size = z_size
        self.y_z_prod = y_size * z_size
        self.array = bitarray(x_size * self.y_z_prod)
        self.array.setall(False)  # initialize to 0 (False)
        self.relative_neighbour_positions = np.array(  # all relative neighbour positions
            ((-1, -1, -1), (-1, -1, 0), (-1, -1, 1), (-1, 0, -1), (-1, 0, 0), (-1, 0, 1), (-1, 1, -1), (-1, 1, 0),
             (-1, 1, 1),
             (0, -1, -1), (0, -1, 0), (0, -1, 1), (0, 0, -1), (0, 0, 1), (0, 1, -1), (0, 1, 0), (0, 1, 1),
             (1, -1, -1), (1, -1, 0), (1, -1, 1), (1, 0, -1), (1, 0, 0), (1, 0, 1), (1, 1, -1), (1, 1, 0), (1, 1, 1)))


    def _index(self, x: int, y: int, z: int) -> int:
        """
        Get the index of the bit at logical position x, y, z

        Args:
            x (int): the x position of the bit
            y (int): the y position of the bit
            z (int): the z position of the bit

        Returns:
            int: the index of the bit at position x, y, z
        """
        if not (0 <= x < self.x_size and 0 <= y < self.y_size and 0 <= z < self.z_size):
            raise IndexError("Index out of bounds")
        return int(x * self.y_z_prod + y * self.z_size + z)


    def get(self, x: int, y: int, z: int) -> int:
        """
        Get the value of the bit at logical position x, y, z

        Args:
            x (int): the x position of the bit
            y (int): the y position of the bit
            z (int): the z position of the bit

        Returns:
            the value of the bit at logical position x, y, z
        """
        return self.array[self._index(x, y, z)]


    def set(self, value: bool, x: int, y: int, z: int):
        """
        Get the value of the bit at logical position x, y, z

        Args:
            value (bool): the value to be set for the bit at logical position x, y, z
            x (int): the x position of the bit
            y (int): the y position of the bit
            z (int): the z position of the bit
        """
        self.array[self._index(x, y, z)] = value


    def shape(self) -> tuple:
        """
        Get a tuple representing the shape of the grid

        Returns:
            tuple: the shape of the grid in the format (x size, y size, z size)
        """
        return self.x_size, self.y_size, self.z_size


    def get_neighbours(self, x: int, y: int, z: int):
        """
        Get a list of valid neighbours that exist within a 3 * 3 cube centered around the provided point

        Args:
            x (int): the x position of the bit
            y (int): the y position of the bit
            z (int): the z position of the bit
        Returns:
            list: a list of valid neighbours
        """
        pos = np.array((x, y, z))
        return self.get_neighbours_np(pos)


    def get_neighbours_np(self, pos: np.ndarray):
        """
        Get a list of valid neighbours that exist within a 3 * 3 cube centered around the provided point

        Args:
            pos (np.ndarray): the position of the point

        Returns:
            list: a list of valid neighbours
        """
        neighbour_positions = self.relative_neighbour_positions + pos
        valid_neighbours = []
        for neighbour in neighbour_positions:
            if 0 <= neighbour[0] < self.x_size and 0 <= neighbour[1] < self.y_size and 0 <= neighbour[
                2] < self.z_size and self.get(neighbour[0], neighbour[1], neighbour[2]):
                valid_neighbours.append(neighbour)
        return valid_neighbours



class DataHolder:
    """
    An object that holds a single piece of data
    """
    def __init__(self):
        self.q = queue.Queue(maxsize=1)


    def get(self):  # blocking if empty
        """
        Get the next piece of data from the DataHolder, blocking until a piece of data is available

        Returns:
            The piece of data most recently entered into the DataHolder
        """
        return self.q.get()


    def put(self, message):
        """
        Put a piece of data into the DataHolder

        Args:
            message: the piece of data entered into the DataHolder
        """
        if self.q.full():
            try:
                self.q.get_nowait()
            except queue.Empty:
                pass
        try:
            self.q.put_nowait(message)
        except queue.Full:
            pass



class PositionProcessor:
    """
    Collects lidar and pose data and allows for the data to be processed and provided
    """
    lidar_holder = None
    pose_holder = None


    @staticmethod
    def initialize(lidar_holder: DataHolder, pose_holder: DataHolder):
        """
        Set the lidar and pose data holders for any PositionProcessor instance

        Args:
            lidar_holder (DataHolder): the lidar data holder
            pose_holder (DataHolder): the pose data holder
        """
        PositionProcessor.lidar_holder = lidar_holder
        PositionProcessor.pose_holder = pose_holder



    def __init__(self, min_z=0, max_z=38, z_processing_padding=2,
                 min_cluster_size=4):
        """
        Create an instance of PositionProcessor

        Args:
            min_z: the minimum z value kept when a height filter is applied
            max_z: the maximum z value kept when a height filter is applied
            z_processing_padding: the decrease of min_z and increase of max_z applied for pre-processing z filtering
        """
        #there is a difference between preprocessing and post-processing so that when performing clustering,
        #it is possible to cut away most of the irrelevant z values, leaving some extra at the top and the bottom that
        #should ideally include the floor so that anything connected to the floor is considered part of the floor
        #cluster so that it is not accidentally excluded
        self.pose_holder = PositionProcessor.pose_holder
        self.lidar_holder = PositionProcessor.lidar_holder
        self.yaw = 0
        self.relative_coordinates = None
        self.absolute_coordinates = None
        self.min_z = min_z
        self.max_z = max_z
        self.z_processing_padding = z_processing_padding
        self.min_cluster_size = min_cluster_size
        self.origin = None
        self.points = None


    def update_message(self):
        """
        Update the values associated with the lidar data and the pose data to the latest available data
        """
        message = self.lidar_holder.get()
        self.origin = message["data"].get("origin", [])
        positions = message["data"]["data"].get("positions", [])
        self.points = np.unique(np.array([positions[i:i + 3] for i in range(0, len(positions), 3)], dtype=np.float32),
                                axis=0)
        pose_message = self.pose_holder.get()
        origin = self.lidar_holder.get()["data"].get("origin", [])
        self.yaw = self.get_state_from_quaternion(*self.get_orientation_from_message(pose_message))[2]
        self.relative_coordinates = self.get_dog_relative_coordinates(self.get_position_from_message(pose_message), origin)
        self.absolute_coordinates = self.get_position_from_message(pose_message) / RESOLUTION



    def get_origin(self):
        """
        Get the origin provided by the lidar data

        Returns:
            The origin
        """
        return self.origin


    def get_points(self):
        """
        Get the lidar points provided by the lidar data

        Returns:
            The lidar points
        """
        return self.points


    def _set_points(self, points):
        """
        Set the current lidar points for the PositionProcessor

        Args:
            points: the current lidar points
        """
        self.points = points


    def compress_z(self):
        """
        Compress the current points into a 2D plane so that all points have a z of 0, so that if a point for an x, y point
        exists at any z, that position is considered occupied, so one point at that x, y and z = 0 exists

        Returns:
            PositionProcessor: this object
        """
        self.points = PositionProcessor.compress_z_points(self.points)
        return self #allows chained operations




    def filter_point_zs(self, pre_processing=False):
        """
        Filter the current lidar points by their z position, keeping only those within the specified valid range

        Returns:
            PositionProcessor: this object
        """
        min_z = self.min_z
        max_z = self.max_z
        if pre_processing:
            min_z -= self.z_processing_padding
            max_z += self.z_processing_padding
        filtered_points = []
        points = self.get_points()
        for point in points:
            if min_z <= point[2] <= max_z:
                filtered_points.append(point)
        self._set_points(np.array(filtered_points))
        return self #allows chained operations


    def enforce_minimum_cluster_size(self):
        """
        Remove all points that are parts of clusters below a minimum cluster size

        Returns:
            PositionProcessor: this object
        """
        min_processing_z = min(self.min_z - self.z_processing_padding, 0)
        points = self.get_points()
        clusters = PositionProcessor.cluster_maker(points, min_processing_z, self.max_z)
        valid_points = []
        for cluster in clusters:
            if len(cluster) >= self.min_cluster_size:
                valid_points.extend(cluster)
        self._set_points(np.array(valid_points))
        return self #allows chained operations


    @staticmethod
    def compress_z_points(points):
        """
        Compress the provided points into a 2D plane so that all points have a z of 0, so that if a point for an x, y point
        exists at any z, that position is considered occupied, so one point at that x, y and z = 0 exists

        Args:
            points: the provided points
        """
        points[:, 2] = 0
        return np.unique(points, axis=0)

    # turns points into clusters, where a cluster includes all points that are connected directly or by other points

    @staticmethod
    def cluster_maker(points, min_z=0,
                      max_z=LIDAR_Z_MAX):  # not that this will adjust z values to go up from zero if not already
        """
        Turn points into clusters, where each cluster includes all points that are connected directly or by other points

        Args:
            points: the provided points
            min_z: the minimum z on which the clustering is applied
            max_z: the maximum z on which the clustering is applied

        Returns:
            list: a list of the clusters
        """
        # min_z will act as offset
        z_size = max_z - min_z + 1  # +1 to include max_z
        grid = ThreeDBitArray(LIDAR_X_MAX, LIDAR_Y_MAX, z_size)
        for point in points:
            grid.set(True, point[0], point[1], point[2] - min_z)
        q = queue.Queue()
        clusters = []
        for point in points:
            if grid.get(point[0], point[1], point[2] - min_z):  ##might need comparator depending on format. Probs not
                q.put(np.array((point[0], point[1],
                                point[
                                    2] - min_z)))  # z of points inside q are now adjusted, don't need to be handled again
                grid.set(False, point[0], point[1], point[2] - min_z)
        return clusters


    @staticmethod
    def process_cluster(grid: ThreeDBitArray, q: queue.Queue):
        """
        Add all points connected to a given point to the cluster

        Args:
            grid (ThreeDBitArray): the array representing the occupancy of each point
            q (queue.Queue): a queue holding one point from the cluster to be created

        Returns:
            list: a list of points in the cluster
        """
        cluster = []
        while not q.empty():
            grid_point = q.get()
            cluster.append(grid_point)
            neighbours = grid.get_neighbours_np(grid_point)
            for neighbour in neighbours:
                grid.set(False, neighbour[0], neighbour[1], neighbour[2])
                q.put(neighbour)
        return cluster


    def get_yaw(self, degrees = True):
        """
        Get the yaw of the dog

        Args:
            degrees (bool): whether the result should be returned in degrees, else radians

        Returns:
            float: the yaw of the dog
        """
        if degrees:
            return self.yaw
        else:
            return PositionProcessor.get_radians_from_degrees(self.yaw)



    def get_relative_coordinates(self):
        """
        Get the position of the dog within the lidar frame

        Returns:
            the position of the dog within the lidar frame
        """
        return self.relative_coordinates


    def get_position_from(self, point):
        """
        Get the distance of the dog from a point

        Args:
            point: the coordinates of the point

        Returns:
            the distance of the dog from the point
        """
        return self.absolute_coordinates - point


    def get_absolute_coordinates(self):
        """
        Get the absolute coordinates of the dog from the origin point

        Returns:
            the absolute coordinates of the dog from the origin point
        """
        return self.absolute_coordinates


    @staticmethod
    def get_radians_from_degrees(degrees):
        """
        Convert degrees to radians

        Args:
            degrees (float): the degrees to convert

        Returns:
            float: the equivalent radians
        """
        return float(degrees) * np.pi / 180


    @staticmethod
    def get_orientation_from_message(message):
        """
            Get the orientation quaternion from the dog's pose message

            Args:
                the message containing the orientation quaternion

            Returns:
                the orientation quaternion from the dog's pose message
        """
        orientation = message["data"]["pose"]["orientation"]
        return orientation["x"], orientation["y"], orientation["z"], orientation["w"]


    @staticmethod
    def get_position_from_message(message):
        """
            Get the position of the dog from the dog's pose message

            Args:
                the message containing the dog's position

            Returns:
                the position of the dog from the dog's pose message
        """
        position = message["data"]["pose"]["position"]
        return np.array((position["x"], position["y"], position["z"]), )


    @staticmethod
    def get_state_from_quaternion(x, y, z, w):
        """
        Get roll, pitch and yaw frame an orientation quaternion

        Args:
            x: the x of the quaternion
            y: the y of the quaternion
            z: the z of the quaternion
            w: the w of the quaternion

        Returns:
            the pitch, roll and yaw from the quaternion
        """
        r = R.from_quat([x, y, z, w])
        return r.as_euler('xyz', degrees=True)  # returns roll, pitch, yaw


    @staticmethod
    def get_dog_relative_coordinates(position, origin):
        """
        Get the relative coordinates of a point from the origin point

        Args:
            position: the position of the point
            origin: the origin
        Return:
            the relative coordinates of the point from the origin point
        """
        global RESOLUTION
        coordinates = np.floor((position - origin) / RESOLUTION).astype(
            int)
        return coordinates


    def get_yaw_full_rotation(self, degrees = True, reverse_direction = True):
        """
        Covert the yaw from range -half rotation to + half rotation to range 0 to full rotation

        Args:
            degrees: whether the result is in degrees, else radians
            reverse_direction: whether the direction of the rotation is reversed

        Returns:
            the yaw from the range 0 to full rotation
        """
        if degrees:
            full_rotation = 360
        else:
            full_rotation = 2 * np.pi
        unscaled_yaw = self.get_yaw(degrees)
        if reverse_direction:
            unscaled_yaw *= -1
        if unscaled_yaw < 0:
            unscaled_yaw = unscaled_yaw + full_rotation
        return unscaled_yaw



    def voxel_to_2d_raw(self, scan_count):
        """
        Get simulated 2d raw lidar data calculated using the current points

        Args:
            scan_count: the number of directions in which a scan occurs per full rotation, with each direction being equally
            spread apart

        Returns:
            the simulated raw lidar data from the current points
        """
        points_relative = PositionProcessor.compress_z_points(self.points - self.relative_coordinates) #this should center the points around the dog

        distances = np.linalg.norm(points_relative, axis=1) #gets the distance from scanner for each point
        azimuths = np.arctan2(points_relative[:, 1], points_relative[:, 0])  # angle in radians of each point

        full_rotation = np.pi * 2

        unscaled_yaw = self.get_yaw_full_rotation(False) # gets the yaw from 0 - 360 going clockwise

        azimuths -= unscaled_yaw #this adjusts azimuths so that 0 is forward relative to the dog
        azimuths %= full_rotation # clips to a full rotation

        scan_distances = np.full((scan_count,), 1e6) # this will hold the scan distances for each small rotation of scanner
        divide_value = float(scan_count) / full_rotation
        scan_indices = np.floor((azimuths * divide_value)) % scan_count# make azimuth angles as a fraction of scan count rather than 2 pi
        stacked = np.column_stack((distances, scan_indices)) # combines distances and scan_indices
        arr_sorted = stacked[np.lexsort((stacked[:,0], stacked[:,1]))] # sorts indices ascending, then distances ascending
        _, unique_indices = np.unique(arr_sorted[:, 1], return_index=True) # gives the first index of each occurence of the scan index, which has the lowest distance for that index
        scan_distances[arr_sorted[unique_indices, 1].astype(int)] = arr_sorted[unique_indices, 0] #for each of the indices with the lowest distance, puts the distance associated into scan distances
        return scan_distances






class TwoDBitGrid:
    """
    A 2D expandable grid
    """

    def __init__(self, negative_x = -10, positive_x = 10, negative_y = -10, positive_y = 10):
        """
        Initialize the 2D grid

        Args:
            negative_x: the starting x size of the grid below 0
            positive_x: the starting x size of the grid above 0
            negative_y: the starting y size of the grid below 0
            positive_y: the starting y size of the grid above 0
        """
        self.grid = []
        self.x_size = 0
        self.y_size = 0
        self.x_offset = 0
        self.y_offset = 0
        self.resize_scale = 1.5
        if negative_x < 0:
            self.resize_grid(True, True, abs(negative_x))
        if positive_x > 0:
            self.resize_grid(True, False, abs(positive_x - negative_x + 1))
        if negative_y < 0:
            self.resize_grid(False, True, abs(negative_y))
        if positive_y > 0:
            self.resize_grid(False, False, abs(positive_y - negative_y + 1))


    def set(self, x, y, value=True):
        """
        Set the value of the point at the provided coordinates

        Args:
            x (int): the x coordinate of the point
            y (int): the y coordinate of the point
            value: the value for the point to be set
        """
        offset_x = int(x + self.x_offset)
        offset_y = int(y + self.y_offset)
        if not self.within_range(offset_x, offset_y):
            if not value:
                return
            self.check_point(x, y)
            offset_x = int(x + self.x_offset)
            offset_y = int(y + self.y_offset)
        # print(f"setting actual index {offset_x}, {offset_y}")
        self.grid[offset_x][offset_y] = value


    def get(self, x, y):
        """
        Get the value of the point at the provided coordinates

        Args:
            x (int): the x coordinate of the point
            y (int): the y coordinate of the point

        Returns:
            the value of the point at the provided coordinates
        """
        offset_x = int(x + self.x_offset)
        offset_y = int(y + self.y_offset)
        # print(f"getting actual index {offset_x}, {offset_y}")
        if self.within_range(offset_x, offset_y):
            return self.grid[offset_x][offset_y]
        # print("not in range")
        return 0


    def get_index(self, i, j):
        """
        Get the value of the point at the provided indices

        Args:
            i (int): the outer coordinates of the point
            j (int): the inner coordinates of the point
        """
        return self.grid[i][j]


    def set_index(self, i, j, value):
        """
        Set the value of the point at the provided indices

        Args:
            i (int): the outer coordinates of the point
            j (int): the inner coordinates of the point
            value : the value for the point to be set to
        """
        self.grid[i][j] = value



    def set_batch(self, batch: np.ndarray, value=True):
        """
        Set a batch of coordinates to a specified value

        Args:
            batch (np.ndarray): the batch of coordinates to be set
            value: the value for the points to be set to
        """
        min_per_column = np.min(batch, axis=0)
        max_per_column = np.max(batch, axis=0)
        self.check_point(min_per_column[0], min_per_column[1])
        self.check_point(max_per_column[0], max_per_column[1])
        for position in batch:
            self.grid[int(position[0] + self.x_offset)][int(position[1] + self.y_offset)] = value
            #print(self.grid[int(position[0] + self.x_offset)][int(position[1] + self.y_offset)])



    def get_batch(self, origin, x_size, y_size):
        """
        Get a copy of a specific section of the grid:

        Args:
            origin: the coordinates of the bottom left corner of the grid copy
            x_size (int): the size of the x grid copy
            y_size (int): the size of the y grid copy

        Returns:
            TwoDBitGrid: the copied section of the grid
        """
        # print(x_size, y_size)
        batch = TwoDBitGrid(0, x_size, 0, y_size)
        batch.x_offset = -origin[0]
        batch.y_offset = -origin[1] #ensures that points in the copy are at the same coordinates as they are in the original
        # print(batch.x_size, batch,y_size)
        # print(x_size, y_size)
        # print(batch.shape())
        for i in range(x_size):
            for j in range(y_size):
                batch.set_index(i, j, self.get(origin[0] + i, origin[1] + j))
        return batch


    def within_range(self, offset_x, offset_y):
        """
        Determine whether a set of indices are within the limits of the current state of the grid

        Args:
            offset_x: the x value after the offset has been applied
            offset_y: the y value after the offset has been applied

        Returns:
            bool: True if the indices are within the current limits, else False

        """
        return (self.x_size > offset_x >= 0) and (self.y_size > offset_y >= 0)


    def check_point(self, x, y):
        """
        Check whether a point is within the current limits of the grid, and if not, appropriately expand the grid

        Args:
            x (int): the x coordinate of the point
            y (int): the y coordinate of the point
        """
        if x >= self.x_size - self.x_offset:
            self.resize_grid(True, False, int((x + self.x_offset + 1) * self.resize_scale) + 1)
        elif x < (-1 * self.x_offset):
            self.resize_grid(True, True, int((-1 * self.x_offset - x + self.x_size) * self.resize_scale) + 1)
        if y >= self.y_size - self.y_offset:
            self.resize_grid(False, False, int((y + self.y_offset + 1) * self.resize_scale))
        elif y < (-1 * self.y_offset):
            self.resize_grid(False, True, int((-1 * self.y_offset - y + self.y_size) * self.resize_scale) + 1)


    def resize_grid(self, resize_x: bool, negative: bool, new_size: int):
        """
        Resize the grid

        Args:
            resize_x (bool): whether the x size should be resized
            negative (bool): whether the grid should be expanded in the negative reaction
            new_size (int): the new size of the dimension
        """
        # print(resize_x, negative, new_size)
        if resize_x:
            if new_size <= self.x_size:
                raise ValueError("new_size must be bigger than x_size")
            new_section = [bitarray(self.y_size) for _ in range(new_size - self.x_size)]
            for column in new_section:
                column.setall(0)
            if negative:
                new_section.extend(self.grid)
                self.grid = new_section
                self.x_offset += new_size - self.x_size
            else:
                self.grid.extend(new_section)
            self.x_size = new_size
        else:
            if new_size <= self.y_size:
                raise ValueError("new_size must be bigger than y_size")
            for i in range(self.x_size):
                column = self.grid[i]
                new_column = bitarray(new_size)
                new_column.setall(0)
                difference = 0
                if negative:
                    difference = new_size - self.y_size
                for j in range(len(column)):
                    new_column[j + difference] = column[j]
                self.grid[i] = new_column
            if negative:
                self.y_offset += new_size - self.y_size
            self.y_size = new_size


    def shape(self):
        """
        Get the shape of the grid

        Returns:
            tuple: the shape of the grid in format (x_size, y_size)
        """
        return self.x_size, self.y_size


class TwoDBitGridHolder:
    """
    A holder for a TwoDBitGrid that doesn't allow modification
    """


    def __init__(self, grid: TwoDBitGrid):
        """
        Instantiates TwoDBitGridHolder

        Args:
            grid (TwoDBitGrid): the TwoDBitGrid the holder holds
        """
        self.grid = grid

    def get(self, x, y):
        """
        Get the value of the grid at position (x, y)
        Args:
            x: the x position
            y: the y position

        Returns:
            the value of the grid at position (x, y)
        """
        return self.grid.get(x, y)

    def get_index(self, i, j):
        """
        Get the value of the grid at index (i, j)
        Args:
            i: the x index
            j: the y index

        Returns:
            the value of the grid at index (i, j)
        """
        return self.grid.get_index(i, j)

    def shape(self):
        """
        Get the shape of the grid

        Returns:
            tuple: the shape of the grid in format (x_size, y_size)
        """
        return self.grid.shape()

class MovingPoint:
    """
    A moving point, that holds a series of possible movements that can be completed
    """
    def __init__(self, start_position: np.ndarray, movements: np.ndarray, active = True):
        """
        Instantiates MovingPoint
        Args:
            start_position: the starting position of the moving point
            movements: the movements that the moving point can make
            active (bool): whether the moving point is currently active
        """
        # print("starting pos")
        # print(start_position)
        self.position = start_position.astype(int)
        self.movements = movements.astype(int)
        self.active = active


    def move(self, movement_index):
        """
        Move the moving point by the specified movement
        Args:
            movement_index: the index of the movement to be executed
        """
        if movement_index < 0 or movement_index >= len(self.movements):
            return
        self.position += self.movements[movement_index]

    def set_active(self, activation):
        """
        Set the activation of the moving point

        Args:
            activation (bool): whether the moving point is currently active
        """
        self.active = activation

    def is_active(self):
        """
        Determine whether the moving point is currently active
        Returns:
            bool: the active status of the point
        """
        return self.active

    def get_position(self):
        """
        Get the position of the moving point
        Returns:
            the position of the moving point
        """
        return self.position

    def set_position(self, position):
        """
        Set the position of the moving point
        Args:
            position: the position for the moving point to be set to
        """
        self.position = position


class PointAccumulator:
    """
    Accumulates points to develop a global map
    """
    FRAME_SIZE = 128
    def __init__(self, resolution=0.05):
        """
        Instantiates PointAccumulator
        Args:
            resolution: the resolution of the map and the lidar points
        """
        self.grid = TwoDBitGrid()
        self.relative_offset = None
        self.resolution = resolution



    def add_points(self, points, origin, scanner_position):  # assumes that frame is 2d
        """
        Add points to the accumulator grid
        Args:
            points: the points to be added
            origin: the origin provided by the lidar data
            scanner_position: the position of the lidar scanner
        """
        origin_coordinates = np.floor(origin / self.resolution).astype(int)
        scanner_coordinates = np.floor(scanner_position).astype(int)
        print("coordinates")
        if self.relative_offset is None:
            self.relative_offset = origin_coordinates
        corner_coordinates = origin_coordinates - self.relative_offset
        corrected_scanner_coordinates = scanner_coordinates- self.relative_offset
        points += corner_coordinates
        self.grid.set_batch(points, False)
        remaining_batch = self.grid.get_batch(corner_coordinates, PointAccumulator.FRAME_SIZE, PointAccumulator.FRAME_SIZE)
        self.grid.set_batch(points, True)
        self.check_point_differentials(remaining_batch, corner_coordinates, corrected_scanner_coordinates)


    @staticmethod
    def within_frame(coordinates, corner_coordinates):
        """
        Checks whether given coordinates are within the limits of a lidar frame
        Args:
            coordinates: the coordinates to be checked
            corner_coordinates: the coordinates of the bottom left corner of the frame

        Returns:
            bool: whether the coordinates are within the limits of the frame
        """
        # print("within frame")
        # print(coordinates, corner_coordinates)
        return corner_coordinates[0] <= coordinates[0] < (corner_coordinates[0] + PointAccumulator.FRAME_SIZE) and \
            (corner_coordinates[1] <= coordinates[1] < corner_coordinates[1] + PointAccumulator.FRAME_SIZE)

    def check_point_differentials(self, remaining_batch, corner_coordinates, scanner_coordinates):
        """
        Determines which points don't exist in the newest lidar frame but appear in the global map, and only keeps them
        if they are blocked by another point
        Args:
            remaining_batch: a batch containing only points that existed within the global map, but not the newest lidar
             frame
            corner_coordinates: the coordinates of the bottom left corner of the frame
            scanner_coordinates: the coordinates of the lidar scanner
        """
        top_left = MovingPoint(np.array([scanner_coordinates[0], scanner_coordinates[1]]), np.array([(-1, 1), (1, 0)]))
        top_right = MovingPoint(np.array([scanner_coordinates[0], scanner_coordinates[1]]), np.array([(1, 1), (0, -1)]))
        bottom_right = MovingPoint(np.array([scanner_coordinates[0], scanner_coordinates[1]]), np.array([(1, -1), (-1, 0)]))
        bottom_left = MovingPoint(np.array([scanner_coordinates[0], scanner_coordinates[1]]), np.array([(-1, -1), (0, 1)]))
        moving_points = (top_left, top_right, bottom_right, bottom_left)
        movements = 2
        while top_left.is_active() or top_right.is_active() or bottom_right.is_active() or bottom_left.is_active():
            for point in moving_points:
                point.move(0)
                if not point.is_active():
                    continue
                if not PointAccumulator.within_frame(point.position, corner_coordinates):
                    print("triggered on " + str(movements))
                    point.set_active(False)
                    continue
                if remaining_batch.get(point.position[0], point.position[1]):  #neaten
                    self.grid.set(int(point.position[0]), int(point.position[1]), (not (PointAccumulator.check_if_blocked(self.grid, point.position, scanner_coordinates)) is None))
                start_position = point.get_position()
                for movement in range(movements):
                    print(not (PointAccumulator.check_if_blocked(self.grid, point.position, scanner_coordinates)) is None)
                    point.move(1)
                    if remaining_batch.get(point.position[0], point.position[1]):
                        self.grid.set(int(point.position[0]), int(point.position[1]),
                                      not (PointAccumulator.check_if_blocked(self.grid, point.position, scanner_coordinates)) is None)
                point.set_position(start_position)
            movements += 2


    @staticmethod
    def check_if_blocked(grid, point, scanner):
        """
        Check if a point's line of sight to the scanner is blocked by another point
        Args:
            grid: the grid containing the points
            point: the point to check
            scanner: the position of the lidar scanner

        Returns:
            The position of the blocking point if the point is blocked, else, None
        """
        x0, y0 = map(int, np.rint(scanner[:2]))
        x1, y1 = map(int, np.rint(point[:2]))

        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            # Stop *before* reaching the final cell (the target point)
            if (x0, y0) != (x1, y1) and grid.get(x0, y0):
                print(f"{point} blocked by {x0, y0}")
                return x0, y0  # return the blocking cell coordinates

            if (x0, y0) == (x1, y1):
                break

            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

        return None  # no blockage found


    def get_map(self):
        """
        Get the accumulator map
        Returns:
            the accumulator map
        """
        return TwoDBitGridHolder(self.grid)

    def get_relative_offset(self):
        """
        Get the relative offset of the map
        Returns:
            the relative offset of the map
        """
        return self.relative_offset





class Controller:
    """
    Allows for the Go2 dog's movement to be controlled programatically
    """
    def __init__(self):
        print("trying controller get connection")
        self.connection = Controller.connection_q.get()
        print(type(self.connection))
        try:
            Controller.connection_q.put(self.connection)
        except:
            pass
        print("controller got connection")
        self.take_inputs = False
        self.listen = False
        self.movement_queue = queue.Queue()
        self.done_movement = queue.Queue(maxsize=1)

    connection_q = queue.Queue(maxsize=1)
    controller = None
    MOVEMENT_MAGNITUDE = 2.0
    MOVEMENT_DELAY = 2.0
    ROTATION_MAGNITUDE = 1
    ROTATION_DELAY = 2.0

    async def listen_queue(self):
        """
        Listen for movement events
        """
        self.listen = True
        while self.listen:
            await self.move(*self.movement_queue.get())
            if self.movement_queue.empty():
                self.done_movement.put(True)

    def stop_listen(self):
        """
        Stop listening for movement events
        """
        self.listen = False

    def put_movement(self, movement, blocking= False):
        """
        Put a movement into the movement queue
        Args:
            movement (Movement): the movement put into the queue
            blocking (bool): whether to block until movements are complete
        """
        self.movement_queue.put(movement)
        if blocking:
            self.done_movement.get()



    async def start_inputs(self):
        """
        Listen for keyboard inputs and trigger corresponding movements
        """
        # print("Loop ID start inputs:", id(asyncio.get_running_loop()))
        self.take_inputs = True
        while self.take_inputs:
            movement = [0.0, 0.0, 0.0]
            if keyboard.is_pressed("w"): movement[0] += self.MOVEMENT_MAGNITUDE
            if keyboard.is_pressed("a"): movement[1] += self.MOVEMENT_MAGNITUDE
            if keyboard.is_pressed("s"): movement[0] -= self.MOVEMENT_MAGNITUDE
            if keyboard.is_pressed("d"): movement[1] -= self.MOVEMENT_MAGNITUDE
            if keyboard.is_pressed("q"): movement[2] += self.ROTATION_MAGNITUDE
            if keyboard.is_pressed("e"): movement[2] -= self.ROTATION_MAGNITUDE
            if any(m != 0 for m in movement):
                await self.move(*movement) #could be made to use the movement queue so no async required
            else:
                await asyncio.sleep(0.1)

    def end_inputs(self):
        """
        Stop listening for keyboard inputs and triggering corresponding movements
        """
        self.take_inputs = False



    async def move(self, x, y, z, delay = None):
        """
        Executes the specified movement
        Args:
            x: the forwards magnitude of the movement
            y: the sideways magnitude of the movement
            z: the rotation of the movement
            delay: the sellp delay after the movement

        Returns:

        """
        # print(type(self.connection))
        # print("Loop ID move:", id(asyncio.get_running_loop()))
        # print("moving")
        if delay is None:
            delay = Controller.get_default_delay(x, y, z)
        await self.connection.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["SPORT_MOD"],
            {
                "api_id": SPORT_CMD["Move"],
                "parameter": {"x": x, "y": y, "z": z}
            }
        )
        await asyncio.sleep(delay)

    async def set_sports_mode(self):
        # print("Loop ID set sports mode:", id(asyncio.get_running_loop()))
        # print(type(self.connection))
        """
        Puts the dog into sports mode
        """
        response = await self.connection.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["MOTION_SWITCHER"],
            {"api_id": 1001}
        )
        if response['data']['header']['status']['code'] == 0:
            data = json.loads(response['data']['data'])
            current_motion_switcher_mode = data['name']
            print(f"Current motion mode: {current_motion_switcher_mode}")
        else:
            raise ValueError("error code received from dog")

        # Switch to "normal" mode if not already
        if current_motion_switcher_mode != "normal":
            print(f"Switching motion mode from {current_motion_switcher_mode} to 'normal'...")
            await self.connection.datachannel.pub_sub.publish_request_new(
                RTC_TOPIC["MOTION_SWITCHER"],
                {
                    "api_id": 1002,
                    "parameter": {"name": "normal"}
                }
            )
            await asyncio.sleep(5)  # Wait while it stands up

        # Perform "initialising"
        #  will not move on first command, needs placeholder command to initialise movement
        print("Initialising movement")
        await self.connection.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["SPORT_MOD"],
            {
                "api_id": SPORT_CMD["Move"],
                "parameter": {"x": 1.2, "y": 0, "z": 0}
            }
        )

        await asyncio.sleep(2)

    @staticmethod
    def get_default_delay(x, y, z):
        """
        Get the default delay for the movement
        Args:
            x: the forwards magnitude of the movement
            y: the sideways magnitude of the movement
            z: the rotation of the movement

        Returns:
            The default delay for the movement
        """
        if z == 0:
            delay = Controller.MOVEMENT_DELAY
        elif x == 0 and y == 0:
            delay = Controller.ROTATION_DELAY
        else:
            delay = max(Controller.MOVEMENT_DELAY, Controller.ROTATION_DELAY)
        return delay



    @staticmethod
    def put_connection(connection):
        """
        Provides the controller with a connection
        Args:
            connection: the connection to the dog
        """
        Controller.connection_q.put(connection)

    @staticmethod
    def set_controller(controller):
        """
        Set the controller object held in the static variable
        Args:
            controller: the controller held in the static variable
        """
        Controller.controller = controller

    @staticmethod
    def get_controller():
        """
        Get the controller object held in the static variable
        Returns:
            the controller held in the static variable
        """
        return Controller.controller

def ang_wrap(a: float, deg = True) -> float:
    """
    Wrap an angle to be between - half a full rotation and + half a full rotation
    Args:
        a: the angle to wrap
        deg: whether the angle and result is in degrees, else radians

    Returns:
        the wrapped angle
    """
    # wrap angle to [-180, 180)
    if deg:
        half = 180
    else:
        half = np.pi
    return (a + half) % (2 * half) - half

class Movement:
    """
    Represents a movement
    """
    #Only allows for forward and rotation, not sideways. Better off just storing x, y and z values if want sideways
    def __init__(self, walk, magnitude):
        """
        Initialize the movement
        Args:
            walk: whether the movement is walking, else rotating
            magnitude:
        """
        self.walk = walk
        self.magnitude = magnitude


#experimental
class PrecisionMover:
    """Makes precise movements"""
    Current_Mover = None
    MAX_ROTATION = 90
    MAX_MOVEMENT = 2

    def __init__(self,epsilon_degrees = 3, epsilon_distance = 0.1, rotation_tries = 3, movement_tries = 3):
        """
        Initialize PrecisionMover
        Args:
            epsilon_degrees: the acceptable level of inaccuracy in degrees
            epsilon_distance: the acceptable level of inaccuracy in meters
            rotation_tries: the number of retries to attempt to achieve acceptable rotation accuracy
            movement_tries: the number of retries to attempt to achieve acceptable movement accuracy
        """
        self.controller = Controller()
        self.position_processor = PositionProcessor()
        self.epsilon_radians = PositionProcessor.get_radians_from_degrees(epsilon_degrees)
        self.epsilon_distance = epsilon_distance
        self.rotation_tries = rotation_tries
        self.movement_tries = movement_tries
        self.movement_queue = queue.Queue()
        self.listen = False
        PrecisionMover.Current_Mover = self

    @staticmethod
    def get_movement(value, movement: bool):
        """
        Provides the next movement, decrementing the provided movement magnitude by either itself, or the max
        movement / rotation value and returning both the decremented value and the magnitude of the movement to be made
        Args:
            value: the total remaining magnitude of the movement / rotation
            movement: whether the movement is walking, else rotating

        Returns:
            value: the decremented total remaining value
            magnitude: the magnitude of the movement / rotation
        """
        #the point of this is that the dog seems to not respond well to very large movements, so this attempts to
        #break the movements into smaller sections
        if movement:
            maximum = PrecisionMover.MAX_MOVEMENT
        else:
            maximum = PrecisionMover.MAX_ROTATION
        if value > maximum:
            magnitude = maximum
            value -= maximum
        elif value < -maximum:
            magnitude = -maximum
            value += maximum
        else:
            magnitude = value
            value = 0
        return value, magnitude


    async def move(self, x, y, z):
        """
        Executes the movement
        Args:
            x: the forwards magnitude of the movement
            y: the sideways magnitude of the movement
            z: the rotation of the movement
        """
        while not x == 0 and y == 0 and z == 0:
            x, x_movement = PrecisionMover.get_movement(x, True)
            y, y_movement = PrecisionMover.get_movement(y, True)
            z, z_movement = PrecisionMover.get_movement(z, False)
            await self.controller.move(x, y, z)


    async def precision_rotation(self, degrees, absolute_yaw: bool = False):
        """
        Executes a precise rotation
        Args:
            degrees: the degrees to be rotated
            absolute_yaw: whether the provided degrees signify that the dogs yaw should be moved to the provided degrees
            else the degrees indicate the rotation that should occur relative to the dog's current rotation
        """
        self.position_processor.update_message()
        if degrees > 180 or degrees < -180:
            raise ValueError("invalid rotation angle")
        radians = PositionProcessor.get_radians_from_degrees(degrees)
        start_yaw = self.position_processor.get_yaw(False)
        if absolute_yaw:
            end_yaw = radians
            difference = self.get_wrapped_angle_difference(start_yaw, end_yaw, False)
        else:
            difference = radians
            end_yaw = ang_wrap(start_yaw + difference, False)
        tries = 0
        while tries < self.rotation_tries and abs(difference) > self.epsilon_radians:
            await self.move(0.0, 0.0, difference)
            #might need to add sleep here
            self.position_processor.update_message()
            difference = PrecisionMover.get_wrapped_angle_difference(self.position_processor.get_yaw(False), end_yaw, False)
            tries += 1

    @staticmethod
    def get_wrapped_angle_difference(start, end, degrees=True):
        """
        Find the difference between two angles wrapped in the range - half rotation to + half rotation
        Args:
            start: the start angle
            end: the end angle
            degrees: whether the angles are in degrees, else radians

        Returns:
            the difference between the angles
        """
        if degrees:
            limit = 180
            full = 360
        else:
            limit = np.pi
            full = 2 * np.pi

        dif = (end - start + limit) % full - limit
        return dif

    async def precision_movement(self, magnitude: float, readjust_yaw: bool = False):
        """
        Executes a precise movement
        Args:
            magnitude: the magnitude of the forward movement
            readjust_yaw: whether the yaw should be readjusted to what it was before the movement took place once the
            movement is completed
        """
        self.position_processor.update_message()
        start = self.position_processor.get_absolute_coordinates()
        yaw_deg = self.position_processor.get_yaw(True)
        yaw_rad = self.position_processor.get_yaw(False)

        # Target endpoint
        direction = np.array([np.cos(yaw_rad), np.sin(yaw_rad), 0.0])
        target = start + direction * magnitude

        tries = 0
        next_movement = (magnitude, 0.0, 0.0)

        while tries < self.movement_tries:
            await self.move(*next_movement)
            self.position_processor.update_message()
            current = self.position_processor.get_absolute_coordinates()
            yaw_rad = self.position_processor.get_yaw(False)

            pos_error = target - current
            magnitude = np.linalg.norm(pos_error[:2])
            if abs(magnitude) < self.epsilon_distance:
                break
            direction = np.arctan2(pos_error[1], pos_error[0])
            wrapped_turn = PrecisionMover.get_wrapped_angle_difference(yaw_rad, direction, False)
            next_movement = (np.cos(wrapped_turn) * magnitude, np.sin(wrapped_turn) * magnitude, 0.0)
            tries += 1
        if readjust_yaw:
            await self.precision_rotation(yaw_deg, True) #this is correct, it takes degrees. Internally uses rads but is passed degrs

    async def execute_sequence(self, sequence: list[Movement], absolute_yaw: bool = False, readjust_yaw: bool = False):
        """
        Executes a sequence of movements

        Args:
            sequence: the sequence to be executed
            absolute_yaw: whether the provided degrees signify that the dogs yaw should be moved to the provided degrees
            readjust_yaw: whether the yaw should be readjusted to what it was before the movement took place once the
            movement is completed
        """
        for movement in sequence:
            await self.execute_movement(movement, absolute_yaw, readjust_yaw)



    async def execute_movement(self, movement: Movement, absolute_yaw: bool = False, readjust_yaw: bool = False):
        """
        Executes a singular movement

        Args:
            movement (Movement): the movement to be executed
            absolute_yaw: whether the provided degrees signify that the dogs yaw should be moved to the provided degrees
            readjust_yaw: whether the yaw should be readjusted to what it was before the movement took place once the
            movement is completed
        """
        if movement.walk:
            await self.precision_movement(movement.magnitude, readjust_yaw)
        else:
            await self.precision_rotation(movement.magnitude, absolute_yaw)

    async def listen_queue(self):
        """
        Listen and execute incoming movements
        """
        self.listen = True
        while self.listen:
            await self.execute_movement(self.movement_queue.get())

    def stop_listen(self):
        """
        Stop listening for incoming movements
        """
        self.listen = False


    def put_movement(self, movement: Movement):
        """
        Put a movement into the queue
        Args:
            movement: the movement to be put into the queue
        """
        self.movement_queue.put(movement)


#experimental and untested
class SimpleNavigator:
    """
    Navigates through a single direction closed in area
    """
    # This class essentially tries to find a point in the lidar frame to the left, in front of, and to the right of
    # the dog, uses a hitscan algorithm to figure out how far a solid object is in each of these directions, and
    # tries to navigate using this data

    MAX_GRADIENT = 1E5
    PADDING = 4
    MIN_GRID = 0 + PADDING
    MAX_GRID = 128 - PADDING
    SIDE_THRESHOLD = 20
    FRONT_THRESHOLD = 10
    ANGLE_THRESHOLD = np.pi / 18


    def __init__(self):
        self.processor = PositionProcessor()

    def get_points(self):
        """
        Get the furthest points to the left, in front of and to the right of the dog

        Returns:
            The points chosen that are furthest to the left, in front of and to the right of the dog
        """
        yaw = self.processor.get_yaw(False)
        gradients, intercepts, forward_direction = SimpleNavigator.get_gradients_intercepts_directions(yaw, self.processor.get_relative_coordinates())
        points = np.zeros((len(gradients), 2))
        for i in range(len(points)):
            points[i] = SimpleNavigator.find_point(gradients[i], intercepts[i], forward_direction[i])
        return points

    @staticmethod
    def get_point_and_neighbours(point):
        """
        Get a 3 by 3 square of points surrounding the provided point
        Args:
            point: the point provided

        Returns:
            A 3 by 3 square of points surrounding the provided point
        """
        points = np.zeros((9, 2))
        counter = 0
        for x in range(point[0] - 1, point[0] + 2):
            for y in range(point[1] - 1, point[1] + 2):
                points[counter][0] = x
                points[counter][1] = y
                counter += 1
        return points



    def hitscan_points(self, points, frame: TwoDBitGrid):
        """
        Performs a hitscan from the lidar sensor to the provided points and gives the distances to solid object between
        the sensor and the point that is closest to the sensor
        Args:
            points: the points furthest to the left, front and right of the dog to be scanned
            frame: the lidar frame

        Returns:
            the distances to the solid object between the sensor and the point that is closest to the sensor to the
            left, front and right of the dog
        """
        sensor_relative_position = self.processor.get_relative_coordinates()[:2]
        distances = np.zeros((3,))
        for i in range(len(points)):
            point_and_neighbours = SimpleNavigator.get_point_and_neighbours(points[i])
            closest_distance = np.inf
            for neighbour in point_and_neighbours:
                raytrace_point = PointAccumulator.check_if_blocked(frame, neighbour, sensor_relative_position)
                if raytrace_point is not None:
                    closest_distance = min(np.linalg.norm(sensor_relative_position - raytrace_point), closest_distance)
            distances[i] = closest_distance
        return distances


    def navigate(self):
        """
        Navigates through a single direction closed in area
        """
        while True:
            controller = Controller.get_controller()
            self.processor.update_message()
            frame_grid = TwoDBitGrid()
            frame_grid.set_batch(self.processor.get_points(), True)
            distances = self.hitscan_points(self.get_points(), frame_grid)
            previous_distances = distances
            movement = 1.0
            while not distances[1] < SimpleNavigator.FRONT_THRESHOLD:
                l_r_difference = distances[0] - distances[2]

                if l_r_difference > SimpleNavigator.SIDE_THRESHOLD or l_r_difference < -SimpleNavigator.SIDE_THRESHOLD:
                    prev_l_r_difference = previous_distances[0] - previous_distances[2]
                    l_r_change = (l_r_difference - prev_l_r_difference) * RESOLUTION
                    controller.put_movement((0.0, 0.0, np.arctan(l_r_change / max(0.01, movement))), True)
                    controller.put_movement((0.0, l_r_difference * RESOLUTION, 0.0), True)
                movement = min(float(distances[1]) * RESOLUTION, 2.0)
                controller.put_movement((movement, 0.0, 0.0), True)
                previous_distances = distances
                self.processor.update_message()
                frame_grid = TwoDBitGrid()
                frame_grid.set_batch(self.processor.get_points(), True)
                distances = self.hitscan_points(self.get_points(), frame_grid)
            if distances[0] < distances[2]:
                controller.put_movement((0, 0, -np.pi / 2), True)
            else:
                controller.put_movement((0, 0, np.pi / 2), True)




    @staticmethod
    def find_point(gradient, intercept, forwards):
        """
        finds the point closest to the edge of the lidar frame following the provided line and direction
        Args:
            gradient: the gradient of the provided line
            intercept: the y intercept of the provided line
            forwards: the direction of travel for the provided line

        Returns:
            the coordinates of the point in the format (x, y)
        """
        if forwards:
            x_val = SimpleNavigator.MAX_GRID
        else:
            x_val = SimpleNavigator.MIN_GRID
        y_val_edge = SimpleNavigator.get_y_val(x_val, gradient, intercept)
        if SimpleNavigator.MIN_GRID < y_val_edge < SimpleNavigator.MAX_GRID:
            return np.array([x_val, y_val_edge])
        upper = (gradient < 0) ^ forwards
        if upper:
            y_val = SimpleNavigator.MAX_GRID
        else:
            y_val = SimpleNavigator.MIN_GRID
        x_val_edge = SimpleNavigator.get_x_val(y_val, gradient, intercept)
        return np.array([x_val_edge, y_val])

    @staticmethod
    def get_y_val(x, gradient, intercept):
        """
        Get the y value of the line at a specific x value
        Args:
            x: the x value
            gradient: the gradient of the line
            intercept: the y intercept of the line

        Returns:
            The y value of the line at point x
        """
        return x * gradient + intercept

    @staticmethod
    def get_x_val(y, gradient, intercept):
        """
        Get the x value of the line at a specific y value
        Args:
            y: the y value
            gradient: the gradient of the line
            intercept: the y intercept of the line

        Returns:
            The x value of the line at point y
        """
        if gradient == 0:
            return 1E5 # if 0, then gradient very big
        return (y - intercept) / gradient



    @staticmethod
    def calculate_intercept(known_point, gradient):
        """
        Calculates the y intercept of a line
        Args:
            known_point: a known point on the line
            gradient: the gradient of the line

        Returns:
            the y intercept of the line
        """
        return known_point[1] - (known_point[0] * gradient)

    @staticmethod
    def get_gradients_intercepts_directions(dog_yaw, relative_sensor_coordinates):
        """
        Finds gradients, intercepts and which direction the line should be travelled for lines going to the left,
        straight ahead of and to the right of the dog's lidar sensor
        Args:
            dog_yaw: the current yaw of the dog
            relative_sensor_coordinates: the coordinates of the sensor within the lidar frame

        Returns:
            gradients: the gradients of the lines going left, forward and right of the dog's lidar sensor
            intercepts: the intercepts of the lines going left, forward and right of the dog's lidar sensor
            forward_direction: whether the lines going left, forward and right of the dog's lidar sensor should be
            traversed forwards or backwards
        """
        adjusted_yaw = ang_wrap(dog_yaw + np.pi / 2, False)# +90 to make 0 90 degrees clockwise of up
        straight_gradient = np.clip(np.tan(adjusted_yaw), -SimpleNavigator.MAX_GRADIENT, SimpleNavigator.MAX_GRADIENT)
        left_angle = ang_wrap(adjusted_yaw + (np.pi / 2), False)
        right_angle = ang_wrap(adjusted_yaw - (np.pi / 2), False)
        left_gradient = np.clip(np.tan(left_angle), -SimpleNavigator.MAX_GRADIENT, SimpleNavigator.MAX_GRADIENT)
        right_gradient = np.clip(np.tan(right_angle), -SimpleNavigator.MAX_GRADIENT, SimpleNavigator.MAX_GRADIENT)
        gradients = np.array([left_gradient, straight_gradient, right_gradient])
        intercepts = np.zeros([3,])
        for g in range(len(gradients)):
            intercepts[g] = SimpleNavigator.calculate_intercept(relative_sensor_coordinates, gradients[g])
        forward_direction = np.array([ang_wrap(dog_yaw + np.pi / 2, False) < 0, dog_yaw < 0, ang_wrap(dog_yaw - np.pi / 2, False)]) # determines whether travel forward or backwards across line
        return gradients, intercepts, forward_direction



async def lidar_webrtc_connection(lidar_holder, pose_holder):
    retry_attempts = 0
    connected = False
    conn = None
    while True:
        while retry_attempts < MAX_RETRY_ATTEMPTS and not connected:
            try:
                conn = Go2WebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip="192.168.12.1")  # WebRTC IP
                print("Loop ID get conn:", id(asyncio.get_running_loop()))

                # Connect to WebRTC
                logging.info("Connecting to WebRTC...")
                await conn.connect()
                logging.info("Connected to WebRTC.")
                connected = True
                retry_attempts = 0  # Reset retry attempts on successful connection

                # Disable traffic saving mode
                await conn.datachannel.disableTrafficSaving(True)
            except Exception as e:
                logging.error(e)
                exit(1)

        try:

            print("published on")
            conn.datachannel.pub_sub.publish_without_callback("rt/utlidar/switch", "on")

            async def pose_callback_task(message):
                #print("updating pose")
                pose_holder.put(message)

            async def lidar_callback_task(message):
                #print("updating_lidar")
                lidar_holder.put(message)

            # Subscribe to LIDAR voxel map messages
            conn.datachannel.pub_sub.subscribe(
                "rt/utlidar/voxel_map_compressed",
                lambda message: asyncio.create_task(lidar_callback_task(message))  #

            )

            conn.datachannel.pub_sub.subscribe(
                "rt/utlidar/robot_pose",
                lambda message: asyncio.create_task(pose_callback_task(message))
            )

            # Keep the connection active
            Controller.put_connection(conn)
            #await listen_controller()
            await asyncio.sleep(1000)

        except Exception as e:
            logging.error(f"An error occurred: {e}")
            logging.info(
                f"Reconnecting in {reconnect_interval} seconds... (Attempt {retry_attempts + 1}/{MAX_RETRY_ATTEMPTS})")
            try:
                print("peace out")
                await conn.disconnect()
            except Exception as e:
                logging.error(f"Error during disconnect: {e}")
            connected = False
            await asyncio.sleep(reconnect_interval)

async def listen_controller():
    controller = Controller()
    await controller.set_sports_mode()
    await controller.start_inputs()

async def listen_controller_queue():
    controller = Controller()
    await controller.listen_queue()

async def listen_movement_queue():
    precision = PrecisionMover()
    await precision.listen_queue()


#experimental
def start_pygame_viewer():
    """
    Start a pygame viewer that displays a global map and accumulates points
    """
    print("pygame")
    processor = PositionProcessor(15, 20)
    accumulator = PointAccumulator()
    grid = accumulator.get_map()
    pygame.init()
    screen = pygame.display.set_mode((800, 800))
    clock = pygame.time.Clock()

    entity = [250, 250]
    scale = 5
    camera_x, camera_y = 0, 0

    running = True
    while running:
        #print("loop")
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEWHEEL:
                if event.y > 0: scale = min(20.0, scale * 1.1)
                if event.y < 0: scale = max(1, scale / 1.1)

        keys = pygame.key.get_pressed()
        if keys[pygame.K_UP]: camera_y -= 5
        if keys[pygame.K_DOWN]: camera_y += 5
        if keys[pygame.K_LEFT]: camera_x -= 5
        if keys[pygame.K_RIGHT]: camera_x += 5

        processor.update_message()
        accumulator.add_points(processor.filter_point_zs(False).get_points(), np.array(processor.get_origin()), processor.get_absolute_coordinates())
        ent_pos = processor.get_position_from(
            accumulator.get_relative_offset())  # im thinking this will give the dog's position on the grid
        entity[0] = ent_pos[0]
        entity[1] = ent_pos[1]
        screen.fill((0, 0, 0))
        grid = accumulator.get_map()
        print (grid.shape())

        for x in range(grid.shape()[0]):
            for y in range(grid.shape()[1]):
                #print(grid.get_index(x, y))
                if grid.get_index(x, y):
                    color = (200, 200, 200)
                else:
                    color = (0, 0, 0)
                sx = (x - camera_x) * scale
                sy = (y - camera_y) * scale
                #print(sx, sy, camera_x, camera_y, scale)
                if 0 <= sx < 800 and 0 <= sy < 800:
                    pygame.draw.rect(screen, color, (sx, sy, scale, scale))
        pygame.draw.rect(screen, (255, 0, 0),
                         (float((entity[0] - camera_x) * scale), float((entity[1] - camera_y) * scale), scale, scale))

        pygame.display.flip()
        clock.tick(10)



def start_webrtc(lidar_holder, pose_holder):
    """Run WebRTC connection in a separate asyncio loop."""
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(lidar_webrtc_connection(lidar_holder, pose_holder))


async def launch_connection(lidar_holder, pose_holder):
    webrtc_thread = threading.Thread(target=start_webrtc, daemon=True, args=(lidar_holder, pose_holder))
    webrtc_thread.start()
    await asyncio.sleep(1000)


def main():
    lidar_holder = DataHolder()
    pose_holder = DataHolder()
    PositionProcessor.initialize(lidar_holder, pose_holder)
    # handling_data = threading.Thread(target=simple_navigator, daemon=True)
    # handling_data.start()
    viewer = threading.Thread(target=start_pygame_viewer, daemon=True)
    viewer.start()
    asyncio.run(launch_connection(lidar_holder, pose_holder))


if __name__ == "__main__":
    main()
