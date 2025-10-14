""" UniSC industry project team """
import queue
import threading
import time
from platform import processor

from numpy.version import full_version

from Clean_Lidar_Handler import PoseProcessor
from go2_webrtc_driver.constants import RTC_TOPIC, SPORT_CMD

""" 25/09/2025 """
""" Inspired from lidar_stream.py and plot_lidar_stream.py by @legion1581 and @MrRobotoW at The RoboVerse Discord """

import asyncio
import logging
import numpy as np
# import pyvista as pv
from bitarray import bitarray
from go2_webrtc_driver.webrtc_driver import Go2WebRTCConnection, WebRTCConnectionMethod
from scipy.spatial.transform import Rotation as R
import pygame

# will use less globals once clean up code
reconnect_interval = 5  # Time (seconds) before retrying connection
# Constants
MAX_RETRY_ATTEMPTS = 3
LIDAR_X_MAX = 129
LIDAR_Y_MAX = 129
LIDAR_Z_MAX = 39
RESOLUTION = 0.05
yaw_starting_seconds = -1
yaw_starting_nanoseconds = -1
starting_yaw = None
yaw_drift = None
YAW_DRIFT_CALCULATION_SECONDS = 20.0


# this is just 3d bit array used for later clustering functions
class ThreeDBitArray:
    def __init__(self, x_size: int, y_size: int, z_size: int):
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
        if not (0 <= x < self.x_size and 0 <= y < self.y_size and 0 <= z < self.z_size):
            raise IndexError("Index out of bounds")
        return int(x * self.y_z_prod + y * self.z_size + z)

    def get(self, x: int, y: int, z: int) -> int:
        return self.array[self._index(x, y, z)]

    def set(self, value: bool, x: int, y: int, z: int) -> None:
        self.array[self._index(x, y, z)] = value

    def shape(self) -> tuple:
        return self.x_size, self.y_size, self.z_size

    def get_neighbours(self, x: int, y: int, z: int):
        pos = np.array((x, y, z))
        return self.get_neighbours_np(pos)

    def get_neighbours_np(self, pos: np.ndarray):
        neighbour_positions = self.relative_neighbour_positions + pos
        valid_neighbours = []
        for neighbour in neighbour_positions:
            if 0 <= neighbour[0] < self.x_size and 0 <= neighbour[1] < self.y_size and 0 <= neighbour[
                2] < self.z_size and self.get(neighbour[0], neighbour[1], neighbour[2]):
                valid_neighbours.append(neighbour)
        return valid_neighbours


class DataHolder:
    def __init__(self):
        self.q = queue.Queue(maxsize=1)

    def get(self):  # blocking if empty
        return self.q.get()

    def put(self, message):
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
    def __init__(self, lidar_holder: DataHolder, pose_holder: DataHolder, min_z=0, max_z=38, z_processing_padding=2,
                 min_cluster_size=4):
        self.pose_holder = pose_holder
        self.lidar_holder = lidar_holder
        self.yaw = 0
        self.relative_position = None
        self.absolute_position = None
        self.min_z = min_z
        self.max_z = max_z
        self.z_processing_padding = z_processing_padding
        self.min_cluster_size = min_cluster_size
        self.origin = None
        self.points = None

    def update_message(self):
        message = self.lidar_holder.get()
        self.origin = message["data"].get("origin", [])
        positions = message["data"]["data"].get("positions", [])
        self.points = np.unique(np.array([positions[i:i + 3] for i in range(0, len(positions), 3)], dtype=np.float32),
                                axis=0)
        pose_message = self.pose_holder.get()
        origin = self.lidar_holder.get()["data"].get("origin", [])
        self.yaw = self.get_state_from_quaternion(*self.get_orientation_from_message(pose_message))[2]
        self.relative_position = self.get_dog_relative_coordinates(self.get_position_from_message(pose_message), origin)
        self.absolute_position = self.get_position_from_message(pose_message) / RESOLUTION


    def get_origin(self):
        return self.origin

    def get_points(self):
        return self.points

    def _set_points(self, points):
        self.points = points

    def compress_z(self):
        self.points = PositionProcessor.compress_z_points(self.points)



    def filter_point_zs(self, pre_processing=False):
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
        return self

    def enforce_minimum_cluster_size(self):
        min_processing_z = min(self.min_z - self.z_processing_padding, 0)
        points = self.get_points()
        clusters = PositionProcessor.cluster_maker(points, min_processing_z, self.max_z)
        valid_points = []
        for cluster in clusters:
            if len(cluster) >= self.min_cluster_size:
                valid_points.extend(cluster)
        self._set_points(np.array(valid_points))
        return self

    @staticmethod
    def compress_z_points(points): #this alters entered points, because in current use cases, do not require old points, and compy is inefficient
        points[:, 2] = 0
        return np.unique(points, axis=0)

    # turns points into clusters, where a cluster includes all points that are connected directly or by other points
    @staticmethod
    def cluster_maker(points, min_z=0,
                      max_z=LIDAR_Z_MAX):  # not that this will adjust z values to go up from zero if not already
        # min_z will act as offset
        print("start")
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
                clusters.append(PositionProcessor.process_cluster(grid, q))
        print("end")
        return clusters

    # goes through and forms an entire cluster starting from the point in the queue
    @staticmethod
    def process_cluster(grid: ThreeDBitArray, q: queue.Queue):
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
        if degrees:
            return self.yaw
        else:
            return PositionProcessor.get_radians_from_degrees(self.yaw)



    def get_relative_position(self):
        return self.relative_position

    def get_position_from(self, point):
        return self.absolute_position - point

    @staticmethod
    def get_radians_from_degrees(degrees):
        return degrees * np.pi / 180

    @staticmethod
    def get_orientation_from_message(message):
        orientation = message["data"]["pose"]["orientation"]
        return orientation["x"], orientation["y"], orientation["z"], orientation["w"]

    @staticmethod
    def get_position_from_message(message):
        position = message["data"]["pose"]["position"]
        return np.array((position["x"], position["y"], position["z"]), )

    @staticmethod
    def get_state_from_quaternion(x, y, z, w):
        r = R.from_quat([x, y, z, w])
        return r.as_euler('xyz', degrees=True)  # returns roll, pitch, yaw

    @staticmethod
    def get_dog_relative_coordinates(position, origin):
        global RESOLUTION
        coordinates = np.floor((position - origin) / RESOLUTION).astype(
            int)  # todo check if this works. Maybe are not all np
        return coordinates

    def get_yaw_full_rotation(self, degrees = True, reverse_direction = True):
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
        points_relative = PositionProcessor.compress_z_points(self.points - self.relative_position) #this should center the points around the dog

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
    def __init__(self):
        self.grid = []
        self.x_size = 0
        self.y_size = 0
        self.x_offset = 0
        self.y_offset = 0
        self.resize_scale = 1.5

    def set(self, x, y, value=True):
        offset_x = x + self.x_offset
        offset_y = y + self.y_offset
        if not self.within_range(offset_x, offset_y):
            if not value:
                return
            self.check_point(x, y)
            offset_x = int(x + self.x_offset)
            offset_y = int(y + self.y_offset)
        # print(f"setting actual index {offset_x}, {offset_y}")
        self.grid[offset_x][offset_y] = value

    def get(self, x, y):
        offset_x = x + self.x_offset
        offset_y = y + self.y_offset
        # print(f"getting actual index {offset_x}, {offset_y}")
        if self.within_range(offset_x, offset_y):
            return self.grid[offset_x][offset_y]
        # print("not in range")
        return 0

    def get_index(self, i, j):
        return self.grid[i][j]

    def set_batch(self, batch: np.ndarray, value=True):
        #print(batch.shape)
        min_per_column = np.min(batch, axis=0)
        #print(min_per_column)
        max_per_column = np.max(batch, axis=0)
        #print(max_per_column)
        self.check_point(min_per_column[0], min_per_column[1])
        self.check_point(max_per_column[0], max_per_column[1])
        for position in batch:
            self.grid[int(position[0] + self.x_offset)][int(position[1] + self.y_offset)] = value
            #print(self.grid[int(position[0] + self.x_offset)][int(position[1] + self.y_offset)])

    def within_range(self, offset_x, offset_y):
        return (self.x_size > offset_x >= 0) and (self.y_size > offset_y >= 0)

    def check_point(self, x, y):
        if x >= self.x_size - self.x_offset:
            self.resize_grid(True, False, int((x + self.x_offset + 1) * self.resize_scale) + 1)
        elif x < (-1 * self.x_offset):
            self.resize_grid(True, True, int((-1 * self.x_offset - x + self.x_size) * self.resize_scale) + 1)
        if y >= self.y_size - self.y_offset:
            self.resize_grid(False, False, int((y + self.y_offset + 1) * self.resize_scale))
        elif y < (-1 * self.y_offset):
            self.resize_grid(False, True, int((-1 * self.y_offset - y + self.y_size) * self.resize_scale) + 1)

    def resize_grid(self, resize_x: bool, negative: bool, new_size: int):
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
        return self.x_size, self.y_size


class TwoDBitGridHolder:
    def __init__(self, grid: TwoDBitGrid):
        self.grid = grid

    def get(self, x, y):
        return self.grid.get(x, y)

    def get_index(self, i, j):
        return self.grid.get_index(i, j)

    def shape(self):
        return self.grid.shape()


class PointAccumulator:  # im assuming origin is np.array. Cant remember if true
    def __init__(self, resolution=0.05):
        self.grid = TwoDBitGrid()
        self.relative_offset = None
        self.resolution = resolution

    def add_points(self, points, origin):  # assumes that frame is 2d

        origin_coordinates = np.floor(origin / self.resolution).astype(int)
        if self.relative_offset is None:
            self.relative_offset = origin_coordinates
        corner_coordinates = origin_coordinates - self.relative_offset
        points += corner_coordinates
        self.grid.set_batch(points, True)

    def get_map(self):
        return TwoDBitGridHolder(self.grid)

    def get_relative_offset(self):
        return self.relative_offset


async def lidar_webrtc_connection(lidar_holder, pose_holder):
    retry_attempts = 0
    connected = False
    conn = None
    while True:
        while retry_attempts < MAX_RETRY_ATTEMPTS and not connected:
            try:
                conn = Go2WebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip="192.168.12.1")  # WebRTC IP

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
            # Turn LIDAR sensor on
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
                # lambda message : asyncio.create_task(pass_callback_task(message))
                # lambda message: asyncio.create_task(update_origin(message))
            )

            conn.datachannel.pub_sub.subscribe(
                "rt/utlidar/robot_pose",
                lambda message: asyncio.create_task(pose_callback_task(message))
                # lambda message: asyncio.create_task(pass_callback_task(message))
                # lambda message: asyncio.create_task(print_callback_task((get_state_from_quaternion(*get_orientation_from_message(message))[2])))
                # lambda message : update_latest_position(get_position_from_message(message))
                # lambda message : asyncio.create_task(print_orientation_and_position(message))
            )

            # Keep the connection active
            while True:
                await asyncio.sleep(1)

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


# we will need to somehow calculate drift if the lidar frame does not drift also
# should calculate the amount of drift per second that the sensor has. Not 100% on how consistent the drift is, but if it is consistent, we could counteract it.
# Could be used in conjunction with resetting the connection periodically to reset the drift
def calculate_yaw_drift(message):
    global yaw_starting_seconds, yaw_starting_nanoseconds, yaw_drift, YAW_DRIFT_CALCULATION_SECONDS, starting_yaw
    current_seconds = message["header"]["stamp"]["sec"]
    if starting_yaw is None:
        yaw_starting_seconds = current_seconds
        yaw_starting_nanoseconds = message["header"]["stamp"]["nanosec"]
        starting_yaw = PoseProcessor.get_orientation_from_message(message)[2]
    elif current_seconds >= (
            yaw_starting_seconds + YAW_DRIFT_CALCULATION_SECONDS):  # this is accessible. IDE might say its not
        yaw_drift_calculated = True
        yaw_drift = double(get_orientation_from_message(message)[2] - starting_yaw) / YAW_DRIFT_CALCULATION_SECONDS


# prints 2d map on console
def print_map(values, scale_divisor=1, lowest_z=-1, highest_z=128):
    size = int((128 / scale_divisor) + 1)
    arr = np.zeros((size, size))
    for point in values:
        if lowest_z < point[2] < highest_z:
            arr[int(point[0] / scale_divisor)][int(point[1] / scale_divisor)] = 1
    for line in arr:
        print_string = ""
        for i in range(len(line)):
            if line[i] > 0:
                print_string += "X"
            else:
                print_string += " "
            # print_string += str(int(line[i]))
        print(print_string)


# gives points only within certain z range
def filter_point_zs(points, min_z, max_z):
    filtered_points = []
    for point in points:
        if min_z <= point[2] <= max_z:
            filtered_points.append(point)
    return np.array(filtered_points)


# gives static 3d image of lidar scan. Couldn't get it to dynamically update
def display_values(points, dog_position):
    cloud = pv.PolyData(points)
    plotter = pv.Plotter()
    plotter.add_points(cloud, color='green', point_size=5)
    plotter.add_points(np.array((dog_position,)), color='red', point_size=5)
    plotter.show()


def start_pygame_viewer(lidar_holder: DataHolder, pose_holder: DataHolder):
    print("pygame")
    processor = PositionProcessor(lidar_holder, pose_holder)
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
        print("loop")
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEWHEEL:
                if event.y > 0: scale = min(20, scale * 1.1)
                if event.y < 0: scale = max(1, scale / 1.1)

        keys = pygame.key.get_pressed()
        if keys[pygame.K_w]: camera_y -= 5
        if keys[pygame.K_s]: camera_y += 5
        if keys[pygame.K_a]: camera_x -= 5
        if keys[pygame.K_d]: camera_x += 5

        processor.update_message()
        accumulator.add_points(processor.filter_point_zs(False).get_points(), np.array(processor.get_origin()))
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
                    sx = (x - camera_x) * scale
                    sy = (y - camera_y) * scale
                    print(sx, sy, camera_x, camera_y, scale)
                    if 0 <= sx < 800 and 0 <= sy < 800:
                        print("drawn")
                        pygame.draw.rect(screen, (200, 200, 200), (sx, sy, scale, scale))
        pygame.draw.circle(screen, (255, 0, 0),
                           ((entity[0] - camera_x) * scale, (entity[1] - camera_y) * scale),
                           max(2.0, scale / 2))

        pygame.display.flip()
        clock.tick(1)


def start_webrtc(lidar_holder, pose_holder):
    """Run WebRTC connection in a separate asyncio loop."""
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(lidar_webrtc_connection(lidar_holder, pose_holder))


async def launch_connection(lidar_holder, pose_holder):
    webrtc_thread = threading.Thread(target=start_webrtc, daemon=True, args=(lidar_holder, pose_holder))
    webrtc_thread.start()
    await asyncio.sleep(1000)


async def handle_messages(lidar_holder, pose_holder):
    processor = PositionProcessor(lidar_holder, pose_holder)
    while True:
        processor.update_message()
        filtered_points = processor.filter_point_zs(True).enforce_minimum_cluster_size().filter_point_zs(False)
        display_values(filtered_points, processor.get_relative_position())


def main():
    lidar_holder = DataHolder()
    pose_holder = DataHolder()
    handling_data = threading.Thread(target=start_pygame_viewer, daemon=True, args=(lidar_holder, pose_holder))
    handling_data.start()
    asyncio.run(launch_connection(lidar_holder, pose_holder))


if __name__ == "__main__":
    main()
