""" UniSC industry project team """
import queue
import threading

""" 25/09/2025 """
""" Inspired from lidar_stream.py and plot_lidar_stream.py by @legion1581 and @MrRobotoW at The RoboVerse Discord """


import asyncio
import logging
import numpy as np
import pyvista as pv
from bitarray import bitarray
from go2_webrtc_driver.webrtc_driver import Go2WebRTCConnection, WebRTCConnectionMethod
from scipy.spatial.transform import Rotation as R


#will use less globals once clean up code
reconnect_interval = 5  # Time (seconds) before retrying connection
# Constants
MAX_RETRY_ATTEMPTS = 3
counter = 0
LIDAR_X_MAX = 129
LIDAR_Y_MAX = 129
LIDAR_Z_MAX = 39
displaying = False
latest_position = (0, 0, 0)
latest_origin = (0, 0, 0)
RESOLUTION = 0.05
yaw_starting_seconds = -1
yaw_starting_nanoseconds = -1
starting_yaw = None
yaw_drift = None
YAW_DRIFT_CALCULATION_SECONDS = 20.0


#this is just 3d bit array used for later clustering functions
class ThreeDBitArray:
    def __init__(self, x_size: int, y_size: int, z_size: int):
        self.x_size = x_size
        self.y_size = y_size
        self.z_size = z_size
        self.y_z_prod = y_size * z_size
        self.array = bitarray(x_size * self.y_z_prod)
        self.array.setall(False)  # initialize to 0 (False)
        self.relative_neighbour_positions = np.array( #all relative neighbour positions
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







async def lidar_webrtc_connection():
    retry_attempts = 0
    connected = False
    conn = None
    while True:
        while retry_attempts < MAX_RETRY_ATTEMPTS and not connected:
            try:
                conn = Go2WebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip="192.168.12.1")  # WebRTC IP
                # _webrtc_connection = Go2WebRTCConnection(WebRTCConnectionMethod.Remote, serialNumber="B42D2000XXXXXXXX", username="email@gmail.com", password="pass")
                # _webrtc_connection = Go2WebRTCConnection(WebRTCConnectionMethod.LocalAP)

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
            conn.datachannel.pub_sub.publish_without_callback("rt/utlidar/switch", "on")


            async def lidar_callback_task(message):
                """Task to process incoming LIDAR data."""

                positions = message["data"]["data"].get("positions", [])
                origin = message["data"].get("origin", [])
                points = np.array([positions[i:i + 3] for i in range(0, len(positions), 3)], dtype=np.float32)
                total_points = len(points)
                unique_points = np.unique(points, axis=0)
                global counter, displaying, latest_position
                counter += 1
                if counter == 5:
                    min_z = 14
                    min_z_for_processing = 12
                    max_z = 30
                    min_cluster_size = 20


                    #this section prints 2d map in console
                    # args = ((filter_point_zs(enforce_minimum_cluster_size(filter_point_zs(unique_points, min_z_for_processing, max_z), min_cluster_size, min_z_for_processing, max_z), min_z, max_z)),)
                    # t = threading.Thread(target=print_map, args=args)
                    # t.start()

                    #this section gives 3d model (not continuously updating)
                    #points_arg = filter_point_zs(enforce_minimum_cluster_size(filter_point_zs(unique_points, min_z_for_processing, max_z), min_cluster_size, min_z_for_processing, max_z), min_z, max_z) #this line removes clusters below certain size and creates model
                    points_arg = filter_point_zs(unique_points, 10, max_z) #no cluster filtering
                    args = (points_arg, get_dog_relative_coordinates(latest_position, origin))
                    if displaying: #if displaying, we skip
                        return
                    displaying = True
                    t = threading.Thread(target=display_values, args=args)
                    t.start()
                    counter = 0

            async def print_callback_task(message):
                print(type(message))
                print(message)

            async def pass_callback_task(message):
                pass


            # Subscribe to LIDAR voxel map messages
            conn.datachannel.pub_sub.subscribe(
                "rt/utlidar/voxel_map_compressed",
                lambda message: asyncio.create_task(lidar_callback_task(message)) #
                #lambda message : asyncio.create_task(pass_callback_task(message))
                #lambda message: asyncio.create_task(update_origin(message))
            )

            conn.datachannel.pub_sub.subscribe(
                "rt/utlidar/robot_pose",
                #lambda message: asyncio.create_task(pass_callback_task(message))
                #lambda message: asyncio.create_task(print_callback_task((get_state_from_quaternion(*get_orientation_from_message(message))[2])))
                #lambda message : update_latest_position(get_position_from_message(message))
                #lambda message : asyncio.create_task(print_orientation_and_position(message))
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


async def update_origin(message):
    global latest_origin
    latest_origin = message["data"].get("origin", [])

#gets the coordinates of the dog within the voxel frid
def get_dog_relative_coordinates(position, origin):
    global RESOLUTION
    coordinates = np.zeros(3)
    print(position)
    print(origin)
    for i in range(len(origin)):
        coordinates[i] = int((position[i] - origin[i]) / RESOLUTION)
    print(coordinates)
    return coordinates


#we will need to somehow calculate drift if the lidar frame does not drift also
#should calculate the amount of drift per second that the sensor has. Not 100% on how consistent the drift is, but if it is consistent, we could counteract it.
#Could be used in conjunction with resetting the connection periodically to reset the drift
def calculate_yaw_drift(message):
    global yaw_starting_seconds, yaw_starting_nanoseconds, yaw_drift, YAW_DRIFT_CALCULATION_SECONDS, starting_yaw
    current_seconds = message["header"]["stamp"]["sec"]
    if starting_yaw is None:
        yaw_starting_seconds = current_seconds
        yaw_starting_nanoseconds = message["header"]["stamp"]["nanosec"]
        starting_yaw = get_orientation_from_message(message)[2]
    elif current_seconds >= (yaw_starting_seconds + YAW_DRIFT_CALCULATION_SECONDS):  #this is accessible. IDE might say its not
        yaw_drift_calculated = True
        yaw_drift = double(get_orientation_from_message(message)[2] - starting_yaw) / YAW_DRIFT_CALCULATION_SECONDS




async def print_orientation_and_position(message):
    global latest_origin
    print(get_state_from_quaternion(*get_orientation_from_message(message))[2])
    print(get_position_from_message(message))
    print(latest_origin)



def get_orientation_from_message(message):
    orientation = message["data"]["pose"]["orientation"]
    return orientation["x"], orientation["y"], orientation["z"], orientation["w"]

def get_position_from_message(message):
    position = message["data"]["pose"]["position"]
    return position["x"], position["y"], position["z"]

def update_latest_position(position):
    global latest_position
    latest_position = position

#gets pitch, roll and yaw from a quaternion, which is what robot gives
def get_state_from_quaternion(x, y, z, w):
        r = R.from_quat([x, y, z, w])
        return r.as_euler('xyz', degrees=True)  # returns roll, pitch, yaw

def radians_to_degrees(radians):
    return radians * 180 / np.pi


#prints 2d map on console
def print_map(values, scale_divisor = 1, lowest_z = -1, highest_z = 128):
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


#gives points only within certain z range
def filter_point_zs(points, min_z, max_z):
    filtered_points = []
    for point in points:
        if min_z <= point[2] <= max_z:
            filtered_points.append(point)
    return np.array(filtered_points)





#min and max z must reflect the zs within points. This function does not filter
def enforce_minimum_cluster_size(points, min_cluster_size, min_z = 0, max_z = LIDAR_Z_MAX):
    clusters = cluster_maker(points, min_z, max_z)
    valid_points = []
    for cluster in clusters:
        if len(cluster) >= min_cluster_size:
            valid_points.extend(cluster)
    return np.array(valid_points)


# turns points into clusters, where a cluster includes all points that are connected directly or by other points
def cluster_maker(points, min_z = 0, max_z = LIDAR_Z_MAX): #not that this will adjust z values to go up from zero if not already
    #min_z will act as offset
    print("start")
    z_size = max_z - min_z + 1 # +1 to include max_z
    grid = ThreeDBitArray(LIDAR_X_MAX, LIDAR_Y_MAX, z_size)
    for point in points:
        grid.set(True, point[0], point[1], point[2] - min_z)
    q = queue.Queue()
    clusters = []
    for point in points:
        if grid.get(point[0], point[1], point[2] - min_z): ##might need comparator depending on format. Probs not
            q.put(np.array((point[0], point[1], point[2] - min_z))) # z of points inside q are now adjusted, don't need to be handled again
            grid.set(False, point[0], point[1], point[2] - min_z)
            clusters.append(process_cluster(grid, q))
    print("end")
    return clusters

# goes through and forms an entire cluster starting from the point in the queue
def process_cluster(grid : ThreeDBitArray, q : queue.Queue):
    cluster = []
    while not q.empty():
        grid_point = q.get()
        cluster.append(grid_point)
        neighbours = grid.get_neighbours_np(grid_point)
        for neighbour in neighbours:
            grid.set(False, neighbour[0], neighbour[1], neighbour[2])
            q.put(neighbour)
    return cluster





# gives static 3d image of lidar scan. Couldn't get it to dynamically update
def display_values(points, origin):
    global displaying
    print("executing this")
    cloud = pv.PolyData(points)
    plotter = pv.Plotter()
    plotter.add_points(cloud, color='green', point_size=5)
    plotter.add_points(np.array([origin]), color='red', point_size=5)
    plotter.show()
    displaying = False



def start_webrtc():
    """Run WebRTC connection in a separate asyncio loop."""
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(lidar_webrtc_connection())


async def main():
    webrtc_thread = threading.Thread(target=start_webrtc, daemon=True)
    webrtc_thread.start()
    await asyncio.sleep(1000)

if __name__ == "__main__":
    asyncio.run(main())