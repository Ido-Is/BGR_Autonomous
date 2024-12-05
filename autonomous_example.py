import sys
import os
import time
import numpy
import math
import cv2  # OpenCV for displaying the camera feed
import matplotlib.pyplot as plt

# Adds the fsds package located in the parent directory to the Python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import fsds

# Connect to the simulator
client = fsds.FSDSClient()

# Check network connection, exit if not connected
client.confirmConnection()

# After enabling, set trajectory setpoints via the API.
client.enableApiControl(True)

# Autonomous system constants
max_throttle = 0.2  # m/s^2
target_speed = 4  # m/s
max_steering = 0.3
cones_range_cutoff = 7  # meters


# Function to calculate distance between two points
def distance(x1, y1, x2, y2):
    """Calculate Euclidean distance between two points."""
    return math.sqrt(math.pow(abs(x1 - x2), 2) + math.pow(abs(y1 - y2), 2))


# Function to calculate the center of a group of points
def pointgroup_to_cone(group):
    """Calculate the average x and y coordinates of a group of points."""
    average_x = 0
    average_y = 0
    for point in group:
        average_x += point['x']
        average_y += point['y']
    average_x /= len(group)
    average_y /= len(group)
    return {'x': average_x, 'y': average_y}


# Function to find cones based on lidar data
def find_cones():
    """Find cones based on Lidar data from the simulator."""
    # Get the point cloud from Lidar
    lidardata = client.getLidarData(lidar_name='Lidar')

    # Return an empty list if no points are found
    if len(lidardata.point_cloud) < 3:
        return []

    # Convert the list of floats into a list of xyz coordinates
    points = numpy.array(lidardata.point_cloud, dtype=numpy.dtype('f4'))
    points = numpy.reshape(points, (int(points.shape[0] / 3), 3))

    # Group points that are close together as cones
    current_group = []
    cones = []
    for i in range(1, len(points)):
        distance_to_last_point = distance(points[i][0], points[i][1], points[i - 1][0], points[i - 1][1])

        if distance_to_last_point < 0.1:
            current_group.append({'x': points[i][0], 'y': points[i][1]})
        else:
            if len(current_group) > 0:
                cone = pointgroup_to_cone(current_group)
                if distance(0, 0, cone['x'], cone['y']) < cones_range_cutoff:
                    cones.append(cone)
                current_group = []
    return cones


# Function to calculate steering angle based on cone positions
def calculate_steering(cones):
    """Calculate the steering angle based on the position of cones."""
    average_y = 0
    for cone in cones:
        average_y += cone['y']
    average_y /= len(cones)

    if average_y > 0:
        return -max_steering
    else:
        return max_steering


# Function to calculate throttle based on current speed
def calculate_throttle():
    """Calculate the throttle value based on the current vehicle speed."""
    gps = client.getGpsData()
    velocity = math.sqrt(math.pow(gps.gnss.velocity.x_val, 2) + math.pow(gps.gnss.velocity.y_val, 2))
    return max_throttle * max(1 - velocity / target_speed, 0)


# Main loop for vehicle control and displaying camera feed
while True:
    # Capture and display camera feed
    [image] = client.simGetImages(
        [fsds.ImageRequest("examplecam", fsds.ImageType.Scene, pixels_as_float=False, compress=True)],
        vehicle_name="FSCar")

    # Convert the image data to a format that OpenCV can use
    image_data = numpy.frombuffer(image.image_data_uint8, dtype=numpy.uint8)
    image_data = image_data.reshape((image.height, image.width, 3))
    image_data = cv2.cvtColor(image_data, cv2.COLOR_RGB2BGR)

    # Display the image using OpenCV
    cv2.imshow('Live Camera Feed', image_data)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break  # Exit the loop if 'q' is pressed

    # Process the cones for vehicle control
    cones = find_cones()
    if len(cones) == 0:
        continue  # Skip if no cones are found

    car_controls = fsds.CarControls()
    car_controls.steering = calculate_steering(cones)
    car_controls.throttle = calculate_throttle()
    car_controls.brake = 0
    client.setCarControls(car_controls)

    # Draw cones on the plot (optional)
    plt.clf()
    plt.axis([-cones_range_cutoff, cones_range_cutoff, -2, cones_range_cutoff])
    for cone in cones:
        plt.scatter(x=-1 * cone['y'], y=cone['x'])

    plt.pause(0.05)

# Close the OpenCV window when done
cv2.destroyAllWindows()
plt.show()  # Show the final plot after stopping the loop
