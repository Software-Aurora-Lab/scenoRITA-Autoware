import math
import random

from autoware_auto_perception_msgs.msg import PredictedObject
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, Quaternion
from shapely import LineString, Polygon
from datetime import datetime, timedelta

from std_msgs.msg import Header

from scenoRITA.representation import ObstacleType

# VEHICLE CONFIGS FOR AUTOWARE
AUTOWARE_VEHICLE_LENGTH = 4.77
AUTOWARE_VEHICLE_WIDTH = 1.83
AUTOWARE_VEHICLE_HEIGHT = 2.5
AUTOWARE_VEHICLE_back_edge_to_center = 1.030


def generate_adc_polygon(position: Point, theta: float):
    """
    Generate polygon for the ADC

    Parameters:
        position: Point
            localization pose of ADC
        theta: float
            heading of ADC

    Returns:
        points: List[Point]
            polygon points of the ADC
    """
    points = []
    half_w = AUTOWARE_VEHICLE_WIDTH / 2.0
    front_l = AUTOWARE_VEHICLE_LENGTH - AUTOWARE_VEHICLE_back_edge_to_center
    back_l = -1 * AUTOWARE_VEHICLE_back_edge_to_center
    sin_h = math.sin(theta)
    cos_h = math.cos(theta)
    vectors = [(front_l * cos_h - half_w * sin_h,
                front_l * sin_h + half_w * cos_h),
               (back_l * cos_h - half_w * sin_h,
                back_l * sin_h + half_w * cos_h),
               (back_l * cos_h + half_w * sin_h,
                back_l * sin_h - half_w * cos_h),
               (front_l * cos_h + half_w * sin_h,
                front_l * sin_h - half_w * cos_h)]
    for x, y in vectors:
        p = Point()
        p.x = position.x + x
        p.y = position.y + y
        p.z = 0.0
        points.append(p)
    return points


def generate_polygon(position: Point, theta: float, length: float, width: float):
    """
    Generate polygon for a perception obstacle

    Parameters:
        position: Point
            position vector of the obstacle
        theta: float
            heading of the obstacle
        length: float
            length of the obstacle
        width: float
            width of the obstacle

    Returns:
        points: List[Point]
            polygon points of the obstacle
    """
    points = []
    half_l = length / 2.0
    half_w = width / 2.0
    sin_h = math.sin(theta)
    cos_h = math.cos(theta)
    vectors = [(half_l * cos_h - half_w * sin_h,
                half_l * sin_h + half_w * cos_h),
               (-half_l * cos_h - half_w * sin_h,
                - half_l * sin_h + half_w * cos_h),
               (-half_l * cos_h + half_w * sin_h,
                - half_l * sin_h - half_w * cos_h),
               (half_l * cos_h + half_w * sin_h,
                half_l * sin_h - half_w * cos_h)]
    for x, y in vectors:
        p = Point()
        p.x = position.x + x
        p.y = position.y + y
        p.z = 0.0
        points.append(p)
    return points


def get_lane_boundary_points(boundary):
    """
    Given a lane boundary (left/right), return a list of x, y
    coordinates of all points in the boundary
    """
    return [(pt.x, pt.y) for pt in boundary]


def construct_lane_boundary_linestring(lane):
    """
    Description: Construct two linestrings for the lane's left and right boundary
    Input: A lane message.
    Output: A list containing the linestrings representing the left and right boundary of the lane
    """
    left_boundary_points = get_lane_boundary_points(lane.leftBound)
    right_boundary_points = get_lane_boundary_points(lane.rightBound)
    return LineString(left_boundary_points), LineString(right_boundary_points)


def calculate_velocity(linear_velocity: Point):
    x, y, z = linear_velocity.x, linear_velocity.y, linear_velocity.z
    return round(math.sqrt(x ** 2 + y ** 2), 2)


def calculate_accel(accel: Point):
    x, y, z = accel.x, accel.y, accel.z
    return round(math.sqrt(x ** 2 + y ** 2 + z ** 2), 2)


def quaternion_2_heading(orientation: Quaternion) -> float:
    """
    Convert quaternion to heading

    Parameters:
        orientation: Quaternion
            quaternion of the car

    Returns:
        The heading value of the car

    """

    def normalize_angle(angle):
        a = math.fmod(angle + math.pi, 2.0 * math.pi)
        if a < 0.0:
            a += (2.0 * math.pi)
        return a - math.pi

    yaw = math.atan2(2.0 * (orientation.w * orientation.z - orientation.x * orientation.y),
                     2.0 * (orientation.w * orientation.w + orientation.y * orientation.y) - 1.0)
    return normalize_angle(yaw)


def obstacle_to_polygon(obs: PredictedObject) -> Polygon:
    """
    Generate polygon for the Obstacle Object

    Parameters:
        obs: PredictedObject
            predicted object of the obstacle

    Returns:
        points: Polygon
            polygon of the obstacle
    """
    if obs.shape.type == 1:
        raise NotImplementedError("Not implemented for cylinder")
    obs_heading = quaternion_2_heading(obs.kinematics.initial_pose_with_covariance.pose.orientation)
    points = []
    half_w = obs.shape.dimensions.y / 2.0
    front_l = obs.shape.dimensions.x / 2.0
    # back_l of obstacles is half of the length
    back_l = -1 * obs.shape.dimensions.x / 2.0
    sin_h = math.sin(obs_heading)
    cos_h = math.cos(obs_heading)
    vectors = [(front_l * cos_h - half_w * sin_h,
                front_l * sin_h + half_w * cos_h),
               (back_l * cos_h - half_w * sin_h,
                back_l * sin_h + half_w * cos_h),
               (back_l * cos_h + half_w * sin_h,
                back_l * sin_h - half_w * cos_h),
               (front_l * cos_h + half_w * sin_h,
                front_l * sin_h - half_w * cos_h)]
    for x, y in vectors:
        p = Point()
        p.x = obs.kinematics.initial_pose_with_covariance.pose.position.x + x
        p.y = obs.kinematics.initial_pose_with_covariance.pose.position.y + y
        p.z = 0.0
        points.append(p)

    return Polygon([[x.x, x.y] for x in points])


def get_lane_lst(lane):
    l_pts = [p for p in lane]
    return LineString([[p.x, p.y] for p in l_pts])


def get_lane_lst_seg(lst: LineString):
    return list(map(LineString, zip(lst.coords[:-1], lst.coords[1:])))


def get_s_from_start_lst(lst: LineString, end: int) -> float:
    if end == 0:
        return 0.0
    return round(LineString(lst.coords[:end + 1]).length, 3)


def get_rounded_rand(lower, upper, round_to: int = 3):
    return round(random.uniform(lower, upper), round_to)


def calculate_time_delta(time1: Time, time2: Time) -> float:
    def epoch_time_to_datetime(epoch_time_sec, epoch_time_nsec):
        # Convert epoch time to datetime
        return datetime.utcfromtimestamp(epoch_time_sec) + timedelta(microseconds=epoch_time_nsec / 1000)

    dt1 = epoch_time_to_datetime(time1.sec, time1.nanosec)
    dt2 = epoch_time_to_datetime(time2.sec, time2.nanosec)
    delta = dt2 - dt1
    return delta.total_seconds() * 1e9


def distance(p1: Point, p2: Point):
    return math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)


def get_real_time_from_msg(header: Header):
    return header.stamp.sec * 1000000000 + header.stamp.nanosec


def get_obstacle_type(obs: ObstacleType) -> str:
    if obs == ObstacleType.CAR or obs == ObstacleType.MOTORCYCLE or obs == ObstacleType.BUS or obs == ObstacleType.TRUCK:
        return "vehicle"
    elif obs == ObstacleType.PEDESTRIAN:
        return "pedestrian"
    elif obs == ObstacleType.BICYCLE:
        return "bicycle"
    else:
        raise NotImplementedError("Obstacle type not implemented")


def get_obstacle_specific_type(_t: str) -> list[ObstacleType]:
    if _t == "vehicle":
        return [ObstacleType.CAR, ObstacleType.MOTORCYCLE, ObstacleType.BUS, ObstacleType.TRUCK]
    elif _t == "pedestrian":
        return [ObstacleType.PEDESTRIAN]
    elif _t == "bicycle":
        return [ObstacleType.BICYCLE]
    else:
        raise NotImplementedError("Obstacle type not implemented")


def obs_hash(length: float, width: float, height: float):
    return int(str(hash((length, width, height)) % (10**10))[:7])
