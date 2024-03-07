import math
from geometry_msgs.msg import Point
from shapely import LineString

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
        p.z = position.z
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
        p.z = position.z
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
