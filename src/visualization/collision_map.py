import math

import matplotlib.pyplot as plt
import shapely
from autoware_auto_perception_msgs.msg import TrackedObject
from geometry_msgs.msg import Quaternion
from shapely import Point
from autoware.map_service import load_map_service

from autoware.utils import quaternion_2_heading, obstacle_to_polygon, AUTOWARE_VEHICLE_WIDTH, AUTOWARE_VEHICLE_LENGTH, \
    AUTOWARE_VEHICLE_back_edge_to_center
from matplotlib.patches import Polygon


def draw_last_position():
    fig, ax = plt.subplots()

    def obstacle_to_polygon(
            pos_x: float, pos_y: float, theta: float, length: float, width: float
    ):
        points = []
        half_l = length / 2.0
        half_w = width / 2.0
        sin_h = math.sin(theta)
        cos_h = math.cos(theta)
        vectors = [
            (half_l * cos_h - half_w * sin_h, half_l * sin_h + half_w * cos_h),
            (-half_l * cos_h - half_w * sin_h, -half_l * sin_h + half_w * cos_h),
            (-half_l * cos_h + half_w * sin_h, -half_l * sin_h - half_w * cos_h),
            (half_l * cos_h + half_w * sin_h, half_l * sin_h - half_w * cos_h),
        ]
        for x, y in vectors:
            points.append((pos_x + x, pos_y + y))
        return points

    def generate_adc_polygon(x, y, theta: float):
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
        for x_, y_ in vectors:
            points.append((x + x_, y + y_))
        return points
        # return Polygon([[xx.x, xx.y] for xx in points])

    # q1=Quaternion(x=ego_pose[-1][3], y=ego_pose[-1][4], z=ego_pose[-1][5], w=ego_pose[-1][6])
    # q2=Quaternion(x=obs_pose[-1][3], y=obs_pose[-1][4], z=obs_pose[-1][5], w=obs_pose[-1][6])
    obs_ply = obstacle_to_polygon(81574.5395497169, 50202.73681181819, -1.3962531495594706, 4.0, 1.8)
    # print(obs_ply)
    obs_oo = shapely.geometry.Polygon(obs_ply)
    adc_ply = generate_adc_polygon(81579.16146743215, 50200.23833221861, -3.0552839324882917)
    # print(adc_ply)

    ego_oo = shapely.geometry.Polygon(adc_ply)

    # print(obs_oo.distance(ego_oo))
    x_min = float('inf')
    x_max = -1
    y_min = float('inf')
    y_max = -1
    for obs in obs_ply:
        x_min = min(obs[0], x_min)
        x_max = max(obs[0], x_max)
        y_min = min(obs[1], y_min)
        y_max = max(obs[1], y_max)

    for ego in adc_ply:
        x_min = min(ego[0], x_min)
        x_max = max(ego[0], x_max)
        y_min = min(ego[1], y_min)
        y_max = max(ego[1], y_max)

    lanes_id = [410, 398, 399, 397]
    for lane_id in lanes_id:
        l, r = map_service.get_lane_boundaries_by_id(lane_id)
        cl = map_service.get_center_line_lst_by_id(lane_id)
        draw_lane(l, ax)
        draw_lane(r, ax)
        draw_road_direction(cl, ax)

    # polygon = Polygon(adc_ply, closed=True, edgecolor='#009E73', facecolor='#009E73', linewidth=1, zorder=2)
    # ax.add_patch(polygon)
    # ax.set_label('Polygon')
    #
    # polygon2 = Polygon(obs_ply, closed=True, edgecolor='#ff718e', facecolor='#ff718e', linewidth=1, zorder=2)
    # ax.add_patch(polygon2)
    # ax.set_label('Polygon2')
    # print(x_max, x_min, y_max, y_min)
    ax.set_xlim(x_min - 15, x_max + 15)
    ax.set_ylim(y_min - 15, y_max + 15)

    # ls1 = [(x[0], x[1]) for x in ego_pose]
    # ls2 = [(x[0], x[1]) for x in obs_pose]
    #
    # x, y = zip(*ls1)
    # ax.plot(x, y, label='Linestring')
    #
    # z, w = zip(*ls2)
    # ax.plot(z, w, label='Linestring')
    plt.axis('off')
    plt.savefig("./road.jpg", bbox_inches='tight', pad_inches=0.15, dpi=600)
    plt.show()


def draw_lane(l, ax):
    xx, yy = l.xy
    ax.plot(xx, yy, label='l', color='grey', zorder=1)


def draw_road_direction(l, ax):
    xx, yy = l.xy
    for i in range(len(xx) - 1):
        ax.arrow(x=xx[i], y=yy[i], dx=xx[i + 1] - xx[i], dy=yy[i + 1] - yy[i], head_width=0.5, head_length=.5,
                 fc='lightblue', ec='#2c9abc', zorder=1)


if __name__ == '__main__':
    map_service = load_map_service("awf_cicd_virtualmap")
    draw_last_position()
