import unittest

from autoware.map_service import MapLoader, MapService
from geometry_msgs.msg import Point


class TestMapService(unittest.TestCase):
    def setUp(self):
        self.map_name = "Gebze_turkey_541"
        # self.map_name = "ces2024_demo"
        self.mapLoader = MapLoader(self.map_name)

    def test_find_junction_lanes(self):
        print(MapService.instance().get_junction_lanes())

    def test_find_non_junction_lanes(self):
        print(MapService.instance().get_non_junction_lanes())

    def test_center_lines(self):
        MapService.instance().get_center_line_lst_by_id(3302)
        # type of lane.centerline is lanelet2.core.ConstLineString3d
        # centerline[0] -- centerline[len(..) - 1]

    def test_get_all_lanelets(self):
        a = MapService.instance().ll_map.laneletLayer
        print(a)
        for k in a:
            print(k)

    def test_vel_shortest_path_src_tgt(self):
        pth = MapService.instance().get_vehicle_shortest_path_src_tgt(3302, 3314)
        self.assertEquals(len(pth), 15)

        [print(p.id) for p in pth]

    def test_shortest_path_src(self):
        reachable = MapService.instance().get_shortest_path_src(3302)
        paths = list(filter(lambda x: x is not None and len(x) > 1, reachable.values()))
        print(paths)

    def test_single_lane(self):
        lane = MapService.instance().get_lane_by_id(8252)
        print(lane.attributes.keys())
        print(lane.attributes['speed_limit'])
        print(MapService.instance().get_length_of_lane(8252))
        print(lane)

    def test_speed_limit(self):
        lane = MapService.instance().get_lane_by_id(8252)
        self.assertEqual(30.00, lane.attributes['speed_limit'])

    def test_get_pedestrian_paths(self):
        print(*MapService.instance().get_pedestrian_shortest_path_src(4112)[4112])  # this is for gebze turkey
        # print(MapService.instance().get_pedestrian_following(lane))

    def test_get_bicycles_paths(self):
        print(MapService.instance().get_bicycle_shortest_path_src(316))
        # print(MapService.instance().get_vehicle_shortest_path_src(316))

    def test_get_lane_successors(self):
        print(MapService.instance().find_descendants(800))
        {2051, 1287, 2064, 658, 3357, 3998, 800, 2080, 2208, 677, 3752, 2218, 3885, 4014, 1966, 690, 51, 3380, 2101,
         1981, 959, 2243, 3268, 841, 458, 2123, 1996, 848, 3795, 1109, 2773, 727, 4055, 2268, 1629, 2014, 1246, 4064,
         353, 3810, 4071, 3687, 2035, 631, 3962, 1276}

        {2051, 1287, 2064, 658, 3357, 3998, 800, 2080, 2208, 677, 3752, 2218, 3885, 4014, 1966, 690, 51, 3380, 2101,
         1981, 959, 2243, 3268, 841, 458, 2123, 1996, 848, 3795, 1109, 2773, 727, 4055, 2268, 1629, 2014, 1246, 4064,
         353, 3810, 4071, 3687, 2035, 631, 3962, 1276}

        {2051, 1287, 2064, 658, 3357, 3998, 2080, 800, 2208, 677, 3752, 2218, 3885, 1966, 4014, 690, 51, 3380, 2101,
         1981, 959, 2243, 3268, 841, 458, 2123, 1996, 848, 3795, 2773, 1109, 727, 4055, 2268, 1629, 1246, 2014, 4064,
         353, 3810, 3687, 4071, 2035, 631, 3962, 1276}

        [1981, 1287, 677, 1246, 848, 1276, 841, 959, 2080, 800, 727, 3795, 3810, 4064, 353, 2064, 1996, 2123, 51, 631,
         658, 3962, 2051, 1966, 3998, 2035, 3687, 2208, 2218, 2101, 3752, 2243, 2014, 2268, 2773, 4071, 1109, 3268, 690,
         3357, 3380, 3885, 458, 4014, 1629, 4055]

    def test_get_nearest_lanes_with_heading(self):
        MapService.instance().get_nearest_lanes_with_heading(Point(x=3.0,y=4.0,z=0.0), 0.0)

    def tearDown(self):
        del self.mapLoader


if __name__ == '__main__':
    unittest.main()
