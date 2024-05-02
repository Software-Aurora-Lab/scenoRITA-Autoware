import unittest
import time

from autoware.map_service import MapService, load_map_service
from geometry_msgs.msg import Point


class TestMapServiceGebze(unittest.TestCase):
    def setUp(self):
        self.map_name = "Gebze (Turkey)"
        self.map_service = load_map_service(self.map_name)

    def test_reachable_lanes_w_lane_changes(self):
        a = self.map_service.get_reachable_descendants(208)
        b = self.map_service.get_reachable_descendants(208,
                                                       allow_lane_change=False)
        self.assertIn(927, a)
        self.assertNotIn(927, b)
        self.assertNotEquals(a, b)  # most time a != b

    def test_find_junction_lanes(self):
        junction_lanes = [4064, 4055, 4071]
        self.assertEquals(junction_lanes,
                          self.map_service.get_junction_lanes())

    def test_find_non_junction_lanes(self):
        non_junction_lanes = [642, 2051, 1287, 3981, 2064, 658, 3357, 3998, 927, 2080, 800, 2208, 288, 677, 3752, 2218,
                              940, 3885, 1966, 4014, 690, 51, 3380, 2101, 1981, 959, 2112, 2243, 3268, 4037, 712, 841,
                              458, 2123, 972, 1996, 4042, 848, 208, 3795, 2773, 1109, 727, 2268, 1629, 1246, 223, 2014,
                              353, 3810, 3687, 2035, 631, 3962, 1276]
        self.assertEquals(non_junction_lanes,
                          self.map_service.get_non_junction_lanes())

    def test_veh_lanes_cannot_have_speed_bumps(self):
        for lid in self.map_service.get_vehicle_lanes():
            lane = self.map_service.get_lane_by_id(lid)
            self.assertNotIn('speed_bump', lane.attributes)

    def test_bikes_does_not_have_lanes(self):
        self.assertEquals([], self.map_service.get_bicycle_lanes())

    def test_pedestrian_lanes(self):
        self.assertEquals([4098, 4133, 4105, 4140, 4112, 4119, 4126], self.map_service.get_pedestrian_lanes())

    def test_center_lines(self):
        print(self.map_service.get_center_line_lst_by_id(927))
        # LINESTRING (99079.31709999993 21072.48235000018, 99078.75420000008 21070.046850000042, 99078.19119999977 21067.611250000075, 99077.62825000013 21065.175699999556, 99077.06530000048 21062.7402499998, 99076.50224999996 21060.304700000677, 99075.9393000002 21057.869100000244, 99075.37640000053 21055.433500000276, 99074.81339999993 21052.997950001154, 99074.25045000005 21050.562450001016, 99073.6874500002 21048.126900000498, 99073.12460000068 21045.69180000061, 99072.56150000042 21043.255750000477, 99072.00070000056 21040.821499999613, 99071.43895000045 21038.383699999657, 99070.88285000017 21035.946449999698, 99070.32675000012 21033.509200000204, 99069.77094999957 21031.072700000368, 99069.21484999976 21028.634449999314, 99068.6616500002 21026.197699999902, 99068.10800000024 21023.758800000418, 99067.55780000001 21021.320300000254, 99067.00764999964 21018.88169999933, 99066.4573999996 21016.44319999963, 99065.90720000019 21014.004550000187, 99065.35684999992 21011.566200000234, 99064.80644999951 21009.127700000536, 99064.2557999997 21006.68914999999, 99063.70515 21004.250699999277, 99063.15485000005 21001.813349999953, 99062.60400000005 20999.373700000346, 99062.05699999939 20996.934699999634, 99061.50994999975 20994.495049999095, 99060.96384999924 20992.055499999784, 99060.41789999988 20989.616049999837, 99059.87310000008 20987.1814500005, 99059.32570000022 20984.736900000833, 99058.76470000017 20982.30145000061, 99058.20354999963 20979.865200000815, 99057.64124999993 20977.429750000127, 99057.07895000017 20974.994199999608, 99056.51650000026 20972.55905000074, 99055.95405000006 20970.12315000128, 99055.3904999998 20967.68745000055, 99054.82684999978 20965.251749999356, 99054.26344999991 20962.816849999595, 99053.69975000032 20960.380450000055, 99053.13455000002 20957.947300000116, 99052.56830000004 20955.5103000002, 99051.99670000002 20953.07664999971, 99051.42510000028 20950.643049999606, 99050.8541000004 20948.211849999614, 99050.28200000018 20945.77584999986, 99049.71845000004 20943.343899999745, 99049.15335000027 20940.905200000387, 99048.59930000018 20938.467600000557, 99048.04524999973 20936.02990000043, 99047.49155000015 20933.594350000378, 99046.93689999986 20931.154700000305, 99046.38919999986 20928.717050001025, 99045.84079999995 20926.276550000533, 99045.29704999994 20923.83660000004, 99044.75335000036 20921.39660000056, 99044.20955000032 20918.9565500007, 99043.66575000033 20916.516399999615, 99043.1221000005 20914.076399999205, 99042.57850000018 20911.636450000107, 99042.0350000005 20909.196449999698, 99041.4915 20906.75635000039, 99040.94819999964 20904.317600000184, 99040.40429999959 20901.876249999274, 99039.85679999983 20899.437249999028, 99039.30924999993 20896.998050000053, 99038.76140000025 20894.559199999087, 99038.21345000016 20892.11999999918, 99037.66650000005 20889.682499999646, 99037.11879999976 20887.24164999975, 99036.57644999988 20884.80134999985, 99036.0341000001 20882.360999999568, 99035.49195000005 20879.9211500003, 99034.94959999976 20877.48029999947, 99034.4061499997 20875.041649999563, 99033.86214999977 20872.600449999794, 99033.31444999977 20870.1614000001, 99032.76680000004 20867.722250000108, 99032.2192000004 20865.283199999947, 99031.67155000026 20862.843850001227, 99031.12355000002 20860.405000001658, 99030.57554999983 20857.965750000905, 99030.02709999972 20855.526800000574, 99029.47865000041 20853.08790000016, 99028.93085000024 20850.651550000068, 99028.38184999989 20848.209999999963, 99027.82599999988 20845.7747499994, 99027.26924999955 20843.335750000086, 99026.70715000003 20840.899949998595, 99026.14505000005 20838.464099998586, 99025.58329999971 20836.02929999912, 99025.02110000013 20833.59250000026, 99024.46230000013 20831.15635000076, 99023.90329999977 20828.719450000674, 99023.3454499999 20826.282650000416, 99022.7875999998 20823.845800000243, 99022.23014999961 20821.41069999989, 99021.67204999994 20818.97229999909, 99021.11940000043 20816.534349998925, 99020.5666000002 20814.096099998802, 99020.0144000001 20811.65804999927, 99019.46220000018 20809.219849999994, 99018.91000000056 20806.783850000706, 99018.35689999984 20804.343850000296, 99017.79829999979 20801.907099998556, 99017.2398500003 20799.47034999961, 99016.68130000017 20797.033799998928, 99016.12269999989 20794.596950000152, 99015.56459999934 20792.16054999968, 99015.00649999932 20789.723349999636, 99014.44960000017 20787.28624999989, 99013.89270000014 20784.84909999976, 99013.33605000086 20782.413250000216, 99012.7788500007 20779.974700000137, 99012.21855000005 20777.540549999103, 99011.65729999996 20775.10249999957, 99011.09065000003 20772.667499999516, 99010.52404999977 20770.232699999586, 99009.9589999998 20767.804749999195, 99009.39069999999 20765.36309999926, 99008.84599999996 20762.92874999903, 99008.29899999959 20760.48404999962, 99007.76900000003 20758.04089999944, 99007.23894999985 20755.597749999724, 99006.70994999981 20753.158999999985, 99006.17905000056 20750.711549999658, 99005.63605000032 20748.273099999875, 99005.09225000005 20745.831300000194, 99004.54390000022 20743.392250000034, 99003.9955500004 20740.953150000423, 99003.44835000031 20738.51875000121, 99002.89899999998 20736.075100000482, 99002.33665000007 20733.63979999954, 99001.77400000003 20731.203399999533, 99001.21005000005 20728.768149999436, 99000.64579999959 20726.33250000002, 99000.08304999978 20723.90010000067, 98999.51889999973 20721.461299999617, 98998.96514999965 20719.02344999928, 98998.4114000001 20716.585550000425, 98997.85759999987 20714.147650001105, 98997.30385000026 20711.709800000302, 98996.75009999936 20709.27195000043, 98996.19634999934 20706.83415000001, 98995.98289999948 20705.909549999982, 98995.76935000031 20704.985350000206)
        # type of lane.centerline is lanelet2.core.ConstLineString3d
        # centerline[0] -- centerline[len(..) - 1]

    def test_vel_shortest_path_src_tgt(self):
        pth = self.map_service.get_vehicle_shortest_path_src_tgt(208, 927, False)
        self.assertIsNone(pth)
        pth = self.map_service.get_vehicle_shortest_path_src_tgt(208, 927)
        self.assertIsNotNone(pth)

    def test_single_lane(self):
        lane = self.map_service.get_lane_by_id(208)
        self.assertEquals(
            {
                "location": "urban",
                "one_way": "yes",
                "participant:vehicle": "yes",
                "speed_limit": "30",
                "subtype": "road",
                "type": "lanelet",
            },
            lane.attributes
        )

    @DeprecationWarning
    def reachable_vs_full_graph(self):
        time1 = time.time()
        shortest_paths = self.map_service.get_vehicle_shortest_path_src(3382)
        time2 = time.time()
        print(f"Time taken for shortest path: {time2 - time1}")
        time3 = time.time()
        reachable = self.map_service.get_reachable_descendants(3382)
        self.map_service.get_vehicle_shortest_path_src_tgt(3382, 4109)
        time4 = time.time()
        print(f"Time taken for reachable: {time4 - time3}")

        # Time taken for shortest path: 0.2619130611419678
        # Time taken for reachable: 0.0011911392211914062

    @unittest.skip("unfinished")
    def test_speed_limit(self):
        lane = self.map_service.get_lane_by_id(8252)
        print(lane)
        self.assertEqual(30.00, float(lane.attributes['speed_limit']))

    @unittest.skip("unfinished")
    def test_get_pedestrian_paths(self):
        print(self.map_service.get_pedestrian_shortest_path_src_tgt(4112, 4112))  # this is for gebze turkey

    @unittest.skip("unfinished")
    def test_get_bicycles_paths(self):
        print(self.map_service.get_bicycle_shortest_path_src_tgt(316))
        # print(self.map_service.get_vehicle_shortest_path_src(316))

    @unittest.skip("unfinished")
    def test_get_current_lanelet(self):
        lane = self.map_service.get_veh_current_lanelets(Point(x=59192.2, y=43041.8, z=0.))  # z does not matter
        self.assertEquals([8221], lane.id)

    def test_get_lane_successors(self):
        print(self.map_service.get_reachable_descendants(800))
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

    @unittest.skip("unfinished")
    def test_get_nearest_lanes_with_heading(self):
        self.map_service.get_nearest_lanes_with_heading(Point(x=3.0, y=4.0, z=0.0), 0.0)

    def test_get_lane_length(self):
        print(self.map_service.get_length_of_lane(4071))

    def test_speed_bump(self):
        assert 4196 not in self.map_service.get_vehicle_lanes()


if __name__ == '__main__':
    unittest.main()
