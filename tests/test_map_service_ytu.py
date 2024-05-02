import unittest
import time

from autoware.map_service import MapService, load_map_service
from geometry_msgs.msg import Point, Pose, Quaternion


class TestMapServiceYTU(unittest.TestCase):
    def setUp(self):
        self.map_name = "YTU Davutpasa campus"
        self.map_service = load_map_service(self.map_name)

    def test_reachable_lanes_w_lane_changes(self):
        a = self.map_service.get_reachable_descendants(2672)
        b = self.map_service.get_reachable_descendants(2672, allow_lane_change=False)
        self.assertIn(6452, b)
        print(self.map_service.get_length_of_lane(6452))
        self.assertNotEquals(a, b)  # most time a != b

        print(self.map_service.get_reachable_descendants(7379, allow_lane_change=False))

    def test_successors_predecessors(self):
        print(self.map_service.get_predecessors_for_lane(7898))
        print(self.map_service.get_successors_for_lane(7898))
        print(self.map_service.get_length_of_lane(7898))

    @unittest.skip("unfinished")
    def test_center_lines(self):
        self.map_service.get_center_line_lst_by_id(2866)
        # LINESTRING (802.2410000001546 45367.88584999996, 788.0703499994124 45375.748550000135, 773.6706000004488 45383.015549999895, 773.4805000011984 45383.10980000044, 773.3012500008626 45383.2126000002)
        # type of lane.centerline is lanelet2.core.ConstLineString3d
        # centerline[0] -- centerline[len(..) - 1]

    @unittest.skip("unfinished")
    def test_get_all_lanelets(self):
        self.map_service.get_non_junction_lanes()
        self.assertEquals(788, len(self.map_service.all_ln_ids))

    @unittest.skip("unfinished")
    def test_vel_shortest_path_src_tgt(self):
        pth = self.map_service.get_vehicle_shortest_path_src_tgt(5279, 5280, False)
        self.assertIsNone(pth)
        pth = self.map_service.get_vehicle_shortest_path_src_tgt(5279, 5280, True)
        self.assertIsNotNone(pth)
        self.assertEquals([5279, 5280], [p.id for p in pth])

    @unittest.skip("unfinished")
    def test_single_lane(self):
        lane = self.map_service.get_lane_by_id(2012)
        self.assertEquals(
            {
                "type": "lanelet",
                "subtype": "road",
                "speed_limit": "30",
                "location": 'urban',
                "one_way": "yes"
            }, lane.attributes
        )
        self.assertAlmostEqual(13.75, self.map_service.get_length_of_lane(2012), delta=0.01)

    def test_lane_length(self):
        self.assertAlmostEqual(14.70, self.map_service.get_length_of_lane(2151), delta=0.01)

    @unittest.skip("unfinished")
    def test_get_current_lane(self):
        # 625.8884, 46525.4999
        # 58649.6655, 43455.7251
        lanes = self.map_service.get_veh_current_lanelets(Point(x=58649.6655, y=43455.7251, z=0.))
        lanes = self.map_service.get_nearest_lanes_with_heading(
            Pose(
                position=Point(x=58654.1228, y=43470.7423, z=0.),
                orientation=Quaternion(x=0., y=0., z=.6, w=0.4)
            )
        )
        self.assertEquals([7714], [ll.id for ll in lanes])

    @unittest.skip("skip")
    def test_get_pedestrian_paths(self):
        self.assertEquals([11016, 11037, 11030, 11023], self.map_service.get_pedestrian_lanes())
        for ll in self.map_service.get_pedestrian_lanes():
            self.assertEquals({ll}, self.map_service.get_reachable_descendants(ll, "pedestrian", False))
            self.assertEquals({ll}, self.map_service.get_reachable_descendants(ll, "pedestrian", True))
            self.assertEquals([ll], [p.id for p in
                                     self.map_service.get_pedestrian_shortest_path_src_tgt(ll, ll, False)])
            self.assertEquals([ll], [p.id for p in
                                     self.map_service.get_pedestrian_shortest_path_src_tgt(ll, ll, True)])

    @unittest.skip("skip")
    def test_get_bicycles_paths(self):
        def contain_participants(lane):
            return [p for p in lane.attributes.keys() if 'participant' in p]

        bic_lanes = [2850, 10250, 10251, 10252, 4109, 4108, 4107, 10253, 4119, 4120, 2076, 2077, 2079, 2080, 2081, 6178,
                     6179, 2083, 2082, 6180, 6182, 6184, 6183, 6177, 6181, 4149, 4150, 4151, 8153, 8154, 4164, 8156,
                     4174, 4175, 4176, 4177, 6222, 6223, 6224, 6225, 6226, 6227, 4187, 4188, 10333, 10334, 10335, 10336,
                     2165, 8310, 8311, 2166, 2167, 8314, 8315, 8316, 8317, 8318, 8319, 8320, 8321, 8322, 8323, 8324,
                     2175, 8326, 8327, 8325, 2176, 2177, 2179, 2180, 2181, 2182, 2183, 6288, 4241, 4242, 8339, 8340,
                     6293, 6292, 6286, 2200, 2201, 2202, 10613, 4252, 4253, 8356, 10616, 10407, 10408, 10409, 10410,
                     4267, 4268, 4269, 4266, 4271, 4270, 10618, 10617, 10619, 8378, 8379, 8380, 8381, 6339, 6340, 6341,
                     6342, 6343, 6344, 6345, 6346, 6347, 8397, 8398, 8399, 10453, 10454, 10455, 10456, 8409, 8410,
                     10458, 10457, 10461, 10462, 10463, 10464, 10465, 10466, 10467, 10468, 10469, 10470, 10471, 4325,
                     4326, 10472, 10475, 10476, 8429, 10478, 10479, 10477, 10481, 10482, 4339, 4340, 10480, 8440, 8441,
                     4350, 4351, 10501, 10502, 10503, 10504, 10505, 4367, 6424, 6425, 8477, 8478, 8479, 8480, 8481,
                     6435, 8506, 8507, 8508, 8515, 4435, 8532, 4436, 4437, 6491, 6492, 6493, 4450, 4451, 8558, 8559,
                     8560, 8561, 10610, 10611, 10612, 8563, 6514, 10615, 8562, 6515, 6516, 10614, 10620, 10621, 10622,
                     10623, 10624, 4477, 6529, 6531, 6532, 6530, 4478, 4479, 4488, 8585, 6533, 8586, 6540, 10637, 398,
                     399, 4496, 10638, 6547, 409, 10650, 10651, 410, 8605, 8606, 6554, 6555, 6556, 6566, 4519, 4520,
                     2471, 8618, 10667, 2473, 2474, 2477, 2478, 6567, 8625, 8626, 8627, 4531, 6581, 6574, 4530, 8637,
                     8638, 10687, 10686, 10688, 10689, 1718, 4549, 4550, 6600, 1719, 8653, 1720, 10704, 2516, 10715,
                     10716, 8669, 1723, 4578, 2531, 4579, 4580, 4581, 2530, 3761, 8684, 8691, 8709, 8716, 4625, 4626,
                     2579, 2580, 4629, 4628, 4627, 8732, 4645, 5808, 4646, 4663, 6713, 6714, 6715, 6716, 6717, 4678,
                     4701, 6751, 6752, 6753, 2658, 2659, 2660, 2661, 4708, 6754, 6755, 7857, 4722, 4723, 6773, 6774,
                     6775, 10876, 10877, 10878, 4733, 4734, 6792, 6793, 4748, 10896, 2711, 2712, 2714, 2168, 4764,
                     10911, 10912, 2169, 4771, 4783, 8880, 2173, 10935, 4793, 9908, 2174, 2749, 2750, 2752, 2753, 6857,
                     6858, 2178, 4825, 4826, 4827, 4834, 2184, 4850, 6901, 6902, 2808, 2809, 2810, 2811, 4872, 7482,
                     799, 2849, 802, 803, 2852, 2851, 6950, 2853, 6951, 6952, 4900, 4901, 4902, 2855, 2856, 2854, 2866,
                     2867, 2874, 2884, 2885, 4935, 841, 842, 2902, 7006, 7007, 7008, 2915, 2920, 2921, 2922, 2923, 2924,
                     2925, 878, 882, 2938, 2939, 6285, 6287, 6289, 5021, 5022, 5023, 5024, 5025, 5026, 5027, 5028, 5029,
                     5030, 6290, 6291, 5050, 3025, 3026, 979, 980, 3028, 3027, 981, 3048, 3049, 5100, 1005, 5101, 1007,
                     7165, 7166, 7167, 7168, 1052, 3101, 3102, 1053, 5154, 5155, 5156, 5157, 5158, 5159, 5160, 5161,
                     1067, 1068, 5174, 5175, 5212, 5213, 5214, 1126, 1127, 1128, 1129, 5923, 9326, 9327, 5234, 7311,
                     7317, 5279, 5280, 5281, 5282, 5296, 5297, 5307, 5308, 3261, 3262, 3269, 3273, 5324, 3295, 9441,
                     9442, 9443, 9444, 9445, 5345, 5358, 5359, 3317, 5386, 5387, 9488, 5400, 5401, 5402, 9503, 9504,
                     5412, 5413, 9512, 1484, 1485, 7477, 3382, 3383, 3384, 7478, 7479, 7480, 7481, 7485, 7486, 7487,
                     7488, 7489, 7490, 7491, 7483, 7484, 6374, 5456, 5464, 5465, 5487, 3449, 3450, 3451, 3452, 9610,
                     9611, 9612, 9613, 5518, 9614, 1424, 3473, 3474, 3475, 1431, 5533, 1438, 8428, 5554, 5582, 5583,
                     5564, 5565, 9675, 9676, 9677, 9678, 9679, 9680, 9681, 9682, 1490, 1492, 1489, 1491, 7630, 7631,
                     10473, 10474, 5606, 5607, 7629, 5617, 5618, 7675, 5630, 5631, 3587, 3588, 5644, 5645, 5646, 5659,
                     5660, 3613, 3614, 5661, 7712, 9765, 9766, 9767, 9768, 9769, 9770, 9771, 9772, 9773, 9774, 9775,
                     9776, 1579, 9778, 9777, 9780, 9779, 5677, 3642, 5699, 3654, 1615, 3667, 3668, 3669, 7768, 7769,
                     7770, 7771, 5726, 5727, 7778, 7789, 7790, 7791, 7792, 7805, 7806, 7807, 3718, 3719, 3720, 5767,
                     5768, 5769, 5772, 5770, 5771, 5782, 5783, 7835, 5790, 3748, 3749, 3750, 7848, 7849, 7850, 9901,
                     9902, 9903, 9904, 9905, 9906, 9907, 3760, 9909, 9910, 9911, 9912, 9913, 5818, 9914, 3772, 9917,
                     3771, 5819, 9915, 9916, 1721, 1722, 1724, 7880, 7881, 7882, 3787, 3788, 7883, 7884, 7885, 3798,
                     3799, 3810, 9955, 9956, 9957, 9954, 3811, 5864, 5865, 5863, 5866, 5867, 3821, 5862, 3823, 3827,
                     1782, 1784, 1785, 3843, 3844, 3845, 3846, 5910, 5911, 5912, 7961, 7962, 7960, 7964, 7965, 1819,
                     7967, 7968, 7969, 7970, 7966, 5924, 3877, 7974, 7975, 7976, 3876, 7972, 7973, 7980, 5934, 5935,
                     3889, 3890, 3891, 10041, 10042, 10043, 10044, 3902, 3903, 5962, 5963, 1868, 1867, 5964, 1869, 3923,
                     5988, 6008, 6021, 6022, 6023, 3979, 5676, 6036, 3989, 6037, 6038, 3990, 1946, 1947, 1948, 10145,
                     4003, 6052, 10149, 10150, 6053, 4004, 6063, 6064, 6075, 8124, 6076, 4035, 10187, 10188, 8150, 8151,
                     8152, 2009, 2010, 2011, 2012, 8157, 8158, 8159, 8160, 2013, 8155, 2019, 2015, 2017, 2014, 2016,
                     2018, 8170, 8171, 8172, 8173]

        self.assertEquals(bic_lanes, self.map_service.get_bicycle_lanes())
        for ll in bic_lanes:
            lane = self.map_service.get_lane_by_id(ll)
            self.assertEquals("lanelet", lane.attributes["type"])
            self.assertIn(lane.attributes['subtype'],
                          ['road', 'play_street', 'bicycle_lane', 'exit', 'shared_walkway', 'pedestrian_lane'])

        a = self.map_service.get_reachable_descendants(10251, "bicycle", False)
        b = self.map_service.get_reachable_descendants(10251, "bicycle", True)
        self.assertEquals(a, b)  # no outlet

        pth = self.map_service.get_bicycle_shortest_path_src_tgt(5279, 5280, False)
        self.assertIsNone(pth)
        pth = self.map_service.get_bicycle_shortest_path_src_tgt(5279, 5280, True)
        self.assertIsNotNone(pth)
        self.assertEquals([5279, 5280], [p.id for p in pth])

    @unittest.skip("skip")
    def test_get_current_lanelet(self):
        # 625.8884, 46525.4999
        lane = self.map_service.get_veh_current_lanelets(Point(x=620.4646, y=46512.2191, z=0.))  # z does not matter
        self.assertEquals([10251], [ll.id for ll in lane])

    @unittest.skip("unfinished")
    def test_nearest_lane(self):
        lanes = self.map_service.get_nearest_lanes_w_range(
            Pose(
                position=Point(x=2004.7898, y=45049.6895, z=0.),
                orientation=Quaternion(x=0., y=0., z=0., w=1.)
            ), rng=3.0)
        self.assertEquals([5279, 5280], [ln.id for ln in lanes])

    @unittest.skip("skip")
    def test_is_in_lane(self):
        p = Pose(
            position=Point(x=2004.2418, y=45049.811, z=0.),
            orientation=Quaternion(x=0., y=0., z=0., w=1.)
        )

        self.assertTrue(self.map_service.is_in_lane(p, 5279))

    @unittest.skip("unfinished")
    def test_boundary_is_in_lane(self):
        p = Pose(
            position=Point(x=620.4646, y=46512.2191, z=0.),
            orientation=Quaternion(x=0., y=0., z=0., w=1.)
        )

        self.assertTrue(self.map_service.is_in_lane(p, 10250))
        self.assertTrue(self.map_service.is_in_lane(p, 10251))

    @unittest.skip("skip")
    def test_get_lane_successors(self):
        print(self.map_service.get_reachable_descendants(1718, "vehicle", False))
        print(self.map_service.get_successors_for_lane(1718))
        print(1718)
        print(self.map_service.get_lane_by_id(1718).leftBound)
        print(self.map_service.get_lane_by_id(1718).rightBound)
        print(1720)
        print(self.map_service.get_lane_by_id(1720).leftBound)
        print(self.map_service.get_lane_by_id(1720).rightBound)
        print(1721)
        print(self.map_service.get_lane_by_id(1721).leftBound)
        print(self.map_service.get_lane_by_id(1721).rightBound)

    @unittest.skip("skip")
    def test_get_lane_predecessors(self):
        print(self.map_service.get_successors_for_lane(5818))
        print(self.map_service.get_reachable_descendants(5818))

    def test_gen_0_sce_5(self):
        print(*self.map_service.get_vehicle_shortest_path_src_tgt(1891, 4136, False))

    def tearDown(self):
        del self.map_service


if __name__ == '__main__':
    unittest.main()
