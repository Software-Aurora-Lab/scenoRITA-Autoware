# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/planning/proto/planning.proto

import sys

_b = sys.version_info[0] < 3 and (lambda x: x) or (lambda x: x.encode("latin1"))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf.internal import enum_type_wrapper

# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from modules.canbus.proto import (
    chassis_pb2 as modules_dot_canbus_dot_proto_dot_chassis__pb2,
)
from modules.common.proto import (
    drive_state_pb2 as modules_dot_common_dot_proto_dot_drive__state__pb2,
)
from modules.common.proto import (
    geometry_pb2 as modules_dot_common_dot_proto_dot_geometry__pb2,
)
from modules.common.proto import (
    header_pb2 as modules_dot_common_dot_proto_dot_header__pb2,
)
from modules.common.proto import (
    pnc_point_pb2 as modules_dot_common_dot_proto_dot_pnc__point__pb2,
)
from modules.map.proto import map_id_pb2 as modules_dot_map_dot_proto_dot_map__id__pb2
from modules.planning.proto import (
    decision_pb2 as modules_dot_planning_dot_proto_dot_decision__pb2,
)
from modules.planning.proto import (
    planning_internal_pb2 as modules_dot_planning_dot_proto_dot_planning__internal__pb2,
)

DESCRIPTOR = _descriptor.FileDescriptor(
    name="modules/planning/proto/planning.proto",
    package="apollo.planning",
    syntax="proto2",
    serialized_options=None,
    serialized_pb=_b(
        '\n%modules/planning/proto/planning.proto\x12\x0f\x61pollo.planning\x1a"modules/canbus/proto/chassis.proto\x1a&modules/common/proto/drive_state.proto\x1a#modules/common/proto/geometry.proto\x1a!modules/common/proto/header.proto\x1a$modules/common/proto/pnc_point.proto\x1a\x1emodules/map/proto/map_id.proto\x1a%modules/planning/proto/decision.proto\x1a.modules/planning/proto/planning_internal.proto")\n\x05\x45Stop\x12\x10\n\x08is_estop\x18\x01 \x01(\x08\x12\x0e\n\x06reason\x18\x02 \x01(\t"*\n\tTaskStats\x12\x0c\n\x04name\x18\x01 \x01(\t\x12\x0f\n\x07time_ms\x18\x02 \x01(\x01"q\n\x0cLatencyStats\x12\x15\n\rtotal_time_ms\x18\x01 \x01(\x01\x12.\n\ntask_stats\x18\x02 \x03(\x0b\x32\x1a.apollo.planning.TaskStats\x12\x1a\n\x12init_frame_time_ms\x18\x03 \x01(\x01"\x9f\x02\n\x07RSSInfo\x12\x13\n\x0bis_rss_safe\x18\x01 \x01(\x08\x12\x14\n\x0c\x63ur_dist_lon\x18\x02 \x01(\x01\x12\x19\n\x11rss_safe_dist_lon\x18\x03 \x01(\x01\x12\x1d\n\x15\x61\x63\x63_lon_range_minimum\x18\x04 \x01(\x01\x12\x1d\n\x15\x61\x63\x63_lon_range_maximum\x18\x05 \x01(\x01\x12"\n\x1a\x61\x63\x63_lat_left_range_minimum\x18\x06 \x01(\x01\x12"\n\x1a\x61\x63\x63_lat_left_range_maximum\x18\x07 \x01(\x01\x12#\n\x1b\x61\x63\x63_lat_right_range_minimum\x18\x08 \x01(\x01\x12#\n\x1b\x61\x63\x63_lat_right_range_maximum\x18\t \x01(\x01"\xa3\t\n\rADCTrajectory\x12%\n\x06header\x18\x01 \x01(\x0b\x32\x15.apollo.common.Header\x12\x19\n\x11total_path_length\x18\x02 \x01(\x01\x12\x17\n\x0ftotal_path_time\x18\x03 \x01(\x01\x12%\n\x05\x65stop\x18\x06 \x01(\x0b\x32\x16.apollo.planning.EStop\x12.\n\x05\x64\x65\x62ug\x18\x08 \x01(\x0b\x32\x1f.apollo.planning_internal.Debug\x12\x18\n\tis_replan\x18\t \x01(\x08:\x05\x66\x61lse\x12\x31\n\x04gear\x18\n \x01(\x0e\x32#.apollo.canbus.Chassis.GearPosition\x12\x38\n\x10trajectory_point\x18\x0c \x03(\x0b\x32\x1e.apollo.common.TrajectoryPoint\x12,\n\npath_point\x18\r \x03(\x0b\x32\x18.apollo.common.PathPoint\x12\x31\n\x08\x64\x65\x63ision\x18\x0e \x01(\x0b\x32\x1f.apollo.planning.DecisionResult\x12\x34\n\rlatency_stats\x18\x0f \x01(\x0b\x32\x1d.apollo.planning.LatencyStats\x12-\n\x0erouting_header\x18\x10 \x01(\x0b\x32\x15.apollo.common.Header\x12L\n\x13right_of_way_status\x18\x11 \x01(\x0e\x32/.apollo.planning.ADCTrajectory.RightOfWayStatus\x12!\n\x07lane_id\x18\x12 \x03(\x0b\x32\x10.apollo.hdmap.Id\x12\x32\n\rengage_advice\x18\x13 \x01(\x0b\x32\x1b.apollo.common.EngageAdvice\x12\x46\n\x0f\x63ritical_region\x18\x14 \x01(\x0b\x32-.apollo.planning.ADCTrajectory.CriticalRegion\x12O\n\x0ftrajectory_type\x18\x15 \x01(\x0e\x32-.apollo.planning.ADCTrajectory.TrajectoryType:\x07UNKNOWN\x12\x15\n\rreplan_reason\x18\x16 \x01(\t\x12(\n\x0etarget_lane_id\x18\x17 \x03(\x0b\x32\x10.apollo.hdmap.Id\x12\x17\n\x0f\x63\x61r_in_dead_end\x18\x18 \x01(\x08\x12*\n\x08rss_info\x18\x64 \x01(\x0b\x32\x18.apollo.planning.RSSInfo\x1a\x38\n\x0e\x43riticalRegion\x12&\n\x06region\x18\x01 \x03(\x0b\x32\x16.apollo.common.Polygon"2\n\x10RightOfWayStatus\x12\x0f\n\x0bUNPROTECTED\x10\x00\x12\r\n\tPROTECTED\x10\x01"a\n\x0eTrajectoryType\x12\x0b\n\x07UNKNOWN\x10\x00\x12\n\n\x06NORMAL\x10\x01\x12\x11\n\rPATH_FALLBACK\x10\x02\x12\x12\n\x0eSPEED_FALLBACK\x10\x03\x12\x0f\n\x0bPATH_REUSED\x10\x04*_\n\x07JucType\x12\x0b\n\x07UNKNOWN\x10\x00\x12\x0b\n\x07IN_ROAD\x10\x01\x12\x0e\n\nCROSS_ROAD\x10\x02\x12\r\n\tFORK_ROAD\x10\x03\x12\r\n\tMAIN_SIDE\x10\x04\x12\x0c\n\x08\x44\x45\x41\x44_END\x10\x05'
    ),
    dependencies=[
        modules_dot_canbus_dot_proto_dot_chassis__pb2.DESCRIPTOR,
        modules_dot_common_dot_proto_dot_drive__state__pb2.DESCRIPTOR,
        modules_dot_common_dot_proto_dot_geometry__pb2.DESCRIPTOR,
        modules_dot_common_dot_proto_dot_header__pb2.DESCRIPTOR,
        modules_dot_common_dot_proto_dot_pnc__point__pb2.DESCRIPTOR,
        modules_dot_map_dot_proto_dot_map__id__pb2.DESCRIPTOR,
        modules_dot_planning_dot_proto_dot_decision__pb2.DESCRIPTOR,
        modules_dot_planning_dot_proto_dot_planning__internal__pb2.DESCRIPTOR,
    ],
)

_JUCTYPE = _descriptor.EnumDescriptor(
    name="JucType",
    full_name="apollo.planning.JucType",
    filename=None,
    file=DESCRIPTOR,
    values=[
        _descriptor.EnumValueDescriptor(
            name="UNKNOWN", index=0, number=0, serialized_options=None, type=None
        ),
        _descriptor.EnumValueDescriptor(
            name="IN_ROAD", index=1, number=1, serialized_options=None, type=None
        ),
        _descriptor.EnumValueDescriptor(
            name="CROSS_ROAD", index=2, number=2, serialized_options=None, type=None
        ),
        _descriptor.EnumValueDescriptor(
            name="FORK_ROAD", index=3, number=3, serialized_options=None, type=None
        ),
        _descriptor.EnumValueDescriptor(
            name="MAIN_SIDE", index=4, number=4, serialized_options=None, type=None
        ),
        _descriptor.EnumValueDescriptor(
            name="DEAD_END", index=5, number=5, serialized_options=None, type=None
        ),
    ],
    containing_type=None,
    serialized_options=None,
    serialized_start=2045,
    serialized_end=2140,
)
_sym_db.RegisterEnumDescriptor(_JUCTYPE)

JucType = enum_type_wrapper.EnumTypeWrapper(_JUCTYPE)
UNKNOWN = 0
IN_ROAD = 1
CROSS_ROAD = 2
FORK_ROAD = 3
MAIN_SIDE = 4
DEAD_END = 5


_ADCTRAJECTORY_RIGHTOFWAYSTATUS = _descriptor.EnumDescriptor(
    name="RightOfWayStatus",
    full_name="apollo.planning.ADCTrajectory.RightOfWayStatus",
    filename=None,
    file=DESCRIPTOR,
    values=[
        _descriptor.EnumValueDescriptor(
            name="UNPROTECTED", index=0, number=0, serialized_options=None, type=None
        ),
        _descriptor.EnumValueDescriptor(
            name="PROTECTED", index=1, number=1, serialized_options=None, type=None
        ),
    ],
    containing_type=None,
    serialized_options=None,
    serialized_start=1894,
    serialized_end=1944,
)
_sym_db.RegisterEnumDescriptor(_ADCTRAJECTORY_RIGHTOFWAYSTATUS)

_ADCTRAJECTORY_TRAJECTORYTYPE = _descriptor.EnumDescriptor(
    name="TrajectoryType",
    full_name="apollo.planning.ADCTrajectory.TrajectoryType",
    filename=None,
    file=DESCRIPTOR,
    values=[
        _descriptor.EnumValueDescriptor(
            name="UNKNOWN", index=0, number=0, serialized_options=None, type=None
        ),
        _descriptor.EnumValueDescriptor(
            name="NORMAL", index=1, number=1, serialized_options=None, type=None
        ),
        _descriptor.EnumValueDescriptor(
            name="PATH_FALLBACK", index=2, number=2, serialized_options=None, type=None
        ),
        _descriptor.EnumValueDescriptor(
            name="SPEED_FALLBACK", index=3, number=3, serialized_options=None, type=None
        ),
        _descriptor.EnumValueDescriptor(
            name="PATH_REUSED", index=4, number=4, serialized_options=None, type=None
        ),
    ],
    containing_type=None,
    serialized_options=None,
    serialized_start=1946,
    serialized_end=2043,
)
_sym_db.RegisterEnumDescriptor(_ADCTRAJECTORY_TRAJECTORYTYPE)


_ESTOP = _descriptor.Descriptor(
    name="EStop",
    full_name="apollo.planning.EStop",
    filename=None,
    file=DESCRIPTOR,
    containing_type=None,
    fields=[
        _descriptor.FieldDescriptor(
            name="is_estop",
            full_name="apollo.planning.EStop.is_estop",
            index=0,
            number=1,
            type=8,
            cpp_type=7,
            label=1,
            has_default_value=False,
            default_value=False,
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="reason",
            full_name="apollo.planning.EStop.reason",
            index=1,
            number=2,
            type=9,
            cpp_type=9,
            label=1,
            has_default_value=False,
            default_value=_b("").decode("utf-8"),
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
    ],
    extensions=[],
    nested_types=[],
    enum_types=[],
    serialized_options=None,
    is_extendable=False,
    syntax="proto2",
    extension_ranges=[],
    oneofs=[],
    serialized_start=363,
    serialized_end=404,
)


_TASKSTATS = _descriptor.Descriptor(
    name="TaskStats",
    full_name="apollo.planning.TaskStats",
    filename=None,
    file=DESCRIPTOR,
    containing_type=None,
    fields=[
        _descriptor.FieldDescriptor(
            name="name",
            full_name="apollo.planning.TaskStats.name",
            index=0,
            number=1,
            type=9,
            cpp_type=9,
            label=1,
            has_default_value=False,
            default_value=_b("").decode("utf-8"),
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="time_ms",
            full_name="apollo.planning.TaskStats.time_ms",
            index=1,
            number=2,
            type=1,
            cpp_type=5,
            label=1,
            has_default_value=False,
            default_value=float(0),
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
    ],
    extensions=[],
    nested_types=[],
    enum_types=[],
    serialized_options=None,
    is_extendable=False,
    syntax="proto2",
    extension_ranges=[],
    oneofs=[],
    serialized_start=406,
    serialized_end=448,
)


_LATENCYSTATS = _descriptor.Descriptor(
    name="LatencyStats",
    full_name="apollo.planning.LatencyStats",
    filename=None,
    file=DESCRIPTOR,
    containing_type=None,
    fields=[
        _descriptor.FieldDescriptor(
            name="total_time_ms",
            full_name="apollo.planning.LatencyStats.total_time_ms",
            index=0,
            number=1,
            type=1,
            cpp_type=5,
            label=1,
            has_default_value=False,
            default_value=float(0),
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="task_stats",
            full_name="apollo.planning.LatencyStats.task_stats",
            index=1,
            number=2,
            type=11,
            cpp_type=10,
            label=3,
            has_default_value=False,
            default_value=[],
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="init_frame_time_ms",
            full_name="apollo.planning.LatencyStats.init_frame_time_ms",
            index=2,
            number=3,
            type=1,
            cpp_type=5,
            label=1,
            has_default_value=False,
            default_value=float(0),
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
    ],
    extensions=[],
    nested_types=[],
    enum_types=[],
    serialized_options=None,
    is_extendable=False,
    syntax="proto2",
    extension_ranges=[],
    oneofs=[],
    serialized_start=450,
    serialized_end=563,
)


_RSSINFO = _descriptor.Descriptor(
    name="RSSInfo",
    full_name="apollo.planning.RSSInfo",
    filename=None,
    file=DESCRIPTOR,
    containing_type=None,
    fields=[
        _descriptor.FieldDescriptor(
            name="is_rss_safe",
            full_name="apollo.planning.RSSInfo.is_rss_safe",
            index=0,
            number=1,
            type=8,
            cpp_type=7,
            label=1,
            has_default_value=False,
            default_value=False,
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="cur_dist_lon",
            full_name="apollo.planning.RSSInfo.cur_dist_lon",
            index=1,
            number=2,
            type=1,
            cpp_type=5,
            label=1,
            has_default_value=False,
            default_value=float(0),
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="rss_safe_dist_lon",
            full_name="apollo.planning.RSSInfo.rss_safe_dist_lon",
            index=2,
            number=3,
            type=1,
            cpp_type=5,
            label=1,
            has_default_value=False,
            default_value=float(0),
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="acc_lon_range_minimum",
            full_name="apollo.planning.RSSInfo.acc_lon_range_minimum",
            index=3,
            number=4,
            type=1,
            cpp_type=5,
            label=1,
            has_default_value=False,
            default_value=float(0),
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="acc_lon_range_maximum",
            full_name="apollo.planning.RSSInfo.acc_lon_range_maximum",
            index=4,
            number=5,
            type=1,
            cpp_type=5,
            label=1,
            has_default_value=False,
            default_value=float(0),
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="acc_lat_left_range_minimum",
            full_name="apollo.planning.RSSInfo.acc_lat_left_range_minimum",
            index=5,
            number=6,
            type=1,
            cpp_type=5,
            label=1,
            has_default_value=False,
            default_value=float(0),
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="acc_lat_left_range_maximum",
            full_name="apollo.planning.RSSInfo.acc_lat_left_range_maximum",
            index=6,
            number=7,
            type=1,
            cpp_type=5,
            label=1,
            has_default_value=False,
            default_value=float(0),
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="acc_lat_right_range_minimum",
            full_name="apollo.planning.RSSInfo.acc_lat_right_range_minimum",
            index=7,
            number=8,
            type=1,
            cpp_type=5,
            label=1,
            has_default_value=False,
            default_value=float(0),
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="acc_lat_right_range_maximum",
            full_name="apollo.planning.RSSInfo.acc_lat_right_range_maximum",
            index=8,
            number=9,
            type=1,
            cpp_type=5,
            label=1,
            has_default_value=False,
            default_value=float(0),
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
    ],
    extensions=[],
    nested_types=[],
    enum_types=[],
    serialized_options=None,
    is_extendable=False,
    syntax="proto2",
    extension_ranges=[],
    oneofs=[],
    serialized_start=566,
    serialized_end=853,
)


_ADCTRAJECTORY_CRITICALREGION = _descriptor.Descriptor(
    name="CriticalRegion",
    full_name="apollo.planning.ADCTrajectory.CriticalRegion",
    filename=None,
    file=DESCRIPTOR,
    containing_type=None,
    fields=[
        _descriptor.FieldDescriptor(
            name="region",
            full_name="apollo.planning.ADCTrajectory.CriticalRegion.region",
            index=0,
            number=1,
            type=11,
            cpp_type=10,
            label=3,
            has_default_value=False,
            default_value=[],
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
    ],
    extensions=[],
    nested_types=[],
    enum_types=[],
    serialized_options=None,
    is_extendable=False,
    syntax="proto2",
    extension_ranges=[],
    oneofs=[],
    serialized_start=1836,
    serialized_end=1892,
)

_ADCTRAJECTORY = _descriptor.Descriptor(
    name="ADCTrajectory",
    full_name="apollo.planning.ADCTrajectory",
    filename=None,
    file=DESCRIPTOR,
    containing_type=None,
    fields=[
        _descriptor.FieldDescriptor(
            name="header",
            full_name="apollo.planning.ADCTrajectory.header",
            index=0,
            number=1,
            type=11,
            cpp_type=10,
            label=1,
            has_default_value=False,
            default_value=None,
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="total_path_length",
            full_name="apollo.planning.ADCTrajectory.total_path_length",
            index=1,
            number=2,
            type=1,
            cpp_type=5,
            label=1,
            has_default_value=False,
            default_value=float(0),
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="total_path_time",
            full_name="apollo.planning.ADCTrajectory.total_path_time",
            index=2,
            number=3,
            type=1,
            cpp_type=5,
            label=1,
            has_default_value=False,
            default_value=float(0),
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="estop",
            full_name="apollo.planning.ADCTrajectory.estop",
            index=3,
            number=6,
            type=11,
            cpp_type=10,
            label=1,
            has_default_value=False,
            default_value=None,
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="debug",
            full_name="apollo.planning.ADCTrajectory.debug",
            index=4,
            number=8,
            type=11,
            cpp_type=10,
            label=1,
            has_default_value=False,
            default_value=None,
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="is_replan",
            full_name="apollo.planning.ADCTrajectory.is_replan",
            index=5,
            number=9,
            type=8,
            cpp_type=7,
            label=1,
            has_default_value=True,
            default_value=False,
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="gear",
            full_name="apollo.planning.ADCTrajectory.gear",
            index=6,
            number=10,
            type=14,
            cpp_type=8,
            label=1,
            has_default_value=False,
            default_value=0,
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="trajectory_point",
            full_name="apollo.planning.ADCTrajectory.trajectory_point",
            index=7,
            number=12,
            type=11,
            cpp_type=10,
            label=3,
            has_default_value=False,
            default_value=[],
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="path_point",
            full_name="apollo.planning.ADCTrajectory.path_point",
            index=8,
            number=13,
            type=11,
            cpp_type=10,
            label=3,
            has_default_value=False,
            default_value=[],
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="decision",
            full_name="apollo.planning.ADCTrajectory.decision",
            index=9,
            number=14,
            type=11,
            cpp_type=10,
            label=1,
            has_default_value=False,
            default_value=None,
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="latency_stats",
            full_name="apollo.planning.ADCTrajectory.latency_stats",
            index=10,
            number=15,
            type=11,
            cpp_type=10,
            label=1,
            has_default_value=False,
            default_value=None,
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="routing_header",
            full_name="apollo.planning.ADCTrajectory.routing_header",
            index=11,
            number=16,
            type=11,
            cpp_type=10,
            label=1,
            has_default_value=False,
            default_value=None,
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="right_of_way_status",
            full_name="apollo.planning.ADCTrajectory.right_of_way_status",
            index=12,
            number=17,
            type=14,
            cpp_type=8,
            label=1,
            has_default_value=False,
            default_value=0,
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="lane_id",
            full_name="apollo.planning.ADCTrajectory.lane_id",
            index=13,
            number=18,
            type=11,
            cpp_type=10,
            label=3,
            has_default_value=False,
            default_value=[],
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="engage_advice",
            full_name="apollo.planning.ADCTrajectory.engage_advice",
            index=14,
            number=19,
            type=11,
            cpp_type=10,
            label=1,
            has_default_value=False,
            default_value=None,
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="critical_region",
            full_name="apollo.planning.ADCTrajectory.critical_region",
            index=15,
            number=20,
            type=11,
            cpp_type=10,
            label=1,
            has_default_value=False,
            default_value=None,
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="trajectory_type",
            full_name="apollo.planning.ADCTrajectory.trajectory_type",
            index=16,
            number=21,
            type=14,
            cpp_type=8,
            label=1,
            has_default_value=True,
            default_value=0,
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="replan_reason",
            full_name="apollo.planning.ADCTrajectory.replan_reason",
            index=17,
            number=22,
            type=9,
            cpp_type=9,
            label=1,
            has_default_value=False,
            default_value=_b("").decode("utf-8"),
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="target_lane_id",
            full_name="apollo.planning.ADCTrajectory.target_lane_id",
            index=18,
            number=23,
            type=11,
            cpp_type=10,
            label=3,
            has_default_value=False,
            default_value=[],
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="car_in_dead_end",
            full_name="apollo.planning.ADCTrajectory.car_in_dead_end",
            index=19,
            number=24,
            type=8,
            cpp_type=7,
            label=1,
            has_default_value=False,
            default_value=False,
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="rss_info",
            full_name="apollo.planning.ADCTrajectory.rss_info",
            index=20,
            number=100,
            type=11,
            cpp_type=10,
            label=1,
            has_default_value=False,
            default_value=None,
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
    ],
    extensions=[],
    nested_types=[
        _ADCTRAJECTORY_CRITICALREGION,
    ],
    enum_types=[
        _ADCTRAJECTORY_RIGHTOFWAYSTATUS,
        _ADCTRAJECTORY_TRAJECTORYTYPE,
    ],
    serialized_options=None,
    is_extendable=False,
    syntax="proto2",
    extension_ranges=[],
    oneofs=[],
    serialized_start=856,
    serialized_end=2043,
)

_LATENCYSTATS.fields_by_name["task_stats"].message_type = _TASKSTATS
_ADCTRAJECTORY_CRITICALREGION.fields_by_name[
    "region"
].message_type = modules_dot_common_dot_proto_dot_geometry__pb2._POLYGON
_ADCTRAJECTORY_CRITICALREGION.containing_type = _ADCTRAJECTORY
_ADCTRAJECTORY.fields_by_name[
    "header"
].message_type = modules_dot_common_dot_proto_dot_header__pb2._HEADER
_ADCTRAJECTORY.fields_by_name["estop"].message_type = _ESTOP
_ADCTRAJECTORY.fields_by_name[
    "debug"
].message_type = modules_dot_planning_dot_proto_dot_planning__internal__pb2._DEBUG
_ADCTRAJECTORY.fields_by_name[
    "gear"
].enum_type = modules_dot_canbus_dot_proto_dot_chassis__pb2._CHASSIS_GEARPOSITION
_ADCTRAJECTORY.fields_by_name[
    "trajectory_point"
].message_type = modules_dot_common_dot_proto_dot_pnc__point__pb2._TRAJECTORYPOINT
_ADCTRAJECTORY.fields_by_name[
    "path_point"
].message_type = modules_dot_common_dot_proto_dot_pnc__point__pb2._PATHPOINT
_ADCTRAJECTORY.fields_by_name[
    "decision"
].message_type = modules_dot_planning_dot_proto_dot_decision__pb2._DECISIONRESULT
_ADCTRAJECTORY.fields_by_name["latency_stats"].message_type = _LATENCYSTATS
_ADCTRAJECTORY.fields_by_name[
    "routing_header"
].message_type = modules_dot_common_dot_proto_dot_header__pb2._HEADER
_ADCTRAJECTORY.fields_by_name[
    "right_of_way_status"
].enum_type = _ADCTRAJECTORY_RIGHTOFWAYSTATUS
_ADCTRAJECTORY.fields_by_name[
    "lane_id"
].message_type = modules_dot_map_dot_proto_dot_map__id__pb2._ID
_ADCTRAJECTORY.fields_by_name[
    "engage_advice"
].message_type = modules_dot_common_dot_proto_dot_drive__state__pb2._ENGAGEADVICE
_ADCTRAJECTORY.fields_by_name[
    "critical_region"
].message_type = _ADCTRAJECTORY_CRITICALREGION
_ADCTRAJECTORY.fields_by_name[
    "trajectory_type"
].enum_type = _ADCTRAJECTORY_TRAJECTORYTYPE
_ADCTRAJECTORY.fields_by_name[
    "target_lane_id"
].message_type = modules_dot_map_dot_proto_dot_map__id__pb2._ID
_ADCTRAJECTORY.fields_by_name["rss_info"].message_type = _RSSINFO
_ADCTRAJECTORY_RIGHTOFWAYSTATUS.containing_type = _ADCTRAJECTORY
_ADCTRAJECTORY_TRAJECTORYTYPE.containing_type = _ADCTRAJECTORY
DESCRIPTOR.message_types_by_name["EStop"] = _ESTOP
DESCRIPTOR.message_types_by_name["TaskStats"] = _TASKSTATS
DESCRIPTOR.message_types_by_name["LatencyStats"] = _LATENCYSTATS
DESCRIPTOR.message_types_by_name["RSSInfo"] = _RSSINFO
DESCRIPTOR.message_types_by_name["ADCTrajectory"] = _ADCTRAJECTORY
DESCRIPTOR.enum_types_by_name["JucType"] = _JUCTYPE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

EStop = _reflection.GeneratedProtocolMessageType(
    "EStop",
    (_message.Message,),
    dict(
        DESCRIPTOR=_ESTOP,
        __module__="modules.planning.proto.planning_pb2"
        # @@protoc_insertion_point(class_scope:apollo.planning.EStop)
    ),
)
_sym_db.RegisterMessage(EStop)

TaskStats = _reflection.GeneratedProtocolMessageType(
    "TaskStats",
    (_message.Message,),
    dict(
        DESCRIPTOR=_TASKSTATS,
        __module__="modules.planning.proto.planning_pb2"
        # @@protoc_insertion_point(class_scope:apollo.planning.TaskStats)
    ),
)
_sym_db.RegisterMessage(TaskStats)

LatencyStats = _reflection.GeneratedProtocolMessageType(
    "LatencyStats",
    (_message.Message,),
    dict(
        DESCRIPTOR=_LATENCYSTATS,
        __module__="modules.planning.proto.planning_pb2"
        # @@protoc_insertion_point(class_scope:apollo.planning.LatencyStats)
    ),
)
_sym_db.RegisterMessage(LatencyStats)

RSSInfo = _reflection.GeneratedProtocolMessageType(
    "RSSInfo",
    (_message.Message,),
    dict(
        DESCRIPTOR=_RSSINFO,
        __module__="modules.planning.proto.planning_pb2"
        # @@protoc_insertion_point(class_scope:apollo.planning.RSSInfo)
    ),
)
_sym_db.RegisterMessage(RSSInfo)

ADCTrajectory = _reflection.GeneratedProtocolMessageType(
    "ADCTrajectory",
    (_message.Message,),
    dict(
        CriticalRegion=_reflection.GeneratedProtocolMessageType(
            "CriticalRegion",
            (_message.Message,),
            dict(
                DESCRIPTOR=_ADCTRAJECTORY_CRITICALREGION,
                __module__="modules.planning.proto.planning_pb2"
                # @@protoc_insertion_point(class_scope:apollo.planning.ADCTrajectory.CriticalRegion)
            ),
        ),
        DESCRIPTOR=_ADCTRAJECTORY,
        __module__="modules.planning.proto.planning_pb2"
        # @@protoc_insertion_point(class_scope:apollo.planning.ADCTrajectory)
    ),
)
_sym_db.RegisterMessage(ADCTrajectory)
_sym_db.RegisterMessage(ADCTrajectory.CriticalRegion)


# @@protoc_insertion_point(module_scope)
