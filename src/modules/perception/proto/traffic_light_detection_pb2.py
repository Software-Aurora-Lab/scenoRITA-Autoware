# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/perception/proto/traffic_light_detection.proto

import sys

_b = sys.version_info[0] < 3 and (lambda x: x) or (lambda x: x.encode("latin1"))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database

# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from modules.common.proto import (
    header_pb2 as modules_dot_common_dot_proto_dot_header__pb2,
)

DESCRIPTOR = _descriptor.FileDescriptor(
    name="modules/perception/proto/traffic_light_detection.proto",
    package="apollo.perception",
    syntax="proto2",
    serialized_options=None,
    serialized_pb=_b(
        '\n6modules/perception/proto/traffic_light_detection.proto\x12\x11\x61pollo.perception\x1a!modules/common/proto/header.proto"\xa3\x01\n\x0fTrafficLightBox\x12\t\n\x01x\x18\x01 \x01(\x05\x12\t\n\x01y\x18\x02 \x01(\x05\x12\r\n\x05width\x18\x03 \x01(\x05\x12\x0e\n\x06height\x18\x04 \x01(\x05\x12\x34\n\x05\x63olor\x18\x05 \x01(\x0e\x32%.apollo.perception.TrafficLight.Color\x12\x10\n\x08selected\x18\x06 \x01(\x08\x12\x13\n\x0b\x63\x61mera_name\x18\x07 \x01(\t"\xfa\x03\n\x11TrafficLightDebug\x12\x33\n\x07\x63ropbox\x18\x01 \x01(\x0b\x32".apollo.perception.TrafficLightBox\x12/\n\x03\x62ox\x18\x02 \x03(\x0b\x32".apollo.perception.TrafficLightBox\x12\x12\n\nsignal_num\x18\x03 \x01(\x05\x12\x11\n\tvalid_pos\x18\x04 \x01(\x05\x12\x13\n\x0bts_diff_pos\x18\x05 \x01(\x01\x12\x13\n\x0bts_diff_sys\x18\x06 \x01(\x01\x12\x15\n\rproject_error\x18\x07 \x01(\x05\x12\x1d\n\x15\x64istance_to_stop_line\x18\x08 \x01(\x01\x12\x15\n\tcamera_id\x18\t \x01(\x05\x42\x02\x18\x01\x12\x34\n\x08\x63rop_roi\x18\n \x03(\x0b\x32".apollo.perception.TrafficLightBox\x12\x39\n\rprojected_roi\x18\x0b \x03(\x0b\x32".apollo.perception.TrafficLightBox\x12\x39\n\rrectified_roi\x18\x0c \x03(\x0b\x32".apollo.perception.TrafficLightBox\x12\x35\n\tdebug_roi\x18\r \x03(\x0b\x32".apollo.perception.TrafficLightBox"\xe6\x01\n\x0cTrafficLight\x12\x34\n\x05\x63olor\x18\x01 \x01(\x0e\x32%.apollo.perception.TrafficLight.Color\x12\n\n\x02id\x18\x02 \x01(\t\x12\x15\n\nconfidence\x18\x03 \x01(\x01:\x01\x31\x12\x15\n\rtracking_time\x18\x04 \x01(\x01\x12\r\n\x05\x62link\x18\x05 \x01(\x08\x12\x16\n\x0eremaining_time\x18\x06 \x01(\x01"?\n\x05\x43olor\x12\x0b\n\x07UNKNOWN\x10\x00\x12\x07\n\x03RED\x10\x01\x12\n\n\x06YELLOW\x10\x02\x12\t\n\x05GREEN\x10\x03\x12\t\n\x05\x42LACK\x10\x04"\x82\x03\n\x15TrafficLightDetection\x12%\n\x06header\x18\x02 \x01(\x0b\x32\x15.apollo.common.Header\x12\x36\n\rtraffic_light\x18\x01 \x03(\x0b\x32\x1f.apollo.perception.TrafficLight\x12\x41\n\x13traffic_light_debug\x18\x03 \x01(\x0b\x32$.apollo.perception.TrafficLightDebug\x12\x16\n\x0e\x63ontain_lights\x18\x04 \x01(\x08\x12\x44\n\tcamera_id\x18\x05 \x01(\x0e\x32\x31.apollo.perception.TrafficLightDetection.CameraID"i\n\x08\x43\x61meraID\x12\x15\n\x11\x43\x41MERA_FRONT_LONG\x10\x00\x12\x17\n\x13\x43\x41MERA_FRONT_NARROW\x10\x01\x12\x16\n\x12\x43\x41MERA_FRONT_SHORT\x10\x02\x12\x15\n\x11\x43\x41MERA_FRONT_WIDE\x10\x03'
    ),
    dependencies=[
        modules_dot_common_dot_proto_dot_header__pb2.DESCRIPTOR,
    ],
)


_TRAFFICLIGHT_COLOR = _descriptor.EnumDescriptor(
    name="Color",
    full_name="apollo.perception.TrafficLight.Color",
    filename=None,
    file=DESCRIPTOR,
    values=[
        _descriptor.EnumValueDescriptor(
            name="UNKNOWN", index=0, number=0, serialized_options=None, type=None
        ),
        _descriptor.EnumValueDescriptor(
            name="RED", index=1, number=1, serialized_options=None, type=None
        ),
        _descriptor.EnumValueDescriptor(
            name="YELLOW", index=2, number=2, serialized_options=None, type=None
        ),
        _descriptor.EnumValueDescriptor(
            name="GREEN", index=3, number=3, serialized_options=None, type=None
        ),
        _descriptor.EnumValueDescriptor(
            name="BLACK", index=4, number=4, serialized_options=None, type=None
        ),
    ],
    containing_type=None,
    serialized_options=None,
    serialized_start=955,
    serialized_end=1018,
)
_sym_db.RegisterEnumDescriptor(_TRAFFICLIGHT_COLOR)

_TRAFFICLIGHTDETECTION_CAMERAID = _descriptor.EnumDescriptor(
    name="CameraID",
    full_name="apollo.perception.TrafficLightDetection.CameraID",
    filename=None,
    file=DESCRIPTOR,
    values=[
        _descriptor.EnumValueDescriptor(
            name="CAMERA_FRONT_LONG",
            index=0,
            number=0,
            serialized_options=None,
            type=None,
        ),
        _descriptor.EnumValueDescriptor(
            name="CAMERA_FRONT_NARROW",
            index=1,
            number=1,
            serialized_options=None,
            type=None,
        ),
        _descriptor.EnumValueDescriptor(
            name="CAMERA_FRONT_SHORT",
            index=2,
            number=2,
            serialized_options=None,
            type=None,
        ),
        _descriptor.EnumValueDescriptor(
            name="CAMERA_FRONT_WIDE",
            index=3,
            number=3,
            serialized_options=None,
            type=None,
        ),
    ],
    containing_type=None,
    serialized_options=None,
    serialized_start=1302,
    serialized_end=1407,
)
_sym_db.RegisterEnumDescriptor(_TRAFFICLIGHTDETECTION_CAMERAID)


_TRAFFICLIGHTBOX = _descriptor.Descriptor(
    name="TrafficLightBox",
    full_name="apollo.perception.TrafficLightBox",
    filename=None,
    file=DESCRIPTOR,
    containing_type=None,
    fields=[
        _descriptor.FieldDescriptor(
            name="x",
            full_name="apollo.perception.TrafficLightBox.x",
            index=0,
            number=1,
            type=5,
            cpp_type=1,
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
            name="y",
            full_name="apollo.perception.TrafficLightBox.y",
            index=1,
            number=2,
            type=5,
            cpp_type=1,
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
            name="width",
            full_name="apollo.perception.TrafficLightBox.width",
            index=2,
            number=3,
            type=5,
            cpp_type=1,
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
            name="height",
            full_name="apollo.perception.TrafficLightBox.height",
            index=3,
            number=4,
            type=5,
            cpp_type=1,
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
            name="color",
            full_name="apollo.perception.TrafficLightBox.color",
            index=4,
            number=5,
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
            name="selected",
            full_name="apollo.perception.TrafficLightBox.selected",
            index=5,
            number=6,
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
            name="camera_name",
            full_name="apollo.perception.TrafficLightBox.camera_name",
            index=6,
            number=7,
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
    serialized_start=113,
    serialized_end=276,
)


_TRAFFICLIGHTDEBUG = _descriptor.Descriptor(
    name="TrafficLightDebug",
    full_name="apollo.perception.TrafficLightDebug",
    filename=None,
    file=DESCRIPTOR,
    containing_type=None,
    fields=[
        _descriptor.FieldDescriptor(
            name="cropbox",
            full_name="apollo.perception.TrafficLightDebug.cropbox",
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
            name="box",
            full_name="apollo.perception.TrafficLightDebug.box",
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
            name="signal_num",
            full_name="apollo.perception.TrafficLightDebug.signal_num",
            index=2,
            number=3,
            type=5,
            cpp_type=1,
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
            name="valid_pos",
            full_name="apollo.perception.TrafficLightDebug.valid_pos",
            index=3,
            number=4,
            type=5,
            cpp_type=1,
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
            name="ts_diff_pos",
            full_name="apollo.perception.TrafficLightDebug.ts_diff_pos",
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
            name="ts_diff_sys",
            full_name="apollo.perception.TrafficLightDebug.ts_diff_sys",
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
            name="project_error",
            full_name="apollo.perception.TrafficLightDebug.project_error",
            index=6,
            number=7,
            type=5,
            cpp_type=1,
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
            name="distance_to_stop_line",
            full_name="apollo.perception.TrafficLightDebug.distance_to_stop_line",
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
            name="camera_id",
            full_name="apollo.perception.TrafficLightDebug.camera_id",
            index=8,
            number=9,
            type=5,
            cpp_type=1,
            label=1,
            has_default_value=False,
            default_value=0,
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=_b("\030\001"),
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="crop_roi",
            full_name="apollo.perception.TrafficLightDebug.crop_roi",
            index=9,
            number=10,
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
            name="projected_roi",
            full_name="apollo.perception.TrafficLightDebug.projected_roi",
            index=10,
            number=11,
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
            name="rectified_roi",
            full_name="apollo.perception.TrafficLightDebug.rectified_roi",
            index=11,
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
            name="debug_roi",
            full_name="apollo.perception.TrafficLightDebug.debug_roi",
            index=12,
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
    ],
    extensions=[],
    nested_types=[],
    enum_types=[],
    serialized_options=None,
    is_extendable=False,
    syntax="proto2",
    extension_ranges=[],
    oneofs=[],
    serialized_start=279,
    serialized_end=785,
)


_TRAFFICLIGHT = _descriptor.Descriptor(
    name="TrafficLight",
    full_name="apollo.perception.TrafficLight",
    filename=None,
    file=DESCRIPTOR,
    containing_type=None,
    fields=[
        _descriptor.FieldDescriptor(
            name="color",
            full_name="apollo.perception.TrafficLight.color",
            index=0,
            number=1,
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
            name="id",
            full_name="apollo.perception.TrafficLight.id",
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
        _descriptor.FieldDescriptor(
            name="confidence",
            full_name="apollo.perception.TrafficLight.confidence",
            index=2,
            number=3,
            type=1,
            cpp_type=5,
            label=1,
            has_default_value=True,
            default_value=float(1),
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="tracking_time",
            full_name="apollo.perception.TrafficLight.tracking_time",
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
            name="blink",
            full_name="apollo.perception.TrafficLight.blink",
            index=4,
            number=5,
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
            name="remaining_time",
            full_name="apollo.perception.TrafficLight.remaining_time",
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
    ],
    extensions=[],
    nested_types=[],
    enum_types=[
        _TRAFFICLIGHT_COLOR,
    ],
    serialized_options=None,
    is_extendable=False,
    syntax="proto2",
    extension_ranges=[],
    oneofs=[],
    serialized_start=788,
    serialized_end=1018,
)


_TRAFFICLIGHTDETECTION = _descriptor.Descriptor(
    name="TrafficLightDetection",
    full_name="apollo.perception.TrafficLightDetection",
    filename=None,
    file=DESCRIPTOR,
    containing_type=None,
    fields=[
        _descriptor.FieldDescriptor(
            name="header",
            full_name="apollo.perception.TrafficLightDetection.header",
            index=0,
            number=2,
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
            name="traffic_light",
            full_name="apollo.perception.TrafficLightDetection.traffic_light",
            index=1,
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
        _descriptor.FieldDescriptor(
            name="traffic_light_debug",
            full_name="apollo.perception.TrafficLightDetection.traffic_light_debug",
            index=2,
            number=3,
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
            name="contain_lights",
            full_name="apollo.perception.TrafficLightDetection.contain_lights",
            index=3,
            number=4,
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
            name="camera_id",
            full_name="apollo.perception.TrafficLightDetection.camera_id",
            index=4,
            number=5,
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
    ],
    extensions=[],
    nested_types=[],
    enum_types=[
        _TRAFFICLIGHTDETECTION_CAMERAID,
    ],
    serialized_options=None,
    is_extendable=False,
    syntax="proto2",
    extension_ranges=[],
    oneofs=[],
    serialized_start=1021,
    serialized_end=1407,
)

_TRAFFICLIGHTBOX.fields_by_name["color"].enum_type = _TRAFFICLIGHT_COLOR
_TRAFFICLIGHTDEBUG.fields_by_name["cropbox"].message_type = _TRAFFICLIGHTBOX
_TRAFFICLIGHTDEBUG.fields_by_name["box"].message_type = _TRAFFICLIGHTBOX
_TRAFFICLIGHTDEBUG.fields_by_name["crop_roi"].message_type = _TRAFFICLIGHTBOX
_TRAFFICLIGHTDEBUG.fields_by_name["projected_roi"].message_type = _TRAFFICLIGHTBOX
_TRAFFICLIGHTDEBUG.fields_by_name["rectified_roi"].message_type = _TRAFFICLIGHTBOX
_TRAFFICLIGHTDEBUG.fields_by_name["debug_roi"].message_type = _TRAFFICLIGHTBOX
_TRAFFICLIGHT.fields_by_name["color"].enum_type = _TRAFFICLIGHT_COLOR
_TRAFFICLIGHT_COLOR.containing_type = _TRAFFICLIGHT
_TRAFFICLIGHTDETECTION.fields_by_name[
    "header"
].message_type = modules_dot_common_dot_proto_dot_header__pb2._HEADER
_TRAFFICLIGHTDETECTION.fields_by_name["traffic_light"].message_type = _TRAFFICLIGHT
_TRAFFICLIGHTDETECTION.fields_by_name[
    "traffic_light_debug"
].message_type = _TRAFFICLIGHTDEBUG
_TRAFFICLIGHTDETECTION.fields_by_name[
    "camera_id"
].enum_type = _TRAFFICLIGHTDETECTION_CAMERAID
_TRAFFICLIGHTDETECTION_CAMERAID.containing_type = _TRAFFICLIGHTDETECTION
DESCRIPTOR.message_types_by_name["TrafficLightBox"] = _TRAFFICLIGHTBOX
DESCRIPTOR.message_types_by_name["TrafficLightDebug"] = _TRAFFICLIGHTDEBUG
DESCRIPTOR.message_types_by_name["TrafficLight"] = _TRAFFICLIGHT
DESCRIPTOR.message_types_by_name["TrafficLightDetection"] = _TRAFFICLIGHTDETECTION
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

TrafficLightBox = _reflection.GeneratedProtocolMessageType(
    "TrafficLightBox",
    (_message.Message,),
    dict(
        DESCRIPTOR=_TRAFFICLIGHTBOX,
        __module__="modules.perception.proto.traffic_light_detection_pb2"
        # @@protoc_insertion_point(class_scope:apollo.perception.TrafficLightBox)
    ),
)
_sym_db.RegisterMessage(TrafficLightBox)

TrafficLightDebug = _reflection.GeneratedProtocolMessageType(
    "TrafficLightDebug",
    (_message.Message,),
    dict(
        DESCRIPTOR=_TRAFFICLIGHTDEBUG,
        __module__="modules.perception.proto.traffic_light_detection_pb2"
        # @@protoc_insertion_point(class_scope:apollo.perception.TrafficLightDebug)
    ),
)
_sym_db.RegisterMessage(TrafficLightDebug)

TrafficLight = _reflection.GeneratedProtocolMessageType(
    "TrafficLight",
    (_message.Message,),
    dict(
        DESCRIPTOR=_TRAFFICLIGHT,
        __module__="modules.perception.proto.traffic_light_detection_pb2"
        # @@protoc_insertion_point(class_scope:apollo.perception.TrafficLight)
    ),
)
_sym_db.RegisterMessage(TrafficLight)

TrafficLightDetection = _reflection.GeneratedProtocolMessageType(
    "TrafficLightDetection",
    (_message.Message,),
    dict(
        DESCRIPTOR=_TRAFFICLIGHTDETECTION,
        __module__="modules.perception.proto.traffic_light_detection_pb2"
        # @@protoc_insertion_point(class_scope:apollo.perception.TrafficLightDetection)
    ),
)
_sym_db.RegisterMessage(TrafficLightDetection)


_TRAFFICLIGHTDEBUG.fields_by_name["camera_id"]._options = None
# @@protoc_insertion_point(module_scope)
