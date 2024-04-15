# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/map/proto/map_signal.proto

import sys

_b = sys.version_info[0] < 3 and (lambda x: x) or (lambda x: x.encode("latin1"))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database

# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from modules.common.proto import (
    geometry_pb2 as modules_dot_common_dot_proto_dot_geometry__pb2,
)
from modules.map.proto import (
    map_geometry_pb2 as modules_dot_map_dot_proto_dot_map__geometry__pb2,
)
from modules.map.proto import map_id_pb2 as modules_dot_map_dot_proto_dot_map__id__pb2

DESCRIPTOR = _descriptor.FileDescriptor(
    name="modules/map/proto/map_signal.proto",
    package="apollo.hdmap",
    syntax="proto2",
    serialized_options=None,
    serialized_pb=_b(
        '\n"modules/map/proto/map_signal.proto\x12\x0c\x61pollo.hdmap\x1a#modules/common/proto/geometry.proto\x1a$modules/map/proto/map_geometry.proto\x1a\x1emodules/map/proto/map_id.proto"\xa1\x02\n\tSubsignal\x12\x1c\n\x02id\x18\x01 \x01(\x0b\x32\x10.apollo.hdmap.Id\x12*\n\x04type\x18\x02 \x01(\x0e\x32\x1c.apollo.hdmap.Subsignal.Type\x12)\n\x08location\x18\x03 \x01(\x0b\x32\x17.apollo.common.PointENU"\x9e\x01\n\x04Type\x12\x0b\n\x07UNKNOWN\x10\x01\x12\n\n\x06\x43IRCLE\x10\x02\x12\x0e\n\nARROW_LEFT\x10\x03\x12\x11\n\rARROW_FORWARD\x10\x04\x12\x0f\n\x0b\x41RROW_RIGHT\x10\x05\x12\x1a\n\x16\x41RROW_LEFT_AND_FORWARD\x10\x06\x12\x1b\n\x17\x41RROW_RIGHT_AND_FORWARD\x10\x07\x12\x10\n\x0c\x41RROW_U_TURN\x10\x08"a\n\x08SignInfo\x12)\n\x04type\x18\x01 \x01(\x0e\x32\x1b.apollo.hdmap.SignInfo.Type"*\n\x04Type\x12\x08\n\x04None\x10\x00\x12\x18\n\x14NO_RIGHT_TURN_ON_RED\x10\x01"\x92\x03\n\x06Signal\x12\x1c\n\x02id\x18\x01 \x01(\x0b\x32\x10.apollo.hdmap.Id\x12\'\n\x08\x62oundary\x18\x02 \x01(\x0b\x32\x15.apollo.hdmap.Polygon\x12*\n\tsubsignal\x18\x03 \x03(\x0b\x32\x17.apollo.hdmap.Subsignal\x12$\n\noverlap_id\x18\x04 \x03(\x0b\x32\x10.apollo.hdmap.Id\x12\'\n\x04type\x18\x05 \x01(\x0e\x32\x19.apollo.hdmap.Signal.Type\x12&\n\tstop_line\x18\x06 \x03(\x0b\x32\x13.apollo.hdmap.Curve\x12)\n\tsign_info\x18\x07 \x03(\x0b\x32\x16.apollo.hdmap.SignInfo"s\n\x04Type\x12\x0b\n\x07UNKNOWN\x10\x01\x12\x14\n\x10MIX_2_HORIZONTAL\x10\x02\x12\x12\n\x0eMIX_2_VERTICAL\x10\x03\x12\x14\n\x10MIX_3_HORIZONTAL\x10\x04\x12\x12\n\x0eMIX_3_VERTICAL\x10\x05\x12\n\n\x06SINGLE\x10\x06'
    ),
    dependencies=[
        modules_dot_common_dot_proto_dot_geometry__pb2.DESCRIPTOR,
        modules_dot_map_dot_proto_dot_map__geometry__pb2.DESCRIPTOR,
        modules_dot_map_dot_proto_dot_map__id__pb2.DESCRIPTOR,
    ],
)


_SUBSIGNAL_TYPE = _descriptor.EnumDescriptor(
    name="Type",
    full_name="apollo.hdmap.Subsignal.Type",
    filename=None,
    file=DESCRIPTOR,
    values=[
        _descriptor.EnumValueDescriptor(
            name="UNKNOWN", index=0, number=1, serialized_options=None, type=None
        ),
        _descriptor.EnumValueDescriptor(
            name="CIRCLE", index=1, number=2, serialized_options=None, type=None
        ),
        _descriptor.EnumValueDescriptor(
            name="ARROW_LEFT", index=2, number=3, serialized_options=None, type=None
        ),
        _descriptor.EnumValueDescriptor(
            name="ARROW_FORWARD", index=3, number=4, serialized_options=None, type=None
        ),
        _descriptor.EnumValueDescriptor(
            name="ARROW_RIGHT", index=4, number=5, serialized_options=None, type=None
        ),
        _descriptor.EnumValueDescriptor(
            name="ARROW_LEFT_AND_FORWARD",
            index=5,
            number=6,
            serialized_options=None,
            type=None,
        ),
        _descriptor.EnumValueDescriptor(
            name="ARROW_RIGHT_AND_FORWARD",
            index=6,
            number=7,
            serialized_options=None,
            type=None,
        ),
        _descriptor.EnumValueDescriptor(
            name="ARROW_U_TURN", index=7, number=8, serialized_options=None, type=None
        ),
    ],
    containing_type=None,
    serialized_options=None,
    serialized_start=291,
    serialized_end=449,
)
_sym_db.RegisterEnumDescriptor(_SUBSIGNAL_TYPE)

_SIGNINFO_TYPE = _descriptor.EnumDescriptor(
    name="Type",
    full_name="apollo.hdmap.SignInfo.Type",
    filename=None,
    file=DESCRIPTOR,
    values=[
        _descriptor.EnumValueDescriptor(
            name="None", index=0, number=0, serialized_options=None, type=None
        ),
        _descriptor.EnumValueDescriptor(
            name="NO_RIGHT_TURN_ON_RED",
            index=1,
            number=1,
            serialized_options=None,
            type=None,
        ),
    ],
    containing_type=None,
    serialized_options=None,
    serialized_start=506,
    serialized_end=548,
)
_sym_db.RegisterEnumDescriptor(_SIGNINFO_TYPE)

_SIGNAL_TYPE = _descriptor.EnumDescriptor(
    name="Type",
    full_name="apollo.hdmap.Signal.Type",
    filename=None,
    file=DESCRIPTOR,
    values=[
        _descriptor.EnumValueDescriptor(
            name="UNKNOWN", index=0, number=1, serialized_options=None, type=None
        ),
        _descriptor.EnumValueDescriptor(
            name="MIX_2_HORIZONTAL",
            index=1,
            number=2,
            serialized_options=None,
            type=None,
        ),
        _descriptor.EnumValueDescriptor(
            name="MIX_2_VERTICAL", index=2, number=3, serialized_options=None, type=None
        ),
        _descriptor.EnumValueDescriptor(
            name="MIX_3_HORIZONTAL",
            index=3,
            number=4,
            serialized_options=None,
            type=None,
        ),
        _descriptor.EnumValueDescriptor(
            name="MIX_3_VERTICAL", index=4, number=5, serialized_options=None, type=None
        ),
        _descriptor.EnumValueDescriptor(
            name="SINGLE", index=5, number=6, serialized_options=None, type=None
        ),
    ],
    containing_type=None,
    serialized_options=None,
    serialized_start=838,
    serialized_end=953,
)
_sym_db.RegisterEnumDescriptor(_SIGNAL_TYPE)


_SUBSIGNAL = _descriptor.Descriptor(
    name="Subsignal",
    full_name="apollo.hdmap.Subsignal",
    filename=None,
    file=DESCRIPTOR,
    containing_type=None,
    fields=[
        _descriptor.FieldDescriptor(
            name="id",
            full_name="apollo.hdmap.Subsignal.id",
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
            name="type",
            full_name="apollo.hdmap.Subsignal.type",
            index=1,
            number=2,
            type=14,
            cpp_type=8,
            label=1,
            has_default_value=False,
            default_value=1,
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="location",
            full_name="apollo.hdmap.Subsignal.location",
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
    ],
    extensions=[],
    nested_types=[],
    enum_types=[
        _SUBSIGNAL_TYPE,
    ],
    serialized_options=None,
    is_extendable=False,
    syntax="proto2",
    extension_ranges=[],
    oneofs=[],
    serialized_start=160,
    serialized_end=449,
)


_SIGNINFO = _descriptor.Descriptor(
    name="SignInfo",
    full_name="apollo.hdmap.SignInfo",
    filename=None,
    file=DESCRIPTOR,
    containing_type=None,
    fields=[
        _descriptor.FieldDescriptor(
            name="type",
            full_name="apollo.hdmap.SignInfo.type",
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
    ],
    extensions=[],
    nested_types=[],
    enum_types=[
        _SIGNINFO_TYPE,
    ],
    serialized_options=None,
    is_extendable=False,
    syntax="proto2",
    extension_ranges=[],
    oneofs=[],
    serialized_start=451,
    serialized_end=548,
)


_SIGNAL = _descriptor.Descriptor(
    name="Signal",
    full_name="apollo.hdmap.Signal",
    filename=None,
    file=DESCRIPTOR,
    containing_type=None,
    fields=[
        _descriptor.FieldDescriptor(
            name="id",
            full_name="apollo.hdmap.Signal.id",
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
            name="boundary",
            full_name="apollo.hdmap.Signal.boundary",
            index=1,
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
            name="subsignal",
            full_name="apollo.hdmap.Signal.subsignal",
            index=2,
            number=3,
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
            name="overlap_id",
            full_name="apollo.hdmap.Signal.overlap_id",
            index=3,
            number=4,
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
            name="type",
            full_name="apollo.hdmap.Signal.type",
            index=4,
            number=5,
            type=14,
            cpp_type=8,
            label=1,
            has_default_value=False,
            default_value=1,
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="stop_line",
            full_name="apollo.hdmap.Signal.stop_line",
            index=5,
            number=6,
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
            name="sign_info",
            full_name="apollo.hdmap.Signal.sign_info",
            index=6,
            number=7,
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
    enum_types=[
        _SIGNAL_TYPE,
    ],
    serialized_options=None,
    is_extendable=False,
    syntax="proto2",
    extension_ranges=[],
    oneofs=[],
    serialized_start=551,
    serialized_end=953,
)

_SUBSIGNAL.fields_by_name[
    "id"
].message_type = modules_dot_map_dot_proto_dot_map__id__pb2._ID
_SUBSIGNAL.fields_by_name["type"].enum_type = _SUBSIGNAL_TYPE
_SUBSIGNAL.fields_by_name[
    "location"
].message_type = modules_dot_common_dot_proto_dot_geometry__pb2._POINTENU
_SUBSIGNAL_TYPE.containing_type = _SUBSIGNAL
_SIGNINFO.fields_by_name["type"].enum_type = _SIGNINFO_TYPE
_SIGNINFO_TYPE.containing_type = _SIGNINFO
_SIGNAL.fields_by_name[
    "id"
].message_type = modules_dot_map_dot_proto_dot_map__id__pb2._ID
_SIGNAL.fields_by_name[
    "boundary"
].message_type = modules_dot_map_dot_proto_dot_map__geometry__pb2._POLYGON
_SIGNAL.fields_by_name["subsignal"].message_type = _SUBSIGNAL
_SIGNAL.fields_by_name[
    "overlap_id"
].message_type = modules_dot_map_dot_proto_dot_map__id__pb2._ID
_SIGNAL.fields_by_name["type"].enum_type = _SIGNAL_TYPE
_SIGNAL.fields_by_name[
    "stop_line"
].message_type = modules_dot_map_dot_proto_dot_map__geometry__pb2._CURVE
_SIGNAL.fields_by_name["sign_info"].message_type = _SIGNINFO
_SIGNAL_TYPE.containing_type = _SIGNAL
DESCRIPTOR.message_types_by_name["Subsignal"] = _SUBSIGNAL
DESCRIPTOR.message_types_by_name["SignInfo"] = _SIGNINFO
DESCRIPTOR.message_types_by_name["Signal"] = _SIGNAL
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

Subsignal = _reflection.GeneratedProtocolMessageType(
    "Subsignal",
    (_message.Message,),
    dict(
        DESCRIPTOR=_SUBSIGNAL,
        __module__="modules.map.proto.map_signal_pb2"
        # @@protoc_insertion_point(class_scope:apollo.hdmap.Subsignal)
    ),
)
_sym_db.RegisterMessage(Subsignal)

SignInfo = _reflection.GeneratedProtocolMessageType(
    "SignInfo",
    (_message.Message,),
    dict(
        DESCRIPTOR=_SIGNINFO,
        __module__="modules.map.proto.map_signal_pb2"
        # @@protoc_insertion_point(class_scope:apollo.hdmap.SignInfo)
    ),
)
_sym_db.RegisterMessage(SignInfo)

Signal = _reflection.GeneratedProtocolMessageType(
    "Signal",
    (_message.Message,),
    dict(
        DESCRIPTOR=_SIGNAL,
        __module__="modules.map.proto.map_signal_pb2"
        # @@protoc_insertion_point(class_scope:apollo.hdmap.Signal)
    ),
)
_sym_db.RegisterMessage(Signal)


# @@protoc_insertion_point(module_scope)