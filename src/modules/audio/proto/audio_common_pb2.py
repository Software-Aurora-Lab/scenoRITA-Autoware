# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/audio/proto/audio_common.proto

import sys

_b = sys.version_info[0] < 3 and (lambda x: x) or (lambda x: x.encode("latin1"))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf.internal import enum_type_wrapper

# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


DESCRIPTOR = _descriptor.FileDescriptor(
    name="modules/audio/proto/audio_common.proto",
    package="apollo.audio",
    syntax="proto2",
    serialized_options=None,
    serialized_pb=_b(
        "\n&modules/audio/proto/audio_common.proto\x12\x0c\x61pollo.audio*K\n\x0cMovingResult\x12\x0b\n\x07UNKNOWN\x10\x00\x12\x0f\n\x0b\x41PPROACHING\x10\x01\x12\r\n\tDEPARTING\x10\x02\x12\x0e\n\nSTATIONARY\x10\x03*G\n\tAudioType\x12\x10\n\x0cUNKNOWN_TYPE\x10\x00\x12\n\n\x06POLICE\x10\x01\x12\r\n\tAMBULANCE\x10\x02\x12\r\n\tFIRETRUCK\x10\x03*Q\n\x0e\x41udioDirection\x12\x15\n\x11UNKNOWN_DIRECTION\x10\x00\x12\t\n\x05\x46RONT\x10\x01\x12\x08\n\x04LEFT\x10\x02\x12\x08\n\x04\x42\x41\x43K\x10\x03\x12\t\n\x05RIGHT\x10\x04"
    ),
)

_MOVINGRESULT = _descriptor.EnumDescriptor(
    name="MovingResult",
    full_name="apollo.audio.MovingResult",
    filename=None,
    file=DESCRIPTOR,
    values=[
        _descriptor.EnumValueDescriptor(
            name="UNKNOWN", index=0, number=0, serialized_options=None, type=None
        ),
        _descriptor.EnumValueDescriptor(
            name="APPROACHING", index=1, number=1, serialized_options=None, type=None
        ),
        _descriptor.EnumValueDescriptor(
            name="DEPARTING", index=2, number=2, serialized_options=None, type=None
        ),
        _descriptor.EnumValueDescriptor(
            name="STATIONARY", index=3, number=3, serialized_options=None, type=None
        ),
    ],
    containing_type=None,
    serialized_options=None,
    serialized_start=56,
    serialized_end=131,
)
_sym_db.RegisterEnumDescriptor(_MOVINGRESULT)

MovingResult = enum_type_wrapper.EnumTypeWrapper(_MOVINGRESULT)
_AUDIOTYPE = _descriptor.EnumDescriptor(
    name="AudioType",
    full_name="apollo.audio.AudioType",
    filename=None,
    file=DESCRIPTOR,
    values=[
        _descriptor.EnumValueDescriptor(
            name="UNKNOWN_TYPE", index=0, number=0, serialized_options=None, type=None
        ),
        _descriptor.EnumValueDescriptor(
            name="POLICE", index=1, number=1, serialized_options=None, type=None
        ),
        _descriptor.EnumValueDescriptor(
            name="AMBULANCE", index=2, number=2, serialized_options=None, type=None
        ),
        _descriptor.EnumValueDescriptor(
            name="FIRETRUCK", index=3, number=3, serialized_options=None, type=None
        ),
    ],
    containing_type=None,
    serialized_options=None,
    serialized_start=133,
    serialized_end=204,
)
_sym_db.RegisterEnumDescriptor(_AUDIOTYPE)

AudioType = enum_type_wrapper.EnumTypeWrapper(_AUDIOTYPE)
_AUDIODIRECTION = _descriptor.EnumDescriptor(
    name="AudioDirection",
    full_name="apollo.audio.AudioDirection",
    filename=None,
    file=DESCRIPTOR,
    values=[
        _descriptor.EnumValueDescriptor(
            name="UNKNOWN_DIRECTION",
            index=0,
            number=0,
            serialized_options=None,
            type=None,
        ),
        _descriptor.EnumValueDescriptor(
            name="FRONT", index=1, number=1, serialized_options=None, type=None
        ),
        _descriptor.EnumValueDescriptor(
            name="LEFT", index=2, number=2, serialized_options=None, type=None
        ),
        _descriptor.EnumValueDescriptor(
            name="BACK", index=3, number=3, serialized_options=None, type=None
        ),
        _descriptor.EnumValueDescriptor(
            name="RIGHT", index=4, number=4, serialized_options=None, type=None
        ),
    ],
    containing_type=None,
    serialized_options=None,
    serialized_start=206,
    serialized_end=287,
)
_sym_db.RegisterEnumDescriptor(_AUDIODIRECTION)

AudioDirection = enum_type_wrapper.EnumTypeWrapper(_AUDIODIRECTION)
UNKNOWN = 0
APPROACHING = 1
DEPARTING = 2
STATIONARY = 3
UNKNOWN_TYPE = 0
POLICE = 1
AMBULANCE = 2
FIRETRUCK = 3
UNKNOWN_DIRECTION = 0
FRONT = 1
LEFT = 2
BACK = 3
RIGHT = 4


DESCRIPTOR.enum_types_by_name["MovingResult"] = _MOVINGRESULT
DESCRIPTOR.enum_types_by_name["AudioType"] = _AUDIOTYPE
DESCRIPTOR.enum_types_by_name["AudioDirection"] = _AUDIODIRECTION
_sym_db.RegisterFileDescriptor(DESCRIPTOR)


# @@protoc_insertion_point(module_scope)
