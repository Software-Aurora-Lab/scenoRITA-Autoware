# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/perception/onboard/proto/trafficlights_perception_component.proto

import sys

_b = sys.version_info[0] < 3 and (lambda x: x) or (lambda x: x.encode("latin1"))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database

# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


DESCRIPTOR = _descriptor.FileDescriptor(
    name="modules/perception/onboard/proto/trafficlights_perception_component.proto",
    package="apollo.perception.onboard",
    syntax="proto2",
    serialized_options=None,
    serialized_pb=_b(
        '\nImodules/perception/onboard/proto/trafficlights_perception_component.proto\x12\x19\x61pollo.perception.onboard"\x8e\x08\n\x0cTrafficLight\x12\x1e\n\x0ftl_tf2_frame_id\x18\x01 \x01(\t:\x05world\x12<\n\x15tl_tf2_child_frame_id\x18\x02 \x01(\t:\x1dperception_localization_100hz\x12 \n\x12tf2_timeout_second\x18\x03 \x01(\x01:\x04\x30.01\x12*\n\x0c\x63\x61mera_names\x18\x04 \x01(\t:\x14\x66ront_6mm,front_12mm\x12^\n\x14\x63\x61mera_channel_names\x18\x05 \x01(\t:@/apollo/sensor/camera/front_6mm,/apollo/sensor/camera/front_12mm\x12$\n\x19tl_image_timestamp_offset\x18\x06 \x01(\x01:\x01\x30\x12 \n\x15max_process_image_fps\x18\x07 \x01(\x05:\x01\x38\x12&\n\x19query_tf_interval_seconds\x18\x08 \x01(\x01:\x03\x30.3\x12!\n\x14valid_hdmap_interval\x18\t \x01(\x01:\x03\x31.5\x12(\n\x1bimage_sys_ts_diff_threshold\x18\n \x01(\x01:\x03\x30.5\x12"\n\x15sync_interval_seconds\x18\x0b \x01(\x01:\x03\x30.5\x12H\n(camera_traffic_light_perception_conf_dir\x18\x0c \x01(\t:\x16\x63onf/perception/camera\x12\x42\n)camera_traffic_light_perception_conf_file\x18\r \x01(\t:\x0ftrafficlight.pt\x12&\n\x19\x64\x65\x66\x61ult_image_border_size\x18\x0e \x01(\x05:\x03\x31\x30\x30\x12K\n!traffic_light_output_channel_name\x18\x0f \x01(\t: /apollo/perception/traffic_light\x12L\n\x17simulation_channel_name\x18\x10 \x01(\t:+/apollo/perception/traffic_light_simulation\x12G\n$v2x_trafficlights_input_channel_name\x18\x11 \x01(\t:\x19/apollo/v2x/traffic_light\x12&\n\x19v2x_sync_interval_seconds\x18\x12 \x01(\x01:\x03\x30.1\x12!\n\x15max_v2x_msg_buff_size\x18\x13 \x01(\x05:\x02\x35\x30\x12,\n\x14tl_preprocessor_name\x18\x14 \x01(\t:\x0eTLPreprocessor'
    ),
)


_TRAFFICLIGHT = _descriptor.Descriptor(
    name="TrafficLight",
    full_name="apollo.perception.onboard.TrafficLight",
    filename=None,
    file=DESCRIPTOR,
    containing_type=None,
    fields=[
        _descriptor.FieldDescriptor(
            name="tl_tf2_frame_id",
            full_name="apollo.perception.onboard.TrafficLight.tl_tf2_frame_id",
            index=0,
            number=1,
            type=9,
            cpp_type=9,
            label=1,
            has_default_value=True,
            default_value=_b("world").decode("utf-8"),
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="tl_tf2_child_frame_id",
            full_name="apollo.perception.onboard.TrafficLight.tl_tf2_child_frame_id",
            index=1,
            number=2,
            type=9,
            cpp_type=9,
            label=1,
            has_default_value=True,
            default_value=_b("perception_localization_100hz").decode("utf-8"),
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="tf2_timeout_second",
            full_name="apollo.perception.onboard.TrafficLight.tf2_timeout_second",
            index=2,
            number=3,
            type=1,
            cpp_type=5,
            label=1,
            has_default_value=True,
            default_value=float(0.01),
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="camera_names",
            full_name="apollo.perception.onboard.TrafficLight.camera_names",
            index=3,
            number=4,
            type=9,
            cpp_type=9,
            label=1,
            has_default_value=True,
            default_value=_b("front_6mm,front_12mm").decode("utf-8"),
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="camera_channel_names",
            full_name="apollo.perception.onboard.TrafficLight.camera_channel_names",
            index=4,
            number=5,
            type=9,
            cpp_type=9,
            label=1,
            has_default_value=True,
            default_value=_b(
                "/apollo/sensor/camera/front_6mm,/apollo/sensor/camera/front_12mm"
            ).decode("utf-8"),
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="tl_image_timestamp_offset",
            full_name="apollo.perception.onboard.TrafficLight.tl_image_timestamp_offset",
            index=5,
            number=6,
            type=1,
            cpp_type=5,
            label=1,
            has_default_value=True,
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
            name="max_process_image_fps",
            full_name="apollo.perception.onboard.TrafficLight.max_process_image_fps",
            index=6,
            number=7,
            type=5,
            cpp_type=1,
            label=1,
            has_default_value=True,
            default_value=8,
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="query_tf_interval_seconds",
            full_name="apollo.perception.onboard.TrafficLight.query_tf_interval_seconds",
            index=7,
            number=8,
            type=1,
            cpp_type=5,
            label=1,
            has_default_value=True,
            default_value=float(0.3),
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="valid_hdmap_interval",
            full_name="apollo.perception.onboard.TrafficLight.valid_hdmap_interval",
            index=8,
            number=9,
            type=1,
            cpp_type=5,
            label=1,
            has_default_value=True,
            default_value=float(1.5),
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="image_sys_ts_diff_threshold",
            full_name="apollo.perception.onboard.TrafficLight.image_sys_ts_diff_threshold",
            index=9,
            number=10,
            type=1,
            cpp_type=5,
            label=1,
            has_default_value=True,
            default_value=float(0.5),
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="sync_interval_seconds",
            full_name="apollo.perception.onboard.TrafficLight.sync_interval_seconds",
            index=10,
            number=11,
            type=1,
            cpp_type=5,
            label=1,
            has_default_value=True,
            default_value=float(0.5),
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="camera_traffic_light_perception_conf_dir",
            full_name="apollo.perception.onboard.TrafficLight.camera_traffic_light_perception_conf_dir",
            index=11,
            number=12,
            type=9,
            cpp_type=9,
            label=1,
            has_default_value=True,
            default_value=_b("conf/perception/camera").decode("utf-8"),
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="camera_traffic_light_perception_conf_file",
            full_name="apollo.perception.onboard.TrafficLight.camera_traffic_light_perception_conf_file",
            index=12,
            number=13,
            type=9,
            cpp_type=9,
            label=1,
            has_default_value=True,
            default_value=_b("trafficlight.pt").decode("utf-8"),
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="default_image_border_size",
            full_name="apollo.perception.onboard.TrafficLight.default_image_border_size",
            index=13,
            number=14,
            type=5,
            cpp_type=1,
            label=1,
            has_default_value=True,
            default_value=100,
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="traffic_light_output_channel_name",
            full_name="apollo.perception.onboard.TrafficLight.traffic_light_output_channel_name",
            index=14,
            number=15,
            type=9,
            cpp_type=9,
            label=1,
            has_default_value=True,
            default_value=_b("/apollo/perception/traffic_light").decode("utf-8"),
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="simulation_channel_name",
            full_name="apollo.perception.onboard.TrafficLight.simulation_channel_name",
            index=15,
            number=16,
            type=9,
            cpp_type=9,
            label=1,
            has_default_value=True,
            default_value=_b("/apollo/perception/traffic_light_simulation").decode(
                "utf-8"
            ),
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="v2x_trafficlights_input_channel_name",
            full_name="apollo.perception.onboard.TrafficLight.v2x_trafficlights_input_channel_name",
            index=16,
            number=17,
            type=9,
            cpp_type=9,
            label=1,
            has_default_value=True,
            default_value=_b("/apollo/v2x/traffic_light").decode("utf-8"),
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="v2x_sync_interval_seconds",
            full_name="apollo.perception.onboard.TrafficLight.v2x_sync_interval_seconds",
            index=17,
            number=18,
            type=1,
            cpp_type=5,
            label=1,
            has_default_value=True,
            default_value=float(0.1),
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="max_v2x_msg_buff_size",
            full_name="apollo.perception.onboard.TrafficLight.max_v2x_msg_buff_size",
            index=18,
            number=19,
            type=5,
            cpp_type=1,
            label=1,
            has_default_value=True,
            default_value=50,
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="tl_preprocessor_name",
            full_name="apollo.perception.onboard.TrafficLight.tl_preprocessor_name",
            index=19,
            number=20,
            type=9,
            cpp_type=9,
            label=1,
            has_default_value=True,
            default_value=_b("TLPreprocessor").decode("utf-8"),
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
    serialized_start=105,
    serialized_end=1143,
)

DESCRIPTOR.message_types_by_name["TrafficLight"] = _TRAFFICLIGHT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

TrafficLight = _reflection.GeneratedProtocolMessageType(
    "TrafficLight",
    (_message.Message,),
    dict(
        DESCRIPTOR=_TRAFFICLIGHT,
        __module__="modules.perception.onboard.proto.trafficlights_perception_component_pb2"
        # @@protoc_insertion_point(class_scope:apollo.perception.onboard.TrafficLight)
    ),
)
_sym_db.RegisterMessage(TrafficLight)


# @@protoc_insertion_point(module_scope)
