import sqlite3
import os
import glob

import yaml
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
from yaml import SafeLoader


class ROSBagReader:

    def __init__(self, bag_dir_path: str):
        self.bag_dir_path = bag_dir_path

        db3_path = self.get_file_path("db3")
        self.yaml_path = self.get_file_path("yaml")

        self.sql_connection = sqlite3.connect(db3_path)
        self.sql_cursor = self.sql_connection.cursor()
        self.db3_path = db3_path
        self.yaml = None
        # create a message type map
        topics_data = self.sql_cursor.execute("SELECT id, name, type FROM topics").fetchall()
        self.topic_id = {name_of: id_of for id_of, name_of, type_of in topics_data}
        self.topic_msg_message = {name_of: get_message(type_of) for id_of, name_of, type_of in topics_data}

        self.fetch_all_msgs_sql = "select topics.name as topic, data as message, timestamp as t from messages join topics on messages.topic_id = topics.id order by timestamp"

    def get_file_path(self, extension: str) -> str:
        pattern = os.path.join(self.bag_dir_path, f'*.{extension}')
        files = glob.glob(pattern)
        if len(files) == 0:
            raise Exception(f"No {extension} files found in {self.bag_dir_path}")
        return files[0]

    def read_messages(self):
        rows = self.sql_cursor.execute(self.fetch_all_msgs_sql).fetchall()
        return rows

    def deserialize_msg(self, data, topic_name):
        return deserialize_message(data, self.topic_msg_message[topic_name])

    def __del__(self):
        if self.sql_connection:
            self.sql_connection.close()

    def read_specific_messages(self, topic_name: str):
        topic_data_list = []
        for topic, message, t in self.read_messages():
            # if topic == topic_name:
            if topic_name in topic:
                msg = self.deserialize_msg(message, topic)
                topic_data_list.append((topic, msg, t))
        return topic_data_list

    def get_duration(self):
        if not self.yaml:
            self.read_yaml()
        return self.yaml['rosbag2_bagfile_information']['duration']['nanoseconds']

    def read_yaml(self):
        self.yaml = yaml.load(open(self.yaml_path, 'r'), Loader=SafeLoader)

    def has_routing_msg(self):
        if not self.yaml:
            self.read_yaml()
        tp_msgs = self.yaml['rosbag2_bagfile_information']['topics_with_message_count']
        for tm in tp_msgs:
            if tm['topic_metadata']['name'] == '/planning/mission_planning/route':
                return tm['message_count'] > 0
