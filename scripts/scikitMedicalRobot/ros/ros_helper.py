#!/usr/bin/env python3
import rospy
import roslib
import copy
import tf
import os
import sys
import signal
import subprocess
import time
import yaml
import math
import rosbag
import rospkg
import threading
import pandas as pd
import numpy as np
from pathlib import Path
from tqdm.auto import tqdm
from operator import attrgetter
from pathlib import Path
from roscpp.srv import SetLoggerLevel
from utilities.srv import DataSampleCommand, DataSampleCommandRequest, DataSampleCommandResponse
from ..utilities import generate_timedate_cache_file, mergesort, ensure_dir
from PyQt5 import QtWidgets
from PyQt5.QtCore import QObject, QProcess, pyqtSignal, QCoreApplication
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped, Transform, Vector3, Quaternion


class RosbagRecoder(QObject):

    errorSignal = pyqtSignal(str)
    outputSignal = pyqtSignal(str)

    cmd_rosbag = 'rosbag record'
    cmd_topics = ' --all '
    cmd_output_name = ' --output-name=[name] '
    cmd_chunksize = ' --chunksize=[chunksize] '

    def __init__(self, bag_filename, topics=None, chunksize=4096):
        super(RosbagRecoder, self).__init__()

        if topics is not None:
            self.cmd_topics = ''
            for topic in topics:
                self.cmd_topics += f"{topic} "

        self._cmd_base = self.cmd_rosbag + \
            self.cmd_output_name.replace('[name]', str(bag_filename)) + \
            self.cmd_chunksize.replace('[chunksize]', str(chunksize)) + \
            self.cmd_topics

        self._process = QProcess()
        self._process.readyReadStandardError.connect(self._onReadyReadStandardError)
        self._process.readyReadStandardOutput.connect(self._onReadyReadStandardOutput)

        self._record()

    def _onReadyReadStandardError(self):
        result = self._process.readAllStandardError().data().decode()
        rospy.loginfo(result)
        self.errorSignal.emit(result)

    def _onReadyReadStandardOutput(self):
        result = self._process.readAllStandardOutput().data().decode()
        rospy.loginfo(result)
        self.outputSignal.emit(result)

    def _system_call(self, command):
        """
        Executes a system command.
        """
        self._process.waitForFinished(30000)
        self._process.start(command)

    def _record(self):
        rospy.loginfo('RosbagRecoder: Start Recording ROS Topics...')
        rospy.loginfo(self._cmd_base)
        self._system_call(self._cmd_base)

    def stop(self):
        if self._process.processId() != 0:
            self.terminate_process_and_children()

    def terminate_process_and_children(self):
        ps_command = subprocess.Popen("ps -o pid --ppid %d --noheaders" % self._process.processId(), shell=True, stdout=subprocess.PIPE)
        ps_output = ps_command.stdout.read()
        retcode = ps_command.wait()
        assert retcode == 0, "ps command returned %d" % retcode
        for pid_str in ps_output.decode().split("\n")[:-1]:
            os.kill(int(pid_str), signal.SIGINT)
        os.kill(self._process.processId(), signal.SIGINT)

class RosbagPlayer(QObject):

    errorSignal = pyqtSignal(str)
    outputSignal = pyqtSignal(str)

    cmd_rosbag = 'rosbag play --try-future-version'
    cmd_delay = ' -d '
    cmd_duration = ' -u '
    cmd_start = ' -s '
    cmd_publish_clock = ' --clock '
    cmd_multiply_factor = ' -r '
    cmd_select_topics = ' --topics '
    cmd_bag_file = ' --bags='

    def __init__(self, timeline):
        super(RosbagPlayer, self).__init__()
        self.timeline = timeline
        self.is_publishing = False
        self._cmd_base = self.cmd_rosbag + self.cmd_delay + '0'

        self._process = QProcess()
        self._process.readyReadStandardError.connect(self._onReadyReadStandardError)
        self._process.readyReadStandardOutput.connect(self._onReadyReadStandardOutput)

        self._playhead_offset = 0.0

    def _onReadyReadStandardError(self):
        result = self._process.readAllStandardError().data().decode()
        self.errorSignal.emit(result)

    def _onReadyReadStandardOutput(self):
        result = self._process.readAllStandardOutput().data().decode()
        self.outputSignal.emit(result)

    def _system_call(self, command):
        """
        Executes a system command.
        """
        self._process.waitForFinished(30000)
        self._process.start(command)

    def start_publishing(self, playhead_offset=0.0, speed=1.0):
        if self.is_publishing:
            self.stop_publishing()

        cmd = self._cmd_base + self.cmd_bag_file + '\"' + self.timeline.bag_filename + '\"'
        if not isinstance(playhead_offset, float):
            playhead_offset = 0.0
        cmd = cmd + self.cmd_start + str(playhead_offset)
        self._playhead_offset = playhead_offset
        if not isinstance(speed, float):
            speed = 1.0
        cmd = cmd + self.cmd_multiply_factor + str(speed)
        self._system_call(cmd)
        self.is_publishing = True

    def stop_publishing(self):
        if self._process.processId() != 0:
            self.terminate_process_and_children()
        self.is_publishing = False

    def terminate_process_and_children(self):
        ps_command = subprocess.Popen("ps -o pid --ppid %d --noheaders" % self._process.processId(), shell=True, stdout=subprocess.PIPE)
        ps_output = ps_command.stdout.read()
        retcode = ps_command.wait()
        assert retcode == 0, "ps command returned %d" % retcode
        for pid_str in ps_output.split("\n")[:-1]:
            os.kill(int(pid_str), signal.SIGINT)
        os.kill(self._process.processId(), signal.SIGINT)

    def stop(self):
        self.stop_publishing()

class rosbag_data_sample:
    def __init__(self, topics, path_to_save='./', tag=None) -> None:
        if tag is not None:
            self._tag = tag
        else:
            self._tag = self.__class__.__name__

        self._topics = topics
        self._path_to_save = path_to_save
        rospy.loginfo(f"{self._tag} recording topics: {self._topics}")
        rospy.loginfo(f"{self._tag} saving path: {self._path_to_save}")

        self._recorder = None

        rospy.loginfo(f"{self._tag} - __init__()")

        topic = f"{rospy.get_namespace()}ros_helper/service/{self._tag}"
        rospy.loginfo(f"{self._tag} service name: {topic}")
        self._service = rospy.Service(f"{topic}", DataSampleCommand, self._service_callback)

    def _service_callback(self, req: DataSampleCommandRequest):
        if self._recorder is not None:
            self._recorder.stop()

        if req.operation != 0:
            rospy.loginfo(f"{self._tag} - start sample record.")
            dir_path = Path(self._path_to_save) / Path(req.filename.replace(' ', '_'))
            bag_filename = generate_timedate_cache_file(caches_dir=str(dir_path.parent), suffix=str(dir_path.name))
            rospy.loginfo(f"{self._tag} - rosbag saved to {bag_filename}.bag")
            self._recorder = RosbagRecoder(bag_filename.absolute(), self._topics, chunksize=4096)
        else:
            rospy.loginfo(f"{self._tag} - stop sample record.")
            self._recorder = None
        return DataSampleCommandResponse(True, f"receive: {req.operation}, {req.filename}")

    def exec(self):
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        app = QCoreApplication(sys.argv)
        sys.exit(app.exec_())

class rosbag_extractor:
    def __init__(self, bag_file) -> None:
        self._bagfile_name = Path(bag_file).stem
        self._bag_lock = threading.RLock()
        self._bag = rosbag.Bag(bag_file)
        self._bag_tf = None

        self._topics_by_datatype, self._datatype_by_topic = self.get_topics_and_datatypes()
        self._available_topics = self.get_topics()

    @property
    def bag_tf(self):
        if self._bag_tf is None:
            if ('/tf' in self._available_topics) and ('/tf_static' in self._available_topics):
                self._bag_tf = BagTfTransformer(self._bag)

        return self._bag_tf

    def file_size(self, human=False):
        with self._bag_lock:
            if not human:
                return self._bag.size
            else:
                return self.helper_bagsize_to_str(self._bag)

    def duration(self):
        '''
        :return: float, timestamp in seconds, includes fractions of a second
        '''
        with self._bag_lock:
            return self._bag.get_end_time() - self._bag.get_start_time()
        
    def get_start_stamp(self):
        """
        :return: first stamp in the bags, ''rospy.Time''
        """
        with self._bag_lock:
            start_stamp = None
            bag_start_stamp = self.helper_get_start_stamp(self._bag)
            if bag_start_stamp is not None and (start_stamp is None or bag_start_stamp < start_stamp):
                start_stamp = bag_start_stamp
            return start_stamp
        
    def get_end_stamp(self):
        """
        :return: last stamp in the bags, ''rospy.Time''
        """
        with self._bag_lock:
            end_stamp = None
            bag_end_stamp = self.helper_get_end_stamp(self._bag)
            if bag_end_stamp is not None and (end_stamp is None or bag_end_stamp > end_stamp):
                end_stamp = bag_end_stamp
            return end_stamp
        
    def get_topics(self):
        """
        :return: sorted list of topic names, ''list(str)''
        """
        with self._bag_lock:
            topics = set()
            for topic in self.helper_get_topics(self._bag):
                topics.add(topic)
            return sorted(topics)
        
    def get_topics_and_datatypes(self):
        """
        :return: dict of list of topics for each datatype, ''dict(datatype:list(topic))''
        """
        with self._bag_lock:
            topics_by_datatype = {}
            datatype_by_topic = dict()
            pyclass_by_topic = dict()
            extract_by_topic = dict()
            for datatype, topics in self.helper_get_topics_by_datatypes(self._bag).items():
                topics_by_datatype.setdefault(datatype, []).extend(topics)
                for topic in topics:
                    datatype_by_topic[topic] = datatype
            return topics_by_datatype, datatype_by_topic
        
    def get_datatype(self, topic):
        """
        :return: datatype associated with a topic, ''str''
        :raises: if there are multiple datatypes assigned to a single topic, ''Exception''
        """
        with self._bag_lock:
            datatype = None
            bag_datatype = self.helper_get_datatype(self._bag, topic)
            if datatype and bag_datatype and (bag_datatype != datatype):
                raise Exception('topic %s has multiple datatypes: %s and %s' %
                                (topic, datatype, bag_datatype))
            if bag_datatype:
                datatype = bag_datatype
            return datatype
        
    def get_entries(self, topics, start_stamp=None, end_stamp=None):
        """
        generator function for bag entries
        :param topics: list of topics to query, ''list(str)''
        :param start_stamp: stamp to start at, ''rospy.Time''
        :param end_stamp: stamp to end at, ''rospy,Time''
        :returns: entries the bag file, ''msg''
        """
        with self._bag_lock:
            bag_entries = []

            bag_start_time = self.helper_get_start_stamp(self._bag)
            if bag_start_time is not None and start_stamp is None:
                start_stamp = bag_start_time
            bag_end_time = self.helper_get_end_stamp(self._bag)
            if bag_end_time is not None and end_stamp is None:
                end_stamp = bag_end_time

            if bag_start_time is not None and bag_start_time > end_stamp:
                raise IndexError
            if bag_end_time is not None and bag_end_time < start_stamp:
                raise IndexError
            
            connections = list(self._bag._get_connections(topics))
            bag_entries.append(self._bag._get_entries(connections, start_stamp, end_stamp))

            for entry, _ in mergesort(bag_entries, key=lambda entry: entry.time):
                yield entry

    def get_entries_with_bag(self, topic, start_stamp=None, end_stamp=None):
        """
        generator function for bag entries
        :param topics: list of topics to query, ''list(str)''
        :param start_stamp: stamp to start at, ''rospy.Time''
        :param end_stamp: stamp to end at, ''rospy,Time''
        :returns: tuple of (bag, entry) for the entries in the bag file, ''(rosbag.bag, msg)''
        """
        with self._bag_lock:
            from rosbag import bag  # for _mergesort

            bag_entries = []
            bag_by_iter = {}
            bag_start_time = self.helper_get_start_stamp(self._bag)
            if bag_start_time is not None and start_stamp is None:
                start_stamp = bag_start_time
            bag_end_time = self.helper_get_end_stamp(self._bag)
            if bag_end_time is not None and end_stamp is None:
                end_stamp = bag_end_time

            if bag_start_time is not None and bag_start_time > end_stamp:
                raise IndexError
            if bag_end_time is not None and bag_end_time < start_stamp:
                raise IndexError
            
            connections = list(self._bag._get_connections(topic))
            it = iter(self._bag._get_entries(connections, start_stamp, end_stamp))
            bag_by_iter[it] = self._bag
            bag_entries.append(it)

            for entry, it in mergesort(bag_entries, key=lambda entry: entry.time):
                yield bag_by_iter[it], entry

    def get_entry(self, t, topic):
        """
        Access a bag entry
        :param t: time, ''rospy.Time''
        :param topic: the topic to be accessed, ''str''
        :return: tuple of (bag, entry) corresponding to time t and topic, ''(rosbag.bag, msg)''
        """
        with self._bag_lock:
            entry_bag, entry = None, None

            bag_entry = self._bag._get_entry(t, self._bag._get_connections(topic))
            if bag_entry and (not entry or bag_entry.time > entry.time):
                entry_bag, entry = self._bag, bag_entry

            return entry_bag, entry
        
    def get_entry_before(self, t):
        """
        Access a bag entry
        :param t: time, ''rospy.Time''
        :return: tuple of (bag, entry) corresponding to time t, ''(rosbag.bag, msg)''
        """
        with self._bag_lock:
            entry_bag, entry = None, None
            bag_entry = self._bag._get_entry(t - rospy.Duration(0, 1), self._bag._get_connections())
            if bag_entry and (not entry or bag_entry.time < entry.time):
                entry_bag, entry = self._bag, bag_entry

            return entry_bag, entry
    
    def get_entry_after(self, t):
        """
        Access a bag entry
        :param t: time, ''rospy.Time''
        :return: tuple of (bag, entry) corisponding to time t, ''(rosbag.bag, msg)''
        """
        with self._bag_lock:
            entry_bag, entry = None, None
            bag_entry = self._bag._get_entry_after(t, self._bag._get_connections())
            if bag_entry and (not entry or bag_entry.time < entry.time):
                entry_bag, entry = self._bag, bag_entry

            return entry_bag, entry
    
    def extract_data_from_topics(self, topics_selection, start_stamp=None, end_stamp=None, with_tqdm=True):
        topics_selection, bag_entries = self.prepare_for_extraction(topics_selection, start_stamp, end_stamp)
        return self._run_extract_data_from_topics(topics_selection, bag_entries, with_tqdm)
    
    def list_topics_to_dict(self, topics):
        if isinstance(topics, list):
            topics_dict = {}
            for idx, topic in enumerate(topics):
                topics_dict[topic] = []
            return topics_dict
        return topics
    
    def ensure_topics_exist(self, topics):
        topics_dict = self.list_topics_to_dict(topics)
        valid = {}
        invalid = {}
        for topic in topics_dict.keys():
            if topic in self._available_topics:
                valid[topic] = topics_dict[topic]
            else:
                invalid[topic] = topics_dict[topic]
        return valid, invalid

    def prepare_for_extraction(self, topics_selection, start_stamp=None, end_stamp=None):
        topics_selection, _ = self.ensure_topics_exist(topics_selection)

        bag_entries = list(self.get_entries_with_bag(list(topics_selection.keys()), start_stamp, end_stamp))

        # Get the total number of messages to copy
        total_messages = len(bag_entries)

        # If no messages, prompt the user and return
        if total_messages == 0:
            return None, None

        return topics_selection, bag_entries

    def _run_extract_data_from_topics(self, topics_selection, bag_entries, with_tqdm=True):
        total_messages = len(bag_entries)
        dataframe_by_topic = dict()
        ptr_by_topic = dict()
        for topic in topics_selection.keys():
            if topic in self._datatype_by_topic.keys():
                datatype = self._datatype_by_topic[topic]
                msg_count = self._bag.get_message_count(topic)
                columns = ['timestamp', 'msg']
                index = range(0, msg_count)
                ptr_by_topic[topic] = 0

                for link_pair in topics_selection[topic]:
                    link_pair_columns = [f'{link_pair[0]}->{link_pair[1]}']
                    columns = columns + link_pair_columns
                    
                dataframe_by_topic[topic] = {}
                for key in columns:
                    dataframe_by_topic[topic][key] = []

        with tqdm(total=total_messages, disable=(not with_tqdm)) as pbar:
            for bag, entry in bag_entries:
                try:
                    topic, msg, t = self.read_message(bag, entry.position)
                    ptr = ptr_by_topic[topic]
                    ptr_data = dataframe_by_topic[topic]
                    ptr_data['timestamp'].append(t.to_sec())
                    ptr_data['msg'].append(self.helper_rewrite_message_class(msg))

                    for link_pair in topics_selection[topic]:
                        try:
                            position, quat = self.bag_tf.lookupTransform(link_pair[0], link_pair[1], msg.header.stamp)
                            ptr_data[f'{link_pair[0]}->{link_pair[1]}'].append((position, quat))
                        except Exception:
                            ptr_data[f'{link_pair[0]}->{link_pair[1]}'].append((None, None))

                    ptr_by_topic[topic] = ptr_by_topic[topic] + 1
                except Exception as ex:
                    print('Error exporting message at position %s: %s' % (str(entry.position), str(ex)))
                    pbar.close()
                    return
                pbar.update(1)
        pbar.close()

        return dataframe_by_topic

    def read_message(self, bag, entry_position):
        with self._bag_lock:
            return bag._read_message(entry_position)

    @staticmethod
    def helper_bagsize_to_str(bag):
        size = bag.size
        size_name = ('B', 'KB', 'MB', 'GB', 'TB', 'PB', 'EB', 'ZB', 'YB')
        i = int(math.floor(math.log(size, 1024)))
        p = math.pow(1024, i)
        s = round(size / p, 2)
        if s > 0:
            return '%s %s' % (s, size_name[i])
        return '0 B'

    @staticmethod
    def helper_stamp_to_str(t):
        """
        Convert a rospy.Time to a human-readable string.

        @param t: time to convert
        @type  t: rospy.Time
        """
        if isinstance(t, float):
            t_sec = t
            t = rospy.Time.from_sec(t)
        else:
            t_sec = t.to_sec()
        if t < rospy.Time.from_sec(60 * 60 * 24 * 365 * 5):
            # Display timestamps earlier than 1975 as seconds
            return '%.3fs' % t_sec
        else:
            return time.strftime('%b %d %Y %H:%M:%S', time.localtime(t_sec)) + '.%03d' % (t.nsecs / 1000000)

    # Rewrite bag with header timestamps
    # This is useful in the case that the message receipt time substantially differs from the generation time, 
    # e.g. when messages are recorded over an unreliable or slow connection.

    # Note that this could potentially change the order in which messages are republished by rosbag play.

    @staticmethod
    def helper_rewrite_ts_header(outbag, inputbag):
        for topic, msg, t in inputbag.read_messages():
            # This also replaces tf timestamps under the assumption 
            # that all transforms in the message share the same timestamp
            if topic == "/tf" and msg.transforms:
                outbag.write(topic, msg, msg.transforms[0].header.stamp)
            else:
                outbag.write(topic, msg, msg.header.stamp if msg._has_header else t)

    @staticmethod
    def helper_get_topics(bag):
        """
        Get an alphabetical list of all the unique topics in the bag.

        @return: sorted list of topics
        @rtype:  list of str
        """
        return sorted(set([c.topic for c in bag._get_connections()]))

    @staticmethod
    def helper_get_start_stamp(bag):
        """
        Get the earliest timestamp in the bag.

        @param bag: bag file
        @type  bag: rosbag.Bag
        @return: earliest timestamp
        @rtype:  rospy.Time
        """
        start_stamp = None
        for connection_start_stamp in [index[0].time for index in bag._connection_indexes.values()]:
            if not start_stamp or connection_start_stamp < start_stamp:
                start_stamp = connection_start_stamp
        return start_stamp

    @staticmethod
    def helper_get_end_stamp(bag):
        """
        Get the latest timestamp in the bag.

        @param bag: bag file
        @type  bag: rosbag.Bag
        @return: latest timestamp
        @rtype:  rospy.Time
        """
        end_stamp = None
        for connection_end_stamp in [index[-1].time for index in bag._connection_indexes.values()]:
            if not end_stamp or connection_end_stamp > end_stamp:
                end_stamp = connection_end_stamp

        return end_stamp

    @staticmethod
    def helper_get_topics_by_datatypes(bag):
        """
        Get all the message types in the bag and their associated topics.

        @param bag: bag file
        @type  bag: rosbag.Bag
        @return: mapping from message typename to list of topics
        @rtype:  dict of str to list of str
        """
        topics_by_datatype = {}
        for c in bag._get_connections():
            topics_by_datatype.setdefault(c.datatype, []).append(c.topic)

        return topics_by_datatype

    # Get lists of topics and types from a bag
    @staticmethod
    def helper_get_topics_and_types(bag):
        topics = bag.get_type_and_topic_info()[1].keys()
        types = []
        for i in range(0,len(bag.get_type_and_topic_info()[1].values())):
            types.append(bag.get_type_and_topic_info()[1].values()[i][0])
        return topics, types

    @staticmethod
    def helper_get_datatype(bag, topic):
        """
        Get the datatype of the given topic.

        @param bag: bag file
        @type  bag: rosbag.Bag
        @return: message typename
        @rtype:  str
        """
        for c in bag._get_connections(topic):
            return c.datatype

        return None
    
    # Add metadata to a bag
    @staticmethod
    def helper_add_metadata(bag, metadata_msg):
        bag.write('/metadata', metadata_msg, rospy.Time(bag.get_end_time()))

    @staticmethod
    # Get summary information about a bag
    def helper_get_summary_info_exec(bag_file):
        info_dict = yaml.load(subprocess.Popen(['rosbag', 'info', '--yaml', bag_file], stdout=subprocess.PIPE).communicate()[0])
        return info_dict
    
    @staticmethod
    def helper_get_summary_info(bag):
        info_dict = yaml.load(bag._get_yaml_info())

    # Create a cropped bagfile
    @staticmethod
    def helper_gen_cropped_bag(outbag, inputbag, num_msgs):
        for topic, msg, t in inputbag.read_messages():
            while num_msgs:
                outbag.write(topic, msg, t)
                num_msgs -= 1

    @staticmethod
    def helper_rewrite_message_class(msg):
        try:
            msg.__class__ = roslib.message.get_message_class(msg._type)
            return msg
        except Exception:
            return msg

class BagTfTransformer(object):
    """
    A transformer which transparently uses data recorded from rosbag on the /tf topic
    https://github.com/IFL-CAMP/tf_bag
    """

    def __init__(self, bag):
        """
        Create a new BagTfTransformer from an open rosbag or from a file path
        :param bag: an open rosbag or a file path to a rosbag file
        """
        if type(bag) == str:
            bag = rosbag.Bag(bag)
        self.tf_messages = sorted(
            (self._remove_slash_from_frames(tm) for m in bag if m.topic.strip("/") == 'tf' for tm in
             m.message.transforms),
            key=lambda tfm: tfm.header.stamp.to_nsec())
        self.tf_static_messages = sorted(
            (self._remove_slash_from_frames(tm) for m in bag if m.topic.strip("/") == 'tf_static' for tm in
             m.message.transforms),
            key=lambda tfm: tfm.header.stamp.to_nsec())

        self.tf_times = np.array(list((tfm.header.stamp.to_nsec() for tfm in self.tf_messages)))
        self.transformer = tf.TransformerROS()
        self.last_population_range = (rospy.Time(0), rospy.Time(0))
        self.all_frames = None
        self.all_transform_tuples = None
        self.static_transform_tuples = None

    @staticmethod
    def _remove_slash_from_frames(msg):
        msg.header.frame_id = msg.header.frame_id.strip("/")
        msg.child_frame_id = msg.child_frame_id.strip("/")
        return msg

    def getMessagesInTimeRange(self, min_time=None, max_time=None):
        """
        Returns all messages in the time range between two given ROS times
        :param min_time: the lower end of the desired time range (if None, the bag recording start time)
        :param max_time: the upper end of the desired time range (if None, the bag recording end time)
        :return: an iterator over the messages in the time range
        """
        import genpy
        if min_time is None:
            min_time = -float('inf')
        elif type(min_time) in (genpy.rostime.Time, rospy.rostime.Time):
            min_time = min_time.to_nsec()
        if max_time is None:
            max_time = float('inf')
        elif type(max_time) in (genpy.rostime.Time, rospy.rostime.Time):
            max_time = max_time.to_nsec()
        if max_time < min_time:
            raise ValueError('the minimum time should be lesser than the maximum time!')
        indices_in_range = np.where(np.logical_and(min_time < self.tf_times, self.tf_times < max_time))
        ret = (self.tf_messages[i] for i in indices_in_range[0])
        return ret

    def populateTransformerAtTime(self, target_time, buffer_length=10, lookahead=0.1):
        """
        Fills the buffer of the internal tf Transformer with the messages preceeding the given time
        :param target_time: the time at which the Transformer is going to be queried at next
        :param buffer_length: the length of the buffer, in seconds (default: 10, maximum for tf TransformerBuffer)
        """
        target_start_time = target_time - rospy.Duration(
            min(min(buffer_length, 10) - lookahead, target_time.to_sec()))  # max buffer length of tf Transformer
        target_end_time = target_time + rospy.Duration(lookahead)  # lookahead is there for numerical stability
        # otherwise, messages exactly around that time could be discarded
        previous_start_time, previous_end_time = self.last_population_range

        if target_start_time < previous_start_time:
            self.transformer.clear()  # or Transformer would ignore messages as old ones
            population_start_time = target_start_time
        else:
            population_start_time = max(target_start_time, previous_end_time)

        tf_messages_in_interval = self.getMessagesInTimeRange(population_start_time, target_end_time)
        for m in tf_messages_in_interval:
            self.transformer.setTransform(m)
        for st_tfm in self.tf_static_messages:
            st_tfm.header.stamp = target_time
            self.transformer._buffer.set_transform_static(st_tfm, "default_authority")

        self.last_population_range = (target_start_time, target_end_time)

    def getTimeAtPercent(self, percent):
        """
        Returns the ROS time at the given point in the time range
        :param percent: the point in the recorded time range for which the ROS time is desired
        :return:
        """
        start_time, end_time = self.getStartTime(), self.getEndTime()
        time_range = (end_time - start_time).to_sec()
        ret = start_time + rospy.Duration(time_range * float(percent / 100))
        return ret

    def _filterMessages(self, orig_frame=None, dest_frame=None, start_time=None, end_time=None, reverse=False):
        if reverse:
            messages = reversed(self.tf_messages)
        else:
            messages = self.tf_messages

        if orig_frame:
            messages = filter(lambda m: m.header.frame_id == orig_frame, messages)
        if dest_frame:
            messages = filter(lambda m: m.child_frame_id == dest_frame, messages)
        if start_time:
            messages = filter(lambda m: m.header.stamp > start_time, messages)
        if end_time:
            messages = filter(lambda m: m.header.stamp < end_time, messages)
        return messages

    def getTransformMessagesWithFrame(self, frame, start_time=None, end_time=None, reverse=False):
        """
        Returns all transform messages with given frame as source or target frame
        :param frame: the tf frame of interest
        :param start_time: the time at which the messages should start; if None, all recorded messages
        :param end_time: the time at which the messages should end; if None, all recorded messages
        :param reverse: if True, the messages will be provided in reversed order
        :return: an iterator over the messages respecting the criteria
        """
        for m in self._filterMessages(start_time=start_time, end_time=end_time, reverse=reverse):
            if m.header.frame_id == frame or m.child_frame_id == frame:
                yield m

    def getFrameStrings(self):
        """
        Returns the IDs of all tf frames
        :return: a set containing all known tf frame IDs
        """
        if self.all_frames is None:
            ret = set()
            for m in self.tf_messages:
                ret.add(m.header.frame_id)
                ret.add(m.child_frame_id)
            self.all_frames = ret

        return self.all_frames

    def getTransformFrameTuples(self):
        """
        Returns all pairs of directly connected tf frames
        :return: a set containing all known tf frame pairs
        """
        if self.all_transform_tuples is None:
            ret = set()
            for m in self.tf_messages:
                ret.add((m.header.frame_id, m.child_frame_id))
            for m in self.tf_static_messages:
                ret.add((m.header.frame_id, m.child_frame_id))
            self.all_transform_tuples = ret
            self.static_transform_tuples = {(m.header.frame_id, m.child_frame_id) for m in self.tf_static_messages}

        return self.all_transform_tuples

    def getTransformGraphInfo(self, time=None):
        """
        Returns the output of TfTransformer.allFramesAsDot() at a given point in time
        :param time: the ROS time at which tf should be queried; if None, it will be the buffer middle time
        :return: A string containing information about the tf tree
        """
        if time is None:
            time = self.getTimeAtPercent(50)
        self.populateTransformerAtTime(time)
        return self.transformer.allFramesAsDot()

    def getStartTime(self):
        """
        Returns the time of the first tf message in the buffer
        :return: the ROS time of the first tf message in the buffer
        """
        return self.tf_messages[0].header.stamp

    def getEndTime(self):
        """
        Returns the time of the last tf message in the buffer
        :return: the ROS time of the last tf message in the buffer
        """
        return self.tf_messages[-1].header.stamp

    @staticmethod
    def _getTimeFromTransforms(transforms):
        return (t.header.stamp for t in transforms)

    def getAverageUpdateFrequency(self, orig_frame, dest_frame, start_time=None, end_time=None):
        """
        Computes the average time between two tf messages directly connecting two given frames
        :param orig_frame: the source tf frame of the transform of interest
        :param dest_frame: the target tf frame of the transform of interest
        :param start_time: the first time at which the messages should be considered; if None, all recorded messages
        :param end_time: the last time at which the messages should be considered; if None, all recorded messages
        :return: the average transform update frequency
        """
        messages = self._filterMessages(orig_frame=orig_frame, dest_frame=dest_frame,
                                        start_time=start_time, end_time=end_time)
        message_times = BagTfTransformer._getTimeFromTransforms(messages)
        message_times = np.array(message_times)
        average_delta = (message_times[1:] - message_times[:-1]).mean()
        return average_delta

    def getTransformUpdateTimes(self, orig_frame, dest_frame, trigger_orig_frame=None, trigger_dest_frame=None,
                                start_time=None, end_time=None, reverse=False):
        """
        Returns the times at which the transform between two frames was updated.
        If the two frames are not directly connected, two directly connected "trigger frames" must be provided.
        The result will be then the update times of the transform between the two frames, but will start at the
        time when the entire transformation chain is complete.
        :param orig_frame: the source tf frame of the transform of interest
        :param dest_frame: the target tf frame of the transform of interest
        :param trigger_orig_frame: the source tf frame of a transform in the chain, directly connected to trigger_dest_frame
        :param trigger_dest_frame: the target tf frame of a transform in the chain, directly connected to trigger_orig_frame
        :param start_time: the first time at which the messages should be considered; if None, all recorded messages
        :param end_time: the last time at which the messages should be considered; if None, all recorded messages
        :param reverse: if True, the times will be provided in reversed order
        :return: an iterator over the times at which the transform is updated
        """
        trigger_frames_were_provided = trigger_orig_frame is not None or trigger_dest_frame is not None
        if trigger_orig_frame is None:
            trigger_orig_frame = orig_frame
        if trigger_dest_frame is None:
            trigger_dest_frame = dest_frame
        if (trigger_dest_frame, trigger_orig_frame) in self.getTransformFrameTuples():
            trigger_orig_frame, trigger_dest_frame = trigger_dest_frame, trigger_orig_frame
        updates = list(self._filterMessages(orig_frame=trigger_orig_frame, dest_frame=trigger_dest_frame,
                                            start_time=start_time, end_time=end_time, reverse=reverse))
        if not updates:
            if trigger_frames_were_provided:
                raise RuntimeError('the provided trigger frames ({}->{}) must be directly connected!'
                                   .format(trigger_orig_frame, trigger_dest_frame))
            else:
                raise RuntimeError('the two frames ({}->{}) are not directly connected! you must provide \
                 directly connected "trigger frames"'.format(trigger_orig_frame, trigger_dest_frame))
        first_update_time = self.waitForTransform(orig_frame, dest_frame, start_time=start_time)
        return (t for t in BagTfTransformer._getTimeFromTransforms(updates) if t > first_update_time)

    def waitForTransform(self, orig_frame, dest_frame, start_time=None):
        """
        Returns the first time for which at least a tf message is available for the whole chain between \
        the two provided frames
        :param orig_frame: the source tf frame of the transform of interest
        :param dest_frame: the target tf frame of the transform of interest
        :param start_time: the first time at which the messages should be considered; if None, all recorded messages
        :return: the ROS time at which the transform is available
        """
        if orig_frame == dest_frame:
            return self.tf_messages[0].header.stamp
        if start_time is not None:
            messages = filter(lambda m: m.header.stamp > start_time, self.tf_messages)
        else:
            messages = self.tf_messages
        missing_transforms = set(self.getChainTuples(orig_frame, dest_frame)) - self.static_transform_tuples
        message = messages.__iter__()
        ret = rospy.Time(0)
        try:
            while missing_transforms:
                m = next(message)
                if (m.header.frame_id, m.child_frame_id) in missing_transforms:
                    missing_transforms.remove((m.header.frame_id, m.child_frame_id))
                    ret = max(ret, m.header.stamp)
                if (m.child_frame_id, m.header.frame_id) in missing_transforms:
                    missing_transforms.remove((m.child_frame_id, m.header.frame_id))
                    ret = max(ret, m.header.stamp)
        except StopIteration:
            raise ValueError('Transform not found between {} and {}'.format(orig_frame, dest_frame))
        return ret

    def lookupTransform(self, orig_frame, dest_frame, time):
        """
        Returns the transform between the two provided frames at the given time
        :param orig_frame: the source tf frame of the transform of interest
        :param dest_frame: the target tf frame of the transform of interest
        :param time: the first time at which the messages should be considered; if None, all recorded messages
        :return: the ROS time at which the transform is available
        """
        if orig_frame == dest_frame:
            return (0, 0, 0), (0, 0, 0, 1)

        self.populateTransformerAtTime(time)
        try:
            common_time = self.transformer.getLatestCommonTime(orig_frame, dest_frame)
        except:
            raise RuntimeError('Could not find the transformation {} -> {} in the 10 seconds before time {}'
                               .format(orig_frame, dest_frame, time))

        return self.transformer.lookupTransform(orig_frame, dest_frame, common_time)

    def lookupTransformWhenTransformUpdates(self, orig_frame, dest_frame,
                                            trigger_orig_frame=None, trigger_dest_frame=None,
                                            start_time=None, end_time=None):
        """
        Returns the transform between two frames every time it updates
        If the two frames are not directly connected, two directly connected "trigger frames" must be provided.
        The result will be then sampled at the update times of the transform between the two frames.
        :param orig_frame: the source tf frame of the transform of interest
        :param dest_frame: the target tf frame of the transform of interest
        :param trigger_orig_frame: the source tf frame of a transform in the chain, directly connected to trigger_dest_frame
        :param trigger_dest_frame: the target tf frame of a transform in the chain, directly connected to trigger_orig_frame
        :param start_time: the first time at which the messages should be considered; if None, all recorded messages
        :param end_time: the last time at which the messages should be considered; if None, all recorded messages
        :return: an iterator over tuples containing the update time and the transform
        """
        update_times = self.getTransformUpdateTimes(orig_frame, dest_frame,
                                                    trigger_orig_frame=trigger_orig_frame,
                                                    trigger_dest_frame=trigger_dest_frame,
                                                    start_time=start_time, end_time=end_time)
        ret = ((t, self.lookupTransform(orig_frame=orig_frame, dest_frame=dest_frame, time=t)) for t in update_times)
        return ret

    def getFrameAncestors(self, frame, early_stop_frame=None):
        """
        Returns the ancestor frames of the given tf frame, until the tree root
        :param frame: ID of the tf frame of interest
        :param early_stop_frame: if not None, stop when this frame is encountered
        :return: a list representing the succession of frames from the tf tree root to the provided one
        """
        frame_chain = [frame]
        chain_link = list(filter(lambda tt: tt[1] == frame, self.getTransformFrameTuples()))
        while chain_link and frame_chain[-1] != early_stop_frame:
            frame_chain.append(chain_link[0][0])
            chain_link = list(filter(lambda tt: tt[1] == frame_chain[-1], self.getTransformFrameTuples()))
        return list(reversed(frame_chain))

    def getChain(self, orig_frame, dest_frame):
        """
        Returns the chain of frames between two frames
        :param orig_frame: the source tf frame of the transform of interest
        :param dest_frame: the target tf frame of the transform of interest
        :return: a list representing the succession of frames between the two passed as argument
        """
        # transformer.chain is apparently bugged
        orig_ancestors = self.getFrameAncestors(orig_frame, early_stop_frame=dest_frame)
        if orig_ancestors[0] == dest_frame:
            return orig_ancestors
        dest_ancestors = self.getFrameAncestors(dest_frame, early_stop_frame=orig_frame)
        if dest_ancestors[0] == orig_frame:
            return dest_ancestors
        if orig_ancestors[0] == dest_ancestors[-1]:
            return list(reversed(dest_ancestors)) + orig_ancestors[1:]
        if dest_ancestors[0] == orig_ancestors[-1]:
            return list(reversed(orig_ancestors)) + dest_ancestors[1:]
        while len(dest_ancestors) > 0 and orig_ancestors[0] == dest_ancestors[0]:
            if len(orig_ancestors) > 1 and len(dest_ancestors) > 1 and orig_ancestors[1] == dest_ancestors[1]:
                orig_ancestors.pop(0)
            dest_ancestors.pop(0)
        return list(reversed(orig_ancestors)) + dest_ancestors

    def getChainTuples(self, orig_frame, dest_frame):
        """
        Returns the chain of frame pairs representing the transforms connecting two frames
        :param orig_frame: the source tf frame of the transform chain of interest
        :param dest_frame: the target tf frame of the transform chain of interest
        :return: a list of frame ID pairs representing the succession of transforms between the frames passed as argument
        """
        chain = self.getChain(orig_frame, dest_frame)
        return zip(chain[:-1], chain[1:])

    @staticmethod
    def averageTransforms(transforms):
        """
        Computes the average transform over the ones passed as argument
        :param transforms: a list of transforms
        :return: a transform having the average value
        """
        if not transforms:
            raise RuntimeError('requested average of an empty vector of transforms')
        transforms = list(transforms)
        translations = np.array([t[0] for t in transforms])
        quaternions = np.array([t[1] for t in transforms])
        mean_translation = translations.mean(axis=0).tolist()
        mean_quaternion = quaternions.mean(axis=0)  # I know, it is horrible.. but for small rotations shouldn't matter
        mean_quaternion = (mean_quaternion / np.linalg.norm(mean_quaternion)).tolist()
        return mean_translation, mean_quaternion

    def averageTransformOverTime(self, orig_frame, dest_frame, start_time, end_time,
                                 trigger_orig_frame=None, trigger_dest_frame=None):
        """
        Computes the average value of the transform between two frames
        If the two frames are not directly connected, two directly connected "trigger frames" must be provided.
        The result will be then sampled at the update times of the transform between the two frames, but will start at the
        time when the entire transformation chain is complete.
        :param orig_frame: the source tf frame of the transform of interest
        :param dest_frame: the target tf frame of the transform of interest
        :param start_time: the start time of the averaging time range
        :param end_time: the end time of the averaging time range
        :param trigger_orig_frame: the source tf frame of a transform in the chain, directly connected to trigger_dest_frame
        :param trigger_dest_frame: the target tf frame of a transform in the chain, directly connected to trigger_orig_frame
        :return: the average value of the transformation over the specified time range
        """
        if orig_frame == dest_frame:
            return (0, 0, 0), (0, 0, 0, 1)
        update_times = self.getTransformUpdateTimes(orig_frame=orig_frame, dest_frame=dest_frame,
                                                    start_time=start_time, end_time=end_time,
                                                    trigger_orig_frame=trigger_orig_frame,
                                                    trigger_dest_frame=trigger_dest_frame)
        target_transforms = (self.lookupTransform(orig_frame=orig_frame, dest_frame=dest_frame, time=t)
                             for t in update_times)
        return self.averageTransforms(target_transforms)

    def replicateTransformOverTime(self, transf, orig_frame, dest_frame, frequency, start_time=None, end_time=None):
        """
        Adds a new transform to the graph with the specified value
        This can be useful to add calibration a-posteriori.
        :param transf: value of the transform
        :param orig_frame: the source tf frame of the transform of interest
        :param dest_frame: the target tf frame of the transform of interest
        :param frequency: frequency at which the transform should be published
        :param start_time: the time the transform should be published from
        :param end_time: the time the transform should be published until
        :return:
        """
        if start_time is None:
            start_time = self.getStartTime()
        if end_time is None:
            end_time = self.getEndTime()
        transl, quat = transf
        time_delta = rospy.Duration(1 / frequency)

        t_msg = TransformStamped(header=Header(frame_id=orig_frame),
                                 child_frame_id=dest_frame,
                                 transform=Transform(translation=Vector3(*transl), rotation=Quaternion(*quat)))

        def createMsg(time_nsec):
            time = rospy.Time(time_nsec / 1000000000)
            t_msg2 = copy.deepcopy(t_msg)
            t_msg2.header.stamp = time
            return t_msg2

        new_msgs = [createMsg(t) for t in range(start_time.to_nsec(), end_time.to_nsec(), time_delta.to_nsec())]
        self.tf_messages += new_msgs
        self.tf_messages.sort(key=lambda tfm: tfm.header.stamp.to_nsec())
        self.tf_times = np.array(list((tfm.header.stamp.to_nsec() for tfm in self.tf_messages)))
        self.all_transform_tuples.add((orig_frame, dest_frame))

    def processTransform(self, callback, orig_frame, dest_frame,
                         trigger_orig_frame=None, trigger_dest_frame=None, start_time=None, end_time=None):
        """
        Looks up the transform between two frames and forwards it to a callback at each update
        :param callback: a function taking two arguments (the time and the transform as a tuple of translation and rotation)
        :param orig_frame: the source tf frame of the transform of interest
        :param dest_frame: the target tf frame of the transform of interest
        :param start_time: the start time of the time range
        :param end_time: the end time of the time range
        :param trigger_orig_frame: the source tf frame of a transform in the chain, directly connected to trigger_dest_frame
        :param trigger_dest_frame: the target tf frame of a transform in the chain, directly connected to trigger_orig_frame
        :return: an iterator over the result of calling the callback with the looked up transform as argument
        """
        times = self.getTransformUpdateTimes(orig_frame, dest_frame, trigger_orig_frame, trigger_dest_frame,
                                             start_time=start_time, end_time=end_time)
        transforms = [(t, self.lookupTransform(orig_frame=orig_frame, dest_frame=dest_frame, time=t)) for t in times]
        for time, transform in transforms:
            yield callback(time, transform)

    def plotTranslation(self, orig_frame, dest_frame, axis=None,
                        trigger_orig_frame=None, trigger_dest_frame=None, start_time=None, end_time=None,
                        fig=None, ax=None, color='blue'):
        """
        Creates a 2D or 3D plot of the trajectory described by the values of the translation of the transform over time
        :param orig_frame: the source tf frame of the transform of interest
        :param dest_frame: the target tf frame of the transform of interest
        :param axis: if None, the plot will be 3D; otherwise, it should be 'x', 'y', or 'z': the value will be plotted over time
        :param trigger_orig_frame: the source tf frame of a transform in the chain, directly connected to trigger_dest_frame
        :param trigger_dest_frame: the target tf frame of a transform in the chain, directly connected to trigger_orig_frame
        :param start_time: the start time of the time range
        :param end_time: the end time of the time range
        :param fig: if provided, the Matplotlib figure will be reused; otherwise a new one will be created
        :param ax: if provided, the Matplotlib axis will be reused; otherwise a new one will be created
        :param color: the color of the line
        :return:
        """
        import matplotlib.pyplot as plt
        if axis is None:
            # 3D
            from mpl_toolkits.mplot3d import Axes3D
            translation_data = np.array(list(self.processTransform(lambda t, tr: (tr[0]),
                                                                   orig_frame=orig_frame, dest_frame=dest_frame,
                                                                   trigger_orig_frame=trigger_orig_frame,
                                                                   trigger_dest_frame=trigger_dest_frame,
                                                                   start_time=start_time, end_time=end_time)))
            if fig is None:
                fig = plt.figure()
            if ax is None:
                ax = fig.add_subplot(111, projection='3d')

            ax.scatter(
                translation_data[:, 0],
                translation_data[:, 1],
                translation_data[:, 2],
                c=color
            )
            return ax, fig
        else:
            translation_data = np.array(list(self.processTransform(lambda t, tr: (t.to_nsec(), tr[0][axis]),
                                                                   orig_frame=orig_frame, dest_frame=dest_frame,
                                                                   trigger_orig_frame=trigger_orig_frame,
                                                                   trigger_dest_frame=trigger_dest_frame,
                                                                   start_time=start_time, end_time=end_time)))
            if fig is None:
                fig = plt.figure()
            if ax is None:
                ax = fig.add_subplot(111)
            ax.plot(translation_data[:, 0], translation_data[:, 1], color=color)
            return ax, fig

def set_debug_output(key):
    if bool(rospy.get_param(f"{rospy.get_name()}/{key}")):
        rospy.wait_for_service(f"{rospy.get_name()}/set_logger_level")
        s = rospy.ServiceProxy(f"{rospy.get_name()}/set_logger_level", SetLoggerLevel)
        s.call('rosout', 'DEBUG')

def create_service_client(self, topic, service_type):
    rospy.loginfo(f"{self._tag} - rospy.wait_for_service({topic})")
    rospy.wait_for_service(topic)
    rospy.loginfo(f"{self._tag} - service {topic} is ready.")
    return rospy.ServiceProxy(topic, service_type)
