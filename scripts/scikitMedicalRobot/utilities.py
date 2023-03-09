import json
import os
import logging
import uuid
import time
from datetime import datetime
from pathlib import Path
from itertools import repeat
from collections import OrderedDict
import IPython.display
import numpy as np
import functools
import heapq

def validation_dir(data_dir, training_file='training.pt', testing_file='testing.pt'):
    raw_dir = data_dir + 'raw'
    processed_dir = data_dir + 'processed'
    child_dir = os.listdir(data_dir)
    if 'raw' not in child_dir:
        os.mkdir(raw_dir)
    if 'processed' not in child_dir:
        os.mkdir(processed_dir)

    child_dir = os.listdir(processed_dir)
    if (training_file in child_dir) and (testing_file in child_dir):
        return (True, raw_dir, processed_dir)

    return (False, raw_dir, processed_dir)


def ensure_dir(dirname):
    dirname = Path(dirname)
    if not dirname.is_dir():
        dirname.mkdir(parents=True, exist_ok=False)


def generate_uuid():
    return str(uuid.uuid4())


def generate_datetime():
    return datetime.fromtimestamp(time.time()).strftime(f'%Y%m%d_%H%M%S_%f')


def generate_cache_file(filename, caches_dir='caches', child_dir=None, filetype='', suffix='', prefix=''):
    if child_dir is not None:
        cache = Path(caches_dir) / Path(child_dir)
    else:
        cache = Path(caches_dir)
    cache.mkdir(parents=True, exist_ok=True)
    return cache / Path(
        f"{prefix}{'_' if prefix != '' else ''}{filename}{'_' if suffix != '' else ''}{suffix}{'.' if filetype != '' else ''}{filetype}")


def generate_random_cache_file(caches_dir='caches', child_dir=None, filetype='', suffix='', prefix=''):
    return generate_cache_file(generate_uuid(), caches_dir, child_dir, filetype, suffix, prefix)


def generate_timedate_cache_file(caches_dir='caches', child_dir=None, filetype='', suffix='', prefix=''):
    return generate_cache_file(generate_datetime(), caches_dir, child_dir, filetype, suffix, prefix)


def read_json(fname):
    fname = Path(fname)
    with fname.open('rt') as handle:
        return json.load(handle, object_hook=OrderedDict)


def write_json(content, fname):
    fname = Path(fname)
    with fname.open('wt') as handle:
        json.dump(content, handle, indent=4, sort_keys=False)


def inf_loop(data_loader):
    ''' wrapper function for endless data loader. '''
    for loader in repeat(data_loader):
        yield from loader


def create_simple_logger(logger_name='Test'):
    logger = logging.getLogger(logger_name)
    logger.setLevel(logging.DEBUG)
    streamHandler = logging.StreamHandler()
    basic_format = '%(asctime)s [%(levelname)s] %(filename)s [line:%(lineno)d] %(message)s'
    datefmt = '%Y-%m-%d %H:%M:%S'
    streamHandler.setFormatter(logging.Formatter(fmt=basic_format, datefmt=datefmt))
    logger.addHandler(streamHandler)
    return logger


def mu_law_encoding(data, mu):
    return np.sign(data) * np.log(1 + mu * np.abs(data)) / np.log(mu + 1)


def mu_law_decoding(data, mu):
    return np.sign(data) * (np.exp(np.abs(data) * np.log(mu + 1)) - 1) / mu

def timeit(func):
    @functools.wraps(func)
    def wrapper(*args, **kargs):
        start = time.time()
        result = func(*args, **kargs)
        stop = time.time()
        print(stop - start)
        return result
    return wrapper

'''
example:
@notNoneAttrs('var1_name', 'var2_name', ...)
def func():
    pass
'''
def notNoneAttrs(*notNoneAttrs):
    def decorator(func):
        @functools.wraps(func)
        def wrapper(self, *args, **kwargs):
            for notNoneAttr in notNoneAttrs:
                if getattr(self, notNoneAttr) == None:
                    return None
            return func(self, *args, **kwargs)
        return wrapper
    return decorator

def quick_reset_ur_robot(urcap_fn='ros.urp', sim_experiment=False):
    import rospy
    from std_srvs.srv import Trigger
    from ur_dashboard_msgs.srv import LoadRequest, Load
    load_req = LoadRequest()
    load_req.filename = urcap_fn

    if sim_experiment:
        rospy.ServiceProxy('/ur_hardware_interface/dashboard/power_off', Trigger).call()
        rospy.sleep(0.1)
        rospy.ServiceProxy('/ur_hardware_interface/dashboard/power_on', Trigger).call()
        rospy.sleep(0.1)
        rospy.ServiceProxy('/ur_hardware_interface/dashboard/brake_release', Trigger).call()
        rospy.sleep(0.1)

    rospy.ServiceProxy('/ur_hardware_interface/dashboard/stop', Trigger).call()
    rospy.sleep(0.1)
    rospy.ServiceProxy('/ur_hardware_interface/dashboard/load_program', Load).call(load_req)
    rospy.sleep(0.1)
    rospy.ServiceProxy('/ur_hardware_interface/dashboard/play', Trigger).call()
    rospy.sleep(0.1)

    # if sim_experiment:
    #     global rc, rkdl
    #     rc = robot_control(argv=sys.argv, group_name='manipulator', ee_link_name='tool0')
    #     rkdl = robot_kdl_kinematics(param_robot_description='robot_description', chain_link=('base_link', 'tool0'))

def Audio(audio: np.ndarray, sr: int):
    """
    Use instead of IPython.display.Audio as a workaround for VS Code.
    `audio` is an array with shape (channels, samples) or just (samples,) for mono.
    """

    if np.ndim(audio) == 1:
        channels = [audio.tolist()]
    else:
        channels = audio.tolist()

    return IPython.display.HTML("""
        <script>
            if (!window.audioContext) {
                window.audioContext = new AudioContext();
                const source = audioContext.createBufferSource();
                window.playAudio = function(audioChannels, sr) {
                    const buffer = audioContext.createBuffer(audioChannels.length, audioChannels[0].length, sr);
                    for (let [channel, data] of audioChannels.entries()) {
                        buffer.copyToChannel(Float32Array.from(data), channel);
                    }
                    source.buffer = buffer;
                    source.connect(audioContext.destination);
                    source.start();
                }
                window.stopAudio = function() {
                    source.stop();
                }
            }
        </script>
        <button onclick="playAudio(%s, %s)">Play</button>
        <button onclick="stopAudio()">Stop</button>
    """ % (json.dumps(channels), sr))

def mergesort(list_of_lists, key=None):
    """
    Perform an N-way merge operation on sorted lists.
    @param list_of_lists: (really iterable of iterable) of sorted elements
    (either by naturally or by C{key})
    @param key: specify sort key function (like C{sort()}, C{sorted()})
    @param iterfun: function that returns an iterator.
    Yields tuples of the form C{(item, iterator)}, where the iterator is the
    built-in list iterator or something you pass in, if you pre-generate the
    iterators.
    This is a stable merge; complexity O(N lg N)
    Examples::
    print list(x[0] for x in mergesort([[1,2,3,4],
                                        [2,3.5,3.7,4.5,6,7],
                                        [2.6,3.6,6.6,9]]))
    [1, 2, 2, 2.6, 3, 3.5, 3.6, 3.7, 4, 4.5, 6, 6.6, 7, 9]
    # note stability
    print list(x[0] for x in mergesort([[1,2,3,4],
                                        [2,3.5,3.7,4.5,6,7],
                                        [2.6,3.6,6.6,9]], key=int))
    [1, 2, 2, 2.6, 3, 3.5, 3.6, 3.7, 4, 4.5, 6, 6.6, 7, 9]
    print list(x[0] for x in mergesort([[4,3,2,1],
                                        [7,6.5,4,3.7,3.3,1.9],
                                        [9,8.6,7.6,6.6,5.5,4.4,3.3]],
                                        key=lambda x: -x))
    [9, 8.6, 7.6, 7, 6.6, 6.5, 5.5, 4.4, 4, 4, 3.7, 3.3, 3.3, 3, 2, 1.9, 1]
    """

    heap = []
    for i, itr in enumerate(iter(pl) for pl in list_of_lists):
        try:
            item = next(itr)
            toadd = (key(item), i, item, itr) if key else (item, i, itr)
            heap.append(toadd)
        except StopIteration:
            pass
    heapq.heapify(heap)

    if key:
        while heap:
            _, idx, item, itr = heap[0]
            yield item, itr
            try:
                item = next(itr)
                heapq.heapreplace(heap, (key(item), idx, item, itr) )
            except StopIteration:
                heapq.heappop(heap)

    else:
        while heap:
            item, idx, itr = heap[0]
            yield item, itr
            try:
                heapq.heapreplace(heap, (next(itr), idx, itr))
            except StopIteration:
                heapq.heappop(heap)
