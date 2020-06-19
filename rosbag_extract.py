#!/usr/bin/env python
from __future__ import print_function

import os
import sys
import cv2

import rosbag
import cv_bridge

bridge = cv_bridge.CvBridge()

if len(sys.argv) < 3:
  print('Missing input arguments. Usage: extract input_directory output_directory', file=sys.stderr)
  sys.exit(1)

files = filter(lambda f: f.endswith('.bag'), os.listdir(sys.argv[1]))
print(files)
for f in files:
  bag = rosbag.Bag(os.path.join(sys.argv[1], f))
  info = bag.get_type_and_topic_info()
  image_topics = map(lambda t: t[0], filter(lambda t: t[1] == 'sensor_msgs/Image', [(key, info.topics[key].msg_type)  for key in info.topics]))
  #command_topics = map(lambda t: t[0], filter(lambda t: t[1] == 'Change to the topic that is recording the steering agles', [(key, info.topics[key].msg_type)  for key in info.topics]))

  counts = {}
  for topic, msg, t in bag.read_messages(image_topics):
    if topic not in counts:
      counts[topic] = 0

    im = bridge.imgmsg_to_cv2(msg)

    if not os.path.isdir(os.path.join(sys.argv[2], topic.replace('/', '_')[1:])):
      os.mkdir(os.path.join(sys.argv[2], topic.replace('/', '_')[1:]))

    print(os.path.join(sys.argv[2], topic.replace('/', '_')[1:], f[0:f.index('.')] + '_{}.{}'.format(counts[topic], 'tiff' if msg.encoding == '16UC1' else 'png')))
    cv2.imwrite(os.path.join(sys.argv[2], topic.replace('/', '_')[1:], f[0:f.index('.')] + '_{}.{}'.format(counts[topic], 'tiff' if msg.encoding == '16UC1' else 'png')), im)
    
    counts[topic] += 1

  bag.close()
