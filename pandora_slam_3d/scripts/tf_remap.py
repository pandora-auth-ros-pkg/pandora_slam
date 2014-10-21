#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#


## remap a tf topic

import roslib; roslib.load_manifest('tf')

import rospy
from tf.msg import tfMessage

class TfRemapper:
    def __init__(self):
        self.pub = rospy.Publisher('/tf', tfMessage)

        self.ignore = rospy.get_param('~ignore', [])

        mappings = rospy.get_param('~mappings', [])
        self.mappings = {}
        
        for i in mappings:
            if "old" in i and "new" in i:
                self.mappings[i["old"]] = i["new"]

        print "Applying the following mappings to incoming tf frame ids", self.mappings
        print "Ignore frames", self.ignore
        rospy.Subscriber("/tf_old", tfMessage, self.callback)


    def callback(self, tf_msg):
        new_tf_msg = tfMessage()
        for transform in tf_msg.transforms:
            if transform.header.frame_id not in self.ignore and transform.child_frame_id  not in self.ignore:
                new_tf_msg.transforms.append(transform)

        for transform in new_tf_msg.transforms:
            if transform.header.frame_id in self.mappings:
                transform.header.frame_id = self.mappings[transform.header.frame_id]
            if transform.child_frame_id  in self.mappings:
                transform.child_frame_id = self.mappings[transform.child_frame_id]
                
        self.pub.publish(new_tf_msg)

def remap_tf():
    
    pub.publish(Empty())

        
if __name__ == '__main__':
    rospy.init_node('tf_remapper')
    tfr = TfRemapper()
    rospy.spin()
