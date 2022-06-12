#
#  Copyright (C) 1997-2016 JDE Developers Team
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see http://www.gnu.org/licenses/.
#  Authors :
#       Alberto Martin Florido <almartinflorido@gmail.com>
#       Aitor Martinez Fernandez <aitor.martinez.fernandez@gmail.com>
#

import rospy
from std_msgs.msg import Float64
import threading
from math import pi as PI
from .threadPublisher import ThreadPublisher

class PublisherMotors(object):
 
    def __init__(self, topic, clock):
        self.topic = topic
        self.data = 0.0
        self.pub = rospy.Publisher(self.topic, Float64, queue_size=1)
        self.lock = threading.Lock()

        self.kill_event = threading.Event()
        self.thread = ThreadPublisher(self, self.kill_event, clock)

        self.thread.daemon = True
        self.start()
 
    def publish (self):

        self.lock.acquire()
        tw = self.data
        self.lock.release()
        self.pub.publish(tw)
        
    def stop(self):
   
        self.pub.unregister()
        self.kill_event.set()

    def start (self):

        self.kill_event.clear()
        self.thread.start()
        
    def getPosition(self):
        self.lock.acquire()
        data = self.data
        self.lock.release()
        
        return data

    def sendPosition(self, position):
        self.lock.acquire()
        self.data = position
        self.lock.release()
