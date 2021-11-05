#!/usr/bin/env python3

"""
recognizer.py is a wrapper for pocketsphinx.
  parameters:
    ~lm - filename of language model
    ~dict - filename of dictionary
    ~mic_name - set the pulsesrc device name for the microphone input.
                e.g. a Logitech G35 Headset has the following device name: alsa_input.usb-Logitech_Logitech_G35_Headset-00-Headset_1.analog-mono
                To list audio device info on your machine, in a terminal type: pacmd list-sources
  publications:
    ~output (std_msgs/String) - text output
  services:
    ~start (std_srvs/Empty) - start speech recognition
    ~stop (std_srvs/Empty) - stop speech recognition
"""

import roslib; roslib.load_manifest('pocketsphinx')
import rospy

import gi
gi.require_version('Gtk', '2.0')
gi.require_version('Gst', '1.0')
from gi.repository import Gtk, Gst
Gst.init(None)

from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyResponse

class recognizer(object):
    """ GStreamer based speech recognizer. """

    def __init__(self):
        # Start node
        rospy.init_node("recognizer")

        self._lm_param = "~lm"
        self._dict_param = "~dict"

        self.launch_config = f"tcpserversrc host={rospy.get_param('~host')} port={rospy.get_param('~port')}"\
            " ! rawaudioparse num-channels=1 ! audioconvert ! audioresample ! pocketsphinx name=asr ! fakesink"

        # Configure ROS settings
        self.started = False
        rospy.on_shutdown(self.shutdown)
        self.pub = rospy.Publisher('~output', String, queue_size=1)
        rospy.Service("~start", Empty, self.start)
        rospy.Service("~stop", Empty, self.stop)

        self.start_recognizer()

    def start_recognizer(self):
        rospy.loginfo("Starting recognizer... ")

        self.pipeline = Gst.parse_launch(self.launch_config)
        self.bus = self.pipeline.get_bus()
        self.bus.add_signal_watch()
        self.err_id = self.bus.connect('message::error', self.error)
        self.bus_id = self.bus.connect('message::element', self.message)

        # Configure language model
        self.asr = self.pipeline.get_by_name('asr')
        self.asr.set_property('dsratio', 1)
        if rospy.has_param(self._lm_param):
            self.asr.set_property('lm', rospy.get_param(self._lm_param))
        if rospy.has_param(self._dict_param):
            self.asr.set_property('dict', rospy.get_param(self._dict_param))

        self.pipeline.set_state(Gst.State.PLAYING)
        self.started = True

    def stop_recognizer(self):
        if self.started:
            self.pipeline.set_state(Gst.STATE_NULL)
            self.pipeline.remove(self.asr)
            self.bus.disconnect(self.bus_id)
            self.bus.disconnect(self.err_id)
            self.started = False

    def shutdown(self):
        """ Delete any remaining parameters so they don't affect next launch """
        for param in [self._lm_param, self._dict_param]:
            if rospy.has_param(param):
                rospy.delete_param(param)

        """ Shutdown the GTK thread. """
        Gtk.main_quit()

    def start(self, req):
        self.start_recognizer()
        rospy.loginfo("recognizer started")
        return EmptyResponse()

    def stop(self, req):
        self.stop_recognizer()
        rospy.loginfo("recognizer stopped")
        return EmptyResponse()

    def error(self, bus, msg):
        print(f"Error: {msg.src.name} - {msg.parse_error()}")

    def message(self, bus, msg):
        """Receive element messages from the bus."""
        msgtype = msg.get_structure().get_name()
        if msgtype != 'pocketsphinx':
            return

        if msg.get_structure().get_value('final'):
            self.final_result(
                msg.get_structure().get_value('hypothesis'),
                msg.get_structure().get_value('confidence'))
        elif msg.get_structure().get_value('hypothesis'):
            self.partial_result(msg.get_structure().get_value('hypothesis'))

    def partial_result(self, hyp):
        """Log partial results to stdout."""
        rospy.loginfo("Partial: " + hyp)

    def final_result(self, hyp, confidence):
        """Publish the final result."""
        rospy.loginfo("Final: " + hyp)
        msg = String()
        msg.data = str(hyp.lower())
        self.pub.publish(msg)

if __name__ == "__main__":
    start = recognizer()
    Gtk.main()
