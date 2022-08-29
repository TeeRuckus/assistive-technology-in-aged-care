import rospy
from std_msgs.msg import UInt8MultiArray, UInt16MultiArray, Int16MultiArray, String
import numpy as np
import time
import sys
import os
#TODO: you will need to find out what this is actually going to do in code
import hashlib

#going to hard code the path to the signals which MiRo will require
HELP_SIGNAL_FILE = "test_signal.mp3"
HELP_SIGNAL_PATH = "../../share/media/test_signal.mp3"
BUFFER_STUFF_BYTES = 4000
MAX_STREAM_MSG_SIZE = (4096 - 48)
#this is taken from the mdk/bin/shared/client_stream.py file 
class Streamer():

    def __init__(self):
        #loading in the wav file into memory
        fileName = "/tmp/" + HELP_SIGNAL_FILE + ".decode"

        #TODO: You will need to have a look at what that os.path stuff is going to do


        #loading the wave file into the program 
        with open(fileName, "rb") as inStrm:
            dat = inStrm.read()
        self.data_r = 0

        # convert to numpy array
        dat = np.fromstring(dat, dtype='int16').astype(np.int32)

        # normalise wav
        dat = dat.astype(np.float)
        sc = 32767.0 / np.max(np.abs(dat))
        dat *= sc
        dat = dat.astype(np.int16).tolist()

        # breakdown of digit file is recovered manually



        """
        # handle digits
        if digits:

            # start with some blank so we don't miss beginning
            dat_ = dat
            dat = [dat_[0] * 0] * 8000
            gap = [dat_[0] * 0] * 50

            # now add the digits
            for d in digits:
                if d in "0123456789":
                    s = digits_and_dot[int(d) - int("0")]
                    dat += dat_[s[0]:s[1]]
                    dat += gap
                if d == ".":
                    s = digits_and_dot[10]
                    dat += dat_[s[0]:s[1]]
                    dat += gap
        """

        # store
        self.data = dat

        # state
        self.buffer_space = 0
        self.buffer_total = 0

        # get robot name
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

        # publish
        topic = topic_base_name + "/control/stream"
        print ("publish", topic)
        self.pub_stream = rospy.Publisher(topic, Int16MultiArray, queue_size=0)

        #I am not going to worry about this at the current moment, I will come back to it later
        # subscribe
        topic = topic_base_name + "/platform/log"
        print ("subscribe", topic)
        self.sub_log = rospy.Subscriber(topic, String, self.callback_log, queue_size=5, tcp_nodelay=True)

        # subscribe
        topic = topic_base_name + "/sensors/stream"
        print ("subscribe", topic)
        self.sub_stream = rospy.Subscriber(topic, UInt16MultiArray, self.callback_stream, queue_size=1, tcp_nodelay=True)

    def playSound(self):
        count = 0
        exit = False

        # safety dropout if receiver not present
        dropout_data_r = -1
        dropout_count = 3

        # loop
        while not rospy.core.is_shutdown() :
            if not os.path.isfile(HELP_SIGNAL_FILE):
                exit = True
            # if we've received a report
            if self.buffer_total > 0:
                # compute amount to send
                buffer_rem = self.buffer_total - self.buffer_space
                n_bytes = BUFFER_STUFF_BYTES - buffer_rem
                n_bytes = max(n_bytes, 0)
                n_bytes = min(n_bytes, MAX_STREAM_MSG_SIZE)
                # if amount to send is non-zero
                if n_bytes > 0:
                    msg = Int16MultiArray(data = self.data[self.data_r:self.data_r+n_bytes])
                    self.pub_stream.publish(msg)
                    self.data_r += n_bytes
            # break
            if self.data_r >= len(self.data):
                exit = True

            # report once per second
            if count == 0:
                count = 10
                print ("streaming:", self.data_r, "/", len(self.data), "bytes")

                # check at those moments if we are making progress, also
                if dropout_data_r == self.data_r:
                    if dropout_count == 0:
                        print ("dropping out because of no progress...")
                        exit = True 
                    print ("dropping out in", str(dropout_count) + "...")
                    dropout_count -= 1
                else:
                    dropout_data_r = self.data_r

            # count tenths
            count -= 1
            time.sleep(0.1)

    def callback_log(self, msg):
        sys.stdout.write(msg.data)
        sys.stdout.flush()

    def callback_stream(self, msg):
        self.buffer_space = msg.data[0]
        self.buffer_total = msg.data[1]

if __name__ == "__main__":
    rospy.init_node("test_node", anonymous=True)
    main = Streamer()
    main.playSound()

