import rospy
from std_msgs.msg import UInt8MultiArray, UInt16MultiArray, Int16MultiArray, String
from std_msgs.msg import Bool
import numpy as np
import time
import sys
import os


NODE_NAME = "MiROVoice"

#from Errors import MiRoError



#TODO: you will need to make the naming conventions here all consistent with each other
#TODO: you will need to get rid of depreciated method from the program and file 

#going to hard code the path to the signals which MiRo will require
HELP_SIGNAL_FILE = "test_signal.mp3"
#the path of the file which you want to play

HELP_SIGNAL_PATH = "test_signal.mp3"

BUFFER_STUFF_BYTES = 4000
MAX_STREAM_MSG_SIZE = (4096 - 48)
#this is taken from the mdk/bin/shared/client_stream.py file 
class Streamer():
    def __init__(self):
        rospy.init_node(NODE_NAME, anonymous=True)
        self.__playWarningSignal = False
        
        #SUBSCRIBERS
        rospy.Subscriber("resident/warningSignal/",Bool,self.callBackWarningSignal)

        #PUBLISHERS

        #loading in the wav file into memory
        fileName = "/tmp/" + HELP_SIGNAL_FILE + ".decode"

        if not os.path.isfile(fileName):
            cmd = "ffmpeg -y -i \"" + HELP_SIGNAL_PATH + "\" -f s16le -acodec pcm_s16le -ar 8000 -ac 1 \"" + fileName + "\""

            os.system(cmd)
            if not os.path.isfile(fileName):
                rospy.loginfo("failed decode mp3")
                sys.exit(0)

        #creating the file to decode the sound from

        #loading the wave file into the program 
        dat = self.readBinary(fileName)
        self.dataR = 0
        # convert to numpy array
        dat = np.fromstring(dat, dtype='int16').astype(np.int32)
        # normalise wav
        dat = dat.astype(np.float)
        sc = 32767.0 / np.max(np.abs(dat))
        dat *= sc
        dat = dat.astype(np.int16).tolist()

        # store
        self.data = dat

        # state
        self.bufferSpace = 0
        self.bufferTotal = 0

        # get robot name
        topicBaseName = "/" + os.getenv("MIRO_ROBOT_NAME")

        # publish
        topic = topicBaseName + "/control/stream"
        self.pubStream = rospy.Publisher(topic, Int16MultiArray, queue_size=0)

        topic = topicBaseName + "/platform/log"
        self.subLog = rospy.Subscriber(topic, String, self.callback_log, queue_size=5, tcp_nodelay=True)

        topic = topicBaseName + "/sensors/stream"
        self.subStream = rospy.Subscriber(topic, UInt16MultiArray,
                self.callback_stream, queue_size=1, tcp_nodelay=True)

        #setting up required variables 

    @property
    def playWarningSignal(self):
        """
        ASSERTION:
        """
        return self.__playWarningSignal

    @playWarningSignal.setter
    def playWarningSignal(self, inSound):
        """
        ASSERTION:
        """
        self.__playWarningSignal = self.__validateSound(inSound)

    def playSound(self):

        count = 0
        exit = False

        # safety dropout if receiver not present
        dropout_dataR = -1
        dropout_count = 3

        # loop
        while not rospy.core.is_shutdown() :
            #we only want to play the warning signal once we have subscribed
            #from the signal
            if self.__playWarningSignal:
                if not os.path.isfile(HELP_SIGNAL_FILE):
                    exit = True
                # if we've received a report
                if self.bufferTotal > 0:
                    # compute amount to send
                    buffer_rem = self.bufferTotal - self.bufferSpace
                    n_bytes = BUFFER_STUFF_BYTES - buffer_rem
                    n_bytes = max(n_bytes, 0)
                    n_bytes = min(n_bytes, MAX_STREAM_MSG_SIZE)
                    # if amount to send is non-zero
                    if n_bytes > 0:
                        msg = Int16MultiArray(data = self.data[self.dataR:self.dataR+n_bytes])
                        self.pubStream.publish(msg)
                        self.dataR += n_bytes
                # break
                if self.dataR >= len(self.data):
                    exit = True

                # report once per second
                if count == 0:
                    count = 10
                    print ("streaming:", self.dataR, "/", len(self.data), "bytes")

                    # check at those moments if we are making progress, also
                    if dropout_dataR == self.dataR:
                        if dropout_count == 0:
                            print ("dropping out because of no progress...")
                            exit = True 
                        print ("dropping out in", str(dropout_count) + "...")
                        dropout_count -= 1
                    else:
                        dropout_dataR = self.dataR

                # count tenths
                count -= 1
            time.sleep(0.1)



    def callback_log(self, msg):
        """
        PURPOSE:
        """
        sys.stdout.write(msg.data)
        sys.stdout.flush()

    def callback_stream(self, msg):
        """
        PURPOSE:
        """
        self.bufferSpace = msg.data[0]
        self.bufferTotal = msg.data[1]

    def readBinary(self, fileName):
        """
        PURPOSE:
        """
        dat = None
        with open(fileName, "rb") as inStrm:
            dat = inStrm.read()

        return dat

    def callBackWarningSignal(self, data):
        """
        PURPOSE:
        """
        self.__playWarningSignal = bool(data.data)


    def __validateSound(self, inSound):
        """
        PURPOSE:
        """
        if inSound is None:
            self.soundError("you can't set sound to a none object")

        if not isinstance(inSound, bool):
            self.soundError("In sound must be a boolean variable")

        return inSound

    def soundError(self, msg):
        """
        ASSERTIONS: will display the error message to the terminal and exit program
        """
        rospy.loginfo("ERROR: MiRoVoice.py: %s " % msg)
        sys.exit(0)



if __name__ == "__main__":
    soundInterface = Streamer()
    soundInterface.playSound()



