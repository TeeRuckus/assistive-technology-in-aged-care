import rospy
from std_msgs.msg import UInt8MultiArray, UInt16MultiArray, Int16MultiArray, String
from std_msgs.msg import Bool
import numpy as np
import time
import sys
import os
from gtts import gTTS
#various API's for speech recognition
import speech_recognition as sr
#offline speech recognition engine 
from pocketsphinx import LiveSpeech


NODE_NAME = "MiROVoice"

MIRO_INTRO ="Hello, I am MiRo. I have seen that you have fallen over. Are you Okay?  Tap me on the head if you're okay otherwise, I can go and get help. If you prefer you can respond with yes or no, or sorry to hear the instructions again."
#from Errors import MiRoError



#TODO: you will need to make the naming conventions here all consistent with each other
#TODO: you will need to get rid of depreciated method from the program and file 
#TODO: you will need to have a setter in here which will set the mode of the robot. Whether to use the online API, or to just use the offline version

#going to hard code the path to the signals which MiRo will require
HELP_SIGNAL_FILE = "test_signal.mp3"
#the path of the file which you want to play

HELP_SIGNAL_PATH = "test_signal.mp3"

BUFFER_STUFF_BYTES = 4000
MAX_STREAM_MSG_SIZE = (4096 - 48)
#this is taken from the mdk/bin/shared/client_stream.py file 
class Streamer():

    #TODO: you will need to do a search to determine if you have used self infront of each word
    hotKeys = ["YES", "NO", "YEAH", "NAH", "SORRY"]
    responses = {
            "YES" : "I am glad that you're okay. I am switching off now.",
            "NO" : "Please wait for a moment. I am going to get help.",
            #this will be wherever the intro file is located
            "SORRY" : MIRO_INTRO
            }

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

    def determineResponse(self, response):
        """
        PURPOSE: To look for hot words inside the resident's speech, and depending
        on the hot words found MiRo will determine the correct response.
        """
        exit = False
        miroResponse = ""
        for word in response:
            word = word.strip().upper()
            if word in self.hotKeys:
                print("determineResponse hot word: %s" % word)
                print("valid response")
                exit = True
                if word == "YES" or word == "YEAH":
                    print("you said yes")
                    miroResponse = self.responses["YES"]
                elif word == "NO" or word == "NAH":
                    print("you said no")
                    miroResponse = self.responses["NO"]
                elif word == "SORRY":
                    print("you said sorry")
                    miroResponse = self.responses["SORRY"]
            else:
                miroResponse = "Sorry I didn't catch that. Can you please repeat it again"


        return miroResponse, exit

    def listenToResidentOffline(self):
        """
        PURPOSE: To convert speech into transcribed text. The transcribed text
        will be the residents speech. The purpose is to allow MiRo to be able to
        have some interaction with the residents
        """
        print("listening to resident ... ")

        miroResponse = ""
        exit = False
        #you might need to have a time out time when you port this into ros
        timeStart = time.time()

        while not exit:
            #listening for audio till all the exit conditions are met
            phrase = LiveSpeech()
            phraseIter = iter(phrase)

            if phraseIter:
                wordIter = str(next(phraseIter)).split(" ")
                print(wordIter)
                miroResponse, exit  = determineResponse(wordIter)

        return miroResponse

    def listeToResidentOnline(self):
        """
        PURPOSE: This will use google speech recognition API to listen to the
        resident. For the use of API, this sub-module will require an internet
        connection, and recognition speeds will be determined by internet speed.
        """

        miroResponse = ""
        exit = False

        #going to be using google API's to do speech recognition 
        r = sr.Recognizer()
        mic = sr.Microphone()

        text = ""
        while not exit:
            with mic as source:
                audio = r.listen(source)

                try:
                    text = r.recognize_google(audio)
                except sr.UnknownValueError as err:
                    print("couldn't hear what you were saying")

                text = text.split(" ")
                print(text)
                miroResponse, exit = determineResponse(text)

        return miroResponse

    def generateSpeech(self, inSpeech, inLan="en", accent="com.au", speakSlow=False):
        """
        ASSERTION: Will convert the inputted text into a voice
        """
        print("generating speech")
        tts = gTTS(text=inSpeech, lang=inLan, tld=accent, slow=speakSlow)
        return tts

    def speakComputer(self, inGttsObj, fileName="voice.mp3"):
        """
        PURPOSE: This will play MiRO's voice from the computer.
        """

        print("the computer is speaking...")
        if inGttsObj:
            print("creating a mp3 file ...")
            inGttsObj.save(fileName)

        ps.playsound(fileName)

    def speakMiRo(self, inGttsObj):
        """
        PURPOSE:
        """
        #you will want to first generate a speech format in the wav format, and 
        #then you will want to send that to MiRo to actually speak and say what 
        #he wants to say as well

    def saveMP3(inGttsObj, fileName):
        """
        PURPOSE: To save a text-to-speech object into an .mp3 format.
        """
        #forcing the file extension to be mp3 regardless of extension
        inGttsObj.save(fileName.split(".")[0] + ".mp3")

    def saveWAV(inGttsObj, fileName):
        """
        PURPOSE: To save a text-to-speech object into a .wave format.
        """
        #foricing the file extension to be a wav format regardless of extension
        inGttsObj.save(fileName.split(".")[0] + ".wav")

    def readFile(self, fileName):
        """
        PURPOSE:
        """
        fileContents = []
        with open(fileName, "r") as inStrm:
            fileContents = inStrm.readlines()
            fileContents = "".join(fileContents)
        return fileContents


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



