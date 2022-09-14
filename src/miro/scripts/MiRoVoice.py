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
import wave, struct


NODE_NAME = "MiROVoice"

MIRO_INTRO ="Hello, I am MiRo. I have seen that you have fallen over. Are you Okay?  Tap me on the head if you're okay otherwise, I can go and get help. If you prefer you can respond with yes or no, or sorry to hear the instructions again."

# amount to keep the buffer stuffed - larger numbers mean
# less prone to dropout, but higher latency when we stop
# streaming. with a read-out rate of 8k, 4000 samples will
# buffer for half of a second, for instance.
BUFFER_STUFF_SAMPLES = 4000

# messages larger than this will be dropped by the receiver,
# however, so - whilst we can stuff the buffer more than this -
# we can only send this many samples in any single message.
BUFFER_MARGIN = 1000
BUFFER_MAX = BUFFER_STUFF_SAMPLES + BUFFER_MARGIN
BUFFER_MIN = BUFFER_STUFF_SAMPLES - BUFFER_MARGIN

MAX_STREAM_MSG_SIZE = (4096 - 48)
RECORD_TIME = 2
MIC_SAMPLE_RATE = 20000
SAMPLE_COUNT = RECORD_TIME * MIC_SAMPLE_RATE





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
        self.__micBuff = np.zeros((0,4), 'uint16')
        self.__outBuff = None
        self.__bufferStuff = 0
        
        # get robot name
        topicBaseName = "/" + os.getenv("MIRO_ROBOT_NAME")

        #SUBSCRIBERS
        rospy.Subscriber("resident/warningSignal/",Bool,self.callBackWarningSignal)

        topic = topicBaseName + "/sensors/mics"
        rospy.Subscriber(topic, Int16MultiArray,
                self.callBackMics, queue_size=5, tcp_nodelay=True)


        #PUBLISHERS
        topic = topicBaseName + "/control/stream"
        self.pubStream = rospy.Publisher(topic, Int16MultiArray, queue_size=0)

        topic = topicBaseName + "/platform/log"
        self.subLog = rospy.Subscriber(topic, String, self.callback_log, queue_size=5, tcp_nodelay=True)

        topic = topicBaseName + "/sensors/stream"
        self.subStream = rospy.Subscriber(topic, UInt16MultiArray,
                self.callback_stream, queue_size=1, tcp_nodelay=True)

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
        self.__bufferSpace = 0
        self.__bufferTotal = 0



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
                if self.__bufferTotal > 0:
                    # compute amount to send
                    buffer_rem = self.__bufferTotal - self.__bufferSpace
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


    def saveAudioMics(self):
        """
        PURPOSE:
        """

        while not rospy.core.is_shutdown():

            #if the recording has finished
            if not self.__outBuff is None:
                break

            #re-fresh rate of 50Hz
            time.sleep(0.02)


        #recording sound from mics to file 
        outputFileName = '/tmp/miroResidentAudio.wav'
        file = wave.open(outputFileName, "wb")
        file.setsampwidth(2)
        file.setframerate(MIC_SAMPLE_RATE)


        #writting the recorded data to file for access later on
        print("Writing left and right ears to file")
        file.setnchannels(2)
        x = np.reshape(self.__outBuff[:, [0,1]], (-1))

        for s in x:
            file.writeframes(struct.pack('<h',s))

        #closing the file
        file.close()
        print("audio has being saved")



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

    def listenToResidentOnline(self):
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
        self.__bufferSpace = msg.data[0]
        self.__bufferTotal = msg.data[1]
        self.__bufferStuff = self.__bufferTotal - self.__bufferSpace

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

    def callBackMics(self, msg):
        """
        PURPOSE:
        """

        #if they mics are currently recording
        if not self.__micBuff is None:

            #appending the mic data to store
            data = np.array(msg.data, 'int16')
            oneAxis = np.reshape(data, (-1,500))
            self.__micBuff = np.concatenate((self.__micBuff, oneAxis.T))


            #reporting the progress of recording
            sys.stdout.write(".")
            sys.stdout.flush()

            #TODO: come back and change this to a true and false statement
            #finished recording

            if self.__micBuff.shape[0] >= SAMPLE_COUNT:
                #ending the recording 
                self.__outBuff = self.__micBuff
                self.__micBuff = None
                print("OK!")

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
    #determining if MiRo is being ran offline or online
    """
    mode = None
    try:
        mode = sys.argv[1]
        print(mode)
    except IndexError as err:
        print("no argument, running default ...")

    soundInterface = Streamer()
    soundInterface.playSound()
    """

    print("hello motherfucking world")
    soundInterface = Streamer()
    soundInterface.saveAudioMics()

    #we will actually need to let the sound buffer actually fill up with
    #some bytes



    """
    while not rospy.core.is_shutdown():

        #TODO: you will need to come back and figure out what this going to do
        #downsample for playback 

        outBuff = np.zeros((int(SAMPLE_COUNT / 2.5), 0))
        for c in range(4):
            i = np.arrange(0, SAMPLE_COUNT, 2.5)
            j = np.arrange(0, SAMPLE_COUNT)
            x = np.interp(i,j, self.__outBuff[:, c])
            outBuff = np.concatenate((outbuff, x[:, np.newaxis]), axis=1)


        #channel names 
        chan= = ["LEFT", "RIGHT", "CENTRE", "TAIL"]

        #looping
        while not rospy.core.is_shutdown():

            #checking the stuff output buffer

    """
