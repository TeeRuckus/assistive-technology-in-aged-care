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
import playsound as ps


NODE_NAME = "MiROVoice"

#TODO: Just map this to use the file which has being already saved instead
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
MIRO_SPEECH_PATH = "/tmp/miroSpeech.wav"
MIRO_SPEECH_PATH = "/home/parallels/miroThesisFiles/introduction.wav"
MIRO_SPEECH_FILE = "miroSpeech.wav"






#TODO: you will need to make the naming conventions here all consistent with each other
#TODO: you will need to get rid of depreciated method from the program and file 
#TODO: you will need to have a setter in here which will set the mode of the robot. Whether to use the online API, or to just use the offline version

#going to hard code the path to the signals which MiRo will require
HELP_SIGNAL_FILE = "test_signal.mp3"
#the path of the file which you want to play
HELP_SIGNAL_PATH = "/home/parallels/miroThesisFiles/test_signal.mp3"

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
        #self.__playWarningSignal = False
        #TODO: come back and play with these variables if you get the correct signals
        self.__playWarningSignal = False
        self.__miroSpeak = False
        self.__giveIntro = False
        #TODO: I have removed this to allow t record when I want too
        #self.__micBuff = np.zeros((0,4), 'uint16')
        self.__micBuff = None
        self.__outBuff = None
        
        # get robot name
        topicBaseName = "/" + os.getenv("MIRO_ROBOT_NAME")

        #SUBSCRIBERS
        rospy.Subscriber("resident/warningSignal/",Bool,self.callBackWarningSignal)
        rospy.Subscriber("resident/miroSpeak/", Bool, self.callBackMiRoSpeak)
        rospy.Subscriber("resident/miroIntro/", Bool, self.callBackMiroIntro)
        topic = topicBaseName + "/platform/log"
        self.subLog = rospy.Subscriber(topic, String, self.callback_log, queue_size=5, tcp_nodelay=True)

        topic = topicBaseName + "/sensors/stream"
        self.subStream = rospy.Subscriber(topic, UInt16MultiArray,
                self.callback_stream, queue_size=1, tcp_nodelay=True)


        #PUBLISHERS
        topic = topicBaseName + "/control/stream"
        self.__pubStream = rospy.Publisher(topic, Int16MultiArray, queue_size=0)
        self.__pubStartTimer= rospy.Publisher("resident/startTimer/", Bool,
        queue_size=0)





        self.dataR = 0
        self.data = 0
        self.__bufferSpace = 0
        self.__bufferTotal = 0

    @property
    def playWarningSignal(self):
        """
        IMPORT: None
        EXPORT: Boolean

        ASSERTION: Returns the playWarningSignal class field
        """
        return self.__playWarningSignal

    @property
    def miroSpeak(self):
        """
        IMPORT: None
        EXPORT: Boolean

        ASSERTION: Returns the miroSpeak class field
        """

        return self.__miroSpeak

    @property
    def giveIntro(self):
        """
        IMPORT: None
        EXPORT: Boolean

        ASSERTION: Returns the giveIntro class field
        """
        return self.__giveIntro

    def recordSound(self):
        """
        IMPORT:
        EXPORT:

        PURPOSE:
        """
        topicBaseName = "/" + os.getenv("MIRO_ROBOT_NAME")
        #set up what you need to record sound
        self.__micBuff = np.zeros((0,4), 'uint16')
        topic = topicBaseName + "/sensors/mics"
        rospy.Subscriber(topic, Int16MultiArray,
                self.callBackMics, queue_size=5, tcp_nodelay=True)

    def clearRecording(self):
        """
        IMPORT:
        EXPORT:

        PURPOSE:
        """
        self.__micBuff = None
        self.__outBuff = None

    def startFallenTimer(self):
        """
        IMPORT:
        EXPORT:

        PURPOSE:
        """
        timer = Bool()
        timer.data = True
        self.__pubStartTimer.publish(timer)


    @playWarningSignal.setter
    def playWarningSignal(self, inSound):
        """
        ASSERTION:
        """
        self.__playWarningSignal = self.__validateSound(inSound)

    def decodeFile(self, inDecodeName, path):
        """
        PURPOSE:
        """
        fileName = "/tmp/" + inDecodeName  + ".decode"

        """
        #if the decode file is not found, create a new now
        if not os.path.isfile(fileName):
            cmd = "ffmpeg -y -i \"" + path + "\" -f s16le -acodec pcm_s16le -ar 8000 -ac 1 \"" + fileName + "\""

            os.system(cmd)
            if not os.path.isfile(fileName):
                rospy.loginfo("failed decode mp3")
                sys.exit(0)
        """

        #creating a decode file every time
        cmd = "ffmpeg -y -i \"" + path + "\" -f s16le -acodec pcm_s16le -ar 8000 -ac 1 \"" + fileName + "\""

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

    #TODO: I don't really want to touch this but, I am going to make the same 
    #function but it's going to take any wave format
    def playSound(self, inFile, path):
        """
        PURPOSE:
        """


        self.decodeFile(inFile, path)

        count = 0
        exit = False

        #used to stop the stream once the stream of music has finished
        finishedPlaying = False

        # safety dropout if receiver not present
        dropout_dataR = -1
        dropout_count = 3

        # loop
        while not rospy.core.is_shutdown() and not finishedPlaying:
            #we only want to play the warning signal once we have subscribed
            #from the signal
            #if self.__playWarningSignal:
            if not os.path.isfile(inFile):
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
                    self.__pubStream.publish(msg)
                    self.dataR += n_bytes
            # break
            if self.dataR >= len(self.data):
                exit = True

            # report once per second
            if count == 0:
                count = 10
                print ("streaming:", self.dataR, "/", len(self.data), "bytes")

                if self.dataR >= len(self.data):
                    finishedPlaying = True

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
        #TODO: You will need to make this a global constant and path
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


    def talkToMiRoOnline(self):
        """
        PURPOSE:
        """
        #TODO: remember to make this function pass something in
        voiceRecording = "/tmp/miroResidentAudio.wav"
        self.saveAudioMics()
        response = self.listenToResidentOnlineFile(voiceRecording)
        self.debug(response)
        tts = self.generateSpeech(response)

        #self.speakComputer(tts, "/tmp/miroResponse.wav")

        #TODO: you will need to remember to pass in a global constant to this variable
        self.speakMiRo(tts, "/tmp/miroResponse.wav")

        print("I have finished speaking now")

    def talkToMiRoOffline(self):
        """
        PURPOSE:
        """



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

    def listenToResidentOnlineFile(self, inFile):
        """
        PURPOSE:
        """

        miroResponse = ""
        r = sr.Recognizer()
        contents = sr.AudioFile(inFile)
        exit = False

        while not exit:
            with contents as source:
                audio = r.record(source)
                try:
                    text = r.recognize_google(audio)
                except sr.UnknownValueError as err:
                    print("couldn't hear what you were saying")
                    #get recording from file again
                    self.saveAudioMics()

                text = text.split(" ")
                print(text)
                miroResponse, exit = self.determineResponse(text)

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

    def speakMiRo(self, inGttsObj, savePath):
        """
        PURPOSE:
        """
        #you will want to first generate a speech format in the wav format, and 
        #then you will want to send that to MiRo to actually speak and say what 
        #he wants to say as well

        if inGttsObj:
            print("creating a wav file ... ")
            inGttsObj.save(savePath)
            print("SAVED %s " % savePath)

        #self.speakComputer(inGttsObj, savePath)
        self.playSound(savePath.split('/')[-1] ,savePath)

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
        IMPORT: None
        EXPORT: None

        PURPOSE: The call back to tell whether if MiRO should play a warning
        signal or not
        """
        self.__playWarningSignal = bool(data.data)

    def callBackMiRoSpeak(self, data):
        """
        IMPORT: None
        EXPORT: None

        PURPOSE: The call back to tell miro to speak and to say something
        """
        self.__miroSpeak = bool(data.data)

    def callBackMiroIntro(self, data):
        """
        IMPORT: None
        EXPORT: None

        PURPOSE: The call back to tell miro to introduce himself to the resident
        """
        self.__giveIntro = bool(data.data)

    def callBackMics(self, msg):
        """
        IMPORT: None
        EXPORT: None

        PURPOSE: The call back to tell whether if MiRo should listen, and 
        speak to the resident
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

    def debug(self, inTxt):
        """
        PURPOSE:
        """
        print("=" * 80)
        print(inTxt)
        print("=" * 80)

if __name__ == "__main__":
    #TODO: you will need to come back and implement this functionality later
    #determining if MiRo is being ran offline or online
    mode = None
    try:
        mode = sys.argv[1]
        print(mode)
    except IndexError as err:
        print("no argument, running default ...")


    soundInterface = Streamer()
    while not rospy.core.is_shutdown():
        if soundInterface.playWarningSignal:
            soundInterface.playSound(HELP_SIGNAL_FILE, HELP_SIGNAL_PATH)
        elif soundInterface.giveIntro:
            soundInterface.playSound(MIRO_SPEECH_FILE,MIRO_SPEECH_PATH)
        elif soundInterface.miroSpeak:
            #we want to short circuit operation, we never want both at the same time
            print("INSIDE")
            soundInterface.recordSound()
            soundInterface.talkToMiRoOnline()
            soundInterface.clearRecording()

        #50 hertz refresh rate
        time.sleep(0.02)

    #disconnecting from the robot when processing is done
    rospy.on_shutdown(miroRobot.cleanUp)
    RobotInterface.disconnect

