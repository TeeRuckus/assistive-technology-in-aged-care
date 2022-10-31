# Assistive-technology-in-aged-care

## Maintainers
- **AUTHOR:** Tawana David Kwaramba
- **E-MAIL:** tawanakwaramba@gmail.com
- **LAST MODIFIED:** 31/10/22

## Curtin Custodian
- **NAME:** Professor Tele Tan
- **E-MAIL:** T.Tan@curtin.edu.au

## Purpose
This set of files is to implement a fall detection algorithm onto the MiRo platform. MiRo wil detect a fallen resident using the stereo cameras, and drive to the fallen resident using the differential drive. The differential drive is supported by a PID controller with the values of P=-2, D=-0.03, and I=0. Once MiRo has stopped in front of the resident, MiRo will verbally ask if the fallen resident is okay, and MiRo will listen for a resident from the resident. If the resident is okay, MiRo will switch off. Otherwise, MiRo will do an emergency dance to attract the attention of the carer's. While doing the dance MiRo will send an email to the aged-care facility. The video demonstration of this software is demonstrated here:https://youtu.be/RGSnkb3LFhY

## Key Features
- Fall detection using MediaPipe. Fall methods are based on bounding box
orientation, and or the distance of the nose from the floor. 
- Speech recognition of response given my residents
- Interfacing with the g-mail mailing platform 
- Differential drive control through thew use of a PID controller

## Requirements
- Python version 3.8.10
- MediaPipe 0.8.9.1
- OpenCV 4.6.0
- ROS Noetic 
- Linux Operating System

## Files in Project
- Fallen_analyser Package 
    - scripts
        - **Errors.py:** File holding all the error states MiRo can occur in the pose_fallen.py class 
        - **pose_fallen.py:**: Python script holding the fall detection algorithm, and all the necessary machine vision algorithms.
        - **pose_fallen_test.py:**: Testing code to test the pose_fallen.py file. This file wll obtain fall metrics such as true positives, false negatives, false positives, precision, and recall
        - **results:** This folder will contain all the results of the test conducted
        - **test_data:** This folder will contain all the test data which the algorithm has being tested over
- MiRo Package
    - scripts
        - **Errors.py:** File holding all the error states MiRo can occur in the MiRoKinematics.py and MiRoVoice.py classes
        - **MiRoKinematics.py**: File responsible for all the kinematics movements of MiRo. Operation of the differential drive system, the use of PID controller, the emergency help dance, physical interaction with resident, and the mailing of teh aged-care facility . 
        - **MiRoVoice.py**: File responsible for all the verbal interaction between the resident, and MiRo. This wll be speaking the set of instructions to the resident, and of MiRo listening to MiRo.
        - **test_signal.mp3**: Warning signal which wll be played when the resident has fallen.
