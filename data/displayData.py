import pandas as pd
import matplotlib.pyplot as plt
import os

plt.style.use("ggplot")

def getCsvFiles(dirName):
    files = os.listdir(dirName)
    print(files[1].split(".")[-1])
    #making sure to only include directories
    csvFIles = [x for x in files if x.split(".")[-1] == "csv"]
    print(csvFIles)

    return csvFIles


def displayPID():
    dirName = "sonar_pid_control/I_tuning/"
    files = getCsvFiles(dirName)

    x = []
    y = []
    legendList = []

    for ii in files:
        data = pd.read_csv(dirName + ii)
        x = data["Time (secs)"]
        y = data["Control variable"]
        #labels for the proportional values
        #plt.plot(x,y, label=ii.split(",")[0][4:])
        #labels for the derivative of PID controller
        #plt.plot(x,y, label="P=-3, " + ii.split(",")[2][:-4])
        #labels for all parameters
        plt.plot(x,y, label="P=-3, " + ii[4:-4])
    plt.legend(loc="upper right")
    plt.xlabel("Time (Secs)")
    plt.ylabel("PID Control Variable (%)")
    plt.title("PID Control Variable vs. Time")
    plt.savefig("sonar_pid_control/Integral Tuning ones firs five")
    plt.show()

def displaySonar():
    dirName = "sonar_filtering/Driving results/SMA/"
    files = getCsvFiles(dirName)

    x = []
    y = []
    legendList = []

    for ii in files:
        data = pd.read_csv(dirName + ii)
        x = data["Time (secs)"]
        y = data["Sonar Distance (m)"]
        plt.plot(x,y, label=ii[:-4])
    plt.legend(loc="upper left")
    plt.xlabel("Time (Secs)")
    plt.ylabel("Sonar Distance (m)")
    plt.title("Simple Moving Average Tests")
    plt.savefig("sonar_filtering/Simple Moving Average Results")
    plt.show()

#TODO: change the files
if __name__ == "__main__":
    displaySonar()
