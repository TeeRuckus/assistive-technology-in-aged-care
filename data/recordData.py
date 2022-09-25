#TODO: you will need to have a header here explaining what is going on in this file
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

#making the plot display look a little bit prettier
plt.style.use("ggplot")
x_vals = []
y_vals = []

#the 
headers = ["", ""]


def animatePID(i):
    """
    IMPORT:
    EXPORT:

    PURPOSE:
    """

    try:
        #reading in where the csv is storing the data
        data = pd.read_csv("sonar_pid_control/data.csv")
    except FileNotFoundError as err:
        print("Need to create the file and data")

    x = data["Time (secs)"]
    y = data["Control variable"]


    #clearing the plot so we can have smoother data
    plt.cla()

    plt.plot(x, y)
    #plt.tight_layout()

    plt.title("Control variable against time")
    plt.xlabel("Time (seconds)")
    plt.ylabel("PID control variable")
    plt.xticks(rotation=45, ha="right")


#updating a tenth of a second
def animateSonar(i):
    """
    IMPORT:
    EXPORT:

    PURPOSE:
    """

    dataRaw = pd.read_csv("sonar_filtering/Raw Data.csv")
    xRaw = dataRaw["Time (secs)"]
    yRaw = dataRaw["Sonar Distance (m)"]

    #limiting data to the last 50 points
    #xRaw = xRaw[-150:]
    #yRaw = yRaw[-150:]


    #I want to display last 50 data point

    #clearing the plot so we can have smoother data
    plt.cla()

    #put other sensor data you want to plot here
    plt.plot(xRaw, yRaw, label="Raw data")

    plt.title("Sonar Distance Over Time")
    plt.xlabel("Time (seconds)")
    plt.ylabel("Distance (m)")
    plt.legend(loc="upper left")
    plt.xticks(rotation=45, ha="right")


#uncomment for animating PID controller
#ani = FuncAnimation(plt.gcf(),
        #animatePID,
        #interval=50)

ani = FuncAnimation(plt.gcf(),
        animateSonar,
        interval=50)
plt.show()
