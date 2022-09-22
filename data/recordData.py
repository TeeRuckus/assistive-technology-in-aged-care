#TODO: you will need to have a header here explaining what is going on in this file
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

#making the plot display look a little bit prettier
plt.style.use("fivethirtyeight")
x_vals = []
y_vals = []

#the 
headers = ["", ""]


def animate(i):
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

#creating a 
ani = FuncAnimation(plt.gcf(),
        animate,
        interval=50)
#plt.tight_layout()
plt.show()
