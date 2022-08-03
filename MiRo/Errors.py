import sys

class Error(Exception):
    pass


class MiRoError(Error):
    def __init__(self, mssg):
        print(mssg)
        sys.exit()

