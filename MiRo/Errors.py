class Error(Exception): 
    pass


class MiRoError(Error):
    def __init(self, mssg):
        self.message = mssg

