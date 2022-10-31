"""
AUTHOR: Tawana David Kwaramba
EMAIL: tawanakwaramba@gmail.com
LAST MODIFIED DATE: 31/10/22
PURPOSE: for all the erros possible in the kinemtic package
"""
import sys

class Error(Exception):
    pass


class MiRoError(Error):
    def __init__(self, mssg):
        print(mssg)
        sys.exit()

