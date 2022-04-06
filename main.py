# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.

import requests
import time
import _thread
import random

urlBase = "http://192.168.0.101/"
def GetUrlForStepper(spd,steps):
    url = urlBase + "ar?content={\"ver\":1,\"cmd\":\"stepper_run\",\"speed\":\"" + str(spd) + "\",\"steps\":\"" +str(steps) + "\"}"
    return url

payload={}
headers = {}

def GetUrlForManRot(s):
    url = urlBase + "ar?content={\"ver\": 1,\"cmd\": \"" + s + "\"}"
    return url
def manRot():
    while True:
        try:
            i=random.randint(1,3)
            url=""
            if (i==1):
                url=GetUrlForManRot("servo_rot_1")
            elif (i==2):
                url=GetUrlForManRot("servo_rot_2")
            else:
                url=GetUrlForManRot("servo_rot_middle")

            response = requests.request("GET", url, headers=headers, data=payload)

            timeToWait=random.uniform(1,3)
            time.sleep(timeToWait)
        except:
            pass

timeToRun = 8
stepsToRun = 1024
# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    _thread.start_new_thread(manRot,())
    while True:
        try:
            response = requests.request("GET", GetUrlForStepper(200, stepsToRun), headers=headers, data=payload)
            time.sleep(timeToRun)
            response = requests.request("GET", GetUrlForStepper(200,-stepsToRun), headers=headers, data=payload)
            time.sleep(timeToRun)
        except:
            pass
