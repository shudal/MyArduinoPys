import requests
import time

urlBase = "http://192.168.0.104/"

def GetUrlForMotorRun(s):
    url = urlBase + "ar?content={\"ver\": 1,\"cmd\": \"" + s + "\"}"
    return url

payload = {}
headers = {}
def MotorForward():
    urlForward=GetUrlForMotorRun("motor_run_reverse")
    response = requests.request("GET", urlForward, headers=headers, data=payload)
    print(response.text)
def MotorBack():
    urlBack=GetUrlForMotorRun("motor_run_default")
    response = requests.request("GET", urlBack, headers=headers, data=payload)
    print(response.text)
def MotorSilent():
    url=GetUrlForMotorRun("motor_run_silent")
    response = requests.request("GET", url, headers=headers, data=payload)
    print(response.text)
def stepperForward():
    url=urlBase + "ar?content={\"ver\":1,\"cmd\":\"stepper_run\",\"speed\":\"200\",\"steps\":\"1024\"}"
    response = requests.request("GET", url, headers=headers, data=payload)
    print(response.text)

timeToMoveMotor=6.5
timeToMoveMotorTmpOutIn=0.5

if __name__ == '__main__':
    while True:
        try:

            MotorBack()
            time.sleep(timeToMoveMotor - timeToMoveMotorTmpOutIn)
            MotorForward()
            time.sleep(timeToMoveMotor)

            MotorBack()
            time.sleep(timeToMoveMotorTmpOutIn)

            MotorSilent()
            time.sleep(1)
            stepperForward()
            time.sleep(timeToMoveMotor * 2)
        except Exception as e:
            print(e)
