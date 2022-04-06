import mpmath
import requests
from math import sin, cos, sqrt, asin, fabs, dist
import time
import numpy as np
from PIL import Image


def CalDis2(x, y, z, l1, l2, the1, the2, the3, bPrint):
    l2 = 0.055
    l3 = 0.065

    afterA = -1 * (l1 * cos(the2) + l2 * cos(the3)) + l3
    afterX = afterA * cos(the1)
    afterY = afterA * sin(the1)
    afterZ = l1 * sin(the2) + l2 * sin(the3)
    nowdis = dist([x, y, z], [afterX, afterY, afterZ])
    if bPrint == True:
        print("\nTarget:", x, y, z)
        print("After:", afterX, afterY, afterZ)
        print("Angle:", mpmath.degrees(the1), mpmath.degrees(the2), mpmath.degrees(the3))
        print("dis:", nowdis)
    return nowdis


# x > 0, z >0,y正负都可
# x、y、z单位为cm
# x、y、z绝对值为(0,15]
def MoveHand(x, y, z, bOpenMouse, bSafeUpMouseToSky, bSafeUpMouseHighForPreWrite):
    x /= 100.0
    y /= 100.0
    z /= 100.0

    l1 = 0.055
    l2 = 0.11
    a = sqrt(pow(x, 2) + pow(y, 2))
    the1 = asin(y / a)

    MaxThe2 = 180
    # MaxThe3=90

    GAP1 = 1.0

    v1 = 0
    bestV1 = -1.0
    bestV2 = -1.0
    MinDis = 1
    # print(int(MaxThe2 / GAP1))
    for i in range(0, int(MaxThe2 / GAP1)):

        v2Min = v1 - 45
        v2Max = v1 + 135
        v2 = v2Min
        for k in range(0, int((v2Max - v2Min) / GAP1)):

            v1Rad = mpmath.radians(v1)
            v2Rad = mpmath.radians(v2)

            nowdis = CalDis2(x, y, z, l1, l2, the1, v1Rad, v2Rad, False)
            # print(v1,v2,nowdis)
            if (nowdis < MinDis):
                MinDis = nowdis
                bestV1 = v1
                bestV2 = v2
                # print("better", bestV1, bestV2)
                # nowdis = CalDis(x, y, z, l1, l2, the1, v1Rad, v2Rad, True)

            v2 += (GAP1)
        v1 += (GAP1)

    print(mpmath.degrees(the1), bestV1, bestV2, MinDis)

    servoRot1 = mpmath.degrees(the1)
    servoRot1 = 90 + servoRot1

    servoRot2 = bestV1
    servoRot3 = 45 + bestV2 - bestV1

    servoRot4 = 90 + 180 - bestV2

    if (bSafeUpMouseToSky):
        servoRot2 = 60
        servoRot4 = 0
    if (bSafeUpMouseHighForPreWrite):
        servoRot2= bestV1
        servoRot4 = 0


    servoRot6 = 160
    if bOpenMouse:
        servoRot6 = 30
    print(servoRot1, servoRot2, servoRot3, servoRot4)
    url = "http://192.168.0.104/hand?content={\"ver\":1,\"cmd\":\"set_servo_angle\",\"angles\":[{\"idx\":1,\"angle\":" + str(
        servoRot1)
    url = url + "},{\"idx\":2,\"angle\":" + str(servoRot2)
    url += "},{\"idx\":3,\"angle\":" + str(servoRot3)
    url += "},{\"idx\":4,\"angle\":" + str(servoRot4)
    url += "},{\"idx\":5,\"angle\":90},{\"idx\":6,\"angle\":" + str(servoRot6)
    url += "}]}"

    payload = {}
    headers = {}

    response = requests.request("GET", url, headers=headers, data=payload)

    # print(response.text)


timeToWaitHandMove = 0.1

def IsPixBlank(pix):
    if pix[0] == 0:
        return True
    else:
        return False


#bitFlagOnlyStartPoint = 1
#bitFlagOpenMouse = 2
#bitFlagSafeUpMouseToSky = 4
MaxLengthX = 15.0
MaxLengthY = 30.0


def IsValidIdxForMat(mat,i,k):
    return i >= 0 and i < len(mat) and k >= 0 and k < len(mat[0])
def dfs(mat, i, k, points, vis):
    dx = [0, 0, 1, -1,  1,1,-1,-1]
    dy = [1, -1, 0, 0,  1,-1,1,-1]

    bEverStrech = False
    if (IsPixBlank(mat[i][k]) == False and vis[i][k] == 0):
        points.append((i, k))
        vis[i][k] = 1
        bEverStrech = True
        for x in range(0, 8):
            subi = i + dx[x]
            subk = k + dy[x]
            if (IsValidIdxForMat(mat,subi,subk)):
                bSubAns=dfs(mat, subi, subk, points, vis)
                if (bSubAns):
                    break
    return bEverStrech

def InterpArr(points, point2Idx):
    InterpolatedPoints = []
    InterpolatedPoint2Idx = []
    if (len(points) > 0):
        InterpolatedPoints.append(points[0])
        InterpolatedPoint2Idx.append(point2Idx[0])
    for idx1 in range(0, len(points) - 1):
        nowp = points[idx1]
        nxtp = points[idx1 + 1]

        # cm
        distOfPs = dist(nowp, nxtp)
        pointMaxGap = 0.05

        def MyInterp(iniX, targetX, Fen):
            return iniX + (targetX - iniX) * Fen

        if (distOfPs > pointMaxGap):
            duanCnt = int(distOfPs / pointMaxGap)
            for idx2 in range(0, duanCnt - 1):
                interpPoint = [0, 0, 0]
                interpPoint[0] = MyInterp(nowp[0], nxtp[0], (idx2 + 1.0) / duanCnt)
                interpPoint[1] = MyInterp(nowp[1], nxtp[1], (idx2 + 1.0) / duanCnt)
                interpPoint[2] = MyInterp(nowp[2], nxtp[2], (idx2 + 1.0) / duanCnt)
                InterpolatedPoints.append(interpPoint)
                InterpolatedPoint2Idx.append(point2Idx[idx1])
        InterpolatedPoints.append(nxtp)
        InterpolatedPoint2Idx.append(point2Idx[idx1 + 1])
    return InterpolatedPoints, InterpolatedPoint2Idx


def f9():
    image = Image.open('E:\\project\\新建文件夹\\Sprite-0007.png')
    mat = np.array(image)
    # print(mat)
    vis = np.zeros((len(mat), len(mat[0])), dtype=int)

    MoveHand(0.1, 0.1, 12, False, False,False)

    drawedPic = np.zeros((len(mat), len(mat[0])), dtype=int)
    for k in range(0, len(mat[0])):
        for i in range(0, len(mat)):
            pix = mat[i][k]
            if (IsPixBlank(pix) == False and vis[i][k] == 0):
                points = []
                dfs(mat, i, k, points, vis)
                print(points)

                pointPos = []
                point2Idx = []
                for p in points:
                    coordPoint = (
                        MaxLengthX * (1.0 - p[0] / (len(mat) - 1.0)),
                        MaxLengthY * (1.0 / 2 - (p[1] / (len(mat[0]) - 1))),
                        2.5)
                    pointPos.append(coordPoint)
                    point2Idx.append(p)
                pointPos, point2Idx = InterpArr(pointPos, point2Idx)


                for idx1 in range(0, len(pointPos)):
                    p = pointPos[idx1]
                    print("now point:", p)

                    drawedPic[point2Idx[idx1][0]][point2Idx[idx1][1]] = 1
                    print(drawedPic)
                    bOpenMouse = False

                    idxIInPic = point2Idx[idx1][0]
                    idxKInPic = point2Idx[idx1][1]
                    #if ((mat[idxIInPic][idxKInPic][2] & bitFlagOpenMouse) > 0):
                    #    bOpenMouse = True

                    bSafeUpMouseToSky = False
                    #if ((mat[idxIInPic][idxKInPic][2] & bitFlagSafeUpMouseToSky) > 0):
                    #    bSafeUpMouseToSky = True

                    if (idx1 == 0):
                        MoveHand(p[0], p[1], p[2], bOpenMouse, False,True)
                        time.sleep(0.5)
                    MoveHand(p[0], p[1], p[2], bOpenMouse, bSafeUpMouseToSky,False)
                    #time.sleep(timeToWaitHandMove)
                MoveHand(pointPos[len(pointPos)-1][0],pointPos[len(pointPos)-1][1],pointPos[len(pointPos)-1][2],False,False,True)

if __name__ == '__main__':
    f9()