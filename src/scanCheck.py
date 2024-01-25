# scanData 선 생성 조건 클래스
from math import atan

from calculate import calculate_dist, calculate_move
from unit import Cord, Line

totalDistMean = 0
totalInterMean = 0
maxCrit = 100
totalcount = 0

# 데이터 비교 클래스
class DataCheck:
    def __init__(self, index, prev, curr):
        self.index = index
        self.prev = prev
        self.curr = curr
        self.avg = (prev+curr)/2
    def __le__(self, other):
        return self.avg <= other.avg
    def __ge__(self, other):
        return self.avg >= other.avg
    def __gt__(self, other):
        return self.avg > other.avg
    def __lt__(self, other):
        return self.avg < other.avg


# 점 -> 선 결정 클래스
class ScanCheckDotToLine:
    def __init__(self, sindex:int, scan):
        self.startIndex = sindex
        self.endIndex = -99

        self.interSum = scan.interInfo[sindex]
        self.interMean = scan.interInfo[sindex]
        self.distSum = scan.distInfo[sindex]
        self.distMean = scan.distInfo[sindex]

        self.crit = self.moveDirection(scan, sindex)

    def update(self, newEndIndex:int, scan) -> bool:
        self.endIndex = newEndIndex
        inter = scan.interInfo[newEndIndex]
        dist = scan.distInfo[newEndIndex]

        # 조건
        if(self.crit != self.moveDirection(scan, newEndIndex)):
            return False
        if (abs(inter) <= 2*self.interMean) & (abs(dist) <= 5*self.distMean):
            self.interSum += inter
            self.interMean = self.interSum / (newEndIndex - self.startIndex + 1)
            self.distSum += dist
            self.distMean = self.distSum / (newEndIndex - self.startIndex + 1)
            return True
        else:
            return False

    # x/y축 이동여부 검사
    def moveDirection(self, scan, index) -> str:
        # cordI = scan.cordInfo[self.startIndex]
        # cordII = scan.cordInfo[(self.startIndex+1)%len(scan.cordInfo)]

        # xDiff = abs(cordII.x - cordI.x)
        # yDiff = abs(cordII.y - cordI.y)
        # crit = 5
        # if (xDiff <= crit) & (yDiff <= crit):
        #     return 'X' if xDiff < yDiff else 'Y'
        # elif (xDiff <= crit):
        #     return 'X'
        # elif (yDiff <= crit):
        #     return 'Y'
        # else:
        #     return 'Z'
        func = None
        if index == 0:
            func = abs(scan.funcInfo[0])
        else:
            prevFunc = scan.funcInfo[index]
            curFunc = scan.funcInfo[(index+1)%len(scan.funcInfo)]
            func = abs(curFunc/prevFunc)

        return 'Y' if func >= 1 else 'X'


    # 단일 선 여부 검사
    def isolation(self, scan) -> bool:
        if (self.interSum in scan.interOutlier) | (self.distSum in scan.distOutlier):
            return False
        else:
            return True


# 선 이동 클래스
class MoveLine:
    def __init__(self):
        self.moveX = 0
        self.moveY = 0
        self.rotate = 0
        self.sum = [0, 0, 0]
        self.count = 0

    def update(self, old:Line, new:Line) -> Line:
        self.count += 1
        oldMid = old.calMid()
        newMid = new.calMid()
        oldFunc = old.calFunc()
        newFunc = new.calFunc()
        # 이동 계산
        movex = (oldMid[0] - newMid[0])
        movey = (oldMid[1] - newMid[1])
        rotate = atan((newFunc - oldFunc) / (1 + newFunc * oldFunc))
        # 합 계산
        self.sum[0] += movex
        self.sum[1] += movey
        self.sum[2] += rotate
        # 평균
        self.moveX = self.sum[0] / self.count
        self.moveY = self.sum[1] / self.count
        self.rotate = self.sum[2] / self.count

        # 두 직선을 병합
        start1 = (old.startX, old.startY)
        end1 = (old.endX, old.endY)
        self.e = 43
        start2 = calculate_move(new.startX, new.startY, self.e, movey, rotate)
        end2 = calculate_move(new.endX, new.endY, self.e, movey, rotate)
        # 거리 비교
        newStart = start1 if calculate_dist(start1, oldMid) > calculate_dist(start2, oldMid) else start2
        newEnd = end1 if calculate_dist(end1, oldMid) > calculate_dist(end2, oldMid) else end2


        return Line(Cord(newStart), Cord(newEnd), 0, 0)

    def justMove(self, new:Line) -> Line:
        newStart = calculate_move(new.startX, new.startY, self.e, self.moveY, 0)
        newEnd = calculate_move(new.endX, new.endY, self.e, self.moveY, 0)

        return Line(Cord(newStart), Cord(newEnd), 0 ,0)

