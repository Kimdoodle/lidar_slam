# scanData 선 생성 조건 클래스

totalDistMean = 0
totalInterMean = 0
maxCrit = 100
totalcount = 0

class ScanCheck():
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