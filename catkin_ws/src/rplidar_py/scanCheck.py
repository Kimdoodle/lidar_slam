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

    def update(self, newEndIndex:int, scan) -> bool:
        self.endIndex = newEndIndex
        inter = scan.interInfo[newEndIndex]
        dist = scan.distInfo[newEndIndex]
        
        # 조건
        if (abs(inter) <= 2*self.interMean) & (abs(dist) <= 5*self.distMean):
            self.interSum += inter
            self.interMean = self.interSum / (newEndIndex - self.startIndex + 1)
            self.distSum += dist
            self.distMean = self.distSum / (newEndIndex - self.startIndex + 1)
            return True
        else:
            # totalDistMean = (totalDistMean * totalcount + self.distMean) / (totalcount + 1)
            # totalInterMean = (totalInterMean * totalcount + self.interMean) / (totalcount + 1)
            # totalcount += 1
            return False
        
    # 단일 선 여부 검사
    def isolation(self, scan) -> bool:
        if (self.interSum in scan.interOutlier) | (self.distSum in scan.distOutlier):
            return False
        else:
            return True