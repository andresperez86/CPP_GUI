import utm

class calcularCoordsFromPoint():
    def __init__(self, corner = (-22.118847771958745, -51.448133644956364), lenghtSide = 280):#-22.118847771958745, -51.448133644956364//(10.448436368184836, -75.39254156781966),300
        self.corner = corner
        self.side = float(lenghtSide)
        self.coordGPSTwo = ()
        self.coordGPSOne = ()
        self.coordGPSThree = ()
        self.coordGPSCenter = ()

    def calculate(self):
        coordUTM = utm.from_latlon(self.corner[0], self.corner[1])
        UTMZone = coordUTM[2:]
        # print(coordUTM[0], coordUTM[1])
        coordOne = (coordUTM[0]+self.side, coordUTM[1])
        coordTwo = (coordUTM[0]+self.side, coordUTM[1]-self.side)
        coordThree = (coordUTM[0], coordUTM[1]-self.side)
        center = (coordUTM[0]+float(self.side)/2, coordUTM[1]-float(self.side)/2)
        # coordFour = (coordUTM[0]-self.side, coordUTM[1]-self.side)
        self.coordGPSOne = self.UTM2GPS(coordOne,UTMZone)
        self.coordGPSTwo = self.UTM2GPS(coordTwo,UTMZone)
        self.coordGPSThree = self.UTM2GPS(coordThree,UTMZone)
        self.coordGPSCenter = self.UTM2GPS(center,UTMZone)
        print(self.coordGPSOne[0], self.coordGPSOne[1])
        print(self.coordGPSTwo[0], self.coordGPSTwo[1])
        print(self.coordGPSThree[0], self.coordGPSThree[1])
        print(self.coordGPSCenter[0], self.coordGPSCenter[1])

    def UTM2GPS(self, cord, zone):
        return utm.to_latlon(*cord, *zone)


def main():
    calculos = calcularCoordsFromPoint()
    calculos.calculate()

if __name__ == "__main__":
    main()
        
