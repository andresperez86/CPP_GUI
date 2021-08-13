import utm

class calcularCoordsFromCenter():
    def __init__(self, center = (10.4471,-75.3912), lenghtSide = 175):
        self.center = center
        self.side = float(lenghtSide)
        self.coordGPSTwo = ()
        self.coordGPSOne = ()
        self.coordGPSThree = ()
        self.coordGPSFour = ()

    def calculate(self):
        coordUTM = utm.from_latlon(self.center[0], self.center[1])
        UTMZone = coordUTM[2:]
        # print(coordUTM[0], coordUTM[1])
        coordOne = (coordUTM[0]-self.side, coordUTM[1]+self.side)
        coordTwo = (coordUTM[0]+self.side, coordUTM[1]+self.side)
        coordThree = (coordUTM[0]+self.side, coordUTM[1]-self.side)
        CoordFour = (coordUTM[0]-self.side, coordUTM[1]-self.side)
        # coordFour = (coordUTM[0]-self.side, coordUTM[1]-self.side)
        self.coordGPSOne = self.UTM2GPS(coordOne,UTMZone)
        self.coordGPSTwo = self.UTM2GPS(coordTwo,UTMZone)
        self.coordGPSThree = self.UTM2GPS(coordThree,UTMZone)
        self.coordGPSFour = self.UTM2GPS(CoordFour,UTMZone)
        print(self.coordGPSOne[0], self.coordGPSOne[1])
        print(self.coordGPSTwo[0], self.coordGPSTwo[1])
        print(self.coordGPSThree[0], self.coordGPSThree[1])
        print(self.coordGPSFour[0], self.coordGPSFour[1])

    def UTM2GPS(self, cord, zone):
        return utm.to_latlon(*cord, *zone)


def main():
    calculos = calcularCoordsFromCenter()
    calculos.calculate()

if __name__ == "__main__":
    main()
        
