
class transfomToHours():
    def __init__(self):
        self.hours = 0
        self.minutes = 0
        self.seconds = 0

    def calculateHours(self,time):
        resto = time%3600
        minutos = int(resto/60)
        horas = int(time/3600)
        segundos = resto%60
        return self.fillZeros(horas),self.fillZeros(minutos),self.fillZeros(segundos)

    def fillZeros(self, value):
        if value <10:
            return str(value).zfill(2)
        else:
            return str(value)
def main():
    horas= transfomToHours()
    hora, minutos, segundos= horas.calculateHours(350)
    print(hora,minutos,segundos)

if __name__ == "__main__":
    main()
