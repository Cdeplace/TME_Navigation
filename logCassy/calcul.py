import matplotlib.pyplot as plt
import numpy as np

names = ["log1.txt", "log2.txt", "log3.txt", "log4.txt", "log5.txt"]

valcum = []

for name in names :
    file = open(name, "r")

    val = []
    str = "go"

    while str != "" :
        str = file.readline()
        if str != "" :
            val.append(float(str))

    x = range(len(val))
    plt.plot(x, val, label = name)

    if valcum == [] :
        valcum = val
    else:
        valcum = np.add(valcum, val)


quart1Deb = np.percentile(valcum[0:10], 25)
medDeb = np.percentile(valcum[0:10], 50)
quart3Deb = np.percentile(valcum[0:10], 75)
quart1Fin = np.percentile(valcum[30:40], 25)
medFin = np.percentile(valcum[30:40], 50)
quart3Fin = np.percentile(valcum[30:40], 75)

print(quart1Deb)
print(medDeb)
print(quart3Deb)
print(quart1Fin)
print(medFin)
print(quart3Fin)

plt.title("temps pris pour atteindre la récompense à chaque itération en Qlearning")
plt.xlabel("itération")
plt.ylabel("temps (s)")
plt.legend()
plt.savefig("performance en temps du Qlearning")
plt.show()