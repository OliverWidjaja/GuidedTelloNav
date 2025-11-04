error = [i*0.1 for i in range(5,-5, -1)]
sVal = [0] * (len(error))

for i, _ in enumerate(error):
    """ When error is less, set value is less. When error is more, set value is more """
    sVal[i] = error[i] * -0.1

print(error, sVal)
