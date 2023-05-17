length = 30
true_length = 4
false_length = 6
counter = 3

buffer = [False for x in range(length)]

for i in range(0, length, true_length + false_length):
    for j in range(0, true_length):
        buffer[(i + j + counter) % length] = True
    for j in range(true_length, false_length):
        buffer[(i + j + counter) % length] = False

buffer = ''.join(["0" if x else "-" for x in buffer])
print(buffer)

buffer = [False for x in range(length)]

for i in range(0, length):
    a = (i - counter) % length
    t = a % (true_length + false_length)
    buffer[i] = t < true_length

buffer = ''.join(["0" if x else "-" for x in buffer])
print(buffer)
