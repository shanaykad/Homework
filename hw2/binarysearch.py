# To test the code run it and input your number as instructed
import random
import math

list = []

for i in range(101):
    list.append(random.randint(0,1000))
sortedList = sorted(list)
binarySearch = sortedList

x = -1
while (x<0 or x>1000):
    x = input('Input a number between 0 and 1000\n')
    x = int(x)

print(list)
print(sortedList)
print(x)
    
done = 0
while(done != 1):
    halfLength = math.floor(len(binarySearch)/2)
    if (binarySearch[halfLength] == x):
        print('TRUE')
        done = 1
    elif (halfLength == 0):
        print('FALSE')
        done = 1
    elif (binarySearch[halfLength] < x):
        binarySearch = binarySearch[halfLength:]
    elif (binarySearch[halfLength] > x):
        binarySearch = binarySearch[:halfLength]