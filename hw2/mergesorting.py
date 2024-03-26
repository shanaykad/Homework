# To test the code just run it
import random

list1 = []
list2 = []

for i in range(50):
    list1.append(random.randint(0,1000))
    list2.append(random.randint(0,1000))

print(list1)
print(list2)

sortedList = sorted(list1+list2)
print(sortedList)