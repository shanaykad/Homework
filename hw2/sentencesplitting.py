# To test the code, run it and enter your string as instructed
inputString = input('Enter your string:\n')

sentence = ' '
sentenceNum = 1

print(inputString)

for i in range(len(inputString)):
    sentence = sentence + inputString[i]
    if (inputString[i] == '.' or inputString[i] == '!' or inputString[i] == '?'):
        print(sentenceNum, sentence)
        sentence = ''
        sentenceNum = sentenceNum + 1