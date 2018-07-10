import pyttsx

#init
print "initializing speechModule..."
speechEngine = pyttsx.init()
speechEngine.setProperty('rate', 150) #150 words per minute
speechEngine.say('i am robofriend')
speechEngine.runAndWait()

# globals

def speak(text):
    global speechEngine
    speechEngine.say(text)
    speechEngine.runAndWait()