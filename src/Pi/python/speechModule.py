import pyttsx

# globals
# --- none ---

def speak(text):
    global speechEngine
    print "speaking: " + text
    speechEngine = pyttsx.init()
    speechEngine.setProperty('rate', 150) #150 words per minute
    speechEngine.say(text)
    speechEngine.runAndWait()

#init
print "initializing speechModule..."
speak('i am robofriend')