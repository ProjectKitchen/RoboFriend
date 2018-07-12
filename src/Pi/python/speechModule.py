import pyttsx

# globals
# --- none ---

def speak(text):
    global speechEngine

    if not speechEngine.isBusy():
        print "speaking: " + text
        speechEngine.say(text)
        speechEngine.runAndWait()

#init
print "initializing speechModule..."
speechEngine = pyttsx.init()
speechEngine.setProperty('rate', 150) #150 words per minute
speak('i am robofriend')