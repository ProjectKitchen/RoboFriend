import pygame

#init
print "initializing soundModule..."
pygame.init()
if pygame.mixer and not pygame.mixer.get_init():
    print ('Warning, no sound')
    pygame.mixer = None

# methods
def close():
    pygame.quit()

def stop():
    pygame.mixer.stop()

def playRandom():
    playsound("data/fabibox/;random")

def playMood():
    playsound("data/;mood")

# deprecated: use single methods like playRandom()
# this function is called by chooseAction if the robot has to speak/make a sound
def playsound(dataArray):
    stop()
    soundname = dataArray[0] #sound(name +) pfad!
    if len(dataArray) > 1: #random sounds oder Aehnliches
        dataArray = dataArray[1:]
        info = dataArray[0]
        print ("info = " + info)

        if len(dataArray) > 1: # sollten mal noch mehr parameter notwendig sein
            dataArray = dataArray[1:]
            return
        elif info == "random":
            print ("random sound")
            r_sounds = ["s1", "s2", "s3", "s4", "s5", "s6", "s7", "s8"]
            rsnr =random.randint(0,7)
            soundpath = soundname+r_sounds[rsnr]+".wav"
        elif info == "mood":
            print ("mood sound")
            mood_sounds = ["sad","happy"]
            if faceModule.isSad():
                soundpath = soundname+mood_sounds[0]+".wav"
            else:
                soundpath = soundname+mood_sounds[1]+".wav"
    else:
        print ("playing soundfile " + soundname)
        soundpath = soundname

    try:
        print ("playing sound "+soundpath)
        selectedSound = pygame.mixer.Sound(soundpath)
    except:
        print ("could not open "+soundpath)
    else:
        selectedSound.play()