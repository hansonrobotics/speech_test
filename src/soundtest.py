import pyglet
import threading
import pydub

OFFSET = 0
WINDOW = 10

sound = pyglet.media.load("../res/gspeech.wav", streaming=False)
player = sound.play()

segment = pydub.AudioSegment.from_wav("../res/gspeech.wav")

timer = None

pyglet.clock._get_sleep_time = pyglet.clock.get_sleep_time
pyglet.clock.get_sleep_time = lambda sleep_idle: pyglet.clock._get_sleep_time(False)

def startpyglet():
  threading.Timer(0.0, pyglet.app.run).start()
  hit()

def hit():
  curtime = int(player.time*1000) + OFFSET
  print segment[curtime:curtime+WINDOW].rms

  global timer
  timer = threading.Timer(0.1, hit)
  timer.start()

def endpyglet():
  pyglet.app.exit()
  timer.cancel()

