import pyglet
import pydub
import threading
import os

class SoundFile():

  dub_offset = 240
  dub_window = 20
  callback_interval = 0.05

  def on_playmore(self, rms):
    raise NotImplementedError

  def hit(self):
    startpos = int(self.gletplayer.time*1000) + self.dub_offset - self.dub_window / 2
    endpos = startpos + self.dub_window / 2
    rms = self.dubsegment[startpos:endpos].rms
    self.on_playmore(rms)

    self.timer = threading.Timer(self.callback_interval, self.hit)
    self.timer.start()

  def play(self):
    self.gletplayer.play()
    self.hit()

  def stop(self):
    self.gletplayer.pause()
    self.gletplayer.seek(0)
    self.timer.cancel()
    self.on_playmore(0)

  def __init__(self, filename):
    dirname, _ = os.path.split(os.path.abspath(__file__))
    filepath = os.path.join(dirname, "..", "res", filename)
    
    gletsource = pyglet.media.load(filepath, streaming=False)
    self.gletplayer = pyglet.media.Player()
    self.gletplayer.queue(gletsource)
    self.gletplayer.set_handler("on_eos", self.stop)

    self.dubsegment = pydub.AudioSegment.from_wav(filepath)