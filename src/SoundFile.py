import pyglet
import pydub
import threading
import os

class SoundFile():

  # Time offset in milliseconds used to synchronize jaw with the sound.
  # E.g. 240 means jaw will move 240ms ahead of the sound than default.
  dub_offset = 240

  # Number of miliseconds to calculate the root mean squared of the waveform from.
  dub_window = 20

  # on_playmore() will be called every this seconds, when the file is playing.
  callback_interval = 0.05

  is_playing = False

  def on_playmore(self, rms):
    raise NotImplementedError

  def hit(self):
    # Calculate the root mean squared of the currently playing waveform and
    # call back with it.
    startpos = int(self.gletplayer.time*1000) + self.dub_offset - self.dub_window / 2
    endpos = startpos + self.dub_window / 2
    rms = self.dubsegment[startpos:endpos].rms
    self.on_playmore(rms)

    # Keep calling this method until the timer is stopped.
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
    # Get full filepath (relative to the 'res' folder)
    dirname, _ = os.path.split(os.path.abspath(__file__))
    filepath = os.path.join(dirname, "..", "res", filename)
    
    # Load file to pyglet, for playing.
    gletsource = pyglet.media.load(filepath, streaming=False)
    self.gletplayer = pyglet.media.Player()
    self.gletplayer.queue(gletsource)
    self.gletplayer.set_handler("on_eos", self.stop)

    # Load file to pydub, for getting the current power (volume) in the sound file.
    self.dubsegment = pydub.AudioSegment.from_wav(filepath)