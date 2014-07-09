#!/usr/bin/env python
import wave
import pyaudio
import sys
import pylab
import numpy

def recordsound():
    chunk = 1024
    FORMAT = pyaudio.paInt16
    CHANNELS = 1
    RATE = 44100
    RECORD_SECONDS = 2
    WAVE_OUTPUT_FILENAME = "output.wav"
    
    p = pyaudio.PyAudio()
    
    stream = p.open(format = FORMAT,
                    channels = CHANNELS,
                    rate = RATE,
                    input = True,
                    frames_per_buffer = chunk)
    
    print "* recording"
    all = []
    for i in range(0, RATE / chunk * RECORD_SECONDS):
        data = stream.read(chunk)
        all.append(data)
    print "* done recording"
    
    stream.close()
    p.terminate()

    # write data to WAVE file
    data = ''.join(all)
    result = numpy.frombuffer(data,dtype="int16") / float(2**15)

    pylab.plot(result)
    pylab.ylim([-1,1])
    pylab.show()

def cepstrum():
    # read wave
    wf = wave.open("/Users/ssb/workspace/data/sound/voice/shobon.wav","r")
    data = wf.readframes(wf.getnframes())
    wavplot = numpy.frombuffer(data, dtype="int16") / float(2**15)
    # hamming window for non-continous ends
    fs = wf.getframerate() # sampling frequnecy
    N = 512 # number of samples (frequency resolution) # 2048
    hamming = numpy.hamming(N)
    wav = wavplot[0:N]
    windowed_wav = hamming*wav
    # fourier transform for breaking down waves
    dft = numpy.fft.fft(wav) # discrete Fourier transform
    windowed_dft = numpy.fft.fft(windowed_wav)
    fscale = numpy.fft.fftfreq(N, d=1.0/fs)
    amp = [numpy.sqrt(c.real**2+c.imag**2) for c in dft] # amplitude
    windowed_amp = [numpy.sqrt(c.real**2+c.imag**2) for c in windowed_dft]
    pylab.subplot(411)
    pylab.plot(range(0, N), wav)
    pylab.axis([0, len(wavplot), -1.0, 1.0])
    pylab.subplot(412)
    pylab.plot(range(0, N), windowed_wav)
    pylab.axis([0, len(wavplot), -1.0, 1.0])
    pylab.show()



if __name__ == '__main__':
    cepstrum()
    print "main"
