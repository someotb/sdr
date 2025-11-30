import numpy as np
import matplotlib.pyplot as plt

data = np.fromfile("/home/someotb/Files/Code/sdr/pluto/dev/build/rx.pcm", dtype=np.int16)

samples = []
for x in range(0, len(data) - 1, 2):
    samples.append(data[x] + 1j * data[x+1])

real = np.real(samples)

sps = 10
segments = []
for i in range(0, len(real) - 2*sps, sps):
    segments.append(real[i:i + 2*sps])

plt.plot(np.array(segments).T)
plt.xlim(7.5, 12.5)
plt.grid(True)
plt.show()