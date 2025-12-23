import numpy as np
from auto_offset import gardner
from matplotlib import pyplot as plt

rx = np.fromfile(f"/home/plutoSDR/Документы/someotb/sdr/pluto/dev/build/rx.pcm", dtype=np.int16) # 2937360*2:2939644*2

samples = []

for x in range(0, len(rx), 2):
    samples.append((rx[x] + 1j * rx[x + 1]))

plt.figure()
plt.scatter(np.real(samples), np.imag(samples))
plt.title("Real and Imag parts before gardner")
plt.axhline()
plt.axvline()

impulse = np.ones(10)

samples_rx = (rx[0::2] + 1j * rx[1::2])
list_offset = []
SPS = 10

samples_mf = np.convolve(samples_rx, np.ones(SPS))
signal, offset_list = gardner(samples_mf, SPS)

plt.figure()
plt.title("Samples after Gardner")
plt.plot(samples_rx)
plt.legend()
plt.grid()
plt.show()
