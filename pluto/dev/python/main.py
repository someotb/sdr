import numpy as np
from auto_offset import gardner
from matplotlib import pyplot as plt

print("""Choose 1 - Tx
Choose 2 - Rx""")

answer = int(input())

if answer == 2:
    nx = np.fromfile(
        "/home/someotb/Code/sdr/pluto/dev/pcm/rx.pcm",
        dtype=np.int16,
    )
else:
    nx = np.fromfile(
        "/home/someotb/Code/sdr/pluto/dev/pcm/tx.pcm",
        dtype=np.int16,
    )

rx = np.fromfile("/home/someotb/Code/sdr/pluto/dev/pcm/rx.pcm", dtype=np.int16)
tx = np.fromfile("/home/someotb/Code/sdr/pluto/dev/pcm/tx.pcm", dtype=np.int16)

samples = []

for x in range(0, len(nx), 2):
    samples.append((nx[x] + 1j * nx[x + 1]))

plt.figure()
plt.title("Samples before Gardner")
plt.plot(np.real(samples), label="REAL")
plt.plot(np.imag(samples), label="IMAG")

plt.figure()
plt.title("Samples before gardner")
plt.scatter(np.real(samples), np.imag(samples))
plt.axhline()
plt.axvline()
plt.grid()

impulse = np.ones(10)

samples_nx = nx[0::2] + 1j * nx[1::2]
list_offset = []
SPS = 10

samples_mf = np.convolve(samples_nx, np.ones(SPS))
signal = samples_mf
signal, offset_list = gardner(samples_mf, SPS)

plt.figure()
plt.title("Samples after Gardner")
plt.plot(np.real(signal), label="REAL")
plt.plot(np.imag(signal), label="IMAG")
plt.legend()
plt.grid()

plt.figure()
plt.title("Samples after Gardner")
plt.scatter(np.real(signal), np.imag(signal))
plt.axhline()
plt.axvline()
plt.grid()
plt.show()
