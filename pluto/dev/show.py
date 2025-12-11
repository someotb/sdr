import matplotlib.pyplot as plt
import numpy as np

print("""Choose 0 - Tx
Choose 1 - Rx""")

answer = int(input())

if answer == 1:
    nx = np.fromfile(
        f"/home/plutoSDR/Документы/someotb/code/sdr/pluto/dev/build/rx.pcm",
        dtype=np.int16,
    )
else:
    nx = np.fromfile(
        f"/home/plutoSDR/Документы/someotb/code/sdr/pluto/dev/build/tx.pcm",
        dtype=np.int16,
    )

rx = np.fromfile(
    f"/home/plutoSDR/Документы/someotb/code/sdr/pluto/dev/build/rx.pcm", dtype=np.int16
)
tx = np.fromfile(
    f"/home/plutoSDR/Документы/someotb/code/sdr/pluto/dev/build/tx.pcm", dtype=np.int16
)
print(f"RX: {rx}\nTX: {tx}")

samples = []

for x in range(0, len(nx), 2):
    samples.append(nx[x] + 1j * nx[x + 1])

ampl = np.abs(samples)
phase = np.angle(samples)
time = np.arange(len(samples))

# plt.subplot(3,1,1)
# plt.plot(time, ampl)
# plt.grid(True)
# plt.tight_layout()
# plt.title("Amplitude")

# plt.subplot(3,1,2)
# plt.plot(time, phase)
# plt.grid(True)
# plt.tight_layout()
# plt.title("Phase")


# plt.subplot(3,1,3)
plt.plot(time, nx[0::2])
plt.plot(time, nx[1::2])
plt.grid(True)
plt.tight_layout()
plt.title("Real and Imag parts")
plt.show()
