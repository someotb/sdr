import matplotlib.pyplot as plt
import numpy as np

rx = np.fromfile(f"/home/someotb/Files/Code/sdr/pluto/dev/build/rx.pcm", dtype=np.int16)
tx = np.fromfile(f"/home/someotb/Files/Code/sdr/pluto/dev/build/tx.pcm", dtype=np.int16)

rx_samples = []
tx_samples = []

for x in range(0, len(rx), 2):
    rx_samples.append(rx[x] + 1j * rx[x + 1])

for x in range(0, len(tx), 2):
    tx_samples.append(tx[x] + 1j * tx[x + 1])

rx_ampl = np.abs(rx_samples)
rx_phase = np.angle(rx_samples)
rx_time = np.arange(len(rx_samples))

rx_ampl = np.abs(tx_samples)
rx_phase = np.angle(tx_samples)
rx_time = np.arange(len(tx_samples))

# plot
# plt.subplot(3,1,1)
# plt.legend
# plt.plot(rx_time, rx_ampl)
# plt.grid(True)
# plt.title("Rx ampl")
# plt.tight_layout()

# plt.subplot(3,1,2)
# plt.legend
# plt.plot(rx_time, rx_phase)
# plt.grid(True)
# plt.title("Rx phase")
# plt.tight_layout()


# plt.subplot(3,1,3)
# plt.legend
# plt.plot(rx_time, rx[0::2])
# plt.plot(rx_time, rx[1::2])
# plt.grid(True)


# plot
plt.subplot(3, 1, 1)
plt.legend
plt.plot(rx_time, rx_ampl)
plt.grid(True)
plt.title("Rx ampl")
plt.tight_layout()

plt.subplot(3, 1, 2)
plt.legend
plt.plot(rx_time, rx_phase)
plt.grid(True)
plt.title("Rx phase")
plt.tight_layout()


plt.subplot(3, 1, 3)
plt.legend
plt.plot(rx_time, tx[0::2])
plt.plot(rx_time, tx[1::2])
plt.grid(True)
plt.show()
