from matplotlib import pyplot as plt
import numpy as np

rx = np.fromfile(f"/home/someotb/Files/Code/sdr/pluto/dev/build/rx.pcm", dtype=np.int16)

samples = []

for x in range(0, len(rx), 2):
    samples.append((rx[x] + 1j * rx[x+1])/np.max(rx))
    
impulse = np.ones(10)

I = np.real(samples)
Q = np.imag(samples)

i_convolve = np.convolve(I, impulse)
q_convolve = np.convolve(Q, impulse)
i_con_dec = []
q_con_dec = []

for i in range(-1, len(i_convolve), 10):
    i_con_dec.append(i_convolve[i])
    q_con_dec.append(q_convolve[i])

# Убираем шумы
i_con_dec = [x for x in i_con_dec if not (abs(x) <= 1)]
q_con_dec = [x for x in q_con_dec if not (abs(x) <= 1)]
    
plt.scatter(i_con_dec, q_con_dec)
plt.axhline()
plt.axvline()
plt.show()