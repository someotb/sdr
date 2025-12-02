from matplotlib import pyplot as plt
import numpy as np
from auto_offset import gardner

rx = np.fromfile(f"/home/someotb/Files/Code/sdr/pluto/dev/build/rx.pcm", dtype=np.int16)

samples = []

for x in range(0, len(rx), 2):
    samples.append((rx[x] + 1j * rx[x+1])/np.max(rx))

plt.figure()
plt.scatter(np.real(samples), np.imag(samples))
plt.title("Real and Imag parts")
plt.axhline()
plt.axvline()

impulse = np.ones(10)

I = np.real(samples)
Q = np.imag(samples)

i_convolve = np.convolve(I, impulse)
q_convolve = np.convolve(Q, impulse)

i_con_dec = []
q_con_dec = []
list_offset = []
list_err = []

offset, list_offset, list_err = gardner(i_convolve + 1j * q_convolve)
print(f"offset - {offset}") 

plt.figure()
plt.title("err")
plt.plot(np.arange(len(list_err)), list_err)
plt.grid()

plt.figure()
plt.title("offset")
plt.plot(np.arange(len(list_offset)), list_offset)
plt.grid()
# plt.ylim([-20, 20])
# plt.xlim([4060, 4560])

for i in range(offset, len(i_convolve), 10):
    i_con_dec.append(i_convolve[i])
    q_con_dec.append(q_convolve[i])

# Убираем шумы
# i_con_dec = [x for x in i_con_dec if not (abs(x) <= 1)]
# q_con_dec = [x for x in q_con_dec if not (abs(x) <= 1)]

# print("lenght i_con_dec: ", len(i_con_dec))
# print("lenght q_con_dec: ", len(q_con_dec))

plt.figure()
plt.subplot(2,1,1)
plt.scatter(i_con_dec, q_con_dec)
plt.axhline()
plt.axvline()
plt.subplot(2,1,2)
plt.grid()
plt.xlim([5000, 5500])
plt.ylim([-30, 30])
plt.scatter(np.arange(len(i_convolve)), i_convolve)
plt.scatter(np.arange(len(q_convolve)), q_convolve)
plt.show()