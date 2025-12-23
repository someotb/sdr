import numpy as np

def gardner(complex_symbols_after_convolve, SPS = 10):
    K1 = 0; K2 = 0; p1 = 0; p2 = 0; e = 0; offset = 0; Kp = 4
    BnTs = 0.01; 
    zeta = np.sqrt(2)/2
    teta = ((BnTs)/10)/(zeta + 1/(4*zeta))
    K1 = (-4*zeta*teta)/((1 + 2*zeta*teta + teta**2)*Kp)
    K2 = (-4*teta**2)/((1 + 2*zeta*teta + teta**2)*Kp)
    s = complex_symbols_after_convolve
    offset_list = []
    fixed = []
    for i in range(0, len(s)//SPS-1):
        n = offset
        e  = (np.real(s[n + SPS + SPS*i]) - np.real(s[n + SPS*i])) * np.real(s[n + SPS//2 + SPS*i])
        e += (np.imag(s[n + SPS + SPS*i]) - np.imag(s[n + SPS*i])) * np.imag(s[n + SPS//2 + SPS*i])
        p1 = e*K1
        p2 += p1 + e * K2
        p2 %= 1
        offset = int(np.round(p2*SPS))

        fixed.append(s[SPS*i + offset])
        offset_list.append(offset)
    offset_list = np.array(offset_list, dtype=int)
    fixed = np.array(fixed)
    return fixed, offset_list