import numpy as np

def gardner(complex_symbols_saw):
    p1 = 0
    p2 = 0
    offset = 0
    BnTs = 0.01
    Nsp = 10 # Samples per symbol
    zeta = np.sqrt(2)/2
    teta = (BnTs/Nsp)/(zeta + 1/(4*zeta))
    s = complex_symbols_saw[5000:6000]
    Kp = 2
    K1 = (-4 * zeta * teta)/((1 + 2 * zeta * teta + teta ** 2) * Kp)
    K2 = (-4 * teta ** 2)/((1 + 2 * zeta * teta + teta ** 2) * Kp)
    list_of_offset = []
    list_of_err = []
    
    for i in range(0, len(s)//10-1):
        n = offset
        err = (np.real(s[n + Nsp + Nsp*i]) - np.real(s[n + Nsp*i])) * np.real(s[n + Nsp//2 + Nsp*i])
        err += (np.imag(s[n + Nsp + Nsp*i]) - np.imag(s[n + Nsp*i])) * np.imag(s[n + Nsp//2 + Nsp*i])
        list_of_err.append(err)
        p1 = err * K1
        p2 = p2 + p1 + err * K2
        p2 %= 1
            
        offset = int(np.round(p2 * Nsp))
        list_of_offset.append(offset)
    return int(offset), list_of_offset, list_of_err