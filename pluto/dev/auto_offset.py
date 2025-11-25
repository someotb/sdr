import numpy as np
def gardner(complex_symbols_saw):
    p1 = 0
    p2 = 0
    offset = 0
    BnTs = 0.01
    Nsp = 10 # Samples per symbol
    zeta = np.sqrt(2)/2
    teta = (BnTs/Nsp)/(zeta + 1/(4*zeta))
    css = complex_symbols_saw
    Kp = 0.002
    K1 = (-4 * zeta * teta)/((1 + 2 * zeta * teta + teta ** 2) * Kp)
    K2 = (-4 * teta ** 2)/((1 + 2 * zeta * teta + teta ** 2) * Kp)
    list_of_offset = []
    
    for n in range(0, len(css)-22):
        err = (np.real(css[n + Nsp + Nsp]) - np.real(css[n + Nsp])) * np.real(css[n + Nsp + Nsp//2]) + (np.imag(css[n + Nsp + Nsp]) - np.imag(css[n + Nsp])) * np.imag(css[n + Nsp + Nsp//2])
        err = np.real(err)
        p1 = err * K1
        p2 = p2 + p1 + err * K2
        while p2 > 1:
            p2 = p2 - 1
        while p2 < -1:
            p2 = p2 + 1
        offset = np.round(p2 * Nsp)
        list_of_offset.append(offset)
        
    return int(offset), list_of_offset