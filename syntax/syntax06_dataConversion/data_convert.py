raw = [45, 230, 35, 131, 109, 242, 14, 23]

x = 0
for n in range(8):
    x += raw[n] << (8 * n)
print(f'x = {x}')