import matplotlib.pyplot as plt

# Исходные точки
r  = (-5.33015, -0.506815)
v  = (-5.05134, -1.67989)
p1 = (-5.62943,  0.752358)
p2 = (-4.44298, -3.73435)

# Разворачиваем координаты для удобства
xr, yr = r
xv, yv = v
xp1, yp1 = p1
xp2, yp2 = p2

plt.figure(figsize=(7, 7))

# Рисуем точки
plt.scatter([xv], [yv], s=80, color='black', label='v (vertex)')
plt.scatter([xp1], [yp1], s=80, color='blue', label='p1')
plt.scatter([xp2], [yp2], s=80, color='green', label='p2')
plt.scatter([xr], [yr], s=80, color='red', label='r')

# Подписываем
plt.text(xv,  yv,  '  v',  fontsize=12)
plt.text(xp1, yp1, '  p1', fontsize=12)
plt.text(xp2, yp2, '  p2', fontsize=12)
plt.text(xr,  yr,  '  r',  fontsize=12)

# Рисуем лучи v→p1 и v→p2
plt.plot([xv, xp1], [yv, yp1], color='blue')
plt.plot([xv, xp2], [yv, yp2], color='green')

# Рисуем луч v→r
plt.plot([xv, xr], [yv, yr], color='red', linestyle='--')

plt.title("Визуализация точек v, p1, p2 и r")
plt.axis('equal')
plt.grid(True)
plt.legend()
plt.show()
