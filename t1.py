import numpy as np

x = np.array([1, 2, 3, 4])

print(x*5)


x = np.array([[1, 2, 3, 4],
              [1, 2, 3, 4],
              [1, 2, 3, 4]])

print(x[:, 0])
print(np.sqrt(x[0, 0]**2 + x[1, 0]**2 + x[2,0]**2))


cableVecs = np.array([[1, 2, 3, 4],
                      [1, 1, 1, 1],
                      [1, 1, 2, 1]])

cableVec_unit = np.zeros((3, 4))
for i in range(4):
    vec = cableVecs[:, i]
    print(f'vec: {vec}')
    mag = np.linalg.norm(vec)
    print(f'unit: {vec/mag}')
    cableVec_unit[:, i] = vec/mag

print(cableVec_unit)



print('====================')


x = np.array([[1], [2], [1]])
y = np.array([[2], [2], [3]])

z = x*y
print(z)



z = [0, 0, 0, 1]

print(f'z: {z[0]}')

print(x*3)

print('=======')

u = np.ones((4, 10))
print(u[:, 0])