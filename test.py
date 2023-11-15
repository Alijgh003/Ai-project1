from collections import OrderedDict

a = OrderedDict()


a[1] = 10

a[2] = 2

a[3] = 3

b = OrderedDict()

b[4] = 11
b[2] =12
b[6] = 14

print(set(a))
print(set(b))

print(len((set(a).intersection(set(b)))))
