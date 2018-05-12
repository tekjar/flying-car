* for a point in 3d space to be colinear

det of [(x1, y1, z1), (x2, y2, z2), (x3, y3, z3)] is a necessary but not sufficient condition

* for a point in 2d space to be colinear

det of [(x1, y1, 1), (x2, y2, 1), (x3, y3, 1)] is a sufficient condition

```
determininent of
[
    x1, y1, 1
    x2, y2, 1
    x3, y3, 1
]

== 0
```

using numpy

```
mat = numpy.vstack((
    [x1, y1, 1],
    [x2, y2, 1],
    [x3, y3, 1]
))

numpy.linalg.det(mat)
```