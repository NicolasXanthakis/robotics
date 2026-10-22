"""
Generate the original palm shell in metres; no downloaded assets.

A single bevelled solid widens from wrist to knuckles. Flat faces keep the
robotic style simple; there are no anatomical pads or overlapping shells.
"""
import math
from pathlib import Path

triangles = []


def surface(rings):
    """Triangulate and cap a set of circular cross sections."""
    for a, b in zip(rings, rings[1:]):
        for i in range(len(a)):
            j = (i+1) % len(a)
            triangles.extend(((a[i], a[j], b[j]), (a[i], b[j], b[i])))
    for ring, reverse in ((rings[0], True), (rings[-1], False)):
        center = tuple(sum(p[k] for p in ring)/len(ring) for k in range(3))
        for i in range(len(ring)):
            a, b = ring[i], ring[(i+1) % len(ring)]
            triangles.append((center, b, a) if reverse else (center, a, b))


# Four bevelled rectangular sections: narrow wrist, broad knuckle edge.
sections = [(-.055, .024, .007), (-.047, .028, .010),
            (.044, .044, .010), (.055, .042, .008)]
corners = [(-1, -.65), (-.85, -1), (.85, -1), (1, -.65),
           (1, .65), (.85, 1), (-.85, 1), (-1, .65)]
surface([[(width*x-.002, depth*y, z) for x, y in corners]
         for z, width, depth in sections])
output = ['solid palm']
for a, b, c in triangles:
    u = [b[k]-a[k] for k in range(3)]
    v = [c[k]-a[k] for k in range(3)]
    n = [u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0]]
    length = math.hypot(*n)
    n = [x/length for x in n] if length else [0, 0, 0]
    output.append('facet normal '+' '.join(f'{x:.7g}' for x in n)+'\n outer loop')
    output.extend('  vertex '+' '.join(f'{x:.7g}' for x in p) for p in (a, b, c))
    output.append(' endloop\nendfacet')
output.append('endsolid palm')
Path(__file__).with_name('palm.stl').write_text('\n'.join(output)+'\n')
