"""Generate a broad thenar wedge in palm coordinates, with bevelled faces."""

import math
from pathlib import Path

# Blend into the wrist heel and rise toward the thumb/index web space.
outline = [(.010, -.048), (.027, -.047), (.044, -.034), (.056, -.015),
           (.057, .001), (.048, .014), (.030, .018), (.018, -.005)]
center_x = sum(x for x, z in outline)/len(outline)
center_z = sum(z for x, z in outline)/len(outline)
rings = [[(center_x+(x-center_x)*scale, y, center_z+(z-center_z)*scale)
          for x, z in outline]
         for y, scale in [(-.017, .78), (-.010, 1.), (.005, 1.), (.009, .82)]]
triangles = []
for lower, upper in zip(rings, rings[1:]):
    for i in range(len(outline)):
        j = (i+1) % len(outline)
        triangles.extend(((lower[i], lower[j], upper[j]),
                          (lower[i], upper[j], upper[i])))
for ring in (rings[0], rings[-1]):
    center = (center_x, ring[0][1], center_z)
    for i in range(len(outline)):
        triangles.append((center, ring[i], ring[(i+1) % len(outline)]))
output = ['solid thumb_pad']
solid_center = (center_x, -.004, center_z)
for a, b, c in triangles:
    u = [b[k]-a[k] for k in range(3)]
    v = [c[k]-a[k] for k in range(3)]
    normal = [u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0]]
    outward = [(a[k]+b[k]+c[k])/3-solid_center[k] for k in range(3)]
    if sum(n*d for n, d in zip(normal, outward)) < 0:
        b, c = c, b
        normal = [-n for n in normal]
    length = math.hypot(*normal)
    output.append('facet normal '+' '.join(f'{x/length:.7g}' for x in normal))
    output.append(' outer loop')
    output.extend('  vertex '+' '.join(f'{x:.7g}' for x in p) for p in (a, b, c))
    output.append(' endloop\nendfacet')
output.append('endsolid thumb_pad')
Path(__file__).with_name('thumb_pad.stl').write_text('\n'.join(output)+'\n')
