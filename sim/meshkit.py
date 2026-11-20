"""Tiny procedural mesh kit (numpy only) for the robot's visual model.

Every builder returns a Mesh with positions, smooth normals and UVs, written as OBJ.
Used by make_assets.py; nothing here affects the physics.
"""
import numpy as np


class Mesh:
    def __init__(self):
        self.v, self.n, self.t, self.f = [], [], [], []

    def add(self, V, N, F, T=None):
        """Append vertices (V), normals (N), triangles (F, indices into V) and optional UVs (T).
        Triangles are re-wound so they face the way the normals point."""
        V, N, F = np.asarray(V, float), np.asarray(N, float), np.asarray(F, int)
        T = np.zeros((len(V), 2)) if T is None else np.asarray(T, float)
        geo = np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]])
        flip = np.einsum("ij,ij->i", geo, N[F].sum(1)) < 0
        F = F.copy()
        F[flip] = F[flip][:, ::-1]
        keep = np.linalg.norm(geo, axis=1) > 1e-14          # drop zero-area triangles
        base = sum(len(x) for x in self.v)
        self.v.append(V), self.n.append(N), self.t.append(T), self.f.append(F[keep] + base)
        return self

    def merge(self, *others):
        for o in others:
            if o.v:
                V, N, T, F = o.arrays()
                base = sum(len(x) for x in self.v)
                self.v.append(V), self.n.append(N), self.t.append(T), self.f.append(F + base)
        return self

    def arrays(self):
        return np.concatenate(self.v), np.concatenate(self.n), np.concatenate(self.t), np.concatenate(self.f)

    def moved(self, R=None, p=(0, 0, 0)):
        """A copy rotated by R (3x3, may mirror) and shifted by p."""
        R = np.eye(3) if R is None else np.asarray(R, float)
        V, N, T, F = self.arrays()
        out = Mesh()
        out.v, out.t = [V @ R.T + np.asarray(p, float)], [T]
        out.n = [N @ np.linalg.inv(R)]
        out.n[0] /= np.linalg.norm(out.n[0], axis=1, keepdims=True) + 1e-12
        out.f = [F[:, ::-1] if np.linalg.det(R) < 0 else F]
        return out

    def write(self, path):
        V, N, T, F = self.arrays()
        with open(path, "w") as fh:
            fh.write("".join(f"v {a:.5f} {b:.5f} {c:.5f}\n" for a, b, c in V))
            fh.write("".join(f"vt {a:.4f} {b:.4f}\n" for a, b in T))
            fh.write("".join(f"vn {a:.3f} {b:.3f} {c:.3f}\n" for a, b, c in N))
            fh.write("".join(f"f {a}/{a}/{a} {b}/{b}/{b} {c}/{c}/{c}\n" for a, b, c in F + 1))
        return len(F)


# -- rotations ---------------------------------------------------------------
def frame(x=None, y=None, z=None):
    """Rotation whose columns are where the local x, y, z axes point (give any two)."""
    x, y, z = (None if a is None else np.asarray(a, float) / np.linalg.norm(a) for a in (x, y, z))
    if x is None:
        x = np.cross(y, z)
    if y is None:
        y = np.cross(z, x)
    if z is None:
        z = np.cross(x, y)
    return np.column_stack([x, y, z])


def z_to(d):
    """Rotation taking local +z to direction d."""
    d = np.asarray(d, float) / np.linalg.norm(d)
    a = np.array([1.0, 0, 0]) if abs(d[0]) < 0.9 else np.array([0, 1.0, 0])
    x = np.cross(a, d)
    return frame(x=x, z=d)


# -- builders ----------------------------------------------------------------
def rounded_box(hx, hy, hz, r, n=4):
    """Box with half-sizes (hx, hy, hz) and edge radius r."""
    h = np.array([hx, hy, hz])
    r = min(r, h.min())
    g = np.linspace(-1, 1, 2 * n)
    m = Mesh()
    for ax in range(3):
        b, c = (ax + 1) % 3, (ax + 2) % 3
        for sgn in (-1, 1):
            P = np.zeros((2 * n, 2 * n, 3))
            P[..., ax], P[..., b], P[..., c] = sgn, g[:, None], g[None, :]
            D = P / np.linalg.norm(P, axis=2, keepdims=True)
            V = (r * D + np.sign(D) * (h - r)).reshape(-1, 3)
            idx = np.arange((2 * n) ** 2).reshape(2 * n, 2 * n)
            a, b_, c_, d = idx[:-1, :-1].ravel(), idx[1:, :-1].ravel(), idx[1:, 1:].ravel(), idx[:-1, 1:].ravel()
            F = np.concatenate([np.column_stack([a, b_, c_]), np.column_stack([a, c_, d])])
            T = np.column_stack([V[:, 0] / (2 * hx) + 0.5, V[:, 1] / (2 * hy) + 0.5])
            m.add(V, D.reshape(-1, 3), F, T)
    return m


def box(hx, hy, hz):
    return rounded_box(hx, hy, hz, 1e-5, n=1)


def slab_uv(hx, hy, z0, z1):
    """Thin box whose top face carries the whole texture (u along +x, v along +y)."""
    m = Mesh()
    c = np.array([[-hx, -hy], [hx, -hy], [hx, hy], [-hx, hy]])
    top = np.column_stack([c, np.full(4, z1)])
    bot = np.column_stack([c, np.full(4, z0)])
    quad = np.array([[0, 1, 2], [0, 2, 3]])
    m.add(top, np.tile([0, 0, 1.0], (4, 1)), quad, [[0, 0], [1, 0], [1, 1], [0, 1]])
    m.add(bot, np.tile([0, 0, -1.0], (4, 1)), quad, np.full((4, 2), 0.004))
    for i in range(4):
        j = (i + 1) % 4
        nrm = np.array([c[j, 1] - c[i, 1], c[i, 0] - c[j, 0], 0.0])
        m.add([bot[i], bot[j], top[j], top[i]], np.tile(nrm / np.linalg.norm(nrm), (4, 1)), quad, np.full((4, 2), 0.004))
    return m


def lathe(profile, n=20):
    """Revolve a profile [(radius, z), ...] around z. Each profile segment is shaded on its own."""
    m = Mesh()
    ang = np.linspace(0, 2 * np.pi, n + 1)
    cs, sn = np.cos(ang), np.sin(ang)
    for (r0, z0), (r1, z1) in zip(profile[:-1], profile[1:]):
        dr, dz = r1 - r0, z1 - z0
        L = np.hypot(dr, dz)
        if L < 1e-9:
            continue
        nr, nz = dz / L, -dr / L
        V = np.concatenate([np.column_stack([r0 * cs, r0 * sn, np.full(n + 1, z0)]),
                            np.column_stack([r1 * cs, r1 * sn, np.full(n + 1, z1)])])
        N = np.tile(np.column_stack([nr * cs, nr * sn, np.full(n + 1, nz)]), (2, 1))
        i = np.arange(n)
        F = np.concatenate([np.column_stack([i, i + 1, i + n + 2]), np.column_stack([i, i + n + 2, i + n + 1])])
        m.add(V, N, F)
    return m


def cyl(r, z0, z1, n=20, bevel=0.0):
    """Closed cylinder along z, optional bevel on both rims."""
    b = min(bevel, r * 0.9, (z1 - z0) / 2)
    return lathe([(0, z0), (r - b, z0), (r, z0 + b), (r, z1 - b), (r - b, z1), (0, z1)], n)


def sphere(r, n=14):
    a = np.linspace(-np.pi / 2, np.pi / 2, n // 2 + 1)
    return lathe([(r * np.cos(t), r * np.sin(t)) for t in a], n)


def rounded_rect(hx, hy, r, n=6):
    """CCW outline of a rectangle with rounded corners."""
    r = max(min(r, hx, hy), 1e-6)
    pts = []
    for k, (sx, sy) in enumerate(((1, 1), (-1, 1), (-1, -1), (1, -1))):
        for a in np.linspace(0, np.pi / 2, n + 1) + k * np.pi / 2:
            pts.append([sx * (hx - r) + r * np.cos(a), sy * (hy - r) + r * np.sin(a)])
    return np.array(pts)


def hull(points):
    """2D convex hull (CCW), monotone chain."""
    P = sorted(map(tuple, np.round(points, 9)))
    cross = lambda o, a, b: (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lo, up = [], []
    for p in P:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0:
            lo.pop()
        lo.append(p)
    for p in reversed(P):
        while len(up) >= 2 and cross(up[-2], up[-1], p) <= 0:
            up.pop()
        up.append(p)
    return np.array(lo[:-1] + up[:-1])


def link_outline(p0, r0, p1, r1, n=28):
    """Outline of a link plate: the hull of two circles (a 'dog bone')."""
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    c = np.column_stack([np.cos(a), np.sin(a)])
    return hull(np.concatenate([np.asarray(p0) + r0 * c, np.asarray(p1) + r1 * c]))


def inset(poly, d):
    """Move every vertex of a convex CCW outline inward by d."""
    e = np.roll(poly, -1, 0) - poly
    nrm = np.column_stack([e[:, 1], -e[:, 0]])
    nrm /= np.linalg.norm(nrm, axis=1, keepdims=True)
    vn = nrm + np.roll(nrm, 1, 0)
    vn /= np.linalg.norm(vn, axis=1, keepdims=True)
    return poly - d * vn


def loft(rings):
    """Skin through convex CCW outlines [(outline Nx2, z), ...] (same vertex count), capped at both ends."""
    m = Mesh()
    n = len(rings[0][0])
    i = np.arange(n)
    j = (i + 1) % n
    for (A, za), (B, zb) in zip(rings[:-1], rings[1:]):
        A3, B3 = np.column_stack([A, np.full(n, za)]), np.column_stack([B, np.full(n, zb)])
        t = np.roll(A3, -1, 0) - np.roll(A3, 1, 0)
        N = np.cross(t, B3 - A3)
        N /= np.linalg.norm(N, axis=1, keepdims=True) + 1e-12
        m.add(np.concatenate([A3, B3]), np.concatenate([N, N]),
              np.concatenate([np.column_stack([i, j, j + n]), np.column_stack([i, j + n, i + n])]))
    for (P, z), up in ((rings[0], -1.0), (rings[-1], 1.0)):
        V = np.column_stack([P, np.full(n, z)])
        V = np.concatenate([V, [[*P.mean(0), z]]])
        m.add(V, np.tile([0, 0, up], (n + 1, 1)), np.column_stack([i, j, np.full(n, n)]))
    return m


def plate(outline, z0, z1, bevel=0.0005):
    """Flat plate with bevelled rims."""
    b = min(bevel, (z1 - z0) / 2)
    return loft([(inset(outline, b), z0), (outline, z0 + b), (outline, z1 - b), (inset(outline, b), z1)])


def tube(path, r, n=8, closed=False):
    """Round tube along a 3D path (parallel-transport frames), capped unless closed."""
    P = np.asarray(path, float)
    if closed:
        P = np.concatenate([P, P[:1]])
    T = np.gradient(P, axis=0)
    if closed:
        T[0] = T[-1] = P[1] - P[-2]
    T /= np.linalg.norm(T, axis=1, keepdims=True)
    u = np.cross(T[0], [0, 0, 1.0] if abs(T[0][2]) < 0.9 else [1.0, 0, 0])
    u /= np.linalg.norm(u)
    ang = np.linspace(0, 2 * np.pi, n, endpoint=False)
    V, N = [], []
    for p, t in zip(P, T):
        u = u - t * (u @ t)
        u /= np.linalg.norm(u)
        w = np.cross(t, u)
        ring = np.outer(np.cos(ang), u) + np.outer(np.sin(ang), w)
        V.append(p + r * ring), N.append(ring)
    V, N = np.concatenate(V), np.concatenate(N)
    F = []
    for k in range(len(P) - 1):
        a = k * n + np.arange(n)
        b = k * n + (np.arange(n) + 1) % n
        F += [np.column_stack([a, b, b + n]), np.column_stack([a, b + n, a + n])]
    m = Mesh().add(V, N, np.concatenate(F))
    if not closed:
        for k, sgn in ((0, -1.0), (len(P) - 1, 1.0)):
            ring = V[k * n:(k + 1) * n]
            m.add(np.concatenate([ring, [P[k]]]), np.tile(sgn * T[k], (n + 1, 1)),
                  np.column_stack([np.arange(n), (np.arange(n) + 1) % n, np.full(n, n)]))
    return m


def torus(R, r, n=24, k=8):
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    return tube(np.column_stack([R * np.cos(a), R * np.sin(a), np.zeros(n)]), r, k, closed=True)


def smooth(points, k=6):
    """Round the corners of a path (Chaikin corner cutting)."""
    P = np.asarray(points, float)
    for _ in range(3):
        Q = [P[0]]
        for a, b in zip(P[:-1], P[1:]):
            Q += [0.75 * a + 0.25 * b, 0.25 * a + 0.75 * b]
        P = np.array(Q + [P[-1]])
    return P
