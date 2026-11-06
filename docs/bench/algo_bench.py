# -*- coding: utf-8 -*-
"""
纯算法基准/验证脚本（只用 Python 标准库 math / struct / random，不使用 numpy 等优化库）。
float32 行为通过 struct 逐运算舍入模拟（每次 + - * / sqrt 之后立即舍入到 IEEE-754 binary32，
不模拟 FMA 融合；arm64 默认开启 FP 收缩，实际误差介于本模型与更小值之间）。

运行：D:\\app\\miniconda\\envs\\7z\\python.exe docs\\bench\\algo_bench.py
"""
import math
import random
import struct
import time

_pack = struct.Struct('f')


def f32(x):
    return _pack.unpack(_pack.pack(x))[0]


def fa(a, b): return f32(a + b)
def fs(a, b): return f32(a - b)
def fm(a, b): return f32(a * b)
def fd(a, b): return f32(a / b)
def fsq(a): return f32(math.sqrt(a))


def dot3f(a, b):
    return fa(fa(fm(a[0], b[0]), fm(a[1], b[1])), fm(a[2], b[2]))


def header(t):
    print('\n' + '=' * 78)
    print(t)
    print('=' * 78)


# ---------------------------------------------------------------------------
# 1. 三角化精度：现有 float 法方程克莱姆 vs double 克莱姆 vs 中点法(1-b^2) vs 中点法(叉积)
# ---------------------------------------------------------------------------
def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return [[c, 0, s], [0, 1, 0], [-s, 0, c]]


def matvec(R, v):
    return [R[i][0] * v[0] + R[i][1] * v[1] + R[i][2] * v[2] for i in range(3)]


def transpose(R):
    return [[R[j][i] for j in range(3)] for i in range(3)]


def tri_cramer(P1, P2, x1, y1, x2, y2, F):
    """Converter::TriangulateWithCenters 的逐行复刻；F 为舍入函数集合（float32 或 double）"""
    add, sub, mul, div = F
    def row(x, P, r):
        return [sub(mul(x, P[2][k]), P[r][k]) for k in range(4)]
    a0 = row(x1, P1, 0); a1 = row(y1, P1, 1); a2 = row(x2, P2, 0); a3 = row(y2, P2, 1)
    A = [a0, a1, a2, a3]
    def s4(i, j):
        acc = mul(A[0][i], A[0][j])
        for r in range(1, 4):
            acc = add(acc, mul(A[r][i], A[r][j]))
        return acc
    m00, m01, m02 = s4(0, 0), s4(0, 1), s4(0, 2)
    m11, m12, m22 = s4(1, 1), s4(1, 2), s4(2, 2)
    b0, b1, b2 = -s4(0, 3), -s4(1, 3), -s4(2, 3)
    c00 = sub(mul(m11, m22), mul(m12, m12))
    c01 = sub(mul(m02, m12), mul(m01, m22))
    c02 = sub(mul(m01, m12), mul(m02, m11))
    det = add(add(mul(m00, c00), mul(m01, c01)), mul(m02, c02))
    if abs(det) < 1e-12:
        return None
    inv = div(1.0, det)
    c11 = sub(mul(m00, m22), mul(m02, m02))
    c12 = sub(mul(m01, m02), mul(m00, m12))
    c22 = sub(mul(m00, m11), mul(m01, m01))
    X = mul(add(add(mul(c00, b0), mul(c01, b1)), mul(c02, b2)), inv)
    Y = mul(add(add(mul(c01, b0), mul(c11, b1)), mul(c12, b2)), inv)
    Z = mul(add(add(mul(c02, b0), mul(c12, b1)), mul(c22, b2)), inv)
    return [X, Y, Z]


F32 = (fa, fs, fm, fd)
F64 = (lambda a, b: a + b, lambda a, b: a - b, lambda a, b: a * b, lambda a, b: a / b)


def mid_1mb2_f32(O1, d1, O2, d2):
    """Converter::TriangulateMidpoint 复刻（d1,d2 为 float 归一化单位向量）"""
    rx, ry, rz = fs(O1[0], O2[0]), fs(O1[1], O2[1]), fs(O1[2], O2[2])
    b = dot3f(d1, d2)
    det = fs(1.0, fm(b, b))
    if det < 1e-6:
        return None
    inv = fd(1.0, det)
    r = (rx, ry, rz)
    d = dot3f(d1, r)
    e = dot3f(d2, r)
    s = fm(fs(fm(b, e), d), inv)
    t = fm(fs(e, fm(b, d)), inv)
    if s <= 0 or t <= 0:
        return None
    return [fm(0.5, fa(fa(O1[k], fm(s, d1[k])), fa(O2[k], fm(t, d2[k])))) for k in range(3)]


def cross3f(a, b):
    return [fs(fm(a[1], b[2]), fm(a[2], b[1])),
            fs(fm(a[2], b[0]), fm(a[0], b[2])),
            fs(fm(a[0], b[1]), fm(a[1], b[0]))]


def mid_cross_f32(O1, r1, O2, r2):
    """提议：叉积式中点法，射线无需归一化（省 2 sqrt + 2 除 + 6 乘），无 1-b^2 相消"""
    w = [fs(O2[k], O1[k]) for k in range(3)]
    n = cross3f(r1, r2)
    nn = dot3f(n, n)
    if nn <= 0.0:
        return None
    inv = fd(1.0, nn)
    s = fm(dot3f(cross3f(w, r2), n), inv)
    t = fm(dot3f(cross3f(w, r1), n), inv)
    if s <= 0 or t <= 0:
        return None
    return [fm(0.5, fa(fa(O1[k], fm(s, r1[k])), fa(O2[k], fm(t, r2[k])))) for k in range(3)]


def rel_err(p, g):
    if p is None:
        return float('nan')
    num = math.sqrt(sum((p[k] - g[k]) ** 2 for k in range(3)))
    den = math.sqrt(sum(g[k] ** 2 for k in range(3)))
    return num / den


def bench_triangulation():
    header('1. 三角化精度（无噪声投影，真值已知；相对误差 |X-Xgt|/|Xgt|）')
    rnd = random.Random(1)
    fx = fy = 500.0; cx, cy = 320.0, 240.0
    K = [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
    stats = {}
    for parallax_deg in (0.2, 0.5, 1.0, 2.0, 5.0):
        errs = {'Cramer_f32_px': [], 'Cramer_f64_px': [], 'Cramer_f32_norm': [],
                'Mid_1-b2_f32': [], 'Mid_cross_f32': []}
        for _ in range(400):
            depth = rnd.uniform(1.0, 6.0)
            Xgt = [rnd.uniform(-1.5, 1.5), rnd.uniform(-1.0, 1.0), depth]
            base = 2.0 * depth * math.tan(math.radians(parallax_deg) / 2.0)
            ang = rnd.uniform(-0.05, 0.05)
            R = rot_y(ang)
            O2 = [base, rnd.uniform(-0.01, 0.01), rnd.uniform(-0.01, 0.01)]
            t = [-v for v in matvec(R, O2)]
            Xc2 = [a + b for a, b in zip(matvec(R, Xgt), t)]
            u1 = fx * Xgt[0] / Xgt[2] + cx; v1 = fy * Xgt[1] / Xgt[2] + cy
            u2 = fx * Xc2[0] / Xc2[2] + cx; v2 = fy * Xc2[1] / Xc2[2] + cy
            u1, v1, u2, v2 = f32(u1), f32(v1), f32(u2), f32(v2)
            # Initializer 路径：像素坐标 + P = K[R|t]
            P1 = [[f32(K[i][j]) if j < 3 else 0.0 for j in range(4)] for i in range(3)]
            Rt = [[R[i][0], R[i][1], R[i][2], t[i]] for i in range(3)]
            P2 = [[f32(sum(K[i][k] * Rt[k][j] for k in range(3))) for j in range(4)] for i in range(3)]
            errs['Cramer_f32_px'].append(rel_err(tri_cramer(P1, P2, u1, v1, u2, v2, F32), Xgt))
            errs['Cramer_f64_px'].append(rel_err(tri_cramer(P1, P2, u1, v1, u2, v2, F64), Xgt))
            # LocalMapping 回退路径：归一化坐标 + P = [R|t]
            n1x, n1y = f32((u1 - cx) / fx), f32((v1 - cy) / fy)
            n2x, n2y = f32((u2 - cx) / fx), f32((v2 - cy) / fy)
            Q1 = [[1.0, 0, 0, 0], [0, 1.0, 0, 0], [0, 0, 1.0, 0]]
            Q2 = [[f32(Rt[i][j]) for j in range(4)] for i in range(3)]
            errs['Cramer_f32_norm'].append(rel_err(tri_cramer(Q1, Q2, n1x, n1y, n2x, n2y, F32), Xgt))
            # LocalMapping 中点法：射线 r = Rwc * (xn, yn, 1)
            Rwc2 = transpose(R)
            r1 = [n1x, n1y, 1.0]
            Rwc2f = [[f32(Rwc2[i][j]) for j in range(3)] for i in range(3)]
            r2 = [fa(fa(fm(Rwc2f[i][0], n2x), fm(Rwc2f[i][1], n2y)), Rwc2f[i][2]) for i in range(3)]
            O1f = [0.0, 0.0, 0.0]; O2f = [f32(v) for v in O2]
            inv1 = fd(1.0, fsq(dot3f(r1, r1))); inv2 = fd(1.0, fsq(dot3f(r2, r2)))
            d1 = [fm(v, inv1) for v in r1]; d2 = [fm(v, inv2) for v in r2]
            errs['Mid_1-b2_f32'].append(rel_err(mid_1mb2_f32(O1f, d1, O2f, d2), Xgt))
            errs['Mid_cross_f32'].append(rel_err(mid_cross_f32(O1f, r1, O2f, r2), Xgt))
        stats[parallax_deg] = errs
    names = ['Cramer_f32_px', 'Cramer_f64_px', 'Cramer_f32_norm', 'Mid_1-b2_f32', 'Mid_cross_f32']
    print('视差(°) | ' + ' | '.join('%-16s' % n for n in names) + '   (中位数 / 最大值)')
    for p, errs in stats.items():
        cells = []
        for n in names:
            v = sorted(x for x in errs[n] if not math.isnan(x))
            fails = sum(1 for x in errs[n] if math.isnan(x))
            if v:
                cells.append('%.1e/%.1e%s' % (v[len(v) // 2], v[-1], ('(f%d)' % fails) if fails else ''))
            else:
                cells.append('all-fail')
        print('%7.1f | ' % p + ' | '.join('%-16s' % c for c in cells))


# ---------------------------------------------------------------------------
# 2. g2o 数值雅可比步长：1e-9 vs cbrt(eps)≈6e-6（中心差分）
# ---------------------------------------------------------------------------
def bench_numeric_jacobian():
    header('2. g2o BaseBinaryEdge 数值雅可比步长（中心差分，double）')
    fx = 500.0
    p = [0.7, -0.3, 2.5]
    def u_of(a):
        R = rot_y(a)
        q = matvec(R, p)
        return fx * q[0] / q[2]
    def du_exact(a):
        c, s = math.cos(a), math.sin(a)
        X = c * p[0] + s * p[2]; Z = -s * p[0] + c * p[2]
        dX = -s * p[0] + c * p[2]; dZ = -c * p[0] - s * p[2]
        return fx * (dX * Z - X * dZ) / (Z * Z)
    for h in (1e-9, 1e-7, 6.055e-6, 1e-4):
        worst = 0.0
        for a in [i * 0.01 for i in range(-50, 51)]:
            num = (u_of(a + h) - u_of(a - h)) / (2 * h)
            ex = du_exact(a)
            worst = max(worst, abs(num - ex) / abs(ex))
        print('h=%-9.3g 最大相对误差 = %.3e' % (h, worst))
    print('说明：最佳 h ≈ cbrt(3·eps)·|x| ≈ 6e-6；1e-9 时舍入误差 eps/h 主导。')


# ---------------------------------------------------------------------------
# 3. Sim3::log 小角度分支
# ---------------------------------------------------------------------------
def bench_sim3_log():
    header('3. g2o Sim3::log 小角度分支 (d > 1-1e-5 ⇒ θ < 4.47e-3) 的相对误差')
    worst_old = worst_new = worst_acos = 0.0
    for i in range(1, 2001):
        th = 4.47e-3 * i / 2000.0
        sin_t = math.sin(th)
        # deltaR 的模 = 2 sin θ，精确 ω 模 = θ
        old = 0.5 * (2 * sin_t)                         # 现：omega = 0.5*deltaR
        n = 2 * sin_t
        d = math.cos(th)
        new = math.atan2(0.5 * n, d)                    # 提议：θ = atan2(|deltaR|/2, d)
        worst_old = max(worst_old, abs(old - th) / th)
        worst_new = max(worst_new, abs(new - th) / th)
        dd = 0.5 * ((1 + 2 * d) - 1)                    # 由 trace 重建 d，模拟 acos 路径
        worst_acos = max(worst_acos, abs(math.acos(min(1.0, dd)) - th) / th)
    print('现分支 omega=0.5*deltaR   最大相对误差 = %.3e (≈θ²/6)' % worst_old)
    print('acos(d) 在 θ→0 的条件数   最大相对误差 = %.3e' % worst_acos)
    print('提议 atan2(|deltaR|/2, d) 最大相对误差 = %.3e' % worst_new)
    A_old = 0.5
    th = 4.47e-3
    A_ex = (1 - math.cos(th)) / th ** 2
    print('A=1/2 相对误差 @θ=4.47e-3: %.3e；级数 1/2-θ²/24 相对误差: %.3e'
          % (abs(A_old - A_ex) / A_ex, abs(0.5 - th * th / 24 - A_ex) / A_ex))


# ---------------------------------------------------------------------------
# 4. 旋转直方图因子
# ---------------------------------------------------------------------------
def bench_histogram():
    header('4. ORBmatcher 旋转直方图：factor=1/30 vs 30/360')
    used_old, used_new = set(), set()
    for k in range(36000):
        rot = k / 100.0
        b_old = int(math.floor(rot * (1.0 / 30) + 0.5))
        if b_old == 30: b_old = 0
        b_new = int(math.floor(rot * (30 / 360.0) + 0.5))
        if b_new == 30: b_new = 0
        used_old.add(b_old); used_new.add(b_new)
    print('factor=1/30   实际使用的 bin: %s (箱宽 30°，[345,360) 与 [0,15) 被拆成 12 与 0)'
          % sorted(used_old))
    print('factor=30/360 实际使用的 bin 数: %d (箱宽 12°，360→0 回绕生效)' % len(used_new))


# ---------------------------------------------------------------------------
# 5. Sim3 阈值截断（vector<size_t>）
# ---------------------------------------------------------------------------
def bench_sim3_threshold():
    header('5. Sim3Solver 阈值 9.210*σ² 存入 size_t 的截断')
    s = 1.0
    for lvl in range(8):
        sig2 = f32(s * s)
        th = f32(9.210 * sig2)
        print('level %d σ²=%.4f 设计阈值=%.4f 实际=%d 收紧 %.2f%%'
              % (lvl, sig2, th, int(th), 100 * (th - int(th)) / th))
        s *= 1.2


# ---------------------------------------------------------------------------
# 6. RANSAC 最小集采样：整表复制 vs 交换-恢复（证明逐位等价并计数）
# ---------------------------------------------------------------------------
class LCG:
    """与项目 LCG 等价性无关：此处只需同一随机源驱动两种实现"""
    def __init__(self, seed): self.s = seed & 0xffffffff
    def randint(self, lo, hi):
        self.s = (1664525 * self.s + 1013904223) & 0xffffffff
        return lo + self.s % (hi - lo + 1)


def sets_copy(N, iters, m, seed):
    g = LCG(seed); allidx = list(range(N)); out = []; writes = 0
    for _ in range(iters):
        avail = list(allidx); writes += N
        cur = []
        for _ in range(m):
            r = g.randint(0, len(avail) - 1)
            cur.append(avail[r]); avail[r] = avail[-1]; avail.pop(); writes += 1
        out.append(cur)
    return out, writes


def sets_swap(N, iters, m, seed):
    g = LCG(seed); a = list(range(N)); out = []; writes = 0
    for _ in range(iters):
        cur = []; log = []
        n = N
        for _ in range(m):
            r = g.randint(0, n - 1)
            cur.append(a[r]); last = n - 1
            a[r], a[last] = a[last], a[r]; writes += 2
            log.append((r, last)); n -= 1
        for r, last in reversed(log):
            a[r], a[last] = a[last], a[r]; writes += 2
        out.append(cur)
    return out, writes


def bench_ransac_sampling():
    header('6. Initializer/Sim3/UIUtils 最小集采样：vAvailable=vAll 复制 vs 交换-恢复')
    for N, iters, m in ((300, 200, 8), (1500, 200, 8), (120, 300, 3), (2000, 50, 3)):
        A, wa = sets_copy(N, iters, m, 7)
        B, wb = sets_swap(N, iters, m, 7)
        print('N=%5d iters=%3d m=%d 结果逐位相同=%s 写入次数 %d -> %d (%.1fx)'
              % (N, iters, m, A == B, wa, wb, wa / wb))


# ---------------------------------------------------------------------------
# 7. 陀螺仪 getRotationMatrixFromVector(ω·dt) 角度翻倍
# ---------------------------------------------------------------------------
def android_rot_from_vec(v):
    q1, q2, q3 = v
    q0 = 1 - q1 * q1 - q2 * q2 - q3 * q3
    q0 = math.sqrt(q0) if q0 > 0 else 0.0
    # 旋转角 = 2*acos(q0)
    return 2 * math.acos(min(1.0, q0))


def bench_gyro():
    header('7. OrientationSensor.integrateGyroscope：传入 ω·dt 作为旋转向量')
    for w, dt in ((1.0, 0.005), (3.0, 0.01), (6.0, 0.02)):
        th = w * dt
        got = android_rot_from_vec([th, 0, 0])
        fix = android_rot_from_vec([math.sin(th / 2), 0, 0])
        print('|ω|=%.1f rad/s dt=%.3f s 期望 %.5f rad 现得 %.5f (×%.4f) 修正后 %.5f'
              % (w, dt, th, got, got / th, fix))


# ---------------------------------------------------------------------------
# 8. CheckHomography：H 与 -H 的得分（w<=0 拒绝 vs 符号无关判据）
# ---------------------------------------------------------------------------
def solve_lin(A, b):
    n = len(A)
    M = [row[:] + [b[i]] for i, row in enumerate(A)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(M[r][c]))
        M[c], M[p] = M[p], M[c]
        for r in range(n):
            if r != c:
                f = M[r][c] / M[c][c]
                for k in range(c, n + 1):
                    M[r][k] -= f * M[c][k]
    return [M[i][n] / M[i][i] for i in range(n)]


def homog_from4(src, dst):
    A = []; b = []
    for (x, y), (u, v) in zip(src, dst):
        A.append([x, y, 1, 0, 0, 0, -u * x, -u * y]); b.append(u)
        A.append([0, 0, 0, x, y, 1, -v * x, -v * y]); b.append(v)
    h = solve_lin(A, b)
    return [[h[0], h[1], h[2]], [h[3], h[4], h[5]], [h[6], h[7], 1.0]]


def inv3(M):
    a, b, c = M[0]; d, e, f = M[1]; g, h, i = M[2]
    A = e * i - f * h; B = -(d * i - f * g); C = d * h - e * g
    det = a * A + b * B + c * C
    return [[A / det, -(b * i - c * h) / det, (b * f - c * e) / det],
            [B / det, (a * i - c * g) / det, -(a * f - c * d) / det],
            [C / det, -(a * h - b * g) / det, (a * e - b * d) / det]]


def check_h(H21, H12, pts1, pts2, reject_neg):
    th = 5.991; score = 0.0; n = 0
    for (u1, v1), (u2, v2) in zip(pts1, pts2):
        w = H12[2][0] * u2 + H12[2][1] * v2 + H12[2][2]
        if (reject_neg and w <= 0) or w == 0: continue
        du = u1 * w - (H12[0][0] * u2 + H12[0][1] * v2 + H12[0][2])
        dv = v1 * w - (H12[1][0] * u2 + H12[1][1] * v2 + H12[1][2])
        if du * du + dv * dv > th * w * w: continue
        w2 = H21[2][0] * u1 + H21[2][1] * v1 + H21[2][2]
        if (reject_neg and w2 <= 0) or w2 == 0: continue
        du2 = u2 * w2 - (H21[0][0] * u1 + H21[0][1] * v1 + H21[0][2])
        dv2 = v2 * w2 - (H21[1][0] * u1 + H21[1][1] * v1 + H21[1][2])
        if du2 * du2 + dv2 * dv2 > th * w2 * w2: continue
        score += 2 * th - (du * du + dv * dv) / (w * w) - (du2 * du2 + dv2 * dv2) / (w2 * w2)
        n += 1
    return score, n


def bench_homography_sign():
    header('8. Initializer::CheckHomography：SVD 零向量符号任意，H 与 -H 的得分')
    rnd = random.Random(3)
    src = [(rnd.uniform(0, 640), rnd.uniform(0, 480)) for _ in range(200)]
    Hgt = [[1.02, 0.01, 5.0], [-0.02, 0.99, -3.0], [1e-5, -2e-5, 1.0]]
    def ap(H, p):
        x, y = p; w = H[2][0] * x + H[2][1] * y + H[2][2]
        return ((H[0][0] * x + H[0][1] * y + H[0][2]) / w, (H[1][0] * x + H[1][1] * y + H[1][2]) / w)
    dst = [ap(Hgt, p) for p in src]
    H = homog_from4(src[:4], dst[:4])
    Hn = [[-v for v in r] for r in H]
    for name, M in (('+H', H), ('-H', Hn)):
        Mi = inv3(M)
        s_old, n_old = check_h(M, Mi, src, dst, True)
        s_new, n_new = check_h(M, Mi, src, dst, False)
        print('%s 现实现(w<=0 拒绝) 内点=%3d score=%.1f | 符号无关判据 内点=%3d score=%.1f'
              % (name, n_old, s_old, n_new, s_new))


# ---------------------------------------------------------------------------
# 9. PnP CheckInliers 未检查 Zc>0：相机后方镜像点与前方点投影完全相同
# ---------------------------------------------------------------------------
def bench_pnp_behind():
    header('9. PnPsolver::CheckInliers 缺少 Zc>0 检查')
    fx = fy = 500.0; cx, cy = 320.0, 240.0
    P = [0.4, -0.2, 3.0]
    Pm = [-v for v in P]
    for name, q in (('前方点', P), ('后方镜像点', Pm)):
        u = cx + fx * q[0] / q[2]; v = cy + fy * q[1] / q[2]
        print('%s Zc=%+.1f 投影=(%.4f, %.4f)' % (name, q[2], u, v))
    print('结论：任何 Zc<0 的 3D 点只要与观测成镜像就会被计为内点。')


# ---------------------------------------------------------------------------
# 10. LM：每次 optimize(1) 重置 λ/ν（现状） vs 单次 optimize(N) 保留 λ/ν
# ---------------------------------------------------------------------------
def lm_fit(restart, iters, data, x0, tau=1e-5):
    """g2o OptimizationAlgorithmLevenberg::solve 的逐行复刻（2 参数指数拟合，Huber 关闭）"""
    def resid(x):
        a, b = x
        return [a * math.exp(b * t) - y for t, y in data]
    def jac(x):
        a, b = x
        return [[math.exp(b * t), a * t * math.exp(b * t)] for t, _ in data]
    def chi(x):
        return sum(r * r for r in resid(x))
    x = list(x0); lam = None; ni = 2.0; evals = 0
    hist = []
    for it in range(iters):
        r = resid(x); J = jac(x); evals += 1
        H = [[sum(J[k][i] * J[k][j] for k in range(len(J))) for j in range(2)] for i in range(2)]
        g = [-sum(J[k][i] * r[k] for k in range(len(J))) for i in range(2)]
        if restart or lam is None:
            lam = tau * max(H[0][0], H[1][1]); ni = 2.0
        cur = chi(x)
        q = 0; rho = -1
        while True:
            A = [[H[0][0] + lam, H[0][1]], [H[1][0], H[1][1] + lam]]
            det = A[0][0] * A[1][1] - A[0][1] * A[1][0]
            dx = [(g[0] * A[1][1] - A[0][1] * g[1]) / det, (A[0][0] * g[1] - A[1][0] * g[0]) / det]
            xn = [x[0] + dx[0], x[1] + dx[1]]
            tmp = chi(xn); evals += 1
            scale = lam * (dx[0] ** 2 + dx[1] ** 2) + dx[0] * g[0] + dx[1] * g[1] + 1e-3
            rho = (cur - tmp) / scale
            if rho > 0 and math.isfinite(tmp):
                alpha = 1. - (2 * rho - 1) ** 3
                alpha = min(alpha, 2. / 3.)
                lam *= max(1. / 3., alpha); ni = 2; x = xn
                break
            lam *= ni; ni *= 2; q += 1
            if q >= 10:
                break
        hist.append(chi(x))
    return x, hist, evals


def bench_lm_restart():
    header('10. LocalBA optimize(1)×N（λ 每次重置 + 每次 buildStructure） vs optimize(N)')
    rnd = random.Random(5)
    data = [(t / 10.0, 2.0 * math.exp(-1.3 * t / 10.0) + rnd.gauss(0, 0.01)) for t in range(60)]
    for x0 in ([1.0, 0.5], [5.0, -4.0], [0.2, 1.5]):
        _, h1, e1 = lm_fit(True, 5, data, x0)
        _, h2, e2 = lm_fit(False, 5, data, x0)
        print('x0=%-12s 重置: chi2=%s 求值=%d' % (x0, ['%.4g' % v for v in h1], e1))
        print('%-15s 保留: chi2=%s 求值=%d' % ('', ['%.4g' % v for v in h2], e2))
    print('注：除收敛轨迹外，optimize(1) 每轮额外执行 init()+buildStructure()（符号结构/内存重建），'
          '该成本与边数线性相关且与是否重置 λ 无关。')


# ---------------------------------------------------------------------------
# 11. 网格 CSR vs vector<vector>：内存字节与分配次数（按 libc++ arm64 布局计算）
# ---------------------------------------------------------------------------
def bench_grid_memory():
    header('11. Frame::mGrid[64][48] vector<vector<size_t>> vs CSR（arm64 libc++）')
    cells = 64 * 48
    for N in (500, 1000, 1500, 2000):
        vec_header = cells * 24
        # 每个非空 cell 一次堆分配；容量按 libc++ 2x 增长，取均匀分布期望
        per = N / cells
        nonempty = cells * (1 - math.exp(-per))
        payload = N * 8 * 1.5
        csr = (cells + 1) * 4 + N * 4
        print('N=%4d vector<vector>: 头 %d B + 载荷≈%d B, 堆分配≈%d 次 | CSR: %d B, 0 次额外分配'
              % (N, vec_header, payload, nonempty, csr))


if __name__ == '__main__':
    t0 = time.time()
    bench_triangulation()
    bench_numeric_jacobian()
    bench_sim3_log()
    bench_histogram()
    bench_sim3_threshold()
    bench_ransac_sampling()
    bench_gyro()
    bench_homography_sign()
    bench_pnp_behind()
    bench_lm_restart()
    bench_grid_memory()
    print('\n总耗时 %.1f s' % (time.time() - t0))
