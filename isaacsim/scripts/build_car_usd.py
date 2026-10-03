"""
VRX RH818 (1/8) 자율주행 개조 RC카 -> Isaac Sim 6.x 차량 에셋 USD 생성기
출력: autoRCcar_RH818.usda (지면/물리씬 없는 에셋 전용 파일, 씬에 Reference로 추가해 사용)
사용: Isaac Sim 설치 폴더에서
      ./python.sh build_car_usd.py [출력폴더]      (출력폴더 생략 시 현재 폴더)
      ※ isaacsim 모듈을 사용하므로 일반 python으로는 실행되지 않음
좌표계: Z-up, X-forward, Y-left, 단위 m / kg
base_link 원점 = 휠베이스 중앙, 차축 높이(지면 위 55mm)
"""
import math, os, sys

from isaacsim import SimulationApp

# PhysxSchema가 등록된 상태에서 USD를 만들기 위해 pxr import보다 먼저 생성
simulation_app = SimulationApp({
    "headless": True,
})

try:
    import numpy as np
    from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Gf, Vt

    OUT = sys.argv[1] if len(sys.argv) > 1 else "."
    os.makedirs(OUT, exist_ok=True)

    # ---------------- 주요 치수 (사용자 실측) ----------------
    WHEEL_R = 0.110 / 2
    WHEEL_W = 0.065
    WHEELBASE = 0.325
    TRACK_F = 0.270                  # 앞 트레드 (좌우 타이어 중심 간)
    TRACK_R = 0.260                  # 뒤 트레드
    HX = WHEELBASE / 2
    HY_F, HY_R = TRACK_F / 2, TRACK_R / 2
    def hy(sx):                      # sx>0: 전륜, sx<0: 후륜
        return HY_F if sx > 0 else HY_R
    MAX_STEER = 45.0

    # ---------------- 질량 (전체 3.8 kg 실측) ----------------
    MASS_TOTAL = 3.80
    MASS_WHEEL = 0.16                # 타이어+림 (추정)
    MASS_KNUCKLE = 0.08              # 조향 너클 (추정)
    MASS_BASE = MASS_TOTAL - 4 * MASS_WHEEL - 2 * MASS_KNUCKLE   # 3.00 kg

    def gz(z):  # 지면 기준 높이 -> base_link 프레임 z
        return z - WHEEL_R

    # ======================= 메시 빌더 =======================
    def rot(axis, deg):
        a = math.radians(deg); c, s = math.cos(a), math.sin(a)
        if axis == 'x': return np.array([[1,0,0],[0,c,-s],[0,s,c]])
        if axis == 'y': return np.array([[c,0,s],[0,1,0],[-s,0,c]])
        return np.array([[c,-s,0],[s,c,0],[0,0,1]])

    class MB:
        def __init__(self):
            self.P, self.C, self.I = [], [], []

        def _add(self, pts, faces, ref=None, ref_fn=None):
            pts = np.asarray(pts, float)
            o = len(self.P)
            self.P += [tuple(p) for p in pts]
            for f in faces:
                f = list(f)
                if ref is not None or ref_fn is not None:
                    q = pts[f]
                    cen = q.mean(0)
                    n = np.cross(q[1]-q[0], q[-1]-q[0])
                    r = ref if ref is not None else ref_fn(cen)
                    if np.dot(n, cen - r) < 0:
                        f = f[::-1]
                self.C.append(len(f)); self.I += [o+i for i in f]
            return self

        def box(self, c, size, R=None):
            c = np.asarray(c, float); h = np.asarray(size, float)/2
            R = np.eye(3) if R is None else R
            corners = [np.array([sx*h[0], sy*h[1], sz*h[2]]) for sx in (-1,1) for sy in (-1,1) for sz in (-1,1)]
            pts = [c + R @ p for p in corners]
            faces = [(0,1,3,2),(4,6,7,5),(0,4,5,1),(2,3,7,6),(0,2,6,4),(1,5,7,3)]
            return self._add(pts, faces, ref=c)

        def cyl(self, p0, p1, r, seg=24, r1=None):
            p0, p1 = np.asarray(p0,float), np.asarray(p1,float)
            r1 = r if r1 is None else r1
            ax = p1 - p0; ax /= np.linalg.norm(ax)
            t = np.array([1,0,0]) if abs(ax[0]) < 0.9 else np.array([0,1,0])
            u = np.cross(ax, t); u /= np.linalg.norm(u); v = np.cross(ax, u)
            pts = []
            for i in range(seg):
                a = 2*math.pi*i/seg; d = u*math.cos(a) + v*math.sin(a)
                pts += [p0 + r*d, p1 + r1*d]
            faces = [(2*i, 2*((i+1)%seg), 2*((i+1)%seg)+1, 2*i+1) for i in range(seg)]
            faces += [tuple(2*i for i in range(seg)), tuple(2*i+1 for i in range(seg))]
            return self._add(pts, faces, ref=(p0+p1)/2)

        def revolve_y(self, profile, inside, seg=48, center=(0,0,0)):
            """Y축 기준 회전체. profile=[(r,y)], inside=(r,y) 단면 내부점"""
            c = np.asarray(center, float); n = len(profile); pts = []
            for i in range(seg):
                a = 2*math.pi*i/seg
                for r, y in profile:
                    pts.append(c + np.array([r*math.cos(a), y, r*math.sin(a)]))
            faces = []
            for i in range(seg):
                j = (i+1) % seg
                for k in range(n-1):
                    faces.append((i*n+k, j*n+k, j*n+k+1, i*n+k+1))
            ri, yi = inside
            def ref_fn(cen):
                d = cen - c; ang = math.atan2(d[2], d[0])
                return c + np.array([ri*math.cos(ang), yi, ri*math.sin(ang)])
            return self._add(pts, faces, ref_fn=ref_fn)

        def extrude(self, poly, z0, z1):
            poly = [np.asarray(p, float) for p in poly]
            area = sum(poly[i][0]*poly[(i+1)%len(poly)][1] - poly[(i+1)%len(poly)][0]*poly[i][1] for i in range(len(poly)))
            if area < 0: poly = poly[::-1]
            n = len(poly)
            pts = [(p[0], p[1], z0) for p in poly] + [(p[0], p[1], z1) for p in poly]
            faces = [(i, (i+1)%n, n+(i+1)%n, n+i) for i in range(n)]
            faces += [tuple(range(n, 2*n)), tuple(range(n-1, -1, -1))]
            return self._add(pts, faces)

        def dome(self, c, r, seg=32, rings=10):
            c = np.asarray(c, float); pts = [c + np.array([0,0,r])]
            for k in range(1, rings+1):
                ph = (math.pi/2)*k/rings
                for i in range(seg):
                    a = 2*math.pi*i/seg
                    pts.append(c + r*np.array([math.sin(ph)*math.cos(a), math.sin(ph)*math.sin(a), math.cos(ph)]))
            faces = [(0, 1+i, 1+(i+1)%seg) for i in range(seg)]
            for k in range(rings-1):
                b0, b1 = 1+k*seg, 1+(k+1)*seg
                faces += [(b0+i, b1+i, b1+(i+1)%seg, b0+(i+1)%seg) for i in range(seg)]
            faces.append(tuple(1+(rings-1)*seg+i for i in range(seg)))
            return self._add(pts, faces, ref=c + np.array([0,0,r*0.3]))

        def torus_x(self, c, R, r, seg=32, sseg=10):
            """X축을 회전축으로 하는 토러스 (YZ 평면의 고리)"""
            c = np.asarray(c, float); pts = []
            for i in range(seg):
                a = 2*math.pi*i/seg; d = np.array([0, math.cos(a), math.sin(a)])
                for j in range(sseg):
                    b = 2*math.pi*j/sseg
                    pts.append(c + d*(R + r*math.cos(b)) + np.array([r*math.sin(b),0,0]))
            faces = []
            for i in range(seg):
                for j in range(sseg):
                    i2, j2 = (i+1)%seg, (j+1)%sseg
                    faces.append((i*sseg+j, i2*sseg+j, i2*sseg+j2, i*sseg+j2))
            def ref_fn(cen):
                d = cen - c; d[0] = 0; d /= np.linalg.norm(d)
                return c + d*R
            return self._add(pts, faces, ref_fn=ref_fn)

    # ======================= Stage =======================
    ASSET_PATH = os.path.join(OUT, "autoRCcar_RH818.usda")
    if os.path.exists(ASSET_PATH):
        os.remove(ASSET_PATH)
    stage = Usd.Stage.CreateNew(ASSET_PATH)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)

    root = UsdGeom.Xform.Define(stage, "/rc_car")
    stage.SetDefaultPrim(root.GetPrim())
    UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())
    rp = root.GetPrim()
    rp.AddAppliedSchema("PhysxArticulationAPI")
    rp.CreateAttribute("physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool).Set(False)
    rp.CreateAttribute("physxArticulation:solverPositionIterationCount", Sdf.ValueTypeNames.Int).Set(32)
    rp.CreateAttribute("physxArticulation:solverVelocityIterationCount", Sdf.ValueTypeNames.Int).Set(1)

    # ---------------- 재질 ----------------
    UsdGeom.Scope.Define(stage, "/rc_car/Looks")
    MATS, PREVIEW = {}, {}
    def material(name, rgb, rough=0.5, metal=0.0, emit=None, opacity=1.0):
        m = UsdShade.Material.Define(stage, f"/rc_car/Looks/{name}")
        sh = UsdShade.Shader.Define(stage, f"/rc_car/Looks/{name}/Shader")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metal)
        if opacity < 1: sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
        if emit: sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*emit))
        m.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        MATS[name] = m; PREVIEW[name] = rgb

    material("wood",        (0.86, 0.74, 0.55), 0.8)
    material("wood_cut",    (0.18, 0.14, 0.10), 0.9)
    material("plastic_blk", (0.04, 0.04, 0.045), 0.45)
    material("rubber",      (0.035, 0.035, 0.035), 0.95)
    material("rim_blk",     (0.02, 0.02, 0.025), 0.3)
    material("alu",         (0.62, 0.63, 0.65), 0.35, 0.9)
    material("alu_dark",    (0.35, 0.36, 0.38), 0.4, 0.8)
    material("dome",        (0.03, 0.08, 0.14), 0.05, 0.3)
    material("battery",     (0.30, 0.31, 0.33), 0.4, 0.6)
    material("strap",       (0.10, 0.10, 0.10), 0.9)
    material("pcb_blk",     (0.06, 0.07, 0.06), 0.5)
    material("pcb_red",     (0.65, 0.08, 0.08), 0.5)
    material("pcb_blue",    (0.10, 0.25, 0.70), 0.5)
    material("tape_red",    (0.80, 0.12, 0.12), 0.6)
    material("blue_anod",   (0.10, 0.20, 0.85), 0.3, 0.8)
    material("steel",       (0.75, 0.75, 0.77), 0.25, 1.0)
    material("white",       (0.92, 0.92, 0.90), 0.6)
    material("led",         (1.0, 0.75, 0.35), 0.5, 0.0, emit=(1.0, 0.55, 0.15))
    material("led_g",       (0.2, 0.9, 0.2), 0.3)
    material("led_r",       (0.9, 0.15, 0.1), 0.3)
    material("led_y",       (0.95, 0.85, 0.1), 0.3)

    PREVIEW_MESHES = []  # (body_world_offset, points, counts, indices, color)

    def mesh(parent, name, mb, mat, body_pos):
        m = UsdGeom.Mesh.Define(stage, f"{parent}/{name}")
        m.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in mb.P]))
        m.CreateFaceVertexCountsAttr(Vt.IntArray(mb.C))
        m.CreateFaceVertexIndicesAttr(Vt.IntArray(mb.I))
        m.CreateSubdivisionSchemeAttr("none")
        m.CreateDoubleSidedAttr(True)
        P = np.array(mb.P)
        m.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*P.min(0)), Gf.Vec3f(*P.max(0))]))
        UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(MATS[mat])
        PREVIEW_MESHES.append((np.asarray(body_pos), P, mb.C, mb.I, PREVIEW[mat]))
        return m

    def xf(path, pos, quat=None):
        x = UsdGeom.Xform.Define(stage, path)
        x.AddTranslateOp().Set(Gf.Vec3d(*pos))
        if quat is not None:
            x.AddOrientOp(UsdGeom.XformOp.PrecisionFloat).Set(quat)
        return x

    # ---------------- 충돌체 ----------------
    phys_mat = UsdShade.Material.Define(stage, "/rc_car/Looks/tire_physics")
    pm = UsdPhysics.MaterialAPI.Apply(phys_mat.GetPrim())
    pm.CreateStaticFrictionAttr(1.0); pm.CreateDynamicFrictionAttr(0.9); pm.CreateRestitutionAttr(0.0)
    chassis_pmat = UsdShade.Material.Define(stage, "/rc_car/Looks/chassis_physics")
    cm = UsdPhysics.MaterialAPI.Apply(chassis_pmat.GetPrim())
    cm.CreateStaticFrictionAttr(0.5); cm.CreateDynamicFrictionAttr(0.4); cm.CreateRestitutionAttr(0.0)

    def bind_phys(prim, mat):
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat, UsdShade.Tokens.weakerThanDescendants, "physics")

    def col_box(parent, name, center, size):
        c = UsdGeom.Cube.Define(stage, f"{parent}/{name}")
        c.CreateSizeAttr(1.0)
        c.AddTranslateOp().Set(Gf.Vec3d(*center))
        c.AddScaleOp().Set(Gf.Vec3f(*size))
        c.CreatePurposeAttr(UsdGeom.Tokens.guide)
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
        bind_phys(c.GetPrim(), chassis_pmat)

    # ---------------- 강체 ----------------
    def rigid_body(path, pos, mass, com=None, inertia=None):
        x = xf(path, pos)
        p = x.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(p)
        ma = UsdPhysics.MassAPI.Apply(p)
        ma.CreateMassAttr(mass)
        if com is not None: ma.CreateCenterOfMassAttr(Gf.Vec3f(*com))
        if inertia is not None: ma.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia))
        p.AddAppliedSchema("PhysxRigidBodyAPI")
        p.CreateAttribute("physxRigidBody:maxAngularVelocity", Sdf.ValueTypeNames.Float).Set(30000.0)  # deg/s
        return x

    BASE_POS = (0.0, 0.0, WHEEL_R)
    BASE = "/rc_car/base_link"
    rigid_body(BASE, BASE_POS, MASS_BASE, com=(-0.010, 0.0, 0.030))
    V = BASE + "/visuals"; C = BASE + "/collisions"; S = BASE + "/sensors"
    UsdGeom.Xform.Define(stage, V); UsdGeom.Xform.Define(stage, C); UsdGeom.Xform.Define(stage, S)

    def vis(name, mb, mat): mesh(V, name, mb, mat, BASE_POS)

    # ======================= 섀시 (RH818) =======================
    vis("chassis_tub", MB().box((0,0,gz(0.068)), (0.36,0.10,0.045)), "plastic_blk")
    vis("chassis_top", MB().box((-0.01,0,gz(0.093)), (0.30,0.13,0.006)), "plastic_blk")
    vis("led_strip", MB().box((0,0,gz(0.054)), (0.27,0.103,0.007)), "led")
    vis("esc_motor_heatsink", MB().box((-0.03,0.035,gz(0.103)), (0.055,0.035,0.022)), "blue_anod")
    vis("power_switch", MB().cyl((0.02,-0.051,gz(0.050)),(0.02,-0.056,gz(0.050)),0.006), "plastic_blk")
    vis("front_bumper", MB().box((0.197,0,gz(0.062)), (0.036,0.09,0.025)), "plastic_blk")
    vis("rear_bumper",  MB().box((-0.197,0,gz(0.058)), (0.036,0.06,0.02)), "plastic_blk")
    vis("rear_bumper_plate", MB().box((-0.214,0,gz(0.052)), (0.006,0.055,0.012)), "blue_anod")

    sus = MB()
    for sx in (1,-1):
        x = sx*HX
        sus.box((x - sx*0.035, 0, gz(0.098)), (0.008, 0.10, 0.045))       # 쇼크 타워
        for sy in (1,-1):
            sus.cyl((x, sy*0.045, gz(0.048)), (x, sy*(hy(sx)-0.035), gz(0.050)), 0.006, 10)   # 하부 암
            sus.cyl((x, sy*0.045, gz(0.070)), (x, sy*(hy(sx)-0.040), gz(0.066)), 0.003, 8)    # 상부 링크
    vis("suspension_arms", sus, "plastic_blk")
    shock, cap, spring = MB(), MB(), MB()
    for sx in (1,-1):
        for sy in (1,-1):
            top = np.array([sx*(HX-0.040), sy*0.040, gz(0.115)])
            bot = np.array([sx*(HX-0.010), sy*(hy(sx)-0.045), gz(0.052)])
            d = (bot-top)/np.linalg.norm(bot-top)
            shock.cyl(top, bot, 0.0045, 12)
            cap.cyl(top - d*0.004, top + d*0.012, 0.0075, 14)
            spring.cyl(top + d*0.014, bot - d*0.006, 0.0085, 14)
    vis("shock_bodies", shock, "steel")
    vis("shock_caps", cap, "blue_anod")
    vis("shock_springs", spring, "plastic_blk")
    shafts = MB()
    for sx in (1,-1):
        for sy in (1,-1):
            shafts.cyl((sx*HX, sy*0.05, 0), (sx*HX, sy*(hy(sx)-0.038), 0), 0.0035, 10)
    vis("drive_shafts", shafts, "steel")
    hubs = MB()
    for sy in (1,-1):   # 후륜 허브 캐리어 (전륜은 너클 바디에 있음)
        hubs.box((-HX, sy*(HY_R-0.040), 0), (0.022, 0.014, 0.032))
    vis("rear_hub_carriers", hubs, "plastic_blk")

    # ======================= 메인 목재 플레이트 =======================
    PL_Z0, PL_Z1 = gz(0.115), gz(0.118)
    def rounded_outline():
        ch = 0.010
        pts = [(0.202, 0.100-ch), (0.202-ch, 0.100), (0.150, 0.100), (0.140, 0.127-0.005),
               (0.140-0.006, 0.127), (-0.110, 0.127), (-0.121, 0.112), (-0.121, 0.105),
               (-0.203+ch, 0.105), (-0.203, 0.105-ch)]
        return pts + [(x, -y) for x, y in reversed(pts)]
    vis("main_plate", MB().extrude(rounded_outline(), PL_Z0, PL_Z1), "wood")

    cut = MB()
    for sy in (1,-1):   # 전면 삼각형 컷아웃
        cut.extrude([(0.185, sy*0.060), (0.160, sy*0.085), (0.160, sy*0.045)], PL_Z1, PL_Z1+0.0004)
        cut.extrude([(-0.170, sy*0.060), (-0.150, sy*0.085), (-0.150, sy*0.040)], PL_Z1, PL_Z1+0.0004)
        cut.box((0.0, sy*0.090, PL_Z1+0.0002), (0.10, 0.006, 0.0004))
    cut.box((0.110, 0, PL_Z1+0.0002), (0.030, 0.030, 0.0004))
    vis("main_plate_cutouts", cut, "wood_cut")
    tape = MB()
    tape.box((0.183, 0, PL_Z1+0.0006), (0.012, 0.072, 0.0012))
    tape.box((0.167, 0, PL_Z1+0.0006), (0.012, 0.072, 0.0012))
    vis("vhb_tape", tape, "tape_red")
    posts = MB()
    for sy in (1,-1):
        posts.cyl((0.175, sy*0.022, PL_Z1), (0.175, sy*0.022, gz(0.142)), 0.0035, 12)
    vis("body_posts", posts, "plastic_blk")

    # ======================= LiDAR 데크 =======================
    DK_Z0, DK_Z1 = gz(0.172), gz(0.175)
    DK_X0, DK_X1, DK_HY = -0.030, 0.140, 0.075
    vis("lidar_deck", MB().box(((DK_X0+DK_X1)/2, 0, (DK_Z0+DK_Z1)/2), (DK_X1-DK_X0, 2*DK_HY, 0.003)), "wood")
    slots = MB()
    for sy in (1,-1):
        for x in (0.115, -0.005):
            slots.box((x, sy*0.050, DK_Z1+0.0002), (0.006, 0.040, 0.0004))
            slots.box((x-0.012, sy*0.050, DK_Z1+0.0002), (0.006, 0.040, 0.0004))
    vis("lidar_deck_slots", slots, "wood_cut")
    so = MB()
    for x in (DK_X1-0.008, DK_X0+0.008):
        for sy in (1,-1):
            so.cyl((x, sy*(DK_HY-0.007), PL_Z1), (x, sy*(DK_HY-0.007), DK_Z0), 0.003, 10)
    vis("deck_standoffs", so, "plastic_blk")

    # ---------------- Livox MID-360 ----------------
    LX, LZ0 = 0.058, DK_Z1
    lb = MB().box((LX, 0, LZ0+0.016), (0.056, 0.056, 0.032))
    fins = MB()
    for k in range(7):
        o = -0.024 + k*0.008
        for s in (1,-1):
            fins.box((LX+s*0.0295, o, LZ0+0.016), (0.004, 0.0035, 0.030))
            fins.box((LX+o, s*0.0295, LZ0+0.016), (0.0035, 0.004, 0.030))
    lb.box((LX, 0, LZ0+0.001), (0.065, 0.065, 0.002))
    vis("mid360_body", lb, "alu")
    vis("mid360_fins", fins, "alu")
    vis("mid360_ring", MB().cyl((LX,0,LZ0+0.030),(LX,0,LZ0+0.038),0.031,40, r1=0.028), "alu_dark")
    vis("mid360_dome", MB().dome((LX,0,LZ0+0.038), 0.024, 40, 12), "dome")
    vis("mid360_connector", MB().cyl((LX-0.030,0,LZ0+0.014),(LX-0.048,0,LZ0+0.014),0.007,16), "steel")
    vis("mid360_cable", MB().cyl((LX-0.048,0,LZ0+0.014),(LX-0.075,0.01,LZ0+0.010),0.004,10), "plastic_blk")

    # ---------------- 카메라 (See3CAM_24CUG) ----------------
    CAM_X, CAM_Z = 0.128, gz(0.155)
    vis("camera_body", MB().box((CAM_X, 0, CAM_Z), (0.024, 0.040, 0.040)), "plastic_blk")
    vis("camera_lens", MB().cyl((CAM_X+0.012,0,CAM_Z),(CAM_X+0.026,0,CAM_Z),0.0095,24, r1=0.0085), "rim_blk")
    vis("camera_lens_glass", MB().cyl((CAM_X+0.026,0,CAM_Z),(CAM_X+0.0265,0,CAM_Z),0.006,24), "dome")

    # ---------------- Jetson Orin Nano Dev Kit ----------------
    JX = 0.050
    jet = MB()
    jet.box((JX, 0, gz(0.1295)), (0.100, 0.079, 0.003))
    vis("jetson_carrier", jet, "pcb_blk")
    vis("jetson_module_heatsink", MB().box((JX+0.005, 0, gz(0.141)), (0.070, 0.062, 0.020)), "alu_dark")
    vis("jetson_fan", MB().cyl((JX+0.005,0,gz(0.151)),(JX+0.005,0,gz(0.152)),0.018,24), "plastic_blk")
    ports = MB()
    for i in range(3):
        ports.box((JX-0.047, -0.025+i*0.018, gz(0.137)), (0.014, 0.014, 0.012))
    vis("jetson_ports", ports, "steel")
    jso = MB()
    for x in (JX-0.043, JX+0.043):
        for sy in (1,-1):
            jso.cyl((x, sy*0.033, PL_Z1), (x, sy*0.033, gz(0.128)), 0.0025, 8)
    vis("jetson_standoffs", jso, "white")
    vis("usb_hub", MB().box((0.03, 0.055, gz(0.165)), (0.090, 0.028, 0.012)), "alu")

    # ---------------- ESP32 / 전원보드 ----------------
    vis("esp32_perfboard", MB().box((0.000, -0.100, gz(0.1195)), (0.080, 0.050, 0.0016)), "pcb_blk")
    vis("esp32_devkit", MB().box((0.008, -0.098, gz(0.127)), (0.052, 0.028, 0.012)), "pcb_blk")
    vis("esp32_shield", MB().box((0.000, -0.098, gz(0.1335)), (0.018, 0.016, 0.003)), "steel")
    for i, m in enumerate(("led_y", "led_r", "led_g")):
        vis(f"status_led_{i}", MB().cyl((-0.030+0.0, -0.115+i*0.008, gz(0.120)), (-0.030, -0.115+i*0.008, gz(0.128)), 0.0025, 10), m)
    vis("power_meter_board", MB().box((0.005, 0.112, gz(0.1195)), (0.050, 0.022, 0.003)), "pcb_blue")

    # ---------------- 배터리 + GPS-16344 ----------------
    BX = -0.085
    bpl = MB().box((BX, 0, gz(0.1365)), (0.085, 0.160, 0.003))
    vis("battery_plate", bpl, "wood")
    bso = MB()
    for x in (BX-0.035, BX+0.035):
        for sy in (1,-1):
            bso.cyl((x, sy*0.070, PL_Z1), (x, sy*0.070, gz(0.135)), 0.0035, 10)
    vis("battery_standoffs", bso, "white")
    vis("gps16344_board", MB().box((BX, 0, gz(0.1195)), (0.043, 0.043, 0.0016)), "pcb_red")
    vis("gps16344_module", MB().box((BX, 0, gz(0.1225)), (0.022, 0.017, 0.0045)), "steel")
    vis("battery_ctb21cap", MB().box((BX, 0, gz(0.156)), (0.074, 0.150, 0.036)), "battery")
    straps = MB()
    for sy in (0.042, -0.042):
        straps.box((BX, sy, gz(0.156)), (0.0765, 0.018, 0.0385))
    vis("battery_straps", straps, "strap")

    # ---------------- 리어 윙 + GNSS 안테나 ----------------
    wing = MB()
    wing.box((-0.233, 0, gz(0.145)), (0.056, 0.190, 0.003), rot('y', -18))
    for sy in (1,-1):
        wing.box((-0.233, sy*0.095, gz(0.140)), (0.056, 0.003, 0.045), rot('y', -18))
        wing.cyl((-0.205, sy*0.025, gz(0.090)), (-0.225, sy*0.025, gz(0.138)), 0.003, 8)
    vis("rear_wing", wing, "plastic_blk")
    ANT_X, ANT_Y = -0.207, 0.010
    vis("gnss_mast", MB().cyl((ANT_X, ANT_Y, PL_Z1), (ANT_X, ANT_Y, gz(0.222)), 0.005, 12), "plastic_blk")
    ant = MB()
    ant.box((ANT_X, ANT_Y, gz(0.231)), (0.062, 0.062, 0.018))
    ant.box((ANT_X, ANT_Y, gz(0.224)), (0.088, 0.020, 0.004))
    vis("gnss_antenna", ant, "plastic_blk")
    coil = MB()
    for k in range(3):
        coil.torus_x((ANT_X+0.004*k-0.004, ANT_Y+0.005, gz(0.170)), 0.033 - 0.002*k, 0.003, 32, 8)
    vis("gnss_cable_coil", coil, "plastic_blk")

    # ---------------- base 충돌체 ----------------
    col_box(C, "chassis", (0, 0, gz(0.068)), (0.40, 0.10, 0.046))
    col_box(C, "main_plate", (0, 0, (PL_Z0+PL_Z1)/2), (0.405, 0.20, 0.004))
    col_box(C, "electronics", (0.020, 0, gz(0.146)), (0.24, 0.15, 0.056))
    col_box(C, "lidar", (LX, 0, LZ0+0.030), (0.065, 0.065, 0.060))
    col_box(C, "gnss", (ANT_X, ANT_Y, gz(0.200)), (0.07, 0.07, 0.08))
    col_box(C, "wing", (-0.233, 0, gz(0.140)), (0.06, 0.19, 0.05))

    # ---------------- 센서 프레임 ----------------
    def quat_from_rows(rows):
        r = rows
        m = Gf.Matrix4d(r[0][0], r[0][1], r[0][2], 0, r[1][0], r[1][1], r[1][2], 0,
                        r[2][0], r[2][1], r[2][2], 0, 0, 0, 0, 1)
        q = m.ExtractRotationQuat()
        return Gf.Quatf(q.GetReal(), Gf.Vec3f(q.GetImaginary()))

    xf(S + "/lidar_link", (LX, 0.0, LZ0 + 0.047))           # MID-360 측정 원점(근사)
    xf(S + "/imu_link", (BX, 0.0, gz(0.1225)))              # ZED-F9R 내장 IMU
    xf(S + "/gnss_antenna_link", (ANT_X, ANT_Y, gz(0.240)))
    xf(S + "/camera_link", (CAM_X + 0.026, 0.0, CAM_Z))     # X-forward 프레임
    # USD 카메라는 -Z를 바라보고 +Y가 위 -> 차량 +X 방향으로 회전
    cam_q = quat_from_rows([(0, -1, 0), (0, 0, 1), (-1, 0, 0)])
    cam = UsdGeom.Camera.Define(stage, S + "/camera_link/camera_optical")
    cam.AddOrientOp(UsdGeom.XformOp.PrecisionFloat).Set(cam_q)
    cam.CreateFocalLengthAttr(2.8)             # 렌즈 초점거리(mm) - 실제 렌즈 사양으로 수정
    cam.CreateHorizontalApertureAttr(5.76)     # AR0234: 1920 x 3.0um
    cam.CreateVerticalApertureAttr(3.60)       # 1200 x 3.0um
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 100.0))

    # ======================= 바퀴 / 조향 =======================
    def tire_mesh(s):
        mb = MB()
        prof = [(0.036, -0.029), (0.044, -0.0322), (0.049, -0.0310), (0.0512, -0.027),
                (0.0515, -0.010), (0.0515, 0.010), (0.0512, 0.027), (0.049, 0.0310),
                (0.044, 0.0322), (0.036, 0.029)]
        mb.revolve_y(prof, inside=(0.045, 0.0), seg=56)
        N = 22
        for row, (y0, y1, off) in enumerate([(-0.030, -0.013, 0.0), (-0.011, 0.011, 0.5), (0.013, 0.030, 0.0)]):
            for i in range(N):
                a = 2*math.pi*(i + off)/N
                for sub in (0, 1):   # 블록을 2조각으로 -> 러그 패턴 느낌
                    if row == 1 and sub == 1: continue
                    w_ang = 2*math.pi/N*0.55
                    yy0, yy1 = (y0, (y0+y1)/2 - 0.001) if sub == 0 else ((y0+y1)/2 + 0.001, y1)
                    if row == 1: yy0, yy1 = y0, y1
                    ac = a + (sub*0.25 - 0.1)*2*math.pi/N
                    R = np.array([[math.cos(ac), 0, -math.sin(ac)], [0,1,0], [math.sin(ac), 0, math.cos(ac)]])
                    cen = np.array([0.0533*math.cos(ac), (yy0+yy1)/2, 0.0533*math.sin(ac)])
                    mb.box(cen, (0.0045, yy1-yy0, 0.0515*w_ang*(0.9 if row==1 else 1.0)), R)
        return mb

    def rim_mesh(s):
        mb = MB()
        face_y = s*0.025
        mb.revolve_y([(0.036, -0.028), (0.036, 0.028)], inside=(0.030, 0.0), seg=40)            # 림 배럴
        mb.revolve_y([(0.029, face_y), (0.036, face_y), (0.036, face_y + s*0.004), (0.029, face_y + s*0.004)],
                     inside=(0.0325, face_y + s*0.002), seg=40)                                 # 림 립
        for k in range(5):
            a = 2*math.pi*k/5
            R = np.array([[math.cos(a), 0, -math.sin(a)], [0,1,0], [math.sin(a), 0, math.cos(a)]])
            cen = np.array([0.020*math.cos(a), face_y + s*0.001, 0.020*math.sin(a)])
            mb.box(cen, (0.022, 0.005, 0.008), R)                                                # 스포크
        mb.cyl((0, face_y - s*0.020, 0), (0, face_y + s*0.004, 0), 0.010, 20)                    # 허브
        mb.cyl((0, -s*0.010, 0), (0, -s*0.009, 0), 0.030, 30)                                    # 안쪽 디스크
        return mb

    bodies_for_filter = []
    joint_scope = "/rc_car/joints"
    UsdGeom.Scope.Define(stage, joint_scope)

    def revolute(name, body0, body1, pos0, axis, lower=None, upper=None):
        j = UsdPhysics.RevoluteJoint.Define(stage, f"{joint_scope}/{name}")
        j.CreateAxisAttr(axis)
        j.CreateBody0Rel().SetTargets([body0]); j.CreateBody1Rel().SetTargets([body1])
        j.CreateLocalPos0Attr(Gf.Vec3f(*pos0)); j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0)); j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        if lower is not None:
            j.CreateLowerLimitAttr(lower); j.CreateUpperLimitAttr(upper)
        p = j.GetPrim()
        p.AddAppliedSchema("PhysxJointAPI")
        p.CreateAttribute("physxJoint:maxJointVelocity", Sdf.ValueTypeNames.Float).Set(30000.0)
        return j

    def drive(joint, stiffness, damping, max_force):
        d = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(stiffness); d.CreateDampingAttr(damping)
        d.CreateMaxForceAttr(max_force)
        d.CreateTargetPositionAttr(0.0); d.CreateTargetVelocityAttr(0.0)

    wheel_I = (MASS_WHEEL*(3*WHEEL_R**2 + WHEEL_W**2)/12, MASS_WHEEL*WHEEL_R**2/2, MASS_WHEEL*(3*WHEEL_R**2 + WHEEL_W**2)/12)

    for tag, sx, sy in (("front_left", 1, 1), ("front_right", 1, -1), ("rear_left", -1, 1), ("rear_right", -1, -1)):
        wpos = (sx*HX, sy*hy(sx), WHEEL_R)
        wheel = f"/rc_car/{tag}_wheel"
        rigid_body(wheel, wpos, MASS_WHEEL, com=(0, 0, 0), inertia=wheel_I)
        mesh(wheel, "tire", tire_mesh(sy), "rubber", wpos)
        mesh(wheel, "rim", rim_mesh(sy), "rim_blk", wpos)
        # 충돌체: 32각 원통 메시(정점 64개) + convexHull 근사 (customGeometry 미사용)
        cmb = MB().cyl((0, -WHEEL_W/2, 0), (0, WHEEL_W/2, 0), WHEEL_R, 32)
        col = UsdGeom.Mesh.Define(stage, f"{wheel}/collision")
        col.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in cmb.P]))
        col.CreateFaceVertexCountsAttr(Vt.IntArray(cmb.C))
        col.CreateFaceVertexIndicesAttr(Vt.IntArray(cmb.I))
        Pc = np.array(cmb.P)
        col.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*Pc.min(0)), Gf.Vec3f(*Pc.max(0))]))
        col.CreatePurposeAttr(UsdGeom.Tokens.guide)
        cp = col.GetPrim()
        UsdPhysics.CollisionAPI.Apply(cp)
        UsdPhysics.MeshCollisionAPI.Apply(cp).CreateApproximationAttr("convexHull")
        bind_phys(cp, phys_mat)
        bodies_for_filter.append(wheel)

        if sx > 0:   # 전륜: 조향 너클
            kn = f"/rc_car/{tag}_steer_knuckle"
            rigid_body(kn, wpos, MASS_KNUCKLE, com=(0, 0, 0), inertia=(1e-4, 1e-4, 1e-4))
            mesh(kn, "knuckle", MB().box((0, -sy*0.040, 0), (0.022, 0.014, 0.032)), "plastic_blk", wpos)
            bodies_for_filter.append(kn)
            sj = revolute(f"{tag}_steer_joint", BASE, kn, (sx*HX, sy*hy(sx), 0), "Z", -MAX_STEER, MAX_STEER)
            drive(sj, stiffness=1.0, damping=0.05, max_force=3.0)     # 서보: 약 3 N·m
            wj = revolute(f"{tag}_wheel_joint", kn, wheel, (0, 0, 0), "Y")
        else:
            wj = revolute(f"{tag}_wheel_joint", BASE, wheel, (sx*HX, sy*hy(sx), 0), "Y")
        drive(wj, stiffness=0.0, damping=0.05, max_force=4.0)        # 속도 제어

    fp = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(BASE))
    fp.CreateFilteredPairsRel().SetTargets([Sdf.Path(b) for b in bodies_for_filter])

    stage.GetRootLayer().Save()

    print(f"Created asset: {ASSET_PATH}")
    print(
        f"Total mass: {MASS_TOTAL:.3f} kg "
        f"(base={MASS_BASE:.3f}, wheels={4*MASS_WHEEL:.3f}, "
        f"knuckles={2*MASS_KNUCKLE:.3f})"
    )
finally:
    # 정상 종료/예외 발생 모두 헤드리스 Isaac Sim 앱을 확실히 종료
    simulation_app.close()
