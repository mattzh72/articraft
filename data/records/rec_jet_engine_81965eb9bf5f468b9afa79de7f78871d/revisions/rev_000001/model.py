from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    LatheGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _clamshell_bucket_mesh(sign: float) -> MeshGeometry:
    """Thin curved bucket with its rear hinge line at the part origin.

    The engine axis is local/global X.  A positive sign builds the upper bucket;
    a negative sign mirrors the lower bucket.  The shell is a closed annular
    sector whose aft crown is clipped to the hinge barrel at local (0, *, 0).
    """

    geom = MeshGeometry()
    length = 0.55
    rear_x = -0.035
    outer_radius = 0.58
    thickness = 0.035
    inner_radius = outer_radius - thickness
    half_width = 0.47
    nx = 11
    ny = 21

    def local_z(radius: float, y: float) -> float:
        return sign * (math.sqrt(max(radius * radius - y * y, 0.0)) - outer_radius)

    outer: list[list[int]] = []
    inner: list[list[int]] = []
    for ix in range(nx):
        t = ix / (nx - 1)
        x = -length + t * (rear_x + length)
        row_outer: list[int] = []
        row_inner: list[int] = []
        for iy in range(ny):
            u = iy / (ny - 1)
            y = -half_width + 2.0 * half_width * u
            # Very slight fore-aft flare gives the bucket a cast, non-flat look.
            flare = 0.010 * (1.0 - t)
            z_outer = local_z(outer_radius + flare, y)
            z_inner = local_z(inner_radius + flare, y)
            row_outer.append(geom.add_vertex(x, y, z_outer))
            row_inner.append(geom.add_vertex(x, y, z_inner))
        outer.append(row_outer)
        inner.append(row_inner)

    # Outer and inner skins.
    for ix in range(nx - 1):
        for iy in range(ny - 1):
            a = outer[ix][iy]
            b = outer[ix + 1][iy]
            c = outer[ix + 1][iy + 1]
            d = outer[ix][iy + 1]
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)

            ai = inner[ix][iy]
            bi = inner[ix][iy + 1]
            ci = inner[ix + 1][iy + 1]
            di = inner[ix + 1][iy]
            geom.add_face(ai, bi, ci)
            geom.add_face(ai, ci, di)

    # Close the side, forward, and rear lips.
    for ix in range(nx - 1):
        for iy in (0, ny - 1):
            a = outer[ix][iy]
            b = inner[ix][iy]
            c = inner[ix + 1][iy]
            d = outer[ix + 1][iy]
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)

    for ix in (0, nx - 1):
        for iy in range(ny - 1):
            a = outer[ix][iy]
            b = outer[ix][iy + 1]
            c = inner[ix][iy + 1]
            d = inner[ix][iy]
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="business_jet_turbofan")

    satin_grey = model.material("satin_grey", rgba=(0.63, 0.66, 0.68, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.08, 0.085, 0.09, 1.0))
    titanium = model.material("titanium", rgba=(0.42, 0.43, 0.44, 1.0))
    blade_black = model.material("blade_black", rgba=(0.025, 0.028, 0.032, 1.0))
    bucket_grey = model.material("bucket_grey", rgba=(0.52, 0.54, 0.55, 1.0))

    nacelle = model.part("nacelle")

    nacelle_shell = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.43, -1.00),
            (0.51, -0.91),
            (0.54, -0.25),
            (0.50, 0.38),
            (0.40, 0.84),
            (0.35, 1.00),
        ],
        inner_profile=[
            (0.31, -1.00),
            (0.36, -0.88),
            (0.38, -0.25),
            (0.34, 0.40),
            (0.26, 1.00),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    nacelle.visual(
        mesh_from_geometry(nacelle_shell, "nacelle_shell"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_grey,
        name="nacelle_shell",
    )

    core_body = LatheGeometry(
        [
            (0.0, -0.82),
            (0.075, -0.75),
            (0.155, -0.34),
            (0.170, 0.55),
            (0.110, 0.96),
            (0.0, 1.08),
        ],
        segments=56,
    )
    nacelle.visual(
        mesh_from_geometry(core_body, "core_body"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="core_body",
    )

    nacelle.visual(
        Cylinder(radius=0.036, length=0.74),
        origin=Origin(xyz=(-0.66, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=titanium,
        name="fan_shaft",
    )
    nacelle.visual(
        Cylinder(radius=0.082, length=0.080),
        origin=Origin(xyz=(-0.74, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=titanium,
        name="front_bearing",
    )

    for i in range(6):
        angle = i * math.tau / 6.0
        radius_mid = 0.275
        nacelle.visual(
            Box((0.060, 0.026, 0.340)),
            origin=Origin(
                xyz=(-0.48, radius_mid * math.sin(angle), radius_mid * math.cos(angle)),
                rpy=(-angle, 0.0, 0.0),
            ),
            material=titanium,
            name=f"stator_vane_{i}",
        )

    nacelle.visual(
        Box((0.62, 0.17, 0.22)),
        origin=Origin(xyz=(-0.10, 0.0, 0.62)),
        material=satin_grey,
        name="pylon_fairing",
    )

    for hinge_name, z, support_prefix in (
        ("upper_hinge_pin", 0.58, "upper_hinge"),
        ("lower_hinge_pin", -0.58, "lower_hinge"),
    ):
        nacelle.visual(
            Cylinder(radius=0.018, length=0.96),
            origin=Origin(xyz=(0.94, 0.0, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=titanium,
            name=hinge_name,
        )
        nacelle.visual(
            Box((0.080, 0.16, 0.16)),
            origin=Origin(xyz=(0.98, 0.0, z * 0.55)),
            material=titanium,
            name=f"{support_prefix}_root",
        )
        nacelle.visual(
            Box((0.080, 1.08, 0.080)),
            origin=Origin(xyz=(0.98, 0.0, z * 0.73)),
            material=titanium,
            name=f"{support_prefix}_crossbar",
        )
        for lug_i, y in enumerate((-0.52, 0.52)):
            nacelle.visual(
                Box((0.080, 0.10, 0.16)),
                origin=Origin(xyz=(0.96, y, z * 0.93)),
                material=titanium,
                name=f"{support_prefix}_lug_{lug_i}",
            )

    fan = model.part("fan")
    fan_rotor = FanRotorGeometry(
        outer_radius=0.335,
        hub_radius=0.090,
        blade_count=18,
        thickness=0.085,
        blade_pitch_deg=34.0,
        blade_sweep_deg=32.0,
        blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=14.0, camber=0.12),
        hub=FanRotorHub(style="spinner", bore_diameter=0.090),
    )
    fan.visual(
        mesh_from_geometry(fan_rotor, "fan_rotor"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blade_black,
        name="fan_rotor",
    )
    fan.visual(
        Cylinder(radius=0.050, length=0.080),
        origin=Origin(xyz=(-0.045, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=titanium,
        name="fan_bearing",
    )

    for part_name, sign in (("upper_bucket", 1.0), ("lower_bucket", -1.0)):
        bucket = model.part(part_name)
        bucket.visual(
            mesh_from_geometry(_clamshell_bucket_mesh(sign), f"{part_name}_shell"),
            material=bucket_grey,
            name="bucket_shell",
        )
        bucket.visual(
            Cylinder(radius=0.035, length=0.86),
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=titanium,
            name="bucket_hinge",
        )

    model.articulation(
        "fan_spin",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=fan,
        origin=Origin(xyz=(-0.82, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=120.0),
    )

    model.articulation(
        "upper_bucket_hinge",
        ArticulationType.REVOLUTE,
        parent=nacelle,
        child="upper_bucket",
        origin=Origin(xyz=(0.94, 0.0, 0.58)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=350.0, velocity=1.2, lower=0.0, upper=1.05),
    )
    model.articulation(
        "lower_bucket_hinge",
        ArticulationType.REVOLUTE,
        parent=nacelle,
        child="lower_bucket",
        origin=Origin(xyz=(0.94, 0.0, -0.58)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=350.0, velocity=1.2, lower=0.0, upper=1.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    nacelle = object_model.get_part("nacelle")
    fan = object_model.get_part("fan")
    upper_bucket = object_model.get_part("upper_bucket")
    lower_bucket = object_model.get_part("lower_bucket")
    fan_spin = object_model.get_articulation("fan_spin")
    upper_hinge = object_model.get_articulation("upper_bucket_hinge")
    lower_hinge = object_model.get_articulation("lower_bucket_hinge")

    ctx.allow_overlap(
        nacelle,
        upper_bucket,
        elem_a="upper_hinge_pin",
        elem_b="bucket_hinge",
        reason="The fixed hinge pin is intentionally captured inside the upper bucket hinge barrel.",
    )
    ctx.allow_overlap(
        nacelle,
        lower_bucket,
        elem_a="lower_hinge_pin",
        elem_b="bucket_hinge",
        reason="The fixed hinge pin is intentionally captured inside the lower bucket hinge barrel.",
    )
    ctx.allow_overlap(
        nacelle,
        fan,
        elem_a="fan_shaft",
        elem_b="fan_bearing",
        reason="The rotor bearing sleeve is intentionally captured on the fixed center shaft.",
    )

    ctx.expect_within(
        nacelle,
        upper_bucket,
        axes="xz",
        inner_elem="upper_hinge_pin",
        outer_elem="bucket_hinge",
        margin=0.002,
        name="upper pin is coaxial with bucket barrel",
    )
    ctx.expect_within(
        nacelle,
        lower_bucket,
        axes="xz",
        inner_elem="lower_hinge_pin",
        outer_elem="bucket_hinge",
        margin=0.002,
        name="lower pin is coaxial with bucket barrel",
    )
    ctx.expect_overlap(
        nacelle,
        upper_bucket,
        axes="y",
        elem_a="upper_hinge_pin",
        elem_b="bucket_hinge",
        min_overlap=0.70,
        name="upper pin spans the hinge barrel",
    )
    ctx.expect_overlap(
        nacelle,
        lower_bucket,
        axes="y",
        elem_a="lower_hinge_pin",
        elem_b="bucket_hinge",
        min_overlap=0.70,
        name="lower pin spans the hinge barrel",
    )
    ctx.expect_within(
        nacelle,
        fan,
        axes="yz",
        inner_elem="fan_shaft",
        outer_elem="fan_bearing",
        margin=0.001,
        name="fan bearing stays centered on shaft",
    )
    ctx.expect_overlap(
        nacelle,
        fan,
        axes="x",
        elem_a="fan_shaft",
        elem_b="fan_bearing",
        min_overlap=0.06,
        name="fan bearing retains shaft engagement",
    )

    ctx.expect_origin_distance(
        fan,
        nacelle,
        axes="yz",
        max_dist=0.001,
        name="fan rotor is on the engine centerline",
    )
    ctx.check(
        "fan spins about engine axis",
        tuple(round(v, 3) for v in fan_spin.axis) == (1.0, 0.0, 0.0),
        details=f"axis={fan_spin.axis}",
    )

    def _aabb_center_z(part):
        box = ctx.part_world_aabb(part)
        if box is None:
            return None
        return (box[0][2] + box[1][2]) * 0.5

    upper_origin_rest = ctx.part_world_position(upper_bucket)
    lower_origin_rest = ctx.part_world_position(lower_bucket)
    upper_center_rest = _aabb_center_z(upper_bucket)
    lower_center_rest = _aabb_center_z(lower_bucket)
    with ctx.pose({upper_hinge: 1.0, lower_hinge: 1.0}):
        upper_origin_open = ctx.part_world_position(upper_bucket)
        lower_origin_open = ctx.part_world_position(lower_bucket)
        upper_center_open = _aabb_center_z(upper_bucket)
        lower_center_open = _aabb_center_z(lower_bucket)

    ctx.check(
        "upper bucket stays clipped to rear hinge",
        upper_origin_rest is not None
        and upper_origin_open is not None
        and max(abs(a - b) for a, b in zip(upper_origin_rest, upper_origin_open)) < 1e-6,
        details=f"rest={upper_origin_rest}, open={upper_origin_open}",
    )
    ctx.check(
        "lower bucket stays clipped to rear hinge",
        lower_origin_rest is not None
        and lower_origin_open is not None
        and max(abs(a - b) for a, b in zip(lower_origin_rest, lower_origin_open)) < 1e-6,
        details=f"rest={lower_origin_rest}, open={lower_origin_open}",
    )
    ctx.check(
        "upper bucket deploys upward",
        upper_center_rest is not None and upper_center_open is not None and upper_center_open > upper_center_rest + 0.10,
        details=f"rest_z={upper_center_rest}, open_z={upper_center_open}",
    )
    ctx.check(
        "lower bucket deploys downward",
        lower_center_rest is not None and lower_center_open is not None and lower_center_open < lower_center_rest - 0.10,
        details=f"rest_z={lower_center_rest}, open_z={lower_center_open}",
    )

    return ctx.report()


object_model = build_object_model()
