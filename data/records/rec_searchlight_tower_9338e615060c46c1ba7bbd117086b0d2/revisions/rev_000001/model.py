from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="searchlight_tower")

    galvanized = model.material("galvanized", rgba=(0.66, 0.68, 0.70, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.20, 0.23, 1.0))
    lamp_paint = model.material("lamp_paint", rgba=(0.78, 0.80, 0.76, 1.0))
    glass = model.material("glass", rgba=(0.74, 0.84, 0.92, 0.42))
    matte_black = model.material("matte_black", rgba=(0.08, 0.09, 0.10, 1.0))
    warm_metal = model.material("warm_metal", rgba=(0.58, 0.55, 0.50, 1.0))

    tower_base = model.part("tower_base")
    tower_base.visual(
        Box((0.72, 0.72, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=dark_steel,
        name="base_plinth",
    )

    foot_offsets = [(-0.24, -0.24), (-0.24, 0.24), (0.24, -0.24), (0.24, 0.24)]
    for index, (x, y) in enumerate(foot_offsets):
        tower_base.visual(
            Box((0.14, 0.14, 0.05)),
            origin=Origin(xyz=(x, y, 0.065)),
            material=galvanized,
            name=f"foot_{index:02d}",
        )

    mast_half = 0.11
    levels = [0.10, 0.44, 0.78, 1.12, 1.38]
    corners = [
        (mast_half, mast_half),
        (mast_half, -mast_half),
        (-mast_half, -mast_half),
        (-mast_half, mast_half),
    ]

    for x, y in corners:
        _add_member(
            tower_base,
            (x, y, levels[0]),
            (x, y, levels[-1]),
            radius=0.014,
            material=galvanized,
        )

    for z in levels:
        for x, y in corners:
            tower_base.visual(
                Sphere(radius=0.022),
                origin=Origin(xyz=(x, y, z)),
                material=galvanized,
            )
        for i in range(4):
            x0, y0 = corners[i]
            x1, y1 = corners[(i + 1) % 4]
            _add_member(
                tower_base,
                (x0, y0, z),
                (x1, y1, z),
                radius=0.011,
                material=galvanized,
            )

    for i in range(len(levels) - 1):
        z0 = levels[i]
        z1 = levels[i + 1]
        for j in range(4):
            x0, y0 = corners[j]
            x1, y1 = corners[(j + 1) % 4]
            _add_member(
                tower_base,
                (x0, y0, z0),
                (x1, y1, z1),
                radius=0.0085,
                material=galvanized,
            )
            _add_member(
                tower_base,
                (x1, y1, z0),
                (x0, y0, z1),
                radius=0.0085,
                material=galvanized,
            )

    anchor_nodes = [
        ((0.18, 0.18, 0.09), (mast_half, mast_half, levels[0])),
        ((0.18, -0.18, 0.09), (mast_half, -mast_half, levels[0])),
        ((-0.18, -0.18, 0.09), (-mast_half, -mast_half, levels[0])),
        ((-0.18, 0.18, 0.09), (-mast_half, mast_half, levels[0])),
    ]
    for a, b in anchor_nodes:
        _add_member(
            tower_base,
            a,
            b,
            radius=0.012,
            material=galvanized,
        )

    tower_base.visual(
        Cylinder(radius=0.12, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 1.42)),
        material=dark_steel,
        name="top_mount_collar",
    )
    tower_base.visual(
        Cylinder(radius=0.055, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, 1.45)),
        material=dark_steel,
        name="kingpin_stub",
    )
    tower_base.inertial = Inertial.from_geometry(
        Box((0.72, 0.72, 1.49)),
        mass=38.0,
        origin=Origin(xyz=(0.0, 0.0, 0.745)),
    )

    pan_yoke = model.part("pan_yoke")
    pan_yoke.visual(
        Cylinder(radius=0.135, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=dark_steel,
        name="slew_ring",
    )
    pan_yoke.visual(
        Cylinder(radius=0.09, length=0.07),
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
        material=matte_black,
        name="rotary_drum",
    )
    pan_yoke.visual(
        Cylinder(radius=0.018, length=0.062),
        origin=Origin(xyz=(0.0, 0.0, 0.101)),
        material=dark_steel,
        name="center_post",
    )
    pan_yoke.visual(
        Cylinder(radius=0.018, length=0.21),
        origin=Origin(xyz=(0.105, 0.108, 0.122)),
        material=dark_steel,
        name="left_yoke_arm",
    )
    pan_yoke.visual(
        Cylinder(radius=0.018, length=0.21),
        origin=Origin(xyz=(0.105, -0.108, 0.122)),
        material=dark_steel,
        name="right_yoke_arm",
    )
    pan_yoke.visual(
        Cylinder(radius=0.024, length=0.034),
        origin=Origin(xyz=(0.105, 0.093, 0.122), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="left_bearing_housing",
    )
    pan_yoke.visual(
        Cylinder(radius=0.024, length=0.034),
        origin=Origin(xyz=(0.105, -0.093, 0.122), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="right_bearing_housing",
    )
    pan_yoke.visual(
        Cylinder(radius=0.014, length=0.228),
        origin=Origin(xyz=(0.082, 0.0, 0.034), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="lower_crossbeam",
    )
    pan_yoke.visual(
        Cylinder(radius=0.013, length=0.228),
        origin=Origin(xyz=(0.105, 0.0, 0.210), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="upper_crossbeam",
    )
    _add_member(
        pan_yoke,
        (0.0, 0.060, 0.068),
        (0.105, 0.090, 0.060),
        radius=0.012,
        material=dark_steel,
        name="left_riser",
    )
    _add_member(
        pan_yoke,
        (0.0, -0.060, 0.068),
        (0.105, -0.090, 0.060),
        radius=0.012,
        material=dark_steel,
        name="right_riser",
    )
    _add_member(
        pan_yoke,
        (0.0, 0.0, 0.132),
        (0.105, 0.0, 0.210),
        radius=0.010,
        material=dark_steel,
        name="backbone_brace",
    )
    pan_yoke.inertial = Inertial.from_geometry(
        Box((0.35, 0.27, 0.24)),
        mass=7.0,
        origin=Origin(xyz=(0.05, 0.0, 0.11)),
    )

    hood_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile=[
                (0.066, 0.0),
                (0.074, 0.035),
                (0.080, 0.075),
                (0.083, 0.110),
            ],
            inner_profile=[
                (0.058, 0.008),
                (0.065, 0.038),
                (0.070, 0.076),
                (0.073, 0.106),
            ],
            segments=48,
        ),
        "searchlight_hood",
    )
    handle_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.045, 0.0, 0.060),
                (0.070, 0.0, 0.102),
                (0.120, 0.0, 0.122),
                (0.170, 0.0, 0.102),
                (0.195, 0.0, 0.060),
            ],
            radius=0.0055,
            samples_per_segment=16,
            radial_segments=16,
            cap_ends=True,
            up_hint=(0.0, 1.0, 0.0),
        ),
        "searchlight_handle",
    )

    lamp_head = model.part("lamp_head")
    lamp_head.visual(
        Cylinder(radius=0.072, length=0.24),
        origin=Origin(xyz=(0.135, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lamp_paint,
        name="main_barrel",
    )
    lamp_head.visual(
        Cylinder(radius=0.052, length=0.052),
        origin=Origin(xyz=(0.018, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="rear_housing",
    )
    lamp_head.visual(
        Box((0.09, 0.10, 0.055)),
        origin=Origin(xyz=(0.045, 0.0, -0.042)),
        material=dark_steel,
        name="gearbox",
    )
    lamp_head.visual(
        Cylinder(radius=0.016, length=0.188),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=warm_metal,
        name="trunnion_shaft",
    )
    lamp_head.visual(
        Cylinder(radius=0.060, length=0.014),
        origin=Origin(xyz=(0.248, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_metal,
        name="front_bezel",
    )
    lamp_head.visual(
        Cylinder(radius=0.057, length=0.012),
        origin=Origin(xyz=(0.256, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="front_lens",
    )
    lamp_head.visual(
        hood_mesh,
        origin=Origin(xyz=(0.175, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lamp_paint,
        name="hood_shell",
    )
    lamp_head.visual(
        handle_mesh,
        material=matte_black,
        name="carry_handle",
    )
    lamp_head.inertial = Inertial.from_geometry(
        Box((0.38, 0.18, 0.19)),
        mass=6.5,
        origin=Origin(xyz=(0.13, 0.0, 0.0)),
    )

    model.articulation(
        "mast_pan",
        ArticulationType.REVOLUTE,
        parent=tower_base,
        child=pan_yoke,
        origin=Origin(xyz=(0.0, 0.0, 1.46)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.8,
            lower=-2.6,
            upper=2.6,
        ),
    )
    model.articulation(
        "yoke_tilt",
        ArticulationType.REVOLUTE,
        parent=pan_yoke,
        child=lamp_head,
        origin=Origin(xyz=(0.105, 0.0, 0.122)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=55.0,
            velocity=0.8,
            lower=-0.30,
            upper=0.82,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower_base = object_model.get_part("tower_base")
    pan_yoke = object_model.get_part("pan_yoke")
    lamp_head = object_model.get_part("lamp_head")
    mast_pan = object_model.get_articulation("mast_pan")
    yoke_tilt = object_model.get_articulation("yoke_tilt")

    ctx.expect_gap(
        pan_yoke,
        tower_base,
        axis="z",
        positive_elem="slew_ring",
        negative_elem="top_mount_collar",
        max_gap=0.001,
        max_penetration=0.0005,
        name="slew ring seats on top collar",
    )
    ctx.allow_overlap(
        pan_yoke,
        lamp_head,
        elem_a="left_bearing_housing",
        elem_b="trunnion_shaft",
        reason="The trunnion shaft is intentionally represented as passing through the left bearing housing rather than a drilled bore.",
    )
    ctx.allow_overlap(
        pan_yoke,
        lamp_head,
        elem_a="right_bearing_housing",
        elem_b="trunnion_shaft",
        reason="The trunnion shaft is intentionally represented as passing through the right bearing housing rather than a drilled bore.",
    )

    def _center_z(aabb) -> float:
        return (aabb[0][2] + aabb[1][2]) * 0.5

    ctx.expect_overlap(
        lamp_head,
        pan_yoke,
        axes="xz",
        elem_a="trunnion_shaft",
        elem_b="left_bearing_housing",
        min_overlap=0.020,
        name="left trunnion stays aligned with the left bearing housing",
    )
    ctx.expect_overlap(
        lamp_head,
        pan_yoke,
        axes="xz",
        elem_a="trunnion_shaft",
        elem_b="right_bearing_housing",
        min_overlap=0.020,
        name="right trunnion stays aligned with the right bearing housing",
    )
    ctx.expect_overlap(
        lamp_head,
        pan_yoke,
        axes="y",
        elem_a="trunnion_shaft",
        elem_b="left_bearing_housing",
        min_overlap=0.012,
        name="left trunnion remains inserted in the left bearing housing",
    )
    ctx.expect_overlap(
        lamp_head,
        pan_yoke,
        axes="y",
        elem_a="trunnion_shaft",
        elem_b="right_bearing_housing",
        min_overlap=0.012,
        name="right trunnion remains inserted in the right bearing housing",
    )

    rest_lens_aabb = ctx.part_element_world_aabb(lamp_head, elem="front_lens")
    def _center_y(aabb) -> float:
        return (aabb[0][1] + aabb[1][1]) * 0.5

    with ctx.pose({mast_pan: 0.8}):
        panned_lens_aabb = ctx.part_element_world_aabb(lamp_head, elem="front_lens")
    ctx.check(
        "pan joint swings lamp nose toward +Y",
        rest_lens_aabb is not None
        and panned_lens_aabb is not None
        and _center_y(panned_lens_aabb) > _center_y(rest_lens_aabb) + 0.12,
        details=f"rest={rest_lens_aabb}, panned={panned_lens_aabb}",
    )

    tilt_upper = yoke_tilt.motion_limits.upper if yoke_tilt.motion_limits else None
    with ctx.pose({yoke_tilt: tilt_upper if tilt_upper is not None else 0.0}):
        tilted_lens_aabb = ctx.part_element_world_aabb(lamp_head, elem="front_lens")
    ctx.check(
        "tilt joint raises the beam axis",
        rest_lens_aabb is not None
        and tilted_lens_aabb is not None
        and _center_z(tilted_lens_aabb) > _center_z(rest_lens_aabb) + 0.09,
        details=f"rest={rest_lens_aabb}, tilted={tilted_lens_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
