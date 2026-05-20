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


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _shade_shell_mesh():
    profile = [
        (0.0, 0.0),
        (0.022, -0.010),
        (0.056, -0.036),
        (0.104, -0.112),
        (0.129, -0.192),
        (0.118, -0.256),
        (0.080, -0.282),
        (0.076, -0.282),
        (0.111, -0.252),
        (0.122, -0.190),
        (0.098, -0.112),
        (0.050, -0.038),
        (0.018, -0.012),
        (0.0, -0.004),
    ]
    return _mesh("arc_lamp_shade_shell", LatheGeometry(profile, segments=56))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="arc_floor_lamp")

    marble = model.material("marble", rgba=(0.88, 0.89, 0.90, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.73, 0.75, 0.77, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.17, 0.18, 0.20, 1.0))
    painted_black = model.material("painted_black", rgba=(0.11, 0.11, 0.12, 1.0))
    warm_white = model.material("warm_white", rgba=(0.96, 0.95, 0.89, 1.0))
    warm_bulb = model.material("warm_bulb", rgba=(0.98, 0.90, 0.68, 0.85))

    base = model.part("base")
    base.visual(
        Box((0.46, 0.30, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=marble,
        name="marble_base",
    )
    base.visual(
        Box((0.14, 0.10, 0.016)),
        origin=Origin(xyz=(-0.14, 0.0, 0.088)),
        material=satin_steel,
        name="mount_plinth",
    )
    base.visual(
        Box((0.056, 0.090, 0.044)),
        origin=Origin(xyz=(-0.14, 0.0, 0.118)),
        material=dark_metal,
        name="mount_tower",
    )
    base.visual(
        Box((0.036, 0.008, 0.060)),
        origin=Origin(xyz=(-0.14, 0.018, 0.160)),
        material=dark_metal,
        name="left_yoke_cheek",
    )
    base.visual(
        Box((0.036, 0.008, 0.060)),
        origin=Origin(xyz=(-0.14, -0.018, 0.160)),
        material=dark_metal,
        name="right_yoke_cheek",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.46, 0.30, 0.20)),
        mass=58.0,
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
    )

    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=0.014, length=0.026),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="pivot_barrel",
    )
    arm.visual(
        Cylinder(radius=0.018, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=dark_metal,
        name="pivot_collar",
    )
    arm.visual(
        _mesh(
            "arc_lamp_post_tube",
            tube_from_spline_points(
                [
                    (0.0, 0.0, 0.045),
                    (0.10, 0.0, 0.26),
                    (0.33, 0.0, 0.96),
                    (0.72, 0.0, 1.55),
                    (1.10, 0.0, 1.80),
                    (1.24, 0.0, 1.76),
                ],
                radius=0.017,
                samples_per_segment=18,
                radial_segments=20,
                cap_ends=True,
            ),
        ),
        material=painted_black,
        name="arc_post",
    )
    arm.visual(
        Cylinder(radius=0.014, length=0.100),
        origin=Origin(xyz=(1.235, 0.0, 1.700)),
        material=dark_metal,
        name="tip_drop",
    )
    arm.visual(
        Box((0.050, 0.040, 0.018)),
        origin=Origin(xyz=(1.225, 0.0, 1.648)),
        material=dark_metal,
        name="shade_bracket_bridge",
    )
    arm.visual(
        Box((0.030, 0.008, 0.035)),
        origin=Origin(xyz=(1.235, 0.016, 1.630)),
        material=dark_metal,
        name="shade_bracket_left",
    )
    arm.visual(
        Box((0.030, 0.008, 0.035)),
        origin=Origin(xyz=(1.235, -0.016, 1.630)),
        material=dark_metal,
        name="shade_bracket_right",
    )
    arm.inertial = Inertial.from_geometry(
        Box((1.28, 0.08, 1.84)),
        mass=11.5,
        origin=Origin(xyz=(0.64, 0.0, 0.92)),
    )

    shade = model.part("shade")
    shade.visual(
        Cylinder(radius=0.007, length=0.022),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="shade_hinge_barrel",
    )
    shade.visual(
        Box((0.020, 0.020, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material=dark_metal,
        name="shade_hanger_block",
    )
    shade.visual(
        Cylinder(radius=0.012, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, -0.042)),
        material=dark_metal,
        name="shade_stem",
    )
    shade.visual(
        Cylinder(radius=0.030, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.070)),
        material=dark_metal,
        name="shade_cap",
    )
    shade.visual(
        _shade_shell_mesh(),
        origin=Origin(xyz=(0.0, 0.0, -0.075)),
        material=warm_white,
        name="shade_shell",
    )
    shade.visual(
        Cylinder(radius=0.015, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, -0.109)),
        material=dark_metal,
        name="lamp_socket",
    )
    shade.visual(
        Sphere(radius=0.034),
        origin=Origin(xyz=(0.0, 0.0, -0.158)),
        material=warm_bulb,
        name="bulb",
    )
    shade.inertial = Inertial.from_geometry(
        Box((0.29, 0.29, 0.33)),
        mass=2.6,
        origin=Origin(xyz=(0.0, 0.0, -0.16)),
    )

    model.articulation(
        "base_tilt",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(xyz=(-0.14, 0.0, 0.160)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.5,
            lower=-0.30,
            upper=0.45,
        ),
    )
    model.articulation(
        "shade_tilt",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=shade,
        origin=Origin(xyz=(1.235, 0.0, 1.630)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=-0.70,
            upper=0.65,
        ),
    )

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    arm = object_model.get_part("arm")
    shade = object_model.get_part("shade")
    arm_joint = object_model.get_articulation("base_tilt")
    shade_joint = object_model.get_articulation("shade_tilt")

    ctx.expect_gap(
        shade,
        base,
        axis="z",
        min_gap=1.18,
        name="shade hangs well above the marble base",
    )
    ctx.expect_origin_gap(
        shade,
        base,
        axis="x",
        min_gap=0.95,
        name="shade projects forward of the base",
    )

    rest_shell = _aabb_center(ctx.part_element_world_aabb(shade, elem="shade_shell"))
    with ctx.pose({arm_joint: 0.35}):
        raised_shell = _aabb_center(ctx.part_element_world_aabb(shade, elem="shade_shell"))
        ctx.expect_gap(
            shade,
            base,
            axis="z",
            min_gap=1.24,
            name="raised arm keeps the shade clearly above the base",
        )
    with ctx.pose({arm_joint: -0.20}):
        lowered_shell = _aabb_center(ctx.part_element_world_aabb(shade, elem="shade_shell"))
        ctx.expect_gap(
            shade,
            base,
            axis="z",
            min_gap=0.94,
            name="lowered arm still keeps the shade above the base",
        )

    ctx.check(
        "positive base tilt lifts the arc",
        rest_shell is not None
        and raised_shell is not None
        and raised_shell[2] > rest_shell[2] + 0.10
        and raised_shell[0] < rest_shell[0] - 0.03,
        details=f"rest={rest_shell}, raised={raised_shell}",
    )
    ctx.check(
        "negative base tilt lowers the arc",
        rest_shell is not None
        and lowered_shell is not None
        and lowered_shell[2] < rest_shell[2] - 0.05
        and lowered_shell[0] > rest_shell[0] + 0.03,
        details=f"rest={rest_shell}, lowered={lowered_shell}",
    )

    with ctx.pose({shade_joint: 0.45}):
        forward_shell = _aabb_center(ctx.part_element_world_aabb(shade, elem="shade_shell"))
    with ctx.pose({shade_joint: -0.45}):
        backward_shell = _aabb_center(ctx.part_element_world_aabb(shade, elem="shade_shell"))

    ctx.check(
        "positive shade tilt aims the shade forward",
        rest_shell is not None
        and forward_shell is not None
        and forward_shell[0] > rest_shell[0] + 0.06,
        details=f"rest={rest_shell}, forward={forward_shell}",
    )
    ctx.check(
        "negative shade tilt aims the shade back",
        rest_shell is not None
        and backward_shell is not None
        and backward_shell[0] < rest_shell[0] - 0.06,
        details=f"rest={rest_shell}, backward={backward_shell}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
