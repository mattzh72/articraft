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
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _xy_loop(
    width: float,
    depth: float,
    radius: float,
    z: float,
    *,
    shift_x: float = 0.0,
    shift_y: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (x + shift_x, y + shift_y, z)
        for x, y in rounded_rect_profile(width, depth, radius)
    ]


def _yz_loop(
    width: float,
    height: float,
    radius: float,
    x: float,
    *,
    center_z: float = 0.0,
    center_y: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (x, y + center_y, z + center_z)
        for z, y in rounded_rect_profile(height, width, radius)
    ]


def _build_paddle_mesh():
    outer = rounded_rect_profile(0.058, 0.100, 0.010)
    slot_upper = [(x, y + 0.024) for x, y in rounded_rect_profile(0.016, 0.026, 0.004)]
    slot_lower = [(x, y - 0.024) for x, y in rounded_rect_profile(0.016, 0.026, 0.004)]
    geom = ExtrudeWithHolesGeometry(
        outer,
        [slot_upper, slot_lower],
        0.006,
        cap=True,
        center=True,
        closed=True,
    )
    geom.rotate_x(math.pi / 2.0)
    geom.translate(0.0, 0.0, -0.148)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stand_mixer")

    body = model.material("body", rgba=(0.83, 0.18, 0.16, 1.0))
    trim = model.material("trim", rgba=(0.13, 0.13, 0.14, 1.0))
    steel = model.material("steel", rgba=(0.86, 0.87, 0.89, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.45, 0.47, 0.50, 1.0))

    base = model.part("base")

    base_shell = section_loft(
        [
            _xy_loop(0.400, 0.270, 0.060, 0.000),
            _xy_loop(0.350, 0.235, 0.040, 0.042),
            _xy_loop(0.300, 0.190, 0.032, 0.080),
        ]
    )
    base.visual(
        mesh_from_geometry(base_shell, "base_shell"),
        origin=Origin(xyz=(0.085, 0.0, 0.0)),
        material=body,
        name="base_shell",
    )

    pedestal_shell = section_loft(
        [
            _xy_loop(0.120, 0.135, 0.020, 0.000, shift_x=-0.090),
            _xy_loop(0.100, 0.122, 0.022, 0.140, shift_x=-0.080),
            _xy_loop(0.090, 0.112, 0.024, 0.270, shift_x=-0.064),
            _xy_loop(0.082, 0.108, 0.025, 0.365, shift_x=-0.052),
        ]
    )
    base.visual(
        mesh_from_geometry(pedestal_shell, "pedestal_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material=body,
        name="pedestal_shell",
    )
    base.visual(
        Cylinder(radius=0.022, length=0.026),
        origin=Origin(
            xyz=(-0.052, -0.039, 0.480),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_steel,
        name="left_hinge_lug",
    )
    base.visual(
        Cylinder(radius=0.022, length=0.026),
        origin=Origin(
            xyz=(-0.052, 0.039, 0.480),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_steel,
        name="right_hinge_lug",
    )
    base.visual(
        Box((0.032, 0.028, 0.028)),
        origin=Origin(xyz=(-0.052, -0.039, 0.451)),
        material=body,
        name="left_hinge_support",
    )
    base.visual(
        Box((0.032, 0.028, 0.028)),
        origin=Origin(xyz=(-0.052, 0.039, 0.451)),
        material=body,
        name="right_hinge_support",
    )
    base.visual(
        Box((0.180, 0.016, 0.018)),
        origin=Origin(xyz=(0.172, -0.060, 0.079)),
        material=trim,
        name="left_slide_rail",
    )
    base.visual(
        Box((0.180, 0.016, 0.018)),
        origin=Origin(xyz=(0.172, 0.060, 0.079)),
        material=trim,
        name="right_slide_rail",
    )
    base.visual(
        Box((0.060, 0.220, 0.010)),
        origin=Origin(xyz=(0.045, 0.0, 0.085)),
        material=trim,
        name="trim_band",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.430, 0.280, 0.470)),
        mass=12.5,
        origin=Origin(xyz=(0.045, 0.0, 0.235)),
    )

    cradle = model.part("cradle")
    cradle.visual(
        Box((0.150, 0.112, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=trim,
        name="carriage",
    )
    cradle.visual(
        Cylinder(radius=0.030, length=0.058),
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
        material=dark_steel,
        name="pedestal_post",
    )
    cradle.visual(
        Cylinder(radius=0.076, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.082)),
        material=steel,
        name="bowl_plate",
    )
    cradle.visual(
        Box((0.055, 0.018, 0.046)),
        origin=Origin(xyz=(-0.032, 0.047, 0.041)),
        material=trim,
        name="left_brace",
    )
    cradle.visual(
        Box((0.055, 0.018, 0.046)),
        origin=Origin(xyz=(-0.032, -0.047, 0.041)),
        material=trim,
        name="right_brace",
    )
    cradle.inertial = Inertial.from_geometry(
        Box((0.160, 0.120, 0.094)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
    )

    model.articulation(
        "base_to_cradle",
        ArticulationType.PRISMATIC,
        parent=base,
        child=cradle,
        origin=Origin(xyz=(0.165, 0.0, 0.082)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.08,
            lower=0.0,
            upper=0.040,
        ),
    )

    bowl = model.part("bowl")
    bowl_shell = LatheGeometry.from_shell_profiles(
        [
            (0.030, 0.000),
            (0.084, 0.015),
            (0.114, 0.070),
            (0.122, 0.145),
            (0.118, 0.198),
        ],
        [
            (0.000, 0.005),
            (0.074, 0.018),
            (0.104, 0.070),
            (0.111, 0.146),
            (0.108, 0.192),
        ],
        segments=60,
    )
    bowl.visual(
        mesh_from_geometry(bowl_shell, "mixing_bowl"),
        material=steel,
        name="bowl_shell",
    )
    bowl.visual(
        Cylinder(radius=0.032, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=dark_steel,
        name="bowl_foot",
    )
    bowl.inertial = Inertial.from_geometry(
        Cylinder(radius=0.122, length=0.200),
        mass=1.3,
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
    )

    model.articulation(
        "cradle_to_bowl",
        ArticulationType.FIXED,
        parent=cradle,
        child=bowl,
        origin=Origin(xyz=(0.0, 0.0, 0.088)),
    )

    head = model.part("head")
    head_shell = section_loft(
        [
            _yz_loop(0.086, 0.094, 0.018, 0.050, center_z=0.032),
            _yz_loop(0.168, 0.188, 0.040, 0.135, center_z=0.040),
            _yz_loop(0.150, 0.168, 0.036, 0.245, center_z=0.022),
            _yz_loop(0.110, 0.118, 0.024, 0.340, center_z=-0.004),
            _yz_loop(0.076, 0.084, 0.016, 0.394, center_z=-0.012),
        ]
    )
    head.visual(
        mesh_from_geometry(head_shell, "head_shell"),
        material=body,
        name="head_shell",
    )
    head.visual(
        Box((0.074, 0.020, 0.040)),
        origin=Origin(
            xyz=(0.034, 0.0, 0.008),
        ),
        material=body,
        name="rear_bridge",
    )
    head.visual(
        Cylinder(radius=0.016, length=0.018),
        origin=Origin(
            xyz=(0.0, -0.017, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_steel,
        name="left_hinge_cheek",
    )
    head.visual(
        Cylinder(radius=0.016, length=0.018),
        origin=Origin(
            xyz=(0.0, 0.017, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_steel,
        name="right_hinge_cheek",
    )
    head.visual(
        Box((0.090, 0.056, 0.070)),
        origin=Origin(xyz=(0.214, 0.0, -0.026)),
        material=body,
        name="nose_bridge",
    )
    head.visual(
        Cylinder(radius=0.016, length=0.024),
        origin=Origin(xyz=(0.217, 0.0, -0.072)),
        material=dark_steel,
        name="drive_housing",
    )
    head.visual(
        Box((0.030, 0.044, 0.028)),
        origin=Origin(xyz=(0.050, 0.0, -0.018)),
        material=trim,
        name="lock_receiver",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.410, 0.170, 0.190)),
        mass=5.8,
        origin=Origin(xyz=(0.190, 0.0, -0.028)),
    )

    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.052, 0.0, 0.480)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.0,
            lower=0.0,
            upper=math.radians(62.0),
        ),
    )

    paddle = model.part("paddle")
    paddle.visual(
        Cylinder(radius=0.007, length=0.076),
        origin=Origin(xyz=(0.0, 0.0, -0.062)),
        material=steel,
        name="shaft",
    )
    paddle.visual(
        Cylinder(radius=0.010, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.032)),
        material=dark_steel,
        name="hub",
    )
    paddle.visual(
        Box((0.016, 0.006, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, -0.106)),
        material=steel,
        name="stem",
    )
    paddle.visual(
        mesh_from_geometry(_build_paddle_mesh(), "paddle_plate"),
        material=steel,
        name="blade",
    )
    paddle.inertial = Inertial.from_geometry(
        Box((0.070, 0.016, 0.185)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, -0.092)),
    )

    model.articulation(
        "head_to_paddle",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=paddle,
        origin=Origin(xyz=(0.217, 0.0, -0.060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=22.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    cradle = object_model.get_part("cradle")
    bowl = object_model.get_part("bowl")
    head = object_model.get_part("head")
    paddle = object_model.get_part("paddle")

    cradle_slide = object_model.get_articulation("base_to_cradle")
    head_hinge = object_model.get_articulation("base_to_head")

    ctx.expect_origin_distance(
        bowl,
        paddle,
        axes="xy",
        max_dist=0.012,
        name="paddle drive axis stays centered over the bowl",
    )
    ctx.expect_gap(
        bowl,
        base,
        axis="z",
        min_gap=0.0,
        max_gap=0.090,
        positive_elem="bowl_foot",
        negative_elem="left_slide_rail",
        name="bowl is carried just above the base slide track",
    )
    ctx.expect_gap(
        head,
        bowl,
        axis="z",
        min_gap=0.0,
        max_gap=0.060,
        positive_elem="drive_housing",
        negative_elem="bowl_shell",
        name="drive housing sits just above the bowl rim",
    )

    rest_cradle_x = ctx.part_world_position(cradle)
    rest_paddle_pos = ctx.part_world_position(paddle)
    with ctx.pose({cradle_slide: 0.040}):
        moved_cradle_x = ctx.part_world_position(cradle)
    with ctx.pose({head_hinge: math.radians(62.0)}):
        opened_paddle_pos = ctx.part_world_position(paddle)
        ctx.expect_gap(
            paddle,
            bowl,
            axis="z",
            min_gap=0.090,
            name="tilted head lifts the paddle away from the bowl",
        )

    ctx.check(
        "cradle slides forward",
        rest_cradle_x is not None
        and moved_cradle_x is not None
        and moved_cradle_x[0] > rest_cradle_x[0] + 0.030,
        details=f"rest={rest_cradle_x}, moved={moved_cradle_x}",
    )
    ctx.check(
        "head opens upward",
        rest_paddle_pos is not None
        and opened_paddle_pos is not None
        and opened_paddle_pos[2] > rest_paddle_pos[2] + 0.100,
        details=f"rest={rest_paddle_pos}, opened={opened_paddle_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
