from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, pi, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="timber_belfry")

    timber = model.material("timber", rgba=(0.55, 0.38, 0.22, 1.0))
    weathered_timber = model.material("weathered_timber", rgba=(0.45, 0.31, 0.18, 1.0))
    roof_slate = model.material("roof_slate", rgba=(0.22, 0.22, 0.24, 1.0))
    iron = model.material("iron", rgba=(0.20, 0.21, 0.22, 1.0))
    bell_bronze = model.material("bell_bronze", rgba=(0.63, 0.46, 0.21, 1.0))

    post_radius = 0.095
    post_height = 2.62
    post_offset = 0.78

    platform_beam_z = 1.34
    platform_beam_h = 0.12
    deck_thickness = 0.05
    deck_z = platform_beam_z + 0.5 * platform_beam_h + 0.5 * deck_thickness

    headstock_axis_z = 2.24
    top_plate_z = 2.42
    ridge_z = 3.05
    roof_half_run = 1.02
    roof_rise = ridge_z - top_plate_z
    roof_angle = atan2(roof_rise, roof_half_run)
    roof_slope_len = sqrt(roof_half_run**2 + roof_rise**2)
    roof_mid_z = top_plate_z + 0.5 * roof_rise + 0.02

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((2.30, 2.30, 3.20)),
        mass=420.0,
        origin=Origin(xyz=(0.0, 0.0, 1.60)),
    )

    post_names = {
        (-post_offset, -post_offset): "post_front_left",
        (post_offset, -post_offset): "post_front_right",
        (-post_offset, post_offset): "post_back_left",
        (post_offset, post_offset): "post_back_right",
    }
    for (x_pos, y_pos), visual_name in post_names.items():
        frame.visual(
            Cylinder(radius=post_radius, length=post_height),
            origin=Origin(xyz=(x_pos, y_pos, 0.5 * post_height)),
            material=timber,
            name=visual_name,
        )

    frame.visual(
        Box((1.56, 0.16, platform_beam_h)),
        origin=Origin(xyz=(0.0, -post_offset, platform_beam_z)),
        material=weathered_timber,
        name="platform_front",
    )
    frame.visual(
        Box((1.56, 0.16, platform_beam_h)),
        origin=Origin(xyz=(0.0, post_offset, platform_beam_z)),
        material=weathered_timber,
        name="platform_back",
    )
    frame.visual(
        Box((0.16, 1.56, platform_beam_h)),
        origin=Origin(xyz=(-post_offset, 0.0, platform_beam_z)),
        material=weathered_timber,
        name="platform_left",
    )
    frame.visual(
        Box((0.16, 1.56, platform_beam_h)),
        origin=Origin(xyz=(post_offset, 0.0, platform_beam_z)),
        material=weathered_timber,
        name="platform_right",
    )

    frame.visual(
        Box((1.40, 0.28, deck_thickness)),
        origin=Origin(xyz=(0.0, -0.54, deck_z)),
        material=weathered_timber,
        name="platform_deck_front",
    )
    frame.visual(
        Box((1.40, 0.28, deck_thickness)),
        origin=Origin(xyz=(0.0, 0.54, deck_z)),
        material=weathered_timber,
        name="platform_deck_back",
    )
    frame.visual(
        Box((0.28, 0.84, deck_thickness)),
        origin=Origin(xyz=(-0.54, 0.0, deck_z)),
        material=weathered_timber,
        name="platform_deck_left",
    )
    frame.visual(
        Box((0.28, 0.84, deck_thickness)),
        origin=Origin(xyz=(0.54, 0.0, deck_z)),
        material=weathered_timber,
        name="platform_deck_right",
    )

    brace_angle = atan2(0.28, 0.26)
    brace_len = 0.40
    brace_z = 1.15
    brace_x = 0.62
    brace_y = 0.62
    for y_pos in (-post_offset, post_offset):
        frame.visual(
            Box((brace_len, 0.08, 0.08)),
            origin=Origin(xyz=(-brace_x, y_pos, brace_z), rpy=(0.0, -brace_angle, 0.0)),
            material=weathered_timber,
        )
        frame.visual(
            Box((brace_len, 0.08, 0.08)),
            origin=Origin(xyz=(brace_x, y_pos, brace_z), rpy=(0.0, brace_angle, 0.0)),
            material=weathered_timber,
        )
    for x_pos in (-post_offset, post_offset):
        frame.visual(
            Box((0.08, brace_len, 0.08)),
            origin=Origin(xyz=(x_pos, -brace_y, brace_z), rpy=(brace_angle, 0.0, 0.0)),
            material=weathered_timber,
        )
        frame.visual(
            Box((0.08, brace_len, 0.08)),
            origin=Origin(xyz=(x_pos, brace_y, brace_z), rpy=(-brace_angle, 0.0, 0.0)),
            material=weathered_timber,
        )

    frame.visual(
        Box((1.62, 0.14, 0.12)),
        origin=Origin(xyz=(0.0, -post_offset, top_plate_z)),
        material=weathered_timber,
        name="top_plate_front",
    )
    frame.visual(
        Box((1.62, 0.14, 0.12)),
        origin=Origin(xyz=(0.0, post_offset, top_plate_z)),
        material=weathered_timber,
        name="top_plate_back",
    )
    frame.visual(
        Box((0.14, 1.62, 0.12)),
        origin=Origin(xyz=(-post_offset, 0.0, top_plate_z)),
        material=weathered_timber,
        name="top_plate_left",
    )
    frame.visual(
        Box((0.14, 1.62, 0.12)),
        origin=Origin(xyz=(post_offset, 0.0, top_plate_z)),
        material=weathered_timber,
        name="top_plate_right",
    )

    side_rail_z = 2.12
    frame.visual(
        Box((0.14, 1.56, 0.14)),
        origin=Origin(xyz=(-post_offset, 0.0, side_rail_z)),
        material=weathered_timber,
        name="headstock_side_left",
    )
    frame.visual(
        Box((0.14, 1.56, 0.14)),
        origin=Origin(xyz=(post_offset, 0.0, side_rail_z)),
        material=weathered_timber,
        name="headstock_side_right",
    )

    bearing_center_x = 0.63
    frame.visual(
        Box((0.18, 0.22, 0.16)),
        origin=Origin(xyz=(-bearing_center_x, 0.0, headstock_axis_z)),
        material=weathered_timber,
        name="left_bearing",
    )
    frame.visual(
        Box((0.18, 0.22, 0.16)),
        origin=Origin(xyz=(bearing_center_x, 0.0, headstock_axis_z)),
        material=weathered_timber,
        name="right_bearing",
    )

    frame.visual(
        Box((1.94, 0.12, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, ridge_z)),
        material=weathered_timber,
        name="ridge_beam",
    )
    for x_pos in (-0.52, 0.52):
        frame.visual(
            Box((0.10, roof_slope_len + 0.16, 0.10)),
            origin=Origin(xyz=(x_pos, -0.5 * roof_half_run, roof_mid_z), rpy=(roof_angle, 0.0, 0.0)),
            material=weathered_timber,
        )
        frame.visual(
            Box((0.10, roof_slope_len + 0.16, 0.10)),
            origin=Origin(xyz=(x_pos, 0.5 * roof_half_run, roof_mid_z), rpy=(-roof_angle, 0.0, 0.0)),
            material=weathered_timber,
        )

    frame.visual(
        Box((2.08, roof_slope_len + 0.20, 0.055)),
        origin=Origin(xyz=(0.0, -0.5 * roof_half_run, roof_mid_z + 0.035), rpy=(roof_angle, 0.0, 0.0)),
        material=roof_slate,
        name="roof_front",
    )
    frame.visual(
        Box((2.08, roof_slope_len + 0.20, 0.055)),
        origin=Origin(xyz=(0.0, 0.5 * roof_half_run, roof_mid_z + 0.035), rpy=(-roof_angle, 0.0, 0.0)),
        material=roof_slate,
        name="roof_back",
    )

    bell = model.part("bell")
    bell.inertial = Inertial.from_geometry(
        Box((1.10, 0.82, 0.90)),
        mass=95.0,
        origin=Origin(xyz=(0.0, 0.0, -0.34)),
    )

    bell_outer = [
        (0.060, 0.000),
        (0.115, -0.050),
        (0.185, -0.120),
        (0.255, -0.260),
        (0.335, -0.500),
        (0.390, -0.680),
    ]
    bell_inner = [
        (0.022, -0.008),
        (0.060, -0.050),
        (0.120, -0.145),
        (0.205, -0.340),
        (0.300, -0.575),
        (0.335, -0.680),
    ]
    bell.visual(
        _mesh(
            "belfry_bell_shell",
            LatheGeometry.from_shell_profiles(
                bell_outer,
                bell_inner,
                segments=72,
                start_cap="round",
                end_cap="flat",
                lip_samples=10,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.06)),
        material=bell_bronze,
        name="bell_shell",
    )
    bell.visual(
        Cylinder(radius=0.045, length=1.08),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=iron,
        name="bell_axle",
    )
    bell.visual(
        Box((0.24, 0.18, 0.07)),
        origin=Origin(xyz=(0.0, 0.0, -0.08)),
        material=weathered_timber,
        name="yoke_block",
    )
    bell.visual(
        Box((0.26, 0.025, 0.20)),
        origin=Origin(xyz=(0.0, -0.08, -0.15)),
        material=iron,
        name="yoke_front",
    )
    bell.visual(
        Box((0.26, 0.025, 0.20)),
        origin=Origin(xyz=(0.0, 0.08, -0.15)),
        material=iron,
        name="yoke_back",
    )

    model.articulation(
        "bell_swing",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=bell,
        origin=Origin(xyz=(0.0, 0.0, headstock_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=1.5, lower=-1.0, upper=1.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    bell = object_model.get_part("bell")
    bell_swing = object_model.get_articulation("bell_swing")

    ctx.expect_contact(
        bell,
        frame,
        elem_a="bell_axle",
        elem_b="left_bearing",
        name="bell axle seats in the left timber bearing",
    )
    ctx.expect_contact(
        bell,
        frame,
        elem_a="bell_axle",
        elem_b="right_bearing",
        name="bell axle seats in the right timber bearing",
    )

    ctx.expect_gap(
        frame,
        bell,
        axis="z",
        positive_elem="roof_back",
        negative_elem="bell_shell",
        min_gap=0.18,
        name="bell clears the saddle roof at rest",
    )

    rest_shell = ctx.part_element_world_aabb(bell, elem="bell_shell")
    with ctx.pose({bell_swing: 0.75}):
        ctx.expect_gap(
            frame,
            bell,
            axis="z",
            positive_elem="roof_back",
            negative_elem="bell_shell",
            min_gap=0.08,
            name="bell still clears the roof when swung",
        )
        swung_shell = ctx.part_element_world_aabb(bell, elem="bell_shell")

    ctx.check(
        "bell swings toward positive y for positive hinge motion",
        rest_shell is not None
        and swung_shell is not None
        and swung_shell[1][1] > rest_shell[1][1] + 0.18,
        details=f"rest={rest_shell}, swung={swung_shell}",
    )
    ctx.check(
        "bell mouth rises as the bell swings",
        rest_shell is not None
        and swung_shell is not None
        and swung_shell[1][2] > rest_shell[1][2] + 0.05,
        details=f"rest={rest_shell}, swung={swung_shell}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
