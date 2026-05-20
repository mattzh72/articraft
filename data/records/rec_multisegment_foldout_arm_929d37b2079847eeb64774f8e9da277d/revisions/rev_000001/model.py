from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


TONGUE_THICKNESS = 0.008
CHEEK_THICKNESS = 0.006
CHEEK_CENTER_Y = -(TONGUE_THICKNESS + CHEEK_THICKNESS) / 2.0
FORK_OFFSET_Y = (TONGUE_THICKNESS + CHEEK_THICKNESS) / 2.0
PIN_HOLE_RADIUS = 0.0055
LINK_WEB_HEIGHT = 0.018
ROOT_BOSS_RADIUS = 0.013
DISTAL_BOSS_RADIUS = 0.012

LINK_1_LENGTH = 0.145
LINK_2_LENGTH = 0.130
LINK_3_LENGTH = 0.115
BRACKET_REACH = 0.062
PLATE_THICKNESS = 0.006
PLATE_OFFSET = 0.016
BOSS_LENGTH = PLATE_OFFSET - PLATE_THICKNESS / 2.0
BOSS_RADIUS = 0.0105
PLATE_HEIGHT = 0.024
END_CAP_RADIUS = 0.012


def _y_axis_origin(x: float, y: float, z: float = 0.0) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(pi / 2.0, 0.0, 0.0))


def _inboard_boss_origin(side_sign: int, x: float, z: float = 0.0) -> Origin:
    return _y_axis_origin(x, side_sign * (BOSS_LENGTH / 2.0), z)


def _side_plate_y(side_sign: int) -> float:
    return side_sign * PLATE_OFFSET


def _add_link_visuals(part, *, length: float, side_sign: int, material: str) -> None:
    plate_y = _side_plate_y(side_sign)
    part.visual(
        Box((length, PLATE_THICKNESS, PLATE_HEIGHT)),
        origin=Origin(xyz=(length / 2.0, plate_y, 0.0)),
        material=material,
        name="plate_web",
    )
    part.visual(
        Cylinder(radius=END_CAP_RADIUS, length=PLATE_THICKNESS),
        origin=_y_axis_origin(0.0, plate_y),
        material=material,
        name="root_cap",
    )
    part.visual(
        Cylinder(radius=END_CAP_RADIUS * 0.95, length=PLATE_THICKNESS),
        origin=_y_axis_origin(length, plate_y),
        material=material,
        name="distal_cap",
    )
    part.visual(
        Cylinder(radius=BOSS_RADIUS, length=BOSS_LENGTH),
        origin=_inboard_boss_origin(side_sign, 0.0),
        material=material,
        name="root_boss",
    )
    part.visual(
        Cylinder(radius=BOSS_RADIUS * 0.92, length=BOSS_LENGTH),
        origin=_inboard_boss_origin(side_sign, length),
        material=material,
        name="distal_boss",
    )


def _box_solid(
    x_size: float,
    y_size: float,
    z_size: float,
    *,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
) -> cq.Workplane:
    return cq.Workplane("XY").box(x_size, y_size, z_size).translate((x, y, z))


def _y_cylinder(radius: float, length: float, *, x: float = 0.0, z: float = 0.0) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .center(x, z)
        .circle(radius)
        .extrude(length)
        .translate((0.0, -length / 2.0, 0.0))
    )


def _slot_cut(x_start: float, x_end: float, width: float, height: float) -> cq.Workplane:
    return cq.Workplane("XY").box(x_end - x_start, width, height).translate(
        ((x_start + x_end) / 2.0, 0.0, 0.0)
    )


def _pin_hole(*, x: float, y_span: float = 0.030, z: float = 0.0) -> cq.Workplane:
    return _y_cylinder(PIN_HOLE_RADIUS, y_span, x=x, z=z)


def _fold_link_shape(length: float) -> cq.Workplane:
    tongue_reach = length - 0.030
    fork_start = length - 0.028

    tongue = _box_solid(
        tongue_reach,
        TONGUE_THICKNESS,
        LINK_WEB_HEIGHT,
        x=tongue_reach / 2.0,
    ).union(_y_cylinder(ROOT_BOSS_RADIUS, TONGUE_THICKNESS, x=0.0))

    fork_core = _box_solid(
        0.020,
        TONGUE_THICKNESS + 0.004,
        LINK_WEB_HEIGHT * 0.80,
        x=length - 0.038,
    )
    lower_cheek = _box_solid(
        length - fork_start,
        CHEEK_THICKNESS,
        LINK_WEB_HEIGHT * 1.05,
        x=(fork_start + length) / 2.0,
        y=-FORK_OFFSET_Y,
    ).union(_y_cylinder(DISTAL_BOSS_RADIUS, CHEEK_THICKNESS, x=length, z=0.0).translate((0.0, -FORK_OFFSET_Y, 0.0)))
    upper_cheek = _box_solid(
        length - fork_start,
        CHEEK_THICKNESS,
        LINK_WEB_HEIGHT * 1.05,
        x=(fork_start + length) / 2.0,
        y=FORK_OFFSET_Y,
    ).union(_y_cylinder(DISTAL_BOSS_RADIUS, CHEEK_THICKNESS, x=length, z=0.0).translate((0.0, FORK_OFFSET_Y, 0.0)))

    shape = tongue.union(fork_core).union(lower_cheek).union(upper_cheek)
    shape = shape.cut(_pin_hole(x=0.0))
    shape = shape.cut(_pin_hole(x=length))
    return shape


def _fixed_cheek_shape() -> cq.Workplane:
    shape = _box_solid(0.058, CHEEK_THICKNESS, 0.090, x=-0.031, y=CHEEK_CENTER_Y).union(
        _box_solid(0.032, CHEEK_THICKNESS, 0.052, x=-0.004, y=CHEEK_CENTER_Y)
    ).union(
        _y_cylinder(ROOT_BOSS_RADIUS, CHEEK_THICKNESS, x=0.0).translate((0.0, CHEEK_CENTER_Y, 0.0))
    )
    shape = shape.cut(_pin_hole(x=0.0))
    for hole_z in (-0.023, 0.023):
        shape = shape.cut(
            _y_cylinder(0.0055, CHEEK_THICKNESS + 0.004, x=-0.030, z=hole_z).translate(
                (0.0, CHEEK_CENTER_Y, 0.0)
            )
        )
    return shape


def _platform_bracket_shape() -> cq.Workplane:
    arm = _box_solid(0.040, TONGUE_THICKNESS, LINK_WEB_HEIGHT * 0.90, x=0.020).union(
        _box_solid(0.036, TONGUE_THICKNESS, 0.010, x=BRACKET_REACH - 0.010, z=-0.004)
    ).union(
        _box_solid(0.022, TONGUE_THICKNESS, 0.026, x=BRACKET_REACH, z=0.010)
    ).union(_y_cylinder(ROOT_BOSS_RADIUS, TONGUE_THICKNESS, x=0.0))
    arm = arm.cut(_pin_hole(x=0.0))
    deck = _box_solid(0.048, 0.040, 0.005, x=BRACKET_REACH + 0.024, z=0.026)
    back_flange = _box_solid(0.006, 0.040, 0.022, x=BRACKET_REACH, z=0.016)
    front_lip = _box_solid(0.006, 0.040, 0.014, x=BRACKET_REACH + 0.048, z=0.018)
    return arm.union(deck).union(back_flange).union(front_lip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fold_out_side_plate_arm")

    model.material("cheek_steel", rgba=(0.20, 0.22, 0.25, 1.0))
    model.material("link_aluminum", rgba=(0.73, 0.76, 0.80, 1.0))
    model.material("bracket_gray", rgba=(0.58, 0.61, 0.65, 1.0))

    fixed_cheek = model.part("fixed_cheek")
    fixed_cheek.visual(
        Box((0.060, PLATE_THICKNESS, 0.090)),
        origin=Origin(xyz=(-0.030, _side_plate_y(-1), 0.0)),
        material="cheek_steel",
        name="cheek_plate",
    )
    fixed_cheek.visual(
        Box((0.028, PLATE_THICKNESS, 0.050)),
        origin=Origin(xyz=(-0.004, _side_plate_y(-1), 0.0)),
        material="cheek_steel",
        name="cheek_nose",
    )
    fixed_cheek.visual(
        Cylinder(radius=0.015, length=PLATE_THICKNESS),
        origin=_y_axis_origin(0.0, _side_plate_y(-1)),
        material="cheek_steel",
        name="pivot_pad",
    )
    fixed_cheek.visual(
        Cylinder(radius=BOSS_RADIUS, length=BOSS_LENGTH),
        origin=_inboard_boss_origin(-1, 0.0),
        material="cheek_steel",
        name="pivot_boss",
    )

    link_1 = model.part("link_1")
    _add_link_visuals(link_1, length=LINK_1_LENGTH, side_sign=1, material="link_aluminum")

    link_2 = model.part("link_2")
    _add_link_visuals(link_2, length=LINK_2_LENGTH, side_sign=-1, material="link_aluminum")

    link_3 = model.part("link_3")
    _add_link_visuals(link_3, length=LINK_3_LENGTH, side_sign=1, material="link_aluminum")

    platform_bracket = model.part("platform_bracket")
    platform_bracket.visual(
        Box((BRACKET_REACH, PLATE_THICKNESS, 0.020)),
        origin=Origin(xyz=(BRACKET_REACH / 2.0, _side_plate_y(-1), 0.0)),
        material="bracket_gray",
        name="bracket_arm",
    )
    platform_bracket.visual(
        Cylinder(radius=END_CAP_RADIUS * 0.98, length=PLATE_THICKNESS),
        origin=_y_axis_origin(0.0, _side_plate_y(-1)),
        material="bracket_gray",
        name="root_cap",
    )
    platform_bracket.visual(
        Cylinder(radius=BOSS_RADIUS * 0.92, length=BOSS_LENGTH),
        origin=_inboard_boss_origin(-1, 0.0),
        material="bracket_gray",
        name="root_boss",
    )
    platform_bracket.visual(
        Box((0.052, 0.040, 0.005)),
        origin=Origin(xyz=(BRACKET_REACH + 0.026, -0.022, 0.026)),
        material="bracket_gray",
        name="deck",
    )
    platform_bracket.visual(
        Box((0.006, 0.040, 0.022)),
        origin=Origin(xyz=(BRACKET_REACH, -0.022, 0.017)),
        material="bracket_gray",
        name="back_flange",
    )
    platform_bracket.visual(
        Box((0.006, 0.040, 0.014)),
        origin=Origin(xyz=(BRACKET_REACH + 0.052, -0.022, 0.019)),
        material="bracket_gray",
        name="front_lip",
    )

    common_limits = MotionLimits(lower=-2.15, upper=2.15, effort=18.0, velocity=1.8)

    model.articulation(
        "cheek_to_link_1",
        ArticulationType.REVOLUTE,
        parent=fixed_cheek,
        child=link_1,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=common_limits,
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(LINK_1_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=common_limits,
    )
    model.articulation(
        "link_2_to_link_3",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=link_3,
        origin=Origin(xyz=(LINK_2_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=common_limits,
    )
    model.articulation(
        "link_3_to_platform_bracket",
        ArticulationType.REVOLUTE,
        parent=link_3,
        child=platform_bracket,
        origin=Origin(xyz=(LINK_3_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.75, upper=1.60, effort=10.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed_cheek = object_model.get_part("fixed_cheek")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    link_3 = object_model.get_part("link_3")
    platform_bracket = object_model.get_part("platform_bracket")

    cheek_to_link_1 = object_model.get_articulation("cheek_to_link_1")
    link_1_to_link_2 = object_model.get_articulation("link_1_to_link_2")
    link_2_to_link_3 = object_model.get_articulation("link_2_to_link_3")
    link_3_to_platform_bracket = object_model.get_articulation("link_3_to_platform_bracket")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    for joint in (
        cheek_to_link_1,
        link_1_to_link_2,
        link_2_to_link_3,
        link_3_to_platform_bracket,
    ):
        ctx.check(
            f"{joint.name}_parallel_axis",
            joint.axis == (0.0, -1.0, 0.0),
            details=f"expected supported parallel hinge axis (0, -1, 0), got {joint.axis}",
        )

    ctx.expect_contact(link_1, fixed_cheek, contact_tol=5e-4, name="cheek_supports_link_1")
    ctx.expect_contact(link_2, link_1, contact_tol=5e-4, name="link_1_supports_link_2")
    ctx.expect_contact(link_3, link_2, contact_tol=5e-4, name="link_2_supports_link_3")
    ctx.expect_contact(
        platform_bracket,
        link_3,
        contact_tol=5e-4,
        name="link_3_supports_platform_bracket",
    )

    ctx.expect_origin_gap(
        platform_bracket,
        fixed_cheek,
        axis="x",
        min_gap=LINK_1_LENGTH + LINK_2_LENGTH + LINK_3_LENGTH - 0.01,
        name="rest_pose_reaches_forward",
    )

    with ctx.pose(
        {
            cheek_to_link_1: 0.70,
            link_1_to_link_2: 0.58,
            link_2_to_link_3: 0.40,
            link_3_to_platform_bracket: -0.20,
        }
    ):
        ctx.expect_origin_gap(
            platform_bracket,
            fixed_cheek,
            axis="z",
            min_gap=0.18,
            name="folded_pose_lifts_platform",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
