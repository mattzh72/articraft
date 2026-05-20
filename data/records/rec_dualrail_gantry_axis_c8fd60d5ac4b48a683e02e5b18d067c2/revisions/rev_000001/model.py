from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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
)


FRAME_LENGTH = 1.20
FRAME_WIDTH = 0.78
SIDE_BEAM_WIDTH = 0.10
SIDE_BEAM_HEIGHT = 0.09
SIDE_BEAM_Y = (FRAME_WIDTH / 2.0) - (SIDE_BEAM_WIDTH / 2.0)
INNER_FRAME_SPAN = FRAME_WIDTH - (2.0 * SIDE_BEAM_WIDTH)
CROSS_MEMBER_X = 0.08
CROSS_MEMBER_Z = 0.07
RAIL_LENGTH = 1.02
RAIL_WIDTH = 0.028
RAIL_HEIGHT = 0.018

GANTRY_SHOE_X = 0.12
GANTRY_SHOE_Y = 0.085
GANTRY_SHOE_Z = 0.028
GANTRY_CHEEK_X = 0.12
GANTRY_CHEEK_Y = 0.06
GANTRY_CHEEK_Z = 0.162
GANTRY_CHEEK_Y_CENTER = SIDE_BEAM_Y - 0.03
BEAM_BODY_X = 0.12
BEAM_BODY_Y = 0.62
BEAM_BODY_Z = 0.12
BEAM_BODY_Z_CENTER = 0.25
GUIDE_PLATE_X = 0.014
GUIDE_PLATE_Y = 0.52
GUIDE_PLATE_Z = 0.13
GUIDE_PLATE_X_CENTER = (BEAM_BODY_X / 2.0) + (GUIDE_PLATE_X / 2.0)
GUIDE_PLATE_Z_CENTER = 0.235
GUIDE_RAIL_X = 0.018
GUIDE_RAIL_Y = 0.46
GUIDE_RAIL_Z = 0.016
GUIDE_RAIL_X_CENTER = GUIDE_PLATE_X_CENTER + (GUIDE_PLATE_X / 2.0) + (GUIDE_RAIL_X / 2.0)
GUIDE_UPPER_Z = 0.275
GUIDE_LOWER_Z = 0.195
GUIDE_FRONT_X = GUIDE_RAIL_X_CENTER + (GUIDE_RAIL_X / 2.0)
GUIDE_MID_Z = 0.5 * (GUIDE_UPPER_Z + GUIDE_LOWER_Z)

CARRIAGE_RUNNER_X = 0.022
CARRIAGE_RUNNER_Y = 0.14
CARRIAGE_RUNNER_Z = 0.028
CARRIAGE_RUNNER_OFFSET_Z = 0.04
CARRIAGE_BRIDGE_X = 0.016
CARRIAGE_BRIDGE_Y = 0.14
CARRIAGE_BRIDGE_Z = 0.108
CARRIAGE_BODY_X = 0.08
CARRIAGE_BODY_Y = 0.16
CARRIAGE_BODY_Z = 0.12
TOOL_PLATE_X = 0.028
TOOL_PLATE_Y = 0.10
TOOL_PLATE_Z = 0.14
TOOL_BARREL_RADIUS = 0.018
TOOL_BARREL_LENGTH = 0.08
TOOL_TIP_RADIUS = 0.009
TOOL_TIP_LENGTH = 0.05

BEAM_TRAVEL_LOWER = -0.30
BEAM_TRAVEL_UPPER = 0.30
TOOL_TRAVEL_LOWER = -0.15
TOOL_TRAVEL_UPPER = 0.15


def _add_box(
    part,
    *,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material: str,
    name: str,
) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _add_cylinder(
    part,
    *,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    rpy: tuple[float, float, float],
    material: str,
    name: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ladder_frame_gantry_axis")

    model.material("frame_aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("beam_aluminum", rgba=(0.80, 0.82, 0.85, 1.0))
    model.material("rail_steel", rgba=(0.33, 0.35, 0.38, 1.0))
    model.material("carriage_dark", rgba=(0.16, 0.18, 0.20, 1.0))
    model.material("tool_steel", rgba=(0.53, 0.55, 0.58, 1.0))

    side_beam_z = -(RAIL_HEIGHT + (SIDE_BEAM_HEIGHT / 2.0))
    cross_member_z = -(RAIL_HEIGHT + (CROSS_MEMBER_Z / 2.0))
    rail_z = -(RAIL_HEIGHT / 2.0)
    cheek_z = GANTRY_SHOE_Z + (GANTRY_CHEEK_Z / 2.0)
    shoe_z = GANTRY_SHOE_Z / 2.0

    base = model.part("base_frame")
    _add_box(
        base,
        size=(FRAME_LENGTH, SIDE_BEAM_WIDTH, SIDE_BEAM_HEIGHT),
        xyz=(0.0, SIDE_BEAM_Y, side_beam_z),
        material="frame_aluminum",
        name="left_side_beam",
    )
    _add_box(
        base,
        size=(FRAME_LENGTH, SIDE_BEAM_WIDTH, SIDE_BEAM_HEIGHT),
        xyz=(0.0, -SIDE_BEAM_Y, side_beam_z),
        material="frame_aluminum",
        name="right_side_beam",
    )
    for index, x_pos in enumerate((-0.50, 0.0, 0.50), start=1):
        _add_box(
            base,
            size=(CROSS_MEMBER_X, INNER_FRAME_SPAN, CROSS_MEMBER_Z),
            xyz=(x_pos, 0.0, cross_member_z),
            material="frame_aluminum",
            name=f"cross_member_{index}",
        )
    _add_box(
        base,
        size=(RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT),
        xyz=(0.0, SIDE_BEAM_Y, rail_z),
        material="rail_steel",
        name="left_rail",
    )
    _add_box(
        base,
        size=(RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT),
        xyz=(0.0, -SIDE_BEAM_Y, rail_z),
        material="rail_steel",
        name="right_rail",
    )

    beam = model.part("moving_beam")
    _add_box(
        beam,
        size=(GANTRY_SHOE_X, GANTRY_SHOE_Y, GANTRY_SHOE_Z),
        xyz=(0.0, SIDE_BEAM_Y, shoe_z),
        material="carriage_dark",
        name="left_shoe",
    )
    _add_box(
        beam,
        size=(GANTRY_SHOE_X, GANTRY_SHOE_Y, GANTRY_SHOE_Z),
        xyz=(0.0, -SIDE_BEAM_Y, shoe_z),
        material="carriage_dark",
        name="right_shoe",
    )
    _add_box(
        beam,
        size=(GANTRY_CHEEK_X, GANTRY_CHEEK_Y, GANTRY_CHEEK_Z),
        xyz=(0.0, GANTRY_CHEEK_Y_CENTER, cheek_z),
        material="beam_aluminum",
        name="left_cheek",
    )
    _add_box(
        beam,
        size=(GANTRY_CHEEK_X, GANTRY_CHEEK_Y, GANTRY_CHEEK_Z),
        xyz=(0.0, -GANTRY_CHEEK_Y_CENTER, cheek_z),
        material="beam_aluminum",
        name="right_cheek",
    )
    _add_box(
        beam,
        size=(BEAM_BODY_X, BEAM_BODY_Y, BEAM_BODY_Z),
        xyz=(0.0, 0.0, BEAM_BODY_Z_CENTER),
        material="beam_aluminum",
        name="beam_body",
    )
    _add_box(
        beam,
        size=(GUIDE_PLATE_X, GUIDE_PLATE_Y, GUIDE_PLATE_Z),
        xyz=(GUIDE_PLATE_X_CENTER, 0.0, GUIDE_PLATE_Z_CENTER),
        material="beam_aluminum",
        name="guide_plate",
    )
    _add_box(
        beam,
        size=(GUIDE_RAIL_X, GUIDE_RAIL_Y, GUIDE_RAIL_Z),
        xyz=(GUIDE_RAIL_X_CENTER, 0.0, GUIDE_UPPER_Z),
        material="rail_steel",
        name="upper_guide",
    )
    _add_box(
        beam,
        size=(GUIDE_RAIL_X, GUIDE_RAIL_Y, GUIDE_RAIL_Z),
        xyz=(GUIDE_RAIL_X_CENTER, 0.0, GUIDE_LOWER_Z),
        material="rail_steel",
        name="lower_guide",
    )

    carriage = model.part("tool_carriage")
    _add_box(
        carriage,
        size=(CARRIAGE_RUNNER_X, CARRIAGE_RUNNER_Y, CARRIAGE_RUNNER_Z),
        xyz=(CARRIAGE_RUNNER_X / 2.0, 0.0, CARRIAGE_RUNNER_OFFSET_Z),
        material="carriage_dark",
        name="upper_runner",
    )
    _add_box(
        carriage,
        size=(CARRIAGE_RUNNER_X, CARRIAGE_RUNNER_Y, CARRIAGE_RUNNER_Z),
        xyz=(CARRIAGE_RUNNER_X / 2.0, 0.0, -CARRIAGE_RUNNER_OFFSET_Z),
        material="carriage_dark",
        name="lower_runner",
    )
    _add_box(
        carriage,
        size=(CARRIAGE_BRIDGE_X, CARRIAGE_BRIDGE_Y, CARRIAGE_BRIDGE_Z),
        xyz=(CARRIAGE_RUNNER_X + (CARRIAGE_BRIDGE_X / 2.0), 0.0, 0.0),
        material="beam_aluminum",
        name="bridge_plate",
    )
    _add_box(
        carriage,
        size=(CARRIAGE_BODY_X, CARRIAGE_BODY_Y, CARRIAGE_BODY_Z),
        xyz=(CARRIAGE_RUNNER_X + CARRIAGE_BRIDGE_X + (CARRIAGE_BODY_X / 2.0), 0.0, 0.0),
        material="carriage_dark",
        name="carriage_body",
    )
    _add_box(
        carriage,
        size=(TOOL_PLATE_X, TOOL_PLATE_Y, TOOL_PLATE_Z),
        xyz=(
            CARRIAGE_RUNNER_X
            + CARRIAGE_BRIDGE_X
            + CARRIAGE_BODY_X
            + (TOOL_PLATE_X / 2.0),
            0.0,
            -0.01,
        ),
        material="beam_aluminum",
        name="tool_plate",
    )
    _add_cylinder(
        carriage,
        radius=TOOL_BARREL_RADIUS,
        length=TOOL_BARREL_LENGTH,
        xyz=(
            CARRIAGE_RUNNER_X
            + CARRIAGE_BRIDGE_X
            + CARRIAGE_BODY_X
            + TOOL_PLATE_X
            + (TOOL_BARREL_LENGTH / 2.0),
            0.0,
            -0.02,
        ),
        rpy=(0.0, pi / 2.0, 0.0),
        material="tool_steel",
        name="tool_barrel",
    )
    _add_cylinder(
        carriage,
        radius=TOOL_TIP_RADIUS,
        length=TOOL_TIP_LENGTH,
        xyz=(
            CARRIAGE_RUNNER_X
            + CARRIAGE_BRIDGE_X
            + CARRIAGE_BODY_X
            + TOOL_PLATE_X
            + TOOL_BARREL_LENGTH
            + (TOOL_TIP_LENGTH / 2.0),
            0.0,
            -0.02,
        ),
        rpy=(0.0, pi / 2.0, 0.0),
        material="tool_steel",
        name="tool_tip",
    )

    model.articulation(
        "base_to_beam",
        ArticulationType.PRISMATIC,
        parent=base,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=BEAM_TRAVEL_LOWER,
            upper=BEAM_TRAVEL_UPPER,
            effort=900.0,
            velocity=0.70,
        ),
    )
    model.articulation(
        "beam_to_tool_carriage",
        ArticulationType.PRISMATIC,
        parent=beam,
        child=carriage,
        origin=Origin(xyz=(GUIDE_FRONT_X, 0.0, GUIDE_MID_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=TOOL_TRAVEL_LOWER,
            upper=TOOL_TRAVEL_UPPER,
            effort=300.0,
            velocity=0.50,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_frame")
    beam = object_model.get_part("moving_beam")
    carriage = object_model.get_part("tool_carriage")
    beam_axis = object_model.get_articulation("base_to_beam")
    tool_axis = object_model.get_articulation("beam_to_tool_carriage")

    left_rail = base.get_visual("left_rail")
    right_rail = base.get_visual("right_rail")
    left_shoe = beam.get_visual("left_shoe")
    right_shoe = beam.get_visual("right_shoe")
    upper_guide = beam.get_visual("upper_guide")
    lower_guide = beam.get_visual("lower_guide")
    upper_runner = carriage.get_visual("upper_runner")
    lower_runner = carriage.get_visual("lower_runner")

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

    ctx.expect_contact(
        beam,
        base,
        elem_a=left_shoe,
        elem_b=left_rail,
        name="left_shoe_contacts_left_rail",
    )
    ctx.expect_contact(
        beam,
        base,
        elem_a=right_shoe,
        elem_b=right_rail,
        name="right_shoe_contacts_right_rail",
    )
    ctx.expect_contact(
        carriage,
        beam,
        elem_a=upper_runner,
        elem_b=upper_guide,
        name="upper_runner_contacts_upper_guide",
    )
    ctx.expect_contact(
        carriage,
        beam,
        elem_a=lower_runner,
        elem_b=lower_guide,
        name="lower_runner_contacts_lower_guide",
    )
    ctx.expect_origin_gap(
        carriage,
        beam,
        axis="y",
        min_gap=-1e-6,
        max_gap=1e-6,
        name="tool_carriage_is_centered_on_beam_at_home",
    )

    with ctx.pose({beam_axis: 0.20}):
        ctx.expect_origin_gap(
            beam,
            base,
            axis="x",
            min_gap=0.195,
            max_gap=0.205,
            name="beam_moves_along_positive_x",
        )

    with ctx.pose({tool_axis: 0.12}):
        ctx.expect_origin_gap(
            carriage,
            beam,
            axis="y",
            min_gap=0.115,
            max_gap=0.125,
            name="tool_carriage_moves_along_positive_y",
        )

    with ctx.pose({beam_axis: BEAM_TRAVEL_LOWER}):
        ctx.expect_within(
            beam,
            base,
            axes="x",
            inner_elem=left_shoe,
            outer_elem=left_rail,
            margin=0.0,
            name="left_shoe_stays_on_left_rail_at_x_lower_limit",
        )

    with ctx.pose({beam_axis: BEAM_TRAVEL_UPPER}):
        ctx.expect_within(
            beam,
            base,
            axes="x",
            inner_elem=left_shoe,
            outer_elem=left_rail,
            margin=0.0,
            name="left_shoe_stays_on_left_rail_at_x_upper_limit",
        )

    with ctx.pose({tool_axis: TOOL_TRAVEL_LOWER}):
        ctx.expect_within(
            carriage,
            beam,
            axes="y",
            inner_elem=upper_runner,
            outer_elem=upper_guide,
            margin=0.0,
            name="upper_runner_stays_on_upper_guide_at_y_lower_limit",
        )

    with ctx.pose({tool_axis: TOOL_TRAVEL_UPPER}):
        ctx.expect_within(
            carriage,
            beam,
            axes="y",
            inner_elem=upper_runner,
            outer_elem=upper_guide,
            margin=0.0,
            name="upper_runner_stays_on_upper_guide_at_y_upper_limit",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
