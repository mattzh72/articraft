from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


RAIL_LENGTH = 0.78
BASE_WIDTH = 0.16
BASE_THICKNESS = 0.018
GUIDE_LENGTH = 0.68
GUIDE_WIDTH = 0.024
GUIDE_HEIGHT = 0.028
RAIL_PAIR_SPACING = 0.076
END_STOP_LENGTH = 0.034
END_STOP_WIDTH = 0.112
END_STOP_HEIGHT = 0.028

CARRIAGE_LENGTH = 0.16
CARRIAGE_WIDTH = 0.118
CARRIAGE_SHOE_WIDTH = 0.026
CARRIAGE_SHOE_HEIGHT = 0.024
CARRIAGE_BRIDGE_HEIGHT = 0.018
CARRIAGE_BRIDGE_WIDTH = 0.104
CARRIAGE_TOP_PAD_HEIGHT = 0.018
CARRIAGE_TOP_PAD_WIDTH = 0.072
CARRIAGE_TOP_PAD_Z = 0.028
CARRIAGE_SIDE_PLATE_LENGTH = 0.090
CARRIAGE_SIDE_PLATE_THICKNESS = 0.016
CARRIAGE_SIDE_PLATE_HEIGHT = 0.024
CARRIAGE_SIDE_PLATE_Y = 0.057
CARRIAGE_SIDE_PLATE_Z = 0.020

CLEVIS_LUG_THICKNESS = 0.014
CLEVIS_GAP = 0.036
CLEVIS_LUG_REACH = 0.018
CLEVIS_LUG_HEIGHT = 0.028
CLEVIS_PIVOT_Y = 0.072
CLEVIS_PIVOT_Z = 0.054
ARM_ROOT_WIDTH = CLEVIS_GAP
ARM_ROOT_HEIGHT = 0.018
ARM_ROOT_Y = 0.012

ARM_LENGTH = 0.125
ARM_THICKNESS = 0.018
ARM_HEIGHT = 0.018
ARM_TIP_RADIUS = 0.014

SLIDE_START_X = -0.18
SLIDE_TRAVEL = 0.36
RAIL_TOP_Z = BASE_THICKNESS + GUIDE_HEIGHT


def make_base_rail() -> cq.Workplane:
    base = (
        cq.Workplane("XY")
        .box(
            RAIL_LENGTH,
            BASE_WIDTH,
            BASE_THICKNESS,
            centered=(True, True, False),
        )
        .edges("|Z")
        .fillet(0.004)
    )
    rail_offset_y = RAIL_PAIR_SPACING / 2.0
    left_rail = (
        cq.Workplane("XY")
        .box(
            GUIDE_LENGTH,
            GUIDE_WIDTH,
            GUIDE_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, -rail_offset_y, BASE_THICKNESS))
    )
    right_rail = (
        cq.Workplane("XY")
        .box(
            GUIDE_LENGTH,
            GUIDE_WIDTH,
            GUIDE_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, rail_offset_y, BASE_THICKNESS))
    )
    stop_offset = GUIDE_LENGTH / 2.0 + END_STOP_LENGTH / 2.0
    left_stop = (
        cq.Workplane("XY")
        .box(
            END_STOP_LENGTH,
            END_STOP_WIDTH,
            END_STOP_HEIGHT,
            centered=(True, True, False),
        )
        .translate((-stop_offset, 0.0, BASE_THICKNESS))
    )
    right_stop = (
        cq.Workplane("XY")
        .box(
            END_STOP_LENGTH,
            END_STOP_WIDTH,
            END_STOP_HEIGHT,
            centered=(True, True, False),
        )
        .translate((stop_offset, 0.0, BASE_THICKNESS))
    )
    return base.union(left_rail).union(right_rail).union(left_stop).union(right_stop)


def make_carriage() -> cq.Workplane:
    rail_offset_y = RAIL_PAIR_SPACING / 2.0
    left_shoe = (
        cq.Workplane("XY")
        .box(
            CARRIAGE_LENGTH,
            CARRIAGE_SHOE_WIDTH,
            CARRIAGE_SHOE_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, -rail_offset_y, 0.0))
    )
    right_shoe = (
        cq.Workplane("XY")
        .box(
            CARRIAGE_LENGTH,
            CARRIAGE_SHOE_WIDTH,
            CARRIAGE_SHOE_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, rail_offset_y, 0.0))
    )
    bridge = (
        cq.Workplane("XY")
        .box(
            CARRIAGE_LENGTH,
            CARRIAGE_BRIDGE_WIDTH,
            CARRIAGE_BRIDGE_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, CARRIAGE_SHOE_HEIGHT))
    )
    top_pad = (
        cq.Workplane("XY")
        .box(
            CARRIAGE_LENGTH * 0.62,
            CARRIAGE_TOP_PAD_WIDTH,
            CARRIAGE_TOP_PAD_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, CARRIAGE_TOP_PAD_Z))
    )
    side_plate = (
        cq.Workplane("XY")
        .box(
            CARRIAGE_SIDE_PLATE_LENGTH,
            CARRIAGE_SIDE_PLATE_THICKNESS,
            CARRIAGE_SIDE_PLATE_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, CARRIAGE_SIDE_PLATE_Y, CARRIAGE_SIDE_PLATE_Z))
    )
    carriage = left_shoe.union(right_shoe).union(bridge).union(top_pad).union(side_plate)

    lug_x = CLEVIS_GAP / 2.0 + CLEVIS_LUG_THICKNESS / 2.0
    lug_a = (
        cq.Workplane("XY")
        .box(
            CLEVIS_LUG_THICKNESS,
            CLEVIS_LUG_REACH,
            CLEVIS_LUG_HEIGHT,
            centered=(True, True, False),
        )
        .translate((-lug_x, CLEVIS_PIVOT_Y, CLEVIS_PIVOT_Z - CLEVIS_LUG_HEIGHT / 2.0))
    )
    lug_b = (
        cq.Workplane("XY")
        .box(
            CLEVIS_LUG_THICKNESS,
            CLEVIS_LUG_REACH,
            CLEVIS_LUG_HEIGHT,
            centered=(True, True, False),
        )
        .translate((lug_x, CLEVIS_PIVOT_Y, CLEVIS_PIVOT_Z - CLEVIS_LUG_HEIGHT / 2.0))
    )
    return carriage.union(lug_a).union(lug_b)


def make_arm() -> cq.Workplane:
    arm_start_y = CLEVIS_LUG_REACH / 2.0
    root_block = (
        cq.Workplane("XY")
        .box(
            ARM_ROOT_WIDTH,
            ARM_ROOT_Y,
            ARM_ROOT_HEIGHT,
            centered=(True, True, True),
        )
        .translate((0.0, arm_start_y + ARM_ROOT_Y / 2.0, 0.0))
    )
    beam = (
        cq.Workplane("XY")
        .box(
            ARM_THICKNESS,
            ARM_LENGTH,
            ARM_HEIGHT,
            centered=(True, False, True),
        )
        .translate((0.0, arm_start_y, 0.0))
    )
    tip = (
        cq.Workplane("YZ")
        .circle(ARM_TIP_RADIUS)
        .extrude(ARM_THICKNESS)
        .translate((-ARM_THICKNESS / 2.0, arm_start_y + ARM_LENGTH, 0.0))
    )
    return root_block.union(beam).union(tip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="prismatic_revolute_chain")

    rail_material = model.material("rail_steel", rgba=(0.44, 0.46, 0.50, 1.0))
    carriage_material = model.material("carriage_dark", rgba=(0.15, 0.16, 0.18, 1.0))
    arm_material = model.material("arm_paint", rgba=(0.78, 0.33, 0.14, 1.0))

    base_rail = model.part("base_rail")
    base_rail.visual(
        mesh_from_cadquery(make_base_rail(), "base_rail"),
        material=rail_material,
        name="rail_shell",
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(make_carriage(), "carriage"),
        material=carriage_material,
        name="carriage_shell",
    )

    arm = model.part("arm")
    arm.visual(
        mesh_from_cadquery(make_arm(), "arm"),
        material=arm_material,
        name="arm_shell",
    )

    model.articulation(
        "rail_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base_rail,
        child=carriage,
        origin=Origin(xyz=(SLIDE_START_X, 0.0, RAIL_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.45,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )
    model.articulation(
        "carriage_to_arm",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=arm,
        origin=Origin(xyz=(0.0, CLEVIS_PIVOT_Y, CLEVIS_PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=2.5,
            lower=-0.6,
            upper=1.2,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_rail = object_model.get_part("base_rail")
    carriage = object_model.get_part("carriage")
    arm = object_model.get_part("arm")
    slide = object_model.get_articulation("rail_to_carriage")
    hinge = object_model.get_articulation("carriage_to_arm")

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

    ctx.check(
        "prismatic axis follows rail",
        tuple(slide.axis) == (1.0, 0.0, 0.0),
        details=f"expected slide axis (1, 0, 0), got {slide.axis}",
    )
    ctx.check(
        "hinge axis supported on clevis",
        tuple(hinge.axis) == (1.0, 0.0, 0.0),
        details=f"expected hinge axis (1, 0, 0), got {hinge.axis}",
    )
    ctx.expect_contact(
        carriage,
        base_rail,
        contact_tol=0.003,
        name="carriage is physically guided by rail",
    )
    ctx.expect_contact(
        arm,
        carriage,
        contact_tol=0.003,
        name="arm is supported by clevis pivot",
    )

    closed_carriage_x = ctx.part_world_position(carriage)[0]
    with ctx.pose({slide: slide.motion_limits.upper}):
        open_carriage_x = ctx.part_world_position(carriage)[0]
    ctx.check(
        "carriage translates along rail",
        open_carriage_x > closed_carriage_x + 0.30,
        details=(
            f"expected >0.30 m travel, got {open_carriage_x - closed_carriage_x:.4f} m"
        ),
    )

    closed_arm_aabb = ctx.part_world_aabb(arm)
    with ctx.pose({hinge: hinge.motion_limits.upper}):
        raised_arm_aabb = ctx.part_world_aabb(arm)
    ctx.check(
        "arm lifts upward about side pivot",
        raised_arm_aabb[1][2] > closed_arm_aabb[1][2] + 0.05,
        details=(
            f"expected arm max z to rise by >0.05 m, got "
            f"{raised_arm_aabb[1][2] - closed_arm_aabb[1][2]:.4f} m"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
