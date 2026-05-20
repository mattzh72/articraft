from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

BED_WIDTH = 0.56
BED_DEPTH = 0.38
BED_THICKNESS = 0.024

CUT_STRIP_WIDTH = 0.018
CUT_STRIP_LENGTH = 0.34
CUT_STRIP_THICKNESS = 0.003

BLADE_ARM_WIDTH = 0.045
BLADE_ARM_LENGTH = 0.435
BLADE_ARM_THICKNESS = 0.014

HOLD_BAR_WIDTH = 0.016
HOLD_BAR_LENGTH = 0.275
HOLD_BAR_THICKNESS = 0.006

BLADE_PIVOT_X = -BED_WIDTH / 2.0 + 0.032
BLADE_PIVOT_Y = BED_DEPTH / 2.0 - 0.025
CUT_STRIP_X = BLADE_PIVOT_X + 0.016
BLADE_PIVOT_Z = BED_THICKNESS + CUT_STRIP_THICKNESS + BLADE_ARM_THICKNESS / 2.0

HOLD_PIVOT_X = BLADE_PIVOT_X + 0.048
HOLD_PIVOT_Y = -BED_DEPTH / 2.0 + 0.025
HOLD_PIVOT_Z = BED_THICKNESS + 0.012


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="school_paper_cutter")

    bed_laminate = model.material("bed_laminate", color=(0.72, 0.66, 0.52))
    steel = model.material("steel", color=(0.74, 0.77, 0.80))
    dark_steel = model.material("dark_steel", color=(0.22, 0.24, 0.27))
    grip_black = model.material("grip_black", color=(0.09, 0.09, 0.10))
    guide_gray = model.material("guide_gray", color=(0.56, 0.58, 0.60))

    base = model.part("base")
    base.visual(
        Box((BED_WIDTH, BED_DEPTH, BED_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BED_THICKNESS / 2.0)),
        material=bed_laminate,
        name="bed",
    )
    base.visual(
        Box((CUT_STRIP_WIDTH, CUT_STRIP_LENGTH, CUT_STRIP_THICKNESS)),
        origin=Origin(
            xyz=(
                CUT_STRIP_X,
                -0.01,
                BED_THICKNESS + CUT_STRIP_THICKNESS / 2.0,
            )
        ),
        material=steel,
        name="cut_strip",
    )
    base.visual(
        Box((0.50, 0.025, 0.006)),
        origin=Origin(xyz=(0.03, BED_DEPTH / 2.0 - 0.0125, BED_THICKNESS + 0.003)),
        material=guide_gray,
        name="rear_rule",
    )
    base.visual(
        Box((0.024, 0.32, 0.006)),
        origin=Origin(xyz=(CUT_STRIP_X + 0.060, -0.01, BED_THICKNESS + 0.003)),
        material=guide_gray,
        name="side_rule",
    )
    base.visual(
        Box((0.055, 0.022, 0.020)),
        origin=Origin(xyz=(BLADE_PIVOT_X, BLADE_PIVOT_Y + 0.019, BED_THICKNESS + 0.010)),
        material=dark_steel,
        name="pivot_block",
    )
    base.visual(
        Box((0.012, 0.020, 0.016)),
        origin=Origin(xyz=(HOLD_PIVOT_X - 0.020, HOLD_PIVOT_Y, BED_THICKNESS + 0.008)),
        material=dark_steel,
        name="hold_bracket_left",
    )
    base.visual(
        Box((0.012, 0.020, 0.016)),
        origin=Origin(xyz=(HOLD_PIVOT_X + 0.020, HOLD_PIVOT_Y, BED_THICKNESS + 0.008)),
        material=dark_steel,
        name="hold_bracket_right",
    )
    base.visual(
        Cylinder(radius=0.003, length=0.060),
        origin=Origin(
            xyz=(HOLD_PIVOT_X, HOLD_PIVOT_Y, HOLD_PIVOT_Z),
            rpy=(0.0, 1.5707963267948966, 0.0),
        ),
        material=steel,
        name="hinge_rod",
    )
    base.inertial = Inertial.from_geometry(
        Box((BED_WIDTH, BED_DEPTH, BED_THICKNESS)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, BED_THICKNESS / 2.0)),
    )

    blade_arm = model.part("blade_arm")
    blade_arm.visual(
        Box((BLADE_ARM_WIDTH, BLADE_ARM_LENGTH, BLADE_ARM_THICKNESS)),
        origin=Origin(xyz=(0.0, -BLADE_ARM_LENGTH / 2.0, 0.0)),
        material=dark_steel,
        name="blade_plate",
    )
    blade_arm.visual(
        Box((0.005, 0.340, 0.004)),
        origin=Origin(xyz=(0.016, -0.170, -0.005)),
        material=steel,
        name="blade_edge",
    )
    blade_arm.visual(
        Box((0.052, 0.110, 0.018)),
        origin=Origin(xyz=(0.0, -0.410, 0.006)),
        material=grip_black,
        name="handle_block",
    )
    blade_arm.visual(
        Cylinder(radius=0.014, length=0.060),
        origin=Origin(
            xyz=(0.0, -0.445, 0.019),
            rpy=(0.0, 1.5707963267948966, 0.0),
        ),
        material=grip_black,
        name="handle_grip",
    )
    blade_arm.visual(
        Cylinder(radius=0.008, length=0.055),
        origin=Origin(rpy=(0.0, 1.5707963267948966, 0.0)),
        material=steel,
        name="pivot_hub",
    )
    blade_arm.inertial = Inertial.from_geometry(
        Box((BLADE_ARM_WIDTH, BLADE_ARM_LENGTH, BLADE_ARM_THICKNESS)),
        mass=1.4,
        origin=Origin(xyz=(0.0, -BLADE_ARM_LENGTH / 2.0, 0.0)),
    )

    hold_down_bar = model.part("hold_down_bar")
    hold_down_bar.visual(
        Box((HOLD_BAR_WIDTH, HOLD_BAR_LENGTH, HOLD_BAR_THICKNESS)),
        origin=Origin(xyz=(0.0, HOLD_BAR_LENGTH / 2.0 + 0.0075, 0.0)),
        material=dark_steel,
        name="bar_main",
    )
    hold_down_bar.visual(
        Box((0.024, 0.040, 0.010)),
        origin=Origin(xyz=(0.0, 0.215, -0.007)),
        material=guide_gray,
        name="pressure_pad",
    )
    hold_down_bar.visual(
        Box((0.026, 0.022, 0.008)),
        origin=Origin(xyz=(0.0, 0.005, 0.0070)),
        material=steel,
        name="clip_body",
    )
    hold_down_bar.visual(
        Box((0.026, 0.004, 0.008)),
        origin=Origin(xyz=(0.0, -0.006, 0.0)),
        material=steel,
        name="clip_front_tab",
    )
    hold_down_bar.visual(
        Box((0.026, 0.004, 0.008)),
        origin=Origin(xyz=(0.0, 0.006, 0.0)),
        material=steel,
        name="clip_rear_tab",
    )
    hold_down_bar.inertial = Inertial.from_geometry(
        Box((HOLD_BAR_WIDTH, HOLD_BAR_LENGTH, HOLD_BAR_THICKNESS)),
        mass=0.35,
        origin=Origin(xyz=(0.0, HOLD_BAR_LENGTH / 2.0 + 0.0075, 0.0)),
    )

    model.articulation(
        "base_to_blade_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=blade_arm,
        origin=Origin(xyz=(BLADE_PIVOT_X, BLADE_PIVOT_Y, BLADE_PIVOT_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.5,
            lower=0.0,
            upper=1.25,
        ),
    )
    model.articulation(
        "base_to_hold_down_bar",
        ArticulationType.REVOLUTE,
        parent=base,
        child=hold_down_bar,
        origin=Origin(xyz=(HOLD_PIVOT_X, HOLD_PIVOT_Y, HOLD_PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=1.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    blade_arm = object_model.get_part("blade_arm")
    hold_down_bar = object_model.get_part("hold_down_bar")
    blade_joint = object_model.get_articulation("base_to_blade_arm")
    hold_joint = object_model.get_articulation("base_to_hold_down_bar")

    bed = base.get_visual("bed")
    cut_strip = base.get_visual("cut_strip")
    blade_edge = blade_arm.get_visual("blade_edge")
    handle_grip = blade_arm.get_visual("handle_grip")
    bar_main = hold_down_bar.get_visual("bar_main")
    pressure_pad = hold_down_bar.get_visual("pressure_pad")

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
        "blade pivot axis is horizontal",
        abs(blade_joint.axis[0]) == 1.0 and blade_joint.axis[1] == 0.0 and blade_joint.axis[2] == 0.0,
        details=f"unexpected blade axis {blade_joint.axis}",
    )
    ctx.check(
        "hold-down pivot axis is horizontal",
        hold_joint.axis == (1.0, 0.0, 0.0),
        details=f"unexpected hold-down axis {hold_joint.axis}",
    )

    with ctx.pose({blade_joint: 0.0, hold_joint: 0.0}):
        ctx.expect_contact(
            blade_arm,
            base,
            elem_a=blade_edge,
            elem_b=cut_strip,
            name="blade edge seats on cut strip",
        )
        ctx.expect_contact(
            hold_down_bar,
            base,
            elem_a=pressure_pad,
            elem_b=bed,
            name="hold-down bar presses onto bed",
        )
        ctx.expect_within(
            hold_down_bar,
            base,
            axes="x",
            inner_elem=bar_main,
            outer_elem=bed,
            margin=0.0,
            name="hold-down bar stays over the bed",
        )

    with ctx.pose({blade_joint: 0.0}):
        blade_closed_aabb = ctx.part_element_world_aabb(blade_arm, elem=handle_grip)
    with ctx.pose({blade_joint: 1.05}):
        blade_open_aabb = ctx.part_element_world_aabb(blade_arm, elem=handle_grip)

    blade_closed_top = None if blade_closed_aabb is None else blade_closed_aabb[1][2]
    blade_open_top = None if blade_open_aabb is None else blade_open_aabb[1][2]
    ctx.check(
        "blade arm opens upward",
        blade_closed_top is not None
        and blade_open_top is not None
        and blade_open_top > blade_closed_top + 0.18,
        details=f"closed_top={blade_closed_top}, open_top={blade_open_top}",
    )

    with ctx.pose({hold_joint: 0.0}):
        hold_closed_aabb = ctx.part_element_world_aabb(hold_down_bar, elem=pressure_pad)
    with ctx.pose({hold_joint: 0.75}):
        hold_open_aabb = ctx.part_element_world_aabb(hold_down_bar, elem=pressure_pad)
        ctx.expect_gap(
            hold_down_bar,
            base,
            axis="z",
            min_gap=0.09,
            positive_elem=pressure_pad,
            negative_elem=bed,
            name="raised hold-down bar clears the bed",
        )

    hold_closed_top = None if hold_closed_aabb is None else hold_closed_aabb[1][2]
    hold_open_top = None if hold_open_aabb is None else hold_open_aabb[1][2]
    ctx.check(
        "hold-down bar swings upward",
        hold_closed_top is not None
        and hold_open_top is not None
        and hold_open_top > hold_closed_top + 0.10,
        details=f"closed_top={hold_closed_top}, open_top={hold_open_top}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
