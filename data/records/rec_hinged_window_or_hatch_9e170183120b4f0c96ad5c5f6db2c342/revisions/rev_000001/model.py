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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


CURB_OUTER_X = 1.20
CURB_OUTER_Y = 0.80
CURB_WALL = 0.05
CURB_HEIGHT = 0.18
CURB_FLANGE_OVERHANG = 0.06
CURB_FLANGE_THICKNESS = 0.018

LID_OUTER_X = 1.24
LID_OUTER_Y = 0.84
LID_FRAME_WIDTH = 0.06
LID_FRAME_HEIGHT = 0.045
LID_PANE_THICKNESS = 0.008

HINGE_Y = -CURB_OUTER_Y / 2.0
HINGE_Z = CURB_HEIGHT

LOWER_PIVOT_Y_OFFSET = -0.04
LOWER_PIVOT_Z_OFFSET = -0.02
UPPER_PIVOT_Y = 0.12
UPPER_PIVOT_Z = 0.01

PROP_PIN_RADIUS = 0.008
PROP_PIN_LENGTH = 0.012
PROP_BAR_THICKNESS = 0.008
PROP_BAR_WIDTH = 0.018

PROP_VECTOR_Y = UPPER_PIVOT_Y - LOWER_PIVOT_Y_OFFSET
PROP_VECTOR_Z = UPPER_PIVOT_Z - LOWER_PIVOT_Z_OFFSET
PROP_ARM_LENGTH = math.hypot(PROP_VECTOR_Y, PROP_VECTOR_Z)
PROP_ARM_ANGLE = math.atan2(PROP_VECTOR_Z, PROP_VECTOR_Y)

PROP_PIVOT_X = CURB_OUTER_X / 2.0 + 0.046
LOWER_MOUNT_CENTER_X = CURB_OUTER_X / 2.0 + 0.010
UPPER_MOUNT_CENTER_X = LID_OUTER_X / 2.0 + 0.010

LID_OPEN_TEST_ANGLE = 0.76
_OPEN_UPPER_PIVOT_Y = (
    UPPER_PIVOT_Y * math.cos(LID_OPEN_TEST_ANGLE)
    - UPPER_PIVOT_Z * math.sin(LID_OPEN_TEST_ANGLE)
)
_OPEN_UPPER_PIVOT_Z = (
    UPPER_PIVOT_Y * math.sin(LID_OPEN_TEST_ANGLE)
    + UPPER_PIVOT_Z * math.cos(LID_OPEN_TEST_ANGLE)
)
PROP_OPEN_TEST_ANGLE = (
    math.atan2(
        _OPEN_UPPER_PIVOT_Z - LOWER_PIVOT_Z_OFFSET,
        _OPEN_UPPER_PIVOT_Y - LOWER_PIVOT_Y_OFFSET,
    )
    - PROP_ARM_ANGLE
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="roof_skylight_hatch")

    curb_material = model.material("curb_aluminum", rgba=(0.28, 0.30, 0.32, 1.0))
    frame_material = model.material("lid_frame", rgba=(0.74, 0.75, 0.77, 1.0))
    arm_material = model.material("prop_arm", rgba=(0.60, 0.61, 0.63, 1.0))
    pane_material = model.material("clear_glazing", rgba=(0.78, 0.90, 0.98, 0.35))
    seal_material = model.material("gasket", rgba=(0.10, 0.10, 0.11, 1.0))

    curb = model.part("curb_frame")
    curb.visual(
        Box(
            (
                CURB_OUTER_X + 2.0 * CURB_FLANGE_OVERHANG,
                CURB_OUTER_Y + 2.0 * CURB_FLANGE_OVERHANG,
                CURB_FLANGE_THICKNESS,
            )
        ),
        origin=Origin(xyz=(0.0, 0.0, -CURB_FLANGE_THICKNESS / 2.0)),
        material=curb_material,
        name="base_flange",
    )
    curb.visual(
        Box((CURB_OUTER_X, CURB_WALL, CURB_HEIGHT)),
        origin=Origin(xyz=(0.0, HINGE_Y + CURB_WALL / 2.0, CURB_HEIGHT / 2.0)),
        material=curb_material,
        name="rear_wall",
    )
    curb.visual(
        Box((CURB_OUTER_X, CURB_WALL, CURB_HEIGHT)),
        origin=Origin(
            xyz=(0.0, CURB_OUTER_Y / 2.0 - CURB_WALL / 2.0, CURB_HEIGHT / 2.0)
        ),
        material=curb_material,
        name="front_wall",
    )
    curb.visual(
        Box((CURB_WALL, CURB_OUTER_Y - 2.0 * CURB_WALL, CURB_HEIGHT)),
        origin=Origin(xyz=(-CURB_OUTER_X / 2.0 + CURB_WALL / 2.0, 0.0, CURB_HEIGHT / 2.0)),
        material=curb_material,
        name="left_wall",
    )
    curb.visual(
        Box((CURB_WALL, CURB_OUTER_Y - 2.0 * CURB_WALL, CURB_HEIGHT)),
        origin=Origin(xyz=(CURB_OUTER_X / 2.0 - CURB_WALL / 2.0, 0.0, CURB_HEIGHT / 2.0)),
        material=curb_material,
        name="right_wall",
    )
    curb.visual(
        Box((CURB_OUTER_X - 0.08, 0.028, 0.022)),
        origin=Origin(xyz=(0.0, HINGE_Y + 0.014, CURB_HEIGHT - 0.011)),
        material=seal_material,
        name="hinge_rail",
    )
    for side_name, sign in (("left", -1.0), ("right", 1.0)):
        curb.visual(
            Box((0.06, 0.10, 0.04)),
            origin=Origin(
                xyz=(
                    sign * LOWER_MOUNT_CENTER_X,
                    HINGE_Y - 0.005,
                    HINGE_Z + LOWER_PIVOT_Z_OFFSET,
                )
            ),
            material=curb_material,
            name=f"{side_name}_lower_mount",
        )

    lid = model.part("lid")
    lid.visual(
        Box((LID_OUTER_X, LID_FRAME_WIDTH, LID_FRAME_HEIGHT)),
        origin=Origin(xyz=(0.0, LID_FRAME_WIDTH / 2.0, LID_FRAME_HEIGHT / 2.0)),
        material=frame_material,
        name="rear_rail",
    )
    lid.visual(
        Box((LID_OUTER_X, LID_FRAME_WIDTH, LID_FRAME_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                LID_OUTER_Y - LID_FRAME_WIDTH / 2.0,
                LID_FRAME_HEIGHT / 2.0,
            )
        ),
        material=frame_material,
        name="front_rail",
    )
    lid.visual(
        Box((LID_FRAME_WIDTH, LID_OUTER_Y - 2.0 * LID_FRAME_WIDTH, LID_FRAME_HEIGHT)),
        origin=Origin(
            xyz=(
                -LID_OUTER_X / 2.0 + LID_FRAME_WIDTH / 2.0,
                LID_OUTER_Y / 2.0,
                LID_FRAME_HEIGHT / 2.0,
            )
        ),
        material=frame_material,
        name="left_rail",
    )
    lid.visual(
        Box((LID_FRAME_WIDTH, LID_OUTER_Y - 2.0 * LID_FRAME_WIDTH, LID_FRAME_HEIGHT)),
        origin=Origin(
            xyz=(
                LID_OUTER_X / 2.0 - LID_FRAME_WIDTH / 2.0,
                LID_OUTER_Y / 2.0,
                LID_FRAME_HEIGHT / 2.0,
            )
        ),
        material=frame_material,
        name="right_rail",
    )
    lid.visual(
        Box(
            (
                LID_OUTER_X - 2.0 * LID_FRAME_WIDTH + 0.02,
                LID_OUTER_Y - 2.0 * LID_FRAME_WIDTH + 0.02,
                LID_PANE_THICKNESS,
            )
        ),
        origin=Origin(
            xyz=(
                0.0,
                LID_OUTER_Y / 2.0,
                LID_PANE_THICKNESS / 2.0 + 0.008,
            )
        ),
        material=pane_material,
        name="clear_pane",
    )
    lid.visual(
        Box((LID_OUTER_X - 0.10, 0.028, 0.010)),
        origin=Origin(xyz=(0.0, 0.012, 0.007)),
        material=seal_material,
        name="hinge_leaf",
    )
    lid.visual(
        Box((0.20, 0.03, 0.012)),
        origin=Origin(xyz=(0.0, LID_OUTER_Y - 0.035, 0.020)),
        material=frame_material,
        name="front_grip",
    )
    for side_name, sign in (("left", -1.0), ("right", 1.0)):
        lid.visual(
            Box((0.02, 0.032, 0.03)),
            origin=Origin(
                xyz=(sign * UPPER_MOUNT_CENTER_X, UPPER_PIVOT_Y, 0.015)
            ),
            material=frame_material,
            name=f"{side_name}_upper_mount",
        )

    def add_prop_arm(part_name: str, side_name: str, sign: float) -> None:
        arm = model.part(part_name)
        arm.visual(
            Box((PROP_BAR_THICKNESS, PROP_ARM_LENGTH, PROP_BAR_WIDTH)),
            origin=Origin(
                xyz=(0.0, PROP_VECTOR_Y / 2.0, PROP_VECTOR_Z / 2.0),
                rpy=(PROP_ARM_ANGLE, 0.0, 0.0),
            ),
            material=arm_material,
            name="arm_bar",
        )
        arm.visual(
            Cylinder(radius=PROP_PIN_RADIUS, length=PROP_PIN_LENGTH),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=arm_material,
            name="lower_pin",
        )
        arm.visual(
            Cylinder(radius=PROP_PIN_RADIUS, length=PROP_PIN_LENGTH),
            origin=Origin(
                xyz=(0.0, PROP_VECTOR_Y, PROP_VECTOR_Z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=arm_material,
            name="upper_pin",
        )

        model.articulation(
            f"curb_to_{side_name}_prop_arm",
            ArticulationType.REVOLUTE,
            parent=curb,
            child=arm,
            origin=Origin(
                xyz=(
                    sign * PROP_PIVOT_X,
                    HINGE_Y + LOWER_PIVOT_Y_OFFSET,
                    HINGE_Z + LOWER_PIVOT_Z_OFFSET,
                )
            ),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=15.0,
                velocity=1.2,
                lower=0.0,
                upper=0.9,
            ),
        )

    add_prop_arm("left_prop_arm", "left", -1.0)
    add_prop_arm("right_prop_arm", "right", 1.0)

    model.articulation(
        "curb_to_lid",
        ArticulationType.REVOLUTE,
        parent=curb,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.0,
            lower=0.0,
            upper=1.10,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    curb = object_model.get_part("curb_frame")
    lid = object_model.get_part("lid")
    left_arm = object_model.get_part("left_prop_arm")
    right_arm = object_model.get_part("right_prop_arm")
    lid_hinge = object_model.get_articulation("curb_to_lid")
    left_prop = object_model.get_articulation("curb_to_left_prop_arm")
    right_prop = object_model.get_articulation("curb_to_right_prop_arm")

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
        "all skylight parts are present",
        all(part is not None for part in (curb, lid, left_arm, right_arm)),
        "Expected curb frame, lid, and two prop arms.",
    )
    ctx.check(
        "all articulated pivots use the long horizontal axis",
        lid_hinge.axis == (1.0, 0.0, 0.0)
        and left_prop.axis == (1.0, 0.0, 0.0)
        and right_prop.axis == (1.0, 0.0, 0.0),
        "The curb hinge and both prop pivots should rotate around the long x-axis.",
    )
    ctx.check(
        "skylight motion limits stay in a realistic opening range",
        lid_hinge.motion_limits is not None
        and left_prop.motion_limits is not None
        and right_prop.motion_limits is not None
        and math.isclose(lid_hinge.motion_limits.lower or 0.0, 0.0, abs_tol=1e-9)
        and math.isclose(left_prop.motion_limits.lower or 0.0, 0.0, abs_tol=1e-9)
        and math.isclose(right_prop.motion_limits.lower or 0.0, 0.0, abs_tol=1e-9)
        and (lid_hinge.motion_limits.upper or 0.0) >= 1.0
        and (left_prop.motion_limits.upper or 0.0) >= 0.5
        and (right_prop.motion_limits.upper or 0.0) >= 0.5,
        "The lid should open farther than the short prop arms, all from a closed pose at zero.",
    )

    with ctx.pose({lid_hinge: 0.0, left_prop: 0.0, right_prop: 0.0}):
        ctx.expect_gap(
            lid,
            curb,
            axis="z",
            positive_elem="hinge_leaf",
            negative_elem="hinge_rail",
            max_gap=0.004,
            max_penetration=0.0,
            name="lid stays seated on the curb hinge line when closed",
        )
        ctx.expect_overlap(
            lid,
            curb,
            axes="xy",
            min_overlap=0.75,
            name="closed lid covers the curb opening",
        )
        ctx.expect_overlap(
            left_arm,
            curb,
            axes="yz",
            elem_a="lower_pin",
            elem_b="left_lower_mount",
            min_overlap=0.012,
            name="left prop arm stays aligned with the curb support mount",
        )
        ctx.expect_overlap(
            right_arm,
            curb,
            axes="yz",
            elem_a="lower_pin",
            elem_b="right_lower_mount",
            min_overlap=0.012,
            name="right prop arm stays aligned with the curb support mount",
        )
        ctx.expect_overlap(
            left_arm,
            lid,
            axes="yz",
            elem_a="upper_pin",
            elem_b="left_upper_mount",
            min_overlap=0.012,
            name="left prop arm reaches the lid support point when closed",
        )
        ctx.expect_overlap(
            right_arm,
            lid,
            axes="yz",
            elem_a="upper_pin",
            elem_b="right_upper_mount",
            min_overlap=0.012,
            name="right prop arm reaches the lid support point when closed",
        )

    with ctx.pose(
        {
            lid_hinge: LID_OPEN_TEST_ANGLE,
            left_prop: PROP_OPEN_TEST_ANGLE,
            right_prop: PROP_OPEN_TEST_ANGLE,
        }
    ):
        ctx.expect_gap(
            lid,
            curb,
            axis="z",
            positive_elem="front_rail",
            negative_elem="front_wall",
            min_gap=0.30,
            name="opened lid clears the curb front edge",
        )
        ctx.expect_overlap(
            left_arm,
            lid,
            axes="yz",
            elem_a="upper_pin",
            elem_b="left_upper_mount",
            min_overlap=0.010,
            name="left prop arm remains aligned with the lid when opened",
        )
        ctx.expect_overlap(
            right_arm,
            lid,
            axes="yz",
            elem_a="upper_pin",
            elem_b="right_upper_mount",
            min_overlap=0.010,
            name="right prop arm remains aligned with the lid when opened",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
