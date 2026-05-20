from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


ROOT_BASE_LENGTH = 0.060
ROOT_BASE_WIDTH = 0.034
ROOT_BASE_HEIGHT = 0.014
ROOT_BASE_CENTER = (-0.018, 0.0, -0.023)

CLEVIS_GAP = 0.020
EAR_THICKNESS = 0.005
EAR_LENGTH = 0.028
EAR_HEIGHT = 0.032
EAR_CENTER_X = -0.004
EAR_CENTER_Z = 0.004
EAR_CENTER_Y = 0.5 * (CLEVIS_GAP + EAR_THICKNESS)

ARM_BOSS_RADIUS = 0.009
ARM_BOSS_LENGTH = CLEVIS_GAP
ARM_SHOULDER_LENGTH = 0.020
ARM_SHOULDER_WIDTH = 0.016
ARM_SHOULDER_HEIGHT = 0.016
ARM_SHOULDER_CENTER = (0.010, 0.0, 0.008)
ARM_BEAM_LENGTH = 0.086
ARM_BEAM_WIDTH = 0.016
ARM_BEAM_HEIGHT = 0.014
ARM_BEAM_CENTER_X = 0.063
ARM_BEAM_CENTER_Z = 0.010

GUIDE_LENGTH = 0.068
GUIDE_WIDTH = 0.020
GUIDE_FLOOR_HEIGHT = 0.004
GUIDE_RAIL_THICKNESS = 0.003
GUIDE_RAIL_HEIGHT = 0.012
GUIDE_CENTER_X = 0.140
SLIDER_JOINT_X = GUIDE_CENTER_X - GUIDE_LENGTH / 2.0 + 0.004

SLIDER_BAR_LENGTH = 0.042
SLIDER_BAR_WIDTH = 0.013
SLIDER_BAR_HEIGHT = 0.010
SLIDER_BAR_CENTER_X = SLIDER_BAR_LENGTH / 2.0
SLIDER_BAR_CENTER_Z = 0.009
SLIDER_MAX_EXTENSION = 0.040


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hinged_telescoping_arm")

    root_material = model.material("root_graphite", rgba=(0.20, 0.20, 0.22, 1.0))
    arm_material = model.material("arm_aluminum", rgba=(0.72, 0.74, 0.76, 1.0))
    slider_material = model.material("slider_black", rgba=(0.14, 0.14, 0.15, 1.0))

    root = model.part("root_clevis")
    root.visual(
        Box((ROOT_BASE_LENGTH, ROOT_BASE_WIDTH, ROOT_BASE_HEIGHT)),
        origin=Origin(xyz=ROOT_BASE_CENTER),
        material=root_material,
        name="base_block",
    )
    root.visual(
        Box((EAR_LENGTH, EAR_THICKNESS, EAR_HEIGHT)),
        origin=Origin(xyz=(EAR_CENTER_X, EAR_CENTER_Y, EAR_CENTER_Z)),
        material=root_material,
        name="left_ear",
    )
    root.visual(
        Box((EAR_LENGTH, EAR_THICKNESS, EAR_HEIGHT)),
        origin=Origin(xyz=(EAR_CENTER_X, -EAR_CENTER_Y, EAR_CENTER_Z)),
        material=root_material,
        name="right_ear",
    )
    root.visual(
        Box((0.010, CLEVIS_GAP, 0.018)),
        origin=Origin(xyz=(-0.020, 0.0, -0.007)),
        material=root_material,
        name="rear_web",
    )

    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=ARM_BOSS_RADIUS, length=ARM_BOSS_LENGTH),
        origin=Origin(rpy=(1.5707963267948966, 0.0, 0.0)),
        material=arm_material,
        name="pivot_boss",
    )
    arm.visual(
        Box((ARM_SHOULDER_LENGTH, ARM_SHOULDER_WIDTH, ARM_SHOULDER_HEIGHT)),
        origin=Origin(xyz=ARM_SHOULDER_CENTER),
        material=arm_material,
        name="shoulder_block",
    )
    arm.visual(
        Box((ARM_BEAM_LENGTH, ARM_BEAM_WIDTH, ARM_BEAM_HEIGHT)),
        origin=Origin(xyz=(ARM_BEAM_CENTER_X, 0.0, ARM_BEAM_CENTER_Z)),
        material=arm_material,
        name="beam",
    )
    arm.visual(
        Box((GUIDE_LENGTH, GUIDE_WIDTH, GUIDE_FLOOR_HEIGHT)),
        origin=Origin(xyz=(GUIDE_CENTER_X, 0.0, GUIDE_FLOOR_HEIGHT / 2.0)),
        material=arm_material,
        name="guide_floor",
    )
    rail_center_y = GUIDE_WIDTH / 2.0 - GUIDE_RAIL_THICKNESS / 2.0
    rail_center_z = GUIDE_FLOOR_HEIGHT + GUIDE_RAIL_HEIGHT / 2.0
    arm.visual(
        Box((GUIDE_LENGTH, GUIDE_RAIL_THICKNESS, GUIDE_RAIL_HEIGHT)),
        origin=Origin(xyz=(GUIDE_CENTER_X, rail_center_y, rail_center_z)),
        material=arm_material,
        name="left_rail",
    )
    arm.visual(
        Box((GUIDE_LENGTH, GUIDE_RAIL_THICKNESS, GUIDE_RAIL_HEIGHT)),
        origin=Origin(xyz=(GUIDE_CENTER_X, -rail_center_y, rail_center_z)),
        material=arm_material,
        name="right_rail",
    )

    slider = model.part("slider")
    slider.visual(
        Box((SLIDER_BAR_LENGTH, SLIDER_BAR_WIDTH, SLIDER_BAR_HEIGHT)),
        origin=Origin(xyz=(SLIDER_BAR_CENTER_X, 0.0, SLIDER_BAR_CENTER_Z)),
        material=slider_material,
        name="slider_body",
    )

    model.articulation(
        "root_to_arm",
        ArticulationType.REVOLUTE,
        parent=root,
        child=arm,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=2.5,
            lower=0.0,
            upper=1.15,
        ),
    )

    model.articulation(
        "arm_to_slider",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=slider,
        origin=Origin(xyz=(SLIDER_JOINT_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.18,
            lower=0.0,
            upper=SLIDER_MAX_EXTENSION,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root = object_model.get_part("root_clevis")
    arm = object_model.get_part("arm")
    slider = object_model.get_part("slider")
    shoulder = object_model.get_articulation("root_to_arm")
    extension = object_model.get_articulation("arm_to_slider")

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

    ctx.expect_origin_distance(
        arm,
        root,
        axes="xyz",
        max_dist=0.0001,
        name="arm hinge origin coincides with root clevis axis",
    )
    ctx.expect_contact(
        arm,
        root,
        contact_tol=1e-6,
        name="arm boss touches clevis ears",
    )
    ctx.expect_contact(
        slider,
        arm,
        contact_tol=1e-6,
        elem_b="guide_floor",
        name="slider is supported by guide floor",
    )
    ctx.expect_within(
        slider,
        arm,
        axes="yz",
        margin=0.001,
        name="slider stays laterally within arm envelope",
    )

    with ctx.pose({shoulder: 0.0, extension: 0.0}):
        ctx.expect_overlap(
            slider,
            arm,
            axes="x",
            min_overlap=0.04,
            name="closed slider remains substantially nested in arm",
        )
        closed_arm_top = ctx.part_world_aabb(arm)[1][2]
        closed_slider_x = ctx.part_world_position(slider)[0]

    with ctx.pose({shoulder: shoulder.motion_limits.upper, extension: 0.0}):
        opened_arm_top = ctx.part_world_aabb(arm)[1][2]

    ctx.check(
        "arm opens upward",
        opened_arm_top > closed_arm_top + 0.09,
        details=(
            f"expected opened arm to rise noticeably; closed_top={closed_arm_top:.4f}, "
            f"opened_top={opened_arm_top:.4f}"
        ),
    )

    with ctx.pose({shoulder: 0.0, extension: extension.motion_limits.upper}):
        ctx.expect_within(
            slider,
            arm,
            axes="yz",
            margin=0.001,
            name="extended slider stays aligned in guide",
        )
        ctx.expect_overlap(
            slider,
            arm,
            axes="x",
            min_overlap=0.018,
            name="extended slider keeps guide engagement",
        )
        extended_slider_x = ctx.part_world_position(slider)[0]

    ctx.check(
        "slider extends outward along arm",
        extended_slider_x > closed_slider_x + 0.03,
        details=(
            f"expected slider origin to move outward along +X; "
            f"closed_x={closed_slider_x:.4f}, extended_x={extended_slider_x:.4f}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
