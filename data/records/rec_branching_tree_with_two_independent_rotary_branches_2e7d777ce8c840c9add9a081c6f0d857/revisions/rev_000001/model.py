from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 0.34
BASE_WIDTH = 0.24
BASE_THICKNESS = 0.028

MAST_WIDTH = 0.06
MAST_DEPTH = 0.08
MAST_HEIGHT = 0.72

LOWER_HUB_HEIGHT = 0.31
UPPER_HUB_HEIGHT = 0.56
LOWER_HUB_Y = 0.024
UPPER_HUB_Y = -0.024

HUB_REAR = -0.057
HUB_FRONT = 0.003
HUB_DEPTH = HUB_FRONT - HUB_REAR
HUB_WIDTH = 0.056
HUB_HEIGHT = 0.078
HUB_BORE_RADIUS = 0.0072

ARM_PIVOT_RADIUS = 0.014
ARM_PIVOT_THICKNESS = 0.006
ARM_ROOT_LENGTH = 0.022
ARM_ROOT_WIDTH = 0.020
ARM_ROOT_HEIGHT = 0.024
ARM_BEAM_LENGTH = 0.14
ARM_BEAM_WIDTH = 0.016
ARM_BEAM_HEIGHT = 0.018
ARM_TIP_RADIUS = 0.012
ARM_TIP_LENGTH = 0.016


def make_base_mast() -> cq.Workplane:
    base = cq.Workplane("XY").box(
        BASE_LENGTH,
        BASE_WIDTH,
        BASE_THICKNESS,
        centered=(True, True, False),
    )
    mast = (
        cq.Workplane("XY")
        .box(MAST_WIDTH, MAST_DEPTH, MAST_HEIGHT, centered=(True, True, False))
        .translate((0.0, 0.0, BASE_THICKNESS))
    )

    gusset_profile = (
        cq.Workplane("XZ")
        .polyline(
            [
                (MAST_WIDTH / 2.0 - 0.004, BASE_THICKNESS),
                (MAST_WIDTH / 2.0 - 0.004, BASE_THICKNESS + 0.18),
                (0.13, BASE_THICKNESS),
            ]
        )
        .close()
    )
    right_gusset = gusset_profile.extrude(0.012).translate((0.0, -0.038, 0.0))
    left_gusset = right_gusset.mirror("XZ")

    return base.union(mast).union(right_gusset).union(left_gusset)


def make_hub_block(direction: int = 1) -> cq.Workplane:
    body_center_x = 0.5 * (HUB_REAR + HUB_FRONT)
    body = cq.Workplane("XY").box(HUB_DEPTH, HUB_WIDTH, HUB_HEIGHT).translate(
        (body_center_x, 0.0, 0.0)
    )
    bore = cq.Workplane("XZ").circle(HUB_BORE_RADIUS).extrude(
        0.5 * (HUB_WIDTH + 0.01), both=True
    )
    hub = body.cut(bore)
    if direction < 0:
        hub = hub.mirror("YZ")
    return hub


def make_spoke_arm(direction: int = 1) -> cq.Workplane:
    pivot_center_x = HUB_FRONT + 0.5 * ARM_PIVOT_THICKNESS
    root_start_x = HUB_FRONT + ARM_PIVOT_THICKNESS
    root_center_x = root_start_x + 0.5 * ARM_ROOT_LENGTH
    beam_start_x = root_start_x + ARM_ROOT_LENGTH
    beam_center_x = beam_start_x + 0.5 * ARM_BEAM_LENGTH
    tip_center_x = beam_start_x + ARM_BEAM_LENGTH + 0.5 * ARM_TIP_LENGTH - 0.001

    pivot = cq.Workplane("YZ").circle(ARM_PIVOT_RADIUS).extrude(
        0.5 * ARM_PIVOT_THICKNESS, both=True
    ).translate((pivot_center_x, 0.0, 0.0))
    root = cq.Workplane("XY").box(
        ARM_ROOT_LENGTH, ARM_ROOT_WIDTH, ARM_ROOT_HEIGHT
    ).translate((root_center_x, 0.0, 0.0))
    beam = cq.Workplane("XY").box(
        ARM_BEAM_LENGTH, ARM_BEAM_WIDTH, ARM_BEAM_HEIGHT
    ).translate((beam_center_x, 0.0, 0.0))
    tip = cq.Workplane("YZ").circle(ARM_TIP_RADIUS).extrude(
        0.5 * ARM_TIP_LENGTH, both=True
    ).translate((tip_center_x, 0.0, 0.0))

    arm = pivot.union(root).union(beam).union(tip)
    if direction < 0:
        arm = arm.mirror("YZ")
    return arm


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_branch_motion_rig")

    dark_frame = model.material("dark_frame", rgba=(0.22, 0.24, 0.27, 1.0))
    hub_finish = model.material("hub_finish", rgba=(0.52, 0.56, 0.62, 1.0))
    arm_finish = model.material("arm_finish", rgba=(0.84, 0.50, 0.18, 1.0))

    mast = model.part("mast")
    mast.visual(
        mesh_from_cadquery(make_base_mast(), "mast_frame"),
        material=dark_frame,
        name="frame_shell",
    )
    lower_hub_x = MAST_WIDTH / 2.0 - HUB_REAR
    upper_hub_x = -(MAST_WIDTH / 2.0 - HUB_REAR)
    mast.visual(
        mesh_from_cadquery(make_hub_block(1), "lower_hub_block"),
        origin=Origin(xyz=(lower_hub_x, LOWER_HUB_Y, LOWER_HUB_HEIGHT)),
        material=hub_finish,
        name="lower_hub_shell",
    )
    mast.visual(
        mesh_from_cadquery(make_hub_block(-1), "upper_hub_block"),
        origin=Origin(xyz=(upper_hub_x, UPPER_HUB_Y, UPPER_HUB_HEIGHT)),
        material=hub_finish,
        name="upper_hub_shell",
    )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        mesh_from_cadquery(make_spoke_arm(1), "lower_spoke_arm"),
        material=arm_finish,
        name="arm_shell",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        mesh_from_cadquery(make_spoke_arm(-1), "upper_spoke_arm"),
        material=arm_finish,
        name="arm_shell",
    )

    model.articulation(
        "mast_to_lower_arm",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=lower_arm,
        origin=Origin(xyz=(lower_hub_x, LOWER_HUB_Y, LOWER_HUB_HEIGHT)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=-0.65,
            upper=1.20,
        ),
    )
    model.articulation(
        "mast_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=upper_arm,
        origin=Origin(xyz=(upper_hub_x, UPPER_HUB_Y, UPPER_HUB_HEIGHT)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=-0.55,
            upper=1.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    lower_hub = mast.get_visual("lower_hub_shell")
    upper_hub = mast.get_visual("upper_hub_shell")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    lower_joint = object_model.get_articulation("mast_to_lower_arm")
    upper_joint = object_model.get_articulation("mast_to_upper_arm")

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
        lower_arm,
        mast,
        elem_b=lower_hub,
        name="lower arm is carried by lower hub block",
    )
    ctx.expect_contact(
        upper_arm,
        mast,
        elem_b=upper_hub,
        name="upper arm is carried by upper hub block",
    )

    ctx.expect_origin_gap(
        lower_arm,
        mast,
        axis="x",
        min_gap=0.07,
        name="lower arm sits on positive x side of mast",
    )
    ctx.expect_origin_gap(
        mast,
        upper_arm,
        axis="x",
        min_gap=0.07,
        name="upper arm sits on negative x side of mast",
    )

    ctx.check(
        "lower arm joint axis is horizontal",
        tuple(round(v, 6) for v in lower_joint.axis) == (0.0, -1.0, 0.0),
        details=f"axis={lower_joint.axis}",
    )
    ctx.check(
        "upper arm joint axis is horizontal",
        tuple(round(v, 6) for v in upper_joint.axis) == (0.0, 1.0, 0.0),
        details=f"axis={upper_joint.axis}",
    )

    def center_z(part) -> float | None:
        aabb = ctx.part_world_aabb(part)
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    def center_x(part) -> float | None:
        aabb = ctx.part_world_aabb(part)
        if aabb is None:
            return None
        return 0.5 * (aabb[0][0] + aabb[1][0])

    with ctx.pose({lower_joint: 0.0, upper_joint: 0.0}):
        lower_rest_z = center_z(lower_arm)
        upper_rest_z = center_z(upper_arm)
        lower_rest_x = center_x(lower_arm)
        upper_rest_x = center_x(upper_arm)

    with ctx.pose({lower_joint: 0.9, upper_joint: 0.0}):
        lower_lifted_z = center_z(lower_arm)
        upper_still_z = center_z(upper_arm)

    with ctx.pose({lower_joint: 0.0, upper_joint: 0.9}):
        lower_still_z = center_z(lower_arm)
        upper_lifted_z = center_z(upper_arm)
        lower_still_x = center_x(lower_arm)
        upper_lifted_x = center_x(upper_arm)

    ctx.check(
        "lower arm lifts for positive rotation",
        lower_rest_z is not None
        and lower_lifted_z is not None
        and lower_lifted_z > lower_rest_z + 0.035,
        details=f"rest_z={lower_rest_z}, lifted_z={lower_lifted_z}",
    )
    ctx.check(
        "upper arm lifts for positive rotation",
        upper_rest_z is not None
        and upper_lifted_z is not None
        and upper_lifted_z > upper_rest_z + 0.035,
        details=f"rest_z={upper_rest_z}, lifted_z={upper_lifted_z}",
    )
    ctx.check(
        "upper arm stays put when lower arm moves",
        upper_rest_z is not None
        and upper_still_z is not None
        and abs(upper_still_z - upper_rest_z) < 0.003,
        details=f"rest_z={upper_rest_z}, moved_z={upper_still_z}",
    )
    ctx.check(
        "lower arm stays put when upper arm moves",
        lower_rest_z is not None
        and lower_still_z is not None
        and abs(lower_still_z - lower_rest_z) < 0.003,
        details=f"rest_z={lower_rest_z}, moved_z={lower_still_z}",
    )
    ctx.check(
        "arms remain on opposite sides after articulation",
        lower_rest_x is not None
        and upper_lifted_x is not None
        and lower_still_x is not None
        and upper_rest_x is not None
        and lower_rest_x > 0.1
        and lower_still_x > 0.1
        and upper_rest_x < -0.1
        and upper_lifted_x < -0.1,
        details=(
            f"lower_rest_x={lower_rest_x}, lower_still_x={lower_still_x}, "
            f"upper_rest_x={upper_rest_x}, upper_lifted_x={upper_lifted_x}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
