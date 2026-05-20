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


BACKPLATE_W = 0.28
BACKPLATE_H = 0.44
BACKPLATE_T = 0.016

ROTARY_CENTER_Y = 0.11
BEAM_W = 0.11
BEAM_D = 0.112
BEAM_H = 0.08
PAD_R = 0.068
PAD_T = 0.018

PLATE_R = 0.09
PLATE_T = 0.016
SADDLE_W = 0.114
SADDLE_D = 0.07
SADDLE_H = 0.04
ARM_T = 0.02
ARM_D = 0.082
ARM_H = 0.13
YOKE_INNER_HALF = 0.071
ARM_CENTER_X = YOKE_INNER_HALF + (ARM_T / 2.0)
REAR_BRIDGE_D = 0.018
REAR_BRIDGE_H = 0.086
REAR_BRIDGE_Y = -0.028
TILT_AXIS_Z = 0.118

FACE_W = 0.13
FACE_H = 0.15
FACE_T = 0.018
FACE_CENTER_Y = 0.065
AXLE_R = 0.012
AXLE_HALF = 0.071
SPINE_W = 0.044
SPINE_D = 0.052
SPINE_H = 0.094
SPINE_CENTER_Y = 0.026
RIB_W = 0.084
RIB_D = 0.022
RIB_H = 0.016
RIB_CENTER_Y = 0.047
RIB_CENTER_Z = 0.045
BRACE_W = 0.018
BRACE_D = 0.08
BRACE_H = 0.11
BRACE_Y = 0.058
BRACE_Z = -0.055


def _aabb_center(bounds: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if bounds is None:
        return None
    mins, maxs = bounds
    return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_backed_rotary_tilt_fixture")

    model.material("powder_charcoal", rgba=(0.22, 0.23, 0.25, 1.0))
    model.material("machined_steel", rgba=(0.63, 0.66, 0.70, 1.0))
    model.material("face_aluminum", rgba=(0.76, 0.78, 0.80, 1.0))

    base_support = model.part("base_support")
    base_support.visual(
        Box((BACKPLATE_W, BACKPLATE_T, BACKPLATE_H)),
        origin=Origin(xyz=(0.0, BACKPLATE_T / 2.0, 0.0)),
        material="powder_charcoal",
        name="backplate",
    )
    base_support.visual(
        Box((BEAM_W, BEAM_D, BEAM_H)),
        origin=Origin(
            xyz=(0.0, BACKPLATE_T + (BEAM_D / 2.0), -(BEAM_H / 2.0)),
        ),
        material="powder_charcoal",
        name="support_beam",
    )
    base_support.visual(
        Cylinder(radius=PAD_R, length=PAD_T),
        origin=Origin(
            xyz=(0.0, ROTARY_CENTER_Y, -(PAD_T / 2.0)),
        ),
        material="machined_steel",
        name="support_pad",
    )
    base_support.visual(
        Box((BRACE_W, BRACE_D, BRACE_H)),
        origin=Origin(xyz=(0.05, BRACE_Y, BRACE_Z)),
        material="powder_charcoal",
        name="right_brace",
    )
    base_support.visual(
        Box((BRACE_W, BRACE_D, BRACE_H)),
        origin=Origin(xyz=(-0.05, BRACE_Y, BRACE_Z)),
        material="powder_charcoal",
        name="left_brace",
    )

    rotary_stage = model.part("rotary_stage")
    rotary_stage.visual(
        Cylinder(radius=PLATE_R, length=PLATE_T),
        origin=Origin(xyz=(0.0, 0.0, PLATE_T / 2.0)),
        material="machined_steel",
        name="rotary_plate",
    )
    rotary_stage.visual(
        Box((SADDLE_W, SADDLE_D, SADDLE_H)),
        origin=Origin(xyz=(0.0, 0.0, PLATE_T + (SADDLE_H / 2.0))),
        material="machined_steel",
        name="saddle_block",
    )
    arm_center_z = PLATE_T + SADDLE_H + (ARM_H / 2.0)
    rotary_stage.visual(
        Box((ARM_T, ARM_D, ARM_H)),
        origin=Origin(xyz=(ARM_CENTER_X, 0.0, arm_center_z)),
        material="machined_steel",
        name="right_yoke_arm",
    )
    rotary_stage.visual(
        Box((ARM_T, ARM_D, ARM_H)),
        origin=Origin(xyz=(-ARM_CENTER_X, 0.0, arm_center_z)),
        material="machined_steel",
        name="left_yoke_arm",
    )
    rotary_stage.visual(
        Box((((2.0 * ARM_CENTER_X) + ARM_T), REAR_BRIDGE_D, REAR_BRIDGE_H)),
        origin=Origin(
            xyz=(0.0, REAR_BRIDGE_Y, PLATE_T + SADDLE_H + (REAR_BRIDGE_H / 2.0)),
        ),
        material="machined_steel",
        name="rear_bridge",
    )

    work_face = model.part("work_face")
    work_face.visual(
        Cylinder(radius=AXLE_R, length=2.0 * AXLE_HALF),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material="face_aluminum",
        name="trunnion_axle",
    )
    work_face.visual(
        Box((SPINE_W, SPINE_D, SPINE_H)),
        origin=Origin(xyz=(0.0, SPINE_CENTER_Y, 0.0)),
        material="machined_steel",
        name="center_spine",
    )
    work_face.visual(
        Box((RIB_W, RIB_D, RIB_H)),
        origin=Origin(xyz=(0.0, RIB_CENTER_Y, RIB_CENTER_Z)),
        material="machined_steel",
        name="upper_rib",
    )
    work_face.visual(
        Box((RIB_W, RIB_D, RIB_H)),
        origin=Origin(xyz=(0.0, RIB_CENTER_Y, -RIB_CENTER_Z)),
        material="machined_steel",
        name="lower_rib",
    )
    work_face.visual(
        Box((FACE_W, FACE_T, FACE_H)),
        origin=Origin(xyz=(0.0, FACE_CENTER_Y, 0.0)),
        material="face_aluminum",
        name="face_panel",
    )

    model.articulation(
        "base_swivel",
        ArticulationType.REVOLUTE,
        parent=base_support,
        child=rotary_stage,
        origin=Origin(xyz=(0.0, ROTARY_CENTER_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.25,
            lower=-2.4,
            upper=2.4,
        ),
    )
    model.articulation(
        "face_tilt",
        ArticulationType.REVOLUTE,
        parent=rotary_stage,
        child=work_face,
        origin=Origin(xyz=(0.0, 0.0, TILT_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.4,
            lower=-0.75,
            upper=0.95,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_support = object_model.get_part("base_support")
    rotary_stage = object_model.get_part("rotary_stage")
    work_face = object_model.get_part("work_face")
    base_swivel = object_model.get_articulation("base_swivel")
    face_tilt = object_model.get_articulation("face_tilt")

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
        "fixture_parts_present",
        all(part is not None for part in (base_support, rotary_stage, work_face)),
        details="Expected base_support, rotary_stage, and work_face parts.",
    )
    ctx.check(
        "base_swivel_axis_is_vertical",
        tuple(base_swivel.axis) == (0.0, 0.0, 1.0),
        details=f"base_swivel axis was {base_swivel.axis}, expected (0, 0, 1).",
    )
    ctx.check(
        "face_tilt_axis_is_horizontal",
        tuple(face_tilt.axis) == (1.0, 0.0, 0.0),
        details=f"face_tilt axis was {face_tilt.axis}, expected (1, 0, 0).",
    )

    with ctx.pose({base_swivel: 0.0, face_tilt: 0.0}):
        ctx.expect_contact(
            rotary_stage,
            base_support,
            contact_tol=1e-5,
            name="rotary_plate_is_seated_on_support",
        )
        ctx.expect_gap(
            rotary_stage,
            base_support,
            axis="z",
            min_gap=0.0,
            max_gap=0.02,
            positive_elem="rotary_plate",
            negative_elem="support_pad",
            name="rotary_plate_has_no_vertical_lift_off",
        )
        ctx.expect_contact(
            work_face,
            rotary_stage,
            contact_tol=1e-5,
            name="work_face_trunnion_is_seated_in_yoke",
        )

        rest_center = _aabb_center(
            ctx.part_element_world_aabb(work_face, elem="face_panel")
        )

    with ctx.pose({base_swivel: 0.7, face_tilt: 0.0}):
        ctx.expect_contact(
            rotary_stage,
            base_support,
            contact_tol=1e-5,
            name="swiveled_plate_stays_on_support",
        )
        swivel_center = _aabb_center(
            ctx.part_element_world_aabb(work_face, elem="face_panel")
        )

    with ctx.pose({base_swivel: 0.0, face_tilt: 0.55}):
        ctx.expect_contact(
            work_face,
            rotary_stage,
            contact_tol=1e-5,
            name="tilted_face_stays_captured_by_yoke",
        )
        tilt_center = _aabb_center(
            ctx.part_element_world_aabb(work_face, elem="face_panel")
        )

    swivel_ok = (
        rest_center is not None
        and swivel_center is not None
        and swivel_center[0] < rest_center[0] - 0.03
    )
    ctx.check(
        "base_swivel_rotates_work_face_about_vertical_axis",
        swivel_ok,
        details=f"rest_center={rest_center}, swivel_center={swivel_center}",
    )

    tilt_ok = (
        rest_center is not None
        and tilt_center is not None
        and tilt_center[2] > rest_center[2] + 0.02
    )
    ctx.check(
        "positive_face_tilt_raises_work_face",
        tilt_ok,
        details=f"rest_center={rest_center}, tilt_center={tilt_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
