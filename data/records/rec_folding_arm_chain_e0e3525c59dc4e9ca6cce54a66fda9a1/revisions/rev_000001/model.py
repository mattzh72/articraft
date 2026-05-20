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


FOOT_LENGTH = 0.18
FOOT_WIDTH = 0.10
FOOT_THICKNESS = 0.012
SHOULDER_AXIS_Z = 0.102

JOINT_RADIUS = 0.012
HINGE_GAP = 0.014
SIDE_PLATE_THICKNESS = 0.004
OUTER_HINGE_WIDTH = HINGE_GAP + 2.0 * SIDE_PLATE_THICKNESS
PLATE_LENGTH = 0.020
PLATE_HEIGHT = 0.032
PLATE_Y = 0.5 * HINGE_GAP + 0.5 * SIDE_PLATE_THICKNESS

LINK_BEAM_WIDTH = 0.010
LINK_BEAM_THICKNESS = 0.012

LINK_1_LENGTH = 0.18
LINK_2_LENGTH = 0.165
LINK_3_LENGTH = 0.13

END_PAD_LENGTH = 0.048
END_PAD_WIDTH = 0.042
END_PAD_BASE_THICKNESS = 0.009
END_PAD_CAP_THICKNESS = 0.003


def _y_cylinder_origin(center: tuple[float, float, float]) -> Origin:
    return Origin(xyz=center, rpy=(pi / 2.0, 0.0, 0.0))


def _add_side_plates(part, *, axis_x: float, axis_z: float, prefix: str, material: str) -> None:
    part.visual(
        Box((PLATE_LENGTH, SIDE_PLATE_THICKNESS, PLATE_HEIGHT)),
        origin=Origin(xyz=(axis_x - 0.006, PLATE_Y, axis_z)),
        material=material,
        name=f"{prefix}_left_plate",
    )
    part.visual(
        Box((PLATE_LENGTH, SIDE_PLATE_THICKNESS, PLATE_HEIGHT)),
        origin=Origin(xyz=(axis_x - 0.006, -PLATE_Y, axis_z)),
        material=material,
        name=f"{prefix}_right_plate",
    )


def _add_link_visuals(model: ArticulatedObject, name: str, length: float) -> None:
    link = model.part(name)
    link.visual(
        Cylinder(radius=JOINT_RADIUS, length=HINGE_GAP),
        origin=_y_cylinder_origin((0.0, 0.0, 0.0)),
        material="link_silver",
        name="proximal_barrel",
    )
    link.visual(
        Box((0.032, LINK_BEAM_WIDTH, 0.016)),
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
        material="link_silver",
        name="root_cheek",
    )
    beam_start = 0.028
    bridge_start = length - 0.041
    link.visual(
        Box((bridge_start - beam_start, LINK_BEAM_WIDTH, LINK_BEAM_THICKNESS)),
        origin=Origin(xyz=(0.5 * (beam_start + bridge_start), 0.0, 0.0)),
        material="link_silver",
        name="beam",
    )
    link.visual(
        Box((0.028, OUTER_HINGE_WIDTH, 0.012)),
        origin=Origin(xyz=(length - 0.027, 0.0, -0.006)),
        material="link_silver",
        name="distal_bridge",
    )
    _add_side_plates(link, axis_x=length, axis_z=0.0, prefix="distal", material="plate_clear")


def _add_terminal_link_visuals(model: ArticulatedObject) -> None:
    link = model.part("link_3")
    link.visual(
        Cylinder(radius=JOINT_RADIUS, length=HINGE_GAP),
        origin=_y_cylinder_origin((0.0, 0.0, 0.0)),
        material="link_silver",
        name="proximal_barrel",
    )
    link.visual(
        Box((0.032, LINK_BEAM_WIDTH, 0.016)),
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
        material="link_silver",
        name="root_cheek",
    )
    link.visual(
        Box((LINK_3_LENGTH - 0.024, LINK_BEAM_WIDTH, LINK_BEAM_THICKNESS)),
        origin=Origin(xyz=(0.5 * (0.028 + LINK_3_LENGTH - 0.004), 0.0, 0.0)),
        material="link_silver",
        name="beam",
    )
    link.visual(
        Box((0.026, 0.018, 0.014)),
        origin=Origin(xyz=(LINK_3_LENGTH + 0.003, 0.0, 0.0)),
        material="link_silver",
        name="pad_neck",
    )
    link.visual(
        Box((END_PAD_LENGTH, END_PAD_WIDTH, END_PAD_BASE_THICKNESS)),
        origin=Origin(xyz=(LINK_3_LENGTH + 0.5 * END_PAD_LENGTH + 0.010, 0.0, -0.001)),
        material="link_silver",
        name="pad_body",
    )
    link.visual(
        Box((0.040, 0.032, END_PAD_CAP_THICKNESS)),
        origin=Origin(xyz=(LINK_3_LENGTH + 0.5 * END_PAD_LENGTH + 0.010, 0.0, 0.003)),
        material="pad_black",
        name="end_pad",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_top_folding_arm_chain")

    model.material("base_gray", rgba=(0.17, 0.18, 0.20, 1.0))
    model.material("link_silver", rgba=(0.74, 0.76, 0.79, 1.0))
    model.material("plate_clear", rgba=(0.80, 0.86, 0.92, 0.55))
    model.material("pad_black", rgba=(0.08, 0.09, 0.10, 1.0))

    base = model.part("base_foot")
    base.visual(
        Box((FOOT_LENGTH, FOOT_WIDTH, FOOT_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, 0.5 * FOOT_THICKNESS)),
        material="base_gray",
        name="foot_pad",
    )
    base.visual(
        Box((0.074, 0.058, 0.068)),
        origin=Origin(xyz=(-0.044, 0.0, 0.040)),
        material="base_gray",
        name="rear_spine",
    )
    base.visual(
        Box((0.022, OUTER_HINGE_WIDTH, 0.080)),
        origin=Origin(xyz=(-0.016, 0.0, 0.046)),
        material="base_gray",
        name="front_upright",
    )
    base.visual(
        Box((0.028, OUTER_HINGE_WIDTH, 0.018)),
        origin=Origin(xyz=(-0.014, 0.0, 0.080)),
        material="base_gray",
        name="shoulder_saddle",
    )
    _add_side_plates(base, axis_x=0.0, axis_z=SHOULDER_AXIS_Z, prefix="shoulder", material="plate_clear")

    _add_link_visuals(model, "link_1", LINK_1_LENGTH)
    _add_link_visuals(model, "link_2", LINK_2_LENGTH)
    _add_terminal_link_visuals(model)

    model.articulation(
        "base_to_link_1",
        ArticulationType.REVOLUTE,
        parent=base,
        child="link_1",
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=0.0, upper=1.45),
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent="link_1",
        child="link_2",
        origin=Origin(xyz=(LINK_1_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=1.8, lower=0.0, upper=1.55),
    )
    model.articulation(
        "link_2_to_link_3",
        ArticulationType.REVOLUTE,
        parent="link_2",
        child="link_3",
        origin=Origin(xyz=(LINK_2_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=0.0, upper=1.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_foot")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    link_3 = object_model.get_part("link_3")

    shoulder = object_model.get_articulation("base_to_link_1")
    elbow = object_model.get_articulation("link_1_to_link_2")
    wrist = object_model.get_articulation("link_2_to_link_3")

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
        "all_joint_axes_match",
        shoulder.axis == elbow.axis == wrist.axis == (0.0, -1.0, 0.0),
        details=f"axes={shoulder.axis}, {elbow.axis}, {wrist.axis}",
    )

    ctx.expect_contact(base, link_1, name="base_supports_link_1")
    ctx.expect_contact(link_1, link_2, name="link_1_supports_link_2")
    ctx.expect_contact(link_2, link_3, name="link_2_supports_link_3")

    with ctx.pose({shoulder: 0.55, elbow: 0.70, wrist: 0.45}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_in_raised_pose")
        ctx.expect_contact(base, link_1, name="shoulder_contact_in_raised_pose")
        ctx.expect_contact(link_1, link_2, name="elbow_contact_in_raised_pose")
        ctx.expect_contact(link_2, link_3, name="wrist_contact_in_raised_pose")
        ctx.expect_gap(link_3, base, axis="z", min_gap=0.04, name="end_pad_lifts_clear_of_base")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
