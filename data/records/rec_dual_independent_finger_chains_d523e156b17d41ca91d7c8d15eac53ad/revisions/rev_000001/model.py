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


PALM_THICKNESS = 0.024
PALM_WIDTH = 0.160
PALM_HEIGHT = 0.190
ROOT_ORIGIN_X = 0.044
FINGER_CENTER_Y = 0.022
ROOT_LINK_LENGTH = 0.056
MID_LINK_LENGTH = 0.046
INNER_OFFSET = 0.006


def _add_box(part, size, xyz, *, material: str, name: str) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _add_y_cylinder(
    part,
    radius: float,
    length: float,
    xyz,
    *,
    material: str,
    name: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _build_palm(model: ArticulatedObject):
    palm = model.part("palm")
    material = "palm_dark"

    _add_box(
        palm,
        (PALM_THICKNESS, PALM_WIDTH, PALM_HEIGHT),
        (PALM_THICKNESS / 2.0, 0.0, 0.0),
        material=material,
        name="rear_plate",
    )
    _add_box(
        palm,
        (0.020, 0.072, 0.014),
        (0.034, 0.0, 0.054),
        material=material,
        name="top_bridge",
    )
    _add_box(
        palm,
        (0.020, 0.072, 0.014),
        (0.034, 0.0, -0.054),
        material=material,
        name="bottom_bridge",
    )
    _add_box(
        palm,
        (0.018, 0.020, 0.108),
        (0.033, 0.0, 0.0),
        material=material,
        name="center_rib",
    )
    _add_box(
        palm,
        (0.020, 0.012, 0.094),
        (0.034, 0.028, 0.0),
        material=material,
        name="left_tower_outer",
    )
    _add_box(
        palm,
        (0.020, 0.012, 0.094),
        (0.034, -0.028, 0.0),
        material=material,
        name="right_tower_outer",
    )
    _add_box(
        palm,
        (0.018, 0.020, 0.018),
        (0.031, 0.020, 0.0),
        material=material,
        name="left_tower_web",
    )
    _add_box(
        palm,
        (0.018, 0.020, 0.018),
        (0.031, -0.020, 0.0),
        material=material,
        name="right_tower_web",
    )

    return palm


def _build_root_link(model: ArticulatedObject, name: str, side_sign: float):
    part = model.part(name)
    material = "finger_metal"
    off = side_sign * INNER_OFFSET

    _add_box(
        part,
        (0.016, 0.012, 0.036),
        (0.008, off, 0.0),
        material=material,
        name="rear_module",
    )
    _add_box(
        part,
        (0.028, 0.010, 0.026),
        (0.026, off, 0.0),
        material=material,
        name="main_beam",
    )
    _add_box(
        part,
        (0.020, 0.012, 0.010),
        (0.021, off, 0.012),
        material=material,
        name="upper_saddle",
    )
    _add_box(
        part,
        (0.016, 0.011, 0.008),
        (0.021, off, -0.012),
        material=material,
        name="lower_saddle",
    )
    _add_box(
        part,
        (0.012, 0.010, 0.022),
        (ROOT_LINK_LENGTH - 0.006, off, 0.0),
        material=material,
        name="front_housing",
    )
    _add_box(
        part,
        (0.010, 0.010, 0.018),
        (0.043, off, 0.0),
        material=material,
        name="front_bridge",
    )
    _add_y_cylinder(
        part,
        0.009,
        0.004,
        (0.008, side_sign * 0.010, 0.0),
        material=material,
        name="rear_bearing_cap",
    )
    _add_y_cylinder(
        part,
        0.007,
        0.004,
        (ROOT_LINK_LENGTH - 0.007, off, 0.0),
        material=material,
        name="front_pin_boss",
    )
    return part


def _build_middle_link(model: ArticulatedObject, name: str, side_sign: float):
    part = model.part(name)
    material = "finger_metal"
    off = side_sign * INNER_OFFSET

    _add_box(
        part,
        (0.012, 0.010, 0.028),
        (0.006, off, 0.0),
        material=material,
        name="rear_module",
    )
    _add_box(
        part,
        (0.022, 0.009, 0.020),
        (0.022, off, 0.0),
        material=material,
        name="beam",
    )
    _add_box(
        part,
        (0.014, 0.010, 0.008),
        (0.019, off, 0.009),
        material=material,
        name="upper_rib",
    )
    _add_box(
        part,
        (0.012, 0.009, 0.006),
        (0.020, off, -0.009),
        material=material,
        name="lower_rib",
    )
    _add_box(
        part,
        (0.010, 0.009, 0.018),
        (MID_LINK_LENGTH - 0.005, off, 0.0),
        material=material,
        name="front_housing",
    )
    _add_box(
        part,
        (0.008, 0.009, 0.014),
        (0.036, off, 0.0),
        material=material,
        name="front_bridge",
    )
    _add_y_cylinder(
        part,
        0.007,
        0.004,
        (0.006, off, 0.0),
        material=material,
        name="rear_spacer",
    )
    _add_y_cylinder(
        part,
        0.0055,
        0.004,
        (MID_LINK_LENGTH - 0.0055, off, 0.0),
        material=material,
        name="front_pin_boss",
    )
    return part


def _build_left_tip(model: ArticulatedObject, name: str, side_sign: float):
    part = model.part(name)
    material = "finger_metal"
    off = side_sign * INNER_OFFSET

    _add_box(
        part,
        (0.010, 0.010, 0.022),
        (0.005, off, 0.0),
        material=material,
        name="rear_module",
    )
    _add_box(
        part,
        (0.020, 0.008, 0.017),
        (0.018, off, 0.0),
        material=material,
        name="distal_beam",
    )
    _add_box(
        part,
        (0.016, 0.012, 0.012),
        (0.037, off - 0.002, 0.0),
        material=material,
        name="paddle_tip",
    )
    _add_box(
        part,
        (0.012, 0.010, 0.014),
        (0.030, off - 0.001, 0.0),
        material=material,
        name="tip_bridge",
    )
    _add_box(
        part,
        (0.010, 0.014, 0.008),
        (0.050, off - 0.003, 0.0),
        material=material,
        name="paddle_nose",
    )
    _add_box(
        part,
        (0.008, 0.012, 0.010),
        (0.043, off - 0.0025, 0.0),
        material=material,
        name="nose_bridge",
    )
    _add_y_cylinder(
        part,
        0.006,
        0.004,
        (0.005, off, 0.0),
        material=material,
        name="rear_spacer",
    )
    return part


def _build_right_tip(model: ArticulatedObject, name: str, side_sign: float):
    part = model.part(name)
    material = "finger_metal"
    off = side_sign * INNER_OFFSET

    _add_box(
        part,
        (0.010, 0.010, 0.022),
        (0.005, off, 0.0),
        material=material,
        name="rear_module",
    )
    _add_box(
        part,
        (0.020, 0.008, 0.016),
        (0.018, off, 0.0),
        material=material,
        name="distal_beam",
    )
    _add_box(
        part,
        (0.016, 0.008, 0.010),
        (0.037, off + 0.001, 0.001),
        material=material,
        name="beak_body",
    )
    _add_box(
        part,
        (0.012, 0.008, 0.012),
        (0.030, off + 0.0005, 0.0005),
        material=material,
        name="beak_bridge",
    )
    _add_box(
        part,
        (0.010, 0.005, 0.008),
        (0.048, off + 0.004, -0.003),
        material=material,
        name="beak_nose",
    )
    _add_box(
        part,
        (0.010, 0.007, 0.006),
        (0.041, off + 0.002, -0.006),
        material=material,
        name="beak_hook",
    )
    _add_y_cylinder(
        part,
        0.006,
        0.004,
        (0.005, off, 0.0),
        material=material,
        name="rear_spacer",
    )
    return part


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_gripper_hand_study")

    model.material("palm_dark", rgba=(0.20, 0.22, 0.25, 1.0))
    model.material("finger_metal", rgba=(0.73, 0.75, 0.78, 1.0))

    palm = _build_palm(model)
    left_root = _build_root_link(model, "left_root", side_sign=-1.0)
    left_middle = _build_middle_link(model, "left_middle", side_sign=-1.0)
    left_distal = _build_left_tip(model, "left_distal", side_sign=-1.0)
    right_root = _build_root_link(model, "right_root", side_sign=1.0)
    right_middle = _build_middle_link(model, "right_middle", side_sign=1.0)
    right_distal = _build_right_tip(model, "right_distal", side_sign=1.0)

    model.articulation(
        "palm_to_left_root",
        ArticulationType.REVOLUTE,
        parent=palm,
        child=left_root,
        origin=Origin(xyz=(ROOT_ORIGIN_X, FINGER_CENTER_Y, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=0.0,
            upper=0.40,
        ),
    )
    model.articulation(
        "left_root_to_left_middle",
        ArticulationType.REVOLUTE,
        parent=left_root,
        child=left_middle,
        origin=Origin(xyz=(ROOT_LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=0.0,
            upper=0.60,
        ),
    )
    model.articulation(
        "left_middle_to_left_distal",
        ArticulationType.REVOLUTE,
        parent=left_middle,
        child=left_distal,
        origin=Origin(xyz=(MID_LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=3.0,
            lower=0.0,
            upper=0.70,
        ),
    )
    model.articulation(
        "palm_to_right_root",
        ArticulationType.REVOLUTE,
        parent=palm,
        child=right_root,
        origin=Origin(xyz=(ROOT_ORIGIN_X, -FINGER_CENTER_Y, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=0.0,
            upper=0.40,
        ),
    )
    model.articulation(
        "right_root_to_right_middle",
        ArticulationType.REVOLUTE,
        parent=right_root,
        child=right_middle,
        origin=Origin(xyz=(ROOT_LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=0.0,
            upper=0.60,
        ),
    )
    model.articulation(
        "right_middle_to_right_distal",
        ArticulationType.REVOLUTE,
        parent=right_middle,
        child=right_distal,
        origin=Origin(xyz=(MID_LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=3.0,
            lower=0.0,
            upper=0.70,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    palm = object_model.get_part("palm")
    left_root = object_model.get_part("left_root")
    left_middle = object_model.get_part("left_middle")
    left_distal = object_model.get_part("left_distal")
    right_root = object_model.get_part("right_root")
    right_middle = object_model.get_part("right_middle")
    right_distal = object_model.get_part("right_distal")

    palm_to_left_root = object_model.get_articulation("palm_to_left_root")
    left_root_to_left_middle = object_model.get_articulation("left_root_to_left_middle")
    left_middle_to_left_distal = object_model.get_articulation(
        "left_middle_to_left_distal"
    )
    palm_to_right_root = object_model.get_articulation("palm_to_right_root")
    right_root_to_right_middle = object_model.get_articulation(
        "right_root_to_right_middle"
    )
    right_middle_to_right_distal = object_model.get_articulation(
        "right_middle_to_right_distal"
    )

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

    all_joints = (
        palm_to_left_root,
        left_root_to_left_middle,
        left_middle_to_left_distal,
        palm_to_right_root,
        right_root_to_right_middle,
        right_middle_to_right_distal,
    )
    ctx.check(
        "six revolute joints present",
        len(all_joints) == 6
        and all(joint.articulation_type == ArticulationType.REVOLUTE for joint in all_joints),
        "Expected exactly six revolute joints across the two independent finger chains.",
    )
    ctx.check(
        "joint axes bend inward from opposite towers",
        palm_to_left_root.axis == (0.0, 1.0, 0.0)
        and left_root_to_left_middle.axis == (0.0, 1.0, 0.0)
        and left_middle_to_left_distal.axis == (0.0, 1.0, 0.0)
        and palm_to_right_root.axis == (0.0, 1.0, 0.0)
        and right_root_to_right_middle.axis == (0.0, 1.0, 0.0)
        and right_middle_to_right_distal.axis == (0.0, 1.0, 0.0),
        "Both finger chains should curl in the XZ plane around Y-axis pins.",
    )

    ctx.expect_contact(left_root, palm, name="left root is seated in palm tower")
    ctx.expect_contact(right_root, palm, name="right root is seated in palm tower")
    ctx.expect_contact(
        left_root,
        left_middle,
        name="left root module carries the middle link on a real joint interface",
    )
    ctx.expect_contact(
        left_middle,
        left_distal,
        name="left middle link carries the distal link on a real joint interface",
    )
    ctx.expect_contact(
        right_root,
        right_middle,
        name="right root module carries the middle link on a real joint interface",
    )
    ctx.expect_contact(
        right_middle,
        right_distal,
        name="right middle link carries the distal link on a real joint interface",
    )

    ctx.expect_gap(
        left_root,
        right_root,
        axis="y",
        min_gap=0.018,
        name="finger towers stay as separate chains with a center gap",
    )

    with ctx.pose(
        {
            palm_to_left_root: 0.18,
            left_root_to_left_middle: 0.34,
            left_middle_to_left_distal: 0.36,
            palm_to_right_root: 0.18,
            right_root_to_right_middle: 0.34,
            right_middle_to_right_distal: 0.36,
        }
    ):
        ctx.expect_gap(
            left_distal,
            right_distal,
            axis="y",
            min_gap=0.014,
            max_gap=0.040,
            name="curled tips stay close without sharing hardware",
        )
        ctx.expect_gap(
            left_distal,
            palm,
            axis="x",
            min_gap=0.008,
            name="left distal link clears the rear palm structure when curled",
        )
        ctx.expect_gap(
            right_distal,
            palm,
            axis="x",
            min_gap=0.008,
            name="right distal link clears the rear palm structure when curled",
        )

    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=24,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
