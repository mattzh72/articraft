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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


OUTER_WIDTH = 0.054
JOINT_GAP = 0.024
SIDE_PLATE_THICKNESS = (OUTER_WIDTH - JOINT_GAP) / 2.0
SIDE_Y_OFFSET = JOINT_GAP / 2.0 + SIDE_PLATE_THICKNESS / 2.0

ROOT_BARREL_RADIUS = 0.016
MIDDLE_BARREL_RADIUS = 0.015
DISTAL_BARREL_RADIUS = 0.013

ROOT_PITCH = 0.078
MIDDLE_PITCH = 0.064


def _add_box(part, name, size, center, material) -> None:
    part.visual(Box(size), origin=Origin(xyz=center), material=material, name=name)


def _add_y_cylinder(part, name, radius, length, center, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=(pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_finger_module")

    dark_metal = model.material("dark_metal", rgba=(0.24, 0.26, 0.29, 1.0))
    mid_metal = model.material("mid_metal", rgba=(0.42, 0.45, 0.49, 1.0))
    light_metal = model.material("light_metal", rgba=(0.60, 0.62, 0.66, 1.0))
    pad_rubber = model.material("pad_rubber", rgba=(0.12, 0.13, 0.14, 1.0))

    base = model.part("base")
    _add_box(base, "base_housing", (0.052, 0.066, 0.050), (-0.044, 0.0, 0.0), dark_metal)
    _add_box(base, "base_flange", (0.056, 0.074, 0.012), (-0.042, 0.0, -0.025), dark_metal)
    _add_box(base, "base_left_cheek", (0.026, SIDE_PLATE_THICKNESS, 0.042), (-0.001, SIDE_Y_OFFSET, 0.0), mid_metal)
    _add_box(base, "base_right_cheek", (0.026, SIDE_PLATE_THICKNESS, 0.042), (-0.001, -SIDE_Y_OFFSET, 0.0), mid_metal)
    _add_y_cylinder(base, "base_left_cap", ROOT_BARREL_RADIUS + 0.004, SIDE_PLATE_THICKNESS, (0.0, SIDE_Y_OFFSET, 0.0), mid_metal)
    _add_y_cylinder(base, "base_right_cap", ROOT_BARREL_RADIUS + 0.004, SIDE_PLATE_THICKNESS, (0.0, -SIDE_Y_OFFSET, 0.0), mid_metal)
    base.inertial = Inertial.from_geometry(
        Box((0.072, 0.074, 0.056)),
        mass=1.8,
        origin=Origin(xyz=(-0.036, 0.0, -0.003)),
    )

    root_knuckle = model.part("root_knuckle")
    _add_y_cylinder(root_knuckle, "root_barrel", ROOT_BARREL_RADIUS, JOINT_GAP, (0.0, 0.0, 0.0), mid_metal)
    _add_box(root_knuckle, "root_body", (0.050, JOINT_GAP, 0.038), (0.025, 0.0, 0.0), mid_metal)
    _add_box(root_knuckle, "root_taper", (0.016, 0.020, 0.030), (0.050, 0.0, 0.0), mid_metal)
    _add_box(root_knuckle, "root_left_fork", (0.050, SIDE_PLATE_THICKNESS, 0.032), (0.058, SIDE_Y_OFFSET, 0.0), light_metal)
    _add_box(root_knuckle, "root_right_fork", (0.050, SIDE_PLATE_THICKNESS, 0.032), (0.058, -SIDE_Y_OFFSET, 0.0), light_metal)
    _add_y_cylinder(root_knuckle, "root_left_cap", MIDDLE_BARREL_RADIUS + 0.003, SIDE_PLATE_THICKNESS, (ROOT_PITCH, SIDE_Y_OFFSET, 0.0), light_metal)
    _add_y_cylinder(root_knuckle, "root_right_cap", MIDDLE_BARREL_RADIUS + 0.003, SIDE_PLATE_THICKNESS, (ROOT_PITCH, -SIDE_Y_OFFSET, 0.0), light_metal)
    root_knuckle.inertial = Inertial.from_geometry(
        Box((0.094, OUTER_WIDTH, 0.042)),
        mass=0.72,
        origin=Origin(xyz=(0.047, 0.0, 0.0)),
    )

    middle_link = model.part("middle_link")
    _add_y_cylinder(middle_link, "middle_barrel", MIDDLE_BARREL_RADIUS, JOINT_GAP, (0.0, 0.0, 0.0), light_metal)
    _add_box(middle_link, "middle_body", (0.040, JOINT_GAP, 0.032), (0.020, 0.0, 0.0), light_metal)
    _add_box(middle_link, "middle_taper", (0.012, 0.020, 0.026), (0.043, 0.0, 0.0), light_metal)
    _add_box(middle_link, "middle_left_fork", (0.042, SIDE_PLATE_THICKNESS, 0.028), (0.049, SIDE_Y_OFFSET, 0.0), mid_metal)
    _add_box(middle_link, "middle_right_fork", (0.042, SIDE_PLATE_THICKNESS, 0.028), (0.049, -SIDE_Y_OFFSET, 0.0), mid_metal)
    _add_y_cylinder(middle_link, "middle_left_cap", DISTAL_BARREL_RADIUS + 0.003, SIDE_PLATE_THICKNESS, (MIDDLE_PITCH, SIDE_Y_OFFSET, 0.0), mid_metal)
    _add_y_cylinder(middle_link, "middle_right_cap", DISTAL_BARREL_RADIUS + 0.003, SIDE_PLATE_THICKNESS, (MIDDLE_PITCH, -SIDE_Y_OFFSET, 0.0), mid_metal)
    middle_link.inertial = Inertial.from_geometry(
        Box((0.078, OUTER_WIDTH, 0.034)),
        mass=0.42,
        origin=Origin(xyz=(0.036, 0.0, 0.0)),
    )

    distal_link = model.part("distal_link")
    _add_y_cylinder(distal_link, "distal_barrel", DISTAL_BARREL_RADIUS, JOINT_GAP, (0.0, 0.0, 0.0), light_metal)
    _add_box(distal_link, "distal_body", (0.020, 0.017, 0.028), (0.015, 0.0, 0.0), light_metal)
    _add_box(distal_link, "distal_taper", (0.018, 0.015, 0.024), (0.034, 0.0, 0.0), light_metal)
    _add_box(distal_link, "distal_nose", (0.026, 0.030, 0.020), (0.054, 0.0, 0.0), light_metal)
    _add_box(distal_link, "tip_pad", (0.032, 0.060, 0.012), (0.067, 0.0, -0.016), pad_rubber)
    distal_link.inertial = Inertial.from_geometry(
        Box((0.084, 0.060, 0.034)),
        mass=0.28,
        origin=Origin(xyz=(0.044, 0.0, -0.004)),
    )

    model.articulation(
        "base_to_root",
        ArticulationType.REVOLUTE,
        parent=base,
        child=root_knuckle,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.6, lower=-0.10, upper=1.15),
    )
    model.articulation(
        "root_to_middle",
        ArticulationType.REVOLUTE,
        parent=root_knuckle,
        child=middle_link,
        origin=Origin(xyz=(ROOT_PITCH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=3.0, lower=-0.05, upper=1.30),
    )
    model.articulation(
        "middle_to_distal",
        ArticulationType.REVOLUTE,
        parent=middle_link,
        child=distal_link,
        origin=Origin(xyz=(MIDDLE_PITCH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.2, lower=-0.05, upper=1.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    root_knuckle = object_model.get_part("root_knuckle")
    middle_link = object_model.get_part("middle_link")
    distal_link = object_model.get_part("distal_link")

    base_to_root = object_model.get_articulation("base_to_root")
    root_to_middle = object_model.get_articulation("root_to_middle")
    middle_to_distal = object_model.get_articulation("middle_to_distal")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "all_parts_present",
        all(part is not None for part in (base, root_knuckle, middle_link, distal_link)),
        "Expected base, root_knuckle, middle_link, and distal_link parts.",
    )
    ctx.check(
        "serial_joint_axes_parallel_y",
        all(joint.axis == (0.0, 1.0, 0.0) for joint in (base_to_root, root_to_middle, middle_to_distal)),
        "The root, middle, and distal joints should all bend in one plane around the Y axis.",
    )
    ctx.check(
        "joint_limits_match_closing_finger",
        (
            base_to_root.motion_limits is not None
            and root_to_middle.motion_limits is not None
            and middle_to_distal.motion_limits is not None
            and base_to_root.motion_limits.lower is not None
            and root_to_middle.motion_limits.lower is not None
            and middle_to_distal.motion_limits.lower is not None
            and base_to_root.motion_limits.upper is not None
            and root_to_middle.motion_limits.upper is not None
            and middle_to_distal.motion_limits.upper is not None
            and base_to_root.motion_limits.lower >= -0.12
            and root_to_middle.motion_limits.lower >= -0.12
            and middle_to_distal.motion_limits.lower >= -0.12
            and base_to_root.motion_limits.upper >= 1.0
            and root_to_middle.motion_limits.upper >= 1.0
            and middle_to_distal.motion_limits.upper >= 0.9
        ),
        "Finger joints should allow large forward flexion with only slight reverse extension.",
    )

    with ctx.pose(base_to_root=0.0, root_to_middle=0.0, middle_to_distal=0.0):
        ctx.expect_contact(
            root_knuckle,
            base,
            elem_a="root_barrel",
            elem_b="base_left_cheek",
            name="root_barrel_contacts_left_base_cheek",
        )
        ctx.expect_contact(
            root_knuckle,
            base,
            elem_a="root_barrel",
            elem_b="base_right_cheek",
            name="root_barrel_contacts_right_base_cheek",
        )
        ctx.expect_contact(
            middle_link,
            root_knuckle,
            elem_a="middle_barrel",
            elem_b="root_left_fork",
            name="middle_barrel_contacts_left_root_fork",
        )
        ctx.expect_contact(
            middle_link,
            root_knuckle,
            elem_a="middle_barrel",
            elem_b="root_right_fork",
            name="middle_barrel_contacts_right_root_fork",
        )
        ctx.expect_contact(
            distal_link,
            middle_link,
            elem_a="distal_barrel",
            elem_b="middle_left_fork",
            name="distal_barrel_contacts_left_middle_fork",
        )
        ctx.expect_contact(
            distal_link,
            middle_link,
            elem_a="distal_barrel",
            elem_b="middle_right_fork",
            name="distal_barrel_contacts_right_middle_fork",
        )
        ctx.expect_origin_gap(
            middle_link,
            root_knuckle,
            axis="x",
            min_gap=ROOT_PITCH - 1e-6,
            max_gap=ROOT_PITCH + 1e-6,
            name="middle_joint_pitch_matches_design",
        )
        ctx.expect_origin_gap(
            distal_link,
            middle_link,
            axis="x",
            min_gap=MIDDLE_PITCH - 1e-6,
            max_gap=MIDDLE_PITCH + 1e-6,
            name="distal_joint_pitch_matches_design",
        )

        tip_pad_aabb = ctx.part_element_world_aabb(distal_link, elem="tip_pad")
        distal_barrel_aabb = ctx.part_element_world_aabb(distal_link, elem="distal_barrel")
        root_body_aabb = ctx.part_element_world_aabb(root_knuckle, elem="root_body")
        middle_body_aabb = ctx.part_element_world_aabb(middle_link, elem="middle_body")
        distal_body_aabb = ctx.part_element_world_aabb(distal_link, elem="distal_body")
        ctx.check(
            "tip_pad_is_broad_and_forward",
            tip_pad_aabb is not None
            and distal_barrel_aabb is not None
            and (tip_pad_aabb[1][0] - distal_barrel_aabb[1][0]) >= 0.050
            and (tip_pad_aabb[1][1] - tip_pad_aabb[0][1]) >= 0.058,
            "The distal tip should carry a broad forward pad beyond the distal joint barrel.",
        )
        ctx.check(
            "links_step_down_in_height",
            root_body_aabb is not None
            and middle_body_aabb is not None
            and distal_body_aabb is not None
            and (root_body_aabb[1][2] - root_body_aabb[0][2]) > (middle_body_aabb[1][2] - middle_body_aabb[0][2])
            and (middle_body_aabb[1][2] - middle_body_aabb[0][2]) > (distal_body_aabb[1][2] - distal_body_aabb[0][2])
            and (root_body_aabb[1][0] - root_body_aabb[0][0]) > (middle_body_aabb[1][0] - middle_body_aabb[0][0])
            and (middle_body_aabb[1][0] - middle_body_aabb[0][0]) > (distal_body_aabb[1][0] - distal_body_aabb[0][0]),
            "The rigid core links should taper from a thick root knuckle to a smaller middle link to a shorter distal body.",
        )

        open_tip_center_x = None
        open_tip_center_z = None
        if tip_pad_aabb is not None:
            open_tip_center_x = (tip_pad_aabb[0][0] + tip_pad_aabb[1][0]) / 2.0
            open_tip_center_z = (tip_pad_aabb[0][2] + tip_pad_aabb[1][2]) / 2.0
    with ctx.pose(base_to_root=0.55, root_to_middle=0.70, middle_to_distal=0.50):
        curled_tip_aabb = ctx.part_element_world_aabb(distal_link, elem="tip_pad")
        curled_tip_center_x = None
        curled_tip_center_z = None
        if curled_tip_aabb is not None:
            curled_tip_center_x = (curled_tip_aabb[0][0] + curled_tip_aabb[1][0]) / 2.0
            curled_tip_center_z = (curled_tip_aabb[0][2] + curled_tip_aabb[1][2]) / 2.0
        ctx.check(
            "finger_curls_tip_back_and_down",
            open_tip_center_x is not None
            and open_tip_center_z is not None
            and curled_tip_center_x is not None
            and curled_tip_center_z is not None
            and curled_tip_center_x < open_tip_center_x - 0.018
            and curled_tip_center_z < open_tip_center_z - 0.030,
            "A curled pose should pull the fingertip back and down relative to the open pose.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
