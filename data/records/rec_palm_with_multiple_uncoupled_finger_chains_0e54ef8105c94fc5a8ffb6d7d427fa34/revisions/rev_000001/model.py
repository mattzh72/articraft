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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


PALM_WIDTH = 0.082
PALM_LENGTH = 0.060
PALM_HEIGHT = 0.022
PALM_FRONT_Y = PALM_LENGTH * 0.5
PALM_LEFT_X = -PALM_WIDTH * 0.5
DIGIT_THICKNESS = 0.012
FINGER_BASE_AXIS = (-1.0, 0.0, 0.0)
THUMB_BASE_AXIS = (0.0, 0.0, -1.0)
THUMB_YAW = -0.55

FINGER_SPECS = (
    {"name": "index", "root_x": -0.027, "width": 0.0130, "inner_len": 0.028, "outer_len": 0.023},
    {"name": "middle", "root_x": -0.009, "width": 0.0140, "inner_len": 0.030, "outer_len": 0.024},
    {"name": "ring", "root_x": 0.009, "width": 0.0130, "inner_len": 0.028, "outer_len": 0.022},
    {"name": "pinky", "root_x": 0.027, "width": 0.0115, "inner_len": 0.024, "outer_len": 0.019},
)

THUMB_SPEC = {
    "name": "thumb",
    "root_y": -0.004,
    "root_z": -0.013,
    "width": 0.0140,
    "inner_len": 0.025,
    "outer_len": 0.021,
}


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _profile_section(profile, z: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in profile]


def _finger_barrel_radius(thickness: float = DIGIT_THICKNESS) -> float:
    return thickness * 0.45


def _build_finger_inner(part, *, width: float, inner_len: float, thickness: float, shell_mat, joint_mat) -> None:
    barrel_radius = _finger_barrel_radius(thickness)
    cheek_clear = 0.0004
    cheek_thickness = 0.003
    tongue_width = width * 0.46
    cheek_offset = tongue_width * 0.5 + cheek_clear + cheek_thickness * 0.5
    yoke_length = 0.008

    part.visual(
        Cylinder(radius=barrel_radius, length=width + 0.002),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=joint_mat,
        name="root_barrel",
    )
    part.visual(
        Box((width, inner_len - 0.002, thickness * 0.76)),
        origin=Origin(xyz=(0.0, inner_len * 0.5, 0.0)),
        material=shell_mat,
        name="inner_beam",
    )
    part.visual(
        Box((width * 0.84, inner_len * 0.48, thickness * 0.18)),
        origin=Origin(xyz=(0.0, inner_len * 0.40, thickness * 0.19)),
        material=shell_mat,
        name="dorsal_cover",
    )
    part.visual(
        Box((cheek_thickness, yoke_length, thickness * 0.90)),
        origin=Origin(xyz=(-cheek_offset, inner_len - yoke_length * 0.5, 0.0)),
        material=shell_mat,
        name="distal_cheek_left",
    )
    part.visual(
        Box((cheek_thickness, yoke_length, thickness * 0.90)),
        origin=Origin(xyz=(cheek_offset, inner_len - yoke_length * 0.5, 0.0)),
        material=shell_mat,
        name="distal_cheek_right",
    )
    part.inertial = Inertial.from_geometry(
        Box((width + 0.002, inner_len + 0.004, thickness)),
        mass=0.040,
        origin=Origin(xyz=(0.0, inner_len * 0.5, 0.0)),
    )


def _build_finger_outer(part, *, width: float, outer_len: float, thickness: float, shell_mat, joint_mat, pad_mat) -> None:
    hinge_radius = thickness * 0.39
    tongue_width = width * 0.46

    part.visual(
        Cylinder(radius=hinge_radius, length=tongue_width),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=joint_mat,
        name="hinge_barrel",
    )
    part.visual(
        Box((tongue_width, 0.006, thickness * 0.58)),
        origin=Origin(xyz=(0.0, 0.001, 0.0)),
        material=joint_mat,
        name="hinge_tongue",
    )
    part.visual(
        Box((width * 0.90, outer_len - 0.002, thickness * 0.68)),
        origin=Origin(xyz=(0.0, 0.004 + (outer_len - 0.002) * 0.5, 0.0)),
        material=shell_mat,
        name="outer_beam",
    )
    part.visual(
        Box((width * 0.82, 0.008, thickness * 0.24)),
        origin=Origin(xyz=(0.0, outer_len - 0.002, -thickness * 0.34)),
        material=pad_mat,
        name="finger_pad",
    )
    part.inertial = Inertial.from_geometry(
        Box((width, outer_len + 0.008, thickness)),
        mass=0.028,
        origin=Origin(xyz=(0.0, outer_len * 0.5, 0.0)),
    )


def _thumb_offset(length: float, yaw: float) -> tuple[float, float, float]:
    return (-math.sin(yaw) * length, math.cos(yaw) * length, 0.0)


def _thumb_point(length: float, yaw: float, z: float = 0.0) -> tuple[float, float, float]:
    dx, dy, _ = _thumb_offset(length, yaw)
    return (dx, dy, z)


def _build_thumb_inner(part, *, width: float, inner_len: float, thickness: float, yaw: float, shell_mat, joint_mat) -> None:
    barrel_radius = _finger_barrel_radius(thickness)
    plate_thickness = 0.0026
    tongue_thickness = thickness * 0.42
    yoke_length = 0.007
    root_bridge_length = 0.010
    beam_start = 0.009
    beam_end = inner_len - yoke_length
    beam_length = beam_end - beam_start
    cover_length = beam_length * 0.64
    cover_center = _thumb_point(beam_start + cover_length * 0.5, yaw, thickness * 0.18)
    yoke_center = _thumb_point(inner_len - yoke_length * 0.5, yaw)

    part.visual(
        Cylinder(radius=barrel_radius, length=thickness * 1.15),
        material=joint_mat,
        name="root_barrel",
    )
    part.visual(
        Box((width * 0.78, root_bridge_length, thickness * 0.76)),
        origin=Origin(xyz=_thumb_point(root_bridge_length * 0.5, yaw), rpy=(0.0, 0.0, yaw)),
        material=joint_mat,
        name="root_bridge",
    )
    part.visual(
        Box((width, beam_length, thickness * 0.74)),
        origin=Origin(xyz=_thumb_point(beam_start + beam_length * 0.5, yaw), rpy=(0.0, 0.0, yaw)),
        material=shell_mat,
        name="inner_beam",
    )
    part.visual(
        Box((width * 0.82, cover_length, thickness * 0.18)),
        origin=Origin(xyz=cover_center, rpy=(0.0, 0.0, yaw)),
        material=shell_mat,
        name="dorsal_cover",
    )
    part.visual(
        Box((width * 0.92, yoke_length, plate_thickness)),
        origin=Origin(
            xyz=(yoke_center[0], yoke_center[1], -(tongue_thickness * 0.5 + plate_thickness * 0.5)),
            rpy=(0.0, 0.0, yaw),
        ),
        material=shell_mat,
        name="distal_plate_bottom",
    )
    part.visual(
        Box((width * 0.92, yoke_length, plate_thickness)),
        origin=Origin(
            xyz=(yoke_center[0], yoke_center[1], tongue_thickness * 0.5 + plate_thickness * 0.5),
            rpy=(0.0, 0.0, yaw),
        ),
        material=shell_mat,
        name="distal_plate_top",
    )
    part.inertial = Inertial.from_geometry(
        Box((width + 0.004, inner_len + 0.004, thickness)),
        mass=0.036,
        origin=Origin(xyz=(0.0, inner_len * 0.5, 0.0)),
    )


def _build_thumb_outer(part, *, width: float, outer_len: float, thickness: float, yaw: float, shell_mat, joint_mat, pad_mat) -> None:
    tongue_thickness = thickness * 0.42
    tongue_length = 0.012
    beam_start = tongue_length
    beam_length = outer_len - beam_start
    pad_length = 0.010
    beam_height = thickness * 0.66
    pad_height = thickness * 0.24

    part.visual(
        Cylinder(radius=thickness * 0.37, length=tongue_thickness),
        material=joint_mat,
        name="hinge_barrel",
    )
    part.visual(
        Box((width * 0.80, tongue_length, tongue_thickness)),
        origin=Origin(xyz=_thumb_point(beam_start * 0.5, yaw), rpy=(0.0, 0.0, yaw)),
        material=joint_mat,
        name="hinge_tongue",
    )
    part.visual(
        Box((width * 0.88, beam_length, thickness * 0.66)),
        origin=Origin(xyz=_thumb_point(beam_start + beam_length * 0.5, yaw), rpy=(0.0, 0.0, yaw)),
        material=shell_mat,
        name="outer_beam",
    )
    part.visual(
        Box((width * 0.78, pad_length, pad_height)),
        origin=Origin(
            xyz=_thumb_point(outer_len - pad_length * 0.5, yaw, -(beam_height * 0.5 + pad_height * 0.5) + 0.0002),
            rpy=(0.0, 0.0, yaw),
        ),
        material=pad_mat,
        name="finger_pad",
    )
    part.inertial = Inertial.from_geometry(
        Box((width + 0.004, outer_len + 0.008, thickness)),
        mass=0.026,
        origin=Origin(xyz=(0.0, outer_len * 0.5, 0.0)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_tabletop_robot_hand")

    palm_metal = model.material("palm_metal", rgba=(0.30, 0.32, 0.36, 1.0))
    digit_shell = model.material("digit_shell", rgba=(0.71, 0.73, 0.77, 1.0))
    joint_metal = model.material("joint_metal", rgba=(0.16, 0.17, 0.19, 1.0))
    rubber_pad = model.material("rubber_pad", rgba=(0.08, 0.09, 0.10, 1.0))

    palm = model.part("palm")
    palm_shell = section_loft(
        [
            _profile_section(rounded_rect_profile(0.074, 0.054, 0.010, corner_segments=8), -0.010),
            _profile_section(rounded_rect_profile(0.082, 0.060, 0.013, corner_segments=8), 0.000),
            _profile_section(rounded_rect_profile(0.074, 0.054, 0.010, corner_segments=8), 0.010),
        ]
    )
    palm.visual(_mesh("robot_hand_palm_shell", palm_shell), material=palm_metal, name="palm_shell")
    palm.visual(
        Box((0.070, 0.004, 0.014)),
        origin=Origin(xyz=(0.0, 0.028, 0.001)),
        material=palm_metal,
        name="front_mount_rail",
    )
    palm.visual(
        Box((0.010, 0.008, 0.020)),
        origin=Origin(xyz=(-0.0480, -0.003, -0.013)),
        material=palm_metal,
        name="thumb_mount_pad",
    )
    palm.visual(
        Box((0.004, 0.010, 0.014)),
        origin=Origin(xyz=(-0.0420, -0.003, -0.010)),
        material=palm_metal,
        name="thumb_mount_web",
    )
    palm.visual(
        Box((0.024, 0.020, 0.016)),
        origin=Origin(xyz=(0.0, -0.040, 0.000)),
        material=joint_metal,
        name="wrist_stub",
    )
    palm.inertial = Inertial.from_geometry(
        Box((0.082, 0.084, 0.022)),
        mass=0.24,
        origin=Origin(xyz=(0.0, -0.004, 0.0)),
    )

    for spec in FINGER_SPECS:
        inner = model.part(f"{spec['name']}_inner")
        outer = model.part(f"{spec['name']}_outer")
        _build_finger_inner(
            inner,
            width=spec["width"],
            inner_len=spec["inner_len"],
            thickness=DIGIT_THICKNESS,
            shell_mat=digit_shell,
            joint_mat=joint_metal,
        )
        _build_finger_outer(
            outer,
            width=spec["width"],
            outer_len=spec["outer_len"],
            thickness=DIGIT_THICKNESS,
            shell_mat=digit_shell,
            joint_mat=joint_metal,
            pad_mat=rubber_pad,
        )

        root_origin = Origin(
            xyz=(spec["root_x"], PALM_FRONT_Y + _finger_barrel_radius(), 0.003),
        )
        distal_origin = Origin(xyz=(0.0, spec["inner_len"], 0.0))

        model.articulation(
            f"palm_to_{spec['name']}_inner",
            ArticulationType.REVOLUTE,
            parent=palm,
            child=inner,
            origin=root_origin,
            axis=FINGER_BASE_AXIS,
            motion_limits=MotionLimits(effort=1.4, velocity=3.5, lower=0.0, upper=1.12),
        )
        model.articulation(
            f"{spec['name']}_inner_to_outer",
            ArticulationType.REVOLUTE,
            parent=inner,
            child=outer,
            origin=distal_origin,
            axis=FINGER_BASE_AXIS,
            motion_limits=MotionLimits(effort=1.0, velocity=4.0, lower=0.0, upper=1.28),
        )

    thumb_inner = model.part("thumb_inner")
    thumb_outer = model.part("thumb_outer")
    _build_thumb_inner(
        thumb_inner,
        width=THUMB_SPEC["width"],
        inner_len=THUMB_SPEC["inner_len"],
        thickness=DIGIT_THICKNESS,
        yaw=THUMB_YAW,
        shell_mat=digit_shell,
        joint_mat=joint_metal,
    )
    _build_thumb_outer(
        thumb_outer,
        width=THUMB_SPEC["width"],
        outer_len=THUMB_SPEC["outer_len"],
        thickness=DIGIT_THICKNESS,
        yaw=THUMB_YAW,
        shell_mat=digit_shell,
        joint_mat=joint_metal,
        pad_mat=rubber_pad,
    )

    thumb_root_origin = Origin(
        xyz=(
            PALM_LEFT_X - 0.0175,
            THUMB_SPEC["root_y"],
            THUMB_SPEC["root_z"],
        ),
    )
    thumb_distal_origin = Origin(xyz=_thumb_offset(THUMB_SPEC["inner_len"], THUMB_YAW))

    model.articulation(
        "palm_to_thumb_inner",
        ArticulationType.REVOLUTE,
        parent=palm,
        child=thumb_inner,
        origin=thumb_root_origin,
        axis=THUMB_BASE_AXIS,
        motion_limits=MotionLimits(effort=1.2, velocity=3.0, lower=0.0, upper=0.95),
    )
    model.articulation(
        "thumb_inner_to_outer",
        ArticulationType.REVOLUTE,
        parent=thumb_inner,
        child=thumb_outer,
        origin=thumb_distal_origin,
        axis=THUMB_BASE_AXIS,
        motion_limits=MotionLimits(effort=0.9, velocity=3.5, lower=0.0, upper=1.08),
    )

    return model


def _center_from_aabb(aabb) -> tuple[float, float, float]:
    return tuple((aabb[0][index] + aabb[1][index]) * 0.5 for index in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    palm = object_model.get_part("palm")
    thumb_inner = object_model.get_part("thumb_inner")
    thumb_outer = object_model.get_part("thumb_outer")

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

    for spec in FINGER_SPECS:
        name = spec["name"]
        inner = object_model.get_part(f"{name}_inner")
        outer = object_model.get_part(f"{name}_outer")
        root_joint = object_model.get_articulation(f"palm_to_{name}_inner")
        distal_joint = object_model.get_articulation(f"{name}_inner_to_outer")

        ctx.check(
            f"{name}_root_axis",
            tuple(root_joint.axis) == FINGER_BASE_AXIS,
            f"Expected {FINGER_BASE_AXIS} but got {root_joint.axis}",
        )
        ctx.check(
            f"{name}_distal_axis",
            tuple(distal_joint.axis) == FINGER_BASE_AXIS,
            f"Expected {FINGER_BASE_AXIS} but got {distal_joint.axis}",
        )
        ctx.expect_gap(
            inner,
            palm,
            axis="y",
            max_gap=0.0008,
            max_penetration=0.0,
            positive_elem="root_barrel",
            negative_elem="front_mount_rail",
            name=f"{name}_root_seated",
        )
        ctx.expect_overlap(
            inner,
            palm,
            axes="xz",
            min_overlap=0.008,
            elem_a="root_barrel",
            elem_b="front_mount_rail",
            name=f"{name}_root_aligned",
        )
        ctx.expect_gap(
            inner,
            outer,
            axis="x",
            max_gap=0.0008,
            max_penetration=0.0,
            positive_elem="distal_cheek_right",
            negative_elem="hinge_tongue",
            name=f"{name}_knuckle_right_capture",
        )
        ctx.expect_gap(
            outer,
            inner,
            axis="x",
            max_gap=0.0008,
            max_penetration=0.0,
            positive_elem="hinge_tongue",
            negative_elem="distal_cheek_left",
            name=f"{name}_knuckle_left_capture",
        )

    thumb_root_joint = object_model.get_articulation("palm_to_thumb_inner")
    thumb_distal_joint = object_model.get_articulation("thumb_inner_to_outer")

    ctx.check(
        "thumb_root_axis",
        tuple(thumb_root_joint.axis) == THUMB_BASE_AXIS,
        f"Expected {THUMB_BASE_AXIS} but got {thumb_root_joint.axis}",
    )
    ctx.check(
        "thumb_distal_axis",
        tuple(thumb_distal_joint.axis) == THUMB_BASE_AXIS,
        f"Expected {THUMB_BASE_AXIS} but got {thumb_distal_joint.axis}",
    )
    ctx.expect_gap(
        palm,
        thumb_inner,
        axis="x",
        max_gap=0.0008,
        max_penetration=0.0,
        positive_elem="thumb_mount_pad",
        negative_elem="root_barrel",
        name="thumb_root_seated",
    )
    ctx.expect_overlap(
        palm,
        thumb_inner,
        axes="yz",
        min_overlap=0.008,
        elem_a="thumb_mount_pad",
        elem_b="root_barrel",
        name="thumb_root_aligned",
    )
    ctx.expect_gap(
        thumb_inner,
        thumb_outer,
        axis="z",
        max_gap=0.0008,
        max_penetration=0.0,
        positive_elem="distal_plate_top",
        negative_elem="hinge_tongue",
        name="thumb_knuckle_top_capture",
    )
    ctx.expect_gap(
        thumb_outer,
        thumb_inner,
        axis="z",
        max_gap=0.0008,
        max_penetration=0.0,
        positive_elem="hinge_tongue",
        negative_elem="distal_plate_bottom",
        name="thumb_knuckle_bottom_capture",
    )

    middle_root = object_model.get_articulation("palm_to_middle_inner")
    middle_distal = object_model.get_articulation("middle_inner_to_outer")
    middle_outer = object_model.get_part("middle_outer")
    middle_tip_rest = ctx.part_element_world_aabb(middle_outer, elem="finger_pad")
    assert middle_tip_rest is not None

    thumb_tip_rest = ctx.part_element_world_aabb(thumb_outer, elem="finger_pad")
    assert thumb_tip_rest is not None

    with ctx.pose({middle_root: 0.82, middle_distal: 1.02, thumb_root_joint: 0.55, thumb_distal_joint: 0.72}):
        ctx.expect_gap(
            object_model.get_part("middle_inner"),
            middle_outer,
            axis="x",
            max_gap=0.0008,
            max_penetration=0.0,
            positive_elem="distal_cheek_right",
            negative_elem="hinge_tongue",
            name="middle_knuckle_right_capture_flexed",
        )
        ctx.expect_gap(
            middle_outer,
            object_model.get_part("middle_inner"),
            axis="x",
            max_gap=0.0008,
            max_penetration=0.0,
            positive_elem="hinge_tongue",
            negative_elem="distal_cheek_left",
            name="middle_knuckle_left_capture_flexed",
        )
        ctx.expect_gap(
            thumb_inner,
            thumb_outer,
            axis="z",
            max_gap=0.0008,
            max_penetration=0.0,
            positive_elem="distal_plate_top",
            negative_elem="hinge_tongue",
            name="thumb_knuckle_top_capture_flexed",
        )
        ctx.expect_gap(
            thumb_outer,
            thumb_inner,
            axis="z",
            max_gap=0.0008,
            max_penetration=0.0,
            positive_elem="hinge_tongue",
            negative_elem="distal_plate_bottom",
            name="thumb_knuckle_bottom_capture_flexed",
        )

        middle_tip_curled = ctx.part_element_world_aabb(middle_outer, elem="finger_pad")
        thumb_tip_curled = ctx.part_element_world_aabb(thumb_outer, elem="finger_pad")
        assert middle_tip_curled is not None
        assert thumb_tip_curled is not None

        middle_rest_center = _center_from_aabb(middle_tip_rest)
        middle_curled_center = _center_from_aabb(middle_tip_curled)
        thumb_rest_center = _center_from_aabb(thumb_tip_rest)
        thumb_curled_center = _center_from_aabb(thumb_tip_curled)

        ctx.check(
            "middle_tip_curls_down",
            middle_curled_center[2] < middle_rest_center[2] - 0.020,
            f"Expected middle fingertip to drop while curling; rest={middle_rest_center}, curled={middle_curled_center}",
        )
        ctx.check(
            "thumb_tip_sweeps_inward",
            thumb_curled_center[0] > thumb_rest_center[0] + 0.010,
            f"Expected thumb tip to sweep inward toward the palm center; rest={thumb_rest_center}, curled={thumb_curled_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
