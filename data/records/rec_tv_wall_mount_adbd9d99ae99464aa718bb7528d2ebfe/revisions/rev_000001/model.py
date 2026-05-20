from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


WALL_TO_REAR = "wall_to_rear"
REAR_TO_FORE = "rear_to_fore"
FORE_TO_HEAD = "fore_to_head"
HEAD_TO_FRAME = "head_to_frame"

REAR_LINK_LENGTH = 0.34
FORE_LINK_LENGTH = 0.30


def _rounded_plate_mesh(name: str, height_z: float, width_y: float, thickness_x: float, radius: float):
    """Rounded vertical plate made in local XY, then rotated by the visual."""
    profile = rounded_rect_profile(height_z, width_y, radius, corner_segments=10)
    return mesh_from_geometry(ExtrudeGeometry(profile, thickness_x, center=True), name)


def _rounded_link_mesh(name: str, pivot_distance: float, width: float, thickness: float):
    """Flat capsule-like link with rounded ends centered on the two pivots."""
    profile = rounded_rect_profile(pivot_distance + width, width, width * 0.5, corner_segments=12)
    return mesh_from_geometry(ExtrudeGeometry(profile, thickness, center=True), name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_wall_display_arm")

    powder_coat = Material("satin_black_powder_coat", rgba=(0.015, 0.016, 0.018, 1.0))
    dark_metal = Material("dark_anodized_aluminum", rgba=(0.055, 0.060, 0.065, 1.0))
    edge_black = Material("black_plastic_edge_caps", rgba=(0.005, 0.005, 0.006, 1.0))
    screw_metal = Material("dark_burnished_screw_heads", rgba=(0.16, 0.16, 0.15, 1.0))

    # The root frame is the shoulder-pivot axis.  The broad plate sits behind it
    # on the wall, while the first folding link is held forward in a shallow clevis.
    wall_plate = model.part("wall_plate")
    wall_plate.visual(
        _rounded_plate_mesh("wall_plate_shell_mesh", 0.48, 0.36, 0.030, 0.035),
        origin=Origin(xyz=(-0.085, 0.0, 0.0), rpy=(0.0, -math.pi / 2.0, 0.0)),
        material=powder_coat,
        name="wall_plate_shell",
    )
    wall_plate.visual(
        Box((0.018, 0.22, 0.31)),
        origin=Origin(xyz=(-0.066, 0.0, 0.0)),
        material=dark_metal,
        name="raised_center_spine",
    )
    wall_plate.visual(
        Box((0.115, 0.135, 0.012)),
        origin=Origin(xyz=(-0.015, 0.0, 0.052)),
        material=dark_metal,
        name="shoulder_upper_ear",
    )
    wall_plate.visual(
        Box((0.115, 0.135, 0.012)),
        origin=Origin(xyz=(-0.015, 0.0, -0.018)),
        material=dark_metal,
        name="shoulder_lower_ear",
    )
    wall_plate.visual(
        Box((0.020, 0.145, 0.090)),
        origin=Origin(xyz=(-0.066, 0.0, 0.017)),
        material=dark_metal,
        name="shoulder_back_block",
    )
    wall_plate.visual(
        Cylinder(radius=0.012, length=0.098),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=screw_metal,
        name="shoulder_pin",
    )
    for i, (y, z) in enumerate(((-0.13, -0.17), (0.13, -0.17), (-0.13, 0.17), (0.13, 0.17))):
        wall_plate.visual(
            Cylinder(radius=0.018, length=0.007),
            origin=Origin(xyz=(-0.0695, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=screw_metal,
            name=f"wall_screw_{i}",
        )

    rear_link = model.part("rear_link")
    rear_link.visual(
        _rounded_link_mesh("rear_link_body_mesh", REAR_LINK_LENGTH, 0.108, 0.026),
        origin=Origin(xyz=(REAR_LINK_LENGTH * 0.5, 0.0, 0.018)),
        material=dark_metal,
        name="rear_body",
    )
    rear_link.visual(
        Cylinder(radius=0.011, length=0.086),
        origin=Origin(xyz=(REAR_LINK_LENGTH, 0.0, 0.0)),
        material=screw_metal,
        name="elbow_pin",
    )
    rear_link.visual(
        Cylinder(radius=0.033, length=0.007),
        origin=Origin(xyz=(REAR_LINK_LENGTH, 0.0, 0.037)),
        material=edge_black,
        name="elbow_cap",
    )

    fore_link = model.part("fore_link")
    fore_link.visual(
        _rounded_link_mesh("fore_link_body_mesh", FORE_LINK_LENGTH, 0.092, 0.024),
        origin=Origin(xyz=(FORE_LINK_LENGTH * 0.5, 0.0, -0.019)),
        material=dark_metal,
        name="fore_body",
    )
    fore_link.visual(
        Cylinder(radius=0.010, length=0.100),
        origin=Origin(xyz=(FORE_LINK_LENGTH, 0.0, 0.010)),
        material=screw_metal,
        name="wrist_pin",
    )

    head_swivel = model.part("head_swivel")
    head_swivel.visual(
        Cylinder(radius=0.028, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=dark_metal,
        name="swivel_barrel",
    )
    head_swivel.visual(
        Cylinder(radius=0.032, length=0.009),
        origin=Origin(xyz=(0.0, 0.0, 0.069)),
        material=edge_black,
        name="swivel_cap",
    )
    head_swivel.visual(
        Box((0.049, 0.100, 0.024)),
        origin=Origin(xyz=(0.037, 0.0, 0.040)),
        material=dark_metal,
        name="tilt_bridge",
    )
    head_swivel.visual(
        Box((0.040, 0.012, 0.070)),
        origin=Origin(xyz=(0.0795, 0.044, 0.040)),
        material=dark_metal,
        name="tilt_cheek_0",
    )
    head_swivel.visual(
        Box((0.040, 0.012, 0.070)),
        origin=Origin(xyz=(0.0795, -0.044, 0.040)),
        material=dark_metal,
        name="tilt_cheek_1",
    )
    head_swivel.visual(
        Cylinder(radius=0.008, length=0.108),
        origin=Origin(xyz=(0.080, 0.0, 0.040), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=screw_metal,
        name="tilt_pin",
    )

    monitor_frame = model.part("monitor_frame")
    monitor_frame.visual(
        Cylinder(radius=0.017, length=0.066),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=screw_metal,
        name="tilt_socket",
    )
    monitor_frame.visual(
        Box((0.052, 0.050, 0.034)),
        origin=Origin(xyz=(0.038, 0.0, 0.0)),
        material=dark_metal,
        name="neck_block",
    )
    monitor_frame.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.160, 0.270),
                (0.235, 0.350),
                0.020,
                opening_shape="rounded_rect",
                outer_shape="rounded_rect",
                opening_corner_radius=0.012,
                outer_corner_radius=0.020,
            ),
            "monitor_frame_bezel_mesh",
        ),
        origin=Origin(xyz=(0.070, 0.0, 0.0), rpy=(0.0, -math.pi / 2.0, 0.0)),
        material=powder_coat,
        name="frame_bezel",
    )
    monitor_frame.visual(
        Box((0.020, 0.292, 0.016)),
        origin=Origin(xyz=(0.060, 0.0, 0.0)),
        material=dark_metal,
        name="horizontal_rail",
    )
    monitor_frame.visual(
        Box((0.020, 0.016, 0.182)),
        origin=Origin(xyz=(0.060, 0.0, 0.0)),
        material=dark_metal,
        name="vertical_rail",
    )
    monitor_frame.visual(
        Box((0.023, 0.110, 0.110)),
        origin=Origin(xyz=(0.048, 0.0, 0.0)),
        material=dark_metal,
        name="vesa_plate",
    )
    for i, (y, z) in enumerate(((-0.0375, -0.0375), (0.0375, -0.0375), (-0.0375, 0.0375), (0.0375, 0.0375))):
        monitor_frame.visual(
            Cylinder(radius=0.0085, length=0.006),
            origin=Origin(xyz=(0.034, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=screw_metal,
            name=f"vesa_boss_{i}",
        )

    model.articulation(
        WALL_TO_REAR,
        ArticulationType.REVOLUTE,
        parent=wall_plate,
        child=rear_link,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.6, lower=-1.25, upper=1.25),
    )
    model.articulation(
        REAR_TO_FORE,
        ArticulationType.REVOLUTE,
        parent=rear_link,
        child=fore_link,
        origin=Origin(xyz=(REAR_LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.8, lower=-2.45, upper=2.45),
    )
    model.articulation(
        FORE_TO_HEAD,
        ArticulationType.REVOLUTE,
        parent=fore_link,
        child=head_swivel,
        origin=Origin(xyz=(FORE_LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-1.40, upper=1.40),
    )
    model.articulation(
        HEAD_TO_FRAME,
        ArticulationType.REVOLUTE,
        parent=head_swivel,
        child=monitor_frame,
        origin=Origin(xyz=(0.080, 0.0, 0.040)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=-0.45, upper=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    wall = object_model.get_part("wall_plate")
    rear = object_model.get_part("rear_link")
    fore = object_model.get_part("fore_link")
    head = object_model.get_part("head_swivel")
    frame = object_model.get_part("monitor_frame")

    wall_joint = object_model.get_articulation(WALL_TO_REAR)
    elbow_joint = object_model.get_articulation(REAR_TO_FORE)
    swivel_joint = object_model.get_articulation(FORE_TO_HEAD)
    tilt_joint = object_model.get_articulation(HEAD_TO_FRAME)

    ctx.check(
        "four revolute mechanisms are present",
        all(j.articulation_type == ArticulationType.REVOLUTE for j in (wall_joint, elbow_joint, swivel_joint, tilt_joint)),
        details="Expected two folding joints plus head swivel and tilt.",
    )

    ctx.allow_overlap(
        wall,
        rear,
        elem_a="shoulder_pin",
        elem_b="rear_body",
        reason="The shoulder pin is intentionally captured inside the rear link pivot boss.",
    )
    ctx.expect_within(
        wall,
        rear,
        axes="xy",
        inner_elem="shoulder_pin",
        outer_elem="rear_body",
        margin=0.001,
        name="shoulder pin sits inside rear pivot boss",
    )
    ctx.expect_overlap(
        wall,
        rear,
        axes="z",
        elem_a="shoulder_pin",
        elem_b="rear_body",
        min_overlap=0.020,
        name="shoulder pin passes through rear link thickness",
    )

    ctx.allow_overlap(
        rear,
        fore,
        elem_a="elbow_pin",
        elem_b="fore_body",
        reason="The elbow pin is intentionally captured inside the lower fore-link pivot boss.",
    )
    ctx.expect_within(
        rear,
        fore,
        axes="xy",
        inner_elem="elbow_pin",
        outer_elem="fore_body",
        margin=0.001,
        name="elbow pin sits inside fore pivot boss",
    )
    ctx.expect_overlap(
        rear,
        fore,
        axes="z",
        elem_a="elbow_pin",
        elem_b="fore_body",
        min_overlap=0.020,
        name="elbow pin passes through fore link thickness",
    )

    ctx.allow_overlap(
        fore,
        head,
        elem_a="wrist_pin",
        elem_b="swivel_barrel",
        reason="The wrist swivel barrel is retained by the vertical wrist pin.",
    )
    ctx.expect_within(
        fore,
        head,
        axes="xy",
        inner_elem="wrist_pin",
        outer_elem="swivel_barrel",
        margin=0.001,
        name="wrist pin is centered in swivel barrel",
    )
    ctx.expect_overlap(
        fore,
        head,
        axes="z",
        elem_a="wrist_pin",
        elem_b="swivel_barrel",
        min_overlap=0.040,
        name="wrist pin engages swivel barrel height",
    )

    ctx.allow_overlap(
        head,
        frame,
        elem_a="tilt_pin",
        elem_b="tilt_socket",
        reason="The tilt pin intentionally passes through the compact monitor-frame socket.",
    )
    ctx.expect_within(
        head,
        frame,
        axes="xz",
        inner_elem="tilt_pin",
        outer_elem="tilt_socket",
        margin=0.001,
        name="tilt pin is centered in the tilt socket",
    )
    ctx.expect_overlap(
        head,
        frame,
        axes="y",
        elem_a="tilt_pin",
        elem_b="tilt_socket",
        min_overlap=0.060,
        name="tilt pin spans the frame socket",
    )

    wall_aabb = ctx.part_world_aabb(wall)
    rear_aabb = ctx.part_world_aabb(rear)
    head_aabb = ctx.part_world_aabb(head)
    frame_aabb = ctx.part_world_aabb(frame)
    if wall_aabb and rear_aabb and head_aabb and frame_aabb:
        wall_dims = tuple(wall_aabb[1][i] - wall_aabb[0][i] for i in range(3))
        rear_dims = tuple(rear_aabb[1][i] - rear_aabb[0][i] for i in range(3))
        head_dims = tuple(head_aabb[1][i] - head_aabb[0][i] for i in range(3))
        frame_dims = tuple(frame_aabb[1][i] - frame_aabb[0][i] for i in range(3))
        ctx.check(
            "broad wall plate and compact head proportions",
            wall_dims[1] > 0.32
            and wall_dims[2] > 0.44
            and head_dims[0] < rear_dims[0] * 0.45
            and head_dims[1] < wall_dims[1] * 0.38
            and frame_dims[0] < 0.18,
            details=f"wall={wall_dims}, rear={rear_dims}, head={head_dims}, frame={frame_dims}",
        )
    else:
        ctx.fail("aabbs are available for proportion checks", "One or more part AABBs were unavailable.")

    rest_pos = ctx.part_world_position(frame)
    with ctx.pose({elbow_joint: -1.10, wall_joint: 0.75, swivel_joint: 0.60, tilt_joint: 0.30}):
        moved_pos = ctx.part_world_position(frame)
    ctx.check(
        "folding and head joints move the frame",
        rest_pos is not None
        and moved_pos is not None
        and (abs(moved_pos[0] - rest_pos[0]) > 0.05 or abs(moved_pos[1] - rest_pos[1]) > 0.05),
        details=f"rest={rest_pos}, moved={moved_pos}",
    )

    return ctx.report()


object_model = build_object_model()
