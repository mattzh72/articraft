from __future__ import annotations

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


def _rotate_y(x: float, y: float, z: float, angle: float) -> tuple[float, float, float]:
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    return (x * cos_a + z * sin_a, y, -x * sin_a + z * cos_a)


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="minimalist_task_lamp")

    graphite = model.material("graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    aluminum = model.material("aluminum", rgba=(0.72, 0.73, 0.75, 1.0))
    diffuser = model.material("diffuser", rgba=(0.94, 0.95, 0.93, 1.0))
    accent = model.material("accent", rgba=(0.83, 0.36, 0.18, 1.0))

    base = model.part("base")
    base_plate_size = (0.18, 0.12, 0.012)
    base.visual(
        Box(base_plate_size),
        origin=Origin(xyz=(0.0, 0.0, base_plate_size[2] * 0.5)),
        material=graphite,
        name="base_plate",
    )

    pedestal_size = (0.04, 0.05, 0.02)
    pedestal_x = -0.055
    pedestal_z = base_plate_size[2] + pedestal_size[2] * 0.5
    base.visual(
        Box(pedestal_size),
        origin=Origin(xyz=(pedestal_x, 0.0, pedestal_z)),
        material=graphite,
        name="pivot_block",
    )

    base_hinge_z = 0.046
    base_cheek_size = (0.014, 0.008, 0.028)
    for index, y in enumerate((-0.015, 0.015)):
        base.visual(
            Box(base_cheek_size),
            origin=Origin(xyz=(pedestal_x, y, base_hinge_z)),
            material=graphite,
            name=f"base_cheek_{index}",
        )

    upright = model.part("upright")
    upright_barrel_radius = 0.006
    upright_barrel_length = 0.022
    upright.visual(
        Cylinder(radius=upright_barrel_radius, length=upright_barrel_length),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=aluminum,
        name="upright_barrel",
    )

    upright_pitch = 0.26
    upright_beam_len = 0.20
    upright_beam_size = (0.014, 0.012, upright_beam_len)
    upright_beam_center = _rotate_y(0.0, 0.0, upright_beam_len * 0.5, upright_pitch)
    upright.visual(
        Box(upright_beam_size),
        origin=Origin(xyz=upright_beam_center, rpy=(0.0, upright_pitch, 0.0)),
        material=aluminum,
        name="upright_beam",
    )

    elbow_local_z = 0.208
    elbow_center = _rotate_y(0.0, 0.0, elbow_local_z, upright_pitch)
    elbow_cheek_local_z = 0.202
    elbow_cheek_center = _rotate_y(0.0, 0.0, elbow_cheek_local_z, upright_pitch)
    upright_elbow_cheek_size = (0.016, 0.005, 0.032)
    for index, y in enumerate((-0.011, 0.011)):
        cheek_center = (elbow_cheek_center[0], y, elbow_cheek_center[2])
        upright.visual(
            Box(upright_elbow_cheek_size),
            origin=Origin(xyz=cheek_center, rpy=(0.0, upright_pitch, 0.0)),
            material=aluminum,
            name=f"elbow_cheek_{index}",
        )
    elbow_mount_center = _rotate_y(0.0, 0.0, 0.188, upright_pitch)
    upright.visual(
        Box((0.012, 0.024, 0.018)),
        origin=Origin(xyz=elbow_mount_center, rpy=(0.0, upright_pitch, 0.0)),
        material=aluminum,
        name="elbow_mount",
    )

    forearm = model.part("forearm")
    forearm_barrel_radius = 0.005
    forearm_barrel_length = 0.018
    forearm.visual(
        Cylinder(radius=forearm_barrel_radius, length=forearm_barrel_length),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=aluminum,
        name="forearm_barrel",
    )

    forearm_pitch = 0.62
    forearm_beam_len = 0.16
    forearm_beam_size = (0.012, 0.010, forearm_beam_len)
    forearm_beam_center = _rotate_y(0.0, 0.0, forearm_beam_len * 0.5, forearm_pitch)
    forearm.visual(
        Box(forearm_beam_size),
        origin=Origin(xyz=forearm_beam_center, rpy=(0.0, forearm_pitch, 0.0)),
        material=aluminum,
        name="forearm_beam",
    )

    head_joint_local_z = 0.170
    head_joint_center = _rotate_y(0.0, 0.0, head_joint_local_z, forearm_pitch)
    head_cheek_local_z = 0.164
    head_cheek_center = _rotate_y(0.0, 0.0, head_cheek_local_z, forearm_pitch)
    forearm_head_cheek_size = (0.014, 0.0045, 0.022)
    for index, y in enumerate((-0.0085, 0.0085)):
        forearm.visual(
            Box(forearm_head_cheek_size),
            origin=Origin(
                xyz=(head_cheek_center[0], y, head_cheek_center[2]),
                rpy=(0.0, forearm_pitch, 0.0),
            ),
            material=aluminum,
            name=f"head_cheek_{index}",
        )
    head_mount_center = _rotate_y(0.0, 0.0, 0.152, forearm_pitch)
    forearm.visual(
        Box((0.010, 0.018, 0.020)),
        origin=Origin(xyz=head_mount_center, rpy=(0.0, forearm_pitch, 0.0)),
        material=aluminum,
        name="head_mount",
    )

    head = model.part("head")
    head_barrel_radius = 0.004
    head_barrel_length = 0.0125
    head.visual(
        Cylinder(radius=head_barrel_radius, length=head_barrel_length),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=aluminum,
        name="head_barrel",
    )
    head.visual(
        Box((0.012, 0.010, 0.010)),
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
        material=aluminum,
        name="head_neck",
    )

    head_start = 0.01
    head_len = 0.08
    head_width = 0.034
    head_height = 0.022
    wall = 0.0025

    head.visual(
        Box((head_len, head_width, wall)),
        origin=Origin(xyz=(head_start + head_len * 0.5, 0.0, head_height * 0.5 - wall * 0.5)),
        material=graphite,
        name="head_top",
    )
    head.visual(
        Box((0.078, 0.029, 0.002)),
        origin=Origin(xyz=(head_start + head_len * 0.5, 0.0, -head_height * 0.5 + 0.001)),
        material=diffuser,
        name="diffuser_panel",
    )
    head.visual(
        Box((wall, head_width, head_height)),
        origin=Origin(xyz=(head_start + wall * 0.5, 0.0, 0.0)),
        material=graphite,
        name="rear_wall",
    )
    head.visual(
        Box((wall, head_width, head_height)),
        origin=Origin(xyz=(head_start + head_len - wall * 0.5, 0.0, 0.0)),
        material=graphite,
        name="front_bezel",
    )
    head.visual(
        Box((head_len, wall, head_height)),
        origin=Origin(xyz=(head_start + head_len * 0.5, -head_width * 0.5 + wall * 0.5, 0.0)),
        material=graphite,
        name="left_wall",
    )

    button_opening_x = 0.029
    button_opening_len = 0.022
    side_wall_y = head_width * 0.5 - wall * 0.5
    side_segment_rear_len = button_opening_x - button_opening_len * 0.5 - head_start
    side_segment_front_len = head_start + head_len - (button_opening_x + button_opening_len * 0.5)
    rear_segment_center_x = head_start + side_segment_rear_len * 0.5
    front_segment_center_x = head_start + head_len - side_segment_front_len * 0.5

    head.visual(
        Box((side_segment_rear_len, wall, head_height)),
        origin=Origin(xyz=(rear_segment_center_x, side_wall_y, 0.0)),
        material=graphite,
        name="button_side_rear",
    )
    head.visual(
        Box((side_segment_front_len, wall, head_height)),
        origin=Origin(xyz=(front_segment_center_x, side_wall_y, 0.0)),
        material=graphite,
        name="button_side_front",
    )
    head.visual(
        Box((button_opening_len, wall, 0.006)),
        origin=Origin(xyz=(button_opening_x, side_wall_y, 0.008)),
        material=graphite,
        name="button_side_top",
    )
    head.visual(
        Box((button_opening_len, wall, 0.006)),
        origin=Origin(xyz=(button_opening_x, side_wall_y, -0.008)),
        material=graphite,
        name="button_side_bottom",
    )

    side_button = model.part("side_button")
    side_button.visual(
        Box((0.024, 0.002, 0.012)),
        origin=Origin(xyz=(0.0, 0.0035, 0.0)),
        material=accent,
        name="button_cap",
    )
    side_button.visual(
        Box((0.009, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, -0.0025, 0.0)),
        material=accent,
        name="button_stem",
    )

    model.articulation(
        "base_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upright,
        origin=Origin(xyz=(pedestal_x, 0.0, base_hinge_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.2, lower=-0.6, upper=0.75),
    )
    model.articulation(
        "elbow_hinge",
        ArticulationType.REVOLUTE,
        parent=upright,
        child=forearm,
        origin=Origin(xyz=elbow_center),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.8, lower=-1.0, upper=0.9),
    )
    model.articulation(
        "head_hinge",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=head,
        origin=Origin(xyz=head_joint_center),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=-0.8, upper=0.7),
    )
    model.articulation(
        "button_slide",
        ArticulationType.PRISMATIC,
        parent=head,
        child=side_button,
        origin=Origin(xyz=(button_opening_x, head_width * 0.5 - wall, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.04, lower=-0.002, upper=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    upright = object_model.get_part("upright")
    forearm = object_model.get_part("forearm")
    head = object_model.get_part("head")
    side_button = object_model.get_part("side_button")

    base_hinge = object_model.get_articulation("base_hinge")
    elbow_hinge = object_model.get_articulation("elbow_hinge")
    head_hinge = object_model.get_articulation("head_hinge")
    button_slide = object_model.get_articulation("button_slide")

    head_center = _aabb_center(ctx.part_world_aabb(head))
    base_aabb = ctx.part_world_aabb(base)
    head_top = base_aabb[1][2] if base_aabb is not None else None
    ctx.check(
        "head sits well above the desk base",
        head_center is not None and head_top is not None and head_center[2] > head_top + 0.20,
        details=f"head_center={head_center}, base_aabb={base_aabb}",
    )

    button_center = _aabb_center(ctx.part_element_world_aabb(side_button, elem="button_cap"))
    head_aabb = ctx.part_world_aabb(head)
    head_side_y = head_aabb[1][1] if head_aabb is not None else None
    ctx.check(
        "button cap protrudes from the head side",
        button_center is not None and head_side_y is not None and button_center[1] > head_side_y + 0.0005,
        details=f"button_center={button_center}, head_aabb={head_aabb}",
    )

    upright_rest = _aabb_center(ctx.part_element_world_aabb(upright, elem="upright_beam"))
    with ctx.pose({base_hinge: 0.45}):
        upright_forward = _aabb_center(ctx.part_element_world_aabb(upright, elem="upright_beam"))
    ctx.check(
        "upright swings forward from the base hinge",
        upright_rest is not None and upright_forward is not None and upright_forward[0] > upright_rest[0] + 0.03,
        details=f"rest={upright_rest}, forward={upright_forward}",
    )

    head_rest = _aabb_center(ctx.part_world_aabb(head))
    with ctx.pose({elbow_hinge: 0.45}):
        head_reached = _aabb_center(ctx.part_world_aabb(head))
    ctx.check(
        "forearm hinge carries the head farther forward",
        head_rest is not None and head_reached is not None and head_reached[0] > head_rest[0] + 0.025,
        details=f"rest={head_rest}, reached={head_reached}",
    )

    front_rest = _aabb_center(ctx.part_element_world_aabb(head, elem="front_bezel"))
    with ctx.pose({head_hinge: 0.45}):
        front_tilted = _aabb_center(ctx.part_element_world_aabb(head, elem="front_bezel"))
    ctx.check(
        "head hinge tilts the light downward",
        front_rest is not None and front_tilted is not None and front_tilted[2] < front_rest[2] - 0.01,
        details=f"rest={front_rest}, tilted={front_tilted}",
    )

    button_rest = _aabb_center(ctx.part_element_world_aabb(side_button, elem="button_cap"))
    with ctx.pose({button_slide: -0.0015}):
        button_pressed = _aabb_center(ctx.part_element_world_aabb(side_button, elem="button_cap"))
    ctx.check(
        "button depresses inward",
        button_rest is not None and button_pressed is not None and button_pressed[1] < button_rest[1] - 0.001,
        details=f"rest={button_rest}, pressed={button_pressed}",
    )

    button_cap_aabb = ctx.part_element_world_aabb(side_button, elem="button_cap")
    head_body_aabb = ctx.part_world_aabb(head)
    button_cap_center = _aabb_center(button_cap_aabb)
    ctx.check(
        "button stays within the head length and height envelope",
        button_cap_aabb is not None
        and head_body_aabb is not None
        and button_cap_center is not None
        and head_body_aabb[0][0] <= button_cap_center[0] <= head_body_aabb[1][0]
        and head_body_aabb[0][2] <= button_cap_center[2] <= head_body_aabb[1][2],
        details=f"button_cap_aabb={button_cap_aabb}, head_body_aabb={head_body_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
