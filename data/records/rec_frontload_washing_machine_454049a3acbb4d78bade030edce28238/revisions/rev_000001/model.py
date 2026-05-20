from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _circle_profile(radius: float, *, segments: int = 72, center=(0.0, 0.0)):
    cx, cy = center
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * i / segments),
            cy + radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _annular_plate_mesh(
    outer_radius: float,
    inner_radius: float,
    depth: float,
    name: str,
    *,
    center=(0.0, 0.0, 0.0),
):
    """Flat annulus in the local XZ plane, with thickness along local Y."""
    geom = ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius),
        [_circle_profile(inner_radius)],
        depth,
        center=True,
    )
    # ExtrudeWithHolesGeometry is built in local XY with depth along Z.
    # Rotate so the circular face becomes XZ and the depth becomes Y.
    geom.rotate_x(math.pi / 2.0)
    geom.translate(*center)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stacked_washer_unit")

    white = Material("warm_white_enamel", rgba=(0.93, 0.94, 0.92, 1.0))
    seam = Material("soft_gray_trim", rgba=(0.56, 0.58, 0.58, 1.0))
    dark = Material("black_rubber", rgba=(0.015, 0.016, 0.017, 1.0))
    steel = Material("brushed_steel", rgba=(0.58, 0.62, 0.64, 1.0))
    dark_steel = Material("dark_perforated_steel", rgba=(0.30, 0.32, 0.33, 1.0))
    glass = Material("smoky_blue_glass", rgba=(0.33, 0.48, 0.58, 0.42))
    display = Material("black_glass_display", rgba=(0.02, 0.03, 0.035, 1.0))

    width = 0.64
    depth = 0.64
    height = 1.56
    wall = 0.032
    front_skin = 0.025
    lower_center_z = 0.40
    upper_center_z = 1.17
    door_center_z = lower_center_z
    front_panel_y = -depth / 2.0 - front_skin / 2.0

    cabinet = model.part("cabinet")

    # White enamel stacked case: two compact square laundry modules on one shell.
    cabinet.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(-width / 2.0 + wall / 2.0, 0.0, height / 2.0)),
        material=white,
        name="side_panel_0",
    )
    cabinet.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(width / 2.0 - wall / 2.0, 0.0, height / 2.0)),
        material=white,
        name="side_panel_1",
    )
    cabinet.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, wall / 2.0)),
        material=white,
        name="base_panel",
    )
    cabinet.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, height - wall / 2.0)),
        material=white,
        name="top_panel",
    )
    cabinet.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, 0.78)),
        material=white,
        name="module_divider",
    )
    cabinet.visual(
        Box((width, wall, height)),
        origin=Origin(xyz=(0.0, depth / 2.0 - wall / 2.0, height / 2.0)),
        material=white,
        name="rear_panel",
    )

    # Lower front skin is a real plate with a circular porthole opening.
    lower_front = ExtrudeWithHolesGeometry(
        rounded_rect_profile(width, 0.72, 0.026, corner_segments=8),
        [_circle_profile(0.238, segments=96)],
        front_skin,
        center=True,
    )
    lower_front.rotate_x(math.pi / 2.0)
    lower_front.translate(0.0, front_panel_y, lower_center_z)
    cabinet.visual(
        mesh_from_geometry(lower_front, "lower_front_panel"),
        material=white,
        name="lower_front_panel",
    )

    cabinet.visual(
        Box((width, front_skin, 0.72)),
        origin=Origin(xyz=(0.0, front_panel_y, upper_center_z)),
        material=white,
        name="upper_front_panel",
    )
    cabinet.visual(
        Box((width * 0.92, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, -depth / 2.0 - 0.006, 0.78)),
        material=seam,
        name="stacking_seam",
    )

    # Rubber boot around the washer opening, slightly proud of the lower skin.
    gasket_mesh = _annular_plate_mesh(
        0.252,
        0.214,
        0.018,
        "front_gasket_mesh",
        center=(0.0, -depth / 2.0 - front_skin - 0.002, door_center_z),
    )
    cabinet.visual(gasket_mesh, material=dark, name="front_gasket")

    # Upper dryer-style panel treatment and a modest control fascia.
    cabinet.visual(
        Box((0.50, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, -depth / 2.0 - 0.020, 1.26)),
        material=seam,
        name="upper_trim_top",
    )
    cabinet.visual(
        Box((0.50, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, -depth / 2.0 - 0.020, 0.98)),
        material=seam,
        name="upper_trim_bottom",
    )
    cabinet.visual(
        Box((0.012, 0.010, 0.28)),
        origin=Origin(xyz=(-0.25, -depth / 2.0 - 0.020, 1.12)),
        material=seam,
        name="upper_trim_0",
    )
    cabinet.visual(
        Box((0.012, 0.010, 0.28)),
        origin=Origin(xyz=(0.25, -depth / 2.0 - 0.020, 1.12)),
        material=seam,
        name="upper_trim_1",
    )
    cabinet.visual(
        Box((0.54, 0.014, 0.095)),
        origin=Origin(xyz=(0.0, -depth / 2.0 - 0.018, 1.43)),
        material=seam,
        name="control_fascia",
    )
    cabinet.visual(
        Box((0.19, 0.010, 0.045)),
        origin=Origin(xyz=(-0.09, -depth / 2.0 - 0.024, 1.43)),
        material=display,
        name="display_window",
    )

    # Visible mounts: left door hinge leaf and rear bearing for the spinning drum.
    cabinet.visual(
        Box((0.040, 0.018, 0.46)),
        origin=Origin(xyz=(-0.225, -depth / 2.0 - 0.015, door_center_z)),
        material=steel,
        name="hinge_leaf",
    )
    cabinet.visual(
        Cylinder(radius=0.048, length=0.070),
        origin=Origin(xyz=(0.0, 0.285, door_center_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="rear_bearing",
    )

    # The drum is a hollow metal cylinder on a continuous front-to-back axle.
    drum = model.part("drum")
    drum_shell = LatheGeometry.from_shell_profiles(
        [(0.223, -0.18), (0.223, 0.18)],
        [(0.200, -0.18), (0.200, 0.18)],
        segments=96,
        start_cap="flat",
        end_cap="flat",
    )
    drum_shell.rotate_x(math.pi / 2.0)
    drum.visual(
        mesh_from_geometry(drum_shell, "drum_shell_mesh"),
        material=steel,
        name="drum_shell",
    )
    drum.visual(
        Cylinder(radius=0.207, length=0.016),
        origin=Origin(xyz=(0.0, 0.178, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="drum_back",
    )
    for i, theta in enumerate((math.pi / 2.0, 7.0 * math.pi / 6.0, 11.0 * math.pi / 6.0)):
        radial_x = math.cos(theta)
        radial_z = math.sin(theta)
        baffle_angle = math.pi / 2.0 - theta
        drum.visual(
            Box((0.075, 0.285, 0.045)),
            origin=Origin(
                xyz=(0.181 * radial_x, 0.0, 0.181 * radial_z),
                rpy=(0.0, baffle_angle, 0.0),
            ),
            material=dark_steel,
            name=f"drum_baffle_{i}",
        )
    drum.visual(
        Cylinder(radius=0.022, length=0.070),
        origin=Origin(xyz=(0.0, 0.215, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="drum_axle",
    )

    model.articulation(
        "cabinet_to_drum",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=drum,
        origin=Origin(xyz=(0.0, 0.0, door_center_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=12.0),
    )

    # Round porthole door, framed in white with a blue smoked glass window.
    door = model.part("door")
    door_outer_radius = 0.205
    door.visual(
        _annular_plate_mesh(
            door_outer_radius,
            0.128,
            0.040,
            "door_ring_mesh",
            center=(door_outer_radius, 0.0, 0.0),
        ),
        material=white,
        name="door_ring",
    )
    door.visual(
        Cylinder(radius=0.134, length=0.014),
        origin=Origin(xyz=(door_outer_radius, -0.016, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=glass,
        name="porthole_glass",
    )
    door.visual(
        Cylinder(radius=0.017, length=0.130),
        origin=Origin(xyz=(0.0, -0.003, 0.130)),
        material=steel,
        name="hinge_knuckle_0",
    )
    door.visual(
        Cylinder(radius=0.017, length=0.130),
        origin=Origin(xyz=(0.0, -0.003, -0.130)),
        material=steel,
        name="hinge_knuckle_1",
    )
    door.visual(
        Box((0.038, 0.020, 0.190)),
        origin=Origin(xyz=(0.024, 0.004, 0.0)),
        material=steel,
        name="door_hinge_leaf",
    )

    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(-door_outer_radius, -0.372, door_center_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=0.0, upper=1.75),
    )

    # Flip-up latch handle carried by the porthole door.
    latch = model.part("latch")
    latch.visual(
        Cylinder(radius=0.020, length=0.060),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_pin",
    )
    latch.visual(
        Box((0.125, 0.024, 0.035)),
        origin=Origin(xyz=(0.072, -0.017, 0.0)),
        material=dark,
        name="latch_grip",
    )
    latch.visual(
        Cylinder(radius=0.024, length=0.012),
        origin=Origin(xyz=(0.0, -0.036, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="latch_boss",
    )

    model.articulation(
        "door_to_latch",
        ArticulationType.REVOLUTE,
        parent=door,
        child=latch,
        origin=Origin(xyz=(0.355, -0.035, 0.075)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=1.25),
    )

    # A single rotary program selector on the front control fascia.
    selector = model.part("selector")
    selector.visual(
        Cylinder(radius=0.036, length=0.026),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=white,
        name="selector_cap",
    )
    selector.visual(
        Box((0.006, 0.006, 0.044)),
        origin=Origin(xyz=(0.0, -0.012, 0.012)),
        material=dark,
        name="selector_mark",
    )

    model.articulation(
        "cabinet_to_selector",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=selector,
        origin=Origin(xyz=(0.205, -0.358, 1.43)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    latch = object_model.get_part("latch")
    selector = object_model.get_part("selector")

    drum_joint = object_model.get_articulation("cabinet_to_drum")
    door_joint = object_model.get_articulation("cabinet_to_door")
    latch_joint = object_model.get_articulation("door_to_latch")
    selector_joint = object_model.get_articulation("cabinet_to_selector")

    ctx.allow_overlap(
        latch,
        door,
        elem_a="pivot_pin",
        elem_b="door_ring",
        reason="The flip-up latch pivot pin is intentionally captured through the porthole door ring.",
    )
    ctx.expect_overlap(
        latch,
        door,
        axes="xz",
        elem_a="pivot_pin",
        elem_b="door_ring",
        min_overlap=0.010,
        name="latch pivot lands through the door ring",
    )
    ctx.expect_gap(
        door,
        latch,
        axis="y",
        positive_elem="door_ring",
        negative_elem="pivot_pin",
        max_penetration=0.020,
        name="latch pin embedding is shallow",
    )

    ctx.check(
        "drum uses a front-to-back continuous axle",
        drum_joint.articulation_type == ArticulationType.CONTINUOUS
        and abs(drum_joint.axis[1]) > 0.95,
        details=f"type={drum_joint.articulation_type}, axis={drum_joint.axis}",
    )
    ctx.check(
        "selector is a rotary control",
        selector_joint.articulation_type == ArticulationType.CONTINUOUS
        and abs(selector_joint.axis[1]) > 0.95,
        details=f"type={selector_joint.articulation_type}, axis={selector_joint.axis}",
    )
    ctx.check(
        "door hinge has realistic swing limits",
        door_joint.motion_limits is not None
        and door_joint.motion_limits.lower == 0.0
        and door_joint.motion_limits.upper is not None
        and 1.4 < door_joint.motion_limits.upper < 2.1,
        details=f"limits={door_joint.motion_limits}",
    )
    ctx.check(
        "latch handle has flip-up limits",
        latch_joint.motion_limits is not None
        and latch_joint.motion_limits.lower == 0.0
        and latch_joint.motion_limits.upper is not None
        and 0.9 < latch_joint.motion_limits.upper < 1.5,
        details=f"limits={latch_joint.motion_limits}",
    )

    ctx.expect_overlap(
        drum,
        door,
        axes="xz",
        elem_a="drum_shell",
        elem_b="porthole_glass",
        min_overlap=0.22,
        name="drum is visible behind the round porthole",
    )
    ctx.expect_within(
        drum,
        cabinet,
        axes="xz",
        inner_elem="drum_shell",
        outer_elem="lower_front_panel",
        margin=0.015,
        name="drum fits within the lower square module",
    )

    rest_door_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_joint: 1.20}):
        opened_door_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door opens forward from its left hinge",
        rest_door_aabb is not None
        and opened_door_aabb is not None
        and opened_door_aabb[0][1] < rest_door_aabb[0][1] - 0.05,
        details=f"rest={rest_door_aabb}, opened={opened_door_aabb}",
    )

    rest_latch_aabb = ctx.part_world_aabb(latch)
    with ctx.pose({latch_joint: 1.05}):
        raised_latch_aabb = ctx.part_world_aabb(latch)
    ctx.check(
        "latch flips upward",
        rest_latch_aabb is not None
        and raised_latch_aabb is not None
        and raised_latch_aabb[1][2] > rest_latch_aabb[1][2] + 0.040,
        details=f"rest={rest_latch_aabb}, raised={raised_latch_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
