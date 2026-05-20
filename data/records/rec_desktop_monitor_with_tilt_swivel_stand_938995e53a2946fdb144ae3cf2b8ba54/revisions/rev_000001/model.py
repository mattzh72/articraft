from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_desktop_monitor")

    satin_black = model.material("satin_black", rgba=(0.015, 0.016, 0.018, 1.0))
    graphite = model.material("graphite", rgba=(0.12, 0.13, 0.145, 1.0))
    dark_graphite = model.material("dark_graphite", rgba=(0.055, 0.060, 0.068, 1.0))
    glass = model.material("screen_glass", rgba=(0.015, 0.025, 0.035, 1.0))
    shadow_glass = model.material("subtle_lcd", rgba=(0.030, 0.055, 0.075, 1.0))
    brushed_metal = model.material("brushed_metal", rgba=(0.48, 0.50, 0.52, 1.0))
    rubber = model.material("rubber", rgba=(0.006, 0.006, 0.007, 1.0))

    base = model.part("base")
    base_plate = ExtrudeGeometry(
        rounded_rect_profile(0.420, 0.285, 0.085, corner_segments=10),
        0.034,
    )
    base.visual(
        mesh_from_geometry(base_plate, "base_plate"),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=dark_graphite,
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=0.080, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        material=graphite,
        name="swivel_socket",
    )
    base.visual(
        Cylinder(radius=0.060, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.057)),
        material=brushed_metal,
        name="bearing_ring",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.420, 0.285, 0.060)),
        mass=5.5,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
    )

    stand_spine = model.part("stand_spine")
    stand_spine.visual(
        Cylinder(radius=0.062, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=graphite,
        name="turntable_cap",
    )

    column_geom = ExtrudeGeometry(
        rounded_rect_profile(0.112, 0.072, 0.018, corner_segments=8),
        0.540,
    )
    stand_spine.visual(
        mesh_from_geometry(column_geom, "broad_column"),
        origin=Origin(xyz=(0.0, 0.070, 0.295)),
        material=graphite,
        name="broad_column",
    )
    stand_spine.visual(
        Box((0.084, 0.004, 0.370)),
        origin=Origin(xyz=(0.006, 0.108, 0.315)),
        material=satin_black,
        name="cable_recess",
    )
    stand_spine.visual(
        Box((0.012, 0.008, 0.380)),
        origin=Origin(xyz=(-0.046, 0.108, 0.315)),
        material=brushed_metal,
        name="door_hinge_socket",
    )

    head_geom = ExtrudeGeometry(
        rounded_rect_profile(0.220, 0.110, 0.032, corner_segments=10),
        0.150,
    )
    stand_spine.visual(
        mesh_from_geometry(head_geom, "deep_stand_head"),
        origin=Origin(xyz=(0.0, 0.050, 0.620)),
        material=graphite,
        name="deep_stand_head",
    )
    stand_spine.visual(
        Box((0.026, 0.054, 0.092)),
        origin=Origin(xyz=(-0.083, -0.026, 0.640)),
        material=graphite,
        name="tilt_yoke_side_0",
    )
    stand_spine.visual(
        Box((0.026, 0.054, 0.092)),
        origin=Origin(xyz=(0.083, -0.026, 0.640)),
        material=graphite,
        name="tilt_yoke_side_1",
    )
    stand_spine.inertial = Inertial.from_geometry(
        Box((0.240, 0.180, 0.720)),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.055, 0.350)),
    )

    cable_door = model.part("cable_door")
    cable_door.visual(
        Box((0.078, 0.008, 0.340)),
        origin=Origin(xyz=(0.042, 0.004, 0.0)),
        material=dark_graphite,
        name="door_panel",
    )
    cable_door.visual(
        Cylinder(radius=0.006, length=0.350),
        origin=Origin(),
        material=brushed_metal,
        name="door_hinge_barrel",
    )
    cable_door.visual(
        Box((0.012, 0.006, 0.150)),
        origin=Origin(xyz=(0.074, 0.009, 0.0)),
        material=satin_black,
        name="finger_lip",
    )
    cable_door.inertial = Inertial.from_geometry(
        Box((0.086, 0.012, 0.350)),
        mass=0.18,
        origin=Origin(xyz=(0.040, 0.006, 0.0)),
    )

    tilt_carriage = model.part("tilt_carriage")
    tilt_carriage.visual(
        Cylinder(radius=0.013, length=0.185),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_metal,
        name="tilt_axle",
    )
    for side, x in (("side_0", -0.062), ("side_1", 0.062)):
        tilt_carriage.visual(
            Box((0.026, 0.058, 0.030)),
            origin=Origin(xyz=(x, -0.029, 0.0)),
            material=dark_graphite,
            name=f"tilt_arm_{side}",
        )
    tilt_carriage.visual(
        Cylinder(radius=0.048, length=0.026),
        origin=Origin(xyz=(0.0, -0.050, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_graphite,
        name="pivot_bearing",
    )
    tilt_carriage.visual(
        Box((0.150, 0.018, 0.028)),
        origin=Origin(xyz=(0.0, -0.050, 0.0)),
        material=dark_graphite,
        name="pivot_bridge",
    )
    tilt_carriage.visual(
        Box((0.170, 0.012, 0.125)),
        origin=Origin(xyz=(0.0, -0.066, 0.0)),
        material=dark_graphite,
        name="vesa_plate",
    )
    tilt_carriage.inertial = Inertial.from_geometry(
        Box((0.200, 0.120, 0.140)),
        mass=0.9,
        origin=Origin(xyz=(0.0, -0.052, 0.0)),
    )

    display = model.part("display_shell")
    rear_shell = ExtrudeGeometry(
        rounded_rect_profile(0.640, 0.375, 0.026, corner_segments=10),
        0.044,
    )
    rear_shell.rotate_x(math.pi / 2.0)
    display.visual(
        mesh_from_geometry(rear_shell, "rear_shell"),
        origin=Origin(),
        material=dark_graphite,
        name="rear_shell",
    )
    bezel = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.636, 0.371, 0.024, corner_segments=10),
        [rounded_rect_profile(0.590, 0.329, 0.010, corner_segments=8)],
        0.008,
    )
    bezel.rotate_x(math.pi / 2.0)
    display.visual(
        mesh_from_geometry(bezel, "front_bezel"),
        origin=Origin(xyz=(0.0, -0.023, 0.0)),
        material=satin_black,
        name="front_bezel",
    )
    display.visual(
        Box((0.606, 0.004, 0.344)),
        origin=Origin(xyz=(0.0, -0.027, 0.0)),
        material=glass,
        name="glass_panel",
    )
    display.visual(
        Box((0.588, 0.002, 0.326)),
        origin=Origin(xyz=(0.0, -0.030, 0.0)),
        material=shadow_glass,
        name="active_lcd_area",
    )
    display.visual(
        Box((0.150, 0.010, 0.105)),
        origin=Origin(xyz=(0.0, 0.027, 0.0)),
        material=graphite,
        name="rear_mount_pad",
    )
    display.inertial = Inertial.from_geometry(
        Box((0.650, 0.055, 0.385)),
        mass=4.0,
        origin=Origin(),
    )

    model.articulation(
        "base_to_stand",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=stand_spine,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.6),
    )
    model.articulation(
        "stand_to_cable_door",
        ArticulationType.REVOLUTE,
        parent=stand_spine,
        child=cable_door,
        origin=Origin(xyz=(-0.046, 0.118, 0.315)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=0.0, upper=1.75),
    )
    model.articulation(
        "stand_to_tilt",
        ArticulationType.REVOLUTE,
        parent=stand_spine,
        child=tilt_carriage,
        origin=Origin(xyz=(0.0, -0.026, 0.640)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.4,
            lower=math.radians(-6.0),
            upper=math.radians(22.0),
        ),
    )
    model.articulation(
        "tilt_to_display",
        ArticulationType.CONTINUOUS,
        parent=tilt_carriage,
        child=display,
        origin=Origin(xyz=(0.0, -0.104, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    stand = object_model.get_part("stand_spine")
    cable_door = object_model.get_part("cable_door")
    tilt = object_model.get_part("tilt_carriage")
    display = object_model.get_part("display_shell")

    stand_swivel = object_model.get_articulation("base_to_stand")
    door_hinge = object_model.get_articulation("stand_to_cable_door")
    tilt_hinge = object_model.get_articulation("stand_to_tilt")
    portrait_pivot = object_model.get_articulation("tilt_to_display")

    ctx.allow_overlap(
        stand,
        tilt,
        elem_a="tilt_yoke_side_0",
        elem_b="tilt_axle",
        reason="The tilt axle is intentionally captured inside the stand-head yoke lug.",
    )
    ctx.allow_overlap(
        stand,
        tilt,
        elem_a="tilt_yoke_side_1",
        elem_b="tilt_axle",
        reason="The tilt axle is intentionally captured inside the stand-head yoke lug.",
    )

    ctx.check(
        "primary articulations are present",
        stand_swivel.articulation_type == ArticulationType.CONTINUOUS
        and door_hinge.articulation_type == ArticulationType.REVOLUTE
        and tilt_hinge.articulation_type == ArticulationType.REVOLUTE
        and portrait_pivot.articulation_type == ArticulationType.CONTINUOUS,
        details="Expected stand swivel, cable-door hinge, display tilt, and portrait pivot.",
    )
    ctx.check(
        "tilt range is monitor realistic",
        tilt_hinge.motion_limits is not None
        and tilt_hinge.motion_limits.lower < 0.0
        and tilt_hinge.motion_limits.upper > math.radians(15.0),
        details=f"tilt limits={tilt_hinge.motion_limits}",
    )
    ctx.check(
        "cable door opens on a vertical edge",
        door_hinge.axis == (0.0, 0.0, 1.0)
        and door_hinge.motion_limits is not None
        and door_hinge.motion_limits.upper > 1.2,
        details=f"axis={door_hinge.axis}, limits={door_hinge.motion_limits}",
    )

    ctx.expect_contact(
        cable_door,
        stand,
        elem_a="door_hinge_barrel",
        elem_b="door_hinge_socket",
        contact_tol=0.003,
        name="cable door hinge is carried by the stand spine",
    )
    ctx.expect_gap(
        cable_door,
        stand,
        axis="y",
        positive_elem="door_panel",
        negative_elem="cable_recess",
        max_gap=0.012,
        max_penetration=0.0,
        name="closed cable door sits proud of the rear cable recess",
    )
    ctx.expect_contact(
        tilt,
        display,
        elem_a="vesa_plate",
        elem_b="rear_mount_pad",
        contact_tol=0.006,
        name="pivot plate bears against the screen rear mount pad",
    )
    ctx.expect_overlap(
        display,
        tilt,
        axes="xz",
        elem_a="rear_mount_pad",
        elem_b="vesa_plate",
        min_overlap=0.090,
        name="rear mount pad overlaps the VESA plate footprint",
    )
    ctx.expect_contact(
        stand,
        tilt,
        elem_a="tilt_yoke_side_0",
        elem_b="tilt_axle",
        contact_tol=0.004,
        name="tilt axle is supported by one yoke lug",
    )
    ctx.expect_contact(
        stand,
        tilt,
        elem_a="tilt_yoke_side_1",
        elem_b="tilt_axle",
        contact_tol=0.004,
        name="tilt axle is supported by the other yoke lug",
    )

    rest_display_pos = ctx.part_world_position(display)
    with ctx.pose({tilt_hinge: math.radians(20.0)}):
        tilted_display_pos = ctx.part_world_position(display)
    ctx.check(
        "positive tilt moves the screen about the stand head",
        rest_display_pos is not None
        and tilted_display_pos is not None
        and abs(tilted_display_pos[2] - rest_display_pos[2]) > 0.010,
        details=f"rest={rest_display_pos}, tilted={tilted_display_pos}",
    )

    with ctx.pose({door_hinge: 1.20}):
        opened_aabb = ctx.part_world_aabb(cable_door)
    closed_aabb = ctx.part_world_aabb(cable_door)
    ctx.check(
        "cable door swings outward",
        opened_aabb is not None
        and closed_aabb is not None
        and opened_aabb[1][1] > closed_aabb[1][1] + 0.035,
        details=f"closed={closed_aabb}, opened={opened_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
