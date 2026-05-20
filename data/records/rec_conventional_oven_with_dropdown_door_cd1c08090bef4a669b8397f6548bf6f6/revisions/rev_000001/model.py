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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rotisserie_countertop_oven")

    stainless = model.material("brushed_stainless", rgba=(0.68, 0.70, 0.68, 1.0))
    dark = model.material("matte_black_trim", rgba=(0.02, 0.022, 0.024, 1.0))
    dark_gray = model.material("dark_interior", rgba=(0.06, 0.055, 0.05, 1.0))
    glass = model.material("smoked_glass", rgba=(0.16, 0.23, 0.28, 0.38))
    warm_metal = model.material("warm_rotisserie_steel", rgba=(0.84, 0.78, 0.66, 1.0))
    copper_heat = model.material("copper_heating_element", rgba=(0.95, 0.30, 0.08, 1.0))
    white = model.material("white_markings", rgba=(0.90, 0.92, 0.88, 1.0))

    housing = model.part("housing")

    # Coordinate frame: +X goes into the oven, +Y is the wide horizontal span,
    # +Z is up.  The front opening is at X ~= 0.
    housing.visual(
        Box((0.42, 0.62, 0.030)),
        origin=Origin(xyz=(0.210, 0.0, 0.015)),
        material=stainless,
        name="bottom_shell",
    )
    housing.visual(
        Box((0.42, 0.62, 0.030)),
        origin=Origin(xyz=(0.210, 0.0, 0.305)),
        material=stainless,
        name="top_shell",
    )
    housing.visual(
        Box((0.42, 0.030, 0.320)),
        origin=Origin(xyz=(0.210, -0.295, 0.160)),
        material=stainless,
        name="side_wall_0",
    )
    housing.visual(
        Box((0.42, 0.030, 0.320)),
        origin=Origin(xyz=(0.210, 0.295, 0.160)),
        material=stainless,
        name="side_wall_1",
    )
    housing.visual(
        Box((0.030, 0.62, 0.320)),
        origin=Origin(xyz=(0.405, 0.0, 0.160)),
        material=dark_gray,
        name="rear_wall",
    )
    housing.visual(
        Box((0.33, 0.53, 0.012)),
        origin=Origin(xyz=(0.195, -0.045, 0.044)),
        material=dark_gray,
        name="oven_floor",
    )

    # Front bezel and the fixed right-side control fascia leave a clear, wide
    # rectangular door opening into the hollow oven cavity.
    housing.visual(
        Box((0.020, 0.515, 0.026)),
        origin=Origin(xyz=(0.004, -0.052, 0.292)),
        material=dark,
        name="front_top_bezel",
    )
    housing.visual(
        Box((0.032, 0.515, 0.026)),
        origin=Origin(xyz=(0.004, -0.052, 0.050)),
        material=dark,
        name="front_bottom_bezel",
    )
    housing.visual(
        Box((0.020, 0.025, 0.268)),
        origin=Origin(xyz=(0.004, -0.300, 0.171)),
        material=dark,
        name="front_side_bezel_0",
    )
    housing.visual(
        Box((0.020, 0.025, 0.268)),
        origin=Origin(xyz=(0.004, 0.196, 0.171)),
        material=dark,
        name="front_side_bezel_1",
    )
    housing.visual(
        Box((0.024, 0.105, 0.278)),
        origin=Origin(xyz=(-0.002, 0.248, 0.171)),
        material=dark,
        name="control_fascia",
    )

    # Simple shelves/heating elements are fixed to the side walls so the hollow
    # interior reads as an oven instead of an empty box.
    for z, name in ((0.083, "lower_heater"), (0.257, "upper_heater")):
        housing.visual(
            Cylinder(radius=0.006, length=0.560),
            origin=Origin(xyz=(0.145, 0.0, z), rpy=(-math.pi / 2, 0.0, 0.0)),
            material=copper_heat,
            name=name,
        )
    for y, name in ((-0.277, "spit_socket_0"), (0.277, "spit_socket_1")):
        housing.visual(
            Cylinder(radius=0.026, length=0.014),
            origin=Origin(xyz=(0.190, y, 0.178), rpy=(-math.pi / 2, 0.0, 0.0)),
            material=warm_metal,
            name=name,
        )

    # Small feet connect under the shell and lift the countertop appliance.
    for x in (0.070, 0.350):
        for y in (-0.225, 0.225):
            housing.visual(
                Box((0.060, 0.060, 0.026)),
                origin=Origin(xyz=(x, y, -0.013)),
                material=dark,
                name=f"rubber_foot_{x:.2f}_{y:.2f}",
            )

    # White tick marks around the two front rotary controls.
    tick_positions = [
        (-0.016, 0.246, 0.263, 0.006, 0.002, 0.022),
        (-0.016, 0.246, 0.187, 0.006, 0.002, 0.022),
        (-0.016, 0.205, 0.225, 0.006, 0.002, 0.018),
        (-0.016, 0.287, 0.225, 0.006, 0.002, 0.018),
        (-0.016, 0.246, 0.163, 0.006, 0.002, 0.022),
        (-0.016, 0.246, 0.087, 0.006, 0.002, 0.022),
        (-0.016, 0.205, 0.125, 0.006, 0.002, 0.018),
        (-0.016, 0.287, 0.125, 0.006, 0.002, 0.018),
    ]
    for i, (x, y, z, sx, sy, sz) in enumerate(tick_positions):
        housing.visual(
            Box((sx, sy, sz)),
            origin=Origin(xyz=(x, y, z)),
            material=white,
            name=f"control_tick_{i}",
        )

    door = model.part("door")
    door_width = 0.480
    door_center_y = -0.055
    door.visual(
        Cylinder(radius=0.012, length=door_width),
        origin=Origin(xyz=(-0.012, door_center_y, 0.000), rpy=(-math.pi / 2, 0.0, 0.0)),
        material=dark,
        name="hinge_barrel",
    )
    door.visual(
        Box((0.018, door_width, 0.036)),
        origin=Origin(xyz=(-0.012, door_center_y, 0.018)),
        material=dark,
        name="bottom_rail",
    )
    door.visual(
        Box((0.018, door_width, 0.034)),
        origin=Origin(xyz=(-0.012, door_center_y, 0.250)),
        material=dark,
        name="top_rail",
    )
    for y, name in ((door_center_y - door_width / 2 + 0.018, "side_rail_0"), (door_center_y + door_width / 2 - 0.018, "side_rail_1")):
        door.visual(
            Box((0.018, 0.036, 0.230)),
            origin=Origin(xyz=(-0.012, y, 0.136)),
            material=dark,
            name=name,
        )
    door.visual(
        Box((0.005, 0.440, 0.205)),
        origin=Origin(xyz=(-0.015, door_center_y, 0.136)),
        material=glass,
        name="glass_window",
    )
    for y in (door_center_y - 0.155, door_center_y + 0.155):
        door.visual(
            Box((0.076, 0.024, 0.020)),
            origin=Origin(xyz=(-0.050, y, 0.210)),
            material=dark,
            name=f"handle_standoff_{y:.2f}",
        )
    door.visual(
        Cylinder(radius=0.012, length=0.365),
        origin=Origin(xyz=(-0.082, door_center_y, 0.210), rpy=(-math.pi / 2, 0.0, 0.0)),
        material=stainless,
        name="front_handle",
    )

    spit = model.part("spit")
    spit.visual(
        Cylinder(radius=0.005, length=0.540),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2, 0.0, 0.0)),
        material=warm_metal,
        name="spit_rod",
    )
    for side, y, direction in (("0", -0.130, 1.0), ("1", 0.130, -1.0)):
        spit.visual(
            Cylinder(radius=0.020, length=0.026),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2, 0.0, 0.0)),
            material=warm_metal,
            name=f"fork_collar_{side}",
        )
        for i, (x_off, z_off) in enumerate(((0.014, 0.014), (0.014, -0.014), (-0.014, 0.014), (-0.014, -0.014))):
            tine_center_y = y + direction * 0.054
            spit.visual(
                Cylinder(radius=0.0022, length=0.090),
                origin=Origin(xyz=(x_off, tine_center_y, z_off), rpy=(-math.pi / 2, 0.0, 0.0)),
                material=warm_metal,
                name=f"fork_tine_{side}_{i}",
            )

    for name, z in (("timer_knob", 0.225), ("temp_knob", 0.125)):
        knob = model.part(name)
        knob.visual(
            Cylinder(radius=0.030, length=0.006),
            origin=Origin(xyz=(-0.003, 0.0, 0.0), rpy=(0.0, -math.pi / 2, 0.0)),
            material=dark,
            name="knob_skirt",
        )
        knob.visual(
            Cylinder(radius=0.025, length=0.026),
            origin=Origin(xyz=(-0.013, 0.0, 0.0), rpy=(0.0, -math.pi / 2, 0.0)),
            material=dark,
            name="knob_cap",
        )
        knob.visual(
            Cylinder(radius=0.018, length=0.004),
            origin=Origin(xyz=(-0.028, 0.0, 0.0), rpy=(0.0, -math.pi / 2, 0.0)),
            material=stainless,
            name="knob_front_cap",
        )
        knob.visual(
            Box((0.003, 0.006, 0.033)),
            origin=Origin(xyz=(-0.031, 0.0, 0.008)),
            material=white,
            name="pointer_line",
        )
        model.articulation(
            f"housing_to_{name}",
            ArticulationType.REVOLUTE,
            parent=housing,
            child=knob,
            origin=Origin(xyz=(-0.016, 0.246, z)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.4, velocity=2.0, lower=-2.35, upper=2.35),
        )

    model.articulation(
        "housing_to_door",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(xyz=(-0.012, 0.0, 0.045)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.6, lower=0.0, upper=1.45),
    )
    model.articulation(
        "housing_to_spit",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=spit,
        origin=Origin(xyz=(0.190, 0.0, 0.178)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=6.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    door = object_model.get_part("door")
    housing = object_model.get_part("housing")
    spit = object_model.get_part("spit")
    door_joint = object_model.get_articulation("housing_to_door")
    spit_joint = object_model.get_articulation("housing_to_spit")

    ctx.check(
        "door uses bottom revolute hinge",
        door_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 3) for v in door_joint.axis) == (0.0, -1.0, 0.0)
        and door_joint.motion_limits is not None
        and door_joint.motion_limits.lower == 0.0
        and door_joint.motion_limits.upper >= 1.2,
        details=f"type={door_joint.articulation_type}, axis={door_joint.axis}, limits={door_joint.motion_limits}",
    )
    ctx.check(
        "spit is continuous horizontal rotation",
        spit_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 3) for v in spit_joint.axis) == (0.0, 1.0, 0.0),
        details=f"type={spit_joint.articulation_type}, axis={spit_joint.axis}",
    )

    with ctx.pose({door_joint: 0.0}):
        ctx.expect_overlap(
            door,
            housing,
            axes="yz",
            min_overlap=0.18,
            elem_a="glass_window",
            elem_b="rear_wall",
            name="closed door spans the front opening width and height",
        )
        closed_top = ctx.part_element_world_aabb(door, elem="top_rail")
    with ctx.pose({door_joint: 1.25}):
        open_top = ctx.part_element_world_aabb(door, elem="top_rail")

    if closed_top is not None and open_top is not None:
        closed_min, closed_max = closed_top
        open_min, open_max = open_top
        ctx.check(
            "door drops outward and downward",
            open_max[2] < closed_max[2] - 0.07 and open_min[0] < closed_min[0] - 0.12,
            details=f"closed={closed_top}, open={open_top}",
        )
    else:
        ctx.fail("door drops outward and downward", "top rail AABB unavailable")

    with ctx.pose({spit_joint: 0.0}):
        rest_tine = ctx.part_element_world_aabb(spit, elem="fork_tine_0_1")
    with ctx.pose({spit_joint: math.pi / 2}):
        quarter_tine = ctx.part_element_world_aabb(spit, elem="fork_tine_0_1")

    if rest_tine is not None and quarter_tine is not None:
        rest_min, rest_max = rest_tine
        quarter_min, quarter_max = quarter_tine
        rest_center_x = 0.5 * (rest_min[0] + rest_max[0])
        quarter_center_x = 0.5 * (quarter_min[0] + quarter_max[0])
        rest_center_z = 0.5 * (rest_min[2] + rest_max[2])
        quarter_center_z = 0.5 * (quarter_min[2] + quarter_max[2])
        radial_motion = math.hypot(quarter_center_x - rest_center_x, quarter_center_z - rest_center_z)
        ctx.check(
            "off-axis fork tine moves around the spit axis",
            radial_motion > 0.018,
            details=f"rest=({rest_center_x:.3f}, {rest_center_z:.3f}), quarter=({quarter_center_x:.3f}, {quarter_center_z:.3f})",
        )
    else:
        ctx.fail("off-axis fork tine moves around the spit axis", "fork tine AABB unavailable")

    return ctx.report()


object_model = build_object_model()
