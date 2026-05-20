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
    model = ArticulatedObject(name="stacked_oven_range_cooker")

    stainless = model.material("brushed_stainless", color=(0.62, 0.62, 0.58, 1.0))
    dark_enamel = model.material("black_enamel", color=(0.02, 0.022, 0.02, 1.0))
    cast_iron = model.material("matte_cast_iron", color=(0.006, 0.006, 0.005, 1.0))
    glass = model.material("smoky_oven_glass", color=(0.03, 0.045, 0.055, 0.72))
    handle_metal = model.material("polished_handle", color=(0.78, 0.76, 0.70, 1.0))
    pointer = model.material("white_indicator", color=(0.92, 0.90, 0.82, 1.0))

    body = model.part("body")

    # Full-size domestic range cooker proportions: about 0.95 m wide, 0.70 m
    # deep, and just under 1 m high including the top grates.
    body.visual(Box((0.95, 0.70, 0.08)), origin=Origin(xyz=(0.0, 0.0, 0.04)), material=stainless, name="plinth")
    body.visual(Box((0.035, 0.66, 0.785)), origin=Origin(xyz=(-0.4575, 0.0, 0.4725)), material=stainless, name="side_stile_0")
    body.visual(Box((0.035, 0.66, 0.785)), origin=Origin(xyz=(0.4575, 0.0, 0.4725)), material=stainless, name="side_stile_1")
    body.visual(Box((0.90, 0.035, 0.785)), origin=Origin(xyz=(0.0, 0.3125, 0.4725)), material=stainless, name="rear_panel")
    body.visual(Box((0.95, 0.70, 0.06)), origin=Origin(xyz=(0.0, 0.0, 0.89)), material=dark_enamel, name="cooktop")

    # Front frame and control fascia are separate visible stamped-metal members,
    # deliberately leaving the two oven door fields clear.
    front_y = -0.3475
    body.visual(Box((0.04, 0.035, 0.66)), origin=Origin(xyz=(-0.43, front_y, 0.405)), material=stainless, name="front_stile_0")
    body.visual(Box((0.04, 0.035, 0.66)), origin=Origin(xyz=(0.43, front_y, 0.405)), material=stainless, name="front_stile_1")
    body.visual(Box((0.88, 0.035, 0.035)), origin=Origin(xyz=(0.0, front_y, 0.0975)), material=stainless, name="lower_sill")
    body.visual(Box((0.88, 0.035, 0.030)), origin=Origin(xyz=(0.0, front_y, 0.405)), material=stainless, name="center_rail")
    body.visual(Box((0.88, 0.035, 0.030)), origin=Origin(xyz=(0.0, front_y, 0.725)), material=stainless, name="upper_rail")
    body.visual(Box((0.88, 0.035, 0.13)), origin=Origin(xyz=(0.0, front_y, 0.795)), material=stainless, name="control_fascia")

    # Dark recessed oven voids visible around the glass in the closed doors.
    body.visual(Box((0.84, 0.012, 0.300)), origin=Origin(xyz=(0.0, -0.326, 0.250)), material=dark_enamel, name="lower_cavity")
    body.visual(Box((0.84, 0.012, 0.310)), origin=Origin(xyz=(0.0, -0.326, 0.575)), material=dark_enamel, name="upper_cavity")

    burner_positions = [
        (-0.31, -0.16),
        (0.0, -0.16),
        (0.31, -0.16),
        (-0.31, 0.15),
        (0.0, 0.15),
        (0.31, 0.15),
    ]
    for index, (x_pos, y_pos) in enumerate(burner_positions):
        body.visual(
            Cylinder(radius=0.078, length=0.006),
            origin=Origin(xyz=(x_pos, y_pos, 0.923)),
            material=handle_metal,
            name=f"burner_ring_{index}",
        )
        body.visual(
            Cylinder(radius=0.052, length=0.010),
            origin=Origin(xyz=(x_pos, y_pos, 0.928)),
            material=cast_iron,
            name=f"burner_cap_{index}",
        )

    # Six individual fixed cast-iron surface grates, each with four feet and a
    # connected ring/cross layout around its corresponding burner.
    grate_size_x = 0.235
    grate_size_y = 0.215
    bar = 0.020
    for index, (x_pos, y_pos) in enumerate(burner_positions):
        grate = model.part(f"grate_{index}")
        for foot_index, (fx, fy) in enumerate(
            (
                (-grate_size_x / 2 + 0.026, -grate_size_y / 2 + 0.026),
                (grate_size_x / 2 - 0.026, -grate_size_y / 2 + 0.026),
                (-grate_size_x / 2 + 0.026, grate_size_y / 2 - 0.026),
                (grate_size_x / 2 - 0.026, grate_size_y / 2 - 0.026),
            )
        ):
            grate.visual(
                Box((0.034, 0.034, 0.012)),
                origin=Origin(xyz=(fx, fy, 0.006)),
                material=cast_iron,
                name=f"foot_{foot_index}",
            )
        grate.visual(Box((grate_size_x, bar, 0.016)), origin=Origin(xyz=(0.0, -grate_size_y / 2, 0.020)), material=cast_iron, name="front_bar")
        grate.visual(Box((grate_size_x, bar, 0.016)), origin=Origin(xyz=(0.0, grate_size_y / 2, 0.020)), material=cast_iron, name="rear_bar")
        grate.visual(Box((bar, grate_size_y, 0.016)), origin=Origin(xyz=(-grate_size_x / 2, 0.0, 0.020)), material=cast_iron, name="side_bar_0")
        grate.visual(Box((bar, grate_size_y, 0.016)), origin=Origin(xyz=(grate_size_x / 2, 0.0, 0.020)), material=cast_iron, name="side_bar_1")
        grate.visual(Box((bar, grate_size_y, 0.016)), origin=Origin(xyz=(0.0, 0.0, 0.020)), material=cast_iron, name="cross_bar_y")
        grate.visual(Box((grate_size_x, bar, 0.016)), origin=Origin(xyz=(0.0, 0.0, 0.020)), material=cast_iron, name="cross_bar_x")
        model.articulation(
            f"body_to_grate_{index}",
            ArticulationType.FIXED,
            parent=body,
            child=grate,
            origin=Origin(xyz=(x_pos, y_pos, 0.920)),
        )

    def add_oven_door(name: str, bottom_z: float, height: float, glass_height: float) -> None:
        door = model.part(name)
        door.visual(
            Box((0.80, 0.040, height)),
            origin=Origin(xyz=(0.0, -0.020, height / 2)),
            material=stainless,
            name="door_panel",
        )
        door.visual(
            Box((0.58, 0.006, glass_height)),
            origin=Origin(xyz=(0.0, -0.0425, height * 0.53)),
            material=glass,
            name="glass_window",
        )
        # A robust horizontal pull handle mounted to the door skin by two posts.
        handle_z = height - 0.058
        door.visual(
            Cylinder(radius=0.012, length=0.62),
            origin=Origin(xyz=(0.0, -0.078, handle_z), rpy=(0.0, math.pi / 2, 0.0)),
            material=handle_metal,
            name="handle_bar",
        )
        for post_index, x_pos in enumerate((-0.27, 0.27)):
            door.visual(
                Cylinder(radius=0.008, length=0.048),
                origin=Origin(xyz=(x_pos, -0.061, handle_z), rpy=(math.pi / 2, 0.0, 0.0)),
                material=handle_metal,
                name=f"handle_post_{post_index}",
            )
        model.articulation(
            f"body_to_{name}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=door,
            origin=Origin(xyz=(0.0, -0.365, bottom_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=45.0, velocity=1.0, lower=0.0, upper=1.45),
        )

    add_oven_door("lower_door", bottom_z=0.100, height=0.300, glass_height=0.150)
    add_oven_door("upper_door", bottom_z=0.430, height=0.290, glass_height=0.140)

    # Appliance controls: six burner knobs and two oven thermostat knobs, all
    # articulated as finite rotary controls on the fascia.
    knob_positions = [-0.36, -0.255, -0.15, -0.045, 0.085, 0.195, 0.305, 0.405]
    for index, x_pos in enumerate(knob_positions):
        knob_name = f"knob_{index}"
        knob = model.part(knob_name)
        radius = 0.020 if index < 6 else 0.022
        knob.visual(
            Cylinder(radius=radius, length=0.034),
            origin=Origin(xyz=(0.0, -0.017, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
            material=handle_metal,
            name="knob_cap",
        )
        knob.visual(
            Box((0.004, 0.005, radius * 1.35)),
            origin=Origin(xyz=(0.0, -0.036, radius * 0.16)),
            material=pointer,
            name="indicator_line",
        )
        model.articulation(
            f"body_to_{knob_name}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=knob,
            origin=Origin(xyz=(x_pos, -0.365, 0.795)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=3.0, lower=-2.35, upper=2.35),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    def _coord(vec, index: int, attr: str) -> float:
        if hasattr(vec, attr):
            return float(getattr(vec, attr))
        return float(vec[index])

    def _aabb_tuple(aabb):
        if aabb is None:
            return None
        return (
            (_coord(aabb[0], 0, "x"), _coord(aabb[0], 1, "y"), _coord(aabb[0], 2, "z")),
            (_coord(aabb[1], 0, "x"), _coord(aabb[1], 1, "y"), _coord(aabb[1], 2, "z")),
        )

    body = object_model.get_part("body")
    grate_names = [part.name for part in object_model.parts if part.name.startswith("grate_")]
    ctx.check("six surface burner grates", len(grate_names) == 6, details=f"grates={grate_names}")

    for index in range(6):
        grate = object_model.get_part(f"grate_{index}")
        ctx.expect_contact(
            grate,
            body,
            elem_a="foot_0",
            elem_b="cooktop",
            contact_tol=0.002,
            name=f"grate_{index} foot rests on cooktop",
        )
        ctx.expect_overlap(
            grate,
            body,
            axes="xy",
            elem_a="cross_bar_x",
            elem_b=f"burner_cap_{index}",
            min_overlap=0.015,
            name=f"grate_{index} centered over burner",
        )

    for door_name in ("lower_door", "upper_door"):
        door = object_model.get_part(door_name)
        hinge = object_model.get_articulation(f"body_to_{door_name}")
        ctx.check(
            f"{door_name} has bottom-edge hinge axis",
            tuple(hinge.axis) == (1.0, 0.0, 0.0) and hinge.motion_limits.lower == 0.0 and hinge.motion_limits.upper > 1.2,
            details=f"axis={hinge.axis}, limits={hinge.motion_limits}",
        )
        closed = _aabb_tuple(ctx.part_world_aabb(door))
        with ctx.pose({hinge: 1.20}):
            opened = _aabb_tuple(ctx.part_world_aabb(door))
        ok = (
            closed is not None
            and opened is not None
            and opened[1][2] < closed[1][2] - 0.07
            and opened[0][1] < closed[0][1] - 0.16
        )
        ctx.check(
            f"{door_name} folds downward and outward",
            ok,
            details=f"closed={closed}, opened={opened}",
        )

    knob_joints = [joint.name for joint in object_model.articulations if joint.name.startswith("body_to_knob_")]
    ctx.check("front rotary controls are articulated", len(knob_joints) == 8, details=f"knob_joints={knob_joints}")

    return ctx.report()


object_model = build_object_model()
