from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_toaster_oven")

    brushed = model.material("brushed_stainless", rgba=(0.72, 0.70, 0.66, 1.0))
    dark = model.material("dark_cavity", rgba=(0.035, 0.034, 0.032, 1.0))
    black = model.material("black_trim", rgba=(0.015, 0.014, 0.013, 1.0))
    glass = model.material("smoky_glass", rgba=(0.22, 0.36, 0.43, 0.38))
    chrome = model.material("polished_chrome", rgba=(0.88, 0.86, 0.80, 1.0))
    rack_metal = model.material("rack_wire", rgba=(0.86, 0.84, 0.78, 1.0))
    white = model.material("white_marking", rgba=(0.95, 0.93, 0.86, 1.0))
    rubber = model.material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))

    body = model.part("body")

    # Countertop-scale outer envelope: 56 cm wide, 38 cm deep, 31 cm tall.
    # The front is at negative Y.  The left, broad aperture is intentionally
    # built from separate connected panels so the cavity remains visibly hollow.
    body.visual(
        Box((0.56, 0.38, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.2975)),
        material=brushed,
        name="top_shell",
    )
    body.visual(
        Box((0.56, 0.38, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=brushed,
        name="bottom_shell",
    )
    body.visual(
        Box((0.025, 0.38, 0.31)),
        origin=Origin(xyz=(-0.2675, 0.0, 0.155)),
        material=brushed,
        name="side_wall_0",
    )
    body.visual(
        Box((0.025, 0.38, 0.31)),
        origin=Origin(xyz=(0.2675, 0.0, 0.155)),
        material=brushed,
        name="side_wall_1",
    )
    body.visual(
        Box((0.56, 0.025, 0.31)),
        origin=Origin(xyz=(0.0, 0.1775, 0.155)),
        material=dark,
        name="rear_wall",
    )
    body.visual(
        Box((0.14, 0.025, 0.25)),
        origin=Origin(xyz=(0.20, -0.1775, 0.160)),
        material=brushed,
        name="control_panel",
    )
    body.visual(
        Box((0.025, 0.38, 0.25)),
        origin=Origin(xyz=(0.1325, 0.0, 0.160)),
        material=dark,
        name="cavity_divider",
    )
    body.visual(
        Box((0.39, 0.025, 0.035)),
        origin=Origin(xyz=(-0.065, -0.1775, 0.2675)),
        material=black,
        name="front_top_lip",
    )
    body.visual(
        Box((0.39, 0.025, 0.030)),
        origin=Origin(xyz=(-0.065, -0.1775, 0.050)),
        material=black,
        name="front_hinge_rail",
    )
    body.visual(
        Box((0.025, 0.025, 0.23)),
        origin=Origin(xyz=(-0.2425, -0.1775, 0.160)),
        material=black,
        name="front_jamb_0",
    )
    body.visual(
        Box((0.025, 0.025, 0.23)),
        origin=Origin(xyz=(0.1225, -0.1775, 0.160)),
        material=black,
        name="front_jamb_1",
    )

    # Straight side runners inside the oven.  Their upper faces carry the rack
    # side rods, proving that the prismatic rack is on rails rather than floating.
    for x, name in [(-0.235, "runner_0"), (0.105, "runner_1")]:
        body.visual(
            Box((0.018, 0.305, 0.009)),
            origin=Origin(xyz=(x, 0.020, 0.1355)),
            material=chrome,
            name=name,
        )
        body.visual(
            Box((0.010, 0.055, 0.040)),
            origin=Origin(xyz=(x + (0.011 if x < 0 else -0.011), 0.145, 0.153)),
            material=dark,
            name=f"{name}_side_flange",
        )

    # Two warm heating rods read through the glass and above/below the rack.
    for z, name in [(0.085, "lower_heater"), (0.245, "upper_heater")]:
        body.visual(
            Cylinder(radius=0.006, length=0.370),
            origin=Origin(xyz=(-0.065, 0.040, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=Material("copper_glow", rgba=(0.95, 0.34, 0.10, 1.0)),
            name=name,
        )

    # Small feet are attached to the bottom shell at the four corners.
    for i, (x, y) in enumerate(
        [(-0.22, -0.13), (0.22, -0.13), (-0.22, 0.13), (0.22, 0.13)]
    ):
        body.visual(
            Cylinder(radius=0.026, length=0.020),
            origin=Origin(xyz=(x, y, -0.010)),
            material=rubber,
            name=f"foot_{i}",
        )

    door = model.part("door")
    # Door part frame is the lower hinge line at the center of the broad front
    # opening.  Closed geometry extends upward in local +Z and forward in -Y.
    door.visual(
        Box((0.390, 0.024, 0.030)),
        origin=Origin(xyz=(0.0, 0.003, 0.015)),
        material=black,
        name="lower_rail",
    )
    door.visual(
        Box((0.390, 0.024, 0.032)),
        origin=Origin(xyz=(0.0, 0.003, 0.191)),
        material=black,
        name="upper_rail",
    )
    for x, name in [(-0.178, "side_rail_0"), (0.178, "side_rail_1")]:
        door.visual(
            Box((0.034, 0.024, 0.190)),
            origin=Origin(xyz=(x, 0.003, 0.102)),
            material=black,
            name=name,
        )
    door.visual(
        Box((0.300, 0.006, 0.125)),
        origin=Origin(xyz=(0.0, -0.0005, 0.105)),
        material=glass,
        name="glass_pane",
    )
    door.visual(
        Cylinder(radius=0.011, length=0.270),
        origin=Origin(xyz=(0.0, -0.040, 0.166), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="handle_bar",
    )
    for x, name in [(-0.115, "handle_mount_0"), (0.115, "handle_mount_1")]:
        door.visual(
            Box((0.027, 0.045, 0.024)),
            origin=Origin(xyz=(x, -0.019, 0.166)),
            material=chrome,
            name=name,
        )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(-0.065, -0.205, 0.065)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=7.0, velocity=1.5, lower=0.0, upper=1.85),
    )

    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.055,
            0.026,
            body_style="skirted",
            top_diameter=0.043,
            edge_radius=0.0012,
            skirt=KnobSkirt(0.064, 0.006, flare=0.07, chamfer=0.001),
            grip=KnobGrip(style="fluted", count=18, depth=0.0010),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0006),
            center=False,
        ),
        "toaster_oven_dial",
    )

    for z, part_name, joint_name in [
        (0.202, "upper_dial", "body_to_upper_dial"),
        (0.118, "lower_dial", "body_to_lower_dial"),
    ]:
        dial = model.part(part_name)
        dial.visual(
            Cylinder(radius=0.010, length=0.012),
            origin=Origin(xyz=(0.0, -0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=chrome,
            name="short_shaft",
        )
        dial.visual(
            knob_mesh,
            origin=Origin(xyz=(0.0, -0.012, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=black,
            name="dial_cap",
        )
        # A raised white tick on the dial face makes continuous rotation visible.
        dial.visual(
            Box((0.006, 0.0025, 0.030)),
            origin=Origin(xyz=(0.0, -0.0395, 0.010)),
            material=white,
            name="pointer_tick",
        )
        model.articulation(
            joint_name,
            ArticulationType.CONTINUOUS,
            parent=body,
            child=dial,
            origin=Origin(xyz=(0.200, -0.190, z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=0.8, velocity=6.0),
        )

    rack = model.part("broil_rack")
    # The rack frame is centered on the two side runners at q=0 and slides
    # forward along -Y while retaining insertion in the cavity.
    for x, name in [(-0.170, "side_wire_0"), (0.170, "side_wire_1")]:
        rack.visual(
            Cylinder(radius=0.003, length=0.290),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rack_metal,
            name=name,
        )
    for i, y in enumerate([-0.125, -0.080, -0.035, 0.010, 0.055, 0.100, 0.140]):
        rack.visual(
            Cylinder(radius=0.0022, length=0.340),
            origin=Origin(xyz=(0.0, y, 0.0012), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rack_metal,
            name=f"cross_wire_{i}",
        )
    rack.visual(
        Cylinder(radius=0.004, length=0.350),
        origin=Origin(xyz=(0.0, -0.145, 0.002), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rack_metal,
        name="front_wire",
    )

    model.articulation(
        "body_to_broil_rack",
        ArticulationType.PRISMATIC,
        parent=body,
        child=rack,
        origin=Origin(xyz=(-0.065, 0.020, 0.144)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.30, lower=0.0, upper=0.155),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    door = object_model.get_part("door")
    rack = object_model.get_part("broil_rack")
    door_hinge = object_model.get_articulation("body_to_door")
    rack_slide = object_model.get_articulation("body_to_broil_rack")

    with ctx.pose({door_hinge: 0.0, rack_slide: 0.0}):
        ctx.expect_gap(
            body,
            door,
            axis="y",
            min_gap=-0.001,
            max_gap=0.006,
            name="closed door is seated just in front of the opening",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="xz",
            min_overlap=0.16,
            name="broad door covers the lower front aperture",
        )
        ctx.expect_contact(
            rack,
            body,
            elem_a="side_wire_0",
            elem_b="runner_0",
            contact_tol=0.002,
            name="rack rides on first straight side runner",
        )
        ctx.expect_contact(
            rack,
            body,
            elem_a="side_wire_1",
            elem_b="runner_1",
            contact_tol=0.002,
            name="rack rides on second straight side runner",
        )
        ctx.expect_overlap(
            rack,
            body,
            axes="y",
            elem_a="side_wire_0",
            elem_b="runner_0",
            min_overlap=0.20,
            name="rack is substantially inserted on the runner at rest",
        )

    closed_aabb = ctx.part_world_aabb(door)
    rack_rest = ctx.part_world_position(rack)
    with ctx.pose({door_hinge: 1.25, rack_slide: 0.155}):
        opened_aabb = ctx.part_world_aabb(door)
        rack_extended = ctx.part_world_position(rack)
        ctx.expect_overlap(
            rack,
            body,
            axes="y",
            elem_a="side_wire_0",
            elem_b="runner_0",
            min_overlap=0.08,
            name="extended rack remains retained by the side runner",
        )

    ctx.check(
        "door rotates downward from the lower hinge",
        closed_aabb is not None
        and opened_aabb is not None
        and opened_aabb[1][2] < closed_aabb[1][2] - 0.03,
        details=f"closed_aabb={closed_aabb}, opened_aabb={opened_aabb}",
    )
    ctx.check(
        "rack slides outward toward the user",
        rack_rest is not None and rack_extended is not None and rack_extended[1] < rack_rest[1] - 0.10,
        details=f"rest={rack_rest}, extended={rack_extended}",
    )

    return ctx.report()


object_model = build_object_model()
