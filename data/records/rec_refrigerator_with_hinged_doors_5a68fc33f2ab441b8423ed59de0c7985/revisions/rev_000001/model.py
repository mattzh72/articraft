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
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="glazed_beverage_refrigerator")

    powder_black = model.material("powder_black", rgba=(0.015, 0.016, 0.018, 1.0))
    satin_black = model.material("satin_black", rgba=(0.035, 0.037, 0.040, 1.0))
    dark_gasket = model.material("dark_gasket", rgba=(0.003, 0.003, 0.004, 1.0))
    cool_glass = model.material("cool_glass", rgba=(0.45, 0.78, 0.95, 0.36))
    shelf_glass = model.material("shelf_glass", rgba=(0.68, 0.88, 1.0, 0.28))
    brushed_metal = model.material("brushed_metal", rgba=(0.58, 0.61, 0.63, 1.0))
    white_plastic = model.material("white_plastic", rgba=(0.88, 0.90, 0.88, 1.0))
    indicator_white = model.material("indicator_white", rgba=(1.0, 0.98, 0.82, 1.0))

    cabinet = model.part("cabinet")

    # Tall, narrow refrigerator carcass: separate walls leave a real hollow
    # refrigerated cavity visible through the glazed door.
    cabinet.visual(
        Box((0.036, 0.500, 1.220)),
        origin=Origin(xyz=(-0.202, 0.015, 0.630)),
        material=powder_black,
        name="side_wall_0",
    )
    cabinet.visual(
        Box((0.036, 0.500, 1.220)),
        origin=Origin(xyz=(0.202, 0.015, 0.630)),
        material=powder_black,
        name="side_wall_1",
    )
    cabinet.visual(
        Box((0.440, 0.030, 1.200)),
        origin=Origin(xyz=(0.000, 0.250, 0.620)),
        material=satin_black,
        name="rear_wall",
    )
    cabinet.visual(
        Box((0.440, 0.500, 0.040)),
        origin=Origin(xyz=(0.000, 0.015, 1.220)),
        material=powder_black,
        name="top_panel",
    )
    cabinet.visual(
        Box((0.440, 0.500, 0.040)),
        origin=Origin(xyz=(0.000, 0.015, 0.020)),
        material=powder_black,
        name="bottom_panel",
    )
    cabinet.visual(
        Box((0.440, 0.030, 0.100)),
        origin=Origin(xyz=(0.000, -0.220, 1.150)),
        material=powder_black,
        name="front_top_frame",
    )
    cabinet.visual(
        Box((0.440, 0.030, 0.115)),
        origin=Origin(xyz=(0.000, -0.220, 0.0775)),
        material=powder_black,
        name="front_kick_frame",
    )

    # Subtle lower ventilation slots in the kick panel.
    for i, x in enumerate((-0.150, -0.100, -0.050, 0.000, 0.050, 0.100, 0.150)):
        cabinet.visual(
            Box((0.026, 0.006, 0.054)),
            origin=Origin(xyz=(x, -0.237, 0.078)),
            material=dark_gasket,
            name=f"vent_slot_{i}",
        )

    # Transparent shelves are supported by small metal rails bonded to the side
    # walls, so the shelves do not read as floating plates.
    for level, z in enumerate((0.430, 0.735)):
        cabinet.visual(
            Box((0.350, 0.390, 0.008)),
            origin=Origin(xyz=(0.000, 0.035, z)),
            material=shelf_glass,
            name=f"glass_shelf_{level}",
        )
        cabinet.visual(
            Box((0.016, 0.390, 0.018)),
            origin=Origin(xyz=(-0.176, 0.035, z - 0.013)),
            material=brushed_metal,
            name=f"shelf_rail_{level}_0",
        )
        cabinet.visual(
            Box((0.016, 0.390, 0.018)),
            origin=Origin(xyz=(0.176, 0.035, z - 0.013)),
            material=brushed_metal,
            name=f"shelf_rail_{level}_1",
        )

    # The small inner control pod is on the top frame, behind the door glass.
    cabinet.visual(
        Box((0.180, 0.120, 0.070)),
        origin=Origin(xyz=(0.105, -0.110, 1.165)),
        material=white_plastic,
        name="control_pod",
    )

    # Cabinet-side hinge knuckles and leaves.  Door-side knuckles occupy the
    # alternating gaps, avoiding full-time part interpenetration.
    hinge_x = -0.238
    hinge_y = -0.269
    for i, (z_center, z_len) in enumerate(((0.135, 0.120), (0.590, 0.160), (1.105, 0.140))):
        cabinet.visual(
            Box((0.034, 0.040, z_len)),
            origin=Origin(xyz=(hinge_x + 0.017, hinge_y + 0.017, z_center)),
            material=brushed_metal,
            name=f"hinge_leaf_{i}",
        )
        cabinet.visual(
            Cylinder(radius=0.011, length=z_len),
            origin=Origin(xyz=(hinge_x, hinge_y, z_center)),
            material=brushed_metal,
            name=f"hinge_knuckle_{i}",
        )

    for i, (x, y) in enumerate(((-0.170, 0.165), (0.170, 0.165), (-0.170, -0.145), (0.170, -0.145))):
        cabinet.visual(
            Cylinder(radius=0.020, length=0.030),
            origin=Origin(xyz=(x, y, -0.015)),
            material=dark_gasket,
            name=f"leveling_foot_{i}",
        )

    door = model.part("glass_door")
    # Door frame is authored in its hinge frame: local +X spans across the door,
    # local +Z rises, and local Y is door thickness.  At q=0 it sits just in
    # front of the cabinet face with a small seal gap.
    door.visual(
        Box((0.405, 0.040, 0.080)),
        origin=Origin(xyz=(0.2375, 0.000, 0.040)),
        material=satin_black,
        name="bottom_rail",
    )
    door.visual(
        Box((0.405, 0.040, 0.080)),
        origin=Origin(xyz=(0.2375, 0.000, 1.140)),
        material=satin_black,
        name="top_rail",
    )
    door.visual(
        Box((0.052, 0.040, 1.180)),
        origin=Origin(xyz=(0.063, 0.000, 0.590)),
        material=satin_black,
        name="hinge_stile",
    )
    door.visual(
        Box((0.055, 0.040, 1.180)),
        origin=Origin(xyz=(0.4125, 0.000, 0.590)),
        material=satin_black,
        name="latch_stile",
    )
    door.visual(
        Box((0.318, 0.010, 1.020)),
        origin=Origin(xyz=(0.237, -0.004, 0.590)),
        material=cool_glass,
        name="glass_pane",
    )
    door.visual(
        Box((0.336, 0.012, 0.020)),
        origin=Origin(xyz=(0.237, -0.022, 0.095)),
        material=dark_gasket,
        name="lower_gasket",
    )
    door.visual(
        Box((0.336, 0.012, 0.020)),
        origin=Origin(xyz=(0.237, -0.022, 1.085)),
        material=dark_gasket,
        name="upper_gasket",
    )
    door.visual(
        Box((0.020, 0.012, 1.000)),
        origin=Origin(xyz=(0.073, -0.022, 0.590)),
        material=dark_gasket,
        name="hinge_gasket",
    )
    door.visual(
        Box((0.020, 0.012, 1.000)),
        origin=Origin(xyz=(0.401, -0.022, 0.590)),
        material=dark_gasket,
        name="latch_gasket",
    )

    for i, (z_center, z_len) in enumerate(((0.3125, 0.315), (0.8125, 0.365))):
        door.visual(
            Box((0.050, 0.018, z_len)),
            origin=Origin(xyz=(0.025, 0.000, z_center)),
            material=brushed_metal,
            name=f"door_hinge_leaf_{i}",
        )
        door.visual(
            Cylinder(radius=0.011, length=z_len),
            origin=Origin(xyz=(0.000, 0.000, z_center)),
            material=brushed_metal,
            name=f"door_hinge_knuckle_{i}",
        )

    door.visual(
        Cylinder(radius=0.012, length=0.700),
        origin=Origin(xyz=(0.405, -0.075, 0.625)),
        material=brushed_metal,
        name="pull_handle",
    )
    door.visual(
        Box((0.026, 0.070, 0.030)),
        origin=Origin(xyz=(0.405, -0.035, 0.340)),
        material=brushed_metal,
        name="handle_mount_0",
    )
    door.visual(
        Box((0.026, 0.070, 0.030)),
        origin=Origin(xyz=(0.405, -0.035, 0.910)),
        material=brushed_metal,
        name="handle_mount_1",
    )

    knob = model.part("thermostat_knob")
    knob_body = KnobGeometry(
        0.058,
        0.026,
        body_style="skirted",
        base_diameter=0.064,
        top_diameter=0.046,
        edge_radius=0.0015,
        grip=KnobGrip(style="fluted", count=18, depth=0.0012),
        indicator=KnobIndicator(style="line", mode="engraved", angle_deg=0.0, depth=0.0008),
        center=False,
    )
    knob.visual(
        Cylinder(radius=0.007, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=dark_gasket,
        name="shaft",
    )
    knob.visual(
        mesh_from_geometry(knob_body, "thermostat_knob_body"),
        material=white_plastic,
        name="dial_cap",
    )
    knob.visual(
        Box((0.006, 0.025, 0.0015)),
        origin=Origin(xyz=(0.0, 0.010, 0.026)),
        material=indicator_white,
        name="pointer_mark",
    )

    door_joint = model.articulation(
        "cabinet_to_glass_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, 0.040)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.4, lower=0.0, upper=1.75),
    )
    door_joint.meta["description"] = "Vertical side hinge; positive motion swings the door outward from the cabinet."

    knob_joint = model.articulation(
        "cabinet_to_thermostat_knob",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=knob,
        origin=Origin(xyz=(0.105, -0.170, 1.165), rpy=(math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.35, velocity=3.0, lower=-2.35, upper=2.35),
    )
    knob_joint.meta["description"] = "Thermostat dial rotates about its short shaft on the inner top frame."

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("glass_door")
    knob = object_model.get_part("thermostat_knob")
    door_joint = object_model.get_articulation("cabinet_to_glass_door")
    knob_joint = object_model.get_articulation("cabinet_to_thermostat_knob")

    with ctx.pose({door_joint: 0.0, knob_joint: 0.0}):
        ctx.expect_gap(
            cabinet,
            door,
            axis="y",
            positive_elem="top_panel",
            negative_elem="top_rail",
            min_gap=0.006,
            max_gap=0.025,
            name="closed glazed door sits just proud of cabinet face",
        )
        ctx.expect_overlap(
            door,
            cabinet,
            axes="xz",
            elem_a="glass_pane",
            elem_b="rear_wall",
            min_overlap=0.30,
            name="glass pane covers the refrigerated cavity",
        )
        ctx.expect_contact(
            knob,
            cabinet,
            elem_a="shaft",
            elem_b="control_pod",
            contact_tol=1e-5,
            name="thermostat shaft is seated in the top control pod",
        )

    closed_latch = ctx.part_element_world_aabb(door, elem="latch_stile")
    with ctx.pose({door_joint: 1.25}):
        opened_latch = ctx.part_element_world_aabb(door, elem="latch_stile")
    ctx.check(
        "positive door rotation swings free edge outward",
        closed_latch is not None
        and opened_latch is not None
        and opened_latch[0][1] < closed_latch[0][1] - 0.25,
        details=f"closed_latch={closed_latch}, opened_latch={opened_latch}",
    )

    ctx.check(
        "thermostat knob has finite rotary travel",
        knob_joint.motion_limits is not None
        and knob_joint.motion_limits.lower is not None
        and knob_joint.motion_limits.upper is not None
        and knob_joint.motion_limits.lower < 0.0 < knob_joint.motion_limits.upper
        and (knob_joint.motion_limits.upper - knob_joint.motion_limits.lower) > 4.0,
        details=f"limits={knob_joint.motion_limits}",
    )

    return ctx.report()


object_model = build_object_model()
