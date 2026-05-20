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
    model = ArticulatedObject(name="compact_under_counter_refrigerator")

    enamel = model.material("warm_white_enamel", color=(0.92, 0.90, 0.84, 1.0))
    liner = model.material("molded_white_liner", color=(0.96, 0.96, 0.92, 1.0))
    gasket = model.material("dark_rubber_gasket", color=(0.03, 0.035, 0.035, 1.0))
    hinge_metal = model.material("brushed_hinge_metal", color=(0.62, 0.63, 0.60, 1.0))
    handle_mat = model.material("satin_plastic_handle", color=(0.86, 0.86, 0.82, 1.0))
    freezer_mat = model.material("pale_freezer_plastic", color=(0.82, 0.91, 0.96, 1.0))
    glass = model.material("smoked_glass_shelf", color=(0.55, 0.75, 0.78, 0.42))

    # Real under-counter proportions: roughly 560 mm wide, 550 mm deep, and
    # 820 mm tall.  The cabinet is an open-front shell so the refrigerator reads
    # as a hollow appliance rather than a solid block.
    width = 0.56
    depth = 0.55
    height = 0.82
    wall = 0.045
    front_y = -depth / 2.0

    cabinet = model.part("cabinet")

    # Structural outer shell and front return frame.
    cabinet.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(-width / 2.0 + wall / 2.0, 0.0, height / 2.0)),
        material=enamel,
        name="side_wall_0",
    )
    cabinet.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(width / 2.0 - wall / 2.0, 0.0, height / 2.0)),
        material=enamel,
        name="side_wall_1",
    )
    cabinet.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, height - wall / 2.0)),
        material=enamel,
        name="top_wall",
    )
    cabinet.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, wall / 2.0)),
        material=enamel,
        name="bottom_wall",
    )
    cabinet.visual(
        Box((width, wall, height)),
        origin=Origin(xyz=(0.0, depth / 2.0 - wall / 2.0, height / 2.0)),
        material=enamel,
        name="back_wall",
    )
    cabinet.visual(
        Box((width, 0.030, wall)),
        origin=Origin(xyz=(0.0, front_y - 0.010, height - wall / 2.0)),
        material=enamel,
        name="front_frame_top",
    )
    cabinet.visual(
        Box((width, 0.030, wall)),
        origin=Origin(xyz=(0.0, front_y - 0.010, wall / 2.0)),
        material=enamel,
        name="front_frame_bottom",
    )
    cabinet.visual(
        Box((wall, 0.030, height)),
        origin=Origin(xyz=(-width / 2.0 + wall / 2.0, front_y - 0.010, height / 2.0)),
        material=enamel,
        name="front_jamb_0",
    )
    cabinet.visual(
        Box((wall, 0.030, height)),
        origin=Origin(xyz=(width / 2.0 - wall / 2.0, front_y - 0.010, height / 2.0)),
        material=enamel,
        name="front_jamb_1",
    )

    # Interior liner panels and a supported glass shelf.
    cabinet.visual(
        Box((0.470, 0.012, 0.700)),
        origin=Origin(xyz=(0.0, depth / 2.0 - wall - 0.006, 0.405)),
        material=liner,
        name="inner_back_liner",
    )
    cabinet.visual(
        Box((0.470, 0.360, 0.010)),
        origin=Origin(xyz=(0.0, -0.015, 0.375)),
        material=glass,
        name="glass_shelf",
    )
    cabinet.visual(
        Box((0.030, 0.360, 0.020)),
        origin=Origin(xyz=(-0.235, -0.015, 0.368)),
        material=liner,
        name="shelf_rail_0",
    )
    cabinet.visual(
        Box((0.030, 0.360, 0.020)),
        origin=Origin(xyz=(0.235, -0.015, 0.368)),
        material=liner,
        name="shelf_rail_1",
    )

    # Fixed freezer compartment at the top of the cavity.  Its front is open and
    # the small articulated flap below covers that opening.
    cabinet.visual(
        Box((0.420, 0.470, 0.026)),
        origin=Origin(xyz=(0.0, 0.000, 0.763)),
        material=freezer_mat,
        name="freezer_roof",
    )
    cabinet.visual(
        Box((0.420, 0.470, 0.022)),
        origin=Origin(xyz=(0.0, 0.000, 0.596)),
        material=freezer_mat,
        name="freezer_floor",
    )
    cabinet.visual(
        Box((0.026, 0.470, 0.178)),
        origin=Origin(xyz=(-0.210, 0.000, 0.675)),
        material=freezer_mat,
        name="freezer_side_0",
    )
    cabinet.visual(
        Box((0.026, 0.470, 0.178)),
        origin=Origin(xyz=(0.210, 0.000, 0.675)),
        material=freezer_mat,
        name="freezer_side_1",
    )
    cabinet.visual(
        Box((0.420, 0.030, 0.178)),
        origin=Origin(xyz=(0.0, 0.220, 0.675)),
        material=freezer_mat,
        name="freezer_back",
    )

    # Cabinet-side hinge knuckles and hinge leaves for the main door.
    hinge_x = -width / 2.0 - 0.025
    hinge_y = front_y - 0.030
    for i, (z_center, z_len) in enumerate(((0.268, 0.075), (0.555, 0.075))):
        cabinet.visual(
            Cylinder(radius=0.012, length=z_len),
            origin=Origin(xyz=(hinge_x, hinge_y, z_center)),
            material=hinge_metal,
            name=f"cabinet_hinge_barrel_{i}",
        )
        cabinet.visual(
            Box((0.032, 0.014, z_len)),
            origin=Origin(xyz=(hinge_x + 0.017, hinge_y + 0.016, z_center)),
            material=hinge_metal,
            name=f"cabinet_hinge_leaf_{i}",
        )

    # Door link frame sits on the vertical hinge axis. At q=0 the panel spans
    # across the front. Positive rotation about -Z swings the free edge outward.
    door = model.part("door")
    door.visual(
        Box((0.550, 0.064, 0.780)),
        origin=Origin(xyz=(0.305, -0.048, 0.390)),
        material=enamel,
        name="door_panel",
    )
    door.visual(
        Box((0.455, 0.016, 0.020)),
        origin=Origin(xyz=(0.305, -0.008, 0.745)),
        material=gasket,
        name="gasket_top",
    )
    door.visual(
        Box((0.455, 0.016, 0.020)),
        origin=Origin(xyz=(0.305, -0.008, 0.035)),
        material=gasket,
        name="gasket_bottom",
    )
    door.visual(
        Box((0.020, 0.016, 0.705)),
        origin=Origin(xyz=(0.075, -0.008, 0.390)),
        material=gasket,
        name="gasket_side_0",
    )
    door.visual(
        Box((0.020, 0.016, 0.705)),
        origin=Origin(xyz=(0.535, -0.008, 0.390)),
        material=gasket,
        name="gasket_side_1",
    )
    for i, (z_center, z_len) in enumerate(((0.130, 0.160), (0.415, 0.165), (0.695, 0.135))):
        door.visual(
            Cylinder(radius=0.012, length=z_len),
            origin=Origin(xyz=(0.0, 0.0, z_center)),
            material=hinge_metal,
            name=f"door_hinge_barrel_{i}",
        )
        door.visual(
            Box((0.028, 0.014, z_len)),
            origin=Origin(xyz=(0.019, -0.015, z_center)),
            material=hinge_metal,
            name=f"door_hinge_leaf_{i}",
        )
    door.visual(
        Cylinder(radius=0.012, length=0.420),
        origin=Origin(xyz=(0.510, -0.114, 0.430)),
        material=handle_mat,
        name="front_handle",
    )
    for i, z in enumerate((0.265, 0.595)):
        door.visual(
            Cylinder(radius=0.006, length=0.040),
            origin=Origin(xyz=(0.510, -0.096, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=handle_mat,
            name=f"handle_standoff_{i}",
        )

    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, 0.020)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=0.0, upper=1.85),
    )

    # Small internal freezer flap, hinged across its upper edge.
    freezer_flap = model.part("freezer_flap")
    freezer_flap.visual(
        Box((0.380, 0.018, 0.160)),
        origin=Origin(xyz=(0.0, -0.006, -0.080)),
        material=freezer_mat,
        name="flap_panel",
    )
    freezer_flap.visual(
        Cylinder(radius=0.008, length=0.390),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_metal,
        name="flap_hinge_barrel",
    )
    freezer_flap.visual(
        Box((0.270, 0.020, 0.020)),
        origin=Origin(xyz=(0.0, -0.022, -0.145)),
        material=handle_mat,
        name="flap_pull_lip",
    )

    model.articulation(
        "cabinet_to_freezer_flap",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=freezer_flap,
        origin=Origin(xyz=(0.0, -0.238, 0.735)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.5, lower=0.0, upper=1.30),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    freezer_flap = object_model.get_part("freezer_flap")
    door_hinge = object_model.get_articulation("cabinet_to_door")
    flap_hinge = object_model.get_articulation("cabinet_to_freezer_flap")

    ctx.check(
        "two visible hinged mechanisms",
        len(object_model.articulations) == 2
        and door_hinge.articulation_type == ArticulationType.REVOLUTE
        and flap_hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"articulations={[joint.name for joint in object_model.articulations]}",
    )

    with ctx.pose({door_hinge: 0.0, flap_hinge: 0.0}):
        ctx.expect_gap(
            cabinet,
            door,
            axis="y",
            positive_elem="front_frame_top",
            negative_elem="gasket_top",
            min_gap=0.0,
            max_gap=0.012,
            name="closed door gasket sits just in front of cabinet frame",
        )
        ctx.expect_overlap(
            door,
            cabinet,
            axes="xz",
            elem_a="door_panel",
            elem_b="front_frame_top",
            min_overlap=0.020,
            name="main door covers the front opening",
        )
        ctx.expect_gap(
            cabinet,
            freezer_flap,
            axis="y",
            positive_elem="freezer_floor",
            negative_elem="flap_panel",
            min_gap=0.0,
            max_gap=0.020,
            name="freezer flap is seated in front of freezer compartment",
        )
        ctx.expect_overlap(
            freezer_flap,
            cabinet,
            axes="x",
            elem_a="flap_panel",
            elem_b="freezer_floor",
            min_overlap=0.300,
            name="freezer flap spans most of the freezer opening",
        )
        closed_door_box = ctx.part_element_world_aabb(door, elem="door_panel")
        closed_flap_box = ctx.part_element_world_aabb(freezer_flap, elem="flap_panel")

    with ctx.pose({door_hinge: 1.45, flap_hinge: 1.05}):
        opened_door_box = ctx.part_element_world_aabb(door, elem="door_panel")
        opened_flap_box = ctx.part_element_world_aabb(freezer_flap, elem="flap_panel")

    ctx.check(
        "main door swings outward from side hinge",
        closed_door_box is not None
        and opened_door_box is not None
        and opened_door_box[0][1] < closed_door_box[0][1] - 0.18,
        details=f"closed={closed_door_box}, opened={opened_door_box}",
    )
    ctx.check(
        "freezer flap lifts forward about upper hinge",
        closed_flap_box is not None
        and opened_flap_box is not None
        and opened_flap_box[0][1] < closed_flap_box[0][1] - 0.10
        and opened_flap_box[0][2] > closed_flap_box[0][2] + 0.04,
        details=f"closed={closed_flap_box}, opened={opened_flap_box}",
    )

    return ctx.report()


object_model = build_object_model()
