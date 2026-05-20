from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sliding_monitor_mount")

    satin_black = Material("satin_black", rgba=(0.02, 0.022, 0.025, 1.0))
    dark_rail = Material("dark_anodized_rail", rgba=(0.08, 0.085, 0.09, 1.0))
    carriage_finish = Material("powder_coated_carriage", rgba=(0.12, 0.125, 0.13, 1.0))
    hinge_steel = Material("brushed_hinge_steel", rgba=(0.62, 0.62, 0.58, 1.0))
    hole_black = Material("black_recesses", rgba=(0.0, 0.0, 0.0, 1.0))

    rail = model.part("rail")
    rail.visual(
        Box((0.90, 0.035, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_rail,
        name="wall_plate",
    )
    rail.visual(
        Box((0.82, 0.035, 0.025)),
        origin=Origin(xyz=(0.0, 0.035, 0.045)),
        material=satin_black,
        name="top_track",
    )
    rail.visual(
        Box((0.82, 0.035, 0.025)),
        origin=Origin(xyz=(0.0, 0.035, -0.045)),
        material=satin_black,
        name="bottom_track",
    )
    rail.visual(
        Box((0.030, 0.070, 0.160)),
        origin=Origin(xyz=(-0.435, 0.018, 0.0)),
        material=dark_rail,
        name="end_stop_0",
    )
    rail.visual(
        Box((0.030, 0.070, 0.160)),
        origin=Origin(xyz=(0.435, 0.018, 0.0)),
        material=dark_rail,
        name="end_stop_1",
    )
    for i, x in enumerate((-0.34, 0.34)):
        for j, z in enumerate((-0.045, 0.045)):
            rail.visual(
                Cylinder(radius=0.012, length=0.006),
                origin=Origin(xyz=(x, 0.0205, z), rpy=(-pi / 2.0, 0.0, 0.0)),
                material=hinge_steel,
                name=f"mount_screw_{i}_{j}",
            )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.160, 0.045, 0.170)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=carriage_finish,
        name="front_slide_plate",
    )
    carriage.visual(
        Box((0.120, 0.045, 0.065)),
        origin=Origin(xyz=(0.0, -0.045, 0.0)),
        material=carriage_finish,
        name="slot_tongue",
    )
    carriage.visual(
        Box((0.075, 0.080, 0.080)),
        origin=Origin(xyz=(0.0, 0.060, 0.0)),
        material=carriage_finish,
        name="hinge_neck",
    )
    carriage.visual(
        Cylinder(radius=0.016, length=0.055),
        origin=Origin(xyz=(0.0, 0.116, 0.055)),
        material=carriage_finish,
        name="upper_barrel",
    )
    carriage.visual(
        Cylinder(radius=0.016, length=0.055),
        origin=Origin(xyz=(0.0, 0.116, -0.055)),
        material=carriage_finish,
        name="lower_barrel",
    )

    display_plate = model.part("display_plate")
    display_plate.visual(
        Cylinder(radius=0.006, length=0.180),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=hinge_steel,
        name="hinge_pin",
    )
    display_plate.visual(
        Cylinder(radius=0.014, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=carriage_finish,
        name="center_knuckle",
    )
    display_plate.visual(
        Box((0.100, 0.075, 0.055)),
        origin=Origin(xyz=(0.0, 0.040, 0.0)),
        material=carriage_finish,
        name="hinge_leaf",
    )
    display_plate.visual(
        Box((0.420, 0.018, 0.280)),
        origin=Origin(xyz=(0.0, 0.078, 0.0)),
        material=satin_black,
        name="vesa_plate",
    )
    display_plate.visual(
        Box((0.340, 0.008, 0.205)),
        origin=Origin(xyz=(0.0, 0.091, 0.0)),
        material=carriage_finish,
        name="raised_panel",
    )
    for i, x in enumerate((-0.050, 0.050)):
        for j, z in enumerate((-0.050, 0.050)):
            display_plate.visual(
                Cylinder(radius=0.012, length=0.004),
                origin=Origin(xyz=(x, 0.097, z), rpy=(-pi / 2.0, 0.0, 0.0)),
                material=hole_black,
                name=f"vesa_hole_{i}_{j}",
            )

    model.articulation(
        "rail_to_carriage",
        ArticulationType.PRISMATIC,
        parent=rail,
        child=carriage,
        origin=Origin(xyz=(-0.300, 0.095, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.35, lower=0.0, upper=0.600),
    )
    model.articulation(
        "carriage_to_display_plate",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=display_plate,
        origin=Origin(xyz=(0.0, 0.116, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=-0.75, upper=0.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    rail = object_model.get_part("rail")
    carriage = object_model.get_part("carriage")
    display_plate = object_model.get_part("display_plate")
    slide = object_model.get_articulation("rail_to_carriage")
    hinge = object_model.get_articulation("carriage_to_display_plate")

    ctx.allow_overlap(
        display_plate,
        carriage,
        elem_a="hinge_pin",
        elem_b="upper_barrel",
        reason="The steel hinge pin is intentionally captured inside the solid proxy of the upper hinge barrel.",
    )
    ctx.allow_overlap(
        display_plate,
        carriage,
        elem_a="hinge_pin",
        elem_b="lower_barrel",
        reason="The steel hinge pin is intentionally captured inside the solid proxy of the lower hinge barrel.",
    )

    with ctx.pose({slide: 0.0, hinge: 0.0}):
        ctx.expect_contact(
            carriage,
            rail,
            elem_a="slot_tongue",
            elem_b="top_track",
            contact_tol=0.001,
            name="tongue bears on upper rail track",
        )
        ctx.expect_contact(
            carriage,
            rail,
            elem_a="slot_tongue",
            elem_b="bottom_track",
            contact_tol=0.001,
            name="tongue bears on lower rail track",
        )
        ctx.expect_within(
            carriage,
            rail,
            axes="x",
            inner_elem="slot_tongue",
            outer_elem="top_track",
            margin=0.001,
            name="carriage tongue starts within rail length",
        )
        ctx.expect_within(
            display_plate,
            carriage,
            axes="xy",
            inner_elem="hinge_pin",
            outer_elem="upper_barrel",
            margin=0.001,
            name="pin is centered in upper hinge barrel",
        )
        ctx.expect_within(
            display_plate,
            carriage,
            axes="xy",
            inner_elem="hinge_pin",
            outer_elem="lower_barrel",
            margin=0.001,
            name="pin is centered in lower hinge barrel",
        )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.600, hinge: 0.0}):
        ctx.expect_within(
            carriage,
            rail,
            axes="x",
            inner_elem="slot_tongue",
            outer_elem="top_track",
            margin=0.001,
            name="extended carriage remains retained by rail",
        )
        extended_pos = ctx.part_world_position(carriage)

    ctx.check(
        "carriage slides along the rail",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.55,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    closed_plate_aabb = ctx.part_element_world_aabb(display_plate, elem="vesa_plate")
    with ctx.pose({slide: 0.0, hinge: 0.75}):
        yawed_plate_aabb = ctx.part_element_world_aabb(display_plate, elem="vesa_plate")

    closed_center = None
    yawed_center = None
    if closed_plate_aabb is not None and yawed_plate_aabb is not None:
        closed_center = tuple((closed_plate_aabb[0][i] + closed_plate_aabb[1][i]) * 0.5 for i in range(3))
        yawed_center = tuple((yawed_plate_aabb[0][i] + yawed_plate_aabb[1][i]) * 0.5 for i in range(3))
    ctx.check(
        "display plate yaws on hinge",
        closed_center is not None
        and yawed_center is not None
        and yawed_center[0] < closed_center[0] - 0.04
        and yawed_center[1] < closed_center[1] - 0.01,
        details=f"closed_center={closed_center}, yawed_center={yawed_center}",
    )

    return ctx.report()


object_model = build_object_model()
