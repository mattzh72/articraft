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
    model = ArticulatedObject(name="meter_cubicle")

    powder_coat = Material("warm_light_gray_powder_coat", rgba=(0.72, 0.72, 0.68, 1.0))
    dark_trim = Material("dark_gray_trim", rgba=(0.10, 0.11, 0.11, 1.0))
    black = Material("black_rubber_and_hardware", rgba=(0.01, 0.01, 0.012, 1.0))
    brass = Material("brushed_brass_latch", rgba=(0.72, 0.56, 0.25, 1.0))
    meter_face = Material("matte_off_white_meter_faces", rgba=(0.88, 0.88, 0.82, 1.0))
    glass = Material("slightly_blue_clear_polycarbonate", rgba=(0.48, 0.76, 0.95, 0.32))
    red = Material("red_meter_needles", rgba=(0.85, 0.05, 0.02, 1.0))
    copper = Material("copper_bus_bars", rgba=(0.75, 0.36, 0.15, 1.0))

    enclosure = model.part("enclosure")

    # Rectangular wall-mounted cubicle body: a shallow hollow metal enclosure with
    # an open front meter window and a back-mounted instrument bank.
    enclosure.visual(Box((0.80, 0.030, 1.20)), origin=Origin(xyz=(0.0, 0.075, 0.60)), material=powder_coat, name="back_plate")
    enclosure.visual(Box((0.038, 0.180, 1.20)), origin=Origin(xyz=(-0.381, 0.0, 0.60)), material=powder_coat, name="side_wall_0")
    enclosure.visual(Box((0.038, 0.180, 1.20)), origin=Origin(xyz=(0.381, 0.0, 0.60)), material=powder_coat, name="side_wall_1")
    enclosure.visual(Box((0.80, 0.180, 0.038)), origin=Origin(xyz=(0.0, 0.0, 1.181)), material=powder_coat, name="top_wall")
    enclosure.visual(Box((0.80, 0.180, 0.038)), origin=Origin(xyz=(0.0, 0.0, 0.019)), material=powder_coat, name="bottom_wall")

    # Fixed front face around the meter-window opening.  The broad lower rail
    # carries the latch keeper while the top rail carries the hinge knuckles.
    enclosure.visual(Box((0.084, 0.034, 0.86)), origin=Origin(xyz=(-0.358, -0.100, 0.625)), material=powder_coat, name="front_stile_0")
    enclosure.visual(Box((0.084, 0.034, 0.86)), origin=Origin(xyz=(0.358, -0.100, 0.625)), material=powder_coat, name="front_stile_1")
    enclosure.visual(Box((0.80, 0.034, 0.090)), origin=Origin(xyz=(0.0, -0.100, 1.030)), material=powder_coat, name="front_top_rail")
    enclosure.visual(Box((0.80, 0.034, 0.120)), origin=Origin(xyz=(0.0, -0.100, 0.200)), material=powder_coat, name="front_bottom_rail")

    # Meter bank backboard and four round meter instruments behind the window.
    enclosure.visual(Box((0.62, 0.082, 0.75)), origin=Origin(xyz=(0.0, 0.022, 0.615)), material=dark_trim, name="meter_backboard")
    meter_positions = [(-0.18, 0.75), (0.18, 0.75), (-0.18, 0.47), (0.18, 0.47)]
    for idx, (x, z) in enumerate(meter_positions):
        enclosure.visual(
            Cylinder(radius=0.088, length=0.044),
            origin=Origin(xyz=(x, -0.022, z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=black,
            name=f"meter_ring_{idx}",
        )
        enclosure.visual(
            Cylinder(radius=0.074, length=0.010),
            origin=Origin(xyz=(x, -0.049, z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=meter_face,
            name=f"meter_dial_{idx}",
        )
        enclosure.visual(
            Box((0.010, 0.004, 0.070)),
            origin=Origin(xyz=(x + 0.018, -0.056, z + 0.018), rpy=(0.0, 0.58, 0.0)),
            material=red,
            name=f"meter_needle_{idx}",
        )
        enclosure.visual(
            Box((0.060, 0.006, 0.018)),
            origin=Origin(xyz=(x, -0.056, z - 0.046)),
            material=black,
            name=f"meter_counter_{idx}",
        )

    enclosure.visual(Box((0.055, 0.010, 0.55)), origin=Origin(xyz=(-0.325, -0.020, 0.61)), material=copper, name="bus_bar_0")
    enclosure.visual(Box((0.055, 0.010, 0.55)), origin=Origin(xyz=(0.325, -0.020, 0.61)), material=copper, name="bus_bar_1")

    # Alternating parent-side hinge knuckles mounted to the upper rail.
    for idx, x in enumerate((-0.255, 0.0, 0.255)):
        enclosure.visual(
            Box((0.108, 0.024, 0.016)),
            origin=Origin(xyz=(x, -0.126, 1.035)),
            material=dark_trim,
            name=f"fixed_hinge_leaf_{idx}",
        )
        enclosure.visual(
            Cylinder(radius=0.018, length=0.120),
            origin=Origin(xyz=(x, -0.135, 1.055), rpy=(0.0, pi / 2.0, 0.0)),
            material=dark_trim,
            name=f"fixed_knuckle_{idx}",
        )

    # Bottom compression-latch keeper/catch fixed to the cubicle face.
    enclosure.visual(Box((0.145, 0.020, 0.034)), origin=Origin(xyz=(0.0, -0.105, 0.275)), material=black, name="latch_keeper")

    cover = model.part("cover")
    # The child frame lies on the top hinge axis.  The cover hangs downward in
    # local -Z when q=0; positive hinge travel swings the lower edge outward.
    cover.visual(Box((0.705, 0.020, 0.035)), origin=Origin(xyz=(0.0, -0.014, -0.045)), material=dark_trim, name="top_frame")
    cover.visual(Box((0.705, 0.020, 0.035)), origin=Origin(xyz=(0.0, -0.014, -0.780)), material=dark_trim, name="bottom_frame")
    cover.visual(Box((0.036, 0.020, 0.790)), origin=Origin(xyz=(-0.335, -0.014, -0.399)), material=dark_trim, name="side_frame_0")
    cover.visual(Box((0.036, 0.020, 0.790)), origin=Origin(xyz=(0.335, -0.014, -0.399)), material=dark_trim, name="side_frame_1")
    cover.visual(Box((0.610, 0.008, 0.700)), origin=Origin(xyz=(0.0, -0.018, -0.405)), material=glass, name="clear_window")
    cover.visual(Box((0.640, 0.006, 0.014)), origin=Origin(xyz=(0.0, -0.010, -0.057)), material=black, name="top_gasket")
    cover.visual(Box((0.640, 0.006, 0.014)), origin=Origin(xyz=(0.0, -0.010, -0.753)), material=black, name="bottom_gasket")
    cover.visual(Box((0.014, 0.006, 0.700)), origin=Origin(xyz=(-0.312, -0.010, -0.405)), material=black, name="side_gasket_0")
    cover.visual(Box((0.014, 0.006, 0.700)), origin=Origin(xyz=(0.312, -0.010, -0.405)), material=black, name="side_gasket_1")

    for idx, x in enumerate((-0.145, 0.145)):
        cover.visual(
            Cylinder(radius=0.018, length=0.110),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=dark_trim,
            name=f"cover_knuckle_{idx}",
        )
        cover.visual(
            Box((0.070, 0.010, 0.036)),
            origin=Origin(xyz=(x, -0.010, -0.026)),
            material=dark_trim,
            name=f"hinge_leaf_{idx}",
        )

    latch = model.part("latch")
    latch.visual(
        Cylinder(radius=0.040, length=0.020),
        origin=Origin(xyz=(0.0, -0.022, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="thumb_turn",
    )
    latch.visual(
        Box((0.090, 0.010, 0.022)),
        origin=Origin(xyz=(0.0, -0.034, 0.0)),
        material=black,
        name="grip_slot",
    )
    latch.visual(
        Cylinder(radius=0.014, length=0.050),
        origin=Origin(xyz=(0.0, 0.000, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="latch_shaft",
    )
    latch.visual(
        Box((0.126, 0.010, 0.024)),
        origin=Origin(xyz=(0.0, 0.030, 0.0)),
        material=brass,
        name="cam_blade",
    )

    model.articulation(
        "cover_hinge",
        ArticulationType.REVOLUTE,
        parent=enclosure,
        child=cover,
        origin=Origin(xyz=(0.0, -0.135, 1.055)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=0.0, upper=1.75),
    )
    model.articulation(
        "latch_turn",
        ArticulationType.REVOLUTE,
        parent=cover,
        child=latch,
        origin=Origin(xyz=(0.0, -0.018, -0.780)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.0, lower=0.0, upper=1.5708),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    enclosure = object_model.get_part("enclosure")
    cover = object_model.get_part("cover")
    latch = object_model.get_part("latch")
    cover_hinge = object_model.get_articulation("cover_hinge")
    latch_turn = object_model.get_articulation("latch_turn")

    ctx.allow_overlap(
        cover,
        latch,
        elem_a="bottom_frame",
        elem_b="latch_shaft",
        reason="The compression latch shaft intentionally passes through the cover frame bore.",
    )
    with ctx.pose({cover_hinge: 0.0, latch_turn: 0.0}):
        ctx.expect_overlap(cover, enclosure, axes="xz", elem_a="clear_window", elem_b="meter_backboard", min_overlap=0.45, name="cover spans the meter bank")
        ctx.expect_gap(enclosure, cover, axis="y", positive_elem="front_top_rail", negative_elem="top_frame", min_gap=0.0, max_gap=0.030, name="closed cover sits just in front of the enclosure")
        ctx.expect_contact(cover, latch, elem_a="bottom_frame", elem_b="latch_shaft", contact_tol=0.030, name="latch shaft is mounted through bottom cover frame")
        ctx.expect_overlap(latch, enclosure, axes="xz", elem_a="cam_blade", elem_b="latch_keeper", min_overlap=0.02, name="compression cam aligns with keeper")
        locked_cam = ctx.part_element_world_aabb(latch, elem="cam_blade")

    closed_pos = ctx.part_world_position(latch)
    with ctx.pose({cover_hinge: 1.20}):
        opened_pos = ctx.part_world_position(latch)
        ctx.expect_gap(enclosure, cover, axis="y", positive_elem="front_bottom_rail", negative_elem="bottom_frame", min_gap=0.10, name="open cover swings outward from the bottom rail")

    ctx.check(
        "hinged cover moves the latch outward",
        closed_pos is not None and opened_pos is not None and opened_pos[1] < closed_pos[1] - 0.05,
        details=f"closed latch position={closed_pos}, opened latch position={opened_pos}",
    )

    with ctx.pose({cover_hinge: 0.0, latch_turn: 1.5708}):
        turned_cam = ctx.part_element_world_aabb(latch, elem="cam_blade")

    if locked_cam is not None and turned_cam is not None:
        locked_x = locked_cam[1][0] - locked_cam[0][0]
        locked_z = locked_cam[1][2] - locked_cam[0][2]
        turned_x = turned_cam[1][0] - turned_cam[0][0]
        turned_z = turned_cam[1][2] - turned_cam[0][2]
    else:
        locked_x = locked_z = turned_x = turned_z = 0.0
    ctx.check(
        "latch cam makes a quarter turn",
        locked_x > locked_z * 3.0 and turned_z > turned_x * 3.0,
        details=f"locked_cam={locked_cam}, turned_cam={turned_cam}",
    )

    return ctx.report()


object_model = build_object_model()
