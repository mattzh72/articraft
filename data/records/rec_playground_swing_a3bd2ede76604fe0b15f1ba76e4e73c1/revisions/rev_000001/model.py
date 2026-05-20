from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="playground_swing_lap_bar")

    metal = Material("powder_coated_steel", rgba=(0.62, 0.65, 0.66, 1.0))
    dark_metal = Material("dark_hinge_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    wood = Material("sealed_wood_slats", rgba=(0.58, 0.34, 0.15, 1.0))
    red = Material("red_safety_bar", rgba=(0.86, 0.05, 0.03, 1.0))
    rubber = Material("black_rubber_caps", rgba=(0.02, 0.02, 0.018, 1.0))

    frame = model.part("frame")
    # Top beam, side A-frames, and lower skid rails form one rigid playground frame.
    frame.visual(
        Cylinder(radius=0.050, length=1.60),
        origin=Origin(xyz=(0.0, 0.0, 2.14), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="top_beam",
    )
    leg_len = 2.22
    leg_angle = 0.348
    for side_index, y in enumerate((-0.70, 0.70)):
        for front_index, (x, angle) in enumerate(((0.375, -leg_angle), (-0.375, leg_angle))):
            frame.visual(
                Cylinder(radius=0.034, length=leg_len),
                origin=Origin(xyz=(x, y, 1.075), rpy=(0.0, angle, 0.0)),
                material=metal,
                name=f"leg_{side_index}_{front_index}",
            )
        frame.visual(
            Box((1.58, 0.070, 0.045)),
            origin=Origin(xyz=(0.0, y, 0.045)),
            material=metal,
            name=f"ground_rail_{side_index}",
        )

    # Beam-mounted clevis blocks visibly locate the two hanger pivots.
    for idx, y in enumerate((-0.36, 0.36)):
        for plate_idx, plate_y in enumerate((y - 0.055, y + 0.055)):
            frame.visual(
                Box((0.085, 0.016, 0.115)),
                origin=Origin(xyz=(0.0, plate_y, 2.075)),
                material=dark_metal,
                name=f"beam_clevis_{idx}_{plate_idx}",
            )
        frame.visual(
            Cylinder(radius=0.018, length=0.100),
            origin=Origin(xyz=(0.0, y, 2.050), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name=f"beam_pin_{idx}",
        )

    seat = model.part("seat")
    # The seat part frame is the top pivot line; the whole suspended assembly
    # swings around that line.  Two side hanger links descend to a slatted seat.
    for idx, y in enumerate((-0.36, 0.36)):
        seat.visual(
            Cylinder(radius=0.040, length=0.080),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name=f"top_pivot_eye_{idx}",
        )
        seat.visual(
            Box((0.040, 0.035, 1.180)),
            origin=Origin(xyz=(0.0, y, -0.630)),
            material=metal,
            name=f"hanger_link_{idx}",
        )
        seat.visual(
            Box((0.450, 0.048, 0.060)),
            origin=Origin(xyz=(0.0, y, -1.225)),
            material=metal,
            name=f"side_rail_{idx}",
        )
        seat.visual(
            Box((0.040, 0.050, 0.230)),
            origin=Origin(xyz=(0.245, y, -1.105)),
            material=dark_metal,
            name=f"lap_hinge_riser_{idx}",
        )
        seat.visual(
            Cylinder(radius=0.024, length=0.055),
            origin=Origin(xyz=(0.245, y, -0.995), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name=f"lap_hinge_knuckle_{idx}",
        )

    seat.visual(
        Box((0.066, 0.690, 0.038)),
        origin=Origin(xyz=(-0.168, 0.0, -1.205)),
        material=wood,
        name="slat_0",
    )
    seat.visual(
        Box((0.066, 0.690, 0.038)),
        origin=Origin(xyz=(-0.084, 0.0, -1.205)),
        material=wood,
        name="slat_1",
    )
    seat.visual(
        Box((0.066, 0.690, 0.038)),
        origin=Origin(xyz=(0.0, 0.0, -1.205)),
        material=wood,
        name="slat_2",
    )
    seat.visual(
        Box((0.066, 0.690, 0.038)),
        origin=Origin(xyz=(0.084, 0.0, -1.205)),
        material=wood,
        name="slat_3",
    )
    seat.visual(
        Box((0.066, 0.690, 0.038)),
        origin=Origin(xyz=(0.168, 0.0, -1.205)),
        material=wood,
        name="slat_4",
    )

    # Small rear and front cross ties make the slatted board read as a real bolted seat.
    for idx, x in enumerate((-0.215, 0.215)):
        seat.visual(
            Box((0.032, 0.725, 0.030)),
            origin=Origin(xyz=(x, 0.0, -1.180)),
            material=dark_metal,
            name=f"cross_tie_{idx}",
        )

    lap_bar = model.part("lap_bar")
    # The lap bar frame is on the hinge axis at the front seat corners.  Closed,
    # short arms project slightly forward and upward to a padded cross tube.
    for idx, y in enumerate((-0.315, 0.315)):
        lap_bar.visual(
            Cylinder(radius=0.018, length=0.040),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name=f"hinge_pin_{idx}",
        )
        lap_bar.visual(
            Box((0.110, 0.030, 0.030)),
            origin=Origin(xyz=(0.055, y, 0.018)),
            material=red,
            name=f"bar_arm_{idx}",
        )
        lap_bar.visual(
            Box((0.032, 0.030, 0.095)),
            origin=Origin(xyz=(0.110, y, 0.065)),
            material=red,
            name=f"upright_arm_{idx}",
        )

    lap_bar.visual(
        Cylinder(radius=0.027, length=0.650),
        origin=Origin(xyz=(0.110, 0.0, 0.115), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=red,
        name="padded_crossbar",
    )
    for idx, y in enumerate((-0.335, 0.335)):
        lap_bar.visual(
            Sphere(radius=0.030),
            origin=Origin(xyz=(0.110, y, 0.115)),
            material=rubber,
            name=f"bar_cap_{idx}",
        )

    model.articulation(
        "swing_pivot",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 2.050)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=2.0, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "lap_bar_hinge",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=lap_bar,
        origin=Origin(xyz=(0.245, 0.0, -0.995)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=0.0, upper=1.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    seat = object_model.get_part("seat")
    lap_bar = object_model.get_part("lap_bar")
    swing_pivot = object_model.get_articulation("swing_pivot")
    lap_hinge = object_model.get_articulation("lap_bar_hinge")

    for idx in range(2):
        ctx.allow_overlap(
            frame,
            seat,
            elem_a=f"beam_pin_{idx}",
            elem_b=f"top_pivot_eye_{idx}",
            reason="The fixed beam pin is intentionally captured through the hanger pivot eye.",
        )
        ctx.expect_within(
            frame,
            seat,
            axes="xz",
            inner_elem=f"beam_pin_{idx}",
            outer_elem=f"top_pivot_eye_{idx}",
            margin=0.001,
            name=f"beam pin {idx} is centered in its pivot eye",
        )
        ctx.expect_overlap(
            frame,
            seat,
            axes="y",
            elem_a=f"beam_pin_{idx}",
            elem_b=f"top_pivot_eye_{idx}",
            min_overlap=0.070,
            name=f"beam pin {idx} passes through the pivot eye",
        )

    ctx.expect_gap(
        frame,
        seat,
        axis="z",
        positive_elem="top_beam",
        negative_elem="slat_2",
        min_gap=1.0,
        name="slatted seat hangs well below the top beam",
    )
    ctx.expect_gap(
        lap_bar,
        seat,
        axis="z",
        positive_elem="padded_crossbar",
        negative_elem="slat_2",
        min_gap=0.20,
        name="lap bar sits above the slatted seat",
    )
    ctx.expect_gap(
        lap_bar,
        seat,
        axis="x",
        positive_elem="padded_crossbar",
        negative_elem="slat_4",
        min_gap=0.08,
        name="lap bar is mounted at the front edge",
    )

    closed_seat_aabb = ctx.part_world_aabb(seat)
    with ctx.pose({swing_pivot: 0.45}):
        swung_seat_aabb = ctx.part_world_aabb(seat)
    ctx.check(
        "seat swings from the beam pivot",
        closed_seat_aabb is not None
        and swung_seat_aabb is not None
        and abs(
            ((swung_seat_aabb[0][0] + swung_seat_aabb[1][0]) * 0.5)
            - ((closed_seat_aabb[0][0] + closed_seat_aabb[1][0]) * 0.5)
        )
        > 0.20,
        details=f"closed={closed_seat_aabb}, swung={swung_seat_aabb}",
    )

    closed_bar_aabb = ctx.part_element_world_aabb(lap_bar, elem="padded_crossbar")
    with ctx.pose({lap_hinge: 0.75}):
        raised_bar_aabb = ctx.part_element_world_aabb(lap_bar, elem="padded_crossbar")
    ctx.check(
        "lap bar rotates upward on side hinges",
        closed_bar_aabb is not None
        and raised_bar_aabb is not None
        and raised_bar_aabb[1][2] > closed_bar_aabb[1][2] + 0.035,
        details=f"closed={closed_bar_aabb}, raised={raised_bar_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
