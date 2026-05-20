from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="camera_hot_shoe_slider")

    # Materials
    model.material("rail_black", rgba=(0.18, 0.18, 0.20, 1.0))
    model.material("plate_silver", rgba=(0.72, 0.75, 0.78, 1.0))
    model.material("screw_zinc", rgba=(0.82, 0.84, 0.86, 1.0))
    model.material("pad_dark", rgba=(0.22, 0.22, 0.24, 1.0))

    # Dimensions
    rail_length = 0.120      # 120mm rail
    rail_width = 0.025       # 25mm wide
    rail_height = 0.008      # 8mm thick
    lip_height = 0.006       # 6mm tall end lips
    lip_length = 0.010       # 10mm deep lips

    plate_length = 0.080     # 80mm plate
    plate_width = 0.022      # 22mm wide (fits within rail)
    plate_height = 0.006     # 6mm thick

    pad_size = 0.016         # 16mm accessory pad
    pad_height = 0.010       # 10mm raised pad

    screw_radius = 0.004     # 4mm thumb screw
    screw_height = 0.014     # 14mm screw

    travel = 0.040           # 40mm travel (plate 80mm, gap between lips 80mm)

    # Build base rail with beveled edges and end lips using CadQuery
    rail_body = (
        cq.Workplane("XY")
        .box(rail_length, rail_width, rail_height)
        .edges("|Z").fillet(0.002)  # Fillet top/bottom edges
    )

    # Add end lips to retain the sliding plate
    # Lips are at each end, extending upward
    lip_profile = (
        cq.Workplane("XY")
        .rect(lip_length, rail_width)
        .extrude(lip_height)
    )

    rail_with_lips = (
        cq.Workplane("XY")
        .union(rail_body)
        .union(lip_profile.translate((rail_length/2 - lip_length/2, 0, rail_height/2)))
        .union(lip_profile.translate((-rail_length/2 + lip_length/2, 0, rail_height/2)))
    )

    base_rail = model.part("base_rail")
    base_rail.visual(
        mesh_from_cadquery(rail_with_lips, "base_rail"),
        material="rail_black",
        name="rail_body",
    )

    # Build sliding plate with raised accessory pad
    plate_body = (
        cq.Workplane("XY")
        .box(plate_length, plate_width, plate_height)
        .edges("|Z").fillet(0.0015)  # Fillet edges
    )

    # Add raised accessory pad on top (for mounting accessories)
    accessory_pad = (
        cq.Workplane("XY")
        .box(pad_size, pad_size, pad_height)
        .edges("|Z").fillet(0.002)
    )

    plate_with_pad = (
        cq.Workplane("XY")
        .union(plate_body)
        .union(accessory_pad.translate((0, 0, plate_height/2 + pad_height/2)))
    )

    sliding_plate = model.part("sliding_plate")
    sliding_plate.visual(
        mesh_from_cadquery(plate_with_pad, "sliding_plate"),
        material="plate_silver",
        name="plate_body",
    )

    # Add locking thumb screw visual on top of the plate
    sliding_plate.visual(
        Cylinder(radius=screw_radius, length=screw_height),
        origin=Origin(xyz=(0.0, 0.0, plate_height/2 + pad_height + screw_height/2)),
        material="screw_zinc",
        name="thumb_screw",
    )

    # Prismatic joint for sliding motion along X axis
    # At q=0, plate is centered on the rail
    # Positive q moves the plate outward along +X
    model.articulation(
        "rail_to_plate",
        ArticulationType.PRISMATIC,
        parent=base_rail,
        child=sliding_plate,
        origin=Origin(xyz=(0.0, 0.0, rail_height/2 + plate_height/2)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-travel/2,
            upper=travel/2,
            effort=50.0,
            velocity=0.30,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_rail = object_model.get_part("base_rail")
    sliding_plate = object_model.get_part("sliding_plate")
    rail_to_plate = object_model.get_articulation("rail_to_plate")

    # Allow intentional overlap: sliding plate sits on rail and is retained by lips
    ctx.allow_overlap(
        "sliding_plate",
        "base_rail",
        reason="Sliding plate intentionally sits on rail surface and is retained by end lips; contact is seating, not collision.",
    )

    # Test that sliding plate is centered on rail in Y axis
    ctx.expect_within(
        sliding_plate,
        base_rail,
        axes="y",
        margin=0.002,
        name="sliding plate is centered on rail in Y",
    )

    # Test that sliding plate overlaps with rail on X axis (retained insertion)
    ctx.expect_overlap(
        sliding_plate,
        base_rail,
        axes="x",
        min_overlap=0.020,
        name="sliding plate retains insertion with rail",
    )

    # Test thumb screw is mounted on top of plate
    ctx.expect_contact(
        sliding_plate,
        sliding_plate,
        elem_a="thumb_screw",
        elem_b="plate_body",
        name="thumb screw contacts sliding plate",
    )

    # Test the sliding mechanism - check it moves along +X
    rest_pos = ctx.part_world_position(sliding_plate)
    with ctx.pose({rail_to_plate: 0.020}):
        ctx.expect_within(
            sliding_plate,
            base_rail,
            axes="y",
            margin=0.002,
            name="extended plate is centered on rail in Y",
        )
        extended_pos = ctx.part_world_position(sliding_plate)

    ctx.check(
        "sliding plate extends along +X",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.01,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    # Check that plate reaches near the end lips at full travel
    with ctx.pose({rail_to_plate: 0.020}):
        ctx.expect_overlap(
            sliding_plate,
            base_rail,
            axes="x",
            min_overlap=0.030,
            name="extended plate still retains insertion",
        )

    return ctx.report()


object_model = build_object_model()
