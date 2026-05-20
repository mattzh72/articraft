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


def _filleted_body_mesh():
    """Machined rectangular USB body with slightly softened vertical edges."""
    shape = cq.Workplane("XY").box(0.060, 0.020, 0.008)
    shape = shape.edges("|Z").fillet(0.0022)
    return mesh_from_cadquery(
        shape,
        "machined_body",
        tolerance=0.00018,
        angular_tolerance=0.04,
    )


def _swivel_cover_mesh():
    """Single-piece cover strap: long cover plate plus pivot lobe and clearance hole."""
    thickness = 0.0016
    lobe = cq.Workplane("XY").circle(0.0065).extrude(thickness)
    strap = (
        cq.Workplane("XY")
        .center(0.036, -0.014)
        .rect(0.088, 0.024)
        .extrude(thickness)
    )
    hole = cq.Workplane("XY").circle(0.0028).extrude(thickness + 0.002)
    shape = lobe.union(strap).cut(hole)
    return mesh_from_cadquery(
        shape,
        "swivel_cover_plate",
        tolerance=0.00016,
        angular_tolerance=0.04,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_usb_swivel_drive")

    anodized = model.material("dark_anodized_aluminum", rgba=(0.06, 0.07, 0.08, 1.0))
    satin = model.material("satin_stainless_steel", rgba=(0.66, 0.68, 0.66, 1.0))
    cover_metal = model.material("brushed_cover_metal", rgba=(0.48, 0.51, 0.52, 1.0))
    black = model.material("black_polymer", rgba=(0.01, 0.012, 0.014, 1.0))
    laser = model.material("laser_etched_black", rgba=(0.0, 0.0, 0.0, 1.0))
    blue = model.material("calibration_blue", rgba=(0.1, 0.26, 0.9, 1.0))
    gold = model.material("contact_gold", rgba=(1.0, 0.73, 0.22, 1.0))

    # Main drive frame: +X is the USB connector end.  The side pivot is on +Y.
    body = model.part("body")
    body.visual(_filleted_body_mesh(), material=anodized, name="body_shell")

    # USB-A connector shell, represented as four clearanced plates plus the
    # internal tongue so the front actually reads as an open connector.
    body.visual(
        Box((0.016, 0.012, 0.0007)),
        origin=Origin(xyz=(0.038, 0.0, 0.00225)),
        material=satin,
        name="connector_top",
    )
    body.visual(
        Box((0.016, 0.012, 0.0007)),
        origin=Origin(xyz=(0.038, 0.0, -0.00225)),
        material=satin,
        name="connector_bottom",
    )
    body.visual(
        Box((0.016, 0.0007, 0.0045)),
        origin=Origin(xyz=(0.038, 0.00615, 0.0)),
        material=satin,
        name="connector_side_0",
    )
    body.visual(
        Box((0.016, 0.0007, 0.0045)),
        origin=Origin(xyz=(0.038, -0.00615, 0.0)),
        material=satin,
        name="connector_side_1",
    )
    body.visual(
        Box((0.026, 0.0065, 0.0009)),
        origin=Origin(xyz=(0.033, 0.0, 0.0)),
        material=black,
        name="connector_tongue",
    )
    for i, y in enumerate((-0.0017, 0.0017)):
        body.visual(
            Box((0.0045, 0.00085, 0.00016)),
            origin=Origin(xyz=(0.043, y, 0.00050)),
            material=gold,
            name=f"contact_pad_{i}",
        )

    # Datum-friendly raised rails and tiny gap pads.  Their top faces sit below
    # the cover underside, giving a controlled visible clearance instead of a
    # fused plate.
    for i, y in enumerate((-0.0064, 0.0064)):
        body.visual(
            Box((0.047, 0.00125, 0.0005)),
            origin=Origin(xyz=(0.000, y, 0.00425)),
            material=satin,
            name=f"datum_rail_{i}",
        )
    for i, x in enumerate((-0.012, 0.018)):
        body.visual(
            Box((0.004, 0.003, 0.0007)),
            origin=Origin(xyz=(x, -0.008, 0.00435)),
            material=blue,
            name=f"gap_pad_{i}",
        )

    # Fixed calibration scale and datum fence on the top face.
    body.visual(
        Box((0.0012, 0.016, 0.00016)),
        origin=Origin(xyz=(0.022, 0.0, 0.00408)),
        material=laser,
        name="datum_fence",
    )
    for i, x in enumerate((-0.024, -0.018, -0.012, -0.006)):
        body.visual(
            Box((0.0008, 0.008, 0.00016)),
            origin=Origin(xyz=(x, 0.000, 0.00408)),
            material=laser,
            name=f"index_tick_{i}",
        )

    # Side-pinned pivot stack: a side lug grows out of the body, and the visible
    # pin passes through the cover's clearance hole.  No part is free floating.
    pivot_x = -0.024
    pivot_y = 0.014
    body.visual(
        Box((0.016, 0.008, 0.006)),
        origin=Origin(xyz=(pivot_x, pivot_y, 0.0)),
        material=anodized,
        name="side_lug",
    )
    body.visual(
        Cylinder(radius=0.0030, length=0.014),
        origin=Origin(xyz=(pivot_x, pivot_y, 0.0028)),
        material=satin,
        name="pivot_pin",
    )
    body.visual(
        Cylinder(radius=0.0035, length=0.0012),
        origin=Origin(xyz=(pivot_x, pivot_y, 0.0074)),
        material=satin,
        name="pin_head",
    )

    # Two exposed calibration adjusters.  They are captive screws on revolute
    # axes so the adjustment features remain articulated rather than painted on.
    screw_positions = [(-0.004, 0.0), (0.014, 0.0)]
    for i, (x, y) in enumerate(screw_positions):
        screw = model.part(f"set_screw_{i}")
        screw.visual(
            Cylinder(radius=0.0021, length=0.0008),
            origin=Origin(xyz=(0.0, 0.0, 0.0004)),
            material=satin,
            name="screw_head",
        )
        screw.visual(
            Box((0.0035, 0.00045, 0.00022)),
            origin=Origin(xyz=(0.0, 0.0, 0.00082)),
            material=laser,
            name="screw_slot",
        )
        model.articulation(
            f"body_to_set_screw_{i}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=screw,
            origin=Origin(xyz=(x, y, 0.0040)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.3, velocity=2.0),
        )

    cover = model.part("swivel_cover")
    cover.visual(
        _swivel_cover_mesh(),
        origin=Origin(xyz=(0.0, 0.0, 0.0052)),
        material=cover_metal,
        name="cover_plate",
    )
    cover.visual(
        Box((0.082, 0.0012, 0.0040)),
        origin=Origin(xyz=(0.037, -0.0255, 0.0032)),
        material=cover_metal,
        name="side_lip",
    )
    cover.visual(
        Box((0.010, 0.00055, 0.00018)),
        origin=Origin(xyz=(0.000, -0.0062, 0.00689)),
        material=laser,
        name="cover_pointer",
    )
    for i, x in enumerate((0.016, 0.030, 0.044)):
        cover.visual(
            Box((0.0007, 0.010, 0.00018)),
            origin=Origin(xyz=(x, -0.014, 0.00689)),
            material=laser,
            name=f"cover_tick_{i}",
        )

    model.articulation(
        "body_to_swivel_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cover,
        origin=Origin(xyz=(pivot_x, pivot_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.2, velocity=2.0, lower=0.0, upper=2.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    cover = object_model.get_part("swivel_cover")
    hinge = object_model.get_articulation("body_to_swivel_cover")

    ctx.allow_overlap(
        body,
        cover,
        elem_a="pivot_pin",
        elem_b="cover_plate",
        reason=(
            "The visible pivot pin is intentionally modeled as a slight press fit "
            "through the cover bearing hole so the swivel cover is mechanically captured."
        ),
    )
    ctx.expect_gap(
        cover,
        body,
        axis="z",
        min_gap=0.0005,
        max_gap=0.0018,
        positive_elem="cover_plate",
        negative_elem="datum_rail_0",
        name="cover rides above datum rail with controlled gap",
    )
    ctx.expect_gap(
        body,
        cover,
        axis="z",
        min_gap=-0.00005,
        max_gap=0.00025,
        positive_elem="pin_head",
        negative_elem="cover_plate",
        name="pin head bears on cover top without floating",
    )
    ctx.expect_overlap(
        body,
        cover,
        axes="xy",
        min_overlap=0.003,
        elem_a="pivot_pin",
        elem_b="cover_plate",
        name="cover hole remains centered around visible pivot pin",
    )
    ctx.expect_overlap(
        cover,
        body,
        axes="xy",
        min_overlap=0.010,
        elem_a="cover_plate",
        elem_b="connector_top",
        name="closed cover spans the USB connector",
    )

    limits = hinge.motion_limits
    ctx.check(
        "swivel cover has repeatable limited rotation",
        limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and 2.6 < limits.upper < 2.9,
        details=f"limits={limits}",
    )

    closed_aabb = ctx.part_element_world_aabb(cover, elem="cover_plate")
    connector_aabb = ctx.part_element_world_aabb(body, elem="connector_top")
    with ctx.pose({hinge: 2.75}):
        open_aabb = ctx.part_element_world_aabb(cover, elem="cover_plate")
        ctx.expect_gap(
            cover,
            body,
            axis="z",
            min_gap=0.0005,
            max_gap=0.0018,
            positive_elem="cover_plate",
            negative_elem="datum_rail_0",
            name="open cover preserves vertical running clearance",
        )

    if closed_aabb is not None and open_aabb is not None and connector_aabb is not None:
        closed_center_y = (closed_aabb[0][1] + closed_aabb[1][1]) * 0.5
        open_center_y = (open_aabb[0][1] + open_aabb[1][1]) * 0.5
        ctx.check(
            "open swivel cover clears connector end",
            open_aabb[1][0] < connector_aabb[0][0] - 0.025
            and open_center_y > closed_center_y + 0.020,
            details=f"closed={closed_aabb}, open={open_aabb}, connector={connector_aabb}",
        )
    else:
        ctx.fail("open swivel cover clears connector end", "missing cover or connector AABB")

    return ctx.report()


object_model = build_object_model()
