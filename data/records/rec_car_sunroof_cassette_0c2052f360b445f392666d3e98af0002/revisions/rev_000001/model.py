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
    model = ArticulatedObject(name="tilt_slide_sunroof_cassette")

    aluminium = model.material("brushed_aluminium", rgba=(0.72, 0.74, 0.73, 1.0))
    dark_anodized = model.material("dark_anodized_track", rgba=(0.05, 0.055, 0.06, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    tinted_glass = model.material("smoked_translucent_glass", rgba=(0.06, 0.12, 0.16, 0.42))
    bearing_steel = model.material("bearing_steel", rgba=(0.80, 0.80, 0.76, 1.0))

    frame = model.part("cassette_frame")
    # Vehicle convention used here: +Y is the front of the opening, -Y is rearward
    # into the cassette.  The frame is a shallow aluminium tray with a large open
    # centre and two long rail channels along the sides.
    frame.visual(
        Box((0.085, 1.45, 0.055)),
        origin=Origin(xyz=(-0.575, -0.10, 0.0275)),
        material=aluminium,
        name="side_member_0",
    )
    frame.visual(
        Box((0.085, 1.45, 0.055)),
        origin=Origin(xyz=(0.575, -0.10, 0.0275)),
        material=aluminium,
        name="side_member_1",
    )
    frame.visual(
        Box((1.15, 0.085, 0.055)),
        origin=Origin(xyz=(0.0, 0.625, 0.0275)),
        material=aluminium,
        name="front_crossmember",
    )
    frame.visual(
        Box((1.15, 0.085, 0.055)),
        origin=Origin(xyz=(0.0, -0.825, 0.0275)),
        material=aluminium,
        name="rear_crossmember",
    )

    frame.visual(
        Box((0.120, 1.41, 0.012)),
        origin=Origin(xyz=(-0.46, -0.10, 0.058)),
        material=dark_anodized,
        name="guide_bottom_0",
    )
    frame.visual(
        Box((0.120, 1.41, 0.012)),
        origin=Origin(xyz=(0.46, -0.10, 0.058)),
        material=dark_anodized,
        name="guide_bottom_1",
    )
    for i, x in enumerate((-0.46, 0.46)):
        for lip, lip_x in enumerate((x - 0.058, x + 0.058)):
            frame.visual(
                Box((0.014, 1.41, 0.040)),
                origin=Origin(xyz=(lip_x, -0.10, 0.074)),
                material=aluminium,
                name=f"guide_lip_{i}_{lip}",
            )

    # Small drain/fastener details make the cassette read as a manufactured roof
    # module while remaining fused to the frame.
    for i, x in enumerate((-0.31, 0.31)):
        frame.visual(
            Cylinder(radius=0.012, length=0.006),
            origin=Origin(xyz=(x, 0.615, 0.0575)),
            material=dark_anodized,
            name=f"front_screw_{i}",
        )
        frame.visual(
            Cylinder(radius=0.012, length=0.006),
            origin=Origin(xyz=(x, -0.815, 0.0575)),
            material=dark_anodized,
            name=f"rear_screw_{i}",
        )

    slide_carrier = model.part("slide_carrier")
    # The carrier frame is located at the rear hinge line.  It moves rearward in
    # the two cassette rails and carries four rolling shoe carriages.
    slide_carrier.visual(
        Cylinder(radius=0.009, length=0.98),
        origin=Origin(xyz=(0.0, 0.0, 0.032), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bearing_steel,
        name="rear_hinge_pin",
    )
    for i, x in enumerate((-0.20, 0.20)):
        slide_carrier.visual(
            Box((0.040, 0.070, 0.030)),
            origin=Origin(xyz=(x, -0.040, 0.012)),
            material=aluminium,
            name=f"hinge_stanchion_{i}",
        )
    slide_carrier.visual(
        Box((0.96, 0.030, 0.018)),
        origin=Origin(xyz=(0.0, 0.008, -0.003)),
        material=aluminium,
        name="rear_tie_bar",
    )

    slide_carrier.visual(
        Box((0.035, 0.89, 0.018)),
        origin=Origin(xyz=(-0.46, 0.445, -0.003)),
        material=aluminium,
        name="side_beam_0",
    )
    slide_carrier.visual(
        Box((0.035, 0.89, 0.018)),
        origin=Origin(xyz=(0.46, 0.445, -0.003)),
        material=aluminium,
        name="side_beam_1",
    )
    for i, x in enumerate((-0.46, 0.46)):
        for j, y in enumerate((0.035, 0.865)):
            slide_carrier.visual(
                Box((0.075, 0.082, 0.030)),
                origin=Origin(xyz=(x, y, -0.017)),
                material=aluminium,
                name=f"shoe_block_{i}_{j}",
            )
            slide_carrier.visual(
                Cylinder(radius=0.018, length=0.054),
                origin=Origin(xyz=(x, y, -0.026), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=black_rubber,
                name=f"shoe_roller_{i}_{j}",
            )
            slide_carrier.visual(
                Cylinder(radius=0.006, length=0.086),
                origin=Origin(xyz=(x, y, -0.026), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=bearing_steel,
                name=f"roller_axle_{i}_{j}",
            )

    glass = model.part("glass_panel")
    # The glass part frame is exactly on the rear hinge line.  The closed panel
    # extends forward (+Y), so a positive rotation about +X lifts the front edge.
    glass.visual(
        Box((0.88, 0.90, 0.016)),
        origin=Origin(xyz=(0.0, 0.505, 0.020)),
        material=tinted_glass,
        name="glass_sheet",
    )
    glass.visual(
        Box((0.94, 0.050, 0.022)),
        origin=Origin(xyz=(0.0, 0.035, 0.019)),
        material=black_rubber,
        name="rear_seal",
    )
    glass.visual(
        Box((0.94, 0.050, 0.022)),
        origin=Origin(xyz=(0.0, 0.935, 0.019)),
        material=black_rubber,
        name="front_seal",
    )
    for i, x in enumerate((-0.455, 0.455)):
        glass.visual(
            Box((0.034, 0.94, 0.022)),
            origin=Origin(xyz=(x, 0.485, 0.019)),
            material=black_rubber,
            name=f"side_seal_{i}",
        )
    glass.visual(
        Cylinder(radius=0.011, length=0.100),
        origin=Origin(xyz=(-0.32, 0.000, 0.032), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminium,
        name="hinge_knuckle_0",
    )
    glass.visual(
        Cylinder(radius=0.011, length=0.100),
        origin=Origin(xyz=(0.32, 0.000, 0.032), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminium,
        name="hinge_knuckle_1",
    )

    slide_joint = model.articulation(
        "frame_to_carrier",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=slide_carrier,
        origin=Origin(xyz=(0.0, -0.33, 0.108)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.35, lower=0.0, upper=0.36),
    )
    slide_joint.meta["description"] = "glass carrier slides rearward in the twin guide rails"

    tilt_joint = model.articulation(
        "carrier_to_glass",
        ArticulationType.REVOLUTE,
        parent=slide_carrier,
        child=glass,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.9, lower=0.0, upper=0.24),
    )
    tilt_joint.meta["description"] = "rear-edge hinge tilts the glass before sliding"

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("cassette_frame")
    carrier = object_model.get_part("slide_carrier")
    glass = object_model.get_part("glass_panel")
    slide = object_model.get_articulation("frame_to_carrier")
    tilt = object_model.get_articulation("carrier_to_glass")

    ctx.allow_overlap(
        glass,
        carrier,
        elem_a="hinge_knuckle_0",
        elem_b="rear_hinge_pin",
        reason="The left rear glass hinge knuckle is intentionally modeled as a captured bushing around the hinge pin.",
    )
    ctx.allow_overlap(
        glass,
        carrier,
        elem_a="hinge_knuckle_1",
        elem_b="rear_hinge_pin",
        reason="The right rear glass hinge knuckle is intentionally modeled as a captured bushing around the hinge pin.",
    )

    ctx.check(
        "has independent tilt and slide joints",
        slide.articulation_type == ArticulationType.PRISMATIC
        and tilt.articulation_type == ArticulationType.REVOLUTE,
        details=f"slide={slide.articulation_type}, tilt={tilt.articulation_type}",
    )

    ctx.expect_within(
        carrier,
        frame,
        axes="x",
        inner_elem="side_beam_0",
        outer_elem="guide_bottom_0",
        margin=0.002,
        name="left carrier beam is captured by its guide channel",
    )
    ctx.expect_within(
        carrier,
        frame,
        axes="x",
        inner_elem="side_beam_1",
        outer_elem="guide_bottom_1",
        margin=0.002,
        name="right carrier beam is captured by its guide channel",
    )
    ctx.expect_overlap(
        carrier,
        frame,
        axes="y",
        elem_a="side_beam_0",
        elem_b="guide_bottom_0",
        min_overlap=0.65,
        name="closed carrier has retained rail engagement",
    )
    ctx.expect_overlap(
        glass,
        carrier,
        axes="x",
        elem_a="hinge_knuckle_0",
        elem_b="rear_hinge_pin",
        min_overlap=0.08,
        name="left hinge knuckle surrounds the pin along the hinge axis",
    )
    ctx.expect_gap(
        glass,
        carrier,
        axis="z",
        positive_elem="hinge_knuckle_0",
        negative_elem="rear_hinge_pin",
        max_penetration=0.021,
        name="left captured hinge-pin penetration is local and bounded",
    )
    ctx.expect_overlap(
        glass,
        carrier,
        axes="x",
        elem_a="hinge_knuckle_1",
        elem_b="rear_hinge_pin",
        min_overlap=0.08,
        name="right hinge knuckle surrounds the pin along the hinge axis",
    )
    ctx.expect_gap(
        glass,
        carrier,
        axis="z",
        positive_elem="hinge_knuckle_1",
        negative_elem="rear_hinge_pin",
        max_penetration=0.021,
        name="right captured hinge-pin penetration is local and bounded",
    )
    ctx.expect_overlap(
        glass,
        frame,
        axes="xy",
        elem_a="glass_sheet",
        elem_b="front_crossmember",
        min_overlap=0.04,
        name="closed glass spans the front of the cassette opening",
    )

    rest_carrier_pos = ctx.part_world_position(carrier)
    rest_front_aabb = ctx.part_element_world_aabb(glass, elem="front_seal")

    with ctx.pose({tilt: 0.24}):
        tilted_front_aabb = ctx.part_element_world_aabb(glass, elem="front_seal")

    ctx.check(
        "rear-edge tilt raises the front glass edge",
        rest_front_aabb is not None
        and tilted_front_aabb is not None
        and tilted_front_aabb[1][2] > rest_front_aabb[1][2] + 0.13,
        details=f"rest_front={rest_front_aabb}, tilted_front={tilted_front_aabb}",
    )

    with ctx.pose({slide: 0.36}):
        extended_carrier_pos = ctx.part_world_position(carrier)
        ctx.expect_overlap(
            carrier,
            frame,
            axes="y",
            elem_a="side_beam_0",
            elem_b="guide_bottom_0",
            min_overlap=0.65,
            name="slid carrier remains engaged in the rail",
        )
        ctx.expect_within(
            carrier,
            frame,
            axes="x",
            inner_elem="side_beam_1",
            outer_elem="guide_bottom_1",
            margin=0.002,
            name="slid carrier stays centered in the opposite rail",
        )

    ctx.check(
        "slide joint moves the glass carrier rearward",
        rest_carrier_pos is not None
        and extended_carrier_pos is not None
        and extended_carrier_pos[1] < rest_carrier_pos[1] - 0.34,
        details=f"rest={rest_carrier_pos}, extended={extended_carrier_pos}",
    )

    return ctx.report()


object_model = build_object_model()
