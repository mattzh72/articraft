from __future__ import annotations

import math

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
    model = ArticulatedObject(name="spoiler_sunroof_cassette")

    satin_aluminium = model.material("satin_aluminium", rgba=(0.72, 0.74, 0.74, 1.0))
    dark_anodized = model.material("dark_anodized_rail", rgba=(0.08, 0.09, 0.10, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.01, 0.012, 0.012, 1.0))
    smoked_glass = model.material("smoked_blue_glass", rgba=(0.10, 0.18, 0.24, 0.45))
    roller_steel = model.material("roller_steel", rgba=(0.52, 0.54, 0.56, 1.0))

    frame = model.part("cassette_frame")

    # A real cassette is much larger than the visible glass: the aluminium frame
    # carries front/rear cross rails plus wide side tracks around a roof opening.
    frame.visual(
        Box((0.30, 0.90, 0.035)),
        origin=Origin(xyz=(-0.600, 0.0, 0.0175)),
        material=satin_aluminium,
        name="front_rail",
    )
    frame.visual(
        Box((0.30, 0.90, 0.035)),
        origin=Origin(xyz=(0.600, 0.0, 0.0175)),
        material=satin_aluminium,
        name="rear_rail",
    )
    for i, y in enumerate((-0.395, 0.395)):
        frame.visual(
            Box((1.35, 0.11, 0.035)),
            origin=Origin(xyz=(0.0, y, 0.0175)),
            material=satin_aluminium,
            name=f"side_rail_{i}",
        )

    # Twin guide rails sit proud of the side frame and form shallow channels for
    # the rolling shoes.  They are long enough to retain the shoes at full slide.
    rail_len = 1.22
    guide_base_names = ("guide_base_0", "guide_base_1")
    guide_lip_names = (("guide_lip_0_0", "guide_lip_0_1"), ("guide_lip_1_0", "guide_lip_1_1"))
    for i, sign in enumerate((-1.0, 1.0)):
        y = sign * 0.355
        frame.visual(
            Box((rail_len, 0.055, 0.015)),
            origin=Origin(xyz=(0.050, y, 0.0425)),
            material=dark_anodized,
            name=guide_base_names[i],
        )
        for j, lip_y in enumerate((sign * 0.325, sign * 0.385)):
            frame.visual(
                Box((rail_len, 0.006, 0.012)),
                origin=Origin(xyz=(0.050, lip_y, 0.056)),
                material=dark_anodized,
                name=guide_lip_names[i][j],
            )

    # Low-profile black fastener heads make the aluminium cassette read as a
    # bolted vehicle roof module without cutting structural holes in the frame.
    screw_locations = [
        (-0.60, -0.34),
        (-0.60, 0.34),
        (0.60, -0.34),
        (0.60, 0.34),
        (-0.18, -0.415),
        (0.18, -0.415),
        (-0.18, 0.415),
        (0.18, 0.415),
    ]
    for i, (x, y) in enumerate(screw_locations):
        frame.visual(
            Cylinder(radius=0.018, length=0.004),
            origin=Origin(xyz=(x, y, 0.037)),
            material=black_rubber,
            name=f"fastener_{i}",
        )

    carriage = model.part("slide_carriage")

    # The carriage frame origin lies on the front hinge axis in the closed
    # position.  Rear rolling shoes ride in both rails and a transverse bridge
    # keeps the two shoes tied together, as on a cassette lift/slide mechanism.
    rear_shoe_names = ("rear_shoe_0", "rear_shoe_1")
    roller_names = (("roller_0_0", "roller_0_1"), ("roller_1_0", "roller_1_1"))
    side_link_names = ("side_link_0", "side_link_1")
    for i, sign in enumerate((-1.0, 1.0)):
        y = sign * 0.355
        carriage.visual(
            Box((0.120, 0.045, 0.014)),
            origin=Origin(xyz=(0.840, y, -0.033)),
            material=dark_anodized,
            name=rear_shoe_names[i],
        )
        for j, x in enumerate((0.790, 0.890)):
            carriage.visual(
                Cylinder(radius=0.010, length=0.035),
                origin=Origin(xyz=(x, y, -0.036), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=roller_steel,
                name=roller_names[i][j],
            )
        carriage.visual(
            Box((0.860, 0.018, 0.006)),
            origin=Origin(xyz=(0.430, sign * 0.310, -0.030)),
            material=dark_anodized,
            name=side_link_names[i],
        )

    carriage.visual(
        Box((0.045, 0.740, 0.006)),
        origin=Origin(xyz=(0.835, 0.0, -0.030)),
        material=dark_anodized,
        name="rear_bridge",
    )

    for i, sign in enumerate((-1.0, 1.0)):
        carriage.visual(
            Box((0.030, 0.030, 0.034)),
            origin=Origin(xyz=(0.010, sign * 0.300, -0.023)),
            material=dark_anodized,
            name=f"front_upright_{i}",
        )

    # Alternating hinge barrels along the width expose the horizontal pivot at
    # the front rail while leaving a central barrel for the glass-side leaf.
    for i, y in enumerate((-0.235, 0.235)):
        carriage.visual(
            Cylinder(radius=0.014, length=0.150),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=satin_aluminium,
            name=f"hinge_barrel_{i}",
        )

    glass = model.part("glass_panel")

    glass_shape = (
        cq.Workplane("XY")
        .box(0.920, 0.620, 0.010)
        .edges("|Z")
        .fillet(0.035)
        .edges(">Z or <Z")
        .fillet(0.002)
    )
    glass.visual(
        mesh_from_cadquery(glass_shape, "smoked_glass_lite", tolerance=0.001),
        origin=Origin(xyz=(0.495, 0.0, 0.0)),
        material=smoked_glass,
        name="glass_lite",
    )

    # Black ceramic frit and rubber edge treatment are separate visuals so they
    # remain visible through/around the translucent glass.
    glass.visual(
        Box((0.920, 0.040, 0.003)),
        origin=Origin(xyz=(0.495, 0.310, 0.0065)),
        material=black_rubber,
        name="side_frit_0",
    )
    glass.visual(
        Box((0.920, 0.040, 0.003)),
        origin=Origin(xyz=(0.495, -0.310, 0.0065)),
        material=black_rubber,
        name="side_frit_1",
    )
    glass.visual(
        Box((0.055, 0.620, 0.003)),
        origin=Origin(xyz=(0.065, 0.0, 0.0065)),
        material=black_rubber,
        name="front_frit",
    )
    glass.visual(
        Box((0.055, 0.620, 0.003)),
        origin=Origin(xyz=(0.925, 0.0, 0.0065)),
        material=black_rubber,
        name="rear_frit",
    )

    glass.visual(
        Cylinder(radius=0.014, length=0.220),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_rubber,
        name="hinge_knuckle",
    )
    glass.visual(
        Box((0.105, 0.180, 0.006)),
        origin=Origin(xyz=(0.045, 0.0, -0.007)),
        material=black_rubber,
        name="front_hinge_strap",
    )

    # Rear corner lift brackets drop from the glass edge onto the shoe pads in
    # the closed position; as the hinge opens, these brackets lift with the
    # glass, making the spoiler action legible.
    rear_lift_bracket_names = ("rear_lift_bracket_0", "rear_lift_bracket_1")
    for i, sign in enumerate((-1.0, 1.0)):
        glass.visual(
            Box((0.070, 0.100, 0.021)),
            origin=Origin(xyz=(0.865, sign * 0.315, -0.0155)),
            material=black_rubber,
            name=rear_lift_bracket_names[i],
        )

    slide = model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(-0.500, 0.0, 0.096)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.22, lower=0.0, upper=0.240),
    )
    model.articulation(
        "carriage_to_glass",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=glass,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.0, lower=0.0, upper=0.285),
        meta={"coordinated_with": slide.name, "spoiler_angle_per_slide_m": 1.18},
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("cassette_frame")
    carriage = object_model.get_part("slide_carriage")
    glass = object_model.get_part("glass_panel")
    slide = object_model.get_articulation("frame_to_carriage")
    hinge = object_model.get_articulation("carriage_to_glass")

    ctx.check(
        "tilt is documented as coordinated with rearward slide",
        hinge.meta.get("coordinated_with") == slide.name
        and hinge.meta.get("spoiler_angle_per_slide_m", 0.0) > 1.0,
        details=f"hinge meta={hinge.meta}",
    )
    ctx.check(
        "carriage slides rearward along cassette rails",
        slide.axis == (1.0, 0.0, 0.0)
        and slide.motion_limits is not None
        and slide.motion_limits.upper is not None
        and slide.motion_limits.upper >= 0.20,
        details=f"axis={slide.axis}, limits={slide.motion_limits}",
    )

    with ctx.pose({slide: 0.0, hinge: 0.0}):
        ctx.expect_within(
            glass,
            frame,
            axes="xy",
            inner_elem="glass_lite",
            margin=0.0,
            name="closed glass fits within aluminium cassette footprint",
        )
        ctx.expect_gap(
            glass,
            frame,
            axis="z",
            min_gap=0.004,
            max_gap=0.050,
            positive_elem="rear_lift_bracket_0",
            negative_elem="guide_lip_0_0",
            name="rear lift bracket clears guide rail lip",
        )
        ctx.expect_gap(
            carriage,
            frame,
            axis="z",
            min_gap=-0.001,
            max_gap=0.002,
            positive_elem="roller_0_0",
            negative_elem="guide_base_0",
            name="rolling shoe rests on guide rail",
        )

    rest_aabb = ctx.part_element_world_aabb(glass, elem="glass_lite")
    upper = slide.motion_limits.upper if slide.motion_limits and slide.motion_limits.upper else 0.24
    hinge_upper = hinge.motion_limits.upper if hinge.motion_limits and hinge.motion_limits.upper else 0.285
    with ctx.pose({slide: upper, hinge: hinge_upper}):
        extended_aabb = ctx.part_element_world_aabb(glass, elem="glass_lite")
        ctx.expect_gap(
            glass,
            frame,
            axis="z",
            min_gap=0.015,
            name="open spoiler glass clears cassette rails",
        )
        ctx.expect_overlap(
            carriage,
            frame,
            axes="x",
            min_overlap=0.020,
            elem_a="roller_1_1",
            elem_b="guide_base_1",
            name="rear roller remains captured in guide at full slide",
        )

    ctx.check(
        "glass panel slides rearward and spoilers upward",
        rest_aabb is not None
        and extended_aabb is not None
        and extended_aabb[1][0] > rest_aabb[1][0] + 0.12
        and extended_aabb[1][2] > rest_aabb[1][2] + 0.15,
        details=f"rest_aabb={rest_aabb}, extended_aabb={extended_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
