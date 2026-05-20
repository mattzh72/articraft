from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pivoting_inspection_arm")

    dark = model.material("dark_powdercoat", rgba=(0.05, 0.055, 0.06, 1.0))
    graphite = model.material("graphite_anodized_aluminum", rgba=(0.18, 0.20, 0.21, 1.0))
    machined = model.material("brushed_steel", rgba=(0.63, 0.64, 0.62, 1.0))
    black = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    glass = model.material("smoked_glass_lens", rgba=(0.03, 0.06, 0.09, 0.82))
    amber = model.material("warm_led_diffuser", rgba=(1.0, 0.72, 0.32, 1.0))

    # Root: a bolted table/wall mounting plate carrying the fixed half of the
    # vertical base hinge.  Dimensions are in real inspection-arm scale.
    base = model.part("base")
    base.visual(
        Box((0.28, 0.20, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=dark,
        name="mount_plate",
    )
    for i, (x, y) in enumerate(
        ((-0.105, -0.065), (-0.105, 0.065), (0.105, -0.065), (0.105, 0.065))
    ):
        base.visual(
            Cylinder(radius=0.014, length=0.006),
            origin=Origin(xyz=(x, y, 0.027)),
            material=machined,
            name=f"screw_head_{i}",
        )
    base.visual(
        Cylinder(radius=0.045, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=dark,
        name="fixed_post",
    )
    base.visual(
        Cylinder(radius=0.056, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.119)),
        material=machined,
        name="bearing_seat",
    )
    base.visual(
        Cylinder(radius=0.029, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.116)),
        material=black,
        name="dust_seal",
    )

    # Revolute child: the rotating hinge collar and a rigid outer telescoping
    # sleeve.  The sleeve is built as a rectangular tube from four overlapping
    # walls so the following prismatic stage can slide through a real opening.
    boom = model.part("boom")
    boom.visual(
        Cylinder(radius=0.042, length=0.058),
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        material=graphite,
        name="rotating_collar",
    )
    boom.visual(
        Cylinder(radius=0.026, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.063)),
        material=machined,
        name="hinge_cap",
    )
    for i, y in enumerate((-0.035, 0.035)):
        boom.visual(
            Box((0.105, 0.014, 0.055)),
            origin=Origin(xyz=(0.066, y, 0.055)),
            material=graphite,
            name=f"hinge_cheek_{i}",
        )
    boom.visual(
        Box((0.480, 0.086, 0.012)),
        origin=Origin(xyz=(0.315, 0.0, 0.080)),
        material=graphite,
        name="outer_top_wall",
    )
    boom.visual(
        Box((0.480, 0.086, 0.012)),
        origin=Origin(xyz=(0.315, 0.0, 0.030)),
        material=graphite,
        name="outer_bottom_wall",
    )
    for i, y in enumerate((-0.036, 0.036)):
        boom.visual(
            Box((0.480, 0.014, 0.060)),
            origin=Origin(xyz=(0.315, y, 0.055)),
            material=graphite,
            name=f"outer_side_wall_{i}",
        )
    boom.visual(
        Box((0.018, 0.086, 0.060)),
        origin=Origin(xyz=(0.078, 0.0, 0.055)),
        material=graphite,
        name="rear_sleeve_band",
    )
    boom.visual(
        Box((0.018, 0.086, 0.012)),
        origin=Origin(xyz=(0.550, 0.0, 0.080)),
        material=machined,
        name="front_band_top",
    )
    boom.visual(
        Box((0.018, 0.086, 0.012)),
        origin=Origin(xyz=(0.550, 0.0, 0.030)),
        material=machined,
        name="front_band_bottom",
    )
    for i, y in enumerate((-0.036, 0.036)):
        boom.visual(
            Box((0.018, 0.014, 0.060)),
            origin=Origin(xyz=(0.550, y, 0.055)),
            material=machined,
            name=f"front_band_side_{i}",
        )

    # Prismatic child: one sliding rectangular stage with a fixed inspection
    # camera/light head at the end.  The rear of the tube remains inside the
    # sleeve even at the upper travel limit.
    slider = model.part("slider")
    slider.visual(
        Box((0.520, 0.058, 0.038)),
        origin=Origin(xyz=(-0.100, 0.0, 0.055)),
        material=machined,
        name="inner_tube",
    )
    slider.visual(
        Box((0.045, 0.058, 0.040)),
        origin=Origin(xyz=(0.158, 0.0, 0.055)),
        material=machined,
        name="end_collar",
    )
    slider.visual(
        Box((0.065, 0.070, 0.058)),
        origin=Origin(xyz=(0.205, 0.0, 0.055)),
        material=black,
        name="camera_housing",
    )
    slider.visual(
        Cylinder(radius=0.019, length=0.008),
        origin=Origin(xyz=(0.241, 0.0, 0.055), rpy=(0.0, pi / 2.0, 0.0)),
        material=glass,
        name="front_lens",
    )
    slider.visual(
        Box((0.010, 0.020, 0.040)),
        origin=Origin(xyz=(0.240, -0.027, 0.055)),
        material=amber,
        name="led_strip_0",
    )
    slider.visual(
        Box((0.010, 0.020, 0.040)),
        origin=Origin(xyz=(0.240, 0.027, 0.055)),
        material=amber,
        name="led_strip_1",
    )
    slider.visual(
        Sphere(radius=0.010),
        origin=Origin(xyz=(0.205, 0.0, 0.091)),
        material=black,
        name="cable_grommet",
    )

    model.articulation(
        "base_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=boom,
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.2, lower=-1.45, upper=1.45),
    )
    model.articulation(
        "extension_slide",
        ArticulationType.PRISMATIC,
        parent=boom,
        child=slider,
        origin=Origin(xyz=(0.540, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.250),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    boom = object_model.get_part("boom")
    slider = object_model.get_part("slider")
    base_hinge = object_model.get_articulation("base_hinge")
    extension_slide = object_model.get_articulation("extension_slide")

    ctx.expect_contact(
        "base",
        boom,
        elem_a="bearing_seat",
        elem_b="rotating_collar",
        contact_tol=0.001,
        name="rotating collar sits on the base bearing",
    )
    ctx.expect_within(
        slider,
        boom,
        axes="yz",
        inner_elem="inner_tube",
        margin=0.0,
        name="inner tube is centered inside the outer sleeve cross section",
    )
    ctx.expect_overlap(
        slider,
        boom,
        axes="x",
        elem_a="inner_tube",
        elem_b="outer_top_wall",
        min_overlap=0.30,
        name="collapsed slider remains deeply inserted in sleeve",
    )

    rest_pos = ctx.part_world_position(slider)
    with ctx.pose({extension_slide: 0.250}):
        ctx.expect_within(
            slider,
            boom,
            axes="yz",
            inner_elem="inner_tube",
            margin=0.0,
            name="extended slider stays centered in sleeve cross section",
        )
        ctx.expect_overlap(
            slider,
            boom,
            axes="x",
            elem_a="inner_tube",
            elem_b="outer_top_wall",
            min_overlap=0.08,
            name="extended slider retains insertion in sleeve",
        )
        extended_pos = ctx.part_world_position(slider)

    ctx.check(
        "extension slide moves the inspection head outward",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.20,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    rest_tip = ctx.part_world_position(slider)
    with ctx.pose({base_hinge: 1.0}):
        pivoted_tip = ctx.part_world_position(slider)
    ctx.check(
        "base hinge pivots the whole arm around the vertical post",
        rest_tip is not None
        and pivoted_tip is not None
        and abs(pivoted_tip[1] - rest_tip[1]) > 0.40,
        details=f"rest={rest_tip}, pivoted={pivoted_tip}",
    )

    return ctx.report()


object_model = build_object_model()
