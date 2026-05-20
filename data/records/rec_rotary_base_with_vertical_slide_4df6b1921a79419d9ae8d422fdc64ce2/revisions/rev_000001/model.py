from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="wall_backed_rotary_column_z_slide")

    painted_steel = model.material("painted_steel", rgba=(0.58, 0.62, 0.64, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    bearing_black = model.material("bearing_black", rgba=(0.015, 0.017, 0.020, 1.0))
    guide_chrome = model.material("guide_chrome", rgba=(0.78, 0.80, 0.76, 1.0))
    rotary_blue = model.material("rotary_blue", rgba=(0.08, 0.20, 0.38, 1.0))
    carriage_orange = model.material("carriage_orange", rgba=(0.95, 0.42, 0.12, 1.0))
    label_white = model.material("label_white", rgba=(0.92, 0.93, 0.88, 1.0))

    backplate = model.part("backplate")
    backplate.visual(
        Box((0.50, 0.050, 1.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.525)),
        material=painted_steel,
        name="wall_plate",
    )
    # Raised rim ribs make the wall plate read as a rigid bolted fabrication.
    for x, name in ((-0.232, "rim_0"), (0.232, "rim_1")):
        backplate.visual(
            Box((0.035, 0.026, 0.98)),
            origin=Origin(xyz=(x, 0.035, 0.525)),
            material=dark_steel,
            name=name,
        )
    for z, name in ((0.075, "rim_2"), (0.975, "rim_3")):
        backplate.visual(
            Box((0.46, 0.026, 0.035)),
            origin=Origin(xyz=(0.0, 0.035, z)),
            material=dark_steel,
            name=name,
        )
    # Four screw heads are slightly seated into the raised face, not floating.
    for i, (x, z) in enumerate(((-0.18, 0.16), (0.18, 0.16), (-0.18, 0.89), (0.18, 0.89))):
        backplate.visual(
            Cylinder(radius=0.022, length=0.012),
            origin=Origin(xyz=(x, 0.030, z), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=bearing_black,
            name=f"screw_{i}",
        )

    # A fixed cantilever shelf and side webs carry the rotary bearing out from the wall.
    backplate.visual(
        Box((0.30, 0.23, 0.050)),
        origin=Origin(xyz=(0.0, 0.135, 0.205)),
        material=dark_steel,
        name="support_shelf",
    )
    for x, name in ((-0.135, "web_0"), (0.135, "web_1")):
        backplate.visual(
            Box((0.030, 0.18, 0.29)),
            origin=Origin(xyz=(x, 0.110, 0.345)),
            material=dark_steel,
            name=name,
        )
    backplate.visual(
        Cylinder(radius=0.118, length=0.035),
        origin=Origin(xyz=(0.0, 0.200, 0.2475)),
        material=bearing_black,
        name="fixed_bearing",
    )

    rotary_stage = model.part("rotary_stage")
    rotary_stage.visual(
        Cylinder(radius=0.105, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=rotary_blue,
        name="rotating_disk",
    )
    rotary_stage.visual(
        Cylinder(radius=0.048, length=0.085),
        origin=Origin(xyz=(0.0, 0.0, 0.0825)),
        material=dark_steel,
        name="hub",
    )
    rotary_stage.visual(
        Box((0.18, 0.070, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.137)),
        material=rotary_blue,
        name="guide_foot",
    )
    for x, name in ((-0.055, "guide_rail_0"), (0.055, "guide_rail_1")):
        rotary_stage.visual(
            Cylinder(radius=0.011, length=0.590),
            origin=Origin(xyz=(x, 0.0, 0.455)),
            material=guide_chrome,
            name=name,
        )
    rotary_stage.visual(
        Cylinder(radius=0.006, length=0.590),
        origin=Origin(xyz=(0.0, -0.027, 0.455)),
        material=dark_steel,
        name="center_leadscrew",
    )
    rotary_stage.visual(
        Box((0.17, 0.070, 0.042)),
        origin=Origin(xyz=(0.0, 0.0, 0.762)),
        material=rotary_blue,
        name="top_bridge",
    )
    rotary_stage.visual(
        Box((0.025, 0.032, 0.62)),
        origin=Origin(xyz=(0.0, -0.032, 0.455)),
        material=dark_steel,
        name="rear_spine",
    )
    # Thin white scale strip on the spine emphasizes that this upright guide is a Z slide.
    rotary_stage.visual(
        Box((0.010, 0.006, 0.46)),
        origin=Origin(xyz=(0.0, -0.051, 0.455)),
        material=label_white,
        name="scale_strip",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.200, 0.035, 0.120)),
        origin=Origin(xyz=(0.0, 0.050, 0.0)),
        material=carriage_orange,
        name="front_plate",
    )
    # The shoe blocks just kiss the outside of the two guide rails at q=0 so the
    # carriage has a physical support path while still reading as a sliding fit.
    for x, name in ((-0.080, "guide_shoe_0"), (0.080, "guide_shoe_1")):
        carriage.visual(
            Box((0.028, 0.050, 0.140)),
            origin=Origin(xyz=(x, 0.017, 0.0)),
            material=carriage_orange,
            name=name,
        )
    carriage.visual(
        Box((0.110, 0.025, 0.070)),
        origin=Origin(xyz=(0.0, 0.080, 0.0)),
        material=dark_steel,
        name="tool_face",
    )
    for i, (x, z) in enumerate(((-0.052, -0.037), (0.052, -0.037), (-0.052, 0.037), (0.052, 0.037))):
        carriage.visual(
            Cylinder(radius=0.012, length=0.012),
            origin=Origin(xyz=(x, 0.069, z), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=bearing_black,
            name=f"roller_{i}",
        )

    model.articulation(
        "backplate_to_rotary_stage",
        ArticulationType.REVOLUTE,
        parent=backplate,
        child=rotary_stage,
        origin=Origin(xyz=(0.0, 0.200, 0.265)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=70.0, velocity=1.2, lower=-2.1, upper=2.1),
    )
    model.articulation(
        "rotary_stage_to_carriage",
        ArticulationType.PRISMATIC,
        parent=rotary_stage,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.240)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.30, lower=0.0, upper=0.320),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    backplate = object_model.get_part("backplate")
    rotary_stage = object_model.get_part("rotary_stage")
    carriage = object_model.get_part("carriage")
    rotary_joint = object_model.get_articulation("backplate_to_rotary_stage")
    slide_joint = object_model.get_articulation("rotary_stage_to_carriage")

    ctx.check(
        "base stage is a vertical revolute joint",
        rotary_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(rotary_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={rotary_joint.articulation_type}, axis={rotary_joint.axis}",
    )
    ctx.check(
        "carriage is a vertical prismatic joint",
        slide_joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(slide_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={slide_joint.articulation_type}, axis={slide_joint.axis}",
    )

    ctx.expect_contact(
        rotary_stage,
        backplate,
        elem_a="rotating_disk",
        elem_b="fixed_bearing",
        contact_tol=1e-5,
        name="rotary disk is seated on the fixed wall bearing",
    )
    ctx.expect_gap(
        carriage,
        rotary_stage,
        axis="y",
        positive_elem="front_plate",
        negative_elem="guide_rail_0",
        min_gap=0.015,
        max_gap=0.025,
        name="carriage face clears the left guide rail",
    )
    ctx.expect_gap(
        carriage,
        rotary_stage,
        axis="y",
        positive_elem="front_plate",
        negative_elem="guide_rail_1",
        min_gap=0.015,
        max_gap=0.025,
        name="carriage face clears the right guide rail",
    )
    ctx.expect_overlap(
        carriage,
        rotary_stage,
        axes="z",
        elem_a="guide_shoe_0",
        elem_b="guide_rail_0",
        min_overlap=0.12,
        name="left shoe remains engaged on the vertical guide",
    )
    ctx.expect_overlap(
        carriage,
        rotary_stage,
        axes="z",
        elem_a="guide_shoe_1",
        elem_b="guide_rail_1",
        min_overlap=0.12,
        name="right shoe remains engaged on the vertical guide",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide_joint: 0.320}):
        ctx.expect_overlap(
            carriage,
            rotary_stage,
            axes="z",
            elem_a="guide_shoe_0",
            elem_b="guide_rail_0",
            min_overlap=0.12,
            name="extended left shoe remains engaged",
        )
        ctx.expect_overlap(
            carriage,
            rotary_stage,
            axes="z",
            elem_a="guide_shoe_1",
            elem_b="guide_rail_1",
            min_overlap=0.12,
            name="extended right shoe remains engaged",
        )
        extended_pos = ctx.part_world_position(carriage)

    ctx.check(
        "Z slide moves carriage upward",
        rest_pos is not None and extended_pos is not None and extended_pos[2] > rest_pos[2] + 0.30,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
