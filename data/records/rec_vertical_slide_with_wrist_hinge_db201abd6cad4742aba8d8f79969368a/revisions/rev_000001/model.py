from __future__ import annotations

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
    model = ArticulatedObject(name="side_wall_vertical_axis_wrist_tab")

    painted_steel = model.material("painted_steel", rgba=(0.18, 0.21, 0.23, 1.0))
    rail_steel = model.material("rail_steel", rgba=(0.38, 0.40, 0.42, 1.0))
    dark_slider = model.material("dark_slider", rgba=(0.08, 0.09, 0.10, 1.0))
    bearing_face = model.material("bearing_face", rgba=(0.58, 0.60, 0.62, 1.0))
    safety_orange = model.material("safety_orange", rgba=(0.95, 0.38, 0.08, 1.0))
    rubber = model.material("rubber", rgba=(0.02, 0.02, 0.018, 1.0))

    side_plate = model.part("side_plate")
    side_plate.visual(
        Box((0.180, 0.018, 0.500)),
        origin=Origin(xyz=(0.0, 0.0, 0.259)),
        material=painted_steel,
        name="back_plate",
    )
    side_plate.visual(
        Box((0.220, 0.120, 0.018)),
        origin=Origin(xyz=(0.0, 0.045, 0.009)),
        material=painted_steel,
        name="base_foot",
    )
    side_plate.visual(
        Box((0.020, 0.020, 0.420)),
        origin=Origin(xyz=(-0.060, 0.019, 0.260)),
        material=rail_steel,
        name="guide_rail_0",
    )
    side_plate.visual(
        Box((0.020, 0.020, 0.420)),
        origin=Origin(xyz=(0.060, 0.019, 0.260)),
        material=rail_steel,
        name="guide_rail_1",
    )
    side_plate.visual(
        Box((0.145, 0.022, 0.020)),
        origin=Origin(xyz=(0.0, 0.020, 0.040)),
        material=rail_steel,
        name="lower_stop",
    )
    side_plate.visual(
        Box((0.145, 0.022, 0.020)),
        origin=Origin(xyz=(0.0, 0.020, 0.480)),
        material=rail_steel,
        name="upper_stop",
    )
    for index, (x, z) in enumerate(
        ((-0.075, 0.105), (0.075, 0.105), (-0.075, 0.415), (0.075, 0.415))
    ):
        side_plate.visual(
            Cylinder(radius=0.008, length=0.006),
            origin=Origin(xyz=(x, 0.0115, z), rpy=(1.57079632679, 0.0, 0.0)),
            material=rail_steel,
            name=f"screw_head_{index}",
        )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.075, 0.030, 0.105)),
        origin=Origin(),
        material=dark_slider,
        name="slider_block",
    )
    carriage.visual(
        Box((0.061, 0.008, 0.084)),
        origin=Origin(xyz=(0.0, 0.017, 0.0)),
        material=bearing_face,
        name="front_bearing_plate",
    )
    carriage.visual(
        Box((0.036, 0.056, 0.034)),
        origin=Origin(xyz=(0.0, 0.029, -0.036)),
        material=dark_slider,
        name="lower_hinge_web",
    )
    carriage.visual(
        Box((0.036, 0.056, 0.034)),
        origin=Origin(xyz=(0.0, 0.029, 0.036)),
        material=dark_slider,
        name="upper_hinge_web",
    )
    carriage.visual(
        Cylinder(radius=0.011, length=0.033),
        origin=Origin(xyz=(0.0, 0.043, -0.036)),
        material=rail_steel,
        name="lower_knuckle",
    )
    carriage.visual(
        Cylinder(radius=0.011, length=0.033),
        origin=Origin(xyz=(0.0, 0.043, 0.036)),
        material=rail_steel,
        name="upper_knuckle",
    )
    carriage.visual(
        Cylinder(radius=0.0042, length=0.116),
        origin=Origin(xyz=(0.0, 0.043, 0.0)),
        material=bearing_face,
        name="hinge_pin",
    )

    wrist_tab = model.part("wrist_tab")
    wrist_tab.visual(
        Cylinder(radius=0.0095, length=0.030),
        origin=Origin(),
        material=rail_steel,
        name="center_knuckle",
    )
    wrist_tab.visual(
        Box((0.055, 0.075, 0.016)),
        origin=Origin(xyz=(0.0, 0.043, 0.0)),
        material=safety_orange,
        name="tab_plate",
    )
    wrist_tab.visual(
        Cylinder(radius=0.0275, length=0.016),
        origin=Origin(xyz=(0.0, 0.079, 0.0)),
        material=safety_orange,
        name="rounded_tip",
    )
    wrist_tab.visual(
        Box((0.038, 0.038, 0.004)),
        origin=Origin(xyz=(0.0, 0.061, 0.0095)),
        material=rubber,
        name="thumb_pad",
    )

    model.articulation(
        "plate_to_carriage",
        ArticulationType.PRISMATIC,
        parent=side_plate,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.024, 0.140)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.18, lower=0.0, upper=0.220),
    )
    model.articulation(
        "carriage_to_tab",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=wrist_tab,
        origin=Origin(xyz=(0.0, 0.043, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=-1.20, upper=1.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    side_plate = object_model.get_part("side_plate")
    carriage = object_model.get_part("carriage")
    wrist_tab = object_model.get_part("wrist_tab")
    carriage_slide = object_model.get_articulation("plate_to_carriage")
    tab_hinge = object_model.get_articulation("carriage_to_tab")

    ctx.allow_overlap(
        carriage,
        wrist_tab,
        elem_a="hinge_pin",
        elem_b="center_knuckle",
        reason="The wrist tab's center barrel is intentionally captured around the vertical hinge pin.",
    )

    ctx.check(
        "carriage joint is vertical prismatic",
        carriage_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(carriage_slide.axis) == (0.0, 0.0, 1.0),
        details=f"type={carriage_slide.articulation_type}, axis={carriage_slide.axis}",
    )
    ctx.check(
        "tab joint is vertical revolute",
        tab_hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(tab_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"type={tab_hinge.articulation_type}, axis={tab_hinge.axis}",
    )

    with ctx.pose({carriage_slide: 0.0, tab_hinge: 0.0}):
        ctx.expect_gap(
            carriage,
            side_plate,
            axis="y",
            positive_elem="slider_block",
            negative_elem="back_plate",
            min_gap=0.0,
            max_gap=0.001,
            name="slider block bears on side plate",
        )
        ctx.expect_overlap(
            carriage,
            side_plate,
            axes="z",
            elem_a="slider_block",
            elem_b="guide_rail_0",
            min_overlap=0.080,
            name="carriage overlaps guide rail height at low travel",
        )
        ctx.expect_gap(
            wrist_tab,
            carriage,
            axis="y",
            positive_elem="tab_plate",
            negative_elem="slider_block",
            min_gap=0.025,
            name="tab plate clears carriage face",
        )
        ctx.expect_within(
            carriage,
            wrist_tab,
            axes="xy",
            inner_elem="hinge_pin",
            outer_elem="center_knuckle",
            margin=0.001,
            name="hinge pin is centered inside tab knuckle",
        )
        ctx.expect_overlap(
            carriage,
            wrist_tab,
            axes="z",
            elem_a="hinge_pin",
            elem_b="center_knuckle",
            min_overlap=0.028,
            name="hinge pin passes through tab knuckle",
        )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({carriage_slide: 0.220}):
        ctx.expect_gap(
            carriage,
            side_plate,
            axis="y",
            positive_elem="slider_block",
            negative_elem="back_plate",
            min_gap=0.0,
            max_gap=0.001,
            name="raised carriage remains on side plate",
        )
        ctx.expect_overlap(
            carriage,
            side_plate,
            axes="z",
            elem_a="slider_block",
            elem_b="guide_rail_0",
            min_overlap=0.080,
            name="carriage overlaps guide rail height at high travel",
        )
        raised_pos = ctx.part_world_position(carriage)
    ctx.check(
        "carriage translates upward",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.20,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    with ctx.pose({tab_hinge: 1.0}):
        ctx.expect_gap(
            wrist_tab,
            carriage,
            axis="y",
            positive_elem="tab_plate",
            negative_elem="slider_block",
            min_gap=0.005,
            name="swung tab clears carriage face",
        )

    return ctx.report()


object_model = build_object_model()
