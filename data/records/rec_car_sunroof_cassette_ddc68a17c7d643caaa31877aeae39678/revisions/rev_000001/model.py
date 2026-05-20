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
    model = ArticulatedObject(name="pop_up_sunroof_cassette")

    satin_black = model.material("satin_black", rgba=(0.01, 0.011, 0.012, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.0, 0.0, 0.0, 1.0))
    dark_glass = model.material("smoked_glass", rgba=(0.04, 0.09, 0.12, 0.46))
    ceramic_frit = model.material("ceramic_frit", rgba=(0.0, 0.0, 0.0, 0.92))
    aluminum = model.material("brushed_aluminum", rgba=(0.55, 0.56, 0.54, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.90, 0.10, 0.030)),
        origin=Origin(xyz=(0.0, -0.270, 0.015)),
        material=satin_black,
        name="front_rail",
    )
    frame.visual(
        Box((0.90, 0.10, 0.030)),
        origin=Origin(xyz=(0.0, 0.290, 0.015)),
        material=satin_black,
        name="rear_rail",
    )
    for index, x in enumerate((-0.405, 0.405)):
        frame.visual(
            Box((0.090, 0.660, 0.030)),
            origin=Origin(xyz=(x, 0.010, 0.015)),
            material=satin_black,
            name=f"side_rail_{index}",
        )

    # Raised rubber seal around the roof aperture, seated into the flat frame.
    frame.visual(
        Box((0.720, 0.018, 0.014)),
        origin=Origin(xyz=(0.0, -0.220, 0.035)),
        material=rubber_black,
        name="front_seal",
    )
    frame.visual(
        Box((0.720, 0.018, 0.014)),
        origin=Origin(xyz=(0.0, 0.240, 0.035)),
        material=rubber_black,
        name="rear_seal",
    )
    frame.visual(
        Box((0.018, 0.460, 0.014)),
        origin=Origin(xyz=(-0.350, 0.010, 0.035)),
        material=rubber_black,
        name="side_seal_0",
    )
    frame.visual(
        Box((0.018, 0.460, 0.014)),
        origin=Origin(xyz=(0.350, 0.010, 0.035)),
        material=rubber_black,
        name="side_seal_1",
    )

    # Rear cassette guide tracks for the sliding section.
    frame.visual(
        Box((0.030, 0.560, 0.014)),
        origin=Origin(xyz=(-0.310, 0.200, 0.037)),
        material=aluminum,
        name="slide_track_0",
    )
    frame.visual(
        Box((0.030, 0.560, 0.014)),
        origin=Origin(xyz=(0.310, 0.200, 0.037)),
        material=aluminum,
        name="slide_track_1",
    )

    # Exposed front hinge knuckles mounted to short saddles on the front rail.
    hinge_segments = ((-0.310, 0.140), (0.0, 0.200), (0.310, 0.140))
    for index, (x, length) in enumerate(hinge_segments):
        frame.visual(
            Box((length, 0.018, 0.012)),
            origin=Origin(xyz=(x, -0.224, 0.034)),
            material=satin_black,
            name=f"hinge_saddle_{index}",
        )
        frame.visual(
            Cylinder(radius=0.012, length=length),
            origin=Origin(xyz=(x, -0.225, 0.050), rpy=(0.0, pi / 2.0, 0.0)),
            material=satin_black,
            name=f"frame_knuckle_{index}",
        )

    for index, (x, y) in enumerate(
        ((-0.405, -0.270), (0.405, -0.270), (-0.405, 0.290), (0.405, 0.290))
    ):
        frame.visual(
            Cylinder(radius=0.018, length=0.010),
            origin=Origin(xyz=(x, y, 0.033)),
            material=aluminum,
            name=f"screw_boss_{index}",
        )

    glass = model.part("glass_panel")
    glass.visual(
        Box((0.660, 0.410, 0.010)),
        origin=Origin(xyz=(0.0, 0.220, 0.004)),
        material=dark_glass,
        name="glass_panel",
    )
    glass.visual(
        Box((0.660, 0.030, 0.002)),
        origin=Origin(xyz=(0.0, 0.030, 0.010)),
        material=ceramic_frit,
        name="front_frit",
    )
    glass.visual(
        Box((0.660, 0.035, 0.002)),
        origin=Origin(xyz=(0.0, 0.408, 0.010)),
        material=ceramic_frit,
        name="rear_frit",
    )
    for index, x in enumerate((-0.315, 0.315)):
        glass.visual(
            Box((0.030, 0.380, 0.002)),
            origin=Origin(xyz=(x, 0.225, 0.010)),
            material=ceramic_frit,
            name=f"side_frit_{index}",
        )

    for index, x in enumerate((-0.170, 0.170)):
        glass.visual(
            Box((0.120, 0.070, 0.006)),
            origin=Origin(xyz=(x, 0.035, 0.002)),
            material=satin_black,
            name=f"hinge_leaf_{index}",
        )
        glass.visual(
            Cylinder(radius=0.012, length=0.120),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=satin_black,
            name=f"glass_knuckle_{index}",
        )

    model.articulation(
        "frame_to_glass",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=glass,
        origin=Origin(xyz=(0.0, -0.225, 0.050)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=24.0, velocity=1.5, lower=0.0, upper=0.58),
    )

    rear_slide = model.part("rear_slide")
    rear_slide.visual(
        Box((0.570, 0.240, 0.008)),
        origin=Origin(xyz=(0.0, 0.120, -0.006)),
        material=satin_black,
        name="rear_section",
    )
    rear_slide.visual(
        Box((0.024, 0.260, 0.010)),
        origin=Origin(xyz=(-0.310, 0.130, -0.006)),
        material=aluminum,
        name="side_runner_0",
    )
    rear_slide.visual(
        Box((0.024, 0.260, 0.010)),
        origin=Origin(xyz=(0.310, 0.130, -0.006)),
        material=aluminum,
        name="side_runner_1",
    )
    rear_slide.visual(
        Box((0.640, 0.025, 0.018)),
        origin=Origin(xyz=(0.0, 0.245, 0.002)),
        material=rubber_black,
        name="rear_lip",
    )

    model.articulation(
        "frame_to_rear_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=rear_slide,
        origin=Origin(xyz=(0.0, 0.255, 0.045)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.140),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    glass = object_model.get_part("glass_panel")
    rear_slide = object_model.get_part("rear_slide")
    glass_hinge = object_model.get_articulation("frame_to_glass")
    rear_track = object_model.get_articulation("frame_to_rear_slide")

    for index in (0, 1):
        ctx.allow_overlap(
            frame,
            rear_slide,
            elem_a=f"slide_track_{index}",
            elem_b=f"side_runner_{index}",
            reason=(
                "The rear sunroof slide runners are intentionally captured inside "
                "simplified solid guide-track proxies."
            ),
        )

    ctx.check(
        "front glass joint is revolute",
        glass_hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"joint_type={glass_hinge.articulation_type}",
    )
    ctx.check(
        "rear section joint is prismatic",
        rear_track.articulation_type == ArticulationType.PRISMATIC,
        details=f"joint_type={rear_track.articulation_type}",
    )

    ctx.expect_gap(
        glass,
        frame,
        axis="z",
        min_gap=0.003,
        positive_elem="glass_panel",
        negative_elem="side_seal_0",
        name="closed glass rides above side seal",
    )
    ctx.expect_gap(
        rear_slide,
        frame,
        axis="z",
        min_gap=0.003,
        positive_elem="rear_section",
        negative_elem="rear_rail",
        name="rear slide clears rear rail",
    )
    ctx.expect_overlap(
        rear_slide,
        frame,
        axes="y",
        min_overlap=0.15,
        elem_a="side_runner_0",
        elem_b="slide_track_0",
        name="rear slide is captured in track at rest",
    )
    ctx.expect_within(
        rear_slide,
        frame,
        axes="x",
        margin=0.001,
        inner_elem="side_runner_0",
        outer_elem="slide_track_0",
        name="rear slide runner is centered in track",
    )

    closed_glass_aabb = ctx.part_element_world_aabb(glass, elem="glass_panel")
    closed_slide_pos = ctx.part_world_position(rear_slide)
    with ctx.pose({glass_hinge: 0.58, rear_track: 0.140}):
        raised_glass_aabb = ctx.part_element_world_aabb(glass, elem="glass_panel")
        extended_slide_pos = ctx.part_world_position(rear_slide)
        ctx.expect_overlap(
            rear_slide,
            frame,
            axes="y",
            min_overlap=0.07,
            elem_a="side_runner_0",
            elem_b="slide_track_0",
            name="rear slide remains retained when extended",
        )

    ctx.check(
        "glass rear edge lifts upward",
        closed_glass_aabb is not None
        and raised_glass_aabb is not None
        and raised_glass_aabb[1][2] > closed_glass_aabb[1][2] + 0.12,
        details=f"closed={closed_glass_aabb}, raised={raised_glass_aabb}",
    )
    ctx.check(
        "rear section slides rearward",
        closed_slide_pos is not None
        and extended_slide_pos is not None
        and extended_slide_pos[1] > closed_slide_pos[1] + 0.12,
        details=f"closed={closed_slide_pos}, extended={extended_slide_pos}",
    )

    return ctx.report()


object_model = build_object_model()
