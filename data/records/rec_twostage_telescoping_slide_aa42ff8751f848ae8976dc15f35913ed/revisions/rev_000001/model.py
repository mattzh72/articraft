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
    model = ArticulatedObject(name="low_profile_drawer_slide")

    dark_zinc = model.material("dark_zinc", rgba=(0.22, 0.23, 0.24, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.62, 0.64, 0.63, 1.0))
    polished_race = model.material("polished_race", rgba=(0.82, 0.84, 0.80, 1.0))
    black_plate = model.material("black_plate", rgba=(0.04, 0.045, 0.05, 1.0))
    screw_dark = model.material("screw_dark", rgba=(0.11, 0.11, 0.12, 1.0))

    channel = model.part("channel")
    # Root frame is at the rear lower center of the grounded outer guide.
    channel.visual(
        Box((0.520, 0.065, 0.004)),
        origin=Origin(xyz=(0.260, 0.0, 0.002)),
        material=dark_zinc,
        name="bottom_web",
    )
    for y in (-0.0305, 0.0305):
        channel.visual(
            Box((0.520, 0.004, 0.031)),
            origin=Origin(xyz=(0.260, y, 0.0185)),
            material=dark_zinc,
            name=f"side_wall_{'neg' if y < 0 else 'pos'}",
        )
        channel.visual(
            Box((0.520, 0.014, 0.004)),
            origin=Origin(xyz=(0.260, 0.022 * (1 if y > 0 else -1), 0.0315)),
            material=dark_zinc,
            name=f"retaining_lip_{'neg' if y < 0 else 'pos'}",
        )
        channel.visual(
            Box((0.480, 0.002, 0.006)),
            origin=Origin(xyz=(0.260, 0.02775 * (1 if y > 0 else -1), 0.018)),
            material=polished_race,
            name=f"outer_race_{'neg' if y < 0 else 'pos'}",
        )

    middle_slide = model.part("middle_slide")
    # This member is a nested low C-section: side ribs and lips leave a central
    # slot for the inner slide while still reading as one telescoping rail.
    middle_slide.visual(
        Box((0.430, 0.036, 0.004)),
        origin=Origin(xyz=(0.215, 0.0, 0.006)),
        material=satin_steel,
        name="middle_bottom",
    )
    for y in (-0.018, 0.018):
        middle_slide.visual(
            Box((0.430, 0.004, 0.017)),
            origin=Origin(xyz=(0.215, y, 0.0165)),
            material=satin_steel,
            name=f"middle_side_{'neg' if y < 0 else 'pos'}",
        )
        middle_slide.visual(
            Box((0.430, 0.006, 0.0035)),
            origin=Origin(xyz=(0.215, 0.013 * (1 if y > 0 else -1), 0.0251)),
            material=satin_steel,
            name=f"middle_lip_{'neg' if y < 0 else 'pos'}",
        )
        middle_slide.visual(
            Box((0.400, 0.002, 0.006)),
            origin=Origin(xyz=(0.215, 0.0152 * (1 if y > 0 else -1), 0.017)),
            material=polished_race,
            name=f"middle_race_{'neg' if y < 0 else 'pos'}",
        )

    inner_slide = model.part("inner_slide")
    inner_slide.visual(
        Box((0.360, 0.012, 0.0115)),
        origin=Origin(xyz=(0.180, 0.0, 0.01375)),
        material=satin_steel,
        name="inner_tongue",
    )
    inner_slide.visual(
        Box((0.360, 0.012, 0.003)),
        origin=Origin(xyz=(0.180, 0.0, 0.0208)),
        material=polished_race,
        name="inner_top_race",
    )
    inner_slide.visual(
        Box((0.006, 0.016, 0.014)),
        origin=Origin(xyz=(0.357, 0.0, 0.017)),
        material=satin_steel,
        name="front_nose",
    )

    equipment_plate = model.part("equipment_plate")
    equipment_plate.visual(
        Box((0.012, 0.018, 0.015)),
        origin=Origin(xyz=(0.006, 0.0, 0.0295)),
        material=black_plate,
        name="end_tab",
    )
    equipment_plate.visual(
        Box((0.090, 0.078, 0.006)),
        origin=Origin(xyz=(0.052, 0.0, 0.0398)),
        material=black_plate,
        name="mount_plate",
    )
    for x in (0.030, 0.074):
        for y in (-0.023, 0.023):
            equipment_plate.visual(
                Cylinder(radius=0.0055, length=0.0026),
                origin=Origin(xyz=(x, y, 0.0440)),
                material=screw_dark,
                name=f"screw_{x:.3f}_{'neg' if y < 0 else 'pos'}",
            )

    model.articulation(
        "channel_to_middle",
        ArticulationType.PRISMATIC,
        parent=channel,
        child=middle_slide,
        origin=Origin(xyz=(0.045, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=65.0, velocity=0.35, lower=0.0, upper=0.180),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle_slide,
        child=inner_slide,
        origin=Origin(xyz=(0.035, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.35, lower=0.0, upper=0.160),
    )
    model.articulation(
        "inner_to_plate",
        ArticulationType.FIXED,
        parent=inner_slide,
        child=equipment_plate,
        origin=Origin(xyz=(0.360, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    channel = object_model.get_part("channel")
    middle = object_model.get_part("middle_slide")
    inner = object_model.get_part("inner_slide")
    plate = object_model.get_part("equipment_plate")
    channel_to_middle = object_model.get_articulation("channel_to_middle")
    middle_to_inner = object_model.get_articulation("middle_to_inner")

    ctx.expect_within(
        middle,
        channel,
        axes="yz",
        inner_elem="middle_bottom",
        outer_elem="bottom_web",
        margin=0.035,
        name="middle slide remains low and centered in the channel envelope",
    )
    ctx.expect_within(
        inner,
        middle,
        axes="yz",
        inner_elem="inner_tongue",
        outer_elem="middle_bottom",
        margin=0.025,
        name="inner slide remains nested in the middle envelope",
    )
    ctx.expect_contact(
        plate,
        inner,
        elem_a="end_tab",
        elem_b="front_nose",
        contact_tol=0.001,
        name="equipment plate tab is fixed to the inner slide nose",
    )
    ctx.expect_overlap(
        middle,
        channel,
        axes="x",
        elem_a="middle_bottom",
        elem_b="bottom_web",
        min_overlap=0.38,
        name="collapsed middle slide is deeply retained in the channel",
    )
    ctx.expect_overlap(
        inner,
        middle,
        axes="x",
        elem_a="inner_tongue",
        elem_b="middle_bottom",
        min_overlap=0.32,
        name="collapsed inner slide is deeply retained in the middle slide",
    )

    rest_plate_pos = ctx.part_world_position(plate)
    with ctx.pose({channel_to_middle: 0.180, middle_to_inner: 0.160}):
        ctx.expect_overlap(
            middle,
            channel,
            axes="x",
            elem_a="middle_bottom",
            elem_b="bottom_web",
            min_overlap=0.24,
            name="extended middle slide keeps retained insertion",
        )
        ctx.expect_overlap(
            inner,
            middle,
            axes="x",
            elem_a="inner_tongue",
            elem_b="middle_bottom",
            min_overlap=0.18,
            name="extended inner slide keeps retained insertion",
        )
        ctx.expect_within(
            inner,
            middle,
            axes="yz",
            inner_elem="inner_tongue",
            outer_elem="middle_bottom",
            margin=0.025,
            name="extended inner slide stays coaxial with the guide",
        )
        extended_plate_pos = ctx.part_world_position(plate)

    ctx.check(
        "serial prismatic extension moves the equipment plate forward",
        rest_plate_pos is not None
        and extended_plate_pos is not None
        and extended_plate_pos[0] > rest_plate_pos[0] + 0.30,
        details=f"rest={rest_plate_pos}, extended={extended_plate_pos}",
    )

    return ctx.report()


object_model = build_object_model()
