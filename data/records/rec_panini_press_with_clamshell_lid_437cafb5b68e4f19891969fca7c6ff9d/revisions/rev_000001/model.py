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
    model = ArticulatedObject(name="family_panini_grill")

    body_dark = model.material("body_dark", rgba=(0.16, 0.17, 0.18, 1.0))
    plate_black = model.material("plate_black", rgba=(0.10, 0.10, 0.11, 1.0))
    trim_black = model.material("trim_black", rgba=(0.06, 0.06, 0.07, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.08, 0.08, 0.08, 1.0))
    accent_grey = model.material("accent_grey", rgba=(0.46, 0.47, 0.48, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.34, 0.43, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=body_dark,
        name="base_shell",
    )
    for index, spine_y in enumerate((-0.118, 0.118)):
        base.visual(
            Box((0.014, 0.090, 0.022)),
            origin=Origin(xyz=(-0.163, spine_y, 0.071)),
            material=body_dark,
            name=f"hinge_pedestal_{index}",
        )
    base.visual(
        Box((0.295, 0.370, 0.012)),
        origin=Origin(xyz=(0.005, 0.0, 0.066)),
        material=plate_black,
        name="lower_plate",
    )
    for index, rib_y in enumerate((-0.145, -0.087, -0.029, 0.029, 0.087, 0.145)):
        base.visual(
            Box((0.260, 0.010, 0.003)),
            origin=Origin(xyz=(0.005, rib_y, 0.0735)),
            material=accent_grey,
            name=f"lower_rib_{index}",
        )
    base.visual(
        Box((0.010, 0.070, 0.014)),
        origin=Origin(xyz=(0.174, 0.0, 0.067)),
        material=trim_black,
        name="latch_keeper",
    )
    for index, barrel_y in enumerate((-0.118, 0.118)):
        base.visual(
            Cylinder(radius=0.011, length=0.082),
            origin=Origin(
                xyz=(-0.155, barrel_y, 0.082),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=trim_black,
            name=f"hinge_barrel_{index}",
        )
    for index, (foot_x, foot_y) in enumerate(
        ((-0.110, -0.155), (-0.110, 0.155), (0.112, -0.155), (0.112, 0.155))
    ):
        base.visual(
            Box((0.050, 0.060, 0.008)),
            origin=Origin(xyz=(foot_x, foot_y, 0.004)),
            material=rubber_black,
            name=f"foot_{index}",
        )
    base.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(
            xyz=(0.105, 0.218, 0.050),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=trim_black,
        name="knob_collar",
    )

    lid = model.part("lid")
    lid.visual(
        Box((0.310, 0.390, 0.036)),
        origin=Origin(xyz=(0.175, 0.0, 0.020)),
        material=body_dark,
        name="lid_cover",
    )
    for index, side_y in enumerate((-0.185, 0.185)):
        lid.visual(
            Box((0.310, 0.020, 0.024)),
            origin=Origin(xyz=(0.175, side_y, 0.008)),
            material=body_dark,
            name=f"side_skirt_{index}",
        )
    lid.visual(
        Box((0.024, 0.360, 0.032)),
        origin=Origin(xyz=(0.318, 0.0, 0.008)),
        material=body_dark,
        name="front_lip",
    )
    lid.visual(
        Box((0.022, 0.140, 0.026)),
        origin=Origin(xyz=(0.011, 0.0, 0.001)),
        material=body_dark,
        name="hinge_block",
    )
    lid.visual(
        Box((0.286, 0.350, 0.008)),
        origin=Origin(xyz=(0.163, 0.0, -0.004)),
        material=plate_black,
        name="upper_plate",
    )
    for index, rib_y in enumerate((-0.135, -0.081, -0.027, 0.027, 0.081, 0.135)):
        lid.visual(
            Box((0.255, 0.010, 0.003)),
            origin=Origin(xyz=(0.163, rib_y, 0.0015)),
            material=accent_grey,
            name=f"upper_rib_{index}",
        )
    lid.visual(
        Cylinder(radius=0.010, length=0.140),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=trim_black,
        name="hinge_barrel",
    )
    lid.visual(
        Box((0.020, 0.150, 0.010)),
        origin=Origin(xyz=(0.319, 0.0, 0.028)),
        material=trim_black,
        name="front_grip",
    )

    latch_tab = model.part("latch_tab")
    latch_tab.visual(
        Cylinder(radius=0.004, length=0.060),
        origin=Origin(
            xyz=(0.003, 0.0, -0.004),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=trim_black,
        name="tab_knuckle",
    )
    latch_tab.visual(
        Box((0.010, 0.085, 0.030)),
        origin=Origin(xyz=(0.009, 0.0, -0.016)),
        material=trim_black,
        name="tab_body",
    )
    latch_tab.visual(
        Box((0.014, 0.050, 0.008)),
        origin=Origin(xyz=(0.010, 0.0, -0.024)),
        material=trim_black,
        name="tab_hook",
    )

    thermostat_knob = model.part("thermostat_knob")
    thermostat_knob.visual(
        Cylinder(radius=0.006, length=0.014),
        origin=Origin(
            xyz=(0.0, 0.007, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=trim_black,
        name="knob_shaft",
    )
    thermostat_knob.visual(
        Cylinder(radius=0.020, length=0.018),
        origin=Origin(
            xyz=(0.0, 0.023, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=trim_black,
        name="knob_body",
    )
    thermostat_knob.visual(
        Box((0.003, 0.008, 0.013)),
        origin=Origin(xyz=(0.0, 0.032, 0.012)),
        material=accent_grey,
        name="knob_pointer",
    )

    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(-0.155, 0.0, 0.082)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.2,
            lower=0.0,
            upper=1.75,
        ),
    )
    model.articulation(
        "lid_to_latch_tab",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=latch_tab,
        origin=Origin(xyz=(0.331, 0.0, 0.024)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.5,
            lower=0.0,
            upper=0.95,
        ),
    )
    model.articulation(
        "base_to_thermostat_knob",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=thermostat_knob,
        origin=Origin(xyz=(0.105, 0.221, 0.050)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=6.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    latch_tab = object_model.get_part("latch_tab")
    thermostat_knob = object_model.get_part("thermostat_knob")

    lid_hinge = object_model.get_articulation("base_to_lid")
    latch_hinge = object_model.get_articulation("lid_to_latch_tab")
    knob_joint = object_model.get_articulation("base_to_thermostat_knob")

    with ctx.pose({lid_hinge: 0.0, latch_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            positive_elem="upper_plate",
            negative_elem="lower_plate",
            min_gap=0.001,
            max_gap=0.004,
            name="upper plate sits just above lower plate when closed",
        )
        ctx.expect_overlap(
            lid,
            base,
            axes="xy",
            elem_a="upper_plate",
            elem_b="lower_plate",
            min_overlap=0.28,
            name="upper and lower plates keep a broad cooking footprint",
        )
        ctx.expect_gap(
            latch_tab,
            base,
            axis="x",
            positive_elem="tab_hook",
            negative_elem="latch_keeper",
            min_gap=0.0,
            max_gap=0.004,
            name="closed latch hook sits just ahead of the keeper",
        )
        ctx.expect_gap(
            thermostat_knob,
            base,
            axis="y",
            positive_elem="knob_shaft",
            negative_elem="knob_collar",
            min_gap=0.0,
            max_gap=0.001,
            name="thermostat knob rides on the side collar without floating away",
        )

    closed_front = ctx.part_element_world_aabb(lid, elem="front_grip")
    with ctx.pose({lid_hinge: 1.55}):
        open_front = ctx.part_element_world_aabb(lid, elem="front_grip")
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            positive_elem="front_grip",
            negative_elem="base_shell",
            min_gap=0.150,
            name="open lid lifts the front grip well above the base",
        )

    ctx.check(
        "lid front rises when opened",
        closed_front is not None
        and open_front is not None
        and open_front[0][2] > closed_front[0][2] + 0.15,
        details=f"closed={closed_front}, open={open_front}",
    )

    closed_hook = ctx.part_element_world_aabb(latch_tab, elem="tab_hook")
    with ctx.pose({latch_hinge: 0.70}):
        open_hook = ctx.part_element_world_aabb(latch_tab, elem="tab_hook")

    ctx.check(
        "latch tab swings downward to release",
        closed_hook is not None
        and open_hook is not None
        and open_hook[0][2] < closed_hook[0][2] - 0.003
        and abs(open_hook[0][0] - closed_hook[0][0]) > 0.010,
        details=f"closed={closed_hook}, open={open_hook}",
    )

    ctx.check(
        "thermostat uses continuous rotation",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={knob_joint.articulation_type}",
    )

    return ctx.report()


object_model = build_object_model()
