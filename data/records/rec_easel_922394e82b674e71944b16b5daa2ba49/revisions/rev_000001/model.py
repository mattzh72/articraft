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
    model = ArticulatedObject(name="tabletop_adjustable_easel")

    wood = model.material("warm_beech_wood", rgba=(0.72, 0.48, 0.25, 1.0))
    endgrain = model.material("darker_endgrain", rgba=(0.45, 0.27, 0.12, 1.0))
    dark = model.material("dark_recessed_slots", rgba=(0.035, 0.032, 0.028, 1.0))
    metal = model.material("brushed_steel", rgba=(0.62, 0.61, 0.56, 1.0))
    rubber = model.material("dark_rubber_feet", rgba=(0.025, 0.023, 0.020, 1.0))

    frame = model.part("frame")
    # Open front frame: two vertical stiles with real-looking metal slot tracks.
    frame.visual(
        Box((0.030, 0.026, 0.360)),
        origin=Origin(xyz=(-0.115, 0.0, 0.220)),
        material=wood,
        name="upright_0",
    )
    frame.visual(
        Box((0.010, 0.003, 0.250)),
        origin=Origin(xyz=(-0.115, 0.0145, 0.220)),
        material=dark,
        name="slot_track_0",
    )
    frame.visual(
        Box((0.090, 0.160, 0.040)),
        origin=Origin(xyz=(-0.115, 0.025, 0.020)),
        material=endgrain,
        name="desktop_foot_0",
    )
    frame.visual(
        Box((0.030, 0.026, 0.360)),
        origin=Origin(xyz=(0.115, 0.0, 0.220)),
        material=wood,
        name="upright_1",
    )
    frame.visual(
        Box((0.010, 0.003, 0.250)),
        origin=Origin(xyz=(0.115, 0.0145, 0.220)),
        material=dark,
        name="slot_track_1",
    )
    frame.visual(
        Box((0.090, 0.160, 0.040)),
        origin=Origin(xyz=(0.115, 0.025, 0.020)),
        material=endgrain,
        name="desktop_foot_1",
    )

    frame.visual(
        Box((0.305, 0.030, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        material=wood,
        name="lower_crossbar",
    )
    frame.visual(
        Box((0.285, 0.028, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.405)),
        material=wood,
        name="top_crossbar",
    )
    frame.visual(
        Box((0.205, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, 0.010, 0.340)),
        material=wood,
        name="upper_canvas_stop",
    )

    # Exposed hinge knuckles mounted behind the top rail.
    for suffix, x in (("0", -0.0725), ("1", 0.0725)):
        frame.visual(
            Box((0.050, 0.020, 0.045)),
            origin=Origin(xyz=(x, -0.021, 0.405)),
            material=metal,
            name=f"hinge_leaf_{suffix}",
        )
        frame.visual(
            Cylinder(radius=0.012, length=0.065),
            origin=Origin(xyz=(x, -0.034, 0.420), rpy=(0.0, pi / 2.0, 0.0)),
            material=metal,
            name=f"hinge_barrel_{suffix}",
        )

    rear_leg = model.part("rear_leg")
    rear_leg.visual(
        Cylinder(radius=0.0125, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=metal,
        name="hinge_barrel",
    )
    rear_leg.visual(
        Box((0.058, 0.034, 0.036)),
        origin=Origin(xyz=(0.0, -0.019, -0.018)),
        material=metal,
        name="hinge_yoke",
    )
    # The support leg is a single sloping wooden strut reaching to the table.
    rear_leg.visual(
        Box((0.034, 0.028, 0.380)),
        origin=Origin(xyz=(0.0, -0.094, -0.199), rpy=(2.700, 0.0, 0.0)),
        material=wood,
        name="leg_strut",
    )
    rear_leg.visual(
        Box((0.160, 0.052, 0.020)),
        origin=Origin(xyz=(0.0, -0.175, -0.370)),
        material=rubber,
        name="rear_foot",
    )

    ledge = model.part("ledge")
    ledge.visual(
        Box((0.340, 0.070, 0.022)),
        origin=Origin(xyz=(0.0, 0.065, -0.010)),
        material=wood,
        name="shelf_plank",
    )
    ledge.visual(
        Box((0.340, 0.018, 0.046)),
        origin=Origin(xyz=(0.0, 0.104, 0.010)),
        material=wood,
        name="front_lip",
    )
    ledge.visual(
        Box((0.300, 0.018, 0.060)),
        origin=Origin(xyz=(0.0, 0.028, 0.010)),
        material=wood,
        name="rear_backplate",
    )
    ledge.visual(
        Box((0.026, 0.014, 0.075)),
        origin=Origin(xyz=(-0.115, 0.023, 0.004)),
        material=metal,
        name="slider_shoe_0",
    )
    ledge.visual(
        Cylinder(radius=0.017, length=0.028),
        origin=Origin(xyz=(-0.115, 0.043, 0.004), rpy=(pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="thumb_knob_0",
    )
    ledge.visual(
        Box((0.026, 0.014, 0.075)),
        origin=Origin(xyz=(0.115, 0.023, 0.004)),
        material=metal,
        name="slider_shoe_1",
    )
    ledge.visual(
        Cylinder(radius=0.017, length=0.028),
        origin=Origin(xyz=(0.115, 0.043, 0.004), rpy=(pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="thumb_knob_1",
    )

    model.articulation(
        "rear_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=rear_leg,
        origin=Origin(xyz=(0.0, -0.034, 0.420)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=1.5, lower=0.0, upper=0.45),
    )

    model.articulation(
        "ledge_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=ledge,
        origin=Origin(xyz=(0.0, 0.0, 0.130)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.18, lower=0.0, upper=0.170),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    rear_leg = object_model.get_part("rear_leg")
    ledge = object_model.get_part("ledge")
    rear_hinge = object_model.get_articulation("rear_hinge")
    ledge_slide = object_model.get_articulation("ledge_slide")

    ctx.expect_overlap(
        ledge,
        frame,
        axes="xz",
        elem_a="slider_shoe_0",
        elem_b="slot_track_0",
        min_overlap=0.008,
        name="ledge shoe rides in left slot projection",
    )
    ctx.expect_overlap(
        ledge,
        frame,
        axes="xz",
        elem_a="slider_shoe_1",
        elem_b="slot_track_1",
        min_overlap=0.008,
        name="ledge shoe rides in right slot projection",
    )
    ctx.expect_gap(
        ledge,
        frame,
        axis="y",
        positive_elem="slider_shoe_0",
        negative_elem="slot_track_0",
        min_gap=0.0,
        max_gap=0.001,
        name="sliding shoe kisses the slot face",
    )

    rest_ledge_pos = ctx.part_world_position(ledge)
    with ctx.pose({ledge_slide: 0.170}):
        raised_ledge_pos = ctx.part_world_position(ledge)
        ctx.expect_overlap(
            ledge,
            frame,
            axes="xz",
            elem_a="slider_shoe_0",
            elem_b="slot_track_0",
            min_overlap=0.008,
            name="raised ledge remains captured in slot",
        )
    ctx.check(
        "ledge slide raises the shelf",
        rest_ledge_pos is not None
        and raised_ledge_pos is not None
        and raised_ledge_pos[2] > rest_ledge_pos[2] + 0.150,
        details=f"rest={rest_ledge_pos}, raised={raised_ledge_pos}",
    )

    deployed_aabb = ctx.part_element_world_aabb(rear_leg, elem="rear_foot")
    with ctx.pose({rear_hinge: 0.45}):
        folded_aabb = ctx.part_element_world_aabb(rear_leg, elem="rear_foot")
    deployed_y = None if deployed_aabb is None else (deployed_aabb[0][1] + deployed_aabb[1][1]) * 0.5
    folded_y = None if folded_aabb is None else (folded_aabb[0][1] + folded_aabb[1][1]) * 0.5
    ctx.check(
        "rear leg folds toward the frame",
        deployed_y is not None
        and folded_y is not None
        and folded_y > deployed_y + 0.12,
        details=f"deployed_y={deployed_y}, folded_y={folded_y}",
    )

    return ctx.report()


object_model = build_object_model()
