from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mounted_folding_arm")

    powder = model.material("graphite_powder_coat", rgba=(0.10, 0.11, 0.12, 1.0))
    dark = model.material("matte_black_slider", rgba=(0.015, 0.016, 0.017, 1.0))
    steel = model.material("brushed_steel", rgba=(0.58, 0.58, 0.54, 1.0))
    wall_paint = model.material("painted_wall_plate", rgba=(0.73, 0.75, 0.76, 1.0))
    rubber = model.material("black_rubber_pads", rgba=(0.025, 0.023, 0.020, 1.0))
    zinc = model.material("zinc_fasteners", rgba=(0.72, 0.70, 0.64, 1.0))

    wall = model.part("wall_bracket")
    wall.visual(
        Box((0.026, 0.320, 0.340)),
        origin=Origin(xyz=(-0.092, 0.0, 0.0)),
        material=wall_paint,
        name="wall_plate",
    )
    wall.visual(
        Box((0.030, 0.150, 0.118)),
        origin=Origin(xyz=(-0.070, 0.0, 0.0)),
        material=powder,
        name="rear_bridge",
    )
    wall.visual(
        Box((0.150, 0.150, 0.012)),
        origin=Origin(xyz=(0.010, 0.0, 0.054)),
        material=powder,
        name="upper_cheek",
    )
    wall.visual(
        Box((0.150, 0.150, 0.012)),
        origin=Origin(xyz=(0.010, 0.0, -0.054)),
        material=powder,
        name="lower_cheek",
    )
    for y in (-0.082, 0.082):
        wall.visual(
            Box((0.030, 0.014, 0.118)),
            origin=Origin(xyz=(-0.070, y, 0.0)),
            material=powder,
            name=f"side_web_{0 if y < 0 else 1}",
        )
    for y, z, idx in ((-0.108, -0.115, 0), (0.108, -0.115, 1), (-0.108, 0.115, 2), (0.108, 0.115, 3)):
        wall.visual(
            Cylinder(radius=0.014, length=0.008),
            origin=Origin(xyz=(-0.075, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=zinc,
            name=f"bolt_head_{idx}",
        )
    wall.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.068)),
        material=steel,
        name="root_pin_cap",
    )
    wall.visual(
        Cylinder(radius=0.014, length=0.132),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=steel,
        name="root_pin",
    )
    wall.visual(
        Box((0.034, 0.026, 0.018)),
        origin=Origin(xyz=(0.046, 0.064, 0.067)),
        material=steel,
        name="root_stop_tab",
    )

    first = model.part("first_link")
    for z, suffix in ((0.032, "upper"), (-0.032, "lower")):
        first.visual(
            Box((0.360, 0.088, 0.010)),
            origin=Origin(xyz=(0.220, 0.0, z)),
            material=powder,
            name=f"{suffix}_strap",
        )
    first.visual(
        Cylinder(radius=0.042, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=powder,
        name="upper_root_boss",
    )
    first.visual(
        Cylinder(radius=0.042, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.032)),
        material=powder,
        name="lower_root_boss",
    )
    first.visual(
        Cylinder(radius=0.044, length=0.010),
        origin=Origin(xyz=(0.420, 0.0, 0.032)),
        material=powder,
        name="upper_elbow_boss",
    )
    first.visual(
        Cylinder(radius=0.044, length=0.010),
        origin=Origin(xyz=(0.420, 0.0, -0.032)),
        material=powder,
        name="lower_elbow_boss",
    )
    for y, suffix in ((-0.047, "near"), (0.047, "far")):
        first.visual(
            Box((0.300, 0.006, 0.008)),
            origin=Origin(xyz=(0.205, y, 0.041)),
            material=powder,
            name=f"{suffix}_formed_edge",
        )
    for x, idx in ((0.130, 0), (0.285, 1)):
        first.visual(
            Box((0.018, 0.072, 0.070)),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=powder,
            name=f"box_spacer_{idx}",
        )
    for x, y, idx in ((0.0, -0.035, 0),):
        first.visual(
            Box((0.022, 0.010, 0.070)),
            origin=Origin(xyz=(x, y, 0.0)),
            material=steel,
            name=f"pin_sleeve_{idx}",
        )
    first.visual(
        Cylinder(radius=0.014, length=0.080),
        origin=Origin(xyz=(0.420, 0.0, 0.0)),
        material=steel,
        name="elbow_pin",
    )
    first.visual(
        Box((0.030, 0.020, 0.014)),
        origin=Origin(xyz=(0.063, -0.054, 0.039)),
        material=steel,
        name="root_stop_lug",
    )

    distal = model.part("distal_link")
    distal.visual(
        Cylinder(radius=0.026, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=powder,
        name="elbow_tongue_boss",
    )
    distal.visual(
        Box((0.086, 0.042, 0.018)),
        origin=Origin(xyz=(0.064, 0.0, 0.0)),
        material=powder,
        name="elbow_tongue",
    )
    distal.visual(
        Box((0.056, 0.154, 0.018)),
        origin=Origin(xyz=(0.076, 0.066, 0.0)),
        material=powder,
        name="offset_bridge",
    )
    distal.visual(
        Box((0.365, 0.070, 0.014)),
        origin=Origin(xyz=(0.235, 0.120, 0.0)),
        material=powder,
        name="slide_web",
    )
    for y, suffix in ((0.080, "inner"), (0.160, "outer")):
        distal.visual(
            Box((0.365, 0.010, 0.044)),
            origin=Origin(xyz=(0.235, y, 0.022)),
            material=powder,
            name=f"{suffix}_formed_rail",
        )
        distal.visual(
            Box((0.245, 0.012, 0.012)),
            origin=Origin(xyz=(0.285, y, 0.049)),
            material=powder,
            name=f"{suffix}_keeper_strip",
        )
    distal.visual(
        Box((0.020, 0.020, 0.055)),
        origin=Origin(xyz=(0.414, 0.175, 0.025)),
        material=steel,
        name="end_stop",
    )
    distal.visual(
        Box((0.034, 0.018, 0.018)),
        origin=Origin(xyz=(0.098, 0.040, 0.018)),
        material=steel,
        name="elbow_stop_tab",
    )

    nose = model.part("nose_carriage")
    nose.visual(
        Box((0.290, 0.052, 0.018)),
        origin=Origin(xyz=(-0.105, 0.0, 0.028)),
        material=dark,
        name="inner_slide",
    )
    for y, suffix in ((-0.030, "inner"), (0.030, "outer")):
        nose.visual(
            Box((0.180, 0.010, 0.014)),
            origin=Origin(xyz=(-0.080, y, 0.029)),
            material=rubber,
            name=f"{suffix}_slider_pad",
        )
    nose.visual(
        Box((0.130, 0.090, 0.060)),
        origin=Origin(xyz=(0.092, 0.0, 0.030)),
        material=dark,
        name="nose_body",
    )
    nose.visual(
        Box((0.012, 0.108, 0.070)),
        origin=Origin(xyz=(0.163, 0.0, 0.033)),
        material=steel,
        name="front_plate",
    )
    nose.visual(
        Box((0.075, 0.030, 0.014)),
        origin=Origin(xyz=(0.100, 0.0, -0.007)),
        material=rubber,
        name="underside_pad",
    )

    model.articulation(
        "root_pivot",
        ArticulationType.REVOLUTE,
        parent=wall,
        child=first,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.2, lower=0.0, upper=1.55),
    )
    model.articulation(
        "elbow_pivot",
        ArticulationType.REVOLUTE,
        parent=first,
        child=distal,
        origin=Origin(xyz=(0.420, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=70.0, velocity=1.4, lower=-2.62, upper=0.22),
    )
    model.articulation(
        "nose_slide",
        ArticulationType.PRISMATIC,
        parent=distal,
        child=nose,
        origin=Origin(xyz=(0.400, 0.120, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.25, lower=0.0, upper=0.200),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall = object_model.get_part("wall_bracket")
    first = object_model.get_part("first_link")
    distal = object_model.get_part("distal_link")
    nose = object_model.get_part("nose_carriage")
    root = object_model.get_articulation("root_pivot")
    elbow = object_model.get_articulation("elbow_pivot")
    slide = object_model.get_articulation("nose_slide")

    ctx.allow_overlap(
        wall,
        first,
        elem_a="root_pin",
        elem_b="upper_root_boss",
        reason="The fixed root pin is intentionally captured inside the upper pivot boss bore proxy.",
    )
    ctx.allow_overlap(
        wall,
        first,
        elem_a="root_pin",
        elem_b="lower_root_boss",
        reason="The fixed root pin is intentionally captured inside the lower pivot boss bore proxy.",
    )
    ctx.allow_overlap(
        first,
        distal,
        elem_a="elbow_pin",
        elem_b="elbow_tongue_boss",
        reason="The elbow pin is intentionally captured through the distal tongue boss bore proxy.",
    )

    ctx.check(
        "root elbow and nose mechanisms are authored",
        len(object_model.articulations) == 3,
        details=f"found {len(object_model.articulations)} articulations",
    )
    ctx.expect_overlap(
        wall,
        first,
        axes="xyz",
        elem_a="root_pin",
        elem_b="upper_root_boss",
        min_overlap=0.008,
        name="upper root boss surrounds the fixed pin",
    )
    ctx.expect_overlap(
        wall,
        first,
        axes="xyz",
        elem_a="root_pin",
        elem_b="lower_root_boss",
        min_overlap=0.008,
        name="lower root boss surrounds the fixed pin",
    )
    ctx.expect_overlap(
        first,
        distal,
        axes="xyz",
        elem_a="elbow_pin",
        elem_b="elbow_tongue_boss",
        min_overlap=0.020,
        name="elbow tongue boss surrounds the hinge pin",
    )
    ctx.expect_overlap(
        first,
        wall,
        axes="xy",
        elem_a="upper_root_boss",
        elem_b="upper_cheek",
        min_overlap=0.030,
        name="root boss sits inside the wall clevis footprint",
    )
    ctx.expect_gap(
        wall,
        first,
        axis="z",
        positive_elem="upper_cheek",
        negative_elem="upper_root_boss",
        min_gap=0.006,
        name="upper root cheek clears the moving strap",
    )
    ctx.expect_gap(
        first,
        wall,
        axis="z",
        positive_elem="lower_root_boss",
        negative_elem="lower_cheek",
        min_gap=0.006,
        name="lower root cheek clears the moving strap",
    )
    ctx.expect_overlap(
        distal,
        first,
        axes="xy",
        elem_a="elbow_tongue_boss",
        elem_b="upper_elbow_boss",
        min_overlap=0.024,
        name="elbow tongue is captured between first-link cheeks",
    )
    ctx.expect_gap(
        first,
        distal,
        axis="z",
        positive_elem="upper_elbow_boss",
        negative_elem="elbow_tongue_boss",
        min_gap=0.006,
        name="upper elbow cheek clears the distal tongue",
    )
    ctx.expect_gap(
        distal,
        first,
        axis="z",
        positive_elem="elbow_tongue_boss",
        negative_elem="lower_elbow_boss",
        min_gap=0.006,
        name="lower elbow cheek clears the distal tongue",
    )
    ctx.expect_within(
        nose,
        distal,
        axes="yz",
        inner_elem="inner_slide",
        margin=0.004,
        name="nose slide stays inside the distal guide envelope",
    )
    ctx.expect_overlap(
        nose,
        distal,
        axes="x",
        elem_a="inner_slide",
        elem_b="slide_web",
        min_overlap=0.180,
        name="retracted carriage keeps a long rail engagement",
    )

    retracted = ctx.part_world_position(nose)
    with ctx.pose({slide: 0.200}):
        ctx.expect_overlap(
            nose,
            distal,
            axes="x",
            elem_a="inner_slide",
            elem_b="slide_web",
            min_overlap=0.040,
            name="extended carriage still retains rail engagement",
        )
        extended = ctx.part_world_position(nose)
    ctx.check(
        "nose carriage extends only along the distal link",
        retracted is not None and extended is not None and extended[0] > retracted[0] + 0.190,
        details=f"retracted={retracted}, extended={extended}",
    )

    with ctx.pose({root: 1.25, elbow: -2.50, slide: 0.160}):
        ctx.expect_origin_distance(
            first,
            distal,
            axes="xy",
            min_dist=0.38,
            max_dist=0.46,
            name="folded elbow keeps the second link on its pinned radius",
        )

    return ctx.report()


object_model = build_object_model()
