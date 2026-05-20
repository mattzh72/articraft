from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="compact_console_monitor_arm")

    matte_black = model.material("matte_black", rgba=(0.015, 0.016, 0.018, 1.0))
    dark_graphite = model.material("dark_graphite", rgba=(0.07, 0.075, 0.08, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.45, 0.47, 0.48, 1.0))
    rubber = model.material("rubber", rgba=(0.005, 0.005, 0.006, 1.0))

    base = model.part("base_bracket")
    base.visual(
        Box((0.220, 0.130, 0.018)),
        origin=Origin(xyz=(0.025, 0.0, 0.009)),
        material=matte_black,
        name="floor_plate",
    )
    base.visual(
        Box((0.185, 0.095, 0.004)),
        origin=Origin(xyz=(0.025, 0.0, 0.020)),
        material=rubber,
        name="top_pad",
    )
    base.visual(
        Cylinder(radius=0.030, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
        material=dark_graphite,
        name="pedestal",
    )
    base.visual(
        Box((0.055, 0.076, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.076)),
        material=dark_graphite,
        name="clevis_bridge",
    )
    for side, y in (("cheek_0", 0.032), ("cheek_1", -0.032)):
        base.visual(
            Box((0.046, 0.012, 0.070)),
            origin=Origin(xyz=(0.0, y, 0.116)),
            material=dark_graphite,
            name=side,
        )
    for name, y, roll in (("pivot_cap_0", 0.040, pi / 2), ("pivot_cap_1", -0.040, pi / 2)):
        base.visual(
            Cylinder(radius=0.017, length=0.004),
            origin=Origin(xyz=(0.0, y, 0.116), rpy=(roll, 0.0, 0.0)),
            material=satin_metal,
            name=name,
        )
    for name, x, y in (
        ("foot_0", -0.070, -0.045),
        ("foot_1", -0.070, 0.045),
        ("foot_2", 0.115, -0.045),
        ("foot_3", 0.115, 0.045),
    ):
        base.visual(
            Cylinder(radius=0.014, length=0.004),
            origin=Origin(xyz=(x, y, 0.002)),
            material=rubber,
            name=name,
        )

    lower = model.part("lower_link")
    lower.visual(
        Cylinder(radius=0.020, length=0.040),
        origin=Origin(rpy=(pi / 2, 0.0, 0.0)),
        material=satin_metal,
        name="base_lug",
    )
    for name, y in (("rail_0", 0.022), ("rail_1", -0.022)):
        lower.visual(
            Box((0.164, 0.008, 0.014)),
            origin=Origin(xyz=(0.082, y, 0.0)),
            material=matte_black,
            name=name,
        )
    for name, y in (("elbow_cheek_0", 0.027), ("elbow_cheek_1", -0.027)):
        lower.visual(
            Cylinder(radius=0.020, length=0.014),
            origin=Origin(xyz=(0.180, y, 0.0), rpy=(pi / 2, 0.0, 0.0)),
            material=satin_metal,
            name=name,
        )
    upper = model.part("upper_link")
    upper.visual(
        Cylinder(radius=0.018, length=0.040),
        origin=Origin(rpy=(pi / 2, 0.0, 0.0)),
        material=satin_metal,
        name="elbow_lug",
    )
    upper.visual(
        Box((0.150, 0.026, 0.016)),
        origin=Origin(xyz=(-0.075, 0.0, 0.0)),
        material=matte_black,
        name="spine",
    )
    upper.visual(
        Box((0.035, 0.032, 0.030)),
        origin=Origin(xyz=(-0.130, 0.0, 0.022)),
        material=dark_graphite,
        name="swivel_riser",
    )
    upper.visual(
        Cylinder(radius=0.024, length=0.006),
        origin=Origin(xyz=(-0.130, 0.0, 0.038)),
        material=satin_metal,
        name="turntable",
    )

    swivel = model.part("swivel_hub")
    swivel.visual(
        Cylinder(radius=0.022, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=satin_metal,
        name="swivel_disk",
    )
    swivel.visual(
        Cylinder(radius=0.012, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=dark_graphite,
        name="swivel_post",
    )
    swivel.visual(
        Box((0.040, 0.030, 0.014)),
        origin=Origin(xyz=(-0.020, 0.0, 0.034)),
        material=dark_graphite,
        name="post_bridge",
    )
    for name, y in (("tilt_arm_0", 0.019), ("tilt_arm_1", -0.019)):
        swivel.visual(
            Box((0.046, 0.010, 0.014)),
            origin=Origin(xyz=(-0.033, y, 0.034)),
            material=dark_graphite,
            name=name,
        )
    for name, y in (("tilt_cheek_0", 0.025), ("tilt_cheek_1", -0.025)):
        swivel.visual(
            Box((0.018, 0.006, 0.046)),
            origin=Origin(xyz=(-0.055, y, 0.034)),
            material=dark_graphite,
            name=name,
        )
    for name, y in (("tilt_cap_0", 0.030), ("tilt_cap_1", -0.030)):
        swivel.visual(
            Cylinder(radius=0.011, length=0.004),
            origin=Origin(xyz=(-0.055, y, 0.034), rpy=(pi / 2, 0.0, 0.0)),
            material=satin_metal,
            name=name,
        )

    head = model.part("head_frame")
    head.visual(
        Cylinder(radius=0.014, length=0.028),
        origin=Origin(rpy=(pi / 2, 0.0, 0.0)),
        material=satin_metal,
        name="tilt_barrel",
    )
    head.visual(
        Box((0.030, 0.020, 0.024)),
        origin=Origin(xyz=(-0.017, 0.0, 0.0)),
        material=dark_graphite,
        name="center_boss",
    )
    head.visual(
        Box((0.008, 0.100, 0.008)),
        origin=Origin(xyz=(-0.036, 0.0, 0.040)),
        material=matte_black,
        name="top_bar",
    )
    head.visual(
        Box((0.008, 0.100, 0.008)),
        origin=Origin(xyz=(-0.036, 0.0, -0.040)),
        material=matte_black,
        name="bottom_bar",
    )
    head.visual(
        Box((0.008, 0.008, 0.088)),
        origin=Origin(xyz=(-0.036, 0.050, 0.0)),
        material=matte_black,
        name="side_bar_0",
    )
    head.visual(
        Box((0.008, 0.008, 0.088)),
        origin=Origin(xyz=(-0.036, -0.050, 0.0)),
        material=matte_black,
        name="side_bar_1",
    )
    head.visual(
        Box((0.008, 0.106, 0.006)),
        origin=Origin(xyz=(-0.034, 0.0, 0.0)),
        material=dark_graphite,
        name="cross_bar_y",
    )
    head.visual(
        Box((0.008, 0.006, 0.086)),
        origin=Origin(xyz=(-0.034, 0.0, 0.0)),
        material=dark_graphite,
        name="cross_bar_z",
    )
    for name, y, z in (
        ("screw_spoke_0", 0.033, 0.013),
        ("screw_spoke_1", -0.033, 0.013),
        ("screw_spoke_2", 0.033, -0.013),
        ("screw_spoke_3", -0.033, -0.013),
    ):
        head.visual(
            Box((0.006, 0.006, 0.030)),
            origin=Origin(xyz=(-0.038, y, z)),
            material=dark_graphite,
            name=name,
        )
    for name, y, z in (
        ("screw_pad_0", 0.033, 0.026),
        ("screw_pad_1", -0.033, 0.026),
        ("screw_pad_2", 0.033, -0.026),
        ("screw_pad_3", -0.033, -0.026),
    ):
        head.visual(
            Cylinder(radius=0.006, length=0.003),
            origin=Origin(xyz=(-0.041, y, z), rpy=(0.0, pi / 2, 0.0)),
            material=satin_metal,
            name=name,
        )

    model.articulation(
        "base_joint",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower,
        origin=Origin(xyz=(0.0, 0.0, 0.116)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=-0.55, upper=1.15),
    )
    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=upper,
        origin=Origin(xyz=(0.180, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=1.5, lower=-0.35, upper=2.15),
    )
    model.articulation(
        "head_swivel",
        ArticulationType.REVOLUTE,
        parent=upper,
        child=swivel,
        origin=Origin(xyz=(-0.130, 0.0, 0.041)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=-1.40, upper=1.40),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=swivel,
        child=head,
        origin=Origin(xyz=(-0.055, 0.0, 0.034)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.5, lower=-0.75, upper=0.75),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base_bracket")
    upper = object_model.get_part("upper_link")
    swivel = object_model.get_part("swivel_hub")
    head = object_model.get_part("head_frame")
    fold_base = object_model.get_articulation("base_joint")
    fold_elbow = object_model.get_articulation("elbow_joint")
    head_swivel = object_model.get_articulation("head_swivel")
    head_tilt = object_model.get_articulation("head_tilt")

    ctx.check(
        "arm uses four revolute axes",
        all(
            joint.articulation_type == ArticulationType.REVOLUTE
            for joint in (fold_base, fold_elbow, head_swivel, head_tilt)
        ),
    )

    ctx.expect_gap(
        swivel,
        upper,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="swivel_disk",
        negative_elem="turntable",
        name="swivel disk seats on upper link turntable",
    )
    ctx.expect_overlap(
        head,
        base,
        axes="x",
        min_overlap=0.010,
        name="folded head remains close over the base bracket",
    )

    folded_head = ctx.part_world_position(head)
    with ctx.pose({fold_base: 0.80, fold_elbow: 1.10}):
        opened_head = ctx.part_world_position(head)
    ctx.check(
        "two links unfold the head upward",
        folded_head is not None
        and opened_head is not None
        and opened_head[2] > folded_head[2] + 0.12,
        details=f"folded={folded_head}, opened={opened_head}",
    )

    centered_head = ctx.part_world_position(head)
    with ctx.pose({head_swivel: 0.70}):
        swiveled_head = ctx.part_world_position(head)
    ctx.check(
        "head swivel yaws the head frame sideways",
        centered_head is not None
        and swiveled_head is not None
        and abs(swiveled_head[1] - centered_head[1]) > 0.025,
        details=f"centered={centered_head}, swiveled={swiveled_head}",
    )

    flat_top = ctx.part_element_world_aabb(head, elem="top_bar")
    with ctx.pose({head_tilt: 0.55}):
        tilted_top = ctx.part_element_world_aabb(head, elem="top_bar")
    flat_x = (flat_top[0][0] + flat_top[1][0]) / 2.0 if flat_top else None
    tilted_x = (tilted_top[0][0] + tilted_top[1][0]) / 2.0 if tilted_top else None
    ctx.check(
        "head tilt pitches the VESA frame",
        flat_x is not None and tilted_x is not None and abs(tilted_x - flat_x) > 0.015,
        details=f"flat_x={flat_x}, tilted_x={tilted_x}",
    )

    return ctx.report()


object_model = build_object_model()
