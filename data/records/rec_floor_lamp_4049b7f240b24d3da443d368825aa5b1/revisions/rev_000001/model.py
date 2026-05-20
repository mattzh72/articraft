from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swing_arm_floor_lamp")

    matte_black = model.material("matte_black", rgba=(0.015, 0.014, 0.012, 1.0))
    dark_graphite = model.material("dark_graphite", rgba=(0.08, 0.08, 0.075, 1.0))
    warm_brass = model.material("warm_brass", rgba=(0.78, 0.58, 0.28, 1.0))
    shade_linen = model.material("warm_linen_shade", rgba=(0.92, 0.82, 0.62, 0.96))
    bulb_glass = model.material("warm_bulb_glass", rgba=(1.0, 0.86, 0.45, 0.65))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.205, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=matte_black,
        name="weighted_disk",
    )
    base.visual(
        Cylinder(radius=0.125, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.051)),
        material=dark_graphite,
        name="raised_plinth",
    )
    base.visual(
        Cylinder(radius=0.018, length=1.238),
        origin=Origin(xyz=(0.0, 0.0, 0.676)),
        material=matte_black,
        name="vertical_post",
    )
    base.visual(
        Cylinder(radius=0.044, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 1.310)),
        material=warm_brass,
        name="top_bearing",
    )
    base.visual(
        Cylinder(radius=0.026, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 1.3375)),
        material=warm_brass,
        name="top_pivot",
    )

    first_arm = model.part("first_arm")
    first_arm.visual(
        Cylinder(radius=0.044, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=warm_brass,
        name="proximal_hub",
    )
    for y in (-0.020, 0.020):
        first_arm.visual(
            Cylinder(radius=0.010, length=0.540),
            origin=Origin(xyz=(0.270, y, 0.0125), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=matte_black,
            name=f"rail_{'neg' if y < 0 else 'pos'}",
        )
    first_arm.visual(
        Cylinder(radius=0.042, length=0.025),
        origin=Origin(xyz=(0.550, 0.0, 0.0125)),
        material=warm_brass,
        name="elbow_hub",
    )
    first_arm.visual(
        Box((0.024, 0.080, 0.016)),
        origin=Origin(xyz=(0.550, 0.0, 0.0125)),
        material=warm_brass,
        name="elbow_bridge",
    )

    second_arm = model.part("second_arm")
    second_arm.visual(
        Cylinder(radius=0.040, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=warm_brass,
        name="proximal_hub",
    )
    for y in (-0.020, 0.020):
        second_arm.visual(
            Cylinder(radius=0.010, length=0.405),
            origin=Origin(xyz=(0.2025, y, 0.0125), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=matte_black,
            name=f"rail_{'neg' if y < 0 else 'pos'}",
        )
    second_arm.visual(
        Box((0.014, 0.118, 0.018)),
        origin=Origin(xyz=(0.405, 0.0, 0.0125)),
        material=warm_brass,
        name="fork_bridge",
    )
    for y, side in ((-0.053, "neg"), (0.053, "pos")):
        fork_name = "fork_arm_neg" if side == "neg" else "fork_arm_pos"
        yoke_name = "yoke_ear_neg" if side == "neg" else "yoke_ear_pos"
        second_arm.visual(
            Box((0.090, 0.014, 0.018)),
            origin=Origin(xyz=(0.445, y, 0.0125)),
            material=warm_brass,
            name=fork_name,
        )
        second_arm.visual(
            Box((0.050, 0.016, 0.060)),
            origin=Origin(xyz=(0.450, y, 0.0)),
            material=warm_brass,
            name=yoke_name,
        )

    shade = model.part("shade")
    shade.visual(
        Cylinder(radius=0.018, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=warm_brass,
        name="tilt_trunnion",
    )
    shade.visual(
        Cylinder(radius=0.012, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, -0.0225)),
        material=warm_brass,
        name="shade_stem",
    )
    shade_shell = LatheGeometry.from_shell_profiles(
        outer_profile=((0.055, -0.045), (0.078, -0.095), (0.140, -0.225)),
        inner_profile=((0.048, -0.050), (0.071, -0.098), (0.132, -0.218)),
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )
    shade.visual(
        mesh_from_geometry(shade_shell, "bell_shade_shell"),
        material=shade_linen,
        name="shade_shell",
    )
    shade.visual(
        Cylinder(radius=0.024, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, -0.070)),
        material=dark_graphite,
        name="lamp_socket",
    )
    shade.visual(
        Sphere(radius=0.034),
        origin=Origin(xyz=(0.0, 0.0, -0.121)),
        material=bulb_glass,
        name="bulb",
    )
    shade.visual(
        Box((0.060, 0.006, 0.006)),
        origin=Origin(xyz=(0.036, 0.0, -0.047)),
        material=warm_brass,
        name="spoke_x_pos",
    )
    shade.visual(
        Box((0.060, 0.006, 0.006)),
        origin=Origin(xyz=(-0.036, 0.0, -0.047)),
        material=warm_brass,
        name="spoke_x_neg",
    )
    shade.visual(
        Box((0.006, 0.060, 0.006)),
        origin=Origin(xyz=(0.0, 0.036, -0.047)),
        material=warm_brass,
        name="spoke_y_pos",
    )
    shade.visual(
        Box((0.006, 0.060, 0.006)),
        origin=Origin(xyz=(0.0, -0.036, -0.047)),
        material=warm_brass,
        name="spoke_y_neg",
    )

    model.articulation(
        "post_to_first",
        ArticulationType.REVOLUTE,
        parent=base,
        child=first_arm,
        origin=Origin(xyz=(0.0, 0.0, 1.350)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.6, lower=-2.35, upper=2.35),
    )
    model.articulation(
        "first_to_second",
        ArticulationType.REVOLUTE,
        parent=first_arm,
        child=second_arm,
        origin=Origin(xyz=(0.550, 0.0, 0.025)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.8, lower=-2.30, upper=2.30),
    )
    model.articulation(
        "second_to_shade",
        ArticulationType.REVOLUTE,
        parent=second_arm,
        child=shade,
        origin=Origin(xyz=(0.450, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.4, lower=-0.85, upper=0.75),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    first_arm = object_model.get_part("first_arm")
    second_arm = object_model.get_part("second_arm")
    shade = object_model.get_part("shade")
    post_to_first = object_model.get_articulation("post_to_first")
    first_to_second = object_model.get_articulation("first_to_second")
    second_to_shade = object_model.get_articulation("second_to_shade")

    ctx.check(
        "three revolute mechanisms",
        all(
            joint.articulation_type == ArticulationType.REVOLUTE
            for joint in (post_to_first, first_to_second, second_to_shade)
        ),
        details="The post joint, elbow, and shade tilt joint must all be revolute.",
    )

    ctx.expect_contact(
        first_arm,
        base,
        elem_a="proximal_hub",
        elem_b="top_pivot",
        contact_tol=0.001,
        name="first arm sits on the post-top pivot",
    )
    ctx.expect_contact(
        second_arm,
        first_arm,
        elem_a="proximal_hub",
        elem_b="elbow_hub",
        contact_tol=0.001,
        name="second arm stacks on the elbow hinge",
    )
    ctx.expect_gap(
        shade,
        second_arm,
        axis="y",
        positive_elem="tilt_trunnion",
        negative_elem="yoke_ear_neg",
        min_gap=0.0,
        max_gap=0.002,
        name="shade trunnion reaches the negative yoke ear",
    )
    ctx.expect_gap(
        second_arm,
        shade,
        axis="y",
        positive_elem="yoke_ear_pos",
        negative_elem="tilt_trunnion",
        min_gap=0.0,
        max_gap=0.002,
        name="shade trunnion reaches the positive yoke ear",
    )

    rest_tip = ctx.part_world_position(shade)
    with ctx.pose({post_to_first: 0.7}):
        swung_tip = ctx.part_world_position(shade)
    ctx.check(
        "post joint swings arm around the vertical post",
        rest_tip is not None
        and swung_tip is not None
        and swung_tip[1] > rest_tip[1] + 0.45,
        details=f"rest={rest_tip}, swung={swung_tip}",
    )

    with ctx.pose({first_to_second: 1.15}):
        folded_tip = ctx.part_world_position(shade)
    ctx.check(
        "elbow folds the second link",
        rest_tip is not None
        and folded_tip is not None
        and folded_tip[0] < rest_tip[0] - 0.12
        and folded_tip[1] > rest_tip[1] + 0.35,
        details=f"rest={rest_tip}, folded={folded_tip}",
    )

    closed_aabb = ctx.part_element_world_aabb(shade, elem="shade_shell")
    with ctx.pose({second_to_shade: 0.55}):
        tilted_aabb = ctx.part_element_world_aabb(shade, elem="shade_shell")
    ctx.check(
        "shade tilt pitches the bell shade",
        closed_aabb is not None
        and tilted_aabb is not None
        and tilted_aabb[0][0] < closed_aabb[0][0] - 0.05,
        details=f"rest_aabb={closed_aabb}, tilted_aabb={tilted_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
