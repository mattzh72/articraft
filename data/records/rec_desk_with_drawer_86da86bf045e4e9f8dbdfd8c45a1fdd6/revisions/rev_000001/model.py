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
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vanity_dressing_table")

    wood = model.material("warm_walnut", rgba=(0.50, 0.28, 0.12, 1.0))
    dark_wood = model.material("dark_walnut_edges", rgba=(0.30, 0.16, 0.07, 1.0))
    shadow = model.material("drawer_shadow", rgba=(0.08, 0.055, 0.035, 1.0))
    brass = model.material("brushed_brass", rgba=(0.95, 0.70, 0.28, 1.0))
    mirror_blue = model.material("cool_mirror_glass", rgba=(0.72, 0.86, 0.92, 0.72))
    mirror_back = model.material("dark_mirror_backing", rgba=(0.05, 0.06, 0.07, 1.0))

    table = model.part("table")

    # Broad vanity top and a darker lipped apron make the table read as a real
    # furniture piece rather than a simple board.
    table.visual(
        Box((1.50, 0.58, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.755)),
        material=wood,
        name="wide_top",
    )
    table.visual(
        Box((1.54, 0.035, 0.040)),
        origin=Origin(xyz=(0.0, -0.3075, 0.750)),
        material=dark_wood,
        name="front_apron",
    )
    table.visual(
        Box((1.54, 0.030, 0.055)),
        origin=Origin(xyz=(0.0, 0.295, 0.728)),
        material=dark_wood,
        name="rear_lip",
    )

    # Drawer pedestals: each side is an open bay with side panels, bottom deck,
    # rear stop, and a pair of visible guide rails for the sliding drawer.
    for prefix, x in (("side_drawer_0", -0.45), ("side_drawer_1", 0.45)):
        for suffix, sx in (("outer_panel", x - 0.225), ("inner_panel", x + 0.225)):
            table.visual(
                Box((0.030, 0.520, 0.155)),
                origin=Origin(xyz=(sx, -0.015, 0.6475)),
                material=wood,
                name=f"{prefix}_{suffix}",
            )
        table.visual(
            Box((0.480, 0.520, 0.030)),
            origin=Origin(xyz=(x, -0.015, 0.585)),
            material=wood,
            name=f"{prefix}_bottom_deck",
        )
        table.visual(
            Box((0.480, 0.030, 0.155)),
            origin=Origin(xyz=(x, 0.245, 0.6475)),
            material=dark_wood,
            name=f"{prefix}_rear_stop",
        )
        table.visual(
            Box((0.015, 0.420, 0.022)),
            origin=Origin(xyz=(x - 0.2025, -0.050, 0.662)),
            material=brass,
            name=f"{prefix}_guide_rail_0",
        )
        table.visual(
            Box((0.015, 0.420, 0.022)),
            origin=Origin(xyz=(x + 0.2025, -0.050, 0.662)),
            material=brass,
            name=f"{prefix}_guide_rail_1",
        )

    # Slender legs and a low rear stretcher keep every root feature physically
    # connected while leaving the knee space open.
    for i, (x, y) in enumerate(
        (
            (-0.675, -0.250),
            (-0.225, -0.250),
            (0.225, -0.250),
            (0.675, -0.250),
            (-0.675, 0.235),
            (-0.225, 0.235),
            (0.225, 0.235),
            (0.675, 0.235),
        )
    ):
        table.visual(
            Box((0.050, 0.050, 0.570)),
            origin=Origin(xyz=(x, y, 0.285)),
            material=dark_wood,
            name=f"leg_{i}",
        )
    table.visual(
        Box((1.35, 0.030, 0.060)),
        origin=Origin(xyz=(0.0, 0.235, 0.430)),
        material=dark_wood,
        name="rear_stretcher",
    )

    # Tall rear posts carry the tilting mirror.  The brass bosses at the hinge
    # height represent the left and right fixed hinge leaves.
    for i, x in enumerate((-0.49, 0.49)):
        table.visual(
            Cylinder(radius=0.026, length=0.660),
            origin=Origin(xyz=(x, 0.240, 1.115)),
            material=wood,
            name=f"top_post_{i}",
        )
        table.visual(
            Sphere(radius=0.034),
            origin=Origin(xyz=(x, 0.240, 1.465)),
            material=dark_wood,
            name=f"post_finial_{i}",
        )
        table.visual(
            Cylinder(radius=0.020, length=0.040),
            origin=Origin(
                xyz=(x - 0.018 if x > 0 else x + 0.018, 0.240, 1.220),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=brass,
            name=f"fixed_hinge_boss_{i}",
        )

    for drawer_name, x in (("drawer_0", -0.45), ("drawer_1", 0.45)):
        drawer = model.part(drawer_name)
        drawer.visual(
            Box((0.390, 0.456, 0.105)),
            origin=Origin(),
            material=wood,
            name="drawer_box",
        )
        drawer.visual(
            Box((0.380, 0.030, 0.135)),
            origin=Origin(xyz=(0.0, -0.243, 0.0)),
            material=dark_wood,
            name="drawer_front",
        )
        drawer.visual(
            Box((0.300, 0.008, 0.085)),
            origin=Origin(xyz=(0.0, -0.260, 0.0)),
            material=wood,
            name="inset_panel",
        )
        drawer.visual(
            Cylinder(radius=0.006, length=0.026),
            origin=Origin(xyz=(-0.080, -0.272, 0.010), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brass,
            name="handle_post_0",
        )
        drawer.visual(
            Cylinder(radius=0.006, length=0.026),
            origin=Origin(xyz=(0.080, -0.272, 0.010), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brass,
            name="handle_post_1",
        )
        drawer.visual(
            Cylinder(radius=0.008, length=0.190),
            origin=Origin(xyz=(0.0, -0.286, 0.010), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brass,
            name="handle_bar",
        )

        model.articulation(
            f"table_to_{drawer_name}",
            ArticulationType.PRISMATIC,
            parent=table,
            child=drawer,
            origin=Origin(xyz=(x, -0.050, 0.662)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=55.0, velocity=0.35, lower=0.0, upper=0.270),
        )

    mirror = model.part("mirror")
    mirror.visual(
        Box((0.810, 0.025, 0.640)),
        origin=Origin(xyz=(0.0, 0.014, 0.020)),
        material=mirror_back,
        name="mirror_backing",
    )
    mirror.visual(
        Box((0.780, 0.030, 0.055)),
        origin=Origin(xyz=(0.0, -0.004, 0.3375)),
        material=wood,
        name="top_frame",
    )
    mirror.visual(
        Box((0.780, 0.030, 0.055)),
        origin=Origin(xyz=(0.0, -0.004, -0.2975)),
        material=wood,
        name="bottom_frame",
    )
    mirror.visual(
        Box((0.055, 0.030, 0.640)),
        origin=Origin(xyz=(-0.390, -0.004, 0.020)),
        material=wood,
        name="side_frame_0",
    )
    mirror.visual(
        Box((0.055, 0.030, 0.640)),
        origin=Origin(xyz=(0.390, -0.004, 0.020)),
        material=wood,
        name="side_frame_1",
    )
    mirror.visual(
        Box((0.725, 0.008, 0.555)),
        origin=Origin(xyz=(0.0, -0.021, 0.020)),
        material=mirror_blue,
        name="mirror_glass",
    )
    for i, x in enumerate((-0.4325, 0.4325)):
        mirror.visual(
            Cylinder(radius=0.014, length=0.065),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brass,
            name=f"hinge_pin_{i}",
        )

    model.articulation(
        "table_to_mirror",
        ArticulationType.REVOLUTE,
        parent=table,
        child=mirror,
        origin=Origin(xyz=(0.0, 0.240, 1.220)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=-0.38, upper=0.38),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    table = object_model.get_part("table")
    mirror = object_model.get_part("mirror")
    drawer_0 = object_model.get_part("drawer_0")
    drawer_1 = object_model.get_part("drawer_1")
    mirror_hinge = object_model.get_articulation("table_to_mirror")
    drawer_joint_0 = object_model.get_articulation("table_to_drawer_0")
    drawer_joint_1 = object_model.get_articulation("table_to_drawer_1")

    ctx.expect_contact(
        drawer_0,
        table,
        elem_a="drawer_box",
        elem_b="side_drawer_0_guide_rail_0",
        name="left drawer rides on its guide rail",
    )
    ctx.expect_contact(
        drawer_1,
        table,
        elem_a="drawer_box",
        elem_b="side_drawer_1_guide_rail_1",
        name="right drawer rides on its guide rail",
    )
    ctx.expect_overlap(
        drawer_0,
        table,
        axes="y",
        elem_a="drawer_box",
        elem_b="side_drawer_0_guide_rail_0",
        min_overlap=0.35,
        name="closed left drawer remains captured by rail length",
    )
    ctx.expect_overlap(
        drawer_1,
        table,
        axes="y",
        elem_a="drawer_box",
        elem_b="side_drawer_1_guide_rail_1",
        min_overlap=0.35,
        name="closed right drawer remains captured by rail length",
    )

    rest_left = ctx.part_world_position(drawer_0)
    rest_right = ctx.part_world_position(drawer_1)
    with ctx.pose({drawer_joint_0: 0.27, drawer_joint_1: 0.27}):
        pulled_left = ctx.part_world_position(drawer_0)
        pulled_right = ctx.part_world_position(drawer_1)
        ctx.expect_overlap(
            drawer_0,
            table,
            axes="y",
            elem_a="drawer_box",
            elem_b="side_drawer_0_guide_rail_0",
            min_overlap=0.12,
            name="extended left drawer still retained on rail",
        )
        ctx.expect_overlap(
            drawer_1,
            table,
            axes="y",
            elem_a="drawer_box",
            elem_b="side_drawer_1_guide_rail_1",
            min_overlap=0.12,
            name="extended right drawer still retained on rail",
        )

    ctx.check(
        "drawers slide toward the user",
        rest_left is not None
        and rest_right is not None
        and pulled_left is not None
        and pulled_right is not None
        and pulled_left[1] < rest_left[1] - 0.25
        and pulled_right[1] < rest_right[1] - 0.25,
        details=f"rest={rest_left, rest_right}, pulled={pulled_left, pulled_right}",
    )

    ctx.expect_contact(
        mirror,
        table,
        elem_a="hinge_pin_0",
        elem_b="fixed_hinge_boss_0",
        contact_tol=0.002,
        name="mirror left hinge meets left post",
    )
    ctx.expect_contact(
        mirror,
        table,
        elem_a="hinge_pin_1",
        elem_b="fixed_hinge_boss_1",
        contact_tol=0.002,
        name="mirror right hinge meets right post",
    )
    ctx.allow_overlap(
        mirror,
        table,
        elem_a="hinge_pin_0",
        elem_b="fixed_hinge_boss_0",
        reason="The brass mirror hinge pin is intentionally captured inside the fixed post boss.",
    )
    ctx.allow_overlap(
        mirror,
        table,
        elem_a="hinge_pin_1",
        elem_b="fixed_hinge_boss_1",
        reason="The brass mirror hinge pin is intentionally captured inside the fixed post boss.",
    )

    rest_glass_aabb = ctx.part_element_world_aabb(mirror, elem="mirror_glass")
    with ctx.pose({mirror_hinge: 0.34}):
        tilted_glass_aabb = ctx.part_element_world_aabb(mirror, elem="mirror_glass")
    ctx.check(
        "mirror tilts on a horizontal side-post hinge",
        rest_glass_aabb is not None
        and tilted_glass_aabb is not None
        and (tilted_glass_aabb[1][1] - tilted_glass_aabb[0][1])
        > (rest_glass_aabb[1][1] - rest_glass_aabb[0][1]) + 0.15,
        details=f"rest={rest_glass_aabb}, tilted={tilted_glass_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
