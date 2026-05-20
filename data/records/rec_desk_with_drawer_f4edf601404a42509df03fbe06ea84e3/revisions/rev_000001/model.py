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
    model = ArticulatedObject(name="secretary_drop_front_desk")

    walnut = Material("dark_walnut", color=(0.33, 0.17, 0.07, 1.0))
    endgrain = Material("endgrain_walnut", color=(0.24, 0.11, 0.045, 1.0))
    brass = Material("aged_brass", color=(0.85, 0.61, 0.23, 1.0))
    shadow = Material("shadowed_interior", color=(0.08, 0.055, 0.035, 1.0))

    width = 0.92
    depth = 0.42
    body_h = 0.58
    body_bottom = 0.76
    body_top = body_bottom + body_h
    wall = 0.035
    front_y = -depth / 2.0
    hinge_y = front_y - 0.025

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((wall, depth, body_h)),
        origin=Origin(xyz=(-width / 2.0 + wall / 2.0, 0.0, body_bottom + body_h / 2.0)),
        material=walnut,
        name="side_panel_0",
    )
    cabinet.visual(
        Box((wall, depth, body_h)),
        origin=Origin(xyz=(width / 2.0 - wall / 2.0, 0.0, body_bottom + body_h / 2.0)),
        material=walnut,
        name="side_panel_1",
    )
    cabinet.visual(
        Box((width - 2.0 * wall, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, body_bottom + wall / 2.0)),
        material=walnut,
        name="bottom_panel",
    )
    cabinet.visual(
        Box((width - 2.0 * wall, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, body_top - wall / 2.0)),
        material=walnut,
        name="top_panel",
    )
    cabinet.visual(
        Box((width - 2.0 * wall, wall, body_h - 2.0 * wall)),
        origin=Origin(xyz=(0.0, depth / 2.0 - wall / 2.0, body_bottom + body_h / 2.0)),
        material=endgrain,
        name="back_panel",
    )

    divider_x = 0.115
    divider_t = 0.025
    cabinet.visual(
        Box((divider_t, depth - wall, body_h - 2.0 * wall)),
        origin=Origin(
            xyz=(
                divider_x,
                (front_y + (depth / 2.0 - wall)) / 2.0,
                body_bottom + body_h / 2.0,
            )
        ),
        material=walnut,
        name="vertical_divider",
    )

    # Fixed guide rails for the pull-out writing shelf and the two small drawers.
    rail_t = 0.018
    rail_h = 0.018
    rail_len = 0.285
    rail_y = front_y + rail_len / 2.0 + 0.035
    shelf_z = body_bottom + 0.085
    shelf_th = 0.026
    shelf_rail_z = shelf_z - 0.025 - rail_h / 2.0
    for name, x in (
        ("shelf_rail_0", -width / 2.0 + wall + rail_t / 2.0),
        ("shelf_rail_1", divider_x - divider_t / 2.0 - rail_t / 2.0),
    ):
        cabinet.visual(
            Box((rail_t, rail_len, rail_h)),
            origin=Origin(xyz=(x, rail_y, shelf_rail_z)),
            material=endgrain,
            name=name,
        )

    drawer_w = 0.27
    drawer_d = 0.245
    drawer_h = 0.125
    drawer_x = 0.27
    drawer_y = -0.005
    lower_z = body_bottom + 0.245
    upper_z = body_bottom + 0.430
    for row, z in enumerate((lower_z, upper_z)):
        rail_z = z - drawer_h / 2.0 - 0.018 - rail_h / 2.0
        for side, x in (
            (0, divider_x + divider_t / 2.0 + rail_t / 2.0),
            (1, width / 2.0 - wall - rail_t / 2.0),
        ):
            cabinet.visual(
                Box((rail_t, rail_len, rail_h)),
                origin=Origin(xyz=(x, rail_y, rail_z)),
                material=endgrain,
                name=f"drawer_{row}_rail_{side}",
            )

    # Interleaved hinge knuckles fixed to the cabinet edge.
    hinge_r = 0.015
    for name, x, length in (
        ("hinge_knuckle_0", -0.31, 0.18),
        ("hinge_knuckle_1", 0.31, 0.18),
    ):
        cabinet.visual(
            Cylinder(radius=hinge_r, length=length),
            origin=Origin(xyz=(x, hinge_y, body_bottom), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brass,
            name=name,
        )
        cabinet.visual(
            Box((length, 0.020, 0.008)),
            origin=Origin(xyz=(x, front_y - 0.010, body_bottom + 0.009)),
            material=brass,
            name=f"{name}_plate",
        )

    # A dark backer makes the cabinet read as hollow when the fall-front is open.
    cabinet.visual(
        Box((0.48, 0.004, 0.19)),
        origin=Origin(xyz=(-0.17, depth / 2.0 - wall - 0.002, body_bottom + 0.18)),
        material=shadow,
        name="shelf_bay_shadow",
    )

    drop_front = model.part("drop_front")
    leaf_h = body_h - 0.02
    leaf_t = 0.030
    drop_front.visual(
        Box((width - 0.08, leaf_t, leaf_h)),
        origin=Origin(xyz=(0.0, 0.005, hinge_r + leaf_h / 2.0)),
        material=walnut,
        name="leaf_panel",
    )
    drop_front.visual(
        Box((width - 0.16, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, -0.015, hinge_r + 0.055)),
        material=endgrain,
        name="lower_raised_rail",
    )
    drop_front.visual(
        Box((width - 0.16, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, -0.015, hinge_r + leaf_h - 0.055)),
        material=endgrain,
        name="upper_raised_rail",
    )
    drop_front.visual(
        Box((0.020, 0.010, leaf_h - 0.14)),
        origin=Origin(xyz=(-0.30, -0.015, hinge_r + leaf_h / 2.0)),
        material=endgrain,
        name="raised_stile_0",
    )
    drop_front.visual(
        Box((0.020, 0.010, leaf_h - 0.14)),
        origin=Origin(xyz=(0.30, -0.015, hinge_r + leaf_h / 2.0)),
        material=endgrain,
        name="raised_stile_1",
    )
    drop_front.visual(
        Cylinder(radius=hinge_r, length=0.31),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="leaf_hinge_knuckle",
    )

    writing_shelf = model.part("writing_shelf")
    shelf_w = 0.50
    shelf_d = 0.300
    shelf_x = -0.165
    shelf_y = -0.005
    writing_shelf.visual(
        Box((shelf_w, shelf_d, shelf_th)),
        origin=Origin(),
        material=walnut,
        name="shelf_board",
    )
    writing_shelf.visual(
        Box((shelf_w, 0.026, 0.040)),
        origin=Origin(xyz=(0.0, -shelf_d / 2.0 - 0.013, 0.007)),
        material=endgrain,
        name="front_lip",
    )
    writing_shelf.visual(
        Box((0.024, shelf_d, 0.012)),
        origin=Origin(xyz=(-shelf_w / 2.0 + 0.012, 0.0, -0.019)),
        material=endgrain,
        name="shelf_runner_0",
    )
    writing_shelf.visual(
        Box((0.024, shelf_d, 0.012)),
        origin=Origin(xyz=(shelf_w / 2.0 - 0.012, 0.0, -0.019)),
        material=endgrain,
        name="shelf_runner_1",
    )

    def add_drawer(name: str) -> object:
        drawer = model.part(name)
        drawer.visual(
            Box((drawer_w, drawer_d, drawer_h)),
            origin=Origin(),
            material=walnut,
            name="drawer_box",
        )
        drawer.visual(
            Box((drawer_w + 0.018, 0.020, drawer_h + 0.018)),
            origin=Origin(xyz=(0.0, -drawer_d / 2.0 - 0.010, 0.0)),
            material=endgrain,
            name="drawer_front",
        )
        drawer.visual(
            Cylinder(radius=0.018, length=0.026),
            origin=Origin(xyz=(0.0, -drawer_d / 2.0 - 0.033, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brass,
            name="round_pull",
        )
        drawer.visual(
            Box((0.020, drawer_d, 0.018)),
            origin=Origin(xyz=(-drawer_w / 2.0 + 0.010, 0.0, -drawer_h / 2.0 - 0.009)),
            material=endgrain,
            name="side_runner_0",
        )
        drawer.visual(
            Box((0.020, drawer_d, 0.018)),
            origin=Origin(xyz=(drawer_w / 2.0 - 0.010, 0.0, -drawer_h / 2.0 - 0.009)),
            material=endgrain,
            name="side_runner_1",
        )
        return drawer

    lower_drawer = add_drawer("lower_drawer")
    upper_drawer = add_drawer("upper_drawer")

    model.articulation(
        "cabinet_to_drop_front",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=drop_front,
        origin=Origin(xyz=(0.0, hinge_y, body_bottom)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=math.pi / 2.0),
    )
    model.articulation(
        "cabinet_to_writing_shelf",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=writing_shelf,
        origin=Origin(xyz=(shelf_x, shelf_y, shelf_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.25, lower=0.0, upper=0.25),
    )
    model.articulation(
        "cabinet_to_lower_drawer",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=lower_drawer,
        origin=Origin(xyz=(drawer_x, drawer_y, lower_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.25, lower=0.0, upper=0.20),
    )
    model.articulation(
        "cabinet_to_upper_drawer",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=upper_drawer,
        origin=Origin(xyz=(drawer_x, drawer_y, upper_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.25, lower=0.0, upper=0.20),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    drop_front = object_model.get_part("drop_front")
    shelf = object_model.get_part("writing_shelf")
    lower = object_model.get_part("lower_drawer")
    upper = object_model.get_part("upper_drawer")

    leaf_hinge = object_model.get_articulation("cabinet_to_drop_front")
    shelf_slide = object_model.get_articulation("cabinet_to_writing_shelf")
    lower_slide = object_model.get_articulation("cabinet_to_lower_drawer")
    upper_slide = object_model.get_articulation("cabinet_to_upper_drawer")

    ctx.expect_gap(
        cabinet,
        drop_front,
        axis="y",
        positive_elem="side_panel_0",
        negative_elem="leaf_panel",
        min_gap=0.0,
        max_gap=0.012,
        name="closed leaf sits just proud of cabinet front",
    )
    ctx.expect_overlap(
        drop_front,
        cabinet,
        axes="xz",
        elem_a="leaf_panel",
        elem_b="back_panel",
        min_overlap=0.05,
        name="closed leaf covers the front opening height",
    )

    shelf_rest = ctx.part_world_position(shelf)
    with ctx.pose({shelf_slide: 0.25}):
        shelf_out = ctx.part_world_position(shelf)
        ctx.expect_overlap(
            shelf,
            cabinet,
            axes="y",
            elem_a="shelf_board",
            elem_b="bottom_panel",
            min_overlap=0.05,
            name="extended writing shelf remains captured in cabinet",
        )
    ctx.check(
        "writing shelf slides outward",
        shelf_rest is not None and shelf_out is not None and shelf_out[1] < shelf_rest[1] - 0.20,
        details=f"rest={shelf_rest}, extended={shelf_out}",
    )

    for drawer, joint, label in (
        (lower, lower_slide, "lower drawer"),
        (upper, upper_slide, "upper drawer"),
    ):
        rest = ctx.part_world_position(drawer)
        with ctx.pose({joint: 0.20}):
            extended = ctx.part_world_position(drawer)
            ctx.expect_overlap(
                drawer,
                cabinet,
                axes="y",
                elem_a="drawer_box",
                elem_b="bottom_panel",
                min_overlap=0.05,
                name=f"{label} remains on guide rails when extended",
            )
        ctx.check(
            f"{label} slides outward",
            rest is not None and extended is not None and extended[1] < rest[1] - 0.16,
            details=f"rest={rest}, extended={extended}",
        )

    with ctx.pose({leaf_hinge: math.pi / 2.0}):
        leaf_aabb = ctx.part_world_aabb(drop_front)
        ctx.check(
            "drop front opens to a flat work surface",
            leaf_aabb is not None
            and (leaf_aabb[1][2] - leaf_aabb[0][2]) < 0.08
            and leaf_aabb[0][1] < -0.70,
            details=f"aabb={leaf_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
