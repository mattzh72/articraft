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
    model = ArticulatedObject(name="wall_mount_laundry_desk")

    painted = model.material("soft_white_laminate", rgba=(0.86, 0.88, 0.84, 1.0))
    maple = model.material("sealed_maple_worktop", rgba=(0.73, 0.55, 0.34, 1.0))
    edge = model.material("maple_edge_band", rgba=(0.58, 0.39, 0.22, 1.0))
    metal = model.material("brushed_steel", rgba=(0.45, 0.47, 0.49, 1.0))
    shadow = model.material("shadow_gap", rgba=(0.08, 0.09, 0.09, 1.0))
    fabric_light = model.material("light_laundry_liner", rgba=(0.78, 0.74, 0.65, 1.0))
    fabric_blue = model.material("blue_sorting_liner", rgba=(0.32, 0.48, 0.68, 1.0))
    fabric_gray = model.material("gray_sorting_liner", rgba=(0.42, 0.43, 0.42, 1.0))

    width = 0.95
    depth = 0.28
    side_t = 0.024
    back_t = 0.018
    bottom_z = 0.42
    top_z = 1.38
    hinge_z = 0.82
    joint_x = depth + 0.020

    cabinet = model.part("cabinet")
    # A shallow wall-mounted carcass: continuous back, side cheeks, shelves, and
    # a lower drawer bay.  The open panel construction keeps the cabinet hollow.
    cabinet.visual(
        Box((back_t, width, top_z - bottom_z)),
        origin=Origin(xyz=(back_t / 2.0, 0.0, (top_z + bottom_z) / 2.0)),
        material=painted,
        name="back_panel",
    )
    cabinet.visual(
        Box((0.035, width - 0.08, 0.045)),
        origin=Origin(xyz=(-0.006, 0.0, 1.28)),
        material=painted,
        name="wall_cleat",
    )
    for y in (-width / 2.0 + side_t / 2.0, width / 2.0 - side_t / 2.0):
        cabinet.visual(
            Box((depth, side_t, top_z - bottom_z)),
            origin=Origin(xyz=(depth / 2.0, y, (top_z + bottom_z) / 2.0)),
            material=painted,
            name=f"side_cheek_{0 if y < 0 else 1}",
        )
    for z, name, thickness in (
        (bottom_z + 0.0125, "bottom_shelf", 0.025),
        (0.770, "drawer_top_shelf", 0.025),
        (hinge_z - 0.015, "hinge_shelf", 0.030),
        (top_z - 0.0125, "top_shelf", 0.025),
    ):
        cabinet.visual(
            Box((depth, width, thickness)),
            origin=Origin(xyz=(depth / 2.0, 0.0, z)),
            material=painted,
            name=name,
        )
    cabinet.visual(
        Box((0.045, width, 0.050)),
        origin=Origin(xyz=(depth - 0.0225, 0.0, hinge_z - 0.010)),
        material=painted,
        name="front_hinge_rail",
    )
    cabinet.visual(
        Box((0.040, width, 0.045)),
        origin=Origin(xyz=(depth - 0.020, 0.0, 0.755)),
        material=painted,
        name="drawer_face_stop",
    )

    # Fixed guide rails: upper and lower lips form a shallow C-channel on each
    # side of the lower bay.
    rail_y = width / 2.0 - side_t - 0.012
    for sign, upper_name, lower_name in (
        (-1, "rail_upper_0", "rail_lower_0"),
        (1, "rail_upper_1", "rail_lower_1"),
    ):
        cabinet.visual(
            Box((0.235, 0.024, 0.018)),
            origin=Origin(xyz=(0.165, sign * rail_y, 0.650)),
            material=metal,
            name=upper_name,
        )
        cabinet.visual(
            Box((0.235, 0.024, 0.018)),
            origin=Origin(xyz=(0.165, sign * rail_y, 0.590)),
            material=metal,
            name=lower_name,
        )

    # Alternating hinge knuckles mounted to the cabinet side zones, with small
    # plates tying the barrels into the lower front edge.
    for sign, suffix in ((-1, "0"), (1, "1")):
        cabinet.visual(
            Box((0.055, 0.205, 0.016)),
            origin=Origin(xyz=(depth + 0.006, sign * 0.345, hinge_z - 0.002)),
            material=metal,
            name=f"hinge_plate_{suffix}",
        )
        cabinet.visual(
            Cylinder(radius=0.020, length=0.205),
            origin=Origin(xyz=(joint_x, sign * 0.345, hinge_z), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=metal,
            name=f"hinge_barrel_{suffix}",
        )

    leaf = model.part("leaf")
    # The child frame is the hinge axis.  At q=0 the panel is vertical; positive
    # rotation about +Y lays it outward into a horizontal folding surface.
    leaf.visual(
        Box((0.032, width - 0.11, 0.540)),
        origin=Origin(xyz=(0.022, 0.0, 0.300)),
        material=maple,
        name="leaf_panel",
    )
    leaf.visual(
        Box((0.040, width - 0.08, 0.026)),
        origin=Origin(xyz=(0.023, 0.0, 0.575)),
        material=edge,
        name="free_edge_band",
    )
    for sign, suffix in ((-1, "0"), (1, "1")):
        leaf.visual(
            Box((0.040, 0.026, 0.555)),
            origin=Origin(xyz=(0.023, sign * (width / 2.0 - 0.043), 0.305)),
            material=edge,
            name=f"side_edge_{suffix}",
        )
    leaf.visual(
        Cylinder(radius=0.020, length=0.485),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="hinge_knuckle",
    )
    leaf.visual(
        Box((0.040, 0.380, 0.035)),
        origin=Origin(xyz=(0.017, 0.0, 0.024)),
        material=metal,
        name="hinge_strap",
    )
    leaf.visual(
        Box((0.032, 0.020, 0.090)),
        origin=Origin(xyz=(0.054, -0.180, 0.465)),
        material=metal,
        name="handle_post_0",
    )
    leaf.visual(
        Box((0.032, 0.020, 0.090)),
        origin=Origin(xyz=(0.054, 0.180, 0.465)),
        material=metal,
        name="handle_post_1",
    )
    leaf.visual(
        Box((0.026, 0.440, 0.028)),
        origin=Origin(xyz=(0.078, 0.0, 0.465)),
        material=metal,
        name="leaf_pull",
    )

    drawer = model.part("drawer")
    # Open-top sorting tray with three laundry compartments.  Its closed front
    # sits flush with the lower cabinet face and the whole tray translates out.
    drawer.visual(
        Box((0.035, width - 0.13, 0.280)),
        origin=Origin(xyz=(depth + 0.0175, 0.0, 0.610)),
        material=painted,
        name="drawer_front",
    )
    drawer.visual(
        Box((0.250, width - 0.21, 0.020)),
        origin=Origin(xyz=(0.155, 0.0, 0.485)),
        material=painted,
        name="tray_bottom",
    )
    for sign, side_name, runner_name in (
        (-1, "tray_side_0", "drawer_runner_0"),
        (1, "tray_side_1", "drawer_runner_1"),
    ):
        drawer.visual(
            Box((0.250, 0.030, 0.125)),
            origin=Origin(xyz=(0.155, sign * (width / 2.0 - 0.105), 0.550)),
            material=painted,
            name=side_name,
        )
        drawer.visual(
            Box((0.245, 0.090, 0.020)),
            origin=Origin(xyz=(0.160, sign * (width / 2.0 - 0.065), 0.620)),
            material=metal,
            name=runner_name,
        )
    drawer.visual(
        Box((0.026, width - 0.21, 0.125)),
        origin=Origin(xyz=(0.055, 0.0, 0.550)),
        material=painted,
        name="tray_back",
    )
    for y, suffix, material in (
        (-0.135, "0", fabric_light),
        (0.0, "1", fabric_blue),
        (0.135, "2", fabric_gray),
    ):
        drawer.visual(
            Box((0.185, 0.100, 0.012)),
            origin=Origin(xyz=(0.157, y, 0.502)),
            material=material,
            name=f"liner_floor_{suffix}",
        )
    for y, suffix in ((-0.135, "0"), (0.135, "1")):
        drawer.visual(
            Box((0.195, 0.014, 0.105)),
            origin=Origin(xyz=(0.160, y, 0.548)),
            material=painted,
            name=f"divider_{suffix}",
        )
    drawer.visual(
        Box((0.032, 0.025, 0.060)),
        origin=Origin(xyz=(depth + 0.051, -0.210, 0.610)),
        material=metal,
        name="drawer_pull_post_0",
    )
    drawer.visual(
        Box((0.032, 0.025, 0.060)),
        origin=Origin(xyz=(depth + 0.051, 0.210, 0.610)),
        material=metal,
        name="drawer_pull_post_1",
    )
    drawer.visual(
        Box((0.026, 0.500, 0.030)),
        origin=Origin(xyz=(depth + 0.080, 0.0, 0.610)),
        material=metal,
        name="drawer_pull",
    )

    model.articulation(
        "cabinet_to_leaf",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=leaf,
        origin=Origin(xyz=(joint_x, 0.0, hinge_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.0, lower=0.0, upper=pi / 2.0),
    )
    model.articulation(
        "cabinet_to_drawer",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=drawer,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.35, lower=0.0, upper=0.300),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    leaf = object_model.get_part("leaf")
    drawer = object_model.get_part("drawer")
    leaf_hinge = object_model.get_articulation("cabinet_to_leaf")
    drawer_slide = object_model.get_articulation("cabinet_to_drawer")

    with ctx.pose({leaf_hinge: 0.0, drawer_slide: 0.0}):
        ctx.expect_gap(
            leaf,
            cabinet,
            axis="x",
            min_gap=0.0,
            max_gap=0.030,
            positive_elem="leaf_panel",
            negative_elem="front_hinge_rail",
            name="closed leaf sits just proud of cabinet face",
        )
        ctx.expect_gap(
            drawer,
            cabinet,
            axis="x",
            min_gap=0.0,
            max_gap=0.003,
            positive_elem="drawer_front",
            negative_elem="drawer_face_stop",
            name="closed drawer front is flush with face stop",
        )
        closed_leaf_aabb = ctx.part_element_world_aabb(leaf, elem="leaf_panel")
        closed_drawer_pos = ctx.part_world_position(drawer)

    with ctx.pose({leaf_hinge: pi / 2.0, drawer_slide: 0.0}):
        open_leaf_aabb = ctx.part_element_world_aabb(leaf, elem="leaf_panel")
        ctx.expect_gap(
            leaf,
            drawer,
            axis="z",
            min_gap=0.020,
            positive_elem="leaf_panel",
            negative_elem="drawer_front",
            name="folded desk surface clears drawer front",
        )

    with ctx.pose({drawer_slide: 0.300, leaf_hinge: 0.0}):
        extended_drawer_pos = ctx.part_world_position(drawer)
        ctx.expect_gap(
            drawer,
            cabinet,
            axis="z",
            min_gap=-0.005,
            max_gap=0.070,
            positive_elem="drawer_runner_1",
            negative_elem="rail_lower_1",
            name="drawer runner stays in guide rail channel height",
        )

    if closed_leaf_aabb is not None and open_leaf_aabb is not None:
        closed_size_x = closed_leaf_aabb[1][0] - closed_leaf_aabb[0][0]
        closed_size_z = closed_leaf_aabb[1][2] - closed_leaf_aabb[0][2]
        open_size_x = open_leaf_aabb[1][0] - open_leaf_aabb[0][0]
        open_size_z = open_leaf_aabb[1][2] - open_leaf_aabb[0][2]
        ctx.check(
            "leaf rotates from vertical front to horizontal desk",
            closed_size_z > 0.50 and closed_size_x < 0.060 and open_size_x > 0.50 and open_size_z < 0.060,
            details=f"closed x/z=({closed_size_x:.3f}, {closed_size_z:.3f}), "
            f"open x/z=({open_size_x:.3f}, {open_size_z:.3f})",
        )
    else:
        ctx.fail("leaf rotates from vertical front to horizontal desk", "missing leaf panel AABB")

    ctx.check(
        "drawer extends outward on prismatic slides",
        closed_drawer_pos is not None
        and extended_drawer_pos is not None
        and extended_drawer_pos[0] > closed_drawer_pos[0] + 0.250,
        details=f"closed={closed_drawer_pos}, extended={extended_drawer_pos}",
    )

    return ctx.report()


object_model = build_object_model()
