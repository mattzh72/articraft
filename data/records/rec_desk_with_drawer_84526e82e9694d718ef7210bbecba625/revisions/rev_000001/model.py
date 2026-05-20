from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.hypot(dx, dy), dz)
    return (0.0, pitch, yaw)


def _add_pipe(
    part,
    name: str,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    material,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _add_node(
    part,
    name: str,
    pos: tuple[float, float, float],
    *,
    radius: float,
    material,
) -> None:
    part.visual(
        Sphere(radius=radius),
        origin=Origin(xyz=pos),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_pipe_frame_desk")

    powder_coat = model.material("powder_coat", rgba=(0.16, 0.17, 0.18, 1.0))
    graphite = model.material("graphite", rgba=(0.24, 0.25, 0.27, 1.0))
    walnut = model.material("walnut", rgba=(0.46, 0.31, 0.18, 1.0))
    blackened_steel = model.material("blackened_steel", rgba=(0.14, 0.14, 0.15, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.64, 0.66, 0.68, 1.0))

    desk = model.part("desk")

    top_width = 1.35
    top_depth = 0.65
    top_thickness = 0.035
    frame_half_width = 0.50
    frame_half_depth = 0.22
    tube_radius = 0.016
    top_rail_z = 0.699
    floor_node_z = tube_radius
    lower_rail_z = 0.165
    lower_front_y = 0.12
    lower_rear_y = -0.20

    front_left_top = (-frame_half_width, frame_half_depth, top_rail_z)
    front_right_top = (frame_half_width, frame_half_depth, top_rail_z)
    rear_left_top = (-frame_half_width, -frame_half_depth, top_rail_z)
    rear_right_top = (frame_half_width, -frame_half_depth, top_rail_z)

    front_left_bottom = (-frame_half_width, frame_half_depth, floor_node_z)
    front_right_bottom = (frame_half_width, frame_half_depth, floor_node_z)
    rear_left_bottom = (-frame_half_width, -frame_half_depth, floor_node_z)
    rear_right_bottom = (frame_half_width, -frame_half_depth, floor_node_z)

    left_front_lower = (-frame_half_width, lower_front_y, lower_rail_z)
    right_front_lower = (frame_half_width, lower_front_y, lower_rail_z)
    left_rear_lower = (-frame_half_width, lower_rear_y, lower_rail_z)
    right_rear_lower = (frame_half_width, lower_rear_y, lower_rail_z)

    for name, a, b, radius in (
        ("front_top_rail", front_left_top, front_right_top, tube_radius),
        ("rear_top_rail", rear_left_top, rear_right_top, tube_radius),
        ("left_top_rail", front_left_top, rear_left_top, tube_radius),
        ("right_top_rail", front_right_top, rear_right_top, tube_radius),
        ("front_left_leg", front_left_bottom, front_left_top, tube_radius),
        ("front_right_leg", front_right_bottom, front_right_top, tube_radius),
        ("rear_left_leg", rear_left_bottom, rear_left_top, tube_radius),
        ("rear_right_leg", rear_right_bottom, rear_right_top, tube_radius),
        ("left_lower_stretcher", left_rear_lower, left_front_lower, 0.013),
        ("right_lower_stretcher", right_rear_lower, right_front_lower, 0.013),
        ("rear_lower_stretcher", left_rear_lower, right_rear_lower, 0.013),
        ("front_footrest_bar", left_front_lower, right_front_lower, 0.013),
        ("left_back_brace", left_rear_lower, rear_right_top, 0.010),
        ("right_back_brace", right_rear_lower, rear_left_top, 0.010),
        (
            "left_top_support",
            (-0.44, -0.155, 0.703),
            (-0.10, -0.155, 0.703),
            0.011,
        ),
        (
            "right_top_support",
            (0.10, -0.155, 0.703),
            (0.44, -0.155, 0.703),
            0.011,
        ),
    ):
        _add_pipe(desk, name, a, b, radius=radius, material=powder_coat)

    for index, pos in enumerate(
        (
            front_left_top,
            front_right_top,
            rear_left_top,
            rear_right_top,
            front_left_bottom,
            front_right_bottom,
            rear_left_bottom,
            rear_right_bottom,
            left_front_lower,
            right_front_lower,
            left_rear_lower,
            right_rear_lower,
        )
    ):
        _add_node(
            desk,
            f"frame_node_{index}",
            pos,
            radius=0.019 if pos[2] > 0.30 else 0.017,
            material=graphite,
        )

    for index, x in enumerate((-0.47, 0.47)):
        desk.visual(
            Cylinder(radius=0.020, length=0.010),
            origin=Origin(xyz=(x, frame_half_depth, 0.005)),
            material=graphite,
            name=f"front_leveler_{index}",
        )
        desk.visual(
            Cylinder(radius=0.020, length=0.010),
            origin=Origin(xyz=(x, -frame_half_depth, 0.005)),
            material=graphite,
            name=f"rear_leveler_{index}",
        )

    desk.visual(
        Box((top_width, top_depth, top_thickness)),
        origin=Origin(xyz=(0.0, 0.0, 0.7325)),
        material=walnut,
        name="desktop",
    )

    guide_y = 0.005
    guide_z = 0.686
    guide_length = 0.420
    guide_width = 0.046
    guide_thickness = 0.010
    guide_x = 0.402

    desk.visual(
        Box((guide_width, guide_length, guide_thickness)),
        origin=Origin(xyz=(-guide_x, guide_y, guide_z)),
        material=blackened_steel,
        name="guide_left",
    )
    desk.visual(
        Box((guide_width, guide_length, guide_thickness)),
        origin=Origin(xyz=(guide_x, guide_y, guide_z)),
        material=blackened_steel,
        name="guide_right",
    )
    desk.visual(
        Box((0.006, guide_length, 0.022)),
        origin=Origin(xyz=(-(guide_x - 0.023), guide_y, 0.670)),
        material=blackened_steel,
        name="guide_left_keeper",
    )
    desk.visual(
        Box((0.006, guide_length, 0.022)),
        origin=Origin(xyz=((guide_x - 0.023), guide_y, 0.670)),
        material=blackened_steel,
        name="guide_right_keeper",
    )
    desk.visual(
        Box((0.86, 0.020, 0.012)),
        origin=Origin(xyz=(0.0, -0.175, 0.697)),
        material=blackened_steel,
        name="guide_cross_rear",
    )
    desk.visual(
        Box((0.86, 0.020, 0.012)),
        origin=Origin(xyz=(0.0, 0.165, 0.697)),
        material=blackened_steel,
        name="guide_cross_front",
    )
    for name, x, y in (
        ("guide_bracket_fl", -guide_x, 0.165),
        ("guide_bracket_fr", guide_x, 0.165),
        ("guide_bracket_rl", -guide_x, -0.175),
        ("guide_bracket_rr", guide_x, -0.175),
    ):
        desk.visual(
            Box((0.012, 0.026, 0.024)),
            origin=Origin(xyz=(x, y, 0.702)),
            material=blackened_steel,
            name=name,
        )

    desk.inertial = Inertial.from_geometry(
        Box((top_width, top_depth, 0.75)),
        mass=31.0,
        origin=Origin(xyz=(0.0, 0.0, 0.375)),
    )

    drawer = model.part("drawer")

    drawer_width = 0.84
    drawer_depth = 0.42
    drawer_shell_height = 0.105
    wall_thickness = 0.012
    bottom_thickness = 0.012
    runner_width = 0.024
    runner_height = 0.012
    runner_x = 0.401

    drawer.visual(
        Box((drawer_width - 2.0 * wall_thickness, 0.398, bottom_thickness)),
        origin=Origin(xyz=(0.0, -0.002, -0.054)),
        material=graphite,
        name="drawer_bottom",
    )
    drawer.visual(
        Box((wall_thickness, 0.388, drawer_shell_height)),
        origin=Origin(xyz=(-0.414, -0.006, -0.0015)),
        material=graphite,
        name="drawer_left_wall",
    )
    drawer.visual(
        Box((wall_thickness, 0.388, drawer_shell_height)),
        origin=Origin(xyz=(0.414, -0.006, -0.0015)),
        material=graphite,
        name="drawer_right_wall",
    )
    drawer.visual(
        Box((drawer_width - 2.0 * wall_thickness, wall_thickness, 0.078)),
        origin=Origin(xyz=(0.0, -0.194, -0.015)),
        material=graphite,
        name="drawer_back_wall",
    )
    drawer.visual(
        Box((drawer_width - 2.0 * wall_thickness, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, 0.190, -0.049)),
        material=blackened_steel,
        name="drawer_hinge_strip",
    )
    drawer.visual(
        Box((runner_width, 0.398, runner_height)),
        origin=Origin(xyz=(-runner_x, -0.003, 0.056)),
        material=blackened_steel,
        name="drawer_runner_left",
    )
    drawer.visual(
        Box((runner_width, 0.398, runner_height)),
        origin=Origin(xyz=(runner_x, -0.003, 0.056)),
        material=blackened_steel,
        name="drawer_runner_right",
    )

    drawer.inertial = Inertial.from_geometry(
        Box((drawer_width, drawer_depth, 0.13)),
        mass=6.8,
        origin=Origin(),
    )

    front_panel = model.part("front_panel")
    panel_width = drawer_width - 2.0 * wall_thickness
    panel_height = 0.108
    panel_thickness = 0.018

    front_panel.visual(
        Box((panel_width, panel_thickness, panel_height)),
        origin=Origin(xyz=(0.0, panel_thickness * 0.5, panel_height * 0.5)),
        material=walnut,
        name="front_leaf",
    )
    front_panel.visual(
        Cylinder(radius=0.0045, length=0.018),
        origin=Origin(xyz=(-0.120, 0.027, 0.072), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="pull_post_left",
    )
    front_panel.visual(
        Cylinder(radius=0.0045, length=0.018),
        origin=Origin(xyz=(0.120, 0.027, 0.072), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="pull_post_right",
    )
    front_panel.visual(
        Cylinder(radius=0.006, length=0.260),
        origin=Origin(xyz=(0.0, 0.038, 0.072), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="pull_bar",
    )
    front_panel.inertial = Inertial.from_geometry(
        Box((panel_width, 0.050, panel_height)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.025, panel_height * 0.5)),
    )

    model.articulation(
        "desk_to_drawer",
        ArticulationType.PRISMATIC,
        parent=desk,
        child=drawer,
        origin=Origin(xyz=(0.0, 0.020, 0.615)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.35,
            lower=0.0,
            upper=0.21,
        ),
    )
    model.articulation(
        "drawer_to_front_panel",
        ArticulationType.REVOLUTE,
        parent=drawer,
        child=front_panel,
        origin=Origin(xyz=(0.0, 0.195, -0.060)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.4,
            lower=0.0,
            upper=1.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    desk = object_model.get_part("desk")
    drawer = object_model.get_part("drawer")
    front_panel = object_model.get_part("front_panel")
    slide = object_model.get_articulation("desk_to_drawer")
    hinge = object_model.get_articulation("drawer_to_front_panel")

    ctx.expect_gap(
        desk,
        drawer,
        axis="z",
        positive_elem="desktop",
        negative_elem="drawer_runner_left",
        min_gap=0.030,
        max_gap=0.050,
        name="drawer runners stay just below the desktop",
    )
    ctx.expect_gap(
        desk,
        drawer,
        axis="z",
        positive_elem="guide_left",
        negative_elem="drawer_runner_left",
        min_gap=0.002,
        max_gap=0.010,
        name="left runner sits directly beneath the left guide rail",
    )
    ctx.expect_gap(
        desk,
        drawer,
        axis="z",
        positive_elem="guide_right",
        negative_elem="drawer_runner_right",
        min_gap=0.002,
        max_gap=0.010,
        name="right runner sits directly beneath the right guide rail",
    )
    ctx.expect_overlap(
        drawer,
        desk,
        axes="y",
        elem_a="drawer_runner_left",
        elem_b="guide_left",
        min_overlap=0.30,
        name="closed drawer remains deeply engaged in the guide rails",
    )
    ctx.expect_gap(
        front_panel,
        drawer,
        axis="y",
        min_gap=0.0,
        max_gap=0.030,
        positive_elem="front_leaf",
        negative_elem="drawer_hinge_strip",
        name="closed bin front sits just ahead of the drawer hinge strip",
    )
    ctx.expect_overlap(
        front_panel,
        drawer,
        axes="x",
        elem_a="front_leaf",
        elem_b="drawer_hinge_strip",
        min_overlap=0.75,
        name="closed bin front aligns with the drawer opening",
    )

    closed_drawer_pos = ctx.part_world_position(drawer)
    with ctx.pose({slide: 0.21}):
        ctx.expect_overlap(
            drawer,
            desk,
            axes="y",
            elem_a="drawer_runner_left",
            elem_b="guide_left",
            min_overlap=0.18,
            name="extended drawer still retains insertion in the left guide",
        )
        extended_drawer_pos = ctx.part_world_position(drawer)

    ctx.check(
        "drawer extends forward along the desk depth axis",
        closed_drawer_pos is not None
        and extended_drawer_pos is not None
        and extended_drawer_pos[1] > closed_drawer_pos[1] + 0.18,
        details=f"closed={closed_drawer_pos}, extended={extended_drawer_pos}",
    )

    closed_leaf = ctx.part_element_world_aabb(front_panel, elem="front_leaf")
    with ctx.pose({slide: 0.18, hinge: 1.25}):
        opened_leaf = ctx.part_element_world_aabb(front_panel, elem="front_leaf")

    ctx.check(
        "drop-front panel pivots downward and outward",
        closed_leaf is not None
        and opened_leaf is not None
        and opened_leaf[1][1] > closed_leaf[1][1] + 0.07
        and opened_leaf[1][2] < closed_leaf[1][2] - 0.04,
        details=f"closed={closed_leaf}, opened={opened_leaf}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
