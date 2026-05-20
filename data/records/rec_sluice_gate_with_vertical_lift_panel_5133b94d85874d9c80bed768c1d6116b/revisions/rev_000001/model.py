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
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _aabb_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    low, high = aabb
    return (
        (low[0] + high[0]) * 0.5,
        (low[1] + high[1]) * 0.5,
        (low[2] + high[2]) * 0.5,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stormwater_sluice_gate")

    frame_steel = model.material("frame_steel", rgba=(0.30, 0.32, 0.34, 1.0))
    gate_steel = model.material("gate_steel", rgba=(0.47, 0.49, 0.50, 1.0))
    gearbox_paint = model.material("gearbox_paint", rgba=(0.23, 0.25, 0.27, 1.0))
    wheel_iron = model.material("wheel_iron", rgba=(0.18, 0.19, 0.20, 1.0))
    access_door = model.material("access_door", rgba=(0.36, 0.39, 0.41, 1.0))
    hardware = model.material("hardware", rgba=(0.60, 0.62, 0.64, 1.0))

    opening_width = 1.20
    opening_height = 1.40
    frame_depth = 0.30
    jamb_width = 0.22
    top_beam_height = 0.22
    sill_height = 0.16
    total_frame_width = opening_width + 2.0 * jamb_width
    total_frame_height = 3.60

    panel_width = 1.096
    panel_thickness = 0.058
    panel_height = 1.82

    guide_height = 3.30
    guide_back_width = 0.10
    guide_back_depth = 0.06
    guide_back_y = -0.061
    keeper_width = 0.05
    keeper_depth = 0.04
    keeper_y = 0.051
    guide_center_x = opening_width * 0.5

    frame = model.part("frame")
    frame.visual(
        Box((jamb_width, frame_depth, total_frame_height)),
        origin=Origin(xyz=(-(opening_width * 0.5 + jamb_width * 0.5), 0.0, 1.64)),
        material=frame_steel,
        name="left_jamb",
    )
    frame.visual(
        Box((jamb_width, frame_depth, total_frame_height)),
        origin=Origin(xyz=((opening_width * 0.5 + jamb_width * 0.5), 0.0, 1.64)),
        material=frame_steel,
        name="right_jamb",
    )
    frame.visual(
        Box((total_frame_width, 0.14, top_beam_height)),
        origin=Origin(xyz=(0.0, -0.10, opening_height + top_beam_height * 0.5)),
        material=frame_steel,
        name="top_beam",
    )
    frame.visual(
        Box((total_frame_width, frame_depth, sill_height)),
        origin=Origin(xyz=(0.0, 0.0, -sill_height * 0.5)),
        material=frame_steel,
        name="sill",
    )
    frame.visual(
        Box((guide_back_width, guide_back_depth, guide_height)),
        origin=Origin(xyz=(-guide_center_x, guide_back_y, guide_height * 0.5)),
        material=frame_steel,
        name="left_guide_back",
    )
    frame.visual(
        Box((guide_back_width, guide_back_depth, guide_height)),
        origin=Origin(xyz=(guide_center_x, guide_back_y, guide_height * 0.5)),
        material=frame_steel,
        name="right_guide_back",
    )
    frame.visual(
        Box((keeper_width, keeper_depth, guide_height)),
        origin=Origin(xyz=(-0.575, keeper_y, guide_height * 0.5)),
        material=frame_steel,
        name="left_keeper",
    )
    frame.visual(
        Box((keeper_width, keeper_depth, guide_height)),
        origin=Origin(xyz=(0.575, keeper_y, guide_height * 0.5)),
        material=frame_steel,
        name="right_keeper",
    )
    frame.visual(
        Box((1.06, 0.06, 0.10)),
        origin=Origin(xyz=(0.0, -0.12, opening_height + 0.12)),
        material=frame_steel,
        name="head_cover",
    )
    frame.visual(
        Box((0.34, 0.10, 0.06)),
        origin=Origin(xyz=(0.45, 0.10, opening_height + top_beam_height + 0.03)),
        material=frame_steel,
        name="gearbox_shelf",
    )

    panel = model.part("panel")
    panel.visual(
        Box((panel_width, panel_thickness, panel_height)),
        origin=Origin(xyz=(0.0, 0.0, panel_height * 0.5)),
        material=gate_steel,
        name="leaf",
    )
    panel.visual(
        Box((1.04, 0.10, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        material=gate_steel,
        name="bottom_stiffener",
    )
    panel.visual(
        Box((1.04, 0.10, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, panel_height - 0.07)),
        material=gate_steel,
        name="top_stiffener",
    )
    for rib_name, rib_x in (("rib_outer_0", -0.26), ("rib_center", 0.0), ("rib_outer_1", 0.26)):
        panel.visual(
            Box((0.09, 0.09, 1.18)),
            origin=Origin(xyz=(rib_x, 0.0, 0.71)),
            material=gate_steel,
            name=rib_name,
        )

    gearbox = model.part("gearbox")
    gearbox.visual(
        Box((0.24, 0.14, 0.04)),
        origin=Origin(xyz=(0.0, 0.07, 0.02)),
        material=gearbox_paint,
        name="mount_plate",
    )
    gearbox.visual(
        Box((0.04, 0.12, 0.18)),
        origin=Origin(xyz=(-0.09, 0.06, 0.09)),
        material=gearbox_paint,
        name="mount_cheek_0",
    )
    gearbox.visual(
        Box((0.04, 0.12, 0.18)),
        origin=Origin(xyz=(0.09, 0.06, 0.09)),
        material=gearbox_paint,
        name="mount_cheek_1",
    )
    gearbox.visual(
        Box((0.42, 0.30, 0.32)),
        origin=Origin(xyz=(0.0, 0.22, 0.20)),
        material=gearbox_paint,
        name="housing_box",
    )
    gearbox.visual(
        Box((0.44, 0.32, 0.04)),
        origin=Origin(xyz=(0.0, 0.22, 0.38)),
        material=gearbox_paint,
        name="housing_cap",
    )
    gearbox.visual(
        Cylinder(radius=0.038, length=0.07),
        origin=Origin(xyz=(0.245, 0.22, 0.20), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gearbox_paint,
        name="shaft_boss",
    )
    gearbox.visual(
        Box((0.028, 0.026, 0.22)),
        origin=Origin(xyz=(-0.15, 0.358, 0.19)),
        material=hardware,
        name="hinge_post",
    )
    gearbox.visual(
        Box((0.032, 0.014, 0.028)),
        origin=Origin(xyz=(0.040, 0.357, 0.20)),
        material=hardware,
        name="latch_strike",
    )

    crank_wheel = model.part("crank_wheel")
    crank_wheel.visual(
        Cylinder(radius=0.018, length=0.10),
        origin=Origin(xyz=(0.05, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="shaft",
    )
    crank_wheel.visual(
        Cylinder(radius=0.050, length=0.042),
        origin=Origin(xyz=(0.112, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=wheel_iron,
        name="hub",
    )
    rim_center_x = 0.135
    rim_radius = 0.20
    rim_segments = 12
    rim_points = []
    for index in range(rim_segments):
        angle = 2.0 * math.pi * index / rim_segments
        rim_points.append((rim_center_x, rim_radius * math.cos(angle), rim_radius * math.sin(angle)))
    for index in range(rim_segments):
        _add_member(
            crank_wheel,
            rim_points[index],
            rim_points[(index + 1) % rim_segments],
            0.012,
            wheel_iron,
        )
    for spoke_angle in (0.0, math.pi * 0.5, math.pi, math.pi * 1.5):
        _add_member(
            crank_wheel,
            (rim_center_x, 0.0, 0.0),
            (rim_center_x, rim_radius * math.cos(spoke_angle), rim_radius * math.sin(spoke_angle)),
            0.010,
            wheel_iron,
        )
    crank_wheel.visual(
        Cylinder(radius=0.016, length=0.08),
        origin=Origin(xyz=(0.178, 0.0, rim_radius), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="wheel_grip",
    )

    inspection_door = model.part("inspection_door")
    inspection_door.visual(
        Box((0.17, 0.018, 0.18)),
        origin=Origin(xyz=(0.085, 0.0, 0.09)),
        material=access_door,
        name="door_leaf",
    )
    inspection_door.visual(
        Box((0.13, 0.010, 0.14)),
        origin=Origin(xyz=(0.085, 0.010, 0.09)),
        material=gearbox_paint,
        name="door_stiffener",
    )
    inspection_door.visual(
        Box((0.024, 0.028, 0.040)),
        origin=Origin(xyz=(0.145, 0.015, 0.09)),
        material=hardware,
        name="door_latch",
    )
    inspection_door.visual(
        Cylinder(radius=0.010, length=0.046),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=hardware,
        name="hinge_barrel_0",
    )
    inspection_door.visual(
        Cylinder(radius=0.010, length=0.046),
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        material=hardware,
        name="hinge_barrel_1",
    )

    model.articulation(
        "panel_lift",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=panel,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18000.0, velocity=0.30, lower=0.0, upper=1.48),
    )
    model.articulation(
        "frame_to_gearbox",
        ArticulationType.FIXED,
        parent=frame,
        child=gearbox,
        origin=Origin(xyz=(0.45, frame_depth * 0.5, opening_height + top_beam_height)),
    )
    model.articulation(
        "crank_spin",
        ArticulationType.CONTINUOUS,
        parent=gearbox,
        child=crank_wheel,
        origin=Origin(xyz=(0.280, 0.220, 0.200)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=300.0, velocity=6.0),
    )
    model.articulation(
        "inspection_door_hinge",
        ArticulationType.REVOLUTE,
        parent=gearbox,
        child=inspection_door,
        origin=Origin(xyz=(-0.150, 0.381, 0.100)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=0.0,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    panel = object_model.get_part("panel")
    gearbox = object_model.get_part("gearbox")
    crank_wheel = object_model.get_part("crank_wheel")
    inspection_door = object_model.get_part("inspection_door")

    panel_lift = object_model.get_articulation("panel_lift")
    crank_spin = object_model.get_articulation("crank_spin")
    door_hinge = object_model.get_articulation("inspection_door_hinge")

    lift_limits = panel_lift.motion_limits
    door_limits = door_hinge.motion_limits

    ctx.expect_gap(
        panel,
        frame,
        axis="z",
        positive_elem="leaf",
        negative_elem="sill",
        min_gap=0.0,
        max_gap=0.002,
        name="closed panel seats on sill",
    )
    ctx.expect_gap(
        panel,
        frame,
        axis="x",
        positive_elem="leaf",
        negative_elem="left_guide_back",
        min_gap=0.001,
        max_gap=0.004,
        name="panel clears left guide with tight running fit",
    )
    ctx.expect_gap(
        frame,
        panel,
        axis="x",
        positive_elem="right_guide_back",
        negative_elem="leaf",
        min_gap=0.001,
        max_gap=0.004,
        name="panel clears right guide with tight running fit",
    )
    ctx.expect_gap(
        panel,
        frame,
        axis="y",
        positive_elem="leaf",
        negative_elem="left_guide_back",
        min_gap=0.001,
        max_gap=0.004,
        name="panel clears rear guide shoes",
    )
    ctx.expect_gap(
        frame,
        panel,
        axis="y",
        positive_elem="left_keeper",
        negative_elem="leaf",
        min_gap=0.001,
        max_gap=0.004,
        name="panel clears front keeper lips",
    )
    ctx.expect_gap(
        inspection_door,
        gearbox,
        axis="y",
        positive_elem="door_leaf",
        negative_elem="housing_box",
        min_gap=0.001,
        max_gap=0.004,
        name="inspection door sits just proud of gearbox face",
    )

    closed_grip_center = _aabb_center(ctx.part_element_world_aabb(crank_wheel, elem="wheel_grip"))
    if lift_limits is not None and lift_limits.upper is not None:
        closed_panel_pos = ctx.part_world_position(panel)
        with ctx.pose({panel_lift: lift_limits.upper}):
            ctx.expect_gap(
                panel,
                frame,
                axis="x",
                positive_elem="leaf",
                negative_elem="left_guide_back",
                min_gap=0.001,
                max_gap=0.004,
                name="raised panel stays in left guide channel",
            )
            ctx.expect_gap(
                frame,
                panel,
                axis="x",
                positive_elem="right_guide_back",
                negative_elem="leaf",
                min_gap=0.001,
                max_gap=0.004,
                name="raised panel stays in right guide channel",
            )
            raised_panel_pos = ctx.part_world_position(panel)
        ctx.check(
            "panel lifts clear of the waterway",
            closed_panel_pos is not None
            and raised_panel_pos is not None
            and raised_panel_pos[2] > 1.45
            and raised_panel_pos[2] > closed_panel_pos[2] + 1.40,
            details=f"closed={closed_panel_pos}, raised={raised_panel_pos}",
        )

    if door_limits is not None and door_limits.upper is not None:
        closed_door_aabb = ctx.part_world_aabb(inspection_door)
        with ctx.pose({door_hinge: door_limits.upper}):
            open_door_aabb = ctx.part_world_aabb(inspection_door)
        ctx.check(
            "inspection door swings outward on its hinge",
            closed_door_aabb is not None
            and open_door_aabb is not None
            and open_door_aabb[1][1] > closed_door_aabb[1][1] + 0.10,
            details=f"closed={closed_door_aabb}, open={open_door_aabb}",
        )

    with ctx.pose({crank_spin: math.pi / 2.0}):
        quarter_turn_grip_center = _aabb_center(
            ctx.part_element_world_aabb(crank_wheel, elem="wheel_grip")
        )
    ctx.check(
        "crank wheel grip moves on a circular path",
        closed_grip_center is not None
        and quarter_turn_grip_center is not None
        and abs(quarter_turn_grip_center[1] - closed_grip_center[1]) > 0.12
        and abs(quarter_turn_grip_center[2] - closed_grip_center[2]) > 0.12,
        details=f"closed={closed_grip_center}, quarter_turn={quarter_turn_grip_center}",
    )

    return ctx.report()


object_model = build_object_model()
