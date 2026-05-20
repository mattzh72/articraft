from __future__ import annotations

import math
from typing import Iterable

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


STAINLESS = Material("brushed_stainless", rgba=(0.72, 0.74, 0.72, 1.0))
DARK_STEEL = Material("dark_steel", rgba=(0.16, 0.17, 0.17, 1.0))
BLACK_RUBBER = Material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
PANEL_BLACK = Material("black_control_face", rgba=(0.03, 0.035, 0.04, 1.0))
RACK_COAT = Material("grey_nylon_coated_wire", rgba=(0.78, 0.80, 0.78, 1.0))
WASH_ARM_BLUE = Material("blue_grey_wash_arm", rgba=(0.30, 0.39, 0.43, 1.0))
AMBER = Material("amber_indicator", rgba=(0.95, 0.55, 0.06, 1.0))


def _cq_box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _washer_shell() -> cq.Workplane:
    """One-piece open-front stainless cabinet with a real empty wash cavity."""

    width = 0.62
    depth = 0.60
    body_height = 0.78
    bottom = 0.08
    wall = 0.035

    outer = _cq_box((width, depth, body_height), (0.0, depth / 2.0, bottom + body_height / 2.0))
    # The cutter deliberately enters from in front of the appliance and stops
    # short of the rear, leaving side/top/bottom/back walls instead of a filled box.
    cut_depth = depth - wall + 0.05
    cut_center_y = (-0.02 + (depth - wall)) / 2.0
    cavity = _cq_box(
        (width - 2.0 * wall, cut_depth, body_height - 2.0 * wall),
        (0.0, cut_center_y, bottom + body_height / 2.0),
    )
    shell = outer.cut(cavity)
    return shell.edges("|Z or |X").fillet(0.006)


def _door_liner() -> cq.Workplane:
    """Shallow hollow tray on the inner door, open toward the wash chamber."""

    width = 0.48
    height = 0.52
    depth = 0.060
    wall = 0.010
    center_z = 0.38
    # Local door coordinates: +Z is upward when closed, +Y faces into chamber.
    outer = _cq_box((width, depth, height), (0.0, 0.025, center_z))
    inner = _cq_box((width - 2 * wall, depth + 0.030, height - 2 * wall), (0.0, 0.046, center_z))
    tray = outer.cut(inner)
    return tray.edges("|Z or |X").fillet(0.004)


def _rod_between(
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
    radius: float,
) -> cq.Solid:
    start = cq.Vector(*p0)
    delta = cq.Vector(p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2])
    length = delta.Length
    if length <= 1e-6:
        raise ValueError("rod endpoints are too close")
    return cq.Solid.makeCylinder(radius, length, start, delta.normalized())


def _fused_rods(rods: Iterable[tuple[tuple[float, float, float], tuple[float, float, float], float]]) -> cq.Shape:
    solids = [_rod_between(a, b, r) for a, b, r in rods]
    result = solids[0]
    for solid in solids[1:]:
        result = result.fuse(solid)
    return result


def _rack_geometry(*, width: float, depth: float, tine_height: float, shallow: bool) -> cq.Shape:
    """Connected wire rack with perimeter, cross wires, runners, and glass tines."""

    x0 = -width / 2.0
    x1 = width / 2.0
    y0 = -depth / 2.0
    y1 = depth / 2.0
    r = 0.0045 if not shallow else 0.0038
    rods: list[tuple[tuple[float, float, float], tuple[float, float, float], float]] = []

    runner_inset = 0.01915 if shallow else 0.0200
    runner_left = x0 + runner_inset
    runner_right = x1 - runner_inset

    # Perimeter rectangle and side runners that sit on the appliance guide rails.
    rods += [
        ((x0 - r, y0, 0.0), (x1 + r, y0, 0.0), r),
        ((x1, y0 - r, 0.0), (x1, y1 + r, 0.0), r),
        ((x1 + r, y1, 0.0), (x0 - r, y1, 0.0), r),
        ((x0, y1 + r, 0.0), (x0, y0 - r, 0.0), r),
        ((runner_left, y0, -0.024), (runner_left, y1, -0.024), r * 1.1),
        ((runner_right, y0, -0.024), (runner_right, y1, -0.024), r * 1.1),
        ((runner_left, y0, -0.024), (runner_left, y0, 0.0), r * 0.85),
        ((runner_left, y1, -0.024), (runner_left, y1, 0.0), r * 0.85),
        ((runner_right, y0, -0.024), (runner_right, y0, 0.0), r * 0.85),
        ((runner_right, y1, -0.024), (runner_right, y1, 0.0), r * 0.85),
    ]

    for i in range(1, 6 if not shallow else 5):
        y = y0 + i * depth / (6 if not shallow else 5)
        rods.append(((x0 - r, y, 0.0), (x1 + r, y, 0.0), r * 0.72))
    for i in range(1, 5 if not shallow else 4):
        x = x0 + i * width / (5 if not shallow else 4)
        rods.append(((x, y0 - r, 0.0), (x, y1 + r, 0.0), r * 0.72))

    # Upright glass tines lean slightly so the rack reads like commercial glassware hardware.
    tine_rows = 2 if shallow else 3
    tine_cols = 4 if shallow else 5
    for row in range(tine_rows):
        y = y0 + 0.095 + row * (depth - 0.190) / max(1, tine_rows - 1)
        for col in range(tine_cols):
            x = x0 + 0.075 + col * (width - 0.150) / max(1, tine_cols - 1)
            rods.append(((x, y, -r), (x + 0.010, y + 0.010, tine_height), r * 0.62))

    basket = _fused_rods(rods)
    # Two low center spines make the wire network a single manufactured rack
    # rather than a collection of separate rods while still reading as open.
    spine_y = _cq_box((0.012, depth + 0.020, 0.010), (0.0, 0.0, 0.000)).val()
    spine_x = _cq_box((width + 0.020, 0.012, 0.010), (0.0, 0.0, 0.000)).val()
    return basket.fuse(spine_y).fuse(spine_x)


def _small_hinge_bar(length: float, radius: float = 0.008) -> cq.Shape:
    return _rod_between((-length / 2.0, 0.0, 0.0), (length / 2.0, 0.0, 0.0), radius)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stainless_underbar_glasswasher")
    for material in (STAINLESS, DARK_STEEL, BLACK_RUBBER, PANEL_BLACK, RACK_COAT, WASH_ARM_BLUE, AMBER):
        model.material(material.name, rgba=material.rgba)

    chassis = model.part("chassis")
    chassis.visual(
        mesh_from_cadquery(_washer_shell(), "open_wash_chamber", tolerance=0.001),
        material=STAINLESS,
        name="hollow_chamber",
    )
    chassis.visual(Box((0.60, 0.55, 0.080)), origin=Origin(xyz=(0.0, 0.30, 0.040)), material=STAINLESS, name="toe_base")
    chassis.visual(Box((0.53, 0.019, 0.020)), origin=Origin(xyz=(0.0, -0.009, 0.103)), material=BLACK_RUBBER, name="door_gasket")

    # Rack guide rails, visibly supported by the chamber side walls.
    for level_name, z, rail_x in (("lower", 0.202, 0.250), ("upper", 0.508, 0.250)):
        for side, x in (("guide_0", -rail_x), ("guide_1", rail_x)):
            rail_len = 0.430 if level_name == "lower" else 0.350
            chassis.visual(
                Box((0.030, rail_len, 0.018)),
                origin=Origin(xyz=(x, 0.320, z)),
                material=STAINLESS,
                name=f"{level_name}_{side}",
            )
            bridge_x = -0.270 if x < 0 else 0.270
            chassis.visual(
                Box((0.010, rail_len, 0.018)),
                origin=Origin(xyz=(bridge_x, 0.320, z)),
                material=STAINLESS,
                name=f"{level_name}_mount_{side}",
            )
    # Rear spray plumbing and central sump/hub bosses support the rotating arms.
    chassis.visual(Cylinder(radius=0.035, length=0.030), origin=Origin(xyz=(0.0, 0.350, 0.130)), material=DARK_STEEL, name="lower_hub_socket")
    chassis.visual(Cylinder(radius=0.028, length=0.030), origin=Origin(xyz=(0.0, 0.350, 0.415)), material=DARK_STEEL, name="upper_hub_socket")
    chassis.visual(Box((0.040, 0.030, 0.285)), origin=Origin(xyz=(0.0, 0.565, 0.285)), material=DARK_STEEL, name="rear_riser_pipe")
    chassis.visual(Box((0.030, 0.220, 0.025)), origin=Origin(xyz=(0.0, 0.455, 0.415)), material=DARK_STEEL, name="upper_feed_pipe")
    for x in (-0.255, 0.255):
        chassis.visual(Box((0.050, 0.060, 0.045)), origin=Origin(xyz=(x, -0.005, 0.05725)), material=STAINLESS, name=f"hinge_bracket_{0 if x < 0 else 1}")

    control_pod = model.part("control_pod")
    control_pod.visual(Box((0.62, 0.090, 0.105)), origin=Origin(xyz=(0.0, -0.045, 0.832)), material=STAINLESS, name="pod_body")
    control_pod.visual(Box((0.46, 0.006, 0.070)), origin=Origin(xyz=(0.030, -0.092, 0.835)), material=PANEL_BLACK, name="control_face")
    control_pod.visual(Box((0.075, 0.007, 0.018)), origin=Origin(xyz=(0.215, -0.096, 0.868)), material=AMBER, name="pilot_light")
    model.articulation("chassis_to_control_pod", ArticulationType.FIXED, parent=chassis, child=control_pod, origin=Origin())

    door = model.part("door")
    door.visual(Box((0.600, 0.050, 0.690)), origin=Origin(xyz=(0.0, -0.027, 0.345)), material=STAINLESS, name="outer_slab")
    door.visual(
        mesh_from_cadquery(_door_liner(), "door_liner", tolerance=0.001),
        material=STAINLESS,
        name="hollow_liner",
    )
    door.visual(
        mesh_from_cadquery(_small_hinge_bar(0.405, 0.017), "bottom_hinge_bar", tolerance=0.001),
        material=DARK_STEEL,
        name="hinge_bar",
    )
    door.visual(Cylinder(radius=0.016, length=0.430), origin=Origin(xyz=(0.0, -0.094, 0.560), rpy=(0.0, math.pi / 2.0, 0.0)), material=DARK_STEEL, name="front_handle")
    for x in (-0.170, 0.170):
        door.visual(Box((0.035, 0.046, 0.060)), origin=Origin(xyz=(x, -0.069, 0.560)), material=DARK_STEEL, name=f"handle_post_{0 if x < 0 else 1}")
    door.visual(Box((0.175, 0.060, 0.010)), origin=Origin(xyz=(-0.120, 0.030, 0.300)), material=DARK_STEEL, name="detergent_mount_web")
    door.visual(Box((0.175, 0.014, 0.018)), origin=Origin(xyz=(-0.120, 0.062, 0.300)), material=DARK_STEEL, name="detergent_hinge_boss")
    door_hinge = model.articulation(
        "chassis_to_door",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=door,
        origin=Origin(xyz=(0.0, -0.025, 0.080)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.0, lower=0.0, upper=1.70),
    )

    detergent_flap = model.part("detergent_flap")
    detergent_flap.visual(Box((0.165, 0.012, 0.090)), origin=Origin(xyz=(0.0, 0.006, 0.045)), material=PANEL_BLACK, name="flap_panel")
    detergent_flap.visual(Cylinder(radius=0.006, length=0.165), origin=Origin(xyz=(0.0, 0.001, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)), material=DARK_STEEL, name="flap_pin")
    model.articulation(
        "door_to_detergent_flap",
        ArticulationType.REVOLUTE,
        parent=door,
        child=detergent_flap,
        origin=Origin(xyz=(-0.120, 0.074, 0.300)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.5, lower=0.0, upper=1.25),
    )

    lower_rack = model.part("lower_rack")
    lower_rack.visual(
        mesh_from_cadquery(_rack_geometry(width=0.500, depth=0.410, tine_height=0.120, shallow=False), "lower_glass_rack", tolerance=0.0012),
        material=RACK_COAT,
        name="wire_basket",
    )
    lower_rack.visual(Box((0.520, 0.018, 0.012)), origin=Origin(xyz=(0.0, 0.0, 0.0)), material=RACK_COAT, name="cross_spine")
    lower_rack.visual(Box((0.018, 0.430, 0.012)), origin=Origin(xyz=(0.0, 0.0, 0.0)), material=RACK_COAT, name="center_spine")
    lower_rack.visual(Box((0.485, 0.395, 0.006)), origin=Origin(xyz=(0.0, 0.0, -0.001)), material=RACK_COAT, name="welded_floor_grid")
    lower_slide = model.articulation(
        "chassis_to_lower_rack",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=lower_rack,
        origin=Origin(xyz=(0.0, 0.335, 0.240)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=0.35, lower=0.0, upper=0.310),
    )

    upper_rack = model.part("upper_rack")
    upper_rack.visual(
        mesh_from_cadquery(_rack_geometry(width=0.500, depth=0.320, tine_height=0.080, shallow=True), "upper_glass_rack", tolerance=0.0012),
        material=RACK_COAT,
        name="shallow_basket",
    )
    upper_rack.visual(Box((0.520, 0.016, 0.010)), origin=Origin(xyz=(0.0, 0.0, 0.0)), material=RACK_COAT, name="cross_spine")
    upper_rack.visual(Box((0.016, 0.340, 0.010)), origin=Origin(xyz=(0.0, 0.0, 0.0)), material=RACK_COAT, name="center_spine")
    upper_rack.visual(Box((0.485, 0.305, 0.005)), origin=Origin(xyz=(0.0, 0.0, -0.001)), material=RACK_COAT, name="shallow_floor_grid")
    for i, (x, y) in enumerate(((-0.231, -0.115), (-0.231, 0.115), (0.231, -0.115), (0.231, 0.115))):
        upper_rack.visual(
            Box((0.014, 0.032, 0.012)),
            origin=Origin(xyz=(x, y, -0.022)),
            material=RACK_COAT,
            name=f"upper_roller_{i}",
        )
    upper_slide = model.articulation(
        "chassis_to_upper_rack",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=upper_rack,
        origin=Origin(xyz=(0.0, 0.320, 0.545)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.30, lower=0.0, upper=0.240),
    )

    for name, z, length, width in (("lower_wash_arm", 0.145, 0.430, 0.038), ("upper_wash_arm", 0.430, 0.360, 0.032)):
        arm = model.part(name)
        arm.visual(Cylinder(radius=0.030 if name == "lower_wash_arm" else 0.025, length=0.035), origin=Origin(xyz=(0.0, 0.0, 0.0175)), material=WASH_ARM_BLUE, name="vertical_hub")
        arm.visual(Box((length, width, 0.014)), origin=Origin(xyz=(0.0, 0.0, 0.022)), material=WASH_ARM_BLUE, name="spray_bar")
        arm.visual(Box((0.045, width * 0.75, 0.018)), origin=Origin(xyz=(length / 2.0 - 0.030, 0.0, 0.024), rpy=(0.0, 0.0, 0.25)), material=WASH_ARM_BLUE, name="swept_tip_0")
        arm.visual(Box((0.045, width * 0.75, 0.018)), origin=Origin(xyz=(-length / 2.0 + 0.030, 0.0, 0.024), rpy=(0.0, 0.0, 0.25)), material=WASH_ARM_BLUE, name="swept_tip_1")
        for i, x in enumerate((-length * 0.32, -length * 0.12, length * 0.12, length * 0.32)):
            arm.visual(Cylinder(radius=0.004, length=0.006), origin=Origin(xyz=(x, width * (0.20 if i % 2 == 0 else -0.20), 0.029)), material=DARK_STEEL, name=f"spray_nozzle_{i}")
        model.articulation(
            f"chassis_to_{name}",
            ArticulationType.CONTINUOUS,
            parent=chassis,
            child=arm,
            origin=Origin(xyz=(0.0, 0.350, z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=3.0, velocity=12.0),
        )

    timer_knob = model.part("timer_knob")
    timer_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.058,
                0.030,
                body_style="skirted",
                top_diameter=0.045,
                skirt=KnobSkirt(0.070, 0.006, flare=0.05, chamfer=0.001),
                grip=KnobGrip(style="fluted", count=22, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=30.0),
                center=False,
            ),
            "timer_knob",
        ),
        material=DARK_STEEL,
        name="knob_cap",
    )
    model.articulation(
        "control_pod_to_timer_knob",
        ArticulationType.CONTINUOUS,
        parent=control_pod,
        child=timer_knob,
        origin=Origin(xyz=(-0.135, -0.095, 0.835), rpy=(math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=4.0),
    )

    for idx, x in enumerate((0.030, 0.105)):
        rocker = model.part(f"rocker_{idx}")
        rocker.visual(Box((0.050, 0.012, 0.034)), origin=Origin(xyz=(0.0, -0.006, 0.0)), material=PANEL_BLACK, name="rocker_paddle")
        rocker.visual(Box((0.038, 0.004, 0.006)), origin=Origin(xyz=(0.0, -0.013, 0.012)), material=AMBER if idx == 0 else DARK_STEEL, name="rocker_mark")
        model.articulation(
            f"control_pod_to_rocker_{idx}",
            ArticulationType.REVOLUTE,
            parent=control_pod,
            child=rocker,
            origin=Origin(xyz=(x, -0.095, 0.835)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.7, velocity=8.0, lower=-0.24, upper=0.24),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    door = object_model.get_part("door")
    lower_rack = object_model.get_part("lower_rack")
    upper_rack = object_model.get_part("upper_rack")
    lower_arm = object_model.get_part("lower_wash_arm")
    upper_arm = object_model.get_part("upper_wash_arm")
    timer = object_model.get_part("timer_knob")
    chassis = object_model.get_part("chassis")

    door_hinge = object_model.get_articulation("chassis_to_door")
    lower_slide = object_model.get_articulation("chassis_to_lower_rack")
    upper_slide = object_model.get_articulation("chassis_to_upper_rack")

    ctx.expect_within(lower_rack, chassis, axes="x", margin=0.010, name="lower rack fits between side guides")
    ctx.expect_within(upper_rack, chassis, axes="x", margin=0.010, name="upper rack fits between side guides")
    ctx.expect_gap(lower_rack, lower_arm, axis="z", min_gap=0.020, name="lower wash arm clears lower rack")
    ctx.expect_gap(upper_rack, upper_arm, axis="z", min_gap=0.040, name="upper wash arm clears upper rack")
    ctx.expect_contact(timer, "control_pod", elem_a="knob_cap", elem_b="control_face", contact_tol=0.004, name="timer knob seats on pod face")

    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.35}):
        opened_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "dropdown door opens outward and downward",
        closed_aabb is not None
        and opened_aabb is not None
        and opened_aabb[0][1] < closed_aabb[0][1] - 0.25
        and opened_aabb[1][2] < closed_aabb[1][2] - 0.20,
        details=f"closed={closed_aabb}, opened={opened_aabb}",
    )

    lower_rest = ctx.part_world_position(lower_rack)
    with ctx.pose({lower_slide: 0.250}):
        lower_extended = ctx.part_world_position(lower_rack)
        ctx.expect_overlap(lower_rack, chassis, axes="y", min_overlap=0.070, name="lower rack remains retained on guides")
    upper_rest = ctx.part_world_position(upper_rack)
    with ctx.pose({upper_slide: 0.200}):
        upper_extended = ctx.part_world_position(upper_rack)
        ctx.expect_overlap(upper_rack, chassis, axes="y", min_overlap=0.060, name="upper rack remains retained on guides")

    ctx.check(
        "rack slides move out the front",
        lower_rest is not None
        and lower_extended is not None
        and upper_rest is not None
        and upper_extended is not None
        and lower_extended[1] < lower_rest[1] - 0.20
        and upper_extended[1] < upper_rest[1] - 0.16,
        details=f"lower {lower_rest}->{lower_extended}, upper {upper_rest}->{upper_extended}",
    )

    for joint_name in (
        "chassis_to_lower_wash_arm",
        "chassis_to_upper_wash_arm",
        "control_pod_to_timer_knob",
    ):
        joint = object_model.get_articulation(joint_name)
        ctx.check(f"{joint_name} is continuous", joint.articulation_type == ArticulationType.CONTINUOUS, details=str(joint.articulation_type))

    for joint_name in ("control_pod_to_rocker_0", "control_pod_to_rocker_1", "door_to_detergent_flap"):
        joint = object_model.get_articulation(joint_name)
        ctx.check(f"{joint_name} has independent revolute limits", joint.motion_limits is not None and joint.motion_limits.lower is not None and joint.motion_limits.upper is not None, details=str(joint.motion_limits))

    return ctx.report()


object_model = build_object_model()
