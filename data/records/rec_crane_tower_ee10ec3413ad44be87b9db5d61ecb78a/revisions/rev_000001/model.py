from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _midpoint(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.hypot(dx, dy), dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _build_hook_mesh():
    hook_geom = tube_from_spline_points(
        [
            (0.000, 0.0, -1.335),
            (0.000, 0.0, -1.455),
            (0.040, 0.0, -1.535),
            (0.122, 0.0, -1.555),
            (0.190, 0.0, -1.500),
            (0.188, 0.0, -1.405),
            (0.112, 0.0, -1.374),
            (0.065, 0.0, -1.412),
        ],
        radius=0.020,
        samples_per_segment=14,
        radial_segments=18,
        up_hint=(0.0, 1.0, 0.0),
    )
    return mesh_from_geometry(hook_geom, "gantry_hook")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="overhead_gantry_crane")

    safety_yellow = model.material("safety_yellow", rgba=(0.95, 0.72, 0.10, 1.0))
    blue_steel = model.material("blue_steel", rgba=(0.08, 0.19, 0.34, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.12, 0.13, 0.14, 1.0))
    rail_steel = model.material("rail_steel", rgba=(0.50, 0.52, 0.54, 1.0))
    concrete = model.material("concrete", rgba=(0.58, 0.57, 0.53, 1.0))
    cable_black = model.material("cable_black", rgba=(0.04, 0.04, 0.045, 1.0))
    warning_red = model.material("warning_red", rgba=(0.78, 0.08, 0.04, 1.0))

    hook_mesh = _build_hook_mesh()

    portal = model.part("portal_frame")
    # Concrete pads and floor rails tie the two runway lines into a single grounded frame.
    portal.visual(Box((7.60, 0.32, 0.08)), origin=Origin(xyz=(0.0, -2.55, 0.04)), material=concrete, name="foundation_0")
    portal.visual(Box((7.60, 0.32, 0.08)), origin=Origin(xyz=(0.0, 2.55, 0.04)), material=concrete, name="foundation_1")
    portal.visual(Box((0.22, 5.42, 0.08)), origin=Origin(xyz=(-3.70, 0.0, 0.04)), material=concrete, name="foundation_tie_0")
    portal.visual(Box((0.22, 5.42, 0.08)), origin=Origin(xyz=(3.70, 0.0, 0.04)), material=concrete, name="foundation_tie_1")
    portal.visual(Box((7.42, 0.10, 0.10)), origin=Origin(xyz=(0.0, -2.55, 0.13)), material=rail_steel, name="floor_rail_0")
    portal.visual(Box((7.42, 0.10, 0.10)), origin=Origin(xyz=(0.0, 2.55, 0.13)), material=rail_steel, name="floor_rail_1")

    for x in (-3.45, 3.45):
        portal.visual(Box((0.42, 0.42, 0.12)), origin=Origin(xyz=(x, -2.55, 0.20)), material=dark_steel, name=f"foot_{x}_0")
        portal.visual(Box((0.42, 0.42, 0.12)), origin=Origin(xyz=(x, 2.55, 0.20)), material=dark_steel, name=f"foot_{x}_1")
        portal.visual(Box((0.26, 0.26, 3.10)), origin=Origin(xyz=(x, -2.55, 1.81)), material=safety_yellow, name=f"leg_{x}_0")
        portal.visual(Box((0.26, 0.26, 3.10)), origin=Origin(xyz=(x, 2.55, 1.81)), material=safety_yellow, name=f"leg_{x}_1")
        portal.visual(Box((0.32, 5.36, 0.28)), origin=Origin(xyz=(x, 0.0, 3.46)), material=safety_yellow, name=f"portal_cross_{x}")
        _add_member(portal, (x, -2.55, 0.42), (x, 2.55, 3.32), 0.045, safety_yellow)
        _add_member(portal, (x, 2.55, 0.42), (x, -2.55, 3.32), 0.045, safety_yellow)

    # Elevated longitudinal runway beams, with narrow steel rails on top for the moving bridge wheels.
    portal.visual(Box((7.30, 0.30, 0.30)), origin=Origin(xyz=(0.0, -2.55, 3.46)), material=safety_yellow, name="runway_beam_0")
    portal.visual(Box((7.30, 0.30, 0.30)), origin=Origin(xyz=(0.0, 2.55, 3.46)), material=safety_yellow, name="runway_beam_1")
    portal.visual(Box((7.20, 0.08, 0.10)), origin=Origin(xyz=(0.0, -2.55, 3.66)), material=rail_steel, name="runway_rail_0")
    portal.visual(Box((7.20, 0.08, 0.10)), origin=Origin(xyz=(0.0, 2.55, 3.66)), material=rail_steel, name="runway_rail_1")
    for y in (-2.55, 2.55):
        _add_member(portal, (-3.45, y, 0.45), (3.45, y, 3.28), 0.035, safety_yellow)
        _add_member(portal, (3.45, y, 0.45), (-3.45, y, 3.28), 0.035, safety_yellow)
    portal.inertial = Inertial.from_geometry(Box((7.60, 5.42, 3.76)), mass=6500.0, origin=Origin(xyz=(0.0, 0.0, 1.88)))

    bridge = model.part("bridge_girder")
    bridge.visual(Box((0.46, 4.86, 0.08)), origin=Origin(xyz=(0.0, 0.0, 0.08)), material=blue_steel, name="bottom_flange")
    bridge.visual(Box((0.16, 4.74, 0.42)), origin=Origin(xyz=(0.0, 0.0, 0.31)), material=blue_steel, name="web")
    bridge.visual(Box((0.42, 4.86, 0.08)), origin=Origin(xyz=(0.0, 0.0, 0.56)), material=blue_steel, name="top_flange")
    bridge.visual(Box((0.72, 0.50, 0.16)), origin=Origin(xyz=(0.0, -2.55, 0.04)), material=dark_steel, name="end_truck_0")
    bridge.visual(Box((0.72, 0.50, 0.16)), origin=Origin(xyz=(0.0, 2.55, 0.04)), material=dark_steel, name="end_truck_1")
    for x, y, wheel_name in [
        (-0.22, -2.55, "bridge_rear_near_wheel"),
        (0.22, -2.55, "bridge_front_near_wheel"),
        (-0.22, 2.55, "bridge_rear_far_wheel"),
        (0.22, 2.55, "bridge_front_far_wheel"),
    ]:
        bridge.visual(
            Cylinder(radius=0.070, length=0.11),
            origin=Origin(xyz=(x, y, 0.030), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=wheel_name,
        )
    bridge.visual(Box((0.30, 4.38, 0.035)), origin=Origin(xyz=(0.0, 0.0, 0.0225)), material=rail_steel, name="trolley_track")
    bridge.inertial = Inertial.from_geometry(Box((0.72, 5.10, 0.66)), mass=2300.0, origin=Origin(xyz=(0.0, 0.0, 0.25)))

    trolley = model.part("crab_trolley")
    trolley.visual(Box((0.66, 0.58, 0.18)), origin=Origin(xyz=(0.0, 0.0, -0.10)), material=dark_steel, name="crab_frame")
    trolley.visual(Box((0.54, 0.34, 0.16)), origin=Origin(xyz=(0.0, 0.0, -0.080)), material=safety_yellow, name="machinery_cover")
    trolley.visual(Box((0.74, 0.06, 0.13)), origin=Origin(xyz=(0.0, -0.24, -0.065)), material=dark_steel, name="side_plate_0")
    trolley.visual(Box((0.74, 0.06, 0.13)), origin=Origin(xyz=(0.0, 0.24, -0.065)), material=dark_steel, name="side_plate_1")
    for x, y, wheel_name in [
        (-0.24, -0.21, "trolley_rear_near_wheel"),
        (0.24, -0.21, "trolley_front_near_wheel"),
        (-0.24, 0.21, "trolley_rear_far_wheel"),
        (0.24, 0.21, "trolley_front_far_wheel"),
    ]:
        trolley.visual(
            Cylinder(radius=0.045, length=0.080),
            origin=Origin(xyz=(x, y, -0.005), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rail_steel,
            name=wheel_name,
        )
    trolley.visual(
        Cylinder(radius=0.090, length=0.42),
        origin=Origin(xyz=(0.0, 0.0, -0.115), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rail_steel,
        name="hoist_drum",
    )
    trolley.inertial = Inertial.from_geometry(Box((0.78, 0.62, 0.28)), mass=900.0, origin=Origin(xyz=(0.0, 0.0, -0.07)))

    hook = model.part("hook_block")
    hook.visual(Box((0.34, 0.08, 0.06)), origin=Origin(xyz=(0.0, 0.0, 0.0)), material=dark_steel, name="top_spreader")
    for x in (-0.115, 0.115):
        for y in (-0.028, 0.028):
            hook.visual(Cylinder(radius=0.010, length=1.18), origin=Origin(xyz=(x, y, -0.60)), material=cable_black, name=f"suspension_rope_{x}_{y}")
    hook.visual(Box((0.42, 0.18, 0.16)), origin=Origin(xyz=(0.0, 0.0, -1.21)), material=safety_yellow, name="lower_sheave_case")
    hook.visual(
        Cylinder(radius=0.070, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, -1.21), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rail_steel,
        name="sheave_pin",
    )
    hook.visual(Box((0.10, 0.08, 0.12)), origin=Origin(xyz=(0.0, 0.0, -1.31)), material=dark_steel, name="hook_shank")
    hook.visual(hook_mesh, origin=Origin(), material=warning_red, name="j_hook")
    hook.inertial = Inertial.from_geometry(Box((0.46, 0.22, 1.58)), mass=420.0, origin=Origin(xyz=(0.0, 0.0, -0.76)))

    model.articulation(
        "bridge_travel",
        ArticulationType.PRISMATIC,
        parent=portal,
        child=bridge,
        origin=Origin(xyz=(-2.40, 0.0, 3.75)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12000.0, velocity=0.50, lower=0.0, upper=4.80),
    )
    model.articulation(
        "trolley_travel",
        ArticulationType.PRISMATIC,
        parent=bridge,
        child=trolley,
        origin=Origin(xyz=(0.0, -1.75, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3500.0, velocity=0.65, lower=0.0, upper=3.50),
    )
    model.articulation(
        "hook_suspension",
        ArticulationType.FIXED,
        parent=trolley,
        child=hook,
        origin=Origin(xyz=(0.0, 0.0, -0.235)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    portal = object_model.get_part("portal_frame")
    bridge = object_model.get_part("bridge_girder")
    trolley = object_model.get_part("crab_trolley")
    hook = object_model.get_part("hook_block")
    bridge_joint = object_model.get_articulation("bridge_travel")
    trolley_joint = object_model.get_articulation("trolley_travel")

    ctx.expect_gap(
        bridge,
        portal,
        axis="z",
        positive_elem="bridge_rear_near_wheel",
        negative_elem="runway_rail_0",
        max_gap=0.002,
        max_penetration=0.0,
        name="bridge wheel sits on runway rail",
    )
    ctx.expect_gap(
        bridge,
        trolley,
        axis="z",
        positive_elem="bottom_flange",
        negative_elem="trolley_rear_near_wheel",
        max_gap=0.002,
        max_penetration=0.0,
        name="crab trolley wheel bears under bridge girder",
    )
    ctx.expect_gap(
        trolley,
        hook,
        axis="z",
        positive_elem="hoist_drum",
        negative_elem="top_spreader",
        max_gap=0.002,
        max_penetration=0.0,
        name="fixed hook suspension hangs below hoist drum",
    )

    bridge_rest = ctx.part_world_position(bridge)
    with ctx.pose({bridge_joint: 4.80}):
        bridge_far = ctx.part_world_position(bridge)
        ctx.expect_gap(
            bridge,
            portal,
            axis="z",
            positive_elem="bridge_rear_near_wheel",
            negative_elem="runway_rail_0",
            max_gap=0.002,
            max_penetration=0.0,
            name="bridge remains supported at far travel",
        )
    ctx.check(
        "bridge travels along portal rails",
        bridge_rest is not None and bridge_far is not None and bridge_far[0] > bridge_rest[0] + 4.7,
        details=f"rest={bridge_rest}, far={bridge_far}",
    )

    trolley_rest = ctx.part_world_position(trolley)
    with ctx.pose({trolley_joint: 3.50}):
        trolley_far = ctx.part_world_position(trolley)
        ctx.expect_gap(
            bridge,
            trolley,
            axis="z",
            positive_elem="bottom_flange",
            negative_elem="trolley_rear_near_wheel",
            max_gap=0.002,
            max_penetration=0.0,
            name="crab trolley remains on bridge at far travel",
        )
    ctx.check(
        "crab trolley slides along bridge girder",
        trolley_rest is not None and trolley_far is not None and trolley_far[1] > trolley_rest[1] + 3.4,
        details=f"rest={trolley_rest}, far={trolley_far}",
    )

    return ctx.report()


object_model = build_object_model()
