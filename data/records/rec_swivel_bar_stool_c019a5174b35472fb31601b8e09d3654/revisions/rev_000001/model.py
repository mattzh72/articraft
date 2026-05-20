from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    boolean_difference,
    mesh_from_geometry,
)


def _ring_band(
    *,
    outer_radius: float,
    inner_radius: float,
    height: float,
    radial_segments: int = 72,
) -> MeshGeometry:
    """Annular plate/tube centered on the local origin and local Z axis."""
    outer = CylinderGeometry(radius=outer_radius, height=height, radial_segments=radial_segments)
    inner = CylinderGeometry(
        radius=inner_radius,
        height=height + 0.006,
        radial_segments=radial_segments,
    )
    return boolean_difference(outer, inner)


def _triangular_gusset() -> MeshGeometry:
    """A welded triangular stiffener plate in local X/Z, with thickness in local Y."""
    profile = [(-0.010, 0.000), (0.205, 0.000), (-0.010, 0.170)]
    return ExtrudeGeometry(profile, 0.014, cap=True, center=True).rotate_x(math.pi / 2.0)


def _add_horizontal_cylinder(part, *, axis: str, center, radius: float, length: float, material, name=None) -> None:
    if axis == "x":
        rpy = (0.0, math.pi / 2.0, 0.0)
    elif axis == "y":
        rpy = (math.pi / 2.0, 0.0, 0.0)
    else:
        rpy = (0.0, 0.0, 0.0)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=rpy),
        material=material,
        name=name,
    )


def _add_bolt_circle(
    part,
    *,
    radius: float,
    z: float,
    count: int,
    bolt_radius: float,
    bolt_height: float,
    material,
    name_prefix: str,
) -> None:
    for index in range(count):
        angle = math.tau * index / count
        part.visual(
            Cylinder(radius=bolt_radius, length=bolt_height),
            origin=Origin(
                xyz=(radius * math.cos(angle), radius * math.sin(angle), z),
            ),
            material=material,
            name=f"{name_prefix}_{index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_safety_swivel_bar_stool")

    dark_steel = model.material("dark_steel", rgba=(0.16, 0.17, 0.18, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.28, 0.30, 0.31, 1.0))
    brushed = model.material("brushed_steel", rgba=(0.58, 0.60, 0.58, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.95, 0.72, 0.06, 1.0))
    safety_red = model.material("safety_red", rgba=(0.82, 0.08, 0.05, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.02, 0.022, 0.020, 1.0))
    warning_black = model.material("warning_black", rgba=(0.01, 0.01, 0.01, 1.0))

    bearing_guard_mesh = mesh_from_geometry(
        _ring_band(outer_radius=0.190, inner_radius=0.162, height=0.086, radial_segments=84),
        "stationary_bearing_guard",
    )
    top_flange_mesh = mesh_from_geometry(
        _ring_band(outer_radius=0.178, inner_radius=0.055, height=0.026, radial_segments=84),
        "top_flange",
    )
    lower_race_mesh = mesh_from_geometry(
        _ring_band(outer_radius=0.158, inner_radius=0.058, height=0.050, radial_segments=84),
        "lower_bearing_race",
    )
    upper_race_mesh = mesh_from_geometry(
        _ring_band(outer_radius=0.158, inner_radius=0.052, height=0.025, radial_segments=84),
        "upper_bearing_race",
    )
    lock_sleeve_mesh = mesh_from_geometry(
        _ring_band(outer_radius=0.033, inner_radius=0.0155, height=0.118, radial_segments=36),
        "lock_sleeve",
    )
    gusset_mesh = mesh_from_geometry(_triangular_gusset(), "triangular_gusset")
    foot_rail_mesh = mesh_from_geometry(TorusGeometry(radius=0.292, tube=0.014, radial_segments=16, tubular_segments=84), "foot_rail_ring")
    seat_lip_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.230, tube=0.010, radial_segments=16, tubular_segments=72),
        "rolled_seat_lip",
    )

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.355, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark_steel,
        name="floor_plate",
    )
    pedestal.visual(
        Cylinder(radius=0.070, length=0.610),
        origin=Origin(xyz=(0.0, 0.0, 0.340)),
        material=gunmetal,
        name="center_post",
    )
    pedestal.visual(
        Cylinder(radius=0.145, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        material=dark_steel,
        name="lower_collar",
    )
    pedestal.visual(
        top_flange_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.658)),
        material=dark_steel,
        name="top_flange",
    )
    pedestal.visual(
        lower_race_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.695)),
        material=brushed,
        name="lower_race",
    )
    pedestal.visual(
        bearing_guard_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.713)),
        material=safety_yellow,
        name="bearing_guard",
    )
    pedestal.visual(
        foot_rail_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
        material=brushed,
        name="foot_rail",
    )

    for angle in (0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0):
        pedestal.visual(
            gusset_mesh,
            origin=Origin(xyz=(0.068 * math.cos(angle), 0.068 * math.sin(angle), 0.035), rpy=(0.0, 0.0, angle)),
            material=gunmetal,
            name=f"base_gusset_{round(angle, 3)}",
        )
        pedestal.visual(
            Box((0.255, 0.040, 0.032)),
            origin=Origin(
                xyz=(0.171 * math.cos(angle), 0.171 * math.sin(angle), 0.300),
                rpy=(0.0, 0.0, angle),
            ),
            material=gunmetal,
            name=f"rail_spoke_{round(angle, 3)}",
        )

    # Fixed hard stops for the swivel tab.  They sit on the stationary flange,
    # not on the rotating seat, so the load path is visible and serviceable.
    stop_radius = 0.226
    tab_rest_angle = math.pi / 2.0
    for label, angle in (("lower_stop", tab_rest_angle - 2.35), ("upper_stop", tab_rest_angle + 2.35)):
        pedestal.visual(
            Box((0.078, 0.058, 0.050)),
            origin=Origin(
                xyz=(stop_radius * math.cos(angle), stop_radius * math.sin(angle), 0.747),
                rpy=(0.0, 0.0, angle),
            ),
            material=safety_red,
            name=label,
        )
        pedestal.visual(
            Box((0.030, 0.020, 0.020)),
            origin=Origin(
                xyz=((stop_radius + 0.044) * math.cos(angle), (stop_radius + 0.044) * math.sin(angle), 0.780),
                rpy=(0.0, 0.0, angle + 0.55),
            ),
            material=warning_black,
            name=f"{label}_stripe",
        )

    # Spring lockout support: hollow guide sleeve, welded web, and a yellow
    # anti-bump cage, all stationary and tied back into the bearing guard.
    pedestal.visual(
        lock_sleeve_mesh,
        origin=Origin(xyz=(0.228, 0.0, 0.735), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed,
        name="lock_sleeve",
    )
    pedestal.visual(
        Box((0.075, 0.044, 0.070)),
        origin=Origin(xyz=(0.205, 0.0, 0.717)),
        material=dark_steel,
        name="lock_bracket_web",
    )
    pedestal.visual(
        Box((0.032, 0.112, 0.018)),
        origin=Origin(xyz=(0.225, 0.0, 0.761)),
        material=safety_yellow,
        name="lock_guard_bridge",
    )
    pedestal.visual(
        Box((0.090, 0.017, 0.018)),
        origin=Origin(xyz=(0.255, -0.054, 0.761)),
        material=safety_yellow,
        name="lock_guard_rail_0",
    )
    pedestal.visual(
        Box((0.090, 0.017, 0.018)),
        origin=Origin(xyz=(0.255, 0.054, 0.761)),
        material=safety_yellow,
        name="lock_guard_rail_1",
    )
    _add_bolt_circle(
        pedestal,
        radius=0.300,
        z=0.038,
        count=8,
        bolt_radius=0.014,
        bolt_height=0.010,
        material=brushed,
        name_prefix="floor_bolt",
    )
    _add_bolt_circle(
        pedestal,
        radius=0.126,
        z=0.674,
        count=8,
        bolt_radius=0.008,
        bolt_height=0.009,
        material=brushed,
        name_prefix="flange_bolt",
    )
    pedestal.inertial = Inertial.from_geometry(
        Cylinder(radius=0.36, length=0.72),
        mass=26.0,
        origin=Origin(xyz=(0.0, 0.0, 0.36)),
    )

    seat_stage = model.part("seat_stage")
    seat_stage.visual(
        Cylinder(radius=0.042, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=brushed,
        name="center_journal",
    )
    seat_stage.visual(
        upper_race_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=brushed,
        name="upper_race",
    )
    seat_stage.visual(
        Cylinder(radius=0.075, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        material=gunmetal,
        name="hub_boss",
    )
    seat_stage.visual(
        Cylinder(radius=0.198, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=dark_steel,
        name="spider_plate",
    )
    seat_stage.visual(
        Box((0.405, 0.052, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material=gunmetal,
        name="seat_crossbar_0",
    )
    seat_stage.visual(
        Box((0.052, 0.405, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material=gunmetal,
        name="seat_crossbar_1",
    )
    seat_stage.visual(
        Cylinder(radius=0.245, length=0.027),
        origin=Origin(xyz=(0.0, 0.0, 0.116)),
        material=dark_steel,
        name="seat_pan",
    )
    seat_stage.visual(
        Cylinder(radius=0.230, length=0.046),
        origin=Origin(xyz=(0.0, 0.0, 0.1525)),
        material=black_rubber,
        name="cushion",
    )
    seat_stage.visual(
        seat_lip_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.139)),
        material=brushed,
        name="rolled_seat_lip",
    )
    seat_stage.visual(
        Box((0.084, 0.044, 0.050)),
        origin=Origin(xyz=(0.0, 0.226, 0.060)),
        material=safety_red,
        name="stop_tab",
    )
    seat_stage.visual(
        Box((0.042, 0.210, 0.030)),
        origin=Origin(xyz=(0.0, 0.120, 0.060)),
        material=safety_red,
        name="stop_tab_arm",
    )
    for angle in (0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0):
        seat_stage.visual(
            Box((0.175, 0.019, 0.050)),
            origin=Origin(
                xyz=(0.118 * math.cos(angle), 0.118 * math.sin(angle), 0.060),
                rpy=(0.0, 0.0, angle + math.pi / 4.0),
            ),
            material=brushed,
            name=f"seat_web_{round(angle, 3)}",
        )
    _add_bolt_circle(
        seat_stage,
        radius=0.160,
        z=0.133,
        count=8,
        bolt_radius=0.007,
        bolt_height=0.007,
        material=brushed,
        name_prefix="seat_bolt",
    )
    seat_stage.inertial = Inertial.from_geometry(
        Cylinder(radius=0.26, length=0.19),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
    )

    lock_pin = model.part("lock_pin")
    _add_horizontal_cylinder(
        lock_pin,
        axis="x",
        center=(0.0, 0.0, 0.0),
        radius=0.010,
        length=0.250,
        material=brushed,
        name="lock_pin_shaft",
    )
    _add_horizontal_cylinder(
        lock_pin,
        axis="x",
        center=(0.075, 0.0, 0.0),
        radius=0.018,
        length=0.026,
        material=gunmetal,
        name="spring_collar",
    )
    _add_horizontal_cylinder(
        lock_pin,
        axis="x",
        center=(0.113, 0.0, 0.0),
        radius=0.031,
        length=0.026,
        material=safety_red,
        name="pull_knob",
    )
    lock_pin.visual(
        Box((0.020, 0.088, 0.016)),
        origin=Origin(xyz=(0.133, 0.0, 0.0)),
        material=safety_red,
        name="pull_tab",
    )
    lock_pin.inertial = Inertial.from_geometry(
        Cylinder(radius=0.035, length=0.20),
        mass=0.45,
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
    )

    model.articulation(
        "seat_swivel",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=seat_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.720)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.2, lower=-2.35, upper=2.35),
    )
    model.articulation(
        "lock_pull",
        ArticulationType.PRISMATIC,
        parent=pedestal,
        child=lock_pin,
        origin=Origin(xyz=(0.228, 0.0, 0.735)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.18, lower=0.0, upper=0.035),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    seat_stage = object_model.get_part("seat_stage")
    lock_pin = object_model.get_part("lock_pin")
    swivel = object_model.get_articulation("seat_swivel")
    lock_pull = object_model.get_articulation("lock_pull")

    ctx.allow_overlap(
        lock_pin,
        pedestal,
        elem_a="lock_pin_shaft",
        elem_b="lock_bracket_web",
        reason="The lockout pin intentionally passes through a simplified drilled support web.",
    )
    ctx.allow_overlap(
        lock_pin,
        pedestal,
        elem_a="lock_pin_shaft",
        elem_b="bearing_guard",
        reason="The lockout pin passes through a drilled passage in the stationary bearing guard.",
    )
    ctx.allow_overlap(
        lock_pin,
        seat_stage,
        elem_a="lock_pin_shaft",
        elem_b="upper_race",
        reason="At rest the lockout pin seats in the rotating detent ring to prevent unintended swivel.",
    )

    ctx.expect_contact(
        seat_stage,
        pedestal,
        elem_a="upper_race",
        elem_b="lower_race",
        name="upper bearing race sits on supported lower race",
    )
    ctx.expect_overlap(
        seat_stage,
        pedestal,
        axes="xy",
        elem_a="upper_race",
        elem_b="lower_race",
        min_overlap=0.12,
        name="bearing races are concentric",
    )
    ctx.expect_within(
        seat_stage,
        pedestal,
        axes="xy",
        inner_elem="center_journal",
        outer_elem="lower_race",
        margin=0.002,
        name="journal remains inside lower bearing footprint",
    )
    ctx.expect_overlap(
        lock_pin,
        pedestal,
        axes="x",
        elem_a="lock_pin_shaft",
        elem_b="lock_sleeve",
        min_overlap=0.08,
        name="lock pin is retained inside its guide sleeve",
    )
    ctx.expect_overlap(
        lock_pin,
        pedestal,
        axes="x",
        elem_a="lock_pin_shaft",
        elem_b="lock_bracket_web",
        min_overlap=0.04,
        name="lock pin crosses the drilled support web",
    )
    ctx.expect_overlap(
        lock_pin,
        pedestal,
        axes="x",
        elem_a="lock_pin_shaft",
        elem_b="bearing_guard",
        min_overlap=0.025,
        name="lock pin crosses the stationary guard passage",
    )
    ctx.expect_within(
        lock_pin,
        pedestal,
        axes="yz",
        inner_elem="lock_pin_shaft",
        outer_elem="lock_bracket_web",
        margin=0.004,
        name="lock pin is centered in support web bore",
    )
    ctx.expect_overlap(
        lock_pin,
        seat_stage,
        axes="x",
        elem_a="lock_pin_shaft",
        elem_b="upper_race",
        min_overlap=0.015,
        name="lock pin engages the detent ring",
    )
    ctx.expect_within(
        lock_pin,
        pedestal,
        axes="yz",
        inner_elem="lock_pin_shaft",
        outer_elem="lock_sleeve",
        margin=0.004,
        name="lock pin is centered in guide sleeve",
    )

    rest_lock = ctx.part_world_position(lock_pin)
    with ctx.pose({lock_pull: 0.035}):
        pulled_lock = ctx.part_world_position(lock_pin)
    ctx.check(
        "lockout pull moves outward",
        rest_lock is not None and pulled_lock is not None and pulled_lock[0] > rest_lock[0] + 0.030,
        details=f"rest={rest_lock}, pulled={pulled_lock}",
    )

    def _element_center(part, elem_name: str):
        aabb = ctx.part_element_world_aabb(part, elem=elem_name)
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    with ctx.pose({swivel: 2.35}):
        tab = _element_center(seat_stage, "stop_tab")
        stop = _element_center(pedestal, "upper_stop")
        near_upper = tab is not None and stop is not None and math.hypot(tab[0] - stop[0], tab[1] - stop[1]) < 0.055
    ctx.check(
        "upper over-travel stop aligns with tab",
        near_upper,
        details=f"tab={tab}, stop={stop}",
    )

    with ctx.pose({swivel: -2.35}):
        tab = _element_center(seat_stage, "stop_tab")
        stop = _element_center(pedestal, "lower_stop")
        near_lower = tab is not None and stop is not None and math.hypot(tab[0] - stop[0], tab[1] - stop[1]) < 0.055
    ctx.check(
        "lower over-travel stop aligns with tab",
        near_lower,
        details=f"tab={tab}, stop={stop}",
    )

    return ctx.report()


object_model = build_object_model()
