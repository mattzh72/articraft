from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BAR_THICKNESS = 0.016
BAR_HALF_WIDTH = 0.030
HOLE_RADIUS = 0.014
PIN_RADIUS = 0.009
WASHER_RADIUS = 0.023
WASHER_THICKNESS = 0.006
WASHER_GAP = 0.001
LAYER_STEP = 0.029


def _circle_profile(
    center: tuple[float, float],
    radius: float,
    *,
    segments: int = 32,
    clockwise: bool = False,
) -> list[tuple[float, float]]:
    cx, cy = center
    points = [
        (
            cx + radius * math.cos(2.0 * math.pi * i / segments),
            cy + radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]
    return list(reversed(points)) if clockwise else points


def _capsule_profile(length: float, radius: float, *, segments: int = 18) -> list[tuple[float, float]]:
    """Flat link outline with semicircular eyes centered at x=0 and x=length."""
    points: list[tuple[float, float]] = []
    # Bottom straight into the far-end arc, then top straight into the near-end arc.
    for i in range(segments + 1):
        angle = -math.pi / 2.0 + math.pi * i / segments
        points.append((length + radius * math.cos(angle), radius * math.sin(angle)))
    for i in range(segments + 1):
        angle = math.pi / 2.0 + math.pi * i / segments
        points.append((radius * math.cos(angle), radius * math.sin(angle)))
    return points


def _link_plate_mesh(length: float, name: str):
    outer = _capsule_profile(length, BAR_HALF_WIDTH, segments=20)
    holes = [
        _circle_profile((0.0, 0.0), HOLE_RADIUS, segments=36, clockwise=True),
        _circle_profile((length, 0.0), HOLE_RADIUS, segments=36, clockwise=True),
    ]
    geometry = ExtrudeWithHolesGeometry(outer, holes, BAR_THICKNESS, center=True)
    return mesh_from_geometry(geometry, name)


def _bearing_liner_mesh(name: str):
    # A thin annular bushing: it slightly bites into the link plate hole but
    # leaves a real clearance bore for the steel pivot shaft.
    outer = _circle_profile((0.0, 0.0), HOLE_RADIUS * 1.04, segments=40)
    inner = _circle_profile((0.0, 0.0), PIN_RADIUS + 0.0018, segments=40, clockwise=True)
    geometry = ExtrudeWithHolesGeometry(outer, [inner], BAR_THICKNESS + 0.001, center=True)
    return mesh_from_geometry(geometry, name)


def _add_link_part(
    model: ArticulatedObject,
    *,
    name: str,
    length: float,
    material: Material,
):
    part = model.part(name)
    part.visual(
        _link_plate_mesh(length, f"{name}_plate"),
        material=material,
        name="link_plate",
    )
    return part


def _add_washer(part, *, z: float, material: str, name: str) -> None:
    part.visual(
        Cylinder(radius=WASHER_RADIUS, length=WASHER_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, z)),
        material=material,
        name=name,
    )


def _add_chain_pin_hardware(
    part,
    *,
    parent_bar_z: float | None,
    child_bar_z: float | None,
    base_surface_z: float | None = None,
    knob: bool = False,
) -> None:
    """Add a vertical pivot shaft with capture washers around parent/child links."""
    z_extents: list[float] = []
    if base_surface_z is not None:
        # A pedestal flange touches the base frame and carries the first pivot.
        flange_t = 0.008
        part.visual(
            Cylinder(radius=0.033, length=flange_t),
            origin=Origin(xyz=(0.0, 0.0, base_surface_z + flange_t / 2.0)),
            material="pin_steel",
            name="base_flange",
        )
        z_extents.extend([base_surface_z, base_surface_z + flange_t])

    def add_bar_capture(zc: float, prefix: str) -> None:
        below = zc - BAR_THICKNESS / 2.0 - WASHER_GAP - WASHER_THICKNESS / 2.0
        above = zc + BAR_THICKNESS / 2.0 + WASHER_GAP + WASHER_THICKNESS / 2.0
        _add_washer(part, z=below, material="pin_steel", name=f"{prefix}_bottom_washer")
        _add_washer(part, z=above, material="pin_steel", name=f"{prefix}_top_washer")
        z_extents.extend(
            [
                below - WASHER_THICKNESS / 2.0,
                below + WASHER_THICKNESS / 2.0,
                above - WASHER_THICKNESS / 2.0,
                above + WASHER_THICKNESS / 2.0,
            ]
        )

    if parent_bar_z is not None:
        add_bar_capture(parent_bar_z, "parent")
    if child_bar_z is not None:
        add_bar_capture(child_bar_z, "child")

    if knob:
        knob_center = (child_bar_z or parent_bar_z or 0.0) + BAR_THICKNESS / 2.0 + 0.026
        part.visual(
            Cylinder(radius=0.019, length=0.022),
            origin=Origin(xyz=(0.0, 0.0, knob_center)),
            material="handle_red",
            name="tip_knob",
        )
        z_extents.extend([knob_center - 0.011, knob_center + 0.011])

    low = min(z_extents) if z_extents else -0.01
    high = max(z_extents) if z_extents else 0.01
    # One continuous shaft intersects every washer/flange but clears the link holes.
    part.visual(
        Cylinder(radius=PIN_RADIUS, length=high - low),
        origin=Origin(xyz=(0.0, 0.0, (high + low) / 2.0)),
        material="pin_steel",
        name="pivot_shaft",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="serial_lever_chain_study")

    model.material("base_blue", rgba=(0.08, 0.12, 0.18, 1.0))
    model.material("rail_dark", rgba=(0.03, 0.035, 0.04, 1.0))
    model.material("pin_steel", rgba=(0.68, 0.70, 0.70, 1.0))
    model.material("bearing_dark", rgba=(0.005, 0.006, 0.007, 1.0))
    model.material("rubber_black", rgba=(0.012, 0.012, 0.014, 1.0))
    model.material("bar_blue", rgba=(0.05, 0.26, 0.78, 1.0))
    model.material("bar_orange", rgba=(0.95, 0.42, 0.07, 1.0))
    model.material("bar_green", rgba=(0.08, 0.50, 0.25, 1.0))
    model.material("bar_yellow", rgba=(0.96, 0.76, 0.12, 1.0))
    model.material("handle_red", rgba=(0.78, 0.05, 0.04, 1.0))

    base = model.part("base_frame")
    base.visual(
        Box((1.20, 0.50, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material="base_blue",
        name="base_plate",
    )
    # Raised side rails and cross ties give the study a real bench frame.
    for y, name in [(-0.205, "rail_0"), (0.205, "rail_1")]:
        base.visual(
            Box((1.12, 0.030, 0.030)),
            origin=Origin(xyz=(0.0, y, 0.040)),
            material="rail_dark",
            name=name,
        )
    for x, name in [(-0.54, "cross_tie_0"), (0.0, "cross_tie_1"), (0.54, "cross_tie_2")]:
        base.visual(
            Box((0.030, 0.44, 0.026)),
            origin=Origin(xyz=(x, 0.0, 0.038)),
            material="rail_dark",
            name=name,
        )
    # Subtle centerline and station ticks, like a mechanical linkage lab fixture.
    base.visual(
        Box((1.05, 0.006, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material="pin_steel",
        name="centerline",
    )
    for i, x in enumerate([-0.45, -0.30, -0.15, 0.0, 0.15, 0.30, 0.45]):
        base.visual(
            Box((0.006, 0.060, 0.0025)),
            origin=Origin(xyz=(x, 0.0, 0.054)),
            material="pin_steel",
            name=f"station_tick_{i}",
        )
    for i, (x, y) in enumerate([(-0.55, -0.22), (-0.55, 0.22), (0.55, -0.22), (0.55, 0.22)]):
        base.visual(
            Cylinder(radius=0.035, length=0.010),
            origin=Origin(xyz=(x, y, -0.005)),
            material="rubber_black",
            name=f"rubber_foot_{i}",
        )

    lengths = [0.30, 0.24, 0.28, 0.20]
    bar_materials = ["bar_blue", "bar_orange", "bar_green", "bar_yellow"]
    bars = [
        _add_link_part(model, name=f"bar_{i}", length=length, material=bar_materials[i])
        for i, length in enumerate(lengths)
    ]

    pivot_0 = model.part("pivot_0")
    base_surface_z = 0.025 - 0.045
    _add_chain_pin_hardware(
        pivot_0,
        parent_bar_z=None,
        child_bar_z=0.0,
        base_surface_z=base_surface_z,
    )

    pivots = [pivot_0]
    # Each internal pin is fixed to the preceding bar and carries the next bar
    # at an alternating height, avoiding plate-on-plate collisions at the eyes.
    layer_deltas = [LAYER_STEP, -LAYER_STEP, LAYER_STEP]
    for i, dz in enumerate(layer_deltas, start=1):
        pivot = model.part(f"pivot_{i}")
        _add_chain_pin_hardware(
            pivot,
            parent_bar_z=-dz,
            child_bar_z=0.0,
        )
        pivots.append(pivot)

    tip_pin = model.part("tip_pin")
    _add_chain_pin_hardware(
        tip_pin,
        parent_bar_z=0.0,
        child_bar_z=None,
        knob=True,
    )

    base_yaw = math.radians(20.0)
    relative_yaws = [math.radians(-35.0), math.radians(45.0), math.radians(-50.0)]

    model.articulation(
        "base_to_pivot_0",
        ArticulationType.FIXED,
        parent=base,
        child=pivot_0,
        origin=Origin(xyz=(-0.44, -0.10, 0.045), rpy=(0.0, 0.0, base_yaw)),
    )
    for i, bar in enumerate(bars):
        model.articulation(
            f"pivot_{i}_to_bar_{i}",
            ArticulationType.REVOLUTE,
            parent=pivots[i],
            child=bar,
            origin=Origin(),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                lower=-1.05,
                upper=1.05,
                effort=8.0,
                velocity=2.5,
            ),
        )
        if i < len(bars) - 1:
            model.articulation(
                f"bar_{i}_to_pivot_{i + 1}",
                ArticulationType.FIXED,
                parent=bar,
                child=pivots[i + 1],
                origin=Origin(
                    xyz=(lengths[i], 0.0, layer_deltas[i]),
                    rpy=(0.0, 0.0, relative_yaws[i]),
                ),
            )
        else:
            model.articulation(
                "bar_3_to_tip_pin",
                ArticulationType.FIXED,
                parent=bar,
                child=tip_pin,
                origin=Origin(xyz=(lengths[i], 0.0, 0.0)),
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    revolute_names = [f"pivot_{i}_to_bar_{i}" for i in range(4)]
    revolute_joints = [object_model.get_articulation(name) for name in revolute_names]

    ctx.check(
        "four serial revolute joints",
        all(j.articulation_type == ArticulationType.REVOLUTE for j in revolute_joints),
        details=str([j.articulation_type for j in revolute_joints]),
    )
    ctx.check(
        "all revolute axes are vertical",
        all(tuple(round(v, 6) for v in j.axis) == (0.0, 0.0, 1.0) for j in revolute_joints),
        details=str([j.axis for j in revolute_joints]),
    )

    base = object_model.get_part("base_frame")
    pivot_0 = object_model.get_part("pivot_0")
    bar_0 = object_model.get_part("bar_0")

    shaft_eye_pairs = [
        ("bar_0", "pivot_0"),
        ("bar_0", "pivot_1"),
        ("bar_1", "pivot_1"),
        ("bar_1", "pivot_2"),
        ("bar_2", "pivot_2"),
        ("bar_2", "pivot_3"),
        ("bar_3", "pivot_3"),
        ("bar_3", "tip_pin"),
    ]
    for bar_name, pivot_name in shaft_eye_pairs:
        ctx.allow_overlap(
            bar_name,
            pivot_name,
            elem_a="link_plate",
            elem_b="pivot_shaft",
            reason=(
                "The steel pivot shaft is intentionally captured through the bored eye "
                "of the lever link; the local shaft-through-hole fit is the revolute bearing."
            ),
        )
        ctx.expect_overlap(
            pivot_name,
            bar_name,
            axes="xy",
            elem_a="pivot_shaft",
            elem_b="link_plate",
            min_overlap=0.015,
            name=f"{pivot_name} shaft is centered in {bar_name} eye",
        )

    ctx.expect_gap(
        pivot_0,
        base,
        axis="z",
        positive_elem="base_flange",
        negative_elem="base_plate",
        max_gap=0.0008,
        max_penetration=0.0,
        name="first pivot flange sits on the base plate",
    )
    ctx.expect_overlap(
        pivot_0,
        bar_0,
        axes="xy",
        elem_a="pivot_shaft",
        elem_b="link_plate",
        min_overlap=0.015,
        name="first shaft passes through the first bar eye",
    )
    ctx.expect_gap(
        pivot_0,
        bar_0,
        axis="z",
        positive_elem="child_top_washer",
        negative_elem="link_plate",
        min_gap=0.0005,
        max_gap=0.002,
        name="upper washer clears the first bar",
    )
    ctx.expect_gap(
        bar_0,
        pivot_0,
        axis="z",
        positive_elem="link_plate",
        negative_elem="child_bottom_washer",
        min_gap=0.0005,
        max_gap=0.002,
        name="lower washer clears the first bar",
    )

    tip_pin = object_model.get_part("tip_pin")
    rest_tip = ctx.part_world_position(tip_pin)
    with ctx.pose({"pivot_0_to_bar_0": 0.45, "pivot_1_to_bar_1": -0.35}):
        moved_tip = ctx.part_world_position(tip_pin)
    ctx.check(
        "serial chain endpoint moves when upstream joints rotate",
        rest_tip is not None
        and moved_tip is not None
        and math.hypot(moved_tip[0] - rest_tip[0], moved_tip[1] - rest_tip[1]) > 0.08,
        details=f"rest={rest_tip}, moved={moved_tip}",
    )

    return ctx.report()


object_model = build_object_model()
