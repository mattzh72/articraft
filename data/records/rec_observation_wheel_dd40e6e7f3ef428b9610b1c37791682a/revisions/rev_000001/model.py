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
    TorusGeometry,
    mesh_from_geometry,
)


WHEEL_CENTER_Z = 3.0
OUTER_RADIUS = 2.35
INNER_RADIUS = 1.75
RIM_Y = 0.36
CABIN_COUNT = 8


def _cylinder_origin_between(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> tuple[Origin, float]:
    """Return an Origin that places a local-Z cylinder between two points."""

    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError("Cannot place a cylinder between coincident points.")
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    origin = Origin(
        xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
        rpy=(0.0, pitch, yaw),
    )
    return origin, length


def _add_cylinder_between(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    *,
    material: Material,
    name: str,
) -> None:
    origin, length = _cylinder_origin_between(start, end)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="twin_rim_observation_wheel")

    painted_steel = model.material("painted_steel", rgba=(0.82, 0.84, 0.86, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.10, 0.12, 0.14, 1.0))
    yellow = model.material("warm_cabin", rgba=(0.95, 0.72, 0.18, 1.0))
    dark_roof = model.material("cabin_roof", rgba=(0.16, 0.18, 0.20, 1.0))
    glass = model.material("blue_glass", rgba=(0.25, 0.55, 0.82, 0.55))
    concrete = model.material("concrete", rgba=(0.48, 0.48, 0.45, 1.0))

    # Static A-frame tower and base.  The side legs sit outside the rotating
    # twin rims so the wheel can pass between them on the central axle.
    tower = model.part("tower")
    tower.visual(
        Box((3.25, 1.72, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=concrete,
        name="foundation_slab",
    )
    for y in (-0.72, 0.72):
        tower.visual(
            Box((3.05, 0.12, 0.12)),
            origin=Origin(xyz=(0.0, y, 0.17)),
            material=dark_steel,
            name=f"base_beam_{'rear' if y < 0 else 'front'}",
        )
        _add_cylinder_between(
            tower,
            (-1.35, y, 0.22),
            (0.0, y, WHEEL_CENTER_Z),
            0.055,
            material=painted_steel,
            name=f"leg_a_{'rear' if y < 0 else 'front'}",
        )
        _add_cylinder_between(
            tower,
            (1.35, y, 0.22),
            (0.0, y, WHEEL_CENTER_Z),
            0.055,
            material=painted_steel,
            name=f"leg_b_{'rear' if y < 0 else 'front'}",
        )
        if y < 0:
            tower.visual(
                Box((0.34, 0.12, 0.28)),
                origin=Origin(xyz=(0.0, y, WHEEL_CENTER_Z)),
                material=dark_steel,
                name="bearing_block_rear",
            )
        else:
            tower.visual(
                Box((0.34, 0.12, 0.28)),
                origin=Origin(xyz=(0.0, y, WHEEL_CENTER_Z)),
                material=dark_steel,
                name="bearing_block_front",
            )
    for x in (-1.35, 1.35):
        tower.visual(
            Box((0.16, 1.62, 0.10)),
            origin=Origin(xyz=(x, 0.0, 0.16)),
            material=dark_steel,
            name=f"cross_tie_{'negative' if x < 0 else 'positive'}",
        )
    tower.visual(
        Cylinder(radius=0.095, length=1.56),
        origin=Origin(xyz=(0.0, 0.0, WHEEL_CENTER_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="fixed_axle",
    )

    # Rotating wheel assembly.  The part frame sits on the axle centerline so
    # the articulation origin and visible hub share one horizontal axis.
    wheel = model.part("wheel")
    outer_ring_mesh = mesh_from_geometry(
        TorusGeometry(OUTER_RADIUS, 0.035, radial_segments=18, tubular_segments=96).rotate_x(math.pi / 2.0),
        "outer_rim_ring",
    )
    inner_ring_mesh = mesh_from_geometry(
        TorusGeometry(INNER_RADIUS, 0.022, radial_segments=14, tubular_segments=96).rotate_x(math.pi / 2.0),
        "inner_hanger_ring",
    )
    hub_ring_mesh = mesh_from_geometry(
        TorusGeometry(0.18, 0.030, radial_segments=14, tubular_segments=48).rotate_x(math.pi / 2.0),
        "hub_bearing_ring",
    )
    wheel.visual(
        Cylinder(radius=0.125, length=0.86),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hub_sleeve",
    )
    for side, y in (("rear", -RIM_Y), ("front", RIM_Y)):
        if side == "rear":
            wheel.visual(
                outer_ring_mesh,
                origin=Origin(xyz=(0.0, y, 0.0)),
                material=painted_steel,
                name="outer_rim_rear",
            )
        else:
            wheel.visual(
                outer_ring_mesh,
                origin=Origin(xyz=(0.0, y, 0.0)),
                material=painted_steel,
                name="outer_rim_front",
            )
        wheel.visual(inner_ring_mesh, origin=Origin(xyz=(0.0, y, 0.0)), material=painted_steel, name=f"inner_ring_{side}")
        wheel.visual(hub_ring_mesh, origin=Origin(xyz=(0.0, y, 0.0)), material=dark_steel, name=f"hub_ring_{side}")
        for j, (dx, dz) in enumerate(((1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0))):
            _add_cylinder_between(
                wheel,
                (0.125 * dx, y, 0.125 * dz),
                (0.205 * dx, y, 0.205 * dz),
                0.018,
                material=dark_steel,
                name=f"hub_connector_{side}_{j}",
            )
        for i in range(16):
            theta = 2.0 * math.pi * i / 16.0
            dx, dz = math.sin(theta), math.cos(theta)
            _add_cylinder_between(
                wheel,
                (0.19 * dx, y, 0.19 * dz),
                (2.35 * dx, y, 2.35 * dz),
                0.018,
                material=painted_steel,
                name=f"spoke_{side}_{i}",
            )
    for i in range(CABIN_COUNT):
        theta = 2.0 * math.pi * i / CABIN_COUNT
        dx, dz = math.sin(theta), math.cos(theta)
        _add_cylinder_between(
            wheel,
            (OUTER_RADIUS * dx, -RIM_Y, OUTER_RADIUS * dz),
            (OUTER_RADIUS * dx, RIM_Y, OUTER_RADIUS * dz),
            0.018,
            material=dark_steel,
            name=f"rim_crossbar_{i}",
        )
        _add_cylinder_between(
            wheel,
            (INNER_RADIUS * dx, -RIM_Y, INNER_RADIUS * dz),
            (INNER_RADIUS * dx, RIM_Y, INNER_RADIUS * dz),
            0.020,
            material=dark_steel,
            name=f"hanger_pin_{i}",
        )

    model.articulation(
        "tower_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, WHEEL_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8000.0, velocity=0.45),
    )

    # Gondolas are child parts of the rotating wheel and pivot on the short
    # cross-pins between the two rim planes.
    for i in range(CABIN_COUNT):
        theta = 2.0 * math.pi * i / CABIN_COUNT
        x = INNER_RADIUS * math.sin(theta)
        z = INNER_RADIUS * math.cos(theta)
        cabin = model.part(f"cabin_{i}")
        cabin.visual(
            Box((0.42, 0.40, 0.32)),
            origin=Origin(xyz=(0.0, 0.0, -0.36)),
            material=yellow,
            name="body",
        )
        cabin.visual(
            Box((0.46, 0.44, 0.055)),
            origin=Origin(xyz=(0.0, 0.0, -0.180)),
            material=dark_roof,
            name="roof",
        )
        cabin.visual(
            Box((0.46, 0.44, 0.045)),
            origin=Origin(xyz=(0.0, 0.0, -0.535)),
            material=dark_roof,
            name="floor",
        )
        cabin.visual(
            Box((0.30, 0.010, 0.16)),
            origin=Origin(xyz=(0.0, 0.205, -0.34)),
            material=glass,
            name="front_window",
        )
        cabin.visual(
            Box((0.30, 0.010, 0.16)),
            origin=Origin(xyz=(0.0, -0.205, -0.34)),
            material=glass,
            name="rear_window",
        )
        cabin.visual(
            Box((0.010, 0.26, 0.14)),
            origin=Origin(xyz=(0.215, 0.0, -0.35)),
            material=glass,
            name="side_window_0",
        )
        cabin.visual(
            Box((0.010, 0.26, 0.14)),
            origin=Origin(xyz=(-0.215, 0.0, -0.35)),
            material=glass,
            name="side_window_1",
        )
        for y in (-0.17, 0.17):
            cabin.visual(
                Cylinder(radius=0.010, length=0.210),
                origin=Origin(xyz=(0.0, y, -0.105)),
                material=dark_steel,
                name=f"hanger_rod_{'rear' if y < 0 else 'front'}",
            )
        cabin.visual(
            Cylinder(radius=0.012, length=0.38),
            origin=Origin(xyz=(0.0, 0.0, -0.180), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name="roof_crossbar",
        )
        model.articulation(
            f"wheel_to_cabin_{i}",
            ArticulationType.REVOLUTE,
            parent=wheel,
            child=cabin,
            origin=Origin(xyz=(x, 0.0, z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=120.0, velocity=1.2, lower=-1.35, upper=1.35),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wheel = object_model.get_part("wheel")
    tower = object_model.get_part("tower")
    wheel_joint = object_model.get_articulation("tower_to_wheel")

    ctx.check(
        "wheel has a horizontal continuous axle",
        wheel_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 6) for v in wheel_joint.axis) == (0.0, 1.0, 0.0),
        details=f"type={wheel_joint.articulation_type}, axis={wheel_joint.axis}",
    )
    ctx.allow_overlap(
        wheel,
        tower,
        elem_a="hub_sleeve",
        elem_b="fixed_axle",
        reason="The rotating hub sleeve is intentionally modeled around the fixed axle bearing.",
    )
    ctx.expect_within(
        tower,
        wheel,
        axes="xz",
        inner_elem="fixed_axle",
        outer_elem="hub_sleeve",
        margin=0.001,
        name="fixed axle is centered inside the hub sleeve",
    )
    ctx.expect_overlap(
        wheel,
        tower,
        axes="y",
        elem_a="hub_sleeve",
        elem_b="fixed_axle",
        min_overlap=0.80,
        name="hub sleeve overlaps the axle along the bearing length",
    )
    ctx.expect_gap(
        tower,
        wheel,
        axis="y",
        min_gap=0.20,
        max_gap=0.32,
        positive_elem="bearing_block_front",
        negative_elem="outer_rim_front",
        name="front rim clears the side bearing",
    )
    ctx.expect_gap(
        wheel,
        tower,
        axis="y",
        min_gap=0.20,
        max_gap=0.32,
        positive_elem="outer_rim_rear",
        negative_elem="bearing_block_rear",
        name="rear rim clears the side bearing",
    )

    for i in range(CABIN_COUNT):
        cabin = object_model.get_part(f"cabin_{i}")
        hinge = object_model.get_articulation(f"wheel_to_cabin_{i}")
        pivot = ctx.part_world_position(cabin)
        body_box = ctx.part_element_world_aabb(cabin, elem="body")
        radial = math.sqrt((pivot[0] if pivot else 99.0) ** 2 + ((pivot[2] - WHEEL_CENTER_Z) if pivot else 99.0) ** 2)
        ctx.check(
            f"cabin_{i} pivots on the wheel axle direction",
            hinge.articulation_type == ArticulationType.REVOLUTE
            and tuple(round(v, 6) for v in hinge.axis) == (0.0, 1.0, 0.0),
            details=f"type={hinge.articulation_type}, axis={hinge.axis}",
        )
        ctx.check(
            f"cabin_{i} hanger is on the inner structure",
            radial < OUTER_RADIUS - 0.35,
            details=f"pivot_radius={radial:.3f}, outer_radius={OUTER_RADIUS}",
        )
        for rod_name in ("hanger_rod_rear", "hanger_rod_front"):
            ctx.allow_overlap(
                cabin,
                wheel,
                elem_a=rod_name,
                elem_b=f"hanger_pin_{i}",
                reason="The cabin hanger rod is intentionally captured around the wheel's short pivot pin.",
            )
            ctx.expect_gap(
                wheel,
                cabin,
                axis="z",
                max_penetration=0.025,
                positive_elem=f"hanger_pin_{i}",
                negative_elem=rod_name,
                name=f"cabin_{i} {rod_name} seats on its pivot pin",
            )
        ctx.expect_within(
            cabin,
            wheel,
            axes="y",
            margin=0.02,
            inner_elem="body",
            outer_elem=None,
            name=f"cabin_{i} body stays between rim planes",
        )
        ctx.check(
            f"cabin_{i} hangs below its pivot",
            pivot is not None and body_box is not None and body_box[1][2] < pivot[2] - 0.12,
            details=f"pivot={pivot}, body_aabb={body_box}",
        )

    # A decisive pose check: when the wheel rotates, an independently posed
    # cabin can swing on the same horizontal hanger axis instead of being fused
    # to the rim.
    cabin_0 = object_model.get_part("cabin_0")
    before = ctx.part_world_aabb(cabin_0)
    with ctx.pose({"wheel_to_cabin_0": 0.50}):
        after = ctx.part_world_aabb(cabin_0)
    ctx.check(
        "cabin swings on its hanger pivot",
        before is not None and after is not None and abs(before[0][0] - after[0][0]) > 0.10,
        details=f"before={before}, after={after}",
    )

    return ctx.report()


object_model = build_object_model()
