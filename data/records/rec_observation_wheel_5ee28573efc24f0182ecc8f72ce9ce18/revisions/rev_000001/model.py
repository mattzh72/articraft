from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
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


def _build_ring_mesh(outer_radius: float, inner_radius: float, width: float):
    return mesh_from_cadquery(
        cq.Workplane("XZ").circle(outer_radius).circle(inner_radius).extrude(width * 0.5, both=True),
        "wheel_ring",
    )


def _build_capsule_shell_mesh(depth: float, width: float, height: float, wall: float):
    outer = cq.Workplane("XY").box(depth, width, height).edges().fillet(0.10)
    inner = (
        cq.Workplane("XY")
        .box(depth - 2.0 * wall, width - 2.0 * wall, height - 2.0 * wall)
        .edges()
        .fillet(0.07)
    )
    shell = outer.cut(inner)

    front_window = cq.Workplane("XY").box(wall * 2.4, width - 0.54, height - 0.72).translate(
        (depth * 0.5 - wall * 0.55, 0.0, 0.02)
    )
    rear_window = cq.Workplane("XY").box(wall * 2.4, width - 0.54, height - 0.72).translate(
        (-depth * 0.5 + wall * 0.55, 0.0, 0.02)
    )
    left_window = cq.Workplane("XY").box(depth - 0.46, wall * 2.4, height - 0.82).translate(
        (0.0, width * 0.5 - wall * 0.55, 0.00)
    )
    right_window = cq.Workplane("XY").box(depth - 0.46, wall * 2.4, height - 0.82).translate(
        (0.0, -width * 0.5 + wall * 0.55, 0.00)
    )

    shell = shell.cut(front_window).cut(rear_window).cut(left_window).cut(right_window)
    return mesh_from_cadquery(shell, "capsule_shell")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="city_observation_wheel")

    support_white = model.material("support_white", rgba=(0.93, 0.95, 0.97, 1.0))
    wheel_white = model.material("wheel_white", rgba=(0.96, 0.97, 0.98, 1.0))
    steel = model.material("steel", rgba=(0.60, 0.62, 0.66, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.25, 0.27, 0.30, 1.0))
    foundation = model.material("foundation", rgba=(0.69, 0.70, 0.72, 1.0))
    glass = model.material("glass", rgba=(0.70, 0.84, 0.92, 0.28))
    cabin_trim = model.material("cabin_trim", rgba=(0.82, 0.84, 0.87, 1.0))
    cabin_floor = model.material("cabin_floor", rgba=(0.25, 0.26, 0.28, 1.0))

    center_z = 28.4
    wheel_radius = 22.5
    rim_outer_radius = 22.85
    rim_inner_radius = 22.10
    ring_y = 1.25
    hub_radius = 1.10
    hub_disc_radius = 2.20
    support_y = 3.00
    cabin_count = 12
    arm_outset = 1.45
    pivot_drop = 0.55
    pivot_y = 1.13

    ring_mesh = _build_ring_mesh(rim_outer_radius, rim_inner_radius, 0.16)
    capsule_shell_mesh = _build_capsule_shell_mesh(depth=1.50, width=2.20, height=2.10, wall=0.10)

    base = model.part("base")
    base.visual(
        Box((18.0, 8.2, 0.75)),
        origin=Origin(xyz=(0.0, 0.0, 0.375)),
        material=foundation,
        name="foundation_slab",
    )
    base.visual(
        Box((7.0, 4.2, 0.55)),
        origin=Origin(xyz=(0.0, 0.0, 1.02)),
        material=foundation,
        name="loading_plinth",
    )
    base.visual(
        Box((4.6, 1.8, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 1.355)),
        material=steel,
        name="boarding_deck",
    )
    for y_sign in (-1.0, 1.0):
        y = y_sign * support_y
        left_foot = (-6.8, y, 0.75)
        right_foot = (6.8, y, 0.75)
        left_apex = (-1.55, y, center_z - 0.25)
        right_apex = (1.55, y, center_z - 0.25)
        left_mid = (-3.4, y, 13.4)
        right_mid = (3.4, y, 13.4)

        base.visual(
            Box((2.3, 1.20, 0.85)),
            origin=Origin(xyz=(left_foot[0], y, 1.18)),
            material=foundation,
            name=f"footing_{int(y_sign)}_0",
        )
        base.visual(
            Box((2.3, 1.20, 0.85)),
            origin=Origin(xyz=(right_foot[0], y, 1.18)),
            material=foundation,
            name=f"footing_{int(y_sign)}_1",
        )
        _add_member(base, left_foot, left_apex, 0.48, support_white, name=f"support_leg_{int(y_sign)}_0")
        _add_member(base, right_foot, right_apex, 0.48, support_white, name=f"support_leg_{int(y_sign)}_1")
        _add_member(base, left_foot, right_mid, 0.19, steel, name=f"brace_{int(y_sign)}_0")
        _add_member(base, right_foot, left_mid, 0.19, steel, name=f"brace_{int(y_sign)}_1")
        _add_member(base, left_mid, right_mid, 0.16, dark_steel, name=f"spread_{int(y_sign)}")
        _add_member(base, left_apex, right_apex, 0.24, dark_steel, name=f"top_spread_{int(y_sign)}")
        base.visual(
            Box((4.20, 0.65, 0.92)),
            origin=Origin(xyz=(0.0, y, center_z - 0.25)),
            material=dark_steel,
            name=f"bearing_house_{int(y_sign)}",
        )
        base.visual(
            Cylinder(radius=0.46, length=0.44),
            origin=Origin(xyz=(0.0, y_sign * 2.82, center_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"bearing_drum_{int(y_sign)}",
        )

    base.visual(
        Box((2.4, 5.0, 0.38)),
        origin=Origin(xyz=(-4.10, 0.0, 1.485)),
        material=dark_steel,
        name="machine_house",
    )

    wheel = model.part("wheel")
    wheel.visual(
        ring_mesh,
        origin=Origin(xyz=(0.0, -ring_y, 0.0)),
        material=wheel_white,
        name="rim_inner_side",
    )
    wheel.visual(
        ring_mesh,
        origin=Origin(xyz=(0.0, ring_y, 0.0)),
        material=wheel_white,
        name="rim_outer_side",
    )
    wheel.visual(
        Cylinder(radius=hub_radius, length=2.60),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hub_barrel",
    )
    wheel.visual(
        Cylinder(radius=hub_disc_radius, length=0.24),
        origin=Origin(xyz=(0.0, -1.25, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hub_disc_0",
    )
    wheel.visual(
        Cylinder(radius=hub_disc_radius, length=0.24),
        origin=Origin(xyz=(0.0, 1.25, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hub_disc_1",
    )
    wheel.visual(
        Cylinder(radius=0.30, length=1.56),
        origin=Origin(xyz=(0.0, -1.82, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="axle_stub_0",
    )
    wheel.visual(
        Cylinder(radius=0.30, length=1.56),
        origin=Origin(xyz=(0.0, 1.82, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="axle_stub_1",
    )

    spoke_count = 20
    spoke_start_radius = 1.85
    spoke_end_radius = 22.42
    for side_index, y in enumerate((-1.25, 1.25)):
        for spoke_index in range(spoke_count):
            theta = (2.0 * math.pi * spoke_index) / spoke_count
            inner = (
                math.cos(theta) * spoke_start_radius,
                y,
                math.sin(theta) * spoke_start_radius,
            )
            outer = (
                math.cos(theta) * spoke_end_radius,
                y,
                math.sin(theta) * spoke_end_radius,
            )
            _add_member(
                wheel,
                inner,
                outer,
                0.09,
                steel,
                name=f"spoke_{side_index}_{spoke_index}",
            )

    for capsule_index in range(cabin_count):
        theta = (2.0 * math.pi * capsule_index) / cabin_count
        radial = (math.cos(theta), 0.0, math.sin(theta))

        for y_sign in (-1.0, 1.0):
            anchor = (
                radial[0] * 22.55,
                y_sign * ring_y,
                radial[2] * 22.55,
            )
            arm_head = (
                radial[0] * (22.55 + arm_outset),
                y_sign * 1.50,
                radial[2] * (22.55 + arm_outset),
            )
            lug = (
                arm_head[0],
                y_sign * pivot_y,
                arm_head[2] - pivot_drop,
            )
            _add_member(
                wheel,
                anchor,
                arm_head,
                0.10,
                wheel_white,
                name=f"arm_{capsule_index}_{int(y_sign)}_0",
            )
            _add_member(
                wheel,
                arm_head,
                lug,
                0.09,
                steel,
                name=f"arm_{capsule_index}_{int(y_sign)}_1",
            )
            wheel.visual(
                Box((0.18, 0.14, 0.22)),
                origin=Origin(xyz=lug),
                material=dark_steel,
                name=f"pivot_lug_{capsule_index}_{int(y_sign)}",
            )

    for capsule_index in range(cabin_count):
        capsule = model.part(f"capsule_{capsule_index}")
        capsule.visual(
            capsule_shell_mesh,
            origin=Origin(xyz=(0.0, 0.0, -1.34)),
            material=cabin_trim,
            name="shell",
        )
        capsule.visual(
            Box((1.00, 1.82, 0.10)),
            origin=Origin(xyz=(0.0, 0.0, -2.00)),
            material=cabin_floor,
            name="cabin_floor",
        )
        capsule.visual(
            Box((1.08, 1.88, 0.10)),
            origin=Origin(xyz=(0.0, 0.0, -0.18)),
            material=cabin_trim,
            name="roof_cap",
        )
        capsule.visual(
            Box((1.12, 1.84, 1.72)),
            origin=Origin(xyz=(0.0, 0.0, -1.09)),
            material=glass,
            name="glazing",
        )
        capsule.visual(
            Box((0.12, 0.12, 0.44)),
            origin=Origin(xyz=(0.0, 1.00, -0.24)),
            material=dark_steel,
            name="hanger_0",
        )
        capsule.visual(
            Box((0.12, 0.12, 0.44)),
            origin=Origin(xyz=(0.0, -1.00, -0.24)),
            material=dark_steel,
            name="hanger_1",
        )
        capsule.visual(
            Cylinder(radius=0.08, length=1.86),
            origin=Origin(xyz=(0.0, 0.0, -0.20), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name="hanger_beam",
        )

        theta = (2.0 * math.pi * capsule_index) / cabin_count
        pivot_xyz = (
            math.cos(theta) * (22.55 + arm_outset),
            0.0,
            math.sin(theta) * (22.55 + arm_outset) - pivot_drop,
        )
        model.articulation(
            f"capsule_pivot_{capsule_index}",
            ArticulationType.CONTINUOUS,
            parent=wheel,
            child=capsule,
            origin=Origin(xyz=pivot_xyz),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=800.0, velocity=1.2),
            mimic=Mimic(joint="wheel_spin", multiplier=-1.0),
        )

    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, center_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=250000.0, velocity=0.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    wheel_spin = object_model.get_articulation("wheel_spin")
    capsule_0 = object_model.get_part("capsule_0")

    ctx.expect_gap(
        capsule_0,
        capsule_0,
        axis="z",
        positive_elem="roof_cap",
        negative_elem="cabin_floor",
        min_gap=1.68,
        name="capsule_0 cabin stack is tall at rest",
    )

    rest_pos = ctx.part_world_position(capsule_0)
    with ctx.pose({wheel_spin: math.pi / 2.0}):
        turned_pos = ctx.part_world_position(capsule_0)
        ctx.expect_gap(
            capsule_0,
            capsule_0,
            axis="z",
            positive_elem="roof_cap",
            negative_elem="cabin_floor",
            min_gap=1.68,
            name="capsule_0 stays level when wheel turns",
        )

    ctx.check(
        "capsule_0 rides upward with positive wheel rotation",
        rest_pos is not None and turned_pos is not None and turned_pos[2] > rest_pos[2] + 18.0,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


object_model = build_object_model()
