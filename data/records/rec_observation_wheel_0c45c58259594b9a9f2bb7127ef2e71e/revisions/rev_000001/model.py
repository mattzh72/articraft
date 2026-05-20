from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def _cylinder_between_xz(
    part,
    p1: tuple[float, float, float],
    p2: tuple[float, float, float],
    *,
    radius: float,
    material: Material,
    name: str,
):
    """Add a cylinder whose endpoints lie in a local XZ plane at constant Y."""
    mid = tuple((a + b) * 0.5 for a, b in zip(p1, p2))
    dx = p2[0] - p1[0]
    dz = p2[2] - p1[2]
    length = math.sqrt(dx * dx + dz * dz)
    pitch = math.atan2(dx, dz)
    return part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=mid, rpy=(0.0, pitch, 0.0)),
        material=material,
        name=name,
    )


def _cylinder_x(part, *, center, length, radius, material, name):
    return part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _cylinder_y(part, *, center, length, radius, material, name):
    return part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="observation_wheel")

    steel = model.material("painted_steel", rgba=(0.82, 0.84, 0.86, 1.0))
    dark_steel = model.material("dark_hub_steel", rgba=(0.18, 0.20, 0.23, 1.0))
    base_mat = model.material("matte_concrete", rgba=(0.44, 0.43, 0.40, 1.0))
    accent = model.material("rim_accent_blue", rgba=(0.10, 0.32, 0.70, 1.0))
    pod_shell = model.material("pod_warm_white", rgba=(0.95, 0.90, 0.78, 1.0))
    pod_floor = model.material("pod_dark_floor", rgba=(0.16, 0.14, 0.13, 1.0))
    glass = model.material("blue_tinted_glass", rgba=(0.30, 0.65, 0.95, 0.46))

    hub_height = 3.0
    rim_radius = 1.85
    inner_rim_radius = 1.66
    pod_count = 8
    support_y = -0.28
    pod_y = 0.42

    support = model.part("support")
    support.visual(
        Box((3.60, 1.25, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=base_mat,
        name="broad_base",
    )
    support.visual(
        Box((0.55, 0.28, 0.14)),
        origin=Origin(xyz=(-1.28, support_y, 0.23)),
        material=base_mat,
        name="foot_0",
    )
    support.visual(
        Box((0.55, 0.28, 0.14)),
        origin=Origin(xyz=(1.28, support_y, 0.23)),
        material=base_mat,
        name="foot_1",
    )
    _cylinder_between_xz(
        support,
        (-1.32, support_y, 0.18),
        (0.0, support_y, hub_height),
        radius=0.060,
        material=steel,
        name="leg_0",
    )
    _cylinder_between_xz(
        support,
        (1.32, support_y, 0.18),
        (0.0, support_y, hub_height),
        radius=0.060,
        material=steel,
        name="leg_1",
    )
    _cylinder_x(
        support,
        center=(0.0, support_y, 1.30),
        length=1.75,
        radius=0.035,
        material=dark_steel,
        name="cross_brace",
    )
    _cylinder_y(
        support,
        center=(0.0, -0.24, hub_height),
        length=0.22,
        radius=0.26,
        material=dark_steel,
        name="rear_bearing",
    )
    support.visual(
        Box((0.44, 0.18, 0.36)),
        origin=Origin(xyz=(0.0, -0.28, hub_height - 0.03)),
        material=dark_steel,
        name="bearing_block",
    )

    wheel = model.part("wheel")
    _cylinder_y(
        wheel,
        center=(0.0, 0.0, 0.0),
        length=0.26,
        radius=0.22,
        material=dark_steel,
        name="central_hub",
    )
    _cylinder_y(
        wheel,
        center=(0.0, 0.16, 0.0),
        length=0.08,
        radius=0.27,
        material=accent,
        name="front_hub_cap",
    )

    rim_segments = 32
    for i in range(rim_segments):
        a0 = 2.0 * math.pi * i / rim_segments
        a1 = 2.0 * math.pi * (i + 1) / rim_segments
        p0 = (rim_radius * math.cos(a0), 0.0, rim_radius * math.sin(a0))
        p1 = (rim_radius * math.cos(a1), 0.0, rim_radius * math.sin(a1))
        _cylinder_between_xz(
            wheel,
            p0,
            p1,
            radius=0.035,
            material=accent,
            name=f"outer_rim_{i}",
        )
        q0 = (
            inner_rim_radius * math.cos(a0),
            0.0,
            inner_rim_radius * math.sin(a0),
        )
        q1 = (
            inner_rim_radius * math.cos(a1),
            0.0,
            inner_rim_radius * math.sin(a1),
        )
        _cylinder_between_xz(
            wheel,
            q0,
            q1,
            radius=0.024,
            material=steel,
            name=f"inner_rim_{i}",
        )

    for i in range(pod_count):
        a = 2.0 * math.pi * i / pod_count
        ca = math.cos(a)
        sa = math.sin(a)
        _cylinder_between_xz(
            wheel,
            (0.0, 0.0, 0.0),
            ((rim_radius + 0.08) * ca, 0.0, (rim_radius + 0.08) * sa),
            radius=0.023,
            material=steel,
            name=f"spoke_{i}",
        )
        rim_x = rim_radius * ca
        rim_z = rim_radius * sa
        pivot_z = rim_z - 0.18
        _cylinder_y(
            wheel,
            center=(rim_x, 0.28, rim_z),
            length=0.56,
            radius=0.024,
            material=dark_steel,
            name=f"bracket_{i}_arm",
        )
        for side, plate_y in enumerate((0.28, 0.56)):
            wheel.visual(
                Box((0.090, 0.040, 0.170)),
                origin=Origin(xyz=(rim_x, plate_y, pivot_z + 0.085)),
                material=dark_steel,
                name=f"bracket_{i}_yoke_{side}",
            )

    wheel_joint = model.articulation(
        "support_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=support,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, hub_height)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=500.0, velocity=0.45),
    )

    for i in range(pod_count):
        a = 2.0 * math.pi * i / pod_count
        rim_x = rim_radius * math.cos(a)
        rim_z = rim_radius * math.sin(a)
        pivot_z = rim_z - 0.18
        pod = model.part(f"pod_{i}")
        _cylinder_y(
            pod,
            center=(0.0, 0.0, 0.0),
            length=0.24,
            radius=0.032,
            material=dark_steel,
            name="pivot_axle",
        )
        _cylinder_between_xz(
            pod,
            (0.0, 0.0, -0.020),
            (-0.19, 0.0, -0.34),
            radius=0.014,
            material=dark_steel,
            name="hanger_0",
        )
        _cylinder_between_xz(
            pod,
            (0.0, 0.0, -0.020),
            (0.19, 0.0, -0.34),
            radius=0.014,
            material=dark_steel,
            name="hanger_1",
        )
        _cylinder_x(
            pod,
            center=(0.0, 0.0, -0.35),
            length=0.42,
            radius=0.020,
            material=dark_steel,
            name="roof_cradle",
        )
        _cylinder_x(
            pod,
            center=(0.0, 0.0, -0.55),
            length=0.42,
            radius=0.18,
            material=pod_shell,
            name="rounded_cabin",
        )
        pod.visual(
            Sphere(radius=0.18),
            origin=Origin(xyz=(-0.21, 0.0, -0.55)),
            material=pod_shell,
            name="cabin_cap_0",
        )
        pod.visual(
            Sphere(radius=0.18),
            origin=Origin(xyz=(0.21, 0.0, -0.55)),
            material=pod_shell,
            name="cabin_cap_1",
        )
        pod.visual(
            Box((0.50, 0.018, 0.105)),
            origin=Origin(xyz=(0.0, 0.184, -0.53)),
            material=glass,
            name="front_window",
        )
        pod.visual(
            Box((0.50, 0.018, 0.105)),
            origin=Origin(xyz=(0.0, -0.184, -0.53)),
            material=glass,
            name="rear_window",
        )
        pod.visual(
            Box((0.42, 0.25, 0.055)),
            origin=Origin(xyz=(0.0, 0.0, -0.72)),
            material=pod_floor,
            name="floor_band",
        )

        model.articulation(
            f"wheel_to_pod_{i}",
            ArticulationType.CONTINUOUS,
            parent=wheel,
            child=pod,
            origin=Origin(xyz=(rim_x, pod_y, pivot_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=30.0, velocity=1.8),
            mimic=Mimic(joint=wheel_joint.name, multiplier=-1.0, offset=0.0),
        )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    wheel_joint = object_model.get_articulation("support_to_wheel")
    wheel = object_model.get_part("wheel")
    support = object_model.get_part("support")
    ctx.check(
        "wheel has a continuous horizontal hub axis",
        wheel_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(wheel_joint.axis) == (0.0, 1.0, 0.0),
        details=f"type={wheel_joint.articulation_type}, axis={wheel_joint.axis}",
    )
    ctx.expect_gap(
        wheel,
        support,
        axis="y",
        positive_elem="central_hub",
        negative_elem="rear_bearing",
        min_gap=0.0,
        max_gap=0.03,
        name="rotating hub is closely captured by rear bearing",
    )

    pod_count = 8
    pod_joints = [object_model.get_articulation(f"wheel_to_pod_{i}") for i in range(pod_count)]
    ctx.check(
        "every rim bracket carries a pivoting pod",
        all(j.articulation_type == ArticulationType.CONTINUOUS and tuple(j.axis) == (0.0, 1.0, 0.0) for j in pod_joints),
        details=", ".join(f"{j.name}:{j.articulation_type}:{j.axis}" for j in pod_joints),
    )

    for i in range(pod_count):
        pod = object_model.get_part(f"pod_{i}")
        ctx.expect_gap(
            wheel,
            pod,
            axis="z",
            positive_elem=f"bracket_{i}_yoke_0",
            negative_elem="rounded_cabin",
            min_gap=0.12,
            name=f"pod_{i} cabin hangs below its bracket",
        )

    before = ctx.part_element_world_aabb(wheel, elem="bracket_0_arm")
    with ctx.pose({wheel_joint: math.pi / 2.0}):
        after = ctx.part_element_world_aabb(wheel, elem="bracket_0_arm")
        for i in (0, 2, 4, 6):
            pod = object_model.get_part(f"pod_{i}")
            pivot = ctx.part_world_position(pod)
            cabin = ctx.part_element_world_aabb(pod, elem="rounded_cabin")
            ok = pivot is not None and cabin is not None and cabin[1][2] < pivot[2] - 0.12
            ctx.check(
                f"pod_{i} remains suspended under its pivot while wheel turns",
                ok,
                details=f"pivot={pivot}, cabin_aabb={cabin}",
            )
    ctx.check(
        "wheel pose moves a rim bracket around the hub",
        before is not None and after is not None and abs(after[0][2] - before[0][2]) > 1.0,
        details=f"before={before}, after={after}",
    )

    return ctx.report()


object_model = build_object_model()
