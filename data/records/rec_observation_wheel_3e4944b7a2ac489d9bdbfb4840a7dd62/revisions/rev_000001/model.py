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
    mesh_from_geometry,
    superellipse_side_loft,
    tube_from_spline_points,
    wire_from_points,
)


TAU = 2.0 * math.pi


def _circle_points(radius: float, y: float, *, count: int = 48) -> list[tuple[float, float, float]]:
    return [
        (radius * math.cos(TAU * i / count), y, radius * math.sin(TAU * i / count))
        for i in range(count)
    ]


def _tube_between(
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
    *,
    radius: float,
    segments: int = 16,
):
    return wire_from_points(
        [p0, p1],
        radius=radius,
        radial_segments=segments,
        cap_ends=True,
        corner_mode="miter",
        up_hint=(0.0, 0.0, 1.0),
    )


def _capsule_shell_mesh():
    # A closed, enclosed passenger pod: elongated along local Y, with a rounded
    # superellipse cross section and tapered nose/tail faces.
    sections = [
        (-1.25, -2.45, -0.95, 1.35),
        (-1.05, -2.70, -0.72, 2.00),
        (-0.55, -2.85, -0.62, 2.30),
        (0.55, -2.85, -0.62, 2.30),
        (1.05, -2.70, -0.72, 2.00),
        (1.25, -2.45, -0.95, 1.35),
    ]
    return mesh_from_geometry(
        superellipse_side_loft(sections, exponents=2.7, segments=44, cap=True),
        "capsule_shell",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="giant_city_observation_wheel")

    steel = Material("painted_steel", rgba=(0.62, 0.66, 0.68, 1.0))
    dark_steel = Material("dark_grey_steel", rgba=(0.12, 0.14, 0.16, 1.0))
    concrete = Material("warm_concrete", rgba=(0.55, 0.54, 0.50, 1.0))
    white = Material("capsule_white", rgba=(0.92, 0.94, 0.92, 1.0))
    glass = Material("blue_tinted_glass", rgba=(0.22, 0.50, 0.72, 0.58))
    rubber = Material("black_window_gasket", rgba=(0.03, 0.035, 0.04, 1.0))

    hub_height = 18.0
    rim_radius = 14.0
    pivot_radius = 14.45
    wheel_half_width = 1.55
    support_y = 3.10
    capsule_count = 12

    support = model.part("support")
    support.visual(
        Box((18.5, 7.4, 0.60)),
        origin=Origin(xyz=(0.0, 0.0, 0.30)),
        material=concrete,
        name="foundation",
    )
    support.visual(
        Box((14.0, 0.42, 0.42)),
        origin=Origin(xyz=(0.0, support_y, 1.05)),
        material=steel,
        name="front_base_tie",
    )
    support.visual(
        Box((14.0, 0.42, 0.42)),
        origin=Origin(xyz=(0.0, -support_y, 1.05)),
        material=steel,
        name="rear_base_tie",
    )

    for y, frame_name in ((support_y, "front"), (-support_y, "rear")):
        for x, side_name in ((-7.4, "leg_0"), (7.4, "leg_1")):
            leg = _tube_between((x, y, 0.55), (0.0, y, hub_height), radius=0.30, segments=18)
            support.visual(
                mesh_from_geometry(leg, f"{frame_name}_{side_name}"),
                material=steel,
                name=f"{frame_name}_{side_name}",
            )
        brace = _tube_between((-5.8, y, 5.2), (5.8, y, 5.2), radius=0.16, segments=14)
        support.visual(
            mesh_from_geometry(brace, f"{frame_name}_low_crossbrace"),
            material=dark_steel,
            name=f"{frame_name}_low_crossbrace",
        )
        support.visual(
            Box((1.7, 0.58, 1.20)),
            origin=Origin(xyz=(0.0, y, hub_height)),
            material=steel,
            name=f"{frame_name}_bearing_block",
        )

    for x, name in ((-7.4, "base_cross_0"), (7.4, "base_cross_1")):
        cross = _tube_between((x, -support_y, 0.92), (x, support_y, 0.92), radius=0.18, segments=14)
        support.visual(
            mesh_from_geometry(cross, name),
            material=dark_steel,
            name=name,
        )

    support.visual(
        Cylinder(radius=0.22, length=7.0),
        origin=Origin(xyz=(0.0, 0.0, hub_height), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="main_axle",
    )

    wheel = model.part("wheel")
    wheel.visual(
        Cylinder(radius=0.74, length=3.35),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hub_shell",
    )
    wheel.visual(
        Cylinder(radius=1.05, length=0.18),
        origin=Origin(xyz=(0.0, wheel_half_width, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="front_hub_flange",
    )
    wheel.visual(
        Cylinder(radius=1.05, length=0.18),
        origin=Origin(xyz=(0.0, -wheel_half_width, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="rear_hub_flange",
    )

    for y, name in ((wheel_half_width, "front_rim"), (-wheel_half_width, "rear_rim")):
        rim = tube_from_spline_points(
            _circle_points(rim_radius, y, count=64),
            radius=0.15,
            samples_per_segment=5,
            closed_spline=True,
            radial_segments=18,
            up_hint=(0.0, 1.0, 0.0),
        )
        wheel.visual(mesh_from_geometry(rim, name), material=steel, name=name)

    for i in range(24):
        angle = TAU * i / 24
        ux, uz = math.cos(angle), math.sin(angle)
        for y, plane in ((wheel_half_width, "front"), (-wheel_half_width, "rear")):
            start = (0.88 * ux, y, 0.88 * uz)
            end = ((rim_radius - 0.14) * ux, y, (rim_radius - 0.14) * uz)
            spoke = _tube_between(start, end, radius=0.055, segments=10)
            wheel.visual(
                mesh_from_geometry(spoke, f"{plane}_spoke_{i}"),
                material=dark_steel,
                name=f"{plane}_spoke_{i}",
            )

    for i in range(capsule_count):
        angle = TAU * i / capsule_count
        ux, uz = math.cos(angle), math.sin(angle)
        front_rim_point = (rim_radius * ux, wheel_half_width, rim_radius * uz)
        rear_rim_point = (rim_radius * ux, -wheel_half_width, rim_radius * uz)
        front_arm_end = ((pivot_radius - 0.30) * ux, 0.12, (pivot_radius - 0.30) * uz)
        rear_arm_end = ((pivot_radius - 0.30) * ux, -0.12, (pivot_radius - 0.30) * uz)
        pivot = (pivot_radius * ux, 0.0, pivot_radius * uz)
        front_arm = _tube_between(front_rim_point, front_arm_end, radius=0.095, segments=12)
        rear_arm = _tube_between(rear_rim_point, rear_arm_end, radius=0.095, segments=12)
        wheel.visual(
            mesh_from_geometry(front_arm, f"front_rim_arm_{i}"),
            material=steel,
            name=f"front_rim_arm_{i}",
        )
        wheel.visual(
            mesh_from_geometry(rear_arm, f"rear_rim_arm_{i}"),
            material=steel,
            name=f"rear_rim_arm_{i}",
        )
        wheel.visual(
            Sphere(radius=0.26),
            origin=Origin(xyz=pivot),
            material=dark_steel,
            name=f"pivot_socket_{i}",
        )

    main_joint = model.articulation(
        "main_axle",
        ArticulationType.CONTINUOUS,
        parent=support,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, hub_height)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=50000.0, velocity=0.35),
    )

    capsule_shell = _capsule_shell_mesh()

    for i in range(capsule_count):
        capsule = model.part(f"capsule_{i}")
        capsule.visual(capsule_shell, material=white, name="shell")
        capsule.visual(
            Cylinder(radius=0.16, length=1.06),
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name="pivot_pin",
        )
        capsule.visual(
            Box((0.24, 0.16, 0.62)),
            origin=Origin(xyz=(0.0, 0.36, -0.47)),
            material=dark_steel,
            name="hanger_0",
        )
        capsule.visual(
            Box((0.24, 0.16, 0.62)),
            origin=Origin(xyz=(0.0, -0.36, -0.47)),
            material=dark_steel,
            name="hanger_1",
        )
        capsule.visual(
            Box((1.34, 0.08, 0.78)),
            origin=Origin(xyz=(0.0, 1.235, -1.55)),
            material=glass,
            name="front_window",
        )
        capsule.visual(
            Box((1.34, 0.08, 0.78)),
            origin=Origin(xyz=(0.0, -1.235, -1.55)),
            material=glass,
            name="rear_window",
        )
        capsule.visual(
            Box((0.035, 1.52, 0.68)),
            origin=Origin(xyz=(1.165, 0.0, -1.55)),
            material=glass,
            name="side_window_0",
        )
        capsule.visual(
            Box((0.035, 1.52, 0.68)),
            origin=Origin(xyz=(-1.165, 0.0, -1.55)),
            material=glass,
            name="side_window_1",
        )
        capsule.visual(
            Box((2.05, 2.10, 0.09)),
            origin=Origin(xyz=(0.0, 0.0, -2.67)),
            material=rubber,
            name="lower_gasket",
        )

        angle = TAU * i / capsule_count
        pivot = (pivot_radius * math.cos(angle), 0.0, pivot_radius * math.sin(angle))
        model.articulation(
            f"capsule_pivot_{i}",
            ArticulationType.CONTINUOUS,
            parent=wheel,
            child=capsule,
            origin=Origin(xyz=pivot),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2500.0, velocity=0.8),
            mimic=Mimic(joint=main_joint.name, multiplier=-1.0, offset=0.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    support = object_model.get_part("support")
    wheel = object_model.get_part("wheel")
    main = object_model.get_articulation("main_axle")

    ctx.allow_overlap(
        support,
        wheel,
        elem_a="main_axle",
        elem_b="hub_shell",
        reason="The static axle is intentionally captured inside the rotating hub proxy.",
    )
    ctx.expect_within(
        support,
        wheel,
        axes="xz",
        inner_elem="main_axle",
        outer_elem="hub_shell",
        margin=0.0,
        name="static axle is centered inside hub",
    )
    ctx.expect_overlap(
        support,
        wheel,
        axes="y",
        elem_a="main_axle",
        elem_b="hub_shell",
        min_overlap=1.0,
        name="hub remains threaded on axle",
    )
    for flange in ("front_hub_flange", "rear_hub_flange"):
        ctx.allow_overlap(
            support,
            wheel,
            elem_a="main_axle",
            elem_b=flange,
            reason="The visible hub flange is modeled as a solid collar around the static axle.",
        )
        ctx.expect_within(
            support,
            wheel,
            axes="xz",
            inner_elem="main_axle",
            outer_elem=flange,
            margin=0.0,
            name=f"axle is centered in {flange}",
        )
        ctx.expect_overlap(
            support,
            wheel,
            axes="y",
            elem_a="main_axle",
            elem_b=flange,
            min_overlap=0.15,
            name=f"axle passes through {flange}",
        )

    for i in range(12):
        capsule = object_model.get_part(f"capsule_{i}")
        pivot = object_model.get_articulation(f"capsule_pivot_{i}")
        ctx.check(
            f"capsule {i} pivot uses leveling mimic",
            pivot.mimic is not None and pivot.mimic.joint == main.name and abs(pivot.mimic.multiplier + 1.0) < 1e-9,
            details=f"mimic={pivot.mimic}",
        )
        ctx.allow_overlap(
            wheel,
            capsule,
            elem_a=f"pivot_socket_{i}",
            elem_b="pivot_pin",
            reason="The capsule pin is intentionally captured in the rim-arm pivot socket.",
        )
        ctx.expect_overlap(
            wheel,
            capsule,
            axes="xyz",
            elem_a=f"pivot_socket_{i}",
            elem_b="pivot_pin",
            min_overlap=0.12,
            name=f"capsule {i} pin is clipped to socket",
        )

    for i in (2, 3, 4):
        capsule = object_model.get_part(f"capsule_{i}")
        for arm, hanger, side in (
            (f"front_rim_arm_{i}", "hanger_0", "front"),
            (f"rear_rim_arm_{i}", "hanger_1", "rear"),
        ):
            ctx.allow_overlap(
                wheel,
                capsule,
                elem_a=arm,
                elem_b=hanger,
                reason="At the upper yoke, the rim arm is intentionally nested against the capsule hanger strap at the shared pivot bracket.",
            )
            ctx.expect_overlap(
                wheel,
                capsule,
                axes="xyz",
                elem_a=arm,
                elem_b=hanger,
                min_overlap=0.05,
                name=f"capsule {i} {side} yoke captures hanger strap",
            )

    # The bottom capsule must clear the foundation while hanging below its pin.
    bottom_capsule = object_model.get_part("capsule_9")
    ctx.expect_gap(
        bottom_capsule,
        support,
        axis="z",
        positive_elem="shell",
        negative_elem="foundation",
        min_gap=0.05,
        name="lowest capsule clears foundation",
    )

    # At a quarter-turn pose, the mimic leveling joints should keep the cabin
    # body vertically suspended below its pivot rather than rotating sideways
    # with the wheel rim.
    side_capsule = object_model.get_part("capsule_0")
    with ctx.pose({main: math.pi / 2.0}):
        pivot_pos = ctx.part_world_position(side_capsule)
        shell_aabb = ctx.part_element_world_aabb(side_capsule, elem="shell")
        ok = pivot_pos is not None and shell_aabb is not None and shell_aabb[1][2] < pivot_pos[2] - 0.35
        ctx.check(
            "leveled capsule hangs below pivot at quarter turn",
            ok,
            details=f"pivot={pivot_pos}, shell_aabb={shell_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
