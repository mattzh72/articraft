from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


AXLE_Z = 7.20
RIM_RADIUS = 5.00
PIVOT_RADIUS = 5.08
PIVOT_Y = 1.12
CABIN_COUNT = 8


def _tube_mesh(name: str, points, *, radius: float, segments: int = 18):
    return mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=3,
            radial_segments=segments,
            cap_ends=True,
        ),
        name,
    )


def _capsule_body_mesh(name: str):
    # A glazed, rounded observation capsule: long horizontally, shallow in depth.
    body = CapsuleGeometry(radius=0.36, length=0.72, radial_segments=32, height_segments=10)
    body.rotate_y(pi / 2.0).scale(1.0, 0.75, 1.0)
    return mesh_from_geometry(body, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="modern_observation_wheel")

    white = model.material("white_painted_steel", rgba=(0.88, 0.90, 0.91, 1.0))
    dark = model.material("dark_bearing_steel", rgba=(0.12, 0.13, 0.14, 1.0))
    silver = model.material("brushed_aluminum", rgba=(0.70, 0.73, 0.76, 1.0))
    concrete = model.material("pale_concrete", rgba=(0.58, 0.58, 0.55, 1.0))
    glass = model.material("blue_tinted_glass", rgba=(0.38, 0.68, 0.88, 0.58))
    window = model.material("dark_window_band", rgba=(0.05, 0.11, 0.17, 0.86))

    support = model.part("support_frame")
    support.visual(
        Box((8.4, 3.0, 0.26)),
        origin=Origin(xyz=(0.0, 0.0, 0.13)),
        material=concrete,
        name="foundation_slab",
    )
    support.visual(
        Cylinder(radius=0.16, length=1.72),
        origin=Origin(xyz=(0.0, 0.0, AXLE_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="fixed_axle",
    )
    support.visual(
        Cylinder(radius=0.42, length=0.32),
        origin=Origin(xyz=(0.0, 0.72, AXLE_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="front_bearing_housing",
    )
    support.visual(
        Cylinder(radius=0.42, length=0.32),
        origin=Origin(xyz=(0.0, -0.72, AXLE_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="rear_bearing_housing",
    )
    for side_name, y in (("front", 0.68), ("rear", -0.68)):
        support.visual(
            _tube_mesh(
                f"{side_name}_leg_0",
                [(-3.25, y, 0.25), (-2.25, y * 0.96, 2.6), (-0.06, y * 1.18, AXLE_Z)],
                radius=0.095,
                segments=20,
            ),
            material=white,
            name=f"{side_name}_leg_0",
        )
        support.visual(
            _tube_mesh(
                f"{side_name}_leg_1",
                [(3.25, y, 0.25), (2.25, y * 0.96, 2.6), (0.06, y * 1.18, AXLE_Z)],
                radius=0.095,
                segments=20,
            ),
            material=white,
            name=f"{side_name}_leg_1",
        )
        support.visual(
            _tube_mesh(
                f"{side_name}_cross_tie",
                [(-3.05, y, 0.74), (0.0, y, 1.04), (3.05, y, 0.74)],
                radius=0.052,
                segments=16,
            ),
            material=white,
            name=f"{side_name}_cross_tie",
        )
    support.inertial = Inertial.from_geometry(
        Box((8.4, 3.0, AXLE_Z + 0.8)),
        mass=22000.0,
        origin=Origin(xyz=(0.0, 0.0, (AXLE_Z + 0.8) / 2.0)),
    )

    wheel = model.part("wheel")
    wheel.visual(
        mesh_from_geometry(
            TorusGeometry(radius=RIM_RADIUS, tube=0.055, radial_segments=18, tubular_segments=128),
            "slender_rim",
        ),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=white,
        name="rim",
    )
    wheel.visual(
        mesh_from_geometry(
            TorusGeometry(radius=RIM_RADIUS - 0.42, tube=0.032, radial_segments=14, tubular_segments=112),
            "inner_tension_ring",
        ),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="inner_ring",
    )
    wheel.visual(
        Cylinder(radius=0.70, length=0.96),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="hub_drum",
    )
    wheel.visual(
        Cylinder(radius=0.48, length=1.08),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="hub_core",
    )
    for index in range(16):
        theta = 2.0 * pi * index / 16.0
        wheel.visual(
            _tube_mesh(
                f"spoke_{index}",
                [
                    (0.58 * cos(theta), 0.0, 0.58 * sin(theta)),
                    (2.55 * cos(theta), 0.0, 2.55 * sin(theta)),
                    ((RIM_RADIUS - 0.03) * cos(theta), 0.0, (RIM_RADIUS - 0.03) * sin(theta)),
                ],
                radius=0.030 if index % 2 else 0.038,
                segments=14,
            ),
            material=silver,
            name=f"spoke_{index}",
        )
    for index in range(CABIN_COUNT):
        theta = 2.0 * pi * index / CABIN_COUNT
        rim_x = RIM_RADIUS * cos(theta)
        rim_z = RIM_RADIUS * sin(theta)
        x = PIVOT_RADIUS * cos(theta)
        z = PIVOT_RADIUS * sin(theta)
        wheel.visual(
            _tube_mesh(
                f"hanger_arm_{index}",
                [(rim_x, 0.035, rim_z), (x, 0.48, z), (x, PIVOT_Y - 0.19, z)],
                radius=0.038,
                segments=16,
            ),
            material=dark,
            name=f"hanger_arm_{index}",
        )
        wheel.visual(
            Cylinder(radius=0.060, length=0.10),
            origin=Origin(xyz=(x, PIVOT_Y - 0.19, z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=dark,
            name=f"pivot_socket_{index}",
        )
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=RIM_RADIUS, length=0.24),
        mass=9800.0,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )

    cabin_body = _capsule_body_mesh("glazed_capsule_body")
    hanger_mesh = _tube_mesh(
        "capsule_hanger_yoke",
        [
            (0.0, -0.11, 0.0),
            (-0.30, -0.10, -0.30),
            (-0.40, 0.0, -0.49),
            (-0.30, 0.10, -0.30),
            (0.0, 0.11, 0.0),
        ],
        radius=0.030,
        segments=14,
    )
    for index in range(CABIN_COUNT):
        cabin = model.part(f"capsule_{index}")
        cabin.visual(
            Cylinder(radius=0.050, length=0.38),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=dark,
            name="pivot_pin",
        )
        cabin.visual(hanger_mesh, material=dark, name="hanger_yoke")
        cabin.visual(
            cabin_body,
            origin=Origin(xyz=(0.0, 0.0, -0.86)),
            material=glass,
            name="capsule_body",
        )
        cabin.visual(
            Box((1.03, 0.018, 0.34)),
            origin=Origin(xyz=(0.0, 0.258, -0.86)),
            material=window,
            name="front_window",
        )
        cabin.visual(
            Box((1.03, 0.018, 0.34)),
            origin=Origin(xyz=(0.0, -0.258, -0.86)),
            material=window,
            name="rear_window",
        )
        cabin.visual(
            Box((1.34, 0.035, 0.045)),
            origin=Origin(xyz=(0.0, 0.0, -0.48)),
            material=white,
            name="roof_frame",
        )
        cabin.visual(
            Box((1.18, 0.030, 0.040)),
            origin=Origin(xyz=(0.0, 0.0, -1.22)),
            material=white,
            name="floor_frame",
        )
        cabin.inertial = Inertial.from_geometry(
            Box((1.45, 0.58, 0.82)),
            mass=750.0,
            origin=Origin(xyz=(0.0, 0.0, -0.86)),
        )

    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=support,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, AXLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=180000.0, velocity=0.20),
    )
    for index in range(CABIN_COUNT):
        theta = 2.0 * pi * index / CABIN_COUNT
        model.articulation(
            f"capsule_pivot_{index}",
            ArticulationType.CONTINUOUS,
            parent=wheel,
            child=f"capsule_{index}",
            origin=Origin(xyz=(PIVOT_RADIUS * cos(theta), PIVOT_Y, PIVOT_RADIUS * sin(theta))),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=900.0, velocity=0.8),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support_frame")
    wheel = object_model.get_part("wheel")
    spin = object_model.get_articulation("wheel_spin")

    ctx.allow_overlap(
        support,
        wheel,
        elem_a="fixed_axle",
        elem_b="hub_drum",
        reason="The stationary axle is intentionally represented as passing through the rotating hub bearing.",
    )
    ctx.allow_overlap(
        support,
        wheel,
        elem_a="fixed_axle",
        elem_b="hub_core",
        reason="The dark hub core is the visible rotating bearing sleeve around the stationary axle.",
    )
    ctx.expect_within(
        support,
        wheel,
        axes="xz",
        inner_elem="fixed_axle",
        outer_elem="hub_drum",
        margin=0.0,
        name="fixed axle lies within the hub bearing cross-section",
    )
    ctx.expect_overlap(
        support,
        wheel,
        axes="y",
        elem_a="fixed_axle",
        elem_b="hub_drum",
        min_overlap=0.85,
        name="fixed axle spans through the broad hub",
    )
    ctx.expect_within(
        support,
        wheel,
        axes="xz",
        inner_elem="fixed_axle",
        outer_elem="hub_core",
        margin=0.0,
        name="fixed axle lies within the central hub sleeve",
    )

    ctx.check(
        "main wheel uses a horizontal spin axis",
        spin is not None and tuple(round(v, 3) for v in spin.axis) == (0.0, 1.0, 0.0),
        details=f"axis={None if spin is None else spin.axis}",
    )

    for index in range(CABIN_COUNT):
        joint = object_model.get_articulation(f"capsule_pivot_{index}")
        cabin_i = object_model.get_part(f"capsule_{index}")
        ctx.check(
            f"capsule_{index}_has_horizontal_pivot",
            joint is not None and tuple(round(v, 3) for v in joint.axis) == (0.0, 1.0, 0.0),
            details=f"axis={None if joint is None else joint.axis}",
        )
        ctx.allow_overlap(
            cabin_i,
            wheel,
            elem_a="pivot_pin",
            elem_b=f"pivot_socket_{index}",
            reason="The capsule pivot pin is intentionally captured inside the wheel-side socket.",
        )
        ctx.expect_within(
            cabin_i,
            wheel,
            axes="xz",
            inner_elem="pivot_pin",
            outer_elem=f"pivot_socket_{index}",
            margin=0.0,
            name=f"capsule_{index}_pin_centered_in_socket",
        )
        ctx.expect_overlap(
            cabin_i,
            wheel,
            axes="y",
            elem_a="pivot_pin",
            elem_b=f"pivot_socket_{index}",
            min_overlap=0.04,
            name=f"capsule_{index}_pin_captured_by_socket",
        )

    capsule = object_model.get_part("capsule_0")
    pivot = object_model.get_articulation("capsule_pivot_0")
    ctx.expect_gap(
        capsule,
        wheel,
        axis="y",
        positive_elem="capsule_body",
        negative_elem="rim",
        min_gap=0.10,
        name="capsules are mounted outside the rim plane",
    )

    rest_pos = ctx.part_world_position(capsule)
    with ctx.pose({spin: 0.55}):
        spun_pos = ctx.part_world_position(capsule)
    ctx.check(
        "wheel spin carries capsule pivots around the axle",
        rest_pos is not None
        and spun_pos is not None
        and abs(spun_pos[0] - rest_pos[0]) > 0.20
        and abs(spun_pos[2] - rest_pos[2]) > 0.20,
        details=f"rest={rest_pos}, spun={spun_pos}",
    )

    rest_body = ctx.part_element_world_aabb(capsule, elem="capsule_body")
    with ctx.pose({pivot: 0.45}):
        tilted_body = ctx.part_element_world_aabb(capsule, elem="capsule_body")
    if rest_body is not None and tilted_body is not None:
        rest_center_x = (float(rest_body[0][0]) + float(rest_body[1][0])) * 0.5
        tilted_center_x = (float(tilted_body[0][0]) + float(tilted_body[1][0])) * 0.5
        moved = abs(tilted_center_x - rest_center_x) > 0.18
    else:
        moved = False
        rest_center_x = tilted_center_x = None
    ctx.check(
        "capsule pivot rotates the enclosed cabin",
        moved,
        details=f"rest_center_x={rest_center_x}, tilted_center_x={tilted_center_x}",
    )

    return ctx.report()


object_model = build_object_model()
