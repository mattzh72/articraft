from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


WHEEL_CENTER_Z = 19.8
RIM_RADIUS = 14.0
RIM_TUBE_RADIUS = 0.18
RING_OFFSET_Y = 0.62
HUB_RADIUS = 2.25
HUB_LENGTH = 1.50
CAPSULE_COUNT = 12
MOUNT_BLOCK_LENGTH = 0.42
MOUNT_OUTER_RADIUS = 14.34
HANGER_ARM_LENGTH = 0.72
MESH_REVISION = "r5"
_MESH_CACHE: dict[str, object] = {}


def _mesh(name: str, geometry):
    cached = _MESH_CACHE.get(name)
    if cached is not None:
        return cached
    mesh = mesh_from_geometry(geometry, f"{name}_{MESH_REVISION}")
    _MESH_CACHE[name] = mesh
    return mesh


def _polar_xz(radius: float, angle: float) -> tuple[float, float, float]:
    return (radius * math.cos(angle), 0.0, radius * math.sin(angle))


def _down_in_rotated_frame(angle: float, drop: float) -> tuple[float, float, float]:
    return (-math.sin(angle) * drop, 0.0, -math.cos(angle) * drop)


def _capsule_body_mesh():
    body_profile = rounded_rect_profile(1.70, 1.94, radius=0.34, corner_segments=12)
    return _mesh(
        "observation_capsule_body",
        ExtrudeGeometry(body_profile, 1.28, center=True).rotate_x(math.pi / 2.0),
    )


def _tube(name: str, points: list[tuple[float, float, float]], radius: float):
    return _mesh(
        name,
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=10,
            radial_segments=18,
            cap_ends=True,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="modern_observation_wheel")

    structure_white = model.material("structure_white", rgba=(0.92, 0.94, 0.96, 1.0))
    rim_silver = model.material("rim_silver", rgba=(0.76, 0.79, 0.82, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.20, 0.22, 0.25, 1.0))
    platform_gray = model.material("platform_gray", rgba=(0.58, 0.60, 0.63, 1.0))
    glass_blue = model.material("glass_blue", rgba=(0.62, 0.77, 0.88, 0.42))
    capsule_trim = model.material("capsule_trim", rgba=(0.86, 0.88, 0.90, 1.0))
    capsule_floor = model.material("capsule_floor", rgba=(0.34, 0.36, 0.39, 1.0))

    capsule_body_mesh = _capsule_body_mesh()
    rim_mesh = _mesh(
        "observation_wheel_rim",
        TorusGeometry(
            radius=RIM_RADIUS,
            tube=RIM_TUBE_RADIUS,
            radial_segments=20,
            tubular_segments=96,
        ).rotate_x(math.pi / 2.0),
    )

    support = model.part("support_frame")
    support.visual(
        Box((24.0, 6.4, 0.8)),
        origin=Origin(xyz=(0.0, 0.0, 0.4)),
        material=platform_gray,
        name="foundation",
    )
    support.visual(
        Box((7.8, 4.6, 0.35)),
        origin=Origin(xyz=(0.0, 0.0, 0.975)),
        material=platform_gray,
        name="loading_platform",
    )
    support.visual(
        Box((0.72, 0.72, 13.2)),
        origin=Origin(xyz=(0.0, 0.0, 7.4)),
        material=dark_steel,
        name="central_mast",
    )
    support.visual(
        Box((4.6, 0.90, 0.78)),
        origin=Origin(xyz=(0.0, 1.25, WHEEL_CENTER_Z - 0.55)),
        material=dark_steel,
        name="front_bearing_support",
    )
    support.visual(
        Box((4.6, 0.90, 0.78)),
        origin=Origin(xyz=(0.0, -1.25, WHEEL_CENTER_Z - 0.55)),
        material=dark_steel,
        name="rear_bearing_support",
    )
    support.visual(
        Cylinder(radius=1.08, length=0.26),
        origin=Origin(xyz=(0.0, 0.88, WHEEL_CENTER_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="front_bearing",
    )
    support.visual(
        Cylinder(radius=1.08, length=0.26),
        origin=Origin(xyz=(0.0, -0.88, WHEEL_CENTER_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="rear_bearing",
    )
    support.visual(
        Box((5.4, 0.40, 0.40)),
        origin=Origin(xyz=(0.0, 1.25, 14.6)),
        material=structure_white,
        name="front_tie_beam",
    )
    support.visual(
        Box((5.4, 0.40, 0.40)),
        origin=Origin(xyz=(0.0, -1.25, 14.6)),
        material=structure_white,
        name="rear_tie_beam",
    )
    support.visual(
        Box((4.6, 0.40, 0.40)),
        origin=Origin(xyz=(0.0, 0.0, 10.2)),
        material=structure_white,
        name="lower_cross_tie",
    )
    support.visual(
        Box((0.36, 0.36, 4.6)),
        origin=Origin(xyz=(0.0, 1.25, 16.9)),
        material=structure_white,
        name="front_center_upright",
    )
    support.visual(
        Box((0.36, 0.36, 4.6)),
        origin=Origin(xyz=(0.0, -1.25, 16.9)),
        material=structure_white,
        name="rear_center_upright",
    )

    leg_specs = [
        ("leg_front_left", (-8.9, 1.55, 0.8), (-5.2, 1.40, 9.0), (-1.95, 1.05, 18.9)),
        ("leg_front_right", (8.9, 1.55, 0.8), (5.2, 1.40, 9.0), (1.95, 1.05, 18.9)),
        ("leg_rear_left", (-8.9, -1.55, 0.8), (-5.2, -1.40, 9.0), (-1.95, -1.05, 18.9)),
        ("leg_rear_right", (8.9, -1.55, 0.8), (5.2, -1.40, 9.0), (1.95, -1.05, 18.9)),
    ]
    for name, start, mid, end in leg_specs:
        support.visual(
            _tube(f"observation_wheel_{name}", [start, mid, end], radius=0.28),
            material=structure_white,
            name=name,
        )

    brace_specs = []
    for name, start, end in brace_specs:
        support.visual(
            _tube(f"observation_wheel_{name}", [start, end], radius=0.16),
            material=structure_white,
            name=name,
        )

    support.inertial = Inertial.from_geometry(
        Box((24.0, 6.4, 19.0)),
        mass=95000.0,
        origin=Origin(xyz=(0.0, 0.0, 9.5)),
    )

    wheel = model.part("wheel_assembly")
    wheel.visual(
        Cylinder(radius=HUB_RADIUS, length=HUB_LENGTH),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hub_drum",
    )
    wheel.visual(
        Cylinder(radius=2.85, length=0.18),
        origin=Origin(xyz=(0.0, 0.56, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rim_silver,
        name="front_hub_flange",
    )
    wheel.visual(
        Cylinder(radius=2.85, length=0.18),
        origin=Origin(xyz=(0.0, -0.56, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rim_silver,
        name="rear_hub_flange",
    )
    wheel.visual(
        rim_mesh,
        origin=Origin(xyz=(0.0, RING_OFFSET_Y, 0.0)),
        material=rim_silver,
        name="front_rim",
    )
    wheel.visual(
        rim_mesh,
        origin=Origin(xyz=(0.0, -RING_OFFSET_Y, 0.0)),
        material=rim_silver,
        name="rear_rim",
    )

    for index in range(CAPSULE_COUNT):
        angle = math.pi / 2.0 - (math.tau * index / CAPSULE_COUNT)
        spoke_front_start = (
            2.78 * math.cos(angle),
            RING_OFFSET_Y,
            2.78 * math.sin(angle),
        )
        spoke_back_start = (
            2.78 * math.cos(angle + math.pi / CAPSULE_COUNT),
            -RING_OFFSET_Y,
            2.78 * math.sin(angle + math.pi / CAPSULE_COUNT),
        )
        spoke_front_end = (
            13.98 * math.cos(angle),
            RING_OFFSET_Y,
            13.98 * math.sin(angle),
        )
        spoke_back_end = (
            13.98 * math.cos(angle + math.pi / CAPSULE_COUNT),
            -RING_OFFSET_Y,
            13.98 * math.sin(angle + math.pi / CAPSULE_COUNT),
        )
        wheel.visual(
            _tube(f"wheel_spoke_front_{index:02d}", [spoke_front_start, spoke_front_end], radius=0.07),
            material=structure_white,
            name=f"spoke_front_{index:02d}",
        )
        wheel.visual(
            _tube(f"wheel_spoke_back_{index:02d}", [spoke_back_start, spoke_back_end], radius=0.07),
            material=structure_white,
            name=f"spoke_back_{index:02d}",
        )
        wheel.visual(
            Box((0.24, 0.30, 0.24)),
            origin=Origin(
                xyz=(RIM_RADIUS * math.cos(angle), RING_OFFSET_Y + 0.20, RIM_RADIUS * math.sin(angle)),
                rpy=(0.0, -angle, 0.0),
            ),
            material=dark_steel,
            name=f"mount_block_{index:02d}",
        )

    wheel.inertial = Inertial.from_geometry(
        Box((29.2, 1.8, 29.2)),
        mass=42000.0,
        origin=Origin(),
    )

    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=support,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, WHEEL_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=160000.0, velocity=0.25),
    )

    for index in range(CAPSULE_COUNT):
        angle = math.pi / 2.0 - (math.tau * index / CAPSULE_COUNT)

        hanger = model.part(f"hanger_arm_{index:02d}")
        hanger.visual(
            Box((0.28, 0.18, 0.20)),
            origin=Origin(xyz=(0.0, 0.09, 0.0)),
            material=dark_steel,
            name="mount_pad",
        )
        hanger.visual(
            Box((0.16, 0.56, 0.12)),
            origin=Origin(xyz=(0.0, 0.46, 0.0)),
            material=dark_steel,
            name="arm_beam",
        )
        hanger.visual(
            Box((0.28, 0.14, 0.18)),
            origin=Origin(xyz=(0.0, 0.81, 0.0)),
            material=dark_steel,
            name="pivot_yoke",
        )
        hanger.inertial = Inertial.from_geometry(
            Box((0.30, 0.88, 0.22)),
            mass=420.0,
            origin=Origin(xyz=(0.0, 0.44, 0.0)),
        )

        model.articulation(
            f"wheel_to_hanger_{index:02d}",
            ArticulationType.FIXED,
            parent=wheel,
            child=hanger,
            origin=Origin(
                xyz=(RIM_RADIUS * math.cos(angle), RING_OFFSET_Y + 0.35, RIM_RADIUS * math.sin(angle))
            ),
        )

        capsule = model.part(f"capsule_{index:02d}")
        capsule.visual(
            Cylinder(radius=0.09, length=0.22),
            origin=Origin(xyz=(0.0, 0.11, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=capsule_trim,
            name="pivot_block",
        )
        capsule.visual(
            Box((0.18, 0.22, 0.26)),
            origin=Origin(xyz=(0.0, 0.23, -0.13)),
            material=capsule_trim,
            name="hanger_neck",
        )
        capsule.visual(
            capsule_body_mesh,
            origin=Origin(xyz=(0.0, 0.78, -1.18)),
            material=glass_blue,
            name="cabin_body",
        )
        capsule.inertial = Inertial.from_geometry(
            Box((1.9, 1.4, 2.2)),
            mass=1850.0,
            origin=Origin(xyz=(0.0, 0.78, -1.12)),
        )

        model.articulation(
            f"capsule_pivot_{index:02d}",
            ArticulationType.CONTINUOUS,
            parent=hanger,
            child=capsule,
            origin=Origin(xyz=(0.0, 0.88, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2800.0, velocity=1.5),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support_frame")
    wheel = object_model.get_part("wheel_assembly")
    hanger_00 = object_model.get_part("hanger_arm_00")
    capsule_00 = object_model.get_part("capsule_00")
    bottom_capsule = object_model.get_part(f"capsule_{CAPSULE_COUNT // 2:02d}")
    wheel_spin = object_model.get_articulation("wheel_spin")
    capsule_00_pivot = object_model.get_articulation("capsule_pivot_00")
    capsule_pivots = [
        object_model.get_articulation(f"capsule_pivot_{index:02d}")
        for index in range(CAPSULE_COUNT)
    ]

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "capsule_count_present",
        all(object_model.get_part(f"capsule_{index:02d}") is not None for index in range(CAPSULE_COUNT)),
        details=f"expected {CAPSULE_COUNT} capsule parts",
    )
    ctx.check(
        "hanger_count_present",
        all(object_model.get_part(f"hanger_arm_{index:02d}") is not None for index in range(CAPSULE_COUNT)),
        details=f"expected {CAPSULE_COUNT} hanger parts",
    )
    ctx.check(
        "wheel_spin_axis_y",
        tuple(round(value, 6) for value in wheel_spin.axis) == (0.0, 1.0, 0.0),
        details=f"wheel_spin axis was {wheel_spin.axis}",
    )
    ctx.check(
        "capsule_pivot_axis_y",
        tuple(round(value, 6) for value in capsule_00_pivot.axis) == (0.0, 1.0, 0.0),
        details=f"capsule_pivot_00 axis was {capsule_00_pivot.axis}",
    )

    ctx.expect_contact(
        wheel,
        support,
        elem_a="hub_drum",
        elem_b="front_bearing",
        name="wheel_front_bearing_contact",
    )
    ctx.expect_contact(
        wheel,
        support,
        elem_a="hub_drum",
        elem_b="rear_bearing",
        name="wheel_rear_bearing_contact",
    )
    ctx.expect_contact(
        hanger_00,
        wheel,
        elem_b="mount_block_00",
        name="hanger_00_mount_contact",
    )
    ctx.expect_contact(
        capsule_00,
        hanger_00,
        elem_a="pivot_block",
        elem_b="pivot_yoke",
        name="capsule_00_pivot_contact",
    )
    ctx.expect_origin_distance(
        capsule_00,
        wheel,
        axes="xz",
        min_dist=13.8,
        max_dist=15.2,
        name="capsule_00_outboard_mounting",
    )
    ctx.expect_gap(
        bottom_capsule,
        support,
        axis="z",
        negative_elem="loading_platform",
        min_gap=0.45,
        name="bottom_capsule_loading_clearance",
    )

    leveled_pose = {wheel_spin: math.pi / 6.0}
    for pivot in capsule_pivots:
        leveled_pose[pivot] = -math.pi / 6.0
    with ctx.pose(leveled_pose):
        ctx.fail_if_parts_overlap_in_current_pose(name="wheel_rotated_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="wheel_rotated_pose_no_floating")

    with ctx.pose({capsule_00_pivot: 0.45}):
        ctx.fail_if_parts_overlap_in_current_pose(name="capsule_00_operating_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="capsule_00_operating_pose_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
