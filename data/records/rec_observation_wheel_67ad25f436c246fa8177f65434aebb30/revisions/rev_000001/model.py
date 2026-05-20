from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, cos, hypot, pi, sin, sqrt

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
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = hypot(dx, dy)
    yaw = atan2(dy, dx)
    pitch = atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _xz_section(
    width: float,
    height: float,
    corner_radius: float,
    *,
    y: float,
    z_center: float,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z + z_center)
        for x, z in rounded_rect_profile(
            width, height, corner_radius, corner_segments=8
        )
    ]


def _build_capsule_mesh():
    shell_geom = section_loft(
        [
            _xz_section(1.38, 1.98, 0.16, y=-0.28, z_center=-1.80),
            _xz_section(1.60, 2.18, 0.22, y=0.00, z_center=-1.76),
            _xz_section(1.38, 1.98, 0.16, y=0.28, z_center=-1.80),
        ]
    )
    return _save_mesh("observation_capsule_shell", shell_geom)


def _build_capsule_part(
    model: ArticulatedObject,
    name: str,
    *,
    shell_mesh,
    cabin_shell,
    cabin_frame,
    glass_material,
):
    capsule = model.part(name)
    capsule.inertial = Inertial.from_geometry(
        Box((1.68, 0.72, 2.86)),
        mass=850.0,
        origin=Origin(xyz=(0.0, 0.0, -1.42)),
    )
    capsule.visual(
        Cylinder(radius=0.085, length=0.10),
        origin=Origin(xyz=(0.0, 0.28, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=cabin_frame,
        name="pivot_ear_front",
    )
    capsule.visual(
        Cylinder(radius=0.085, length=0.10),
        origin=Origin(xyz=(0.0, -0.28, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=cabin_frame,
        name="pivot_ear_back",
    )
    capsule.visual(
        Box((0.18, 0.56, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, -0.11)),
        material=cabin_frame,
        name="top_mount",
    )
    capsule.visual(
        Box((0.10, 0.08, 0.76)),
        origin=Origin(xyz=(0.0, 0.22, -0.49)),
        material=cabin_frame,
        name="front_hanger",
    )
    capsule.visual(
        Box((0.10, 0.08, 0.76)),
        origin=Origin(xyz=(0.0, -0.22, -0.49)),
        material=cabin_frame,
        name="rear_hanger",
    )
    capsule.visual(
        Box((0.56, 0.46, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, -0.83)),
        material=cabin_frame,
        name="roof_bridge",
    )
    capsule.visual(
        shell_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=cabin_shell,
        name="capsule_shell",
    )
    capsule.visual(
        Box((1.16, 0.52, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, -2.81)),
        material=cabin_shell,
        name="base_ring",
    )
    capsule.visual(
        Box((1.00, 0.06, 1.18)),
        origin=Origin(xyz=(0.0, 0.25, -1.71)),
        material=glass_material,
        name="front_window",
    )
    capsule.visual(
        Box((1.00, 0.06, 1.18)),
        origin=Origin(xyz=(0.0, -0.25, -1.71)),
        material=glass_material,
        name="rear_window",
    )
    return capsule


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="city_observation_wheel")

    support_white = model.material("support_white", rgba=(0.92, 0.94, 0.96, 1.0))
    wheel_white = model.material("wheel_white", rgba=(0.96, 0.97, 0.98, 1.0))
    machinery_dark = model.material("machinery_dark", rgba=(0.23, 0.26, 0.30, 1.0))
    deck_gray = model.material("deck_gray", rgba=(0.48, 0.51, 0.55, 1.0))
    steel = model.material("steel", rgba=(0.69, 0.72, 0.75, 1.0))
    glass_blue = model.material("glass_blue", rgba=(0.60, 0.84, 0.94, 0.42))
    cabin_shell = model.material("cabin_shell", rgba=(0.87, 0.90, 0.94, 1.0))
    cabin_frame = model.material("cabin_frame", rgba=(0.29, 0.33, 0.38, 1.0))
    accent_red = model.material("accent_red", rgba=(0.70, 0.11, 0.12, 1.0))

    hub_height = 12.8
    wheel_radius = 7.15
    pivot_radius = 8.35
    wheel_half_depth = 0.72
    support_frame = model.part("support_frame")
    support_frame.inertial = Inertial.from_geometry(
        Box((14.6, 5.2, 13.4)),
        mass=68000.0,
        origin=Origin(xyz=(0.0, 0.0, 6.7)),
    )
    support_frame.visual(
        Box((14.6, 5.2, 0.55)),
        origin=Origin(xyz=(0.0, 0.0, 0.275)),
        material=deck_gray,
        name="base_plinth",
    )
    support_frame.visual(
        Box((3.4, 1.8, 0.28)),
        origin=Origin(xyz=(0.0, 0.0, 0.69)),
        material=steel,
        name="boarding_platform",
    )
    for x in (-5.10, 5.10):
        for y in (-1.55, 1.55):
            support_frame.visual(
                Box((1.50, 1.10, 0.36)),
                origin=Origin(xyz=(x, y, 0.73)),
                material=machinery_dark,
                name=f"footing_{'pos' if x > 0 else 'neg'}x_{'front' if y > 0 else 'rear'}",
            )

    for y in (-1.55, 1.55):
        _add_member(
            support_frame,
            (-5.10, y, 0.55),
            (0.0, y, hub_height),
            0.24,
            support_white,
            name=f"{'front' if y > 0 else 'rear'}_left_leg",
        )
        _add_member(
            support_frame,
            (5.10, y, 0.55),
            (0.0, y, hub_height),
            0.24,
            support_white,
            name=f"{'front' if y > 0 else 'rear'}_right_leg",
        )
        support_frame.visual(
            Box((1.18, 0.34, 1.04)),
            origin=Origin(xyz=(0.0, 1.42 if y > 0 else -1.42, hub_height)),
            material=machinery_dark,
            name=f"{'front' if y > 0 else 'rear'}_bearing_house",
        )
        support_frame.visual(
            Box((1.45, 0.64, 0.34)),
            origin=Origin(xyz=(0.0, y, hub_height + 0.58)),
            material=machinery_dark,
            name=f"{'front' if y > 0 else 'rear'}_crown_beam",
        )

    _add_member(
        support_frame,
        (-5.00, -1.55, 0.82),
        (-5.00, 1.55, 0.82),
        0.12,
        steel,
        name="left_base_tie",
    )
    _add_member(
        support_frame,
        (5.00, -1.55, 0.82),
        (5.00, 1.55, 0.82),
        0.12,
        steel,
        name="right_base_tie",
    )
    wheel = model.part("wheel")
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius + 0.9, length=2.5),
        mass=32000.0,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    outer_rim_mesh = _save_mesh(
        "observation_wheel_outer_rim",
        TorusGeometry(radius=wheel_radius, tube=0.16, radial_segments=22, tubular_segments=112).rotate_x(
            pi / 2.0
        ),
    )
    inner_rim_mesh = _save_mesh(
        "observation_wheel_inner_rim",
        TorusGeometry(radius=4.65, tube=0.06, radial_segments=16, tubular_segments=88).rotate_x(
            pi / 2.0
        ),
    )
    wheel.visual(
        Cylinder(radius=0.18, length=2.50),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=machinery_dark,
        name="main_axle",
    )
    wheel.visual(
        Cylinder(radius=0.78, length=1.62),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=machinery_dark,
        name="hub_drum",
    )
    wheel.visual(
        Cylinder(radius=1.32, length=0.14),
        origin=Origin(xyz=(0.0, 0.55, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=accent_red,
        name="front_hub_disc",
    )
    wheel.visual(
        Cylinder(radius=1.32, length=0.14),
        origin=Origin(xyz=(0.0, -0.55, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=accent_red,
        name="rear_hub_disc",
    )
    wheel.visual(
        Cylinder(radius=0.94, length=0.48),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hub_cap",
    )
    wheel.visual(
        outer_rim_mesh,
        origin=Origin(xyz=(0.0, wheel_half_depth, 0.0)),
        material=wheel_white,
        name="front_outer_rim",
    )
    wheel.visual(
        outer_rim_mesh,
        origin=Origin(xyz=(0.0, -wheel_half_depth, 0.0)),
        material=wheel_white,
        name="rear_outer_rim",
    )
    wheel.visual(
        inner_rim_mesh,
        origin=Origin(xyz=(0.0, wheel_half_depth, 0.0)),
        material=steel,
        name="front_inner_ring",
    )
    wheel.visual(
        inner_rim_mesh,
        origin=Origin(xyz=(0.0, -wheel_half_depth, 0.0)),
        material=steel,
        name="rear_inner_ring",
    )

    capsule_count = 12
    shell_mesh = _build_capsule_mesh()

    for index in range(capsule_count):
        angle = 2.0 * pi * index / capsule_count
        ca = cos(angle)
        sa = sin(angle)
        front_hub = (1.10 * ca, 0.55, 1.10 * sa)
        rear_hub = (1.10 * ca, -0.55, 1.10 * sa)
        front_mid = (4.65 * ca, wheel_half_depth, 4.65 * sa)
        rear_mid = (4.65 * ca, -wheel_half_depth, 4.65 * sa)
        front_rim = (wheel_radius * ca, wheel_half_depth, wheel_radius * sa)
        rear_rim = (wheel_radius * ca, -wheel_half_depth, wheel_radius * sa)
        pivot = (pivot_radius * ca, 0.0, pivot_radius * sa)
        front_pin = (pivot[0], 0.28, pivot[2])
        rear_pin = (pivot[0], -0.28, pivot[2])
        front_plate = (pivot[0], 0.45, pivot[2])
        rear_plate = (pivot[0], -0.45, pivot[2])

        _add_member(
            wheel,
            front_hub,
            front_mid,
            0.05,
            wheel_white,
            name=f"front_spoke_inner_{index:02d}",
        )
        _add_member(
            wheel,
            front_mid,
            front_rim,
            0.05,
            wheel_white,
            name=f"front_spoke_outer_{index:02d}",
        )
        _add_member(
            wheel,
            rear_hub,
            rear_mid,
            0.05,
            wheel_white,
            name=f"rear_spoke_inner_{index:02d}",
        )
        _add_member(
            wheel,
            rear_mid,
            rear_rim,
            0.05,
            wheel_white,
            name=f"rear_spoke_outer_{index:02d}",
        )
        _add_member(
            wheel,
            front_mid,
            rear_mid,
            0.045,
            steel,
            name=f"inner_tie_{index:02d}",
        )
        _add_member(
            wheel,
            front_rim,
            front_plate,
            0.055,
            machinery_dark,
            name=f"front_capsule_arm_{index:02d}",
        )
        _add_member(
            wheel,
            rear_rim,
            rear_plate,
            0.055,
            machinery_dark,
            name=f"rear_capsule_arm_{index:02d}",
        )
        wheel.visual(
            Box((0.16, 0.12, 0.18)),
            origin=Origin(xyz=front_plate),
            material=machinery_dark,
            name=f"front_pivot_plate_{index:02d}",
        )
        wheel.visual(
            Box((0.16, 0.12, 0.18)),
            origin=Origin(xyz=rear_plate),
            material=machinery_dark,
            name=f"rear_pivot_plate_{index:02d}",
        )
        wheel.visual(
            Cylinder(radius=0.05, length=0.10),
            origin=Origin(
                xyz=(front_pin[0], (front_pin[1] + front_plate[1]) * 0.5, front_pin[2]),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material=machinery_dark,
            name=f"front_pivot_pin_{index:02d}",
        )
        wheel.visual(
            Cylinder(radius=0.05, length=0.10),
            origin=Origin(
                xyz=(rear_pin[0], (rear_pin[1] + rear_plate[1]) * 0.5, rear_pin[2]),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material=machinery_dark,
            name=f"rear_pivot_pin_{index:02d}",
        )

        capsule = _build_capsule_part(
            model,
            f"capsule_{index:02d}",
            shell_mesh=shell_mesh,
            cabin_shell=cabin_shell,
            cabin_frame=cabin_frame,
            glass_material=glass_blue,
        )
        model.articulation(
            f"wheel_to_capsule_{index:02d}",
            ArticulationType.CONTINUOUS,
            parent=wheel,
            child=capsule,
            origin=Origin(xyz=pivot),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2500.0, velocity=1.8),
        )

    model.articulation(
        "support_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=support_frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, hub_height)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=950000.0, velocity=0.22),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    support_frame = object_model.get_part("support_frame")
    wheel = object_model.get_part("wheel")
    wheel_spin = object_model.get_articulation("support_to_wheel")

    capsule_parts = [object_model.get_part(f"capsule_{index:02d}") for index in range(12)]
    capsule_joints = [
        object_model.get_articulation(f"wheel_to_capsule_{index:02d}") for index in range(12)
    ]

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()

    ctx.check(
        "wheel_spin_is_continuous",
        wheel_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 6) for v in wheel_spin.axis) == (0.0, 1.0, 0.0),
        f"Expected wheel to rotate continuously about +Y, got type={wheel_spin.articulation_type} axis={wheel_spin.axis}",
    )
    ctx.expect_contact(wheel, support_frame, name="wheel_axle_seated_in_bearings")

    for capsule, joint in zip(capsule_parts, capsule_joints):
        ctx.check(
            f"{joint.name}_axis_is_horizontal",
            joint.articulation_type == ArticulationType.CONTINUOUS
            and tuple(round(v, 6) for v in joint.axis) == (0.0, 1.0, 0.0),
            f"Expected {joint.name} to be a continuous leveling pivot about +Y, got type={joint.articulation_type} axis={joint.axis}",
        )
        ctx.expect_contact(capsule, wheel, name=f"{capsule.name}_pivot_contacts_rim_arm")
        index = capsule.name.split("_")[-1]
        ctx.allow_overlap(
            capsule,
            wheel,
            elem_a="pivot_ear_front",
            elem_b=f"front_pivot_pin_{index}",
            reason="Ferris wheel cabin hinge pin passes through the front pivot ear.",
        )
        ctx.allow_overlap(
            capsule,
            wheel,
            elem_a="pivot_ear_back",
            elem_b=f"rear_pivot_pin_{index}",
            reason="Ferris wheel cabin hinge pin passes through the rear pivot ear.",
        )

    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    coordinated_angle = 0.35
    coordinated_pose = {wheel_spin: coordinated_angle}
    for joint in capsule_joints:
        coordinated_pose[joint] = -coordinated_angle

    with ctx.pose(coordinated_pose):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlaps_in_coordinated_spin_pose")
        ctx.expect_contact(wheel, support_frame, name="wheel_axle_stays_seated_when_spun")

        for capsule in capsule_parts:
            pivot = ctx.part_world_position(capsule)
            aabb = ctx.part_world_aabb(capsule)
            ok = (
                pivot is not None
                and aabb is not None
                and aabb[1][2] <= pivot[2] + 0.20
                and aabb[0][2] <= pivot[2] - 1.85
            )
            details = (
                f"pivot={pivot}, aabb={aabb}"
                if pivot is not None and aabb is not None
                else "missing pose-space measurement"
            )
            ctx.check(
                f"{capsule.name}_hangs_below_its_pivot",
                ok,
                details,
            )
            ctx.expect_contact(capsule, wheel, name=f"{capsule.name}_pivot_contact_in_spin_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
