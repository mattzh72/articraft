from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, cos, hypot, pi, sin

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
    mesh_from_geometry,
    rounded_rect_profile,
)


WHEEL_RADIUS = 11.5
RIM_TUBE_RADIUS = 0.22
WHEEL_SIDE_OFFSET = 0.62
AXLE_CENTER_Z = 17.0
YOKE_ATTACH_RADIUS = 11.72
YOKE_PIVOT_Y = 1.05
YOKE_PIVOT_Z = 0.84
CAPSULE_COUNT = 8


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return ((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2) ** 0.5


def _member_rpy(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = hypot(dx, dy)
    yaw = atan2(dy, dx)
    pitch = atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_cylinder_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_member_rpy(a, b)),
        material=material,
        name=name,
    )


def _add_box_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    width: float,
    depth: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Box((width, depth, _distance(a, b))),
        origin=Origin(xyz=_midpoint(a, b), rpy=_member_rpy(a, b)),
        material=material,
        name=name,
    )


def _circle_point(radius: float, angle: float, y: float = 0.0) -> tuple[float, float, float]:
    return (radius * cos(angle), y, radius * sin(angle))


def _capsule_shell_mesh():
    shell = ExtrudeGeometry(
        rounded_rect_profile(1.48, 1.62, 0.22, corner_segments=8),
        0.82,
        center=True,
    ).rotate_x(pi / 2.0)
    return mesh_from_geometry(shell, "observation_capsule_shell")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="waterfront_observation_wheel")

    pier = model.material("pier", rgba=(0.49, 0.55, 0.60, 1.0))
    support_white = model.material("support_white", rgba=(0.93, 0.95, 0.97, 1.0))
    wheel_white = model.material("wheel_white", rgba=(0.96, 0.97, 0.98, 1.0))
    steel = model.material("steel", rgba=(0.53, 0.56, 0.60, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.24, 0.27, 0.30, 1.0))
    capsule_white = model.material("capsule_white", rgba=(0.98, 0.98, 0.99, 1.0))
    capsule_trim = model.material("capsule_trim", rgba=(0.36, 0.40, 0.46, 1.0))
    glass = model.material("glass", rgba=(0.62, 0.79, 0.86, 0.40))

    capsule_shell = _capsule_shell_mesh()

    support = model.part("support_frame")
    support.inertial = Inertial.from_geometry(
        Box((18.0, 8.0, 29.0)),
        mass=48000.0,
        origin=Origin(xyz=(0.0, 0.0, 14.5)),
    )
    support.visual(
        Box((18.0, 8.0, 0.80)),
        origin=Origin(xyz=(0.0, 0.0, 0.40)),
        material=pier,
        name="pier_deck",
    )
    support.visual(
        Box((13.0, 5.0, 0.24)),
        origin=Origin(xyz=(0.0, 0.0, 0.92)),
        material=steel,
        name="deck_edge",
    )
    support.visual(
        Box((8.2, 1.05, 0.66)),
        origin=Origin(xyz=(0.0, 0.0, 1.13)),
        material=dark_steel,
        name="lower_tie_beam",
    )
    _add_box_member(
        support,
        (-5.9, 0.0, 0.80),
        (-1.75, 0.0, 16.35),
        width=0.60,
        depth=0.72,
        material=support_white,
        name="left_side_leg",
    )
    _add_box_member(
        support,
        (5.9, 0.0, 0.80),
        (1.75, 0.0, 16.35),
        width=0.60,
        depth=0.72,
        material=support_white,
        name="right_side_leg",
    )
    support.visual(
        Box((4.8, 0.22, 0.32)),
        origin=Origin(xyz=(0.0, 1.70, 15.70)),
        material=dark_steel,
        name="axle_crosshead",
    )
    support.visual(
        Box((4.8, 0.22, 0.32)),
        origin=Origin(xyz=(0.0, -1.70, 15.70)),
        material=dark_steel,
        name="axle_crosshead_rear",
    )
    support.visual(
        Box((0.88, 0.62, 1.22)),
        origin=Origin(xyz=(0.0, 1.35, 16.36)),
        material=support_white,
        name="front_bearing_pedestal",
    )
    support.visual(
        Box((0.88, 0.62, 1.22)),
        origin=Origin(xyz=(0.0, -1.35, 16.36)),
        material=support_white,
        name="rear_bearing_pedestal",
    )
    support.visual(
        Cylinder(radius=0.40, length=0.18),
        origin=Origin(xyz=(0.0, 1.13, AXLE_CENTER_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="front_bearing_collar",
    )
    support.visual(
        Cylinder(radius=0.40, length=0.18),
        origin=Origin(xyz=(0.0, -1.13, AXLE_CENTER_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="rear_bearing_collar",
    )

    wheel = model.part("wheel")
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=12.0, length=2.08),
        mass=16500.0,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    wheel.visual(
        Cylinder(radius=0.34, length=2.08),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="axle_shaft",
    )
    wheel.visual(
        Cylinder(radius=1.02, length=1.24),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hub_drum",
    )
    wheel.visual(
        Cylinder(radius=1.34, length=0.12),
        origin=Origin(xyz=(0.0, 0.68, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=wheel_white,
        name="front_hub_plate",
    )
    wheel.visual(
        Cylinder(radius=1.34, length=0.12),
        origin=Origin(xyz=(0.0, -0.68, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=wheel_white,
        name="rear_hub_plate",
    )

    rim_segments = 40
    for side_y in (-WHEEL_SIDE_OFFSET, WHEEL_SIDE_OFFSET):
        for segment_index in range(rim_segments):
            start_angle = (2.0 * pi * segment_index) / rim_segments
            end_angle = (2.0 * pi * (segment_index + 1)) / rim_segments
            _add_cylinder_member(
                wheel,
                _circle_point(WHEEL_RADIUS, start_angle, side_y),
                _circle_point(WHEEL_RADIUS, end_angle, side_y),
                radius=RIM_TUBE_RADIUS,
                material=wheel_white,
            )

    spoke_count = 16
    for side_y in (-WHEEL_SIDE_OFFSET, WHEEL_SIDE_OFFSET):
        for spoke_index in range(spoke_count):
            angle = (2.0 * pi * spoke_index) / spoke_count
            _add_cylinder_member(
                wheel,
                _circle_point(1.34, angle, side_y),
                _circle_point(WHEEL_RADIUS - RIM_TUBE_RADIUS, angle, side_y),
                radius=0.11,
                material=steel,
            )

    for capsule_index in range(CAPSULE_COUNT):
        angle = (pi / 2.0) - (2.0 * pi * capsule_index / CAPSULE_COUNT)
        _add_cylinder_member(
            wheel,
            _circle_point(WHEEL_RADIUS, angle, -WHEEL_SIDE_OFFSET),
            _circle_point(WHEEL_RADIUS, angle, WHEEL_SIDE_OFFSET),
            radius=0.08,
            material=steel,
        )
        _add_box_member(
            wheel,
            _circle_point(11.46, angle, WHEEL_SIDE_OFFSET),
            _circle_point(11.72, angle, WHEEL_SIDE_OFFSET),
            width=0.32,
            depth=0.20,
            material=steel,
            name=f"mount_pad_{capsule_index:02d}",
        )

    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=support,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, AXLE_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=280000.0, velocity=0.35),
    )

    for capsule_index in range(CAPSULE_COUNT):
        angle = (pi / 2.0) - (2.0 * pi * capsule_index / CAPSULE_COUNT)
        yoke = model.part(f"yoke_{capsule_index:02d}")
        yoke.inertial = Inertial.from_geometry(
            Box((0.36, 1.18, 1.16)),
            mass=240.0,
            origin=Origin(xyz=(0.0, 0.53, 0.42)),
        )
        yoke.visual(
            Box((0.34, 0.24, 0.30)),
            origin=Origin(xyz=(0.0, 0.0, -0.15)),
            material=steel,
            name="mount_shoe",
        )
        yoke.visual(
            Box((0.10, 0.18, 0.82)),
            origin=Origin(xyz=(-0.12, 0.0, 0.26)),
            material=wheel_white,
            name="hanger_post",
        )
        yoke.visual(
            Box((0.10, 0.56, 0.10)),
            origin=Origin(xyz=(-0.155, 0.28, 0.67)),
            material=wheel_white,
            name="transfer_arm",
        )
        yoke.visual(
            Box((0.10, 0.10, 0.30)),
            origin=Origin(xyz=(-0.19, YOKE_PIVOT_Y - 0.47, 0.82)),
            material=wheel_white,
            name="rear_upright",
        )
        yoke.visual(
            Box((0.10, 0.10, 0.30)),
            origin=Origin(xyz=(-0.19, YOKE_PIVOT_Y + 0.47, 0.82)),
            material=wheel_white,
            name="front_upright",
        )
        yoke.visual(
            Box((0.10, 0.94, 0.08)),
            origin=Origin(xyz=(-0.19, YOKE_PIVOT_Y, 0.97)),
            material=wheel_white,
            name="top_rail",
        )
        yoke.visual(
            Box((0.22, 0.08, 0.20)),
            origin=Origin(xyz=(-0.17, YOKE_PIVOT_Y + 0.47, YOKE_PIVOT_Z)),
            material=dark_steel,
            name="front_ear",
        )
        yoke.visual(
            Box((0.22, 0.08, 0.20)),
            origin=Origin(xyz=(-0.17, YOKE_PIVOT_Y - 0.47, YOKE_PIVOT_Z)),
            material=dark_steel,
            name="rear_ear",
        )

        capsule = model.part(f"capsule_{capsule_index:02d}")
        capsule.inertial = Inertial.from_geometry(
            Box((1.50, 0.88, 2.34)),
            mass=880.0,
            origin=Origin(xyz=(0.0, 0.0, -1.52)),
        )
        capsule.visual(
            Cylinder(radius=0.06, length=0.86),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=capsule_trim,
            name="pivot_tube",
        )
        capsule.visual(
            Box((0.10, 0.20, 0.10)),
            origin=Origin(xyz=(0.0, 0.0, -0.05)),
            material=capsule_trim,
            name="pivot_housing",
        )
        capsule.visual(
            Box((0.76, 0.18, 0.10)),
            origin=Origin(xyz=(0.0, 0.0, -0.14)),
            material=capsule_trim,
            name="roof_beam",
        )
        capsule.visual(
            Box((0.18, 0.16, 0.92)),
            origin=Origin(xyz=(0.0, 0.0, -0.56)),
            material=capsule_trim,
            name="hanger_stem",
        )
        capsule.visual(
            capsule_shell,
            origin=Origin(xyz=(0.0, 0.0, -1.72)),
            material=capsule_white,
            name="cabin_shell",
        )
        capsule.visual(
            Box((1.10, 0.05, 1.02)),
            origin=Origin(xyz=(0.0, 0.385, -1.72)),
            material=glass,
            name="front_window",
        )
        capsule.visual(
            Box((1.10, 0.05, 1.02)),
            origin=Origin(xyz=(0.0, -0.385, -1.72)),
            material=glass,
            name="rear_window",
        )

        model.articulation(
            f"wheel_to_yoke_{capsule_index:02d}",
            ArticulationType.FIXED,
            parent=wheel,
            child=yoke,
            origin=Origin(
                xyz=(
                    YOKE_ATTACH_RADIUS * cos(angle),
                    WHEEL_SIDE_OFFSET,
                    YOKE_ATTACH_RADIUS * sin(angle),
                ),
                rpy=(0.0, (pi / 2.0) - angle, 0.0),
            ),
        )
        model.articulation(
            f"yoke_to_capsule_{capsule_index:02d}",
            ArticulationType.CONTINUOUS,
            parent=yoke,
            child=capsule,
            origin=Origin(
                xyz=(0.0, YOKE_PIVOT_Y, YOKE_PIVOT_Z),
                rpy=(0.0, angle - (pi / 2.0), 0.0),
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=4000.0, velocity=2.5),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    support = object_model.get_part("support_frame")
    wheel = object_model.get_part("wheel")
    wheel_spin = object_model.get_articulation("wheel_spin")
    yokes = [object_model.get_part(f"yoke_{index:02d}") for index in range(CAPSULE_COUNT)]
    capsules = [object_model.get_part(f"capsule_{index:02d}") for index in range(CAPSULE_COUNT)]
    pivots = [
        object_model.get_articulation(f"yoke_to_capsule_{index:02d}")
        for index in range(CAPSULE_COUNT)
    ]

    ctx.check(
        "wheel rotates about horizontal axle",
        tuple(round(value, 6) for value in wheel_spin.axis) == (0.0, 1.0, 0.0),
        details=f"wheel axis was {wheel_spin.axis!r}",
    )
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    for yoke, capsule in zip(yokes, capsules):
        ctx.allow_overlap(
            wheel,
            yoke,
            reason="simplified rim clamp bracket shares volume with the wheel pad at the mount interface",
        )
        ctx.allow_overlap(
            yoke,
            capsule,
            reason="simplified pivot cartridge nests inside the yoke clevis at the capsule hanger",
        )
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.expect_contact(wheel, support, name="wheel axle is carried by support frame")

    for index, (yoke, capsule, pivot) in enumerate(zip(yokes, capsules, pivots)):
        ctx.expect_contact(yoke, wheel, name=f"{yoke.name} is clipped to the rim")
        ctx.expect_contact(capsule, yoke, name=f"{capsule.name} is clipped inside its yoke")
        ctx.expect_gap(
            capsule,
            wheel,
            axis="y",
            positive_elem="cabin_shell",
            min_gap=0.12,
            name=f"{capsule.name} cabin stays outside the wheel plane",
        )
        ctx.check(
            f"{pivot.name} is parallel to the main axle",
            tuple(round(value, 6) for value in pivot.axis) == (0.0, 1.0, 0.0),
            details=f"pivot axis was {pivot.axis!r}",
        )

    bottom_capsule = object_model.get_part("capsule_04")
    ctx.expect_gap(
        bottom_capsule,
        support,
        axis="z",
        positive_elem="cabin_shell",
        negative_elem="pier_deck",
        min_gap=0.45,
        name="lowest capsule clears the pier deck",
    )

    compensated_angle = 0.82
    pose_map = {wheel_spin: compensated_angle}
    for pivot in pivots:
        pose_map[pivot] = -compensated_angle

    with ctx.pose(pose_map):
        for yoke, capsule in zip(yokes, capsules):
            ctx.expect_contact(
                capsule,
                yoke,
                name=f"{capsule.name} remains clipped to its yoke in rotation",
            )
            pivot_position = ctx.part_world_position(capsule)
            shell_aabb = ctx.part_element_world_aabb(capsule, elem="cabin_shell")
            upright = False
            details = "missing pose measurement"
            if pivot_position is not None and shell_aabb is not None:
                shell_center = tuple(
                    (shell_aabb[0][axis] + shell_aabb[1][axis]) * 0.5 for axis in range(3)
                )
                dx = abs(shell_center[0] - pivot_position[0])
                dy = abs(shell_center[1] - pivot_position[1])
                dz = abs((shell_center[2] - pivot_position[2]) + 1.72)
                upright = dx <= 0.07 and dy <= 0.07 and dz <= 0.08
                details = f"offsets were dx={dx:.3f}, dy={dy:.3f}, dz={dz:.3f}"
            ctx.check(
                f"{capsule.name} stays upright in compensated wheel pose",
                upright,
                details=details,
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
