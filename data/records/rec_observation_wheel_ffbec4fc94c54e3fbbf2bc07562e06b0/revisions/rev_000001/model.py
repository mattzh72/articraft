from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

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
    mesh_from_geometry,
    tube_from_spline_points,
    wire_from_points,
)


WHEEL_RADIUS = 6.2
RIM_TUBE_RADIUS = 0.18
RIM_OFFSET_Y = 0.55
AXLE_HEIGHT = 8.92
SUPPORT_Y = 1.25
SUPPORT_FOOT_X = 3.8
GONDOLA_COUNT = 8
GONDOLA_PIVOT_RADIUS = 5.88


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _circle_points(radius: float, y: float, count: int = 24) -> list[tuple[float, float, float]]:
    return [
        (radius * cos((2.0 * pi * index) / count), y, radius * sin((2.0 * pi * index) / count))
        for index in range(count)
    ]


def _tube_mesh(
    name: str,
    points: list[tuple[float, float, float]],
    *,
    radius: float,
    closed_spline: bool = False,
    cap_ends: bool = True,
    samples_per_segment: int = 8,
    radial_segments: int = 18,
):
    return _save_mesh(
        name,
        tube_from_spline_points(
            points,
            radius=radius,
            closed_spline=closed_spline,
            cap_ends=cap_ends,
            samples_per_segment=samples_per_segment,
            radial_segments=radial_segments,
        ),
    )


def _polar_xz(radius: float, angle: float) -> tuple[float, float]:
    return radius * cos(angle), radius * sin(angle)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="observation_wheel")

    off_white = model.material("off_white", rgba=(0.92, 0.93, 0.91, 1.0))
    support_gray = model.material("support_gray", rgba=(0.72, 0.74, 0.77, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.24, 0.27, 1.0))
    axle_gray = model.material("axle_gray", rgba=(0.43, 0.45, 0.48, 1.0))
    rim_red = model.material("rim_red", rgba=(0.69, 0.18, 0.15, 1.0))
    deck_gray = model.material("deck_gray", rgba=(0.54, 0.56, 0.58, 1.0))
    gondola_red = model.material("gondola_red", rgba=(0.77, 0.19, 0.17, 1.0))
    gondola_blue = model.material("gondola_blue", rgba=(0.19, 0.42, 0.74, 1.0))
    gondola_yellow = model.material("gondola_yellow", rgba=(0.87, 0.67, 0.16, 1.0))
    gondola_green = model.material("gondola_green", rgba=(0.23, 0.58, 0.35, 1.0))
    canopy_cream = model.material("canopy_cream", rgba=(0.94, 0.92, 0.84, 1.0))

    support_base_z = 0.62
    support_frame_mesh = _save_mesh(
        "support_frame_v2",
        wire_from_points(
            [
                (-SUPPORT_FOOT_X, 0.0, support_base_z),
                (0.0, 0.0, AXLE_HEIGHT),
                (SUPPORT_FOOT_X, 0.0, support_base_z),
            ],
            radius=0.16,
            radial_segments=18,
            cap_ends=True,
            corner_mode="fillet",
            corner_radius=0.45,
            corner_segments=12,
        ),
    )

    front_rim_mesh = _tube_mesh(
        "front_rim_ring_v2",
        _circle_points(WHEEL_RADIUS, RIM_OFFSET_Y),
        radius=RIM_TUBE_RADIUS,
        closed_spline=True,
        cap_ends=False,
        samples_per_segment=10,
    )
    rear_rim_mesh = _tube_mesh(
        "rear_rim_ring_v2",
        _circle_points(WHEEL_RADIUS, -RIM_OFFSET_Y),
        radius=RIM_TUBE_RADIUS,
        closed_spline=True,
        cap_ends=False,
        samples_per_segment=10,
    )
    front_spoke_mesh = _tube_mesh(
        "front_spoke_v2",
        [(0.60, RIM_OFFSET_Y, 0.0), (6.05, RIM_OFFSET_Y, 0.0)],
        radius=0.07,
        samples_per_segment=2,
        radial_segments=16,
    )
    rear_spoke_mesh = _tube_mesh(
        "rear_spoke_v2",
        [(0.60, -RIM_OFFSET_Y, 0.0), (6.05, -RIM_OFFSET_Y, 0.0)],
        radius=0.07,
        samples_per_segment=2,
        radial_segments=16,
    )
    front_mount_arm_mesh = _tube_mesh(
        "front_mount_arm_v4",
        [(GONDOLA_PIVOT_RADIUS, 0.47, 0.0), (WHEEL_RADIUS, RIM_OFFSET_Y, 0.0)],
        radius=0.04,
        samples_per_segment=2,
        radial_segments=14,
    )
    rear_mount_arm_mesh = _tube_mesh(
        "rear_mount_arm_v4",
        [(GONDOLA_PIVOT_RADIUS, -0.47, 0.0), (WHEEL_RADIUS, -RIM_OFFSET_Y, 0.0)],
        radius=0.04,
        samples_per_segment=2,
        radial_segments=14,
    )

    base_frame = model.part("base_frame")
    base_frame.visual(
        Box((8.40, 3.10, 0.44)),
        origin=Origin(xyz=(0.0, 0.0, 0.22)),
        material=dark_steel,
        name="base_slab",
    )
    base_frame.visual(
        Box((8.40, 0.46, 0.44)),
        origin=Origin(xyz=(0.0, SUPPORT_Y, 0.22)),
        material=dark_steel,
        name="front_beam",
    )
    base_frame.visual(
        Box((8.40, 0.46, 0.44)),
        origin=Origin(xyz=(0.0, -SUPPORT_Y, 0.22)),
        material=dark_steel,
        name="rear_beam",
    )
    base_frame.visual(
        Box((0.42, 2.00, 0.38)),
        origin=Origin(xyz=(-2.75, 0.0, 0.19)),
        material=dark_steel,
        name="left_crossbeam",
    )
    base_frame.visual(
        Box((0.42, 2.00, 0.38)),
        origin=Origin(xyz=(2.75, 0.0, 0.19)),
        material=dark_steel,
        name="right_crossbeam",
    )
    base_frame.visual(
        Box((1.50, 1.60, 0.28)),
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
        material=deck_gray,
        name="service_deck",
    )
    base_frame.visual(
        Box((0.92, 1.20, 0.78)),
        origin=Origin(xyz=(0.0, 0.0, 0.67)),
        material=axle_gray,
        name="drive_house",
    )
    base_frame.inertial = Inertial.from_geometry(
        Box((8.40, 2.20, 1.20)),
        mass=12000.0,
        origin=Origin(xyz=(0.0, 0.0, 0.60)),
    )

    front_support = model.part("front_support")
    front_support.visual(
        Box((0.48, 0.20, 0.18)),
        origin=Origin(xyz=(-SUPPORT_FOOT_X, 0.0, 0.53)),
        material=support_gray,
        name="left_foot",
    )
    front_support.visual(
        Box((0.48, 0.20, 0.18)),
        origin=Origin(xyz=(SUPPORT_FOOT_X, 0.0, 0.53)),
        material=support_gray,
        name="right_foot",
    )
    front_support.visual(support_frame_mesh, material=off_white, name="a_frame")
    front_support.visual(
        Box((7.80, 0.18, 0.28)),
        origin=Origin(xyz=(0.0, 0.0, 0.64)),
        material=support_gray,
        name="base_tie",
    )
    front_support.visual(
        Box((1.58, 0.20, 0.94)),
        origin=Origin(xyz=(0.0, -0.25, AXLE_HEIGHT)),
        material=axle_gray,
        name="bearing_block",
    )
    front_support.inertial = Inertial.from_geometry(
        Box((7.80, 0.30, 8.60)),
        mass=1800.0,
        origin=Origin(xyz=(0.0, 0.0, 4.30)),
    )

    rear_support = model.part("rear_support")
    rear_support.visual(
        Box((0.48, 0.20, 0.18)),
        origin=Origin(xyz=(-SUPPORT_FOOT_X, 0.0, 0.53)),
        material=support_gray,
        name="left_foot",
    )
    rear_support.visual(
        Box((0.48, 0.20, 0.18)),
        origin=Origin(xyz=(SUPPORT_FOOT_X, 0.0, 0.53)),
        material=support_gray,
        name="right_foot",
    )
    rear_support.visual(support_frame_mesh, material=off_white, name="a_frame")
    rear_support.visual(
        Box((7.80, 0.18, 0.28)),
        origin=Origin(xyz=(0.0, 0.0, 0.64)),
        material=support_gray,
        name="base_tie",
    )
    rear_support.visual(
        Box((1.58, 0.20, 0.94)),
        origin=Origin(xyz=(0.0, 0.25, AXLE_HEIGHT)),
        material=axle_gray,
        name="bearing_block",
    )
    rear_support.inertial = Inertial.from_geometry(
        Box((7.80, 0.30, 8.60)),
        mass=1800.0,
        origin=Origin(xyz=(0.0, 0.0, 4.30)),
    )

    wheel = model.part("wheel")
    wheel.visual(
        Cylinder(radius=0.42, length=1.40),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=axle_gray,
        name="hub_barrel",
    )
    wheel.visual(
        Box((0.24, 0.20, 0.24)),
        origin=Origin(xyz=(0.0, 0.80, 0.0)),
        material=dark_steel,
        name="front_axle_stub",
    )
    wheel.visual(
        Box((0.24, 0.20, 0.24)),
        origin=Origin(xyz=(0.0, -0.80, 0.0)),
        material=dark_steel,
        name="rear_axle_stub",
    )
    wheel.visual(
        Cylinder(radius=0.72, length=0.12),
        origin=Origin(xyz=(0.0, RIM_OFFSET_Y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="front_hub_flange",
    )
    wheel.visual(
        Cylinder(radius=0.72, length=0.12),
        origin=Origin(xyz=(0.0, -RIM_OFFSET_Y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="rear_hub_flange",
    )
    wheel.visual(front_rim_mesh, material=rim_red, name="front_rim")
    wheel.visual(rear_rim_mesh, material=rim_red, name="rear_rim")
    for spoke_index in range(12):
        angle = (2.0 * pi * spoke_index) / 12.0
        wheel.visual(
            front_spoke_mesh,
            origin=Origin(rpy=(0.0, angle, 0.0)),
            material=off_white,
            name=f"front_spoke_{spoke_index:02d}",
        )
        wheel.visual(
            rear_spoke_mesh,
            origin=Origin(rpy=(0.0, angle, 0.0)),
            material=off_white,
            name=f"rear_spoke_{spoke_index:02d}",
        )
    for gondola_index in range(GONDOLA_COUNT):
        angle = (2.0 * pi * gondola_index) / GONDOLA_COUNT
        mount_x, mount_z = _polar_xz(GONDOLA_PIVOT_RADIUS, angle)
        wheel.visual(
            front_mount_arm_mesh,
            origin=Origin(rpy=(0.0, angle, 0.0)),
            material=support_gray,
            name=f"mount_{gondola_index:02d}_front_arm",
        )
        wheel.visual(
            rear_mount_arm_mesh,
            origin=Origin(rpy=(0.0, angle, 0.0)),
            material=support_gray,
            name=f"mount_{gondola_index:02d}_rear_arm",
        )
        wheel.visual(
            Box((0.22, 0.10, 0.22)),
            origin=Origin(xyz=(mount_x, 0.42, mount_z), rpy=(0.0, angle, 0.0)),
            material=axle_gray,
            name=f"mount_{gondola_index:02d}_front_plate",
        )
        wheel.visual(
            Box((0.22, 0.10, 0.22)),
            origin=Origin(xyz=(mount_x, -0.42, mount_z), rpy=(0.0, angle, 0.0)),
            material=axle_gray,
            name=f"mount_{gondola_index:02d}_rear_plate",
        )
    wheel.inertial = Inertial.from_geometry(
        Box((12.90, 1.90, 12.90)),
        mass=3500.0,
        origin=Origin(),
    )

    gondola_palette = [gondola_red, gondola_blue, gondola_yellow, gondola_green]
    for gondola_index in range(GONDOLA_COUNT):
        gondola_color = gondola_palette[gondola_index % len(gondola_palette)]
        gondola = model.part(f"gondola_{gondola_index:02d}")
        gondola.visual(
            Box((0.12, 0.10, 0.12)),
            origin=Origin(xyz=(0.0, 0.32, 0.0)),
            material=axle_gray,
            name="front_pivot_stub",
        )
        gondola.visual(
            Box((0.12, 0.10, 0.12)),
            origin=Origin(xyz=(0.0, -0.32, 0.0)),
            material=axle_gray,
            name="rear_pivot_stub",
        )
        gondola.visual(
            Box((0.10, 0.05, 0.58)),
            origin=Origin(xyz=(0.0, 0.28, -0.29)),
            material=dark_steel,
            name="front_hanger",
        )
        gondola.visual(
            Box((0.10, 0.05, 0.58)),
            origin=Origin(xyz=(0.0, -0.28, -0.29)),
            material=dark_steel,
            name="rear_hanger",
        )
        gondola.visual(
            Box((1.34, 0.74, 0.06)),
            origin=Origin(xyz=(0.0, 0.0, -0.55)),
            material=gondola_color,
            name="roof",
        )
        for post_index, (post_x, post_y) in enumerate(((-0.56, -0.27), (-0.56, 0.27), (0.56, -0.27), (0.56, 0.27))):
            gondola.visual(
                Box((0.05, 0.05, 0.50)),
                origin=Origin(xyz=(post_x, post_y, -0.82)),
                material=dark_steel,
                name=f"post_{post_index:02d}",
            )
        gondola.visual(
            Box((1.20, 0.62, 0.08)),
            origin=Origin(xyz=(0.0, 0.0, -1.10)),
            material=deck_gray,
            name="floor",
        )
        gondola.visual(
            Box((1.14, 0.04, 0.28)),
            origin=Origin(xyz=(0.0, 0.29, -0.88)),
            material=gondola_color,
            name="front_rail",
        )
        gondola.visual(
            Box((1.14, 0.04, 0.28)),
            origin=Origin(xyz=(0.0, -0.29, -0.88)),
            material=gondola_color,
            name="back_rail",
        )
        gondola.visual(
            Box((0.06, 0.54, 0.10)),
            origin=Origin(xyz=(-0.57, 0.0, -0.89)),
            material=gondola_color,
            name="left_side_rail",
        )
        gondola.visual(
            Box((0.06, 0.54, 0.10)),
            origin=Origin(xyz=(0.57, 0.0, -0.89)),
            material=gondola_color,
            name="right_side_rail",
        )
        gondola.visual(
            Box((0.92, 0.24, 0.12)),
            origin=Origin(xyz=(0.0, 0.0, -1.00)),
            material=canopy_cream,
            name="seat",
        )
        gondola.visual(
            Box((0.92, 0.06, 0.36)),
            origin=Origin(xyz=(0.0, -0.24, -0.84)),
            material=canopy_cream,
            name="seat_back",
        )
        gondola.inertial = Inertial.from_geometry(
            Box((1.34, 0.74, 1.20)),
            mass=160.0,
            origin=Origin(xyz=(0.0, 0.0, -0.72)),
        )

    model.articulation(
        "front_support_mount",
        ArticulationType.FIXED,
        parent=base_frame,
        child=front_support,
        origin=Origin(xyz=(0.0, SUPPORT_Y, 0.0)),
    )
    model.articulation(
        "rear_support_mount",
        ArticulationType.FIXED,
        parent=base_frame,
        child=rear_support,
        origin=Origin(xyz=(0.0, -SUPPORT_Y, 0.0)),
    )
    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=base_frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, AXLE_HEIGHT)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120000.0, velocity=0.35),
    )
    for gondola_index in range(GONDOLA_COUNT):
        angle = (2.0 * pi * gondola_index) / GONDOLA_COUNT
        pivot_x, pivot_z = _polar_xz(GONDOLA_PIVOT_RADIUS, angle)
        model.articulation(
            f"wheel_to_gondola_{gondola_index:02d}",
            ArticulationType.CONTINUOUS,
            parent=wheel,
            child=f"gondola_{gondola_index:02d}",
            origin=Origin(xyz=(pivot_x, 0.0, pivot_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=3500.0, velocity=2.5),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base_frame = object_model.get_part("base_frame")
    front_support = object_model.get_part("front_support")
    rear_support = object_model.get_part("rear_support")
    wheel = object_model.get_part("wheel")
    wheel_spin = object_model.get_articulation("wheel_spin")
    gondolas = [object_model.get_part(f"gondola_{index:02d}") for index in range(GONDOLA_COUNT)]
    gondola_joints = [object_model.get_articulation(f"wheel_to_gondola_{index:02d}") for index in range(GONDOLA_COUNT)]

    def _axis_is(axis, expected: tuple[float, float, float]) -> bool:
        return all(abs(float(actual) - target) <= 1e-9 for actual, target in zip(axis, expected))

    def _floor_center_z(part_name_index: int) -> float | None:
        floor_aabb = ctx.part_element_world_aabb(gondolas[part_name_index], elem="floor")
        if floor_aabb is None:
            return None
        return (floor_aabb[0][2] + floor_aabb[1][2]) * 0.5

    def _upright_pose(angle: float) -> dict[object, float]:
        pose_map: dict[object, float] = {wheel_spin: angle}
        for gondola_joint in gondola_joints:
            pose_map[gondola_joint] = -angle
        return pose_map

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "gondola_count",
        len(gondolas) == GONDOLA_COUNT,
        details=f"expected {GONDOLA_COUNT} gondolas, found {len(gondolas)}",
    )
    ctx.check(
        "wheel_spin_axis_is_horizontal",
        _axis_is(wheel_spin.axis, (0.0, 1.0, 0.0)),
        details=f"wheel axis was {wheel_spin.axis!r}",
    )
    for index, gondola_joint in enumerate(gondola_joints):
        ctx.check(
            f"gondola_{index:02d}_axis_is_horizontal",
            _axis_is(gondola_joint.axis, (0.0, 1.0, 0.0)),
            details=f"gondola joint axis was {gondola_joint.axis!r}",
        )

    wheel_aabb = ctx.part_world_aabb(wheel)
    if wheel_aabb is None:
        ctx.fail("wheel_aabb_available", "wheel world AABB was unavailable")
    else:
        wheel_diameter = wheel_aabb[1][2] - wheel_aabb[0][2]
        ctx.check(
            "wheel_diameter_realistic",
            12.2 <= wheel_diameter <= 13.2,
            details=f"wheel diameter was {wheel_diameter:.3f} m",
        )

    wheel_position = ctx.part_world_position(wheel)
    if wheel_position is None:
        ctx.fail("wheel_position_available", "wheel world position was unavailable")
    else:
        ctx.check(
            "axle_height_realistic",
            8.7 <= wheel_position[2] <= 9.1,
            details=f"axle height was {wheel_position[2]:.3f} m",
        )

    ctx.expect_contact(
        front_support,
        base_frame,
        elem_a="left_foot",
        elem_b="front_beam",
        name="front_support_left_foot_contact",
    )
    ctx.expect_contact(
        front_support,
        base_frame,
        elem_a="right_foot",
        elem_b="front_beam",
        name="front_support_right_foot_contact",
    )
    ctx.expect_contact(
        rear_support,
        base_frame,
        elem_a="left_foot",
        elem_b="rear_beam",
        name="rear_support_left_foot_contact",
    )
    ctx.expect_contact(
        rear_support,
        base_frame,
        elem_a="right_foot",
        elem_b="rear_beam",
        name="rear_support_right_foot_contact",
    )
    ctx.expect_contact(
        wheel,
        front_support,
        elem_a="front_axle_stub",
        elem_b="bearing_block",
        name="wheel_hub_contacts_front_support",
    )
    ctx.expect_contact(
        wheel,
        rear_support,
        elem_a="rear_axle_stub",
        elem_b="bearing_block",
        name="wheel_hub_contacts_rear_support",
    )

    for index, gondola in enumerate(gondolas):
        ctx.allow_overlap(
            gondola,
            wheel,
            elem_a="front_pivot_stub",
            elem_b=f"mount_{index:02d}_front_plate",
            reason="gondola front pivot pin is intentionally captured inside the wheel-side bracket",
        )
        ctx.allow_overlap(
            gondola,
            wheel,
            elem_a="rear_pivot_stub",
            elem_b=f"mount_{index:02d}_rear_plate",
            reason="gondola rear pivot pin is intentionally captured inside the wheel-side bracket",
        )
        ctx.expect_contact(
            gondola,
            wheel,
            elem_a="front_pivot_stub",
            elem_b=f"mount_{index:02d}_front_plate",
            name=f"gondola_{index:02d}_front_pivot_contact",
        )
        ctx.expect_contact(
            gondola,
            wheel,
            elem_a="rear_pivot_stub",
            elem_b=f"mount_{index:02d}_rear_plate",
            name=f"gondola_{index:02d}_rear_pivot_contact",
        )
        ctx.expect_origin_distance(
            gondola,
            wheel,
            axes="xz",
            min_dist=5.75,
            max_dist=6.00,
            name=f"gondola_{index:02d}_mounted_near_rim",
        )

    ctx.expect_gap(
        gondolas[6],
        base_frame,
        axis="z",
        min_gap=0.80,
        name="rest_pose_bottom_gondola_ground_clearance",
    )

    with ctx.pose(_upright_pose(pi / 2.0)):
        ctx.fail_if_parts_overlap_in_current_pose(name="quarter_turn_no_overlap")
        ctx.fail_if_isolated_parts(name="quarter_turn_no_floating")
        ctx.expect_gap(
            gondolas[4],
            base_frame,
            axis="z",
            min_gap=0.80,
            name="quarter_turn_bottom_gondola_ground_clearance",
        )
        gondola_pivot_position = ctx.part_world_position(gondolas[0])
        floor_center_z = _floor_center_z(0)
        if gondola_pivot_position is None or floor_center_z is None:
            ctx.fail(
                "quarter_turn_gondola_00_pose_available",
                "gondola pivot position or floor AABB was unavailable",
            )
        else:
            drop = gondola_pivot_position[2] - floor_center_z
            ctx.check(
                "quarter_turn_gondola_00_hangs_below_pivot",
                0.80 <= drop <= 1.20,
                details=f"pivot-to-floor vertical drop was {drop:.3f} m",
            )

    with ctx.pose(_upright_pose(pi)):
        ctx.fail_if_parts_overlap_in_current_pose(name="half_turn_no_overlap")
        ctx.fail_if_isolated_parts(name="half_turn_no_floating")
        ctx.expect_gap(
            gondolas[2],
            base_frame,
            axis="z",
            min_gap=0.80,
            name="half_turn_bottom_gondola_ground_clearance",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
