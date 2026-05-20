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
)


WHEEL_CENTER_Z = 1.42
WHEEL_WIDTH = 0.52
AXLE_LENGTH = 1.20
SUPPORT_X = 0.82
RUNNER_LENGTH = 1.50
BUCKET_COUNT = 12


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
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


def _rotate_x(
    point: tuple[float, float, float], angle: float
) -> tuple[float, float, float]:
    x, y, z = point
    c = cos(angle)
    s = sin(angle)
    return (x, y * c - z * s, y * s + z * c)


def _wheel_ring_point(radius: float, angle: float) -> tuple[float, float, float]:
    return (0.0, radius * sin(angle), radius * cos(angle))


def _build_frame(model: ArticulatedObject, timber, iron) -> None:
    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((1.95, RUNNER_LENGTH, 2.75)),
        mass=420.0,
        origin=Origin(xyz=(0.0, 0.0, 1.375)),
    )

    runner_size = (0.16, RUNNER_LENGTH, 0.14)
    for side_name, side_x in (("left", SUPPORT_X), ("right", -SUPPORT_X)):
        frame.visual(
            Box(runner_size),
            origin=Origin(xyz=(side_x, 0.0, 0.07)),
            material=timber,
            name=f"{side_name}_runner",
        )
        for y_pos, post_name in ((-0.28, "front"), (0.28, "rear")):
            frame.visual(
                Box((0.16, 0.12, 1.48)),
                origin=Origin(xyz=(side_x, y_pos, 0.88)),
                material=timber,
                name=f"{side_name}_{post_name}_post",
            )
        frame.visual(
            Box((0.16, 0.72, 0.10)),
            origin=Origin(xyz=(side_x, 0.0, 0.36)),
            material=timber,
            name=f"{side_name}_side_rail",
        )
        frame.visual(
            Box((0.18, 0.72, 0.12)),
            origin=Origin(xyz=(side_x, 0.0, 1.60)),
            material=timber,
            name=f"{side_name}_top_cap",
        )
        block_x = 0.69 if side_x > 0.0 else -0.69
        frame.visual(
            Box((0.18, 0.18, 0.20)),
            origin=Origin(xyz=(block_x, 0.0, WHEEL_CENTER_Z)),
            material=iron,
            name=f"{side_name}_bearing_block",
        )

        _add_member(
            frame,
            (side_x, -0.68, 0.14),
            (side_x, 0.00, 1.60),
            0.035,
            timber,
        )
        _add_member(
            frame,
            (side_x, 0.68, 0.14),
            (side_x, 0.00, 1.60),
            0.035,
            timber,
        )
        _add_member(
            frame,
            (side_x, -0.28, 0.34),
            (block_x, 0.00, WHEEL_CENTER_Z + 0.06),
            0.025,
            iron,
        )
        _add_member(
            frame,
            (side_x, 0.28, 0.34),
            (block_x, 0.00, WHEEL_CENTER_Z + 0.06),
            0.025,
            iron,
        )

    for y_pos, tie_name in ((-0.56, "rear"), (0.56, "front")):
        frame.visual(
            Box((1.80, 0.12, 0.12)),
            origin=Origin(xyz=(0.0, y_pos, 0.18)),
            material=timber,
            name=f"{tie_name}_tie",
        )

    frame.visual(
        Box((0.12, 0.12, 1.06)),
        origin=Origin(xyz=(0.72, -0.34, 2.13)),
        material=timber,
        name="left_chute_post",
    )
    frame.visual(
        Box((0.12, 0.12, 1.06)),
        origin=Origin(xyz=(-0.72, -0.34, 2.13)),
        material=timber,
        name="right_chute_post",
    )
    frame.visual(
        Box((1.56, 0.12, 0.12)),
        origin=Origin(xyz=(0.0, -0.34, 2.63)),
        material=timber,
        name="chute_support_beam",
    )


def _build_wheel(model: ArticulatedObject, timber, iron, dark_iron) -> None:
    wheel = model.part("wheel")
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=1.22, length=WHEEL_WIDTH),
        mass=260.0,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    rim_mesh = _save_mesh("waterwheel_rim", TorusGeometry(radius=1.10, tube=0.05))
    tire_band_mesh = _save_mesh("waterwheel_tire_band", TorusGeometry(radius=1.12, tube=0.014))
    wheel.visual(
        Cylinder(radius=0.055, length=AXLE_LENGTH),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_iron,
        name="axle",
    )
    wheel.visual(
        Cylinder(radius=0.15, length=0.56),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_iron,
        name="hub_barrel",
    )
    wheel.visual(
        Cylinder(radius=0.22, length=0.04),
        origin=Origin(xyz=(0.12, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=iron,
        name="left_hub_plate",
    )
    wheel.visual(
        Cylinder(radius=0.22, length=0.04),
        origin=Origin(xyz=(-0.12, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=iron,
        name="right_hub_plate",
    )
    wheel.visual(
        rim_mesh,
        origin=Origin(xyz=(0.23, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=timber,
        name="left_rim",
    )
    wheel.visual(
        rim_mesh,
        origin=Origin(xyz=(-0.23, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=timber,
        name="right_rim",
    )
    wheel.visual(
        tire_band_mesh,
        origin=Origin(xyz=(0.285, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=iron,
        name="left_tire_band",
    )
    wheel.visual(
        tire_band_mesh,
        origin=Origin(xyz=(-0.285, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=iron,
        name="right_tire_band",
    )

    spoke_radius = 0.032
    for index in range(BUCKET_COUNT):
        angle = (2.0 * pi * index) / BUCKET_COUNT
        inner_point = _wheel_ring_point(0.17, angle)
        outer_point = _wheel_ring_point(1.11, angle)
        _add_member(
            wheel,
            (0.16, inner_point[1], inner_point[2]),
            (0.16, outer_point[1], outer_point[2]),
            spoke_radius,
            timber,
            name=f"left_spoke_{index:02d}",
        )
        _add_member(
            wheel,
            (-0.16, inner_point[1], inner_point[2]),
            (-0.16, outer_point[1], outer_point[2]),
            spoke_radius,
            timber,
            name=f"right_spoke_{index:02d}",
        )

        roll = -angle
        bucket_base = _wheel_ring_point(1.00, angle)
        floor_size = (0.46, 0.14, 0.028)
        wall_size = (0.46, 0.028, 0.12)
        wall_offset = _rotate_x((0.0, -0.056, 0.046), roll)
        lip_offset = _rotate_x((0.0, 0.058, 0.038), roll)
        cheek_offset_left = _rotate_x((0.205, -0.004, 0.047), roll)
        cheek_offset_right = _rotate_x((-0.205, -0.004, 0.047), roll)

        wheel.visual(
            Box(floor_size),
            origin=Origin(xyz=bucket_base, rpy=(roll, 0.0, 0.0)),
            material=timber,
            name=f"bucket_floor_{index:02d}",
        )
        wheel.visual(
            Box(wall_size),
            origin=Origin(
                xyz=(
                    bucket_base[0] + wall_offset[0],
                    bucket_base[1] + wall_offset[1],
                    bucket_base[2] + wall_offset[2],
                ),
                rpy=(roll, 0.0, 0.0),
            ),
            material=timber,
            name=f"bucket_back_{index:02d}",
        )
        wheel.visual(
            Box((0.46, 0.024, 0.08)),
            origin=Origin(
                xyz=(
                    bucket_base[0] + lip_offset[0],
                    bucket_base[1] + lip_offset[1],
                    bucket_base[2] + lip_offset[2],
                ),
                rpy=(roll, 0.0, 0.0),
            ),
            material=timber,
            name=f"bucket_lip_{index:02d}",
        )
        wheel.visual(
            Box((0.024, 0.13, 0.11)),
            origin=Origin(
                xyz=(
                    bucket_base[0] + cheek_offset_left[0],
                    bucket_base[1] + cheek_offset_left[1],
                    bucket_base[2] + cheek_offset_left[2],
                ),
                rpy=(roll, 0.0, 0.0),
            ),
            material=timber,
            name=f"bucket_left_cheek_{index:02d}",
        )
        wheel.visual(
            Box((0.024, 0.13, 0.11)),
            origin=Origin(
                xyz=(
                    bucket_base[0] + cheek_offset_right[0],
                    bucket_base[1] + cheek_offset_right[1],
                    bucket_base[2] + cheek_offset_right[2],
                ),
                rpy=(roll, 0.0, 0.0),
            ),
            material=timber,
            name=f"bucket_right_cheek_{index:02d}",
        )


def _build_chute(model: ArticulatedObject, timber, iron) -> None:
    chute = model.part("chute")
    chute.inertial = Inertial.from_geometry(
        Box((0.46, 0.70, 0.24)),
        mass=55.0,
        origin=Origin(xyz=(0.0, 0.22, 0.12)),
    )

    slope = -0.28
    chute.visual(
        Box((0.42, 0.12, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=iron,
        name="mount_pad",
    )
    chute.visual(
        Box((0.40, 0.60, 0.05)),
        origin=Origin(xyz=(0.0, 0.26, 0.10), rpy=(slope, 0.0, 0.0)),
        material=timber,
        name="chute_floor",
    )
    chute.visual(
        Box((0.04, 0.60, 0.16)),
        origin=Origin(xyz=(0.19, 0.26, 0.16), rpy=(slope, 0.0, 0.0)),
        material=timber,
        name="left_side_wall",
    )
    chute.visual(
        Box((0.04, 0.60, 0.16)),
        origin=Origin(xyz=(-0.19, 0.26, 0.16), rpy=(slope, 0.0, 0.0)),
        material=timber,
        name="right_side_wall",
    )
    chute.visual(
        Box((0.40, 0.04, 0.18)),
        origin=Origin(xyz=(0.0, -0.02, 0.14), rpy=(slope, 0.0, 0.0)),
        material=timber,
        name="back_plate",
    )
    chute.visual(
        Box((0.40, 0.03, 0.10)),
        origin=Origin(xyz=(0.0, 0.54, 0.12), rpy=(slope, 0.0, 0.0)),
        material=timber,
        name="outlet_lip",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="overshot_waterwheel")

    weathered_timber = model.material("weathered_timber", rgba=(0.49, 0.34, 0.20, 1.0))
    wet_timber = model.material("wet_timber", rgba=(0.39, 0.26, 0.15, 1.0))
    iron = model.material("iron", rgba=(0.42, 0.43, 0.45, 1.0))
    dark_iron = model.material("dark_iron", rgba=(0.22, 0.23, 0.24, 1.0))

    _build_frame(model, weathered_timber, iron)
    _build_wheel(model, wet_timber, iron, dark_iron)
    _build_chute(model, weathered_timber, iron)

    frame = model.get_part("frame")
    wheel = model.get_part("wheel")
    chute = model.get_part("chute")

    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, WHEEL_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1800.0, velocity=2.5),
    )
    model.articulation(
        "chute_mount",
        ArticulationType.FIXED,
        parent=frame,
        child=chute,
        origin=Origin(xyz=(0.0, -0.34, 2.69)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    wheel = object_model.get_part("wheel")
    chute = object_model.get_part("chute")
    spin = object_model.get_articulation("wheel_spin")
    axle = wheel.get_visual("axle")
    left_block = frame.get_visual("left_bearing_block")
    right_block = frame.get_visual("right_bearing_block")
    left_runner = frame.get_visual("left_runner")
    support_beam = frame.get_visual("chute_support_beam")
    mount_pad = chute.get_visual("mount_pad")
    chute_floor = chute.get_visual("chute_floor")

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
        "wheel_spin_is_continuous",
        spin.joint_type == ArticulationType.CONTINUOUS,
        f"expected continuous wheel spin, got {spin.joint_type}",
    )
    ctx.check(
        "wheel_spin_axis_horizontal_x",
        tuple(spin.axis) == (1.0, 0.0, 0.0),
        f"expected axle axis (1, 0, 0), got {spin.axis}",
    )
    limits = spin.motion_limits
    ctx.check(
        "wheel_spin_limits_unbounded",
        limits is not None and limits.lower is None and limits.upper is None,
        f"continuous wheel should have no lower/upper limits, got {limits}",
    )

    ctx.expect_contact(wheel, frame, elem_a=axle, elem_b=left_block, name="left_bearing_contacts_axle")
    ctx.expect_contact(wheel, frame, elem_a=axle, elem_b=right_block, name="right_bearing_contacts_axle")
    ctx.expect_contact(chute, frame, elem_a=mount_pad, elem_b=support_beam, name="chute_mount_contacts_support_beam")
    ctx.expect_gap(chute, wheel, axis="z", min_gap=0.035, max_gap=0.13, positive_elem=chute_floor, name="chute_clears_wheel")
    ctx.expect_overlap(chute, wheel, axes="x", min_overlap=0.35, elem_a=chute_floor, name="chute_spans_wheel_width")
    ctx.expect_gap(wheel, frame, axis="z", min_gap=0.08, max_gap=0.20, negative_elem=left_runner, name="wheel_clears_ground_runners")
    ctx.expect_overlap(frame, wheel, axes="y", min_overlap=1.20, elem_a=left_runner, name="frame_depth_brackets_wheel")

    wheel_aabb = ctx.part_world_aabb(wheel)
    if wheel_aabb is not None:
        wheel_height = wheel_aabb[1][2] - wheel_aabb[0][2]
        wheel_depth = wheel_aabb[1][1] - wheel_aabb[0][1]
        ctx.check(
            "wheel_realistic_diameter",
            2.25 <= wheel_height <= 2.55,
            f"wheel height {wheel_height:.3f} m should read as a large waterwheel",
        )
        ctx.check(
            "wheel_thickness_reasonable",
            0.45 <= wheel_depth <= 2.50,
            f"wheel depth projection {wheel_depth:.3f} m should remain substantial",
        )

    frame_aabb = ctx.part_world_aabb(frame)
    if frame_aabb is not None:
        frame_width = frame_aabb[1][0] - frame_aabb[0][0]
        frame_height = frame_aabb[1][2] - frame_aabb[0][2]
        ctx.check(
            "frame_wider_than_wheel_axle_span",
            1.75 <= frame_width <= 2.10,
            f"frame width {frame_width:.3f} m should fit the wheel and side supports",
        )
        ctx.check(
            "frame_has_tall_chute_support",
            2.65 <= frame_height <= 2.85,
            f"frame height {frame_height:.3f} m should include the chute mast above the wheel",
        )

    for pose_name, angle in (("rest", 0.0), ("quarter_turn", pi / 4.0), ("half_turn", pi / 2.0)):
        with ctx.pose({spin: angle}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{pose_name}_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{pose_name}_no_floating")
            ctx.expect_contact(
                wheel,
                frame,
                elem_a=axle,
                elem_b=left_block,
                name=f"{pose_name}_left_bearing_contact",
            )
            ctx.expect_contact(
                wheel,
                frame,
                elem_a=axle,
                elem_b=right_block,
                name=f"{pose_name}_right_bearing_contact",
            )
            ctx.expect_gap(
                chute,
                wheel,
                axis="z",
                min_gap=0.035,
                max_gap=0.13,
                positive_elem=chute_floor,
                name=f"{pose_name}_chute_clearance",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
