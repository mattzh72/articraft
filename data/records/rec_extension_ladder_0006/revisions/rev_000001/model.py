from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
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


def _add_member(part, a, b, *, radius: float, material, name: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _add_cylinder_x(part, *, radius: float, length: float, xyz, material, name: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _add_cylinder_y(part, *, radius: float, length: float, xyz, material, name: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="roof_ladder", assets=ASSETS)

    ladder_aluminum = model.material("ladder_aluminum", rgba=(0.80, 0.81, 0.82, 1.0))
    galvanized_steel = model.material("galvanized_steel", rgba=(0.43, 0.45, 0.47, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.10, 0.10, 0.10, 1.0))
    wheel_steel = model.material("wheel_steel", rgba=(0.60, 0.61, 0.63, 1.0))

    rail_half_spacing = 0.19
    rail_outer_x = rail_half_spacing + 0.0225
    rail_width = 0.045
    rail_thickness = 0.018
    rail_center_y = 0.030
    ladder_bottom_z = 0.08
    ladder_top_z = 2.00
    ladder_length = ladder_top_z - ladder_bottom_z

    wheel_center_x = 0.256
    wheel_center_y = 0.057
    wheel_center_z = 0.125
    wheel_radius = 0.055
    wheel_tire_width = 0.022
    wheel_hub_width = 0.033
    axle_collar_x = 0.226
    axle_collar_length = 0.027

    ladder = model.part("ladder_section")
    ladder.visual(
        Box((rail_width, rail_thickness, ladder_length)),
        origin=Origin(xyz=(-rail_half_spacing, rail_center_y, ladder_bottom_z + ladder_length * 0.5)),
        material=ladder_aluminum,
        name="left_rail",
    )
    ladder.visual(
        Box((rail_width, rail_thickness, ladder_length)),
        origin=Origin(xyz=(rail_half_spacing, rail_center_y, ladder_bottom_z + ladder_length * 0.5)),
        material=ladder_aluminum,
        name="right_rail",
    )
    _add_cylinder_x(
        ladder,
        radius=0.013,
        length=0.372,
        xyz=(0.0, rail_center_y, 0.105),
        material=ladder_aluminum,
        name="base_crossbar",
    )
    _add_cylinder_x(
        ladder,
        radius=0.011,
        length=0.360,
        xyz=(0.0, rail_center_y, 1.930),
        material=ladder_aluminum,
        name="top_crossbar",
    )
    for index, rung_z in enumerate(
        (0.24, 0.42, 0.60, 0.78, 0.96, 1.14, 1.32, 1.50, 1.68, 1.86),
        start=1,
    ):
        _add_cylinder_x(
            ladder,
            radius=0.012,
            length=0.350,
            xyz=(0.0, rail_center_y, rung_z),
            material=ladder_aluminum,
            name=f"rung_{index:02d}",
        )

    ladder.visual(
        Box((0.050, 0.026, 0.060)),
        origin=Origin(xyz=(-rail_half_spacing, rail_center_y + 0.022, 1.970)),
        material=galvanized_steel,
        name="left_frame_pad",
    )
    ladder.visual(
        Box((0.050, 0.026, 0.060)),
        origin=Origin(xyz=(rail_half_spacing, rail_center_y + 0.022, 1.970)),
        material=galvanized_steel,
        name="right_frame_pad",
    )
    ladder.visual(
        Box((0.050, 0.026, 0.100)),
        origin=Origin(xyz=(-rail_half_spacing, rail_center_y + 0.022, 1.830)),
        material=galvanized_steel,
        name="left_hook_pad",
    )
    ladder.visual(
        Box((0.050, 0.026, 0.100)),
        origin=Origin(xyz=(rail_half_spacing, rail_center_y + 0.022, 1.830)),
        material=galvanized_steel,
        name="right_hook_pad",
    )

    _add_cylinder_x(
        ladder,
        radius=0.015,
        length=axle_collar_length,
        xyz=(-axle_collar_x, wheel_center_y, wheel_center_z),
        material=galvanized_steel,
        name="left_axle_collar",
    )
    _add_cylinder_x(
        ladder,
        radius=0.015,
        length=axle_collar_length,
        xyz=(axle_collar_x, wheel_center_y, wheel_center_z),
        material=galvanized_steel,
        name="right_axle_collar",
    )
    for name, a, b in (
        (
            "left_upper_brace",
            (-rail_outer_x, rail_center_y, 0.165),
            (-axle_collar_x, wheel_center_y, 0.138),
        ),
        (
            "left_lower_brace",
            (-rail_outer_x, rail_center_y, 0.095),
            (-axle_collar_x, wheel_center_y, 0.112),
        ),
        (
            "right_upper_brace",
            (rail_outer_x, rail_center_y, 0.165),
            (axle_collar_x, wheel_center_y, 0.138),
        ),
        (
            "right_lower_brace",
            (rail_outer_x, rail_center_y, 0.095),
            (axle_collar_x, wheel_center_y, 0.112),
        ),
    ):
        _add_member(
            ladder,
            a,
            b,
            radius=0.008,
            material=galvanized_steel,
            name=name,
        )
    ladder.inertial = Inertial.from_geometry(
        Box((0.60, 0.16, 2.05)),
        mass=13.5,
        origin=Origin(xyz=(0.0, 0.060, 1.025)),
    )

    frame = model.part("railing_frame")
    frame.visual(
        Box((0.050, 0.026, 0.060)),
        origin=Origin(xyz=(-rail_half_spacing, 0.013, 0.030)),
        material=galvanized_steel,
        name="left_base_shoe",
    )
    frame.visual(
        Box((0.050, 0.026, 0.060)),
        origin=Origin(xyz=(rail_half_spacing, 0.013, 0.030)),
        material=galvanized_steel,
        name="right_base_shoe",
    )
    _add_cylinder_y(
        frame,
        radius=0.013,
        length=0.360,
        xyz=(-rail_half_spacing, 0.206, 0.030),
        material=ladder_aluminum,
        name="left_post",
    )
    _add_cylinder_y(
        frame,
        radius=0.013,
        length=0.360,
        xyz=(rail_half_spacing, 0.206, 0.030),
        material=ladder_aluminum,
        name="right_post",
    )
    _add_cylinder_x(
        frame,
        radius=0.011,
        length=0.380,
        xyz=(0.0, 0.195, 0.030),
        material=ladder_aluminum,
        name="mid_crossbar",
    )
    _add_cylinder_x(
        frame,
        radius=0.011,
        length=0.420,
        xyz=(0.0, 0.386, 0.030),
        material=ladder_aluminum,
        name="top_crossbar",
    )
    _add_member(
        frame,
        (-rail_half_spacing, 0.026, 0.060),
        (-0.100, 0.195, 0.030),
        radius=0.008,
        material=galvanized_steel,
        name="left_brace",
    )
    _add_member(
        frame,
        (rail_half_spacing, 0.026, 0.060),
        (0.100, 0.195, 0.030),
        radius=0.008,
        material=galvanized_steel,
        name="right_brace",
    )
    frame.inertial = Inertial.from_geometry(
        Box((0.46, 0.42, 0.12)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.210, 0.030)),
    )

    hook = model.part("hook_bracket")
    hook.visual(
        Box((0.050, 0.026, 0.060)),
        origin=Origin(xyz=(-rail_half_spacing, 0.013, 0.030)),
        material=galvanized_steel,
        name="left_hook_shoe",
    )
    hook.visual(
        Box((0.050, 0.026, 0.060)),
        origin=Origin(xyz=(rail_half_spacing, 0.013, 0.030)),
        material=galvanized_steel,
        name="right_hook_shoe",
    )
    left_hook_points = (
        (-rail_half_spacing, 0.026, 0.030),
        (-0.205, 0.090, 0.045),
        (-0.245, 0.190, 0.190),
        (-0.255, 0.300, 0.420),
        (-0.185, 0.245, 0.620),
        (-0.075, 0.105, 0.740),
    )
    right_hook_points = tuple((-x, y, z) for x, y, z in left_hook_points)
    for side_name, points in (("left", left_hook_points), ("right", right_hook_points)):
        for segment_index, (start, end) in enumerate(zip(points[:-1], points[1:]), start=1):
            _add_member(
                hook,
                start,
                end,
                radius=0.010,
                material=galvanized_steel,
                name=f"{side_name}_hook_segment_{segment_index}",
            )
    _add_cylinder_x(
        hook,
        radius=0.010,
        length=0.510,
        xyz=(0.0, 0.300, 0.420),
        material=galvanized_steel,
        name="upper_tie_bar",
    )
    hook.inertial = Inertial.from_geometry(
        Box((0.46, 0.32, 0.68)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.150, 0.300)),
    )

    def _build_wheel(part_name: str) -> None:
        wheel = model.part(part_name)
        _add_cylinder_x(
            wheel,
            radius=wheel_radius,
            length=wheel_tire_width,
            xyz=(0.0, 0.0, 0.0),
            material=dark_rubber,
            name="tire",
        )
        _add_cylinder_x(
            wheel,
            radius=0.037,
            length=wheel_hub_width,
            xyz=(0.0, 0.0, 0.0),
            material=wheel_steel,
            name="rim",
        )
        _add_cylinder_x(
            wheel,
            radius=0.018,
            length=wheel_hub_width,
            xyz=(0.0, 0.0, 0.0),
            material=galvanized_steel,
            name="hub",
        )
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=wheel_radius, length=wheel_tire_width),
            mass=0.85,
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        )

    _build_wheel("left_wheel")
    _build_wheel("right_wheel")

    model.articulation(
        "ladder_to_frame",
        ArticulationType.FIXED,
        parent=ladder,
        child=frame,
        origin=Origin(xyz=(0.0, rail_center_y + rail_thickness * 0.5, 2.000)),
    )
    model.articulation(
        "ladder_to_hook_bracket",
        ArticulationType.FIXED,
        parent=ladder,
        child=hook,
        origin=Origin(xyz=(0.0, rail_center_y + rail_thickness * 0.5, 1.880)),
    )
    model.articulation(
        "left_wheel_axle",
        ArticulationType.REVOLUTE,
        parent=ladder,
        child="left_wheel",
        origin=Origin(xyz=(-wheel_center_x, wheel_center_y, wheel_center_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=18.0,
            lower=-12.0 * math.pi,
            upper=12.0 * math.pi,
        ),
    )
    model.articulation(
        "right_wheel_axle",
        ArticulationType.REVOLUTE,
        parent=ladder,
        child="right_wheel",
        origin=Origin(xyz=(wheel_center_x, wheel_center_y, wheel_center_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=18.0,
            lower=-12.0 * math.pi,
            upper=12.0 * math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ladder = object_model.get_part("ladder_section")
    frame = object_model.get_part("railing_frame")
    hook = object_model.get_part("hook_bracket")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    left_axle = object_model.get_articulation("left_wheel_axle")
    right_axle = object_model.get_articulation("right_wheel_axle")

    left_rail = ladder.get_visual("left_rail")
    right_rail = ladder.get_visual("right_rail")
    left_frame_pad = ladder.get_visual("left_frame_pad")
    right_frame_pad = ladder.get_visual("right_frame_pad")
    left_hook_pad = ladder.get_visual("left_hook_pad")
    right_hook_pad = ladder.get_visual("right_hook_pad")
    left_collar = ladder.get_visual("left_axle_collar")
    right_collar = ladder.get_visual("right_axle_collar")

    left_base_shoe = frame.get_visual("left_base_shoe")
    right_base_shoe = frame.get_visual("right_base_shoe")
    left_hook_shoe = hook.get_visual("left_hook_shoe")
    right_hook_shoe = hook.get_visual("right_hook_shoe")
    left_hub = left_wheel.get_visual("hub")
    right_hub = right_wheel.get_visual("hub")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    ctx.expect_contact(frame, ladder, elem_a=left_base_shoe, elem_b=left_frame_pad)
    ctx.expect_contact(frame, ladder, elem_a=right_base_shoe, elem_b=right_frame_pad)
    ctx.expect_contact(hook, ladder, elem_a=left_hook_shoe, elem_b=left_hook_pad)
    ctx.expect_contact(hook, ladder, elem_a=right_hook_shoe, elem_b=right_hook_pad)
    ctx.expect_contact(left_wheel, ladder, elem_a=left_hub, elem_b=left_collar)
    ctx.expect_contact(right_wheel, ladder, elem_a=right_hub, elem_b=right_collar)
    ctx.expect_origin_distance(frame, ladder, axes="x", max_dist=0.001)
    ctx.expect_origin_distance(hook, ladder, axes="x", max_dist=0.001)
    ctx.expect_origin_distance(left_wheel, right_wheel, axes="yz", max_dist=0.001)
    ctx.expect_gap(right_wheel, left_wheel, axis="x", min_gap=0.44)
    ctx.expect_overlap(frame, ladder, axes="x", min_overlap=0.30)
    ctx.expect_overlap(hook, ladder, axes="x", min_overlap=0.30)
    ctx.expect_overlap(frame, ladder, axes="xy", min_overlap=0.020, elem_a=left_base_shoe, elem_b=left_frame_pad)
    ctx.expect_overlap(hook, ladder, axes="xy", min_overlap=0.020, elem_a=right_hook_shoe, elem_b=right_hook_pad)

    ladder_aabb = ctx.part_world_aabb(ladder)
    frame_aabb = ctx.part_world_aabb(frame)
    hook_aabb = ctx.part_world_aabb(hook)
    left_wheel_aabb = ctx.part_world_aabb(left_wheel)
    right_wheel_aabb = ctx.part_world_aabb(right_wheel)
    left_rail_aabb = ctx.part_element_world_aabb(ladder, elem=left_rail)
    right_rail_aabb = ctx.part_element_world_aabb(ladder, elem=right_rail)
    assert ladder_aabb is not None
    assert frame_aabb is not None
    assert hook_aabb is not None
    assert left_wheel_aabb is not None
    assert right_wheel_aabb is not None
    assert left_rail_aabb is not None
    assert right_rail_aabb is not None

    ctx.check(
        "frame_projects_above_ladder",
        frame_aabb[1][1] > ladder_aabb[1][1] + 0.30,
        details=(
            f"frame max y {frame_aabb[1][1]:.3f} should rise well above ladder max y "
            f"{ladder_aabb[1][1]:.3f}"
        ),
    )
    ctx.check(
        "hook_reaches_over_ridge",
        hook_aabb[1][2] > ladder_aabb[1][2] + 0.55 and hook_aabb[1][1] > ladder_aabb[1][1] + 0.18,
        details=(
            f"hook aabb max={hook_aabb[1]} should extend beyond ladder top max={ladder_aabb[1]}"
        ),
    )
    ctx.check(
        "wheels_mounted_at_base",
        left_wheel_aabb[1][2] < 0.19
        and right_wheel_aabb[1][2] < 0.19
        and left_wheel_aabb[1][0] < left_rail_aabb[0][0] - 0.015
        and right_wheel_aabb[0][0] > right_rail_aabb[1][0] + 0.015,
        details=(
            f"wheel aabbs left={left_wheel_aabb}, right={right_wheel_aabb}, "
            f"rail aabbs left={left_rail_aabb}, right={right_rail_aabb}"
        ),
    )
    ctx.check(
        "wheel_axes_are_revolute_x",
        tuple(left_axle.axis) == (1.0, 0.0, 0.0) and tuple(right_axle.axis) == (1.0, 0.0, 0.0),
        details=f"left axis={left_axle.axis}, right axis={right_axle.axis}",
    )

    with ctx.pose({left_axle: 7.1, right_axle: -6.4}):
        ctx.expect_contact(left_wheel, ladder, elem_a=left_hub, elem_b=left_collar)
        ctx.expect_contact(right_wheel, ladder, elem_a=right_hub, elem_b=right_collar)
        ctx.expect_gap(right_wheel, left_wheel, axis="x", min_gap=0.44)
        ctx.fail_if_parts_overlap_in_current_pose(name="wheel_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="wheel_pose_no_floating")

    left_limits = left_axle.motion_limits
    right_limits = right_axle.motion_limits
    if (
        left_limits is not None
        and right_limits is not None
        and left_limits.lower is not None
        and right_limits.upper is not None
    ):
        with ctx.pose({left_axle: left_limits.lower, right_axle: right_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="wheel_limit_pose_no_overlap")
            ctx.fail_if_isolated_parts(name="wheel_limit_pose_no_floating")
            ctx.expect_contact(left_wheel, ladder, elem_a=left_hub, elem_b=left_collar)
            ctx.expect_contact(right_wheel, ladder, elem_a=right_hub, elem_b=right_collar)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
