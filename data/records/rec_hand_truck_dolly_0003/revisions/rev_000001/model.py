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


def _tube_origin(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> tuple[float, Origin]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError("tube endpoints must be distinct")
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    midpoint = (
        (start[0] + end[0]) * 0.5,
        (start[1] + end[1]) * 0.5,
        (start[2] + end[2]) * 0.5,
    )
    return length, Origin(xyz=midpoint, rpy=(0.0, pitch, yaw))


def _add_tube(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str | None = None,
) -> None:
    length, origin = _tube_origin(start, end)
    if name is None:
        part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material)
    else:
        part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="convertible_hand_truck", assets=ASSETS)

    frame_steel = model.material("frame_steel", rgba=(0.29, 0.31, 0.34, 1.0))
    plate_steel = model.material("plate_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.07, 0.07, 0.08, 1.0))
    grip_black = model.material("grip_black", rgba=(0.10, 0.10, 0.11, 1.0))
    hub_metal = model.material("hub_metal", rgba=(0.63, 0.65, 0.68, 1.0))

    tube_r = 0.016
    rail_bottom_left = (-0.15, -0.03, 0.18)
    rail_bottom_right = (0.15, -0.03, 0.18)
    rail_top_left = (-0.13, -0.08, 1.08)
    rail_top_right = (0.13, -0.08, 1.08)
    rail_mid_left = (-0.141, -0.058, 0.70)
    rail_mid_right = (0.141, -0.058, 0.70)
    rail_low_left = (-0.146, -0.047, 0.43)
    rail_low_right = (0.146, -0.047, 0.43)

    axle_y = -0.055
    axle_z = 0.135
    wheel_center_x = 0.211
    wheel_radius = 0.135
    wheel_width = 0.046
    wheel_inner_x = wheel_center_x - wheel_width * 0.5
    collar_length = 0.004
    collar_center_x = wheel_inner_x - collar_length * 0.5

    toe_width = 0.34
    toe_depth = 0.23
    toe_thickness = 0.005
    toe_center = (0.0, 0.095, 0.0125)
    toe_front_y = toe_center[1] + toe_depth * 0.5

    extension_width = 0.31
    extension_depth = 0.17
    extension_thickness = 0.005

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.48, 0.36, 1.12)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.02, 0.56)),
    )

    _add_tube(frame, rail_bottom_left, rail_top_left, radius=tube_r, material=frame_steel, name="left_rail")
    _add_tube(frame, rail_bottom_right, rail_top_right, radius=tube_r, material=frame_steel, name="right_rail")
    _add_tube(frame, rail_top_left, rail_top_right, radius=tube_r, material=frame_steel, name="handle_crossbar")
    _add_tube(frame, rail_mid_left, rail_mid_right, radius=tube_r * 0.92, material=frame_steel, name="mid_brace")
    _add_tube(frame, rail_low_left, rail_low_right, radius=tube_r * 0.9, material=frame_steel, name="lower_brace")
    _add_tube(
        frame,
        rail_bottom_left,
        rail_bottom_right,
        radius=tube_r * 0.95,
        material=frame_steel,
        name="base_crossbar",
    )

    frame.visual(
        Box((0.028, 0.038, 0.068)),
        origin=Origin(xyz=(0.170, -0.045, 0.157)),
        material=frame_steel,
        name="left_axle_bracket",
    )
    frame.visual(
        Box((0.028, 0.038, 0.068)),
        origin=Origin(xyz=(-0.170, -0.045, 0.157)),
        material=frame_steel,
        name="right_axle_bracket",
    )
    _add_tube(frame, (0.0, -0.03, 0.18), (0.0, axle_y, axle_z + 0.010), radius=tube_r * 0.7, material=frame_steel)

    left_toe_anchor = (-0.11, -0.008, toe_center[2] + 0.003)
    right_toe_anchor = (0.11, -0.008, toe_center[2] + 0.003)
    toe_mid_anchor = (0.0, -0.008, toe_center[2] + 0.004)
    _add_tube(frame, rail_bottom_left, left_toe_anchor, radius=tube_r * 0.82, material=frame_steel, name="left_toe_support")
    _add_tube(frame, rail_bottom_right, right_toe_anchor, radius=tube_r * 0.82, material=frame_steel, name="right_toe_support")
    _add_tube(frame, (0.0, -0.03, 0.18), toe_mid_anchor, radius=tube_r * 0.75, material=frame_steel, name="center_toe_support")
    _add_tube(frame, left_toe_anchor, right_toe_anchor, radius=tube_r * 0.78, material=frame_steel, name="toe_back_crossbar")

    frame.visual(
        Box((toe_width, toe_depth, toe_thickness)),
        origin=Origin(xyz=toe_center),
        material=plate_steel,
        name="main_toe_plate",
    )
    frame.visual(
        Cylinder(radius=0.012, length=(wheel_inner_x * 2.0)),
        origin=Origin(xyz=(0.0, axle_y, axle_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_metal,
        name="axle_shaft",
    )
    frame.visual(
        Cylinder(radius=0.050, length=collar_length),
        origin=Origin(xyz=(collar_center_x, axle_y, axle_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_metal,
        name="left_axle_collar",
    )
    frame.visual(
        Cylinder(radius=0.050, length=collar_length),
        origin=Origin(xyz=(-collar_center_x, axle_y, axle_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_metal,
        name="right_axle_collar",
    )
    frame.visual(
        Cylinder(radius=0.020, length=0.080),
        origin=Origin(xyz=(-0.085, -0.08, 1.08), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grip_black,
        name="left_handle_grip",
    )
    frame.visual(
        Cylinder(radius=0.020, length=0.080),
        origin=Origin(xyz=(0.085, -0.08, 1.08), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grip_black,
        name="right_handle_grip",
    )

    left_wheel = model.part("left_wheel")
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=wheel_width),
        mass=1.8,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    left_wheel.visual(
        Cylinder(radius=wheel_radius, length=wheel_width),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_rubber,
        name="tire",
    )
    left_wheel.visual(
        Cylinder(radius=0.095, length=0.030),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_metal,
        name="rim",
    )
    left_wheel.visual(
        Cylinder(radius=0.052, length=wheel_width),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_metal,
        name="hub",
    )

    right_wheel = model.part("right_wheel")
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=wheel_width),
        mass=1.8,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    right_wheel.visual(
        Cylinder(radius=wheel_radius, length=wheel_width),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_rubber,
        name="tire",
    )
    right_wheel.visual(
        Cylinder(radius=0.095, length=0.030),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_metal,
        name="rim",
    )
    right_wheel.visual(
        Cylinder(radius=0.052, length=wheel_width),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_metal,
        name="hub",
    )

    extension = model.part("extension_plate")
    extension.inertial = Inertial.from_geometry(
        Box((extension_width, extension_depth, extension_thickness)),
        mass=1.2,
        origin=Origin(xyz=(0.0, -extension_depth * 0.5, -extension_thickness * 0.5)),
    )
    extension.visual(
        Box((extension_width, extension_depth, extension_thickness)),
        origin=Origin(xyz=(0.0, -extension_depth * 0.5, -extension_thickness * 0.5)),
        material=plate_steel,
        name="extension_plate",
    )

    model.articulation(
        "left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=left_wheel,
        origin=Origin(xyz=(wheel_center_x, axle_y, axle_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=25.0),
    )
    model.articulation(
        "right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=right_wheel,
        origin=Origin(xyz=(-wheel_center_x, axle_y, axle_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=25.0),
    )
    model.articulation(
        "extension_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=extension,
        origin=Origin(xyz=(0.0, toe_front_y, toe_center[2] - toe_thickness * 0.5)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.5, lower=0.0, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    frame = object_model.get_part("frame")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    extension = object_model.get_part("extension_plate")

    left_wheel_spin = object_model.get_articulation("left_wheel_spin")
    right_wheel_spin = object_model.get_articulation("right_wheel_spin")
    extension_hinge = object_model.get_articulation("extension_hinge")

    main_toe_plate = frame.get_visual("main_toe_plate")
    left_axle_collar = frame.get_visual("left_axle_collar")
    right_axle_collar = frame.get_visual("right_axle_collar")
    left_hub = left_wheel.get_visual("hub")
    right_hub = right_wheel.get_visual("hub")
    extension_plate = extension.get_visual("extension_plate")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
        "wheel_joints_are_continuous_x_axis",
        left_wheel_spin.articulation_type == ArticulationType.CONTINUOUS
        and right_wheel_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(left_wheel_spin.axis) == (1.0, 0.0, 0.0)
        and tuple(right_wheel_spin.axis) == (1.0, 0.0, 0.0),
        f"wheel axes: left={left_wheel_spin.axis}, right={right_wheel_spin.axis}",
    )
    ctx.check(
        "extension_hinge_range",
        extension_hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(extension_hinge.axis) == (1.0, 0.0, 0.0)
        and extension_hinge.motion_limits is not None
        and abs(extension_hinge.motion_limits.lower - 0.0) < 1e-9
        and abs(extension_hinge.motion_limits.upper - math.pi) < 1e-9,
        f"extension hinge limits: {extension_hinge.motion_limits}",
    )

    ctx.expect_contact(left_wheel, frame, elem_a=left_hub, elem_b=left_axle_collar)
    ctx.expect_contact(right_wheel, frame, elem_a=right_hub, elem_b=right_axle_collar)
    ctx.expect_origin_distance(left_wheel, right_wheel, axes="x", min_dist=0.40, max_dist=0.43)

    frame_aabb = ctx.part_world_aabb(frame)
    assert frame_aabb is not None
    frame_height = frame_aabb[1][2] - frame_aabb[0][2]
    ctx.check(
        "frame_height_realistic",
        1.05 <= frame_height <= 1.12,
        f"frame height {frame_height:.3f} m not in expected range",
    )

    toe_aabb = ctx.part_element_world_aabb(frame, elem="main_toe_plate")
    assert toe_aabb is not None
    toe_depth = toe_aabb[1][1] - toe_aabb[0][1]
    ctx.check(
        "main_toe_depth_realistic",
        0.22 <= toe_depth <= 0.24,
        f"main toe depth {toe_depth:.3f} m not in expected range",
    )

    with ctx.pose({extension_hinge: 0.0}):
        ctx.expect_contact(extension, frame, elem_a=extension_plate, elem_b=main_toe_plate)
        ctx.expect_within(extension, frame, axes="xy", inner_elem=extension_plate, outer_elem=main_toe_plate)
        ctx.expect_overlap(extension, frame, axes="xy", min_overlap=0.15, elem_a=extension_plate, elem_b=main_toe_plate)

    with ctx.pose({extension_hinge: math.pi}):
        ctx.expect_gap(
            extension,
            frame,
            axis="y",
            positive_elem=extension_plate,
            negative_elem=main_toe_plate,
            max_gap=0.001,
            max_penetration=0.0,
        )
        ctx.expect_overlap(extension, frame, axes="xz", min_overlap=0.004, elem_a=extension_plate, elem_b=main_toe_plate)

        deployed_main_aabb = ctx.part_element_world_aabb(frame, elem="main_toe_plate")
        deployed_ext_aabb = ctx.part_element_world_aabb(extension, elem="extension_plate")
        assert deployed_main_aabb is not None
        assert deployed_ext_aabb is not None
        combined_depth = max(deployed_main_aabb[1][1], deployed_ext_aabb[1][1]) - min(
            deployed_main_aabb[0][1],
            deployed_ext_aabb[0][1],
        )
        ctx.check(
            "extension_increases_toe_depth",
            combined_depth >= 0.39,
            f"combined deployed toe depth {combined_depth:.3f} m is too short",
        )

    with ctx.pose({extension_hinge: math.pi * 0.5}):
        extension_mid_aabb = ctx.part_world_aabb(extension)
        assert extension_mid_aabb is not None
        extension_vertical_span = extension_mid_aabb[1][2] - extension_mid_aabb[0][2]
        ctx.check(
            "extension_swings_vertical_midstroke",
            extension_vertical_span >= 0.16,
            f"mid-stroke extension z span {extension_vertical_span:.3f} m is too small",
        )

    with ctx.pose({left_wheel_spin: math.pi / 3.0, right_wheel_spin: -math.pi / 2.0}):
        ctx.expect_contact(left_wheel, frame, elem_a=left_hub, elem_b=left_axle_collar)
        ctx.expect_contact(right_wheel, frame, elem_a=right_hub, elem_b=right_axle_collar)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
