from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os

os.chdir("/")

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _x_axis_origin(xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0))


def _y_axis_origin(xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=(-math.pi / 2.0, 0.0, 0.0))


def _segment_origin(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> tuple[Origin, float]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.hypot(dx, dy), dz)
    center = (
        (start[0] + end[0]) * 0.5,
        (start[1] + end[1]) * 0.5,
        (start[2] + end[2]) * 0.5,
    )
    return Origin(xyz=center, rpy=(0.0, pitch, yaw)), length


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="refractor_telescope")

    painted_tube = model.material("painted_tube", rgba=(0.93, 0.93, 0.91, 1.0))
    mount_metal = model.material("mount_metal", rgba=(0.22, 0.24, 0.27, 1.0))
    dark_hardware = model.material("dark_hardware", rgba=(0.09, 0.10, 0.11, 1.0))
    wood = model.material("wood", rgba=(0.50, 0.33, 0.18, 1.0))
    brass = model.material("brass", rgba=(0.67, 0.55, 0.28, 1.0))
    leather = model.material("leather", rgba=(0.22, 0.15, 0.10, 1.0))
    glass = model.material("glass", rgba=(0.70, 0.83, 0.92, 0.45))

    tripod = model.part("tripod")
    tripod.visual(
        Cylinder(radius=0.070, length=0.105),
        origin=Origin(xyz=(0.0, 0.0, 0.985)),
        material=wood,
        name="hub_core",
    )
    tripod.visual(
        Cylinder(radius=0.078, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 1.048)),
        material=dark_hardware,
        name="tripod_plate",
    )
    tripod.visual(
        Cylinder(radius=0.018, length=0.440),
        origin=Origin(xyz=(0.0, 0.0, 0.730)),
        material=brass,
        name="center_rod",
    )
    tripod.visual(
        Cylinder(radius=0.135, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.560)),
        material=wood,
        name="accessory_tray",
    )

    leg_angles = (math.pi / 2.0, math.pi / 2.0 + 2.0 * math.pi / 3.0, math.pi / 2.0 + 4.0 * math.pi / 3.0)
    for index, angle in enumerate(leg_angles, start=1):
        top = (0.055 * math.cos(angle), 0.055 * math.sin(angle), 0.965)
        foot = (0.470 * math.cos(angle), 0.470 * math.sin(angle), 0.030)
        leg_origin, leg_length = _segment_origin(top, foot)
        tripod.visual(
            Box((0.038, 0.024, leg_length)),
            origin=leg_origin,
            material=wood,
            name=f"leg_{index}",
        )

        tray_anchor = (0.060 * math.cos(angle), 0.060 * math.sin(angle), 0.560)
        spreader_anchor = (0.240 * math.cos(angle), 0.240 * math.sin(angle), 0.500)
        spreader_origin, spreader_length = _segment_origin(tray_anchor, spreader_anchor)
        tripod.visual(
            Box((0.022, 0.016, spreader_length)),
            origin=spreader_origin,
            material=brass,
            name=f"spreader_{index}",
        )

    mount_head = model.part("mount_head")
    mount_head.visual(
        Cylinder(radius=0.074, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=mount_metal,
        name="az_base",
    )
    mount_head.visual(
        Cylinder(radius=0.030, length=0.154),
        origin=Origin(xyz=(0.0, 0.0, 0.103)),
        material=mount_metal,
        name="pier",
    )
    mount_head.visual(
        Box((0.090, 0.120, 0.030)),
        origin=Origin(xyz=(0.045, 0.0, 0.184)),
        material=mount_metal,
        name="head_crossbar",
    )
    mount_head.visual(
        Box((0.030, 0.012, 0.122)),
        origin=Origin(xyz=(0.045, 0.054, 0.245)),
        material=mount_metal,
        name="left_arm",
    )
    mount_head.visual(
        Box((0.030, 0.012, 0.122)),
        origin=Origin(xyz=(0.045, -0.054, 0.245)),
        material=mount_metal,
        name="right_arm",
    )

    optical_tube = model.part("optical_tube")
    optical_tube.visual(
        Box((0.060, 0.068, 0.100)),
        origin=Origin(xyz=(0.020, 0.0, 0.050)),
        material=mount_metal,
        name="support_block",
    )
    optical_tube.visual(
        Box((0.055, 0.016, 0.070)),
        origin=Origin(xyz=(0.020, 0.034, 0.035)),
        material=mount_metal,
        name="left_support",
    )
    optical_tube.visual(
        Box((0.055, 0.016, 0.070)),
        origin=Origin(xyz=(0.020, -0.034, 0.035)),
        material=mount_metal,
        name="right_support",
    )
    optical_tube.visual(
        Cylinder(radius=0.015, length=0.016),
        origin=_y_axis_origin((0.0, 0.042, 0.0)),
        material=dark_hardware,
        name="left_trunnion",
    )
    optical_tube.visual(
        Cylinder(radius=0.015, length=0.016),
        origin=_y_axis_origin((0.0, -0.042, 0.0)),
        material=dark_hardware,
        name="right_trunnion",
    )
    optical_tube.visual(
        Cylinder(radius=0.052, length=0.040),
        origin=_x_axis_origin((0.040, 0.0, 0.100)),
        material=mount_metal,
        name="tube_ring",
    )
    optical_tube.visual(
        Cylinder(radius=0.046, length=0.560),
        origin=_x_axis_origin((0.330, 0.0, 0.100)),
        material=painted_tube,
        name="main_tube",
    )
    optical_tube.visual(
        Cylinder(radius=0.040, length=0.070),
        origin=_x_axis_origin((0.000, 0.0, 0.100)),
        material=mount_metal,
        name="focuser_adapter",
    )
    optical_tube.visual(
        Cylinder(radius=0.034, length=0.100),
        origin=_x_axis_origin((-0.080, 0.0, 0.100)),
        material=mount_metal,
        name="focuser_housing",
    )
    optical_tube.visual(
        Cylinder(radius=0.008, length=0.016),
        origin=_y_axis_origin((-0.105, 0.038, 0.100)),
        material=dark_hardware,
        name="focus_knob_left",
    )
    optical_tube.visual(
        Cylinder(radius=0.008, length=0.016),
        origin=_y_axis_origin((-0.105, -0.038, 0.100)),
        material=dark_hardware,
        name="focus_knob_right",
    )
    optical_tube.visual(
        Cylinder(radius=0.054, length=0.040),
        origin=_x_axis_origin((0.630, 0.0, 0.100)),
        material=painted_tube,
        name="objective_cell",
    )
    optical_tube.visual(
        Cylinder(radius=0.058, length=0.180),
        origin=_x_axis_origin((0.740, 0.0, 0.100)),
        material=painted_tube,
        name="dew_shield",
    )
    optical_tube.visual(
        Cylinder(radius=0.052, length=0.012),
        origin=_x_axis_origin((0.824, 0.0, 0.100)),
        material=glass,
        name="objective_glass",
    )
    optical_tube.visual(
        Box((0.024, 0.018, 0.044)),
        origin=Origin(xyz=(0.130, 0.0, 0.149)),
        material=mount_metal,
        name="finder_rear_bracket",
    )
    optical_tube.visual(
        Box((0.024, 0.018, 0.044)),
        origin=Origin(xyz=(0.300, 0.0, 0.149)),
        material=mount_metal,
        name="finder_front_bracket",
    )
    optical_tube.visual(
        Cylinder(radius=0.012, length=0.220),
        origin=_x_axis_origin((0.215, 0.0, 0.182)),
        material=painted_tube,
        name="finder_scope",
    )
    optical_tube.visual(
        Cylinder(radius=0.010, length=0.025),
        origin=_x_axis_origin((0.325, 0.0, 0.182)),
        material=glass,
        name="finder_objective",
    )

    focuser = model.part("focuser")
    focuser.visual(
        Cylinder(radius=0.026, length=0.150),
        origin=_x_axis_origin((-0.025, 0.0, 0.0)),
        material=painted_tube,
        name="drawtube",
    )
    focuser.visual(
        Cylinder(radius=0.018, length=0.055),
        origin=_x_axis_origin((-0.120, 0.0, 0.0)),
        material=mount_metal,
        name="diagonal_body",
    )
    focuser.visual(
        Cylinder(radius=0.016, length=0.065),
        origin=Origin(xyz=(-0.147, 0.0, 0.0325)),
        material=mount_metal,
        name="diagonal_stem",
    )
    focuser.visual(
        Cylinder(radius=0.012, length=0.080),
        origin=Origin(xyz=(-0.147, 0.0, 0.105)),
        material=dark_hardware,
        name="eyepiece",
    )
    focuser.visual(
        Cylinder(radius=0.014, length=0.030),
        origin=Origin(xyz=(-0.147, 0.0, 0.160)),
        material=leather,
        name="eyecup",
    )

    model.articulation(
        "azimuth_axis",
        ArticulationType.REVOLUTE,
        parent=tripod,
        child=mount_head,
        origin=Origin(xyz=(0.0, 0.0, 1.060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "altitude_axis",
        ArticulationType.REVOLUTE,
        parent=mount_head,
        child=optical_tube,
        origin=Origin(xyz=(0.045, 0.0, 0.245)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=-0.35,
            upper=1.25,
        ),
    )
    model.articulation(
        "focus_drawtube",
        ArticulationType.PRISMATIC,
        parent=optical_tube,
        child=focuser,
        origin=Origin(xyz=(-0.105, 0.0, 0.100)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.060,
            lower=0.0,
            upper=0.040,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tripod = object_model.get_part("tripod")
    mount_head = object_model.get_part("mount_head")
    optical_tube = object_model.get_part("optical_tube")
    focuser = object_model.get_part("focuser")
    azimuth = object_model.get_articulation("azimuth_axis")
    altitude = object_model.get_articulation("altitude_axis")
    focus = object_model.get_articulation("focus_drawtube")

    tripod_plate = tripod.get_visual("tripod_plate")
    az_base = mount_head.get_visual("az_base")
    left_arm = mount_head.get_visual("left_arm")
    right_arm = mount_head.get_visual("right_arm")
    left_trunnion = optical_tube.get_visual("left_trunnion")
    right_trunnion = optical_tube.get_visual("right_trunnion")
    focuser_housing = optical_tube.get_visual("focuser_housing")
    dew_shield = optical_tube.get_visual("dew_shield")
    objective_glass = optical_tube.get_visual("objective_glass")
    main_tube = optical_tube.get_visual("main_tube")
    finder_scope = optical_tube.get_visual("finder_scope")
    drawtube = focuser.get_visual("drawtube")
    eyepiece = focuser.get_visual("eyepiece")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts(max_pose_samples=16)
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        focuser,
        optical_tube,
        elem_a=drawtube,
        elem_b=focuser_housing,
        reason="The drawtube telescopes inside the focuser housing through its focus range.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=96)

    ctx.expect_overlap(mount_head, tripod, axes="xy", min_overlap=0.100, elem_a=az_base, elem_b=tripod_plate)
    ctx.expect_gap(
        mount_head,
        tripod,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=az_base,
        negative_elem=tripod_plate,
    )
    ctx.expect_contact(mount_head, tripod, elem_a=az_base, elem_b=tripod_plate)

    ctx.expect_contact(optical_tube, mount_head, elem_a=left_trunnion, elem_b=left_arm)
    ctx.expect_contact(optical_tube, mount_head, elem_a=right_trunnion, elem_b=right_arm)
    ctx.expect_within(
        optical_tube,
        optical_tube,
        axes="yz",
        inner_elem=objective_glass,
        outer_elem=dew_shield,
    )
    ctx.expect_gap(
        optical_tube,
        optical_tube,
        axis="z",
        min_gap=0.020,
        positive_elem=finder_scope,
        negative_elem=main_tube,
    )
    ctx.expect_contact(focuser, optical_tube, elem_a=drawtube, elem_b=focuser_housing)

    tripod_aabb = ctx.part_world_aabb(tripod)
    optical_aabb = ctx.part_world_aabb(optical_tube)
    if tripod_aabb is not None:
        tripod_height = tripod_aabb[1][2] - tripod_aabb[0][2]
        ctx.check(
            "tripod_height_realistic",
            1.00 <= tripod_height <= 1.20,
            details=f"tripod height {tripod_height:.3f} m is outside a realistic deployed range",
        )
    if optical_aabb is not None:
        optical_length = optical_aabb[1][0] - optical_aabb[0][0]
        ctx.check(
            "refractor_length_realistic",
            optical_length >= 0.75,
            details=f"optical tube assembly length {optical_length:.3f} m is too short",
        )

    def _aabb_center(aabb):
        return (
            (aabb[0][0] + aabb[1][0]) * 0.5,
            (aabb[0][1] + aabb[1][1]) * 0.5,
            (aabb[0][2] + aabb[1][2]) * 0.5,
        )

    objective_rest_aabb = ctx.part_element_world_aabb(optical_tube, elem="objective_glass")
    eyepiece_rest_aabb = ctx.part_element_world_aabb(focuser, elem="eyepiece")
    if objective_rest_aabb is not None:
        objective_rest = _aabb_center(objective_rest_aabb)
        with ctx.pose({altitude: altitude.motion_limits.upper}):
            objective_up_aabb = ctx.part_element_world_aabb(optical_tube, elem="objective_glass")
            if objective_up_aabb is not None:
                objective_up = _aabb_center(objective_up_aabb)
                ctx.check(
                    "altitude_axis_raises_objective",
                    objective_up[2] > objective_rest[2] + 0.45,
                    details=(
                        f"objective moved from z={objective_rest[2]:.3f} m "
                        f"to z={objective_up[2]:.3f} m under altitude rotation"
                    ),
                )

        with ctx.pose({azimuth: 1.20}):
            objective_turned_aabb = ctx.part_element_world_aabb(optical_tube, elem="objective_glass")
            if objective_turned_aabb is not None:
                objective_turned = _aabb_center(objective_turned_aabb)
                ctx.check(
                    "azimuth_axis_swings_scope_around_tripod",
                    abs(objective_turned[1]) > 0.55 and objective_turned[0] < objective_rest[0] - 0.25,
                    details=(
                        f"objective center changed from ({objective_rest[0]:.3f}, {objective_rest[1]:.3f}) "
                        f"to ({objective_turned[0]:.3f}, {objective_turned[1]:.3f}) in the ground plane"
                    ),
                )

    if eyepiece_rest_aabb is not None:
        eyepiece_rest = _aabb_center(eyepiece_rest_aabb)
        with ctx.pose({focus: focus.motion_limits.upper}):
            eyepiece_extended_aabb = ctx.part_element_world_aabb(focuser, elem="eyepiece")
            if eyepiece_extended_aabb is not None:
                eyepiece_extended = _aabb_center(eyepiece_extended_aabb)
                ctx.check(
                    "focus_drawtube_extends_eyepiece",
                    eyepiece_extended[0] < eyepiece_rest[0] - 0.03,
                    details=(
                        f"eyepiece center moved from x={eyepiece_rest[0]:.3f} m "
                        f"to x={eyepiece_extended[0]:.3f} m during focusing"
                    ),
                )

    with ctx.pose({azimuth: 1.20}):
        ctx.expect_overlap(mount_head, tripod, axes="xy", min_overlap=0.100, elem_a=az_base, elem_b=tripod_plate)
        ctx.expect_gap(
            mount_head,
            tripod,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=az_base,
            negative_elem=tripod_plate,
        )
        ctx.expect_contact(mount_head, tripod, elem_a=az_base, elem_b=tripod_plate)

    with ctx.pose({altitude: altitude.motion_limits.upper}):
        ctx.expect_contact(optical_tube, mount_head, elem_a=left_trunnion, elem_b=left_arm)
        ctx.expect_contact(optical_tube, mount_head, elem_a=right_trunnion, elem_b=right_arm)
        ctx.expect_contact(focuser, optical_tube, elem_a=drawtube, elem_b=focuser_housing)

    with ctx.pose({azimuth: -1.10, altitude: altitude.motion_limits.lower, focus: focus.motion_limits.upper}):
        ctx.expect_contact(optical_tube, mount_head, elem_a=left_trunnion, elem_b=left_arm)
        ctx.expect_contact(optical_tube, mount_head, elem_a=right_trunnion, elem_b=right_arm)
        ctx.expect_contact(focuser, optical_tube, elem_a=drawtube, elem_b=focuser_housing)

    for articulation in (azimuth, altitude, focus):
        limits = articulation.motion_limits
        if limits is None or limits.lower is None or limits.upper is None:
            continue
        with ctx.pose({articulation: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{articulation.name}_lower_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{articulation.name}_lower_no_floating")
        with ctx.pose({articulation: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{articulation.name}_upper_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{articulation.name}_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
