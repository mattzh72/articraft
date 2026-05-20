from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os

try:
    os.getcwd()
except FileNotFoundError:
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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="refractor_telescope")

    cast_iron = model.material("cast_iron", rgba=(0.22, 0.22, 0.24, 1.0))
    tube_paint = model.material("tube_paint", rgba=(0.91, 0.89, 0.82, 1.0))
    brass = model.material("brass", rgba=(0.73, 0.58, 0.28, 1.0))
    leg_finish = model.material("leg_finish", rgba=(0.33, 0.20, 0.11, 1.0))
    rubber = model.material("rubber", rgba=(0.12, 0.12, 0.12, 1.0))
    satin_black = model.material("satin_black", rgba=(0.10, 0.10, 0.11, 1.0))
    glass = model.material("glass", rgba=(0.72, 0.84, 0.92, 0.45))

    along_x = (0.0, math.pi / 2.0, 0.0)
    along_y = (math.pi / 2.0, 0.0, 0.0)
    leg_tilt = math.radians(24.0)
    leg_length = 0.74
    leg_run = leg_length * math.sin(leg_tilt)
    leg_drop = leg_length * math.cos(leg_tilt)
    leg_mount_radius = 0.09
    leg_mount_z = 0.64

    tripod_core = model.part("tripod_core")
    tripod_core.visual(
        Cylinder(radius=0.035, length=0.50),
        origin=Origin(xyz=(0.0, 0.0, 0.50)),
        material=cast_iron,
        name="center_column",
    )
    tripod_core.visual(
        Cylinder(radius=0.050, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.34)),
        material=cast_iron,
        name="lower_collar",
    )
    tripod_core.visual(
        Cylinder(radius=0.090, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, leg_mount_z)),
        material=cast_iron,
        name="upper_casting",
    )
    tripod_core.visual(
        Cylinder(radius=0.085, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, 0.75)),
        material=cast_iron,
        name="top_plate",
    )
    tripod_core.visual(
        Cylinder(radius=0.040, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.31)),
        material=cast_iron,
        name="spreader_hub",
    )
    for name, yaw in (
        ("front", 0.0),
        ("left", 2.0 * math.pi / 3.0),
        ("right", -2.0 * math.pi / 3.0),
    ):
        socket_x = leg_mount_radius * math.cos(yaw)
        socket_y = leg_mount_radius * math.sin(yaw)
        arm_x = 0.11 * math.cos(yaw)
        arm_y = 0.11 * math.sin(yaw)
        tripod_core.visual(
            Box((0.060, 0.040, 0.060)),
            origin=Origin(xyz=(socket_x, socket_y, leg_mount_z), rpy=(0.0, 0.0, yaw)),
            material=cast_iron,
            name=f"{name}_socket",
        )
        tripod_core.visual(
            Box((0.200, 0.022, 0.014)),
            origin=Origin(xyz=(arm_x, arm_y, 0.31), rpy=(0.0, 0.0, yaw)),
            material=cast_iron,
            name=f"{name}_spreader_arm",
        )

    def add_leg(name: str, yaw: float) -> None:
        leg = model.part(name)
        leg.visual(
            Box((0.046, 0.026, leg_length)),
            origin=Origin(
                xyz=(0.5 * leg_run, 0.0, -0.5 * leg_drop),
                rpy=(0.0, -leg_tilt, 0.0),
            ),
            material=leg_finish,
            name="leg_beam",
        )
        leg.visual(
            Box((0.060, 0.042, 0.060)),
            origin=Origin(xyz=(0.0, 0.0, -0.030)),
            material=cast_iron,
            name="upper_clamp",
        )
        leg.visual(
            Box((0.090, 0.055, 0.020)),
            origin=Origin(xyz=(leg_run, 0.0, -leg_drop - 0.010)),
            material=rubber,
            name="foot_pad",
        )
        model.articulation(
            f"tripod_core_to_{name}",
            ArticulationType.FIXED,
            parent=tripod_core,
            child=leg,
            origin=Origin(
                xyz=(leg_mount_radius * math.cos(yaw), leg_mount_radius * math.sin(yaw), leg_mount_z),
                rpy=(0.0, 0.0, yaw),
            ),
        )

    add_leg("leg_front", 0.0)
    add_leg("leg_left", 2.0 * math.pi / 3.0)
    add_leg("leg_right", -2.0 * math.pi / 3.0)

    azimuth_head = model.part("azimuth_head")
    azimuth_head.visual(
        Cylinder(radius=0.074, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=cast_iron,
        name="base_turntable",
    )
    azimuth_head.visual(
        Cylinder(radius=0.028, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=cast_iron,
        name="pedestal",
    )
    azimuth_head.visual(
        Box((0.090, 0.064, 0.034)),
        origin=Origin(xyz=(0.005, 0.056, 0.065)),
        material=cast_iron,
        name="left_front_web",
    )
    azimuth_head.visual(
        Box((0.090, 0.064, 0.034)),
        origin=Origin(xyz=(0.005, -0.056, 0.065)),
        material=cast_iron,
        name="right_front_web",
    )
    azimuth_head.visual(
        Box((0.045, 0.014, 0.100)),
        origin=Origin(xyz=(-0.022, 0.091, 0.100)),
        material=cast_iron,
        name="left_gusset",
    )
    azimuth_head.visual(
        Box((0.045, 0.014, 0.100)),
        origin=Origin(xyz=(-0.022, -0.091, 0.100)),
        material=cast_iron,
        name="right_gusset",
    )
    azimuth_head.visual(
        Box((0.060, 0.030, 0.220)),
        origin=Origin(xyz=(0.000, 0.097, 0.150)),
        material=cast_iron,
        name="left_arm",
    )
    azimuth_head.visual(
        Box((0.060, 0.030, 0.220)),
        origin=Origin(xyz=(0.000, -0.097, 0.150)),
        material=cast_iron,
        name="right_arm",
    )
    azimuth_head.visual(
        Cylinder(radius=0.016, length=0.014),
        origin=Origin(xyz=(0.000, 0.075, 0.240), rpy=along_y),
        material=brass,
        name="left_bearing_boss",
    )
    azimuth_head.visual(
        Cylinder(radius=0.016, length=0.014),
        origin=Origin(xyz=(0.000, -0.075, 0.240), rpy=along_y),
        material=brass,
        name="right_bearing_boss",
    )

    optical_tube = model.part("optical_tube")
    optical_tube.visual(
        Cylinder(radius=0.055, length=0.72),
        origin=Origin(xyz=(0.320, 0.0, 0.0), rpy=along_x),
        material=tube_paint,
        name="main_tube",
    )
    optical_tube.visual(
        Cylinder(radius=0.064, length=0.20),
        origin=Origin(xyz=(0.770, 0.0, 0.0), rpy=along_x),
        material=tube_paint,
        name="dew_shield",
    )
    optical_tube.visual(
        Cylinder(radius=0.066, length=0.030),
        origin=Origin(xyz=(0.875, 0.0, 0.0), rpy=along_x),
        material=brass,
        name="objective_cell",
    )
    optical_tube.visual(
        Cylinder(radius=0.052, length=0.006),
        origin=Origin(xyz=(0.885, 0.0, 0.0), rpy=along_x),
        material=glass,
        name="objective_glass",
    )
    optical_tube.visual(
        Box((0.120, 0.088, 0.088)),
        origin=Origin(xyz=(-0.100, 0.0, 0.0)),
        material=satin_black,
        name="focuser_housing",
    )
    optical_tube.visual(
        Box((0.090, 0.060, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, -0.038)),
        material=brass,
        name="tube_saddle",
    )
    optical_tube.visual(
        Box((0.070, 0.016, 0.090)),
        origin=Origin(xyz=(0.0, 0.061, 0.0)),
        material=brass,
        name="left_cradle_cheek",
    )
    optical_tube.visual(
        Box((0.070, 0.016, 0.090)),
        origin=Origin(xyz=(0.0, -0.061, 0.0)),
        material=brass,
        name="right_cradle_cheek",
    )
    optical_tube.visual(
        Box((0.060, 0.018, 0.010)),
        origin=Origin(xyz=(0.210, 0.0, 0.060)),
        material=brass,
        name="finder_shoe",
    )
    optical_tube.visual(
        Cylinder(radius=0.015, length=0.014),
        origin=Origin(xyz=(0.0, 0.075, 0.0), rpy=along_y),
        material=brass,
        name="left_trunnion",
    )
    optical_tube.visual(
        Cylinder(radius=0.015, length=0.014),
        origin=Origin(xyz=(0.0, -0.075, 0.0), rpy=along_y),
        material=brass,
        name="right_trunnion",
    )

    drawtube_assembly = model.part("drawtube_assembly")
    drawtube_assembly.visual(
        Cylinder(radius=0.024, length=0.050),
        origin=Origin(xyz=(-0.020, 0.0, 0.0), rpy=along_x),
        material=satin_black,
        name="drawtube",
    )
    drawtube_assembly.visual(
        Cylinder(radius=0.022, length=0.060),
        origin=Origin(xyz=(-0.065, 0.0, 0.021), rpy=(0.0, -math.pi / 4.0, 0.0)),
        material=satin_black,
        name="star_diagonal",
    )
    drawtube_assembly.visual(
        Cylinder(radius=0.019, length=0.080),
        origin=Origin(xyz=(-0.065, 0.0, 0.082)),
        material=satin_black,
        name="eyepiece_barrel",
    )
    drawtube_assembly.visual(
        Cylinder(radius=0.023, length=0.030),
        origin=Origin(xyz=(-0.065, 0.0, 0.137)),
        material=rubber,
        name="eyecup",
    )

    focus_knob = model.part("focus_knob")
    focus_knob.visual(
        Cylinder(radius=0.019, length=0.016),
        origin=Origin(xyz=(0.0, -0.016, 0.0), rpy=along_y),
        material=satin_black,
        name="coarse_knob",
    )
    focus_knob.visual(
        Cylinder(radius=0.011, length=0.010),
        origin=Origin(xyz=(0.0, -0.028, 0.0), rpy=along_y),
        material=brass,
        name="fine_knob",
    )
    focus_knob.visual(
        Cylinder(radius=0.0025, length=0.018),
        origin=Origin(xyz=(0.0, -0.005, 0.0), rpy=along_y),
        material=cast_iron,
        name="knob_axle",
    )
    focus_knob.visual(
        Cylinder(radius=0.0025, length=0.016),
        origin=Origin(xyz=(0.008, -0.028, 0.010), rpy=along_x),
        material=brass,
        name="fine_handle",
    )

    model.articulation(
        "tripod_to_azimuth",
        ArticulationType.CONTINUOUS,
        parent=tripod_core,
        child=azimuth_head,
        origin=Origin(xyz=(0.0, 0.0, 0.760)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5),
    )
    model.articulation(
        "azimuth_to_optical_tube",
        ArticulationType.REVOLUTE,
        parent=azimuth_head,
        child=optical_tube,
        origin=Origin(xyz=(0.0, 0.0, 0.240)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.0,
            lower=math.radians(-70.0),
            upper=math.radians(18.0),
        ),
    )
    model.articulation(
        "optical_tube_to_drawtube",
        ArticulationType.PRISMATIC,
        parent=optical_tube,
        child=drawtube_assembly,
        origin=Origin(xyz=(-0.160, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.05,
            lower=0.0,
            upper=0.020,
        ),
    )
    model.articulation(
        "optical_tube_to_focus_knob",
        ArticulationType.REVOLUTE,
        parent=optical_tube,
        child=focus_knob,
        origin=Origin(xyz=(-0.135, -0.046, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=8.0,
            lower=-2.0 * math.pi,
            upper=2.0 * math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tripod_core = object_model.get_part("tripod_core")
    leg_front = object_model.get_part("leg_front")
    leg_left = object_model.get_part("leg_left")
    leg_right = object_model.get_part("leg_right")
    azimuth_head = object_model.get_part("azimuth_head")
    optical_tube = object_model.get_part("optical_tube")
    drawtube_assembly = object_model.get_part("drawtube_assembly")
    focus_knob = object_model.get_part("focus_knob")

    tripod_to_azimuth = object_model.get_articulation("tripod_to_azimuth")
    azimuth_to_optical_tube = object_model.get_articulation("azimuth_to_optical_tube")
    optical_tube_to_drawtube = object_model.get_articulation("optical_tube_to_drawtube")
    optical_tube_to_focus_knob = object_model.get_articulation("optical_tube_to_focus_knob")

    top_plate = tripod_core.get_visual("top_plate")
    front_socket = tripod_core.get_visual("front_socket")
    left_socket = tripod_core.get_visual("left_socket")
    right_socket = tripod_core.get_visual("right_socket")
    base_turntable = azimuth_head.get_visual("base_turntable")
    left_arm = azimuth_head.get_visual("left_arm")
    right_arm = azimuth_head.get_visual("right_arm")
    left_bearing_boss = azimuth_head.get_visual("left_bearing_boss")
    right_bearing_boss = azimuth_head.get_visual("right_bearing_boss")
    objective_glass = optical_tube.get_visual("objective_glass")
    focuser_housing = optical_tube.get_visual("focuser_housing")
    left_cradle_cheek = optical_tube.get_visual("left_cradle_cheek")
    right_cradle_cheek = optical_tube.get_visual("right_cradle_cheek")
    left_trunnion = optical_tube.get_visual("left_trunnion")
    right_trunnion = optical_tube.get_visual("right_trunnion")
    drawtube = drawtube_assembly.get_visual("drawtube")
    eyepiece_barrel = drawtube_assembly.get_visual("eyepiece_barrel")
    front_clamp = leg_front.get_visual("upper_clamp")
    left_clamp = leg_left.get_visual("upper_clamp")
    right_clamp = leg_right.get_visual("upper_clamp")
    knob_axle = focus_knob.get_visual("knob_axle")
    fine_handle = focus_knob.get_visual("fine_handle")

    def center_of(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
        lower, upper = aabb
        return tuple(0.5 * (lo + hi) for lo, hi in zip(lower, upper))

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()

    ctx.allow_overlap(
        leg_front,
        tripod_core,
        reason="front leg hardware is modeled as nested into the cast tripod head and spreader lug",
    )
    ctx.allow_overlap(
        leg_left,
        tripod_core,
        reason="left leg hardware is modeled as nested into the cast tripod head and spreader lug",
    )
    ctx.allow_overlap(
        leg_right,
        tripod_core,
        reason="right leg hardware is modeled as nested into the cast tripod head and spreader lug",
    )
    ctx.allow_overlap(
        optical_tube,
        azimuth_head,
        reason="altitude trunnions and brass cradle cheeks seat into the fork bearing blocks",
    )
    ctx.allow_overlap(
        drawtube_assembly,
        optical_tube,
        elem_a=drawtube,
        elem_b=focuser_housing,
        reason="the drawtube is intentionally nested inside the focuser body while remaining slidable",
    )
    ctx.allow_overlap(
        focus_knob,
        optical_tube,
        reason="fine-focus spindle is intentionally seated into the focuser housing",
    )

    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=64, overlap_tol=0.0015, overlap_volume_tol=0.0)

    ctx.expect_contact(leg_front, tripod_core, elem_a=front_clamp, elem_b=front_socket)
    ctx.expect_contact(leg_left, tripod_core, elem_a=left_clamp, elem_b=left_socket)
    ctx.expect_contact(leg_right, tripod_core, elem_a=right_clamp, elem_b=right_socket)

    ctx.expect_overlap(
        azimuth_head,
        tripod_core,
        axes="xy",
        min_overlap=0.010,
        elem_a=base_turntable,
        elem_b=top_plate,
    )
    ctx.expect_gap(
        azimuth_head,
        tripod_core,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=base_turntable,
        negative_elem=top_plate,
    )

    ctx.expect_contact(optical_tube, azimuth_head, elem_a=left_trunnion, elem_b=left_bearing_boss)
    ctx.expect_contact(optical_tube, azimuth_head, elem_a=right_trunnion, elem_b=right_bearing_boss)
    ctx.expect_contact(optical_tube, azimuth_head, elem_a=left_cradle_cheek, elem_b=left_bearing_boss)
    ctx.expect_contact(optical_tube, azimuth_head, elem_a=right_cradle_cheek, elem_b=right_bearing_boss)
    ctx.expect_contact(drawtube_assembly, optical_tube, elem_a=drawtube, elem_b=focuser_housing)
    ctx.expect_gap(
        optical_tube,
        azimuth_head,
        axis="x",
        min_gap=0.45,
        positive_elem=objective_glass,
        name="objective projects well forward of mount head",
    )

    ctx.expect_contact(focus_knob, optical_tube, elem_a=knob_axle, elem_b=focuser_housing)

    ctx.check(
        "azimuth_joint_is_vertical",
        tripod_to_azimuth.articulation_type == ArticulationType.CONTINUOUS
        and tripod_to_azimuth.axis == (0.0, 0.0, 1.0),
        "azimuth stage should rotate continuously about the vertical axis",
    )
    altitude_limits = azimuth_to_optical_tube.motion_limits
    ctx.check(
        "altitude_joint_has_realistic_range",
        altitude_limits is not None
        and altitude_limits.lower is not None
        and altitude_limits.upper is not None
        and altitude_limits.lower <= math.radians(-70.0)
        and altitude_limits.upper >= math.radians(15.0)
        and azimuth_to_optical_tube.axis == (0.0, 1.0, 0.0),
        "altitude joint should swing on the transverse axis with a mostly skyward refractor range",
    )
    focus_limits = optical_tube_to_focus_knob.motion_limits
    drawtube_limits = optical_tube_to_drawtube.motion_limits
    ctx.check(
        "focus_knob_joint_is_side_axis_revolute",
        optical_tube_to_focus_knob.articulation_type == ArticulationType.REVOLUTE
        and optical_tube_to_focus_knob.axis == (0.0, 1.0, 0.0)
        and focus_limits is not None
        and focus_limits.lower is not None
        and focus_limits.upper is not None
        and focus_limits.lower <= -2.0 * math.pi
        and focus_limits.upper >= 2.0 * math.pi,
        "fine-focus knob should rotate about its spindle with multiple turns of travel",
    )
    ctx.check(
        "drawtube_joint_is_focuser_slide",
        optical_tube_to_drawtube.articulation_type == ArticulationType.PRISMATIC
        and optical_tube_to_drawtube.axis == (-1.0, 0.0, 0.0)
        and drawtube_limits is not None
        and drawtube_limits.lower == 0.0
        and drawtube_limits.upper is not None
        and drawtube_limits.upper >= 0.018,
        "the focuser drawtube should slide rearward with a short realistic travel range",
    )

    objective_rest = ctx.part_element_world_aabb(optical_tube, elem=objective_glass)
    eyepiece_rest = ctx.part_element_world_aabb(drawtube_assembly, elem=eyepiece_barrel)
    fine_handle_rest = ctx.part_element_world_aabb(focus_knob, elem=fine_handle)
    ctx.check(
        "objective_aabb_available",
        objective_rest is not None,
        "objective glass should produce a measurable world-space AABB",
    )
    ctx.check(
        "eyepiece_aabb_available",
        eyepiece_rest is not None,
        "eyepiece barrel should produce a measurable world-space AABB",
    )
    ctx.check(
        "focus_handle_aabb_available",
        fine_handle_rest is not None,
        "fine-focus handle should produce a measurable world-space AABB",
    )

    with ctx.pose({optical_tube_to_focus_knob: math.pi}):
        ctx.fail_if_parts_overlap_in_current_pose(name="focus_knob_half_turn_no_unallowed_overlap")
        ctx.fail_if_isolated_parts(name="focus_knob_half_turn_no_floating")
        ctx.expect_contact(focus_knob, optical_tube, elem_a=knob_axle, elem_b=focuser_housing)
        fine_handle_half_turn = ctx.part_element_world_aabb(focus_knob, elem=fine_handle)
        if fine_handle_rest is not None and fine_handle_half_turn is not None:
            rest_center = center_of(fine_handle_rest)
            turned_center = center_of(fine_handle_half_turn)
            ctx.check(
                "fine_focus_handle_orbits_spindle",
                abs(turned_center[0] - rest_center[0]) > 0.010 and abs(turned_center[2] - rest_center[2]) > 0.010,
                "the fine-focus handle should visibly move in an arc when the knob turns",
            )

    if drawtube_limits is not None and drawtube_limits.upper is not None:
        with ctx.pose({optical_tube_to_drawtube: drawtube_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="drawtube_extended_no_unallowed_overlap")
            ctx.fail_if_isolated_parts(name="drawtube_extended_no_floating")
            ctx.expect_overlap(
                drawtube_assembly,
                optical_tube,
                axes="yz",
                min_overlap=0.03,
                elem_a=drawtube,
                elem_b=focuser_housing,
                name="drawtube_stays_guided_in_focuser_body",
            )
            eyepiece_extended = ctx.part_element_world_aabb(drawtube_assembly, elem=eyepiece_barrel)
            if eyepiece_rest is not None and eyepiece_extended is not None:
                rest_center = center_of(eyepiece_rest)
                extended_center = center_of(eyepiece_extended)
                ctx.check(
                    "drawtube_extension_moves_eyepiece_back",
                    extended_center[0] < rest_center[0] - 0.015,
                    "extending the focuser should slide the eyepiece rearward",
                )

    with ctx.pose({tripod_to_azimuth: math.radians(50.0)}):
        ctx.fail_if_parts_overlap_in_current_pose(name="azimuth_rotated_no_unallowed_overlap")
        ctx.fail_if_isolated_parts(name="azimuth_rotated_no_floating")
        azimuth_turn_objective = ctx.part_element_world_aabb(optical_tube, elem=objective_glass)
        if objective_rest is not None and azimuth_turn_objective is not None:
            rest_center = center_of(objective_rest)
            turned_center = center_of(azimuth_turn_objective)
            ctx.check(
                "azimuth_rotation_swings_objective_sideways",
                abs(turned_center[1] - rest_center[1]) > 0.50,
                "turning the alt-az head should swing the objective sideways around the tripod axis",
            )

    if altitude_limits is not None and altitude_limits.lower is not None and altitude_limits.upper is not None:
        with ctx.pose({azimuth_to_optical_tube: altitude_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="altitude_lower_no_unallowed_overlap")
            ctx.fail_if_isolated_parts(name="altitude_lower_no_floating")
            ctx.expect_contact(optical_tube, azimuth_head, elem_a=left_trunnion, elem_b=left_bearing_boss)
            ctx.expect_contact(optical_tube, azimuth_head, elem_a=right_trunnion, elem_b=right_bearing_boss)
            objective_up = ctx.part_element_world_aabb(optical_tube, elem=objective_glass)
            if objective_rest is not None and objective_up is not None:
                rest_center = center_of(objective_rest)
                up_center = center_of(objective_up)
                ctx.check(
                    "altitude_raise_lifts_objective",
                    up_center[2] > rest_center[2] + 0.55,
                    "raising the telescope should carry the objective well above the mount head",
                )

        with ctx.pose({azimuth_to_optical_tube: altitude_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="altitude_upper_no_unallowed_overlap")
            ctx.fail_if_isolated_parts(name="altitude_upper_no_floating")
            ctx.expect_contact(optical_tube, azimuth_head, elem_a=left_trunnion, elem_b=left_bearing_boss)
            ctx.expect_contact(optical_tube, azimuth_head, elem_a=right_trunnion, elem_b=right_bearing_boss)

    if focus_limits is not None and focus_limits.lower is not None and focus_limits.upper is not None:
        with ctx.pose({optical_tube_to_focus_knob: focus_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="focus_knob_lower_no_unallowed_overlap")
            ctx.fail_if_isolated_parts(name="focus_knob_lower_no_floating")
            ctx.expect_contact(focus_knob, optical_tube, elem_a=knob_axle, elem_b=focuser_housing)

        with ctx.pose({optical_tube_to_focus_knob: focus_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="focus_knob_upper_no_unallowed_overlap")
            ctx.fail_if_isolated_parts(name="focus_knob_upper_no_floating")
            ctx.expect_contact(focus_knob, optical_tube, elem_a=knob_axle, elem_b=focuser_housing)

    with ctx.pose({tripod_to_azimuth: math.radians(50.0), azimuth_to_optical_tube: math.radians(-42.0)}):
        ctx.fail_if_parts_overlap_in_current_pose(name="tracking_pose_no_unallowed_overlap")
        ctx.fail_if_isolated_parts(name="tracking_pose_no_floating")
        ctx.expect_contact(optical_tube, azimuth_head, elem_a=left_trunnion, elem_b=left_bearing_boss)
        ctx.expect_contact(optical_tube, azimuth_head, elem_a=right_trunnion, elem_b=right_bearing_boss)
        ctx.expect_gap(
            optical_tube,
            tripod_core,
            axis="z",
            min_gap=0.42,
            positive_elem=objective_glass,
            negative_elem=top_plate,
            name="raised telescope points distinctly above the tripod head",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
