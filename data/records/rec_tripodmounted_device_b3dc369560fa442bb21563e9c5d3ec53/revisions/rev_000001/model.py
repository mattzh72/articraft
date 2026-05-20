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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _leg_direction(azimuth_deg: float, splay_deg: float) -> tuple[float, float, float]:
    azimuth = math.radians(azimuth_deg)
    splay = math.radians(splay_deg)
    return (
        math.sin(splay) * math.cos(azimuth),
        math.sin(splay) * math.sin(azimuth),
        -math.cos(splay),
    )


def _axis_rpy_from_direction(direction: tuple[float, float, float]) -> tuple[float, float, float]:
    dx, dy, dz = direction
    azimuth = math.atan2(dy, dx)
    splay = math.atan2(math.hypot(dx, dy), -dz)
    return (0.0, math.pi - splay, azimuth)


def _scaled(direction: tuple[float, float, float], distance: float) -> tuple[float, float, float]:
    return tuple(component * distance for component in direction)


def _offset(base: tuple[float, float, float], delta: tuple[float, float, float]) -> tuple[float, float, float]:
    return tuple(a + b for a, b in zip(base, delta))


def _add_tripod_leg(part, *, azimuth_deg: float, metal: str, dark_metal: str, rubber: str) -> None:
    direction = _leg_direction(azimuth_deg, 20.0)
    leg_rpy = _axis_rpy_from_direction(direction)

    hinge_bottom = (0.0, 0.0, -0.05)
    upper_center = _offset(hinge_bottom, _scaled(direction, 0.31))
    collar_center = _offset(hinge_bottom, _scaled(direction, 0.60))
    lower_center = _offset(hinge_bottom, _scaled(direction, 0.82))
    foot_center = _offset(hinge_bottom, _scaled(direction, 1.039))

    part.visual(
        Box((0.060, 0.036, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=dark_metal,
        name="hinge_block",
    )
    part.visual(
        Box((0.055, 0.042, 0.620)),
        origin=Origin(xyz=upper_center, rpy=leg_rpy),
        material=metal,
        name="upper_tube",
    )
    part.visual(
        Box((0.070, 0.056, 0.065)),
        origin=Origin(xyz=collar_center, rpy=leg_rpy),
        material=dark_metal,
        name="lock_collar",
    )
    part.visual(
        Box((0.040, 0.032, 0.400)),
        origin=Origin(xyz=lower_center, rpy=leg_rpy),
        material=metal,
        name="lower_tube",
    )
    part.visual(
        Cylinder(radius=0.032, length=0.038),
        origin=Origin(xyz=foot_center, rpy=leg_rpy),
        material=rubber,
        name="foot_pad",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_service_tripod_device")

    aluminum = model.material("aluminum", rgba=(0.60, 0.62, 0.66, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.20, 0.22, 0.25, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.06, 1.0))
    polymer = model.material("polymer", rgba=(0.12, 0.12, 0.13, 1.0))
    device_gray = model.material("device_gray", rgba=(0.34, 0.36, 0.39, 1.0))

    apex = model.part("apex")
    apex.visual(
        Box((0.180, 0.180, 0.100)),
        origin=Origin(xyz=(0.0, 0.0, -0.050)),
        material=dark_steel,
        name="crown_body",
    )
    apex.visual(
        Cylinder(radius=0.095, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=aluminum,
        name="head_plate",
    )
    apex.visual(
        Cylinder(radius=0.048, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=dark_steel,
        name="service_boss",
    )

    leg_front = model.part("leg_front")
    _add_tripod_leg(leg_front, azimuth_deg=0.0, metal=aluminum, dark_metal=dark_steel, rubber=rubber)

    leg_left = model.part("leg_left")
    _add_tripod_leg(leg_left, azimuth_deg=130.0, metal=aluminum, dark_metal=dark_steel, rubber=rubber)

    leg_right = model.part("leg_right")
    _add_tripod_leg(leg_right, azimuth_deg=-130.0, metal=aluminum, dark_metal=dark_steel, rubber=rubber)

    model.articulation(
        "apex_to_leg_front",
        ArticulationType.FIXED,
        parent=apex,
        child=leg_front,
        origin=Origin(xyz=(0.058, 0.0, -0.100)),
    )
    model.articulation(
        "apex_to_leg_left",
        ArticulationType.FIXED,
        parent=apex,
        child=leg_left,
        origin=Origin(xyz=(-0.037, 0.044, -0.100)),
    )
    model.articulation(
        "apex_to_leg_right",
        ArticulationType.FIXED,
        parent=apex,
        child=leg_right,
        origin=Origin(xyz=(-0.037, -0.044, -0.100)),
    )

    head_base = model.part("head_base")
    head_base.visual(
        Cylinder(radius=0.085, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=dark_steel,
        name="mount_flange",
    )
    head_base.visual(
        Cylinder(radius=0.070, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=dark_steel,
        name="pan_housing",
    )
    head_base.visual(
        Box((0.095, 0.060, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=dark_steel,
        name="service_window_frame",
    )
    head_base.visual(
        Cylinder(radius=0.014, length=0.030),
        origin=Origin(xyz=(0.084, 0.0, 0.050), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polymer,
        name="pan_lock_knob",
    )
    model.articulation(
        "apex_to_head_base",
        ArticulationType.FIXED,
        parent=apex,
        child=head_base,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
    )

    pan_stage = model.part("pan_stage")
    pan_stage.visual(
        Cylinder(radius=0.075, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=dark_steel,
        name="pan_disk",
    )
    pan_stage.visual(
        Box((0.060, 0.060, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material=dark_steel,
        name="pan_core",
    )
    pan_stage.visual(
        Box((0.050, 0.110, 0.030)),
        origin=Origin(xyz=(-0.025, 0.0, 0.097)),
        material=dark_steel,
        name="tilt_shoulder",
    )
    pan_stage.visual(
        Box((0.020, 0.130, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.076)),
        material=dark_steel,
        name="lower_bridge",
    )
    pan_stage.visual(
        Box((0.030, 0.020, 0.120)),
        origin=Origin(xyz=(0.0, 0.065, 0.136)),
        material=dark_steel,
        name="left_cheek",
    )
    pan_stage.visual(
        Box((0.030, 0.020, 0.120)),
        origin=Origin(xyz=(0.0, -0.065, 0.136)),
        material=dark_steel,
        name="right_cheek",
    )
    pan_stage.visual(
        Box((0.020, 0.150, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.196)),
        material=dark_steel,
        name="top_bridge",
    )
    pan_stage.visual(
        Box((0.030, 0.030, 0.030)),
        origin=Origin(xyz=(-0.030, -0.075, 0.115)),
        material=dark_steel,
        name="handle_boss",
    )
    pan_stage.visual(
        Cylinder(radius=0.012, length=0.160),
        origin=Origin(xyz=(-0.110, -0.075, 0.115), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polymer,
        name="pan_handle",
    )
    model.articulation(
        "pan",
        ArticulationType.CONTINUOUS,
        parent=head_base,
        child=pan_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0),
    )

    bracket_frame = model.part("bracket_frame")
    bracket_frame.visual(
        Cylinder(radius=0.009, length=0.110),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="tilt_shaft",
    )
    bracket_frame.visual(
        Box((0.060, 0.080, 0.060)),
        origin=Origin(xyz=(0.040, 0.0, -0.010)),
        material=aluminum,
        name="backbone",
    )
    bracket_frame.visual(
        Box((0.020, 0.030, 0.040)),
        origin=Origin(xyz=(0.015, 0.040, 0.0)),
        material=aluminum,
        name="right_trunnion_block",
    )
    bracket_frame.visual(
        Box((0.020, 0.030, 0.040)),
        origin=Origin(xyz=(0.015, -0.040, 0.0)),
        material=aluminum,
        name="left_trunnion_block",
    )
    bracket_frame.visual(
        Box((0.080, 0.050, 0.012)),
        origin=Origin(xyz=(0.160, 0.125, -0.085)),
        material=aluminum,
        name="right_arm",
    )
    bracket_frame.visual(
        Box((0.080, 0.050, 0.012)),
        origin=Origin(xyz=(0.160, -0.125, -0.085)),
        material=aluminum,
        name="left_arm",
    )
    bracket_frame.visual(
        Box((0.050, 0.180, 0.110)),
        origin=Origin(xyz=(0.085, 0.0, -0.030)),
        material=aluminum,
        name="rail_gusset",
    )
    bracket_frame.visual(
        Box((0.180, 0.250, 0.012)),
        origin=Origin(xyz=(0.160, 0.0, -0.085)),
        material=aluminum,
        name="lower_rail",
    )
    bracket_frame.visual(
        Box((0.060, 0.014, 0.104)),
        origin=Origin(xyz=(0.160, -0.104, -0.029)),
        material=aluminum,
        name="fixed_jaw_body",
    )
    bracket_frame.visual(
        Box((0.055, 0.006, 0.094)),
        origin=Origin(xyz=(0.160, -0.100, -0.029)),
        material=rubber,
        name="fixed_jaw_pad",
    )
    bracket_frame.visual(
        Box((0.012, 0.100, 0.012)),
        origin=Origin(xyz=(0.244, 0.0, -0.085)),
        material=dark_steel,
        name="front_lip",
    )
    bracket_frame.visual(
        Cylinder(radius=0.012, length=0.028),
        origin=Origin(xyz=(0.130, -0.118, -0.005), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polymer,
        name="tilt_lock_knob",
    )
    model.articulation(
        "tilt",
        ArticulationType.REVOLUTE,
        parent=pan_stage,
        child=bracket_frame,
        origin=Origin(xyz=(0.005, 0.0, 0.136)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=-0.75, upper=1.05),
    )

    device_body = model.part("device_body")
    device_body.visual(
        Box((0.160, 0.170, 0.140)),
        material=device_gray,
        name="housing",
    )
    device_body.visual(
        Box((0.010, 0.130, 0.095)),
        origin=Origin(xyz=(0.085, 0.0, 0.0)),
        material=polymer,
        name="front_bezel",
    )
    device_body.visual(
        Box((0.140, 0.012, 0.100)),
        origin=Origin(xyz=(0.0, 0.091, 0.0)),
        material=polymer,
        name="right_bumper",
    )
    device_body.visual(
        Box((0.140, 0.012, 0.100)),
        origin=Origin(xyz=(0.0, -0.091, 0.0)),
        material=polymer,
        name="left_bumper",
    )
    device_body.visual(
        Box((0.080, 0.120, 0.010)),
        origin=Origin(xyz=(-0.010, 0.0, 0.075)),
        material=dark_steel,
        name="service_hatch",
    )
    model.articulation(
        "bracket_to_device",
        ArticulationType.FIXED,
        parent=bracket_frame,
        child=device_body,
        origin=Origin(xyz=(0.205, 0.0, -0.009)),
    )

    clamp_jaw = model.part("clamp_jaw")
    clamp_jaw.visual(
        Box((0.060, 0.014, 0.104)),
        origin=Origin(xyz=(0.0, 0.0, -0.029)),
        material=aluminum,
        name="jaw_body",
    )
    clamp_jaw.visual(
        Box((0.055, 0.006, 0.094)),
        origin=Origin(xyz=(0.0, -0.004, -0.029)),
        material=rubber,
        name="jaw_pad",
    )
    clamp_jaw.visual(
        Cylinder(radius=0.014, length=0.028),
        origin=Origin(xyz=(0.0, 0.018, -0.005), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polymer,
        name="adjuster_knob",
    )
    model.articulation(
        "clamp_slide",
        ArticulationType.PRISMATIC,
        parent=bracket_frame,
        child=clamp_jaw,
        origin=Origin(xyz=(0.160, 0.104, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.10, lower=0.0, upper=0.035),
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
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    apex = object_model.get_part("apex")
    leg_front = object_model.get_part("leg_front")
    leg_left = object_model.get_part("leg_left")
    leg_right = object_model.get_part("leg_right")
    head_base = object_model.get_part("head_base")
    pan_stage = object_model.get_part("pan_stage")
    bracket_frame = object_model.get_part("bracket_frame")
    device_body = object_model.get_part("device_body")
    clamp_jaw = object_model.get_part("clamp_jaw")

    pan = object_model.get_articulation("pan")
    tilt = object_model.get_articulation("tilt")
    clamp_slide = object_model.get_articulation("clamp_slide")

    ctx.expect_contact(leg_front, apex, contact_tol=1e-4, name="front_leg_seated_to_apex")
    ctx.expect_contact(leg_left, apex, contact_tol=1e-4, name="left_leg_seated_to_apex")
    ctx.expect_contact(leg_right, apex, contact_tol=1e-4, name="right_leg_seated_to_apex")
    ctx.expect_contact(head_base, apex, contact_tol=1e-4, name="head_base_bolted_to_plate")
    ctx.expect_contact(pan_stage, head_base, contact_tol=1e-4, name="pan_stage_supported_by_head_base")
    ctx.expect_contact(bracket_frame, pan_stage, contact_tol=1e-4, name="tilt_frame_supported_in_yoke")
    ctx.expect_contact(device_body, bracket_frame, contact_tol=1e-4, name="device_housing_supported_by_bracket")
    ctx.expect_contact(clamp_jaw, bracket_frame, contact_tol=1e-4, name="clamp_jaw_guided_on_rail")
    ctx.expect_contact(clamp_jaw, device_body, contact_tol=1e-4, name="clamp_jaw_closes_on_device")

    ctx.check(
        "head_joint_types_and_axes",
        pan.articulation_type == ArticulationType.CONTINUOUS
        and pan.axis == (0.0, 0.0, 1.0)
        and tilt.articulation_type == ArticulationType.REVOLUTE
        and tilt.axis == (0.0, -1.0, 0.0)
        and clamp_slide.articulation_type == ArticulationType.PRISMATIC
        and clamp_slide.axis == (0.0, 1.0, 0.0),
        details="Pan, tilt, and clamp axes should match a real tripod head layout.",
    )

    level_device_pos = ctx.part_world_position(device_body)
    with ctx.pose(pan=0.70):
        panned_device_pos = ctx.part_world_position(device_body)
    with ctx.pose(tilt=0.70):
        tilted_device_pos = ctx.part_world_position(device_body)
    jaw_closed_pos = ctx.part_world_position(clamp_jaw)
    with ctx.pose(clamp_slide=0.030):
        jaw_open_pos = ctx.part_world_position(clamp_jaw)
        ctx.expect_contact(clamp_jaw, bracket_frame, contact_tol=1e-4, name="clamp_jaw_stays_supported_when_open")
        ctx.expect_gap(
            clamp_jaw,
            device_body,
            axis="y",
            min_gap=0.020,
            name="clamp_can_release_device",
        )

    ctx.check(
        "pan_moves_device_around_vertical_axis",
        level_device_pos is not None
        and panned_device_pos is not None
        and panned_device_pos[1] > level_device_pos[1] + 0.05,
        details="Positive pan should swing the mounted device toward +Y.",
    )
    ctx.check(
        "tilt_raises_device",
        level_device_pos is not None
        and tilted_device_pos is not None
        and tilted_device_pos[2] > level_device_pos[2] + 0.05,
        details="Positive tilt should raise the front-carried device.",
    )
    ctx.check(
        "clamp_opens_in_positive_y",
        jaw_closed_pos is not None
        and jaw_open_pos is not None
        and jaw_open_pos[1] > jaw_closed_pos[1] + 0.020,
        details="Positive clamp travel should open the movable jaw outward.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
