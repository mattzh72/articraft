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
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _yz_rounded_section(width_y: float, height_z: float, radius: float, x: float):
    return [(x, y, z) for y, z in rounded_rect_profile(width_y, height_z, radius)]


def _leg_direction(theta: float, spread_angle: float) -> tuple[float, float, float]:
    return (
        math.sin(spread_angle) * math.cos(theta),
        math.sin(spread_angle) * math.sin(theta),
        -math.cos(spread_angle),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_tripod_device")

    painted_metal = model.material("painted_metal", rgba=(0.23, 0.25, 0.29, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.63, 0.66, 0.70, 1.0))
    polymer = model.material("polymer", rgba=(0.12, 0.13, 0.15, 1.0))
    elastomer = model.material("elastomer", rgba=(0.16, 0.16, 0.17, 1.0))
    glass = model.material("glass", rgba=(0.06, 0.07, 0.08, 1.0))

    tripod_base = model.part("tripod_base")
    tripod_base.visual(
        Cylinder(radius=0.018, length=0.88),
        origin=Origin(xyz=(0.0, 0.0, 0.69)),
        material=painted_metal,
        name="center_column",
    )
    tripod_base.visual(
        Cylinder(radius=0.038, length=0.13),
        origin=Origin(xyz=(0.0, 0.0, 1.145)),
        material=painted_metal,
        name="crown_hub",
    )
    tripod_base.visual(
        Cylinder(radius=0.034, length=0.07),
        origin=Origin(xyz=(0.0, 0.0, 1.225)),
        material=painted_metal,
        name="upper_collar",
    )
    tripod_base.visual(
        Cylinder(radius=0.036, length=0.01),
        origin=Origin(xyz=(0.0, 0.0, 1.265)),
        material=polymer,
        name="pan_base",
    )

    leg_socket_length = 0.055
    leg_spread_angle = math.radians(25.0)
    leg_socket_inner_radius = 0.040
    leg_socket_z = 1.145
    leg_specs = {
        "front": 0.0,
        "left": 2.0 * math.pi / 3.0,
        "right": -2.0 * math.pi / 3.0,
    }
    for leg_name, theta in leg_specs.items():
        dx, dy, dz = _leg_direction(theta, leg_spread_angle)
        socket_inner = (
            leg_socket_inner_radius * math.cos(theta),
            leg_socket_inner_radius * math.sin(theta),
            leg_socket_z,
        )
        socket_center = (
            socket_inner[0] + dx * (leg_socket_length * 0.5),
            socket_inner[1] + dy * (leg_socket_length * 0.5),
            socket_inner[2] + dz * (leg_socket_length * 0.5),
        )
        tripod_base.visual(
            Cylinder(radius=0.017, length=leg_socket_length),
            origin=Origin(
                xyz=socket_center,
                rpy=(0.0, math.pi - leg_spread_angle, theta),
            ),
            material=painted_metal,
            name=f"{leg_name}_socket",
        )
        flange_center = (
            0.032 * math.cos(theta),
            0.032 * math.sin(theta),
            1.125,
        )
        tripod_base.visual(
            Box((0.050, 0.030, 0.028)),
            origin=Origin(xyz=flange_center, rpy=(0.0, 0.0, theta)),
            material=painted_metal,
            name=f"{leg_name}_flange",
        )

    tripod_base.inertial = Inertial.from_geometry(
        Box((0.18, 0.18, 1.32)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, 0.66)),
    )

    upper_leg_len = 0.66
    lower_leg_len = 0.58
    for leg_name, theta in leg_specs.items():
        leg = model.part(f"{leg_name}_leg")
        leg.visual(
            Cylinder(radius=0.015, length=upper_leg_len),
            origin=Origin(xyz=(0.0, 0.0, upper_leg_len * 0.5)),
            material=painted_metal,
            name="upper_tube",
        )
        leg.visual(
            Cylinder(radius=0.0165, length=0.038),
            origin=Origin(xyz=(0.0, 0.0, 0.642)),
            material=polymer,
            name="leg_lock",
        )
        leg.visual(
            Cylinder(radius=0.012, length=lower_leg_len),
            origin=Origin(xyz=(0.0, 0.0, 0.900)),
            material=satin_metal,
            name="lower_tube",
        )
        leg.visual(
            Cylinder(radius=0.018, length=0.03),
            origin=Origin(xyz=(0.0, 0.0, 1.205)),
            material=elastomer,
            name="foot",
        )
        leg.inertial = Inertial.from_geometry(
            Box((0.05, 0.05, 1.23)),
            mass=0.7,
            origin=Origin(xyz=(0.0, 0.0, 0.62)),
        )

        dx, dy, dz = _leg_direction(theta, leg_spread_angle)
        leg_socket_inner = (
            leg_socket_inner_radius * math.cos(theta),
            leg_socket_inner_radius * math.sin(theta),
            leg_socket_z,
        )
        leg_origin = (
            leg_socket_inner[0] + dx * leg_socket_length,
            leg_socket_inner[1] + dy * leg_socket_length,
            leg_socket_inner[2] + dz * leg_socket_length,
        )
        model.articulation(
            f"tripod_base_to_{leg_name}_leg",
            ArticulationType.FIXED,
            parent=tripod_base,
            child=leg,
            origin=Origin(
                xyz=leg_origin,
                rpy=(0.0, math.pi - leg_spread_angle, theta),
            ),
        )

    pan_stage = model.part("pan_stage")
    pan_stage.visual(
        Cylinder(radius=0.036, length=0.01),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=polymer,
        name="pan_disc",
    )
    pan_stage.visual(
        Cylinder(radius=0.028, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        material=polymer,
        name="pan_collar",
    )
    pan_stage.visual(
        Box((0.018, 0.028, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=polymer,
        name="head_spine",
    )
    for side, y_center, boss_y in (("left", 0.034, 0.044), ("right", -0.034, -0.044)):
        pan_stage.visual(
            Box((0.018, 0.010, 0.050)),
            origin=Origin(xyz=(0.0, y_center, 0.078)),
            material=polymer,
            name=f"{side}_cheek",
        )
        pan_stage.visual(
            Box((0.014, 0.022, 0.014)),
            origin=Origin(
                xyz=(0.0, 0.023 if side == "left" else -0.023, 0.066),
            ),
            material=polymer,
            name=f"{side}_web",
        )
        pan_stage.visual(
            Cylinder(radius=0.012, length=0.010),
            origin=Origin(
                xyz=(0.0, boss_y, 0.078),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=painted_metal,
            name=f"{side}_tilt_boss",
        )
    pan_stage.inertial = Inertial.from_geometry(
        Box((0.11, 0.11, 0.11)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
    )

    model.articulation(
        "tripod_base_to_pan_stage",
        ArticulationType.REVOLUTE,
        parent=tripod_base,
        child=pan_stage,
        origin=Origin(xyz=(0.0, 0.0, 1.27)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.5,
            lower=-math.radians(150.0),
            upper=math.radians(150.0),
        ),
    )

    tilt_head = model.part("tilt_head")
    for side, y_center in (("left", 0.024), ("right", -0.024)):
        tilt_head.visual(
            Cylinder(radius=0.010, length=0.010),
            origin=Origin(
                xyz=(0.0, y_center, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=painted_metal,
            name=f"{side}_trunnion",
        )
    tilt_head.visual(
        Box((0.028, 0.044, 0.018)),
        origin=Origin(xyz=(0.022, 0.0, 0.0)),
        material=painted_metal,
        name="bridge",
    )
    tilt_head.visual(
        Box((0.060, 0.028, 0.018)),
        origin=Origin(xyz=(0.060, 0.0, 0.0)),
        material=painted_metal,
        name="support_arm",
    )
    tilt_head.visual(
        Box((0.016, 0.028, 0.056)),
        origin=Origin(xyz=(0.066, 0.0, 0.023)),
        material=painted_metal,
        name="upright_fin",
    )
    tilt_head.visual(
        Box((0.052, 0.034, 0.008)),
        origin=Origin(xyz=(0.074, 0.0, 0.050)),
        material=polymer,
        name="plate_saddle",
    )
    tilt_head.visual(
        Box((0.018, 0.020, 0.026)),
        origin=Origin(xyz=(0.098, 0.0, 0.020)),
        material=polymer,
        name="front_nose",
    )
    tilt_head.inertial = Inertial.from_geometry(
        Box((0.12, 0.08, 0.10)),
        mass=0.35,
        origin=Origin(xyz=(0.050, 0.0, 0.020)),
    )

    model.articulation(
        "pan_stage_to_tilt_head",
        ArticulationType.REVOLUTE,
        parent=pan_stage,
        child=tilt_head,
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=1.2,
            lower=-math.radians(45.0),
            upper=math.radians(60.0),
        ),
    )

    head_plate = model.part("head_plate")
    plate_geom = ExtrudeGeometry(
        rounded_rect_profile(0.062, 0.040, 0.006),
        0.01,
        center=True,
    )
    plate_mesh = mesh_from_geometry(plate_geom, "head_plate_shell")
    head_plate.visual(
        plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=satin_metal,
        name="plate_shell",
    )
    head_plate.visual(
        Box((0.048, 0.006, 0.004)),
        origin=Origin(xyz=(0.0, 0.014, 0.002)),
        material=painted_metal,
        name="left_dovetail",
    )
    head_plate.visual(
        Box((0.048, 0.006, 0.004)),
        origin=Origin(xyz=(0.0, -0.014, 0.002)),
        material=painted_metal,
        name="right_dovetail",
    )
    head_plate.inertial = Inertial.from_geometry(
        Box((0.062, 0.040, 0.010)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
    )

    model.articulation(
        "tilt_head_to_head_plate",
        ArticulationType.FIXED,
        parent=tilt_head,
        child=head_plate,
        origin=Origin(xyz=(0.074, 0.0, 0.054)),
    )

    clamp_body = model.part("clamp_body")
    clamp_body.visual(
        Box((0.016, 0.024, 0.008)),
        origin=Origin(xyz=(-0.012, 0.0, -0.076)),
        material=painted_metal,
        name="mount_block",
    )
    clamp_body.visual(
        Box((0.012, 0.032, 0.088)),
        origin=Origin(xyz=(-0.012, 0.0, -0.028)),
        material=painted_metal,
        name="lower_spine",
    )
    clamp_body.visual(
        Box((0.006, 0.060, 0.132)),
        origin=Origin(xyz=(-0.004, 0.0, 0.004)),
        material=polymer,
        name="back_plate",
    )
    clamp_body.visual(
        Box((0.020, 0.080, 0.012)),
        origin=Origin(xyz=(0.002, 0.0, -0.060)),
        material=polymer,
        name="shelf",
    )
    clamp_body.visual(
        Box((0.010, 0.052, 0.002)),
        origin=Origin(xyz=(0.004, 0.0, -0.054)),
        material=elastomer,
        name="shelf_pad",
    )
    clamp_body.visual(
        Box((0.014, 0.006, 0.084)),
        origin=Origin(xyz=(0.006, -0.040, -0.011)),
        material=polymer,
        name="fixed_jaw",
    )
    clamp_body.visual(
        Box((0.008, 0.014, 0.056)),
        origin=Origin(xyz=(0.003, -0.033, -0.044)),
        material=polymer,
        name="fixed_jaw_web",
    )
    clamp_body.visual(
        Box((0.002, 0.003, 0.056)),
        origin=Origin(xyz=(0.014, -0.0355, -0.011)),
        material=elastomer,
        name="fixed_jaw_pad",
    )
    clamp_body.visual(
        Box((0.002, 0.016, 0.104)),
        origin=Origin(xyz=(-0.011, 0.040, 0.0)),
        material=painted_metal,
        name="slider_rail",
    )
    clamp_body.visual(
        Box((0.006, 0.012, 0.060)),
        origin=Origin(xyz=(-0.008, 0.034, -0.010)),
        material=painted_metal,
        name="rail_mount",
    )
    clamp_body.visual(
        Box((0.002, 0.040, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=elastomer,
        name="upper_back_pad",
    )
    clamp_body.visual(
        Box((0.002, 0.040, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=elastomer,
        name="lower_back_pad",
    )
    clamp_body.inertial = Inertial.from_geometry(
        Box((0.10, 0.10, 0.20)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
    )

    model.articulation(
        "head_plate_to_clamp_body",
        ArticulationType.FIXED,
        parent=head_plate,
        child=clamp_body,
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
    )

    sliding_jaw = model.part("sliding_jaw")
    sliding_jaw.visual(
        Box((0.004, 0.012, 0.104)),
        origin=Origin(xyz=(-0.008, 0.0, 0.0)),
        material=painted_metal,
        name="carriage",
    )
    sliding_jaw.visual(
        Box((0.012, 0.006, 0.084)),
        origin=Origin(xyz=(0.0, 0.0, -0.011)),
        material=polymer,
        name="moving_jaw",
    )
    sliding_jaw.visual(
        Box((0.002, 0.003, 0.056)),
        origin=Origin(xyz=(0.007, -0.0015, -0.011)),
        material=elastomer,
        name="moving_jaw_pad",
    )
    sliding_jaw.inertial = Inertial.from_geometry(
        Box((0.03, 0.03, 0.10)),
        mass=0.06,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    model.articulation(
        "clamp_body_to_sliding_jaw",
        ArticulationType.PRISMATIC,
        parent=clamp_body,
        child=sliding_jaw,
        origin=Origin(xyz=(0.0, 0.040, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.04,
            lower=0.0,
            upper=0.024,
        ),
    )

    device = model.part("device")
    device_shell = section_loft(
        [
            _yz_rounded_section(0.072, 0.156, 0.009, -0.0045),
            _yz_rounded_section(0.074, 0.158, 0.011, 0.0),
            _yz_rounded_section(0.072, 0.156, 0.009, 0.0045),
        ]
    )
    device.visual(
        mesh_from_geometry(device_shell, "device_shell"),
        material=satin_metal,
        name="device_shell",
    )
    device.visual(
        Box((0.001, 0.068, 0.148)),
        origin=Origin(xyz=(0.004, 0.0, 0.0)),
        material=glass,
        name="front_glass",
    )
    device.visual(
        Box((0.001, 0.060, 0.128)),
        origin=Origin(xyz=(-0.0055, 0.0, -0.004)),
        material=polymer,
        name="rear_flat",
    )
    device.visual(
        Box((0.003, 0.026, 0.046)),
        origin=Origin(xyz=(-0.0055, 0.017, 0.046)),
        material=polymer,
        name="camera_island",
    )
    for idx, z_pos in enumerate((0.060, 0.046, 0.032), start=1):
        device.visual(
            Cylinder(radius=0.005, length=0.002),
            origin=Origin(
                xyz=(-0.0065, 0.017, z_pos),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=glass,
            name=f"lens_{idx}",
        )
    device.inertial = Inertial.from_geometry(
        Box((0.009, 0.074, 0.158)),
        mass=0.24,
        origin=Origin(),
    )

    model.articulation(
        "clamp_body_to_device",
        ArticulationType.FIXED,
        parent=clamp_body,
        child=device,
        origin=Origin(xyz=(0.007, 0.0, 0.026)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tripod_base = object_model.get_part("tripod_base")
    front_leg = object_model.get_part("front_leg")
    left_leg = object_model.get_part("left_leg")
    right_leg = object_model.get_part("right_leg")
    pan_stage = object_model.get_part("pan_stage")
    tilt_head = object_model.get_part("tilt_head")
    head_plate = object_model.get_part("head_plate")
    clamp_body = object_model.get_part("clamp_body")
    sliding_jaw = object_model.get_part("sliding_jaw")
    device = object_model.get_part("device")

    pan_joint = object_model.get_articulation("tripod_base_to_pan_stage")
    tilt_joint = object_model.get_articulation("pan_stage_to_tilt_head")
    clamp_joint = object_model.get_articulation("clamp_body_to_sliding_jaw")

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
        "pan_axis_is_vertical",
        pan_joint.axis == (0.0, 0.0, 1.0),
        details=f"expected vertical pan axis, got {pan_joint.axis}",
    )
    ctx.check(
        "tilt_axis_is_lateral",
        tilt_joint.axis == (0.0, 1.0, 0.0),
        details=f"expected lateral tilt axis, got {tilt_joint.axis}",
    )
    ctx.check(
        "clamp_axis_slides_sideways",
        clamp_joint.axis == (0.0, 1.0, 0.0),
        details=f"expected sideways clamp axis, got {clamp_joint.axis}",
    )

    ctx.expect_contact(front_leg, tripod_base, contact_tol=1e-4, name="front_leg_mounted")
    ctx.expect_contact(left_leg, tripod_base, contact_tol=1e-4, name="left_leg_mounted")
    ctx.expect_contact(right_leg, tripod_base, contact_tol=1e-4, name="right_leg_mounted")
    ctx.expect_contact(pan_stage, tripod_base, contact_tol=1e-4, name="pan_stage_seated")
    ctx.expect_contact(tilt_head, pan_stage, contact_tol=1e-4, name="tilt_head_supported")
    ctx.expect_contact(head_plate, tilt_head, contact_tol=1e-4, name="head_plate_supported")
    ctx.expect_contact(clamp_body, head_plate, contact_tol=1e-4, name="bracket_attached_to_plate")
    ctx.expect_contact(device, clamp_body, contact_tol=1e-4, name="device_supported_in_bracket")
    ctx.expect_contact(sliding_jaw, clamp_body, contact_tol=1e-4, name="slider_guided_by_bracket")

    with ctx.pose({clamp_joint: 0.0}):
        ctx.expect_contact(
            sliding_jaw,
            device,
            contact_tol=1e-4,
            name="closed_clamp_contacts_device",
        )

    with ctx.pose({clamp_joint: 0.02}):
        ctx.expect_gap(
            sliding_jaw,
            device,
            axis="y",
            min_gap=0.016,
            name="open_clamp_clears_device",
        )

    rest_pos = ctx.part_world_position(device)
    with ctx.pose({pan_joint: math.radians(45.0)}):
        panned_pos = ctx.part_world_position(device)
    pan_moves_device = (
        rest_pos is not None
        and panned_pos is not None
        and math.hypot(
            panned_pos[0] - rest_pos[0],
            panned_pos[1] - rest_pos[1],
        )
        > 0.05
        and abs(panned_pos[2] - rest_pos[2]) < 1e-4
    )
    ctx.check(
        "pan_motion_moves_device_horizontally",
        pan_moves_device,
        details=f"rest={rest_pos}, panned={panned_pos}",
    )

    with ctx.pose({tilt_joint: math.radians(35.0)}):
        tilted_pos = ctx.part_world_position(device)
    tilt_moves_device = (
        rest_pos is not None
        and tilted_pos is not None
        and abs(tilted_pos[2] - rest_pos[2]) > 0.05
    )
    ctx.check(
        "tilt_motion_repositions_device",
        tilt_moves_device,
        details=f"rest={rest_pos}, tilted={tilted_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
