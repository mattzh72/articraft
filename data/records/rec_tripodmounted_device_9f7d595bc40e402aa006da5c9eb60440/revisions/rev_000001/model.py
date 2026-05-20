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
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def _lerp_point(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    t: float,
) -> tuple[float, float, float]:
    return (
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        a[2] + (b[2] - a[2]) * t,
    )


def _segment_origin_and_length(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> tuple[Origin, float]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 1e-9:
        raise ValueError("segment length must be positive")
    yaw = math.atan2(dy, dx)
    pitch = math.acos(max(-1.0, min(1.0, dz / length)))
    return (
        Origin(
            xyz=(
                (start[0] + end[0]) * 0.5,
                (start[1] + end[1]) * 0.5,
                (start[2] + end[2]) * 0.5,
            ),
            rpy=(0.0, pitch, yaw),
        ),
        length,
    )


def _add_tube(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    *,
    material,
    name: str,
) -> None:
    origin, length = _segment_origin_and_length(start, end)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=origin,
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="weatherproof_tripod_mounted_device")

    anodized_dark = model.material("anodized_dark", rgba=(0.20, 0.22, 0.24, 1.0))
    aluminum_gray = model.material("aluminum_gray", rgba=(0.56, 0.59, 0.62, 1.0))
    stainless = model.material("stainless", rgba=(0.77, 0.79, 0.81, 1.0))
    shell_offwhite = model.material("shell_offwhite", rgba=(0.86, 0.87, 0.84, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.10, 0.11, 0.12, 1.0))
    seal_dark = model.material("seal_dark", rgba=(0.14, 0.15, 0.16, 1.0))
    glass_smoke = model.material("glass_smoke", rgba=(0.18, 0.26, 0.30, 0.50))

    tripod_frame = model.part("tripod_frame")
    tripod_frame.visual(
        Box((0.17, 0.17, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=aluminum_gray,
        name="head_plate",
    )
    tripod_frame.visual(
        Cylinder(radius=0.070, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, -0.045)),
        material=anodized_dark,
        name="apex_casting",
    )
    tripod_frame.visual(
        Cylinder(radius=0.040, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, -0.720)),
        material=anodized_dark,
        name="spreader_hub",
    )

    leg_attach_radius = 0.074
    foot_radius = 0.530
    leg_top_z = -0.060
    foot_z = -1.280
    leg_split = 0.56
    for index in range(3):
        angle = (2.0 * math.pi * index) / 3.0 + math.pi / 6.0
        attach = (
            leg_attach_radius * math.cos(angle),
            leg_attach_radius * math.sin(angle),
            leg_top_z,
        )
        foot = (
            foot_radius * math.cos(angle),
            foot_radius * math.sin(angle),
            foot_z,
        )
        split = _lerp_point(attach, foot, leg_split)
        brace_target = _lerp_point(attach, foot, 0.54)
        collar_a = _lerp_point(attach, foot, leg_split - 0.015)
        collar_b = _lerp_point(attach, foot, leg_split + 0.015)

        _add_tube(
            tripod_frame,
            attach,
            split,
            0.020,
            material=aluminum_gray,
            name=f"upper_leg_{index}",
        )
        _add_tube(
            tripod_frame,
            split,
            foot,
            0.016,
            material=anodized_dark,
            name=f"lower_leg_{index}",
        )
        _add_tube(
            tripod_frame,
            collar_a,
            collar_b,
            0.026,
            material=stainless,
            name=f"leg_clamp_{index}",
        )
        _add_tube(
            tripod_frame,
            (0.0, 0.0, -0.705),
            brace_target,
            0.009,
            material=aluminum_gray,
            name=f"spreader_brace_{index}",
        )
        tripod_frame.visual(
            Sphere(radius=0.024),
            origin=Origin(xyz=foot),
            material=rubber_black,
            name=f"foot_pad_{index}",
        )
        tripod_frame.visual(
            Cylinder(radius=0.012, length=0.016),
            origin=Origin(
                xyz=(
                    attach[0] * 0.92,
                    attach[1] * 0.92,
                    -0.020,
                )
            ),
            material=stainless,
            name=f"leg_bolt_{index}",
        )
    tripod_frame.inertial = Inertial.from_geometry(
        Box((1.10, 1.10, 1.30)),
        mass=6.8,
        origin=Origin(xyz=(0.0, 0.0, -0.650)),
    )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.055, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=anodized_dark,
        name="lower_bearing",
    )
    pan_head.visual(
        Box((0.022, 0.028, 0.094)),
        origin=Origin(xyz=(-0.030, 0.0, 0.077)),
        material=shell_offwhite,
        name="pan_body",
    )
    pan_head.visual(
        Box((0.056, 0.116, 0.012)),
        origin=Origin(xyz=(-0.015, 0.0, 0.130)),
        material=shell_offwhite,
        name="tilt_bridge",
    )
    pan_head.visual(
        Box((0.010, 0.018, 0.040)),
        origin=Origin(xyz=(-0.010, 0.047, 0.110)),
        material=shell_offwhite,
        name="right_yoke",
    )
    pan_head.visual(
        Box((0.010, 0.018, 0.040)),
        origin=Origin(xyz=(-0.010, -0.047, 0.110)),
        material=shell_offwhite,
        name="left_yoke",
    )
    pan_head.inertial = Inertial.from_geometry(
        Box((0.15, 0.15, 0.17)),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
    )

    tilt_carriage = model.part("tilt_carriage")
    tilt_carriage.visual(
        Cylinder(radius=0.008, length=0.076),
        origin=Origin(xyz=(-0.010, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="trunnion_shaft",
    )
    tilt_carriage.visual(
        Box((0.020, 0.096, 0.016)),
        origin=Origin(xyz=(-0.010, 0.0, 0.0)),
        material=anodized_dark,
        name="pivot_web",
    )
    tilt_carriage.visual(
        Box((0.080, 0.024, 0.060)),
        origin=Origin(xyz=(0.030, 0.0, -0.030)),
        material=anodized_dark,
        name="spine_backbone",
    )
    tilt_carriage.visual(
        Box((0.130, 0.070, 0.010)),
        origin=Origin(xyz=(0.110, 0.0, -0.055)),
        material=anodized_dark,
        name="guide_plate",
    )
    tilt_carriage.visual(
        Box((0.010, 0.080, 0.040)),
        origin=Origin(xyz=(0.065, 0.0, -0.030)),
        material=stainless,
        name="fixed_jaw",
    )
    tilt_carriage.inertial = Inertial.from_geometry(
        Box((0.23, 0.15, 0.12)),
        mass=0.9,
        origin=Origin(xyz=(0.100, 0.0, -0.020)),
    )

    clamp_slider = model.part("clamp_slider")
    clamp_slider.visual(
        Box((0.010, 0.080, 0.040)),
        origin=Origin(xyz=(0.005, 0.0, 0.000)),
        material=stainless,
        name="moving_jaw",
    )
    clamp_slider.visual(
        Box((0.016, 0.020, 0.050)),
        origin=Origin(xyz=(0.008, 0.0, -0.025)),
        material=anodized_dark,
        name="jaw_web",
    )
    clamp_slider.visual(
        Box((0.030, 0.050, 0.010)),
        origin=Origin(xyz=(0.012, 0.0, -0.035)),
        material=anodized_dark,
        name="guide_shoe",
    )
    clamp_slider.visual(
        Cylinder(radius=0.006, length=0.030),
        origin=Origin(xyz=(0.026, 0.0, -0.025), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=anodized_dark,
        name="clamp_screw",
    )
    clamp_slider.visual(
        Cylinder(radius=0.016, length=0.014),
        origin=Origin(xyz=(0.046, 0.0, -0.025), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber_black,
        name="clamp_knob",
    )
    clamp_slider.inertial = Inertial.from_geometry(
        Box((0.085, 0.14, 0.06)),
        mass=0.28,
        origin=Origin(xyz=(0.042, 0.0, 0.000)),
    )

    device_enclosure = model.part("device_enclosure")
    device_enclosure.visual(
        Box((0.040, 0.090, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=stainless,
        name="mount_rail",
    )
    device_enclosure.visual(
        Box((0.030, 0.080, 0.040)),
        origin=Origin(xyz=(-0.005, 0.0, 0.025)),
        material=anodized_dark,
        name="mount_riser",
    )
    device_enclosure.visual(
        Box((0.150, 0.120, 0.006)),
        origin=Origin(xyz=(0.055, 0.0, 0.048)),
        material=shell_offwhite,
        name="bottom_shell",
    )
    device_enclosure.visual(
        Box((0.006, 0.108, 0.078)),
        origin=Origin(xyz=(-0.017, 0.0, 0.090)),
        material=shell_offwhite,
        name="back_wall",
    )
    device_enclosure.visual(
        Box((0.128, 0.006, 0.080)),
        origin=Origin(xyz=(0.047, 0.057, 0.090)),
        material=shell_offwhite,
        name="right_wall",
    )
    device_enclosure.visual(
        Box((0.128, 0.006, 0.080)),
        origin=Origin(xyz=(0.047, -0.057, 0.090)),
        material=shell_offwhite,
        name="left_wall",
    )
    device_enclosure.visual(
        Box((0.110, 0.108, 0.006)),
        origin=Origin(xyz=(0.038, 0.0, 0.131)),
        material=shell_offwhite,
        name="roof",
    )
    device_enclosure.visual(
        Box((0.060, 0.124, 0.006)),
        origin=Origin(xyz=(0.120, 0.0, 0.137)),
        material=shell_offwhite,
        name="hood_top",
    )
    device_enclosure.visual(
        Box((0.056, 0.006, 0.020)),
        origin=Origin(xyz=(0.118, 0.059, 0.124)),
        material=shell_offwhite,
        name="hood_right",
    )
    device_enclosure.visual(
        Box((0.056, 0.006, 0.020)),
        origin=Origin(xyz=(0.118, -0.059, 0.124)),
        material=shell_offwhite,
        name="hood_left",
    )
    device_enclosure.visual(
        Box((0.004, 0.078, 0.092)),
        origin=Origin(xyz=(0.148, 0.0, 0.088)),
        material=glass_smoke,
        name="front_window",
    )
    device_enclosure.visual(
        Box((0.020, 0.108, 0.006)),
        origin=Origin(xyz=(-0.004, 0.0, 0.137)),
        material=shell_offwhite,
        name="rear_drip_lip",
    )
    device_enclosure.visual(
        Cylinder(radius=0.010, length=0.020),
        origin=Origin(xyz=(-0.020, 0.0, 0.005), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber_black,
        name="cable_gland",
    )
    device_enclosure.inertial = Inertial.from_geometry(
        Box((0.20, 0.13, 0.13)),
        mass=1.8,
        origin=Origin(xyz=(0.055, 0.0, 0.055)),
    )

    model.articulation(
        "tripod_to_pan",
        ArticulationType.CONTINUOUS,
        parent=tripod_frame,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.2),
    )
    model.articulation(
        "pan_to_tilt",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=tilt_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.119)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.2,
            lower=math.radians(-20.0),
            upper=math.radians(58.0),
        ),
    )
    model.articulation(
        "tilt_to_clamp",
        ArticulationType.PRISMATIC,
        parent=tilt_carriage,
        child=clamp_slider,
        origin=Origin(xyz=(0.110, 0.0, -0.020)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.05,
            lower=0.0,
            upper=0.030,
        ),
    )
    model.articulation(
        "tilt_to_device",
        ArticulationType.FIXED,
        parent=tilt_carriage,
        child=device_enclosure,
        origin=Origin(xyz=(0.090, 0.0, -0.030)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tripod_frame = object_model.get_part("tripod_frame")
    pan_head = object_model.get_part("pan_head")
    tilt_carriage = object_model.get_part("tilt_carriage")
    clamp_slider = object_model.get_part("clamp_slider")
    device_enclosure = object_model.get_part("device_enclosure")
    pan = object_model.get_articulation("tripod_to_pan")
    tilt = object_model.get_articulation("pan_to_tilt")
    clamp = object_model.get_articulation("tilt_to_clamp")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_overlap(
        pan_head,
        tilt_carriage,
        reason="Protected trunnion runs inside the pan-head yoke shroud with intentionally nested bearing geometry.",
    )
    ctx.allow_overlap(
        clamp_slider,
        tilt_carriage,
        reason="Clamp slider rides in a captured guide channel on the tilt bracket and uses intentionally nested guide geometry.",
    )
    ctx.allow_overlap(
        device_enclosure,
        tilt_carriage,
        reason="Device rail is intentionally captured within the fixed clamp bracket geometry.",
    )

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
        "pan_axis_vertical",
        tuple(round(v, 6) for v in pan.axis) == (0.0, 0.0, 1.0),
        details=f"pan axis was {pan.axis}",
    )
    ctx.check(
        "tilt_axis_pitches_upward",
        tuple(round(v, 6) for v in tilt.axis) == (0.0, -1.0, 0.0),
        details=f"tilt axis was {tilt.axis}",
    )
    ctx.check(
        "clamp_axis_opens_sideways",
        tuple(round(v, 6) for v in clamp.axis) == (1.0, 0.0, 0.0),
        details=f"clamp axis was {clamp.axis}",
    )

    ctx.expect_contact(
        pan_head,
        tripod_frame,
        elem_a="lower_bearing",
        elem_b="head_plate",
        name="pan_head_seated_on_tripod_plate",
    )
    ctx.expect_contact(
        tilt_carriage,
        pan_head,
        elem_a="trunnion_shaft",
        name="tilt_trunnion_supported_by_head",
    )
    ctx.expect_contact(
        device_enclosure,
        tilt_carriage,
        elem_a="mount_rail",
        elem_b="guide_plate",
        name="device_rail_seated_on_clamp_plate",
    )
    ctx.expect_contact(
        device_enclosure,
        tilt_carriage,
        elem_a="mount_rail",
        elem_b="fixed_jaw",
        name="device_rail_captured_by_fixed_jaw",
    )
    ctx.expect_contact(
        clamp_slider,
        tilt_carriage,
        elem_a="guide_shoe",
        elem_b="guide_plate",
        name="slider_supported_by_guide_plate",
    )
    ctx.expect_contact(
        clamp_slider,
        device_enclosure,
        elem_a="moving_jaw",
        elem_b="mount_rail",
        name="closed_clamp_contacts_device_rail",
    )

    with ctx.pose({clamp: 0.030}):
        ctx.expect_contact(
            clamp_slider,
            tilt_carriage,
            elem_a="guide_shoe",
            elem_b="guide_plate",
            name="open_clamp_stays_captured_on_guide_plate",
        )
        ctx.expect_gap(
            clamp_slider,
            device_enclosure,
            axis="x",
            positive_elem="moving_jaw",
            negative_elem="mount_rail",
            min_gap=0.028,
            max_gap=0.032,
            name="open_clamp_creates_service_gap",
        )

    with ctx.pose({tilt: math.radians(30.0)}):
        tilted_pos = ctx.part_world_position(device_enclosure)
    with ctx.pose({tilt: 0.0}):
        level_pos = ctx.part_world_position(device_enclosure)
    ctx.check(
        "tilt_motion_raises_device",
        tilted_pos is not None
        and level_pos is not None
        and tilted_pos[2] > level_pos[2] + 0.040,
        details=f"level={level_pos}, tilted={tilted_pos}",
    )

    with ctx.pose({pan: 0.0}):
        pan_zero_pos = ctx.part_world_position(device_enclosure)
    with ctx.pose({pan: 1.0}):
        pan_rotated_pos = ctx.part_world_position(device_enclosure)
    ctx.check(
        "pan_motion_swings_device_around_vertical_axis",
        pan_zero_pos is not None
        and pan_rotated_pos is not None
        and abs(pan_rotated_pos[1] - pan_zero_pos[1]) > 0.060,
        details=f"pan0={pan_zero_pos}, pan1={pan_rotated_pos}",
    )

    with ctx.pose({pan: 0.8, tilt: math.radians(25.0), clamp: 0.024}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlaps_in_open_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
