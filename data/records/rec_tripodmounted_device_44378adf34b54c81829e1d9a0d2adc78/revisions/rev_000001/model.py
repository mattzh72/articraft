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
    Sphere,
    TestContext,
    TestReport,
)


def _segment_origin(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> tuple[Origin, float]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    midpoint = (
        (start[0] + end[0]) * 0.5,
        (start[1] + end[1]) * 0.5,
        (start[2] + end[2]) * 0.5,
    )
    return Origin(xyz=midpoint, rpy=(0.0, pitch, yaw)), length


def _polar_point(radius: float, angle: float, z: float) -> tuple[float, float, float]:
    return (radius * math.cos(angle), radius * math.sin(angle), z)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tripod_mounted_device")

    aluminum = model.material("aluminum", rgba=(0.72, 0.74, 0.77, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.16, 0.17, 0.19, 1.0))
    mid_plastic = model.material("mid_plastic", rgba=(0.28, 0.30, 0.33, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))
    pad_gray = model.material("pad_gray", rgba=(0.35, 0.37, 0.39, 1.0))

    tripod_frame = model.part("tripod_frame")
    tripod_frame.visual(
        Cylinder(radius=0.055, length=0.042),
        origin=Origin(xyz=(0.0, 0.0, 0.302)),
        material=mid_plastic,
        name="hub_shell",
    )
    tripod_frame.visual(
        Cylinder(radius=0.026, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.338)),
        material=mid_plastic,
        name="hub_collar",
    )
    tripod_frame.visual(
        Cylinder(radius=0.014, length=0.366),
        origin=Origin(xyz=(0.0, 0.0, 0.535)),
        material=aluminum,
        name="center_column",
    )

    leg_top_radius = 0.035
    foot_radius = 0.365
    leg_top_z = 0.304
    foot_top_z = 0.016
    for index in range(3):
        angle = index * (math.tau / 3.0)
        leg_top = _polar_point(leg_top_radius, angle, leg_top_z)
        foot_top = _polar_point(foot_radius, angle, foot_top_z)
        leg_origin, leg_length = _segment_origin(leg_top, foot_top)
        tripod_frame.visual(
            Cylinder(radius=0.011, length=leg_length),
            origin=leg_origin,
            material=aluminum,
            name=f"leg_{index}",
        )
        tripod_frame.visual(
            Cylinder(radius=0.018, length=0.016),
            origin=Origin(xyz=(foot_top[0], foot_top[1], 0.008)),
            material=rubber,
            name=f"foot_{index}",
        )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.040, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=dark_plastic,
        name="pan_base",
    )
    pan_head.visual(
        Box((0.030, 0.076, 0.038)),
        origin=Origin(xyz=(-0.004, 0.0, 0.052)),
        material=mid_plastic,
        name="yoke_body",
    )
    pan_head.visual(
        Box((0.014, 0.012, 0.060)),
        origin=Origin(xyz=(0.0, 0.038, 0.087)),
        material=mid_plastic,
        name="left_yoke_cheek",
    )
    pan_head.visual(
        Box((0.014, 0.012, 0.060)),
        origin=Origin(xyz=(0.0, -0.038, 0.087)),
        material=mid_plastic,
        name="right_yoke_cheek",
    )
    pan_head.visual(
        Cylinder(radius=0.010, length=0.018),
        origin=Origin(xyz=(-0.014, 0.0, 0.026), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="handle_boss",
    )
    pan_head.visual(
        Cylinder(radius=0.0045, length=0.180),
        origin=Origin(xyz=(-0.104, 0.0, 0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="pan_handle",
    )
    pan_head.visual(
        Sphere(radius=0.011),
        origin=Origin(xyz=(-0.194, 0.0, 0.020)),
        material=dark_plastic,
        name="handle_grip",
    )
    pan_head.visual(
        Cylinder(radius=0.009, length=0.016),
        origin=Origin(xyz=(-0.010, 0.046, 0.022), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="pan_lock_knob",
    )
    pan_head.visual(
        Cylinder(radius=0.007, length=0.020),
        origin=Origin(xyz=(0.004, -0.054, 0.084), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="tilt_lock_knob",
    )

    head_plate = model.part("head_plate")
    head_plate.visual(
        Cylinder(radius=0.007, length=0.064),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="trunnion_shaft",
    )
    head_plate.visual(
        Box((0.026, 0.036, 0.014)),
        origin=Origin(xyz=(0.013, 0.0, 0.007)),
        material=mid_plastic,
        name="tilt_carrier",
    )
    head_plate.visual(
        Box((0.082, 0.050, 0.008)),
        origin=Origin(xyz=(0.046, 0.0, 0.014)),
        material=mid_plastic,
        name="plate_surface",
    )
    head_plate.visual(
        Box((0.066, 0.036, 0.002)),
        origin=Origin(xyz=(0.046, 0.0, 0.019)),
        material=rubber,
        name="plate_pad",
    )

    bracket_body = model.part("bracket_body")
    bracket_body.visual(
        Box((0.024, 0.032, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=dark_plastic,
        name="mount_pad",
    )
    bracket_body.visual(
        Box((0.010, 0.050, 0.132)),
        origin=Origin(xyz=(0.005, 0.0, 0.074)),
        material=dark_plastic,
        name="back_spine",
    )
    bracket_body.visual(
        Box((0.040, 0.050, 0.016)),
        origin=Origin(xyz=(0.020, 0.0, 0.012)),
        material=dark_plastic,
        name="lower_jaw",
    )
    bracket_body.visual(
        Box((0.004, 0.006, 0.086)),
        origin=Origin(xyz=(0.010, 0.026, 0.076)),
        material=mid_plastic,
        name="left_guide_rail",
    )
    bracket_body.visual(
        Box((0.004, 0.006, 0.086)),
        origin=Origin(xyz=(0.010, -0.026, 0.076)),
        material=mid_plastic,
        name="right_guide_rail",
    )
    bracket_body.visual(
        Box((0.004, 0.034, 0.002)),
        origin=Origin(xyz=(0.030, 0.0, 0.021)),
        material=pad_gray,
        name="lower_pad",
    )

    clamp_slider = model.part("clamp_slider")
    clamp_slider.visual(
        Box((0.006, 0.034, 0.046)),
        origin=Origin(xyz=(0.003, 0.0, 0.023)),
        material=mid_plastic,
        name="slider_carriage",
    )
    clamp_slider.visual(
        Box((0.040, 0.050, 0.016)),
        origin=Origin(xyz=(0.020, 0.0, 0.008)),
        material=dark_plastic,
        name="upper_jaw",
    )
    clamp_slider.visual(
        Box((0.004, 0.034, 0.002)),
        origin=Origin(xyz=(0.030, 0.0, -0.001)),
        material=pad_gray,
        name="upper_pad",
    )
    clamp_slider.visual(
        Cylinder(radius=0.009, length=0.012),
        origin=Origin(xyz=(0.046, 0.0, 0.008), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="thumb_knob",
    )

    model.articulation(
        "tripod_to_pan",
        ArticulationType.CONTINUOUS,
        parent=tripod_frame,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 0.718)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.5),
    )
    model.articulation(
        "pan_to_plate",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=head_plate,
        origin=Origin(xyz=(0.0, 0.0, 0.084)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.2,
            lower=-0.35,
            upper=0.95,
        ),
    )
    model.articulation(
        "plate_to_bracket",
        ArticulationType.FIXED,
        parent=head_plate,
        child=bracket_body,
        origin=Origin(xyz=(0.046, 0.0, 0.018)),
    )
    model.articulation(
        "bracket_to_slider",
        ArticulationType.PRISMATIC,
        parent=bracket_body,
        child=clamp_slider,
        origin=Origin(xyz=(0.010, 0.0, 0.076)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=0.08,
            lower=0.0,
            upper=0.038,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tripod_frame = object_model.get_part("tripod_frame")
    pan_head = object_model.get_part("pan_head")
    head_plate = object_model.get_part("head_plate")
    bracket_body = object_model.get_part("bracket_body")
    clamp_slider = object_model.get_part("clamp_slider")

    pan_joint = object_model.get_articulation("tripod_to_pan")
    tilt_joint = object_model.get_articulation("pan_to_plate")
    clamp_joint = object_model.get_articulation("bracket_to_slider")

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
        "pan_joint_is_vertical_continuous",
        pan_joint.articulation_type == ArticulationType.CONTINUOUS and pan_joint.axis == (0.0, 0.0, 1.0),
        details=f"expected continuous vertical pan joint, got type={pan_joint.articulation_type} axis={pan_joint.axis}",
    )
    ctx.check(
        "tilt_joint_opens_upward",
        tilt_joint.articulation_type == ArticulationType.REVOLUTE and tilt_joint.axis == (0.0, -1.0, 0.0),
        details=f"expected upward-opening tilt axis (0,-1,0), got type={tilt_joint.articulation_type} axis={tilt_joint.axis}",
    )
    ctx.check(
        "clamp_joint_slides_upward",
        clamp_joint.articulation_type == ArticulationType.PRISMATIC and clamp_joint.axis == (0.0, 0.0, 1.0),
        details=f"expected upward sliding clamp axis (0,0,1), got type={clamp_joint.articulation_type} axis={clamp_joint.axis}",
    )

    ctx.expect_contact(pan_head, tripod_frame, name="pan_head_seats_on_center_column")
    ctx.expect_contact(
        head_plate,
        pan_head,
        elem_a="trunnion_shaft",
        name="head_plate_trunnion_supported_by_yoke",
    )
    ctx.expect_contact(
        bracket_body,
        head_plate,
        elem_a="mount_pad",
        elem_b="plate_surface",
        name="device_bracket_bolts_to_head_plate",
    )
    ctx.expect_contact(clamp_slider, bracket_body, name="slider_carriage_guided_by_bracket")

    with ctx.pose({clamp_joint: 0.0}):
        ctx.expect_gap(
            clamp_slider,
            bracket_body,
            axis="z",
            positive_elem="upper_pad",
            negative_elem="lower_pad",
            min_gap=0.051,
            max_gap=0.056,
            name="closed_clamp_gap_fits_phone_width",
        )

    with ctx.pose({clamp_joint: 0.038}):
        ctx.expect_gap(
            clamp_slider,
            bracket_body,
            axis="z",
            positive_elem="upper_pad",
            negative_elem="lower_pad",
            min_gap=0.089,
            max_gap=0.094,
            name="open_clamp_gap_expands_upward",
        )

    with ctx.pose({tilt_joint: 0.0}):
        closed_plate_aabb = ctx.part_element_world_aabb(head_plate, elem="plate_surface")
    with ctx.pose({tilt_joint: 0.95}):
        raised_plate_aabb = ctx.part_element_world_aabb(head_plate, elem="plate_surface")
    if closed_plate_aabb is not None and raised_plate_aabb is not None:
        ctx.check(
            "positive_tilt_raises_head_plate",
            raised_plate_aabb[1][2] > closed_plate_aabb[1][2] + 0.03,
            details=(
                f"expected positive tilt to raise the plate; "
                f"closed max z={closed_plate_aabb[1][2]:.4f}, raised max z={raised_plate_aabb[1][2]:.4f}"
            ),
        )
    else:
        ctx.fail("positive_tilt_raises_head_plate", "could not measure head plate AABBs")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
