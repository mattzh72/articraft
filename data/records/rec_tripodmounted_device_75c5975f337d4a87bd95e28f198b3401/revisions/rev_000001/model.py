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
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_desktop_tripod_mount")

    anodized = model.material("anodized", rgba=(0.36, 0.38, 0.41, 1.0))
    black = model.material("black", rgba=(0.12, 0.12, 0.13, 1.0))
    rubber = model.material("rubber", rgba=(0.09, 0.09, 0.09, 1.0))
    pad = model.material("pad", rgba=(0.20, 0.20, 0.22, 1.0))

    hinge_radius = 0.018
    hinge_z = 0.086
    leg_length = 0.125
    deploy_pitch = 0.78
    leg_fold_upper = 1.05

    def xy_from_polar(radius: float, angle: float, tangential: float = 0.0) -> tuple[float, float]:
        ca = math.cos(angle)
        sa = math.sin(angle)
        return (radius * ca - tangential * sa, radius * sa + tangential * ca)

    tripod_body = model.part("tripod_body")
    tripod_body.visual(
        Cylinder(radius=0.018, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, hinge_z)),
        material=black,
        name="hub_shell",
    )
    tripod_body.visual(
        Cylinder(radius=0.010, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.122)),
        material=anodized,
        name="center_column",
    )
    tripod_body.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.151)),
        material=black,
        name="top_plate",
    )
    tripod_body.inertial = Inertial.from_geometry(
        Box((0.090, 0.090, 0.170)),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
    )

    def add_leg(name: str) -> None:
        root_length = 0.004
        connector_length = 0.006
        beam_start = root_length + connector_length
        leg = model.part(name)
        leg.visual(
            Box((root_length, 0.014, 0.014)),
            origin=Origin(xyz=(0.5 * root_length, 0.0, 0.0)),
            material=black,
            name="root_knuckle",
        )
        leg.visual(
            Box((connector_length, 0.010, 0.008)),
            origin=Origin(
                xyz=(
                    (root_length + 0.5 * connector_length) * math.cos(deploy_pitch),
                    0.0,
                    -(root_length + 0.5 * connector_length) * math.sin(deploy_pitch),
                ),
                rpy=(0.0, deploy_pitch, 0.0),
            ),
            material=black,
            name="leg_connector",
        )
        leg.visual(
            Box((leg_length, 0.016, 0.010)),
            origin=Origin(
                xyz=(
                    (beam_start + 0.5 * leg_length) * math.cos(deploy_pitch),
                    0.0,
                    -(beam_start + 0.5 * leg_length) * math.sin(deploy_pitch),
                ),
                rpy=(0.0, deploy_pitch, 0.0),
            ),
            material=anodized,
            name="leg_beam",
        )
        foot_run = beam_start + leg_length + 0.004
        leg.visual(
            Box((0.020, 0.022, 0.006)),
            origin=Origin(
                xyz=(
                    foot_run * math.cos(deploy_pitch),
                    0.0,
                    -foot_run * math.sin(deploy_pitch) - 0.002,
                ),
                rpy=(0.0, deploy_pitch, 0.0),
            ),
            material=rubber,
            name="foot_pad",
        )
        leg.inertial = Inertial.from_geometry(
            Box((0.160, 0.024, 0.024)),
            mass=0.11,
            origin=Origin(xyz=(0.060, 0.0, -0.050)),
        )

    add_leg("front_leg")
    add_leg("left_leg")
    add_leg("right_leg")

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=black,
        name="pan_disk",
    )
    pan_head.visual(
        Box((0.016, 0.024, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=black,
        name="pan_riser",
    )
    pan_head.visual(
        Box((0.010, 0.006, 0.024)),
        origin=Origin(xyz=(0.0, -0.012, 0.038)),
        material=black,
        name="left_tilt_cheek",
    )
    pan_head.visual(
        Box((0.010, 0.006, 0.024)),
        origin=Origin(xyz=(0.0, 0.012, 0.038)),
        material=black,
        name="right_tilt_cheek",
    )
    pan_head.inertial = Inertial.from_geometry(
        Box((0.050, 0.040, 0.050)),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )

    tilt_carriage = model.part("tilt_carriage")
    tilt_carriage.visual(
        Cylinder(radius=0.005, length=0.018),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="tilt_trunnion",
    )
    tilt_carriage.visual(
        Box((0.008, 0.010, 0.044)),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=black,
        name="tilt_upright",
    )
    tilt_carriage.visual(
        Box((0.034, 0.028, 0.008)),
        origin=Origin(xyz=(0.010, 0.0, 0.046)),
        material=anodized,
        name="head_plate",
    )
    tilt_carriage.inertial = Inertial.from_geometry(
        Box((0.060, 0.030, 0.060)),
        mass=0.10,
        origin=Origin(xyz=(0.008, 0.0, 0.030)),
    )

    device_bracket = model.part("device_bracket")
    device_bracket.visual(
        Box((0.012, 0.022, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=black,
        name="mount_shoe",
    )
    device_bracket.visual(
        Box((0.010, 0.022, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=black,
        name="shoe_post",
    )
    device_bracket.visual(
        Box((0.010, 0.022, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        material=black,
        name="center_spine",
    )
    device_bracket.visual(
        Box((0.004, 0.060, 0.074)),
        origin=Origin(xyz=(0.003, 0.0, 0.055)),
        material=black,
        name="back_plate",
    )
    device_bracket.visual(
        Box((0.006, 0.014, 0.064)),
        origin=Origin(xyz=(0.008, 0.0, 0.056)),
        material=black,
        name="guide_track",
    )
    device_bracket.visual(
        Box((0.016, 0.018, 0.012)),
        origin=Origin(xyz=(0.008, 0.0, 0.012)),
        material=black,
        name="lower_bridge",
    )
    device_bracket.visual(
        Box((0.024, 0.066, 0.010)),
        origin=Origin(xyz=(0.020, 0.0, 0.010)),
        material=black,
        name="lower_jaw",
    )
    device_bracket.visual(
        Box((0.002, 0.048, 0.056)),
        origin=Origin(xyz=(0.005, 0.0, 0.046)),
        material=pad,
        name="back_pad",
    )
    device_bracket.visual(
        Box((0.002, 0.054, 0.004)),
        origin=Origin(xyz=(0.032, 0.0, 0.011)),
        material=pad,
        name="lower_pad",
    )
    device_bracket.inertial = Inertial.from_geometry(
        Box((0.040, 0.080, 0.105)),
        mass=0.15,
        origin=Origin(xyz=(0.010, 0.0, 0.052)),
    )

    clamp_slider = model.part("clamp_slider")
    clamp_slider.visual(
        Box((0.006, 0.014, 0.022)),
        origin=Origin(xyz=(0.008, 0.0, 0.012)),
        material=black,
        name="carriage_block",
    )
    clamp_slider.visual(
        Box((0.018, 0.014, 0.008)),
        origin=Origin(xyz=(0.017, 0.0, 0.024)),
        material=black,
        name="slider_bridge",
    )
    clamp_slider.visual(
        Box((0.010, 0.018, 0.040)),
        origin=Origin(xyz=(0.025, 0.0, 0.048)),
        material=black,
        name="drop_link",
    )
    clamp_slider.visual(
        Box((0.008, 0.074, 0.010)),
        origin=Origin(xyz=(0.024, 0.0, 0.064)),
        material=black,
        name="front_bridge",
    )
    clamp_slider.visual(
        Box((0.028, 0.066, 0.010)),
        origin=Origin(xyz=(0.032, 0.0, 0.073)),
        material=black,
        name="upper_jaw",
    )
    clamp_slider.visual(
        Box((0.002, 0.054, 0.004)),
        origin=Origin(xyz=(0.046, 0.0, 0.074)),
        material=pad,
        name="upper_pad",
    )
    clamp_slider.inertial = Inertial.from_geometry(
        Box((0.060, 0.100, 0.100)),
        mass=0.05,
        origin=Origin(xyz=(0.000, 0.0, 0.060)),
    )

    model.articulation(
        "body_to_front_leg",
        ArticulationType.REVOLUTE,
        parent=tripod_body,
        child="front_leg",
        origin=Origin(xyz=(hinge_radius, 0.0, hinge_z), rpy=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=leg_fold_upper,
        ),
    )
    model.articulation(
        "body_to_left_leg",
        ArticulationType.REVOLUTE,
        parent=tripod_body,
        child="left_leg",
        origin=Origin(xyz=xy_from_polar(hinge_radius, 2.0 * math.pi / 3.0) + (hinge_z,), rpy=(0.0, 0.0, 2.0 * math.pi / 3.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=leg_fold_upper,
        ),
    )
    model.articulation(
        "body_to_right_leg",
        ArticulationType.REVOLUTE,
        parent=tripod_body,
        child="right_leg",
        origin=Origin(xyz=xy_from_polar(hinge_radius, -2.0 * math.pi / 3.0) + (hinge_z,), rpy=(0.0, 0.0, -2.0 * math.pi / 3.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=leg_fold_upper,
        ),
    )
    model.articulation(
        "body_to_pan_head",
        ArticulationType.CONTINUOUS,
        parent=tripod_body,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 0.154)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0),
    )
    model.articulation(
        "pan_head_to_tilt_carriage",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=tilt_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=1.5,
            lower=-0.65,
            upper=0.85,
        ),
    )
    model.articulation(
        "tilt_carriage_to_device_bracket",
        ArticulationType.FIXED,
        parent=tilt_carriage,
        child=device_bracket,
        origin=Origin(xyz=(0.010, 0.0, 0.050)),
    )
    model.articulation(
        "device_bracket_to_clamp_slider",
        ArticulationType.PRISMATIC,
        parent=device_bracket,
        child=clamp_slider,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.08,
            lower=0.0,
            upper=0.040,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tripod_body = object_model.get_part("tripod_body")
    front_leg = object_model.get_part("front_leg")
    left_leg = object_model.get_part("left_leg")
    right_leg = object_model.get_part("right_leg")
    pan_head = object_model.get_part("pan_head")
    tilt_carriage = object_model.get_part("tilt_carriage")
    device_bracket = object_model.get_part("device_bracket")
    clamp_slider = object_model.get_part("clamp_slider")

    front_hinge = object_model.get_articulation("body_to_front_leg")
    left_hinge = object_model.get_articulation("body_to_left_leg")
    right_hinge = object_model.get_articulation("body_to_right_leg")
    pan_joint = object_model.get_articulation("body_to_pan_head")
    tilt_joint = object_model.get_articulation("pan_head_to_tilt_carriage")
    clamp_joint = object_model.get_articulation("device_bracket_to_clamp_slider")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_overlap(
        front_leg,
        tripod_body,
        reason="compact tripod leg root is modeled as a captured pivot knuckle inside the hub socket",
    )
    ctx.allow_overlap(
        left_leg,
        tripod_body,
        reason="compact tripod leg root is modeled as a captured pivot knuckle inside the hub socket",
    )
    ctx.allow_overlap(
        right_leg,
        tripod_body,
        reason="compact tripod leg root is modeled as a captured pivot knuckle inside the hub socket",
    )
    ctx.allow_overlap(
        device_bracket,
        tilt_carriage,
        reason="the device bracket shoe is represented as a seated mount nested onto the tilt head plate",
    )
    ctx.allow_overlap(
        clamp_slider,
        device_bracket,
        reason="the clamp slider uses a simplified captured guide channel around the fixed bracket track",
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

    ctx.expect_contact(front_leg, tripod_body, contact_tol=0.0005, name="front_leg_mounts_on_hub")
    ctx.expect_contact(left_leg, tripod_body, contact_tol=0.0005, name="left_leg_mounts_on_hub")
    ctx.expect_contact(right_leg, tripod_body, contact_tol=0.0005, name="right_leg_mounts_on_hub")
    ctx.expect_contact(
        pan_head,
        tripod_body,
        elem_a="pan_disk",
        elem_b="top_plate",
        contact_tol=0.0005,
        name="pan_head_seats_on_top_plate",
    )
    ctx.expect_contact(
        tilt_carriage,
        pan_head,
        contact_tol=0.0005,
        name="tilt_carriage_supported_by_pan_head",
    )
    ctx.expect_contact(
        device_bracket,
        tilt_carriage,
        elem_a="mount_shoe",
        elem_b="head_plate",
        contact_tol=0.0005,
        name="device_bracket_bolts_to_head_plate",
    )
    ctx.expect_gap(
        device_bracket,
        clamp_slider,
        axis="z",
        positive_elem="guide_track",
        negative_elem="carriage_block",
        min_gap=0.0,
        max_gap=0.0015,
        name="slider_is_guided_by_guide_track",
    )
    ctx.expect_gap(
        clamp_slider,
        device_bracket,
        axis="z",
        positive_elem="upper_jaw",
        negative_elem="lower_jaw",
        min_gap=0.052,
        max_gap=0.058,
        name="closed_clamp_gap_fits_phone_body",
    )
    ctx.expect_overlap(
        clamp_slider,
        device_bracket,
        axes="y",
        elem_a="upper_jaw",
        elem_b="lower_jaw",
        min_overlap=0.060,
        name="upper_and_lower_jaws_remain_aligned",
    )

    with ctx.pose({clamp_joint: 0.040}):
        ctx.expect_gap(
            clamp_slider,
            device_bracket,
            axis="z",
            positive_elem="upper_jaw",
            negative_elem="lower_jaw",
            min_gap=0.092,
            max_gap=0.098,
            name="opened_clamp_gap_clears_larger_device",
        )

    with ctx.pose({tilt_joint: 0.75, clamp_joint: 0.015, pan_joint: 0.4}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlaps_in_forward_tilt_pose")

    with ctx.pose({tilt_joint: -0.55, clamp_joint: 0.015, pan_joint: -0.35}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlaps_in_back_tilt_pose")

    with ctx.pose({front_hinge: 1.00, left_hinge: 1.00, right_hinge: 1.00}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlaps_with_legs_folded_for_stow")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
