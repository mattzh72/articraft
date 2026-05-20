from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_X = 0.084
BODY_Y = 0.056
BODY_Z = 0.054

BRIDGE_Y = 0.012
BRIDGE_CENTER_Y = 0.034

RAIL_X = 0.292
RAIL_Y = 0.010
RAIL_Z = 0.006
RAIL_CENTER_Y = 0.045
RAIL_Z_OFFSET = 0.017

RUNNER_X = 0.032
RUNNER_Y = 0.006
RUNNER_Z = 0.010
RUNNER_CENTER_Y = 0.053

SPINE_X = 0.028
SPINE_Y = 0.010
SPINE_Z = 0.056
SPINE_CENTER_Y = 0.058

STOP_X = 0.126
STOP_SIZE = (0.010, 0.012, 0.040)

LEFT_JAW_X = -0.102
RIGHT_JAW_X = 0.102
JAW_TRAVEL = 0.032


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _merge(*solids: cq.Workplane) -> cq.Workplane:
    result = solids[0]
    for solid in solids[1:]:
        result = result.union(solid)
    return result


def _body_shell_shape() -> cq.Workplane:
    shell = cq.Workplane("XY").box(BODY_X, BODY_Y, BODY_Z).edges("|Z").fillet(0.0035)
    rear_mount = _box((0.036, 0.014, 0.030), (0.0, -0.035, 0.0))
    shell = _merge(shell, rear_mount)

    shell = shell.cut(_box((0.046, 0.018, 0.024), (0.0, 0.020, 0.003)))
    shell = shell.cut(_box((0.034, 0.018, 0.010), (0.0, 0.000, 0.022)))
    shell = shell.cut(_box((0.016, 0.020, 0.020), (-0.024, 0.014, -0.010)))
    shell = shell.cut(_box((0.016, 0.020, 0.020), (0.024, 0.014, -0.010)))
    return shell


def _front_bridge_shape() -> cq.Workplane:
    bridge = _box((0.220, BRIDGE_Y, 0.048), (0.0, BRIDGE_CENTER_Y, 0.0))
    bridge = bridge.cut(_box((0.104, 0.020, 0.016), (0.0, BRIDGE_CENTER_Y, 0.0)))
    bridge = bridge.cut(_box((0.030, 0.020, 0.020), (-0.075, BRIDGE_CENTER_Y, 0.0)))
    bridge = bridge.cut(_box((0.030, 0.020, 0.020), (0.075, BRIDGE_CENTER_Y, 0.0)))
    return bridge


def _rail_shape(z_offset: float) -> cq.Workplane:
    return _box((RAIL_X, RAIL_Y, RAIL_Z), (0.0, RAIL_CENTER_Y, z_offset))


def _stop_shape(sign: float) -> cq.Workplane:
    stop_x = sign * STOP_X
    stop = _box(STOP_SIZE, (stop_x, RAIL_CENTER_Y, 0.0))
    stop = stop.cut(_box((0.004, 0.020, 0.014), (stop_x, RAIL_CENTER_Y, 0.0)))
    return stop


def _guide_carriage_shape() -> cq.Workplane:
    upper = _box((RUNNER_X, RUNNER_Y, RUNNER_Z), (0.0, RUNNER_CENTER_Y, RAIL_Z_OFFSET))
    lower = _box((RUNNER_X, RUNNER_Y, RUNNER_Z), (0.0, RUNNER_CENTER_Y, -RAIL_Z_OFFSET))
    spine = _box((SPINE_X, SPINE_Y, SPINE_Z), (0.0, SPINE_CENTER_Y, 0.0))
    return _merge(upper, lower, spine)


def _finger_shape(sign: float) -> cq.Workplane:
    tower = _box((0.012, 0.018, 0.090), (sign * 0.018, 0.061, 0.018))
    saddle = _box((0.038, 0.012, 0.022), (sign * 0.008, 0.058, 0.016))
    lower_rib = _box((0.014, 0.010, 0.040), (sign * 0.010, 0.056, -0.008))
    return _merge(tower, saddle, lower_rib)


def _cover_shape(sign: float) -> cq.Workplane:
    cover_shell = _box((0.042, 0.009, 0.018), (sign * 0.018, 0.0485, 0.0))
    rear_clip = _box((0.018, 0.004, 0.012), (sign * 0.010, 0.051, 0.0))
    return _merge(cover_shell, rear_clip)


def _tip_shape(sign: float) -> cq.Workplane:
    pad = _box((0.014, 0.008, 0.048), (sign * 0.026, 0.068, 0.020))
    upper_tine = _box((0.034, 0.005, 0.006), (sign * 0.048, 0.074, 0.033))
    lower_tine = _box((0.034, 0.005, 0.006), (sign * 0.048, 0.074, 0.007))
    return _merge(pad, upper_tine, lower_tine)


def _jaw_inertial(sign: float) -> Inertial:
    return Inertial.from_geometry(
        Box((0.078, 0.060, 0.094)),
        mass=0.34,
        origin=Origin(xyz=(sign * 0.018, 0.059, 0.015)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wafer_handling_gripper")

    body_dark = model.material("body_dark", rgba=(0.24, 0.26, 0.29, 1.0))
    rail_steel = model.material("rail_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    cover_steel = model.material("cover_steel", rgba=(0.82, 0.83, 0.85, 1.0))
    jaw_dark = model.material("jaw_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    tip_steel = model.material("tip_steel", rgba=(0.84, 0.85, 0.80, 1.0))
    bumper_black = model.material("bumper_black", rgba=(0.10, 0.10, 0.11, 1.0))

    body = model.part(
        "body",
        inertial=Inertial.from_geometry(
            Box((0.220, 0.080, 0.060)),
            mass=1.9,
            origin=Origin(xyz=(0.0, 0.010, 0.0)),
        ),
    )
    body.visual(
        mesh_from_cadquery(_body_shell_shape(), "body_shell"),
        material=body_dark,
        name="body_shell",
    )
    body.visual(
        mesh_from_cadquery(_front_bridge_shape(), "front_bridge"),
        material=body_dark,
        name="front_bridge",
    )
    body.visual(
        mesh_from_cadquery(_rail_shape(RAIL_Z_OFFSET), "upper_rail"),
        material=rail_steel,
        name="upper_rail",
    )
    body.visual(
        mesh_from_cadquery(_rail_shape(-RAIL_Z_OFFSET), "lower_rail"),
        material=rail_steel,
        name="lower_rail",
    )
    body.visual(
        mesh_from_cadquery(_stop_shape(-1.0), "left_stop"),
        material=bumper_black,
        name="left_stop",
    )
    body.visual(
        mesh_from_cadquery(_stop_shape(1.0), "right_stop"),
        material=bumper_black,
        name="right_stop",
    )

    left_jaw = model.part("left_jaw", inertial=_jaw_inertial(1.0))
    left_jaw.visual(
        mesh_from_cadquery(_guide_carriage_shape(), "left_guide_blocks"),
        material=jaw_dark,
        name="guide_blocks",
    )
    left_jaw.visual(
        mesh_from_cadquery(_finger_shape(1.0), "left_finger"),
        material=jaw_dark,
        name="finger",
    )
    left_jaw.visual(
        mesh_from_cadquery(_cover_shape(1.0), "left_cover"),
        material=cover_steel,
        name="cover",
    )
    left_jaw.visual(
        mesh_from_cadquery(_tip_shape(1.0), "left_tip"),
        material=tip_steel,
        name="tip",
    )

    right_jaw = model.part("right_jaw", inertial=_jaw_inertial(-1.0))
    right_jaw.visual(
        mesh_from_cadquery(_guide_carriage_shape(), "right_guide_blocks"),
        material=jaw_dark,
        name="guide_blocks",
    )
    right_jaw.visual(
        mesh_from_cadquery(_finger_shape(-1.0), "right_finger"),
        material=jaw_dark,
        name="finger",
    )
    right_jaw.visual(
        mesh_from_cadquery(_cover_shape(-1.0), "right_cover"),
        material=cover_steel,
        name="cover",
    )
    right_jaw.visual(
        mesh_from_cadquery(_tip_shape(-1.0), "right_tip"),
        material=tip_steel,
        name="tip",
    )

    model.articulation(
        "left_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=left_jaw,
        origin=Origin(xyz=(LEFT_JAW_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.20,
            lower=0.0,
            upper=JAW_TRAVEL,
        ),
    )
    model.articulation(
        "right_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=right_jaw,
        origin=Origin(xyz=(RIGHT_JAW_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.20,
            lower=-JAW_TRAVEL,
            upper=0.0,
        ),
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

    body = object_model.get_part("body")
    left_jaw = object_model.get_part("left_jaw")
    right_jaw = object_model.get_part("right_jaw")
    left_slide = object_model.get_articulation("left_slide")
    right_slide = object_model.get_articulation("right_slide")

    left_ok = (
        left_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(left_slide.axis) == (1.0, 0.0, 0.0)
        and left_slide.motion_limits is not None
        and left_slide.motion_limits.lower == 0.0
        and left_slide.motion_limits.upper == JAW_TRAVEL
    )
    right_ok = (
        right_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(right_slide.axis) == (1.0, 0.0, 0.0)
        and right_slide.motion_limits is not None
        and right_slide.motion_limits.lower == -JAW_TRAVEL
        and right_slide.motion_limits.upper == 0.0
    )
    ctx.check(
        "jaw_slides_are_mirrored_prismatic_guides",
        left_ok and right_ok,
        details=(
            f"left={left_slide.axis, left_slide.motion_limits} "
            f"right={right_slide.axis, right_slide.motion_limits}"
        ),
    )

    ctx.expect_contact(
        left_jaw,
        body,
        elem_a="guide_blocks",
        elem_b="upper_rail",
        name="left_upper_runner_contacts_upper_rail",
    )
    ctx.expect_contact(
        left_jaw,
        body,
        elem_a="guide_blocks",
        elem_b="lower_rail",
        name="left_lower_runner_contacts_lower_rail",
    )
    ctx.expect_contact(
        right_jaw,
        body,
        elem_a="guide_blocks",
        elem_b="upper_rail",
        name="right_upper_runner_contacts_upper_rail",
    )
    ctx.expect_contact(
        right_jaw,
        body,
        elem_a="guide_blocks",
        elem_b="lower_rail",
        name="right_lower_runner_contacts_lower_rail",
    )

    with ctx.pose({left_slide: JAW_TRAVEL, right_slide: -JAW_TRAVEL}):
        ctx.expect_gap(
            right_jaw,
            left_jaw,
            axis="x",
            positive_elem="tip",
            negative_elem="tip",
            min_gap=0.008,
            max_gap=0.014,
            name="closed_tips_leave_pickup_gap",
        )
        ctx.expect_gap(
            left_jaw,
            body,
            axis="y",
            positive_elem="cover",
            negative_elem="front_bridge",
            min_gap=0.0015,
            max_gap=0.0045,
            name="left_cover_clears_front_bridge",
        )
        ctx.expect_gap(
            right_jaw,
            body,
            axis="y",
            positive_elem="cover",
            negative_elem="front_bridge",
            min_gap=0.0015,
            max_gap=0.0045,
            name="right_cover_clears_front_bridge",
        )

    with ctx.pose({left_slide: 0.0, right_slide: 0.0}):
        ctx.expect_gap(
            left_jaw,
            body,
            axis="x",
            positive_elem="guide_blocks",
            negative_elem="left_stop",
            min_gap=0.0025,
            max_gap=0.0065,
            name="left_jaw_stays_inside_left_end_stop",
        )
        ctx.expect_gap(
            body,
            right_jaw,
            axis="x",
            positive_elem="right_stop",
            negative_elem="guide_blocks",
            min_gap=0.0025,
            max_gap=0.0065,
            name="right_jaw_stays_inside_right_end_stop",
        )

    ctx.warn_if_articulation_overlaps(max_pose_samples=24)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
