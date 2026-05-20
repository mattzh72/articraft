from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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
    cut_opening_on_face,
    mesh_from_geometry,
    rounded_rect_profile,
)


BODY_W = 0.240
BODY_D = 0.172
BODY_T = 0.028
BODY_R = 0.022
BODY_BOTTOM_T = 0.0075
BODY_MID_T = 0.013
BODY_TOP_T = BODY_T - BODY_BOTTOM_T - BODY_MID_T
SWITCH_RECESS_SPAN_Y = 0.030
POD_W = 0.026
POD_D = 0.018
POD_H = 0.010
POD_CHEEK_X = 0.005
POD_CHEEK_Y = 0.010
POD_CHEEK_Z = 0.010
ANTENNA_BARREL_R = 0.0025
ANTENNA_BARREL_L = POD_W - (2.0 * POD_CHEEK_X)
ANTENNA_PANEL_W = 0.022
ANTENNA_PANEL_H = 0.092
ANTENNA_PANEL_T = 0.005
SWITCH_W = 0.020
SWITCH_H = 0.013
SWITCH_DEPTH = 0.010
SWITCH_FRAME_SPAN_Y = 0.030
SWITCH_FRAME_H = 0.012
SWITCH_FRAME_BACK_T = 0.0015
SWITCH_FRAME_CHEEK_T = 0.003
SWITCH_FRAME_DEPTH = 0.008
SWITCH_TRUNNION_LEN = SWITCH_FRAME_SPAN_Y - (2.0 * SWITCH_FRAME_CHEEK_T)
SWITCH_PIVOT_X = 0.0042


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)
def _antenna_panel_mesh(name: str):
    return _mesh(
        name,
        ExtrudeGeometry(
            rounded_rect_profile(
                ANTENNA_PANEL_W,
                ANTENNA_PANEL_H,
                radius=0.0032,
                corner_segments=6,
            ),
            ANTENNA_PANEL_T,
            center=True,
        ).rotate_x(pi / 2.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="home_router")

    body_black = model.material("body_black", rgba=(0.13, 0.14, 0.15, 1.0))
    trim_black = model.material("trim_black", rgba=(0.08, 0.09, 0.10, 1.0))
    antenna_black = model.material("antenna_black", rgba=(0.10, 0.10, 0.11, 1.0))
    led_green = model.material("led_green", rgba=(0.18, 0.72, 0.36, 1.0))

    body = model.part("body")
    lower_shell = ExtrudeGeometry.from_z0(
        rounded_rect_profile(BODY_W, BODY_D, BODY_R, corner_segments=10),
        BODY_BOTTOM_T,
    )
    upper_shell = ExtrudeGeometry.from_z0(
        rounded_rect_profile(BODY_W, BODY_D, BODY_R, corner_segments=10),
        BODY_TOP_T,
    ).translate(0.0, 0.0, BODY_BOTTOM_T + BODY_MID_T)
    body.visual(_mesh("router_body_lower_shell", lower_shell), material=body_black, name="body_lower_shell")
    body.visual(_mesh("router_body_upper_shell", upper_shell), material=body_black, name="body_upper_shell")

    middle_z = BODY_BOTTOM_T + (BODY_MID_T * 0.5)
    recess_band = BODY_D - (2.0 * BODY_R)
    front_rear_span = (recess_band - SWITCH_RECESS_SPAN_Y) * 0.5
    right_fill_width = SWITCH_DEPTH + BODY_R

    body.visual(
        Box((BODY_W - (2.0 * BODY_R), BODY_D, BODY_MID_T)),
        origin=Origin(xyz=(0.0, 0.0, middle_z)),
        material=body_black,
        name="body_mid_cross_strip",
    )
    body.visual(
        Box((BODY_W - SWITCH_DEPTH, BODY_D - (2.0 * BODY_R), BODY_MID_T)),
        origin=Origin(xyz=(-SWITCH_DEPTH * 0.5, 0.0, middle_z)),
        material=body_black,
        name="body_mid_center_strip",
    )
    body.visual(
        Box((right_fill_width, front_rear_span, BODY_MID_T)),
        origin=Origin(
            xyz=(
                (BODY_W * 0.5) - (right_fill_width * 0.5),
                -((SWITCH_RECESS_SPAN_Y * 0.5) + (front_rear_span * 0.5)),
                middle_z,
            )
        ),
        material=body_black,
        name="body_mid_front_fill",
    )
    body.visual(
        Box((right_fill_width, front_rear_span, BODY_MID_T)),
        origin=Origin(
            xyz=(
                (BODY_W * 0.5) - (right_fill_width * 0.5),
                (SWITCH_RECESS_SPAN_Y * 0.5) + (front_rear_span * 0.5),
                middle_z,
            )
        ),
        material=body_black,
        name="body_mid_rear_fill",
    )
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            body.visual(
                Cylinder(radius=BODY_R, length=BODY_MID_T),
                origin=Origin(
                    xyz=(
                        x_sign * ((BODY_W * 0.5) - BODY_R),
                        y_sign * ((BODY_D * 0.5) - BODY_R),
                        middle_z,
                    )
                ),
                material=body_black,
                name=f"mid_corner_{'r' if x_sign > 0 else 'l'}_{'rear' if y_sign > 0 else 'front'}",
            )
    body.visual(
        Box((0.092, 0.004, 0.002)),
        origin=Origin(xyz=(0.0, (-BODY_D * 0.5) + 0.002, BODY_T * 0.44)),
        material=led_green,
        name="status_light",
    )
    body.visual(
        Box((0.120, 0.020, 0.002)),
        origin=Origin(xyz=(0.0, 0.040, BODY_T - 0.001)),
        material=trim_black,
        name="rear_vent_strip",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_D, BODY_T)),
        mass=0.85,
        origin=Origin(xyz=(0.0, 0.0, BODY_T * 0.5)),
    )

    pod_x_positions = (-0.074, 0.074)
    antenna_mesh = _antenna_panel_mesh("router_antenna_panel")
    pod_parts = []
    antenna_parts = []

    for side_name, x_pos in (("left", pod_x_positions[0]), ("right", pod_x_positions[1])):
        pod = model.part(f"{side_name}_pod")
        pod.visual(
            Box((POD_W, POD_D, POD_H)),
            origin=Origin(xyz=(0.0, 0.0, POD_H * 0.5)),
            material=trim_black,
            name="pod_base",
        )
        pod.visual(
            Box((POD_CHEEK_X, POD_CHEEK_Y, POD_CHEEK_Z)),
            origin=Origin(
                xyz=(
                    -(ANTENNA_BARREL_L * 0.5 + POD_CHEEK_X * 0.5),
                    POD_D * 0.5 - POD_CHEEK_Y * 0.5,
                    POD_H + POD_CHEEK_Z * 0.5,
                )
            ),
            material=trim_black,
            name="inner_left_cheek",
        )
        pod.visual(
            Box((POD_CHEEK_X, POD_CHEEK_Y, POD_CHEEK_Z)),
            origin=Origin(
                xyz=(
                    ANTENNA_BARREL_L * 0.5 + POD_CHEEK_X * 0.5,
                    POD_D * 0.5 - POD_CHEEK_Y * 0.5,
                    POD_H + POD_CHEEK_Z * 0.5,
                )
            ),
            material=trim_black,
            name="inner_right_cheek",
        )
        pod.inertial = Inertial.from_geometry(
            Box((POD_W, POD_D, POD_H + POD_CHEEK_Z)),
            mass=0.04,
            origin=Origin(xyz=(0.0, 0.0, 0.010)),
        )

        antenna = model.part(f"{side_name}_antenna")
        antenna.visual(
            Cylinder(radius=ANTENNA_BARREL_R, length=ANTENNA_BARREL_L),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=trim_black,
            name="hinge_barrel",
        )
        antenna.visual(
            Box((0.018, 0.005, 0.018)),
            origin=Origin(xyz=(0.0, 0.0045, 0.010)),
            material=antenna_black,
            name="antenna_stem",
        )
        antenna.visual(
            antenna_mesh,
            origin=Origin(xyz=(0.0, 0.0055, 0.064)),
            material=antenna_black,
            name="antenna_panel",
        )
        antenna.inertial = Inertial.from_geometry(
            Box((0.024, 0.012, 0.110)),
            mass=0.07,
            origin=Origin(xyz=(0.0, 0.004, 0.055)),
        )

        model.articulation(
            f"body_to_{side_name}_pod",
            ArticulationType.FIXED,
            parent=body,
            child=pod,
            origin=Origin(xyz=(x_pos, (BODY_D * 0.5) - (POD_D * 0.5), BODY_T)),
        )
        model.articulation(
            f"{side_name}_antenna_hinge",
            ArticulationType.REVOLUTE,
            parent=pod,
            child=antenna,
            origin=Origin(xyz=(0.0, POD_D * 0.5, POD_H + POD_CHEEK_Z * 0.5)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.2,
                velocity=2.0,
                lower=-1.50,
                upper=0.10,
            ),
        )
        pod_parts.append(pod)
        antenna_parts.append(antenna)

    switch_frame = model.part("switch_frame")
    switch_frame.visual(
        Box((SWITCH_FRAME_BACK_T, SWITCH_FRAME_SPAN_Y, SWITCH_FRAME_H)),
        origin=Origin(xyz=(SWITCH_FRAME_BACK_T * 0.5, 0.0, 0.0)),
        material=trim_black,
        name="frame_backplate",
    )
    switch_frame.visual(
        Box((SWITCH_FRAME_DEPTH, SWITCH_FRAME_CHEEK_T, SWITCH_FRAME_H)),
        origin=Origin(
            xyz=(
                SWITCH_FRAME_DEPTH * 0.5,
                -((SWITCH_FRAME_SPAN_Y * 0.5) - (SWITCH_FRAME_CHEEK_T * 0.5)),
                0.0,
            )
        ),
        material=trim_black,
        name="frame_front_cheek",
    )
    switch_frame.visual(
        Box((SWITCH_FRAME_DEPTH, SWITCH_FRAME_CHEEK_T, SWITCH_FRAME_H)),
        origin=Origin(
            xyz=(
                SWITCH_FRAME_DEPTH * 0.5,
                (SWITCH_FRAME_SPAN_Y * 0.5) - (SWITCH_FRAME_CHEEK_T * 0.5),
                0.0,
            )
        ),
        material=trim_black,
        name="frame_rear_cheek",
    )
    switch_frame.inertial = Inertial.from_geometry(
        Box((SWITCH_FRAME_DEPTH, SWITCH_FRAME_SPAN_Y, SWITCH_FRAME_H)),
        mass=0.012,
        origin=Origin(xyz=(SWITCH_FRAME_DEPTH * 0.5, 0.0, 0.0)),
    )

    power_switch = model.part("power_switch")
    power_switch.visual(
        Cylinder(radius=0.0018, length=SWITCH_TRUNNION_LEN),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=trim_black,
        name="switch_trunnion",
    )
    power_switch.visual(
        Box((0.0045, 0.022, 0.0085)),
        origin=Origin(xyz=(0.0022, 0.0, 0.0)),
        material=trim_black,
        name="switch_core",
    )
    power_switch.visual(
        Box((0.0032, 0.018, 0.0105)),
        origin=Origin(xyz=(0.0056, 0.0, 0.0)),
        material=trim_black,
        name="switch_paddle",
    )
    power_switch.inertial = Inertial.from_geometry(
        Box((0.009, SWITCH_TRUNNION_LEN, 0.011)),
        mass=0.01,
        origin=Origin(xyz=(0.0035, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_switch_frame",
        ArticulationType.FIXED,
        parent=body,
        child=switch_frame,
        origin=Origin(
            xyz=(
                (BODY_W * 0.5) - SWITCH_DEPTH,
                0.0,
                BODY_BOTTOM_T + (BODY_MID_T * 0.5),
            )
        ),
    )
    model.articulation(
        "power_switch_rocker",
        ArticulationType.REVOLUTE,
        parent=switch_frame,
        child=power_switch,
        origin=Origin(xyz=(SWITCH_PIVOT_X, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=3.0,
            lower=-0.22,
            upper=0.22,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    left_pod = object_model.get_part("left_pod")
    right_pod = object_model.get_part("right_pod")
    switch_frame = object_model.get_part("switch_frame")
    left_antenna = object_model.get_part("left_antenna")
    right_antenna = object_model.get_part("right_antenna")
    power_switch = object_model.get_part("power_switch")

    left_hinge = object_model.get_articulation("left_antenna_hinge")
    right_hinge = object_model.get_articulation("right_antenna_hinge")
    switch_hinge = object_model.get_articulation("power_switch_rocker")

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

    root_parts = object_model.root_parts()
    ctx.check(
        "single_root_body",
        len(root_parts) == 1 and root_parts[0].name == "body",
        f"expected body as the single root part, got {[part.name for part in root_parts]}",
    )
    ctx.check(
        "antenna_hinges_horizontal",
        left_hinge.axis == (1.0, 0.0, 0.0) and right_hinge.axis == (1.0, 0.0, 0.0),
        f"unexpected antenna hinge axes: left={left_hinge.axis}, right={right_hinge.axis}",
    )
    ctx.check(
        "switch_axis_transverse",
        switch_hinge.axis == (0.0, 1.0, 0.0),
        f"unexpected switch rocker axis: {switch_hinge.axis}",
    )

    body_aabb = ctx.part_world_aabb(body)
    if body_aabb is None:
        ctx.fail("body_aabb_available", "body world AABB was unavailable")
    else:
        body_size = tuple(body_aabb[1][axis] - body_aabb[0][axis] for axis in range(3))
        ctx.check(
            "router_body_proportions",
            0.22 <= body_size[0] <= 0.25
            and 0.15 <= body_size[1] <= 0.19
            and 0.025 <= body_size[2] <= 0.035,
            f"unexpected body size {body_size}",
        )

    ctx.expect_contact(left_pod, body, name="left_pod_mounted_to_body")
    ctx.expect_contact(right_pod, body, name="right_pod_mounted_to_body")
    ctx.expect_contact(left_antenna, left_pod, name="left_antenna_hinge_contact")
    ctx.expect_contact(right_antenna, right_pod, name="right_antenna_hinge_contact")
    ctx.expect_contact(switch_frame, body, name="switch_frame_seated_in_recess")
    ctx.expect_contact(power_switch, switch_frame, name="switch_seated_in_recess")

    ctx.expect_overlap(left_pod, body, axes="xy", min_overlap=0.017, name="left_pod_back_edge_overlap")
    ctx.expect_overlap(right_pod, body, axes="xy", min_overlap=0.017, name="right_pod_back_edge_overlap")
    ctx.expect_within(switch_frame, body, axes="yz", margin=0.0, name="switch_sidewall_within_body")
    ctx.expect_within(power_switch, switch_frame, axes="yz", margin=0.001, name="switch_within_recess_span")

    def _check_joint_limits(joint, moving_part, support_part, stem_name: str) -> None:
        limits = joint.motion_limits
        if limits is None or limits.lower is None or limits.upper is None:
            ctx.fail(f"{stem_name}_limits_present", "expected finite joint limits")
            return
        with ctx.pose({joint: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{stem_name}_lower_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{stem_name}_lower_no_floating")
            ctx.expect_contact(moving_part, support_part, name=f"{stem_name}_lower_contact")
        with ctx.pose({joint: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{stem_name}_upper_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{stem_name}_upper_no_floating")
            ctx.expect_contact(moving_part, support_part, name=f"{stem_name}_upper_contact")

    _check_joint_limits(left_hinge, left_antenna, left_pod, "left_antenna_hinge")
    _check_joint_limits(right_hinge, right_antenna, right_pod, "right_antenna_hinge")
    _check_joint_limits(switch_hinge, power_switch, switch_frame, "power_switch_rocker")

    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=48,
        ignore_adjacent=False,
        ignore_fixed=True,
    )
    ctx.fail_if_isolated_parts(max_pose_samples=24, name="router_pose_sweep_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
