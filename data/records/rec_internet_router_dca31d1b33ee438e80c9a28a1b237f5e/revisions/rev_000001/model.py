from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose, pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


BODY_WIDTH = 0.240
BODY_DEPTH = 0.160
BODY_HEIGHT = 0.034
SHELL_THICKNESS = 0.003

FOOT_WIDTH = 0.074
FOOT_DEPTH = 0.016
FOOT_HEIGHT = 0.003
FOOT_X_OFFSET = 0.057
FOOT_Y = 0.000

BUTTON_CENTER_X = 0.084
BUTTON_CENTER_Z = 0.016
BUTTON_WIDTH = 0.012
BUTTON_HEIGHT = 0.006
BUTTON_DEPTH = 0.011
BUTTON_TRAVEL = 0.0035

BUTTON_GUIDE_FRAME = 0.002
BUTTON_GUIDE_DEPTH = 0.014
BUTTON_REST_CENTER_Y = 0.0725
BUTTON_GUIDE_CENTER_Y = 0.070

ANTENNA_XS = (-0.090, -0.030, 0.030, 0.090)
ANTENNA_HINGE_Y = 0.073
ANTENNA_BARREL_RADIUS = 0.007
ANTENNA_BARREL_LENGTH = 0.026
ANTENNA_HINGE_Z = BODY_HEIGHT + ANTENNA_BARREL_RADIUS
ANTENNA_BASE_SIZE = (0.022, 0.014, 0.004)
ANTENNA_LIMIT_LOWER = -1.25
ANTENNA_LIMIT_UPPER = 0.55


def _add_box(part, name, size, xyz, material):
    return part.visual(
        Box(size),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def _add_cylinder(part, name, radius, length, xyz, material, rpy=(0.0, 0.0, 0.0)):
    return part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _aabb_dims(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple(maxs[i] - mins[i] for i in range(3))


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))


def _rounded_rect_section(width, depth, z, radius):
    limited_radius = min(radius, width * 0.49, depth * 0.49)
    return [
        (x, y, z)
        for x, y in rounded_rect_profile(width, depth, limited_radius, corner_segments=8)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_wifi_router")

    body_mat = model.material("body_plastic", rgba=(0.13, 0.14, 0.15, 1.0))
    trim_mat = model.material("trim_plastic", rgba=(0.08, 0.09, 0.10, 1.0))
    button_mat = model.material("button_plastic", rgba=(0.23, 0.24, 0.26, 1.0))
    foot_mat = model.material("rubber_feet", rgba=(0.05, 0.05, 0.05, 1.0))

    body = model.part("body")
    crown_mesh = mesh_from_geometry(
        section_loft(
            [
                _rounded_rect_section(0.186, 0.104, 0.000, 0.018),
                _rounded_rect_section(0.178, 0.098, 0.0025, 0.017),
                _rounded_rect_section(0.166, 0.086, 0.0050, 0.015),
            ]
        ),
        "router_top_crown",
    )
    antenna_blade_mesh = mesh_from_geometry(
        section_loft(
            [
                _rounded_rect_section(0.0180, 0.0062, 0.002, 0.0024),
                _rounded_rect_section(0.0168, 0.0058, 0.030, 0.0022),
                _rounded_rect_section(0.0150, 0.0052, 0.084, 0.0020),
                _rounded_rect_section(0.0130, 0.0046, 0.132, 0.0018),
                _rounded_rect_section(0.0100, 0.0038, 0.166, 0.0014),
            ]
        ),
        "router_antenna_blade",
    )

    inner_width = BODY_WIDTH - 2.0 * SHELL_THICKNESS
    inner_depth = BODY_DEPTH - 2.0 * SHELL_THICKNESS
    wall_height = BODY_HEIGHT - 2.0 * SHELL_THICKNESS
    inner_half_x = BODY_WIDTH * 0.5 - SHELL_THICKNESS
    inner_top_z = BODY_HEIGHT - SHELL_THICKNESS

    hole_left = BUTTON_CENTER_X - BUTTON_WIDTH * 0.5
    hole_right = BUTTON_CENTER_X + BUTTON_WIDTH * 0.5
    hole_bottom = BUTTON_CENTER_Z - BUTTON_HEIGHT * 0.5
    hole_top = BUTTON_CENTER_Z + BUTTON_HEIGHT * 0.5

    rear_left_width = hole_left + inner_half_x
    rear_right_width = inner_half_x - hole_right
    rear_lower_height = hole_bottom - SHELL_THICKNESS
    rear_upper_height = inner_top_z - hole_top

    _add_box(
        body,
        "bottom_shell",
        (BODY_WIDTH, BODY_DEPTH, SHELL_THICKNESS),
        (0.0, 0.0, SHELL_THICKNESS * 0.5),
        body_mat,
    )
    _add_box(
        body,
        "top_shell",
        (inner_width, inner_depth, SHELL_THICKNESS),
        (0.0, 0.0, BODY_HEIGHT - SHELL_THICKNESS * 0.5),
        body_mat,
    )
    body.visual(
        crown_mesh,
        origin=Origin(xyz=(0.0, -0.010, BODY_HEIGHT - SHELL_THICKNESS)),
        material=trim_mat,
        name="top_crown",
    )
    _add_box(
        body,
        "front_wall",
        (inner_width, SHELL_THICKNESS, wall_height),
        (0.0, -BODY_DEPTH * 0.5 + SHELL_THICKNESS * 0.5, SHELL_THICKNESS + wall_height * 0.5),
        body_mat,
    )
    _add_box(
        body,
        "left_wall",
        (SHELL_THICKNESS, inner_depth, wall_height),
        (-BODY_WIDTH * 0.5 + SHELL_THICKNESS * 0.5, 0.0, SHELL_THICKNESS + wall_height * 0.5),
        body_mat,
    )
    _add_box(
        body,
        "right_wall",
        (SHELL_THICKNESS, inner_depth, wall_height),
        (BODY_WIDTH * 0.5 - SHELL_THICKNESS * 0.5, 0.0, SHELL_THICKNESS + wall_height * 0.5),
        body_mat,
    )
    _add_box(
        body,
        "rear_wall_left",
        (rear_left_width, SHELL_THICKNESS, wall_height),
        ((-inner_half_x + hole_left) * 0.5, BODY_DEPTH * 0.5 - SHELL_THICKNESS * 0.5, SHELL_THICKNESS + wall_height * 0.5),
        body_mat,
    )
    _add_box(
        body,
        "rear_wall_right",
        (rear_right_width, SHELL_THICKNESS, wall_height),
        ((hole_right + inner_half_x) * 0.5, BODY_DEPTH * 0.5 - SHELL_THICKNESS * 0.5, SHELL_THICKNESS + wall_height * 0.5),
        body_mat,
    )
    _add_box(
        body,
        "rear_wall_lower",
        (BUTTON_WIDTH, SHELL_THICKNESS, rear_lower_height),
        (BUTTON_CENTER_X, BODY_DEPTH * 0.5 - SHELL_THICKNESS * 0.5, SHELL_THICKNESS + rear_lower_height * 0.5),
        body_mat,
    )
    _add_box(
        body,
        "rear_wall_upper",
        (BUTTON_WIDTH, SHELL_THICKNESS, rear_upper_height),
        (BUTTON_CENTER_X, BODY_DEPTH * 0.5 - SHELL_THICKNESS * 0.5, hole_top + rear_upper_height * 0.5),
        body_mat,
    )

    guide_outer_width = BUTTON_WIDTH + 2.0 * BUTTON_GUIDE_FRAME
    guide_outer_height = BUTTON_HEIGHT + 2.0 * BUTTON_GUIDE_FRAME
    guide_side_x = BUTTON_CENTER_X + BUTTON_WIDTH * 0.5 + BUTTON_GUIDE_FRAME * 0.5
    guide_top_z = BUTTON_CENTER_Z + BUTTON_HEIGHT * 0.5 + BUTTON_GUIDE_FRAME * 0.5
    guide_bottom_z = BUTTON_CENTER_Z - BUTTON_HEIGHT * 0.5 - BUTTON_GUIDE_FRAME * 0.5

    _add_box(
        body,
        "button_guide_left",
        (BUTTON_GUIDE_FRAME, BUTTON_GUIDE_DEPTH, guide_outer_height),
        (BUTTON_CENTER_X - BUTTON_WIDTH * 0.5 - BUTTON_GUIDE_FRAME * 0.5, BUTTON_GUIDE_CENTER_Y, BUTTON_CENTER_Z),
        trim_mat,
    )
    _add_box(
        body,
        "button_guide_right",
        (BUTTON_GUIDE_FRAME, BUTTON_GUIDE_DEPTH, guide_outer_height),
        (guide_side_x, BUTTON_GUIDE_CENTER_Y, BUTTON_CENTER_Z),
        trim_mat,
    )
    _add_box(
        body,
        "button_guide_upper",
        (guide_outer_width, BUTTON_GUIDE_DEPTH, BUTTON_GUIDE_FRAME),
        (BUTTON_CENTER_X, BUTTON_GUIDE_CENTER_Y, guide_top_z),
        trim_mat,
    )
    _add_box(
        body,
        "button_guide_lower",
        (guide_outer_width, BUTTON_GUIDE_DEPTH, BUTTON_GUIDE_FRAME),
        (BUTTON_CENTER_X, BUTTON_GUIDE_CENTER_Y, guide_bottom_z),
        trim_mat,
    )

    _add_box(
        body,
        "left_foot",
        (FOOT_WIDTH, FOOT_DEPTH, FOOT_HEIGHT),
        (-FOOT_X_OFFSET, FOOT_Y, -FOOT_HEIGHT * 0.5),
        foot_mat,
    )
    _add_box(
        body,
        "right_foot",
        (FOOT_WIDTH, FOOT_DEPTH, FOOT_HEIGHT),
        (FOOT_X_OFFSET, FOOT_Y, -FOOT_HEIGHT * 0.5),
        foot_mat,
    )

    for index, antenna_x in enumerate(ANTENNA_XS, start=1):
        _add_box(
            body,
            f"antenna_base_{index}",
            ANTENNA_BASE_SIZE,
            (antenna_x, ANTENNA_HINGE_Y - 0.002, BODY_HEIGHT - ANTENNA_BASE_SIZE[2] * 0.5),
            trim_mat,
        )

        antenna = model.part(f"antenna_{index}")
        _add_cylinder(
            antenna,
            "hinge_barrel",
            ANTENNA_BARREL_RADIUS,
            ANTENNA_BARREL_LENGTH,
            (0.0, 0.0, 0.0),
            trim_mat,
            rpy=(0.0, pi * 0.5, 0.0),
        )
        antenna.visual(
            antenna_blade_mesh,
            origin=Origin(xyz=(0.0, 0.0, ANTENNA_BARREL_RADIUS - 0.0015)),
            material=trim_mat,
            name="blade",
        )
        _add_box(
            antenna,
            "hinge_cap",
            (0.012, 0.008, 0.008),
            (0.0, 0.0, ANTENNA_BARREL_RADIUS * 0.8),
            trim_mat,
        )

        model.articulation(
            f"antenna_{index}_hinge",
            ArticulationType.REVOLUTE,
            parent=body,
            child=antenna,
            origin=Origin(xyz=(antenna_x, ANTENNA_HINGE_Y, ANTENNA_HINGE_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.2,
                velocity=2.0,
                lower=ANTENNA_LIMIT_LOWER,
                upper=ANTENNA_LIMIT_UPPER,
            ),
        )

    button = model.part("reset_button")
    _add_box(
        button,
        "button_plunger",
        (BUTTON_WIDTH, BUTTON_DEPTH, BUTTON_HEIGHT),
        (0.0, 0.0, 0.0),
        button_mat,
    )
    model.articulation(
        "reset_button_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=button,
        origin=Origin(xyz=(BUTTON_CENTER_X, BUTTON_REST_CENTER_Y, BUTTON_CENTER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.15,
            lower=0.0,
            upper=BUTTON_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    button = object_model.get_part("reset_button")
    antenna_parts = [object_model.get_part(f"antenna_{index}") for index in range(1, 5)]
    antenna_joints = [object_model.get_articulation(f"antenna_{index}_hinge") for index in range(1, 5)]
    button_joint = object_model.get_articulation("reset_button_slide")

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
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=24,
        ignore_adjacent=False,
        ignore_fixed=True,
    )
    ctx.fail_if_isolated_parts(max_pose_samples=24, name="router_sampled_no_floating")
    ctx.fail_if_articulation_overlaps(max_pose_samples=24, name="router_articulation_clearance")

    body_aabb = ctx.part_world_aabb(body)
    body_dims = _aabb_dims(body_aabb)
    if body_dims is not None:
        ctx.check(
            "body_realistic_size",
            0.22 <= body_dims[0] <= 0.25 and 0.15 <= body_dims[1] <= 0.17 and 0.035 <= body_dims[2] <= 0.040,
            f"body dims were {body_dims}",
        )
    else:
        ctx.fail("body_realistic_size", "body AABB unavailable")

    ctx.check(
        "body_is_single_root",
        [part.name for part in object_model.root_parts()] == ["body"],
        f"root parts were {[part.name for part in object_model.root_parts()]}",
    )

    for index, (joint, expected_x) in enumerate(zip(antenna_joints, ANTENNA_XS, strict=False), start=1):
        limits = joint.motion_limits
        ctx.check(
            f"antenna_{index}_joint_type",
            joint.articulation_type == ArticulationType.REVOLUTE,
            f"expected REVOLUTE, got {joint.articulation_type}",
        )
        ctx.check(
            f"antenna_{index}_joint_axis",
            tuple(joint.axis) == (1.0, 0.0, 0.0),
            f"axis was {joint.axis}",
        )
        ctx.check(
            f"antenna_{index}_joint_x_position",
            isclose(joint.origin.xyz[0], expected_x, abs_tol=1e-6)
            and isclose(joint.origin.xyz[1], ANTENNA_HINGE_Y, abs_tol=1e-6)
            and isclose(joint.origin.xyz[2], ANTENNA_HINGE_Z, abs_tol=1e-6),
            f"origin was {joint.origin.xyz}",
        )
        ctx.check(
            f"antenna_{index}_joint_limits",
            limits is not None
            and isclose(limits.lower, ANTENNA_LIMIT_LOWER, abs_tol=1e-6)
            and isclose(limits.upper, ANTENNA_LIMIT_UPPER, abs_tol=1e-6),
            f"limits were {limits}",
        )

    button_limits = button_joint.motion_limits
    ctx.check(
        "button_joint_type",
        button_joint.articulation_type == ArticulationType.PRISMATIC,
        f"expected PRISMATIC, got {button_joint.articulation_type}",
    )
    ctx.check(
        "button_joint_axis",
        tuple(button_joint.axis) == (0.0, -1.0, 0.0),
        f"axis was {button_joint.axis}",
    )
    ctx.check(
        "button_joint_limits",
        button_limits is not None
        and isclose(button_limits.lower, 0.0, abs_tol=1e-6)
        and isclose(button_limits.upper, BUTTON_TRAVEL, abs_tol=1e-6),
        f"limits were {button_limits}",
    )

    for index, antenna in enumerate(antenna_parts, start=1):
        joint = antenna_joints[index - 1]
        limits = joint.motion_limits
        ctx.expect_contact(antenna, body, name=f"antenna_{index}_mounted_contact")
        ctx.expect_gap(
            antenna,
            body,
            axis="z",
            min_gap=0.0,
            max_gap=0.0002,
            negative_elem="top_shell",
            name=f"antenna_{index}_mounted_on_roof",
        )

        with ctx.pose({joint: 0.0}):
            rest_aabb = ctx.part_world_aabb(antenna)
        rest_center = _aabb_center(rest_aabb)

        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({joint: limits.lower}):
                lower_aabb = ctx.part_world_aabb(antenna)
                lower_center = _aabb_center(lower_aabb)
                ctx.fail_if_parts_overlap_in_current_pose(name=f"antenna_{index}_lower_no_overlap")
                ctx.fail_if_isolated_parts(name=f"antenna_{index}_lower_no_floating")
                ctx.expect_contact(antenna, body, name=f"antenna_{index}_lower_contact")
                ctx.expect_gap(
                    antenna,
                    body,
                    axis="z",
                    min_gap=0.0,
                    max_gap=0.0002,
                    negative_elem="top_shell",
                    name=f"antenna_{index}_lower_mount_gap",
                )

            with ctx.pose({joint: limits.upper}):
                upper_aabb = ctx.part_world_aabb(antenna)
                upper_center = _aabb_center(upper_aabb)
                ctx.fail_if_parts_overlap_in_current_pose(name=f"antenna_{index}_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"antenna_{index}_upper_no_floating")
                ctx.expect_contact(antenna, body, name=f"antenna_{index}_upper_contact")
                ctx.expect_gap(
                    antenna,
                    body,
                    axis="z",
                    min_gap=0.0,
                    max_gap=0.0002,
                    negative_elem="top_shell",
                    name=f"antenna_{index}_upper_mount_gap",
                )

            if rest_center is not None and lower_center is not None and upper_center is not None:
                ctx.check(
                    f"antenna_{index}_tilts_back_and_forward",
                    lower_center[1] > rest_center[1] + 0.030 and upper_center[1] < rest_center[1] - 0.015,
                    f"center y values were lower={lower_center[1]}, rest={rest_center[1]}, upper={upper_center[1]}",
                )

    ctx.expect_contact(button, body, name="button_guided_contact")
    ctx.expect_within(button, body, axes="xz", margin=0.0, name="button_within_body_aperture")

    rear_frame_aabb = ctx.part_element_world_aabb(body, elem="rear_wall_upper")
    rest_button_face_recess = None
    pressed_button_face_recess = None

    if rear_frame_aabb is None:
        ctx.fail("button_recess_measure_reference", "rear wall upper AABB unavailable")
    else:
        frame_outer_y = rear_frame_aabb[1][1]

        with ctx.pose({button_joint: 0.0}):
            rest_plunger_aabb = ctx.part_element_world_aabb(button, elem="button_plunger")
            ctx.fail_if_parts_overlap_in_current_pose(name="button_rest_no_overlap")
            ctx.fail_if_isolated_parts(name="button_rest_no_floating")
            ctx.expect_contact(button, body, name="button_rest_contact")

        with ctx.pose({button_joint: BUTTON_TRAVEL}):
            pressed_plunger_aabb = ctx.part_element_world_aabb(button, elem="button_plunger")
            ctx.fail_if_parts_overlap_in_current_pose(name="button_pressed_no_overlap")
            ctx.fail_if_isolated_parts(name="button_pressed_no_floating")
            ctx.expect_contact(button, body, name="button_pressed_contact")
            ctx.expect_within(button, body, axes="xz", margin=0.0, name="button_pressed_within_body_aperture")

        if rest_plunger_aabb is None or pressed_plunger_aabb is None:
            ctx.fail("button_recess_measurements", "button plunger AABB unavailable")
        else:
            rest_button_face_recess = frame_outer_y - rest_plunger_aabb[1][1]
            pressed_button_face_recess = frame_outer_y - pressed_plunger_aabb[1][1]
            travel_distance = rest_plunger_aabb[1][1] - pressed_plunger_aabb[1][1]

            ctx.check(
                "button_recessed_at_rest",
                0.0015 <= rest_button_face_recess <= 0.0025,
                f"rest recess was {rest_button_face_recess}",
            )
            ctx.check(
                "button_recessed_when_pressed",
                0.0050 <= pressed_button_face_recess <= 0.0065,
                f"pressed recess was {pressed_button_face_recess}",
            )
            ctx.check(
                "button_short_prismatic_travel",
                isclose(travel_distance, BUTTON_TRAVEL, abs_tol=2e-4),
                f"button travel was {travel_distance}",
            )

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
