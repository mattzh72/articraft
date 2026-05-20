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


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return math.sqrt(
        (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2
    )


def _rpy_for_segment(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_segment(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_segment(a, b)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cctv_column")

    galvanized_steel = model.material(
        "galvanized_steel", rgba=(0.63, 0.66, 0.69, 1.0)
    )
    stainless = model.material("stainless", rgba=(0.77, 0.79, 0.81, 1.0))
    dark_powder = model.material("dark_powder", rgba=(0.19, 0.20, 0.22, 1.0))
    camera_white = model.material("camera_white", rgba=(0.84, 0.85, 0.87, 1.0))
    lens_black = model.material("lens_black", rgba=(0.05, 0.05, 0.06, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.11, 0.14, 0.16, 0.55))

    column = model.part("column")
    column.visual(
        Box((0.42, 0.42, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=galvanized_steel,
        name="base_plate",
    )
    column.visual(
        Cylinder(radius=0.089, length=4.20),
        origin=Origin(xyz=(0.0, 0.0, 2.124)),
        material=galvanized_steel,
        name="pole_shaft",
    )
    column.visual(
        Cylinder(radius=0.132, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        material=dark_powder,
        name="pole_flange",
    )
    for sx in (-0.145, 0.145):
        for sy in (-0.145, 0.145):
            column.visual(
                Cylinder(radius=0.011, length=0.080),
                origin=Origin(xyz=(sx, sy, 0.064)),
                material=stainless,
                name=f"anchor_bolt_{'p' if sx > 0 else 'n'}x_{'p' if sy > 0 else 'n'}y",
            )
    for sign in (-1.0, 1.0):
        column.visual(
            Box((0.110, 0.012, 0.180)),
            origin=Origin(xyz=(sign * 0.135, 0.0, 0.114)),
            material=galvanized_steel,
            name=f"gusset_x_{'pos' if sign > 0 else 'neg'}",
        )
        column.visual(
            Box((0.012, 0.110, 0.180)),
            origin=Origin(xyz=(0.0, sign * 0.135, 0.114)),
            material=galvanized_steel,
            name=f"gusset_y_{'pos' if sign > 0 else 'neg'}",
        )
    column.inertial = Inertial.from_geometry(
        Box((0.42, 0.42, 4.224)),
        mass=118.0,
        origin=Origin(xyz=(0.0, 0.0, 2.112)),
    )

    side_arm_bracket = model.part("side_arm_bracket")
    side_arm_bracket.visual(
        Cylinder(radius=0.105, length=0.180),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material=galvanized_steel,
        name="bracket_sleeve",
    )
    side_arm_bracket.visual(
        Box((0.180, 0.180, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.186)),
        material=galvanized_steel,
        name="top_cap",
    )
    side_arm_bracket.visual(
        Box((0.600, 0.090, 0.080)),
        origin=Origin(xyz=(0.300, 0.0, 0.140)),
        material=galvanized_steel,
        name="side_arm",
    )
    side_arm_bracket.visual(
        Box((0.050, 0.090, 0.030)),
        origin=Origin(xyz=(0.575, 0.0, 0.115)),
        material=galvanized_steel,
        name="arm_end_plate",
    )
    _add_segment(
        side_arm_bracket,
        (0.045, 0.0, 0.060),
        (0.470, 0.0, 0.100),
        radius=0.025,
        material=galvanized_steel,
        name="diagonal_brace",
    )
    side_arm_bracket.visual(
        Box((0.080, 0.120, 0.060)),
        origin=Origin(xyz=(0.060, 0.0, 0.130)),
        material=dark_powder,
        name="junction_box",
    )
    side_arm_bracket.inertial = Inertial.from_geometry(
        Box((0.660, 0.180, 0.250)),
        mass=16.0,
        origin=Origin(xyz=(0.300, 0.0, 0.125)),
    )

    pan_pod = model.part("pan_pod")
    pan_pod.visual(
        Cylinder(radius=0.042, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.007)),
        material=dark_powder,
        name="pan_disc",
    )
    pan_pod.visual(
        Cylinder(radius=0.034, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
        material=dark_powder,
        name="pan_body",
    )
    pan_pod.visual(
        Box((0.050, 0.092, 0.040)),
        origin=Origin(xyz=(-0.010, 0.0, -0.086)),
        material=dark_powder,
        name="yoke_bridge",
    )
    pan_pod.visual(
        Box((0.028, 0.012, 0.148)),
        origin=Origin(xyz=(0.044, 0.050, -0.153)),
        material=dark_powder,
        name="left_yoke",
    )
    pan_pod.visual(
        Box((0.016, 0.014, 0.080)),
        origin=Origin(xyz=(0.022, 0.045, -0.116)),
        material=dark_powder,
        name="left_yoke_connector",
    )
    pan_pod.visual(
        Box((0.028, 0.012, 0.148)),
        origin=Origin(xyz=(0.044, -0.050, -0.153)),
        material=dark_powder,
        name="right_yoke",
    )
    pan_pod.visual(
        Box((0.016, 0.014, 0.080)),
        origin=Origin(xyz=(0.022, -0.045, -0.116)),
        material=dark_powder,
        name="right_yoke_connector",
    )
    pan_pod.inertial = Inertial.from_geometry(
        Box((0.120, 0.140, 0.220)),
        mass=6.0,
        origin=Origin(xyz=(0.024, 0.0, -0.110)),
    )

    camera_head = model.part("camera_head")
    camera_head.visual(
        Cylinder(radius=0.010, length=0.088),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="tilt_axle",
    )
    camera_head.visual(
        Box((0.048, 0.068, 0.044)),
        origin=Origin(xyz=(0.014, 0.0, 0.0)),
        material=dark_powder,
        name="pivot_collar",
    )
    camera_head.visual(
        Cylinder(radius=0.040, length=0.220),
        origin=Origin(xyz=(0.148, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=camera_white,
        name="camera_body",
    )
    camera_head.visual(
        Box((0.160, 0.092, 0.026)),
        origin=Origin(xyz=(0.160, 0.0, 0.048)),
        material=camera_white,
        name="sunshade",
    )
    camera_head.visual(
        Cylinder(radius=0.046, length=0.024),
        origin=Origin(xyz=(0.270, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=camera_white,
        name="front_bezel",
    )
    camera_head.visual(
        Cylinder(radius=0.022, length=0.010),
        origin=Origin(xyz=(0.282, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=smoked_glass,
        name="lens_window",
    )
    camera_head.visual(
        Cylinder(radius=0.015, length=0.014),
        origin=Origin(xyz=(0.284, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_black,
        name="lens_core",
    )
    camera_head.visual(
        Cylinder(radius=0.006, length=0.040),
        origin=Origin(xyz=(0.120, 0.0, -0.035), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_black,
        name="drain_tube",
    )
    camera_head.inertial = Inertial.from_geometry(
        Box((0.270, 0.100, 0.120)),
        mass=3.2,
        origin=Origin(xyz=(0.150, 0.0, 0.0)),
    )

    model.articulation(
        "bracket_mount",
        ArticulationType.FIXED,
        parent=column,
        child=side_arm_bracket,
        origin=Origin(xyz=(0.0, 0.0, 4.224)),
    )
    model.articulation(
        "camera_pan",
        ArticulationType.REVOLUTE,
        parent=side_arm_bracket,
        child=pan_pod,
        origin=Origin(xyz=(0.560, 0.0, 0.100)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.8,
            lower=-1.45,
            upper=1.45,
        ),
    )
    model.articulation(
        "camera_tilt",
        ArticulationType.REVOLUTE,
        parent=pan_pod,
        child=camera_head,
        origin=Origin(xyz=(0.044, 0.0, -0.153)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.6,
            lower=-0.45,
            upper=0.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    column = object_model.get_part("column")
    side_arm_bracket = object_model.get_part("side_arm_bracket")
    pan_pod = object_model.get_part("pan_pod")
    camera_head = object_model.get_part("camera_head")

    pan_joint = object_model.get_articulation("camera_pan")
    tilt_joint = object_model.get_articulation("camera_tilt")

    base_plate = column.get_visual("base_plate")
    pole_shaft = column.get_visual("pole_shaft")
    bracket_sleeve = side_arm_bracket.get_visual("bracket_sleeve")
    side_arm = side_arm_bracket.get_visual("side_arm")
    pan_disc = pan_pod.get_visual("pan_disc")
    left_yoke = pan_pod.get_visual("left_yoke")
    tilt_axle = camera_head.get_visual("tilt_axle")
    camera_body = camera_head.get_visual("camera_body")

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
        max_pose_samples=20,
        ignore_adjacent=False,
        ignore_fixed=True,
    )
    ctx.fail_if_isolated_parts(
        max_pose_samples=20,
        name="motion_sweep_no_floating",
    )

    ctx.check(
        "camera_pan_axis_vertical",
        tuple(round(value, 6) for value in pan_joint.axis) == (0.0, 0.0, 1.0),
        f"camera_pan axis is {pan_joint.axis}",
    )
    ctx.check(
        "camera_tilt_axis_horizontal",
        tuple(round(value, 6) for value in tilt_joint.axis) == (0.0, 1.0, 0.0),
        f"camera_tilt axis is {tilt_joint.axis}",
    )

    pan_limits = pan_joint.motion_limits
    tilt_limits = tilt_joint.motion_limits
    ctx.check(
        "camera_pan_limits_realistic",
        pan_limits is not None
        and pan_limits.lower is not None
        and pan_limits.upper is not None
        and pan_limits.lower < 0.0 < pan_limits.upper
        and pan_limits.upper <= 1.50,
        f"camera_pan limits are {pan_limits}",
    )
    ctx.check(
        "camera_tilt_limits_realistic",
        tilt_limits is not None
        and tilt_limits.lower is not None
        and tilt_limits.upper is not None
        and tilt_limits.lower < 0.0 < tilt_limits.upper
        and tilt_limits.lower >= -1.10
        and tilt_limits.upper <= 0.60,
        f"camera_tilt limits are {tilt_limits}",
    )

    with ctx.pose({pan_joint: 0.0, tilt_joint: 0.0}):
        ctx.expect_contact(
            column,
            side_arm_bracket,
            elem_a=pole_shaft,
            elem_b=bracket_sleeve,
            name="bracket_sleeve_contacts_pole",
        )
        ctx.expect_contact(
            side_arm_bracket,
            pan_pod,
            elem_a=side_arm,
            elem_b=pan_disc,
            name="pan_disc_under_arm",
        )
        ctx.expect_contact(
            pan_pod,
            camera_head,
            elem_a=left_yoke,
            elem_b=tilt_axle,
            name="tilt_axle_in_yoke",
        )
        ctx.expect_gap(
            camera_head,
            column,
            axis="x",
            min_gap=0.24,
            name="camera_clear_of_pole_in_front",
        )
        ctx.expect_overlap(
            pan_pod,
            camera_head,
            axes="yz",
            min_overlap=0.05,
            name="camera_aligned_with_pan_yoke",
        )
        ctx.expect_origin_gap(
            side_arm_bracket,
            column,
            axis="z",
            min_gap=4.20,
            max_gap=4.24,
            name="bracket_at_column_top",
        )

        base_plate_aabb = ctx.part_element_world_aabb(column, elem=base_plate)
        if base_plate_aabb is None:
            ctx.fail("square_flange_base_size", "base plate AABB unavailable")
        else:
            plate_dx = base_plate_aabb[1][0] - base_plate_aabb[0][0]
            plate_dy = base_plate_aabb[1][1] - base_plate_aabb[0][1]
            ctx.check(
                "square_flange_base_size",
                0.41 <= plate_dx <= 0.43
                and 0.41 <= plate_dy <= 0.43
                and abs(plate_dx - plate_dy) <= 0.003,
                f"base plate size is {(plate_dx, plate_dy)}",
            )

        pole_aabb = ctx.part_element_world_aabb(column, elem=pole_shaft)
        if pole_aabb is None:
            ctx.fail("pole_height_realistic", "pole shaft AABB unavailable")
        else:
            pole_height = pole_aabb[1][2] - pole_aabb[0][2]
            ctx.check(
                "pole_height_realistic",
                4.15 <= pole_height <= 4.25,
                f"pole height is {pole_height}",
            )

        camera_body_aabb = ctx.part_element_world_aabb(camera_head, elem=camera_body)
        if camera_body_aabb is None:
            ctx.fail("bullet_camera_body_proportions", "camera body AABB unavailable")
        else:
            body_dx = camera_body_aabb[1][0] - camera_body_aabb[0][0]
            body_dy = camera_body_aabb[1][1] - camera_body_aabb[0][1]
            body_dz = camera_body_aabb[1][2] - camera_body_aabb[0][2]
            ctx.check(
                "bullet_camera_body_proportions",
                0.21 <= body_dx <= 0.23 and 0.07 <= max(body_dy, body_dz) <= 0.09,
                f"camera body size is {(body_dx, body_dy, body_dz)}",
            )

        column_aabb = ctx.part_world_aabb(column)
        camera_aabb = ctx.part_world_aabb(camera_head)
        if column_aabb is None or camera_aabb is None:
            ctx.fail("overall_height_realistic", "world AABB unavailable")
        else:
            total_height = max(column_aabb[1][2], camera_aabb[1][2])
            ctx.check(
                "overall_height_realistic",
                4.20 <= total_height <= 4.65,
                f"overall height is {total_height}",
            )

    if pan_limits is not None and pan_limits.lower is not None and pan_limits.upper is not None:
        with ctx.pose({pan_joint: pan_limits.lower, tilt_joint: 0.0}):
            ctx.fail_if_parts_overlap_in_current_pose(name="camera_pan_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="camera_pan_lower_no_floating")
            ctx.expect_contact(
                side_arm_bracket,
                pan_pod,
                elem_a=side_arm,
                elem_b=pan_disc,
                name="camera_pan_lower_mount_contact",
            )
        with ctx.pose({pan_joint: pan_limits.upper, tilt_joint: 0.0}):
            ctx.fail_if_parts_overlap_in_current_pose(name="camera_pan_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="camera_pan_upper_no_floating")
            ctx.expect_contact(
                side_arm_bracket,
                pan_pod,
                elem_a=side_arm,
                elem_b=pan_disc,
                name="camera_pan_upper_mount_contact",
            )

    if tilt_limits is not None and tilt_limits.lower is not None and tilt_limits.upper is not None:
        with ctx.pose({pan_joint: 0.0, tilt_joint: tilt_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="camera_tilt_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="camera_tilt_lower_no_floating")
            ctx.expect_contact(
                pan_pod,
                camera_head,
                elem_a=left_yoke,
                elem_b=tilt_axle,
                name="camera_tilt_lower_mount_contact",
            )
        with ctx.pose({pan_joint: 0.0, tilt_joint: tilt_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="camera_tilt_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="camera_tilt_upper_no_floating")
            ctx.expect_contact(
                pan_pod,
                camera_head,
                elem_a=left_yoke,
                elem_b=tilt_axle,
                name="camera_tilt_upper_mount_contact",
            )
        with ctx.pose({pan_joint: pan_limits.upper if pan_limits is not None and pan_limits.upper is not None else 0.0, tilt_joint: tilt_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="camera_combined_extreme_no_overlap")
            ctx.fail_if_isolated_parts(name="camera_combined_extreme_no_floating")

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
