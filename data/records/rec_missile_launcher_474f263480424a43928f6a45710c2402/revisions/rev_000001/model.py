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


def _add_box(
    part,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    *,
    material,
    name: str,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=center, rpy=rpy),
        material=material,
        name=name,
    )


def _add_x_cylinder(
    part,
    radius: float,
    length: float,
    center: tuple[float, float, float],
    *,
    material,
    name: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_missile_launcher")

    base_paint = model.material("base_paint", rgba=(0.28, 0.33, 0.27, 1.0))
    pod_paint = model.material("pod_paint", rgba=(0.34, 0.39, 0.30, 1.0))
    steel = model.material("steel", rgba=(0.39, 0.41, 0.42, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.20, 1.0))
    blast_dark = model.material("blast_dark", rgba=(0.10, 0.11, 0.11, 1.0))

    base = model.part("base")
    _add_box(
        base,
        (1.80, 1.60, 0.16),
        (0.0, 0.0, 0.08),
        material=base_paint,
        name="base_plate",
    )
    _add_box(
        base,
        (1.00, 0.92, 0.48),
        (0.0, 0.0, 0.40),
        material=base_paint,
        name="pedestal_lower",
    )
    _add_box(
        base,
        (0.78, 0.78, 0.62),
        (0.0, 0.0, 0.95),
        material=base_paint,
        name="pedestal_column",
    )
    base.visual(
        Cylinder(radius=0.56, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 1.30)),
        material=steel,
        name="pedestal_race",
    )
    _add_box(
        base,
        (0.42, 0.30, 0.28),
        (0.0, -0.61, 0.30),
        material=dark_steel,
        name="service_housing",
    )
    base.inertial = Inertial.from_geometry(
        Box((1.80, 1.60, 1.34)),
        mass=3500.0,
        origin=Origin(xyz=(0.0, 0.0, 0.67)),
    )

    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.62, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=steel,
        name="turntable_ring",
    )
    _add_box(
        turntable,
        (1.96, 0.94, 0.30),
        (0.0, -0.02, 0.15),
        material=base_paint,
        name="turret_body",
    )
    _add_box(
        turntable,
        (0.66, 0.44, 0.28),
        (0.0, -0.34, 0.44),
        material=dark_steel,
        name="counterweight_box",
    )
    _add_box(
        turntable,
        (0.18, 0.22, 0.78),
        (-0.84, 0.0, 0.51),
        material=base_paint,
        name="left_support",
    )
    _add_box(
        turntable,
        (0.18, 0.22, 0.78),
        (0.84, 0.0, 0.51),
        material=base_paint,
        name="right_support",
    )
    _add_box(
        turntable,
        (0.22, 0.14, 0.12),
        (-0.76, 0.0, 0.86),
        material=steel,
        name="left_bearing",
    )
    _add_box(
        turntable,
        (0.22, 0.14, 0.12),
        (0.76, 0.0, 0.86),
        material=steel,
        name="right_bearing",
    )
    turntable.inertial = Inertial.from_geometry(
        Box((2.00, 1.20, 1.02)),
        mass=1200.0,
        origin=Origin(xyz=(0.0, -0.06, 0.51)),
    )

    cradle = model.part("cradle")
    _add_x_cylinder(
        cradle,
        0.06,
        0.08,
        (-0.61, 0.0, 0.0),
        material=steel,
        name="left_trunnion",
    )
    _add_x_cylinder(
        cradle,
        0.06,
        0.08,
        (0.61, 0.0, 0.0),
        material=steel,
        name="right_trunnion",
    )
    _add_box(
        cradle,
        (0.12, 0.16, 0.60),
        (-0.51, 0.02, 0.12),
        material=steel,
        name="left_cheek",
    )
    _add_box(
        cradle,
        (0.12, 0.16, 0.60),
        (0.51, 0.02, 0.12),
        material=steel,
        name="right_cheek",
    )
    _add_box(
        cradle,
        (0.98, 0.12, 0.08),
        (0.0, 0.10, 0.20),
        material=steel,
        name="rear_mount_beam",
    )
    _add_box(
        cradle,
        (0.12, 1.40, 0.08),
        (-0.52, 0.79, 0.42),
        material=steel,
        name="left_rail",
    )
    _add_box(
        cradle,
        (0.12, 1.40, 0.08),
        (0.52, 0.79, 0.42),
        material=steel,
        name="right_rail",
    )
    _add_box(
        cradle,
        (0.98, 0.12, 0.08),
        (0.0, 0.80, 0.42),
        material=dark_steel,
        name="mid_tie",
    )
    _add_box(
        cradle,
        (0.98, 0.12, 0.08),
        (0.0, 1.46, 0.42),
        material=dark_steel,
        name="front_tie",
    )
    cradle.inertial = Inertial.from_geometry(
        Box((1.20, 1.60, 0.40)),
        mass=650.0,
        origin=Origin(xyz=(0.0, 0.78, 0.16)),
    )

    launch_pod = model.part("launch_pod")
    pod_width = 1.24
    pod_height = 0.80
    pod_y_min = 0.00
    pod_y_max = 1.80
    pod_depth = pod_y_max - pod_y_min
    pod_z_min = 0.00
    pod_z_max = pod_z_min + pod_height
    shell = 0.04
    divider = 0.03
    front_frame_depth = 0.06
    inner_width = pod_width - 2.0 * shell
    inner_height = pod_height - 2.0 * shell
    front_frame_center_y = pod_y_max - front_frame_depth / 2.0
    rear_wall_center_y = pod_y_min + shell / 2.0
    divider_center_y = (rear_wall_center_y + shell / 2.0 + front_frame_center_y - front_frame_depth / 2.0) / 2.0
    divider_length = (front_frame_center_y - front_frame_depth / 2.0) - (rear_wall_center_y + shell / 2.0)
    opening_centers_x = (-0.40, 0.0, 0.40)
    opening_centers_z = (0.20, 0.56)

    _add_box(
        launch_pod,
        (pod_width, pod_depth, shell),
        (0.0, (pod_y_min + pod_y_max) / 2.0, pod_z_min + shell / 2.0),
        material=pod_paint,
        name="bottom_panel",
    )
    _add_box(
        launch_pod,
        (pod_width, pod_depth, shell),
        (0.0, (pod_y_min + pod_y_max) / 2.0, pod_z_max - shell / 2.0),
        material=pod_paint,
        name="top_panel",
    )
    _add_box(
        launch_pod,
        (shell, pod_depth, inner_height),
        (-(pod_width / 2.0) + shell / 2.0, (pod_y_min + pod_y_max) / 2.0, (pod_z_min + pod_z_max) / 2.0),
        material=pod_paint,
        name="left_wall",
    )
    _add_box(
        launch_pod,
        (shell, pod_depth, inner_height),
        ((pod_width / 2.0) - shell / 2.0, (pod_y_min + pod_y_max) / 2.0, (pod_z_min + pod_z_max) / 2.0),
        material=pod_paint,
        name="right_wall",
    )
    _add_box(
        launch_pod,
        (inner_width, shell, inner_height),
        (0.0, rear_wall_center_y, (pod_z_min + pod_z_max) / 2.0),
        material=pod_paint,
        name="rear_wall",
    )
    _add_box(
        launch_pod,
        (inner_width, front_frame_depth, 0.10),
        (0.0, front_frame_center_y, pod_z_max - 0.05),
        material=pod_paint,
        name="front_top_bar",
    )
    _add_box(
        launch_pod,
        (inner_width, front_frame_depth, 0.10),
        (0.0, front_frame_center_y, pod_z_min + 0.05),
        material=pod_paint,
        name="front_bottom_bar",
    )
    _add_box(
        launch_pod,
        (0.06, front_frame_depth, inner_height),
        (-0.25, front_frame_center_y, (pod_z_min + pod_z_max) / 2.0),
        material=pod_paint,
        name="front_left_separator",
    )
    _add_box(
        launch_pod,
        (0.06, front_frame_depth, inner_height),
        (0.25, front_frame_center_y, (pod_z_min + pod_z_max) / 2.0),
        material=pod_paint,
        name="front_right_separator",
    )
    _add_box(
        launch_pod,
        (inner_width, front_frame_depth, 0.06),
        (0.0, front_frame_center_y, (pod_z_min + pod_z_max) / 2.0),
        material=pod_paint,
        name="front_mid_separator",
    )
    _add_box(
        launch_pod,
        (divider, divider_length, inner_height),
        (-0.25, divider_center_y, (pod_z_min + pod_z_max) / 2.0),
        material=pod_paint,
        name="vertical_divider_left",
    )
    _add_box(
        launch_pod,
        (divider, divider_length, inner_height),
        (0.25, divider_center_y, (pod_z_min + pod_z_max) / 2.0),
        material=pod_paint,
        name="vertical_divider_right",
    )
    _add_box(
        launch_pod,
        (inner_width, divider_length, divider),
        (0.0, divider_center_y, (pod_z_min + pod_z_max) / 2.0),
        material=pod_paint,
        name="horizontal_divider",
    )
    _add_box(
        launch_pod,
        (0.10, 0.18, 0.04),
        (-0.34, -0.08, pod_z_max + 0.02),
        material=steel,
        name="left_lifting_lug",
    )
    _add_box(
        launch_pod,
        (0.10, 0.18, 0.04),
        (0.34, -0.08, pod_z_max + 0.02),
        material=steel,
        name="right_lifting_lug",
    )

    for row_index, center_z in enumerate(opening_centers_z):
        for col_index, center_x in enumerate(opening_centers_x):
            _add_box(
                launch_pod,
                (0.30, 0.04, 0.28),
                (center_x, pod_y_min + 0.06, center_z),
                material=blast_dark,
                name=f"cell_back_{row_index}_{col_index}",
            )

    launch_pod.inertial = Inertial.from_geometry(
        Box((pod_width, pod_depth, pod_height + 0.04)),
        mass=950.0,
        origin=Origin(xyz=(0.0, (pod_y_min + pod_y_max) / 2.0, (pod_z_min + pod_z_max) / 2.0)),
    )

    model.articulation(
        "base_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 1.34)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=9000.0,
            velocity=0.8,
            lower=-math.radians(160.0),
            upper=math.radians(160.0),
        ),
    )
    model.articulation(
        "cradle_pitch",
        ArticulationType.REVOLUTE,
        parent=turntable,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.86)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6000.0,
            velocity=0.6,
            lower=-math.radians(10.0),
            upper=math.radians(65.0),
        ),
    )
    model.articulation(
        "cradle_to_launch_pod",
        ArticulationType.FIXED,
        parent=cradle,
        child=launch_pod,
        origin=Origin(xyz=(0.0, 0.0, 0.46)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    turntable = object_model.get_part("turntable")
    cradle = object_model.get_part("cradle")
    launch_pod = object_model.get_part("launch_pod")
    base_yaw = object_model.get_articulation("base_yaw")
    cradle_pitch = object_model.get_articulation("cradle_pitch")

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

    ctx.fail_if_isolated_parts(max_pose_samples=8, name="sampled_pose_no_floating")
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=24)

    ctx.expect_contact(turntable, base, elem_a="turntable_ring", elem_b="pedestal_race")
    ctx.expect_contact(cradle, turntable, elem_a="left_trunnion", elem_b="left_bearing")
    ctx.expect_contact(cradle, turntable, elem_a="right_trunnion", elem_b="right_bearing")
    ctx.expect_contact(launch_pod, cradle, elem_a="bottom_panel", elem_b="left_rail")
    ctx.expect_contact(launch_pod, cradle, elem_a="bottom_panel", elem_b="right_rail")

    ctx.expect_origin_distance(turntable, base, axes="xy", max_dist=0.001)
    ctx.expect_origin_distance(cradle, turntable, axes="xy", max_dist=0.001)
    ctx.expect_origin_gap(cradle, turntable, axis="z", min_gap=0.84, max_gap=0.88)
    ctx.expect_overlap(launch_pod, cradle, axes="xy", min_overlap=0.40)

    def aabb_center(aabb):
        return tuple((aabb[0][index] + aabb[1][index]) * 0.5 for index in range(3))

    base_aabb = ctx.part_world_aabb(base)
    pod_aabb = ctx.part_world_aabb(launch_pod)
    if base_aabb is None or pod_aabb is None:
        ctx.fail("launcher_aabbs_available", "Base or launch pod AABB unavailable.")
    else:
        overall_height = pod_aabb[1][2] - base_aabb[0][2]
        ctx.check(
            "launcher_height_realistic",
            2.8 <= overall_height <= 3.8,
            f"overall height = {overall_height:.3f} m",
        )

    front_rest_aabb = ctx.part_element_world_aabb(launch_pod, elem="front_top_bar")
    if front_rest_aabb is None:
        ctx.fail("front_top_bar_present", "Launch pod front_top_bar AABB unavailable.")
        front_rest_center = None
    else:
        front_rest_center = aabb_center(front_rest_aabb)

    with ctx.pose({base_yaw: math.radians(90.0)}):
        yaw_aabb = ctx.part_element_world_aabb(launch_pod, elem="front_top_bar")
        if yaw_aabb is None or front_rest_center is None:
            ctx.fail("yaw_pose_front_bar_available", "Yaw-pose front_top_bar AABB unavailable.")
        else:
            yaw_center = aabb_center(yaw_aabb)
            ctx.check(
                "yaw_rotates_launcher_face",
                abs(yaw_center[0]) > 0.80 and abs(yaw_center[1]) < 0.20,
                f"rest center={front_rest_center}, yaw center={yaw_center}",
            )

    pitch_limits = cradle_pitch.motion_limits
    if pitch_limits is not None and pitch_limits.lower is not None and pitch_limits.upper is not None:
        with ctx.pose({cradle_pitch: pitch_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="cradle_pitch_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="cradle_pitch_lower_no_floating")
            ctx.expect_gap(
                launch_pod,
                turntable,
                axis="z",
                min_gap=0.08,
                name="launch_pod_clears_turntable_at_depression",
            )
            lower_aabb = ctx.part_element_world_aabb(launch_pod, elem="front_top_bar")
            if lower_aabb is None or front_rest_center is None:
                ctx.fail("pitch_lower_front_bar_available", "Lower-pose front_top_bar AABB unavailable.")
            else:
                lower_center = aabb_center(lower_aabb)
                ctx.check(
                    "pitch_depression_lowers_launcher_face",
                    lower_center[2] < front_rest_center[2] - 0.10,
                    f"rest center={front_rest_center}, lower center={lower_center}",
                )

        with ctx.pose({cradle_pitch: pitch_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="cradle_pitch_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="cradle_pitch_upper_no_floating")
            upper_aabb = ctx.part_element_world_aabb(launch_pod, elem="front_top_bar")
            if upper_aabb is None or front_rest_center is None:
                ctx.fail("pitch_upper_front_bar_available", "Upper-pose front_top_bar AABB unavailable.")
            else:
                upper_center = aabb_center(upper_aabb)
                ctx.check(
                    "pitch_elevation_raises_launcher_face",
                    upper_center[2] > front_rest_center[2] + 0.60,
                    f"rest center={front_rest_center}, upper center={upper_center}",
                )

    yaw_limits = base_yaw.motion_limits
    if yaw_limits is not None and yaw_limits.lower is not None and yaw_limits.upper is not None:
        with ctx.pose({base_yaw: yaw_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="base_yaw_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="base_yaw_lower_no_floating")
        with ctx.pose({base_yaw: yaw_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="base_yaw_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="base_yaw_upper_no_floating")

    if pitch_limits is not None and pitch_limits.lower is not None:
        with ctx.pose({base_yaw: math.radians(90.0), cradle_pitch: pitch_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="combined_yaw_pitch_pose_no_overlap")
            ctx.fail_if_isolated_parts(name="combined_yaw_pitch_pose_no_floating")

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
