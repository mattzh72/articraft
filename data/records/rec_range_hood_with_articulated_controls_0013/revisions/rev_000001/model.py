from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chimney_range_hood", assets=ASSETS)

    stainless = model.material("stainless", rgba=(0.79, 0.80, 0.81, 1.0))
    brushed_shadow = model.material("brushed_shadow", rgba=(0.66, 0.67, 0.69, 1.0))
    black_glass = model.material("black_glass", rgba=(0.12, 0.12, 0.13, 1.0))
    filter_steel = model.material("filter_steel", rgba=(0.52, 0.54, 0.56, 1.0))
    light_lens = model.material("light_lens", rgba=(0.92, 0.93, 0.88, 0.75))
    knob_black = model.material("knob_black", rgba=(0.10, 0.10, 0.11, 1.0))
    knob_marker = model.material("knob_marker", rgba=(0.86, 0.86, 0.88, 1.0))

    body_width = 0.90
    body_depth = 0.52
    canopy_top_z = 0.18
    rear_bottom_z = 0.10
    front_bottom_z = 0.00
    shell_t = 0.015
    front_y = body_depth * 0.5
    rear_y = -body_depth * 0.5
    inner_width = body_width - (2.0 * shell_t)
    inner_depth = body_depth - (2.0 * shell_t)
    chimney_outer = 0.28
    chimney_t = 0.012
    chimney_height = 0.72
    chimney_center_y = -0.08

    def _save_mesh(filename: str, geometry):
        return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))

    def _side_shell_mesh(filename: str, x_min: float, x_max: float):
        outline = [
            (rear_y, rear_bottom_z),
            (rear_y, canopy_top_z),
            (front_y, canopy_top_z),
            (front_y, front_bottom_z),
        ]
        return _save_mesh(
            filename,
            ExtrudeGeometry.from_z0(outline, shell_t, cap=True, closed=True)
            .rotate_x(math.pi * 0.5)
            .rotate_z(math.pi * 0.5)
            .translate(x_min, 0.0, 0.0),
        )

    hood_body = model.part("hood_body")
    hood_body.visual(
        Box((inner_width, inner_depth, shell_t)),
        origin=Origin(xyz=(0.0, 0.0, canopy_top_z - (shell_t * 0.5))),
        material=stainless,
        name="canopy_top",
    )
    hood_body.visual(
        _side_shell_mesh("range_hood_left_side.obj", -body_width * 0.5, -(body_width * 0.5) + shell_t),
        material=stainless,
        name="left_side",
    )
    hood_body.visual(
        _side_shell_mesh("range_hood_right_side.obj", (body_width * 0.5) - shell_t, body_width * 0.5),
        material=stainless,
        name="right_side",
    )
    hood_body.visual(
        Box((inner_width, shell_t, canopy_top_z - rear_bottom_z)),
        origin=Origin(
            xyz=(
                0.0,
                rear_y + (shell_t * 0.5),
                rear_bottom_z + ((canopy_top_z - rear_bottom_z) * 0.5),
            )
        ),
        material=brushed_shadow,
        name="back_panel",
    )

    control_strip_width = 0.11
    main_front_width = inner_width - control_strip_width
    hood_body.visual(
        Box((main_front_width, shell_t, canopy_top_z - front_bottom_z)),
        origin=Origin(
            xyz=(
                -control_strip_width * 0.5,
                front_y - (shell_t * 0.5),
                (canopy_top_z - front_bottom_z) * 0.5,
            )
        ),
        material=stainless,
        name="front_main",
    )
    hood_body.visual(
        Box((control_strip_width, shell_t, canopy_top_z - front_bottom_z)),
        origin=Origin(
            xyz=(
                (inner_width * 0.5) - (control_strip_width * 0.5),
                front_y - (shell_t * 0.5),
                (canopy_top_z - front_bottom_z) * 0.5,
            )
        ),
        material=black_glass,
        name="front_controls",
    )
    hood_body.visual(
        Box((0.31, 0.20, 0.008)),
        origin=Origin(xyz=(-0.165, 0.075, canopy_top_z - shell_t - 0.004)),
        material=filter_steel,
        name="left_filter",
    )
    hood_body.visual(
        Box((0.31, 0.20, 0.008)),
        origin=Origin(xyz=(0.165, 0.075, canopy_top_z - shell_t - 0.004)),
        material=filter_steel,
        name="right_filter",
    )
    hood_body.visual(
        Box((0.18, 0.022, 0.006)),
        origin=Origin(xyz=(0.0, 0.165, canopy_top_z - shell_t - 0.003)),
        material=light_lens,
        name="task_light",
    )

    chimney_panel_span = chimney_outer - (2.0 * chimney_t)
    chimney_center_z = canopy_top_z + (chimney_height * 0.5)
    hood_body.visual(
        Box((chimney_panel_span, chimney_t, chimney_height)),
        origin=Origin(
            xyz=(
                0.0,
                chimney_center_y + (chimney_outer * 0.5) - (chimney_t * 0.5),
                chimney_center_z,
            )
        ),
        material=stainless,
        name="chimney_front",
    )
    hood_body.visual(
        Box((chimney_panel_span, chimney_t, chimney_height)),
        origin=Origin(
            xyz=(
                0.0,
                chimney_center_y - (chimney_outer * 0.5) + (chimney_t * 0.5),
                chimney_center_z,
            )
        ),
        material=stainless,
        name="chimney_back",
    )
    hood_body.visual(
        Box((chimney_t, chimney_panel_span, chimney_height)),
        origin=Origin(
            xyz=(
                (chimney_outer * 0.5) - (chimney_t * 0.5),
                chimney_center_y,
                chimney_center_z,
            )
        ),
        material=stainless,
        name="chimney_right",
    )
    hood_body.visual(
        Box((chimney_t, chimney_panel_span, chimney_height)),
        origin=Origin(
            xyz=(
                -(chimney_outer * 0.5) + (chimney_t * 0.5),
                chimney_center_y,
                chimney_center_z,
            )
        ),
        material=stainless,
        name="chimney_left",
    )
    hood_body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, canopy_top_z + chimney_height)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, (canopy_top_z + chimney_height) * 0.5)),
    )

    knob_axis_roll = -math.pi * 0.5
    knob_x = (inner_width * 0.5) - (control_strip_width * 0.5)

    def _add_knob(part_name: str, joint_name: str, z_pos: float) -> None:
        knob = model.part(part_name)
        knob.visual(
            Cylinder(radius=0.019, length=0.008),
            origin=Origin(xyz=(0.0, 0.004, 0.0), rpy=(knob_axis_roll, 0.0, 0.0)),
            material=knob_black,
            name="mount_flange",
        )
        knob.visual(
            Cylinder(radius=0.017, length=0.018),
            origin=Origin(xyz=(0.0, 0.017, 0.0), rpy=(knob_axis_roll, 0.0, 0.0)),
            material=knob_black,
            name="knob_grip",
        )
        knob.visual(
            Cylinder(radius=0.006, length=0.006),
            origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(knob_axis_roll, 0.0, 0.0)),
            material=knob_black,
            name="center_hub",
        )
        knob.visual(
            Box((0.003, 0.003, 0.009)),
            origin=Origin(xyz=(0.0, 0.0275, 0.012)),
            material=knob_marker,
            name="pointer_mark",
        )
        knob.inertial = Inertial.from_geometry(
            Box((0.042, 0.032, 0.042)),
            mass=0.06,
            origin=Origin(xyz=(0.0, 0.017, 0.0)),
        )
        model.articulation(
            joint_name,
            ArticulationType.CONTINUOUS,
            parent=hood_body,
            child=knob,
            origin=Origin(xyz=(knob_x, front_y, z_pos)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.25, velocity=8.0),
        )

    _add_knob("knob_upper", "hood_to_knob_upper", 0.130)
    _add_knob("knob_middle", "hood_to_knob_middle", 0.090)
    _add_knob("knob_lower", "hood_to_knob_lower", 0.050)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    hood_body = object_model.get_part("hood_body")
    knob_upper = object_model.get_part("knob_upper")
    knob_middle = object_model.get_part("knob_middle")
    knob_lower = object_model.get_part("knob_lower")
    upper_joint = object_model.get_articulation("hood_to_knob_upper")
    middle_joint = object_model.get_articulation("hood_to_knob_middle")
    lower_joint = object_model.get_articulation("hood_to_knob_lower")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

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
        "only_three_articulations",
        len(getattr(object_model, "articulations", ())) == 3,
        details=f"expected exactly 3 articulations, found {len(getattr(object_model, 'articulations', ()))}",
    )

    expected_visuals = (
        "canopy_top",
        "left_side",
        "right_side",
        "back_panel",
        "front_main",
        "front_controls",
        "left_filter",
        "right_filter",
        "task_light",
        "chimney_front",
        "chimney_back",
        "chimney_left",
        "chimney_right",
    )
    ctx.check(
        "hood_body_visuals_present",
        all(hood_body.get_visual(name) is not None for name in expected_visuals),
        details="missing one or more named hood-body visuals",
    )

    for joint_name, joint in (
        ("upper", upper_joint),
        ("middle", middle_joint),
        ("lower", lower_joint),
    ):
        limits = joint.motion_limits
        axis_ok = all(abs(a - b) < 1e-9 for a, b in zip(joint.axis, (0.0, 1.0, 0.0)))
        continuous_ok = joint.articulation_type == ArticulationType.CONTINUOUS
        unbounded_ok = limits is not None and limits.lower is None and limits.upper is None
        ctx.check(
            f"{joint_name}_knob_joint_type",
            continuous_ok,
            details=f"{joint.name} should be a continuous rotary joint",
        )
        ctx.check(
            f"{joint_name}_knob_axis_front_to_back",
            axis_ok,
            details=f"{joint.name} axis should be world +Y, got {joint.axis}",
        )
        ctx.check(
            f"{joint_name}_knob_joint_unbounded",
            unbounded_ok,
            details=f"{joint.name} should not define lower/upper limits",
        )

    for knob_name, knob in (
        ("upper", knob_upper),
        ("middle", knob_middle),
        ("lower", knob_lower),
    ):
        ctx.expect_contact(
            knob,
            hood_body,
            elem_a="mount_flange",
            elem_b="front_controls",
            name=f"{knob_name}_knob_front_mount_contact",
        )
        ctx.expect_within(
            knob,
            hood_body,
            axes="xz",
            margin=0.0,
            outer_elem="front_controls",
            name=f"{knob_name}_knob_within_control_strip",
        )
        ctx.expect_overlap(
            knob,
            hood_body,
            axes="xz",
            min_overlap=0.030,
            elem_b="front_controls",
            name=f"{knob_name}_knob_overlaps_control_strip",
        )

    ctx.expect_origin_distance(
        knob_upper,
        knob_middle,
        axes="x",
        max_dist=0.001,
        name="upper_middle_knobs_same_column_x",
    )
    ctx.expect_origin_distance(
        knob_middle,
        knob_lower,
        axes="x",
        max_dist=0.001,
        name="middle_lower_knobs_same_column_x",
    )
    ctx.expect_origin_gap(
        knob_upper,
        knob_middle,
        axis="z",
        min_gap=0.035,
        max_gap=0.045,
        name="upper_above_middle",
    )
    ctx.expect_origin_gap(
        knob_middle,
        knob_lower,
        axis="z",
        min_gap=0.035,
        max_gap=0.045,
        name="middle_above_lower",
    )

    hood_body_aabb = ctx.part_world_aabb(hood_body)
    canopy_top_aabb = ctx.part_element_world_aabb(hood_body, elem="canopy_top")
    chimney_front_aabb = ctx.part_element_world_aabb(hood_body, elem="chimney_front")
    if hood_body_aabb is None or canopy_top_aabb is None or chimney_front_aabb is None:
        ctx.fail("hood_body_measurements_available", "could not measure hood body or named visuals")
    else:
        width = hood_body_aabb[1][0] - hood_body_aabb[0][0]
        depth = hood_body_aabb[1][1] - hood_body_aabb[0][1]
        height = hood_body_aabb[1][2] - hood_body_aabb[0][2]
        ctx.check(
            "range_hood_width_realistic",
            0.84 <= width <= 0.94,
            details=f"hood width should read as a 90 cm appliance, got {width:.3f} m",
        )
        ctx.check(
            "range_hood_depth_realistic",
            0.49 <= depth <= 0.55,
            details=f"hood depth should read as a deep canopy, got {depth:.3f} m",
        )
        ctx.check(
            "range_hood_height_realistic",
            0.86 <= height <= 0.92,
            details=f"hood height should include the chimney column, got {height:.3f} m",
        )
        ctx.check(
            "chimney_starts_above_canopy",
            chimney_front_aabb[0][2] >= canopy_top_aabb[1][2] - 1e-6,
            details="chimney column should rise from the canopy top without sinking below it",
        )

    pose_checks = (
        ("knobs_pose_neutral", {upper_joint: 0.0, middle_joint: 0.0, lower_joint: 0.0}),
        ("knobs_pose_offset_a", {upper_joint: 1.2, middle_joint: -0.7, lower_joint: 2.4}),
        ("knobs_pose_offset_b", {upper_joint: -2.1, middle_joint: 2.0, lower_joint: -1.3}),
    )
    for pose_name, pose_map in pose_checks:
        with ctx.pose(pose_map):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{pose_name}_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{pose_name}_no_floating")
            ctx.expect_contact(
                knob_upper,
                hood_body,
                elem_a="mount_flange",
                elem_b="front_controls",
                name=f"{pose_name}_upper_mount_contact",
            )
            ctx.expect_contact(
                knob_middle,
                hood_body,
                elem_a="mount_flange",
                elem_b="front_controls",
                name=f"{pose_name}_middle_mount_contact",
            )
            ctx.expect_contact(
                knob_lower,
                hood_body,
                elem_a="mount_flange",
                elem_b="front_controls",
                name=f"{pose_name}_lower_mount_contact",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
