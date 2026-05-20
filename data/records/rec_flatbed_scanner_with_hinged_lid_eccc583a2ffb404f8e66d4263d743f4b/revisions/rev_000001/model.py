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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="legal_flatbed_scanner")

    body_color = model.material("body_color", rgba=(0.81, 0.83, 0.85, 1.0))
    trim_color = model.material("trim_color", rgba=(0.21, 0.23, 0.26, 1.0))
    lid_color = model.material("lid_color", rgba=(0.92, 0.93, 0.94, 1.0))
    glass_color = model.material("glass_color", rgba=(0.42, 0.60, 0.72, 0.45))
    guide_color = model.material("guide_color", rgba=(0.14, 0.15, 0.17, 1.0))

    body_w = 0.56
    body_d = 0.35
    body_h = 0.078
    body_r = 0.026

    glass_w = 0.406
    glass_d = 0.262
    glass_t = 0.003
    bezel_w = 0.48
    bezel_d = 0.304
    bezel_t = 0.004

    hinge_axis_y = 0.163
    hinge_axis_z = 0.088
    hinge_r = 0.010

    body = model.part("body")
    body_mesh = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(body_w, body_d, body_r), body_h),
        "scanner_body_shell",
    )
    body.visual(
        body_mesh,
        origin=Origin(xyz=(0.0, 0.0, body_h / 2.0)),
        material=body_color,
        name="body_shell",
    )

    bezel_side_w = (bezel_w - glass_w) / 2.0
    bezel_end_d = (bezel_d - glass_d) / 2.0
    bezel_z = body_h - bezel_t / 2.0

    body.visual(
        Box((bezel_side_w, bezel_d, bezel_t)),
        origin=Origin(xyz=(-(glass_w / 2.0 + bezel_side_w / 2.0), 0.0, bezel_z)),
        material=trim_color,
        name="left_bezel",
    )
    body.visual(
        Box((bezel_side_w, bezel_d, bezel_t)),
        origin=Origin(xyz=((glass_w / 2.0 + bezel_side_w / 2.0), 0.0, bezel_z)),
        material=trim_color,
        name="right_bezel",
    )
    body.visual(
        Box((glass_w, bezel_end_d, bezel_t)),
        origin=Origin(xyz=(0.0, -(glass_d / 2.0 + bezel_end_d / 2.0), bezel_z)),
        material=trim_color,
        name="front_bezel",
    )
    body.visual(
        Box((glass_w, bezel_end_d, bezel_t)),
        origin=Origin(xyz=(0.0, (glass_d / 2.0 + bezel_end_d / 2.0), bezel_z)),
        material=trim_color,
        name="rear_bezel",
    )
    body.visual(
        Box((glass_w, glass_d, glass_t)),
        origin=Origin(xyz=(0.0, 0.0, body_h - 0.0055)),
        material=glass_color,
        name="platen_glass",
    )

    rail_x = glass_w / 2.0 + 0.014
    rail_len = 0.228
    rail_h = 0.006
    body.visual(
        Box((0.010, rail_len, rail_h)),
        origin=Origin(xyz=(rail_x, 0.0, body_h + rail_h / 2.0 - 0.001)),
        material=guide_color,
        name="alignment_rail",
    )

    for side_name, sign in (("left", -1.0), ("right", 1.0)):
        pedestal_x = sign * 0.190
        body.visual(
            Box((0.060, 0.018, 0.014)),
            origin=Origin(xyz=(pedestal_x, hinge_axis_y, body_h + 0.007)),
            material=trim_color,
            name=f"{side_name}_hinge_pedestal",
        )
        body.visual(
            Cylinder(radius=hinge_r, length=0.060),
            origin=Origin(
                xyz=(pedestal_x, hinge_axis_y, hinge_axis_z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=trim_color,
            name=f"{side_name}_body_hinge_barrel",
        )

    body.inertial = Inertial.from_geometry(
        Box((body_w, body_d, hinge_axis_z + 0.018)),
        mass=6.8,
        origin=Origin(xyz=(0.0, 0.0, (hinge_axis_z + 0.018) / 2.0)),
    )

    lid = model.part("lid")
    lid_panel_mesh = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.544, 0.296, 0.020), 0.010),
        "scanner_lid_panel",
    )
    lid.visual(
        lid_panel_mesh,
        origin=Origin(xyz=(0.0, -0.168, 0.017)),
        material=lid_color,
        name="lid_panel",
    )
    lid.visual(
        Box((0.240, 0.024, 0.024)),
        origin=Origin(xyz=(0.0, -0.012, 0.008)),
        material=lid_color,
        name="rear_spine",
    )
    lid.visual(
        Box((0.460, 0.020, 0.024)),
        origin=Origin(xyz=(0.0, -0.306, 0.008)),
        material=lid_color,
        name="front_frame",
    )
    lid.visual(
        Box((0.020, 0.280, 0.024)),
        origin=Origin(xyz=(-0.262, -0.160, 0.008)),
        material=lid_color,
        name="left_frame",
    )
    lid.visual(
        Box((0.020, 0.280, 0.024)),
        origin=Origin(xyz=(0.262, -0.160, 0.008)),
        material=lid_color,
        name="right_frame",
    )
    lid.visual(
        Box((0.420, 0.016, 0.006)),
        origin=Origin(xyz=(0.0, -0.306, -0.007)),
        material=trim_color,
        name="front_pad",
    )
    lid.visual(
        Box((0.010, 0.220, 0.006)),
        origin=Origin(xyz=(-0.251, -0.160, -0.007)),
        material=trim_color,
        name="left_pad",
    )
    lid.visual(
        Box((0.034, 0.020, 0.020)),
        origin=Origin(xyz=(-0.135, -0.010, 0.006)),
        material=trim_color,
        name="left_hinge_mount",
    )
    lid.visual(
        Box((0.034, 0.020, 0.020)),
        origin=Origin(xyz=(0.135, -0.010, 0.006)),
        material=trim_color,
        name="right_hinge_mount",
    )
    lid.visual(
        Cylinder(radius=hinge_r, length=0.030),
        origin=Origin(
            xyz=(-0.145, 0.0, 0.006),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=trim_color,
        name="left_lid_hinge_barrel",
    )
    lid.visual(
        Cylinder(radius=hinge_r, length=0.030),
        origin=Origin(
            xyz=(0.145, 0.0, 0.006),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=trim_color,
        name="right_lid_hinge_barrel",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.544, 0.316, 0.032)),
        mass=2.4,
        origin=Origin(xyz=(0.0, -0.158, 0.008)),
    )

    alignment_bar = model.part("alignment_bar")
    alignment_bar.visual(
        Box((0.014, 0.090, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=guide_color,
        name="runner",
    )
    alignment_bar.visual(
        Box((0.014, 0.090, 0.005)),
        origin=Origin(xyz=(-0.012, 0.0, 0.0065)),
        material=guide_color,
        name="fence",
    )
    alignment_bar.inertial = Inertial.from_geometry(
        Box((0.028, 0.090, 0.012)),
        mass=0.12,
        origin=Origin(xyz=(-0.006, 0.0, 0.006)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(108.0),
        ),
    )
    model.articulation(
        "body_to_alignment_bar",
        ArticulationType.PRISMATIC,
        parent=body,
        child=alignment_bar,
        origin=Origin(xyz=(rail_x, 0.0, body_h + rail_h - 0.001)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=0.18,
            lower=-0.068,
            upper=0.068,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    alignment_bar = object_model.get_part("alignment_bar")
    lid_hinge = object_model.get_articulation("body_to_lid")
    slider = object_model.get_articulation("body_to_alignment_bar")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    body_aabb = ctx.part_world_aabb(body)
    assert body_aabb is not None
    body_size = (
        body_aabb[1][0] - body_aabb[0][0],
        body_aabb[1][1] - body_aabb[0][1],
        body_aabb[1][2] - body_aabb[0][2],
    )
    ctx.check(
        "scanner_body_realistic_size",
        0.50 <= body_size[0] <= 0.62
        and 0.31 <= body_size[1] <= 0.39
        and 0.07 <= body_size[2] <= 0.11,
        details=f"Unexpected body size {body_size}",
    )
    ctx.check(
        "lid_hinge_axis_widthwise",
        tuple(lid_hinge.axis) == (-1.0, 0.0, 0.0),
        details=f"Expected widthwise lid hinge axis, got {lid_hinge.axis}",
    )
    ctx.check(
        "alignment_bar_axis_depthwise",
        tuple(slider.axis) == (0.0, 1.0, 0.0),
        details=f"Expected depthwise slider axis, got {slider.axis}",
    )

    ctx.expect_contact(lid, body, elem_a="front_pad", elem_b="front_bezel")
    ctx.expect_contact(alignment_bar, body, elem_a="runner", elem_b="alignment_rail")
    ctx.expect_within(
        alignment_bar,
        body,
        axes="y",
        inner_elem="runner",
        outer_elem="alignment_rail",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_panel",
        elem_b="platen_glass",
        min_overlap=0.24,
    )

    lid_shell_rest = ctx.part_element_world_aabb(lid, elem="lid_panel")
    bar_rest = ctx.part_world_position(alignment_bar)
    assert lid_shell_rest is not None
    assert bar_rest is not None

    lid_limits = lid_hinge.motion_limits
    assert lid_limits is not None
    assert lid_limits.lower is not None
    assert lid_limits.upper is not None
    with ctx.pose({lid_hinge: lid_limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
        ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
        ctx.expect_contact(lid, body, elem_a="front_pad", elem_b="front_bezel")
    with ctx.pose({lid_hinge: lid_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
        ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")
        ctx.expect_contact(
            lid,
            body,
            elem_a="left_lid_hinge_barrel",
            elem_b="left_body_hinge_barrel",
        )
        lid_shell_open = ctx.part_element_world_aabb(lid, elem="lid_panel")
        assert lid_shell_open is not None
        ctx.check(
            "lid_opens_upward",
            lid_shell_open[1][2] > lid_shell_rest[1][2] + 0.16,
            details=(
                f"Lid panel max-z did not rise enough: rest={lid_shell_rest[1][2]}, "
                f"open={lid_shell_open[1][2]}"
            ),
        )

    slider_limits = slider.motion_limits
    assert slider_limits is not None
    assert slider_limits.lower is not None
    assert slider_limits.upper is not None
    with ctx.pose({slider: slider_limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="slider_lower_no_overlap")
        ctx.fail_if_isolated_parts(name="slider_lower_no_floating")
        ctx.expect_contact(alignment_bar, body, elem_a="runner", elem_b="alignment_rail")
        ctx.expect_within(
            alignment_bar,
            body,
            axes="y",
            inner_elem="runner",
            outer_elem="alignment_rail",
            name="slider_lower_within_rail",
        )
    with ctx.pose({slider: slider_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="slider_upper_no_overlap")
        ctx.fail_if_isolated_parts(name="slider_upper_no_floating")
        ctx.expect_contact(alignment_bar, body, elem_a="runner", elem_b="alignment_rail")
        ctx.expect_within(
            alignment_bar,
            body,
            axes="y",
            inner_elem="runner",
            outer_elem="alignment_rail",
            name="slider_upper_within_rail",
        )
        bar_upper = ctx.part_world_position(alignment_bar)
        assert bar_upper is not None
        ctx.check(
            "alignment_bar_slides_along_right_edge",
            abs(bar_upper[0] - bar_rest[0]) < 1e-6
            and abs(bar_upper[2] - bar_rest[2]) < 1e-6
            and bar_upper[1] > bar_rest[1] + 0.06,
            details=f"Unexpected slider motion rest={bar_rest}, upper={bar_upper}",
        )

    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
