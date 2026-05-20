from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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
    model = ArticulatedObject(name="photo_scanning_platform")

    body_color = model.material("body_color", rgba=(0.80, 0.82, 0.84, 1.0))
    trim_color = model.material("trim_color", rgba=(0.18, 0.19, 0.21, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.72, 0.74, 0.76, 1.0))
    tray_color = model.material("tray_color", rgba=(0.12, 0.13, 0.14, 1.0))
    glass_color = model.material("glass_color", rgba=(0.60, 0.78, 0.88, 0.35))
    foot_color = model.material("foot_color", rgba=(0.10, 0.10, 0.10, 1.0))

    body_w = 0.46
    body_d = 0.33
    body_h = 0.078
    core_h = 0.060
    bezel_h = body_h - core_h

    platen_w = 0.372
    platen_d = 0.254
    platen_t = 0.003
    platen_top = 0.069
    platen_z = platen_top - platen_t / 2.0

    border_x = (body_w - platen_w) / 2.0
    border_y = (body_d - platen_d) / 2.0

    hinge_radius = 0.0055
    hinge_y = -0.160
    hinge_z = body_h + hinge_radius
    hinge_segment_len = 0.072
    hinge_centers_base = (-0.164, 0.0, 0.164)
    hinge_centers_lid = (-0.082, 0.082)

    guide_rail_len = 0.340
    guide_rail_d = 0.010
    guide_rail_t = 0.002
    guide_rail_z = platen_top + guide_rail_t / 2.0
    guide_rail_y = 0.085

    tray_w = 0.150
    tray_d = 0.190
    tray_side_w = 0.008
    tray_bar_d = 0.010
    tray_t = 0.003
    tray_z = platen_top + guide_rail_t + tray_t / 2.0

    lid_w = 0.446
    lid_d = 0.320
    lid_t = 0.013
    lid_z = 0.0025
    lid_glass_t = 0.004
    lid_glass_w = lid_w - 0.036
    lid_glass_d = 0.274
    lid_side_d = lid_d - 0.018

    base = model.part("base")
    base.visual(
        Box((body_w, body_d, core_h)),
        origin=Origin(xyz=(0.0, 0.0, core_h / 2.0)),
        material=body_color,
        name="base_shell",
    )
    base.visual(
        Box((body_w, border_y, bezel_h)),
        origin=Origin(xyz=(0.0, body_d / 2.0 - border_y / 2.0, core_h + bezel_h / 2.0)),
        material=trim_color,
        name="front_bezel",
    )
    base.visual(
        Box((body_w, border_y, bezel_h)),
        origin=Origin(xyz=(0.0, -body_d / 2.0 + border_y / 2.0, core_h + bezel_h / 2.0)),
        material=trim_color,
        name="rear_bezel",
    )
    base.visual(
        Box((border_x, platen_d, bezel_h)),
        origin=Origin(xyz=(-body_w / 2.0 + border_x / 2.0, 0.0, core_h + bezel_h / 2.0)),
        material=trim_color,
        name="left_bezel",
    )
    base.visual(
        Box((border_x, platen_d, bezel_h)),
        origin=Origin(xyz=(body_w / 2.0 - border_x / 2.0, 0.0, core_h + bezel_h / 2.0)),
        material=trim_color,
        name="right_bezel",
    )
    base.visual(
        Box((platen_w, platen_d, platen_t)),
        origin=Origin(xyz=(0.0, 0.0, platen_z)),
        material=glass_color,
        name="platen_glass",
    )
    base.visual(
        Box((guide_rail_len, guide_rail_d, guide_rail_t)),
        origin=Origin(xyz=(0.0, guide_rail_y, guide_rail_z)),
        material=tray_color,
        name="guide_rail_front",
    )
    base.visual(
        Box((guide_rail_len, guide_rail_d, guide_rail_t)),
        origin=Origin(xyz=(0.0, -guide_rail_y, guide_rail_z)),
        material=tray_color,
        name="guide_rail_rear",
    )
    for idx, x_center in enumerate(hinge_centers_base, start=1):
        base.visual(
            Cylinder(radius=hinge_radius, length=hinge_segment_len),
            origin=Origin(xyz=(x_center, hinge_y, hinge_z), rpy=(0.0, 1.5707963267948966, 0.0)),
            material=hinge_metal,
            name=f"base_hinge_knuckle_{idx}",
        )
    foot_w = 0.040
    foot_d = 0.032
    foot_h = 0.006
    foot_x = body_w / 2.0 - 0.060
    foot_y = body_d / 2.0 - 0.050
    for idx, (x_sign, y_sign) in enumerate(
        ((-1.0, -1.0), (-1.0, 1.0), (1.0, -1.0), (1.0, 1.0)),
        start=1,
    ):
        base.visual(
            Box((foot_w, foot_d, foot_h)),
            origin=Origin(xyz=(x_sign * foot_x, y_sign * foot_y, -foot_h / 2.0)),
            material=foot_color,
            name=f"foot_{idx}",
        )
    base.inertial = Inertial.from_geometry(
        Box((body_w, body_d, body_h)),
        mass=4.6,
        origin=Origin(xyz=(0.0, 0.0, body_h / 2.0)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((lid_w, 0.016, lid_t)),
        origin=Origin(xyz=(0.0, 0.018, lid_z)),
        material=trim_color,
        name="lid_rear_frame",
    )
    lid.visual(
        Box((lid_w, 0.020, lid_t)),
        origin=Origin(xyz=(0.0, lid_d - 0.010, lid_z)),
        material=trim_color,
        name="lid_front_frame",
    )
    lid.visual(
        Box((0.018, lid_side_d, lid_t)),
        origin=Origin(xyz=(-lid_w / 2.0 + 0.009, 0.018 + lid_side_d / 2.0, lid_z)),
        material=trim_color,
        name="lid_left_frame",
    )
    lid.visual(
        Box((0.018, lid_side_d, lid_t)),
        origin=Origin(xyz=(lid_w / 2.0 - 0.009, 0.018 + lid_side_d / 2.0, lid_z)),
        material=trim_color,
        name="lid_right_frame",
    )
    lid.visual(
        Box((lid_glass_w, lid_glass_d, lid_glass_t)),
        origin=Origin(xyz=(0.0, 0.163, 0.007)),
        material=glass_color,
        name="lid_glass",
    )
    lid.visual(
        Box((hinge_segment_len, 0.010, 0.006)),
        origin=Origin(xyz=(hinge_centers_lid[0], 0.010, -0.001)),
        material=hinge_metal,
        name="lid_hinge_leaf_left",
    )
    lid.visual(
        Box((hinge_segment_len, 0.010, 0.006)),
        origin=Origin(xyz=(hinge_centers_lid[1], 0.010, -0.001)),
        material=hinge_metal,
        name="lid_hinge_leaf_right",
    )
    for idx, x_center in enumerate(hinge_centers_lid, start=1):
        lid.visual(
            Cylinder(radius=hinge_radius, length=hinge_segment_len),
            origin=Origin(xyz=(x_center, 0.0, 0.0), rpy=(0.0, 1.5707963267948966, 0.0)),
            material=hinge_metal,
            name=f"lid_hinge_knuckle_{idx}",
        )
    lid.visual(
        Box((0.020, 0.018, 0.0015)),
        origin=Origin(xyz=(-0.173, lid_d - 0.022, -0.00475)),
        material=foot_color,
        name="lid_pad_left",
    )
    lid.visual(
        Box((0.020, 0.018, 0.0015)),
        origin=Origin(xyz=(0.173, lid_d - 0.022, -0.00475)),
        material=foot_color,
        name="lid_pad_right",
    )
    lid.inertial = Inertial.from_geometry(
        Box((lid_w, lid_d, lid_t)),
        mass=1.1,
        origin=Origin(xyz=(0.0, lid_d / 2.0, lid_z)),
    )

    tray = model.part("tray")
    tray.visual(
        Box((tray_w, tray_bar_d, tray_t)),
        origin=Origin(xyz=(0.0, guide_rail_y, 0.0)),
        material=tray_color,
        name="tray_front_bar",
    )
    tray.visual(
        Box((tray_w, tray_bar_d, tray_t)),
        origin=Origin(xyz=(0.0, -guide_rail_y, 0.0)),
        material=tray_color,
        name="tray_rear_bar",
    )
    tray.visual(
        Box((tray_side_w, tray_d, tray_t)),
        origin=Origin(xyz=(-tray_w / 2.0 + tray_side_w / 2.0, 0.0, 0.0)),
        material=tray_color,
        name="tray_left_side",
    )
    tray.visual(
        Box((tray_side_w, tray_d, tray_t)),
        origin=Origin(xyz=(tray_w / 2.0 - tray_side_w / 2.0, 0.0, 0.0)),
        material=tray_color,
        name="tray_right_side",
    )
    tray.visual(
        Box((0.030, 0.016, 0.004)),
        origin=Origin(xyz=(0.0, tray_d / 2.0 - 0.010, 0.0005)),
        material=tray_color,
        name="tray_pull_tab",
    )
    tray.inertial = Inertial.from_geometry(
        Box((tray_w, tray_d, 0.004)),
        mass=0.10,
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=0.0, upper=1.25),
    )
    model.articulation(
        "tray_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=tray,
        origin=Origin(xyz=(0.0, 0.0, tray_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.25, lower=-0.090, upper=0.090),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    tray = object_model.get_part("tray")
    lid_hinge = object_model.get_articulation("lid_hinge")
    tray_slide = object_model.get_articulation("tray_slide")

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
        "lid_hinge_is_full_width_revolute",
        lid_hinge.articulation_type == ArticulationType.REVOLUTE
        and lid_hinge.axis == (1.0, 0.0, 0.0)
        and lid_hinge.motion_limits is not None
        and lid_hinge.motion_limits.lower == 0.0
        and lid_hinge.motion_limits.upper is not None
        and lid_hinge.motion_limits.upper >= 1.2,
        details=f"Unexpected lid hinge setup: type={lid_hinge.articulation_type}, axis={lid_hinge.axis}, limits={lid_hinge.motion_limits}",
    )
    ctx.check(
        "tray_uses_prismatic_guide_motion",
        tray_slide.articulation_type == ArticulationType.PRISMATIC
        and tray_slide.axis == (1.0, 0.0, 0.0)
        and tray_slide.motion_limits is not None
        and tray_slide.motion_limits.lower is not None
        and tray_slide.motion_limits.upper is not None
        and tray_slide.motion_limits.lower < 0.0 < tray_slide.motion_limits.upper,
        details=f"Unexpected tray slide setup: type={tray_slide.articulation_type}, axis={tray_slide.axis}, limits={tray_slide.motion_limits}",
    )

    ctx.expect_contact(lid, base, elem_a="lid_pad_left", elem_b="front_bezel")
    ctx.expect_contact(lid, base, elem_a="lid_pad_right", elem_b="front_bezel")
    ctx.expect_contact(tray, base, elem_a="tray_front_bar", elem_b="guide_rail_front")
    ctx.expect_contact(tray, base, elem_a="tray_rear_bar", elem_b="guide_rail_rear")
    ctx.expect_within(tray, base, axes="y", outer_elem="platen_glass", margin=0.0)

    with ctx.pose({tray_slide: tray_slide.motion_limits.lower}):
        ctx.expect_contact(tray, base, elem_a="tray_front_bar", elem_b="guide_rail_front")
        ctx.expect_contact(tray, base, elem_a="tray_rear_bar", elem_b="guide_rail_rear")
        ctx.expect_overlap(
            tray,
            base,
            axes="x",
            elem_b="platen_glass",
            min_overlap=0.05,
            name="tray_keeps_left_platen_coverage",
        )

    with ctx.pose({tray_slide: tray_slide.motion_limits.upper}):
        ctx.expect_contact(tray, base, elem_a="tray_front_bar", elem_b="guide_rail_front")
        ctx.expect_contact(tray, base, elem_a="tray_rear_bar", elem_b="guide_rail_rear")
        ctx.expect_overlap(
            tray,
            base,
            axes="x",
            elem_b="platen_glass",
            min_overlap=0.05,
            name="tray_keeps_right_platen_coverage",
        )

    with ctx.pose({lid_hinge: 1.0}):
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            positive_elem="lid_front_frame",
            negative_elem="platen_glass",
            min_gap=0.20,
            name="lid_front_edge_lifts_clear_when_open",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
