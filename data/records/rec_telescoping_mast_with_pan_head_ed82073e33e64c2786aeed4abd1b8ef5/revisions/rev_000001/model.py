from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def ring(outer_d: float, inner_d: float, height: float, z0: float = 0.0) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_d / 2.0)
        .circle(inner_d / 2.0)
        .extrude(height)
        .translate((0.0, 0.0, z0))
    )


def disc(diameter: float, height: float, z0: float = 0.0) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(diameter / 2.0)
        .extrude(height)
        .translate((0.0, 0.0, z0))
    )


def centered_box(
    size: tuple[float, float, float], center: tuple[float, float, float]
) -> cq.Workplane:
    sx, sy, sz = size
    cx, cy, cz = center
    return cq.Workplane("XY").box(sx, sy, sz).translate((cx, cy, cz))


def fuse_all(*shapes: cq.Workplane) -> cq.Workplane:
    fused = shapes[0]
    for shape in shapes[1:]:
        fused = fused.union(shape)
    return fused


def build_base_can() -> cq.Workplane:
    can_od = 0.26
    can_wall = 0.004
    can_h = 0.20
    base_plate_t = 0.008
    top_cap_t = 0.010

    sleeve_od = 0.118
    sleeve_id = 0.106
    sleeve_h = 0.325

    trim_od = 0.17
    trim_id = 0.126
    trim_h = 0.048

    gland_od = 0.128
    gland_id = 0.102
    gland_h = 0.025

    return fuse_all(
        ring(can_od, can_od - 2.0 * can_wall, can_h, 0.0),
        disc(can_od, base_plate_t, 0.0),
        ring(can_od, sleeve_od + 0.008, top_cap_t, can_h - top_cap_t),
        ring(trim_od, trim_id, trim_h, can_h - top_cap_t),
        ring(sleeve_od, sleeve_id, sleeve_h, can_h - top_cap_t),
        ring(gland_od, gland_id, gland_h, can_h - top_cap_t + sleeve_h - gland_h),
    )


def build_lower_stage() -> cq.Workplane:
    tube_od = 0.094
    tube_wall = 0.0035
    tube_h = 0.82

    base_seat_od = 0.114
    base_seat_id = 0.094
    base_seat_h = 0.018

    trim_od = 0.102
    trim_id = 0.092
    trim_h = 0.018

    lock_od = 0.100
    lock_h = 0.018

    gland_od = 0.106
    gland_id = 0.086
    gland_h = 0.045

    return fuse_all(
        ring(base_seat_od, base_seat_id, base_seat_h, 0.0),
        ring(tube_od, tube_od - 2.0 * tube_wall, tube_h - 0.018, 0.018),
        ring(trim_od, trim_id, trim_h, 0.685),
        ring(lock_od, tube_od, lock_h, 0.712),
        ring(gland_od, gland_id, gland_h, tube_h - gland_h),
    )


def build_upper_stage() -> cq.Workplane:
    tube_od = 0.076
    tube_wall = 0.003
    tube_h = 0.78

    support_seat_od = 0.100
    support_seat_id = 0.076
    support_seat_h = 0.018

    lock_od = 0.084
    lock_h = 0.018

    top_bearing_od = 0.102
    top_bearing_id = 0.084
    top_bearing_h = 0.016

    return fuse_all(
        ring(support_seat_od, support_seat_id, support_seat_h, 0.0),
        ring(tube_od, tube_od - 2.0 * tube_wall, tube_h - 0.018, 0.018),
        ring(lock_od, tube_od, lock_h, 0.690),
        ring(top_bearing_od, top_bearing_id, top_bearing_h, tube_h - top_bearing_h),
    )


def build_pan_head() -> cq.Workplane:
    collar = ring(0.102, 0.086, 0.060, 0.0)
    lower_trim = ring(0.110, 0.090, 0.014, 0.0)
    top_hub = disc(0.092, 0.012, 0.060)

    pedestal = centered_box((0.034, 0.044, 0.050), (0.040, 0.000, 0.085))
    top_plate = centered_box((0.110, 0.050, 0.008), (0.064, 0.000, 0.114))
    ear_left = centered_box((0.018, 0.008, 0.030), (0.098, 0.018, 0.133))
    ear_right = centered_box((0.018, 0.008, 0.030), (0.098, -0.018, 0.133))

    gusset_profile = (
        cq.Workplane("XZ")
        .moveTo(0.014, 0.060)
        .lineTo(0.044, 0.060)
        .lineTo(0.080, 0.110)
        .lineTo(0.050, 0.110)
        .close()
        .extrude(0.008)
    )
    gusset_left = gusset_profile.translate((0.0, 0.015, 0.0))
    gusset_right = gusset_profile.translate((0.0, -0.023, 0.0))

    return fuse_all(
        collar,
        lower_trim,
        top_hub,
        pedestal,
        top_plate,
        ear_left,
        ear_right,
        gusset_left,
        gusset_right,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rooftop_inspection_mast")

    base_gray = model.material("base_gray", rgba=(0.27, 0.29, 0.31, 1.0))
    tube_metal = model.material("tube_metal", rgba=(0.74, 0.76, 0.79, 1.0))
    head_dark = model.material("head_dark", rgba=(0.16, 0.17, 0.18, 1.0))

    base = model.part("base_can")
    base.visual(
        mesh_from_cadquery(build_base_can(), "base_can"),
        material=base_gray,
        name="base_shell",
    )

    lower_stage = model.part("lower_stage")
    lower_stage.visual(
        mesh_from_cadquery(build_lower_stage(), "lower_stage"),
        material=tube_metal,
        name="lower_stage_shell",
    )

    upper_stage = model.part("upper_stage")
    upper_stage.visual(
        mesh_from_cadquery(build_upper_stage(), "upper_stage"),
        material=tube_metal,
        name="upper_stage_shell",
    )

    pan_head = model.part("pan_head")
    pan_head.visual(
        mesh_from_cadquery(build_pan_head(), "pan_head"),
        material=head_dark,
        name="pan_head_shell",
    )

    model.articulation(
        "base_to_lower_stage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lower_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.515)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.35,
            lower=0.0,
            upper=0.30,
        ),
    )

    model.articulation(
        "lower_to_upper_stage",
        ArticulationType.PRISMATIC,
        parent=lower_stage,
        child=upper_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.820)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.35,
            lower=0.0,
            upper=0.36,
        ),
    )

    model.articulation(
        "upper_to_pan_head",
        ArticulationType.REVOLUTE,
        parent=upper_stage,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 0.780)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=-2.8,
            upper=2.8,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_can")
    lower_stage = object_model.get_part("lower_stage")
    upper_stage = object_model.get_part("upper_stage")
    pan_head = object_model.get_part("pan_head")

    lower_slide = object_model.get_articulation("base_to_lower_stage")
    upper_slide = object_model.get_articulation("lower_to_upper_stage")
    pan = object_model.get_articulation("upper_to_pan_head")

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=36, name="articulation_paths_clear")

    ctx.check(
        "mast_parts_present",
        all(part is not None for part in (base, lower_stage, upper_stage, pan_head)),
        "Expected base can, two telescoping stages, and a pan head assembly.",
    )
    ctx.expect_overlap(
        lower_stage,
        base,
        axes="xy",
        min_overlap=0.09,
        name="lower_stage_centered_in_base_sleeve",
    )
    ctx.expect_overlap(
        upper_stage,
        lower_stage,
        axes="xy",
        min_overlap=0.07,
        name="upper_stage_centered_in_lower_stage",
    )
    ctx.expect_overlap(
        pan_head,
        upper_stage,
        axes="xy",
        min_overlap=0.07,
        name="pan_head_grounded_on_mast_axis",
    )

    with ctx.pose({lower_slide: 0.0, upper_slide: 0.0, pan: 0.0}):
        ctx.expect_contact(
            lower_stage,
            base,
            name="lower_stage_supported_by_base_guide",
        )
        ctx.expect_contact(
            upper_stage,
            lower_stage,
            name="upper_stage_supported_by_lower_stage_guide",
        )
        ctx.expect_contact(
            pan_head,
            upper_stage,
            name="pan_head_grounded_by_upper_stage_bearing",
        )
        ctx.expect_gap(
            pan_head,
            lower_stage,
            axis="z",
            min_gap=0.70,
            name="pan_head_clears_lower_stage_trim_at_rest",
        )

    with ctx.pose({lower_slide: 0.30, upper_slide: 0.36, pan: 1.4}):
        ctx.expect_gap(
            pan_head,
            base,
            axis="z",
            min_gap=2.00,
            name="pan_head_lifts_clear_of_base_when_extended",
        )
        ctx.expect_gap(
            pan_head,
            lower_stage,
            axis="z",
            min_gap=1.00,
            name="pan_head_remains_visibly_separate_from_extension_stack",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
