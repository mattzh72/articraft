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


def _axis_is_x(axis: tuple[float, float, float]) -> bool:
    return tuple(round(value, 6) for value in axis) == (1.0, 0.0, 0.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pickup_tailgate_with_work_surface")

    body_blue = model.material("body_blue", rgba=(0.16, 0.22, 0.34, 1.0))
    liner_black = model.material("liner_black", rgba=(0.10, 0.11, 0.12, 1.0))
    trim_black = model.material("trim_black", rgba=(0.08, 0.08, 0.09, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.28, 0.30, 0.33, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.76, 0.78, 0.80, 1.0))
    rubber_dark = model.material("rubber_dark", rgba=(0.14, 0.14, 0.15, 1.0))

    bed_frame = model.part("bed_frame")
    bed_frame.visual(
        Box((1.72, 0.54, 0.05)),
        origin=Origin(xyz=(0.0, -0.27, 0.025)),
        material=liner_black,
        name="bed_floor",
    )
    bed_frame.visual(
        Box((1.72, 0.04, 0.38)),
        origin=Origin(xyz=(0.0, -0.52, 0.24)),
        material=body_blue,
        name="front_bulkhead",
    )
    for side_sign, side_name in ((1.0, "left"), (-1.0, "right")):
        x_center = 0.82 * side_sign
        bed_frame.visual(
            Box((0.08, 0.50, 0.38)),
            origin=Origin(xyz=(x_center, -0.27, 0.24)),
            material=body_blue,
            name=f"{side_name}_bedside",
        )
        bed_frame.visual(
            Box((0.08, 0.10, 0.32)),
            origin=Origin(xyz=(x_center, 0.03, 0.21)),
            material=body_blue,
            name=f"{side_name}_rear_stub",
        )
        bed_frame.visual(
            Box((0.14, 0.05, 0.06)),
            origin=Origin(xyz=(0.79 * side_sign, 0.025, 0.05)),
            material=dark_steel,
            name=f"{side_name}_hinge_support",
        )
        bed_frame.visual(
            Cylinder(radius=0.009, length=0.06),
            origin=Origin(
                xyz=(0.75 * side_sign, 0.0, 0.05),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=dark_steel,
            name=f"{side_name}_hinge_pin",
        )
    bed_frame.inertial = Inertial.from_geometry(
        Box((1.72, 0.54, 0.43)),
        mass=120.0,
        origin=Origin(xyz=(0.0, -0.27, 0.215)),
    )

    tailgate = model.part("tailgate")
    tailgate.visual(
        Box((1.54, 0.01, 0.56)),
        origin=Origin(xyz=(0.0, 0.025, 0.31)),
        material=body_blue,
        name="outer_skin",
    )
    tailgate.visual(
        Box((1.48, 0.05, 0.08)),
        origin=Origin(xyz=(0.0, -0.005, 0.07)),
        material=liner_black,
        name="bottom_frame",
    )
    tailgate.visual(
        Box((1.54, 0.05, 0.08)),
        origin=Origin(xyz=(0.0, -0.005, 0.55)),
        material=liner_black,
        name="top_frame",
    )
    for side_sign, side_name in ((1.0, "left"), (-1.0, "right")):
        tailgate.visual(
            Box((0.10, 0.05, 0.40)),
            origin=Origin(xyz=(0.71 * side_sign, -0.005, 0.31)),
            material=liner_black,
            name=f"{side_name}_side_frame",
        )
        tailgate.visual(
            Box((0.04, 0.022, 0.10)),
            origin=Origin(xyz=(0.75 * side_sign, -0.019, 0.08)),
            material=dark_steel,
            name=f"{side_name}_hinge_strap",
        )
        tailgate.visual(
            Box((0.05, 0.018, 0.01)),
            origin=Origin(xyz=(0.75 * side_sign, 0.0, 0.064)),
            material=dark_steel,
            name=f"{side_name}_hinge_clip_top",
        )
        tailgate.visual(
            Box((0.05, 0.018, 0.01)),
            origin=Origin(xyz=(0.75 * side_sign, 0.0, 0.036)),
            material=dark_steel,
            name=f"{side_name}_hinge_clip_bottom",
        )
        tailgate.visual(
            Box((0.05, 0.008, 0.028)),
            origin=Origin(xyz=(0.75 * side_sign, 0.013, 0.05)),
            material=dark_steel,
            name=f"{side_name}_hinge_clip_web",
        )

    tailgate.visual(
        Box((1.20, 0.018, 0.03)),
        origin=Origin(xyz=(0.0, -0.021, 0.095)),
        material=liner_black,
        name="panel_lower_sill",
    )
    tailgate.visual(
        Box((1.20, 0.018, 0.08)),
        origin=Origin(xyz=(0.0, -0.021, 0.47)),
        material=liner_black,
        name="panel_upper_header",
    )
    tailgate.visual(
        Cylinder(radius=0.007, length=1.10),
        origin=Origin(xyz=(0.0, -0.020, 0.125), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="panel_hinge_rod",
    )
    for side_sign, side_name in ((1.0, "left"), (-1.0, "right")):
        tailgate.visual(
            Box((0.05, 0.018, 0.32)),
            origin=Origin(xyz=(0.575 * side_sign, -0.021, 0.27)),
            material=liner_black,
            name=f"{side_name}_panel_side_rail",
        )

    tailgate.visual(
        Box((0.20, 0.010, 0.05)),
        origin=Origin(xyz=(0.0, 0.033, 0.47)),
        material=trim_black,
        name="handle_bezel",
    )
    tailgate.inertial = Inertial.from_geometry(
        Box((1.54, 0.06, 0.59)),
        mass=35.0,
        origin=Origin(xyz=(0.0, 0.0, 0.295)),
    )

    work_panel = model.part("work_panel")
    work_panel.visual(
        Box((1.06, 0.016, 0.29)),
        origin=Origin(xyz=(0.0, -0.016, 0.155)),
        material=brushed_aluminum,
        name="work_surface",
    )
    work_panel.visual(
        Box((1.06, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, -0.020, 0.291)),
        material=trim_black,
        name="top_edge_trim",
    )
    for clip_index, x_center in enumerate((-0.33, 0.0, 0.33)):
        work_panel.visual(
            Box((0.06, 0.012, 0.034)),
            origin=Origin(xyz=(x_center, -0.012, 0.0)),
            material=dark_steel,
            name=f"clip_stem_{clip_index}",
        )
        work_panel.visual(
            Box((0.08, 0.016, 0.008)),
            origin=Origin(xyz=(x_center, 0.0, 0.011)),
            material=dark_steel,
            name=f"clip_top_{clip_index}",
        )
        work_panel.visual(
            Box((0.08, 0.016, 0.008)),
            origin=Origin(xyz=(x_center, 0.0, -0.011)),
            material=dark_steel,
            name=f"clip_bottom_{clip_index}",
        )
        work_panel.visual(
            Box((0.08, 0.006, 0.024)),
            origin=Origin(xyz=(x_center, 0.007, 0.0)),
            material=dark_steel,
            name=f"clip_web_{clip_index}",
        )
    work_panel.inertial = Inertial.from_geometry(
        Box((1.06, 0.03, 0.29)),
        mass=8.0,
        origin=Origin(xyz=(0.0, -0.012, 0.145)),
    )

    latch_handle = model.part("latch_handle")
    latch_handle.visual(
        Cylinder(radius=0.006, length=0.14),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_black,
        name="handle_pivot_rod",
    )
    for side_sign, side_name in ((1.0, "left"), (-1.0, "right")):
        latch_handle.visual(
            Box((0.024, 0.014, 0.024)),
            origin=Origin(xyz=(0.055 * side_sign, 0.004, -0.01)),
            material=trim_black,
            name=f"{side_name}_handle_arm",
        )
    latch_handle.visual(
        Box((0.18, 0.014, 0.03)),
        origin=Origin(xyz=(0.0, 0.01, -0.018)),
        material=rubber_dark,
        name="handle_paddle",
    )
    latch_handle.visual(
        Box((0.14, 0.012, 0.008)),
        origin=Origin(xyz=(0.0, 0.018, -0.032)),
        material=trim_black,
        name="handle_grip_lip",
    )
    latch_handle.inertial = Inertial.from_geometry(
        Box((0.18, 0.03, 0.05)),
        mass=0.6,
        origin=Origin(xyz=(0.0, 0.008, -0.016)),
    )

    model.articulation(
        "bed_to_tailgate",
        ArticulationType.REVOLUTE,
        parent=bed_frame,
        child=tailgate,
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=1.2,
            lower=-math.radians(95.0),
            upper=0.0,
        ),
    )
    model.articulation(
        "tailgate_to_work_panel",
        ArticulationType.REVOLUTE,
        parent=tailgate,
        child=work_panel,
        origin=Origin(xyz=(0.0, -0.020, 0.125)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(105.0),
        ),
    )
    model.articulation(
        "tailgate_to_latch_handle",
        ArticulationType.REVOLUTE,
        parent=tailgate,
        child=latch_handle,
        origin=Origin(xyz=(0.0, 0.042, 0.47)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=4.0,
            lower=0.0,
            upper=math.radians(32.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bed_frame = object_model.get_part("bed_frame")
    tailgate = object_model.get_part("tailgate")
    work_panel = object_model.get_part("work_panel")
    latch_handle = object_model.get_part("latch_handle")

    bed_to_tailgate = object_model.get_articulation("bed_to_tailgate")
    tailgate_to_work_panel = object_model.get_articulation("tailgate_to_work_panel")
    tailgate_to_latch_handle = object_model.get_articulation("tailgate_to_latch_handle")

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

    ctx.check("tailgate_hinge_axis", _axis_is_x(bed_to_tailgate.axis), f"axis={bed_to_tailgate.axis}")
    ctx.check(
        "work_panel_hinge_axis",
        _axis_is_x(tailgate_to_work_panel.axis),
        f"axis={tailgate_to_work_panel.axis}",
    )
    ctx.check(
        "latch_handle_axis",
        _axis_is_x(tailgate_to_latch_handle.axis),
        f"axis={tailgate_to_latch_handle.axis}",
    )

    tailgate_limits = bed_to_tailgate.motion_limits
    panel_limits = tailgate_to_work_panel.motion_limits
    handle_limits = tailgate_to_latch_handle.motion_limits
    ctx.check(
        "tailgate_motion_range",
        tailgate_limits is not None
        and tailgate_limits.lower is not None
        and tailgate_limits.upper is not None
        and tailgate_limits.lower < -1.4
        and abs(tailgate_limits.upper) < 1e-9,
        f"limits={tailgate_limits}",
    )
    ctx.check(
        "work_panel_motion_range",
        panel_limits is not None
        and panel_limits.lower is not None
        and panel_limits.upper is not None
        and abs(panel_limits.lower) < 1e-9
        and panel_limits.upper > 1.4,
        f"limits={panel_limits}",
    )
    ctx.check(
        "latch_handle_motion_range",
        handle_limits is not None
        and handle_limits.lower is not None
        and handle_limits.upper is not None
        and abs(handle_limits.lower) < 1e-9
        and 0.45 < handle_limits.upper < 0.7,
        f"limits={handle_limits}",
    )

    ctx.expect_contact(tailgate, bed_frame, contact_tol=0.0015)
    ctx.expect_contact(work_panel, tailgate, contact_tol=0.0015)
    ctx.expect_contact(latch_handle, tailgate, contact_tol=0.0015)
    ctx.expect_within(work_panel, tailgate, axes="xz", margin=0.03)
    ctx.expect_origin_gap(latch_handle, tailgate, axis="y", min_gap=0.03, max_gap=0.06)

    tailgate_closed = ctx.part_element_world_aabb(tailgate, elem="outer_skin")
    work_panel_closed = ctx.part_element_world_aabb(work_panel, elem="work_surface")
    handle_closed = ctx.part_element_world_aabb(latch_handle, elem="handle_paddle")

    assert tailgate_closed is not None
    assert work_panel_closed is not None
    assert handle_closed is not None

    with ctx.pose({bed_to_tailgate: -math.radians(90.0)}):
        tailgate_open = ctx.part_element_world_aabb(tailgate, elem="outer_skin")
        assert tailgate_open is not None
        ctx.check(
            "tailgate_drops_downward",
            tailgate_open[1][1] > tailgate_closed[1][1] + 0.45
            and tailgate_open[1][2] < tailgate_closed[1][2] - 0.35,
            f"closed={tailgate_closed} open={tailgate_open}",
        )

    with ctx.pose({tailgate_to_work_panel: math.radians(90.0)}):
        work_panel_open = ctx.part_element_world_aabb(work_panel, elem="work_surface")
        assert work_panel_open is not None
        ctx.check(
            "work_panel_folds_out_from_inner_face",
            work_panel_open[0][1] < work_panel_closed[0][1] - 0.20
            and work_panel_open[1][2] < work_panel_closed[1][2] - 0.14,
            f"closed={work_panel_closed} open={work_panel_open}",
        )

    with ctx.pose({tailgate_to_latch_handle: math.radians(25.0)}):
        handle_open = ctx.part_element_world_aabb(latch_handle, elem="handle_paddle")
        assert handle_open is not None
        ctx.check(
            "latch_handle_rotates_outward",
            handle_open[1][1] > handle_closed[1][1] + 0.008,
            f"closed={handle_closed} open={handle_open}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
