from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, pi

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
    mesh_from_geometry,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _add_bolt_grid(
    part,
    *,
    prefix: str,
    xs: tuple[float, ...],
    zs: tuple[float, ...],
    y: float,
    material,
    radius: float = 0.007,
    depth: float = 0.008,
) -> None:
    index = 0
    for x in xs:
        for z in zs:
            part.visual(
                Cylinder(radius=radius, length=depth),
                origin=Origin(xyz=(x, y, z), rpy=(pi / 2.0, 0.0, 0.0)),
                material=material,
                name=f"{prefix}_{index:02d}",
            )
            index += 1


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_safety_sliding_window")

    frame_gray = model.material("frame_gray", rgba=(0.28, 0.31, 0.34, 1.0))
    sash_gray = model.material("sash_gray", rgba=(0.46, 0.48, 0.50, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.86, 0.73, 0.08, 1.0))
    galvanized = model.material("galvanized", rgba=(0.74, 0.77, 0.80, 1.0))
    gasket_black = model.material("gasket_black", rgba=(0.09, 0.10, 0.11, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.55, 0.72, 0.82, 0.36))

    frame = model.part("frame_assembly")
    frame.inertial = Inertial.from_geometry(
        Box((1.80, 0.24, 1.40)),
        mass=118.0,
    )

    # Primary frame ring.
    frame.visual(
        Box((1.80, 0.18, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.66)),
        material=frame_gray,
        name="header",
    )
    frame.visual(
        Box((1.80, 0.18, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, -0.66)),
        material=frame_gray,
        name="sill",
    )
    frame.visual(
        Box((0.08, 0.18, 1.24)),
        origin=Origin(xyz=(-0.86, 0.0, 0.0)),
        material=frame_gray,
        name="left_jamb",
    )
    frame.visual(
        Box((0.08, 0.18, 1.24)),
        origin=Origin(xyz=(0.86, 0.0, 0.0)),
        material=frame_gray,
        name="right_jamb",
    )

    # Rear fixed-lite frame and glazing pocket.
    frame.visual(
        Box((0.76, 0.05, 0.05)),
        origin=Origin(xyz=(-0.40, -0.040, 0.535)),
        material=frame_gray,
        name="fixed_top_rail",
    )
    frame.visual(
        Box((0.76, 0.05, 0.05)),
        origin=Origin(xyz=(-0.40, -0.040, -0.535)),
        material=frame_gray,
        name="fixed_bottom_rail",
    )
    frame.visual(
        Box((0.05, 0.05, 1.12)),
        origin=Origin(xyz=(-0.78, -0.040, 0.0)),
        material=frame_gray,
        name="fixed_left_stile",
    )
    frame.visual(
        Box((0.06, 0.05, 1.22)),
        origin=Origin(xyz=(-0.02, -0.040, 0.0)),
        material=frame_gray,
        name="center_post",
    )
    frame.visual(
        Box((0.72, 0.008, 0.99)),
        origin=Origin(xyz=(-0.40, -0.040, 0.0)),
        material=glass_tint,
        name="fixed_glass",
    )
    frame.visual(
        Box((0.74, 0.014, 0.025)),
        origin=Origin(xyz=(-0.40, -0.038, 0.507)),
        material=gasket_black,
        name="fixed_glass_gasket_top",
    )
    frame.visual(
        Box((0.74, 0.014, 0.025)),
        origin=Origin(xyz=(-0.40, -0.038, -0.507)),
        material=gasket_black,
        name="fixed_glass_gasket_bottom",
    )
    frame.visual(
        Box((0.022, 0.014, 1.01)),
        origin=Origin(xyz=(-0.749, -0.038, 0.0)),
        material=gasket_black,
        name="fixed_glass_gasket_left",
    )
    frame.visual(
        Box((0.022, 0.014, 1.01)),
        origin=Origin(xyz=(-0.051, -0.038, 0.0)),
        material=gasket_black,
        name="fixed_glass_gasket_right",
    )

    # Sliding track: rear wear bars and front anti-lift lips.
    frame.visual(
        Box((1.64, 0.012, 0.030)),
        origin=Origin(xyz=(0.0, -0.018, 0.605)),
        material=galvanized,
        name="rear_track_bar_top",
    )
    frame.visual(
        Box((1.64, 0.012, 0.030)),
        origin=Origin(xyz=(0.0, -0.018, -0.605)),
        material=galvanized,
        name="rear_track_bar_bottom",
    )
    frame.visual(
        Box((1.64, 0.020, 0.036)),
        origin=Origin(xyz=(0.0, 0.056, 0.602)),
        material=galvanized,
        name="front_track_lip_top",
    )
    frame.visual(
        Box((1.64, 0.020, 0.036)),
        origin=Origin(xyz=(0.0, 0.056, -0.602)),
        material=galvanized,
        name="front_track_lip_bottom",
    )

    # Over-travel stop blocks in the sliding channel.
    frame.visual(
        Box((0.030, 0.020, 0.050)),
        origin=Origin(xyz=(-0.650, 0.056, 0.559)),
        material=safety_yellow,
        name="open_stop_top",
    )
    frame.visual(
        Box((0.030, 0.020, 0.050)),
        origin=Origin(xyz=(-0.650, 0.056, -0.559)),
        material=safety_yellow,
        name="open_stop_bottom",
    )
    frame.visual(
        Box((0.025, 0.020, 0.050)),
        origin=Origin(xyz=(0.8075, 0.056, 0.559)),
        material=safety_yellow,
        name="closed_stop_top",
    )
    frame.visual(
        Box((0.025, 0.020, 0.050)),
        origin=Origin(xyz=(0.8075, 0.056, -0.559)),
        material=safety_yellow,
        name="closed_stop_bottom",
    )

    # External guard frame with diagonal bracing.
    frame.visual(
        Box((1.50, 0.016, 0.040)),
        origin=Origin(xyz=(0.0, 0.108, 0.55)),
        material=safety_yellow,
        name="guard_top_rail",
    )
    frame.visual(
        Box((1.50, 0.016, 0.040)),
        origin=Origin(xyz=(0.0, 0.108, -0.55)),
        material=safety_yellow,
        name="guard_bottom_rail",
    )
    frame.visual(
        Box((0.040, 0.016, 1.10)),
        origin=Origin(xyz=(-0.74, 0.108, 0.0)),
        material=safety_yellow,
        name="guard_left_rail",
    )
    frame.visual(
        Box((0.040, 0.016, 1.10)),
        origin=Origin(xyz=(0.74, 0.108, 0.0)),
        material=safety_yellow,
        name="guard_right_rail",
    )
    for name, x, z in (
        ("guard_mount_lt", -0.74, 0.55),
        ("guard_mount_rt", 0.74, 0.55),
        ("guard_mount_lb", -0.74, -0.55),
        ("guard_mount_rb", 0.74, -0.55),
    ):
        frame.visual(
            Box((0.08, 0.020, 0.08)),
            origin=Origin(xyz=(x, 0.092, z)),
            material=galvanized,
            name=name,
        )
    _add_bolt_grid(
        frame,
        prefix="guard_mount_bolt_left_top",
        xs=(-0.76, -0.72),
        zs=(0.53, 0.57),
        y=0.094,
        material=galvanized,
        radius=0.005,
        depth=0.008,
    )
    _add_bolt_grid(
        frame,
        prefix="guard_mount_bolt_right_top",
        xs=(0.72, 0.76),
        zs=(0.53, 0.57),
        y=0.094,
        material=galvanized,
        radius=0.005,
        depth=0.008,
    )
    _add_bolt_grid(
        frame,
        prefix="guard_mount_bolt_left_bottom",
        xs=(-0.76, -0.72),
        zs=(-0.57, -0.53),
        y=0.094,
        material=galvanized,
        radius=0.005,
        depth=0.008,
    )
    _add_bolt_grid(
        frame,
        prefix="guard_mount_bolt_right_bottom",
        xs=(0.72, 0.76),
        zs=(-0.57, -0.53),
        y=0.094,
        material=galvanized,
        radius=0.005,
        depth=0.008,
    )
    for index, x in enumerate((-0.50, -0.18, 0.16, 0.52)):
        frame.visual(
            Cylinder(radius=0.009, length=1.06),
            origin=Origin(xyz=(x, 0.108, 0.0)),
            material=safety_yellow,
            name=f"guard_bar_{index + 1}",
        )
    brace_angle = atan2(0.94, 1.30)
    frame.visual(
        Box((1.62, 0.008, 0.035)),
        origin=Origin(xyz=(0.0, 0.108, 0.0), rpy=(0.0, -brace_angle, 0.0)),
        material=galvanized,
        name="guard_diagonal_brace",
    )

    # Reinforcement and lockout hardware.
    for plate_name, x, z in (
        ("corner_plate_lt", -0.74, 0.55),
        ("corner_plate_rt", 0.74, 0.55),
        ("corner_plate_lb", -0.74, -0.55),
        ("corner_plate_rb", 0.74, -0.55),
        ("center_tie_plate_top", -0.02, 0.56),
        ("center_tie_plate_bottom", -0.02, -0.56),
    ):
        size_x = 0.16 if "corner" in plate_name else 0.12
        size_z = 0.16 if "corner" in plate_name else 0.12
        frame.visual(
            Box((size_x, 0.012, size_z)),
            origin=Origin(xyz=(x, 0.084, z)),
            material=galvanized,
            name=plate_name,
        )
        _add_bolt_grid(
            frame,
            prefix=f"{plate_name}_bolt",
            xs=(x - size_x * 0.22, x + size_x * 0.22),
            zs=(z - size_z * 0.22, z + size_z * 0.22),
            y=0.086,
            material=galvanized,
        )

    frame.visual(
        Box((0.09, 0.040, 0.18)),
        origin=Origin(xyz=(0.855, 0.070, 0.0)),
        material=safety_yellow,
        name="lockout_housing",
    )
    frame.visual(
        Box((0.13, 0.010, 0.26)),
        origin=Origin(xyz=(0.800, 0.086, 0.0)),
        material=galvanized,
        name="lockout_backing_plate",
    )
    _add_bolt_grid(
        frame,
        prefix="lockout_bolt",
        xs=(0.765, 0.835),
        zs=(-0.07, 0.07),
        y=0.088,
        material=galvanized,
    )

    sash = model.part("sliding_sash")
    sash.inertial = Inertial.from_geometry(
        Box((0.82, 0.07, 1.14)),
        mass=34.0,
    )

    sash.visual(
        Box((0.80, 0.050, 0.050)),
        origin=Origin(xyz=(0.0, 0.018, 0.535)),
        material=sash_gray,
        name="top_rail",
    )
    sash.visual(
        Box((0.80, 0.050, 0.050)),
        origin=Origin(xyz=(0.0, 0.018, -0.535)),
        material=sash_gray,
        name="bottom_rail",
    )
    sash.visual(
        Box((0.050, 0.050, 1.12)),
        origin=Origin(xyz=(-0.375, 0.018, 0.0)),
        material=sash_gray,
        name="left_stile",
    )
    sash.visual(
        Box((0.050, 0.050, 1.12)),
        origin=Origin(xyz=(0.375, 0.018, 0.0)),
        material=sash_gray,
        name="right_stile",
    )
    sash.visual(
        Box((0.69, 0.008, 0.99)),
        origin=Origin(xyz=(0.0, 0.010, 0.0)),
        material=glass_tint,
        name="sash_glass",
    )
    sash.visual(
        Box((0.71, 0.014, 0.024)),
        origin=Origin(xyz=(0.0, 0.012, 0.507)),
        material=gasket_black,
        name="sash_gasket_top",
    )
    sash.visual(
        Box((0.71, 0.014, 0.024)),
        origin=Origin(xyz=(0.0, 0.012, -0.507)),
        material=gasket_black,
        name="sash_gasket_bottom",
    )
    sash.visual(
        Box((0.022, 0.014, 1.01)),
        origin=Origin(xyz=(-0.339, 0.012, 0.0)),
        material=gasket_black,
        name="sash_gasket_left",
    )
    sash.visual(
        Box((0.022, 0.014, 1.01)),
        origin=Origin(xyz=(0.339, 0.012, 0.0)),
        material=gasket_black,
        name="sash_gasket_right",
    )

    sash.visual(
        Box((0.10, 0.046, 0.036)),
        origin=Origin(xyz=(-0.26, 0.011, 0.572)),
        material=galvanized,
        name="top_guide_shoe_left",
    )
    sash.visual(
        Box((0.10, 0.046, 0.036)),
        origin=Origin(xyz=(0.26, 0.011, 0.572)),
        material=galvanized,
        name="top_guide_shoe_right",
    )
    sash.visual(
        Box((0.10, 0.046, 0.036)),
        origin=Origin(xyz=(-0.26, 0.011, -0.572)),
        material=galvanized,
        name="bottom_guide_shoe_left",
    )
    sash.visual(
        Box((0.10, 0.046, 0.036)),
        origin=Origin(xyz=(0.26, 0.011, -0.572)),
        material=galvanized,
        name="bottom_guide_shoe_right",
    )

    for gusset_name, x, z in (
        ("gusset_lt", -0.320, 0.470),
        ("gusset_rt", 0.320, 0.470),
        ("gusset_lb", -0.320, -0.470),
        ("gusset_rb", 0.320, -0.470),
    ):
        sash.visual(
            Box((0.12, 0.010, 0.12)),
            origin=Origin(xyz=(x, 0.038, z)),
            material=galvanized,
            name=gusset_name,
        )
        _add_bolt_grid(
            sash,
            prefix=f"{gusset_name}_bolt",
            xs=(x - 0.030, x + 0.030),
            zs=(z - 0.030, z + 0.030),
            y=0.040,
            material=galvanized,
            radius=0.0055,
            depth=0.006,
        )

    sash.visual(
        Box((0.12, 0.012, 0.28)),
        origin=Origin(xyz=(0.24, 0.037, 0.0)),
        material=galvanized,
        name="handle_mount_plate",
    )
    handle_geom = tube_from_spline_points(
        [
            (0.0, 0.0, -0.11),
            (0.0, 0.028, -0.11),
            (0.0, 0.055, -0.06),
            (0.0, 0.066, 0.0),
            (0.0, 0.055, 0.06),
            (0.0, 0.028, 0.11),
            (0.0, 0.0, 0.11),
        ],
        radius=0.010,
        samples_per_segment=10,
        radial_segments=18,
        cap_ends=True,
    )
    sash.visual(
        _mesh("window_pull_handle", handle_geom),
        origin=Origin(xyz=(0.24, 0.033, 0.0)),
        material=safety_yellow,
        name="pull_handle",
    )
    _add_bolt_grid(
        sash,
        prefix="handle_bolt",
        xs=(0.21, 0.27),
        zs=(-0.09, 0.09),
        y=0.039,
        material=galvanized,
    )

    sash.visual(
        Box((0.06, 0.012, 0.18)),
        origin=Origin(xyz=(0.372, 0.037, 0.0)),
        material=safety_yellow,
        name="lockout_strike",
    )
    sash.visual(
        Box((0.08, 0.010, 0.22)),
        origin=Origin(xyz=(0.338, 0.033, 0.0)),
        material=galvanized,
        name="lockout_strike_backer",
    )

    model.articulation(
        "sash_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=sash,
        origin=Origin(xyz=(0.39, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=600.0, velocity=0.70, lower=0.0, upper=0.62),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame_assembly")
    sash = object_model.get_part("sliding_sash")
    slide = object_model.get_articulation("sash_slide")

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

    ctx.expect_gap(
        sash,
        frame,
        axis="y",
        positive_elem="sash_glass",
        negative_elem="fixed_glass",
        min_gap=0.03,
        max_gap=0.06,
        name="sash_runs_in_front_of_fixed_lite",
    )
    ctx.expect_gap(
        sash,
        frame,
        axis="y",
        positive_elem="top_guide_shoe_left",
        negative_elem="rear_track_bar_top",
        min_gap=-1e-6,
        max_gap=0.001,
        max_penetration=1e-6,
        name="top_shoe_bears_on_rear_track",
    )
    ctx.expect_gap(
        frame,
        sash,
        axis="y",
        positive_elem="front_track_lip_top",
        negative_elem="top_guide_shoe_left",
        min_gap=0.008,
        max_gap=0.016,
        name="front_lip_retains_top_shoe",
    )

    with ctx.pose({slide: 0.0}):
        ctx.expect_gap(
            frame,
            sash,
            axis="x",
            positive_elem="closed_stop_top",
            negative_elem="right_stile",
            min_gap=0.0,
            max_gap=0.01,
            name="closed_position_hits_right_stop",
        )
        ctx.expect_gap(
            frame,
            sash,
            axis="x",
            positive_elem="lockout_housing",
            negative_elem="lockout_strike",
            min_gap=0.015,
            max_gap=0.03,
            name="lockout_region_aligns_when_closed",
        )

    with ctx.pose({slide: slide.motion_limits.upper}):
        ctx.expect_gap(
            sash,
            frame,
            axis="x",
            positive_elem="left_stile",
            negative_elem="open_stop_top",
            min_gap=0.0,
            max_gap=0.01,
            name="open_position_hits_left_stop",
        )
        ctx.expect_overlap(
            sash,
            frame,
            axes="x",
            min_overlap=0.45,
            elem_a="sash_glass",
            elem_b="fixed_glass",
            name="open_sash_stacks_over_fixed_lite",
        )

    with ctx.pose({slide: 0.0}):
        closed_pos = ctx.part_world_position(sash)
    with ctx.pose({slide: slide.motion_limits.upper}):
        open_pos = ctx.part_world_position(sash)
    ctx.check(
        "sash_positive_motion_opens_left",
        closed_pos is not None
        and open_pos is not None
        and open_pos[0] < closed_pos[0] - 0.50,
        details=f"closed={closed_pos}, open={open_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
