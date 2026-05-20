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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_watch_winder_box")

    exterior = model.material("exterior", rgba=(0.16, 0.16, 0.17, 1.0))
    trim = model.material("trim", rgba=(0.62, 0.66, 0.70, 1.0))
    liner = model.material("liner", rgba=(0.09, 0.09, 0.10, 1.0))
    cradle_finish = model.material("cradle_finish", rgba=(0.18, 0.18, 0.20, 1.0))
    datum = model.material("datum", rgba=(0.78, 0.80, 0.82, 1.0))
    accent = model.material("accent", rgba=(0.72, 0.77, 0.82, 1.0))

    outer_w = 0.280
    outer_d = 0.190
    wall_t = 0.010
    floor_t = 0.008
    base_wall_h = 0.090
    rim_h = 0.006
    hinge_z = 0.094
    hinge_y = outer_d / 2.0

    base = model.part("base")
    base.visual(
        Box((outer_w, outer_d, floor_t)),
        origin=Origin(xyz=(0.0, 0.0, floor_t / 2.0)),
        material=exterior,
        name="floor",
    )
    base.visual(
        Box((outer_w, wall_t, base_wall_h)),
        origin=Origin(xyz=(0.0, -0.090, 0.049)),
        material=exterior,
        name="front_wall",
    )
    base.visual(
        Box((outer_w, 0.012, base_wall_h)),
        origin=Origin(xyz=(0.0, 0.081, 0.049)),
        material=exterior,
        name="rear_wall",
    )
    base.visual(
        Box((wall_t, 0.166, base_wall_h)),
        origin=Origin(xyz=(-0.135, -0.001, 0.049)),
        material=exterior,
        name="left_wall",
    )
    base.visual(
        Box((wall_t, 0.166, base_wall_h)),
        origin=Origin(xyz=(0.135, -0.001, 0.049)),
        material=exterior,
        name="right_wall",
    )
    base.visual(
        Box((0.248, 0.158, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=liner,
        name="liner",
    )
    base.visual(
        Box((0.260, 0.008, rim_h)),
        origin=Origin(xyz=(0.0, -0.081, 0.096)),
        material=trim,
        name="front_rim",
    )
    base.visual(
        Box((0.008, 0.154, rim_h)),
        origin=Origin(xyz=(-0.126, -0.004, 0.096)),
        material=trim,
        name="left_rim",
    )
    base.visual(
        Box((0.008, 0.154, rim_h)),
        origin=Origin(xyz=(0.126, -0.004, 0.096)),
        material=trim,
        name="right_rim",
    )
    base.visual(
        Box((0.120, 0.014, 0.006)),
        origin=Origin(xyz=(0.0, -0.063, 0.015)),
        material=trim,
        name="datum_rail",
    )
    base.visual(
        Box((0.030, 0.018, 0.004)),
        origin=Origin(xyz=(-0.052, -0.063, 0.014)),
        material=datum,
        name="left_datum_pad",
    )
    base.visual(
        Box((0.030, 0.018, 0.004)),
        origin=Origin(xyz=(0.052, -0.063, 0.014)),
        material=datum,
        name="right_datum_pad",
    )
    base.visual(
        Box((0.004, 0.010, 0.002)),
        origin=Origin(xyz=(-0.018, -0.063, 0.019)),
        material=accent,
        name="datum_mark_left",
    )
    base.visual(
        Box((0.004, 0.010, 0.002)),
        origin=Origin(xyz=(0.0, -0.063, 0.019)),
        material=accent,
        name="datum_mark_center",
    )
    base.visual(
        Box((0.004, 0.010, 0.002)),
        origin=Origin(xyz=(0.018, -0.063, 0.019)),
        material=accent,
        name="datum_mark_right",
    )
    base.visual(
        Cylinder(radius=0.004, length=0.010),
        origin=Origin(xyz=(-0.082, -0.058, 0.013)),
        material=trim,
        name="left_adjust_stem",
    )
    base.visual(
        Cylinder(radius=0.009, length=0.006),
        origin=Origin(xyz=(-0.082, -0.058, 0.018)),
        material=trim,
        name="left_adjust_knob",
    )
    base.visual(
        Cylinder(radius=0.004, length=0.010),
        origin=Origin(xyz=(0.082, -0.058, 0.013)),
        material=trim,
        name="right_adjust_stem",
    )
    base.visual(
        Cylinder(radius=0.009, length=0.006),
        origin=Origin(xyz=(0.082, -0.058, 0.018)),
        material=trim,
        name="right_adjust_knob",
    )
    base.visual(
        Box((0.018, 0.020, 0.014)),
        origin=Origin(xyz=(-0.104, 0.089, 0.087)),
        material=trim,
        name="left_hinge_ear",
    )
    base.visual(
        Box((0.018, 0.020, 0.014)),
        origin=Origin(xyz=(0.104, 0.089, 0.087)),
        material=trim,
        name="right_hinge_ear",
    )
    base.visual(
        Cylinder(radius=0.006, length=0.046),
        origin=Origin(xyz=(-0.104, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim,
        name="left_hinge_knuckle",
    )
    base.visual(
        Cylinder(radius=0.006, length=0.046),
        origin=Origin(xyz=(0.104, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim,
        name="right_hinge_knuckle",
    )
    base.inertial = Inertial.from_geometry(
        Box((outer_w, outer_d, 0.100)),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
    )

    drive_frame = model.part("drive_frame")
    drive_frame.visual(
        Box((0.182, 0.092, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=liner,
        name="frame_plate",
    )
    drive_frame.visual(
        Box((0.166, 0.010, 0.024)),
        origin=Origin(xyz=(0.0, -0.029, 0.018)),
        material=trim,
        name="rear_bridge",
    )
    drive_frame.visual(
        Box((0.150, 0.012, 0.016)),
        origin=Origin(xyz=(0.0, 0.030, 0.012)),
        material=trim,
        name="calibration_beam",
    )
    drive_frame.visual(
        Cylinder(radius=0.007, length=0.006),
        origin=Origin(xyz=(-0.060, 0.030, 0.021)),
        material=trim,
        name="beam_adjust_left",
    )
    drive_frame.visual(
        Cylinder(radius=0.007, length=0.006),
        origin=Origin(xyz=(0.060, 0.030, 0.021)),
        material=trim,
        name="beam_adjust_right",
    )
    drive_frame.visual(
        Box((0.004, 0.010, 0.002)),
        origin=Origin(xyz=(-0.020, 0.030, 0.021)),
        material=accent,
        name="beam_mark_left",
    )
    drive_frame.visual(
        Box((0.004, 0.010, 0.002)),
        origin=Origin(xyz=(0.0, 0.030, 0.021)),
        material=accent,
        name="beam_mark_center",
    )
    drive_frame.visual(
        Box((0.004, 0.010, 0.002)),
        origin=Origin(xyz=(0.020, 0.030, 0.021)),
        material=accent,
        name="beam_mark_right",
    )
    drive_frame.visual(
        Box((0.016, 0.026, 0.024)),
        origin=Origin(xyz=(-0.073, 0.0, 0.018)),
        material=trim,
        name="left_column",
    )
    drive_frame.visual(
        Box((0.016, 0.026, 0.024)),
        origin=Origin(xyz=(0.073, 0.0, 0.018)),
        material=trim,
        name="right_column",
    )
    drive_frame.visual(
        Box((0.016, 0.040, 0.006)),
        origin=Origin(xyz=(-0.073, 0.0, 0.029)),
        material=datum,
        name="left_saddle",
    )
    drive_frame.visual(
        Box((0.016, 0.040, 0.006)),
        origin=Origin(xyz=(0.073, 0.0, 0.029)),
        material=datum,
        name="right_saddle",
    )
    drive_frame.visual(
        Box((0.016, 0.006, 0.024)),
        origin=Origin(xyz=(-0.073, -0.018, 0.042)),
        material=trim,
        name="left_rear_guide",
    )
    drive_frame.visual(
        Box((0.016, 0.006, 0.024)),
        origin=Origin(xyz=(-0.073, 0.018, 0.042)),
        material=trim,
        name="left_front_guide",
    )
    drive_frame.visual(
        Box((0.016, 0.006, 0.024)),
        origin=Origin(xyz=(0.073, -0.018, 0.042)),
        material=trim,
        name="right_rear_guide",
    )
    drive_frame.visual(
        Box((0.016, 0.006, 0.024)),
        origin=Origin(xyz=(0.073, 0.018, 0.042)),
        material=trim,
        name="right_front_guide",
    )
    drive_frame.inertial = Inertial.from_geometry(
        Box((0.182, 0.092, 0.066)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
    )

    model.articulation(
        "base_to_drive_frame",
        ArticulationType.FIXED,
        parent=base,
        child=drive_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
    )

    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.005, length=0.176),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim,
        name="shaft",
    )
    cradle.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(xyz=(-0.073, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim,
        name="left_hub",
    )
    cradle.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(xyz=(0.073, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim,
        name="right_hub",
    )
    cradle.visual(
        Cylinder(radius=0.024, length=0.092),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cradle_finish,
        name="back_drum",
    )
    cradle.visual(
        Box((0.074, 0.010, 0.046)),
        origin=Origin(xyz=(0.0, 0.029, 0.0)),
        material=datum,
        name="datum_plate",
    )
    cradle.visual(
        Box((0.010, 0.020, 0.044)),
        origin=Origin(xyz=(-0.032, 0.028, 0.0)),
        material=cradle_finish,
        name="left_cheek",
    )
    cradle.visual(
        Box((0.010, 0.020, 0.044)),
        origin=Origin(xyz=(0.032, 0.028, 0.0)),
        material=cradle_finish,
        name="right_cheek",
    )
    cradle.visual(
        Box((0.058, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, 0.028, -0.021)),
        material=cradle_finish,
        name="lower_bridge",
    )
    cradle.visual(
        Box((0.054, 0.022, 0.024)),
        origin=Origin(xyz=(0.0, 0.014, 0.0)),
        material=liner,
        name="watch_pad",
    )
    cradle.visual(
        Cylinder(radius=0.022, length=0.006),
        origin=Origin(xyz=(0.085, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=datum,
        name="index_disk",
    )
    cradle.visual(
        Box((0.004, 0.006, 0.010)),
        origin=Origin(xyz=(0.087, 0.0, 0.018)),
        material=accent,
        name="index_mark_top",
    )
    cradle.visual(
        Box((0.004, 0.010, 0.006)),
        origin=Origin(xyz=(0.087, 0.018, 0.0)),
        material=accent,
        name="index_mark_front",
    )
    cradle.visual(
        Box((0.004, 0.006, 0.010)),
        origin=Origin(xyz=(0.087, 0.0, -0.018)),
        material=accent,
        name="index_mark_bottom",
    )
    cradle.inertial = Inertial.from_geometry(
        Box((0.176, 0.070, 0.082)),
        mass=0.45,
        origin=Origin(),
    )

    model.articulation(
        "drive_frame_to_cradle",
        ArticulationType.CONTINUOUS,
        parent=drive_frame,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.5, velocity=4.0),
    )

    lid = model.part("lid")
    lid.visual(
        Cylinder(radius=0.006, length=0.164),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim,
        name="hinge_barrel",
    )
    lid.visual(
        Box((0.272, 0.184, 0.006)),
        origin=Origin(xyz=(0.0, -0.092, 0.055)),
        material=exterior,
        name="top_panel",
    )
    lid.visual(
        Box((0.006, 0.178, 0.046)),
        origin=Origin(xyz=(-0.133, -0.089, 0.029)),
        material=exterior,
        name="left_skirt",
    )
    lid.visual(
        Box((0.006, 0.178, 0.048)),
        origin=Origin(xyz=(0.133, -0.089, 0.030)),
        material=exterior,
        name="right_skirt",
    )
    lid.visual(
        Box((0.260, 0.006, 0.048)),
        origin=Origin(xyz=(0.0, -0.181, 0.030)),
        material=exterior,
        name="front_skirt",
    )
    lid.visual(
        Box((0.248, 0.160, 0.004)),
        origin=Origin(xyz=(0.0, -0.092, 0.010)),
        material=liner,
        name="inner_liner",
    )
    lid.visual(
        Box((0.160, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, -0.006, 0.012)),
        material=trim,
        name="hinge_bridge",
    )
    lid.visual(
        Box((0.160, 0.010, 0.056)),
        origin=Origin(xyz=(0.0, -0.005, 0.028)),
        material=trim,
        name="rear_web",
    )
    lid.visual(
        Box((0.090, 0.012, 0.004)),
        origin=Origin(xyz=(0.0, -0.160, 0.014)),
        material=datum,
        name="presentation_stop",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.272, 0.184, 0.060)),
        mass=1.2,
        origin=Origin(xyz=(0.0, -0.092, 0.030)),
    )

    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(100.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    drive_frame = object_model.get_part("drive_frame")
    cradle = object_model.get_part("cradle")
    lid = object_model.get_part("lid")
    lid_hinge = object_model.get_articulation("base_to_lid")
    cradle_spin = object_model.get_articulation("drive_frame_to_cradle")

    def aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        drive_frame,
        base,
        elem_a="frame_plate",
        elem_b="liner",
        contact_tol=1e-4,
        name="drive_frame_seats_on_base_liner",
    )
    ctx.expect_contact(
        cradle,
        drive_frame,
        elem_a="left_hub",
        elem_b="left_saddle",
        contact_tol=1e-4,
        name="left_cradle_support_point_present",
    )
    ctx.expect_contact(
        cradle,
        drive_frame,
        elem_a="right_hub",
        elem_b="right_saddle",
        contact_tol=1e-4,
        name="right_cradle_support_point_present",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            positive_elem="front_skirt",
            negative_elem="front_rim",
            min_gap=0.0005,
            max_gap=0.0020,
            name="front_lid_gap_is_controlled",
        )
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            positive_elem="left_skirt",
            negative_elem="left_rim",
            min_gap=0.0005,
            max_gap=0.0020,
            name="left_lid_gap_is_controlled",
        )
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            positive_elem="right_skirt",
            negative_elem="right_rim",
            min_gap=0.0005,
            max_gap=0.0020,
            name="right_lid_gap_is_controlled",
        )
        ctx.expect_gap(
            lid,
            cradle,
            axis="z",
            positive_elem="inner_liner",
            negative_elem="datum_plate",
            min_gap=0.012,
            max_gap=0.040,
            name="closed_lid_clears_rotor_package",
        )

    closed_top_center = aabb_center(ctx.part_element_world_aabb(lid, elem="top_panel"))
    with ctx.pose({lid_hinge: math.radians(72.0)}):
        opened_top_center = aabb_center(ctx.part_element_world_aabb(lid, elem="top_panel"))
    ctx.check(
        "lid_opens_upward_from_hinge_line",
        closed_top_center is not None
        and opened_top_center is not None
        and opened_top_center[2] > closed_top_center[2] + 0.040
        and opened_top_center[1] > closed_top_center[1] + 0.080,
        details=f"closed={closed_top_center}, opened={opened_top_center}",
    )

    with ctx.pose({cradle_spin: -math.pi / 2.0}):
        ctx.expect_gap(
            cradle,
            base,
            axis="z",
            positive_elem="datum_plate",
            negative_elem="liner",
            min_gap=0.002,
            max_gap=0.020,
            name="rotor_clears_floor_at_downward_index",
        )
        ctx.expect_within(
            cradle,
            base,
            axes="xy",
            margin=0.010,
            name="rotor_stays_within_presentation_box_footprint",
        )

    mark_home = aabb_center(ctx.part_element_world_aabb(cradle, elem="index_mark_top"))
    with ctx.pose({cradle_spin: math.pi / 2.0}):
        mark_quarter = aabb_center(ctx.part_element_world_aabb(cradle, elem="index_mark_top"))
    ctx.check(
        "cradle_index_mark_moves_with_spin_axis",
        mark_home is not None
        and mark_quarter is not None
        and abs(mark_quarter[1] - mark_home[1]) > 0.012
        and abs(mark_quarter[2] - mark_home[2]) > 0.012,
        details=f"home={mark_home}, quarter={mark_quarter}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
