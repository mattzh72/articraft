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
    xyz: tuple[float, float, float],
    material,
    name: str,
    *,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
):
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _add_cylinder_x(
    part,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material,
    name: str,
):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="multifunction_tailgate")

    body_red = model.material("body_red", rgba=(0.73, 0.12, 0.10, 1.0))
    liner_black = model.material("liner_black", rgba=(0.13, 0.13, 0.14, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.24, 0.25, 0.27, 1.0))
    steel_mid = model.material("steel_mid", rgba=(0.42, 0.43, 0.45, 1.0))
    handle_black = model.material("handle_black", rgba=(0.08, 0.08, 0.09, 1.0))
    aluminum = model.material("aluminum", rgba=(0.73, 0.75, 0.78, 1.0))

    bed_width = 1.70
    bed_side_depth = 0.38
    floor_thickness = 0.045
    floor_top = 0.585
    bed_side_height = 0.430
    bed_wall_thickness = 0.080

    tailgate_width = 1.50
    tailgate_height = 0.550
    shell_depth = 0.054
    shell_half_depth = shell_depth * 0.5
    outer_skin_thickness = 0.014
    inner_panel_thickness = 0.012
    side_rail_width = 0.070
    top_rail_height = 0.065
    lower_beam_height = 0.020
    opening_width = 0.740
    flap_width = 0.736
    flap_height = 0.114
    flap_thickness = 0.020
    flap_axis_z = 0.034
    inner_panel_y = 0.022
    tailgate_hinge_y = -0.028

    inner_width = tailgate_width - (2.0 * side_rail_width)
    filler_width = (inner_width - opening_width) * 0.5
    filler_center_x = (opening_width * 0.5) + (filler_width * 0.5)

    bed_frame = model.part("bed_frame")
    _add_box(
        bed_frame,
        (bed_width, bed_side_depth, floor_thickness),
        (0.0, bed_side_depth * 0.5, floor_top - (floor_thickness * 0.5)),
        liner_black,
        "bed_floor",
    )
    _add_box(
        bed_frame,
        (bed_wall_thickness, bed_side_depth, bed_side_height),
        (
            -(bed_width * 0.5) + (bed_wall_thickness * 0.5),
            bed_side_depth * 0.5,
            floor_top + (bed_side_height * 0.5) - 0.004,
        ),
        body_red,
        "left_bedside",
    )
    _add_box(
        bed_frame,
        (bed_wall_thickness, bed_side_depth, bed_side_height),
        (
            (bed_width * 0.5) - (bed_wall_thickness * 0.5),
            bed_side_depth * 0.5,
            floor_top + (bed_side_height * 0.5) - 0.004,
        ),
        body_red,
        "right_bedside",
    )
    _add_box(
        bed_frame,
        (bed_width - 0.16, 0.030, 0.055),
        (0.0, 0.015, floor_top - 0.0275),
        steel_dark,
        "rear_sill_structure",
    )
    for side_name, x_sign in (("left", -1.0), ("right", 1.0)):
        _add_box(
            bed_frame,
            (0.070, 0.040, 0.056),
            (
                x_sign * 0.690,
                0.030,
                floor_top + 0.028,
            ),
            steel_mid,
            f"{side_name}_hinge_mount",
        )
        _add_cylinder_x(
            bed_frame,
            0.010,
            0.080,
            (
                x_sign * 0.690,
                0.014,
                floor_top + 0.020,
            ),
            steel_dark,
            f"{side_name}_hinge_pin_housing",
        )
    bed_frame.inertial = Inertial.from_geometry(
        Box((bed_width, bed_side_depth, bed_side_height + floor_thickness)),
        mass=72.0,
        origin=Origin(
            xyz=(0.0, bed_side_depth * 0.5, floor_top + ((bed_side_height - floor_thickness) * 0.5))
        ),
    )

    main_tailgate = model.part("main_tailgate")
    _add_box(
        main_tailgate,
        (tailgate_width, outer_skin_thickness, tailgate_height),
        (0.0, -0.027, tailgate_height * 0.5),
        body_red,
        "outer_skin",
    )
    _add_box(
        main_tailgate,
        (1.08, 0.006, 0.190),
        (0.0, -0.023, 0.320),
        steel_mid,
        "outer_stamp_panel",
    )
    _add_box(
        main_tailgate,
        (0.260, 0.006, 0.080),
        (0.0, -0.023, 0.468),
        steel_dark,
        "handle_bezel",
    )
    _add_box(
        main_tailgate,
        (side_rail_width, shell_depth, tailgate_height),
        (-(tailgate_width * 0.5) + (side_rail_width * 0.5), 0.0, tailgate_height * 0.5),
        body_red,
        "left_side_rail",
    )
    _add_box(
        main_tailgate,
        (side_rail_width, shell_depth, tailgate_height),
        ((tailgate_width * 0.5) - (side_rail_width * 0.5), 0.0, tailgate_height * 0.5),
        body_red,
        "right_side_rail",
    )
    _add_box(
        main_tailgate,
        (tailgate_width, shell_depth, top_rail_height),
        (0.0, 0.0, tailgate_height - (top_rail_height * 0.5)),
        body_red,
        "top_rail",
    )
    _add_box(
        main_tailgate,
        (tailgate_width, 0.028, lower_beam_height),
        (0.0, -0.010, lower_beam_height * 0.5),
        steel_dark,
        "lower_rear_beam",
    )
    _add_box(
        main_tailgate,
        (opening_width + 0.010, inner_panel_thickness, flap_axis_z),
        (0.0, inner_panel_y, flap_axis_z * 0.5),
        liner_black,
        "inner_lower_lip",
    )
    _add_box(
        main_tailgate,
        (inner_width, inner_panel_thickness, 0.290),
        (0.0, inner_panel_y, 0.343),
        liner_black,
        "inner_upper_panel",
    )
    _add_box(
        main_tailgate,
        (inner_width, inner_panel_thickness, 0.054),
        (0.0, inner_panel_y, 0.175),
        liner_black,
        "inner_mid_rail",
    )
    _add_box(
        main_tailgate,
        (filler_width + 0.004, inner_panel_thickness, 0.116),
        (-filler_center_x, inner_panel_y, 0.091),
        liner_black,
        "left_flap_filler",
    )
    _add_box(
        main_tailgate,
        (filler_width + 0.004, inner_panel_thickness, 0.116),
        (filler_center_x, inner_panel_y, 0.091),
        liner_black,
        "right_flap_filler",
    )
    for side_name, x_sign in (("left", -1.0), ("right", 1.0)):
        _add_box(
            main_tailgate,
            (0.040, 0.020, 0.040),
            (x_sign * 0.395, 0.008, 0.028),
            steel_dark,
            f"{side_name}_flap_hinge_clip",
        )
        _add_box(
            main_tailgate,
            (0.070, 0.036, 0.036),
            (x_sign * 0.680, -0.012, 0.018),
            steel_mid,
            f"{side_name}_corner_hinge_ear",
        )
    main_tailgate.inertial = Inertial.from_geometry(
        Box((tailgate_width, shell_depth, tailgate_height)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, tailgate_height * 0.5)),
    )

    ramp_flap = model.part("ramp_flap")
    _add_box(
        ramp_flap,
        (flap_width, flap_thickness, flap_height),
        (0.0, 0.006, flap_height * 0.5),
        aluminum,
        "flap_panel",
    )
    _add_box(
        ramp_flap,
        (flap_width - 0.100, 0.006, 0.012),
        (0.0, 0.013, flap_height - 0.012),
        liner_black,
        "flap_top_tread",
    )
    _add_box(
        ramp_flap,
        (flap_width - 0.140, 0.004, 0.010),
        (0.0, 0.014, flap_height - 0.036),
        liner_black,
        "flap_mid_tread",
    )
    for side_name, x_sign in (("left", -1.0), ("right", 1.0)):
        _add_cylinder_x(
            ramp_flap,
            0.010,
            0.060,
            (x_sign * 0.310, -0.008, 0.0),
            steel_dark,
            f"{side_name}_flap_knuckle",
        )
    ramp_flap.inertial = Inertial.from_geometry(
        Box((flap_width, flap_thickness, flap_height)),
        mass=4.5,
        origin=Origin(xyz=(0.0, 0.006, flap_height * 0.5)),
    )

    outer_handle = model.part("outer_handle")
    _add_box(
        outer_handle,
        (0.190, 0.016, 0.050),
        (0.0, -0.008, -0.025),
        handle_black,
        "handle_paddle",
    )
    _add_box(
        outer_handle,
        (0.140, 0.010, 0.014),
        (0.0, -0.017, -0.039),
        steel_mid,
        "handle_pull_lip",
    )
    _add_box(
        outer_handle,
        (0.026, 0.016, 0.016),
        (-0.078, -0.008, -0.008),
        handle_black,
        "left_handle_pivot_pad",
    )
    _add_box(
        outer_handle,
        (0.026, 0.016, 0.016),
        (0.078, -0.008, -0.008),
        handle_black,
        "right_handle_pivot_pad",
    )
    outer_handle.inertial = Inertial.from_geometry(
        Box((0.190, 0.022, 0.060)),
        mass=0.7,
        origin=Origin(xyz=(0.0, -0.011, -0.024)),
    )

    model.articulation(
        "bed_to_tailgate",
        ArticulationType.REVOLUTE,
        parent=bed_frame,
        child=main_tailgate,
        origin=Origin(xyz=(0.0, tailgate_hinge_y, floor_top)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=1.0,
            lower=0.0,
            upper=math.radians(92.0),
        ),
    )
    model.articulation(
        "tailgate_to_ramp_flap",
        ArticulationType.REVOLUTE,
        parent=main_tailgate,
        child=ramp_flap,
        origin=Origin(xyz=(0.0, 0.012, flap_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(88.0),
        ),
    )
    model.articulation(
        "tailgate_to_outer_handle",
        ArticulationType.REVOLUTE,
        parent=main_tailgate,
        child=outer_handle,
        origin=Origin(xyz=(0.0, -0.034, 0.468)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=3.0,
            lower=math.radians(-24.0),
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bed_frame = object_model.get_part("bed_frame")
    main_tailgate = object_model.get_part("main_tailgate")
    ramp_flap = object_model.get_part("ramp_flap")
    outer_handle = object_model.get_part("outer_handle")
    bed_to_tailgate = object_model.get_articulation("bed_to_tailgate")
    tailgate_to_ramp_flap = object_model.get_articulation("tailgate_to_ramp_flap")
    tailgate_to_outer_handle = object_model.get_articulation("tailgate_to_outer_handle")

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
        "main_tailgate_uses_lower_horizontal_hinge",
        tuple(bed_to_tailgate.axis) == (1.0, 0.0, 0.0)
        and bed_to_tailgate.motion_limits is not None
        and bed_to_tailgate.motion_limits.lower == 0.0
        and bed_to_tailgate.motion_limits.upper is not None
        and bed_to_tailgate.motion_limits.upper > 1.4,
        "The main tailgate should rotate downward on a lower horizontal hinge axis.",
    )
    ctx.check(
        "ramp_flap_uses_its_own_horizontal_hinge",
        tuple(tailgate_to_ramp_flap.axis) == (1.0, 0.0, 0.0)
        and tailgate_to_ramp_flap.motion_limits is not None
        and tailgate_to_ramp_flap.motion_limits.lower == 0.0
        and tailgate_to_ramp_flap.motion_limits.upper is not None
        and tailgate_to_ramp_flap.motion_limits.upper > 1.2,
        "The loading-ramp flap should fold on its own lower horizontal hinge inside the tailgate.",
    )
    ctx.check(
        "outer_handle_has_local_pivot",
        tuple(tailgate_to_outer_handle.axis) == (1.0, 0.0, 0.0)
        and tailgate_to_outer_handle.motion_limits is not None
        and tailgate_to_outer_handle.motion_limits.lower is not None
        and tailgate_to_outer_handle.motion_limits.lower < -0.2
        and tailgate_to_outer_handle.motion_limits.upper == 0.0,
        "The outer latch handle should pivot on its own local horizontal axis.",
    )

    ctx.expect_contact(
        main_tailgate,
        bed_frame,
        name="closed_tailgate_is_seated_on_bed_structure",
    )
    ctx.expect_gap(
        bed_frame,
        main_tailgate,
        axis="y",
        min_gap=0.0,
        max_gap=0.002,
        name="closed_tailgate_sits_at_rear_bed_plane",
    )
    ctx.expect_overlap(
        main_tailgate,
        bed_frame,
        axes="x",
        min_overlap=1.45,
        name="tailgate_spans_the_bed_opening_width",
    )
    ctx.expect_contact(
        ramp_flap,
        main_tailgate,
        name="ramp_flap_stays_clipped_into_the_tailgate_opening",
    )
    ctx.expect_within(
        ramp_flap,
        main_tailgate,
        axes="x",
        margin=0.02,
        name="ramp_flap_stays_within_tailgate_width",
    )
    ctx.expect_contact(
        outer_handle,
        main_tailgate,
        name="outer_handle_rests_on_the_outer_panel_when_closed",
    )

    closed_gate_aabb = ctx.part_world_aabb(main_tailgate)
    closed_flap_aabb = ctx.part_world_aabb(ramp_flap)
    closed_handle_aabb = ctx.part_world_aabb(outer_handle)
    assert closed_gate_aabb is not None
    assert closed_flap_aabb is not None
    assert closed_handle_aabb is not None

    with ctx.pose(
        {
            bed_to_tailgate: math.radians(88.0),
            tailgate_to_ramp_flap: math.radians(78.0),
            tailgate_to_outer_handle: math.radians(-18.0),
        }
    ):
        open_gate_aabb = ctx.part_world_aabb(main_tailgate)
        open_flap_aabb = ctx.part_world_aabb(ramp_flap)
        open_handle_aabb = ctx.part_world_aabb(outer_handle)
        assert open_gate_aabb is not None
        assert open_flap_aabb is not None
        assert open_handle_aabb is not None

        ctx.check(
            "main_tailgate_swings_down_from_the_bed",
            open_gate_aabb[0][1] < closed_gate_aabb[0][1] - 0.45
            and open_gate_aabb[1][2] < closed_gate_aabb[1][2] - 0.40,
            "The main gate should rotate outward and downward from the lower hinge line.",
        )
        ctx.check(
            "ramp_flap_folds_below_open_tailgate_surface",
            open_flap_aabb[0][2] < open_gate_aabb[0][2] - 0.06
            and open_flap_aabb[1][1] < open_gate_aabb[1][1] + 0.02,
            "The small flap should fold down from the inner panel when the tailgate is open.",
        )
        ctx.check(
            "outer_handle_pulls_outward",
            open_handle_aabb[0][1] < closed_handle_aabb[0][1] - 0.010,
            "The latch handle should rotate outward from the outer skin.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
