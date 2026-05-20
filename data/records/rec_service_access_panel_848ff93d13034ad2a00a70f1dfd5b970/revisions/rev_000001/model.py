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
    model = ArticulatedObject(name="service_access_panel")

    frame_paint = model.material("frame_paint", rgba=(0.28, 0.31, 0.33, 1.0))
    panel_paint = model.material("panel_paint", rgba=(0.44, 0.47, 0.49, 1.0))
    hardware = model.material("hardware", rgba=(0.15, 0.16, 0.17, 1.0))
    zinc = model.material("zinc", rgba=(0.73, 0.74, 0.76, 1.0))
    wear_poly = model.material("wear_poly", rgba=(0.82, 0.80, 0.70, 1.0))
    gasket = model.material("gasket", rgba=(0.07, 0.08, 0.09, 1.0))

    outer_w = 0.58
    outer_h = 0.74
    frame_depth = 0.10
    opening_w = 0.46
    opening_h = 0.62
    jamb_w = (outer_w - opening_w) * 0.5
    rail_h = (outer_h - opening_h) * 0.5

    hinge_axis_y = 0.026
    hinge_radius = 0.012
    hinge_segment_len = 0.110

    door_w = 0.448
    door_h = 0.608
    door_depth = 0.036
    door_skin_t = 0.003
    door_edge = 0.028
    door_left_clearance = 0.006

    frame = model.part("frame")
    frame.visual(
        Box((jamb_w, frame_depth, outer_h)),
        origin=Origin(xyz=(-outer_w * 0.5 + jamb_w * 0.5, 0.0, 0.0)),
        material=frame_paint,
        name="left_jamb",
    )
    frame.visual(
        Box((jamb_w, frame_depth, outer_h)),
        origin=Origin(xyz=(outer_w * 0.5 - jamb_w * 0.5, 0.0, 0.0)),
        material=frame_paint,
        name="right_jamb",
    )
    frame.visual(
        Box((outer_w, frame_depth, rail_h)),
        origin=Origin(xyz=(0.0, 0.0, outer_h * 0.5 - rail_h * 0.5)),
        material=frame_paint,
        name="top_rail",
    )
    frame.visual(
        Box((outer_w, frame_depth, rail_h)),
        origin=Origin(xyz=(0.0, 0.0, -outer_h * 0.5 + rail_h * 0.5)),
        material=frame_paint,
        name="bottom_rail",
    )

    flange_depth = 0.008
    flange_y = frame_depth * 0.5 - flange_depth * 0.5
    frame.visual(
        Box((outer_w, flange_depth, 0.074)),
        origin=Origin(xyz=(0.0, flange_y, outer_h * 0.5 - 0.037)),
        material=hardware,
        name="top_flange",
    )
    frame.visual(
        Box((outer_w, flange_depth, 0.074)),
        origin=Origin(xyz=(0.0, flange_y, -outer_h * 0.5 + 0.037)),
        material=hardware,
        name="bottom_flange",
    )
    frame.visual(
        Box((0.074, flange_depth, outer_h - 0.148)),
        origin=Origin(xyz=(-outer_w * 0.5 + 0.037, flange_y, 0.0)),
        material=hardware,
        name="left_flange",
    )
    frame.visual(
        Box((0.074, flange_depth, outer_h - 0.148)),
        origin=Origin(xyz=(outer_w * 0.5 - 0.037, flange_y, 0.0)),
        material=hardware,
        name="right_flange",
    )

    frame.visual(
        Box((0.026, door_depth, opening_h - 0.020)),
        origin=Origin(xyz=(-opening_w * 0.5 - 0.025, hinge_axis_y, 0.0)),
        material=hardware,
        name="frame_hinge_leaf",
    )
    frame.visual(
        Box((0.024, 0.050, opening_h - 0.060)),
        origin=Origin(xyz=(-opening_w * 0.5 - 0.026, 0.014, 0.0)),
        material=hardware,
        name="hinge_reinforcement",
    )
    for index, center_z in enumerate((-0.240, 0.000, 0.240)):
        frame.visual(
            Cylinder(radius=hinge_radius, length=hinge_segment_len),
            origin=Origin(xyz=(-opening_w * 0.5, hinge_axis_y, center_z)),
            material=zinc,
            name=f"frame_hinge_knuckle_{index}",
        )

    frame.inertial = Inertial.from_geometry(
        Box((outer_w, frame_depth, outer_h)),
        mass=24.0,
        origin=Origin(),
    )

    service_panel = model.part("service_panel")
    service_panel.visual(
        Box((door_w, door_skin_t, door_h)),
        origin=Origin(
            xyz=(
                door_left_clearance + door_w * 0.5,
                door_depth * 0.5 - door_skin_t * 0.5,
                0.0,
            )
        ),
        material=panel_paint,
        name="outer_skin",
    )
    service_panel.visual(
        Box((0.016, door_depth, door_h)),
        origin=Origin(xyz=(0.036, 0.0, 0.0)),
        material=panel_paint,
        name="hinge_stile",
    )
    service_panel.visual(
        Box((door_edge, door_depth, door_h)),
        origin=Origin(
            xyz=(door_left_clearance + door_w - door_edge * 0.5, 0.0, 0.0)
        ),
        material=panel_paint,
        name="latch_stile",
    )
    service_panel.visual(
        Box((door_w - 2.0 * door_edge, door_depth, door_edge)),
        origin=Origin(
            xyz=(
                door_left_clearance + door_w * 0.5,
                0.0,
                door_h * 0.5 - door_edge * 0.5,
            )
        ),
        material=panel_paint,
        name="top_stile",
    )
    service_panel.visual(
        Box((door_w - 2.0 * door_edge, door_depth, door_edge)),
        origin=Origin(
            xyz=(
                door_left_clearance + door_w * 0.5,
                0.0,
                -door_h * 0.5 + door_edge * 0.5,
            )
        ),
        material=panel_paint,
        name="bottom_stile",
    )
    service_panel.visual(
        Box((0.320, 0.012, 0.420)),
        origin=Origin(xyz=(door_left_clearance + 0.228, 0.012, 0.0)),
        material=panel_paint,
        name="service_doubler",
    )
    service_panel.visual(
        Box((0.022, 0.018, 0.430)),
        origin=Origin(xyz=(door_left_clearance + 0.152, -0.002, 0.0)),
        material=hardware,
        name="left_stiffener",
    )
    service_panel.visual(
        Box((0.022, 0.018, 0.430)),
        origin=Origin(xyz=(door_left_clearance + 0.304, -0.002, 0.0)),
        material=hardware,
        name="right_stiffener",
    )
    service_panel.visual(
        Box((0.016, door_depth, door_h - 0.040)),
        origin=Origin(xyz=(0.026, 0.0, 0.0)),
        material=hardware,
        name="door_hinge_leaf",
    )
    service_panel.visual(
        Box((0.010, 0.022, 0.130)),
        origin=Origin(xyz=(0.019, 0.0, 0.170)),
        material=zinc,
        name="upper_hinge_ear",
    )
    service_panel.visual(
        Box((0.010, 0.022, 0.130)),
        origin=Origin(xyz=(0.019, 0.0, -0.170)),
        material=zinc,
        name="lower_hinge_ear",
    )
    service_panel.visual(
        Box((0.012, 0.022, 0.130)),
        origin=Origin(xyz=(0.030, 0.0, 0.170)),
        material=hardware,
        name="upper_hinge_web",
    )
    service_panel.visual(
        Box((0.012, 0.022, 0.130)),
        origin=Origin(xyz=(0.030, 0.0, -0.170)),
        material=hardware,
        name="lower_hinge_web",
    )

    service_panel.visual(
        Box((0.090, 0.026, 0.180)),
        origin=Origin(xyz=(door_left_clearance + 0.386, -0.004, 0.0)),
        material=hardware,
        name="latch_backer",
    )
    service_panel.visual(
        Box((0.060, 0.018, 0.120)),
        origin=Origin(xyz=(door_left_clearance + 0.386, 0.008, 0.0)),
        material=hardware,
        name="latch_housing",
    )
    service_panel.visual(
        Box((0.018, 0.020, 0.018)),
        origin=Origin(xyz=(door_left_clearance + 0.386, 0.020, 0.030)),
        material=hardware,
        name="upper_handle_standoff",
    )
    service_panel.visual(
        Box((0.018, 0.020, 0.018)),
        origin=Origin(xyz=(door_left_clearance + 0.386, 0.020, -0.030)),
        material=hardware,
        name="lower_handle_standoff",
    )
    service_panel.visual(
        Cylinder(radius=0.009, length=0.090),
        origin=Origin(xyz=(door_left_clearance + 0.386, 0.030, 0.0)),
        material=gasket,
        name="handle_grip",
    )
    service_panel.visual(
        Box((0.022, 0.020, 0.110)),
        origin=Origin(xyz=(door_left_clearance + 0.436, -0.008, 0.0)),
        material=hardware,
        name="latch_arm",
    )
    service_panel.visual(
        Box((0.010, 0.014, 0.110)),
        origin=Origin(xyz=(door_left_clearance + 0.443, -0.010, 0.0)),
        material=zinc,
        name="latch_bolt",
    )
    service_panel.visual(
        Box((0.018, 0.014, 0.060)),
        origin=Origin(xyz=(door_left_clearance + 0.439, -0.025, 0.210)),
        material=hardware,
        name="upper_stop",
    )
    service_panel.visual(
        Box((0.018, 0.014, 0.060)),
        origin=Origin(xyz=(door_left_clearance + 0.439, -0.025, -0.210)),
        material=hardware,
        name="lower_stop",
    )

    service_panel.inertial = Inertial.from_geometry(
        Box((door_w + 0.028, door_depth, door_h)),
        mass=12.0,
        origin=Origin(xyz=(door_left_clearance + door_w * 0.5, 0.0, 0.0)),
    )

    model.articulation(
        "frame_to_service_panel",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=service_panel,
        origin=Origin(xyz=(-opening_w * 0.5, hinge_axis_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.0,
            lower=0.0,
            upper=1.30,
        ),
    )

    striker = model.part("striker")
    striker.visual(
        Box((0.006, 0.048, 0.160)),
        origin=Origin(xyz=(-0.003, 0.012, 0.0)),
        material=hardware,
        name="back_plate",
    )
    striker.visual(
        Box((0.006, 0.018, 0.120)),
        origin=Origin(xyz=(-0.003, 0.016, 0.0)),
        material=zinc,
        name="keeper",
    )
    striker.inertial = Inertial.from_geometry(
        Box((0.006, 0.048, 0.160)),
        mass=0.7,
        origin=Origin(xyz=(-0.003, 0.012, 0.0)),
    )
    model.articulation(
        "frame_to_striker",
        ArticulationType.FIXED,
        parent=frame,
        child=striker,
        origin=Origin(xyz=(opening_w * 0.5, 0.0, 0.0)),
    )

    for part_name, center_z in (("upper_wear_pad", 0.220), ("lower_wear_pad", -0.220)):
        wear_pad = model.part(part_name)
        wear_pad.visual(
            Box((0.006, 0.040, 0.080)),
            origin=Origin(xyz=(-0.003, -0.004, 0.0)),
            material=hardware,
            name="backer",
        )
        wear_pad.visual(
            Box((0.006, 0.016, 0.050)),
            origin=Origin(xyz=(-0.003, -0.004, 0.0)),
            material=wear_poly,
            name="pad",
        )
        wear_pad.inertial = Inertial.from_geometry(
            Box((0.006, 0.040, 0.080)),
            mass=0.25,
            origin=Origin(xyz=(-0.003, -0.004, 0.0)),
        )
        model.articulation(
            f"frame_to_{part_name}",
            ArticulationType.FIXED,
            parent=frame,
            child=wear_pad,
            origin=Origin(xyz=(opening_w * 0.5, 0.0, center_z)),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    service_panel = object_model.get_part("service_panel")
    striker = object_model.get_part("striker")
    upper_wear_pad = object_model.get_part("upper_wear_pad")
    lower_wear_pad = object_model.get_part("lower_wear_pad")
    hinge = object_model.get_articulation("frame_to_service_panel")

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

    limits = hinge.motion_limits
    ctx.check(
        "hinge_axis_is_vertical",
        hinge.axis == (0.0, 0.0, 1.0),
        details=f"expected vertical hinge axis, got {hinge.axis}",
    )
    ctx.check(
        "hinge_limits_read_as_outward_swing",
        limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and limits.upper >= 1.20,
        details=f"expected outward service swing limits, got {limits}",
    )

    ctx.expect_contact(
        striker,
        frame,
        elem_a="back_plate",
        elem_b="right_jamb",
        name="striker_is_bolted_to_frame",
    )
    ctx.expect_contact(
        upper_wear_pad,
        frame,
        elem_a="backer",
        elem_b="right_jamb",
        name="upper_wear_pad_is_bolted_to_frame",
    )
    ctx.expect_contact(
        lower_wear_pad,
        frame,
        elem_a="backer",
        elem_b="right_jamb",
        name="lower_wear_pad_is_bolted_to_frame",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_overlap(
            service_panel,
            frame,
            axes="xz",
            min_overlap=0.40,
            name="panel_covers_service_opening_when_closed",
        )
        ctx.expect_contact(
            service_panel,
            striker,
            elem_a="latch_bolt",
            elem_b="keeper",
            name="latch_bolt_meets_keeper_when_closed",
        )
        ctx.expect_contact(
            service_panel,
            upper_wear_pad,
            elem_a="upper_stop",
            elem_b="pad",
            name="upper_stop_lands_on_replaceable_pad",
        )
        ctx.expect_contact(
            service_panel,
            lower_wear_pad,
            elem_a="lower_stop",
            elem_b="pad",
            name="lower_stop_lands_on_replaceable_pad",
        )

    with ctx.pose({hinge: 1.15}):
        ctx.expect_gap(
            service_panel,
            striker,
            axis="y",
            positive_elem="latch_bolt",
            negative_elem="keeper",
            min_gap=0.25,
            name="latch_side_swings_clear_in_service_pose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
