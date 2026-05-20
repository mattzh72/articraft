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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _circle_profile(
    radius: float,
    *,
    dx: float = 0.0,
    dy: float = 0.0,
    segments: int = 18,
) -> list[tuple[float, float]]:
    return [
        (
            dx + radius * math.cos((2.0 * math.pi * index) / segments),
            dy + radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _plate_with_bolt_holes(
    name: str,
    *,
    width: float,
    height: float,
    thickness: float,
    corner_radius: float,
    hole_centers: list[tuple[float, float]],
    hole_radius: float,
):
    outer = rounded_rect_profile(width, height, corner_radius, corner_segments=8)
    holes = [
        _circle_profile(hole_radius, dx=hole_x, dy=hole_y, segments=16)
        for hole_x, hole_y in hole_centers
    ]
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            outer,
            holes,
            height=thickness,
            center=True,
        ),
        name,
    )


def _add_bolt_heads(
    part,
    *,
    positions: list[tuple[float, float]],
    z_center: float,
    radius: float,
    height: float,
    material,
    prefix: str,
) -> None:
    for index, (x_pos, y_pos) in enumerate(positions):
        part.visual(
            Cylinder(radius=radius, length=height),
            origin=Origin(xyz=(x_pos, y_pos, z_center)),
            material=material,
            name=f"{prefix}_{index:02d}",
        )


def _aabb_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    minimum, maximum = aabb
    return tuple((minimum[index] + maximum[index]) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_legacy_turntable")

    walnut = model.material("walnut", rgba=(0.46, 0.30, 0.18, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.76, 0.78, 0.80, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.23, 0.24, 0.26, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    smoked_paint = model.material("smoked_paint", rgba=(0.15, 0.16, 0.17, 1.0))
    fastener_steel = model.material("fastener_steel", rgba=(0.66, 0.68, 0.71, 1.0))
    phenolic = model.material("phenolic", rgba=(0.10, 0.10, 0.11, 1.0))

    plinth_width = 0.46
    plinth_depth = 0.36
    plinth_body_height = 0.058
    top_plate_thickness = 0.006
    top_surface_z = plinth_body_height + top_plate_thickness

    platter_center = (-0.040, 0.000)
    tonearm_mount = (0.166, 0.098)
    motor_hatch_pos = (-0.060, 0.020)
    signal_hatch_pos = (0.125, 0.078)

    plinth = model.part("plinth")
    plinth.visual(
        Box((plinth_width, plinth_depth, plinth_body_height)),
        origin=Origin(xyz=(0.0, 0.0, plinth_body_height * 0.5)),
        material=walnut,
        name="plinth_body",
    )
    plinth.visual(
        Box((0.452, 0.352, top_plate_thickness)),
        origin=Origin(xyz=(0.0, 0.0, plinth_body_height + top_plate_thickness * 0.5)),
        material=smoked_paint,
        name="top_deck",
    )
    plinth.visual(
        Box((0.452, 0.016, 0.020)),
        origin=Origin(xyz=(0.0, 0.172, 0.050)),
        material=dark_steel,
        name="rear_service_band",
    )
    plinth.visual(
        Box((0.088, 0.028, 0.006)),
        origin=Origin(xyz=(-0.158, -0.144, top_surface_z)),
        material=dark_steel,
        name="speed_selector_escutcheon",
    )
    for foot_index, (foot_x, foot_y) in enumerate(
        [
            (-0.185, -0.135),
            (-0.185, 0.135),
            (0.185, -0.135),
            (0.185, 0.135),
        ]
    ):
        plinth.visual(
            Cylinder(radius=0.022, length=0.010),
            origin=Origin(xyz=(foot_x, foot_y, -0.005)),
            material=black_rubber,
            name=f"foot_{foot_index:02d}",
        )
    plinth.inertial = Inertial.from_geometry(
        Box((plinth_width, plinth_depth, plinth_body_height + 0.010)),
        mass=11.5,
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
    )

    bearing_support = model.part("bearing_support")
    bearing_bolt_positions = [(-0.029, -0.029), (-0.029, 0.029), (0.029, -0.029), (0.029, 0.029)]
    bearing_support.visual(
        _plate_with_bolt_holes(
            "bearing_adapter_plate",
            width=0.090,
            height=0.090,
            thickness=0.004,
            corner_radius=0.008,
            hole_centers=bearing_bolt_positions,
            hole_radius=0.0035,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=dark_steel,
        name="bearing_plate",
    )
    _add_bolt_heads(
        bearing_support,
        positions=bearing_bolt_positions,
        z_center=0.0055,
        radius=0.0045,
        height=0.003,
        material=fastener_steel,
        prefix="bearing_bolt",
    )
    bearing_support.visual(
        Cylinder(radius=0.018, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=dark_steel,
        name="bearing_barrel",
    )
    bearing_support.visual(
        Cylinder(radius=0.026, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=brushed_aluminum,
        name="thrust_collar",
    )
    bearing_support.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=dark_steel,
        name="bearing_skirt",
    )
    bearing_support.inertial = Inertial.from_geometry(
        Box((0.090, 0.090, 0.026)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
    )

    platter = model.part("platter")
    platter.visual(
        Cylinder(radius=0.034, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.0025)),
        material=dark_steel,
        name="hub_flange",
    )
    platter.visual(
        Cylinder(radius=0.146, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=brushed_aluminum,
        name="platter_body",
    )
    platter.visual(
        Cylinder(radius=0.153, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=brushed_aluminum,
        name="strobe_ring",
    )
    platter.visual(
        Cylinder(radius=0.142, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0285)),
        material=black_rubber,
        name="record_mat",
    )
    platter.visual(
        Cylinder(radius=0.003, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.039)),
        material=fastener_steel,
        name="spindle_pin",
    )
    platter.inertial = Inertial.from_geometry(
        Cylinder(radius=0.153, length=0.031),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, 0.0155)),
    )

    tonearm_pedestal = model.part("tonearm_pedestal")
    tonearm_bolt_positions = [(-0.026, -0.038), (-0.026, 0.038), (0.026, -0.038), (0.026, 0.038)]
    tonearm_pedestal.visual(
        _plate_with_bolt_holes(
            "tonearm_adapter_plate",
            width=0.078,
            height=0.102,
            thickness=0.004,
            corner_radius=0.008,
            hole_centers=tonearm_bolt_positions,
            hole_radius=0.0035,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=dark_steel,
        name="armboard",
    )
    _add_bolt_heads(
        tonearm_pedestal,
        positions=tonearm_bolt_positions,
        z_center=0.0055,
        radius=0.0045,
        height=0.003,
        material=fastener_steel,
        prefix="armboard_bolt",
    )
    tonearm_pedestal.visual(
        Cylinder(radius=0.018, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=dark_steel,
        name="pivot_column",
    )
    tonearm_pedestal.visual(
        Cylinder(radius=0.024, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=brushed_aluminum,
        name="pivot_collar",
    )
    tonearm_pedestal.visual(
        Box((0.016, 0.040, 0.016)),
        origin=Origin(xyz=(-0.012, 0.0, 0.012)),
        material=dark_steel,
        name="rear_brace",
    )
    tonearm_pedestal.visual(
        Box((0.022, 0.012, 0.020)),
        origin=Origin(xyz=(-0.006, 0.018, 0.014)),
        material=dark_steel,
        name="side_web_upper",
    )
    tonearm_pedestal.visual(
        Box((0.022, 0.012, 0.020)),
        origin=Origin(xyz=(-0.006, -0.018, 0.014)),
        material=dark_steel,
        name="side_web_lower",
    )
    tonearm_pedestal.visual(
        Cylinder(radius=0.003, length=0.018),
        origin=Origin(xyz=(0.024, -0.028, 0.013)),
        material=fastener_steel,
        name="rest_post",
    )
    tonearm_pedestal.visual(
        Box((0.010, 0.004, 0.004)),
        origin=Origin(xyz=(0.024, -0.028, 0.024)),
        material=phenolic,
        name="rest_cradle",
    )
    tonearm_pedestal.inertial = Inertial.from_geometry(
        Box((0.090, 0.102, 0.040)),
        mass=0.65,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
    )

    tonearm_stage = model.part("tonearm_stage")
    arm_tube_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.000, 0.000, 0.010),
                (0.042, 0.000, 0.012),
                (0.135, 0.006, 0.011),
                (0.214, 0.014, 0.009),
            ],
            radius=0.005,
            samples_per_segment=16,
            radial_segments=18,
            cap_ends=True,
        ),
        "tonearm_tube",
    )
    tonearm_stage.visual(
        Cylinder(radius=0.026, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=brushed_aluminum,
        name="stage_base",
    )
    tonearm_stage.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=dark_steel,
        name="pivot_cap",
    )
    tonearm_stage.visual(
        arm_tube_mesh,
        material=brushed_aluminum,
        name="arm_tube",
    )
    tonearm_stage.visual(
        Box((0.022, 0.018, 0.004)),
        origin=Origin(xyz=(0.224, 0.016, 0.009)),
        material=dark_steel,
        name="headshell",
    )
    tonearm_stage.visual(
        Box((0.012, 0.010, 0.005)),
        origin=Origin(xyz=(0.228, 0.019, 0.0045)),
        material=phenolic,
        name="cartridge",
    )
    tonearm_stage.visual(
        Cylinder(radius=0.004, length=0.042),
        origin=Origin(xyz=(-0.021, 0.0, 0.011), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="counter_stub",
    )
    tonearm_stage.visual(
        Cylinder(radius=0.011, length=0.022),
        origin=Origin(xyz=(-0.053, 0.0, 0.011), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=fastener_steel,
        name="counterweight",
    )
    tonearm_stage.visual(
        Cylinder(radius=0.0025, length=0.018),
        origin=Origin(xyz=(-0.004, 0.018, 0.013), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=fastener_steel,
        name="anti_skate_bar",
    )
    tonearm_stage.visual(
        Cylinder(radius=0.0035, length=0.008),
        origin=Origin(xyz=(-0.004, 0.031, 0.013), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=fastener_steel,
        name="anti_skate_weight",
    )
    tonearm_stage.inertial = Inertial.from_geometry(
        Box((0.300, 0.060, 0.028)),
        mass=0.28,
        origin=Origin(xyz=(0.110, 0.0, 0.011)),
    )

    motor_hatch = model.part("motor_hatch")
    motor_hatch_bolts = [(-0.047, -0.032), (-0.047, 0.032), (0.047, -0.032), (0.047, 0.032)]
    motor_hatch.visual(
        _plate_with_bolt_holes(
            "motor_service_hatch",
            width=0.130,
            height=0.105,
            thickness=0.004,
            corner_radius=0.008,
            hole_centers=motor_hatch_bolts,
            hole_radius=0.003,
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=dark_steel,
        name="motor_panel",
    )
    _add_bolt_heads(
        motor_hatch,
        positions=motor_hatch_bolts,
        z_center=-0.0055,
        radius=0.004,
        height=0.003,
        material=fastener_steel,
        prefix="motor_hatch_bolt",
    )
    motor_hatch.visual(
        Box((0.090, 0.014, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=dark_steel,
        name="motor_stiffener",
    )
    motor_hatch.inertial = Inertial.from_geometry(
        Box((0.130, 0.105, 0.014)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, -0.007)),
    )

    signal_hatch = model.part("signal_hatch")
    signal_hatch_bolts = [(-0.033, -0.042), (-0.033, 0.042), (0.033, -0.042), (0.033, 0.042)]
    signal_hatch.visual(
        _plate_with_bolt_holes(
            "signal_service_hatch",
            width=0.096,
            height=0.122,
            thickness=0.004,
            corner_radius=0.008,
            hole_centers=signal_hatch_bolts,
            hole_radius=0.003,
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=dark_steel,
        name="signal_panel",
    )
    _add_bolt_heads(
        signal_hatch,
        positions=signal_hatch_bolts,
        z_center=-0.0055,
        radius=0.004,
        height=0.003,
        material=fastener_steel,
        prefix="signal_hatch_bolt",
    )
    signal_hatch.visual(
        Box((0.066, 0.014, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=dark_steel,
        name="signal_stiffener",
    )
    signal_hatch.inertial = Inertial.from_geometry(
        Box((0.096, 0.122, 0.014)),
        mass=0.20,
        origin=Origin(xyz=(0.0, 0.0, -0.007)),
    )

    model.articulation(
        "plinth_to_bearing_support",
        ArticulationType.FIXED,
        parent=plinth,
        child=bearing_support,
        origin=Origin(xyz=(platter_center[0], platter_center[1], top_surface_z)),
    )
    model.articulation(
        "bearing_to_platter",
        ArticulationType.CONTINUOUS,
        parent=bearing_support,
        child=platter,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0),
    )
    model.articulation(
        "plinth_to_tonearm_pedestal",
        ArticulationType.FIXED,
        parent=plinth,
        child=tonearm_pedestal,
        origin=Origin(xyz=(tonearm_mount[0], tonearm_mount[1], top_surface_z)),
    )

    tonearm_rest_yaw = math.atan2(-0.040, -0.196)
    model.articulation(
        "pedestal_to_tonearm_stage",
        ArticulationType.REVOLUTE,
        parent=tonearm_pedestal,
        child=tonearm_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.036), rpy=(0.0, 0.0, tonearm_rest_yaw)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=1.5,
            lower=-0.25,
            upper=0.55,
        ),
    )
    model.articulation(
        "plinth_to_motor_hatch",
        ArticulationType.FIXED,
        parent=plinth,
        child=motor_hatch,
        origin=Origin(xyz=(motor_hatch_pos[0], motor_hatch_pos[1], 0.0)),
    )
    model.articulation(
        "plinth_to_signal_hatch",
        ArticulationType.FIXED,
        parent=plinth,
        child=signal_hatch,
        origin=Origin(xyz=(signal_hatch_pos[0], signal_hatch_pos[1], 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    plinth = object_model.get_part("plinth")
    bearing_support = object_model.get_part("bearing_support")
    platter = object_model.get_part("platter")
    tonearm_pedestal = object_model.get_part("tonearm_pedestal")
    tonearm_stage = object_model.get_part("tonearm_stage")
    motor_hatch = object_model.get_part("motor_hatch")
    signal_hatch = object_model.get_part("signal_hatch")
    platter_spin = object_model.get_articulation("bearing_to_platter")
    tonearm_sweep = object_model.get_articulation("pedestal_to_tonearm_stage")

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

    ctx.expect_contact(bearing_support, plinth, name="bearing_support_is_mounted")
    ctx.expect_contact(tonearm_pedestal, plinth, name="tonearm_pedestal_is_mounted")
    ctx.expect_contact(motor_hatch, plinth, name="motor_hatch_is_seated")
    ctx.expect_contact(signal_hatch, plinth, name="signal_hatch_is_seated")
    ctx.expect_contact(platter, bearing_support, name="platter_is_supported_by_bearing")
    ctx.expect_contact(tonearm_stage, tonearm_pedestal, name="tonearm_stage_is_supported")

    ctx.expect_overlap(
        platter,
        bearing_support,
        axes="xy",
        min_overlap=0.040,
        name="platter_is_concentric_with_bearing",
    )
    ctx.expect_gap(
        tonearm_stage,
        platter,
        axis="z",
        min_gap=0.002,
        max_gap=0.060,
        positive_elem="headshell",
        negative_elem="record_mat",
        name="tonearm_stage_stays_above_record_plane",
    )
    ctx.expect_within(motor_hatch, plinth, axes="xy", margin=0.005, name="motor_hatch_within_plinth")
    ctx.expect_within(signal_hatch, plinth, axes="xy", margin=0.005, name="signal_hatch_within_plinth")

    ctx.check(
        "platter_joint_axis_is_vertical",
        platter_spin.axis == (0.0, 0.0, 1.0),
        details=f"expected platter axis (0,0,1), got {platter_spin.axis}",
    )
    tonearm_limits = tonearm_sweep.motion_limits
    ctx.check(
        "tonearm_arc_limits_are_playable",
        tonearm_limits is not None
        and tonearm_limits.lower is not None
        and tonearm_limits.upper is not None
        and tonearm_limits.lower < 0.0 < tonearm_limits.upper
        and (tonearm_limits.upper - tonearm_limits.lower) >= 0.70,
        details=f"unexpected tonearm limits: {tonearm_limits}",
    )

    with ctx.pose({tonearm_sweep: 0.25}):
        ctx.expect_gap(
            tonearm_stage,
            platter,
            axis="z",
            min_gap=0.001,
            max_gap=0.012,
            positive_elem="headshell",
            negative_elem="record_mat",
            name="headshell_tracks_above_record_mat",
        )

    lower_headshell = None
    upper_headshell = None
    if tonearm_limits is not None and tonearm_limits.lower is not None and tonearm_limits.upper is not None:
        with ctx.pose({tonearm_sweep: tonearm_limits.lower}):
            lower_headshell = _aabb_center(ctx.part_element_world_aabb(tonearm_stage, elem="headshell"))
        with ctx.pose({tonearm_sweep: tonearm_limits.upper}):
            upper_headshell = _aabb_center(ctx.part_element_world_aabb(tonearm_stage, elem="headshell"))

    sweep_ok = (
        lower_headshell is not None
        and upper_headshell is not None
        and abs(lower_headshell[0] - upper_headshell[0]) > 0.050
        and abs(lower_headshell[1] - upper_headshell[1]) > 0.050
    )
    ctx.check(
        "headshell_sweeps_across_a_real_arc",
        sweep_ok,
        details=f"lower={lower_headshell}, upper={upper_headshell}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
