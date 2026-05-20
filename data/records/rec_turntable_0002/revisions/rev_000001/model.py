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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    superellipse_profile,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _rounded_section(width: float, depth: float, radius: float, z: float):
    return [(x, y, z) for x, y in rounded_rect_profile(width, depth, radius, corner_segments=8)]


def _ring_mesh(
    name: str,
    *,
    outer_diameter: float,
    inner_diameter: float,
    height: float,
    outer_segments: int = 80,
    inner_segments: int = 40,
):
    geom = ExtrudeWithHolesGeometry(
        superellipse_profile(
            outer_diameter,
            outer_diameter,
            exponent=2.0,
            segments=outer_segments,
        ),
        [
            superellipse_profile(
                inner_diameter,
                inner_diameter,
                exponent=2.0,
                segments=inner_segments,
            )
        ],
        height=height,
        center=True,
    )
    return _save_mesh(name, geom)


def _add_fasteners(
    part,
    positions: list[tuple[float, float]],
    *,
    z: float,
    head_radius: float,
    head_height: float,
    material,
) -> None:
    for index, (x, y) in enumerate(positions):
        part.visual(
            Cylinder(radius=head_radius, length=head_height),
            origin=Origin(xyz=(x, y, z + head_height * 0.5)),
            material=material,
            name=f"fastener_head_{index:02d}",
        )
        part.visual(
            Box((head_radius * 1.5, head_radius * 0.22, head_height * 0.55)),
            origin=Origin(xyz=(x, y, z + head_height * 0.72)),
            material=material,
            name=f"fastener_slot_{index:02d}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_utility_turntable", assets=ASSETS)

    plinth_paint = model.material("plinth_paint", rgba=(0.19, 0.21, 0.24, 1.0))
    deck_finish = model.material("deck_finish", rgba=(0.29, 0.31, 0.34, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.07, 0.07, 0.08, 1.0))
    aluminum = model.material("aluminum", rgba=(0.76, 0.78, 0.81, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.61, 0.64, 0.67, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.23, 0.24, 0.26, 1.0))
    polymer = model.material("polymer", rgba=(0.12, 0.13, 0.15, 1.0))
    accent_orange = model.material("accent_orange", rgba=(0.86, 0.43, 0.15, 1.0))
    indicator_white = model.material("indicator_white", rgba=(0.94, 0.95, 0.96, 1.0))

    plinth_shell_mesh = _save_mesh(
        "turntable_plinth_shell.obj",
        section_loft(
            [
                _rounded_section(0.460, 0.370, 0.034, 0.012),
                _rounded_section(0.444, 0.354, 0.026, 0.070),
            ]
        ),
    )
    platter_disc_mesh = _ring_mesh(
        "turntable_platter_disc.obj",
        outer_diameter=0.332,
        inner_diameter=0.010,
        height=0.015,
    )
    platter_strobe_mesh = _ring_mesh(
        "turntable_platter_strobe.obj",
        outer_diameter=0.332,
        inner_diameter=0.296,
        height=0.004,
    )
    platter_mat_mesh = _ring_mesh(
        "turntable_rubber_mat.obj",
        outer_diameter=0.292,
        inner_diameter=0.010,
        height=0.003,
    )
    arm_tube_mesh = _save_mesh(
        "turntable_arm_tube.obj",
        tube_from_spline_points(
            [
                (0.000, 0.000, 0.000),
                (-0.050, 0.000, -0.0010),
                (-0.120, 0.000, -0.0027),
                (-0.185, 0.000, -0.0040),
                (-0.214, 0.000, -0.0048),
            ],
            radius=0.0052,
            samples_per_segment=16,
            radial_segments=16,
            cap_ends=True,
            up_hint=(0.0, 0.0, 1.0),
        ),
    )

    plinth = model.part("plinth")
    plinth.visual(plinth_shell_mesh, material=plinth_paint, name="plinth_shell")
    plinth.visual(
        Box((0.446, 0.356, 0.008)),
        origin=Origin(xyz=(0.000, 0.000, 0.074)),
        material=deck_finish,
        name="top_plate",
    )
    plinth.visual(
        Box((0.438, 0.028, 0.010)),
        origin=Origin(xyz=(0.000, -0.156, 0.083)),
        material=deck_finish,
        name="front_bumper",
    )
    plinth.visual(
        Box((0.018, 0.290, 0.012)),
        origin=Origin(xyz=(-0.213, -0.010, 0.084)),
        material=deck_finish,
        name="left_side_rail",
    )
    plinth.visual(
        Box((0.018, 0.290, 0.012)),
        origin=Origin(xyz=(0.213, -0.010, 0.084)),
        material=deck_finish,
        name="right_side_rail",
    )
    plinth.visual(
        Cylinder(radius=0.070, length=0.004),
        origin=Origin(xyz=(-0.020, 0.000, 0.080)),
        material=deck_finish,
        name="motor_pad",
    )
    plinth.visual(
        Cylinder(radius=0.056, length=0.008),
        origin=Origin(xyz=(-0.020, 0.000, 0.082)),
        material=dark_steel,
        name="bearing_collar",
    )
    plinth.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(xyz=(-0.020, 0.000, 0.081)),
        material=dark_steel,
        name="spindle_boss",
    )
    plinth.visual(
        Box((0.090, 0.084, 0.014)),
        origin=Origin(xyz=(0.166, -0.060, 0.085)),
        material=deck_finish,
        name="tonearm_reinforcement",
    )
    plinth.visual(
        Cylinder(radius=0.032, length=0.018),
        origin=Origin(xyz=(0.182, -0.060, 0.087)),
        material=dark_steel,
        name="tonearm_pedestal",
    )
    plinth.visual(
        Cylinder(radius=0.020, length=0.008),
        origin=Origin(xyz=(0.182, -0.060, 0.100)),
        material=satin_steel,
        name="tonearm_collar",
    )
    plinth.visual(
        Cylinder(radius=0.013, length=0.006),
        origin=Origin(xyz=(0.172, -0.106, 0.091)),
        material=dark_steel,
        name="anti_skate_mount",
    )
    plinth.visual(
        Box((0.032, 0.020, 0.003)),
        origin=Origin(xyz=(0.172, -0.126, 0.0935)),
        material=accent_orange,
        name="anti_skate_badge",
    )
    plinth.visual(
        Cylinder(radius=0.014, length=0.008),
        origin=Origin(xyz=(0.172, -0.106, 0.097)),
        material=polymer,
        name="anti_skate_dial",
    )
    plinth.visual(
        Box((0.012, 0.003, 0.004)),
        origin=Origin(xyz=(0.183, -0.106, 0.103)),
        material=indicator_white,
        name="anti_skate_pointer",
    )
    plinth.visual(
        Box((0.092, 0.036, 0.003)),
        origin=Origin(xyz=(0.168, -0.132, 0.0795)),
        material=dark_steel,
        name="pitch_track",
    )
    plinth.visual(
        Box((0.076, 0.008, 0.0015)),
        origin=Origin(xyz=(0.168, -0.132, 0.08225)),
        material=accent_orange,
        name="pitch_scale",
    )
    plinth.visual(
        Cylinder(radius=0.006, length=0.030),
        origin=Origin(xyz=(0.198, -0.124, 0.093)),
        material=satin_steel,
        name="arm_rest_post",
    )
    plinth.visual(
        Box((0.018, 0.028, 0.006)),
        origin=Origin(xyz=(0.198, -0.124, 0.109)),
        material=polymer,
        name="arm_rest_cradle",
    )
    plinth.visual(
        Box((0.008, 0.034, 0.020)),
        origin=Origin(xyz=(0.206, -0.124, 0.100)),
        material=polymer,
        name="arm_rest_backstop",
    )
    plinth.visual(
        Cylinder(radius=0.018, length=0.008),
        origin=Origin(xyz=(-0.182, 0.126, 0.082)),
        material=dark_steel,
        name="adapter_dock",
    )
    plinth.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(-0.182, 0.126, 0.092)),
        material=aluminum,
        name="adapter_puck",
    )
    for index, (x, y) in enumerate(((-0.170, -0.126), (0.170, -0.126), (-0.170, 0.126), (0.170, 0.126))):
        plinth.visual(
            Cylinder(radius=0.022, length=0.012),
            origin=Origin(xyz=(x, y, 0.006)),
            material=rubber_black,
            name=f"foot_{index}",
        )
    _add_fasteners(
        plinth,
        [
            (-0.195, -0.145),
            (0.195, -0.145),
            (-0.195, 0.145),
            (0.195, 0.145),
            (0.038, 0.132),
            (0.155, 0.036),
            (0.211, -0.040),
            (0.125, -0.098),
            (-0.092, 0.072),
            (-0.092, -0.072),
        ],
        z=0.078,
        head_radius=0.0046,
        head_height=0.0026,
        material=satin_steel,
    )
    plinth.inertial = Inertial.from_geometry(
        Box((0.460, 0.370, 0.110)),
        mass=7.8,
        origin=Origin(xyz=(0.000, 0.000, 0.055)),
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=0.0065, length=0.006),
        origin=Origin(xyz=(0.000, 0.000, 0.003)),
        material=dark_steel,
        name="spindle_collar",
    )
    spindle.visual(
        Cylinder(radius=0.0030, length=0.036),
        origin=Origin(xyz=(0.000, 0.000, 0.021)),
        material=satin_steel,
        name="spindle_shaft",
    )
    spindle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0065, length=0.042),
        mass=0.06,
        origin=Origin(xyz=(0.000, 0.000, 0.021)),
    )

    platter = model.part("platter")
    platter.visual(
        Cylinder(radius=0.055, length=0.006),
        origin=Origin(xyz=(0.000, 0.000, 0.003)),
        material=dark_steel,
        name="hub_flange",
    )
    platter.visual(
        platter_disc_mesh,
        origin=Origin(xyz=(0.000, 0.000, 0.0135)),
        material=aluminum,
        name="platter_disc",
    )
    platter.visual(
        platter_strobe_mesh,
        origin=Origin(xyz=(0.000, 0.000, 0.0100)),
        material=satin_steel,
        name="strobe_ring",
    )
    platter.visual(
        platter_mat_mesh,
        origin=Origin(xyz=(0.000, 0.000, 0.0225)),
        material=rubber_black,
        name="mat",
    )
    platter.visual(
        Box((0.010, 0.004, 0.001)),
        origin=Origin(xyz=(0.138, 0.000, 0.0245)),
        material=indicator_white,
        name="strobe_dot",
    )
    platter.inertial = Inertial.from_geometry(
        Cylinder(radius=0.166, length=0.029),
        mass=1.6,
        origin=Origin(xyz=(0.000, 0.000, 0.0145)),
    )

    tonearm_carriage = model.part("tonearm_carriage")
    tonearm_carriage.visual(
        Cylinder(radius=0.020, length=0.008),
        origin=Origin(xyz=(0.000, 0.000, 0.004)),
        material=satin_steel,
        name="base_collar",
    )
    tonearm_carriage.visual(
        Cylinder(radius=0.008, length=0.024),
        origin=Origin(xyz=(0.000, 0.000, 0.016)),
        material=dark_steel,
        name="pivot_pillar",
    )
    tonearm_carriage.visual(
        Box((0.020, 0.026, 0.006)),
        origin=Origin(xyz=(0.004, 0.000, 0.024)),
        material=satin_steel,
        name="bridge_plate",
    )
    tonearm_carriage.visual(
        Box((0.010, 0.004, 0.018)),
        origin=Origin(xyz=(0.002, -0.007, 0.034)),
        material=satin_steel,
        name="left_cheek",
    )
    tonearm_carriage.visual(
        Box((0.010, 0.004, 0.018)),
        origin=Origin(xyz=(0.002, 0.007, 0.034)),
        material=satin_steel,
        name="right_cheek",
    )
    tonearm_carriage.inertial = Inertial.from_geometry(
        Box((0.050, 0.036, 0.040)),
        mass=0.18,
        origin=Origin(xyz=(0.006, 0.000, 0.020)),
    )

    tonearm_tube = model.part("tonearm_tube")
    tonearm_tube.visual(
        Cylinder(radius=0.0034, length=0.010),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="pivot_pin",
    )
    tonearm_tube.visual(
        Box((0.008, 0.012, 0.012)),
        origin=Origin(xyz=(-0.004, 0.000, 0.000)),
        material=dark_steel,
        name="pivot_block",
    )
    tonearm_tube.visual(
        Cylinder(radius=0.0052, length=0.220),
        origin=Origin(xyz=(-0.110, 0.000, -0.0005), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="arm_tube",
    )
    tonearm_tube.visual(
        Box((0.028, 0.018, 0.004)),
        origin=Origin(xyz=(-0.218, 0.000, -0.0045)),
        material=aluminum,
        name="headshell",
    )
    tonearm_tube.visual(
        Box((0.012, 0.010, 0.010)),
        origin=Origin(xyz=(-0.232, 0.000, -0.007)),
        material=polymer,
        name="cartridge",
    )
    tonearm_tube.visual(
        Box((0.018, 0.0035, 0.0035)),
        origin=Origin(xyz=(-0.206, 0.0090, -0.002)),
        material=aluminum,
        name="finger_lift",
    )
    tonearm_tube.visual(
        Cylinder(radius=0.0010, length=0.009),
        origin=Origin(xyz=(-0.232, 0.000, -0.016)),
        material=satin_steel,
        name="stylus",
    )
    tonearm_tube.visual(
        Cylinder(radius=0.0045, length=0.020),
        origin=Origin(xyz=(0.010, 0.000, -0.0005), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="counterweight_stub",
    )
    tonearm_tube.visual(
        Cylinder(radius=0.010, length=0.018),
        origin=Origin(xyz=(0.029, 0.000, -0.0005), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="counterweight_lock",
    )
    tonearm_tube.visual(
        Cylinder(radius=0.015, length=0.030),
        origin=Origin(xyz=(0.053, 0.000, -0.0005), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="counterweight",
    )
    tonearm_tube.inertial = Inertial.from_geometry(
        Box((0.300, 0.040, 0.040)),
        mass=0.24,
        origin=Origin(xyz=(-0.074, 0.000, -0.002)),
    )

    pitch_slider = model.part("pitch_slider")
    pitch_slider.visual(
        Box((0.016, 0.034, 0.003)),
        origin=Origin(xyz=(0.000, 0.000, 0.0015)),
        material=polymer,
        name="slider_shoe",
    )
    pitch_slider.visual(
        Box((0.016, 0.024, 0.008)),
        origin=Origin(xyz=(0.000, 0.000, 0.007)),
        material=deck_finish,
        name="slider_knob",
    )
    pitch_slider.visual(
        Box((0.010, 0.016, 0.004)),
        origin=Origin(xyz=(0.000, 0.000, 0.013)),
        material=accent_orange,
        name="slider_grip",
    )
    pitch_slider.inertial = Inertial.from_geometry(
        Box((0.016, 0.034, 0.016)),
        mass=0.05,
        origin=Origin(xyz=(0.000, 0.000, 0.008)),
    )

    model.articulation(
        "plinth_to_spindle",
        ArticulationType.FIXED,
        parent=plinth,
        child=spindle,
        origin=Origin(xyz=(-0.020, 0.000, 0.084)),
    )
    model.articulation(
        "plinth_to_platter",
        ArticulationType.CONTINUOUS,
        parent=plinth,
        child=platter,
        origin=Origin(xyz=(-0.020, 0.000, 0.086)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.5, velocity=12.0),
    )
    model.articulation(
        "plinth_to_tonearm_carriage",
        ArticulationType.REVOLUTE,
        parent=plinth,
        child=tonearm_carriage,
        origin=Origin(xyz=(0.182, -0.060, 0.104), rpy=(0.0, 0.0, math.radians(47.0))),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=2.0,
            lower=-1.10,
            upper=0.06,
        ),
    )
    model.articulation(
        "tonearm_carriage_to_tube",
        ArticulationType.REVOLUTE,
        parent=tonearm_carriage,
        child=tonearm_tube,
        origin=Origin(xyz=(0.002, 0.000, 0.034)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=1.0,
            lower=0.0,
            upper=0.08,
        ),
    )
    model.articulation(
        "plinth_to_pitch_slider",
        ArticulationType.PRISMATIC,
        parent=plinth,
        child=pitch_slider,
        origin=Origin(xyz=(0.168, -0.132, 0.081)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=0.08,
            lower=-0.020,
            upper=0.020,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)

    plinth = object_model.get_part("plinth")
    spindle = object_model.get_part("spindle")
    platter = object_model.get_part("platter")
    tonearm_carriage = object_model.get_part("tonearm_carriage")
    tonearm_tube = object_model.get_part("tonearm_tube")
    pitch_slider = object_model.get_part("pitch_slider")

    platter_spin = object_model.get_articulation("plinth_to_platter")
    tonearm_swing = object_model.get_articulation("plinth_to_tonearm_carriage")
    tonearm_lift = object_model.get_articulation("tonearm_carriage_to_tube")
    pitch_adjust = object_model.get_articulation("plinth_to_pitch_slider")

    def center_of(part, elem: str) -> tuple[float, float, float]:
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        assert aabb is not None
        return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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

    ctx.expect_contact(spindle, plinth, elem_a="spindle_collar", elem_b="spindle_boss")
    ctx.expect_contact(platter, plinth, elem_a="hub_flange", elem_b="bearing_collar")
    ctx.expect_contact(tonearm_carriage, plinth, elem_a="base_collar", elem_b="tonearm_collar")
    ctx.expect_contact(
        tonearm_tube,
        tonearm_carriage,
        elem_a="pivot_pin",
        elem_b="left_cheek",
        name="tonearm_pin_left_contact",
    )
    ctx.expect_contact(
        tonearm_tube,
        tonearm_carriage,
        elem_a="pivot_pin",
        elem_b="right_cheek",
        name="tonearm_pin_right_contact",
    )
    ctx.expect_contact(pitch_slider, plinth, elem_a="slider_shoe", elem_b="pitch_track")
    ctx.expect_overlap(platter, plinth, axes="xy", min_overlap=0.28)
    ctx.expect_gap(platter, plinth, axis="z", min_gap=0.028, max_gap=0.034, positive_elem="mat", negative_elem="top_plate")

    dot_rest = center_of(platter, "strobe_dot")
    with ctx.pose({platter_spin: math.pi / 2.0}):
        dot_turn = center_of(platter, "strobe_dot")
        assert abs(dot_turn[0] - dot_rest[0]) > 0.09
        assert abs(dot_turn[1] - dot_rest[1]) > 0.09
        ctx.expect_contact(platter, plinth, elem_a="hub_flange", elem_b="bearing_collar")

    rest_head = center_of(tonearm_tube, "headshell")
    assert rest_head[1] < -0.18
    with ctx.pose({tonearm_swing: -1.05, tonearm_lift: 0.0}):
        play_head = center_of(tonearm_tube, "headshell")
        assert play_head[0] < rest_head[0] - 0.05
        assert play_head[1] > rest_head[1] + 0.18
        ctx.expect_overlap(tonearm_tube, platter, axes="xy", min_overlap=0.010, elem_a="headshell")
        ctx.expect_gap(
            tonearm_tube,
            platter,
            axis="z",
            min_gap=0.0015,
            positive_elem="cartridge",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="tonearm_play_pose_clear")

    with ctx.pose({tonearm_swing: -1.05, tonearm_lift: 0.16}):
        lifted_head = center_of(tonearm_tube, "headshell")
        assert lifted_head[2] > play_head[2] + 0.015
        ctx.expect_gap(
            tonearm_tube,
            platter,
            axis="z",
            min_gap=0.016,
            positive_elem="cartridge",
            name="tonearm_lifted_record_clearance",
        )

    with ctx.pose({pitch_adjust: -0.02}):
        pitch_low = ctx.part_world_position(pitch_slider)
        assert pitch_low is not None
    with ctx.pose({pitch_adjust: 0.02}):
        pitch_high = ctx.part_world_position(pitch_slider)
        assert pitch_high is not None
        assert pitch_high[0] > pitch_low[0] + 0.039
        ctx.expect_contact(pitch_slider, plinth, elem_a="slider_shoe", elem_b="pitch_track")

    for articulation in (tonearm_swing, tonearm_lift, pitch_adjust):
        limits = articulation.motion_limits
        assert limits is not None
        assert limits.lower is not None and limits.upper is not None
        with ctx.pose({articulation: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{articulation.name}_lower_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{articulation.name}_lower_no_floating")
        with ctx.pose({articulation: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{articulation.name}_upper_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{articulation.name}_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
