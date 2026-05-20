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
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)

FACE_THICKNESS = 0.060
FACE_WIDTH = 0.620
FACE_HEIGHT = 0.780
OPENING_WIDTH = 0.340
OPENING_HEIGHT = 0.500
BEZEL_WIDTH = 0.472
BEZEL_HEIGHT = 0.632
BEZEL_THICKNESS = 0.012

PANEL_THICKNESS = 0.006
PANEL_WIDTH = 0.470
PANEL_HEIGHT = 0.640
PANEL_RETURN_DEPTH = 0.003
PANEL_RETURN_STRIP = 0.018
PANEL_RETURN_SIDE_HEIGHT = OPENING_HEIGHT - 0.068
PANEL_RETURN_TOP_WIDTH = OPENING_WIDTH - 0.070
PANEL_RETURN_SIDE_OFFSET_Y = 0.151
PANEL_RETURN_TOP_OFFSET_Z = 0.229

HINGE_AXIS_X = 0.050
HINGE_AXIS_Y = -0.252
PANEL_CENTER_Y = -HINGE_AXIS_Y
FRAME_FASTENER_CENTER_X = 0.032
PANEL_FASTENER_CENTER_X = -0.00625
PANEL_HINGE_FASTENER_CENTER_X = -0.0065

LATCH_EDGE_Y = PANEL_CENTER_Y + (PANEL_WIDTH * 0.5) - 0.040
LATCH_UPPER_Z = 0.125
LATCH_LOWER_Z = -0.125
LATCH_AXIS_LOCAL_X = -0.0025
LATCH_PIN_RADIUS = 0.004
LATCH_SLEEVE_OUTER_RADIUS = 0.0105
LATCH_SLEEVE_INNER_RADIUS = 0.0046
LATCH_SLEEVE_LENGTH = 0.008

HINGE_PIN_RADIUS = 0.0045
HINGE_SLEEVE_OUTER_RADIUS = 0.0105
HINGE_SLEEVE_INNER_RADIUS = 0.0056
HINGE_PANEL_SLEEVE_LENGTH = 0.220
HINGE_FRAME_SLEEVE_LENGTH = 0.110


def _save_mesh(geometry: MeshGeometry, filename: str):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def _box_ring(
    outer_size: tuple[float, float, float],
    inner_yz: tuple[float, float],
    *,
    through_x_margin: float = 0.004,
) -> MeshGeometry:
    outer = BoxGeometry(outer_size)
    inner = BoxGeometry(
        (
            outer_size[0] + through_x_margin,
            inner_yz[0],
            inner_yz[1],
        )
    )
    return boolean_difference(outer, inner)


def _cylindrical_sleeve(
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    radial_segments: int = 28,
) -> MeshGeometry:
    outer = CylinderGeometry(radius=outer_radius, height=length, radial_segments=radial_segments)
    inner = CylinderGeometry(
        radius=inner_radius,
        height=length + 0.004,
        radial_segments=radial_segments,
    )
    return boolean_difference(outer, inner)


def _add_bolt_head(
    part,
    *,
    xyz: tuple[float, float, float],
    radius: float,
    height: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=height),
        origin=Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_service_access_panel", assets=ASSETS)

    utility_paint = model.material("utility_paint", rgba=(0.36, 0.40, 0.35, 1.0))
    utility_paint_dark = model.material("utility_paint_dark", rgba=(0.28, 0.31, 0.27, 1.0))
    molded_polymer = model.material("molded_polymer", rgba=(0.14, 0.15, 0.16, 1.0))
    fastener_metal = model.material("fastener_metal", rgba=(0.62, 0.64, 0.67, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.44, 0.47, 0.50, 1.0))
    gasket_black = model.material("gasket_black", rgba=(0.08, 0.08, 0.08, 1.0))

    enclosure = model.part("enclosure_face")
    enclosure.visual(
        _save_mesh(
            _box_ring(
                (FACE_THICKNESS, FACE_WIDTH, FACE_HEIGHT),
                (OPENING_WIDTH, OPENING_HEIGHT),
            ),
            "enclosure_frame.obj",
        ),
        material=utility_paint,
        name="face_frame",
    )
    enclosure.visual(
        _save_mesh(
            _box_ring(
                (BEZEL_THICKNESS, BEZEL_WIDTH, BEZEL_HEIGHT),
                (OPENING_WIDTH + 0.022, OPENING_HEIGHT + 0.022),
            ),
            "opening_bezel.obj",
        ),
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
        material=utility_paint_dark,
        name="opening_bezel",
    )
    enclosure.visual(
        _save_mesh(
            _box_ring(
                (0.002, OPENING_WIDTH + 0.034, OPENING_HEIGHT + 0.034),
                (OPENING_WIDTH + 0.006, OPENING_HEIGHT + 0.006),
                through_x_margin=0.002,
            ),
            "opening_gasket.obj",
        ),
        origin=Origin(xyz=(FACE_THICKNESS * 0.5 + 0.001, 0.0, 0.0)),
        material=gasket_black,
        name="opening_gasket",
    )
    enclosure.visual(
        _save_mesh(
            _box_ring(
                (0.018, OPENING_WIDTH + 0.056, OPENING_HEIGHT + 0.056),
                (OPENING_WIDTH + 0.010, OPENING_HEIGHT + 0.010),
                through_x_margin=0.003,
            ),
            "inner_jamb_frame.obj",
        ),
        origin=Origin(xyz=(-0.010, 0.0, 0.0)),
        material=utility_paint_dark,
        name="inner_jamb_frame",
    )
    enclosure.visual(
        Box((0.018, 0.070, 0.620)),
        origin=Origin(xyz=(0.010, -0.218, 0.0)),
        material=utility_paint_dark,
        name="hinge_side_reinforcement",
    )
    enclosure.visual(
        Box((0.018, 0.062, 0.500)),
        origin=Origin(xyz=(0.010, 0.197, 0.0)),
        material=utility_paint_dark,
        name="latch_side_reinforcement",
    )
    enclosure.visual(
        _save_mesh(
            _cylindrical_sleeve(
                outer_radius=HINGE_SLEEVE_OUTER_RADIUS,
                inner_radius=HINGE_PIN_RADIUS,
                length=HINGE_FRAME_SLEEVE_LENGTH,
            ),
            "upper_hinge_knuckle.obj",
        ),
        origin=Origin(xyz=(HINGE_AXIS_X, HINGE_AXIS_Y, 0.185)),
        material=hinge_metal,
        name="upper_hinge_knuckle",
    )
    enclosure.visual(
        _save_mesh(
            _cylindrical_sleeve(
                outer_radius=HINGE_SLEEVE_OUTER_RADIUS,
                inner_radius=HINGE_PIN_RADIUS,
                length=HINGE_FRAME_SLEEVE_LENGTH,
            ),
            "lower_hinge_knuckle.obj",
        ),
        origin=Origin(xyz=(HINGE_AXIS_X, HINGE_AXIS_Y, -0.185)),
        material=hinge_metal,
        name="lower_hinge_knuckle",
    )
    enclosure.visual(
        Cylinder(radius=HINGE_PIN_RADIUS, length=0.560),
        origin=Origin(xyz=(HINGE_AXIS_X, HINGE_AXIS_Y, 0.0)),
        material=fastener_metal,
        name="hinge_pin",
    )
    enclosure.visual(
        Box((0.024, 0.042, 0.138)),
        origin=Origin(xyz=(0.038, -0.273, 0.185)),
        material=utility_paint_dark,
        name="upper_hinge_bracket",
    )
    enclosure.visual(
        Box((0.024, 0.042, 0.138)),
        origin=Origin(xyz=(0.038, -0.273, -0.185)),
        material=utility_paint_dark,
        name="lower_hinge_bracket",
    )
    enclosure.visual(
        Box((0.020, 0.040, 0.190)),
        origin=Origin(xyz=(0.014, 0.216, 0.120)),
        material=molded_polymer,
        name="upper_strike_block",
    )
    enclosure.visual(
        Box((0.020, 0.040, 0.190)),
        origin=Origin(xyz=(0.014, 0.216, -0.120)),
        material=molded_polymer,
        name="lower_strike_block",
    )
    enclosure.visual(
        Box((0.016, 0.080, 0.420)),
        origin=Origin(xyz=(0.008, 0.194, 0.0)),
        material=utility_paint_dark,
        name="latch_rail",
    )
    for index, (bolt_y, bolt_z) in enumerate(
        (
            (-0.236, 0.248),
            (-0.236, 0.122),
            (-0.236, -0.122),
            (-0.236, -0.248),
            (0.214, 0.220),
            (0.214, 0.020),
            (0.214, -0.020),
            (0.214, -0.220),
            (-0.145, 0.302),
            (0.145, 0.302),
            (-0.145, -0.302),
            (0.145, -0.302),
        ),
        start=1,
    ):
        _add_bolt_head(
            enclosure,
            xyz=(FRAME_FASTENER_CENTER_X, bolt_y, bolt_z),
            radius=0.0075,
            height=0.005,
            material=fastener_metal,
            name=f"frame_bolt_{index:02d}",
        )
    enclosure.inertial = Inertial.from_geometry(
        Box((0.100, FACE_WIDTH, FACE_HEIGHT)),
        mass=14.5,
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
    )

    panel = model.part("service_panel")
    panel.visual(
        Box((PANEL_THICKNESS, PANEL_WIDTH, PANEL_HEIGHT)),
        origin=Origin(xyz=(-0.0115, PANEL_CENTER_Y, 0.0)),
        material=utility_paint,
        name="panel_skin",
    )
    panel.visual(
        Box((PANEL_RETURN_DEPTH, PANEL_RETURN_STRIP, PANEL_RETURN_SIDE_HEIGHT)),
        origin=Origin(xyz=(-0.0160, PANEL_CENTER_Y - PANEL_RETURN_SIDE_OFFSET_Y, 0.0)),
        material=utility_paint_dark,
        name="panel_return_hinge",
    )
    panel.visual(
        Box((PANEL_RETURN_DEPTH, PANEL_RETURN_STRIP, PANEL_RETURN_SIDE_HEIGHT)),
        origin=Origin(xyz=(-0.0160, PANEL_CENTER_Y + PANEL_RETURN_SIDE_OFFSET_Y, 0.0)),
        material=utility_paint_dark,
        name="panel_return_latch",
    )
    panel.visual(
        Box((PANEL_RETURN_DEPTH, PANEL_RETURN_TOP_WIDTH, PANEL_RETURN_STRIP)),
        origin=Origin(xyz=(-0.0160, PANEL_CENTER_Y, PANEL_RETURN_TOP_OFFSET_Z)),
        material=utility_paint_dark,
        name="panel_return_top",
    )
    panel.visual(
        Box((PANEL_RETURN_DEPTH, PANEL_RETURN_TOP_WIDTH, PANEL_RETURN_STRIP)),
        origin=Origin(xyz=(-0.0160, PANEL_CENTER_Y, -PANEL_RETURN_TOP_OFFSET_Z)),
        material=utility_paint_dark,
        name="panel_return_bottom",
    )
    panel.visual(
        _save_mesh(
            _box_ring(
                (0.006, 0.284, 0.444),
                (0.180, 0.320),
                through_x_margin=0.003,
            ),
            "panel_reinforcement_ring.obj",
        ),
        origin=Origin(xyz=(-0.0145, PANEL_CENTER_Y, 0.0)),
        material=utility_paint_dark,
        name="panel_reinforcement_ring",
    )
    panel.visual(
        Box((0.008, 0.044, 0.402)),
        origin=Origin(xyz=(-0.0155, PANEL_CENTER_Y - 0.022, 0.0)),
        material=utility_paint_dark,
        name="vertical_stiffener",
    )
    panel.visual(
        Box((0.008, 0.224, 0.046)),
        origin=Origin(xyz=(-0.0155, PANEL_CENTER_Y - 0.016, 0.0)),
        material=utility_paint_dark,
        name="horizontal_stiffener",
    )
    panel.visual(
        Box((0.008, 0.032, PANEL_HEIGHT - 0.170)),
        origin=Origin(xyz=(-0.0135, LATCH_EDGE_Y - 0.004, 0.0)),
        material=utility_paint_dark,
        name="latch_doubler",
    )
    panel.visual(
        Box((0.004, 0.018, 0.230)),
        origin=Origin(xyz=(-0.0075, 0.0110, 0.0)),
        material=utility_paint_dark,
        name="panel_hinge_strap",
    )
    panel.visual(
        Box((0.006, 0.042, 0.110)),
        origin=Origin(xyz=(-0.0145, LATCH_EDGE_Y, LATCH_UPPER_Z)),
        material=utility_paint_dark,
        name="upper_latch_backer",
    )
    panel.visual(
        Box((0.006, 0.042, 0.110)),
        origin=Origin(xyz=(-0.0145, LATCH_EDGE_Y, LATCH_LOWER_Z)),
        material=utility_paint_dark,
        name="lower_latch_backer",
    )
    panel.visual(
        Cylinder(radius=LATCH_PIN_RADIUS, length=0.010),
        origin=Origin(xyz=(-0.0050, LATCH_EDGE_Y, LATCH_UPPER_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_metal,
        name="upper_latch_pin",
    )
    panel.visual(
        Cylinder(radius=LATCH_PIN_RADIUS, length=0.010),
        origin=Origin(xyz=(-0.0050, LATCH_EDGE_Y, LATCH_LOWER_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_metal,
        name="lower_latch_pin",
    )
    panel.visual(
        _save_mesh(
            _cylindrical_sleeve(
                outer_radius=HINGE_SLEEVE_OUTER_RADIUS,
                inner_radius=HINGE_SLEEVE_INNER_RADIUS,
                length=HINGE_PANEL_SLEEVE_LENGTH,
            ),
            "panel_hinge_sleeve.obj",
        ),
        origin=Origin(),
        material=hinge_metal,
        name="panel_hinge_sleeve",
    )
    for index, bolt_z in enumerate((0.205, 0.075, -0.075, -0.205), start=1):
        _add_bolt_head(
            panel,
            xyz=(PANEL_FASTENER_CENTER_X, PANEL_CENTER_Y + (PANEL_WIDTH * 0.5) - 0.024, bolt_z),
            radius=0.007,
            height=0.0045,
            material=fastener_metal,
            name=f"panel_edge_bolt_{index:02d}",
        )
    panel.inertial = Inertial.from_geometry(
        Box((0.070, PANEL_WIDTH + 0.040, PANEL_HEIGHT)),
        mass=6.8,
        origin=Origin(xyz=(-0.006, PANEL_CENTER_Y, 0.0)),
    )

    model.articulation(
        "panel_hinge",
        ArticulationType.REVOLUTE,
        parent=enclosure,
        child=panel,
        origin=Origin(xyz=(HINGE_AXIS_X, HINGE_AXIS_Y, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=32.0,
            velocity=1.6,
            lower=0.0,
            upper=2.05,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    enclosure = object_model.get_part("enclosure_face")
    panel = object_model.get_part("service_panel")
    panel_hinge = object_model.get_articulation("panel_hinge")

    face_frame = enclosure.get_visual("face_frame")
    opening_bezel = enclosure.get_visual("opening_bezel")
    opening_gasket = enclosure.get_visual("opening_gasket")
    inner_jamb_frame = enclosure.get_visual("inner_jamb_frame")
    hinge_pin = enclosure.get_visual("hinge_pin")
    upper_strike_block = enclosure.get_visual("upper_strike_block")
    lower_strike_block = enclosure.get_visual("lower_strike_block")

    panel_skin = panel.get_visual("panel_skin")
    panel_return_hinge = panel.get_visual("panel_return_hinge")
    panel_return_latch = panel.get_visual("panel_return_latch")
    panel_return_top = panel.get_visual("panel_return_top")
    panel_return_bottom = panel.get_visual("panel_return_bottom")
    panel_sleeve = panel.get_visual("panel_hinge_sleeve")
    panel_ring = panel.get_visual("panel_reinforcement_ring")
    upper_latch_backer = panel.get_visual("upper_latch_backer")
    lower_latch_backer = panel.get_visual("lower_latch_backer")
    upper_latch_pin = panel.get_visual("upper_latch_pin")
    lower_latch_pin = panel.get_visual("lower_latch_pin")

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=48)

    ctx.check(
        "panel_hinge_axis_is_vertical",
        tuple(panel_hinge.axis) == (0.0, 0.0, -1.0),
        details=f"panel hinge axis was {panel_hinge.axis}",
    )

    ctx.expect_overlap(
        panel,
        enclosure,
        axes="yz",
        min_overlap=0.360,
        elem_a=panel_skin,
        elem_b=opening_bezel,
        name="panel_covers_bezel_footprint",
    )
    ctx.expect_within(
        panel,
        enclosure,
        axes="yz",
        inner_elem=panel_skin,
        outer_elem=opening_bezel,
        margin=0.045,
        name="panel_stays_within_outer_bezel",
    )
    ctx.expect_within(
        panel,
        enclosure,
        axes="yz",
        inner_elem=panel_return_hinge,
        outer_elem=opening_gasket,
        margin=0.006,
        name="panel_hinge_return_stays_inside_opening",
    )
    ctx.expect_within(
        panel,
        enclosure,
        axes="yz",
        inner_elem=panel_return_latch,
        outer_elem=opening_gasket,
        margin=0.006,
        name="panel_latch_return_stays_inside_opening",
    )
    ctx.expect_within(
        panel,
        enclosure,
        axes="yz",
        inner_elem=panel_return_top,
        outer_elem=opening_gasket,
        margin=0.006,
        name="panel_top_return_stays_inside_opening",
    )
    ctx.expect_within(
        panel,
        enclosure,
        axes="yz",
        inner_elem=panel_return_bottom,
        outer_elem=opening_gasket,
        margin=0.006,
        name="panel_bottom_return_stays_inside_opening",
    )
    ctx.expect_gap(
        panel,
        enclosure,
        axis="x",
        min_gap=0.0015,
        max_gap=0.0045,
        positive_elem=panel_skin,
        negative_elem=opening_gasket,
        name="panel_seats_just_proud_of_gasket",
    )
    ctx.expect_within(
        enclosure,
        panel,
        axes="xy",
        inner_elem=hinge_pin,
        outer_elem=panel_sleeve,
        margin=0.0013,
        name="hinge_pin_runs_inside_panel_sleeve",
    )
    ctx.expect_overlap(
        enclosure,
        panel,
        axes="z",
        elem_a=hinge_pin,
        elem_b=panel_sleeve,
        min_overlap=0.200,
        name="hinge_pin_and_sleeve_share_vertical_engagement",
    )
    ctx.expect_overlap(
        panel,
        enclosure,
        axes="yz",
        elem_a=upper_latch_backer,
        elem_b=upper_strike_block,
        min_overlap=0.018,
        name="upper_latch_zone_aligns_with_strike",
    )
    ctx.expect_overlap(
        panel,
        enclosure,
        axes="yz",
        elem_a=lower_latch_backer,
        elem_b=lower_strike_block,
        min_overlap=0.018,
        name="lower_latch_zone_aligns_with_strike",
    )
    ctx.expect_gap(
        panel,
        enclosure,
        axis="x",
        min_gap=0.006,
        max_gap=0.0145,
        positive_elem=upper_latch_pin,
        negative_elem=opening_bezel,
        name="upper_latch_receptacle_sits_proud_of_bezel",
    )
    ctx.expect_gap(
        panel,
        enclosure,
        axis="x",
        min_gap=0.006,
        max_gap=0.0145,
        positive_elem=lower_latch_pin,
        negative_elem=opening_bezel,
        name="lower_latch_receptacle_sits_proud_of_bezel",
    )
    ctx.expect_overlap(
        panel,
        enclosure,
        axes="yz",
        elem_a=panel_ring,
        elem_b=inner_jamb_frame,
        min_overlap=0.220,
        name="panel_reinforcement_reads_centered_in_opening",
    )

    with ctx.pose({panel_hinge: 1.35}):
        ctx.expect_within(
            enclosure,
            panel,
            axes="xy",
            inner_elem=hinge_pin,
            outer_elem=panel_sleeve,
            margin=0.0013,
            name="hinge_pin_remains_captured_when_open",
        )
        ctx.expect_gap(
            panel,
            enclosure,
            axis="x",
            min_gap=0.140,
            positive_elem=panel_return_latch,
            negative_elem=opening_bezel,
            name="latch_edge_swings_clear_of_frame_when_open",
        )
        ctx.expect_overlap(
            panel,
            enclosure,
            axes="z",
            elem_a=panel_skin,
            elem_b=face_frame,
            min_overlap=0.420,
            name="panel_retains_vertical_span_in_open_pose",
        )

    limits = panel_hinge.motion_limits
    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({panel_hinge: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="panel_hinge_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="panel_hinge_lower_no_floating")
        with ctx.pose({panel_hinge: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="panel_hinge_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="panel_hinge_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
