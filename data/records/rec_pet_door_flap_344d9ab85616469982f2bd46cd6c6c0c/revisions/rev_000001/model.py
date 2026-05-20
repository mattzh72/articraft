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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


FRAME_OUTER_WIDTH = 0.245
FRAME_OUTER_HEIGHT = 0.305
FRAME_BEZEL_DEPTH = 0.018
FRAME_TUNNEL_DEPTH = 0.032
OPENING_WIDTH = 0.170
OPENING_HEIGHT = 0.215
SIDE_WALL_THICKNESS = 0.010
BOTTOM_SILL_THICKNESS = 0.010
HOOD_ROOF_THICKNESS = 0.015


def _make_frame_bezel_mesh():
    outer = rounded_rect_profile(FRAME_OUTER_WIDTH, FRAME_OUTER_HEIGHT, 0.018)
    inner = rounded_rect_profile(OPENING_WIDTH, OPENING_HEIGHT, 0.012)
    bezel = ExtrudeWithHolesGeometry(
        outer_profile=outer,
        hole_profiles=[inner],
        height=FRAME_BEZEL_DEPTH,
        cap=True,
        center=True,
        closed=True,
    )
    return mesh_from_geometry(bezel, "cat_flap_frame_bezel")


def _make_flap_panel_mesh():
    panel = ExtrudeGeometry(
        rounded_rect_profile(0.160, 0.200, 0.010),
        0.004,
        cap=True,
        center=True,
        closed=True,
    )
    return mesh_from_geometry(panel, "cat_flap_panel")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cat_flap_assembly")

    frame_plastic = model.material("frame_plastic", rgba=(0.90, 0.90, 0.88, 1.0))
    trim_plastic = model.material("trim_plastic", rgba=(0.80, 0.81, 0.79, 1.0))
    flap_tint = model.material("flap_tint", rgba=(0.36, 0.45, 0.50, 0.45))
    rubber_dark = model.material("rubber_dark", rgba=(0.18, 0.19, 0.20, 1.0))
    dial_dark = model.material("dial_dark", rgba=(0.17, 0.18, 0.19, 1.0))
    dial_mark = model.material("dial_mark", rgba=(0.78, 0.79, 0.80, 1.0))

    frame = model.part("frame")
    frame.visual(
        _make_frame_bezel_mesh(),
        origin=Origin(
            xyz=(0.0, FRAME_BEZEL_DEPTH * 0.5, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=frame_plastic,
        name="front_bezel",
    )
    frame.visual(
        Box((SIDE_WALL_THICKNESS, FRAME_TUNNEL_DEPTH, OPENING_HEIGHT)),
        origin=Origin(
            xyz=(
                -(OPENING_WIDTH * 0.5 + SIDE_WALL_THICKNESS * 0.5),
                FRAME_BEZEL_DEPTH + FRAME_TUNNEL_DEPTH * 0.5,
                0.0,
            )
        ),
        material=trim_plastic,
        name="left_tunnel_wall",
    )
    frame.visual(
        Box((SIDE_WALL_THICKNESS, FRAME_TUNNEL_DEPTH, OPENING_HEIGHT)),
        origin=Origin(
            xyz=(
                OPENING_WIDTH * 0.5 + SIDE_WALL_THICKNESS * 0.5,
                FRAME_BEZEL_DEPTH + FRAME_TUNNEL_DEPTH * 0.5,
                0.0,
            )
        ),
        material=trim_plastic,
        name="right_tunnel_wall",
    )
    frame.visual(
        Box((OPENING_WIDTH, FRAME_TUNNEL_DEPTH, BOTTOM_SILL_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                FRAME_BEZEL_DEPTH + FRAME_TUNNEL_DEPTH * 0.5,
                -(OPENING_HEIGHT * 0.5 + BOTTOM_SILL_THICKNESS * 0.5),
            )
        ),
        material=trim_plastic,
        name="bottom_tunnel_sill",
    )
    frame.visual(
        Box((OPENING_WIDTH + 0.020, FRAME_TUNNEL_DEPTH, HOOD_ROOF_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                FRAME_BEZEL_DEPTH + FRAME_TUNNEL_DEPTH * 0.5,
                OPENING_HEIGHT * 0.5 + HOOD_ROOF_THICKNESS * 0.5,
            )
        ),
        material=trim_plastic,
        name="hood_roof",
    )
    frame.visual(
        Box((OPENING_WIDTH + 0.020, FRAME_BEZEL_DEPTH, 0.022)),
        origin=Origin(
            xyz=(
                0.0,
                FRAME_BEZEL_DEPTH * 0.5,
                OPENING_HEIGHT * 0.5 - 0.0075,
            )
        ),
        material=trim_plastic,
        name="hood_lip",
    )
    frame.inertial = Inertial.from_geometry(
        Box(
            (
                FRAME_OUTER_WIDTH,
                FRAME_BEZEL_DEPTH + FRAME_TUNNEL_DEPTH,
                FRAME_OUTER_HEIGHT,
            )
        ),
        mass=0.65,
        origin=Origin(
            xyz=(0.0, (FRAME_BEZEL_DEPTH + FRAME_TUNNEL_DEPTH) * 0.5, 0.0)
        ),
    )

    flap = model.part("flap")
    flap.visual(
        _make_flap_panel_mesh(),
        origin=Origin(
            xyz=(0.0, 0.0, -0.100),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=flap_tint,
        name="panel",
    )
    flap.visual(
        Cylinder(radius=0.006, length=0.150),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_plastic,
        name="hinge_barrel",
    )
    flap.visual(
        Box((0.140, 0.008, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.195)),
        material=rubber_dark,
        name="bottom_sweep",
    )
    flap.inertial = Inertial.from_geometry(
        Box((0.160, 0.010, 0.206)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, -0.100)),
    )
    model.articulation(
        "frame_to_flap",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=flap,
        origin=Origin(xyz=(0.0, 0.024, 0.101)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.8,
            lower=-1.15,
            upper=1.15,
        ),
    )

    selector_dial = model.part("selector_dial")
    selector_dial.visual(
        Cylinder(radius=0.017, length=0.010),
        origin=Origin(xyz=(0.0, -0.005, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dial_dark,
        name="dial_body",
    )
    selector_dial.visual(
        Cylinder(radius=0.019, length=0.003),
        origin=Origin(
            xyz=(0.0, -0.0115, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=dial_dark,
        name="dial_face_ring",
    )
    selector_dial.visual(
        Cylinder(radius=0.007, length=0.004),
        origin=Origin(xyz=(0.0, -0.0135, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=trim_plastic,
        name="dial_cap",
    )
    selector_dial.visual(
        Box((0.005, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, -0.013, 0.011)),
        material=dial_mark,
        name="grip_top",
    )
    selector_dial.visual(
        Box((0.005, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, -0.013, -0.011)),
        material=dial_mark,
        name="grip_bottom",
    )
    selector_dial.visual(
        Box((0.010, 0.004, 0.005)),
        origin=Origin(xyz=(0.011, -0.013, 0.0)),
        material=dial_mark,
        name="grip_right",
    )
    selector_dial.visual(
        Box((0.010, 0.004, 0.005)),
        origin=Origin(xyz=(-0.011, -0.013, 0.0)),
        material=dial_mark,
        name="grip_left",
    )
    selector_dial.inertial = Inertial.from_geometry(
        Box((0.040, 0.016, 0.040)),
        mass=0.05,
        origin=Origin(xyz=(0.0, -0.008, 0.0)),
    )
    model.articulation(
        "frame_to_selector_dial",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=selector_dial,
        origin=Origin(xyz=(0.094, 0.0, -0.120)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=4.0,
            lower=-2.356,
            upper=2.356,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    flap = object_model.get_part("flap")
    selector_dial = object_model.get_part("selector_dial")
    flap_hinge = object_model.get_articulation("frame_to_flap")
    dial_joint = object_model.get_articulation("frame_to_selector_dial")

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

    flap_limits = flap_hinge.motion_limits
    dial_limits = dial_joint.motion_limits
    ctx.check(
        "flap hinge axis is horizontal",
        flap_hinge.axis == (1.0, 0.0, 0.0),
        details=f"expected (1, 0, 0), got {flap_hinge.axis}",
    )
    ctx.check(
        "flap swings both ways",
        flap_limits is not None
        and flap_limits.lower is not None
        and flap_limits.upper is not None
        and flap_limits.lower < 0.0
        and flap_limits.upper > 0.0,
        details=f"unexpected flap limits: {flap_limits}",
    )
    ctx.check(
        "selector dial rotates on frame-normal axis",
        dial_joint.axis == (0.0, 1.0, 0.0),
        details=f"expected (0, 1, 0), got {dial_joint.axis}",
    )
    ctx.check(
        "selector dial has multi-position travel",
        dial_limits is not None
        and dial_limits.lower is not None
        and dial_limits.upper is not None
        and dial_limits.upper - dial_limits.lower >= 4.0,
        details=f"unexpected dial limits: {dial_limits}",
    )
    ctx.expect_gap(
        frame,
        flap,
        axis="z",
        positive_elem="hood_roof",
        negative_elem="hinge_barrel",
        min_gap=0.0001,
        max_gap=0.010,
        name="hinge_barrel_stays_under_hood_roof",
    )
    ctx.expect_gap(
        flap,
        frame,
        axis="z",
        positive_elem="panel",
        negative_elem="bottom_tunnel_sill",
        min_gap=0.004,
        max_gap=0.020,
        name="flap_bottom_clears_sill",
    )
    ctx.expect_contact(
        frame,
        selector_dial,
        elem_a="front_bezel",
        elem_b="dial_body",
        contact_tol=1e-5,
        name="selector_dial_mounts_flush_to_frame",
    )
    ctx.expect_within(
        flap,
        frame,
        axes="xz",
        margin=0.0,
        name="flap_stays_within_frame_face",
    )
    ctx.expect_within(
        selector_dial,
        frame,
        axes="xz",
        margin=0.0,
        name="selector_dial_sits_on_lower_frame_corner",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
