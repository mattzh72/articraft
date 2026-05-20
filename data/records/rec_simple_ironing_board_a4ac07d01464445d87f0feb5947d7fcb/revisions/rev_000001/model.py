from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


Point3 = tuple[float, float, float]
Point2 = tuple[float, float]


def _midpoint(a: Point3, b: Point3) -> Point3:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _rod_origin(a: Point3, b: Point3) -> tuple[float, Origin]:
    vx = b[0] - a[0]
    vy = b[1] - a[1]
    vz = b[2] - a[2]
    length = sqrt(vx * vx + vy * vy + vz * vz)
    yaw = atan2(vy, vx)
    pitch = atan2(sqrt(vx * vx + vy * vy), vz)
    return length, Origin(xyz=_midpoint(a, b), rpy=(0.0, pitch, yaw))


def _add_rod(part, name: str, a: Point3, b: Point3, radius: float, material) -> None:
    length, origin = _rod_origin(a, b)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=origin,
        material=material,
        name=name,
    )


def _board_profile() -> list[Point2]:
    return [
        (-0.40, -0.125),
        (-0.16, -0.125),
        (0.06, -0.117),
        (0.20, -0.100),
        (0.31, -0.074),
        (0.37, -0.042),
        (0.402, -0.014),
        (0.410, 0.000),
        (0.402, 0.014),
        (0.37, 0.042),
        (0.31, 0.074),
        (0.20, 0.100),
        (0.06, 0.117),
        (-0.16, 0.125),
        (-0.40, 0.125),
        (-0.418, 0.100),
        (-0.425, 0.060),
        (-0.425, -0.060),
        (-0.418, -0.100),
    ]


def _scale_profile(profile: list[Point2], sx: float, sy: float) -> list[Point2]:
    return [(x * sx, y * sy) for x, y in profile]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_ironing_board")

    cover = model.material("cover_fabric", rgba=(0.47, 0.58, 0.70, 1.0))
    shell = model.material("shell", rgba=(0.89, 0.89, 0.87, 1.0))
    steel = model.material("steel", rgba=(0.73, 0.74, 0.76, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.34, 0.36, 0.39, 1.0))
    rubber = model.material("rubber", rgba=(0.17, 0.17, 0.18, 1.0))

    board_profile = _board_profile()
    deck_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(board_profile, 0.012),
        "deck_panel",
    )
    cover_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(_scale_profile(board_profile, 0.988, 0.972), 0.004),
        "deck_cover",
    )

    deck = model.part("deck")
    deck.visual(deck_mesh, material=shell, name="deck_panel")
    deck.visual(
        cover_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0115)),
        material=cover,
        name="cover_pad",
    )
    deck.visual(
        Box((0.32, 0.055, 0.016)),
        origin=Origin(xyz=(0.000, 0.0, -0.008)),
        material=dark_steel,
        name="underside_rail",
    )
    deck.visual(
        Box((0.100, 0.190, 0.010)),
        origin=Origin(xyz=(-0.150, 0.0, -0.005)),
        material=dark_steel,
        name="hinge_plate",
    )
    deck.visual(
        Box((0.060, 0.028, 0.006)),
        origin=Origin(xyz=(0.105, 0.0, -0.003)),
        material=dark_steel,
        name="lock_rack",
    )
    deck.visual(
        Box((0.016, 0.050, 0.008)),
        origin=Origin(xyz=(0.105, 0.0, -0.004)),
        material=dark_steel,
        name="lock_stop_low",
    )
    deck.visual(
        Box((0.016, 0.050, 0.008)),
        origin=Origin(xyz=(0.145, 0.0, -0.004)),
        material=dark_steel,
        name="lock_stop_mid",
    )
    deck.visual(
        Box((0.016, 0.050, 0.008)),
        origin=Origin(xyz=(0.185, 0.0, -0.004)),
        material=dark_steel,
        name="lock_stop_open",
    )
    deck.visual(
        Box((0.026, 0.065, 0.010)),
        origin=Origin(xyz=(0.255, 0.0, -0.005)),
        material=dark_steel,
        name="stow_stop",
    )

    primary_frame = model.part("primary_frame")
    _add_rod(
        primary_frame,
        "top_crossbar",
        (0.0, -0.100, -0.016),
        (0.0, 0.100, -0.016),
        0.007,
        steel,
    )
    _add_rod(
        primary_frame,
        "left_hanger",
        (0.0, -0.100, -0.016),
        (0.0, -0.100, 0.012),
        0.006,
        dark_steel,
    )
    _add_rod(
        primary_frame,
        "right_hanger",
        (0.0, 0.100, -0.016),
        (0.0, 0.100, 0.012),
        0.006,
        dark_steel,
    )
    _add_rod(
        primary_frame,
        "left_leg",
        (0.0, -0.100, -0.016),
        (0.220, -0.118, -0.175),
        0.008,
        steel,
    )
    _add_rod(
        primary_frame,
        "right_leg",
        (0.0, 0.100, -0.016),
        (0.220, 0.118, -0.175),
        0.008,
        steel,
    )
    _add_rod(
        primary_frame,
        "left_foot",
        (0.198, -0.118, -0.175),
        (0.246, -0.118, -0.175),
        0.010,
        rubber,
    )
    _add_rod(
        primary_frame,
        "right_foot",
        (0.198, 0.118, -0.175),
        (0.246, 0.118, -0.175),
        0.010,
        rubber,
    )
    primary_frame.visual(
        Box((0.016, 0.024, 0.014)),
        origin=Origin(xyz=(0.110, -0.088, -0.094)),
        material=dark_steel,
        name="left_pivot_lug",
    )
    primary_frame.visual(
        Box((0.016, 0.024, 0.014)),
        origin=Origin(xyz=(0.110, 0.088, -0.094)),
        material=dark_steel,
        name="right_pivot_lug",
    )
    _add_rod(
        primary_frame,
        "left_pivot_gusset",
        (0.110, -0.088, -0.094),
        (0.058, -0.103, -0.056),
        0.005,
        dark_steel,
    )
    _add_rod(
        primary_frame,
        "right_pivot_gusset",
        (0.110, 0.088, -0.094),
        (0.058, 0.103, -0.056),
        0.005,
        dark_steel,
    )

    secondary_frame = model.part("secondary_frame")
    secondary_frame.visual(
        Box((0.014, 0.024, 0.014)),
        origin=Origin(xyz=(0.0, -0.064, 0.0)),
        material=dark_steel,
        name="left_pivot_tab",
    )
    secondary_frame.visual(
        Box((0.014, 0.024, 0.014)),
        origin=Origin(xyz=(0.0, 0.064, 0.0)),
        material=dark_steel,
        name="right_pivot_tab",
    )
    secondary_frame.visual(
        Box((0.012, 0.112, 0.006)),
        origin=Origin(xyz=(0.095, 0.0, 0.028)),
        material=dark_steel,
        name="brace_carrier",
    )
    _add_rod(
        secondary_frame,
        "left_carrier_strut",
        (0.100, -0.046, 0.031),
        (0.141, -0.046, 0.070),
        0.003,
        dark_steel,
    )
    _add_rod(
        secondary_frame,
        "right_carrier_strut",
        (0.100, 0.046, 0.031),
        (0.141, 0.046, 0.070),
        0.003,
        dark_steel,
    )
    _add_rod(
        secondary_frame,
        "top_bar",
        (0.145, -0.058, 0.075),
        (0.145, 0.058, 0.075),
        0.008,
        steel,
    )
    _add_rod(
        secondary_frame,
        "left_upper_leg",
        (0.0, -0.058, 0.0),
        (0.145, -0.058, 0.075),
        0.007,
        steel,
    )
    _add_rod(
        secondary_frame,
        "right_upper_leg",
        (0.0, 0.058, 0.0),
        (0.145, 0.058, 0.075),
        0.007,
        steel,
    )
    _add_rod(
        secondary_frame,
        "left_lower_leg",
        (0.0, -0.058, 0.0),
        (-0.155, -0.058, -0.102),
        0.007,
        steel,
    )
    _add_rod(
        secondary_frame,
        "right_lower_leg",
        (0.0, 0.058, 0.0),
        (-0.155, 0.058, -0.102),
        0.007,
        steel,
    )
    _add_rod(
        secondary_frame,
        "bottom_bar",
        (-0.155, -0.058, -0.102),
        (-0.155, 0.058, -0.102),
        0.010,
        rubber,
    )

    lock_brace = model.part("lock_brace")
    _add_rod(
        lock_brace,
        "brace_pin",
        (0.0, -0.022, 0.0),
        (0.0, 0.022, 0.0),
        0.009,
        dark_steel,
    )
    _add_rod(
        lock_brace,
        "brace_body",
        (0.0, 0.0, 0.0),
        (0.122, 0.0, 0.041),
        0.006,
        dark_steel,
    )
    _add_rod(
        lock_brace,
        "hook_neck",
        (0.112, 0.0, 0.038),
        (0.126, 0.0, 0.060),
        0.005,
        dark_steel,
    )
    lock_brace.visual(
        Box((0.018, 0.032, 0.008)),
        origin=Origin(xyz=(0.126, 0.0, 0.062)),
        material=dark_steel,
        name="brace_hook",
    )

    model.articulation(
        "deck_to_primary_frame",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=primary_frame,
        origin=Origin(xyz=(-0.150, 0.0, -0.022)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.8,
            lower=0.0,
            upper=1.05,
        ),
    )
    model.articulation(
        "primary_to_secondary_frame",
        ArticulationType.REVOLUTE,
        parent=primary_frame,
        child=secondary_frame,
        origin=Origin(xyz=(0.110, 0.0, -0.094)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.8,
            lower=-1.25,
            upper=0.25,
        ),
    )
    model.articulation(
        "secondary_to_lock_brace",
        ArticulationType.REVOLUTE,
        parent=secondary_frame,
        child=lock_brace,
        origin=Origin(xyz=(0.095, 0.0, 0.040)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=-1.30,
            upper=0.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    primary_frame = object_model.get_part("primary_frame")
    secondary_frame = object_model.get_part("secondary_frame")
    lock_brace = object_model.get_part("lock_brace")

    hinge = object_model.get_articulation("deck_to_primary_frame")
    scissor = object_model.get_articulation("primary_to_secondary_frame")
    brace_joint = object_model.get_articulation("secondary_to_lock_brace")

    hinge_plate = deck.get_visual("hinge_plate")
    underside_rail = deck.get_visual("underside_rail")
    lock_stop_open = deck.get_visual("lock_stop_open")
    deck_panel = deck.get_visual("deck_panel")

    left_hanger = primary_frame.get_visual("left_hanger")
    left_pivot_lug = primary_frame.get_visual("left_pivot_lug")
    top_bar = secondary_frame.get_visual("top_bar")
    left_pivot_tab = secondary_frame.get_visual("left_pivot_tab")
    bottom_bar = secondary_frame.get_visual("bottom_bar")
    brace_carrier = secondary_frame.get_visual("brace_carrier")
    brace_pin = lock_brace.get_visual("brace_pin")
    brace_hook = lock_brace.get_visual("brace_hook")

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

    ctx.expect_contact(
        deck,
        primary_frame,
        elem_a=hinge_plate,
        elem_b=left_hanger,
        contact_tol=0.0035,
        name="primary_frame_hangs_from_hinge_plate",
    )
    ctx.expect_contact(
        primary_frame,
        secondary_frame,
        elem_a=left_pivot_lug,
        elem_b=left_pivot_tab,
        contact_tol=0.0035,
        name="scissor_frames_meet_at_center_pivot",
    )
    ctx.expect_contact(
        secondary_frame,
        lock_brace,
        elem_a=brace_carrier,
        elem_b=brace_pin,
        contact_tol=0.0035,
        name="lock_brace_is_hinged_on_secondary_frame",
    )
    ctx.expect_gap(
        deck,
        secondary_frame,
        axis="z",
        positive_elem=underside_rail,
        negative_elem=top_bar,
        min_gap=0.002,
        max_gap=0.020,
        name="open_stance_keeps_top_bar_clear_of_rail",
    )
    ctx.expect_contact(
        deck,
        lock_brace,
        elem_a=lock_stop_open,
        elem_b=brace_hook,
        contact_tol=0.004,
        name="lock_brace_engages_open_stop",
    )
    ctx.expect_gap(
        deck,
        secondary_frame,
        axis="z",
        positive_elem=deck_panel,
        negative_elem=bottom_bar,
        min_gap=0.180,
        max_gap=0.235,
        name="desktop_open_height_reads_supported",
    )

    with ctx.pose({hinge: 0.84, scissor: -0.98, brace_joint: -0.90}):
        ctx.expect_within(
            primary_frame,
            deck,
            axes="xy",
            margin=0.070,
            name="primary_frame_stows_within_board_footprint",
        )
        ctx.expect_within(
            secondary_frame,
            deck,
            axes="xy",
            margin=0.070,
            name="secondary_frame_stows_within_board_footprint",
        )
        ctx.expect_gap(
            deck,
            secondary_frame,
            axis="z",
            positive_elem=underside_rail,
            negative_elem=bottom_bar,
            min_gap=0.010,
            max_gap=0.085,
            name="secondary_frame_folds_close_under_board",
        )
        ctx.expect_gap(
            deck,
            lock_brace,
            axis="z",
            positive_elem=deck_panel,
            negative_elem=brace_hook,
            min_gap=0.002,
            max_gap=0.060,
            name="brace_tucks_clear_when_folded",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
