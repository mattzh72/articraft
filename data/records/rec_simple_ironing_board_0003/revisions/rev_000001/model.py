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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
    wire_from_points,
)

ASSETS = AssetContext.from_script(__file__)


BOARD_LENGTH = 1.34
BOARD_HALF_WIDTH = 0.195
SHELL_THICKNESS = 0.018
PAD_THICKNESS = 0.008
HINGE_Z = -0.035


def _save_mesh(name: str, geometry):
    ASSETS.mesh_dir.mkdir(parents=True, exist_ok=True)
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _smoothstep(value: float) -> float:
    value = max(0.0, min(1.0, value))
    return value * value * (3.0 - 2.0 * value)


def _board_outline(*, length: float, half_width: float, inset: float = 0.0) -> list[tuple[float, float]]:
    start_x = (-length * 0.5) + inset
    tip_x = (length * 0.5) - inset
    rear_span = 0.18
    nose_span = 0.46
    effective_half_width = max(0.025, half_width - inset)

    right_side: list[tuple[float, float]] = []
    sample_count = 56
    for index in range(sample_count + 1):
        t = index / sample_count
        x = start_x + (tip_x - start_x) * t
        if x <= start_x + rear_span:
            rear_t = (x - start_x) / rear_span
            width_factor = math.sin((_smoothstep(rear_t) * math.pi) * 0.5)
        elif x >= tip_x - nose_span:
            nose_t = (x - (tip_x - nose_span)) / nose_span
            width_factor = math.cos((_smoothstep(nose_t) * math.pi) * 0.5) ** 0.92
        else:
            center_t = (x - (start_x + rear_span)) / max(1e-6, (tip_x - nose_span) - (start_x + rear_span))
            width_factor = 0.985 - 0.045 * math.sin(center_t * math.pi)
        right_side.append((x, max(0.0, effective_half_width * width_factor)))

    profile = right_side + [(x, -y) for x, y in reversed(right_side[1:-1])]
    return profile


def _leg_frame_mesh(
    name: str,
    *,
    width: float,
    foot_x: float,
    foot_z: float,
    shoulder_x: float,
    shoulder_z: float,
    tube_radius: float,
):
    half_width = width * 0.5
    frame_path = [
        (foot_x, -half_width, foot_z),
        (shoulder_x, -half_width, shoulder_z),
        (0.0, -half_width, 0.0),
        (0.0, half_width, 0.0),
        (shoulder_x, half_width, shoulder_z),
        (foot_x, half_width, foot_z),
    ]
    return _save_mesh(
        name,
        wire_from_points(
            frame_path,
            radius=tube_radius,
            radial_segments=18,
            cap_ends=True,
            corner_mode="fillet",
            corner_radius=0.05,
            corner_segments=12,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_ironing_board", assets=ASSETS)

    cover_fabric = model.material("cover_fabric", rgba=(0.52, 0.57, 0.62, 1.0))
    shell_satin = model.material("shell_satin", rgba=(0.82, 0.84, 0.86, 1.0))
    frame_satin = model.material("frame_satin", rgba=(0.72, 0.74, 0.77, 1.0))
    hardware_dark = model.material("hardware_dark", rgba=(0.28, 0.30, 0.33, 1.0))
    foot_tpe = model.material("foot_tpe", rgba=(0.14, 0.15, 0.16, 1.0))

    deck_shell_mesh = _save_mesh(
        "ironing_board_deck_shell.obj",
        ExtrudeGeometry(_board_outline(length=BOARD_LENGTH, half_width=BOARD_HALF_WIDTH), SHELL_THICKNESS),
    )
    cover_pad_mesh = _save_mesh(
        "ironing_board_cover_pad.obj",
        ExtrudeGeometry(_board_outline(length=BOARD_LENGTH - 0.016, half_width=BOARD_HALF_WIDTH - 0.004), PAD_THICKNESS),
    )
    seam_band_mesh = _save_mesh(
        "ironing_board_cover_seam_band.obj",
        ExtrudeWithHolesGeometry(
            _board_outline(length=BOARD_LENGTH - 0.016, half_width=BOARD_HALF_WIDTH - 0.004),
            [_board_outline(length=BOARD_LENGTH - 0.048, half_width=BOARD_HALF_WIDTH - 0.016)],
            height=0.0014,
            center=True,
        ),
    )
    front_frame_mesh = _leg_frame_mesh(
        "ironing_board_front_frame.obj",
        width=0.292,
        foot_x=-0.392,
        foot_z=-0.790,
        shoulder_x=-0.050,
        shoulder_z=-0.115,
        tube_radius=0.008,
    )
    rear_frame_mesh = _leg_frame_mesh(
        "ironing_board_rear_frame.obj",
        width=0.238,
        foot_x=0.392,
        foot_z=-0.790,
        shoulder_x=0.050,
        shoulder_z=-0.115,
        tube_radius=0.008,
    )
    brace_bar_mesh = _save_mesh(
        "ironing_board_lock_brace.obj",
        tube_from_spline_points(
            [
                (0.0, 0.0, 0.0),
                (0.016, 0.0, 0.046),
                (0.040, 0.0, 0.098),
                (0.058, 0.0, 0.150),
            ],
            radius=0.0046,
            samples_per_segment=14,
            radial_segments=14,
            cap_ends=True,
        ),
    )

    deck = model.part("deck")
    deck.visual(deck_shell_mesh, material=shell_satin, name="deck_shell")
    deck.visual(
        cover_pad_mesh,
        origin=Origin(xyz=(0.0, 0.0, (SHELL_THICKNESS + PAD_THICKNESS) * 0.5)),
        material=cover_fabric,
        name="cover_pad",
    )
    deck.visual(
        seam_band_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0163)),
        material=frame_satin,
        name="cover_seam_band",
    )
    deck.visual(
        Box((0.54, 0.022, 0.016)),
        origin=Origin(xyz=(0.02, -0.110, -0.017)),
        material=frame_satin,
        name="left_underside_rail",
    )
    deck.visual(
        Box((0.54, 0.022, 0.016)),
        origin=Origin(xyz=(0.02, 0.110, -0.017)),
        material=frame_satin,
        name="right_underside_rail",
    )
    deck.visual(
        Box((0.060, 0.185, 0.015)),
        origin=Origin(xyz=(0.140, 0.0, -0.0165)),
        material=hardware_dark,
        name="front_mount_block",
    )
    deck.visual(
        Box((0.060, 0.165, 0.015)),
        origin=Origin(xyz=(-0.140, 0.0, -0.0165)),
        material=hardware_dark,
        name="rear_mount_block",
    )
    deck.visual(
        Box((0.030, 0.020, 0.012)),
        origin=Origin(xyz=(0.004, 0.110, -0.031)),
        material=hardware_dark,
        name="lock_block",
    )
    deck.visual(
        Box((0.050, 0.010, 0.006)),
        origin=Origin(xyz=(-0.555, 0.0, 0.010)),
        material=hardware_dark,
        name="tail_trim_cap",
    )
    deck.inertial = Inertial.from_geometry(
        Box((BOARD_LENGTH, 0.40, 0.055)),
        mass=6.4,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    front_frame = model.part("front_frame")
    front_frame.visual(front_frame_mesh, material=frame_satin, name="frame_tube")
    front_frame.visual(
        Cylinder(radius=0.011, length=0.188),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_dark,
        name="top_hinge_tube",
    )
    front_frame.visual(
        Box((0.070, 0.030, 0.014)),
        origin=Origin(xyz=(-0.392, -0.146, -0.800)),
        material=foot_tpe,
        name="front_left_foot",
    )
    front_frame.visual(
        Box((0.070, 0.030, 0.014)),
        origin=Origin(xyz=(-0.392, 0.146, -0.800)),
        material=foot_tpe,
        name="front_right_foot",
    )
    front_frame.inertial = Inertial.from_geometry(
        Box((0.46, 0.31, 0.82)),
        mass=1.5,
        origin=Origin(xyz=(-0.19, 0.0, -0.40)),
    )

    rear_frame = model.part("rear_frame")
    rear_frame.visual(rear_frame_mesh, material=frame_satin, name="frame_tube")
    rear_frame.visual(
        Cylinder(radius=0.011, length=0.168),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_dark,
        name="top_hinge_tube",
    )
    rear_frame.visual(
        Box((0.070, 0.028, 0.014)),
        origin=Origin(xyz=(0.392, -0.119, -0.800)),
        material=foot_tpe,
        name="rear_left_foot",
    )
    rear_frame.visual(
        Box((0.070, 0.028, 0.014)),
        origin=Origin(xyz=(0.392, 0.119, -0.800)),
        material=foot_tpe,
        name="rear_right_foot",
    )
    rear_frame.inertial = Inertial.from_geometry(
        Box((0.46, 0.25, 0.82)),
        mass=1.45,
        origin=Origin(xyz=(0.19, 0.0, -0.40)),
    )

    lock_brace = model.part("lock_brace")
    lock_brace.visual(
        Cylinder(radius=0.0075, length=0.012),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_dark,
        name="brace_knuckle",
    )
    lock_brace.visual(brace_bar_mesh, material=hardware_dark, name="brace_bar")
    lock_brace.visual(
        Box((0.022, 0.012, 0.008)),
        origin=Origin(xyz=(0.062, 0.0, 0.154)),
        material=hardware_dark,
        name="brace_tip",
    )
    lock_brace.inertial = Inertial.from_geometry(
        Box((0.090, 0.020, 0.165)),
        mass=0.18,
        origin=Origin(xyz=(0.030, 0.0, 0.075)),
    )

    model.articulation(
        "front_leg_fold",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=front_frame,
        origin=Origin(xyz=(0.140, 0.0, HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.4,
            lower=0.0,
            upper=0.52,
        ),
    )
    model.articulation(
        "rear_leg_fold",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=rear_frame,
        origin=Origin(xyz=(-0.140, 0.0, HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.4,
            lower=-0.52,
            upper=0.0,
        ),
    )
    model.articulation(
        "lock_brace_fold",
        ArticulationType.REVOLUTE,
        parent=rear_frame,
        child=lock_brace,
        origin=Origin(xyz=(0.082, 0.110, -0.160)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-1.05,
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    deck = object_model.get_part("deck")
    front_frame = object_model.get_part("front_frame")
    rear_frame = object_model.get_part("rear_frame")
    lock_brace = object_model.get_part("lock_brace")

    front_leg_fold = object_model.get_articulation("front_leg_fold")
    rear_leg_fold = object_model.get_articulation("rear_leg_fold")
    lock_brace_fold = object_model.get_articulation("lock_brace_fold")

    deck_shell = deck.get_visual("deck_shell")
    cover_pad = deck.get_visual("cover_pad")
    front_mount_block = deck.get_visual("front_mount_block")
    rear_mount_block = deck.get_visual("rear_mount_block")
    lock_block = deck.get_visual("lock_block")
    cover_seam_band = deck.get_visual("cover_seam_band")

    front_hinge_tube = front_frame.get_visual("top_hinge_tube")
    rear_hinge_tube = rear_frame.get_visual("top_hinge_tube")
    front_left_foot = front_frame.get_visual("front_left_foot")
    rear_right_foot = rear_frame.get_visual("rear_right_foot")
    brace_tip = lock_brace.get_visual("brace_tip")

    def _aabb_center(aabb):
        return tuple((lower + upper) * 0.5 for lower, upper in zip(aabb[0], aabb[1]))

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
    ctx.allow_overlap(
        rear_frame,
        lock_brace,
        reason="Brace knuckle is a seated hinge barrel nested into the rear-frame pivot lug.",
        elem_a=rear_frame.get_visual("frame_tube"),
        elem_b=lock_brace.get_visual("brace_knuckle"),
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_within(
        deck,
        deck,
        axes="xy",
        inner_elem=cover_pad,
        outer_elem=deck_shell,
        margin=0.003,
        name="cover_pad_stays_inside_board_shell",
    )
    ctx.expect_gap(
        deck,
        deck,
        axis="z",
        positive_elem=cover_pad,
        negative_elem=deck_shell,
        min_gap=0.0,
        max_gap=0.0001,
        name="cover_pad_sits_flush_on_shell",
    )
    ctx.expect_within(
        deck,
        deck,
        axes="xy",
        inner_elem=cover_seam_band,
        outer_elem=cover_pad,
        margin=0.0015,
        name="seam_band_stays_inside_cover_outline",
    )
    ctx.expect_gap(
        deck,
        front_frame,
        axis="z",
        positive_elem=front_mount_block,
        negative_elem=front_hinge_tube,
        min_gap=0.0,
        max_gap=0.0001,
        name="front_hinge_seats_against_mount_block",
    )
    ctx.expect_overlap(
        front_frame,
        deck,
        axes="y",
        elem_a=front_hinge_tube,
        elem_b=front_mount_block,
        min_overlap=0.16,
        name="front_hinge_tracks_under_mount_block",
    )
    ctx.expect_gap(
        deck,
        rear_frame,
        axis="z",
        positive_elem=rear_mount_block,
        negative_elem=rear_hinge_tube,
        min_gap=0.0,
        max_gap=0.0001,
        name="rear_hinge_seats_against_mount_block",
    )
    ctx.expect_overlap(
        rear_frame,
        deck,
        axes="y",
        elem_a=rear_hinge_tube,
        elem_b=rear_mount_block,
        min_overlap=0.14,
        name="rear_hinge_tracks_under_mount_block",
    )
    ctx.expect_within(front_frame, deck, axes="y", margin=0.02, name="front_frame_stays_within_board_width")
    ctx.expect_within(rear_frame, deck, axes="y", margin=0.02, name="rear_frame_stays_within_board_width")
    ctx.expect_overlap(
        front_frame,
        rear_frame,
        axes="xz",
        min_overlap=0.18,
        name="scissor_frames_cross_in_side_view",
    )
    ctx.expect_gap(
        deck,
        lock_brace,
        axis="z",
        positive_elem=lock_block,
        negative_elem=brace_tip,
        min_gap=0.0,
        max_gap=0.001,
        name="open_position_lock_seat_is_tight",
    )
    ctx.expect_overlap(
        lock_brace,
        deck,
        axes="xy",
        elem_a=brace_tip,
        elem_b=lock_block,
        min_overlap=0.010,
        name="brace_tip_aligns_with_lock_block",
    )

    front_open_aabb = ctx.part_element_world_aabb(front_frame, elem=front_left_foot)
    rear_open_aabb = ctx.part_element_world_aabb(rear_frame, elem=rear_right_foot)
    assert front_open_aabb is not None
    assert rear_open_aabb is not None
    front_open_center = _aabb_center(front_open_aabb)
    rear_open_center = _aabb_center(rear_open_aabb)

    with ctx.pose({front_leg_fold: 0.42, rear_leg_fold: -0.42, lock_brace_fold: -0.95}):
        front_folded_aabb = ctx.part_element_world_aabb(front_frame, elem=front_left_foot)
        rear_folded_aabb = ctx.part_element_world_aabb(rear_frame, elem=rear_right_foot)
        assert front_folded_aabb is not None
        assert rear_folded_aabb is not None
        front_folded_center = _aabb_center(front_folded_aabb)
        rear_folded_center = _aabb_center(rear_folded_aabb)
        ctx.check(
            "front_leg_lifts_when_folding",
            front_folded_center[2] > front_open_center[2] + 0.12,
            details=f"front foot z open={front_open_center[2]:.4f}, folded={front_folded_center[2]:.4f}",
        )
        ctx.check(
            "rear_leg_lifts_when_folding",
            rear_folded_center[2] > rear_open_center[2] + 0.12,
            details=f"rear foot z open={rear_open_center[2]:.4f}, folded={rear_folded_center[2]:.4f}",
        )
        ctx.expect_gap(
            deck,
            lock_brace,
            axis="x",
            positive_elem=lock_block,
            negative_elem=brace_tip,
            min_gap=0.06,
            name="brace_clears_lock_block_in_folded_pose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
