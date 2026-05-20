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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    sample_catmull_rom_spline_2d,
)


def _scaled_profile(
    profile: list[tuple[float, float]],
    sx: float,
    sy: float,
) -> list[tuple[float, float]]:
    return [(x * sx, y * sy) for x, y in profile]


def _add_tube_xz(
    part,
    *,
    name: str,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    material,
) -> None:
    dx = end[0] - start[0]
    dz = end[2] - start[2]
    length = math.hypot(dx, dz)
    mx = (start[0] + end[0]) * 0.5
    my = start[1]
    mz = (start[2] + end[2]) * 0.5
    angle_y = math.atan2(dx, dz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(mx, my, mz), rpy=(0.0, angle_y, 0.0)),
        material=material,
        name=name,
    )


def _add_y_crossbar(
    part,
    *,
    name: str,
    center: tuple[float, float, float],
    length: float,
    radius: float,
    material,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _add_box_beam_xz(
    part,
    *,
    name: str,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    size_y: float,
    size_z: float,
    material,
) -> None:
    dx = end[0] - start[0]
    dz = end[2] - start[2]
    length = math.hypot(dx, dz)
    mx = (start[0] + end[0]) * 0.5
    my = (start[1] + end[1]) * 0.5
    mz = (start[2] + end[2]) * 0.5
    angle_y = math.atan2(-dz, dx)
    part.visual(
        Box((length, size_y, size_z)),
        origin=Origin(xyz=(mx, my, mz), rpy=(0.0, angle_y, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_ironing_board")

    deck_metal = model.material("deck_metal", rgba=(0.76, 0.78, 0.80, 1.0))
    cover_fabric = model.material("cover_fabric", rgba=(0.90, 0.91, 0.92, 1.0))
    frame_metal = model.material("frame_metal", rgba=(0.19, 0.21, 0.24, 1.0))
    polymer = model.material("polymer", rgba=(0.70, 0.72, 0.75, 1.0))
    elastomer = model.material("elastomer", rgba=(0.10, 0.11, 0.12, 1.0))

    deck_outline = sample_catmull_rom_spline_2d(
        [
            (-0.68, 0.17),
            (-0.60, 0.19),
            (-0.34, 0.19),
            (0.08, 0.18),
            (0.36, 0.14),
            (0.56, 0.09),
            (0.67, 0.03),
            (0.69, 0.00),
            (0.67, -0.03),
            (0.56, -0.09),
            (0.36, -0.14),
            (0.08, -0.18),
            (-0.34, -0.19),
            (-0.60, -0.19),
            (-0.68, -0.17),
        ],
        samples_per_segment=10,
        closed=True,
    )
    cover_outline = _scaled_profile(deck_outline, 0.992, 0.976)

    deck_shell_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(deck_outline, 0.008),
        "ironing_board_deck_shell",
    )
    cover_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(cover_outline, 0.008),
        "ironing_board_cover_pad",
    )

    deck = model.part("deck")
    deck.visual(deck_shell_mesh, material=deck_metal, name="deck_shell")
    deck.visual(
        cover_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0076)),
        material=cover_fabric,
        name="cover_pad",
    )
    deck.visual(
        Box((0.34, 0.28, 0.014)),
        origin=Origin(xyz=(-0.31, 0.0, -0.0040)),
        material=frame_metal,
        name="tail_stiffener",
    )
    deck.visual(
        Box((0.18, 0.34, 0.015)),
        origin=Origin(xyz=(0.18, 0.0, -0.0040)),
        material=frame_metal,
        name="hinge_plate",
    )
    deck.visual(
        Box((0.60, 0.080, 0.018)),
        origin=Origin(xyz=(-0.09, 0.0, -0.0050)),
        material=frame_metal,
        name="underframe_spine",
    )
    deck.visual(
        Box((0.045, 0.016, 0.071)),
        origin=Origin(xyz=(0.18, 0.163, -0.0340)),
        material=frame_metal,
        name="left_hinge_bracket",
    )
    deck.visual(
        Box((0.045, 0.016, 0.071)),
        origin=Origin(xyz=(0.18, -0.163, -0.0340)),
        material=frame_metal,
        name="right_hinge_bracket",
    )
    deck.visual(
        Box((0.46, 0.090, 0.008)),
        origin=Origin(xyz=(-0.12, 0.0, -0.019)),
        material=frame_metal,
        name="slider_track",
    )
    deck.visual(
        Box((0.040, 0.040, 0.020)),
        origin=Origin(xyz=(0.04, 0.0, -0.0065)),
        material=frame_metal,
        name="front_track_rib",
    )
    deck.visual(
        Box((0.040, 0.040, 0.020)),
        origin=Origin(xyz=(-0.32, 0.0, -0.0065)),
        material=frame_metal,
        name="rear_track_rib",
    )
    deck.visual(
        Box((0.24, 0.030, 0.008)),
        origin=Origin(xyz=(-0.08, 0.0, -0.041)),
        material=frame_metal,
        name="stance_rack",
    )
    for i, x_pos in enumerate((-0.16, -0.12, -0.08, -0.04, 0.00), start=1):
        deck.visual(
            Box((0.018, 0.026, 0.006)),
            origin=Origin(xyz=(x_pos, 0.0, -0.048)),
            material=frame_metal,
            name=f"rack_tooth_{i}",
        )
    deck.visual(
        Box((0.020, 0.024, 0.014)),
        origin=Origin(xyz=(-0.16, 0.0, -0.030)),
        material=frame_metal,
        name="left_rack_tab",
    )
    deck.visual(
        Box((0.020, 0.024, 0.014)),
        origin=Origin(xyz=(0.00, 0.0, -0.030)),
        material=frame_metal,
        name="right_rack_tab",
    )
    deck.inertial = Inertial.from_geometry(
        Box((1.38, 0.40, 0.10)),
        mass=5.4,
        origin=Origin(xyz=(0.0, 0.0, -0.01)),
    )

    front_frame = model.part("front_frame")
    front_y = 0.145
    front_top = (0.0, front_y, 0.0)
    front_bottom = (-0.39, front_y, -0.847)
    _add_tube_xz(
        front_frame,
        name="front_left_leg",
        start=front_top,
        end=front_bottom,
        radius=0.009,
        material=frame_metal,
    )
    _add_tube_xz(
        front_frame,
        name="front_right_leg",
        start=(front_top[0], -front_y, front_top[2]),
        end=(front_bottom[0], -front_y, front_bottom[2]),
        radius=0.009,
        material=frame_metal,
    )
    front_frame.visual(
        Box((0.034, 0.024, 0.009)),
        origin=Origin(xyz=(0.0, front_y, 0.0)),
        material=frame_metal,
        name="front_left_hinge_lug",
    )
    front_frame.visual(
        Box((0.034, 0.024, 0.009)),
        origin=Origin(xyz=(0.0, -front_y, 0.0)),
        material=frame_metal,
        name="front_right_hinge_lug",
    )
    _add_y_crossbar(
        front_frame,
        name="front_bottom_crossbar",
        center=(-0.39, 0.0, -0.847),
        length=front_y * 2.0,
        radius=0.009,
        material=frame_metal,
    )
    _add_y_crossbar(
        front_frame,
        name="front_upper_tie",
        center=(-0.05, 0.0, -0.105),
        length=front_y * 2.0,
        radius=0.007,
        material=frame_metal,
    )
    front_frame.visual(
        Box((0.026, 0.016, 0.026)),
        origin=Origin(xyz=(-0.195, 0.145, -0.4235)),
        material=frame_metal,
        name="left_pivot_boss",
    )
    front_frame.visual(
        Box((0.026, 0.016, 0.026)),
        origin=Origin(xyz=(-0.195, -0.145, -0.4235)),
        material=frame_metal,
        name="right_pivot_boss",
    )
    front_frame.visual(
        Box((0.016, 0.024, 0.020)),
        origin=Origin(xyz=(-0.39, front_y, -0.856)),
        material=frame_metal,
        name="front_left_foot_shank",
    )
    front_frame.visual(
        Box((0.016, 0.024, 0.020)),
        origin=Origin(xyz=(-0.39, -front_y, -0.856)),
        material=frame_metal,
        name="front_right_foot_shank",
    )
    front_frame.visual(
        Box((0.054, 0.034, 0.020)),
        origin=Origin(xyz=(-0.39, front_y, -0.865)),
        material=elastomer,
        name="front_left_foot",
    )
    front_frame.visual(
        Box((0.054, 0.034, 0.020)),
        origin=Origin(xyz=(-0.39, -front_y, -0.865)),
        material=elastomer,
        name="front_right_foot",
    )
    front_frame.inertial = Inertial.from_geometry(
        Box((0.46, 0.34, 0.90)),
        mass=1.7,
        origin=Origin(xyz=(-0.20, 0.0, -0.42)),
    )

    rear_frame = model.part("rear_frame")
    rear_y = 0.115
    rear_top_left = (-0.225, rear_y, 0.4235)
    rear_bottom_left = (0.225, rear_y, -0.4235)
    _add_tube_xz(
        rear_frame,
        name="rear_left_leg",
        start=rear_top_left,
        end=rear_bottom_left,
        radius=0.009,
        material=frame_metal,
    )
    _add_tube_xz(
        rear_frame,
        name="rear_right_leg",
        start=(rear_top_left[0], -rear_y, rear_top_left[2]),
        end=(rear_bottom_left[0], -rear_y, rear_bottom_left[2]),
        radius=0.009,
        material=frame_metal,
    )
    _add_y_crossbar(
        rear_frame,
        name="rear_bottom_crossbar",
        center=(0.225, 0.0, -0.4235),
        length=rear_y * 2.0,
        radius=0.009,
        material=frame_metal,
    )
    rear_frame.visual(
        Box((0.080, 0.250, 0.018)),
        origin=Origin(xyz=(-0.225, 0.0, 0.4025)),
        material=frame_metal,
        name="top_bridge",
    )
    rear_frame.visual(
        Box((0.052, 0.026, 0.010)),
        origin=Origin(xyz=(-0.225, rear_y, 0.4045)),
        material=polymer,
        name="left_glide",
    )
    rear_frame.visual(
        Box((0.052, 0.026, 0.010)),
        origin=Origin(xyz=(-0.225, -rear_y, 0.4045)),
        material=polymer,
        name="right_glide",
    )
    rear_frame.visual(
        Box((0.054, 0.034, 0.020)),
        origin=Origin(xyz=(0.225, rear_y, -0.4415)),
        material=elastomer,
        name="rear_left_foot",
    )
    rear_frame.visual(
        Box((0.054, 0.034, 0.020)),
        origin=Origin(xyz=(0.225, -rear_y, -0.4415)),
        material=elastomer,
        name="rear_right_foot",
    )
    rear_frame.inertial = Inertial.from_geometry(
        Box((0.48, 0.30, 0.90)),
        mass=1.5,
        origin=Origin(xyz=(0.02, 0.0, -0.03)),
    )

    lock_bar = model.part("lock_bar")
    lock_bar_start = (0.0, 0.0, 0.0)
    lock_bar_end = (-0.145, 0.0, 0.251)
    lock_bar.visual(
        Box((0.028, 0.024, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=frame_metal,
        name="pivot_head",
    )
    _add_box_beam_xz(
        lock_bar,
        name="lock_link",
        start=lock_bar_start,
        end=lock_bar_end,
        size_y=0.022,
        size_z=0.008,
        material=frame_metal,
    )
    lock_bar.visual(
        Box((0.024, 0.024, 0.010)),
        origin=Origin(xyz=lock_bar_end),
        material=frame_metal,
        name="hook_tip",
    )
    lock_bar.visual(
        Box((0.012, 0.022, 0.040)),
        origin=Origin(xyz=(0.012, 0.0, -0.015)),
        material=frame_metal,
        name="handle_stem",
    )
    lock_bar.visual(
        Cylinder(radius=0.010, length=0.090),
        origin=Origin(xyz=(0.022, 0.0, -0.035), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=elastomer,
        name="release_grip",
    )
    lock_bar.inertial = Inertial.from_geometry(
        Box((0.30, 0.10, 0.06)),
        mass=0.24,
        origin=Origin(xyz=(-0.06, 0.0, 0.11)),
    )

    model.articulation(
        "deck_to_front_frame",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=front_frame,
        origin=Origin(xyz=(0.18, 0.0, -0.016)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=0.0,
            upper=1.18,
        ),
    )
    model.articulation(
        "front_to_rear_frame",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=rear_frame,
        origin=Origin(xyz=(-0.195, 0.0, -0.4235)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=-1.05,
            upper=0.35,
        ),
    )
    model.articulation(
        "front_to_lock_bar",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=lock_bar,
        origin=Origin(xyz=(-0.135, 0.0, -0.280)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=-0.65,
            upper=0.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    front_frame = object_model.get_part("front_frame")
    rear_frame = object_model.get_part("rear_frame")
    lock_bar = object_model.get_part("lock_bar")

    deck_to_front = object_model.get_articulation("deck_to_front_frame")
    front_to_rear = object_model.get_articulation("front_to_rear_frame")
    front_to_lock = object_model.get_articulation("front_to_lock_bar")

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

    def _bottom_z(part, elem: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        return None if aabb is None else aabb[0][2]

    ctx.check(
        "widthwise_joint_axes",
        all(
            math.isclose(j.axis[0], 0.0, abs_tol=1e-9)
            and math.isclose(j.axis[1], 1.0, abs_tol=1e-9)
            and math.isclose(j.axis[2], 0.0, abs_tol=1e-9)
            for j in (deck_to_front, front_to_rear, front_to_lock)
        ),
        "Scissor and lock articulations should all rotate about the board width axis.",
    )

    ctx.expect_contact(
        lock_bar,
        deck,
        elem_a="hook_tip",
        elem_b="stance_rack",
        contact_tol=0.0015,
        name="lock_hook_contacts_stance_rack",
    )
    ctx.expect_overlap(
        lock_bar,
        deck,
        axes="xy",
        elem_a="hook_tip",
        elem_b="stance_rack",
        min_overlap=0.018,
        name="lock_hook_sits_under_rack",
    )
    ctx.expect_gap(
        deck,
        rear_frame,
        axis="z",
        positive_elem="slider_track",
        negative_elem="top_bridge",
        min_gap=0.001,
        max_gap=0.006,
        name="rear_frame_runs_just_under_slider_track",
    )
    ctx.expect_overlap(
        rear_frame,
        deck,
        axes="xy",
        elem_a="top_bridge",
        elem_b="slider_track",
        min_overlap=0.075,
        name="rear_frame_top_bridge_is_captured_by_slider_track_footprint",
    )

    feet = [
        _bottom_z(front_frame, "front_left_foot"),
        _bottom_z(front_frame, "front_right_foot"),
        _bottom_z(rear_frame, "rear_left_foot"),
        _bottom_z(rear_frame, "rear_right_foot"),
    ]
    feet_ok = all(v is not None for v in feet)
    spread = 0.0 if not feet_ok else max(feet) - min(feet)  # type: ignore[arg-type]
    ctx.check(
        "feet_share_a_ground_plane",
        feet_ok and spread <= 0.004,
        f"Foot bottom spread should stay within 4 mm for a stable stance, got {spread:.4f} m.",
    )

    open_front_aabb = ctx.part_world_aabb(front_frame)
    open_rear_aabb = ctx.part_world_aabb(rear_frame)

    with ctx.pose(
        {
            deck_to_front: 1.18,
            front_to_rear: 0.0,
            front_to_lock: 0.15,
        }
    ):
        folded_front_aabb = ctx.part_world_aabb(front_frame)
        folded_rear_aabb = ctx.part_world_aabb(rear_frame)
        fold_ok = (
            open_front_aabb is not None
            and open_rear_aabb is not None
            and folded_front_aabb is not None
            and folded_rear_aabb is not None
            and folded_front_aabb[0][2] > open_front_aabb[0][2] + 0.45
            and folded_rear_aabb[0][2] > open_rear_aabb[0][2] + 0.38
        )
        ctx.check(
            "scissor_frames_fold_upward",
            fold_ok,
            "The leg assemblies should retract substantially toward the deck in the folded pose.",
        )
        ctx.expect_gap(
            lock_bar,
            deck,
            axis="z",
            positive_elem="hook_tip",
            negative_elem="stance_rack",
            min_gap=0.05,
            name="released_lock_hook_clears_rack",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
