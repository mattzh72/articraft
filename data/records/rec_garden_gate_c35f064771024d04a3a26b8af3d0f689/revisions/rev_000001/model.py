from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _arch_height(x: float, *, x0: float, x1: float, side_top: float, peak_top: float) -> float:
    center = 0.5 * (x0 + x1)
    half_span = 0.5 * (x1 - x0)
    t = (x - center) / half_span
    return side_top + (peak_top - side_top) * (1.0 - t * t)


def _arched_top_rail_mesh(
    *,
    name: str,
    x0: float,
    x1: float,
    side_top: float,
    peak_top: float,
    band_height: float,
    thickness: float,
):
    samples = 28
    profile: list[tuple[float, float]] = []
    for index in range(samples + 1):
        x = x0 + (x1 - x0) * (index / samples)
        profile.append((x, _arch_height(x, x0=x0, x1=x1, side_top=side_top, peak_top=peak_top)))
    for index in range(samples, -1, -1):
        x = x0 + (x1 - x0) * (index / samples)
        profile.append(
            (
                x,
                _arch_height(x, x0=x0, x1=x1, side_top=side_top, peak_top=peak_top) - band_height,
            )
        )
    return mesh_from_geometry(
        ExtrudeGeometry.centered(profile, thickness, cap=True, closed=True).rotate_x(pi / 2.0),
        name,
    )


def _hinge_barrel_mesh(*, name: str, inner_radius: float, outer_radius: float, height: float):
    half_height = 0.5 * height
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(outer_radius, -half_height), (outer_radius, half_height)],
            [(inner_radius, -half_height), (inner_radius, half_height)],
            segments=36,
            start_cap="flat",
            end_cap="flat",
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="arched_wicket_gate")

    stone = model.material("stone", rgba=(0.67, 0.67, 0.66, 1.0))
    stone_cap = model.material("stone_cap", rgba=(0.77, 0.76, 0.74, 1.0))
    oak = model.material("oak", rgba=(0.56, 0.37, 0.19, 1.0))
    oak_dark = model.material("oak_dark", rgba=(0.45, 0.28, 0.14, 1.0))
    iron = model.material("iron", rgba=(0.15, 0.15, 0.16, 1.0))

    post_width = 0.18
    post_depth = 0.20
    post_height = 1.46
    cap_width = 0.22
    cap_depth = 0.24
    cap_height = 0.05
    opening_width = 1.00

    hinge_axis_x = 0.018
    gate_bottom_z = 0.118
    gate_thickness = 0.038
    gate_x0 = 0.022
    gate_x1 = 0.952
    stile_width = 0.045
    side_top = 1.05
    peak_top = 1.28
    top_rail_band = 0.085
    bottom_rail_height = 0.10
    picket_bottom = bottom_rail_height

    leaf_upper_hinge_z = 0.92
    leaf_lower_hinge_z = 0.26
    post_hinge_z_offset = gate_bottom_z - 0.08
    upper_hinge_z = leaf_upper_hinge_z + post_hinge_z_offset
    lower_hinge_z = leaf_lower_hinge_z + post_hinge_z_offset

    top_rail_mesh = _arched_top_rail_mesh(
        name="gate_top_rail",
        x0=gate_x0,
        x1=gate_x1,
        side_top=side_top,
        peak_top=peak_top,
        band_height=top_rail_band,
        thickness=gate_thickness,
    )
    hinge_barrel_mesh = _hinge_barrel_mesh(
        name="gate_hinge_barrel",
        inner_radius=0.0075,
        outer_radius=0.016,
        height=0.050,
    )

    threshold = model.part("threshold")
    threshold.visual(
        Box((opening_width + (2.0 * post_width), 0.22, 0.08)),
        origin=Origin(xyz=(0.50, 0.0, 0.04)),
        material=stone,
        name="stone_sill",
    )
    threshold.visual(
        Box((opening_width, 0.16, 0.015)),
        origin=Origin(xyz=(0.50, 0.0, 0.0875)),
        material=stone_cap,
        name="sill_cap",
    )
    threshold.inertial = Inertial.from_geometry(
        Box((opening_width + (2.0 * post_width), 0.22, 0.095)),
        mass=85.0,
        origin=Origin(xyz=(0.50, 0.0, 0.0475)),
    )

    left_post = model.part("left_post")
    left_post.visual(
        Box((post_width, post_depth, post_height)),
        origin=Origin(xyz=(0.0, 0.0, post_height * 0.5)),
        material=stone,
        name="left_post_body",
    )
    left_post.visual(
        Box((cap_width, cap_depth, cap_height)),
        origin=Origin(xyz=(0.0, 0.0, post_height + (cap_height * 0.5))),
        material=stone_cap,
        name="left_post_cap",
    )
    for hinge_name, hinge_z in (("upper", upper_hinge_z), ("lower", lower_hinge_z)):
        left_post.visual(
            Box((0.014, 0.080, 0.120)),
            origin=Origin(xyz=(0.086, 0.0, hinge_z)),
            material=iron,
            name=f"{hinge_name}_hinge_plate",
        )
        left_post.visual(
            Cylinder(radius=0.006, length=0.080),
            origin=Origin(xyz=(0.099, 0.0, hinge_z)),
            material=iron,
            name=f"{hinge_name}_hinge_pin",
        )
        left_post.visual(
            Cylinder(radius=0.010, length=0.010),
            origin=Origin(xyz=(0.090, 0.0, hinge_z + 0.028)),
            material=iron,
            name=f"{hinge_name}_hinge_cap",
        )
    left_post.visual(
        Box((0.016, 0.020, 0.090)),
        origin=Origin(xyz=(0.085, 0.096, 1.14)),
        material=iron,
        name="closer_post_bracket",
    )
    left_post.inertial = Inertial.from_geometry(
        Box((cap_width, cap_depth, post_height + cap_height)),
        mass=120.0,
        origin=Origin(xyz=(0.0, 0.0, (post_height + cap_height) * 0.5)),
    )

    right_post = model.part("right_post")
    right_post.visual(
        Box((post_width, post_depth, post_height)),
        origin=Origin(xyz=(0.0, 0.0, post_height * 0.5)),
        material=stone,
        name="right_post_body",
    )
    right_post.visual(
        Box((cap_width, cap_depth, cap_height)),
        origin=Origin(xyz=(0.0, 0.0, post_height + (cap_height * 0.5))),
        material=stone_cap,
        name="right_post_cap",
    )
    right_post.visual(
        Box((0.010, 0.046, 0.140)),
        origin=Origin(xyz=(-0.095, 0.033, 0.96)),
        material=iron,
        name="striker_plate",
    )
    right_post.visual(
        Box((0.025, 0.025, 0.040)),
        origin=Origin(xyz=(-0.082, 0.033, 0.985)),
        material=iron,
        name="striker_keep",
    )
    right_post.inertial = Inertial.from_geometry(
        Box((cap_width, cap_depth, post_height + cap_height)),
        mass=120.0,
        origin=Origin(xyz=(0.0, 0.0, (post_height + cap_height) * 0.5)),
    )

    gate_leaf = model.part("gate_leaf")
    gate_leaf.visual(
        Box((stile_width, gate_thickness, side_top)),
        origin=Origin(xyz=(gate_x0 + (stile_width * 0.5), 0.0, side_top * 0.5)),
        material=oak,
        name="hinge_stile",
    )
    gate_leaf.visual(
        Box((stile_width, gate_thickness, side_top)),
        origin=Origin(xyz=(gate_x1 - (stile_width * 0.5), 0.0, side_top * 0.5)),
        material=oak,
        name="latch_stile",
    )
    gate_leaf.visual(
        Box(((gate_x1 - gate_x0) - (2.0 * stile_width), gate_thickness, bottom_rail_height)),
        origin=Origin(
            xyz=((gate_x0 + gate_x1) * 0.5, 0.0, bottom_rail_height * 0.5),
        ),
        material=oak_dark,
        name="bottom_rail",
    )
    gate_leaf.visual(top_rail_mesh, material=oak_dark, name="arched_top_rail")
    picket_centers = [0.135 + (0.092 * index) for index in range(8)]
    for index, picket_x in enumerate(picket_centers):
        picket_top = _arch_height(
            picket_x,
            x0=gate_x0,
            x1=gate_x1,
            side_top=side_top,
            peak_top=peak_top,
        ) - top_rail_band - 0.010
        gate_leaf.visual(
            Box((0.026, 0.020, picket_top - picket_bottom)),
            origin=Origin(
                xyz=(
                    picket_x,
                    0.0,
                    picket_bottom + ((picket_top - picket_bottom) * 0.5),
                )
            ),
            material=oak,
            name=f"picket_{index}",
        )
    brace_start = (gate_x0 + 0.070, 0.0, 0.16)
    brace_end = (gate_x1 - 0.085, 0.0, 0.95)
    brace_dx = brace_end[0] - brace_start[0]
    brace_dz = brace_end[2] - brace_start[2]
    gate_leaf.visual(
        Box(((brace_dx * brace_dx + brace_dz * brace_dz) ** 0.5, 0.024, 0.078)),
        origin=Origin(
            xyz=(
                0.5 * (brace_start[0] + brace_end[0]),
                -0.002,
                0.5 * (brace_start[2] + brace_end[2]),
            ),
            rpy=(0.0, atan2(brace_dz, brace_dx), 0.0),
        ),
        material=oak_dark,
        name="diagonal_brace",
    )
    for hinge_name, hinge_z in (("upper", leaf_upper_hinge_z), ("lower", leaf_lower_hinge_z)):
        gate_leaf.visual(
            Box((0.180, 0.014, 0.044)),
            origin=Origin(xyz=(0.104, 0.012, hinge_z)),
            material=iron,
            name=f"{hinge_name}_strap",
        )
        gate_leaf.visual(
            hinge_barrel_mesh,
            origin=Origin(xyz=(0.0, 0.0, hinge_z)),
            material=iron,
            name=f"{hinge_name}_hinge_barrel",
        )
    gate_leaf.visual(
        Box((0.022, 0.014, 0.090)),
        origin=Origin(xyz=(0.192, 0.018, 1.10)),
        material=iron,
        name="closer_gate_bracket",
    )
    gate_leaf.visual(
        Box((0.018, 0.016, 0.040)),
        origin=Origin(xyz=(0.214, 0.033, 1.10)),
        material=iron,
        name="closer_gate_lug",
    )
    gate_leaf.visual(
        Box((0.026, 0.010, 0.040)),
        origin=Origin(xyz=(0.211, 0.028, 1.10)),
        material=iron,
        name="closer_gate_web",
    )
    gate_leaf.inertial = Inertial.from_geometry(
        Box((gate_x1 + 0.016, 0.060, peak_top)),
        mass=28.0,
        origin=Origin(xyz=((gate_x1 + 0.016) * 0.5, 0.0, peak_top * 0.5)),
    )

    thumb_latch = model.part("thumb_latch")
    thumb_latch.visual(
        Cylinder(radius=0.007, length=0.018),
        origin=Origin(rpy=(pi * 0.5, 0.0, 0.0)),
        material=iron,
        name="latch_pivot_hub",
    )
    thumb_latch.visual(
        Box((0.034, 0.010, 0.056)),
        origin=Origin(xyz=(0.011, -0.005, -0.024)),
        material=iron,
        name="latch_mount_plate",
    )
    thumb_latch.visual(
        Box((0.048, 0.012, 0.014)),
        origin=Origin(xyz=(0.024, 0.0, -0.006)),
        material=iron,
        name="latch_handle",
    )
    thumb_latch.visual(
        Box((0.012, 0.016, 0.088)),
        origin=Origin(xyz=(0.000, 0.0, -0.050)),
        material=iron,
        name="thumb_piece",
    )
    thumb_latch.visual(
        Cylinder(radius=0.005, length=0.012),
        origin=Origin(xyz=(0.046, 0.0, 0.006), rpy=(0.0, pi * 0.5, 0.0)),
        material=iron,
        name="latch_nose",
    )
    thumb_latch.inertial = Inertial.from_geometry(
        Box((0.060, 0.020, 0.110)),
        mass=0.45,
        origin=Origin(xyz=(0.023, 0.0, -0.036)),
    )

    closer_post_arm = model.part("closer_post_arm")
    closer_post_arm.visual(
        Cylinder(radius=0.008, length=0.016),
        material=iron,
        name="post_arm_pivot",
    )
    closer_post_arm.visual(
        Box((0.066, 0.014, 0.014)),
        origin=Origin(xyz=(0.033, 0.0, 0.0)),
        material=iron,
        name="post_arm_bar",
    )
    closer_post_arm.visual(
        Cylinder(radius=0.006, length=0.016),
        origin=Origin(xyz=(0.066, 0.0, 0.0)),
        material=iron,
        name="post_arm_eye",
    )
    closer_post_arm.inertial = Inertial.from_geometry(
        Box((0.074, 0.018, 0.018)),
        mass=0.35,
        origin=Origin(xyz=(0.037, 0.0, 0.0)),
    )

    closer_gate_arm = model.part("closer_gate_arm")
    closer_gate_arm.visual(
        Box((0.016, 0.012, 0.036)),
        origin=Origin(xyz=(0.0, 0.006, 0.0)),
        name="gate_arm_pivot",
        material=iron,
    )
    closer_gate_arm.visual(
        Box((0.020, 0.008, 0.010)),
        origin=Origin(xyz=(-0.010, 0.014, 0.0)),
        material=iron,
        name="gate_arm_neck",
    )
    closer_gate_arm.visual(
        Box((0.076, 0.010, 0.010)),
        origin=Origin(xyz=(-0.048, 0.018, 0.0)),
        material=iron,
        name="gate_arm_bar",
    )
    closer_gate_arm.visual(
        Cylinder(radius=0.006, length=0.016),
        origin=Origin(xyz=(-0.090, 0.018, 0.0)),
        material=iron,
        name="gate_arm_eye",
    )
    closer_gate_arm.inertial = Inertial.from_geometry(
        Box((0.100, 0.026, 0.040)),
        mass=0.35,
        origin=Origin(xyz=(-0.045, 0.012, 0.0)),
    )

    model.articulation(
        "threshold_to_left_post",
        ArticulationType.FIXED,
        parent=threshold,
        child=left_post,
        origin=Origin(xyz=(-0.09, 0.0, 0.08)),
    )
    model.articulation(
        "threshold_to_right_post",
        ArticulationType.FIXED,
        parent=threshold,
        child=right_post,
        origin=Origin(xyz=(opening_width + 0.09, 0.0, 0.08)),
    )
    model.articulation(
        "gate_swing",
        ArticulationType.REVOLUTE,
        parent=left_post,
        child=gate_leaf,
        origin=Origin(xyz=(0.108, 0.0, gate_bottom_z - 0.08)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=28.0, velocity=1.2, lower=0.0, upper=1.35),
    )
    model.articulation(
        "latch_pivot",
        ArticulationType.REVOLUTE,
        parent=gate_leaf,
        child=thumb_latch,
        origin=Origin(xyz=(0.880, 0.028, 0.965)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=-0.15, upper=0.55),
    )
    model.articulation(
        "closer_post_pivot",
        ArticulationType.REVOLUTE,
        parent=left_post,
        child=closer_post_arm,
        origin=Origin(xyz=(0.101, 0.106, 1.14)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=-0.30, upper=1.35),
    )
    model.articulation(
        "closer_gate_pivot",
        ArticulationType.REVOLUTE,
        parent=gate_leaf,
        child=closer_gate_arm,
        origin=Origin(xyz=(0.214, 0.041, 1.10)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=-0.20, upper=1.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    threshold = object_model.get_part("threshold")
    left_post = object_model.get_part("left_post")
    right_post = object_model.get_part("right_post")
    gate_leaf = object_model.get_part("gate_leaf")
    thumb_latch = object_model.get_part("thumb_latch")
    closer_post_arm = object_model.get_part("closer_post_arm")
    closer_gate_arm = object_model.get_part("closer_gate_arm")

    gate_swing = object_model.get_articulation("gate_swing")
    latch_pivot = object_model.get_articulation("latch_pivot")
    closer_post_pivot = object_model.get_articulation("closer_post_pivot")
    closer_gate_pivot = object_model.get_articulation("closer_gate_pivot")

    upper_hinge_barrel = gate_leaf.get_visual("upper_hinge_barrel")
    lower_hinge_barrel = gate_leaf.get_visual("lower_hinge_barrel")
    upper_hinge_pin = left_post.get_visual("upper_hinge_pin")
    lower_hinge_pin = left_post.get_visual("lower_hinge_pin")
    latch_mount_plate = thumb_latch.get_visual("latch_mount_plate")
    latch_stile = gate_leaf.get_visual("latch_stile")
    closer_post_bracket = left_post.get_visual("closer_post_bracket")
    closer_gate_bracket = gate_leaf.get_visual("closer_gate_bracket")
    closer_gate_lug = gate_leaf.get_visual("closer_gate_lug")
    post_arm_pivot = closer_post_arm.get_visual("post_arm_pivot")
    gate_arm_pivot = closer_gate_arm.get_visual("gate_arm_pivot")

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
    ctx.allow_overlap(
        gate_leaf,
        left_post,
        reason="Hinge barrels wrap the masonry-post hinge pins as an intentional nested hinge representation.",
        elem_a=upper_hinge_barrel,
        elem_b=upper_hinge_pin,
    )
    ctx.allow_overlap(
        gate_leaf,
        left_post,
        reason="Hinge barrels wrap the masonry-post hinge pins as an intentional nested hinge representation.",
        elem_a=lower_hinge_barrel,
        elem_b=lower_hinge_pin,
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "gate_swing_axis_is_vertical",
        gate_swing.axis == (0.0, 0.0, 1.0),
        details=f"Expected vertical hinge axis, got {gate_swing.axis}.",
    )
    ctx.check(
        "thumb_latch_axis_runs_through_gate",
        latch_pivot.axis == (0.0, 1.0, 0.0),
        details=f"Expected thumb latch axis along gate depth, got {latch_pivot.axis}.",
    )
    ctx.check(
        "closer_support_axes_are_vertical",
        closer_post_pivot.axis == (0.0, 0.0, 1.0) and closer_gate_pivot.axis == (0.0, 0.0, 1.0),
        details=(
            f"Expected both closer pivots vertical, got "
            f"{closer_post_pivot.axis} and {closer_gate_pivot.axis}."
        ),
    )

    ctx.expect_gap(
        gate_leaf,
        threshold,
        axis="z",
        min_gap=0.02,
        max_gap=0.04,
        name="leaf_clears_stone_sill",
    )
    ctx.expect_gap(
        right_post,
        gate_leaf,
        axis="x",
        min_gap=0.008,
        max_gap=0.025,
        name="leaf_stops_just_short_of_strike_post",
    )
    ctx.expect_gap(
        right_post,
        thumb_latch,
        axis="x",
        min_gap=0.02,
        max_gap=0.07,
        name="thumb_latch_stays_clear_of_striker_post",
    )
    ctx.expect_overlap(
        gate_leaf,
        left_post,
        axes="yz",
        elem_a=upper_hinge_barrel,
        elem_b=upper_hinge_pin,
        min_overlap=0.010,
        name="upper_hinge_barrel_tracks_pin_closed",
    )
    ctx.expect_overlap(
        gate_leaf,
        left_post,
        axes="yz",
        elem_a=lower_hinge_barrel,
        elem_b=lower_hinge_pin,
        min_overlap=0.010,
        name="lower_hinge_barrel_tracks_pin_closed",
    )
    ctx.expect_contact(
        thumb_latch,
        gate_leaf,
        elem_a=latch_mount_plate,
        elem_b=latch_stile,
        name="thumb_latch_mounts_to_latch_stile",
    )
    ctx.expect_contact(
        closer_post_arm,
        left_post,
        elem_a=post_arm_pivot,
        elem_b=closer_post_bracket,
        name="post_side_closer_arm_mounts_to_post_bracket",
    )
    ctx.expect_contact(
        closer_gate_arm,
        gate_leaf,
        elem_a=gate_arm_pivot,
        elem_b=closer_gate_lug,
        name="gate_side_closer_arm_mounts_to_gate_bracket",
    )

    latch_stile_rest = ctx.part_element_world_aabb(gate_leaf, elem="latch_stile")
    assert latch_stile_rest is not None

    latch_handle_rest = ctx.part_element_world_aabb(thumb_latch, elem="latch_handle")
    assert latch_handle_rest is not None

    post_eye_rest = ctx.part_element_world_aabb(closer_post_arm, elem="post_arm_eye")
    gate_eye_rest = ctx.part_element_world_aabb(closer_gate_arm, elem="gate_arm_eye")
    assert post_eye_rest is not None
    assert gate_eye_rest is not None

    with ctx.pose({latch_pivot: 0.38}):
        latch_handle_pressed = ctx.part_element_world_aabb(thumb_latch, elem="latch_handle")
        assert latch_handle_pressed is not None
        ctx.check(
            "thumb_latch_rotates_down_when_pressed",
            latch_handle_pressed[0][2] < latch_handle_rest[0][2] - 0.010,
            details=(
                f"Expected latch handle to dip when pressed, got rest min z "
                f"{latch_handle_rest[0][2]:.4f} and pressed min z {latch_handle_pressed[0][2]:.4f}."
            ),
        )

    with ctx.pose({gate_swing: 1.05, closer_post_pivot: 1.10, closer_gate_pivot: 0.45}):
        latch_stile_open = ctx.part_element_world_aabb(gate_leaf, elem="latch_stile")
        assert latch_stile_open is not None
        ctx.check(
            "gate_leaf_swings_open_toward_positive_y",
            latch_stile_open[0][1] > latch_stile_rest[0][1] + 0.70
            and latch_stile_open[1][0] < latch_stile_rest[1][0] - 0.40,
            details=(
                f"Expected open gate to move toward +y and back from the latch post, "
                f"got rest {latch_stile_rest} and open {latch_stile_open}."
            ),
        )
        ctx.expect_overlap(
            gate_leaf,
            left_post,
            axes="yz",
            elem_a=upper_hinge_barrel,
            elem_b=upper_hinge_pin,
            min_overlap=0.010,
            name="upper_hinge_barrel_tracks_pin_open",
        )
        ctx.expect_overlap(
            gate_leaf,
            left_post,
            axes="yz",
            elem_a=lower_hinge_barrel,
            elem_b=lower_hinge_pin,
            min_overlap=0.010,
            name="lower_hinge_barrel_tracks_pin_open",
        )

        post_eye_open = ctx.part_element_world_aabb(closer_post_arm, elem="post_arm_eye")
        gate_eye_open = ctx.part_element_world_aabb(closer_gate_arm, elem="gate_arm_eye")
        assert post_eye_open is not None
        assert gate_eye_open is not None
        ctx.check(
            "closer_post_arm_rotates_at_post_support",
            post_eye_open[0][1] > post_eye_rest[0][1] + 0.03,
            details=(
                f"Expected post-side closer eye to sweep outward in y, "
                f"got rest {post_eye_rest} and open {post_eye_open}."
            ),
        )
        ctx.check(
            "closer_gate_arm_rotates_with_gate",
            (
                (((gate_eye_open[0][0] + gate_eye_open[1][0]) * 0.5) - ((gate_eye_rest[0][0] + gate_eye_rest[1][0]) * 0.5)) ** 2
                + (((gate_eye_open[0][1] + gate_eye_open[1][1]) * 0.5) - ((gate_eye_rest[0][1] + gate_eye_rest[1][1]) * 0.5)) ** 2
            ) ** 0.5 > 0.08
            and ((gate_eye_open[0][1] + gate_eye_open[1][1]) * 0.5) > ((gate_eye_rest[0][1] + gate_eye_rest[1][1]) * 0.5) + 0.05,
            details=(
                f"Expected gate-side closer eye to travel with the opening leaf, "
                f"got rest {gate_eye_rest} and open {gate_eye_open}."
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
