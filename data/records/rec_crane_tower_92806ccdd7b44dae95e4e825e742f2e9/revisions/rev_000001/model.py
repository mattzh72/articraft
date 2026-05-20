from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _beam_origin_between(p0: tuple[float, float, float], p1: tuple[float, float, float]) -> tuple[Origin, float]:
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError("beam endpoints must differ")
    yaw = math.atan2(dy, dx)
    horizontal = math.sqrt(dx * dx + dy * dy)
    pitch = math.atan2(-dz, horizontal)
    origin = Origin(
        xyz=((p0[0] + p1[0]) * 0.5, (p0[1] + p1[1]) * 0.5, (p0[2] + p1[2]) * 0.5),
        rpy=(0.0, pitch, yaw),
    )
    return origin, length


def _add_beam(part, name: str, p0, p1, thickness: float, material: Material) -> None:
    origin, length = _beam_origin_between(p0, p1)
    part.visual(Box((length, thickness, thickness)), origin=origin, material=material, name=name)


def _lerp(a: tuple[float, float, float], b: tuple[float, float, float], t: float) -> tuple[float, float, float]:
    return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="luffing_jib_tower_crane")

    yellow = Material("crane_yellow", rgba=(1.0, 0.72, 0.05, 1.0))
    dark = Material("dark_steel", rgba=(0.08, 0.08, 0.08, 1.0))
    concrete = Material("concrete_grey", rgba=(0.46, 0.45, 0.42, 1.0))
    cab_white = Material("cab_white", rgba=(0.92, 0.90, 0.84, 1.0))
    glass = Material("blue_tinted_glass", rgba=(0.25, 0.45, 0.65, 0.65))
    red = Material("hook_red", rgba=(0.82, 0.05, 0.04, 1.0))

    mast = model.part("mast")
    mast.visual(Box((4.2, 3.4, 0.40)), origin=Origin(xyz=(0.0, 0.0, 0.20)), material=concrete, name="base_slab")
    mast.visual(Box((1.55, 1.55, 0.32)), origin=Origin(xyz=(0.0, 0.0, 0.56)), material=yellow, name="mast_foot_frame")

    half = 0.55
    z_bottom = 0.40
    z_top = 16.80
    corners = [(-half, -half), (half, -half), (half, half), (-half, half)]
    for i, (x, y) in enumerate(corners):
        _add_beam(mast, f"mast_leg_{i}", (x, y, z_bottom), (x, y, z_top), 0.16, yellow)

    ring_levels = [0.60, 3.25, 5.90, 8.55, 11.20, 13.85, 16.50]
    for level_i, z in enumerate(ring_levels):
        pts = [(x, y, z) for x, y in corners]
        for side_i in range(4):
            _add_beam(mast, f"mast_ring_{level_i}_{side_i}", pts[side_i], pts[(side_i + 1) % 4], 0.08, yellow)

    for bay_i in range(len(ring_levels) - 1):
        z0 = ring_levels[bay_i]
        z1 = ring_levels[bay_i + 1]
        # X-bracing on all four faces of the tower section.
        _add_beam(mast, f"front_brace_a_{bay_i}", (-half, -half, z0), (half, -half, z1), 0.045, yellow)
        _add_beam(mast, f"front_brace_b_{bay_i}", (half, -half, z0), (-half, -half, z1), 0.045, yellow)
        _add_beam(mast, f"rear_brace_a_{bay_i}", (-half, half, z0), (half, half, z1), 0.045, yellow)
        _add_beam(mast, f"rear_brace_b_{bay_i}", (half, half, z0), (-half, half, z1), 0.045, yellow)
        _add_beam(mast, f"side_brace_a_{bay_i}", (-half, -half, z0), (-half, half, z1), 0.045, yellow)
        _add_beam(mast, f"side_brace_b_{bay_i}", (-half, half, z0), (-half, -half, z1), 0.045, yellow)
        _add_beam(mast, f"side_brace_c_{bay_i}", (half, -half, z0), (half, half, z1), 0.045, yellow)
        _add_beam(mast, f"side_brace_d_{bay_i}", (half, half, z0), (half, -half, z1), 0.045, yellow)

    mast.visual(Box((1.80, 1.80, 0.36)), origin=Origin(xyz=(0.0, 0.0, 16.92)), material=yellow, name="slew_frame")
    mast.visual(Box((3.15, 1.55, 0.33)), origin=Origin(xyz=(-0.45, 0.0, 17.23)), material=yellow, name="machinery_deck")
    mast.visual(Box((0.70, 1.20, 0.95)), origin=Origin(xyz=(-1.75, 0.0, 17.78)), material=concrete, name="counterweight_stack")
    mast.visual(Box((0.82, 0.58, 0.58)), origin=Origin(xyz=(0.52, -1.00, 17.38)), material=cab_white, name="operator_cab")
    mast.visual(Box((0.035, 0.48, 0.32)), origin=Origin(xyz=(0.95, -1.00, 17.44)), material=glass, name="cab_front_window")
    mast.visual(Box((0.42, 0.045, 0.30)), origin=Origin(xyz=(0.54, -1.305, 17.44)), material=glass, name="cab_side_window")

    hinge_x = 0.82
    hinge_z = 17.85
    mast.visual(Box((0.24, 0.12, 0.80)), origin=Origin(xyz=(hinge_x, -0.36, hinge_z)), material=yellow, name="hinge_yoke_0")
    mast.visual(Box((0.24, 0.12, 0.80)), origin=Origin(xyz=(hinge_x, 0.36, hinge_z)), material=yellow, name="hinge_yoke_1")
    _add_beam(mast, "yoke_post_0", (0.72, -0.36, 17.30), (hinge_x, -0.36, hinge_z + 0.35), 0.10, yellow)
    _add_beam(mast, "yoke_post_1", (0.72, 0.36, 17.30), (hinge_x, 0.36, hinge_z + 0.35), 0.10, yellow)

    # Tall luffing-head gantry above the machinery deck.
    mast.visual(
        Cylinder(radius=0.14, length=0.90),
        origin=Origin(xyz=(-0.35, 0.0, 19.95), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="luffing_head_sheave",
    )
    _add_beam(mast, "gantry_leg_0", (0.35, -0.45, 17.30), (-0.35, -0.18, 19.95), 0.10, yellow)
    _add_beam(mast, "gantry_leg_1", (0.35, 0.45, 17.30), (-0.35, 0.18, 19.95), 0.10, yellow)
    _add_beam(mast, "rear_backstay_0", (-0.35, -0.18, 19.95), (-1.95, -0.40, 18.22), 0.055, dark)
    _add_beam(mast, "rear_backstay_1", (-0.35, 0.18, 19.95), (-1.95, 0.40, 18.22), 0.055, dark)

    jib = model.part("jib")
    jib.visual(
        Cylinder(radius=0.19, length=0.60),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="hinge_barrel",
    )
    jib.visual(Box((0.50, 0.08, 0.32)), origin=Origin(xyz=(0.22, -0.22, 0.0)), material=yellow, name="root_cheek_0")
    jib.visual(Box((0.50, 0.08, 0.32)), origin=Origin(xyz=(0.22, 0.22, 0.0)), material=yellow, name="root_cheek_1")

    length = 14.0
    lower_a0 = (0.05, -0.20, -0.05)
    lower_a1 = (length, -0.15, 0.00)
    lower_b0 = (0.05, 0.20, -0.05)
    lower_b1 = (length, 0.15, 0.00)
    upper0 = (0.25, 0.0, 0.70)
    upper1 = (length - 0.15, 0.0, 0.18)
    _add_beam(jib, "lower_chord_0", lower_a0, lower_a1, 0.12, yellow)
    _add_beam(jib, "lower_chord_1", lower_b0, lower_b1, 0.12, yellow)
    _add_beam(jib, "upper_chord", upper0, upper1, 0.11, yellow)
    _add_beam(jib, "root_upper_post", (0.00, 0.0, 0.0), upper0, 0.10, yellow)
    _add_beam(jib, "tip_closing_post_0", lower_a1, upper1, 0.08, yellow)
    _add_beam(jib, "tip_closing_post_1", lower_b1, upper1, 0.08, yellow)

    bays = 7
    for i in range(bays + 1):
        t = i / bays
        left = _lerp(lower_a0, lower_a1, t)
        right = _lerp(lower_b0, lower_b1, t)
        top = _lerp(upper0, upper1, t)
        _add_beam(jib, f"cross_tie_{i}", left, right, 0.055, yellow)
        _add_beam(jib, f"web_post_0_{i}", left, top, 0.052, yellow)
        _add_beam(jib, f"web_post_1_{i}", right, top, 0.052, yellow)
        if i < bays:
            next_top = _lerp(upper0, upper1, (i + 1) / bays)
            next_left = _lerp(lower_a0, lower_a1, (i + 1) / bays)
            next_right = _lerp(lower_b0, lower_b1, (i + 1) / bays)
            _add_beam(jib, f"side_web_0_{i}", left, next_top, 0.045, yellow)
            _add_beam(jib, f"side_web_1_{i}", right, next_top, 0.045, yellow)
            _add_beam(jib, f"reverse_web_0_{i}", top, next_left, 0.040, yellow)
            _add_beam(jib, f"reverse_web_1_{i}", top, next_right, 0.040, yellow)

    jib.visual(
        Cylinder(radius=0.24, length=0.28),
        origin=Origin(xyz=(length, 0.0, -0.24), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="tip_sheave",
    )
    jib.visual(Box((0.40, 0.50, 0.15)), origin=Origin(xyz=(length - 0.08, 0.0, -0.03)), material=yellow, name="tip_frame")
    _add_beam(jib, "pendant_tie_0", (0.60, -0.05, 0.70), (length - 0.50, -0.04, 0.25), 0.035, dark)
    _add_beam(jib, "pendant_tie_1", (0.60, 0.05, 0.70), (length - 0.50, 0.04, 0.25), 0.035, dark)

    hook_block = model.part("hook_block")
    hook_block.visual(
        Cylinder(radius=0.032, length=2.95),
        origin=Origin(xyz=(0.0, 0.0, -1.475)),
        material=dark,
        name="hoist_line",
    )
    hook_block.visual(Box((0.46, 0.30, 0.34)), origin=Origin(xyz=(0.0, 0.0, -3.12)), material=red, name="block_case")
    hook_block.visual(
        Cylinder(radius=0.13, length=0.34),
        origin=Origin(xyz=(0.0, 0.0, -3.12), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="block_sheave",
    )
    hook_mesh = tube_from_spline_points(
        [
            (0.00, 0.0, -3.28),
            (0.00, 0.0, -3.55),
            (0.20, 0.0, -3.72),
            (0.22, 0.0, -3.96),
            (0.02, 0.0, -4.12),
            (-0.19, 0.0, -4.02),
            (-0.22, 0.0, -3.82),
        ],
        radius=0.045,
        samples_per_segment=10,
        radial_segments=16,
        cap_ends=True,
    )
    hook_block.visual(mesh_from_geometry(hook_mesh, "curved_hook_mesh"), material=dark, name="curved_hook")

    model.articulation(
        "mast_to_jib",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=jib,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=250000.0, velocity=0.18, lower=0.0, upper=1.05),
    )
    model.articulation(
        "jib_to_hook",
        ArticulationType.REVOLUTE,
        parent=jib,
        child=hook_block,
        origin=Origin(xyz=(length, 0.0, -0.48)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.0, lower=-0.55, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    jib = object_model.get_part("jib")
    hook_block = object_model.get_part("hook_block")
    luff = object_model.get_articulation("mast_to_jib")
    swing = object_model.get_articulation("jib_to_hook")

    ctx.expect_contact(
        jib,
        hook_block,
        elem_a="tip_sheave",
        elem_b="hoist_line",
        contact_tol=0.08,
        name="hoist line is seated at the jib tip sheave",
    )
    ctx.expect_gap(
        jib,
        hook_block,
        axis="z",
        max_gap=0.02,
        max_penetration=0.0,
        positive_elem="tip_sheave",
        negative_elem="hoist_line",
        name="cable starts directly below tip sheave",
    )
    ctx.check(
        "luffing jib has realistic upward range",
        luff.motion_limits is not None
        and luff.motion_limits.lower == 0.0
        and luff.motion_limits.upper is not None
        and luff.motion_limits.upper > 0.9,
        details=str(luff.motion_limits),
    )

    mast_aabb = ctx.part_world_aabb(mast)
    jib_aabb = ctx.part_world_aabb(jib)
    ctx.check(
        "tower mast is tall relative to jib depth",
        mast_aabb is not None and (mast_aabb[1][2] - mast_aabb[0][2]) > 18.0,
        details=f"mast_aabb={mast_aabb}",
    )
    ctx.check(
        "jib projects well beyond the mast",
        jib_aabb is not None and (jib_aabb[1][0] - jib_aabb[0][0]) > 13.0,
        details=f"jib_aabb={jib_aabb}",
    )

    rest_aabb = ctx.part_world_aabb(jib)
    with ctx.pose({luff: 0.95, swing: -0.50}):
        raised_aabb = ctx.part_world_aabb(jib)
    ctx.check(
        "positive luffing angle raises the jib tip",
        rest_aabb is not None and raised_aabb is not None and raised_aabb[1][2] > rest_aabb[1][2] + 5.0,
        details=f"rest={rest_aabb}, raised={raised_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
