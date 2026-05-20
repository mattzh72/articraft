from __future__ import annotations

import math

import cadquery as cq
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
    TorusGeometry,
    mesh_from_geometry,
    superellipse_profile,
)


def _tube_pose(start: tuple[float, float, float], end: tuple[float, float, float]) -> tuple[Origin, float]:
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    origin = Origin(
        xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
        rpy=(0.0, pitch, yaw),
    )
    return origin, length


def _add_tube(part, start, end, radius, material, name):
    origin, length = _tube_pose(start, end)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stationary_exercise_bike")

    matte_black = model.material("matte_black", rgba=(0.02, 0.022, 0.024, 1.0))
    charcoal = model.material("charcoal_plastic", rgba=(0.10, 0.115, 0.125, 1.0))
    rubber = model.material("black_rubber", rgba=(0.006, 0.006, 0.005, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.20, 1.0))
    accent = model.material("red_accent", rgba=(0.72, 0.04, 0.025, 1.0))
    saddle_mat = model.material("saddle_vinyl", rgba=(0.015, 0.013, 0.012, 1.0))

    frame = model.part("floor_frame")

    # Floor-contacting frame, kept compact in width so the stationary bike has a tight silhouette.
    _add_tube(frame, (-0.56, 0.0, 0.07), (0.50, 0.0, 0.07), 0.024, matte_black, "center_rail")
    _add_tube(frame, (-0.50, -0.25, 0.045), (-0.50, 0.25, 0.045), 0.026, matte_black, "front_foot")
    _add_tube(frame, (0.43, -0.27, 0.045), (0.43, 0.27, 0.045), 0.026, matte_black, "rear_foot")

    for idx, y in enumerate((-0.235, 0.235)):
        frame.visual(
            Box((0.11, 0.055, 0.026)),
            origin=Origin(xyz=(0.43, y, 0.022)),
            material=rubber,
            name=f"rear_pad_{idx}",
        )
        frame.visual(
            Cylinder(radius=0.052, length=0.030),
            origin=Origin(xyz=(-0.535, y * 0.82, 0.055), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name=f"transport_wheel_{idx}",
        )

    # Main frame tubes and fixed bearing housings.
    flywheel_center = (-0.34, 0.0, 0.43)
    bottom_bracket = (-0.065, 0.0, 0.43)
    _add_tube(frame, (-0.46, 0.0, 0.07), (-0.17, 0.0, 0.12), 0.022, matte_black, "lower_down_tube")
    _add_tube(frame, (-0.17, 0.0, 0.12), (-0.09, 0.0, 0.37), 0.022, matte_black, "crank_support")
    _add_tube(frame, (-0.065, 0.0, 0.476), (0.192, 0.0, 0.76), 0.026, matte_black, "top_tube")
    _add_tube(frame, (0.43, 0.0, 0.07), (0.25, 0.0, 0.31), 0.022, matte_black, "seat_stay")
    _add_tube(frame, (0.25, 0.0, 0.31), (0.25, 0.0, 0.80), 0.032, matte_black, "seat_sleeve")

    frame.visual(
        Cylinder(radius=0.046, length=0.160),
        origin=Origin(xyz=bottom_bracket, rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="bottom_bracket_shell",
    )
    frame.visual(
        Box((0.32, 0.030, 0.200)),
        origin=Origin(xyz=(-0.205, -0.180, 0.43)),
        material=charcoal,
        name="drive_cover",
    )
    frame.visual(
        Box((0.200, 0.170, 0.030)),
        origin=Origin(xyz=(-0.100, -0.105, 0.520)),
        material=charcoal,
        name="cover_mount",
    )

    # Compact flywheel shroud: a stationary guard ring with close-set side bearing ears.
    frame.visual(
        mesh_from_geometry(TorusGeometry(radius=0.235, tube=0.022), "flywheel_guard"),
        origin=Origin(xyz=flywheel_center, rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="flywheel_guard",
    )
    _add_tube(frame, (-0.41, 0.0, 0.07), (-0.34, 0.0, 0.195), 0.018, matte_black, "guard_strut")
    _add_tube(frame, (-0.46, -0.055, 0.10), (-0.46, 0.055, 0.10), 0.016, matte_black, "fork_bridge")
    for idx, y in enumerate((-0.055, 0.055)):
        _add_tube(frame, (-0.46, y, 0.10), (-0.355, y, 0.405), 0.012, matte_black, f"flywheel_fork_{idx}")
        frame.visual(
            Cylinder(radius=0.030, length=0.026),
            origin=Origin(xyz=(-0.34, y, 0.43), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"flywheel_bearing_{idx}",
        )

    # Handlebar mast and compact bullhorn-style grips.
    _add_tube(frame, (-0.57, 0.0, 0.07), (-0.55, 0.0, 0.92), 0.025, matte_black, "handlebar_mast")
    _add_tube(frame, (-0.55, 0.0, 0.92), (-0.48, 0.0, 1.00), 0.020, matte_black, "handlebar_stem")
    _add_tube(frame, (-0.48, -0.24, 1.00), (-0.48, 0.24, 1.00), 0.018, matte_black, "handlebar")
    for idx, y in enumerate((-0.285, 0.285)):
        _add_tube(frame, (-0.48, y * 0.84, 1.00), (-0.48, y, 1.00), 0.021, rubber, f"grip_{idx}")

    # Rotating front flywheel, nested just inside the shroud.
    flywheel = model.part("flywheel")
    flywheel.visual(
        Cylinder(radius=0.185, length=0.038),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="disc",
    )
    flywheel.visual(
        Cylinder(radius=0.016, length=0.140),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="axle",
    )
    flywheel.visual(
        Cylinder(radius=0.050, length=0.054),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hub",
    )
    for idx in range(6):
        theta = idx * math.tau / 6.0
        flywheel.visual(
            Box((0.145, 0.011, 0.014)),
            origin=Origin(
                xyz=(0.062 * math.cos(theta), 0.0, 0.062 * math.sin(theta)),
                rpy=(0.0, -theta, 0.0),
            ),
            material=steel,
            name=f"spoke_{idx}",
        )

    model.articulation(
        "flywheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=flywheel,
        origin=Origin(xyz=flywheel_center),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=30.0),
    )

    # Crank set at the bottom bracket. Pedals are kept close to the bike body.
    crank = model.part("crank")
    crank.visual(
        Cylinder(radius=0.018, length=0.300),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="spindle",
    )
    crank.visual(
        Cylinder(radius=0.062, length=0.020),
        origin=Origin(xyz=(0.0, -0.105, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="chainring",
    )
    crank.visual(
        Box((0.030, 0.014, 0.210)),
        origin=Origin(xyz=(0.0, -0.145, -0.105)),
        material=steel,
        name="arm_0",
    )
    crank.visual(
        Box((0.030, 0.014, 0.210)),
        origin=Origin(xyz=(0.0, 0.145, 0.105)),
        material=steel,
        name="arm_1",
    )
    crank.visual(
        Box((0.115, 0.045, 0.026)),
        origin=Origin(xyz=(0.0, -0.162, -0.215)),
        material=rubber,
        name="pedal_0",
    )
    crank.visual(
        Box((0.115, 0.045, 0.026)),
        origin=Origin(xyz=(0.0, 0.162, 0.215)),
        material=rubber,
        name="pedal_1",
    )
    model.articulation(
        "crank_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=crank,
        origin=Origin(xyz=bottom_bracket),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=18.0),
    )

    # Adjustable seat post: the child frame sits at the sleeve lip and slides upward.
    seat_post = model.part("seat_post")
    seat_post.visual(
        Cylinder(radius=0.023, length=0.540),
        origin=Origin(xyz=(0.0, 0.0, -0.070)),
        material=steel,
        name="inner_post",
    )
    saddle_main = ExtrudeGeometry(superellipse_profile(0.245, 0.170, exponent=2.8, segments=48), 0.046)
    saddle_nose = ExtrudeGeometry(superellipse_profile(0.145, 0.080, exponent=2.4, segments=40), 0.042)
    seat_post.visual(
        mesh_from_geometry(saddle_main, "saddle_main"),
        origin=Origin(xyz=(0.055, 0.0, 0.245)),
        material=saddle_mat,
        name="saddle_rear",
    )
    seat_post.visual(
        mesh_from_geometry(saddle_nose, "saddle_nose"),
        origin=Origin(xyz=(-0.080, 0.0, 0.242)),
        material=saddle_mat,
        name="saddle_nose",
    )
    seat_post.visual(
        Box((0.090, 0.070, 0.020)),
        origin=Origin(xyz=(-0.005, 0.0, 0.205)),
        material=dark_steel,
        name="saddle_clamp",
    )
    _add_tube(seat_post, (-0.060, -0.035, 0.214), (0.078, -0.035, 0.224), 0.006, dark_steel, "saddle_rail_0")
    _add_tube(seat_post, (-0.060, 0.035, 0.214), (0.078, 0.035, 0.224), 0.006, dark_steel, "saddle_rail_1")
    model.articulation(
        "seat_height",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=seat_post,
        origin=Origin(xyz=(0.25, 0.0, 0.80)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=100.0, velocity=0.20, lower=0.0, upper=0.18),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("floor_frame")
    flywheel = object_model.get_part("flywheel")
    crank = object_model.get_part("crank")
    seat_post = object_model.get_part("seat_post")
    flywheel_spin = object_model.get_articulation("flywheel_spin")
    crank_spin = object_model.get_articulation("crank_spin")
    seat_height = object_model.get_articulation("seat_height")

    ctx.check("flywheel joint is continuous", flywheel_spin.articulation_type == ArticulationType.CONTINUOUS)
    ctx.check("crank joint is continuous", crank_spin.articulation_type == ArticulationType.CONTINUOUS)
    ctx.check("seat joint is prismatic", seat_height.articulation_type == ArticulationType.PRISMATIC)

    ctx.allow_overlap(
        frame,
        crank,
        elem_a="bottom_bracket_shell",
        elem_b="spindle",
        reason="The crank spindle is intentionally captured inside the bottom-bracket bearing shell.",
    )
    ctx.expect_within(
        crank,
        frame,
        axes="xz",
        inner_elem="spindle",
        outer_elem="bottom_bracket_shell",
        margin=0.002,
        name="spindle is centered in bottom bracket",
    )
    ctx.expect_overlap(
        crank,
        frame,
        axes="y",
        elem_a="spindle",
        elem_b="bottom_bracket_shell",
        min_overlap=0.14,
        name="spindle passes through bottom bracket",
    )

    for bearing in ("flywheel_bearing_0", "flywheel_bearing_1"):
        ctx.allow_overlap(
            frame,
            flywheel,
            elem_a=bearing,
            elem_b="axle",
            reason="The flywheel axle is intentionally captured in the side bearing ear.",
        )
        ctx.expect_within(
            flywheel,
            frame,
            axes="xz",
            inner_elem="axle",
            outer_elem=bearing,
            margin=0.002,
            name=f"axle is centered in {bearing}",
        )

    ctx.expect_within(
        flywheel,
        frame,
        axes="xz",
        inner_elem="disc",
        outer_elem="flywheel_guard",
        margin=0.0,
        name="flywheel stays inside compact guard",
    )

    ctx.allow_overlap(
        frame,
        seat_post,
        elem_a="seat_sleeve",
        elem_b="inner_post",
        reason="The adjustable post is intentionally represented as a sliding member inside the seat sleeve.",
    )
    ctx.expect_within(
        seat_post,
        frame,
        axes="xy",
        inner_elem="inner_post",
        outer_elem="seat_sleeve",
        margin=0.002,
        name="seat post stays centered in sleeve",
    )
    ctx.expect_overlap(
        seat_post,
        frame,
        axes="z",
        elem_a="inner_post",
        elem_b="seat_sleeve",
        min_overlap=0.25,
        name="lowered seat post remains deeply inserted",
    )

    lowered = ctx.part_world_position(seat_post)
    with ctx.pose({seat_height: 0.18, crank_spin: math.pi / 2.0, flywheel_spin: math.pi / 2.0}):
        raised = ctx.part_world_position(seat_post)
        ctx.expect_within(
            seat_post,
            frame,
            axes="xy",
            inner_elem="inner_post",
            outer_elem="seat_sleeve",
            margin=0.002,
            name="raised seat post stays centered in sleeve",
        )
        ctx.expect_overlap(
            seat_post,
            frame,
            axes="z",
            elem_a="inner_post",
            elem_b="seat_sleeve",
            min_overlap=0.12,
            name="raised seat post retains insertion",
        )

    ctx.check(
        "seat adjustment moves upward",
        lowered is not None and raised is not None and raised[2] > lowered[2] + 0.15,
        details=f"lowered={lowered}, raised={raised}",
    )

    return ctx.report()


object_model = build_object_model()
