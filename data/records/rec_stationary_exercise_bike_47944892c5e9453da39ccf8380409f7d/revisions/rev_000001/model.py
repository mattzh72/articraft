from __future__ import annotations

import math

import cadquery as cq
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
    mesh_from_cadquery,
)


def _cylinder_between(part, name, p0, p1, radius, material):
    """Add a round tube between two points, for the mostly X/Z/Y frame tubes."""
    x0, y0, z0 = p0
    x1, y1, z1 = p1
    dx, dy, dz = x1 - x0, y1 - y0, z1 - z0
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0:
        raise ValueError(f"zero length tube {name}")

    # All authored tubes are either in the XZ plane or along Y, so a single
    # principal rotation is enough and keeps the joint/geometry frames readable.
    if abs(dy) > 1e-9 and abs(dx) < 1e-9 and abs(dz) < 1e-9:
        rpy = (-math.pi / 2.0 if dy > 0 else math.pi / 2.0, 0.0, 0.0)
    else:
        rpy = (0.0, math.atan2(dx, dz), 0.0)

    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((x0 + x1) / 2.0, (y0 + y1) / 2.0, (z0 + z1) / 2.0),
            rpy=rpy,
        ),
        material=material,
        name=name,
    )


def _annular_disk_mesh(name: str, outer_radius: float, inner_radius: float, width: float):
    outer = cq.Workplane("XZ").circle(outer_radius).extrude(width, both=True)
    inner = cq.Workplane("XZ").circle(inner_radius).extrude(width * 1.25, both=True)
    return mesh_from_cadquery(
        outer.cut(inner),
        name,
        tolerance=0.0012,
        angular_tolerance=0.08,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stationary_exercise_bike")

    satin_black = model.material("satin_black", rgba=(0.015, 0.016, 0.018, 1.0))
    rubber = model.material("rubber_black", rgba=(0.005, 0.005, 0.004, 1.0))
    red = model.material("powder_coat_red", rgba=(0.78, 0.045, 0.025, 1.0))
    dark_gray = model.material("dark_graphite", rgba=(0.08, 0.085, 0.09, 1.0))
    metal = model.material("brushed_steel", rgba=(0.65, 0.67, 0.66, 1.0))
    vinyl = model.material("matte_vinyl", rgba=(0.02, 0.018, 0.016, 1.0))

    frame = model.part("floor_frame")

    # Floor contact and wide stabilizer stance.  The small transport rollers are
    # deliberately set far outboard of the tubes for the wider stance requested.
    _cylinder_between(frame, "center_spine", (-0.56, 0.0, 0.070), (0.62, 0.0, 0.070), 0.026, red)
    _cylinder_between(frame, "front_stabilizer", (-0.52, -0.60, 0.055), (-0.52, 0.60, 0.055), 0.027, red)
    _cylinder_between(frame, "rear_stabilizer", (0.55, -0.60, 0.055), (0.55, 0.60, 0.055), 0.027, red)

    for x, prefix in ((-0.52, "front"), (0.55, "rear")):
        for y, suffix in ((-0.622, "negative"), (0.622, "positive")):
            frame.visual(
                Cylinder(radius=0.055, length=0.052),
                origin=Origin(xyz=(x, y, 0.055), rpy=(-math.pi / 2.0, 0.0, 0.0)),
                material=rubber,
                name=f"{prefix}_wheel_{suffix}",
            )
            frame.visual(
                Cylinder(radius=0.018, length=0.074),
                origin=Origin(xyz=(x, y * 0.978, 0.055), rpy=(-math.pi / 2.0, 0.0, 0.0)),
                material=metal,
                name=f"{prefix}_axle_{suffix}",
            )

    # Main triangular frame, bottom bracket support, seat sleeve, and handlebar mast.
    bottom_bracket = (0.0, 0.0, 0.53)
    flywheel_center = (-0.34, 0.0, 0.49)
    seat_sleeve_top = (0.26, 0.0, 0.73)
    _cylinder_between(frame, "front_upright", (-0.52, 0.0, 0.070), (-0.34, 0.0, 0.235), 0.023, red)
    _cylinder_between(frame, "flywheel_strut", (-0.52, 0.0, 0.070), (-0.145, 0.0, 0.292), 0.022, red)
    _cylinder_between(frame, "bottom_tube", (-0.45, 0.0, 0.078), (0.0, 0.0, 0.484), 0.024, red)
    _cylinder_between(frame, "top_tube", (0.044, 0.0, 0.545), (0.205, 0.0, 0.708), 0.023, red)
    _cylinder_between(frame, "rear_stay", (0.55, 0.0, 0.070), (0.26, 0.0, 0.45), 0.022, red)
    _cylinder_between(frame, "handlebar_mast", (-0.08, 0.0, 0.55), (-0.18, 0.0, 1.13), 0.024, red)
    _cylinder_between(frame, "handlebar_brace", (-0.07, 0.0, 0.57), (-0.18, 0.0, 1.03), 0.018, red)

    frame.visual(
        _annular_disk_mesh("bottom_bracket_shell", 0.048, 0.017, 0.125),
        origin=Origin(xyz=bottom_bracket),
        material=dark_gray,
        name="bottom_bracket_shell",
    )

    seat_sleeve_outer = cq.Workplane("XY").circle(0.037).extrude(0.36, both=True)
    seat_sleeve_inner = cq.Workplane("XY").circle(0.024).extrude(0.42, both=True)
    seat_sleeve = mesh_from_cadquery(
        seat_sleeve_outer.cut(seat_sleeve_inner),
        "seat_sleeve",
        tolerance=0.001,
        angular_tolerance=0.08,
    )
    frame.visual(
        seat_sleeve,
        origin=Origin(xyz=(0.26, 0.0, 0.55)),
        material=dark_gray,
        name="seat_sleeve",
    )
    frame.visual(
        Box((0.110, 0.024, 0.030)),
        origin=Origin(xyz=(0.26, -0.045, 0.725)),
        material=dark_gray,
        name="seat_clamp_negative",
    )
    frame.visual(
        Box((0.110, 0.024, 0.030)),
        origin=Origin(xyz=(0.26, 0.045, 0.725)),
        material=dark_gray,
        name="seat_clamp_positive",
    )
    frame.visual(
        Cylinder(radius=0.007, length=0.105),
        origin=Origin(xyz=(0.300, 0.0, 0.725), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="seat_clamp_bolt",
    )
    frame.visual(
        Box((0.030, 0.046, 0.040)),
        origin=Origin(xyz=(0.285, 0.0, 0.725)),
        material=dark_gray,
        name="seat_clamp_pad",
    )

    # Flywheel guard: a real open annular housing with side bearing rings and
    # spokes, leaving the rotating metal flywheel visible through the center.
    frame.visual(
        _annular_disk_mesh("flywheel_housing", 0.285, 0.205, 0.102),
        origin=Origin(xyz=flywheel_center),
        material=satin_black,
        name="flywheel_housing",
    )
    frame.visual(
        _annular_disk_mesh("flywheel_bearing_near", 0.047, 0.016, 0.018),
        origin=Origin(xyz=(flywheel_center[0], -0.064, flywheel_center[2])),
        material=dark_gray,
        name="flywheel_bearing_near",
    )
    frame.visual(
        _annular_disk_mesh("flywheel_bearing_far", 0.047, 0.016, 0.018),
        origin=Origin(xyz=(flywheel_center[0], 0.064, flywheel_center[2])),
        material=dark_gray,
        name="flywheel_bearing_far",
    )
    for y, side in ((-0.064, "near"), (0.064, "far")):
        _cylinder_between(
            frame,
            f"flywheel_spoke_upper_{side}",
            (flywheel_center[0], y, flywheel_center[2] + 0.043),
            (flywheel_center[0], y, flywheel_center[2] + 0.214),
            0.010,
            dark_gray,
        )
        _cylinder_between(
            frame,
            f"flywheel_spoke_forward_{side}",
            (flywheel_center[0] - 0.043, y, flywheel_center[2]),
            (flywheel_center[0] - 0.214, y, flywheel_center[2]),
            0.010,
            dark_gray,
        )

    # Static handlebar with padded grips.
    _cylinder_between(frame, "handlebar_crossbar", (-0.19, -0.35, 1.16), (-0.19, 0.35, 1.16), 0.019, satin_black)
    _cylinder_between(frame, "handlebar_stem", (-0.18, 0.0, 1.08), (-0.19, 0.0, 1.16), 0.018, satin_black)
    _cylinder_between(frame, "negative_grip", (-0.19, -0.49, 1.16), (-0.19, -0.35, 1.16), 0.024, rubber)
    _cylinder_between(frame, "positive_grip", (-0.19, 0.35, 1.16), (-0.19, 0.49, 1.16), 0.024, rubber)

    flywheel = model.part("flywheel")
    flywheel.visual(
        Cylinder(radius=0.182, length=0.040),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="flywheel_disc",
    )
    flywheel.visual(
        Cylinder(radius=0.017, length=0.155),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="flywheel_axle",
    )

    crank = model.part("crank_set")
    crank.visual(
        Cylinder(radius=0.018, length=0.330),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="crank_spindle",
    )
    crank.visual(
        Box((0.030, 0.022, 0.180)),
        origin=Origin(xyz=(0.0, 0.165, -0.090)),
        material=metal,
        name="positive_arm",
    )
    crank.visual(
        Box((0.030, 0.022, 0.180)),
        origin=Origin(xyz=(0.0, -0.165, 0.090)),
        material=metal,
        name="negative_arm",
    )
    crank.visual(
        Cylinder(radius=0.010, length=0.090),
        origin=Origin(xyz=(0.0, 0.210, -0.180), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="positive_pedal_axle",
    )
    crank.visual(
        Cylinder(radius=0.010, length=0.090),
        origin=Origin(xyz=(0.0, -0.210, 0.180), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="negative_pedal_axle",
    )
    crank.visual(
        Box((0.095, 0.045, 0.030)),
        origin=Origin(xyz=(0.0, 0.260, -0.180)),
        material=rubber,
        name="positive_pedal",
    )
    crank.visual(
        Box((0.095, 0.045, 0.030)),
        origin=Origin(xyz=(0.0, -0.260, 0.180)),
        material=rubber,
        name="negative_pedal",
    )

    seat_post = model.part("seat_post")
    seat_post.visual(
        Cylinder(radius=0.018, length=0.640),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=metal,
        name="inner_post",
    )
    seat_post.visual(
        Box((0.120, 0.065, 0.040)),
        origin=Origin(xyz=(0.015, 0.0, 0.415)),
        material=dark_gray,
        name="saddle_clamp",
    )
    saddle_shape = (
        cq.Workplane("XY")
        .ellipse(0.135, 0.090)
        .extrude(0.045, both=True)
        .union(cq.Workplane("XY").box(0.220, 0.060, 0.040).translate((-0.085, 0.0, 0.0)))
    )
    seat_post.visual(
        mesh_from_cadquery(saddle_shape, "saddle_pad", tolerance=0.001, angular_tolerance=0.08),
        origin=Origin(xyz=(0.030, 0.0, 0.470)),
        material=vinyl,
        name="saddle_pad",
    )

    model.articulation(
        "frame_to_flywheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=flywheel,
        origin=Origin(xyz=flywheel_center),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=30.0),
    )
    model.articulation(
        "frame_to_crank",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=crank,
        origin=Origin(xyz=bottom_bracket),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=20.0),
    )
    model.articulation(
        "frame_to_seat_post",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=seat_post,
        origin=Origin(xyz=seat_sleeve_top),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.25, lower=0.0, upper=0.16),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("floor_frame")
    flywheel = object_model.get_part("flywheel")
    crank = object_model.get_part("crank_set")
    seat_post = object_model.get_part("seat_post")
    crank_joint = object_model.get_articulation("frame_to_crank")
    flywheel_joint = object_model.get_articulation("frame_to_flywheel")
    seat_joint = object_model.get_articulation("frame_to_seat_post")

    ctx.allow_overlap(
        crank,
        frame,
        elem_a="crank_spindle",
        elem_b="bottom_bracket_shell",
        reason="The crank spindle is captured in the bottom-bracket bearing cup.",
    )
    ctx.allow_overlap(
        flywheel,
        frame,
        elem_a="flywheel_axle",
        elem_b="flywheel_bearing_near",
        reason="The flywheel axle is seated in the bearing ring.",
    )
    ctx.allow_overlap(
        flywheel,
        frame,
        elem_a="flywheel_axle",
        elem_b="flywheel_bearing_far",
        reason="The flywheel axle is seated in the opposite bearing ring.",
    )
    ctx.allow_overlap(
        frame,
        seat_post,
        elem_a="seat_clamp_pad",
        elem_b="inner_post",
        reason="The seat-post clamp pad lightly compresses the sliding post.",
    )

    ctx.expect_overlap(
        crank,
        frame,
        axes="y",
        elem_a="crank_spindle",
        elem_b="bottom_bracket_shell",
        min_overlap=0.10,
        name="crank spindle is retained in bottom bracket",
    )
    ctx.expect_within(
        crank,
        frame,
        axes="xz",
        inner_elem="crank_spindle",
        outer_elem="bottom_bracket_shell",
        margin=0.006,
        name="crank spindle remains centered in bottom bracket",
    )
    for bearing in ("flywheel_bearing_near", "flywheel_bearing_far"):
        ctx.expect_overlap(
            flywheel,
            frame,
            axes="y",
            elem_a="flywheel_axle",
            elem_b=bearing,
            min_overlap=0.012,
            name=f"flywheel axle retained by {bearing}",
        )
        ctx.expect_within(
            flywheel,
            frame,
            axes="xz",
            inner_elem="flywheel_axle",
            outer_elem=bearing,
            margin=0.006,
            name=f"flywheel axle centered in {bearing}",
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
        min_overlap=0.20,
        name="collapsed seat post remains inserted",
    )
    ctx.expect_overlap(
        frame,
        seat_post,
        axes="z",
        elem_a="seat_clamp_pad",
        elem_b="inner_post",
        min_overlap=0.030,
        name="seat clamp pad bears on post",
    )
    rest_pos = ctx.part_world_position(seat_post)
    with ctx.pose({seat_joint: 0.16, crank_joint: math.pi / 2.0, flywheel_joint: math.pi}):
        ctx.expect_overlap(
            seat_post,
            frame,
            axes="z",
            elem_a="inner_post",
            elem_b="seat_sleeve",
            min_overlap=0.055,
            name="raised seat post still retained",
        )
        raised_pos = ctx.part_world_position(seat_post)

    ctx.check(
        "seat post prismatic joint raises saddle",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.14,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    neg = ctx.part_element_world_aabb(frame, elem="front_wheel_negative")
    pos = ctx.part_element_world_aabb(frame, elem="front_wheel_positive")
    stance = None
    if neg is not None and pos is not None:
        stance = pos[1][1] - neg[0][1]
    ctx.check(
        "outboard stabilizer wheels give wide stance",
        stance is not None and stance > 1.25,
        details=f"front wheel outside width={stance}",
    )

    return ctx.report()


object_model = build_object_model()
