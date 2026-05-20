from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _annular_cylinder(outer_radius: float, inner_radius: float, length: float) -> cq.Workplane:
    """A centered hollow cylinder whose local axis is +Z."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((0.0, 0.0, -0.5 * length))
    )


def _saddle_shape() -> cq.Workplane:
    """Tapered, rounded top shape for the padded bike saddle."""
    return (
        cq.Workplane("XY")
        .polyline(
            [
                (-0.155, -0.035),
                (-0.120, -0.088),
                (0.055, -0.115),
                (0.150, -0.082),
                (0.168, -0.045),
                (0.168, 0.045),
                (0.150, 0.082),
                (0.055, 0.115),
                (-0.120, 0.088),
                (-0.155, 0.035),
            ]
        )
        .close()
        .extrude(0.045)
        .translate((0.0, 0.0, -0.0225))
        .edges("|Z")
        .fillet(0.012)
    )


def _tube_between(part, name, start, end, radius, material):
    """Add a round tube between two points that share the same Y coordinate."""
    sx, sy, sz = start
    ex, ey, ez = end
    dx = ex - sx
    dy = ey - sy
    dz = ez - sz
    if abs(dy) > 1e-9:
        raise ValueError("_tube_between is intended for side-profile XZ tubes")
    length = math.sqrt(dx * dx + dz * dz)
    pitch = math.atan2(dx, dz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((sx + ex) * 0.5, sy, (sz + ez) * 0.5),
            rpy=(0.0, pitch, 0.0),
        ),
        material=material,
        name=name,
    )


def _cylinder_y(part, name, center, radius, length, material):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _cylinder_x(part, name, center, radius, length, material):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stationary_exercise_bike")

    satin_black = model.material("satin_black", color=(0.015, 0.017, 0.018, 1.0))
    charcoal = model.material("charcoal", color=(0.08, 0.09, 0.095, 1.0))
    graphite = model.material("graphite", color=(0.17, 0.18, 0.18, 1.0))
    dark_rubber = model.material("dark_rubber", color=(0.01, 0.01, 0.012, 1.0))
    brushed_steel = model.material("brushed_steel", color=(0.62, 0.64, 0.62, 1.0))
    flywheel_metal = model.material("flywheel_metal", color=(0.43, 0.46, 0.46, 1.0))
    accent_red = model.material("accent_red", color=(0.75, 0.04, 0.025, 1.0))

    frame = model.part("floor_frame")

    # Low, wide stabilizing feet and a chunky center spine.
    frame.visual(
        Box((0.18, 0.82, 0.07)),
        origin=Origin(xyz=(-0.62, 0.0, 0.035)),
        material=satin_black,
        name="front_foot",
    )
    frame.visual(
        Box((0.20, 0.78, 0.07)),
        origin=Origin(xyz=(0.54, 0.0, 0.035)),
        material=satin_black,
        name="rear_foot",
    )
    frame.visual(
        Box((1.22, 0.13, 0.085)),
        origin=Origin(xyz=(-0.03, 0.0, 0.095)),
        material=satin_black,
        name="floor_spine",
    )
    _cylinder_x(frame, "front_roller", (-0.69, 0.0, 0.030), 0.035, 0.12, dark_rubber)
    _cylinder_x(frame, "rear_pad", (0.62, 0.0, 0.030), 0.032, 0.14, dark_rubber)

    flywheel_center = (-0.48, 0.0, 0.46)
    bottom_bracket = (-0.08, 0.0, 0.43)
    sleeve_center = (0.32, 0.0, 0.50)
    sleeve_top_z = 0.75

    # Stout frame members. Attachment points land on the outside of sleeves and
    # bearing shells so the moving post and crank spindle retain their bores.
    _tube_between(frame, "front_fork_0", (-0.61, -0.18, 0.105), (-0.48, -0.18, 0.46), 0.035, satin_black)
    _tube_between(frame, "front_fork_1", (-0.61, 0.18, 0.105), (-0.48, 0.18, 0.46), 0.035, satin_black)
    frame.visual(
        Box((0.12, 0.13, 0.060)),
        origin=Origin(xyz=(-0.61, -0.18, 0.078)),
        material=satin_black,
        name="fork_root_0",
    )
    frame.visual(
        Box((0.12, 0.13, 0.060)),
        origin=Origin(xyz=(-0.61, 0.18, 0.078)),
        material=satin_black,
        name="fork_root_1",
    )
    _tube_between(frame, "axle_stay_0", (-0.48, -0.155, 0.46), (-0.155, -0.155, 0.485), 0.024, satin_black)
    _tube_between(frame, "axle_stay_1", (-0.48, 0.155, 0.46), (-0.155, 0.155, 0.485), 0.024, satin_black)
    _tube_between(frame, "rear_chainstay", (0.52, 0.0, 0.135), (-0.025, 0.0, 0.370), 0.040, satin_black)
    _tube_between(frame, "seat_support", (0.49, 0.0, 0.125), (0.405, 0.0, 0.425), 0.042, satin_black)
    _tube_between(frame, "top_frame_tube", (0.245, 0.0, 0.685), (-0.015, 0.0, 0.825), 0.038, satin_black)
    _tube_between(frame, "handlebar_mast", (0.045, 0.0, 0.300), (-0.030, 0.0, 1.045), 0.045, satin_black)
    _tube_between(frame, "mast_brace_0", (-0.525, -0.075, 0.170), (-0.150, -0.075, 0.565), 0.030, satin_black)
    _tube_between(frame, "mast_brace_1", (-0.525, 0.075, 0.170), (-0.150, 0.075, 0.565), 0.030, satin_black)

    # Hollow bearing shells and housings.
    frame.visual(
        mesh_from_cadquery(_annular_cylinder(0.073, 0.034, 0.190), "bottom_bracket_shell"),
        origin=Origin(xyz=bottom_bracket, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="bottom_bracket_shell",
    )
    frame.visual(
        mesh_from_cadquery(_annular_cylinder(0.045, 0.029, 0.500), "seat_sleeve"),
        origin=Origin(xyz=sleeve_center),
        material=graphite,
        name="seat_sleeve",
    )
    frame.visual(
        mesh_from_cadquery(_annular_cylinder(0.060, 0.030, 0.070), "seat_collar"),
        origin=Origin(xyz=(0.32, 0.0, sleeve_top_z)),
        material=charcoal,
        name="seat_collar",
    )
    frame.visual(
        mesh_from_cadquery(_annular_cylinder(0.340, 0.260, 0.160), "flywheel_cowling"),
        origin=Origin(xyz=flywheel_center, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="flywheel_cowling",
    )
    for y in (-0.11, 0.11):
        frame.visual(
            mesh_from_cadquery(_annular_cylinder(0.060, 0.026, 0.090), f"flywheel_bearing_{y:+.3f}"),
            origin=Origin(xyz=(flywheel_center[0], y, flywheel_center[2]), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=graphite,
            name=f"flywheel_bearing_{0 if y < 0 else 1}",
        )
        _tube_between(frame, f"housing_web_{0 if y < 0 else 1}", (-0.48, y, 0.505), (-0.48, y, 0.718), 0.015, charcoal)
        _tube_between(frame, f"lower_web_{0 if y < 0 else 1}", (-0.48, y, 0.415), (-0.48, y, 0.205), 0.015, charcoal)

    # Fixed handlebar assembly with rubber grips.
    _tube_between(frame, "bar_stem", (-0.030, 0.0, 1.045), (-0.205, 0.0, 1.115), 0.030, satin_black)
    _cylinder_y(frame, "handlebar_crossbar", (-0.215, 0.0, 1.115), 0.028, 0.620, satin_black)
    _cylinder_y(frame, "grip_0", (-0.215, -0.330, 1.115), 0.033, 0.120, dark_rubber)
    _cylinder_y(frame, "grip_1", (-0.215, 0.330, 1.115), 0.033, 0.120, dark_rubber)
    frame.visual(
        Box((0.18, 0.060, 0.090)),
        origin=Origin(xyz=(-0.105, 0.0, 1.045), rpy=(0.0, -0.40, 0.0)),
        material=charcoal,
        name="small_console",
    )

    flywheel = model.part("flywheel")
    _cylinder_y(flywheel, "flywheel_disk", (0.0, 0.0, 0.0), 0.235, 0.065, flywheel_metal)
    _cylinder_y(flywheel, "flywheel_rim", (0.0, 0.0, 0.0), 0.248, 0.020, brushed_steel)
    flywheel.visual(
        Cylinder(radius=0.026, length=0.220),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="flywheel_axle",
    )
    _cylinder_y(flywheel, "flywheel_hub", (0.0, 0.0, 0.0), 0.065, 0.090, brushed_steel)
    flywheel.visual(
        Box((0.044, 0.008, 0.035)),
        origin=Origin(xyz=(0.150, -0.032, 0.120)),
        material=accent_red,
        name="rim_marker",
    )

    crank = model.part("crank")
    crank.visual(
        Cylinder(radius=0.0345, length=0.235),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="crank_spindle",
    )
    crank.visual(
        Box((0.034, 0.024, 0.210)),
        origin=Origin(xyz=(0.0, -0.120, -0.105)),
        material=brushed_steel,
        name="crank_arm_0",
    )
    crank.visual(
        Box((0.034, 0.024, 0.210)),
        origin=Origin(xyz=(0.0, 0.120, 0.105)),
        material=brushed_steel,
        name="crank_arm_1",
    )
    crank.visual(
        Box((0.155, 0.060, 0.035)),
        origin=Origin(xyz=(0.0, -0.150, -0.220)),
        material=dark_rubber,
        name="pedal_0",
    )
    crank.visual(
        Box((0.155, 0.060, 0.035)),
        origin=Origin(xyz=(0.0, 0.150, 0.220)),
        material=dark_rubber,
        name="pedal_1",
    )

    seat_post = model.part("seat_post")
    seat_post.visual(
        Cylinder(radius=0.029, length=0.620),
        origin=Origin(xyz=(0.0, 0.0, -0.060)),
        material=brushed_steel,
        name="inner_post",
    )
    seat_post.visual(
        Box((0.135, 0.075, 0.050)),
        origin=Origin(xyz=(0.030, 0.0, 0.252)),
        material=graphite,
        name="saddle_clamp",
    )
    seat_post.visual(
        mesh_from_cadquery(_saddle_shape(), "saddle_pad"),
        origin=Origin(xyz=(0.075, 0.0, 0.298)),
        material=dark_rubber,
        name="saddle_pad",
    )

    model.articulation(
        "flywheel_axle",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=flywheel,
        origin=Origin(xyz=flywheel_center),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=30.0),
    )
    model.articulation(
        "crank_axis",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=crank,
        origin=Origin(xyz=bottom_bracket),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=20.0),
    )
    model.articulation(
        "seat_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=seat_post,
        origin=Origin(xyz=(0.32, 0.0, sleeve_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.180, effort=120.0, velocity=0.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("floor_frame")
    flywheel = object_model.get_part("flywheel")
    crank = object_model.get_part("crank")
    seat_post = object_model.get_part("seat_post")
    flywheel_joint = object_model.get_articulation("flywheel_axle")
    crank_joint = object_model.get_articulation("crank_axis")
    seat_slide = object_model.get_articulation("seat_slide")

    ctx.allow_overlap(
        frame,
        seat_post,
        elem_a="seat_sleeve",
        elem_b="inner_post",
        reason=(
            "The adjustable post is intentionally represented as a close sliding "
            "fit inside the sleeve/collar proxy while remaining retained."
        ),
    )
    ctx.allow_overlap(
        crank,
        frame,
        elem_a="crank_spindle",
        elem_b="bottom_bracket_shell",
        reason=(
            "The crank spindle is intentionally captured in the bottom-bracket "
            "bearing shell proxy to show a supported rotating axle."
        ),
    )
    for bearing_name in ("flywheel_bearing_0", "flywheel_bearing_1"):
        ctx.allow_overlap(
            frame,
            flywheel,
            elem_a=bearing_name,
            elem_b="flywheel_axle",
            reason=(
                "The flywheel axle is intentionally captured by the bearing block "
                "proxy so the heavy flywheel reads as supported on the frame fork."
            ),
        )

    ctx.check(
        "flywheel is continuous",
        flywheel_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={flywheel_joint.articulation_type}",
    )
    ctx.check(
        "crank is continuous",
        crank_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={crank_joint.articulation_type}",
    )
    ctx.check(
        "seat post is prismatic",
        seat_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={seat_slide.articulation_type}",
    )
    ctx.expect_within(
        seat_post,
        frame,
        axes="xy",
        inner_elem="inner_post",
        outer_elem="seat_sleeve",
        margin=0.002,
        name="seat post centered in sleeve",
    )
    ctx.expect_overlap(
        seat_post,
        frame,
        axes="z",
        elem_a="inner_post",
        elem_b="seat_sleeve",
        min_overlap=0.30,
        name="seat post retained at low height",
    )
    ctx.expect_within(
        crank,
        frame,
        axes="xz",
        inner_elem="crank_spindle",
        outer_elem="bottom_bracket_shell",
        margin=0.001,
        name="crank spindle sits in bearing shell",
    )
    ctx.expect_overlap(
        crank,
        frame,
        axes="y",
        elem_a="crank_spindle",
        elem_b="bottom_bracket_shell",
        min_overlap=0.18,
        name="crank spindle spans bottom bracket",
    )
    for bearing_name in ("flywheel_bearing_0", "flywheel_bearing_1"):
        ctx.expect_within(
            flywheel,
            frame,
            axes="xz",
            inner_elem="flywheel_axle",
            outer_elem=bearing_name,
            margin=0.002,
            name=f"flywheel axle centered in {bearing_name}",
        )
        ctx.expect_overlap(
            flywheel,
            frame,
            axes="y",
            elem_a="flywheel_axle",
            elem_b=bearing_name,
            min_overlap=0.045,
            name=f"flywheel axle retained in {bearing_name}",
        )

    rest_seat = ctx.part_world_position(seat_post)
    rest_pedal = ctx.part_element_world_aabb(crank, elem="pedal_0")
    rest_marker = ctx.part_element_world_aabb(flywheel, elem="rim_marker")

    with ctx.pose({seat_slide: 0.180}):
        ctx.expect_within(
            seat_post,
            frame,
            axes="xy",
            inner_elem="inner_post",
            outer_elem="seat_sleeve",
            margin=0.002,
            name="raised seat stays aligned",
        )
        ctx.expect_overlap(
            seat_post,
            frame,
            axes="z",
            elem_a="inner_post",
            elem_b="seat_sleeve",
            min_overlap=0.18,
            name="raised seat remains inserted",
        )
        raised_seat = ctx.part_world_position(seat_post)

    with ctx.pose({crank_joint: math.pi / 2.0}):
        turned_pedal = ctx.part_element_world_aabb(crank, elem="pedal_0")

    with ctx.pose({flywheel_joint: math.pi / 2.0}):
        turned_marker = ctx.part_element_world_aabb(flywheel, elem="rim_marker")

    ctx.check(
        "seat raises along column",
        rest_seat is not None and raised_seat is not None and raised_seat[2] > rest_seat[2] + 0.17,
        details=f"rest={rest_seat}, raised={raised_seat}",
    )
    ctx.check(
        "crank rotation moves pedal",
        rest_pedal is not None
        and turned_pedal is not None
        and abs(turned_pedal[0][2] - rest_pedal[0][2]) > 0.10,
        details=f"rest={rest_pedal}, turned={turned_pedal}",
    )
    ctx.check(
        "flywheel rotation moves marker",
        rest_marker is not None
        and turned_marker is not None
        and abs(turned_marker[0][2] - rest_marker[0][2]) > 0.08,
        details=f"rest={rest_marker}, turned={turned_marker}",
    )

    return ctx.report()


object_model = build_object_model()
