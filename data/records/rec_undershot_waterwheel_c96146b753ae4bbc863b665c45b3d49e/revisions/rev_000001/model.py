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
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


WHEEL_X = 0.36
AXLE_Z = 0.64
WHEEL_RIM_RADIUS = 0.43
PADDLE_DEPTH = 0.11


def _annular_disk(radius: float, inner_radius: float, width_y: float, y_center: float):
    """CadQuery annulus in the local XZ plane, extruded along +Y and re-centered."""
    return (
        cq.Workplane("XZ")
        .circle(radius)
        .circle(inner_radius)
        .extrude(width_y)
        .translate((0.0, y_center + width_y / 2.0, 0.0))
    )


def _wheel_wood_shape():
    """Connected wooden rim, paired spokes, hub, and undershot paddle boards."""
    rim_thick = 0.045
    ring_width = 0.055
    side_y = 0.18
    hub_radius = 0.085
    hub_width = 0.34
    spoke_length = WHEEL_RIM_RADIUS - rim_thick - hub_radius * 0.55
    spoke_center = hub_radius * 0.55 + spoke_length / 2.0

    shape = _annular_disk(WHEEL_RIM_RADIUS, WHEEL_RIM_RADIUS - rim_thick, ring_width, -side_y)
    shape = shape.union(_annular_disk(WHEEL_RIM_RADIUS, WHEEL_RIM_RADIUS - rim_thick, ring_width, side_y))
    shape = shape.union(_annular_disk(hub_radius, 0.028, hub_width, 0.0))

    for angle in range(0, 360, 45):
        for y_center in (-side_y, side_y):
            spoke = (
                cq.Workplane()
                .box(spoke_length, 0.060, 0.034)
                .translate((spoke_center, y_center, 0.0))
                .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), angle)
            )
            shape = shape.union(spoke)

    paddle_center_radius = WHEEL_RIM_RADIUS + PADDLE_DEPTH / 2.0 - 0.025
    for angle in range(0, 360, 30):
        paddle = (
            cq.Workplane()
            .box(PADDLE_DEPTH, 0.50, 0.040)
            .translate((paddle_center_radius, 0.0, 0.0))
            .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), angle)
        )
        shape = shape.union(paddle)

    return shape


def _bearing_ring_shape(y_center: float):
    return _annular_disk(0.105, 0.065, 0.070, y_center).translate((WHEEL_X, 0.0, AXLE_Z))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="undershot_waterwheel")

    weathered_wood = Material("weathered_wood", rgba=(0.50, 0.31, 0.16, 1.0))
    dark_endgrain = Material("dark_endgrain", rgba=(0.28, 0.17, 0.08, 1.0))
    iron = Material("dark_wrought_iron", rgba=(0.08, 0.08, 0.075, 1.0))
    water = Material("channel_water", rgba=(0.10, 0.38, 0.72, 0.62))

    frame = model.part("frame")

    # Ground skids and cross ties: the fixed base is centered near the root frame,
    # while the wheel/axle is deliberately offset to +X.
    for y in (-0.45, 0.45):
        frame.visual(
            Box((1.45, 0.075, 0.055)),
            origin=Origin(xyz=(0.12, y, 0.0275)),
            material=weathered_wood,
            name=f"base_rail_{0 if y < 0 else 1}",
        )
    for x in (-0.42, -0.23, 0.20, 0.72):
        frame.visual(
            Box((0.075, 0.98, 0.050)),
            origin=Origin(xyz=(x, 0.0, 0.050)),
            material=dark_endgrain,
            name=f"cross_tie_{x:.2f}",
        )

    # A single taller column holds the simple trough; the paddle wheel is not
    # centered on it, satisfying the requested offset rotating member.
    frame.visual(
        Box((0.085, 0.12, 1.135)),
        origin=Origin(xyz=(-0.23, 0.0, 0.6425)),
        material=weathered_wood,
        name="main_column",
    )

    # Two side A-frames cradle the annular bearing blocks just outside the wheel.
    leg_span_x = 0.34
    leg_top_z = AXLE_Z - 0.115
    leg_bottom_z = 0.075
    leg_length = math.sqrt(leg_span_x * leg_span_x + (leg_top_z - leg_bottom_z) ** 2)
    leg_angle = math.atan2(leg_span_x, leg_top_z - leg_bottom_z)
    for y in (-0.45, 0.45):
        for direction in (-1, 1):
            x_bottom = WHEEL_X + direction * leg_span_x
            x_top = WHEEL_X + direction * 0.055
            frame.visual(
                Box((0.060, 0.075, leg_length)),
                origin=Origin(
                    xyz=((x_bottom + x_top) / 2.0, y, (leg_bottom_z + leg_top_z) / 2.0),
                    rpy=(0.0, -direction * leg_angle, 0.0),
                ),
                material=weathered_wood,
                name=f"side_leg_{0 if y < 0 else 1}_{0 if direction < 0 else 1}",
            )
        frame.visual(
            Box((0.31, 0.078, 0.090)),
            origin=Origin(xyz=(WHEEL_X, y, AXLE_Z - 0.125)),
            material=weathered_wood,
            name=f"bearing_sill_{0 if y < 0 else 1}",
        )

    for idx, y in enumerate((-0.45, 0.45)):
        frame.visual(
            mesh_from_cadquery(_bearing_ring_shape(y), f"bearing_ring_{idx}"),
            material=iron,
            name=f"bearing_{idx}",
        )

    # Open wooden trough above the wheel.  It is intentionally simple: bottom
    # board, side boards, a little blue water plane, and a squared discharge lip.
    frame.visual(
        Box((0.92, 0.32, 0.038)),
        origin=Origin(xyz=(-0.10, 0.0, 1.222)),
        material=weathered_wood,
        name="trough_bottom",
    )
    for y in (-0.185, 0.185):
        frame.visual(
            Box((0.92, 0.040, 0.135)),
            origin=Origin(xyz=(-0.10, y, 1.292)),
            material=weathered_wood,
            name=f"trough_side_{0 if y < 0 else 1}",
        )
    frame.visual(
        Box((0.018, 0.34, 0.105)),
        origin=Origin(xyz=(0.36, 0.0, 1.205)),
        material=dark_endgrain,
        name="trough_lip",
    )
    frame.visual(
        Box((0.78, 0.34, 0.012)),
        origin=Origin(xyz=(-0.17, 0.0, 1.286)),
        material=water,
        name="water_surface",
    )

    wheel = model.part("wheel")
    wheel.visual(
        mesh_from_cadquery(_wheel_wood_shape(), "paddle_wheel_wood", tolerance=0.0015),
        material=weathered_wood,
        name="wheel_wood",
    )
    wheel.visual(
        Cylinder(radius=0.040, length=1.02),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="axle",
    )
    wheel.visual(
        Cylinder(radius=0.070, length=0.20),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="hub_band",
    )

    model.articulation(
        "axle_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(WHEEL_X, 0.0, AXLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=3.0),
        motion_properties=MotionProperties(damping=0.02, friction=0.01),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    wheel = object_model.get_part("wheel")
    spin = object_model.get_articulation("axle_spin")

    ctx.check(
        "wheel uses a continuous axle joint",
        spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint_type={spin.articulation_type}",
    )
    ctx.check(
        "axle axis is horizontal across side frames",
        tuple(round(v, 6) for v in spin.axis) == (0.0, 1.0, 0.0),
        details=f"axis={spin.axis}",
    )
    wheel_pos = ctx.part_world_position(wheel)
    frame_pos = ctx.part_world_position(frame)
    ctx.check(
        "rotating member is offset from root support center",
        wheel_pos is not None
        and frame_pos is not None
        and wheel_pos[0] - frame_pos[0] > 0.30,
        details=f"frame={frame_pos}, wheel={wheel_pos}",
    )

    ctx.expect_gap(
        frame,
        wheel,
        axis="z",
        positive_elem="trough_bottom",
        negative_elem="wheel_wood",
        min_gap=0.035,
        name="trough clears the top of the rotating wheel",
    )
    for bearing_name in ("bearing_0", "bearing_1"):
        ctx.expect_within(
            wheel,
            frame,
            axes="xz",
            inner_elem="axle",
            outer_elem=bearing_name,
            margin=0.0,
            name=f"axle centered inside {bearing_name}",
        )
        ctx.expect_overlap(
            wheel,
            frame,
            axes="y",
            elem_a="axle",
            elem_b=bearing_name,
            min_overlap=0.020,
            name=f"axle passes through {bearing_name}",
        )

    return ctx.report()


object_model = build_object_model()
