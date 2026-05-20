from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)
import cadquery as cq


AXLE_Z = 0.72
SIDE_Y = 0.60


def _annular_bearing(outer_radius: float, inner_radius: float, thickness: float) -> cq.Workplane:
    """A centered annular bearing housing, with its local bore along +Z."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(thickness / 2.0, both=True)
    )


def _add_beam(
    part,
    name: str,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    width: float,
    depth: float,
    material: str,
) -> None:
    """Add a rectangular timber whose long local +Z axis runs from start to end."""
    sx, sy, sz = start
    ex, ey, ez = end
    dx = ex - sx
    dy = ey - sy
    dz = ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if abs(dy) > 1e-6:
        raise ValueError("This helper is for XZ-plane beams at a fixed Y.")
    pitch = math.atan2(dx, dz)
    part.visual(
        Box((width, depth, length)),
        origin=Origin(
            xyz=((sx + ex) / 2.0, (sy + ey) / 2.0, (sz + ez) / 2.0),
            rpy=(0.0, pitch, 0.0),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="undershot_waterwheel")

    model.material("weathered_wood", rgba=(0.45, 0.26, 0.11, 1.0))
    model.material("dark_iron", rgba=(0.07, 0.075, 0.075, 1.0))
    model.material("wet_water", rgba=(0.10, 0.38, 0.75, 0.58))
    model.material("pale_timber", rgba=(0.58, 0.36, 0.16, 1.0))

    frame = model.part("frame")
    wheel = model.part("wheel")

    bearing_mesh = mesh_from_cadquery(_annular_bearing(0.19, 0.085, 0.12), "bearing_housing")

    # A shallow undershot channel beneath the wheel establishes the water path.
    frame.visual(
        Box((1.70, 0.82, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material="weathered_wood",
        name="lower_channel_floor",
    )
    frame.visual(
        Box((1.58, 0.06, 0.11)),
        origin=Origin(xyz=(0.0, 0.37, 0.080)),
        material="weathered_wood",
        name="lower_channel_wall_0",
    )
    frame.visual(
        Box((1.58, 0.06, 0.11)),
        origin=Origin(xyz=(0.0, -0.37, 0.080)),
        material="weathered_wood",
        name="lower_channel_wall_1",
    )
    frame.visual(
        Box((1.52, 0.69, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        material="wet_water",
        name="undershot_water",
    )

    # Base skids and cross ties make the two side frames one rigid, supported assembly.
    for side, y, bearing_name in ((0, -SIDE_Y, "bearing_0"), (1, SIDE_Y, "bearing_1")):
        frame.visual(
            Box((1.72, 0.14, 0.10)),
            origin=Origin(xyz=(0.0, y, 0.065)),
            material="weathered_wood",
            name=f"side_sill_{side}",
        )
        _add_beam(
            frame,
            f"inclined_post_{side}_0",
            (-0.72, y, 0.11),
            (-0.14, y, AXLE_Z - 0.08),
            width=0.11,
            depth=0.13,
            material="weathered_wood",
        )
        _add_beam(
            frame,
            f"inclined_post_{side}_1",
            (0.72, y, 0.11),
            (0.14, y, AXLE_Z - 0.08),
            width=0.11,
            depth=0.13,
            material="weathered_wood",
        )
        frame.visual(
            bearing_mesh,
            origin=Origin(xyz=(0.0, y, AXLE_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material="dark_iron",
            name=bearing_name,
        )
        for bolt, angle in enumerate((math.radians(45), math.radians(135), math.radians(225), math.radians(315))):
            frame.visual(
                Cylinder(radius=0.018, length=0.030),
                origin=Origin(
                    xyz=(
                        0.13 * math.cos(angle),
                        y + (0.070 if y > 0 else -0.070),
                        AXLE_Z + 0.13 * math.sin(angle),
                    ),
                    rpy=(-math.pi / 2.0, 0.0, 0.0),
                ),
                material="dark_iron",
                name=f"bearing_bolt_{side}_{bolt}",
            )
        for x in (-0.70, 0.70):
            frame.visual(
                Box((0.12, 0.12, 1.30)),
                origin=Origin(xyz=(x, y, 0.70)),
                material="weathered_wood",
                name=f"trough_post_{side}_{0 if x < 0 else 1}",
            )

    for i, x in enumerate((-0.80, 0.80)):
        frame.visual(
            Box((0.12, 1.34, 0.12)),
            origin=Origin(xyz=(x, 0.0, 0.085)),
            material="weathered_wood",
            name=f"base_tie_{i}",
        )

    # Simple raised trough edge over the paddle wheel.
    frame.visual(
        Box((1.55, 1.14, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 1.360)),
        material="pale_timber",
        name="trough_bottom",
    )
    frame.visual(
        Box((1.55, 0.08, 0.22)),
        origin=Origin(xyz=(0.0, 0.58, 1.440)),
        material="pale_timber",
        name="trough_wall_0",
    )
    frame.visual(
        Box((1.55, 0.08, 0.22)),
        origin=Origin(xyz=(0.0, -0.58, 1.440)),
        material="pale_timber",
        name="trough_wall_1",
    )
    frame.visual(
        Box((1.45, 1.09, 0.012)),
        origin=Origin(xyz=(-0.02, 0.0, 1.405)),
        material="wet_water",
        name="trough_water",
    )

    # The rotary stage: metal axle and hub with a timber rim and broad undershot paddles.
    wheel.visual(
        Cylinder(radius=0.040, length=1.28),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="dark_iron",
        name="axle",
    )
    wheel.visual(
        Cylinder(radius=0.105, length=0.52),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="dark_iron",
        name="hub",
    )
    for side, y in enumerate((-0.47, 0.47)):
        wheel.visual(
            Cylinder(radius=0.065, length=0.055),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material="dark_iron",
            name=f"axle_collar_{side}",
        )
    for side, y, washer_name in ((0, -0.5225, "thrust_washer_0"), (1, 0.5225, "thrust_washer_1")):
        wheel.visual(
            Cylinder(radius=0.120, length=0.035),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material="dark_iron",
            name=washer_name,
        )

    rim_mesh = mesh_from_geometry(TorusGeometry(0.48, 0.024, radial_segments=16, tubular_segments=64), "wooden_rim")
    inner_rim_mesh = mesh_from_geometry(
        TorusGeometry(0.30, 0.014, radial_segments=12, tubular_segments=48),
        "inner_wooden_rim",
    )
    for side, y in enumerate((-0.24, 0.24)):
        wheel.visual(
            rim_mesh,
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material="weathered_wood",
            name=f"outer_rim_{side}",
        )
        wheel.visual(
            inner_rim_mesh,
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material="weathered_wood",
            name=f"inner_rim_{side}",
        )

    for spoke in range(8):
        theta = spoke * math.tau / 8.0
        pitch = math.pi / 2.0 - theta
        wheel.visual(
            Box((0.038, 0.54, 0.44)),
            origin=Origin(
                xyz=(0.24 * math.cos(theta), 0.0, 0.24 * math.sin(theta)),
                rpy=(0.0, pitch, 0.0),
            ),
            material="weathered_wood",
            name=f"spoke_{spoke}",
        )

    paddle_names = (
        "paddle_0",
        "paddle_1",
        "paddle_2",
        "paddle_3",
        "paddle_4",
        "paddle_5",
        "paddle_6",
        "paddle_7",
        "paddle_8",
        "paddle_9",
        "paddle_10",
        "paddle_11",
    )
    for paddle, paddle_name in enumerate(paddle_names):
        theta = paddle * math.tau / 12.0
        pitch = math.pi / 2.0 - theta
        wheel.visual(
            Box((0.050, 0.56, 0.165)),
            origin=Origin(
                xyz=(0.505 * math.cos(theta), 0.0, 0.505 * math.sin(theta)),
                rpy=(0.0, pitch, 0.0),
            ),
            material="pale_timber",
            name=paddle_name,
        )

    model.articulation(
        "axle_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, AXLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=4.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    wheel = object_model.get_part("wheel")
    spin = object_model.get_articulation("axle_spin")

    ctx.check(
        "wheel uses a continuous rotary axle",
        spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint_type={spin.articulation_type}",
    )
    ctx.check(
        "axle axis runs horizontally through both side frames",
        abs(spin.axis[0]) < 1e-6 and abs(spin.axis[1] - 1.0) < 1e-6 and abs(spin.axis[2]) < 1e-6,
        details=f"axis={spin.axis}",
    )

    ctx.expect_overlap(
        wheel,
        frame,
        axes="y",
        elem_a="axle",
        elem_b="bearing_0",
        min_overlap=0.09,
        name="axle passes through first bearing housing",
    )
    ctx.expect_overlap(
        wheel,
        frame,
        axes="y",
        elem_a="axle",
        elem_b="bearing_1",
        min_overlap=0.09,
        name="axle passes through second bearing housing",
    )
    ctx.expect_within(
        wheel,
        frame,
        axes="xz",
        inner_elem="thrust_washer_0",
        outer_elem="bearing_0",
        margin=0.0,
        name="first thrust washer is centered inside bearing rim",
    )
    ctx.expect_within(
        wheel,
        frame,
        axes="xz",
        inner_elem="thrust_washer_1",
        outer_elem="bearing_1",
        margin=0.0,
        name="second thrust washer is centered inside bearing rim",
    )
    ctx.expect_gap(
        frame,
        wheel,
        axis="z",
        positive_elem="trough_bottom",
        negative_elem="paddle_3",
        min_gap=0.015,
        max_gap=0.060,
        name="upper paddle clears underside of raised trough",
    )

    def _center(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((lower[i] + upper[i]) / 2.0 for i in range(3))

    rest_origin = ctx.part_world_position(wheel)
    rest_paddle = _center(ctx.part_element_world_aabb(wheel, elem="paddle_0"))
    with ctx.pose({spin: 0.80}):
        spun_origin = ctx.part_world_position(wheel)
        spun_paddle = _center(ctx.part_element_world_aabb(wheel, elem="paddle_0"))
    ctx.check(
        "rotary pose spins paddles around a fixed axle",
        rest_origin is not None
        and spun_origin is not None
        and rest_paddle is not None
        and spun_paddle is not None
        and max(abs(rest_origin[i] - spun_origin[i]) for i in range(3)) < 1e-6
        and abs(rest_paddle[0] - spun_paddle[0]) > 0.10
        and abs(rest_paddle[2] - spun_paddle[2]) > 0.20,
        details=f"rest_origin={rest_origin}, spun_origin={spun_origin}, rest_paddle={rest_paddle}, spun_paddle={spun_paddle}",
    )
    return ctx.report()


object_model = build_object_model()
