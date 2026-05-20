from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _bar_between(part, name, start, end, thickness, material):
    """Add a rectangular tube-like bar whose local X axis spans start->end."""
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    yaw = math.atan2(dy, dx)
    horiz = math.sqrt(dx * dx + dy * dy)
    pitch = math.atan2(-dz, horiz)
    part.visual(
        Box((length, thickness, thickness)),
        origin=Origin(
            xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def _tray_shell_geometry():
    """Open, sloped sheet-metal tray with a real cavity rather than a solid block."""
    bottom_len = 0.74
    bottom_width = 0.34
    top_len = 1.12
    top_width = 0.64
    depth = 0.32
    wall = 0.030

    outer = (
        cq.Workplane("XY")
        .rect(bottom_len, bottom_width)
        .workplane(offset=depth)
        .rect(top_len, top_width)
        .loft(combine=True)
    )
    inner_cutter = (
        cq.Workplane("XY")
        .workplane(offset=wall)
        .rect(bottom_len - 2.0 * wall, bottom_width - 2.0 * wall)
        .workplane(offset=depth - wall + 0.040)
        .rect(top_len - 2.0 * wall, top_width - 2.0 * wall)
        .loft(combine=True)
    )
    return outer.cut(inner_cutter)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_wheel_wheelbarrow")

    tray_green = model.material("painted_green_steel", rgba=(0.10, 0.42, 0.18, 1.0))
    tube_black = model.material("black_tube_steel", rgba=(0.02, 0.02, 0.018, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    wheel_metal = model.material("galvanized_wheel", rgba=(0.72, 0.72, 0.68, 1.0))

    chassis = model.part("chassis")

    chassis.visual(
        mesh_from_cadquery(_tray_shell_geometry(), "hollow_tray", tolerance=0.002),
        origin=Origin(xyz=(0.04, 0.0, 0.34)),
        material=tray_green,
        name="tray_shell",
    )

    # Rolled-looking top rim, slightly proud of the tray shell.
    chassis.visual(
        Box((1.16, 0.030, 0.030)),
        origin=Origin(xyz=(0.04, 0.335, 0.665)),
        material=tray_green,
        name="side_rim_0",
    )
    chassis.visual(
        Box((1.16, 0.030, 0.030)),
        origin=Origin(xyz=(0.04, -0.335, 0.665)),
        material=tray_green,
        name="side_rim_1",
    )
    chassis.visual(
        Box((0.030, 0.66, 0.030)),
        origin=Origin(xyz=(-0.54, 0.0, 0.665)),
        material=tray_green,
        name="front_rim",
    )
    chassis.visual(
        Box((0.030, 0.66, 0.030)),
        origin=Origin(xyz=(0.62, 0.0, 0.665)),
        material=tray_green,
        name="rear_rim",
    )

    # Two long handle/support rails that run beside the tray and continue rearward.
    _bar_between(chassis, "support_rail_0", (-0.58, 0.315, 0.38), (1.05, 0.355, 0.62), 0.038, tube_black)
    _bar_between(chassis, "support_rail_1", (-0.58, -0.315, 0.38), (1.05, -0.355, 0.62), 0.038, tube_black)
    chassis.visual(
        Box((0.040, 0.040, 0.215)),
        origin=Origin(xyz=(0.00, 0.325, 0.565)),
        material=tube_black,
        name="tray_mount_0",
    )
    chassis.visual(
        Box((0.040, 0.040, 0.215)),
        origin=Origin(xyz=(0.00, -0.325, 0.565)),
        material=tube_black,
        name="tray_mount_1",
    )
    chassis.visual(
        Cylinder(radius=0.030, length=0.20),
        origin=Origin(xyz=(1.12, 0.357, 0.63), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="grip_0",
    )
    chassis.visual(
        Cylinder(radius=0.030, length=0.20),
        origin=Origin(xyz=(1.12, -0.357, 0.63), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="grip_1",
    )

    # Rear resting legs with small feet.
    _bar_between(chassis, "resting_leg_0", (0.34, 0.335, 0.51), (0.62, 0.285, 0.075), 0.040, tube_black)
    _bar_between(chassis, "resting_leg_1", (0.34, -0.335, 0.51), (0.62, -0.285, 0.075), 0.040, tube_black)
    chassis.visual(
        Box((0.18, 0.065, 0.050)),
        origin=Origin(xyz=(0.62, 0.285, 0.055), rpy=(0.0, 0.0, 0.05)),
        material=tube_black,
        name="rear_foot_0",
    )
    chassis.visual(
        Box((0.18, 0.065, 0.050)),
        origin=Origin(xyz=(0.62, -0.285, 0.055), rpy=(0.0, 0.0, -0.05)),
        material=tube_black,
        name="rear_foot_1",
    )

    # The requested fixed brace member tying the two main supports together near the base.
    chassis.visual(
        Box((0.055, 0.63, 0.040)),
        origin=Origin(xyz=(0.52, 0.0, 0.225)),
        material=tube_black,
        name="base_brace",
    )

    # Front fork and yoke around the single wheel.
    _bar_between(chassis, "fork_stay_0", (-0.58, 0.315, 0.38), (-0.66, 0.095, 0.215), 0.036, tube_black)
    _bar_between(chassis, "fork_stay_1", (-0.58, -0.315, 0.38), (-0.66, -0.095, 0.215), 0.036, tube_black)
    chassis.visual(
        Box((0.16, 0.022, 0.25)),
        origin=Origin(xyz=(-0.66, 0.095, 0.225)),
        material=tube_black,
        name="fork_plate_0",
    )
    chassis.visual(
        Box((0.16, 0.022, 0.25)),
        origin=Origin(xyz=(-0.66, -0.095, 0.225)),
        material=tube_black,
        name="fork_plate_1",
    )
    chassis.visual(
        Cylinder(radius=0.028, length=0.034),
        origin=Origin(xyz=(-0.66, 0.122, 0.205), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wheel_metal,
        name="axle_boss_0",
    )
    chassis.visual(
        Cylinder(radius=0.028, length=0.034),
        origin=Origin(xyz=(-0.66, -0.122, 0.205), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wheel_metal,
        name="axle_boss_1",
    )
    chassis.visual(
        Cylinder(radius=0.0115, length=0.210),
        origin=Origin(xyz=(-0.66, 0.0, 0.205), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wheel_metal,
        name="axle_pin",
    )

    wheel = model.part("wheel")
    wheel.visual(
        mesh_from_geometry(
            TireGeometry(
                0.180,
                0.086,
                inner_radius=0.124,
                tread=TireTread(style="block", depth=0.009, count=18, land_ratio=0.56),
                sidewall=TireSidewall(style="square", bulge=0.025),
                shoulder=TireShoulder(width=0.010, radius=0.004),
            ),
            "front_tire",
        ),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=rubber,
        name="tire",
    )
    wheel.visual(
        mesh_from_geometry(
            WheelGeometry(
                0.135,
                0.096,
                rim=WheelRim(inner_radius=0.070, flange_height=0.010, flange_thickness=0.006),
                hub=WheelHub(
                    radius=0.045,
                    width=0.104,
                    cap_style="domed",
                    bolt_pattern=BoltPattern(count=4, circle_diameter=0.052, hole_diameter=0.006),
                ),
                face=WheelFace(dish_depth=0.010, front_inset=0.004, rear_inset=0.004),
                spokes=WheelSpokes(style="straight", count=6, thickness=0.006, window_radius=0.014),
                bore=WheelBore(style="round", diameter=0.018),
            ),
            "front_wheel_rim",
        ),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=wheel_metal,
        name="rim",
    )

    model.articulation(
        "wheel_axle",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=wheel,
        origin=Origin(xyz=(-0.66, 0.0, 0.205)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=25.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    chassis = object_model.get_part("chassis")
    wheel = object_model.get_part("wheel")
    axle = object_model.get_articulation("wheel_axle")

    ctx.allow_overlap(
        chassis,
        wheel,
        elem_a="axle_pin",
        elem_b="rim",
        reason="The fixed axle pin is intentionally captured through the wheel hub bore so the wheel is supported while rotating.",
    )
    ctx.expect_overlap(
        chassis,
        wheel,
        axes="y",
        elem_a="axle_pin",
        elem_b="rim",
        min_overlap=0.080,
        name="axle pin passes through the wheel hub",
    )
    ctx.expect_within(
        chassis,
        wheel,
        axes="xz",
        inner_elem="axle_pin",
        outer_elem="rim",
        margin=0.002,
        name="axle pin is centered in the wheel hub bore",
    )

    ctx.check(
        "front wheel uses a continuous axle joint",
        axle.articulation_type == ArticulationType.CONTINUOUS and tuple(axle.axis) == (0.0, 1.0, 0.0),
        details=f"type={axle.articulation_type}, axis={axle.axis}",
    )
    ctx.expect_gap(
        chassis,
        wheel,
        axis="y",
        positive_elem="fork_plate_0",
        negative_elem="rim",
        min_gap=0.002,
        name="wheel has side clearance to one fork plate",
    )
    ctx.expect_gap(
        wheel,
        chassis,
        axis="y",
        positive_elem="rim",
        negative_elem="fork_plate_1",
        min_gap=0.002,
        name="wheel has side clearance to the opposite fork plate",
    )
    ctx.expect_overlap(
        chassis,
        wheel,
        axes="xz",
        elem_a="fork_plate_0",
        elem_b="tire",
        min_overlap=0.05,
        name="fork plates straddle the wheel profile",
    )

    brace_aabb = ctx.part_element_world_aabb(chassis, elem="base_brace")
    ctx.check(
        "base brace spans between the main supports near the base",
        brace_aabb is not None
        and brace_aabb[0][1] < -0.28
        and brace_aabb[1][1] > 0.28
        and 0.16 < brace_aabb[0][2] < 0.26,
        details=f"base_brace_aabb={brace_aabb}",
    )

    rest_pos = ctx.part_world_position(wheel)
    with ctx.pose({axle: math.pi / 2.0}):
        turned_pos = ctx.part_world_position(wheel)
        ctx.expect_gap(
            chassis,
            wheel,
            axis="y",
            positive_elem="fork_plate_0",
            negative_elem="rim",
            min_gap=0.002,
            name="rotating wheel remains inside fork clearance",
        )
    ctx.check(
        "wheel spins in place about the axle",
        rest_pos is not None and turned_pos is not None and max(abs(rest_pos[i] - turned_pos[i]) for i in range(3)) < 1e-6,
        details=f"rest={rest_pos}, spun={turned_pos}",
    )

    return ctx.report()


object_model = build_object_model()
