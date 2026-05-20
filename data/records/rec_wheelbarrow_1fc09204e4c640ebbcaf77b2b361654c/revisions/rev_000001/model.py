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
    TireCarcass,
    TireGeometry,
    TireGroove,
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


def _tube_between(part, name, p0, p1, thickness, material):
    """A chunky rectangular tube whose local X axis spans two points."""
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    yaw = math.atan2(dy, dx)
    pitch = -math.atan2(dz, math.hypot(dx, dy))
    midpoint = (
        (p0[0] + p1[0]) * 0.5,
        (p0[1] + p1[1]) * 0.5,
        (p0[2] + p1[2]) * 0.5,
    )
    part.visual(
        Box((length, thickness, thickness)),
        origin=Origin(xyz=midpoint, rpy=(0.0, pitch, yaw)),
        material=material,
        name=name,
    )


def _make_tray_shell():
    """Open, flared steel wheelbarrow tray with a real hollow basin."""
    outer = (
        cq.Workplane("XY")
        .rect(0.72, 0.42)
        .workplane(offset=0.34)
        .rect(1.10, 0.78)
        .loft(combine=True)
    )
    inner_cut = (
        cq.Workplane("XY")
        .rect(0.56, 0.27)
        .workplane(offset=0.39)
        .rect(0.99, 0.67)
        .loft(combine=True)
        .translate((0.0, 0.0, 0.035))
    )
    return outer.cut(inner_cut)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_wheel_wheelbarrow")

    green = Material("painted_green_steel", color=(0.10, 0.46, 0.22, 1.0))
    dark_steel = Material("dark_powder_coated_steel", color=(0.06, 0.065, 0.06, 1.0))
    black_rubber = Material("black_rubber", color=(0.01, 0.01, 0.01, 1.0))
    rim_yellow = Material("yellow_enamel_rim", color=(0.95, 0.69, 0.06, 1.0))
    axle_gray = Material("galvanized_axle_blocks", color=(0.48, 0.50, 0.48, 1.0))

    body = model.part("body")
    tray_shell = _make_tray_shell()
    body.visual(
        mesh_from_cadquery(tray_shell, "tray_shell", tolerance=0.0012),
        origin=Origin(xyz=(0.0, 0.0, 0.31)),
        material=green,
        name="tray_shell",
    )

    # Rolled lip around the open tray.  These bars visibly overlap the shell rim
    # as a welded/rolled edge and help the tub read as hollow sheet steel.
    body.visual(
        Box((1.16, 0.060, 0.055)),
        origin=Origin(xyz=(0.0, 0.405, 0.665)),
        material=green,
        name="rim_0",
    )
    body.visual(
        Box((1.16, 0.060, 0.055)),
        origin=Origin(xyz=(0.0, -0.405, 0.665)),
        material=green,
        name="rim_1",
    )
    body.visual(
        Box((0.060, 0.84, 0.055)),
        origin=Origin(xyz=(-0.575, 0.0, 0.665)),
        material=green,
        name="front_rim",
    )
    body.visual(
        Box((0.060, 0.84, 0.055)),
        origin=Origin(xyz=(0.575, 0.0, 0.665)),
        material=green,
        name="rear_rim",
    )

    # One-piece chunky undercarriage: two long handles/rails, cross braces,
    # fork tines, and rear resting legs all weld into a single supported frame.
    for sign, suffix in ((1.0, "0"), (-1.0, "1")):
        fork_block_name = "fork_block_0" if suffix == "0" else "fork_block_1"
        rear_foot_name = "rear_foot_0" if suffix == "0" else "rear_foot_1"
        y_rail = sign * 0.285
        _tube_between(
            body,
            f"handle_rail_{suffix}",
            (-0.72, sign * 0.175, 0.225),
            (1.03, y_rail, 0.385),
            0.060,
            dark_steel,
        )
        body.visual(
            Cylinder(radius=0.045, length=0.25),
            origin=Origin(xyz=(1.10, y_rail, 0.392), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black_rubber,
            name=f"grip_{suffix}",
        )
        _tube_between(
            body,
            f"fork_tine_{suffix}",
            (-0.47, sign * 0.210, 0.335),
            (-0.75, sign * 0.150, 0.220),
            0.075,
            dark_steel,
        )
        body.visual(
            Box((0.150, 0.060, 0.255)),
            origin=Origin(xyz=(-0.745, sign * 0.125, 0.235)),
            material=dark_steel,
            name=f"fork_plate_{suffix}",
        )
        body.visual(
            Box((0.145, 0.070, 0.095)),
            origin=Origin(xyz=(-0.735, sign * 0.105, 0.230)),
            material=axle_gray,
            name=fork_block_name,
        )
        _tube_between(
            body,
            f"rear_leg_{suffix}",
            (0.43, sign * 0.260, 0.325),
            (0.65, sign * 0.330, 0.030),
            0.070,
            dark_steel,
        )
        body.visual(
            Box((0.215, 0.090, 0.045)),
            origin=Origin(xyz=(0.685, sign * 0.340, 0.0225)),
            material=dark_steel,
            name=rear_foot_name,
        )

    _tube_between(body, "front_crossbar", (-0.45, -0.32, 0.325), (-0.45, 0.32, 0.325), 0.070, dark_steel)
    _tube_between(body, "rear_crossbar", (0.49, -0.34, 0.315), (0.49, 0.34, 0.315), 0.070, dark_steel)
    _tube_between(body, "leg_crossbrace", (0.58, -0.30, 0.115), (0.58, 0.30, 0.115), 0.045, dark_steel)
    body.visual(
        Cylinder(radius=0.035, length=0.135),
        origin=Origin(xyz=(-0.735, 0.190, 0.230), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=axle_gray,
        name="axle_cap_0",
    )
    body.visual(
        Cylinder(radius=0.035, length=0.135),
        origin=Origin(xyz=(-0.735, -0.190, 0.230), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=axle_gray,
        name="axle_cap_1",
    )

    wheel = model.part("wheel")
    tire = TireGeometry(
        0.220,
        0.100,
        inner_radius=0.155,
        carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.06),
        tread=TireTread(style="block", depth=0.012, count=22, land_ratio=0.54),
        grooves=(TireGroove(center_offset=0.0, width=0.010, depth=0.004),),
        sidewall=TireSidewall(style="square", bulge=0.025),
        shoulder=TireShoulder(width=0.012, radius=0.004),
    )
    wheel.visual(
        mesh_from_geometry(tire, "front_tire"),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=black_rubber,
        name="tire",
    )
    rim = WheelGeometry(
        0.162,
        0.078,
        rim=WheelRim(inner_radius=0.105, flange_height=0.010, flange_thickness=0.006),
        hub=WheelHub(
            radius=0.040,
            width=0.090,
            cap_style="domed",
            bolt_pattern=BoltPattern(count=4, circle_diameter=0.055, hole_diameter=0.007),
        ),
        face=WheelFace(dish_depth=0.010, front_inset=0.004, rear_inset=0.004),
        spokes=WheelSpokes(style="straight", count=6, thickness=0.008, window_radius=0.024),
        bore=WheelBore(style="round", diameter=0.026),
    )
    wheel.visual(
        mesh_from_geometry(rim, "front_rim_mesh"),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=rim_yellow,
        name="rim",
    )
    wheel.visual(
        Cylinder(radius=0.030, length=0.140),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=axle_gray,
        name="hub_sleeve",
    )
    wheel.visual(
        Cylinder(radius=0.052, length=0.020),
        origin=Origin(xyz=(0.0, 0.060, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=axle_gray,
        name="hub_collar_0",
    )
    wheel.visual(
        Cylinder(radius=0.052, length=0.020),
        origin=Origin(xyz=(0.0, -0.060, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=axle_gray,
        name="hub_collar_1",
    )

    model.articulation(
        "body_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=wheel,
        origin=Origin(xyz=(-0.735, 0.0, 0.230)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    wheel = object_model.get_part("wheel")
    spin = object_model.get_articulation("body_to_wheel")

    ctx.check(
        "front wheel uses continuous rotation",
        spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (0.0, 1.0, 0.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )
    ctx.expect_gap(
        body,
        wheel,
        axis="y",
        positive_elem="fork_block_0",
        negative_elem="tire",
        min_gap=0.008,
        max_gap=0.050,
        name="positive axle block clears tire sidewall",
    )
    ctx.expect_gap(
        wheel,
        body,
        axis="y",
        positive_elem="tire",
        negative_elem="fork_block_1",
        min_gap=0.008,
        max_gap=0.050,
        name="negative axle block clears tire sidewall",
    )
    ctx.expect_overlap(
        body,
        wheel,
        axes="xz",
        elem_a="fork_block_0",
        elem_b="rim",
        min_overlap=0.040,
        name="axle blocks bracket the wheel hub projection",
    )

    wheel_aabb = ctx.part_world_aabb(wheel)
    foot_aabb = ctx.part_element_world_aabb(body, elem="rear_foot_0")
    ctx.check(
        "wheel and rear leg feet rest on the same ground plane",
        wheel_aabb is not None
        and foot_aabb is not None
        and abs(wheel_aabb[0][2]) < 0.006
        and abs(foot_aabb[0][2]) < 0.006,
        details=f"wheel_aabb={wheel_aabb}, foot_aabb={foot_aabb}",
    )

    rest_pos = ctx.part_world_position(wheel)
    with ctx.pose({spin: 1.75}):
        spun_pos = ctx.part_world_position(wheel)
    ctx.check(
        "wheel spin keeps axle center fixed",
        rest_pos is not None
        and spun_pos is not None
        and max(abs(a - b) for a, b in zip(rest_pos, spun_pos)) < 1e-6,
        details=f"rest={rest_pos}, spun={spun_pos}",
    )

    return ctx.report()


object_model = build_object_model()
