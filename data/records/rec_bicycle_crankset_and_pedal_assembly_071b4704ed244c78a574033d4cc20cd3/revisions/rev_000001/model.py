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


def _annular_tube_x(outer_radius: float, inner_radius: float, length: float) -> cq.Workplane:
    """A bottom-bracket tube centered on the X axis."""
    return (
        cq.Workplane("YZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((-length / 2.0, 0.0, 0.0))
    )


def _toothed_chainring(
    *,
    tooth_count: int = 42,
    tooth_radius: float = 0.108,
    root_radius: float = 0.101,
    inner_radius: float = 0.066,
    thickness: float = 0.004,
) -> cq.Workplane:
    """Flat narrow-wide style chainring in the local XY plane, thickness on Z."""
    pts = []
    for i in range(tooth_count * 2):
        a = (2.0 * math.pi * i) / (tooth_count * 2)
        # Alternating tooth tips/root valleys give a legible single-ring profile.
        r = tooth_radius if i % 2 == 0 else root_radius
        pts.append((r * math.cos(a), r * math.sin(a)))

    ring = (
        cq.Workplane("XY")
        .polyline(pts)
        .close()
        .extrude(thickness)
        .translate((0.0, 0.0, -thickness / 2.0))
    )
    center_cut = (
        cq.Workplane("XY")
        .circle(inner_radius)
        .extrude(thickness * 3.0)
        .translate((0.0, 0.0, -1.5 * thickness))
    )
    ring = ring.cut(center_cut)

    # Five bolt holes on the spider bolt circle.
    bolt_pts = []
    for k in range(5):
        a = math.radians(18.0) + 2.0 * math.pi * k / 5.0
        bolt_pts.append((0.077 * math.cos(a), 0.077 * math.sin(a)))
    bolt_cuts = (
        cq.Workplane("XY")
        .pushPoints(bolt_pts)
        .circle(0.0036)
        .extrude(thickness * 3.0)
        .translate((0.0, 0.0, -1.5 * thickness))
    )
    return ring.cut(bolt_cuts)


def _spider(thickness: float = 0.008) -> cq.Workplane:
    """Five-arm power-meter spider with a hub and wide strain-gauge spokes."""
    spider = (
        cq.Workplane("XY")
        .circle(0.031)
        .circle(0.0105)
        .extrude(thickness)
        .translate((0.0, 0.0, -thickness / 2.0))
    )
    for k in range(5):
        a = math.radians(18.0) + 2.0 * math.pi * k / 5.0
        radial = (math.cos(a), math.sin(a))
        tangent = (-math.sin(a), math.cos(a))
        r0, r1 = 0.022, 0.083
        w0, w1 = 0.008, 0.006
        pts = [
            (radial[0] * r0 + tangent[0] * w0, radial[1] * r0 + tangent[1] * w0),
            (radial[0] * r1 + tangent[0] * w1, radial[1] * r1 + tangent[1] * w1),
            (radial[0] * r1 - tangent[0] * w1, radial[1] * r1 - tangent[1] * w1),
            (radial[0] * r0 - tangent[0] * w0, radial[1] * r0 - tangent[1] * w0),
        ]
        arm = (
            cq.Workplane("XY")
            .polyline(pts)
            .close()
            .extrude(thickness)
            .translate((0.0, 0.0, -thickness / 2.0))
        )
        spider = spider.union(arm)
    return spider


def _crank_arm(x_center: float, z_end: float, thickness: float = 0.024) -> cq.Workplane:
    """Tapered hollow-forged crank arm in the YZ plane, extruded along X."""
    sign = 1.0 if z_end >= 0.0 else -1.0
    z_root = sign * 0.020
    root_half_width = 0.018
    end_half_width = 0.013
    body_pts = [
        (-root_half_width, z_root),
        (root_half_width, z_root),
        (end_half_width, z_end),
        (-end_half_width, z_end),
    ]
    body = cq.Workplane("YZ").polyline(body_pts).close().extrude(thickness)

    root_boss = (
        cq.Workplane("YZ")
        .circle(0.030)
        .circle(0.0102)
        .extrude(thickness)
    )
    pedal_boss = (
        cq.Workplane("YZ")
        .center(0.0, z_end)
        .circle(0.018)
        .circle(0.0075)
        .extrude(thickness)
    )

    arm = body.union(root_boss).union(pedal_boss)

    root_cut = (
        cq.Workplane("YZ")
        .circle(0.0102)
        .extrude(thickness * 3.0)
        .translate((-thickness, 0.0, 0.0))
    )
    pedal_cut = (
        cq.Workplane("YZ")
        .center(0.0, z_end)
        .circle(0.0075)
        .extrude(thickness * 3.0)
        .translate((-thickness, 0.0, 0.0))
    )
    arm = arm.cut(root_cut).cut(pedal_cut)

    # Long through-window leaves raised forged rails instead of a solid bar.
    slot = (
        cq.Workplane("YZ")
        .center(0.0, sign * 0.096)
        .rect(0.010, 0.080)
        .extrude(thickness * 3.0)
        .translate((-thickness, 0.0, 0.0))
    )
    arm = arm.cut(slot)
    return arm.translate((x_center - thickness / 2.0, 0.0, 0.0))


def _pedal_body(thickness: float = 0.028) -> cq.Workplane:
    """Compact clipless pedal body with an open spindle window and binding jaws."""
    outer_pts = [
        (-0.036, -0.050),
        (0.036, -0.050),
        (0.044, -0.040),
        (0.044, 0.040),
        (0.036, 0.050),
        (-0.036, 0.050),
        (-0.044, 0.040),
        (-0.044, -0.040),
    ]
    body = (
        cq.Workplane("XY")
        .polyline(outer_pts)
        .close()
        .extrude(thickness)
        .translate((0.0, 0.0, -thickness / 2.0))
    )
    # Raised front hook and rear spring bar that make the part read as clipless.
    front_jaw = cq.Workplane("XY").box(0.058, 0.008, 0.010).translate((0.0, 0.034, 0.010))
    rear_jaw = cq.Workplane("XY").box(0.058, 0.010, 0.009).translate((0.0, -0.034, 0.009))
    side_plate_0 = cq.Workplane("XY").box(0.007, 0.062, 0.008).translate((0.036, 0.0, 0.010))
    side_plate_1 = cq.Workplane("XY").box(0.007, 0.062, 0.008).translate((-0.036, 0.0, 0.010))
    return (
        body.union(front_jaw)
        .union(rear_jaw)
        .union(side_plate_0)
        .union(side_plate_1)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="power_meter_crankset")

    shell_mat = model.material("dark_anodized_shell", rgba=(0.05, 0.055, 0.060, 1.0))
    arm_mat = model.material("matte_forged_black", rgba=(0.005, 0.006, 0.007, 1.0))
    ring_mat = model.material("black_chainring", rgba=(0.015, 0.014, 0.012, 1.0))
    metal_mat = model.material("brushed_steel", rgba=(0.68, 0.67, 0.62, 1.0))
    gauge_mat = model.material("amber_gauge_pads", rgba=(0.86, 0.42, 0.10, 1.0))
    pedal_mat = model.material("pedal_black", rgba=(0.02, 0.022, 0.024, 1.0))

    bb_shell = model.part("bb_shell")
    bb_shell.visual(
        mesh_from_cadquery(_annular_tube_x(0.026, 0.016, 0.076), "bb_shell_tube"),
        material=shell_mat,
        name="hollow_shell",
    )
    # Thin silver bearing lips at the shell faces emphasize that the spindle runs
    # through a real hollow bottom bracket rather than through a solid cylinder.
    for idx, x in enumerate((-0.040, 0.040)):
        bb_shell.visual(
            mesh_from_cadquery(_annular_tube_x(0.024, 0.0108, 0.004), f"bearing_lip_{idx}"),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=metal_mat,
            name=f"bearing_lip_{idx}",
        )

    crank = model.part("crank_assembly")
    crank.visual(
        Cylinder(radius=0.011, length=0.192),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_mat,
        name="spindle",
    )
    crank.visual(
        mesh_from_cadquery(_toothed_chainring(), "toothed_chainring"),
        origin=Origin(xyz=(0.054, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=ring_mat,
        name="single_chainring",
    )
    crank.visual(
        mesh_from_cadquery(_spider(), "strain_gauge_spider"),
        origin=Origin(xyz=(0.050, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=arm_mat,
        name="spider",
    )
    crank.visual(
        mesh_from_cadquery(_crank_arm(0.088, -0.175), "drive_crank_arm"),
        material=arm_mat,
        name="arm_0",
    )
    crank.visual(
        mesh_from_cadquery(_crank_arm(-0.088, 0.175), "opposite_crank_arm"),
        material=arm_mat,
        name="arm_1",
    )

    # Five sensor/bridge pads and chainring bolts on the spider face.
    for k in range(5):
        theta = math.radians(18.0) + 2.0 * math.pi * k / 5.0
        y = 0.052 * math.sin(theta)
        z = -0.052 * math.cos(theta)
        crank.visual(
            Box((0.003, 0.007, 0.026)),
            origin=Origin(
                xyz=(0.0528, y, z),
                rpy=(theta - math.pi, 0.0, 0.0),
            ),
            material=gauge_mat,
            name=f"gauge_pad_{k}",
        )
        bolt_y = 0.077 * math.sin(theta)
        bolt_z = -0.077 * math.cos(theta)
        crank.visual(
            Cylinder(radius=0.0042, length=0.004),
            origin=Origin(
                xyz=(0.0560, bolt_y, bolt_z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=metal_mat,
            name=f"chainring_bolt_{k}",
        )

    model.articulation(
        "bb_to_crank",
        ArticulationType.CONTINUOUS,
        parent=bb_shell,
        child=crank,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=12.0),
    )

    # Two clipless pedals: each has a continuously rotating spindle carrier and
    # a bounded float/tilt joint through the pedal body's long fore-aft axis.
    pedal_specs = (
        ("0", 1.0, (0.088, 0.0, -0.175)),
        ("1", -1.0, (-0.088, 0.0, 0.175)),
    )
    for suffix, side, joint_xyz in pedal_specs:
        axle = model.part(f"pedal_axle_{suffix}")
        axle.visual(
            Cylinder(radius=0.006, length=0.100),
            origin=Origin(xyz=(side * 0.050, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal_mat,
            name="spindle",
        )
        axle.visual(
            Cylinder(radius=0.013, length=0.026),
            origin=Origin(xyz=(side * 0.105, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal_mat,
            name="bearing_barrel",
        )
        axle.visual(
            Cylinder(radius=0.010, length=0.010),
            origin=Origin(xyz=(side * 0.0168, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal_mat,
            name="threaded_shoulder",
        )
        model.articulation(
            f"crank_to_axle_{suffix}",
            ArticulationType.CONTINUOUS,
            parent=crank,
            child=axle,
            origin=Origin(xyz=joint_xyz),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=6.0, velocity=20.0),
        )

        pedal = model.part(f"pedal_body_{suffix}")
        pedal.visual(
            mesh_from_cadquery(_pedal_body(), f"clipless_pedal_body_{suffix}"),
            material=pedal_mat,
            name="clipless_body",
        )
        pedal.visual(
            Cylinder(radius=0.003, length=0.058),
            origin=Origin(xyz=(0.0, 0.034, 0.016), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal_mat,
            name="front_hook_pin",
        )
        pedal.visual(
            Cylinder(radius=0.003, length=0.058),
            origin=Origin(xyz=(0.0, -0.035, 0.015), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal_mat,
            name="rear_spring_pin",
        )
        model.articulation(
            f"axle_to_pedal_{suffix}",
            ArticulationType.REVOLUTE,
            parent=axle,
            child=pedal,
            origin=Origin(xyz=(side * 0.105, 0.0, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=-0.18, upper=0.18),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    crank_joint = object_model.get_articulation("bb_to_crank")
    axle_joints = [
        object_model.get_articulation("crank_to_axle_0"),
        object_model.get_articulation("crank_to_axle_1"),
    ]
    float_joints = [
        object_model.get_articulation("axle_to_pedal_0"),
        object_model.get_articulation("axle_to_pedal_1"),
    ]

    ctx.check(
        "crankset has continuous bottom-bracket rotation",
        crank_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(crank_joint.axis) == (1.0, 0.0, 0.0),
        details=f"type={crank_joint.articulation_type}, axis={crank_joint.axis}",
    )
    ctx.check(
        "both pedal axles rotate continuously",
        all(j.articulation_type == ArticulationType.CONTINUOUS for j in axle_joints),
        details=", ".join(f"{j.name}:{j.articulation_type}" for j in axle_joints),
    )
    ctx.check(
        "both pedals have bounded float tilt",
        all(
            j.articulation_type == ArticulationType.REVOLUTE
            and j.motion_limits is not None
            and j.motion_limits.lower <= -0.15
            and j.motion_limits.upper >= 0.15
            for j in float_joints
        ),
        details=", ".join(f"{j.name}:{j.motion_limits}" for j in float_joints),
    )

    # The pedal bearing barrel is intentionally represented as seated inside the
    # compact clipless pedal body, just as a real pedal body carries bearings
    # around the spindle.  The overlap is element-scoped and paired with exact
    # containment/retention checks.
    for suffix in ("0", "1"):
        axle = object_model.get_part(f"pedal_axle_{suffix}")
        pedal = object_model.get_part(f"pedal_body_{suffix}")
        ctx.allow_overlap(
            axle,
            pedal,
            elem_a="bearing_barrel",
            elem_b="clipless_body",
            reason="The pedal bearing barrel is intentionally captured inside the clipless pedal body's bearing pocket.",
        )
        ctx.allow_overlap(
            axle,
            pedal,
            elem_a="spindle",
            elem_b="clipless_body",
            reason="The pedal spindle is intentionally modeled as entering the compact pedal body before the bearing pocket.",
        )
        ctx.expect_within(
            axle,
            pedal,
            axes="yz",
            inner_elem="bearing_barrel",
            outer_elem="clipless_body",
            margin=0.001,
            name=f"pedal_{suffix} bearing is centered in body pocket",
        )
        ctx.expect_overlap(
            axle,
            pedal,
            axes="x",
            elem_a="bearing_barrel",
            elem_b="clipless_body",
            min_overlap=0.020,
            name=f"pedal_{suffix} bearing remains inserted laterally",
        )
        ctx.expect_within(
            axle,
            pedal,
            axes="yz",
            inner_elem="spindle",
            outer_elem="clipless_body",
            margin=0.001,
            name=f"pedal_{suffix} spindle passes through body centerline",
        )
        ctx.expect_overlap(
            axle,
            pedal,
            axes="x",
            elem_a="spindle",
            elem_b="clipless_body",
            min_overlap=0.030,
            name=f"pedal_{suffix} spindle insertion is retained",
        )

    rest_pos = ctx.part_world_position(object_model.get_part("pedal_axle_0"))
    with ctx.pose({crank_joint: math.pi / 2.0}):
        quarter_pos = ctx.part_world_position(object_model.get_part("pedal_axle_0"))
    ctx.check(
        "bottom-bracket rotation sweeps the crank arm",
        rest_pos is not None
        and quarter_pos is not None
        and quarter_pos[1] > rest_pos[1] + 0.15
        and quarter_pos[2] > rest_pos[2] + 0.15,
        details=f"rest={rest_pos}, quarter_turn={quarter_pos}",
    )

    return ctx.report()


object_model = build_object_model()
