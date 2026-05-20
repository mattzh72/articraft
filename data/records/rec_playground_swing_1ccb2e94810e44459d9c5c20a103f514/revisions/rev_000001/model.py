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


def _bucket_shell() -> cq.Workplane:
    """Rounded, hollow toddler bucket seat with two leg openings."""
    width = 0.56
    depth = 0.46
    height = 0.38
    wall = 0.045

    outer = (
        cq.Workplane("XY")
        .box(width, depth, height)
        .translate((0.0, 0.0, height / 2.0))
        .edges("|Z")
        .fillet(0.035)
    )
    inner_cut = (
        cq.Workplane("XY")
        .box(width - 2.0 * wall, depth - 2.0 * wall, height + 0.10)
        .translate((0.0, 0.0, wall + (height + 0.10) / 2.0))
        .edges("|Z")
        .fillet(0.020)
    )
    shell = outer.cut(inner_cut)

    # Circular toddler leg holes through the high front face, leaving a
    # central pommel/bridge between them.
    for x in (-0.105, 0.105):
        hole = (
            cq.Workplane("XY")
            .cylinder(depth + 0.16, 0.070)
            .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
            .translate((x, depth / 2.0, 0.155))
        )
        shell = shell.cut(hole)

    return shell


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="toddler_bucket_swing")

    steel = model.material("galvanized_steel", rgba=(0.62, 0.65, 0.66, 1.0))
    dark_steel = model.material("dark_hinge_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    red_rubber = model.material("molded_red_rubber", rgba=(0.82, 0.05, 0.03, 1.0))
    black_webbing = model.material("black_safety_webbing", rgba=(0.01, 0.01, 0.012, 1.0))
    yellow_cap = model.material("yellow_side_caps", rgba=(1.0, 0.72, 0.05, 1.0))

    top_z = 2.25
    pivot_z = 2.065
    beam_length = 2.80
    end_x = 1.22
    foot_y = 0.78

    frame = model.part("frame")
    frame.visual(
        Cylinder(radius=0.045, length=beam_length),
        origin=Origin(xyz=(0.0, 0.0, top_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="top_beam",
    )

    # Four splayed A-frame legs, with ground tubes that tie each end together.
    leg_length = math.sqrt(top_z**2 + foot_y**2)
    leg_roll = math.atan2(foot_y, top_z)
    for i, x in enumerate((-end_x, end_x)):
        for side, label in ((1.0, "front"), (-1.0, "rear")):
            frame.visual(
                Cylinder(radius=0.035, length=leg_length),
                origin=Origin(
                    xyz=(x, side * foot_y / 2.0, top_z / 2.0),
                    rpy=(side * leg_roll, 0.0, 0.0),
                ),
                material=steel,
                name=f"{label}_leg_{i}",
            )
        frame.visual(
            Cylinder(radius=0.032, length=1.78),
            origin=Origin(xyz=(x, 0.0, 0.045), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"ground_tube_{i}",
        )
        frame.visual(
            Cylinder(radius=0.026, length=1.40),
            origin=Origin(xyz=(x, 0.0, 1.12), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"mid_brace_{i}",
        )

    # Two short hanger clevises under the top beam.  Each clevis is fixed to
    # the frame, while the swing part carries a visible barrel between its fork
    # plates at the same pivot line.
    for i, x in enumerate((-0.31, 0.31)):
        frame.visual(
            Cylinder(radius=0.060, length=0.120),
            origin=Origin(xyz=(x, 0.0, top_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_steel,
            name=f"beam_clamp_{i}",
        )
        frame.visual(
            Box((0.060, 0.050, 0.120)),
            origin=Origin(xyz=(x, 0.0, 2.155)),
            material=dark_steel,
            name=f"hanger_stem_{i}",
        )
        for offset, side_name in ((-0.031, "inner"), (0.031, "outer")):
            frame.visual(
                Box((0.014, 0.070, 0.165)),
                origin=Origin(xyz=(x + offset, 0.0, pivot_z)),
                material=dark_steel,
                name=f"clevis_{side_name}_{i}",
            )

    swing = model.part("swing")
    swing.visual(
        mesh_from_cadquery(_bucket_shell(), "bucket_shell", tolerance=0.0015, angular_tolerance=0.12),
        origin=Origin(xyz=(0.0, 0.0, -1.60)),
        material=red_rubber,
        name="bucket_shell",
    )
    swing.visual(
        Box((0.50, 0.050, 0.050)),
        origin=Origin(xyz=(0.0, -0.238, -1.215)),
        material=yellow_cap,
        name="rear_rim",
    )
    swing.visual(
        Box((0.48, 0.045, 0.045)),
        origin=Origin(xyz=(0.0, 0.240, -1.230)),
        material=yellow_cap,
        name="front_rim",
    )

    for i, x in enumerate((-0.31, 0.31)):
        swing.visual(
            Cylinder(radius=0.025, length=0.070),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_steel,
            name=f"pivot_barrel_{i}",
        )
        swing.visual(
            Box((0.044, 0.014, 1.310)),
            origin=Origin(xyz=(x, 0.0, -0.655)),
            material=black_webbing,
            name=f"side_strap_{i}",
        )
        swing.visual(
            Box((0.070, 0.070, 0.110)),
            origin=Origin(xyz=(x, 0.0, -1.245)),
            material=yellow_cap,
            name=f"side_mount_{i}",
        )
        swing.visual(
            Cylinder(radius=0.026, length=0.090),
            origin=Origin(xyz=(x, 0.0, -1.245), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_steel,
            name=f"side_bolt_{i}",
        )

    model.articulation(
        "frame_to_swing",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=swing,
        origin=Origin(xyz=(0.0, 0.0, pivot_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=2.0, lower=-0.65, upper=0.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    swing = object_model.get_part("swing")
    hinge = object_model.get_articulation("frame_to_swing")

    for i in range(2):
        for side in ("inner", "outer"):
            ctx.allow_overlap(
                frame,
                swing,
                elem_a=f"clevis_{side}_{i}",
                elem_b=f"pivot_barrel_{i}",
                reason="The swing pivot barrel is intentionally captured inside the steel hanger clevis.",
            )
            ctx.expect_overlap(
                frame,
                swing,
                axes="x",
                elem_a=f"clevis_{side}_{i}",
                elem_b=f"pivot_barrel_{i}",
                min_overlap=0.006,
                name=f"pivot barrel {i} captured by {side} clevis plate",
            )

    ctx.expect_gap(
        frame,
        swing,
        axis="z",
        positive_elem="top_beam",
        negative_elem="bucket_shell",
        min_gap=1.20,
        name="bucket hangs well below top beam",
    )

    rest_aabb = ctx.part_element_world_aabb(swing, elem="bucket_shell")
    with ctx.pose({hinge: 0.55}):
        forward_aabb = ctx.part_element_world_aabb(swing, elem="bucket_shell")
    with ctx.pose({hinge: -0.55}):
        rear_aabb = ctx.part_element_world_aabb(swing, elem="bucket_shell")

    def _center_y(aabb):
        if aabb is None:
            return None
        mn, mx = aabb
        return (mn[1] + mx[1]) / 2.0

    rest_y = _center_y(rest_aabb)
    forward_y = _center_y(forward_aabb)
    rear_y = _center_y(rear_aabb)
    ctx.check(
        "seat swings forward and rearward about top pivots",
        rest_y is not None
        and forward_y is not None
        and rear_y is not None
        and forward_y > rest_y + 0.45
        and rear_y < rest_y - 0.45,
        details=f"rest_y={rest_y}, forward_y={forward_y}, rear_y={rear_y}",
    )

    return ctx.report()


object_model = build_object_model()
