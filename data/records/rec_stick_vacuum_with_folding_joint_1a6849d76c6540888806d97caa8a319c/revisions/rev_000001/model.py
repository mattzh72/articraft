from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    ConeGeometry,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    wire_from_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_stick_vacuum")

    graphite = model.material("graphite", rgba=(0.07, 0.075, 0.08, 1.0))
    charcoal = model.material("charcoal", rgba=(0.16, 0.17, 0.18, 1.0))
    blue = model.material("satin_blue", rgba=(0.08, 0.18, 0.35, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    steel = model.material("brushed_steel", rgba=(0.67, 0.70, 0.72, 1.0))
    clear_bin = model.material("smoked_clear_bin", rgba=(0.55, 0.72, 0.86, 0.42))

    body = model.part("motor_body")
    body.visual(
        mesh_from_geometry(CapsuleGeometry(0.095, 0.28, radial_segments=40), "motor_pod"),
        origin=Origin(xyz=(-0.145, 0.0, 0.300)),
        material=graphite,
        name="motor_pod",
    )
    body.visual(
        Cylinder(radius=0.066, length=0.245),
        origin=Origin(xyz=(0.035, 0.0, 0.185)),
        material=clear_bin,
        name="dust_bin",
    )
    body.visual(
        mesh_from_geometry(ConeGeometry(0.055, 0.105, radial_segments=36), "cyclone_cone"),
        origin=Origin(xyz=(0.035, 0.0, 0.125), rpy=(pi, 0.0, 0.0)),
        material=blue,
        name="cyclone_cone",
    )
    body.visual(
        Box((0.090, 0.120, 0.080)),
        origin=Origin(xyz=(-0.090, 0.0, 0.040)),
        material=charcoal,
        name="hinge_neck",
    )
    for side, y in (("side_0", -0.066), ("side_1", 0.066)):
        body.visual(
            Box((0.060, 0.026, 0.058)),
            origin=Origin(xyz=(-0.025, y, 0.020)),
            material=charcoal,
            name=f"fold_fork_{side}",
        )
        body.visual(
            Cylinder(radius=0.034, length=0.040),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"fold_barrel_{side}",
        )

    handle_loop = wire_from_points(
        [
            (-0.150, 0.0, 0.075),
            (-0.245, 0.0, 0.165),
            (-0.225, 0.0, 0.355),
            (-0.075, 0.0, 0.405),
            (-0.020, 0.0, 0.305),
            (-0.115, 0.0, 0.235),
        ],
        radius=0.012,
        radial_segments=18,
        closed_path=True,
        corner_mode="fillet",
        corner_radius=0.030,
        corner_segments=10,
    )
    body.visual(
        mesh_from_geometry(handle_loop, "handle_loop"),
        material=graphite,
        name="handle_loop",
    )
    guard_hoop = wire_from_points(
        [
            (0.086, -0.086, -0.105),
            (0.086, 0.086, -0.105),
            (-0.086, 0.086, -0.105),
            (-0.086, -0.086, -0.105),
        ],
        radius=0.0065,
        radial_segments=16,
        closed_path=True,
        corner_mode="fillet",
        corner_radius=0.030,
        corner_segments=8,
    )
    guard_hoop.merge(
        wire_from_points(
            [(0.032, -0.086, 0.0), (0.086, -0.086, -0.105)],
            radius=0.0065,
            radial_segments=16,
            cap_ends=True,
        )
    )
    guard_hoop.merge(
        wire_from_points(
            [(0.032, 0.086, 0.0), (0.086, 0.086, -0.105)],
            radius=0.0065,
            radial_segments=16,
            cap_ends=True,
        )
    )
    body.visual(
        mesh_from_geometry(guard_hoop, "fixed_guard_frame"),
        material=steel,
        name="guard_frame",
    )

    wand = model.part("wand")
    wand.visual(
        Cylinder(radius=0.018, length=0.940),
        origin=Origin(xyz=(0.0, 0.0, -0.475)),
        material=blue,
        name="wand_tube",
    )
    wand.visual(
        Cylinder(radius=0.030, length=0.092),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="fold_knuckle",
    )
    wand.visual(
        Cylinder(radius=0.024, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, -0.960), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="head_knuckle",
    )
    wand.visual(
        Cylinder(radius=0.024, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, -0.065)),
        material=charcoal,
        name="upper_collar",
    )
    wand.visual(
        Cylinder(radius=0.023, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, -0.905)),
        material=charcoal,
        name="lower_collar",
    )

    floor_head = model.part("floor_head")
    for side, y in (("side_0", -0.067), ("side_1", 0.067)):
        floor_head.visual(
            Cylinder(radius=0.028, length=0.034),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"head_barrel_{side}",
        )
        floor_head.visual(
            Box((0.082, 0.026, 0.026)),
            origin=Origin(xyz=(0.040, y, -0.027)),
            material=charcoal,
            name=f"head_yoke_{side}",
        )
    floor_head.visual(
        Box((0.230, 0.300, 0.060)),
        origin=Origin(xyz=(0.160, 0.0, -0.058)),
        material=charcoal,
        name="head_shell",
    )
    floor_head.visual(
        Box((0.026, 0.292, 0.040)),
        origin=Origin(xyz=(0.285, 0.0, -0.062)),
        material=rubber,
        name="front_bumper",
    )
    floor_head.visual(
        Cylinder(radius=0.023, length=0.245),
        origin=Origin(xyz=(0.220, 0.0, -0.092), rpy=(pi / 2.0, 0.0, 0.0)),
        material=blue,
        name="brush_roller",
    )
    for side, y in (("side_0", -0.151), ("side_1", 0.151)):
        floor_head.visual(
            Cylinder(radius=0.023, length=0.018),
            origin=Origin(xyz=(0.075, y, -0.087), rpy=(pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name=f"wheel_{side}",
        )

    model.articulation(
        "fold_joint",
        ArticulationType.REVOLUTE,
        parent=body,
        child=wand,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=1.65),
    )
    model.articulation(
        "head_pitch",
        ArticulationType.REVOLUTE,
        parent=wand,
        child=floor_head,
        origin=Origin(xyz=(0.0, 0.0, -0.960)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.5, lower=-0.55, upper=0.75),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("motor_body")
    wand = object_model.get_part("wand")
    floor_head = object_model.get_part("floor_head")
    fold = object_model.get_articulation("fold_joint")
    pitch = object_model.get_articulation("head_pitch")

    ctx.check(
        "fold joint is a horizontal revolute hinge",
        fold.articulation_type == ArticulationType.REVOLUTE and abs(fold.axis[1]) > 0.99,
        details=f"type={fold.articulation_type}, axis={fold.axis}",
    )
    ctx.check(
        "floor head pitch is a horizontal revolute hinge",
        pitch.articulation_type == ArticulationType.REVOLUTE and abs(pitch.axis[1]) > 0.99,
        details=f"type={pitch.articulation_type}, axis={pitch.axis}",
    )
    ctx.expect_within(
        wand,
        body,
        axes="xy",
        inner_elem="wand_tube",
        outer_elem="guard_frame",
        margin=0.0,
        name="fixed guard frame surrounds the first wand stage",
    )
    ctx.expect_overlap(
        body,
        wand,
        axes="z",
        elem_a="guard_frame",
        elem_b="wand_tube",
        min_overlap=0.050,
        name="guard frame spans along the upper moving wand stage",
    )

    rest_head = ctx.part_world_position(floor_head)
    with ctx.pose({fold: 1.20}):
        folded_head = ctx.part_world_position(floor_head)
    ctx.check(
        "fold joint swings the wand and head upward in front of the motor body",
        rest_head is not None
        and folded_head is not None
        and folded_head[0] > rest_head[0] + 0.55
        and folded_head[2] > rest_head[2] + 0.55,
        details=f"rest={rest_head}, folded={folded_head}",
    )

    rest_bumper = ctx.part_element_world_aabb(floor_head, elem="front_bumper")
    with ctx.pose({pitch: 0.55}):
        pitched_bumper = ctx.part_element_world_aabb(floor_head, elem="front_bumper")
    ctx.check(
        "floor head pitch lifts the front bumper",
        rest_bumper is not None
        and pitched_bumper is not None
        and (pitched_bumper[0][2] + pitched_bumper[1][2]) * 0.5
        > (rest_bumper[0][2] + rest_bumper[1][2]) * 0.5 + 0.070,
        details=f"rest={rest_bumper}, pitched={pitched_bumper}",
    )

    return ctx.report()


object_model = build_object_model()
