from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    mesh_from_cadquery,
)

HOUSING_WIDTH = 0.222
HOUSING_HEIGHT = 0.222
HOUSING_DEPTH = 0.096
HOUSING_WALL = 0.009
HOUSING_CORNER = 0.020
PIVOT_HEIGHT = 0.145
STAND_HALF_WIDTH = 0.127


def _housing_shell_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(HOUSING_WIDTH, HOUSING_DEPTH, HOUSING_HEIGHT)
        .edges("|Y")
        .fillet(HOUSING_CORNER)
    )
    inner = (
        cq.Workplane("XY")
        .box(
            HOUSING_WIDTH - 2.0 * HOUSING_WALL,
            HOUSING_DEPTH + 0.002,
            HOUSING_HEIGHT - 2.0 * HOUSING_WALL,
        )
        .edges("|Y")
        .fillet(max(HOUSING_CORNER - HOUSING_WALL * 0.65, 0.006))
    )
    shell = outer.cut(inner)

    motor_y = -0.012
    motor_support = (
        cq.Workplane("XZ")
        .circle(0.024)
        .extrude(0.030, both=True)
        .translate((0.0, motor_y, 0.0))
    )
    cross_strut = (
        cq.Workplane("XY")
        .box(0.210, 0.007, 0.007)
        .translate((0.0, motor_y, 0.0))
    )
    vertical_strut = (
        cq.Workplane("XY")
        .box(0.007, 0.007, 0.210)
        .translate((0.0, motor_y, 0.0))
    )

    return shell.union(motor_support).union(cross_strut).union(vertical_strut)


def _front_grille_shape() -> cq.Workplane:
    grille_y = HOUSING_DEPTH * 0.5 - 0.004
    outer_span = HOUSING_WIDTH - 2.0 * HOUSING_WALL
    clear_span = outer_span - 0.026
    ring = (
        cq.Workplane("XY")
        .box(outer_span, 0.003, outer_span)
        .translate((0.0, grille_y, 0.0))
        .cut(
            cq.Workplane("XY")
            .box(clear_span, 0.006, clear_span)
            .translate((0.0, grille_y, 0.0))
        )
    )

    grille = ring
    for x_pos in (-0.072, -0.054, -0.036, -0.018, 0.0, 0.018, 0.036, 0.054, 0.072):
        grille = grille.union(
            cq.Workplane("XY")
            .box(0.004, 0.003, clear_span)
            .translate((x_pos, grille_y, 0.0))
        )
    for z_pos in (-0.072, -0.054, -0.036, -0.018, 0.0, 0.018, 0.036, 0.054, 0.072):
        grille = grille.union(
            cq.Workplane("XY")
            .box(clear_span, 0.003, 0.004)
            .translate((0.0, grille_y, z_pos))
        )
    return grille


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desk_box_fan")

    stand_finish = model.material("stand_finish", rgba=(0.20, 0.21, 0.23, 1.0))
    housing_finish = model.material("housing_finish", rgba=(0.82, 0.84, 0.86, 1.0))
    grille_finish = model.material("grille_finish", rgba=(0.18, 0.19, 0.21, 1.0))

    stand = model.part("stand")
    runner_length = 0.132
    tube_radius = 0.007

    stand.visual(
        Cylinder(radius=tube_radius, length=runner_length),
        origin=Origin(xyz=(-STAND_HALF_WIDTH, 0.016, tube_radius), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stand_finish,
        name="runner_0",
    )
    stand.visual(
        Cylinder(radius=tube_radius, length=runner_length),
        origin=Origin(xyz=(STAND_HALF_WIDTH, 0.016, tube_radius), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stand_finish,
        name="runner_1",
    )
    stand.visual(
        Cylinder(radius=tube_radius, length=2.0 * STAND_HALF_WIDTH),
        origin=Origin(xyz=(0.0, -0.050, tube_radius), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stand_finish,
        name="rear_bridge",
    )
    stand.visual(
        Cylinder(radius=tube_radius, length=PIVOT_HEIGHT - tube_radius),
        origin=Origin(xyz=(-STAND_HALF_WIDTH, -0.034, (PIVOT_HEIGHT - tube_radius) * 0.5 + tube_radius)),
        material=stand_finish,
        name="upright_0",
    )
    stand.visual(
        Cylinder(radius=tube_radius, length=PIVOT_HEIGHT - tube_radius),
        origin=Origin(xyz=(STAND_HALF_WIDTH, -0.034, (PIVOT_HEIGHT - tube_radius) * 0.5 + tube_radius)),
        material=stand_finish,
        name="upright_1",
    )
    stand.visual(
        Cylinder(radius=tube_radius, length=0.034),
        origin=Origin(xyz=(-STAND_HALF_WIDTH, -0.017, PIVOT_HEIGHT), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stand_finish,
        name="top_arm_0",
    )
    stand.visual(
        Cylinder(radius=tube_radius, length=0.034),
        origin=Origin(xyz=(STAND_HALF_WIDTH, -0.017, PIVOT_HEIGHT), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stand_finish,
        name="top_arm_1",
    )
    stand.visual(
        Cylinder(radius=0.0125, length=0.016),
        origin=Origin(xyz=(-STAND_HALF_WIDTH, 0.0, PIVOT_HEIGHT), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stand_finish,
        name="pivot_collar_0",
    )
    stand.visual(
        Cylinder(radius=0.0125, length=0.016),
        origin=Origin(xyz=(STAND_HALF_WIDTH, 0.0, PIVOT_HEIGHT), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stand_finish,
        name="pivot_collar_1",
    )
    stand.visual(
        Box((0.040, 0.018, 0.006)),
        origin=Origin(xyz=(-STAND_HALF_WIDTH, 0.070, 0.003)),
        material=stand_finish,
        name="foot_0",
    )
    stand.visual(
        Box((0.040, 0.018, 0.006)),
        origin=Origin(xyz=(STAND_HALF_WIDTH, 0.070, 0.003)),
        material=stand_finish,
        name="foot_1",
    )

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_housing_shell_shape(), "fan_housing_shell"),
        material=housing_finish,
        name="housing_shell",
    )
    housing.visual(
        mesh_from_cadquery(_front_grille_shape(), "fan_front_grille"),
        material=grille_finish,
        name="front_grille",
    )
    housing.visual(
        Box((0.032, 0.020, 0.030)),
        origin=Origin(xyz=(0.091, 0.038, -0.096)),
        material=housing_finish,
        name="control_pod",
    )
    housing.visual(
        Cylinder(radius=0.012, length=0.008),
        origin=Origin(xyz=(-(HOUSING_WIDTH * 0.5 + 0.004), 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grille_finish,
        name="hinge_cap_0",
    )
    housing.visual(
        Cylinder(radius=0.012, length=0.008),
        origin=Origin(xyz=(HOUSING_WIDTH * 0.5 + 0.004, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grille_finish,
        name="hinge_cap_1",
    )

    model.articulation(
        "stand_to_housing",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=housing,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=-0.28,
            upper=0.62,
        ),
    )

    blade_finish = model.material("blade_finish", rgba=(0.17, 0.19, 0.20, 1.0))
    knob_finish = model.material("knob_finish", rgba=(0.10, 0.10, 0.11, 1.0))

    blade = model.part("blade")
    blade.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                0.079,
                0.023,
                5,
                thickness=0.010,
                blade_pitch_deg=29.0,
                blade_sweep_deg=24.0,
                blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=12.0, camber=0.12),
                hub=FanRotorHub(style="domed", rear_collar_height=0.006, rear_collar_radius=0.016, bore_diameter=0.004),
            ),
            "desk_box_fan_blade",
        ),
        material=blade_finish,
        name="rotor",
    )
    blade.visual(
        Cylinder(radius=0.004, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.007)),
        material=grille_finish,
        name="shaft",
    )

    model.articulation(
        "housing_to_blade",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=blade,
        origin=Origin(xyz=(0.0, 0.032, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.2, velocity=35.0),
    )

    knob = model.part("knob")
    knob.visual(
        Cylinder(radius=0.0032, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=grille_finish,
        name="shaft",
    )
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.018,
                0.012,
                body_style="cylindrical",
                edge_radius=0.001,
                grip=KnobGrip(style="fluted", count=12, depth=0.0008),
                indicator=KnobIndicator(style="dot", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "desk_box_fan_knob",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=knob_finish,
        name="knob_body",
    )

    model.articulation(
        "housing_to_knob",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=knob,
        origin=Origin(xyz=(0.091, HOUSING_DEPTH * 0.5, -0.096), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.05, velocity=8.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.
    stand = object_model.get_part("stand")
    housing = object_model.get_part("housing")
    blade = object_model.get_part("blade")
    knob = object_model.get_part("knob")
    housing_hinge = object_model.get_articulation("stand_to_housing")
    blade_joint = object_model.get_articulation("housing_to_blade")
    knob_joint = object_model.get_articulation("housing_to_knob")

    ctx.allow_overlap(
        blade,
        housing,
        elem_a="shaft",
        elem_b="housing_shell",
        reason="The blade shaft is intentionally represented as seated inside a simplified motor support proxy in the housing center.",
    )

    housing_aabb = ctx.part_world_aabb(housing)
    housing_min_z = None if housing_aabb is None else float(housing_aabb[0][2])
    ctx.check(
        "housing sits above the desktop",
        housing_min_z is not None and housing_min_z >= 0.025,
        details=f"housing_min_z={housing_min_z}",
    )
    ctx.expect_overlap(
        housing,
        stand,
        axes="x",
        min_overlap=0.180,
        name="housing stays laterally captured between the stand sides",
    )
    ctx.expect_gap(
        housing,
        blade,
        axis="y",
        positive_elem="front_grille",
        negative_elem="rotor",
        min_gap=0.004,
        max_gap=0.018,
        name="blade sits safely behind the front grille",
    )
    ctx.expect_overlap(
        blade,
        housing,
        axes="xz",
        elem_a="rotor",
        elem_b="front_grille",
        min_overlap=0.140,
        name="blade stays centered inside the square opening",
    )

    rest_front = ctx.part_element_world_aabb(housing, elem="front_grille")
    upper_limit = housing_hinge.motion_limits.upper if housing_hinge.motion_limits is not None else None
    with ctx.pose({housing_hinge: upper_limit if upper_limit is not None else 0.5}):
        tilted_front = ctx.part_element_world_aabb(housing, elem="front_grille")

    rest_front_z = None if rest_front is None else float(rest_front[0][2] + rest_front[1][2]) * 0.5
    tilted_front_z = None if tilted_front is None else float(tilted_front[0][2] + tilted_front[1][2]) * 0.5
    ctx.check(
        "positive housing tilt raises the grille",
        rest_front_z is not None and tilted_front_z is not None and tilted_front_z > rest_front_z + 0.020,
        details=f"rest_front_z={rest_front_z}, tilted_front_z={tilted_front_z}",
    )
    knob_pos = ctx.part_world_position(knob)
    ctx.check(
        "knob sits on the front lower corner",
        knob_pos is not None and knob_pos[0] > 0.07 and knob_pos[1] > 0.040 and knob_pos[2] < 0.065,
        details=f"knob_pos={knob_pos}",
    )
    ctx.check(
        "blade and knob use continuous rotation",
        blade_joint.motion_limits is not None
        and knob_joint.motion_limits is not None
        and blade_joint.motion_limits.lower is None
        and blade_joint.motion_limits.upper is None
        and knob_joint.motion_limits.lower is None
        and knob_joint.motion_limits.upper is None,
        details=f"blade_limits={blade_joint.motion_limits}, knob_limits={knob_joint.motion_limits}",
    )

    return ctx.report()


object_model = build_object_model()
