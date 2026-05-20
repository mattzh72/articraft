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
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    wire_from_points,
)


HOUSING_OUTER = 0.248
HOUSING_DEPTH = 0.108
HOUSING_HEIGHT = 0.244
HOUSING_OPENING_RADIUS = 0.092
PIVOT_HEIGHT = 0.170


def _build_housing_shape() -> object:
    front_y = -HOUSING_DEPTH * 0.5
    rear_y = HOUSING_DEPTH * 0.5
    top_z = HOUSING_HEIGHT * 0.5

    shell = cq.Workplane("XY").box(HOUSING_OUTER, HOUSING_DEPTH, HOUSING_HEIGHT)
    shell = shell.edges("|Y").fillet(0.012)

    airflow_passage = (
        cq.Workplane("XZ")
        .circle(HOUSING_OPENING_RADIUS)
        .extrude(HOUSING_DEPTH + 0.040, both=True)
    )
    shell = shell.cut(airflow_passage)

    grille_bar_depth = 0.004
    grille_bar_thickness = 0.004
    grille_span = 0.208
    for x_pos in (-0.044, 0.0, 0.044):
        shell = shell.union(
            cq.Workplane("XY")
            .box(grille_bar_thickness, grille_bar_depth, grille_span)
            .translate((x_pos, front_y + 0.005, 0.0))
        )
        shell = shell.union(
            cq.Workplane("XY")
            .box(grille_bar_thickness, grille_bar_depth, grille_span)
            .translate((x_pos, rear_y - 0.005, 0.0))
        )
    for z_pos in (-0.044, 0.0, 0.044):
        shell = shell.union(
            cq.Workplane("XY")
            .box(grille_span, grille_bar_depth, grille_bar_thickness)
            .translate((0.0, front_y + 0.005, z_pos))
        )
        shell = shell.union(
            cq.Workplane("XY")
            .box(grille_span, grille_bar_depth, grille_bar_thickness)
            .translate((0.0, rear_y - 0.005, z_pos))
        )

    motor_hub = (
        cq.Workplane("XZ")
        .circle(0.028)
        .extrude(0.032, both=True)
        .translate((0.0, 0.024, 0.0))
    )
    shell = shell.union(motor_hub)
    shell = shell.union(
        cq.Workplane("XY").box(0.150, 0.008, 0.016).translate((0.0, 0.024, 0.0))
    )
    shell = shell.union(
        cq.Workplane("XY").box(0.016, 0.008, 0.150).translate((0.0, 0.024, 0.0))
    )

    top_recess = (
        cq.Workplane("XY")
        .box(0.086, 0.030, 0.008)
        .translate((0.0, 0.010, top_z - 0.004))
    )
    shell = shell.cut(top_recess)

    side_boss = cq.Workplane("YZ").circle(0.0085).extrude(0.012, both=True)
    shell = shell.union(side_boss.translate((HOUSING_OUTER * 0.5, 0.0, 0.0)))
    shell = shell.union(side_boss.translate((-HOUSING_OUTER * 0.5, 0.0, 0.0)))

    return shell


def _build_hook_mesh():
    hook_path = wire_from_points(
        [
            (-0.030, 0.0, 0.0),
            (-0.030, 0.028, 0.0),
            (0.0, 0.046, 0.0),
            (0.030, 0.028, 0.0),
            (0.030, 0.0, 0.0),
        ],
        radius=0.0024,
        closed_path=False,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.010,
        corner_segments=10,
        radial_segments=16,
    )
    return hook_path


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_square_utility_fan")

    stand_finish = model.material("stand_finish", rgba=(0.15, 0.16, 0.17, 1.0))
    housing_finish = model.material("housing_finish", rgba=(0.23, 0.24, 0.26, 1.0))
    blade_finish = model.material("blade_finish", rgba=(0.13, 0.13, 0.14, 1.0))
    control_finish = model.material("control_finish", rgba=(0.78, 0.48, 0.12, 1.0))
    hook_finish = model.material("hook_finish", rgba=(0.12, 0.13, 0.14, 1.0))

    stand = model.part("stand")
    stand.visual(
        Box((0.016, 0.112, 0.200)),
        origin=Origin(xyz=(-0.144, 0.0, 0.106)),
        material=stand_finish,
        name="side_plate_0",
    )
    stand.visual(
        Box((0.016, 0.112, 0.200)),
        origin=Origin(xyz=(0.144, 0.0, 0.106)),
        material=stand_finish,
        name="side_plate_1",
    )
    stand.visual(
        Box((0.308, 0.018, 0.016)),
        origin=Origin(xyz=(0.0, -0.046, 0.008)),
        material=stand_finish,
        name="foot_bar_front",
    )
    stand.visual(
        Box((0.308, 0.018, 0.016)),
        origin=Origin(xyz=(0.0, 0.046, 0.008)),
        material=stand_finish,
        name="foot_bar_rear",
    )
    stand.visual(
        Box((0.032, 0.112, 0.012)),
        origin=Origin(xyz=(-0.144, 0.0, 0.018)),
        material=stand_finish,
        name="foot_block_0",
    )
    stand.visual(
        Box((0.032, 0.112, 0.012)),
        origin=Origin(xyz=(0.144, 0.0, 0.018)),
        material=stand_finish,
        name="foot_block_1",
    )
    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_build_housing_shape(), "fan_housing"),
        material=housing_finish,
        name="housing_shell",
    )
    housing.visual(
        Box((0.068, 0.012, 0.006)),
        origin=Origin(xyz=(0.0, -0.010, HOUSING_HEIGHT * 0.5 + 0.0015)),
        material=housing_finish,
        name="hinge_bridge",
    )
    housing.visual(
        Cylinder(radius=0.005, length=0.020),
        origin=Origin(xyz=(0.0, 0.002, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=housing_finish,
        name="spindle",
    )
    housing.visual(
        Cylinder(radius=0.013, length=0.024),
        origin=Origin(xyz=(0.142, 0.020, 0.055), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=housing_finish,
        name="knob_boss",
    )
    housing.visual(
        Box((0.020, 0.042, 0.030)),
        origin=Origin(xyz=(0.132, 0.020, 0.055)),
        material=housing_finish,
        name="knob_pod",
    )

    model.articulation(
        "stand_to_housing",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=housing,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_HEIGHT)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=-0.60,
            upper=0.82,
        ),
    )

    blade = model.part("blade")
    blade.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                0.088,
                0.024,
                5,
                thickness=0.014,
                blade_pitch_deg=27.0,
                blade_sweep_deg=18.0,
                blade=FanRotorBlade(shape="broad", tip_pitch_deg=11.0, camber=0.12),
                hub=FanRotorHub(
                    style="domed",
                    rear_collar_height=0.006,
                    rear_collar_radius=0.016,
                    bore_diameter=0.006,
                ),
            ),
            "fan_blade",
        ),
        origin=Origin(xyz=(0.0, -0.036, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=blade_finish,
        name="rotor",
    )
    blade.visual(
        Cylinder(radius=0.006, length=0.024),
        origin=Origin(xyz=(0.0, -0.020, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=blade_finish,
        name="hub_stub",
    )
    model.articulation(
        "housing_to_blade",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=blade,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=40.0),
    )

    knob = model.part("knob")
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.024,
                0.018,
                body_style="skirted",
                top_diameter=0.019,
                skirt=KnobSkirt(0.028, 0.004, flare=0.06),
                grip=KnobGrip(style="fluted", count=14, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007),
                center=False,
            ),
            "speed_knob",
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=control_finish,
        name="knob_cap",
    )
    model.articulation(
        "housing_to_knob",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=knob,
        origin=Origin(xyz=(0.154, 0.020, 0.055)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=8.0),
    )

    hook = model.part("hook")
    hook.visual(
        Cylinder(radius=0.0034, length=0.060),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hook_finish,
        name="hinge_barrel",
    )
    hook.visual(
        mesh_from_geometry(_build_hook_mesh(), "hanging_hook"),
        material=hook_finish,
        name="hook_loop",
    )
    hook.visual(
        Cylinder(radius=0.0027, length=0.012),
        origin=Origin(
            xyz=(-0.030, 0.004, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=hook_finish,
        name="hook_link_0",
    )
    hook.visual(
        Cylinder(radius=0.0027, length=0.012),
        origin=Origin(
            xyz=(0.030, 0.004, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=hook_finish,
        name="hook_link_1",
    )
    model.articulation(
        "housing_to_hook",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=hook,
        origin=Origin(xyz=(0.0, -0.010, HOUSING_HEIGHT * 0.5 + 0.0079)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.5,
            lower=0.0,
            upper=1.95,
        ),
    )

    return model


def _max_z(aabb):
    if aabb is None:
        return None
    return aabb[1][2]


def _min_z(aabb):
    if aabb is None:
        return None
    return aabb[0][2]


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stand = object_model.get_part("stand")
    housing = object_model.get_part("housing")
    blade = object_model.get_part("blade")
    knob = object_model.get_part("knob")
    hook = object_model.get_part("hook")

    tilt = object_model.get_articulation("stand_to_housing")
    hook_hinge = object_model.get_articulation("housing_to_hook")

    ctx.allow_overlap(
        blade,
        housing,
        elem_a="hub_stub",
        elem_b="housing_shell",
        reason="The rotor hub sleeve is intentionally captured by the simplified motor-side hub proxy inside the housing shell.",
    )
    ctx.expect_origin_distance(
        blade,
        housing,
        axes="xz",
        max_dist=0.002,
        name="blade stays centered in the housing",
    )
    ctx.expect_gap(
        knob,
        housing,
        axis="x",
        max_gap=0.003,
        max_penetration=0.0,
        name="speed knob sits flush on the right sidewall",
    )
    ctx.expect_gap(
        hook,
        housing,
        axis="z",
        max_gap=0.012,
        max_penetration=0.0,
        name="stored hook stays close to the top frame",
    )
    ctx.expect_gap(
        housing,
        stand,
        axis="z",
        min_gap=0.020,
        negative_elem="foot_bar_front",
        name="housing clears the front foot bar at rest",
    )

    rest_blade_aabb = ctx.part_world_aabb(blade)
    rest_hook_aabb = ctx.part_world_aabb(hook)
    with ctx.pose({tilt: 0.70}):
        tilted_blade_aabb = ctx.part_world_aabb(blade)
    with ctx.pose({hook_hinge: 1.80}):
        opened_hook_aabb = ctx.part_world_aabb(hook)

    ctx.check(
        "housing tilts upward at the positive limit",
        rest_blade_aabb is not None
        and tilted_blade_aabb is not None
        and _min_z(tilted_blade_aabb) is not None
        and _min_z(rest_blade_aabb) is not None
        and _min_z(tilted_blade_aabb) > _min_z(rest_blade_aabb) + 0.030,
        details=f"rest={rest_blade_aabb}, tilted={tilted_blade_aabb}",
    )
    ctx.check(
        "hook lifts above the housing when opened",
        rest_hook_aabb is not None
        and opened_hook_aabb is not None
        and _max_z(opened_hook_aabb) is not None
        and _max_z(rest_hook_aabb) is not None
        and _max_z(opened_hook_aabb) > _max_z(rest_hook_aabb) + 0.040,
        details=f"rest={rest_hook_aabb}, opened={opened_hook_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
