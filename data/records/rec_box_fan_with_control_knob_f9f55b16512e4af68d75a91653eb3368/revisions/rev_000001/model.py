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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_DEPTH = 0.145
BODY_WIDTH = 0.390
BODY_HEIGHT = 0.390
BODY_WALL = 0.018
FRONT_FRAME_DEPTH = 0.020
REAR_FRAME_DEPTH = 0.022
CORNER_RADIUS = 0.026

OPENING_RADIUS = 0.155
GRILLE_BAR_HALF = 0.0015
GRILLE_THICKNESS = 0.003

SHROUD_INNER_RADIUS = 0.148
SHROUD_OUTER_RADIUS = 0.160

ROTOR_RADIUS = 0.133

HANDLE_PIVOT_X = -BODY_DEPTH / 2.0 - 0.0155
HANDLE_SUPPORT_X = -BODY_DEPTH / 2.0 - 0.006
HANDLE_PIVOT_Z = 0.145
HANDLE_BARREL_Y = 0.136
HANDLE_OUTBOARD_Y = 0.160
HANDLE_CROSSBAR_X = -0.055
HANDLE_CROSSBAR_Z = 0.035
HANDLE_CROSSBAR_LENGTH = 0.220
SPINDLE_LENGTH = BODY_DEPTH / 2.0 - REAR_FRAME_DEPTH * 0.55

KNOB_BOSS_RADIUS = 0.022
KNOB_BOSS_DEPTH = 0.006
KNOB_X = 0.040
KNOB_Z = -0.026


def annulus_on_yz(inner_radius: float, outer_radius: float, thickness: float, x_center: float) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(thickness)
        .translate((x_center - thickness / 2.0, 0.0, 0.0))
    )


def disk_on_yz(radius: float, thickness: float, x_center: float) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .circle(radius)
        .extrude(thickness)
        .translate((x_center - thickness / 2.0, 0.0, 0.0))
    )


def grille_bar(x_center: float, y_size: float, z_size: float, angle_deg: float = 0.0) -> cq.Workplane:
    bar = cq.Workplane("XY").box(GRILLE_THICKNESS, y_size, z_size)
    if angle_deg:
        bar = bar.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), angle_deg)
    return bar.translate((x_center, 0.0, 0.0))


def build_radial_grille(x_center: float) -> cq.Workplane:
    grille = disk_on_yz(0.023, GRILLE_THICKNESS, x_center)
    for radius in (0.045, 0.068, 0.091, 0.114, 0.137):
        grille = grille.union(
            annulus_on_yz(
                radius - GRILLE_BAR_HALF,
                radius + GRILLE_BAR_HALF,
                GRILLE_THICKNESS,
                x_center,
            )
        )

    grille = grille.union(grille_bar(x_center, 0.008, OPENING_RADIUS * 2.18))
    grille = grille.union(grille_bar(x_center, OPENING_RADIUS * 2.18, 0.008))
    grille = grille.union(grille_bar(x_center, OPENING_RADIUS * 1.92, 0.005, 45.0))
    grille = grille.union(grille_bar(x_center, OPENING_RADIUS * 1.92, 0.005, -45.0))
    return grille


def build_body_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").box(BODY_DEPTH, BODY_WIDTH, BODY_HEIGHT).edges("|X").fillet(CORNER_RADIUS)
    inner = cq.Workplane("XY").box(
        BODY_DEPTH - FRONT_FRAME_DEPTH - REAR_FRAME_DEPTH,
        BODY_WIDTH - 2.0 * BODY_WALL,
        BODY_HEIGHT - 2.0 * BODY_WALL,
    )
    body = outer.cut(inner)

    front_cut_thickness = FRONT_FRAME_DEPTH + 0.005
    rear_cut_thickness = REAR_FRAME_DEPTH + 0.005
    front_cutter = (
        cq.Workplane("YZ")
        .circle(OPENING_RADIUS)
        .extrude(front_cut_thickness)
        .translate((BODY_DEPTH / 2.0 - front_cut_thickness, 0.0, 0.0))
    )
    rear_cutter = (
        cq.Workplane("YZ")
        .circle(OPENING_RADIUS)
        .extrude(rear_cut_thickness)
        .translate((-BODY_DEPTH / 2.0, 0.0, 0.0))
    )
    body = body.cut(front_cutter).cut(rear_cutter)

    shroud_length = BODY_DEPTH - FRONT_FRAME_DEPTH - REAR_FRAME_DEPTH
    shroud_center_x = (REAR_FRAME_DEPTH - FRONT_FRAME_DEPTH) / 2.0
    shroud = annulus_on_yz(
        SHROUD_INNER_RADIUS,
        SHROUD_OUTER_RADIUS,
        shroud_length,
        shroud_center_x,
    )
    body = body.union(shroud)

    front_grille_x = BODY_DEPTH / 2.0 - FRONT_FRAME_DEPTH * 0.55
    rear_grille_x = -BODY_DEPTH / 2.0 + REAR_FRAME_DEPTH * 0.55
    body = body.union(build_radial_grille(front_grille_x))
    body = body.union(build_radial_grille(rear_grille_x))

    knob_boss = (
        cq.Workplane("XZ")
        .circle(KNOB_BOSS_RADIUS)
        .extrude(KNOB_BOSS_DEPTH)
        .translate((KNOB_X, BODY_WIDTH / 2.0, KNOB_Z))
    )
    body = body.union(knob_boss)
    knob_hole = (
        cq.Workplane("XZ")
        .circle(0.0052)
        .extrude(0.028)
        .translate((KNOB_X, BODY_WIDTH / 2.0 - 0.016, KNOB_Z))
    )
    body = body.cut(knob_hole)

    for side in (-1.0, 1.0):
        support = cq.Workplane("XY").box(0.012, 0.010, 0.024).translate(
            (HANDLE_SUPPORT_X, side * HANDLE_OUTBOARD_Y, HANDLE_PIVOT_Z)
        )
        body = body.union(support)

    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="box_fan")

    housing = model.material("housing", rgba=(0.20, 0.22, 0.24, 1.0))
    trim = model.material("trim", rgba=(0.14, 0.15, 0.16, 1.0))
    rotor_finish = model.material("rotor_finish", rgba=(0.80, 0.82, 0.84, 1.0))
    metal = model.material("metal", rgba=(0.65, 0.68, 0.72, 1.0))
    control = model.material("control", rgba=(0.12, 0.12, 0.13, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(build_body_shape(), "body_shell"),
        material=housing,
        name="housing_shell",
    )
    body.visual(
        Cylinder(radius=0.005, length=SPINDLE_LENGTH),
        origin=Origin(
            xyz=(-SPINDLE_LENGTH / 2.0, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=metal,
        name="spindle",
    )

    blade = model.part("blade")
    blade.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                ROTOR_RADIUS,
                0.030,
                5,
                thickness=0.012,
                blade_pitch_deg=30.0,
                blade_sweep_deg=26.0,
                blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=16.0, camber=0.15),
                hub=FanRotorHub(
                    style="spinner",
                    rear_collar_height=0.008,
                    rear_collar_radius=0.022,
                    bore_diameter=0.011,
                ),
            ),
            "fan_rotor",
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rotor_finish,
        name="rotor",
    )
    blade.visual(
        Cylinder(radius=0.0032, length=0.008),
        origin=Origin(
            xyz=(0.0, 0.024, 0.004),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=trim,
        name="marker",
    )
    blade.visual(
        Cylinder(radius=0.0125, length=0.014),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0), xyz=(0.007, 0.0, 0.0)),
        material=metal,
        name="mount_collar",
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.009, length=HANDLE_CROSSBAR_LENGTH),
        origin=Origin(
            xyz=(HANDLE_CROSSBAR_X, 0.0, HANDLE_CROSSBAR_Z),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=trim,
        name="crossbar",
    )

    arm_length = math.hypot(-HANDLE_CROSSBAR_X, HANDLE_CROSSBAR_Z)
    arm_pitch = math.atan2(HANDLE_CROSSBAR_X, HANDLE_CROSSBAR_Z)
    for index, side in enumerate((-1.0, 1.0)):
        handle.visual(
            Cylinder(radius=0.0075, length=arm_length),
            origin=Origin(
                xyz=(HANDLE_CROSSBAR_X / 2.0, side * 0.110, HANDLE_CROSSBAR_Z / 2.0),
                rpy=(0.0, arm_pitch, 0.0),
            ),
            material=trim,
            name=f"arm_{index}",
        )
        handle.visual(
            Cylinder(radius=0.0035, length=0.040),
            origin=Origin(
                xyz=(0.0, side * HANDLE_BARREL_Y, 0.0),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=trim,
            name=f"pivot_{index}",
        )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.032,
                0.018,
                body_style="cylindrical",
                grip=KnobGrip(style="fluted", count=12, depth=0.0011),
                center=False,
            ),
            "selector_knob",
        ),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=control,
        name="knob",
    )
    selector_knob.visual(
        Box((0.0045, 0.0025, 0.0040)),
        origin=Origin(xyz=(0.0, 0.0175, 0.0125)),
        material=metal,
        name="pointer",
    )
    selector_knob.visual(
        Cylinder(radius=0.0044, length=0.008),
        origin=Origin(
            xyz=(0.0, -0.002, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=metal,
        name="shaft",
    )

    model.articulation(
        "body_to_blade",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=blade,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=18.0),
    )
    model.articulation(
        "body_to_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(HANDLE_PIVOT_X, 0.0, HANDLE_PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.5,
            lower=0.0,
            upper=1.18,
        ),
    )
    model.articulation(
        "body_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector_knob,
        origin=Origin(xyz=(KNOB_X, BODY_WIDTH / 2.0 + KNOB_BOSS_DEPTH, KNOB_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    blade = object_model.get_part("blade")
    handle = object_model.get_part("handle")
    selector_knob = object_model.get_part("selector_knob")

    blade_joint = object_model.get_articulation("body_to_blade")
    handle_joint = object_model.get_articulation("body_to_handle")
    knob_joint = object_model.get_articulation("body_to_selector_knob")

    ctx.expect_origin_distance(
        blade,
        body,
        axes="yz",
        max_dist=0.001,
        name="blade stays centered in the housing opening",
    )
    ctx.expect_within(
        blade,
        body,
        axes="yz",
        margin=0.0,
        name="blade stays inside the box fan footprint",
    )

    folded_crossbar = ctx.part_element_world_aabb(handle, elem="crossbar")
    raised_crossbar = None
    with ctx.pose({handle_joint: handle_joint.motion_limits.upper or 1.10}):
        raised_crossbar = ctx.part_element_world_aabb(handle, elem="crossbar")

    ctx.check(
        "folded handle parks behind the rear grille frame",
        folded_crossbar is not None and folded_crossbar[1][0] < (-BODY_DEPTH / 2.0) - 0.001,
        details=f"folded_crossbar={folded_crossbar}",
    )
    ctx.check(
        "handle lifts above the housing when raised",
        folded_crossbar is not None
        and raised_crossbar is not None
        and raised_crossbar[1][2] > folded_crossbar[1][2] + 0.028
        and raised_crossbar[1][2] > (BODY_HEIGHT / 2.0) + 0.022,
        details=f"folded_crossbar={folded_crossbar}, raised_crossbar={raised_crossbar}",
    )

    marker_rest = ctx.part_element_world_aabb(blade, elem="marker")
    marker_spun = None
    with ctx.pose({blade_joint: 0.95}):
        marker_spun = ctx.part_element_world_aabb(blade, elem="marker")

    ctx.check(
        "blade marker sweeps around the axle",
        marker_rest is not None
        and marker_spun is not None
        and abs((marker_spun[0][1] + marker_spun[1][1]) * 0.5 - (marker_rest[0][1] + marker_rest[1][1]) * 0.5)
        > 0.008
        and abs((marker_spun[0][2] + marker_spun[1][2]) * 0.5 - (marker_rest[0][2] + marker_rest[1][2]) * 0.5)
        > 0.008,
        details=f"marker_rest={marker_rest}, marker_spun={marker_spun}",
    )

    pointer_rest = ctx.part_element_world_aabb(selector_knob, elem="pointer")
    pointer_turned = None
    with ctx.pose({knob_joint: 1.20}):
        pointer_turned = ctx.part_element_world_aabb(selector_knob, elem="pointer")

    ctx.check(
        "selector knob pointer rotates around the side shaft",
        pointer_rest is not None
        and pointer_turned is not None
        and abs((pointer_turned[0][0] + pointer_turned[1][0]) * 0.5 - (pointer_rest[0][0] + pointer_rest[1][0]) * 0.5)
        > 0.008
        and abs((pointer_turned[0][2] + pointer_turned[1][2]) * 0.5 - (pointer_rest[0][2] + pointer_rest[1][2]) * 0.5)
        > 0.004,
        details=f"pointer_rest={pointer_rest}, pointer_turned={pointer_turned}",
    )

    return ctx.report()


object_model = build_object_model()
