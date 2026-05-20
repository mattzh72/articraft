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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    wire_from_points,
)

HOUSING_WIDTH = 0.58
HOUSING_HEIGHT = 0.58
HOUSING_DEPTH = 0.22
HOUSING_WALL = 0.028
GRILLE_BAR = 0.008
GRILLE_INSET = 0.010
PIVOT_HEIGHT = 0.42


def cq_box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def cq_cylinder_x(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    cx, cy, cz = center
    return cq.Workplane("YZ").circle(radius).extrude(length).translate((cx - length / 2.0, cy, cz))


def cq_cylinder_y(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    cx, cy, cz = center
    return cq.Workplane("XZ").circle(radius).extrude(length).translate((cx, cy - length / 2.0, cz))


def build_housing_shape() -> cq.Workplane:
    shell = cq.Workplane("XY").box(HOUSING_WIDTH, HOUSING_DEPTH, HOUSING_HEIGHT)
    shell = shell.edges("|Y").fillet(0.020)

    inner_cut = cq.Workplane("XY").box(
        HOUSING_WIDTH - 2.0 * HOUSING_WALL,
        HOUSING_DEPTH + 0.040,
        HOUSING_HEIGHT - 2.0 * HOUSING_WALL,
    )
    shell = shell.cut(inner_cut)

    grille_span_x = HOUSING_WIDTH - 2.0 * HOUSING_WALL + 0.010
    grille_span_z = HOUSING_HEIGHT - 2.0 * HOUSING_WALL + 0.010
    front_y = HOUSING_DEPTH / 2.0 - GRILLE_INSET
    rear_y = -HOUSING_DEPTH / 2.0 + GRILLE_INSET
    grid_offsets = (-0.17, -0.085, 0.0, 0.085, 0.17)

    for face_y in (front_y, rear_y):
        for x in grid_offsets:
            shell = shell.union(cq_box((GRILLE_BAR, 0.010, grille_span_z), (x, face_y, 0.0)))
        for z in grid_offsets:
            shell = shell.union(cq_box((grille_span_x, 0.010, GRILLE_BAR), (0.0, face_y, z)))

    motor_pod = cq_cylinder_y(0.058, 0.050, (0.0, -0.065, 0.0))
    motor_cross_x = cq_box((0.26, 0.012, 0.016), (0.0, -0.096, 0.0))
    motor_cross_z = cq_box((0.016, 0.012, 0.26), (0.0, -0.096, 0.0))
    shell = shell.union(motor_pod).union(motor_cross_x).union(motor_cross_z)

    pivot_radius = 0.028
    pivot_length = 0.036
    right_pivot = cq_cylinder_x(
        pivot_radius,
        pivot_length,
        (HOUSING_WIDTH / 2.0 + 0.010, 0.0, 0.0),
    )
    left_pivot = cq_cylinder_x(
        pivot_radius,
        pivot_length,
        (-HOUSING_WIDTH / 2.0 - 0.010, 0.0, 0.0),
    )
    shell = shell.union(right_pivot).union(left_pivot)

    handle_ear_size = (0.026, 0.032, 0.054)
    handle_y = -HOUSING_DEPTH / 2.0 - 0.008
    handle_z = HOUSING_HEIGHT / 2.0 - 0.075
    shell = shell.union(cq_box(handle_ear_size, (-0.165, handle_y, handle_z)))
    shell = shell.union(cq_box(handle_ear_size, (0.165, handle_y, handle_z)))

    knob_boss = cq_cylinder_x(
        0.031,
        0.032,
        (HOUSING_WIDTH / 2.0 + 0.016, -0.058, -0.190),
    )
    shell = shell.union(knob_boss)

    return shell


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="garage_box_fan")

    frame_black = model.material("frame_black", rgba=(0.12, 0.12, 0.13, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.66, 0.68, 0.71, 1.0))
    blade_black = model.material("blade_black", rgba=(0.08, 0.08, 0.09, 1.0))
    knob_black = model.material("knob_black", rgba=(0.10, 0.10, 0.11, 1.0))

    cradle = model.part("cradle")
    cradle.visual(
        Box((0.75, 0.030, 0.040)),
        origin=Origin(xyz=(0.0, 0.120, 0.020)),
        material=satin_steel,
        name="front_rail",
    )
    cradle.visual(
        Box((0.75, 0.030, 0.040)),
        origin=Origin(xyz=(0.0, -0.120, 0.020)),
        material=satin_steel,
        name="rear_rail",
    )
    cradle.visual(
        Box((0.030, 0.270, 0.040)),
        origin=Origin(xyz=(-0.360, 0.0, 0.020)),
        material=satin_steel,
        name="runner_0",
    )
    cradle.visual(
        Box((0.030, 0.270, 0.040)),
        origin=Origin(xyz=(0.360, 0.0, 0.020)),
        material=satin_steel,
        name="runner_1",
    )
    cradle.visual(
        Box((0.030, 0.030, 0.380)),
        origin=Origin(xyz=(-0.360, 0.0, 0.210)),
        material=satin_steel,
        name="upright_0",
    )
    cradle.visual(
        Box((0.030, 0.030, 0.380)),
        origin=Origin(xyz=(0.360, 0.0, 0.210)),
        material=satin_steel,
        name="upright_1",
    )
    cradle.visual(
        Box((0.030, 0.140, 0.030)),
        origin=Origin(xyz=(-0.360, 0.0, PIVOT_HEIGHT)),
        material=satin_steel,
        name="yoke_0",
    )
    cradle.visual(
        Box((0.030, 0.140, 0.030)),
        origin=Origin(xyz=(0.360, 0.0, PIVOT_HEIGHT)),
        material=satin_steel,
        name="yoke_1",
    )
    cradle.visual(
        Cylinder(radius=0.034, length=0.040),
        origin=Origin(xyz=(-0.338, 0.0, PIVOT_HEIGHT), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="pivot_collar_0",
    )
    cradle.visual(
        Cylinder(radius=0.034, length=0.040),
        origin=Origin(xyz=(0.338, 0.0, PIVOT_HEIGHT), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="pivot_collar_1",
    )

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(build_housing_shape(), "housing"),
        material=frame_black,
        name="housing_shell",
    )
    housing.visual(
        Cylinder(radius=0.010, length=0.087),
        origin=Origin(xyz=(0.0, -0.087, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=frame_black,
        name="shaft_nose",
    )
    model.articulation(
        "cradle_to_housing",
        ArticulationType.REVOLUTE,
        parent=cradle,
        child=housing,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.0,
            lower=math.radians(-20.0),
            upper=math.radians(45.0),
        ),
    )

    blade = model.part("blade")
    blade.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                0.220,
                0.052,
                5,
                thickness=0.034,
                blade_pitch_deg=31.0,
                blade_sweep_deg=22.0,
                blade=FanRotorBlade(shape="broad", tip_pitch_deg=14.0, camber=0.16),
                hub=FanRotorHub(
                    style="spinner",
                    rear_collar_height=0.010,
                    rear_collar_radius=0.036,
                    bore_diameter=0.014,
                ),
            ),
            "blade",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=blade_black,
        name="blade_rotor",
    )
    blade.visual(
        Cylinder(radius=0.017, length=0.050),
        origin=Origin(xyz=(0.0, -0.020, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=blade_black,
        name="hub_collar",
    )

    model.articulation(
        "housing_to_blade",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=blade,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=35.0),
    )

    handle = model.part("handle")
    handle_path = wire_from_points(
        [
            (-0.165, -0.020, 0.0),
            (-0.165, -0.042, 0.030),
            (-0.105, -0.090, 0.115),
            (0.105, -0.090, 0.115),
            (0.165, -0.042, 0.030),
            (0.165, -0.020, 0.0),
        ],
        radius=0.009,
        closed_path=False,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.022,
        corner_segments=10,
    )
    handle.visual(
        mesh_from_geometry(handle_path, "handle"),
        material=satin_steel,
        name="handle_bar",
    )
    handle.visual(
        Cylinder(radius=0.006, length=0.018),
        origin=Origin(xyz=(-0.165, 0.007, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="handle_sleeve_0",
    )
    handle.visual(
        Cylinder(radius=0.006, length=0.018),
        origin=Origin(xyz=(0.165, 0.007, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="handle_sleeve_1",
    )
    handle.visual(
        Box((0.014, 0.036, 0.012)),
        origin=Origin(xyz=(-0.165, -0.017, 0.0)),
        material=satin_steel,
        name="handle_bridge_0",
    )
    handle.visual(
        Box((0.014, 0.036, 0.012)),
        origin=Origin(xyz=(0.165, -0.017, 0.0)),
        material=satin_steel,
        name="handle_bridge_1",
    )

    model.articulation(
        "housing_to_handle",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=handle,
        origin=Origin(
            xyz=(
                0.0,
                -HOUSING_DEPTH / 2.0 - 0.036,
                HOUSING_HEIGHT / 2.0 - 0.075,
            )
        ),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(88.0),
        ),
    )

    speed_knob = model.part("speed_knob")
    speed_knob.visual(
        Cylinder(radius=0.027, length=0.014),
        origin=Origin(xyz=(0.007, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_black,
        name="knob_skirt",
    )
    speed_knob.visual(
        Cylinder(radius=0.022, length=0.020),
        origin=Origin(xyz=(0.024, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_black,
        name="knob_cap",
    )
    speed_knob.visual(
        Box((0.0025, 0.006, 0.016)),
        origin=Origin(xyz=(0.035, 0.0, 0.013)),
        material=satin_steel,
        name="knob_indicator",
    )

    model.articulation(
        "housing_to_speed_knob",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=speed_knob,
        origin=Origin(xyz=(HOUSING_WIDTH / 2.0 + 0.032, -0.058, -0.190)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    blade = object_model.get_part("blade")
    handle = object_model.get_part("handle")
    speed_knob = object_model.get_part("speed_knob")

    tilt_joint = object_model.get_articulation("cradle_to_housing")
    handle_joint = object_model.get_articulation("housing_to_handle")
    blade_joint = object_model.get_articulation("housing_to_blade")
    knob_joint = object_model.get_articulation("housing_to_speed_knob")

    ctx.allow_overlap(
        handle,
        housing,
        elem_a="handle_sleeve_0",
        elem_b="housing_shell",
        reason="The carry handle uses simplified sleeve geometry that seats inside the rear pivot ear proxy.",
    )
    ctx.allow_overlap(
        handle,
        housing,
        elem_a="handle_sleeve_1",
        elem_b="housing_shell",
        reason="The carry handle uses simplified sleeve geometry that seats inside the rear pivot ear proxy.",
    )
    ctx.allow_overlap(
        blade,
        housing,
        elem_a="hub_collar",
        elem_b="shaft_nose",
        reason="The blade hub is represented by a simplified collar running on the stationary motor shaft proxy.",
    )

    ctx.expect_within(
        blade,
        housing,
        axes="xz",
        margin=0.040,
        name="blade stays inside square opening",
    )
    ctx.expect_contact(
        speed_knob,
        housing,
        contact_tol=0.002,
        name="speed knob seats on side boss",
    )
    ctx.expect_origin_gap(
        housing,
        handle,
        axis="y",
        min_gap=0.120,
        max_gap=0.160,
        name="handle origin sits behind the housing",
    )

    rest_housing_aabb = ctx.part_world_aabb(housing)
    tilt_upper = tilt_joint.motion_limits.upper if tilt_joint.motion_limits is not None else None
    if tilt_upper is not None:
        with ctx.pose({tilt_joint: tilt_upper}):
            tilted_housing_aabb = ctx.part_world_aabb(housing)
            ctx.check(
                "housing pitches in the cradle",
                rest_housing_aabb is not None
                and tilted_housing_aabb is not None
                and tilted_housing_aabb[1][1] > rest_housing_aabb[1][1] + 0.120
                and tilted_housing_aabb[0][1] < rest_housing_aabb[0][1] - 0.120,
                details=f"rest={rest_housing_aabb}, tilted={tilted_housing_aabb}",
            )
            ctx.expect_within(
                blade,
                housing,
                axes="xz",
                margin=0.045,
                name="blade remains caged while tilted",
            )

    handle_upper = handle_joint.motion_limits.upper if handle_joint.motion_limits is not None else None
    if handle_upper is not None:
        with ctx.pose({handle_joint: handle_upper}):
            handle_aabb = ctx.part_world_aabb(handle)
            lifted_housing_aabb = ctx.part_world_aabb(housing)
            ctx.check(
                "handle lifts above housing",
                handle_aabb is not None
                and lifted_housing_aabb is not None
                and handle_aabb[1][2] > lifted_housing_aabb[1][2] + 0.030,
                details=f"handle={handle_aabb}, housing={lifted_housing_aabb}",
            )

    with ctx.pose({blade_joint: 1.4, knob_joint: 1.2}):
        ctx.expect_within(
            blade,
            housing,
            axes="xz",
            margin=0.045,
            name="blade stays centered while spinning",
        )
        knob_aabb = ctx.part_world_aabb(speed_knob)
        housing_aabb = ctx.part_world_aabb(housing)
        ctx.check(
            "speed knob remains outside housing wall",
            knob_aabb is not None
            and housing_aabb is not None
            and knob_aabb[0][0] >= housing_aabb[1][0] - 0.002,
            details=f"knob={knob_aabb}, housing={housing_aabb}",
        )

    ctx.check(
        "blade spin is continuous",
        blade_joint.motion_limits is not None
        and blade_joint.motion_limits.lower is None
        and blade_joint.motion_limits.upper is None,
        details=f"limits={blade_joint.motion_limits}",
    )
    ctx.check(
        "speed knob rotation is continuous",
        knob_joint.motion_limits is not None
        and knob_joint.motion_limits.lower is None
        and knob_joint.motion_limits.upper is None,
        details=f"limits={knob_joint.motion_limits}",
    )

    return ctx.report()


object_model = build_object_model()
