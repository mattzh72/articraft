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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)

HOUSING_W = 0.205
HOUSING_H = 0.205
HOUSING_D = 0.088
HOUSING_CORNER = 0.028
APERTURE_W = 0.174
APERTURE_H = 0.174

PIVOT_Y = -0.008
PIVOT_Z = 0.124

HOUSING_BOSS_RADIUS = 0.011
HOUSING_BOSS_LEN = 0.012
HOUSING_BOSS_X = HOUSING_W * 0.5 + HOUSING_BOSS_LEN * 0.5

STAND_TUBE_RADIUS = 0.007
STAND_SOCKET_RADIUS = 0.0105
STAND_SOCKET_LEN = 0.018
STAND_SOCKET_X = HOUSING_BOSS_X + 0.015

HANDLE_PIVOT_X = 0.082
HANDLE_PIVOT_Y = -(HOUSING_D * 0.5 + 0.004)
HANDLE_PIVOT_Z = 0.066

KNOB_Z = -0.041
KNOB_Y = -0.002
KNOB_X = -(HOUSING_W * 0.5 + 0.002)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _midpoint(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, *, name: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _build_housing_shell():
    outer = cq.Workplane("XY").box(HOUSING_W, HOUSING_D, HOUSING_H).edges("|Y").fillet(HOUSING_CORNER)
    inner = (
        cq.Workplane("XY")
        .box(APERTURE_W, HOUSING_D + 0.006, APERTURE_H)
        .edges("|Y")
        .fillet(max(HOUSING_CORNER - 0.007, 0.004))
    )
    return outer.cut(inner)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_square_desk_fan")

    shell_white = model.material("shell_white", rgba=(0.89, 0.90, 0.88, 1.0))
    warm_gray = model.material("warm_gray", rgba=(0.53, 0.55, 0.57, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.22, 0.23, 0.25, 1.0))
    charcoal = model.material("charcoal", rgba=(0.10, 0.11, 0.12, 1.0))
    knob_black = model.material("knob_black", rgba=(0.08, 0.08, 0.09, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.06, 1.0))

    stand = model.part("stand")

    left_arm_points = [
        (-STAND_SOCKET_X, 0.072, STAND_TUBE_RADIUS),
        (-STAND_SOCKET_X, -0.062, STAND_TUBE_RADIUS),
        (-STAND_SOCKET_X, -0.040, 0.060),
        (-STAND_SOCKET_X, PIVOT_Y, PIVOT_Z),
    ]
    right_arm_points = [(-x, y, z) for x, y, z in left_arm_points]
    stand.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                left_arm_points,
                radius=STAND_TUBE_RADIUS,
                samples_per_segment=18,
                radial_segments=18,
                cap_ends=True,
            ),
            "stand_arm_0",
        ),
        material=dark_gray,
        name="arm_0",
    )
    stand.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                right_arm_points,
                radius=STAND_TUBE_RADIUS,
                samples_per_segment=18,
                radial_segments=18,
                cap_ends=True,
            ),
            "stand_arm_1",
        ),
        material=dark_gray,
        name="arm_1",
    )
    stand.visual(
        Cylinder(radius=STAND_TUBE_RADIUS, length=STAND_SOCKET_X * 2.0),
        origin=Origin(xyz=(0.0, 0.072, STAND_TUBE_RADIUS), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_gray,
        name="front_bar",
    )
    stand.visual(
        Cylinder(radius=STAND_TUBE_RADIUS, length=STAND_SOCKET_X * 2.0),
        origin=Origin(xyz=(0.0, -0.062, STAND_TUBE_RADIUS), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_gray,
        name="rear_bar",
    )
    stand.visual(
        Cylinder(radius=STAND_SOCKET_RADIUS, length=STAND_SOCKET_LEN),
        origin=Origin(xyz=(-STAND_SOCKET_X, PIVOT_Y, PIVOT_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_gray,
        name="socket_0",
    )
    stand.visual(
        Cylinder(radius=STAND_SOCKET_RADIUS, length=STAND_SOCKET_LEN),
        origin=Origin(xyz=(STAND_SOCKET_X, PIVOT_Y, PIVOT_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_gray,
        name="socket_1",
    )
    for index, xyz in enumerate(
        (
            (-STAND_SOCKET_X, 0.072, STAND_TUBE_RADIUS),
            (STAND_SOCKET_X, 0.072, STAND_TUBE_RADIUS),
            (-STAND_SOCKET_X, -0.062, STAND_TUBE_RADIUS),
            (STAND_SOCKET_X, -0.062, STAND_TUBE_RADIUS),
        )
    ):
        stand.visual(
            Cylinder(radius=0.009, length=0.004),
            origin=Origin(xyz=(xyz[0], xyz[1], xyz[2] - 0.002)),
            material=rubber,
            name=f"foot_{index}",
        )

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_build_housing_shell(), "housing_shell"),
        material=shell_white,
        name="shell",
    )

    for index, z in enumerate((-0.058, -0.029, 0.0, 0.029, 0.058)):
        housing.visual(
            Box((0.186, 0.004, 0.004)),
            origin=Origin(xyz=(0.0, HOUSING_D * 0.5 - 0.003, z)),
            material=warm_gray,
            name=f"front_bar_{index}",
        )
    for index, x in enumerate((-0.058, -0.029, 0.029, 0.058)):
        housing.visual(
            Box((0.004, 0.004, 0.186)),
            origin=Origin(xyz=(x, HOUSING_D * 0.5 - 0.003, 0.0)),
            material=warm_gray,
            name=f"front_stile_{index}",
        )

    housing.visual(
        Cylinder(radius=0.018, length=0.004),
        origin=Origin(xyz=(0.0, HOUSING_D * 0.5 - 0.003, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=warm_gray,
        name="front_badge",
    )

    housing.visual(
        Box((0.192, 0.006, 0.006)),
        origin=Origin(xyz=(0.0, -0.044, 0.0)),
        material=warm_gray,
        name="rear_bar_x",
    )
    housing.visual(
        Box((0.006, 0.006, 0.192)),
        origin=Origin(xyz=(0.0, -0.044, 0.0)),
        material=warm_gray,
        name="rear_bar_z",
    )
    housing.visual(
        Cylinder(radius=0.041, length=0.034),
        origin=Origin(xyz=(0.0, -(HOUSING_D * 0.5 + 0.017), 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="rear_cap",
    )
    housing.visual(
        Cylinder(radius=0.002, length=0.052),
        origin=Origin(xyz=(0.0, -0.026, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=warm_gray,
        name="motor_shaft",
    )

    housing.visual(
        Cylinder(radius=HOUSING_BOSS_RADIUS, length=HOUSING_BOSS_LEN),
        origin=Origin(xyz=(-HOUSING_BOSS_X, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_gray,
        name="pivot_boss_0",
    )
    housing.visual(
        Cylinder(radius=HOUSING_BOSS_RADIUS, length=HOUSING_BOSS_LEN),
        origin=Origin(xyz=(HOUSING_BOSS_X, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_gray,
        name="pivot_boss_1",
    )

    housing.visual(
        Cylinder(radius=0.008, length=0.014),
        origin=Origin(xyz=(-0.095, HANDLE_PIVOT_Y, HANDLE_PIVOT_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_gray,
        name="handle_lug_0",
    )
    housing.visual(
        Cylinder(radius=0.008, length=0.014),
        origin=Origin(xyz=(0.095, HANDLE_PIVOT_Y, HANDLE_PIVOT_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_gray,
        name="handle_lug_1",
    )

    housing.visual(
        Cylinder(radius=0.014, length=0.008),
        origin=Origin(xyz=(-(HOUSING_W * 0.5 + 0.002), KNOB_Y, KNOB_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_gray,
        name="knob_collar",
    )

    blade = model.part("blade")
    blade.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                0.073,
                0.021,
                5,
                thickness=0.014,
                blade_pitch_deg=29.0,
                blade_sweep_deg=20.0,
                blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=14.0, camber=0.12),
                hub=FanRotorHub(
                    style="spinner",
                    rear_collar_height=0.008,
                    rear_collar_radius=0.012,
                    bore_diameter=0.004,
                ),
            ),
            "blade_rotor",
        ),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="rotor",
    )

    handle = model.part("handle")
    handle.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (-HANDLE_PIVOT_X, 0.0, 0.0),
                    (-HANDLE_PIVOT_X, -0.022, -0.055),
                    (-0.060, -0.055, -0.120),
                    (0.0, -0.050, -0.140),
                    (0.060, -0.055, -0.120),
                    (HANDLE_PIVOT_X, -0.022, -0.055),
                    (HANDLE_PIVOT_X, 0.0, 0.0),
                ],
                radius=0.005,
                samples_per_segment=18,
                radial_segments=18,
                cap_ends=True,
            ),
            "rear_handle",
        ),
        material=dark_gray,
        name="loop",
    )
    handle.visual(
        Cylinder(radius=0.009, length=0.012),
        origin=Origin(xyz=(-HANDLE_PIVOT_X, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="sleeve_0",
    )
    handle.visual(
        Cylinder(radius=0.009, length=0.012),
        origin=Origin(xyz=(HANDLE_PIVOT_X, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="sleeve_1",
    )

    knob = model.part("knob")
    knob.visual(
        Cylinder(radius=0.0036, length=0.008),
        origin=Origin(xyz=(-0.004, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_gray,
        name="shaft",
    )
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.019,
                0.011,
                body_style="cylindrical",
                edge_radius=0.0012,
                crown_radius=0.0008,
                center=False,
            ),
            "control_knob",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, -math.pi / 2.0, 0.0)),
        material=knob_black,
        name="grip",
    )

    model.articulation(
        "stand_to_housing",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=housing,
        origin=Origin(xyz=(0.0, PIVOT_Y, PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.8,
            lower=-0.55,
            upper=0.70,
        ),
    )
    model.articulation(
        "housing_to_blade",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=blade,
        origin=Origin(xyz=(0.0, 0.006, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=25.0),
    )
    model.articulation(
        "housing_to_handle",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=handle,
        origin=Origin(xyz=(0.0, HANDLE_PIVOT_Y, HANDLE_PIVOT_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=1.40,
        ),
    )
    model.articulation(
        "housing_to_knob",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=knob,
        origin=Origin(xyz=(KNOB_X, KNOB_Y, KNOB_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stand = object_model.get_part("stand")
    housing = object_model.get_part("housing")
    blade = object_model.get_part("blade")
    handle = object_model.get_part("handle")
    knob = object_model.get_part("knob")

    tilt_joint = object_model.get_articulation("stand_to_housing")
    handle_joint = object_model.get_articulation("housing_to_handle")

    ctx.expect_within(
        blade,
        housing,
        axes="xz",
        margin=0.016,
        name="blade stays centered within housing",
    )
    ctx.expect_overlap(
        blade,
        housing,
        axes="y",
        min_overlap=0.010,
        name="blade remains within housing depth",
    )

    housing_pos = ctx.part_world_position(housing)
    knob_pos = ctx.part_world_position(knob)
    ctx.check(
        "knob is mounted on the left side",
        housing_pos is not None
        and knob_pos is not None
        and knob_pos[0] < housing_pos[0] - 0.09
        and abs(knob_pos[2] - housing_pos[2]) < 0.07,
        details=f"housing={housing_pos}, knob={knob_pos}",
    )

    rest_housing_aabb = ctx.part_world_aabb(housing)
    if tilt_joint.motion_limits is not None and tilt_joint.motion_limits.upper is not None:
        rest_front = ctx.part_element_world_aabb(housing, elem="front_badge")
        with ctx.pose({tilt_joint: tilt_joint.motion_limits.upper}):
            tilted_housing_aabb = ctx.part_world_aabb(housing)
            tilted_front = ctx.part_element_world_aabb(housing, elem="front_badge")
        ctx.check(
            "housing tilts upward on the stand pivots",
            rest_front is not None
            and tilted_front is not None
            and tilted_housing_aabb is not None
            and ((tilted_front[0][2] + tilted_front[1][2]) * 0.5) > ((rest_front[0][2] + rest_front[1][2]) * 0.5) + 0.020,
            details=f"rest_front={rest_front}, tilted_front={tilted_front}, rest={rest_housing_aabb}, tilted={tilted_housing_aabb}",
        )

    rest_handle_loop = ctx.part_element_world_aabb(handle, elem="loop")
    if handle_joint.motion_limits is not None and handle_joint.motion_limits.upper is not None:
        with ctx.pose({handle_joint: handle_joint.motion_limits.upper}):
            raised_handle_loop = ctx.part_element_world_aabb(handle, elem="loop")
        ctx.check(
            "rear handle folds upward",
            rest_handle_loop is not None
            and raised_handle_loop is not None
            and ((raised_handle_loop[0][2] + raised_handle_loop[1][2]) * 0.5)
            > ((rest_handle_loop[0][2] + rest_handle_loop[1][2]) * 0.5) + 0.045,
            details=f"rest_loop={rest_handle_loop}, raised_loop={raised_handle_loop}",
        )

    stand_aabb = ctx.part_world_aabb(stand)
    ctx.check(
        "housing is supported above the tabletop stand",
        stand_aabb is not None
        and rest_housing_aabb is not None
        and rest_housing_aabb[0][2] > stand_aabb[0][2] + 0.012,
        details=f"stand={stand_aabb}, housing={rest_housing_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
