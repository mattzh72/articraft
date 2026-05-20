from __future__ import annotations

import math

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
    mesh_from_geometry,
)


OUTER_WIDTH = 0.340
OUTER_HEIGHT = 0.360
BODY_DEPTH = 0.105
WALL = 0.018
FRONT_FACE = 0.008
REAR_GUARD = 0.006
SHELL_CENTER_Z = -0.030
SHELL_TOP = SHELL_CENTER_Z + OUTER_HEIGHT * 0.5
SHELL_BOTTOM = SHELL_CENTER_Z - OUTER_HEIGHT * 0.5

OPENING_WIDTH = 0.256
OPENING_HEIGHT = 0.240
OPENING_TOP = OPENING_HEIGHT * 0.5
OPENING_BOTTOM = -OPENING_HEIGHT * 0.5
INNER_WIDTH = OUTER_WIDTH - 2.0 * WALL
SIDE_BEZEL = (INNER_WIDTH - OPENING_WIDTH) * 0.5

PIVOT_Z = 0.225
STAND_SIDE_X = 0.188
STAND_FRAME_X = 0.195
PIVOT_BOSS_RADIUS = 0.010
PIVOT_BOSS_LENGTH = 0.014
PIVOT_BOSS_X = OUTER_WIDTH * 0.5 + PIVOT_BOSS_LENGTH * 0.5


def _x_cylinder(
    part,
    *,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material,
    name: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _y_cylinder(
    part,
    *,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material,
    name: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_box_fan")

    stand_finish = model.material("stand_finish", rgba=(0.72, 0.75, 0.78, 1.0))
    housing_finish = model.material("housing_finish", rgba=(0.86, 0.88, 0.90, 1.0))
    grille_finish = model.material("grille_finish", rgba=(0.74, 0.77, 0.80, 1.0))
    rotor_finish = model.material("rotor_finish", rgba=(0.48, 0.56, 0.70, 0.88))
    knob_finish = model.material("knob_finish", rgba=(0.18, 0.20, 0.22, 1.0))
    shaft_finish = model.material("shaft_finish", rgba=(0.35, 0.37, 0.39, 1.0))

    stand = model.part("stand")
    for index, x in enumerate((-STAND_FRAME_X, STAND_FRAME_X)):
        stand.visual(
            Box((0.022, 0.200, 0.014)),
            origin=Origin(xyz=(x, -0.005, 0.007)),
            material=stand_finish,
            name=f"foot_{index}",
        )
        stand.visual(
            Box((0.018, 0.028, 0.220)),
            origin=Origin(xyz=(x, 0.0, 0.115)),
            material=stand_finish,
            name=f"arm_{index}",
        )
        stand.visual(
            Box((0.018, 0.090, 0.070)),
            origin=Origin(xyz=(x, -0.045, 0.042)),
            material=stand_finish,
            name=f"brace_{index}",
        )

    stand.visual(
        Box((0.368, 0.018, 0.014)),
        origin=Origin(xyz=(0.0, -0.096, 0.007)),
        material=stand_finish,
        name="rear_crossbar",
    )
    stand.visual(
        Box((0.368, 0.018, 0.014)),
        origin=Origin(xyz=(0.0, 0.086, 0.007)),
        material=stand_finish,
        name="front_crossbar",
    )
    for index, x in enumerate((-STAND_SIDE_X, STAND_SIDE_X)):
        _x_cylinder(
            stand,
            radius=0.018,
            length=0.008,
            xyz=(x, 0.0, PIVOT_Z),
            material=stand_finish,
            name=f"pivot_plate_{index}",
        )

    housing = model.part("housing")
    for name, x in (
        ("left_wall", -OUTER_WIDTH * 0.5 + WALL * 0.5),
        ("right_wall", OUTER_WIDTH * 0.5 - WALL * 0.5),
    ):
        housing.visual(
            Box((WALL, BODY_DEPTH, OUTER_HEIGHT)),
            origin=Origin(xyz=(x, 0.0, SHELL_CENTER_Z)),
            material=housing_finish,
            name=name,
        )

    housing.visual(
        Box((INNER_WIDTH, BODY_DEPTH, WALL)),
        origin=Origin(xyz=(0.0, 0.0, SHELL_TOP - WALL * 0.5)),
        material=housing_finish,
        name="top_wall",
    )
    housing.visual(
        Box((INNER_WIDTH, BODY_DEPTH, WALL)),
        origin=Origin(xyz=(0.0, 0.0, SHELL_BOTTOM + WALL * 0.5)),
        material=housing_finish,
        name="bottom_wall",
    )

    front_y = BODY_DEPTH * 0.5 - FRONT_FACE * 0.5
    housing.visual(
        Box((INNER_WIDTH, FRONT_FACE, SHELL_TOP - OPENING_TOP)),
        origin=Origin(xyz=(0.0, front_y, (SHELL_TOP + OPENING_TOP) * 0.5)),
        material=housing_finish,
        name="front_top_frame",
    )
    housing.visual(
        Box((INNER_WIDTH, FRONT_FACE, OPENING_BOTTOM - SHELL_BOTTOM)),
        origin=Origin(xyz=(0.0, front_y, (OPENING_BOTTOM + SHELL_BOTTOM) * 0.5)),
        material=housing_finish,
        name="front_panel",
    )
    for name, x in (
        ("front_left_frame", -(OPENING_WIDTH * 0.5 + SIDE_BEZEL * 0.5)),
        ("front_right_frame", OPENING_WIDTH * 0.5 + SIDE_BEZEL * 0.5),
    ):
        housing.visual(
            Box((SIDE_BEZEL, FRONT_FACE, OPENING_HEIGHT)),
            origin=Origin(xyz=(x, front_y, 0.0)),
            material=housing_finish,
            name=name,
        )

    grille_y = BODY_DEPTH * 0.5 - FRONT_FACE
    for index, x in enumerate((-0.086, -0.043, 0.0, 0.043, 0.086)):
        housing.visual(
            Box((0.004, 0.004, OPENING_HEIGHT)),
            origin=Origin(xyz=(x, grille_y, 0.0)),
            material=grille_finish,
            name=f"front_grille_v_{index}",
        )
    for index, z in enumerate((-0.090, -0.045, 0.0, 0.045, 0.090)):
        housing.visual(
            Box((OPENING_WIDTH, 0.004, 0.004)),
            origin=Origin(xyz=(0.0, grille_y, z)),
            material=grille_finish,
            name=f"front_grille_h_{index}",
        )

    rear_y = -BODY_DEPTH * 0.5 + REAR_GUARD * 0.5
    for index, z in enumerate((-0.090, -0.030, 0.030, 0.090)):
        housing.visual(
            Box((INNER_WIDTH, REAR_GUARD, 0.006)),
            origin=Origin(xyz=(0.0, rear_y, z)),
            material=grille_finish,
            name=f"rear_guard_{index}",
        )
    housing.visual(
        Box((0.006, REAR_GUARD, OPENING_HEIGHT)),
        origin=Origin(xyz=(0.0, rear_y, 0.0)),
        material=grille_finish,
        name="rear_guard_center",
    )

    _y_cylinder(
        housing,
        radius=0.036,
        length=0.036,
        xyz=(0.0, -0.036, 0.0),
        material=shaft_finish,
        name="motor_pod",
    )
    _y_cylinder(
        housing,
        radius=0.0045,
        length=0.044,
        xyz=(0.0, 0.0, 0.0),
        material=shaft_finish,
        name="shaft",
    )

    housing.visual(
        Box((0.116, 0.014, 0.010)),
        origin=Origin(xyz=(-0.094, -0.042, 0.0)),
        material=shaft_finish,
        name="motor_support_left",
    )
    housing.visual(
        Box((0.116, 0.014, 0.010)),
        origin=Origin(xyz=(0.094, -0.042, 0.0)),
        material=shaft_finish,
        name="motor_support_right",
    )
    housing.visual(
        Box((0.010, 0.014, 0.096)),
        origin=Origin(xyz=(0.0, -0.042, 0.084)),
        material=shaft_finish,
        name="motor_support_top",
    )
    housing.visual(
        Box((0.010, 0.014, 0.156)),
        origin=Origin(xyz=(0.0, -0.042, -0.114)),
        material=shaft_finish,
        name="motor_support_bottom",
    )

    for index, x in enumerate((-PIVOT_BOSS_X, PIVOT_BOSS_X)):
        _x_cylinder(
            housing,
            radius=PIVOT_BOSS_RADIUS,
            length=PIVOT_BOSS_LENGTH,
            xyz=(x, 0.0, 0.0),
            material=housing_finish,
            name=f"pivot_boss_{index}",
        )

    rotor = model.part("rotor")
    rotor.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                0.112,
                0.028,
                5,
                thickness=0.014,
                blade_pitch_deg=32.0,
                blade_sweep_deg=16.0,
                blade=FanRotorBlade(shape="broad", camber=0.10),
                hub=FanRotorHub(style="domed", bore_diameter=0.009),
            ),
            "box_fan_rotor",
        ),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rotor_finish,
        name="rotor",
    )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.046,
                0.020,
                body_style="skirted",
                top_diameter=0.032,
                base_diameter=0.050,
                crown_radius=0.004,
                edge_radius=0.0015,
                center=False,
            ),
            "box_fan_selector_knob",
        ),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=knob_finish,
        name="knob",
    )
    selector_knob.visual(
        Box((0.003, 0.0015, 0.010)),
        origin=Origin(
            xyz=(0.0, 0.017, 0.014),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=grille_finish,
        name="indicator",
    )

    model.articulation(
        "stand_to_housing",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=housing,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=-0.35,
            upper=0.65,
        ),
    )
    model.articulation(
        "housing_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=rotor,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=25.0),
    )
    model.articulation(
        "housing_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=selector_knob,
        origin=Origin(xyz=(0.0, BODY_DEPTH * 0.5, -0.165)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.08, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    rotor = object_model.get_part("rotor")
    selector_knob = object_model.get_part("selector_knob")
    tilt = object_model.get_articulation("stand_to_housing")
    rotor_joint = object_model.get_articulation("housing_to_rotor")
    knob_joint = object_model.get_articulation("housing_to_selector_knob")

    ctx.allow_overlap(
        housing,
        rotor,
        elem_a="shaft",
        elem_b="rotor",
        reason="The impeller hub is intentionally captured on the motor shaft.",
    )

    ctx.expect_within(
        rotor,
        housing,
        axes="xz",
        margin=0.030,
        name="rotor stays within housing silhouette",
    )
    ctx.expect_origin_gap(
        rotor,
        selector_knob,
        axis="z",
        min_gap=0.130,
        max_gap=0.190,
        name="selector knob sits below rotor center",
    )
    ctx.expect_origin_gap(
        selector_knob,
        housing,
        axis="y",
        min_gap=0.048,
        max_gap=0.058,
        name="selector knob protrudes from front panel",
    )
    ctx.check(
        "housing tilt joint is revolute",
        tilt.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={tilt.articulation_type!r}",
    )
    ctx.check(
        "rotor spins continuously",
        rotor_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={rotor_joint.articulation_type!r}",
    )
    ctx.check(
        "selector knob spins continuously",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={knob_joint.articulation_type!r}",
    )

    rest_aabb = ctx.part_element_world_aabb(housing, elem="front_panel")
    rest_center_z = None
    if rest_aabb is not None:
        rest_center_z = (float(rest_aabb[0][2]) + float(rest_aabb[1][2])) * 0.5

    upper = tilt.motion_limits.upper if tilt.motion_limits is not None else None
    raised_center_z = None
    if upper is not None:
        with ctx.pose({tilt: upper}):
            raised_aabb = ctx.part_element_world_aabb(housing, elem="front_panel")
            if raised_aabb is not None:
                raised_center_z = (float(raised_aabb[0][2]) + float(raised_aabb[1][2])) * 0.5

    ctx.check(
        "positive tilt raises the front of the housing",
        rest_center_z is not None
        and raised_center_z is not None
        and raised_center_z > rest_center_z + 0.020,
        details=f"rest_z={rest_center_z}, raised_z={raised_center_z}",
    )

    return ctx.report()


object_model = build_object_model()
