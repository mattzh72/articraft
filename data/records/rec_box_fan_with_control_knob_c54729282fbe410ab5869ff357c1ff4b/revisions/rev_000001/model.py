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
    KnobGrip,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _add_x_tube(part, *, radius: float, length: float, xyz, material, name: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _add_y_tube(part, *, radius: float, length: float, xyz, material, name: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _add_z_tube(part, *, radius: float, length: float, xyz, material, name: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="box_fan_floor_stand")

    stand_steel = model.material("stand_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    housing_paint = model.material("housing_paint", rgba=(0.25, 0.28, 0.31, 1.0))
    grille_dark = model.material("grille_dark", rgba=(0.16, 0.17, 0.18, 1.0))
    rotor_black = model.material("rotor_black", rgba=(0.11, 0.11, 0.12, 1.0))
    knob_black = model.material("knob_black", rgba=(0.08, 0.08, 0.09, 1.0))

    stand = model.part("stand")
    _add_x_tube(
        stand,
        radius=0.018,
        length=0.44,
        xyz=(0.0, 0.16, 0.018),
        material=stand_steel,
        name="front_foot",
    )
    _add_x_tube(
        stand,
        radius=0.018,
        length=0.32,
        xyz=(0.0, -0.16, 0.018),
        material=stand_steel,
        name="rear_foot",
    )
    _add_y_tube(
        stand,
        radius=0.018,
        length=0.32,
        xyz=(0.0, 0.0, 0.018),
        material=stand_steel,
        name="base_spine",
    )
    _add_z_tube(
        stand,
        radius=0.018,
        length=0.664,
        xyz=(0.0, -0.13, 0.35),
        material=stand_steel,
        name="mast",
    )
    _add_x_tube(
        stand,
        radius=0.015,
        length=0.556,
        xyz=(0.0, -0.13, 0.682),
        material=stand_steel,
        name="yoke_crossbar",
    )
    for side, x_pos in (("0", -0.278), ("1", 0.278)):
        _add_y_tube(
            stand,
            radius=0.015,
            length=0.13,
            xyz=(x_pos, -0.065, 0.682),
            material=stand_steel,
            name=f"arm_{side}",
        )
        _add_x_tube(
            stand,
            radius=0.018,
            length=0.028,
            xyz=(x_pos, 0.0, 0.682),
            material=stand_steel,
            name=f"pivot_cap_{side}",
        )

    housing = model.part("housing")
    housing.visual(
        Box((0.024, 0.17, 0.46)),
        origin=Origin(xyz=(-0.218, 0.0, 0.0)),
        material=housing_paint,
        name="left_wall",
    )
    housing.visual(
        Box((0.024, 0.17, 0.46)),
        origin=Origin(xyz=(0.218, 0.0, 0.0)),
        material=housing_paint,
        name="right_wall",
    )
    housing.visual(
        Box((0.416, 0.17, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.218)),
        material=housing_paint,
        name="top_wall",
    )
    housing.visual(
        Box((0.416, 0.17, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, -0.218)),
        material=housing_paint,
        name="bottom_wall",
    )
    housing.visual(
        Box((0.11, 0.010, 0.060)),
        origin=Origin(xyz=(0.175, 0.080, -0.182)),
        material=housing_paint,
        name="control_panel",
    )
    for index, x_pos in enumerate((-0.103, 0.0, 0.103)):
        housing.visual(
            Box((0.012, 0.008, 0.412)),
            origin=Origin(xyz=(x_pos, 0.079, 0.0)),
            material=grille_dark,
            name=f"front_vertical_{index}",
        )
    for index, z_pos in enumerate((-0.103, 0.0, 0.103)):
        housing.visual(
            Box((0.412, 0.008, 0.012)),
            origin=Origin(xyz=(0.0, 0.079, z_pos)),
            material=grille_dark,
            name=f"front_horizontal_{index}",
        )
    housing.visual(
        Box((0.412, 0.012, 0.024)),
        origin=Origin(xyz=(0.0, -0.074, 0.0)),
        material=grille_dark,
        name="rear_horizontal",
    )
    housing.visual(
        Box((0.024, 0.012, 0.412)),
        origin=Origin(xyz=(0.0, -0.074, 0.0)),
        material=grille_dark,
        name="rear_vertical",
    )
    _add_y_tube(
        housing,
        radius=0.055,
        length=0.078,
        xyz=(0.0, -0.040, 0.0),
        material=grille_dark,
        name="motor_can",
    )
    _add_y_tube(
        housing,
        radius=0.018,
        length=0.010,
        xyz=(0.178, 0.080, -0.182),
        material=grille_dark,
        name="knob_boss",
    )
    _add_x_tube(
        housing,
        radius=0.024,
        length=0.034,
        xyz=(-0.247, 0.0, 0.0),
        material=grille_dark,
        name="pivot_boss_0",
    )
    _add_x_tube(
        housing,
        radius=0.024,
        length=0.034,
        xyz=(0.247, 0.0, 0.0),
        material=grille_dark,
        name="pivot_boss_1",
    )

    rotor = model.part("rotor")
    rotor.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                0.185,
                0.058,
                5,
                thickness=0.040,
                blade_pitch_deg=33.0,
                blade_sweep_deg=26.0,
                blade=FanRotorBlade(
                    shape="scimitar",
                    tip_pitch_deg=14.0,
                    camber=0.16,
                    tip_clearance=0.005,
                ),
                hub=FanRotorHub(
                    style="domed",
                    rear_collar_height=0.012,
                    rear_collar_radius=0.040,
                    bore_diameter=0.012,
                ),
            ),
            "box_fan_rotor",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rotor_black,
        name="blade_set",
    )
    _add_y_tube(
        rotor,
        radius=0.010,
        length=0.036,
        xyz=(0.0, -0.018, 0.0),
        material=rotor_black,
        name="shaft_collar",
    )

    knob = model.part("knob")
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.034,
                0.020,
                body_style="cylindrical",
                edge_radius=0.0012,
                grip=KnobGrip(
                    style="knurled",
                    count=30,
                    depth=0.0008,
                    helix_angle_deg=24.0,
                ),
                center=False,
            ),
            "box_fan_knob",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=knob_black,
        name="knob_shell",
    )

    model.articulation(
        "stand_to_housing",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=housing,
        origin=Origin(xyz=(0.0, 0.0, 0.682)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.2,
            lower=-math.radians(20.0),
            upper=math.radians(50.0),
        ),
    )
    model.articulation(
        "housing_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.035, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=30.0),
    )
    model.articulation(
        "housing_to_knob",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=knob,
        origin=Origin(xyz=(0.178, 0.085, -0.182)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    rotor = object_model.get_part("rotor")
    knob = object_model.get_part("knob")
    housing_tilt = object_model.get_articulation("stand_to_housing")
    rotor_spin = object_model.get_articulation("housing_to_rotor")
    knob_spin = object_model.get_articulation("housing_to_knob")

    ctx.check(
        "housing_tilt_axis_is_horizontal",
        tuple(float(v) for v in housing_tilt.axis) == (1.0, 0.0, 0.0),
        details=f"axis={housing_tilt.axis!r}",
    )
    ctx.check(
        "rotor_joint_is_continuous",
        rotor_spin.articulation_type == ArticulationType.CONTINUOUS and tuple(float(v) for v in rotor_spin.axis) == (0.0, 1.0, 0.0),
        details=f"type={rotor_spin.articulation_type!r}, axis={rotor_spin.axis!r}",
    )
    ctx.check(
        "knob_joint_is_continuous",
        knob_spin.articulation_type == ArticulationType.CONTINUOUS and tuple(float(v) for v in knob_spin.axis) == (0.0, 1.0, 0.0),
        details=f"type={knob_spin.articulation_type!r}, axis={knob_spin.axis!r}",
    )
    ctx.expect_origin_distance(
        housing,
        rotor,
        axes="xz",
        max_dist=0.002,
        name="rotor stays centered in the housing opening",
    )
    ctx.expect_overlap(
        rotor,
        housing,
        axes="xz",
        min_overlap=0.34,
        name="rotor substantially fills the square housing opening",
    )
    ctx.expect_origin_gap(
        knob,
        housing,
        axis="x",
        min_gap=0.15,
        max_gap=0.21,
        name="knob sits on the right side of the housing",
    )
    ctx.expect_origin_gap(
        housing,
        knob,
        axis="z",
        min_gap=0.15,
        max_gap=0.21,
        name="knob sits near the lower corner of the housing",
    )

    rest_knob_pos = ctx.part_world_position(knob)
    upper_tilt = housing_tilt.motion_limits.upper if housing_tilt.motion_limits is not None else None
    with ctx.pose({housing_tilt: upper_tilt if upper_tilt is not None else math.radians(50.0)}):
        tilted_knob_pos = ctx.part_world_position(knob)
    ctx.check(
        "housing_tilts_upward_on_side_pivots",
        rest_knob_pos is not None
        and tilted_knob_pos is not None
        and tilted_knob_pos[2] > rest_knob_pos[2] + 0.05,
        details=f"rest={rest_knob_pos}, tilted={tilted_knob_pos}",
    )

    return ctx.report()


object_model = build_object_model()
