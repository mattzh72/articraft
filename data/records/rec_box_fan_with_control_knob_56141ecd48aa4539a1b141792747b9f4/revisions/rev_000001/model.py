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
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


HOUSING_SIZE = 0.34
HOUSING_DEPTH = 0.12
HOUSING_WALL = 0.022
PIVOT_HEIGHT = 0.18
ROTOR_RADIUS = 0.115


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="table_box_fan")

    stand_finish = model.material("stand_finish", rgba=(0.77, 0.79, 0.82, 1.0))
    housing_finish = model.material("housing_finish", rgba=(0.88, 0.89, 0.91, 1.0))
    guard_finish = model.material("guard_finish", rgba=(0.72, 0.75, 0.79, 1.0))
    rotor_finish = model.material("rotor_finish", rgba=(0.33, 0.36, 0.40, 1.0))
    knob_finish = model.material("knob_finish", rgba=(0.18, 0.19, 0.21, 1.0))
    axle_finish = model.material("axle_finish", rgba=(0.55, 0.57, 0.61, 1.0))

    stand = model.part("stand")
    stand.visual(
        Box((0.028, 0.210, 0.014)),
        origin=Origin(xyz=(-0.184, 0.0, 0.007)),
        material=stand_finish,
        name="left_skid",
    )
    stand.visual(
        Box((0.028, 0.210, 0.014)),
        origin=Origin(xyz=(0.184, 0.0, 0.007)),
        material=stand_finish,
        name="right_skid",
    )
    stand.visual(
        Box((0.340, 0.022, 0.012)),
        origin=Origin(xyz=(0.0, -0.092, 0.014)),
        material=stand_finish,
        name="front_tie",
    )
    stand.visual(
        Box((0.340, 0.022, 0.012)),
        origin=Origin(xyz=(0.0, 0.092, 0.014)),
        material=stand_finish,
        name="rear_tie",
    )
    stand.visual(
        Box((0.024, 0.022, 0.150)),
        origin=Origin(xyz=(-0.198, 0.0, 0.085)),
        material=stand_finish,
        name="left_arm",
    )
    stand.visual(
        Box((0.024, 0.022, 0.150)),
        origin=Origin(xyz=(0.198, 0.0, 0.085)),
        material=stand_finish,
        name="right_arm",
    )
    stand.visual(
        Cylinder(radius=0.024, length=0.018),
        origin=Origin(xyz=(-0.195, 0.0, PIVOT_HEIGHT), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stand_finish,
        name="left_hinge_cap",
    )
    stand.visual(
        Cylinder(radius=0.024, length=0.018),
        origin=Origin(xyz=(0.195, 0.0, PIVOT_HEIGHT), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stand_finish,
        name="right_hinge_cap",
    )

    housing = model.part("housing")
    housing.visual(
        Box((HOUSING_SIZE, HOUSING_DEPTH, HOUSING_WALL)),
        origin=Origin(xyz=(0.0, 0.0, HOUSING_SIZE / 2.0 - HOUSING_WALL / 2.0)),
        material=housing_finish,
        name="top_shell",
    )
    housing.visual(
        Box((HOUSING_SIZE, HOUSING_DEPTH, HOUSING_WALL)),
        origin=Origin(xyz=(0.0, 0.0, -HOUSING_SIZE / 2.0 + HOUSING_WALL / 2.0)),
        material=housing_finish,
        name="bottom_shell",
    )
    housing.visual(
        Box((HOUSING_WALL, HOUSING_DEPTH, HOUSING_SIZE - 2.0 * HOUSING_WALL)),
        origin=Origin(xyz=(-HOUSING_SIZE / 2.0 + HOUSING_WALL / 2.0, 0.0, 0.0)),
        material=housing_finish,
        name="left_shell",
    )
    housing.visual(
        Box((HOUSING_WALL, HOUSING_DEPTH, HOUSING_SIZE - 2.0 * HOUSING_WALL)),
        origin=Origin(xyz=(HOUSING_SIZE / 2.0 - HOUSING_WALL / 2.0, 0.0, 0.0)),
        material=housing_finish,
        name="right_shell",
    )

    bezel_depth = 0.010
    bezel_face = HOUSING_WALL + 0.010
    bezel_side = HOUSING_WALL + 0.010
    front_y = -HOUSING_DEPTH / 2.0 + bezel_depth / 2.0
    rear_y = HOUSING_DEPTH / 2.0 - 0.008 / 2.0
    housing.visual(
        Box((HOUSING_SIZE, bezel_depth, bezel_face)),
        origin=Origin(xyz=(0.0, front_y, HOUSING_SIZE / 2.0 - bezel_face / 2.0)),
        material=housing_finish,
        name="front_top_bezel",
    )
    housing.visual(
        Box((HOUSING_SIZE, bezel_depth, bezel_face)),
        origin=Origin(xyz=(0.0, front_y, -HOUSING_SIZE / 2.0 + bezel_face / 2.0)),
        material=housing_finish,
        name="front_bottom_bezel",
    )
    housing.visual(
        Box((bezel_side, bezel_depth, HOUSING_SIZE - 2.0 * bezel_face)),
        origin=Origin(xyz=(-HOUSING_SIZE / 2.0 + bezel_side / 2.0, front_y, 0.0)),
        material=housing_finish,
        name="front_left_bezel",
    )
    housing.visual(
        Box((bezel_side, bezel_depth, HOUSING_SIZE - 2.0 * bezel_face)),
        origin=Origin(xyz=(HOUSING_SIZE / 2.0 - bezel_side / 2.0, front_y, 0.0)),
        material=housing_finish,
        name="front_right_bezel",
    )
    housing.visual(
        Box((HOUSING_SIZE - 0.010, 0.008, HOUSING_WALL)),
        origin=Origin(xyz=(0.0, rear_y, HOUSING_SIZE / 2.0 - HOUSING_WALL / 2.0)),
        material=housing_finish,
        name="rear_top_rim",
    )
    housing.visual(
        Box((HOUSING_SIZE - 0.010, 0.008, HOUSING_WALL)),
        origin=Origin(xyz=(0.0, rear_y, -HOUSING_SIZE / 2.0 + HOUSING_WALL / 2.0)),
        material=housing_finish,
        name="rear_bottom_rim",
    )
    housing.visual(
        Box((HOUSING_WALL, 0.008, HOUSING_SIZE - 2.0 * HOUSING_WALL)),
        origin=Origin(xyz=(-HOUSING_SIZE / 2.0 + HOUSING_WALL / 2.0, rear_y, 0.0)),
        material=housing_finish,
        name="rear_left_rim",
    )
    housing.visual(
        Box((HOUSING_WALL, 0.008, HOUSING_SIZE - 2.0 * HOUSING_WALL)),
        origin=Origin(xyz=(HOUSING_SIZE / 2.0 - HOUSING_WALL / 2.0, rear_y, 0.0)),
        material=housing_finish,
        name="rear_right_rim",
    )

    guard_mesh = mesh_from_geometry(
        PerforatedPanelGeometry(
            (0.286, 0.286),
            0.0025,
            hole_diameter=0.010,
            pitch=(0.015, 0.015),
            frame=0.010,
            corner_radius=0.008,
            stagger=True,
        ),
        "box_fan_guard",
    )
    housing.visual(
        guard_mesh,
        origin=Origin(xyz=(0.0, -0.04875, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=guard_finish,
        name="front_guard",
    )
    housing.visual(
        guard_mesh,
        origin=Origin(xyz=(0.0, 0.04875, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=guard_finish,
        name="rear_guard",
    )

    housing.visual(
        Cylinder(radius=0.034, length=0.044),
        origin=Origin(xyz=(0.0, 0.032, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stand_finish,
        name="motor_can",
    )
    housing.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(0.0, 0.008, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=axle_finish,
        name="axle_bushing",
    )
    housing.visual(
        Box((0.296, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, 0.028, 0.0)),
        material=guard_finish,
        name="cross_strut_x",
    )
    housing.visual(
        Box((0.012, 0.010, 0.296)),
        origin=Origin(xyz=(0.0, 0.028, 0.0)),
        material=guard_finish,
        name="cross_strut_z",
    )
    housing.visual(
        Cylinder(radius=0.022, length=0.016),
        origin=Origin(xyz=(-0.176, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=housing_finish,
        name="left_hinge_boss",
    )
    housing.visual(
        Cylinder(radius=0.022, length=0.016),
        origin=Origin(xyz=(0.176, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=housing_finish,
        name="right_hinge_boss",
    )
    housing.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(xyz=(0.167, 0.018, 0.105), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=housing_finish,
        name="knob_collar",
    )

    rotor = model.part("rotor")
    rotor.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                ROTOR_RADIUS,
                0.030,
                5,
                thickness=0.016,
                blade_pitch_deg=30.0,
                blade_sweep_deg=18.0,
                blade=FanRotorBlade(shape="broad", camber=0.10),
                hub=FanRotorHub(style="spinner", bore_diameter=0.006),
            ),
            "box_fan_rotor",
        ),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rotor_finish,
        name="rotor",
    )
    rotor.visual(
        Cylinder(radius=0.009, length=0.018),
        origin=Origin(xyz=(0.0, -0.004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=axle_finish,
        name="shaft_stub",
    )

    knob = model.part("knob")
    knob.visual(
        Cylinder(radius=0.005, length=0.010),
        origin=Origin(xyz=(0.005, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=axle_finish,
        name="shaft",
    )
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.040,
                0.018,
                body_style="skirted",
                top_diameter=0.032,
                edge_radius=0.0015,
                grip=KnobGrip(style="fluted", count=16, depth=0.0012),
                center=False,
            ),
            "box_fan_speed_knob",
        ),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_finish,
        name="knob",
    )

    model.articulation(
        "stand_to_housing",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=housing,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_HEIGHT)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.6,
            lower=math.radians(-20.0),
            upper=math.radians(32.0),
        ),
    )
    model.articulation(
        "housing_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=rotor,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=24.0),
    )
    model.articulation(
        "housing_to_knob",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=knob,
        origin=Origin(xyz=(0.172, 0.018, 0.105)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stand = object_model.get_part("stand")
    housing = object_model.get_part("housing")
    rotor = object_model.get_part("rotor")
    knob = object_model.get_part("knob")
    tilt = object_model.get_articulation("stand_to_housing")
    rotor_joint = object_model.get_articulation("housing_to_rotor")
    knob_joint = object_model.get_articulation("housing_to_knob")

    ctx.check(
        "housing uses tilt hinge",
        tilt.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={tilt.articulation_type!r}",
    )
    ctx.check(
        "rotor spins continuously",
        rotor_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={rotor_joint.articulation_type!r}",
    )
    ctx.check(
        "speed knob spins continuously",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={knob_joint.articulation_type!r}",
    )

    ctx.expect_within(
        rotor,
        housing,
        axes="xz",
        inner_elem="rotor",
        outer_elem="front_guard",
        margin=0.008,
        name="rotor stays within front guard footprint",
    )
    ctx.expect_contact(
        knob,
        housing,
        elem_a="shaft",
        elem_b="knob_collar",
        name="speed knob shaft seats on collar",
    )
    ctx.expect_origin_gap(
        knob,
        housing,
        axis="x",
        min_gap=0.15,
        name="speed knob sits on the side wall",
    )

    def element_center_z(part_name: str, elem_name: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part_name, elem=elem_name)
        if aabb is None:
            return None
        mins, maxs = aabb
        return float(mins[2] + maxs[2]) * 0.5

    rest_front_z = element_center_z("housing", "front_guard")
    with ctx.pose({tilt: math.radians(22.0)}):
        ctx.expect_within(
            rotor,
            housing,
            axes="xz",
            inner_elem="rotor",
            outer_elem="front_guard",
            margin=0.008,
            name="rotor stays guarded when tilted",
        )
        tilted_front_z = element_center_z("housing", "front_guard")

    ctx.check(
        "positive tilt raises the front guard",
        rest_front_z is not None and tilted_front_z is not None and tilted_front_z > rest_front_z + 0.012,
        details=f"rest_front_z={rest_front_z}, tilted_front_z={tilted_front_z}",
    )

    return ctx.report()


object_model = build_object_model()
