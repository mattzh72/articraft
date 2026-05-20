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
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="table_tilt_stand_box_fan")

    housing_mat = model.material("warm_white_plastic", rgba=(0.82, 0.84, 0.80, 1.0))
    grille_mat = model.material("pale_gray_grille", rgba=(0.72, 0.75, 0.73, 1.0))
    dark_mat = model.material("graphite_stand", rgba=(0.05, 0.055, 0.06, 1.0))
    rubber_mat = model.material("matte_black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    blade_mat = model.material("translucent_blue_blade", rgba=(0.42, 0.64, 0.84, 0.82))

    cyl_x = (0.0, math.pi / 2.0, 0.0)
    cyl_y = (-math.pi / 2.0, 0.0, 0.0)

    stand = model.part("stand_base")
    stand.visual(
        Cylinder(radius=0.175, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark_mat,
        name="round_base",
    )
    stand.visual(
        Cylinder(radius=0.110, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=rubber_mat,
        name="rubber_foot",
    )
    stand.visual(
        Cylinder(radius=0.022, length=0.074),
        origin=Origin(xyz=(0.0, 0.0, 0.067)),
        material=dark_mat,
        name="center_post",
    )
    stand.visual(
        Cylinder(radius=0.018, length=0.390),
        origin=Origin(xyz=(0.0, 0.0, 0.096), rpy=cyl_x),
        material=dark_mat,
        name="lower_crossbar",
    )
    stand.visual(
        Box((0.025, 0.038, 0.255)),
        origin=Origin(xyz=(-0.195, 0.0, 0.2175)),
        material=dark_mat,
        name="stand_arm_0",
    )
    stand.visual(
        Cylinder(radius=0.036, length=0.021),
        origin=Origin(xyz=(-0.1905, 0.0, 0.340), rpy=cyl_x),
        material=dark_mat,
        name="pivot_disk_0",
    )
    stand.visual(
        Box((0.025, 0.038, 0.255)),
        origin=Origin(xyz=(0.195, 0.0, 0.2175)),
        material=dark_mat,
        name="stand_arm_1",
    )
    stand.visual(
        Cylinder(radius=0.036, length=0.021),
        origin=Origin(xyz=(0.1905, 0.0, 0.340), rpy=cyl_x),
        material=dark_mat,
        name="pivot_disk_1",
    )

    housing = model.part("housing")
    # A square box-fan frame, centered on the tilt axis.  The open center carries
    # separate perforated grilles so the rotor remains visibly inside the cage.
    housing.visual(
        Box((0.350, 0.110, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.165)),
        material=housing_mat,
        name="top_rail",
    )
    housing.visual(
        Box((0.350, 0.110, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, -0.165)),
        material=housing_mat,
        name="bottom_rail",
    )
    housing.visual(
        Box((0.024, 0.110, 0.330)),
        origin=Origin(xyz=(-0.163, 0.0, 0.0)),
        material=housing_mat,
        name="side_rail_0",
    )
    housing.visual(
        Cylinder(radius=0.031, length=0.005),
        origin=Origin(xyz=(-0.1775, 0.0, 0.0), rpy=cyl_x),
        material=housing_mat,
        name="pivot_boss_0",
    )
    housing.visual(
        Box((0.024, 0.110, 0.330)),
        origin=Origin(xyz=(0.163, 0.0, 0.0)),
        material=housing_mat,
        name="side_rail_1",
    )
    housing.visual(
        Cylinder(radius=0.031, length=0.005),
        origin=Origin(xyz=(0.1775, 0.0, 0.0), rpy=cyl_x),
        material=housing_mat,
        name="pivot_boss_1",
    )

    front_grille = PerforatedPanelGeometry(
        (0.300, 0.300),
        0.004,
        hole_diameter=0.014,
        pitch=(0.026, 0.026),
        frame=0.012,
        corner_radius=0.010,
        stagger=True,
    )
    rear_grille = PerforatedPanelGeometry(
        (0.300, 0.300),
        0.004,
        hole_diameter=0.012,
        pitch=(0.024, 0.024),
        frame=0.012,
        corner_radius=0.010,
        stagger=True,
    )
    housing.visual(
        mesh_from_geometry(front_grille, "front_grille"),
        origin=Origin(xyz=(0.0, -0.053, 0.0), rpy=cyl_y),
        material=grille_mat,
        name="front_grille",
    )
    housing.visual(
        mesh_from_geometry(rear_grille, "rear_grille"),
        origin=Origin(xyz=(0.0, 0.053, 0.0), rpy=cyl_y),
        material=grille_mat,
        name="rear_grille",
    )
    housing.visual(
        Cylinder(radius=0.052, length=0.040),
        origin=Origin(xyz=(0.0, 0.026, 0.0), rpy=cyl_y),
        material=dark_mat,
        name="motor_pod",
    )
    housing.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=cyl_y),
        material=dark_mat,
        name="shaft_socket",
    )
    housing.visual(
        Box((0.318, 0.014, 0.014)),
        origin=Origin(xyz=(0.0, 0.026, 0.0)),
        material=dark_mat,
        name="motor_strut_x",
    )
    housing.visual(
        Box((0.014, 0.014, 0.318)),
        origin=Origin(xyz=(0.0, 0.026, 0.0)),
        material=dark_mat,
        name="motor_strut_z",
    )
    housing.visual(
        Cylinder(radius=0.027, length=0.008),
        origin=Origin(xyz=(0.179, -0.022, 0.105), rpy=cyl_x),
        material=housing_mat,
        name="knob_socket",
    )

    blade = model.part("blade")
    rotor = FanRotorGeometry(
        0.125,
        0.034,
        5,
        thickness=0.024,
        blade_pitch_deg=32.0,
        blade_sweep_deg=24.0,
        blade=FanRotorBlade(shape="broad", tip_pitch_deg=14.0, camber=0.18, tip_clearance=0.008),
        hub=FanRotorHub(style="spinner", rear_collar_height=0.005, rear_collar_radius=0.022),
    )
    blade.visual(
        mesh_from_geometry(rotor, "fan_blade"),
        origin=Origin(rpy=cyl_y),
        material=blade_mat,
        name="rotor",
    )

    knob = model.part("knob")
    dial = KnobGeometry(
        0.045,
        0.028,
        body_style="domed",
        top_diameter=0.038,
        edge_radius=0.0015,
        grip=KnobGrip(style="fluted", count=20, depth=0.0012),
        indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
        center=False,
    )
    knob.visual(
        mesh_from_geometry(dial, "side_control_knob"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_mat,
        name="dial",
    )

    model.articulation(
        "tilt_hinge",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=housing,
        origin=Origin(xyz=(0.0, 0.0, 0.340)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=-0.45, upper=0.45),
    )
    model.articulation(
        "blade_spin",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=blade,
        origin=Origin(xyz=(0.0, -0.018, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=45.0),
    )
    model.articulation(
        "knob_spin",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=knob,
        origin=Origin(xyz=(0.183, -0.022, 0.105)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand_base")
    housing = object_model.get_part("housing")
    blade = object_model.get_part("blade")
    knob = object_model.get_part("knob")
    tilt = object_model.get_articulation("tilt_hinge")
    blade_spin = object_model.get_articulation("blade_spin")
    knob_spin = object_model.get_articulation("knob_spin")

    ctx.check(
        "housing tilts on a horizontal hinge",
        tilt.articulation_type == ArticulationType.REVOLUTE
        and tuple(tilt.axis) == (1.0, 0.0, 0.0)
        and tilt.motion_limits is not None
        and tilt.motion_limits.lower < 0.0
        and tilt.motion_limits.upper > 0.0,
        details=f"type={tilt.articulation_type}, axis={tilt.axis}, limits={tilt.motion_limits}",
    )
    ctx.check(
        "fan blade has continuous spin",
        blade_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(blade_spin.axis) == (0.0, 1.0, 0.0),
        details=f"type={blade_spin.articulation_type}, axis={blade_spin.axis}",
    )
    ctx.check(
        "side knob has continuous spin",
        knob_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(knob_spin.axis) == (1.0, 0.0, 0.0),
        details=f"type={knob_spin.articulation_type}, axis={knob_spin.axis}",
    )

    ctx.expect_contact(
        housing,
        stand,
        elem_a="pivot_boss_1",
        elem_b="pivot_disk_1",
        contact_tol=0.0015,
        name="housing pivot boss seats against stand disk",
    )
    ctx.expect_contact(
        knob,
        housing,
        elem_a="dial",
        elem_b="knob_socket",
        contact_tol=0.0015,
        name="control knob seats on side socket",
    )
    ctx.expect_within(
        blade,
        housing,
        axes="xz",
        inner_elem="rotor",
        outer_elem="front_grille",
        margin=0.0,
        name="rotor fits inside square grille opening",
    )
    ctx.expect_gap(
        blade,
        housing,
        axis="y",
        positive_elem="rotor",
        negative_elem="front_grille",
        min_gap=0.010,
        name="blade clears the front grille",
    )

    rest_aabb = ctx.part_element_world_aabb(housing, elem="front_grille")
    with ctx.pose({tilt: 0.35}):
        tilted_aabb = ctx.part_element_world_aabb(housing, elem="front_grille")
    ctx.check(
        "tilt pose moves the square housing cage",
        rest_aabb is not None
        and tilted_aabb is not None
        and abs(tilted_aabb[0][1] - rest_aabb[0][1]) > 0.020,
        details=f"rest={rest_aabb}, tilted={tilted_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
