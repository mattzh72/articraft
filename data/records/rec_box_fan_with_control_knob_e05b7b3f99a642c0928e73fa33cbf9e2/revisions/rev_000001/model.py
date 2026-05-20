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
    MotionProperties,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desk_box_fan")

    plastic = model.material("warm_gray_plastic", rgba=(0.74, 0.74, 0.70, 1.0))
    dark = model.material("dark_control_plastic", rgba=(0.06, 0.065, 0.07, 1.0))
    grille_mat = model.material("pale_safety_grille", rgba=(0.86, 0.87, 0.84, 1.0))
    blade_mat = model.material("smoky_blue_blades", rgba=(0.20, 0.36, 0.55, 0.72))
    rubber = model.material("matte_rubber", rgba=(0.025, 0.025, 0.025, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.16, 0.50, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=plastic,
        name="base_foot",
    )
    base.visual(
        Box((0.055, 0.030, 0.172)),
        origin=Origin(xyz=(0.0, 0.229, 0.109)),
        material=plastic,
        name="ear_0",
    )
    base.visual(
        Box((0.055, 0.030, 0.172)),
        origin=Origin(xyz=(0.0, -0.229, 0.109)),
        material=plastic,
        name="ear_1",
    )
    base.visual(
        Box((0.020, 0.455, 0.018)),
        origin=Origin(xyz=(-0.055, 0.0, 0.033)),
        material=plastic,
        name="rear_stiffener",
    )
    base.visual(
        Box((0.028, 0.050, 0.006)),
        origin=Origin(xyz=(-0.052, 0.185, 0.003)),
        material=rubber,
        name="foot_0",
    )
    base.visual(
        Box((0.028, 0.050, 0.006)),
        origin=Origin(xyz=(-0.052, -0.185, 0.003)),
        material=rubber,
        name="foot_1",
    )

    housing = model.part("housing")
    # The housing frame is authored in its own tilt frame.  The frame origin is
    # the horizontal trunnion axis, slightly above the bottom of the square box.
    housing.visual(
        Box((0.180, 0.405, 0.036)),
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
        material=plastic,
        name="top_rail",
    )
    housing.visual(
        Box((0.180, 0.405, 0.036)),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=plastic,
        name="bottom_rail",
    )
    housing.visual(
        Box((0.180, 0.036, 0.365)),
        origin=Origin(xyz=(0.0, 0.184, 0.135)),
        material=plastic,
        name="side_rail_0",
    )
    housing.visual(
        Box((0.180, 0.036, 0.365)),
        origin=Origin(xyz=(0.0, -0.184, 0.135)),
        material=plastic,
        name="side_rail_1",
    )
    housing.visual(
        Cylinder(radius=0.022, length=0.024),
        origin=Origin(xyz=(0.0, 0.202, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=plastic,
        name="pivot_0",
    )
    housing.visual(
        Cylinder(radius=0.022, length=0.024),
        origin=Origin(xyz=(0.0, -0.202, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=plastic,
        name="pivot_1",
    )

    grille = PerforatedPanelGeometry(
        (0.322, 0.322),
        0.004,
        hole_diameter=0.017,
        pitch=(0.029, 0.029),
        frame=0.018,
        corner_radius=0.006,
        stagger=True,
    )
    housing.visual(
        mesh_from_geometry(grille, "front_grille"),
        origin=Origin(xyz=(-0.082, 0.0, 0.135), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grille_mat,
        name="front_grille",
    )
    housing.visual(
        mesh_from_geometry(grille, "rear_grille"),
        origin=Origin(xyz=(0.082, 0.0, 0.135), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grille_mat,
        name="rear_grille",
    )
    housing.visual(
        Cylinder(radius=0.055, length=0.050),
        origin=Origin(xyz=(0.058, 0.0, 0.135), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="motor_can",
    )
    housing.visual(
        Box((0.012, 0.012, 0.096)),
        origin=Origin(xyz=(0.058, 0.0, 0.236)),
        material=dark,
        name="motor_strut_0",
    )
    housing.visual(
        Box((0.012, 0.012, 0.096)),
        origin=Origin(xyz=(0.058, 0.0, 0.034)),
        material=dark,
        name="motor_strut_1",
    )
    housing.visual(
        Box((0.012, 0.114, 0.012)),
        origin=Origin(xyz=(0.058, 0.111, 0.135)),
        material=dark,
        name="motor_strut_2",
    )
    housing.visual(
        Box((0.012, 0.114, 0.012)),
        origin=Origin(xyz=(0.058, -0.111, 0.135)),
        material=dark,
        name="motor_strut_3",
    )
    housing.visual(
        Box((0.060, 0.006, 0.115)),
        origin=Origin(xyz=(0.018, 0.203, 0.224)),
        material=dark,
        name="control_plate",
    )

    blade = model.part("blade")
    rotor_mesh = mesh_from_geometry(
        FanRotorGeometry(
            0.132,
            0.035,
            5,
            thickness=0.026,
            blade_pitch_deg=32.0,
            blade_sweep_deg=24.0,
            blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=14.0, camber=0.13, tip_clearance=0.004),
            hub=FanRotorHub(style="spinner", rear_collar_height=0.006, rear_collar_radius=0.026),
        ),
        "fan_blade_rotor",
    )
    blade.visual(
        rotor_mesh,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blade_mat,
        name="rotor",
    )
    blade.visual(
        Cylinder(radius=0.018, length=0.049),
        origin=Origin(xyz=(0.0345, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="rear_collar",
    )

    knob = model.part("knob")
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.044,
            0.026,
            body_style="faceted",
            top_diameter=0.034,
            base_diameter=0.047,
            edge_radius=0.0012,
            grip=KnobGrip(style="ribbed", count=18, depth=0.0011, width=0.0016),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
            center=False,
        ),
        "side_control_knob",
    )
    knob.visual(
        knob_mesh,
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="knob_cap",
    )

    model.articulation(
        "base_to_housing",
        ArticulationType.REVOLUTE,
        parent=base,
        child=housing,
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=-0.38, upper=0.38),
        motion_properties=MotionProperties(damping=0.15, friction=0.08),
    )
    model.articulation(
        "housing_to_blade",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=blade,
        origin=Origin(xyz=(-0.026, 0.0, 0.135)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=60.0),
        motion_properties=MotionProperties(damping=0.01, friction=0.0),
    )
    model.articulation(
        "housing_to_knob",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=knob,
        origin=Origin(xyz=(0.018, 0.206, 0.224)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=6.0),
        motion_properties=MotionProperties(damping=0.02, friction=0.03),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    housing = object_model.get_part("housing")
    blade = object_model.get_part("blade")
    knob = object_model.get_part("knob")
    tilt = object_model.get_articulation("base_to_housing")
    spin = object_model.get_articulation("housing_to_blade")
    knob_spin = object_model.get_articulation("housing_to_knob")

    ctx.check(
        "housing tilts on a limited horizontal joint",
        tilt.articulation_type == ArticulationType.REVOLUTE
        and tilt.axis == (0.0, 1.0, 0.0)
        and tilt.motion_limits is not None
        and tilt.motion_limits.lower is not None
        and tilt.motion_limits.upper is not None
        and tilt.motion_limits.lower < 0.0 < tilt.motion_limits.upper,
        details=f"type={tilt.articulation_type}, axis={tilt.axis}, limits={tilt.motion_limits}",
    )
    ctx.check(
        "blade and side knob rotate continuously",
        spin.articulation_type == ArticulationType.CONTINUOUS
        and spin.axis == (1.0, 0.0, 0.0)
        and knob_spin.articulation_type == ArticulationType.CONTINUOUS
        and knob_spin.axis == (0.0, 1.0, 0.0),
        details=f"blade={spin.articulation_type}/{spin.axis}, knob={knob_spin.articulation_type}/{knob_spin.axis}",
    )

    ctx.expect_contact(
        housing,
        base,
        elem_a="pivot_0",
        elem_b="ear_0",
        contact_tol=0.0015,
        name="one pivot boss seats against a base ear",
    )
    ctx.expect_contact(
        housing,
        base,
        elem_a="pivot_1",
        elem_b="ear_1",
        contact_tol=0.0015,
        name="opposite pivot boss seats against a base ear",
    )
    ctx.expect_within(
        blade,
        housing,
        axes="yz",
        inner_elem="rotor",
        outer_elem="front_grille",
        margin=0.006,
        name="rotor fits inside the square guard opening",
    )
    ctx.expect_gap(
        blade,
        housing,
        axis="x",
        positive_elem="rotor",
        negative_elem="front_grille",
        min_gap=0.025,
        name="rotor clears the front safety grille",
    )
    ctx.expect_gap(
        housing,
        blade,
        axis="x",
        positive_elem="rear_grille",
        negative_elem="rotor",
        min_gap=0.035,
        name="rotor clears the rear safety grille",
    )
    ctx.expect_contact(
        knob,
        housing,
        elem_a="knob_cap",
        elem_b="control_plate",
        contact_tol=0.0015,
        name="rotary knob is seated on the side control panel",
    )

    rest_aabb = ctx.part_element_world_aabb(housing, elem="front_grille")
    rest_center_z = None
    if rest_aabb is not None:
        rest_center_z = (rest_aabb[0][2] + rest_aabb[1][2]) * 0.5
    with ctx.pose({tilt: tilt.motion_limits.upper}):
        up_aabb = ctx.part_element_world_aabb(housing, elem="front_grille")
    with ctx.pose({tilt: tilt.motion_limits.lower}):
        down_aabb = ctx.part_element_world_aabb(housing, elem="front_grille")
    up_center_z = None if up_aabb is None else (up_aabb[0][2] + up_aabb[1][2]) * 0.5
    down_center_z = None if down_aabb is None else (down_aabb[0][2] + down_aabb[1][2]) * 0.5
    ctx.check(
        "tilt joint angles the fan face up and down",
        rest_center_z is not None
        and up_center_z is not None
        and down_center_z is not None
        and up_center_z > rest_center_z + 0.015
        and down_center_z < rest_center_z - 0.015,
        details=f"down={down_center_z}, rest={rest_center_z}, up={up_center_z}",
    )

    return ctx.report()


object_model = build_object_model()
