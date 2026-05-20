from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_WIDTH = 0.235
BASE_DEPTH = 0.182
BASE_THICKNESS = 0.018
STAND_OFFSET_Y = 0.010

OUTER_SLEEVE_HEIGHT = 0.128
OUTER_SLEEVE_BOTTOM = 0.015
OUTER_SLEEVE_TOP = OUTER_SLEEVE_BOTTOM + OUTER_SLEEVE_HEIGHT

MAST_TRAVEL = 0.080
HOUSING_WIDTH = 0.505
HOUSING_DEPTH = 0.044
HOUSING_HEIGHT = 0.313


def _pedestal_base_mesh():
    return mesh_from_cadquery(
        cq.Workplane("XY").ellipse(BASE_WIDTH * 0.5, BASE_DEPTH * 0.5).extrude(BASE_THICKNESS),
        "monitor_pedestal_base",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="business_monitor")

    base_black = model.material("base_black", rgba=(0.12, 0.12, 0.13, 1.0))
    stand_black = model.material("stand_black", rgba=(0.18, 0.18, 0.19, 1.0))
    housing_black = model.material("housing_black", rgba=(0.14, 0.14, 0.15, 1.0))
    trim_black = model.material("trim_black", rgba=(0.09, 0.09, 0.10, 1.0))
    glass_black = model.material("glass_black", rgba=(0.07, 0.10, 0.12, 0.72))
    control_black = model.material("control_black", rgba=(0.10, 0.10, 0.11, 1.0))

    base = model.part("base")
    base.visual(_pedestal_base_mesh(), material=base_black, name="pedestal")
    base.visual(
        Cylinder(radius=0.030, length=0.006),
        origin=Origin(xyz=(0.0, STAND_OFFSET_Y, BASE_THICKNESS + 0.003)),
        material=stand_black,
        name="swivel_pad",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_WIDTH, BASE_DEPTH, BASE_THICKNESS + 0.012)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
    )

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.026, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=stand_black,
        name="swivel_collar",
    )
    stand.visual(
        Box((0.007, 0.028, OUTER_SLEEVE_HEIGHT)),
        origin=Origin(xyz=(-0.0185, 0.0, OUTER_SLEEVE_BOTTOM + OUTER_SLEEVE_HEIGHT * 0.5)),
        material=stand_black,
        name="outer_side_0",
    )
    stand.visual(
        Box((0.007, 0.028, OUTER_SLEEVE_HEIGHT)),
        origin=Origin(xyz=(0.0185, 0.0, OUTER_SLEEVE_BOTTOM + OUTER_SLEEVE_HEIGHT * 0.5)),
        material=stand_black,
        name="outer_side_1",
    )
    stand.visual(
        Box((0.032, 0.004, OUTER_SLEEVE_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.012, OUTER_SLEEVE_BOTTOM + OUTER_SLEEVE_HEIGHT * 0.5)),
        material=stand_black,
        name="outer_face_0",
    )
    stand.visual(
        Box((0.032, 0.004, OUTER_SLEEVE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.012, OUTER_SLEEVE_BOTTOM + OUTER_SLEEVE_HEIGHT * 0.5)),
        material=stand_black,
        name="outer_face_1",
    )
    stand.visual(
        Box((0.010, 0.008, 0.020)),
        origin=Origin(xyz=(-0.019, 0.0, 0.019)),
        material=stand_black,
        name="gusset_0",
    )
    stand.visual(
        Box((0.010, 0.008, 0.020)),
        origin=Origin(xyz=(0.019, 0.0, 0.019)),
        material=stand_black,
        name="gusset_1",
    )
    stand.inertial = Inertial.from_geometry(
        Box((0.060, 0.040, OUTER_SLEEVE_TOP + 0.020)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
    )

    model.articulation(
        "base_to_stand",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=stand,
        origin=Origin(xyz=(0.0, STAND_OFFSET_Y, BASE_THICKNESS + 0.006)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=14.0, velocity=4.0),
    )

    mast = model.part("mast")
    mast.visual(
        Box((0.028, 0.016, 0.235)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=stand_black,
        name="inner_slide",
    )
    mast.visual(
        Box((0.050, 0.022, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.094)),
        material=stand_black,
        name="head_support",
    )
    mast.visual(
        Box((0.096, 0.012, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.106)),
        material=trim_black,
        name="head_cap",
    )
    mast.inertial = Inertial.from_geometry(
        Box((0.100, 0.040, 0.260)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
    )

    model.articulation(
        "stand_to_mast",
        ArticulationType.PRISMATIC,
        parent=stand,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, OUTER_SLEEVE_TOP)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.14,
            lower=0.0,
            upper=MAST_TRAVEL,
        ),
    )

    housing = model.part("housing")
    housing.visual(
        Box((0.485, 0.012, 0.295)),
        origin=Origin(xyz=(0.0, -0.016, 0.159)),
        material=housing_black,
        name="back_panel",
    )
    housing.visual(
        Box((0.010, HOUSING_DEPTH, HOUSING_HEIGHT)),
        origin=Origin(xyz=(-0.2475, -0.032, HOUSING_HEIGHT * 0.5)),
        material=housing_black,
        name="left_frame",
    )
    housing.visual(
        Box((0.010, HOUSING_DEPTH, HOUSING_HEIGHT)),
        origin=Origin(xyz=(0.2475, -0.032, HOUSING_HEIGHT * 0.5)),
        material=housing_black,
        name="right_frame",
    )
    housing.visual(
        Box((0.485, HOUSING_DEPTH, 0.010)),
        origin=Origin(xyz=(0.0, -0.032, 0.308)),
        material=housing_black,
        name="top_frame",
    )
    housing.visual(
        Box((HOUSING_WIDTH, HOUSING_DEPTH, 0.018)),
        origin=Origin(xyz=(0.0, -0.032, 0.009)),
        material=housing_black,
        name="bottom_frame",
    )
    housing.visual(
        Box((0.114, 0.020, 0.088)),
        origin=Origin(xyz=(0.0, -0.010, 0.062)),
        material=trim_black,
        name="hinge_block",
    )
    housing.inertial = Inertial.from_geometry(
        Box((HOUSING_WIDTH, 0.070, HOUSING_HEIGHT)),
        mass=2.7,
        origin=Origin(xyz=(0.0, -0.016, HOUSING_HEIGHT * 0.5)),
    )

    model.articulation(
        "mast_to_housing",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=housing,
        origin=Origin(xyz=(0.0, 0.0, 0.106)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.6,
            lower=math.radians(-6.0),
            upper=math.radians(24.0),
        ),
    )

    glass = model.part("glass")
    glass.visual(
        Box((0.485, 0.003, 0.285)),
        material=glass_black,
        name="display_glass",
    )
    glass.visual(
        Box((0.430, 0.017, 0.240)),
        origin=Origin(xyz=(0.0, 0.010, 0.0)),
        material=trim_black,
        name="display_body",
    )
    glass.inertial = Inertial.from_geometry(
        Box((0.485, 0.020, 0.285)),
        mass=0.25,
    )
    model.articulation(
        "housing_to_glass",
        ArticulationType.FIXED,
        parent=housing,
        child=glass,
        origin=Origin(xyz=(0.0, -0.049, 0.1605)),
    )

    button_x_positions = (0.126, 0.146, 0.166)
    for index, x_pos in enumerate(button_x_positions):
        button = model.part(f"menu_button_{index}")
        button.visual(
            Box((0.012, 0.007, 0.004)),
            origin=Origin(xyz=(0.0, 0.0, -0.002)),
            material=control_black,
            name="button_cap",
        )
        button.inertial = Inertial.from_geometry(
            Box((0.012, 0.007, 0.004)),
            mass=0.004,
            origin=Origin(xyz=(0.0, 0.0, -0.002)),
        )
        model.articulation(
            f"housing_to_menu_button_{index}",
            ArticulationType.PRISMATIC,
            parent=housing,
            child=button,
            origin=Origin(xyz=(x_pos, -0.048, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=0.05,
                lower=0.0,
                upper=0.0018,
            ),
        )

    power_rocker = model.part("power_rocker")
    power_rocker.visual(
        Box((0.008, 0.010, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=control_black,
        name="rocker_body",
    )
    power_rocker.inertial = Inertial.from_geometry(
        Box((0.008, 0.010, 0.016)),
        mass=0.006,
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
    )
    model.articulation(
        "housing_to_power_rocker",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=power_rocker,
        origin=Origin(xyz=(0.198, -0.048, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=1.5,
            lower=math.radians(-12.0),
            upper=math.radians(12.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stand = object_model.get_part("stand")
    mast = object_model.get_part("mast")
    glass = object_model.get_part("glass")
    power_rocker = object_model.get_part("power_rocker")
    mast_slide = object_model.get_articulation("stand_to_mast")
    swivel = object_model.get_articulation("base_to_stand")
    tilt = object_model.get_articulation("mast_to_housing")
    menu_button = object_model.get_part("menu_button_1")
    menu_button_joint = object_model.get_articulation("housing_to_menu_button_1")
    rocker_joint = object_model.get_articulation("housing_to_power_rocker")

    ctx.expect_within(
        mast,
        stand,
        axes="xy",
        inner_elem="inner_slide",
        margin=0.0005,
        name="mast stays centered in sleeve",
    )
    ctx.expect_overlap(
        mast,
        stand,
        axes="z",
        elem_a="inner_slide",
        min_overlap=0.11,
        name="collapsed mast remains inserted",
    )

    rest_glass_pos = ctx.part_world_position(glass)
    with ctx.pose({mast_slide: MAST_TRAVEL}):
        ctx.expect_within(
            mast,
            stand,
            axes="xy",
            inner_elem="inner_slide",
            margin=0.0005,
            name="extended mast stays centered in sleeve",
        )
        ctx.expect_overlap(
            mast,
            stand,
            axes="z",
            elem_a="inner_slide",
            min_overlap=0.036,
            name="extended mast keeps retained insertion",
        )
        raised_glass_pos = ctx.part_world_position(glass)

    ctx.check(
        "mast raises display",
        rest_glass_pos is not None
        and raised_glass_pos is not None
        and raised_glass_pos[2] > rest_glass_pos[2] + 0.070,
        details=f"rest={rest_glass_pos}, raised={raised_glass_pos}",
    )

    with ctx.pose({tilt: math.radians(24.0)}):
        tilted_glass_pos = ctx.part_world_position(glass)
    ctx.check(
        "positive tilt leans display backward",
        rest_glass_pos is not None
        and tilted_glass_pos is not None
        and tilted_glass_pos[1] > rest_glass_pos[1] + 0.025,
        details=f"rest={rest_glass_pos}, tilted={tilted_glass_pos}",
    )

    with ctx.pose({swivel: math.pi / 2.0}):
        swiveled_glass_pos = ctx.part_world_position(glass)
    ctx.check(
        "stand swivels around the pedestal",
        rest_glass_pos is not None
        and swiveled_glass_pos is not None
        and swiveled_glass_pos[0] > rest_glass_pos[0] + 0.025,
        details=f"rest={rest_glass_pos}, swiveled={swiveled_glass_pos}",
    )

    rest_button_pos = ctx.part_world_position(menu_button)
    with ctx.pose({menu_button_joint: 0.0015}):
        pressed_button_pos = ctx.part_world_position(menu_button)
    ctx.check(
        "menu button depresses upward",
        rest_button_pos is not None
        and pressed_button_pos is not None
        and pressed_button_pos[2] > rest_button_pos[2] + 0.001,
        details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
    )

    rest_rocker_aabb = ctx.part_element_world_aabb(power_rocker, elem="rocker_body")
    with ctx.pose({rocker_joint: math.radians(10.0)}):
        rocked_aabb = ctx.part_element_world_aabb(power_rocker, elem="rocker_body")
    rest_rocker_center = (
        None
        if rest_rocker_aabb is None
        else tuple(
            (rest_rocker_aabb[0][axis] + rest_rocker_aabb[1][axis]) * 0.5 for axis in range(3)
        )
    )
    rocked_center = (
        None
        if rocked_aabb is None
        else tuple((rocked_aabb[0][axis] + rocked_aabb[1][axis]) * 0.5 for axis in range(3))
    )
    ctx.check(
        "power rocker tips along its pivot",
        rest_rocker_center is not None
        and rocked_center is not None
        and rocked_center[1] > rest_rocker_center[1] + 0.0008,
        details=f"rest={rest_rocker_center}, rocked={rocked_center}",
    )

    return ctx.report()


object_model = build_object_model()
