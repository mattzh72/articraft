from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    KnobGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_DEPTH = 0.118
BODY_WIDTH = 0.082
BODY_HEIGHT = 0.096
FRONT_X = 0.056

CRANK_SHAFT_X = 0.008
CRANK_SHAFT_Z = 0.070

DRAWER_TRAVEL = 0.036
DRAWER_PANEL_THICKNESS = 0.006


def _build_housing_shape() -> cq.Workplane:
    side_profile = [
        (-0.056, 0.000),
        (0.056, 0.000),
        (0.056, 0.062),
        (0.044, 0.080),
        (0.012, 0.094),
        (-0.040, 0.092),
        (-0.056, 0.074),
    ]

    def side_plate(y_center: float) -> cq.Workplane:
        plate = (
            cq.Workplane("XZ")
            .polyline(side_profile)
            .close()
            .extrude(0.004, both=True)
            .translate((0.0, y_center, 0.0))
        )
        window = (
            cq.Workplane("XY")
            .box(0.088, 0.012, 0.056, centered=(True, True, False))
            .translate((0.012, y_center, 0.006))
        )
        return plate.cut(window)

    housing = side_plate(0.037).union(side_plate(-0.037))

    rear_bridge = (
        cq.Workplane("XY")
        .box(0.018, 0.066, 0.076, centered=(True, True, False))
        .translate((-0.047, 0.0, 0.0))
    )
    roof = (
        cq.Workplane("XY")
        .box(0.084, 0.066, 0.024, centered=(True, True, False))
        .translate((-0.002, 0.0, 0.062))
    )
    nose = (
        cq.Workplane("XY")
        .box(0.032, 0.066, 0.014, centered=(True, True, False))
        .translate((0.040, 0.0, 0.080))
    )
    front_bridge = (
        cq.Workplane("XY")
        .box(0.018, 0.066, 0.018, centered=(True, True, False))
        .translate((0.047, 0.0, 0.062))
    )
    front_sill = (
        cq.Workplane("XY")
        .box(0.010, 0.066, 0.006, centered=(True, True, False))
        .translate((0.051, 0.0, 0.0))
    )

    housing = housing.union(rear_bridge).union(roof).union(nose).union(front_bridge).union(front_sill)

    pencil_hole = (
        cq.Workplane("YZ")
        .circle(0.0052)
        .extrude(0.030)
        .translate((0.041, 0.0, 0.050))
    )
    housing = housing.cut(pencil_hole)

    shaft_boss = (
        cq.Workplane("XZ")
        .center(CRANK_SHAFT_X, CRANK_SHAFT_Z)
        .circle(0.011)
        .extrude(BODY_WIDTH * 0.5 + 0.006, both=True)
    )
    housing = housing.union(shaft_boss)

    shaft_hole = (
        cq.Workplane("XZ")
        .center(CRANK_SHAFT_X, CRANK_SHAFT_Z)
        .circle(0.0066)
        .extrude(BODY_WIDTH * 0.5 + 0.012, both=True)
    )
    return housing.cut(shaft_hole)


def _build_drawer_shape() -> cq.Workplane:
    front_panel = cq.Workplane("XY").box(
        DRAWER_PANEL_THICKNESS,
        0.068,
        0.054,
        centered=(True, True, False),
    )

    outer_bin = (
        cq.Workplane("XY")
        .box(0.078, 0.058, 0.040, centered=(True, True, False))
        .translate((-0.042, 0.0, 0.007))
    )
    inner_bin = (
        cq.Workplane("XY")
        .box(0.072, 0.052, 0.036, centered=(True, True, False))
        .translate((-0.042, 0.0, 0.010))
    )
    drawer_bin = outer_bin.cut(inner_bin)

    pull_lip = (
        cq.Workplane("XY")
        .box(0.012, 0.050, 0.009, centered=(True, True, False))
        .translate((0.003, 0.0, 0.007))
    )

    return front_panel.union(drawer_bin).union(pull_lip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="classroom_pencil_sharpener")

    housing_enamel = model.material("housing_enamel", rgba=(0.61, 0.10, 0.11, 1.0))
    drawer_black = model.material("drawer_black", rgba=(0.10, 0.11, 0.12, 1.0))
    steel = model.material("steel", rgba=(0.68, 0.69, 0.72, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.27, 0.28, 0.30, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.12, 0.12, 0.13, 1.0))
    knob_ivory = model.material("knob_ivory", rgba=(0.84, 0.82, 0.74, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_housing_shape(), "sharpener_housing"),
        material=housing_enamel,
        name="housing_shell",
    )
    body.visual(
        Cylinder(radius=0.016, length=0.008),
        origin=Origin(xyz=(-0.047, 0.0, -0.004)),
        material=rubber_black,
        name="suction_core",
    )
    body.visual(
        Cylinder(radius=0.028, length=0.002),
        origin=Origin(xyz=(-0.047, 0.0, -0.001)),
        material=rubber_black,
        name="suction_rim",
    )
    body.visual(
        Box((0.018, 0.010, 0.010)),
        origin=Origin(xyz=(-0.022, 0.030, -0.005)),
        material=dark_metal,
        name="lever_mount",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_DEPTH, BODY_WIDTH, 0.104)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
    )

    drawer_guide_0 = model.part("drawer_guide_0")
    drawer_guide_0.visual(
        Box((0.094, 0.004, 0.040)),
        material=dark_metal,
        name="guide_bar",
    )
    drawer_guide_0.inertial = Inertial.from_geometry(
        Box((0.094, 0.004, 0.040)),
        mass=0.02,
    )

    drawer_guide_1 = model.part("drawer_guide_1")
    drawer_guide_1.visual(
        Box((0.094, 0.004, 0.040)),
        material=dark_metal,
        name="guide_bar",
    )
    drawer_guide_1.inertial = Inertial.from_geometry(
        Box((0.094, 0.004, 0.040)),
        mass=0.02,
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.006, 0.068, 0.054)),
        material=drawer_black,
        name="drawer_front",
    )
    drawer.visual(
        Box((0.078, 0.052, 0.004)),
        origin=Origin(xyz=(-0.042, 0.0, -0.025)),
        material=drawer_black,
        name="drawer_floor",
    )
    drawer.visual(
        Box((0.078, 0.004, 0.040)),
        origin=Origin(xyz=(-0.042, 0.028, -0.007)),
        material=drawer_black,
        name="drawer_side_0",
    )
    drawer.visual(
        Box((0.078, 0.004, 0.040)),
        origin=Origin(xyz=(-0.042, -0.028, -0.007)),
        material=drawer_black,
        name="drawer_side_1",
    )
    drawer.visual(
        Box((0.004, 0.052, 0.040)),
        origin=Origin(xyz=(-0.079, 0.0, -0.007)),
        material=drawer_black,
        name="drawer_back",
    )
    drawer.visual(
        Box((0.010, 0.050, 0.009)),
        origin=Origin(xyz=(0.004, 0.0, -0.0225)),
        material=drawer_black,
        name="pull_lip",
    )
    drawer.inertial = Inertial.from_geometry(
        Box((0.090, 0.068, 0.054)),
        mass=0.12,
        origin=Origin(xyz=(-0.040, 0.0, -0.002)),
    )

    crank = model.part("crank")
    crank.visual(
        Cylinder(radius=0.0055, length=0.082),
        origin=Origin(xyz=(0.0, 0.021, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=dark_metal,
        name="shaft_core",
    )
    crank.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(0.0, 0.051, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=steel,
        name="crank_hub",
    )
    crank.visual(
        Cylinder(radius=0.0038, length=0.060),
        origin=Origin(xyz=(0.030, 0.056, -0.006), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=steel,
        name="crank_arm",
    )
    crank.visual(
        Cylinder(radius=0.0027, length=0.030),
        origin=Origin(xyz=(0.060, 0.056, 0.010)),
        material=steel,
        name="spinner_pin",
    )
    crank.inertial = Inertial.from_geometry(
        Box((0.070, 0.090, 0.040)),
        mass=0.08,
        origin=Origin(xyz=(0.030, 0.040, 0.005)),
    )

    spinner_knob = model.part("spinner_knob")
    spinner_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.018,
                0.028,
                body_style="mushroom",
                top_diameter=0.015,
                crown_radius=0.0025,
                edge_radius=0.001,
                center=False,
            ),
            "spinner_knob",
        ),
        material=knob_ivory,
        name="spinner_body",
    )
    spinner_knob.inertial = Inertial.from_geometry(
        Box((0.020, 0.020, 0.028)),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
    )

    lock_lever = model.part("lock_lever")
    lock_lever.visual(
        Cylinder(radius=0.0045, length=0.010),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=rubber_black,
        name="lever_barrel",
    )
    lock_lever.visual(
        Box((0.022, 0.008, 0.004)),
        origin=Origin(xyz=(0.012, 0.0, -0.005)),
        material=rubber_black,
        name="lever_arm",
    )
    lock_lever.visual(
        Box((0.010, 0.010, 0.006)),
        origin=Origin(xyz=(0.020, 0.0, -0.009)),
        material=rubber_black,
        name="lever_tip",
    )
    lock_lever.inertial = Inertial.from_geometry(
        Box((0.032, 0.012, 0.020)),
        mass=0.02,
        origin=Origin(xyz=(0.016, 0.0, -0.006)),
    )

    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(FRONT_X + DRAWER_PANEL_THICKNESS * 0.5, 0.0, 0.034)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.12,
            lower=0.0,
            upper=DRAWER_TRAVEL,
        ),
    )
    model.articulation(
        "body_to_drawer_guide_0",
        ArticulationType.FIXED,
        parent=body,
        child=drawer_guide_0,
        origin=Origin(xyz=(0.009, 0.032, 0.029)),
    )
    model.articulation(
        "body_to_drawer_guide_1",
        ArticulationType.FIXED,
        parent=body,
        child=drawer_guide_1,
        origin=Origin(xyz=(0.009, -0.032, 0.029)),
    )
    model.articulation(
        "body_to_crank",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=crank,
        origin=Origin(xyz=(CRANK_SHAFT_X, 0.0, CRANK_SHAFT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=14.0),
    )
    model.articulation(
        "crank_to_spinner_knob",
        ArticulationType.CONTINUOUS,
        parent=crank,
        child=spinner_knob,
        origin=Origin(xyz=(0.060, 0.056, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=20.0),
    )
    model.articulation(
        "body_to_lock_lever",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lock_lever,
        origin=Origin(xyz=(-0.022, 0.040, -0.005)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=0.0,
            upper=0.75,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    crank = object_model.get_part("crank")
    spinner_knob = object_model.get_part("spinner_knob")
    lock_lever = object_model.get_part("lock_lever")

    drawer_joint = object_model.get_articulation("body_to_drawer")
    lever_joint = object_model.get_articulation("body_to_lock_lever")

    ctx.allow_overlap(
        crank,
        spinner_knob,
        elem_a="spinner_pin",
        elem_b="spinner_body",
        reason="The spinner knob is intentionally modeled as rotating around a retained axle pin.",
    )

    closed_drawer_pos = ctx.part_world_position(drawer)
    ctx.expect_overlap(
        drawer,
        body,
        axes="yz",
        min_overlap=0.052,
        name="drawer aligns with the sharpener body at rest",
    )

    extended_drawer_pos = None
    lever_tip_rest = ctx.part_element_world_aabb(lock_lever, elem="lever_tip")
    lever_tip_open = None

    drawer_limits = drawer_joint.motion_limits
    if drawer_limits is not None and drawer_limits.upper is not None:
        with ctx.pose({drawer_joint: drawer_limits.upper}):
            ctx.expect_overlap(
                drawer,
                body,
                axes="yz",
                min_overlap=0.050,
                name="drawer stays centered when fully pulled out",
            )
            ctx.expect_overlap(
                drawer,
                body,
                axes="x",
                min_overlap=0.035,
                name="drawer retains insertion at full extension",
            )
            extended_drawer_pos = ctx.part_world_position(drawer)

    ctx.check(
        "drawer slides forward",
        closed_drawer_pos is not None
        and extended_drawer_pos is not None
        and extended_drawer_pos[0] > closed_drawer_pos[0] + 0.03,
        details=f"closed={closed_drawer_pos}, extended={extended_drawer_pos}",
    )

    lever_limits = lever_joint.motion_limits
    if lever_limits is not None and lever_limits.upper is not None:
        with ctx.pose({lever_joint: lever_limits.upper}):
            lever_tip_open = ctx.part_element_world_aabb(lock_lever, elem="lever_tip")

    lever_rest_z = None if lever_tip_rest is None else lever_tip_rest[0][2]
    lever_open_z = None if lever_tip_open is None else lever_tip_open[0][2]
    ctx.check(
        "lock lever drops below the base when actuated",
        lever_rest_z is not None and lever_open_z is not None and lever_open_z < lever_rest_z - 0.008,
        details=f"rest_min_z={lever_rest_z}, open_min_z={lever_open_z}",
    )

    return ctx.report()


object_model = build_object_model()
