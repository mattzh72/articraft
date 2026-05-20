from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_WIDTH = 0.198
BOWL_HEIGHT = 0.162
BOWL_RIM_RADIUS = 0.128
LIFT_TRAVEL = 0.080
HUB_TWIST = 0.60
def _bowl_shell_mesh():
    outer_profile = [
        (0.056, 0.000),
        (0.060, 0.008),
        (0.072, 0.020),
        (0.093, 0.062),
        (0.114, 0.124),
        (0.124, 0.152),
        (BOWL_RIM_RADIUS, BOWL_HEIGHT),
    ]
    inner_profile = [
        (0.000, 0.006),
        (0.046, 0.012),
        (0.060, 0.028),
        (0.082, 0.064),
        (0.103, 0.124),
        (0.114, 0.148),
        (0.120, 0.156),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(outer_profile, inner_profile, segments=72),
        "mixer_bowl",
    )


def _beater_shape() -> cq.Workplane:
    shaft = cq.Workplane("XY").circle(0.006).extrude(-0.054)
    collar = cq.Workplane("XY").circle(0.011).extrude(-0.018).translate((0.0, 0.0, -0.054))
    bridge = cq.Workplane("XY").box(0.014, 0.030, 0.024).translate((0.0, 0.0, -0.066))
    paddle = (
        cq.Workplane("XZ")
        .center(0.0, -0.118)
        .rect(0.076, 0.092)
        .extrude(0.015, both=True)
    )
    return shaft.union(collar).union(bridge).union(paddle)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bowl_lift_stand_mixer")

    enamel = model.material("enamel", rgba=(0.78, 0.08, 0.10, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.12, 0.12, 0.13, 1.0))
    steel = model.material("steel", rgba=(0.86, 0.87, 0.89, 1.0))
    gray_cast = model.material("gray_cast", rgba=(0.58, 0.60, 0.62, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.360, BODY_WIDTH, 0.048)),
        origin=Origin(xyz=(-0.012, 0.0, 0.024)),
        material=enamel,
        name="base_plinth",
    )
    body.visual(
        Box((0.108, 0.122, 0.282)),
        origin=Origin(xyz=(-0.150, 0.0, 0.189)),
        material=enamel,
        name="column_shell",
    )
    body.visual(
        Box((0.112, 0.134, 0.106)),
        origin=Origin(xyz=(-0.048, 0.0, 0.302)),
        material=enamel,
        name="shoulder_shell",
    )
    body.visual(
        Box((0.214, 0.152, 0.108)),
        origin=Origin(xyz=(0.070, 0.0, 0.388)),
        material=enamel,
        name="head_shell",
    )
    body.visual(
        Box((0.050, 0.126, 0.090)),
        origin=Origin(xyz=(-0.052, 0.0, 0.286)),
        material=enamel,
        name="neck_shell",
    )
    body.visual(
        Box((0.050, 0.118, 0.068)),
        origin=Origin(xyz=(0.170, 0.0, 0.352)),
        material=enamel,
        name="nose_shell",
    )
    body.visual(
        Cylinder(radius=0.030, length=0.028),
        origin=Origin(xyz=(0.191, 0.0, 0.386), rpy=(0.0, 1.57079632679, 0.0)),
        material=dark_trim,
        name="hub_boss",
    )
    body.visual(
        Cylinder(radius=0.024, length=0.044),
        origin=Origin(xyz=(0.106, 0.0, 0.336)),
        material=dark_trim,
        name="drive_hub",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.072, 0.108, 0.172)),
        origin=Origin(xyz=(0.0, 0.0, 0.086)),
        material=gray_cast,
        name="guide_block",
    )
    carriage.visual(
        Box((0.082, 0.164, 0.020)),
        origin=Origin(xyz=(0.028, 0.0, 0.030)),
        material=gray_cast,
        name="crossbar",
    )
    carriage.visual(
        Box((0.168, 0.018, 0.024)),
        origin=Origin(xyz=(0.086, 0.086, 0.016)),
        material=gray_cast,
        name="arm_0",
    )
    carriage.visual(
        Box((0.168, 0.018, 0.024)),
        origin=Origin(xyz=(0.086, -0.086, 0.016)),
        material=gray_cast,
        name="arm_1",
    )
    carriage.visual(
        Box((0.140, 0.040, 0.010)),
        origin=Origin(xyz=(0.088, 0.0, -0.001)),
        material=gray_cast,
        name="tongue",
    )
    carriage.visual(
        Cylinder(radius=0.054, length=0.006),
        origin=Origin(xyz=(0.156, 0.0, -0.003)),
        material=gray_cast,
        name="tray",
    )

    bowl = model.part("bowl")
    bowl.visual(_bowl_shell_mesh(), material=steel, name="bowl_shell")
    bowl.visual(
        Box((0.034, 0.024, 0.020)),
        origin=Origin(xyz=(0.0, 0.126, 0.114)),
        material=steel,
        name="handle_0",
    )
    bowl.visual(
        Box((0.034, 0.024, 0.020)),
        origin=Origin(xyz=(0.0, -0.126, 0.114)),
        material=steel,
        name="handle_1",
    )

    dial = model.part("side_dial")
    dial.visual(
        Cylinder(radius=0.006, length=0.008),
        origin=Origin(xyz=(0.0, 0.004, 0.0), rpy=(1.57079632679, 0.0, 0.0)),
        material=dark_trim,
        name="dial_shaft",
    )
    dial.visual(
        Cylinder(radius=0.022, length=0.012),
        origin=Origin(xyz=(0.0, 0.014, 0.0), rpy=(1.57079632679, 0.0, 0.0)),
        material=dark_trim,
        name="dial_knob",
    )
    dial.visual(
        Cylinder(radius=0.017, length=0.006),
        origin=Origin(xyz=(0.0, 0.023, 0.0), rpy=(1.57079632679, 0.0, 0.0)),
        material=steel,
        name="dial_face",
    )

    hub_cap = model.part("hub_cap")
    hub_cap.visual(
        Cylinder(radius=0.027, length=0.018),
        origin=Origin(xyz=(0.009, 0.0, 0.0), rpy=(0.0, 1.57079632679, 0.0)),
        material=steel,
        name="cap_disk",
    )
    hub_cap.visual(
        Box((0.008, 0.010, 0.004)),
        origin=Origin(xyz=(0.004, 0.021, 0.0)),
        material=steel,
        name="lug_0",
    )
    hub_cap.visual(
        Box((0.008, 0.010, 0.004)),
        origin=Origin(xyz=(0.004, -0.021, 0.0)),
        material=steel,
        name="lug_1",
    )

    beater = model.part("beater")
    beater.visual(
        mesh_from_cadquery(_beater_shape(), "beater"),
        material=steel,
        name="beater_shell",
    )

    model.articulation(
        "body_to_carriage",
        ArticulationType.PRISMATIC,
        parent=body,
        child=carriage,
        origin=Origin(xyz=(-0.060, 0.0, 0.060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.08,
            lower=0.0,
            upper=LIFT_TRAVEL,
        ),
    )
    model.articulation(
        "carriage_to_bowl",
        ArticulationType.FIXED,
        parent=carriage,
        child=bowl,
        origin=Origin(xyz=(0.166, 0.0, 0.0)),
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.028, 0.076, 0.334)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=12.0),
    )
    model.articulation(
        "body_to_hub_cap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=hub_cap,
        origin=Origin(xyz=(0.205, 0.0, 0.386)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=3.0,
            lower=0.0,
            upper=HUB_TWIST,
        ),
    )
    model.articulation(
        "body_to_beater",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=beater,
        origin=Origin(xyz=(0.106, 0.0, 0.314)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=25.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    bowl = object_model.get_part("bowl")
    carriage = object_model.get_part("carriage")
    dial = object_model.get_part("side_dial")
    hub_cap = object_model.get_part("hub_cap")
    beater = object_model.get_part("beater")
    lift = object_model.get_articulation("body_to_carriage")
    twist = object_model.get_articulation("body_to_hub_cap")

    ctx.expect_origin_gap(
        bowl,
        body,
        axis="x",
        min_gap=0.06,
        name="bowl sits forward of the stand column",
    )
    ctx.expect_overlap(
        bowl,
        carriage,
        axes="xy",
        min_overlap=0.08,
        name="bowl stays centered over the lift carriage",
    )
    ctx.expect_overlap(
        beater,
        bowl,
        axes="xy",
        min_overlap=0.03,
        name="beater stays centered over the bowl",
    )
    ctx.expect_origin_gap(
        dial,
        body,
        axis="y",
        min_gap=0.07,
        name="side dial sits proud of the mixer body",
    )
    ctx.expect_origin_gap(
        hub_cap,
        body,
        axis="x",
        min_gap=0.18,
        name="hub cap mounts on the front accessory hub",
    )

    rest_pos = ctx.part_world_position(bowl)
    with ctx.pose({lift: LIFT_TRAVEL}):
        raised_pos = ctx.part_world_position(bowl)
        ctx.expect_overlap(
            beater,
            bowl,
            axes="xy",
            min_overlap=0.03,
            name="raised bowl stays under the beater axis",
        )

    with ctx.pose({twist: HUB_TWIST}):
        ctx.expect_overlap(
            hub_cap,
            body,
            axes="yz",
            min_overlap=0.03,
            name="hub cap stays aligned during the bayonet twist",
        )

    ctx.check(
        "bowl rises on the lift travel",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.07,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    return ctx.report()


object_model = build_object_model()
