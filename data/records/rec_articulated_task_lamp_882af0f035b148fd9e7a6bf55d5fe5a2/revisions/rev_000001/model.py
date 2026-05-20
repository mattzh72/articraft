from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


LOWER_ARM_LENGTH = 0.340
UPPER_ARM_LENGTH = 0.300
SHOULDER_Z = 0.173


def _cylinder_y(radius: float, length: float) -> Cylinder:
    return Cylinder(radius=radius, length=length)


def _build_shade_shell() -> object:
    return (
        cq.Workplane("XZ")
        .moveTo(0.030, 0.022)
        .spline([(0.056, 0.039), (0.112, 0.055), (0.182, 0.056)])
        .lineTo(0.182, 0.051)
        .spline([(0.110, 0.046), (0.054, 0.031), (0.030, 0.016)])
        .close()
        .revolve(360, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    )


def _add_link_visuals(
    part,
    *,
    length: float,
    barrel_radius: float,
    barrel_length: float,
    beam_size: tuple[float, float, float],
    bridge_size: tuple[float, float, float],
    bridge_center_x: float,
    cheek_size: tuple[float, float, float],
    cheek_center_x: float,
    cheek_center_y: float,
    material,
) -> None:
    part.visual(
        _cylinder_y(barrel_radius, barrel_length),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name="base_barrel",
    )
    part.visual(
        Box(beam_size),
        origin=Origin(xyz=(0.010 + beam_size[0] / 2.0, 0.0, 0.0)),
        material=material,
        name="beam",
    )
    part.visual(
        Box(bridge_size),
        origin=Origin(xyz=(bridge_center_x, 0.0, 0.0)),
        material=material,
        name="fork_bridge",
    )
    for index, side in enumerate((-1.0, 1.0)):
        part.visual(
            Box(cheek_size),
            origin=Origin(xyz=(cheek_center_x, side * cheek_center_y, 0.0)),
            material=material,
            name=f"fork_cheek_{index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clamp_task_lamp")

    dark_metal = model.material("dark_metal", rgba=(0.19, 0.20, 0.22, 1.0))
    steel = model.material("steel", rgba=(0.60, 0.62, 0.66, 1.0))
    shade_paint = model.material("shade_paint", rgba=(0.83, 0.84, 0.86, 1.0))
    knob_black = model.material("knob_black", rgba=(0.08, 0.08, 0.09, 1.0))
    socket_grey = model.material("socket_grey", rgba=(0.72, 0.73, 0.75, 1.0))

    clamp = model.part("clamp")
    clamp.visual(
        Box((0.024, 0.060, 0.140)),
        origin=Origin(xyz=(-0.030, 0.0, 0.070)),
        material=dark_metal,
        name="spine",
    )
    clamp.visual(
        Box((0.072, 0.060, 0.018)),
        origin=Origin(xyz=(0.010, 0.0, 0.131)),
        material=dark_metal,
        name="top_jaw",
    )
    clamp.visual(
        Box((0.056, 0.060, 0.018)),
        origin=Origin(xyz=(0.002, 0.0, 0.025)),
        material=dark_metal,
        name="bottom_jaw",
    )
    clamp.visual(
        Box((0.020, 0.060, 0.072)),
        origin=Origin(xyz=(0.028, 0.0, 0.061)),
        material=dark_metal,
        name="front_drop",
    )
    clamp.visual(
        Box((0.020, 0.060, 0.024)),
        origin=Origin(xyz=(-0.020, 0.0, 0.147)),
        material=dark_metal,
        name="shoulder_post",
    )
    clamp.visual(
        Box((0.018, 0.074, 0.030)),
        origin=Origin(xyz=(-0.024, 0.0, 0.166)),
        material=dark_metal,
        name="shoulder_bridge",
    )
    for index, side in enumerate((-1.0, 1.0)):
        clamp.visual(
            Box((0.032, 0.008, 0.040)),
            origin=Origin(xyz=(-0.001, side * 0.041, SHOULDER_Z)),
            material=dark_metal,
            name=f"shoulder_cheek_{index}",
        )
    clamp.visual(
        Cylinder(radius=0.008, length=0.020),
        origin=Origin(xyz=(0.028, 0.0, 0.125)),
        material=steel,
        name="screw_boss",
    )
    clamp.visual(
        Cylinder(radius=0.006, length=0.132),
        origin=Origin(xyz=(0.028, 0.0, 0.053)),
        material=steel,
        name="screw_shaft",
    )
    clamp.visual(
        Cylinder(radius=0.015, length=0.006),
        origin=Origin(xyz=(0.028, 0.0, 0.006)),
        material=steel,
        name="pressure_pad",
    )
    clamp.visual(
        Sphere(radius=0.018),
        origin=Origin(xyz=(0.028, 0.0, -0.020)),
        material=dark_metal,
        name="screw_knob",
    )

    lower_arm = model.part("lower_arm")
    _add_link_visuals(
        lower_arm,
        length=LOWER_ARM_LENGTH,
        barrel_radius=0.015,
        barrel_length=0.074,
        beam_size=(0.292, 0.022, 0.018),
        bridge_size=(0.024, 0.082, 0.016),
        bridge_center_x=0.314,
        cheek_size=(0.022, 0.008, 0.036),
        cheek_center_x=0.337,
        cheek_center_y=0.041,
        material=dark_metal,
    )

    upper_arm = model.part("upper_arm")
    _add_link_visuals(
        upper_arm,
        length=UPPER_ARM_LENGTH,
        barrel_radius=0.014,
        barrel_length=0.074,
        beam_size=(0.255, 0.020, 0.016),
        bridge_size=(0.024, 0.082, 0.016),
        bridge_center_x=0.273,
        cheek_size=(0.022, 0.008, 0.034),
        cheek_center_x=0.296,
        cheek_center_y=0.039,
        material=dark_metal,
    )

    shade = model.part("shade")
    shade.visual(
        mesh_from_cadquery(_build_shade_shell(), "task_lamp_shade_shell"),
        material=shade_paint,
        name="shade_body",
    )
    shade.visual(
        Cylinder(radius=0.022, length=0.044),
        origin=Origin(xyz=(0.018, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=shade_paint,
        name="neck_tube",
    )
    shade.visual(
        _cylinder_y(0.015, 0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=shade_paint,
        name="trunnion",
    )
    shade.visual(
        Cylinder(radius=0.010, length=0.008),
        origin=Origin(xyz=(0.020, 0.0, 0.022)),
        material=shade_paint,
        name="knob_mount",
    )
    shade.visual(
        Cylinder(radius=0.017, length=0.042),
        origin=Origin(xyz=(0.044, 0.0, -0.002), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=socket_grey,
        name="socket",
    )

    knob = model.part("dimmer_knob")
    knob.visual(
        Cylinder(radius=0.004, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=steel,
        name="shaft",
    )
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.018,
                0.010,
                body_style="cylindrical",
                edge_radius=0.0012,
                grip=KnobGrip(
                    style="knurled",
                    count=28,
                    depth=0.0007,
                    helix_angle_deg=18.0,
                ),
                center=False,
            ),
            "task_lamp_dimmer_knob",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=knob_black,
        name="knob",
    )

    model.articulation(
        "clamp_to_lower_arm",
        ArticulationType.REVOLUTE,
        parent=clamp,
        child=lower_arm,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=-0.60,
            upper=1.20,
        ),
    )
    model.articulation(
        "lower_arm_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=(LOWER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=1.8,
            lower=-1.10,
            upper=1.10,
        ),
    )
    model.articulation(
        "upper_arm_to_shade",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=shade,
        origin=Origin(xyz=(UPPER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.2,
            lower=-1.05,
            upper=0.75,
        ),
    )
    model.articulation(
        "shade_to_dimmer_knob",
        ArticulationType.CONTINUOUS,
        parent=shade,
        child=knob,
        origin=Origin(xyz=(0.020, 0.0, 0.026)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.08,
            velocity=8.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    clamp = object_model.get_part("clamp")
    lower_arm = object_model.get_part("lower_arm")
    shade = object_model.get_part("shade")
    knob = object_model.get_part("dimmer_knob")
    shoulder = object_model.get_articulation("clamp_to_lower_arm")
    elbow = object_model.get_articulation("lower_arm_to_upper_arm")
    shade_tilt = object_model.get_articulation("upper_arm_to_shade")
    dimmer = object_model.get_articulation("shade_to_dimmer_knob")

    ctx.expect_gap(
        lower_arm,
        clamp,
        axis="z",
        positive_elem="beam",
        negative_elem="top_jaw",
        min_gap=0.015,
        name="lower arm clears the clamp at rest",
    )
    ctx.expect_contact(
        knob,
        shade,
        elem_a="shaft",
        elem_b="knob_mount",
        name="dimmer shaft seats on the neck mount",
    )

    with ctx.pose({shoulder: 0.40, elbow: 0.20}):
        ctx.expect_origin_gap(
            shade,
            clamp,
            axis="x",
            min_gap=0.50,
            name="arm reaches well out over the work surface",
        )
        ctx.expect_origin_gap(
            shade,
            clamp,
            axis="z",
            min_gap=0.28,
            name="raised arm lifts the shade above the clamp",
        )

    with ctx.pose({shoulder: 0.45, elbow: 0.15, shade_tilt: 0.35}):
        tilted_up_aabb = ctx.part_world_aabb(shade)
    with ctx.pose({shoulder: 0.45, elbow: 0.15, shade_tilt: -0.70}):
        tilted_down_aabb = ctx.part_world_aabb(shade)

    ctx.check(
        "shade tilt can aim downward",
        tilted_up_aabb is not None
        and tilted_down_aabb is not None
        and tilted_down_aabb[0][2] < tilted_up_aabb[0][2] - 0.025,
        details=f"tilted_up_aabb={tilted_up_aabb}, tilted_down_aabb={tilted_down_aabb}",
    )
    ctx.check(
        "dimmer uses continuous rotation",
        dimmer.articulation_type == ArticulationType.CONTINUOUS
        and dimmer.motion_limits is not None
        and dimmer.motion_limits.lower is None
        and dimmer.motion_limits.upper is None,
        details=f"type={dimmer.articulation_type}, limits={dimmer.motion_limits}",
    )

    return ctx.report()


object_model = build_object_model()
