from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

BASE_RADIUS = 0.17
BASE_LOWER_THICKNESS = 0.021
BASE_UPPER_RADIUS = 0.145
BASE_UPPER_THICKNESS = 0.013
BASE_THICKNESS = BASE_LOWER_THICKNESS + BASE_UPPER_THICKNESS

POST_RADIUS = 0.012
POST_HEIGHT = 1.055
SHOULDER_Z = BASE_THICKNESS + POST_HEIGHT
SHOULDER_OFFSET = 0.038
SHOULDER_SUPPORT_RADIUS = 0.0065
SHOULDER_GAP = 0.028
SHOULDER_BARREL_RADIUS = 0.014
SHOULDER_CHEEK_THICKNESS = 0.006
SHOULDER_YOKE_LENGTH = 0.034
SHOULDER_YOKE_HEIGHT = 0.042
SHOULDER_BRIDGE_THICKNESS = 0.010

LOWER_ARM_LENGTH = 0.44
LOWER_BEAM_RADIUS = 0.0105
LOWER_ROOT_HUB_RADIUS = 0.0125
LOWER_ROOT_HUB_LENGTH = 0.060
ELBOW_GAP = 0.024
ELBOW_BARREL_RADIUS = 0.013
ELBOW_CHEEK_THICKNESS = 0.0055
ELBOW_YOKE_LENGTH = 0.030
ELBOW_YOKE_HEIGHT = 0.040
ELBOW_BRIDGE_THICKNESS = 0.010

UPPER_ARM_LENGTH = 0.38
UPPER_BEAM_RADIUS = 0.0095
UPPER_ROOT_HUB_RADIUS = 0.011
UPPER_ROOT_HUB_LENGTH = 0.056
TIP_GAP = 0.022
TIP_BARREL_RADIUS = 0.011
TIP_CHEEK_THICKNESS = 0.005
TIP_YOKE_LENGTH = 0.026
TIP_YOKE_HEIGHT = 0.036
TIP_BRIDGE_THICKNESS = 0.009

SHADE_NECK_RADIUS = 0.0095
SHADE_NECK_LENGTH = 0.055
SHADE_BODY_START = 0.045
SHADE_BODY_END = 0.225
SHADE_REAR_RADIUS = 0.038
SHADE_FRONT_RADIUS = 0.102
SHADE_WALL = 0.006
YOKE_BRIDGE_HEIGHT = 0.010


def _cylinder_x(start_x: float, end_x: float, radius: float, *, z: float = 0.0) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .workplane(offset=start_x)
        .center(0.0, z)
        .circle(radius)
        .extrude(end_x - start_x)
    )


def _cylinder_y(center_x: float, center_z: float, radius: float, length: float) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .center(center_x, center_z)
        .circle(radius)
        .extrude(length / 2.0, both=True)
    )


def _box(center_xyz: tuple[float, float, float], size_xyz: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size_xyz, centered=(True, True, True)).translate(center_xyz)


def _frustum_x(start_x: float, end_x: float, start_radius: float, end_radius: float) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .workplane(offset=start_x)
        .circle(start_radius)
        .workplane(offset=end_x - start_x)
        .circle(end_radius)
        .loft(combine=True)
    )


def _yoke_x(
    center_x: float,
    *,
    gap: float,
    cheek_thickness: float,
    length_x: float,
    height_z: float,
    bridge_thickness: float,
    z_center: float = 0.0,
    bridge_height: float = YOKE_BRIDGE_HEIGHT,
) -> cq.Workplane:
    cheek_center_y = gap / 2.0 + cheek_thickness / 2.0
    outer_width = gap + 2.0 * cheek_thickness
    bridge_center_x = center_x - (length_x - bridge_thickness) / 2.0
    bridge_center_z = z_center - height_z / 2.0 + bridge_height / 2.0

    cheek_pos = _box((center_x, cheek_center_y, z_center), (length_x, cheek_thickness, height_z))
    cheek_neg = _box((center_x, -cheek_center_y, z_center), (length_x, cheek_thickness, height_z))
    bridge = _box((bridge_center_x, 0.0, bridge_center_z), (bridge_thickness, outer_width, bridge_height))
    return cheek_pos.union(cheek_neg).union(bridge)


def _build_stand_shape() -> cq.Workplane:
    lower_base = cq.Workplane("XY").circle(BASE_RADIUS).extrude(BASE_LOWER_THICKNESS)
    upper_base = (
        cq.Workplane("XY")
        .workplane(offset=BASE_LOWER_THICKNESS)
        .circle(BASE_UPPER_RADIUS)
        .extrude(BASE_UPPER_THICKNESS)
    )
    post = (
        cq.Workplane("XY")
        .workplane(offset=BASE_THICKNESS)
        .circle(POST_RADIUS)
        .extrude(POST_HEIGHT)
    )
    shoulder_bridge_center_z = SHOULDER_Z - SHOULDER_YOKE_HEIGHT / 2.0 + YOKE_BRIDGE_HEIGHT / 2.0
    shoulder_support_end = SHOULDER_OFFSET - SHOULDER_YOKE_LENGTH / 2.0 + SHOULDER_BRIDGE_THICKNESS
    shoulder_support = _cylinder_x(0.0, shoulder_support_end, SHOULDER_SUPPORT_RADIUS, z=shoulder_bridge_center_z)
    shoulder_yoke = _yoke_x(
        SHOULDER_OFFSET,
        gap=SHOULDER_GAP,
        cheek_thickness=SHOULDER_CHEEK_THICKNESS,
        length_x=SHOULDER_YOKE_LENGTH,
        height_z=SHOULDER_YOKE_HEIGHT,
        bridge_thickness=SHOULDER_BRIDGE_THICKNESS,
        z_center=SHOULDER_Z,
    )
    return lower_base.union(upper_base).union(post).union(shoulder_support).union(shoulder_yoke)


def _build_arm_shape(
    *,
    length: float,
    root_barrel_radius: float,
    root_barrel_length: float,
    root_hub_radius: float,
    root_hub_length: float,
    beam_radius: float,
    distal_gap: float,
    distal_cheek_thickness: float,
    distal_yoke_length: float,
    distal_yoke_height: float,
    distal_bridge_thickness: float,
) -> cq.Workplane:
    root_barrel = _cylinder_y(0.0, 0.0, root_barrel_radius, root_barrel_length)
    root_hub = _cylinder_x(0.0, root_hub_length, root_hub_radius)
    beam = _cylinder_x(
        root_hub_length - 0.012,
        length - distal_yoke_length / 2.0 + distal_bridge_thickness,
        beam_radius,
    )
    distal_yoke = _yoke_x(
        length,
        gap=distal_gap,
        cheek_thickness=distal_cheek_thickness,
        length_x=distal_yoke_length,
        height_z=distal_yoke_height,
        bridge_thickness=distal_bridge_thickness,
    )
    return root_barrel.union(root_hub).union(beam).union(distal_yoke)


def _build_shade_shape() -> cq.Workplane:
    root_barrel = _cylinder_y(0.0, 0.0, TIP_BARREL_RADIUS, TIP_GAP)
    neck = _cylinder_x(0.0, SHADE_NECK_LENGTH, SHADE_NECK_RADIUS)
    outer_shell = _frustum_x(
        SHADE_BODY_START,
        SHADE_BODY_END,
        SHADE_REAR_RADIUS,
        SHADE_FRONT_RADIUS,
    )
    inner_cut = _frustum_x(
        SHADE_BODY_START + 0.012,
        SHADE_BODY_END + 0.004,
        SHADE_REAR_RADIUS - SHADE_WALL,
        SHADE_FRONT_RADIUS - SHADE_WALL,
    )
    return root_barrel.union(neck).union(outer_shell).cut(inner_cut)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swing_arm_floor_lamp")

    model.material("powder_black", rgba=(0.12, 0.12, 0.13, 1.0))
    model.material("aged_brass", rgba=(0.63, 0.53, 0.30, 1.0))
    model.material("linen", rgba=(0.93, 0.91, 0.84, 1.0))

    stand = model.part("stand")
    stand.visual(
        mesh_from_cadquery(_build_stand_shape(), "stand"),
        material="powder_black",
        name="stand_body",
    )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        mesh_from_cadquery(
            _build_arm_shape(
                length=LOWER_ARM_LENGTH,
                root_barrel_radius=SHOULDER_BARREL_RADIUS,
                root_barrel_length=SHOULDER_GAP,
                root_hub_radius=LOWER_ROOT_HUB_RADIUS,
                root_hub_length=LOWER_ROOT_HUB_LENGTH,
                beam_radius=LOWER_BEAM_RADIUS,
                distal_gap=ELBOW_GAP,
                distal_cheek_thickness=ELBOW_CHEEK_THICKNESS,
                distal_yoke_length=ELBOW_YOKE_LENGTH,
                distal_yoke_height=ELBOW_YOKE_HEIGHT,
                distal_bridge_thickness=ELBOW_BRIDGE_THICKNESS,
            ),
            "lower_arm",
        ),
        material="aged_brass",
        name="lower_arm_body",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        mesh_from_cadquery(
            _build_arm_shape(
                length=UPPER_ARM_LENGTH,
                root_barrel_radius=ELBOW_BARREL_RADIUS,
                root_barrel_length=ELBOW_GAP,
                root_hub_radius=UPPER_ROOT_HUB_RADIUS,
                root_hub_length=UPPER_ROOT_HUB_LENGTH,
                beam_radius=UPPER_BEAM_RADIUS,
                distal_gap=TIP_GAP,
                distal_cheek_thickness=TIP_CHEEK_THICKNESS,
                distal_yoke_length=TIP_YOKE_LENGTH,
                distal_yoke_height=TIP_YOKE_HEIGHT,
                distal_bridge_thickness=TIP_BRIDGE_THICKNESS,
            ),
            "upper_arm",
        ),
        material="aged_brass",
        name="upper_arm_body",
    )

    shade = model.part("shade")
    shade.visual(
        mesh_from_cadquery(_build_shade_shape(), "shade"),
        material="linen",
        name="shade_body",
    )

    model.articulation(
        "stand_to_lower_arm",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=lower_arm,
        origin=Origin(xyz=(SHOULDER_OFFSET, 0.0, SHOULDER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.75, upper=1.05, effort=18.0, velocity=1.4),
    )
    model.articulation(
        "lower_arm_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=(LOWER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.15, upper=1.15, effort=14.0, velocity=1.8),
    )
    model.articulation(
        "upper_arm_to_shade",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=shade,
        origin=Origin(xyz=(UPPER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.8, upper=0.9, effort=6.0, velocity=2.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stand = object_model.get_part("stand")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    shade = object_model.get_part("shade")

    shoulder = object_model.get_articulation("stand_to_lower_arm")
    elbow = object_model.get_articulation("lower_arm_to_upper_arm")
    shade_tilt = object_model.get_articulation("upper_arm_to_shade")

    ctx.allow_overlap(
        stand,
        lower_arm,
        reason="The shoulder hinge is represented with an exposed yoke and barrel that intentionally share a small simplified embed region at the pivot.",
    )
    ctx.allow_overlap(
        lower_arm,
        upper_arm,
        reason="The elbow hinge uses a simplified yoke-and-barrel joint with intentional local embed at the pivot housing.",
    )
    ctx.allow_overlap(
        upper_arm,
        shade,
        reason="The shade trunnion is simplified as a compact yoke-and-barrel hinge with intentional local embed at the tip joint.",
    )

    ctx.expect_contact(stand, lower_arm, name="shoulder pivot remains mounted")
    ctx.expect_contact(lower_arm, upper_arm, name="elbow pivot remains mounted")
    ctx.expect_contact(upper_arm, shade, name="shade hinge remains mounted")

    rest_elbow_pos = ctx.part_world_position(upper_arm)
    with ctx.pose({shoulder: shoulder.motion_limits.upper}):
        raised_elbow_pos = ctx.part_world_position(upper_arm)
    ctx.check(
        "shoulder joint raises the elbow",
        rest_elbow_pos is not None
        and raised_elbow_pos is not None
        and raised_elbow_pos[2] > rest_elbow_pos[2] + 0.22,
        details=f"rest={rest_elbow_pos}, raised={raised_elbow_pos}",
    )

    rest_shade_pos = ctx.part_world_position(shade)
    with ctx.pose({elbow: elbow.motion_limits.upper}):
        elbow_raised_shade_pos = ctx.part_world_position(shade)
    ctx.check(
        "elbow joint raises the shade",
        rest_shade_pos is not None
        and elbow_raised_shade_pos is not None
        and elbow_raised_shade_pos[2] > rest_shade_pos[2] + 0.15,
        details=f"rest={rest_shade_pos}, raised={elbow_raised_shade_pos}",
    )

    rest_shade_aabb = ctx.part_element_world_aabb(shade, elem="shade_body")
    with ctx.pose({shade_tilt: shade_tilt.motion_limits.upper}):
        tilted_shade_aabb = ctx.part_element_world_aabb(shade, elem="shade_body")
    ctx.check(
        "shade tilt can lift the lamp head",
        rest_shade_aabb is not None
        and tilted_shade_aabb is not None
        and tilted_shade_aabb[1][2] > rest_shade_aabb[1][2] + 0.05,
        details=f"rest={rest_shade_aabb}, tilted={tilted_shade_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
