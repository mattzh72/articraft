from __future__ import annotations

import math

import cadquery as cq

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
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

BODY_W = 0.54
BODY_H = 0.54
BODY_D = 0.17
BODY_WALL = 0.024
INNER_W = BODY_W - 2.0 * BODY_WALL
INNER_H = BODY_H - 2.0 * BODY_WALL

GRILLE_T = 0.0055
GRILLE_FRAME = 0.018
GUARD_RING_OUTER_R = 0.206
GUARD_RING_INNER_R = 0.194
HUB_R = 0.030
SPOKE_W = 0.010

ROTOR_RADIUS = 0.182
ROTOR_THICKNESS = 0.016
ROTOR_SHAFT_R = 0.005
ROTOR_SHAFT_L = 0.025

MOTOR_POD_R = 0.047
MOTOR_POD_L = 0.041

KNOB_Y = 0.125
KNOB_Z = 0.0
KNOB_COLLAR_R = 0.024
KNOB_COLLAR_L = 0.008
KNOB_PAD_T = 0.003
KNOB_HOLE_R = 0.0085
KNOB_SHAFT_R = 0.0048
KNOB_SHAFT_L = 0.010
KNOB_D = 0.038
KNOB_L = 0.027

FOOT_HINGE_Y = -BODY_H / 2.0 - 0.010
FOOT_HINGE_Z = BODY_D / 2.0 + 0.007
FOOT_X_OFFSET = 0.165
FOOT_BARREL_R = 0.0065
FOOT_BARREL_L = 0.060
FOOT_ARM_W = 0.056
FOOT_ARM_T = 0.009
FOOT_ARM_L = 0.078
FOOT_PAD_W = 0.088
FOOT_PAD_T = 0.015
FOOT_PAD_L = 0.030
FOOT_FOLD_LIMIT = -0.92


def _rotated_bar(length: float, width: float, thickness: float, angle_deg: float) -> cq.Workplane:
    return cq.Workplane("XY").box(length, width, thickness).rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)


def _grille_panel(z_center: float, hub_radius: float, outer_extra: float = 0.0) -> cq.Workplane:
    frame = cq.Workplane("XY").box(INNER_W + outer_extra, INNER_H + outer_extra, GRILLE_T).cut(
        cq.Workplane("XY").box(
            INNER_W - 2.0 * GRILLE_FRAME,
            INNER_H - 2.0 * GRILLE_FRAME,
            GRILLE_T + 0.003,
        )
    )
    guard_ring = (
        cq.Workplane("XY")
        .circle(GUARD_RING_OUTER_R)
        .extrude(GRILLE_T / 2.0, both=True)
        .cut(cq.Workplane("XY").circle(GUARD_RING_INNER_R).extrude(GRILLE_T / 2.0 + 0.002, both=True))
    )
    hub = cq.Workplane("XY").circle(hub_radius).extrude(GRILLE_T / 2.0, both=True)

    panel = frame.union(guard_ring).union(hub)
    for angle_deg in (0.0, 45.0, 90.0, 135.0):
        panel = panel.union(_rotated_bar(GUARD_RING_INNER_R * 2.0, SPOKE_W, GRILLE_T, angle_deg))

    bridge_x = INNER_W / 2.0 - GUARD_RING_OUTER_R + 0.026
    bridge_y = INNER_H / 2.0 - GUARD_RING_OUTER_R + 0.026
    bridge_offset_x = GUARD_RING_OUTER_R + bridge_x / 2.0 - 0.013
    bridge_offset_y = GUARD_RING_OUTER_R + bridge_y / 2.0 - 0.013
    for sign in (-1.0, 1.0):
        panel = panel.union(
            cq.Workplane("XY").box(bridge_x, SPOKE_W, GRILLE_T).translate((sign * bridge_offset_x, 0.0, 0.0))
        )
        panel = panel.union(
            cq.Workplane("XY").box(SPOKE_W, bridge_y, GRILLE_T).translate((0.0, sign * bridge_offset_y, 0.0))
        )

    return panel.translate((0.0, 0.0, z_center))


def _body_mesh() -> cq.Workplane:
    shell = cq.Workplane("XY").box(BODY_W, BODY_H, BODY_D).cut(
        cq.Workplane("XY").box(INNER_W, INNER_H, BODY_D + 0.004)
    )
    front_grille = _grille_panel(BODY_D / 2.0 - BODY_WALL + GRILLE_T / 2.0, HUB_R, outer_extra=0.004)
    rear_grille = _grille_panel(-BODY_D / 2.0 + BODY_WALL - GRILLE_T / 2.0, HUB_R + 0.008, outer_extra=0.004)
    motor_pod = cq.Workplane("XY").circle(MOTOR_POD_R).extrude(0.028).translate(
        (0.0, 0.0, -BODY_D / 2.0 + BODY_WALL + 0.028 / 2.0 - 0.006)
    )
    return shell.union(front_grille).union(rear_grille).union(motor_pod)


def _knob_mount_mesh() -> cq.Workplane:
    pad = cq.Workplane("XY").box(KNOB_PAD_T, 0.070, 0.060).translate(
        (BODY_W / 2.0 + KNOB_PAD_T / 2.0, KNOB_Y, KNOB_Z)
    )
    collar = (
        cq.Workplane("YZ", origin=(BODY_W / 2.0 + KNOB_PAD_T, KNOB_Y, KNOB_Z))
        .circle(KNOB_COLLAR_R)
        .extrude(KNOB_COLLAR_L)
    )
    bore = (
        cq.Workplane("YZ", origin=(BODY_W / 2.0, KNOB_Y, KNOB_Z))
        .circle(KNOB_HOLE_R)
        .extrude(KNOB_PAD_T + KNOB_COLLAR_L + 0.002)
    )
    return pad.union(collar).cut(bore)


def _foot_mount_mesh() -> cq.Workplane:
    rail = cq.Workplane("XY").box(BODY_W - 0.080, 0.010, 0.012).translate(
        (0.0, -BODY_H / 2.0 - 0.005, FOOT_HINGE_Z - 0.014)
    )
    mount = rail
    for x_center in (-FOOT_X_OFFSET, FOOT_X_OFFSET):
        for x_shift in (-0.036, 0.036):
            lug = cq.Workplane("XY").box(0.012, 0.014, 0.012).translate(
                (
                    x_center + x_shift,
                    -BODY_H / 2.0 - 0.0105,
                    FOOT_HINGE_Z - 0.010,
                )
            )
            mount = mount.union(lug)
    return mount


def _foot_mesh() -> cq.Workplane:
    barrel = cq.Workplane("YZ").circle(FOOT_BARREL_R).extrude(FOOT_BARREL_L / 2.0, both=True)
    arm = cq.Workplane("XY").box(FOOT_ARM_W, FOOT_ARM_T, FOOT_ARM_L).translate(
        (0.0, -0.008, FOOT_ARM_L / 2.0 - 0.002)
    )
    pad = cq.Workplane("XY").box(FOOT_PAD_W, FOOT_PAD_T, FOOT_PAD_L).translate(
        (0.0, -0.012, FOOT_ARM_L + FOOT_PAD_L / 2.0 - 0.004)
    )
    toe = cq.Workplane("XY").box(FOOT_PAD_W * 0.82, 0.007, 0.010).translate(
        (0.0, -0.006, FOOT_ARM_L + FOOT_PAD_L - 0.005)
    )
    return barrel.union(arm).union(pad).union(toe)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="kitchen_window_box_fan")

    housing_white = model.material("housing_white", rgba=(0.91, 0.92, 0.89, 1.0))
    rotor_gray = model.material("rotor_gray", rgba=(0.23, 0.25, 0.28, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.36, 0.39, 0.42, 1.0))
    knob_dark = model.material("knob_dark", rgba=(0.14, 0.15, 0.17, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_mesh(), "box_fan_body"),
        material=housing_white,
        name="body_shell",
    )

    foot_mount = model.part("foot_mount")
    foot_mount.visual(
        mesh_from_cadquery(_foot_mount_mesh(), "foot_mount"),
        material=housing_white,
        name="mount_shell",
    )

    knob_mount = model.part("knob_mount")
    knob_mount.visual(
        mesh_from_cadquery(_knob_mount_mesh(), "knob_mount"),
        material=housing_white,
        name="mount_shell",
    )

    rotor = model.part("rotor")
    rotor.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                ROTOR_RADIUS,
                0.048,
                5,
                thickness=ROTOR_THICKNESS,
                blade_pitch_deg=23.0,
                blade_sweep_deg=18.0,
                blade=FanRotorBlade(shape="broad", tip_pitch_deg=12.0, camber=0.10),
                hub=FanRotorHub(style="domed", rear_collar_height=0.006, rear_collar_radius=0.022, bore_diameter=0.010),
            ),
            "box_fan_rotor",
        ),
        material=rotor_gray,
        name="rotor_shell",
    )
    rotor.visual(
        Cylinder(radius=ROTOR_SHAFT_R, length=ROTOR_SHAFT_L),
        origin=Origin(xyz=(0.0, 0.0, -ROTOR_SHAFT_L / 2.0)),
        material=trim_gray,
        name="rotor_axle",
    )

    side_knob = model.part("side_knob")
    side_knob.visual(
        Cylinder(radius=KNOB_SHAFT_R, length=KNOB_SHAFT_L),
        origin=Origin(xyz=(-KNOB_SHAFT_L / 2.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_gray,
        name="knob_shaft",
    )
    side_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                KNOB_D,
                KNOB_L,
                body_style="skirted",
                top_diameter=0.030,
                skirt=KnobSkirt(0.044, 0.005, flare=0.05),
                grip=KnobGrip(style="fluted", count=18, depth=0.0010),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007, angle_deg=0.0),
                center=False,
            ),
            "box_fan_knob",
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_dark,
        name="knob_shell",
    )

    left_foot = model.part("left_foot")
    left_foot.visual(
        mesh_from_cadquery(_foot_mesh(), "left_foot"),
        material=trim_gray,
        name="foot_shell",
    )

    right_foot = model.part("right_foot")
    right_foot.visual(
        mesh_from_cadquery(_foot_mesh(), "right_foot"),
        material=trim_gray,
        name="foot_shell",
    )

    model.articulation(
        "body_to_foot_mount",
        ArticulationType.FIXED,
        parent=body,
        child=foot_mount,
        origin=Origin(),
    )
    model.articulation(
        "body_to_knob_mount",
        ArticulationType.FIXED,
        parent=body,
        child=knob_mount,
        origin=Origin(),
    )

    model.articulation(
        "body_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=rotor,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=28.0),
    )
    model.articulation(
        "body_to_side_knob",
        ArticulationType.CONTINUOUS,
        parent=knob_mount,
        child=side_knob,
        origin=Origin(xyz=(BODY_W / 2.0 + KNOB_PAD_T + KNOB_COLLAR_L, KNOB_Y, KNOB_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=8.0),
    )
    model.articulation(
        "foot_mount_to_left_foot",
        ArticulationType.REVOLUTE,
        parent=foot_mount,
        child=left_foot,
        origin=Origin(xyz=(-FOOT_X_OFFSET, FOOT_HINGE_Y, FOOT_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=FOOT_FOLD_LIMIT, upper=0.0),
    )
    model.articulation(
        "foot_mount_to_right_foot",
        ArticulationType.REVOLUTE,
        parent=foot_mount,
        child=right_foot,
        origin=Origin(xyz=(FOOT_X_OFFSET, FOOT_HINGE_Y, FOOT_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=FOOT_FOLD_LIMIT, upper=0.0),
    )

    return model


def _size_from_aabb(aabb):
    min_pt, max_pt = aabb
    return tuple(float(max_pt[i] - min_pt[i]) for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    rotor = object_model.get_part("rotor")
    side_knob = object_model.get_part("side_knob")
    left_foot = object_model.get_part("left_foot")
    right_foot = object_model.get_part("right_foot")

    rotor_joint = object_model.get_articulation("body_to_rotor")
    knob_joint = object_model.get_articulation("body_to_side_knob")
    left_hinge = object_model.get_articulation("foot_mount_to_left_foot")
    right_hinge = object_model.get_articulation("foot_mount_to_right_foot")

    body_aabb = ctx.part_world_aabb(body)
    rotor_aabb = ctx.part_world_aabb(rotor)
    knob_aabb = ctx.part_world_aabb(side_knob)
    left_aabb = ctx.part_world_aabb(left_foot)
    right_aabb = ctx.part_world_aabb(right_foot)

    ctx.check("body_aabb_present", body_aabb is not None, "Expected a world AABB for the fan body.")
    ctx.check("rotor_aabb_present", rotor_aabb is not None, "Expected a world AABB for the propeller.")
    ctx.check("knob_aabb_present", knob_aabb is not None, "Expected a world AABB for the side knob.")
    ctx.check("feet_aabb_present", left_aabb is not None and right_aabb is not None, "Expected world AABBs for both stabilizer feet.")

    if body_aabb is None or rotor_aabb is None or knob_aabb is None or left_aabb is None or right_aabb is None:
        return ctx.report()

    body_size = _size_from_aabb(body_aabb)
    rotor_size = _size_from_aabb(rotor_aabb)

    ctx.check("window_fan_width", 0.50 <= body_size[0] <= 0.58, details=f"size={body_size!r}")
    ctx.check("window_fan_height", 0.50 <= body_size[1] <= 0.58, details=f"size={body_size!r}")
    ctx.check("window_fan_depth", 0.14 <= body_size[2] <= 0.20, details=f"size={body_size!r}")
    ctx.check("rotor_fills_opening", 0.34 <= max(rotor_size[0], rotor_size[1]) <= 0.39, details=f"size={rotor_size!r}")

    body_min, body_max = body_aabb
    knob_min, knob_max = knob_aabb
    left_min, left_max = left_aabb
    right_min, right_max = right_aabb

    ctx.check(
        "knob_sits_on_one_side",
        knob_max[0] > body_max[0] + 0.010 and knob_min[1] > 0.02,
        details=f"body_max={body_max!r}, knob_min={knob_min!r}, knob_max={knob_max!r}",
    )
    ctx.check(
        "feet_deployed_below_housing",
        left_max[1] < -BODY_H / 2.0 - 0.001 and right_max[1] < -BODY_H / 2.0 - 0.001,
        details=f"left_max={left_max!r}, right_max={right_max!r}",
    )
    ctx.check(
        "feet_extend_in_front",
        left_max[2] > body_max[2] + 0.040 and right_max[2] > body_max[2] + 0.040,
        details=f"body_max={body_max!r}, left_max={left_max!r}, right_max={right_max!r}",
    )

    ctx.expect_within(
        rotor,
        body,
        axes="xy",
        margin=0.015,
        name="rotor_stays_within_body_face",
    )

    ctx.check(
        "rotor_joint_is_continuous",
        rotor_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(rotor_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={rotor_joint.articulation_type!r}, axis={tuple(rotor_joint.axis)!r}",
    )
    ctx.check(
        "knob_joint_is_continuous",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(knob_joint.axis) == (1.0, 0.0, 0.0),
        details=f"type={knob_joint.articulation_type!r}, axis={tuple(knob_joint.axis)!r}",
    )

    rotor_pos_rest = ctx.part_world_position(rotor)
    knob_pos_rest = ctx.part_world_position(side_knob)
    with ctx.pose({rotor_joint: 1.4, knob_joint: 0.9}):
        rotor_pos_spun = ctx.part_world_position(rotor)
        knob_pos_spun = ctx.part_world_position(side_knob)
    ctx.check(
        "continuous_parts_spin_in_place",
        rotor_pos_rest == rotor_pos_spun and knob_pos_rest == knob_pos_spun,
        details=f"rotor_rest={rotor_pos_rest!r}, rotor_spun={rotor_pos_spun!r}, knob_rest={knob_pos_rest!r}, knob_spun={knob_pos_spun!r}",
    )

    folded_pose = {
        left_hinge: left_hinge.motion_limits.lower,
        right_hinge: right_hinge.motion_limits.lower,
    }
    with ctx.pose(folded_pose):
        left_folded = ctx.part_world_aabb(left_foot)
        right_folded = ctx.part_world_aabb(right_foot)
    ctx.check("folded_feet_aabb_present", left_folded is not None and right_folded is not None, "Expected folded-pose AABBs for the stabilizer feet.")
    if left_folded is not None and right_folded is not None:
        _, left_folded_max = left_folded
        _, right_folded_max = right_folded
        ctx.check(
            "feet_can_fold_upward",
            left_folded_max[2] < left_max[2] - 0.020
            and right_folded_max[2] < right_max[2] - 0.020
            and left_folded_max[1] > left_max[1] + 0.030
            and right_folded_max[1] > right_max[1] + 0.030,
            details=(
                f"deployed_left={left_max!r}, folded_left={left_folded_max!r}, "
                f"deployed_right={right_max!r}, folded_right={right_folded_max!r}"
            ),
        )

    return ctx.report()


object_model = build_object_model()
