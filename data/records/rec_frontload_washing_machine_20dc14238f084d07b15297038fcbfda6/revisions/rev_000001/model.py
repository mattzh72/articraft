from __future__ import annotations

from math import cos, pi, sin

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_W = 0.54
BODY_D = 0.60
BODY_H = 1.34
PANEL_T = 0.024
FRONT_Y = -BODY_D / 2.0
DOOR_CENTER_Z = 0.62
DOOR_RADIUS = 0.18
DOOR_Y = FRONT_Y - 0.066
HINGE_X = -DOOR_RADIUS
DRAWER_CENTER = (-0.095, 1.17)


def _front_panel_geometry() -> object:
    """One continuous front sheet with the porthole and drawer cut through it."""

    door_local_z = DOOR_CENTER_Z - BODY_H / 2.0
    drawer_local_z = DRAWER_CENTER[1] - BODY_H / 2.0

    panel = cq.Workplane("XY").box(BODY_W, BODY_H, PANEL_T)
    porthole_cutter = (
        cq.Workplane("XY")
        .center(0.0, door_local_z)
        .circle(0.165)
        .extrude(PANEL_T * 3.0, both=True)
    )
    drawer_cutter = (
        cq.Workplane("XY")
        .center(DRAWER_CENTER[0], drawer_local_z)
        .rect(0.315, 0.105)
        .extrude(PANEL_T * 3.0, both=True)
    )
    return panel.cut(porthole_cutter).cut(drawer_cutter)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_stacked_washer")

    white = model.material("warm_white_enamel", rgba=(0.94, 0.95, 0.92, 1.0))
    shadow = model.material("cavity_shadow", rgba=(0.03, 0.035, 0.04, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    dark_steel = model.material("dark_stainless", rgba=(0.22, 0.23, 0.23, 1.0))
    glass = model.material("smoky_glass", rgba=(0.40, 0.58, 0.70, 0.38))
    drawer_plastic = model.material("slightly_off_white_plastic", rgba=(0.88, 0.90, 0.87, 1.0))
    label_gray = model.material("soft_gray_label", rgba=(0.45, 0.48, 0.50, 1.0))

    body = model.part("body")

    # Narrow cabinet shell.  The front sheet is a single cut panel rather than
    # loose bars, so the porthole and detergent slot read as real openings.
    body.visual(
        mesh_from_cadquery(_front_panel_geometry(), "front_panel", tolerance=0.001),
        origin=Origin(xyz=(0.0, FRONT_Y, BODY_H / 2.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=white,
        name="front_panel",
    )
    body.visual(
        Box((0.026, BODY_D, BODY_H)),
        origin=Origin(xyz=(-(BODY_W - 0.026) / 2.0, 0.0, BODY_H / 2.0)),
        material=white,
        name="side_wall_0",
    )
    body.visual(
        Box((0.026, BODY_D, BODY_H)),
        origin=Origin(xyz=((BODY_W - 0.026) / 2.0, 0.0, BODY_H / 2.0)),
        material=white,
        name="side_wall_1",
    )
    body.visual(
        Box((BODY_W, BODY_D, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=white,
        name="bottom_panel",
    )
    body.visual(
        Box((BODY_W, BODY_D, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, BODY_H - 0.020)),
        material=white,
        name="top_panel",
    )
    body.visual(
        Box((BODY_W, 0.030, BODY_H)),
        origin=Origin(xyz=(0.0, BODY_D / 2.0 - 0.015, BODY_H / 2.0)),
        material=white,
        name="rear_panel",
    )

    body.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.335, 0.335),
                (0.435, 0.435),
                0.026,
                opening_shape="circle",
                outer_shape="circle",
            ),
            "porthole_bezel",
        ),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.018, DOOR_CENTER_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=white,
        name="porthole_bezel",
    )
    body.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.295, 0.295),
                (0.342, 0.342),
                0.015,
                opening_shape="circle",
                outer_shape="circle",
            ),
            "body_gasket",
        ),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.033, DOOR_CENTER_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="body_gasket",
    )
    body.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.315, 0.105),
                (0.362, 0.146),
                0.018,
                opening_shape="rounded_rect",
                outer_shape="rounded_rect",
                opening_corner_radius=0.010,
                outer_corner_radius=0.015,
            ),
            "drawer_frame",
        ),
        origin=Origin(
            xyz=(DRAWER_CENTER[0], FRONT_Y - 0.015, DRAWER_CENTER[1]),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=white,
        name="drawer_frame",
    )
    body.visual(
        Box((0.012, 0.290, 0.012)),
        origin=Origin(xyz=(DRAWER_CENTER[0] - 0.148, -0.160, DRAWER_CENTER[1] - 0.036)),
        material=steel,
        name="drawer_rail_0",
    )
    body.visual(
        Box((0.012, 0.290, 0.012)),
        origin=Origin(xyz=(DRAWER_CENTER[0] + 0.148, -0.160, DRAWER_CENTER[1] - 0.036)),
        material=steel,
        name="drawer_rail_1",
    )
    body.visual(
        Box((0.250, 0.010, 0.090)),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.017, DOOR_CENTER_Z + 0.288)),
        material=label_gray,
        name="control_label",
    )
    body.visual(
        Box((0.160, 0.006, 0.050)),
        origin=Origin(xyz=(0.080, FRONT_Y - 0.021, DOOR_CENTER_Z + 0.287)),
        material=shadow,
        name="display_window",
    )
    body.visual(
        Box((0.335, 0.250, 0.012)),
        origin=Origin(xyz=(DRAWER_CENTER[0], -0.165, DRAWER_CENTER[1] - 0.047)),
        material=steel,
        name="drawer_shelf",
    )
    body.visual(
        Box((0.095, 0.120, 0.095)),
        origin=Origin(xyz=(0.0, 0.225, DOOR_CENTER_Z)),
        material=steel,
        name="rear_bearing_mount",
    )
    body.visual(
        Cylinder(radius=0.056, length=0.030),
        origin=Origin(xyz=(0.0, 0.175, DOOR_CENTER_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="rear_bearing",
    )

    # Two exposed hinge stacks on the left edge of the round door.  Each stack
    # has fixed outer knuckles on the cabinet and a moving center knuckle on the
    # door, leaving small axial clearances rather than overlapping barrels.
    for idx, hinge_z in enumerate((DOOR_CENTER_Z + 0.135, DOOR_CENTER_Z - 0.135)):
        for seg_idx, z_offset in enumerate((0.030, -0.030)):
            body.visual(
                Box((0.030, 0.060, 0.026)),
                origin=Origin(xyz=(HINGE_X - 0.018, FRONT_Y - 0.036, hinge_z + z_offset)),
                material=steel,
                name=f"hinge_leaf_{idx}_{seg_idx}",
            )
            body.visual(
                Cylinder(radius=0.011, length=0.026),
                origin=Origin(xyz=(HINGE_X, DOOR_Y, hinge_z + z_offset)),
                material=steel,
                name=f"hinge_knuckle_{idx}_{seg_idx}",
            )

    for idx, foot_x in enumerate((-0.190, 0.190)):
        body.visual(
            Cylinder(radius=0.035, length=0.022),
            origin=Origin(xyz=(foot_x, FRONT_Y + 0.070, -0.011)),
            material=rubber,
            name=f"front_foot_{idx}",
        )

    drum = model.part("drum")
    drum.visual(
        Cylinder(radius=0.205, length=0.380),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="drum_shell",
    )
    drum.visual(
        Cylinder(radius=0.168, length=0.008),
        origin=Origin(xyz=(0.0, -0.188, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=shadow,
        name="drum_mouth",
    )
    drum.visual(
        Cylinder(radius=0.052, length=0.040),
        origin=Origin(xyz=(0.0, -0.198, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="center_hub",
    )
    for idx, angle in enumerate((0.0, 2.0 * pi / 3.0, 4.0 * pi / 3.0)):
        x = 0.165 * sin(angle)
        z = 0.165 * cos(angle)
        drum.visual(
            Box((0.030, 0.285, 0.026)),
            origin=Origin(xyz=(x, 0.0, z), rpy=(0.0, angle, 0.0)),
            material=steel,
            name=f"drum_lifter_{idx}",
        )

    door = model.part("porthole_door")
    door.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.244, 0.244),
                (0.360, 0.360),
                0.042,
                opening_shape="circle",
                outer_shape="circle",
            ),
            "door_ring",
        ),
        origin=Origin(xyz=(DOOR_RADIUS, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=white,
        name="door_ring",
    )
    door.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.236, 0.236),
                (0.292, 0.292),
                0.018,
                opening_shape="circle",
                outer_shape="circle",
            ),
            "door_gasket",
        ),
        origin=Origin(xyz=(DOOR_RADIUS, -0.010, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="door_gasket",
    )
    door.visual(
        Cylinder(radius=0.125, length=0.010),
        origin=Origin(xyz=(DOOR_RADIUS, -0.015, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=glass,
        name="glass_bowl",
    )
    for idx, local_z in enumerate((0.135, -0.135)):
        door.visual(
            Box((0.086, 0.012, 0.032)),
            origin=Origin(xyz=(0.043, 0.0, local_z)),
            material=steel,
            name=f"door_hinge_leaf_{idx}",
        )
        door.visual(
            Cylinder(radius=0.011, length=0.034),
            origin=Origin(xyz=(0.0, 0.0, local_z)),
            material=steel,
            name=f"door_hinge_knuckle_{idx}",
        )

    drawer = model.part("detergent_drawer")
    drawer.visual(
        Box((0.285, 0.026, 0.090)),
        origin=Origin(),
        material=drawer_plastic,
        name="drawer_fascia",
    )
    drawer.visual(
        Box((0.160, 0.004, 0.014)),
        origin=Origin(xyz=(0.0, -0.015, 0.010)),
        material=shadow,
        name="drawer_pull_slot",
    )
    drawer.visual(
        Box((0.245, 0.320, 0.008)),
        origin=Origin(xyz=(0.0, 0.155, -0.037)),
        material=drawer_plastic,
        name="drawer_bottom",
    )
    drawer.visual(
        Box((0.010, 0.320, 0.052)),
        origin=Origin(xyz=(-0.123, 0.155, -0.014)),
        material=drawer_plastic,
        name="drawer_side_0",
    )
    drawer.visual(
        Box((0.010, 0.320, 0.052)),
        origin=Origin(xyz=(0.123, 0.155, -0.014)),
        material=drawer_plastic,
        name="drawer_side_1",
    )
    drawer.visual(
        Box((0.245, 0.010, 0.052)),
        origin=Origin(xyz=(0.0, 0.312, -0.014)),
        material=drawer_plastic,
        name="drawer_back",
    )
    drawer.visual(
        Box((0.008, 0.240, 0.043)),
        origin=Origin(xyz=(-0.045, 0.170, -0.0135)),
        material=label_gray,
        name="drawer_divider",
    )

    model.articulation(
        "body_to_drum",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=drum,
        origin=Origin(xyz=(0.0, -0.030, DOOR_CENTER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0),
    )
    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(HINGE_X, DOOR_Y, DOOR_CENTER_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=0.0, upper=1.85),
    )
    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(DRAWER_CENTER[0], FRONT_Y - 0.037, DRAWER_CENTER[1])),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.35, lower=0.0, upper=0.18),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    drum = object_model.get_part("drum")
    door = object_model.get_part("porthole_door")
    drawer = object_model.get_part("detergent_drawer")
    drum_joint = object_model.get_articulation("body_to_drum")
    door_joint = object_model.get_articulation("body_to_door")
    drawer_joint = object_model.get_articulation("body_to_drawer")

    def axis_value(vec, axis_index: int):
        try:
            return vec[axis_index]
        except TypeError:
            return (vec.x, vec.y, vec.z)[axis_index]

    ctx.check(
        "primary mechanisms are articulated",
        drum_joint.articulation_type == ArticulationType.CONTINUOUS
        and door_joint.articulation_type == ArticulationType.REVOLUTE
        and drawer_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"types={[drum_joint.articulation_type, door_joint.articulation_type, drawer_joint.articulation_type]}",
    )
    ctx.check(
        "door and drawer have realistic travel",
        door_joint.motion_limits is not None
        and door_joint.motion_limits.upper is not None
        and door_joint.motion_limits.upper > 1.5
        and drawer_joint.motion_limits is not None
        and drawer_joint.motion_limits.upper is not None
        and 0.15 <= drawer_joint.motion_limits.upper <= 0.22,
        details=f"door_limits={door_joint.motion_limits}, drawer_limits={drawer_joint.motion_limits}",
    )

    ctx.expect_overlap(
        door,
        body,
        axes="xz",
        elem_a="door_ring",
        elem_b="porthole_bezel",
        min_overlap=0.30,
        name="round door is centered over porthole bezel",
    )
    ctx.expect_gap(
        body,
        door,
        axis="y",
        positive_elem="body_gasket",
        negative_elem="door_ring",
        min_gap=0.003,
        max_gap=0.020,
        name="closed door sits just proud of front gasket",
    )
    ctx.expect_contact(
        door,
        body,
        elem_a="door_hinge_knuckle_0",
        elem_b="hinge_knuckle_0_0",
        contact_tol=0.002,
        name="upper barrel hinge knuckles meet on the hinge line",
    )
    ctx.expect_contact(
        drum,
        body,
        elem_a="drum_shell",
        elem_b="rear_bearing",
        contact_tol=0.002,
        name="drum is carried on rear bearing axle",
    )
    ctx.expect_contact(
        drawer,
        body,
        elem_a="drawer_bottom",
        elem_b="drawer_shelf",
        contact_tol=0.002,
        name="detergent drawer rides on its shelf",
    )
    ctx.expect_within(
        drawer,
        body,
        axes="xz",
        inner_elem="drawer_bottom",
        outer_elem="drawer_frame",
        margin=0.001,
        name="drawer tray fits inside top front slot",
    )
    ctx.expect_overlap(
        drawer,
        body,
        axes="y",
        elem_a="drawer_bottom",
        elem_b="drawer_frame",
        min_overlap=0.015,
        name="closed drawer remains inserted through frame",
    )

    closed_door_aabb = ctx.part_world_aabb(door)
    closed_drawer_aabb = ctx.part_world_aabb(drawer)
    with ctx.pose({door_joint: 1.25, drawer_joint: 0.18, drum_joint: 0.8}):
        open_door_aabb = ctx.part_world_aabb(door)
        extended_drawer_aabb = ctx.part_world_aabb(drawer)
        ctx.expect_overlap(
            drawer,
            body,
            axes="y",
            elem_a="drawer_bottom",
            elem_b="drawer_frame",
            min_overlap=0.015,
            name="extended drawer keeps retained insertion",
        )

    ctx.check(
        "door swings outward from left hinge",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and axis_value(open_door_aabb[0], 1) < axis_value(closed_door_aabb[0], 1) - 0.10,
        details=f"closed={closed_door_aabb}, opened={open_door_aabb}",
    )
    ctx.check(
        "drawer slides outward at top front",
        closed_drawer_aabb is not None
        and extended_drawer_aabb is not None
        and axis_value(extended_drawer_aabb[0], 1) < axis_value(closed_drawer_aabb[0], 1) - 0.15,
        details=f"closed={closed_drawer_aabb}, extended={extended_drawer_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
