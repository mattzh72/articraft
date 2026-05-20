from __future__ import annotations

from math import pi

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_W = 0.68
BODY_D = 0.72
BODY_H = 1.05
PANEL_T = 0.028
FRONT_Y = -BODY_D / 2.0
REAR_Y = BODY_D / 2.0
DRUM_Z = 0.56
DOOR_HINGE_X = 0.31
DOOR_HINGE_Y = FRONT_Y - 0.035
TOP_HINGE_Y = REAR_Y - 0.055


def _annular_cylinder(outer_radius: float, inner_radius: float, length: float) -> cq.Workplane:
    """CadQuery annular cylinder centered on local Z."""
    outer = cq.Workplane("XY").circle(outer_radius).extrude(length).translate((0, 0, -length / 2))
    inner = (
        cq.Workplane("XY")
        .circle(inner_radius)
        .extrude(length + 0.004)
        .translate((0, 0, -(length + 0.004) / 2))
    )
    return outer.cut(inner)


def _drum_shell() -> cq.Workplane:
    """A hollow, ribbed metal dryer drum centered on local Z before rotation."""
    radius = 0.252
    length = 0.48
    shell = _annular_cylinder(radius, radius - 0.018, length)

    # Three raised tumbling baffles are unioned to the inner drum wall so the
    # rotating drum reads as one manufactured part rather than a plain tube.
    for angle_deg in (90, 210, 330):
        rib = (
            cq.Workplane("XY")
            .box(0.050, 0.040, length * 0.86)
            .translate((0.0, radius - 0.018 - 0.020, 0.0))
            .rotate((0, 0, 0), (0, 0, 1), angle_deg)
        )
        shell = shell.union(rib)

    front_lip = _annular_cylinder(radius + 0.012, radius - 0.030, 0.035).translate((0, 0, -length / 2 + 0.018))
    rear_lip = _annular_cylinder(radius + 0.006, radius - 0.026, 0.026).translate((0, 0, length / 2 - 0.013))
    return shell.union(front_lip).union(rear_lip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wifi_smart_dryer")

    white = model.material("warm_white_enamel", rgba=(0.94, 0.95, 0.93, 1.0))
    edge_white = model.material("soft_white_edges", rgba=(0.86, 0.88, 0.86, 1.0))
    dark = model.material("black_glass", rgba=(0.02, 0.025, 0.03, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    blue = model.material("wifi_blue", rgba=(0.05, 0.45, 1.0, 1.0))
    glass = model.material("smoked_window", rgba=(0.35, 0.56, 0.68, 0.36))
    rubber = model.material("dark_rubber", rgba=(0.01, 0.01, 0.012, 1.0))

    cabinet = model.part("cabinet")

    # Thin rectangular panels leave a real central porthole opening and a top
    # service opening instead of hiding the drum inside a solid block.
    cabinet.visual(
        Box((PANEL_T, BODY_D, BODY_H)),
        origin=Origin(xyz=(-BODY_W / 2 + PANEL_T / 2, 0.0, BODY_H / 2)),
        material=white,
        name="side_panel_0",
    )
    cabinet.visual(
        Box((PANEL_T, BODY_D, BODY_H)),
        origin=Origin(xyz=(BODY_W / 2 - PANEL_T / 2, 0.0, BODY_H / 2)),
        material=white,
        name="side_panel_1",
    )
    cabinet.visual(
        Box((BODY_W, PANEL_T, BODY_H)),
        origin=Origin(xyz=(0.0, REAR_Y - PANEL_T / 2, BODY_H / 2)),
        material=white,
        name="rear_panel",
    )
    cabinet.visual(
        Box((BODY_W, BODY_D, PANEL_T)),
        origin=Origin(xyz=(0.0, 0.0, PANEL_T / 2)),
        material=white,
        name="floor_panel",
    )

    access_w = 0.50
    access_d = 0.36
    access_y = 0.10
    cabinet.visual(
        Box((BODY_W, TOP_HINGE_Y - access_y - access_d / 2, PANEL_T)),
        origin=Origin(xyz=(0.0, (TOP_HINGE_Y + access_y + access_d / 2) / 2, BODY_H - PANEL_T / 2)),
        material=white,
        name="top_rear_panel",
    )
    cabinet.visual(
        Box((BODY_W, access_y - access_d / 2 - FRONT_Y, PANEL_T)),
        origin=Origin(xyz=(0.0, (FRONT_Y + access_y - access_d / 2) / 2, BODY_H - PANEL_T / 2)),
        material=white,
        name="top_front_panel",
    )
    cabinet.visual(
        Box(((BODY_W - access_w) / 2, access_d, PANEL_T)),
        origin=Origin(xyz=(-(BODY_W + access_w) / 4, access_y, BODY_H - PANEL_T / 2)),
        material=white,
        name="top_side_panel_0",
    )
    cabinet.visual(
        Box(((BODY_W - access_w) / 2, access_d, PANEL_T)),
        origin=Origin(xyz=((BODY_W + access_w) / 4, access_y, BODY_H - PANEL_T / 2)),
        material=white,
        name="top_side_panel_1",
    )

    front_z_min = 0.06
    port_radius = 0.315
    front_center_y = FRONT_Y + PANEL_T / 2
    cabinet.visual(
        Box((BODY_W, PANEL_T, DRUM_Z - port_radius - front_z_min)),
        origin=Origin(xyz=(0.0, front_center_y, (DRUM_Z - port_radius + front_z_min) / 2)),
        material=white,
        name="front_lower_panel",
    )
    cabinet.visual(
        Box((BODY_W, PANEL_T, BODY_H - (DRUM_Z + port_radius))),
        origin=Origin(xyz=(0.0, front_center_y, (BODY_H + DRUM_Z + port_radius) / 2)),
        material=white,
        name="front_upper_panel",
    )
    side_strip_w = (BODY_W - 2 * port_radius) / 2
    cabinet.visual(
        Box((side_strip_w, PANEL_T, 2 * port_radius)),
        origin=Origin(xyz=(-(BODY_W - side_strip_w) / 2, front_center_y, DRUM_Z)),
        material=white,
        name="front_side_panel_0",
    )
    cabinet.visual(
        Box((side_strip_w, PANEL_T, 2 * port_radius)),
        origin=Origin(xyz=((BODY_W - side_strip_w) / 2, front_center_y, DRUM_Z)),
        material=white,
        name="front_side_panel_1",
    )

    # Toe plinth, feet, and control fascia.
    cabinet.visual(
        Box((BODY_W * 0.86, 0.055, 0.060)),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.010, 0.055)),
        material=edge_white,
        name="toe_plinth",
    )
    for i, x in enumerate((-0.24, 0.24)):
        cabinet.visual(
            Cylinder(radius=0.035, length=0.030),
            origin=Origin(xyz=(x, -0.22, 0.015)),
            material=rubber,
            name=f"front_foot_{i}",
        )
        cabinet.visual(
            Cylinder(radius=0.032, length=0.030),
            origin=Origin(xyz=(x, 0.25, 0.015)),
            material=rubber,
            name=f"rear_foot_{i}",
        )
    cabinet.visual(
        Box((0.56, 0.012, 0.105)),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.006, 0.955)),
        material=dark,
        name="control_fascia",
    )
    cabinet.visual(
        Box((0.16, 0.006, 0.050)),
        origin=Origin(xyz=(-0.055, FRONT_Y - 0.014, 0.960)),
        material=Material("dim_display", rgba=(0.0, 0.08, 0.11, 1.0)),
        name="status_display",
    )
    cabinet.visual(
        Box((0.012, 0.004, 0.032)),
        origin=Origin(xyz=(-0.145, FRONT_Y - 0.018, 0.965)),
        material=blue,
        name="wifi_mark",
    )
    cabinet.visual(
        Box((0.032, 0.004, 0.008)),
        origin=Origin(xyz=(-0.145, FRONT_Y - 0.019, 0.948)),
        material=blue,
        name="wifi_base",
    )

    # Stationary hinge hardware mounted to the cabinet.
    cabinet.visual(
        Box((0.030, 0.090, 0.105)),
        origin=Origin(xyz=(DOOR_HINGE_X + 0.018, DOOR_HINGE_Y, DRUM_Z - 0.19)),
        material=steel,
        name="door_hinge_leaf_0",
    )
    cabinet.visual(
        Cylinder(radius=0.010, length=0.130),
        origin=Origin(xyz=(DOOR_HINGE_X, DOOR_HINGE_Y, DRUM_Z - 0.19)),
        material=steel,
        name="door_hinge_pin_0",
    )
    cabinet.visual(
        Box((0.030, 0.090, 0.105)),
        origin=Origin(xyz=(DOOR_HINGE_X + 0.018, DOOR_HINGE_Y, DRUM_Z + 0.19)),
        material=steel,
        name="door_hinge_leaf_1",
    )
    cabinet.visual(
        Cylinder(radius=0.010, length=0.130),
        origin=Origin(xyz=(DOOR_HINGE_X, DOOR_HINGE_Y, DRUM_Z + 0.19)),
        material=steel,
        name="door_hinge_pin_1",
    )
    cabinet.visual(
        Box((0.135, 0.034, 0.030)),
        origin=Origin(xyz=(-0.18, TOP_HINGE_Y + 0.012, BODY_H - 0.010)),
        material=steel,
        name="cover_hinge_leaf_0",
    )
    cabinet.visual(
        Cylinder(radius=0.009, length=0.145),
        origin=Origin(xyz=(-0.18, TOP_HINGE_Y, BODY_H + 0.010), rpy=(0.0, pi / 2, 0.0)),
        material=steel,
        name="cover_hinge_pin_0",
    )
    cabinet.visual(
        Box((0.135, 0.034, 0.030)),
        origin=Origin(xyz=(0.18, TOP_HINGE_Y + 0.012, BODY_H - 0.010)),
        material=steel,
        name="cover_hinge_leaf_1",
    )
    cabinet.visual(
        Cylinder(radius=0.009, length=0.145),
        origin=Origin(xyz=(0.18, TOP_HINGE_Y, BODY_H + 0.010), rpy=(0.0, pi / 2, 0.0)),
        material=steel,
        name="cover_hinge_pin_1",
    )
    cabinet.visual(
        Cylinder(radius=0.056, length=0.115),
        origin=Origin(xyz=(0.0, 0.286, DRUM_Z), rpy=(pi / 2, 0.0, 0.0)),
        material=steel,
        name="rear_bearing",
    )

    drum = model.part("drum")
    drum.visual(
        mesh_from_cadquery(_drum_shell(), "ribbed_drum", tolerance=0.0025, angular_tolerance=0.08),
        origin=Origin(rpy=(pi / 2, 0.0, 0.0)),
        material=steel,
        name="drum_shell",
    )
    drum.visual(
        Cylinder(radius=0.045, length=0.075),
        origin=Origin(xyz=(0.0, 0.225, 0.0), rpy=(pi / 2, 0.0, 0.0)),
        material=steel,
        name="rear_hub",
    )
    drum.visual(
        Box((0.205, 0.026, 0.028)),
        origin=Origin(xyz=(0.120, 0.202, 0.0)),
        material=steel,
        name="rear_spoke_0",
    )
    drum.visual(
        Box((0.205, 0.026, 0.028)),
        origin=Origin(xyz=(-0.120, 0.202, 0.0)),
        material=steel,
        name="rear_spoke_1",
    )
    drum.visual(
        Box((0.028, 0.026, 0.205)),
        origin=Origin(xyz=(0.0, 0.202, 0.120)),
        material=steel,
        name="rear_spoke_2",
    )

    door = model.part("door")
    door.visual(
        mesh_from_cadquery(_annular_cylinder(0.300, 0.205, 0.055), "porthole_door_ring", tolerance=0.002),
        origin=Origin(xyz=(-0.310, 0.0, 0.0), rpy=(pi / 2, 0.0, 0.0)),
        material=white,
        name="door_ring",
    )
    door.visual(
        Cylinder(radius=0.212, length=0.010),
        origin=Origin(xyz=(-0.310, -0.020, 0.0), rpy=(pi / 2, 0.0, 0.0)),
        material=glass,
        name="window_glass",
    )
    door.visual(
        Box((0.040, 0.014, 0.210)),
        origin=Origin(xyz=(-0.560, -0.034, 0.0)),
        material=dark,
        name="handle_recess",
    )
    door.visual(
        Box((0.018, 0.018, 0.155)),
        origin=Origin(xyz=(-0.570, -0.047, 0.0)),
        material=edge_white,
        name="handle_bar",
    )
    door.visual(
        Cylinder(radius=0.018, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, -0.19)),
        material=steel,
        name="door_barrel_0",
    )
    door.visual(
        Box((0.085, 0.012, 0.072)),
        origin=Origin(xyz=(-0.0555, 0.006, -0.19)),
        material=steel,
        name="door_leaf_0",
    )
    door.visual(
        Cylinder(radius=0.018, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 0.19)),
        material=steel,
        name="door_barrel_1",
    )
    door.visual(
        Box((0.085, 0.012, 0.072)),
        origin=Origin(xyz=(-0.0555, 0.006, 0.19)),
        material=steel,
        name="door_leaf_1",
    )

    top_cover = model.part("top_cover")
    top_cover.visual(
        Box((0.540, 0.400, 0.026)),
        origin=Origin(xyz=(0.0, -0.200, 0.018)),
        material=white,
        name="cover_panel",
    )
    top_cover.visual(
        Box((0.450, 0.018, 0.008)),
        origin=Origin(xyz=(0.0, -0.370, 0.034)),
        material=edge_white,
        name="front_lip",
    )
    top_cover.visual(
        Cylinder(radius=0.015, length=0.120),
        origin=Origin(xyz=(-0.18, 0.0, 0.0), rpy=(0.0, pi / 2, 0.0)),
        material=steel,
        name="cover_barrel_0",
    )
    top_cover.visual(
        Box((0.115, 0.060, 0.012)),
        origin=Origin(xyz=(-0.18, -0.045, 0.012)),
        material=steel,
        name="cover_leaf_0",
    )
    top_cover.visual(
        Cylinder(radius=0.015, length=0.120),
        origin=Origin(xyz=(0.18, 0.0, 0.0), rpy=(0.0, pi / 2, 0.0)),
        material=steel,
        name="cover_barrel_1",
    )
    top_cover.visual(
        Box((0.115, 0.060, 0.012)),
        origin=Origin(xyz=(0.18, -0.045, 0.012)),
        material=steel,
        name="cover_leaf_1",
    )

    cycle_dial = model.part("cycle_dial")
    cycle_dial.visual(
        Cylinder(radius=0.045, length=0.035),
        origin=Origin(xyz=(0.0, -0.0175, 0.0), rpy=(pi / 2, 0.0, 0.0)),
        material=edge_white,
        name="dial_cap",
    )
    cycle_dial.visual(
        Box((0.007, 0.004, 0.032)),
        origin=Origin(xyz=(0.0, -0.037, 0.026)),
        material=dark,
        name="dial_pointer",
    )

    start_button = model.part("start_button")
    start_button.visual(
        Cylinder(radius=0.024, length=0.014),
        origin=Origin(xyz=(0.0, -0.007, 0.0), rpy=(pi / 2, 0.0, 0.0)),
        material=blue,
        name="button_cap",
    )

    model.articulation(
        "cabinet_to_drum",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=drum,
        origin=Origin(xyz=(0.0, -0.025, DRUM_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=18.0),
    )
    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(DOOR_HINGE_X, DOOR_HINGE_Y, DRUM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.8, lower=0.0, upper=1.85),
    )
    model.articulation(
        "cabinet_to_top_cover",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=top_cover,
        origin=Origin(xyz=(0.0, TOP_HINGE_Y, BODY_H + 0.010)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=0.0, upper=1.25),
    )
    model.articulation(
        "cabinet_to_cycle_dial",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=cycle_dial,
        origin=Origin(xyz=(0.155, FRONT_Y - 0.012, 0.956)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=3.0, lower=-2.7, upper=2.7),
    )
    model.articulation(
        "cabinet_to_start_button",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=start_button,
        origin=Origin(xyz=(-0.218, FRONT_Y - 0.012, 0.956)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=0.08, lower=0.0, upper=0.008),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    top_cover = object_model.get_part("top_cover")
    cycle_dial = object_model.get_part("cycle_dial")
    start_button = object_model.get_part("start_button")

    drum_joint = object_model.get_articulation("cabinet_to_drum")
    door_joint = object_model.get_articulation("cabinet_to_door")
    cover_joint = object_model.get_articulation("cabinet_to_top_cover")
    dial_joint = object_model.get_articulation("cabinet_to_cycle_dial")
    button_joint = object_model.get_articulation("cabinet_to_start_button")

    ctx.allow_overlap(
        cabinet,
        door,
        elem_a="door_hinge_pin_0",
        elem_b="door_barrel_0",
        reason="The lower visible hinge pin is intentionally captured inside its barrel proxy.",
    )
    ctx.allow_overlap(
        cabinet,
        door,
        elem_a="door_hinge_leaf_0",
        elem_b="door_barrel_0",
        reason="The lower hinge barrel locally nests into the mounted hinge leaf knuckle.",
    )
    ctx.allow_overlap(
        cabinet,
        door,
        elem_a="door_hinge_pin_1",
        elem_b="door_barrel_1",
        reason="The upper visible hinge pin is intentionally captured inside its barrel proxy.",
    )
    ctx.allow_overlap(
        cabinet,
        door,
        elem_a="door_hinge_leaf_1",
        elem_b="door_barrel_1",
        reason="The upper hinge barrel locally nests into the mounted hinge leaf knuckle.",
    )
    ctx.allow_overlap(
        cabinet,
        top_cover,
        elem_a="cover_hinge_pin_0",
        elem_b="cover_barrel_0",
        reason="The rear service-cover hinge pin is intentionally captured inside its barrel proxy.",
    )
    ctx.allow_overlap(
        cabinet,
        top_cover,
        elem_a="cover_hinge_leaf_0",
        elem_b="cover_barrel_0",
        reason="The service-cover barrel locally nests into the fixed hinge leaf.",
    )
    ctx.allow_overlap(
        cabinet,
        top_cover,
        elem_a="cover_hinge_pin_1",
        elem_b="cover_barrel_1",
        reason="The rear service-cover hinge pin is intentionally captured inside its barrel proxy.",
    )
    ctx.allow_overlap(
        cabinet,
        top_cover,
        elem_a="cover_hinge_leaf_1",
        elem_b="cover_barrel_1",
        reason="The service-cover barrel locally nests into the fixed hinge leaf.",
    )
    ctx.allow_overlap(
        cabinet,
        drum,
        elem_a="rear_bearing",
        elem_b="rear_hub",
        reason="The drum hub is intentionally seated inside the rear bearing proxy on the continuous axle.",
    )

    ctx.check(
        "drum axle is continuous",
        drum_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint_type={drum_joint.articulation_type}",
    )
    ctx.check(
        "door has two hinge barrels",
        len([v for v in door.visuals if v.name and v.name.startswith("door_barrel_")]) == 2,
    )
    ctx.check(
        "cover has two hinge barrels",
        len([v for v in top_cover.visuals if v.name and v.name.startswith("cover_barrel_")]) == 2,
    )

    ctx.expect_contact(
        cabinet,
        door,
        elem_a="door_hinge_pin_0",
        elem_b="door_barrel_0",
        contact_tol=0.012,
        name="lower door hinge is captured",
    )
    ctx.expect_contact(
        cabinet,
        door,
        elem_a="door_hinge_pin_1",
        elem_b="door_barrel_1",
        contact_tol=0.012,
        name="upper door hinge is captured",
    )
    ctx.expect_contact(
        cabinet,
        door,
        elem_a="door_hinge_leaf_0",
        elem_b="door_barrel_0",
        contact_tol=0.004,
        name="lower door hinge leaf supports barrel",
    )
    ctx.expect_contact(
        cabinet,
        door,
        elem_a="door_hinge_leaf_1",
        elem_b="door_barrel_1",
        contact_tol=0.004,
        name="upper door hinge leaf supports barrel",
    )
    ctx.expect_contact(
        cabinet,
        top_cover,
        elem_a="cover_hinge_pin_0",
        elem_b="cover_barrel_0",
        contact_tol=0.010,
        name="service cover hinge is captured",
    )
    ctx.expect_contact(
        cabinet,
        top_cover,
        elem_a="cover_hinge_pin_1",
        elem_b="cover_barrel_1",
        contact_tol=0.010,
        name="second service cover hinge is captured",
    )
    ctx.expect_contact(
        cabinet,
        top_cover,
        elem_a="cover_hinge_leaf_0",
        elem_b="cover_barrel_0",
        contact_tol=0.004,
        name="service cover leaf supports barrel",
    )
    ctx.expect_contact(
        cabinet,
        top_cover,
        elem_a="cover_hinge_leaf_1",
        elem_b="cover_barrel_1",
        contact_tol=0.004,
        name="second service cover leaf supports barrel",
    )
    ctx.expect_contact(
        cabinet,
        drum,
        elem_a="rear_bearing",
        elem_b="rear_hub",
        contact_tol=0.004,
        name="drum hub is seated in rear bearing",
    )
    ctx.expect_overlap(
        drum,
        door,
        axes="xz",
        min_overlap=0.30,
        elem_a="drum_shell",
        elem_b="window_glass",
        name="drum sits behind porthole window",
    )

    closed_door_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_joint: 1.20}):
        open_door_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door swings outward",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[0][1] < closed_door_aabb[0][1] - 0.12,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    closed_cover_aabb = ctx.part_world_aabb(top_cover)
    with ctx.pose({cover_joint: 0.85}):
        open_cover_aabb = ctx.part_world_aabb(top_cover)
    ctx.check(
        "top cover lifts upward",
        closed_cover_aabb is not None
        and open_cover_aabb is not None
        and open_cover_aabb[1][2] > closed_cover_aabb[1][2] + 0.13,
        details=f"closed={closed_cover_aabb}, open={open_cover_aabb}",
    )

    dial_limits = dial_joint.motion_limits
    button_limits = button_joint.motion_limits
    ctx.check(
        "smart controls are articulated",
        dial_limits is not None
        and dial_limits.lower is not None
        and dial_limits.upper is not None
        and dial_limits.upper - dial_limits.lower > 5.0
        and button_limits is not None
        and button_limits.upper is not None
        and button_limits.upper >= 0.006,
    )

    return ctx.report()


object_model = build_object_model()
