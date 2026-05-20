from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _arched_prism(
    *,
    width: float,
    height: float,
    side_height: float,
    length: float,
    x_start: float,
    z_start: float,
):
    """D-profile prism extruded along world X, using Y/Z as the profile plane."""
    return (
        cq.Workplane("YZ")
        .moveTo(-width / 2.0, z_start)
        .lineTo(width / 2.0, z_start)
        .lineTo(width / 2.0, z_start + side_height)
        .threePointArc((0.0, z_start + height), (-width / 2.0, z_start + side_height))
        .close()
        .extrude(length)
        .translate((x_start, 0.0, 0.0))
    )


def _mailbox_shell(
    *,
    center_x: float,
    length: float,
    width: float,
    height: float,
    side_height: float,
    bottom_z: float,
    wall: float,
):
    outer = _arched_prism(
        width=width,
        height=height,
        side_height=side_height,
        length=length,
        x_start=center_x - length / 2.0,
        z_start=bottom_z,
    )
    inner = _arched_prism(
        width=width - 2.0 * wall,
        height=height - 2.0 * wall,
        side_height=side_height - wall,
        length=length + 0.05,
        x_start=center_x - length / 2.0 + wall,
        z_start=bottom_z + wall,
    )
    return outer.cut(inner).edges("|X").fillet(0.003)


def _arched_ring(
    *,
    center_x: float,
    length: float,
    outer_width: float,
    outer_height: float,
    outer_side_height: float,
    inner_width: float,
    inner_height: float,
    inner_side_height: float,
    bottom_z: float,
):
    outer = _arched_prism(
        width=outer_width,
        height=outer_height,
        side_height=outer_side_height,
        length=length,
        x_start=center_x,
        z_start=bottom_z,
    )
    inner = _arched_prism(
        width=inner_width,
        height=inner_height,
        side_height=inner_side_height,
        length=length + 0.012,
        x_start=center_x - 0.006,
        z_start=bottom_z + (outer_height - inner_height) * 0.45,
    )
    return outer.cut(inner).edges("|X").fillet(0.002)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="weatherproof_post_mailbox")

    galvanized = model.material("galvanized_steel", rgba=(0.63, 0.70, 0.72, 1.0))
    darker_steel = model.material("dark_galvanized_edges", rgba=(0.42, 0.49, 0.51, 1.0))
    black = model.material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    red = model.material("red_signal_flag", rgba=(0.82, 0.04, 0.02, 1.0))
    cedar = model.material("weathered_cedar", rgba=(0.55, 0.36, 0.20, 1.0))
    concrete = model.material("rough_concrete", rgba=(0.45, 0.45, 0.42, 1.0))

    body_center_x = 0.06
    body_length = 0.58
    body_width = 0.30
    body_height = 0.32
    body_side_height = 0.17
    body_bottom_z = 1.05
    wall = 0.018
    body_front_x = body_center_x + body_length / 2.0

    body = model.part("body")

    body.visual(
        Box((0.23, 0.23, 0.075)),
        origin=Origin(xyz=(-0.10, 0.0, 0.0375)),
        material=concrete,
        name="concrete_footing",
    )
    body.visual(
        Box((0.105, 0.105, 1.03)),
        origin=Origin(xyz=(-0.10, 0.0, 0.555)),
        material=cedar,
        name="cedar_post",
    )
    body.visual(
        Box((0.58, 0.135, 0.085)),
        origin=Origin(xyz=(0.00, 0.0, 1.0125)),
        material=cedar,
        name="support_arm",
    )
    for y, name in ((-0.052, "brace_0"), (0.052, "brace_1")):
        body.visual(
            Box((0.36, 0.030, 0.045)),
            origin=Origin(xyz=(-0.07, y, 0.918), rpy=(0.0, -0.55, 0.0)),
            material=cedar,
            name=name,
        )
    body.visual(
        Box((0.46, 0.035, 0.030)),
        origin=Origin(xyz=(0.06, -0.112, 1.065)),
        material=darker_steel,
        name="mounting_rail_0",
    )
    body.visual(
        Box((0.46, 0.035, 0.030)),
        origin=Origin(xyz=(0.06, 0.112, 1.065)),
        material=darker_steel,
        name="mounting_rail_1",
    )

    shell = _mailbox_shell(
        center_x=body_center_x,
        length=body_length,
        width=body_width,
        height=body_height,
        side_height=body_side_height,
        bottom_z=body_bottom_z,
        wall=wall,
    )
    body.visual(
        mesh_from_cadquery(shell, "arched_weatherproof_shell", tolerance=0.0008),
        material=galvanized,
        name="shell",
    )

    front_lip = _arched_ring(
        center_x=body_front_x - 0.004,
        length=0.026,
        outer_width=body_width + 0.020,
        outer_height=body_height + 0.015,
        outer_side_height=body_side_height + 0.004,
        inner_width=body_width - 0.052,
        inner_height=body_height - 0.065,
        inner_side_height=body_side_height - 0.033,
        bottom_z=body_bottom_z - 0.004,
    )
    body.visual(
        mesh_from_cadquery(front_lip, "front_weather_lip", tolerance=0.0008),
        material=darker_steel,
        name="front_lip",
    )
    body.visual(
        Box((0.030, body_width + 0.050, 0.012)),
        origin=Origin(xyz=(body_front_x + 0.008, 0.0, body_bottom_z - 0.006)),
        material=black,
        name="bottom_drip_gasket",
    )
    body.visual(
        Box((0.014, 0.052, 0.018)),
        origin=Origin(xyz=(body_front_x + 0.027, 0.0, body_bottom_z + 0.040)),
        material=black,
        name="door_stop_bumper",
    )

    # Exposed hinge barrels are mounted under the mouth, with a center barrel on
    # the door between these two fixed body barrels.
    for y, name in ((-0.182, "hinge_barrel_0"), (0.182, "hinge_barrel_1")):
        body.visual(
            Cylinder(radius=0.012, length=0.068),
            origin=Origin(
                xyz=(body_front_x + 0.034, y, body_bottom_z + 0.010),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=darker_steel,
            name=name,
        )
        body.visual(
            Box((0.036, 0.050, 0.010)),
            origin=Origin(xyz=(body_front_x + 0.019, y, body_bottom_z + 0.022)),
            material=darker_steel,
            name=f"hinge_tab_{name[-1]}",
        )

    # Side hardware: raised screw heads, a flag pivot washer, and a row of side
    # rivets all read as outdoor sheet-metal construction.
    for x in (-0.145, -0.020, 0.105, 0.230):
        for z in (body_bottom_z + 0.060, body_bottom_z + 0.142):
            body.visual(
                Cylinder(radius=0.0065, length=0.006),
                origin=Origin(
                    xyz=(x, body_width / 2.0 + 0.003, z),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=darker_steel,
                name=f"side_rivet_{x:.2f}_{z:.2f}",
            )
    flag_pivot_x = body_center_x + 0.075
    flag_pivot_z = body_bottom_z + 0.185
    body.visual(
        Cylinder(radius=0.032, length=0.008),
        origin=Origin(
            xyz=(flag_pivot_x, body_width / 2.0 + 0.004, flag_pivot_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=black,
        name="flag_pivot_washer",
    )

    door = model.part("door")
    door_width = 0.282
    door_height = 0.305
    door_side_height = 0.154
    door_thickness = 0.014
    door_panel = _arched_prism(
        width=door_width,
        height=door_height,
        side_height=door_side_height,
        length=door_thickness,
        x_start=0.0,
        z_start=0.0,
    ).edges("|X").fillet(0.0025)
    door.visual(
        mesh_from_cadquery(door_panel, "arched_front_door", tolerance=0.0008),
        material=galvanized,
        name="door_panel",
    )

    raised_border = _arched_ring(
        center_x=door_thickness - 0.001,
        length=0.006,
        outer_width=door_width - 0.038,
        outer_height=door_height - 0.038,
        outer_side_height=door_side_height - 0.020,
        inner_width=door_width - 0.088,
        inner_height=door_height - 0.092,
        inner_side_height=door_side_height - 0.050,
        bottom_z=0.022,
    )
    door.visual(
        mesh_from_cadquery(raised_border, "raised_door_border", tolerance=0.0008),
        material=darker_steel,
        name="raised_border",
    )
    door.visual(
        Cylinder(radius=0.022, length=0.022),
        origin=Origin(xyz=(0.032, 0.0, 0.198), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="pull_knob",
    )
    door.visual(
        Cylinder(radius=0.010, length=0.024),
        origin=Origin(xyz=(0.020, 0.0, 0.198), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=darker_steel,
        name="knob_stem",
    )
    door.visual(
        Cylinder(radius=0.011, length=0.080),
        origin=Origin(xyz=(0.000, 0.0, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=darker_steel,
        name="door_hinge_barrel",
    )
    for y, name in ((-0.032, "door_hinge_tab_0"), (0.032, "door_hinge_tab_1")):
        door.visual(
            Box((0.022, 0.018, 0.040)),
            origin=Origin(xyz=(0.007, y, 0.020)),
            material=darker_steel,
            name=name,
        )
    for y in (-0.100, 0.100):
        for z in (0.058, 0.225):
            door.visual(
                Cylinder(radius=0.0055, length=0.005),
                origin=Origin(xyz=(0.017, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=darker_steel,
                name=f"door_rivet_{y:.2f}_{z:.2f}",
            )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(body_front_x + 0.034, 0.0, body_bottom_z + 0.010)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=0.0, upper=1.90),
    )

    flag = model.part("flag")
    flag.visual(
        Cylinder(radius=0.026, length=0.010),
        origin=Origin(xyz=(0.0, 0.000, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=red,
        name="pivot_cap",
    )
    flag.visual(
        Box((0.205, 0.012, 0.018)),
        origin=Origin(xyz=(-0.105, 0.008, 0.0)),
        material=red,
        name="flag_arm",
    )
    flag.visual(
        Box((0.070, 0.014, 0.090)),
        origin=Origin(xyz=(-0.205, 0.009, 0.038)),
        material=red,
        name="flag_paddle",
    )
    model.articulation(
        "body_to_flag",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flag,
        origin=Origin(xyz=(flag_pivot_x, body_width / 2.0 + 0.013, flag_pivot_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=3.0, lower=0.0, upper=1.57),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    body = object_model.get_part("body")
    door = object_model.get_part("door")
    flag = object_model.get_part("flag")
    door_joint = object_model.get_articulation("body_to_door")
    flag_joint = object_model.get_articulation("body_to_flag")

    ctx.check(
        "weatherproof shell and support are present",
        all(
            body.get_visual(name) is not None
            for name in ("shell", "front_lip", "cedar_post", "support_arm")
        ),
        details="The mailbox needs a rounded shell, front lip, post, and support arm.",
    )
    ctx.check(
        "door opens downward from a bottom hinge",
        door_joint.axis == (0.0, 1.0, 0.0)
        and door_joint.motion_limits is not None
        and door_joint.motion_limits.lower == 0.0
        and door_joint.motion_limits.upper is not None
        and door_joint.motion_limits.upper > 1.5,
        details=f"door axis={door_joint.axis}, limits={door_joint.motion_limits}",
    )
    ctx.check(
        "side signal flag rotates upward",
        flag_joint.axis == (0.0, 1.0, 0.0)
        and flag_joint.motion_limits is not None
        and flag_joint.motion_limits.upper is not None
        and flag_joint.motion_limits.upper >= 1.5,
        details=f"flag axis={flag_joint.axis}, limits={flag_joint.motion_limits}",
    )

    with ctx.pose({door_joint: 0.0, flag_joint: 0.0}):
        ctx.expect_gap(
            door,
            body,
            axis="x",
            min_gap=0.001,
            max_gap=0.040,
            positive_elem="door_panel",
            negative_elem="front_lip",
            name="closed door sits just proud of the weather lip",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="yz",
            min_overlap=0.20,
            elem_a="door_panel",
            elem_b="front_lip",
            name="closed door covers the arched mouth",
        )
        closed_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
        flag_down_aabb = ctx.part_element_world_aabb(flag, elem="flag_arm")

    with ctx.pose({door_joint: 1.25, flag_joint: 1.57}):
        open_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
        flag_up_aabb = ctx.part_element_world_aabb(flag, elem="flag_arm")

    ctx.check(
        "opened door swings outward and down",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][0] > closed_aabb[1][0] + 0.11
        and open_aabb[1][2] < closed_aabb[1][2] - 0.02,
        details=f"closed={closed_aabb}, opened={open_aabb}",
    )
    ctx.check(
        "raised flag visibly stands above its lowered pose",
        flag_down_aabb is not None
        and flag_up_aabb is not None
        and flag_up_aabb[1][2] > flag_down_aabb[1][2] + 0.12,
        details=f"down={flag_down_aabb}, up={flag_up_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
