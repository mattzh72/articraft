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
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


DOOR_W = 0.46
DOOR_H = 0.60
DOOR_T = 0.080
HINGE_X = -0.245
HINGE_Y = 0.015
DOOR_FRONT_Y = 0.015
DIAL_X = DOOR_W / 2.0 + 0.015
DIAL_Z = 0.105
WHEEL_Z = -0.125


def _cut_y_hole(shape, x: float, z: float, radius: float, depth: float):
    """Cut a clearance cylinder running through a slab along local Y."""
    cutter = cq.Workplane("XZ").center(x, z).circle(radius).extrude(depth, both=True)
    return shape.cut(cutter)


def _wall_plate():
    wall = cq.Workplane("XY").box(0.90, 0.045, 0.88)
    opening = cq.Workplane("XY").box(0.59, 0.070, 0.73)
    return wall.cut(opening).translate((0.0, -0.020, 0.0))


def _outer_frame():
    frame = cq.Workplane("XY").box(0.64, 0.030, 0.78)
    opening = cq.Workplane("XY").box(0.58, 0.060, 0.72)
    frame = frame.cut(opening)
    try:
        frame = frame.edges("|Y").fillet(0.006)
    except Exception:
        pass
    return frame.translate((0.0, 0.010, 0.0))


def _recess_box():
    left = cq.Workplane("XY").box(0.025, 0.200, 0.730).translate((-0.2825, -0.110, 0.0))
    right = cq.Workplane("XY").box(0.025, 0.200, 0.730).translate((0.2825, -0.110, 0.0))
    top = cq.Workplane("XY").box(0.590, 0.200, 0.025).translate((0.0, -0.110, 0.3525))
    bottom = cq.Workplane("XY").box(0.590, 0.200, 0.025).translate((0.0, -0.110, -0.3525))
    back = cq.Workplane("XY").box(0.590, 0.025, 0.730).translate((0.0, -0.205, 0.0))
    box = left.union(right).union(top).union(bottom).union(back)
    try:
        box = box.edges("|Y").fillet(0.003)
    except Exception:
        pass
    return box


def _fixed_hinge():
    hinge = cq.Workplane("XY").box(0.024, 0.008, 0.590).translate((HINGE_X - 0.041, HINGE_Y - 0.003, 0.0))
    for zc, length in ((0.2225, 0.145), (0.0, 0.120), (-0.2225, 0.145)):
        tab = cq.Workplane("XY").box(0.042, 0.008, length * 0.82).translate((HINGE_X - 0.023, HINGE_Y - 0.003, zc))
        barrel = (
            cq.Workplane("XY")
            .circle(0.014)
            .extrude(length)
            .translate((HINGE_X, HINGE_Y, zc - length / 2.0))
        )
        hinge = hinge.union(tab).union(barrel)
    return hinge


def _door_slab():
    door = cq.Workplane("XY").box(DOOR_W, DOOR_T, DOOR_H).translate((DOOR_W / 2.0 + 0.015, -0.025, 0.0))
    try:
        door = door.edges("|Y").fillet(0.010)
    except Exception:
        pass
    door = _cut_y_hole(door, DIAL_X, DIAL_Z, 0.017, 0.240)
    door = _cut_y_hole(door, DIAL_X, WHEEL_Z, 0.018, 0.240)
    return door


def _moving_hinge():
    leaf = cq.Workplane("XY").box(0.024, 0.007, 0.455).translate((0.026, 0.003, 0.0))
    top_pad = cq.Workplane("XY").box(0.026, 0.009, 0.070).translate((0.023, 0.003, 0.155))
    bottom_pad = cq.Workplane("XY").box(0.026, 0.009, 0.070).translate((0.023, 0.003, -0.155))
    return leaf.union(top_pad).union(bottom_pad)


def _wheel_handle():
    ring = cq.Workplane("XZ").circle(0.063).circle(0.049).extrude(0.018, both=True).translate((0.0, 0.032, 0.0))
    spoke_x = cq.Workplane("XZ").rect(0.102, 0.010).extrude(0.016, both=True).translate((0.0, 0.032, 0.0))
    spoke_z = cq.Workplane("XZ").rect(0.010, 0.102).extrude(0.016, both=True).translate((0.0, 0.032, 0.0))
    hub = cq.Workplane("XZ").circle(0.022).extrude(0.032, both=True).translate((0.0, 0.024, 0.0))
    shaft = cq.Workplane("XZ").circle(0.011).extrude(0.120).translate((0.0, -0.095, 0.0))
    wheel = ring.union(spoke_x).union(spoke_z).union(hub).union(shaft)
    try:
        wheel = wheel.edges().fillet(0.0015)
    except Exception:
        pass
    return wheel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="recessed_wall_safe")

    wall_mat = Material("painted_wall", color=(0.72, 0.72, 0.69, 1.0))
    frame_mat = Material("blackened_steel", color=(0.035, 0.038, 0.040, 1.0))
    door_mat = Material("charcoal_door", color=(0.075, 0.080, 0.085, 1.0))
    trim_mat = Material("brushed_steel", color=(0.52, 0.55, 0.56, 1.0))
    brass_mat = Material("aged_brass", color=(0.78, 0.60, 0.30, 1.0))
    white_mat = Material("painted_mark", color=(0.92, 0.90, 0.82, 1.0))

    case = model.part("wall_case")
    case.visual(mesh_from_cadquery(_wall_plate(), "wall_plate"), material=wall_mat, name="wall_plate")
    case.visual(mesh_from_cadquery(_recess_box(), "recess_box"), material=frame_mat, name="recess_box")
    case.visual(mesh_from_cadquery(_outer_frame(), "outer_frame"), material=frame_mat, name="outer_frame")
    case.visual(mesh_from_cadquery(_fixed_hinge(), "fixed_hinge"), material=trim_mat, name="fixed_hinge")

    door = model.part("door")
    door.visual(mesh_from_cadquery(_door_slab(), "door_slab"), material=door_mat, name="door_slab")
    door.visual(mesh_from_cadquery(_moving_hinge(), "moving_hinge"), material=trim_mat, name="moving_hinge")
    door.visual(
        Box((0.340, 0.004, 0.455)),
        origin=Origin(xyz=(DIAL_X, DOOR_FRONT_Y + 0.002, -0.020)),
        material=Material("recessed_panel", color=(0.045, 0.048, 0.052, 1.0)),
        name="front_recess",
    )
    for i in range(12):
        angle = i * math.tau / 12.0
        tick_len = 0.020 if i % 3 == 0 else 0.013
        tick_w = 0.006 if i % 3 == 0 else 0.003
        radius = 0.083
        door.visual(
            Box((tick_w, 0.003, tick_len)),
            origin=Origin(
                xyz=(DIAL_X + radius * math.sin(angle), DOOR_FRONT_Y + 0.004, DIAL_Z + radius * math.cos(angle)),
                rpy=(0.0, angle, 0.0),
            ),
            material=white_mat,
            name=f"tick_{i}",
        )

    dial = model.part("dial")
    dial.visual(
        Cylinder(0.010, 0.130),
        origin=Origin(xyz=(0.0, -0.045, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=trim_mat,
        name="dial_shaft",
    )
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.112,
                0.026,
                body_style="faceted",
                base_diameter=0.112,
                top_diameter=0.094,
                edge_radius=0.0015,
                grip=KnobGrip(style="ribbed", count=36, depth=0.0012, width=0.0020),
                indicator=KnobIndicator(style="line", mode="engraved", angle_deg=0.0),
                center=False,
            ),
            "combination_dial",
        ),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=frame_mat,
        name="dial_cap",
    )
    dial.visual(
        Box((0.010, 0.004, 0.037)),
        origin=Origin(xyz=(0.0, 0.0275, 0.026)),
        material=brass_mat,
        name="dial_pointer",
    )

    wheel = model.part("wheel_handle")
    wheel.visual(
        Cylinder(0.011, 0.135),
        origin=Origin(xyz=(0.0, -0.040, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=trim_mat,
        name="wheel_shaft",
    )
    wheel.visual(
        Cylinder(0.022, 0.036),
        origin=Origin(xyz=(0.0, 0.035, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=trim_mat,
        name="wheel_hub",
    )
    wheel.visual(
        mesh_from_geometry(TorusGeometry(0.057, 0.006, radial_segments=16, tubular_segments=48), "wheel_ring"),
        origin=Origin(xyz=(0.0, 0.045, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=trim_mat,
        name="wheel_ring",
    )
    for i, angle in enumerate((0.0, math.pi / 2.0, math.pi / 4.0, -math.pi / 4.0)):
        wheel.visual(
            Box((0.108, 0.012, 0.010)),
            origin=Origin(xyz=(0.0, 0.045, 0.0), rpy=(0.0, angle, 0.0)),
            material=trim_mat,
            name=f"wheel_spoke_{i}",
        )
    wheel.visual(
        Cylinder(0.009, 0.040),
        origin=Origin(xyz=(0.038, 0.070, -0.038), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass_mat,
        name="grip_peg",
    )

    model.articulation(
        "case_to_door",
        ArticulationType.REVOLUTE,
        parent=case,
        child=door,
        origin=Origin(xyz=(HINGE_X, HINGE_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.8, lower=0.0, upper=1.55),
    )
    model.articulation(
        "door_to_dial",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=dial,
        origin=Origin(xyz=(DIAL_X, DOOR_FRONT_Y + 0.001, DIAL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=6.0),
    )
    model.articulation(
        "door_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=wheel,
        origin=Origin(xyz=(DIAL_X, DOOR_FRONT_Y + 0.001, WHEEL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    case = object_model.get_part("wall_case")
    door = object_model.get_part("door")
    dial = object_model.get_part("dial")
    wheel = object_model.get_part("wheel_handle")
    door_hinge = object_model.get_articulation("case_to_door")
    dial_joint = object_model.get_articulation("door_to_dial")
    wheel_joint = object_model.get_articulation("door_to_wheel")

    ctx.check(
        "door uses a vertical side hinge",
        door_hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 3) for v in door_hinge.axis) == (0.0, 0.0, 1.0)
        and door_hinge.motion_limits is not None
        and door_hinge.motion_limits.upper is not None
        and door_hinge.motion_limits.upper > 1.2,
        details=f"axis={door_hinge.axis}, limits={door_hinge.motion_limits}",
    )
    ctx.check(
        "dial is a continuous rotary control",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 3) for v in dial_joint.axis) == (0.0, 1.0, 0.0),
        details=f"type={dial_joint.articulation_type}, axis={dial_joint.axis}",
    )
    ctx.check(
        "wheel handle is a continuous rotary control",
        wheel_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 3) for v in wheel_joint.axis) == (0.0, 1.0, 0.0),
        details=f"type={wheel_joint.articulation_type}, axis={wheel_joint.axis}",
    )

    ctx.expect_within(
        door,
        case,
        axes="xz",
        inner_elem="door_slab",
        outer_elem="outer_frame",
        margin=0.020,
        name="closed door is captured inside the slim frame opening",
    )
    ctx.expect_overlap(
        dial,
        door,
        axes="y",
        elem_a="dial_shaft",
        elem_b="door_slab",
        min_overlap=0.050,
        name="dial shaft passes through the thick door",
    )
    ctx.expect_overlap(
        wheel,
        door,
        axes="y",
        elem_a="wheel_shaft",
        elem_b="door_slab",
        min_overlap=0.050,
        name="wheel shaft passes through the thick door",
    )

    closed_door_aabb = ctx.part_element_world_aabb(door, elem="door_slab")
    with ctx.pose({door_hinge: 1.05}):
        opened_door_aabb = ctx.part_element_world_aabb(door, elem="door_slab")
    ctx.check(
        "door swings outward from the wall",
        closed_door_aabb is not None
        and opened_door_aabb is not None
        and opened_door_aabb[1][1] > closed_door_aabb[1][1] + 0.12,
        details=f"closed={closed_door_aabb}, opened={opened_door_aabb}",
    )

    pointer_aabb_0 = ctx.part_element_world_aabb(dial, elem="dial_pointer")
    with ctx.pose({dial_joint: math.pi / 2.0}):
        pointer_aabb_90 = ctx.part_element_world_aabb(dial, elem="dial_pointer")
    ctx.check(
        "dial pointer moves around the central shaft",
        pointer_aabb_0 is not None
        and pointer_aabb_90 is not None
        and abs(((pointer_aabb_90[0][0] + pointer_aabb_90[1][0]) / 2.0) - ((pointer_aabb_0[0][0] + pointer_aabb_0[1][0]) / 2.0)) > 0.018,
        details=f"start={pointer_aabb_0}, quarter_turn={pointer_aabb_90}",
    )

    grip_aabb_0 = ctx.part_element_world_aabb(wheel, elem="grip_peg")
    with ctx.pose({wheel_joint: math.pi / 2.0}):
        grip_aabb_90 = ctx.part_element_world_aabb(wheel, elem="grip_peg")
    ctx.check(
        "wheel grip rotates on the coaxial shaft",
        grip_aabb_0 is not None
        and grip_aabb_90 is not None
        and abs(((grip_aabb_90[0][0] + grip_aabb_90[1][0]) / 2.0) - ((grip_aabb_0[0][0] + grip_aabb_0[1][0]) / 2.0)) > 0.030,
        details=f"start={grip_aabb_0}, quarter_turn={grip_aabb_90}",
    )

    return ctx.report()


object_model = build_object_model()
