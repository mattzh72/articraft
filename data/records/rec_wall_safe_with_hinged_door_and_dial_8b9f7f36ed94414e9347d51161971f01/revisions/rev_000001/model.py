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
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _cq_box(size: tuple[float, float, float], center: tuple[float, float, float]):
    return cq.Workplane("XY").box(*size).translate(center)


def _safe_case_mesh():
    """One continuous front frame plus recessed wall box, in meters."""
    outer = 0.50
    opening = 0.40
    frame_depth = 0.035
    frame_center_x = 0.0055  # spans x=-0.012 .. +0.023

    body_depth = 0.160
    body_outer = 0.410
    cavity = 0.395
    body_center_x = 0.080

    frame = _cq_box((frame_depth, outer, outer), (frame_center_x, 0.0, 0.0)).cut(
        _cq_box((frame_depth * 2.2, opening, opening), (frame_center_x, 0.0, 0.0))
    )
    body_tube = _cq_box((body_depth, body_outer, body_outer), (body_center_x, 0.0, 0.0)).cut(
        _cq_box((body_depth * 1.2, cavity, cavity), (body_center_x, 0.0, 0.0))
    )
    rear_wall = _cq_box((0.014, body_outer, body_outer), (body_depth - 0.007, 0.0, 0.0))
    return frame.union(body_tube).union(rear_wall)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_wall_safe")

    gunmetal = model.material("powder_coated_gunmetal", rgba=(0.12, 0.13, 0.14, 1.0))
    dark_panel = model.material("dark_recessed_steel", rgba=(0.055, 0.060, 0.065, 1.0))
    hinge_steel = model.material("brushed_hinge_steel", rgba=(0.46, 0.48, 0.47, 1.0))
    dial_black = model.material("black_knurled_dial", rgba=(0.015, 0.015, 0.017, 1.0))
    pale_mark = model.material("pale_index_marks", rgba=(0.86, 0.84, 0.76, 1.0))
    lever_metal = model.material("satin_latch_metal", rgba=(0.62, 0.61, 0.56, 1.0))

    case = model.part("case")
    case.visual(
        mesh_from_cadquery(_safe_case_mesh(), "square_frame_and_recess"),
        material=gunmetal,
        name="square_frame",
    )

    # Four low screw heads make the square wall frame read as a fastened metal surround.
    for idx, (y, z) in enumerate(((-0.215, -0.215), (-0.215, 0.215), (0.215, -0.215), (0.215, 0.215))):
        case.visual(
            Cylinder(radius=0.012, length=0.004),
            origin=Origin(xyz=(-0.014, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hinge_steel,
            name=f"frame_screw_{idx}",
        )

    # Fixed hinge knuckles on the frame side, alternating with the door knuckles.
    for idx, (z, length) in enumerate(((-0.130, 0.080), (0.000, 0.060), (0.130, 0.080))):
        case.visual(
            Cylinder(radius=0.011, length=length),
            origin=Origin(xyz=(0.0, 0.180, z)),
            material=hinge_steel,
            name=f"fixed_knuckle_{idx}",
        )
        case.visual(
            Box((0.020, 0.032, length)),
            origin=Origin(xyz=(0.0, 0.205, z)),
            material=hinge_steel,
            name=f"hinge_bracket_{idx}",
        )

    door = model.part("door")
    door.visual(
        Box((0.026, 0.360, 0.360)),
        origin=Origin(xyz=(-0.023, -0.180, 0.0)),
        material=gunmetal,
        name="door_slab",
    )
    door.visual(
        Box((0.004, 0.270, 0.270)),
        origin=Origin(xyz=(-0.038, -0.180, 0.0)),
        material=dark_panel,
        name="raised_face_panel",
    )
    door.visual(
        Box((0.004, 0.030, 0.006)),
        origin=Origin(xyz=(-0.041, -0.180, 0.107)),
        material=pale_mark,
        name="dial_index_mark",
    )
    for idx, z in enumerate((-0.060, 0.060)):
        door.visual(
            Cylinder(radius=0.0105, length=0.050),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=hinge_steel,
            name=f"door_knuckle_{idx}",
        )

    door_hinge = model.articulation(
        "case_to_door",
        ArticulationType.REVOLUTE,
        parent=case,
        child=door,
        origin=Origin(xyz=(0.0, 0.180, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=0.0, upper=1.75),
    )
    door_hinge.meta["description"] = "Right-side vertical hinge; positive travel swings the door outward."

    dial = model.part("combination_dial")
    dial_mesh = mesh_from_geometry(
        KnobGeometry(
            0.074,
            0.022,
            body_style="cylindrical",
            edge_radius=0.001,
            grip=KnobGrip(style="knurled", count=48, depth=0.0009, helix_angle_deg=22.0),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
        ),
        "knurled_combination_dial",
    )
    dial.visual(
        dial_mesh,
        origin=Origin(xyz=(-0.011, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dial_black,
        name="dial_cap",
    )
    dial.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(-0.024, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_steel,
        name="dial_center_hub",
    )

    model.articulation(
        "door_to_dial",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=dial,
        origin=Origin(xyz=(-0.040, -0.180, 0.060)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
    )

    latch_lever = model.part("latch_lever")
    latch_lever.visual(
        Cylinder(radius=0.014, length=0.012),
        origin=Origin(xyz=(-0.006, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lever_metal,
        name="pivot_boss",
    )
    latch_lever.visual(
        Box((0.008, 0.090, 0.016)),
        origin=Origin(xyz=(-0.013, 0.045, 0.0)),
        material=lever_metal,
        name="lever_bar",
    )
    latch_lever.visual(
        Sphere(radius=0.011),
        origin=Origin(xyz=(-0.013, 0.092, 0.0)),
        material=lever_metal,
        name="rounded_tip",
    )

    model.articulation(
        "door_to_latch_lever",
        ArticulationType.REVOLUTE,
        parent=door,
        child=latch_lever,
        origin=Origin(xyz=(-0.040, -0.240, -0.130)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=3.0, lower=-0.55, upper=0.70),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    case = object_model.get_part("case")
    door = object_model.get_part("door")
    dial = object_model.get_part("combination_dial")
    latch = object_model.get_part("latch_lever")
    door_hinge = object_model.get_articulation("case_to_door")
    dial_joint = object_model.get_articulation("door_to_dial")
    latch_joint = object_model.get_articulation("door_to_latch_lever")

    ctx.expect_within(
        door,
        case,
        axes="yz",
        margin=0.0,
        name="closed door fits inside the square frame outline",
    )
    ctx.expect_within(
        dial,
        door,
        axes="yz",
        margin=0.0,
        name="combination dial is mounted on the door face",
    )
    ctx.expect_within(
        latch,
        door,
        axes="yz",
        margin=0.0,
        name="latch lever remains on the lower door face",
    )

    rest_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.20}):
        open_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door hinge swings outward",
        rest_aabb is not None and open_aabb is not None and open_aabb[0][0] < rest_aabb[0][0] - 0.08,
        details=f"rest_aabb={rest_aabb}, open_aabb={open_aabb}",
    )

    ctx.check(
        "dial is continuous about the door-normal axis",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(dial_joint.axis) == (1.0, 0.0, 0.0),
        details=f"type={dial_joint.articulation_type}, axis={dial_joint.axis}",
    )
    ctx.check(
        "latch lever has a short limited throw",
        latch_joint.motion_limits is not None
        and latch_joint.motion_limits.lower < 0.0
        and latch_joint.motion_limits.upper > 0.0
        and latch_joint.motion_limits.upper - latch_joint.motion_limits.lower < 1.5,
        details=f"limits={latch_joint.motion_limits}",
    )

    return ctx.report()


object_model = build_object_model()
