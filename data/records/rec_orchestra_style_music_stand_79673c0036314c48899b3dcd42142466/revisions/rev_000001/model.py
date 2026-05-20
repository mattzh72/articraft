from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _tube_mesh(outer_radius: float, inner_radius: float, length: float, name: str):
    """Centered, open annular tube used for the stand sleeves and collars."""
    tube = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
    )
    return mesh_from_cadquery(tube, name, tolerance=0.0007, angular_tolerance=0.08)


def _cylinder_between(start: tuple[float, float, float], end: tuple[float, float, float]):
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    yaw = math.atan2(dy, dx)
    pitch = math.acos(dz / length)
    return length, Origin(
        xyz=((start[0] + end[0]) / 2.0, (start[1] + end[1]) / 2.0, (start[2] + end[2]) / 2.0),
        rpy=(0.0, pitch, yaw),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="school_orchestra_stand")

    black_metal = model.material("satin_black_powdercoat", rgba=(0.015, 0.017, 0.018, 1.0))
    dark_panel = model.material("matte_black_desk", rgba=(0.025, 0.027, 0.028, 1.0))
    rubber = model.material("dull_black_rubber", rgba=(0.006, 0.006, 0.005, 1.0))
    worn_metal = model.material("dark_worn_steel", rgba=(0.30, 0.31, 0.30, 1.0))

    tripod = model.part("tripod")
    tripod.visual(
        _tube_mesh(0.026, 0.019, 0.56, "outer_sleeve"),
        origin=Origin(xyz=(0.0, 0.0, 0.42)),
        material=black_metal,
        name="outer_sleeve",
    )
    tripod.visual(
        _tube_mesh(0.039, 0.019, 0.052, "collar_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.685)),
        material=black_metal,
        name="collar_ring",
    )
    tripod.visual(
        Box((0.0038, 0.018, 0.035)),
        origin=Origin(xyz=(0.0171, 0.0, 0.666)),
        material=worn_metal,
        name="guide_pad",
    )
    tripod.visual(
        Cylinder(radius=0.055, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.225)),
        material=black_metal,
        name="tripod_hub",
    )
    tripod.visual(
        Sphere(radius=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.220)),
        material=black_metal,
        name="hub_bulge",
    )

    for i, angle in enumerate((math.radians(90.0), math.radians(210.0), math.radians(330.0))):
        radial = (math.cos(angle), math.sin(angle))
        top = (0.038 * radial[0], 0.038 * radial[1], 0.235)
        foot = (0.42 * radial[0], 0.42 * radial[1], 0.026)
        leg_length, leg_origin = _cylinder_between(top, foot)
        tripod.visual(
            Cylinder(radius=0.012, length=leg_length),
            origin=leg_origin,
            material=black_metal,
            name=f"leg_{i}",
        )
        tripod.visual(
            Cylinder(radius=0.035, length=0.020),
            origin=Origin(xyz=(foot[0], foot[1], 0.018)),
            material=rubber,
            name=f"foot_{i}",
        )

    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=0.0155, length=0.86),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=worn_metal,
        name="inner_tube",
    )
    mast.visual(
        Cylinder(radius=0.020, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.427)),
        material=worn_metal,
        name="top_plug",
    )

    collar_knob = model.part("collar_knob")
    collar_knob.visual(
        Cylinder(radius=0.006, length=0.038),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=worn_metal,
        name="threaded_stem",
    )
    collar_knob.visual(
        Cylinder(radius=0.018, length=0.005),
        origin=Origin(xyz=(0.019, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=worn_metal,
        name="washer",
    )
    clamp_knob = KnobGeometry(
        0.055,
        0.030,
        body_style="lobed",
        base_diameter=0.040,
        top_diameter=0.050,
        crown_radius=0.0015,
        grip=KnobGrip(style="ribbed", count=6, depth=0.0015),
        bore=KnobBore(style="round", diameter=0.008),
    )
    collar_knob.visual(
        mesh_from_geometry(clamp_knob, "height_knob"),
        origin=Origin(xyz=(0.035, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_metal,
        name="knob_cap",
    )

    tilt_head = model.part("tilt_head")
    tilt_head.visual(
        Cylinder(radius=0.027, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=black_metal,
        name="mast_socket",
    )
    tilt_head.visual(
        Box((0.22, 0.024, 0.024)),
        origin=Origin(xyz=(0.0, -0.041, 0.055)),
        material=black_metal,
        name="yoke_bridge",
    )
    tilt_head.visual(
        Box((0.050, 0.020, 0.040)),
        origin=Origin(xyz=(0.0, -0.030, 0.040)),
        material=black_metal,
        name="neck_web",
    )
    for x, name in ((-0.092, "cheek_0"), (0.092, "cheek_1")):
        tilt_head.visual(
            Box((0.026, 0.045, 0.090)),
            origin=Origin(xyz=(x, -0.020, 0.070)),
            material=black_metal,
            name=name,
        )
    tilt_head.visual(
        Cylinder(radius=0.007, length=0.230),
        origin=Origin(xyz=(0.0, 0.0, 0.085), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=worn_metal,
        name="hinge_pin",
    )

    desk = model.part("desk")
    desk.visual(
        Cylinder(radius=0.014, length=0.140),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_metal,
        name="hinge_barrel",
    )
    desk.visual(
        Box((0.12, 0.026, 0.060)),
        origin=Origin(xyz=(0.0, 0.004, 0.043)),
        material=black_metal,
        name="hinge_web",
    )
    desk.visual(
        Box((0.54, 0.014, 0.335)),
        origin=Origin(xyz=(0.0, 0.012, 0.198)),
        material=dark_panel,
        name="panel",
    )
    desk.visual(
        Box((0.55, 0.018, 0.014)),
        origin=Origin(xyz=(0.0, 0.017, 0.372)),
        material=black_metal,
        name="top_rail",
    )
    for x, name in ((-0.278, "side_rail_0"), (0.278, "side_rail_1")):
        desk.visual(
            Box((0.014, 0.018, 0.340)),
            origin=Origin(xyz=(x, 0.017, 0.205)),
            material=black_metal,
            name=name,
        )
    desk.visual(
        Box((0.56, 0.065, 0.014)),
        origin=Origin(xyz=(0.0, 0.024, 0.042)),
        material=black_metal,
        name="lower_shelf",
    )
    desk.visual(
        Box((0.56, 0.012, 0.050)),
        origin=Origin(xyz=(0.0, 0.059, 0.065)),
        material=black_metal,
        name="retaining_lip",
    )
    for x, z, name in ((-0.20, 0.30, "rivet_0"), (0.20, 0.30, "rivet_1"), (-0.20, 0.11, "rivet_2"), (0.20, 0.11, "rivet_3")):
        desk.visual(
            Cylinder(radius=0.008, length=0.003),
            origin=Origin(xyz=(x, 0.0203, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=worn_metal,
            name=name,
        )

    model.articulation(
        "tripod_to_mast",
        ArticulationType.PRISMATIC,
        parent=tripod,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 0.700)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.20, lower=0.0, upper=0.30),
    )
    model.articulation(
        "tripod_to_collar_knob",
        ArticulationType.CONTINUOUS,
        parent=tripod,
        child=collar_knob,
        origin=Origin(xyz=(0.039, 0.0, 0.685)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=6.0),
    )
    model.articulation(
        "mast_to_tilt_head",
        ArticulationType.FIXED,
        parent=mast,
        child=tilt_head,
        origin=Origin(xyz=(0.0, 0.0, 0.440)),
    )
    model.articulation(
        "tilt_head_to_desk",
        ArticulationType.REVOLUTE,
        parent=tilt_head,
        child=desk,
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=-0.35, upper=0.70),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tripod = object_model.get_part("tripod")
    mast = object_model.get_part("mast")
    knob = object_model.get_part("collar_knob")
    head = object_model.get_part("tilt_head")
    desk = object_model.get_part("desk")
    height_slide = object_model.get_articulation("tripod_to_mast")
    tilt = object_model.get_articulation("tilt_head_to_desk")

    ctx.allow_overlap(
        tripod,
        knob,
        elem_a="collar_ring",
        elem_b="threaded_stem",
        reason="The threaded lock screw intentionally passes through the collar wall.",
    )
    ctx.allow_overlap(
        knob,
        tripod,
        elem_a="threaded_stem",
        elem_b="outer_sleeve",
        reason="The same lock screw passes through the outer sleeve wall below the collar.",
    )
    ctx.allow_overlap(
        head,
        desk,
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        reason="The desk hinge barrel is captured around the head pin.",
    )

    ctx.expect_within(
        mast,
        tripod,
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="outer_sleeve",
        margin=0.002,
        name="mast is centered in the hub tube",
    )
    ctx.expect_overlap(
        mast,
        tripod,
        axes="z",
        elem_a="inner_tube",
        elem_b="outer_sleeve",
        min_overlap=0.40,
        name="collapsed mast has retained insertion",
    )
    rest_pos = ctx.part_world_position(mast)
    with ctx.pose({height_slide: 0.30}):
        ctx.expect_within(
            mast,
            tripod,
            axes="xy",
            inner_elem="inner_tube",
            outer_elem="outer_sleeve",
            margin=0.002,
            name="extended mast remains centered",
        )
        ctx.expect_overlap(
            mast,
            tripod,
            axes="z",
            elem_a="inner_tube",
            elem_b="outer_sleeve",
            min_overlap=0.10,
            name="extended mast remains inserted",
        )
        extended_pos = ctx.part_world_position(mast)
    ctx.check(
        "mast slides upward",
        rest_pos is not None and extended_pos is not None and extended_pos[2] > rest_pos[2] + 0.25,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    ctx.expect_overlap(
        head,
        desk,
        axes="x",
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        min_overlap=0.12,
        name="head pin spans the desk hinge barrel",
    )
    ctx.expect_gap(
        desk,
        head,
        axis="z",
        positive_elem="hinge_barrel",
        negative_elem="hinge_pin",
        max_penetration=0.023,
        name="hinge barrel surrounds the pin locally",
    )

    ctx.expect_overlap(
        knob,
        tripod,
        axes="x",
        elem_a="threaded_stem",
        elem_b="collar_ring",
        min_overlap=0.010,
        name="lock screw crosses collar wall",
    )
    ctx.expect_overlap(
        knob,
        tripod,
        axes="x",
        elem_a="threaded_stem",
        elem_b="outer_sleeve",
        min_overlap=0.005,
        name="lock screw crosses sleeve wall",
    )

    rest_panel = ctx.part_element_world_aabb(desk, elem="panel")
    with ctx.pose({tilt: 0.60}):
        tilted_panel = ctx.part_element_world_aabb(desk, elem="panel")
    if rest_panel is not None and tilted_panel is not None:
        rest_y = (rest_panel[0][1] + rest_panel[1][1]) / 2.0
        tilted_y = (tilted_panel[0][1] + tilted_panel[1][1]) / 2.0
        ok = tilted_y < rest_y - 0.05
    else:
        rest_y = tilted_y = None
        ok = False
    ctx.check("desk tilts rearward on the horizontal hinge", ok, details=f"rest_y={rest_y}, tilted_y={tilted_y}")

    knob_joint = object_model.get_articulation("tripod_to_collar_knob")
    ctx.check(
        "collar knob is continuous",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={knob_joint.articulation_type}",
    )

    return ctx.report()


object_model = build_object_model()
