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
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

HOB_WIDTH = 0.60
HOB_DEPTH = 0.51
TOP_THICKNESS = 0.008
BODY_WIDTH = 0.56
BODY_DEPTH = 0.45
BODY_HEIGHT = 0.032
CONTROL_RAIL_DEPTH = 0.050
CONTROL_RAIL_HEIGHT = 0.054
FRONT_FACE_Y = HOB_DEPTH / 2.0
CONTROL_RAIL_CENTER_Y = FRONT_FACE_Y - CONTROL_RAIL_DEPTH / 2.0
KNOB_AXIS_Y = FRONT_FACE_Y
KNOB_AXIS_Z = -0.029
COVER_AXIS_Y = FRONT_FACE_Y + 0.002
COVER_AXIS_Z = -0.010

BURNER_LAYOUT = (
    {"name": "burner_0", "xyz": (-0.165, 0.096, 0.0), "tray_radius": 0.058, "ring_radius": 0.043, "cap_radius": 0.025, "grate_size": (0.195, 0.195)},
    {"name": "burner_1", "xyz": (0.165, 0.096, 0.0), "tray_radius": 0.048, "ring_radius": 0.035, "cap_radius": 0.021, "grate_size": (0.175, 0.175)},
    {"name": "burner_2", "xyz": (-0.165, -0.096, 0.0), "tray_radius": 0.048, "ring_radius": 0.035, "cap_radius": 0.021, "grate_size": (0.175, 0.175)},
    {"name": "burner_3", "xyz": (0.165, -0.096, 0.0), "tray_radius": 0.058, "ring_radius": 0.043, "cap_radius": 0.025, "grate_size": (0.195, 0.195)},
)

KNOB_X = (-0.180, -0.060, 0.060, 0.180)


def _make_grate_shape(width: float, depth: float) -> cq.Workplane:
    foot_height = 0.024
    bar_height = 0.006
    frame_bar = 0.012
    cross_bar = 0.010
    top_z = foot_height + bar_height / 2.0
    foot_size = 0.012
    foot_inset_x = width / 2.0 - frame_bar / 2.0
    foot_inset_y = depth / 2.0 - frame_bar / 2.0
    ring_outer = min(width, depth) * 0.26
    ring_inner = ring_outer - 0.010

    shape = (
        cq.Workplane("XY")
        .box(width, frame_bar, bar_height)
        .translate((0.0, depth / 2.0 - frame_bar / 2.0, top_z))
    )
    shape = shape.union(
        cq.Workplane("XY")
        .box(width, frame_bar, bar_height)
        .translate((0.0, -depth / 2.0 + frame_bar / 2.0, top_z))
    )
    shape = shape.union(
        cq.Workplane("XY")
        .box(frame_bar, depth, bar_height)
        .translate((width / 2.0 - frame_bar / 2.0, 0.0, top_z))
    )
    shape = shape.union(
        cq.Workplane("XY")
        .box(frame_bar, depth, bar_height)
        .translate((-width / 2.0 + frame_bar / 2.0, 0.0, top_z))
    )
    shape = shape.union(
        cq.Workplane("XY")
        .box(width - frame_bar, cross_bar, bar_height)
        .translate((0.0, 0.0, top_z))
    )
    shape = shape.union(
        cq.Workplane("XY")
        .box(cross_bar, depth - frame_bar, bar_height)
        .translate((0.0, 0.0, top_z))
    )
    shape = shape.union(
        cq.Workplane("XY")
        .circle(ring_outer)
        .circle(ring_inner)
        .extrude(bar_height)
        .translate((0.0, 0.0, foot_height))
    )

    for x in (-foot_inset_x, foot_inset_x):
        for y in (-foot_inset_y, foot_inset_y):
            shape = shape.union(
                cq.Workplane("XY")
                .box(foot_size, foot_size, foot_height + 0.001)
                .translate((x, y, (foot_height + 0.001) / 2.0 - 0.001))
            )
    return shape


def _knob_mesh():
    return mesh_from_geometry(
        KnobGeometry(
            0.042,
            0.026,
            body_style="skirted",
            top_diameter=0.034,
            skirt=KnobSkirt(0.052, 0.006, flare=0.06),
            grip=KnobGrip(style="fluted", count=18, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008, angle_deg=0.0),
            bore=KnobBore(style="d_shaft", diameter=0.006, flat_depth=0.001),
            center=False,
        ),
        "gas_hob_knob",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gas_hob")

    stainless = model.material("stainless", rgba=(0.79, 0.80, 0.80, 1.0))
    rail_finish = model.material("rail_finish", rgba=(0.72, 0.73, 0.74, 1.0))
    shadow = model.material("shadow", rgba=(0.19, 0.20, 0.22, 1.0))
    burner_finish = model.material("burner_finish", rgba=(0.30, 0.31, 0.33, 1.0))
    cast_iron = model.material("cast_iron", rgba=(0.09, 0.09, 0.10, 1.0))
    knob_finish = model.material("knob_finish", rgba=(0.12, 0.12, 0.13, 1.0))
    ignition_finish = model.material("ignition_finish", rgba=(0.13, 0.13, 0.14, 1.0))
    cover_tint = model.material("cover_tint", rgba=(0.24, 0.25, 0.28, 0.78))
    hinge_finish = model.material("hinge_finish", rgba=(0.58, 0.60, 0.62, 1.0))

    small_grate = mesh_from_cadquery(_make_grate_shape(0.175, 0.175), "small_grate")
    large_grate = mesh_from_cadquery(_make_grate_shape(0.195, 0.195), "large_grate")
    knob_mesh = _knob_mesh()

    hob = model.part("hob")
    hob.visual(
        Box((HOB_WIDTH, HOB_DEPTH, TOP_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, -TOP_THICKNESS / 2.0)),
        material=stainless,
        name="top_plate",
    )
    hob.visual(
        Box((BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.008, -TOP_THICKNESS - BODY_HEIGHT / 2.0)),
        material=shadow,
        name="underbody",
    )
    hob.visual(
        Box((BODY_WIDTH, CONTROL_RAIL_DEPTH, CONTROL_RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, CONTROL_RAIL_CENTER_Y, -CONTROL_RAIL_HEIGHT / 2.0)),
        material=rail_finish,
        name="control_rail",
    )
    hob.visual(
        Box((0.106, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, FRONT_FACE_Y - 0.006, COVER_AXIS_Z + 0.004)),
        material=rail_finish,
        name="cover_header",
    )

    for index, knob_x in enumerate(KNOB_X):
        hob.visual(
            Cylinder(radius=0.025, length=0.004),
            origin=Origin(
                xyz=(knob_x, FRONT_FACE_Y - 0.002, KNOB_AXIS_Z),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=rail_finish,
            name=f"knob_bezel_{index}",
        )

    switch_positions = (-0.020, 0.020)
    for index, switch_x in enumerate(switch_positions):
        hob.visual(
            Box((0.016, 0.006, 0.010)),
            origin=Origin(xyz=(switch_x, FRONT_FACE_Y - 0.001, -0.026)),
            material=ignition_finish,
            name=f"ignition_switch_{index}",
        )

    for index, burner in enumerate(BURNER_LAYOUT):
        x, y, _ = burner["xyz"]
        hob.visual(
            Cylinder(radius=burner["tray_radius"], length=0.005),
            origin=Origin(xyz=(x, y, 0.0025)),
            material=burner_finish,
            name=f"burner_tray_{index}",
        )
        hob.visual(
            Cylinder(radius=burner["ring_radius"], length=0.008),
            origin=Origin(xyz=(x, y, 0.0060)),
            material=cast_iron,
            name=f"burner_ring_{index}",
        )
        hob.visual(
            Cylinder(radius=burner["cap_radius"], length=0.010),
            origin=Origin(xyz=(x, y, 0.010)),
            material=cast_iron,
            name=f"burner_cap_{index}",
        )
        hob.visual(
            large_grate if burner["grate_size"][0] > 0.180 else small_grate,
            origin=Origin(xyz=(x, y, -0.001)),
            material=cast_iron,
            name=f"grate_{index}",
        )

    cover = model.part("safety_cover")
    cover.visual(
        Cylinder(radius=0.0022, length=0.084),
        origin=Origin(xyz=(0.0, 0.0014, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_finish,
        name="cover_hinge",
    )
    cover.visual(
        Box((0.090, 0.003, 0.030)),
        origin=Origin(xyz=(0.0, 0.0015, -0.015)),
        material=cover_tint,
        name="cover_panel",
    )
    cover.visual(
        Box((0.032, 0.007, 0.003)),
        origin=Origin(xyz=(0.0, 0.0045, -0.029)),
        material=cover_tint,
        name="cover_lip",
    )

    model.articulation(
        "cover_hinge_joint",
        ArticulationType.REVOLUTE,
        parent=hob,
        child=cover,
        origin=Origin(xyz=(0.0, COVER_AXIS_Y, COVER_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.45, effort=1.0, velocity=2.5),
    )

    for index, knob_x in enumerate(KNOB_X):
        knob = model.part(f"knob_{index}")
        knob.visual(
            knob_mesh,
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=knob_finish,
            name="knob_shell",
        )
        model.articulation(
            f"knob_{index}_spin",
            ArticulationType.CONTINUOUS,
            parent=hob,
            child=knob,
            origin=Origin(xyz=(knob_x, KNOB_AXIS_Y, KNOB_AXIS_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.4, velocity=8.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    hob = object_model.get_part("hob")
    cover = object_model.get_part("safety_cover")
    cover_hinge = object_model.get_articulation("cover_hinge_joint")
    knobs = [object_model.get_part(f"knob_{index}") for index in range(4)]
    knob_joints = [object_model.get_articulation(f"knob_{index}_spin") for index in range(4)]

    ctx.expect_gap(
        cover,
        hob,
        axis="y",
        positive_elem="cover_panel",
        negative_elem="control_rail",
        min_gap=0.001,
        max_gap=0.006,
        name="cover sits just proud of the control rail",
    )
    ctx.expect_overlap(
        cover,
        hob,
        axes="xz",
        elem_a="cover_panel",
        elem_b="control_rail",
        min_overlap=0.025,
        name="cover spans the ignition area on the rail",
    )

    closed_cover_aabb = ctx.part_world_aabb(cover)
    with ctx.pose({cover_hinge: 1.25}):
        open_cover_aabb = ctx.part_world_aabb(cover)
    ctx.check(
        "cover opens upward and outward",
        closed_cover_aabb is not None
        and open_cover_aabb is not None
        and open_cover_aabb[0][2] > closed_cover_aabb[0][2] + 0.018
        and open_cover_aabb[1][1] > closed_cover_aabb[1][1] + 0.012,
        details=f"closed={closed_cover_aabb}, open={open_cover_aabb}",
    )

    knob_positions = [ctx.part_world_position(knob) for knob in knobs]
    ctx.check(
        "four knob shafts line the front rail",
        all(position is not None for position in knob_positions)
        and all(abs(position[1] - KNOB_AXIS_Y) < 0.0015 for position in knob_positions if position is not None)
        and all(abs(position[2] - KNOB_AXIS_Z) < 0.0015 for position in knob_positions if position is not None)
        and knob_positions[0][0] < knob_positions[1][0] < knob_positions[2][0] < knob_positions[3][0],
        details=f"positions={knob_positions}",
    )

    for index, knob in enumerate(knobs):
        ctx.expect_gap(
            knob,
            hob,
            axis="y",
            positive_elem="knob_shell",
            negative_elem="control_rail",
            min_gap=0.0,
            max_gap=0.008,
            name=f"knob_{index} projects forward of the rail",
        )

    burner_caps_present = True
    try:
        for index in range(4):
            hob.get_visual(f"burner_cap_{index}")
            hob.get_visual(f"grate_{index}")
    except Exception:
        burner_caps_present = False
    ctx.check(
        "four burner stations are authored",
        burner_caps_present,
        details="Expected burner cap and grate visuals for all four stations.",
    )

    continuous_knobs = all(
        joint.articulation_type == ArticulationType.CONTINUOUS and joint.motion_limits is not None
        for joint in knob_joints
    )
    ctx.check(
        "knobs use continuous rotation joints",
        continuous_knobs,
        details=f"joint_types={[joint.articulation_type for joint in knob_joints]}",
    )

    return ctx.report()


object_model = build_object_model()
