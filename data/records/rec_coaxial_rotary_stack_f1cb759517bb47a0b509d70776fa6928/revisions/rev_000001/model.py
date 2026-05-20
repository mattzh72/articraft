from __future__ import annotations

import math

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


def _annular_cylinder(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    """Centered annular solid with a true central through-bore."""
    outer = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .extrude(height)
        .translate((0.0, 0.0, -height / 2.0))
    )
    cutter = (
        cq.Workplane("XY")
        .circle(inner_radius)
        .extrude(height * 2.0)
        .translate((0.0, 0.0, -height))
    )
    return outer.cut(cutter)


def _central_bore(radius: float, height: float = 0.10) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(height)
        .translate((0.0, 0.0, -height / 2.0))
    )


def _soften_vertical_edges(shape: cq.Workplane, radius: float) -> cq.Workplane:
    """Best-effort small roundovers; keep the exact solid if CadQuery cannot fillet."""
    try:
        return shape.edges("|Z").fillet(radius)
    except Exception:
        return shape


def _lower_stage_shape() -> cq.Workplane:
    """Broad lower platter with raised outer rim and a rotating bearing sleeve."""
    deck_height = 0.030
    deck = _soften_vertical_edges(_annular_cylinder(0.420, 0.036, deck_height), 0.003)

    outer_lip = (
        _annular_cylinder(0.420, 0.378, 0.014)
        .translate((0.0, 0.0, deck_height / 2.0))
    )
    outer_lip = _soften_vertical_edges(outer_lip, 0.002)
    center_sleeve = _soften_vertical_edges(_annular_cylinder(0.074, 0.036, 0.046), 0.002)
    shallow_groove = (
        _annular_cylinder(0.300, 0.286, 0.006)
        .translate((0.0, 0.0, deck_height / 2.0 + 0.002))
    )
    shallow_groove = _soften_vertical_edges(shallow_groove, 0.001)
    return deck.union(outer_lip).union(center_sleeve).union(shallow_groove)


def _middle_ring_shape() -> cq.Workplane:
    """Open middle ring with four radial bridges to a central bearing collar."""
    ring = _soften_vertical_edges(_annular_cylinder(0.315, 0.230, 0.026), 0.0025)
    collar = _soften_vertical_edges(_annular_cylinder(0.072, 0.036, 0.038), 0.002)
    shape = ring.union(collar)

    for angle in (0.0, 90.0, 180.0, 270.0):
        spoke = (
            cq.Workplane("XY")
            .box(0.360, 0.045, 0.022)
            .translate((0.160, 0.0, 0.0))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle)
        )
        spoke = _soften_vertical_edges(spoke, 0.006)
        shape = shape.union(spoke)

    return shape.cut(_central_bore(0.036, 0.080))




def _top_plate_shape() -> cq.Workplane:
    """Smaller upper plate with a visible bore, raised rim, and shallow top pad."""
    plate = _soften_vertical_edges(_annular_cylinder(0.220, 0.036, 0.024), 0.0025)
    rim = (
        _annular_cylinder(0.220, 0.188, 0.010)
        .translate((0.0, 0.0, 0.016))
    )
    rim = _soften_vertical_edges(rim, 0.0015)
    center_boss = _soften_vertical_edges(_annular_cylinder(0.060, 0.036, 0.034), 0.0015)
    grip_pad = (
        _annular_cylinder(0.150, 0.120, 0.004)
        .translate((0.0, 0.0, 0.014))
    )
    grip_pad = _soften_vertical_edges(grip_pad, 0.001)
    return plate.union(rim).union(center_boss).union(grip_pad)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_coaxial_turntable")

    brushed_steel = Material("brushed_steel", rgba=(0.68, 0.70, 0.70, 1.0))
    dark_plate = Material("dark_anodized_plate", rgba=(0.05, 0.055, 0.06, 1.0))
    satin_blue = Material("satin_blue_ring", rgba=(0.05, 0.19, 0.42, 1.0))
    warm_aluminum = Material("warm_aluminum", rgba=(0.78, 0.74, 0.66, 1.0))
    bearing_black = Material("black_bearing", rgba=(0.01, 0.01, 0.012, 1.0))

    model.materials.extend(
        [brushed_steel, dark_plate, satin_blue, warm_aluminum, bearing_black]
    )

    bridge_frame = model.part("bridge_frame")
    bridge_frame.visual(
        Box((1.12, 0.82, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=brushed_steel,
        name="base_slab",
    )
    bridge_frame.visual(
        Box((0.07, 0.16, 0.36)),
        origin=Origin(xyz=(-0.50, 0.0, 0.22)),
        material=brushed_steel,
        name="pier_0",
    )
    bridge_frame.visual(
        Box((0.07, 0.16, 0.36)),
        origin=Origin(xyz=(0.50, 0.0, 0.22)),
        material=brushed_steel,
        name="pier_1",
    )
    bridge_frame.visual(
        Box((1.07, 0.15, 0.065)),
        origin=Origin(xyz=(0.0, 0.0, 0.425)),
        material=brushed_steel,
        name="bridge_beam",
    )
    bridge_frame.visual(
        Cylinder(radius=0.028, length=0.365),
        origin=Origin(xyz=(0.0, 0.0, 0.2225)),
        material=brushed_steel,
        name="fixed_shaft",
    )
    bridge_frame.visual(
        Cylinder(radius=0.058, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.053)),
        material=bearing_black,
        name="lower_bearing",
    )
    bridge_frame.visual(
        Cylinder(radius=0.070, length=0.017),
        origin=Origin(xyz=(0.0, 0.0, 0.1175)),
        material=bearing_black,
        name="middle_bearing",
    )
    bridge_frame.visual(
        Cylinder(radius=0.060, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.178)),
        material=bearing_black,
        name="top_bearing",
    )
    bridge_frame.visual(
        Cylinder(radius=0.055, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.3825)),
        material=bearing_black,
        name="upper_bearing",
    )

    lower_stage = model.part("lower_stage")
    lower_stage.visual(
        mesh_from_cadquery(_lower_stage_shape(), "lower_stage"),
        material=dark_plate,
        name="lower_platter",
    )

    middle_ring = model.part("middle_ring")
    middle_ring.visual(
        mesh_from_cadquery(_middle_ring_shape(), "middle_ring"),
        material=satin_blue,
        name="ring_spider",
    )

    top_plate = model.part("top_plate")
    top_plate.visual(
        mesh_from_cadquery(_top_plate_shape(), "top_plate"),
        material=warm_aluminum,
        name="top_disk",
    )

    full_turn = MotionLimits(
        effort=20.0, velocity=2.0, lower=-math.pi, upper=math.pi
    )
    model.articulation(
        "bridge_to_lower_stage",
        ArticulationType.REVOLUTE,
        parent=bridge_frame,
        child=lower_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=full_turn,
    )
    model.articulation(
        "bridge_to_middle_ring",
        ArticulationType.REVOLUTE,
        parent=bridge_frame,
        child=middle_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=full_turn,
    )
    model.articulation(
        "bridge_to_top_plate",
        ArticulationType.REVOLUTE,
        parent=bridge_frame,
        child=top_plate,
        origin=Origin(xyz=(0.0, 0.0, 0.205)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=full_turn,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bridge = object_model.get_part("bridge_frame")
    lower = object_model.get_part("lower_stage")
    middle = object_model.get_part("middle_ring")
    top = object_model.get_part("top_plate")
    lower_joint = object_model.get_articulation("bridge_to_lower_stage")
    middle_joint = object_model.get_articulation("bridge_to_middle_ring")
    top_joint = object_model.get_articulation("bridge_to_top_plate")

    joints = (lower_joint, middle_joint, top_joint)
    origins = [joint.origin.xyz for joint in joints]
    ctx.check(
        "three stages use one shaft line",
        all(abs(origin[0]) < 1e-9 and abs(origin[1]) < 1e-9 for origin in origins)
        and all(tuple(joint.axis) == (0.0, 0.0, 1.0) for joint in joints),
        details=f"origins={origins}, axes={[joint.axis for joint in joints]}",
    )
    ctx.check(
        "stage joint heights are stacked",
        origins[0][2] < origins[1][2] < origins[2][2],
        details=f"joint z heights={[origin[2] for origin in origins]}",
    )
    ctx.check(
        "each stage can make a full turn",
        all(
            joint.motion_limits is not None
            and joint.motion_limits.lower <= -math.pi
            and joint.motion_limits.upper >= math.pi
            for joint in joints
        ),
        details=f"limits={[joint.motion_limits for joint in joints]}",
    )

    ctx.expect_contact(
        bridge,
        lower,
        elem_a="lower_bearing",
        elem_b="lower_platter",
        name="lower stage bears on fixed race",
    )
    ctx.expect_contact(
        bridge,
        middle,
        elem_a="middle_bearing",
        elem_b="ring_spider",
        name="middle ring bears on fixed race",
    )
    ctx.expect_contact(
        bridge,
        top,
        elem_a="top_bearing",
        elem_b="top_disk",
        name="top plate bears on fixed race",
    )
    ctx.expect_within(
        top,
        lower,
        axes="xy",
        elem_a="top_disk",
        elem_b="lower_platter",
        margin=0.0,
        name="top plate is smaller than lower platter",
    )

    rest_positions = [ctx.part_world_position(part) for part in (lower, middle, top)]
    with ctx.pose({lower_joint: 0.75, middle_joint: -0.50, top_joint: 1.10}):
        moved_positions = [ctx.part_world_position(part) for part in (lower, middle, top)]
        ctx.expect_contact(
            bridge,
            middle,
            elem_a="middle_bearing",
            elem_b="ring_spider",
            name="middle bearing remains coaxial while turned",
        )

    ctx.check(
        "independent turns stay centered",
        rest_positions == moved_positions,
        details=f"rest={rest_positions}, moved={moved_positions}",
    )

    return ctx.report()


object_model = build_object_model()
