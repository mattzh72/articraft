from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    HingeHolePattern,
    Mimic,
    MotionLimits,
    Origin,
    PianoHingeGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _open_freezer_shell(width: float, depth: float, height: float, wall: float, bottom: float) -> cq.Workplane:
    """A hollow, open-topped insulated freezer body in the model frame."""
    outer = cq.Workplane("XY").box(width, depth, height, centered=(True, True, False))
    cutter = (
        cq.Workplane("XY")
        .box(width - 2.0 * wall, depth - 2.0 * wall, height + 0.04, centered=(True, True, False))
        .translate((0.0, 0.0, bottom))
    )
    return outer.cut(cutter)


def _rect_ring(width: float, depth: float, height: float, border: float, *, center_y: float = 0.0) -> cq.Workplane:
    outer = cq.Workplane("XY").box(width, depth, height, centered=(True, True, True)).translate((0.0, center_y, 0.0))
    inner = (
        cq.Workplane("XY")
        .box(width - 2.0 * border, depth - 2.0 * border, height * 2.5, centered=(True, True, True))
        .translate((0.0, center_y, 0.0))
    )
    return outer.cut(inner)


def _slotted_hasp_arm(width: float, thickness: float, length: float) -> cq.Workplane:
    """Vertical hasp plate with a real through-slot for the body staple."""
    plate = (
        cq.Workplane("XY")
        .box(width, thickness, length, centered=(True, True, True))
        .translate((0.0, -0.018, -0.132))
    )
    slot = (
        cq.Workplane("XY")
        .box(width * 0.76, thickness * 3.0, length * 0.48, centered=(True, True, True))
        .translate((0.0, -0.018, -0.155))
    )
    return plate.cut(slot)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="glass_top_display_chest_freezer")

    cabinet_white = model.material("powder_coated_white", rgba=(0.92, 0.94, 0.93, 1.0))
    liner_mat = model.material("white_plastic_liner", rgba=(0.82, 0.88, 0.90, 1.0))
    gasket_black = model.material("black_rubber_gasket", rgba=(0.01, 0.012, 0.012, 1.0))
    aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.72, 0.68, 1.0))
    galvanized = model.material("galvanized_steel", rgba=(0.62, 0.62, 0.58, 1.0))
    dark_grille = model.material("dark_vent_grille", rgba=(0.05, 0.055, 0.06, 1.0))
    glass = model.material("slightly_blue_glass", rgba=(0.55, 0.80, 0.95, 0.34))

    width = 1.80
    depth = 0.75
    body_h = 0.82
    wall = 0.070
    bottom = 0.115
    gasket_h = 0.025
    lid_depth = 0.78
    lid_w = 1.84
    lid_t = 0.050
    rear_y = depth / 2.0
    hinge_y = rear_y + 0.015
    hinge_z = body_h + gasket_h + lid_t / 2.0

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_open_freezer_shell(width, depth, body_h, wall, bottom), "insulated_body_shell"),
        material=cabinet_white,
        name="insulated_shell",
    )
    body.visual(
        mesh_from_cadquery(_rect_ring(width + 0.035, depth + 0.035, gasket_h, wall * 0.68), "top_gasket_ring"),
        origin=Origin(xyz=(0.0, 0.0, body_h + gasket_h / 2.0)),
        material=gasket_black,
        name="top_gasket",
    )
    body.visual(
        Box((width - 2.0 * wall, depth - 2.0 * wall, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, bottom + 0.009)),
        material=liner_mat,
        name="inner_floor",
    )
    # Front compressor ventilation slots and a recessed toe kick make it read as an appliance.
    for i, x in enumerate((-0.30, -0.18, -0.06, 0.06, 0.18, 0.30)):
        body.visual(
            Box((0.065, 0.006, 0.180)),
            origin=Origin(xyz=(x, -depth / 2.0 - 0.002, 0.235)),
            material=dark_grille,
            name=f"front_vent_{i}",
        )
    body.visual(
        Box((0.145, 0.012, 0.090)),
        origin=Origin(xyz=(0.0, -depth / 2.0 - 0.005, 0.720)),
        material=galvanized,
        name="lock_plate",
    )
    # Padlock staple/loop on the body: two upright rods, a crossbar, and short welded feet.
    for i, x in enumerate((-0.032, 0.032)):
        body.visual(
            Cylinder(radius=0.0065, length=0.092),
            origin=Origin(xyz=(x, -depth / 2.0 - 0.040, 0.720)),
            material=galvanized,
            name=f"padlock_loop_post_{i}",
        )
        body.visual(
            Cylinder(radius=0.0065, length=0.034),
            origin=Origin(xyz=(x, -depth / 2.0 - 0.024, 0.676), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=galvanized,
            name=f"padlock_loop_foot_{i}",
        )
    body.visual(
        Cylinder(radius=0.0065, length=0.064),
        origin=Origin(xyz=(0.0, -depth / 2.0 - 0.040, 0.766), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="padlock_loop_crossbar",
    )
    for i, x in enumerate((-0.73, 0.73)):
        body.visual(
            Box((0.14, 0.10, 0.055)),
            origin=Origin(xyz=(x, -0.27, 0.0275)),
            material=dark_grille,
            name=f"front_foot_{i}",
        )
        body.visual(
            Box((0.14, 0.10, 0.055)),
            origin=Origin(xyz=(x, 0.27, 0.0275)),
            material=dark_grille,
            name=f"rear_foot_{i}",
        )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_rect_ring(lid_w, lid_depth, lid_t, 0.080, center_y=-lid_depth / 2.0), "aluminum_lid_frame"),
        material=aluminum,
        name="lid_frame",
    )
    lid.visual(
        Box((lid_w - 0.13, lid_depth - 0.13, 0.022)),
        origin=Origin(xyz=(0.0, -lid_depth / 2.0, 0.004)),
        material=glass,
        name="glass_panel",
    )
    lid.visual(
        Box((0.145, 0.030, 0.035)),
        origin=Origin(xyz=(0.0, -lid_depth - 0.005, -0.006)),
        material=galvanized,
        name="hasp_hinge_leaf",
    )

    hinge_holes = HingeHolePattern(style="round", count=9, diameter=0.005, edge_margin=0.070)
    piano_hinge_0 = PianoHingeGeometry(
        width * 0.96,
        leaf_width_a=0.035,
        leaf_width_b=0.035,
        leaf_thickness=0.003,
        pin_diameter=0.007,
        knuckle_pitch=0.075,
        holes_a=hinge_holes,
        holes_b=hinge_holes,
    )
    piano_hinge_1 = PianoHingeGeometry(
        width * 0.96,
        leaf_width_a=0.030,
        leaf_width_b=0.030,
        leaf_thickness=0.003,
        pin_diameter=0.006,
        knuckle_pitch=0.065,
        holes_a=hinge_holes,
        holes_b=hinge_holes,
    )
    body.visual(
        mesh_from_geometry(piano_hinge_0, "full_width_piano_hinge_0"),
        origin=Origin(xyz=(0.0, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="piano_hinge_0",
    )

    rear_hinge_leaf = model.part("rear_hinge_leaf")
    rear_hinge_leaf.visual(
        mesh_from_geometry(piano_hinge_1, "full_width_piano_hinge_1"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="piano_hinge_1",
    )
    rear_hinge_leaf.visual(
        Box((width * 0.96, 0.026, 0.030)),
        origin=Origin(xyz=(0.0, -0.0045, 0.0)),
        material=galvanized,
        name="rear_hinge_back_leaf",
    )

    hasp = model.part("hasp_arm")
    hasp.visual(
        mesh_from_cadquery(_slotted_hasp_arm(0.118, 0.012, 0.255), "slotted_security_hasp"),
        material=galvanized,
        name="slotted_arm",
    )
    hasp.visual(
        Cylinder(radius=0.014, length=0.145),
        origin=Origin(xyz=(0.0, -0.018, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="hasp_barrel",
    )

    model.articulation(
        "rear_hinge_0",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.8, lower=0.0, upper=1.25),
    )
    model.articulation(
        "rear_hinge_1",
        ArticulationType.REVOLUTE,
        parent=body,
        child=rear_hinge_leaf,
        origin=Origin(xyz=(0.0, hinge_y + 0.002, hinge_z - 0.100)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.8, lower=0.0, upper=1.25),
        mimic=Mimic(joint="rear_hinge_0", multiplier=1.0, offset=0.0),
    )
    model.articulation(
        "hasp_hinge",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=hasp,
        origin=Origin(xyz=(0.0, -lid_depth - 0.010, -0.002)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=0.0, upper=1.55),
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
    lid = object_model.get_part("lid")
    hasp = object_model.get_part("hasp_arm")
    hinge = object_model.get_articulation("rear_hinge_0")
    hasp_hinge = object_model.get_articulation("hasp_hinge")

    ctx.allow_overlap(
        body,
        lid,
        elem_a="piano_hinge_0",
        elem_b="lid_frame",
        reason="The fixed lower piano-hinge strip is seated under the rear aluminum lid frame as a local hidden hinge-leaf embed.",
    )
    ctx.allow_overlap(
        hasp,
        lid,
        elem_a="hasp_barrel",
        elem_b="hasp_hinge_leaf",
        reason="The hasp barrel is intentionally captured in the small hinge leaf mounted on the lid front.",
    )

    ctx.expect_contact(
        lid,
        body,
        elem_a="lid_frame",
        elem_b="top_gasket",
        contact_tol=0.003,
        name="closed glass lid seats on the rubber gasket",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="glass_panel",
        elem_b="insulated_shell",
        min_overlap=0.45,
        name="glass display panel spans the freezer opening",
    )
    ctx.expect_within(
        hasp,
        body,
        axes="x",
        inner_elem="slotted_arm",
        outer_elem="lock_plate",
        margin=0.01,
        name="hasp arm aligns over the front lock plate",
    )
    ctx.expect_overlap(
        hasp,
        body,
        axes="xz",
        elem_a="slotted_arm",
        elem_b="padlock_loop_crossbar",
        min_overlap=0.010,
        name="body staple passes through the dropped hasp slot",
    )
    ctx.expect_overlap(
        body,
        lid,
        axes="x",
        elem_a="piano_hinge_0",
        elem_b="lid_frame",
        min_overlap=1.50,
        name="full-width rear piano hinge spans the lid frame",
    )
    ctx.expect_contact(
        hasp,
        lid,
        elem_a="hasp_barrel",
        elem_b="hasp_hinge_leaf",
        contact_tol=0.002,
        name="hasp barrel is captured by the front hinge leaf",
    )

    rest_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({hinge: 0.85}):
        open_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "rear hinge opens the heavy glass lid upward",
        rest_aabb is not None and open_aabb is not None and open_aabb[1][2] > rest_aabb[1][2] + 0.20,
        details=f"rest_aabb={rest_aabb}, open_aabb={open_aabb}",
    )

    rest_hasp_aabb = ctx.part_element_world_aabb(hasp, elem="slotted_arm")
    with ctx.pose({hasp_hinge: 1.35}):
        swung_hasp_aabb = ctx.part_element_world_aabb(hasp, elem="slotted_arm")
    ctx.check(
        "hasp hinge swings the arm upward off the padlock loop",
        rest_hasp_aabb is not None
        and swung_hasp_aabb is not None
        and swung_hasp_aabb[0][2] > rest_hasp_aabb[0][2] + 0.060,
        details=f"rest_hasp_aabb={rest_hasp_aabb}, swung_hasp_aabb={swung_hasp_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
