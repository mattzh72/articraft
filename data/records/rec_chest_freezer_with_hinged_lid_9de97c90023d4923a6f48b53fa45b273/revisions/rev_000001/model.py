from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_DEPTH = 0.75
BODY_WIDTH = 1.10
BODY_HEIGHT = 0.78
WALL_THICKNESS = 0.065
FLOOR_THICKNESS = 0.075
RIM_HEIGHT = 0.035
RIM_TOP_Z = BODY_HEIGHT + RIM_HEIGHT

HINGE_X = -BODY_DEPTH / 2.0 - 0.050
HINGE_Z = RIM_TOP_Z + 0.080

LID_DEPTH = 0.82
LID_WIDTH = 1.18
LID_THICKNESS = 0.12
LID_REAR_CLEARANCE = 0.045
GASKET_HEIGHT = 0.018
GASKET_GAP = 0.002


def _box_at(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _ring_box(
    outer_size: tuple[float, float, float],
    inner_size: tuple[float, float, float],
    center: tuple[float, float, float],
) -> cq.Workplane:
    outer = _box_at(outer_size, center)
    cutter = _box_at((inner_size[0], inner_size[1], outer_size[2] + 0.020), center)
    return outer.cut(cutter)


def _body_shell_mesh():
    inner_depth = BODY_DEPTH - 2.0 * WALL_THICKNESS
    inner_width = BODY_WIDTH - 2.0 * WALL_THICKNESS

    outer = _box_at((BODY_DEPTH, BODY_WIDTH, BODY_HEIGHT), (0.0, 0.0, BODY_HEIGHT / 2.0))
    outer = outer.edges("|Z").fillet(0.035)

    cutter = _box_at(
        (inner_depth, inner_width, BODY_HEIGHT + 0.20),
        (0.0, 0.0, FLOOR_THICKNESS + (BODY_HEIGHT + 0.20) / 2.0),
    )
    cutter = cutter.edges("|Z").fillet(0.022)
    shell = outer.cut(cutter)

    rim = _ring_box(
        (BODY_DEPTH + 0.030, BODY_WIDTH + 0.030, RIM_HEIGHT),
        (inner_depth - 0.030, inner_width - 0.030, RIM_HEIGHT + 0.020),
        (0.0, 0.0, BODY_HEIGHT + RIM_HEIGHT / 2.0),
    )
    rim = rim.edges("|Z").fillet(0.018)
    shell = shell.union(rim)

    return mesh_from_cadquery(shell, "insulated_body_shell", tolerance=0.002, angular_tolerance=0.15)


def _lid_shell_mesh():
    lid = _box_at(
        (LID_DEPTH, LID_WIDTH, LID_THICKNESS),
        (LID_REAR_CLEARANCE + LID_DEPTH / 2.0, 0.0, 0.0),
    )
    lid = lid.edges().fillet(0.025)
    raised_panel = _box_at(
        (LID_DEPTH - 0.18, LID_WIDTH - 0.18, 0.018),
        (LID_REAR_CLEARANCE + LID_DEPTH / 2.0 + 0.018, 0.0, LID_THICKNESS / 2.0 + 0.009),
    )
    raised_panel = raised_panel.edges("|Z").fillet(0.020)
    lid = lid.union(raised_panel)
    return mesh_from_cadquery(lid, "heavy_insulated_lid", tolerance=0.002, angular_tolerance=0.15)


def _gasket_mesh():
    outer = (LID_DEPTH - 0.12, LID_WIDTH - 0.14, GASKET_HEIGHT)
    inner = (outer[0] - 0.055, outer[1] - 0.055, GASKET_HEIGHT + 0.020)
    gasket = _ring_box(
        outer,
        inner,
        (
            LID_REAR_CLEARANCE + LID_DEPTH / 2.0 + 0.010,
            0.0,
            -LID_THICKNESS / 2.0 - GASKET_HEIGHT / 2.0,
        ),
    )
    gasket = gasket.edges("|Z").fillet(0.010)
    return mesh_from_cadquery(gasket, "pressure_seal_gasket", tolerance=0.0015, angular_tolerance=0.12)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="laboratory_sample_chest_freezer")

    enamel = model.material("white_powder_coated_steel", rgba=(0.92, 0.95, 0.96, 1.0))
    liner = model.material("pale_liner", rgba=(0.78, 0.84, 0.86, 1.0))
    gasket_rubber = model.material("black_pressure_seal", rgba=(0.01, 0.012, 0.012, 1.0))
    brushed_steel = model.material("brushed_stainless_steel", rgba=(0.62, 0.64, 0.64, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.025, 0.028, 0.030, 1.0))
    blue_label = model.material("blue_lab_label", rgba=(0.08, 0.24, 0.58, 1.0))
    red_indicator = model.material("red_lock_indicator", rgba=(0.75, 0.04, 0.03, 1.0))

    body = model.part("body")
    body.visual(_body_shell_mesh(), material=enamel, name="body_shell")
    body.visual(
        Box((BODY_DEPTH - 2.0 * WALL_THICKNESS - 0.030, BODY_WIDTH - 2.0 * WALL_THICKNESS - 0.030, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, FLOOR_THICKNESS + 0.006)),
        material=liner,
        name="inner_liner_floor",
    )

    # Four short feet and a lower compressor grille make the object read as a lab freezer,
    # while overlapping the shell slightly so the body remains one supported assembly.
    for i, x in enumerate((-0.250, 0.250)):
        for j, y in enumerate((-0.410, 0.410)):
            body.visual(
                Box((0.105, 0.090, 0.055)),
                origin=Origin(xyz=(x, y, -0.022)),
                material=dark_plastic,
                name=f"foot_{i}_{j}",
            )
    for idx, z in enumerate((0.120, 0.155, 0.190, 0.225)):
        body.visual(
            Box((0.010, 0.330, 0.013)),
            origin=Origin(xyz=(BODY_DEPTH / 2.0 + 0.004, -0.275, z)),
            material=dark_plastic,
            name=f"front_vent_slot_{idx}",
        )
    body.visual(
        Box((0.012, 0.250, 0.090)),
        origin=Origin(xyz=(BODY_DEPTH / 2.0 + 0.005, 0.240, 0.455)),
        material=blue_label,
        name="sample_label_holder",
    )
    body.visual(
        Box((0.018, 0.155, 0.075)),
        origin=Origin(xyz=(BODY_DEPTH / 2.0 + 0.006, 0.0, 0.705)),
        material=brushed_steel,
        name="latch_keeper_plate",
    )

    hinge_centers = (-0.330, 0.330)
    for h, y0 in enumerate(hinge_centers):
        for seg, yoff in enumerate((-0.052, 0.052)):
            body.visual(
                Box((0.040, 0.046, 0.090)),
                origin=Origin(xyz=(HINGE_X + 0.030, y0 + yoff, HINGE_Z - 0.110)),
                material=brushed_steel,
                name=f"hinge_leaf_{h}_{seg}",
            )
            body.visual(
                Box((0.030, 0.046, 0.065)),
                origin=Origin(xyz=(HINGE_X, y0 + yoff, HINGE_Z - 0.0325)),
                material=brushed_steel,
                name=f"hinge_saddle_{h}_{seg}",
            )
            body.visual(
                Cylinder(radius=0.018, length=0.050),
                origin=Origin(xyz=(HINGE_X, y0 + yoff, HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=brushed_steel,
                name=f"body_knuckle_{h}_{seg}",
            )

    lid = model.part("lid")
    lid.visual(_lid_shell_mesh(), material=enamel, name="lid_shell")
    lid.visual(_gasket_mesh(), material=gasket_rubber, name="seal_gasket")
    for h, y0 in enumerate(hinge_centers):
        lid.visual(
            Box((0.090, 0.050, 0.030)),
            origin=Origin(xyz=(0.040, y0, -0.010)),
            material=brushed_steel,
            name=f"lid_hinge_leaf_{h}",
        )
        lid.visual(
            Cylinder(radius=0.017, length=0.054),
            origin=Origin(xyz=(0.0, y0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brushed_steel,
            name=f"lid_knuckle_{h}",
        )

    latch = model.part("latch")
    latch.visual(
        Cylinder(radius=0.018, length=0.175),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="cam_pivot",
    )
    latch.visual(
        Box((0.034, 0.160, 0.118)),
        origin=Origin(xyz=(0.026, 0.0, -0.070)),
        material=dark_plastic,
        name="locking_handle",
    )
    latch.visual(
        Box((0.052, 0.066, 0.018)),
        origin=Origin(xyz=(0.022, 0.0, -0.132)),
        material=brushed_steel,
        name="cam_tongue",
    )
    latch.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(0.045, 0.0, -0.020), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=red_indicator,
        name="lock_indicator",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.0, lower=0.0, upper=1.30),
    )

    model.articulation(
        "lid_to_latch",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=latch,
        origin=Origin(xyz=(LID_REAR_CLEARANCE + LID_DEPTH - 0.012, 0.0, -0.015)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=0.0, upper=1.05),
    )

    body.inertial = Inertial.from_geometry(Box((BODY_DEPTH, BODY_WIDTH, BODY_HEIGHT)), mass=92.0)
    lid.inertial = Inertial.from_geometry(
        Box((LID_DEPTH, LID_WIDTH, LID_THICKNESS)),
        mass=24.0,
        origin=Origin(xyz=(LID_REAR_CLEARANCE + LID_DEPTH / 2.0, 0.0, 0.0)),
    )
    latch.inertial = Inertial.from_geometry(Box((0.065, 0.180, 0.160)), mass=0.8, origin=Origin(xyz=(0.02, 0.0, -0.06)))

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    latch = object_model.get_part("latch")
    lid_hinge = object_model.get_articulation("body_to_lid")
    cam_joint = object_model.get_articulation("lid_to_latch")

    ctx.allow_overlap(
        latch,
        lid,
        elem_a="cam_pivot",
        elem_b="lid_shell",
        reason="The cam latch pivot shaft is intentionally seated through the front edge of the heavy lid.",
    )

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="seal_gasket",
        negative_elem="body_shell",
        max_gap=0.004,
        max_penetration=0.0,
        name="pressure gasket sits just above the freezer rim",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="seal_gasket",
        elem_b="body_shell",
        min_overlap=0.50,
        name="lid seal footprint covers the insulated rim",
    )
    ctx.expect_contact(
        latch,
        lid,
        elem_a="cam_pivot",
        elem_b="lid_shell",
        contact_tol=0.020,
        name="cam latch pivot is mounted on the lid front",
    )
    ctx.expect_contact(
        lid,
        body,
        elem_a="lid_knuckle_0",
        elem_b="body_knuckle_0_0",
        contact_tol=0.007,
        name="rear hinge knuckles share the hinge pin line",
    )
    ctx.expect_contact(
        lid,
        body,
        elem_a="lid_knuckle_1",
        elem_b="body_knuckle_1_0",
        contact_tol=0.007,
        name="second rear hinge knuckle shares the hinge pin line",
    )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    closed_latch_aabb = ctx.part_element_world_aabb(latch, elem="locking_handle")
    with ctx.pose({lid_hinge: 1.10}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    with ctx.pose({cam_joint: 0.85}):
        turned_latch_aabb = ctx.part_element_world_aabb(latch, elem="locking_handle")

    ctx.check(
        "heavy lid opens upward on the rear hinges",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.35,
        details=f"closed={closed_lid_aabb}, opened={open_lid_aabb}",
    )
    ctx.check(
        "front cam latch visibly rotates",
        closed_latch_aabb is not None
        and turned_latch_aabb is not None
        and abs(turned_latch_aabb[0][2] - closed_latch_aabb[0][2]) > 0.020,
        details=f"closed={closed_latch_aabb}, turned={turned_latch_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
