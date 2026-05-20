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


CASE_LENGTH = 0.44
CASE_WIDTH = 0.24
BASE_HEIGHT = 0.12
LID_HEIGHT = 0.040
HINGE_X = -CASE_LENGTH / 2.0 - 0.006
HINGE_Z = BASE_HEIGHT + 0.006
LID_FRONT_X = CASE_LENGTH
LATCH_Y = (-0.070, 0.070)


def _base_tray_mesh():
    wall = 0.012
    bottom = 0.008
    outer = cq.Workplane("XY").box(CASE_LENGTH, CASE_WIDTH, BASE_HEIGHT).translate(
        (0.0, 0.0, BASE_HEIGHT / 2.0)
    )
    cutter = (
        cq.Workplane("XY")
        .box(CASE_LENGTH - 2.0 * wall, CASE_WIDTH - 2.0 * wall, BASE_HEIGHT)
        .translate((0.0, 0.0, bottom + BASE_HEIGHT / 2.0))
    )
    tray = outer.cut(cutter)
    return tray.edges("|Z").fillet(0.003)


def _gasket_frame_mesh():
    outer_x = CASE_LENGTH - 0.008
    outer_y = CASE_WIDTH - 0.008
    inner_x = CASE_LENGTH - 0.040
    inner_y = CASE_WIDTH - 0.040
    height = 0.003
    outer = cq.Workplane("XY").box(outer_x, outer_y, height).translate(
        (0.0, 0.0, BASE_HEIGHT + height / 2.0 - 0.001)
    )
    cutter = cq.Workplane("XY").box(inner_x, inner_y, height + 0.004).translate(
        (0.0, 0.0, BASE_HEIGHT + height / 2.0 - 0.001)
    )
    return outer.cut(cutter)


def _lid_shell_mesh():
    wall = 0.010
    top = 0.006
    start_x = 0.012
    body_length = CASE_LENGTH - start_x
    outer = cq.Workplane("XY").box(body_length, CASE_WIDTH, LID_HEIGHT).translate(
        (start_x + body_length / 2.0, 0.0, LID_HEIGHT / 2.0)
    )
    cutter_height = LID_HEIGHT - top + 0.006
    cutter_z = (LID_HEIGHT - top - 0.003) / 2.0
    cutter = (
        cq.Workplane("XY")
        .box(body_length - 2.0 * wall, CASE_WIDTH - 2.0 * wall, cutter_height)
        .translate((start_x + body_length / 2.0, 0.0, cutter_z))
    )
    shell = outer.cut(cutter)
    return shell.edges("|Z").fillet(0.0025)


def _box(part, name, size, xyz, material):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _cylinder_y(part, name, radius, length, xyz, material):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_tackle_box")

    graphite = model.material("glass_filled_graphite", rgba=(0.07, 0.085, 0.095, 1.0))
    lid_blue = model.material("satin_blue_lid", rgba=(0.11, 0.22, 0.32, 1.0))
    black = model.material("black_nitrile", rgba=(0.005, 0.006, 0.006, 1.0))
    steel = model.material("brushed_stainless", rgba=(0.70, 0.72, 0.70, 1.0))
    brass = model.material("calibration_brass", rgba=(0.86, 0.63, 0.28, 1.0))
    white = model.material("etched_white", rgba=(0.92, 0.92, 0.86, 1.0))
    orange = model.material("index_orange", rgba=(1.0, 0.36, 0.05, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_tray_mesh(), "hollow_base_tray", tolerance=0.0008),
        material=graphite,
        name="base_shell",
    )
    base.visual(
        mesh_from_cadquery(_gasket_frame_mesh(), "raised_gasket_frame", tolerance=0.0008),
        material=black,
        name="gasket_frame",
    )

    # Rear hinge leaf and alternating base knuckles put the lid on a real datum line.
    _box(
        base,
        "base_hinge_leaf",
        (0.010, CASE_WIDTH * 0.86, 0.034),
        (-CASE_LENGTH / 2.0 - 0.002, 0.0, BASE_HEIGHT - 0.017),
        steel,
    )
    for i, y in enumerate((-0.078, 0.078)):
        _cylinder_y(base, f"base_hinge_knuckle_{i}", 0.006, 0.060, (HINGE_X, y, HINGE_Z), steel)
    for i, y in enumerate((-0.126, 0.126)):
        _cylinder_y(base, f"hinge_pin_head_{i}", 0.0045, 0.012, (HINGE_X, y * 0.90, HINGE_Z), steel)

    # Front latch keepers are fixed to the base and give the latch hooks a hard stop.
    for i, y in enumerate(LATCH_Y):
        _box(
            base,
            f"keeper_block_{i}",
            (0.008, 0.052, 0.014),
            (CASE_LENGTH / 2.0 + 0.004, y, 0.074),
            steel,
        )
        _cylinder_y(
            base,
            f"keeper_pin_{i}",
            0.0035,
            0.052,
            (CASE_LENGTH / 2.0 + 0.009, y, 0.084),
            steel,
        )
        _box(
            base,
            f"base_index_mark_{i}",
            (0.0012, 0.004, 0.020),
            (CASE_LENGTH / 2.0 + 0.0002, y, 0.101),
            orange,
        )
        _box(
            base,
            f"datum_stop_{i}",
            (0.026, 0.018, 0.004),
            (0.205, y, BASE_HEIGHT + 0.002),
            brass,
        )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shell_mesh(), "hollow_lid_cap", tolerance=0.0008),
        material=lid_blue,
        name="lid_shell",
    )
    _cylinder_y(lid, "lid_hinge_knuckle", 0.006, 0.078, (0.0, 0.0, 0.0), steel)
    _box(lid, "lid_hinge_leaf", (0.026, 0.084, 0.004), (0.018, 0.0, -0.002), steel)
    _box(lid, "top_datum_plate", (0.165, 0.068, 0.002), (0.245, 0.0, LID_HEIGHT + 0.001), steel)
    _box(lid, "datum_cross_x", (0.150, 0.003, 0.0012), (0.245, 0.0, LID_HEIGHT + 0.0026), white)
    _box(lid, "datum_cross_y", (0.003, 0.060, 0.0012), (0.245, 0.0, LID_HEIGHT + 0.0026), white)

    # Tick marks straddle the front seam so the calibrated gap is visually readable.
    for i, y in enumerate(LATCH_Y):
        _box(lid, f"lid_index_mark_{i}", (0.0012, 0.004, 0.020), (LID_FRONT_X + 0.0002, y, 0.012), white)
        _box(lid, f"latch_lug_a_{i}", (0.008, 0.010, 0.012), (LID_FRONT_X + 0.004, y - 0.026, 0.025), steel)
        _box(lid, f"latch_lug_b_{i}", (0.008, 0.010, 0.012), (LID_FRONT_X + 0.004, y + 0.026, 0.025), steel)

    lid_hinge = model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.8, lower=0.0, upper=1.45),
    )

    for i, y in enumerate(LATCH_Y):
        latch = model.part(f"latch_{i}")
        _cylinder_y(latch, "latch_barrel", 0.0046, 0.030, (0.0, 0.0, 0.0), steel)
        _box(latch, "latch_plate", (0.006, 0.040, 0.058), (0.006, 0.0, -0.032), steel)
        _box(latch, "latch_hook", (0.012, 0.042, 0.006), (0.004, 0.0, -0.062), steel)
        _box(latch, "pull_lip", (0.010, 0.036, 0.008), (0.011, 0.0, -0.047), black)
        model.articulation(
            f"lid_to_latch_{i}",
            ArticulationType.REVOLUTE,
            parent=lid,
            child=latch,
            origin=Origin(xyz=(LID_FRONT_X + 0.004, y, 0.025)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=2.5, velocity=3.0, lower=0.0, upper=1.20),
        )

    for i, y in enumerate((-0.052, 0.052)):
        adjuster = model.part(f"adjuster_{i}")
        adjuster.visual(
            Cylinder(radius=0.012, length=0.008),
            origin=Origin(xyz=(0.0, 0.0, 0.004)),
            material=brass,
            name="adjuster_cap",
        )
        _box(adjuster, "adjuster_witness", (0.003, 0.019, 0.0012), (0.0, 0.0, 0.0086), white)
        model.articulation(
            f"lid_to_adjuster_{i}",
            ArticulationType.REVOLUTE,
            parent=lid,
            child=adjuster,
            origin=Origin(xyz=(0.165, y, LID_HEIGHT)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.6, velocity=2.0, lower=-math.pi, upper=math.pi),
        )

    model.meta["design_note"] = (
        "Hollow tackle box tray with raised gasket frame, datum plate, index marks, "
        "two constrained flip latches, and rotary calibration adjusters."
    )
    _ = lid_hinge
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    lid_hinge = object_model.get_articulation("base_to_lid")
    latch_0 = object_model.get_part("latch_0")
    latch_joint_0 = object_model.get_articulation("lid_to_latch_0")

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="gasket_frame",
            min_gap=0.002,
            max_gap=0.006,
            name="closed lid has controlled gasket gap",
        )
        ctx.expect_overlap(
            lid,
            base,
            axes="xy",
            elem_a="lid_shell",
            elem_b="base_shell",
            min_overlap=0.18,
            name="lid footprint registers over base tray",
        )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    with ctx.pose({lid_hinge: 1.25}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    ctx.check(
        "lid opens upward about rear hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.16,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    closed_latch_aabb = ctx.part_element_world_aabb(latch_0, elem="latch_plate")
    with ctx.pose({latch_joint_0: 1.0}):
        open_latch_aabb = ctx.part_element_world_aabb(latch_0, elem="latch_plate")
    ctx.check(
        "latch flips outward and upward",
        closed_latch_aabb is not None
        and open_latch_aabb is not None
        and open_latch_aabb[1][0] > closed_latch_aabb[1][0] + 0.015
        and open_latch_aabb[0][2] > closed_latch_aabb[0][2] + 0.010,
        details=f"closed={closed_latch_aabb}, open={open_latch_aabb}",
    )

    for i in range(2):
        ctx.check(
            f"calibration adjuster {i} is articulated",
            object_model.get_articulation(f"lid_to_adjuster_{i}") is not None,
            details="Each brass adjuster cap has its own constrained revolute joint.",
        )

    return ctx.report()


object_model = build_object_model()
