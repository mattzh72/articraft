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


CASE_W = 0.038
CASE_D = 0.015
LOWER_H = 0.052
LID_H = 0.026
WALL = 0.0016
HINGE_R = 0.0014
HINGE_X = -CASE_W / 2.0 - HINGE_R + 0.0004


def _open_lower_shell() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(CASE_W, CASE_D, LOWER_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.0022)
    )
    inner = (
        cq.Workplane("XY")
        .box(CASE_W - 2 * WALL, CASE_D - 2 * WALL, LOWER_H + 0.004, centered=(True, True, False))
        .translate((0.0, 0.0, WALL))
    )
    return outer.cut(inner)


def _hollow_lid() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(CASE_W, CASE_D, LID_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.0020)
    )
    inner = (
        cq.Workplane("XY")
        .box(CASE_W - 2 * 0.0014, CASE_D - 2 * 0.0014, LID_H, centered=(True, True, False))
        .translate((0.0, 0.0, -0.001))
    )
    return outer.cut(inner)


def _chimney() -> cq.Workplane:
    chimney_w = 0.020
    chimney_d = 0.010
    chimney_h = 0.0225
    chimney_t = 0.0010
    chimney_x = 0.007
    base_z = 0.0530

    outer = (
        cq.Workplane("XY")
        .box(chimney_w, chimney_d, chimney_h, centered=(True, True, False))
    )
    inner = (
        cq.Workplane("XY")
        .box(chimney_w - 2 * chimney_t, chimney_d - 2 * chimney_t, chimney_h + 0.003, centered=(True, True, False))
        .translate((0.0, 0.0, chimney_t))
    )
    sleeve = outer.cut(inner)

    # Pipe-style lighters vent the flame sideways.  This broad side window is
    # near the flame opening but leaves a top bridge and bottom rail in the
    # chimney insert.
    side_window = (
        cq.Workplane("XY")
        .box(0.004, 0.0072, 0.0115, centered=(True, True, True))
        .translate((chimney_w / 2.0, 0.0, 0.0147))
    )
    sleeve = sleeve.cut(side_window)

    air_hole_0 = cq.Workplane("XY").cylinder(0.004, 0.0012).rotate((0, 0, 0), (0, 1, 0), 90).translate((-0.004, -chimney_d / 2.0, 0.010))
    air_hole_1 = cq.Workplane("XY").cylinder(0.004, 0.0012).rotate((0, 0, 0), (0, 1, 0), 90).translate((0.002, -chimney_d / 2.0, 0.010))
    sleeve = sleeve.cut(air_hole_0).cut(air_hole_1)

    return sleeve.translate((chimney_x, 0.0, base_z))


def _fork_yoke() -> cq.Workplane:
    wheel_x = -0.009
    wheel_z = 0.064
    y = 0.0052
    base = cq.Workplane("XY").box(0.011, 0.0110, 0.0032, centered=(True, True, False)).translate((wheel_x, 0.0, 0.0520))
    cheek_a = cq.Workplane("XY").box(0.0062, 0.0012, 0.014, centered=(True, True, False)).translate((wheel_x, y, 0.0545))
    cheek_b = cq.Workplane("XY").box(0.0062, 0.0012, 0.014, centered=(True, True, False)).translate((wheel_x, -y, 0.0545))
    bridge = cq.Workplane("XY").box(0.0045, 0.0104, 0.0016, centered=(True, True, False)).translate((wheel_x, 0.0, wheel_z + 0.0064))
    return base.union(cheek_a).union(cheek_b).union(bridge)


def _toothed_wheel() -> cq.Workplane:
    teeth = 28
    root_r = 0.00345
    tip_r = 0.00405
    pts = []
    for i in range(teeth * 2):
        a = 2.0 * math.pi * i / (teeth * 2)
        r = tip_r if i % 2 == 0 else root_r
        pts.append((r * math.cos(a), r * math.sin(a)))
    disk = cq.Workplane("XY").polyline(pts).close().extrude(0.0056, both=True)
    bore = cq.Workplane("XY").circle(0.0011).extrude(0.0070, both=True)
    return disk.cut(bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pipe_style_lighter")

    brass = model.material("brushed_brass", rgba=(0.78, 0.56, 0.27, 1.0))
    dark = model.material("dark_lacquer", rgba=(0.035, 0.035, 0.032, 1.0))
    steel = model.material("stainless_steel", rgba=(0.70, 0.72, 0.70, 1.0))
    heat_blue = model.material("heat_tinted_steel", rgba=(0.36, 0.44, 0.56, 1.0))
    wick_mat = model.material("braided_wick", rgba=(0.86, 0.80, 0.62, 1.0))

    case_shell = model.part("case_shell")
    case_shell.visual(
        Box((CASE_W, CASE_D, WALL)),
        origin=Origin(xyz=(0.0, 0.0, WALL / 2.0)),
        material=dark,
        name="case_bottom",
    )
    case_shell.visual(
        Box((WALL, CASE_D, LOWER_H)),
        origin=Origin(xyz=(-CASE_W / 2.0 + WALL / 2.0, 0.0, LOWER_H / 2.0)),
        material=dark,
        name="hinge_side_wall",
    )
    case_shell.visual(
        Box((WALL, CASE_D, LOWER_H)),
        origin=Origin(xyz=(CASE_W / 2.0 - WALL / 2.0, 0.0, LOWER_H / 2.0)),
        material=dark,
        name="outer_side_wall",
    )
    case_shell.visual(
        Box((CASE_W, WALL, LOWER_H)),
        origin=Origin(xyz=(0.0, -CASE_D / 2.0 + WALL / 2.0, LOWER_H / 2.0)),
        material=dark,
        name="front_wall",
    )
    case_shell.visual(
        Box((CASE_W, WALL, LOWER_H)),
        origin=Origin(xyz=(0.0, CASE_D / 2.0 - WALL / 2.0, LOWER_H / 2.0)),
        material=dark,
        name="back_wall",
    )
    case_shell.visual(
        Cylinder(radius=HINGE_R, length=0.014),
        origin=Origin(xyz=(HINGE_X, 0.0, 0.010)),
        material=brass,
        name="lower_hinge_barrel",
    )
    case_shell.visual(
        Cylinder(radius=HINGE_R, length=0.014),
        origin=Origin(xyz=(HINGE_X, 0.0, 0.038)),
        material=brass,
        name="upper_hinge_barrel",
    )

    insert = model.part("insert")
    insert.visual(
        Box((0.0328, 0.0104, 0.0485)),
        origin=Origin(xyz=(0.0, 0.0, WALL + 0.0485 / 2.0)),
        material=steel,
        name="insert_body",
    )
    insert.visual(
        Box((0.0338, 0.0110, 0.0030)),
        origin=Origin(xyz=(0.0, 0.0, 0.0515)),
        material=steel,
        name="top_lip",
    )
    insert.visual(
        mesh_from_cadquery(_chimney(), "chimney_insert", tolerance=0.00025, angular_tolerance=0.08),
        material=heat_blue,
        name="chimney",
    )
    insert.visual(
        Box((0.0110, 0.0110, 0.0032)),
        origin=Origin(xyz=(-0.009, 0.0, 0.0536)),
        material=steel,
        name="yoke_base",
    )
    insert.visual(
        Box((0.0062, 0.0012, 0.0140)),
        origin=Origin(xyz=(-0.009, 0.0052, 0.0615)),
        material=steel,
        name="yoke_front_cheek",
    )
    insert.visual(
        Box((0.0062, 0.0012, 0.0140)),
        origin=Origin(xyz=(-0.009, -0.0052, 0.0615)),
        material=steel,
        name="yoke_back_cheek",
    )
    insert.visual(
        Box((0.0045, 0.0104, 0.0016)),
        origin=Origin(xyz=(-0.009, 0.0, 0.0693)),
        material=steel,
        name="yoke_bridge",
    )
    insert.visual(
        Cylinder(radius=0.00065, length=0.0124),
        origin=Origin(xyz=(-0.009, 0.0, 0.0640), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="striker_axle",
    )
    insert.visual(
        Cylinder(radius=0.00105, length=0.0125),
        origin=Origin(xyz=(0.006, 0.0, 0.0585)),
        material=wick_mat,
        name="wick",
    )

    lid = model.part("lid")
    lid.visual(
        Box((CASE_W, CASE_D, 0.0014)),
        origin=Origin(xyz=(-HINGE_X, 0.0, LID_H - 0.0007)),
        material=dark,
        name="lid_top",
    )
    lid.visual(
        Box((0.0014, CASE_D, LID_H)),
        origin=Origin(xyz=(-HINGE_X - CASE_W / 2.0 + 0.0007, 0.0, LID_H / 2.0)),
        material=dark,
        name="lid_hinge_wall",
    )
    lid.visual(
        Box((0.0014, CASE_D, LID_H)),
        origin=Origin(xyz=(-HINGE_X + CASE_W / 2.0 - 0.0007, 0.0, LID_H / 2.0)),
        material=dark,
        name="lid_outer_wall",
    )
    lid.visual(
        Box((CASE_W, 0.0014, LID_H)),
        origin=Origin(xyz=(-HINGE_X, -CASE_D / 2.0 + 0.0007, LID_H / 2.0)),
        material=dark,
        name="lid_front_wall",
    )
    lid.visual(
        Box((CASE_W, 0.0014, LID_H)),
        origin=Origin(xyz=(-HINGE_X, CASE_D / 2.0 - 0.0007, LID_H / 2.0)),
        material=dark,
        name="lid_back_wall",
    )
    lid.visual(
        Cylinder(radius=HINGE_R, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=brass,
        name="lid_hinge_barrel",
    )

    striker_wheel = model.part("striker_wheel")
    striker_wheel.visual(
        mesh_from_cadquery(_toothed_wheel(), "striker_wheel", tolerance=0.00012, angular_tolerance=0.04),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="knurled_wheel",
    )

    model.articulation(
        "case_to_insert",
        ArticulationType.PRISMATIC,
        parent=case_shell,
        child=insert,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.12, lower=0.0, upper=0.035),
    )
    model.articulation(
        "case_to_lid",
        ArticulationType.REVOLUTE,
        parent=case_shell,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, LOWER_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=4.0, lower=0.0, upper=2.05),
    )
    model.articulation(
        "insert_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=insert,
        child=striker_wheel,
        origin=Origin(xyz=(-0.009, 0.0, 0.064)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    case_shell = object_model.get_part("case_shell")
    insert = object_model.get_part("insert")
    lid = object_model.get_part("lid")
    wheel = object_model.get_part("striker_wheel")
    insert_slide = object_model.get_articulation("case_to_insert")
    lid_hinge = object_model.get_articulation("case_to_lid")
    wheel_spin = object_model.get_articulation("insert_to_wheel")

    ctx.check("insert slides along case height", insert_slide.articulation_type == ArticulationType.PRISMATIC and insert_slide.axis == (0.0, 0.0, 1.0))
    ctx.check("lid uses side hinge", lid_hinge.articulation_type == ArticulationType.REVOLUTE and lid_hinge.axis == (0.0, 0.0, 1.0))
    ctx.check("striker wheel is continuous", wheel_spin.articulation_type == ArticulationType.CONTINUOUS and wheel_spin.axis == (0.0, 1.0, 0.0))

    ctx.expect_within(
        insert,
        case_shell,
        axes="xy",
        inner_elem="insert_body",
        outer_elem="case_bottom",
        margin=0.0,
        name="insert body fits inside lower shell",
    )
    ctx.expect_overlap(
        insert,
        case_shell,
        axes="z",
        elem_a="insert_body",
        min_overlap=0.045,
        name="seated insert is deeply retained",
    )
    ctx.expect_gap(
        lid,
        case_shell,
        axis="z",
        max_gap=0.0006,
        max_penetration=0.0,
        name="closed lid sits on case seam",
    )
    ctx.expect_within(
        wheel,
        insert,
        axes="xy",
        inner_elem="knurled_wheel",
        outer_elem="yoke_base",
        margin=0.005,
        name="wheel sits between yoke cheeks",
    )

    rest_pos = ctx.part_world_position(insert)
    with ctx.pose({lid_hinge: 1.7, insert_slide: 0.035, wheel_spin: 3.2}):
        ctx.expect_within(
            insert,
            case_shell,
            axes="xy",
            inner_elem="insert_body",
            outer_elem="case_bottom",
            margin=0.0,
            name="lifted insert stays centered in shell",
        )
        ctx.expect_overlap(
            insert,
            case_shell,
            axes="z",
            elem_a="insert_body",
            min_overlap=0.012,
            name="lifted insert remains retained",
        )
        lifted_pos = ctx.part_world_position(insert)

    ctx.check(
        "insert lift exposes removable assembly",
        rest_pos is not None and lifted_pos is not None and lifted_pos[2] > rest_pos[2] + 0.030,
        details=f"rest={rest_pos}, lifted={lifted_pos}",
    )

    return ctx.report()


object_model = build_object_model()
