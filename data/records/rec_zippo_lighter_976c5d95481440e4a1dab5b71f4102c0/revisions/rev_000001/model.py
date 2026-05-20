from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


CASE_W = 0.038
CASE_D = 0.014
CASE_H = 0.040
WALL = 0.0012
LID_H = 0.019
HINGE_R = 0.00165
HINGE_X = CASE_W / 2.0 + HINGE_R + 0.00045


def _case_shell() -> cq.Workplane:
    """Open-topped lower shell with an attached side hinge leaf and knuckles."""
    w, d, h, t = CASE_W, CASE_D, CASE_H, WALL
    shell = (
        cq.Workplane("XY")
        .box(w, d, t)
        .translate((0.0, 0.0, t / 2.0))
        .union(cq.Workplane("XY").box(w, t, h).translate((0.0, -d / 2.0 + t / 2.0, h / 2.0)))
        .union(cq.Workplane("XY").box(w, t, h).translate((0.0, d / 2.0 - t / 2.0, h / 2.0)))
        .union(cq.Workplane("XY").box(t, d - 2.0 * t, h).translate((-w / 2.0 + t / 2.0, 0.0, h / 2.0)))
        .union(cq.Workplane("XY").box(t, d - 2.0 * t, h).translate((w / 2.0 - t / 2.0, 0.0, h / 2.0)))
    )

    # The hinge barrels are outside the right side of the case.  Short leaves
    # connect each barrel to the shell so they do not read as decorative floats.
    leaf = cq.Workplane("XY").box(0.0032, 0.0040, 0.034).translate((CASE_W / 2.0 + 0.0005, 0.0, 0.021))
    shell = shell.union(leaf)
    for zc, length in ((0.010, 0.013), (0.030, 0.013)):
        barrel = cq.Workplane("XY").circle(HINGE_R).extrude(length).translate((HINGE_X, 0.0, zc - length / 2.0))
        shell = shell.union(barrel)

    return shell


def _lid_shell() -> cq.Workplane:
    """Hollow cap, authored in the lid frame whose origin is the hinge line."""
    w, d, h, t = CASE_W, CASE_D, LID_H, WALL
    x_center = -HINGE_X
    bottom = 0.00035

    cap = (
        cq.Workplane("XY")
        .box(w, d, t)
        .translate((x_center, 0.0, h - t / 2.0))
        .union(cq.Workplane("XY").box(w, t, h - bottom).translate((x_center, -d / 2.0 + t / 2.0, bottom + (h - bottom) / 2.0)))
        .union(cq.Workplane("XY").box(w, t, h - bottom).translate((x_center, d / 2.0 - t / 2.0, bottom + (h - bottom) / 2.0)))
        .union(cq.Workplane("XY").box(t, d - 2.0 * t, h - bottom).translate((x_center - w / 2.0 + t / 2.0, 0.0, bottom + (h - bottom) / 2.0)))
        .union(cq.Workplane("XY").box(t, d - 2.0 * t, h - bottom).translate((x_center + w / 2.0 - t / 2.0, 0.0, bottom + (h - bottom) / 2.0)))
    )

    # Lid-side hinge barrel and connecting leaf.
    lid_barrel = cq.Workplane("XY").circle(HINGE_R).extrude(0.0145).translate((0.0, 0.0, 0.0028))
    lid_leaf = cq.Workplane("XY").box(0.0034, 0.0040, 0.0145).translate((-0.00185, 0.0, 0.01005))
    cap = cap.union(lid_barrel).union(lid_leaf)

    # Two small cheeks inside the lid support the separate cam lever pivot.
    for y in (-0.00225, 0.00225):
        cheek = cq.Workplane("XY").box(0.0040, 0.00055, 0.0048).translate((-0.0060, y, 0.0076))
        cap = cap.union(cheek)

    return cap


def _chimney_shell() -> cq.Workplane:
    """Separate insert: lower can plus open chimney with pipe-style side cutout."""
    lower_w = 0.030
    lower_d = 0.0092
    lower_h = 0.0368
    lower = cq.Workplane("XY").box(lower_w, lower_d, lower_h).translate((0.0, 0.0, WALL + lower_h / 2.0))

    chim_w = 0.020
    chim_d = 0.0094
    chim_t = 0.00075
    chim_bottom = 0.0380
    chim_h = 0.0185
    zc = chim_bottom + chim_h / 2.0
    front = cq.Workplane("XY").box(chim_w, chim_t, chim_h).translate((0.0, -chim_d / 2.0 + chim_t / 2.0, zc))
    back = cq.Workplane("XY").box(chim_w, chim_t, chim_h).translate((0.0, chim_d / 2.0 - chim_t / 2.0, zc))
    side_l = cq.Workplane("XY").box(chim_t, chim_d - 2 * chim_t, chim_h).translate((-chim_w / 2.0 + chim_t / 2.0, 0.0, zc))
    side_r = cq.Workplane("XY").box(chim_t, chim_d - 2 * chim_t, chim_h).translate((chim_w / 2.0 - chim_t / 2.0, 0.0, zc))
    top_rim = cq.Workplane("XY").box(chim_w, chim_d, chim_t).translate((0.0, 0.0, chim_bottom + chim_h - chim_t / 2.0))
    inner_void = cq.Workplane("XY").box(chim_w - 2 * chim_t, chim_d - 2 * chim_t, chim_t * 1.8).translate(
        (0.0, 0.0, chim_bottom + chim_h - chim_t / 2.0)
    )
    chimney = front.union(back).union(side_l).union(side_r).union(top_rim.cut(inner_void))

    # Pipe-lighter side cutout through the left chimney wall near the flame.
    side_hole = (
        cq.Workplane("YZ")
        .circle(0.0039)
        .extrude(0.0040)
        .translate((-chim_w / 2.0 - 0.0020, 0.0, chim_bottom + 0.0125))
    )
    chimney = chimney.cut(side_hole)

    # A small wick tube rises from the insert body and is bridged by the body top.
    wick_tube = cq.Workplane("XY").circle(0.00135).extrude(0.011).translate((0.0, 0.0, chim_bottom))
    return lower.union(chimney).union(wick_tube)


def _striker_wheel() -> cq.Workplane:
    """Tiny serrated flint wheel, with its axis along local X."""
    teeth = 34
    base_r = 0.00265
    tooth_r = 0.00305
    pts = []
    for i in range(teeth * 2):
        angle = 2.0 * math.pi * i / (teeth * 2)
        r = tooth_r if i % 2 == 0 else base_r
        pts.append((r * math.cos(angle), r * math.sin(angle)))
    return cq.Workplane("YZ").polyline(pts).close().extrude(0.0042).translate((-0.0021, 0.0, 0.0))


def _cam_lever() -> cq.Workplane:
    """Small cam lever in its own pivot frame; pivot axis is local Y."""
    collar = cq.Workplane("YZ").circle(0.0010).extrude(0.0022).translate((-0.0011, 0.0, 0.0))
    arm = cq.Workplane("XY").box(0.0080, 0.00145, 0.00115).translate((-0.0040, 0.0, 0.0018))
    cam_tip = cq.Workplane("YZ").circle(0.0010).extrude(0.0012).translate((-0.0086, 0.0, 0.0018))
    return collar.union(arm).union(cam_tip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pipe_style_lighter")

    chrome = model.material("brushed_chrome", rgba=(0.70, 0.68, 0.62, 1.0))
    dark_steel = model.material("dark_blued_steel", rgba=(0.08, 0.085, 0.09, 1.0))
    insert_steel = model.material("heat_stained_chimney", rgba=(0.50, 0.52, 0.50, 1.0))
    brass = model.material("spring_brass", rgba=(0.83, 0.61, 0.25, 1.0))
    wick_mat = model.material("charred_wick", rgba=(0.08, 0.055, 0.035, 1.0))

    case = model.part("case")
    case.visual(Box((CASE_W, CASE_D, WALL)), origin=Origin(xyz=(0.0, 0.0, WALL / 2.0)), material=chrome, name="case_base")
    case.visual(
        Box((CASE_W, WALL, CASE_H)),
        origin=Origin(xyz=(0.0, -CASE_D / 2.0 + WALL / 2.0, CASE_H / 2.0)),
        material=chrome,
        name="front_wall",
    )
    case.visual(
        Box((CASE_W, WALL, CASE_H)),
        origin=Origin(xyz=(0.0, CASE_D / 2.0 - WALL / 2.0, CASE_H / 2.0)),
        material=chrome,
        name="rear_wall",
    )
    case.visual(
        Box((WALL, CASE_D - 2.0 * WALL, CASE_H)),
        origin=Origin(xyz=(-CASE_W / 2.0 + WALL / 2.0, 0.0, CASE_H / 2.0)),
        material=chrome,
        name="side_wall_0",
    )
    case.visual(
        Box((WALL, CASE_D - 2.0 * WALL, CASE_H)),
        origin=Origin(xyz=(CASE_W / 2.0 - WALL / 2.0, 0.0, CASE_H / 2.0)),
        material=chrome,
        name="side_wall_1",
    )
    case.visual(
        Box((0.0032, 0.0040, 0.034)),
        origin=Origin(xyz=(CASE_W / 2.0 + 0.0005, 0.0, 0.021)),
        material=chrome,
        name="hinge_leaf",
    )
    for idx, (zc, length) in enumerate(((0.010, 0.013), (0.030, 0.013))):
        case.visual(
            Cylinder(radius=HINGE_R, length=length),
            origin=Origin(xyz=(HINGE_X, 0.0, zc)),
            material=chrome,
            name=f"case_hinge_barrel_{idx}",
        )
    case.visual(
        Cylinder(radius=0.00055, length=0.055),
        origin=Origin(xyz=(HINGE_X, 0.0, 0.0315)),
        material=dark_steel,
        name="hinge_pin",
    )

    chimney = model.part("chimney_insert")
    chimney.visual(
        mesh_from_cadquery(_chimney_shell(), "chimney_shell", tolerance=0.00018),
        material=insert_steel,
        name="chimney_shell",
    )
    chimney.visual(
        Cylinder(radius=0.00085, length=0.0065),
        origin=Origin(xyz=(0.0, 0.0, 0.0520)),
        material=wick_mat,
        name="wick",
    )
    chimney.visual(
        Box((0.0068, 0.0010, 0.0012)),
        origin=Origin(xyz=(0.0062, -0.00215, 0.0472)),
        material=insert_steel,
        name="striker_bridge",
    )
    for idx, x in enumerate((0.0028, 0.0096)):
        chimney.visual(
            Box((0.0008, 0.0015, 0.0075)),
            origin=Origin(xyz=(x, -0.00215, 0.0506)),
            material=insert_steel,
            name=f"striker_fork_{idx}",
        )
    chimney.visual(
        Cylinder(radius=0.00062, length=0.0088),
        origin=Origin(xyz=(0.0062, -0.00215, 0.0520), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="striker_axle",
    )

    lid = model.part("lid")
    lid_bottom = 0.00035
    lid_x = -HINGE_X
    lid.visual(Box((CASE_W, CASE_D, WALL)), origin=Origin(xyz=(lid_x, 0.0, LID_H - WALL / 2.0)), material=chrome, name="lid_top")
    lid.visual(
        Box((CASE_W, WALL, LID_H - lid_bottom)),
        origin=Origin(xyz=(lid_x, -CASE_D / 2.0 + WALL / 2.0, lid_bottom + (LID_H - lid_bottom) / 2.0)),
        material=chrome,
        name="lid_front_wall",
    )
    lid.visual(
        Box((CASE_W, WALL, LID_H - lid_bottom)),
        origin=Origin(xyz=(lid_x, CASE_D / 2.0 - WALL / 2.0, lid_bottom + (LID_H - lid_bottom) / 2.0)),
        material=chrome,
        name="lid_rear_wall",
    )
    lid.visual(
        Box((WALL, CASE_D - 2.0 * WALL, LID_H - lid_bottom)),
        origin=Origin(xyz=(lid_x - CASE_W / 2.0 + WALL / 2.0, 0.0, lid_bottom + (LID_H - lid_bottom) / 2.0)),
        material=chrome,
        name="lid_side_wall_0",
    )
    lid.visual(
        Box((WALL, CASE_D - 2.0 * WALL, LID_H - lid_bottom)),
        origin=Origin(xyz=(lid_x + CASE_W / 2.0 - WALL / 2.0, 0.0, lid_bottom + (LID_H - lid_bottom) / 2.0)),
        material=chrome,
        name="lid_side_wall_1",
    )
    lid.visual(
        Box((0.0034, 0.0040, 0.0145)),
        origin=Origin(xyz=(-0.00185, 0.0, 0.01005)),
        material=chrome,
        name="lid_hinge_leaf",
    )
    lid.visual(
        Cylinder(radius=HINGE_R, length=0.0145),
        origin=Origin(xyz=(0.0, 0.0, 0.01005)),
        material=chrome,
        name="lid_hinge_barrel",
    )
    for idx, y in enumerate((-0.00225, 0.00225)):
        lid.visual(
            Box((0.0040, 0.00055, 0.0048)),
            origin=Origin(xyz=(-0.0060, y, 0.0076)),
            material=chrome,
            name=f"cam_cheek_{idx}",
        )
        lid.visual(
            Box((0.0010, 0.00055, 0.0048)),
            origin=Origin(xyz=(-0.00365, y, 0.0076)),
            material=chrome,
            name=f"cam_cheek_bridge_{idx}",
        )

    striker = model.part("striker_wheel")
    striker.visual(mesh_from_cadquery(_striker_wheel(), "striker_wheel", tolerance=0.00008), material=dark_steel, name="serrated_wheel")

    cam = model.part("cam_lever")
    cam.visual(
        Cylinder(radius=0.0010, length=0.0022),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="cam_collar",
    )
    cam.visual(Box((0.0072, 0.00145, 0.0012)), origin=Origin(xyz=(-0.0038, 0.0, 0.0005)), material=brass, name="cam_arm")
    cam.visual(
        Cylinder(radius=0.0010, length=0.0014),
        origin=Origin(xyz=(-0.0077, 0.0, 0.0005), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="cam_tip",
    )

    model.articulation(
        "case_to_chimney",
        ArticulationType.FIXED,
        parent=case,
        child=chimney,
        origin=Origin(),
    )
    model.articulation(
        "case_to_lid",
        ArticulationType.REVOLUTE,
        parent=case,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, CASE_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=7.0, lower=0.0, upper=1.95),
    )
    model.articulation(
        "chimney_to_striker",
        ArticulationType.CONTINUOUS,
        parent=chimney,
        child=striker,
        origin=Origin(xyz=(0.0062, -0.00215, 0.0520)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.08, velocity=30.0),
    )
    model.articulation(
        "lid_to_cam_lever",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=cam,
        origin=Origin(xyz=(-0.0060, 0.0, 0.0076)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.08, velocity=8.0, lower=-0.05, upper=0.9),
        mimic=Mimic(joint="case_to_lid", multiplier=0.42, offset=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    case = object_model.get_part("case")
    chimney = object_model.get_part("chimney_insert")
    lid = object_model.get_part("lid")
    striker = object_model.get_part("striker_wheel")
    cam = object_model.get_part("cam_lever")
    lid_hinge = object_model.get_articulation("case_to_lid")
    striker_spin = object_model.get_articulation("chimney_to_striker")
    cam_pivot = object_model.get_articulation("lid_to_cam_lever")

    ctx.check(
        "pocket lighter scale",
        (CASE_W > 0.030 and CASE_W < 0.050 and CASE_H + LID_H > 0.052 and CASE_H + LID_H < 0.070),
        details=f"case_width={CASE_W}, overall_height={CASE_H + LID_H}",
    )
    ctx.expect_within(
        chimney,
        case,
        axes="xy",
        margin=0.0,
        name="separate chimney insert sits inside case footprint",
    )
    ctx.expect_contact(
        chimney,
        case,
        elem_a="chimney_shell",
        elem_b="case_base",
        contact_tol=0.0008,
        name="insert is seated on the lower case shell",
    )
    ctx.expect_within(
        cam,
        lid,
        axes="xyz",
        margin=0.001,
        name="cam lever stays inside the lid edge support",
    )
    ctx.check(
        "striker wheel has continuous joint",
        striker_spin.articulation_type == ArticulationType.CONTINUOUS and tuple(striker_spin.axis) == (1.0, 0.0, 0.0),
        details=f"type={striker_spin.articulation_type}, axis={striker_spin.axis}",
    )
    ctx.check(
        "cam lever follows lid opening",
        cam_pivot.mimic is not None and cam_pivot.mimic.joint == "case_to_lid" and cam_pivot.mimic.multiplier > 0.0,
        details=f"mimic={cam_pivot.mimic}",
    )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    closed_cam_aabb = ctx.part_world_aabb(cam)
    with ctx.pose({lid_hinge: 1.45, striker_spin: 2.0}):
        open_lid_aabb = ctx.part_world_aabb(lid)
        open_cam_aabb = ctx.part_world_aabb(cam)

    ctx.check(
        "side hinge swings lid outward",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[0][1] < closed_lid_aabb[0][1] - 0.010,
        details=f"closed={closed_lid_aabb}, opened={open_lid_aabb}",
    )
    ctx.check(
        "cam lever changes pose with lid",
        closed_cam_aabb is not None
        and open_cam_aabb is not None
        and abs(open_cam_aabb[1][2] - closed_cam_aabb[1][2]) > 0.001,
        details=f"closed={closed_cam_aabb}, opened={open_cam_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
