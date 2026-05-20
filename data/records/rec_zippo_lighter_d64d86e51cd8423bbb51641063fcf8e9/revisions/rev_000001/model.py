from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    KnobGeometry,
    KnobGrip,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


CASE_W = 0.038
CASE_D = 0.012
LOWER_H = 0.039
LID_H = 0.018
WALL = 0.0012
HINGE_X = -CASE_W / 2.0 - 0.0016
OPEN_ANGLE = 1.65


def _rot_z(x: float, y: float, angle: float) -> tuple[float, float]:
    ca = math.cos(angle)
    sa = math.sin(angle)
    return (ca * x - sa * y, sa * x + ca * y)


def _origin_from_closed(
    xyz: tuple[float, float, float],
    angle: float = OPEN_ANGLE,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> Origin:
    x, y = _rot_z(xyz[0], xyz[1], angle)
    return Origin(xyz=(x, y, xyz[2]), rpy=(rpy[0], rpy[1], rpy[2] + angle))


def _hollow_cup(width: float, depth: float, height: float, wall: float):
    outer = cq.Workplane("XY").box(width, depth, height, centered=(True, True, False))
    inner = (
        cq.Workplane("XY")
        .box(width - 2.0 * wall, depth - 2.0 * wall, height, centered=(True, True, False))
        .translate((0.0, 0.0, wall))
    )
    cup = outer.cut(inner)
    return cup.edges("|Z").chamfer(0.00055)


def _hollow_lid(width: float, depth: float, height: float, wall: float):
    outer = cq.Workplane("XY").box(width, depth, height, centered=(True, True, False))
    void = cq.Workplane("XY").box(
        width - 2.0 * wall,
        depth - 2.0 * wall,
        height - wall,
        centered=(True, True, False),
    )
    cap = outer.cut(void)
    return cap.edges("|Z").chamfer(0.00050)


def _circle_profile(cx: float, cy: float, radius: float, count: int = 14):
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * i / count),
            cy + radius * math.sin(2.0 * math.pi * i / count),
        )
        for i in range(count)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="brushed_metal_pocket_lighter")

    brushed = model.material("brushed_stainless", rgba=(0.68, 0.69, 0.66, 1.0))
    dark_brush = model.material("dark_brush_lines", rgba=(0.31, 0.32, 0.31, 1.0))
    insert_mat = model.material("polished_insert", rgba=(0.82, 0.83, 0.80, 1.0))
    wheel_mat = model.material("dull_spark_wheel", rgba=(0.42, 0.42, 0.39, 1.0))
    wick_mat = model.material("charred_wick", rgba=(0.055, 0.045, 0.035, 1.0))

    lower_shell = model.part("lower_shell")
    lower_shell.visual(
        Box((CASE_W, CASE_D, WALL)),
        origin=Origin(xyz=(0.0, 0.0, WALL / 2.0)),
        material=brushed,
        name="lower_shell_body",
    )
    lower_shell.visual(
        Box((CASE_W, WALL, LOWER_H)),
        origin=Origin(xyz=(0.0, -CASE_D / 2.0 + WALL / 2.0, LOWER_H / 2.0)),
        material=brushed,
        name="front_wall",
    )
    lower_shell.visual(
        Box((CASE_W, WALL, LOWER_H)),
        origin=Origin(xyz=(0.0, CASE_D / 2.0 - WALL / 2.0, LOWER_H / 2.0)),
        material=brushed,
        name="rear_wall",
    )
    lower_shell.visual(
        Box((WALL, CASE_D - 2.0 * WALL, LOWER_H)),
        origin=Origin(xyz=(-CASE_W / 2.0 + WALL / 2.0, 0.0, LOWER_H / 2.0)),
        material=brushed,
        name="side_wall_0",
    )
    lower_shell.visual(
        Box((WALL, CASE_D - 2.0 * WALL, LOWER_H)),
        origin=Origin(xyz=(CASE_W / 2.0 - WALL / 2.0, 0.0, LOWER_H / 2.0)),
        material=brushed,
        name="side_wall_1",
    )

    # Fine, shallow horizontal strokes make the case read as brushed metal while
    # staying crisp and rectangular at handheld scale.
    for i, z in enumerate((0.009, 0.015, 0.021, 0.027, 0.033)):
        lower_shell.visual(
            Box((CASE_W * 0.82, 0.00010, 0.00022)),
            origin=Origin(xyz=(0.0, -CASE_D / 2.0 + 0.00002, z)),
            material=dark_brush,
            name=f"front_brush_{i}",
        )
        lower_shell.visual(
            Box((CASE_W * 0.82, 0.00010, 0.00022)),
            origin=Origin(xyz=(0.0, CASE_D / 2.0 - 0.00002, z)),
            material=dark_brush,
            name=f"rear_brush_{i}",
        )

    hinge_radius = 0.00125
    lower_shell.visual(
        Cylinder(radius=0.00055, length=0.056),
        origin=Origin(xyz=(HINGE_X, 0.0, 0.0280)),
        material=dark_brush,
        name="hinge_pin",
    )
    for i, zc in enumerate((0.009, 0.030)):
        lower_shell.visual(
            Cylinder(radius=hinge_radius, length=0.012),
            origin=Origin(xyz=(HINGE_X, 0.0, zc)),
            material=brushed,
            name=f"hinge_knuckle_{i}",
        )
        lower_shell.visual(
            Box((0.0020, CASE_D * 0.70, 0.010)),
            origin=Origin(xyz=(-CASE_W / 2.0 - 0.00045, 0.0, zc)),
            material=brushed,
            name=f"hinge_leaf_{i}",
        )

    insert = model.part("insert")
    insert.visual(
        Box((0.027, 0.0082, LOWER_H + 0.004 - WALL)),
        origin=Origin(xyz=(0.0010, 0.0, WALL + (LOWER_H + 0.004 - WALL) / 2.0)),
        material=insert_mat,
        name="insert_body",
    )
    insert.visual(
        Box((0.029, 0.0090, 0.0016)),
        origin=Origin(xyz=(0.0010, 0.0, LOWER_H + 0.0008)),
        material=insert_mat,
        name="insert_lip",
    )

    chimney_w = 0.020
    chimney_d = 0.0080
    chimney_h = 0.0145
    chimney_z = LOWER_H + 0.003 + chimney_h / 2.0
    plate_t = 0.0007
    outer = [
        (-chimney_w / 2.0, -chimney_h / 2.0),
        (chimney_w / 2.0, -chimney_h / 2.0),
        (chimney_w / 2.0, chimney_h / 2.0),
        (-chimney_w / 2.0, chimney_h / 2.0),
    ]
    holes = []
    for row_z in (-0.0042, 0.0, 0.0042):
        for hx in (-0.0060, 0.0, 0.0060):
            holes.append(_circle_profile(hx, row_z, 0.00105))
    chimney_plate = mesh_from_geometry(
        ExtrudeWithHolesGeometry(outer, holes, plate_t, center=True),
        "perforated_chimney_plate",
    )
    for name, y in (("chimney_front", -chimney_d / 2.0), ("chimney_rear", chimney_d / 2.0)):
        insert.visual(
            chimney_plate,
            origin=Origin(xyz=(0.0, y, chimney_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=insert_mat,
            name=name,
        )
    insert.visual(
        Box((plate_t, chimney_d, chimney_h)),
        origin=Origin(xyz=(-chimney_w / 2.0, 0.0, chimney_z)),
        material=insert_mat,
        name="chimney_side_0",
    )
    insert.visual(
        Box((plate_t, chimney_d, chimney_h)),
        origin=Origin(xyz=(chimney_w / 2.0, 0.0, chimney_z)),
        material=insert_mat,
        name="chimney_side_1",
    )
    insert.visual(
        Cylinder(radius=0.0010, length=0.010),
        origin=Origin(xyz=(-0.0025, 0.0, LOWER_H + 0.0105)),
        material=wick_mat,
        name="wick",
    )
    insert.visual(
        Cylinder(radius=0.00075, length=0.0050),
        origin=Origin(xyz=(-0.0025, 0.0, LOWER_H + 0.0030)),
        material=insert_mat,
        name="wick_tube",
    )

    wheel_x = 0.0068
    wheel_z = LOWER_H + 0.0120
    insert.visual(
        Box((0.0042, 0.0009, 0.0052)),
        origin=Origin(xyz=(wheel_x, -0.0049, LOWER_H + 0.0034)),
        material=insert_mat,
        name="front_yoke_post",
    )
    insert.visual(
        Box((0.0042, 0.0009, 0.0052)),
        origin=Origin(xyz=(wheel_x, 0.0049, LOWER_H + 0.0034)),
        material=insert_mat,
        name="rear_yoke_post",
    )
    insert.visual(
        Box((0.0042, 0.0009, 0.0100)),
        origin=Origin(xyz=(wheel_x, -0.0049, wheel_z - 0.0010)),
        material=insert_mat,
        name="wheel_yoke_front",
    )
    insert.visual(
        Box((0.0042, 0.0009, 0.0100)),
        origin=Origin(xyz=(wheel_x, 0.0049, wheel_z - 0.0010)),
        material=insert_mat,
        name="wheel_yoke_rear",
    )

    lid = model.part("lid")
    # The lid mesh is authored from local z=0 to z=LID_H, so the closed
    # transform places its lower rim directly on the case seam.
    closed_lid_center = (-HINGE_X, 0.0, 0.0)
    lid.visual(
        mesh_from_cadquery(_hollow_lid(CASE_W, CASE_D, LID_H, WALL), "lid_cap", tolerance=0.00025),
        origin=_origin_from_closed(closed_lid_center),
        material=brushed,
        name="lid_cap",
    )
    for i, z in enumerate((0.005, 0.010, 0.015)):
        lid.visual(
            Box((CASE_W * 0.78, 0.00010, 0.00020)),
            origin=_origin_from_closed((closed_lid_center[0], -CASE_D / 2.0 + 0.00002, z)),
            material=dark_brush,
            name=f"lid_brush_{i}",
        )
    lid.visual(
        Cylinder(radius=hinge_radius, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.0090)),
        material=brushed,
        name="lid_hinge_knuckle",
    )
    lid.visual(
        Box((0.0020, CASE_D * 0.70, 0.010)),
        origin=_origin_from_closed((0.0010, 0.0, 0.0090)),
        material=brushed,
        name="lid_hinge_leaf",
    )

    spark_wheel = model.part("spark_wheel")
    wheel_mesh = mesh_from_geometry(
        KnobGeometry(
            0.0073,
            0.0044,
            body_style="cylindrical",
            grip=KnobGrip(style="ribbed", count=28, depth=0.00045, width=0.00075),
            edge_radius=0.00015,
        ),
        "knurled_spark_wheel",
    )
    spark_wheel.visual(
        wheel_mesh,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wheel_mat,
        name="wheel_disc",
    )
    spark_wheel.visual(
        Cylinder(radius=0.00065, length=0.0108),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wheel_mat,
        name="axle_pin",
    )

    model.articulation(
        "shell_to_insert",
        ArticulationType.FIXED,
        parent=lower_shell,
        child=insert,
        origin=Origin(),
    )
    model.articulation(
        "side_hinge",
        ArticulationType.REVOLUTE,
        parent=lower_shell,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, LOWER_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=5.0, lower=-OPEN_ANGLE, upper=0.35),
    )
    model.articulation(
        "wheel_axle",
        ArticulationType.CONTINUOUS,
        parent=insert,
        child=spark_wheel,
        origin=Origin(xyz=(wheel_x, 0.0, wheel_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.05, velocity=40.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_shell = object_model.get_part("lower_shell")
    insert = object_model.get_part("insert")
    lid = object_model.get_part("lid")
    spark_wheel = object_model.get_part("spark_wheel")
    hinge = object_model.get_articulation("side_hinge")
    wheel_axle = object_model.get_articulation("wheel_axle")

    ctx.allow_overlap(
        "insert",
        "spark_wheel",
        elem_a="wheel_yoke_front",
        elem_b="axle_pin",
        reason="The short spark-wheel axle is intentionally captured in the fixed fork cheek.",
    )
    ctx.allow_overlap(
        "insert",
        "spark_wheel",
        elem_a="wheel_yoke_rear",
        elem_b="axle_pin",
        reason="The short spark-wheel axle is intentionally captured in the fixed fork cheek.",
    )
    ctx.allow_overlap(
        "lower_shell",
        "lid",
        elem_a="hinge_pin",
        elem_b="lid_hinge_knuckle",
        reason="The fixed hinge pin intentionally passes through the lid's rotating hinge barrel.",
    )

    ctx.expect_contact(
        lower_shell,
        insert,
        elem_a="lower_shell_body",
        elem_b="insert_body",
        contact_tol=0.0008,
        name="insert seats on the hollow lower shell floor",
    )
    ctx.expect_overlap(
        insert,
        spark_wheel,
        axes="y",
        elem_a="wheel_yoke_front",
        elem_b="axle_pin",
        min_overlap=0.00025,
        name="front fork captures the axle end",
    )
    ctx.expect_overlap(
        insert,
        spark_wheel,
        axes="y",
        elem_a="wheel_yoke_rear",
        elem_b="axle_pin",
        min_overlap=0.00025,
        name="rear fork captures the axle end",
    )
    ctx.expect_overlap(
        lower_shell,
        lid,
        axes="z",
        elem_a="hinge_pin",
        elem_b="lid_hinge_knuckle",
        min_overlap=0.008,
        name="side hinge pin passes through the lid barrel",
    )

    with ctx.pose({hinge: -OPEN_ANGLE}):
        ctx.expect_gap(
            lid,
            lower_shell,
            axis="z",
            positive_elem="lid_cap",
            negative_elem="front_wall",
            max_gap=0.0010,
            max_penetration=0.0002,
            name="closed lid fits at the case seam",
        )
        ctx.expect_within(
            insert,
            lid,
            axes="xy",
            inner_elem="chimney_front",
            outer_elem="lid_cap",
            margin=0.0030,
            name="closed lid surrounds the chimney insert",
        )

    closed_aabb = None
    with ctx.pose({hinge: -OPEN_ANGLE}):
        closed_aabb = ctx.part_element_world_aabb(lid, elem="lid_cap")
    open_aabb = ctx.part_element_world_aabb(lid, elem="lid_cap")
    closed_y = None if closed_aabb is None else (closed_aabb[0][1] + closed_aabb[1][1]) / 2.0
    open_y = None if open_aabb is None else (open_aabb[0][1] + open_aabb[1][1]) / 2.0
    ctx.check(
        "side hinge swings the lid off the insert",
        closed_y is not None and open_y is not None and open_y > closed_y + 0.010,
        details=f"closed_y={closed_y}, open_y={open_y}",
    )

    before = ctx.part_world_position(spark_wheel)
    with ctx.pose({wheel_axle: math.pi / 2.0}):
        after = ctx.part_world_position(spark_wheel)
        ctx.expect_within(
            spark_wheel,
            insert,
            axes="xz",
            inner_elem="axle_pin",
            outer_elem="wheel_yoke_front",
            margin=0.0040,
            name="spark wheel remains on its short axle while spinning",
        )
    ctx.check(
        "spark wheel rotates without translating",
        before is not None
        and after is not None
        and all(abs(before[i] - after[i]) < 1e-6 for i in range(3)),
        details=f"before={before}, after={after}",
    )

    return ctx.report()


object_model = build_object_model()
