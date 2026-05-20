from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    KnobGeometry,
    KnobGrip,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


CASE_W = 0.038
CASE_D = 0.013
LOWER_H = 0.036
LID_H = 0.021
WALL_T = 0.0014
FLOOR_T = 0.0020
TOP_T = 0.0018
CORNER_R = 0.0034
SEAM_GAP = 0.00045
HINGE_R = 0.0022
HINGE_X = CASE_W / 2.0 + HINGE_R


def _rounded_shell_side(width: float, depth: float, height: float, wall: float, radius: float):
    """A rounded rectangular tube extruded upward along local Z."""
    outer = rounded_rect_profile(width, depth, radius, corner_segments=8)
    inner = rounded_rect_profile(
        width - 2.0 * wall,
        depth - 2.0 * wall,
        max(0.0005, radius - wall),
        corner_segments=8,
    )
    return ExtrudeWithHolesGeometry(outer, [inner], height, center=True)


def _rounded_plate(width: float, depth: float, height: float, radius: float):
    return ExtrudeGeometry(
        rounded_rect_profile(width, depth, radius, corner_segments=8),
        height,
        center=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="brushed_zippo_lighter")

    brushed = model.material("brushed_stainless", rgba=(0.70, 0.71, 0.68, 1.0))
    brushed_dark = model.material("dark_brush_grain", rgba=(0.46, 0.47, 0.45, 1.0))
    insert_metal = model.material("insert_satin_steel", rgba=(0.78, 0.78, 0.73, 1.0))
    shadow = model.material("blackened_holes", rgba=(0.015, 0.014, 0.012, 1.0))
    wheel_steel = model.material("knurled_spark_steel", rgba=(0.30, 0.31, 0.30, 1.0))
    wick_mat = model.material("charred_cotton_wick", rgba=(0.10, 0.08, 0.055, 1.0))

    lower_shell = model.part("lower_shell")
    lower_wall_h = LOWER_H - FLOOR_T
    wall_z = FLOOR_T + lower_wall_h / 2.0 - 0.00005
    lower_shell.visual(
        Box((CASE_W, WALL_T, lower_wall_h)),
        origin=Origin(xyz=(0.0, -(CASE_D / 2.0 - WALL_T / 2.0), wall_z)),
        material=brushed,
        name="front_wall",
    )
    lower_shell.visual(
        Box((CASE_W, WALL_T, lower_wall_h)),
        origin=Origin(xyz=(0.0, CASE_D / 2.0 - WALL_T / 2.0, wall_z)),
        material=brushed,
        name="back_wall",
    )
    lower_shell.visual(
        Box((WALL_T, CASE_D, lower_wall_h)),
        origin=Origin(xyz=(-(CASE_W / 2.0 - WALL_T / 2.0), 0.0, wall_z)),
        material=brushed,
        name="free_side_wall",
    )
    lower_shell.visual(
        Box((WALL_T, CASE_D, lower_wall_h)),
        origin=Origin(xyz=(CASE_W / 2.0 - WALL_T / 2.0, 0.0, wall_z)),
        material=brushed,
        name="hinge_side_wall",
    )
    floor = _rounded_plate(CASE_W, CASE_D, FLOOR_T, CORNER_R)
    floor.translate(0.0, 0.0, FLOOR_T / 2.0)
    lower_shell.visual(
        mesh_from_geometry(floor, "floor_plate"),
        material=brushed,
        name="floor_plate",
    )
    lower_shell.visual(
        Cylinder(radius=HINGE_R, length=LOWER_H - 0.005),
        origin=Origin(xyz=(HINGE_X, 0.0, LOWER_H / 2.0)),
        material=brushed,
        name="lower_hinge_barrel",
    )
    lower_shell.visual(
        Box((0.0042, CASE_D, LOWER_H - 0.006)),
        origin=Origin(xyz=(CASE_W / 2.0 + 0.00055, 0.0, LOWER_H / 2.0)),
        material=brushed,
        name="lower_hinge_leaf",
    )
    lower_shell.visual(
        Box((CASE_W - 0.008, 0.00018, LOWER_H - 0.008)),
        origin=Origin(xyz=(0.0, -CASE_D / 2.0 - 0.00003, LOWER_H / 2.0)),
        material=brushed_dark,
        name="front_brush_sheen",
    )

    insert = model.part("insert")
    insert_body_h = 0.035
    insert.visual(
        Box((0.0300, 0.0086, insert_body_h)),
        origin=Origin(xyz=(0.0, 0.0, insert_body_h / 2.0)),
        material=insert_metal,
        name="insert_body",
    )
    insert.visual(
        Box((0.026, 0.0008, 0.0022)),
        origin=Origin(xyz=(-0.0015, -0.0047, insert_body_h + 0.0006)),
        material=insert_metal,
        name="front_lip",
    )
    insert.visual(
        Box((0.026, 0.0008, 0.0022)),
        origin=Origin(xyz=(-0.0015, 0.0047, insert_body_h + 0.0006)),
        material=insert_metal,
        name="back_lip",
    )

    chimney_bottom = 0.032
    chimney_h = 0.021
    chimney_z = chimney_bottom + chimney_h / 2.0
    insert.visual(
        Box((0.021, 0.00085, chimney_h)),
        origin=Origin(xyz=(-0.0030, -0.0047, chimney_z)),
        material=insert_metal,
        name="chimney_front",
    )
    insert.visual(
        Box((0.021, 0.00085, chimney_h)),
        origin=Origin(xyz=(-0.0030, 0.0047, chimney_z)),
        material=insert_metal,
        name="chimney_back",
    )
    insert.visual(
        Box((0.0009, 0.0094, 0.017)),
        origin=Origin(xyz=(-0.0140, 0.0, chimney_bottom + 0.0085)),
        material=insert_metal,
        name="chimney_side",
    )
    insert.visual(
        Cylinder(radius=0.0012, length=0.010),
        origin=Origin(xyz=(-0.0045, 0.0, insert_body_h + 0.005)),
        material=wick_mat,
        name="wick",
    )

    hole_xs = (-0.009, -0.004, 0.0015)
    hole_zs = (0.038, 0.0435, 0.049)
    hole_index = 0
    for z in hole_zs:
        for x in hole_xs:
            insert.visual(
                Box((0.0020, 0.00018, 0.00145)),
                origin=Origin(xyz=(x, -0.00516, z)),
                material=shadow,
                name=f"front_chimney_hole_{hole_index}",
            )
            insert.visual(
                Box((0.0020, 0.00018, 0.00145)),
                origin=Origin(xyz=(x, 0.00516, z)),
                material=shadow,
                name=f"back_chimney_hole_{hole_index}",
            )
            hole_index += 1

    wheel_center = (0.0095, 0.0, 0.0440)
    for y, suffix in ((-0.0042, "front"), (0.0042, "back")):
        insert.visual(
            Box((0.0040, 0.0008, 0.0130)),
            origin=Origin(xyz=(wheel_center[0], y, 0.0415)),
            material=insert_metal,
            name=f"spark_fork_{suffix}",
        )

    cam_pivot = (0.0144, 0.0, 0.0395)
    for y, suffix in ((-0.0041, "front"), (0.0041, "back")):
        insert.visual(
            Box((0.0021, 0.0008, 0.0090)),
            origin=Origin(xyz=(cam_pivot[0], y, cam_pivot[2] - 0.0002)),
            material=insert_metal,
            name=f"cam_bracket_{suffix}",
        )

    lid = model.part("lid")
    lid_case_x = -HINGE_X
    lid_side_h = LID_H - TOP_T
    lid_wall_z = SEAM_GAP + lid_side_h / 2.0
    lid.visual(
        Box((CASE_W, WALL_T, lid_side_h)),
        origin=Origin(xyz=(lid_case_x, -(CASE_D / 2.0 - WALL_T / 2.0), lid_wall_z)),
        material=brushed,
        name="lid_front_wall",
    )
    lid.visual(
        Box((CASE_W, WALL_T, lid_side_h)),
        origin=Origin(xyz=(lid_case_x, CASE_D / 2.0 - WALL_T / 2.0, lid_wall_z)),
        material=brushed,
        name="lid_back_wall",
    )
    lid.visual(
        Box((WALL_T, CASE_D, lid_side_h)),
        origin=Origin(xyz=(lid_case_x - (CASE_W / 2.0 - WALL_T / 2.0), 0.0, lid_wall_z)),
        material=brushed,
        name="lid_free_wall",
    )
    lid.visual(
        Box((WALL_T, CASE_D, lid_side_h)),
        origin=Origin(xyz=(lid_case_x + (CASE_W / 2.0 - WALL_T / 2.0), 0.0, lid_wall_z)),
        material=brushed,
        name="lid_hinge_wall",
    )
    lid_top = _rounded_plate(CASE_W, CASE_D, TOP_T, CORNER_R)
    lid_top.translate(lid_case_x, 0.0, SEAM_GAP + lid_side_h + TOP_T / 2.0)
    lid.visual(
        mesh_from_geometry(lid_top, "lid_top"),
        material=brushed,
        name="lid_top",
    )
    lid.visual(
        Cylinder(radius=HINGE_R, length=LID_H - 0.003),
        origin=Origin(xyz=(0.0, 0.0, SEAM_GAP + LID_H / 2.0)),
        material=brushed,
        name="lid_hinge_barrel",
    )
    lid.visual(
        Box((0.0042, CASE_D, LID_H - 0.005)),
        origin=Origin(xyz=(-0.00055, 0.0, SEAM_GAP + LID_H / 2.0)),
        material=brushed,
        name="lid_hinge_leaf",
    )
    lid.visual(
        Box((CASE_W - 0.008, 0.00018, LID_H - 0.007)),
        origin=Origin(xyz=(lid_case_x, -CASE_D / 2.0 - 0.00003, SEAM_GAP + LID_H / 2.0)),
        material=brushed_dark,
        name="lid_front_sheen",
    )

    spark_wheel = model.part("spark_wheel")
    spark_wheel.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.0072,
                0.0076,
                body_style="cylindrical",
                grip=KnobGrip(style="knurled", count=28, depth=0.00035, helix_angle_deg=25.0),
            ),
            "spark_wheel_knurl",
        ),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=wheel_steel,
        name="knurled_wheel",
    )

    cam_lever = model.part("cam_lever")
    cam_lever.visual(
        Cylinder(radius=0.00125, length=0.0074),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=wheel_steel,
        name="cam_pivot_sleeve",
    )
    cam_lever.visual(
        Box((0.0018, 0.0010, 0.0100)),
        origin=Origin(xyz=(-0.0004, 0.0, 0.0050)),
        material=wheel_steel,
        name="cam_arm",
    )

    model.articulation(
        "shell_to_insert",
        ArticulationType.FIXED,
        parent=lower_shell,
        child=insert,
        origin=Origin(xyz=(0.0, 0.0, FLOOR_T)),
    )
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=lower_shell,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, LOWER_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=5.0, lower=0.0, upper=1.95),
    )
    model.articulation(
        "spark_wheel_axis",
        ArticulationType.CONTINUOUS,
        parent=insert,
        child=spark_wheel,
        origin=Origin(xyz=wheel_center),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.05, velocity=40.0),
    )
    model.articulation(
        "cam_pivot",
        ArticulationType.REVOLUTE,
        parent=insert,
        child=cam_lever,
        origin=Origin(xyz=cam_pivot),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.15, velocity=8.0, lower=0.0, upper=0.95),
        mimic=Mimic(joint="lid_hinge", multiplier=0.45, offset=0.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_shell = object_model.get_part("lower_shell")
    insert = object_model.get_part("insert")
    lid = object_model.get_part("lid")
    spark_wheel = object_model.get_part("spark_wheel")
    cam_lever = object_model.get_part("cam_lever")
    lid_hinge = object_model.get_articulation("lid_hinge")
    wheel_axis = object_model.get_articulation("spark_wheel_axis")

    ctx.expect_gap(
        insert,
        lower_shell,
        axis="z",
        positive_elem="insert_body",
        negative_elem="floor_plate",
        max_gap=0.0007,
        max_penetration=0.0,
        name="insert body seats on the lower case floor",
    )
    ctx.expect_within(
        insert,
        lower_shell,
        axes="xy",
        inner_elem="insert_body",
        margin=0.0,
        name="separate insert remains inside the outer shell footprint",
    )
    ctx.expect_gap(
        lid,
        lower_shell,
        axis="z",
        positive_elem="lid_front_wall",
        negative_elem="front_wall",
        min_gap=0.0001,
        max_gap=0.0010,
        name="closed lid has a narrow visible case seam",
    )
    ctx.expect_contact(
        spark_wheel,
        insert,
        elem_a="knurled_wheel",
        elem_b="spark_fork_front",
        contact_tol=0.0003,
        name="spark wheel bears against the front fork cheek",
    )
    ctx.expect_contact(
        cam_lever,
        insert,
        elem_a="cam_pivot_sleeve",
        elem_b="cam_bracket_front",
        contact_tol=0.0003,
        name="cam lever sleeve is captured by its pivot bracket",
    )
    ctx.check(
        "spark wheel joint is continuous",
        wheel_axis.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint_type={wheel_axis.articulation_type}",
    )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_top")
    closed_cam_aabb = ctx.part_element_world_aabb(cam_lever, elem="cam_arm")
    with ctx.pose({lid_hinge: 1.75}):
        opened_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_top")
        opened_cam_aabb = ctx.part_element_world_aabb(cam_lever, elem="cam_arm")

    closed_lid_y = (
        (closed_lid_aabb[0][1] + closed_lid_aabb[1][1]) / 2.0
        if closed_lid_aabb is not None
        else None
    )
    opened_lid_y = (
        (opened_lid_aabb[0][1] + opened_lid_aabb[1][1]) / 2.0
        if opened_lid_aabb is not None
        else None
    )
    ctx.check(
        "lid swings outward on the narrow side hinge",
        closed_lid_y is not None
        and opened_lid_y is not None
        and opened_lid_y < closed_lid_y - 0.012,
        details=f"closed_lid_y={closed_lid_y}, opened_lid_y={opened_lid_y}",
    )
    closed_cam_max_x = closed_cam_aabb[1][0] if closed_cam_aabb is not None else None
    opened_cam_max_x = opened_cam_aabb[1][0] if opened_cam_aabb is not None else None
    ctx.check(
        "cam lever follows the lid opening motion",
        closed_cam_max_x is not None
        and opened_cam_max_x is not None
        and opened_cam_max_x > closed_cam_max_x + 0.003,
        details=f"closed_cam_max_x={closed_cam_max_x}, opened_cam_max_x={opened_cam_max_x}",
    )

    return ctx.report()


object_model = build_object_model()
