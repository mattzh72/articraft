from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    shape = cq.Workplane("XY").box(*size)
    if radius > 0.0:
        shape = shape.edges("|Z").fillet(radius)
    return shape


def _hollow_bin_shell(
    depth: float,
    width: float,
    height: float,
    wall: float,
    bottom: float,
) -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(depth, width, height)
        .translate((0.0, 0.0, height / 2.0))
        .edges("|Z")
        .fillet(0.018)
    )
    cutter = (
        cq.Workplane("XY")
        .box(depth - 2.0 * wall, width - 2.0 * wall, height + 0.08)
        .translate((0.0, 0.0, bottom + (height + 0.08) / 2.0))
    )
    return outer.cut(cutter)


def _cylinder_y(radius: float, length: float) -> tuple[Cylinder, Origin]:
    return Cylinder(radius=radius, length=length), Origin(rpy=(math.pi / 2.0, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rectangular_kitchen_pedal_bin")

    shell_mat = model.material("warm_white_powder_coated_shell", rgba=(0.78, 0.77, 0.70, 1.0))
    dark_mat = model.material("dark_grey_plastic_trim", rgba=(0.06, 0.065, 0.07, 1.0))
    lid_mat = model.material("matte_charcoal_lid", rgba=(0.035, 0.038, 0.04, 1.0))
    hinge_mat = model.material("black_hinge_hardware", rgba=(0.015, 0.016, 0.018, 1.0))
    metal_mat = model.material("brushed_steel_pedal", rgba=(0.70, 0.72, 0.70, 1.0))
    wire_mat = model.material("stainless_wire_handle", rgba=(0.82, 0.84, 0.82, 1.0))

    depth = 0.34
    width = 0.25
    height = 0.62
    wall = 0.016
    bottom = 0.045
    rim_h = 0.010

    shell = model.part("outer_shell")
    shell.visual(
        mesh_from_cadquery(
            _hollow_bin_shell(depth, width, height, wall, bottom),
            "hollow_outer_shell",
            tolerance=0.0008,
            angular_tolerance=0.08,
        ),
        material=shell_mat,
        name="shell_body",
    )
    # Dark top rim strips leave a visible, even seam to the closed lid.
    shell.visual(
        Box((wall, width, rim_h)),
        origin=Origin(xyz=(depth / 2.0 - wall / 2.0, 0.0, height + rim_h / 2.0)),
        material=dark_mat,
        name="top_rim_front",
    )
    shell.visual(
        Box((wall, width, rim_h)),
        origin=Origin(xyz=(-depth / 2.0 + wall / 2.0, 0.0, height + rim_h / 2.0)),
        material=dark_mat,
        name="top_rim_rear",
    )
    shell.visual(
        Box((depth - 2.0 * wall, wall, rim_h)),
        origin=Origin(xyz=(0.0, width / 2.0 - wall / 2.0, height + rim_h / 2.0)),
        material=dark_mat,
        name="top_rim_side_0",
    )
    shell.visual(
        Box((depth - 2.0 * wall, wall, rim_h)),
        origin=Origin(xyz=(0.0, -width / 2.0 + wall / 2.0, height + rim_h / 2.0)),
        material=dark_mat,
        name="top_rim_side_1",
    )

    # Subtle inner bucket lip and bail handle visible just below the lid line.
    shell.visual(
        Box((depth - 0.060, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, width / 2.0 - wall - 0.005, height - 0.030)),
        material=dark_mat,
        name="inner_bucket_side_0",
    )
    shell.visual(
        Box((depth - 0.060, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, -width / 2.0 + wall + 0.005, height - 0.030)),
        material=dark_mat,
        name="inner_bucket_side_1",
    )
    handle_geom = tube_from_spline_points(
        [
            (0.045, -0.109, height - 0.018),
            (0.045, -0.065, height + 0.003),
            (0.045, 0.000, height + 0.010),
            (0.045, 0.065, height + 0.003),
            (0.045, 0.109, height - 0.018),
        ],
        radius=0.0032,
        samples_per_segment=10,
        radial_segments=16,
        cap_ends=True,
    )
    shell.visual(
        mesh_from_geometry(handle_geom, "inner_bucket_handle"),
        material=wire_mat,
        name="inner_bucket_handle",
    )

    # Rear lid hinge supports mounted into the shell, split to leave space for the
    # moving lid knuckle.
    lid_hinge_x = -0.195
    lid_hinge_z = height + 0.020
    for suffix, y in (("0", -0.088), ("1", 0.088)):
        shell.visual(
            Box((0.034, 0.046, 0.014)),
            origin=Origin(xyz=(lid_hinge_x + 0.012, y, lid_hinge_z - 0.010)),
            material=hinge_mat,
            name=f"lid_hinge_leaf_{suffix}",
        )
        cyl, rot = _cylinder_y(0.010, 0.050)
        shell.visual(
            cyl,
            origin=Origin(xyz=(lid_hinge_x, y, lid_hinge_z), rpy=rot.rpy),
            material=hinge_mat,
            name=f"lid_hinge_barrel_{suffix}",
        )
    cyl, rot = _cylinder_y(0.0032, 0.235)
    shell.visual(
        cyl,
        origin=Origin(xyz=(lid_hinge_x, 0.0, lid_hinge_z), rpy=rot.rpy),
        material=hinge_mat,
        name="lid_hinge_pin",
    )

    # Lower front pedal pivot bosses are visibly tied back into the front wall.
    pedal_x = depth / 2.0 + 0.017
    pedal_z = 0.075
    for suffix, y in (("0", -0.089), ("1", 0.089)):
        shell.visual(
            Box((0.034, 0.030, 0.030)),
            origin=Origin(xyz=(depth / 2.0 + 0.010, y, pedal_z)),
            material=hinge_mat,
            name=f"pedal_pivot_boss_{suffix}",
        )

    # Small rear lugs for the soft-close damper cover hinge.
    damper_x = -0.270
    damper_z = height + 0.008
    for suffix, y in (("0", -0.083), ("1", 0.083)):
        shell.visual(
            Box((0.100, 0.020, 0.014)),
            origin=Origin(xyz=((damper_x - depth / 2.0) / 2.0, y, damper_z)),
            material=hinge_mat,
            name=f"damper_hinge_bridge_{suffix}",
        )
        shell.visual(
            Box((0.032, 0.026, 0.026)),
            origin=Origin(xyz=(damper_x + 0.012, y, damper_z)),
            material=hinge_mat,
            name=f"damper_hinge_lug_{suffix}",
        )
    cyl, rot = _cylinder_y(0.0030, 0.185)
    shell.visual(
        cyl,
        origin=Origin(xyz=(damper_x, 0.0, damper_z), rpy=rot.rpy),
        material=hinge_mat,
        name="damper_hinge_pin",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(
            _rounded_box((0.340, 0.238, 0.026), 0.018).translate((0.190, 0.0, 0.013)),
            "flat_top_lid",
            tolerance=0.0006,
            angular_tolerance=0.08,
        ),
        material=lid_mat,
        name="lid_panel",
    )
    lid.visual(
        Box((0.030, 0.070, 0.006)),
        origin=Origin(xyz=(0.015, 0.0, 0.010)),
        material=hinge_mat,
        name="lid_hinge_leaf",
    )
    cyl, rot = _cylinder_y(0.009, 0.064)
    lid.visual(
        cyl,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=rot.rpy),
        material=hinge_mat,
        name="lid_hinge_barrel",
    )

    pedal = model.part("pedal_plate")
    cyl, rot = _cylinder_y(0.007, 0.205)
    pedal.visual(
        cyl,
        origin=Origin(rpy=rot.rpy),
        material=hinge_mat,
        name="pedal_shaft",
    )
    pedal.visual(
        Box((0.032, 0.110, 0.020)),
        origin=Origin(xyz=(0.018, 0.0, 0.010)),
        material=hinge_mat,
        name="pedal_neck",
    )
    pedal.visual(
        Box((0.014, 0.155, 0.066)),
        origin=Origin(xyz=(0.038, 0.0, 0.046)),
        material=metal_mat,
        name="pedal_face",
    )
    for i, z in enumerate((0.026, 0.043, 0.060)):
        pedal.visual(
            Box((0.003, 0.130, 0.004)),
            origin=Origin(xyz=(0.0465, 0.0, z)),
            material=dark_mat,
            name=f"pedal_grip_{i}",
        )

    damper_cover = model.part("damper_cover")
    cyl, rot = _cylinder_y(0.007, 0.055)
    damper_cover.visual(
        cyl,
        origin=Origin(rpy=rot.rpy),
        material=hinge_mat,
        name="damper_cover_barrel",
    )
    damper_cover.visual(
        mesh_from_cadquery(
            _rounded_box((0.055, 0.120, 0.035), 0.010).translate((0.033, 0.0, 0.008)),
            "soft_close_damper_cover",
            tolerance=0.0006,
            angular_tolerance=0.08,
        ),
        material=dark_mat,
        name="damper_cap",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=lid,
        origin=Origin(xyz=(lid_hinge_x, 0.0, lid_hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=0.0, upper=1.35),
    )
    model.articulation(
        "pedal_pivot",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=pedal,
        origin=Origin(xyz=(pedal_x, 0.0, pedal_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=0.38),
    )
    model.articulation(
        "damper_hinge",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=damper_cover,
        origin=Origin(xyz=(damper_x, 0.0, damper_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.2, lower=0.0, upper=1.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shell = object_model.get_part("outer_shell")
    lid = object_model.get_part("lid")
    pedal = object_model.get_part("pedal_plate")
    cover = object_model.get_part("damper_cover")
    lid_hinge = object_model.get_articulation("lid_hinge")
    pedal_pivot = object_model.get_articulation("pedal_pivot")
    damper_hinge = object_model.get_articulation("damper_hinge")

    ctx.allow_overlap(
        shell,
        lid,
        elem_a="lid_hinge_pin",
        elem_b="lid_hinge_barrel",
        reason="The fixed hinge pin is intentionally captured inside the moving lid barrel.",
    )
    ctx.expect_within(
        shell,
        lid,
        axes="xz",
        inner_elem="lid_hinge_pin",
        outer_elem="lid_hinge_barrel",
        margin=0.001,
        name="lid hinge pin sits inside the moving barrel",
    )
    ctx.expect_overlap(
        shell,
        lid,
        axes="y",
        elem_a="lid_hinge_pin",
        elem_b="lid_hinge_barrel",
        min_overlap=0.055,
        name="lid barrel remains carried on the hinge pin",
    )

    ctx.allow_overlap(
        shell,
        cover,
        elem_a="damper_hinge_pin",
        elem_b="damper_cover_barrel",
        reason="The damper-cover hinge pin is intentionally captured inside the cover barrel.",
    )
    ctx.expect_within(
        shell,
        cover,
        axes="xz",
        inner_elem="damper_hinge_pin",
        outer_elem="damper_cover_barrel",
        margin=0.001,
        name="damper cover pin sits inside its barrel",
    )
    ctx.expect_overlap(
        shell,
        cover,
        axes="y",
        elem_a="damper_hinge_pin",
        elem_b="damper_cover_barrel",
        min_overlap=0.045,
        name="damper cover barrel remains carried on its short pin",
    )

    for boss in ("pedal_pivot_boss_0", "pedal_pivot_boss_1"):
        ctx.allow_overlap(
            shell,
            pedal,
            elem_a=boss,
            elem_b="pedal_shaft",
            reason="The pedal shaft is intentionally captured through the lower front pivot boss.",
        )
        ctx.expect_within(
            pedal,
            shell,
            axes="xz",
            inner_elem="pedal_shaft",
            outer_elem=boss,
            margin=0.001,
            name=f"{boss} contains the pedal shaft",
        )
        ctx.expect_overlap(
            shell,
            pedal,
            axes="y",
            elem_a=boss,
            elem_b="pedal_shaft",
            min_overlap=0.020,
            name=f"{boss} axially supports the pedal shaft",
        )

    ctx.expect_overlap(
        lid,
        shell,
        axes="xy",
        elem_a="lid_panel",
        elem_b="shell_body",
        min_overlap=0.20,
        name="closed lid covers the rectangular top opening",
    )
    ctx.expect_gap(
        lid,
        shell,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="top_rim_front",
        min_gap=0.006,
        max_gap=0.020,
        name="lid sits just above the clean top seam",
    )
    ctx.expect_gap(
        pedal,
        shell,
        axis="x",
        positive_elem="pedal_face",
        negative_elem="shell_body",
        min_gap=0.004,
        max_gap=0.050,
        name="pedal plate is mounted proud of the lower front face",
    )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({lid_hinge: 1.0}):
        opened_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    ctx.check(
        "lid hinge raises the flat lid upward",
        closed_lid_aabb is not None
        and opened_lid_aabb is not None
        and opened_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.10,
        details=f"closed={closed_lid_aabb}, opened={opened_lid_aabb}",
    )

    rest_pedal_aabb = ctx.part_element_world_aabb(pedal, elem="pedal_face")
    with ctx.pose({pedal_pivot: 0.30}):
        pressed_pedal_aabb = ctx.part_element_world_aabb(pedal, elem="pedal_face")
    ctx.check(
        "front pedal rotates about the lower pivot",
        rest_pedal_aabb is not None
        and pressed_pedal_aabb is not None
        and pressed_pedal_aabb[1][0] > rest_pedal_aabb[1][0] + 0.010
        and pressed_pedal_aabb[0][2] < rest_pedal_aabb[0][2] - 0.004,
        details=f"rest={rest_pedal_aabb}, pressed={pressed_pedal_aabb}",
    )

    closed_cover_aabb = ctx.part_element_world_aabb(cover, elem="damper_cap")
    with ctx.pose({damper_hinge: 0.80}):
        opened_cover_aabb = ctx.part_element_world_aabb(cover, elem="damper_cap")
    ctx.check(
        "damper cover rotates on its short rear hinge",
        closed_cover_aabb is not None
        and opened_cover_aabb is not None
        and opened_cover_aabb[1][2] > closed_cover_aabb[1][2] + 0.025,
        details=f"closed={closed_cover_aabb}, opened={opened_cover_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
