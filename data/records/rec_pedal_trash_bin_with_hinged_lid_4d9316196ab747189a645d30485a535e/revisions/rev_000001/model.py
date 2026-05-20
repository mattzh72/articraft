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


# Object frame: +X is the front with the foot pedal, +Y is across the bin,
# and +Z is upward.  Dimensions are in meters.
DEPTH = 0.28
WIDTH = 0.32
HEIGHT = 0.42
WALL = 0.016


def _rounded_box_mesh(size: tuple[float, float, float], radius: float, name: str):
    """CadQuery rounded rectangular prism centered on the local origin."""
    sx, sy, sz = size
    safe_radius = min(radius, sx * 0.45, sy * 0.45)
    body = cq.Workplane("XY").box(sx, sy, sz).edges("|Z").fillet(safe_radius)
    return mesh_from_cadquery(body, name, tolerance=0.0008, angular_tolerance=0.08)


def _open_shell_mesh():
    """One-piece hollow plastic bin shell with a low side cleaning opening."""
    hatch_open_x = 0.118
    hatch_open_z = 0.138
    hatch_center_x = 0.024
    hatch_center_z = 0.145

    outer = (
        cq.Workplane("XY")
        .box(DEPTH, WIDTH, HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.028)
    )
    # Shell the top face inward to make the body genuinely hollow/open.
    shell = outer.faces(">Z").shell(-WALL)
    # Cut a cleaning hatch through the -Y side wall.
    cutter = (
        cq.Workplane("XY")
        .box(hatch_open_x, WALL * 4.0, hatch_open_z)
        .translate((hatch_center_x, -WIDTH / 2.0, hatch_center_z))
    )
    shell = shell.cut(cutter)
    return mesh_from_cadquery(shell, "shell_body", tolerance=0.001, angular_tolerance=0.08)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_pedal_bin")

    shell_mat = model.material("warm_grey_plastic", rgba=(0.72, 0.73, 0.70, 1.0))
    lid_mat = model.material("darker_grey_lid", rgba=(0.36, 0.38, 0.38, 1.0))
    hatch_mat = model.material("slate_hatch", rgba=(0.44, 0.48, 0.47, 1.0))
    gasket_mat = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    metal_mat = model.material("brushed_steel", rgba=(0.70, 0.70, 0.66, 1.0))

    shell = model.part("shell")
    shell.visual(_open_shell_mesh(), material=shell_mat, name="shell_body")

    # Dark top gasket/rim strips emphasize the open, hollow top rather than a
    # solid block.  They sit on the shell and remain part of the rigid shell.
    z_gasket = HEIGHT + 0.002
    shell.visual(
        Box((0.022, WIDTH - 0.030, 0.004)),
        origin=Origin(xyz=(DEPTH / 2.0 - 0.011, 0.0, z_gasket)),
        material=gasket_mat,
        name="top_rim_front",
    )
    shell.visual(
        Box((0.022, WIDTH - 0.030, 0.004)),
        origin=Origin(xyz=(-DEPTH / 2.0 + 0.011, 0.0, z_gasket)),
        material=gasket_mat,
        name="top_rim_rear",
    )
    shell.visual(
        Box((DEPTH - 0.034, 0.022, 0.004)),
        origin=Origin(xyz=(0.0, WIDTH / 2.0 - 0.011, z_gasket)),
        material=gasket_mat,
        name="top_rim_side_0",
    )
    shell.visual(
        Box((DEPTH - 0.034, 0.022, 0.004)),
        origin=Origin(xyz=(0.0, -WIDTH / 2.0 + 0.011, z_gasket)),
        material=gasket_mat,
        name="top_rim_side_1",
    )

    # Recessed dark service-hatch frame around the side opening.
    frame_y = -WIDTH / 2.0 - 0.001
    frame_x = 0.024
    frame_z = 0.145
    shell.visual(
        Box((0.165, 0.005, 0.012)),
        origin=Origin(xyz=(frame_x, frame_y, frame_z + 0.081)),
        material=gasket_mat,
        name="hatch_frame_top",
    )
    shell.visual(
        Box((0.165, 0.005, 0.012)),
        origin=Origin(xyz=(frame_x, frame_y, frame_z - 0.081)),
        material=gasket_mat,
        name="hatch_frame_bottom",
    )
    shell.visual(
        Box((0.012, 0.005, 0.162)),
        origin=Origin(xyz=(frame_x - 0.082, frame_y, frame_z)),
        material=gasket_mat,
        name="hatch_frame_rear",
    )
    shell.visual(
        Box((0.012, 0.005, 0.162)),
        origin=Origin(xyz=(frame_x + 0.082, frame_y, frame_z)),
        material=gasket_mat,
        name="hatch_frame_front",
    )

    # Static lower pivot brackets for the pedal.
    pivot_x = DEPTH / 2.0 + 0.008
    pivot_z = 0.070
    shell.visual(
        Box((0.040, 0.020, 0.065)),
        origin=Origin(xyz=(pivot_x, -0.122, 0.050)),
        material=shell_mat,
        name="pedal_bracket_0",
    )
    shell.visual(
        Box((0.040, 0.020, 0.065)),
        origin=Origin(xyz=(pivot_x, 0.122, 0.050)),
        material=shell_mat,
        name="pedal_bracket_1",
    )
    shell.visual(
        Cylinder(radius=0.008, length=0.020),
        origin=Origin(xyz=(pivot_x, -0.122, pivot_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_mat,
        name="pedal_bushing_0",
    )
    shell.visual(
        Cylinder(radius=0.008, length=0.020),
        origin=Origin(xyz=(pivot_x, 0.122, pivot_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_mat,
        name="pedal_bushing_1",
    )

    # Top rear hinge fixed knuckles and leaves.
    top_hinge_x = -DEPTH / 2.0 - 0.014
    top_hinge_z = HEIGHT + 0.020
    for i, y in enumerate((-0.120, 0.120)):
        shell.visual(
            Cylinder(radius=0.007, length=0.060),
            origin=Origin(xyz=(top_hinge_x, y, top_hinge_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal_mat,
            name=f"top_hinge_knuckle_{i}",
        )
        shell.visual(
            Box((0.018, 0.060, 0.028)),
            origin=Origin(xyz=(top_hinge_x + 0.008, y, top_hinge_z - 0.012)),
            material=metal_mat,
            name=f"top_hinge_leaf_{i}",
        )

    # Side service hatch fixed hinge knuckles.
    hatch_hinge_x = frame_x - 0.089
    hatch_hinge_y = -WIDTH / 2.0 - 0.010
    hatch_bottom_z = 0.058
    for i, z in enumerate((hatch_bottom_z + 0.030, hatch_bottom_z + 0.166)):
        shell.visual(
            Cylinder(radius=0.006, length=0.040),
            origin=Origin(xyz=(hatch_hinge_x, hatch_hinge_y, z)),
            material=metal_mat,
            name=f"side_hinge_knuckle_{i}",
        )
        shell.visual(
            Box((0.016, 0.011, 0.040)),
            origin=Origin(xyz=(hatch_hinge_x, -WIDTH / 2.0 - 0.004, z)),
            material=metal_mat,
            name=f"side_hinge_leaf_{i}",
        )

    pedal = model.part("pedal")
    pedal.visual(
        Cylinder(radius=0.010, length=0.205),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_mat,
        name="pedal_axle",
    )
    for i, y in enumerate((-0.065, 0.065)):
        pedal.visual(
            Box((0.150, 0.014, 0.038)),
            origin=Origin(xyz=(0.075, y, -0.020)),
            material=metal_mat,
            name=f"pedal_arm_{i}",
        )
    pedal.visual(
        _rounded_box_mesh((0.150, 0.210, 0.018), 0.018, "pedal_pad"),
        origin=Origin(xyz=(0.150, 0.0, -0.045)),
        material=gasket_mat,
        name="pedal_pad",
    )
    for i, x in enumerate((0.105, 0.140, 0.175)):
        pedal.visual(
            Box((0.010, 0.185, 0.003)),
            origin=Origin(xyz=(x, 0.0, -0.035)),
            material=Material("raised_pedal_tread", rgba=(0.08, 0.08, 0.075, 1.0)),
            name=f"pedal_tread_{i}",
        )

    pedal_joint = model.articulation(
        "shell_to_pedal",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=pedal,
        origin=Origin(xyz=(pivot_x, 0.0, pivot_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=0.0, upper=0.35),
    )

    lid = model.part("lid")
    lid.visual(
        _rounded_box_mesh((DEPTH + 0.026, WIDTH + 0.025, 0.024), 0.030, "lid_panel"),
        # The panel starts just in front of the rear hinge axis and extends
        # forward over the bin opening.
        origin=Origin(xyz=((DEPTH + 0.026) / 2.0 + 0.020, 0.0, 0.0)),
        material=lid_mat,
        name="lid_panel",
    )
    lid.visual(
        Cylinder(radius=0.0065, length=0.180),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_mat,
        name="lid_hinge_sleeve",
    )
    lid.visual(
        Box((0.026, 0.120, 0.005)),
        origin=Origin(xyz=(0.011, 0.0, -0.002)),
        material=metal_mat,
        name="lid_hinge_leaf",
    )
    lid.visual(
        Box((0.020, 0.115, 0.010)),
        origin=Origin(xyz=(DEPTH + 0.045, 0.0, -0.002)),
        material=gasket_mat,
        name="front_lip",
    )

    model.articulation(
        "shell_to_lid",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=lid,
        origin=Origin(xyz=(top_hinge_x, 0.0, top_hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.05),
        mimic=Mimic(joint=pedal_joint.name, multiplier=3.0, offset=0.0),
    )

    side_hatch = model.part("side_hatch")
    hatch_panel_w = 0.150
    hatch_panel_h = 0.182
    side_hatch.visual(
        _rounded_box_mesh((hatch_panel_w, 0.010, hatch_panel_h), 0.012, "side_hatch_panel"),
        origin=Origin(xyz=(hatch_panel_w / 2.0 + 0.010, -0.021, hatch_panel_h / 2.0)),
        material=hatch_mat,
        name="hatch_panel",
    )
    side_hatch.visual(
        Cylinder(radius=0.0055, length=0.096),
        origin=Origin(xyz=(0.0, 0.0, 0.098)),
        material=metal_mat,
        name="hatch_hinge_sleeve",
    )
    side_hatch.visual(
        Box((0.014, 0.018, 0.096)),
        origin=Origin(xyz=(0.006, -0.014, 0.098)),
        material=metal_mat,
        name="hatch_hinge_leaf",
    )
    side_hatch.visual(
        Box((0.038, 0.010, 0.018)),
        origin=Origin(xyz=(hatch_panel_w - 0.006, -0.028, hatch_panel_h * 0.56)),
        material=gasket_mat,
        name="hatch_pull",
    )

    model.articulation(
        "shell_to_side_hatch",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=side_hatch,
        origin=Origin(xyz=(hatch_hinge_x, hatch_hinge_y, hatch_bottom_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shell = object_model.get_part("shell")
    lid = object_model.get_part("lid")
    pedal = object_model.get_part("pedal")
    side_hatch = object_model.get_part("side_hatch")
    pedal_joint = object_model.get_articulation("shell_to_pedal")
    hatch_joint = object_model.get_articulation("shell_to_side_hatch")

    ctx.expect_overlap(
        lid,
        shell,
        axes="xy",
        elem_a="lid_panel",
        elem_b="shell_body",
        min_overlap=0.24,
        name="closed lid covers the top opening footprint",
    )
    ctx.expect_gap(
        lid,
        shell,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="top_rim_front",
        min_gap=0.001,
        max_gap=0.014,
        name="closed lid sits just above top rim",
    )
    ctx.expect_gap(
        shell,
        side_hatch,
        axis="y",
        positive_elem="shell_body",
        negative_elem="hatch_panel",
        min_gap=0.010,
        max_gap=0.035,
        name="side hatch is mounted outside side wall",
    )
    ctx.expect_overlap(
        side_hatch,
        shell,
        axes="xz",
        elem_a="hatch_panel",
        elem_b="shell_body",
        min_overlap=0.10,
        name="side hatch covers low cleaning opening",
    )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    closed_pedal_aabb = ctx.part_element_world_aabb(pedal, elem="pedal_pad")
    with ctx.pose({pedal_joint: 0.33}):
        opened_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
        depressed_pedal_aabb = ctx.part_element_world_aabb(pedal, elem="pedal_pad")
    ctx.check(
        "pedal depression raises the lid",
        closed_lid_aabb is not None
        and opened_lid_aabb is not None
        and opened_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.12,
        details=f"closed={closed_lid_aabb}, opened={opened_lid_aabb}",
    )
    ctx.check(
        "pedal pad moves downward about transverse pivot",
        closed_pedal_aabb is not None
        and depressed_pedal_aabb is not None
        and depressed_pedal_aabb[0][2] < closed_pedal_aabb[0][2] - 0.035,
        details=f"closed={closed_pedal_aabb}, depressed={depressed_pedal_aabb}",
    )

    closed_hatch_aabb = ctx.part_element_world_aabb(side_hatch, elem="hatch_panel")
    with ctx.pose({hatch_joint: 0.95}):
        open_hatch_aabb = ctx.part_element_world_aabb(side_hatch, elem="hatch_panel")
    ctx.check(
        "side hatch swings outward from side wall",
        closed_hatch_aabb is not None
        and open_hatch_aabb is not None
        and open_hatch_aabb[0][1] < closed_hatch_aabb[0][1] - 0.040,
        details=f"closed={closed_hatch_aabb}, opened={open_hatch_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
