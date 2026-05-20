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
)


def _tray_shell(width: float, depth: float, height: float, wall: float):
    """One-piece open tackle-box tub with a real hollow interior."""
    outer = cq.Workplane("XY").box(width, depth, height, centered=(True, True, False))
    # Small molded radii make the shell read as a rugged polymer casting.
    outer = outer.edges("|Z").fillet(0.010)
    inner = (
        cq.Workplane("XY")
        .box(width - 2.0 * wall, depth - 2.0 * wall, height + 0.030, centered=(True, True, False))
        .translate((0.0, 0.0, wall))
    )
    return outer.cut(inner)


def _tube_x(length: float, outer_radius: float, inner_radius: float):
    """Hollow tube centered at the origin with its axis along local X."""
    return (
        cq.Workplane("YZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((-0.5 * length, 0.0, 0.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_service_tackle_box")

    olive = model.material("olive_polymer", rgba=(0.26, 0.34, 0.20, 1.0))
    darker_olive = model.material("reinforced_rib_polymer", rgba=(0.16, 0.22, 0.13, 1.0))
    black_rubber = model.material("replaceable_black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    zinc = model.material("zinc_plated_steel", rgba=(0.72, 0.72, 0.66, 1.0))
    latch_red = model.material("red_latch_tab", rgba=(0.72, 0.10, 0.07, 1.0))

    width = 0.540
    depth = 0.300
    shell_height = 0.220
    wall = 0.018
    hinge_y = depth / 2.0 + 0.024
    hinge_z = shell_height + 0.024

    shell = model.part("shell")
    shell.visual(
        mesh_from_cadquery(_tray_shell(width, depth, shell_height, wall), "open_hollow_shell"),
        material=olive,
        name="shell_body",
    )

    # Replaceable bolted wear parts are authored as distinct rubber/steel visuals
    # on the shell so the tub stays one serviceable, connected root assembly.
    for i, y in enumerate((-0.112, 0.112)):
        shell.visual(
            Box((0.430, 0.026, 0.016)),
            origin=Origin(xyz=(0.0, y, -0.004)),
            material=black_rubber,
            name=f"bottom_skid_{i}",
        )
    for i, x in enumerate((-0.225, 0.225)):
        shell.visual(
            Box((0.040, 0.040, 0.090)),
            origin=Origin(xyz=(x, -depth / 2.0 - 0.006, 0.070)),
            material=black_rubber,
            name=f"front_bumper_{i}",
        )
        shell.visual(
            Box((0.040, 0.040, 0.090)),
            origin=Origin(xyz=(x, depth / 2.0 + 0.006, 0.070)),
            material=black_rubber,
            name=f"rear_bumper_{i}",
        )

    # Front latch strike points: thick replaceable catches with visible pins.
    latch_xs = (-0.145, 0.145)
    for i, x in enumerate(latch_xs):
        shell.visual(
            Box((0.086, 0.018, 0.052)),
            origin=Origin(xyz=(x, -depth / 2.0 - 0.009, 0.145)),
            material=zinc,
            name=f"catch_{i}",
        )
        shell.visual(
            Cylinder(radius=0.0055, length=0.078),
            origin=Origin(
                xyz=(x, -depth / 2.0 - 0.023, 0.142),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=zinc,
            name=f"catch_bar_{i}",
        )
        for z in (0.128, 0.162):
            shell.visual(
                Cylinder(radius=0.006, length=0.004),
                origin=Origin(
                    xyz=(x - 0.028, -depth / 2.0 - 0.016, z),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=zinc,
                name=f"catch_screw_{i}_{z:.3f}",
            )
            shell.visual(
                Cylinder(radius=0.006, length=0.004),
                origin=Origin(
                    xyz=(x + 0.028, -depth / 2.0 - 0.016, z),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=zinc,
                name=f"catch_screw_alt_{i}_{z:.3f}",
            )

    # Alternating hinge knuckles and a continuous serviceable hinge pin.
    base_knuckles = [(-0.228, 0.076), (0.0, 0.116), (0.228, 0.076)]
    for i, (x, length) in enumerate(base_knuckles):
        shell.visual(
            mesh_from_cadquery(_tube_x(length, 0.014, 0.008), f"base_hinge_knuckle_{i}"),
            origin=Origin(xyz=(x, hinge_y, hinge_z)),
            material=zinc,
            name=f"base_knuckle_{i}",
        )
        shell.visual(
            Box((length, 0.014, 0.036)),
            origin=Origin(xyz=(x, hinge_y - 0.017, hinge_z - 0.018)),
            material=zinc,
            name=f"base_hinge_leaf_{i}",
        )
    shell.visual(
        Cylinder(radius=0.0045, length=0.536),
        origin=Origin(xyz=(0.0, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=zinc,
        name="hinge_pin",
    )
    for i, x in enumerate((-0.265, 0.265)):
        shell.visual(
            Cylinder(radius=0.011, length=0.004),
            origin=Origin(xyz=(x, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=zinc,
            name=f"hinge_pin_retainer_{i}",
        )

    lid = model.part("lid")
    lid.visual(
        Box((0.570, 0.315, 0.024)),
        origin=Origin(xyz=(0.0, -0.184, -0.008)),
        material=olive,
        name="lid_top",
    )
    lid.visual(
        Box((0.570, 0.026, 0.018)),
        origin=Origin(xyz=(0.0, -0.184, 0.010)),
        material=darker_olive,
        name="top_center_rib",
    )
    for i, x in enumerate((-0.214, 0.214)):
        lid.visual(
            Box((0.030, 0.300, 0.018)),
            origin=Origin(xyz=(x, -0.184, 0.012)),
            material=darker_olive,
            name=f"top_length_rib_{i}",
        )
    lid.visual(
        Box((0.570, 0.014, 0.054)),
        origin=Origin(xyz=(0.0, -0.340, -0.038)),
        material=olive,
        name="front_skirt",
    )
    for i, x in enumerate((-0.284, 0.284)):
        lid.visual(
            Box((0.014, 0.300, 0.052)),
            origin=Origin(xyz=(x, -0.184, -0.037)),
            material=olive,
            name=f"side_skirt_{i}",
        )
    # A compressible-looking gasket stops shy of the shell rim in closed pose.
    lid.visual(
        Box((0.510, 0.014, 0.008)),
        origin=Origin(xyz=(0.0, -0.324, -0.017)),
        material=black_rubber,
        name="front_gasket",
    )
    for i, x in enumerate((-0.250, 0.250)):
        lid.visual(
            Box((0.014, 0.260, 0.008)),
            origin=Origin(xyz=(x, -0.184, -0.017)),
            material=black_rubber,
            name=f"side_gasket_{i}",
        )

    lid_knuckles = [(-0.130, 0.108), (0.130, 0.108)]
    for i, (x, length) in enumerate(lid_knuckles):
        lid.visual(
            mesh_from_cadquery(_tube_x(length, 0.014, 0.008), f"lid_hinge_knuckle_{i}"),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=zinc,
            name=f"lid_knuckle_{i}",
        )
        lid.visual(
            Box((length, 0.032, 0.012)),
            origin=Origin(xyz=(x, -0.020, -0.002)),
            material=zinc,
            name=f"lid_hinge_leaf_{i}",
        )

    # Chunky fixed handle with bolted collars; it is not a separate mechanism.
    lid.visual(
        Box((0.300, 0.034, 0.032)),
        origin=Origin(xyz=(0.0, -0.176, 0.044)),
        material=black_rubber,
        name="top_handle_grip",
    )
    for i, x in enumerate((-0.140, 0.140)):
        lid.visual(
            Box((0.040, 0.060, 0.046)),
            origin=Origin(xyz=(x, -0.176, 0.023)),
            material=zinc,
            name=f"handle_stand_{i}",
        )

    # Latch hinge pins mounted in reinforced front bosses on the lid.
    latch_joint_y = -0.360
    latch_joint_z = -0.026
    for i, x in enumerate(latch_xs):
        lid.visual(
            Box((0.092, 0.023, 0.030)),
            origin=Origin(xyz=(x, latch_joint_y + 0.020, latch_joint_z - 0.004)),
            material=zinc,
            name=f"latch_boss_{i}",
        )
        lid.visual(
            Cylinder(radius=0.0045, length=0.086),
            origin=Origin(
                xyz=(x, latch_joint_y, latch_joint_z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=zinc,
            name=f"latch_pin_{i}",
        )
        for side, sx in enumerate((-0.038, 0.038)):
            lid.visual(
                Cylinder(radius=0.009, length=0.006),
                origin=Origin(
                    xyz=(x + sx, latch_joint_y, latch_joint_z),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=zinc,
                name=f"latch_collar_{i}_{side}",
            )

    lid_hinge = model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=1.45),
    )

    for i, x in enumerate(latch_xs):
        latch = model.part(f"latch_{i}")
        latch.visual(
            mesh_from_cadquery(_tube_x(0.070, 0.008, 0.006), f"latch_hinge_tube_{i}"),
            material=zinc,
            name="latch_tube",
        )
        latch.visual(
            Box((0.056, 0.010, 0.092)),
            origin=Origin(xyz=(0.0, -0.009, -0.050)),
            material=latch_red,
            name="latch_plate",
        )
        latch.visual(
            Box((0.050, 0.026, 0.012)),
            origin=Origin(xyz=(0.0, 0.004, -0.094)),
            material=zinc,
            name="hook_toe",
        )
        for side, sx in enumerate((-0.023, 0.023)):
            latch.visual(
                Box((0.006, 0.014, 0.080)),
                origin=Origin(xyz=(sx, -0.010, -0.053)),
                material=zinc,
                name=f"side_flange_{side}",
            )
        model.articulation(
            f"latch_pivot_{i}",
            ArticulationType.REVOLUTE,
            parent=lid,
            child=latch,
            origin=Origin(xyz=(x, latch_joint_y, latch_joint_z)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=0.0, upper=1.25),
        )

    # Keep a small hint for downstream reviewers that the front latches are meant
    # to be operated independently before the lid is swung open.
    model.meta["service_notes"] = {
        "maintenance_access": "open-top tub, exposed hinge pin, replaceable skid and catch hardware",
        "primary_motion": [lid_hinge.name, "latch_pivot_0", "latch_pivot_1"],
    }
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shell = object_model.get_part("shell")
    lid = object_model.get_part("lid")
    lid_hinge = object_model.get_articulation("lid_hinge")

    ctx.expect_gap(
        lid,
        shell,
        axis="z",
        positive_elem="lid_top",
        negative_elem="shell_body",
        min_gap=0.002,
        max_gap=0.010,
        name="closed lid has service clearance above shell rim",
    )
    ctx.expect_overlap(
        lid,
        shell,
        axes="xy",
        elem_a="lid_top",
        elem_b="shell_body",
        min_overlap=0.250,
        name="lid covers the tackle box opening",
    )
    for i in (0, 1):
        ctx.expect_within(
            shell,
            lid,
            axes="yz",
            inner_elem="hinge_pin",
            outer_elem=f"lid_knuckle_{i}",
            margin=0.002,
            name=f"hinge pin runs through lid knuckle {i}",
        )
        ctx.expect_overlap(
            shell,
            lid,
            axes="x",
            elem_a="hinge_pin",
            elem_b=f"lid_knuckle_{i}",
            min_overlap=0.090,
            name=f"lid knuckle {i} has retained hinge engagement",
        )

    closed_lid_box = ctx.part_element_world_aabb(lid, elem="lid_top")
    with ctx.pose({lid_hinge: 1.20}):
        open_lid_box = ctx.part_element_world_aabb(lid, elem="lid_top")
    ctx.check(
        "lid opens upward around rear hinge",
        closed_lid_box is not None
        and open_lid_box is not None
        and open_lid_box[1][2] > closed_lid_box[1][2] + 0.120,
        details=f"closed={closed_lid_box}, open={open_lid_box}",
    )

    for i in (0, 1):
        latch = object_model.get_part(f"latch_{i}")
        latch_joint = object_model.get_articulation(f"latch_pivot_{i}")
        ctx.expect_within(
            lid,
            latch,
            axes="yz",
            inner_elem=f"latch_pin_{i}",
            outer_elem="latch_tube",
            margin=0.002,
            name=f"latch {i} is captured on its hinge pin",
        )
        ctx.expect_overlap(
            latch,
            shell,
            axes="x",
            elem_a="hook_toe",
            elem_b=f"catch_bar_{i}",
            min_overlap=0.040,
            name=f"latch {i} hook aligns with replaceable catch bar",
        )
        ctx.expect_gap(
            shell,
            latch,
            axis="z",
            positive_elem=f"catch_bar_{i}",
            negative_elem="hook_toe",
            min_gap=0.003,
            max_gap=0.012,
            name=f"latch {i} has robust catch clearance",
        )
        closed_plate = ctx.part_element_world_aabb(latch, elem="latch_plate")
        with ctx.pose({latch_joint: 1.05}):
            open_plate = ctx.part_element_world_aabb(latch, elem="latch_plate")
        ctx.check(
            f"latch {i} flips outward and upward",
            closed_plate is not None
            and open_plate is not None
            and open_plate[0][1] < closed_plate[0][1] - 0.030
            and open_plate[0][2] > closed_plate[0][2] + 0.018,
            details=f"closed={closed_plate}, open={open_plate}",
        )

    return ctx.report()


object_model = build_object_model()
