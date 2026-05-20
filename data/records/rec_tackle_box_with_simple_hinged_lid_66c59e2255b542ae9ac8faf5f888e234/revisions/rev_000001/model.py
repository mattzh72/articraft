from __future__ import annotations

import math

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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_tackle_box")

    olive = Material("aged_olive_enamel", rgba=(0.30, 0.38, 0.26, 1.0))
    dark_olive = Material("dark_worn_edges", rgba=(0.17, 0.22, 0.15, 1.0))
    steel = Material("brushed_steel", rgba=(0.70, 0.69, 0.63, 1.0))
    galvanized = Material("galvanized_hardware", rgba=(0.56, 0.58, 0.55, 1.0))
    rubber = Material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    brass = Material("dulled_brass", rgba=(0.74, 0.56, 0.27, 1.0))

    length = 0.56
    depth = 0.32
    base_h = 0.18
    wall = 0.018
    bottom_t = 0.018
    rim_h = 0.018
    rim_w = 0.024

    lid_length = 0.60
    lid_depth = 0.36
    lid_top_t = 0.018
    lid_skirt_t = 0.018
    lid_skirt_h = 0.065
    hinge_y = depth / 2.0 + 0.015
    hinge_z = base_h + 0.010
    hinge_r = 0.009

    box_shell = model.part("box_shell")

    # Open metal tub: separate overlapped panels keep the top visibly hollow
    # while still reading as one welded/spot-riveted shell.
    box_shell.visual(
        Box((length, depth, bottom_t)),
        origin=Origin(xyz=(0.0, 0.0, bottom_t / 2.0)),
        material=olive,
        name="base_bottom",
    )
    box_shell.visual(
        Box((length, wall, base_h)),
        origin=Origin(xyz=(0.0, -depth / 2.0 + wall / 2.0, base_h / 2.0)),
        material=olive,
        name="front_wall",
    )
    box_shell.visual(
        Box((length, wall, base_h)),
        origin=Origin(xyz=(0.0, depth / 2.0 - wall / 2.0, base_h / 2.0)),
        material=olive,
        name="rear_wall",
    )
    box_shell.visual(
        Box((wall, depth, base_h)),
        origin=Origin(xyz=(-length / 2.0 + wall / 2.0, 0.0, base_h / 2.0)),
        material=olive,
        name="side_wall_0",
    )
    box_shell.visual(
        Box((wall, depth, base_h)),
        origin=Origin(xyz=(length / 2.0 - wall / 2.0, 0.0, base_h / 2.0)),
        material=olive,
        name="side_wall_1",
    )

    # Rolled rim/lip and bottom skids: old-school shell with pragmatic modern
    # stiffening at the places users actually pry, lift, and set the box down.
    box_shell.visual(
        Box((length + 0.020, rim_w, rim_h)),
        origin=Origin(xyz=(0.0, -depth / 2.0, base_h + rim_h / 2.0)),
        material=dark_olive,
        name="front_rim",
    )
    box_shell.visual(
        Box((length + 0.020, rim_w, rim_h)),
        origin=Origin(xyz=(0.0, depth / 2.0 - 0.018, base_h + rim_h / 2.0)),
        material=dark_olive,
        name="rear_rim",
    )
    box_shell.visual(
        Box((rim_w, depth + 0.020, rim_h)),
        origin=Origin(xyz=(-length / 2.0, 0.0, base_h + rim_h / 2.0)),
        material=dark_olive,
        name="side_rim_0",
    )
    box_shell.visual(
        Box((rim_w, depth + 0.020, rim_h)),
        origin=Origin(xyz=(length / 2.0, 0.0, base_h + rim_h / 2.0)),
        material=dark_olive,
        name="side_rim_1",
    )
    for i, x in enumerate((-0.16, 0.16)):
        box_shell.visual(
            Box((0.045, depth + 0.035, 0.014)),
            origin=Origin(xyz=(x, 0.0, 0.004)),
            material=rubber,
            name=f"skid_rail_{i}",
        )

    # Corner straps and gussets make the old rectangular tub look reinforced
    # rather than just like a plain box.
    corner_specs = (
        (-1.0, -1.0, "corner_strap_0"),
        (1.0, -1.0, "corner_strap_1"),
        (-1.0, 1.0, "corner_strap_2"),
        (1.0, 1.0, "corner_strap_3"),
    )
    for sx, sy, name in corner_specs:
        box_shell.visual(
            Box((0.040, 0.020, 0.145)),
            origin=Origin(
                xyz=(
                    sx * (length / 2.0 - 0.020),
                    sy * (depth / 2.0 - 0.006),
                    0.085,
                )
            ),
            material=galvanized,
            name=name,
        )

    # Static service hatches on both sides.  They are bolted through the side
    # walls and deliberately not floating decorative rectangles.
    for side, sx in (("0", -1.0), ("1", 1.0)):
        x = sx * (length / 2.0 + 0.004)
        box_shell.visual(
            Box((0.008, 0.125, 0.074)),
            origin=Origin(xyz=(x, 0.012, 0.096)),
            material=dark_olive,
            name=f"service_hatch_{side}",
        )
        for j, (yy, zz) in enumerate(
            ((-0.040, 0.070), (0.064, 0.070), (-0.040, 0.122), (0.064, 0.122))
        ):
            box_shell.visual(
                Cylinder(radius=0.005, length=0.006),
                origin=Origin(
                    xyz=(sx * (length / 2.0 + 0.010), yy, zz),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=steel,
                name=f"hatch_bolt_{side}_{j}",
            )

    # Front latch adapters project from the legacy shell so the moving catches
    # land on replaceable bolted hardware instead of thin sheet.
    latch_xs = (-0.16, 0.16)
    for i, x in enumerate(latch_xs):
        box_shell.visual(
            Box((0.096, 0.046, 0.064)),
            origin=Origin(xyz=(x, -depth / 2.0 - 0.020, 0.110)),
            material=galvanized,
            name=f"latch_adapter_{i}",
        )
        box_shell.visual(
            Cylinder(radius=0.006, length=0.075),
            origin=Origin(
                xyz=(x, -depth / 2.0 - 0.047, 0.110),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=steel,
            name=f"catch_bar_{i}",
        )
        for j, bx in enumerate((-0.030, 0.030)):
            box_shell.visual(
                Cylinder(radius=0.0045, length=0.005),
                origin=Origin(
                    xyz=(x + bx, -depth / 2.0 - 0.045, 0.140),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=brass,
                name=f"adapter_bolt_{i}_{j}",
            )

    # Alternating rear hinge knuckles carried by the box shell.
    for i, (x, knuckle_len) in enumerate(((-0.21, 0.090), (0.0, 0.110), (0.21, 0.090))):
        box_shell.visual(
            Cylinder(radius=hinge_r, length=knuckle_len),
            origin=Origin(
                xyz=(x, hinge_y, hinge_z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=steel,
            name=f"base_hinge_knuckle_{i}",
        )
        box_shell.visual(
            Box((knuckle_len, 0.016, 0.010)),
            origin=Origin(xyz=(x, hinge_y - 0.012, hinge_z - 0.006)),
            material=steel,
            name=f"base_hinge_web_{i}",
        )
    box_shell.visual(
        Box((length - 0.030, 0.010, 0.044)),
        origin=Origin(xyz=(0.0, depth / 2.0 - 0.001, base_h - 0.010)),
        material=galvanized,
        name="base_hinge_leaf",
    )

    lid = model.part("lid")
    roof_z = 0.020
    lid.visual(
        Box((lid_length, lid_depth, lid_top_t)),
        origin=Origin(xyz=(0.0, -lid_depth / 2.0, roof_z)),
        material=olive,
        name="lid_roof",
    )
    lid.visual(
        Box((lid_length, lid_skirt_t, lid_skirt_h)),
        origin=Origin(xyz=(0.0, -lid_depth - lid_skirt_t / 2.0, -0.015)),
        material=olive,
        name="front_skirt",
    )
    lid.visual(
        Box((lid_skirt_t, lid_depth, lid_skirt_h)),
        origin=Origin(xyz=(-lid_length / 2.0 - lid_skirt_t / 2.0, -lid_depth / 2.0, -0.015)),
        material=olive,
        name="side_skirt_0",
    )
    lid.visual(
        Box((lid_skirt_t, lid_depth, lid_skirt_h)),
        origin=Origin(xyz=(lid_length / 2.0 + lid_skirt_t / 2.0, -lid_depth / 2.0, -0.015)),
        material=olive,
        name="side_skirt_1",
    )
    lid.visual(
        Box((0.500, 0.014, 0.050)),
        origin=Origin(xyz=(0.0, -0.060, -0.010)),
        material=galvanized,
        name="lid_hinge_leaf",
    )
    for i, (x, knuckle_len) in enumerate(((-0.105, 0.095), (0.105, 0.095))):
        lid.visual(
            Cylinder(radius=hinge_r * 0.96, length=knuckle_len),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"lid_hinge_knuckle_{i}",
        )
        lid.visual(
            Box((knuckle_len, 0.062, 0.010)),
            origin=Origin(xyz=(x, -0.034, 0.010)),
            material=steel,
            name=f"lid_hinge_web_{i}",
        )

    # Lid ribs, a service plate, and a fixed arched carry bridge give the lid
    # the retrofit mix: simple legacy sheet form with modern repairable panels.
    for i, y in enumerate((-0.085, -0.180, -0.275)):
        lid.visual(
            Box((0.500, 0.020, 0.012)),
            origin=Origin(xyz=(0.0, y, roof_z + lid_top_t / 2.0 + 0.005)),
            material=dark_olive,
            name=f"lid_rib_{i}",
        )
    lid.visual(
        Box((0.165, 0.105, 0.006)),
        origin=Origin(xyz=(0.0, -0.260, roof_z + lid_top_t / 2.0 + 0.006)),
        material=galvanized,
        name="lid_service_panel",
    )
    for j, (x, y) in enumerate(((-0.065, -0.300), (0.065, -0.300), (-0.065, -0.220), (0.065, -0.220))):
        lid.visual(
            Cylinder(radius=0.0045, length=0.005),
            origin=Origin(xyz=(x, y, roof_z + lid_top_t / 2.0 + 0.011)),
            material=steel,
            name=f"lid_panel_bolt_{j}",
        )

    for i, x in enumerate((-0.105, 0.105)):
        lid.visual(
            Box((0.052, 0.034, 0.012)),
            origin=Origin(xyz=(x, -0.115, roof_z + lid_top_t / 2.0 + 0.006)),
            material=galvanized,
            name=f"handle_foot_{i}",
        )
        lid.visual(
            Box((0.018, 0.026, 0.058)),
            origin=Origin(xyz=(x, -0.115, roof_z + lid_top_t / 2.0 + 0.040)),
            material=steel,
            name=f"handle_post_{i}",
        )
    lid.visual(
        Box((0.235, 0.032, 0.020)),
        origin=Origin(xyz=(0.0, -0.115, roof_z + lid_top_t / 2.0 + 0.074)),
        material=steel,
        name="carry_grip",
    )

    # Latch hinge mounts fixed to the lid front skirt.
    latch_pin_y = -lid_depth - 0.028
    latch_pin_z = -0.005
    for i, x in enumerate(latch_xs):
        lid.visual(
            Box((0.086, 0.008, 0.040)),
            origin=Origin(xyz=(x, latch_pin_y + 0.013, latch_pin_z)),
            material=galvanized,
            name=f"latch_mount_{i}",
        )
        for j, lug_x in enumerate((-0.035, 0.035)):
            lid.visual(
                Box((0.014, 0.018, 0.030)),
                origin=Origin(xyz=(x + lug_x, latch_pin_y + 0.004, latch_pin_z)),
                material=steel,
                name=f"latch_lug_{i}_{j}",
            )

    lid_joint = model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=box_shell,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.6, lower=0.0, upper=1.25),
    )

    for i, x in enumerate(latch_xs):
        latch = model.part(f"latch_{i}")
        latch.visual(
            Cylinder(radius=0.006, length=0.052),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name="latch_barrel",
        )
        latch.visual(
            Box((0.050, 0.007, 0.090)),
            origin=Origin(xyz=(0.0, -0.003, -0.046)),
            material=steel,
            name="latch_plate",
        )
        latch.visual(
            Box((0.052, 0.022, 0.008)),
            origin=Origin(xyz=(0.0, 0.003, -0.092)),
            material=brass,
            name="latch_hook",
        )
        latch.visual(
            Box((0.060, 0.010, 0.022)),
            origin=Origin(xyz=(0.0, -0.007, -0.027)),
            material=galvanized,
            name="latch_stiffener",
        )
        model.articulation(
            f"latch_hinge_{i}",
            ArticulationType.REVOLUTE,
            parent=lid,
            child=latch,
            origin=Origin(xyz=(x, latch_pin_y, latch_pin_z)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=2.10),
        )

    lid_joint.meta["purpose"] = "rear utility hinge with alternating knuckles"
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    box_shell = object_model.get_part("box_shell")
    lid = object_model.get_part("lid")
    latch_0 = object_model.get_part("latch_0")
    latch_1 = object_model.get_part("latch_1")
    lid_hinge = object_model.get_articulation("lid_hinge")
    latch_hinge_0 = object_model.get_articulation("latch_hinge_0")
    latch_hinge_1 = object_model.get_articulation("latch_hinge_1")

    ctx.expect_overlap(
        lid,
        box_shell,
        axes="xy",
        min_overlap=0.25,
        elem_a="lid_roof",
        elem_b="base_bottom",
        name="lid covers the tackle box footprint",
    )
    ctx.expect_gap(
        lid,
        box_shell,
        axis="z",
        min_gap=0.0,
        max_gap=0.010,
        positive_elem="lid_roof",
        negative_elem="front_rim",
        name="closed lid clears the reinforced rim",
    )

    for i, latch in enumerate((latch_0, latch_1)):
        ctx.expect_overlap(
            latch,
            lid,
            axes="xz",
            min_overlap=0.010,
            elem_a="latch_barrel",
            elem_b=f"latch_mount_{i}",
            name=f"latch {i} barrel is captured by lid hardware",
        )
        ctx.expect_overlap(
            latch,
            box_shell,
            axes="x",
            min_overlap=0.040,
            elem_a="latch_hook",
            elem_b=f"catch_bar_{i}",
            name=f"latch {i} hook aligns with catch bar",
        )
        ctx.expect_gap(
            box_shell,
            latch,
            axis="z",
            min_gap=0.003,
            max_gap=0.030,
            positive_elem=f"catch_bar_{i}",
            negative_elem="latch_hook",
            name=f"latch {i} hook sits below catch",
        )

    closed_roof = ctx.part_element_world_aabb(lid, elem="lid_roof")
    with ctx.pose({lid_hinge: 1.10, latch_hinge_0: 2.0, latch_hinge_1: 2.0}):
        opened_roof = ctx.part_element_world_aabb(lid, elem="lid_roof")
    ctx.check(
        "lid opens upward from rear hinge",
        closed_roof is not None
        and opened_roof is not None
        and opened_roof[1][2] > closed_roof[1][2] + 0.14,
        details=f"closed={closed_roof}, opened={opened_roof}",
    )

    closed_hook = ctx.part_element_world_aabb(latch_0, elem="latch_hook")
    with ctx.pose({latch_hinge_0: 2.0}):
        opened_hook = ctx.part_element_world_aabb(latch_0, elem="latch_hook")
    ctx.check(
        "latch swings outward and clear",
        closed_hook is not None
        and opened_hook is not None
        and opened_hook[0][1] < closed_hook[0][1] - 0.040
        and opened_hook[0][2] > closed_hook[0][2] + 0.080,
        details=f"closed={closed_hook}, opened={opened_hook}",
    )

    return ctx.report()


object_model = build_object_model()
