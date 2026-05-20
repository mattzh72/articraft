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
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="triple_canister_candy_vendor")

    painted = Material("painted_red_steel", rgba=(0.72, 0.06, 0.04, 1.0))
    dark = Material("dark_mechanism", rgba=(0.03, 0.035, 0.04, 1.0))
    chrome = Material("brushed_chrome", rgba=(0.78, 0.76, 0.70, 1.0))
    clear = Material("clear_polycarbonate", rgba=(0.78, 0.94, 1.0, 0.34))
    smoked = Material("smoked_clear_door", rgba=(0.30, 0.45, 0.55, 0.45))
    label = Material("cream_price_labels", rgba=(0.95, 0.86, 0.62, 1.0))
    candy_red = Material("red_candy", rgba=(0.88, 0.03, 0.04, 1.0))
    candy_gold = Material("gold_candy", rgba=(1.0, 0.62, 0.05, 1.0))
    candy_blue = Material("blue_candy", rgba=(0.08, 0.22, 0.88, 1.0))

    frame = model.part("frame")

    # Store-lobby proportions: a tall freestanding machine with a metal base,
    # front mechanism column, rear side rails, and three visibly separate bins.
    frame.visual(
        Box((0.62, 0.50, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark,
        name="floor_plinth",
    )
    frame.visual(
        Box((0.58, 0.46, 0.28)),
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        material=painted,
        name="base_cabinet",
    )
    frame.visual(
        Box((0.16, 0.07, 1.13)),
        origin=Origin(xyz=(0.0, -0.25, 0.845)),
        material=dark,
        name="mechanism_column",
    )
    frame.visual(
        Box((0.31, 0.006, 0.18)),
        origin=Origin(xyz=(0.0, -0.233, 0.15)),
        material=dark,
        name="pickup_recess",
    )
    frame.visual(
        Box((0.15, 0.006, 0.028)),
        origin=Origin(xyz=(0.0, -0.288, 1.37)),
        material=chrome,
        name="coin_slot",
    )

    for x in (-0.265, 0.265):
        frame.visual(
            Box((0.026, 0.032, 1.13)),
            origin=Origin(xyz=(x, 0.145, 0.845)),
            material=chrome,
            name=f"rear_post_{0 if x < 0 else 1}",
        )

    bin_ranges = [(0.34, 0.63), (0.68, 0.97), (1.02, 1.31)]
    candy_mats = [candy_red, candy_gold, candy_blue]
    for idx, (z_min, z_max) in enumerate(bin_ranges):
        zc = (z_min + z_max) / 2.0
        # Opaque rims and shelves make the three canisters read as separate
        # modules rather than one continuous transparent block.
        frame.visual(
            Box((0.52, 0.36, 0.018)),
            origin=Origin(xyz=(0.0, -0.015, z_min - 0.009)),
            material=chrome,
            name=f"bin_{idx}_lower_rim",
        )
        frame.visual(
            Box((0.52, 0.36, 0.018)),
            origin=Origin(xyz=(0.0, -0.015, z_max + 0.009)),
            material=chrome,
            name=f"bin_{idx}_upper_rim",
        )
        frame.visual(
            Box((0.45, 0.30, 0.012)),
            origin=Origin(xyz=(0.0, -0.015, z_min + 0.006)),
            material=clear,
            name=f"bin_{idx}_floor",
        )
        frame.visual(
            Box((0.012, 0.312, 0.29)),
            origin=Origin(xyz=(-0.231, -0.015, zc)),
            material=clear,
            name=f"bin_{idx}_side_0",
        )
        frame.visual(
            Box((0.012, 0.312, 0.29)),
            origin=Origin(xyz=(0.231, -0.015, zc)),
            material=clear,
            name=f"bin_{idx}_side_1",
        )
        frame.visual(
            Box((0.45, 0.012, 0.29)),
            origin=Origin(xyz=(0.0, -0.171, zc)),
            material=clear,
            name=f"bin_{idx}_front",
        )
        frame.visual(
            Box((0.45, 0.012, 0.29)),
            origin=Origin(xyz=(0.0, 0.141, zc)),
            material=clear,
            name=f"bin_{idx}_back",
        )
        # Narrow bridges show how each bin is served by the mechanism column,
        # while leaving air gaps around most of the column face.
        frame.visual(
            Box((0.070, 0.052, 0.040)),
            origin=Origin(xyz=(0.0, -0.191, zc - 0.025)),
            material=dark,
            name=f"bin_{idx}_feed_bridge",
        )
        frame.visual(
            Box((0.11, 0.008, 0.035)),
            origin=Origin(xyz=(0.0, -0.289, zc + 0.095)),
            material=label,
            name=f"price_label_{idx}",
        )
        # A connected candy mound in the bottom of each clear canister.
        frame.visual(
            Box((0.34, 0.18, 0.045)),
            origin=Origin(xyz=(0.0, -0.015, z_min + 0.034)),
            material=candy_mats[idx],
            name=f"candy_mound_{idx}",
        )
        for bump, (x, y) in enumerate(((-0.10, -0.06), (0.02, -0.01), (0.12, 0.055))):
            frame.visual(
                Sphere(0.038),
                origin=Origin(xyz=(x, y, z_min + 0.067)),
                material=candy_mats[idx],
                name=f"candy_bump_{idx}_{bump}",
            )

    frame.visual(
        Box((0.53, 0.37, 0.04)),
        origin=Origin(xyz=(0.0, -0.015, 1.43)),
        material=painted,
        name="top_cap",
    )

    knob_levels = [0.485, 0.825, 1.165]
    for idx, z in enumerate(knob_levels):
        knob = model.part(f"selector_knob_{idx}")
        knob.visual(
            Cylinder(radius=0.018, length=0.055),
            origin=Origin(xyz=(0.0, -0.015, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=chrome,
            name="shaft",
        )
        knob.visual(
            Cylinder(radius=0.062, length=0.050),
            origin=Origin(xyz=(0.0, -0.045, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=painted,
            name="grip_disk",
        )
        knob.visual(
            Box((0.016, 0.010, 0.078)),
            origin=Origin(xyz=(0.0, -0.074, 0.0)),
            material=chrome,
            name="pointer_bar",
        )
        model.articulation(
            f"knob_{idx}_spin",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=knob,
            origin=Origin(xyz=(0.0, -0.287, z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.8, velocity=8.0),
        )

    door = model.part("pickup_door")
    door.visual(
        Box((0.28, 0.022, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.088)),
        material=smoked,
        name="door_panel",
    )
    door.visual(
        Cylinder(radius=0.012, length=0.31),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="hinge_barrel",
    )
    door.visual(
        Box((0.17, 0.012, 0.022)),
        origin=Origin(xyz=(0.0, -0.016, 0.155)),
        material=chrome,
        name="pull_lip",
    )
    model.articulation(
        "pickup_door_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=door,
        origin=Origin(xyz=(0.0, -0.247, 0.065)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.5, lower=0.0, upper=1.15),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    door = object_model.get_part("pickup_door")
    door_hinge = object_model.get_articulation("pickup_door_hinge")

    for idx in range(3):
        knob = object_model.get_part(f"selector_knob_{idx}")
        spin = object_model.get_articulation(f"knob_{idx}_spin")
        ctx.check(
            f"selector knob {idx} is continuous",
            spin.articulation_type == ArticulationType.CONTINUOUS,
            details=f"articulation_type={spin.articulation_type}",
        )
        ctx.allow_overlap(
            frame,
            knob,
            elem_a="mechanism_column",
            elem_b="shaft",
            reason="The selector shaft is intentionally seated through the front mechanism column proxy.",
        )
        ctx.expect_gap(
            frame,
            knob,
            axis="y",
            positive_elem="mechanism_column",
            negative_elem="shaft",
            max_penetration=0.018,
            name=f"selector shaft {idx} is locally captured in column",
        )
        ctx.expect_overlap(
            knob,
            frame,
            axes="xz",
            elem_a="shaft",
            elem_b="mechanism_column",
            min_overlap=0.025,
            name=f"selector shaft {idx} aligns with column bore",
        )

    ctx.expect_gap(
        frame,
        door,
        axis="y",
        positive_elem="base_cabinet",
        negative_elem="door_panel",
        min_gap=0.001,
        max_gap=0.012,
        name="pickup door sits just proud of base",
    )

    closed_panel_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({door_hinge: 1.0}):
        open_panel_aabb = ctx.part_element_world_aabb(door, elem="door_panel")

    ctx.check(
        "pickup door swings outward and downward",
        closed_panel_aabb is not None
        and open_panel_aabb is not None
        and open_panel_aabb[0][1] < closed_panel_aabb[0][1] - 0.04
        and open_panel_aabb[1][2] < closed_panel_aabb[1][2] - 0.03,
        details=f"closed={closed_panel_aabb}, open={open_panel_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
