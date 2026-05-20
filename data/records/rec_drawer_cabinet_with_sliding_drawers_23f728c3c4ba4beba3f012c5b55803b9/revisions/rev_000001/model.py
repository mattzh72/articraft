from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stainless_mortuary_supply_cabinet")

    satin_steel = model.material("satin_stainless_steel", rgba=(0.72, 0.74, 0.73, 1.0))
    brushed_steel = model.material("brushed_stainless_steel", rgba=(0.86, 0.88, 0.87, 1.0))
    slide_steel = model.material("polished_slide_steel", rgba=(0.58, 0.60, 0.60, 1.0))
    hinge_steel = model.material("darker_hinge_steel", rgba=(0.48, 0.50, 0.50, 1.0))
    shadow = model.material("dark_recess_shadow", rgba=(0.05, 0.055, 0.06, 1.0))

    width = 1.10
    depth = 0.68
    height = 1.30
    wall = 0.035
    front_y = -depth / 2.0
    rear_y = depth / 2.0

    cabinet = model.part("cabinet")

    # Sheet-metal carcass: side panels, rear, bottom plinth, and front rails all
    # touch so the fixed cabinet reads as one welded stainless assembly.
    cabinet.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(-width / 2 + wall / 2, 0.0, height / 2)),
        material=satin_steel,
        name="side_panel_0",
    )
    cabinet.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(width / 2 - wall / 2, 0.0, height / 2)),
        material=satin_steel,
        name="side_panel_1",
    )
    cabinet.visual(
        Box((width, wall, height)),
        origin=Origin(xyz=(0.0, rear_y - wall / 2, height / 2)),
        material=satin_steel,
        name="rear_panel",
    )
    cabinet.visual(
        Box((width, depth, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=brushed_steel,
        name="bottom_pan",
    )
    cabinet.visual(
        Box((width, 0.030, 0.030)),
        origin=Origin(xyz=(0.0, front_y - 0.015, height - 0.045)),
        material=brushed_steel,
        name="top_front_rail",
    )
    cabinet.visual(
        Box((width, 0.030, 0.030)),
        origin=Origin(xyz=(0.0, front_y - 0.015, 0.115)),
        material=brushed_steel,
        name="bottom_front_rail",
    )
    cabinet.visual(
        Box((0.030, 0.030, height - 0.10)),
        origin=Origin(xyz=(-width / 2 + 0.015, front_y - 0.015, height / 2 + 0.035)),
        material=brushed_steel,
        name="front_stile_0",
    )
    cabinet.visual(
        Box((0.030, 0.030, height - 0.10)),
        origin=Origin(xyz=(width / 2 - 0.015, front_y - 0.015, height / 2 + 0.035)),
        material=brushed_steel,
        name="front_stile_1",
    )

    for idx, z in enumerate((0.37, 0.62, 0.87, 1.12)):
        cabinet.visual(
            Box((width, 0.026, 0.024)),
            origin=Origin(xyz=(0.0, front_y - 0.013, z)),
            material=brushed_steel,
            name=f"bay_rail_{idx}",
        )

    cabinet.visual(
        Box((width - 2 * wall, 0.020, height - 0.13)),
        origin=Origin(xyz=(0.0, rear_y - wall - 0.010, height / 2 + 0.03)),
        material=shadow,
        name="rear_shadow",
    )

    shelf_zs = (0.235, 0.485, 0.735, 0.985)
    for idx, z in enumerate(shelf_zs):
        # Fixed cabinet slide channels attached to the inside faces of the side
        # panels.  The moving rails on the shelves run just inboard of them.
        cabinet.visual(
            Box((0.020, 0.565, 0.035)),
            origin=Origin(xyz=(-width / 2 + wall + 0.010, -0.020, z - 0.022)),
            material=slide_steel,
            name=f"fixed_slide_{idx}_0",
        )
        cabinet.visual(
            Box((0.020, 0.565, 0.035)),
            origin=Origin(xyz=(width / 2 - wall - 0.010, -0.020, z - 0.022)),
            material=slide_steel,
            name=f"fixed_slide_{idx}_1",
        )

    # Two exposed rear hinge stations: fixed leaves and alternating fixed
    # knuckles are welded to the rear edge of the cabinet.
    hinge_xs = (-0.32, 0.32)
    hinge_axis_y = rear_y + 0.018
    hinge_axis_z = height + 0.018
    hinge_len = 0.18
    knuckle_len = 0.054
    for hidx, x in enumerate(hinge_xs):
        cabinet.visual(
            Box((hinge_len, 0.006, 0.074)),
            origin=Origin(xyz=(x, rear_y + 0.003, height - 0.037)),
            material=hinge_steel,
            name=f"hinge_leaf_{hidx}",
        )
        cabinet.visual(
            Box((hinge_len, 0.024, 0.012)),
            origin=Origin(xyz=(x, rear_y + 0.012, height + 0.004)),
            material=hinge_steel,
            name=f"hinge_curl_{hidx}",
        )
        for side, dx in enumerate((-0.063, 0.063)):
            cabinet.visual(
                Cylinder(radius=0.012, length=knuckle_len),
                origin=Origin(
                    xyz=(x + dx, hinge_axis_y, hinge_axis_z),
                    rpy=(0.0, pi / 2.0, 0.0),
                ),
                material=hinge_steel,
                name=f"hinge_knuckle_{hidx}_{side}",
            )

    # Four independent pull-out shelves.  Each shelf frame is centered on its
    # closed tray position; positive prismatic travel moves it out the front.
    shelf_width = 0.86
    shelf_depth = 0.54
    deck_thickness = 0.026
    side_lip_thickness = 0.024
    front_lip_height = 0.078
    side_lip_height = 0.047
    slide_len = 0.530
    slide_travel = 0.500

    for idx, z in enumerate(shelf_zs):
        shelf = model.part(f"shelf_{idx}")
        shelf.visual(
            Box((shelf_width, shelf_depth, deck_thickness)),
            origin=Origin(),
            material=brushed_steel,
            name="deck",
        )
        shelf.visual(
            Box((shelf_width + 2 * side_lip_thickness, 0.030, front_lip_height)),
            origin=Origin(xyz=(0.0, -shelf_depth / 2 - 0.015, front_lip_height / 2)),
            material=brushed_steel,
            name="front_lip",
        )
        shelf.visual(
            Box((side_lip_thickness, shelf_depth, side_lip_height)),
            origin=Origin(
                xyz=(
                    -shelf_width / 2 - side_lip_thickness / 2,
                    0.0,
                    side_lip_height / 2 - deck_thickness / 2,
                )
            ),
            material=brushed_steel,
            name="side_lip_0",
        )
        shelf.visual(
            Box((side_lip_thickness, shelf_depth, side_lip_height)),
            origin=Origin(
                xyz=(
                    shelf_width / 2 + side_lip_thickness / 2,
                    0.0,
                    side_lip_height / 2 - deck_thickness / 2,
                )
            ),
            material=brushed_steel,
            name="side_lip_1",
        )
        shelf.visual(
            Box((0.041, slide_len, 0.032)),
            origin=Origin(xyz=(-shelf_width / 2 - side_lip_thickness - 0.0205, 0.0, -0.024)),
            material=slide_steel,
            name="moving_slide_0",
        )
        shelf.visual(
            Box((0.041, slide_len, 0.032)),
            origin=Origin(xyz=(shelf_width / 2 + side_lip_thickness + 0.0205, 0.0, -0.024)),
            material=slide_steel,
            name="moving_slide_1",
        )
        shelf.visual(
            Box((shelf_width, 0.018, 0.045)),
            origin=Origin(xyz=(0.0, shelf_depth / 2 + 0.009, 0.008)),
            material=brushed_steel,
            name="rear_stiffener",
        )

        model.articulation(
            f"cabinet_to_shelf_{idx}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=shelf,
            origin=Origin(xyz=(0.0, -0.020, z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=180.0, velocity=0.35, lower=0.0, upper=slide_travel),
            motion_properties=MotionProperties(damping=8.0, friction=1.5),
        )

    lid = model.part("top_lid")
    lid_depth = depth + 0.040
    lid_width = width + 0.040
    lid_thickness = 0.035
    lid.visual(
        Box((lid_width, lid_depth, lid_thickness)),
        origin=Origin(xyz=(0.0, -0.018 - lid_depth / 2, 0.0)),
        material=brushed_steel,
        name="lid_panel",
    )
    lid.visual(
        Box((lid_width - 0.07, 0.020, 0.030)),
        origin=Origin(xyz=(0.0, -lid_depth + 0.060, -0.002)),
        material=satin_steel,
        name="front_rolled_edge",
    )
    for hidx, x in enumerate(hinge_xs):
        lid.visual(
            Cylinder(radius=0.0115, length=0.060),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=hinge_steel,
            name=f"moving_knuckle_{hidx}",
        )
        lid.visual(
            Box((0.060, 0.026, 0.010)),
            origin=Origin(xyz=(x, -0.006, 0.0)),
            material=hinge_steel,
            name=f"moving_curl_{hidx}",
        )
        lid.visual(
            Box((0.145, 0.100, 0.006)),
            origin=Origin(xyz=(x, -0.064, 0.0205)),
            material=hinge_steel,
            name=f"moving_leaf_{hidx}",
        )

    model.articulation(
        "cabinet_to_top_lid",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.2, lower=0.0, upper=1.25),
        motion_properties=MotionProperties(damping=2.0, friction=0.4),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    lid = object_model.get_part("top_lid")
    lid_joint = object_model.get_articulation("cabinet_to_top_lid")

    ctx.check(
        "top lid uses a rear revolute hinge",
        lid_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(lid_joint.axis) == (-1.0, 0.0, 0.0),
        details=f"type={lid_joint.articulation_type}, axis={lid_joint.axis}",
    )
    ctx.expect_gap(
        lid,
        cabinet,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem="lid_panel",
        negative_elem="side_panel_0",
        name="closed lid rests just above cabinet rim",
    )

    with ctx.pose({lid_joint: 1.25}):
        lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
        lid_zmax = None if lid_aabb is None else lid_aabb[1][2]
        ctx.check(
            "top lid opens upward clear of the cabinet",
            lid_zmax is not None and lid_zmax > 1.80,
            details=f"opened lid zmax={lid_zmax}",
        )

    for idx in range(4):
        shelf = object_model.get_part(f"shelf_{idx}")
        slide_joint = object_model.get_articulation(f"cabinet_to_shelf_{idx}")
        ctx.check(
            f"shelf {idx} uses a full-extension prismatic slide",
            slide_joint.articulation_type == ArticulationType.PRISMATIC
            and tuple(slide_joint.axis) == (0.0, -1.0, 0.0)
            and slide_joint.motion_limits is not None
            and slide_joint.motion_limits.upper >= 0.49,
            details=(
                f"type={slide_joint.articulation_type}, axis={slide_joint.axis}, "
                f"limits={slide_joint.motion_limits}"
            ),
        )
        ctx.expect_overlap(
            shelf,
            cabinet,
            axes="y",
            min_overlap=0.40,
            elem_a="moving_slide_0",
            elem_b=f"fixed_slide_{idx}_0",
            name=f"shelf {idx} left slide is fully nested when closed",
        )
        closed_pos = ctx.part_world_position(shelf)
        with ctx.pose({slide_joint: 0.50}):
            ctx.expect_overlap(
                shelf,
                cabinet,
                axes="y",
                min_overlap=0.045,
                elem_a="moving_slide_0",
                elem_b=f"fixed_slide_{idx}_0",
                name=f"shelf {idx} keeps slide engagement at full extension",
            )
            extended_pos = ctx.part_world_position(shelf)
        ctx.check(
            f"shelf {idx} travels out the front",
            closed_pos is not None
            and extended_pos is not None
            and extended_pos[1] < closed_pos[1] - 0.45,
            details=f"closed={closed_pos}, extended={extended_pos}",
        )

    return ctx.report()


object_model = build_object_model()
