from __future__ import annotations

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


def _rounded_box(size: tuple[float, float, float], radius: float) -> object:
    """CadQuery rounded slab with softened vertical edges."""
    sx, sy, sz = size
    fillet = min(radius, sx * 0.20, sy * 0.20, sz * 0.20)
    return cq.Workplane("XY").box(sx, sy, sz).edges("|Z").fillet(fillet)


def _add_door_details(
    door,
    *,
    door_width: float,
    door_height: float,
    door_thickness: float,
    handle_kind: str,
    body_material,
    gasket_material,
    handle_material,
    hinge_material,
) -> None:
    """Add gaskets, handle, and hinge-side reinforcement in the door frame."""
    back_y = 0.0
    front_y = -1.0 * door_thickness - 0.012

    # Magnetic gasket ring on the inner/back face, visible when the door swings open.
    # It protrudes a few millimeters past the slab and just touches the cabinet
    # front frame in the closed pose, as a real compressible refrigerator seal does.
    gasket_y = back_y + 0.0025
    door.visual(
        Box((door_width - 0.070, 0.011, 0.020)),
        origin=Origin(xyz=(-door_width * 0.5, gasket_y, door_height - 0.030)),
        material=gasket_material,
        name="gasket_top",
    )
    door.visual(
        Box((door_width - 0.070, 0.011, 0.020)),
        origin=Origin(xyz=(-door_width * 0.5, gasket_y, 0.030)),
        material=gasket_material,
        name="gasket_bottom",
    )
    door.visual(
        Box((0.020, 0.011, door_height - 0.080)),
        origin=Origin(xyz=(-door_width + 0.030, gasket_y, door_height * 0.5)),
        material=gasket_material,
        name="gasket_pull_side",
    )
    door.visual(
        Box((0.020, 0.011, door_height - 0.080)),
        origin=Origin(xyz=(-0.030, gasket_y, door_height * 0.5)),
        material=gasket_material,
        name="gasket_hinge_side",
    )

    # A narrow metal hinge leaf on the moving door edge.  It is embedded in the
    # door edge, so it remains part of the door and rotates around the same axis.
    for index, zc in enumerate((door_height * 0.22, door_height * 0.78)):
        door.visual(
            Box((0.035, 0.010, 0.105)),
            origin=Origin(xyz=(-0.006, back_y - 0.004, zc)),
            material=hinge_material,
            name=f"hinge_leaf_{index}",
        )

    # Raised pull handles on the side opposite the hinges.  Posts partially
    # embed in the front skin and the bar overlaps the posts, keeping the door
    # part as one mechanically supported assembly.
    if handle_kind == "freezer":
        zc = 0.120
        x0 = -door_width + 0.115
        door.visual(
            Box((0.240, 0.034, 0.040)),
            origin=Origin(xyz=(x0, front_y - 0.020, zc)),
            material=handle_material,
            name="handle_bar",
        )
        for idx, dx in enumerate((-0.085, 0.085)):
            door.visual(
                Box((0.036, 0.048, 0.055)),
                origin=Origin(xyz=(x0 + dx, front_y + 0.002, zc)),
                material=handle_material,
                name=f"handle_post_{idx}",
            )
    else:
        x0 = -door_width + 0.105
        zc = door_height - 0.340
        handle_height = 0.520
        door.visual(
            Box((0.042, 0.034, handle_height)),
            origin=Origin(xyz=(x0, front_y - 0.020, zc)),
            material=handle_material,
            name="handle_bar",
        )
        for idx, dz in enumerate((-handle_height * 0.36, handle_height * 0.36)):
            door.visual(
                Box((0.062, 0.048, 0.060)),
                origin=Origin(xyz=(x0, front_y + 0.002, zc + dz)),
                material=handle_material,
                name=f"handle_post_{idx}",
            )

    # Thin inset panels suggest the molded door skin and keep the broad white
    # surface from reading as a featureless block.
    door.visual(
        Box((door_width - 0.105, 0.004, 0.018)),
        origin=Origin(xyz=(-door_width * 0.5, -door_thickness - 0.002, door_height - 0.055)),
        material=body_material,
        name="top_mold_line",
    )
    door.visual(
        Box((door_width - 0.105, 0.004, 0.018)),
        origin=Origin(xyz=(-door_width * 0.5, -door_thickness - 0.002, 0.055)),
        material=body_material,
        name="bottom_mold_line",
    )

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="top_freezer_refrigerator")

    enamel = model.material("warm_white_enamel", rgba=(0.91, 0.91, 0.87, 1.0))
    liner = model.material("pale_liner", rgba=(0.78, 0.80, 0.78, 1.0))
    shadow = model.material("dark_cavity", rgba=(0.045, 0.050, 0.055, 1.0))
    gasket = model.material("black_gasket", rgba=(0.020, 0.022, 0.024, 1.0))
    chrome = model.material("brushed_chrome", rgba=(0.66, 0.68, 0.68, 1.0))
    hinge = model.material("hinge_metal", rgba=(0.47, 0.49, 0.50, 1.0))
    glass = model.material("shelf_glass", rgba=(0.62, 0.78, 0.86, 0.42))
    grille = model.material("toe_grille_black", rgba=(0.035, 0.035, 0.035, 1.0))

    cabinet_width = 0.760
    cabinet_depth = 0.700
    cabinet_height = 1.720
    wall = 0.035
    front_y = -cabinet_depth * 0.5
    back_y = cabinet_depth * 0.5
    split_z = 1.185

    door_width = 0.710
    lower_height = 1.060
    upper_height = 0.480
    lower_bottom = 0.105
    upper_bottom = 1.205
    door_thickness = 0.065
    hinge_x = 0.365
    hinge_y = front_y - 0.008

    cabinet = model.part("cabinet")

    # Hollow cabinet carcass: open at the front, with side walls, back wall,
    # top/bottom, a real freezer/fresh-food divider, and visible interior shelves.
    cabinet.visual(
        Box((wall, cabinet_depth, cabinet_height)),
        origin=Origin(xyz=(-cabinet_width * 0.5 + wall * 0.5, 0.0, cabinet_height * 0.5)),
        material=enamel,
        name="side_wall_0",
    )
    cabinet.visual(
        Box((wall, cabinet_depth, cabinet_height)),
        origin=Origin(xyz=(cabinet_width * 0.5 - wall * 0.5, 0.0, cabinet_height * 0.5)),
        material=enamel,
        name="side_wall_1",
    )
    cabinet.visual(
        Box((cabinet_width, wall, cabinet_height)),
        origin=Origin(xyz=(0.0, back_y - wall * 0.5, cabinet_height * 0.5)),
        material=enamel,
        name="rear_wall",
    )
    cabinet.visual(
        Box((cabinet_width, cabinet_depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, cabinet_height - wall * 0.5)),
        material=enamel,
        name="top_wall",
    )
    cabinet.visual(
        Box((cabinet_width, cabinet_depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, wall * 0.5)),
        material=enamel,
        name="bottom_wall",
    )
    cabinet.visual(
        Box((cabinet_width, cabinet_depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, split_z)),
        material=liner,
        name="center_divider",
    )
    cabinet.visual(
        Box((cabinet_width - 2.0 * wall, 0.010, cabinet_height - 0.120)),
        origin=Origin(xyz=(0.0, back_y - wall - 0.006, cabinet_height * 0.5 + 0.020)),
        material=shadow,
        name="cavity_shadow",
    )

    # Front lips and mullion visible around the closed doors.
    cabinet.visual(
        Box((0.052, 0.034, cabinet_height - 0.035)),
        origin=Origin(xyz=(-cabinet_width * 0.5 + 0.026, front_y + 0.017, cabinet_height * 0.5)),
        material=enamel,
        name="front_stile_0",
    )
    cabinet.visual(
        Box((0.052, 0.034, cabinet_height - 0.035)),
        origin=Origin(xyz=(cabinet_width * 0.5 - 0.026, front_y + 0.017, cabinet_height * 0.5)),
        material=enamel,
        name="front_stile_1",
    )
    cabinet.visual(
        Box((cabinet_width, 0.034, 0.052)),
        origin=Origin(xyz=(0.0, front_y + 0.017, cabinet_height - 0.026)),
        material=enamel,
        name="top_lip",
    )
    cabinet.visual(
        Box((cabinet_width, 0.034, 0.052)),
        origin=Origin(xyz=(0.0, front_y + 0.017, 0.026)),
        material=enamel,
        name="bottom_lip",
    )
    cabinet.visual(
        Box((cabinet_width, 0.034, 0.056)),
        origin=Origin(xyz=(0.0, front_y + 0.017, split_z)),
        material=enamel,
        name="front_mullion",
    )
    cabinet.visual(
        Box((cabinet_width - 0.150, 0.020, 0.050)),
        origin=Origin(xyz=(0.0, front_y - 0.010, 0.075)),
        material=grille,
        name="toe_grille",
    )
    for i, x in enumerate((-0.265, -0.175, -0.085, 0.005, 0.095, 0.185, 0.275)):
        cabinet.visual(
            Box((0.030, 0.006, 0.044)),
            origin=Origin(xyz=(x, front_y - 0.022, 0.075)),
            material=shadow,
            name=f"grille_slot_{i}",
        )

    # Interior glass shelves contact both side walls, so they read as supported.
    for idx, zc in enumerate((0.430, 0.720, 0.995)):
        cabinet.visual(
            Box((cabinet_width - 2.0 * wall, cabinet_depth - 0.145, 0.010)),
            origin=Origin(xyz=(0.0, 0.035, zc)),
            material=glass,
            name=f"fresh_shelf_{idx}",
        )
        cabinet.visual(
            Box((0.018, cabinet_depth - 0.145, 0.030)),
            origin=Origin(xyz=(-cabinet_width * 0.5 + wall + 0.009, 0.035, zc)),
            material=liner,
            name=f"shelf_rail_0_{idx}",
        )
        cabinet.visual(
            Box((0.018, cabinet_depth - 0.145, 0.030)),
            origin=Origin(xyz=(cabinet_width * 0.5 - wall - 0.009, 0.035, zc)),
            material=liner,
            name=f"shelf_rail_1_{idx}",
        )
    cabinet.visual(
        Box((cabinet_width - 2.0 * wall, cabinet_depth - 0.145, 0.012)),
        origin=Origin(xyz=(0.0, 0.035, 1.425)),
        material=glass,
        name="freezer_shelf",
    )

    # Exposed hinge barrels mounted to the cabinet side.  Separate upper and
    # lower groups make the two vertical hinge axes visually unambiguous.
    for idx, (zc, length) in enumerate(((1.300, 0.120), (1.585, 0.120))):
        cabinet.visual(
            Cylinder(radius=0.018, length=length),
            origin=Origin(xyz=(hinge_x + 0.031, hinge_y - 0.004, zc)),
            material=hinge,
            name=f"freezer_hinge_barrel_{idx}",
        )
        cabinet.visual(
            Box((0.036, 0.018, length * 0.78)),
            origin=Origin(xyz=(hinge_x + 0.026, hinge_y + 0.015, zc)),
            material=hinge,
            name=f"freezer_hinge_leaf_{idx}",
        )
    for idx, (zc, length) in enumerate(((0.265, 0.145), (0.720, 0.145), (1.065, 0.145))):
        cabinet.visual(
            Cylinder(radius=0.018, length=length),
            origin=Origin(xyz=(hinge_x + 0.031, hinge_y - 0.004, zc)),
            material=hinge,
            name=f"fresh_hinge_barrel_{idx}",
        )
        cabinet.visual(
            Box((0.036, 0.018, length * 0.76)),
            origin=Origin(xyz=(hinge_x + 0.026, hinge_y + 0.015, zc)),
            material=hinge,
            name=f"fresh_hinge_leaf_{idx}",
        )

    upper_door = model.part("freezer_door")
    upper_door.visual(
        mesh_from_cadquery(
            _rounded_box((door_width, door_thickness, upper_height), 0.018),
            "freezer_door_slab",
        ),
        origin=Origin(xyz=(-door_width * 0.5, -door_thickness * 0.5, upper_height * 0.5)),
        material=enamel,
        name="door_slab",
    )
    _add_door_details(
        upper_door,
        door_width=door_width,
        door_height=upper_height,
        door_thickness=door_thickness,
        handle_kind="freezer",
        body_material=enamel,
        gasket_material=gasket,
        handle_material=chrome,
        hinge_material=hinge,
    )

    lower_door = model.part("fresh_door")
    lower_door.visual(
        mesh_from_cadquery(
            _rounded_box((door_width, door_thickness, lower_height), 0.020),
            "fresh_door_slab",
        ),
        origin=Origin(xyz=(-door_width * 0.5, -door_thickness * 0.5, lower_height * 0.5)),
        material=enamel,
        name="door_slab",
    )
    _add_door_details(
        lower_door,
        door_width=door_width,
        door_height=lower_height,
        door_thickness=door_thickness,
        handle_kind="fresh",
        body_material=enamel,
        gasket_material=gasket,
        handle_material=chrome,
        hinge_material=hinge,
    )

    model.articulation(
        "cabinet_to_freezer_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=upper_door,
        origin=Origin(xyz=(hinge_x, hinge_y, upper_bottom)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.8, lower=0.0, upper=1.95),
    )
    model.articulation(
        "cabinet_to_fresh_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lower_door,
        origin=Origin(xyz=(hinge_x, hinge_y, lower_bottom)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=22.0, velocity=1.6, lower=0.0, upper=1.95),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    freezer = object_model.get_part("freezer_door")
    fresh = object_model.get_part("fresh_door")
    freezer_hinge = object_model.get_articulation("cabinet_to_freezer_door")
    fresh_hinge = object_model.get_articulation("cabinet_to_fresh_door")

    freezer_box = ctx.part_element_world_aabb(freezer, elem="door_slab")
    fresh_box = ctx.part_element_world_aabb(fresh, elem="door_slab")
    freezer_height = freezer_box[1][2] - freezer_box[0][2] if freezer_box else 0.0
    fresh_height = fresh_box[1][2] - fresh_box[0][2] if fresh_box else 0.0
    ctx.check(
        "upper freezer door is smaller",
        0.0 < freezer_height < fresh_height * 0.60,
        details=f"freezer_height={freezer_height:.3f}, fresh_height={fresh_height:.3f}",
    )

    with ctx.pose({freezer_hinge: 0.0, fresh_hinge: 0.0}):
        ctx.expect_gap(
            cabinet,
            freezer,
            axis="y",
            min_gap=0.003,
            max_gap=0.030,
            positive_elem="front_mullion",
            negative_elem="door_slab",
            name="freezer door sits just in front of cabinet",
        )
        ctx.expect_gap(
            cabinet,
            fresh,
            axis="y",
            min_gap=0.003,
            max_gap=0.030,
            positive_elem="front_mullion",
            negative_elem="door_slab",
            name="fresh door sits just in front of cabinet",
        )
        ctx.expect_gap(
            freezer,
            fresh,
            axis="z",
            min_gap=0.012,
            max_gap=0.055,
            positive_elem="door_slab",
            negative_elem="door_slab",
            name="stacked doors have center reveal",
        )
        ctx.expect_overlap(
            freezer,
            cabinet,
            axes="x",
            min_overlap=0.42,
            elem_a="door_slab",
            elem_b="center_divider",
            name="freezer door covers upper compartment width",
        )
        ctx.expect_overlap(
            fresh,
            cabinet,
            axes="x",
            min_overlap=0.62,
            elem_a="door_slab",
            elem_b="center_divider",
            name="fresh door covers lower compartment width",
        )

    rest_freezer = ctx.part_world_aabb(freezer)
    with ctx.pose({freezer_hinge: 1.25}):
        open_freezer = ctx.part_world_aabb(freezer)
    ctx.check(
        "freezer door swings outward",
        rest_freezer is not None
        and open_freezer is not None
        and open_freezer[0][1] < rest_freezer[0][1] - 0.12,
        details=f"rest={rest_freezer}, open={open_freezer}",
    )

    rest_fresh = ctx.part_world_aabb(fresh)
    with ctx.pose({fresh_hinge: 1.25}):
        open_fresh = ctx.part_world_aabb(fresh)
    ctx.check(
        "fresh door swings outward",
        rest_fresh is not None
        and open_fresh is not None
        and open_fresh[0][1] < rest_fresh[0][1] - 0.12,
        details=f"rest={rest_fresh}, open={open_fresh}",
    )

    return ctx.report()


object_model = build_object_model()
