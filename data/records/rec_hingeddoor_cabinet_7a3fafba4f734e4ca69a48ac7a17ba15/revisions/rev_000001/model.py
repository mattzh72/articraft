from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="workshop_storage_cabinet")

    cabinet_steel = Material("powder_coated_grey_steel", color=(0.55, 0.59, 0.60, 1.0))
    door_steel = Material("blue_grey_door_steel", color=(0.28, 0.36, 0.42, 1.0))
    dark_steel = Material("dark_latch_steel", color=(0.03, 0.035, 0.04, 1.0))
    galvanized = Material("galvanized_hinge_steel", color=(0.74, 0.75, 0.72, 1.0))
    pegboard_mat = Material("brown_hardboard_pegboard", color=(0.56, 0.38, 0.22, 1.0))

    width = 1.20
    depth = 0.45
    height = 1.80
    sheet = 0.025
    door_thick = 0.028
    door_height = 1.72
    hinge_x = 0.625
    hinge_y = -depth / 2.0 - door_thick / 2.0
    door_width = hinge_x - 0.0075
    half_height = height / 2.0

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((sheet, depth, height)),
        origin=Origin(xyz=(-width / 2.0 + sheet / 2.0, 0.0, half_height)),
        material=cabinet_steel,
        name="side_panel_0",
    )
    cabinet.visual(
        Box((sheet, depth, height)),
        origin=Origin(xyz=(width / 2.0 - sheet / 2.0, 0.0, half_height)),
        material=cabinet_steel,
        name="side_panel_1",
    )
    cabinet.visual(
        Box((width, sheet, height)),
        origin=Origin(xyz=(0.0, depth / 2.0 - sheet / 2.0, half_height)),
        material=cabinet_steel,
        name="back_panel",
    )
    cabinet.visual(
        Box((width, depth, sheet)),
        origin=Origin(xyz=(0.0, 0.0, sheet / 2.0)),
        material=cabinet_steel,
        name="bottom_panel",
    )
    cabinet.visual(
        Box((width, depth, sheet)),
        origin=Origin(xyz=(0.0, 0.0, height - sheet / 2.0)),
        material=cabinet_steel,
        name="top_panel",
    )

    # Inset lips make the open front read as a bent-steel cabinet frame while
    # leaving the double doors free to close against the front plane.
    lip_y = -depth / 2.0 + sheet / 2.0
    cabinet.visual(
        Box((sheet, sheet, height - 2.0 * sheet)),
        origin=Origin(xyz=(-width / 2.0 + sheet / 2.0, lip_y, half_height)),
        material=cabinet_steel,
        name="front_stile_0",
    )
    cabinet.visual(
        Box((sheet, sheet, height - 2.0 * sheet)),
        origin=Origin(xyz=(width / 2.0 - sheet / 2.0, lip_y, half_height)),
        material=cabinet_steel,
        name="front_stile_1",
    )
    cabinet.visual(
        Box((width, sheet, sheet)),
        origin=Origin(xyz=(0.0, lip_y, sheet / 2.0)),
        material=cabinet_steel,
        name="front_bottom_lip",
    )
    cabinet.visual(
        Box((width, sheet, sheet)),
        origin=Origin(xyz=(0.0, lip_y, height - sheet / 2.0)),
        material=cabinet_steel,
        name="front_top_lip",
    )

    peg_mesh = mesh_from_geometry(
        PerforatedPanelGeometry(
            (0.50, 1.38),
            0.004,
            hole_diameter=0.008,
            pitch=(0.038, 0.038),
            frame=0.018,
            stagger=False,
        ),
        "internal_pegboard_panel",
    )
    peg_y = depth / 2.0 - sheet - 0.002
    for index, x in enumerate((-0.285, 0.285)):
        cabinet.visual(
            peg_mesh,
            origin=Origin(xyz=(x, peg_y, half_height), rpy=(pi / 2.0, 0.0, 0.0)),
            material=pegboard_mat,
            name=f"pegboard_panel_{index}",
        )

    door_peg_mesh = mesh_from_geometry(
        PerforatedPanelGeometry(
            (0.44, 1.20),
            0.004,
            hole_diameter=0.007,
            pitch=(0.035, 0.035),
            frame=0.016,
            stagger=False,
        ),
        "door_inner_pegboard",
    )

    hinge_zs = (-0.58, 0.0, 0.58)
    for side_index, side in enumerate((-1.0, 1.0)):
        side_x = side * (width / 2.0 + 0.003)
        for hinge_index, z in enumerate(hinge_zs):
            cabinet.visual(
                Box((0.008, 0.085, 0.155)),
                origin=Origin(xyz=(side_x, -depth / 2.0 + 0.046, half_height + z)),
                material=galvanized,
                name=f"fixed_hinge_leaf_{side_index}_{hinge_index}",
            )

    def add_door(name: str, side: float) -> object:
        door = model.part(name)
        panel_center_x = -side * door_width / 2.0
        meeting_x = -side * door_width

        door.visual(
            Box((door_width, door_thick, door_height)),
            origin=Origin(xyz=(panel_center_x, 0.0, 0.0)),
            material=door_steel,
            name="main_panel",
        )
        door.visual(
            door_peg_mesh,
            origin=Origin(xyz=(panel_center_x, door_thick / 2.0 + 0.002, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=pegboard_mat,
            name="inner_pegboard",
        )
        rib_y = -door_thick / 2.0 - 0.006 + 0.001
        door.visual(
            Box((door_width, 0.012, 0.046)),
            origin=Origin(xyz=(panel_center_x, rib_y, door_height / 2.0 - 0.023)),
            material=door_steel,
            name="top_rib",
        )
        door.visual(
            Box((door_width, 0.012, 0.046)),
            origin=Origin(xyz=(panel_center_x, rib_y, -door_height / 2.0 + 0.023)),
            material=door_steel,
            name="bottom_rib",
        )
        door.visual(
            Box((0.044, 0.012, door_height)),
            origin=Origin(xyz=(-side * 0.022, rib_y, 0.0)),
            material=door_steel,
            name="hinge_stile",
        )
        door.visual(
            Box((0.044, 0.012, door_height)),
            origin=Origin(xyz=(meeting_x + side * 0.022, rib_y, 0.0)),
            material=door_steel,
            name="meeting_stile",
        )

        for hinge_index, z in enumerate(hinge_zs):
            door.visual(
                Cylinder(radius=0.018, length=0.175),
                origin=Origin(xyz=(0.0, 0.0, z)),
                material=galvanized,
                name=f"hinge_barrel_{hinge_index}",
            )
            door.visual(
                Box((0.055, 0.010, 0.150)),
                origin=Origin(xyz=(-side * 0.027, rib_y, z)),
                material=galvanized,
                name=f"moving_hinge_leaf_{hinge_index}",
            )

        latch_x = meeting_x + side * 0.095
        escutcheon_thick = 0.006
        door.visual(
            Cylinder(radius=0.043, length=escutcheon_thick),
            origin=Origin(
                xyz=(latch_x, -door_thick / 2.0 - escutcheon_thick / 2.0, 0.0),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material=galvanized,
            name="latch_escutcheon",
        )
        door.visual(
            Box((0.014, 0.020, 0.175)),
            origin=Origin(xyz=(meeting_x + side * 0.011, -door_thick / 2.0 - 0.010, 0.0)),
            material=dark_steel,
            name="meeting_seal",
        )
        return door

    door_0 = add_door("door_0", -1.0)
    door_1 = add_door("door_1", 1.0)

    model.articulation(
        "cabinet_to_door_0",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door_0,
        origin=Origin(xyz=(-hinge_x, hinge_y, half_height)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.5, lower=0.0, upper=1.85),
    )
    model.articulation(
        "cabinet_to_door_1",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door_1,
        origin=Origin(xyz=(hinge_x, hinge_y, half_height)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.5, lower=0.0, upper=1.85),
    )

    def add_latch(name: str, door, side: float) -> None:
        latch = model.part(name)
        collar_thick = 0.014
        latch.visual(
            Cylinder(radius=0.034, length=collar_thick),
            origin=Origin(xyz=(0.0, -collar_thick / 2.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name="collar",
        )
        latch.visual(
            Box((0.165, 0.014, 0.030)),
            origin=Origin(xyz=(side * 0.008, -collar_thick - 0.007, 0.0)),
            material=dark_steel,
            name="lever",
        )
        latch.visual(
            Box((0.026, 0.018, 0.070)),
            origin=Origin(xyz=(side * 0.066, -collar_thick - 0.016, 0.0)),
            material=dark_steel,
            name="finger_grip",
        )
        latch_x = -side * door_width + side * 0.095
        model.articulation(
            f"{door.name}_to_{name}",
            ArticulationType.REVOLUTE,
            parent=door,
            child=latch,
            origin=Origin(xyz=(latch_x, -door_thick / 2.0 - 0.006, 0.0)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=-pi / 2.0, upper=pi / 2.0),
        )

    add_latch("latch_0", door_0, -1.0)
    add_latch("latch_1", door_1, 1.0)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    door_0 = object_model.get_part("door_0")
    door_1 = object_model.get_part("door_1")
    latch_0 = object_model.get_part("latch_0")
    latch_1 = object_model.get_part("latch_1")
    door_joint_0 = object_model.get_articulation("cabinet_to_door_0")
    door_joint_1 = object_model.get_articulation("cabinet_to_door_1")
    latch_joint_0 = object_model.get_articulation("door_0_to_latch_0")
    latch_joint_1 = object_model.get_articulation("door_1_to_latch_1")

    ctx.check(
        "two hinged doors and two rotary latches",
        all(part is not None for part in (door_0, door_1, latch_0, latch_1))
        and all(joint is not None for joint in (door_joint_0, door_joint_1, latch_joint_0, latch_joint_1)),
    )
    ctx.check(
        "door hinges use vertical side axes",
        tuple(door_joint_0.axis) == (0.0, 0.0, -1.0) and tuple(door_joint_1.axis) == (0.0, 0.0, 1.0),
    )
    ctx.check(
        "latch pivots are normal to the door faces",
        tuple(latch_joint_0.axis) == (0.0, -1.0, 0.0) and tuple(latch_joint_1.axis) == (0.0, -1.0, 0.0),
    )

    cabinet_visual_names = {visual.name for visual in cabinet.visuals}
    door_0_visual_names = {visual.name for visual in door_0.visuals}
    door_1_visual_names = {visual.name for visual in door_1.visuals}
    ctx.check(
        "internal pegboard panels are present",
        {"pegboard_panel_0", "pegboard_panel_1"}.issubset(cabinet_visual_names)
        and "inner_pegboard" in door_0_visual_names
        and "inner_pegboard" in door_1_visual_names,
    )

    with ctx.pose({door_joint_0: 0.0, door_joint_1: 0.0, latch_joint_0: 0.0, latch_joint_1: 0.0}):
        ctx.expect_gap(
            cabinet,
            door_0,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="side_panel_0",
            negative_elem="main_panel",
            name="door_0 closes against cabinet front",
        )
        ctx.expect_gap(
            cabinet,
            door_1,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="side_panel_1",
            negative_elem="main_panel",
            name="door_1 closes against cabinet front",
        )
        ctx.expect_contact(
            latch_0,
            door_0,
            elem_a="collar",
            elem_b="latch_escutcheon",
            contact_tol=0.001,
            name="latch_0 is mounted on its escutcheon",
        )
        ctx.expect_contact(
            latch_1,
            door_1,
            elem_a="collar",
            elem_b="latch_escutcheon",
            contact_tol=0.001,
            name="latch_1 is mounted on its escutcheon",
        )

        rest_0 = ctx.part_element_world_aabb(door_0, elem="main_panel")
        rest_1 = ctx.part_element_world_aabb(door_1, elem="main_panel")

    with ctx.pose({door_joint_0: 1.20, door_joint_1: 1.20}):
        open_0 = ctx.part_element_world_aabb(door_0, elem="main_panel")
        open_1 = ctx.part_element_world_aabb(door_1, elem="main_panel")

    def aabb_center(aabb, axis_index: int) -> float | None:
        if aabb is None:
            return None
        return (aabb[0][axis_index] + aabb[1][axis_index]) / 2.0

    rest_0_y = aabb_center(rest_0, 1)
    rest_1_y = aabb_center(rest_1, 1)
    open_0_y = aabb_center(open_0, 1)
    open_1_y = aabb_center(open_1, 1)
    ctx.check(
        "both doors swing outward",
        rest_0_y is not None
        and rest_1_y is not None
        and open_0_y is not None
        and open_1_y is not None
        and open_0_y < rest_0_y - 0.15
        and open_1_y < rest_1_y - 0.15,
        details=f"rest_y=({rest_0_y}, {rest_1_y}), open_y=({open_0_y}, {open_1_y})",
    )

    def aabb_size(aabb, axis_index: int) -> float | None:
        if aabb is None:
            return None
        return aabb[1][axis_index] - aabb[0][axis_index]

    with ctx.pose({latch_joint_0: 0.0, latch_joint_1: 0.0}):
        latch_0_rest = ctx.part_element_world_aabb(latch_0, elem="lever")
        latch_1_rest = ctx.part_element_world_aabb(latch_1, elem="lever")
    with ctx.pose({latch_joint_0: pi / 2.0, latch_joint_1: pi / 2.0}):
        latch_0_turn = ctx.part_element_world_aabb(latch_0, elem="lever")
        latch_1_turn = ctx.part_element_world_aabb(latch_1, elem="lever")

    ctx.check(
        "latch levers rotate from horizontal to vertical",
        all(
            value is not None
            for value in (
                aabb_size(latch_0_rest, 0),
                aabb_size(latch_0_turn, 2),
                aabb_size(latch_1_rest, 0),
                aabb_size(latch_1_turn, 2),
            )
        )
        and aabb_size(latch_0_turn, 2) > aabb_size(latch_0_rest, 2) + 0.08
        and aabb_size(latch_1_turn, 2) > aabb_size(latch_1_rest, 2) + 0.08,
    )

    return ctx.report()


object_model = build_object_model()
