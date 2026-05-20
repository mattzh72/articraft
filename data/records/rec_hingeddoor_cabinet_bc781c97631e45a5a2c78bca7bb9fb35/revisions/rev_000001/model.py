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
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="small_wall_cabinet")

    painted_wood = Material("warm_white_painted_wood", rgba=(0.86, 0.82, 0.72, 1.0))
    inner_wood = Material("shadowed_interior_wood", rgba=(0.74, 0.68, 0.57, 1.0))
    brass = Material("brushed_brass", rgba=(0.76, 0.57, 0.25, 1.0))
    dark_metal = Material("dark_oil_rubbed_metal", rgba=(0.05, 0.045, 0.04, 1.0))

    width = 0.60
    depth = 0.22
    height = 0.65
    case_thick = 0.018
    back_thick = 0.012
    shelf_thick = 0.014
    door_thick = 0.018
    center_gap = 0.008
    door_width = (width - center_gap) / 2.0
    door_height = height - 0.030
    door_center_y = -depth / 2.0 - door_thick / 2.0
    hinge_radius = 0.006

    cabinet = model.part("cabinet")
    inner_width = width - 2.0 * case_thick
    inner_height = height - 2.0 * case_thick
    shelf_depth = depth - back_thick

    cabinet.visual(
        Box((case_thick, depth, height)),
        origin=Origin(xyz=(-width / 2.0 + case_thick / 2.0, 0.0, height / 2.0)),
        material=painted_wood,
        name="side_panel_0",
    )
    cabinet.visual(
        Box((case_thick, depth, height)),
        origin=Origin(xyz=(width / 2.0 - case_thick / 2.0, 0.0, height / 2.0)),
        material=painted_wood,
        name="side_panel_1",
    )
    cabinet.visual(
        Box((inner_width, depth, case_thick)),
        origin=Origin(xyz=(0.0, 0.0, case_thick / 2.0)),
        material=painted_wood,
        name="bottom_panel",
    )
    cabinet.visual(
        Box((inner_width, depth, case_thick)),
        origin=Origin(xyz=(0.0, 0.0, height - case_thick / 2.0)),
        material=painted_wood,
        name="top_panel",
    )
    cabinet.visual(
        Box((inner_width, back_thick, inner_height)),
        origin=Origin(xyz=(0.0, depth / 2.0 - back_thick / 2.0, height / 2.0)),
        material=inner_wood,
        name="back_panel",
    )
    for index, z in enumerate((0.245, 0.425)):
        cabinet.visual(
            Box((inner_width, shelf_depth, shelf_thick)),
            origin=Origin(xyz=(0.0, -back_thick / 2.0, z)),
            material=inner_wood,
            name=f"shelf_{index}",
        )

    for suffix, x, side_sign in (("0", -width / 2.0 - 0.002, -1.0), ("1", width / 2.0 + 0.002, 1.0)):
        for index, z in enumerate((0.18, 0.47)):
            cabinet.visual(
                Box((0.004, 0.065, 0.080)),
                origin=Origin(xyz=(x, -depth / 2.0 + 0.040, z)),
                material=brass,
                name=f"hinge_leaf_{suffix}_{index}",
            )
            cabinet.visual(
                Cylinder(radius=0.0045, length=0.082),
                origin=Origin(xyz=(side_sign * (width / 2.0 + 0.0045), -depth / 2.0 + 0.006, z)),
                material=brass,
                name=f"hinge_knuckle_{suffix}_{index}",
            )

    def add_door(
        name: str,
        hinge_x: float,
        panel_center_x: float,
        free_edge_x: float,
        hinge_axis: tuple[float, float, float],
    ):
        door = model.part(name)
        # The child frame is the hinge line.  Closed door geometry is offset from
        # that frame so the two leaves meet with a narrow, realistic center gap.
        door.visual(
            Box((door_width, door_thick, door_height)),
            origin=Origin(xyz=(panel_center_x, 0.0, height / 2.0)),
            material=painted_wood,
            name="door_panel",
        )
        stile_w = 0.030
        rail_h = 0.040
        trim_y = -door_thick / 2.0 - 0.003
        trim_t = 0.006
        door_sign = 1.0 if panel_center_x > 0.0 else -1.0
        for trim_name, x, z, sx, sz in (
            ("hinge_stile", door_sign * (stile_w / 2.0), height / 2.0, stile_w, door_height - 0.035),
            ("free_stile", door_sign * (door_width - stile_w / 2.0), height / 2.0, stile_w, door_height - 0.035),
            ("top_rail", panel_center_x, height / 2.0 + door_height / 2.0 - rail_h / 2.0 - 0.018, door_width - 0.025, rail_h),
            ("bottom_rail", panel_center_x, height / 2.0 - door_height / 2.0 + rail_h / 2.0 + 0.018, door_width - 0.025, rail_h),
        ):
            door.visual(
                Box((sx, trim_t, sz)),
                origin=Origin(xyz=(x, trim_y, z)),
                material=painted_wood,
                name=trim_name,
            )

        pull_x = free_edge_x
        pull_y = -door_thick / 2.0 - 0.038
        for post_index, post_z in enumerate((height / 2.0 - 0.075, height / 2.0 + 0.075)):
            door.visual(
                Cylinder(radius=0.005, length=0.038),
                origin=Origin(xyz=(pull_x, -door_thick / 2.0 - 0.019, post_z), rpy=(pi / 2.0, 0.0, 0.0)),
                material=dark_metal,
                name=f"pull_post_{post_index}",
            )
        door.visual(
            Cylinder(radius=0.007, length=0.170),
            origin=Origin(xyz=(pull_x, pull_y, height / 2.0)),
            material=dark_metal,
            name="pull_grip",
        )
        for knuckle_index, z in enumerate((0.18, 0.47)):
            door.visual(
                Box((0.020, 0.004, 0.070)),
                origin=Origin(xyz=(0.0, -door_thick / 2.0 - 0.002, z)),
                material=brass,
                name=f"door_hinge_leaf_{knuckle_index}",
            )
            door.visual(
                Cylinder(radius=hinge_radius, length=0.085),
                origin=Origin(xyz=(0.0, -0.003, z)),
                material=brass,
                name=f"door_hinge_barrel_{knuckle_index}",
            )

        model.articulation(
            f"cabinet_to_{name}",
            ArticulationType.REVOLUTE,
            parent=cabinet,
            child=door,
            origin=Origin(xyz=(hinge_x, door_center_y, 0.0)),
            axis=hinge_axis,
            motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.70),
        )
        return door

    add_door(
        "door_0",
        hinge_x=-width / 2.0,
        panel_center_x=door_width / 2.0,
        free_edge_x=door_width - 0.035,
        hinge_axis=(0.0, 0.0, -1.0),
    )
    add_door(
        "door_1",
        hinge_x=width / 2.0,
        panel_center_x=-door_width / 2.0,
        free_edge_x=-door_width + 0.035,
        hinge_axis=(0.0, 0.0, 1.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    door_0 = object_model.get_part("door_0")
    door_1 = object_model.get_part("door_1")
    hinge_0 = object_model.get_articulation("cabinet_to_door_0")
    hinge_1 = object_model.get_articulation("cabinet_to_door_1")

    ctx.expect_gap(
        cabinet,
        door_0,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        negative_elem="door_panel",
        name="door_0 closes flush on cabinet front",
    )
    ctx.expect_gap(
        cabinet,
        door_1,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        negative_elem="door_panel",
        name="door_1 closes flush on cabinet front",
    )
    ctx.expect_gap(
        door_1,
        door_0,
        axis="x",
        min_gap=0.004,
        max_gap=0.012,
        positive_elem="door_panel",
        negative_elem="door_panel",
        name="closed doors keep a narrow center reveal",
    )
    ctx.check(
        "hinge axes are vertical and mirrored",
        tuple(hinge_0.axis) == (0.0, 0.0, -1.0) and tuple(hinge_1.axis) == (0.0, 0.0, 1.0),
        details=f"axis0={hinge_0.axis}, axis1={hinge_1.axis}",
    )

    closed_0 = ctx.part_element_world_aabb(door_0, elem="door_panel")
    closed_1 = ctx.part_element_world_aabb(door_1, elem="door_panel")
    with ctx.pose({hinge_0: 1.20, hinge_1: 1.20}):
        open_0 = ctx.part_element_world_aabb(door_0, elem="door_panel")
        open_1 = ctx.part_element_world_aabb(door_1, elem="door_panel")
    ctx.check(
        "both doors swing outward at their upper limits",
        closed_0 is not None
        and closed_1 is not None
        and open_0 is not None
        and open_1 is not None
        and open_0[0][1] < closed_0[0][1] - 0.10
        and open_1[0][1] < closed_1[0][1] - 0.10,
        details=f"closed0={closed_0}, open0={open_0}, closed1={closed_1}, open1={open_1}",
    )

    return ctx.report()


object_model = build_object_model()
