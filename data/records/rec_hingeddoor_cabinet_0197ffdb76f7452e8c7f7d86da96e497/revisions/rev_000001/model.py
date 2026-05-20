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
    model = ArticulatedObject(name="tall_utility_cabinet")

    powder_gray = model.material("powder_gray", rgba=(0.55, 0.58, 0.58, 1.0))
    door_blue = model.material("door_blue_gray", rgba=(0.30, 0.38, 0.44, 1.0))
    shelf_gray = model.material("shelf_gray", rgba=(0.68, 0.70, 0.68, 1.0))
    hinge_metal = model.material("zinc_hinge", rgba=(0.78, 0.76, 0.70, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.03, 0.03, 0.03, 1.0))

    width = 0.75
    depth = 0.45
    height = 1.80
    wall = 0.035
    foot_h = 0.08
    z_mid = foot_h + height / 2.0
    front_y = -depth / 2.0
    back_y = depth / 2.0

    carcass = model.part("carcass")
    carcass.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(-width / 2.0 + wall / 2.0, 0.0, z_mid)),
        material=powder_gray,
        name="side_wall_0",
    )
    carcass.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(width / 2.0 - wall / 2.0, 0.0, z_mid)),
        material=powder_gray,
        name="side_wall_1",
    )
    carcass.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, foot_h + wall / 2.0)),
        material=powder_gray,
        name="bottom_panel",
    )
    carcass.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, foot_h + height - wall / 2.0)),
        material=powder_gray,
        name="top_panel",
    )
    carcass.visual(
        Box((width, wall, height)),
        origin=Origin(xyz=(0.0, back_y - wall / 2.0, z_mid)),
        material=powder_gray,
        name="back_panel",
    )

    inner_width = width - 2.0 * wall
    shelf_depth = depth - wall
    shelf_y = front_y + shelf_depth / 2.0
    for idx, z in enumerate((0.43, 0.78, 1.13, 1.48)):
        carcass.visual(
            Box((inner_width, shelf_depth, 0.024)),
            origin=Origin(xyz=(0.0, shelf_y, z)),
            material=shelf_gray,
            name=f"shelf_{idx}",
        )

    # Narrow front lips make the open carcass read as a formed metal enclosure.
    carcass.visual(
        Box((0.030, 0.012, height)),
        origin=Origin(xyz=(-width / 2.0 + 0.015, front_y - 0.002, z_mid)),
        material=powder_gray,
        name="front_stile_0",
    )
    carcass.visual(
        Box((0.030, 0.012, height)),
        origin=Origin(xyz=(width / 2.0 - 0.015, front_y - 0.002, z_mid)),
        material=powder_gray,
        name="front_stile_1",
    )

    for idx, (x, y) in enumerate(
        (
            (-width / 2.0 + 0.08, front_y + 0.08),
            (width / 2.0 - 0.08, front_y + 0.08),
            (-width / 2.0 + 0.08, back_y - 0.08),
            (width / 2.0 - 0.08, back_y - 0.08),
        )
    ):
        carcass.visual(
            Cylinder(radius=0.032, length=foot_h),
            origin=Origin(xyz=(x, y, foot_h / 2.0)),
            material=black_rubber,
            name=f"foot_{idx}",
        )

    hinge_x = -width / 2.0 - 0.035
    hinge_y = front_y - 0.024
    hinge_radius = 0.012
    hinge_len = 0.260
    knuckle_gap = 0.006
    knuckle_len = (hinge_len - 4.0 * knuckle_gap) / 5.0

    def add_parent_hinge(prefix: str, zc: float) -> None:
        z0 = zc - hinge_len / 2.0
        carcass.visual(
            Box((0.004, 0.060, hinge_len)),
            origin=Origin(xyz=(-width / 2.0 - 0.002, hinge_y + 0.016, zc)),
            material=hinge_metal,
            name=f"{prefix}_frame_leaf",
        )
        for i in (0, 2, 4):
            z = z0 + i * (knuckle_len + knuckle_gap) + knuckle_len / 2.0
            carcass.visual(
                Cylinder(radius=hinge_radius, length=knuckle_len),
                origin=Origin(xyz=(hinge_x, hinge_y, z)),
                material=hinge_metal,
                name=f"{prefix}_frame_knuckle_{i}",
            )
            carcass.visual(
                Box((0.026, 0.006, knuckle_len)),
                origin=Origin(xyz=(hinge_x + 0.023, hinge_y, z)),
                material=hinge_metal,
                name=f"{prefix}_frame_web_{i}",
            )

    for name, zc in (("lower_hinge", 0.48), ("upper_hinge", 1.48)):
        add_parent_hinge(name, zc)

    door = model.part("door")
    door_width = 0.760
    door_height = 1.780
    door_thick = 0.028
    hinge_to_door_edge = 0.045
    door_panel_center_x = hinge_to_door_edge + door_width / 2.0
    door_panel_center_y = 0.004
    door_center_z = foot_h + height / 2.0
    door.visual(
        Box((door_width, door_thick, door_height)),
        origin=Origin(xyz=(door_panel_center_x, door_panel_center_y, door_center_z)),
        material=door_blue,
        name="door_panel",
    )

    # Slightly proud stamped border and recessed-looking field on the front face.
    trim_y = door_panel_center_y - door_thick / 2.0 - 0.002
    trim_z = door_center_z
    door.visual(
        Box((door_width - 0.090, 0.006, 0.035)),
        origin=Origin(xyz=(door_panel_center_x + 0.010, trim_y, foot_h + height - 0.090)),
        material=door_blue,
        name="top_door_rail",
    )
    door.visual(
        Box((door_width - 0.090, 0.006, 0.035)),
        origin=Origin(xyz=(door_panel_center_x + 0.010, trim_y, foot_h + 0.090)),
        material=door_blue,
        name="bottom_door_rail",
    )
    door.visual(
        Box((0.035, 0.006, door_height - 0.180)),
        origin=Origin(xyz=(hinge_to_door_edge + 0.055, trim_y, trim_z)),
        material=door_blue,
        name="hinge_door_rail",
    )
    door.visual(
        Box((0.035, 0.006, door_height - 0.180)),
        origin=Origin(xyz=(hinge_to_door_edge + door_width - 0.055, trim_y, trim_z)),
        material=door_blue,
        name="pull_door_rail",
    )

    # Pull handle fixed to the door with two small standoffs.
    handle_x = hinge_to_door_edge + door_width - 0.100
    handle_y = trim_y - 0.042
    door.visual(
        Cylinder(radius=0.011, length=0.420),
        origin=Origin(xyz=(handle_x, handle_y, door_center_z)),
        material=hinge_metal,
        name="pull_handle",
    )
    for idx, z in enumerate((door_center_z - 0.160, door_center_z + 0.160)):
        door.visual(
            Box((0.030, 0.046, 0.020)),
            origin=Origin(xyz=(handle_x, (handle_y + trim_y) / 2.0, z)),
            material=hinge_metal,
            name=f"handle_standoff_{idx}",
        )

    def add_door_hinge(prefix: str, zc: float) -> None:
        z0 = zc - hinge_len / 2.0
        door.visual(
            Box((0.090, 0.005, hinge_len)),
            origin=Origin(xyz=(hinge_to_door_edge + 0.045, trim_y, zc)),
            material=hinge_metal,
            name=f"{prefix}_door_leaf",
        )
        for i in (1, 3):
            z = z0 + i * (knuckle_len + knuckle_gap) + knuckle_len / 2.0
            door.visual(
                Cylinder(radius=hinge_radius, length=knuckle_len),
                origin=Origin(xyz=(0.0, 0.0, z)),
                material=hinge_metal,
                name=f"{prefix}_door_knuckle_{i}",
            )
            door.visual(
                Box((0.037, 0.006, knuckle_len)),
                origin=Origin(xyz=(0.0285, trim_y / 2.0, z)),
                material=hinge_metal,
                name=f"{prefix}_door_web_{i}",
            )

    for name, zc in (("lower_hinge", 0.48), ("upper_hinge", 1.48)):
        add_door_hinge(name, zc)

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.5, lower=0.0, upper=math.radians(120.0)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")
    door = object_model.get_part("door")
    hinge = object_model.get_articulation("door_hinge")

    ctx.check(
        "single door hinge swings 120 degrees",
        hinge.motion_limits is not None
        and hinge.motion_limits.lower == 0.0
        and abs(hinge.motion_limits.upper - math.radians(120.0)) < 1.0e-6,
        details=f"limits={getattr(hinge, 'motion_limits', None)}",
    )
    ctx.expect_gap(
        carcass,
        door,
        axis="y",
        min_gap=0.004,
        max_gap=0.010,
        positive_elem="bottom_panel",
        negative_elem="door_panel",
        name="closed door has a small reveal in front of the cabinet",
    )
    ctx.expect_overlap(
        door,
        carcass,
        axes="xz",
        min_overlap=0.70,
        elem_a="door_panel",
        elem_b="back_panel",
        name="full-height door covers the cabinet opening",
    )

    closed_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({hinge: math.radians(120.0)}):
        open_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
        ctx.expect_gap(
            carcass,
            door,
            axis="y",
            min_gap=0.020,
            positive_elem="front_stile_0",
            negative_elem="door_panel",
            name="open door clears the hinge-side front stile",
        )

    ctx.check(
        "upper-limit pose moves the free edge outward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][1] < closed_aabb[0][1] - 0.45,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )
    return ctx.report()


object_model = build_object_model()
