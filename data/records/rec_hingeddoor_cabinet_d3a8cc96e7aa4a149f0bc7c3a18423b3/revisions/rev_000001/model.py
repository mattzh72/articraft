from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tall_utility_cabinet")

    powder_coat = model.material("warm_gray_powder_coat", color=(0.62, 0.64, 0.62, 1.0))
    dark_recess = model.material("shadowed_interior", color=(0.12, 0.13, 0.13, 1.0))
    black_rubber = model.material("black_rubber", color=(0.025, 0.025, 0.022, 1.0))
    hinge_steel = model.material("brushed_hinge_steel", color=(0.68, 0.69, 0.67, 1.0))
    handle_black = model.material("black_latch_handle", color=(0.02, 0.022, 0.024, 1.0))

    width = 0.75
    depth = 0.45
    body_height = 1.80
    wall = 0.035
    foot_height = 0.08
    body_bottom = foot_height
    body_top = body_bottom + body_height
    front_y = -depth / 2.0
    back_y = depth / 2.0

    hinge_x = -width / 2.0 - 0.018
    hinge_y = front_y - 0.025
    door_bottom = body_bottom + 0.040
    door_height = body_height - 0.080
    door_thickness = 0.040
    door_width = width
    door_left_offset = 0.030

    carcass = model.part("carcass")
    carcass.visual(
        Box((wall, depth, body_height)),
        origin=Origin(xyz=(-width / 2.0 + wall / 2.0, 0.0, body_bottom + body_height / 2.0)),
        material=powder_coat,
        name="left_wall",
    )
    carcass.visual(
        Box((wall, depth, body_height)),
        origin=Origin(xyz=(width / 2.0 - wall / 2.0, 0.0, body_bottom + body_height / 2.0)),
        material=powder_coat,
        name="side_wall",
    )
    carcass.visual(
        Box((width, wall, body_height)),
        origin=Origin(xyz=(0.0, back_y - wall / 2.0, body_bottom + body_height / 2.0)),
        material=dark_recess,
        name="back_panel",
    )
    carcass.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, body_top - wall / 2.0)),
        material=powder_coat,
        name="top_panel",
    )
    carcass.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, body_bottom + wall / 2.0)),
        material=powder_coat,
        name="bottom_panel",
    )
    carcass.visual(
        Box((0.050, 0.032, body_height)),
        origin=Origin(xyz=(-width / 2.0 + 0.025, front_y + 0.016, body_bottom + body_height / 2.0)),
        material=powder_coat,
        name="hinge_jamb",
    )
    carcass.visual(
        Box((0.050, 0.032, body_height)),
        origin=Origin(xyz=(width / 2.0 - 0.025, front_y + 0.016, body_bottom + body_height / 2.0)),
        material=powder_coat,
        name="latch_jamb",
    )
    carcass.visual(
        Box((width, 0.032, 0.052)),
        origin=Origin(xyz=(0.0, front_y + 0.016, body_top - 0.026)),
        material=powder_coat,
        name="top_front_rail",
    )
    carcass.visual(
        Box((width, 0.032, 0.052)),
        origin=Origin(xyz=(0.0, front_y + 0.016, body_bottom + 0.026)),
        material=powder_coat,
        name="bottom_front_rail",
    )

    for index, x in enumerate((-0.285, 0.285)):
        for y in (-0.165, 0.165):
            carcass.visual(
                Box((0.085, 0.085, foot_height + 0.008)),
                origin=Origin(xyz=(x, y, foot_height / 2.0 - 0.004)),
                material=black_rubber,
                name=f"foot_{index}_{0 if y < 0.0 else 1}",
            )

    hinge_centers = (door_bottom + 0.25, door_bottom + door_height / 2.0, door_bottom + door_height - 0.25)
    for hinge_index, zc in enumerate(hinge_centers):
        for segment_index, dz in enumerate((-0.080, 0.080)):
            carcass.visual(
                Cylinder(radius=0.018, length=0.070),
                origin=Origin(xyz=(hinge_x, hinge_y, zc + dz)),
                material=hinge_steel,
                name=f"fixed_hinge_barrel_{hinge_index}_{segment_index}",
            )
            carcass.visual(
                Box((0.006, 0.092, 0.080)),
                origin=Origin(xyz=(-width / 2.0 - 0.003, front_y - 0.006, zc + dz)),
                material=hinge_steel,
                name=f"fixed_hinge_leaf_{hinge_index}_{segment_index}",
            )

    door = model.part("door")
    door.visual(
        Box((door_width, door_thickness, door_height)),
        origin=Origin(xyz=(door_left_offset + door_width / 2.0, 0.0, door_height / 2.0)),
        material=powder_coat,
        name="door_slab",
    )
    door.visual(
        Box((door_width - 0.060, 0.010, 0.050)),
        origin=Origin(xyz=(door_left_offset + door_width / 2.0, -door_thickness / 2.0 - 0.005, door_height - 0.060)),
        material=powder_coat,
        name="top_door_rail",
    )
    door.visual(
        Box((door_width - 0.060, 0.010, 0.050)),
        origin=Origin(xyz=(door_left_offset + door_width / 2.0, -door_thickness / 2.0 - 0.005, 0.060)),
        material=powder_coat,
        name="bottom_door_rail",
    )
    door.visual(
        Box((0.048, 0.010, door_height - 0.130)),
        origin=Origin(xyz=(door_left_offset + 0.050, -door_thickness / 2.0 - 0.005, door_height / 2.0)),
        material=powder_coat,
        name="hinge_door_stile",
    )
    door.visual(
        Box((0.048, 0.010, door_height - 0.130)),
        origin=Origin(xyz=(door_left_offset + door_width - 0.050, -door_thickness / 2.0 - 0.005, door_height / 2.0)),
        material=powder_coat,
        name="latch_door_stile",
    )
    door.visual(
        Box((door_width - 0.190, 0.006, door_height - 0.260)),
        origin=Origin(xyz=(door_left_offset + door_width / 2.0, -door_thickness / 2.0 - 0.010, door_height / 2.0)),
        material=dark_recess,
        name="recessed_field",
    )
    door.visual(
        Box((0.110, 0.006, 0.220)),
        origin=Origin(xyz=(door_left_offset + door_width - 0.085, -door_thickness / 2.0 - 0.007, door_height / 2.0)),
        material=hinge_steel,
        name="latch_backplate",
    )

    for hinge_index, zc_world in enumerate(hinge_centers):
        zc = zc_world - door_bottom
        door.visual(
            Cylinder(radius=0.018, length=0.080),
            origin=Origin(xyz=(0.0, 0.0, zc)),
            material=hinge_steel,
            name=f"door_hinge_barrel_{hinge_index}",
        )
        door.visual(
            Box((0.095, 0.012, 0.090)),
            origin=Origin(xyz=(0.040, -door_thickness / 2.0 - 0.002, zc)),
            material=hinge_steel,
            name=f"door_hinge_leaf_{hinge_index}",
        )

    door_joint = model.articulation(
        "carcass_to_door",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, door_bottom)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=1.75),
    )

    handle_pivot_x = door_left_offset + door_width - 0.085
    handle_pivot_z = door_height / 2.0
    handle = model.part("latch_handle")
    handle.visual(
        Cylinder(radius=0.045, length=0.025),
        origin=Origin(xyz=(0.0, 0.0035, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=handle_black,
        name="round_hub",
    )
    handle.visual(
        Box((0.035, 0.030, 0.280)),
        origin=Origin(xyz=(0.0, -0.020, 0.0)),
        material=handle_black,
        name="lever_grip",
    )
    handle.visual(
        Box((0.090, 0.026, 0.032)),
        origin=Origin(xyz=(0.0, -0.038, 0.110)),
        material=handle_black,
        name="upper_finger_pull",
    )
    handle.visual(
        Box((0.090, 0.026, 0.032)),
        origin=Origin(xyz=(0.0, -0.038, -0.110)),
        material=handle_black,
        name="lower_finger_pull",
    )

    model.articulation(
        "door_to_latch_handle",
        ArticulationType.REVOLUTE,
        parent=door,
        child=handle,
        origin=Origin(xyz=(handle_pivot_x, -door_thickness / 2.0 - 0.026, handle_pivot_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=4.0, lower=-pi / 2.0, upper=pi / 2.0),
    )

    door_joint.meta["description"] = "Full-height cabinet door swings outward about the exposed vertical hinge line."
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")
    door = object_model.get_part("door")
    handle = object_model.get_part("latch_handle")
    door_hinge = object_model.get_articulation("carcass_to_door")
    latch_pivot = object_model.get_articulation("door_to_latch_handle")

    ctx.expect_gap(
        carcass,
        door,
        axis="y",
        min_gap=0.003,
        max_gap=0.012,
        positive_elem="left_wall",
        negative_elem="door_slab",
        name="closed door sits just proud of cabinet opening",
    )
    ctx.expect_overlap(
        door,
        carcass,
        axes="xz",
        min_overlap=0.62,
        elem_a="door_slab",
        elem_b="back_panel",
        name="full-height door covers the enclosure footprint",
    )
    ctx.expect_contact(
        handle,
        door,
        elem_a="round_hub",
        elem_b="latch_backplate",
        contact_tol=0.002,
        name="latch handle hub is seated on its backplate",
    )

    closed_door_aabb = ctx.part_element_world_aabb(door, elem="door_slab")
    with ctx.pose({door_hinge: 1.25}):
        open_door_aabb = ctx.part_element_world_aabb(door, elem="door_slab")
    closed_y = None if closed_door_aabb is None else (closed_door_aabb[0][1] + closed_door_aabb[1][1]) / 2.0
    open_y = None if open_door_aabb is None else (open_door_aabb[0][1] + open_door_aabb[1][1]) / 2.0
    ctx.check(
        "door swings outward from the front",
        closed_y is not None and open_y is not None and open_y < closed_y - 0.25,
        details=f"closed_y={closed_y}, open_y={open_y}",
    )

    vertical_aabb = ctx.part_element_world_aabb(handle, elem="lever_grip")
    with ctx.pose({latch_pivot: pi / 2.0}):
        turned_aabb = ctx.part_element_world_aabb(handle, elem="lever_grip")
    vertical_dx = None if vertical_aabb is None else vertical_aabb[1][0] - vertical_aabb[0][0]
    vertical_dz = None if vertical_aabb is None else vertical_aabb[1][2] - vertical_aabb[0][2]
    turned_dx = None if turned_aabb is None else turned_aabb[1][0] - turned_aabb[0][0]
    turned_dz = None if turned_aabb is None else turned_aabb[1][2] - turned_aabb[0][2]
    ctx.check(
        "latch handle rotates on its own face pivot",
        (
            vertical_dx is not None
            and vertical_dz is not None
            and turned_dx is not None
            and turned_dz is not None
            and vertical_dz > vertical_dx * 5.0
            and turned_dx > turned_dz * 5.0
        ),
        details=f"vertical_dx={vertical_dx}, vertical_dz={vertical_dz}, turned_dx={turned_dx}, turned_dz={turned_dz}",
    )

    return ctx.report()


object_model = build_object_model()
