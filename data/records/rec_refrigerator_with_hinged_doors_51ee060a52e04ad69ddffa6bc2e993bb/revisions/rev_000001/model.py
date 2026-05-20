from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _rounded_door_panel_mesh(name: str, *, width: float, height: float, thickness: float):
    """Return a softly radiused refrigerator door slab, centered on local X/Y/Z."""
    profile = rounded_rect_profile(width, height, radius=0.035, corner_segments=8)
    geom = ExtrudeGeometry(profile, thickness, center=True)
    # ExtrudeGeometry builds thickness along local Z. Rotate so that the panel
    # face spans local X/Z and the door thickness runs along local Y.
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="top_freezer_refrigerator")

    painted_case = model.material("painted_case", rgba=(0.82, 0.84, 0.84, 1.0))
    stainless = model.material("brushed_stainless", rgba=(0.70, 0.73, 0.74, 1.0))
    dark_liner = model.material("dark_liner", rgba=(0.10, 0.11, 0.12, 1.0))
    white_liner = model.material("white_liner", rgba=(0.90, 0.92, 0.92, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.03, 0.035, 0.04, 1.0))
    handle_metal = model.material("handle_metal", rgba=(0.56, 0.58, 0.59, 1.0))

    # Overall domestic refrigerator proportions in meters.
    cabinet_width = 0.80
    cabinet_depth = 0.68
    cabinet_height = 1.75
    wall = 0.045
    front_y = -cabinet_depth / 2.0
    back_y = cabinet_depth / 2.0
    door_width = 0.76
    door_thickness = 0.075
    hinge_x = -cabinet_width / 2.0
    hinge_y = front_y - 0.045

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((wall, cabinet_depth, cabinet_height)),
        origin=Origin(xyz=(-cabinet_width / 2.0 + wall / 2.0, 0.0, cabinet_height / 2.0)),
        material=painted_case,
        name="hinge_side_wall",
    )
    cabinet.visual(
        Box((wall, cabinet_depth, cabinet_height)),
        origin=Origin(xyz=(cabinet_width / 2.0 - wall / 2.0, 0.0, cabinet_height / 2.0)),
        material=painted_case,
        name="opening_side_wall",
    )
    cabinet.visual(
        Box((cabinet_width, wall, cabinet_height)),
        origin=Origin(xyz=(0.0, back_y - wall / 2.0, cabinet_height / 2.0)),
        material=painted_case,
        name="back_wall",
    )
    cabinet.visual(
        Box((cabinet_width, cabinet_depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, cabinet_height - wall / 2.0)),
        material=painted_case,
        name="top_cap",
    )
    cabinet.visual(
        Box((cabinet_width, cabinet_depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, wall / 2.0)),
        material=painted_case,
        name="bottom_cap",
    )
    cabinet.visual(
        Box((cabinet_width, cabinet_depth, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 1.205)),
        material=painted_case,
        name="center_divider",
    )
    cabinet.visual(
        Box((0.70, 0.018, 1.05)),
        origin=Origin(xyz=(0.0, back_y - wall - 0.006, 0.625)),
        material=white_liner,
        name="food_liner_back",
    )
    cabinet.visual(
        Box((0.70, 0.018, 0.43)),
        origin=Origin(xyz=(0.0, back_y - wall - 0.006, 1.465)),
        material=white_liner,
        name="freezer_liner_back",
    )
    for index, z in enumerate((0.48, 0.78, 1.03)):
        cabinet.visual(
            Box((0.710, 0.580, 0.018)),
            origin=Origin(xyz=(0.0, 0.020, z)),
            material=white_liner,
            name=f"food_shelf_{index}",
        )
    cabinet.visual(
        Box((0.68, 0.020, 0.060)),
        origin=Origin(xyz=(0.0, front_y - 0.002, 0.045)),
        material=dark_liner,
        name="toe_recess",
    )
    for index, z in enumerate((0.028, 0.046, 0.064)):
        cabinet.visual(
            Box((0.58, 0.012, 0.006)),
            origin=Origin(xyz=(0.0, front_y - 0.010, z)),
            material=black_rubber,
            name=f"toe_grille_slat_{index}",
        )
    for index, z in enumerate((0.30, 0.94, 1.37, 1.63)):
        cabinet.visual(
            Box((0.050, 0.026, 0.105)),
            origin=Origin(xyz=(-cabinet_width / 2.0 - 0.020, hinge_y + 0.033, z)),
            material=handle_metal,
            name=f"hinge_plate_{index}",
        )

    def add_door(
        name: str,
        *,
        lower_z: float,
        height: float,
        handle_height: float,
        handle_center_z: float,
        hinge_segments: tuple[tuple[float, float], ...],
    ):
        door = model.part(name)
        door.visual(
            _rounded_door_panel_mesh(f"{name}_panel_mesh", width=door_width, height=height, thickness=door_thickness),
            origin=Origin(xyz=(door_width / 2.0 + 0.005, 0.0, height / 2.0)),
            material=stainless,
            name="door_panel",
        )
        door.visual(
            Box((0.018, 0.010, height - 0.060)),
            origin=Origin(xyz=(door_width - 0.018, door_thickness / 2.0 + 0.001, height / 2.0)),
            material=black_rubber,
            name="magnetic_gasket",
        )
        handle_x = door_width - 0.070
        door.visual(
            Box((0.034, 0.034, handle_height)),
            origin=Origin(xyz=(handle_x, -0.071, handle_center_z)),
            material=handle_metal,
            name="handle_grip",
        )
        for support_index, support_z in enumerate((handle_center_z - handle_height * 0.38, handle_center_z + handle_height * 0.38)):
            door.visual(
                Box((0.052, 0.056, 0.070)),
                origin=Origin(xyz=(handle_x, -0.046, support_z)),
                material=handle_metal,
                name=f"handle_post_{support_index}",
            )
        for segment_index, (segment_center_z, segment_height) in enumerate(hinge_segments):
            door.visual(
                Cylinder(radius=0.020, length=segment_height),
                origin=Origin(xyz=(0.0, 0.0, segment_center_z)),
                material=handle_metal,
                name=f"hinge_barrel_{segment_index}",
            )
        model.articulation(
            f"cabinet_to_{name}",
            ArticulationType.REVOLUTE,
            parent=cabinet,
            child=door,
            origin=Origin(xyz=(hinge_x, hinge_y, lower_z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=45.0, velocity=1.6, lower=0.0, upper=2.05),
        )
        return door

    add_door(
        "food_door",
        lower_z=0.095,
        height=1.075,
        handle_height=0.68,
        handle_center_z=0.56,
        hinge_segments=((0.22, 0.17), (0.84, 0.18)),
    )
    add_door(
        "freezer_door",
        lower_z=1.235,
        height=0.455,
        handle_height=0.265,
        handle_center_z=0.225,
        hinge_segments=((0.115, 0.105), (0.345, 0.105)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    food_door = object_model.get_part("food_door")
    freezer_door = object_model.get_part("freezer_door")
    food_hinge = object_model.get_articulation("cabinet_to_food_door")
    freezer_hinge = object_model.get_articulation("cabinet_to_freezer_door")

    ctx.expect_gap(
        cabinet,
        food_door,
        axis="y",
        min_gap=0.002,
        max_gap=0.012,
        positive_elem="hinge_side_wall",
        negative_elem="door_panel",
        name="food door sits just in front of cabinet",
    )
    ctx.expect_gap(
        cabinet,
        freezer_door,
        axis="y",
        min_gap=0.002,
        max_gap=0.012,
        positive_elem="hinge_side_wall",
        negative_elem="door_panel",
        name="freezer door sits just in front of cabinet",
    )

    food_aabb = ctx.part_world_aabb(food_door)
    freezer_aabb = ctx.part_world_aabb(freezer_door)
    if food_aabb is not None and freezer_aabb is not None:
        food_size = tuple(float(food_aabb[1][i] - food_aabb[0][i]) for i in range(3))
        freezer_size = tuple(float(freezer_aabb[1][i] - freezer_aabb[0][i]) for i in range(3))
        ctx.check(
            "freezer door is above fresh food door",
            freezer_aabb[0][2] > food_aabb[1][2] + 0.040,
            details=f"food={food_aabb}, freezer={freezer_aabb}",
        )
        ctx.check(
            "freezer door is smaller than fresh food door",
            freezer_size[2] < food_size[2] * 0.55,
            details=f"food_size={food_size}, freezer_size={freezer_size}",
        )

    ctx.check(
        "both hinge axes are vertical and parallel",
        food_hinge.axis == freezer_hinge.axis == (0.0, 0.0, -1.0),
        details=f"food_axis={food_hinge.axis}, freezer_axis={freezer_hinge.axis}",
    )
    ctx.check(
        "doors hinge from same cabinet side",
        abs(food_hinge.origin.xyz[0] - freezer_hinge.origin.xyz[0]) < 1e-6
        and abs(food_hinge.origin.xyz[1] - freezer_hinge.origin.xyz[1]) < 1e-6,
        details=f"food_origin={food_hinge.origin.xyz}, freezer_origin={freezer_hinge.origin.xyz}",
    )

    rest_food_aabb = ctx.part_world_aabb(food_door)
    rest_freezer_aabb = ctx.part_world_aabb(freezer_door)
    with ctx.pose({food_hinge: 1.05, freezer_hinge: 1.05}):
        open_food_aabb = ctx.part_world_aabb(food_door)
        open_freezer_aabb = ctx.part_world_aabb(freezer_door)
    if rest_food_aabb is not None and open_food_aabb is not None:
        ctx.check(
            "food door rotates outward",
            open_food_aabb[0][1] < rest_food_aabb[0][1] - 0.25,
            details=f"rest={rest_food_aabb}, open={open_food_aabb}",
        )
    if rest_freezer_aabb is not None and open_freezer_aabb is not None:
        ctx.check(
            "freezer door rotates outward",
            open_freezer_aabb[0][1] < rest_freezer_aabb[0][1] - 0.15,
            details=f"rest={rest_freezer_aabb}, open={open_freezer_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
