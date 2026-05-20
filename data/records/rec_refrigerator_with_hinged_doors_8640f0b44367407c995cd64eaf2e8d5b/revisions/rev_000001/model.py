from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pantry_refrigerator")

    height = 1.86
    body_width = 0.82
    body_depth = 0.60
    shell_thickness = 0.025
    divider_thickness = 0.02
    door_thickness = 0.05
    door_leaf_width = body_width / 2.0 - 0.003
    handle_bar_radius = 0.009
    handle_standoff_radius = 0.007
    handle_bar_length = 1.02
    handle_standoff_length = 0.045

    left_inner_min_x = -body_width / 2.0 + shell_thickness
    left_inner_max_x = -divider_thickness / 2.0
    right_inner_min_x = divider_thickness / 2.0
    right_inner_max_x = body_width / 2.0 - shell_thickness
    compartment_width = left_inner_max_x - left_inner_min_x

    shelf_depth = 0.44
    shelf_center_y = (-body_depth / 2.0 + shell_thickness) + shelf_depth / 2.0
    shelf_levels = (0.38, 0.74, 1.10, 1.46)

    stainless = model.material("stainless", rgba=(0.77, 0.79, 0.81, 1.0))
    cabinet_white = model.material("cabinet_white", rgba=(0.93, 0.94, 0.95, 1.0))
    warm_white = model.material("warm_white", rgba=(0.96, 0.96, 0.94, 1.0))
    gasket_gray = model.material("gasket_gray", rgba=(0.72, 0.74, 0.76, 1.0))
    rail_silver = model.material("rail_silver", rgba=(0.84, 0.85, 0.87, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((shell_thickness, body_depth, height)),
        origin=Origin(xyz=(-body_width / 2.0 + shell_thickness / 2.0, 0.0, height / 2.0)),
        material=cabinet_white,
        name="left_wall",
    )
    cabinet.visual(
        Box((shell_thickness, body_depth, height)),
        origin=Origin(xyz=(body_width / 2.0 - shell_thickness / 2.0, 0.0, height / 2.0)),
        material=cabinet_white,
        name="right_wall",
    )
    cabinet.visual(
        Box((body_width, body_depth, shell_thickness)),
        origin=Origin(xyz=(0.0, 0.0, shell_thickness / 2.0)),
        material=cabinet_white,
        name="cabinet_floor",
    )
    cabinet.visual(
        Box((body_width, body_depth, shell_thickness)),
        origin=Origin(xyz=(0.0, 0.0, height - shell_thickness / 2.0)),
        material=cabinet_white,
        name="cabinet_roof",
    )
    cabinet.visual(
        Box((body_width, shell_thickness, height)),
        origin=Origin(xyz=(0.0, -body_depth / 2.0 + shell_thickness / 2.0, height / 2.0)),
        material=warm_white,
        name="back_panel",
    )
    cabinet.visual(
        Box((divider_thickness, body_depth, height)),
        origin=Origin(xyz=(0.0, 0.0, height / 2.0)),
        material=warm_white,
        name="center_divider",
    )

    for index, shelf_z in enumerate(shelf_levels):
        cabinet.visual(
            Box((compartment_width, shelf_depth, 0.012)),
            origin=Origin(
                xyz=(
                    (left_inner_min_x + left_inner_max_x) / 2.0,
                    shelf_center_y,
                    shelf_z,
                )
            ),
            material=warm_white,
            name=f"left_shelf_{index + 1}",
        )
        cabinet.visual(
            Box((compartment_width, shelf_depth, 0.012)),
            origin=Origin(
                xyz=(
                    (right_inner_min_x + right_inner_max_x) / 2.0,
                    shelf_center_y,
                    shelf_z,
                )
            ),
            material=warm_white,
            name=f"right_shelf_{index + 1}",
        )

    left_door = model.part("left_door")
    left_door.visual(
        Box((door_leaf_width, door_thickness, height)),
        origin=Origin(xyz=(door_leaf_width / 2.0, door_thickness / 2.0, height / 2.0)),
        material=stainless,
        name="left_door_panel",
    )
    gasket_width = door_leaf_width - 0.06

    left_door.visual(
        Box((gasket_width, 0.01, height - 0.04)),
        origin=Origin(xyz=(0.03 + gasket_width / 2.0, -0.005, height / 2.0)),
        material=gasket_gray,
        name="left_gasket",
    )
    left_door.visual(
        Cylinder(radius=handle_bar_radius, length=handle_bar_length),
        origin=Origin(
            xyz=(door_leaf_width - 0.07, door_thickness + 0.04, 0.98)
        ),
        material=stainless,
        name="left_handle_bar",
    )
    left_door.visual(
        Cylinder(radius=handle_standoff_radius, length=handle_standoff_length),
        origin=Origin(
            xyz=(
                door_leaf_width - 0.07,
                door_thickness + handle_standoff_length / 2.0,
                0.98 - handle_bar_length / 2.0 + 0.075,
            ),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=stainless,
        name="left_handle_standoff_top",
    )
    left_door.visual(
        Cylinder(radius=handle_standoff_radius, length=handle_standoff_length),
        origin=Origin(
            xyz=(
                door_leaf_width - 0.07,
                door_thickness + handle_standoff_length / 2.0,
                0.98 + handle_bar_length / 2.0 - 0.075,
            ),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=stainless,
        name="left_handle_standoff_bottom",
    )

    right_door = model.part("right_door")
    right_door.visual(
        Box((door_leaf_width, door_thickness, height)),
        origin=Origin(xyz=(-door_leaf_width / 2.0, door_thickness / 2.0, height / 2.0)),
        material=stainless,
        name="right_door_panel",
    )
    right_door.visual(
        Box((gasket_width, 0.01, height - 0.04)),
        origin=Origin(xyz=(-(0.03 + gasket_width / 2.0), -0.005, height / 2.0)),
        material=gasket_gray,
        name="right_gasket",
    )
    right_door.visual(
        Cylinder(radius=handle_bar_radius, length=handle_bar_length),
        origin=Origin(
            xyz=(-(door_leaf_width - 0.07), door_thickness + 0.04, 0.98)
        ),
        material=stainless,
        name="right_handle_bar",
    )
    right_door.visual(
        Cylinder(radius=handle_standoff_radius, length=handle_standoff_length),
        origin=Origin(
            xyz=(
                -(door_leaf_width - 0.07),
                door_thickness + handle_standoff_length / 2.0,
                0.98 - handle_bar_length / 2.0 + 0.075,
            ),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=stainless,
        name="right_handle_standoff_top",
    )
    right_door.visual(
        Cylinder(radius=handle_standoff_radius, length=handle_standoff_length),
        origin=Origin(
            xyz=(
                -(door_leaf_width - 0.07),
                door_thickness + handle_standoff_length / 2.0,
                0.98 + handle_bar_length / 2.0 - 0.075,
            ),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=stainless,
        name="right_handle_standoff_bottom",
    )

    rack_width = 0.27
    rack_depth = 0.115
    rack_height = 0.18
    rack_bottom_z = 0.18
    rack_center_x = -0.205

    right_door.visual(
        Box((rack_width, 0.016, rack_height)),
        origin=Origin(xyz=(rack_center_x, -0.008, rack_bottom_z + rack_height / 2.0)),
        material=warm_white,
        name="right_lower_bin_back",
    )
    right_door.visual(
        Box((rack_width, rack_depth, 0.014)),
        origin=Origin(xyz=(rack_center_x, -rack_depth / 2.0, rack_bottom_z + 0.007)),
        material=warm_white,
        name="right_lower_bin_floor",
    )
    for suffix, x_offset in (
        ("inner", -rack_width / 2.0 + 0.008),
        ("outer", rack_width / 2.0 - 0.008),
    ):
        right_door.visual(
            Box((0.016, rack_depth, rack_height)),
            origin=Origin(
                xyz=(
                    rack_center_x + x_offset,
                    -rack_depth / 2.0,
                    rack_bottom_z + rack_height / 2.0,
                )
            ),
            material=warm_white,
            name=f"right_lower_bin_side_{suffix}",
        )
    right_door.visual(
        Box((rack_width, 0.012, 0.04)),
        origin=Origin(
            xyz=(rack_center_x, -rack_depth + 0.006, rack_bottom_z + 0.02)
        ),
        material=warm_white,
        name="right_lower_bin_front_lip",
    )

    rail_span = rack_width - 0.05
    rail_geometry = tube_from_spline_points(
        [
            (-rail_span / 2.0, 0.0, 0.0),
            (-rail_span / 2.0, -0.02, 0.02),
            (-rail_span / 2.0, -0.065, 0.072),
            (0.0, -0.084, 0.09),
            (rail_span / 2.0, -0.065, 0.072),
            (rail_span / 2.0, -0.02, 0.02),
            (rail_span / 2.0, 0.0, 0.0),
        ],
        radius=0.006,
        samples_per_segment=10,
        radial_segments=16,
        cap_ends=True,
    )
    bottle_rail = model.part("right_door_bottle_rail")
    bottle_rail.visual(
        mesh_from_geometry(rail_geometry, "right_door_bottle_rail"),
        material=rail_silver,
        name="rail_wire",
    )

    model.articulation(
        "left_door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=left_door,
        origin=Origin(xyz=(-body_width / 2.0, body_depth / 2.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.8,
            lower=0.0,
            upper=1.75,
        ),
    )
    model.articulation(
        "right_door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=right_door,
        origin=Origin(xyz=(body_width / 2.0, body_depth / 2.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.8,
            lower=0.0,
            upper=1.75,
        ),
    )
    model.articulation(
        "right_bottle_rail_hinge",
        ArticulationType.REVOLUTE,
        parent=right_door,
        child=bottle_rail,
        origin=Origin(xyz=(rack_center_x, -0.022, rack_bottom_z + 0.06)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.5,
            lower=0.0,
            upper=1.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    cabinet = object_model.get_part("cabinet")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    bottle_rail = object_model.get_part("right_door_bottle_rail")

    left_hinge = object_model.get_articulation("left_door_hinge")
    right_hinge = object_model.get_articulation("right_door_hinge")
    rail_hinge = object_model.get_articulation("right_bottle_rail_hinge")

    ctx.check(
        "hinge axes match refrigerator mechanisms",
        left_hinge.axis == (0.0, 0.0, 1.0)
        and right_hinge.axis == (0.0, 0.0, -1.0)
        and rail_hinge.axis == (1.0, 0.0, 0.0),
        details=(
            f"left={left_hinge.axis}, right={right_hinge.axis}, rail={rail_hinge.axis}"
        ),
    )

    ctx.expect_gap(
        left_door,
        cabinet,
        axis="y",
        min_gap=0.0,
        max_gap=0.003,
        positive_elem="left_door_panel",
        name="left door panel sits flush to cabinet front",
    )
    ctx.expect_gap(
        right_door,
        cabinet,
        axis="y",
        min_gap=0.0,
        max_gap=0.003,
        positive_elem="right_door_panel",
        name="right door panel sits flush to cabinet front",
    )
    ctx.expect_overlap(
        left_door,
        cabinet,
        axes="xz",
        min_overlap=0.35,
        elem_a="left_door_panel",
        name="left door covers cabinet opening footprint",
    )
    ctx.expect_overlap(
        right_door,
        cabinet,
        axes="xz",
        min_overlap=0.35,
        elem_a="right_door_panel",
        name="right door covers cabinet opening footprint",
    )

    left_closed = ctx.part_element_world_aabb(left_door, elem="left_door_panel")
    right_closed = ctx.part_element_world_aabb(right_door, elem="right_door_panel")
    with ctx.pose({left_hinge: 1.25, right_hinge: 1.25}):
        left_open = ctx.part_element_world_aabb(left_door, elem="left_door_panel")
        right_open = ctx.part_element_world_aabb(right_door, elem="right_door_panel")

    ctx.check(
        "left door swings outward from left wall",
        left_closed is not None
        and left_open is not None
        and left_open[1][1] > left_closed[1][1] + 0.20
        and left_open[1][0] < left_closed[1][0] - 0.15,
        details=f"closed={left_closed}, open={left_open}",
    )
    ctx.check(
        "right door swings outward from right wall",
        right_closed is not None
        and right_open is not None
        and right_open[1][1] > right_closed[1][1] + 0.20
        and right_open[0][0] > right_closed[0][0] + 0.15,
        details=f"closed={right_closed}, open={right_open}",
    )

    with ctx.pose({right_hinge: 1.2, rail_hinge: 0.0}):
        rail_closed = ctx.part_element_world_aabb(bottle_rail, elem="rail_wire")
    with ctx.pose({right_hinge: 1.2, rail_hinge: 1.0}):
        rail_open = ctx.part_element_world_aabb(bottle_rail, elem="rail_wire")

    ctx.check(
        "bottle rail folds down to release the lower door bin",
        rail_closed is not None
        and rail_open is not None
        and rail_open[0][2] < rail_closed[0][2] - 0.06
        and rail_open[0][1] < rail_closed[0][1] - 0.01,
        details=f"closed={rail_closed}, open={rail_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
