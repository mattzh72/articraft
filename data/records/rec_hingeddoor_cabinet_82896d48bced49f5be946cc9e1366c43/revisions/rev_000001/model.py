from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


def _add_framed_door(
    part,
    *,
    width: float,
    height: float,
    thickness: float,
    stile_width: float,
    rail_height: float,
    x_sign: float,
    material: str,
    panel_material: str,
) -> None:
    def sx(value: float) -> float:
        return x_sign * value

    panel_thickness = thickness * 0.56
    panel_recess = 0.006
    panel_center_y = thickness / 2.0 - panel_recess - panel_thickness / 2.0
    panel_width = width - 2.0 * (stile_width - 0.010)
    panel_height = height - 2.0 * (rail_height - 0.010)

    part.visual(
        Box((stile_width, thickness, height)),
        origin=Origin(xyz=(sx(stile_width / 2.0), 0.0, height / 2.0)),
        material=material,
        name="hinge_stile",
    )
    part.visual(
        Box((stile_width, thickness, height)),
        origin=Origin(xyz=(sx(width - stile_width / 2.0), 0.0, height / 2.0)),
        material=material,
        name="meeting_stile",
    )
    part.visual(
        Box((width - 2.0 * stile_width, thickness, rail_height)),
        origin=Origin(xyz=(sx(width / 2.0), 0.0, rail_height / 2.0)),
        material=material,
        name="bottom_rail",
    )
    part.visual(
        Box((width - 2.0 * stile_width, thickness, rail_height)),
        origin=Origin(xyz=(sx(width / 2.0), 0.0, height - rail_height / 2.0)),
        material=material,
        name="top_rail",
    )
    part.visual(
        Box((panel_width, panel_thickness, panel_height)),
        origin=Origin(xyz=(sx(width / 2.0), panel_center_y, height / 2.0)),
        material=panel_material,
        name="center_panel",
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) / 2.0 for i in range(3))


def _aabb_dims(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple(maxs[i] - mins[i] for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pantry_cabinet")

    painted_white = model.material("painted_white", rgba=(0.94, 0.93, 0.90, 1.0))
    warm_panel = model.material("warm_panel", rgba=(0.90, 0.89, 0.85, 1.0))
    interior_white = model.material("interior_white", rgba=(0.96, 0.95, 0.92, 1.0))
    shelf_white = model.material("shelf_white", rgba=(0.95, 0.95, 0.93, 1.0))
    charcoal = model.material("charcoal", rgba=(0.20, 0.20, 0.22, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.24, 0.22, 0.20, 1.0))

    cabinet_width = 0.92
    cabinet_depth = 0.42
    cabinet_height = 2.02
    side_thickness = 0.018
    carcass_panel_thickness = 0.020
    back_thickness = 0.008
    toe_kick_height = 0.100
    toe_kick_recess = 0.070

    door_thickness = 0.022
    door_bottom = 0.110
    door_top_gap = 0.030
    meeting_gap = 0.004
    door_height = cabinet_height - door_bottom - door_top_gap
    door_width = cabinet_width / 2.0 - meeting_gap / 2.0
    stile_width = 0.070
    rail_height = 0.095

    cabinet = model.part("cabinet")

    interior_width = cabinet_width - 2.0 * side_thickness
    cabinet.visual(
        Box((side_thickness, cabinet_depth, cabinet_height)),
        origin=Origin(xyz=(-cabinet_width / 2.0 + side_thickness / 2.0, 0.0, cabinet_height / 2.0)),
        material=painted_white,
        name="left_side",
    )
    cabinet.visual(
        Box((side_thickness, cabinet_depth, cabinet_height)),
        origin=Origin(xyz=(cabinet_width / 2.0 - side_thickness / 2.0, 0.0, cabinet_height / 2.0)),
        material=painted_white,
        name="right_side",
    )
    cabinet.visual(
        Box((interior_width, cabinet_depth, carcass_panel_thickness)),
        origin=Origin(xyz=(0.0, 0.0, cabinet_height - carcass_panel_thickness / 2.0)),
        material=painted_white,
        name="top_panel",
    )
    cabinet.visual(
        Box((interior_width, cabinet_depth, carcass_panel_thickness)),
        origin=Origin(xyz=(0.0, 0.0, toe_kick_height + carcass_panel_thickness / 2.0)),
        material=painted_white,
        name="bottom_panel",
    )
    cabinet.visual(
        Box((interior_width, back_thickness, cabinet_height)),
        origin=Origin(xyz=(0.0, -cabinet_depth / 2.0 + back_thickness / 2.0, cabinet_height / 2.0)),
        material=interior_white,
        name="back_panel",
    )
    cabinet.visual(
        Box((interior_width, 0.018, toe_kick_height)),
        origin=Origin(
            xyz=(0.0, cabinet_depth / 2.0 - toe_kick_recess - 0.009, toe_kick_height / 2.0)
        ),
        material=painted_white,
        name="toe_kick_face",
    )

    shelf_depth = cabinet_depth - 0.080
    shelf_thickness = 0.018
    shelf_positions = (0.48, 0.86, 1.24, 1.62)
    for index, shelf_z in enumerate(shelf_positions, start=1):
        cabinet.visual(
            Box((interior_width, shelf_depth, shelf_thickness)),
            origin=Origin(xyz=(0.0, -0.012, shelf_z)),
            material=shelf_white,
            name=f"shelf_{index}",
        )

    hole_radius = 0.0025
    hole_length = 0.002
    hole_front_y = 0.125
    hole_rear_y = -0.125
    hole_heights = (0.36, 0.56, 0.76, 0.96, 1.16, 1.36, 1.56)
    left_inner_x = -cabinet_width / 2.0 + side_thickness - hole_length / 2.0
    right_inner_x = cabinet_width / 2.0 - side_thickness + hole_length / 2.0
    for side_name, hole_x in (("left", left_inner_x), ("right", right_inner_x)):
        for row_name, hole_y in (("front", hole_front_y), ("rear", hole_rear_y)):
            for idx, hole_z in enumerate(hole_heights, start=1):
                cabinet.visual(
                    Cylinder(radius=hole_radius, length=hole_length),
                    origin=Origin(
                        xyz=(hole_x, hole_y, hole_z),
                        rpy=(0.0, pi / 2.0, 0.0),
                    ),
                    material=charcoal,
                    name=f"{side_name}_{row_name}_pin_hole_{idx}",
                )

    left_door = model.part("left_door")
    _add_framed_door(
        left_door,
        width=door_width,
        height=door_height,
        thickness=door_thickness,
        stile_width=stile_width,
        rail_height=rail_height,
        x_sign=1.0,
        material="painted_white",
        panel_material="warm_panel",
    )

    right_door = model.part("right_door")
    _add_framed_door(
        right_door,
        width=door_width,
        height=door_height,
        thickness=door_thickness,
        stile_width=stile_width,
        rail_height=rail_height,
        x_sign=-1.0,
        material="painted_white",
        panel_material="warm_panel",
    )

    latch = model.part("meeting_latch")
    hub_radius = 0.012
    hub_length = 0.006
    latch_bar_length = 0.078
    latch_bar_width = 0.012
    latch_bar_thickness = 0.005
    latch.visual(
        Cylinder(radius=hub_radius, length=hub_length),
        origin=Origin(xyz=(0.0, hub_length / 2.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="latch_hub",
    )
    latch.visual(
        Box((latch_bar_width, latch_bar_thickness, latch_bar_length)),
        origin=Origin(xyz=(0.0, hub_length + latch_bar_thickness / 2.0, latch_bar_length / 2.0)),
        material=dark_metal,
        name="latch_bar",
    )

    model.articulation(
        "cabinet_to_left_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=left_door,
        origin=Origin(xyz=(-cabinet_width / 2.0, cabinet_depth / 2.0 + door_thickness / 2.0, door_bottom)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.8, lower=0.0, upper=1.75),
    )
    model.articulation(
        "cabinet_to_right_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=right_door,
        origin=Origin(xyz=(cabinet_width / 2.0, cabinet_depth / 2.0 + door_thickness / 2.0, door_bottom)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.8, lower=0.0, upper=1.75),
    )
    model.articulation(
        "right_door_to_latch",
        ArticulationType.REVOLUTE,
        parent=right_door,
        child=latch,
        origin=Origin(
            xyz=(
                -(door_width - stile_width / 2.0),
                door_thickness / 2.0,
                1.03,
            )
        ),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0, lower=0.0, upper=pi / 2.0),
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
    latch = object_model.get_part("meeting_latch")

    left_hinge = object_model.get_articulation("cabinet_to_left_door")
    right_hinge = object_model.get_articulation("cabinet_to_right_door")
    latch_joint = object_model.get_articulation("right_door_to_latch")

    with ctx.pose({left_hinge: 0.0, right_hinge: 0.0, latch_joint: 0.0}):
        ctx.expect_gap(
            left_door,
            cabinet,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            name="left door closes flush to cabinet front",
        )
        ctx.expect_gap(
            right_door,
            cabinet,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            name="right door closes flush to cabinet front",
        )
        ctx.expect_gap(
            right_door,
            left_door,
            axis="x",
            min_gap=0.002,
            max_gap=0.008,
            name="closed doors keep a narrow meeting gap",
        )
        ctx.expect_contact(
            latch,
            right_door,
            elem_a="latch_hub",
            contact_tol=0.001,
            name="rotary latch is mounted to the right door",
        )

        closed_left_stile = _aabb_center(ctx.part_element_world_aabb(left_door, elem="meeting_stile"))
        closed_right_stile = _aabb_center(ctx.part_element_world_aabb(right_door, elem="meeting_stile"))
        closed_latch_bar_dims = _aabb_dims(ctx.part_element_world_aabb(latch, elem="latch_bar"))

    with ctx.pose({left_hinge: 1.30, right_hinge: 0.0, latch_joint: 0.0}):
        open_left_stile = _aabb_center(ctx.part_element_world_aabb(left_door, elem="meeting_stile"))

    with ctx.pose({left_hinge: 0.0, right_hinge: 1.30, latch_joint: 0.0}):
        open_right_stile = _aabb_center(ctx.part_element_world_aabb(right_door, elem="meeting_stile"))

    with ctx.pose({left_hinge: 0.0, right_hinge: 0.0, latch_joint: pi / 2.0}):
        turned_latch_bar_dims = _aabb_dims(ctx.part_element_world_aabb(latch, elem="latch_bar"))

    ctx.check(
        "left door swings outward on the cabinet side hinge",
        closed_left_stile is not None
        and open_left_stile is not None
        and open_left_stile[1] > closed_left_stile[1] + 0.18,
        details=f"closed={closed_left_stile}, open={open_left_stile}",
    )
    ctx.check(
        "right door swings outward on the cabinet side hinge",
        closed_right_stile is not None
        and open_right_stile is not None
        and open_right_stile[1] > closed_right_stile[1] + 0.18,
        details=f"closed={closed_right_stile}, open={open_right_stile}",
    )
    ctx.check(
        "latch bar rotates about its own pivot",
        closed_latch_bar_dims is not None
        and turned_latch_bar_dims is not None
        and turned_latch_bar_dims[0] > closed_latch_bar_dims[0] + 0.035
        and turned_latch_bar_dims[2] < closed_latch_bar_dims[2] - 0.035,
        details=f"closed={closed_latch_bar_dims}, turned={turned_latch_bar_dims}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
