from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_under_counter_refrigerator")

    cabinet_white = model.material("cabinet_white", rgba=(0.95, 0.96, 0.97, 1.0))
    liner_white = model.material("liner_white", rgba=(0.91, 0.93, 0.95, 1.0))
    gasket_gray = model.material("gasket_gray", rgba=(0.69, 0.72, 0.75, 1.0))
    handle_gray = model.material("handle_gray", rgba=(0.56, 0.58, 0.61, 1.0))
    glass_blue = model.material("glass_blue", rgba=(0.79, 0.88, 0.95, 0.45))
    flap_white = model.material("flap_white", rgba=(0.86, 0.90, 0.95, 0.96))

    width = 0.595
    depth = 0.600
    height = 0.820

    side_t = 0.022
    top_t = 0.030
    bottom_t = 0.085
    back_t = 0.018

    cavity_width = width - (2.0 * side_t)
    cavity_height = height - top_t - bottom_t
    cavity_back_y = (-depth / 2.0) + back_t
    cavity_top_z = height - top_t

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((side_t, depth, height)),
        origin=Origin(xyz=(-(width / 2.0) + (side_t / 2.0), 0.0, height / 2.0)),
        material=cabinet_white,
        name="left_wall",
    )
    cabinet.visual(
        Box((side_t, depth, height)),
        origin=Origin(xyz=((width / 2.0) - (side_t / 2.0), 0.0, height / 2.0)),
        material=cabinet_white,
        name="right_wall",
    )
    cabinet.visual(
        Box((cavity_width, depth, top_t)),
        origin=Origin(xyz=(0.0, 0.0, height - (top_t / 2.0))),
        material=cabinet_white,
        name="top_shell",
    )
    cabinet.visual(
        Box((cavity_width, depth, bottom_t)),
        origin=Origin(xyz=(0.0, 0.0, bottom_t / 2.0)),
        material=cabinet_white,
        name="bottom_base",
    )
    cabinet.visual(
        Box((cavity_width, back_t, cavity_height)),
        origin=Origin(
            xyz=(
                0.0,
                (-depth / 2.0) + (back_t / 2.0),
                bottom_t + (cavity_height / 2.0),
            )
        ),
        material=liner_white,
        name="back_liner",
    )

    shelf_depth = 0.340
    shelf_t = 0.006
    cabinet.visual(
        Box((cavity_width, shelf_depth, shelf_t)),
        origin=Origin(
            xyz=(
                0.0,
                cavity_back_y + (shelf_depth / 2.0),
                0.365,
            )
        ),
        material=glass_blue,
        name="glass_shelf",
    )

    freezer_width = 0.430
    freezer_depth = 0.215
    freezer_height = 0.135
    freezer_top_t = 0.010
    freezer_side_t = 0.012
    freezer_floor_t = 0.008
    freezer_center_y = cavity_back_y + (freezer_depth / 2.0)
    freezer_bottom_z = cavity_top_z - freezer_height
    freezer_wall_height = freezer_height - freezer_top_t - freezer_floor_t

    cabinet.visual(
        Box((freezer_width, freezer_depth, freezer_top_t)),
        origin=Origin(
            xyz=(
                0.0,
                freezer_center_y,
                cavity_top_z - (freezer_top_t / 2.0),
            )
        ),
        material=liner_white,
        name="freezer_top",
    )
    cabinet.visual(
        Box((freezer_width, freezer_side_t, freezer_wall_height)),
        origin=Origin(
            xyz=(
                0.0,
                cavity_back_y + (freezer_side_t / 2.0),
                freezer_bottom_z + freezer_floor_t + (freezer_wall_height / 2.0),
            )
        ),
        material=liner_white,
        name="freezer_back",
    )
    cabinet.visual(
        Box((freezer_side_t, freezer_depth, freezer_wall_height)),
        origin=Origin(
            xyz=(
                -(freezer_width / 2.0) + (freezer_side_t / 2.0),
                freezer_center_y,
                freezer_bottom_z + freezer_floor_t + (freezer_wall_height / 2.0),
            )
        ),
        material=liner_white,
        name="freezer_left_cheek",
    )
    cabinet.visual(
        Box((freezer_side_t, freezer_depth, freezer_wall_height)),
        origin=Origin(
            xyz=(
                (freezer_width / 2.0) - (freezer_side_t / 2.0),
                freezer_center_y,
                freezer_bottom_z + freezer_floor_t + (freezer_wall_height / 2.0),
            )
        ),
        material=liner_white,
        name="freezer_right_cheek",
    )
    cabinet.visual(
        Box((freezer_width, freezer_depth, freezer_floor_t)),
        origin=Origin(
            xyz=(
                0.0,
                freezer_center_y,
                freezer_bottom_z + (freezer_floor_t / 2.0),
            )
        ),
        material=liner_white,
        name="freezer_floor",
    )
    cabinet.inertial = Inertial.from_geometry(
        Box((width, depth, height)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, height / 2.0)),
    )

    door = model.part("main_door")
    door_width = width - 0.004
    door_height = height - 0.004
    outer_panel_t = 0.030
    inner_liner_t = 0.018
    inner_liner_width = cavity_width - 0.014
    inner_liner_height = cavity_height - 0.015
    door_center_x = -(door_width / 2.0)

    door.visual(
        Box((door_width, outer_panel_t, door_height)),
        origin=Origin(
            xyz=(
                door_center_x,
                outer_panel_t / 2.0,
                door_height / 2.0,
            )
        ),
        material=cabinet_white,
        name="outer_panel",
    )
    door.visual(
        Box((inner_liner_width, inner_liner_t, inner_liner_height)),
        origin=Origin(
            xyz=(
                door_center_x,
                -(inner_liner_t / 2.0),
                0.091 + (inner_liner_height / 2.0),
            )
        ),
        material=liner_white,
        name="inner_liner",
    )

    gasket_depth = 0.007
    gasket_side_w = 0.018
    gasket_top_h = 0.018
    gasket_span_h = inner_liner_height
    gasket_span_w = inner_liner_width - (2.0 * gasket_side_w)
    gasket_y = -(gasket_depth / 2.0)
    door.visual(
        Box((gasket_side_w, gasket_depth, gasket_span_h)),
        origin=Origin(
            xyz=(
                door_center_x - (inner_liner_width / 2.0) + (gasket_side_w / 2.0),
                gasket_y,
                0.091 + (inner_liner_height / 2.0),
            )
        ),
        material=gasket_gray,
        name="gasket_left",
    )
    door.visual(
        Box((gasket_side_w, gasket_depth, gasket_span_h)),
        origin=Origin(
            xyz=(
                door_center_x + (inner_liner_width / 2.0) - (gasket_side_w / 2.0),
                gasket_y,
                0.091 + (inner_liner_height / 2.0),
            )
        ),
        material=gasket_gray,
        name="gasket_right",
    )
    door.visual(
        Box((gasket_span_w, gasket_depth, gasket_top_h)),
        origin=Origin(
            xyz=(
                door_center_x,
                gasket_y,
                0.091 + inner_liner_height - (gasket_top_h / 2.0),
            )
        ),
        material=gasket_gray,
        name="gasket_top",
    )
    door.visual(
        Box((gasket_span_w, gasket_depth, gasket_top_h)),
        origin=Origin(
            xyz=(
                door_center_x,
                gasket_y,
                0.091 + (gasket_top_h / 2.0),
            )
        ),
        material=gasket_gray,
        name="gasket_bottom",
    )
    door.visual(
        Box((0.018, 0.016, 0.340)),
        origin=Origin(
            xyz=(
                -door_width + 0.050,
                outer_panel_t + 0.008,
                0.445,
            )
        ),
        material=handle_gray,
        name="door_handle",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_width, outer_panel_t + inner_liner_t, door_height)),
        mass=5.5,
        origin=Origin(
            xyz=(
                door_center_x,
                (outer_panel_t - inner_liner_t) / 2.0,
                door_height / 2.0,
            )
        ),
    )

    freezer_flap = model.part("freezer_flap")
    flap_width = freezer_width - 0.040
    flap_height = freezer_height - 0.023
    flap_t = 0.014
    freezer_flap.visual(
        Box((flap_width, flap_t, flap_height)),
        origin=Origin(xyz=(0.0, flap_t / 2.0, -(flap_height / 2.0))),
        material=flap_white,
        name="flap_panel",
    )
    freezer_flap.visual(
        Box((0.120, 0.010, 0.018)),
        origin=Origin(
            xyz=(
                0.0,
                flap_t + 0.005,
                -flap_height + 0.009,
            )
        ),
        material=handle_gray,
        name="flap_pull",
    )
    freezer_flap.inertial = Inertial.from_geometry(
        Box((flap_width, flap_t, flap_height)),
        mass=0.45,
        origin=Origin(xyz=(0.0, flap_t / 2.0, -(flap_height / 2.0))),
    )

    model.articulation(
        "cabinet_to_main_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(width / 2.0, depth / 2.0, 0.002)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.8,
            lower=0.0,
            upper=1.75,
        ),
    )
    model.articulation(
        "cabinet_to_freezer_flap",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=freezer_flap,
        origin=Origin(
            xyz=(
                0.0,
                cavity_back_y + freezer_depth - 0.006,
                cavity_top_z - freezer_top_t,
            )
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.2,
            lower=0.0,
            upper=1.20,
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
    door = object_model.get_part("main_door")
    flap = object_model.get_part("freezer_flap")
    door_hinge = object_model.get_articulation("cabinet_to_main_door")
    flap_hinge = object_model.get_articulation("cabinet_to_freezer_flap")

    ctx.check(
        "main door hinge axis is vertical",
        tuple(door_hinge.axis) == (0.0, 0.0, -1.0),
        details=f"axis={door_hinge.axis}",
    )
    ctx.check(
        "freezer flap hinge axis is horizontal",
        tuple(flap_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"axis={flap_hinge.axis}",
    )

    with ctx.pose({door_hinge: 0.0, flap_hinge: 0.0}):
        ctx.expect_gap(
            door,
            cabinet,
            axis="y",
            positive_elem="outer_panel",
            max_gap=0.0025,
            max_penetration=0.0,
            name="main door outer skin closes flush to cabinet front",
        )
        ctx.expect_overlap(
            door,
            cabinet,
            axes="xz",
            elem_a="outer_panel",
            min_overlap=0.55,
            name="main door covers the cabinet opening footprint",
        )
        closed_door_aabb = ctx.part_element_world_aabb(door, elem="outer_panel")
        closed_flap_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")

    with ctx.pose({door_hinge: 1.35}):
        open_door_aabb = ctx.part_element_world_aabb(door, elem="outer_panel")

    ctx.check(
        "main door swings outward from the cabinet",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][1] > closed_door_aabb[1][1] + 0.18
        and open_door_aabb[0][0] > closed_door_aabb[0][0] + 0.12,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    with ctx.pose({flap_hinge: 1.00}):
        open_flap_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")

    ctx.check(
        "freezer flap lifts upward and outward into the cavity",
        closed_flap_aabb is not None
        and open_flap_aabb is not None
        and open_flap_aabb[1][1] > closed_flap_aabb[1][1] + 0.05
        and open_flap_aabb[0][2] > closed_flap_aabb[0][2] + 0.03,
        details=f"closed={closed_flap_aabb}, open={open_flap_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
