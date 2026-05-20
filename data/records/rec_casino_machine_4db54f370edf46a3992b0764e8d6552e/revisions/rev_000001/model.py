from __future__ import annotations

from math import pi, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)
import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="upright_casino_machine")

    cabinet_mat = Material("deep_red_lacquer", rgba=(0.48, 0.05, 0.04, 1.0))
    trim_mat = Material("brushed_gold", rgba=(0.92, 0.63, 0.16, 1.0))
    dark_mat = Material("black_enamel", rgba=(0.01, 0.01, 0.012, 1.0))
    glass_mat = Material("smoked_glass", rgba=(0.22, 0.35, 0.45, 0.34))
    reel_mat = Material("warm_ivory", rgba=(0.96, 0.88, 0.67, 1.0))
    metal_mat = Material("polished_steel", rgba=(0.78, 0.78, 0.74, 1.0))
    lock_mat = Material("dark_keyway", rgba=(0.02, 0.018, 0.015, 1.0))
    symbol_red = Material("symbol_red", rgba=(0.85, 0.04, 0.03, 1.0))
    symbol_yellow = Material("symbol_yellow", rgba=(0.98, 0.82, 0.05, 1.0))
    symbol_green = Material("symbol_green", rgba=(0.02, 0.62, 0.18, 1.0))
    symbol_blue = Material("symbol_blue", rgba=(0.05, 0.22, 0.82, 1.0))

    cabinet = model.part("cabinet")

    # Upright rectangular cabinet shell.  The front is built from rails and
    # stiles so the reel window and lower coin-tray opening are genuinely open.
    cabinet.visual(Box((0.72, 0.045, 1.65)), origin=Origin(xyz=(0.0, 0.2475, 0.825)), material=cabinet_mat, name="rear_wall")
    cabinet.visual(Box((0.045, 0.52, 1.65)), origin=Origin(xyz=(-0.36, 0.0, 0.825)), material=cabinet_mat, name="side_wall_0")
    cabinet.visual(Box((0.045, 0.52, 1.65)), origin=Origin(xyz=(0.36, 0.0, 0.825)), material=cabinet_mat, name="side_wall_1")
    cabinet.visual(Box((0.72, 0.52, 0.055)), origin=Origin(xyz=(0.0, 0.0, 0.0275)), material=cabinet_mat, name="floor_panel")
    cabinet.visual(Box((0.72, 0.52, 0.06)), origin=Origin(xyz=(0.0, 0.0, 1.62)), material=cabinet_mat, name="top_panel")

    # Continuous front face members and decorative trim.
    cabinet.visual(Box((0.055, 0.05, 1.58)), origin=Origin(xyz=(-0.3325, -0.255, 0.82)), material=trim_mat, name="front_stile_0")
    cabinet.visual(Box((0.055, 0.05, 1.58)), origin=Origin(xyz=(0.3325, -0.255, 0.82)), material=trim_mat, name="front_stile_1")
    cabinet.visual(Box((0.72, 0.05, 0.075)), origin=Origin(xyz=(0.0, -0.255, 1.565)), material=trim_mat, name="top_rail")
    cabinet.visual(Box((0.72, 0.05, 0.075)), origin=Origin(xyz=(0.0, -0.255, 0.075)), material=trim_mat, name="base_rail")

    cabinet.visual(Box((0.60, 0.042, 0.25)), origin=Origin(xyz=(0.0, -0.259, 1.43)), material=cabinet_mat, name="marquee_panel")
    cabinet.visual(Box((0.48, 0.012, 0.07)), origin=Origin(xyz=(0.0, -0.284, 1.48)), material=trim_mat, name="marquee_plaque")
    cabinet.visual(Box((0.64, 0.042, 0.29)), origin=Origin(xyz=(0.0, -0.259, 0.62)), material=cabinet_mat, name="control_panel")
    cabinet.visual(Box((0.18, 0.011, 0.035)), origin=Origin(xyz=(0.0, -0.284, 0.69)), material=dark_mat, name="coin_slot")
    for i, x in enumerate((-0.13, 0.0, 0.13)):
        cabinet.visual(Cylinder(radius=0.026, length=0.012), origin=Origin(xyz=(x, -0.286, 0.57), rpy=(pi / 2, 0.0, 0.0)), material=trim_mat, name=f"button_{i}")

    # Reel viewing window: gold frame, clear glass, and a red pay line.
    cabinet.visual(Box((0.67, 0.055, 0.055)), origin=Origin(xyz=(0.0, -0.282, 1.285)), material=trim_mat, name="window_top_rail")
    cabinet.visual(Box((0.67, 0.055, 0.055)), origin=Origin(xyz=(0.0, -0.282, 0.815)), material=trim_mat, name="window_bottom_rail")
    cabinet.visual(Box((0.055, 0.055, 0.47)), origin=Origin(xyz=(-0.335, -0.282, 1.05)), material=trim_mat, name="window_stile_0")
    cabinet.visual(Box((0.055, 0.055, 0.47)), origin=Origin(xyz=(0.335, -0.282, 1.05)), material=trim_mat, name="window_stile_1")
    cabinet.visual(Box((0.64, 0.009, 0.45)), origin=Origin(xyz=(0.0, -0.314, 1.05)), material=glass_mat, name="glass_panel")
    cabinet.visual(Box((0.62, 0.006, 0.012)), origin=Origin(xyz=(0.0, -0.320, 1.05)), material=symbol_red, name="payline")
    cabinet.visual(Box((0.012, 0.006, 0.41)), origin=Origin(xyz=(-0.095, -0.320, 1.05)), material=dark_mat, name="reel_separator_0")
    cabinet.visual(Box((0.012, 0.006, 0.41)), origin=Origin(xyz=(0.095, -0.320, 1.05)), material=dark_mat, name="reel_separator_1")

    # Lower coin tray recess frame and exposed hinge knuckles.
    cabinet.visual(Box((0.045, 0.045, 0.27)), origin=Origin(xyz=(-0.26, -0.258, 0.285)), material=trim_mat, name="coin_jamb_0")
    cabinet.visual(Box((0.045, 0.045, 0.27)), origin=Origin(xyz=(0.26, -0.258, 0.285)), material=trim_mat, name="coin_jamb_1")
    cabinet.visual(Box((0.565, 0.045, 0.052)), origin=Origin(xyz=(0.0, -0.258, 0.446)), material=trim_mat, name="coin_header")
    cabinet.visual(Box((0.565, 0.045, 0.030)), origin=Origin(xyz=(0.0, -0.258, 0.115)), material=trim_mat, name="coin_sill")
    cabinet.visual(Cylinder(radius=0.016, length=0.11), origin=Origin(xyz=(-0.285, -0.265, 0.15), rpy=(0.0, pi / 2, 0.0)), material=metal_mat, name="hinge_knuckle_0")
    cabinet.visual(Cylinder(radius=0.016, length=0.11), origin=Origin(xyz=(0.285, -0.265, 0.15), rpy=(0.0, pi / 2, 0.0)), material=metal_mat, name="hinge_knuckle_1")

    # Side pivot plate for the pull handle.
    cabinet.visual(Cylinder(radius=0.072, length=0.022), origin=Origin(xyz=(0.392, -0.04, 1.02), rpy=(0.0, pi / 2, 0.0)), material=trim_mat, name="handle_pivot_plate")

    # Three independently spinning reel drums on parallel horizontal axes.
    reel_centers = (-0.19, 0.0, 0.19)
    symbol_sets = (
        (symbol_red, symbol_yellow, symbol_green),
        (symbol_blue, symbol_red, symbol_yellow),
        (symbol_green, symbol_blue, symbol_red),
    )
    reel_radius = 0.135
    reel_center_y = -0.10
    reel_center_z = 1.05
    reel_width = 0.135
    cabinet.visual(Box((0.70, 0.035, 0.035)), origin=Origin(xyz=(0.0, 0.065, reel_center_z)), material=metal_mat, name="reel_back_rail")
    for i, x in enumerate(reel_centers):
        for side_index, side in enumerate((-1.0, 1.0)):
            collar_x = x + side * (reel_width / 2.0 + 0.005)
            cabinet.visual(
                Box((0.010, 0.160, 0.025)),
                origin=Origin(xyz=(collar_x, -0.025, reel_center_z)),
                material=metal_mat,
                name=f"reel_bracket_{i}_{side_index}",
            )
            cabinet.visual(
                Cylinder(radius=0.052, length=0.010),
                origin=Origin(xyz=(collar_x, reel_center_y, reel_center_z), rpy=(0.0, pi / 2, 0.0)),
                material=metal_mat,
                name=f"reel_collar_{i}_{side_index}",
            )
        reel = model.part(f"reel_{i}")
        reel.visual(Cylinder(radius=reel_radius, length=0.135), origin=Origin(rpy=(0.0, pi / 2, 0.0)), material=reel_mat, name="reel_drum")
        reel.visual(Cylinder(radius=0.045, length=0.115), origin=Origin(rpy=(0.0, pi / 2, 0.0)), material=metal_mat, name="center_hub")
        for j, z in enumerate((-0.082, 0.0, 0.082)):
            surface_y = -sqrt(reel_radius * reel_radius - z * z)
            reel.visual(
                Box((0.105, 0.008, 0.052)),
                origin=Origin(xyz=(0.0, surface_y - 0.002, z)),
                material=symbol_sets[i][j],
                name=f"symbol_{j}",
            )
        model.articulation(
            f"cabinet_to_reel_{i}",
            ArticulationType.CONTINUOUS,
            parent=cabinet,
            child=reel,
            origin=Origin(xyz=(x, reel_center_y, reel_center_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=12.0),
        )

    handle = model.part("handle")
    handle.visual(Cylinder(radius=0.052, length=0.090), origin=Origin(xyz=(0.045, 0.0, 0.0), rpy=(0.0, pi / 2, 0.0)), material=metal_mat, name="pivot_hub")
    handle.visual(Cylinder(radius=0.020, length=0.38), origin=Origin(xyz=(0.103, 0.0, 0.205)), material=metal_mat, name="lever_stem")
    handle.visual(Sphere(radius=0.058), origin=Origin(xyz=(0.103, 0.0, 0.43)), material=dark_mat, name="grip_knob")
    model.articulation(
        "cabinet_to_handle",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=handle,
        origin=Origin(xyz=(0.403, -0.04, 1.02)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=2.0, lower=0.0, upper=1.1),
    )

    coin_door = model.part("coin_door")
    coin_door.visual(Box((0.42, 0.026, 0.266)), origin=Origin(xyz=(0.0, -0.017, 0.133)), material=cabinet_mat, name="door_panel")
    coin_door.visual(Box((0.34, 0.010, 0.045)), origin=Origin(xyz=(0.0, -0.034, 0.088)), material=dark_mat, name="tray_pull")
    coin_door.visual(Cylinder(radius=0.026, length=0.012), origin=Origin(xyz=(0.135, -0.034, 0.185), rpy=(pi / 2, 0.0, 0.0)), material=trim_mat, name="lock_bezel")
    coin_door.visual(Box((0.008, 0.006, 0.024)), origin=Origin(xyz=(0.135, -0.041, 0.185)), material=lock_mat, name="key_slot")
    coin_door.visual(Cylinder(radius=0.014, length=0.46), origin=Origin(xyz=(0.0, -0.013, 0.0), rpy=(0.0, pi / 2, 0.0)), material=metal_mat, name="door_hinge_barrel")
    model.articulation(
        "cabinet_to_coin_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=coin_door,
        origin=Origin(xyz=(0.0, -0.252, 0.15)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.4, lower=0.0, upper=1.15),
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
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    cabinet = object_model.get_part("cabinet")
    handle = object_model.get_part("handle")
    coin_door = object_model.get_part("coin_door")
    reel_parts = [object_model.get_part(f"reel_{i}") for i in range(3)]
    reel_joints = [object_model.get_articulation(f"cabinet_to_reel_{i}") for i in range(3)]
    handle_joint = object_model.get_articulation("cabinet_to_handle")
    door_joint = object_model.get_articulation("cabinet_to_coin_door")

    ctx.check(
        "three independent continuous reel joints",
        all(j.articulation_type == ArticulationType.CONTINUOUS for j in reel_joints),
        details=f"joint types={[j.articulation_type for j in reel_joints]}",
    )
    ctx.check(
        "reel axes are parallel and horizontal",
        all(tuple(j.axis) == (1.0, 0.0, 0.0) for j in reel_joints),
        details=f"axes={[j.axis for j in reel_joints]}",
    )
    for i, reel in enumerate(reel_parts):
        ctx.expect_within(
            reel,
            cabinet,
            axes="xz",
            outer_elem="glass_panel",
            margin=0.004,
            name=f"reel_{i} visible within viewing window",
        )

    ctx.expect_gap(
        handle,
        cabinet,
        axis="x",
        positive_elem="pivot_hub",
        negative_elem="handle_pivot_plate",
        min_gap=0.0,
        max_gap=0.006,
        name="handle hub seats just outside side pivot plate",
    )
    ctx.expect_gap(
        coin_door,
        cabinet,
        axis="z",
        positive_elem="door_panel",
        negative_elem="coin_sill",
        min_gap=0.018,
        max_gap=0.024,
        name="coin door clears the lower sill at the hinge edge",
    )

    closed_door_aabb = ctx.part_world_aabb(coin_door)
    closed_handle_aabb = ctx.part_world_aabb(handle)
    with ctx.pose({door_joint: 1.15, handle_joint: 1.1}):
        opened_door_aabb = ctx.part_world_aabb(coin_door)
        pulled_handle_aabb = ctx.part_world_aabb(handle)

    ctx.check(
        "coin tray door swings downward",
        closed_door_aabb is not None
        and opened_door_aabb is not None
        and opened_door_aabb[1][2] < closed_door_aabb[1][2] - 0.045,
        details=f"closed={closed_door_aabb}, opened={opened_door_aabb}",
    )
    ctx.check(
        "side handle rotates forward and downward",
        closed_handle_aabb is not None
        and pulled_handle_aabb is not None
        and pulled_handle_aabb[0][1] < closed_handle_aabb[0][1] - 0.12
        and pulled_handle_aabb[1][2] < closed_handle_aabb[1][2] - 0.08,
        details=f"closed={closed_handle_aabb}, pulled={pulled_handle_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
