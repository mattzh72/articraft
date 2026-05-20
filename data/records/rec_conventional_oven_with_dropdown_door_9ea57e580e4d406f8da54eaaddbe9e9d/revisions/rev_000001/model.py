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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rotisserie_countertop_oven")

    body_black = model.material("body_black", rgba=(0.14, 0.15, 0.16, 1.0))
    trim_black = model.material("trim_black", rgba=(0.08, 0.09, 0.10, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.74, 0.76, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.34, 0.36, 0.39, 1.0))
    glass = model.material("glass", rgba=(0.30, 0.38, 0.42, 0.30))
    foot_rubber = model.material("foot_rubber", rgba=(0.07, 0.07, 0.08, 1.0))

    width = 0.56
    depth = 0.43
    height = 0.35
    wall_t = 0.016
    top_t = 0.018
    bottom_t = 0.016
    foot_h = 0.012
    front_frame_depth = 0.022
    opening_width = 0.49
    opening_height = 0.23
    opening_bottom = 0.038
    opening_top = opening_bottom + opening_height
    front_y = depth * 0.5
    back_y = -depth * 0.5
    inner_side_x = (width * 0.5) - wall_t

    body = model.part("oven_body")
    body.visual(
        Box((width, depth, bottom_t)),
        origin=Origin(xyz=(0.0, 0.0, foot_h + (bottom_t * 0.5))),
        material=body_black,
        name="bottom_pan",
    )
    body.visual(
        Box((width, depth, top_t)),
        origin=Origin(xyz=(0.0, 0.0, height - (top_t * 0.5))),
        material=body_black,
        name="top_shell",
    )
    side_wall_height = height - foot_h
    body.visual(
        Box((wall_t, depth, side_wall_height)),
        origin=Origin(xyz=(-(width * 0.5) + (wall_t * 0.5), 0.0, foot_h + (side_wall_height * 0.5))),
        material=body_black,
        name="left_shell_wall",
    )
    body.visual(
        Box((wall_t, depth, side_wall_height)),
        origin=Origin(xyz=((width * 0.5) - (wall_t * 0.5), 0.0, foot_h + (side_wall_height * 0.5))),
        material=body_black,
        name="right_shell_wall",
    )
    body.visual(
        Box((width - (2.0 * wall_t), wall_t, height - foot_h)),
        origin=Origin(
            xyz=(0.0, back_y + (wall_t * 0.5), foot_h + ((height - foot_h) * 0.5))
        ),
        material=body_black,
        name="rear_panel",
    )

    front_column_w = (width - opening_width) * 0.5
    body.visual(
        Box((front_column_w, front_frame_depth, height - opening_bottom)),
        origin=Origin(
            xyz=(
                -((opening_width * 0.5) + (front_column_w * 0.5)),
                front_y - (front_frame_depth * 0.5),
                opening_bottom + ((height - opening_bottom) * 0.5),
            )
        ),
        material=trim_black,
        name="front_left_column",
    )
    body.visual(
        Box((front_column_w, front_frame_depth, height - opening_bottom)),
        origin=Origin(
            xyz=(
                (opening_width * 0.5) + (front_column_w * 0.5),
                front_y - (front_frame_depth * 0.5),
                opening_bottom + ((height - opening_bottom) * 0.5),
            )
        ),
        material=trim_black,
        name="front_right_column",
    )
    body.visual(
        Box((width, front_frame_depth, height - opening_top)),
        origin=Origin(
            xyz=(0.0, front_y - (front_frame_depth * 0.5), opening_top + ((height - opening_top) * 0.5))
        ),
        material=trim_black,
        name="top_fascia",
    )
    body.visual(
        Box((width, front_frame_depth, opening_bottom - foot_h)),
        origin=Origin(
            xyz=(0.0, front_y - (front_frame_depth * 0.5), foot_h + ((opening_bottom - foot_h) * 0.5))
        ),
        material=trim_black,
        name="lower_sill",
    )

    body.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(xyz=(-0.255, 0.0, 0.18), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=dark_steel,
        name="left_drive_boss",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(xyz=(0.255, 0.0, 0.18), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=dark_steel,
        name="right_bearing_boss",
    )

    for idx, sx in enumerate((-0.215, 0.215)):
        for jdx, sy in enumerate((-0.150, 0.150)):
            body.visual(
                Box((0.048, 0.040, foot_h)),
                origin=Origin(xyz=(sx, sy, foot_h * 0.5)),
                material=foot_rubber,
                name=f"foot_{idx}_{jdx}",
            )

    body.inertial = Inertial.from_geometry(
        Box((width, depth, height)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, height * 0.5)),
    )

    door = model.part("door")
    door_width = 0.50
    door_height = opening_height
    door_thickness = 0.028
    frame_w = 0.034
    rail_h = 0.034
    glass_width = 0.432
    glass_height = 0.172
    door.visual(
        Box((frame_w, door_thickness, door_height)),
        origin=Origin(xyz=(-(door_width * 0.5) + (frame_w * 0.5), door_thickness * 0.5, door_height * 0.5)),
        material=trim_black,
        name="left_door_rail",
    )
    door.visual(
        Box((frame_w, door_thickness, door_height)),
        origin=Origin(xyz=((door_width * 0.5) - (frame_w * 0.5), door_thickness * 0.5, door_height * 0.5)),
        material=trim_black,
        name="right_door_rail",
    )
    door.visual(
        Box((door_width, door_thickness, rail_h)),
        origin=Origin(xyz=(0.0, door_thickness * 0.5, rail_h * 0.5)),
        material=trim_black,
        name="bottom_door_rail",
    )
    door.visual(
        Box((door_width, door_thickness, rail_h)),
        origin=Origin(xyz=(0.0, door_thickness * 0.5, door_height - (rail_h * 0.5))),
        material=trim_black,
        name="top_door_rail",
    )
    door.visual(
        Box((glass_width, door_thickness * 0.28, glass_height)),
        origin=Origin(xyz=(0.0, 0.008, door_height * 0.5)),
        material=glass,
        name="door_glass",
    )
    door.visual(
        Box((0.032, 0.022, 0.038)),
        origin=Origin(xyz=(-0.120, 0.020, 0.183)),
        material=trim_black,
        name="left_handle_mount",
    )
    door.visual(
        Box((0.032, 0.022, 0.038)),
        origin=Origin(xyz=(0.120, 0.020, 0.183)),
        material=trim_black,
        name="right_handle_mount",
    )
    door.visual(
        Cylinder(radius=0.006, length=0.030),
        origin=Origin(xyz=(-0.120, door_thickness + 0.014, 0.183), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=steel,
        name="left_handle_post",
    )
    door.visual(
        Cylinder(radius=0.006, length=0.030),
        origin=Origin(xyz=(0.120, door_thickness + 0.014, 0.183), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=steel,
        name="right_handle_post",
    )
    door.visual(
        Cylinder(radius=0.009, length=0.340),
        origin=Origin(xyz=(0.0, door_thickness + 0.028, 0.183), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=steel,
        name="handle_bar",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_width, 0.070, door_height)),
        mass=2.6,
        origin=Origin(xyz=(0.0, 0.035, door_height * 0.5)),
    )

    spit = model.part("rotisserie_spit")
    spit.visual(
        Cylinder(radius=0.0035, length=0.408),
        origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
        material=steel,
        name="spit_rod",
    )
    spit.visual(
        Cylinder(radius=0.005, length=0.042),
        origin=Origin(xyz=(-0.225, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=dark_steel,
        name="left_journal",
    )
    spit.visual(
        Cylinder(radius=0.005, length=0.042),
        origin=Origin(xyz=(0.225, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=dark_steel,
        name="right_journal",
    )
    spit.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(xyz=(-0.10, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=dark_steel,
        name="left_fork_collar",
    )
    spit.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(xyz=(0.10, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=dark_steel,
        name="right_fork_collar",
    )
    spit.visual(
        Box((0.010, 0.005, 0.090)),
        origin=Origin(xyz=(-0.10, 0.012, 0.045)),
        material=steel,
        name="left_upper_front_tine",
    )
    spit.visual(
        Box((0.010, 0.005, 0.090)),
        origin=Origin(xyz=(-0.10, -0.012, 0.045)),
        material=steel,
        name="left_upper_rear_tine",
    )
    spit.visual(
        Box((0.010, 0.005, 0.090)),
        origin=Origin(xyz=(-0.10, 0.012, -0.045)),
        material=steel,
        name="left_lower_front_tine",
    )
    spit.visual(
        Box((0.010, 0.005, 0.090)),
        origin=Origin(xyz=(-0.10, -0.012, -0.045)),
        material=steel,
        name="left_lower_rear_tine",
    )
    spit.visual(
        Box((0.010, 0.005, 0.090)),
        origin=Origin(xyz=(0.10, 0.012, 0.045)),
        material=steel,
        name="right_upper_front_tine",
    )
    spit.visual(
        Box((0.010, 0.005, 0.090)),
        origin=Origin(xyz=(0.10, -0.012, 0.045)),
        material=steel,
        name="right_upper_rear_tine",
    )
    spit.visual(
        Box((0.010, 0.005, 0.090)),
        origin=Origin(xyz=(0.10, 0.012, -0.045)),
        material=steel,
        name="right_lower_front_tine",
    )
    spit.visual(
        Box((0.010, 0.005, 0.090)),
        origin=Origin(xyz=(0.10, -0.012, -0.045)),
        material=steel,
        name="right_lower_rear_tine",
    )
    spit.inertial = Inertial.from_geometry(
        Box((0.492, 0.060, 0.180)),
        mass=0.45,
        origin=Origin(),
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.0, front_y, opening_bottom)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.4,
            lower=0.0,
            upper=math.radians(92.0),
        ),
    )
    model.articulation(
        "spit_rotation",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=spit,
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=7.0,
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
    body = object_model.get_part("oven_body")
    door = object_model.get_part("door")
    spit = object_model.get_part("rotisserie_spit")
    door_hinge = object_model.get_articulation("door_hinge")
    spit_rotation = object_model.get_articulation("spit_rotation")

    ctx.check("body part exists", body is not None)
    ctx.check("door part exists", door is not None)
    ctx.check("spit part exists", spit is not None)

    ctx.expect_gap(
        door,
        body,
        axis="y",
        min_gap=0.0,
        max_gap=0.001,
        name="door closes flush against the front frame",
    )
    ctx.expect_overlap(
        door,
        body,
        axes="xz",
        min_overlap=0.20,
        name="door covers the oven opening in the closed pose",
    )

    ctx.expect_contact(
        spit,
        body,
        elem_a="left_journal",
        elem_b="left_drive_boss",
        name="left spit journal is seated on the drive boss",
    )
    ctx.expect_contact(
        spit,
        body,
        elem_a="right_journal",
        elem_b="right_bearing_boss",
        name="right spit journal is seated on the support boss",
    )

    closed_handle = ctx.part_element_world_aabb(door, elem="handle_bar")
    with ctx.pose({door_hinge: math.radians(80.0)}):
        open_handle = ctx.part_element_world_aabb(door, elem="handle_bar")
    closed_ok = closed_handle is not None
    open_ok = open_handle is not None
    if closed_ok and open_ok:
        closed_handle_center = tuple(
            (closed_handle[0][axis] + closed_handle[1][axis]) * 0.5 for axis in range(3)
        )
        open_handle_center = tuple(
            (open_handle[0][axis] + open_handle[1][axis]) * 0.5 for axis in range(3)
        )
    else:
        closed_handle_center = None
        open_handle_center = None
    ctx.check(
        "door drops downward and outward when opened",
        closed_handle_center is not None
        and open_handle_center is not None
        and open_handle_center[2] < closed_handle_center[2] - 0.11
        and open_handle_center[1] > closed_handle_center[1] + 0.10,
        details=f"closed={closed_handle_center}, open={open_handle_center}",
    )

    fork_rest = ctx.part_element_world_aabb(spit, elem="left_upper_front_tine")
    with ctx.pose({spit_rotation: math.pi * 0.5}):
        fork_quarter_turn = ctx.part_element_world_aabb(spit, elem="left_upper_front_tine")
    fork_rest_center = None
    fork_quarter_turn_center = None
    if fork_rest is not None:
        fork_rest_center = tuple((fork_rest[0][axis] + fork_rest[1][axis]) * 0.5 for axis in range(3))
    if fork_quarter_turn is not None:
        fork_quarter_turn_center = tuple(
            (fork_quarter_turn[0][axis] + fork_quarter_turn[1][axis]) * 0.5 for axis in range(3)
        )
    ctx.check(
        "spit rotates its fork tines around the horizontal axis",
        fork_rest_center is not None
        and fork_quarter_turn_center is not None
        and abs(fork_quarter_turn_center[1] - fork_rest_center[1]) > 0.03
        and abs(fork_quarter_turn_center[2] - fork_rest_center[2]) > 0.03,
        details=f"rest={fork_rest_center}, quarter_turn={fork_quarter_turn_center}",
    )

    limits = spit_rotation.motion_limits
    ctx.check(
        "spit articulation is continuous",
        limits is not None and limits.lower is None and limits.upper is None,
        details=f"limits={limits}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
