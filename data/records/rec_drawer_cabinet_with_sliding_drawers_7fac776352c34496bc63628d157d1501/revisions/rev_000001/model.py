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


def _add_caster(
    model: ArticulatedObject,
    body,
    *,
    name: str,
    mount_xyz: tuple[float, float, float],
    metal,
    wheel_rubber,
) -> None:
    caster = model.part(name)
    caster.visual(
        Box((0.055, 0.045, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=metal,
        name="mount_plate",
    )
    caster.visual(
        Cylinder(radius=0.010, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=metal,
        name="swivel_stem",
    )
    caster.visual(
        Box((0.030, 0.024, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=metal,
        name="yoke_block",
    )
    caster.visual(
        Box((0.006, 0.022, 0.038)),
        origin=Origin(xyz=(-0.010, 0.0, -0.051)),
        material=metal,
        name="left_fork",
    )
    caster.visual(
        Box((0.006, 0.022, 0.038)),
        origin=Origin(xyz=(0.010, 0.0, -0.051)),
        material=metal,
        name="right_fork",
    )
    caster.visual(
        Cylinder(radius=0.0035, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, -0.060), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="axle",
    )
    caster.visual(
        Cylinder(radius=0.020, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.060), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=wheel_rubber,
        name="wheel",
    )
    caster.inertial = Inertial.from_geometry(
        Box((0.055, 0.045, 0.086)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, -0.043)),
    )
    model.articulation(
        f"{name}_mount",
        ArticulationType.FIXED,
        parent=body,
        child=caster,
        origin=Origin(xyz=mount_xyz),
    )


def _add_drawer(
    model: ArticulatedObject,
    body,
    *,
    index: int,
    bottom_z: float,
    front_y: float,
    face_width: float,
    face_height: float,
    front_t: float,
    box_width: float,
    box_depth: float,
    box_height: float,
    extend_limit: float,
    drawer_front,
    drawer_shell,
    handle_metal,
) -> None:
    side_t = 0.008
    bottom_t = 0.008
    back_t = 0.008
    runner_t = 0.010
    runner_h = 0.014
    runner_len = 0.330
    runner_x = box_width / 2.0 + 0.0405

    drawer = model.part(f"drawer_{index}")
    drawer.visual(
        Box((face_width, front_t, face_height)),
        origin=Origin(xyz=(0.0, front_t / 2.0, face_height / 2.0)),
        material=drawer_front,
        name="front_panel",
    )
    drawer.visual(
        Box((box_width - 2.0 * side_t, box_depth, bottom_t)),
        origin=Origin(xyz=(0.0, -box_depth / 2.0, bottom_t / 2.0)),
        material=drawer_shell,
        name="drawer_bottom",
    )
    drawer.visual(
        Box((side_t, box_depth, box_height)),
        origin=Origin(
            xyz=(-(box_width / 2.0) + side_t / 2.0, -box_depth / 2.0, box_height / 2.0)
        ),
        material=drawer_shell,
        name="left_side",
    )
    drawer.visual(
        Box((side_t, box_depth, box_height)),
        origin=Origin(
            xyz=((box_width / 2.0) - side_t / 2.0, -box_depth / 2.0, box_height / 2.0)
        ),
        material=drawer_shell,
        name="right_side",
    )
    drawer.visual(
        Box((box_width - 2.0 * side_t, back_t, box_height)),
        origin=Origin(
            xyz=(0.0, -(box_depth - back_t / 2.0), box_height / 2.0)
        ),
        material=drawer_shell,
        name="back_panel",
    )
    drawer.visual(
        Box((0.170, 0.018, 0.022)),
        origin=Origin(xyz=(0.0, front_t + 0.009, face_height * 0.53)),
        material=handle_metal,
        name="pull_handle",
    )
    drawer.visual(
        Box((runner_t, runner_len, runner_h)),
        origin=Origin(xyz=(-runner_x, -0.195, 0.060)),
        material=handle_metal,
        name="left_runner",
    )
    drawer.visual(
        Box((runner_t, runner_len, runner_h)),
        origin=Origin(xyz=(runner_x, -0.195, 0.060)),
        material=handle_metal,
        name="right_runner",
    )
    for side_sign in (-1.0, 1.0):
        for mount_y in (-0.085, -0.270):
            drawer.visual(
                Box((0.036, 0.030, 0.012)),
                origin=Origin(
                    xyz=(side_sign * (box_width / 2.0 + 0.018), mount_y, 0.060)
                ),
                material=handle_metal,
            )
    drawer.inertial = Inertial.from_geometry(
        Box((face_width, box_depth + front_t + 0.018, face_height)),
        mass=1.8,
        origin=Origin(xyz=(0.0, -0.140, face_height / 2.0)),
    )
    model.articulation(
        f"body_to_drawer_{index}",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(0.0, front_y, bottom_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.22,
            lower=0.0,
            upper=extend_limit,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dental_supply_cabinet")

    cabinet_white = model.material("cabinet_white", rgba=(0.92, 0.94, 0.95, 1.0))
    drawer_white = model.material("drawer_white", rgba=(0.96, 0.97, 0.98, 1.0))
    frame_grey = model.material("frame_grey", rgba=(0.76, 0.79, 0.82, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.60, 0.64, 0.68, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.15, 0.16, 0.17, 1.0))
    smoked_panel = model.material("smoked_panel", rgba=(0.58, 0.66, 0.71, 0.28))

    body_w = 0.420
    body_d = 0.500
    body_h = 0.820
    caster_h = 0.080
    wall_t = 0.018
    frame_t = 0.022
    center_column_w = 0.320
    stile_w = (body_w - center_column_w) / 2.0
    face_w = 0.304
    door_h = 0.220
    face_h = 0.160
    door_t = 0.016
    drawer_front_t = 0.018
    drawer_box_w = 0.276
    drawer_box_d = 0.340
    drawer_box_h = 0.112
    drawer_travel = 0.230
    fuse = 0.0015
    front_y = body_d / 2.0
    rail_t = 0.010
    rail_y = -0.002
    rail_x = body_w / 2.0 - wall_t - rail_t / 2.0 + fuse
    rail_len = 0.420
    rail_h = 0.016

    drawer_bottoms = (0.105, 0.279, 0.453)
    door_bottom = 0.627

    body = model.part("body")
    body.visual(
        Box((wall_t, body_d, body_h)),
        origin=Origin(xyz=(-(body_w - wall_t) / 2.0, 0.0, caster_h + body_h / 2.0)),
        material=cabinet_white,
        name="left_side_wall",
    )
    body.visual(
        Box((wall_t, body_d, body_h)),
        origin=Origin(xyz=((body_w - wall_t) / 2.0, 0.0, caster_h + body_h / 2.0)),
        material=cabinet_white,
        name="right_side_wall",
    )
    body.visual(
        Box((body_w - 2.0 * wall_t + 2.0 * fuse, wall_t, body_h)),
        origin=Origin(
            xyz=(0.0, -(body_d - wall_t) / 2.0, caster_h + body_h / 2.0)
        ),
        material=cabinet_white,
        name="back_panel",
    )
    body.visual(
        Box((body_w, body_d, wall_t)),
        origin=Origin(xyz=(0.0, 0.0, caster_h + wall_t / 2.0)),
        material=cabinet_white,
        name="bottom_panel",
    )
    body.visual(
        Box((body_w, body_d, wall_t)),
        origin=Origin(xyz=(0.0, 0.0, caster_h + body_h - wall_t / 2.0)),
        material=cabinet_white,
        name="top_panel",
    )
    body.visual(
        Box((stile_w + fuse, frame_t, body_h - 2.0 * wall_t + 2.0 * fuse)),
        origin=Origin(
            xyz=(
                -(center_column_w / 2.0 + stile_w / 2.0),
                (body_d - frame_t) / 2.0,
                caster_h + body_h / 2.0,
            )
        ),
        material=frame_grey,
        name="left_front_stile",
    )
    body.visual(
        Box((stile_w + fuse, frame_t, body_h - 2.0 * wall_t + 2.0 * fuse)),
        origin=Origin(
            xyz=(
                (center_column_w / 2.0 + stile_w / 2.0),
                (body_d - frame_t) / 2.0,
                caster_h + body_h / 2.0,
            )
        ),
        material=frame_grey,
        name="right_front_stile",
    )
    body.visual(
        Box((center_column_w + 2.0 * fuse, frame_t, 0.028)),
        origin=Origin(
            xyz=(0.0, (body_d - frame_t) / 2.0, caster_h + body_h - 0.014)
        ),
        material=frame_grey,
        name="top_header",
    )
    body.visual(
        Box((center_column_w + 2.0 * fuse, frame_t, 0.012)),
        origin=Origin(
            xyz=(0.0, (body_d - frame_t) / 2.0, door_bottom - 0.006)
        ),
        material=frame_grey,
        name="door_drawer_divider_face",
    )
    body.visual(
        Box((body_w - 2.0 * wall_t + 2.0 * fuse, body_d - wall_t - frame_t, 0.012)),
        origin=Origin(
            xyz=(
                0.0,
                ((body_d / 2.0) - frame_t + (-(body_d / 2.0) + wall_t)) / 2.0,
                door_bottom - 0.006,
            )
        ),
        material=cabinet_white,
        name="instrument_compartment_floor",
    )
    body.visual(
        Box((center_column_w + 2.0 * fuse, frame_t, 0.022)),
        origin=Origin(
            xyz=(0.0, (body_d - frame_t) / 2.0, caster_h + 0.011)
        ),
        material=frame_grey,
        name="lower_kick_rail",
    )

    for index, drawer_bottom in enumerate(drawer_bottoms, start=1):
        rail_z = drawer_bottom + 0.060
        body.visual(
            Box((rail_t, rail_len, rail_h)),
            origin=Origin(xyz=(-rail_x, rail_y, rail_z)),
            material=satin_metal,
            name=f"guide_rail_left_{index}",
        )
        body.visual(
            Box((rail_t, rail_len, rail_h)),
            origin=Origin(xyz=(rail_x, rail_y, rail_z)),
            material=satin_metal,
            name=f"guide_rail_right_{index}",
        )

    body.inertial = Inertial.from_geometry(
        Box((body_w, body_d, body_h + caster_h)),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, (body_h + caster_h) / 2.0)),
    )

    door = model.part("instrument_door")
    door.visual(
        Box((face_w, door_t, door_h)),
        origin=Origin(xyz=(face_w / 2.0, door_t / 2.0, door_h / 2.0)),
        material=drawer_white,
        name="door_panel",
    )
    door.visual(
        Box((face_w - 0.060, 0.006, door_h - 0.070)),
        origin=Origin(xyz=(face_w / 2.0 + 0.006, door_t - 0.003, door_h / 2.0)),
        material=smoked_panel,
        name="door_insert",
    )
    door.visual(
        Cylinder(radius=0.007, length=door_h - 0.018),
        origin=Origin(xyz=(0.0, door_t / 2.0, door_h / 2.0)),
        material=satin_metal,
        name="hinge_barrel",
    )
    door.visual(
        Box((0.016, 0.018, 0.102)),
        origin=Origin(
            xyz=(face_w - 0.026, door_t + 0.009, door_h / 2.0)
        ),
        material=satin_metal,
        name="door_pull",
    )
    door.inertial = Inertial.from_geometry(
        Box((face_w, door_t + 0.018, door_h)),
        mass=2.6,
        origin=Origin(xyz=(face_w / 2.0, 0.017, door_h / 2.0)),
    )
    model.articulation(
        "body_to_instrument_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(-face_w / 2.0, front_y, door_bottom)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.4,
            lower=0.0,
            upper=1.85,
        ),
    )

    for index, drawer_bottom in enumerate(drawer_bottoms, start=1):
        _add_drawer(
            model,
            body,
            index=index,
            bottom_z=drawer_bottom,
            front_y=front_y,
            face_width=face_w,
            face_height=face_h,
            front_t=drawer_front_t,
            box_width=drawer_box_w,
            box_depth=drawer_box_d,
            box_height=drawer_box_h,
            extend_limit=drawer_travel,
            drawer_front=drawer_white,
            drawer_shell=cabinet_white,
            handle_metal=satin_metal,
        )

    caster_x = body_w / 2.0 - 0.055
    caster_y = body_d / 2.0 - 0.065
    for caster_name, xyz in (
        ("caster_front_left", (-caster_x, caster_y, caster_h)),
        ("caster_front_right", (caster_x, caster_y, caster_h)),
        ("caster_rear_left", (-caster_x, -caster_y, caster_h)),
        ("caster_rear_right", (caster_x, -caster_y, caster_h)),
    ):
        _add_caster(
            model,
            body,
            name=caster_name,
            mount_xyz=xyz,
            metal=satin_metal,
            wheel_rubber=dark_rubber,
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

    body = object_model.get_part("body")
    door = object_model.get_part("instrument_door")
    drawers = [object_model.get_part(f"drawer_{index}") for index in (1, 2, 3)]
    door_joint = object_model.get_articulation("body_to_instrument_door")
    drawer_joints = [
        object_model.get_articulation(f"body_to_drawer_{index}") for index in (1, 2, 3)
    ]

    ctx.check(
        "all storage articulations exist",
        door_joint is not None and len(drawer_joints) == 3,
        details="Expected one door hinge and three drawer slides.",
    )
    ctx.check(
        "door hinge has cabinet-like swing range",
        door_joint.motion_limits is not None
        and door_joint.motion_limits.upper is not None
        and door_joint.motion_limits.upper >= 1.5,
        details=f"door limits={door_joint.motion_limits}",
    )

    with ctx.pose({door_joint: 0.0, **{joint: 0.0 for joint in drawer_joints}}):
        ctx.expect_gap(
            door,
            body,
            axis="y",
            positive_elem="door_panel",
            max_gap=0.002,
            max_penetration=0.0,
            name="door closes flush with cabinet face",
        )
        for index, drawer in enumerate(drawers, start=1):
            ctx.expect_gap(
                drawer,
                body,
                axis="y",
                positive_elem="front_panel",
                max_gap=0.002,
                max_penetration=0.0,
                name=f"drawer {index} front sits flush when closed",
            )

    closed_door_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({door_joint: 1.35}):
        open_door_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    ctx.check(
        "instrument door swings outward on its vertical hinge",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][1] > closed_door_aabb[1][1] + 0.18,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    for index, (drawer, joint) in enumerate(zip(drawers, drawer_joints), start=1):
        limits = joint.motion_limits
        upper = 0.0 if limits is None or limits.upper is None else limits.upper
        closed_pos = ctx.part_world_position(drawer)
        with ctx.pose({joint: upper}):
            extended_pos = ctx.part_world_position(drawer)
            ctx.expect_overlap(
                drawer,
                body,
                axes="y",
                elem_a="drawer_bottom",
                min_overlap=0.10,
                name=f"drawer {index} retains insertion on its guide rails",
            )
        ctx.check(
            f"drawer {index} extends forward",
            closed_pos is not None
            and extended_pos is not None
            and extended_pos[1] > closed_pos[1] + 0.18,
            details=f"closed={closed_pos}, extended={extended_pos}, upper={upper}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
