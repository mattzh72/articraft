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


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="document_wall_safe")

    body_paint = model.material("body_paint", rgba=(0.18, 0.19, 0.21, 1.0))
    frame_paint = model.material("frame_paint", rgba=(0.26, 0.27, 0.30, 1.0))
    door_paint = model.material("door_paint", rgba=(0.30, 0.31, 0.34, 1.0))
    interior_paint = model.material("interior_paint", rgba=(0.72, 0.73, 0.70, 1.0))
    rail_metal = model.material("rail_metal", rgba=(0.63, 0.65, 0.67, 1.0))
    drawer_finish = model.material("drawer_finish", rgba=(0.62, 0.64, 0.66, 1.0))
    drawer_front_finish = model.material("drawer_front_finish", rgba=(0.40, 0.42, 0.45, 1.0))
    dial_steel = model.material("dial_steel", rgba=(0.80, 0.81, 0.82, 1.0))
    dial_dark = model.material("dial_dark", rgba=(0.10, 0.11, 0.12, 1.0))

    outer_depth = 0.26
    outer_width = 0.64
    outer_height = 0.48
    shell_t = 0.018
    stop_t = 0.010

    safe_body = model.part("safe_body")

    safe_body.visual(
        Box((shell_t, outer_width, outer_height)),
        origin=Origin(xyz=(-outer_depth + shell_t * 0.5, 0.0, 0.0)),
        material=body_paint,
        name="back_wall",
    )
    safe_body.visual(
        Box((outer_depth - shell_t, shell_t, outer_height - 2.0 * shell_t)),
        origin=Origin(
            xyz=(
                -(outer_depth - shell_t) * 0.5,
                -(outer_width * 0.5 - shell_t * 0.5),
                0.0,
            )
        ),
        material=body_paint,
        name="left_wall",
    )
    safe_body.visual(
        Box((outer_depth - shell_t, shell_t, outer_height - 2.0 * shell_t)),
        origin=Origin(
            xyz=(
                -(outer_depth - shell_t) * 0.5,
                outer_width * 0.5 - shell_t * 0.5,
                0.0,
            )
        ),
        material=body_paint,
        name="right_wall",
    )
    safe_body.visual(
        Box((outer_depth - shell_t, outer_width - 2.0 * shell_t, shell_t)),
        origin=Origin(
            xyz=(
                -(outer_depth - shell_t) * 0.5,
                0.0,
                outer_height * 0.5 - shell_t * 0.5,
            )
        ),
        material=body_paint,
        name="top_wall",
    )
    safe_body.visual(
        Box((outer_depth - shell_t, outer_width - 2.0 * shell_t, shell_t)),
        origin=Origin(
            xyz=(
                -(outer_depth - shell_t) * 0.5,
                0.0,
                -(outer_height * 0.5 - shell_t * 0.5),
            )
        ),
        material=body_paint,
        name="bottom_wall",
    )

    inner_width = outer_width - 2.0 * shell_t
    inner_height = outer_height - 2.0 * shell_t
    opening_width = inner_width - 2.0 * stop_t
    opening_height = inner_height - 2.0 * stop_t

    safe_body.visual(
        Box((stop_t, stop_t, inner_height)),
        origin=Origin(
            xyz=(-stop_t * 0.5, -(inner_width * 0.5 - stop_t * 0.5), 0.0)
        ),
        material=frame_paint,
        name="left_jamb_stop",
    )
    safe_body.visual(
        Box((stop_t, stop_t, inner_height)),
        origin=Origin(
            xyz=(-stop_t * 0.5, inner_width * 0.5 - stop_t * 0.5, 0.0)
        ),
        material=frame_paint,
        name="right_jamb_stop",
    )
    safe_body.visual(
        Box((stop_t, opening_width, stop_t)),
        origin=Origin(
            xyz=(-stop_t * 0.5, 0.0, inner_height * 0.5 - stop_t * 0.5)
        ),
        material=frame_paint,
        name="top_stop",
    )
    safe_body.visual(
        Box((stop_t, opening_width, stop_t)),
        origin=Origin(
            xyz=(-stop_t * 0.5, 0.0, -(inner_height * 0.5 - stop_t * 0.5))
        ),
        material=frame_paint,
        name="threshold_stop",
    )

    runner_length = 0.17
    runner_width = 0.072
    runner_height = 0.008
    runner_center_x = -0.12
    runner_center_z = -0.134
    runner_center_y = inner_width * 0.5 - runner_width * 0.5
    safe_body.visual(
        Box((runner_length, runner_width, runner_height)),
        origin=Origin(xyz=(runner_center_x, -runner_center_y, runner_center_z)),
        material=rail_metal,
        name="left_runner",
    )
    safe_body.visual(
        Box((runner_length, runner_width, runner_height)),
        origin=Origin(xyz=(runner_center_x, runner_center_y, runner_center_z)),
        material=rail_metal,
        name="right_runner",
    )

    safe_body.inertial = Inertial.from_geometry(
        Box((outer_depth, outer_width, outer_height)),
        mass=32.0,
        origin=Origin(xyz=(-outer_depth * 0.5, 0.0, 0.0)),
    )

    door = model.part("door")
    door_thickness = 0.032
    door_width = outer_width - 0.022
    door_height = outer_height - 0.022
    door_x_center = door_thickness * 0.5

    door.visual(
        Box((door_thickness, door_width, door_height)),
        origin=Origin(xyz=(door_x_center, door_width * 0.5, 0.0)),
        material=door_paint,
        name="door_shell",
    )
    for idx, z_center in enumerate((0.145, 0.0, -0.145), start=1):
        door.visual(
            Cylinder(radius=0.011, length=0.105),
            origin=Origin(xyz=(0.0115, 0.011, z_center)),
            material=frame_paint,
            name=f"hinge_knuckle_{idx}",
        )
    door.visual(
        Box((0.004, 0.012, 0.006)),
        origin=Origin(xyz=(door_thickness + 0.002, 0.400, 0.090)),
        material=dial_steel,
        name="dial_index",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_thickness, door_width, door_height)),
        mass=18.0,
        origin=Origin(xyz=(door_x_center, door_width * 0.5, 0.0)),
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.050, length=0.020),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dial_steel,
        name="dial_bezel",
    )
    dial.visual(
        Cylinder(radius=0.018, length=0.028),
        origin=Origin(xyz=(0.020, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dial_dark,
        name="dial_hub",
    )
    dial.visual(
        Box((0.010, 0.012, 0.024)),
        origin=Origin(xyz=(0.031, 0.0, 0.024)),
        material=dial_dark,
        name="dial_grip",
    )
    dial.inertial = Inertial.from_geometry(
        Cylinder(radius=0.050, length=0.028),
        mass=0.6,
        origin=Origin(xyz=(0.014, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    drawer = model.part("drawer")
    drawer_depth = 0.18
    drawer_width = 0.46
    drawer_height = 0.08
    drawer_wall_t = 0.012

    drawer.visual(
        Box((drawer_depth, drawer_width, drawer_wall_t)),
        origin=Origin(xyz=(0.0, 0.0, -(drawer_height * 0.5 - drawer_wall_t * 0.5))),
        material=drawer_finish,
        name="drawer_bottom",
    )
    drawer.visual(
        Box((drawer_depth, drawer_wall_t, drawer_height - drawer_wall_t)),
        origin=Origin(
            xyz=(0.0, -(drawer_width * 0.5 - drawer_wall_t * 0.5), drawer_wall_t * 0.5)
        ),
        material=drawer_finish,
        name="drawer_left_side",
    )
    drawer.visual(
        Box((drawer_depth, drawer_wall_t, drawer_height - drawer_wall_t)),
        origin=Origin(
            xyz=(0.0, drawer_width * 0.5 - drawer_wall_t * 0.5, drawer_wall_t * 0.5)
        ),
        material=drawer_finish,
        name="drawer_right_side",
    )
    drawer.visual(
        Box((drawer_wall_t, drawer_width - 2.0 * drawer_wall_t, drawer_height - drawer_wall_t)),
        origin=Origin(
            xyz=(
                -(drawer_depth * 0.5 - drawer_wall_t * 0.5),
                0.0,
                drawer_wall_t * 0.5,
            )
        ),
        material=drawer_finish,
        name="drawer_back",
    )
    drawer.visual(
        Box((0.016, drawer_width - 2.0 * drawer_wall_t, drawer_height)),
        origin=Origin(xyz=(drawer_depth * 0.5 - 0.008, 0.0, 0.0)),
        material=drawer_front_finish,
        name="drawer_front",
    )
    drawer.visual(
        Box((0.012, 0.18, 0.016)),
        origin=Origin(xyz=(drawer_depth * 0.5, 0.0, 0.010)),
        material=drawer_front_finish,
        name="drawer_pull",
    )
    drawer.inertial = Inertial.from_geometry(
        Box((drawer_depth, drawer_width, drawer_height)),
        mass=2.0,
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=safe_body,
        child=door,
        origin=Origin(xyz=(0.0, -(door_width * 0.5), 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(100.0),
        ),
    )
    model.articulation(
        "dial_spin",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=dial,
        origin=Origin(xyz=(door_thickness, 0.400, 0.030)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=12.0),
    )
    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=safe_body,
        child=drawer,
        origin=Origin(xyz=(-0.120, 0.0, -0.090)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.30,
            lower=0.0,
            upper=0.120,
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

    safe_body = object_model.get_part("safe_body")
    door = object_model.get_part("door")
    dial = object_model.get_part("dial")
    drawer = object_model.get_part("drawer")

    door_hinge = object_model.get_articulation("door_hinge")
    dial_spin = object_model.get_articulation("dial_spin")
    drawer_slide = object_model.get_articulation("drawer_slide")

    ctx.check(
        "door hinge axis is vertical at the left jamb",
        tuple(round(v, 3) for v in door_hinge.axis) == (0.0, 0.0, -1.0),
        details=f"axis={door_hinge.axis}",
    )
    ctx.check(
        "dial spins about its front-to-back center shaft",
        tuple(round(v, 3) for v in dial_spin.axis) == (1.0, 0.0, 0.0),
        details=f"axis={dial_spin.axis}",
    )
    ctx.check(
        "drawer slides straight out of the safe body",
        tuple(round(v, 3) for v in drawer_slide.axis) == (1.0, 0.0, 0.0),
        details=f"axis={drawer_slide.axis}",
    )

    with ctx.pose({door_hinge: 0.0, drawer_slide: 0.0}):
        ctx.expect_gap(
            door,
            safe_body,
            axis="x",
            positive_elem="door_shell",
            max_gap=0.004,
            max_penetration=0.0,
            name="door sits nearly flush to the front frame when closed",
        )
        ctx.expect_overlap(
            door,
            safe_body,
            axes="yz",
            elem_a="door_shell",
            min_overlap=0.44,
            name="closed door covers the wide opening",
        )
        ctx.expect_within(
            drawer,
            safe_body,
            axes="yz",
            margin=0.006,
            name="drawer stays centered within the body at rest",
        )

        drawer_rest_pos = ctx.part_world_position(drawer)
        door_closed_aabb = ctx.part_element_world_aabb(door, elem="door_shell")
        dial_grip_rest = _aabb_center(ctx.part_element_world_aabb(dial, elem="dial_grip"))

    with ctx.pose({door_hinge: math.radians(85.0)}):
        door_open_aabb = ctx.part_element_world_aabb(door, elem="door_shell")

    ctx.check(
        "door swings outward on the left hinge",
        door_closed_aabb is not None
        and door_open_aabb is not None
        and door_open_aabb[1][0] > door_closed_aabb[1][0] + 0.18,
        details=f"closed={door_closed_aabb}, open={door_open_aabb}",
    )

    with ctx.pose({dial_spin: math.pi * 0.5}):
        dial_grip_quarter_turn = _aabb_center(ctx.part_element_world_aabb(dial, elem="dial_grip"))

    ctx.check(
        "dial grip visibly changes position under continuous rotation",
        dial_grip_rest is not None
        and dial_grip_quarter_turn is not None
        and (
            abs(dial_grip_rest[1] - dial_grip_quarter_turn[1]) > 0.020
            or abs(dial_grip_rest[2] - dial_grip_quarter_turn[2]) > 0.020
        ),
        details=f"rest={dial_grip_rest}, quarter_turn={dial_grip_quarter_turn}",
    )

    with ctx.pose({door_hinge: math.radians(92.0), drawer_slide: 0.120}):
        ctx.expect_within(
            drawer,
            safe_body,
            axes="yz",
            margin=0.006,
            name="drawer remains between the side walls when extended",
        )
        ctx.expect_overlap(
            drawer,
            safe_body,
            axes="x",
            min_overlap=0.08,
            name="drawer retains insertion on its runners at full extension",
        )
        drawer_extended_pos = ctx.part_world_position(drawer)

    ctx.check(
        "drawer extends outward from behind the threshold",
        drawer_rest_pos is not None
        and drawer_extended_pos is not None
        and drawer_extended_pos[0] > drawer_rest_pos[0] + 0.08,
        details=f"rest={drawer_rest_pos}, extended={drawer_extended_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
