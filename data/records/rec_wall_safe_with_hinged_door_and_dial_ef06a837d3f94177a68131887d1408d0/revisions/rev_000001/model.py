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
    model = ArticulatedObject(name="wall_safe")

    body_w = 0.40
    body_h = 0.48
    body_d = 0.24
    wall_t = 0.018

    door_gap = 0.006
    door_w = body_w - (2.0 * door_gap)
    door_h = body_h - (2.0 * door_gap)
    door_t = 0.032

    shelf_t = 0.014
    shelf_z = -0.020

    drawer_front_w = 0.312
    drawer_front_h = 0.092
    drawer_front_t = 0.010
    drawer_depth = 0.150
    drawer_side_t = 0.008
    drawer_bottom_t = 0.006
    drawer_closed_x = (body_d * 0.5) - wall_t - 0.010
    drawer_z = -0.145
    drawer_travel = 0.100

    safe_paint = model.material("safe_paint", rgba=(0.18, 0.20, 0.22, 1.0))
    safe_interior = model.material("safe_interior", rgba=(0.24, 0.25, 0.27, 1.0))
    steel_trim = model.material("steel_trim", rgba=(0.70, 0.72, 0.75, 1.0))
    dark_hardware = model.material("dark_hardware", rgba=(0.10, 0.10, 0.11, 1.0))

    body = model.part("safe_body")

    body.visual(
        Box((wall_t, body_w, body_h)),
        origin=Origin(xyz=(-(body_d * 0.5) + (wall_t * 0.5), 0.0, 0.0)),
        material=safe_paint,
        name="body_back",
    )
    body.visual(
        Box((body_d, wall_t, body_h)),
        origin=Origin(xyz=(0.0, -(body_w * 0.5) + (wall_t * 0.5), 0.0)),
        material=safe_paint,
        name="body_left_wall",
    )
    body.visual(
        Box((body_d, wall_t, body_h)),
        origin=Origin(xyz=(0.0, (body_w * 0.5) - (wall_t * 0.5), 0.0)),
        material=safe_paint,
        name="body_right_wall",
    )
    body.visual(
        Box((body_d, body_w, wall_t)),
        origin=Origin(xyz=(0.0, 0.0, -(body_h * 0.5) + (wall_t * 0.5))),
        material=safe_paint,
        name="body_floor",
    )
    body.visual(
        Box((body_d, body_w, wall_t)),
        origin=Origin(xyz=(0.0, 0.0, (body_h * 0.5) - (wall_t * 0.5))),
        material=safe_paint,
        name="body_roof",
    )
    body.visual(
        Box((body_d - 0.018, body_w - (2.0 * wall_t) + 0.002, shelf_t)),
        origin=Origin(xyz=(-0.010, 0.0, shelf_z)),
        material=safe_interior,
        name="interior_shelf",
    )
    body.visual(
        Box((0.110, 0.016, 0.010)),
        origin=Origin(xyz=(0.030, -0.174, -0.165)),
        material=steel_trim,
        name="left_runner",
    )
    body.visual(
        Box((0.110, 0.016, 0.010)),
        origin=Origin(xyz=(0.030, 0.174, -0.165)),
        material=steel_trim,
        name="right_runner",
    )
    body.inertial = Inertial.from_geometry(Box((body_d, body_w, body_h)), mass=36.0)

    door = model.part("door")
    door.visual(
        Box((door_t, door_w, door_h)),
        origin=Origin(xyz=(door_t * 0.5, -(door_w * 0.5), 0.0)),
        material=safe_paint,
        name="door_panel",
    )
    door.visual(
        Box((door_t * 0.72, door_w - 0.050, door_h - 0.050)),
        origin=Origin(xyz=(door_t * 0.44, -(door_w * 0.5), 0.0)),
        material=safe_interior,
        name="door_inner_plate",
    )

    handle_y = -(door_w * 0.5)
    handle_z = -0.050
    handle_post_x = door_t + 0.010
    handle_bar_x = door_t + 0.024
    handle_span = 0.082
    handle_post_offset = 0.024

    door.visual(
        Cylinder(radius=0.006, length=0.020),
        origin=Origin(
            xyz=(handle_post_x, handle_y - handle_post_offset, handle_z),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=dark_hardware,
        name="handle_post_upper",
    )
    door.visual(
        Cylinder(radius=0.006, length=0.020),
        origin=Origin(
            xyz=(handle_post_x, handle_y + handle_post_offset, handle_z),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=dark_hardware,
        name="handle_post_lower",
    )
    door.visual(
        Cylinder(radius=0.007, length=handle_span),
        origin=Origin(
            xyz=(handle_bar_x, handle_y, handle_z),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=dark_hardware,
        name="handle_bar",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_t, door_w, door_h)),
        mass=18.0,
        origin=Origin(xyz=(door_t * 0.5, -(door_w * 0.5), 0.0)),
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.014, length=0.006),
        origin=Origin(xyz=(0.003, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=steel_trim,
        name="dial_shaft",
    )
    dial.visual(
        Cylinder(radius=0.040, length=0.020),
        origin=Origin(xyz=(0.016, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=steel_trim,
        name="dial_ring",
    )
    dial.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(0.031, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=dark_hardware,
        name="dial_knob",
    )
    dial.inertial = Inertial.from_geometry(
        Cylinder(radius=0.040, length=0.036),
        mass=0.6,
        origin=Origin(xyz=(0.018, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
    )

    drawer = model.part("service_drawer")
    drawer.visual(
        Box((drawer_front_t, drawer_front_w, drawer_front_h)),
        origin=Origin(xyz=(-(drawer_front_t * 0.5), 0.0, 0.0)),
        material=safe_paint,
        name="drawer_front",
    )
    drawer.visual(
        Box((drawer_depth, drawer_front_w - (2.0 * drawer_side_t), drawer_bottom_t)),
        origin=Origin(
            xyz=(-(drawer_front_t + (drawer_depth * 0.5)), 0.0, -(drawer_front_h * 0.5) + 0.010)
        ),
        material=safe_interior,
        name="drawer_bottom",
    )
    drawer.visual(
        Box((drawer_depth, drawer_side_t, drawer_front_h - 0.018)),
        origin=Origin(
            xyz=(-(drawer_front_t + (drawer_depth * 0.5)), -((drawer_front_w * 0.5) - (drawer_side_t * 0.5)), -0.004)
        ),
        material=safe_interior,
        name="drawer_left_side",
    )
    drawer.visual(
        Box((drawer_depth, drawer_side_t, drawer_front_h - 0.018)),
        origin=Origin(
            xyz=(-(drawer_front_t + (drawer_depth * 0.5)), (drawer_front_w * 0.5) - (drawer_side_t * 0.5), -0.004)
        ),
        material=safe_interior,
        name="drawer_right_side",
    )
    drawer.visual(
        Box((drawer_side_t, drawer_front_w - (2.0 * drawer_side_t), drawer_front_h - 0.022)),
        origin=Origin(
            xyz=(-(drawer_front_t + drawer_depth - (drawer_side_t * 0.5)), 0.0, -0.006)
        ),
        material=safe_interior,
        name="drawer_back",
    )
    drawer.visual(
        Box((0.110, 0.016, 0.010)),
        origin=Origin(xyz=(-0.062, -0.158, -0.020)),
        material=steel_trim,
        name="drawer_left_glide",
    )
    drawer.visual(
        Box((0.110, 0.016, 0.010)),
        origin=Origin(xyz=(-0.062, 0.158, -0.020)),
        material=steel_trim,
        name="drawer_right_glide",
    )
    drawer.visual(
        Box((0.010, 0.120, 0.016)),
        origin=Origin(xyz=(0.005, 0.0, 0.016)),
        material=dark_hardware,
        name="drawer_pull",
    )
    drawer.inertial = Inertial.from_geometry(
        Box((drawer_front_t + drawer_depth, drawer_front_w, drawer_front_h)),
        mass=2.8,
        origin=Origin(xyz=(-0.080, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=((body_d * 0.5), (body_w * 0.5) - door_gap, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(105.0),
        ),
    )
    model.articulation(
        "door_to_dial",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=dial,
        origin=Origin(xyz=(door_t, -(door_w * 0.5), 0.095)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=12.0),
    )
    model.articulation(
        "body_to_service_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(drawer_closed_x, 0.0, drawer_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.18,
            lower=0.0,
            upper=drawer_travel,
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
    body = object_model.get_part("safe_body")
    door = object_model.get_part("door")
    dial = object_model.get_part("dial")
    drawer = object_model.get_part("service_drawer")

    door_joint = object_model.get_articulation("body_to_door")
    dial_joint = object_model.get_articulation("door_to_dial")
    drawer_joint = object_model.get_articulation("body_to_service_drawer")

    ctx.check("safe body exists", body is not None)
    ctx.check("door exists", door is not None)
    ctx.check("dial exists", dial is not None)
    ctx.check("service drawer exists", drawer is not None)

    with ctx.pose({door_joint: 0.0, drawer_joint: 0.0, dial_joint: 0.0}):
        ctx.expect_gap(
            door,
            body,
            axis="x",
            max_gap=0.001,
            max_penetration=0.0,
            name="closed door seats against safe front",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="yz",
            min_overlap=0.35,
            name="closed door covers the safe opening",
        )
        ctx.expect_contact(
            dial,
            door,
            elem_a="dial_shaft",
            elem_b="door_panel",
            contact_tol=1e-6,
            name="dial shaft stays mounted on door face",
        )

    door_closed_aabb = ctx.part_world_aabb(door)
    door_open_aabb = None
    with ctx.pose({door_joint: math.radians(80.0)}):
        door_open_aabb = ctx.part_world_aabb(door)

    if door_closed_aabb is not None and door_open_aabb is not None:
        closed_center_x = 0.5 * (door_closed_aabb[0][0] + door_closed_aabb[1][0])
        open_center_x = 0.5 * (door_open_aabb[0][0] + door_open_aabb[1][0])
        ctx.check(
            "door opens outward on right-side hinge",
            open_center_x > (closed_center_x + 0.10),
            details=f"closed_center_x={closed_center_x:.4f}, open_center_x={open_center_x:.4f}",
        )

    drawer_rest_pos = None
    drawer_extended_pos = None
    with ctx.pose({door_joint: math.radians(100.0), drawer_joint: 0.0}):
        ctx.expect_within(
            drawer,
            body,
            axes="yz",
            margin=0.0,
            name="drawer stays centered in lower compartment when closed",
        )
        ctx.expect_overlap(
            drawer,
            body,
            axes="x",
            min_overlap=0.12,
            name="drawer remains deeply inserted when closed",
        )
        ctx.expect_gap(
            body,
            drawer,
            axis="z",
            positive_elem="interior_shelf",
            min_gap=0.040,
            name="drawer sits below the interior shelf",
        )
        drawer_rest_pos = ctx.part_world_position(drawer)

    with ctx.pose({door_joint: math.radians(100.0), drawer_joint: 0.100, dial_joint: math.pi}):
        ctx.expect_within(
            drawer,
            body,
            axes="yz",
            margin=0.0,
            name="drawer stays aligned on runners when extended",
        )
        ctx.expect_overlap(
            drawer,
            body,
            axes="x",
            min_overlap=0.040,
            name="drawer retains insertion at full extension",
        )
        ctx.expect_contact(
            dial,
            door,
            elem_a="dial_shaft",
            elem_b="door_panel",
            contact_tol=1e-6,
            name="dial remains seated while spinning",
        )
        drawer_extended_pos = ctx.part_world_position(drawer)

    if drawer_rest_pos is not None and drawer_extended_pos is not None:
        ctx.check(
            "drawer extends outward",
            drawer_extended_pos[0] > (drawer_rest_pos[0] + 0.08),
            details=f"rest={drawer_rest_pos}, extended={drawer_extended_pos}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
