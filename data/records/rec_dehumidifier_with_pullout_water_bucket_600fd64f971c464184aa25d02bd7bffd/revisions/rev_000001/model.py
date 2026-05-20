from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

BODY_W = 0.34
BODY_D = 0.245
BODY_H = 0.56

BUCKET_W = 0.278
BUCKET_D = 0.205
BUCKET_H = 0.175
BUCKET_FLOOR_Z = 0.050
BUCKET_TRAVEL = 0.110

HANDLE_PIVOT_Z = 0.536
HANDLE_PIVOT_Y = -0.030

FILTER_DOOR_D = 0.155
FILTER_DOOR_H = 0.236
FILTER_HINGE_Y = 0.105
FILTER_CENTER_Z = 0.382

PANEL_T = 0.018
PANEL_D = 0.072
PANEL_H = 0.124
PANEL_CENTER_Y = -0.092
PANEL_CENTER_Z = 0.410

BUTTON_TRAVEL = 0.0035


def _x_cylinder(length: float, radius: float, *, center: tuple[float, float, float]) -> cq.Workplane:
    x, y, z = center
    return cq.Workplane("YZ").circle(radius).extrude(length).translate((x - length / 2.0, y, z))


def _y_cylinder(length: float, radius: float, *, center: tuple[float, float, float]) -> cq.Workplane:
    x, y, z = center
    return cq.Workplane("XZ").circle(radius).extrude(length).translate((x, y - length / 2.0, z))


def _body_shape() -> cq.Workplane:
    side_t = 0.022
    top_t = 0.028
    back_t = 0.018
    front_t = 0.022
    front_upper_h = 0.300
    front_upper_center_z = 0.410
    plinth_h = 0.048
    floor_t = 0.008
    inner_w = BODY_W - 2.0 * side_t

    left_wall = cq.Workplane("XY").box(side_t, BODY_D, BODY_H - 0.004).translate(
        (-BODY_W / 2.0 + side_t / 2.0, 0.0, (BODY_H - 0.004) / 2.0)
    )

    right_wall = cq.Workplane("XY").box(side_t, BODY_D, BODY_H - 0.004).translate(
        (BODY_W / 2.0 - side_t / 2.0, 0.0, (BODY_H - 0.004) / 2.0)
    )
    filter_opening = cq.Workplane("XY").box(side_t + 0.004, 0.142, 0.224).translate(
        (BODY_W / 2.0 - side_t / 2.0, 0.025, FILTER_CENTER_Z)
    )
    right_wall = right_wall.cut(filter_opening)

    top_cap = cq.Workplane("XY").box(inner_w, BODY_D, top_t).translate((0.0, 0.0, BODY_H - top_t / 2.0))
    top_recess = cq.Workplane("XY").box(0.236, 0.086, 0.032).translate((0.0, -0.002, BODY_H - 0.016))
    top_cap = top_cap.cut(top_recess)
    for vent_y in (0.014, 0.028, 0.042, 0.056):
        slot = cq.Workplane("XY").box(0.152, 0.008, 0.024).translate((0.0, vent_y, BODY_H - 0.010))
        top_cap = top_cap.cut(slot)

    back_wall = cq.Workplane("XY").box(inner_w, back_t, BODY_H - top_t).translate(
        (0.0, BODY_D / 2.0 - back_t / 2.0, (BODY_H - top_t) / 2.0)
    )

    front_upper = cq.Workplane("XY").box(inner_w, front_t, front_upper_h).translate(
        (0.0, -BODY_D / 2.0 + front_t / 2.0, front_upper_center_z)
    )
    for slot_z in (0.336, 0.360, 0.384, 0.408, 0.432):
        slot = cq.Workplane("XY").box(0.206, front_t + 0.012, 0.010).translate(
            (0.0, -BODY_D / 2.0 + front_t / 2.0, slot_z)
        )
        front_upper = front_upper.cut(slot)

    front_plinth = cq.Workplane("XY").box(inner_w, front_t, plinth_h).translate(
        (0.0, -BODY_D / 2.0 + front_t / 2.0, plinth_h / 2.0)
    )

    bucket_floor = cq.Workplane("XY").box(inner_w, BUCKET_D, floor_t).translate(
        (0.0, -BODY_D / 2.0 + BUCKET_D / 2.0, BUCKET_FLOOR_Z - floor_t / 2.0)
    )

    shell = left_wall.union(right_wall)
    shell = shell.union(top_cap)
    shell = shell.union(back_wall)
    shell = shell.union(front_upper)
    shell = shell.union(front_plinth)
    shell = shell.union(bucket_floor)
    return shell


def _bucket_shape() -> cq.Workplane:
    wall_side = 0.008
    wall_back = 0.010
    wall_front = 0.014
    bottom = 0.010
    front_wall_h = 0.086

    bucket = cq.Workplane("XY").box(BUCKET_W, BUCKET_D, BUCKET_H).translate((0.0, BUCKET_D / 2.0, BUCKET_H / 2.0))
    bucket = bucket.edges("|Z").fillet(0.012)

    inner = cq.Workplane("XY").box(
        BUCKET_W - 2.0 * wall_side,
        BUCKET_D - wall_front - wall_back,
        BUCKET_H,
    ).translate(
        (
            0.0,
            wall_front + (BUCKET_D - wall_front - wall_back) / 2.0,
            bottom + BUCKET_H / 2.0,
        )
    )
    bucket = bucket.cut(inner)

    front_scoop = cq.Workplane("XY").box(BUCKET_W - 0.050, wall_front + 0.020, BUCKET_H).translate(
        (0.0, (wall_front + 0.020) / 2.0, front_wall_h + BUCKET_H / 2.0)
    )
    bucket = bucket.cut(front_scoop)

    finger_slot = (
        cq.Workplane("XY")
        .box(0.118, wall_front + 0.024, 0.030)
        .translate((0.0, (wall_front + 0.024) / 2.0, 0.045))
        .edges("|X")
        .fillet(0.010)
    )
    bucket = bucket.cut(finger_slot)

    return bucket


def _handle_shape() -> cq.Workplane:
    span = 0.230
    pivot_len = 0.026
    arm_depth = 0.056

    left_pivot = cq.Workplane("XY").box(pivot_len, 0.014, 0.014).translate((-span / 2.0, 0.0, 0.0))
    right_pivot = cq.Workplane("XY").box(pivot_len, 0.014, 0.014).translate((span / 2.0, 0.0, 0.0))
    left_arm = cq.Workplane("XY").box(0.014, arm_depth, 0.014).translate((-span / 2.0, arm_depth / 2.0, 0.0))
    right_arm = cq.Workplane("XY").box(0.014, arm_depth, 0.014).translate((span / 2.0, arm_depth / 2.0, 0.0))
    grip = cq.Workplane("XY").box(span, 0.016, 0.016).translate((0.0, arm_depth, 0.0))
    return left_pivot.union(right_pivot).union(left_arm).union(right_arm).union(grip)


def _filter_door_shape() -> cq.Workplane:
    thickness = 0.006
    door = cq.Workplane("YZ").rect(FILTER_DOOR_D, FILTER_DOOR_H).extrude(thickness).translate((0.0, -FILTER_DOOR_D / 2.0, 0.0))
    door = door.edges("|X").fillet(0.010)

    for slot_z in (-0.070, -0.044, -0.018, 0.008, 0.034, 0.060):
        slot = cq.Workplane("XY").box(thickness + 0.004, 0.120, 0.010).translate((thickness / 2.0, -0.078, slot_z))
        door = door.cut(slot)

    pull = cq.Workplane("XY").box(0.010, 0.028, 0.016).translate((0.008, -FILTER_DOOR_D + 0.020, -0.068))
    return door.union(pull)


def _control_panel_shape() -> cq.Workplane:
    panel = cq.Workplane("YZ").rect(PANEL_D, PANEL_H).extrude(PANEL_T)
    panel = panel.edges("|X").fillet(0.010)

    dial_well = (
        cq.Workplane("YZ")
        .circle(0.023)
        .extrude(0.006)
        .translate((PANEL_T - 0.006, 0.012, 0.028))
    )
    panel = panel.cut(dial_well)

    for button_z in (-0.010, -0.044):
        well = cq.Workplane("XY").box(0.010, 0.018, 0.012).translate((PANEL_T - 0.005, -0.012, button_z))
        panel = panel.cut(well)

    return panel


def _dial_shape() -> cq.Workplane:
    dial = cq.Workplane("YZ").circle(0.023).extrude(0.006)
    dial = dial.union(cq.Workplane("YZ").circle(0.017).extrude(0.018).translate((0.006, 0.0, 0.0)))
    pointer = cq.Workplane("XY").box(0.003, 0.0025, 0.010).translate((0.022, 0.0, 0.010))
    return dial.union(pointer)


def _button_shape() -> cq.Workplane:
    guide = cq.Workplane("YZ").rect(0.018, 0.012).extrude(0.006)
    cap = cq.Workplane("YZ").rect(0.016, 0.012).extrude(0.009).translate((0.003, 0.0, 0.0))
    return guide.union(cap)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bedroom_dehumidifier")

    body_finish = model.material("body_finish", rgba=(0.92, 0.93, 0.95, 1.0))
    vent_dark = model.material("vent_dark", rgba=(0.66, 0.69, 0.73, 1.0))
    bucket_finish = model.material("bucket_finish", rgba=(0.83, 0.86, 0.90, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.30, 0.33, 0.37, 1.0))
    control_dark = model.material("control_dark", rgba=(0.15, 0.17, 0.19, 1.0))
    button_light = model.material("button_light", rgba=(0.95, 0.96, 0.97, 1.0))

    body = model.part("body")
    body.visual(mesh_from_cadquery(_body_shape(), "dehumidifier_body"), material=body_finish, name="body_shell")

    bucket = model.part("bucket")
    bucket.visual(mesh_from_cadquery(_bucket_shape(), "dehumidifier_bucket"), material=bucket_finish, name="bucket_shell")
    model.articulation(
        "body_to_bucket",
        ArticulationType.PRISMATIC,
        parent=body,
        child=bucket,
        origin=Origin(xyz=(0.0, -BODY_D / 2.0, BUCKET_FLOOR_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=0.18, lower=0.0, upper=BUCKET_TRAVEL),
    )

    handle = model.part("handle")
    handle.visual(mesh_from_cadquery(_handle_shape(), "dehumidifier_handle"), material=trim_dark, name="handle_shell")
    model.articulation(
        "body_to_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, HANDLE_PIVOT_Y, HANDLE_PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.4, lower=0.0, upper=1.25),
    )

    filter_door = model.part("filter_door")
    filter_door.visual(
        mesh_from_cadquery(_filter_door_shape(), "dehumidifier_filter_door"),
        material=vent_dark,
        name="filter_panel",
    )
    model.articulation(
        "body_to_filter_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=filter_door,
        origin=Origin(xyz=(BODY_W / 2.0, FILTER_HINGE_Y, FILTER_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.2, lower=0.0, upper=1.55),
    )

    control_panel = model.part("control_panel")
    control_panel.visual(
        mesh_from_cadquery(_control_panel_shape(), "dehumidifier_control_panel"),
        material=vent_dark,
        name="panel_shell",
    )
    model.articulation(
        "body_to_control_panel",
        ArticulationType.FIXED,
        parent=body,
        child=control_panel,
        origin=Origin(xyz=(BODY_W / 2.0, PANEL_CENTER_Y, PANEL_CENTER_Z)),
    )

    dial = model.part("dial")
    dial.visual(mesh_from_cadquery(_dial_shape(), "dehumidifier_dial"), material=control_dark, name="dial_knob")
    model.articulation(
        "panel_to_dial",
        ArticulationType.CONTINUOUS,
        parent=control_panel,
        child=dial,
        origin=Origin(xyz=(PANEL_T - 0.006, 0.012, 0.028)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=8.0),
    )

    button_positions = {"button_0": -0.010, "button_1": -0.044}
    for button_name, button_z in button_positions.items():
        button = model.part(button_name)
        button.visual(
            mesh_from_cadquery(_button_shape(), f"dehumidifier_{button_name}"),
            material=button_light,
            name="button_cap",
        )
        model.articulation(
            f"panel_to_{button_name}",
            ArticulationType.PRISMATIC,
            parent=control_panel,
            child=button,
            origin=Origin(xyz=(PANEL_T - 0.010 + BUTTON_TRAVEL, -0.012, button_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=0.08, lower=-BUTTON_TRAVEL, upper=0.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    bucket = object_model.get_part("bucket")
    handle = object_model.get_part("handle")
    filter_door = object_model.get_part("filter_door")
    dial = object_model.get_part("dial")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")

    bucket_slide = object_model.get_articulation("body_to_bucket")
    handle_hinge = object_model.get_articulation("body_to_handle")
    door_hinge = object_model.get_articulation("body_to_filter_door")
    dial_joint = object_model.get_articulation("panel_to_dial")
    button_0_joint = object_model.get_articulation("panel_to_button_0")
    button_1_joint = object_model.get_articulation("panel_to_button_1")

    ctx.allow_overlap(
        body,
        bucket,
        elem_a="body_shell",
        elem_b="bucket_shell",
        reason="The pullout bucket is represented inside a single connected shell assembly that remains visibly open around the tank bay.",
    )
    ctx.allow_overlap(
        body,
        handle,
        elem_a="body_shell",
        elem_b="handle_shell",
        reason="The carry handle nests into the recessed top pocket in the stowed pose.",
    )
    ctx.allow_overlap(
        "control_panel",
        "dial",
        elem_a="panel_shell",
        elem_b="dial_knob",
        reason="The dial base is intentionally seated into the control-panel recess.",
    )
    ctx.allow_overlap(
        "control_panel",
        "button_0",
        elem_a="panel_shell",
        elem_b="button_cap",
        reason="The button guide is intentionally nested in the push-button well.",
    )
    ctx.allow_overlap(
        "control_panel",
        "button_1",
        elem_a="panel_shell",
        elem_b="button_cap",
        reason="The button guide is intentionally nested in the push-button well.",
    )

    ctx.expect_within(
        bucket,
        body,
        axes="xz",
        margin=0.004,
        name="bucket stays nested within the lower body opening",
    )
    ctx.expect_overlap(
        bucket,
        body,
        axes="y",
        min_overlap=0.19,
        name="bucket is substantially inserted when closed",
    )

    if bucket_slide.motion_limits is not None and bucket_slide.motion_limits.upper is not None:
        bucket_rest = ctx.part_world_position(bucket)
        with ctx.pose({bucket_slide: bucket_slide.motion_limits.upper}):
            ctx.expect_within(
                bucket,
                body,
                axes="xz",
                margin=0.004,
                name="extended bucket stays aligned with the body bay",
            )
            ctx.expect_overlap(
                bucket,
                body,
                axes="y",
                min_overlap=0.085,
                name="extended bucket remains retained in the body",
            )
            bucket_open = ctx.part_world_position(bucket)
        ctx.check(
            "bucket pulls forward",
            bucket_rest is not None
            and bucket_open is not None
            and bucket_open[1] < bucket_rest[1] - 0.08,
            details=f"rest={bucket_rest}, open={bucket_open}",
        )

    ctx.expect_gap(
        filter_door,
        body,
        axis="x",
        max_gap=0.003,
        max_penetration=0.0,
        name="filter door sits flush against the side shell when closed",
    )
    if door_hinge.motion_limits is not None and door_hinge.motion_limits.upper is not None:
        door_rest = ctx.part_world_aabb(filter_door)
        with ctx.pose({door_hinge: door_hinge.motion_limits.upper}):
            door_open = ctx.part_world_aabb(filter_door)
        ctx.check(
            "filter door swings outward",
            door_rest is not None
            and door_open is not None
            and door_open[1][0] > door_rest[1][0] + 0.08,
            details=f"closed={door_rest}, open={door_open}",
        )

    if handle_hinge.motion_limits is not None and handle_hinge.motion_limits.upper is not None:
        handle_rest = ctx.part_world_aabb(handle)
        with ctx.pose({handle_hinge: handle_hinge.motion_limits.upper}):
            handle_up = ctx.part_world_aabb(handle)
        ctx.check(
            "carry handle lifts above the shell",
            handle_rest is not None
            and handle_up is not None
            and handle_up[1][2] > handle_rest[1][2] + 0.03,
            details=f"rest={handle_rest}, raised={handle_up}",
        )

    ctx.check(
        "mode dial uses unbounded rotation limits",
        dial_joint.motion_limits is not None
        and dial_joint.motion_limits.lower is None
        and dial_joint.motion_limits.upper is None,
        details=f"motion_limits={dial_joint.motion_limits!r}",
    )
    with ctx.pose({dial_joint: 1.8}):
        ctx.check(
            "dial remains poseable",
            ctx.part_world_aabb(dial) is not None,
            details="Expected the dial to remain present at a rotated pose.",
        )

    for part_obj, joint_obj, name in (
        (button_0, button_0_joint, "button_0"),
        (button_1, button_1_joint, "button_1"),
    ):
        rest = ctx.part_world_position(part_obj)
        lower = joint_obj.motion_limits.lower if joint_obj.motion_limits is not None else None
        if lower is None:
            ctx.fail(f"{name}_joint_has_lower_limit", f"Expected a press travel for {name}.")
            continue
        with ctx.pose({joint_obj: lower}):
            pressed = ctx.part_world_position(part_obj)
        ctx.check(
            f"{name} pushes inward",
            rest is not None and pressed is not None and pressed[0] < rest[0] - 0.002,
            details=f"rest={rest}, pressed={pressed}",
        )

    return ctx.report()


object_model = build_object_model()
