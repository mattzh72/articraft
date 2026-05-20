from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _tube_shape(outer_radius: float, inner_radius: float, length: float):
    outer = cq.Workplane("XY").circle(outer_radius).extrude(length)
    inner = cq.Workplane("XY").circle(inner_radius).extrude(length)
    return outer.cut(inner).translate((0.0, 0.0, -length * 0.5))


def _body_shell_shape(
    *,
    width: float,
    depth: float,
    height: float,
    wall: float,
    corner_radius: float,
    upper_open_w: float,
    upper_open_h: float,
    upper_open_z: float,
    lower_open_w: float,
    lower_open_h: float,
    lower_open_z: float,
):
    outer = cq.Workplane("XY").box(width, depth, height).edges("|Z").fillet(corner_radius)
    inner = (
        cq.Workplane("XY")
        .box(width - 2.0 * wall, depth - 2.0 * wall, height - wall)
        .translate((0.0, 0.0, wall * 0.5))
    )
    shell = outer.cut(inner)
    upper_cut = cq.Workplane("XY").box(upper_open_w, wall * 10.0, upper_open_h).translate(
        (0.0, depth * 0.5 - wall * 1.5, upper_open_z - height * 0.5)
    )
    lower_cut = cq.Workplane("XY").box(lower_open_w, wall * 10.0, lower_open_h).translate(
        (0.0, depth * 0.5 - wall * 1.5, lower_open_z - height * 0.5)
    )
    return shell.cut(upper_cut).cut(lower_cut).translate((0.0, 0.0, height * 0.5))


def _lid_shell_shape(
    *,
    width: float,
    depth: float,
    height: float,
    wall: float,
    corner_radius: float,
):
    outer = cq.Workplane("XY").box(width, depth, height).edges("|Z").fillet(corner_radius)
    inner = (
        cq.Workplane("XY")
        .box(width - 2.0 * wall, depth - 2.0 * wall, height - wall)
        .translate((0.0, 0.0, -wall * 0.5))
    )
    lid = outer.cut(inner)
    return lid.translate((0.0, depth * 0.5, height * 0.5))


def _drawer_shape(
    *,
    width: float,
    tray_depth: float,
    tray_height: float,
    wall: float,
    front_width: float,
    front_height: float,
    front_thickness: float,
    front_gap: float,
    finger_pull: bool,
):
    tray = cq.Workplane("XY").box(width, tray_depth, tray_height)
    tray_inner = (
        cq.Workplane("XY")
        .box(width - 2.0 * wall, tray_depth - 2.0 * wall, tray_height - wall)
        .translate((0.0, 0.0, wall * 0.5))
    )
    tray = tray.cut(tray_inner)
    front = cq.Workplane("XY").box(front_width, front_thickness, front_height).translate(
        (
            0.0,
            tray_depth * 0.5 + front_gap + front_thickness * 0.5,
            (front_height - tray_height) * 0.5,
        )
    )
    drawer = tray.union(front)
    if finger_pull:
        groove = (
            cq.Workplane("XY")
            .cylinder(front_width + 0.05, front_height * 0.16)
            .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
            .translate((0.0, tray_depth * 0.5 + front_gap + front_thickness * 0.8, front_height * 0.72))
        )
        relief = cq.Workplane("XY").box(front_width * 0.62, front_thickness * 1.4, front_height * 0.22).translate(
            (0.0, tray_depth * 0.5 + front_gap + front_thickness * 0.55, front_height * 0.64)
        )
        drawer = drawer.cut(groove).cut(relief)
    return drawer.translate((0.0, tray_depth * 0.5, tray_height * 0.5))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="contractor_rolling_toolbox")

    shell_yellow = model.material("shell_yellow", rgba=(0.89, 0.74, 0.15, 1.0))
    trim_black = model.material("trim_black", rgba=(0.11, 0.11, 0.12, 1.0))
    drawer_black = model.material("drawer_black", rgba=(0.14, 0.14, 0.15, 1.0))
    steel = model.material("steel", rgba=(0.70, 0.71, 0.73, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.28, 0.29, 0.31, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))

    body_width = 0.58
    body_depth = 0.38
    body_height = 0.62
    body_bottom = 0.10
    wall = 0.0045
    body_front = body_depth * 0.5
    body_top = body_bottom + body_height

    upper_open_z = 0.44
    lower_open_z = 0.29

    body = model.part("body")
    body.visual(
        Box((wall, body_depth, body_height)),
        origin=Origin(xyz=(-body_width * 0.5 + wall * 0.5, 0.0, body_bottom + body_height * 0.5)),
        material=shell_yellow,
        name="side_wall_0",
    )
    body.visual(
        Box((wall, body_depth, body_height)),
        origin=Origin(xyz=(body_width * 0.5 - wall * 0.5, 0.0, body_bottom + body_height * 0.5)),
        material=shell_yellow,
        name="side_wall_1",
    )
    body.visual(
        Box((body_width - 2.0 * wall, wall, body_height)),
        origin=Origin(xyz=(0.0, -body_depth * 0.5 + wall * 0.5, body_bottom + body_height * 0.5)),
        material=shell_yellow,
        name="back_wall",
    )
    body.visual(
        Box((body_width - 2.0 * wall, body_depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, body_bottom + wall * 0.5)),
        material=shell_yellow,
        name="bottom_floor",
    )
    body.visual(
        Box((0.072, wall, body_height)),
        origin=Origin(xyz=(-0.254, body_front - wall * 0.5, body_bottom + body_height * 0.5)),
        material=shell_yellow,
        name="front_stile_0",
    )
    body.visual(
        Box((0.072, wall, body_height)),
        origin=Origin(xyz=(0.254, body_front - wall * 0.5, body_bottom + body_height * 0.5)),
        material=shell_yellow,
        name="front_stile_1",
    )
    body.visual(
        Box((0.432, wall, 0.245)),
        origin=Origin(xyz=(0.0, body_front - wall * 0.5, 0.2225)),
        material=shell_yellow,
        name="front_lower_panel",
    )
    body.visual(
        Box((0.436, wall, 0.067)),
        origin=Origin(xyz=(0.0, body_front - wall * 0.5, 0.4685)),
        material=shell_yellow,
        name="front_mid_panel",
    )
    body.visual(
        Box((0.436, wall, 0.142)),
        origin=Origin(xyz=(0.0, body_front - wall * 0.5, 0.649)),
        material=shell_yellow,
        name="front_upper_panel",
    )
    for index, x in enumerate((-0.222, 0.222)):
        body.visual(
            Box((0.016, 0.240, 0.010)),
            origin=Origin(xyz=(x, 0.080, body_bottom + 0.387)),
            material=dark_steel,
            name=f"accessory_runner_{index}",
        )
        body.visual(
            Box((0.016, 0.250, 0.010)),
            origin=Origin(xyz=(x, 0.075, body_bottom + 0.229)),
            material=dark_steel,
            name=f"drawer_runner_{index}",
        )
    body.visual(
        Cylinder(radius=0.013, length=0.591),
        origin=Origin(
            xyz=(0.0, -0.115, 0.095),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=dark_steel,
        name="axle",
    )
    for x in (-0.205, 0.205):
        body.visual(
            mesh_from_cadquery(_tube_shape(0.0155, 0.0115, 0.300), f"guide_sleeve_{'p' if x > 0 else 'n'}"),
            origin=Origin(xyz=(x, -0.210, 0.530)),
            material=dark_steel,
            name=f"guide_sleeve_{0 if x < 0 else 1}",
        )
        body.visual(
            Box((0.040, 0.015, 0.120)),
            origin=Origin(xyz=(x, -0.188, 0.430)),
            material=dark_steel,
            name=f"guide_mount_{0 if x < 0 else 1}",
        )
    for index, x in enumerate((-0.175, 0.175)):
        body.visual(
            Box((0.055, 0.050, body_bottom)),
            origin=Origin(xyz=(x, 0.145, body_bottom * 0.5)),
            material=trim_black,
            name=f"front_foot_{index}",
        )
    body.visual(
        Box((0.26, 0.030, 0.030)),
        origin=Origin(xyz=(0.0, body_front - 0.010, body_top - 0.020)),
        material=trim_black,
        name="front_ridge",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(
            _lid_shell_shape(
                width=0.596,
                depth=0.392,
                height=0.058,
                wall=wall,
                corner_radius=0.018,
            ),
            "lid_shell",
        ),
        material=shell_yellow,
        name="lid_shell",
    )
    lid.visual(
        Box((0.120, 0.028, 0.020)),
        origin=Origin(xyz=(0.0, 0.381, 0.022)),
        material=trim_black,
        name="lid_grip",
    )

    handle = model.part("handle")
    for index, x in enumerate((-0.205, 0.205)):
        handle.visual(
            Cylinder(radius=0.0104, length=0.760),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=steel,
            name=f"handle_rail_{index}",
        )
        handle.visual(
            Cylinder(radius=0.017, length=0.012),
            origin=Origin(xyz=(x, 0.0, 0.006)),
            material=dark_steel,
            name=f"handle_stop_{index}",
        )
    handle.visual(
        Box((0.460, 0.024, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.380)),
        material=steel,
        name="handle_crossbar",
    )
    handle.visual(
        Cylinder(radius=0.015, length=0.300),
        origin=Origin(
            xyz=(0.0, 0.0, 0.390),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=trim_black,
        name="handle_grip",
    )

    accessory_drawer = model.part("accessory_drawer")
    accessory_drawer.visual(
        mesh_from_cadquery(
            _drawer_shape(
                width=0.440,
                tray_depth=0.268,
                tray_height=0.072,
                wall=wall,
                front_width=0.456,
                front_height=0.092,
                front_thickness=0.014,
                front_gap=0.0,
                finger_pull=False,
            ),
            "accessory_drawer_shell",
        ),
        material=drawer_black,
        name="accessory_box",
    )
    accessory_drawer.visual(
        Box((0.090, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, 0.276, 0.047)),
        material=trim_black,
        name="accessory_pull",
    )
    for index, x in enumerate((-0.215, 0.215)):
        accessory_drawer.visual(
            Box((0.012, 0.220, 0.008)),
            origin=Origin(xyz=(x, 0.120, 0.004)),
            material=dark_steel,
            name=f"accessory_slide_{index}",
        )

    front_drawer = model.part("front_drawer")
    front_drawer.visual(
        mesh_from_cadquery(
            _drawer_shape(
                width=0.442,
                tray_depth=0.282,
                tray_height=0.082,
                wall=wall,
                front_width=0.460,
                front_height=0.110,
                front_thickness=0.016,
                front_gap=0.0,
                finger_pull=True,
            ),
            "front_drawer_shell",
        ),
        material=drawer_black,
        name="drawer_box",
    )
    for index, x in enumerate((-0.216, 0.216)):
        front_drawer.visual(
            Box((0.012, 0.228, 0.008)),
            origin=Origin(xyz=(x, 0.122, 0.004)),
            material=dark_steel,
            name=f"drawer_slide_{index}",
        )

    for wheel_name in ("wheel_0", "wheel_1"):
        wheel = model.part(wheel_name)
        wheel.visual(
            Cylinder(radius=0.090, length=0.055),
            origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
            material=rubber,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.056, length=0.046),
            origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
            material=dark_steel,
            name="hub",
        )
        wheel.visual(
            Cylinder(radius=0.036, length=0.018),
            origin=Origin(xyz=(0.014, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
            material=steel,
            name="cap",
        )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, -body_depth * 0.5, body_top)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.4,
            lower=0.0,
            upper=1.15,
        ),
    )
    model.articulation(
        "handle_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, -0.210, 0.680)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.25,
            lower=0.0,
            upper=0.280,
        ),
    )
    model.articulation(
        "accessory_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=accessory_drawer,
        origin=Origin(xyz=(0.0, body_front - 0.268, body_bottom + 0.394)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.20,
            lower=0.0,
            upper=0.145,
        ),
    )
    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=front_drawer,
        origin=Origin(xyz=(0.0, body_front - 0.276, body_bottom + 0.235)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.20,
            lower=0.0,
            upper=0.155,
        ),
    )
    model.articulation(
        "body_to_wheel_0",
        ArticulationType.CONTINUOUS,
        parent=body,
        child="wheel_0",
        origin=Origin(xyz=(-0.323, -0.115, 0.095)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=18.0),
    )
    model.articulation(
        "body_to_wheel_1",
        ArticulationType.CONTINUOUS,
        parent=body,
        child="wheel_1",
        origin=Origin(xyz=(0.323, -0.115, 0.095)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    handle = object_model.get_part("handle")
    accessory_drawer = object_model.get_part("accessory_drawer")
    front_drawer = object_model.get_part("front_drawer")

    lid_hinge = object_model.get_articulation("lid_hinge")
    handle_slide = object_model.get_articulation("handle_slide")
    accessory_slide = object_model.get_articulation("accessory_slide")
    drawer_slide = object_model.get_articulation("drawer_slide")

    lid_limits = lid_hinge.motion_limits
    handle_limits = handle_slide.motion_limits
    accessory_limits = accessory_slide.motion_limits
    drawer_limits = drawer_slide.motion_limits

    if lid_limits is not None and lid_limits.upper is not None:
        with ctx.pose({lid_hinge: 0.0}):
            ctx.expect_overlap(
                lid,
                body,
                axes="xy",
                min_overlap=0.35,
                name="closed lid covers the top opening",
            )
            grip_closed = ctx.part_element_world_aabb(lid, elem="lid_grip")
        with ctx.pose({lid_hinge: lid_limits.upper}):
            grip_open = ctx.part_element_world_aabb(lid, elem="lid_grip")
        ctx.check(
            "lid opens upward from the rear hinge",
            grip_closed is not None
            and grip_open is not None
            and grip_open[1][2] > grip_closed[1][2] + 0.18,
            details=f"closed={grip_closed}, open={grip_open}",
        )

    if handle_limits is not None and handle_limits.upper is not None:
        ctx.expect_within(
            handle,
            body,
            axes="xy",
            elem_a="handle_rail_0",
            elem_b="guide_sleeve_0",
            margin=0.006,
            name="left handle rail stays centered in the guide sleeve at rest",
        )
        ctx.expect_within(
            handle,
            body,
            axes="xy",
            elem_a="handle_rail_1",
            elem_b="guide_sleeve_1",
            margin=0.006,
            name="right handle rail stays centered in the guide sleeve at rest",
        )
        ctx.expect_overlap(
            handle,
            body,
            axes="z",
            elem_a="handle_rail_0",
            elem_b="guide_sleeve_0",
            min_overlap=0.28,
            name="collapsed left handle rail remains inserted",
        )
        ctx.expect_overlap(
            handle,
            body,
            axes="z",
            elem_a="handle_rail_1",
            elem_b="guide_sleeve_1",
            min_overlap=0.28,
            name="collapsed right handle rail remains inserted",
        )
        handle_rest = ctx.part_world_position(handle)
        with ctx.pose({handle_slide: handle_limits.upper}):
            ctx.expect_within(
                handle,
                body,
                axes="xy",
                elem_a="handle_rail_0",
                elem_b="guide_sleeve_0",
                margin=0.006,
                name="left handle rail stays centered at full extension",
            )
            ctx.expect_within(
                handle,
                body,
                axes="xy",
                elem_a="handle_rail_1",
                elem_b="guide_sleeve_1",
                margin=0.006,
                name="right handle rail stays centered at full extension",
            )
            ctx.expect_overlap(
                handle,
                body,
                axes="z",
                elem_a="handle_rail_0",
                elem_b="guide_sleeve_0",
                min_overlap=0.09,
                name="left handle rail keeps retained insertion when extended",
            )
            ctx.expect_overlap(
                handle,
                body,
                axes="z",
                elem_a="handle_rail_1",
                elem_b="guide_sleeve_1",
                min_overlap=0.09,
                name="right handle rail keeps retained insertion when extended",
            )
            handle_extended = ctx.part_world_position(handle)
        ctx.check(
            "handle extends upward",
            handle_rest is not None
            and handle_extended is not None
            and handle_extended[2] > handle_rest[2] + 0.20,
            details=f"rest={handle_rest}, extended={handle_extended}",
        )

    if accessory_limits is not None and accessory_limits.upper is not None:
        ctx.expect_gap(
            accessory_drawer,
            body,
            axis="z",
            elem_a="accessory_slide_0",
            elem_b="accessory_runner_0",
            min_gap=0.0,
            max_gap=0.006,
            name="left accessory slide rides just above its runner at rest",
        )
        ctx.expect_gap(
            accessory_drawer,
            body,
            axis="z",
            elem_a="accessory_slide_1",
            elem_b="accessory_runner_1",
            min_gap=0.0,
            max_gap=0.006,
            name="right accessory slide rides just above its runner at rest",
        )
        ctx.expect_overlap(
            accessory_drawer,
            body,
            axes="y",
            elem_a="accessory_slide_0",
            elem_b="accessory_runner_0",
            min_overlap=0.15,
            name="left accessory slide stays engaged at rest",
        )
        ctx.expect_overlap(
            accessory_drawer,
            body,
            axes="y",
            elem_a="accessory_slide_1",
            elem_b="accessory_runner_1",
            min_overlap=0.15,
            name="right accessory slide stays engaged at rest",
        )
        accessory_rest = ctx.part_world_position(accessory_drawer)
        with ctx.pose({accessory_slide: accessory_limits.upper}):
            ctx.expect_overlap(
                accessory_drawer,
                body,
                axes="y",
                elem_a="accessory_slide_0",
                elem_b="accessory_runner_0",
                min_overlap=0.07,
                name="left accessory slide keeps retained engagement when extended",
            )
            ctx.expect_overlap(
                accessory_drawer,
                body,
                axes="y",
                elem_a="accessory_slide_1",
                elem_b="accessory_runner_1",
                min_overlap=0.07,
                name="right accessory slide keeps retained engagement when extended",
            )
            accessory_extended = ctx.part_world_position(accessory_drawer)
        ctx.check(
            "accessory drawer extends forward",
            accessory_rest is not None
            and accessory_extended is not None
            and accessory_extended[1] > accessory_rest[1] + 0.10,
            details=f"rest={accessory_rest}, extended={accessory_extended}",
        )

    if drawer_limits is not None and drawer_limits.upper is not None:
        ctx.expect_gap(
            front_drawer,
            body,
            axis="z",
            elem_a="drawer_slide_0",
            elem_b="drawer_runner_0",
            min_gap=0.0,
            max_gap=0.006,
            name="left lower drawer slide rides just above its runner at rest",
        )
        ctx.expect_gap(
            front_drawer,
            body,
            axis="z",
            elem_a="drawer_slide_1",
            elem_b="drawer_runner_1",
            min_gap=0.0,
            max_gap=0.006,
            name="right lower drawer slide rides just above its runner at rest",
        )
        ctx.expect_overlap(
            front_drawer,
            body,
            axes="y",
            elem_a="drawer_slide_0",
            elem_b="drawer_runner_0",
            min_overlap=0.16,
            name="left lower drawer slide stays engaged at rest",
        )
        ctx.expect_overlap(
            front_drawer,
            body,
            axes="y",
            elem_a="drawer_slide_1",
            elem_b="drawer_runner_1",
            min_overlap=0.16,
            name="right lower drawer slide stays engaged at rest",
        )
        ctx.expect_within(
            front_drawer,
            body,
            axes="xz",
            margin=0.02,
            name="lower drawer stays aligned within the shell envelope at rest",
        )
        drawer_rest = ctx.part_world_position(front_drawer)
        body_aabb = ctx.part_world_aabb(body)
        drawer_aabb = ctx.part_world_aabb(front_drawer)
        ctx.check(
            "lower drawer front stands proud of the outer shell",
            body_aabb is not None
            and drawer_aabb is not None
            and drawer_aabb[1][1] > body_aabb[1][1] + 0.01,
            details=f"body={body_aabb}, drawer={drawer_aabb}",
        )
        with ctx.pose({drawer_slide: drawer_limits.upper}):
            ctx.expect_overlap(
                front_drawer,
                body,
                axes="y",
                elem_a="drawer_slide_0",
                elem_b="drawer_runner_0",
                min_overlap=0.07,
                name="left lower drawer slide keeps retained engagement when extended",
            )
            ctx.expect_overlap(
                front_drawer,
                body,
                axes="y",
                elem_a="drawer_slide_1",
                elem_b="drawer_runner_1",
                min_overlap=0.07,
                name="right lower drawer slide keeps retained engagement when extended",
            )
            ctx.expect_within(
                front_drawer,
                body,
                axes="xz",
                margin=0.02,
                name="lower drawer stays aligned within the shell envelope when extended",
            )
            drawer_extended = ctx.part_world_position(front_drawer)
        ctx.check(
            "lower drawer extends forward",
            drawer_rest is not None
            and drawer_extended is not None
            and drawer_extended[1] > drawer_rest[1] + 0.12,
            details=f"rest={drawer_rest}, extended={drawer_extended}",
        )

    return ctx.report()


object_model = build_object_model()
