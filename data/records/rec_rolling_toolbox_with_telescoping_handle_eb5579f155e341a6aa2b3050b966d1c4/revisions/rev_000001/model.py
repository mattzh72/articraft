from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


CASE_DEPTH = 0.240
CASE_WIDTH = 0.360
CASE_BODY_HEIGHT = 0.280
CASE_BASE_Z = 0.020
CASE_TOP_Z = CASE_BASE_Z + CASE_BODY_HEIGHT


def _make_case_shell() -> cq.Workplane:
    shell = (
        cq.Workplane("XY")
        .box(CASE_DEPTH, CASE_WIDTH, CASE_BODY_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.028)
        .translate((0.0, 0.0, CASE_BASE_Z))
    )

    drawer_cut = (
        cq.Workplane("XY")
        .box(0.190, 0.286, 0.086, centered=(True, True, False))
        .translate((0.030, 0.0, 0.046))
    )
    tray_cut = (
        cq.Workplane("XY")
        .box(0.184, 0.286, 0.092, centered=(True, True, False))
        .translate((0.005, 0.0, 0.214))
    )
    handle_recess = (
        cq.Workplane("XY")
        .box(0.038, 0.248, 0.186, centered=(True, True, False))
        .translate((-0.103, 0.0, 0.102))
    )
    grip_pocket = (
        cq.Workplane("XY")
        .box(0.048, 0.255, 0.042, centered=(True, True, False))
        .translate((-0.098, 0.0, 0.248))
    )

    shell = shell.cut(drawer_cut.union(tray_cut).union(handle_recess).union(grip_pocket))

    tray_rail_left = (
        cq.Workplane("XY")
        .box(0.160, 0.010, 0.006, centered=(True, True, False))
        .translate((0.000, 0.138, 0.226))
    )
    tray_rail_right = (
        cq.Workplane("XY")
        .box(0.160, 0.010, 0.006, centered=(True, True, False))
        .translate((0.000, -0.138, 0.226))
    )

    drawer_rail_left = (
        cq.Workplane("XY")
        .box(0.154, 0.010, 0.008, centered=(True, True, False))
        .translate((0.005, 0.138, 0.064))
    )
    drawer_rail_right = (
        cq.Workplane("XY")
        .box(0.154, 0.010, 0.008, centered=(True, True, False))
        .translate((0.005, -0.138, 0.064))
    )

    guide_left = (
        cq.Workplane("XY")
        .box(0.030, 0.022, 0.178, centered=(True, True, False))
        .translate((-0.109, 0.074, 0.108))
    )
    guide_right = (
        cq.Workplane("XY")
        .box(0.030, 0.022, 0.178, centered=(True, True, False))
        .translate((-0.109, -0.074, 0.108))
    )

    wheel_pod_left = (
        cq.Workplane("XY")
        .box(0.052, 0.010, 0.076, centered=(True, True, False))
        .translate((-0.080, 0.179, 0.010))
    )
    wheel_pod_right = (
        cq.Workplane("XY")
        .box(0.052, 0.010, 0.076, centered=(True, True, False))
        .translate((-0.080, -0.179, 0.010))
    )

    front_foot_left = (
        cq.Workplane("XY")
        .box(0.024, 0.034, 0.020, centered=(True, True, False))
        .translate((0.078, 0.092, 0.000))
    )
    front_foot_right = (
        cq.Workplane("XY")
        .box(0.024, 0.034, 0.020, centered=(True, True, False))
        .translate((0.078, -0.092, 0.000))
    )

    for feature in (
        tray_rail_left,
        tray_rail_right,
        drawer_rail_left,
        drawer_rail_right,
        guide_left,
        guide_right,
        wheel_pod_left,
        wheel_pod_right,
        front_foot_left,
        front_foot_right,
    ):
        shell = shell.union(feature)

    return shell.combine()


def _make_cover_panel() -> cq.Workplane:
    panel = (
        cq.Workplane("XY")
        .box(0.184, 0.288, 0.006, centered=(False, True, False))
        .translate((0.003, 0.0, 0.001))
    )
    side_lip_left = (
        cq.Workplane("XY")
        .box(0.156, 0.008, 0.010, centered=(False, True, False))
        .translate((0.016, 0.140, -0.005))
    )
    side_lip_right = (
        cq.Workplane("XY")
        .box(0.156, 0.008, 0.010, centered=(False, True, False))
        .translate((0.016, -0.140, -0.005))
    )
    finger_ridge = (
        cq.Workplane("XY")
        .box(0.020, 0.120, 0.003, centered=(False, True, False))
        .translate((0.154, 0.0, 0.006))
    )
    return panel.union(side_lip_left).union(side_lip_right).union(finger_ridge)


def _make_tray() -> cq.Workplane:
    tray = cq.Workplane("XY").box(0.168, 0.258, 0.024, centered=(True, True, False))
    tray_void = (
        cq.Workplane("XY")
        .box(0.158, 0.246, 0.018, centered=(True, True, False))
        .translate((0.0, 0.0, 0.004))
    )
    tray = tray.cut(tray_void)

    left_flange = (
        cq.Workplane("XY")
        .box(0.144, 0.010, 0.004, centered=(True, True, False))
        .translate((0.0, 0.134, 0.0))
    )
    right_flange = (
        cq.Workplane("XY")
        .box(0.144, 0.010, 0.004, centered=(True, True, False))
        .translate((0.0, -0.134, 0.0))
    )
    divider_long = (
        cq.Workplane("XY")
        .box(0.008, 0.214, 0.016, centered=(True, True, False))
        .translate((-0.006, 0.0, 0.000))
    )
    divider_cross = (
        cq.Workplane("XY")
        .box(0.060, 0.004, 0.016, centered=(True, True, False))
        .translate((-0.034, 0.0, 0.000))
    )
    pull_tab = (
        cq.Workplane("XY")
        .box(0.016, 0.084, 0.010, centered=(True, True, False))
        .translate((0.092, 0.0, 0.010))
    )

    return (
        tray.union(left_flange)
        .union(right_flange)
        .union(divider_long)
        .union(divider_cross)
        .union(pull_tab)
        .combine()
    )


def _make_drawer() -> cq.Workplane:
    drawer = cq.Workplane("XY").box(0.176, 0.274, 0.070, centered=(True, True, False))
    drawer_void = (
        cq.Workplane("XY")
        .box(0.166, 0.260, 0.060, centered=(True, True, False))
        .translate((0.0, 0.0, 0.006))
    )
    finger_pull = (
        cq.Workplane("YZ")
        .center(0.0, 0.040)
        .circle(0.024)
        .extrude(0.018)
        .translate((0.079, 0.0, 0.0))
    )

    drawer = drawer.cut(drawer_void).cut(finger_pull)

    left_runner = (
        cq.Workplane("XY")
        .box(0.144, 0.008, 0.010, centered=(True, True, False))
        .translate((-0.010, 0.137, 0.024))
    )
    right_runner = (
        cq.Workplane("XY")
        .box(0.144, 0.008, 0.010, centered=(True, True, False))
        .translate((-0.010, -0.137, 0.024))
    )
    handle_rim = (
        cq.Workplane("XY")
        .box(0.010, 0.120, 0.012, centered=(True, True, False))
        .translate((0.088, 0.0, 0.020))
    )

    return drawer.union(left_runner).union(right_runner).union(handle_rim)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rolling_tool_case")

    shell_color = model.material("shell_color", rgba=(0.20, 0.21, 0.23, 1.0))
    accent_color = model.material("accent_color", rgba=(0.72, 0.74, 0.77, 1.0))
    tray_color = model.material("tray_color", rgba=(0.88, 0.89, 0.91, 1.0))
    drawer_color = model.material("drawer_color", rgba=(0.83, 0.84, 0.86, 1.0))
    handle_color = model.material("handle_color", rgba=(0.62, 0.64, 0.68, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    wheel_hub = model.material("wheel_hub", rgba=(0.58, 0.60, 0.64, 1.0))

    shell = model.part("shell")
    shell.visual(
        mesh_from_cadquery(_make_case_shell(), "tool_case_shell"),
        material=shell_color,
        name="shell_body",
    )

    cover = model.part("cover")
    cover.visual(
        mesh_from_cadquery(_make_cover_panel(), "tool_case_cover"),
        material=accent_color,
        name="cover_panel",
    )
    cover.visual(
        Cylinder(radius=0.0045, length=0.264),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=accent_color,
        name="hinge_barrel",
    )

    tray = model.part("tray")
    tray.visual(
        mesh_from_cadquery(_make_tray(), "tool_case_tray"),
        material=tray_color,
        name="tray_insert",
    )

    drawer = model.part("drawer")
    drawer.visual(
        mesh_from_cadquery(_make_drawer(), "tool_case_drawer"),
        material=drawer_color,
        name="drawer_box",
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.006, length=0.186),
        origin=Origin(xyz=(0.0, 0.074, -0.108)),
        material=handle_color,
        name="rod_0",
    )
    handle.visual(
        Cylinder(radius=0.006, length=0.186),
        origin=Origin(xyz=(0.0, -0.074, -0.108)),
        material=handle_color,
        name="rod_1",
    )
    handle.visual(
        Cylinder(radius=0.007, length=0.164),
        origin=Origin(xyz=(0.0, 0.0, -0.015), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=handle_color,
        name="grip",
    )

    for index, side in enumerate((1.0, -1.0)):
        wheel = model.part(f"wheel_{index}")
        wheel.visual(
            Cylinder(radius=0.038, length=0.024),
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=wheel_rubber,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.024, length=0.026),
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=wheel_hub,
            name="hub",
        )
        wheel.visual(
            Cylinder(radius=0.010, length=0.030),
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=accent_color,
            name="cap",
        )

        model.articulation(
            f"shell_to_wheel_{index}",
            ArticulationType.CONTINUOUS,
            parent=shell,
            child=wheel,
            origin=Origin(xyz=(-0.082, side * 0.198, 0.041)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=20.0),
        )

    model.articulation(
        "shell_to_cover",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=cover,
        origin=Origin(xyz=(-0.092, 0.0, 0.305)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.4, lower=0.0, upper=1.75),
    )
    model.articulation(
        "shell_to_tray",
        ArticulationType.PRISMATIC,
        parent=shell,
        child=tray,
        origin=Origin(xyz=(0.002, 0.0, 0.232)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.25, lower=0.0, upper=0.065),
    )
    model.articulation(
        "shell_to_drawer",
        ArticulationType.PRISMATIC,
        parent=shell,
        child=drawer,
        origin=Origin(xyz=(0.030, 0.0, 0.048)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=24.0, velocity=0.25, lower=0.0, upper=0.090),
    )
    model.articulation(
        "shell_to_handle",
        ArticulationType.PRISMATIC,
        parent=shell,
        child=handle,
        origin=Origin(xyz=(-0.095, 0.0, 0.286)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.30, lower=0.0, upper=0.160),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shell = object_model.get_part("shell")
    cover = object_model.get_part("cover")
    tray = object_model.get_part("tray")
    drawer = object_model.get_part("drawer")
    handle = object_model.get_part("handle")

    cover_joint = object_model.get_articulation("shell_to_cover")
    tray_joint = object_model.get_articulation("shell_to_tray")
    drawer_joint = object_model.get_articulation("shell_to_drawer")
    handle_joint = object_model.get_articulation("shell_to_handle")

    cover_limits = cover_joint.motion_limits
    tray_limits = tray_joint.motion_limits
    drawer_limits = drawer_joint.motion_limits
    handle_limits = handle_joint.motion_limits

    ctx.allow_overlap(
        shell,
        tray,
        reason="The organizer bay is represented by a continuous outer shell proxy while the tray remains a separate supported insert on internal rails.",
    )
    ctx.allow_overlap(
        shell,
        drawer,
        reason="The lower drawer rides inside a simplified shell proxy that preserves the exterior housing silhouette.",
    )
    ctx.allow_overlap(
        shell,
        handle,
        reason="The telescoping handle stores inside simplified rear guide channels and a molded pocket represented by the shell proxy.",
    )

    if (
        cover_limits is not None
        and cover_limits.lower is not None
        and cover_limits.upper is not None
        and tray_limits is not None
        and tray_limits.lower is not None
        and tray_limits.upper is not None
        and drawer_limits is not None
        and drawer_limits.lower is not None
        and drawer_limits.upper is not None
        and handle_limits is not None
        and handle_limits.lower is not None
        and handle_limits.upper is not None
    ):
        with ctx.pose(
            {
                cover_joint: cover_limits.lower,
                tray_joint: tray_limits.lower,
                drawer_joint: drawer_limits.lower,
                handle_joint: handle_limits.lower,
            }
        ):
            ctx.expect_overlap(
                cover,
                shell,
                axes="xy",
                elem_a="cover_panel",
                elem_b="shell_body",
                min_overlap=0.18,
                name="cover spans the organizer opening",
            )
            ctx.expect_contact(
                tray,
                shell,
                name="tray insert remains supported on the organizer rails",
            )
            ctx.expect_within(
                tray,
                shell,
                axes="y",
                margin=0.004,
                name="tray stays centered between the organizer walls",
            )
            ctx.expect_contact(
                drawer,
                shell,
                name="drawer stays seated on the lower guide runners",
            )
            ctx.expect_within(
                drawer,
                shell,
                axes="yz",
                margin=0.004,
                name="drawer remains aligned in the lower shell",
            )
            ctx.expect_overlap(
                drawer,
                shell,
                axes="x",
                min_overlap=0.10,
                name="drawer starts fully inserted in the case",
            )

            shell_aabb = ctx.part_world_aabb(shell)
            closed_panel_aabb = ctx.part_element_world_aabb(cover, elem="cover_panel")
            ctx.check(
                "cover sits close to the shell top when closed",
                shell_aabb is not None
                and closed_panel_aabb is not None
                and closed_panel_aabb[0][2] >= shell_aabb[1][2] - 0.001
                and closed_panel_aabb[1][2] <= shell_aabb[1][2] + 0.020,
                details=f"shell={shell_aabb}, panel={closed_panel_aabb}",
            )

            tray_rest_pos = ctx.part_world_position(tray)
            drawer_rest_pos = ctx.part_world_position(drawer)
            handle_rest_pos = ctx.part_world_position(handle)

        with ctx.pose({cover_joint: cover_limits.upper}):
            open_panel_aabb = ctx.part_element_world_aabb(cover, elem="cover_panel")
            ctx.check(
                "cover panel lifts well above the case when opened",
                closed_panel_aabb is not None
                and open_panel_aabb is not None
                and open_panel_aabb[1][2] > closed_panel_aabb[1][2] + 0.10,
                details=f"closed={closed_panel_aabb}, open={open_panel_aabb}",
            )

        with ctx.pose({cover_joint: cover_limits.upper, tray_joint: tray_limits.upper}):
            ctx.expect_contact(
                tray,
                shell,
                name="tray keeps riding on the organizer rails when extended",
            )
            ctx.expect_within(
                tray,
                shell,
                axes="y",
                margin=0.004,
                name="tray stays laterally guided when extended",
            )
            ctx.expect_overlap(
                tray,
                shell,
                axes="x",
                min_overlap=0.08,
                name="tray keeps enough retained insertion at full extension",
            )
            ctx.expect_gap(
                cover,
                tray,
                axis="z",
                min_gap=0.020,
                positive_elem="cover_panel",
                negative_elem="tray_insert",
                name="opened cover clears the extended organizer tray",
            )
            tray_open_pos = ctx.part_world_position(tray)
            ctx.check(
                "tray slides forward from beneath the cover",
                tray_rest_pos is not None
                and tray_open_pos is not None
                and tray_open_pos[0] > tray_rest_pos[0] + 0.050,
                details=f"rest={tray_rest_pos}, open={tray_open_pos}",
            )

        with ctx.pose({drawer_joint: drawer_limits.upper}):
            ctx.expect_contact(
                drawer,
                shell,
                name="drawer remains runner-supported when opened",
            )
            ctx.expect_within(
                drawer,
                shell,
                axes="yz",
                margin=0.004,
                name="drawer stays straight on the runners when extended",
            )
            ctx.expect_overlap(
                drawer,
                shell,
                axes="x",
                min_overlap=0.040,
                name="drawer keeps retained insertion at full extension",
            )
            drawer_open_pos = ctx.part_world_position(drawer)
            ctx.check(
                "drawer moves outward from the front face",
                drawer_rest_pos is not None
                and drawer_open_pos is not None
                and drawer_open_pos[0] > drawer_rest_pos[0] + 0.080,
                details=f"rest={drawer_rest_pos}, open={drawer_open_pos}",
            )

        with ctx.pose({handle_joint: handle_limits.upper}):
            handle_open_pos = ctx.part_world_position(handle)
            ctx.check(
                "rear pull handle telescopes upward",
                handle_rest_pos is not None
                and handle_open_pos is not None
                and handle_open_pos[2] > handle_rest_pos[2] + 0.120,
                details=f"rest={handle_rest_pos}, open={handle_open_pos}",
            )

    return ctx.report()


object_model = build_object_model()
