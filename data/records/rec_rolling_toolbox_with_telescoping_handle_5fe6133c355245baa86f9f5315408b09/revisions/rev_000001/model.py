from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rolling_tool_chest")

    shell_mat = model.material("dark_ribbed_plastic", rgba=(0.08, 0.11, 0.13, 1.0))
    shell_edge_mat = model.material("black_molded_edges", rgba=(0.015, 0.017, 0.018, 1.0))
    drawer_mat = model.material("gray_drawer_front", rgba=(0.28, 0.31, 0.33, 1.0))
    recess_mat = model.material("shadowed_recess", rgba=(0.005, 0.006, 0.007, 1.0))
    metal_mat = model.material("brushed_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    tire_mat = model.material("rubber_tire", rgba=(0.01, 0.01, 0.01, 1.0))
    wheel_mat = model.material("molded_wheel_hub", rgba=(0.20, 0.22, 0.23, 1.0))

    lower_shell = model.part("lower_shell")

    # A deep open plastic tub: bottom, side walls, rear wall, and a framed front
    # opening for the shallow drawer.  The dimensions are garage-storage scale.
    lower_shell.visual(
        Box((0.64, 0.48, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=shell_mat,
        name="bottom_pan",
    )
    lower_shell.visual(
        Box((0.64, 0.035, 0.44)),
        origin=Origin(xyz=(0.0, 0.2225, 0.32)),
        material=shell_mat,
        name="side_wall_0",
    )
    lower_shell.visual(
        Box((0.64, 0.035, 0.44)),
        origin=Origin(xyz=(0.0, -0.2225, 0.32)),
        material=shell_mat,
        name="side_wall_1",
    )
    lower_shell.visual(
        Box((0.035, 0.48, 0.44)),
        origin=Origin(xyz=(-0.3025, 0.0, 0.32)),
        material=shell_mat,
        name="rear_wall",
    )
    lower_shell.visual(
        Box((0.035, 0.48, 0.13)),
        origin=Origin(xyz=(0.3025, 0.0, 0.165)),
        material=shell_mat,
        name="front_lower_rail",
    )
    lower_shell.visual(
        Box((0.035, 0.48, 0.17)),
        origin=Origin(xyz=(0.3025, 0.0, 0.455)),
        material=shell_mat,
        name="front_upper_rail",
    )
    for index, y in enumerate((-0.215, 0.215)):
        lower_shell.visual(
            Box((0.035, 0.030, 0.14)),
            origin=Origin(xyz=(0.3025, y, 0.30)),
            material=shell_mat,
            name=f"front_stile_{index}",
        )

    # Raised horizontal ribs make the shell read as blow-molded plastic.
    for side_index, y in enumerate((-0.246, 0.246)):
        for rib_index, z in enumerate((0.22, 0.34, 0.46)):
            lower_shell.visual(
                Box((0.56, 0.012, 0.018)),
                origin=Origin(xyz=(0.0, y, z)),
                material=shell_edge_mat,
                name=f"side_rib_{side_index}_{rib_index}",
            )
    for rib_index, z in enumerate((0.145, 0.505)):
        lower_shell.visual(
            Box((0.012, 0.40, 0.018)),
            origin=Origin(xyz=(0.326, 0.0, z)),
            material=shell_edge_mat,
            name=f"front_rib_{rib_index}",
        )

    # Straight internal guide runners for the front drawer.
    lower_shell.visual(
        Box((0.46, 0.018, 0.025)),
        origin=Origin(xyz=(0.02, -0.196, 0.2315)),
        material=shell_edge_mat,
        name="guide_runner_0",
    )
    lower_shell.visual(
        Box((0.46, 0.018, 0.025)),
        origin=Origin(xyz=(0.02, 0.196, 0.2315)),
        material=shell_edge_mat,
        name="guide_runner_1",
    )

    # Rear telescoping handle channels, modeled as real rectangular sleeves with
    # an open void rather than solid proxies.
    lower_shell.visual(
        Box((0.006, 0.056, 0.36)),
        origin=Origin(xyz=(-0.323, -0.17, 0.32)),
        material=shell_edge_mat,
        name="handle_sleeve_front_0",
    )
    lower_shell.visual(
        Box((0.006, 0.056, 0.36)),
        origin=Origin(xyz=(-0.385, -0.17, 0.32)),
        material=shell_edge_mat,
        name="handle_sleeve_rear_0",
    )
    lower_shell.visual(
        Box((0.062, 0.006, 0.36)),
        origin=Origin(xyz=(-0.354, -0.198, 0.32)),
        material=shell_edge_mat,
        name="handle_sleeve_side_0_0",
    )
    lower_shell.visual(
        Box((0.062, 0.006, 0.36)),
        origin=Origin(xyz=(-0.354, -0.142, 0.32)),
        material=shell_edge_mat,
        name="handle_sleeve_side_0_1",
    )
    lower_shell.visual(
        Box((0.006, 0.056, 0.36)),
        origin=Origin(xyz=(-0.323, 0.17, 0.32)),
        material=shell_edge_mat,
        name="handle_sleeve_front_1",
    )
    lower_shell.visual(
        Box((0.006, 0.056, 0.36)),
        origin=Origin(xyz=(-0.385, 0.17, 0.32)),
        material=shell_edge_mat,
        name="handle_sleeve_rear_1",
    )
    lower_shell.visual(
        Box((0.062, 0.006, 0.36)),
        origin=Origin(xyz=(-0.354, 0.142, 0.32)),
        material=shell_edge_mat,
        name="handle_sleeve_side_1_0",
    )
    lower_shell.visual(
        Box((0.062, 0.006, 0.36)),
        origin=Origin(xyz=(-0.354, 0.198, 0.32)),
        material=shell_edge_mat,
        name="handle_sleeve_side_1_1",
    )

    lower_shell.visual(
        Cylinder(radius=0.015, length=0.60),
        origin=Origin(xyz=(-0.23, 0.0, 0.09), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_mat,
        name="rear_axle",
    )

    top_lid = model.part("top_lid")
    top_lid.visual(
        Box((0.66, 0.52, 0.055)),
        origin=Origin(xyz=(0.33, 0.0, 0.0275)),
        material=shell_mat,
        name="lid_panel",
    )
    for rib_index, y in enumerate((-0.18, -0.09, 0.0, 0.09, 0.18)):
        top_lid.visual(
            Box((0.54, 0.016, 0.016)),
            origin=Origin(xyz=(0.34, y, 0.063)),
            material=shell_edge_mat,
            name=f"lid_rib_{rib_index}",
        )
    top_lid.visual(
        Cylinder(radius=0.018, length=0.50),
        origin=Origin(xyz=(0.0, 0.0, 0.018), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_mat,
        name="hinge_barrel",
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.032, 0.39, 0.15)),
        origin=Origin(xyz=(0.018, 0.0, 0.0)),
        material=drawer_mat,
        name="drawer_front",
    )
    drawer.visual(
        Box((0.006, 0.24, 0.035)),
        origin=Origin(xyz=(0.037, 0.0, 0.035)),
        material=recess_mat,
        name="finger_pull",
    )
    drawer.visual(
        Box((0.44, 0.34, 0.018)),
        origin=Origin(xyz=(-0.218, 0.0, -0.055)),
        material=drawer_mat,
        name="drawer_floor",
    )
    drawer.visual(
        Box((0.44, 0.018, 0.10)),
        origin=Origin(xyz=(-0.218, -0.175, -0.02)),
        material=drawer_mat,
        name="drawer_side_0",
    )
    drawer.visual(
        Box((0.50, 0.014, 0.022)),
        origin=Origin(xyz=(-0.245, -0.196, -0.045)),
        material=shell_edge_mat,
        name="drawer_runner_0",
    )
    drawer.visual(
        Box((0.44, 0.018, 0.10)),
        origin=Origin(xyz=(-0.218, 0.175, -0.02)),
        material=drawer_mat,
        name="drawer_side_1",
    )
    drawer.visual(
        Box((0.50, 0.014, 0.022)),
        origin=Origin(xyz=(-0.245, 0.196, -0.045)),
        material=shell_edge_mat,
        name="drawer_runner_1",
    )
    drawer.visual(
        Box((0.018, 0.34, 0.10)),
        origin=Origin(xyz=(-0.438, 0.0, -0.02)),
        material=drawer_mat,
        name="drawer_back",
    )

    pull_handle = model.part("pull_handle")
    pull_handle.visual(
        Cylinder(radius=0.008, length=0.62),
        origin=Origin(xyz=(0.0, -0.17, -0.13)),
        material=metal_mat,
        name="handle_rod_0",
    )
    pull_handle.visual(
        Box((0.056, 0.050, 0.030)),
        origin=Origin(xyz=(0.0, -0.17, -0.42)),
        material=shell_edge_mat,
        name="handle_guide_0",
    )
    pull_handle.visual(
        Cylinder(radius=0.008, length=0.62),
        origin=Origin(xyz=(0.0, 0.17, -0.13)),
        material=metal_mat,
        name="handle_rod_1",
    )
    pull_handle.visual(
        Box((0.056, 0.050, 0.030)),
        origin=Origin(xyz=(0.0, 0.17, -0.42)),
        material=shell_edge_mat,
        name="handle_guide_1",
    )
    pull_handle.visual(
        Cylinder(radius=0.018, length=0.40),
        origin=Origin(xyz=(0.0, 0.0, 0.18), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=shell_edge_mat,
        name="handle_grip",
    )

    wheel_mesh = WheelGeometry(
        0.060,
        0.052,
        rim=WheelRim(inner_radius=0.040, flange_height=0.006, flange_thickness=0.003),
        hub=WheelHub(
            radius=0.024,
            width=0.040,
            cap_style="domed",
            bolt_pattern=BoltPattern(count=5, circle_diameter=0.030, hole_diameter=0.004),
        ),
        face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
        spokes=WheelSpokes(style="straight", count=6, thickness=0.004, window_radius=0.010),
        bore=WheelBore(style="round", diameter=0.030),
    )
    tire_mesh = TireGeometry(
        0.090,
        0.060,
        inner_radius=0.062,
        tread=TireTread(style="block", depth=0.006, count=18, land_ratio=0.55),
        grooves=(TireGroove(center_offset=0.0, width=0.006, depth=0.002),),
        sidewall=TireSidewall(style="square", bulge=0.02),
        shoulder=TireShoulder(width=0.006, radius=0.003),
    )
    for index, y in enumerate((-0.275, 0.275)):
        wheel = model.part(f"wheel_{index}")
        wheel.visual(
            mesh_from_geometry(tire_mesh, f"tire_{index}"),
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=tire_mat,
            name="tire",
        )
        wheel.visual(
            mesh_from_geometry(wheel_mesh, f"wheel_hub_{index}"),
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=wheel_mat,
            name="wheel_hub",
        )
        model.articulation(
            f"lower_shell_to_wheel_{index}",
            ArticulationType.CONTINUOUS,
            parent=lower_shell,
            child=wheel,
            origin=Origin(xyz=(-0.23, y, 0.09)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=10.0, velocity=20.0),
        )

    model.articulation(
        "lower_shell_to_lid",
        ArticulationType.REVOLUTE,
        parent=lower_shell,
        child=top_lid,
        origin=Origin(xyz=(-0.32, 0.0, 0.54)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=0.0, upper=1.75),
    )
    model.articulation(
        "lower_shell_to_drawer",
        ArticulationType.PRISMATIC,
        parent=lower_shell,
        child=drawer,
        origin=Origin(xyz=(0.319, 0.0, 0.30)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.35, lower=0.0, upper=0.22),
    )
    model.articulation(
        "lower_shell_to_pull_handle",
        ArticulationType.PRISMATIC,
        parent=lower_shell,
        child=pull_handle,
        origin=Origin(xyz=(-0.354, 0.0, 0.62)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.35, lower=0.0, upper=0.28),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_shell = object_model.get_part("lower_shell")
    lid = object_model.get_part("top_lid")
    drawer = object_model.get_part("drawer")
    pull_handle = object_model.get_part("pull_handle")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")

    lid_joint = object_model.get_articulation("lower_shell_to_lid")
    drawer_joint = object_model.get_articulation("lower_shell_to_drawer")
    handle_joint = object_model.get_articulation("lower_shell_to_pull_handle")
    wheel_joint_0 = object_model.get_articulation("lower_shell_to_wheel_0")
    wheel_joint_1 = object_model.get_articulation("lower_shell_to_wheel_1")

    shell_aabb = ctx.part_world_aabb(lower_shell)
    ctx.check(
        "lower shell is a deep rolling chest body",
        shell_aabb is not None
        and (shell_aabb[1][0] - shell_aabb[0][0]) > 0.60
        and (shell_aabb[1][1] - shell_aabb[0][1]) > 0.45
        and (shell_aabb[1][2] - shell_aabb[0][2]) > 0.43,
        details=f"lower_shell_aabb={shell_aabb}",
    )

    ctx.expect_within(
        drawer,
        lower_shell,
        axes="yz",
        margin=0.015,
        name="drawer remains centered in the lower shell opening",
    )
    ctx.expect_overlap(
        drawer,
        lower_shell,
        axes="x",
        elem_a="drawer_runner_0",
        elem_b="guide_runner_0",
        min_overlap=0.20,
        name="closed drawer runner overlaps guide runner",
    )
    ctx.expect_gap(
        drawer,
        lower_shell,
        axis="x",
        positive_elem="drawer_front",
        negative_elem="front_upper_rail",
        min_gap=0.0,
        max_gap=0.010,
        name="drawer front sits just proud of front shell",
    )

    drawer_rest = ctx.part_world_position(drawer)
    with ctx.pose({drawer_joint: 0.22}):
        ctx.expect_within(
            drawer,
            lower_shell,
            axes="yz",
            margin=0.015,
            name="extended drawer stays aligned on straight runners",
        )
        ctx.expect_overlap(
            drawer,
            lower_shell,
            axes="x",
            elem_a="drawer_runner_0",
            elem_b="guide_runner_0",
            min_overlap=0.16,
            name="extended drawer remains retained by its runner",
        )
        drawer_extended = ctx.part_world_position(drawer)
    ctx.check(
        "drawer slides outward along the front axis",
        drawer_rest is not None
        and drawer_extended is not None
        and drawer_extended[0] > drawer_rest[0] + 0.20,
        details=f"rest={drawer_rest}, extended={drawer_extended}",
    )

    handle_rest = ctx.part_world_position(pull_handle)
    with ctx.pose({handle_joint: 0.28}):
        ctx.expect_contact(
            pull_handle,
            lower_shell,
            elem_a="handle_guide_0",
            elem_b="handle_sleeve_side_0_0",
            contact_tol=0.001,
            name="extended handle guide shoe stays captured in channel",
        )
        handle_extended = ctx.part_world_position(pull_handle)
    ctx.check(
        "pull handle slides upward in the rear channels",
        handle_rest is not None
        and handle_extended is not None
        and handle_extended[2] > handle_rest[2] + 0.25,
        details=f"rest={handle_rest}, extended={handle_extended}",
    )

    lid_closed = ctx.part_world_aabb(lid)
    with ctx.pose({lid_joint: 1.25}):
        lid_open = ctx.part_world_aabb(lid)
    ctx.check(
        "hinged lid rotates upward from the rear hinge line",
        lid_closed is not None and lid_open is not None and lid_open[1][2] > lid_closed[1][2] + 0.25,
        details=f"closed={lid_closed}, open={lid_open}",
    )

    ctx.check(
        "rear wheels use continuous spinning joints",
        wheel_joint_0.articulation_type == ArticulationType.CONTINUOUS
        and wheel_joint_1.articulation_type == ArticulationType.CONTINUOUS,
        details=f"wheel types={wheel_joint_0.articulation_type}, {wheel_joint_1.articulation_type}",
    )
    ctx.allow_overlap(
        lower_shell,
        wheel_0,
        elem_a="rear_axle",
        elem_b="wheel_hub",
        reason="The metal axle is intentionally captured through the wheel hub bore so the wheel can spin on it.",
    )
    ctx.allow_overlap(
        lower_shell,
        wheel_1,
        elem_a="rear_axle",
        elem_b="wheel_hub",
        reason="The metal axle is intentionally captured through the wheel hub bore so the wheel can spin on it.",
    )
    ctx.expect_overlap(
        lower_shell,
        wheel_0,
        axes="y",
        elem_a="rear_axle",
        elem_b="wheel_hub",
        min_overlap=0.035,
        name="wheel 0 hub is retained on the rear axle",
    )
    ctx.expect_overlap(
        lower_shell,
        wheel_1,
        axes="y",
        elem_a="rear_axle",
        elem_b="wheel_hub",
        min_overlap=0.035,
        name="wheel 1 hub is retained on the rear axle",
    )
    ctx.expect_origin_gap(
        wheel_1,
        wheel_0,
        axis="y",
        min_gap=0.45,
        max_gap=0.60,
        name="two rear wheels sit on opposite ends of the axle",
    )

    return ctx.report()


object_model = build_object_model()
