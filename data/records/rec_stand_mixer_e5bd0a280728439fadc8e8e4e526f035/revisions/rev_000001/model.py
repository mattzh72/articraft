from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def _xy_section(
    z: float,
    width_x: float,
    depth_y: float,
    corner_radius: float,
    *,
    x_center: float = 0.0,
    y_center: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (x + x_center, y + y_center, z)
        for x, y in rounded_rect_profile(width_x, depth_y, corner_radius)
    ]


def _yz_section(
    x: float,
    width_y: float,
    height_z: float,
    corner_radius: float,
    *,
    z_center: float,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z + z_center)
        for z, y in rounded_rect_profile(height_z, width_y, corner_radius)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bowl_lift_stand_mixer")

    enamel = model.material("enamel", rgba=(0.90, 0.88, 0.82, 1.0))
    steel = model.material("steel", rgba=(0.87, 0.88, 0.90, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.16, 0.16, 0.18, 1.0))

    body = model.part("body")

    base_geom = ExtrudeGeometry.from_z0(rounded_rect_profile(0.36, 0.24, 0.045), 0.044)
    body.visual(
        mesh_from_geometry(base_geom, "base_shell"),
        origin=Origin(xyz=(0.02, 0.0, 0.0)),
        material=enamel,
        name="base_shell",
    )

    column_geom = section_loft(
        [
            _xy_section(0.044, 0.12, 0.11, 0.028, x_center=-0.07),
            _xy_section(0.18, 0.10, 0.10, 0.026, x_center=-0.055),
            _xy_section(0.31, 0.082, 0.09, 0.022, x_center=-0.035),
        ]
    )
    body.visual(
        mesh_from_geometry(column_geom, "column_shell"),
        material=enamel,
        name="column_shell",
    )

    body.visual(
        Box((0.092, 0.104, 0.078)),
        origin=Origin(xyz=(0.005, 0.0, 0.349)),
        material=enamel,
        name="neck_shell",
    )
    body.visual(
        Box((0.168, 0.132, 0.112)),
        origin=Origin(xyz=(0.040, 0.0, 0.392)),
        material=enamel,
        name="head_shell",
    )
    body.visual(
        Cylinder(radius=0.046, length=0.114),
        origin=Origin(xyz=(0.133, 0.0, 0.362), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=enamel,
        name="nose_shell",
    )

    for index, side in enumerate((1.0, -1.0)):
        body.visual(
            Box((0.014, 0.018, 0.190)),
            origin=Origin(xyz=(0.020, side * 0.138, 0.139)),
            material=enamel,
            name=f"guide_post_{index}",
        )

    body.visual(
        Box((0.130, 0.278, 0.024)),
        origin=Origin(xyz=(-0.045, 0.0, 0.242)),
        material=enamel,
        name="bridge_beam",
    )
    body.visual(
        Box((0.020, 0.022, 0.050)),
        origin=Origin(xyz=(-0.045, 0.050, 0.170)),
        material=dark_trim,
        name="lever_mount",
    )
    body.visual(
        Cylinder(radius=0.022, length=0.018),
        origin=Origin(xyz=(0.132, 0.0, 0.320)),
        material=dark_trim,
        name="drive_collar",
    )
    body.visual(
        Box((0.054, 0.064, 0.018)),
        origin=Origin(xyz=(0.132, 0.0, 0.326)),
        material=enamel,
        name="drive_mount",
    )

    bowl_carriage = model.part("bowl_carriage")
    bowl_carriage.visual(
        Cylinder(radius=0.094, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=steel,
        name="support_plate",
    )
    bowl_carriage.visual(
        Box((0.032, 0.260, 0.024)),
        origin=Origin(xyz=(-0.095, 0.0, 0.030)),
        material=steel,
        name="rear_beam",
    )
    for index, side in enumerate((1.0, -1.0)):
        bowl_carriage.visual(
            Box((0.018, 0.016, 0.130)),
            origin=Origin(xyz=(-0.095, side * 0.138, 0.089)),
            material=steel,
            name=f"guide_shoe_{index}",
        )
        bowl_carriage.visual(
            Box((0.090, 0.018, 0.018)),
            origin=Origin(xyz=(-0.062, side * 0.070, 0.013)),
            material=steel,
            name=f"support_arm_{index}",
        )

    model.articulation(
        "body_to_bowl_carriage",
        ArticulationType.PRISMATIC,
        parent=body,
        child=bowl_carriage,
        origin=Origin(xyz=(0.132, 0.0, 0.044)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.08,
            lower=0.0,
            upper=0.070,
        ),
    )

    bowl = model.part("bowl")
    bowl_geom = LatheGeometry.from_shell_profiles(
        [
            (0.022, 0.000),
            (0.048, 0.018),
            (0.092, 0.070),
            (0.118, 0.122),
            (0.124, 0.142),
        ],
        [
            (0.000, 0.002),
            (0.040, 0.014),
            (0.086, 0.066),
            (0.112, 0.132),
        ],
        segments=56,
        lip_samples=8,
    )
    bowl.visual(
        mesh_from_geometry(bowl_geom, "bowl_shell"),
        material=steel,
        name="bowl_shell",
    )
    for index, side in enumerate((1.0, -1.0)):
        handle_geom = tube_from_spline_points(
            [
                (-0.016, side * 0.112, 0.088),
                (0.010, side * 0.145, 0.094),
                (0.048, side * 0.151, 0.095),
                (0.078, side * 0.136, 0.100),
                (0.060, side * 0.112, 0.106),
            ],
            radius=0.0045,
            samples_per_segment=16,
            radial_segments=14,
        )
        bowl.visual(
            mesh_from_geometry(handle_geom, f"bowl_handle_{index}"),
            material=steel,
            name=f"handle_{index}",
        )
        for mount_index, mount_x, mount_z in (
            (0, -0.016, 0.088),
            (1, 0.060, 0.106),
        ):
            bowl.visual(
                Cylinder(radius=0.006, length=0.034),
                origin=Origin(
                    xyz=(mount_x, side * 0.110, mount_z),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=steel,
                name=f"handle_mount_{index}_{mount_index}",
            )

    model.articulation(
        "carriage_to_bowl",
        ArticulationType.FIXED,
        parent=bowl_carriage,
        child=bowl,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    lift_lever = model.part("lift_lever")
    lift_lever.visual(
        Box((0.014, 0.012, 0.020)),
        origin=Origin(xyz=(0.0, 0.006, 0.0)),
        material=dark_trim,
        name="lever_pivot",
    )
    lift_lever.visual(
        Box((0.014, 0.012, 0.085)),
        origin=Origin(xyz=(0.0, 0.014, -0.042)),
        material=dark_trim,
        name="lever_arm",
    )
    lift_lever.visual(
        Cylinder(radius=0.009, length=0.028),
        origin=Origin(xyz=(0.0, 0.030, -0.082), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="lever_grip",
    )

    model.articulation(
        "body_to_lift_lever",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lift_lever,
        origin=Origin(xyz=(-0.045, 0.061, 0.170)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.2,
            lower=-0.20,
            upper=0.95,
        ),
    )

    whisk = model.part("whisk")
    whisk_geom = CylinderGeometry(radius=0.0055, height=0.040, radial_segments=18).translate(
        0.0, 0.0, -0.020
    )
    whisk_geom.merge(
        CylinderGeometry(radius=0.010, height=0.030, radial_segments=24).translate(
            0.0, 0.0, -0.055
        )
    )
    for loop_index in range(6):
        angle = loop_index * 2.0 * math.pi / 6.0
        c = math.cos(angle)
        s = math.sin(angle)
        whisk_geom.merge(
            tube_from_spline_points(
                [
                    (0.006 * c, 0.006 * s, -0.068),
                    (0.026 * c, 0.026 * s, -0.090),
                    (0.041 * c, 0.041 * s, -0.122),
                    (0.036 * c, 0.036 * s, -0.156),
                    (0.0, 0.0, -0.174),
                    (-0.036 * c, -0.036 * s, -0.156),
                    (-0.041 * c, -0.041 * s, -0.122),
                    (-0.026 * c, -0.026 * s, -0.090),
                    (-0.006 * c, -0.006 * s, -0.068),
                ],
                radius=0.0015,
                samples_per_segment=16,
                radial_segments=12,
            )
        )

    whisk.visual(
        mesh_from_geometry(whisk_geom, "whisk_shell"),
        material=steel,
        name="whisk_shell",
    )

    model.articulation(
        "body_to_whisk",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=whisk,
        origin=Origin(xyz=(0.132, 0.0, 0.311)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    bowl = object_model.get_part("bowl")
    whisk = object_model.get_part("whisk")
    lift_lever = object_model.get_part("lift_lever")

    bowl_lift = object_model.get_articulation("body_to_bowl_carriage")
    lever_joint = object_model.get_articulation("body_to_lift_lever")
    whisk_joint = object_model.get_articulation("body_to_whisk")

    ctx.expect_overlap(
        bowl,
        whisk,
        axes="xy",
        elem_a="bowl_shell",
        elem_b="whisk_shell",
        min_overlap=0.07,
        name="whisk stays centered over the bowl",
    )

    bowl_rest = ctx.part_world_position(bowl)
    with ctx.pose({bowl_lift: bowl_lift.motion_limits.upper}):
        bowl_raised = ctx.part_world_position(bowl)
        ctx.expect_overlap(
            bowl,
            whisk,
            axes="xy",
            elem_a="bowl_shell",
            elem_b="whisk_shell",
            min_overlap=0.07,
            name="raised bowl remains aligned under whisk",
        )
        ctx.expect_gap(
            whisk,
            bowl,
            axis="z",
            positive_elem="whisk_shell",
            negative_elem="bowl_shell",
            min_gap=-0.20,
            max_gap=-0.08,
            name="raised bowl lifts whisk well below the rim",
        )
        ctx.expect_gap(
            body,
            bowl,
            axis="z",
            positive_elem="drive_collar",
            negative_elem="bowl_shell",
            min_gap=0.03,
            max_gap=0.07,
            name="raised bowl stays below the fixed head",
        )

    ctx.check(
        "bowl carriage raises the bowl noticeably",
        bowl_rest is not None
        and bowl_raised is not None
        and bowl_raised[2] > bowl_rest[2] + 0.06,
        details=f"rest={bowl_rest}, raised={bowl_raised}",
    )

    with ctx.pose({lever_joint: lever_joint.motion_limits.lower}):
        lever_low = ctx.part_element_world_aabb(lift_lever, elem="lever_grip")
    with ctx.pose({lever_joint: lever_joint.motion_limits.upper}):
        lever_high = ctx.part_element_world_aabb(lift_lever, elem="lever_grip")

    low_grip_z = None
    high_grip_z = None
    if lever_low is not None:
        low_grip_z = 0.5 * (lever_low[0][2] + lever_low[1][2])
    if lever_high is not None:
        high_grip_z = 0.5 * (lever_high[0][2] + lever_high[1][2])

    ctx.check(
        "lift lever swings upward through its travel",
        low_grip_z is not None and high_grip_z is not None and high_grip_z > low_grip_z + 0.04,
        details=f"low_grip_z={low_grip_z}, high_grip_z={high_grip_z}",
    )

    ctx.check(
        "whisk uses continuous rotation",
        whisk_joint.articulation_type == ArticulationType.CONTINUOUS
        and whisk_joint.motion_limits is not None
        and whisk_joint.motion_limits.lower is None
        and whisk_joint.motion_limits.upper is None,
        details=(
            f"type={whisk_joint.articulation_type}, "
            f"limits={whisk_joint.motion_limits}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
