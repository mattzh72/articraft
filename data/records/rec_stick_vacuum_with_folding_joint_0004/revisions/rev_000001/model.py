from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_stick_vacuum", assets=ASSETS)

    safety_orange = model.material("safety_orange", rgba=(0.84, 0.36, 0.12, 1.0))
    dark_polymer = model.material("dark_polymer", rgba=(0.15, 0.16, 0.18, 1.0))
    molded_grey = model.material("molded_grey", rgba=(0.28, 0.30, 0.33, 1.0))
    wand_metal = model.material("wand_metal", rgba=(0.38, 0.41, 0.44, 1.0))
    steel = model.material("steel", rgba=(0.70, 0.72, 0.75, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.07, 1.0))
    smoke = model.material("smoke", rgba=(0.20, 0.22, 0.24, 0.55))

    def save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, ASSETS.mesh_path(name))

    def section(width: float, depth: float, z: float, radius: float, x_offset: float = 0.0):
        return [(x + x_offset, y, z) for x, y in rounded_rect_profile(width, depth, radius, corner_segments=8)]

    main_body = model.part("main_body")
    body_shell = section_loft(
        [
            section(0.074, 0.092, 0.030, 0.013, 0.000),
            section(0.082, 0.096, 0.120, 0.016, 0.012),
            section(0.072, 0.086, 0.235, 0.015, 0.018),
            section(0.048, 0.060, 0.320, 0.012, 0.008),
        ]
    )
    main_body.visual(save_mesh("vacuum_body_shell.obj", body_shell), material=safety_orange, name="body_shell")
    main_body.visual(
        Cylinder(radius=0.025, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=molded_grey,
        name="socket_collar",
    )
    main_body.visual(
        Cylinder(radius=0.032, length=0.165),
        origin=Origin(xyz=(0.044, 0.0, 0.138)),
        material=smoke,
        name="dust_cup",
    )
    main_body.visual(
        Cylinder(radius=0.028, length=0.038),
        origin=Origin(xyz=(0.044, 0.0, 0.238)),
        material=dark_polymer,
        name="cyclone_cap",
    )
    main_body.visual(
        Box((0.056, 0.072, 0.120)),
        origin=Origin(xyz=(-0.048, 0.0, 0.090)),
        material=dark_polymer,
        name="battery_pack",
    )
    main_body.visual(
        Box((0.034, 0.024, 0.160)),
        origin=Origin(xyz=(-0.028, 0.0, 0.245)),
        material=dark_polymer,
        name="handle_riser",
    )
    main_body.visual(
        Box((0.062, 0.024, 0.026)),
        origin=Origin(xyz=(-0.015, 0.0, 0.322)),
        material=dark_polymer,
        name="handle_bridge",
    )
    main_body.visual(
        Box((0.050, 0.016, 0.050)),
        origin=Origin(xyz=(0.014, 0.0, 0.274)),
        material=molded_grey,
        name="motor_cap",
    )
    for index, z in enumerate((0.064, 0.116)):
        for side in (-1.0, 1.0):
            main_body.visual(
                Cylinder(radius=0.0042, length=0.005),
                origin=Origin(xyz=(-0.048, side * 0.0385, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=steel,
                name=f"battery_bolt_{index}_{int(side > 0)}",
            )
    main_body.inertial = Inertial.from_geometry(
        Box((0.160, 0.110, 0.380)),
        mass=3.8,
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
    )

    upper_wand = model.part("upper_wand")
    upper_wand.visual(
        Cylinder(radius=0.022, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, -0.0225)),
        material=molded_grey,
        name="top_ferrule",
    )
    upper_wand.visual(
        Cylinder(radius=0.0185, length=0.340),
        origin=Origin(xyz=(0.0, 0.0, -0.170)),
        material=wand_metal,
        name="wand_tube",
    )
    upper_wand.visual(
        Cylinder(radius=0.0215, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, -0.070)),
        material=molded_grey,
        name="upper_reinforcement",
    )
    upper_wand.visual(
        Box((0.028, 0.040, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, -0.344)),
        material=molded_grey,
        name="fold_crown",
    )
    for side in (-1.0, 1.0):
        upper_wand.visual(
            Box((0.024, 0.006, 0.050)),
            origin=Origin(xyz=(0.0, side * 0.018, -0.377)),
            material=molded_grey,
            name=f"fold_cheek_{int(side > 0)}",
        )
        upper_wand.visual(
            Cylinder(radius=0.006, length=0.006),
            origin=Origin(xyz=(0.0, side * 0.020, -0.382), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"fold_pivot_cap_{int(side > 0)}",
        )
        upper_wand.visual(
            Cylinder(radius=0.0035, length=0.004),
            origin=Origin(xyz=(0.008, side * 0.016, -0.350), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"fold_service_bolt_{int(side > 0)}",
        )
    upper_wand.inertial = Inertial.from_geometry(
        Box((0.060, 0.060, 0.410)),
        mass=0.85,
        origin=Origin(xyz=(0.0, 0.0, -0.205)),
    )

    lower_wand = model.part("lower_wand")
    lower_wand.visual(
        Box((0.018, 0.030, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, -0.032)),
        material=molded_grey,
        name="fold_knuckle",
    )
    lower_wand.visual(
        Cylinder(radius=0.0160, length=0.330),
        origin=Origin(xyz=(0.0, 0.0, -0.217)),
        material=wand_metal,
        name="lower_tube",
    )
    lower_wand.visual(
        Cylinder(radius=0.0190, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, -0.110)),
        material=molded_grey,
        name="hinge_reinforcement",
    )
    lower_wand.visual(
        Box((0.028, 0.040, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, -0.394)),
        material=molded_grey,
        name="head_yoke_crown",
    )
    for side in (-1.0, 1.0):
        lower_wand.visual(
            Box((0.024, 0.006, 0.056)),
            origin=Origin(xyz=(0.0, side * 0.021, -0.435)),
            material=molded_grey,
            name=f"head_yoke_cheek_{int(side > 0)}",
        )
        lower_wand.visual(
            Cylinder(radius=0.006, length=0.006),
            origin=Origin(xyz=(0.0, side * 0.023, -0.455), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"head_yoke_cap_{int(side > 0)}",
        )
        lower_wand.visual(
            Cylinder(radius=0.0035, length=0.004),
            origin=Origin(xyz=(-0.008, side * 0.019, -0.404), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"head_yoke_bolt_{int(side > 0)}",
        )
    lower_wand.inertial = Inertial.from_geometry(
        Box((0.060, 0.060, 0.490)),
        mass=0.95,
        origin=Origin(xyz=(0.0, 0.0, -0.235)),
    )

    floor_head = model.part("floor_head")
    floor_head.visual(
        Box((0.018, 0.018, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=molded_grey,
        name="head_pivot_block",
    )
    floor_head.visual(
        Box((0.040, 0.040, 0.032)),
        origin=Origin(xyz=(0.018, 0.0, -0.022)),
        material=molded_grey,
        name="neck_block",
    )
    head_shell_geom = section_loft(
        [
            section(0.074, 0.220, -0.010, 0.010, 0.022),
            section(0.114, 0.290, -0.026, 0.016, 0.040),
            section(0.096, 0.262, -0.039, 0.012, 0.034),
        ]
    )
    floor_head.visual(
        save_mesh("vacuum_floor_head_shell.obj", head_shell_geom),
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
        material=dark_polymer,
        name="head_shell",
    )
    floor_head.visual(
        Box((0.102, 0.250, 0.010)),
        origin=Origin(xyz=(0.040, 0.0, -0.035)),
        material=dark_polymer,
        name="head_core",
    )
    floor_head.visual(
        Box((0.040, 0.026, 0.024)),
        origin=Origin(xyz=(0.054, -0.132, -0.034)),
        material=dark_polymer,
        name="left_side_pod",
    )
    floor_head.visual(
        Box((0.040, 0.026, 0.024)),
        origin=Origin(xyz=(0.054, 0.132, -0.034)),
        material=dark_polymer,
        name="right_side_pod",
    )
    floor_head.visual(
        Box((0.018, 0.290, 0.022)),
        origin=Origin(xyz=(0.094, 0.0, -0.033)),
        material=rubber,
        name="front_bumper",
    )
    floor_head.visual(
        Box((0.090, 0.020, 0.004)),
        origin=Origin(xyz=(0.020, -0.132, -0.068)),
        material=rubber,
        name="left_skid",
    )
    floor_head.visual(
        Box((0.090, 0.020, 0.004)),
        origin=Origin(xyz=(0.020, 0.132, -0.068)),
        material=rubber,
        name="right_skid",
    )
    floor_head.visual(
        Box((0.016, 0.010, 0.024)),
        origin=Origin(xyz=(0.034, -0.126, -0.058)),
        material=molded_grey,
        name="left_brush_bearing",
    )
    floor_head.visual(
        Box((0.016, 0.010, 0.024)),
        origin=Origin(xyz=(0.034, 0.126, -0.058)),
        material=molded_grey,
        name="right_brush_bearing",
    )
    floor_head.visual(
        Box((0.020, 0.008, 0.020)),
        origin=Origin(xyz=(0.017, -0.145, -0.076)),
        material=molded_grey,
        name="left_wheel_mount",
    )
    floor_head.visual(
        Box((0.020, 0.008, 0.020)),
        origin=Origin(xyz=(0.017, 0.145, -0.076)),
        material=molded_grey,
        name="right_wheel_mount",
    )
    floor_head.visual(
        Box((0.052, 0.170, 0.008)),
        origin=Origin(xyz=(-0.026, 0.0, -0.036)),
        material=rubber,
        name="squeegee_strip",
    )
    floor_head.inertial = Inertial.from_geometry(
        Box((0.140, 0.300, 0.090)),
        mass=1.35,
        origin=Origin(xyz=(0.040, 0.0, -0.050)),
    )

    brush_roll = model.part("brush_roll")
    brush_roll.visual(
        Cylinder(radius=0.018, length=0.238),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="brush_core",
    )
    for index, x in enumerate((-0.010, 0.0, 0.010)):
        brush_roll.visual(
            Cylinder(radius=0.018, length=0.014),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=safety_orange,
            name=f"brush_band_{index}",
        )
    brush_roll.inertial = Inertial.from_geometry(
        Cylinder(radius=0.018, length=0.238),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    left_wheel = model.part("left_wheel")
    left_wheel.visual(
        Cylinder(radius=0.019, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="left_wheel_tire",
    )
    left_wheel.visual(
        Cylinder(radius=0.009, length=0.004),
        origin=Origin(xyz=(0.0, 0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="left_wheel_cap",
    )
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.019, length=0.016),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    right_wheel = model.part("right_wheel")
    right_wheel.visual(
        Cylinder(radius=0.019, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="right_wheel_tire",
    )
    right_wheel.visual(
        Cylinder(radius=0.009, length=0.004),
        origin=Origin(xyz=(0.0, -0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="right_wheel_cap",
    )
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.019, length=0.016),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_upper_wand",
        ArticulationType.FIXED,
        parent=main_body,
        child=upper_wand,
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
    )
    model.articulation(
        "upper_to_lower_fold",
        ArticulationType.REVOLUTE,
        parent=upper_wand,
        child=lower_wand,
        origin=Origin(xyz=(0.0, 0.0, -0.360)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=math.radians(-72.0),
            upper=0.0,
        ),
    )
    model.articulation(
        "lower_to_floor_head",
        ArticulationType.REVOLUTE,
        parent=lower_wand,
        child=floor_head,
        origin=Origin(xyz=(0.0, 0.0, -0.455)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=math.radians(-42.0),
            upper=math.radians(20.0),
        ),
    )
    model.articulation(
        "head_to_brush",
        ArticulationType.CONTINUOUS,
        parent=floor_head,
        child=brush_roll,
        origin=Origin(xyz=(0.010, 0.0, -0.064)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=30.0),
    )
    model.articulation(
        "head_to_left_wheel",
        ArticulationType.CONTINUOUS,
        parent=floor_head,
        child=left_wheel,
        origin=Origin(xyz=(-0.012, -0.157, -0.076)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=20.0),
    )
    model.articulation(
        "head_to_right_wheel",
        ArticulationType.CONTINUOUS,
        parent=floor_head,
        child=right_wheel,
        origin=Origin(xyz=(-0.012, 0.157, -0.076)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root, seed=0)

    main_body = object_model.get_part("main_body")
    upper_wand = object_model.get_part("upper_wand")
    lower_wand = object_model.get_part("lower_wand")
    floor_head = object_model.get_part("floor_head")
    brush_roll = object_model.get_part("brush_roll")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")

    main_body.get_visual("body_shell")
    floor_head.get_visual("head_shell")
    floor_head.get_visual("front_bumper")
    brush_roll.get_visual("brush_core")

    fold_joint = object_model.get_articulation("upper_to_lower_fold")
    head_pitch = object_model.get_articulation("lower_to_floor_head")
    brush_spin = object_model.get_articulation("head_to_brush")
    left_wheel_spin = object_model.get_articulation("head_to_left_wheel")
    right_wheel_spin = object_model.get_articulation("head_to_right_wheel")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts(max_pose_samples=8, contact_tol=0.003)
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "fold_axis_is_lateral",
        tuple(fold_joint.axis) == (0.0, 1.0, 0.0),
        f"fold axis was {fold_joint.axis}",
    )
    ctx.check(
        "head_pitch_axis_is_lateral",
        tuple(head_pitch.axis) == (0.0, 1.0, 0.0),
        f"head pitch axis was {head_pitch.axis}",
    )
    ctx.check(
        "roller_and_wheels_spin_on_width_axis",
        tuple(brush_spin.axis) == (0.0, 1.0, 0.0)
        and tuple(left_wheel_spin.axis) == (0.0, 1.0, 0.0)
        and tuple(right_wheel_spin.axis) == (0.0, 1.0, 0.0),
        "brush roller or wheel spin axis was not aligned to the floor-head width axis",
    )

    ctx.expect_contact(main_body, upper_wand, contact_tol=0.001, name="body_to_wand_contact")
    ctx.expect_contact(upper_wand, lower_wand, contact_tol=0.0065, name="fold_joint_hardware_contact")
    ctx.expect_contact(lower_wand, floor_head, contact_tol=0.003, name="head_pivot_hardware_contact")
    ctx.expect_contact(floor_head, brush_roll, contact_tol=0.003, name="brush_bearing_contact")
    ctx.expect_contact(floor_head, left_wheel, contact_tol=0.003, name="left_wheel_axle_contact")
    ctx.expect_contact(floor_head, right_wheel, contact_tol=0.003, name="right_wheel_axle_contact")

    ctx.expect_origin_distance(left_wheel, right_wheel, axes="y", min_dist=0.29, max_dist=0.32)
    ctx.expect_origin_gap(main_body, floor_head, axis="z", min_gap=0.78, max_gap=0.86)

    body_aabb = ctx.part_world_aabb(main_body)
    left_aabb = ctx.part_world_aabb(left_wheel)
    right_aabb = ctx.part_world_aabb(right_wheel)
    if body_aabb is None or left_aabb is None or right_aabb is None:
        ctx.fail("world_aabb_available", "expected body and wheel AABBs for proportion checks")
    else:
        parked_height = body_aabb[1][2] - min(left_aabb[0][2], right_aabb[0][2])
        ctx.check(
            "overall_height_realistic",
            1.15 <= parked_height <= 1.30,
            f"overall parked height was {parked_height:.3f} m",
        )

    rest_head_pos = ctx.part_world_position(floor_head)
    if rest_head_pos is None:
        ctx.fail("rest_floor_head_position", "floor head world position unavailable in rest pose")
    else:
        with ctx.pose({fold_joint: math.radians(-68.0)}):
            folded_head_pos = ctx.part_world_position(floor_head)
            if folded_head_pos is None:
                ctx.fail("folded_head_position", "floor head world position unavailable in folded pose")
            else:
                ctx.check(
                    "fold_moves_head_forward",
                    folded_head_pos[0] > rest_head_pos[0] + 0.38,
                    f"folded head x shift was {folded_head_pos[0] - rest_head_pos[0]:.3f} m",
                )
                ctx.check(
                    "fold_raises_head",
                    folded_head_pos[2] > rest_head_pos[2] + 0.24,
                    f"folded head z shift was {folded_head_pos[2] - rest_head_pos[2]:.3f} m",
                )
            ctx.expect_contact(upper_wand, lower_wand, contact_tol=0.0065, name="fold_pose_joint_contact")
            ctx.expect_contact(lower_wand, floor_head, contact_tol=0.003, name="fold_pose_head_link_contact")
            ctx.fail_if_parts_overlap_in_current_pose(name="fold_pose_no_overlap")
            ctx.fail_if_isolated_parts(contact_tol=0.003, name="fold_pose_no_floating")

    rest_bumper = ctx.part_element_world_aabb(floor_head, elem="front_bumper")
    if rest_bumper is None:
        ctx.fail("rest_bumper_aabb", "front bumper AABB unavailable")
    else:
        with ctx.pose({head_pitch: math.radians(-36.0)}):
            pitched_bumper = ctx.part_element_world_aabb(floor_head, elem="front_bumper")
            if pitched_bumper is None:
                ctx.fail("pitched_bumper_aabb", "front bumper AABB unavailable in pitched pose")
            else:
                ctx.check(
                    "head_pitch_raises_nose",
                    pitched_bumper[1][2] > rest_bumper[1][2] + 0.05,
                    f"front bumper max z changed by {pitched_bumper[1][2] - rest_bumper[1][2]:.3f} m",
                )
            ctx.expect_contact(lower_wand, floor_head, contact_tol=0.003, name="pitched_head_joint_contact")
            ctx.fail_if_parts_overlap_in_current_pose(name="head_pitch_pose_no_overlap")
            ctx.fail_if_isolated_parts(contact_tol=0.003, name="head_pitch_pose_no_floating")

    with ctx.pose({brush_spin: 1.3, left_wheel_spin: 2.1, right_wheel_spin: -1.4}):
        ctx.expect_contact(floor_head, brush_roll, contact_tol=0.003, name="brush_contact_when_spinning")
        ctx.expect_contact(floor_head, left_wheel, contact_tol=0.003, name="left_wheel_contact_when_spinning")
        ctx.expect_contact(floor_head, right_wheel, contact_tol=0.003, name="right_wheel_contact_when_spinning")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
