from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os

os.chdir("/")

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    def _add_dual_wheel_assembly(
        model: ArticulatedObject,
        *,
        name: str,
        side_sign: float,
        tire_material,
        wheel_material,
        hub_material,
        stud_material,
    ):
        part = model.part(name)
        spin_origin = Origin(rpy=(0.0, math.pi / 2.0, 0.0))

        part.visual(
            Cylinder(radius=0.160, length=0.024),
            origin=Origin(xyz=(side_sign * 0.012, 0.0, 0.0), rpy=spin_origin.rpy),
            material=hub_material,
            name="mount_plate",
        )
        part.visual(
            Cylinder(radius=0.136, length=0.250),
            origin=Origin(xyz=(side_sign * 0.125, 0.0, 0.0), rpy=spin_origin.rpy),
            material=hub_material,
            name="hub_spacer",
        )
        part.visual(
            Cylinder(radius=0.175, length=0.120),
            origin=Origin(xyz=(side_sign * 0.110, 0.0, 0.0), rpy=spin_origin.rpy),
            material=wheel_material,
            name="inner_rim",
        )
        part.visual(
            Cylinder(radius=0.295, length=0.178),
            origin=Origin(xyz=(side_sign * 0.110, 0.0, 0.0), rpy=spin_origin.rpy),
            material=tire_material,
            name="inner_tire",
        )
        part.visual(
            Cylinder(radius=0.165, length=0.012),
            origin=Origin(xyz=(side_sign * 0.040, 0.0, 0.0), rpy=spin_origin.rpy),
            material=wheel_material,
            name="inner_face",
        )
        part.visual(
            Cylinder(radius=0.175, length=0.120),
            origin=Origin(xyz=(side_sign * 0.318, 0.0, 0.0), rpy=spin_origin.rpy),
            material=wheel_material,
            name="outer_rim",
        )
        part.visual(
            Cylinder(radius=0.295, length=0.178),
            origin=Origin(xyz=(side_sign * 0.318, 0.0, 0.0), rpy=spin_origin.rpy),
            material=tire_material,
            name="outer_tire",
        )
        part.visual(
            Cylinder(radius=0.172, length=0.012),
            origin=Origin(xyz=(side_sign * 0.372, 0.0, 0.0), rpy=spin_origin.rpy),
            material=wheel_material,
            name="outer_face",
        )
        part.visual(
            Cylinder(radius=0.082, length=0.022),
            origin=Origin(xyz=(side_sign * 0.375, 0.0, 0.0), rpy=spin_origin.rpy),
            material=hub_material,
            name="lug_pad",
        )
        for index in range(6):
            angle = math.tau * index / 6.0
            part.visual(
                Cylinder(radius=0.012, length=0.030),
                origin=Origin(
                    xyz=(side_sign * 0.392, math.cos(angle) * 0.055, math.sin(angle) * 0.055),
                    rpy=spin_origin.rpy,
                ),
                material=stud_material,
                name=f"lug_{index}",
            )
        part.visual(
            Cylinder(radius=0.058, length=0.034),
            origin=Origin(xyz=(side_sign * 0.389, 0.0, 0.0), rpy=spin_origin.rpy),
            material=wheel_material,
            name="hub_cap",
        )
        part.inertial = Inertial.from_geometry(
            Box((0.430, 0.620, 0.620)),
            mass=118.0,
            origin=Origin(xyz=(side_sign * 0.205, 0.0, 0.0)),
        )
        return part

    model = ArticulatedObject(name="truck_rear_axle")

    cast_iron = model.material("cast_iron", rgba=(0.28, 0.27, 0.25, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.54, 0.55, 0.57, 1.0))
    axle_paint = model.material("axle_paint", rgba=(0.18, 0.18, 0.19, 1.0))
    wheel_steel = model.material("wheel_steel", rgba=(0.67, 0.68, 0.70, 1.0))
    hub_steel = model.material("hub_steel", rgba=(0.42, 0.43, 0.45, 1.0))
    tire_rubber = model.material("tire_rubber", rgba=(0.06, 0.06, 0.06, 1.0))

    axle_housing = model.part("axle_housing")
    axle_housing.visual(
        Sphere(radius=0.195),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=cast_iron,
        name="differential_casing",
    )
    axle_housing.visual(
        Box((0.360, 0.215, 0.240)),
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
        material=cast_iron,
        name="carrier_body",
    )
    axle_housing.visual(
        Cylinder(radius=0.115, length=0.145),
        origin=Origin(xyz=(0.0, 0.035, 0.145), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cast_iron,
        name="top_rib",
    )
    axle_housing.visual(
        Cylinder(radius=0.145, length=0.078),
        origin=Origin(xyz=(0.0, 0.182, 0.018), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machined_steel,
        name="rear_cover",
    )
    axle_housing.visual(
        Cylinder(radius=0.065, length=0.160),
        origin=Origin(xyz=(0.0, -0.135, -0.015), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=axle_paint,
        name="pinion_nose",
    )
    axle_housing.visual(
        Cylinder(radius=0.096, length=0.058),
        origin=Origin(xyz=(0.0, -0.238, -0.020), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hub_steel,
        name="pinion_yoke",
    )
    axle_housing.visual(
        Box((0.240, 0.170, 0.220)),
        origin=Origin(xyz=(0.0, 0.005, -0.150)),
        material=cast_iron,
        name="lower_sump",
    )
    axle_housing.visual(
        Cylinder(radius=0.085, length=0.105),
        origin=Origin(xyz=(0.0, 0.105, -0.052), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cast_iron,
        name="inspection_boss",
    )
    axle_housing.visual(
        Cylinder(radius=0.078, length=0.720),
        origin=Origin(xyz=(0.540, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=axle_paint,
        name="left_axle_tube",
    )
    axle_housing.visual(
        Cylinder(radius=0.078, length=0.720),
        origin=Origin(xyz=(-0.540, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=axle_paint,
        name="right_axle_tube",
    )
    axle_housing.visual(
        Cylinder(radius=0.103, length=0.110),
        origin=Origin(xyz=(0.235, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=axle_paint,
        name="left_tube_collar",
    )
    axle_housing.visual(
        Cylinder(radius=0.103, length=0.110),
        origin=Origin(xyz=(-0.235, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=axle_paint,
        name="right_tube_collar",
    )
    axle_housing.visual(
        Cylinder(radius=0.110, length=0.120),
        origin=Origin(xyz=(0.610, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=axle_paint,
        name="left_spring_seat",
    )
    axle_housing.visual(
        Cylinder(radius=0.110, length=0.120),
        origin=Origin(xyz=(-0.610, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=axle_paint,
        name="right_spring_seat",
    )
    axle_housing.visual(
        Cylinder(radius=0.120, length=0.180),
        origin=Origin(xyz=(0.960, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_steel,
        name="left_hub_drum",
    )
    axle_housing.visual(
        Cylinder(radius=0.120, length=0.180),
        origin=Origin(xyz=(-0.960, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_steel,
        name="right_hub_drum",
    )
    axle_housing.visual(
        Cylinder(radius=0.165, length=0.048),
        origin=Origin(xyz=(1.074, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="left_hub_flange",
    )
    axle_housing.visual(
        Cylinder(radius=0.165, length=0.048),
        origin=Origin(xyz=(-1.074, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="right_hub_flange",
    )
    axle_housing.inertial = Inertial.from_geometry(
        Box((2.250, 0.540, 0.560)),
        mass=520.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    left_dual_wheels = _add_dual_wheel_assembly(
        model,
        name="left_dual_wheels",
        side_sign=1.0,
        tire_material=tire_rubber,
        wheel_material=wheel_steel,
        hub_material=hub_steel,
        stud_material=machined_steel,
    )
    right_dual_wheels = _add_dual_wheel_assembly(
        model,
        name="right_dual_wheels",
        side_sign=-1.0,
        tire_material=tire_rubber,
        wheel_material=wheel_steel,
        hub_material=hub_steel,
        stud_material=machined_steel,
    )

    for articulation_name, child_part, x_pos in (
        ("left_wheel_spin", left_dual_wheels, 1.098),
        ("right_wheel_spin", right_dual_wheels, -1.098),
    ):
        model.articulation(
            articulation_name,
            ArticulationType.CONTINUOUS,
            parent=axle_housing,
            child=child_part,
            origin=Origin(xyz=(x_pos, 0.0, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=2400.0,
                velocity=24.0,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, seed=0)

    axle_housing = object_model.get_part("axle_housing")
    left_dual_wheels = object_model.get_part("left_dual_wheels")
    right_dual_wheels = object_model.get_part("right_dual_wheels")

    left_wheel_spin = object_model.get_articulation("left_wheel_spin")
    right_wheel_spin = object_model.get_articulation("right_wheel_spin")

    def _aabb_center(aabb):
        return tuple((aabb[0][index] + aabb[1][index]) * 0.5 for index in range(3))

    def _axis_gap(aabb_a, aabb_b, axis_index: int = 0) -> float:
        a_min = aabb_a[0][axis_index]
        a_max = aabb_a[1][axis_index]
        b_min = aabb_b[0][axis_index]
        b_max = aabb_b[1][axis_index]
        if a_max <= b_min:
            return b_min - a_max
        if b_max <= a_min:
            return a_min - b_max
        return -min(a_max - b_min, b_max - a_min)

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=24,
        ignore_adjacent=False,
        ignore_fixed=True,
    )

    axle_aabb = ctx.part_world_aabb(axle_housing)
    if axle_aabb is None:
        ctx.fail("axle_housing_has_geometry", "axle_housing did not produce a world-space AABB.")
    else:
        axle_length = axle_aabb[1][0] - axle_aabb[0][0]
        axle_height = axle_aabb[1][2] - axle_aabb[0][2]
        ctx.check(
            "axle_housing_has_truck_scale_width",
            2.15 <= axle_length <= 2.22,
            details=f"Expected housing width around 2.18 m, got {axle_length:.3f} m.",
        )
        ctx.check(
            "differential_casing_has_heavy_vertical_depth",
            axle_height >= 0.50,
            details=f"Expected deep cast center section, got housing height {axle_height:.3f} m.",
        )

    for joint_name, joint in (
        ("left_wheel_spin", left_wheel_spin),
        ("right_wheel_spin", right_wheel_spin),
    ):
        ctx.check(
            f"{joint_name}_is_continuous_spin",
            joint.joint_type == ArticulationType.CONTINUOUS,
            details=f"{joint_name} should be a continuous wheel spin joint, got {joint.joint_type}.",
        )
        ctx.check(
            f"{joint_name}_axis_aligned_with_axle",
            tuple(joint.axis) == (1.0, 0.0, 0.0),
            details=f"{joint_name} axis should be (1, 0, 0), got {joint.axis}.",
        )

    ctx.expect_contact(
        left_dual_wheels,
        axle_housing,
        elem_a="mount_plate",
        elem_b="left_hub_flange",
        name="left_dual_wheels_seated_on_hub_flange",
    )
    ctx.expect_contact(
        right_dual_wheels,
        axle_housing,
        elem_a="mount_plate",
        elem_b="right_hub_flange",
        name="right_dual_wheels_seated_on_hub_flange",
    )

    left_inner_tire = ctx.part_element_world_aabb(left_dual_wheels, elem="inner_tire")
    left_outer_tire = ctx.part_element_world_aabb(left_dual_wheels, elem="outer_tire")
    right_inner_tire = ctx.part_element_world_aabb(right_dual_wheels, elem="inner_tire")
    right_outer_tire = ctx.part_element_world_aabb(right_dual_wheels, elem="outer_tire")
    if left_inner_tire is None or left_outer_tire is None:
        ctx.fail("left_dual_tire_elements_present", "Left dual-wheel tire AABBs were unavailable.")
    else:
        left_tire_gap = _axis_gap(left_inner_tire, left_outer_tire)
        ctx.check(
            "left_dual_tires_have_air_gap",
            0.025 <= left_tire_gap <= 0.035,
            details=f"Expected 25-35 mm gap between left dual tires, got {left_tire_gap:.4f} m.",
        )
    if right_inner_tire is None or right_outer_tire is None:
        ctx.fail("right_dual_tire_elements_present", "Right dual-wheel tire AABBs were unavailable.")
    else:
        right_tire_gap = _axis_gap(right_inner_tire, right_outer_tire)
        ctx.check(
            "right_dual_tires_have_air_gap",
            0.025 <= right_tire_gap <= 0.035,
            details=f"Expected 25-35 mm gap between right dual tires, got {right_tire_gap:.4f} m.",
        )

    with ctx.pose(
        {
            left_wheel_spin: math.pi / 2.0,
            right_wheel_spin: -math.pi / 2.0,
        }
    ):
        ctx.fail_if_isolated_parts(name="wheels_spun_pose_no_floating")
        ctx.fail_if_parts_overlap_in_current_pose(name="wheels_spun_pose_no_overlap")
        ctx.expect_contact(
            left_dual_wheels,
            axle_housing,
            elem_a="mount_plate",
            elem_b="left_hub_flange",
            name="left_dual_wheels_keep_flange_contact_while_spinning",
        )
        ctx.expect_contact(
            right_dual_wheels,
            axle_housing,
            elem_a="mount_plate",
            elem_b="right_hub_flange",
            name="right_dual_wheels_keep_flange_contact_while_spinning",
        )

    left_lug_rest = ctx.part_element_world_aabb(left_dual_wheels, elem="lug_0")
    if left_lug_rest is None:
        ctx.fail("left_lug_present", "left_dual_wheels lug_0 AABB was unavailable at rest.")
    else:
        rest_center = _aabb_center(left_lug_rest)
        with ctx.pose({left_wheel_spin: math.pi / 2.0}):
            left_lug_spun = ctx.part_element_world_aabb(left_dual_wheels, elem="lug_0")
        if left_lug_spun is None:
            ctx.fail("left_lug_present_spun", "left_dual_wheels lug_0 AABB was unavailable when spun.")
        else:
            spun_center = _aabb_center(left_lug_spun)
            ctx.check(
                "left_lug_moves_when_wheel_spins",
                abs(spun_center[1] - rest_center[1]) >= 0.045 and abs(spun_center[2] - rest_center[2]) >= 0.045,
                details=(
                    "Expected a visible yz position change after 90 degree wheel rotation; "
                    f"rest={rest_center}, spun={spun_center}."
                ),
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
