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
    TorusGeometry,
    mesh_from_geometry,
)


def _torus_mesh(radius: float, tube: float, name: str):
    return mesh_from_geometry(
        TorusGeometry(radius=radius, tube=tube, radial_segments=20, tubular_segments=56),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_ring_light_tripod")

    matte_black = model.material("matte_black", rgba=(0.12, 0.12, 0.13, 1.0))
    satin_black = model.material("satin_black", rgba=(0.18, 0.18, 0.20, 1.0))
    aluminum = model.material("aluminum", rgba=(0.72, 0.74, 0.76, 1.0))
    diffuser_white = model.material("diffuser_white", rgba=(0.95, 0.96, 0.98, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))

    ring_housing_mesh = _torus_mesh(0.200, 0.032, "ring_head_outer_torus_v2")
    diffuser_mesh = _torus_mesh(0.200, 0.018, "ring_head_diffuser_torus_v2")

    stand_base = model.part("stand_base")
    stand_base.visual(
        Cylinder(radius=0.056, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 0.380)),
        material=matte_black,
        name="hub_core",
    )
    stand_base.visual(
        Cylinder(radius=0.038, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.430)),
        material=satin_black,
        name="hub_collar",
    )
    stand_base.visual(
        Box((0.022, 0.012, 0.780)),
        origin=Origin(xyz=(0.0, -0.036, 0.850)),
        material=aluminum,
        name="rear_guide_spine",
    )
    stand_base.visual(
        Box((0.066, 0.026, 0.070)),
        origin=Origin(xyz=(0.0, -0.046, 1.225)),
        material=matte_black,
        name="top_lock_body",
    )

    lug_radius = 0.073
    for leg_index, yaw in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0), start=1):
        stand_base.visual(
            Box((0.016, 0.026, 0.024)),
            origin=Origin(
                xyz=((lug_radius - 0.009) * math.cos(yaw), (lug_radius - 0.009) * math.sin(yaw), 0.383),
                rpy=(0.0, 0.0, yaw),
            ),
            material=matte_black,
            name=f"leg_{leg_index}_lug",
        )

    stand_base.inertial = Inertial.from_geometry(
        Box((1.12, 0.97, 1.36)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, 0.68)),
    )

    center_mast = model.part("center_mast")
    center_mast.visual(
        Cylinder(radius=0.018, length=0.900),
        origin=Origin(xyz=(0.0, 0.0, 0.450)),
        material=aluminum,
        name="upper_mast",
    )
    center_mast.visual(
        Box((0.022, 0.014, 0.880)),
        origin=Origin(xyz=(0.0, -0.023, 0.442)),
        material=aluminum,
        name="rear_slider",
    )
    center_mast.visual(
        Box((0.074, 0.030, 0.100)),
        origin=Origin(xyz=(0.0, -0.015, 0.825)),
        material=matte_black,
        name="tilt_block",
    )
    center_mast.visual(
        Box((0.230, 0.010, 0.018)),
        origin=Origin(xyz=(-0.121, -0.022, 0.825)),
        material=matte_black,
        name="left_support_bar",
    )
    center_mast.visual(
        Box((0.230, 0.010, 0.018)),
        origin=Origin(xyz=(0.121, -0.022, 0.825)),
        material=matte_black,
        name="right_support_bar",
    )
    center_mast.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(xyz=(-0.231, -0.008, 0.825), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="left_clamp",
    )
    center_mast.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(xyz=(0.231, -0.008, 0.825), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="right_clamp",
    )
    center_mast.inertial = Inertial.from_geometry(
        Box((0.52, 0.09, 0.94)),
        mass=1.3,
        origin=Origin(xyz=(0.0, -0.020, 0.470)),
    )

    light_head = model.part("light_head")
    light_head.visual(
        ring_housing_mesh,
        origin=Origin(xyz=(0.0, 0.124, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="ring_housing",
    )
    light_head.visual(
        diffuser_mesh,
        origin=Origin(xyz=(0.0, 0.124, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=diffuser_white,
        name="ring_diffuser",
    )
    light_head.visual(
        Box((0.024, 0.020, 0.024)),
        origin=Origin(xyz=(-0.196, 0.118, 0.0)),
        material=diffuser_white,
        name="left_diffuser_tab",
    )
    light_head.visual(
        Box((0.024, 0.020, 0.024)),
        origin=Origin(xyz=(0.196, 0.118, 0.0)),
        material=diffuser_white,
        name="right_diffuser_tab",
    )
    light_head.visual(
        Box((0.040, 0.112, 0.024)),
        origin=Origin(xyz=(-0.196, 0.056, 0.0)),
        material=satin_black,
        name="left_mount_strut",
    )
    light_head.visual(
        Box((0.040, 0.112, 0.024)),
        origin=Origin(xyz=(0.196, 0.056, 0.0)),
        material=satin_black,
        name="right_mount_strut",
    )
    light_head.visual(
        Box((0.352, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, 0.116, 0.0)),
        material=satin_black,
        name="top_bridge",
    )
    light_head.visual(
        Cylinder(radius=0.016, length=0.010),
        origin=Origin(xyz=(-0.221, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="left_pivot_boss",
    )
    light_head.visual(
        Cylinder(radius=0.016, length=0.010),
        origin=Origin(xyz=(0.221, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="right_pivot_boss",
    )
    light_head.inertial = Inertial.from_geometry(
        Cylinder(radius=0.235, length=0.120),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.124, 0.0)),
    )

    leg_specs = (
        ("leg_1", 1, 0.0),
        ("leg_2", 2, 2.0 * math.pi / 3.0),
        ("leg_3", 3, 4.0 * math.pi / 3.0),
    )
    open_leg_pitch = -0.69
    leg_beam_length = 0.500
    for leg_name, leg_index, yaw in leg_specs:
        leg = model.part(leg_name)
        leg.visual(
            Box((0.032, 0.024, 0.024)),
            origin=Origin(xyz=(0.016, 0.0, 0.0)),
            material=matte_black,
            name="hinge_block",
        )
        leg.visual(
            Box((leg_beam_length, 0.024, 0.020)),
            origin=Origin(xyz=(0.282, 0.0, 0.0)),
            material=aluminum,
            name="leg_tube",
        )
        leg.visual(
            Box((0.090, 0.046, 0.018)),
            origin=Origin(xyz=(0.577, 0.0, 0.0)),
            material=rubber,
            name="foot_pad",
        )
        leg.visual(
            Box((0.090, 0.026, 0.020)),
            origin=Origin(xyz=(0.067, 0.0, 0.0)),
            material=matte_black,
            name="upper_leg_socket",
        )
        leg.visual(
            Box((0.100, 0.026, 0.020)),
            origin=Origin(xyz=(0.500, 0.0, 0.0)),
            material=matte_black,
            name="lower_leg_socket",
        )
        leg.inertial = Inertial.from_geometry(
            Box((0.622, 0.046, 0.024)),
            mass=0.48,
            origin=Origin(xyz=(0.311, 0.0, 0.0)),
        )
        model.articulation(
            f"stand_base_to_{leg_name}",
            ArticulationType.REVOLUTE,
            parent=stand_base,
            child=leg,
            origin=Origin(
                xyz=(0.068 * math.cos(yaw), 0.068 * math.sin(yaw), 0.385),
                rpy=(0.0, open_leg_pitch, yaw),
            ),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=12.0,
                velocity=1.4,
                lower=0.0,
                upper=0.80,
            ),
        )

    model.articulation(
        "stand_base_to_center_mast",
        ArticulationType.PRISMATIC,
        parent=stand_base,
        child=center_mast,
        origin=Origin(xyz=(0.0, 0.0, 0.500)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.25,
            lower=0.0,
            upper=0.320,
        ),
    )

    model.articulation(
        "center_mast_to_light_head",
        ArticulationType.REVOLUTE,
        parent=center_mast,
        child=light_head,
        origin=Origin(xyz=(0.0, 0.0, 0.825)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.6,
            lower=-0.25,
            upper=0.80,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand_base = object_model.get_part("stand_base")
    center_mast = object_model.get_part("center_mast")
    light_head = object_model.get_part("light_head")
    leg_1 = object_model.get_part("leg_1")
    leg_2 = object_model.get_part("leg_2")
    leg_3 = object_model.get_part("leg_3")

    rear_guide_spine = stand_base.get_visual("rear_guide_spine")
    leg_1_lug = stand_base.get_visual("leg_1_lug")
    leg_2_lug = stand_base.get_visual("leg_2_lug")
    leg_3_lug = stand_base.get_visual("leg_3_lug")

    rear_slider = center_mast.get_visual("rear_slider")
    left_clamp = center_mast.get_visual("left_clamp")
    right_clamp = center_mast.get_visual("right_clamp")

    ring_housing = light_head.get_visual("ring_housing")
    left_boss = light_head.get_visual("left_pivot_boss")
    right_boss = light_head.get_visual("right_pivot_boss")

    leg_1_hinge = leg_1.get_visual("hinge_block")
    leg_2_hinge = leg_2.get_visual("hinge_block")
    leg_3_hinge = leg_3.get_visual("hinge_block")
    leg_1_foot = leg_1.get_visual("foot_pad")

    mast_extension = object_model.get_articulation("stand_base_to_center_mast")
    head_tilt = object_model.get_articulation("center_mast_to_light_head")
    leg_joints = [
        object_model.get_articulation("stand_base_to_leg_1"),
        object_model.get_articulation("stand_base_to_leg_2"),
        object_model.get_articulation("stand_base_to_leg_3"),
    ]

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()

    for leg, hinge, lug in (
        (leg_1, leg_1_hinge, leg_1_lug),
        (leg_2, leg_2_hinge, leg_2_lug),
        (leg_3, leg_3_hinge, leg_3_lug),
    ):
        ctx.allow_overlap(
            leg,
            stand_base,
            elem_a=hinge,
            elem_b=lug,
            reason="Tripod leg hinge knuckle nests at the hub pivot.",
        )

    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "mast_joint_axis_vertical",
        tuple(mast_extension.axis) == (0.0, 0.0, 1.0),
        details=f"Expected vertical mast axis, got {mast_extension.axis}",
    )
    ctx.check(
        "tilt_joint_axis_horizontal",
        tuple(head_tilt.axis) == (1.0, 0.0, 0.0),
        details=f"Expected horizontal x-axis tilt, got {head_tilt.axis}",
    )
    for index, leg_joint in enumerate(leg_joints, start=1):
        ctx.check(
            f"leg_{index}_joint_axis_horizontal",
            tuple(leg_joint.axis) == (0.0, -1.0, 0.0),
            details=f"Expected leg hinge axis (0, -1, 0), got {leg_joint.axis}",
        )

    ctx.expect_gap(
        center_mast,
        stand_base,
        axis="y",
        positive_elem=rear_slider,
        negative_elem=rear_guide_spine,
        max_gap=0.0,
        max_penetration=0.0,
    )
    ctx.expect_overlap(
        center_mast,
        stand_base,
        axes="xz",
        elem_a=rear_slider,
        elem_b=rear_guide_spine,
        min_overlap=0.020,
    )
    ctx.expect_contact(light_head, center_mast, elem_a=left_boss, elem_b=left_clamp)
    ctx.expect_contact(light_head, center_mast, elem_a=right_boss, elem_b=right_clamp)
    ctx.expect_contact(leg_1, stand_base, elem_a=leg_1_hinge, elem_b=leg_1_lug)
    ctx.expect_contact(leg_2, stand_base, elem_a=leg_2_hinge, elem_b=leg_2_lug)
    ctx.expect_contact(leg_3, stand_base, elem_a=leg_3_hinge, elem_b=leg_3_lug)
    ctx.expect_origin_gap(light_head, stand_base, axis="z", min_gap=1.20)

    mast_limits = mast_extension.motion_limits
    assert mast_limits is not None and mast_limits.lower is not None and mast_limits.upper is not None
    mast_rest_pos = ctx.part_world_position(center_mast)
    assert mast_rest_pos is not None
    with ctx.pose({mast_extension: mast_limits.upper}):
        mast_extended_pos = ctx.part_world_position(center_mast)
        assert mast_extended_pos is not None
        ctx.check(
            "mast_extension_travel",
            mast_extended_pos[2] > mast_rest_pos[2] + 0.25,
            details=f"Expected center mast to rise, got rest={mast_rest_pos}, extended={mast_extended_pos}",
        )
        ctx.expect_gap(
            center_mast,
            stand_base,
            axis="y",
            positive_elem=rear_slider,
            negative_elem=rear_guide_spine,
            max_gap=0.0,
            max_penetration=0.0,
        )

    tilt_limits = head_tilt.motion_limits
    assert tilt_limits is not None and tilt_limits.lower is not None and tilt_limits.upper is not None
    rest_ring_aabb = ctx.part_element_world_aabb(light_head, elem=ring_housing)
    assert rest_ring_aabb is not None
    with ctx.pose({head_tilt: tilt_limits.upper}):
        tilted_ring_aabb = ctx.part_element_world_aabb(light_head, elem=ring_housing)
        assert tilted_ring_aabb is not None
        ctx.check(
            "light_head_tilts_forward",
            tilted_ring_aabb[1][1] > rest_ring_aabb[1][1] + 0.06,
            details=f"Expected tilt to move ring forward; rest={rest_ring_aabb}, tilted={tilted_ring_aabb}",
        )
        ctx.expect_contact(light_head, center_mast, elem_a=left_boss, elem_b=left_clamp)
        ctx.expect_contact(light_head, center_mast, elem_a=right_boss, elem_b=right_clamp)

    leg_1_limits = leg_joints[0].motion_limits
    assert leg_1_limits is not None and leg_1_limits.upper is not None
    rest_foot_aabb = ctx.part_element_world_aabb(leg_1, elem=leg_1_foot)
    assert rest_foot_aabb is not None
    with ctx.pose({leg_joints[0]: leg_1_limits.upper}):
        folded_foot_aabb = ctx.part_element_world_aabb(leg_1, elem=leg_1_foot)
        assert folded_foot_aabb is not None
        ctx.check(
            "leg_fold_motion_changes_height",
            folded_foot_aabb[0][2] > rest_foot_aabb[0][2] + 0.12,
            details=f"Expected folded leg foot to lift; rest={rest_foot_aabb}, folded={folded_foot_aabb}",
        )
        ctx.expect_contact(leg_1, stand_base, elem_a=leg_1_hinge, elem_b=leg_1_lug)

    for articulated_joint in [mast_extension, head_tilt, *leg_joints]:
        articulated_limits = articulated_joint.motion_limits
        assert articulated_limits is not None
        assert articulated_limits.lower is not None
        assert articulated_limits.upper is not None
        with ctx.pose({articulated_joint: articulated_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{articulated_joint.name}_lower_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{articulated_joint.name}_lower_no_floating")
        with ctx.pose({articulated_joint: articulated_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{articulated_joint.name}_upper_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{articulated_joint.name}_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
