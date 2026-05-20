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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    wire_from_points,
)


BASE_STILE_CENTERS = (-0.185, 0.185)
BASE_LENGTH = 3.60
BASE_BOTTOM_Z = 0.04
BASE_RUNG_Y = -0.010
BASE_RUNG_RADIUS = 0.012
BASE_RUNG_LENGTH = 0.34

FLY_LENGTH = 3.00
FLY_JOINT_Z = 0.70
FLY_STILE_CENTERS = (-0.155, 0.155)
FLY_RUNG_Y = 0.016
FLY_RUNG_RADIUS = 0.011
FLY_RUNG_LENGTH = 0.332

ARM_JOINT_Y = 0.044
ARM_JOINT_Z = FLY_LENGTH - 0.02
FOOT_JOINT_Y = -0.005
FOOT_JOINT_Z = 0.031


def _x_cylinder_origin(
    xyz: tuple[float, float, float],
) -> Origin:
    return Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0))


def _rounded_pad_mesh(width: float, depth: float, height: float, radius: float, name: str):
    return mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(width, depth, radius, corner_segments=8),
            height,
            center=True,
        ),
        name,
    )


def _standoff_arm_tube_mesh():
    return mesh_from_geometry(
        wire_from_points(
            [
                (0.0, 0.0, 0.0),
                (0.0, 0.032, 0.0),
                (0.0, 0.040, 0.182),
                (0.0, 0.300, 0.188),
            ],
            radius=0.009,
            radial_segments=16,
            cap_ends=True,
            corner_mode="fillet",
            corner_radius=0.028,
            corner_segments=10,
        ),
        "standoff_arm_tube",
    )


def _add_base_channel_stile(part, *, x: float, material) -> None:
    z_center = BASE_BOTTOM_Z + BASE_LENGTH * 0.5
    part.visual(
        Box((0.030, 0.010, BASE_LENGTH)),
        origin=Origin(xyz=(x, -0.025, z_center)),
        material=material,
        name=f"base_stile_web_{'left' if x < 0 else 'right'}",
    )
    for dx, side in ((-0.0125, "outer"), (0.0125, "inner")):
        part.visual(
            Box((0.005, 0.050, BASE_LENGTH)),
            origin=Origin(xyz=(x + dx, 0.005, z_center)),
            material=material,
            name=f"base_stile_flange_{'left' if x < 0 else 'right'}_{side}",
        )


def _add_base_rungs(part, *, material) -> None:
    for index in range(11):
        z = 0.40 + index * 0.28
        part.visual(
            Cylinder(radius=BASE_RUNG_RADIUS, length=BASE_RUNG_LENGTH),
            origin=_x_cylinder_origin((0.0, BASE_RUNG_Y, z)),
            material=material,
            name=f"base_rung_{index}",
        )


def _add_base_foot_mount(part, *, x: float, material) -> None:
    part.visual(
        Box((0.030, 0.014, 0.018)),
        origin=Origin(xyz=(x, FOOT_JOINT_Y, 0.049)),
        material=material,
        name=f"foot_bridge_{'left' if x < 0 else 'right'}",
    )
    for dx, side in ((-0.017, "outer"), (0.017, "inner")):
        part.visual(
            Box((0.006, 0.024, 0.022)),
            origin=Origin(xyz=(x + dx, FOOT_JOINT_Y, FOOT_JOINT_Z)),
            material=material,
            name=f"foot_cheek_{'left' if x < 0 else 'right'}_{side}",
        )


def _add_fly_stile(part, *, x: float, material) -> None:
    part.visual(
        Box((0.018, 0.022, FLY_LENGTH)),
        origin=Origin(xyz=(x, 0.016, FLY_LENGTH * 0.5)),
        material=material,
        name=f"fly_stile_main_{'left' if x < 0 else 'right'}",
    )
    guide_x = x - 0.006 if x < 0 else x + 0.006
    part.visual(
        Box((0.018, 0.025, FLY_LENGTH)),
        origin=Origin(xyz=(guide_x, -0.0075, FLY_LENGTH * 0.5)),
        material=material,
        name=f"fly_stile_guide_{'left' if x < 0 else 'right'}",
    )


def _add_fly_rungs(part, *, material) -> None:
    for index in range(10):
        z = 0.28 + index * 0.28
        part.visual(
            Cylinder(radius=FLY_RUNG_RADIUS, length=FLY_RUNG_LENGTH),
            origin=_x_cylinder_origin((0.0, FLY_RUNG_Y, z)),
            material=material,
            name=f"fly_rung_{index}",
        )


def _add_arm_mount(part, *, x: float, material) -> None:
    side = "left" if x < 0 else "right"
    part.visual(
        Box((0.030, 0.012, 0.050)),
        origin=Origin(xyz=(x, ARM_JOINT_Y - 0.016, ARM_JOINT_Z)),
        material=material,
        name=f"arm_bridge_{side}",
    )
    for dx, cheek in ((-0.018, "outer"), (0.018, "inner")):
        part.visual(
            Box((0.006, 0.024, 0.030)),
            origin=Origin(xyz=(x + dx, ARM_JOINT_Y, ARM_JOINT_Z)),
            material=material,
            name=f"arm_cheek_{side}_{cheek}",
        )


def _build_arm_part(
    model: ArticulatedObject,
    name: str,
    *,
    arm_material,
    pad_material,
    tube_mesh,
    shoe_mesh,
):
    arm = model.part(name)
    arm.visual(
        Cylinder(radius=0.009, length=0.032),
        origin=_x_cylinder_origin((0.0, 0.0, 0.0)),
        material=arm_material,
        name="hinge_barrel",
    )
    arm.visual(
        Box((0.018, 0.034, 0.018)),
        origin=Origin(xyz=(0.0, 0.017, 0.0)),
        material=arm_material,
        name="pivot_block",
    )
    arm.visual(
        tube_mesh,
        origin=Origin(),
        material=arm_material,
        name="tube_frame",
    )
    arm.visual(
        shoe_mesh,
        origin=Origin(xyz=(0.0, 0.335, 0.188)),
        material=pad_material,
        name="shoe",
    )
    arm.visual(
        Box((0.028, 0.060, 0.090)),
        origin=Origin(xyz=(0.0, 0.315, 0.233)),
        material=arm_material,
        name="upstand",
    )
    arm.inertial = Inertial.from_geometry(
        Box((0.080, 0.360, 0.280)),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.180, 0.140)),
    )
    return arm


def _build_foot_part(
    model: ArticulatedObject,
    name: str,
    *,
    steel_material,
    rubber_material,
    pad_mesh,
):
    foot = model.part(name)
    foot.visual(
        Cylinder(radius=0.010, length=0.022),
        origin=_x_cylinder_origin((0.0, 0.0, 0.0)),
        material=steel_material,
        name="hinge_barrel",
    )
    foot.visual(
        Box((0.020, 0.020, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=steel_material,
        name="neck",
    )
    foot.visual(
        pad_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.037)),
        material=rubber_material,
        name="pad",
    )
    foot.inertial = Inertial.from_geometry(
        Box((0.100, 0.060, 0.052)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, -0.024)),
    )
    return foot


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="roof_access_extension_ladder")

    aluminum = model.material("aluminum", rgba=(0.79, 0.82, 0.84, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.56, 0.58, 0.60, 1.0))
    rubber = model.material("rubber", rgba=(0.12, 0.12, 0.12, 1.0))
    arm_tube_mesh = _standoff_arm_tube_mesh()
    arm_shoe_mesh = _rounded_pad_mesh(0.070, 0.100, 0.018, 0.015, "standoff_arm_shoe")
    foot_pad_mesh = _rounded_pad_mesh(0.095, 0.060, 0.014, 0.018, "swivel_foot_pad")

    base_frame = model.part("base_frame")
    for x in BASE_STILE_CENTERS:
        _add_base_channel_stile(base_frame, x=x, material=aluminum)
        _add_base_foot_mount(base_frame, x=x, material=brushed_steel)
    _add_base_rungs(base_frame, material=brushed_steel)
    base_frame.inertial = Inertial.from_geometry(
        Box((0.420, 0.080, 3.620)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, 1.850)),
    )

    fly_section = model.part("fly_section")
    for x in FLY_STILE_CENTERS:
        _add_fly_stile(fly_section, x=x, material=aluminum)
        _add_arm_mount(fly_section, x=x, material=brushed_steel)
    _add_fly_rungs(fly_section, material=brushed_steel)
    fly_section.inertial = Inertial.from_geometry(
        Box((0.350, 0.055, FLY_LENGTH)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.004, FLY_LENGTH * 0.5)),
    )

    left_standoff_arm = _build_arm_part(
        model,
        "left_standoff_arm",
        arm_material=aluminum,
        pad_material=rubber,
        tube_mesh=arm_tube_mesh,
        shoe_mesh=arm_shoe_mesh,
    )
    right_standoff_arm = _build_arm_part(
        model,
        "right_standoff_arm",
        arm_material=aluminum,
        pad_material=rubber,
        tube_mesh=arm_tube_mesh,
        shoe_mesh=arm_shoe_mesh,
    )

    left_swivel_foot = _build_foot_part(
        model,
        "left_swivel_foot",
        steel_material=brushed_steel,
        rubber_material=rubber,
        pad_mesh=foot_pad_mesh,
    )
    right_swivel_foot = _build_foot_part(
        model,
        "right_swivel_foot",
        steel_material=brushed_steel,
        rubber_material=rubber,
        pad_mesh=foot_pad_mesh,
    )

    model.articulation(
        "base_to_fly",
        ArticulationType.PRISMATIC,
        parent=base_frame,
        child=fly_section,
        origin=Origin(xyz=(0.0, 0.050, FLY_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.40,
            lower=0.0,
            upper=0.95,
        ),
    )
    model.articulation(
        "fly_to_left_standoff_arm",
        ArticulationType.REVOLUTE,
        parent=fly_section,
        child=left_standoff_arm,
        origin=Origin(xyz=(FLY_STILE_CENTERS[0], ARM_JOINT_Y, ARM_JOINT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.0,
            lower=-0.25,
            upper=1.05,
        ),
    )
    model.articulation(
        "fly_to_right_standoff_arm",
        ArticulationType.REVOLUTE,
        parent=fly_section,
        child=right_standoff_arm,
        origin=Origin(xyz=(FLY_STILE_CENTERS[1], ARM_JOINT_Y, ARM_JOINT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.0,
            lower=-0.25,
            upper=1.05,
        ),
    )
    model.articulation(
        "base_to_left_swivel_foot",
        ArticulationType.REVOLUTE,
        parent=base_frame,
        child=left_swivel_foot,
        origin=Origin(xyz=(BASE_STILE_CENTERS[0], FOOT_JOINT_Y, FOOT_JOINT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.5,
            lower=-0.45,
            upper=0.45,
        ),
    )
    model.articulation(
        "base_to_right_swivel_foot",
        ArticulationType.REVOLUTE,
        parent=base_frame,
        child=right_swivel_foot,
        origin=Origin(xyz=(BASE_STILE_CENTERS[1], FOOT_JOINT_Y, FOOT_JOINT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.5,
            lower=-0.45,
            upper=0.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    base_frame = object_model.get_part("base_frame")
    fly_section = object_model.get_part("fly_section")
    left_standoff_arm = object_model.get_part("left_standoff_arm")
    right_standoff_arm = object_model.get_part("right_standoff_arm")
    left_swivel_foot = object_model.get_part("left_swivel_foot")
    right_swivel_foot = object_model.get_part("right_swivel_foot")

    fly_slide = object_model.get_articulation("base_to_fly")
    left_arm_hinge = object_model.get_articulation("fly_to_left_standoff_arm")
    right_arm_hinge = object_model.get_articulation("fly_to_right_standoff_arm")
    left_foot_hinge = object_model.get_articulation("base_to_left_swivel_foot")
    right_foot_hinge = object_model.get_articulation("base_to_right_swivel_foot")

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(fly_section, base_frame)
    ctx.expect_contact(left_standoff_arm, fly_section)
    ctx.expect_contact(right_standoff_arm, fly_section)
    ctx.expect_contact(left_swivel_foot, base_frame)
    ctx.expect_contact(right_swivel_foot, base_frame)

    base_aabb = ctx.part_world_aabb(base_frame)
    assert base_aabb is not None
    base_width = base_aabb[1][0] - base_aabb[0][0]
    base_height = base_aabb[1][2] - base_aabb[0][2]
    ctx.check(
        "ladder_width_realistic",
        0.39 <= base_width <= 0.45,
        f"Base ladder width should be around 0.42 m, got {base_width:.3f} m.",
    )
    ctx.check(
        "ladder_height_realistic",
        3.55 <= base_height <= 3.70,
        f"Base ladder height should be around 3.6 m, got {base_height:.3f} m.",
    )

    ctx.check(
        "fly_slide_is_vertical_prismatic",
        fly_slide.joint_type == ArticulationType.PRISMATIC and tuple(fly_slide.axis) == (0.0, 0.0, 1.0),
        f"Expected a vertical prismatic fly section, got {fly_slide.joint_type} with axis {fly_slide.axis}.",
    )
    ctx.check(
        "arm_hinges_are_revolute_about_x",
        left_arm_hinge.joint_type == ArticulationType.REVOLUTE
        and right_arm_hinge.joint_type == ArticulationType.REVOLUTE
        and tuple(left_arm_hinge.axis) == (1.0, 0.0, 0.0)
        and tuple(right_arm_hinge.axis) == (1.0, 0.0, 0.0),
        "Stand-off arm brackets should pivot about the x axis.",
    )
    ctx.check(
        "feet_swivel_about_x",
        left_foot_hinge.joint_type == ArticulationType.REVOLUTE
        and right_foot_hinge.joint_type == ArticulationType.REVOLUTE
        and tuple(left_foot_hinge.axis) == (1.0, 0.0, 0.0)
        and tuple(right_foot_hinge.axis) == (1.0, 0.0, 0.0),
        "Swivel feet should rotate about the x axis.",
    )

    fly_rest_pos = ctx.part_world_position(fly_section)
    assert fly_rest_pos is not None
    fly_upper = fly_slide.motion_limits.upper if fly_slide.motion_limits is not None else None
    assert fly_upper is not None
    with ctx.pose({fly_slide: fly_upper}):
        fly_extended_pos = ctx.part_world_position(fly_section)
        assert fly_extended_pos is not None
        ctx.check(
            "fly_section_extends_upward",
            fly_extended_pos[2] > fly_rest_pos[2] + 0.90,
            f"Fly section should rise by about 0.95 m, got {fly_extended_pos[2] - fly_rest_pos[2]:.3f} m.",
        )
        ctx.expect_contact(fly_section, base_frame, name="fly_section_extended_contact")
        ctx.fail_if_parts_overlap_in_current_pose(name="fly_section_upper_no_overlap")
        ctx.fail_if_isolated_parts(name="fly_section_upper_no_floating")

    left_arm_lower = left_arm_hinge.motion_limits.lower if left_arm_hinge.motion_limits is not None else None
    left_arm_upper = left_arm_hinge.motion_limits.upper if left_arm_hinge.motion_limits is not None else None
    assert left_arm_lower is not None
    assert left_arm_upper is not None
    fly_current_aabb = ctx.part_world_aabb(fly_section)
    assert fly_current_aabb is not None
    with ctx.pose({left_arm_hinge: left_arm_lower, right_arm_hinge: left_arm_lower}):
        left_arm_deployed_aabb = ctx.part_world_aabb(left_standoff_arm)
        right_arm_deployed_aabb = ctx.part_world_aabb(right_standoff_arm)
        assert left_arm_deployed_aabb is not None
        assert right_arm_deployed_aabb is not None
        ctx.check(
            "left_standoff_projects_forward",
            left_arm_deployed_aabb[1][1] > fly_current_aabb[1][1] + 0.25,
            "Left stand-off arm should project forward of the fly section when deployed.",
        )
        ctx.check(
            "right_standoff_projects_forward",
            right_arm_deployed_aabb[1][1] > fly_current_aabb[1][1] + 0.25,
            "Right stand-off arm should project forward of the fly section when deployed.",
        )
        ctx.expect_contact(left_standoff_arm, fly_section, name="left_arm_deployed_contact")
        ctx.expect_contact(right_standoff_arm, fly_section, name="right_arm_deployed_contact")
    with ctx.pose({left_arm_hinge: left_arm_upper, right_arm_hinge: left_arm_upper}):
        left_arm_folded_aabb = ctx.part_world_aabb(left_standoff_arm)
        right_arm_folded_aabb = ctx.part_world_aabb(right_standoff_arm)
        assert left_arm_folded_aabb is not None
        assert right_arm_folded_aabb is not None
        ctx.check(
            "standoff_arms_fold_back",
            left_arm_folded_aabb[1][1] < left_arm_deployed_aabb[1][1] - 0.08
            and right_arm_folded_aabb[1][1] < right_arm_deployed_aabb[1][1] - 0.08,
            "Folding the stand-off arms should noticeably reduce their forward projection.",
        )
        ctx.expect_contact(left_standoff_arm, fly_section, name="left_arm_folded_contact")
        ctx.expect_contact(right_standoff_arm, fly_section, name="right_arm_folded_contact")
        ctx.fail_if_parts_overlap_in_current_pose(name="standoff_arms_lower_no_overlap")
        ctx.fail_if_isolated_parts(name="standoff_arms_lower_no_floating")

    left_foot_rest_aabb = ctx.part_world_aabb(left_swivel_foot)
    right_foot_rest_aabb = ctx.part_world_aabb(right_swivel_foot)
    assert left_foot_rest_aabb is not None
    assert right_foot_rest_aabb is not None
    left_pad_rest_aabb = ctx.part_element_world_aabb(left_swivel_foot, elem="pad")
    right_pad_rest_aabb = ctx.part_element_world_aabb(right_swivel_foot, elem="pad")
    assert left_pad_rest_aabb is not None
    assert right_pad_rest_aabb is not None
    left_foot_upper = left_foot_hinge.motion_limits.upper if left_foot_hinge.motion_limits is not None else None
    right_foot_lower = right_foot_hinge.motion_limits.lower if right_foot_hinge.motion_limits is not None else None
    assert left_foot_upper is not None
    assert right_foot_lower is not None
    with ctx.pose({left_foot_hinge: left_foot_upper, right_foot_hinge: right_foot_lower}):
        left_foot_swiveled_aabb = ctx.part_world_aabb(left_swivel_foot)
        right_foot_swiveled_aabb = ctx.part_world_aabb(right_swivel_foot)
        left_pad_swiveled_aabb = ctx.part_element_world_aabb(left_swivel_foot, elem="pad")
        right_pad_swiveled_aabb = ctx.part_element_world_aabb(right_swivel_foot, elem="pad")
        assert left_foot_swiveled_aabb is not None
        assert right_foot_swiveled_aabb is not None
        assert left_pad_swiveled_aabb is not None
        assert right_pad_swiveled_aabb is not None
        ctx.check(
            "feet_change_pose_when_swiveled",
            left_pad_swiveled_aabb[1][2] > left_pad_rest_aabb[1][2] + 0.010
            and right_pad_swiveled_aabb[0][2] < right_pad_rest_aabb[0][2] - 0.008,
            "Swivel feet should rotate enough to change their world-space envelope.",
        )
        ctx.expect_contact(left_swivel_foot, base_frame, name="left_foot_swiveled_contact")
        ctx.expect_contact(right_swivel_foot, base_frame, name="right_foot_swiveled_contact")
        ctx.fail_if_parts_overlap_in_current_pose(name="feet_swiveled_no_overlap")
        ctx.fail_if_isolated_parts(name="feet_swiveled_no_floating")

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")

    for articulation in (
        fly_slide,
        left_arm_hinge,
        right_arm_hinge,
        left_foot_hinge,
        right_foot_hinge,
    ):
        limits = articulation.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({articulation: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(
                    name=f"{articulation.name}_lower_no_overlap"
                )
                ctx.fail_if_isolated_parts(name=f"{articulation.name}_lower_no_floating")
            with ctx.pose({articulation: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(
                    name=f"{articulation.name}_upper_no_overlap"
                )
                ctx.fail_if_isolated_parts(name=f"{articulation.name}_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
