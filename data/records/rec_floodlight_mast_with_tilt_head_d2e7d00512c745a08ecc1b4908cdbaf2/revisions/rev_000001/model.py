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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="facade_wash_light")

    housing_finish = model.material("housing_finish", rgba=(0.11, 0.11, 0.12, 1.0))
    bracket_finish = model.material("bracket_finish", rgba=(0.18, 0.18, 0.19, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.55, 0.64, 0.72, 0.35))

    # Root wall bracket. The part frame sits on the vertical pan axis.
    wall_bracket = model.part("wall_bracket")
    wall_bracket.visual(
        Box((0.012, 0.22, 0.30)),
        origin=Origin(xyz=(-0.104, 0.0, 0.0)),
        material=bracket_finish,
        name="wall_plate",
    )
    wall_bracket.visual(
        Box((0.100, 0.048, 0.060)),
        origin=Origin(xyz=(-0.049, 0.0, 0.0)),
        material=bracket_finish,
        name="support_arm",
    )
    wall_bracket.visual(
        Cylinder(radius=0.055, length=0.024),
        origin=Origin(xyz=(-0.012, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bracket_finish,
        name="pan_plate",
    )
    wall_bracket.visual(
        Box((0.052, 0.020, 0.082)),
        origin=Origin(xyz=(-0.036, 0.0, 0.074)),
        material=bracket_finish,
        name="upper_brace",
    )
    wall_bracket.visual(
        Box((0.052, 0.020, 0.082)),
        origin=Origin(xyz=(-0.036, 0.0, -0.074)),
        material=bracket_finish,
        name="lower_brace",
    )

    pan_yoke = model.part("pan_yoke")
    pan_yoke.visual(
        Cylinder(radius=0.050, length=0.020),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bracket_finish,
        name="rear_swivel",
    )
    pan_yoke.visual(
        Box((0.050, 0.420, 0.040)),
        origin=Origin(xyz=(0.035, 0.0, 0.0)),
        material=bracket_finish,
        name="crossbar",
    )
    pan_yoke.visual(
        Box((0.110, 0.018, 0.200)),
        origin=Origin(xyz=(0.105, -0.201, 0.0)),
        material=bracket_finish,
        name="left_arm",
    )
    pan_yoke.visual(
        Box((0.110, 0.018, 0.200)),
        origin=Origin(xyz=(0.105, 0.201, 0.0)),
        material=bracket_finish,
        name="right_arm",
    )

    housing = model.part("housing")
    housing.visual(
        Box((0.240, 0.360, 0.004)),
        origin=Origin(xyz=(0.020, 0.0, 0.088)),
        material=housing_finish,
        name="top_shell",
    )
    housing.visual(
        Box((0.240, 0.360, 0.004)),
        origin=Origin(xyz=(0.020, 0.0, -0.088)),
        material=housing_finish,
        name="bottom_shell",
    )
    housing.visual(
        Box((0.240, 0.004, 0.172)),
        origin=Origin(xyz=(0.020, -0.178, 0.0)),
        material=housing_finish,
        name="left_shell",
    )
    housing.visual(
        Box((0.240, 0.004, 0.172)),
        origin=Origin(xyz=(0.020, 0.178, 0.0)),
        material=housing_finish,
        name="right_shell",
    )
    housing.visual(
        Box((0.005, 0.360, 0.180)),
        origin=Origin(xyz=(-0.0975, 0.0, 0.0)),
        material=housing_finish,
        name="back_shell",
    )
    housing.visual(
        Box((0.012, 0.360, 0.018)),
        origin=Origin(xyz=(0.134, 0.0, 0.081)),
        material=housing_finish,
        name="bezel_top",
    )
    housing.visual(
        Box((0.012, 0.360, 0.018)),
        origin=Origin(xyz=(0.134, 0.0, -0.081)),
        material=housing_finish,
        name="bezel_bottom",
    )
    housing.visual(
        Box((0.012, 0.018, 0.144)),
        origin=Origin(xyz=(0.134, -0.171, 0.0)),
        material=housing_finish,
        name="bezel_left",
    )
    housing.visual(
        Box((0.012, 0.018, 0.144)),
        origin=Origin(xyz=(0.134, 0.171, 0.0)),
        material=housing_finish,
        name="bezel_right",
    )
    housing.visual(
        Box((0.008, 0.324, 0.144)),
        origin=Origin(xyz=(0.128, 0.0, 0.0)),
        material=lens_glass,
        name="lens",
    )
    housing.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.0, -0.186, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bracket_finish,
        name="left_trunnion",
    )
    housing.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.0, 0.186, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bracket_finish,
        name="right_trunnion",
    )

    top_flap = model.part("top_flap")
    top_flap.visual(
        Box((0.110, 0.332, 0.006)),
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
        material=housing_finish,
        name="panel",
    )
    top_flap.visual(
        Cylinder(radius=0.006, length=0.310),
        origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bracket_finish,
        name="hinge_barrel",
    )

    bottom_flap = model.part("bottom_flap")
    bottom_flap.visual(
        Box((0.110, 0.332, 0.006)),
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
        material=housing_finish,
        name="panel",
    )
    bottom_flap.visual(
        Cylinder(radius=0.006, length=0.310),
        origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bracket_finish,
        name="hinge_barrel",
    )

    left_flap = model.part("left_flap")
    left_flap.visual(
        Box((0.110, 0.006, 0.144)),
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
        material=housing_finish,
        name="panel",
    )
    left_flap.visual(
        Cylinder(radius=0.006, length=0.122),
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
        material=bracket_finish,
        name="hinge_barrel",
    )

    right_flap = model.part("right_flap")
    right_flap.visual(
        Box((0.110, 0.006, 0.144)),
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
        material=housing_finish,
        name="panel",
    )
    right_flap.visual(
        Cylinder(radius=0.006, length=0.122),
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
        material=bracket_finish,
        name="hinge_barrel",
    )

    model.articulation(
        "bracket_to_pan_yoke",
        ArticulationType.REVOLUTE,
        parent=wall_bracket,
        child=pan_yoke,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.5, lower=-1.4, upper=1.4),
    )
    model.articulation(
        "yoke_to_housing",
        ArticulationType.REVOLUTE,
        parent=pan_yoke,
        child=housing,
        origin=Origin(xyz=(0.160, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=16.0, velocity=1.5, lower=-0.8, upper=1.0),
    )
    model.articulation(
        "housing_to_top_flap",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=top_flap,
        origin=Origin(xyz=(0.140, 0.0, 0.081)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=-0.2, upper=1.35),
    )
    model.articulation(
        "housing_to_bottom_flap",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=bottom_flap,
        origin=Origin(xyz=(0.140, 0.0, -0.081)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=-0.2, upper=1.35),
    )
    model.articulation(
        "housing_to_left_flap",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=left_flap,
        origin=Origin(xyz=(0.140, -0.171, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=-0.2, upper=1.35),
    )
    model.articulation(
        "housing_to_right_flap",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=right_flap,
        origin=Origin(xyz=(0.140, 0.171, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=-0.2, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_bracket = object_model.get_part("wall_bracket")
    pan_yoke = object_model.get_part("pan_yoke")
    housing = object_model.get_part("housing")
    top_flap = object_model.get_part("top_flap")
    bottom_flap = object_model.get_part("bottom_flap")
    left_flap = object_model.get_part("left_flap")
    right_flap = object_model.get_part("right_flap")

    pan_joint = object_model.get_articulation("bracket_to_pan_yoke")
    tilt_joint = object_model.get_articulation("yoke_to_housing")
    top_hinge = object_model.get_articulation("housing_to_top_flap")
    bottom_hinge = object_model.get_articulation("housing_to_bottom_flap")
    left_hinge = object_model.get_articulation("housing_to_left_flap")
    right_hinge = object_model.get_articulation("housing_to_right_flap")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

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

    ctx.expect_contact(
        pan_yoke,
        wall_bracket,
        elem_a="rear_swivel",
        elem_b="pan_plate",
        name="pan yoke bears on the wall bracket pan plate",
    )
    ctx.expect_contact(
        housing,
        pan_yoke,
        elem_a="left_trunnion",
        elem_b="left_arm",
        name="left trunnion is supported by the yoke arm",
    )
    ctx.expect_contact(
        housing,
        pan_yoke,
        elem_a="right_trunnion",
        elem_b="right_arm",
        name="right trunnion is supported by the yoke arm",
    )
    ctx.expect_contact(
        top_flap,
        housing,
        elem_a="hinge_barrel",
        elem_b="bezel_top",
        name="top barn door is hinged off the top bezel",
    )
    ctx.expect_contact(
        bottom_flap,
        housing,
        elem_a="hinge_barrel",
        elem_b="bezel_bottom",
        name="bottom barn door is hinged off the bottom bezel",
    )
    ctx.expect_contact(
        left_flap,
        housing,
        elem_a="hinge_barrel",
        elem_b="bezel_left",
        name="left barn door is hinged off the left bezel",
    )
    ctx.expect_contact(
        right_flap,
        housing,
        elem_a="hinge_barrel",
        elem_b="bezel_right",
        name="right barn door is hinged off the right bezel",
    )

    ctx.check(
        "pan joint uses a vertical axis",
        tuple(pan_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={pan_joint.axis}",
    )
    ctx.check(
        "tilt joint uses a horizontal pitch axis",
        tuple(tilt_joint.axis) == (0.0, -1.0, 0.0),
        details=f"axis={tilt_joint.axis}",
    )
    ctx.check(
        "barn door hinge axes are independent and correctly oriented",
        tuple(top_hinge.axis) == (0.0, 1.0, 0.0)
        and tuple(bottom_hinge.axis) == (0.0, -1.0, 0.0)
        and tuple(left_hinge.axis) == (0.0, 0.0, 1.0)
        and tuple(right_hinge.axis) == (0.0, 0.0, -1.0),
        details=(
            f"top={top_hinge.axis}, bottom={bottom_hinge.axis}, "
            f"left={left_hinge.axis}, right={right_hinge.axis}"
        ),
    )

    def center_from_aabb(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    rest_housing_pos = ctx.part_world_position(housing)
    with ctx.pose({pan_joint: 0.7}):
        panned_housing_pos = ctx.part_world_position(housing)
    ctx.check(
        "positive pan swings the head toward +y",
        rest_housing_pos is not None
        and panned_housing_pos is not None
        and panned_housing_pos[1] > rest_housing_pos[1] + 0.08,
        details=f"rest={rest_housing_pos}, panned={panned_housing_pos}",
    )

    rest_lens_center = center_from_aabb(ctx.part_element_world_aabb(housing, elem="lens"))
    with ctx.pose({tilt_joint: 0.6}):
        tilted_lens_center = center_from_aabb(ctx.part_element_world_aabb(housing, elem="lens"))
    ctx.check(
        "positive tilt raises the front lens center",
        rest_lens_center is not None
        and tilted_lens_center is not None
        and tilted_lens_center[2] > rest_lens_center[2] + 0.04,
        details=f"rest={rest_lens_center}, tilted={tilted_lens_center}",
    )

    rest_top_flap_center = center_from_aabb(ctx.part_element_world_aabb(top_flap, elem="panel"))
    with ctx.pose({top_hinge: 1.0}):
        closed_top_flap_center = center_from_aabb(ctx.part_element_world_aabb(top_flap, elem="panel"))
    ctx.check(
        "top barn door swings downward when closed",
        rest_top_flap_center is not None
        and closed_top_flap_center is not None
        and closed_top_flap_center[2] < rest_top_flap_center[2] - 0.03,
        details=f"rest={rest_top_flap_center}, closed={closed_top_flap_center}",
    )

    rest_left_flap_center = center_from_aabb(ctx.part_element_world_aabb(left_flap, elem="panel"))
    with ctx.pose({left_hinge: 1.0}):
        closed_left_flap_center = center_from_aabb(ctx.part_element_world_aabb(left_flap, elem="panel"))
    ctx.check(
        "left barn door swings inward across the aperture",
        rest_left_flap_center is not None
        and closed_left_flap_center is not None
        and closed_left_flap_center[1] > rest_left_flap_center[1] + 0.03,
        details=f"rest={rest_left_flap_center}, closed={closed_left_flap_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
