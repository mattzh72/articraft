from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


BASE_LENGTH = 0.22
BASE_DEPTH = 0.14
BASE_THICKNESS = 0.02

SPINE_WIDTH = 0.08
SPINE_DEPTH = 0.06
SPINE_HEIGHT = 0.44

UPPER_HUB_Z = 0.36
LOWER_HUB_Z = 0.20
HUB_AXIS_X = 0.086

SHAFT_RADIUS = 0.010
SLEEVE_OUTER_RADIUS = 0.024
SLEEVE_LENGTH = 0.044
CHEEK_THICKNESS = 0.010
CHEEK_CENTER_Y = 0.035
CHEEK_LENGTH_X = 0.038
CHEEK_HEIGHT_Z = 0.070
SHAFT_LENGTH = 0.092

UPPER_ARM_LENGTH = 0.235
LOWER_ARM_LENGTH = 0.195
ARM_WIDTH = 0.024
PAD_RADIUS = 0.018
PAD_THICKNESS = 0.010


def y_axis_cylinder(radius: float, length: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
    )


def y_axis_annulus(outer_radius: float, inner_radius: float, length: float) -> cq.Workplane:
    outer = y_axis_cylinder(outer_radius, length)
    inner = y_axis_cylinder(inner_radius, length + 0.002)
    return outer.cut(inner)


def build_frame_shape() -> cq.Workplane:
    frame = cq.Workplane("XY").box(BASE_LENGTH, BASE_DEPTH, BASE_THICKNESS).translate(
        (0.0, 0.0, BASE_THICKNESS / 2.0)
    )
    frame = frame.union(
        cq.Workplane("XY").box(SPINE_WIDTH, SPINE_DEPTH, SPINE_HEIGHT).translate(
            (0.0, 0.0, BASE_THICKNESS + SPINE_HEIGHT / 2.0)
        )
    )

    lower_stiffener = cq.Workplane("XY").box(0.12, 0.05, 0.06).translate((0.0, 0.0, 0.05))
    frame = frame.union(lower_stiffener)

    for z_level in (UPPER_HUB_Z, LOWER_HUB_Z):
        for y_center in (CHEEK_CENTER_Y, -CHEEK_CENTER_Y):
            cheek = cq.Workplane("XY").box(CHEEK_LENGTH_X, CHEEK_THICKNESS, CHEEK_HEIGHT_Z).translate(
                (0.069, y_center, z_level)
            )
            rib = cq.Workplane("XY").box(0.054, 0.014, 0.040).translate((0.040, y_center, z_level))
            frame = frame.union(cheek).union(rib)

        shaft = (
            y_axis_cylinder(0.0085, SHAFT_LENGTH).translate((HUB_AXIS_X, 0.0, z_level))
        )
        frame = frame.union(shaft)

        for y_center in (0.041, -0.041):
            bearing = (
                y_axis_annulus(0.0155, 0.0105, 0.010).translate((HUB_AXIS_X, y_center, z_level))
            )
            frame = frame.union(bearing)

        shaft_lock = cq.Workplane("XY").box(0.016, 0.060, 0.014).translate(
            (0.054, 0.0, z_level - 0.022)
        )
        frame = frame.union(shaft_lock)

    return frame


def build_arm_body(length: float) -> cq.Workplane:
    hub_sleeve = y_axis_annulus(0.0135, 0.0092, 0.060)
    root_block = cq.Workplane("XY").box(0.030, 0.022, 0.020).translate((0.052, 0.0, 0.0))
    beam = cq.Workplane("XY").box(length - 0.082, ARM_WIDTH, 0.018).translate(
        ((length + 0.082) / 2.0, 0.0, 0.0)
    )
    gusset = (
        cq.Workplane("XZ")
        .polyline([(0.036, -0.009), (0.078, -0.009), (0.056, 0.014)])
        .close()
        .extrude(0.016, both=True)
    )
    tip_round = cq.Workplane("YZ").circle(0.012).extrude(0.016).translate((length - 0.016, 0.0, 0.0))

    return hub_sleeve.union(root_block).union(beam).union(gusset).union(tip_round)


def build_tool_pad(length: float) -> cq.Workplane:
    return cq.Workplane("YZ").circle(PAD_RADIUS).extrude(PAD_THICKNESS).translate((length, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_branch_positioning_fixture")

    frame_material = model.material("frame_dark", rgba=(0.22, 0.24, 0.27, 1.0))
    arm_material = model.material("arm_satin", rgba=(0.74, 0.76, 0.79, 1.0))
    pad_material = model.material("tool_pad_amber", rgba=(0.84, 0.56, 0.20, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(build_frame_shape(), "fixture_frame_body"),
        material=frame_material,
        name="frame_body",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        mesh_from_cadquery(build_arm_body(UPPER_ARM_LENGTH), "upper_branch_arm_body"),
        material=arm_material,
        name="upper_arm_body",
    )
    upper_arm.visual(
        mesh_from_cadquery(build_tool_pad(UPPER_ARM_LENGTH), "upper_branch_tool_pad"),
        material=pad_material,
        name="upper_tool_pad",
    )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        mesh_from_cadquery(build_arm_body(LOWER_ARM_LENGTH), "lower_branch_arm_body"),
        material=arm_material,
        name="lower_arm_body",
    )
    lower_arm.visual(
        mesh_from_cadquery(build_tool_pad(LOWER_ARM_LENGTH), "lower_branch_tool_pad"),
        material=pad_material,
        name="lower_tool_pad",
    )

    model.articulation(
        "upper_branch_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=upper_arm,
        origin=Origin(xyz=(HUB_AXIS_X, 0.0, UPPER_HUB_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=22.0, velocity=1.2, lower=-0.55, upper=0.0),
    )

    model.articulation(
        "lower_branch_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=lower_arm,
        origin=Origin(xyz=(HUB_AXIS_X, 0.0, LOWER_HUB_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=22.0, velocity=1.2, lower=0.0, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    upper_arm = object_model.get_part("upper_arm")
    lower_arm = object_model.get_part("lower_arm")
    upper_hinge = object_model.get_articulation("upper_branch_hinge")
    lower_hinge = object_model.get_articulation("lower_branch_hinge")
    frame_body = frame.get_visual("frame_body")
    upper_body = upper_arm.get_visual("upper_arm_body")
    lower_body = lower_arm.get_visual("lower_arm_body")
    upper_pad = upper_arm.get_visual("upper_tool_pad")
    lower_pad = lower_arm.get_visual("lower_tool_pad")

    ctx.allow_overlap(
        frame,
        upper_arm,
        reason="The upper branch hub visually surrounds the stationary support shaft; the running-fit shaft-in-bore is intentionally co-located in the rendered geometry.",
        elem_a=frame_body,
        elem_b=upper_body,
    )
    ctx.allow_overlap(
        frame,
        lower_arm,
        reason="The lower branch hub visually surrounds the stationary support shaft; the running-fit shaft-in-bore is intentionally co-located in the rendered geometry.",
        elem_a=frame_body,
        elem_b=lower_body,
    )

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

    ctx.check(
        "branch_joints_are_revolute",
        upper_hinge.articulation_type == ArticulationType.REVOLUTE
        and lower_hinge.articulation_type == ArticulationType.REVOLUTE,
        details="Both branch arms should be independent revolute joints.",
    )
    ctx.check(
        "branch_axes_are_parallel",
        tuple(round(v, 6) for v in upper_hinge.axis) == tuple(round(v, 6) for v in lower_hinge.axis),
        details=f"upper axis={upper_hinge.axis}, lower axis={lower_hinge.axis}",
    )

    ctx.expect_contact(frame, upper_arm, contact_tol=0.001, name="upper_arm_supported_on_frame")
    ctx.expect_contact(frame, lower_arm, contact_tol=0.001, name="lower_arm_supported_on_frame")
    ctx.expect_origin_gap(
        upper_arm,
        lower_arm,
        axis="z",
        min_gap=0.14,
        max_gap=0.18,
        name="upper_and_lower_hubs_are_separate_along_spine",
    )
    ctx.expect_origin_gap(
        upper_arm,
        frame,
        axis="z",
        min_gap=0.34,
        max_gap=0.38,
        name="upper_hub_sits_high_on_spine",
    )
    ctx.expect_origin_gap(
        lower_arm,
        frame,
        axis="z",
        min_gap=0.18,
        max_gap=0.22,
        name="lower_hub_sits_low_on_spine",
    )

    for joint, child, label in (
        (upper_hinge, upper_arm, "upper_branch_hinge"),
        (lower_hinge, lower_arm, "lower_branch_hinge"),
    ):
        limits = joint.motion_limits
        ctx.check(
            f"{label}_limits_present",
            limits is not None and limits.lower is not None and limits.upper is not None,
            details=f"{label} needs bounded motion limits for a realistic fixture arm.",
        )
        if limits is not None and limits.lower is not None and limits.upper is not None:
            for pose_label, joint_value in (("lower", limits.lower), ("upper", limits.upper)):
                with ctx.pose({joint: joint_value}):
                    ctx.fail_if_parts_overlap_in_current_pose(name=f"{label}_{pose_label}_no_overlap")
                    ctx.fail_if_isolated_parts(name=f"{label}_{pose_label}_no_floating")
                    ctx.expect_contact(
                        frame,
                        child,
                        contact_tol=0.001,
                        name=f"{label}_{pose_label}_still_supported_on_shaft",
                    )

    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=24,
        ignore_adjacent=False,
        ignore_fixed=False,
    )

    frame_aabb = ctx.part_world_aabb(frame)
    upper_pad_aabb = ctx.part_element_world_aabb(upper_arm, elem=upper_pad)
    lower_pad_aabb = ctx.part_element_world_aabb(lower_arm, elem=lower_pad)
    upper_pad_outboard = (
        frame_aabb is not None
        and upper_pad_aabb is not None
        and upper_pad_aabb[0][0] > frame_aabb[1][0] + 0.10
    )
    lower_pad_outboard = (
        frame_aabb is not None
        and lower_pad_aabb is not None
        and lower_pad_aabb[0][0] > frame_aabb[1][0] + 0.07
    )
    ctx.check(
        "upper_tool_pad_reaches_past_frame",
        upper_pad_outboard,
        details=f"frame_aabb={frame_aabb}, upper_pad_aabb={upper_pad_aabb}",
    )
    ctx.check(
        "lower_tool_pad_reaches_past_frame",
        lower_pad_outboard,
        details=f"frame_aabb={frame_aabb}, lower_pad_aabb={lower_pad_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
