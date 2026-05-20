from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pitch_cradle")

    base_width = 0.30
    base_depth = 0.14
    base_thickness = 0.018

    support_center_x = 0.090
    support_thickness = 0.012
    support_depth = 0.11
    support_height = 0.142
    trunnion_axis_z = 0.115
    support_hole_radius = 0.0075

    arm_center_x = 0.079
    arm_thickness = 0.010
    arm_depth = 0.074
    arm_above_axis = 0.018
    arm_below_axis = 0.082
    arm_height = arm_above_axis + arm_below_axis
    arm_outer_x = arm_center_x + (arm_thickness / 2.0)

    bridge_width = 0.166
    bridge_depth = 0.010
    bridge_height = 0.016
    bridge_y = -0.020
    bridge_z = -0.060

    faceplate_width = 0.152
    faceplate_thickness = 0.006
    faceplate_height = 0.085
    faceplate_y = 0.024
    faceplate_z = -0.045

    base_material = model.material("base_powdercoat", color=(0.18, 0.18, 0.20))
    yoke_material = model.material("yoke_alloy", color=(0.72, 0.74, 0.77))
    faceplate_material = model.material("faceplate_black", color=(0.12, 0.13, 0.14))

    def make_support(x_center: float) -> cq.Workplane:
        support = (
            cq.Workplane("XY")
            .box(
                support_thickness,
                support_depth,
                support_height,
                centered=(True, True, False),
            )
            .translate((x_center, 0.0, base_thickness))
        )
        hole = (
            cq.Workplane("YZ")
            .center(0.0, trunnion_axis_z)
            .circle(support_hole_radius)
            .extrude(0.05, both=True)
            .translate((x_center, 0.0, 0.0))
        )
        return support.cut(hole)

    def make_arm(x_center: float) -> cq.Workplane:
        return (
            cq.Workplane("XY")
            .box(
                arm_thickness,
                arm_depth,
                arm_height,
                centered=(True, True, False),
            )
            .translate((x_center, 0.0, -arm_below_axis))
        )

    platform_shape = cq.Workplane("XY").box(
        base_width,
        base_depth,
        base_thickness,
        centered=(True, True, False),
    )
    left_support_shape = make_support(-support_center_x)
    right_support_shape = make_support(support_center_x)

    yoke_frame_shape = (
        make_arm(-arm_center_x)
        .union(make_arm(arm_center_x))
        .union(
            cq.Workplane("XY")
            .box(bridge_width, bridge_depth, bridge_height)
            .translate((0.0, bridge_y, bridge_z))
        )
    )

    faceplate_shape = (
        cq.Workplane("XY")
        .box(faceplate_width, faceplate_thickness, faceplate_height)
        .translate((0.0, faceplate_y, faceplate_z))
    )

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(platform_shape, "base_platform"),
        material=base_material,
        name="base_platform",
    )
    base.visual(
        mesh_from_cadquery(left_support_shape, "left_support"),
        material=base_material,
        name="left_support",
    )
    base.visual(
        mesh_from_cadquery(right_support_shape, "right_support"),
        material=base_material,
        name="right_support",
    )
    base.inertial = Inertial.from_geometry(
        Box((base_width, base_depth, base_thickness + support_height)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, (base_thickness + support_height) / 2.0)),
    )

    yoke = model.part("yoke")
    yoke.visual(
        mesh_from_cadquery(yoke_frame_shape, "yoke_frame"),
        material=yoke_material,
        name="yoke_frame",
    )
    yoke.visual(
        mesh_from_cadquery(faceplate_shape, "faceplate"),
        material=faceplate_material,
        name="faceplate",
    )
    yoke.inertial = Inertial.from_geometry(
        Box((0.168, arm_depth, arm_height)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, (-arm_below_axis + arm_above_axis) / 2.0)),
    )

    model.articulation(
        "base_to_yoke",
        ArticulationType.REVOLUTE,
        parent=base,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, trunnion_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=-1.0,
            upper=1.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    yoke = object_model.get_part("yoke")
    pitch = object_model.get_articulation("base_to_yoke")
    base_platform = base.get_visual("base_platform")
    left_support = base.get_visual("left_support")
    right_support = base.get_visual("right_support")
    yoke_frame = yoke.get_visual("yoke_frame")
    faceplate = yoke.get_visual("faceplate")

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
        "pitch_axis_is_horizontal_x",
        tuple(float(v) for v in pitch.axis) == (1.0, 0.0, 0.0),
        f"expected x-axis revolute joint, got axis={pitch.axis}",
    )

    limits = pitch.motion_limits
    ctx.check(
        "pitch_limits_cover_useful_tilt_range",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower <= -0.9
        and limits.upper >= 0.9,
        f"unexpected pitch limits: {limits}",
    )

    left_support_aabb = ctx.part_element_world_aabb(base, elem="left_support")
    right_support_aabb = ctx.part_element_world_aabb(base, elem="right_support")
    if left_support_aabb is not None and right_support_aabb is not None:
        left_center_x = 0.5 * (left_support_aabb[0][0] + left_support_aabb[1][0])
        right_center_x = 0.5 * (right_support_aabb[0][0] + right_support_aabb[1][0])
        ctx.check(
            "side_supports_are_symmetric",
            abs(left_center_x + right_center_x) <= 0.002,
            f"support centers are not mirrored: left={left_center_x}, right={right_center_x}",
        )

    with ctx.pose({pitch: 0.0}):
        ctx.expect_origin_gap(
            yoke,
            base,
            axis="z",
            min_gap=0.110,
            max_gap=0.120,
            name="trunnion_axis_height_above_base",
        )
        ctx.expect_gap(
            yoke,
            base,
            axis="z",
            negative_elem=base_platform,
            min_gap=0.008,
            name="yoke_clears_base_platform_neutral",
        )
        ctx.expect_contact(
            yoke,
            base,
            elem_a=yoke_frame,
            elem_b=left_support,
            name="left_trunnion_contacts_left_support",
        )
        ctx.expect_contact(
            yoke,
            base,
            elem_a=yoke_frame,
            elem_b=right_support,
            name="right_trunnion_contacts_right_support",
        )
        ctx.expect_within(
            yoke,
            yoke,
            axes="xz",
            inner_elem=faceplate,
            outer_elem=yoke_frame,
            margin=0.025,
            name="faceplate_stays_between_yoke_arms",
        )
        ctx.expect_overlap(
            yoke,
            base,
            axes="x",
            min_overlap=0.16,
            name="yoke_span_aligns_with_side_support_width",
        )

    ctx.fail_if_articulation_overlaps(
        max_pose_samples=24,
        name="pitch_sweep_parent_child_clearance",
    )

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

    if limits is not None and limits.lower is not None and limits.upper is not None:
        for label, angle in (("lower", limits.lower), ("upper", limits.upper)):
            with ctx.pose({pitch: angle}):
                ctx.fail_if_parts_overlap_in_current_pose(
                    name=f"base_to_yoke_{label}_no_overlap"
                )
                ctx.fail_if_isolated_parts(name=f"base_to_yoke_{label}_no_floating")
                ctx.expect_gap(
                    yoke,
                    base,
                    axis="z",
                    negative_elem=base_platform,
                    min_gap=0.008,
                    name=f"yoke_clears_base_platform_{label}",
                )
                ctx.expect_contact(
                    yoke,
                    base,
                    elem_a=yoke_frame,
                    elem_b=left_support,
                    name=f"left_trunnion_contact_{label}",
                )
                ctx.expect_contact(
                    yoke,
                    base,
                    elem_a=yoke_frame,
                    elem_b=right_support,
                    name=f"right_trunnion_contact_{label}",
                )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
