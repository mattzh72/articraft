from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


FRAME_PLATE_THICKNESS = 0.012
FRAME_PLATE_WIDTH = 0.18
FRAME_PLATE_HEIGHT = 0.46
FRAME_PLATE_CENTER_X = FRAME_PLATE_THICKNESS / 2.0
FRAME_PLATE_CENTER_Z = FRAME_PLATE_HEIGHT / 2.0

VERTICAL_RAIL_SIZE = (0.052, 0.072, 0.34)
VERTICAL_RAIL_CENTER = (
    FRAME_PLATE_THICKNESS + VERTICAL_RAIL_SIZE[0] / 2.0,
    0.0,
    0.23,
)

Z_SLIDE_LOWER_CENTER_Z = 0.12
Z_TRAVEL = 0.16

Y_RAIL_SIZE = (0.052, 0.24, 0.048)
Y_RAIL_LOCAL_CENTER = (0.122, 0.14, 0.0)
Y_STAGE_INNER_CENTER_Y = 0.06
Y_TRAVEL = 0.14

PLATFORM_SIZE = (0.10, 0.07, 0.012)
PLATFORM_LOCAL_CENTER = (0.075, 0.125, 0.086)


def _frame_plate_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(FRAME_PLATE_THICKNESS, FRAME_PLATE_WIDTH, FRAME_PLATE_HEIGHT)
        .edges("|X")
        .fillet(0.012)
    )
    return (
        plate.faces(">X")
        .workplane()
        .pushPoints(
            [
                (-0.06, -0.17),
                (0.06, -0.17),
                (-0.06, 0.17),
                (0.06, 0.17),
            ]
        )
        .hole(0.011)
    )


def _platform_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(*PLATFORM_SIZE)
        .edges("|Z")
        .fillet(0.006)
    )
    return (
        plate.faces(">Z")
        .workplane()
        .pushPoints([(-0.022, 0.0), (0.022, 0.0)])
        .hole(0.006)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_mounted_yz_positioning_stage")

    model.material("frame_gray", rgba=(0.23, 0.25, 0.28, 1.0))
    model.material("rail_steel", rgba=(0.72, 0.75, 0.78, 1.0))
    model.material("carriage_silver", rgba=(0.86, 0.88, 0.90, 1.0))
    model.material("platform_blue", rgba=(0.28, 0.43, 0.70, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_frame_plate_shape(), "frame_plate"),
        origin=Origin(xyz=(FRAME_PLATE_CENTER_X, 0.0, FRAME_PLATE_CENTER_Z)),
        material="frame_gray",
        name="frame_plate",
    )
    frame.visual(
        Box(VERTICAL_RAIL_SIZE),
        origin=Origin(xyz=VERTICAL_RAIL_CENTER),
        material="rail_steel",
        name="vertical_rail",
    )
    z_carriage = model.part("z_carriage")
    z_carriage.visual(
        Box((0.03, 0.12, 0.12)),
        origin=Origin(xyz=(0.041, 0.0, 0.0)),
        material="carriage_silver",
        name="z_front_shoe",
    )
    z_carriage.visual(
        Box((0.082, 0.024, 0.12)),
        origin=Origin(xyz=(0.015, 0.048, 0.0)),
        material="carriage_silver",
        name="z_left_shoe",
    )
    z_carriage.visual(
        Box((0.082, 0.024, 0.12)),
        origin=Origin(xyz=(0.015, -0.048, 0.0)),
        material="carriage_silver",
        name="z_right_shoe",
    )
    z_carriage.visual(
        Box((0.04, 0.17, 0.16)),
        origin=Origin(xyz=(0.076, 0.0, 0.0)),
        material="carriage_silver",
        name="z_bridge",
    )
    z_carriage.visual(
        Box(Y_RAIL_SIZE),
        origin=Origin(xyz=Y_RAIL_LOCAL_CENTER),
        material="rail_steel",
        name="y_rail",
    )
    z_carriage.visual(
        Box((0.04, 0.04, 0.03)),
        origin=Origin(xyz=(0.122, 0.13, -0.039)),
        material="carriage_silver",
        name="y_rail_support",
    )

    y_slide = model.part("y_slide")
    y_slide.visual(
        Box((0.03, 0.08, 0.09)),
        origin=Origin(xyz=(0.041, 0.0, 0.0)),
        material="carriage_silver",
        name="y_front_shoe",
    )
    y_slide.visual(
        Box((0.082, 0.08, 0.021)),
        origin=Origin(xyz=(0.015, 0.0, 0.0345)),
        material="carriage_silver",
        name="y_top_shoe",
    )
    y_slide.visual(
        Box((0.082, 0.08, 0.021)),
        origin=Origin(xyz=(0.015, 0.0, -0.0345)),
        material="carriage_silver",
        name="y_bottom_shoe",
    )
    y_slide.visual(
        Box((0.04, 0.08, 0.03)),
        origin=Origin(xyz=(0.072, 0.07, 0.0)),
        material="carriage_silver",
        name="platform_arm",
    )
    y_slide.visual(
        Box((0.045, 0.02, 0.08)),
        origin=Origin(xyz=(0.072, 0.11, 0.04)),
        material="carriage_silver",
        name="platform_riser",
    )
    y_slide.visual(
        mesh_from_cadquery(_platform_shape(), "end_platform"),
        origin=Origin(xyz=PLATFORM_LOCAL_CENTER),
        material="platform_blue",
        name="end_platform",
    )
    y_slide.visual(
        Box((0.006, 0.07, 0.025)),
        origin=Origin(xyz=(0.122, 0.125, 0.0985)),
        material="platform_blue",
        name="platform_stop",
    )

    model.articulation(
        "frame_to_z_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=z_carriage,
        origin=Origin(xyz=(VERTICAL_RAIL_CENTER[0], 0.0, Z_SLIDE_LOWER_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.20,
            lower=0.0,
            upper=Z_TRAVEL,
        ),
    )
    model.articulation(
        "z_carriage_to_y_slide",
        ArticulationType.PRISMATIC,
        parent=z_carriage,
        child=y_slide,
        origin=Origin(xyz=(Y_RAIL_LOCAL_CENTER[0], Y_STAGE_INNER_CENTER_Y, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.25,
            lower=0.0,
            upper=Y_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    z_carriage = object_model.get_part("z_carriage")
    y_slide = object_model.get_part("y_slide")

    z_joint = object_model.get_articulation("frame_to_z_carriage")
    y_joint = object_model.get_articulation("z_carriage_to_y_slide")

    frame_plate = frame.get_visual("frame_plate")
    vertical_rail = frame.get_visual("vertical_rail")
    z_front_shoe = z_carriage.get_visual("z_front_shoe")
    y_rail = z_carriage.get_visual("y_rail")
    y_front_shoe = y_slide.get_visual("y_front_shoe")
    end_platform = y_slide.get_visual("end_platform")

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
        "all_parts_present",
        all(part is not None for part in (frame, z_carriage, y_slide)),
        "Frame, vertical carriage, and horizontal slide must all exist.",
    )
    ctx.check(
        "z_joint_is_vertical_prismatic",
        z_joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(z_joint.axis) == (0.0, 0.0, 1.0),
        f"Expected a vertical prismatic joint on +Z, got type={z_joint.articulation_type}, axis={z_joint.axis}.",
    )
    ctx.check(
        "y_joint_is_sideways_prismatic",
        y_joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(y_joint.axis) == (0.0, 1.0, 0.0),
        f"Expected a sideways prismatic joint on +Y, got type={y_joint.articulation_type}, axis={y_joint.axis}.",
    )

    frame_plate_aabb = ctx.part_element_world_aabb(frame, elem=frame_plate)
    platform_aabb = ctx.part_element_world_aabb(y_slide, elem=end_platform)
    if frame_plate_aabb is not None:
        frame_width = frame_plate_aabb[1][1] - frame_plate_aabb[0][1]
        frame_height = frame_plate_aabb[1][2] - frame_plate_aabb[0][2]
        ctx.check(
            "frame_plate_proportions",
            0.17 <= frame_width <= 0.19 and 0.45 <= frame_height <= 0.47,
            f"Frame plate size should read as a wall-mounted stage frame, got width={frame_width:.3f} m height={frame_height:.3f} m.",
        )
    else:
        ctx.fail("frame_plate_proportions", "Could not resolve frame plate world AABB.")

    if platform_aabb is not None:
        platform_x = platform_aabb[1][0] - platform_aabb[0][0]
        platform_y = platform_aabb[1][1] - platform_aabb[0][1]
        ctx.check(
            "platform_size_is_small",
            0.09 <= platform_x <= 0.11 and 0.06 <= platform_y <= 0.08,
            f"End platform should stay compact, got {platform_x:.3f} x {platform_y:.3f} m.",
        )
    else:
        ctx.fail("platform_size_is_small", "Could not resolve end platform world AABB.")

    with ctx.pose({z_joint: 0.0, y_joint: 0.0}):
        ctx.expect_contact(
            z_carriage,
            frame,
            elem_a=z_front_shoe,
            elem_b=vertical_rail,
            contact_tol=1e-6,
            name="z_carriage_contacts_vertical_rail_lower",
        )
        ctx.expect_overlap(
            z_carriage,
            frame,
            axes="z",
            elem_a=z_front_shoe,
            elem_b=vertical_rail,
            min_overlap=0.12,
            name="z_carriage_keeps_full_lower_rail_engagement",
        )
        ctx.expect_contact(
            y_slide,
            z_carriage,
            elem_a=y_front_shoe,
            elem_b=y_rail,
            contact_tol=1e-6,
            name="y_slide_contacts_side_rail_inner",
        )
        ctx.expect_overlap(
            y_slide,
            z_carriage,
            axes="y",
            elem_a=y_front_shoe,
            elem_b=y_rail,
            min_overlap=0.079,
            name="y_slide_keeps_inner_rail_engagement",
        )
        ctx.expect_gap(
            y_slide,
            z_carriage,
            axis="z",
            positive_elem=end_platform,
            negative_elem=y_rail,
            min_gap=0.05,
            max_gap=0.07,
            name="platform_sits_above_side_rail",
        )
        ctx.expect_origin_gap(
            z_carriage,
            frame,
            axis="z",
            min_gap=0.11,
            max_gap=0.13,
            name="z_carriage_lower_pose_height",
        )
        ctx.expect_origin_gap(
            y_slide,
            z_carriage,
            axis="y",
            min_gap=0.05,
            max_gap=0.07,
            name="y_slide_inner_pose_offset",
        )

    with ctx.pose({z_joint: Z_TRAVEL, y_joint: 0.0}):
        ctx.expect_contact(
            z_carriage,
            frame,
            elem_a=z_front_shoe,
            elem_b=vertical_rail,
            contact_tol=1e-6,
            name="z_carriage_contacts_vertical_rail_upper",
        )
        ctx.expect_overlap(
            z_carriage,
            frame,
            axes="z",
            elem_a=z_front_shoe,
            elem_b=vertical_rail,
            min_overlap=0.12,
            name="z_carriage_keeps_upper_rail_engagement",
        )
        ctx.expect_origin_gap(
            z_carriage,
            frame,
            axis="z",
            min_gap=0.27,
            max_gap=0.29,
            name="z_carriage_upper_pose_height",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="upper_z_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="upper_z_pose_no_floating")

    with ctx.pose({z_joint: 0.08, y_joint: Y_TRAVEL}):
        ctx.expect_contact(
            y_slide,
            z_carriage,
            elem_a=y_front_shoe,
            elem_b=y_rail,
            contact_tol=1e-6,
            name="y_slide_contacts_side_rail_outer",
        )
        ctx.expect_overlap(
            y_slide,
            z_carriage,
            axes="y",
            elem_a=y_front_shoe,
            elem_b=y_rail,
            min_overlap=0.079,
            name="y_slide_keeps_outer_rail_engagement",
        )
        ctx.expect_origin_gap(
            y_slide,
            z_carriage,
            axis="y",
            min_gap=0.19,
            max_gap=0.21,
            name="y_slide_outer_pose_offset",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="outer_y_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="outer_y_pose_no_floating")

    with ctx.pose({z_joint: Z_TRAVEL, y_joint: Y_TRAVEL}):
        ctx.fail_if_parts_overlap_in_current_pose(name="combined_extreme_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="combined_extreme_pose_no_floating")

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

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
