from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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
)

BODY_WIDTH = 0.24
BODY_RADIUS = BODY_WIDTH * 0.5
BODY_STRAIGHT_DEPTH = 0.10
BODY_CAP_OFFSET = BODY_STRAIGHT_DEPTH * 0.5
BODY_SHELL_HEIGHT = 0.66
BODY_SHELL_CENTER_Z = 0.37

BASE_WIDTH = 0.28
BASE_RADIUS = BASE_WIDTH * 0.5
BASE_STRAIGHT_DEPTH = 0.08
BASE_CAP_OFFSET = BASE_STRAIGHT_DEPTH * 0.5

HINGE_X = 0.124
HINGE_Y = 0.110
HINGE_Z = 0.40
HINGE_RADIUS = 0.008
BODY_KNUCKLE_LENGTH = 0.09
COVER_KNUCKLE_LENGTH = 0.30
LOWER_KNUCKLE_Z = 0.205
UPPER_KNUCKLE_Z = 0.595


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="oval_floor_air_purifier")

    shell_white = model.material("shell_white", rgba=(0.94, 0.95, 0.93, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.74, 0.76, 0.77, 1.0))
    graphite = model.material("graphite", rgba=(0.17, 0.18, 0.20, 1.0))
    filter_black = model.material("filter_black", rgba=(0.11, 0.12, 0.13, 1.0))

    body = model.part("purifier_body")
    body.visual(
        Box((BODY_WIDTH, BODY_STRAIGHT_DEPTH, BODY_SHELL_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BODY_SHELL_CENTER_Z)),
        material=shell_white,
        name="body_core",
    )
    body.visual(
        Cylinder(radius=BODY_RADIUS, length=BODY_SHELL_HEIGHT),
        origin=Origin(xyz=(0.0, BODY_CAP_OFFSET, BODY_SHELL_CENTER_Z)),
        material=shell_white,
        name="body_front_curve",
    )
    body.visual(
        Cylinder(radius=BODY_RADIUS, length=BODY_SHELL_HEIGHT),
        origin=Origin(xyz=(0.0, -BODY_CAP_OFFSET, BODY_SHELL_CENTER_Z)),
        material=shell_white,
        name="body_rear_curve",
    )
    body.visual(
        Box((BASE_WIDTH, BASE_STRAIGHT_DEPTH, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=trim_gray,
        name="base_core",
    )
    body.visual(
        Cylinder(radius=BASE_RADIUS, length=0.04),
        origin=Origin(xyz=(0.0, BASE_CAP_OFFSET, 0.02)),
        material=trim_gray,
        name="base_front_curve",
    )
    body.visual(
        Cylinder(radius=BASE_RADIUS, length=0.04),
        origin=Origin(xyz=(0.0, -BASE_CAP_OFFSET, 0.02)),
        material=trim_gray,
        name="base_rear_curve",
    )
    body.visual(
        Box((0.016, 0.136, 0.06)),
        origin=Origin(xyz=(-0.068, 0.0, 0.73)),
        material=graphite,
        name="top_outlet_left_wall",
    )
    body.visual(
        Box((0.016, 0.136, 0.06)),
        origin=Origin(xyz=(0.068, 0.0, 0.73)),
        material=graphite,
        name="top_outlet_right_wall",
    )
    body.visual(
        Box((0.152, 0.016, 0.06)),
        origin=Origin(xyz=(0.0, 0.068, 0.73)),
        material=graphite,
        name="top_outlet_front_wall",
    )
    body.visual(
        Box((0.152, 0.016, 0.06)),
        origin=Origin(xyz=(0.0, -0.068, 0.73)),
        material=graphite,
        name="top_outlet_rear_wall",
    )
    for index, y_pos in enumerate((-0.024, -0.008, 0.008, 0.024), start=1):
        body.visual(
            Box((0.12, 0.006, 0.010)),
            origin=Origin(xyz=(0.0, y_pos, 0.765)),
            material=trim_gray,
            name=f"outlet_slat_{index}",
        )

    body.visual(
        Box((0.010, 0.226, 0.028)),
        origin=Origin(xyz=(0.096, -0.007, 0.124)),
        material=trim_gray,
        name="opening_lower_rail",
    )
    body.visual(
        Box((0.010, 0.226, 0.028)),
        origin=Origin(xyz=(0.096, -0.007, 0.676)),
        material=trim_gray,
        name="opening_upper_rail",
    )
    body.visual(
        Box((0.010, 0.030, 0.56)),
        origin=Origin(xyz=(0.096, -0.095, 0.40)),
        material=trim_gray,
        name="opening_rear_stile",
    )
    body.visual(
        Cylinder(radius=HINGE_RADIUS, length=BODY_KNUCKLE_LENGTH),
        origin=Origin(xyz=(HINGE_X, HINGE_Y, LOWER_KNUCKLE_Z)),
        material=graphite,
        name="body_knuckle_lower",
    )
    body.visual(
        Box((0.020, 0.060, BODY_KNUCKLE_LENGTH)),
        origin=Origin(xyz=(0.108, 0.080, LOWER_KNUCKLE_Z)),
        material=trim_gray,
        name="hinge_support_lower",
    )
    body.visual(
        Cylinder(radius=HINGE_RADIUS, length=BODY_KNUCKLE_LENGTH),
        origin=Origin(xyz=(HINGE_X, HINGE_Y, UPPER_KNUCKLE_Z)),
        material=graphite,
        name="body_knuckle_upper",
    )
    body.visual(
        Box((0.020, 0.060, BODY_KNUCKLE_LENGTH)),
        origin=Origin(xyz=(0.108, 0.080, UPPER_KNUCKLE_Z)),
        material=trim_gray,
        name="hinge_support_upper",
    )
    body.visual(
        Box((0.018, 0.212, 0.56)),
        origin=Origin(xyz=(0.091, -0.007, 0.40)),
        material=trim_gray,
        name="filter_frame",
    )
    body.visual(
        Box((0.010, 0.190, 0.52)),
        origin=Origin(xyz=(0.095, -0.007, 0.40)),
        material=filter_black,
        name="filter_media",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.30, 0.36, 0.78)),
        mass=15.0,
        origin=Origin(xyz=(0.0, 0.0, 0.39)),
    )

    cover = model.part("filter_cover")
    cover.visual(
        Cylinder(radius=HINGE_RADIUS, length=COVER_KNUCKLE_LENGTH),
        material=graphite,
        name="cover_knuckle",
    )
    cover.visual(
        Box((0.008, 0.014, 0.30)),
        origin=Origin(xyz=(0.000, -0.007, 0.0)),
        material=shell_white,
        name="cover_hinge_leaf",
    )
    cover.visual(
        Box((0.010, 0.206, 0.58)),
        origin=Origin(xyz=(0.001, -0.117, 0.0)),
        material=shell_white,
        name="cover_panel",
    )
    cover.visual(
        Box((0.004, 0.156, 0.48)),
        origin=Origin(xyz=(0.003, -0.118, 0.0)),
        material=trim_gray,
        name="cover_grille_inset",
    )
    for index, y_pos in enumerate((-0.178, -0.142, -0.106, -0.070, -0.034), start=1):
        cover.visual(
            Box((0.004, 0.014, 0.44)),
            origin=Origin(xyz=(0.004, y_pos, 0.0)),
            material=shell_white,
            name=f"cover_rib_{index}",
        )
    cover.visual(
        Cylinder(radius=0.008, length=0.14),
        origin=Origin(xyz=(0.010, -0.182, 0.0)),
        material=graphite,
        name="finger_pull",
    )
    cover.inertial = Inertial.from_geometry(
        Box((0.05, 0.24, 0.60)),
        mass=1.0,
        origin=Origin(xyz=(0.0, -0.11, 0.0)),
    )

    model.articulation(
        "cover_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cover,
        origin=Origin(xyz=(HINGE_X, HINGE_Y, HINGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.8, lower=0.0, upper=1.18),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("purifier_body")
    cover = object_model.get_part("filter_cover")
    cover_hinge = object_model.get_articulation("cover_hinge")

    body_core = body.get_visual("body_core")
    filter_frame = body.get_visual("filter_frame")
    body_knuckle_lower = body.get_visual("body_knuckle_lower")
    body_knuckle_upper = body.get_visual("body_knuckle_upper")
    cover_panel = cover.get_visual("cover_panel")
    cover_knuckle = cover.get_visual("cover_knuckle")
    finger_pull = cover.get_visual("finger_pull")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=24,
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
        ignore_adjacent=False,
        ignore_fixed=True,
    )

    ctx.check(
        "cover_hinge_vertical_axis",
        abs(cover_hinge.axis[0]) < 1e-9 and abs(cover_hinge.axis[1]) < 1e-9 and abs(cover_hinge.axis[2] - 1.0) < 1e-9,
        f"hinge axis was {cover_hinge.axis}",
    )

    body_aabb = ctx.part_world_aabb(body)
    if body_aabb is None:
        ctx.fail("body_aabb_present", "Purifier body did not produce a world-space AABB.")
    else:
        body_min, body_max = body_aabb
        body_width = body_max[0] - body_min[0]
        body_depth = body_max[1] - body_min[1]
        body_height = body_max[2] - body_min[2]
        ctx.check(
            "body_height_realistic",
            0.74 <= body_height <= 0.82,
            f"Expected a floor-standing purifier height near 0.78 m, got {body_height:.3f} m.",
        )
        ctx.check(
            "body_oval_planform",
            body_depth > body_width and body_width >= 0.24 and body_depth >= 0.34,
            f"Expected an oval/stadium footprint with depth greater than width, got width={body_width:.3f}, depth={body_depth:.3f}.",
        )

    with ctx.pose({cover_hinge: 0.0}):
        ctx.expect_overlap(
            cover,
            body,
            axes="yz",
            elem_a=cover_panel,
            elem_b=filter_frame,
            min_overlap=0.18,
            name="cover_aligned_over_filter_opening",
        )
        ctx.expect_gap(
            cover,
            body,
            axis="x",
            positive_elem=cover_panel,
            negative_elem=body_core,
            min_gap=0.0,
            max_gap=0.002,
            name="cover_flush_with_body_side",
        )
        ctx.expect_gap(
            cover,
            body,
            axis="x",
            positive_elem=cover_panel,
            negative_elem=filter_frame,
            min_gap=0.018,
            max_gap=0.032,
            name="cover_stands_off_from_inner_filter_frame",
        )
        ctx.expect_contact(
            cover,
            body,
            elem_a=cover_knuckle,
            elem_b=body_knuckle_lower,
            name="lower_knuckle_contact_closed",
        )
        ctx.expect_contact(
            cover,
            body,
            elem_a=cover_knuckle,
            elem_b=body_knuckle_upper,
            name="upper_knuckle_contact_closed",
        )

    limits = cover_hinge.motion_limits
    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({cover_hinge: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="cover_hinge_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="cover_hinge_lower_no_floating")
        with ctx.pose({cover_hinge: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="cover_hinge_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="cover_hinge_upper_no_floating")
            ctx.expect_gap(
                cover,
                body,
                axis="x",
                positive_elem=finger_pull,
                negative_elem=body_core,
                min_gap=0.045,
                name="opened_cover_swings_clear_of_body",
            )
            ctx.expect_contact(
                cover,
                body,
                elem_a=cover_knuckle,
                elem_b=body_knuckle_lower,
                name="lower_knuckle_contact_open",
            )
            ctx.expect_contact(
                cover,
                body,
                elem_a=cover_knuckle,
                elem_b=body_knuckle_upper,
                name="upper_knuckle_contact_open",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
