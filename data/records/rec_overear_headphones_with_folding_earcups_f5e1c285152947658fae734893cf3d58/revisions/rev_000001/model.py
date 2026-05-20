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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="travel_over_ear_headphones")

    matte_black = model.material("matte_black", rgba=(0.10, 0.10, 0.11, 1.0))
    satin_graphite = model.material("satin_graphite", rgba=(0.22, 0.23, 0.25, 1.0))
    anodized_aluminum = model.material("anodized_aluminum", rgba=(0.63, 0.65, 0.69, 1.0))
    cushion_gray = model.material("cushion_gray", rgba=(0.24, 0.25, 0.27, 1.0))
    ear_pad = model.material("ear_pad", rgba=(0.14, 0.14, 0.15, 1.0))
    accent = model.material("accent", rgba=(0.74, 0.77, 0.81, 1.0))

    headband_path = [
        (0.0, -0.081, 0.109),
        (0.0, -0.060, 0.142),
        (0.0, -0.028, 0.160),
        (0.0, 0.000, 0.167),
        (0.0, 0.028, 0.160),
        (0.0, 0.060, 0.142),
        (0.0, 0.081, 0.109),
    ]
    headband_outer_mesh = mesh_from_geometry(
        sweep_profile_along_spline(
            headband_path,
            profile=rounded_rect_profile(0.030, 0.010, radius=0.004, corner_segments=8),
            samples_per_segment=18,
            cap_profile=True,
        ),
        "headband_outer",
    )
    headband_cushion_mesh = mesh_from_geometry(
        sweep_profile_along_spline(
            [
                (0.0, -0.053, 0.122),
                (0.0, -0.030, 0.136),
                (0.0, 0.000, 0.143),
                (0.0, 0.030, 0.136),
                (0.0, 0.053, 0.122),
            ],
            profile=rounded_rect_profile(0.040, 0.007, radius=0.003, corner_segments=8),
            samples_per_segment=16,
            cap_profile=True,
        ),
        "headband_cushion",
    )

    sleeve_profile = [
        (-0.010, -0.005),
        (0.010, -0.005),
        (0.010, 0.005),
        (0.008, 0.005),
        (0.008, -0.003),
        (-0.008, -0.003),
        (-0.008, 0.005),
        (-0.010, 0.005),
    ]
    sleeve_channel_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(sleeve_profile, 0.052, cap=True, closed=True),
        "slider_sleeve_channel",
    )

    cup_outer_profile = rounded_rect_profile(0.078, 0.094, radius=0.018, corner_segments=10)
    cup_inner_profile = rounded_rect_profile(0.048, 0.062, radius=0.012, corner_segments=10)
    pad_outer_profile = rounded_rect_profile(0.084, 0.102, radius=0.022, corner_segments=10)
    pad_inner_profile = rounded_rect_profile(0.044, 0.060, radius=0.013, corner_segments=10)
    cup_ring_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            cup_outer_profile,
            [cup_inner_profile],
            0.026,
            cap=True,
            center=True,
            closed=True,
        ),
        "earcup_shell_ring",
    )
    cup_back_mesh = mesh_from_geometry(
        ExtrudeGeometry.centered(cup_outer_profile, 0.004, cap=True, closed=True),
        "earcup_back_cap",
    )
    cup_pad_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            pad_outer_profile,
            [pad_inner_profile],
            0.016,
            cap=True,
            center=True,
            closed=True,
        ),
        "ear_pad_ring",
    )
    cup_badge_mesh = mesh_from_geometry(
        ExtrudeGeometry.centered(rounded_rect_profile(0.036, 0.048, radius=0.010, corner_segments=8), 0.003),
        "earcup_badge",
    )
    headband = model.part("headband_frame")
    headband.visual(headband_outer_mesh, material=satin_graphite, name="headband_outer")
    headband.visual(
        headband_cushion_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=cushion_gray,
        name="headband_cushion",
    )
    headband.visual(
        Box((0.004, 0.006, 0.052)),
        origin=Origin(xyz=(-0.008, -0.081, 0.089)),
        material=matte_black,
        name="left_sleeve_left_rail",
    )
    headband.visual(
        Box((0.004, 0.006, 0.052)),
        origin=Origin(xyz=(0.008, -0.081, 0.089)),
        material=matte_black,
        name="left_sleeve_right_rail",
    )
    headband.visual(
        Box((0.004, 0.006, 0.052)),
        origin=Origin(xyz=(-0.008, 0.081, 0.089)),
        material=matte_black,
        name="right_sleeve_left_rail",
    )
    headband.visual(
        Box((0.004, 0.006, 0.052)),
        origin=Origin(xyz=(0.008, 0.081, 0.089)),
        material=matte_black,
        name="right_sleeve_right_rail",
    )
    headband.visual(
        Box((0.018, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, -0.081, 0.120)),
        material=satin_graphite,
        name="left_transition_block",
    )
    headband.visual(
        Box((0.018, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, 0.081, 0.120)),
        material=satin_graphite,
        name="right_transition_block",
    )
    headband.visual(
        Box((0.022, 0.100, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.126)),
        material=cushion_gray,
        name="cushion_bridge",
    )
    headband.visual(
        Box((0.022, 0.016, 0.020)),
        origin=Origin(xyz=(0.0, -0.058, 0.124)),
        material=cushion_gray,
        name="left_cushion_connector",
    )
    headband.visual(
        Box((0.022, 0.016, 0.020)),
        origin=Origin(xyz=(0.0, 0.058, 0.124)),
        material=cushion_gray,
        name="right_cushion_connector",
    )
    headband.inertial = Inertial.from_geometry(
        Box((0.140, 0.190, 0.120)),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
    )

    def add_slider_arm(name: str) -> None:
        arm = model.part(name)
        arm.visual(
            Box((0.012, 0.006, 0.006)),
            origin=Origin(xyz=(0.0, 0.0, -0.008)),
            material=anodized_aluminum,
            name="retention_head",
        )
        arm.visual(
            Box((0.010, 0.004, 0.056)),
            origin=Origin(xyz=(0.0, 0.0, -0.034)),
            material=anodized_aluminum,
            name="slider_bar",
        )
        arm.visual(
            Box((0.016, 0.010, 0.014)),
            origin=Origin(xyz=(0.0, 0.0, -0.068)),
            material=matte_black,
            name="hinge_block",
        )
        arm.inertial = Inertial.from_geometry(
            Box((0.018, 0.010, 0.082)),
            mass=0.035,
            origin=Origin(xyz=(0.0, 0.0, -0.041)),
        )

    add_slider_arm("left_slider_arm")
    add_slider_arm("right_slider_arm")

    def add_yoke(name: str) -> None:
        yoke = model.part(name)
        yoke.visual(
            Box((0.022, 0.010, 0.006)),
            origin=Origin(xyz=(0.0, 0.0, -0.003)),
            material=matte_black,
            name="fold_cap",
        )
        yoke.visual(
            Box((0.008, 0.008, 0.014)),
            origin=Origin(xyz=(0.0, 0.0, -0.010)),
            material=matte_black,
            name="hanger_stem",
        )
        yoke.visual(
            Box((0.082, 0.008, 0.006)),
            origin=Origin(xyz=(0.0, 0.0, -0.006)),
            material=matte_black,
            name="yoke_bridge",
        )
        yoke.visual(
            Box((0.008, 0.006, 0.052)),
            origin=Origin(xyz=(0.041, 0.0, -0.035)),
            material=matte_black,
            name="front_arm",
        )
        yoke.visual(
            Box((0.008, 0.006, 0.052)),
            origin=Origin(xyz=(-0.041, 0.0, -0.035)),
            material=matte_black,
            name="rear_arm",
        )
        yoke.inertial = Inertial.from_geometry(
            Box((0.082, 0.014, 0.068)),
            mass=0.045,
            origin=Origin(xyz=(0.0, 0.0, -0.029)),
        )

    add_yoke("left_yoke")
    add_yoke("right_yoke")

    def add_cup(name: str) -> None:
        cup = model.part(name)
        cup.visual(
            Box((0.074, 0.006, 0.008)),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=matte_black,
            name="pivot_bar",
        )
        cup.visual(
            Box((0.012, 0.006, 0.020)),
            origin=Origin(xyz=(0.0, 0.0, -0.014)),
            material=matte_black,
            name="pivot_stem",
        )
        cup.visual(
            cup_ring_mesh,
            origin=Origin(xyz=(0.0, 0.0, -0.070), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=matte_black,
            name="outer_shell_ring",
        )
        cup.visual(
            cup_back_mesh,
            origin=Origin(xyz=(0.0, -0.015, -0.070), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=satin_graphite,
            name="outer_back_cap",
        )
        cup.visual(
            cup_badge_mesh,
            origin=Origin(xyz=(0.0, -0.0185, -0.070), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=accent,
            name="outer_badge",
        )
        cup.visual(
            cup_pad_mesh,
            origin=Origin(xyz=(0.0, 0.021, -0.070), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=ear_pad,
            name="ear_pad_ring",
        )
        cup.inertial = Inertial.from_geometry(
            Box((0.084, 0.050, 0.116)),
            mass=0.11,
            origin=Origin(xyz=(0.0, 0.0, -0.056)),
        )

    add_cup("left_earcup")
    add_cup("right_earcup")

    model.articulation(
        "left_arm_slide",
        ArticulationType.PRISMATIC,
        parent=headband,
        child="left_slider_arm",
        origin=Origin(xyz=(0.0, -0.081, 0.115)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.10,
            lower=0.0,
            upper=0.028,
        ),
    )
    model.articulation(
        "right_arm_slide",
        ArticulationType.PRISMATIC,
        parent=headband,
        child="right_slider_arm",
        origin=Origin(xyz=(0.0, 0.081, 0.115), rpy=(0.0, 0.0, math.pi)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.10,
            lower=0.0,
            upper=0.028,
        ),
    )
    model.articulation(
        "left_fold_hinge",
        ArticulationType.REVOLUTE,
        parent="left_slider_arm",
        child="left_yoke",
        origin=Origin(xyz=(0.0, 0.0, -0.075)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=0.0,
            upper=1.25,
        ),
    )
    model.articulation(
        "right_fold_hinge",
        ArticulationType.REVOLUTE,
        parent="right_slider_arm",
        child="right_yoke",
        origin=Origin(xyz=(0.0, 0.0, -0.075)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=0.0,
            upper=1.25,
        ),
    )
    model.articulation(
        "left_cup_swivel",
        ArticulationType.REVOLUTE,
        parent="left_yoke",
        child="left_earcup",
        origin=Origin(xyz=(0.0, 0.0, -0.022)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=2.0,
            lower=-0.55,
            upper=0.55,
        ),
    )
    model.articulation(
        "right_cup_swivel",
        ArticulationType.REVOLUTE,
        parent="right_yoke",
        child="right_earcup",
        origin=Origin(xyz=(0.0, 0.0, -0.022)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=2.0,
            lower=-0.55,
            upper=0.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    headband = object_model.get_part("headband_frame")
    left_arm = object_model.get_part("left_slider_arm")
    right_arm = object_model.get_part("right_slider_arm")
    left_yoke = object_model.get_part("left_yoke")
    right_yoke = object_model.get_part("right_yoke")
    left_cup = object_model.get_part("left_earcup")
    right_cup = object_model.get_part("right_earcup")

    left_slide = object_model.get_articulation("left_arm_slide")
    right_slide = object_model.get_articulation("right_arm_slide")
    left_fold = object_model.get_articulation("left_fold_hinge")
    right_fold = object_model.get_articulation("right_fold_hinge")
    left_swivel = object_model.get_articulation("left_cup_swivel")
    right_swivel = object_model.get_articulation("right_cup_swivel")

    ctx.expect_contact(
        left_arm,
        headband,
        elem_a="retention_head",
        elem_b="left_sleeve_left_rail",
        name="left_slider_contacts_left_rail",
    )
    ctx.expect_contact(
        right_arm,
        headband,
        elem_a="retention_head",
        elem_b="right_sleeve_left_rail",
        name="right_slider_contacts_left_rail",
    )
    ctx.expect_contact(
        left_yoke,
        left_arm,
        elem_a="fold_cap",
        elem_b="hinge_block",
        name="left_yoke_contacts_hinge_block",
    )
    ctx.expect_contact(
        right_yoke,
        right_arm,
        elem_a="fold_cap",
        elem_b="hinge_block",
        name="right_yoke_contacts_hinge_block",
    )
    ctx.expect_contact(
        left_cup,
        left_yoke,
        elem_a="pivot_bar",
        elem_b="front_arm",
        name="left_cup_seated_in_yoke",
    )
    ctx.expect_contact(
        right_cup,
        right_yoke,
        elem_a="pivot_bar",
        elem_b="front_arm",
        name="right_cup_seated_in_yoke",
    )

    left_slide_upper = left_slide.motion_limits.upper
    right_slide_upper = right_slide.motion_limits.upper
    assert left_slide_upper is not None
    assert right_slide_upper is not None

    left_left_rail_aabb = ctx.part_element_world_aabb(headband, elem="left_sleeve_left_rail")
    left_right_rail_aabb = ctx.part_element_world_aabb(headband, elem="left_sleeve_right_rail")
    right_left_rail_aabb = ctx.part_element_world_aabb(headband, elem="right_sleeve_left_rail")
    right_right_rail_aabb = ctx.part_element_world_aabb(headband, elem="right_sleeve_right_rail")
    left_transition_aabb = ctx.part_element_world_aabb(headband, elem="left_transition_block")
    right_transition_aabb = ctx.part_element_world_aabb(headband, elem="right_transition_block")
    assert left_left_rail_aabb is not None
    assert left_right_rail_aabb is not None
    assert right_left_rail_aabb is not None
    assert right_right_rail_aabb is not None
    assert left_transition_aabb is not None
    assert right_transition_aabb is not None

    left_arm_rest = ctx.part_world_position(left_arm)
    right_arm_rest = ctx.part_world_position(right_arm)
    assert left_arm_rest is not None
    assert right_arm_rest is not None

    with ctx.pose({left_slide: left_slide_upper, right_slide: right_slide_upper}):
        left_arm_extended = ctx.part_world_position(left_arm)
        right_arm_extended = ctx.part_world_position(right_arm)
        left_head_aabb = ctx.part_element_world_aabb(left_arm, elem="retention_head")
        right_head_aabb = ctx.part_element_world_aabb(right_arm, elem="retention_head")

        assert left_arm_extended is not None
        assert right_arm_extended is not None
        assert left_head_aabb is not None
        assert right_head_aabb is not None

        ctx.check(
            "left_slider_extends_straight_down",
            left_arm_extended[2] < left_arm_rest[2] - 0.020
            and abs(left_arm_extended[1] - left_arm_rest[1]) < 1e-6
            and abs(left_arm_extended[0] - left_arm_rest[0]) < 1e-6,
            details=f"rest={left_arm_rest}, extended={left_arm_extended}",
        )
        ctx.check(
            "right_slider_extends_straight_down",
            right_arm_extended[2] < right_arm_rest[2] - 0.020
            and abs(right_arm_extended[1] - right_arm_rest[1]) < 1e-6
            and abs(right_arm_extended[0] - right_arm_rest[0]) < 1e-6,
            details=f"rest={right_arm_rest}, extended={right_arm_extended}",
        )
        ctx.check(
            "left_slider_retention_head_stays_captured",
            left_head_aabb[0][0] >= left_left_rail_aabb[1][0] - 2e-4
            and left_head_aabb[1][0] <= left_right_rail_aabb[0][0] + 2e-4
            and left_head_aabb[0][1] >= left_left_rail_aabb[0][1] - 0.0015
            and left_head_aabb[1][1] <= left_left_rail_aabb[1][1] + 0.0015
            and left_head_aabb[0][2] >= left_left_rail_aabb[0][2] - 1e-6
            and left_head_aabb[1][2] <= left_transition_aabb[1][2] + 1e-6,
            details=(
                f"head={left_head_aabb}, left_rail={left_left_rail_aabb}, "
                f"right_rail={left_right_rail_aabb}, transition={left_transition_aabb}"
            ),
        )
        ctx.check(
            "right_slider_retention_head_stays_captured",
            right_head_aabb[0][0] >= right_left_rail_aabb[1][0] - 2e-4
            and right_head_aabb[1][0] <= right_right_rail_aabb[0][0] + 2e-4
            and right_head_aabb[0][1] >= right_left_rail_aabb[0][1] - 0.0015
            and right_head_aabb[1][1] <= right_left_rail_aabb[1][1] + 0.0015
            and right_head_aabb[0][2] >= right_left_rail_aabb[0][2] - 1e-6
            and right_head_aabb[1][2] <= right_transition_aabb[1][2] + 1e-6,
            details=(
                f"head={right_head_aabb}, left_rail={right_left_rail_aabb}, "
                f"right_rail={right_right_rail_aabb}, transition={right_transition_aabb}"
            ),
        )

    left_shell_rest = ctx.part_element_world_aabb(left_cup, elem="outer_shell_ring")
    right_shell_rest = ctx.part_element_world_aabb(right_cup, elem="outer_shell_ring")
    assert left_shell_rest is not None
    assert right_shell_rest is not None

    with ctx.pose({left_fold: 1.10, right_fold: 1.10}):
        left_cup_folded = ctx.part_element_world_aabb(left_cup, elem="outer_shell_ring")
        right_cup_folded = ctx.part_element_world_aabb(right_cup, elem="outer_shell_ring")
        assert left_cup_folded is not None
        assert right_cup_folded is not None

        left_rest_center = (
            (left_shell_rest[0][0] + left_shell_rest[1][0]) / 2.0,
            (left_shell_rest[0][1] + left_shell_rest[1][1]) / 2.0,
            (left_shell_rest[0][2] + left_shell_rest[1][2]) / 2.0,
        )
        right_rest_center = (
            (right_shell_rest[0][0] + right_shell_rest[1][0]) / 2.0,
            (right_shell_rest[0][1] + right_shell_rest[1][1]) / 2.0,
            (right_shell_rest[0][2] + right_shell_rest[1][2]) / 2.0,
        )
        left_fold_center = (
            (left_cup_folded[0][0] + left_cup_folded[1][0]) / 2.0,
            (left_cup_folded[0][1] + left_cup_folded[1][1]) / 2.0,
            (left_cup_folded[0][2] + left_cup_folded[1][2]) / 2.0,
        )
        right_fold_center = (
            (right_cup_folded[0][0] + right_cup_folded[1][0]) / 2.0,
            (right_cup_folded[0][1] + right_cup_folded[1][1]) / 2.0,
            (right_cup_folded[0][2] + right_cup_folded[1][2]) / 2.0,
        )

        ctx.check(
            "left_fold_hinge_tucks_cup_inward",
            left_fold_center[1] > left_rest_center[1] + 0.030 and left_fold_center[2] > left_rest_center[2] + 0.020,
            details=f"rest={left_rest_center}, folded={left_fold_center}",
        )
        ctx.check(
            "right_fold_hinge_tucks_cup_inward",
            right_fold_center[1] < right_rest_center[1] - 0.030 and right_fold_center[2] > right_rest_center[2] + 0.020,
            details=f"rest={right_rest_center}, folded={right_fold_center}",
        )

    with ctx.pose({left_swivel: 0.50, right_swivel: -0.50}):
        left_shell_swiveled = ctx.part_element_world_aabb(left_cup, elem="outer_shell_ring")
        right_shell_swiveled = ctx.part_element_world_aabb(right_cup, elem="outer_shell_ring")
        assert left_shell_swiveled is not None
        assert right_shell_swiveled is not None

        left_rest_span_y = left_shell_rest[1][1] - left_shell_rest[0][1]
        left_swiveled_span_y = left_shell_swiveled[1][1] - left_shell_swiveled[0][1]
        right_rest_span_y = right_shell_rest[1][1] - right_shell_rest[0][1]
        right_swiveled_span_y = right_shell_swiveled[1][1] - right_shell_swiveled[0][1]

        ctx.check(
            "left_cup_swivel_changes_cup_yaw",
            left_swiveled_span_y > left_rest_span_y + 0.015,
            details=f"rest_span_y={left_rest_span_y}, swiveled_span_y={left_swiveled_span_y}",
        )
        ctx.check(
            "right_cup_swivel_changes_cup_yaw",
            right_swiveled_span_y > right_rest_span_y + 0.015,
            details=f"rest_span_y={right_rest_span_y}, swiveled_span_y={right_swiveled_span_y}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
