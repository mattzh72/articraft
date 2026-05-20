from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


SPINE_WIDTH = 0.028
GUIDE_LENGTH = 0.078
JAW_CLOSED_X = 0.030
JAW_STROKE = 0.040


def _add_box_visual(part, size, xyz, material, name) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def _build_jaw(part, *, inward_sign: float, material) -> None:
    _add_box_visual(part, (0.028, 0.026, 0.062), (-0.018 * inward_sign, -0.006, 0.0), material, "carriage")
    _add_box_visual(part, (0.030, 0.014, 0.018), (-0.020 * inward_sign, -0.022, 0.022), material, "upper_slider")
    _add_box_visual(part, (0.030, 0.014, 0.018), (-0.020 * inward_sign, -0.022, -0.022), material, "lower_slider")
    _add_box_visual(part, (0.024, 0.052, 0.052), (-0.004 * inward_sign, 0.020, 0.0), material, "bridge")
    _add_box_visual(part, (0.014, 0.028, 0.050), (0.012 * inward_sign, 0.052, 0.0), material, "palm")
    _add_box_visual(part, (0.008, 0.100, 0.012), (0.020 * inward_sign, 0.111, 0.022), material, "upper_finger")
    _add_box_visual(part, (0.008, 0.100, 0.012), (0.020 * inward_sign, 0.111, -0.022), material, "lower_finger")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fork_finger_parallel_gripper")

    housing_material = model.material("housing_dark", rgba=(0.20, 0.21, 0.23, 1.0))
    guide_material = model.material("guide_steel", rgba=(0.76, 0.78, 0.80, 1.0))
    jaw_material = model.material("jaw_steel", rgba=(0.70, 0.72, 0.74, 1.0))

    spine = model.part("spine")
    _add_box_visual(spine, (SPINE_WIDTH, 0.062, 0.148), (0.0, -0.010, 0.0), housing_material, "core")
    _add_box_visual(spine, (0.052, 0.034, 0.092), (0.0, -0.046, 0.0), housing_material, "rear_pack")
    _add_box_visual(spine, (0.016, 0.034, 0.104), (0.0, 0.034, 0.0), housing_material, "front_spine")
    _add_box_visual(spine, (GUIDE_LENGTH, 0.008, 0.018), (0.050, -0.033, 0.022), guide_material, "right_upper_guide")
    _add_box_visual(spine, (GUIDE_LENGTH, 0.008, 0.018), (0.050, -0.033, -0.022), guide_material, "right_lower_guide")
    _add_box_visual(spine, (GUIDE_LENGTH, 0.008, 0.018), (-0.050, -0.033, 0.022), guide_material, "left_upper_guide")
    _add_box_visual(spine, (GUIDE_LENGTH, 0.008, 0.018), (-0.050, -0.033, -0.022), guide_material, "left_lower_guide")

    left_jaw = model.part("left_jaw")
    _build_jaw(left_jaw, inward_sign=1.0, material=jaw_material)

    right_jaw = model.part("right_jaw")
    _build_jaw(right_jaw, inward_sign=-1.0, material=jaw_material)

    model.articulation(
        "spine_to_left_jaw",
        ArticulationType.PRISMATIC,
        parent=spine,
        child=left_jaw,
        origin=Origin(xyz=(-JAW_CLOSED_X, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=150.0,
            velocity=0.20,
            lower=0.0,
            upper=JAW_STROKE,
        ),
    )
    model.articulation(
        "spine_to_right_jaw",
        ArticulationType.PRISMATIC,
        parent=spine,
        child=right_jaw,
        origin=Origin(xyz=(JAW_CLOSED_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=150.0,
            velocity=0.20,
            lower=0.0,
            upper=JAW_STROKE,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    spine = object_model.get_part("spine")
    left_jaw = object_model.get_part("left_jaw")
    right_jaw = object_model.get_part("right_jaw")
    left_slide = object_model.get_articulation("spine_to_left_jaw")
    right_slide = object_model.get_articulation("spine_to_right_jaw")

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
        "left_prismatic_axis_points_outward",
        left_slide.joint_type == ArticulationType.PRISMATIC and tuple(left_slide.axis) == (-1.0, 0.0, 0.0),
        details=f"expected left jaw prismatic axis (-1, 0, 0), got {left_slide.axis!r}",
    )
    ctx.check(
        "right_prismatic_axis_points_outward",
        right_slide.joint_type == ArticulationType.PRISMATIC and tuple(right_slide.axis) == (1.0, 0.0, 0.0),
        details=f"expected right jaw prismatic axis (1, 0, 0), got {right_slide.axis!r}",
    )

    with ctx.pose({left_slide: 0.0, right_slide: 0.0}):
        ctx.expect_contact(
            left_jaw,
            spine,
            contact_tol=0.001,
            name="left_jaw_supported_on_guides_closed",
        )
        ctx.expect_contact(
            right_jaw,
            spine,
            contact_tol=0.001,
            name="right_jaw_supported_on_guides_closed",
        )
        ctx.expect_gap(
            right_jaw,
            left_jaw,
            axis="x",
            min_gap=0.010,
            max_gap=0.016,
            name="closed_finger_gap_is_narrow",
        )
        ctx.expect_overlap(
            left_jaw,
            right_jaw,
            axes="yz",
            min_overlap=0.020,
            name="jaws_face_each_other_in_closed_pose",
        )

    with ctx.pose({left_slide: 0.035, right_slide: 0.035}):
        ctx.expect_contact(
            left_jaw,
            spine,
            contact_tol=0.001,
            name="left_jaw_stays_supported_when_open",
        )
        ctx.expect_contact(
            right_jaw,
            spine,
            contact_tol=0.001,
            name="right_jaw_stays_supported_when_open",
        )
        ctx.expect_gap(
            right_jaw,
            left_jaw,
            axis="x",
            min_gap=0.078,
            max_gap=0.100,
            name="open_pose_creates_large_grasp_width",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
