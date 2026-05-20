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
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _yz_rounded_section(
    *,
    x: float,
    center_y: float,
    center_z: float,
    depth: float,
    height: float,
    radius: float,
    corner_segments: int = 8,
) -> list[tuple[float, float, float]]:
    return [
        (x, center_y + local_y, center_z + local_z)
        for local_y, local_z in rounded_rect_profile(
            depth,
            height,
            radius,
            corner_segments=corner_segments,
        )
    ]


def _build_lens_shield():
    section_specs = [
        (-0.096, -0.024, 0.026, 0.0028, 0.042),
        (-0.080, -0.010, 0.027, 0.0028, 0.046),
        (-0.058, 0.006, 0.028, 0.0026, 0.048),
        (-0.030, 0.014, 0.029, 0.0026, 0.050),
        (0.000, 0.016, 0.029, 0.0026, 0.051),
        (0.030, 0.014, 0.029, 0.0026, 0.050),
        (0.058, 0.006, 0.028, 0.0026, 0.048),
        (0.080, -0.010, 0.027, 0.0028, 0.046),
        (0.096, -0.024, 0.026, 0.0028, 0.042),
    ]
    sections = [
        _yz_rounded_section(
            x=x,
            center_y=center_y,
            center_z=center_z,
            depth=depth,
            height=height,
            radius=min(0.0012, depth * 0.45, height * 0.08),
        )
        for x, center_y, center_z, depth, height in section_specs
    ]
    return mesh_from_geometry(section_loft(sections), "safety_glasses_lens_shield")


def _add_front_hinge(
    frame_part,
    *,
    side: str,
    side_sign: float,
    material,
) -> None:
    name_prefix = f"{side}_hinge"
    frame_part.visual(
        Box((0.012, 0.008, 0.012)),
        origin=Origin(xyz=(side_sign * 0.093, -0.022, 0.031)),
        material=material,
        name=f"{name_prefix}_mount",
    )
    frame_part.visual(
        Cylinder(radius=0.0024, length=0.003),
        origin=Origin(xyz=(side_sign * 0.097, -0.024, 0.0345)),
        material=material,
        name=f"{side}_hinge_upper",
    )
    frame_part.visual(
        Cylinder(radius=0.0024, length=0.003),
        origin=Origin(xyz=(side_sign * 0.097, -0.024, 0.0275)),
        material=material,
        name=f"{side}_hinge_lower",
    )


def _build_temple_sleeve(part, *, material) -> None:
    part.visual(
        Cylinder(radius=0.0022, length=0.004),
        material=material,
        name="hinge_barrel",
    )
    part.visual(
        Box((0.012, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, -0.006, 0.0)),
        material=material,
        name="hinge_leaf",
    )
    part.visual(
        Box((0.012, 0.078, 0.0024)),
        origin=Origin(xyz=(0.0, -0.047, 0.0030)),
        material=material,
        name="top_strap",
    )
    part.visual(
        Box((0.0022, 0.078, 0.0064)),
        origin=Origin(xyz=(-0.0044, -0.047, -0.0004)),
        material=material,
        name="left_rail",
    )
    part.visual(
        Box((0.0022, 0.078, 0.0064)),
        origin=Origin(xyz=(0.0044, -0.047, -0.0004)),
        material=material,
        name="right_rail",
    )
    part.visual(
        Box((0.011, 0.022, 0.0042)),
        origin=Origin(xyz=(0.0, -0.017, 0.0020)),
        material=material,
        name="front_cap",
    )
    part.inertial = Inertial.from_geometry(
        Box((0.014, 0.090, 0.010)),
        mass=0.020,
        origin=Origin(xyz=(0.0, -0.043, 0.001)),
    )


def _build_temple_extension(part, *, bar_material, tip_material) -> None:
    part.visual(
        Box((0.0066, 0.060, 0.0040)),
        origin=Origin(xyz=(0.0, -0.030, -0.0002)),
        material=bar_material,
        name="runner",
    )
    part.visual(
        Box((0.0062, 0.072, 0.0038)),
        origin=Origin(xyz=(0.0, -0.091, -0.0004)),
        material=bar_material,
        name="outer_bar",
    )
    part.visual(
        Box((0.0076, 0.036, 0.0052)),
        origin=Origin(xyz=(0.0, -0.129, -0.0105), rpy=(0.34, 0.0, 0.0)),
        material=tip_material,
        name="tip_pad",
    )
    part.visual(
        Box((0.0066, 0.014, 0.0046)),
        origin=Origin(xyz=(0.0, -0.004, -0.0001)),
        material=bar_material,
        name="retention_head",
    )
    part.inertial = Inertial.from_geometry(
        Box((0.008, 0.140, 0.016)),
        mass=0.015,
        origin=Origin(xyz=(0.0, -0.075, -0.004)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="safety_glasses")

    frame_black = model.material("frame_black", rgba=(0.15, 0.16, 0.17, 1.0))
    lens_clear = model.material("lens_clear", rgba=(0.77, 0.88, 0.94, 0.38))
    temple_charcoal = model.material("temple_charcoal", rgba=(0.18, 0.19, 0.20, 1.0))
    temple_tip = model.material("temple_tip", rgba=(0.08, 0.09, 0.10, 1.0))

    lens_mesh = _build_lens_shield()

    front_frame = model.part("front_frame")
    front_frame.visual(lens_mesh, material=lens_clear, name="lens_shield")
    front_frame.visual(
        Box((0.088, 0.006, 0.008)),
        origin=Origin(xyz=(0.0, 0.015, 0.047)),
        material=frame_black,
        name="brow_bar_center",
    )
    front_frame.visual(
        Box((0.042, 0.010, 0.008)),
        origin=Origin(xyz=(-0.064, 0.005, 0.044)),
        material=frame_black,
        name="brow_bar_left",
    )
    front_frame.visual(
        Box((0.042, 0.010, 0.008)),
        origin=Origin(xyz=(0.064, 0.005, 0.044)),
        material=frame_black,
        name="brow_bar_right",
    )
    front_frame.visual(
        Box((0.020, 0.010, 0.016)),
        origin=Origin(xyz=(0.0, 0.012, 0.018)),
        material=frame_black,
        name="bridge",
    )
    front_frame.visual(
        Box((0.008, 0.010, 0.015)),
        origin=Origin(xyz=(-0.010, 0.007, 0.011), rpy=(0.30, 0.0, 0.0)),
        material=frame_black,
        name="left_nose_pad",
    )
    front_frame.visual(
        Box((0.008, 0.010, 0.015)),
        origin=Origin(xyz=(0.010, 0.007, 0.011), rpy=(0.30, 0.0, 0.0)),
        material=frame_black,
        name="right_nose_pad",
    )
    front_frame.visual(
        Box((0.004, 0.022, 0.028)),
        origin=Origin(xyz=(-0.091, -0.024, 0.026)),
        material=lens_clear,
        name="left_side_shield_rib",
    )
    front_frame.visual(
        Box((0.004, 0.022, 0.028)),
        origin=Origin(xyz=(0.091, -0.024, 0.026)),
        material=lens_clear,
        name="right_side_shield_rib",
    )
    _add_front_hinge(front_frame, side="left", side_sign=-1.0, material=frame_black)
    _add_front_hinge(front_frame, side="right", side_sign=1.0, material=frame_black)
    front_frame.inertial = Inertial.from_geometry(
        Box((0.198, 0.060, 0.060)),
        mass=0.060,
        origin=Origin(xyz=(0.0, -0.008, 0.028)),
    )

    left_temple_sleeve = model.part("left_temple_sleeve")
    _build_temple_sleeve(left_temple_sleeve, material=temple_charcoal)

    right_temple_sleeve = model.part("right_temple_sleeve")
    _build_temple_sleeve(right_temple_sleeve, material=temple_charcoal)

    left_temple_extension = model.part("left_temple_extension")
    _build_temple_extension(
        left_temple_extension,
        bar_material=temple_charcoal,
        tip_material=temple_tip,
    )

    right_temple_extension = model.part("right_temple_extension")
    _build_temple_extension(
        right_temple_extension,
        bar_material=temple_charcoal,
        tip_material=temple_tip,
    )

    model.articulation(
        "left_temple_hinge",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=left_temple_sleeve,
        origin=Origin(xyz=(-0.097, -0.024, 0.031)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=4.0,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "right_temple_hinge",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=right_temple_sleeve,
        origin=Origin(xyz=(0.097, -0.024, 0.031)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=4.0,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "left_temple_extension_slide",
        ArticulationType.PRISMATIC,
        parent=left_temple_sleeve,
        child=left_temple_extension,
        origin=Origin(xyz=(0.0, -0.018, -0.0002)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=0.10,
            lower=0.0,
            upper=0.035,
        ),
    )
    model.articulation(
        "right_temple_extension_slide",
        ArticulationType.PRISMATIC,
        parent=right_temple_sleeve,
        child=right_temple_extension,
        origin=Origin(xyz=(0.0, -0.018, -0.0002)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=0.10,
            lower=0.0,
            upper=0.035,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front_frame = object_model.get_part("front_frame")
    left_temple_sleeve = object_model.get_part("left_temple_sleeve")
    right_temple_sleeve = object_model.get_part("right_temple_sleeve")
    left_temple_extension = object_model.get_part("left_temple_extension")
    right_temple_extension = object_model.get_part("right_temple_extension")
    left_hinge = object_model.get_articulation("left_temple_hinge")
    right_hinge = object_model.get_articulation("right_temple_hinge")
    left_slide = object_model.get_articulation("left_temple_extension_slide")
    right_slide = object_model.get_articulation("right_temple_extension_slide")

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
        "left_hinge_axis_vertical",
        left_hinge.axis == (0.0, 0.0, 1.0),
        details=f"unexpected left hinge axis: {left_hinge.axis}",
    )
    ctx.check(
        "right_hinge_axis_vertical",
        right_hinge.axis == (0.0, 0.0, -1.0),
        details=f"unexpected right hinge axis: {right_hinge.axis}",
    )
    ctx.check(
        "left_slide_axis_rearward",
        left_slide.axis == (0.0, -1.0, 0.0),
        details=f"unexpected left slide axis: {left_slide.axis}",
    )
    ctx.check(
        "right_slide_axis_rearward",
        right_slide.axis == (0.0, -1.0, 0.0),
        details=f"unexpected right slide axis: {right_slide.axis}",
    )

    ctx.expect_contact(
        front_frame,
        left_temple_sleeve,
        elem_a="left_hinge_upper",
        elem_b="hinge_barrel",
        name="left_temple_hinge_contacts_frame",
    )
    ctx.expect_contact(
        front_frame,
        right_temple_sleeve,
        elem_a="right_hinge_upper",
        elem_b="hinge_barrel",
        name="right_temple_hinge_contacts_frame",
    )
    ctx.expect_gap(
        left_temple_sleeve,
        left_temple_extension,
        axis="z",
        positive_elem="top_strap",
        negative_elem="runner",
        min_gap=0.0001,
        max_gap=0.0003,
        name="left_runner_has_guided_clearance",
    )
    ctx.expect_gap(
        right_temple_sleeve,
        right_temple_extension,
        axis="z",
        positive_elem="top_strap",
        negative_elem="runner",
        min_gap=0.0001,
        max_gap=0.0003,
        name="right_runner_has_guided_clearance",
    )
    ctx.expect_within(
        left_temple_extension,
        left_temple_sleeve,
        axes="xz",
        inner_elem="runner",
        margin=0.0008,
        name="left_runner_stays_in_sleeve_cross_section",
    )
    ctx.expect_within(
        right_temple_extension,
        right_temple_sleeve,
        axes="xz",
        inner_elem="runner",
        margin=0.0008,
        name="right_runner_stays_in_sleeve_cross_section",
    )

    left_rest = ctx.part_element_world_aabb(left_temple_sleeve, elem="top_strap")
    right_rest = ctx.part_element_world_aabb(right_temple_sleeve, elem="top_strap")
    assert left_rest is not None
    assert right_rest is not None
    left_rest_center_x = (left_rest[0][0] + left_rest[1][0]) * 0.5
    right_rest_center_x = (right_rest[0][0] + right_rest[1][0]) * 0.5

    with ctx.pose({left_hinge: 1.25, right_hinge: 1.25}):
        left_folded = ctx.part_element_world_aabb(left_temple_sleeve, elem="top_strap")
        right_folded = ctx.part_element_world_aabb(right_temple_sleeve, elem="top_strap")
        assert left_folded is not None
        assert right_folded is not None
        left_folded_center_x = (left_folded[0][0] + left_folded[1][0]) * 0.5
        right_folded_center_x = (right_folded[0][0] + right_folded[1][0]) * 0.5
        ctx.check(
            "left_temple_folds_inward",
            left_folded_center_x > left_rest_center_x + 0.030,
            details=(
                f"left temple center x only moved from {left_rest_center_x:.4f} "
                f"to {left_folded_center_x:.4f}"
            ),
        )
        ctx.check(
            "right_temple_folds_inward",
            right_folded_center_x < right_rest_center_x - 0.030,
            details=(
                f"right temple center x only moved from {right_rest_center_x:.4f} "
                f"to {right_folded_center_x:.4f}"
            ),
        )

    left_tip_rest = ctx.part_element_world_aabb(left_temple_extension, elem="tip_pad")
    right_tip_rest = ctx.part_element_world_aabb(right_temple_extension, elem="tip_pad")
    assert left_tip_rest is not None
    assert right_tip_rest is not None

    with ctx.pose({left_slide: 0.035, right_slide: 0.035}):
        left_tip_extended = ctx.part_element_world_aabb(left_temple_extension, elem="tip_pad")
        right_tip_extended = ctx.part_element_world_aabb(right_temple_extension, elem="tip_pad")
        assert left_tip_extended is not None
        assert right_tip_extended is not None
        ctx.check(
            "left_extension_moves_rearward",
            left_tip_extended[0][1] < left_tip_rest[0][1] - 0.030,
            details=(
                f"left extension min y only moved from {left_tip_rest[0][1]:.4f} "
                f"to {left_tip_extended[0][1]:.4f}"
            ),
        )
        ctx.check(
            "right_extension_moves_rearward",
            right_tip_extended[0][1] < right_tip_rest[0][1] - 0.030,
            details=(
                f"right extension min y only moved from {right_tip_rest[0][1]:.4f} "
                f"to {right_tip_extended[0][1]:.4f}"
            ),
        )
        ctx.expect_gap(
            left_temple_sleeve,
            left_temple_extension,
            axis="z",
            positive_elem="top_strap",
            negative_elem="runner",
            min_gap=0.0001,
            max_gap=0.0003,
            name="left_runner_stays_guided_when_extended",
        )
        ctx.expect_gap(
            right_temple_sleeve,
            right_temple_extension,
            axis="z",
            positive_elem="top_strap",
            negative_elem="runner",
            min_gap=0.0001,
            max_gap=0.0003,
            name="right_runner_stays_guided_when_extended",
        )
        ctx.expect_overlap(
            left_temple_extension,
            left_temple_sleeve,
            axes="y",
            elem_a="runner",
            min_overlap=0.030,
            name="left_runner_remains_captured",
        )
        ctx.expect_overlap(
            right_temple_extension,
            right_temple_sleeve,
            axes="y",
            elem_a="runner",
            min_overlap=0.030,
            name="right_runner_remains_captured",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
