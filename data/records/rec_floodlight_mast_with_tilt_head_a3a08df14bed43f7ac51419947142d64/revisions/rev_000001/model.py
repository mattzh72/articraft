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
    tube_from_spline_points,
)


LEG_TRAVEL = 0.10
MAST_TRAVEL = 0.55
HEAD_TILT_LOWER = -1.15
HEAD_TILT_UPPER = 0.40


def _build_carry_handle():
    handle_geom = tube_from_spline_points(
        [
            (-0.13, -0.05, 0.26),
            (-0.13, -0.13, 0.35),
            (0.13, -0.13, 0.35),
            (0.13, -0.05, 0.26),
        ],
        radius=0.009,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    return mesh_from_geometry(handle_geom, "work_light_carry_handle")


def _add_flood_head(part, *, side_sign: float, housing_material, frame_material, lens_material) -> None:
    part.visual(
        Cylinder(radius=0.021, length=0.040),
        origin=Origin(
            xyz=(0.020 * side_sign, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=frame_material,
        name="pivot_barrel",
    )
    part.visual(
        Box((0.072, 0.030, 0.058)),
        origin=Origin(xyz=(0.066 * side_sign, 0.018, -0.015)),
        material=frame_material,
        name="support_arm",
    )
    part.visual(
        Box((0.188, 0.062, 0.132)),
        origin=Origin(xyz=(0.118 * side_sign, 0.050, -0.045)),
        material=housing_material,
        name="housing",
    )
    part.visual(
        Box((0.198, 0.018, 0.142)),
        origin=Origin(xyz=(0.118 * side_sign, 0.076, -0.045)),
        material=frame_material,
        name="front_bezel",
    )
    part.visual(
        Box((0.168, 0.008, 0.106)),
        origin=Origin(xyz=(0.118 * side_sign, 0.084, -0.045)),
        material=lens_material,
        name="lens_panel",
    )
    part.visual(
        Box((0.196, 0.028, 0.018)),
        origin=Origin(xyz=(0.118 * side_sign, 0.074, 0.030)),
        material=frame_material,
        name="visor",
    )
    for index, y in enumerate((-0.004, 0.008, 0.020)):
        part.visual(
            Box((0.158, 0.006, 0.102)),
            origin=Origin(xyz=(0.118 * side_sign, y, -0.045)),
            material=frame_material,
            name=f"heat_sink_fin_{index}",
        )


def _center_z(aabb) -> float | None:
    if aabb is None:
        return None
    return 0.5 * (aabb[0][2] + aabb[1][2])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_work_light_tower")

    charcoal = model.material("charcoal", rgba=(0.16, 0.17, 0.18, 1.0))
    graphite = model.material("graphite", rgba=(0.24, 0.25, 0.27, 1.0))
    aluminum = model.material("aluminum", rgba=(0.72, 0.74, 0.77, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.93, 0.78, 0.16, 1.0))
    smoked_lens = model.material("smoked_lens", rgba=(0.90, 0.94, 0.98, 0.55))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    carry_handle_mesh = _build_carry_handle()

    base = model.part("base_frame")
    base.visual(
        Box((0.48, 0.14, 0.06)),
        origin=Origin(xyz=(-0.30, 0.0, 0.23)),
        material=charcoal,
        name="left_crossarm",
    )
    base.visual(
        Box((0.48, 0.14, 0.06)),
        origin=Origin(xyz=(0.30, 0.0, 0.23)),
        material=charcoal,
        name="right_crossarm",
    )
    base.visual(
        Box((0.12, 0.18, 0.12)),
        origin=Origin(xyz=(-0.12, 0.0, 0.08)),
        material=graphite,
        name="left_ballast",
    )
    base.visual(
        Box((0.12, 0.18, 0.12)),
        origin=Origin(xyz=(0.12, 0.0, 0.08)),
        material=graphite,
        name="right_ballast",
    )
    base.visual(
        Box((0.12, 0.12, 0.26)),
        origin=Origin(xyz=(0.0, 0.0, 0.27)),
        material=charcoal,
        name="mast_sleeve",
    )
    base.visual(
        Box((0.09, 0.09, 0.18)),
        origin=Origin(xyz=(-0.42, 0.0, 0.11)),
        material=charcoal,
        name="left_leg_sleeve",
    )
    base.visual(
        Box((0.09, 0.09, 0.18)),
        origin=Origin(xyz=(0.42, 0.0, 0.11)),
        material=charcoal,
        name="right_leg_sleeve",
    )
    base.visual(
        Box((0.15, 0.03, 0.06)),
        origin=Origin(xyz=(0.0, -0.070, 0.40)),
        material=charcoal,
        name="mast_cap_block",
    )
    base.visual(
        Box((0.10, 0.03, 0.10)),
        origin=Origin(xyz=(0.086, -0.085, 0.26)),
        material=graphite,
        name="control_box",
    )
    base.visual(carry_handle_mesh, material=aluminum, name="carry_handle")
    base.inertial = Inertial.from_geometry(
        Box((1.12, 0.24, 0.46)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.23)),
    )

    left_leg = model.part("left_leg_post")
    left_leg.visual(
        Box((0.06, 0.06, 0.40)),
        origin=Origin(xyz=(0.0, 0.0, -0.03)),
        material=aluminum,
        name="inner_tube",
    )
    left_leg.visual(
        Box((0.08, 0.05, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, -0.214)),
        material=charcoal,
        name="foot_bracket",
    )
    left_leg.visual(
        Box((0.11, 0.07, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, -0.242)),
        material=rubber,
        name="foot_pad",
    )
    left_leg.inertial = Inertial.from_geometry(
        Box((0.12, 0.08, 0.45)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, -0.05)),
    )

    right_leg = model.part("right_leg_post")
    right_leg.visual(
        Box((0.06, 0.06, 0.40)),
        origin=Origin(xyz=(0.0, 0.0, -0.03)),
        material=aluminum,
        name="inner_tube",
    )
    right_leg.visual(
        Box((0.08, 0.05, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, -0.214)),
        material=charcoal,
        name="foot_bracket",
    )
    right_leg.visual(
        Box((0.11, 0.07, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, -0.242)),
        material=rubber,
        name="foot_pad",
    )
    right_leg.inertial = Inertial.from_geometry(
        Box((0.12, 0.08, 0.45)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, -0.05)),
    )

    mast = model.part("mast_column")
    mast.visual(
        Box((0.078, 0.078, 2.30)),
        origin=Origin(xyz=(0.0, 0.0, 0.45)),
        material=aluminum,
        name="inner_tube",
    )
    mast.visual(
        Box((0.10, 0.08, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 1.46)),
        material=charcoal,
        name="headstock",
    )
    mast.visual(
        Box((0.80, 0.05, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 1.56)),
        material=charcoal,
        name="t_bar",
    )
    mast.visual(
        Box((0.12, 0.03, 0.22)),
        origin=Origin(xyz=(0.0, 0.0, 1.45)),
        material=charcoal,
        name="t_bar_spine",
    )
    mast.visual(
        Cylinder(radius=0.022, length=0.04),
        origin=Origin(
            xyz=(-0.405, 0.0, 1.555),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=charcoal,
        name="left_pivot_stub",
    )
    mast.visual(
        Cylinder(radius=0.022, length=0.04),
        origin=Origin(
            xyz=(0.405, 0.0, 1.555),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=charcoal,
        name="right_pivot_stub",
    )
    mast.visual(
        Box((0.10, 0.035, 0.12)),
        origin=Origin(xyz=(0.058, -0.030, 0.58)),
        material=graphite,
        name="switch_box",
    )
    mast.inertial = Inertial.from_geometry(
        Box((0.86, 0.14, 2.35)),
        mass=7.2,
        origin=Origin(xyz=(0.0, 0.0, 0.55)),
    )

    left_head = model.part("left_flood_head")
    _add_flood_head(
        left_head,
        side_sign=-1.0,
        housing_material=safety_yellow,
        frame_material=charcoal,
        lens_material=smoked_lens,
    )
    left_head.inertial = Inertial.from_geometry(
        Box((0.24, 0.12, 0.16)),
        mass=1.1,
        origin=Origin(xyz=(-0.12, 0.045, -0.035)),
    )

    right_head = model.part("right_flood_head")
    _add_flood_head(
        right_head,
        side_sign=1.0,
        housing_material=safety_yellow,
        frame_material=charcoal,
        lens_material=smoked_lens,
    )
    right_head.inertial = Inertial.from_geometry(
        Box((0.24, 0.12, 0.16)),
        mass=1.1,
        origin=Origin(xyz=(0.12, 0.045, -0.035)),
    )

    model.articulation(
        "base_to_left_leg",
        ArticulationType.PRISMATIC,
        parent=base,
        child=left_leg,
        origin=Origin(xyz=(-0.42, 0.0, 0.02)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.12,
            lower=0.0,
            upper=LEG_TRAVEL,
        ),
    )
    model.articulation(
        "base_to_right_leg",
        ArticulationType.PRISMATIC,
        parent=base,
        child=right_leg,
        origin=Origin(xyz=(0.42, 0.0, 0.02)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.12,
            lower=0.0,
            upper=LEG_TRAVEL,
        ),
    )
    model.articulation(
        "base_to_mast",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 0.40)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.20,
            lower=0.0,
            upper=MAST_TRAVEL,
        ),
    )
    model.articulation(
        "mast_to_left_head",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=left_head,
        origin=Origin(xyz=(-0.425, 0.0, 1.555)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.6,
            lower=HEAD_TILT_LOWER,
            upper=HEAD_TILT_UPPER,
        ),
    )
    model.articulation(
        "mast_to_right_head",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=right_head,
        origin=Origin(xyz=(0.425, 0.0, 1.555)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.6,
            lower=HEAD_TILT_LOWER,
            upper=HEAD_TILT_UPPER,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base_frame")
    left_leg = object_model.get_part("left_leg_post")
    right_leg = object_model.get_part("right_leg_post")
    mast = object_model.get_part("mast_column")
    left_head = object_model.get_part("left_flood_head")
    right_head = object_model.get_part("right_flood_head")

    left_leg_slide = object_model.get_articulation("base_to_left_leg")
    right_leg_slide = object_model.get_articulation("base_to_right_leg")
    mast_slide = object_model.get_articulation("base_to_mast")
    left_head_tilt = object_model.get_articulation("mast_to_left_head")
    right_head_tilt = object_model.get_articulation("mast_to_right_head")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.allow_overlap(
        base,
        left_leg,
        elem_a="left_leg_sleeve",
        elem_b="inner_tube",
        reason="The left stabilizer post is represented as telescoping through a solid sleeve proxy.",
    )
    ctx.allow_overlap(
        base,
        right_leg,
        elem_a="right_leg_sleeve",
        elem_b="inner_tube",
        reason="The right stabilizer post is represented as telescoping through a solid sleeve proxy.",
    )
    ctx.allow_overlap(
        base,
        mast,
        elem_a="mast_sleeve",
        elem_b="inner_tube",
        reason="The mast is represented as telescoping inside a solid sleeve proxy on the cross-bar base.",
    )

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
        "leg posts slide downward from the cross-bar",
        left_leg_slide.axis == (0.0, 0.0, -1.0) and right_leg_slide.axis == (0.0, 0.0, -1.0),
        details=f"left_axis={left_leg_slide.axis}, right_axis={right_leg_slide.axis}",
    )
    ctx.check(
        "mast slides upward from the cross-bar",
        mast_slide.axis == (0.0, 0.0, 1.0),
        details=f"mast_axis={mast_slide.axis}",
    )
    ctx.check(
        "flood heads tilt on horizontal pivots",
        left_head_tilt.axis == (1.0, 0.0, 0.0) and right_head_tilt.axis == (1.0, 0.0, 0.0),
        details=f"left_axis={left_head_tilt.axis}, right_axis={right_head_tilt.axis}",
    )

    ctx.expect_within(
        left_leg,
        base,
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="left_leg_sleeve",
        margin=0.016,
        name="left leg stays centered in its sleeve",
    )
    ctx.expect_within(
        right_leg,
        base,
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="right_leg_sleeve",
        margin=0.016,
        name="right leg stays centered in its sleeve",
    )
    ctx.expect_overlap(
        left_leg,
        base,
        axes="z",
        elem_a="inner_tube",
        elem_b="left_leg_sleeve",
        min_overlap=0.15,
        name="left leg retains insertion at rest",
    )
    ctx.expect_overlap(
        right_leg,
        base,
        axes="z",
        elem_a="inner_tube",
        elem_b="right_leg_sleeve",
        min_overlap=0.15,
        name="right leg retains insertion at rest",
    )
    ctx.expect_within(
        mast,
        base,
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="mast_sleeve",
        margin=0.024,
        name="mast stays centered in the mast sleeve",
    )
    ctx.expect_overlap(
        mast,
        base,
        axes="z",
        elem_a="inner_tube",
        elem_b="mast_sleeve",
        min_overlap=0.20,
        name="mast retains insertion at rest",
    )
    ctx.expect_contact(
        mast,
        left_head,
        elem_a="left_pivot_stub",
        elem_b="pivot_barrel",
        name="left flood head is mounted to the left T-bar end",
    )
    ctx.expect_contact(
        mast,
        right_head,
        elem_a="right_pivot_stub",
        elem_b="pivot_barrel",
        name="right flood head is mounted to the right T-bar end",
    )
    ctx.expect_origin_gap(
        right_head,
        left_head,
        axis="x",
        min_gap=0.75,
        name="flood heads are spread across the T-bar",
    )

    left_leg_rest = ctx.part_world_position(left_leg)
    mast_rest = ctx.part_world_position(mast)
    lens_rest = ctx.part_element_world_aabb(left_head, elem="lens_panel")

    with ctx.pose(
        {
            left_leg_slide: LEG_TRAVEL,
            right_leg_slide: LEG_TRAVEL,
            mast_slide: MAST_TRAVEL,
        }
    ):
        ctx.expect_within(
            left_leg,
            base,
            axes="xy",
            inner_elem="inner_tube",
            outer_elem="left_leg_sleeve",
            margin=0.016,
            name="left leg stays centered when extended",
        )
        ctx.expect_within(
            right_leg,
            base,
            axes="xy",
            inner_elem="inner_tube",
            outer_elem="right_leg_sleeve",
            margin=0.016,
            name="right leg stays centered when extended",
        )
        ctx.expect_overlap(
            left_leg,
            base,
            axes="z",
            elem_a="inner_tube",
            elem_b="left_leg_sleeve",
            min_overlap=0.05,
            name="left leg keeps retained insertion when extended",
        )
        ctx.expect_overlap(
            right_leg,
            base,
            axes="z",
            elem_a="inner_tube",
            elem_b="right_leg_sleeve",
            min_overlap=0.05,
            name="right leg keeps retained insertion when extended",
        )
        ctx.expect_within(
            mast,
            base,
            axes="xy",
            inner_elem="inner_tube",
            outer_elem="mast_sleeve",
            margin=0.024,
            name="mast stays centered when raised",
        )
        ctx.expect_overlap(
            mast,
            base,
            axes="z",
            elem_a="inner_tube",
            elem_b="mast_sleeve",
            min_overlap=0.14,
            name="mast keeps retained insertion when raised",
        )
        left_leg_extended = ctx.part_world_position(left_leg)
        mast_extended = ctx.part_world_position(mast)

    with ctx.pose(
        {
            left_head_tilt: HEAD_TILT_LOWER,
            right_head_tilt: HEAD_TILT_LOWER,
        }
    ):
        lens_tilted = ctx.part_element_world_aabb(left_head, elem="lens_panel")

    ctx.check(
        "left leg post drops when extended",
        left_leg_rest is not None
        and left_leg_extended is not None
        and left_leg_extended[2] < left_leg_rest[2] - 0.08,
        details=f"rest={left_leg_rest}, extended={left_leg_extended}",
    )
    ctx.check(
        "mast rises when extended",
        mast_rest is not None and mast_extended is not None and mast_extended[2] > mast_rest[2] + 0.45,
        details=f"rest={mast_rest}, extended={mast_extended}",
    )
    ctx.check(
        "flood heads can tilt downward for task lighting",
        _center_z(lens_rest) is not None
        and _center_z(lens_tilted) is not None
        and _center_z(lens_tilted) < _center_z(lens_rest) - 0.05,
        details=f"rest_center_z={_center_z(lens_rest)}, tilted_center_z={_center_z(lens_tilted)}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
