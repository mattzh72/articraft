from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="xz_positioning_stage")

    anodized_black = model.material("anodized_black", color=(0.18, 0.19, 0.20, 1.0))
    machined_aluminum = model.material(
        "machined_aluminum", color=(0.76, 0.78, 0.80, 1.0)
    )
    plate_blue = model.material("fixture_blue", color=(0.22, 0.41, 0.68, 1.0))

    base = model.part("base_rail")
    base.visual(
        Box((0.46, 0.11, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=anodized_black,
        name="base_plate",
    )
    base.visual(
        Box((0.40, 0.05, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=anodized_black,
        name="center_web",
    )
    base.visual(
        Box((0.40, 0.018, 0.016)),
        origin=Origin(xyz=(0.0, 0.030, 0.026)),
        material=machined_aluminum,
        name="left_rail",
    )
    base.visual(
        Box((0.40, 0.018, 0.016)),
        origin=Origin(xyz=(0.0, -0.030, 0.026)),
        material=machined_aluminum,
        name="right_rail",
    )
    base.visual(
        Box((0.03, 0.11, 0.05)),
        origin=Origin(xyz=(-0.215, 0.0, 0.025)),
        material=anodized_black,
        name="left_end_block",
    )
    base.visual(
        Box((0.03, 0.11, 0.05)),
        origin=Origin(xyz=(0.215, 0.0, 0.025)),
        material=anodized_black,
        name="right_end_block",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.46, 0.11, 0.05)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )

    carriage = model.part("x_carriage")
    carriage.visual(
        Box((0.10, 0.018, 0.014)),
        origin=Origin(xyz=(0.0, 0.030, 0.041)),
        material=machined_aluminum,
        name="left_bearing_shoe",
    )
    carriage.visual(
        Box((0.10, 0.018, 0.014)),
        origin=Origin(xyz=(0.0, -0.030, 0.041)),
        material=machined_aluminum,
        name="right_bearing_shoe",
    )
    carriage.visual(
        Box((0.12, 0.10, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material=machined_aluminum,
        name="carriage_deck",
    )
    carriage.visual(
        Box((0.07, 0.028, 0.247)),
        origin=Origin(xyz=(0.0, 0.0, 0.1825)),
        material=machined_aluminum,
        name="vertical_column",
    )
    carriage.visual(
        Box((0.012, 0.014, 0.22)),
        origin=Origin(xyz=(0.020, 0.021, 0.180)),
        material=anodized_black,
        name="left_mast_rail",
    )
    carriage.visual(
        Box((0.012, 0.014, 0.22)),
        origin=Origin(xyz=(-0.020, 0.021, 0.180)),
        material=anodized_black,
        name="right_mast_rail",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.12, 0.10, 0.31)),
        mass=2.1,
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
    )

    slide = model.part("z_slide")
    slide.visual(
        Box((0.018, 0.012, 0.08)),
        origin=Origin(xyz=(0.020, 0.034, 0.050)),
        material=machined_aluminum,
        name="left_guide_pad",
    )
    slide.visual(
        Box((0.018, 0.012, 0.08)),
        origin=Origin(xyz=(-0.020, 0.034, 0.050)),
        material=machined_aluminum,
        name="right_guide_pad",
    )
    slide.visual(
        Box((0.065, 0.018, 0.10)),
        origin=Origin(xyz=(0.0, 0.047, 0.060)),
        material=machined_aluminum,
        name="slide_body",
    )
    slide.visual(
        Box((0.030, 0.022, 0.018)),
        origin=Origin(xyz=(0.0, 0.050, 0.018)),
        material=anodized_black,
        name="tool_mount",
    )
    slide.visual(
        Box((0.080, 0.010, 0.055)),
        origin=Origin(xyz=(0.0, 0.061, 0.050)),
        material=plate_blue,
        name="tool_plate",
    )
    slide.inertial = Inertial.from_geometry(
        Box((0.08, 0.07, 0.12)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.050, 0.060)),
    )

    model.articulation(
        "x_axis",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=200.0,
            velocity=0.25,
            lower=-0.13,
            upper=0.13,
        ),
    )
    model.articulation(
        "z_axis",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=slide,
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=150.0,
            velocity=0.20,
            lower=0.0,
            upper=0.12,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_rail")
    carriage = object_model.get_part("x_carriage")
    slide = object_model.get_part("z_slide")
    x_axis = object_model.get_articulation("x_axis")
    z_axis = object_model.get_articulation("z_axis")

    left_rail = base.get_visual("left_rail")
    right_rail = base.get_visual("right_rail")
    left_shoe = carriage.get_visual("left_bearing_shoe")
    right_shoe = carriage.get_visual("right_bearing_shoe")
    deck = carriage.get_visual("carriage_deck")
    left_mast_rail = carriage.get_visual("left_mast_rail")
    right_mast_rail = carriage.get_visual("right_mast_rail")
    left_pad = slide.get_visual("left_guide_pad")
    right_pad = slide.get_visual("right_guide_pad")
    tool_plate = slide.get_visual("tool_plate")

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=48)

    ctx.check(
        "x_axis_orientation",
        tuple(x_axis.axis) == (1.0, 0.0, 0.0),
        f"expected x_axis along +X, got {x_axis.axis}",
    )
    ctx.check(
        "z_axis_orientation",
        tuple(z_axis.axis) == (0.0, 0.0, 1.0),
        f"expected z_axis along +Z, got {z_axis.axis}",
    )

    x_limits = x_axis.motion_limits
    z_limits = z_axis.motion_limits
    ctx.check(
        "x_axis_travel_realistic",
        x_limits is not None
        and x_limits.lower is not None
        and x_limits.upper is not None
        and 0.24 <= (x_limits.upper - x_limits.lower) <= 0.30,
        f"unexpected x travel: {x_limits}",
    )
    ctx.check(
        "z_axis_travel_realistic",
        z_limits is not None
        and z_limits.lower is not None
        and z_limits.upper is not None
        and 0.10 <= (z_limits.upper - z_limits.lower) <= 0.16,
        f"unexpected z travel: {z_limits}",
    )

    base_aabb = ctx.part_world_aabb(base)
    if base_aabb is not None:
        base_length = base_aabb[1][0] - base_aabb[0][0]
        base_width = base_aabb[1][1] - base_aabb[0][1]
        ctx.check(
            "base_scale_realistic",
            0.40 <= base_length <= 0.55 and 0.08 <= base_width <= 0.13,
            f"base dimensions look wrong: length={base_length:.3f}, width={base_width:.3f}",
        )

    with ctx.pose({x_axis: 0.0, z_axis: 0.0}):
        ctx.expect_contact(
            carriage,
            base,
            elem_a=left_shoe,
            elem_b=left_rail,
            name="left_x_bearing_contact_rest",
        )
        ctx.expect_contact(
            carriage,
            base,
            elem_a=right_shoe,
            elem_b=right_rail,
            name="right_x_bearing_contact_rest",
        )
        ctx.expect_contact(
            slide,
            carriage,
            elem_a=left_pad,
            elem_b=left_mast_rail,
            name="left_z_pad_contact_rest",
        )
        ctx.expect_contact(
            slide,
            carriage,
            elem_a=right_pad,
            elem_b=right_mast_rail,
            name="right_z_pad_contact_rest",
        )
        ctx.expect_gap(
            slide,
            carriage,
            axis="z",
            min_gap=0.02,
            positive_elem=tool_plate,
            negative_elem=deck,
            name="tool_plate_clears_carriage_deck",
        )

    if x_limits is not None and x_limits.lower is not None and x_limits.upper is not None:
        for pose_value, label in (
            (x_limits.lower, "lower"),
            (x_limits.upper, "upper"),
        ):
            with ctx.pose({x_axis: pose_value, z_axis: 0.0}):
                ctx.fail_if_parts_overlap_in_current_pose(
                    name=f"x_axis_{label}_no_overlap"
                )
                ctx.fail_if_isolated_parts(name=f"x_axis_{label}_no_floating")
                ctx.expect_contact(
                    carriage,
                    base,
                    elem_a=left_shoe,
                    elem_b=left_rail,
                    name=f"left_x_bearing_contact_{label}",
                )
                ctx.expect_contact(
                    carriage,
                    base,
                    elem_a=right_shoe,
                    elem_b=right_rail,
                    name=f"right_x_bearing_contact_{label}",
                )
                ctx.expect_within(
                    carriage,
                    base,
                    axes="x",
                    inner_elem=left_shoe,
                    outer_elem=left_rail,
                    name=f"left_bearing_stays_on_rail_{label}",
                )
                ctx.expect_within(
                    carriage,
                    base,
                    axes="x",
                    inner_elem=right_shoe,
                    outer_elem=right_rail,
                    name=f"right_bearing_stays_on_rail_{label}",
                )

    if z_limits is not None and z_limits.lower is not None and z_limits.upper is not None:
        for pose_value, label in (
            (z_limits.lower, "lower"),
            (z_limits.upper, "upper"),
        ):
            with ctx.pose({x_axis: 0.0, z_axis: pose_value}):
                ctx.fail_if_parts_overlap_in_current_pose(
                    name=f"z_axis_{label}_no_overlap"
                )
                ctx.fail_if_isolated_parts(name=f"z_axis_{label}_no_floating")
                ctx.expect_contact(
                    slide,
                    carriage,
                    elem_a=left_pad,
                    elem_b=left_mast_rail,
                    name=f"left_z_pad_contact_{label}",
                )
                ctx.expect_contact(
                    slide,
                    carriage,
                    elem_a=right_pad,
                    elem_b=right_mast_rail,
                    name=f"right_z_pad_contact_{label}",
                )
                ctx.expect_within(
                    slide,
                    carriage,
                    axes="z",
                    inner_elem=left_pad,
                    outer_elem=left_mast_rail,
                    name=f"left_z_pad_stays_on_rail_{label}",
                )
                ctx.expect_within(
                    slide,
                    carriage,
                    axes="z",
                    inner_elem=right_pad,
                    outer_elem=right_mast_rail,
                    name=f"right_z_pad_stays_on_rail_{label}",
                )

    if (
        x_limits is not None
        and x_limits.lower is not None
        and x_limits.upper is not None
        and z_limits is not None
        and z_limits.lower is not None
        and z_limits.upper is not None
    ):
        for x_value, x_label in ((x_limits.lower, "lower"), (x_limits.upper, "upper")):
            for z_value, z_label in (
                (z_limits.lower, "lower"),
                (z_limits.upper, "upper"),
            ):
                with ctx.pose({x_axis: x_value, z_axis: z_value}):
                    ctx.fail_if_parts_overlap_in_current_pose(
                        name=f"corner_pose_{x_label}_{z_label}_no_overlap"
                    )
                    ctx.fail_if_isolated_parts(
                        name=f"corner_pose_{x_label}_{z_label}_no_floating"
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

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
