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


FRAME_WIDTH = 0.82
FRAME_DEPTH = 0.28
FRAME_HEIGHT = 0.63

X_TRAVEL = 0.16
Y_TRAVEL = 0.08
Z_TRAVEL = 0.12


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _rear_frame_shape() -> cq.Workplane:
    frame = _box((0.18, FRAME_DEPTH, 0.04), (-0.27, -0.14, 0.02))
    frame = frame.union(_box((0.18, FRAME_DEPTH, 0.04), (0.27, -0.14, 0.02)))
    frame = frame.union(_box((0.56, 0.14, 0.06), (0.0, -0.11, 0.07)))
    frame = frame.union(_box((0.10, 0.10, 0.53), (-0.31, -0.05, 0.265)))
    frame = frame.union(_box((0.10, 0.10, 0.53), (0.31, -0.05, 0.265)))
    frame = frame.union(_box((0.74, 0.10, 0.10), (0.0, -0.05, 0.58)))
    rear_panel = _box((0.62, 0.03, 0.22), (0.0, -0.085, 0.27))
    rear_panel = rear_panel.cut(_box((0.40, 0.06, 0.11), (0.0, -0.085, 0.26)))
    frame = frame.union(rear_panel)
    frame = frame.union(_box((0.72, 0.024, 0.026), (0.0, -0.012, 0.423)))
    frame = frame.union(_box((0.72, 0.024, 0.026), (0.0, -0.012, 0.503)))
    frame = frame.union(_box((0.72, 0.016, 0.018), (0.0, -0.008, 0.463)))
    return frame


def _lower_carriage_shape() -> cq.Workplane:
    carriage = _box((0.48, 0.34, 0.12), (0.0, 0.17, -0.005))
    carriage = carriage.cut(_box((0.14, 0.21, 0.14), (0.0, 0.20, -0.005)))
    carriage = carriage.cut(_box((0.28, 0.18, 0.06), (0.0, 0.19, -0.040)))
    carriage = carriage.union(_box((0.024, 0.34, 0.012), (-0.10, 0.17, 0.061)))
    carriage = carriage.union(_box((0.024, 0.34, 0.012), (0.10, 0.17, 0.061)))
    carriage = carriage.union(_box((0.18, 0.035, 0.045), (0.0, 0.325, 0.010)))
    carriage = carriage.union(_box((0.12, 0.030, 0.035), (0.0, 0.020, 0.010)))
    return carriage


def _cross_slide_shape() -> cq.Workplane:
    slide = _box((0.24, 0.16, 0.024), (0.0, 0.0, 0.012))
    slide = slide.union(_box((0.090, 0.012, 0.220), (0.0, -0.024, 0.110)))
    slide = slide.union(_box((0.024, 0.050, 0.100), (-0.044, -0.005, 0.062)))
    slide = slide.union(_box((0.024, 0.050, 0.100), (0.044, -0.005, 0.062)))
    slide = slide.union(_box((0.118, 0.040, 0.016), (0.0, -0.004, 0.116)))
    slide = slide.union(_box((0.120, 0.032, 0.026), (0.0, 0.064, 0.013)))
    slide = slide.union(_box((0.090, 0.024, 0.026), (0.0, -0.068, 0.013)))
    return slide


def _vertical_ram_shape() -> cq.Workplane:
    ram = _box((0.058, 0.040, 0.160), (0.0, 0.012, 0.080))
    ram = ram.union(_box((0.070, 0.010, 0.180), (0.0, -0.013, 0.090)))
    ram = ram.union(_box((0.082, 0.056, 0.018), (0.0, 0.012, 0.169)))
    ram = ram.union(_box((0.034, 0.026, 0.060), (0.0, 0.012, 0.208)))
    ram = ram.union(_box((0.048, 0.040, 0.012), (0.0, 0.012, 0.244)))
    return ram


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_backed_positioning_stack")

    model.material("frame_graphite", rgba=(0.23, 0.25, 0.28, 1.0))
    model.material("carriage_gray", rgba=(0.60, 0.63, 0.67, 1.0))
    model.material("slide_bluegray", rgba=(0.47, 0.52, 0.60, 1.0))
    model.material("ram_silver", rgba=(0.76, 0.78, 0.81, 1.0))

    rear_frame = model.part("rear_frame")
    rear_frame.visual(
        mesh_from_cadquery(_rear_frame_shape(), "rear_frame"),
        material="frame_graphite",
        name="rear_frame_body",
    )

    lower_carriage = model.part("lower_carriage")
    lower_carriage.visual(
        mesh_from_cadquery(_lower_carriage_shape(), "lower_carriage"),
        material="carriage_gray",
        name="lower_carriage_body",
    )

    cross_slide = model.part("cross_slide")
    cross_slide.visual(
        Box((0.24, 0.16, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material="slide_bluegray",
        name="slide_base",
    )
    cross_slide.visual(
        Box((0.11, 0.012, 0.28)),
        origin=Origin(xyz=(0.0, -0.028, 0.140)),
        material="slide_bluegray",
        name="guide_backplate",
    )
    cross_slide.visual(
        Box((0.014, 0.032, 0.24)),
        origin=Origin(xyz=(-0.031, -0.010, 0.120)),
        material="slide_bluegray",
        name="left_guide_rail",
    )
    cross_slide.visual(
        Box((0.014, 0.032, 0.24)),
        origin=Origin(xyz=(0.031, -0.010, 0.120)),
        material="slide_bluegray",
        name="right_guide_rail",
    )
    cross_slide.visual(
        Box((0.10, 0.035, 0.018)),
        origin=Origin(xyz=(0.0, 0.060, 0.009)),
        material="slide_bluegray",
        name="front_pad",
    )
    cross_slide.visual(
        Box((0.024, 0.050, 0.050)),
        origin=Origin(xyz=(-0.044, 0.010, 0.034)),
        material="slide_bluegray",
        name="left_gusset",
    )
    cross_slide.visual(
        Box((0.024, 0.050, 0.050)),
        origin=Origin(xyz=(0.044, 0.010, 0.034)),
        material="slide_bluegray",
        name="right_gusset",
    )

    vertical_ram = model.part("vertical_ram")
    vertical_ram.visual(
        Box((0.042, 0.012, 0.26)),
        origin=Origin(xyz=(0.0, -0.010, 0.130)),
        material="ram_silver",
        name="guide_blade",
    )
    vertical_ram.visual(
        Box((0.016, 0.040, 0.060)),
        origin=Origin(xyz=(0.0, 0.010, 0.055)),
        material="ram_silver",
        name="ram_bracket",
    )
    vertical_ram.visual(
        Box((0.070, 0.050, 0.130)),
        origin=Origin(xyz=(0.0, 0.040, 0.085)),
        material="ram_silver",
        name="ram_column",
    )
    vertical_ram.visual(
        Box((0.086, 0.060, 0.016)),
        origin=Origin(xyz=(0.0, 0.040, 0.158)),
        material="ram_silver",
        name="ram_head",
    )
    vertical_ram.visual(
        Box((0.032, 0.028, 0.048)),
        origin=Origin(xyz=(0.0, 0.040, 0.190)),
        material="ram_silver",
        name="tool_block",
    )

    model.articulation(
        "frame_to_lower_carriage",
        ArticulationType.PRISMATIC,
        parent=rear_frame,
        child=lower_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.463)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-X_TRAVEL,
            upper=X_TRAVEL,
            effort=1800.0,
            velocity=0.45,
        ),
    )
    model.articulation(
        "lower_carriage_to_cross_slide",
        ArticulationType.PRISMATIC,
        parent=lower_carriage,
        child=cross_slide,
        origin=Origin(xyz=(0.0, 0.21, 0.067)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-Y_TRAVEL,
            upper=Y_TRAVEL,
            effort=900.0,
            velocity=0.30,
        ),
    )
    model.articulation(
        "cross_slide_to_vertical_ram",
        ArticulationType.PRISMATIC,
        parent=cross_slide,
        child=vertical_ram,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=Z_TRAVEL,
            effort=700.0,
            velocity=0.22,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rear_frame = object_model.get_part("rear_frame")
    lower_carriage = object_model.get_part("lower_carriage")
    cross_slide = object_model.get_part("cross_slide")
    vertical_ram = object_model.get_part("vertical_ram")

    x_axis = object_model.get_articulation("frame_to_lower_carriage")
    y_axis = object_model.get_articulation("lower_carriage_to_cross_slide")
    z_axis = object_model.get_articulation("cross_slide_to_vertical_ram")

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
        lower_carriage,
        rear_frame,
        name="lower_carriage_is_supported_by_rear_frame",
    )
    ctx.expect_contact(
        cross_slide,
        lower_carriage,
        name="cross_slide_is_supported_by_lower_carriage",
    )
    ctx.expect_within(
        vertical_ram,
        cross_slide,
        axes="xy",
        inner_elem="guide_blade",
        margin=0.0,
        name="vertical_ram_guide_blade_stays_within_cross_slide_envelope",
    )
    ctx.expect_overlap(
        vertical_ram,
        cross_slide,
        axes="z",
        elem_a="guide_blade",
        min_overlap=0.18,
        name="vertical_ram_guide_blade_overlaps_cross_slide_in_z",
    )

    def check_prismatic_motion(joint, child, axis: str, min_delta: float, name: str) -> bool:
        axis_index = {"x": 0, "y": 1, "z": 2}[axis]
        rest = ctx.part_world_position(child)
        with ctx.pose({joint: joint.motion_limits.upper}):
            moved = ctx.part_world_position(child)
        if rest is None or moved is None:
            return ctx.fail(name, "could not resolve child world positions")
        primary_delta = moved[axis_index] - rest[axis_index]
        orthogonal = [abs(moved[i] - rest[i]) for i in range(3) if i != axis_index]
        return ctx.check(
            name,
            primary_delta >= min_delta and max(orthogonal) <= 0.001,
            (
                f"primary_delta={primary_delta:.4f}, "
                f"orthogonal=({orthogonal[0]:.4f}, {orthogonal[1]:.4f})"
            ),
        )

    check_prismatic_motion(
        x_axis,
        lower_carriage,
        axis="x",
        min_delta=0.12,
        name="x_stage_moves_along_world_x",
    )
    check_prismatic_motion(
        y_axis,
        cross_slide,
        axis="y",
        min_delta=0.06,
        name="y_stage_moves_along_world_y",
    )
    check_prismatic_motion(
        z_axis,
        vertical_ram,
        axis="z",
        min_delta=0.09,
        name="z_stage_moves_along_world_z",
    )

    lower_pose = {
        x_axis: x_axis.motion_limits.lower,
        y_axis: y_axis.motion_limits.lower,
        z_axis: z_axis.motion_limits.lower,
    }
    with ctx.pose(lower_pose):
        ctx.expect_contact(
            lower_carriage,
            rear_frame,
            name="lower_carriage_stays_supported_at_lower_limits",
        )
        ctx.expect_contact(
            cross_slide,
            lower_carriage,
            name="cross_slide_stays_supported_at_lower_limits",
        )
        ctx.expect_within(
            vertical_ram,
            cross_slide,
            axes="xy",
            inner_elem="guide_blade",
            margin=0.0,
            name="vertical_ram_guide_blade_stays_within_cross_slide_at_lower_limits",
        )
        ctx.expect_overlap(
            vertical_ram,
            cross_slide,
            axes="z",
            elem_a="guide_blade",
            min_overlap=0.18,
            name="vertical_ram_guide_blade_overlaps_cross_slide_at_lower_limits",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlaps_at_lower_limits")

    upper_pose = {
        x_axis: x_axis.motion_limits.upper,
        y_axis: y_axis.motion_limits.upper,
        z_axis: z_axis.motion_limits.upper,
    }
    with ctx.pose(upper_pose):
        ctx.expect_contact(
            lower_carriage,
            rear_frame,
            name="lower_carriage_stays_supported_at_upper_limits",
        )
        ctx.expect_contact(
            cross_slide,
            lower_carriage,
            name="cross_slide_stays_supported_at_upper_limits",
        )
        ctx.expect_within(
            vertical_ram,
            cross_slide,
            axes="xy",
            inner_elem="guide_blade",
            margin=0.0,
            name="vertical_ram_guide_blade_stays_within_cross_slide_at_upper_limits",
        )
        ctx.expect_overlap(
            vertical_ram,
            cross_slide,
            axes="z",
            elem_a="guide_blade",
            min_overlap=0.12,
            name="vertical_ram_guide_blade_overlaps_cross_slide_at_upper_limits",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlaps_at_upper_limits")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
