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


BOOM_CENTER_Z = 0.40

STAGE_1_LENGTH = 1.00
STAGE_1_WIDTH = 0.18
STAGE_1_HEIGHT = 0.13
STAGE_1_WALL = 0.009

STAGE_2_LENGTH = 0.82
STAGE_2_WIDTH = 0.135
STAGE_2_HEIGHT = 0.098
STAGE_2_WALL = 0.008

STAGE_3_LENGTH = 0.68
STAGE_3_WIDTH = 0.098
STAGE_3_HEIGHT = 0.071
STAGE_3_WALL = 0.0065

SUPPORT_TO_STAGE_1_X = -0.10
STAGE_1_TO_STAGE_2_X = 0.24
STAGE_2_TO_STAGE_3_X = 0.22

SLIDE_1_UPPER = 0.46
SLIDE_2_UPPER = 0.42
SLIDE_3_UPPER = 0.32

MESH_TOLERANCE = 0.0005
MESH_ANGULAR_TOLERANCE = 0.05
SLIDE_CLEARANCE = 0.0


def _box_at(length: float, width: float, height: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(length, width, height).translate(center)


def _make_side_plate(thickness: float) -> cq.Workplane:
    outer = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.22, 0.08),
                (0.08, 0.08),
                (0.16, 0.22),
                (0.16, 0.47),
                (0.01, 0.58),
                (-0.22, 0.58),
            ]
        )
        .close()
        .extrude(thickness)
    )
    window = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.16, 0.16),
                (0.01, 0.16),
                (0.08, 0.27),
                (0.08, 0.41),
                (-0.05, 0.51),
                (-0.16, 0.51),
            ]
        )
        .close()
        .extrude(thickness + 0.02)
        .translate((0.0, -0.01, 0.0))
    )
    return outer.cut(window)


def _make_rear_support() -> cq.Workplane:
    side_thickness = 0.03
    inner_clear_width = 0.22
    side_center_y = inner_clear_width / 2.0 + side_thickness / 2.0

    base = _box_at(0.54, 0.42, 0.08, (0.0, 0.0, 0.04))
    back_plate = _box_at(0.04, 0.28, 0.46, (-0.20, 0.0, 0.31))
    bridge = _box_at(0.18, 0.28, 0.04, (-0.04, 0.0, 0.55))
    lower_saddle = _box_at(0.36, 0.22, 0.03, (-0.02, 0.0, 0.31))
    front_web = _box_at(0.08, 0.10, 0.25, (0.11, 0.0, 0.205))

    pad_top_z = BOOM_CENTER_Z - STAGE_1_HEIGHT / 2.0 - SLIDE_CLEARANCE
    pad_height = 0.010
    pad_center_z = pad_top_z - pad_height / 2.0
    left_pad = _box_at(0.26, 0.03, pad_height, (0.02, 0.055, pad_center_z))
    right_pad = _box_at(0.26, 0.03, pad_height, (0.02, -0.055, pad_center_z))

    side_plate = _make_side_plate(side_thickness)
    left_side = side_plate.translate((0.0, side_center_y - side_thickness / 2.0, 0.0))
    right_side = side_plate.translate((0.0, -side_center_y - side_thickness / 2.0, 0.0))

    support = (
        base.union(back_plate)
        .union(bridge)
        .union(lower_saddle)
        .union(front_web)
        .union(left_pad)
        .union(right_pad)
        .union(left_side)
        .union(right_side)
    )
    return support


def _make_rectangular_stage(
    *,
    length: float,
    outer_width: float,
    outer_height: float,
    wall: float,
    guide_child_width: float | None = None,
    guide_child_height: float | None = None,
    guide_start: float = 0.0,
    guide_length: float = 0.0,
    cap_front: bool = False,
) -> cq.Workplane:
    inner_width = outer_width - 2.0 * wall
    inner_height = outer_height - 2.0 * wall

    stage = (
        cq.Workplane("YZ")
        .sketch()
        .rect(outer_width, outer_height, tag="outer")
        .rect(inner_width, inner_height, mode="s")
        .finalize()
        .extrude(length)
    )

    if guide_child_width is not None and guide_child_height is not None and guide_length > 0.0:
        pad_height = max((inner_height - guide_child_height) / 2.0 - SLIDE_CLEARANCE, 0.0)
        if pad_height > 0.0:
            pad_width = min(0.026, guide_child_width * 0.24)
            pad_center_y = guide_child_width * 0.32
            pad_center_z = -inner_height / 2.0 + pad_height / 2.0
            left_pad = _box_at(
                guide_length,
                pad_width,
                pad_height,
                (guide_start + guide_length / 2.0, pad_center_y, pad_center_z),
            )
            right_pad = _box_at(
                guide_length,
                pad_width,
                pad_height,
                (guide_start + guide_length / 2.0, -pad_center_y, pad_center_z),
            )
            stage = stage.union(left_pad).union(right_pad)

    if cap_front:
        cap_thickness = min(max(outer_height * 0.12, wall * 1.5), 0.018)
        tip_plate = (
            cq.Workplane("YZ")
            .rect(outer_width, outer_height)
            .extrude(cap_thickness)
            .translate((length - cap_thickness, 0.0, 0.0))
        )
        stage = stage.union(tip_plate)

    return stage


def _axis_is_positive_x(axis: tuple[float, float, float]) -> bool:
    return (
        len(axis) == 3
        and abs(axis[0] - 1.0) < 1e-9
        and abs(axis[1]) < 1e-9
        and abs(axis[2]) < 1e-9
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_backed_telescoping_reach_arm")

    support_mat = model.material("support_steel", rgba=(0.20, 0.22, 0.25, 1.0))
    outer_stage_mat = model.material("outer_stage_paint", rgba=(0.82, 0.68, 0.18, 1.0))
    mid_stage_mat = model.material("mid_stage_paint", rgba=(0.85, 0.72, 0.24, 1.0))
    inner_stage_mat = model.material("inner_stage_paint", rgba=(0.88, 0.76, 0.30, 1.0))

    support = model.part("rear_support")
    support.visual(
        mesh_from_cadquery(
            _make_rear_support(),
            "rear_support",
            tolerance=MESH_TOLERANCE,
            angular_tolerance=MESH_ANGULAR_TOLERANCE,
        ),
        material=support_mat,
        name="support_frame",
    )

    stage_1 = model.part("stage_1")
    stage_1.visual(
        mesh_from_cadquery(
            _make_rectangular_stage(
                length=STAGE_1_LENGTH,
                outer_width=STAGE_1_WIDTH,
                outer_height=STAGE_1_HEIGHT,
                wall=STAGE_1_WALL,
                guide_child_width=STAGE_2_WIDTH,
                guide_child_height=STAGE_2_HEIGHT,
                guide_start=0.10,
                guide_length=0.28,
            ),
            "stage_1",
            tolerance=MESH_TOLERANCE,
            angular_tolerance=MESH_ANGULAR_TOLERANCE,
        ),
        material=outer_stage_mat,
        name="stage_shell",
    )

    stage_2 = model.part("stage_2")
    stage_2.visual(
        mesh_from_cadquery(
            _make_rectangular_stage(
                length=STAGE_2_LENGTH,
                outer_width=STAGE_2_WIDTH,
                outer_height=STAGE_2_HEIGHT,
                wall=STAGE_2_WALL,
                guide_child_width=STAGE_3_WIDTH,
                guide_child_height=STAGE_3_HEIGHT,
                guide_start=0.12,
                guide_length=0.24,
            ),
            "stage_2",
            tolerance=MESH_TOLERANCE,
            angular_tolerance=MESH_ANGULAR_TOLERANCE,
        ),
        material=mid_stage_mat,
        name="stage_shell",
    )

    stage_3 = model.part("stage_3")
    stage_3.visual(
        mesh_from_cadquery(
            _make_rectangular_stage(
                length=STAGE_3_LENGTH,
                outer_width=STAGE_3_WIDTH,
                outer_height=STAGE_3_HEIGHT,
                wall=STAGE_3_WALL,
                cap_front=True,
            ),
            "stage_3",
            tolerance=MESH_TOLERANCE,
            angular_tolerance=MESH_ANGULAR_TOLERANCE,
        ),
        material=inner_stage_mat,
        name="stage_shell",
    )

    model.articulation(
        "support_to_stage_1",
        ArticulationType.PRISMATIC,
        parent=support,
        child=stage_1,
        origin=Origin(xyz=(SUPPORT_TO_STAGE_1_X, 0.0, BOOM_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=450.0,
            velocity=0.45,
            lower=0.0,
            upper=SLIDE_1_UPPER,
        ),
    )
    model.articulation(
        "stage_1_to_stage_2",
        ArticulationType.PRISMATIC,
        parent=stage_1,
        child=stage_2,
        origin=Origin(xyz=(STAGE_1_TO_STAGE_2_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=350.0,
            velocity=0.55,
            lower=0.0,
            upper=SLIDE_2_UPPER,
        ),
    )
    model.articulation(
        "stage_2_to_stage_3",
        ArticulationType.PRISMATIC,
        parent=stage_2,
        child=stage_3,
        origin=Origin(xyz=(STAGE_2_TO_STAGE_3_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=0.65,
            lower=0.0,
            upper=SLIDE_3_UPPER,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("rear_support")
    stage_1 = object_model.get_part("stage_1")
    stage_2 = object_model.get_part("stage_2")
    stage_3 = object_model.get_part("stage_3")

    slide_1 = object_model.get_articulation("support_to_stage_1")
    slide_2 = object_model.get_articulation("stage_1_to_stage_2")
    slide_3 = object_model.get_articulation("stage_2_to_stage_3")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_overlap(
        support,
        stage_1,
        reason="rear bridge uses zero-clearance slider pads to carry the first telescoping section",
    )
    ctx.allow_overlap(
        stage_1,
        stage_2,
        reason="nested telescoping stages intentionally share zero-clearance bearing contact at rest",
    )
    ctx.allow_overlap(
        stage_2,
        stage_3,
        reason="nested telescoping stages intentionally share zero-clearance bearing contact at rest",
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

    for joint, minimum_travel in (
        (slide_1, 0.40),
        (slide_2, 0.35),
        (slide_3, 0.25),
    ):
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name}_is_prismatic",
            joint.articulation_type == ArticulationType.PRISMATIC,
            f"{joint.name} should be prismatic, got {joint.articulation_type}",
        )
        ctx.check(
            f"{joint.name}_axis_is_positive_x",
            _axis_is_positive_x(joint.axis),
            f"{joint.name} axis should be (1, 0, 0), got {joint.axis}",
        )
        ctx.check(
            f"{joint.name}_forward_limits",
            limits is not None
            and limits.lower == 0.0
            and limits.upper is not None
            and limits.upper >= minimum_travel,
            f"{joint.name} limits should start at 0 and travel at least {minimum_travel} m, got {limits}",
        )

    closed_stage_1_x = ctx.part_world_position(stage_1)[0]
    closed_stage_2_x = ctx.part_world_position(stage_2)[0]
    closed_stage_3_x = ctx.part_world_position(stage_3)[0]

    ctx.check(
        "nested_sections_step_forward_at_rest",
        closed_stage_1_x < closed_stage_2_x < closed_stage_3_x,
        (
            "expected the nested section origins to step forward in X at rest, "
            f"got stage_1={closed_stage_1_x:.3f}, stage_2={closed_stage_2_x:.3f}, "
            f"stage_3={closed_stage_3_x:.3f}"
        ),
    )

    with ctx.pose({slide_1: 0.0, slide_2: 0.0, slide_3: 0.0}):
        ctx.expect_contact(
            support,
            stage_1,
            contact_tol=0.0005,
            name="rear_support_carries_stage_1",
        )
        ctx.expect_within(
            stage_1,
            support,
            axes="yz",
            margin=0.03,
            name="rear_support_bridges_around_stage_1",
        )
        ctx.expect_contact(
            stage_1,
            stage_2,
            contact_tol=0.0005,
            name="stage_1_carries_stage_2",
        )
        ctx.expect_contact(
            stage_2,
            stage_3,
            contact_tol=0.0005,
            name="stage_2_carries_stage_3",
        )
        ctx.expect_within(
            stage_2,
            stage_1,
            axes="yz",
            margin=0.003,
            name="stage_2_stays_within_stage_1_section",
        )
        ctx.expect_within(
            stage_3,
            stage_2,
            axes="yz",
            margin=0.003,
            name="stage_3_stays_within_stage_2_section",
        )
        ctx.expect_origin_gap(
            stage_2,
            stage_1,
            axis="x",
            min_gap=0.23,
            max_gap=0.25,
            name="stage_2_origin_is_nested_ahead_of_stage_1",
        )
        ctx.expect_origin_gap(
            stage_3,
            stage_2,
            axis="x",
            min_gap=0.21,
            max_gap=0.23,
            name="stage_3_origin_is_nested_ahead_of_stage_2",
        )

    with ctx.pose({slide_1: SLIDE_1_UPPER, slide_2: SLIDE_2_UPPER, slide_3: SLIDE_3_UPPER}):
        extended_stage_1_x = ctx.part_world_position(stage_1)[0]
        extended_stage_2_x = ctx.part_world_position(stage_2)[0]
        extended_stage_3_x = ctx.part_world_position(stage_3)[0]

        ctx.expect_within(
            stage_2,
            stage_1,
            axes="yz",
            margin=0.003,
            name="stage_2_remains_coaxial_when_extended",
        )
        ctx.expect_within(
            stage_3,
            stage_2,
            axes="yz",
            margin=0.003,
            name="stage_3_remains_coaxial_when_extended",
        )
        ctx.check(
            "stage_1_extends_forward",
            extended_stage_1_x > closed_stage_1_x + 0.40,
            f"stage_1 moved from {closed_stage_1_x:.3f} to {extended_stage_1_x:.3f}",
        )
        ctx.check(
            "stage_2_extends_forward",
            extended_stage_2_x > closed_stage_2_x + 0.75,
            f"stage_2 moved from {closed_stage_2_x:.3f} to {extended_stage_2_x:.3f}",
        )
        ctx.check(
            "stage_3_extends_farthest",
            extended_stage_3_x > closed_stage_3_x + 1.10
            and extended_stage_1_x < extended_stage_2_x < extended_stage_3_x,
            (
                "expected the fully extended serial stages to telescope forward in order, "
                f"got stage_1={extended_stage_1_x:.3f}, "
                f"stage_2={extended_stage_2_x:.3f}, stage_3={extended_stage_3_x:.3f}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
