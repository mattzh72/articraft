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


def _y_cylinder(radius: float, length: float) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length * 0.5, both=True)


def _make_support() -> cq.Workplane:
    base = cq.Workplane("XY").box(0.24, 0.16, 0.02).translate((-0.11, 0.0, -0.22))
    rear_column = cq.Workplane("XY").box(0.05, 0.08, 0.16).translate((-0.145, 0.0, -0.13))
    arm = cq.Workplane("XY").box(0.11, 0.05, 0.04).translate((-0.065, 0.0, -0.03))
    yoke_outer = cq.Workplane("XY").box(0.03, 0.06, 0.06).translate((-0.015, 0.0, 0.0))
    yoke_slot = cq.Workplane("XY").box(0.02, 0.034, 0.04).translate((-0.01, 0.0, 0.0))
    yoke = yoke_outer.cut(yoke_slot)

    return base.union(rear_column).union(arm).union(yoke)


def _make_frame() -> cq.Workplane:
    rear_lug = cq.Workplane("XY").box(0.03, 0.03, 0.03).translate((0.015, 0.0, 0.0))
    spine = cq.Workplane("XY").box(0.07, 0.02, 0.026).translate((0.035, 0.0, 0.0))
    left_shoulder = cq.Workplane("XY").box(0.04, 0.016, 0.03).translate((0.07, 0.018, 0.0))
    right_shoulder = cq.Workplane("XY").box(0.04, 0.016, 0.03).translate((0.07, -0.018, 0.0))
    left_rail = cq.Workplane("XY").box(0.21, 0.012, 0.03).translate((0.195, 0.024, 0.0))
    right_rail = cq.Workplane("XY").box(0.21, 0.012, 0.03).translate((0.195, -0.024, 0.0))

    return (
        rear_lug.union(spine)
        .union(left_shoulder)
        .union(right_shoulder)
        .union(left_rail)
        .union(right_rail)
    )


def _make_slider() -> cq.Workplane:
    carriage = cq.Workplane("XY").box(0.08, 0.028, 0.022).translate((0.04, 0.0, 0.0))
    beam = cq.Workplane("XY").box(0.13, 0.022, 0.018).translate((0.145, 0.0, 0.0))
    tip = cq.Workplane("XY").box(0.04, 0.034, 0.028).translate((0.23, 0.0, 0.0))
    return carriage.union(beam).union(tip)


def _make_output() -> cq.Workplane:
    root = cq.Workplane("XY").box(0.028, 0.022, 0.022).translate((0.014, 0.0, 0.0))
    head = cq.Workplane("XY").box(0.03, 0.02, 0.028).translate((0.043, 0.0, 0.0))
    tool = cq.Workplane("XY").box(0.038, 0.012, 0.02).translate((0.077, 0.0, 0.0))
    return root.union(head).union(tool)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_backed_pivot_frame")

    painted_dark = model.material("painted_dark", color=(0.22, 0.24, 0.26, 1.0))
    anodized_gray = model.material("anodized_gray", color=(0.54, 0.57, 0.60, 1.0))
    satin_metal = model.material("satin_metal", color=(0.73, 0.75, 0.77, 1.0))

    support = model.part("rear_support")
    support.visual(
        Box((0.24, 0.16, 0.02)),
        origin=Origin(xyz=(-0.11, 0.0, -0.22)),
        material=painted_dark,
        name="base_plate",
    )
    support.visual(
        Box((0.05, 0.08, 0.16)),
        origin=Origin(xyz=(-0.145, 0.0, -0.13)),
        material=painted_dark,
        name="rear_column",
    )
    support.visual(
        Box((0.11, 0.05, 0.04)),
        origin=Origin(xyz=(-0.065, 0.0, -0.03)),
        material=painted_dark,
        name="support_arm",
    )
    support.visual(
        Box((0.012, 0.074, 0.06)),
        origin=Origin(xyz=(-0.034, 0.0, 0.0)),
        material=painted_dark,
        name="yoke_bridge",
    )
    support.visual(
        Box((0.04, 0.012, 0.06)),
        origin=Origin(xyz=(-0.02, 0.031, 0.0)),
        material=painted_dark,
        name="left_yoke_cheek",
    )
    support.visual(
        Box((0.04, 0.012, 0.06)),
        origin=Origin(xyz=(-0.02, -0.031, 0.0)),
        material=painted_dark,
        name="right_yoke_cheek",
    )

    frame = model.part("pivot_frame")
    frame.visual(
        Box((0.04, 0.06, 0.03)),
        origin=Origin(xyz=(0.02, 0.0, 0.0)),
        material=anodized_gray,
        name="rear_lug",
    )
    frame.visual(
        Box((0.20, 0.008, 0.03)),
        origin=Origin(xyz=(0.14, 0.018, 0.0)),
        material=anodized_gray,
        name="left_rail",
    )
    frame.visual(
        Box((0.20, 0.008, 0.03)),
        origin=Origin(xyz=(0.14, -0.018, 0.0)),
        material=anodized_gray,
        name="right_rail",
    )

    slider = model.part("slider_member")
    slider.visual(
        Box((0.06, 0.028, 0.022)),
        origin=Origin(xyz=(0.03, 0.0, 0.0)),
        material=satin_metal,
        name="carriage",
    )
    slider.visual(
        Box((0.16, 0.018, 0.018)),
        origin=Origin(xyz=(0.14, 0.0, 0.0)),
        material=satin_metal,
        name="slider_beam",
    )

    output = model.part("rotating_output")
    output.visual(
        Box((0.03, 0.028, 0.022)),
        origin=Origin(xyz=(0.015, 0.0, 0.0)),
        material=painted_dark,
        name="wrist_root",
    )
    output.visual(
        Box((0.04, 0.018, 0.028)),
        origin=Origin(xyz=(0.05, 0.0, 0.0)),
        material=painted_dark,
        name="wrist_head",
    )
    output.visual(
        Box((0.05, 0.012, 0.016)),
        origin=Origin(xyz=(0.095, 0.0, 0.0)),
        material=painted_dark,
        name="tool_pad",
    )

    model.articulation(
        "support_to_frame",
        ArticulationType.REVOLUTE,
        parent=support,
        child=frame,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.4,
            lower=-0.35,
            upper=1.15,
        ),
    )

    model.articulation(
        "frame_to_slider",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=slider,
        origin=Origin(xyz=(0.04, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.25,
            lower=0.0,
            upper=0.12,
        ),
    )

    model.articulation(
        "slider_to_output",
        ArticulationType.REVOLUTE,
        parent=slider,
        child=output,
        origin=Origin(xyz=(0.22, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=-0.9,
            upper=0.9,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("rear_support")
    frame = object_model.get_part("pivot_frame")
    slider = object_model.get_part("slider_member")
    output = object_model.get_part("rotating_output")

    support_to_frame = object_model.get_articulation("support_to_frame")
    frame_to_slider = object_model.get_articulation("frame_to_slider")
    slider_to_output = object_model.get_articulation("slider_to_output")

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
        frame,
        support,
        contact_tol=5e-4,
        name="frame_is_carried_by_support_hinge_faces",
    )
    ctx.expect_contact(
        slider,
        frame,
        contact_tol=5e-4,
        name="slider_is_supported_by_frame_rails",
    )
    ctx.expect_contact(
        output,
        slider,
        contact_tol=5e-4,
        name="output_is_supported_by_slider_clevis",
    )

    with ctx.pose({frame_to_slider: frame_to_slider.motion_limits.upper}):
        ctx.expect_within(
            slider,
            frame,
            axes="yz",
            margin=0.0,
            name="slider_stays_within_frame_section",
        )

    with ctx.pose({support_to_frame: 0.0}):
        frame_closed = ctx.part_world_aabb(frame)
    with ctx.pose({support_to_frame: 0.9}):
        frame_open = ctx.part_world_aabb(frame)

    frame_opens_upward = (
        frame_closed is not None
        and frame_open is not None
        and frame_open[1][2] > frame_closed[1][2] + 0.12
    )
    ctx.check(
        "frame_opens_upward",
        frame_opens_upward,
        details=f"closed={frame_closed}, open={frame_open}",
    )

    with ctx.pose({support_to_frame: 0.0, frame_to_slider: 0.0}):
        slider_retracted_pos = ctx.part_world_position(slider)
    with ctx.pose({support_to_frame: 0.0, frame_to_slider: frame_to_slider.motion_limits.upper}):
        slider_extended_pos = ctx.part_world_position(slider)

    slider_extends_forward = (
        slider_retracted_pos is not None
        and slider_extended_pos is not None
        and slider_extended_pos[0] > slider_retracted_pos[0] + 0.10
        and abs(slider_extended_pos[2] - slider_retracted_pos[2]) < 1e-6
    )
    ctx.check(
        "slider_extends_forward_along_frame_axis",
        slider_extends_forward,
        details=f"retracted={slider_retracted_pos}, extended={slider_extended_pos}",
    )

    with ctx.pose(
        {
            support_to_frame: 0.0,
            frame_to_slider: frame_to_slider.motion_limits.upper,
            slider_to_output: 0.0,
        }
    ):
        output_neutral = ctx.part_world_aabb(output)
    with ctx.pose(
        {
            support_to_frame: 0.0,
            frame_to_slider: frame_to_slider.motion_limits.upper,
            slider_to_output: slider_to_output.motion_limits.upper,
        }
    ):
        output_pitched = ctx.part_world_aabb(output)

    wrist_changes_tip_height = (
        output_neutral is not None
        and output_pitched is not None
        and output_pitched[1][2] > output_neutral[1][2] + 0.03
    )
    ctx.check(
        "wristed_output_rotates_upward",
        wrist_changes_tip_height,
        details=f"neutral={output_neutral}, pitched={output_pitched}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
