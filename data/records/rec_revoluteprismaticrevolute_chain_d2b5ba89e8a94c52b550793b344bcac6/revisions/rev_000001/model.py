from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


HALF_PI = pi * 0.5


def _center_from_aabb(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_slung_hinge_slider_wrist")

    support_finish = model.material("support_finish", rgba=(0.27, 0.29, 0.31, 1.0))
    frame_finish = model.material("frame_finish", rgba=(0.55, 0.58, 0.62, 1.0))
    slider_finish = model.material("slider_finish", rgba=(0.76, 0.78, 0.80, 1.0))
    wrist_finish = model.material("wrist_finish", rgba=(0.83, 0.46, 0.18, 1.0))

    support = model.part("top_support")
    support.visual(
        Box((0.20, 0.09, 0.016)),
        material=support_finish,
        name="support_plate",
    )
    support.visual(
        Box((0.036, 0.010, 0.032)),
        origin=Origin(xyz=(0.0, 0.024, -0.024)),
        material=support_finish,
        name="left_hanger_lug",
    )
    support.visual(
        Box((0.036, 0.010, 0.032)),
        origin=Origin(xyz=(0.0, -0.024, -0.024)),
        material=support_finish,
        name="right_hanger_lug",
    )
    support.visual(
        Box((0.014, 0.038, 0.024)),
        origin=Origin(xyz=(-0.017, 0.0, -0.020)),
        material=support_finish,
        name="hinge_boss",
    )

    frame = model.part("pivot_frame")
    frame.visual(
        Cylinder(radius=0.010, length=0.038),
        origin=Origin(rpy=(-HALF_PI, 0.0, 0.0)),
        material=frame_finish,
        name="hinge_barrel",
    )
    frame.visual(
        Box((0.022, 0.038, 0.048)),
        origin=Origin(xyz=(0.011, 0.0, -0.024)),
        material=frame_finish,
        name="rear_web",
    )
    frame.visual(
        Box((0.084, 0.005, 0.040)),
        origin=Origin(xyz=(0.060, 0.0165, -0.038)),
        material=frame_finish,
        name="left_side_rail",
    )
    frame.visual(
        Box((0.084, 0.005, 0.040)),
        origin=Origin(xyz=(0.060, -0.0165, -0.038)),
        material=frame_finish,
        name="right_side_rail",
    )
    frame.visual(
        Box((0.052, 0.038, 0.008)),
        origin=Origin(xyz=(0.052, 0.0, -0.014)),
        material=frame_finish,
        name="guide_roof",
    )
    frame.visual(
        Box((0.012, 0.038, 0.008)),
        origin=Origin(xyz=(0.102, 0.0, -0.054)),
        material=frame_finish,
        name="front_bridge",
    )

    slider = model.part("slider_block")
    slider.visual(
        Box((0.034, 0.028, 0.016)),
        origin=Origin(xyz=(0.017, 0.0, 0.0)),
        material=slider_finish,
        name="carriage_body",
    )
    slider.visual(
        Box((0.022, 0.018, 0.006)),
        origin=Origin(xyz=(0.018, 0.0, 0.009)),
        material=slider_finish,
        name="top_rib",
    )
    slider.visual(
        Box((0.020, 0.006, 0.020)),
        origin=Origin(xyz=(0.044, 0.011, 0.0)),
        material=slider_finish,
        name="left_clevis_ear",
    )
    slider.visual(
        Box((0.020, 0.006, 0.020)),
        origin=Origin(xyz=(0.044, -0.011, 0.0)),
        material=slider_finish,
        name="right_clevis_ear",
    )
    slider.visual(
        Box((0.024, 0.020, 0.006)),
        origin=Origin(xyz=(0.018, 0.0, -0.011)),
        material=slider_finish,
        name="underside_skid",
    )

    wrist = model.part("wrist_tab")
    wrist.visual(
        Cylinder(radius=0.006, length=0.016),
        origin=Origin(rpy=(-HALF_PI, 0.0, 0.0)),
        material=wrist_finish,
        name="wrist_barrel",
    )
    wrist.visual(
        Box((0.026, 0.004, 0.028)),
        origin=Origin(xyz=(0.016, 0.0, -0.018)),
        material=wrist_finish,
        name="tab_blade",
    )
    wrist.visual(
        Box((0.010, 0.008, 0.010)),
        origin=Origin(xyz=(0.028, 0.0, -0.031)),
        material=wrist_finish,
        name="toe_pad",
    )

    model.articulation(
        "support_to_frame",
        ArticulationType.REVOLUTE,
        parent=support,
        child=frame,
        origin=Origin(xyz=(0.0, 0.0, -0.024)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=0.0,
            upper=0.70,
        ),
    )
    model.articulation(
        "frame_to_slider",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=slider,
        origin=Origin(xyz=(0.028, 0.0, -0.034)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.20,
            lower=0.0,
            upper=0.035,
        ),
    )
    model.articulation(
        "slider_to_wrist",
        ArticulationType.REVOLUTE,
        parent=slider,
        child=wrist,
        origin=Origin(xyz=(0.044, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-1.10,
            upper=1.10,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("top_support")
    frame = object_model.get_part("pivot_frame")
    slider = object_model.get_part("slider_block")
    wrist = object_model.get_part("wrist_tab")
    frame_joint = object_model.get_articulation("support_to_frame")
    slider_joint = object_model.get_articulation("frame_to_slider")
    wrist_joint = object_model.get_articulation("slider_to_wrist")

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
        contact_tol=1e-6,
        name="frame barrel is supported by top lugs",
    )
    ctx.expect_contact(
        slider,
        frame,
        contact_tol=1e-6,
        name="slider carriage is guided by frame rails",
    )
    ctx.expect_contact(
        wrist,
        slider,
        contact_tol=1e-6,
        name="wrist tab is captured in slider clevis",
    )

    ctx.check(
        "articulation stack uses revolute-prismatic-revolute",
        (
            frame_joint.articulation_type == ArticulationType.REVOLUTE
            and slider_joint.articulation_type == ArticulationType.PRISMATIC
            and wrist_joint.articulation_type == ArticulationType.REVOLUTE
        ),
        details=(
            f"got {[frame_joint.articulation_type, slider_joint.articulation_type, wrist_joint.articulation_type]}"
        ),
    )
    ctx.check(
        "articulation axes match hinge-slider-wrist layout",
        (
            tuple(round(v, 6) for v in frame_joint.axis) == (0.0, 1.0, 0.0)
            and tuple(round(v, 6) for v in slider_joint.axis) == (1.0, 0.0, 0.0)
            and tuple(round(v, 6) for v in wrist_joint.axis) == (0.0, 1.0, 0.0)
        ),
        details=(
            f"frame={frame_joint.axis}, slider={slider_joint.axis}, wrist={wrist_joint.axis}"
        ),
    )

    frame_rest_center = _center_from_aabb(ctx.part_world_aabb(frame))
    slider_rest_pos = ctx.part_world_position(slider)
    wrist_rest_center = _center_from_aabb(ctx.part_world_aabb(wrist))

    with ctx.pose({"support_to_frame": 0.45}):
        ctx.expect_contact(
            frame,
            support,
            contact_tol=1e-6,
            name="frame remains carried in pitched pose",
        )
        frame_swung_center = _center_from_aabb(ctx.part_world_aabb(frame))

    with ctx.pose({"frame_to_slider": 0.030}):
        ctx.expect_contact(
            slider,
            frame,
            contact_tol=1e-6,
            name="slider stays guided when extended",
        )
        slider_extended_pos = ctx.part_world_position(slider)

    with ctx.pose({"slider_to_wrist": 0.80}):
        ctx.expect_contact(
            wrist,
            slider,
            contact_tol=1e-6,
            name="wrist stays pinned while rotated",
        )
        wrist_swung_center = _center_from_aabb(ctx.part_world_aabb(wrist))

    ctx.check(
        "frame hinge moves frame body through an arc",
        (
            frame_rest_center is not None
            and frame_swung_center is not None
            and frame_swung_center[0] < frame_rest_center[0] - 0.010
            and frame_swung_center[2] < frame_rest_center[2] - 0.012
        ),
        details=f"rest={frame_rest_center}, swung={frame_swung_center}",
    )
    ctx.check(
        "slider prismatic motion is a short x-axis stroke",
        (
            slider_rest_pos is not None
            and slider_extended_pos is not None
            and slider_extended_pos[0] - slider_rest_pos[0] > 0.028
            and abs(slider_extended_pos[1] - slider_rest_pos[1]) < 1e-6
            and abs(slider_extended_pos[2] - slider_rest_pos[2]) < 1e-6
        ),
        details=f"rest={slider_rest_pos}, extended={slider_extended_pos}",
    )
    ctx.check(
        "wrist hinge swings the tab tip through x-z space",
        (
            wrist_rest_center is not None
            and wrist_swung_center is not None
            and abs(wrist_swung_center[0] - wrist_rest_center[0]) > 0.004
            and abs(wrist_swung_center[2] - wrist_rest_center[2]) > 0.004
        ),
        details=f"rest={wrist_rest_center}, swung={wrist_swung_center}",
    )

    with ctx.pose(
        {
            "support_to_frame": 0.45,
            "frame_to_slider": 0.030,
            "slider_to_wrist": 0.80,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="no overlaps in deployed pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
