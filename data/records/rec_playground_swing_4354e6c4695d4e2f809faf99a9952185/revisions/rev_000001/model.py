from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, pi, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    wire_from_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="playground_swing")

    powder_coat = model.material("powder_coat", rgba=(0.22, 0.25, 0.30, 1.0))
    galvanized = model.material("galvanized", rgba=(0.72, 0.74, 0.76, 1.0))
    seat_wood = model.material("seat_wood", rgba=(0.62, 0.43, 0.22, 1.0))
    safety_red = model.material("safety_red", rgba=(0.78, 0.14, 0.12, 1.0))

    beam = model.part("top_beam")
    beam.visual(
        Cylinder(radius=0.055, length=1.25),
        origin=Origin(xyz=(0.0, 0.0, 0.075), rpy=(0.0, pi / 2.0, 0.0)),
        material=powder_coat,
        name="beam_tube",
    )
    left_frame = wire_from_points(
        [
            (-0.55, -0.42, -1.45),
            (-0.55, 0.0, 0.03),
            (-0.55, 0.42, -1.45),
        ],
        radius=0.024,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.14,
        radial_segments=18,
    )
    right_frame = wire_from_points(
        [
            (0.55, -0.42, -1.45),
            (0.55, 0.0, 0.03),
            (0.55, 0.42, -1.45),
        ],
        radius=0.024,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.14,
        radial_segments=18,
    )
    beam.visual(
        mesh_from_geometry(left_frame, "left_support_frame"),
        material=powder_coat,
        name="left_support_frame",
    )
    beam.visual(
        mesh_from_geometry(right_frame, "right_support_frame"),
        material=powder_coat,
        name="right_support_frame",
    )
    beam.visual(
        Box((0.090, 0.140, 0.080)),
        origin=Origin(xyz=(-0.55, 0.0, 0.020)),
        material=powder_coat,
        name="left_support_collar",
    )
    beam.visual(
        Box((0.090, 0.140, 0.080)),
        origin=Origin(xyz=(0.55, 0.0, 0.020)),
        material=powder_coat,
        name="right_support_collar",
    )

    bracket_x = 0.24
    bracket_y = 0.022
    for side, sign in (("left", -1.0), ("right", 1.0)):
        for ear_name, y_sign in (("front", -1.0), ("rear", 1.0)):
            beam.visual(
                Box((0.040, 0.008, 0.080)),
                origin=Origin(xyz=(sign * bracket_x, y_sign * bracket_y, 0.010)),
                material=powder_coat,
                name=f"{side}_{ear_name}_ear",
            )

    swing = model.part("seat_assembly")
    hanger_x = 0.24
    hanger_height = 0.96
    for side, sign in (("left", -1.0), ("right", 1.0)):
        swing.visual(
            Cylinder(radius=0.018, length=0.070),
            origin=Origin(xyz=(sign * hanger_x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=galvanized,
            name=f"{side}_pivot_sleeve",
        )
        swing.visual(
            Box((0.030, 0.018, hanger_height)),
            origin=Origin(xyz=(sign * hanger_x, 0.0, -hanger_height / 2.0)),
            material=galvanized,
            name=f"{side}_hanger_link",
        )

    seat_side_x = 0.22
    swing.visual(
        Box((0.035, 0.300, 0.090)),
        origin=Origin(xyz=(-seat_side_x, 0.0, -1.005)),
        material=powder_coat,
        name="left_seat_side",
    )
    swing.visual(
        Box((0.035, 0.300, 0.090)),
        origin=Origin(xyz=(seat_side_x, 0.0, -1.005)),
        material=powder_coat,
        name="right_seat_side",
    )

    swing.visual(
        Box((0.410, 0.025, 0.040)),
        origin=Origin(xyz=(0.0, -0.135, -0.982)),
        material=powder_coat,
        name="rear_seat_rail",
    )
    swing.visual(
        Box((0.410, 0.025, 0.040)),
        origin=Origin(xyz=(0.0, 0.135, -0.982)),
        material=powder_coat,
        name="front_seat_rail",
    )
    swing.visual(
        Box((0.055, 0.030, 0.040)),
        origin=Origin(xyz=(-0.245, 0.145, -0.945)),
        material=powder_coat,
        name="left_lap_bar_mount",
    )
    swing.visual(
        Box((0.055, 0.030, 0.040)),
        origin=Origin(xyz=(0.245, 0.145, -0.945)),
        material=powder_coat,
        name="right_lap_bar_mount",
    )

    swing.visual(
        Box((0.410, 0.050, 0.018)),
        origin=Origin(xyz=(0.0, -0.105, -0.951)),
        material=seat_wood,
        name="rear_slat",
    )
    swing.visual(
        Box((0.410, 0.050, 0.018)),
        origin=Origin(xyz=(0.0, -0.035, -0.951)),
        material=seat_wood,
        name="mid_rear_slat",
    )
    swing.visual(
        Box((0.410, 0.050, 0.018)),
        origin=Origin(xyz=(0.0, 0.035, -0.951)),
        material=seat_wood,
        name="mid_front_slat",
    )
    swing.visual(
        Box((0.410, 0.050, 0.018)),
        origin=Origin(xyz=(0.0, 0.105, -0.951)),
        material=seat_wood,
        name="front_slat",
    )

    model.articulation(
        "beam_to_seat_swing",
        ArticulationType.REVOLUTE,
        parent=beam,
        child=swing,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=1.6,
            lower=-0.70,
            upper=0.70,
        ),
    )

    lap_bar = model.part("lap_bar")
    bar_half_span = 0.19
    bar_forward = 0.17
    bar_rise = 0.09
    arm_length = sqrt(bar_forward * bar_forward + bar_rise * bar_rise)
    arm_pitch = -atan2(bar_forward, bar_rise)

    lap_bar.visual(
        Cylinder(radius=0.014, length=0.055),
        origin=Origin(xyz=(-bar_half_span, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=galvanized,
        name="left_hinge_barrel",
    )
    lap_bar.visual(
        Cylinder(radius=0.014, length=0.055),
        origin=Origin(xyz=(bar_half_span, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=galvanized,
        name="right_hinge_barrel",
    )
    lap_bar.visual(
        Cylinder(radius=0.011, length=arm_length),
        origin=Origin(
            xyz=(-bar_half_span, bar_forward / 2.0, bar_rise / 2.0),
            rpy=(arm_pitch, 0.0, 0.0),
        ),
        material=safety_red,
        name="left_bar_arm",
    )
    lap_bar.visual(
        Cylinder(radius=0.011, length=arm_length),
        origin=Origin(
            xyz=(bar_half_span, bar_forward / 2.0, bar_rise / 2.0),
            rpy=(arm_pitch, 0.0, 0.0),
        ),
        material=safety_red,
        name="right_bar_arm",
    )
    lap_bar.visual(
        Cylinder(radius=0.011, length=bar_half_span * 2.0),
        origin=Origin(xyz=(0.0, bar_forward, bar_rise), rpy=(0.0, pi / 2.0, 0.0)),
        material=safety_red,
        name="front_guard",
    )

    model.articulation(
        "seat_to_lap_bar",
        ArticulationType.REVOLUTE,
        parent=swing,
        child=lap_bar,
        origin=Origin(xyz=(0.0, 0.164, -0.946)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=2.0,
            lower=0.0,
            upper=1.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    beam = object_model.get_part("top_beam")
    swing = object_model.get_part("seat_assembly")
    lap_bar = object_model.get_part("lap_bar")
    swing_joint = object_model.get_articulation("beam_to_seat_swing")
    lap_bar_joint = object_model.get_articulation("seat_to_lap_bar")

    beam_visual_names = {visual.name for visual in beam.visuals}
    swing_visual_names = {visual.name for visual in swing.visuals}
    lap_bar_visual_names = {visual.name for visual in lap_bar.visuals}

    ctx.check(
        "beam visuals present",
        {"beam_tube", "left_front_ear", "right_rear_ear"}.issubset(beam_visual_names),
        details=f"beam visuals={sorted(beam_visual_names)}",
    )
    ctx.check(
        "seat assembly visuals present",
        {
            "left_hanger_link",
            "right_hanger_link",
            "left_seat_side",
            "right_seat_side",
            "front_slat",
            "rear_slat",
        }.issubset(swing_visual_names),
        details=f"seat visuals={sorted(swing_visual_names)}",
    )
    ctx.check(
        "lap bar visuals present",
        {
            "left_hinge_barrel",
            "right_hinge_barrel",
            "left_bar_arm",
            "right_bar_arm",
            "front_guard",
        }.issubset(lap_bar_visual_names),
        details=f"lap bar visuals={sorted(lap_bar_visual_names)}",
    )

    ctx.expect_gap(
        beam,
        swing,
        axis="z",
        positive_elem="beam_tube",
        negative_elem="front_slat",
        min_gap=0.92,
        max_gap=1.10,
        name="seat hangs well below the top beam",
    )
    ctx.expect_overlap(
        lap_bar,
        swing,
        axes="x",
        elem_a="front_guard",
        elem_b="front_slat",
        min_overlap=0.30,
        name="lap bar spans the width of the seat front",
    )
    ctx.expect_gap(
        lap_bar,
        swing,
        axis="y",
        positive_elem="front_guard",
        negative_elem="front_slat",
        min_gap=0.14,
        max_gap=0.22,
        name="lap bar sits ahead of the seat front edge",
    )

    rest_front_slat = ctx.part_element_world_aabb(swing, elem="front_slat")
    with ctx.pose({swing_joint: 0.45}):
        swung_front_slat = ctx.part_element_world_aabb(swing, elem="front_slat")
    swing_rest_center = None
    swing_open_center = None
    if rest_front_slat is not None:
        swing_rest_center = tuple(
            (rest_front_slat[0][i] + rest_front_slat[1][i]) / 2.0 for i in range(3)
        )
    if swung_front_slat is not None:
        swing_open_center = tuple(
            (swung_front_slat[0][i] + swung_front_slat[1][i]) / 2.0 for i in range(3)
        )
    ctx.check(
        "seat swings forward on the beam pivots",
        swing_rest_center is not None
        and swing_open_center is not None
        and swing_open_center[1] > swing_rest_center[1] + 0.14
        and swing_open_center[2] > swing_rest_center[2] + 0.05,
        details=f"rest={swing_rest_center}, swung={swing_open_center}",
    )

    rest_guard = ctx.part_element_world_aabb(lap_bar, elem="front_guard")
    with ctx.pose({lap_bar_joint: 1.0}):
        open_guard = ctx.part_element_world_aabb(lap_bar, elem="front_guard")
    guard_rest_center = None
    guard_open_center = None
    if rest_guard is not None:
        guard_rest_center = tuple((rest_guard[0][i] + rest_guard[1][i]) / 2.0 for i in range(3))
    if open_guard is not None:
        guard_open_center = tuple((open_guard[0][i] + open_guard[1][i]) / 2.0 for i in range(3))
    ctx.check(
        "lap bar rotates upward from the seat front corners",
        guard_rest_center is not None
        and guard_open_center is not None
        and guard_open_center[2] > guard_rest_center[2] + 0.08
        and guard_open_center[1] < guard_rest_center[1] - 0.10,
        details=f"closed={guard_rest_center}, open={guard_open_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
