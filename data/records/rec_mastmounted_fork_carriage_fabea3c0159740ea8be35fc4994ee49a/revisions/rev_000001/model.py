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


FRAME_NAME = "mast_frame"
CARRIAGE_NAME = "fork_carriage"
LIFT_JOINT_NAME = "mast_to_carriage"

def _add_box(part, name: str, size: tuple[float, float, float], xyz: tuple[float, float, float], material: str) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_backed_lift_mast")

    model.material("frame_blue", color=(0.29, 0.45, 0.56))
    model.material("carriage_charcoal", color=(0.18, 0.18, 0.20))
    model.material("fork_steel", color=(0.24, 0.24, 0.26))

    frame = model.part(FRAME_NAME)
    _add_box(frame, "back_plate", (0.025, 0.86, 2.18), (-0.025, 0.0, 1.09), "frame_blue")
    _add_box(frame, "left_rail", (0.07, 0.095, 1.86), (0.0225, -0.29, 1.10), "frame_blue")
    _add_box(frame, "right_rail", (0.07, 0.095, 1.86), (0.0225, 0.29, 1.10), "frame_blue")
    _add_box(frame, "lower_header", (0.07, 0.58, 0.10), (0.0225, 0.0, 0.20), "frame_blue")
    _add_box(frame, "mid_header", (0.055, 0.45, 0.08), (0.015, 0.0, 1.08), "frame_blue")
    _add_box(frame, "upper_header", (0.07, 0.58, 0.10), (0.0225, 0.0, 2.00), "frame_blue")
    _add_box(frame, "left_wall_tab_low", (0.06, 0.14, 0.10), (-0.055, -0.25, 0.38), "frame_blue")
    _add_box(frame, "right_wall_tab_low", (0.06, 0.14, 0.10), (-0.055, 0.25, 0.38), "frame_blue")
    _add_box(frame, "left_wall_tab_high", (0.06, 0.14, 0.10), (-0.055, -0.25, 1.80), "frame_blue")
    _add_box(frame, "right_wall_tab_high", (0.06, 0.14, 0.10), (-0.055, 0.25, 1.80), "frame_blue")

    carriage = model.part(CARRIAGE_NAME)
    _add_box(carriage, "left_pad", (0.02, 0.07, 0.52), (0.0675, -0.29, 0.46), "carriage_charcoal")
    _add_box(carriage, "right_pad", (0.02, 0.07, 0.52), (0.0675, 0.29, 0.46), "carriage_charcoal")
    _add_box(carriage, "lower_tie", (0.05, 0.56, 0.08), (0.0925, 0.0, 0.28), "carriage_charcoal")
    _add_box(carriage, "upper_tie", (0.05, 0.56, 0.06), (0.0925, 0.0, 0.66), "carriage_charcoal")
    _add_box(carriage, "main_plate", (0.03, 0.62, 0.42), (0.118, 0.0, 0.48), "carriage_charcoal")
    _add_box(carriage, "lower_plate_lip", (0.03, 0.68, 0.06), (0.118, 0.0, 0.26), "carriage_charcoal")

    for side_name, y in (("left", -0.18), ("right", 0.18)):
        _add_box(carriage, f"{side_name}_fork_shank", (0.05, 0.11, 0.28), (0.12, y, 0.20), "fork_steel")
        _add_box(carriage, f"{side_name}_fork_heel", (0.10, 0.11, 0.10), (0.145, y, 0.065), "fork_steel")
        _add_box(carriage, f"{side_name}_fork_tine", (0.74, 0.11, 0.045), (0.49, y, 0.0225), "fork_steel")

    _add_box(carriage, "left_guard_post", (0.025, 0.025, 0.30), (0.118, -0.18, 0.835), "carriage_charcoal")
    _add_box(carriage, "right_guard_post", (0.025, 0.025, 0.30), (0.118, 0.18, 0.835), "carriage_charcoal")
    _add_box(carriage, "guard_mid_bar", (0.018, 0.34, 0.018), (0.118, 0.0, 0.86), "carriage_charcoal")
    _add_box(carriage, "guard_top_bar", (0.025, 0.42, 0.025), (0.118, 0.0, 0.973), "carriage_charcoal")

    model.articulation(
        LIFT_JOINT_NAME,
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4000.0,
            velocity=0.35,
            lower=0.0,
            upper=1.05,
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

    part_names = {part.name for part in object_model.parts}
    articulation_names = {articulation.name for articulation in object_model.articulations}

    ctx.check(
        "required_parts_present",
        {FRAME_NAME, CARRIAGE_NAME}.issubset(part_names),
        f"present parts: {sorted(part_names)}",
    )
    ctx.check(
        "lift_joint_present",
        LIFT_JOINT_NAME in articulation_names,
        f"present articulations: {sorted(articulation_names)}",
    )

    if not (
        {FRAME_NAME, CARRIAGE_NAME}.issubset(part_names)
        and LIFT_JOINT_NAME in articulation_names
    ):
        return ctx.report()

    frame = object_model.get_part(FRAME_NAME)
    carriage = object_model.get_part(CARRIAGE_NAME)
    lift_joint = object_model.get_articulation(LIFT_JOINT_NAME)
    limits = lift_joint.motion_limits

    ctx.check(
        "lift_joint_is_prismatic",
        lift_joint.joint_type == ArticulationType.PRISMATIC,
        f"joint type was {lift_joint.joint_type!r}",
    )
    ctx.check(
        "lift_axis_is_vertical",
        tuple(round(v, 6) for v in lift_joint.axis) == (0.0, 0.0, 1.0),
        f"axis was {lift_joint.axis!r}",
    )
    ctx.check(
        "lift_stroke_is_realistic",
        limits is not None and limits.lower == 0.0 and 1.0 <= limits.upper <= 1.3,
        f"limits were {limits!r}",
    )

    ctx.expect_contact(
        carriage,
        frame,
        name="carriage_guides_touch_frame_at_rest",
    )
    ctx.expect_within(
        carriage,
        frame,
        axes="y",
        margin=0.02,
        name="carriage_stays_within_frame_width",
    )

    with ctx.pose({lift_joint: limits.upper if limits is not None and limits.upper is not None else 1.15}):
        ctx.expect_contact(
            carriage,
            frame,
            name="carriage_guides_touch_frame_at_top",
        )

        frame_aabb = ctx.part_world_aabb(frame)
        carriage_aabb = ctx.part_world_aabb(carriage)
        if frame_aabb is not None and carriage_aabb is not None:
            top_clearance = frame_aabb[1][2] - carriage_aabb[1][2]
            ctx.check(
                "top_clearance_at_full_raise",
                top_clearance >= 0.12,
                f"top clearance was {top_clearance:.4f} m",
            )

    with ctx.pose({lift_joint: 0.0}):
        low_pos = ctx.part_world_position(carriage)
        low_aabb = ctx.part_world_aabb(carriage)
        frame_aabb = ctx.part_world_aabb(frame)

    with ctx.pose({lift_joint: limits.upper if limits is not None and limits.upper is not None else 1.15}):
        high_pos = ctx.part_world_position(carriage)
        high_aabb = ctx.part_world_aabb(carriage)

    if low_pos is not None and high_pos is not None:
        travel = high_pos[2] - low_pos[2]
        ctx.check(
            "carriage_moves_upward_through_mast",
            travel >= 1.0,
            f"carriage vertical travel was {travel:.4f} m",
        )

    if frame_aabb is not None and low_aabb is not None:
        front_projection = low_aabb[1][0] - frame_aabb[1][0]
        ctx.check(
            "forks_project_forward_of_mast",
            front_projection >= 0.75,
            f"forward projection beyond mast was {front_projection:.4f} m",
        )

    if high_aabb is not None and low_aabb is not None:
        ctx.check(
            "forks_remain_level_during_lift",
            abs(high_aabb[0][2] - low_aabb[0][2] - (high_pos[2] - low_pos[2])) <= 1e-6
            if high_pos is not None and low_pos is not None
            else False,
            "fork tine bottom should translate vertically without changing local pitch",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
