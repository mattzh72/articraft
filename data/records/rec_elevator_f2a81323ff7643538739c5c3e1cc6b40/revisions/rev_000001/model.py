from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _box(part, name, size, xyz, *, material=None, rpy=(0.0, 0.0, 0.0)):
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _add_gate_leaf(part, *, side: str, width: float, height: float, material) -> None:
    stile = 0.03
    rail = 0.03
    thickness = 0.03
    bar = 0.02

    if side == "left":
        x0 = 0.0
        x1 = width
    else:
        x0 = -width
        x1 = 0.0

    mid_x = 0.5 * (x0 + x1)
    quarter_x = x0 + 0.33 * (x1 - x0)
    three_quarter_x = x0 + 0.67 * (x1 - x0)

    _box(
        part,
        "outer_stile",
        (stile, thickness, height),
        (x0 + 0.5 * stile if side == "left" else x1 - 0.5 * stile, 0.015, 0.5 * height),
        material=material,
    )
    _box(
        part,
        "inner_stile",
        (stile, thickness, height),
        (x1 - 0.5 * stile if side == "left" else x0 + 0.5 * stile, 0.015, 0.5 * height),
        material=material,
    )
    _box(part, "top_rail", (width, thickness, rail), (mid_x, 0.015, height - 0.5 * rail), material=material)
    _box(part, "bottom_rail", (width, thickness, rail), (mid_x, 0.015, 0.5 * rail), material=material)
    _box(part, "mid_rail", (width, thickness, rail), (mid_x, 0.015, 0.56), material=material)
    _box(part, "lower_guard_rail", (width, thickness, rail), (mid_x, 0.015, 0.30), material=material)
    _box(part, "intermediate_bar_1", (bar, thickness, height - 0.10), (quarter_x, 0.015, 0.5 * (height - 0.10) + 0.05), material=material)
    _box(part, "intermediate_bar_2", (bar, thickness, height - 0.10), (three_quarter_x, 0.015, 0.5 * (height - 0.10) + 0.05), material=material)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="freight_goods_elevator")

    steel_dark = model.material("steel_dark", rgba=(0.24, 0.26, 0.29, 1.0))
    rail_steel = model.material("rail_steel", rgba=(0.40, 0.43, 0.46, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.88, 0.74, 0.14, 1.0))
    deck_grey = model.material("deck_grey", rgba=(0.33, 0.34, 0.36, 1.0))
    counterweight_grey = model.material("counterweight_grey", rgba=(0.42, 0.43, 0.45, 1.0))

    frame = model.part("tower_frame")
    _box(frame, "left_back_wall", (0.26, 0.04, 3.20), (-0.98, -0.09, 1.60), material=steel_dark)
    _box(frame, "left_outer_wall", (0.04, 0.22, 3.20), (-1.09, 0.00, 1.60), material=steel_dark)
    _box(frame, "left_inner_wall", (0.04, 0.22, 3.20), (-0.87, 0.00, 1.60), material=steel_dark)
    _box(frame, "right_column", (0.22, 0.22, 3.20), (0.96, 0.00, 1.60), material=steel_dark)
    _box(frame, "top_crossbeam", (2.18, 0.22, 0.14), (0.00, 0.00, 3.13), material=steel_dark)
    _box(frame, "base_crossbeam", (2.18, 0.22, 0.14), (0.00, 0.00, 0.07), material=steel_dark)
    _box(frame, "left_platform_rail", (0.02, 0.10, 2.90), (-0.84, 0.00, 1.60), material=rail_steel)
    _box(frame, "right_platform_rail", (0.02, 0.10, 2.90), (0.84, 0.00, 1.60), material=rail_steel)
    _box(frame, "cw_outer_track", (0.02, 0.10, 2.90), (-1.06, 0.02, 1.60), material=rail_steel)
    _box(frame, "cw_inner_track", (0.02, 0.10, 2.90), (-0.90, 0.02, 1.60), material=rail_steel)

    platform = model.part("platform")
    _box(platform, "left_side_beam", (0.08, 1.34, 0.12), (-0.73, 0.00, 0.06), material=deck_grey)
    _box(platform, "right_side_beam", (0.08, 1.34, 0.12), (0.73, 0.00, 0.06), material=deck_grey)
    _box(platform, "rear_beam", (1.54, 0.08, 0.12), (0.00, -0.63, 0.06), material=deck_grey)
    _box(platform, "front_beam", (1.54, 0.08, 0.12), (0.00, 0.63, 0.06), material=deck_grey)
    _box(platform, "deck_plate", (1.48, 1.24, 0.03), (0.00, 0.00, 0.135), material=deck_grey)
    _box(platform, "left_toeboard", (0.03, 1.30, 0.16), (-0.745, 0.00, 0.23), material=safety_yellow)
    _box(platform, "right_toeboard", (0.03, 1.30, 0.16), (0.745, 0.00, 0.23), material=safety_yellow)
    _box(platform, "rear_toeboard", (1.48, 0.03, 0.16), (0.00, -0.645, 0.23), material=safety_yellow)

    for name, x, y in (
        ("left_rear_post", -0.76, -0.63),
        ("right_rear_post", 0.76, -0.63),
        ("left_mid_post", -0.76, 0.00),
        ("right_mid_post", 0.76, 0.00),
    ):
        _box(platform, name, (0.06, 0.06, 1.25), (x, y, 0.775), material=safety_yellow)

    _box(platform, "left_gate_post", (0.08, 0.10, 1.25), (-0.76, 0.69, 0.775), material=safety_yellow)
    _box(platform, "right_gate_post", (0.08, 0.10, 1.25), (0.76, 0.69, 0.775), material=safety_yellow)
    _box(platform, "left_mid_rail", (0.04, 1.26, 0.04), (-0.75, 0.00, 0.82), material=safety_yellow)
    _box(platform, "right_mid_rail", (0.04, 1.26, 0.04), (0.75, 0.00, 0.82), material=safety_yellow)
    _box(platform, "left_top_rail", (0.04, 1.26, 0.04), (-0.75, 0.00, 1.34), material=safety_yellow)
    _box(platform, "right_top_rail", (0.04, 1.26, 0.04), (0.75, 0.00, 1.34), material=safety_yellow)
    _box(platform, "rear_mid_rail", (1.46, 0.04, 0.04), (0.00, -0.63, 0.82), material=safety_yellow)
    _box(platform, "rear_top_rail", (1.46, 0.04, 0.04), (0.00, -0.63, 1.34), material=safety_yellow)
    _box(platform, "front_header", (1.44, 0.04, 0.04), (0.00, 0.69, 1.34), material=safety_yellow)
    _box(platform, "left_lower_guide_shoe", (0.04, 0.12, 0.10), (-0.81, 0.00, 0.55), material=rail_steel)
    _box(platform, "right_lower_guide_shoe", (0.04, 0.12, 0.10), (0.81, 0.00, 0.55), material=rail_steel)
    _box(platform, "left_upper_guide_shoe", (0.04, 0.12, 0.10), (-0.81, 0.00, 1.16), material=rail_steel)
    _box(platform, "right_upper_guide_shoe", (0.04, 0.12, 0.10), (0.81, 0.00, 1.16), material=rail_steel)

    left_gate = model.part("left_gate_leaf")
    _add_gate_leaf(left_gate, side="left", width=0.71, height=1.10, material=safety_yellow)

    right_gate = model.part("right_gate_leaf")
    _add_gate_leaf(right_gate, side="right", width=0.71, height=1.10, material=safety_yellow)

    counterweight = model.part("counterweight")
    _box(counterweight, "weight_block", (0.10, 0.14, 0.90), (0.00, 0.00, 0.00), material=counterweight_grey)
    _box(counterweight, "cw_left_shoe", (0.02, 0.10, 0.70), (-0.06, 0.00, 0.00), material=rail_steel)
    _box(counterweight, "cw_right_shoe", (0.02, 0.10, 0.70), (0.06, 0.00, 0.00), material=rail_steel)
    _box(counterweight, "cw_top_tie", (0.14, 0.04, 0.06), (0.00, 0.00, 0.42), material=counterweight_grey)

    platform_lift = model.articulation(
        "tower_to_platform",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=platform,
        origin=Origin(xyz=(0.00, 0.00, 0.16)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18000.0, velocity=0.55, lower=0.0, upper=1.40),
    )

    counterweight_slide = model.articulation(
        "tower_to_counterweight",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=counterweight,
        origin=Origin(xyz=(-0.98, 0.02, 0.85)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12000.0, velocity=0.70, lower=0.0, upper=1.70),
    )

    left_gate_hinge = model.articulation(
        "platform_to_left_gate",
        ArticulationType.REVOLUTE,
        parent=platform,
        child=left_gate,
        origin=Origin(xyz=(-0.72, 0.725, 0.15)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.8, lower=0.0, upper=1.45),
    )

    right_gate_hinge = model.articulation(
        "platform_to_right_gate",
        ArticulationType.REVOLUTE,
        parent=platform,
        child=right_gate,
        origin=Origin(xyz=(0.72, 0.725, 0.15)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.8, lower=0.0, upper=1.45),
    )

    model.meta["primary_articulations"] = (
        platform_lift.name,
        counterweight_slide.name,
        left_gate_hinge.name,
        right_gate_hinge.name,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("tower_frame")
    platform = object_model.get_part("platform")
    left_gate = object_model.get_part("left_gate_leaf")
    right_gate = object_model.get_part("right_gate_leaf")
    counterweight = object_model.get_part("counterweight")
    platform_lift = object_model.get_articulation("tower_to_platform")
    counterweight_slide = object_model.get_articulation("tower_to_counterweight")
    left_gate_hinge = object_model.get_articulation("platform_to_left_gate")
    right_gate_hinge = object_model.get_articulation("platform_to_right_gate")

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
        platform,
        frame,
        elem_a="left_lower_guide_shoe",
        elem_b="left_platform_rail",
        name="left lower guide shoe contacts left rail",
    )
    ctx.expect_contact(
        platform,
        frame,
        elem_a="right_lower_guide_shoe",
        elem_b="right_platform_rail",
        name="right lower guide shoe contacts right rail",
    )
    ctx.expect_contact(
        counterweight,
        frame,
        elem_a="cw_left_shoe",
        elem_b="cw_outer_track",
        name="counterweight outer shoe contacts guide track",
    )
    ctx.expect_contact(
        counterweight,
        frame,
        elem_a="cw_right_shoe",
        elem_b="cw_inner_track",
        name="counterweight inner shoe contacts guide track",
    )
    ctx.expect_contact(
        left_gate,
        platform,
        elem_a="outer_stile",
        elem_b="left_gate_post",
        name="left gate leaf hangs from left post in the closed pose",
    )
    ctx.expect_contact(
        right_gate,
        platform,
        elem_a="outer_stile",
        elem_b="right_gate_post",
        name="right gate leaf hangs from right post in the closed pose",
    )
    ctx.expect_gap(
        right_gate,
        left_gate,
        axis="x",
        min_gap=0.0,
        max_gap=0.03,
        name="closed gate leaves meet near the center without overlap",
    )

    platform_rest = ctx.part_world_position(platform)
    counterweight_rest = ctx.part_world_position(counterweight)
    left_gate_closed_aabb = ctx.part_world_aabb(left_gate)
    right_gate_closed_aabb = ctx.part_world_aabb(right_gate)

    with ctx.pose({platform_lift: platform_lift.motion_limits.upper}):
        platform_raised = ctx.part_world_position(platform)

    with ctx.pose({counterweight_slide: counterweight_slide.motion_limits.upper}):
        counterweight_raised = ctx.part_world_position(counterweight)

    with ctx.pose(
        {
            left_gate_hinge: left_gate_hinge.motion_limits.upper,
            right_gate_hinge: right_gate_hinge.motion_limits.upper,
        }
    ):
        left_gate_open_aabb = ctx.part_world_aabb(left_gate)
        right_gate_open_aabb = ctx.part_world_aabb(right_gate)

    ctx.check(
        "platform rises vertically on the guide rails",
        platform_rest is not None
        and platform_raised is not None
        and platform_raised[2] > platform_rest[2] + 1.0,
        details=f"rest={platform_rest}, raised={platform_raised}",
    )
    ctx.check(
        "counterweight slides upward in its mast guide",
        counterweight_rest is not None
        and counterweight_raised is not None
        and counterweight_raised[2] > counterweight_rest[2] + 1.2,
        details=f"rest={counterweight_rest}, raised={counterweight_raised}",
    )
    ctx.check(
        "left gate leaf swings outward from the platform front",
        left_gate_closed_aabb is not None
        and left_gate_open_aabb is not None
        and left_gate_open_aabb[1][1] > left_gate_closed_aabb[1][1] + 0.45,
        details=f"closed={left_gate_closed_aabb}, open={left_gate_open_aabb}",
    )
    ctx.check(
        "right gate leaf swings outward from the platform front",
        right_gate_closed_aabb is not None
        and right_gate_open_aabb is not None
        and right_gate_open_aabb[1][1] > right_gate_closed_aabb[1][1] + 0.45,
        details=f"closed={right_gate_closed_aabb}, open={right_gate_open_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
