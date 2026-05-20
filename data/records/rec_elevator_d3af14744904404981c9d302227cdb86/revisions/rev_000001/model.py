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


def _box(
    part,
    name: str,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material: str,
):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mine_shaft_cage_elevator")

    model.material("shaft_steel", rgba=(0.24, 0.26, 0.30, 1.0))
    model.material("guide_steel", rgba=(0.32, 0.34, 0.38, 1.0))
    model.material("safety_yellow", rgba=(0.84, 0.72, 0.16, 1.0))
    model.material("gate_metal", rgba=(0.68, 0.70, 0.72, 1.0))
    model.material("floor_dark", rgba=(0.18, 0.18, 0.19, 1.0))

    shaft = model.part("shaft_frame")
    cage = model.part("cage")
    gate = model.part("gate")

    shaft_height = 4.60
    shaft_post = 0.09
    shaft_half_width = 1.00
    shaft_half_depth = 0.80
    guide_rail_x = 0.84
    guide_rail_y = -0.60
    beam_z_bottom = shaft_post / 2.0
    beam_z_top = shaft_height - shaft_post / 2.0

    # Shaft support frame and fixed guide rails.
    for x, y, name in (
        (-shaft_half_width, shaft_half_depth, "left_front_post"),
        (shaft_half_width, shaft_half_depth, "right_front_post"),
        (-shaft_half_width, -shaft_half_depth, "left_rear_post"),
        (shaft_half_width, -shaft_half_depth, "right_rear_post"),
    ):
        _box(
            shaft,
            name,
            (shaft_post, shaft_post, shaft_height),
            (x, y, shaft_height / 2.0),
            "shaft_steel",
        )

    _box(
        shaft,
        "base_front_beam",
        (2.0 * shaft_half_width + shaft_post, shaft_post, shaft_post),
        (0.0, shaft_half_depth, beam_z_bottom),
        "shaft_steel",
    )
    _box(
        shaft,
        "base_rear_beam",
        (2.0 * shaft_half_width + shaft_post, shaft_post, shaft_post),
        (0.0, -shaft_half_depth, beam_z_bottom),
        "shaft_steel",
    )
    _box(
        shaft,
        "base_left_beam",
        (shaft_post, 2.0 * shaft_half_depth + shaft_post, shaft_post),
        (-shaft_half_width, 0.0, beam_z_bottom),
        "shaft_steel",
    )
    _box(
        shaft,
        "base_right_beam",
        (shaft_post, 2.0 * shaft_half_depth + shaft_post, shaft_post),
        (shaft_half_width, 0.0, beam_z_bottom),
        "shaft_steel",
    )
    _box(
        shaft,
        "top_front_beam",
        (2.0 * shaft_half_width + shaft_post, shaft_post, shaft_post),
        (0.0, shaft_half_depth, beam_z_top),
        "shaft_steel",
    )
    _box(
        shaft,
        "top_rear_beam",
        (2.0 * shaft_half_width + shaft_post, shaft_post, shaft_post),
        (0.0, -shaft_half_depth, beam_z_top),
        "shaft_steel",
    )
    _box(
        shaft,
        "top_left_beam",
        (shaft_post, 2.0 * shaft_half_depth + shaft_post, shaft_post),
        (-shaft_half_width, 0.0, beam_z_top),
        "shaft_steel",
    )
    _box(
        shaft,
        "top_right_beam",
        (shaft_post, 2.0 * shaft_half_depth + shaft_post, shaft_post),
        (shaft_half_width, 0.0, beam_z_top),
        "shaft_steel",
    )

    shaft.visual(
        Box((0.08, 0.10, shaft_height)),
        origin=Origin(xyz=(-guide_rail_x, guide_rail_y, shaft_height / 2.0)),
        material="guide_steel",
        name="left_guide_rail",
    )
    shaft.visual(
        Box((0.08, 0.10, shaft_height)),
        origin=Origin(xyz=(guide_rail_x, guide_rail_y, shaft_height / 2.0)),
        material="guide_steel",
        name="right_guide_rail",
    )
    for x, side in ((-guide_rail_x, "left"), (guide_rail_x, "right")):
        _box(
            shaft,
            f"{side}_rail_base_brace",
            (0.08, 0.20, shaft_post),
            (x, -0.70, beam_z_bottom),
            "shaft_steel",
        )
        _box(
            shaft,
            f"{side}_rail_top_brace",
            (0.08, 0.20, shaft_post),
            (x, -0.70, beam_z_top),
            "shaft_steel",
        )

    # Cage proportions.
    cage_width = 1.46
    cage_depth = 1.26
    cage_half_width = cage_width / 2.0
    cage_half_depth = cage_depth / 2.0
    floor_thickness = 0.06
    post_size = 0.06
    roof_beam_h = 0.06
    cage_height = 2.00
    post_length = cage_height - floor_thickness - roof_beam_h
    post_center_z = floor_thickness + post_length / 2.0
    roof_z = cage_height - roof_beam_h / 2.0

    # Cage floor and upper ring.
    _box(
        cage,
        "cage_floor",
        (cage_width, cage_depth, floor_thickness),
        (0.0, 0.0, floor_thickness / 2.0),
        "floor_dark",
    )
    _box(
        cage,
        "roof_front_beam",
        (cage_width, post_size, roof_beam_h),
        (0.0, cage_half_depth - post_size / 2.0, roof_z),
        "safety_yellow",
    )
    _box(
        cage,
        "roof_rear_beam",
        (cage_width, post_size, roof_beam_h),
        (0.0, -cage_half_depth + post_size / 2.0, roof_z),
        "safety_yellow",
    )
    _box(
        cage,
        "roof_left_beam",
        (post_size, cage_depth, roof_beam_h),
        (-cage_half_width + post_size / 2.0, 0.0, roof_z),
        "safety_yellow",
    )
    _box(
        cage,
        "roof_right_beam",
        (post_size, cage_depth, roof_beam_h),
        (cage_half_width - post_size / 2.0, 0.0, roof_z),
        "safety_yellow",
    )

    for x, y, name in (
        (-cage_half_width + post_size / 2.0, cage_half_depth - post_size / 2.0, "left_front_post"),
        (cage_half_width - post_size / 2.0, cage_half_depth - post_size / 2.0, "right_front_post"),
        (-cage_half_width + post_size / 2.0, -cage_half_depth + post_size / 2.0, "left_rear_post"),
        (cage_half_width - post_size / 2.0, -cage_half_depth + post_size / 2.0, "right_rear_post"),
    ):
        _box(
            cage,
            name,
            (post_size, post_size, post_length),
            (x, y, post_center_z),
            "safety_yellow",
        )

    _box(
        cage,
        "left_side_mid_rail",
        (post_size, cage_depth, 0.05),
        (-cage_half_width + post_size / 2.0, 0.0, 1.00),
        "safety_yellow",
    )
    _box(
        cage,
        "right_side_mid_rail",
        (post_size, cage_depth, 0.05),
        (cage_half_width - post_size / 2.0, 0.0, 1.00),
        "safety_yellow",
    )
    _box(
        cage,
        "back_mid_rail",
        (cage_width, post_size, 0.05),
        (0.0, -cage_half_depth + post_size / 2.0, 1.00),
        "safety_yellow",
    )

    for i, x in enumerate((-0.34, 0.0, 0.34), start=1):
        _box(
            cage,
            f"back_bar_{i}",
            (0.03, 0.03, 1.74),
            (x, -cage_half_depth + 0.03, 0.93),
            "safety_yellow",
        )

    track_y = cage_half_depth + 0.015
    cage.visual(
        Box((2.20, 0.05, 0.05)),
        origin=Origin(xyz=(0.37, track_y, 1.78)),
        material="guide_steel",
        name="top_gate_track",
    )
    cage.visual(
        Box((2.20, 0.05, 0.05)),
        origin=Origin(xyz=(0.37, track_y, 0.26)),
        material="guide_steel",
        name="bottom_gate_track",
    )

    # Guide shoe brackets that visually mount the cage to the fixed rails.
    for side, sign in (("left", -1.0), ("right", 1.0)):
        arm_x = sign * 0.74
        for level, z in (("lower", 0.42), ("upper", 1.68)):
            _box(
                cage,
                f"{side}_{level}_arm",
                (0.08, 0.04, 0.04),
                (arm_x, -0.60, z),
                "safety_yellow",
            )

    cage.visual(
        Box((0.04, 0.08, 0.12)),
        origin=Origin(xyz=(-0.78, -0.60, 0.42)),
        material="guide_steel",
        name="left_lower_shoe",
    )
    cage.visual(
        Box((0.04, 0.08, 0.12)),
        origin=Origin(xyz=(-0.78, -0.60, 1.68)),
        material="guide_steel",
        name="left_upper_shoe",
    )
    cage.visual(
        Box((0.04, 0.08, 0.12)),
        origin=Origin(xyz=(0.78, -0.60, 0.42)),
        material="guide_steel",
        name="right_lower_shoe",
    )
    cage.visual(
        Box((0.04, 0.08, 0.12)),
        origin=Origin(xyz=(0.78, -0.60, 1.68)),
        material="guide_steel",
        name="right_upper_shoe",
    )

    # Sliding gate: closed at q=0, opens to +X.
    gate_y = 0.648
    runner_depth = 0.05
    gate_depth = 0.026
    gate.visual(
        Box((0.08, runner_depth, 0.04)),
        origin=Origin(xyz=(-0.60, track_y, 1.735)),
        material="gate_metal",
        name="upper_left_runner",
    )
    gate.visual(
        Box((0.08, runner_depth, 0.04)),
        origin=Origin(xyz=(0.60, track_y, 1.735)),
        material="gate_metal",
        name="upper_right_runner",
    )
    gate.visual(
        Box((0.08, runner_depth, 0.04)),
        origin=Origin(xyz=(-0.60, track_y, 0.305)),
        material="gate_metal",
        name="lower_left_runner",
    )
    gate.visual(
        Box((0.08, runner_depth, 0.04)),
        origin=Origin(xyz=(0.60, track_y, 0.305)),
        material="gate_metal",
        name="lower_right_runner",
    )
    _box(
        gate,
        "gate_left_stile",
        (0.05, gate_depth, 1.39),
        (-0.60, gate_y, 1.02),
        "gate_metal",
    )
    _box(
        gate,
        "gate_right_stile",
        (0.05, gate_depth, 1.39),
        (0.60, gate_y, 1.02),
        "gate_metal",
    )
    _box(
        gate,
        "gate_top_rail",
        (1.20, gate_depth, 0.05),
        (0.0, gate_y, 1.69),
        "gate_metal",
    )
    _box(
        gate,
        "gate_bottom_rail",
        (1.20, gate_depth, 0.05),
        (0.0, gate_y, 0.35),
        "gate_metal",
    )
    for i, x in enumerate((-0.36, -0.12, 0.12, 0.36), start=1):
        _box(
            gate,
            f"gate_bar_{i}",
            (0.02, 0.02, 1.29),
            (x, gate_y, 1.02),
            "gate_metal",
        )

    model.articulation(
        "shaft_to_cage",
        ArticulationType.PRISMATIC,
        parent=shaft,
        child=cage,
        origin=Origin(xyz=(0.0, 0.0, 0.25)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4000.0,
            velocity=0.9,
            lower=0.0,
            upper=2.10,
        ),
    )
    model.articulation(
        "cage_to_gate",
        ArticulationType.PRISMATIC,
        parent=cage,
        child=gate,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=0.8,
            lower=0.0,
            upper=0.82,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shaft = object_model.get_part("shaft_frame")
    cage = object_model.get_part("cage")
    gate = object_model.get_part("gate")
    cage_lift = object_model.get_articulation("shaft_to_cage")
    gate_slide = object_model.get_articulation("cage_to_gate")

    ctx.expect_contact(
        cage,
        shaft,
        elem_a="left_lower_shoe",
        elem_b="left_guide_rail",
        name="left lower guide shoe bears on left rail",
    )
    ctx.expect_contact(
        cage,
        shaft,
        elem_a="right_lower_shoe",
        elem_b="right_guide_rail",
        name="right lower guide shoe bears on right rail",
    )
    ctx.expect_contact(
        gate,
        cage,
        elem_a="upper_left_runner",
        elem_b="top_gate_track",
        name="gate upper runner sits on top track when closed",
    )
    ctx.expect_contact(
        gate,
        cage,
        elem_a="lower_left_runner",
        elem_b="bottom_gate_track",
        name="gate lower runner sits on bottom track when closed",
    )

    cage_rest = ctx.part_world_position(cage)
    gate_rest = ctx.part_world_position(gate)

    with ctx.pose({cage_lift: 2.10, gate_slide: 0.82}):
        ctx.expect_contact(
            cage,
            shaft,
            elem_a="left_upper_shoe",
            elem_b="left_guide_rail",
            name="left upper guide shoe stays on rail at top travel",
        )
        ctx.expect_contact(
            cage,
            shaft,
            elem_a="right_upper_shoe",
            elem_b="right_guide_rail",
            name="right upper guide shoe stays on rail at top travel",
        )
        ctx.expect_contact(
            gate,
            cage,
            elem_a="upper_right_runner",
            elem_b="top_gate_track",
            name="gate stays captured by top track when open",
        )
        ctx.expect_contact(
            gate,
            cage,
            elem_a="lower_right_runner",
            elem_b="bottom_gate_track",
            name="gate stays captured by bottom track when open",
        )
        cage_top = ctx.part_world_position(cage)
        gate_open = ctx.part_world_position(gate)

    ctx.check(
        "cage rises vertically on guide rails",
        cage_rest is not None
        and cage_top is not None
        and cage_top[2] > cage_rest[2] + 2.0,
        details=f"rest={cage_rest}, top={cage_top}",
    )
    ctx.check(
        "gate slides sideways toward storage side",
        gate_rest is not None
        and gate_open is not None
        and gate_open[0] > gate_rest[0] + 0.75,
        details=f"rest={gate_rest}, open={gate_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
