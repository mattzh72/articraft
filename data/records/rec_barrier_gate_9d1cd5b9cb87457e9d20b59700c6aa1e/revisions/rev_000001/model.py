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
)


RAIL_TOP_Z = 0.05
LEFT_RAIL_Y = 0.11
RIGHT_RAIL_Y = -0.11
OUTER_SLIDE_TRAVEL = 0.95
INNER_SLIDE_TRAVEL = 1.89


def _add_gate_frame(
    part,
    *,
    prefix: str,
    width: float,
    height: float,
    depth: float,
    stile_width: float,
    rail_height: float,
    material,
    bottom_z: float = 0.0,
    crossbar_z: float | None = None,
    crossbar_height: float = 0.045,
    picket_count: int = 0,
    picket_width: float = 0.022,
    picket_depth_scale: float = 0.55,
    picket_y: float = 0.0,
) -> None:
    left_stile_name = f"{prefix}_left_stile"
    if prefix == "inner":
        right_stile_name = "inner_right_stile"
    else:
        right_stile_name = f"{prefix}_right_stile"

    part.visual(
        Box((stile_width, depth, height)),
        origin=Origin(xyz=(stile_width * 0.5, 0.0, bottom_z + height * 0.5)),
        material=material,
        name=left_stile_name,
    )
    part.visual(
        Box((stile_width, depth, height)),
        origin=Origin(xyz=(width - stile_width * 0.5, 0.0, bottom_z + height * 0.5)),
        material=material,
        name=right_stile_name,
    )

    rail_length = width - 2.0 * stile_width
    part.visual(
        Box((rail_length, depth, rail_height)),
        origin=Origin(xyz=(width * 0.5, 0.0, bottom_z + rail_height * 0.5)),
        material=material,
        name=f"{prefix}_bottom_rail",
    )
    part.visual(
        Box((rail_length, depth, rail_height)),
        origin=Origin(
            xyz=(width * 0.5, 0.0, bottom_z + height - rail_height * 0.5)
        ),
        material=material,
        name=f"{prefix}_top_rail",
    )

    if crossbar_z is not None:
        part.visual(
            Box((rail_length, depth * 0.78, crossbar_height)),
            origin=Origin(xyz=(width * 0.5, 0.0, bottom_z + crossbar_z)),
            material=material,
            name=f"{prefix}_mid_rail",
        )

    if picket_count > 0:
        clear_width = width - 2.0 * stile_width
        spacing = clear_width / (picket_count + 1)
        picket_height = height - 2.0 * rail_height
        for index in range(picket_count):
            x = stile_width + spacing * (index + 1)
            part.visual(
                Box((picket_width, depth * picket_depth_scale, picket_height)),
                origin=Origin(
                    xyz=(x, picket_y, bottom_z + rail_height + picket_height * 0.5)
                ),
                material=material,
                name=f"{prefix}_picket_{index}",
            )


def _add_outer_carriage(part, *, prefix: str, x: float, wheel_material, frame_material) -> None:
    part.visual(
        Box((0.20, 0.30, 0.018)),
        origin=Origin(xyz=(x, 0.0, 0.118)),
        material=frame_material,
        name=f"{prefix}_carriage_plate",
    )
    part.visual(
        Box((0.11, 0.12, 0.032)),
        origin=Origin(xyz=(x, 0.0, 0.138)),
        material=frame_material,
        name=f"{prefix}_cross_block",
    )
    part.visual(
        Box((0.080, 0.080, 0.100)),
        origin=Origin(xyz=(x, 0.0, 0.170)),
        material=frame_material,
        name=f"{prefix}_frame_strut",
    )
    for side_name, rail_y in (("front", LEFT_RAIL_Y), ("rear", RIGHT_RAIL_Y)):
        if prefix == "leading" and side_name == "front":
            wheel_name = "leading_front_wheel"
        elif prefix == "leading" and side_name == "rear":
            wheel_name = "leading_rear_wheel"
        else:
            wheel_name = f"{prefix}_{side_name}_wheel"
        part.visual(
            Box((0.034, 0.040, 0.092)),
            origin=Origin(xyz=(x, rail_y, 0.081)),
            material=frame_material,
            name=f"{prefix}_{side_name}_hanger",
        )
        part.visual(
            Cylinder(radius=0.035, length=0.028),
            origin=Origin(
                xyz=(x, rail_y, 0.035),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=wheel_material,
            name=wheel_name,
        )
        part.visual(
            Box((0.028, 0.034, 0.030)),
            origin=Origin(xyz=(x, rail_y, 0.060)),
            material=frame_material,
            name=f"{prefix}_{side_name}_hub",
        )


def _combined_span(ctx: TestContext, parts: list) -> float | None:
    mins: list[float] = []
    maxs: list[float] = []
    for part in parts:
        aabb = ctx.part_world_aabb(part)
        if aabb is None:
            return None
        mins.append(aabb[0][0])
        maxs.append(aabb[1][0])
    return max(maxs) - min(mins)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_sliding_gate")

    powder_graphite = model.material(
        "powder_graphite",
        rgba=(0.20, 0.22, 0.24, 1.0),
    )
    gate_silver = model.material(
        "gate_silver",
        rgba=(0.70, 0.72, 0.74, 1.0),
    )
    weathered_steel = model.material(
        "weathered_steel",
        rgba=(0.42, 0.43, 0.45, 1.0),
    )
    rubber_black = model.material(
        "rubber_black",
        rgba=(0.08, 0.08, 0.09, 1.0),
    )
    concrete = model.material(
        "concrete",
        rgba=(0.66, 0.66, 0.64, 1.0),
    )
    safety_yellow = model.material(
        "safety_yellow",
        rgba=(0.84, 0.69, 0.15, 1.0),
    )

    track_assembly = model.part("track_assembly")
    track_assembly.visual(
        Box((5.70, 0.86, 0.12)),
        origin=Origin(xyz=(2.85, 0.0, -0.06)),
        material=concrete,
        name="concrete_pad",
    )
    track_assembly.visual(
        Box((5.40, 0.04, 0.05)),
        origin=Origin(xyz=(2.70, LEFT_RAIL_Y, 0.025)),
        material=weathered_steel,
        name="left_guide_rail",
    )
    track_assembly.visual(
        Box((5.40, 0.04, 0.05)),
        origin=Origin(xyz=(2.70, RIGHT_RAIL_Y, 0.025)),
        material=weathered_steel,
        name="right_guide_rail",
    )
    for index, x in enumerate((0.70, 1.65, 2.60, 3.55, 4.50)):
        track_assembly.visual(
            Box((0.18, 0.24, 0.04)),
            origin=Origin(xyz=(x, 0.0, 0.02)),
            material=weathered_steel,
            name=f"rail_tie_{index}",
        )
    track_assembly.visual(
        Box((0.10, 0.18, 2.05)),
        origin=Origin(xyz=(0.05, 0.0, 1.025)),
        material=powder_graphite,
        name="drive_post",
    )
    track_assembly.visual(
        Box((0.24, 0.20, 0.36)),
        origin=Origin(xyz=(0.28, 0.24, 0.18)),
        material=powder_graphite,
        name="motor_housing",
    )
    track_assembly.visual(
        Box((0.08, 0.12, 0.90)),
        origin=Origin(xyz=(0.14, 0.12, 1.53)),
        material=powder_graphite,
        name="guide_mast",
    )
    track_assembly.visual(
        Box((0.24, 0.12, 0.08)),
        origin=Origin(xyz=(0.26, 0.12, 1.94)),
        material=powder_graphite,
        name="top_guide_head",
    )
    track_assembly.visual(
        Box((0.14, 0.08, 0.10)),
        origin=Origin(xyz=(0.12, 0.08, 1.14)),
        material=powder_graphite,
        name="guide_mast_bracket_lower",
    )
    track_assembly.visual(
        Box((0.18, 0.08, 0.10)),
        origin=Origin(xyz=(0.16, 0.08, 1.82)),
        material=powder_graphite,
        name="guide_mast_bracket_upper",
    )
    track_assembly.visual(
        Box((0.10, 0.18, 2.00)),
        origin=Origin(xyz=(5.18, 0.0, 1.00)),
        material=powder_graphite,
        name="receiver_post",
    )
    track_assembly.visual(
        Box((0.02, 0.10, 1.36)),
        origin=Origin(xyz=(4.99, 0.0, 0.98)),
        material=safety_yellow,
        name="receiver_stop_pad",
    )
    track_assembly.visual(
        Box((0.18, 0.04, 0.05)),
        origin=Origin(xyz=(5.04, 0.0, 0.40)),
        material=powder_graphite,
        name="receiver_lower_bracket",
    )
    track_assembly.visual(
        Box((0.18, 0.04, 0.05)),
        origin=Origin(xyz=(5.04, 0.0, 1.56)),
        material=powder_graphite,
        name="receiver_upper_bracket",
    )
    track_assembly.visual(
        Box((0.10, 0.30, 0.14)),
        origin=Origin(xyz=(5.35, 0.0, 0.07)),
        material=powder_graphite,
        name="track_end_stop",
    )
    track_assembly.inertial = Inertial.from_geometry(
        Box((5.70, 0.86, 2.10)),
        mass=420.0,
        origin=Origin(xyz=(2.85, 0.0, 0.93)),
    )

    outer_panel = model.part("outer_panel")
    _add_gate_frame(
        outer_panel,
        prefix="outer",
        width=2.02,
        height=1.72,
        depth=0.08,
        stile_width=0.054,
        rail_height=0.060,
        material=powder_graphite,
        bottom_z=0.19,
        crossbar_z=None,
        crossbar_height=0.050,
        picket_count=7,
        picket_width=0.024,
        picket_depth_scale=0.20,
        picket_y=0.030,
    )
    outer_panel.visual(
        Box((2.02 - 2.0 * 0.054, 0.020, 0.050)),
        origin=Origin(xyz=(2.02 * 0.5, 0.030, 0.19 + 0.95)),
        material=powder_graphite,
        name="outer_mid_rail",
    )
    outer_panel.visual(
        Box((0.22, 0.04, 0.12)),
        origin=Origin(xyz=(0.18, 0.06, 0.29)),
        material=gate_silver,
        name="outer_pickup_bracket",
    )
    outer_panel.visual(
        Box((0.16, 0.04, 0.18)),
        origin=Origin(xyz=(1.84, 0.06, 0.31)),
        material=gate_silver,
        name="outer_trailing_bracket",
    )
    _add_outer_carriage(
        outer_panel,
        prefix="leading",
        x=0.54,
        wheel_material=rubber_black,
        frame_material=gate_silver,
    )
    _add_outer_carriage(
        outer_panel,
        prefix="trailing",
        x=1.54,
        wheel_material=rubber_black,
        frame_material=gate_silver,
    )
    outer_panel.inertial = Inertial.from_geometry(
        Box((2.10, 0.32, 1.92)),
        mass=85.0,
        origin=Origin(xyz=(1.01, 0.0, 0.96)),
    )

    inner_panel = model.part("inner_panel")
    _add_gate_frame(
        inner_panel,
        prefix="inner",
        width=1.90,
        height=1.46,
        depth=0.036,
        stile_width=0.054,
        rail_height=0.050,
        material=gate_silver,
        bottom_z=0.0,
        crossbar_z=0.78,
        crossbar_height=0.040,
        picket_count=6,
        picket_width=0.020,
        picket_depth_scale=0.72,
    )
    inner_panel.visual(
        Box((0.10, 0.034, 0.16)),
        origin=Origin(xyz=(0.054, 0.0, 0.18)),
        material=powder_graphite,
        name="inner_slide_shoe_lower",
    )
    inner_panel.visual(
        Box((0.10, 0.034, 0.16)),
        origin=Origin(xyz=(0.054, 0.0, 1.28)),
        material=powder_graphite,
        name="inner_slide_shoe_upper",
    )
    inner_panel.inertial = Inertial.from_geometry(
        Box((1.92, 0.05, 1.48)),
        mass=52.0,
        origin=Origin(xyz=(0.95, 0.0, 0.73)),
    )

    model.articulation(
        "track_to_outer_panel",
        ArticulationType.PRISMATIC,
        parent=track_assembly,
        child=outer_panel,
        origin=Origin(xyz=(0.18, 0.0, RAIL_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.35,
            lower=0.0,
            upper=OUTER_SLIDE_TRAVEL,
        ),
    )
    model.articulation(
        "outer_to_inner_panel",
        ArticulationType.PRISMATIC,
        parent=outer_panel,
        child=inner_panel,
        origin=Origin(xyz=(0.066, 0.0, 0.28)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.35,
            lower=0.0,
            upper=INNER_SLIDE_TRAVEL,
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
    track_assembly = object_model.get_part("track_assembly")
    outer_panel = object_model.get_part("outer_panel")
    inner_panel = object_model.get_part("inner_panel")
    outer_slide = object_model.get_articulation("track_to_outer_panel")
    inner_slide = object_model.get_articulation("outer_to_inner_panel")

    ctx.expect_contact(
        outer_panel,
        track_assembly,
        elem_a="leading_front_wheel",
        elem_b="left_guide_rail",
        contact_tol=0.001,
        name="leading front wheel rides on left guide rail",
    )
    ctx.expect_contact(
        outer_panel,
        track_assembly,
        elem_a="leading_rear_wheel",
        elem_b="right_guide_rail",
        contact_tol=0.001,
        name="leading rear wheel rides on right guide rail",
    )
    ctx.expect_within(
        inner_panel,
        outer_panel,
        axes="yz",
        margin=0.001,
        name="inner panel nests within outer panel depth and height at rest",
    )
    ctx.expect_overlap(
        inner_panel,
        outer_panel,
        axes="x",
        min_overlap=1.75,
        name="collapsed telescoping panels substantially overlap in span",
    )

    outer_rest = ctx.part_world_position(outer_panel)
    inner_rest = ctx.part_world_position(inner_panel)
    collapsed_span = _combined_span(ctx, [outer_panel, inner_panel])

    with ctx.pose({outer_slide: OUTER_SLIDE_TRAVEL}):
        outer_shifted = ctx.part_world_position(outer_panel)
    ctx.check(
        "outer panel slides in positive x along the track",
        outer_rest is not None
        and outer_shifted is not None
        and outer_shifted[0] > outer_rest[0] + 0.80,
        details=f"rest={outer_rest}, shifted={outer_shifted}",
    )

    with ctx.pose({inner_slide: INNER_SLIDE_TRAVEL}):
        inner_shifted = ctx.part_world_position(inner_panel)
    ctx.check(
        "inner panel extends out of the outer panel in positive x",
        inner_rest is not None
        and inner_shifted is not None
        and inner_shifted[0] > inner_rest[0] + 1.60,
        details=f"rest={inner_rest}, shifted={inner_shifted}",
    )

    with ctx.pose({outer_slide: OUTER_SLIDE_TRAVEL, inner_slide: INNER_SLIDE_TRAVEL}):
        ctx.expect_within(
            inner_panel,
            outer_panel,
            axes="yz",
            margin=0.002,
            name="extended inner panel stays aligned inside the outer panel guides",
        )
        ctx.expect_overlap(
            inner_panel,
            outer_panel,
            axes="x",
            min_overlap=0.06,
            name="extended inner panel retains insertion inside the outer panel",
        )
        ctx.expect_contact(
            inner_panel,
            track_assembly,
            elem_a="inner_right_stile",
            elem_b="receiver_stop_pad",
            contact_tol=0.003,
            name="fully extended inner panel reaches the receiver stop",
        )
        extended_span = _combined_span(ctx, [outer_panel, inner_panel])

    ctx.check(
        "telescoping gate nearly doubles its stored span when fully extended",
        collapsed_span is not None
        and extended_span is not None
        and extended_span > collapsed_span * 1.9
        and extended_span > 3.80,
        details=f"collapsed_span={collapsed_span}, extended_span={extended_span}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
