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


BODY_WIDTH = 0.325
BODY_DEPTH = 0.138
BOTTOM_THICKNESS = 0.004
WALL_THICKNESS = 0.010
DECK_THICKNESS = 0.002
DECK_TOP_Z = 0.014
KEY_REST_Z = 0.0177
KEY_TRAVEL = 0.002


def _add_key(
    model: ArticulatedObject,
    body,
    *,
    name: str,
    center_xyz: tuple[float, float, float],
    width: float,
    depth: float,
    key_material: str,
    top_material: str,
) -> None:
    key = model.part(name)
    key.visual(
        Box((width, depth, 0.0032)),
        origin=Origin(),
        material=key_material,
        name="cap_base",
    )
    key.visual(
        Box((width * 0.86, depth * 0.80, 0.0020)),
        origin=Origin(xyz=(0.0, 0.0, 0.0018)),
        material=top_material,
        name="cap_top",
    )

    model.articulation(
        f"body_to_{name}",
        ArticulationType.PRISMATIC,
        parent=body,
        child=key,
        origin=Origin(xyz=center_xyz),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=0.12,
            lower=0.0,
            upper=KEY_TRAVEL,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_office_keyboard")

    body_color = model.material("body_graphite", color=(0.20, 0.22, 0.24))
    trim_color = model.material("body_trim", color=(0.14, 0.15, 0.17))
    key_color = model.material("key_mist", color=(0.86, 0.87, 0.88))
    key_top_color = model.material("key_top", color=(0.95, 0.95, 0.96))
    slider_color = model.material("slider_amber", color=(0.82, 0.55, 0.22))

    body = model.part("body")
    body.visual(
        Box((BODY_WIDTH, BODY_DEPTH, BOTTOM_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BOTTOM_THICKNESS / 2.0)),
        material=trim_color,
        name="bottom_shell",
    )
    body.visual(
        Box((BODY_WIDTH, WALL_THICKNESS, 0.010)),
        origin=Origin(xyz=(0.0, -(BODY_DEPTH / 2.0) + (WALL_THICKNESS / 2.0), 0.009)),
        material=body_color,
        name="front_wall",
    )
    body.visual(
        Box((WALL_THICKNESS, BODY_DEPTH - (2.0 * WALL_THICKNESS), 0.010)),
        origin=Origin(xyz=(-(BODY_WIDTH / 2.0) + (WALL_THICKNESS / 2.0), 0.0, 0.009)),
        material=body_color,
        name="left_wall",
    )
    body.visual(
        Box((BODY_WIDTH, WALL_THICKNESS, 0.010)),
        origin=Origin(xyz=(0.0, (BODY_DEPTH / 2.0) - (WALL_THICKNESS / 2.0), 0.009)),
        material=body_color,
        name="rear_lower_wall",
    )
    body.visual(
        Box((BODY_WIDTH, BODY_DEPTH, DECK_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, DECK_TOP_Z - (DECK_THICKNESS / 2.0))),
        material=body_color,
        name="main_deck",
    )

    slot_y_center = 0.01725
    body.visual(
        Box((WALL_THICKNESS, 0.075, 0.010)),
        origin=Origin(xyz=((BODY_WIDTH / 2.0) - (WALL_THICKNESS / 2.0), -0.0315, 0.009)),
        material=body_color,
        name="right_front_wall",
    )
    body.visual(
        Box((WALL_THICKNESS, 0.041, 0.010)),
        origin=Origin(xyz=((BODY_WIDTH / 2.0) - (WALL_THICKNESS / 2.0), 0.0485, 0.009)),
        material=body_color,
        name="right_rear_wall",
    )
    body.visual(
        Box((WALL_THICKNESS, 0.0215, 0.004)),
        origin=Origin(xyz=((BODY_WIDTH / 2.0) - (WALL_THICKNESS / 2.0), slot_y_center, 0.006)),
        material=body_color,
        name="power_slot_lower",
    )
    body.visual(
        Box((WALL_THICKNESS, 0.0215, 0.002)),
        origin=Origin(xyz=((BODY_WIDTH / 2.0) - (WALL_THICKNESS / 2.0), slot_y_center, 0.013)),
        material=body_color,
        name="power_slot_upper",
    )

    body.visual(
        Box((BODY_WIDTH, WALL_THICKNESS, 0.012)),
        origin=Origin(xyz=(0.0, (BODY_DEPTH / 2.0) - (WALL_THICKNESS / 2.0), 0.020)),
        material=body_color,
        name="shelf_back",
    )
    body.visual(
        Box((0.290, 0.018, 0.003)),
        origin=Origin(xyz=(0.0, 0.052, 0.0185)),
        material=body_color,
        name="shelf_floor",
    )
    body.visual(
        Box((0.290, 0.005, 0.006)),
        origin=Origin(xyz=(0.0, 0.0415, 0.017)),
        material=body_color,
        name="shelf_lip",
    )

    unit = 0.0175
    gap = 0.0025
    rows = [
        (0.030, -0.008, 0.0155, [unit] * 11 + [0.0310]),
        (0.010, -0.004, 0.0160, [0.0260] + [unit] * 10 + [0.0260]),
        (-0.010, 0.000, 0.0160, [0.0310] + [unit] * 9 + [0.0310]),
        (-0.030, 0.004, 0.0160, [0.0390] + [unit] * 8 + [0.0390]),
        (-0.048, 0.000, 0.0145, [0.0210, 0.0210, 0.0210, 0.1020, 0.0210, 0.0210, 0.0210, 0.0210]),
    ]

    for row_index, (y_pos, stagger, key_depth, widths) in enumerate(rows):
        total_width = sum(widths) + gap * (len(widths) - 1)
        cursor_x = stagger - (total_width / 2.0)
        for col_index, key_width in enumerate(widths):
            center_x = cursor_x + (key_width / 2.0)
            key_name = f"key_r{row_index}_c{col_index}"
            if row_index == 4 and col_index == 3:
                key_name = "spacebar"
            _add_key(
                model,
                body,
                name=key_name,
                center_xyz=(center_x, y_pos, KEY_REST_Z),
                width=key_width,
                depth=key_depth,
                key_material=key_color.name,
                top_material=key_top_color.name,
            )
            cursor_x += key_width + gap

    power_slider = model.part("power_slider")
    power_slider.visual(
        Box((0.0060, 0.0080, 0.0036)),
        origin=Origin(),
        material=slider_color,
        name="slider_tab",
    )
    power_slider.visual(
        Box((0.0012, 0.0042, 0.0022)),
        origin=Origin(xyz=(0.0024, 0.0, 0.0)),
        material=key_top_color,
        name="slider_ridge",
    )
    model.articulation(
        "body_to_power_slider",
        ArticulationType.PRISMATIC,
        parent=body,
        child=power_slider,
        origin=Origin(xyz=((BODY_WIDTH / 2.0), 0.0105, 0.0100)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=0.04,
            lower=0.0,
            upper=0.008,
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

    for part in object_model.parts:
        if part.name.startswith("key_") or part.name == "spacebar":
            ctx.allow_isolated_part(
                part,
                reason="Each key is intentionally carried by an internal plunger mechanism inside the keyboard shell.",
            )
        if part.name == "power_slider":
            ctx.allow_isolated_part(
                part,
                reason="The power slider is intentionally carried by the molded side guide slot with running clearance.",
            )

    body = object_model.get_part("body")
    sample_key = object_model.get_part("key_r2_c4")
    spacebar = object_model.get_part("spacebar")
    power_slider = object_model.get_part("power_slider")

    sample_key_joint = object_model.get_articulation("body_to_key_r2_c4")
    spacebar_joint = object_model.get_articulation("body_to_spacebar")
    power_slider_joint = object_model.get_articulation("body_to_power_slider")

    with ctx.pose({sample_key_joint: 0.0, spacebar_joint: 0.0, power_slider_joint: 0.0}):
        ctx.expect_gap(
            sample_key,
            body,
            axis="z",
            min_gap=0.0015,
            max_gap=0.0035,
            positive_elem="cap_base",
            negative_elem="main_deck",
            name="sample key sits proud of the deck",
        )
        ctx.expect_overlap(
            sample_key,
            body,
            axes="xy",
            min_overlap=0.012,
            elem_a="cap_base",
            elem_b="main_deck",
            name="sample key stays over the deck opening zone",
        )
        ctx.expect_gap(
            spacebar,
            body,
            axis="z",
            min_gap=0.0015,
            max_gap=0.0035,
            positive_elem="cap_base",
            negative_elem="main_deck",
            name="spacebar sits proud of the deck",
        )
        ctx.expect_gap(
            power_slider,
            body,
            axis="z",
            min_gap=0.0001,
            max_gap=0.0010,
            positive_elem="slider_tab",
            negative_elem="power_slot_lower",
            name="power slider clears the lower guide",
        )
        ctx.expect_gap(
            body,
            power_slider,
            axis="z",
            min_gap=0.0001,
            max_gap=0.0010,
            positive_elem="power_slot_upper",
            negative_elem="slider_tab",
            name="power slider clears the upper guide",
        )
        ctx.expect_overlap(
            power_slider,
            body,
            axes="y",
            min_overlap=0.0075,
            elem_a="slider_tab",
            elem_b="power_slot_lower",
            name="power slider stays captured in the side guide",
        )

    deck_aabb = ctx.part_element_world_aabb(body, elem="main_deck")
    shelf_floor_aabb = ctx.part_element_world_aabb(body, elem="shelf_floor")
    shelf_lip_aabb = ctx.part_element_world_aabb(body, elem="shelf_lip")
    ctx.check(
        "phone shelf floor stands above the key deck",
        deck_aabb is not None
        and shelf_floor_aabb is not None
        and shelf_floor_aabb[0][2] > deck_aabb[1][2] + 0.0025,
        details=f"deck={deck_aabb}, shelf_floor={shelf_floor_aabb}",
    )
    ctx.check(
        "phone shelf has a forward retaining lip",
        shelf_floor_aabb is not None
        and shelf_lip_aabb is not None
        and shelf_floor_aabb[0][1] > shelf_lip_aabb[0][1] + 0.003
        and shelf_lip_aabb[1][2] >= shelf_floor_aabb[1][2] - 0.0005,
        details=f"shelf_floor={shelf_floor_aabb}, shelf_lip={shelf_lip_aabb}",
    )

    sample_key_rest = ctx.part_world_position(sample_key)
    power_slider_rest = ctx.part_world_position(power_slider)
    with ctx.pose(
        {
            sample_key_joint: sample_key_joint.motion_limits.upper or KEY_TRAVEL,
            power_slider_joint: power_slider_joint.motion_limits.upper or 0.008,
        }
    ):
        ctx.expect_gap(
            sample_key,
            body,
            axis="z",
            min_gap=0.0,
            max_gap=0.0012,
            positive_elem="cap_base",
            negative_elem="main_deck",
            name="depressed key nearly seats flush with the deck",
        )
        sample_key_pressed = ctx.part_world_position(sample_key)
        power_slider_on = ctx.part_world_position(power_slider)

    ctx.check(
        "sample key plunges downward",
        sample_key_rest is not None
        and sample_key_pressed is not None
        and sample_key_pressed[2] < sample_key_rest[2] - 0.0015
        and abs(sample_key_pressed[0] - sample_key_rest[0]) < 0.0005
        and abs(sample_key_pressed[1] - sample_key_rest[1]) < 0.0005,
        details=f"rest={sample_key_rest}, pressed={sample_key_pressed}",
    )
    ctx.check(
        "power slider moves rearward along the side wall",
        power_slider_rest is not None
        and power_slider_on is not None
        and power_slider_on[1] > power_slider_rest[1] + 0.006
        and abs(power_slider_on[0] - power_slider_rest[0]) < 0.0005
        and abs(power_slider_on[2] - power_slider_rest[2]) < 0.0005,
        details=f"rest={power_slider_rest}, on={power_slider_on}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
