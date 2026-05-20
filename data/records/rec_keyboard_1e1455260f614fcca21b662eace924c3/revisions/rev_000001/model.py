from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    LoftGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


CASE_WIDTH = 0.330
CASE_DEPTH = 0.145
CASE_HEIGHT = 0.018
KEY_UNIT = 0.0200
KEY_GAP = 0.0030
KEY_DEPTH = 0.0145
KEY_STEM_HEIGHT = 0.0022
KEY_CAP_HEIGHT = 0.0052
# The plunger stem bottoms sit on the recessed key well at rest, so every key
# has a real support contact while still retaining a short downward travel.
KEY_REST_Z = CASE_HEIGHT + 0.00065
KEY_TRAVEL = 0.0018
SWITCH_SLOT_CENTER = (CASE_WIDTH * 0.5 + 0.0020, -0.050, 0.0110)
SWITCH_TRAVEL = 0.009


def _case_shell_mesh():
    """A shallow rounded wedge, like a low-profile compact office keyboard."""
    sections = [
        (CASE_WIDTH, CASE_DEPTH, 0.010, 0.0000),
        (CASE_WIDTH - 0.004, CASE_DEPTH - 0.003, 0.014, 0.0100),
        (CASE_WIDTH - 0.012, CASE_DEPTH - 0.010, 0.018, CASE_HEIGHT),
    ]
    profiles = []
    for width, depth, radius, z in sections:
        profiles.append(
            [(x, y, z) for x, y in rounded_rect_profile(width, depth, radius, corner_segments=8)]
        )
    return mesh_from_geometry(LoftGeometry(profiles, cap=True, closed=True), "keyboard_case_shell")


def _keycap_mesh(width: float, depth: float, height: float, name: str):
    """Slightly tapered rounded-rectangle keycap with a smaller top face."""
    lower = rounded_rect_profile(width, depth, min(0.0025, width * 0.18, depth * 0.18), corner_segments=5)
    upper = rounded_rect_profile(
        max(width - 0.0022, width * 0.80),
        max(depth - 0.0020, depth * 0.78),
        min(0.0022, width * 0.16, depth * 0.16),
        corner_segments=5,
    )
    geom = LoftGeometry(
        [[(x, y, 0.0) for x, y in lower], [(x, y, height) for x, y in upper]],
        cap=True,
        closed=True,
    )
    return mesh_from_geometry(geom, name)


def _add_key(
    model: ArticulatedObject,
    case,
    row_index: int,
    column_index: int,
    x: float,
    y: float,
    width: float,
    cap_mesh,
    key_material: Material,
    stem_material: Material,
):
    key = model.part(f"key_{row_index}_{column_index}")
    stem_width = min(max(width * 0.44, 0.0060), 0.020)
    key.visual(
        Box((stem_width, 0.0068, KEY_STEM_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, KEY_STEM_HEIGHT * 0.5)),
        material=stem_material,
        name="plunger_stem",
    )
    key.visual(
        cap_mesh,
        origin=Origin(xyz=(0.0, 0.0, KEY_STEM_HEIGHT)),
        material=key_material,
        name="keycap",
    )
    # A shallow printed-legend dash gives the grid a recognisable keyboard scale
    # without adding fragile text geometry.
    legend_width = min(width * 0.38, 0.010)
    key.visual(
        Box((legend_width, 0.0010, 0.00025)),
        origin=Origin(xyz=(0.0, -depth_label_offset(width), KEY_STEM_HEIGHT + KEY_CAP_HEIGHT + 0.00005)),
        material=stem_material,
        name="legend_mark",
    )
    model.articulation(
        f"case_to_key_{row_index}_{column_index}",
        ArticulationType.PRISMATIC,
        parent=case,
        child=key,
        origin=Origin(xyz=(x, y, KEY_REST_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=2.2, velocity=0.06, lower=0.0, upper=KEY_TRAVEL),
    )
    return key


def depth_label_offset(width: float) -> float:
    # Wide keys (space and modifiers) get a centred printed dash; small keys
    # get the dash biased toward the upper half like real legends.
    return 0.0015 if width > 0.040 else 0.0028


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_office_keyboard")

    case_mat = model.material("matte_graphite", rgba=(0.12, 0.13, 0.14, 1.0))
    edge_mat = model.material("soft_black", rgba=(0.035, 0.037, 0.040, 1.0))
    slot_mat = model.material("recess_shadow", rgba=(0.015, 0.017, 0.020, 1.0))
    key_mat = model.material("warm_keycaps", rgba=(0.82, 0.84, 0.84, 1.0))
    modifier_mat = model.material("cool_modifier_keys", rgba=(0.56, 0.59, 0.60, 1.0))
    switch_mat = model.material("power_switch_silver", rgba=(0.66, 0.70, 0.72, 1.0))

    case = model.part("case")
    case.visual(_case_shell_mesh(), material=case_mat, name="body_shell")

    # Narrow phone shelf / device slot along the upper edge.  The dark floor is
    # slightly lower than the two lips so it reads as a recessed trough.
    case.visual(
        Box((0.286, 0.013, 0.0015)),
        origin=Origin(xyz=(0.0, 0.054, CASE_HEIGHT + 0.0001)),
        material=slot_mat,
        name="phone_slot_floor",
    )
    case.visual(
        Box((0.296, 0.0032, 0.0052)),
        origin=Origin(xyz=(0.0, 0.0458, CASE_HEIGHT + 0.0025)),
        material=case_mat,
        name="phone_slot_front_lip",
    )
    case.visual(
        Box((0.296, 0.0042, 0.0065)),
        origin=Origin(xyz=(0.0, 0.0649, CASE_HEIGHT + 0.0030)),
        material=case_mat,
        name="phone_slot_rear_lip",
    )
    for x in (-0.150, 0.150):
        case.visual(
            Box((0.005, 0.022, 0.0050)),
            origin=Origin(xyz=(x, 0.055, CASE_HEIGHT + 0.0020)),
            material=case_mat,
            name=f"phone_slot_end_{0 if x < 0 else 1}",
        )

    # A subtle recessed key well under the movable plungers.
    case.visual(
        Box((0.304, 0.094, 0.0010)),
        origin=Origin(xyz=(0.0, -0.015, CASE_HEIGHT + 0.00015)),
        material=edge_mat,
        name="key_well_shadow",
    )

    # Side-wall guide rails for the small sliding power switch.
    guide_x = CASE_WIDTH * 0.5 - 0.0015
    guide_y = SWITCH_SLOT_CENTER[1]
    guide_z = SWITCH_SLOT_CENTER[2]
    case.visual(
        Box((0.0060, 0.052, 0.0020)),
        origin=Origin(xyz=(guide_x, guide_y, guide_z + 0.0055)),
        material=edge_mat,
        name="switch_guide_top",
    )
    case.visual(
        Box((0.0060, 0.052, 0.0020)),
        origin=Origin(xyz=(guide_x, guide_y, guide_z - 0.0055)),
        material=edge_mat,
        name="switch_guide_bottom",
    )
    for index, y in enumerate((guide_y - 0.0275, guide_y + 0.0275)):
        case.visual(
            Box((0.0060, 0.0030, 0.0128)),
            origin=Origin(xyz=(guide_x, y, guide_z)),
            material=edge_mat,
            name=f"switch_guide_stop_{index}",
        )

    cap_mesh_cache: dict[tuple[int, int], object] = {}

    def cap_for(width: float, depth: float = KEY_DEPTH):
        key = (round(width * 10000), round(depth * 10000))
        if key not in cap_mesh_cache:
            cap_mesh_cache[key] = _keycap_mesh(width, depth, KEY_CAP_HEIGHT, f"keycap_{key[0]}_{key[1]}")
        return cap_mesh_cache[key]

    # Five familiar compact office-keyboard rows: number row, QWERTY rows,
    # shift row, and a bottom row with a long space bar.
    row_specs = [
        (0.025, [1.0] * 14),
        (0.006, [1.35] + [1.0] * 12 + [1.15]),
        (-0.013, [1.60] + [1.0] * 10 + [1.90]),
        (-0.032, [2.05] + [1.0] * 9 + [2.05]),
        (-0.052, [1.25, 1.05, 1.05, 5.55, 1.05, 1.05, 1.25]),
    ]
    for row_index, (y, units) in enumerate(row_specs):
        total_units = sum(units)
        start_x = -0.5 * total_units * KEY_UNIT
        cursor = start_x
        for column_index, unit_width in enumerate(units):
            center_x = cursor + unit_width * KEY_UNIT * 0.5
            actual_width = max(unit_width * KEY_UNIT - KEY_GAP, 0.010)
            mat = modifier_mat if (unit_width > 1.20 or row_index == 4) else key_mat
            _add_key(
                model,
                case,
                row_index,
                column_index,
                center_x,
                y,
                actual_width,
                cap_for(actual_width),
                mat,
                edge_mat,
            )
            cursor += unit_width * KEY_UNIT

    power_switch = model.part("power_switch")
    power_switch.visual(
        Box((0.010, 0.012, 0.0045)),
        origin=Origin(xyz=(-0.0045, 0.0, 0.0)),
        material=edge_mat,
        name="retainer_stem",
    )
    power_switch.visual(
        Box((0.0080, 0.0145, 0.0068)),
        origin=Origin(xyz=(0.0040, 0.0, 0.0)),
        material=switch_mat,
        name="thumb_slider",
    )
    power_switch.visual(
        Box((0.0012, 0.0100, 0.0010)),
        origin=Origin(xyz=(0.0086, 0.0, 0.0021)),
        material=edge_mat,
        name="switch_indicator",
    )
    model.articulation(
        "case_to_power_switch",
        ArticulationType.PRISMATIC,
        parent=case,
        child=power_switch,
        origin=Origin(xyz=SWITCH_SLOT_CENTER),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=0.08, lower=-SWITCH_TRAVEL, upper=SWITCH_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    case = object_model.get_part("case")
    power_switch = object_model.get_part("power_switch")
    switch_joint = object_model.get_articulation("case_to_power_switch")
    sample_key = object_model.get_part("key_2_5")
    sample_key_joint = object_model.get_articulation("case_to_key_2_5")

    key_parts = [part for part in object_model.parts if part.name.startswith("key_")]
    ctx.check(
        "standard compact keyboard has five populated key rows",
        len(key_parts) == 58,
        details=f"expected 58 articulated keys, found {len(key_parts)}",
    )

    ctx.expect_gap(
        sample_key,
        case,
        axis="z",
        positive_elem="plunger_stem",
        negative_elem="key_well_shadow",
        max_gap=0.0001,
        max_penetration=0.0,
        name="key plunger is supported by the recessed well at rest",
    )
    rest_key_pos = ctx.part_world_position(sample_key)
    with ctx.pose({sample_key_joint: KEY_TRAVEL}):
        pressed_key_pos = ctx.part_world_position(sample_key)
        ctx.expect_gap(
            sample_key,
            case,
            axis="z",
            positive_elem="keycap",
            negative_elem="key_well_shadow",
            min_gap=0.00015,
            name="pressed keycap still clears the deck",
        )
    ctx.check(
        "key prismatic joint travels downward",
        rest_key_pos is not None
        and pressed_key_pos is not None
        and pressed_key_pos[2] < rest_key_pos[2] - 0.001,
        details=f"rest={rest_key_pos}, pressed={pressed_key_pos}",
    )

    ctx.allow_overlap(
        case,
        power_switch,
        elem_a="body_shell",
        elem_b="retainer_stem",
        reason=(
            "The switch's hidden retainer stem is intentionally captured inside "
            "the side-wall guide slot so the thumb slider cannot separate."
        ),
    )
    ctx.expect_overlap(
        power_switch,
        case,
        axes="x",
        elem_a="retainer_stem",
        elem_b="body_shell",
        min_overlap=0.004,
        name="power switch retainer stays inserted through the case wall",
    )
    ctx.expect_gap(
        power_switch,
        case,
        axis="x",
        positive_elem="thumb_slider",
        negative_elem="body_shell",
        min_gap=0.001,
        max_gap=0.004,
        name="switch thumb slider rides outside the side wall",
    )

    def switch_is_within_slot(label: str) -> None:
        aabb = ctx.part_element_world_aabb(power_switch, elem="retainer_stem")
        if aabb is None:
            ctx.fail(f"switch retainer remains in guide at {label}", "missing retainer stem AABB")
            return
        lo, hi = aabb
        slot_y_min = SWITCH_SLOT_CENTER[1] - 0.024
        slot_y_max = SWITCH_SLOT_CENTER[1] + 0.024
        slot_z_min = SWITCH_SLOT_CENTER[2] - 0.0045
        slot_z_max = SWITCH_SLOT_CENTER[2] + 0.0045
        ctx.check(
            f"switch retainer remains in guide at {label}",
            lo[1] >= slot_y_min
            and hi[1] <= slot_y_max
            and lo[2] >= slot_z_min
            and hi[2] <= slot_z_max,
            details=f"retainer_aabb=({lo}, {hi}), slot_y=({slot_y_min}, {slot_y_max}), slot_z=({slot_z_min}, {slot_z_max})",
        )

    rest_switch_pos = ctx.part_world_position(power_switch)
    switch_is_within_slot("center")
    with ctx.pose({switch_joint: SWITCH_TRAVEL}):
        upper_switch_pos = ctx.part_world_position(power_switch)
        switch_is_within_slot("upper")
    with ctx.pose({switch_joint: -SWITCH_TRAVEL}):
        lower_switch_pos = ctx.part_world_position(power_switch)
        switch_is_within_slot("lower")
    ctx.check(
        "power switch slides along its short side slot",
        rest_switch_pos is not None
        and upper_switch_pos is not None
        and lower_switch_pos is not None
        and upper_switch_pos[1] > rest_switch_pos[1] + 0.006
        and lower_switch_pos[1] < rest_switch_pos[1] - 0.006,
        details=f"lower={lower_switch_pos}, rest={rest_switch_pos}, upper={upper_switch_pos}",
    )

    return ctx.report()


object_model = build_object_model()
