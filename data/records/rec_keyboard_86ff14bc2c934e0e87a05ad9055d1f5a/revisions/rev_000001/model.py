from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


CASE_WIDTH = 0.318
CASE_DEPTH = 0.136
CASE_BASE_THICKNESS = 0.004

KEY_CHANNEL_CENTERS = (-0.056, -0.037, -0.018, 0.001, 0.020)
KEY_CHANNEL_DEPTH = 0.012
KEY_TRAVEL = 0.001
KEY_REST_Z = 0.007
KEY_STEM_HEIGHT = 0.0055
KEY_STEM_DEPTH = 0.012
KEY_CAP_HEIGHT = 0.0045
KEY_CAP_DEPTH = 0.0155
KEY_CAP_WIDTH = 0.026
SPACEBAR_WIDTH = 0.086


def _build_key_specs() -> list[dict[str, float | str]]:
    specs: list[dict[str, float | str]] = []

    bottom_row = (
        ("mod_left", -0.104, 0.022),
        ("mod_left_inner", -0.074, 0.022),
        ("spacebar", -0.002, SPACEBAR_WIDTH),
        ("mod_right_inner", 0.072, 0.022),
        ("mod_right", 0.102, 0.022),
    )
    for name, x, width in bottom_row:
        specs.append({"name": name, "x": x, "y": KEY_CHANNEL_CENTERS[0], "width": width})

    staggered_rows = (
        ("row_1", KEY_CHANNEL_CENTERS[1], (-0.088, -0.056, -0.024, 0.008, 0.040, 0.072, 0.104)),
        ("row_2", KEY_CHANNEL_CENTERS[2], (-0.094, -0.062, -0.030, 0.002, 0.034, 0.066, 0.098)),
        ("row_3", KEY_CHANNEL_CENTERS[3], (-0.100, -0.068, -0.036, -0.004, 0.028, 0.060, 0.092)),
        ("row_4", KEY_CHANNEL_CENTERS[4], (-0.106, -0.074, -0.042, -0.010, 0.022, 0.054, 0.086)),
    )
    for row_name, y, centers in staggered_rows:
        for index, x in enumerate(centers):
            specs.append(
                {
                    "name": f"{row_name}_key_{index}",
                    "x": x,
                    "y": y,
                    "width": KEY_CAP_WIDTH,
                }
            )
    return specs


KEY_SPECS = _build_key_specs()


def _key_joint_name(part_name: str) -> str:
    return f"deck_frame_to_{part_name}"


def _build_keycap_mesh(name: str, width: float, depth: float, height: float):
    return mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(width, depth, radius=min(width, depth) * 0.16, corner_segments=8),
            height,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_office_keyboard")

    case_black = model.material("case_black", rgba=(0.13, 0.14, 0.16, 1.0))
    inner_black = model.material("inner_black", rgba=(0.09, 0.10, 0.11, 1.0))
    deck_grey = model.material("deck_grey", rgba=(0.18, 0.19, 0.21, 1.0))
    key_grey = model.material("key_grey", rgba=(0.76, 0.78, 0.80, 1.0))
    stem_dark = model.material("stem_dark", rgba=(0.22, 0.23, 0.24, 1.0))
    switch_track = model.material("switch_track", rgba=(0.20, 0.21, 0.22, 1.0))
    switch_green = model.material("switch_green", rgba=(0.39, 0.62, 0.50, 1.0))

    small_key_mesh = _build_keycap_mesh(
        "office_keyboard_small_keycap",
        width=KEY_CAP_WIDTH,
        depth=KEY_CAP_DEPTH,
        height=KEY_CAP_HEIGHT,
    )
    spacebar_mesh = _build_keycap_mesh(
        "office_keyboard_spacebar",
        width=SPACEBAR_WIDTH,
        depth=KEY_CAP_DEPTH,
        height=KEY_CAP_HEIGHT,
    )

    case = model.part("case")
    case.visual(
        Box((CASE_WIDTH, CASE_DEPTH, CASE_BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, CASE_BASE_THICKNESS * 0.5)),
        material=case_black,
        name="base_plate",
    )
    case.visual(
        Box((CASE_WIDTH, 0.003, 0.011)),
        origin=Origin(xyz=(0.0, -0.0705, 0.0055)),
        material=case_black,
        name="front_skirt",
    )
    case.visual(
        Box((0.006, 0.119, 0.018)),
        origin=Origin(xyz=(-0.156, -0.0095, 0.009)),
        material=case_black,
        name="left_wall",
    )
    case.visual(
        Box((0.006, 0.093, 0.018)),
        origin=Origin(xyz=(0.156, -0.0225, 0.009)),
        material=case_black,
        name="right_wall_front",
    )
    case.visual(
        Box((0.006, 0.024, 0.024)),
        origin=Origin(xyz=(0.156, 0.056, 0.012)),
        material=case_black,
        name="right_wall_rear",
    )
    case.visual(
        Box((0.006, 0.020, 0.010)),
        origin=Origin(xyz=(0.156, 0.034, 0.005)),
        material=case_black,
        name="right_wall_lower_bridge",
    )
    case.visual(
        Box((0.006, 0.020, 0.008)),
        origin=Origin(xyz=(0.156, 0.034, 0.020)),
        material=case_black,
        name="right_wall_upper_bridge",
    )
    case.visual(
        Box((CASE_WIDTH, 0.024, 0.011)),
        origin=Origin(xyz=(0.0, 0.056, 0.0055)),
        material=case_black,
        name="rear_body",
    )
    case.visual(
        Box((CASE_WIDTH, 0.005, 0.007)),
        origin=Origin(xyz=(0.0, 0.0455, 0.0145)),
        material=case_black,
        name="slot_front_lip",
    )
    case.visual(
        Box((CASE_WIDTH, 0.004, 0.013)),
        origin=Origin(xyz=(0.0, 0.066, 0.0175)),
        material=case_black,
        name="slot_back_wall",
    )
    case.visual(
        Box((0.298, 0.018, 0.002)),
        origin=Origin(xyz=(0.0, 0.056, 0.012)),
        material=inner_black,
        name="phone_shelf_floor",
    )
    case.inertial = Inertial.from_geometry(
        Box((CASE_WIDTH, CASE_DEPTH, 0.026)),
        mass=0.78,
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
    )

    deck_frame = model.part("deck_frame")
    deck_frame.visual(
        Box((0.280, 0.104, 0.002)),
        origin=Origin(xyz=(0.0, -0.016, 0.005)),
        material=inner_black,
        name="tray_floor",
    )
    deck_frame.visual(
        Box((0.280, 0.007, 0.007)),
        origin=Origin(xyz=(0.0, -0.0655, 0.0095)),
        material=deck_grey,
        name="front_beam",
    )
    for beam_index, beam_y in enumerate((-0.0465, -0.0275, -0.0085, 0.0105), start=1):
        deck_frame.visual(
            Box((0.280, 0.007, 0.007)),
            origin=Origin(xyz=(0.0, beam_y, 0.0095)),
            material=deck_grey,
            name=f"row_divider_{beam_index}",
        )
    deck_frame.visual(
        Box((0.280, 0.007, 0.007)),
        origin=Origin(xyz=(0.0, 0.0295, 0.0095)),
        material=deck_grey,
        name="rear_beam",
    )
    deck_frame.visual(
        Box((0.012, 0.104, 0.009)),
        origin=Origin(xyz=(-0.134, -0.016, 0.0085)),
        material=deck_grey,
        name="left_rail",
    )
    deck_frame.visual(
        Box((0.012, 0.104, 0.009)),
        origin=Origin(xyz=(0.134, -0.016, 0.0085)),
        material=deck_grey,
        name="right_rail",
    )
    deck_frame.inertial = Inertial.from_geometry(
        Box((0.280, 0.104, 0.011)),
        mass=0.18,
        origin=Origin(xyz=(0.0, -0.016, 0.0055)),
    )

    model.articulation(
        "case_to_deck_frame",
        ArticulationType.FIXED,
        parent=case,
        child=deck_frame,
        origin=Origin(),
    )

    for spec in KEY_SPECS:
        part_name = str(spec["name"])
        key_width = float(spec["width"])
        key_part = model.part(part_name)
        key_part.visual(
            Box((max(key_width - 0.010, 0.012), KEY_STEM_DEPTH, KEY_STEM_HEIGHT)),
            origin=Origin(xyz=(0.0, 0.0, KEY_STEM_HEIGHT * 0.5)),
            material=stem_dark,
            name="plunger",
        )
        key_part.visual(
            spacebar_mesh if part_name == "spacebar" else small_key_mesh,
            origin=Origin(xyz=(0.0, 0.0, KEY_STEM_HEIGHT)),
            material=key_grey,
            name="keycap",
        )
        key_part.inertial = Inertial.from_geometry(
            Box((key_width, KEY_CAP_DEPTH, KEY_STEM_HEIGHT + KEY_CAP_HEIGHT)),
            mass=0.012 if part_name == "spacebar" else 0.006,
            origin=Origin(xyz=(0.0, 0.0, (KEY_STEM_HEIGHT + KEY_CAP_HEIGHT) * 0.5)),
        )
        model.articulation(
            _key_joint_name(part_name),
            ArticulationType.PRISMATIC,
            parent=deck_frame,
            child=key_part,
            origin=Origin(xyz=(float(spec["x"]), float(spec["y"]), KEY_REST_Z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=0.8,
                velocity=0.05,
                lower=0.0,
                upper=KEY_TRAVEL,
            ),
        )

    switch_guide = model.part("switch_guide")
    switch_guide.visual(
        Box((0.002, 0.024, 0.010)),
        origin=Origin(xyz=(-0.006, 0.0, 0.0)),
        material=switch_track,
        name="backplate",
    )
    switch_guide.visual(
        Box((0.012, 0.024, 0.001)),
        origin=Origin(xyz=(0.0, 0.0, 0.0035)),
        material=switch_track,
        name="top_rail",
    )
    switch_guide.visual(
        Box((0.012, 0.024, 0.001)),
        origin=Origin(xyz=(0.0, 0.0, -0.0035)),
        material=switch_track,
        name="bottom_rail",
    )
    switch_guide.visual(
        Box((0.008, 0.001, 0.010)),
        origin=Origin(xyz=(-0.002, -0.0105, 0.0)),
        material=switch_track,
        name="front_stop",
    )
    switch_guide.visual(
        Box((0.008, 0.001, 0.010)),
        origin=Origin(xyz=(-0.002, 0.0105, 0.0)),
        material=switch_track,
        name="rear_stop",
    )
    switch_guide.inertial = Inertial.from_geometry(
        Box((0.012, 0.024, 0.010)),
        mass=0.01,
        origin=Origin(),
    )
    model.articulation(
        "case_to_switch_guide",
        ArticulationType.FIXED,
        parent=case,
        child=switch_guide,
        origin=Origin(xyz=(0.154, 0.034, 0.013)),
    )

    power_switch = model.part("power_switch")
    power_switch.visual(
        Box((0.010, 0.008, 0.006)),
        origin=Origin(),
        material=switch_green,
        name="slider_body",
    )
    power_switch.visual(
        Box((0.004, 0.010, 0.004)),
        origin=Origin(xyz=(0.007, 0.0, 0.0)),
        material=switch_green,
        name="thumb_tab",
    )
    power_switch.inertial = Inertial.from_geometry(
        Box((0.014, 0.010, 0.006)),
        mass=0.004,
        origin=Origin(),
    )
    model.articulation(
        "switch_guide_to_power_switch",
        ArticulationType.PRISMATIC,
        parent=switch_guide,
        child=power_switch,
        origin=Origin(xyz=(0.0, -0.005, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.3,
            velocity=0.04,
            lower=0.0,
            upper=0.010,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    case = object_model.get_part("case")
    deck_frame = object_model.get_part("deck_frame")
    switch_guide = object_model.get_part("switch_guide")
    power_switch = object_model.get_part("power_switch")

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

    ctx.expect_contact(deck_frame, case, name="deck_frame_supported_by_case")
    ctx.expect_contact(switch_guide, case, name="switch_guide_seated_in_case_wall")
    ctx.expect_contact(power_switch, switch_guide, name="power_switch_captured_in_guide")
    ctx.expect_overlap(
        power_switch,
        switch_guide,
        axes="yz",
        min_overlap=0.006,
        name="power_switch_stays_in_guide_slot",
    )

    for spec in KEY_SPECS:
        part_name = str(spec["name"])
        key_part = object_model.get_part(part_name)
        key_joint = object_model.get_articulation(_key_joint_name(part_name))
        ctx.expect_contact(key_part, deck_frame, name=f"{part_name}_seated_in_key_trough")
        ctx.expect_overlap(
            key_part,
            deck_frame,
            axes="xy",
            min_overlap=0.010,
            name=f"{part_name}_over_keybed",
        )
        limits = key_joint.motion_limits
        ctx.check(
            f"{key_joint.name}_axis",
            tuple(round(value, 6) for value in key_joint.axis) == (0.0, 0.0, -1.0),
            f"Expected key travel on -Z, got axis={key_joint.axis!r}",
        )
        ctx.check(
            f"{key_joint.name}_travel",
            limits is not None
            and limits.lower == 0.0
            and limits.upper is not None
            and abs(limits.upper - KEY_TRAVEL) < 1e-9,
            f"Expected 0 to {KEY_TRAVEL:.4f} m travel, got limits={limits!r}",
        )

    sample_key = object_model.get_part("row_2_key_3")
    sample_key_joint = object_model.get_articulation(_key_joint_name("row_2_key_3"))
    sample_key_rest = ctx.part_world_position(sample_key)
    assert sample_key_rest is not None
    with ctx.pose({sample_key_joint: KEY_TRAVEL}):
        sample_key_pressed = ctx.part_world_position(sample_key)
        assert sample_key_pressed is not None
        ctx.check(
            "sample_key_moves_downward",
            sample_key_pressed[2] < sample_key_rest[2] - 0.0008,
            f"Expected downward travel, got rest={sample_key_rest}, pressed={sample_key_pressed}",
        )
        ctx.expect_contact(sample_key, deck_frame, name="sample_key_remains_guided_when_pressed")

    spacebar = object_model.get_part("spacebar")
    spacebar_joint = object_model.get_articulation(_key_joint_name("spacebar"))
    spacebar_rest = ctx.part_world_position(spacebar)
    assert spacebar_rest is not None
    with ctx.pose({spacebar_joint: KEY_TRAVEL}):
        spacebar_pressed = ctx.part_world_position(spacebar)
        assert spacebar_pressed is not None
        ctx.check(
            "spacebar_moves_downward",
            spacebar_pressed[2] < spacebar_rest[2] - 0.0008,
            f"Expected spacebar downward travel, got rest={spacebar_rest}, pressed={spacebar_pressed}",
        )
        ctx.expect_contact(spacebar, deck_frame, name="spacebar_remains_supported_when_pressed")

    switch_joint = object_model.get_articulation("switch_guide_to_power_switch")
    switch_limits = switch_joint.motion_limits
    ctx.check(
        "power_switch_axis",
        tuple(round(value, 6) for value in switch_joint.axis) == (0.0, 1.0, 0.0),
        f"Expected side switch travel on +Y, got axis={switch_joint.axis!r}",
    )
    ctx.check(
        "power_switch_travel_limits",
        switch_limits is not None
        and switch_limits.lower == 0.0
        and switch_limits.upper is not None
        and abs(switch_limits.upper - 0.010) < 1e-9,
        f"Expected 0 to 0.010 m slider travel, got limits={switch_limits!r}",
    )
    switch_rest = ctx.part_world_position(power_switch)
    assert switch_rest is not None
    with ctx.pose({switch_joint: 0.010}):
        switch_on = ctx.part_world_position(power_switch)
        assert switch_on is not None
        ctx.check(
            "power_switch_translates_in_side_slot",
            switch_on[1] > switch_rest[1] + 0.008 and abs(switch_on[2] - switch_rest[2]) < 1e-6,
            f"Expected pure side-slot translation, got rest={switch_rest}, on={switch_on}",
        )
        ctx.expect_contact(power_switch, switch_guide, name="power_switch_remains_clipped_at_full_travel")
        ctx.expect_overlap(
            power_switch,
            switch_guide,
            axes="yz",
            min_overlap=0.006,
            name="power_switch_stays_captured_at_full_travel",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
