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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


BASE_WIDTH = 0.258
BASE_DEPTH = 0.198
FLOOR_THICKNESS = 0.003
WALL_THICKNESS = 0.004
BASE_TOP_Z = 0.019
DECK_THICKNESS = 0.002

LID_WIDTH = 0.252
LID_DEPTH = 0.194
LID_THICKNESS = 0.010
HINGE_Y = 0.097
HINGE_Z = 0.024

KEY_TRAVEL = 0.0012
NORMAL_KEY_CAP = (0.016, 0.014, 0.003)
SPACEBAR_CAP = (0.082, 0.014, 0.003)
KEY_STEM = (0.008, 0.006, 0.0035)
KEY_RETAINER = (0.012, 0.010, 0.0010)


def _translate_profile(
    profile: list[tuple[float, float]],
    dx: float,
    dy: float,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _key_specs() -> list[dict[str, object]]:
    specs: list[dict[str, object]] = []
    column_positions = (-0.054, -0.027, 0.000, 0.027, 0.054)
    row_layout = (
        (0, 0.034, -0.004),
        (1, 0.010, 0.000),
        (2, -0.014, 0.004),
    )
    for row_index, y, x_offset in row_layout:
        for column_index, base_x in enumerate(column_positions):
            specs.append(
                {
                    "part_name": f"key_r{row_index}_c{column_index}",
                    "joint_name": f"chassis_to_key_r{row_index}_c{column_index}",
                    "x": base_x + x_offset,
                    "y": y,
                    "is_spacebar": False,
                }
            )
    specs.append(
        {
            "part_name": "spacebar",
            "joint_name": "chassis_to_spacebar",
            "x": 0.0,
            "y": -0.056,
            "is_spacebar": True,
        }
    )
    return specs


KEY_SPECS = _key_specs()


def _build_deck_mesh():
    outer_profile = rounded_rect_profile(
        BASE_WIDTH - 2.0 * WALL_THICKNESS,
        BASE_DEPTH - 2.0 * WALL_THICKNESS,
        0.010,
    )
    hole_profiles: list[list[tuple[float, float]]] = []
    for spec in KEY_SPECS:
        key_x = float(spec["x"])
        key_y = float(spec["y"])
        if bool(spec["is_spacebar"]):
            for stem_offset in (-0.022, 0.022):
                hole_profiles.append(
                    _translate_profile(
                        rounded_rect_profile(0.010, 0.008, 0.0015),
                        key_x + stem_offset,
                        key_y,
                    )
                )
        else:
            hole_profiles.append(
                _translate_profile(
                    rounded_rect_profile(0.010, 0.008, 0.0015),
                    key_x,
                    key_y,
                )
            )
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            outer_profile,
            hole_profiles,
            DECK_THICKNESS,
            cap=True,
            center=False,
            closed=True,
        ),
        "education_laptop_keyboard_deck",
    )


def _build_lid_bezel_mesh():
    outer_profile = rounded_rect_profile(LID_WIDTH - 0.008, LID_DEPTH - 0.008, 0.010)
    inner_profile = rounded_rect_profile(0.196, 0.130, 0.004)
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            outer_profile,
            [inner_profile],
            0.002,
            cap=True,
            center=False,
            closed=True,
        ),
        "education_laptop_lid_bezel",
    )


def _build_keycap_mesh(width: float, depth: float, height: float, name: str):
    return mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(width, depth, min(0.0025, depth * 0.35)),
            height,
            cap=True,
            center=False,
            closed=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="education_laptop")

    chassis_color = model.material("rugged_green", rgba=(0.27, 0.33, 0.30, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.12, 0.13, 0.14, 1.0))
    key_dark = model.material("key_dark", rgba=(0.19, 0.20, 0.22, 1.0))
    screen_black = model.material("screen_black", rgba=(0.06, 0.08, 0.10, 1.0))
    glass = model.material("screen_glass", rgba=(0.17, 0.27, 0.31, 0.90))
    bumper = model.material("bumper_gray", rgba=(0.32, 0.35, 0.37, 1.0))
    slider_accent = model.material("slider_amber", rgba=(0.86, 0.56, 0.15, 1.0))

    deck_mesh = _build_deck_mesh()
    bezel_mesh = _build_lid_bezel_mesh()
    normal_keycap_mesh = _build_keycap_mesh(
        NORMAL_KEY_CAP[0],
        NORMAL_KEY_CAP[1],
        NORMAL_KEY_CAP[2],
        "education_laptop_keycap",
    )
    spacebar_mesh = _build_keycap_mesh(
        SPACEBAR_CAP[0],
        SPACEBAR_CAP[1],
        SPACEBAR_CAP[2],
        "education_laptop_spacebar",
    )

    chassis = model.part("chassis")
    chassis.visual(
        Box((BASE_WIDTH, BASE_DEPTH, FLOOR_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, FLOOR_THICKNESS / 2.0)),
        material=chassis_color,
        name="floor_pan",
    )
    chassis.visual(
        Box((BASE_WIDTH, WALL_THICKNESS, BASE_TOP_Z - FLOOR_THICKNESS)),
        origin=Origin(
            xyz=(0.0, -(BASE_DEPTH / 2.0 - WALL_THICKNESS / 2.0), (BASE_TOP_Z + FLOOR_THICKNESS) / 2.0)
        ),
        material=chassis_color,
        name="front_wall",
    )
    chassis.visual(
        Box((BASE_WIDTH, WALL_THICKNESS, BASE_TOP_Z - FLOOR_THICKNESS)),
        origin=Origin(
            xyz=(0.0, BASE_DEPTH / 2.0 - WALL_THICKNESS / 2.0, (BASE_TOP_Z + FLOOR_THICKNESS) / 2.0)
        ),
        material=chassis_color,
        name="rear_wall",
    )
    chassis.visual(
        Box((WALL_THICKNESS, BASE_DEPTH - 2.0 * WALL_THICKNESS, BASE_TOP_Z - FLOOR_THICKNESS)),
        origin=Origin(
            xyz=(
                -(BASE_WIDTH / 2.0 - WALL_THICKNESS / 2.0),
                0.0,
                (BASE_TOP_Z + FLOOR_THICKNESS) / 2.0,
            )
        ),
        material=chassis_color,
        name="left_wall",
    )

    slot_center_y = -0.045
    slot_length = 0.022
    right_wall_span = BASE_DEPTH - 2.0 * WALL_THICKNESS
    wall_bottom_z = FLOOR_THICKNESS
    wall_top_z = BASE_TOP_Z
    slot_bottom_z = 0.009
    slot_top_z = 0.013
    right_wall_x = BASE_WIDTH / 2.0 - WALL_THICKNESS / 2.0
    front_span_min = -right_wall_span / 2.0
    rear_span_max = right_wall_span / 2.0
    slot_min_y = slot_center_y - slot_length / 2.0
    slot_max_y = slot_center_y + slot_length / 2.0
    front_segment_length = slot_min_y - front_span_min
    rear_segment_length = rear_span_max - slot_max_y

    chassis.visual(
        Box((WALL_THICKNESS, front_segment_length, BASE_TOP_Z - FLOOR_THICKNESS)),
        origin=Origin(
            xyz=(
                right_wall_x,
                front_span_min + front_segment_length / 2.0,
                (BASE_TOP_Z + FLOOR_THICKNESS) / 2.0,
            )
        ),
        material=chassis_color,
        name="right_wall_front",
    )
    chassis.visual(
        Box((WALL_THICKNESS, rear_segment_length, BASE_TOP_Z - FLOOR_THICKNESS)),
        origin=Origin(
            xyz=(
                right_wall_x,
                slot_max_y + rear_segment_length / 2.0,
                (BASE_TOP_Z + FLOOR_THICKNESS) / 2.0,
            )
        ),
        material=chassis_color,
        name="right_wall_rear",
    )
    chassis.visual(
        Box((WALL_THICKNESS, slot_length, slot_bottom_z - wall_bottom_z)),
        origin=Origin(
            xyz=(
                right_wall_x,
                slot_center_y,
                wall_bottom_z + (slot_bottom_z - wall_bottom_z) / 2.0,
            )
        ),
        material=chassis_color,
        name="right_wall_lower_rail",
    )
    chassis.visual(
        Box((WALL_THICKNESS, slot_length, wall_top_z - slot_top_z)),
        origin=Origin(
            xyz=(
                right_wall_x,
                slot_center_y,
                slot_top_z + (wall_top_z - slot_top_z) / 2.0,
            )
        ),
        material=chassis_color,
        name="right_wall_upper_rail",
    )

    chassis.visual(
        deck_mesh,
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z - DECK_THICKNESS)),
        material=trim_dark,
        name="deck_plate",
    )
    chassis.visual(
        Box((0.010, 0.010, 0.004)),
        origin=Origin(xyz=(-0.104, -0.083, 0.021)),
        material=bumper,
        name="bumper_front_left",
    )
    chassis.visual(
        Box((0.010, 0.010, 0.004)),
        origin=Origin(xyz=(0.104, -0.083, 0.021)),
        material=bumper,
        name="bumper_front_right",
    )
    chassis.visual(
        Box((0.010, 0.010, 0.004)),
        origin=Origin(xyz=(-0.104, 0.073, 0.021)),
        material=bumper,
        name="bumper_rear_left",
    )
    chassis.visual(
        Box((0.010, 0.010, 0.004)),
        origin=Origin(xyz=(0.104, 0.073, 0.021)),
        material=bumper,
        name="bumper_rear_right",
    )
    chassis.visual(
        Box((0.026, 0.012, 0.005)),
        origin=Origin(xyz=(-0.082, 0.091, 0.0215)),
        material=trim_dark,
        name="left_hinge_pedestal",
    )
    chassis.visual(
        Box((0.026, 0.012, 0.005)),
        origin=Origin(xyz=(0.082, 0.091, 0.0215)),
        material=trim_dark,
        name="right_hinge_pedestal",
    )
    chassis.inertial = Inertial.from_geometry(
        Box((BASE_WIDTH, BASE_DEPTH, BASE_TOP_Z)),
        mass=1.45,
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z / 2.0)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((LID_WIDTH, LID_DEPTH, 0.002)),
        origin=Origin(xyz=(0.0, -LID_DEPTH / 2.0, 0.009)),
        material=chassis_color,
        name="outer_shell",
    )
    lid.visual(
        Box((0.004, LID_DEPTH, LID_THICKNESS)),
        origin=Origin(xyz=(-(LID_WIDTH / 2.0 - 0.002), -LID_DEPTH / 2.0, LID_THICKNESS / 2.0)),
        material=chassis_color,
        name="left_edge",
    )
    lid.visual(
        Box((0.004, LID_DEPTH, LID_THICKNESS)),
        origin=Origin(xyz=((LID_WIDTH / 2.0 - 0.002), -LID_DEPTH / 2.0, LID_THICKNESS / 2.0)),
        material=chassis_color,
        name="right_edge",
    )
    lid.visual(
        Box((LID_WIDTH - 0.008, 0.004, LID_THICKNESS)),
        origin=Origin(xyz=(0.0, -(LID_DEPTH - 0.002), LID_THICKNESS / 2.0)),
        material=chassis_color,
        name="front_edge",
    )
    lid.visual(
        Box((0.026, 0.010, 0.008)),
        origin=Origin(xyz=(-0.082, -0.003, 0.004)),
        material=trim_dark,
        name="left_hinge_leaf",
    )
    lid.visual(
        Box((0.026, 0.010, 0.008)),
        origin=Origin(xyz=(0.082, -0.003, 0.004)),
        material=trim_dark,
        name="right_hinge_leaf",
    )
    lid.visual(
        bezel_mesh,
        origin=Origin(xyz=(0.0, -LID_DEPTH / 2.0, 0.0)),
        material=trim_dark,
        name="bezel_frame",
    )
    lid.visual(
        Box((0.196, 0.130, 0.0012)),
        origin=Origin(xyz=(0.0, -LID_DEPTH / 2.0, 0.0014)),
        material=glass,
        name="screen_panel",
    )
    lid.visual(
        Box((0.170, 0.004, 0.001)),
        origin=Origin(xyz=(0.0, -0.028, 0.0015)),
        material=screen_black,
        name="camera_bar",
    )
    lid.inertial = Inertial.from_geometry(
        Box((LID_WIDTH, LID_DEPTH, LID_THICKNESS)),
        mass=0.62,
        origin=Origin(xyz=(0.0, -LID_DEPTH / 2.0, LID_THICKNESS / 2.0)),
    )

    model.articulation(
        "chassis_to_lid",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=2.2,
        ),
    )

    power_slider = model.part("power_slider")
    power_slider.visual(
        Box((0.008, 0.010, 0.005)),
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
        material=slider_accent,
        name="slider_thumb",
    )
    power_slider.visual(
        Box((0.004, 0.012, 0.0036)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=trim_dark,
        name="slider_shank",
    )
    power_slider.visual(
        Box((0.006, 0.028, 0.005)),
        origin=Origin(xyz=(-0.005, 0.0, 0.0)),
        material=trim_dark,
        name="slider_retainer",
    )
    power_slider.inertial = Inertial.from_geometry(
        Box((0.019, 0.028, 0.005)),
        mass=0.014,
        origin=Origin(xyz=(0.0005, 0.0, 0.0)),
    )
    model.articulation(
        "chassis_to_power_slider",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=power_slider,
        origin=Origin(xyz=(right_wall_x, slot_center_y, 0.011)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.05,
            lower=-0.0045,
            upper=0.0045,
        ),
    )

    for spec in KEY_SPECS:
        part_name = str(spec["part_name"])
        joint_name = str(spec["joint_name"])
        key_part = model.part(part_name)
        is_spacebar = bool(spec["is_spacebar"])
        if is_spacebar:
            key_part.visual(
                spacebar_mesh,
                origin=Origin(xyz=(0.0, 0.0, 0.0015)),
                material=key_dark,
                name="main_cap",
            )
            for suffix, x_offset in (("left", -0.022), ("right", 0.022)):
                key_part.visual(
                    Box((0.008, 0.006, 0.0035)),
                    origin=Origin(xyz=(x_offset, 0.0, -0.00025)),
                    material=trim_dark,
                    name=f"stem_{suffix}",
                )
                key_part.visual(
                    Box((0.012, 0.010, 0.0010)),
                    origin=Origin(xyz=(x_offset, 0.0, -0.0025)),
                    material=trim_dark,
                    name=f"retainer_{suffix}",
                )
            key_part.inertial = Inertial.from_geometry(
                Box((SPACEBAR_CAP[0], SPACEBAR_CAP[1], 0.0065)),
                mass=0.030,
                origin=Origin(xyz=(0.0, 0.0, 0.0005)),
            )
        else:
            key_part.visual(
                normal_keycap_mesh,
                origin=Origin(xyz=(0.0, 0.0, 0.0015)),
                material=key_dark,
                name="main_cap",
            )
            key_part.visual(
                Box(KEY_STEM),
                origin=Origin(xyz=(0.0, 0.0, -0.00025)),
                material=trim_dark,
                name="stem",
            )
            key_part.visual(
                Box(KEY_RETAINER),
                origin=Origin(xyz=(0.0, 0.0, -0.0025)),
                material=trim_dark,
                name="retainer",
            )
            key_part.inertial = Inertial.from_geometry(
                Box((NORMAL_KEY_CAP[0], NORMAL_KEY_CAP[1], 0.0065)),
                mass=0.012,
                origin=Origin(xyz=(0.0, 0.0, 0.0005)),
            )

        model.articulation(
            joint_name,
            ArticulationType.PRISMATIC,
            parent=chassis,
            child=key_part,
            origin=Origin(xyz=(float(spec["x"]), float(spec["y"]), BASE_TOP_Z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=1.0,
                velocity=0.06,
                lower=0.0,
                upper=KEY_TRAVEL,
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

    chassis = object_model.get_part("chassis")
    lid = object_model.get_part("lid")
    power_slider = object_model.get_part("power_slider")

    lid_hinge = object_model.get_articulation("chassis_to_lid")
    slider_joint = object_model.get_articulation("chassis_to_power_slider")
    reference_key = object_model.get_part("key_r1_c2")
    reference_key_joint = object_model.get_articulation("chassis_to_key_r1_c2")
    spacebar = object_model.get_part("spacebar")
    spacebar_joint = object_model.get_articulation("chassis_to_spacebar")

    ctx.expect_contact(lid, chassis, name="lid_contacts_bumpers_when_closed")
    ctx.expect_contact(power_slider, chassis, name="power_slider_clipped_into_side_slot")
    ctx.expect_contact(reference_key, chassis, name="reference_key_captured_in_deck")
    ctx.expect_contact(spacebar, chassis, name="spacebar_captured_in_deck")

    for spec in KEY_SPECS:
        key_part = object_model.get_part(str(spec["part_name"]))
        key_joint = object_model.get_articulation(str(spec["joint_name"]))
        ctx.expect_contact(
            key_part,
            chassis,
            name=f"{key_part.name}_retained_by_keyboard_deck",
        )
        axis_ok = tuple(round(value, 6) for value in key_joint.axis) == (0.0, 0.0, -1.0)
        ctx.check(
            f"{key_joint.name}_moves_vertically",
            axis_ok,
            details=f"Expected axis (0, 0, -1), got {key_joint.axis}",
        )

    lid_limits = lid_hinge.motion_limits
    slider_limits = slider_joint.motion_limits
    ctx.check(
        "lid_hinge_axis_is_rear_width_axis",
        tuple(round(abs(value), 6) for value in lid_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"Expected hinge axis parallel to x, got {lid_hinge.axis}",
    )
    ctx.check(
        "lid_hinge_has_realistic_opening_range",
        lid_limits is not None
        and lid_limits.lower == 0.0
        and lid_limits.upper is not None
        and 1.8 <= lid_limits.upper <= 2.3,
        details=f"Unexpected lid hinge limits: {lid_limits}",
    )
    ctx.check(
        "power_slider_axis_runs_along_side_slot",
        tuple(round(value, 6) for value in slider_joint.axis) == (0.0, 1.0, 0.0),
        details=f"Expected slider axis along +y, got {slider_joint.axis}",
    )
    ctx.check(
        "power_slider_has_short_throw",
        slider_limits is not None
        and slider_limits.lower is not None
        and slider_limits.upper is not None
        and abs(slider_limits.lower + slider_limits.upper) < 1e-9
        and 0.008 <= (slider_limits.upper - slider_limits.lower) <= 0.011,
        details=f"Unexpected slider limits: {slider_limits}",
    )

    key_rest = ctx.part_world_position(reference_key)
    assert key_rest is not None
    with ctx.pose({reference_key_joint: KEY_TRAVEL}):
        key_pressed = ctx.part_world_position(reference_key)
        assert key_pressed is not None
        ctx.check(
            "reference_key_plunges_downward",
            key_pressed[2] < key_rest[2] - 0.0009,
            details=f"Key did not move downward enough: rest={key_rest}, pressed={key_pressed}",
        )
        ctx.expect_gap(
            reference_key,
            chassis,
            axis="z",
            min_gap=0.0002,
            positive_elem="main_cap",
            negative_elem="deck_plate",
            name="pressed_keycap_stays_clear_of_deck",
        )

    spacebar_rest = ctx.part_world_position(spacebar)
    assert spacebar_rest is not None
    with ctx.pose({spacebar_joint: KEY_TRAVEL}):
        spacebar_pressed = ctx.part_world_position(spacebar)
        assert spacebar_pressed is not None
        ctx.check(
            "spacebar_plunges_downward",
            spacebar_pressed[2] < spacebar_rest[2] - 0.0009,
            details=f"Spacebar did not move downward enough: rest={spacebar_rest}, pressed={spacebar_pressed}",
        )

    slider_rest = ctx.part_world_position(power_slider)
    assert slider_rest is not None
    with ctx.pose({slider_joint: 0.0045}):
        slider_on = ctx.part_world_position(power_slider)
        assert slider_on is not None
        ctx.check(
            "power_slider_translates_along_slot",
            slider_on[1] > slider_rest[1] + 0.0035 and abs(slider_on[0] - slider_rest[0]) < 1e-6,
            details=f"Slider did not move correctly in its slot: rest={slider_rest}, on={slider_on}",
        )
        ctx.expect_contact(
            power_slider,
            chassis,
            name="power_slider_stays_clipped_when_moved",
        )

    with ctx.pose({lid_hinge: math.radians(112.0)}):
        ctx.expect_gap(
            lid,
            chassis,
            axis="z",
            min_gap=0.160,
            positive_elem="front_edge",
            name="opened_lid_front_edge_lifts_clear_of_keyboard_deck",
        )
        ctx.expect_overlap(
            lid,
            chassis,
            axes="x",
            min_overlap=0.180,
            elem_a="screen_panel",
            name="opened_lid_stays_centered_over_chassis_width",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
