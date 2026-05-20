from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_mechanical_keyboard", assets=ASSETS)

    unit = 0.019
    field_units = 13.0
    field_width = field_units * unit
    field_depth = 5.0 * unit
    side_wall = 0.012
    front_back_wall = 0.011
    case_width = field_width + 2.0 * side_wall
    case_depth = field_depth + 2.0 * front_back_wall
    bottom_thickness = 0.004
    rim_top = 0.021
    plate_thickness = 0.0015
    plate_top = 0.018
    plate_z = plate_top - plate_thickness * 0.5
    plate_underside_z = plate_top - plate_thickness
    key_travel = 0.002
    stem_bottom_z = 0.006
    stem_height = 0.0141
    cap_height = 0.0095
    key_depth = unit - 0.002
    socket_inner = 0.006
    socket_rail = 0.0015
    socket_outer = socket_inner + 2.0 * socket_rail
    socket_height = plate_underside_z - bottom_thickness + 0.0002
    socket_center_z = bottom_thickness + socket_height * 0.5 - 0.0001

    def _section(width: float, depth: float, z: float, *, y_shift: float = 0.0) -> list[tuple[float, float, float]]:
        radius = min(width, depth) * 0.14
        return [(x, y + y_shift, z) for x, y in rounded_rect_profile(width, depth, radius, corner_segments=6)]

    def _write_keycap_mesh(mesh_name: str, width: float, depth: float, height: float):
        base = _section(width, depth, 0.0)
        shoulder = _section(width * 0.95, depth * 0.94, height * 0.45, y_shift=0.0002)
        top = _section(width * 0.84, depth * 0.80, height, y_shift=0.00055)
        return mesh_from_geometry(
            section_loft([base, shoulder, top]),
            ASSETS.mesh_path(mesh_name),
        )

    def _write_case_wall_mesh(mesh_name: str, *, width: float, lower_depth: float, upper_depth: float, height: float, upper_y_shift: float):
        lower = _section(width, lower_depth, bottom_thickness, y_shift=0.0)
        upper = _section(width, upper_depth, height, y_shift=upper_y_shift)
        return mesh_from_geometry(
            section_loft([lower, upper]),
            ASSETS.mesh_path(mesh_name),
        )

    materials = {
        "case": model.material("case_anodized", rgba=(0.16, 0.18, 0.20, 1.0)),
        "plate": model.material("plate_black", rgba=(0.08, 0.09, 0.10, 1.0)),
        "keycap": model.material("keycap_pbt", rgba=(0.90, 0.91, 0.88, 1.0)),
        "slider": model.material("slider_dark", rgba=(0.22, 0.23, 0.24, 1.0)),
    }

    case = model.part("case")
    case.visual(
        Box((case_width, case_depth, bottom_thickness)),
        origin=Origin(xyz=(0.0, 0.0, bottom_thickness * 0.5)),
        material=materials["case"],
        name="bottom_slab",
    )
    wall_height = rim_top - bottom_thickness
    wall_center_z = bottom_thickness + wall_height * 0.5
    case.visual(
        Box((side_wall, case_depth, wall_height)),
        origin=Origin(xyz=(-field_width * 0.5 - side_wall * 0.5, 0.0, wall_center_z)),
        material=materials["case"],
        name="left_wall",
    )
    case.visual(
        Box((side_wall, case_depth, wall_height)),
        origin=Origin(xyz=(field_width * 0.5 + side_wall * 0.5, 0.0, wall_center_z)),
        material=materials["case"],
        name="right_wall",
    )
    front_wall_mesh = _write_case_wall_mesh(
        "front_wall.obj",
        width=field_width,
        lower_depth=front_back_wall,
        upper_depth=front_back_wall - 0.0025,
        height=rim_top,
        upper_y_shift=0.00125,
    )
    case.visual(
        front_wall_mesh,
        origin=Origin(xyz=(0.0, -field_depth * 0.5 - front_back_wall * 0.5, 0.0)),
        material=materials["case"],
        name="front_wall",
    )
    rear_wall_mesh = _write_case_wall_mesh(
        "rear_wall.obj",
        width=field_width,
        lower_depth=front_back_wall,
        upper_depth=front_back_wall - 0.0025,
        height=rim_top,
        upper_y_shift=-0.00125,
    )
    case.visual(
        rear_wall_mesh,
        origin=Origin(xyz=(0.0, field_depth * 0.5 + front_back_wall * 0.5, 0.0)),
        material=materials["case"],
        name="rear_wall",
    )
    case.visual(
        Box((field_width, 0.001, plate_thickness)),
        origin=Origin(xyz=(0.0, field_depth * 0.5 - 0.0005, plate_z)),
        material=materials["plate"],
        name="plate_top_border",
    )
    case.visual(
        Box((field_width, 0.001, plate_thickness)),
        origin=Origin(xyz=(0.0, -field_depth * 0.5 + 0.0005, plate_z)),
        material=materials["plate"],
        name="plate_bottom_border",
    )
    for row_index in range(4):
        boundary_y = -field_depth * 0.5 + (row_index + 1) * unit
        case.visual(
            Box((field_width, 0.002, plate_thickness)),
            origin=Origin(xyz=(0.0, boundary_y, plate_z)),
            material=materials["plate"],
            name=f"plate_row_gap_{row_index}",
        )

    rows = [
        [("esc", 1.0), ("num1", 1.0), ("num2", 1.0), ("num3", 1.0), ("num4", 1.0), ("num5", 1.0), ("num6", 1.0), ("num7", 1.0), ("num8", 1.0), ("num9", 1.0), ("num0", 1.0), ("backspace", 2.0)],
        [("tab", 1.5), ("q", 1.0), ("w", 1.0), ("e", 1.0), ("r", 1.0), ("t", 1.0), ("y", 1.0), ("u", 1.0), ("i", 1.0), ("o", 1.0), ("p", 1.0), ("backslash", 1.5)],
        [("caps", 1.75), ("a", 1.0), ("s", 1.0), ("d", 1.0), ("f", 1.0), ("g", 1.0), ("h", 1.0), ("j", 1.0), ("k", 1.0), ("l", 1.0), ("enter", 2.25)],
        [("left_shift", 2.25), ("z", 1.0), ("x", 1.0), ("c", 1.0), ("v", 1.0), ("b", 1.0), ("n", 1.0), ("m", 1.0), ("slash", 1.0), ("right_shift", 2.75)],
        [("left_ctrl", 1.25), ("left_fn", 1.25), ("left_alt", 1.25), ("spacebar", 5.5), ("right_alt", 1.25), ("menu", 1.25), ("right_ctrl", 1.25)],
    ]

    for row_index, row in enumerate(rows):
        row_center_y = field_depth * 0.5 - (row_index + 0.5) * unit
        row_band_depth = unit - 0.002
        case.visual(
            Box((0.001, row_band_depth, plate_thickness)),
            origin=Origin(xyz=(-field_width * 0.5 + 0.0005, row_center_y, plate_z)),
            material=materials["plate"],
            name=f"plate_left_row_{row_index}",
        )
        case.visual(
            Box((0.001, row_band_depth, plate_thickness)),
            origin=Origin(xyz=(field_width * 0.5 - 0.0005, row_center_y, plate_z)),
            material=materials["plate"],
            name=f"plate_right_row_{row_index}",
        )
        cursor_units = 0.0
        for key_index, (_, units_wide) in enumerate(row[:-1]):
            cursor_units += units_wide
            boundary_x = -field_width * 0.5 + cursor_units * unit
            case.visual(
                Box((0.002, row_band_depth, plate_thickness)),
                origin=Origin(xyz=(boundary_x, row_center_y, plate_z)),
                material=materials["plate"],
                name=f"plate_gap_r{row_index}_c{key_index}",
            )

    case.inertial = Inertial.from_geometry(
        Box((case_width, case_depth, rim_top)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, rim_top * 0.5)),
    )

    keycap_mesh_cache: dict[float, object] = {}

    def _keycap_mesh(units_wide: float):
        mesh = keycap_mesh_cache.get(units_wide)
        if mesh is None:
            width = units_wide * unit - 0.002
            mesh = _write_keycap_mesh(
                f"keycap_{str(units_wide).replace('.', '_')}.obj",
                width=width,
                depth=key_depth,
                height=cap_height,
            )
            keycap_mesh_cache[units_wide] = mesh
        return mesh

    for row_index, row in enumerate(rows):
        row_center_y = field_depth * 0.5 - (row_index + 0.5) * unit
        cursor_units = 0.0
        for label, units_wide in row:
            part_name = f"{label}_key"
            center_x = -field_width * 0.5 + (cursor_units + units_wide * 0.5) * unit
            case.visual(
                Box((socket_rail, socket_outer, socket_height)),
                origin=Origin(xyz=(center_x - socket_inner * 0.5 - socket_rail * 0.5, row_center_y, socket_center_z)),
                material=materials["plate"],
                name=f"{label}_socket_left",
            )
            case.visual(
                Box((socket_rail, socket_outer, socket_height)),
                origin=Origin(xyz=(center_x + socket_inner * 0.5 + socket_rail * 0.5, row_center_y, socket_center_z)),
                material=materials["plate"],
                name=f"{label}_socket_right",
            )
            case.visual(
                Box((socket_inner, socket_rail, socket_height)),
                origin=Origin(xyz=(center_x, row_center_y - socket_inner * 0.5 - socket_rail * 0.5, socket_center_z)),
                material=materials["plate"],
                name=f"{label}_socket_front",
            )
            case.visual(
                Box((socket_inner, socket_rail, socket_height)),
                origin=Origin(xyz=(center_x, row_center_y + socket_inner * 0.5 + socket_rail * 0.5, socket_center_z)),
                material=materials["plate"],
                name=f"{label}_socket_back",
            )

            key_part = model.part(part_name)
            cap_width = units_wide * unit - 0.002
            key_part.visual(
                Box((0.006, 0.006, stem_height)),
                origin=Origin(xyz=(0.0, 0.0, stem_height * 0.5)),
                material=materials["slider"],
                name="slider",
            )
            if units_wide >= 2.0:
                stabilizer_offset = max(0.012, cap_width * 0.32)
                for stabilizer_name, sign in (("stabilizer_left", -1.0), ("stabilizer_right", 1.0)):
                    key_part.visual(
                        Box((0.0038, 0.0038, 0.014)),
                        origin=Origin(
                            xyz=(
                                sign * stabilizer_offset,
                                0.0,
                                0.007,
                            )
                        ),
                        material=materials["slider"],
                        name=stabilizer_name,
                    )
            key_part.visual(
                _keycap_mesh(units_wide),
                origin=Origin(xyz=(0.0, 0.0, 0.014)),
                material=materials["keycap"],
                name="cap",
            )
            key_part.inertial = Inertial.from_geometry(
                Box((cap_width, key_depth, stem_height + cap_height)),
                mass=0.018 + 0.008 * units_wide,
                origin=Origin(xyz=(0.0, 0.0, (stem_height + cap_height) * 0.5)),
            )

            model.articulation(
                f"case_to_{part_name}",
                ArticulationType.PRISMATIC,
                parent=case,
                child=key_part,
                origin=Origin(xyz=(center_x, row_center_y, stem_bottom_z)),
                axis=(0.0, 0.0, -1.0),
                motion_limits=MotionLimits(
                    effort=1.4,
                    velocity=0.08,
                    lower=0.0,
                    upper=key_travel,
                ),
            )
            cursor_units += units_wide

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    case = object_model.get_part("case")
    esc = object_model.get_part("esc_key")
    enter = object_model.get_part("enter_key")
    left_shift = object_model.get_part("left_shift_key")
    spacebar = object_model.get_part("spacebar_key")
    right_ctrl = object_model.get_part("right_ctrl_key")

    esc_joint = object_model.get_articulation("case_to_esc_key")
    enter_joint = object_model.get_articulation("case_to_enter_key")
    spacebar_joint = object_model.get_articulation("case_to_spacebar_key")

    bottom_slab = case.get_visual("bottom_slab")
    front_wall = case.get_visual("front_wall")
    plate_top_border = case.get_visual("plate_top_border")
    esc_slider = esc.get_visual("slider")
    enter_slider = enter.get_visual("slider")
    spacebar_slider = spacebar.get_visual("slider")
    esc_cap = esc.get_visual("cap")
    spacebar_cap = spacebar.get_visual("cap")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    ctx.check(
        "keyboard_part_count",
        len(object_model.parts) == 53,
        details=f"expected 53 parts including the case, found {len(object_model.parts)}",
    )
    ctx.check(
        "keyboard_articulation_count",
        len(object_model.articulations) == 52,
        details=f"expected 52 prismatic key articulations, found {len(object_model.articulations)}",
    )

    all_key_sliders_are_prismatic = all(
        articulation.articulation_type == ArticulationType.PRISMATIC
        and tuple(articulation.axis) == (0.0, 0.0, -1.0)
        and articulation.motion_limits is not None
        and articulation.motion_limits.lower == 0.0
        and articulation.motion_limits.upper == 0.002
        for articulation in object_model.articulations
    )
    ctx.check(
        "all_keys_are_vertical_prismatic_sliders",
        all_key_sliders_are_prismatic,
        details="every articulation should be a downward-travel prismatic key slider with 2 mm travel",
    )

    for key_name, key_part, slider in (
        ("esc", esc, esc_slider),
        ("enter", enter, enter_slider),
        ("left_shift", left_shift, left_shift.get_visual("slider")),
        ("spacebar", spacebar, spacebar_slider),
        ("right_ctrl", right_ctrl, right_ctrl.get_visual("slider")),
    ):
        ctx.expect_contact(
            key_part,
            case,
            elem_a=slider,
            contact_tol=1e-6,
            name=f"{key_name}_slider_contacts_switch_socket",
        )

    for key_name, key_part in (
        ("esc", esc),
        ("enter", enter),
        ("left_shift", left_shift),
        ("spacebar", spacebar),
        ("right_ctrl", right_ctrl),
    ):
        slider = key_part.get_visual("slider")
        ctx.expect_within(
            key_part,
            case,
            axes="xy",
            margin=0.0,
            inner_elem=slider,
            outer_elem=bottom_slab,
            name=f"{key_name}_slider_within_case_footprint",
        )
        ctx.expect_overlap(
            key_part,
            case,
            axes="xy",
            min_overlap=0.004,
            elem_a=slider,
            elem_b=bottom_slab,
            name=f"{key_name}_slider_over_bottom_slab",
        )

    ctx.expect_gap(
        esc,
        case,
        axis="z",
        min_gap=0.0019,
        max_gap=0.0021,
        positive_elem=esc_slider,
        negative_elem=bottom_slab,
        name="esc_rest_gap_above_bottom_slab",
    )
    ctx.expect_gap(
        enter,
        case,
        axis="z",
        min_gap=0.0019,
        max_gap=0.0021,
        positive_elem=enter_slider,
        negative_elem=bottom_slab,
        name="enter_rest_gap_above_bottom_slab",
    )
    ctx.expect_gap(
        spacebar,
        case,
        axis="z",
        min_gap=0.0019,
        max_gap=0.0021,
        positive_elem=spacebar_slider,
        negative_elem=bottom_slab,
        name="spacebar_rest_gap_above_bottom_slab",
    )

    with ctx.pose({esc_joint: 0.002}):
        ctx.expect_gap(
            esc,
            case,
            axis="z",
            min_gap=0.0,
            max_gap=0.0002,
            positive_elem=esc_slider,
            negative_elem=bottom_slab,
            name="esc_full_press_bottom_stop",
        )
    with ctx.pose({enter_joint: 0.002}):
        ctx.expect_gap(
            enter,
            case,
            axis="z",
            min_gap=0.0,
            max_gap=0.0002,
            positive_elem=enter_slider,
            negative_elem=bottom_slab,
            name="enter_full_press_bottom_stop",
        )
    with ctx.pose({spacebar_joint: 0.002}):
        ctx.expect_gap(
            spacebar,
            case,
            axis="z",
            min_gap=0.0,
            max_gap=0.0002,
            positive_elem=spacebar_slider,
            negative_elem=bottom_slab,
            name="spacebar_full_press_bottom_stop",
        )

    def _size_from_aabb(aabb):
        return (
            aabb[1][0] - aabb[0][0],
            aabb[1][1] - aabb[0][1],
            aabb[1][2] - aabb[0][2],
        )

    case_aabb = ctx.part_world_aabb(case)
    esc_cap_aabb = ctx.part_element_world_aabb(esc, elem=esc_cap)
    spacebar_cap_aabb = ctx.part_element_world_aabb(spacebar, elem=spacebar_cap)
    front_wall_aabb = ctx.part_element_world_aabb(case, elem=front_wall)
    plate_border_aabb = ctx.part_element_world_aabb(case, elem=plate_top_border)
    esc_rest_pos = ctx.part_world_position(esc)
    spacebar_rest_pos = ctx.part_world_position(spacebar)

    if case_aabb is not None:
        case_size = _size_from_aabb(case_aabb)
        ctx.check(
            "case_proportions_compact_keyboard",
            0.26 <= case_size[0] <= 0.28 and 0.11 <= case_size[1] <= 0.12 and 0.02 <= case_size[2] <= 0.022,
            details=f"case dimensions were {case_size}",
        )

    if esc_cap_aabb is not None and spacebar_cap_aabb is not None:
        esc_size = _size_from_aabb(esc_cap_aabb)
        spacebar_size = _size_from_aabb(spacebar_cap_aabb)
        ctx.check(
            "spacebar_is_wide_relative_to_regular_key",
            0.016 <= esc_size[0] <= 0.018 and 0.095 <= spacebar_size[0] <= 0.105 and spacebar_size[0] > esc_size[0] * 5.5,
            details=f"esc width={esc_size[0]}, spacebar width={spacebar_size[0]}",
        )

    if front_wall_aabb is not None and plate_border_aabb is not None:
        rim_height = front_wall_aabb[1][2]
        plate_height = plate_border_aabb[1][2]
        ctx.check(
            "top_plate_is_recessed_below_case_rim",
            0.002 <= rim_height - plate_height <= 0.004,
            details=f"rim height={rim_height}, plate height={plate_height}",
        )

    with ctx.pose({esc_joint: 0.002, spacebar_joint: 0.002}):
        esc_pressed_pos = ctx.part_world_position(esc)
        spacebar_pressed_pos = ctx.part_world_position(spacebar)
    if esc_rest_pos is not None and esc_pressed_pos is not None:
        ctx.check(
            "esc_moves_downward_on_press",
            0.0019 <= esc_rest_pos[2] - esc_pressed_pos[2] <= 0.0021,
            details=f"esc z delta={esc_rest_pos[2] - esc_pressed_pos[2]}",
        )
    if spacebar_rest_pos is not None and spacebar_pressed_pos is not None:
        ctx.check(
            "spacebar_moves_downward_on_press",
            0.0019 <= spacebar_rest_pos[2] - spacebar_pressed_pos[2] <= 0.0021,
            details=f"spacebar z delta={spacebar_rest_pos[2] - spacebar_pressed_pos[2]}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
