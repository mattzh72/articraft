from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ExtrudeWithHolesGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _vec_sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _dot(a, b) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _normalize(v):
    length = max((_dot(v, v)) ** 0.5, 1e-9)
    return (v[0] / length, v[1] / length, v[2] / length)


def _rect_profile(width: float, height: float, *, cx: float = 0.0, cy: float = 0.0):
    half_w = width * 0.5
    half_h = height * 0.5
    return [
        (cx - half_w, cy - half_h),
        (cx + half_w, cy - half_h),
        (cx + half_w, cy + half_h),
        (cx - half_w, cy + half_h),
    ]


def _panel_prism(
    corners: list[tuple[float, float, float]],
    *,
    thickness: float,
    interior_hint: tuple[float, float, float],
) -> MeshGeometry:
    pts = list(corners)
    normal = _normalize(_cross(_vec_sub(pts[1], pts[0]), _vec_sub(pts[2], pts[0])))
    centroid = (
        sum(point[0] for point in pts) / 4.0,
        sum(point[1] for point in pts) / 4.0,
        sum(point[2] for point in pts) / 4.0,
    )
    if _dot(_vec_sub(interior_hint, centroid), normal) > 0.0:
        pts = list(reversed(pts))
        normal = _normalize(_cross(_vec_sub(pts[1], pts[0]), _vec_sub(pts[2], pts[0])))

    inner_pts = [
        (
            point[0] - normal[0] * thickness,
            point[1] - normal[1] * thickness,
            point[2] - normal[2] * thickness,
        )
        for point in pts
    ]

    geom = MeshGeometry()
    outer_ids = [geom.add_vertex(*point) for point in pts]
    inner_ids = [geom.add_vertex(*point) for point in inner_pts]

    _add_quad(geom, outer_ids[0], outer_ids[1], outer_ids[2], outer_ids[3])
    _add_quad(geom, inner_ids[3], inner_ids[2], inner_ids[1], inner_ids[0])
    for index in range(4):
        next_index = (index + 1) % 4
        _add_quad(
            geom,
            outer_ids[index],
            outer_ids[next_index],
            inner_ids[next_index],
            inner_ids[index],
        )
    return geom


def _build_canopy_shell_mesh() -> MeshGeometry:
    bottom_width = 0.90
    bottom_depth = 0.50
    top_width = 0.42
    top_depth = 0.26
    canopy_height = 0.26
    front_fascia_rear_y = 0.232
    front_fascia_top_z = 0.050
    shell_thickness = 0.012
    interior_hint = (0.0, 0.0, 0.135)

    bfl = (-bottom_width * 0.5, bottom_depth * 0.5, 0.0)
    bfr = (bottom_width * 0.5, bottom_depth * 0.5, 0.0)
    bbr = (bottom_width * 0.5, -bottom_depth * 0.5, 0.0)
    bbl = (-bottom_width * 0.5, -bottom_depth * 0.5, 0.0)

    ufl = (-bottom_width * 0.5, front_fascia_rear_y, front_fascia_top_z)
    ufr = (bottom_width * 0.5, front_fascia_rear_y, front_fascia_top_z)

    tfl = (-top_width * 0.5, top_depth * 0.5, canopy_height)
    tfr = (top_width * 0.5, top_depth * 0.5, canopy_height)
    tbr = (top_width * 0.5, -top_depth * 0.5, canopy_height)
    tbl = (-top_width * 0.5, -top_depth * 0.5, canopy_height)

    shell = _panel_prism(
        [ufl, ufr, tfr, tfl],
        thickness=shell_thickness,
        interior_hint=interior_hint,
    )
    shell.merge(
        _panel_prism(
            [bfr, bbr, tbr, tfr],
            thickness=shell_thickness,
            interior_hint=interior_hint,
        )
    )
    shell.merge(
        _panel_prism(
            [bbl, bfl, tfl, tbl],
            thickness=shell_thickness,
            interior_hint=interior_hint,
        )
    )
    shell.merge(
        _panel_prism(
            [bbr, bbl, tbl, tbr],
            thickness=shell_thickness,
            interior_hint=interior_hint,
        )
    )
    shell.merge(
        _panel_prism(
            [tfl, tfr, tbr, tbl],
            thickness=0.014,
            interior_hint=interior_hint,
        )
    )
    return shell


def _build_control_panel_plate_mesh() -> MeshGeometry:
    panel_width = 0.36
    panel_height = 0.022
    panel_thickness = 0.003
    button_pitch = 0.058
    hole_width = 0.025
    hole_height = 0.0085

    hole_profiles = [
        _rect_profile(
            hole_width,
            hole_height,
            cx=(-2 + index) * button_pitch,
            cy=0.0,
        )
        for index in range(5)
    ]
    geom = ExtrudeWithHolesGeometry(
        _rect_profile(panel_width, panel_height),
        hole_profiles,
        panel_thickness,
        center=True,
        cap=True,
        closed=True,
    )
    geom.rotate_x(3.141592653589793 / 2.0)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chimney_range_hood")

    brushed_steel = model.material("brushed_steel", rgba=(0.77, 0.79, 0.81, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.68, 0.70, 0.73, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.17, 0.18, 0.20, 1.0))
    charcoal = model.material("charcoal", rgba=(0.13, 0.14, 0.15, 1.0))

    canopy = model.part("canopy")
    canopy.visual(
        mesh_from_geometry(_build_canopy_shell_mesh(), "range_hood_canopy_shell"),
        material=brushed_steel,
        name="canopy_shell",
    )

    fascia_depth = 0.018
    fascia_front_y = 0.250
    fascia_center_y = fascia_front_y - fascia_depth * 0.5
    canopy.visual(
        Box((0.90, fascia_depth, 0.014)),
        origin=Origin(xyz=(0.0, fascia_center_y, 0.007)),
        material=brushed_steel,
        name="front_fascia_bottom_rail",
    )
    canopy.visual(
        Box((0.90, fascia_depth, 0.014)),
        origin=Origin(xyz=(0.0, fascia_center_y, 0.043)),
        material=brushed_steel,
        name="front_fascia_top_rail",
    )
    canopy.visual(
        Box((0.27, fascia_depth, 0.022)),
        origin=Origin(xyz=(-0.315, fascia_center_y, 0.025)),
        material=brushed_steel,
        name="front_fascia_left_block",
    )
    canopy.visual(
        Box((0.27, fascia_depth, 0.022)),
        origin=Origin(xyz=(0.315, fascia_center_y, 0.025)),
        material=brushed_steel,
        name="front_fascia_right_block",
    )
    canopy.visual(
        mesh_from_geometry(_build_control_panel_plate_mesh(), "range_hood_control_panel"),
        origin=Origin(xyz=(0.0, fascia_front_y - 0.0015, 0.025)),
        material=satin_steel,
        name="control_panel_front",
    )
    canopy.visual(
        Box((0.36, 0.0025, 0.022)),
        origin=Origin(xyz=(0.0, 0.231, 0.025)),
        material=dark_trim,
        name="control_panel_shadow_line",
    )
    canopy.inertial = Inertial.from_geometry(
        Box((0.90, 0.50, 0.28)),
        mass=13.0,
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
    )

    chimney_cover = model.part("chimney_cover")
    chimney_height = 0.72
    chimney_width = 0.34
    chimney_depth = 0.24
    chimney_wall = 0.010
    chimney_cover.visual(
        Box((chimney_width, chimney_wall, chimney_height)),
        origin=Origin(xyz=(0.0, 0.11, chimney_height * 0.5)),
        material=brushed_steel,
        name="chimney_front_wall",
    )
    chimney_cover.visual(
        Box((chimney_width, chimney_wall, chimney_height)),
        origin=Origin(xyz=(0.0, -0.11, chimney_height * 0.5)),
        material=brushed_steel,
        name="chimney_back_wall",
    )
    chimney_cover.visual(
        Box((chimney_wall, chimney_depth, chimney_height)),
        origin=Origin(xyz=(-0.165, 0.0, chimney_height * 0.5)),
        material=brushed_steel,
        name="chimney_left_wall",
    )
    chimney_cover.visual(
        Box((chimney_wall, chimney_depth, chimney_height)),
        origin=Origin(xyz=(0.165, 0.0, chimney_height * 0.5)),
        material=brushed_steel,
        name="chimney_right_wall",
    )
    chimney_cover.inertial = Inertial.from_geometry(
        Box((chimney_width, chimney_depth, chimney_height)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.0, chimney_height * 0.5)),
    )

    model.articulation(
        "canopy_to_chimney_cover",
        ArticulationType.FIXED,
        parent=canopy,
        child=chimney_cover,
        origin=Origin(xyz=(0.0, -0.01, 0.26)),
    )

    button_pitch = 0.058
    button_front_y = 0.250
    button_z = 0.025
    for index in range(5):
        button = model.part(f"button_{index + 1}")
        button.visual(
            Box((0.023, 0.006, 0.0075)),
            origin=Origin(xyz=(0.0, 0.003, 0.0)),
            material=charcoal,
            name="plunger",
        )
        button.visual(
            Box((0.018, 0.011, 0.006)),
            origin=Origin(xyz=(0.0, -0.0055, 0.0)),
            material=charcoal,
            name="stem",
        )
        button.visual(
            Box((0.032, 0.002, 0.010)),
            origin=Origin(xyz=(0.0, -0.004, 0.0)),
            material=dark_trim,
            name="rear_stop",
        )
        button.inertial = Inertial.from_geometry(
            Box((0.032, 0.017, 0.010)),
            mass=0.015,
            origin=Origin(xyz=(0.0, -0.0025, 0.0)),
        )

        model.articulation(
            f"canopy_to_button_{index + 1}",
            ArticulationType.PRISMATIC,
            parent=canopy,
            child=button,
            origin=Origin(xyz=((-2 + index) * button_pitch, button_front_y, button_z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=0.04,
                lower=0.0,
                upper=0.006,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    canopy = object_model.get_part("canopy")
    chimney_cover = object_model.get_part("chimney_cover")
    buttons = [object_model.get_part(f"button_{index}") for index in range(1, 6)]
    button_joints = [
        object_model.get_articulation(f"canopy_to_button_{index}") for index in range(1, 6)
    ]

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

    ctx.expect_contact(chimney_cover, canopy, name="chimney_cover_contacts_canopy")
    ctx.expect_gap(
        chimney_cover,
        canopy,
        axis="z",
        min_gap=0.0,
        max_gap=0.001,
        name="chimney_cover_seats_on_top_deck",
    )
    ctx.expect_overlap(
        chimney_cover,
        canopy,
        axes="xy",
        min_overlap=0.22,
        name="chimney_cover_stays_centered_over_canopy",
    )

    panel_aabb = ctx.part_element_world_aabb(canopy, elem="control_panel_front")
    rest_proud_ok = panel_aabb is not None
    rest_proud_details: list[str] = []
    if panel_aabb is None:
        rest_proud_details.append("control panel front visual AABB unavailable")
    else:
        panel_front_y = panel_aabb[1][1]
        for index, button in enumerate(buttons, start=1):
            ctx.expect_contact(button, canopy, name=f"button_{index}_captured_by_panel")

            joint = button_joints[index - 1]
            axis_ok = tuple(float(value) for value in joint.axis) == (0.0, -1.0, 0.0)
            ctx.check(
                f"button_{index}_axis_points_into_housing",
                axis_ok,
                details=f"expected (0.0, -1.0, 0.0), got {joint.axis}",
            )
            limits = joint.motion_limits
            travel_ok = (
                limits is not None
                and limits.lower == 0.0
                and limits.upper is not None
                and abs(limits.upper - 0.006) < 1e-9
            )
            ctx.check(
                f"button_{index}_travel_is_brief",
                travel_ok,
                details=f"expected 6 mm travel, got {None if limits is None else (limits.lower, limits.upper)}",
            )

            plunger_aabb = ctx.part_element_world_aabb(button, elem="plunger")
            if plunger_aabb is None:
                rest_proud_ok = False
                rest_proud_details.append(f"button_{index} plunger AABB unavailable")
                continue
            proud = plunger_aabb[1][1] - panel_front_y
            if not (0.0045 <= proud <= 0.0065):
                rest_proud_ok = False
                rest_proud_details.append(
                    f"button_{index} rest proud {proud:.4f} m outside expected range"
                )
    ctx.check(
        "buttons_protrude_proud_of_front_panel_at_rest",
        rest_proud_ok,
        details="; ".join(rest_proud_details),
    )

    row_positions = [ctx.part_world_position(button) for button in buttons]
    row_ok = all(position is not None for position in row_positions)
    row_details: list[str] = []
    if row_ok:
        xs = [position[0] for position in row_positions if position is not None]
        zs = [position[2] for position in row_positions if position is not None]
        spacings = [xs[index + 1] - xs[index] for index in range(len(xs) - 1)]
        if max(zs) - min(zs) > 0.0005:
            row_ok = False
            row_details.append("button origins do not share one horizontal row")
        if min(spacings) < 0.055 or max(spacings) > 0.061:
            row_ok = False
            row_details.append(f"button spacing out of range: {spacings}")
    else:
        row_details.append("one or more button world positions unavailable")
    ctx.check(
        "buttons_are_arranged_in_a_front_row",
        row_ok,
        details="; ".join(row_details),
    )

    with ctx.pose({joint: 0.006 for joint in button_joints}):
        ctx.fail_if_parts_overlap_in_current_pose(name="fully_pressed_buttons_clear_canopy")
        pressed_panel_aabb = ctx.part_element_world_aabb(canopy, elem="control_panel_front")
        pressed_ok = pressed_panel_aabb is not None
        pressed_details: list[str] = []
        if pressed_panel_aabb is None:
            pressed_details.append("control panel front visual AABB unavailable in pressed pose")
        else:
            panel_front_y = pressed_panel_aabb[1][1]
            for index, button in enumerate(buttons, start=1):
                plunger_aabb = ctx.part_element_world_aabb(button, elem="plunger")
                if plunger_aabb is None:
                    pressed_ok = False
                    pressed_details.append(f"button_{index} plunger AABB unavailable in pressed pose")
                    continue
                flush_offset = plunger_aabb[1][1] - panel_front_y
                if not (-0.0005 <= flush_offset <= 0.0005):
                    pressed_ok = False
                    pressed_details.append(
                        f"button_{index} pressed offset {flush_offset:.4f} m not near flush"
                    )
        ctx.check(
            "buttons_press_nearly_flush_with_front_panel",
            pressed_ok,
            details="; ".join(pressed_details),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
