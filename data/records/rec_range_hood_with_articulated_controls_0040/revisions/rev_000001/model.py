from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
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


def _circle_profile(
    *,
    center: tuple[float, float] = (0.0, 0.0),
    radius: float,
    segments: int = 24,
    circumscribed: bool = False,
) -> list[tuple[float, float]]:
    cx, cy = center
    actual_radius = radius / math.cos(math.pi / segments) if circumscribed else radius
    return [
        (
            cx + actual_radius * math.cos(2.0 * math.pi * index / segments),
            cy + actual_radius * math.sin(2.0 * math.pi * index / segments),
        )
        for index in range(segments)
    ]


def _make_side_panel_mesh(
    name: str,
    *,
    yz_profile: list[tuple[float, float]],
    x_min: float,
    x_max: float,
):
    return mesh_from_geometry(
        section_loft(
            [
                [(x_min, y, z) for y, z in yz_profile],
                [(x_max, y, z) for y, z in yz_profile],
            ]
        ),
        ASSETS.mesh_path(name),
    )


def _make_control_console_mesh(
    name: str,
    *,
    width: float,
    height: float,
    depth: float,
    button_positions: dict[str, tuple[float, float]],
    button_stem_radius: float,
    knob_position: tuple[float, float],
    knob_shaft_radius: float,
):
    outer = [(x, -y) for x, y in rounded_rect_profile(width, height, radius=0.014, corner_segments=8)]
    holes = [
        list(
            reversed(
                _circle_profile(
                    center=(x, -z),
                    radius=button_stem_radius,
                    segments=24,
                    circumscribed=True,
                )
            )
        )
        for x, z in button_positions.values()
    ]
    holes.append(
        list(
            reversed(
                _circle_profile(
                    center=(knob_position[0], -knob_position[1]),
                    radius=knob_shaft_radius,
                    segments=28,
                    circumscribed=True,
                )
            )
        )
    )
    geom = ExtrudeWithHolesGeometry(outer, holes, depth, cap=True, center=True, closed=True)
    geom.rotate_x(-math.pi / 2.0)
    return mesh_from_geometry(geom, ASSETS.mesh_path(name))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chimney_range_hood", assets=ASSETS)

    stainless = model.material("stainless", rgba=(0.82, 0.84, 0.86, 1.0))
    dark_stainless = model.material("dark_stainless", rgba=(0.62, 0.64, 0.67, 1.0))
    black_glass = model.material("black_glass", rgba=(0.12, 0.13, 0.14, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.19, 0.20, 1.0))
    satin_black = model.material("satin_black", rgba=(0.10, 0.10, 0.11, 1.0))
    filter_grey = model.material("filter_grey", rgba=(0.48, 0.50, 0.52, 1.0))
    lamp_glass = model.material("lamp_glass", rgba=(0.86, 0.88, 0.90, 0.55))

    hood_width = 0.90
    half_width = hood_width * 0.5
    body_side_thickness = 0.014
    front_outer_y = 0.25
    back_inner_y = -0.22
    front_panel_thickness = 0.014
    back_panel_depth = 0.030
    front_height = 0.10
    saddle_depth = 0.18
    saddle_width = 0.34
    saddle_center_y = -0.13
    saddle_top_z = 0.186
    top_panel_thickness = 0.014
    saddle_front_y = saddle_center_y + saddle_depth * 0.5
    top_run = front_outer_y - saddle_front_y
    top_rise = saddle_top_z - front_height
    top_angle = -math.atan2(top_rise, top_run)
    top_length = math.sqrt(top_run**2 + top_rise**2)

    console_width = 0.20
    console_height = 0.094
    console_depth = 0.024
    console_center = (0.0, front_outer_y + console_depth * 0.5, front_height * 0.5)
    button_stem_radius = 0.0045
    knob_shaft_radius = 0.006
    knob_position = (0.0, -0.006)
    button_positions = {
        "button_upper_left": (-0.022, 0.028),
        "button_upper_right": (0.022, 0.028),
        "button_left_upper": (-0.053, 0.010),
        "button_left_lower": (-0.053, -0.024),
        "button_right_upper": (0.053, 0.010),
        "button_right_lower": (0.053, -0.024),
    }

    canopy_side_profile = [
        (front_outer_y, 0.0),
        (front_outer_y, front_height),
        (saddle_front_y, saddle_top_z),
        (back_inner_y, saddle_top_z),
        (back_inner_y, 0.0),
    ]
    left_side_mesh = _make_side_panel_mesh(
        "range_hood_left_side.obj",
        yz_profile=canopy_side_profile,
        x_min=-half_width,
        x_max=-half_width + body_side_thickness,
    )
    right_side_mesh = _make_side_panel_mesh(
        "range_hood_right_side.obj",
        yz_profile=canopy_side_profile,
        x_min=half_width - body_side_thickness,
        x_max=half_width,
    )
    hood_body = model.part("hood_body")
    hood_body.visual(
        Box((hood_width, front_panel_thickness, front_height)),
        origin=Origin(
            xyz=(
                0.0,
                front_outer_y - front_panel_thickness * 0.5,
                front_height * 0.5,
            )
        ),
        material=stainless,
        name="front_fascia",
    )
    hood_body.visual(
        Box((hood_width, back_panel_depth, saddle_top_z)),
        origin=Origin(
            xyz=(
                0.0,
                back_inner_y - back_panel_depth * 0.5,
                saddle_top_z * 0.5,
            )
        ),
        material=stainless,
        name="rear_mount_panel",
    )
    hood_body.visual(
        Box((hood_width, top_length, top_panel_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                (front_outer_y + saddle_front_y) * 0.5 - 0.002,
                (front_height + saddle_top_z) * 0.5,
            ),
            rpy=(top_angle, 0.0, 0.0),
        ),
        material=stainless,
        name="sloped_top",
    )
    hood_body.visual(left_side_mesh, material=stainless, name="left_side_shell")
    hood_body.visual(right_side_mesh, material=stainless, name="right_side_shell")
    hood_body.visual(
        Box((saddle_width, saddle_depth, 0.012)),
        origin=Origin(xyz=(0.0, saddle_center_y, saddle_top_z - 0.006)),
        material=stainless,
        name="chimney_saddle",
    )
    hood_body.visual(
        Box((0.876, 0.030, 0.016)),
        origin=Origin(xyz=(0.0, 0.19, 0.008)),
        material=dark_stainless,
        name="front_filter_support",
    )
    hood_body.visual(
        Box((0.876, 0.030, 0.016)),
        origin=Origin(xyz=(0.0, -0.13, 0.008)),
        material=dark_stainless,
        name="rear_filter_support",
    )
    hood_body.visual(
        Box((0.024, 0.350, 0.014)),
        origin=Origin(xyz=(0.0, 0.03, 0.007)),
        material=dark_stainless,
        name="center_filter_divider",
    )
    hood_body.visual(
        Box((0.405, 0.300, 0.006)),
        origin=Origin(xyz=(-0.213, 0.03, 0.003)),
        material=filter_grey,
        name="left_filter_screen",
    )
    hood_body.visual(
        Box((0.405, 0.300, 0.006)),
        origin=Origin(xyz=(0.213, 0.03, 0.003)),
        material=filter_grey,
        name="right_filter_screen",
    )
    hood_body.visual(
        Box((0.090, 0.020, 0.008)),
        origin=Origin(xyz=(-0.16, 0.226, 0.006)),
        material=lamp_glass,
        name="left_task_light",
    )
    hood_body.visual(
        Box((0.090, 0.020, 0.008)),
        origin=Origin(xyz=(0.16, 0.226, 0.006)),
        material=lamp_glass,
        name="right_task_light",
    )
    hood_body.inertial = Inertial.from_geometry(
        Box((hood_width, 0.50, saddle_top_z)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, saddle_top_z * 0.5)),
    )

    chimney_cover = model.part("chimney_cover")
    chimney_width = 0.32
    chimney_depth = saddle_depth
    chimney_panel_thickness = 0.012
    chimney_height = 0.834
    chimney_cover.visual(
        Box((chimney_width, chimney_panel_thickness, chimney_height)),
        origin=Origin(
            xyz=(
                0.0,
                chimney_depth * 0.5 - chimney_panel_thickness * 0.5,
                chimney_height * 0.5,
            )
        ),
        material=stainless,
        name="front_panel",
    )
    chimney_cover.visual(
        Box((chimney_width, chimney_panel_thickness, chimney_height)),
        origin=Origin(
            xyz=(
                0.0,
                -chimney_depth * 0.5 + chimney_panel_thickness * 0.5,
                chimney_height * 0.5,
            )
        ),
        material=stainless,
        name="back_panel",
    )
    chimney_cover.visual(
        Box((chimney_panel_thickness, chimney_depth, chimney_height)),
        origin=Origin(
            xyz=(
                -chimney_width * 0.5 + chimney_panel_thickness * 0.5,
                0.0,
                chimney_height * 0.5,
            )
        ),
        material=stainless,
        name="left_panel",
    )
    chimney_cover.visual(
        Box((chimney_panel_thickness, chimney_depth, chimney_height)),
        origin=Origin(
            xyz=(
                chimney_width * 0.5 - chimney_panel_thickness * 0.5,
                0.0,
                chimney_height * 0.5,
            )
        ),
        material=stainless,
        name="right_panel",
    )
    chimney_cover.visual(
        Box((chimney_width - 0.040, 0.010, 0.060)),
        origin=Origin(xyz=(0.0, chimney_depth * 0.5 - 0.005, chimney_height - 0.120)),
        material=dark_stainless,
        name="upper_seam_band",
    )
    chimney_cover.inertial = Inertial.from_geometry(
        Box((chimney_width, chimney_depth, chimney_height)),
        mass=4.5,
        origin=Origin(xyz=(0.0, 0.0, chimney_height * 0.5)),
    )
    model.articulation(
        "hood_to_chimney_cover",
        ArticulationType.FIXED,
        parent=hood_body,
        child=chimney_cover,
        origin=Origin(xyz=(0.0, saddle_center_y, saddle_top_z)),
    )

    button_body_size = (0.013, 0.010, 0.013)
    button_cap_size = (0.015, 0.002, 0.015)
    button_guide_wall = 0.0015
    button_guide_depth = 0.012
    knob_body_radius = 0.018
    knob_body_length = 0.014
    knob_bezel_radius = 0.020

    def _add_button_guide(console_part, guide_name: str, x_pos: float, z_pos: float) -> None:
        half_x = button_body_size[0] * 0.5
        half_z = button_body_size[2] * 0.5
        wall_center_y = 0.006
        console_part.visual(
            Box((button_guide_wall, button_guide_depth, button_body_size[2] + 2.0 * button_guide_wall)),
            origin=Origin(
                xyz=(x_pos - half_x - button_guide_wall * 0.5, wall_center_y, z_pos)
            ),
            material=charcoal,
            name=f"{guide_name}_left_wall",
        )
        console_part.visual(
            Box((button_guide_wall, button_guide_depth, button_body_size[2] + 2.0 * button_guide_wall)),
            origin=Origin(
                xyz=(x_pos + half_x + button_guide_wall * 0.5, wall_center_y, z_pos)
            ),
            material=charcoal,
            name=f"{guide_name}_right_wall",
        )
        console_part.visual(
            Box((button_body_size[0], button_guide_depth, button_guide_wall)),
            origin=Origin(
                xyz=(x_pos, wall_center_y, z_pos + half_z + button_guide_wall * 0.5)
            ),
            material=charcoal,
            name=f"{guide_name}_top_wall",
        )
        console_part.visual(
            Box((button_body_size[0], button_guide_depth, button_guide_wall)),
            origin=Origin(
                xyz=(x_pos, wall_center_y, z_pos - half_z - button_guide_wall * 0.5)
            ),
            material=charcoal,
            name=f"{guide_name}_bottom_wall",
        )

    control_console = model.part("control_console")
    control_console.visual(
        Box((console_width, 0.004, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=black_glass,
        name="console_top_rail",
    )
    control_console.visual(
        Box((console_width, 0.004, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, -0.036)),
        material=black_glass,
        name="console_bottom_rail",
    )
    control_console.visual(
        Box((0.041, 0.004, console_height)),
        origin=Origin(xyz=(-0.0795, 0.0, 0.0)),
        material=black_glass,
        name="console_left_rail",
    )
    control_console.visual(
        Box((0.041, 0.004, console_height)),
        origin=Origin(xyz=(0.0795, 0.0, 0.0)),
        material=black_glass,
        name="console_right_rail",
    )
    control_console.visual(
        Box((0.020, 0.004, 0.030)),
        origin=Origin(xyz=(-0.036, 0.0, 0.0)),
        material=black_glass,
        name="console_knob_left_bridge",
    )
    control_console.visual(
        Box((0.020, 0.004, 0.030)),
        origin=Origin(xyz=(0.036, 0.0, 0.0)),
        material=black_glass,
        name="console_knob_right_bridge",
    )
    for button_name, (x_pos, z_pos) in button_positions.items():
        _add_button_guide(control_console, button_name, x_pos, z_pos)
    control_console.visual(
        Box((0.002, 0.016, 0.044)),
        origin=Origin(xyz=(-0.019, 0.010, -0.006)),
        material=charcoal,
        name="knob_left_wall",
    )
    control_console.visual(
        Box((0.002, 0.016, 0.044)),
        origin=Origin(xyz=(0.019, 0.010, -0.006)),
        material=charcoal,
        name="knob_right_wall",
    )
    control_console.visual(
        Box((0.038, 0.016, 0.002)),
        origin=Origin(xyz=(0.0, 0.010, 0.013)),
        material=charcoal,
        name="knob_top_wall",
    )
    control_console.visual(
        Box((0.038, 0.016, 0.002)),
        origin=Origin(xyz=(0.0, 0.010, -0.025)),
        material=charcoal,
        name="knob_bottom_wall",
    )
    control_console.inertial = Inertial.from_geometry(
        Box((console_width, console_depth, console_height)),
        mass=0.45,
        origin=Origin(),
    )
    model.articulation(
        "hood_to_control_console",
        ArticulationType.FIXED,
        parent=hood_body,
        child=control_console,
        origin=Origin(xyz=(console_center[0], front_outer_y + 0.002, console_center[2])),
    )

    knob = model.part("rotary_knob")
    knob.visual(
        Cylinder(radius=knob_bezel_radius, length=0.002),
        origin=Origin(xyz=(0.0, 0.003, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_stainless,
        name="knob_bezel",
    )
    knob.visual(
        Cylinder(radius=knob_body_radius, length=knob_body_length),
        origin=Origin(xyz=(0.0, 0.011, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="knob_body",
    )
    knob.visual(
        Box((0.003, 0.002, 0.012)),
        origin=Origin(xyz=(0.0, 0.018, 0.010)),
        material=stainless,
        name="knob_indicator",
    )
    knob.inertial = Inertial.from_geometry(
        Box((0.040, 0.052, 0.040)),
        mass=0.06,
        origin=Origin(xyz=(0.0, 0.005, 0.0)),
    )
    model.articulation(
        "console_to_rotary_knob",
        ArticulationType.CONTINUOUS,
        parent=control_console,
        child=knob,
        origin=Origin(xyz=(knob_position[0], 0.002, knob_position[1])),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=6.0),
    )

    for button_name, (x_pos, z_pos) in button_positions.items():
        button = model.part(button_name)
        button.visual(
            Box(button_body_size),
            origin=Origin(xyz=(0.0, 0.007, 0.0)),
            material=dark_stainless,
            name="button_body",
        )
        button.visual(
            Box(button_cap_size),
            origin=Origin(xyz=(0.0, 0.013, 0.0)),
            material=stainless,
            name="button_cap",
        )
        button.inertial = Inertial.from_geometry(
            Box((button_cap_size[0], 0.014, button_cap_size[2])),
            mass=0.015,
            origin=Origin(xyz=(0.0, 0.008, 0.0)),
        )
        model.articulation(
            f"console_to_{button_name}",
            ArticulationType.PRISMATIC,
            parent=control_console,
            child=button,
            origin=Origin(xyz=(x_pos, 0.002, z_pos)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=0.05,
                lower=0.0,
                upper=0.004,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    hood_body = object_model.get_part("hood_body")
    chimney_cover = object_model.get_part("chimney_cover")
    control_console = object_model.get_part("control_console")
    rotary_knob = object_model.get_part("rotary_knob")
    knob_joint = object_model.get_articulation("console_to_rotary_knob")
    button_names = (
        "button_upper_left",
        "button_upper_right",
        "button_left_upper",
        "button_left_lower",
        "button_right_upper",
        "button_right_lower",
    )
    button_parts = {name: object_model.get_part(name) for name in button_names}
    button_joints = {
        name: object_model.get_articulation(f"console_to_{name}") for name in button_names
    }

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

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

    for part in (hood_body, chimney_cover, control_console, rotary_knob, *button_parts.values()):
        ctx.check(
            f"{part.name}_present",
            len(part.visuals) > 0,
            f"{part.name} should expose visual geometry",
        )

    ctx.expect_contact(control_console, hood_body, name="control_console_attached_to_hood")
    ctx.expect_gap(
        control_console,
        hood_body,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        name="control_console_flush_on_front_fascia",
    )
    ctx.expect_gap(
        chimney_cover,
        hood_body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        negative_elem="chimney_saddle",
        name="chimney_cover_seated_on_saddle",
    )
    ctx.expect_contact(
        chimney_cover,
        hood_body,
        elem_b="chimney_saddle",
        name="chimney_cover_contacts_saddle",
    )
    ctx.expect_overlap(
        chimney_cover,
        hood_body,
        axes="xy",
        min_overlap=0.18,
        name="chimney_cover_overlaps_canopy_footprint",
    )
    ctx.expect_contact(
        rotary_knob,
        control_console,
        contact_tol=0.0006,
        name="rotary_knob_guided_by_console",
    )
    for name, part in button_parts.items():
        ctx.expect_contact(
            part,
            control_console,
            contact_tol=0.0006,
            name=f"{name}_guided_by_console",
        )

    hood_aabb = ctx.part_world_aabb(hood_body)
    chimney_aabb = ctx.part_world_aabb(chimney_cover)
    console_aabb = ctx.part_world_aabb(control_console)
    if hood_aabb is not None:
        hood_width = hood_aabb[1][0] - hood_aabb[0][0]
        hood_depth = hood_aabb[1][1] - hood_aabb[0][1]
        ctx.check(
            "hood_body_realistic_width",
            0.85 <= hood_width <= 0.95,
            f"hood width should read like a real 90 cm range hood, got {hood_width:.3f} m",
        )
        ctx.check(
            "hood_body_realistic_depth",
            0.46 <= hood_depth <= 0.52,
            f"hood depth should read like a real canopy hood, got {hood_depth:.3f} m",
        )
    if chimney_aabb is not None:
        chimney_height = chimney_aabb[1][2] - chimney_aabb[0][2]
        ctx.check(
            "chimney_cover_tall_enough",
            0.80 <= chimney_height <= 0.88,
            f"chimney cover should be tall, got {chimney_height:.3f} m",
        )
        if hood_aabb is not None:
            ctx.check(
                "chimney_cover_extends_well_above_canopy",
                chimney_aabb[1][2] - hood_aabb[1][2] >= 0.78,
                "chimney should rise prominently above the canopy",
            )
    if console_aabb is not None:
        console_height = console_aabb[1][2] - console_aabb[0][2]
        ctx.check(
            "control_console_slender",
            0.085 <= console_height <= 0.100,
            f"console should stay slim, got {console_height:.3f} m tall",
        )

    knob_axis = knob_joint.axis
    ctx.check(
        "rotary_knob_axis_along_y",
        abs(knob_axis[0]) < 1e-9 and abs(abs(knob_axis[1]) - 1.0) < 1e-9 and abs(knob_axis[2]) < 1e-9,
        f"knob axis should align with the front-to-back shaft, got {knob_axis}",
    )
    knob_limits = knob_joint.motion_limits
    ctx.check(
        "rotary_knob_continuous_limits",
        knob_limits is not None and knob_limits.lower is None and knob_limits.upper is None,
        "continuous knob should not have lower/upper bounds",
    )

    knob_pos = ctx.part_world_position(rotary_knob)
    button_positions_world = {name: ctx.part_world_position(part) for name, part in button_parts.items()}
    if knob_pos is not None and all(pos is not None for pos in button_positions_world.values()):
        ctx.check(
            "top_buttons_above_knob",
            button_positions_world["button_upper_left"][2] > knob_pos[2]
            and button_positions_world["button_upper_right"][2] > knob_pos[2],
            "upper button pair should sit above the central knob",
        )
        ctx.check(
            "left_buttons_left_of_knob",
            button_positions_world["button_left_upper"][0] < knob_pos[0]
            and button_positions_world["button_left_lower"][0] < knob_pos[0],
            "left button pair should stay left of the knob",
        )
        ctx.check(
            "right_buttons_right_of_knob",
            button_positions_world["button_right_upper"][0] > knob_pos[0]
            and button_positions_world["button_right_lower"][0] > knob_pos[0],
            "right button pair should stay right of the knob",
        )
        ctx.check(
            "side_button_columns_ordered_top_to_bottom",
            button_positions_world["button_left_upper"][2] > button_positions_world["button_left_lower"][2]
            and button_positions_world["button_right_upper"][2] > button_positions_world["button_right_lower"][2],
            "each side should read as a vertical pair around the knob",
        )

    rest_button_positions = {name: ctx.part_world_position(part) for name, part in button_parts.items()}
    pressed_pose = {
        joint: joint.motion_limits.upper
        for joint in button_joints.values()
        if joint.motion_limits is not None and joint.motion_limits.upper is not None
    }
    with ctx.pose(pressed_pose):
        ctx.fail_if_parts_overlap_in_current_pose(name="buttons_pressed_no_overlap")
        ctx.fail_if_isolated_parts(name="buttons_pressed_no_floating")
        for name, part in button_parts.items():
            joint = button_joints[name]
            axis = joint.axis
            limits = joint.motion_limits
            ctx.check(
                f"{name}_axis_inward",
                abs(axis[0]) < 1e-9 and abs(axis[1] + 1.0) < 1e-9 and abs(axis[2]) < 1e-9,
                f"{name} should plunge inward along -Y, got {axis}",
            )
            ctx.check(
                f"{name}_short_prismatic_travel",
                limits is not None
                and limits.lower == 0.0
                and limits.upper is not None
                and 0.003 <= limits.upper <= 0.006,
                f"{name} should have a short plunger stroke, got {limits}",
            )
            pressed_pos = ctx.part_world_position(part)
            rest_pos = rest_button_positions[name]
            ctx.check(
                f"{name}_moves_inward_when_pressed",
                rest_pos is not None
                and pressed_pos is not None
                and pressed_pos[1] < rest_pos[1] - 0.003,
                f"{name} should move inward when pressed: rest={rest_pos}, pressed={pressed_pos}",
            )
            ctx.expect_contact(
                part,
                control_console,
                contact_tol=0.0006,
                name=f"{name}_pressed_stays_guided",
            )

    with ctx.pose({knob_joint: 1.8}):
        ctx.fail_if_parts_overlap_in_current_pose(name="knob_rotated_no_overlap")
        ctx.fail_if_isolated_parts(name="knob_rotated_no_floating")
        ctx.expect_contact(
            rotary_knob,
            control_console,
            contact_tol=0.0006,
            name="knob_rotated_stays_guided",
        )

    combined_pose = dict(pressed_pose)
    combined_pose[knob_joint] = -2.4
    with ctx.pose(combined_pose):
        ctx.fail_if_parts_overlap_in_current_pose(name="combined_controls_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="combined_controls_pose_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
