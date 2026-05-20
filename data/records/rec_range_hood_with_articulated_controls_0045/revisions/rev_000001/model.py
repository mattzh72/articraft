from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_cabinet_range_hood", assets=ASSETS)

    stainless = model.material("stainless", rgba=(0.78, 0.80, 0.82, 1.0))
    band_finish = model.material("band_finish", rgba=(0.72, 0.74, 0.76, 1.0))
    button_finish = model.material("button_finish", rgba=(0.13, 0.13, 0.14, 1.0))

    hood_width = 0.76
    hood_depth = 0.50
    hood_height = 0.16
    shell_thickness = 0.012
    band_thickness = 0.018
    band_height = 0.045
    band_proud = 0.003
    lower_front_height = hood_height - shell_thickness - band_height
    shell_wall_height = hood_height - shell_thickness
    band_center_z = lower_front_height + band_height / 2.0
    band_center_y = hood_depth / 2.0 - band_thickness / 2.0 + band_proud

    button_radius = 0.009
    button_travel = 0.0035
    button_center_z = band_center_z
    button_row_center_x = 0.20
    button_spacing = 0.036
    button_cap_length = 0.004
    button_slider_width = 0.014
    button_slider_height = 0.006
    button_slider_depth = 0.018
    button_rest_center_y = band_center_y - 0.002
    button_x_positions = [
        button_row_center_x + button_spacing * (index - 1.5) for index in range(4)
    ]

    button_opening_width = 0.024
    slot_height = 0.021
    side_guide_width = (button_opening_width - button_slider_width) / 2.0
    band_strip_height = (band_height - slot_height) / 2.0
    top_strip_center_z = band_center_z + (band_height - band_strip_height) / 2.0
    bottom_strip_center_z = band_center_z - (band_height - band_strip_height) / 2.0
    opening_edges = [
        (button_x - button_opening_width / 2.0, button_x + button_opening_width / 2.0)
        for button_x in button_x_positions
    ]
    slot_segments = []
    left_edge = -hood_width / 2.0
    for opening_left, opening_right in opening_edges:
        if opening_left > left_edge:
            slot_segments.append((left_edge, opening_left))
        left_edge = opening_right
    if left_edge < hood_width / 2.0:
        slot_segments.append((left_edge, hood_width / 2.0))

    hood_body = model.part("hood_body")
    hood_body.visual(
        Box((hood_width, hood_depth, shell_thickness)),
        origin=Origin(xyz=(0.0, 0.0, hood_height - shell_thickness / 2.0)),
        material=stainless,
        name="top_panel",
    )
    hood_body.visual(
        Box((hood_width, shell_thickness, shell_wall_height)),
        origin=Origin(
            xyz=(
                0.0,
                -hood_depth / 2.0 + shell_thickness / 2.0,
                shell_wall_height / 2.0,
            )
        ),
        material=stainless,
        name="back_panel",
    )
    hood_body.visual(
        Box((shell_thickness, hood_depth, shell_wall_height)),
        origin=Origin(
            xyz=(
                -hood_width / 2.0 + shell_thickness / 2.0,
                0.0,
                shell_wall_height / 2.0,
            )
        ),
        material=stainless,
        name="left_panel",
    )
    hood_body.visual(
        Box((shell_thickness, hood_depth, shell_wall_height)),
        origin=Origin(
            xyz=(
                hood_width / 2.0 - shell_thickness / 2.0,
                0.0,
                shell_wall_height / 2.0,
            )
        ),
        material=stainless,
        name="right_panel",
    )
    hood_body.visual(
        Box((hood_width, shell_thickness, lower_front_height)),
        origin=Origin(
            xyz=(
                0.0,
                hood_depth / 2.0 - shell_thickness / 2.0,
                lower_front_height / 2.0,
            )
        ),
        material=stainless,
        name="front_lower_panel",
    )
    hood_body.visual(
        Box((hood_width, band_thickness, band_strip_height)),
        origin=Origin(
            xyz=(0.0, band_center_y, top_strip_center_z),
        ),
        material=band_finish,
        name="front_band",
    )
    hood_body.visual(
        Box((hood_width, band_thickness, band_strip_height)),
        origin=Origin(
            xyz=(0.0, band_center_y, bottom_strip_center_z),
        ),
        material=band_finish,
        name="front_band_lower",
    )
    for segment_index, (segment_left, segment_right) in enumerate(slot_segments, start=1):
        segment_width = segment_right - segment_left
        if segment_width <= 0.0:
            continue
        hood_body.visual(
            Box((segment_width, band_thickness, slot_height)),
            origin=Origin(
                xyz=(
                    (segment_left + segment_right) / 2.0,
                    band_center_y,
                    band_center_z,
                )
            ),
            material=band_finish,
            name=f"band_slot_segment_{segment_index}",
        )
    for index, button_x in enumerate(button_x_positions, start=1):
        hood_body.visual(
            Box((side_guide_width, band_thickness, slot_height)),
            origin=Origin(
                xyz=(
                    button_x - button_opening_width / 2.0 + side_guide_width / 2.0,
                    band_center_y,
                    button_center_z,
                )
            ),
            material=stainless,
            name=f"button_{index}_guide_left",
        )
        hood_body.visual(
            Box((side_guide_width, band_thickness, slot_height)),
            origin=Origin(
                xyz=(
                    button_x + button_opening_width / 2.0 - side_guide_width / 2.0,
                    band_center_y,
                    button_center_z,
                )
            ),
            material=stainless,
            name=f"button_{index}_guide_right",
        )

    for index, button_x in enumerate(button_x_positions, start=1):
        button = model.part(f"button_{index}")
        button.visual(
            Cylinder(radius=button_radius, length=button_cap_length),
            origin=Origin(
                xyz=(0.0, button_slider_depth / 2.0 + button_cap_length / 2.0, 0.0),
                rpy=(-pi / 2.0, 0.0, 0.0),
            ),
            material=button_finish,
            name="cap",
        )
        button.visual(
            Box((button_slider_width, button_slider_depth, button_slider_height)),
            origin=Origin(),
            material=button_finish,
            name="slider",
        )
        model.articulation(
            f"button_{index}_plunger",
            ArticulationType.PRISMATIC,
            parent=hood_body,
            child=button,
            origin=Origin(
                xyz=(
                    button_x,
                    button_rest_center_y,
                    button_center_z,
                )
            ),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=10.0,
                velocity=0.05,
                lower=0.0,
                upper=button_travel,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    hood_body = object_model.get_part("hood_body")
    front_band = hood_body.get_visual("front_band")
    buttons = [object_model.get_part(f"button_{index}") for index in range(1, 5)]
    plungers = [
        object_model.get_articulation(f"button_{index}_plunger")
        for index in range(1, 5)
    ]
    sliders = [button.get_visual("slider") for button in buttons]
    left_guides = [
        hood_body.get_visual(f"button_{index}_guide_left") for index in range(1, 5)
    ]
    right_guides = [
        hood_body.get_visual(f"button_{index}_guide_right") for index in range(1, 5)
    ]

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

    hood_aabb = ctx.part_world_aabb(hood_body)
    band_aabb = ctx.part_element_world_aabb(hood_body, elem=front_band)
    ctx.check("hood_body_present", hood_aabb is not None, "hood body AABB missing")
    ctx.check("front_band_present", band_aabb is not None, "front band AABB missing")

    if hood_aabb is not None:
        hood_size = tuple(
            hood_aabb[1][axis] - hood_aabb[0][axis] for axis in range(3)
        )
        ctx.check(
            "hood_realistic_width",
            0.68 <= hood_size[0] <= 0.92,
            f"expected realistic 30 in class width, got {hood_size[0]:.3f} m",
        )
        ctx.check(
            "hood_realistic_depth",
            0.42 <= hood_size[1] <= 0.56,
            f"expected realistic under-cabinet depth, got {hood_size[1]:.3f} m",
        )
        ctx.check(
            "hood_realistic_height",
            0.12 <= hood_size[2] <= 0.22,
            f"expected realistic low-profile hood height, got {hood_size[2]:.3f} m",
        )

    if hood_aabb is not None and band_aabb is not None:
        band_width = band_aabb[1][0] - band_aabb[0][0]
        band_top_gap = hood_aabb[1][2] - band_aabb[1][2]
        band_bottom = band_aabb[0][2]
        ctx.check(
            "front_band_spans_body_width",
            band_width >= 0.95 * (hood_aabb[1][0] - hood_aabb[0][0]),
            "front band should read as a full-width flat control band",
        )
        ctx.check(
            "front_band_near_top_front",
            band_top_gap <= 0.02 and band_bottom >= hood_aabb[0][2] + 0.09,
            "front band should sit in the upper front zone of the hood body",
        )

    button_positions = [ctx.part_world_position(button) for button in buttons]
    ctx.check(
        "button_positions_resolved",
        all(position is not None for position in button_positions),
        "all button world positions should resolve",
    )
    if all(position is not None for position in button_positions):
        xs = [position[0] for position in button_positions]
        ys = [position[1] for position in button_positions]
        zs = [position[2] for position in button_positions]
        spacings = [xs[index + 1] - xs[index] for index in range(3)]
        row_mid_x = sum(xs) / len(xs)
        ctx.check(
            "button_row_strictly_horizontal",
            max(abs(z - zs[0]) for z in zs) <= 0.001,
            "buttons should sit in one horizontal row",
        )
        ctx.check(
            "button_row_even_spacing",
            min(spacings) >= 0.03 and max(spacings) <= 0.042 and max(spacings) - min(spacings) <= 0.002,
            f"expected evenly spaced buttons, got spacings {spacings!r}",
        )
        ctx.check(
            "button_row_on_right_half",
            row_mid_x >= 0.12 and min(xs) > 0.0,
            f"expected the button row on the right half, got midpoint x={row_mid_x:.3f}",
        )
        if band_aabb is not None:
            ctx.check(
                "button_row_on_front_face",
                min(ys) >= band_aabb[0][1] - 0.001 and max(ys) <= band_aabb[1][1] + 0.01,
                "buttons should lie on the front control band depth range",
            )

    ctx.check(
        "only_four_button_articulations",
        len(object_model.articulations) == 4,
        f"expected exactly four articulations, got {len(object_model.articulations)}",
    )

    for button, plunger, slider, left_guide, right_guide in zip(
        buttons, plungers, sliders, left_guides, right_guides
    ):
        limits = plunger.motion_limits
        ctx.check(
            f"{plunger.name}_is_prismatic",
            plunger.joint_type == ArticulationType.PRISMATIC,
            f"{plunger.name} should be prismatic",
        )
        ctx.check(
            f"{plunger.name}_axis_normal_to_panel",
            tuple(round(value, 6) for value in plunger.axis) == (0.0, -1.0, 0.0),
            f"{plunger.name} axis should push into the front panel, got {plunger.axis!r}",
        )
        ctx.check(
            f"{plunger.name}_travel_short",
            limits is not None
            and limits.lower == 0.0
            and limits.upper is not None
            and 0.002 <= limits.upper <= 0.006,
            f"{plunger.name} should have short button travel, got {limits!r}",
        )
        ctx.expect_within(
            button,
            hood_body,
            axes="xz",
            margin=0.001,
            name=f"{button.name}_within_hood_face_zone",
        )
        ctx.expect_contact(
            button,
            hood_body,
            elem_a=slider,
            elem_b=left_guide,
            name=f"{button.name}_slider_touches_left_guide",
        )
        ctx.expect_contact(
            button,
            hood_body,
            elem_a=slider,
            elem_b=right_guide,
            name=f"{button.name}_slider_touches_right_guide",
        )

        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({plunger: limits.lower}):
                rest_position = ctx.part_world_position(button)
                ctx.fail_if_parts_overlap_in_current_pose(
                    name=f"{plunger.name}_lower_no_overlap"
                )
                ctx.fail_if_isolated_parts(name=f"{plunger.name}_lower_no_floating")
                ctx.expect_contact(
                    button,
                    hood_body,
                    elem_a=slider,
                    elem_b=left_guide,
                    name=f"{button.name}_lower_left_guide_contact",
                )
                ctx.expect_contact(
                    button,
                    hood_body,
                    elem_a=slider,
                    elem_b=right_guide,
                    name=f"{button.name}_lower_right_guide_contact",
                )
            with ctx.pose({plunger: limits.upper}):
                pressed_position = ctx.part_world_position(button)
                ctx.fail_if_parts_overlap_in_current_pose(
                    name=f"{plunger.name}_upper_no_overlap"
                )
                ctx.fail_if_isolated_parts(name=f"{plunger.name}_upper_no_floating")
                ctx.expect_contact(
                    button,
                    hood_body,
                    elem_a=slider,
                    elem_b=left_guide,
                    name=f"{button.name}_upper_left_guide_contact",
                )
                ctx.expect_contact(
                    button,
                    hood_body,
                    elem_a=slider,
                    elem_b=right_guide,
                    name=f"{button.name}_upper_right_guide_contact",
                )
            if rest_position is not None and pressed_position is not None:
                ctx.check(
                    f"{plunger.name}_moves_inward",
                    pressed_position[1] < rest_position[1] - 0.002,
                    f"{plunger.name} should translate inward along -Y",
                )
                ctx.check(
                    f"{plunger.name}_travel_amount_matches_limit",
                    abs((rest_position[1] - pressed_position[1]) - limits.upper) <= 0.001,
                    f"{plunger.name} travel should closely match its joint limit",
                )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
