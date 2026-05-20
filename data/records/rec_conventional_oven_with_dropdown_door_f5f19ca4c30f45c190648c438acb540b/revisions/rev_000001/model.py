from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="double_deck_oven")

    steel = model.material("steel", color=(0.73, 0.75, 0.77))
    dark_steel = model.material("dark_steel", color=(0.22, 0.23, 0.25))
    graphite = model.material("graphite", color=(0.12, 0.13, 0.14))
    stone = model.material("stone", color=(0.67, 0.61, 0.50))
    knob_black = model.material("knob_black", color=(0.10, 0.10, 0.11))
    badge_red = model.material("badge_red", color=(0.68, 0.10, 0.10))

    width = 1.30
    depth = 1.00
    height = 1.52
    side_wall_w = 0.09
    plinth_h = 0.14
    front_frame_t = 0.04
    roof_t = 0.06
    chamber_open_w = width - 2.0 * side_wall_w
    chamber_open_h = 0.30
    bottom_open_z = 0.24
    interdeck_h = 0.12
    top_open_z = bottom_open_z + chamber_open_h + interdeck_h
    top_open_top = top_open_z + chamber_open_h
    hood_h = height - top_open_top
    chamber_depth = 0.92
    chamber_floor_t = 0.04
    rail_w = 0.028
    rail_t = 0.016
    rail_len = 0.78
    rail_y = 0.08 + rail_len / 2.0
    runner_t = 0.012
    deck_w = 1.09
    deck_d = 0.72
    deck_t = 0.028
    deck_front_y = 0.055
    deck_center_y = deck_front_y + deck_d / 2.0
    rack_travel = 0.28
    hinge_limits = MotionLimits(effort=55.0, velocity=1.5, lower=0.0, upper=1.45)
    slide_limits = MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=rack_travel)

    body = model.part("body")
    body.visual(
        Box((width, depth - 0.10, plinth_h)),
        origin=Origin(xyz=(0.0, 0.55, plinth_h / 2.0)),
        material=dark_steel,
        name="plinth",
    )
    body.visual(
        Box((side_wall_w, depth, height - plinth_h)),
        origin=Origin(xyz=(-(width / 2.0) + (side_wall_w / 2.0), depth / 2.0, plinth_h + (height - plinth_h) / 2.0)),
        material=steel,
        name="left_wall",
    )
    body.visual(
        Box((side_wall_w, depth, height - plinth_h)),
        origin=Origin(xyz=((width / 2.0) - (side_wall_w / 2.0), depth / 2.0, plinth_h + (height - plinth_h) / 2.0)),
        material=steel,
        name="right_wall",
    )
    body.visual(
        Box((width, 0.04, height - plinth_h)),
        origin=Origin(xyz=(0.0, depth - 0.02, plinth_h + (height - plinth_h) / 2.0)),
        material=steel,
        name="rear_panel",
    )
    body.visual(
        Box((width, depth, roof_t)),
        origin=Origin(xyz=(0.0, depth / 2.0, height - roof_t / 2.0)),
        material=steel,
        name="top_cap",
    )
    body.visual(
        Box((chamber_open_w, front_frame_t, bottom_open_z - plinth_h)),
        origin=Origin(xyz=(0.0, front_frame_t / 2.0, plinth_h + (bottom_open_z - plinth_h) / 2.0)),
        material=steel,
        name="bottom_front_sill",
    )
    body.visual(
        Box((chamber_open_w, front_frame_t, interdeck_h)),
        origin=Origin(xyz=(0.0, front_frame_t / 2.0, bottom_open_z + chamber_open_h + interdeck_h / 2.0)),
        material=steel,
        name="mid_front_band",
    )
    body.visual(
        Box((width, front_frame_t, hood_h)),
        origin=Origin(xyz=(0.0, front_frame_t / 2.0, top_open_top + hood_h / 2.0)),
        material=steel,
        name="hood_front",
    )
    body.visual(
        Box((chamber_open_w, chamber_depth, chamber_floor_t)),
        origin=Origin(xyz=(0.0, front_frame_t + chamber_depth / 2.0, bottom_open_z - chamber_floor_t / 2.0)),
        material=dark_steel,
        name="bottom_chamber_floor",
    )
    body.visual(
        Box((chamber_open_w, chamber_depth, interdeck_h)),
        origin=Origin(xyz=(0.0, front_frame_t + chamber_depth / 2.0, bottom_open_z + chamber_open_h + interdeck_h / 2.0)),
        material=dark_steel,
        name="interdeck_block",
    )
    body.visual(
        Box((chamber_open_w, chamber_depth, roof_t)),
        origin=Origin(xyz=(0.0, front_frame_t + chamber_depth / 2.0, top_open_top + roof_t / 2.0)),
        material=dark_steel,
        name="top_chamber_roof",
    )
    body.visual(
        Box((0.40, 0.05, 0.05)),
        origin=Origin(xyz=(0.0, depth - 0.045, height + 0.025)),
        material=dark_steel,
        name="flue_cap",
    )
    body.visual(
        Box((0.18, 0.004, 0.05)),
        origin=Origin(xyz=(0.0, -0.002, height - 0.17)),
        material=badge_red,
        name="brand_badge",
    )
    body.visual(
        Cylinder(radius=0.006, length=chamber_open_w - 0.06),
        origin=Origin(xyz=(0.0, 0.006, top_open_z + 0.006), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="top_hinge_barrel",
    )
    body.visual(
        Cylinder(radius=0.006, length=chamber_open_w - 0.06),
        origin=Origin(xyz=(0.0, 0.006, bottom_open_z + 0.006), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="bottom_hinge_barrel",
    )

    knob_y = -0.017
    knob_z = height - 0.11
    for idx, knob_x in enumerate((-0.36, -0.12, 0.12, 0.36), start=1):
        body.visual(
            Cylinder(radius=0.022, length=0.034),
            origin=Origin(xyz=(knob_x, knob_y, knob_z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=knob_black,
            name=f"control_knob_{idx}",
        )

    rail_x = chamber_open_w / 2.0 - rail_w / 2.0
    top_rail_z = top_open_z + 0.008 + rail_t / 2.0
    bottom_rail_z = bottom_open_z + 0.008 + rail_t / 2.0
    body.visual(
        Box((rail_w, rail_len, rail_t)),
        origin=Origin(xyz=(-rail_x, rail_y, top_rail_z)),
        material=dark_steel,
        name="top_left_rail",
    )
    body.visual(
        Box((rail_w, rail_len, rail_t)),
        origin=Origin(xyz=(rail_x, rail_y, top_rail_z)),
        material=dark_steel,
        name="top_right_rail",
    )
    body.visual(
        Box((rail_w, rail_len, rail_t)),
        origin=Origin(xyz=(-rail_x, rail_y, bottom_rail_z)),
        material=dark_steel,
        name="bottom_left_rail",
    )
    body.visual(
        Box((rail_w, rail_len, rail_t)),
        origin=Origin(xyz=(rail_x, rail_y, bottom_rail_z)),
        material=dark_steel,
        name="bottom_right_rail",
    )

    def add_drop_door(name: str, hinge_name: str, hinge_z: float) -> None:
        door = model.part(name)
        door_w = chamber_open_w - 0.03
        door_h = 0.32
        door_t = 0.035
        panel_y = -door_t / 2.0 - 0.008
        handle_len = 0.76
        handle_r = 0.014
        handle_y = -0.083
        handle_z = 0.23
        bracket_y = -0.060

        door.visual(
            Box((door_w, door_t, door_h)),
            origin=Origin(xyz=(0.0, panel_y, door_h / 2.0)),
            material=steel,
            name="panel",
        )
        door.visual(
            Cylinder(radius=0.006, length=door_w - 0.08),
            origin=Origin(xyz=(0.0, -0.006, 0.006), rpy=(0.0, pi / 2.0, 0.0)),
            material=dark_steel,
            name="hinge_barrel",
        )
        door.visual(
            Box((0.56, 0.006, 0.12)),
            origin=Origin(xyz=(0.0, panel_y - door_t / 2.0 + 0.003, 0.17)),
            material=graphite,
            name="window_strip",
        )
        for side_x, bracket_name in ((-0.27, "left_bracket"), (0.27, "right_bracket")):
            door.visual(
                Box((0.04, 0.038, 0.05)),
                origin=Origin(xyz=(side_x, bracket_y, handle_z)),
                material=dark_steel,
                name=bracket_name,
            )
        door.visual(
            Cylinder(radius=handle_r, length=handle_len),
            origin=Origin(xyz=(0.0, handle_y, handle_z), rpy=(0.0, pi / 2.0, 0.0)),
            material=dark_steel,
            name="handle_bar",
        )
        door.inertial = Inertial.from_geometry(
            Box((door_w, door_t, door_h)),
            mass=18.0,
            origin=Origin(xyz=(0.0, panel_y, door_h / 2.0)),
        )

        model.articulation(
            hinge_name,
            ArticulationType.REVOLUTE,
            parent=body,
            child=door,
            origin=Origin(xyz=(0.0, 0.0, hinge_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=hinge_limits,
        )

    def add_hearth_rack(name: str, slide_name: str, chamber_base_z: float, rail_prefix: str) -> None:
        rack = model.part(name)
        runner_w = 0.020
        runner_len = 0.68
        runner_y = 0.05 + runner_len / 2.0
        lip_t = 0.020
        lip_h = 0.032
        runner_x = chamber_open_w / 2.0 - runner_w / 2.0 - 0.004
        rail_center_z_local = 0.008 + rail_t / 2.0
        runner_center_z_local = rail_center_z_local + rail_t / 2.0 + runner_t / 2.0
        deck_center_z_local = runner_center_z_local + runner_t / 2.0 + deck_t / 2.0
        lip_center_z_local = deck_center_z_local + 0.010
        grip_center_z_local = deck_center_z_local + 0.018

        rack.visual(
            Box((deck_w, deck_d, deck_t)),
            origin=Origin(xyz=(0.0, deck_center_y, deck_center_z_local)),
            material=stone,
            name="deck",
        )
        rack.visual(
            Box((deck_w, lip_t, lip_h)),
            origin=Origin(xyz=(0.0, 0.050, lip_center_z_local)),
            material=dark_steel,
            name="front_lip",
        )
        rack.visual(
            Box((runner_w, runner_len, runner_t)),
            origin=Origin(xyz=(-runner_x, runner_y, runner_center_z_local)),
            material=dark_steel,
            name="left_runner",
        )
        rack.visual(
            Box((runner_w, runner_len, runner_t)),
            origin=Origin(xyz=(runner_x, runner_y, runner_center_z_local)),
            material=dark_steel,
            name="right_runner",
        )
        rack.visual(
            Box((0.08, 0.020, 0.050)),
            origin=Origin(xyz=(0.0, 0.052, grip_center_z_local)),
            material=dark_steel,
            name="pull_grip",
        )
        rack.inertial = Inertial.from_geometry(
            Box((deck_w, deck_d, deck_t)),
            mass=14.0,
            origin=Origin(xyz=(0.0, deck_center_y, deck_center_z_local)),
        )

        model.articulation(
            slide_name,
            ArticulationType.PRISMATIC,
            parent=body,
            child=rack,
            origin=Origin(xyz=(0.0, front_frame_t, chamber_base_z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=slide_limits,
            meta={"rail_prefix": rail_prefix},
        )

    add_drop_door("top_door", "top_door_hinge", top_open_z)
    add_drop_door("bottom_door", "bottom_door_hinge", bottom_open_z)
    add_hearth_rack("top_rack", "top_rack_slide", top_open_z, "top")
    add_hearth_rack("bottom_rack", "bottom_rack_slide", bottom_open_z, "bottom")

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

    body = object_model.get_part("body")
    top_door = object_model.get_part("top_door")
    bottom_door = object_model.get_part("bottom_door")
    top_rack = object_model.get_part("top_rack")
    bottom_rack = object_model.get_part("bottom_rack")

    top_door_hinge = object_model.get_articulation("top_door_hinge")
    bottom_door_hinge = object_model.get_articulation("bottom_door_hinge")
    top_rack_slide = object_model.get_articulation("top_rack_slide")
    bottom_rack_slide = object_model.get_articulation("bottom_rack_slide")

    ctx.expect_gap(
        body,
        top_door,
        axis="y",
        positive_elem="left_wall",
        negative_elem="panel",
        max_gap=0.012,
        max_penetration=0.0,
        name="top door closes flush with cabinet front plane",
    )
    ctx.expect_gap(
        body,
        bottom_door,
        axis="y",
        positive_elem="left_wall",
        negative_elem="panel",
        max_gap=0.012,
        max_penetration=0.0,
        name="bottom door closes flush with cabinet front plane",
    )
    ctx.expect_contact(
        body,
        top_door,
        elem_a="top_hinge_barrel",
        elem_b="hinge_barrel",
        name="top door is physically supported by its hinge barrel",
    )
    ctx.expect_contact(
        body,
        bottom_door,
        elem_a="bottom_hinge_barrel",
        elem_b="hinge_barrel",
        name="bottom door is physically supported by its hinge barrel",
    )
    ctx.expect_contact(
        top_rack,
        body,
        elem_a="left_runner",
        elem_b="top_left_rail",
        name="top rack runner is supported by top left guide rail",
    )
    ctx.expect_contact(
        bottom_rack,
        body,
        elem_a="left_runner",
        elem_b="bottom_left_rail",
        name="bottom rack runner is supported by bottom left guide rail",
    )

    closed_top_door_aabb = ctx.part_world_aabb(top_door)
    with ctx.pose({top_door_hinge: top_door_hinge.motion_limits.upper}):
        open_top_door_aabb = ctx.part_world_aabb(top_door)
        ctx.expect_gap(
            body,
            top_door,
            axis="y",
            positive_elem="mid_front_band",
            negative_elem="panel",
            min_gap=0.0005,
            name="top door clears the front band when open",
        )
    ctx.check(
        "top door drops downward and outward when opened",
        closed_top_door_aabb is not None
        and open_top_door_aabb is not None
        and open_top_door_aabb[1][2] < closed_top_door_aabb[1][2] - 0.18
        and open_top_door_aabb[0][1] < closed_top_door_aabb[0][1] - 0.18,
        details=f"closed={closed_top_door_aabb}, open={open_top_door_aabb}",
    )

    closed_bottom_door_aabb = ctx.part_world_aabb(bottom_door)
    with ctx.pose({bottom_door_hinge: bottom_door_hinge.motion_limits.upper}):
        open_bottom_door_aabb = ctx.part_world_aabb(bottom_door)
        ctx.expect_gap(
            body,
            bottom_door,
            axis="y",
            positive_elem="bottom_front_sill",
            negative_elem="panel",
            min_gap=0.0005,
            name="bottom door clears the front sill when open",
        )
    ctx.check(
        "bottom door drops downward and outward when opened",
        closed_bottom_door_aabb is not None
        and open_bottom_door_aabb is not None
        and open_bottom_door_aabb[1][2] < closed_bottom_door_aabb[1][2] - 0.18
        and open_bottom_door_aabb[0][1] < closed_bottom_door_aabb[0][1] - 0.18,
        details=f"closed={closed_bottom_door_aabb}, open={open_bottom_door_aabb}",
    )

    top_rack_rest = ctx.part_world_position(top_rack)
    with ctx.pose({top_rack_slide: top_rack_slide.motion_limits.upper}):
        top_rack_extended = ctx.part_world_position(top_rack)
        ctx.expect_overlap(
            top_rack,
            body,
            axes="y",
            elem_a="left_runner",
            elem_b="top_left_rail",
            min_overlap=0.12,
            name="top rack retains insertion on its rail at full extension",
        )
        ctx.expect_contact(
            top_rack,
            body,
            elem_a="left_runner",
            elem_b="top_left_rail",
            name="top rack stays seated on its guide rail while extended",
        )
    ctx.check(
        "top rack slides outward toward the operator",
        top_rack_rest is not None
        and top_rack_extended is not None
        and top_rack_extended[1] < top_rack_rest[1] - 0.20,
        details=f"rest={top_rack_rest}, extended={top_rack_extended}",
    )

    bottom_rack_rest = ctx.part_world_position(bottom_rack)
    with ctx.pose({bottom_rack_slide: bottom_rack_slide.motion_limits.upper}):
        bottom_rack_extended = ctx.part_world_position(bottom_rack)
        ctx.expect_overlap(
            bottom_rack,
            body,
            axes="y",
            elem_a="left_runner",
            elem_b="bottom_left_rail",
            min_overlap=0.12,
            name="bottom rack retains insertion on its rail at full extension",
        )
        ctx.expect_contact(
            bottom_rack,
            body,
            elem_a="left_runner",
            elem_b="bottom_left_rail",
            name="bottom rack stays seated on its guide rail while extended",
        )
    ctx.check(
        "bottom rack slides outward toward the operator",
        bottom_rack_rest is not None
        and bottom_rack_extended is not None
        and bottom_rack_extended[1] < bottom_rack_rest[1] - 0.20,
        details=f"rest={bottom_rack_rest}, extended={bottom_rack_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
