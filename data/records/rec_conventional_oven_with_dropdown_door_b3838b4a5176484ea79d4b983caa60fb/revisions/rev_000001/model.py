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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pizza_stone_deck_oven")

    steel = model.material("steel", rgba=(0.39, 0.41, 0.43, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.20, 0.21, 0.22, 1.0))
    rail_steel = model.material("rail_steel", rgba=(0.49, 0.51, 0.53, 1.0))
    stone = model.material("pizza_stone", rgba=(0.77, 0.71, 0.60, 1.0))
    handle_black = model.material("handle_black", rgba=(0.08, 0.08, 0.09, 1.0))

    outer_w = 0.78
    outer_d = 0.72
    outer_h = 0.42
    side_t = 0.015
    base_t = 0.018
    roof_t = 0.040
    frame_d = 0.025

    opening_w = 0.66
    opening_h = 0.20
    opening_bottom_z = 0.070
    opening_top_z = opening_bottom_z + opening_h
    jamb_w = ((outer_w - 2.0 * side_t) - opening_w) * 0.5

    rail_len = 0.58
    rail_w = 0.120
    rail_h = 0.014
    rail_center_y = -0.040
    rail_bottom_z = 0.074

    housing = model.part("housing")
    housing.visual(
        Box((outer_w, outer_d, base_t)),
        origin=Origin(xyz=(0.0, 0.0, base_t * 0.5)),
        material=dark_steel,
        name="bottom_shell",
    )
    housing.visual(
        Box((side_t, outer_d, outer_h - base_t)),
        origin=Origin(
            xyz=(
                -(outer_w * 0.5 - side_t * 0.5),
                0.0,
                base_t + (outer_h - base_t) * 0.5,
            )
        ),
        material=steel,
        name="left_shell",
    )
    housing.visual(
        Box((side_t, outer_d, outer_h - base_t)),
        origin=Origin(
            xyz=(
                outer_w * 0.5 - side_t * 0.5,
                0.0,
                base_t + (outer_h - base_t) * 0.5,
            )
        ),
        material=steel,
        name="right_shell",
    )
    housing.visual(
        Box((outer_w - 2.0 * side_t, side_t, outer_h - base_t)),
        origin=Origin(
            xyz=(
                0.0,
                outer_d * 0.5 - side_t * 0.5,
                base_t + (outer_h - base_t) * 0.5,
            )
        ),
        material=steel,
        name="back_shell",
    )
    housing.visual(
        Box((outer_w, outer_d, roof_t)),
        origin=Origin(xyz=(0.0, 0.0, outer_h - roof_t * 0.5)),
        material=steel,
        name="top_shell",
    )
    housing.visual(
        Box((jamb_w, frame_d, opening_top_z)),
        origin=Origin(
            xyz=(
                -(opening_w * 0.5 + jamb_w * 0.5),
                -outer_d * 0.5 + frame_d * 0.5,
                opening_top_z * 0.5,
            )
        ),
        material=steel,
        name="left_jamb",
    )
    housing.visual(
        Box((jamb_w, frame_d, opening_top_z)),
        origin=Origin(
            xyz=(
                opening_w * 0.5 + jamb_w * 0.5,
                -outer_d * 0.5 + frame_d * 0.5,
                opening_top_z * 0.5,
            )
        ),
        material=steel,
        name="right_jamb",
    )
    housing.visual(
        Box((opening_w + 2.0 * jamb_w, frame_d, outer_h - opening_top_z)),
        origin=Origin(
            xyz=(
                0.0,
                -outer_d * 0.5 + frame_d * 0.5,
                opening_top_z + (outer_h - opening_top_z) * 0.5,
            )
        ),
        material=steel,
        name="front_lintel",
    )
    housing.visual(
        Box((opening_w + 2.0 * jamb_w, frame_d, opening_bottom_z)),
        origin=Origin(
            xyz=(
                0.0,
                -outer_d * 0.5 + frame_d * 0.5,
                opening_bottom_z * 0.5,
            )
        ),
        material=dark_steel,
        name="front_sill",
    )
    housing.visual(
        Box((rail_w, rail_len, rail_h)),
        origin=Origin(
            xyz=(
                -0.315,
                rail_center_y,
                rail_bottom_z + rail_h * 0.5,
            )
        ),
        material=rail_steel,
        name="left_rail",
    )
    housing.visual(
        Box((rail_w, rail_len, rail_h)),
        origin=Origin(
            xyz=(
                0.315,
                rail_center_y,
                rail_bottom_z + rail_h * 0.5,
            )
        ),
        material=rail_steel,
        name="right_rail",
    )
    housing.inertial = Inertial.from_geometry(
        Box((outer_w, outer_d, outer_h)),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, outer_h * 0.5)),
    )

    door = model.part("door")
    door_w = 0.670
    door_h = 0.210
    door_t = 0.028
    door.visual(
        Box((door_w, door_t, door_h)),
        origin=Origin(xyz=(0.0, 0.0, door_h * 0.5)),
        material=dark_steel,
        name="door_panel",
    )
    door.visual(
        Box((0.095, 0.014, 0.020)),
        origin=Origin(xyz=(-0.205, -0.021, 0.115)),
        material=handle_black,
        name="left_handle_standoff",
    )
    door.visual(
        Box((0.095, 0.014, 0.020)),
        origin=Origin(xyz=(0.205, -0.021, 0.115)),
        material=handle_black,
        name="right_handle_standoff",
    )
    door.visual(
        Cylinder(radius=0.010, length=0.450),
        origin=Origin(
            xyz=(0.0, -0.030, 0.115),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=handle_black,
        name="door_handle_bar",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_w, 0.070, door_h)),
        mass=7.5,
        origin=Origin(xyz=(0.0, -0.010, door_h * 0.5)),
    )

    hearth_deck = model.part("hearth_deck")
    runner_w = 0.030
    runner_h = rail_h
    deck_w = 0.600
    deck_d = 0.600
    tray_t = 0.004
    stone_t = 0.020
    front_lip_h = 0.026
    hearth_deck.visual(
        Box((runner_w, deck_d, runner_h)),
        origin=Origin(xyz=(-0.285, deck_d * 0.5, runner_h * 0.5)),
        material=rail_steel,
        name="left_runner",
    )
    hearth_deck.visual(
        Box((runner_w, deck_d, runner_h)),
        origin=Origin(xyz=(0.285, deck_d * 0.5, runner_h * 0.5)),
        material=rail_steel,
        name="right_runner",
    )
    hearth_deck.visual(
        Box((deck_w, deck_d, tray_t)),
        origin=Origin(xyz=(0.0, deck_d * 0.5, runner_h + tray_t * 0.5)),
        material=dark_steel,
        name="deck_base",
    )
    hearth_deck.visual(
        Box((0.560, 0.560, stone_t)),
        origin=Origin(
            xyz=(
                0.0,
                0.020 + 0.560 * 0.5,
                runner_h + tray_t + stone_t * 0.5,
            )
        ),
        material=stone,
        name="stone_slab",
    )
    hearth_deck.visual(
        Box((deck_w, 0.010, front_lip_h)),
        origin=Origin(xyz=(0.0, 0.004, front_lip_h * 0.5)),
        material=dark_steel,
        name="front_lip",
    )
    hearth_deck.inertial = Inertial.from_geometry(
        Box((deck_w, deck_d, 0.040)),
        mass=10.0,
        origin=Origin(xyz=(0.0, deck_d * 0.5, 0.020)),
    )

    model.articulation(
        "housing_to_door",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(
            xyz=(
                0.0,
                -outer_d * 0.5 - door_t * 0.5,
                opening_bottom_z,
            )
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.4,
            lower=0.0,
            upper=1.55,
        ),
    )
    model.articulation(
        "housing_to_hearth_deck",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=hearth_deck,
        origin=Origin(
            xyz=(
                0.0,
                -0.320,
                rail_bottom_z + rail_h,
            )
        ),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.20,
            lower=0.0,
            upper=0.260,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    door = object_model.get_part("door")
    hearth_deck = object_model.get_part("hearth_deck")
    door_hinge = object_model.get_articulation("housing_to_door")
    deck_slide = object_model.get_articulation("housing_to_hearth_deck")

    with ctx.pose({door_hinge: 0.0, deck_slide: 0.0}):
        ctx.expect_gap(
            housing,
            door,
            axis="y",
            min_gap=0.0,
            max_gap=0.003,
            name="closed door seats against the housing face",
        )
        ctx.expect_overlap(
            door,
            housing,
            axes="xz",
            min_overlap=0.19,
            name="closed door covers the oven opening footprint",
        )
        ctx.expect_within(
            hearth_deck,
            housing,
            axes="xz",
            margin=0.030,
            name="hearth deck stays centered within the oven chamber",
        )
        ctx.expect_overlap(
            hearth_deck,
            housing,
            axes="y",
            min_overlap=0.55,
            name="resting hearth deck remains deeply inserted",
        )
        closed_door_panel = ctx.part_element_world_aabb(door, elem="door_panel")
        closed_front_lip = ctx.part_element_world_aabb(hearth_deck, elem="front_lip")

    open_angle = 1.53
    extended_travel = 0.240
    with ctx.pose({door_hinge: open_angle, deck_slide: extended_travel}):
        ctx.expect_overlap(
            hearth_deck,
            housing,
            axes="y",
            min_overlap=0.08,
            name="extended hearth deck still retains insertion on the rails",
        )
        ctx.expect_gap(
            hearth_deck,
            door,
            axis="z",
            min_gap=0.005,
            positive_elem="stone_slab",
            negative_elem="door_panel",
            name="open door drops below the sliding hearth deck",
        )
        open_door_panel = ctx.part_element_world_aabb(door, elem="door_panel")
        extended_front_lip = ctx.part_element_world_aabb(hearth_deck, elem="front_lip")

    ctx.check(
        "door rotates downward and outward",
        closed_door_panel is not None
        and open_door_panel is not None
        and open_door_panel[0][1] < closed_door_panel[0][1] - 0.15
        and open_door_panel[1][2] < closed_door_panel[1][2] - 0.10,
        details=f"closed={closed_door_panel}, open={open_door_panel}",
    )
    ctx.check(
        "hearth deck slides outward from the chamber",
        closed_front_lip is not None
        and extended_front_lip is not None
        and extended_front_lip[0][1] < closed_front_lip[0][1] - 0.20,
        details=f"closed={closed_front_lip}, extended={extended_front_lip}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
