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
    model = ArticulatedObject(name="portable_aluminum_yard_ramp")

    aluminum = model.material("aluminum", rgba=(0.78, 0.80, 0.82, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.28, 0.30, 0.33, 1.0))
    foot_pad = model.material("foot_pad", rgba=(0.18, 0.18, 0.18, 1.0))

    ramp_angle = math.radians(14.0)
    cos_a = math.cos(ramp_angle)
    sin_a = math.sin(ramp_angle)

    deck_length = 3.60
    deck_width = 1.05
    lip_length = 0.18
    dock_lip_length = 0.12
    tread_width = 0.89
    tread_thickness = 0.018
    lip_thickness = 0.008
    dock_lip_thickness = 0.012
    side_beam_width = 0.08
    side_beam_depth = 0.12
    cross_beam_depth = 0.07
    cross_beam_length = 0.08
    cross_beam_span = deck_width - (2.0 * side_beam_width)
    bracket_support_thickness = 0.045
    bracket_lug_length = 0.032
    bracket_support_height = 0.06
    hinge_radius = 0.028
    hinge_length = 0.10
    leg_length = 0.84
    leg_size = (0.06, 0.04, leg_length)
    foot_size = (0.16, 0.09, 0.025)

    beam_center_y = (deck_width / 2.0) - (side_beam_width / 2.0)
    beam_bottom_z = -(tread_thickness + side_beam_depth)
    hinge_x = 0.68
    hinge_y = beam_center_y
    hinge_z = beam_bottom_z - bracket_support_height - hinge_radius

    def deck_point(local_x: float, local_y: float, local_z: float) -> tuple[float, float, float]:
        return (
            (local_x * cos_a) + (local_z * sin_a),
            local_y,
            (local_x * sin_a) + (local_z * cos_a),
        )

    def deck_origin(local_x: float, local_y: float, local_z: float) -> Origin:
        return Origin(xyz=deck_point(local_x, local_y, local_z), rpy=(0.0, -ramp_angle, 0.0))

    def normalized(vec: tuple[float, float, float]) -> tuple[float, float, float]:
        length = math.sqrt(sum(component * component for component in vec))
        return tuple(component / length for component in vec)

    def direction_rpy(direction: tuple[float, float, float]) -> tuple[float, float, float]:
        dx, dy, dz = normalized(direction)
        roll = -math.asin(dy)
        pitch = math.atan2(dx, dz)
        return (roll, pitch, 0.0)

    deck = model.part("deck")

    main_tread_length = deck_length - lip_length - dock_lip_length
    side_beam_length = deck_length - lip_length
    dock_stop_height = 0.045

    deck.visual(
        Box((lip_length, tread_width, lip_thickness)),
        origin=deck_origin(lip_length / 2.0, 0.0, -(lip_thickness / 2.0)),
        material=aluminum,
        name="entry_lip",
    )
    deck.visual(
        Box((main_tread_length, tread_width, tread_thickness)),
        origin=deck_origin(
            lip_length + (main_tread_length / 2.0),
            0.0,
            -(tread_thickness / 2.0),
        ),
        material=aluminum,
        name="main_tread",
    )
    deck.visual(
        Box((dock_lip_length, tread_width, dock_lip_thickness)),
        origin=deck_origin(
            deck_length - (dock_lip_length / 2.0),
            0.0,
            -(dock_lip_thickness / 2.0),
        ),
        material=aluminum,
        name="dock_lip",
    )

    for side_sign, visual_name in ((1.0, "left_side_beam"), (-1.0, "right_side_beam")):
        deck.visual(
            Box((side_beam_length, side_beam_width, side_beam_depth)),
            origin=deck_origin(
                lip_length + (side_beam_length / 2.0),
                side_sign * beam_center_y,
                -(tread_thickness + (side_beam_depth / 2.0)),
            ),
            material=aluminum,
            name=visual_name,
        )

    for index, cross_x in enumerate((0.52, 1.35, 2.18, 3.02), start=1):
        deck.visual(
            Box((cross_beam_length, cross_beam_span, cross_beam_depth)),
            origin=deck_origin(
                cross_x,
                0.0,
                -(tread_thickness + (cross_beam_depth / 2.0)),
            ),
            material=aluminum,
            name=f"cross_beam_{index}",
        )

    deck.visual(
        Box((0.10, deck_width - 0.16, 0.05)),
        origin=deck_origin(
            hinge_x,
            0.0,
            beam_bottom_z - 0.025,
        ),
        material=dark_steel,
        name="leg_cross_tie",
    )

    for side_sign, base_name in ((1.0, "left"), (-1.0, "right")):
        deck.visual(
            Box((bracket_support_thickness, bracket_lug_length, bracket_support_height)),
            origin=deck_origin(
                hinge_x,
                (side_sign * hinge_y) - (side_sign * ((hinge_length + bracket_lug_length) / 2.0)),
                beam_bottom_z - (bracket_support_height / 2.0),
            ),
            material=dark_steel,
            name=f"{base_name}_bracket_support",
        )
        deck.visual(
            Cylinder(radius=hinge_radius, length=bracket_lug_length),
            origin=Origin(
                xyz=deck_point(
                    hinge_x,
                    (side_sign * hinge_y) - (side_sign * ((hinge_length + bracket_lug_length) / 2.0)),
                    hinge_z,
                ),
                rpy=(math.pi / 2.0, -ramp_angle, 0.0),
            ),
            material=dark_steel,
            name=f"{base_name}_bracket",
        )

    deck.visual(
        Box((0.08, deck_width, dock_stop_height)),
        origin=deck_origin(
            deck_length - 0.04,
            0.0,
            dock_stop_height / 2.0,
        ),
        material=dark_steel,
        name="dock_stop",
    )
    deck.inertial = Inertial.from_geometry(
        Box((deck_length, deck_width, 0.18)),
        mass=170.0,
        origin=deck_origin(deck_length / 2.0, 0.0, -0.06),
    )

    def add_support_leg(part_name: str, side_sign: float) -> None:
        leg = model.part(part_name)
        direction = normalized((-0.34, 0.12 * side_sign, -0.93))
        tube_rpy = direction_rpy(direction)
        foot_embed = 0.006
        foot_mount_offset = (foot_size[2] / 2.0) - foot_embed
        foot_center = (
            (direction[0] * leg_length) - (math.sin(ramp_angle) * foot_mount_offset),
            direction[1] * leg_length,
            (direction[2] * leg_length) - (math.cos(ramp_angle) * foot_mount_offset),
        )

        leg.visual(
            Box(leg_size),
            origin=Origin(
                xyz=tuple(component * (leg_length / 2.0) for component in direction),
                rpy=tube_rpy,
            ),
            material=aluminum,
            name="tube",
        )
        leg.visual(
            Cylinder(radius=hinge_radius, length=hinge_length),
            origin=Origin(
                xyz=(0.0, 0.0, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_steel,
            name="knuckle",
        )
        leg.visual(
            Box(foot_size),
            origin=Origin(
                xyz=foot_center,
                rpy=(0.0, ramp_angle, 0.0),
            ),
            material=foot_pad,
            name="foot",
        )
        leg.inertial = Inertial.from_geometry(
            Box((0.18, 0.10, leg_length + 0.08)),
            mass=18.0,
            origin=Origin(
                xyz=tuple(component * ((leg_length + 0.08) / 2.0) for component in direction),
            ),
        )

        model.articulation(
            f"deck_to_{part_name}",
            ArticulationType.REVOLUTE,
            parent=deck,
            child=leg,
            origin=Origin(
                xyz=deck_point(hinge_x, side_sign * hinge_y, hinge_z),
                rpy=(0.0, -ramp_angle, 0.0),
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=250.0,
                velocity=1.2,
                lower=-1.20,
                upper=0.10,
            ),
        )

    add_support_leg("left_leg", 1.0)
    add_support_leg("right_leg", -1.0)

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

    deck = object_model.get_part("deck")
    left_leg = object_model.get_part("left_leg")
    right_leg = object_model.get_part("right_leg")
    left_hinge = object_model.get_articulation("deck_to_left_leg")
    right_hinge = object_model.get_articulation("deck_to_right_leg")
    ctx.expect_contact(
        left_leg,
        deck,
        elem_a="knuckle",
        elem_b="left_bracket",
        name="left leg knuckle seats against the left deck bracket lug",
    )
    ctx.expect_contact(
        right_leg,
        deck,
        elem_a="knuckle",
        elem_b="right_bracket",
        name="right leg knuckle seats against the right deck bracket lug",
    )
    ctx.expect_gap(
        deck,
        left_leg,
        axis="z",
        positive_elem="main_tread",
        negative_elem="foot",
        min_gap=0.55,
        name="left foot hangs well below the inclined deck",
    )
    ctx.expect_gap(
        deck,
        right_leg,
        axis="z",
        positive_elem="main_tread",
        negative_elem="foot",
        min_gap=0.55,
        name="right foot hangs well below the inclined deck",
    )
    ctx.expect_origin_distance(
        left_leg,
        right_leg,
        axes="y",
        min_dist=0.80,
        name="support hinges are spread across the deck width",
    )

    def aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((low + high) / 2.0 for low, high in zip(lower, upper))

    deployed_left_foot = aabb_center(ctx.part_element_world_aabb(left_leg, elem="foot"))
    deployed_right_foot = aabb_center(ctx.part_element_world_aabb(right_leg, elem="foot"))

    with ctx.pose({left_hinge: left_hinge.motion_limits.lower, right_hinge: right_hinge.motion_limits.lower}):
        folded_left_foot = aabb_center(ctx.part_element_world_aabb(left_leg, elem="foot"))
        folded_right_foot = aabb_center(ctx.part_element_world_aabb(right_leg, elem="foot"))

    ctx.check(
        "left leg folds upward toward the deck",
        deployed_left_foot is not None
        and folded_left_foot is not None
        and folded_left_foot[2] > deployed_left_foot[2] + 0.20
        and folded_left_foot[0] > deployed_left_foot[0] + 0.35,
        details=f"deployed={deployed_left_foot}, folded={folded_left_foot}",
    )
    ctx.check(
        "right leg folds upward toward the deck",
        deployed_right_foot is not None
        and folded_right_foot is not None
        and folded_right_foot[2] > deployed_right_foot[2] + 0.20
        and folded_right_foot[0] > deployed_right_foot[0] + 0.35,
        details=f"deployed={deployed_right_foot}, folded={folded_right_foot}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
