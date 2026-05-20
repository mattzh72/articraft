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
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _deck_point(x_along: float, y_across: float, z_normal: float, pitch: float) -> tuple[float, float, float]:
    c = math.cos(pitch)
    s = math.sin(pitch)
    return (
        x_along * c - z_normal * s,
        y_across,
        x_along * s + z_normal * c,
    )


def _rectangular_tube_mesh(
    *,
    length: float,
    width: float,
    height: float,
    wall: float,
    name: str,
):
    outer = [
        (-height / 2.0, -width / 2.0),
        (height / 2.0, -width / 2.0),
        (height / 2.0, width / 2.0),
        (-height / 2.0, width / 2.0),
    ]
    inner = [
        (-(height - 2.0 * wall) / 2.0, -(width - 2.0 * wall) / 2.0),
        ((height - 2.0 * wall) / 2.0, -(width - 2.0 * wall) / 2.0),
        ((height - 2.0 * wall) / 2.0, (width - 2.0 * wall) / 2.0),
        (-(height - 2.0 * wall) / 2.0, (width - 2.0 * wall) / 2.0),
    ]
    geom = ExtrudeWithHolesGeometry(
        outer,
        [inner],
        length,
        cap=False,
        center=True,
        closed=True,
    )
    geom.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _guardrail_frame(part, *, frame_length: float, frame_height: float, tube_radius: float, material) -> None:
    post_offset = frame_length / 2.0 - 0.06

    part.visual(
        Cylinder(radius=tube_radius, length=frame_length),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name="hinge_tube",
    )
    for name, x_pos in (("front_post", -post_offset), ("rear_post", post_offset)):
        part.visual(
            Cylinder(radius=tube_radius, length=frame_height),
            origin=Origin(xyz=(x_pos, 0.0, tube_radius + frame_height / 2.0)),
            material=material,
            name=name,
        )
    part.visual(
        Cylinder(radius=tube_radius, length=frame_length),
        origin=Origin(
            xyz=(0.0, 0.0, tube_radius + frame_height),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=material,
        name="top_rail",
    )
    part.visual(
        Cylinder(radius=tube_radius * 0.9, length=frame_length * 0.92),
        origin=Origin(
            xyz=(0.0, 0.0, tube_radius + frame_height * 0.52),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=material,
        name="mid_rail",
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    low, high = aabb
    return tuple((low[i] + high[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fork_entry_dock_ramp")

    deck_length = 2.0
    deck_width = 1.0
    deck_rise = 0.22
    deck_pitch = math.asin(deck_rise / deck_length)
    deck_rpy = (0.0, -deck_pitch, 0.0)

    plate_thickness = 0.006
    side_beam_width = 0.08
    side_beam_height = 0.09
    fork_pocket_length = 1.10
    fork_pocket_width = 0.11
    fork_pocket_height = 0.07
    fork_pocket_wall = 0.006
    fork_pocket_spacing = 0.56

    hinge_center_z = 0.032
    hinge_offset_y = deck_width / 2.0 + 0.018
    bracket_size = (0.08, 0.045, 0.016)
    bracket_x_positions = (0.35, 1.0, 1.65)

    rail_frame_length = 1.72
    rail_height = 0.95
    rail_tube_radius = 0.016

    deck_steel = model.material("deck_steel", rgba=(0.28, 0.30, 0.33, 1.0))
    rail_yellow = model.material("rail_yellow", rgba=(0.92, 0.77, 0.16, 1.0))
    pocket_steel = model.material("pocket_steel", rgba=(0.20, 0.22, 0.25, 1.0))

    deck = model.part("deck")
    deck.visual(
        Box((deck_length, deck_width, plate_thickness)),
        origin=Origin(
            xyz=_deck_point(deck_length / 2.0, 0.0, -plate_thickness / 2.0, deck_pitch),
            rpy=deck_rpy,
        ),
        material=deck_steel,
        name="deck_plate",
    )
    for name, y_pos in (
        ("left_side_beam", -deck_width / 2.0 + side_beam_width / 2.0),
        ("right_side_beam", deck_width / 2.0 - side_beam_width / 2.0),
    ):
        deck.visual(
            Box((deck_length, side_beam_width, side_beam_height)),
            origin=Origin(
                xyz=_deck_point(
                    deck_length / 2.0,
                    y_pos,
                    -(plate_thickness + side_beam_height / 2.0),
                    deck_pitch,
                ),
                rpy=deck_rpy,
            ),
            material=deck_steel,
            name=name,
        )
    deck.visual(
        Box((0.16, deck_width - 0.16, 0.06)),
        origin=Origin(
            xyz=_deck_point(1.62, 0.0, -(plate_thickness + 0.03), deck_pitch),
            rpy=deck_rpy,
        ),
        material=deck_steel,
        name="rear_crossmember",
    )

    fork_pocket_mesh = _rectangular_tube_mesh(
        length=fork_pocket_length,
        width=fork_pocket_width,
        height=fork_pocket_height,
        wall=fork_pocket_wall,
        name="fork_pocket_tube",
    )
    for name, y_pos in (
        ("left_fork_pocket", -fork_pocket_spacing / 2.0),
        ("right_fork_pocket", fork_pocket_spacing / 2.0),
    ):
        deck.visual(
            fork_pocket_mesh,
            origin=Origin(
                xyz=_deck_point(
                    0.75,
                    y_pos,
                    -(plate_thickness + fork_pocket_height / 2.0),
                    deck_pitch,
                ),
                rpy=deck_rpy,
            ),
            material=pocket_steel,
            name=name,
        )

    for side_name, y_pos in (("left", -hinge_offset_y), ("right", hinge_offset_y)):
        for label, x_pos in zip(("front", "mid", "rear"), bracket_x_positions):
            deck.visual(
                Box(bracket_size),
                origin=Origin(
                    xyz=_deck_point(x_pos, y_pos, bracket_size[2] / 2.0, deck_pitch),
                    rpy=deck_rpy,
                ),
                material=deck_steel,
                name=f"{side_name}_bracket_{label}",
            )

    left_guardrail = model.part("left_guardrail")
    _guardrail_frame(
        left_guardrail,
        frame_length=rail_frame_length,
        frame_height=rail_height,
        tube_radius=rail_tube_radius,
        material=rail_yellow,
    )

    right_guardrail = model.part("right_guardrail")
    _guardrail_frame(
        right_guardrail,
        frame_length=rail_frame_length,
        frame_height=rail_height,
        tube_radius=rail_tube_radius,
        material=rail_yellow,
    )

    model.articulation(
        "deck_to_left_guardrail",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=left_guardrail,
        origin=Origin(
            xyz=_deck_point(deck_length / 2.0, -hinge_offset_y, hinge_center_z, deck_pitch),
            rpy=deck_rpy,
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.5,
            lower=0.0,
            upper=1.52,
        ),
    )
    model.articulation(
        "deck_to_right_guardrail",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=right_guardrail,
        origin=Origin(
            xyz=_deck_point(deck_length / 2.0, hinge_offset_y, hinge_center_z, deck_pitch),
            rpy=deck_rpy,
        ),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.5,
            lower=0.0,
            upper=1.52,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    left_guardrail = object_model.get_part("left_guardrail")
    right_guardrail = object_model.get_part("right_guardrail")
    left_hinge = object_model.get_articulation("deck_to_left_guardrail")
    right_hinge = object_model.get_articulation("deck_to_right_guardrail")

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

    ctx.expect_contact(
        left_guardrail,
        deck,
        elem_a="hinge_tube",
        elem_b="left_bracket_mid",
        name="left guardrail hinge tube bears on left deck bracket",
    )
    ctx.expect_contact(
        right_guardrail,
        deck,
        elem_a="hinge_tube",
        elem_b="right_bracket_mid",
        name="right guardrail hinge tube bears on right deck bracket",
    )

    left_limits = left_hinge.motion_limits
    right_limits = right_hinge.motion_limits
    left_upper = 0.0 if left_limits is None or left_limits.upper is None else left_limits.upper
    right_upper = 0.0 if right_limits is None or right_limits.upper is None else right_limits.upper

    left_deployed = _aabb_center(ctx.part_element_world_aabb(left_guardrail, elem="top_rail"))
    right_deployed = _aabb_center(ctx.part_element_world_aabb(right_guardrail, elem="top_rail"))

    with ctx.pose({left_hinge: left_upper, right_hinge: right_upper}):
        ctx.fail_if_parts_overlap_in_current_pose(
            name="folded guardrails clear the inclined deck and each other",
        )
        ctx.expect_gap(
            deck,
            left_guardrail,
            axis="y",
            positive_elem="deck_plate",
            negative_elem="top_rail",
            min_gap=0.90,
            name="left folded top rail stows outside the deck edge",
        )
        ctx.expect_gap(
            right_guardrail,
            deck,
            axis="y",
            positive_elem="top_rail",
            negative_elem="deck_plate",
            min_gap=0.90,
            name="right folded top rail stows outside the deck edge",
        )
        left_folded = _aabb_center(ctx.part_element_world_aabb(left_guardrail, elem="top_rail"))
        right_folded = _aabb_center(ctx.part_element_world_aabb(right_guardrail, elem="top_rail"))

    ctx.check(
        "left guardrail folds outward from upright to side-stowed position",
        left_deployed is not None
        and left_folded is not None
        and left_folded[1] < left_deployed[1] - 0.75
        and left_folded[2] < left_deployed[2] - 0.65,
        details=f"deployed={left_deployed}, folded={left_folded}",
    )
    ctx.check(
        "right guardrail folds outward from upright to side-stowed position",
        right_deployed is not None
        and right_folded is not None
        and right_folded[1] > right_deployed[1] + 0.75
        and right_folded[2] < right_deployed[2] - 0.65,
        details=f"deployed={right_deployed}, folded={right_folded}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
