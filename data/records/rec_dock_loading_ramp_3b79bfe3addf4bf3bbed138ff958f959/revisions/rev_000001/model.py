from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


DOCK_HEIGHT = 1.18
DECK_WIDTH = 2.24
DECK_AXIS_TO_LIP = 3.17
DECK_PLATE_LENGTH = 3.10
DECK_THICKNESS = 0.05
DECK_SURFACE_Z = -0.005

LEG_PAIR_SPAN = 1.68
LEG_Y = 0.78
LEG_PAIR_0_X = 1.22
LEG_PAIR_1_X = 2.28
LEG_HINGE_Z = -0.13
LEG_LENGTH = 1.00


def _add_y_knuckle(part, *, x: float, y: float, z: float, radius: float, length: float, name: str, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(x, y, z), rpy=(1.5707963267948966, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _add_leg_brackets(deck, *, x: float, name_prefix: str, material) -> None:
    for side, y in (("a", -LEG_Y), ("b", LEG_Y)):
        deck.visual(
            Box((0.14, 0.08, 0.03)),
            origin=Origin(xyz=(x, y, -0.070)),
            material=material,
            name=f"{name_prefix}_pad_{side}",
        )
        deck.visual(
            Box((0.012, 0.08, 0.14)),
            origin=Origin(xyz=(x - 0.050, y, -0.125)),
            material=material,
            name=f"{name_prefix}_cheek_front_{side}",
        )
        deck.visual(
            Box((0.012, 0.08, 0.14)),
            origin=Origin(xyz=(x + 0.050, y, -0.125)),
            material=material,
            name=f"{name_prefix}_cheek_rear_{side}",
        )


def _add_leg_pair(model: ArticulatedObject, *, name: str, x: float, material):
    leg_pair = model.part(name)
    leg_pair.visual(
        Cylinder(radius=0.035, length=LEG_PAIR_SPAN),
        origin=Origin(rpy=(1.5707963267948966, 0.0, 0.0)),
        material=material,
        name="cross_tube",
    )
    for side, y in (("neg", -LEG_Y), ("pos", LEG_Y)):
        leg_pair.visual(
            Box((0.09, 0.08, LEG_LENGTH)),
            origin=Origin(xyz=(0.0, y, -LEG_LENGTH / 2.0)),
            material=material,
            name=f"leg_{side}",
        )
    leg_pair.visual(
        Box((0.14, 1.86, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, -1.03)),
        material=material,
        name="foot_bar",
    )
    leg_pair.visual(
        Box((0.05, 1.54, 0.05)),
        origin=Origin(xyz=(-0.02, 0.0, -0.55)),
        material=material,
        name="tie_bar",
    )

    model.articulation(
        f"{name}_hinge",
        ArticulationType.REVOLUTE,
        parent="deck",
        child=leg_pair,
        origin=Origin(xyz=(x, 0.0, LEG_HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=1.15,
            effort=4000.0,
            velocity=1.5,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dock_loading_ramp")

    steel = model.material("steel", rgba=(0.33, 0.35, 0.38, 1.0))
    painted = model.material("painted_steel", rgba=(0.47, 0.49, 0.52, 1.0))
    dark = model.material("dark_steel", rgba=(0.18, 0.19, 0.21, 1.0))

    dock_frame = model.part("dock_frame")
    dock_frame.visual(
        Box((1.00, 2.60, 0.08)),
        origin=Origin(xyz=(-0.50, 0.0, DOCK_HEIGHT + 0.04)),
        material=painted,
        name="dock_slab",
    )
    dock_frame.visual(
        Box((0.20, 2.60, DOCK_HEIGHT)),
        origin=Origin(xyz=(-0.50, 0.0, DOCK_HEIGHT / 2.0)),
        material=dark,
        name="dock_face",
    )
    dock_frame.visual(
        Box((0.28, 2.40, 0.16)),
        origin=Origin(xyz=(-0.14, 0.0, DOCK_HEIGHT - 0.08)),
        material=steel,
        name="hinge_header",
    )
    for index, y in enumerate((-0.72, 0.0, 0.72)):
        _add_y_knuckle(
            dock_frame,
            x=-0.03,
            y=y,
            z=DOCK_HEIGHT,
            radius=0.03,
            length=0.22,
            name=f"rear_knuckle_{index}",
            material=dark,
        )

    deck = model.part("deck")
    deck.visual(
        Box((DECK_PLATE_LENGTH, DECK_WIDTH, DECK_THICKNESS)),
        origin=Origin(xyz=(1.61, 0.0, -0.030)),
        material=painted,
        name="deck_plate",
    )
    deck.visual(
        Box((3.00, 0.14, 0.16)),
        origin=Origin(xyz=(1.58, -1.01, -0.135)),
        material=steel,
        name="side_beam_0",
    )
    deck.visual(
        Box((3.00, 0.14, 0.16)),
        origin=Origin(xyz=(1.58, 1.01, -0.135)),
        material=steel,
        name="side_beam_1",
    )
    for index, (x, y, length) in enumerate(
        (
            (0.55, -0.42, 0.90),
            (0.55, 0.42, 0.90),
            (1.75, -0.42, 0.80),
            (1.75, 0.42, 0.80),
            (2.78, -0.42, 0.70),
            (2.78, 0.42, 0.70),
        )
    ):
        deck.visual(
            Box((length, 0.10, 0.14)),
            origin=Origin(xyz=(x, y, -0.125)),
            material=steel,
            name=f"inner_stringer_{index}",
        )
    deck.visual(
        Box((0.16, DECK_WIDTH, 0.16)),
        origin=Origin(xyz=(0.08, 0.0, -0.135)),
        material=steel,
        name="rear_beam",
    )
    deck.visual(
        Box((0.12, DECK_WIDTH, 0.14)),
        origin=Origin(xyz=(3.07, 0.0, -0.125)),
        material=steel,
        name="front_beam",
    )
    for index, x in enumerate((0.58, 1.12, 1.66, 2.20, 2.74)):
        deck.visual(
            Box((0.05, 2.00, 0.008)),
            origin=Origin(xyz=(x, 0.0, DECK_SURFACE_Z + 0.004)),
            material=steel,
            name=f"traction_bar_{index}",
        )
    for index, y in enumerate((-0.36, 0.36)):
        _add_y_knuckle(
            deck,
            x=0.04,
            y=y,
            z=0.0,
            radius=0.03,
            length=0.22,
            name=f"rear_lug_{index}",
            material=dark,
        )
    for side, y in enumerate((-0.72, 0.0, 0.72)):
        deck.visual(
            Box((0.05, 0.10, 0.06)),
            origin=Origin(xyz=(3.145, y, -0.050)),
            material=steel,
            name=f"lip_hanger_{side}",
        )
        _add_y_knuckle(
            deck,
            x=3.15,
            y=y,
            z=-0.020,
            radius=0.020,
            length=0.20,
            name=f"lip_knuckle_{side}",
            material=dark,
        )
    _add_leg_brackets(deck, x=LEG_PAIR_0_X, name_prefix="leg_pair_0_bracket", material=steel)
    _add_leg_brackets(deck, x=LEG_PAIR_1_X, name_prefix="leg_pair_1_bracket", material=steel)

    lip = model.part("lip")
    lip.visual(
        Box((0.34, 2.10, 0.035)),
        origin=Origin(xyz=(0.17, 0.0, -0.0025)),
        material=painted,
        name="lip_plate",
    )
    lip.visual(
        Cylinder(radius=0.018, length=2.10),
        origin=Origin(xyz=(0.33, 0.0, -0.012), rpy=(1.5707963267948966, 0.0, 0.0)),
        material=steel,
        name="nose_bar",
    )
    for index, y in enumerate((-0.36, 0.36)):
        _add_y_knuckle(
            lip,
            x=0.02,
            y=y,
            z=0.0,
            radius=0.020,
            length=0.20,
            name=f"lip_lug_{index}",
            material=dark,
        )
    for index, x in enumerate((0.09, 0.20, 0.29)):
        lip.visual(
            Box((0.02, 1.86, 0.006)),
            origin=Origin(xyz=(x, 0.0, 0.015)),
            material=steel,
            name=f"lip_rib_{index}",
        )

    model.articulation(
        "rear_hinge",
        ArticulationType.REVOLUTE,
        parent=dock_frame,
        child=deck,
        origin=Origin(xyz=(0.0, 0.0, DOCK_HEIGHT)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.20,
            upper=1.22,
            effort=12000.0,
            velocity=0.8,
        ),
    )
    model.articulation(
        "lip_hinge",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=lip,
        origin=Origin(xyz=(DECK_AXIS_TO_LIP, 0.0, -0.020)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=0.72,
            effort=2000.0,
            velocity=1.5,
        ),
    )

    _add_leg_pair(model, name="leg_pair_0", x=LEG_PAIR_0_X, material=steel)
    _add_leg_pair(model, name="leg_pair_1", x=LEG_PAIR_1_X, material=steel)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck")
    lip = object_model.get_part("lip")
    leg_pair_0 = object_model.get_part("leg_pair_0")
    leg_pair_1 = object_model.get_part("leg_pair_1")
    rear_hinge = object_model.get_articulation("rear_hinge")
    lip_hinge = object_model.get_articulation("lip_hinge")
    leg_pair_0_hinge = object_model.get_articulation("leg_pair_0_hinge")
    leg_pair_1_hinge = object_model.get_articulation("leg_pair_1_hinge")

    ctx.expect_gap(
        lip,
        deck,
        axis="x",
        positive_elem="lip_plate",
        negative_elem="deck_plate",
        min_gap=0.0,
        max_gap=0.03,
        name="lip sits at the deck front edge",
    )
    ctx.expect_overlap(
        lip,
        deck,
        axes="y",
        elem_a="lip_plate",
        elem_b="deck_plate",
        min_overlap=1.90,
        name="lip spans nearly the full deck width",
    )

    rear_limits = rear_hinge.motion_limits
    if rear_limits is not None and rear_limits.upper is not None:
        rest_front = ctx.part_element_world_aabb(deck, elem="front_beam")
        with ctx.pose({rear_hinge: rear_limits.upper}):
            stored_front = ctx.part_element_world_aabb(deck, elem="front_beam")
        ctx.check(
            "deck rotates upward on the rear storage hinge",
            rest_front is not None
            and stored_front is not None
            and stored_front[1][2] > rest_front[1][2] + 1.2,
            details=f"rest_front={rest_front}, stored_front={stored_front}",
        )

    lip_limits = lip_hinge.motion_limits
    if lip_limits is not None and lip_limits.upper is not None:
        rest_lip = ctx.part_element_world_aabb(lip, elem="lip_plate")
        with ctx.pose({lip_hinge: lip_limits.upper}):
            dropped_lip = ctx.part_element_world_aabb(lip, elem="lip_plate")
        ctx.check(
            "lip drops below the deck when opened",
            rest_lip is not None
            and dropped_lip is not None
            and dropped_lip[0][2] < rest_lip[0][2] - 0.10,
            details=f"rest_lip={rest_lip}, dropped_lip={dropped_lip}",
        )

    leg_limits = leg_pair_0_hinge.motion_limits
    if leg_limits is not None and leg_limits.upper is not None:
        rest_foot_0 = ctx.part_element_world_aabb(leg_pair_0, elem="foot_bar")
        rest_foot_1 = ctx.part_element_world_aabb(leg_pair_1, elem="foot_bar")
        deck_plate = ctx.part_element_world_aabb(deck, elem="deck_plate")
        with ctx.pose({leg_pair_0_hinge: leg_limits.upper, leg_pair_1_hinge: leg_limits.upper}):
            folded_foot_0 = ctx.part_element_world_aabb(leg_pair_0, elem="foot_bar")
            folded_foot_1 = ctx.part_element_world_aabb(leg_pair_1, elem="foot_bar")
        ctx.check(
            "support legs hang well below the deck in the deployed pose",
            deck_plate is not None
            and rest_foot_0 is not None
            and rest_foot_1 is not None
            and rest_foot_0[1][2] < deck_plate[0][2] - 0.90
            and rest_foot_1[1][2] < deck_plate[0][2] - 0.90,
            details=f"deck={deck_plate}, rest_foot_0={rest_foot_0}, rest_foot_1={rest_foot_1}",
        )
        ctx.check(
            "support legs fold upward under the deck",
            rest_foot_0 is not None
            and rest_foot_1 is not None
            and folded_foot_0 is not None
            and folded_foot_1 is not None
            and folded_foot_0[1][2] > rest_foot_0[1][2] + 0.45
            and folded_foot_1[1][2] > rest_foot_1[1][2] + 0.45,
            details=(
                f"rest_foot_0={rest_foot_0}, folded_foot_0={folded_foot_0}, "
                f"rest_foot_1={rest_foot_1}, folded_foot_1={folded_foot_1}"
            ),
        )

    return ctx.report()


object_model = build_object_model()
