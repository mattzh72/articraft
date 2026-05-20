from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)


DECK_LENGTH = 1.85
DECK_WIDTH = 2.10
DECK_THICKNESS = 0.05
LIP_LENGTH = 0.42
LIP_REAR_THICKNESS = 0.045
LIP_NOSE_THICKNESS = 0.012


def _rect_section_x(
    x_pos: float,
    width: float,
    z_bottom: float,
    z_top: float,
) -> list[tuple[float, float, float]]:
    half_width = width * 0.5
    return [
        (x_pos, -half_width, z_bottom),
        (x_pos, half_width, z_bottom),
        (x_pos, half_width, z_top),
        (x_pos, -half_width, z_top),
    ]


def _lip_mesh():
    lip_geom = section_loft(
        [
            _rect_section_x(0.0, DECK_WIDTH, -LIP_REAR_THICKNESS, 0.0),
            _rect_section_x(0.30, DECK_WIDTH, -LIP_REAR_THICKNESS, 0.0),
            _rect_section_x(LIP_LENGTH, DECK_WIDTH, -LIP_NOSE_THICKNESS, 0.0),
        ]
    )
    return mesh_from_geometry(lip_geom, "dock_leveler_lip")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="edge_of_dock_lip_plate_leveler")

    dark_steel = model.material("dark_steel", rgba=(0.24, 0.26, 0.29, 1.0))
    weathered_steel = model.material("weathered_steel", rgba=(0.33, 0.35, 0.37, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.84, 0.69, 0.18, 1.0))

    dock_frame = model.part("dock_frame")
    dock_frame.visual(
        Box((0.26, 2.22, 0.22)),
        origin=Origin(xyz=(-0.13, 0.0, -0.11)),
        material=dark_steel,
        name="mount_beam",
    )
    dock_frame.visual(
        Box((0.08, 2.22, 0.08)),
        origin=Origin(xyz=(-0.04, 0.0, -0.26)),
        material=dark_steel,
        name="mount_drop_plate",
    )
    dock_frame.inertial = Inertial.from_geometry(
        Box((0.26, 2.22, 0.30)),
        mass=180.0,
        origin=Origin(xyz=(-0.13, 0.0, -0.15)),
    )

    deck = model.part("deck")
    deck.visual(
        Box((DECK_LENGTH, DECK_WIDTH, DECK_THICKNESS)),
        origin=Origin(xyz=(DECK_LENGTH * 0.5, 0.0, -DECK_THICKNESS * 0.5)),
        material=weathered_steel,
        name="deck_plate",
    )
    deck.visual(
        Box((1.48, 0.11, 0.11)),
        origin=Origin(xyz=(0.92, -0.64, -0.105)),
        material=weathered_steel,
        name="left_stiffener",
    )
    deck.visual(
        Box((1.52, 0.12, 0.12)),
        origin=Origin(xyz=(0.95, 0.0, -0.11)),
        material=weathered_steel,
        name="center_stiffener",
    )
    deck.visual(
        Box((1.48, 0.11, 0.11)),
        origin=Origin(xyz=(0.92, 0.64, -0.105)),
        material=weathered_steel,
        name="right_stiffener",
    )
    deck.inertial = Inertial.from_geometry(
        Box((DECK_LENGTH, DECK_WIDTH, 0.18)),
        mass=420.0,
        origin=Origin(xyz=(DECK_LENGTH * 0.5, 0.0, -0.09)),
    )

    lip = model.part("lip")
    lip.visual(
        _lip_mesh(),
        material=safety_yellow,
        name="lip_shell",
    )
    lip.inertial = Inertial.from_geometry(
        Box((LIP_LENGTH, DECK_WIDTH, LIP_REAR_THICKNESS)),
        mass=95.0,
        origin=Origin(xyz=(LIP_LENGTH * 0.5, 0.0, -LIP_REAR_THICKNESS * 0.5)),
    )

    model.articulation(
        "dock_to_deck",
        ArticulationType.REVOLUTE,
        parent=dock_frame,
        child=deck,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25000.0,
            velocity=0.6,
            lower=-0.26,
            upper=0.38,
        ),
    )
    model.articulation(
        "deck_to_lip",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=lip,
        origin=Origin(xyz=(DECK_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=9000.0,
            velocity=1.0,
            lower=-0.18,
            upper=0.70,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    dock_frame = object_model.get_part("dock_frame")
    deck = object_model.get_part("deck")
    lip = object_model.get_part("lip")
    deck_hinge = object_model.get_articulation("dock_to_deck")
    lip_hinge = object_model.get_articulation("deck_to_lip")

    ctx.check(
        "deck hinge pitches upward from dock edge",
        deck_hinge.axis == (0.0, -1.0, 0.0),
        details=f"axis={deck_hinge.axis}",
    )
    ctx.check(
        "lip hinge deploys downward from deck front",
        lip_hinge.axis == (0.0, 1.0, 0.0),
        details=f"axis={lip_hinge.axis}",
    )

    with ctx.pose({deck_hinge: 0.0, lip_hinge: 0.0}):
        ctx.expect_gap(
            deck,
            dock_frame,
            axis="x",
            positive_elem="deck_plate",
            negative_elem="mount_beam",
            max_gap=0.001,
            max_penetration=0.0,
            name="deck seats at the dock-edge hinge line",
        )
        ctx.expect_gap(
            lip,
            deck,
            axis="x",
            positive_elem="lip_shell",
            negative_elem="deck_plate",
            max_gap=0.001,
            max_penetration=0.0,
            name="lip seats against the deck front hinge line",
        )
        ctx.expect_within(
            lip,
            deck,
            axes="y",
            inner_elem="lip_shell",
            outer_elem="deck_plate",
            margin=0.0,
            name="lip stays within deck width",
        )

    rest_deck_aabb = ctx.part_world_aabb(deck)
    with ctx.pose({deck_hinge: deck_hinge.motion_limits.upper, lip_hinge: 0.0}):
        raised_deck_aabb = ctx.part_world_aabb(deck)
    ctx.check(
        "deck front edge raises above rest pose",
        rest_deck_aabb is not None
        and raised_deck_aabb is not None
        and raised_deck_aabb[1][2] > rest_deck_aabb[1][2] + 0.18,
        details=f"rest={rest_deck_aabb}, raised={raised_deck_aabb}",
    )

    rest_lip_aabb = ctx.part_world_aabb(lip)
    with ctx.pose({deck_hinge: 0.0, lip_hinge: lip_hinge.motion_limits.upper}):
        deployed_lip_aabb = ctx.part_world_aabb(lip)
        ctx.expect_overlap(
            lip,
            deck,
            axes="y",
            elem_a="lip_shell",
            elem_b="deck_plate",
            min_overlap=1.90,
            name="lip hinge stays aligned across the full deck width",
        )
    ctx.check(
        "lip swings downward to bridge toward a truck bed",
        rest_lip_aabb is not None
        and deployed_lip_aabb is not None
        and deployed_lip_aabb[0][2] < rest_lip_aabb[0][2] - 0.12,
        details=f"rest={rest_lip_aabb}, deployed={deployed_lip_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
