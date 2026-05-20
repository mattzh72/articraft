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


DECK_WIDTH = 2.10
DECK_LENGTH = 1.30
DECK_THICKNESS = 0.10
FACE_PLANE_Y = -0.35
REAR_HINGE_Y = FACE_PLANE_Y
REAR_HINGE_Z = 0.56

LIP_LENGTH = 0.38
LIP_THICKNESS = 0.05

SKIRT_HEIGHT = 0.24
SKIRT_THICKNESS = 0.025
SKIRT_LENGTH = 1.20


def _part_dims(aabb) -> tuple[float, float, float]:
    return (
        aabb[1][0] - aabb[0][0],
        aabb[1][1] - aabb[0][1],
        aabb[1][2] - aabb[0][2],
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vertical_storing_dock_leveler")

    concrete = model.material("concrete", rgba=(0.63, 0.64, 0.66, 1.0))
    steel = model.material("steel", rgba=(0.24, 0.26, 0.29, 1.0))
    lip_steel = model.material("lip_steel", rgba=(0.55, 0.58, 0.61, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.86, 0.72, 0.12, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    dock_frame = model.part("dock_frame")
    dock_frame.visual(
        Box((2.82, 0.10, 0.12)),
        origin=Origin(xyz=(0.0, FACE_PLANE_Y - 0.05, 0.38)),
        material=concrete,
        name="front_sill",
    )
    dock_frame.visual(
        Box((0.22, 0.10, 1.44)),
        origin=Origin(xyz=(-1.39, FACE_PLANE_Y - 0.05, 1.18)),
        material=concrete,
        name="left_face_post",
    )
    dock_frame.visual(
        Box((0.22, 0.10, 1.44)),
        origin=Origin(xyz=(1.39, FACE_PLANE_Y - 0.05, 1.18)),
        material=concrete,
        name="right_face_post",
    )
    dock_frame.visual(
        Box((3.00, 0.10, 0.18)),
        origin=Origin(xyz=(0.0, FACE_PLANE_Y - 0.05, 1.99)),
        material=concrete,
        name="top_header",
    )
    dock_frame.visual(
        Box((2.82, 0.56, 0.20)),
        origin=Origin(xyz=(0.0, -0.64, 0.10)),
        material=concrete,
        name="pit_floor",
    )
    dock_frame.visual(
        Box((0.18, 0.56, 0.56)),
        origin=Origin(xyz=(-1.31, -0.64, 0.28)),
        material=concrete,
        name="left_pit_wall",
    )
    dock_frame.visual(
        Box((0.18, 0.56, 0.56)),
        origin=Origin(xyz=(1.31, -0.64, 0.28)),
        material=concrete,
        name="right_pit_wall",
    )
    dock_frame.visual(
        Box((2.62, 0.12, 0.56)),
        origin=Origin(xyz=(0.0, -0.86, 0.28)),
        material=concrete,
        name="rear_pit_wall",
    )
    dock_frame.visual(
        Box((0.12, 0.12, 0.30)),
        origin=Origin(xyz=(-1.36, FACE_PLANE_Y - 0.01, 0.72)),
        material=rubber,
        name="left_bumper",
    )
    dock_frame.visual(
        Box((0.12, 0.12, 0.30)),
        origin=Origin(xyz=(1.36, FACE_PLANE_Y - 0.01, 0.72)),
        material=rubber,
        name="right_bumper",
    )
    dock_frame.visual(
        Box((0.12, 0.52, 0.10)),
        origin=Origin(xyz=(-0.90, -0.64, 0.48)),
        material=steel,
        name="left_hinge_bracket",
    )
    dock_frame.visual(
        Box((0.12, 0.52, 0.10)),
        origin=Origin(xyz=(0.90, -0.64, 0.48)),
        material=steel,
        name="right_hinge_bracket",
    )
    dock_frame.inertial = Inertial.from_geometry(
        Box((3.00, 0.96, 2.08)),
        mass=1600.0,
        origin=Origin(xyz=(0.0, -0.52, 0.96)),
    )

    deck = model.part("deck")
    deck.visual(
        Cylinder(radius=0.03, length=1.80),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="rear_hinge_tube",
    )
    deck.visual(
        Box((1.80, 0.04, 0.08)),
        origin=Origin(xyz=(0.0, 0.02, 0.04)),
        material=steel,
        name="rear_hinge_frame",
    )
    deck.visual(
        Box((DECK_WIDTH, DECK_THICKNESS, DECK_LENGTH)),
        origin=Origin(xyz=(0.0, DECK_THICKNESS / 2.0, DECK_LENGTH / 2.0)),
        material=steel,
        name="deck_plate",
    )
    deck.inertial = Inertial.from_geometry(
        Box((DECK_WIDTH, 0.12, DECK_LENGTH)),
        mass=430.0,
        origin=Origin(xyz=(0.0, 0.04, DECK_LENGTH / 2.0)),
    )

    lip = model.part("lip")
    lip.visual(
        Box((1.96, LIP_LENGTH, LIP_THICKNESS)),
        origin=Origin(xyz=(0.0, -LIP_LENGTH / 2.0, 0.0)),
        material=lip_steel,
        name="lip_plate",
    )
    lip.inertial = Inertial.from_geometry(
        Box((1.96, LIP_LENGTH, LIP_THICKNESS)),
        mass=78.0,
        origin=Origin(xyz=(0.0, -LIP_LENGTH / 2.0, 0.0)),
    )

    left_skirt = model.part("left_skirt")
    left_skirt.visual(
        Box((SKIRT_HEIGHT, SKIRT_THICKNESS, SKIRT_LENGTH)),
        origin=Origin(xyz=(-0.11, -SKIRT_THICKNESS / 2.0, 0.0)),
        material=safety_yellow,
        name="skirt_panel",
    )
    left_skirt.visual(
        Box((0.04, 0.04, 0.96)),
        origin=Origin(xyz=(-0.18, -0.045, 0.0)),
        material=steel,
        name="outer_flange",
    )
    left_skirt.inertial = Inertial.from_geometry(
        Box((SKIRT_HEIGHT, 0.08, SKIRT_LENGTH)),
        mass=18.0,
        origin=Origin(xyz=(-0.12, -0.04, 0.0)),
    )

    right_skirt = model.part("right_skirt")
    right_skirt.visual(
        Box((SKIRT_HEIGHT, SKIRT_THICKNESS, SKIRT_LENGTH)),
        origin=Origin(xyz=(0.11, -SKIRT_THICKNESS / 2.0, 0.0)),
        material=safety_yellow,
        name="skirt_panel",
    )
    right_skirt.visual(
        Box((0.04, 0.04, 0.96)),
        origin=Origin(xyz=(0.18, -0.045, 0.0)),
        material=steel,
        name="outer_flange",
    )
    right_skirt.inertial = Inertial.from_geometry(
        Box((SKIRT_HEIGHT, 0.08, SKIRT_LENGTH)),
        mass=18.0,
        origin=Origin(xyz=(0.12, -0.04, 0.0)),
    )

    model.articulation(
        "rear_hinge",
        ArticulationType.REVOLUTE,
        parent=dock_frame,
        child=deck,
        origin=Origin(xyz=(0.0, REAR_HINGE_Y, REAR_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=16000.0,
            velocity=0.55,
            lower=-math.pi / 2.0,
            upper=0.0,
        ),
    )
    model.articulation(
        "lip_hinge",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=lip,
        origin=Origin(xyz=(0.0, 0.0, DECK_LENGTH)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5000.0,
            velocity=1.3,
            lower=-math.pi / 2.0,
            upper=0.0,
        ),
    )
    model.articulation(
        "left_skirt_hinge",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=left_skirt,
        origin=Origin(xyz=(-DECK_WIDTH / 2.0, 0.0, DECK_LENGTH / 2.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=400.0,
            velocity=1.2,
            lower=-math.pi / 2.0,
            upper=0.0,
        ),
    )
    model.articulation(
        "right_skirt_hinge",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=right_skirt,
        origin=Origin(xyz=(DECK_WIDTH / 2.0, 0.0, DECK_LENGTH / 2.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=400.0,
            velocity=1.2,
            lower=-math.pi / 2.0,
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    dock_frame = object_model.get_part("dock_frame")
    deck = object_model.get_part("deck")
    lip = object_model.get_part("lip")
    left_skirt = object_model.get_part("left_skirt")
    right_skirt = object_model.get_part("right_skirt")

    front_sill = dock_frame.get_visual("front_sill")
    left_face_post = dock_frame.get_visual("left_face_post")
    right_face_post = dock_frame.get_visual("right_face_post")
    top_header = dock_frame.get_visual("top_header")
    pit_floor = dock_frame.get_visual("pit_floor")
    left_hinge_bracket = dock_frame.get_visual("left_hinge_bracket")
    right_hinge_bracket = dock_frame.get_visual("right_hinge_bracket")
    deck_plate = deck.get_visual("deck_plate")
    rear_hinge_tube = deck.get_visual("rear_hinge_tube")
    lip_plate = lip.get_visual("lip_plate")
    left_panel = left_skirt.get_visual("skirt_panel")
    right_panel = right_skirt.get_visual("skirt_panel")

    rear_hinge = object_model.get_articulation("rear_hinge")
    lip_hinge = object_model.get_articulation("lip_hinge")
    left_skirt_hinge = object_model.get_articulation("left_skirt_hinge")
    right_skirt_hinge = object_model.get_articulation("right_skirt_hinge")

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

    ctx.check("rear_hinge_axis", rear_hinge.axis == (1.0, 0.0, 0.0), details=str(rear_hinge.axis))
    ctx.check("lip_hinge_axis", lip_hinge.axis == (1.0, 0.0, 0.0), details=str(lip_hinge.axis))
    ctx.check(
        "left_skirt_hinge_axis",
        left_skirt_hinge.axis == (0.0, 0.0, 1.0),
        details=str(left_skirt_hinge.axis),
    )
    ctx.check(
        "right_skirt_hinge_axis",
        right_skirt_hinge.axis == (0.0, 0.0, -1.0),
        details=str(right_skirt_hinge.axis),
    )

    stowed_pose = {
        rear_hinge: 0.0,
        lip_hinge: 0.0,
        left_skirt_hinge: 0.0,
        right_skirt_hinge: 0.0,
    }
    with ctx.pose(stowed_pose):
        ctx.expect_overlap(
            deck,
            dock_frame,
            axes="x",
            elem_a=rear_hinge_tube,
            elem_b=left_hinge_bracket,
            min_overlap=0.05,
            name="left_hinge_bracket_captures_tube_stowed",
        )
        ctx.expect_overlap(
            deck,
            dock_frame,
            axes="x",
            elem_a=rear_hinge_tube,
            elem_b=right_hinge_bracket,
            min_overlap=0.05,
            name="right_hinge_bracket_captures_tube_stowed",
        )
        ctx.expect_gap(
            deck,
            dock_frame,
            axis="y",
            positive_elem=rear_hinge_tube,
            negative_elem=left_hinge_bracket,
            max_gap=0.0,
            max_penetration=0.0,
            name="left_hinge_tube_seats_in_bracket_stowed_y",
        )
        ctx.expect_gap(
            deck,
            dock_frame,
            axis="z",
            positive_elem=rear_hinge_tube,
            negative_elem=left_hinge_bracket,
            max_gap=0.0,
            max_penetration=0.0,
            name="left_hinge_tube_seats_in_bracket_stowed_z",
        )
        ctx.expect_contact(lip, deck, name="lip_stowed_connected_to_deck")
        ctx.expect_contact(left_skirt, deck, name="left_skirt_stowed_connected_to_deck")
        ctx.expect_contact(right_skirt, deck, name="right_skirt_stowed_connected_to_deck")
        ctx.expect_gap(
            deck,
            dock_frame,
            axis="y",
            positive_elem=deck_plate,
            negative_elem=front_sill,
            max_gap=0.002,
            max_penetration=0.0,
            name="deck_stows_flush_to_dock_face",
        )
        stowed_plate = ctx.part_element_world_aabb(deck, elem=deck_plate)
        left_post_aabb = ctx.part_element_world_aabb(dock_frame, elem=left_face_post)
        right_post_aabb = ctx.part_element_world_aabb(dock_frame, elem=right_face_post)
        header_aabb = ctx.part_element_world_aabb(dock_frame, elem=top_header)
        sill_aabb = ctx.part_element_world_aabb(dock_frame, elem=front_sill)
        assert stowed_plate is not None
        assert left_post_aabb is not None
        assert right_post_aabb is not None
        assert header_aabb is not None
        assert sill_aabb is not None
        stowed_dims = _part_dims(stowed_plate)
        ctx.check(
            "deck_reads_vertical_when_stowed",
            stowed_dims[2] > 1.20 and stowed_dims[1] < 0.12,
            details=f"deck plate dims in stowed pose were {stowed_dims}",
        )
        ctx.check(
            "deck_fits_opening_when_stowed",
            (
                stowed_plate[0][0] > left_post_aabb[1][0]
                and stowed_plate[1][0] < right_post_aabb[0][0]
                and stowed_plate[0][2] >= sill_aabb[1][2] - 0.002
                and stowed_plate[1][2] <= header_aabb[0][2] + 0.002
            ),
            details=(
                f"deck={stowed_plate}, left_post={left_post_aabb}, "
                f"right_post={right_post_aabb}, sill={sill_aabb}, header={header_aabb}"
            ),
        )

    deployment_pose = {
        rear_hinge: -math.pi / 2.0,
        lip_hinge: -math.pi / 2.0,
        left_skirt_hinge: -math.pi / 2.0,
        right_skirt_hinge: -math.pi / 2.0,
    }
    with ctx.pose(deployment_pose):
        ctx.fail_if_isolated_parts(name="deployed_pose_no_floating")
        ctx.fail_if_parts_overlap_in_current_pose(name="deployed_pose_no_overlap")
        ctx.expect_overlap(
            deck,
            dock_frame,
            axes="x",
            elem_a=rear_hinge_tube,
            elem_b=left_hinge_bracket,
            min_overlap=0.05,
            name="left_hinge_bracket_captures_tube_deployed",
        )
        ctx.expect_overlap(
            deck,
            dock_frame,
            axes="x",
            elem_a=rear_hinge_tube,
            elem_b=right_hinge_bracket,
            min_overlap=0.05,
            name="right_hinge_bracket_captures_tube_deployed",
        )
        ctx.expect_gap(
            deck,
            dock_frame,
            axis="y",
            positive_elem=rear_hinge_tube,
            negative_elem=left_hinge_bracket,
            max_gap=0.0,
            max_penetration=0.0,
            name="left_hinge_tube_seats_in_bracket_deployed_y",
        )
        ctx.expect_gap(
            deck,
            dock_frame,
            axis="z",
            positive_elem=rear_hinge_tube,
            negative_elem=left_hinge_bracket,
            max_gap=0.0,
            max_penetration=0.0,
            name="left_hinge_tube_seats_in_bracket_deployed_z",
        )
        ctx.expect_contact(lip, deck, name="lip_deployed_connected_to_deck")
        ctx.expect_contact(left_skirt, deck, name="left_skirt_deployed_connected_to_deck")
        ctx.expect_contact(right_skirt, deck, name="right_skirt_deployed_connected_to_deck")
        ctx.expect_gap(
            deck,
            dock_frame,
            axis="z",
            positive_elem=deck_plate,
            negative_elem=pit_floor,
            min_gap=0.25,
            name="deck_clears_pit_floor_when_deployed",
        )

        deployed_deck = ctx.part_element_world_aabb(deck, elem=deck_plate)
        deployed_lip = ctx.part_element_world_aabb(lip, elem=lip_plate)
        deployed_left_skirt = ctx.part_element_world_aabb(left_skirt, elem=left_panel)
        deployed_right_skirt = ctx.part_element_world_aabb(right_skirt, elem=right_panel)
        assert deployed_deck is not None
        assert deployed_lip is not None
        assert deployed_left_skirt is not None
        assert deployed_right_skirt is not None

        deployed_dims = _part_dims(deployed_deck)
        ctx.check(
            "deck_reads_horizontal_when_deployed",
            deployed_dims[1] > 1.20 and deployed_dims[2] < 0.12,
            details=f"deck plate dims in deployed pose were {deployed_dims}",
        )
        ctx.check(
            "lip_projects_beyond_front_edge",
            deployed_lip[1][1] > deployed_deck[1][1] + 0.18,
            details=(
                f"lip front y={deployed_lip[1][1]:.4f} "
                f"deck front y={deployed_deck[1][1]:.4f}"
            ),
        )
        ctx.check(
            "left_skirt_hangs_below_deck",
            deployed_left_skirt[0][2] < deployed_deck[0][2] - 0.10,
            details=(
                f"left skirt zmin={deployed_left_skirt[0][2]:.4f} "
                f"deck zmin={deployed_deck[0][2]:.4f}"
            ),
        )
        ctx.check(
            "right_skirt_hangs_below_deck",
            deployed_right_skirt[0][2] < deployed_deck[0][2] - 0.10,
            details=(
                f"right skirt zmin={deployed_right_skirt[0][2]:.4f} "
                f"deck zmin={deployed_deck[0][2]:.4f}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
