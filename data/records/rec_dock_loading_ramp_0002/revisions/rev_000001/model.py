from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os

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

_ORIGINAL_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _ORIGINAL_GETCWD()
    except FileNotFoundError:
        try:
            os.chdir("/")
        except OSError:
            pass
        return "/"


os.getcwd = _safe_getcwd


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hydraulic_dock_leveler")

    steel = model.material("steel", rgba=(0.43, 0.45, 0.48, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.24, 0.26, 0.29, 1.0))
    pin_steel = model.material("pin_steel", rgba=(0.63, 0.65, 0.68, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.86, 0.73, 0.12, 1.0))

    rear_hinge_x = 0.03
    rear_hinge_z = 0.020
    front_hinge_x = 2.00
    front_hinge_z = 0.024

    dock_frame = model.part("dock_frame")
    dock_frame.visual(
        Box((0.26, 2.32, 0.18)),
        origin=Origin(xyz=(-0.13, 0.0, -0.05)),
        material=dark_steel,
        name="rear_header",
    )
    dock_frame.visual(
        Box((2.06, 0.12, 0.26)),
        origin=Origin(xyz=(1.03, 1.11, -0.09)),
        material=dark_steel,
        name="left_wall",
    )
    dock_frame.visual(
        Box((2.06, 0.12, 0.26)),
        origin=Origin(xyz=(1.03, -1.11, -0.09)),
        material=dark_steel,
        name="right_wall",
    )
    dock_frame.visual(
        Box((2.06, 2.10, 0.03)),
        origin=Origin(xyz=(1.03, 0.0, -0.235)),
        material=dark_steel,
        name="pit_floor",
    )
    dock_frame.visual(
        Box((0.06, 0.10, 0.10)),
        origin=Origin(xyz=(rear_hinge_x, 1.03, 0.0)),
        material=dark_steel,
        name="left_hinge_lug",
    )
    dock_frame.visual(
        Box((0.06, 0.10, 0.10)),
        origin=Origin(xyz=(rear_hinge_x, -1.03, 0.0)),
        material=dark_steel,
        name="right_hinge_lug",
    )
    dock_frame.visual(
        Cylinder(radius=0.010, length=2.08),
        origin=Origin(
            xyz=(rear_hinge_x, 0.0, rear_hinge_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=pin_steel,
        name="rear_hinge_pin",
    )

    deck = model.part("deck")
    deck.visual(
        Cylinder(radius=0.018, length=1.96),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="rear_hinge_barrel",
    )
    deck.visual(
        Box((1.94, 2.04, 0.012)),
        origin=Origin(xyz=(1.03, 0.0, 0.040)),
        material=steel,
        name="deck_plate",
    )
    deck.visual(
        Box((0.14, 1.92, 0.126)),
        origin=Origin(xyz=(1.93, 0.0, -0.053)),
        material=dark_steel,
        name="front_beam",
    )
    deck.visual(
        Box((0.14, 1.92, 0.12)),
        origin=Origin(xyz=(0.11, 0.0, -0.110)),
        material=dark_steel,
        name="rear_beam",
    )
    deck.visual(
        Box((0.07, 1.88, 0.050)),
        origin=Origin(xyz=(0.045, 0.0, -0.005)),
        material=dark_steel,
        name="rear_hinge_web",
    )
    deck.visual(
        Box((1.88, 0.08, 0.13)),
        origin=Origin(xyz=(1.02, 0.98, -0.030)),
        material=dark_steel,
        name="left_side_beam",
    )
    deck.visual(
        Box((1.88, 0.08, 0.13)),
        origin=Origin(xyz=(1.02, -0.98, -0.030)),
        material=dark_steel,
        name="right_side_beam",
    )
    deck.visual(
        Box((1.80, 0.09, 0.12)),
        origin=Origin(xyz=(1.00, 0.48, -0.025)),
        material=dark_steel,
        name="left_rib",
    )
    deck.visual(
        Box((1.80, 0.09, 0.12)),
        origin=Origin(xyz=(1.00, 0.0, -0.025)),
        material=dark_steel,
        name="center_rib",
    )
    deck.visual(
        Box((1.80, 0.09, 0.12)),
        origin=Origin(xyz=(1.00, -0.48, -0.025)),
        material=dark_steel,
        name="right_rib",
    )
    deck.visual(
        Cylinder(radius=0.017, length=1.84),
        origin=Origin(
            xyz=(front_hinge_x, 0.0, front_hinge_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=pin_steel,
        name="front_hinge_barrel",
    )
    deck.visual(
        Box((1.82, 0.07, 0.0015)),
        origin=Origin(xyz=(1.00, 0.95, 0.04625)),
        material=safety_yellow,
        name="left_safety_stripe",
    )
    deck.visual(
        Box((1.82, 0.07, 0.0015)),
        origin=Origin(xyz=(1.00, -0.95, 0.04625)),
        material=safety_yellow,
        name="right_safety_stripe",
    )

    lip = model.part("lip")
    lip.visual(
        Cylinder(radius=0.010, length=1.96),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pin_steel,
        name="lip_hinge_pin",
    )
    lip.visual(
        Box((0.38, 1.96, 0.012)),
        origin=Origin(xyz=(0.200, 0.0, 0.030)),
        material=steel,
        name="lip_plate",
    )
    lip.visual(
        Box((0.34, 1.92, 0.016)),
        origin=Origin(xyz=(0.200, 0.0, 0.018)),
        material=dark_steel,
        name="lip_underframe",
    )
    lip.visual(
        Box((0.12, 0.03, 0.030)),
        origin=Origin(xyz=(0.160, 0.945, 0.009)),
        material=dark_steel,
        name="left_lip_rib",
    )
    lip.visual(
        Box((0.12, 0.03, 0.030)),
        origin=Origin(xyz=(0.160, -0.945, 0.009)),
        material=dark_steel,
        name="right_lip_rib",
    )
    lip.visual(
        Box((0.10, 0.02, 0.028)),
        origin=Origin(xyz=(0.065, 0.970, 0.010)),
        material=dark_steel,
        name="left_pin_ear",
    )
    lip.visual(
        Box((0.10, 0.02, 0.028)),
        origin=Origin(xyz=(0.065, -0.970, 0.010)),
        material=dark_steel,
        name="right_pin_ear",
    )
    lip.visual(
        Box((0.06, 1.96, 0.020)),
        origin=Origin(xyz=(0.380, 0.0, 0.024)),
        material=steel,
        name="lip_nose",
    )
    lip.visual(
        Box((0.24, 0.06, 0.0015)),
        origin=Origin(xyz=(0.18, 0.93, 0.03575)),
        material=safety_yellow,
        name="left_lip_stripe",
    )
    lip.visual(
        Box((0.24, 0.06, 0.0015)),
        origin=Origin(xyz=(0.18, -0.93, 0.03575)),
        material=safety_yellow,
        name="right_lip_stripe",
    )

    model.articulation(
        "rear_hinge",
        ArticulationType.REVOLUTE,
        parent=dock_frame,
        child=deck,
        origin=Origin(xyz=(rear_hinge_x, 0.0, rear_hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35000.0,
            velocity=0.8,
            lower=0.0,
            upper=0.55,
        ),
    )
    model.articulation(
        "lip_hinge",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=lip,
        origin=Origin(xyz=(front_hinge_x, 0.0, front_hinge_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12000.0,
            velocity=1.2,
            lower=-0.10,
            upper=1.25,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    dock_frame = object_model.get_part("dock_frame")
    deck = object_model.get_part("deck")
    lip = object_model.get_part("lip")
    rear_hinge = object_model.get_articulation("rear_hinge")
    lip_hinge = object_model.get_articulation("lip_hinge")

    rear_header = dock_frame.get_visual("rear_header")
    left_wall = dock_frame.get_visual("left_wall")
    right_wall = dock_frame.get_visual("right_wall")
    pit_floor = dock_frame.get_visual("pit_floor")
    rear_hinge_pin = dock_frame.get_visual("rear_hinge_pin")
    rear_hinge_barrel = deck.get_visual("rear_hinge_barrel")
    deck_plate = deck.get_visual("deck_plate")
    front_beam = deck.get_visual("front_beam")
    left_side_beam = deck.get_visual("left_side_beam")
    right_side_beam = deck.get_visual("right_side_beam")
    center_rib = deck.get_visual("center_rib")
    front_hinge_barrel = deck.get_visual("front_hinge_barrel")
    lip_plate = lip.get_visual("lip_plate")
    lip_hinge_pin = lip.get_visual("lip_hinge_pin")
    lip_nose = lip.get_visual("lip_nose")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.allow_overlap(
        dock_frame,
        deck,
        elem_a="rear_hinge_pin",
        elem_b="rear_hinge_barrel",
        reason="rear hinge pin is intentionally captured inside the deck hinge barrel",
    )
    ctx.allow_overlap(
        deck,
        lip,
        elem_a="front_hinge_barrel",
        elem_b="lip_hinge_pin",
        reason="front lip pin is intentionally captured inside the deck front barrel",
    )
    ctx.fail_if_isolated_parts()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_within(
        dock_frame,
        deck,
        axes="xz",
        inner_elem=rear_hinge_pin,
        outer_elem=rear_hinge_barrel,
        name="rear hinge pin stays nested inside the rear barrel",
    )
    ctx.expect_contact(
        dock_frame,
        deck,
        elem_a=rear_hinge_pin,
        elem_b=rear_hinge_barrel,
        name="rear hinge remains in contact",
    )
    ctx.expect_gap(
        deck,
        dock_frame,
        axis="y",
        min_gap=0.02,
        max_gap=0.04,
        positive_elem="right_side_beam",
        negative_elem="right_wall",
        name="right side beam stays clear of the right pit wall",
    )
    ctx.expect_gap(
        dock_frame,
        deck,
        axis="y",
        min_gap=0.02,
        max_gap=0.04,
        positive_elem="left_wall",
        negative_elem="left_side_beam",
        name="left side beam stays clear of the left pit wall",
    )
    ctx.expect_gap(
        deck,
        dock_frame,
        axis="z",
        min_gap=0.12,
        positive_elem=center_rib,
        negative_elem=pit_floor,
        name="underside ribs clear the pit floor",
    )
    ctx.expect_gap(
        lip,
        deck,
        axis="x",
        min_gap=0.01,
        max_gap=0.02,
        positive_elem=lip_plate,
        negative_elem=front_beam,
        name="lip leaf seats just beyond the deck front beam",
    )
    ctx.expect_within(
        lip,
        deck,
        axes="xz",
        inner_elem=lip_hinge_pin,
        outer_elem=front_hinge_barrel,
        name="lip hinge pin stays nested inside the front barrel",
    )
    ctx.expect_contact(
        deck,
        lip,
        elem_a=front_hinge_barrel,
        elem_b=lip_hinge_pin,
        name="front hinge remains in contact",
    )
    ctx.expect_overlap(
        lip,
        deck,
        axes="y",
        min_overlap=1.70,
        elem_a=lip_plate,
        elem_b=deck_plate,
        name="lip spans most of the deck width",
    )

    rear_limits = rear_hinge.motion_limits
    if rear_limits is not None and rear_limits.lower is not None and rear_limits.upper is not None:
        with ctx.pose({rear_hinge: rear_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="rear_hinge_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="rear_hinge_lower_no_floating")
        with ctx.pose({rear_hinge: rear_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="rear_hinge_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="rear_hinge_upper_no_floating")

    lip_limits = lip_hinge.motion_limits
    if lip_limits is not None and lip_limits.lower is not None and lip_limits.upper is not None:
        with ctx.pose({lip_hinge: lip_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="lip_hinge_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="lip_hinge_lower_no_floating")
        with ctx.pose({lip_hinge: lip_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="lip_hinge_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="lip_hinge_upper_no_floating")

    with ctx.pose({rear_hinge: 0.52}):
        ctx.expect_gap(
            deck,
            dock_frame,
            axis="z",
            min_gap=0.80,
            positive_elem=front_beam,
            negative_elem=rear_header,
            name="raised deck front lifts well above the rear header",
        )
        ctx.expect_contact(
            dock_frame,
            deck,
            elem_a=rear_hinge_pin,
            elem_b=rear_hinge_barrel,
            name="rear hinge stays engaged while the deck tilts",
        )

    with ctx.pose({lip_hinge: 1.20}):
        ctx.expect_gap(
            deck,
            lip,
            axis="z",
            min_gap=0.30,
            positive_elem=deck_plate,
            negative_elem=lip_nose,
            name="deployed lip hangs below the deck surface",
        )
        ctx.expect_gap(
            lip,
            deck,
            axis="x",
            min_gap=0.10,
            positive_elem=lip_nose,
            negative_elem=deck_plate,
            name="deployed lip projects past the deck front edge",
        )

    with ctx.pose({rear_hinge: 0.40, lip_hinge: 1.05}):
        ctx.expect_contact(
            deck,
            lip,
            elem_a=front_hinge_barrel,
            elem_b=lip_hinge_pin,
            name="front hinge stays engaged in a raised deployed pose",
        )
        ctx.expect_gap(
            deck,
            lip,
            axis="z",
            min_gap=0.001,
            positive_elem=front_beam,
            negative_elem=lip_nose,
            name="raised deck still carries a hanging lip below the deck plane",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
