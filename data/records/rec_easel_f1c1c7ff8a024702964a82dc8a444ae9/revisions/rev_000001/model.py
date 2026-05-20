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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _add_leg_set(
    model: ArticulatedObject,
    *,
    name: str,
    board_name: str,
    tray_name: str,
    board_face_sign: float,
    wood,
    chalkboard,
) -> object:
    leg_set = model.part(name)

    leg_set.visual(
        Box((0.58, 0.03, 0.06)),
        origin=Origin(xyz=(0.0, 0.018 * board_face_sign, -0.0415)),
        material=wood,
        name="top_rail",
    )

    for side_x, side_name, leg_tilt in (
        (-0.27, "left", 0.075),
        (0.27, "right", -0.075),
    ):
        leg_set.visual(
            Box((0.04, 0.028, 0.96)),
            origin=Origin(xyz=(side_x, 0.0, -0.54), rpy=(0.0, leg_tilt, 0.0)),
            material=wood,
            name=f"{side_name}_leg",
        )
        leg_set.visual(
            Box((0.035, 0.024, 0.60)),
            origin=Origin(xyz=(side_x * 0.83, 0.0, -0.39), rpy=(0.0, leg_tilt * 0.45, 0.0)),
            material=wood,
            name=f"{side_name}_board_rail",
        )

    leg_set.visual(
        Box((0.47, 0.024, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, -0.11)),
        material=wood,
        name="board_top_rail",
    )

    leg_set.visual(
        Box((0.45, 0.006, 0.52)),
        origin=Origin(xyz=(0.0, 0.009 * board_face_sign, -0.38)),
        material=chalkboard,
        name="front_chalkboard" if board_name == "front_chalkboard" else "rear_chalkboard",
    )

    leg_set.visual(
        Box((0.56, 0.08, 0.026)),
        origin=Origin(xyz=(0.0, 0.033 * board_face_sign, -0.71)),
        material=wood,
        name=tray_name,
    )

    leg_set.visual(
        Box((0.64, 0.03, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, -0.93)),
        material=wood,
        name="bottom_stretcher",
    )

    return leg_set


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="childrens_chalkboard_easel")

    wood = model.material("birch_wood", rgba=(0.80, 0.69, 0.52, 1.0))
    chalkboard = model.material("chalkboard_green", rgba=(0.16, 0.23, 0.18, 1.0))
    plastic = model.material("powder_gray", rgba=(0.77, 0.79, 0.82, 1.0))
    paper = model.material("paper_roll", rgba=(0.96, 0.95, 0.90, 1.0))

    top_head = model.part("top_head")
    top_head.visual(
        Box((0.58, 0.025, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=plastic,
        name="holder_bridge",
    )
    top_head.visual(
        Box((0.05, 0.09, 0.08)),
        origin=Origin(xyz=(-0.29, 0.0, 0.0)),
        material=plastic,
        name="left_hinge_hub",
    )
    top_head.visual(
        Box((0.05, 0.09, 0.08)),
        origin=Origin(xyz=(0.29, 0.0, 0.0)),
        material=plastic,
        name="right_hinge_hub",
    )
    top_head.visual(
        Box((0.03, 0.012, 0.09)),
        origin=Origin(xyz=(-0.22, 0.0, 0.04)),
        material=plastic,
        name="left_roll_arm",
    )
    top_head.visual(
        Box((0.03, 0.012, 0.09)),
        origin=Origin(xyz=(0.22, 0.0, 0.04)),
        material=plastic,
        name="right_roll_arm",
    )
    top_head.visual(
        Cylinder(radius=0.012, length=0.66),
        origin=Origin(xyz=(0.0, 0.0, 0.075), rpy=(0.0, pi / 2.0, 0.0)),
        material=plastic,
        name="holder_bar",
    )
    top_head.visual(
        Cylinder(radius=0.044, length=0.46),
        origin=Origin(xyz=(0.0, 0.0, 0.075), rpy=(0.0, pi / 2.0, 0.0)),
        material=paper,
        name="paper_roll",
    )

    front_leg_set = _add_leg_set(
        model,
        name="front_leg_set",
        board_name="front_chalkboard",
        tray_name="front_tray",
        board_face_sign=1.0,
        wood=wood,
        chalkboard=chalkboard,
    )
    rear_leg_set = _add_leg_set(
        model,
        name="rear_leg_set",
        board_name="rear_chalkboard",
        tray_name="rear_tray",
        board_face_sign=-1.0,
        wood=wood,
        chalkboard=chalkboard,
    )

    spread_angle = 0.36

    model.articulation(
        "top_to_front_legs",
        ArticulationType.REVOLUTE,
        parent=top_head,
        child=front_leg_set,
        origin=Origin(xyz=(0.0, 0.0, -0.04), rpy=(spread_angle, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=-0.14,
            upper=0.24,
        ),
    )
    model.articulation(
        "top_to_rear_legs",
        ArticulationType.REVOLUTE,
        parent=top_head,
        child=rear_leg_set,
        origin=Origin(xyz=(0.0, 0.0, -0.04), rpy=(-spread_angle, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=-0.14,
            upper=0.24,
        ),
    )

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

    top_head = object_model.get_part("top_head")
    front_leg_set = object_model.get_part("front_leg_set")
    rear_leg_set = object_model.get_part("rear_leg_set")
    front_hinge = object_model.get_articulation("top_to_front_legs")
    rear_hinge = object_model.get_articulation("top_to_rear_legs")
    paper_roll = top_head.get_visual("paper_roll")
    front_board = front_leg_set.get_visual("front_chalkboard")
    rear_board = rear_leg_set.get_visual("rear_chalkboard")

    ctx.check(
        "front hinge opens around the width axis",
        front_hinge.axis == (1.0, 0.0, 0.0),
        details=f"axis={front_hinge.axis}",
    )
    ctx.check(
        "rear hinge opens around the width axis with mirrored sign",
        rear_hinge.axis == (-1.0, 0.0, 0.0),
        details=f"axis={rear_hinge.axis}",
    )

    with ctx.pose({front_hinge: 0.0, rear_hinge: 0.0}):
        ctx.expect_gap(
            front_leg_set,
            rear_leg_set,
            axis="y",
            min_gap=0.08,
            positive_elem=front_board,
            negative_elem=rear_board,
            name="front and rear chalkboards stay separated",
        )
        ctx.expect_overlap(
            front_leg_set,
            rear_leg_set,
            axes="xz",
            min_overlap=0.35,
            elem_a=front_board,
            elem_b=rear_board,
            name="front and rear chalkboards align as a double-sided easel",
        )

        rest_front_aabb = ctx.part_world_aabb(front_leg_set)
        rest_rear_aabb = ctx.part_world_aabb(rear_leg_set)

    with ctx.pose({front_hinge: 0.18, rear_hinge: 0.18}):
        spread_front_aabb = ctx.part_world_aabb(front_leg_set)
        spread_rear_aabb = ctx.part_world_aabb(rear_leg_set)

    ctx.check(
        "front leg set swings farther forward when opened",
        rest_front_aabb is not None
        and spread_front_aabb is not None
        and spread_front_aabb[1][1] > rest_front_aabb[1][1] + 0.045,
        details=f"rest={rest_front_aabb}, opened={spread_front_aabb}",
    )
    ctx.check(
        "rear leg set swings farther rearward when opened",
        rest_rear_aabb is not None
        and spread_rear_aabb is not None
        and spread_rear_aabb[0][1] < rest_rear_aabb[0][1] - 0.045,
        details=f"rest={rest_rear_aabb}, opened={spread_rear_aabb}",
    )
    ctx.check(
        "paper roll holder stays centered over the easel",
        paper_roll.origin.xyz[2] > -0.01,
        details=f"paper_roll_origin={paper_roll.origin.xyz}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
