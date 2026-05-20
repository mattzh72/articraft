from __future__ import annotations

import math

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


def _add_slanted_member(
    part,
    *,
    name: str,
    x: float,
    bottom_y: float,
    bottom_z: float,
    top_y: float,
    top_z: float,
    thickness: float,
    material,
) -> None:
    """Add a rectangular timber whose local +Z axis runs from bottom to top."""

    dy = top_y - bottom_y
    dz = top_z - bottom_z
    length = math.sqrt(dy * dy + dz * dz)
    roll = math.atan2(-dy, dz)
    part.visual(
        Box((thickness, thickness, length)),
        origin=Origin(
            xyz=(x, (bottom_y + top_y) * 0.5, (bottom_z + top_z) * 0.5),
            rpy=(roll, 0.0, 0.0),
        ),
        material=material,
        name=name,
    )


def _shift_along_sloped_z(y: float, z: float, roll: float, offset: float) -> tuple[float, float]:
    """World-space YZ shift along a box's local sloped +Z direction."""

    return y + (-math.sin(roll)) * offset, z + math.cos(roll) * offset


def _build_leg_set(
    part,
    *,
    side: float,
    wood,
    board_mat,
    rubber,
) -> None:
    """Build one child A-frame set in its hinge-local frame.

    side=-1 is the front face, side=+1 is the rear face.
    """

    top_rail_y = side * 0.035
    top_rail_z = -0.020
    top_leg_y = side * 0.052
    bottom_y = side * 0.430
    bottom_z = -1.105

    # The timber just below the top hinge pin visually ties both legs into one
    # moving A-frame leaf while staying clear of the fixed paper-roll bracket.
    part.visual(
        Box((0.78, 0.045, 0.045)),
        origin=Origin(xyz=(0.0, top_rail_y, top_rail_z)),
        material=wood,
        name="top_rail",
    )

    for index, x in enumerate((-0.365, 0.365)):
        part.visual(
            Box((0.070, 0.050, 0.070)),
            origin=Origin(xyz=(x, side * 0.052, -0.042)),
            material=wood,
            name=f"hinge_cheek_{index}",
        )
        _add_slanted_member(
            part,
            name=f"side_leg_{index}",
            x=x,
            bottom_y=bottom_y,
            bottom_z=bottom_z,
            top_y=top_leg_y,
            top_z=-0.052,
            thickness=0.042,
            material=wood,
        )
        part.visual(
            Box((0.105, 0.105, 0.030)),
            origin=Origin(xyz=(x, bottom_y, bottom_z - 0.010)),
            material=rubber,
            name=f"foot_pad_{index}",
        )

    # A lower stretcher keeps the two side legs from reading as separate islands.
    part.visual(
        Box((0.78, 0.040, 0.040)),
        origin=Origin(xyz=(0.0, side * 0.382, -0.965)),
        material=wood,
        name="lower_stretcher",
    )

    roll = math.atan2(-(top_leg_y - bottom_y), (-0.060 - bottom_z))
    board_y = side * 0.235
    board_z = -0.585

    part.visual(
        Box((0.655, 0.018, 0.560)),
        origin=Origin(xyz=(0.0, board_y, board_z), rpy=(roll, 0.0, 0.0)),
        material=board_mat,
        name="chalkboard_panel",
    )

    # Wood frame around the writing surface.  The side stiles overlap the legs
    # slightly, as a real nailed/glued frame would, so the whole leg set is one
    # supported assembly.
    for index, x in enumerate((-0.335, 0.335)):
        part.visual(
            Box((0.045, 0.044, 0.640)),
            origin=Origin(xyz=(x, board_y, board_z), rpy=(roll, 0.0, 0.0)),
            material=wood,
            name=f"board_stile_{index}",
        )

    top_y, top_z = _shift_along_sloped_z(board_y, board_z, roll, 0.305)
    bottom_frame_y, bottom_frame_z = _shift_along_sloped_z(board_y, board_z, roll, -0.305)
    part.visual(
        Box((0.710, 0.044, 0.048)),
        origin=Origin(xyz=(0.0, top_y, top_z), rpy=(roll, 0.0, 0.0)),
        material=wood,
        name="board_top_rail",
    )
    part.visual(
        Box((0.710, 0.044, 0.048)),
        origin=Origin(xyz=(0.0, bottom_frame_y, bottom_frame_z), rpy=(roll, 0.0, 0.0)),
        material=wood,
        name="board_bottom_rail",
    )

    # A shallow chalk tray projects from each writing side.
    part.visual(
        Box((0.730, 0.110, 0.035)),
        origin=Origin(xyz=(0.0, side * 0.395, -0.825)),
        material=wood,
        name="chalk_tray",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="childrens_double_sided_chalkboard_easel")

    honey_wood = model.material("honey_colored_wood", rgba=(0.86, 0.55, 0.25, 1.0))
    dark_green = model.material("dark_green_chalkboard", rgba=(0.03, 0.18, 0.12, 1.0))
    warm_white = model.material("warm_white_paper", rgba=(0.96, 0.94, 0.86, 1.0))
    grey_rubber = model.material("grey_rubber_feet", rgba=(0.08, 0.08, 0.08, 1.0))

    top_bar = model.part("top_bar")
    # Fixed hinge pin and paper-roll holder: this is the stationary root member.
    top_bar.visual(
        Cylinder(radius=0.025, length=0.96),
        origin=Origin(xyz=(0.0, 0.0, 1.150), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=honey_wood,
        name="hinge_pin",
    )
    top_bar.visual(
        Cylinder(radius=0.014, length=0.92),
        origin=Origin(xyz=(0.0, 0.0, 1.270), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=honey_wood,
        name="roll_dowel",
    )
    top_bar.visual(
        Cylinder(radius=0.065, length=0.62),
        origin=Origin(xyz=(0.0, 0.0, 1.270), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_white,
        name="paper_roll",
    )
    for index, x in enumerate((-0.435, 0.435)):
        top_bar.visual(
            Box((0.050, 0.060, 0.190)),
            origin=Origin(xyz=(x, 0.0, 1.205)),
            material=honey_wood,
            name=f"roll_bracket_{index}",
        )
    top_bar.visual(
        Box((0.520, 0.006, 0.220)),
        origin=Origin(xyz=(0.0, -0.067, 1.155)),
        material=warm_white,
        name="paper_leader",
    )

    front_frame = model.part("front_frame")
    _build_leg_set(front_frame, side=-1.0, wood=honey_wood, board_mat=dark_green, rubber=grey_rubber)

    rear_frame = model.part("rear_frame")
    _build_leg_set(rear_frame, side=1.0, wood=honey_wood, board_mat=dark_green, rubber=grey_rubber)

    model.articulation(
        "front_hinge",
        ArticulationType.REVOLUTE,
        parent=top_bar,
        child=front_frame,
        origin=Origin(xyz=(0.0, 0.0, 1.150)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=-0.18, upper=0.22),
    )
    model.articulation(
        "rear_hinge",
        ArticulationType.REVOLUTE,
        parent=top_bar,
        child=rear_frame,
        origin=Origin(xyz=(0.0, 0.0, 1.150)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=-0.18, upper=0.22),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    top_bar = object_model.get_part("top_bar")
    front_frame = object_model.get_part("front_frame")
    rear_frame = object_model.get_part("rear_frame")
    front_hinge = object_model.get_articulation("front_hinge")
    rear_hinge = object_model.get_articulation("rear_hinge")

    ctx.allow_overlap(
        top_bar,
        front_frame,
        elem_a="hinge_pin",
        elem_b="top_rail",
        reason="The fixed top hinge pin is intentionally represented as captured through the front A-frame hinge leaf.",
    )
    ctx.allow_overlap(
        top_bar,
        rear_frame,
        elem_a="hinge_pin",
        elem_b="top_rail",
        reason="The fixed top hinge pin is intentionally represented as captured through the rear A-frame hinge leaf.",
    )

    ctx.check(
        "two moving A-frame leaves",
        front_hinge.articulation_type == ArticulationType.REVOLUTE
        and rear_hinge.articulation_type == ArticulationType.REVOLUTE,
        details="front and rear leg sets must be revolute-hinged at the top",
    )
    ctx.expect_overlap(
        front_frame,
        rear_frame,
        axes="x",
        min_overlap=0.60,
        name="front and rear easel faces share the same child-scale width",
    )
    ctx.expect_gap(
        rear_frame,
        front_frame,
        axis="y",
        min_gap=0.015,
        name="open A-frame leaves are separated front-to-back",
    )
    ctx.expect_overlap(
        top_bar,
        front_frame,
        axes="x",
        min_overlap=0.70,
        elem_a="hinge_pin",
        elem_b="top_rail",
        name="front top rail spans under fixed hinge pin",
    )
    ctx.expect_overlap(
        top_bar,
        rear_frame,
        axes="x",
        min_overlap=0.70,
        elem_a="hinge_pin",
        elem_b="top_rail",
        name="rear top rail spans under fixed hinge pin",
    )

    front_rest = ctx.part_world_aabb(front_frame)
    rear_rest = ctx.part_world_aabb(rear_frame)
    with ctx.pose({front_hinge: -0.15, rear_hinge: -0.15}):
        front_spread = ctx.part_world_aabb(front_frame)
        rear_spread = ctx.part_world_aabb(rear_frame)
    with ctx.pose({front_hinge: 0.18, rear_hinge: 0.18}):
        front_folded = ctx.part_world_aabb(front_frame)
        rear_folded = ctx.part_world_aabb(rear_frame)

    ctx.check(
        "hinges increase spread at lower travel",
        front_rest is not None
        and rear_rest is not None
        and front_spread is not None
        and rear_spread is not None
        and front_spread[0][1] < front_rest[0][1] - 0.04
        and rear_spread[1][1] > rear_rest[1][1] + 0.04,
        details=f"rest={front_rest},{rear_rest} spread={front_spread},{rear_spread}",
    )
    ctx.check(
        "hinges fold both leaves inward at upper travel",
        front_rest is not None
        and rear_rest is not None
        and front_folded is not None
        and rear_folded is not None
        and front_folded[0][1] > front_rest[0][1] + 0.04
        and rear_folded[1][1] < rear_rest[1][1] - 0.04,
        details=f"rest={front_rest},{rear_rest} folded={front_folded},{rear_folded}",
    )

    return ctx.report()


object_model = build_object_model()
