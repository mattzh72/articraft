from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="acrylic_clapperboard")

    acrylic = model.material("acrylic", rgba=(0.86, 0.95, 1.0, 0.24))
    print_white = model.material("print_white", rgba=(0.96, 0.96, 0.96, 0.96))
    matte_black = model.material("matte_black", rgba=(0.08, 0.08, 0.09, 1.0))
    paint_white = model.material("paint_white", rgba=(0.98, 0.98, 0.98, 1.0))
    steel = model.material("steel", rgba=(0.66, 0.68, 0.72, 1.0))

    board_width = 0.280
    board_height = 0.210
    board_thickness = 0.004

    beam_width = 0.278
    beam_depth = 0.012
    beam_height = 0.010
    hinge_axis_y = 0.0
    hinge_axis_z = 0.220

    clap_width = 0.292
    clap_thickness = 0.008
    clap_height = 0.038

    board = model.part("board")
    board.visual(
        Box((board_width, board_thickness, board_height)),
        origin=Origin(xyz=(0.0, 0.0, board_height / 2.0)),
        material=acrylic,
        name="board_panel",
    )

    print_y = board_thickness / 2.0 - 0.0002
    board.visual(
        Box((0.220, 0.0006, 0.032)),
        origin=Origin(xyz=(0.0, print_y, 0.048)),
        material=print_white,
        name="title_band",
    )
    for idx, z_pos in enumerate((0.088, 0.118, 0.148), start=1):
        board.visual(
            Box((0.224, 0.0006, 0.0012)),
            origin=Origin(xyz=(0.0, print_y, z_pos)),
            material=print_white,
            name=f"rule_{idx}",
        )
    for idx, x_pos in enumerate((-0.060, 0.060), start=1):
        board.visual(
            Box((0.0012, 0.0006, 0.062)),
            origin=Origin(xyz=(x_pos, print_y, 0.059)),
            material=print_white,
            name=f"divider_{idx}",
        )

    hinge_beam = model.part("hinge_beam")
    hinge_beam.visual(
        Box((beam_width, beam_depth, beam_height)),
        origin=Origin(xyz=(0.0, -0.006, -0.005)),
        material=steel,
        name="beam_body",
    )

    clapstick = model.part("clapstick")
    clapstick.visual(
        Box((clap_width, clap_thickness, clap_height)),
        origin=Origin(xyz=(0.0, clap_thickness / 2.0, clap_height / 2.0)),
        material=matte_black,
        name="clap_body",
    )

    stripe_angle = 0.58
    stripe_size = (0.044, 0.0010, 0.010)
    stripe_positions = (-0.110, -0.054, 0.002, 0.058, 0.114)
    for idx, x_pos in enumerate(stripe_positions, start=1):
        clapstick.visual(
            Box(stripe_size),
            origin=Origin(xyz=(x_pos, clap_thickness - 0.0005, 0.019), rpy=(0.0, stripe_angle, 0.0)),
            material=paint_white,
            name=f"stripe_{idx}",
        )

    model.articulation(
        "board_to_hinge_beam",
        ArticulationType.FIXED,
        parent=board,
        child=hinge_beam,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
    )
    model.articulation(
        "hinge_beam_to_clapstick",
        ArticulationType.REVOLUTE,
        parent=hinge_beam,
        child=clapstick,
        origin=Origin(),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=0.0, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    board = object_model.get_part("board")
    hinge_beam = object_model.get_part("hinge_beam")
    clapstick = object_model.get_part("clapstick")
    hinge = object_model.get_articulation("hinge_beam_to_clapstick")

    ctx.expect_overlap(
        board,
        hinge_beam,
        axes="x",
        min_overlap=0.270,
        elem_a="board_panel",
        elem_b="beam_body",
        name="hinge beam bridges the board width",
    )
    ctx.expect_gap(
        hinge_beam,
        board,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="beam_body",
        negative_elem="board_panel",
        name="hinge beam seats on the board top edge",
    )
    ctx.expect_gap(
        clapstick,
        hinge_beam,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="clap_body",
        negative_elem="beam_body",
        name="closed clapstick sits just above the hinge beam",
    )

    limits = hinge.motion_limits
    if limits is not None and limits.upper is not None:
        rest_aabb = ctx.part_element_world_aabb(clapstick, elem="clap_body")
        with ctx.pose({hinge: limits.upper}):
            open_aabb = ctx.part_element_world_aabb(clapstick, elem="clap_body")

        rest_center = None
        open_center = None
        if rest_aabb is not None:
            rest_center = tuple((lo + hi) / 2.0 for lo, hi in zip(rest_aabb[0], rest_aabb[1]))
        if open_aabb is not None:
            open_center = tuple((lo + hi) / 2.0 for lo, hi in zip(open_aabb[0], open_aabb[1]))

        ctx.check(
            "clapstick swings forward from the beam",
            rest_center is not None
            and open_center is not None
            and open_center[1] > rest_center[1] + 0.012,
            details=f"rest_center={rest_center}, open_center={open_center}",
        )

    return ctx.report()


object_model = build_object_model()
