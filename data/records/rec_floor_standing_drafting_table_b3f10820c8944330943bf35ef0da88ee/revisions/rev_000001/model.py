from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="teaching_studio_drafting_table")

    dark_metal = Material("dark powder coated steel", color=(0.06, 0.065, 0.07, 1.0))
    satin_metal = Material("satin aluminum rail", color=(0.62, 0.64, 0.66, 1.0))
    black_rubber = Material("black ribbed rubber", color=(0.015, 0.015, 0.014, 1.0))
    board_face = Material("warm matte drafting board", color=(0.78, 0.69, 0.54, 1.0))
    board_edge = Material("sealed hardwood edge", color=(0.48, 0.30, 0.16, 1.0))
    graphite = Material("graphite hardware", color=(0.16, 0.16, 0.17, 1.0))

    frame = model.part("frame")

    # Low, flat H-base on the studio floor.
    frame.visual(
        Box((0.18, 1.25, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=dark_metal,
        name="base_spine",
    )
    frame.visual(
        Box((1.25, 0.22, 0.055)),
        origin=Origin(xyz=(0.0, -0.48, 0.0275)),
        material=dark_metal,
        name="front_foot",
    )
    frame.visual(
        Box((1.25, 0.18, 0.055)),
        origin=Origin(xyz=(0.0, 0.50, 0.0275)),
        material=dark_metal,
        name="rear_foot",
    )

    # A single central vertical rail with caps and low braces rising from the H-base.
    frame.visual(
        Box((0.12, 0.08, 1.30)),
        origin=Origin(xyz=(0.0, 0.10, 0.705)),
        material=satin_metal,
        name="vertical_rail",
    )
    frame.visual(
        Box((0.22, 0.16, 0.08)),
        origin=Origin(xyz=(0.0, 0.10, 0.095)),
        material=dark_metal,
        name="rail_foot_block",
    )
    frame.visual(
        Box((0.22, 0.14, 0.06)),
        origin=Origin(xyz=(0.0, 0.10, 1.385)),
        material=dark_metal,
        name="rail_top_cap",
    )
    frame.visual(
        Box((0.055, 0.26, 0.15)),
        origin=Origin(xyz=(-0.08, 0.20, 0.13)),
        material=dark_metal,
        name="rail_brace_0",
    )
    frame.visual(
        Box((0.055, 0.26, 0.15)),
        origin=Origin(xyz=(0.08, 0.20, 0.13)),
        material=dark_metal,
        name="rail_brace_1",
    )

    # Two base uprights carry the transverse lower foot-bar pivot.
    for suffix, x in (("0", -0.48), ("1", 0.48)):
        frame.visual(
            Box((0.085, 0.12, 0.34)),
            origin=Origin(xyz=(x, -0.42, 0.225)),
            material=dark_metal,
            name=f"base_upright_{suffix}",
        )
        frame.visual(
            Cylinder(radius=0.045, length=0.018),
            origin=Origin(xyz=(x, -0.42, 0.22), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=graphite,
            name=f"pivot_boss_{suffix}",
        )

    carriage = model.part("carriage")
    # The carriage frame origin is the board hinge line.  Its sleeve surrounds,
    # but does not intersect, the fixed vertical rail.
    carriage.visual(
        Box((0.76, 0.05, 0.26)),
        origin=Origin(xyz=(0.0, 0.195, 0.0)),
        material=dark_metal,
        name="sleeve_front",
    )
    carriage.visual(
        Box((0.76, 0.05, 0.26)),
        origin=Origin(xyz=(0.0, 0.325, 0.0)),
        material=dark_metal,
        name="sleeve_rear",
    )
    for suffix, x in (("0", -0.11), ("1", 0.11)):
        carriage.visual(
            Box((0.05, 0.20, 0.26)),
            origin=Origin(xyz=(x, 0.26, 0.0)),
            material=dark_metal,
            name=f"sleeve_side_{suffix}",
        )
    for suffix, x in (("0", -0.31), ("1", 0.31)):
        carriage.visual(
            Box((0.07, 0.20, 0.065)),
            origin=Origin(xyz=(x, 0.085, 0.0)),
            material=dark_metal,
            name=f"hinge_arm_{suffix}",
        )
        carriage.visual(
            Box((0.08, 0.10, 0.14)),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=dark_metal,
            name=f"hinge_lug_{suffix}",
        )
        carriage.visual(
            Cylinder(radius=0.025, length=0.09),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=graphite,
            name=f"lug_bushing_{suffix}",
        )

    board = model.part("board")
    board.visual(
        Box((1.05, 0.040, 0.68)),
        origin=Origin(xyz=(0.0, -0.075, -0.055)),
        material=board_face,
        name="board_panel",
    )
    board.visual(
        Box((1.10, 0.035, 0.042)),
        origin=Origin(xyz=(0.0, -0.108, 0.305)),
        material=board_edge,
        name="top_edge",
    )
    board.visual(
        Box((1.10, 0.040, 0.045)),
        origin=Origin(xyz=(0.0, -0.110, -0.415)),
        material=board_edge,
        name="bottom_edge",
    )
    for suffix, x in (("0", -0.545), ("1", 0.545)):
        board.visual(
            Box((0.040, 0.035, 0.72)),
            origin=Origin(xyz=(x, -0.108, -0.055)),
            material=board_edge,
            name=f"side_edge_{suffix}",
        )
    board.visual(
        Box((1.00, 0.12, 0.045)),
        origin=Origin(xyz=(0.0, -0.135, -0.440)),
        material=board_edge,
        name="bottom_ledge",
    )
    board.visual(
        Box((0.98, 0.025, 0.055)),
        origin=Origin(xyz=(0.0, -0.205, -0.405)),
        material=board_edge,
        name="ledge_lip",
    )
    board.visual(
        Cylinder(radius=0.025, length=0.53),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="hinge_knuckle",
    )
    for suffix, x in (("0", -0.16), ("1", 0.16)):
        board.visual(
            Box((0.07, 0.075, 0.05)),
            origin=Origin(xyz=(x, -0.035, 0.0)),
            material=graphite,
            name=f"hinge_strap_{suffix}",
        )

    foot_bar = model.part("foot_bar")
    foot_bar.visual(
        Cylinder(radius=0.025, length=0.875),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="pivot_tube",
    )
    for suffix, x in (("0", -0.30), ("1", 0.30)):
        foot_bar.visual(
            Box((0.050, 0.060, 0.135)),
            origin=Origin(xyz=(x, -0.050, -0.055)),
            material=dark_metal,
            name=f"swing_arm_{suffix}",
        )
    foot_bar.visual(
        Cylinder(radius=0.032, length=0.74),
        origin=Origin(xyz=(0.0, -0.100, -0.100), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_rubber,
        name="tread_bar",
    )

    model.articulation(
        "carriage_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, -0.16, 0.86)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=-0.24, upper=0.34),
    )
    model.articulation(
        "board_tilt",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=board,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.1, lower=-0.65, upper=0.85),
    )
    model.articulation(
        "foot_bar_pivot",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=foot_bar,
        origin=Origin(xyz=(0.0, -0.42, 0.22)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.5, lower=-0.45, upper=0.55),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    board = object_model.get_part("board")
    foot_bar = object_model.get_part("foot_bar")
    carriage_slide = object_model.get_articulation("carriage_slide")
    board_tilt = object_model.get_articulation("board_tilt")
    foot_bar_pivot = object_model.get_articulation("foot_bar_pivot")

    ctx.expect_overlap(
        carriage,
        frame,
        axes="z",
        elem_a="sleeve_front",
        elem_b="vertical_rail",
        min_overlap=0.20,
        name="carriage sleeve covers rail at rest",
    )
    ctx.expect_gap(
        carriage,
        frame,
        axis="y",
        positive_elem="sleeve_rear",
        negative_elem="vertical_rail",
        max_gap=0.001,
        max_penetration=0.0,
        name="rear sleeve rides on the rail",
    )
    ctx.expect_gap(
        frame,
        carriage,
        axis="y",
        positive_elem="vertical_rail",
        negative_elem="sleeve_front",
        max_gap=0.001,
        max_penetration=0.0,
        name="front sleeve rides on the rail",
    )

    rest_carriage = ctx.part_world_position(carriage)
    with ctx.pose({carriage_slide: 0.34}):
        raised_carriage = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            carriage,
            frame,
            axes="z",
            elem_a="sleeve_front",
            elem_b="vertical_rail",
            min_overlap=0.20,
            name="raised carriage remains on vertical rail",
        )

    ctx.check(
        "carriage slides upward on the rail",
        rest_carriage is not None
        and raised_carriage is not None
        and raised_carriage[2] > rest_carriage[2] + 0.30,
        details=f"rest={rest_carriage}, raised={raised_carriage}",
    )

    def _elem_center(part_obj, elem_name):
        aabb = ctx.part_element_world_aabb(part_obj, elem=elem_name)
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    rest_board_panel = _elem_center(board, "board_panel")
    with ctx.pose({board_tilt: 0.80}):
        tilted_board_panel = _elem_center(board, "board_panel")
    ctx.check(
        "board rotates on the carriage horizontal axis",
        rest_board_panel is not None
        and tilted_board_panel is not None
        and abs(tilted_board_panel[1] - rest_board_panel[1]) > 0.055,
        details=f"rest={rest_board_panel}, tilted={tilted_board_panel}",
    )

    rest_tread = _elem_center(foot_bar, "tread_bar")
    with ctx.pose({foot_bar_pivot: 0.50}):
        swung_tread = _elem_center(foot_bar, "tread_bar")
    ctx.check(
        "lower foot bar swings on transverse pivot",
        rest_tread is not None
        and swung_tread is not None
        and abs(swung_tread[2] - rest_tread[2]) > 0.035,
        details=f"rest={rest_tread}, swung={swung_tread}",
    )

    return ctx.report()


object_model = build_object_model()
