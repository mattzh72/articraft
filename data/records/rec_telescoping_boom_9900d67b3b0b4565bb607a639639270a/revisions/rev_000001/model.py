from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


OUTER_LENGTH = 2.30
MID1_LENGTH = 2.00
MID2_LENGTH = 1.60
TIP_LENGTH = 1.17

OUTER_WIDTH = 0.42
OUTER_HEIGHT = 0.32
OUTER_WALL = 0.014

MID1_WIDTH = 0.36
MID1_HEIGHT = 0.27
MID1_WALL = 0.012

MID2_WIDTH = 0.31
MID2_HEIGHT = 0.23
MID2_WALL = 0.011

TIP_WIDTH = 0.26
TIP_HEIGHT = 0.19
TIP_WALL = 0.010

STAGE1_INSERT = 0.40
STAGE2_INSERT = 0.34
STAGE3_INSERT = 0.26

STAGE1_TRAVEL = 1.15
STAGE2_TRAVEL = 0.95
STAGE3_TRAVEL = 0.80

FUSE_OVERLAP = 0.0010


def _add_box_visual(
    part,
    *,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    material: str,
    name: str,
) -> None:
    part.visual(Box(size), origin=Origin(xyz=center), material=material, name=name)


def _add_rect_tube_visuals(
    part,
    *,
    prefix: str,
    length: float,
    width: float,
    height: float,
    wall: float,
    x_start: float,
    material: str,
    closed_front: bool = False,
    closed_rear: bool = False,
) -> None:
    _add_box_visual(
        part,
        size=(length, width, wall),
        center=(x_start + length / 2.0, 0.0, height / 2.0 - wall / 2.0),
        material=material,
        name=f"{prefix}_top_wall",
    )
    _add_box_visual(
        part,
        size=(length, width, wall),
        center=(x_start + length / 2.0, 0.0, -height / 2.0 + wall / 2.0),
        material=material,
        name=f"{prefix}_bottom_wall",
    )
    side_height = height - 2.0 * wall + 2.0 * FUSE_OVERLAP
    _add_box_visual(
        part,
        size=(length, wall, side_height),
        center=(x_start + length / 2.0, width / 2.0 - wall / 2.0, 0.0),
        material=material,
        name=f"{prefix}_left_wall",
    )
    _add_box_visual(
        part,
        size=(length, wall, side_height),
        center=(x_start + length / 2.0, -width / 2.0 + wall / 2.0, 0.0),
        material=material,
        name=f"{prefix}_right_wall",
    )

    cap_height = height - 2.0 * wall + 2.0 * FUSE_OVERLAP
    cap_width = width - 2.0 * wall + 2.0 * FUSE_OVERLAP
    if closed_front:
        _add_box_visual(
            part,
            size=(wall, cap_width, cap_height),
            center=(x_start + length - wall / 2.0, 0.0, 0.0),
            material=material,
            name=f"{prefix}_front_cap",
        )
    if closed_rear:
        _add_box_visual(
            part,
            size=(wall, cap_width, cap_height),
            center=(x_start + wall / 2.0, 0.0, 0.0),
            material=material,
            name=f"{prefix}_rear_cap",
        )


def _add_wear_pad_visuals(
    part,
    *,
    prefix: str,
    child_length: float,
    child_width: float,
    child_height: float,
    x_start: float,
    side_protrusion: float,
    top_protrusion: float,
    material: str,
) -> None:
    for index, x_center in enumerate((x_start + 0.20, x_start + child_length - 0.24), start=1):
        _add_box_visual(
            part,
            size=(0.18, 0.13, top_protrusion + FUSE_OVERLAP),
            center=(x_center, 0.0, child_height / 2.0 + top_protrusion / 2.0 - FUSE_OVERLAP / 2.0),
            material=material,
            name=f"{prefix}_top_pad_{index}",
        )
        _add_box_visual(
            part,
            size=(0.18, 0.13, top_protrusion + FUSE_OVERLAP),
            center=(x_center, 0.0, -child_height / 2.0 - top_protrusion / 2.0 + FUSE_OVERLAP / 2.0),
            material=material,
            name=f"{prefix}_bottom_pad_{index}",
        )
        _add_box_visual(
            part,
            size=(0.18, side_protrusion + FUSE_OVERLAP, 0.11),
            center=(x_center, child_width / 2.0 + side_protrusion / 2.0 - FUSE_OVERLAP / 2.0, 0.0),
            material=material,
            name=f"{prefix}_left_pad_{index}",
        )
        _add_box_visual(
            part,
            size=(0.18, side_protrusion + FUSE_OVERLAP, 0.11),
            center=(x_center, -child_width / 2.0 - side_protrusion / 2.0 + FUSE_OVERLAP / 2.0, 0.0),
            material=material,
            name=f"{prefix}_right_pad_{index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_reach_boom")

    model.material("bracket_steel", rgba=(0.23, 0.25, 0.28, 1.0))
    model.material("outer_paint", rgba=(0.86, 0.64, 0.18, 1.0))
    model.material("mid_paint", rgba=(0.88, 0.68, 0.20, 1.0))
    model.material("inner_paint", rgba=(0.90, 0.70, 0.24, 1.0))
    model.material("tip_paint", rgba=(0.93, 0.75, 0.28, 1.0))
    model.material("wear_pad", rgba=(0.16, 0.17, 0.19, 1.0))

    base_bracket = model.part("base_bracket")
    _add_box_visual(
        base_bracket,
        size=(0.86, 0.62, 0.06),
        center=(0.12, 0.0, -0.27),
        material="bracket_steel",
        name="base_plate",
    )
    _add_box_visual(
        base_bracket,
        size=(0.82, 0.46, 0.12),
        center=(0.30, 0.0, -0.22),
        material="bracket_steel",
        name="saddle_block",
    )
    _add_box_visual(
        base_bracket,
        size=(0.74, 0.03, 0.45),
        center=(0.22, 0.245, 0.005),
        material="bracket_steel",
        name="left_cheek",
    )
    _add_box_visual(
        base_bracket,
        size=(0.74, 0.03, 0.45),
        center=(0.22, -0.245, 0.005),
        material="bracket_steel",
        name="right_cheek",
    )
    _add_box_visual(
        base_bracket,
        size=(0.08, 0.48, 0.33),
        center=(-0.10, 0.0, -0.015),
        material="bracket_steel",
        name="rear_web",
    )
    _add_box_visual(
        base_bracket,
        size=(0.24, 0.52, 0.06),
        center=(-0.01, 0.0, 0.19),
        material="bracket_steel",
        name="rear_top_tie",
    )
    _add_box_visual(
        base_bracket,
        size=(0.12, 0.52, 0.06),
        center=(0.62, 0.0, 0.19),
        material="bracket_steel",
        name="front_top_tie",
    )
    _add_box_visual(
        base_bracket,
        size=(0.08, 0.03, 0.30),
        center=(0.62, 0.23, -0.01),
        material="bracket_steel",
        name="left_front_post",
    )
    _add_box_visual(
        base_bracket,
        size=(0.08, 0.03, 0.30),
        center=(0.62, -0.23, -0.01),
        material="bracket_steel",
        name="right_front_post",
    )
    _add_box_visual(
        base_bracket,
        size=(0.20, 0.02, 0.12),
        center=(0.05, OUTER_WIDTH / 2.0 + 0.018, -0.01),
        material="bracket_steel",
        name="left_side_guide",
    )
    _add_box_visual(
        base_bracket,
        size=(0.20, 0.02, 0.12),
        center=(0.05, -OUTER_WIDTH / 2.0 - 0.018, -0.01),
        material="bracket_steel",
        name="right_side_guide",
    )
    _add_box_visual(
        base_bracket,
        size=(0.16, 0.10, 0.04),
        center=(-0.19, 0.0, -0.27),
        material="bracket_steel",
        name="rear_mount_pad",
    )

    outer_boom = model.part("outer_boom")
    _add_rect_tube_visuals(
        outer_boom,
        prefix="outer",
        length=OUTER_LENGTH,
        width=OUTER_WIDTH,
        height=OUTER_HEIGHT,
        wall=OUTER_WALL,
        x_start=0.0,
        material="outer_paint",
    )
    _add_box_visual(
        outer_boom,
        size=(0.66, 0.16, 0.018),
        center=(0.33, 0.0, -OUTER_HEIGHT / 2.0 + 0.009),
        material="wear_pad",
        name="outer_bottom_wear_strip",
    )
    _add_box_visual(
        outer_boom,
        size=(0.24, 0.22, 0.018),
        center=(0.12, 0.0, OUTER_HEIGHT / 2.0 - 0.009),
        material="wear_pad",
        name="outer_top_keeper",
    )

    mid_boom_1 = model.part("mid_boom_1")
    _add_rect_tube_visuals(
        mid_boom_1,
        prefix="mid1",
        length=MID1_LENGTH,
        width=MID1_WIDTH,
        height=MID1_HEIGHT,
        wall=MID1_WALL,
        x_start=-STAGE1_INSERT,
        material="mid_paint",
    )
    _add_wear_pad_visuals(
        mid_boom_1,
        prefix="mid1",
        child_length=MID1_LENGTH,
        child_width=MID1_WIDTH,
        child_height=MID1_HEIGHT,
        x_start=-STAGE1_INSERT,
        side_protrusion=(OUTER_WIDTH - 2.0 * OUTER_WALL - MID1_WIDTH) / 2.0,
        top_protrusion=(OUTER_HEIGHT - 2.0 * OUTER_WALL - MID1_HEIGHT) / 2.0,
        material="wear_pad",
    )

    mid_boom_2 = model.part("mid_boom_2")
    _add_rect_tube_visuals(
        mid_boom_2,
        prefix="mid2",
        length=MID2_LENGTH,
        width=MID2_WIDTH,
        height=MID2_HEIGHT,
        wall=MID2_WALL,
        x_start=-STAGE2_INSERT,
        material="inner_paint",
    )
    _add_wear_pad_visuals(
        mid_boom_2,
        prefix="mid2",
        child_length=MID2_LENGTH,
        child_width=MID2_WIDTH,
        child_height=MID2_HEIGHT,
        x_start=-STAGE2_INSERT,
        side_protrusion=(MID1_WIDTH - 2.0 * MID1_WALL - MID2_WIDTH) / 2.0,
        top_protrusion=(MID1_HEIGHT - 2.0 * MID1_WALL - MID2_HEIGHT) / 2.0,
        material="wear_pad",
    )

    tip_boom = model.part("tip_boom")
    _add_rect_tube_visuals(
        tip_boom,
        prefix="tip",
        length=TIP_LENGTH,
        width=TIP_WIDTH,
        height=TIP_HEIGHT,
        wall=TIP_WALL,
        x_start=-STAGE3_INSERT,
        material="tip_paint",
        closed_front=True,
    )
    _add_wear_pad_visuals(
        tip_boom,
        prefix="tip",
        child_length=TIP_LENGTH,
        child_width=TIP_WIDTH,
        child_height=TIP_HEIGHT,
        x_start=-STAGE3_INSERT,
        side_protrusion=(MID2_WIDTH - 2.0 * MID2_WALL - TIP_WIDTH) / 2.0,
        top_protrusion=(MID2_HEIGHT - 2.0 * MID2_WALL - TIP_HEIGHT) / 2.0,
        material="wear_pad",
    )
    _add_box_visual(
        tip_boom,
        size=(0.09, TIP_WIDTH * 0.72, TIP_HEIGHT * 0.56),
        center=(-STAGE3_INSERT + TIP_LENGTH - 0.045, 0.0, 0.0),
        material="tip_paint",
        name="tip_nose_block",
    )

    model.articulation(
        "bracket_to_outer",
        ArticulationType.FIXED,
        parent=base_bracket,
        child=outer_boom,
        origin=Origin(),
    )
    model.articulation(
        "outer_to_mid1",
        ArticulationType.PRISMATIC,
        parent=outer_boom,
        child=mid_boom_1,
        origin=Origin(xyz=(STAGE1_INSERT, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=0.40,
            lower=0.0,
            upper=STAGE1_TRAVEL,
        ),
    )
    model.articulation(
        "mid1_to_mid2",
        ArticulationType.PRISMATIC,
        parent=mid_boom_1,
        child=mid_boom_2,
        origin=Origin(xyz=(STAGE2_INSERT, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=110.0,
            velocity=0.45,
            lower=0.0,
            upper=STAGE2_TRAVEL,
        ),
    )
    model.articulation(
        "mid2_to_tip",
        ArticulationType.PRISMATIC,
        parent=mid_boom_2,
        child=tip_boom,
        origin=Origin(xyz=(STAGE3_INSERT, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.50,
            lower=0.0,
            upper=STAGE3_TRAVEL,
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

    base_bracket = object_model.get_part("base_bracket")
    outer_boom = object_model.get_part("outer_boom")
    mid_boom_1 = object_model.get_part("mid_boom_1")
    mid_boom_2 = object_model.get_part("mid_boom_2")
    tip_boom = object_model.get_part("tip_boom")

    outer_to_mid1 = object_model.get_articulation("outer_to_mid1")
    mid1_to_mid2 = object_model.get_articulation("mid1_to_mid2")
    mid2_to_tip = object_model.get_articulation("mid2_to_tip")

    ctx.expect_contact(
        outer_boom,
        base_bracket,
        contact_tol=0.002,
        name="outer boom is seated in the base bracket",
    )
    ctx.expect_within(
        mid_boom_1,
        outer_boom,
        axes="yz",
        margin=0.0,
        name="first telescoping stage stays centered in the outer boom",
    )
    ctx.expect_overlap(
        mid_boom_1,
        outer_boom,
        axes="x",
        min_overlap=1.80,
        name="first telescoping stage is deeply retained at rest",
    )
    ctx.expect_within(
        mid_boom_2,
        mid_boom_1,
        axes="yz",
        margin=0.0,
        name="second telescoping stage stays centered in the first moving boom",
    )
    ctx.expect_overlap(
        mid_boom_2,
        mid_boom_1,
        axes="x",
        min_overlap=1.55,
        name="second telescoping stage is retained at rest",
    )
    ctx.expect_within(
        tip_boom,
        mid_boom_2,
        axes="yz",
        margin=0.0,
        name="tip boom stays centered in the second moving boom",
    )
    ctx.expect_overlap(
        tip_boom,
        mid_boom_2,
        axes="x",
        min_overlap=1.10,
        name="tip boom is retained at rest",
    )

    rest_mid1 = ctx.part_world_position(mid_boom_1)
    rest_mid2 = ctx.part_world_position(mid_boom_2)
    rest_tip = ctx.part_world_position(tip_boom)

    with ctx.pose(
        {
            outer_to_mid1: STAGE1_TRAVEL,
            mid1_to_mid2: STAGE2_TRAVEL,
            mid2_to_tip: STAGE3_TRAVEL,
        }
    ):
        ctx.expect_within(
            mid_boom_1,
            outer_boom,
            axes="yz",
            margin=0.0,
            name="first telescoping stage stays centered at full extension",
        )
        ctx.expect_overlap(
            mid_boom_1,
            outer_boom,
            axes="x",
            min_overlap=1.10,
            name="first telescoping stage keeps retained insertion at full extension",
        )
        ctx.expect_within(
            mid_boom_2,
            mid_boom_1,
            axes="yz",
            margin=0.0,
            name="second telescoping stage stays centered at full extension",
        )
        ctx.expect_overlap(
            mid_boom_2,
            mid_boom_1,
            axes="x",
            min_overlap=0.60,
            name="second telescoping stage keeps retained insertion at full extension",
        )
        ctx.expect_within(
            tip_boom,
            mid_boom_2,
            axes="yz",
            margin=0.0,
            name="tip boom stays centered at full extension",
        )
        ctx.expect_overlap(
            tip_boom,
            mid_boom_2,
            axes="x",
            min_overlap=0.40,
            name="tip boom keeps retained insertion at full extension",
        )

        extended_mid1 = ctx.part_world_position(mid_boom_1)
        extended_mid2 = ctx.part_world_position(mid_boom_2)
        extended_tip = ctx.part_world_position(tip_boom)

    ctx.check(
        "first stage extends along +X",
        rest_mid1 is not None and extended_mid1 is not None and extended_mid1[0] > rest_mid1[0] + 1.0,
        details=f"rest={rest_mid1}, extended={extended_mid1}",
    )
    ctx.check(
        "second stage extends along +X",
        rest_mid2 is not None and extended_mid2 is not None and extended_mid2[0] > rest_mid2[0] + 2.0,
        details=f"rest={rest_mid2}, extended={extended_mid2}",
    )
    ctx.check(
        "tip stage extends along +X",
        rest_tip is not None and extended_tip is not None and extended_tip[0] > rest_tip[0] + 2.8,
        details=f"rest={rest_tip}, extended={extended_tip}",
    )

    ctx.expect_origin_distance(
        mid_boom_1,
        outer_boom,
        axes="yz",
        max_dist=0.001,
        name="first stage shares the boom centerline",
    )
    ctx.expect_origin_distance(
        mid_boom_2,
        outer_boom,
        axes="yz",
        max_dist=0.001,
        name="second stage shares the boom centerline",
    )
    ctx.expect_origin_distance(
        tip_boom,
        outer_boom,
        axes="yz",
        max_dist=0.001,
        name="tip stage shares the boom centerline",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
