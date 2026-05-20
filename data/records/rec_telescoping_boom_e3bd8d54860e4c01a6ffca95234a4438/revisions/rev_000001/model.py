from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BOOM_Z = 0.85


def _add_box(
    part,
    name: str,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material: Material,
) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _add_rect_tube(
    part,
    *,
    rear: float,
    front: float,
    width: float,
    height: float,
    wall: float,
    material: Material,
    z_center: float = 0.0,
) -> None:
    """Four connected wall boxes forming an open rectangular box tube."""

    length = front - rear
    x_mid = (front + rear) / 2.0
    _add_box(
        part,
        "top_wall",
        (length, width, wall),
        (x_mid, 0.0, z_center + height / 2.0 - wall / 2.0),
        material,
    )
    _add_box(
        part,
        "bottom_wall",
        (length, width, wall),
        (x_mid, 0.0, z_center - height / 2.0 + wall / 2.0),
        material,
    )
    _add_box(
        part,
        "side_wall_0",
        (length, wall, height),
        (x_mid, width / 2.0 - wall / 2.0, z_center),
        material,
    )
    _add_box(
        part,
        "side_wall_1",
        (length, wall, height),
        (x_mid, -width / 2.0 + wall / 2.0, z_center),
        material,
    )


def _add_outer_collar(
    part,
    *,
    front: float,
    width: float,
    height: float,
    wall: float,
    material: Material,
    z_center: float = 0.0,
) -> None:
    """Short welded reinforcement wrapped around the mouth of a boom section."""

    collar_len = 0.10
    collar = wall * 0.85
    x_mid = front - collar_len / 2.0
    _add_box(
        part,
        "mouth_top_collar",
        (collar_len, width + 0.035, collar),
        (x_mid, 0.0, z_center + height / 2.0 + collar / 2.0),
        material,
    )
    _add_box(
        part,
        "mouth_bottom_collar",
        (collar_len, width + 0.035, collar),
        (x_mid, 0.0, z_center - height / 2.0 - collar / 2.0),
        material,
    )
    _add_box(
        part,
        "mouth_side_collar_0",
        (collar_len, collar, height + 0.035),
        (x_mid, width / 2.0 + collar / 2.0, z_center),
        material,
    )
    _add_box(
        part,
        "mouth_side_collar_1",
        (collar_len, collar, height + 0.035),
        (x_mid, -width / 2.0 - collar / 2.0, z_center),
        material,
    )


def _add_guide_pads(
    part,
    *,
    prefix: str,
    x_front: float,
    parent_width: float,
    parent_height: float,
    parent_wall: float,
    child_width: float,
    child_height: float,
    material: Material,
    z_center: float = 0.0,
) -> None:
    """Low-friction pads that fill the guide clearance and touch the sliding tube."""

    pad_len = 0.12
    embed = 0.001
    x_mid = x_front - pad_len / 2.0
    inner_top = parent_height / 2.0 - parent_wall
    inner_side = parent_width / 2.0 - parent_wall
    top_gap = inner_top - child_height / 2.0
    side_gap = inner_side - child_width / 2.0
    if top_gap > 0.0:
        pad_h = top_gap + embed
        _add_box(
            part,
            f"{prefix}_top_pad",
            (pad_len, child_width * 0.48, pad_h),
            (x_mid, 0.0, z_center + child_height / 2.0 + pad_h / 2.0),
            material,
        )
        _add_box(
            part,
            f"{prefix}_bottom_pad",
            (pad_len, child_width * 0.48, pad_h),
            (x_mid, 0.0, z_center - child_height / 2.0 - pad_h / 2.0),
            material,
        )
    if side_gap > 0.0:
        pad_w = side_gap + embed
        _add_box(
            part,
            f"{prefix}_side_pad_0",
            (pad_len, pad_w, child_height * 0.55),
            (x_mid, child_width / 2.0 + pad_w / 2.0, z_center),
            material,
        )
        _add_box(
            part,
            f"{prefix}_side_pad_1",
            (pad_len, pad_w, child_height * 0.55),
            (x_mid, -child_width / 2.0 - pad_w / 2.0, z_center),
            material,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="box_tube_telescoping_reach_boom")

    bracket_mat = model.material("dark_powder_coated_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    gusset_mat = model.material("blue_base_paint", rgba=(0.05, 0.12, 0.24, 1.0))
    outer_mat = model.material("safety_yellow_tube", rgba=(0.94, 0.64, 0.05, 1.0))
    mid_mat = model.material("orange_inner_tube", rgba=(0.93, 0.36, 0.08, 1.0))
    inner_mat = model.material("zinc_plated_inner_tube", rgba=(0.64, 0.66, 0.62, 1.0))
    tip_mat = model.material("dark_tip_tube", rgba=(0.22, 0.23, 0.23, 1.0))
    pad_mat = model.material("black_polymer_wear_pads", rgba=(0.01, 0.01, 0.012, 1.0))

    base = model.part("base_bracket")
    _add_box(base, "floor_plate", (0.90, 0.58, 0.04), (0.04, 0.0, 0.02), bracket_mat)
    _add_box(base, "pedestal_block", (0.34, 0.22, 0.52), (0.00, 0.0, 0.30), gusset_mat)
    _add_box(base, "center_web", (0.54, 0.070, 0.69), (0.00, 0.0, 0.385), gusset_mat)
    _add_box(base, "cheek_plate_0", (0.56, 0.030, 0.72), (0.00, 0.167, 0.400), gusset_mat)
    _add_box(base, "cheek_plate_1", (0.56, 0.030, 0.72), (0.00, -0.167, 0.400), gusset_mat)

    guide_len = 0.50
    guide_width = 0.31
    guide_height = 0.25
    guide_wall = 0.028
    _add_rect_tube(
        base,
        rear=-guide_len / 2.0,
        front=guide_len / 2.0,
        width=guide_width,
        height=guide_height,
        wall=guide_wall,
        material=bracket_mat,
        z_center=BOOM_Z,
    )
    _add_guide_pads(
        base,
        prefix="base_guide",
        x_front=guide_len / 2.0,
        parent_width=guide_width,
        parent_height=guide_height,
        parent_wall=guide_wall,
        child_width=0.240,
        child_height=0.180,
        material=pad_mat,
        z_center=BOOM_Z,
    )

    boom_0 = model.part("boom_0")
    _add_rect_tube(
        boom_0,
        rear=-0.48,
        front=1.30,
        width=0.240,
        height=0.180,
        wall=0.014,
        material=outer_mat,
    )
    _add_outer_collar(
        boom_0,
        front=1.30,
        width=0.240,
        height=0.180,
        wall=0.014,
        material=outer_mat,
    )
    _add_guide_pads(
        boom_0,
        prefix="front",
        x_front=1.30,
        parent_width=0.240,
        parent_height=0.180,
        parent_wall=0.014,
        child_width=0.190,
        child_height=0.130,
        material=pad_mat,
    )

    boom_1 = model.part("boom_1")
    _add_rect_tube(
        boom_1,
        rear=-0.82,
        front=1.00,
        width=0.190,
        height=0.130,
        wall=0.011,
        material=mid_mat,
    )
    _add_outer_collar(
        boom_1,
        front=1.00,
        width=0.190,
        height=0.130,
        wall=0.011,
        material=mid_mat,
    )
    _add_guide_pads(
        boom_1,
        prefix="front",
        x_front=1.00,
        parent_width=0.190,
        parent_height=0.130,
        parent_wall=0.011,
        child_width=0.148,
        child_height=0.092,
        material=pad_mat,
    )

    boom_2 = model.part("boom_2")
    _add_rect_tube(
        boom_2,
        rear=-0.70,
        front=0.86,
        width=0.148,
        height=0.092,
        wall=0.009,
        material=inner_mat,
    )
    _add_outer_collar(
        boom_2,
        front=0.86,
        width=0.148,
        height=0.092,
        wall=0.009,
        material=inner_mat,
    )
    _add_guide_pads(
        boom_2,
        prefix="front",
        x_front=0.86,
        parent_width=0.148,
        parent_height=0.092,
        parent_wall=0.009,
        child_width=0.112,
        child_height=0.064,
        material=pad_mat,
    )

    boom_3 = model.part("boom_3")
    _add_rect_tube(
        boom_3,
        rear=-0.58,
        front=0.72,
        width=0.112,
        height=0.064,
        wall=0.007,
        material=tip_mat,
    )
    _add_box(boom_3, "tip_face_plate", (0.035, 0.132, 0.084), (0.7375, 0.0, 0.0), tip_mat)
    _add_box(boom_3, "tip_lug_0", (0.12, 0.014, 0.075), (0.81, 0.040, 0.0), bracket_mat)
    _add_box(boom_3, "tip_lug_1", (0.12, 0.014, 0.075), (0.81, -0.040, 0.0), bracket_mat)
    _add_box(boom_3, "tip_cross_pin", (0.024, 0.115, 0.024), (0.835, 0.0, 0.0), bracket_mat)

    model.articulation(
        "base_to_boom_0",
        ArticulationType.PRISMATIC,
        parent=base,
        child=boom_0,
        origin=Origin(xyz=(guide_len / 2.0, 0.0, BOOM_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.18, lower=0.0, upper=0.34),
    )
    model.articulation(
        "boom_0_to_boom_1",
        ArticulationType.PRISMATIC,
        parent=boom_0,
        child=boom_1,
        origin=Origin(xyz=(1.30, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=750.0, velocity=0.22, lower=0.0, upper=0.55),
    )
    model.articulation(
        "boom_1_to_boom_2",
        ArticulationType.PRISMATIC,
        parent=boom_1,
        child=boom_2,
        origin=Origin(xyz=(1.00, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=600.0, velocity=0.25, lower=0.0, upper=0.45),
    )
    model.articulation(
        "boom_2_to_boom_3",
        ArticulationType.PRISMATIC,
        parent=boom_2,
        child=boom_3,
        origin=Origin(xyz=(0.86, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=450.0, velocity=0.28, lower=0.0, upper=0.35),
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
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    base = object_model.get_part("base_bracket")
    boom_0 = object_model.get_part("boom_0")
    boom_1 = object_model.get_part("boom_1")
    boom_2 = object_model.get_part("boom_2")
    boom_3 = object_model.get_part("boom_3")
    j0 = object_model.get_articulation("base_to_boom_0")
    j1 = object_model.get_articulation("boom_0_to_boom_1")
    j2 = object_model.get_articulation("boom_1_to_boom_2")
    j3 = object_model.get_articulation("boom_2_to_boom_3")

    ctx.expect_overlap(
        boom_0,
        base,
        axes="x",
        min_overlap=0.12,
        elem_a="top_wall",
        elem_b="top_wall",
        name="outer boom remains captured in base guide",
    )
    ctx.expect_overlap(
        boom_1,
        boom_0,
        axes="x",
        min_overlap=0.70,
        elem_a="top_wall",
        elem_b="top_wall",
        name="second boom is deeply inserted in outer boom at rest",
    )
    ctx.expect_overlap(
        boom_2,
        boom_1,
        axes="x",
        min_overlap=0.60,
        elem_a="top_wall",
        elem_b="top_wall",
        name="third boom is deeply inserted at rest",
    )
    ctx.expect_overlap(
        boom_3,
        boom_2,
        axes="x",
        min_overlap=0.45,
        elem_a="top_wall",
        elem_b="top_wall",
        name="tip boom is deeply inserted at rest",
    )

    rest_tip = ctx.part_world_position(boom_3)
    with ctx.pose({j0: 0.34, j1: 0.55, j2: 0.45, j3: 0.35}):
        ctx.expect_overlap(
            boom_0,
            base,
            axes="x",
            min_overlap=0.08,
            elem_a="top_wall",
            elem_b="top_wall",
            name="outer boom remains captured in base guide when extended",
        )
        ctx.expect_overlap(
            boom_1,
            boom_0,
            axes="x",
            min_overlap=0.20,
            elem_a="top_wall",
            elem_b="top_wall",
            name="second boom retains insertion when extended",
        )
        ctx.expect_overlap(
            boom_2,
            boom_1,
            axes="x",
            min_overlap=0.20,
            elem_a="top_wall",
            elem_b="top_wall",
            name="third boom retains insertion when extended",
        )
        ctx.expect_overlap(
            boom_3,
            boom_2,
            axes="x",
            min_overlap=0.18,
            elem_a="top_wall",
            elem_b="top_wall",
            name="tip boom retains insertion when extended",
        )
        extended_tip = ctx.part_world_position(boom_3)

    ctx.check(
        "serial prismatic joints extend the tip along the boom axis",
        rest_tip is not None
        and extended_tip is not None
        and extended_tip[0] > rest_tip[0] + 1.55,
        details=f"rest_tip={rest_tip}, extended_tip={extended_tip}",
    )

    return ctx.report()


object_model = build_object_model()
