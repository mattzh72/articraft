from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 1.75
BASE_WIDTH = 1.22
LOWER_BODY_HEIGHT = 0.16
REAR_HOUSE_HEIGHT = 0.14
PEDESTAL_HEIGHT = 0.12
BOOM_MOUNT_X = -0.40
BOOM_MOUNT_Z = LOWER_BODY_HEIGHT + REAR_HOUSE_HEIGHT + PEDESTAL_HEIGHT

OUTER_LENGTH = 1.95
OUTER_WIDTH = 0.26
OUTER_HEIGHT = 0.20
OUTER_WALL = 0.012
OUTER_REAR_CAP = 0.16
OUTER_SHOE_HEIGHT = 0.08

STAGE1_LENGTH = 1.72
STAGE1_WIDTH = 0.22
STAGE1_HEIGHT = 0.16
STAGE1_WALL = 0.011
STAGE1_REAR_CAP = 0.10

STAGE2_LENGTH = 1.47
STAGE2_WIDTH = 0.18
STAGE2_HEIGHT = 0.13
STAGE2_WALL = 0.010
STAGE2_REAR_CAP = 0.09

STAGE3_LENGTH = 1.24
STAGE3_WIDTH = 0.145
STAGE3_HEIGHT = 0.10
STAGE3_WALL = 0.009
STAGE3_REAR_CAP = 0.08
STAGE3_HEAD_LENGTH = 0.18

STAGE1_HOME = 0.16
STAGE2_HOME = 0.14
STAGE3_HOME = 0.12

STAGE1_MAX = OUTER_LENGTH - 0.56 - STAGE1_HOME
STAGE2_MAX = STAGE1_LENGTH - 0.48 - STAGE2_HOME
STAGE3_MAX = STAGE2_LENGTH - 0.40 - STAGE3_HOME

STAGE1_GUIDE_THICK = ((OUTER_HEIGHT - 2.0 * OUTER_WALL) - STAGE1_HEIGHT) / 2.0
STAGE2_GUIDE_THICK = ((STAGE1_HEIGHT - 2.0 * STAGE1_WALL) - STAGE2_HEIGHT) / 2.0
STAGE3_GUIDE_THICK = ((STAGE2_HEIGHT - 2.0 * STAGE2_WALL) - STAGE3_HEIGHT) / 2.0
GUIDE_PAD_LENGTH = 0.22
GUIDE_PAD_START = 0.05
GUIDE_PAD_WIDTH_RATIO = 0.62
FUSE_OVERLAP = 0.001

STAGE1_Z_IN_OUTER = OUTER_SHOE_HEIGHT + OUTER_WALL + ((OUTER_HEIGHT - 2.0 * OUTER_WALL) - STAGE1_HEIGHT) / 2.0
STAGE2_Z_IN_STAGE1 = STAGE1_WALL + ((STAGE1_HEIGHT - 2.0 * STAGE1_WALL) - STAGE2_HEIGHT) / 2.0
STAGE3_Z_IN_STAGE2 = STAGE2_WALL + ((STAGE2_HEIGHT - 2.0 * STAGE2_WALL) - STAGE3_HEIGHT) / 2.0


def _tube_frame(
    *,
    length: float,
    width: float,
    height: float,
    wall: float,
    rear_cap: float,
    z_offset: float = 0.0,
) -> cq.Workplane:
    bottom = (
        cq.Workplane("XY")
        .box(length, width, wall, centered=(False, True, False))
        .translate((0.0, 0.0, z_offset))
    )
    top = (
        cq.Workplane("XY")
        .box(length, width, wall, centered=(False, True, False))
        .translate((0.0, 0.0, z_offset + height - wall))
    )
    wall_height = max(height - 2.0 * wall, wall)
    left = (
        cq.Workplane("XY")
        .box(length, wall, wall_height, centered=(False, True, False))
        .translate((0.0, (width - wall) / 2.0, z_offset + wall))
    )
    right = (
        cq.Workplane("XY")
        .box(length, wall, wall_height, centered=(False, True, False))
        .translate((0.0, -(width - wall) / 2.0, z_offset + wall))
    )
    rear = (
        cq.Workplane("XY")
        .box(rear_cap, width, height, centered=(False, True, False))
        .translate((0.0, 0.0, z_offset))
    )
    return bottom.union(top).union(left).union(right).union(rear)


def _guide_pads(
    *,
    body_width: float,
    body_height: float,
    top_pad: float,
    bottom_pad: float,
    start_x: float = GUIDE_PAD_START,
    pad_length: float = GUIDE_PAD_LENGTH,
) -> cq.Workplane:
    pad_width = body_width * GUIDE_PAD_WIDTH_RATIO
    pads = None
    if bottom_pad > 0.0:
        pads = (
            cq.Workplane("XY")
            .box(pad_length, pad_width, bottom_pad + FUSE_OVERLAP, centered=(False, True, False))
            .translate((start_x, 0.0, -bottom_pad))
        )
    if top_pad > 0.0:
        top = (
            cq.Workplane("XY")
            .box(pad_length, pad_width, top_pad + FUSE_OVERLAP, centered=(False, True, False))
            .translate((start_x, 0.0, body_height - FUSE_OVERLAP))
        )
        pads = top if pads is None else pads.union(top)
    return pads


def _base_body_shape() -> cq.Workplane:
    lower = (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, LOWER_BODY_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.04)
    )
    rear_house = (
        cq.Workplane("XY")
        .box(1.10, 0.92, REAR_HOUSE_HEIGHT, centered=(True, True, False))
        .translate((-0.18, 0.0, LOWER_BODY_HEIGHT))
        .edges("|Z")
        .fillet(0.03)
    )
    hood = (
        cq.Workplane("XY")
        .workplane(offset=LOWER_BODY_HEIGHT)
        .center(0.43, 0.0)
        .rect(0.72, 0.86)
        .workplane(offset=0.12)
        .center(0.58, 0.0)
        .rect(0.36, 0.72)
        .loft(combine=True)
    )
    pedestal = (
        cq.Workplane("XY")
        .box(0.44, 0.38, PEDESTAL_HEIGHT, centered=(True, True, False))
        .translate((BOOM_MOUNT_X, 0.0, LOWER_BODY_HEIGHT + REAR_HOUSE_HEIGHT))
        .edges("|Z")
        .fillet(0.025)
    )
    rear_counterweight = (
        cq.Workplane("XY")
        .box(0.40, 1.02, 0.10, centered=(True, True, False))
        .translate((-0.60, 0.0, LOWER_BODY_HEIGHT))
        .edges("|Z")
        .fillet(0.025)
    )
    return lower.union(rear_house).union(hood).union(pedestal).union(rear_counterweight)


def _outer_boom_shape() -> cq.Workplane:
    tube = _tube_frame(
        length=OUTER_LENGTH,
        width=OUTER_WIDTH,
        height=OUTER_HEIGHT,
        wall=OUTER_WALL,
        rear_cap=OUTER_REAR_CAP,
        z_offset=OUTER_SHOE_HEIGHT,
    )
    shoe = cq.Workplane("XY").box(0.66, OUTER_WIDTH + 0.10, OUTER_SHOE_HEIGHT, centered=(False, True, False))
    top_service_box = (
        cq.Workplane("XY")
        .box(0.52, OUTER_WIDTH * 0.58, 0.045, centered=(False, True, False))
        .translate((0.22, 0.0, OUTER_SHOE_HEIGHT + OUTER_HEIGHT))
    )
    return shoe.union(tube).union(top_service_box)


def _stage_shape(
    *,
    length: float,
    width: float,
    height: float,
    wall: float,
    rear_cap: float,
    guide_thickness: float,
    head_length: float = 0.0,
) -> cq.Workplane:
    stage = _tube_frame(length=length, width=width, height=height, wall=wall, rear_cap=rear_cap)
    guides = _guide_pads(
        body_width=width,
        body_height=height,
        top_pad=guide_thickness,
        bottom_pad=guide_thickness,
    )
    if guides is not None:
        stage = stage.union(guides)
    if head_length > 0.0:
        tip_head = (
            cq.Workplane("XY")
            .box(head_length, width * 0.55, height * 0.72, centered=(False, True, False))
            .translate((length - 0.08, 0.0, height * 0.14))
        )
        return stage.union(tip_head)
    return stage


def _add_part(
    model: ArticulatedObject,
    *,
    name: str,
    shape: cq.Workplane,
    material: str,
    visual_name: str,
):
    part = model.part(name)
    part.visual(mesh_from_cadquery(shape, name), material=material, name=visual_name)
    return part


def _add_box_visual(
    part,
    *,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material: str,
    name: str,
):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _add_tube_part(
    model: ArticulatedObject,
    *,
    name: str,
    length: float,
    width: float,
    height: float,
    wall: float,
    rear_cap: float,
    material: str,
    z_offset: float = 0.0,
    inner_guide_thickness: float = 0.0,
    add_shoe: bool = False,
    add_service_box: bool = False,
):
    part = model.part(name)

    _add_box_visual(
        part,
        size=(length, width, wall),
        xyz=(length / 2.0, 0.0, z_offset + wall / 2.0),
        material=material,
        name=f"{name}_bottom",
    )
    _add_box_visual(
        part,
        size=(length, width, wall),
        xyz=(length / 2.0, 0.0, z_offset + height - wall / 2.0),
        material=material,
        name=f"{name}_top",
    )
    _add_box_visual(
        part,
        size=(length, wall, height - 2.0 * wall),
        xyz=(length / 2.0, (width - wall) / 2.0, z_offset + height / 2.0),
        material=material,
        name=f"{name}_left_wall",
    )
    _add_box_visual(
        part,
        size=(length, wall, height - 2.0 * wall),
        xyz=(length / 2.0, -(width - wall) / 2.0, z_offset + height / 2.0),
        material=material,
        name=f"{name}_right_wall",
    )
    _add_box_visual(
        part,
        size=(rear_cap, width, height),
        xyz=(rear_cap / 2.0, 0.0, z_offset + height / 2.0),
        material=material,
        name=f"{name}_rear_cap",
    )

    if inner_guide_thickness > 0.0:
        _add_box_visual(
            part,
            size=(GUIDE_PAD_LENGTH, width * GUIDE_PAD_WIDTH_RATIO, inner_guide_thickness),
            xyz=(
                GUIDE_PAD_START + GUIDE_PAD_LENGTH / 2.0,
                0.0,
                z_offset + wall + inner_guide_thickness / 2.0,
            ),
            material=material,
            name=f"{name}_inner_bottom_guide",
        )
        _add_box_visual(
            part,
            size=(GUIDE_PAD_LENGTH, width * GUIDE_PAD_WIDTH_RATIO, inner_guide_thickness),
            xyz=(
                GUIDE_PAD_START + GUIDE_PAD_LENGTH / 2.0,
                0.0,
                z_offset + height - wall - inner_guide_thickness / 2.0,
            ),
            material=material,
            name=f"{name}_inner_top_guide",
        )

    if add_shoe:
        _add_box_visual(
            part,
            size=(0.66, width + 0.10, OUTER_SHOE_HEIGHT),
            xyz=(0.33, 0.0, OUTER_SHOE_HEIGHT / 2.0),
            material=material,
            name=f"{name}_shoe",
        )
    if add_service_box:
        _add_box_visual(
            part,
            size=(0.52, width * 0.58, 0.045),
            xyz=(0.88, 0.0, z_offset + height + 0.0225),
            material=material,
            name=f"{name}_service_box",
        )

    return part


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_service_telescoping_boom")

    model.material("body_charcoal", rgba=(0.20, 0.22, 0.24, 1.0))
    model.material("boom_outer_orange", rgba=(0.93, 0.49, 0.12, 1.0))
    model.material("boom_stage_orange", rgba=(0.95, 0.60, 0.18, 1.0))
    model.material("boom_stage_sand", rgba=(0.84, 0.77, 0.64, 1.0))
    model.material("boom_stage_steel", rgba=(0.72, 0.74, 0.76, 1.0))

    base_body = _add_part(
        model,
        name="base_body",
        shape=_base_body_shape(),
        material="body_charcoal",
        visual_name="base_shell",
    )
    outer_tube = _add_tube_part(
        model,
        name="outer_tube",
        length=OUTER_LENGTH,
        width=OUTER_WIDTH,
        height=OUTER_HEIGHT,
        wall=OUTER_WALL,
        rear_cap=OUTER_REAR_CAP,
        material="boom_outer_orange",
        z_offset=OUTER_SHOE_HEIGHT,
        inner_guide_thickness=STAGE1_GUIDE_THICK,
        add_shoe=True,
        add_service_box=True,
    )
    stage_1 = _add_tube_part(
        model,
        name="stage_1",
        length=STAGE1_LENGTH,
        width=STAGE1_WIDTH,
        height=STAGE1_HEIGHT,
        wall=STAGE1_WALL,
        rear_cap=STAGE1_REAR_CAP,
        material="boom_stage_orange",
        inner_guide_thickness=STAGE2_GUIDE_THICK,
    )
    stage_2 = _add_tube_part(
        model,
        name="stage_2",
        length=STAGE2_LENGTH,
        width=STAGE2_WIDTH,
        height=STAGE2_HEIGHT,
        wall=STAGE2_WALL,
        rear_cap=STAGE2_REAR_CAP,
        material="boom_stage_sand",
        inner_guide_thickness=STAGE3_GUIDE_THICK,
    )
    stage_3 = _add_tube_part(
        model,
        name="stage_3",
        length=STAGE3_LENGTH + STAGE3_HEAD_LENGTH,
        width=STAGE3_WIDTH,
        height=STAGE3_HEIGHT,
        wall=STAGE3_WALL,
        rear_cap=STAGE3_REAR_CAP,
        material="boom_stage_steel",
    )

    model.articulation(
        "base_to_outer",
        ArticulationType.FIXED,
        parent=base_body,
        child=outer_tube,
        origin=Origin(xyz=(BOOM_MOUNT_X, 0.0, BOOM_MOUNT_Z)),
    )
    model.articulation(
        "outer_to_stage_1",
        ArticulationType.PRISMATIC,
        parent=outer_tube,
        child=stage_1,
        origin=Origin(xyz=(STAGE1_HOME, 0.0, STAGE1_Z_IN_OUTER)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.45, lower=0.0, upper=STAGE1_MAX),
    )
    model.articulation(
        "stage_1_to_stage_2",
        ArticulationType.PRISMATIC,
        parent=stage_1,
        child=stage_2,
        origin=Origin(xyz=(STAGE2_HOME, 0.0, STAGE2_Z_IN_STAGE1)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.50, lower=0.0, upper=STAGE2_MAX),
    )
    model.articulation(
        "stage_2_to_stage_3",
        ArticulationType.PRISMATIC,
        parent=stage_2,
        child=stage_3,
        origin=Origin(xyz=(STAGE3_HOME, 0.0, STAGE3_Z_IN_STAGE2)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.55, lower=0.0, upper=STAGE3_MAX),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_body = object_model.get_part("base_body")
    outer_tube = object_model.get_part("outer_tube")
    stage_1 = object_model.get_part("stage_1")
    stage_2 = object_model.get_part("stage_2")
    stage_3 = object_model.get_part("stage_3")

    outer_to_stage_1 = object_model.get_articulation("outer_to_stage_1")
    stage_1_to_stage_2 = object_model.get_articulation("stage_1_to_stage_2")
    stage_2_to_stage_3 = object_model.get_articulation("stage_2_to_stage_3")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

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

    prismatic_joints = (outer_to_stage_1, stage_1_to_stage_2, stage_2_to_stage_3)
    joint_ok = all(
        joint.articulation_type == ArticulationType.PRISMATIC and tuple(joint.axis) == (1.0, 0.0, 0.0)
        for joint in prismatic_joints
    )
    ctx.check(
        "serial telescoping joints are prismatic",
        joint_ok,
        details="Expected three serial +X prismatic joints for the telescoping boom stages.",
    )

    ctx.expect_contact(base_body, outer_tube, name="base supports outer tube")

    with ctx.pose({outer_to_stage_1: 0.0, stage_1_to_stage_2: 0.0, stage_2_to_stage_3: 0.0}):
        ctx.expect_within(stage_1, outer_tube, axes="yz", margin=0.0, name="stage 1 nests within outer tube")
        ctx.expect_within(stage_2, stage_1, axes="yz", margin=0.0, name="stage 2 nests within stage 1")
        ctx.expect_within(stage_3, stage_2, axes="yz", margin=0.0, name="stage 3 nests within stage 2")

    with ctx.pose(
        {
            outer_to_stage_1: STAGE1_MAX,
            stage_1_to_stage_2: STAGE2_MAX,
            stage_2_to_stage_3: STAGE3_MAX,
        }
    ):
        ctx.expect_within(stage_1, outer_tube, axes="yz", margin=0.0, name="stage 1 stays guided at full reach")
        ctx.expect_within(stage_2, stage_1, axes="yz", margin=0.0, name="stage 2 stays guided at full reach")
        ctx.expect_within(stage_3, stage_2, axes="yz", margin=0.0, name="stage 3 stays guided at full reach")
        ctx.expect_overlap(stage_1, outer_tube, axes="x", min_overlap=0.55, name="outer and stage 1 keep overlap")
        ctx.expect_overlap(stage_2, stage_1, axes="x", min_overlap=0.47, name="stage 1 and stage 2 keep overlap")
        ctx.expect_overlap(stage_3, stage_2, axes="x", min_overlap=0.39, name="stage 2 and stage 3 keep overlap")
        extended_tip_x = ctx.part_world_position(stage_3)[0]

    with ctx.pose({outer_to_stage_1: 0.0, stage_1_to_stage_2: 0.0, stage_2_to_stage_3: 0.0}):
        stowed_tip_x = ctx.part_world_position(stage_3)[0]

    ctx.check(
        "tip stage extends forward",
        extended_tip_x > stowed_tip_x + 3.0,
        details=(
            f"Expected the serial prismatic joints to move the tip stage forward by more than 3.0 m; "
            f"stowed x={stowed_tip_x:.3f}, extended x={extended_tip_x:.3f}."
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
