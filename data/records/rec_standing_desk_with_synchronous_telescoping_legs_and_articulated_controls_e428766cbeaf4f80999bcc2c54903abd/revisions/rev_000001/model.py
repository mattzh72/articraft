from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


TOP_LENGTH = 1.82
TOP_DEPTH = 0.76
TOP_THICKNESS = 0.036

COLUMN_XS = (-0.62, 0.0, 0.62)

FOOT_HEIGHT = 0.034
FOOT_LENGTH = 0.68
FOOT_WIDTH = 0.10
BASE_RAIL_LENGTH = 1.52
BASE_RAIL_WIDTH = 0.10
BASE_RAIL_HEIGHT = 0.05

OUTER_WIDTH = 0.08
OUTER_DEPTH = 0.06
OUTER_HEIGHT = 0.58
OUTER_WALL = 0.003
OUTER_BOTTOM_CAP = 0.010

INNER_WIDTH = 0.068
INNER_DEPTH = 0.048
INNER_BELOW_JOINT = 0.53
INNER_ABOVE_JOINT = 0.04
INNER_PLATE_WIDTH = 0.14
INNER_PLATE_DEPTH = 0.10
INNER_PLATE_THICKNESS = 0.012
LIFT_TRAVEL = 0.42

FRAME_PAD_WIDTH = 0.16
FRAME_PAD_DEPTH = 0.12
FRAME_PAD_THICKNESS = 0.010
FRAME_RAIL_LENGTH = 1.56
FRAME_RAIL_DEPTH = 0.05
FRAME_RAIL_HEIGHT = 0.036
FRAME_RAIL_Y = 0.23
FRAME_END_BEAM_WIDTH = 0.08
FRAME_END_BEAM_DEPTH = 0.49
FRAME_END_BEAM_HEIGHT = 0.030
FRAME_END_X = 0.74
FRAME_CROSS_WIDTH = 0.09
FRAME_CROSS_DEPTH = 0.49
FRAME_CROSS_HEIGHT = 0.028
FRAME_TOP_Z = 0.046

CONTROLLER_WIDTH = 0.16
CONTROLLER_DEPTH = 0.045
CONTROLLER_TOP_THICKNESS = 0.006
CONTROLLER_CHEEK_WIDTH = 0.018
CONTROLLER_CHEEK_DEPTH = 0.032
CONTROLLER_CHEEK_HEIGHT = 0.030
CONTROLLER_BACK_DEPTH = 0.010
CONTROLLER_BACK_HEIGHT = 0.020


def _rounded_box(size: tuple[float, float, float], fillet: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(*size, centered=(True, True, False))
        .edges("|Z")
        .fillet(fillet)
    )


def _rect_tube(
    width: float,
    depth: float,
    height: float,
    wall: float,
    *,
    bottom_cap: float = 0.0,
) -> cq.Workplane:
    outer = cq.Workplane("XY").box(width, depth, height, centered=(True, True, False))
    inner = (
        cq.Workplane("XY")
        .box(
            width - 2.0 * wall,
            depth - 2.0 * wall,
            height - bottom_cap + 0.002,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, bottom_cap))
    )
    return outer.cut(inner)


def _shift(shape: cq.Workplane, xyz: tuple[float, float, float]) -> cq.Workplane:
    return shape.translate(xyz)


def _base_spine_shape() -> cq.Workplane:
    spine = _rounded_box((BASE_RAIL_LENGTH, BASE_RAIL_WIDTH, BASE_RAIL_HEIGHT), 0.010)
    for x in COLUMN_XS:
        foot = _rounded_box((FOOT_WIDTH, FOOT_LENGTH, FOOT_HEIGHT), 0.012).translate(
            (x, 0.0, 0.0)
        )
        spine = spine.union(foot)
    return spine


def _outer_sleeve_shape(x: float) -> cq.Workplane:
    return _shift(
        _rect_tube(
            OUTER_WIDTH,
            OUTER_DEPTH,
            OUTER_HEIGHT,
            OUTER_WALL,
            bottom_cap=OUTER_BOTTOM_CAP,
        ),
        (x, 0.0, FOOT_HEIGHT - 0.002),
    )


def _inner_stage_body_shape() -> cq.Workplane:
    stage_height = INNER_BELOW_JOINT + INNER_ABOVE_JOINT
    stage_z = -INNER_BELOW_JOINT
    body = (
        cq.Workplane("XY")
        .box(INNER_WIDTH, INNER_DEPTH, stage_height, centered=(True, True, False))
        .translate((0.0, 0.0, stage_z))
    )
    guide_pad_thickness = ((OUTER_WIDTH - 2.0 * OUTER_WALL) - INNER_WIDTH) / 2.0
    guide_pad_depth = 0.028
    guide_pad_height = 0.090
    for pad_center_z in (-0.21, -0.06):
        for side in (-1.0, 1.0):
            pad = (
                cq.Workplane("XY")
                .box(
                    guide_pad_thickness,
                    guide_pad_depth,
                    guide_pad_height,
                    centered=(True, True, False),
                )
                .translate(
                    (
                        side * (INNER_WIDTH / 2.0 + guide_pad_thickness / 2.0),
                        0.0,
                        pad_center_z - guide_pad_height / 2.0,
                    )
                )
            )
            body = body.union(pad)
    return body


def _inner_mount_plate_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(
            INNER_PLATE_WIDTH,
            INNER_PLATE_DEPTH,
            INNER_PLATE_THICKNESS,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, INNER_ABOVE_JOINT))
    )


def _frame_core_shape() -> cq.Workplane:
    rail_z = FRAME_PAD_THICKNESS
    core = (
        cq.Workplane("XY")
        .box(
            FRAME_RAIL_LENGTH,
            FRAME_RAIL_DEPTH,
            FRAME_RAIL_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, -FRAME_RAIL_Y, rail_z))
    )
    core = core.union(
        cq.Workplane("XY")
        .box(
            FRAME_RAIL_LENGTH,
            FRAME_RAIL_DEPTH,
            FRAME_RAIL_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, FRAME_RAIL_Y, rail_z))
    )
    for x in (-FRAME_END_X, 0.0, FRAME_END_X):
        core = core.union(
            cq.Workplane("XY")
            .box(
                FRAME_CROSS_WIDTH,
                FRAME_CROSS_DEPTH,
                FRAME_CROSS_HEIGHT,
                centered=(True, True, False),
            )
            .translate((x, 0.0, FRAME_PAD_THICKNESS))
        )
    for x in (-FRAME_END_X, FRAME_END_X):
        core = core.union(
            cq.Workplane("XY")
            .box(
                FRAME_END_BEAM_WIDTH,
                FRAME_END_BEAM_DEPTH,
                FRAME_END_BEAM_HEIGHT,
                centered=(True, True, False),
            )
            .translate((x, 0.0, FRAME_PAD_THICKNESS))
        )
    return core


def _frame_pad_shape(x: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(
            FRAME_PAD_WIDTH,
            FRAME_PAD_DEPTH,
            FRAME_PAD_THICKNESS,
            centered=(True, True, False),
        )
        .translate((x, 0.0, 0.0))
    )


def _top_shape() -> cq.Workplane:
    return _rounded_box((TOP_LENGTH, TOP_DEPTH, TOP_THICKNESS), 0.012)


def _controller_housing_plate() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(
            CONTROLLER_WIDTH,
            CONTROLLER_DEPTH,
            CONTROLLER_TOP_THICKNESS,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, -CONTROLLER_TOP_THICKNESS))
    )


def _controller_cheek(side_x: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(
            CONTROLLER_CHEEK_WIDTH,
            CONTROLLER_CHEEK_DEPTH,
            CONTROLLER_CHEEK_HEIGHT,
            centered=(True, True, False),
        )
        .translate(
            (
                side_x,
                -0.006 - CONTROLLER_CHEEK_DEPTH / 2.0,
                -CONTROLLER_TOP_THICKNESS - CONTROLLER_CHEEK_HEIGHT,
            )
        )
    )


def _controller_back_web() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(
            CONTROLLER_WIDTH - 2.0 * CONTROLLER_CHEEK_WIDTH,
            CONTROLLER_BACK_DEPTH,
            CONTROLLER_BACK_HEIGHT,
            centered=(True, True, False),
        )
        .translate(
            (
                0.0,
                CONTROLLER_DEPTH / 2.0 - CONTROLLER_BACK_DEPTH,
                -CONTROLLER_TOP_THICKNESS - CONTROLLER_BACK_HEIGHT,
            )
        )
    )


def _lever_pivot_bar() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.088, 0.010, 0.010)
        .edges("|X")
        .fillet(0.003)
    )


def _lever_paddle() -> cq.Workplane:
    neck = cq.Workplane("XY").box(0.018, 0.026, 0.012).translate((0.0, -0.012, -0.007))
    arm = (
        cq.Workplane("XY")
        .box(0.022, 0.095, 0.008)
        .translate((0.0, -0.050, -0.013))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 13.0)
    )
    tip = (
        cq.Workplane("XY")
        .box(0.030, 0.028, 0.012)
        .translate((0.0, -0.095, -0.028))
        .edges("|X")
        .fillet(0.004)
    )
    return neck.union(arm).union(tip)


def _mesh(shape: cq.Workplane, name: str):
    return mesh_from_cadquery(shape, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_style_standing_desk")

    powder_dark = model.material("powder_dark", rgba=(0.20, 0.21, 0.23, 1.0))
    powder_mid = model.material("powder_mid", rgba=(0.56, 0.58, 0.61, 1.0))
    walnut = model.material("walnut", rgba=(0.60, 0.45, 0.28, 1.0))
    controller_black = model.material("controller_black", rgba=(0.10, 0.10, 0.11, 1.0))

    base = model.part("base")
    base.visual(_mesh(_base_spine_shape(), "desk_base_spine"), material=powder_dark, name="base_spine")
    base.visual(
        _mesh(_outer_sleeve_shape(COLUMN_XS[0]), "outer_left_sleeve"),
        material=powder_mid,
        name="outer_left_sleeve",
    )
    base.visual(
        _mesh(_outer_sleeve_shape(COLUMN_XS[1]), "outer_center_sleeve"),
        material=powder_mid,
        name="outer_center_sleeve",
    )
    base.visual(
        _mesh(_outer_sleeve_shape(COLUMN_XS[2]), "outer_right_sleeve"),
        material=powder_mid,
        name="outer_right_sleeve",
    )

    left_inner = model.part("left_inner")
    left_inner.visual(
        _mesh(_inner_stage_body_shape(), "left_inner_stage_body"),
        material=powder_mid,
        name="stage_body",
    )
    left_inner.visual(
        _mesh(_inner_mount_plate_shape(), "left_inner_mount_plate"),
        material=powder_dark,
        name="mount_plate",
    )

    center_inner = model.part("center_inner")
    center_inner.visual(
        _mesh(_inner_stage_body_shape(), "center_inner_stage_body"),
        material=powder_mid,
        name="stage_body",
    )
    center_inner.visual(
        _mesh(_inner_mount_plate_shape(), "center_inner_mount_plate"),
        material=powder_dark,
        name="mount_plate",
    )

    right_inner = model.part("right_inner")
    right_inner.visual(
        _mesh(_inner_stage_body_shape(), "right_inner_stage_body"),
        material=powder_mid,
        name="stage_body",
    )
    right_inner.visual(
        _mesh(_inner_mount_plate_shape(), "right_inner_mount_plate"),
        material=powder_dark,
        name="mount_plate",
    )

    desk_frame = model.part("desk_frame")
    desk_frame.visual(
        _mesh(_frame_core_shape(), "desk_frame_core"),
        material=powder_dark,
        name="frame_core",
    )
    desk_frame.visual(
        _mesh(_frame_pad_shape(COLUMN_XS[0]), "desk_frame_pad_left"),
        material=powder_dark,
        name="left_frame_pad",
    )
    desk_frame.visual(
        _mesh(_frame_pad_shape(COLUMN_XS[1]), "desk_frame_pad_center"),
        material=powder_dark,
        name="center_frame_pad",
    )
    desk_frame.visual(
        _mesh(_frame_pad_shape(COLUMN_XS[2]), "desk_frame_pad_right"),
        material=powder_dark,
        name="right_frame_pad",
    )

    top = model.part("top")
    top.visual(_mesh(_top_shape(), "desk_top"), material=walnut, name="top_slab")

    controller_housing = model.part("controller_housing")
    controller_housing.visual(
        _mesh(_controller_housing_plate(), "controller_top_plate"),
        material=controller_black,
        name="housing_plate",
    )
    controller_housing.visual(
        _mesh(_controller_cheek(-0.050), "controller_left_cheek"),
        material=controller_black,
        name="left_cheek",
    )
    controller_housing.visual(
        _mesh(_controller_cheek(0.050), "controller_right_cheek"),
        material=controller_black,
        name="right_cheek",
    )
    controller_housing.visual(
        _mesh(_controller_back_web(), "controller_back_web"),
        material=controller_black,
        name="back_web",
    )

    controller_lever = model.part("controller_lever")
    controller_lever.visual(
        _mesh(_lever_pivot_bar(), "controller_lever_pivot_bar"),
        material=controller_black,
        name="pivot_bar",
    )
    controller_lever.visual(
        _mesh(_lever_paddle(), "controller_lever_paddle"),
        material=controller_black,
        name="lever_paddle",
    )

    for name, child, x in (
        ("base_to_left_inner", left_inner, COLUMN_XS[0]),
        ("base_to_center_inner", center_inner, COLUMN_XS[1]),
        ("base_to_right_inner", right_inner, COLUMN_XS[2]),
    ):
        model.articulation(
            name,
            ArticulationType.PRISMATIC,
            parent=base,
            child=child,
            origin=Origin(xyz=(x, 0.0, FOOT_HEIGHT + OUTER_HEIGHT)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                lower=0.0,
                upper=LIFT_TRAVEL,
                effort=900.0,
                velocity=0.05,
            ),
        )

    model.articulation(
        "center_inner_to_frame",
        ArticulationType.FIXED,
        parent=center_inner,
        child=desk_frame,
        origin=Origin(xyz=(0.0, 0.0, INNER_ABOVE_JOINT + INNER_PLATE_THICKNESS)),
    )
    model.articulation(
        "frame_to_top",
        ArticulationType.FIXED,
        parent=desk_frame,
        child=top,
        origin=Origin(xyz=(0.0, 0.0, FRAME_TOP_Z)),
    )
    model.articulation(
        "frame_to_controller_housing",
        ArticulationType.FIXED,
        parent=desk_frame,
        child=controller_housing,
        origin=Origin(xyz=(0.0, -0.335, FRAME_TOP_Z)),
    )
    model.articulation(
        "housing_to_lever",
        ArticulationType.REVOLUTE,
        parent=controller_housing,
        child=controller_lever,
        origin=Origin(xyz=(0.0, -0.014, -0.020)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.35,
            upper=0.35,
            effort=2.0,
            velocity=2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    left_inner = object_model.get_part("left_inner")
    center_inner = object_model.get_part("center_inner")
    right_inner = object_model.get_part("right_inner")
    desk_frame = object_model.get_part("desk_frame")
    top = object_model.get_part("top")
    controller_housing = object_model.get_part("controller_housing")
    controller_lever = object_model.get_part("controller_lever")

    left_lift = object_model.get_articulation("base_to_left_inner")
    center_lift = object_model.get_articulation("base_to_center_inner")
    right_lift = object_model.get_articulation("base_to_right_inner")
    lever_joint = object_model.get_articulation("housing_to_lever")

    ctx.check(
        "all lift joints translate upward",
        all(j.axis == (0.0, 0.0, 1.0) for j in (left_lift, center_lift, right_lift)),
        details=str([j.axis for j in (left_lift, center_lift, right_lift)]),
    )
    ctx.check(
        "controller lever pivots on transverse axis",
        lever_joint.axis == (-1.0, 0.0, 0.0),
        details=str(lever_joint.axis),
    )

    for inner_name, outer_name in (
        ("left_inner", "outer_left_sleeve"),
        ("center_inner", "outer_center_sleeve"),
        ("right_inner", "outer_right_sleeve"),
    ):
        ctx.allow_overlap(
            base,
            object_model.get_part(inner_name),
            elem_a=outer_name,
            elem_b="stage_body",
            reason=(
                "The telescoping sleeve is visually hollow, but the current exact-overlap "
                "backend treats the watertight sleeve mesh as a solid sliding-column proxy."
            ),
        )

    for inner, outer_name, tag in (
        (left_inner, "outer_left_sleeve", "left"),
        (center_inner, "outer_center_sleeve", "center"),
        (right_inner, "outer_right_sleeve", "right"),
    ):
        ctx.expect_within(
            inner,
            base,
            axes="xy",
            inner_elem="stage_body",
            outer_elem=outer_name,
            name=f"{tag} inner stage stays centered in its sleeve",
        )
        ctx.expect_overlap(
            inner,
            base,
            axes="z",
            elem_a="stage_body",
            elem_b=outer_name,
            min_overlap=0.10,
            name=f"{tag} inner stage retains insertion in sleeve",
        )

    ctx.expect_contact(
        center_inner,
        desk_frame,
        elem_a="mount_plate",
        elem_b="center_frame_pad",
        name="center stage supports moving frame",
    )
    ctx.expect_contact(
        top,
        desk_frame,
        elem_a="top_slab",
        elem_b="frame_core",
        name="desktop sits on moving frame",
    )
    ctx.expect_contact(
        controller_housing,
        top,
        elem_a="housing_plate",
        elem_b="top_slab",
        name="controller housing mounts under desktop",
    )

    with ctx.pose({left_lift: LIFT_TRAVEL, center_lift: LIFT_TRAVEL, right_lift: LIFT_TRAVEL}):
        ctx.expect_contact(
            left_inner,
            desk_frame,
            elem_a="mount_plate",
            elem_b="left_frame_pad",
            name="left stage meets frame pad at full raise",
        )
        ctx.expect_contact(
            right_inner,
            desk_frame,
            elem_a="mount_plate",
            elem_b="right_frame_pad",
            name="right stage meets frame pad at full raise",
        )
        ctx.expect_contact(
            center_inner,
            desk_frame,
            elem_a="mount_plate",
            elem_b="center_frame_pad",
            name="center stage stays seated at full raise",
        )

    left_rest = ctx.part_world_position(left_inner)
    center_rest = ctx.part_world_position(center_inner)
    right_rest = ctx.part_world_position(right_inner)
    with ctx.pose({left_lift: LIFT_TRAVEL, center_lift: LIFT_TRAVEL, right_lift: LIFT_TRAVEL}):
        left_high = ctx.part_world_position(left_inner)
        center_high = ctx.part_world_position(center_inner)
        right_high = ctx.part_world_position(right_inner)

    ctx.check(
        "all three columns raise in sync",
        all(
            rest is not None
            and high is not None
            and high[2] > rest[2] + 0.30
            for rest, high in (
                (left_rest, left_high),
                (center_rest, center_high),
                (right_rest, right_high),
            )
        ),
        details=f"rest={[left_rest, center_rest, right_rest]}, high={[left_high, center_high, right_high]}",
    )

    lever_rest = ctx.part_element_world_aabb(controller_lever, elem="lever_paddle")
    with ctx.pose({lever_joint: 0.30}):
        lever_up = ctx.part_element_world_aabb(controller_lever, elem="lever_paddle")
    ctx.check(
        "lever tip swings upward for positive input",
        lever_rest is not None
        and lever_up is not None
        and lever_up[1][2] > lever_rest[1][2] + 0.01,
        details=f"rest={lever_rest}, up={lever_up}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
