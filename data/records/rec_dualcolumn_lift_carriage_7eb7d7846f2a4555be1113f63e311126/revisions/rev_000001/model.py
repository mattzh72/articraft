from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


COLUMN_OFFSET_X = 0.19
COLUMN_SIZE = (0.072, 0.052, 1.58)
FOOT_SIZE = (0.12, 0.26, 0.04)
LOWER_TIE_SIZE = (0.48, 0.10, 0.09)
TOP_HEADER_SIZE = (0.48, 0.08, 0.08)

GUIDE_OUTER_W = 0.132
GUIDE_OUTER_D = 0.092
GUIDE_INNER_W = COLUMN_SIZE[0] + 0.014
GUIDE_INNER_D = COLUMN_SIZE[1] + 0.014
GUIDE_SLOT_W = 0.040
GUIDE_SLOT_D = 0.036
GUIDE_HEIGHT = 0.56

PLATE_W = 0.46
PLATE_H = 0.48
PLATE_T = 0.014
BRIDGE_SIZE = (0.26, 0.028, 0.048)
WEAR_PAD_SIZE = (GUIDE_INNER_W, 0.007, 0.26)

CARRIAGE_HOME_Z = 0.52
CARRIAGE_TRAVEL = 0.80


def _add_box(
    part,
    *,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material: str,
    name: str,
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def _guide_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").box(GUIDE_OUTER_W, GUIDE_OUTER_D, GUIDE_HEIGHT)
    inner = cq.Workplane("XY").box(GUIDE_INNER_W, GUIDE_INNER_D, GUIDE_HEIGHT + 0.006)
    rear_slot = (
        cq.Workplane("XY")
        .center(0.0, -(GUIDE_OUTER_D / 2.0 - GUIDE_SLOT_D / 2.0))
        .box(GUIDE_SLOT_W, GUIDE_SLOT_D, GUIDE_HEIGHT + 0.008)
    )
    return outer.cut(inner).cut(rear_slot)


def _front_plate_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(PLATE_W, PLATE_T, PLATE_H)
    slot_size = (0.060, PLATE_T + 0.006, 0.240)
    for slot_x in (-0.12, 0.12):
        plate = plate.cut(
            cq.Workplane("XY")
            .center(slot_x, 0.0)
            .box(slot_size[0], slot_size[1], slot_size[2])
        )

    stiffener = (
        cq.Workplane("XY")
        .center(0.0, PLATE_T / 2.0 + 0.008)
        .box(PLATE_W * 0.82, 0.016, 0.052)
    )
    return plate.union(stiffener)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_column_stacker_carriage")

    model.material("mast_steel", rgba=(0.32, 0.34, 0.37, 1.0))
    model.material("carriage_blue", rgba=(0.18, 0.42, 0.76, 1.0))
    model.material("guide_gray", rgba=(0.64, 0.67, 0.71, 1.0))
    model.material("wear_black", rgba=(0.10, 0.10, 0.11, 1.0))

    mast = model.part("mast")
    _add_box(
        mast,
        size=FOOT_SIZE,
        xyz=(-COLUMN_OFFSET_X, 0.0, FOOT_SIZE[2] / 2.0),
        material="mast_steel",
        name="left_foot",
    )
    _add_box(
        mast,
        size=FOOT_SIZE,
        xyz=(COLUMN_OFFSET_X, 0.0, FOOT_SIZE[2] / 2.0),
        material="mast_steel",
        name="right_foot",
    )
    _add_box(
        mast,
        size=LOWER_TIE_SIZE,
        xyz=(0.0, 0.0, 0.08),
        material="mast_steel",
        name="lower_tie",
    )
    _add_box(
        mast,
        size=COLUMN_SIZE,
        xyz=(-COLUMN_OFFSET_X, 0.0, 0.08 + COLUMN_SIZE[2] / 2.0),
        material="mast_steel",
        name="left_column",
    )
    _add_box(
        mast,
        size=COLUMN_SIZE,
        xyz=(COLUMN_OFFSET_X, 0.0, 0.08 + COLUMN_SIZE[2] / 2.0),
        material="mast_steel",
        name="right_column",
    )
    _add_box(
        mast,
        size=TOP_HEADER_SIZE,
        xyz=(0.0, 0.0, 1.70),
        material="mast_steel",
        name="top_header",
    )
    mast.inertial = Inertial.from_geometry(
        Box((0.50, 0.26, 1.74)),
        mass=68.0,
        origin=Origin(xyz=(0.0, 0.0, 0.87)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_guide_shape(), "left_guide"),
        origin=Origin(xyz=(-COLUMN_OFFSET_X, 0.0, 0.0)),
        material="guide_gray",
        name="left_guide",
    )
    carriage.visual(
        mesh_from_cadquery(_guide_shape(), "right_guide"),
        origin=Origin(xyz=(COLUMN_OFFSET_X, 0.0, 0.0)),
        material="guide_gray",
        name="right_guide",
    )
    carriage.visual(
        mesh_from_cadquery(_front_plate_shape(), "front_plate"),
        origin=Origin(xyz=(0.0, GUIDE_OUTER_D / 2.0 + PLATE_T / 2.0 - 0.001, 0.0)),
        material="carriage_blue",
        name="front_plate",
    )
    _add_box(
        carriage,
        size=BRIDGE_SIZE,
        xyz=(0.0, -0.006, 0.22),
        material="carriage_blue",
        name="top_bridge",
    )
    _add_box(
        carriage,
        size=BRIDGE_SIZE,
        xyz=(0.0, -0.006, -0.22),
        material="carriage_blue",
        name="bottom_bridge",
    )
    _add_box(
        carriage,
        size=WEAR_PAD_SIZE,
        xyz=(
            -COLUMN_OFFSET_X,
            GUIDE_INNER_D / 2.0 - WEAR_PAD_SIZE[1] / 2.0,
            0.0,
        ),
        material="wear_black",
        name="left_wear_pad",
    )
    _add_box(
        carriage,
        size=WEAR_PAD_SIZE,
        xyz=(
            COLUMN_OFFSET_X,
            GUIDE_INNER_D / 2.0 - WEAR_PAD_SIZE[1] / 2.0,
            0.0,
        ),
        material="wear_black",
        name="right_wear_pad",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.52, 0.11, GUIDE_HEIGHT)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.02, 0.0)),
    )

    model.articulation(
        "mast_to_carriage",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=CARRIAGE_TRAVEL,
            effort=1800.0,
            velocity=0.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    carriage = object_model.get_part("carriage")
    lift = object_model.get_articulation("mast_to_carriage")

    left_column = mast.get_visual("left_column")
    right_column = mast.get_visual("right_column")
    left_guide = carriage.get_visual("left_guide")
    right_guide = carriage.get_visual("right_guide")
    front_plate = carriage.get_visual("front_plate")

    ctx.expect_within(
        mast,
        carriage,
        axes="xy",
        inner_elem=left_column,
        outer_elem=left_guide,
        margin=0.0,
        name="left guide wraps around left column in plan",
    )
    ctx.expect_within(
        mast,
        carriage,
        axes="xy",
        inner_elem=right_column,
        outer_elem=right_guide,
        margin=0.0,
        name="right guide wraps around right column in plan",
    )
    ctx.expect_overlap(
        mast,
        carriage,
        axes="z",
        elem_a=left_column,
        elem_b=left_guide,
        min_overlap=0.50,
        name="left guide remains engaged on the left upright",
    )
    ctx.expect_overlap(
        mast,
        carriage,
        axes="z",
        elem_a=right_column,
        elem_b=right_guide,
        min_overlap=0.50,
        name="right guide remains engaged on the right upright",
    )
    ctx.expect_gap(
        carriage,
        mast,
        axis="y",
        min_gap=0.015,
        max_gap=0.050,
        positive_elem=front_plate,
        negative_elem=left_column,
        name="front plate stands forward of the uprights",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({lift: CARRIAGE_TRAVEL}):
        ctx.expect_within(
            mast,
            carriage,
            axes="xy",
            inner_elem=left_column,
            outer_elem=left_guide,
            margin=0.0,
            name="left guide stays aligned at full lift",
        )
        ctx.expect_within(
            mast,
            carriage,
            axes="xy",
            inner_elem=right_column,
            outer_elem=right_guide,
            margin=0.0,
            name="right guide stays aligned at full lift",
        )
        ctx.expect_overlap(
            mast,
            carriage,
            axes="z",
            elem_a=left_column,
            elem_b=left_guide,
            min_overlap=0.50,
            name="left guide retains insertion at full lift",
        )
        ctx.expect_overlap(
            mast,
            carriage,
            axes="z",
            elem_a=right_column,
            elem_b=right_guide,
            min_overlap=0.50,
            name="right guide retains insertion at full lift",
        )
        lifted_pos = ctx.part_world_position(carriage)

    ctx.check(
        "carriage lifts upward along the columns",
        rest_pos is not None
        and lifted_pos is not None
        and lifted_pos[2] > rest_pos[2] + 0.75,
        details=f"rest={rest_pos}, lifted={lifted_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
