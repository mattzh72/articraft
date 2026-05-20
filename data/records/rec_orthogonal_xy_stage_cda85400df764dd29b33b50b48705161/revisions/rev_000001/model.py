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


BASE_FOOT_LENGTH = 0.34
BASE_FOOT_WIDTH = 0.22
BASE_FOOT_HEIGHT = 0.03

BASE_DECK_LENGTH = 0.30
BASE_DECK_WIDTH = 0.15
BASE_DECK_HEIGHT = 0.018
BASE_DECK_TOP = BASE_FOOT_HEIGHT + BASE_DECK_HEIGHT

X_RAIL_LENGTH = 0.24
X_RAIL_WIDTH = 0.024
X_RAIL_HEIGHT = 0.014
X_RAIL_OFFSET_Y = 0.052
BASE_TO_LOWER_Z = BASE_DECK_TOP + X_RAIL_HEIGHT
X_TRAVEL = 0.06

LOWER_PAD_LENGTH = 0.18
LOWER_PAD_WIDTH = 0.032
LOWER_PAD_HEIGHT = 0.022
LOWER_BRIDGE_WIDTH = 0.15
LOWER_BRIDGE_HEIGHT = 0.016
LOWER_CROWN_LENGTH = 0.11
LOWER_CROWN_WIDTH = 0.095
LOWER_CROWN_HEIGHT = 0.006
LOWER_BRIDGE_TOP = LOWER_PAD_HEIGHT + LOWER_BRIDGE_HEIGHT

Y_RAIL_LENGTH = 0.16
Y_RAIL_WIDTH = 0.02
Y_RAIL_HEIGHT = 0.012
Y_RAIL_OFFSET_X = 0.045
LOWER_TO_UPPER_Z = LOWER_BRIDGE_TOP + Y_RAIL_HEIGHT
Y_TRAVEL = 0.04

UPPER_PAD_LENGTH_X = 0.03
UPPER_PAD_LENGTH_Y = 0.13
UPPER_PAD_HEIGHT = 0.02
UPPER_BRIDGE_LENGTH_X = 0.14
UPPER_BRIDGE_LENGTH_Y = 0.13
UPPER_BRIDGE_HEIGHT = 0.014
UPPER_MOUNT_LENGTH = 0.10
UPPER_MOUNT_WIDTH = 0.09
UPPER_MOUNT_HEIGHT = 0.012
UPPER_BRIDGE_TOP = UPPER_PAD_HEIGHT + UPPER_BRIDGE_HEIGHT
UPPER_TO_TOP_Z = UPPER_BRIDGE_TOP + UPPER_MOUNT_HEIGHT

TOP_PLATE_LENGTH = 0.18
TOP_PLATE_WIDTH = 0.12
TOP_PLATE_HEIGHT = 0.012
TOP_SLOT_LENGTH = 0.145
TOP_SLOT_WIDTH = 0.01
TOP_SLOT_OFFSETS_Y = (-0.034, 0.0, 0.034)


def _box_on_floor(length: float, width: float, height: float) -> cq.Workplane:
    return cq.Workplane("XY").box(length, width, height, centered=(True, True, False))


def _make_base_body() -> cq.Workplane:
    foot = _box_on_floor(BASE_FOOT_LENGTH, BASE_FOOT_WIDTH, BASE_FOOT_HEIGHT)
    deck = _box_on_floor(BASE_DECK_LENGTH, BASE_DECK_WIDTH, BASE_DECK_HEIGHT).translate(
        (0.0, 0.0, BASE_FOOT_HEIGHT - 0.001)
    )
    body = foot.union(deck)
    front_lip = _box_on_floor(0.19, 0.02, 0.008).translate(
        (0.0, BASE_DECK_WIDTH * 0.5 - 0.01, BASE_DECK_TOP - 0.001)
    )
    rear_lip = _box_on_floor(0.19, 0.02, 0.008).translate(
        (0.0, -BASE_DECK_WIDTH * 0.5 + 0.01, BASE_DECK_TOP - 0.001)
    )
    return body.union(front_lip).union(rear_lip)


def _make_lower_slide_body() -> cq.Workplane:
    left_pad = _box_on_floor(LOWER_PAD_LENGTH, LOWER_PAD_WIDTH, LOWER_PAD_HEIGHT).translate(
        (0.0, X_RAIL_OFFSET_Y, 0.0)
    )
    right_pad = _box_on_floor(LOWER_PAD_LENGTH, LOWER_PAD_WIDTH, LOWER_PAD_HEIGHT).translate(
        (0.0, -X_RAIL_OFFSET_Y, 0.0)
    )
    bridge = _box_on_floor(
        LOWER_PAD_LENGTH,
        LOWER_BRIDGE_WIDTH,
        LOWER_BRIDGE_HEIGHT,
    ).translate((0.0, 0.0, LOWER_PAD_HEIGHT))
    crown = _box_on_floor(
        LOWER_CROWN_LENGTH,
        LOWER_CROWN_WIDTH,
        LOWER_CROWN_HEIGHT,
    ).translate((0.0, 0.0, LOWER_BRIDGE_TOP))
    return left_pad.union(right_pad).union(bridge).union(crown)


def _make_upper_slide_body() -> cq.Workplane:
    left_pad = _box_on_floor(
        UPPER_PAD_LENGTH_X,
        UPPER_PAD_LENGTH_Y,
        UPPER_PAD_HEIGHT,
    ).translate((-Y_RAIL_OFFSET_X, 0.0, 0.0))
    right_pad = _box_on_floor(
        UPPER_PAD_LENGTH_X,
        UPPER_PAD_LENGTH_Y,
        UPPER_PAD_HEIGHT,
    ).translate((Y_RAIL_OFFSET_X, 0.0, 0.0))
    bridge = _box_on_floor(
        UPPER_BRIDGE_LENGTH_X,
        UPPER_BRIDGE_LENGTH_Y,
        UPPER_BRIDGE_HEIGHT,
    ).translate((0.0, 0.0, UPPER_PAD_HEIGHT))
    mount = _box_on_floor(
        UPPER_MOUNT_LENGTH,
        UPPER_MOUNT_WIDTH,
        UPPER_MOUNT_HEIGHT,
    ).translate((0.0, 0.0, UPPER_BRIDGE_TOP))
    return left_pad.union(right_pad).union(bridge).union(mount)


def _make_top_plate() -> cq.Workplane:
    plate = _box_on_floor(TOP_PLATE_LENGTH, TOP_PLATE_WIDTH, TOP_PLATE_HEIGHT)
    for slot_y in TOP_SLOT_OFFSETS_Y:
        cutter = _box_on_floor(
            TOP_SLOT_LENGTH,
            TOP_SLOT_WIDTH,
            TOP_PLATE_HEIGHT + 0.002,
        ).translate((0.0, slot_y, -0.001))
        plate = plate.cut(cutter)
    return plate


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cross_slide_xy_table")

    model.material("cast_iron", rgba=(0.35, 0.36, 0.38, 1.0))
    model.material("way_steel", rgba=(0.64, 0.66, 0.69, 1.0))
    model.material("slide_gray", rgba=(0.52, 0.55, 0.59, 1.0))
    model.material("machined_plate", rgba=(0.79, 0.81, 0.84, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base_body(), "base_body"),
        material="cast_iron",
        name="base_body",
    )
    base.visual(
        Box((X_RAIL_LENGTH, X_RAIL_WIDTH, X_RAIL_HEIGHT)),
        origin=Origin(
            xyz=(0.0, X_RAIL_OFFSET_Y, BASE_DECK_TOP + X_RAIL_HEIGHT * 0.5)
        ),
        material="way_steel",
        name="x_rail_left",
    )
    base.visual(
        Box((X_RAIL_LENGTH, X_RAIL_WIDTH, X_RAIL_HEIGHT)),
        origin=Origin(
            xyz=(0.0, -X_RAIL_OFFSET_Y, BASE_DECK_TOP + X_RAIL_HEIGHT * 0.5)
        ),
        material="way_steel",
        name="x_rail_right",
    )

    lower_slide = model.part("lower_slide")
    lower_slide.visual(
        mesh_from_cadquery(_make_lower_slide_body(), "lower_slide_body"),
        material="slide_gray",
        name="lower_slide_body",
    )
    lower_slide.visual(
        Box((Y_RAIL_WIDTH, Y_RAIL_LENGTH, Y_RAIL_HEIGHT)),
        origin=Origin(
            xyz=(Y_RAIL_OFFSET_X, 0.0, LOWER_BRIDGE_TOP + Y_RAIL_HEIGHT * 0.5)
        ),
        material="way_steel",
        name="y_rail_right",
    )
    lower_slide.visual(
        Box((Y_RAIL_WIDTH, Y_RAIL_LENGTH, Y_RAIL_HEIGHT)),
        origin=Origin(
            xyz=(-Y_RAIL_OFFSET_X, 0.0, LOWER_BRIDGE_TOP + Y_RAIL_HEIGHT * 0.5)
        ),
        material="way_steel",
        name="y_rail_left",
    )

    upper_slide = model.part("upper_slide")
    upper_slide.visual(
        mesh_from_cadquery(_make_upper_slide_body(), "upper_slide_body"),
        material="slide_gray",
        name="upper_slide_body",
    )

    top_plate = model.part("top_plate")
    top_plate.visual(
        mesh_from_cadquery(_make_top_plate(), "top_plate"),
        material="machined_plate",
        name="plate",
    )

    model.articulation(
        "base_to_lower",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lower_slide,
        origin=Origin(xyz=(0.0, 0.0, BASE_TO_LOWER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-X_TRAVEL,
            upper=X_TRAVEL,
            effort=600.0,
            velocity=0.20,
        ),
    )
    model.articulation(
        "lower_to_upper",
        ArticulationType.PRISMATIC,
        parent=lower_slide,
        child=upper_slide,
        origin=Origin(xyz=(0.0, 0.0, LOWER_TO_UPPER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-Y_TRAVEL,
            upper=Y_TRAVEL,
            effort=400.0,
            velocity=0.20,
        ),
    )
    model.articulation(
        "upper_to_top_plate",
        ArticulationType.FIXED,
        parent=upper_slide,
        child=top_plate,
        origin=Origin(xyz=(0.0, 0.0, UPPER_TO_TOP_Z)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lower_slide = object_model.get_part("lower_slide")
    upper_slide = object_model.get_part("upper_slide")
    top_plate = object_model.get_part("top_plate")
    base_to_lower = object_model.get_articulation("base_to_lower")
    lower_to_upper = object_model.get_articulation("lower_to_upper")
    upper_to_top_plate = object_model.get_articulation("upper_to_top_plate")

    ctx.check(
        "base-to-lower is X prismatic",
        base_to_lower.articulation_type == ArticulationType.PRISMATIC
        and tuple(base_to_lower.axis) == (1.0, 0.0, 0.0),
        details=f"type={base_to_lower.articulation_type}, axis={base_to_lower.axis}",
    )
    ctx.check(
        "lower-to-upper is Y prismatic",
        lower_to_upper.articulation_type == ArticulationType.PRISMATIC
        and tuple(lower_to_upper.axis) == (0.0, 1.0, 0.0),
        details=f"type={lower_to_upper.articulation_type}, axis={lower_to_upper.axis}",
    )
    ctx.check(
        "upper-to-top plate is fixed",
        upper_to_top_plate.articulation_type == ArticulationType.FIXED,
        details=f"type={upper_to_top_plate.articulation_type}",
    )

    with ctx.pose({base_to_lower: 0.0, lower_to_upper: 0.0}):
        ctx.expect_gap(
            lower_slide,
            base,
            axis="z",
            negative_elem="x_rail_left",
            max_gap=0.0005,
            max_penetration=0.0,
            name="lower slide seats on left X rail",
        )
        ctx.expect_gap(
            upper_slide,
            lower_slide,
            axis="z",
            negative_elem="y_rail_left",
            max_gap=0.0005,
            max_penetration=0.0,
            name="upper slide seats on left Y rail",
        )
        ctx.expect_gap(
            top_plate,
            upper_slide,
            axis="z",
            max_gap=0.0005,
            max_penetration=0.0,
            name="top plate sits on upper slide mounting pad",
        )
        ctx.expect_overlap(
            lower_slide,
            base,
            axes="x",
            elem_b="x_rail_left",
            min_overlap=0.16,
            name="lower slide remains inserted on X rails at center",
        )
        ctx.expect_overlap(
            upper_slide,
            lower_slide,
            axes="y",
            elem_b="y_rail_left",
            min_overlap=0.12,
            name="upper slide remains inserted on Y rails at center",
        )
        ctx.expect_overlap(
            top_plate,
            upper_slide,
            axes="xy",
            min_overlap=0.08,
            name="top plate is centered over upper slide mount",
        )

    lower_rest = ctx.part_world_position(lower_slide)
    with ctx.pose({base_to_lower: X_TRAVEL, lower_to_upper: 0.0}):
        ctx.expect_overlap(
            lower_slide,
            base,
            axes="x",
            elem_b="x_rail_left",
            min_overlap=0.10,
            name="lower slide keeps X-rail engagement at max travel",
        )
        lower_extended = ctx.part_world_position(lower_slide)
    ctx.check(
        "lower slide moves along +X",
        lower_rest is not None
        and lower_extended is not None
        and lower_extended[0] > lower_rest[0] + 0.03,
        details=f"rest={lower_rest}, extended={lower_extended}",
    )

    upper_rest = ctx.part_world_position(upper_slide)
    with ctx.pose({base_to_lower: 0.0, lower_to_upper: Y_TRAVEL}):
        ctx.expect_overlap(
            upper_slide,
            lower_slide,
            axes="y",
            elem_b="y_rail_left",
            min_overlap=0.08,
            name="upper slide keeps Y-rail engagement at max travel",
        )
        upper_extended = ctx.part_world_position(upper_slide)
    ctx.check(
        "upper slide moves along +Y",
        upper_rest is not None
        and upper_extended is not None
        and upper_extended[1] > upper_rest[1] + 0.02,
        details=f"rest={upper_rest}, extended={upper_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
