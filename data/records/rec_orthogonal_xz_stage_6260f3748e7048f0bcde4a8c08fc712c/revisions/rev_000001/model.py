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


BASE_LENGTH = 0.62
BASE_WIDTH = 0.23
BASE_THICKNESS = 0.028
RAIL_LENGTH = 0.54
RAIL_WIDTH = 0.028
RAIL_HEIGHT = 0.022
RAIL_OFFSET = 0.073
CENTER_SPINE_LENGTH = 0.42
CENTER_SPINE_WIDTH = 0.048
CENTER_SPINE_HEIGHT = 0.010

X_TRAVEL = 0.15
X_CARRIAGE_LENGTH = 0.16
X_CARRIAGE_WIDTH = 0.18
X_SHOE_WIDTH = 0.034
X_SHOE_HEIGHT = 0.016
X_BODY_HEIGHT = 0.036
X_COLUMN_WIDTH = 0.102
X_COLUMN_DEPTH = 0.042
X_COLUMN_HEIGHT = 0.44

Z_GUIDE_WIDTH = 0.018
Z_GUIDE_DEPTH = 0.008
Z_GUIDE_HEIGHT = 0.34
Z_GUIDE_X = 0.031
Z_GUIDE_BOTTOM = 0.09

Z_TRAVEL = 0.09
Z_CARRIAGE_WIDTH = 0.13
Z_CARRIAGE_DEPTH = 0.036
Z_CARRIAGE_HEIGHT = 0.13
Z_REAR_RELIEF_WIDTH = 0.058
Z_REAR_RELIEF_DEPTH = 0.014
Z_RISER_WIDTH = 0.052
Z_RISER_DEPTH = 0.048
Z_RISER_HEIGHT = 0.10
TOP_PLATE_LENGTH = 0.16
TOP_PLATE_WIDTH = 0.11
TOP_PLATE_THICKNESS = 0.012
TOP_PLATE_Y = 0.06

X_RAIL_TOP_Z = BASE_THICKNESS + RAIL_HEIGHT
X_TOTAL_BODY_HEIGHT = X_SHOE_HEIGHT + X_BODY_HEIGHT
Z_GUIDE_FACE_Y = (X_COLUMN_DEPTH / 2.0) + Z_GUIDE_DEPTH
Z_HOME = 0.245
Z_RISER_BASE = Z_CARRIAGE_HEIGHT / 2.0
TOP_PLATE_BOTTOM = Z_RISER_BASE + Z_RISER_HEIGHT


def _box(
    size: tuple[float, float, float],
    center: tuple[float, float, float],
) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _base_body_shape() -> cq.Workplane:
    body = _box(
        (BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS),
        (0.0, 0.0, BASE_THICKNESS / 2.0),
    )
    body = body.edges("|Z").fillet(0.012)
    return body.cut(
        _box(
            (BASE_LENGTH * 0.68, 0.076, 0.008),
            (0.0, 0.0, BASE_THICKNESS - 0.004),
        )
    )


def _base_rails_shape() -> cq.Workplane:
    left_rail = _box(
        (RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT),
        (0.0, -RAIL_OFFSET, BASE_THICKNESS + (RAIL_HEIGHT / 2.0)),
    )
    right_rail = _box(
        (RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT),
        (0.0, RAIL_OFFSET, BASE_THICKNESS + (RAIL_HEIGHT / 2.0)),
    )
    spine = _box(
        (CENTER_SPINE_LENGTH, CENTER_SPINE_WIDTH, CENTER_SPINE_HEIGHT),
        (0.0, 0.0, (BASE_THICKNESS - 0.008) + (CENTER_SPINE_HEIGHT / 2.0)),
    )
    return left_rail.union(right_rail).union(spine)


def _x_saddle_shape() -> cq.Workplane:
    carriage = _box(
        (X_CARRIAGE_LENGTH, X_CARRIAGE_WIDTH, X_TOTAL_BODY_HEIGHT),
        (0.0, 0.0, X_TOTAL_BODY_HEIGHT / 2.0),
    )
    carriage = carriage.cut(
        _box(
            (
                X_CARRIAGE_LENGTH - 0.034,
                X_CARRIAGE_WIDTH - (2.0 * X_SHOE_WIDTH),
                X_SHOE_HEIGHT + 0.010,
            ),
            (0.0, 0.0, (X_SHOE_HEIGHT + 0.010) / 2.0),
        )
    )
    carriage = carriage.edges("|Z").fillet(0.008)

    buttress = _box(
        (0.080, 0.070, 0.070),
        (0.0, -0.004, X_TOTAL_BODY_HEIGHT + 0.035),
    )
    return carriage.union(buttress)


def _x_upright_shape() -> cq.Workplane:
    column = _box(
        (X_COLUMN_WIDTH, X_COLUMN_DEPTH, X_COLUMN_HEIGHT),
        (0.0, 0.0, X_TOTAL_BODY_HEIGHT + (X_COLUMN_HEIGHT / 2.0)),
    )
    return column.edges("|Z").fillet(0.006)


def _z_guides_shape() -> cq.Workplane:
    left_guide = _box(
        (Z_GUIDE_WIDTH, Z_GUIDE_DEPTH, Z_GUIDE_HEIGHT),
        (
            -Z_GUIDE_X,
            (X_COLUMN_DEPTH / 2.0) + (Z_GUIDE_DEPTH / 2.0),
            Z_GUIDE_BOTTOM + (Z_GUIDE_HEIGHT / 2.0),
        ),
    )
    right_guide = _box(
        (Z_GUIDE_WIDTH, Z_GUIDE_DEPTH, Z_GUIDE_HEIGHT),
        (
            Z_GUIDE_X,
            (X_COLUMN_DEPTH / 2.0) + (Z_GUIDE_DEPTH / 2.0),
            Z_GUIDE_BOTTOM + (Z_GUIDE_HEIGHT / 2.0),
        ),
    )
    return left_guide.union(right_guide)


def _z_body_shape() -> cq.Workplane:
    body = _box(
        (Z_CARRIAGE_WIDTH, Z_CARRIAGE_DEPTH, Z_CARRIAGE_HEIGHT),
        (0.0, Z_CARRIAGE_DEPTH / 2.0, 0.0),
    )
    body = body.cut(
        _box(
            (Z_REAR_RELIEF_WIDTH, Z_REAR_RELIEF_DEPTH, Z_CARRIAGE_HEIGHT - 0.028),
            (0.0, Z_REAR_RELIEF_DEPTH / 2.0, 0.0),
        )
    )
    riser = _box(
        (Z_RISER_WIDTH, Z_RISER_DEPTH, Z_RISER_HEIGHT),
        (0.0, 0.040, Z_RISER_BASE + (Z_RISER_HEIGHT / 2.0)),
    )
    return body.union(riser).edges("|Z").fillet(0.005)


def _top_plate_shape() -> cq.Workplane:
    plate = _box(
        (TOP_PLATE_LENGTH, TOP_PLATE_WIDTH, TOP_PLATE_THICKNESS),
        (
            0.0,
            TOP_PLATE_Y,
            TOP_PLATE_BOTTOM + (TOP_PLATE_THICKNESS / 2.0),
        ),
    )
    return plate.edges("|Z").fillet(0.008)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="xz_positioning_stage")

    model.material("base_black", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("machined_aluminum", rgba=(0.73, 0.75, 0.79, 1.0))
    model.material("rail_steel", rgba=(0.57, 0.60, 0.64, 1.0))
    model.material("plate_blue", rgba=(0.24, 0.36, 0.67, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_body_shape(), "base_body"),
        material="base_black",
        name="base_body",
    )
    base.visual(
        mesh_from_cadquery(_base_rails_shape(), "base_rails"),
        material="rail_steel",
        name="base_rails",
    )

    x_stage = model.part("x_stage")
    x_stage.visual(
        mesh_from_cadquery(_x_saddle_shape(), "x_saddle"),
        material="machined_aluminum",
        name="x_saddle",
    )
    x_stage.visual(
        mesh_from_cadquery(_x_upright_shape(), "x_upright"),
        material="machined_aluminum",
        name="x_upright",
    )
    x_stage.visual(
        mesh_from_cadquery(_z_guides_shape(), "z_guides"),
        material="rail_steel",
        name="z_guides",
    )

    z_stage = model.part("z_stage")
    z_stage.visual(
        mesh_from_cadquery(_z_body_shape(), "z_body"),
        material="machined_aluminum",
        name="z_body",
    )
    z_stage.visual(
        mesh_from_cadquery(_top_plate_shape(), "top_plate"),
        material="plate_blue",
        name="top_plate",
    )

    model.articulation(
        "base_to_x",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_stage,
        origin=Origin(xyz=(0.0, 0.0, X_RAIL_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-X_TRAVEL,
            upper=X_TRAVEL,
            effort=160.0,
            velocity=0.35,
        ),
    )
    model.articulation(
        "x_to_z",
        ArticulationType.PRISMATIC,
        parent=x_stage,
        child=z_stage,
        origin=Origin(xyz=(0.0, Z_GUIDE_FACE_Y, Z_HOME)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-Z_TRAVEL,
            upper=Z_TRAVEL,
            effort=120.0,
            velocity=0.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    x_stage = object_model.get_part("x_stage")
    z_stage = object_model.get_part("z_stage")
    x_axis = object_model.get_articulation("base_to_x")
    z_axis = object_model.get_articulation("x_to_z")

    ctx.expect_contact(
        x_stage,
        base,
        elem_a="x_saddle",
        elem_b="base_rails",
        name="x carriage sits on the horizontal rails",
    )
    ctx.expect_contact(
        z_stage,
        x_stage,
        elem_a="z_body",
        elem_b="z_guides",
        name="z carriage rides on the upright guide rails",
    )
    ctx.expect_gap(
        z_stage,
        x_stage,
        axis="y",
        min_gap=0.006,
        positive_elem="z_body",
        negative_elem="x_upright",
        name="z carriage clears the upright backing plate",
    )

    x_rest = ctx.part_world_position(x_stage)
    with ctx.pose({x_axis: X_TRAVEL}):
        ctx.expect_contact(
            x_stage,
            base,
            elem_a="x_saddle",
            elem_b="base_rails",
            name="x carriage stays supported at positive travel",
        )
        ctx.expect_overlap(
            x_stage,
            base,
            axes="x",
            elem_a="x_saddle",
            elem_b="base_rails",
            min_overlap=0.10,
            name="x carriage retains rail engagement at positive travel",
        )
        x_extended = ctx.part_world_position(x_stage)

    z_rest = ctx.part_world_position(z_stage)
    with ctx.pose({z_axis: Z_TRAVEL}):
        ctx.expect_contact(
            z_stage,
            x_stage,
            elem_a="z_body",
            elem_b="z_guides",
            name="z carriage stays supported at positive travel",
        )
        ctx.expect_overlap(
            z_stage,
            x_stage,
            axes="z",
            elem_a="z_body",
            elem_b="z_guides",
            min_overlap=0.12,
            name="z carriage retains guide engagement at positive travel",
        )
        ctx.expect_gap(
            z_stage,
            x_stage,
            axis="y",
            min_gap=0.006,
            positive_elem="z_body",
            negative_elem="x_upright",
            name="z carriage still clears the upright at positive travel",
        )
        z_extended = ctx.part_world_position(z_stage)

    ctx.check(
        "x axis moves along +X",
        x_rest is not None and x_extended is not None and x_extended[0] > x_rest[0] + 0.10,
        details=f"rest={x_rest}, extended={x_extended}",
    )
    ctx.check(
        "z axis moves upward",
        z_rest is not None and z_extended is not None and z_extended[2] > z_rest[2] + 0.06,
        details=f"rest={z_rest}, extended={z_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
