from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)

BASE_L = 0.380
BASE_W = 0.240
BASE_T = 0.016

X_RAIL_LEN = 0.300
X_RAIL_W = 0.018
X_RAIL_H = 0.012
X_RAIL_Y = 0.070
X_RAIL_BED_W = 0.032
X_RAIL_BED_H = 0.008

X_BLOCK_X = 0.040
X_BLOCK_Y = 0.026
X_BLOCK_Z = 0.018
X_BLOCK_X_OFF = 0.040

X_DECK_X = 0.140
X_DECK_Y = 0.170
X_DECK_Z = 0.012

Y_RAIL_LEN = 0.220
Y_RAIL_W = 0.016
Y_RAIL_H = 0.010
Y_RAIL_X = 0.038

Y_BLOCK_X = 0.026
Y_BLOCK_Y = 0.040
Y_BLOCK_Z = 0.016
Y_BLOCK_Y_OFF = 0.040
Y_BLOCK_Z_OFF = -0.010

Y_PLATE_X = 0.120
Y_PLATE_Y = 0.110
Y_PLATE_Z = 0.012
Y_SADDLE_Z_OFF = 0.004

Z_TOWER_X = 0.120
Z_TOWER_Y = 0.016
Z_TOWER_Z = 0.260
Z_TOWER_Y_OFF = 0.032

Z_RAIL_X = 0.016
Z_RAIL_Y = 0.008
Z_RAIL_Z = 0.260
Z_RAIL_X_OFF = 0.034
Z_RAIL_Y_OFF = 0.020

Z_SLIDE_X = 0.110
Z_SLIDE_Y = 0.010
Z_SLIDE_Z = 0.110
Z_SLIDE_Y_OFF = -0.002

Z_BLOCK_X = 0.022
Z_BLOCK_Y = 0.016
Z_BLOCK_Z = 0.028
Z_BLOCK_Z_OFF = 0.025
Z_BLOCK_Y_OFF = 0.008

TOP_TABLE_X = 0.120
TOP_TABLE_Y = 0.090
TOP_TABLE_Z = 0.012
TOP_TABLE_Y_OFF = -0.060

TABLE_ARM_X = 0.060
TABLE_ARM_Y = 0.008
TABLE_ARM_Z = 0.030
TABLE_ARM_Y_OFF = -0.011
TABLE_ARM_Z_OFF = 0.040

X_FRAME_Z = BASE_T + X_RAIL_BED_H + X_RAIL_H + X_BLOCK_Z + X_DECK_Z / 2.0
Y_FRAME_Z = X_FRAME_Z + 0.034
Z_FRAME_Z = Z_SLIDE_Z / 2.0 + 0.041

X_TRAVEL = 0.150
Y_TRAVEL = 0.150
Z_TRAVEL = 0.120


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="xyz_cartesian_stage", assets=ASSETS)

    base_gray = model.material("base_gray", rgba=(0.28, 0.30, 0.33, 1.0))
    rail_steel = model.material("rail_steel", rgba=(0.68, 0.70, 0.74, 1.0))
    carriage_aluminum = model.material("carriage_aluminum", rgba=(0.78, 0.80, 0.82, 1.0))
    table_black = model.material("table_black", rgba=(0.10, 0.10, 0.11, 1.0))

    base = model.part("base")
    base.visual(
        Box((BASE_L, BASE_W, BASE_T)),
        origin=Origin(xyz=(0.0, 0.0, BASE_T / 2.0)),
        material=base_gray,
        name="base_plate",
    )
    for side, y in (("left", X_RAIL_Y), ("right", -X_RAIL_Y)):
        base.visual(
            Box((X_RAIL_LEN + 0.020, X_RAIL_BED_W, X_RAIL_BED_H)),
            origin=Origin(xyz=(0.0, y, BASE_T + X_RAIL_BED_H / 2.0)),
            material=base_gray,
            name=f"x_rail_bed_{side}",
        )
        base.visual(
            Box((X_RAIL_LEN, X_RAIL_W, X_RAIL_H)),
            origin=Origin(
                xyz=(0.0, y, BASE_T + X_RAIL_BED_H + X_RAIL_H / 2.0),
            ),
            material=rail_steel,
            name=f"x_rail_{side}",
        )

    x_carriage = model.part("x_carriage")
    x_carriage.visual(
        Box((X_DECK_X, X_DECK_Y, X_DECK_Z)),
        material=carriage_aluminum,
        name="x_bridge_plate",
    )
    for side, y in (("left", X_RAIL_Y), ("right", -X_RAIL_Y)):
        for pos, x in (("front", X_BLOCK_X_OFF), ("rear", -X_BLOCK_X_OFF)):
            x_carriage.visual(
                Box((X_BLOCK_X, X_BLOCK_Y, X_BLOCK_Z)),
                origin=Origin(xyz=(x, y, -(X_BLOCK_Z + X_DECK_Z) / 2.0)),
                material=carriage_aluminum,
                name=f"x_block_{side}_{pos}",
            )
    for side, x in (("left", -Y_RAIL_X), ("right", Y_RAIL_X)):
        x_carriage.visual(
            Box((Y_RAIL_W, Y_RAIL_LEN, Y_RAIL_H)),
            origin=Origin(xyz=(x, 0.0, X_DECK_Z / 2.0 + Y_RAIL_H / 2.0)),
            material=rail_steel,
            name=f"y_rail_{side}",
        )

    y_stage = model.part("y_stage")
    y_stage.visual(
        Box((Y_PLATE_X, Y_PLATE_Y, Y_PLATE_Z)),
        origin=Origin(xyz=(0.0, 0.0, Y_SADDLE_Z_OFF)),
        material=carriage_aluminum,
        name="y_saddle_plate",
    )
    for side, x in (("left", -Y_RAIL_X), ("right", Y_RAIL_X)):
        for pos, y in (("front", Y_BLOCK_Y_OFF), ("rear", -Y_BLOCK_Y_OFF)):
            y_stage.visual(
                Box((Y_BLOCK_X, Y_BLOCK_Y, Y_BLOCK_Z)),
                origin=Origin(xyz=(x, y, Y_BLOCK_Z_OFF)),
                material=carriage_aluminum,
                name=f"y_block_{side}_{pos}",
            )
    y_stage.visual(
        Box((Z_TOWER_X, Z_TOWER_Y, Z_TOWER_Z)),
        origin=Origin(
            xyz=(0.0, Z_TOWER_Y_OFF, Y_SADDLE_Z_OFF + Y_PLATE_Z / 2.0 + Z_TOWER_Z / 2.0),
        ),
        material=base_gray,
        name="z_backplate",
    )
    for side, x in (("left", -Z_RAIL_X_OFF), ("right", Z_RAIL_X_OFF)):
        y_stage.visual(
            Box((Z_RAIL_X, Z_RAIL_Y, Z_RAIL_Z)),
            origin=Origin(
                xyz=(x, Z_RAIL_Y_OFF, Y_SADDLE_Z_OFF + Y_PLATE_Z / 2.0 + Z_RAIL_Z / 2.0),
            ),
            material=rail_steel,
            name=f"z_rail_{side}",
        )

    z_slide = model.part("z_slide")
    z_slide.visual(
        Box((Z_SLIDE_X, Z_SLIDE_Y, Z_SLIDE_Z)),
        origin=Origin(xyz=(0.0, Z_SLIDE_Y_OFF, 0.0)),
        material=carriage_aluminum,
        name="z_slide_plate",
    )
    for side, x in (("left", -Z_RAIL_X_OFF), ("right", Z_RAIL_X_OFF)):
        for pos, z in (("upper", Z_BLOCK_Z_OFF), ("lower", -Z_BLOCK_Z_OFF)):
            z_slide.visual(
                Box((Z_BLOCK_X, Z_BLOCK_Y, Z_BLOCK_Z)),
                origin=Origin(xyz=(x, Z_BLOCK_Y_OFF, z)),
                material=carriage_aluminum,
                name=f"z_block_{side}_{pos}",
            )
    z_slide.visual(
        Box((TOP_TABLE_X, TOP_TABLE_Y, TOP_TABLE_Z)),
        origin=Origin(
            xyz=(0.0, TOP_TABLE_Y_OFF, Z_SLIDE_Z / 2.0 + TOP_TABLE_Z / 2.0),
        ),
        material=table_black,
        name="top_table",
    )
    z_slide.visual(
        Box((TABLE_ARM_X, TABLE_ARM_Y, TABLE_ARM_Z)),
        origin=Origin(xyz=(0.0, TABLE_ARM_Y_OFF, TABLE_ARM_Z_OFF)),
        material=carriage_aluminum,
        name="table_arm",
    )

    model.articulation(
        "base_to_x",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_carriage,
        origin=Origin(xyz=(0.0, 0.0, X_FRAME_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.25,
            lower=-X_TRAVEL / 2.0,
            upper=X_TRAVEL / 2.0,
        ),
    )
    model.articulation(
        "x_to_y",
        ArticulationType.PRISMATIC,
        parent=x_carriage,
        child=y_stage,
        origin=Origin(xyz=(0.0, 0.0, Y_FRAME_Z - X_FRAME_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=100.0,
            velocity=0.25,
            lower=-Y_TRAVEL / 2.0,
            upper=Y_TRAVEL / 2.0,
        ),
    )
    model.articulation(
        "y_to_z",
        ArticulationType.PRISMATIC,
        parent=y_stage,
        child=z_slide,
        origin=Origin(xyz=(0.0, 0.0, Z_FRAME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.20,
            lower=0.0,
            upper=Z_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    x_carriage = object_model.get_part("x_carriage")
    y_stage = object_model.get_part("y_stage")
    z_slide = object_model.get_part("z_slide")

    x_joint = object_model.get_articulation("base_to_x")
    y_joint = object_model.get_articulation("x_to_y")
    z_joint = object_model.get_articulation("y_to_z")

    base_plate = base.get_visual("base_plate")
    x_rail_left = base.get_visual("x_rail_left")
    x_rail_right = base.get_visual("x_rail_right")

    x_bridge = x_carriage.get_visual("x_bridge_plate")
    x_block_left_front = x_carriage.get_visual("x_block_left_front")
    x_block_right_front = x_carriage.get_visual("x_block_right_front")
    y_rail_left = x_carriage.get_visual("y_rail_left")
    y_rail_right = x_carriage.get_visual("y_rail_right")

    y_saddle = y_stage.get_visual("y_saddle_plate")
    y_block_left_front = y_stage.get_visual("y_block_left_front")
    y_block_right_front = y_stage.get_visual("y_block_right_front")
    z_backplate = y_stage.get_visual("z_backplate")
    z_rail_left = y_stage.get_visual("z_rail_left")
    z_rail_right = y_stage.get_visual("z_rail_right")

    z_slide_plate = z_slide.get_visual("z_slide_plate")
    z_block_left_lower = z_slide.get_visual("z_block_left_lower")
    z_block_right_lower = z_slide.get_visual("z_block_right_lower")
    top_table = z_slide.get_visual("top_table")

    def _close(a: float, b: float, tol: float = 1e-6) -> bool:
        return abs(a - b) <= tol

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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

    ctx.check(
        "all stage parts present",
        all(part is not None for part in (base, x_carriage, y_stage, z_slide)),
        "Expected base, x_carriage, y_stage, and z_slide parts.",
    )
    ctx.check(
        "prismatic axes are orthogonal and vertical",
        tuple(x_joint.axis) == (1.0, 0.0, 0.0)
        and tuple(y_joint.axis) == (0.0, 1.0, 0.0)
        and tuple(z_joint.axis) == (0.0, 0.0, 1.0),
        f"Axes were x={x_joint.axis}, y={y_joint.axis}, z={z_joint.axis}.",
    )
    ctx.check(
        "travel ranges match requested strokes",
        _close(x_joint.motion_limits.upper - x_joint.motion_limits.lower, X_TRAVEL)
        and _close(y_joint.motion_limits.upper - y_joint.motion_limits.lower, Y_TRAVEL)
        and _close(z_joint.motion_limits.upper - z_joint.motion_limits.lower, Z_TRAVEL),
        "Expected 150 mm X, 150 mm Y, and 120 mm Z stroke.",
    )

    ctx.expect_contact(
        x_carriage,
        base,
        elem_a=x_block_left_front,
        elem_b=x_rail_left,
        name="x carriage left bearing contacts left X rail",
    )
    ctx.expect_contact(
        x_carriage,
        base,
        elem_a=x_block_right_front,
        elem_b=x_rail_right,
        name="x carriage right bearing contacts right X rail",
    )
    ctx.expect_contact(
        y_stage,
        x_carriage,
        elem_a=y_block_left_front,
        elem_b=y_rail_left,
        name="y stage left bearing contacts left Y rail",
    )
    ctx.expect_contact(
        y_stage,
        x_carriage,
        elem_a=y_block_right_front,
        elem_b=y_rail_right,
        name="y stage right bearing contacts right Y rail",
    )
    ctx.expect_contact(
        z_slide,
        y_stage,
        elem_a=z_block_left_lower,
        elem_b=z_rail_left,
        name="z slide left bearing contacts left Z rail",
    )
    ctx.expect_contact(
        z_slide,
        y_stage,
        elem_a=z_block_right_lower,
        elem_b=z_rail_right,
        name="z slide right bearing contacts right Z rail",
    )

    ctx.expect_overlap(
        x_carriage,
        base,
        axes="xy",
        elem_a=x_bridge,
        elem_b=base_plate,
        min_overlap=0.120,
        name="x carriage remains visibly supported by base footprint",
    )
    ctx.expect_overlap(
        y_stage,
        x_carriage,
        axes="xy",
        elem_a=y_saddle,
        elem_b=x_bridge,
        min_overlap=0.060,
        name="y stage saddle overlaps x bridge at rest",
    )
    ctx.expect_within(
        z_slide,
        y_stage,
        axes="x",
        inner_elem=z_slide_plate,
        outer_elem=z_backplate,
        margin=0.010,
        name="z slide stays within tower width",
    )
    ctx.expect_gap(
        z_slide,
        y_stage,
        axis="z",
        positive_elem=top_table,
        negative_elem=y_saddle,
        min_gap=0.040,
        name="top table sits above y saddle",
    )

    x_rest = ctx.part_world_position(x_carriage)
    y_rest = ctx.part_world_position(y_stage)
    z_rest = ctx.part_world_position(z_slide)

    with ctx.pose({x_joint: X_TRAVEL / 2.0}):
        x_hi = ctx.part_world_position(x_carriage)
        ctx.check(
            "x carriage moves 75 mm on +X with no YZ drift",
            _close(x_hi[0] - x_rest[0], X_TRAVEL / 2.0)
            and _close(x_hi[1], x_rest[1])
            and _close(x_hi[2], x_rest[2]),
            f"Rest={x_rest}, +X={x_hi}",
        )
        ctx.expect_within(
            x_carriage,
            base,
            axes="x",
            inner_elem=x_bridge,
            outer_elem=base_plate,
            margin=0.0,
            name="x carriage remains within base length at +X limit",
        )

    with ctx.pose({x_joint: -X_TRAVEL / 2.0}):
        ctx.expect_within(
            x_carriage,
            base,
            axes="x",
            inner_elem=x_bridge,
            outer_elem=base_plate,
            margin=0.0,
            name="x carriage remains within base length at -X limit",
        )

    with ctx.pose({y_joint: Y_TRAVEL / 2.0}):
        y_hi = ctx.part_world_position(y_stage)
        ctx.check(
            "y stage moves 75 mm on +Y with no XZ drift",
            _close(y_hi[0], y_rest[0])
            and _close(y_hi[1] - y_rest[1], Y_TRAVEL / 2.0)
            and _close(y_hi[2], y_rest[2]),
            f"Rest={y_rest}, +Y={y_hi}",
        )
        ctx.expect_overlap(
            y_stage,
            x_carriage,
            axes="xy",
            elem_a=y_saddle,
            elem_b=x_bridge,
            min_overlap=0.035,
            name="y saddle still overlaps x bridge at +Y limit",
        )
        ctx.expect_contact(
            z_slide,
            y_stage,
            elem_a=z_block_left_lower,
            elem_b=z_rail_left,
            name="z left bearing still contacts rail at shifted +Y pose",
        )

    with ctx.pose({y_joint: -Y_TRAVEL / 2.0}):
        ctx.expect_overlap(
            y_stage,
            x_carriage,
            axes="xy",
            elem_a=y_saddle,
            elem_b=x_bridge,
            min_overlap=0.035,
            name="y saddle still overlaps x bridge at -Y limit",
        )

    with ctx.pose({z_joint: Z_TRAVEL}):
        z_hi = ctx.part_world_position(z_slide)
        ctx.check(
            "z slide rises 120 mm on +Z with no XY drift",
            _close(z_hi[0], z_rest[0])
            and _close(z_hi[1], z_rest[1])
            and _close(z_hi[2] - z_rest[2], Z_TRAVEL),
            f"Rest={z_rest}, +Z={z_hi}",
        )
        ctx.expect_contact(
            z_slide,
            y_stage,
            elem_a=z_block_right_lower,
            elem_b=z_rail_right,
            name="z right bearing still contacts rail at top of travel",
        )
        ctx.expect_gap(
            z_slide,
            y_stage,
            axis="z",
            positive_elem=top_table,
            negative_elem=y_saddle,
            min_gap=0.140,
            name="top table is substantially raised at top of travel",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
