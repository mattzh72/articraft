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


PLATE_T = 0.014
PLATE_W = 0.220
PLATE_H = 0.300

RAIL_T = 0.018
RAIL_L = 0.150
RAIL_H = 0.030
RAIL_CENTER_X = (PLATE_T / 2.0) + (RAIL_T / 2.0)
RAIL_FRONT_X = (PLATE_T / 2.0) + RAIL_T
RAIL_Z = 0.080

Y_TRAVEL = 0.040
Z_TRAVEL = 0.065

CARRIAGE_BODY_X = 0.028
CARRIAGE_BODY_Y = 0.090
CARRIAGE_BODY_Z = 0.060

HANGER_NECK_X = 0.014
HANGER_NECK_Y = 0.022
HANGER_NECK_Z = 0.020
HANGER_NECK_CX = 0.020
HANGER_NECK_CZ = -0.040

HANGER_HEAD_X = 0.022
HANGER_HEAD_Y = 0.048
HANGER_HEAD_Z = 0.014
HANGER_HEAD_CX = 0.024
HANGER_HEAD_CZ = -0.057

CHEEK_X = 0.012
CHEEK_Y = 0.006
CHEEK_Z = 0.052
CHEEK_XC = HANGER_HEAD_CX
CHEEK_YC = 0.015
CHEEK_ZC = -0.090

RAM_TOP_X = 0.016
RAM_TOP_Y = 0.014
RAM_TOP_Z = 0.008

RAM_BODY_X = 0.010
RAM_BODY_Y = 0.014
RAM_BODY_Z = 0.090
RAM_BODY_CZ = -0.053

RAM_FOOT_X = 0.018
RAM_FOOT_Y = 0.024
RAM_FOOT_Z = 0.014
RAM_FOOT_CZ = -0.105

RAM_NOSE_X = 0.016
RAM_NOSE_Y = 0.014
RAM_NOSE_Z = 0.010
RAM_NOSE_CX = 0.003
RAM_NOSE_CZ = -0.117

RAM_JOINT_X = HANGER_HEAD_CX
RAM_JOINT_Z = HANGER_HEAD_CZ - (HANGER_HEAD_Z / 2.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mounted_yz_stage")

    model.material("plate_gray", rgba=(0.71, 0.73, 0.76, 1.0))
    model.material("carriage_charcoal", rgba=(0.19, 0.21, 0.24, 1.0))
    model.material("ram_silver", rgba=(0.82, 0.84, 0.87, 1.0))
    model.material("fastener_dark", rgba=(0.16, 0.16, 0.17, 1.0))

    wall_plate = model.part("wall_plate")
    wall_plate.visual(
        Box((PLATE_T, PLATE_W, PLATE_H)),
        material="plate_gray",
        name="plate_body",
    )
    wall_plate.visual(
        Box((RAIL_T, RAIL_L, RAIL_H)),
        origin=Origin(xyz=(RAIL_CENTER_X, 0.0, RAIL_Z)),
        material="plate_gray",
        name="guide_rail",
    )
    for i, z in enumerate((0.105, -0.105), start=1):
        for j, y in enumerate((-0.075, 0.075), start=1):
            wall_plate.visual(
                Box((0.006, 0.006, 0.004)),
                origin=Origin(xyz=(PLATE_T / 2.0 + 0.001, y, z)),
                material="fastener_dark",
                name=f"mount_pad_{i}_{j}",
            )

    lateral_carriage = model.part("lateral_carriage")
    lateral_carriage.visual(
        Box((CARRIAGE_BODY_X, CARRIAGE_BODY_Y, CARRIAGE_BODY_Z)),
        origin=Origin(xyz=(CARRIAGE_BODY_X / 2.0, 0.0, 0.0)),
        material="carriage_charcoal",
        name="carriage_body",
    )
    lateral_carriage.visual(
        Box((HANGER_NECK_X, HANGER_NECK_Y, HANGER_NECK_Z)),
        origin=Origin(xyz=(HANGER_NECK_CX, 0.0, HANGER_NECK_CZ)),
        material="carriage_charcoal",
        name="hanger_neck",
    )
    lateral_carriage.visual(
        Box((HANGER_HEAD_X, HANGER_HEAD_Y, HANGER_HEAD_Z)),
        origin=Origin(xyz=(HANGER_HEAD_CX, 0.0, HANGER_HEAD_CZ)),
        material="carriage_charcoal",
        name="hanger_head",
    )
    lateral_carriage.visual(
        Box((CHEEK_X, CHEEK_Y, CHEEK_Z)),
        origin=Origin(xyz=(CHEEK_XC, CHEEK_YC, CHEEK_ZC)),
        material="carriage_charcoal",
        name="left_cheek",
    )
    lateral_carriage.visual(
        Box((CHEEK_X, CHEEK_Y, CHEEK_Z)),
        origin=Origin(xyz=(CHEEK_XC, -CHEEK_YC, CHEEK_ZC)),
        material="carriage_charcoal",
        name="right_cheek",
    )

    vertical_ram = model.part("vertical_ram")
    vertical_ram.visual(
        Box((RAM_TOP_X, RAM_TOP_Y, RAM_TOP_Z)),
        origin=Origin(xyz=(0.0, 0.0, -(RAM_TOP_Z / 2.0))),
        material="ram_silver",
        name="ram_top",
    )
    vertical_ram.visual(
        Box((RAM_BODY_X, RAM_BODY_Y, RAM_BODY_Z)),
        origin=Origin(xyz=(0.0, 0.0, RAM_BODY_CZ)),
        material="ram_silver",
        name="ram_stem",
    )
    vertical_ram.visual(
        Box((RAM_FOOT_X, RAM_FOOT_Y, RAM_FOOT_Z)),
        origin=Origin(xyz=(0.0, 0.0, RAM_FOOT_CZ)),
        material="ram_silver",
        name="ram_foot",
    )
    vertical_ram.visual(
        Box((RAM_NOSE_X, RAM_NOSE_Y, RAM_NOSE_Z)),
        origin=Origin(xyz=(RAM_NOSE_CX, 0.0, RAM_NOSE_CZ)),
        material="ram_silver",
        name="ram_nose",
    )

    model.articulation(
        "plate_to_carriage",
        ArticulationType.PRISMATIC,
        parent=wall_plate,
        child=lateral_carriage,
        origin=Origin(xyz=(RAIL_FRONT_X, 0.0, RAIL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=0.25,
            lower=-Y_TRAVEL,
            upper=Y_TRAVEL,
        ),
    )
    model.articulation(
        "carriage_to_ram",
        ArticulationType.PRISMATIC,
        parent=lateral_carriage,
        child=vertical_ram,
        origin=Origin(xyz=(RAM_JOINT_X, 0.0, RAM_JOINT_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.18,
            lower=0.0,
            upper=Z_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_plate = object_model.get_part("wall_plate")
    lateral_carriage = object_model.get_part("lateral_carriage")
    vertical_ram = object_model.get_part("vertical_ram")
    y_slide = object_model.get_articulation("plate_to_carriage")
    z_slide = object_model.get_articulation("carriage_to_ram")

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

    ctx.expect_contact(
        lateral_carriage,
        wall_plate,
        name="carriage_is_supported_by_plate_rail",
    )
    ctx.expect_contact(
        vertical_ram,
        lateral_carriage,
        name="ram_is_retracted_against_carriage_cap",
    )
    ctx.expect_gap(
        lateral_carriage,
        wall_plate,
        axis="x",
        min_gap=0.0,
        max_gap=0.0,
        name="carriage_rear_face_is_flush_to_rail_front",
    )

    with ctx.pose({y_slide: Y_TRAVEL}):
        ctx.expect_origin_gap(
            lateral_carriage,
            wall_plate,
            axis="y",
            min_gap=Y_TRAVEL - 0.001,
            max_gap=Y_TRAVEL + 0.001,
            name="positive_y_travel_moves_carriage_right",
        )

    rest_ram_pos = ctx.part_world_position(vertical_ram)
    with ctx.pose({z_slide: Z_TRAVEL}):
        extended_ram_pos = ctx.part_world_position(vertical_ram)
        ram_delta = None
        if rest_ram_pos is not None and extended_ram_pos is not None:
            ram_delta = rest_ram_pos[2] - extended_ram_pos[2]
        ctx.check(
            "positive_z_travel_lowers_ram",
            ram_delta is not None and (Z_TRAVEL - 0.001) <= ram_delta <= (Z_TRAVEL + 0.001),
            details=f"expected downward travel of {Z_TRAVEL:.3f} m, got {ram_delta!r}",
        )

    with ctx.pose({y_slide: Y_TRAVEL, z_slide: Z_TRAVEL}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_compound_travel_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
