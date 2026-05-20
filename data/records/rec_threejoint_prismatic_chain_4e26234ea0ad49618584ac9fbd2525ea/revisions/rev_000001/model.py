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


PLATE_T = 0.012
PLATE_W = 0.220
PLATE_H = 0.620

VERT_RAIL_DEPTH = 0.026
VERT_RAIL_W = 0.050
VERT_RAIL_L = 0.560
VERT_RAIL_X = (PLATE_T / 2.0) + (VERT_RAIL_DEPTH / 2.0)

FIRST_HOME_Z = -0.190
FIRST_TRAVEL = 0.300

FIRST_SHOE_X = 0.014
FIRST_SHOE_Y = 0.110
FIRST_SHOE_Z = 0.170
FIRST_BLOCK_X = 0.050
FIRST_BLOCK_Y = 0.120
FIRST_BLOCK_Z = 0.120
FIRST_BODY_CENTER_X = 0.040

SECOND_BEAM_L = 0.320
SECOND_BEAM_Y = 0.060
SECOND_BEAM_Z = 0.032
SECOND_BEAM_BACK_X = 0.070

SECOND_HOME_X = 0.090
SECOND_TRAVEL = 0.210

SECOND_PAD_X = 0.070
SECOND_PAD_Y = 0.110
SECOND_PAD_Z = 0.028
SECOND_BLOCK_X = 0.050
SECOND_BLOCK_Y = 0.080
SECOND_BLOCK_Z = 0.140

TIP_SUPPORT_X = 0.030
TIP_SUPPORT_Y = 0.090
TIP_SUPPORT_Z = 0.190
TIP_SUPPORT_CENTER_X = 0.145
TIP_SUPPORT_CENTER_Z = 0.095

TIP_RAIL_X = 0.024
TIP_RAIL_Y = 0.040
TIP_RAIL_Z = 0.220
TIP_RAIL_CENTER_X = 0.180
TIP_RAIL_CENTER_Z = 0.115

THIRD_HOME_Z = 0.035
THIRD_TRAVEL = 0.100

THIRD_BLOCK_X = 0.012
THIRD_BLOCK_Y = 0.092
THIRD_BLOCK_Z = 0.100
THIRD_BLOCK_CENTER_X = 0.006

TOOL_PLATE_X = 0.018
TOOL_PLATE_Y = 0.120
TOOL_PLATE_Z = 0.110
TOOL_PLATE_CENTER_X = 0.054


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _x_cylinder(length: float, radius: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .circle(radius)
        .extrude(length)
        .translate((center[0] - (length / 2.0), center[1], center[2]))
    )


def _side_plate_shape() -> cq.Workplane:
    plate = _box((PLATE_T, PLATE_W, PLATE_H), (0.0, 0.0, 0.0)).edges("|X").fillet(0.010)
    rail = _box((VERT_RAIL_DEPTH, VERT_RAIL_W, VERT_RAIL_L), (VERT_RAIL_X, 0.0, 0.0))
    upper_mount = _box((0.036, 0.100, 0.040), (0.018, 0.0, 0.260))
    lower_mount = _box((0.036, 0.100, 0.040), (0.018, 0.0, -0.260))
    body = plate.union(rail).union(upper_mount).union(lower_mount)

    for y_pos in (-0.070, 0.070):
        for z_pos in (-0.210, 0.210):
            body = body.cut(_x_cylinder(PLATE_T + 0.006, 0.0065, (0.0, y_pos, z_pos)))

    return body


def _first_carriage_shape() -> cq.Workplane:
    rail_shoe = _box((FIRST_SHOE_X, FIRST_SHOE_Y, FIRST_SHOE_Z), (FIRST_SHOE_X / 2.0, 0.0, 0.0))
    carriage_body = _box(
        (FIRST_BLOCK_X, FIRST_BLOCK_Y, FIRST_BLOCK_Z),
        (FIRST_BODY_CENTER_X, 0.0, 0.0),
    )
    body_bridge = _box((0.006, 0.110, 0.130), (0.015, 0.0, 0.0))
    beam = _box(
        (SECOND_BEAM_L, SECOND_BEAM_Y, SECOND_BEAM_Z),
        (SECOND_BEAM_BACK_X + (SECOND_BEAM_L / 2.0), 0.0, 0.0),
    )
    rib = _box((0.060, 0.090, 0.030), (0.060, 0.0, -0.028))

    return rail_shoe.union(body_bridge).union(carriage_body).union(beam).union(rib)


def _second_carriage_shape() -> cq.Workplane:
    slider_block = _box((SECOND_PAD_X, SECOND_PAD_Y, SECOND_PAD_Z), (SECOND_PAD_X / 2.0, 0.0, SECOND_PAD_Z / 2.0))
    saddle_block = _box(
        (SECOND_BLOCK_X, SECOND_BLOCK_Y, SECOND_BLOCK_Z),
        (0.095, 0.0, 0.082),
    )
    support_arm = _box((0.080, 0.060, 0.020), (0.130, 0.0, 0.038))
    support_plate = _box(
        (TIP_SUPPORT_X, TIP_SUPPORT_Y, TIP_SUPPORT_Z),
        (TIP_SUPPORT_CENTER_X, 0.0, TIP_SUPPORT_CENTER_Z),
    )
    tip_rail = _box((TIP_RAIL_X, TIP_RAIL_Y, TIP_RAIL_Z), (TIP_RAIL_CENTER_X, 0.0, TIP_RAIL_CENTER_Z))
    rail_gusset = _box((0.050, 0.070, 0.040), (0.155, 0.0, 0.060))

    return slider_block.union(saddle_block).union(support_arm).union(support_plate).union(tip_rail).union(rail_gusset)


def _third_stage_shape() -> cq.Workplane:
    slider_block = _box(
        (THIRD_BLOCK_X, THIRD_BLOCK_Y, THIRD_BLOCK_Z),
        (THIRD_BLOCK_CENTER_X, 0.0, THIRD_BLOCK_Z / 2.0),
    )
    stage_body = _box((0.034, 0.104, 0.098), (0.029, 0.0, 0.055))
    neck = _box((0.012, 0.070, 0.050), (0.043, 0.0, 0.055))
    tool_plate = _box(
        (TOOL_PLATE_X, TOOL_PLATE_Y, TOOL_PLATE_Z),
        (TOOL_PLATE_CENTER_X, 0.0, 0.055),
    )

    return slider_block.union(stage_body).union(neck).union(tool_plate)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_wall_triple_carriage_module")

    model.material("painted_steel", rgba=(0.28, 0.31, 0.35, 1.0))
    model.material("machined_aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("dark_anodized", rgba=(0.20, 0.22, 0.25, 1.0))
    model.material("tool_plate_blue", rgba=(0.24, 0.39, 0.70, 1.0))

    side_plate = model.part("side_plate")
    side_plate.visual(
        mesh_from_cadquery(_side_plate_shape(), "side_plate"),
        material="painted_steel",
        name="side_plate_shell",
    )

    first_carriage = model.part("first_carriage")
    first_carriage.visual(
        mesh_from_cadquery(_first_carriage_shape(), "first_carriage"),
        material="machined_aluminum",
        name="first_carriage_shell",
    )

    second_carriage = model.part("second_carriage")
    second_carriage.visual(
        mesh_from_cadquery(_second_carriage_shape(), "second_carriage"),
        material="dark_anodized",
        name="second_carriage_shell",
    )

    third_stage = model.part("third_stage")
    third_stage.visual(
        mesh_from_cadquery(_third_stage_shape(), "third_stage"),
        material="tool_plate_blue",
        name="third_stage_shell",
    )

    model.articulation(
        "side_plate_to_first_carriage",
        ArticulationType.PRISMATIC,
        parent=side_plate,
        child=first_carriage,
        origin=Origin(xyz=((PLATE_T / 2.0) + VERT_RAIL_DEPTH, 0.0, FIRST_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=450.0,
            velocity=0.25,
            lower=0.0,
            upper=FIRST_TRAVEL,
        ),
    )
    model.articulation(
        "first_to_second_carriage",
        ArticulationType.PRISMATIC,
        parent=first_carriage,
        child=second_carriage,
        origin=Origin(xyz=(SECOND_HOME_X, 0.0, SECOND_BEAM_Z / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=260.0,
            velocity=0.30,
            lower=0.0,
            upper=SECOND_TRAVEL,
        ),
    )
    model.articulation(
        "second_to_third_stage",
        ArticulationType.PRISMATIC,
        parent=second_carriage,
        child=third_stage,
        origin=Origin(xyz=(TIP_RAIL_CENTER_X + (TIP_RAIL_X / 2.0), 0.0, THIRD_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.20,
            lower=0.0,
            upper=THIRD_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    side_plate = object_model.get_part("side_plate")
    first_carriage = object_model.get_part("first_carriage")
    second_carriage = object_model.get_part("second_carriage")
    third_stage = object_model.get_part("third_stage")

    first_joint = object_model.get_articulation("side_plate_to_first_carriage")
    second_joint = object_model.get_articulation("first_to_second_carriage")
    third_joint = object_model.get_articulation("second_to_third_stage")

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
        first_carriage,
        side_plate,
        name="first carriage is guided by the side-wall rail",
    )
    ctx.expect_contact(
        second_carriage,
        first_carriage,
        name="second carriage is guided by the first carriage beam",
    )
    ctx.expect_contact(
        third_stage,
        second_carriage,
        name="third stage is guided by the tip rail",
    )

    home_first = ctx.part_world_position(first_carriage)
    home_second = ctx.part_world_position(second_carriage)
    home_third = ctx.part_world_position(third_stage)

    with ctx.pose({first_joint: FIRST_TRAVEL}):
        top_first = ctx.part_world_position(first_carriage)
        ctx.expect_contact(
            first_carriage,
            side_plate,
            name="first carriage stays supported at full vertical travel",
        )
        ctx.check(
            "first carriage travels upward along +Z",
            home_first is not None
            and top_first is not None
            and abs((top_first[2] - home_first[2]) - FIRST_TRAVEL) < 1e-4,
            details=f"expected {FIRST_TRAVEL:.4f} m upward travel",
        )

    with ctx.pose({second_joint: SECOND_TRAVEL}):
        out_second = ctx.part_world_position(second_carriage)
        ctx.expect_contact(
            second_carriage,
            first_carriage,
            name="second carriage stays supported at full outward travel",
        )
        ctx.check(
            "second carriage travels outward along +X",
            home_second is not None
            and out_second is not None
            and abs((out_second[0] - home_second[0]) - SECOND_TRAVEL) < 1e-4,
            details=f"expected {SECOND_TRAVEL:.4f} m outward travel",
        )

    with ctx.pose({third_joint: THIRD_TRAVEL}):
        raised_third = ctx.part_world_position(third_stage)
        ctx.expect_contact(
            third_stage,
            second_carriage,
            name="third stage stays supported at full tip travel",
        )
        ctx.check(
            "third stage travels upward along +Z",
            home_third is not None
            and raised_third is not None
            and abs((raised_third[2] - home_third[2]) - THIRD_TRAVEL) < 1e-4,
            details=f"expected {THIRD_TRAVEL:.4f} m upward travel",
        )

    with ctx.pose(
        {
            first_joint: FIRST_TRAVEL,
            second_joint: SECOND_TRAVEL,
            third_joint: THIRD_TRAVEL,
        }
    ):
        ctx.expect_contact(
            second_carriage,
            first_carriage,
            name="second carriage remains captured in the fully extended pose",
        )
        ctx.expect_contact(
            third_stage,
            second_carriage,
            name="third stage remains captured in the fully extended pose",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no overlaps in the fully extended pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
