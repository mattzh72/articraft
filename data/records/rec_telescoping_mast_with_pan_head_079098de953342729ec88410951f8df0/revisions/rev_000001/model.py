from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


BASE_LENGTH = 0.48
BASE_WIDTH = 0.40
BASE_FOOT_HEIGHT = 0.020
BASE_BODY_BOTTOM = 0.014
BASE_BODY_HEIGHT = 0.126
BASE_TOP_HEIGHT = 0.055
BASE_TOP_Z0 = BASE_BODY_BOTTOM + BASE_BODY_HEIGHT
BASE_PEDESTAL_HEIGHT = 0.065
BASE_PEDESTAL_Z0 = BASE_TOP_Z0 + BASE_TOP_HEIGHT

FIXED_SLEEVE_OUTER = 0.118
FIXED_SLEEVE_INNER = 0.100
FIXED_SLEEVE_Z0 = 0.260
FIXED_SLEEVE_HEIGHT = 0.360
FIXED_CAP_Z0 = FIXED_SLEEVE_Z0 + FIXED_SLEEVE_HEIGHT
FIXED_CAP_HEIGHT = 0.018
FIXED_CAP_OUTER = 0.150
FIXED_CAP_INNER = 0.106

STAGE1_TUBE_OUTER = 0.104
STAGE1_INSERT_OUTER = 0.068
STAGE1_WALL = 0.006
STAGE1_SKIRT_Z0 = -0.280
STAGE1_SKIRT_Z1 = 0.016
STAGE1_COLLAR_OUTER = 0.126
STAGE1_COLLAR_INNER = 0.098
STAGE1_FLANGE_Z1 = 0.022
STAGE1_TUBE_TOP = 0.460
STAGE1_CAP_OUTER = 0.120
STAGE1_CAP_INNER = 0.082
STAGE1_CAP_Z0 = 0.408
STAGE1_CAP_Z1 = 0.430
STAGE1_TO_STAGE2_ORIGIN_Z = STAGE1_TUBE_TOP
STAGE1_TRAVEL = 0.220

STAGE2_TUBE_OUTER = 0.082
STAGE2_INSERT_OUTER = 0.052
STAGE2_WALL = 0.005
STAGE2_SKIRT_Z0 = -0.240
STAGE2_SKIRT_Z1 = 0.016
STAGE2_COLLAR_OUTER = 0.102
STAGE2_COLLAR_INNER = 0.078
STAGE2_FLANGE_Z1 = 0.020
STAGE2_TUBE_TOP = 0.470
STAGE2_CAP_OUTER = 0.094
STAGE2_CAP_INNER = 0.060
STAGE2_CAP_Z0 = 0.420
STAGE2_CAP_Z1 = 0.442
STAGE2_TO_STAGE3_ORIGIN_Z = STAGE2_TUBE_TOP
STAGE2_TRAVEL = 0.200

STAGE3_TUBE_OUTER = 0.060
STAGE3_INSERT_OUTER = 0.038
STAGE3_WALL = 0.004
STAGE3_SKIRT_Z0 = -0.205
STAGE3_SKIRT_Z1 = 0.014
STAGE3_COLLAR_OUTER = 0.080
STAGE3_COLLAR_INNER = 0.056
STAGE3_FLANGE_Z0 = 0.000
STAGE3_FLANGE_Z1 = 0.018
STAGE3_TUBE_TOP = 0.330
STAGE3_HEAD_OUTER = 0.086
STAGE3_HEAD_INNER = 0.050
STAGE3_HEAD_Z0 = 0.300
STAGE3_HEAD_Z1 = 0.320
STAGE3_BEARING_TOP_Z = 0.356
STAGE3_TO_PLATE_ORIGIN_Z = STAGE3_BEARING_TOP_Z
STAGE3_TRAVEL = 0.180

BASE_TO_STAGE1_ORIGIN_Z = FIXED_CAP_Z0 + FIXED_CAP_HEIGHT
PLATE_SWEEP_LIMIT = 2.70


def _box(size_x: float, size_y: float, size_z: float, z0: float) -> cq.Workplane:
    return cq.Workplane("XY").box(
        size_x,
        size_y,
        size_z,
        centered=(True, True, False),
    ).translate((0.0, 0.0, z0))


def _rect_ring(
    outer_x: float,
    outer_y: float,
    inner_x: float,
    inner_y: float,
    z0: float,
    z1: float,
) -> cq.Workplane:
    outer = _box(outer_x, outer_y, z1 - z0, z0)
    inner = _box(inner_x, inner_y, z1 - z0 + 0.004, z0 - 0.002)
    return outer.cut(inner)


def _square_tube(outer: float, wall: float, z0: float, z1: float) -> cq.Workplane:
    inner = outer - 2.0 * wall
    return _rect_ring(outer, outer, inner, inner, z0, z1)


def _cylinder(radius: float, height: float, z0: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(height).translate((0.0, 0.0, z0))


def _mast_stage_shape(
    *,
    insert_outer: float,
    tube_outer: float,
    wall: float,
    skirt_z0: float,
    skirt_z1: float,
    collar_outer: float,
    collar_inner: float,
    collar_z1: float,
    tube_top: float,
    cap_outer: float,
    cap_inner: float,
    cap_z0: float,
    cap_z1: float,
) -> cq.Workplane:
    upper_tube = _square_tube(tube_outer, wall, 0.0, tube_top)
    collar = _rect_ring(
        collar_outer,
        collar_outer,
        collar_inner,
        collar_inner,
        0.0,
        collar_z1,
    )
    rib_height = min(0.090, tube_top * 0.30)
    rib_x = _box(collar_outer * 0.62, 0.012, rib_height, 0.0)
    rib_y = _box(0.012, collar_outer * 0.62, rib_height, 0.0)
    cap = _rect_ring(cap_outer, cap_outer, cap_inner, cap_inner, cap_z0, cap_z1)
    guide_stub = _square_tube(insert_outer, wall, 0.0, max(0.010, skirt_z1))
    return upper_tube.union(collar).union(rib_x).union(rib_y).union(cap).union(guide_stub)


def _base_housing_shape() -> cq.Workplane:
    lower = _box(BASE_LENGTH, BASE_WIDTH, BASE_BODY_HEIGHT, BASE_BODY_BOTTOM)
    lower = lower.edges("|Z").fillet(0.010)

    upper = _box(0.350, 0.290, BASE_TOP_HEIGHT, BASE_TOP_Z0)
    upper = upper.edges("|Z").fillet(0.008)

    pedestal = _box(0.190, 0.190, BASE_PEDESTAL_HEIGHT, BASE_PEDESTAL_Z0)
    pedestal = pedestal.edges("|Z").fillet(0.006)

    fixed_sleeve = _square_tube(
        FIXED_SLEEVE_OUTER,
        (FIXED_SLEEVE_OUTER - FIXED_SLEEVE_INNER) / 2.0,
        FIXED_SLEEVE_Z0,
        FIXED_SLEEVE_Z0 + FIXED_SLEEVE_HEIGHT,
    )
    fixed_cap = _rect_ring(
        FIXED_CAP_OUTER,
        FIXED_CAP_OUTER,
        FIXED_CAP_INNER,
        FIXED_CAP_INNER,
        FIXED_CAP_Z0,
        FIXED_CAP_Z0 + FIXED_CAP_HEIGHT,
    )

    x_rib = _box(0.240, 0.024, 0.110, BASE_PEDESTAL_Z0)
    y_rib = _box(0.024, 0.240, 0.110, BASE_PEDESTAL_Z0)

    foot_offsets = (
        (0.175, 0.135),
        (0.175, -0.135),
        (-0.175, 0.135),
        (-0.175, -0.135),
    )
    feet = None
    for x_pos, y_pos in foot_offsets:
        foot = _box(0.085, 0.060, BASE_FOOT_HEIGHT, 0.0).translate((x_pos, y_pos, 0.0))
        feet = foot if feet is None else feet.union(foot)

    body = lower.union(upper).union(pedestal).union(x_rib).union(y_rib).union(feet)
    body = body.union(fixed_sleeve).union(fixed_cap)

    side_pocket_x = _box(0.280, 0.030, 0.062, 0.050)
    side_pocket_y = _box(0.030, 0.230, 0.062, 0.050)
    body = body.cut(side_pocket_x.translate((0.0, BASE_WIDTH / 2.0 - 0.012, 0.0)))
    body = body.cut(side_pocket_x.translate((0.0, -BASE_WIDTH / 2.0 + 0.012, 0.0)))
    body = body.cut(side_pocket_y.translate((BASE_LENGTH / 2.0 - 0.012, 0.0, 0.0)))
    body = body.cut(side_pocket_y.translate((-BASE_LENGTH / 2.0 + 0.012, 0.0, 0.0)))
    return body


def _stage1_shape() -> cq.Workplane:
    return _mast_stage_shape(
        insert_outer=STAGE1_INSERT_OUTER,
        tube_outer=STAGE1_TUBE_OUTER,
        wall=STAGE1_WALL,
        skirt_z0=STAGE1_SKIRT_Z0,
        skirt_z1=STAGE1_SKIRT_Z1,
        collar_outer=STAGE1_COLLAR_OUTER,
        collar_inner=STAGE1_COLLAR_INNER,
        collar_z1=STAGE1_FLANGE_Z1,
        tube_top=STAGE1_TUBE_TOP,
        cap_outer=STAGE1_CAP_OUTER,
        cap_inner=STAGE1_CAP_INNER,
        cap_z0=STAGE1_CAP_Z0,
        cap_z1=STAGE1_CAP_Z1,
    )


def _stage2_shape() -> cq.Workplane:
    return _mast_stage_shape(
        insert_outer=STAGE2_INSERT_OUTER,
        tube_outer=STAGE2_TUBE_OUTER,
        wall=STAGE2_WALL,
        skirt_z0=STAGE2_SKIRT_Z0,
        skirt_z1=STAGE2_SKIRT_Z1,
        collar_outer=STAGE2_COLLAR_OUTER,
        collar_inner=STAGE2_COLLAR_INNER,
        collar_z1=STAGE2_FLANGE_Z1,
        tube_top=STAGE2_TUBE_TOP,
        cap_outer=STAGE2_CAP_OUTER,
        cap_inner=STAGE2_CAP_INNER,
        cap_z0=STAGE2_CAP_Z0,
        cap_z1=STAGE2_CAP_Z1,
    )


def _stage3_upper_shape() -> cq.Workplane:
    return _mast_stage_shape(
        insert_outer=STAGE3_INSERT_OUTER,
        tube_outer=STAGE3_TUBE_OUTER,
        wall=STAGE3_WALL,
        skirt_z0=STAGE3_SKIRT_Z0,
        skirt_z1=STAGE3_SKIRT_Z1,
        collar_outer=STAGE3_COLLAR_OUTER,
        collar_inner=STAGE3_COLLAR_INNER,
        collar_z1=STAGE3_FLANGE_Z1,
        tube_top=STAGE3_HEAD_Z0,
        cap_outer=0.082,
        cap_inner=0.052,
        cap_z0=0.278,
        cap_z1=STAGE3_HEAD_Z0,
    )


def _stage3_bearing_shape() -> cq.Workplane:
    body = _cylinder(0.034, STAGE3_BEARING_TOP_Z - STAGE3_HEAD_Z0, STAGE3_HEAD_Z0)
    lower_flange = _cylinder(0.043, 0.010, STAGE3_HEAD_Z0)
    thrust_plate = _rect_ring(0.092, 0.092, 0.034, 0.034, STAGE3_BEARING_TOP_Z - 0.014, STAGE3_BEARING_TOP_Z)
    body = body.union(lower_flange).union(thrust_plate)
    bore = _cylinder(0.015, STAGE3_BEARING_TOP_Z - STAGE3_HEAD_Z0 + 0.004, STAGE3_HEAD_Z0 - 0.002)
    return body.cut(bore)


def _instrument_plate_body() -> cq.Workplane:
    body = _box(0.180, 0.120, 0.012, 0.040)
    body = body.cut(_cylinder(0.032, 0.020, 0.036))
    body = body.edges("|Z").fillet(0.010)

    top_pad = _box(0.074, 0.052, 0.006, 0.052).translate((0.022, 0.0, 0.0))
    rib_left = _box(0.112, 0.012, 0.010, 0.032).translate((0.0, 0.026, 0.0))
    rib_right = _box(0.112, 0.012, 0.010, 0.032).translate((0.0, -0.026, 0.0))
    return body.union(top_pad).union(rib_left).union(rib_right)


def _instrument_hub() -> cq.Workplane:
    thrust_flange = _cylinder(0.022, 0.004, 0.0).cut(_cylinder(0.0105, 0.008, -0.002))
    hub_barrel = _cylinder(0.016, 0.046, 0.0).cut(_cylinder(0.0105, 0.050, -0.002))
    mounting_flange = _cylinder(0.037, 0.004, 0.038).cut(_cylinder(0.0105, 0.008, 0.036))
    top_retainer = _cylinder(0.013, 0.008, 0.038)
    return thrust_flange.union(hub_barrel).union(mounting_flange).union(top_retainer)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="deployable_equipment_mast")

    model.material("powder_charcoal", rgba=(0.20, 0.22, 0.24, 1.0))
    model.material("anodized_aluminum", rgba=(0.70, 0.73, 0.77, 1.0))
    model.material("machined_steel", rgba=(0.82, 0.84, 0.87, 1.0))
    model.material("instrument_black", rgba=(0.12, 0.13, 0.14, 1.0))

    base = model.part("base_housing")
    base.visual(
        mesh_from_cadquery(_base_housing_shape(), "base_housing"),
        material="powder_charcoal",
        name="housing",
    )

    stage1 = model.part("lower_mast_stage")
    stage1.visual(
        mesh_from_cadquery(_stage1_shape(), "lower_mast_stage"),
        material="anodized_aluminum",
        name="stage_body",
    )

    stage2 = model.part("middle_mast_stage")
    stage2.visual(
        mesh_from_cadquery(_stage2_shape(), "middle_mast_stage"),
        material="anodized_aluminum",
        name="stage_body",
    )

    stage3 = model.part("upper_mast_stage")
    stage3.visual(
        mesh_from_cadquery(_stage3_upper_shape(), "upper_mast_upper"),
        material="anodized_aluminum",
        name="upper_tube",
    )
    stage3.visual(
        mesh_from_cadquery(_stage3_bearing_shape(), "upper_mast_bearing"),
        material="machined_steel",
        name="mast_head",
    )

    plate = model.part("instrument_plate")
    plate.visual(
        mesh_from_cadquery(_instrument_plate_body(), "instrument_plate_body"),
        material="instrument_black",
        name="instrument_plate",
    )
    plate.visual(
        mesh_from_cadquery(_instrument_hub(), "instrument_plate_hub"),
        material="machined_steel",
        name="turntable_hub",
    )

    model.articulation(
        "base_to_stage1",
        ArticulationType.PRISMATIC,
        parent=base,
        child=stage1,
        origin=Origin(xyz=(0.0, 0.0, BASE_TO_STAGE1_ORIGIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=240.0,
            velocity=0.28,
            lower=0.0,
            upper=STAGE1_TRAVEL,
        ),
    )
    model.articulation(
        "stage1_to_stage2",
        ArticulationType.PRISMATIC,
        parent=stage1,
        child=stage2,
        origin=Origin(xyz=(0.0, 0.0, STAGE1_TO_STAGE2_ORIGIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.26,
            lower=0.0,
            upper=STAGE2_TRAVEL,
        ),
    )
    model.articulation(
        "stage2_to_stage3",
        ArticulationType.PRISMATIC,
        parent=stage2,
        child=stage3,
        origin=Origin(xyz=(0.0, 0.0, STAGE2_TO_STAGE3_ORIGIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.24,
            lower=0.0,
            upper=STAGE3_TRAVEL,
        ),
    )
    model.articulation(
        "stage3_to_plate",
        ArticulationType.REVOLUTE,
        parent=stage3,
        child=plate,
        origin=Origin(xyz=(0.0, 0.0, STAGE3_TO_PLATE_ORIGIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.2,
            lower=-PLATE_SWEEP_LIMIT,
            upper=PLATE_SWEEP_LIMIT,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_housing")
    stage1 = object_model.get_part("lower_mast_stage")
    stage2 = object_model.get_part("middle_mast_stage")
    stage3 = object_model.get_part("upper_mast_stage")
    plate = object_model.get_part("instrument_plate")

    base_to_stage1 = object_model.get_articulation("base_to_stage1")
    stage1_to_stage2 = object_model.get_articulation("stage1_to_stage2")
    stage2_to_stage3 = object_model.get_articulation("stage2_to_stage3")
    stage3_to_plate = object_model.get_articulation("stage3_to_plate")

    mast_head = stage3.get_visual("mast_head")
    turntable_hub = plate.get_visual("turntable_hub")
    plate_body = plate.get_visual("instrument_plate")

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

    ctx.check(
        "all_mast_axes_vertical",
        base_to_stage1.axis == (0.0, 0.0, 1.0)
        and stage1_to_stage2.axis == (0.0, 0.0, 1.0)
        and stage2_to_stage3.axis == (0.0, 0.0, 1.0)
        and stage3_to_plate.axis == (0.0, 0.0, 1.0),
        details="Every telescoping and turntable articulation should be aligned to the vertical mast axis.",
    )

    ctx.expect_contact(base, stage1, name="lower_stage_seats_on_fixed_cap")
    ctx.expect_contact(stage1, stage2, name="middle_stage_seats_on_lower_stage_cap")
    ctx.expect_contact(stage2, stage3, name="upper_stage_seats_on_middle_stage_cap")
    ctx.expect_contact(
        stage3,
        plate,
        elem_a=mast_head,
        elem_b=turntable_hub,
        name="turntable_hub_supported_by_mast_head",
    )
    ctx.expect_gap(
        plate,
        base,
        axis="z",
        min_gap=0.26,
        name="plate_carried_clear_above_base_housing",
    )
    ctx.expect_overlap(
        stage1,
        base,
        axes="xy",
        min_overlap=0.10,
        name="lower_stage_stays_centered_in_base_sleeve",
    )

    with ctx.pose(
        {
            base_to_stage1: STAGE1_TRAVEL,
            stage1_to_stage2: STAGE2_TRAVEL,
            stage2_to_stage3: STAGE3_TRAVEL,
            stage3_to_plate: pi / 2.0,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_fully_extended_and_rotated")
        ctx.expect_overlap(
            stage2,
            stage1,
            axes="xy",
            min_overlap=0.07,
            name="middle_stage_remains_nested_at_full_extension",
        )
        ctx.expect_overlap(
            stage3,
            stage2,
            axes="xy",
            min_overlap=0.05,
            name="upper_stage_remains_nested_at_full_extension",
        )
        ctx.expect_gap(
            plate,
            stage3,
            axis="z",
            positive_elem=plate_body,
            negative_elem=mast_head,
            min_gap=0.018,
            name="plate_sweeps_clear_above_mast_head",
        )
        ctx.expect_gap(
            plate,
            base,
            axis="z",
            min_gap=0.82,
            name="extended_plate_stays_well_above_base",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
