from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

BASE_L = 0.280
BASE_W = 0.180
BASE_T = 0.018
PEDESTAL_L = 0.132
PEDESTAL_W = 0.102
PEDESTAL_H = 0.030
ROOT_SEAT_R_OUT = 0.052
ROOT_SEAT_R_IN = 0.024
ROOT_SEAT_H = 0.030
ROOT_Z = BASE_T + PEDESTAL_H + ROOT_SEAT_H

ROTARY_FLANGE_R_OUT = 0.058
ROTARY_FLANGE_R_IN = 0.034
ROTARY_FLANGE_T = 0.010
ROTARY_HUB_R = 0.038
ROTARY_HUB_H = 0.018
ROTARY_FRAME_X0 = 0.015
ROTARY_FRAME_L = 0.230
ROTARY_SIDE_Y = 0.054
ROTARY_SIDE_T = 0.010
ROTARY_SIDE_H = 0.080
ROTARY_PROX_BLOCK_L = 0.070
ROTARY_DISTAL_BLOCK_L = 0.020
BED_X0 = 0.060
BED_L = 0.162
BED_W = 0.078
BED_Z0 = 0.018
BED_T = 0.010
RAIL_L = 0.148
RAIL_W = 0.016
RAIL_T = 0.006
RAIL_X0 = 0.066
RAIL_Y = 0.023
RAIL_TOP_Z = BED_Z0 + BED_T + RAIL_T
GUIDE_ROD_R = 0.0065
GUIDE_ROD_L = 0.172
GUIDE_ROD_X0 = 0.044
GUIDE_ROD_Y = 0.042
GUIDE_ROD_Z = 0.058
LEADSCREW_R = 0.007
LEADSCREW_L = 0.176
LEADSCREW_X0 = 0.040
LEADSCREW_Z = 0.045

SLIDE_ORIGIN_X = 0.135
SLIDE_ORIGIN_Z = RAIL_TOP_Z
SLIDE_LOWER = -0.020
SLIDE_UPPER = 0.070

CARRIAGE_L = 0.074
CARRIAGE_SHOE_W = 0.022
CARRIAGE_SHOE_Z = 0.010
CARRIAGE_BODY_W = 0.072
CARRIAGE_BODY_Z = 0.030
CARRIAGE_CAP_Z = 0.018
CARRIAGE_CAP_W = 0.060
CARRIAGE_HOUSING_R = 0.0125
CARRIAGE_GUIDE_BORE_R = 0.0098
CARRIAGE_LEAD_BORE_R = 0.0112
CARRIAGE_NOSE_X0 = 0.010
CARRIAGE_NOSE_L = 0.044
CARRIAGE_NOSE_W = 0.046
CARRIAGE_EAR_X0 = 0.052
CARRIAGE_EAR_L = 0.030
CARRIAGE_EAR_Y = 0.021
CARRIAGE_EAR_T = 0.011
CARRIAGE_EAR_Z0 = 0.034
CARRIAGE_EAR_H = 0.056
CLEVIS_GAP = 0.031
WRIST_ORIGIN_X = 0.086
WRIST_ORIGIN_Z = 0.053

WRIST_PIN_R = 0.0048
WRIST_HUB_R = 0.011
WRIST_HUB_BORE_R = 0.0060
WRIST_HUB_W = 0.031
WRIST_PLATE_L = 0.060
WRIST_PLATE_W = 0.016
WRIST_PLATE_X0 = 0.010
WRIST_PLATE_Z0 = -0.020
WRIST_PLATE_H = 0.056


def _x_cylinder(radius: float, length: float) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length)


def _y_cylinder(radius: float, length: float) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length)


def _make_base_shape() -> cq.Workplane:
    base = cq.Workplane("XY").box(BASE_L, BASE_W, BASE_T, centered=(True, True, False))
    base = base.edges("|Z").fillet(0.004)

    pedestal = (
        cq.Workplane("XY")
        .box(PEDESTAL_L, PEDESTAL_W, PEDESTAL_H, centered=(True, True, False))
        .translate((0.0, 0.0, BASE_T))
    )
    seat = (
        cq.Workplane("XY")
        .circle(ROOT_SEAT_R_OUT)
        .circle(ROOT_SEAT_R_IN)
        .extrude(ROOT_SEAT_H)
        .translate((0.0, 0.0, BASE_T + PEDESTAL_H))
    )

    left_rib = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.060, BASE_T),
                (-0.028, BASE_T),
                (-0.028, BASE_T + PEDESTAL_H),
            ]
        )
        .close()
        .extrude(0.008)
        .translate((0.0, PEDESTAL_W * 0.5 - 0.004, 0.0))
    )
    right_rib = left_rib.mirror("XZ")

    stop_boss_a = (
        cq.Workplane("XY")
        .box(0.018, 0.014, 0.012, centered=(True, True, False))
        .translate((0.040, 0.040, BASE_T + PEDESTAL_H - 0.006))
    )
    stop_boss_b = (
        cq.Workplane("XY")
        .box(0.018, 0.014, 0.012, centered=(True, True, False))
        .translate((0.040, -0.040, BASE_T + PEDESTAL_H - 0.006))
    )

    base = base.union(pedestal).union(seat).union(left_rib).union(right_rib)
    base = base.union(stop_boss_a).union(stop_boss_b)

    top_cover_recess = (
        cq.Workplane("XY")
        .box(0.082, 0.052, 0.0022, centered=(True, True, False))
        .translate((0.0, -0.020, BASE_T + PEDESTAL_H - 0.0022))
    )
    base = base.cut(top_cover_recess)

    for x in (-0.030, 0.030):
        for y in (-0.038, -0.002):
            screw_pocket = (
                cq.Workplane("XY")
                .circle(0.0032)
                .extrude(0.0016)
                .translate((x, y, BASE_T + PEDESTAL_H - 0.0016))
            )
            base = base.cut(screw_pocket)

    center_relief = (
        cq.Workplane("XY")
        .circle(0.018)
        .extrude(0.010)
        .translate((0.0, 0.0, BASE_T + PEDESTAL_H + 0.008))
    )
    base = base.cut(center_relief)
    return base


def _make_rotary_module_shape() -> cq.Workplane:
    flange = cq.Workplane("XY").circle(ROTARY_FLANGE_R_OUT).circle(ROTARY_FLANGE_R_IN).extrude(ROTARY_FLANGE_T)
    hub = cq.Workplane("XY").circle(ROTARY_HUB_R).extrude(ROTARY_HUB_H).translate((0.0, 0.0, ROTARY_FLANGE_T))
    prox_block = (
        cq.Workplane("XY")
        .box(
            ROTARY_PROX_BLOCK_L,
            ROTARY_SIDE_Y * 2.0,
            0.042,
            centered=(False, True, False),
        )
        .translate((0.0, 0.0, ROTARY_FLANGE_T))
    )
    left_plate = (
        cq.Workplane("XY")
        .box(
            ROTARY_FRAME_L,
            ROTARY_SIDE_T,
            ROTARY_SIDE_H,
            centered=(False, True, False),
        )
        .translate((ROTARY_FRAME_X0, ROTARY_SIDE_Y, ROTARY_FLANGE_T))
    )
    right_plate = (
        cq.Workplane("XY")
        .box(
            ROTARY_FRAME_L,
            ROTARY_SIDE_T,
            ROTARY_SIDE_H,
            centered=(False, True, False),
        )
        .translate((ROTARY_FRAME_X0, -ROTARY_SIDE_Y, ROTARY_FLANGE_T))
    )
    distal_stop_left = (
        cq.Workplane("XY")
        .box(ROTARY_DISTAL_BLOCK_L, 0.012, 0.030, centered=(False, True, False))
        .translate((ROTARY_FRAME_X0 + ROTARY_FRAME_L - ROTARY_DISTAL_BLOCK_L, 0.044, 0.024))
    )
    distal_stop_right = (
        cq.Workplane("XY")
        .box(ROTARY_DISTAL_BLOCK_L, 0.012, 0.030, centered=(False, True, False))
        .translate((ROTARY_FRAME_X0 + ROTARY_FRAME_L - ROTARY_DISTAL_BLOCK_L, -0.044, 0.024))
    )
    bed = (
        cq.Workplane("XY")
        .box(BED_L, BED_W, BED_T, centered=(False, True, False))
        .translate((BED_X0, 0.0, BED_Z0))
    )
    rail_left = (
        cq.Workplane("XY")
        .box(RAIL_L, RAIL_W, RAIL_T, centered=(False, True, False))
        .translate((RAIL_X0, RAIL_Y, BED_Z0 + BED_T))
    )
    rail_right = (
        cq.Workplane("XY")
        .box(RAIL_L, RAIL_W, RAIL_T, centered=(False, True, False))
        .translate((RAIL_X0, -RAIL_Y, BED_Z0 + BED_T))
    )
    rod_left = _x_cylinder(GUIDE_ROD_R, GUIDE_ROD_L).translate((GUIDE_ROD_X0, GUIDE_ROD_Y, GUIDE_ROD_Z))
    rod_right = _x_cylinder(GUIDE_ROD_R, GUIDE_ROD_L).translate((GUIDE_ROD_X0, -GUIDE_ROD_Y, GUIDE_ROD_Z))
    leadscrew = _x_cylinder(LEADSCREW_R, LEADSCREW_L).translate((LEADSCREW_X0, 0.0, LEADSCREW_Z))
    drive_coupler = _x_cylinder(0.011, 0.020).translate((0.018, 0.0, LEADSCREW_Z))
    fixed_stop_a = (
        cq.Workplane("XY")
        .box(0.010, 0.012, 0.014, centered=(False, True, False))
        .translate((0.074, 0.018, RAIL_TOP_Z))
    )
    fixed_stop_b = (
        cq.Workplane("XY")
        .box(0.010, 0.012, 0.014, centered=(False, True, False))
        .translate((0.210, -0.018, RAIL_TOP_Z))
    )

    module = flange.union(hub).union(prox_block)
    module = module.union(left_plate).union(right_plate).union(distal_stop_left).union(distal_stop_right)
    module = module.union(bed).union(rail_left).union(rail_right)
    module = module.union(rod_left).union(rod_right).union(leadscrew).union(drive_coupler)
    module = module.union(fixed_stop_a).union(fixed_stop_b)

    left_window = (
        cq.Workplane("XY")
        .box(0.108, 0.006, 0.042, centered=(False, True, False))
        .translate((0.092, ROTARY_SIDE_Y, 0.028))
    )
    right_window = (
        cq.Workplane("XY")
        .box(0.108, 0.006, 0.042, centered=(False, True, False))
        .translate((0.092, -ROTARY_SIDE_Y, 0.028))
    )
    top_cover_recess = (
        cq.Workplane("XY")
        .box(0.046, 0.040, 0.0020, centered=(False, True, False))
        .translate((0.010, 0.0, ROTARY_FLANGE_T + 0.040))
    )
    module = module.cut(left_window).cut(right_window).cut(top_cover_recess)

    for x in (0.018, 0.046):
        for y in (-0.014, 0.014):
            pocket = (
                cq.Workplane("XY")
                .circle(0.0027)
                .extrude(0.0014)
                .translate((x, y, ROTARY_FLANGE_T + 0.040 - 0.0014))
            )
            module = module.cut(pocket)

    return module


def _make_rotary_structure_shape() -> cq.Workplane:
    flange = cq.Workplane("XY").circle(ROTARY_FLANGE_R_OUT).circle(ROTARY_FLANGE_R_IN).extrude(ROTARY_FLANGE_T)
    hub = cq.Workplane("XY").circle(ROTARY_HUB_R).extrude(ROTARY_HUB_H).translate((0.0, 0.0, ROTARY_FLANGE_T))
    prox_block = (
        cq.Workplane("XY")
        .box(
            ROTARY_PROX_BLOCK_L,
            ROTARY_SIDE_Y * 2.0,
            0.042,
            centered=(False, True, False),
        )
        .translate((0.0, 0.0, ROTARY_FLANGE_T))
    )
    left_plate = (
        cq.Workplane("XY")
        .box(
            ROTARY_FRAME_L,
            ROTARY_SIDE_T,
            ROTARY_SIDE_H,
            centered=(False, True, False),
        )
        .translate((ROTARY_FRAME_X0, ROTARY_SIDE_Y, ROTARY_FLANGE_T))
    )
    right_plate = (
        cq.Workplane("XY")
        .box(
            ROTARY_FRAME_L,
            ROTARY_SIDE_T,
            ROTARY_SIDE_H,
            centered=(False, True, False),
        )
        .translate((ROTARY_FRAME_X0, -ROTARY_SIDE_Y, ROTARY_FLANGE_T))
    )
    distal_stop_left = (
        cq.Workplane("XY")
        .box(ROTARY_DISTAL_BLOCK_L, 0.012, 0.030, centered=(False, True, False))
        .translate((ROTARY_FRAME_X0 + ROTARY_FRAME_L - ROTARY_DISTAL_BLOCK_L, 0.044, 0.024))
    )
    distal_stop_right = (
        cq.Workplane("XY")
        .box(ROTARY_DISTAL_BLOCK_L, 0.012, 0.030, centered=(False, True, False))
        .translate((ROTARY_FRAME_X0 + ROTARY_FRAME_L - ROTARY_DISTAL_BLOCK_L, -0.044, 0.024))
    )
    bed = (
        cq.Workplane("XY")
        .box(BED_L, BED_W, BED_T, centered=(False, True, False))
        .translate((BED_X0, 0.0, BED_Z0))
    )
    fixed_stop_a = (
        cq.Workplane("XY")
        .box(0.010, 0.012, 0.014, centered=(False, True, False))
        .translate((0.074, 0.018, RAIL_TOP_Z))
    )
    fixed_stop_b = (
        cq.Workplane("XY")
        .box(0.010, 0.012, 0.014, centered=(False, True, False))
        .translate((0.210, -0.018, RAIL_TOP_Z))
    )

    structure = flange.union(hub).union(prox_block)
    structure = structure.union(left_plate).union(right_plate).union(distal_stop_left).union(distal_stop_right)
    structure = structure.union(bed).union(fixed_stop_a).union(fixed_stop_b)

    left_window = (
        cq.Workplane("XY")
        .box(0.108, 0.006, 0.042, centered=(False, True, False))
        .translate((0.092, ROTARY_SIDE_Y, 0.028))
    )
    right_window = (
        cq.Workplane("XY")
        .box(0.108, 0.006, 0.042, centered=(False, True, False))
        .translate((0.092, -ROTARY_SIDE_Y, 0.028))
    )
    top_cover_recess = (
        cq.Workplane("XY")
        .box(0.046, 0.040, 0.0020, centered=(False, True, False))
        .translate((0.010, 0.0, ROTARY_FLANGE_T + 0.040))
    )
    structure = structure.cut(left_window).cut(right_window).cut(top_cover_recess)

    for x in (0.018, 0.046):
        for y in (-0.014, 0.014):
            pocket = (
                cq.Workplane("XY")
                .circle(0.0027)
                .extrude(0.0014)
                .translate((x, y, ROTARY_FLANGE_T + 0.040 - 0.0014))
            )
            structure = structure.cut(pocket)

    return structure


def _make_rotary_rail_shape() -> cq.Workplane:
    rail_left = (
        cq.Workplane("XY")
        .box(RAIL_L, RAIL_W, RAIL_T, centered=(False, True, False))
        .translate((RAIL_X0, RAIL_Y, BED_Z0 + BED_T))
    )
    rail_right = (
        cq.Workplane("XY")
        .box(RAIL_L, RAIL_W, RAIL_T, centered=(False, True, False))
        .translate((RAIL_X0, -RAIL_Y, BED_Z0 + BED_T))
    )
    return rail_left.union(rail_right)


def _make_rotary_guide_shape() -> cq.Workplane:
    rod_left = _x_cylinder(GUIDE_ROD_R, GUIDE_ROD_L).translate((GUIDE_ROD_X0, GUIDE_ROD_Y, GUIDE_ROD_Z))
    rod_right = _x_cylinder(GUIDE_ROD_R, GUIDE_ROD_L).translate((GUIDE_ROD_X0, -GUIDE_ROD_Y, GUIDE_ROD_Z))
    return rod_left.union(rod_right)


def _make_rotary_drive_shape() -> cq.Workplane:
    leadscrew = _x_cylinder(LEADSCREW_R, LEADSCREW_L).translate((LEADSCREW_X0, 0.0, LEADSCREW_Z))
    drive_coupler = _x_cylinder(0.011, 0.020).translate((0.018, 0.0, LEADSCREW_Z))
    return leadscrew.union(drive_coupler)


def _make_rotary_module_shape() -> cq.Workplane:
    return (
        _make_rotary_structure_shape()
        .union(_make_rotary_rail_shape())
        .union(_make_rotary_guide_shape())
        .union(_make_rotary_drive_shape())
    )


def _make_carriage_body_shape() -> cq.Workplane:
    shoe_left = (
        cq.Workplane("XY")
        .box(CARRIAGE_L, CARRIAGE_SHOE_W, CARRIAGE_SHOE_Z, centered=(True, True, False))
        .translate((0.0, RAIL_Y, 0.0))
    )
    shoe_right = (
        cq.Workplane("XY")
        .box(CARRIAGE_L, CARRIAGE_SHOE_W, CARRIAGE_SHOE_Z, centered=(True, True, False))
        .translate((0.0, -RAIL_Y, 0.0))
    )
    rear_saddle = (
        cq.Workplane("XY")
        .box(0.052, 0.054, 0.024, centered=(True, True, False))
        .translate((-0.012, 0.0, CARRIAGE_SHOE_Z))
    )
    top_cover = (
        cq.Workplane("XY")
        .box(0.040, 0.030, 0.012, centered=(True, True, False))
        .translate((-0.010, 0.0, 0.034))
    )
    left_column = (
        cq.Workplane("XY")
        .box(0.048, 0.012, 0.038, centered=(False, True, False))
        .translate((0.000, 0.022, 0.010))
    )
    right_column = (
        cq.Workplane("XY")
        .box(0.048, 0.012, 0.038, centered=(False, True, False))
        .translate((0.000, -0.022, 0.010))
    )
    left_arm = (
        cq.Workplane("XY")
        .box(0.036, 0.010, 0.058, centered=(False, True, False))
        .translate((0.034, 0.021, 0.024))
    )
    right_arm = (
        cq.Workplane("XY")
        .box(0.036, 0.010, 0.058, centered=(False, True, False))
        .translate((0.034, -0.021, 0.024))
    )
    top_strap = (
        cq.Workplane("XY")
        .box(0.024, 0.018, 0.010, centered=(False, True, False))
        .translate((0.034, 0.0, 0.072))
    )
    lower_strap = (
        cq.Workplane("XY")
        .box(0.022, 0.018, 0.012, centered=(False, True, False))
        .translate((0.040, 0.0, 0.020))
    )
    lead_nut_block = (
        cq.Workplane("XY")
        .box(0.020, 0.018, 0.018, centered=(True, True, False))
        .translate((0.010, 0.0, 0.014))
    )
    left_gusset = (
        cq.Workplane("XZ")
        .polyline([(-0.020, 0.010), (0.004, 0.010), (0.018, 0.030), (-0.008, 0.030)])
        .close()
        .extrude(0.010)
        .translate((0.0, 0.012, 0.0))
    )
    right_gusset = left_gusset.mirror("XZ")
    lower_stop_left = (
        cq.Workplane("XY")
        .box(0.010, 0.004, 0.010, centered=(False, True, False))
        .translate((0.056, 0.012, 0.024))
    )
    lower_stop_right = (
        cq.Workplane("XY")
        .box(0.010, 0.004, 0.010, centered=(False, True, False))
        .translate((0.056, -0.012, 0.024))
    )
    upper_stop_left = (
        cq.Workplane("XY")
        .box(0.010, 0.004, 0.010, centered=(False, True, False))
        .translate((0.056, 0.012, 0.070))
    )
    upper_stop_right = (
        cq.Workplane("XY")
        .box(0.010, 0.004, 0.010, centered=(False, True, False))
        .translate((0.056, -0.012, 0.070))
    )
    nose_pad = (
        cq.Workplane("XY")
        .box(0.014, 0.026, 0.020, centered=(False, True, False))
        .translate((0.072, 0.0, 0.018))
    )

    carriage = shoe_left.union(shoe_right).union(rear_saddle).union(top_cover)
    carriage = carriage.union(left_column).union(right_column).union(left_arm).union(right_arm)
    carriage = carriage.union(top_strap).union(lower_strap).union(lead_nut_block).union(nose_pad)
    carriage = carriage.union(left_gusset).union(right_gusset)
    carriage = carriage.union(lower_stop_left).union(lower_stop_right)
    carriage = carriage.union(upper_stop_left).union(upper_stop_right)

    top_cover_recess = (
        cq.Workplane("XY")
        .box(0.026, 0.018, 0.0020, centered=(True, True, False))
        .translate((-0.010, 0.0, 0.046))
    )
    drive_relief = _x_cylinder(0.012, 0.060).translate(
        (-0.030, 0.0, LEADSCREW_Z - SLIDE_ORIGIN_Z)
    )
    carriage = carriage.cut(top_cover_recess).cut(drive_relief)

    for x in (-0.018, 0.010):
        for y in (-0.012, 0.012):
            pocket = (
                cq.Workplane("XY")
                .circle(0.0025)
                .extrude(0.0014)
                .translate((x, y, 0.046))
            )
            carriage = carriage.cut(pocket)

    return carriage


def _make_carriage_pin_shape() -> cq.Workplane:
    hinge_axle = _y_cylinder(WRIST_PIN_R, 0.052).translate((WRIST_ORIGIN_X, 0.026, WRIST_ORIGIN_Z))
    left_retainer = (
        cq.Workplane("XY")
        .box(0.010, 0.006, 0.010, centered=(True, True, True))
        .translate((WRIST_ORIGIN_X, 0.023, WRIST_ORIGIN_Z))
    )
    right_retainer = (
        cq.Workplane("XY")
        .box(0.010, 0.006, 0.010, centered=(True, True, True))
        .translate((WRIST_ORIGIN_X, -0.023, WRIST_ORIGIN_Z))
    )
    return hinge_axle.union(left_retainer).union(right_retainer)


def _make_carriage_shape() -> cq.Workplane:
    return _make_carriage_body_shape().union(_make_carriage_pin_shape())


def _make_wrist_hub_shape() -> cq.Workplane:
    return _y_cylinder(WRIST_HUB_R, WRIST_HUB_W).translate((0.0, WRIST_HUB_W * 0.5, 0.0))


def _make_wrist_arm_shape() -> cq.Workplane:
    root_block = (
        cq.Workplane("XY")
        .box(0.020, 0.012, 0.026, centered=(False, True, False))
        .translate((0.004, 0.0, -0.013))
    )
    plate = (
        cq.Workplane("XY")
        .box(0.056, 0.012, WRIST_PLATE_H, centered=(False, True, False))
        .translate((0.008, 0.0, WRIST_PLATE_Z0))
    )
    lower_web = (
        cq.Workplane("XZ")
        .polyline([(0.004, -0.010), (0.030, -0.010), (0.018, 0.006)])
        .close()
        .extrude(0.010)
        .translate((0.0, -0.005, 0.0))
    )
    upper_web = (
        cq.Workplane("XZ")
        .polyline([(0.010, 0.004), (0.038, 0.004), (0.024, 0.024)])
        .close()
        .extrude(0.010)
        .translate((0.0, -0.005, 0.0))
    )
    front_pad = (
        cq.Workplane("XY")
        .box(0.016, 0.016, 0.020, centered=(False, True, False))
        .translate((0.056, 0.0, -0.002))
    )
    stop_fin = (
        cq.Workplane("XY")
        .box(0.012, 0.010, 0.018, centered=(False, True, False))
        .translate((0.044, 0.0, 0.026))
    )

    arm = root_block.union(plate).union(lower_web).union(upper_web).union(front_pad).union(stop_fin)

    lightening_slot = (
        cq.Workplane("XY")
        .box(0.022, 0.008, 0.022, centered=(False, True, False))
        .translate((0.030, 0.0, 0.000))
    )
    arm = arm.cut(lightening_slot)

    for z in (-0.010, 0.012):
        mounting_hole = (
            cq.Workplane("YZ")
            .circle(0.0032)
            .extrude(0.022)
            .translate((0.054, 0.0, z))
        )
        arm = arm.cut(mounting_hole)
    return arm


def _make_wrist_shape() -> cq.Workplane:
    return _make_wrist_hub_shape().union(_make_wrist_arm_shape())


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mechanical_rpr_study", assets=ASSETS)

    dark_steel = model.material("dark_steel", rgba=(0.34, 0.36, 0.39, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.67, 0.69, 0.71, 1.0))
    aluminum = model.material("bead_blasted_aluminum", rgba=(0.76, 0.78, 0.80, 1.0))
    oxide = model.material("black_oxide", rgba=(0.16, 0.17, 0.18, 1.0))

    base_shape = _make_base_shape()
    rotary_structure_shape = _make_rotary_structure_shape()
    rotary_rail_shape = _make_rotary_rail_shape()
    rotary_guide_shape = _make_rotary_guide_shape()
    rotary_drive_shape = _make_rotary_drive_shape()
    carriage_shape = _make_carriage_shape()
    wrist_hub_shape = _make_wrist_hub_shape()
    wrist_arm_shape = _make_wrist_arm_shape()

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(base_shape, "base_study.obj", assets=ASSETS, tolerance=0.00025, angular_tolerance=0.05),
        material=dark_steel,
        name="base_shell",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_L, BASE_W, ROOT_Z)),
        mass=7.2,
        origin=Origin(xyz=(0.0, 0.0, ROOT_Z * 0.5)),
    )

    rotary_module = model.part("rotary_module")
    rotary_module.visual(
        mesh_from_cadquery(rotary_structure_shape, "rotary_structure.obj", assets=ASSETS, tolerance=0.0002, angular_tolerance=0.05),
        material=machined_steel,
        name="rotary_structure",
    )
    rotary_module.visual(
        mesh_from_cadquery(rotary_rail_shape, "rotary_rails.obj", assets=ASSETS, tolerance=0.00015, angular_tolerance=0.05),
        material=machined_steel,
        name="rotary_rails",
    )
    rotary_module.visual(
        mesh_from_cadquery(rotary_guide_shape, "rotary_guides.obj", assets=ASSETS, tolerance=0.00015, angular_tolerance=0.05),
        material=oxide,
        name="rotary_guides",
    )
    rotary_module.visual(
        mesh_from_cadquery(rotary_drive_shape, "rotary_drive.obj", assets=ASSETS, tolerance=0.00015, angular_tolerance=0.05),
        material=oxide,
        name="rotary_drive",
    )
    rotary_module.inertial = Inertial.from_geometry(
        Box((0.305, ROTARY_SIDE_Y * 2.0, 0.090)),
        mass=3.6,
        origin=Origin(xyz=(0.095, 0.0, 0.045)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(carriage_shape, "carriage_body.obj", assets=ASSETS, tolerance=0.00015, angular_tolerance=0.05),
        material=aluminum,
        name="carriage_body",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.090, 0.090, 0.086)),
        mass=1.7,
        origin=Origin(xyz=(0.018, 0.0, 0.043)),
    )

    wrist = model.part("wrist")
    wrist.visual(
        mesh_from_cadquery(wrist_hub_shape, "wrist_hub.obj", assets=ASSETS, tolerance=0.00012, angular_tolerance=0.05),
        material=oxide,
        name="wrist_hub",
    )
    wrist.visual(
        mesh_from_cadquery(wrist_arm_shape, "wrist_arm.obj", assets=ASSETS, tolerance=0.00015, angular_tolerance=0.05),
        material=oxide,
        name="wrist_arm",
    )
    wrist.inertial = Inertial.from_geometry(
        Box((0.082, 0.030, 0.070)),
        mass=0.8,
        origin=Origin(xyz=(0.028, 0.0, 0.004)),
    )

    model.articulation(
        "base_to_rotary",
        ArticulationType.REVOLUTE,
        parent=base,
        child=rotary_module,
        origin=Origin(xyz=(0.0, 0.0, ROOT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=1.4,
            lower=-1.10,
            upper=1.10,
        ),
    )
    model.articulation(
        "rotary_to_carriage",
        ArticulationType.PRISMATIC,
        parent=rotary_module,
        child=carriage,
        origin=Origin(xyz=(SLIDE_ORIGIN_X, 0.0, SLIDE_ORIGIN_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=0.20,
            lower=SLIDE_LOWER,
            upper=SLIDE_UPPER,
        ),
    )
    model.articulation(
        "carriage_to_wrist",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=wrist,
        origin=Origin(xyz=(WRIST_ORIGIN_X, 0.0, WRIST_ORIGIN_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.8,
            lower=-0.95,
            upper=0.95,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    rotary_module = object_model.get_part("rotary_module")
    carriage = object_model.get_part("carriage")
    wrist = object_model.get_part("wrist")
    base_shell = base.get_visual("base_shell")
    rotary_structure = rotary_module.get_visual("rotary_structure")
    rotary_rails = rotary_module.get_visual("rotary_rails")
    rotary_guides = rotary_module.get_visual("rotary_guides")
    rotary_drive = rotary_module.get_visual("rotary_drive")
    carriage_body = carriage.get_visual("carriage_body")
    wrist_hub = wrist.get_visual("wrist_hub")
    wrist_arm = wrist.get_visual("wrist_arm")
    base_to_rotary = object_model.get_articulation("base_to_rotary")
    rotary_to_carriage = object_model.get_articulation("rotary_to_carriage")
    carriage_to_wrist = object_model.get_articulation("carriage_to_wrist")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.allow_overlap(
        carriage,
        rotary_module,
        elem_a=carriage_body,
        elem_b=rotary_drive,
        reason="lead screw passes through the carriage nut pocket as an intentional nested drive engagement",
    )
    ctx.allow_overlap(
        carriage,
        wrist,
        elem_a=carriage_body,
        elem_b=wrist_hub,
        reason="the carriage body includes an integral hinge axle that intentionally passes through the wrist hub as a captured revolute joint",
    )

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
        "study_has_expected_parts",
        len(object_model.parts) == 4,
        f"expected 4 parts, found {len(object_model.parts)}",
    )
    ctx.check(
        "study_has_expected_articulations",
        len(object_model.articulations) == 3,
        f"expected 3 articulations, found {len(object_model.articulations)}",
    )
    ctx.check(
        "articulation_chain_types",
        (
            base_to_rotary.articulation_type == ArticulationType.REVOLUTE
            and rotary_to_carriage.articulation_type == ArticulationType.PRISMATIC
            and carriage_to_wrist.articulation_type == ArticulationType.REVOLUTE
        ),
        "assembly must remain a revolute-prismatic-revolute chain",
    )
    ctx.check(
        "articulation_axes_are_discrete",
        (
            tuple(round(v, 6) for v in base_to_rotary.axis) == (0.0, 0.0, 1.0)
            and tuple(round(v, 6) for v in rotary_to_carriage.axis) == (1.0, 0.0, 0.0)
            and tuple(round(v, 6) for v in carriage_to_wrist.axis) == (0.0, 1.0, 0.0)
        ),
        "expected vertical rotary axis, longitudinal slide axis, and lateral wrist axis",
    )

    ctx.expect_contact(
        rotary_module,
        base,
        contact_tol=5e-4,
        elem_a=rotary_structure,
        elem_b=base_shell,
        name="rotary_module_seats_on_base_turntable",
    )
    ctx.expect_origin_distance(
        rotary_module,
        base,
        axes="xy",
        max_dist=1e-3,
        name="rotary_axis_stays_centered_over_base",
    )
    ctx.expect_contact(
        carriage,
        rotary_module,
        contact_tol=5e-4,
        elem_a=carriage_body,
        elem_b=rotary_rails,
        name="carriage_is_supported_by_linear_stage",
    )
    ctx.expect_origin_distance(
        carriage,
        rotary_module,
        axes="y",
        max_dist=1e-3,
        name="carriage_stays_between_side_plates",
    )
    ctx.expect_overlap(
        carriage,
        rotary_module,
        axes="xy",
        min_overlap=0.050,
        name="carriage_bridges_rotary_linear_frame",
    )
    ctx.expect_contact(
        wrist,
        carriage,
        contact_tol=5e-4,
        elem_a=wrist_hub,
        elem_b=carriage_body,
        name="wrist_is_captured_in_distal_clevis",
    )
    ctx.expect_origin_distance(
        wrist,
        carriage,
        axes="y",
        max_dist=1e-3,
        name="wrist_hinge_is_centered_in_clevis",
    )
    ctx.expect_overlap(
        wrist,
        carriage,
        axes="yz",
        min_overlap=0.020,
        name="wrist_hinge_has_supported_overlap_with_clevis",
    )

    with ctx.pose({base_to_rotary: 0.90}):
        ctx.expect_contact(
            rotary_module,
            base,
            contact_tol=5e-4,
            elem_a=rotary_structure,
            elem_b=base_shell,
            name="rotary_module_remains_supported_when_slewed",
        )
        ctx.expect_origin_distance(
            rotary_module,
            base,
            axes="xy",
            max_dist=1e-3,
            name="rotary_joint_keeps_module_on_axis_when_slewed",
        )
        ctx.expect_contact(
            carriage,
            rotary_module,
            contact_tol=5e-4,
            elem_a=carriage_body,
            elem_b=rotary_rails,
            name="linear_stage_support_persists_when_slewed",
        )

    with ctx.pose({rotary_to_carriage: SLIDE_LOWER}):
        ctx.expect_contact(
            carriage,
            rotary_module,
            contact_tol=5e-4,
            elem_a=carriage_body,
            elem_b=rotary_rails,
            name="carriage_remains_supported_retracted",
        )
        ctx.expect_origin_gap(
            carriage,
            rotary_module,
            axis="x",
            min_gap=0.114,
            max_gap=0.116,
            name="carriage_retracted_position_matches_slide_origin",
        )

    with ctx.pose({rotary_to_carriage: SLIDE_UPPER}):
        ctx.expect_contact(
            carriage,
            rotary_module,
            contact_tol=5e-4,
            elem_a=carriage_body,
            elem_b=rotary_rails,
            name="carriage_remains_supported_extended",
        )
        ctx.expect_origin_gap(
            carriage,
            rotary_module,
            axis="x",
            min_gap=0.204,
            max_gap=0.206,
            name="carriage_extended_position_matches_slide_travel",
        )
        ctx.expect_overlap(
            carriage,
            rotary_module,
            axes="y",
            min_overlap=0.060,
            name="carriage_stays_within_guide_span_at_full_extension",
        )

    with ctx.pose({carriage_to_wrist: 0.75}):
        ctx.expect_contact(
            wrist,
            carriage,
            contact_tol=5e-4,
            elem_a=wrist_hub,
            elem_b=carriage_body,
            name="wrist_stays_seated_at_positive_pitch",
        )
        ctx.expect_origin_distance(
            wrist,
            carriage,
            axes="y",
            max_dist=1e-3,
            name="wrist_axis_stays_laterally_centered_at_positive_pitch",
        )

    with ctx.pose({carriage_to_wrist: -0.75}):
        ctx.expect_contact(
            wrist,
            carriage,
            contact_tol=5e-4,
            elem_a=wrist_hub,
            elem_b=carriage_body,
            name="wrist_stays_seated_at_negative_pitch",
        )
        ctx.expect_origin_distance(
            wrist,
            carriage,
            axes="y",
            max_dist=1e-3,
            name="wrist_axis_stays_laterally_centered_at_negative_pitch",
        )

    base_aabb = ctx.part_world_aabb(base)
    rotary_aabb = ctx.part_world_aabb(rotary_module)
    carriage_aabb = ctx.part_world_aabb(carriage)
    if base_aabb and rotary_aabb and carriage_aabb:
        base_span_x = base_aabb[1][0] - base_aabb[0][0]
        rotary_span_x = rotary_aabb[1][0] - rotary_aabb[0][0]
        carriage_span_x = carriage_aabb[1][0] - carriage_aabb[0][0]
        ctx.check(
            "mechanical_proportions_read_as_supported_chain",
            rotary_span_x > carriage_span_x * 1.8 and base_span_x > rotary_span_x * 0.9,
            (
                "base, rotary frame, and carriage should read as a supported mechanical chain "
                f"(base={base_span_x:.3f}, rotary={rotary_span_x:.3f}, carriage={carriage_span_x:.3f})"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
