from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import math

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


ROLL_LIMIT = 0.62
PITCH_LIMIT = 0.70

BASE_X = 0.340
BASE_Y = 0.220
BASE_T = 0.016
BASE_Z = -0.132
BASE_TOP = BASE_Z + BASE_T / 2.0

PED_X = 0.036
PED_Y = 0.102
PED_Z = 0.132
PED_XC = 0.130
PED_ZC = BASE_TOP + PED_Z / 2.0
PED_INNER_FACE = PED_XC - PED_X / 2.0
PED_OUTER_FACE = PED_XC + PED_X / 2.0

ROLL_CAP_T = 0.012
ROLL_CAP_Y = 0.074
ROLL_CAP_Z = 0.064
ROLL_CAP_XC = PED_OUTER_FACE
ROLL_CAP_OUTER_FACE = PED_OUTER_FACE + ROLL_CAP_T

ROLL_JOURNAL_R = 0.0195
ROLL_BORE_R = 0.0210
ROLL_CAP_BORE_R = 0.0210
ROLL_SHOULDER_R = 0.0285
ROLL_COLLAR_R = 0.0250
ROLL_ENDCAP_R = 0.0210
ROLL_SHOULDER_X0 = 0.067
ROLL_SHOULDER_X1 = 0.110
ROLL_JOURNAL_X1 = 0.160
ROLL_COLLAR_X1 = 0.168
ROLL_ENDCAP_X1 = 0.173

FRAME_X = 0.136
FRAME_Z = 0.176
FRAME_CHEEK_YC = 0.068
FRAME_CHEEK_T = 0.012
FRAME_TOP_Z = 0.086
FRAME_BOT_Z = -0.086
FRAME_BRIDGE_Y = 0.100
FRAME_BRIDGE_T = 0.012
FRAME_WINDOW_X = 0.108
FRAME_WINDOW_Z = 0.148
FRAME_OUTER_Y = FRAME_CHEEK_YC + FRAME_CHEEK_T / 2.0

PITCH_COVER_T = 0.010
PITCH_COVER_X = 0.078
PITCH_COVER_Z = 0.060
PITCH_COVER_YC = FRAME_OUTER_Y + PITCH_COVER_T
PITCH_COVER_OUTER_FACE = FRAME_OUTER_Y + 2.0 * PITCH_COVER_T

PITCH_JOURNAL_R = 0.0140
PITCH_BORE_R = 0.0158
PITCH_COVER_BORE_R = 0.0190
PITCH_SHOULDER_R = 0.0215
PITCH_COLLAR_R = 0.0195
PITCH_ENDCAP_R = 0.0165
PITCH_SHOULDER_Y0 = 0.045
PITCH_SHOULDER_Y1 = 0.052
PITCH_JOURNAL_Y1 = PITCH_COVER_OUTER_FACE
PITCH_COLLAR_Y1 = 0.091
PITCH_ENDCAP_Y1 = 0.096

CRADLE_ARM_XC = 0.035
CRADLE_ARM_T = 0.012
CRADLE_ARM_Y = 0.086
CRADLE_ARM_Z = 0.086
CRADLE_CROSS_Y = 0.058
CRADLE_CROSS_T = 0.012
CRADLE_TOP_Z = 0.044
CRADLE_BOT_Z = -0.044
CRADLE_PLATE_X = 0.054
CRADLE_PLATE_Y = 0.078
CRADLE_PLATE_ZC = -0.006
CRADLE_PLATE_T = 0.008
CRADLE_TRUNNION_BLOCK_X = 0.040
CRADLE_TRUNNION_BLOCK_Y = 0.020
CRADLE_TRUNNION_BLOCK_Z = 0.044
CRADLE_TRUNNION_BLOCK_YC = 0.046


def _cyl_x(radius: float, x0: float, x1: float) -> cq.Workplane:
    start, end = sorted((x0, x1))
    return cq.Workplane("YZ").circle(radius).extrude(end - start).translate((start, 0.0, 0.0))


def _cyl_y(radius: float, y0: float, y1: float) -> cq.Workplane:
    start, end = sorted((y0, y1))
    return cq.Workplane("XZ").circle(radius).extrude(end - start).translate((0.0, start, 0.0))


def _pedestal_gusset(sign: float, y_center: float) -> cq.Workplane:
    pts = [
        (sign * (PED_INNER_FACE - 0.002), BASE_TOP),
        (sign * (PED_INNER_FACE + 0.018), BASE_TOP),
        (sign * (PED_OUTER_FACE - 0.004), -0.024),
        (sign * (PED_OUTER_FACE - 0.004), -0.092),
        (sign * (PED_INNER_FACE + 0.012), -0.092),
    ]
    return (
        cq.Workplane("XZ")
        .polyline(pts)
        .close()
        .extrude(0.018)
        .translate((0.0, y_center - 0.009, 0.0))
    )


def _make_base_plate() -> cq.Workplane:
    body = cq.Workplane("XY").box(BASE_X, BASE_Y, BASE_T).translate((0.0, 0.0, BASE_Z))
    for y_center in (-0.074, 0.074):
        body = body.union(cq.Workplane("XY").box(0.248, 0.026, 0.018).translate((0.0, y_center, -0.114)))
    body = body.union(cq.Workplane("XY").box(0.158, 0.036, 0.020).translate((0.0, 0.0, -0.108)))

    slot_cut = (
        cq.Workplane("XY")
        .pushPoints([(-0.110, -0.072), (-0.110, 0.072), (0.110, -0.072), (0.110, 0.072)])
        .slot2D(0.030, 0.012, 90.0)
        .extrude(0.050, both=True)
        .translate((0.0, 0.0, BASE_Z))
    )
    body = body.cut(slot_cut)
    body = body.cut(cq.Workplane("XY").box(0.112, 0.082, 0.022).translate((0.0, 0.0, -0.103)))
    return body


def _make_base_pedestal(sign: float) -> cq.Workplane:
    body = cq.Workplane("XY").box(PED_X, PED_Y, PED_Z).translate((sign * PED_XC, 0.0, PED_ZC))
    body = body.union(cq.Workplane("XY").box(0.022, 0.060, 0.030).translate((sign * (PED_INNER_FACE + 0.011), 0.0, -0.086)))
    body = body.union(cq.Workplane("XY").box(0.018, 0.078, 0.020).translate((sign * (PED_OUTER_FACE - 0.009), 0.0, 0.000)))
    for y_center in (-0.032, 0.032):
        body = body.union(_pedestal_gusset(sign, y_center))

    body = body.cut(_cyl_x(ROLL_BORE_R, sign * (PED_INNER_FACE - 0.002), sign * (PED_OUTER_FACE + 0.002)))
    body = body.cut(_cyl_x(0.029, sign * (PED_OUTER_FACE - 0.008), sign * (PED_OUTER_FACE + 0.002)))
    for y_pos, z_pos in ((-0.024, -0.020), (-0.024, 0.020), (0.024, -0.020), (0.024, 0.020)):
        body = body.cut(
            _cyl_x(0.0034, sign * (PED_OUTER_FACE - 0.013), sign * (PED_OUTER_FACE + 0.001)).translate((0.0, y_pos, z_pos))
        )
    return body


def _make_roll_cap() -> cq.Workplane:
    body = cq.Workplane("YZ").rect(ROLL_CAP_Y, ROLL_CAP_Z).extrude(ROLL_CAP_T)
    body = body.union(
        cq.Workplane("YZ").pushPoints([(-0.026, 0.0), (0.026, 0.0)]).circle(0.013).extrude(ROLL_CAP_T)
    )
    body = body.union(_cyl_x(0.029, ROLL_CAP_T - 0.003, ROLL_CAP_T))
    body = body.cut(_cyl_x(ROLL_CAP_BORE_R, -0.001, ROLL_CAP_T + 0.001))
    for y_pos, z_pos in ((-0.022, -0.018), (-0.022, 0.018), (0.022, -0.018), (0.022, 0.018)):
        body = body.cut(_cyl_x(0.0032, -0.001, ROLL_CAP_T + 0.001).translate((0.0, y_pos, z_pos)))
    for zc, width in ((0.020, 0.006), (0.028, 0.010), (0.036, 0.014)):
        body = body.union(cq.Workplane("XY").box(0.0016, width, 0.0022).translate((ROLL_CAP_T - 0.0008, 0.0, zc)))
    body = body.cut(cq.Workplane("XY").box(0.003, 0.018, 0.010).translate((ROLL_CAP_T - 0.0015, 0.0, -0.026)))
    return body


def _make_roll_yoke() -> cq.Workplane:
    body = cq.Workplane("XY").box(FRAME_X, FRAME_CHEEK_T, FRAME_Z).translate((0.0, -FRAME_CHEEK_YC, 0.0))
    body = body.union(cq.Workplane("XY").box(FRAME_X, FRAME_CHEEK_T, FRAME_Z).translate((0.0, FRAME_CHEEK_YC, 0.0)))
    body = body.union(cq.Workplane("XY").box(FRAME_X, FRAME_BRIDGE_Y, FRAME_BRIDGE_T).translate((0.0, 0.0, FRAME_TOP_Z)))
    body = body.union(cq.Workplane("XY").box(FRAME_X, FRAME_BRIDGE_Y, FRAME_BRIDGE_T).translate((0.0, 0.0, FRAME_BOT_Z)))
    body = body.union(cq.Workplane("XY").box(0.022, 0.052, 0.048).translate((0.040, 0.0, 0.0)))
    body = body.union(cq.Workplane("XY").box(0.022, 0.052, 0.048).translate((-0.040, 0.0, 0.0)))
    for y_center in (-FRAME_CHEEK_YC, FRAME_CHEEK_YC):
        body = body.union(_cyl_y(0.030, y_center - FRAME_CHEEK_T / 2.0 - 0.001, y_center + FRAME_CHEEK_T / 2.0 + 0.001))

    body = body.cut(cq.Workplane("XY").box(FRAME_WINDOW_X, 0.180, FRAME_WINDOW_Z))
    body = body.cut(cq.Workplane("XY").box(0.046, 0.180, 0.032).translate((0.0, 0.0, 0.054)))
    body = body.cut(cq.Workplane("XY").box(0.046, 0.180, 0.032).translate((0.0, 0.0, -0.054)))
    body = body.cut(_cyl_y(PITCH_BORE_R, -0.090, 0.090))
    return body


def _make_roll_journal(sign: float) -> cq.Workplane:
    body = _cyl_x(ROLL_SHOULDER_R, sign * ROLL_SHOULDER_X0, sign * ROLL_SHOULDER_X1)
    body = body.union(_cyl_x(ROLL_JOURNAL_R, sign * ROLL_SHOULDER_X1, sign * ROLL_JOURNAL_X1))
    return body


def _make_roll_collar(sign: float) -> cq.Workplane:
    body = _cyl_x(ROLL_COLLAR_R, sign * ROLL_JOURNAL_X1, sign * ROLL_COLLAR_X1)
    body = body.union(_cyl_x(ROLL_ENDCAP_R, sign * ROLL_COLLAR_X1, sign * ROLL_ENDCAP_X1))
    ear_x = sign * 0.1645
    body = body.union(cq.Workplane("XY").box(0.008, 0.012, 0.008).translate((ear_x, 0.0, 0.020)))
    body = body.union(cq.Workplane("XY").box(0.008, 0.012, 0.008).translate((ear_x, 0.0, -0.020)))
    body = body.cut(cq.Workplane("XY").box(0.010, 0.040, 0.002).translate((ear_x, 0.0, 0.024)))
    return body


def _make_pitch_cover() -> cq.Workplane:
    body = cq.Workplane("XZ").rect(PITCH_COVER_X, PITCH_COVER_Z).extrude(PITCH_COVER_T)
    body = body.cut(
        _cyl_y(PITCH_COVER_BORE_R, -0.001, PITCH_COVER_T + 0.001)
    )
    body = body.union(cq.Workplane("XZ").pushPoints([(-0.028, 0.0), (0.028, 0.0)]).circle(0.012).extrude(PITCH_COVER_T))
    body = body.union(
        _cyl_y(0.025, PITCH_COVER_T - 0.003, PITCH_COVER_T)
    )
    for x_pos, z_pos in ((-0.022, -0.018), (-0.022, 0.018), (0.022, -0.018), (0.022, 0.018), (0.0, -0.024)):
        body = body.union(
            cq.Workplane("XZ").center(x_pos, z_pos).circle(0.0040).extrude(0.004)
        )
    for x_pos, z_pos in ((-0.022, -0.018), (-0.022, 0.018), (0.022, -0.018), (0.022, 0.018), (0.0, -0.024)):
        body = body.cut(_cyl_y(0.0029, -0.001, PITCH_COVER_T + 0.001).translate((x_pos, 0.0, z_pos)))
    for xc, length in ((0.020, 0.006), (0.028, 0.010), (0.036, 0.014)):
        body = body.union(cq.Workplane("XY").box(length, 0.0016, 0.0022).translate((xc, PITCH_COVER_T - 0.0008, 0.030)))
    body = body.cut(cq.Workplane("XY").box(0.012, 0.003, 0.010).translate((-0.028, PITCH_COVER_T - 0.0015, -0.024)))
    return body


def _make_cradle_body() -> cq.Workplane:
    body = cq.Workplane("XY").box(CRADLE_ARM_T, CRADLE_ARM_Y, CRADLE_ARM_Z).translate((-CRADLE_ARM_XC, 0.0, 0.0))
    body = body.union(cq.Workplane("XY").box(CRADLE_ARM_T, CRADLE_ARM_Y, CRADLE_ARM_Z).translate((CRADLE_ARM_XC, 0.0, 0.0)))
    body = body.union(cq.Workplane("XY").box(0.084, CRADLE_CROSS_Y, CRADLE_CROSS_T).translate((0.0, 0.0, CRADLE_TOP_Z)))
    body = body.union(cq.Workplane("XY").box(0.084, CRADLE_CROSS_Y, CRADLE_CROSS_T).translate((0.0, 0.0, CRADLE_BOT_Z)))
    body = body.union(cq.Workplane("XY").box(CRADLE_PLATE_X, CRADLE_PLATE_Y, CRADLE_PLATE_T).translate((0.0, 0.0, CRADLE_PLATE_ZC)))

    for y_center in (-CRADLE_TRUNNION_BLOCK_YC, CRADLE_TRUNNION_BLOCK_YC):
        body = body.union(
            cq.Workplane("XY")
            .box(CRADLE_TRUNNION_BLOCK_X, CRADLE_TRUNNION_BLOCK_Y, CRADLE_TRUNNION_BLOCK_Z)
            .translate((0.0, y_center, 0.0))
        )

    body = body.union(cq.Workplane("XY").box(0.010, 0.074, 0.010).translate((-0.026, 0.0, 0.012)))
    body = body.union(cq.Workplane("XY").box(0.010, 0.074, 0.010).translate((0.026, 0.0, 0.012)))
    body = body.union(cq.Workplane("XY").box(0.010, 0.042, 0.026).translate((-0.026, 0.0, 0.028)))
    body = body.union(cq.Workplane("XY").box(0.010, 0.042, 0.026).translate((0.026, 0.0, 0.028)))

    body = body.cut(cq.Workplane("XY").box(0.050, 0.132, 0.062).translate((0.0, 0.0, 0.010)))
    body = body.cut(
        cq.Workplane("XY")
        .pushPoints([(-0.018, 0.0), (0.018, 0.0)])
        .slot2D(0.040, 0.010, 90.0)
        .extrude(0.024, both=True)
        .translate((0.0, 0.0, CRADLE_PLATE_ZC))
    )
    return body


def _make_pitch_journal(sign: float) -> cq.Workplane:
    body = _cyl_y(PITCH_SHOULDER_R, sign * PITCH_SHOULDER_Y0, sign * PITCH_SHOULDER_Y1)
    body = body.union(_cyl_y(PITCH_JOURNAL_R, sign * PITCH_SHOULDER_Y1, sign * PITCH_JOURNAL_Y1))
    return body


def _make_pitch_collar(sign: float) -> cq.Workplane:
    body = _cyl_y(PITCH_COLLAR_R, sign * PITCH_JOURNAL_Y1, sign * PITCH_COLLAR_Y1)
    body = body.union(_cyl_y(PITCH_ENDCAP_R, sign * PITCH_COLLAR_Y1, sign * PITCH_ENDCAP_Y1))
    ear_y = sign * 0.088
    body = body.union(cq.Workplane("XY").box(0.008, 0.008, 0.010).translate((0.018, ear_y, 0.0)))
    body = body.union(cq.Workplane("XY").box(0.008, 0.008, 0.010).translate((-0.018, ear_y, 0.0)))
    body = body.cut(cq.Workplane("XY").box(0.036, 0.010, 0.002).translate((0.0, ear_y, 0.022)))
    return body


def _add_mesh_visual(part, shape: cq.Workplane, filename: str, *, material, name: str) -> None:
    part.visual(mesh_from_cadquery(shape, filename, assets=ASSETS), material=material, name=name)


STUDY_BASE_X = 0.290
STUDY_BASE_Y = 0.190
STUDY_BASE_T = 0.018
STUDY_BASE_Z = -0.120
STUDY_BASE_TOP = STUDY_BASE_Z + STUDY_BASE_T / 2.0

ROLL_AXIS_Z = 0.058
PED_X = 0.030
PED_Y = 0.088
PED_Z = 0.190
PED_XC = 0.102
PED_ZC = STUDY_BASE_TOP + PED_Z / 2.0
PED_OUTER_FACE = PED_XC + PED_X / 2.0

ROLL_CAP_T2 = 0.012
ROLL_CAP_CENTER_X = PED_OUTER_FACE + ROLL_CAP_T2 / 2.0
ROLL_JOURNAL_R2 = 0.0190
ROLL_BORE_R2 = 0.0190
ROLL_SHOULDER_R2 = 0.0280
ROLL_COLLAR_R2 = 0.0250
ROLL_ENDCAP_R2 = 0.0210
ROLL_BODY_OUTER_X = 0.055
ROLL_JOURNAL_INNER_X = 0.087
ROLL_JOURNAL_OUTER_X = 0.129
ROLL_COLLAR_OUTER_X = 0.137
ROLL_ENDCAP_OUTER_X = 0.141

YOKE_X = 0.110
YOKE_CHEEK_YC = 0.056
YOKE_CHEEK_T = 0.012
YOKE_OUTER_Y = YOKE_CHEEK_YC + YOKE_CHEEK_T / 2.0

PITCH_COVER_T2 = 0.010
PITCH_COVER_CENTER_Y = YOKE_OUTER_Y + PITCH_COVER_T2 / 2.0
PITCH_JOURNAL_R2 = 0.0140
PITCH_BORE_R2 = 0.0220
PITCH_SHOULDER_R2 = 0.0215
PITCH_COLLAR_R2 = 0.0195
PITCH_ENDCAP_R2 = 0.0165
PITCH_BODY_OUTER_Y = 0.032
PITCH_SHOULDER_OUTER_Y = 0.050
PITCH_JOURNAL_OUTER_Y = 0.072
PITCH_COLLAR_OUTER_Y = 0.080
PITCH_ENDCAP_OUTER_Y = 0.084


def _study_pedestal_gusset(sign: float, y_center: float) -> cq.Workplane:
    pts = [
        (sign * (PED_XC - 0.010), STUDY_BASE_TOP),
        (sign * (PED_XC + 0.002), STUDY_BASE_TOP),
        (sign * (PED_OUTER_FACE - 0.004), ROLL_AXIS_Z - 0.018),
        (sign * (PED_OUTER_FACE - 0.004), ROLL_AXIS_Z - 0.054),
        (sign * (PED_XC + 0.006), ROLL_AXIS_Z - 0.054),
    ]
    return (
        cq.Workplane("XZ")
        .polyline(pts)
        .close()
        .extrude(0.016)
        .translate((0.0, y_center - 0.008, 0.0))
    )


def _make_study_base_plate() -> cq.Workplane:
    body = cq.Workplane("XY").box(STUDY_BASE_X, STUDY_BASE_Y, STUDY_BASE_T).translate((0.0, 0.0, STUDY_BASE_Z))
    for y_center in (-0.068, 0.068):
        body = body.union(cq.Workplane("XY").box(0.224, 0.024, 0.018).translate((0.0, y_center, -0.108)))
    body = body.union(cq.Workplane("XY").box(0.120, 0.040, 0.014).translate((0.0, 0.0, -0.109)))
    slot_cut = (
        cq.Workplane("XY")
        .pushPoints([(-0.098, -0.064), (-0.098, 0.064), (0.098, -0.064), (0.098, 0.064)])
        .slot2D(0.032, 0.012, 90.0)
        .extrude(0.050, both=True)
        .translate((0.0, 0.0, STUDY_BASE_Z))
    )
    body = body.cut(slot_cut)
    body = body.cut(cq.Workplane("XY").box(0.094, 0.070, 0.016).translate((0.0, 0.0, -0.109)))
    return body


def _make_study_pedestal(sign: float) -> cq.Workplane:
    body = cq.Workplane("XY").box(PED_X, PED_Y, PED_Z).translate((sign * PED_XC, 0.0, PED_ZC))
    body = body.union(cq.Workplane("XY").box(0.018, 0.060, 0.044).translate((sign * (PED_OUTER_FACE - 0.009), 0.0, ROLL_AXIS_Z)))
    body = body.union(cq.Workplane("XY").box(0.016, 0.048, 0.052).translate((sign * (PED_XC - 0.004), 0.0, -0.010)))
    for y_center in (-0.026, 0.026):
        body = body.union(_study_pedestal_gusset(sign, y_center))
    body = body.cut(
        _cyl_x(0.0295, sign * (PED_XC - PED_X / 2.0 - 0.002), sign * (PED_XC + PED_X / 2.0 + 0.002)).translate((0.0, 0.0, ROLL_AXIS_Z))
    )
    body = body.cut(
        _cyl_x(0.0285, sign * (PED_OUTER_FACE - 0.010), sign * (PED_OUTER_FACE + 0.002)).translate((0.0, 0.0, ROLL_AXIS_Z))
    )
    for y_pos, z_pos in ((-0.023, -0.020), (-0.023, 0.020), (0.023, -0.020), (0.023, 0.020)):
        body = body.cut(
            _cyl_x(0.0032, sign * (PED_OUTER_FACE - 0.013), sign * (PED_OUTER_FACE + 0.002)).translate((0.0, y_pos, ROLL_AXIS_Z + z_pos))
        )
    return body


def _make_study_roll_cap() -> cq.Workplane:
    body = cq.Workplane("YZ").rect(0.080, 0.078).extrude(ROLL_CAP_T2).translate((-ROLL_CAP_T2 / 2.0, 0.0, 0.0))
    body = body.union(
        cq.Workplane("YZ").pushPoints([(-0.026, 0.0), (0.026, 0.0)]).circle(0.013).extrude(ROLL_CAP_T2).translate((-ROLL_CAP_T2 / 2.0, 0.0, 0.0))
    )
    body = body.cut(_cyl_x(0.0205, -ROLL_CAP_T2 / 2.0 - 0.001, ROLL_CAP_T2 / 2.0 + 0.001))
    for y_pos, z_pos in ((-0.023, -0.020), (-0.023, 0.020), (0.023, -0.020), (0.023, 0.020)):
        body = body.cut(_cyl_x(0.0032, -ROLL_CAP_T2 / 2.0 - 0.001, ROLL_CAP_T2 / 2.0 + 0.001).translate((0.0, y_pos, z_pos)))
    for zc, width in ((0.017, 0.006), (0.024, 0.010), (0.031, 0.014)):
        body = body.union(cq.Workplane("XY").box(0.0016, width, 0.0020).translate((ROLL_CAP_T2 / 2.0 - 0.0008, 0.0, zc)))
    body = body.cut(cq.Workplane("XY").box(0.003, 0.018, 0.010).translate((ROLL_CAP_T2 / 2.0 - 0.0015, 0.0, -0.024)))
    return body


def _make_study_roll_yoke() -> cq.Workplane:
    body = cq.Workplane("XY").box(YOKE_X, YOKE_CHEEK_T, 0.140).translate((0.0, -YOKE_CHEEK_YC, 0.0))
    body = body.union(cq.Workplane("XY").box(YOKE_X, YOKE_CHEEK_T, 0.140).translate((0.0, YOKE_CHEEK_YC, 0.0)))
    body = body.union(cq.Workplane("XY").box(YOKE_X, 0.100, 0.012).translate((0.0, 0.0, 0.054)))
    body = body.union(cq.Workplane("XY").box(YOKE_X, 0.100, 0.012).translate((0.0, 0.0, -0.054)))
    body = body.union(cq.Workplane("XY").box(0.014, 0.042, 0.048).translate((0.048, 0.0, 0.0)))
    body = body.union(cq.Workplane("XY").box(0.014, 0.042, 0.048).translate((-0.048, 0.0, 0.0)))
    body = body.union(_cyl_y(0.026, -0.062, -0.050))
    body = body.union(_cyl_y(0.026, 0.050, 0.062))
    body = body.cut(cq.Workplane("XY").box(0.072, 0.150, 0.098))
    body = body.cut(cq.Workplane("XY").box(0.040, 0.150, 0.024).translate((0.0, 0.0, 0.050)))
    body = body.cut(cq.Workplane("XY").box(0.040, 0.150, 0.024).translate((0.0, 0.0, -0.050)))
    body = body.cut(_cyl_y(0.0220, -0.070, 0.070))
    return body


def _make_study_roll_journal(sign: float) -> cq.Workplane:
    body = cq.Workplane("XY").box(0.016, 0.046, 0.052).translate((sign * 0.049, 0.0, 0.0))
    body = body.union(_cyl_x(ROLL_SHOULDER_R2, sign * 0.050, sign * ROLL_JOURNAL_INNER_X))
    body = body.union(_cyl_x(ROLL_JOURNAL_R2, sign * ROLL_JOURNAL_INNER_X, sign * ROLL_JOURNAL_OUTER_X))
    return body


def _make_study_roll_collar(sign: float) -> cq.Workplane:
    body = _cyl_x(ROLL_COLLAR_R2, sign * ROLL_JOURNAL_OUTER_X, sign * ROLL_COLLAR_OUTER_X)
    body = body.union(_cyl_x(ROLL_ENDCAP_R2, sign * ROLL_COLLAR_OUTER_X, sign * ROLL_ENDCAP_OUTER_X))
    ear_x = sign * 0.133
    body = body.union(cq.Workplane("XY").box(0.008, 0.012, 0.008).translate((ear_x, 0.0, 0.019)))
    body = body.union(cq.Workplane("XY").box(0.008, 0.012, 0.008).translate((ear_x, 0.0, -0.019)))
    body = body.cut(cq.Workplane("XY").box(0.010, 0.036, 0.002).translate((ear_x, 0.0, 0.022)))
    return body


def _make_study_pitch_cover() -> cq.Workplane:
    body = cq.Workplane("XY").box(0.086, PITCH_COVER_T2, 0.070)
    body = body.union(cq.Workplane("XY").box(0.024, PITCH_COVER_T2, 0.024).translate((-0.028, 0.0, 0.0)))
    body = body.union(cq.Workplane("XY").box(0.024, PITCH_COVER_T2, 0.024).translate((0.028, 0.0, 0.0)))
    body = body.cut(_cyl_y(PITCH_BORE_R2, -PITCH_COVER_T2 / 2.0 - 0.001, PITCH_COVER_T2 / 2.0 + 0.001))
    for x_pos, z_pos in ((-0.022, -0.020), (-0.022, 0.020), (0.022, -0.020), (0.022, 0.020)):
        body = body.cut(_cyl_y(0.0030, -PITCH_COVER_T2 / 2.0 - 0.001, PITCH_COVER_T2 / 2.0 + 0.001).translate((x_pos, 0.0, z_pos)))
    for xc, length in ((0.018, 0.006), (0.026, 0.010), (0.034, 0.014)):
        body = body.union(cq.Workplane("XY").box(length, 0.0016, 0.0020).translate((xc, PITCH_COVER_T2 / 2.0 - 0.0008, 0.028)))
    body = body.cut(cq.Workplane("XY").box(0.012, 0.003, 0.010).translate((-0.028, 0.0, -0.024)))
    return body


def _make_study_cradle_body() -> cq.Workplane:
    body = cq.Workplane("XY").box(0.010, 0.058, 0.068).translate((-0.026, 0.0, 0.0))
    body = body.union(cq.Workplane("XY").box(0.010, 0.058, 0.068).translate((0.026, 0.0, 0.0)))
    body = body.union(cq.Workplane("XY").box(0.052, 0.046, 0.010).translate((0.0, 0.0, 0.032)))
    body = body.union(cq.Workplane("XY").box(0.052, 0.046, 0.010).translate((0.0, 0.0, -0.032)))
    body = body.union(cq.Workplane("XY").box(0.046, 0.062, 0.008).translate((0.0, 0.0, -0.004)))
    body = body.union(cq.Workplane("XY").box(0.008, 0.038, 0.020).translate((-0.016, 0.0, 0.014)))
    body = body.union(cq.Workplane("XY").box(0.008, 0.038, 0.020).translate((0.016, 0.0, 0.014)))
    body = body.cut(cq.Workplane("XY").box(0.028, 0.110, 0.034).translate((0.0, 0.0, 0.008)))
    body = body.cut(
        cq.Workplane("XY")
        .pushPoints([(-0.016, 0.0), (0.016, 0.0)])
        .slot2D(0.032, 0.008, 90.0)
        .extrude(0.020, both=True)
        .translate((0.0, 0.0, -0.004))
    )
    return body


def _make_study_pitch_journal(sign: float) -> cq.Workplane:
    body = cq.Workplane("XY").box(0.030, 0.024, 0.040).translate((0.0, sign * 0.038, 0.0))
    body = body.union(_cyl_y(PITCH_SHOULDER_R2, sign * 0.032, sign * PITCH_SHOULDER_OUTER_Y))
    body = body.union(_cyl_y(PITCH_JOURNAL_R2, sign * PITCH_SHOULDER_OUTER_Y, sign * PITCH_JOURNAL_OUTER_Y))
    return body


def _make_study_pitch_collar(sign: float) -> cq.Workplane:
    body = _cyl_y(PITCH_COLLAR_R2, sign * PITCH_JOURNAL_OUTER_Y, sign * PITCH_COLLAR_OUTER_Y)
    body = body.union(_cyl_y(PITCH_ENDCAP_R2, sign * PITCH_COLLAR_OUTER_Y, sign * PITCH_ENDCAP_OUTER_Y))
    ear_y = sign * 0.076
    body = body.union(cq.Workplane("XY").box(0.008, 0.008, 0.010).translate((0.017, ear_y, 0.0)))
    body = body.union(cq.Workplane("XY").box(0.008, 0.008, 0.010).translate((-0.017, ear_y, 0.0)))
    body = body.cut(cq.Workplane("XY").box(0.034, 0.010, 0.002).translate((0.0, ear_y, 0.021)))
    return body


def _make_study_base_frame_full() -> cq.Workplane:
    body = _make_study_base_plate()
    body = body.union(_make_study_pedestal(-1.0))
    body = body.union(_make_study_pedestal(1.0))
    return body


def _make_study_roll_frame_full() -> cq.Workplane:
    body = _make_study_roll_yoke()
    body = body.union(_make_study_roll_journal(-1.0))
    body = body.union(_make_study_roll_journal(1.0))
    body = body.union(_make_study_roll_collar(-1.0))
    body = body.union(_make_study_roll_collar(1.0))
    return body


def _make_study_pitch_cradle_full() -> cq.Workplane:
    body = _make_study_cradle_body()
    body = body.union(_cyl_y(PITCH_JOURNAL_R2, -0.062, 0.062))
    body = body.union(_cyl_y(PITCH_SHOULDER_R2, -0.062, -0.048))
    body = body.union(_cyl_y(PITCH_SHOULDER_R2, 0.048, 0.062))
    body = body.union(cq.Workplane("XY").box(0.034, 0.020, 0.040).translate((0.0, -0.038, 0.0)))
    body = body.union(cq.Workplane("XY").box(0.034, 0.020, 0.040).translate((0.0, 0.038, 0.0)))
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pitch_roll_module", assets=ASSETS)

    dark_steel = model.material("dark_steel", rgba=(0.30, 0.32, 0.35, 1.0))
    tool_steel = model.material("tool_steel", rgba=(0.44, 0.47, 0.51, 1.0))
    zinc = model.material("zinc", rgba=(0.74, 0.76, 0.79, 1.0))
    nitride = model.material("nitride", rgba=(0.18, 0.19, 0.20, 1.0))

    base_frame = model.part("base_frame")
    _add_mesh_visual(base_frame, _make_study_base_plate(), "study_base_plate.obj", material=dark_steel, name="base_plate")
    _add_mesh_visual(base_frame, _make_study_pedestal(-1.0), "study_left_pedestal.obj", material=dark_steel, name="left_pedestal")
    _add_mesh_visual(base_frame, _make_study_pedestal(1.0), "study_right_pedestal.obj", material=dark_steel, name="right_pedestal")
    base_frame.inertial = Inertial.from_geometry(
        Box((0.29, 0.19, 0.22)),
        mass=19.0,
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
    )

    left_roll_cap = model.part("left_roll_cap")
    _add_mesh_visual(left_roll_cap, _make_study_roll_cap(), "study_left_roll_cap.obj", material=zinc, name="left_roll_cap_shell")
    left_roll_cap.inertial = Inertial.from_geometry(Box((ROLL_CAP_T2, 0.082, 0.078)), mass=0.40)

    right_roll_cap = model.part("right_roll_cap")
    _add_mesh_visual(right_roll_cap, _make_study_roll_cap(), "study_right_roll_cap.obj", material=zinc, name="right_roll_cap_shell")
    right_roll_cap.inertial = Inertial.from_geometry(Box((ROLL_CAP_T2, 0.082, 0.078)), mass=0.40)

    roll_frame = model.part("roll_frame")
    _add_mesh_visual(roll_frame, _make_study_roll_yoke(), "study_roll_yoke.obj", material=tool_steel, name="roll_yoke")
    _add_mesh_visual(roll_frame, _make_study_roll_journal(-1.0), "study_left_roll_journal.obj", material=nitride, name="left_roll_journal")
    _add_mesh_visual(roll_frame, _make_study_roll_journal(1.0), "study_right_roll_journal.obj", material=nitride, name="right_roll_journal")
    _add_mesh_visual(roll_frame, _make_study_roll_collar(-1.0), "study_left_roll_collar.obj", material=tool_steel, name="left_roll_collar")
    _add_mesh_visual(roll_frame, _make_study_roll_collar(1.0), "study_right_roll_collar.obj", material=tool_steel, name="right_roll_collar")
    roll_frame.inertial = Inertial.from_geometry(Box((0.29, 0.13, 0.15)), mass=7.8)

    front_pitch_cover = model.part("front_pitch_cover")
    _add_mesh_visual(front_pitch_cover, _make_study_pitch_cover(), "study_front_pitch_cover.obj", material=zinc, name="front_pitch_cover_shell")
    front_pitch_cover.inertial = Inertial.from_geometry(Box((0.086, PITCH_COVER_T2, 0.070)), mass=0.28)

    rear_pitch_cover = model.part("rear_pitch_cover")
    _add_mesh_visual(rear_pitch_cover, _make_study_pitch_cover(), "study_rear_pitch_cover.obj", material=zinc, name="rear_pitch_cover_shell")
    rear_pitch_cover.inertial = Inertial.from_geometry(Box((0.086, PITCH_COVER_T2, 0.070)), mass=0.28)

    pitch_cradle = model.part("pitch_cradle")
    _add_mesh_visual(pitch_cradle, _make_study_cradle_body(), "study_cradle_body.obj", material=dark_steel, name="cradle_body")
    _add_mesh_visual(pitch_cradle, _make_study_pitch_journal(-1.0), "study_front_pitch_journal.obj", material=nitride, name="front_pitch_journal")
    _add_mesh_visual(pitch_cradle, _make_study_pitch_journal(1.0), "study_rear_pitch_journal.obj", material=nitride, name="rear_pitch_journal")
    _add_mesh_visual(pitch_cradle, _make_study_pitch_collar(-1.0), "study_front_pitch_collar.obj", material=tool_steel, name="front_pitch_collar")
    _add_mesh_visual(pitch_cradle, _make_study_pitch_collar(1.0), "study_rear_pitch_collar.obj", material=tool_steel, name="rear_pitch_collar")
    pitch_cradle.inertial = Inertial.from_geometry(Box((0.09, 0.18, 0.09)), mass=4.5)

    model.articulation(
        "base_to_left_roll_cap",
        ArticulationType.FIXED,
        parent=base_frame,
        child=left_roll_cap,
        origin=Origin(xyz=(-ROLL_CAP_CENTER_X, 0.0, ROLL_AXIS_Z), rpy=(0.0, math.pi, 0.0)),
    )
    model.articulation(
        "base_to_right_roll_cap",
        ArticulationType.FIXED,
        parent=base_frame,
        child=right_roll_cap,
        origin=Origin(xyz=(ROLL_CAP_CENTER_X, 0.0, ROLL_AXIS_Z)),
    )
    model.articulation(
        "base_to_roll",
        ArticulationType.REVOLUTE,
        parent=base_frame,
        child=roll_frame,
        origin=Origin(xyz=(0.0, 0.0, ROLL_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=28.0, velocity=1.4, lower=-ROLL_LIMIT, upper=ROLL_LIMIT),
    )
    model.articulation(
        "roll_to_front_pitch_cover",
        ArticulationType.FIXED,
        parent=roll_frame,
        child=front_pitch_cover,
        origin=Origin(xyz=(0.0, -PITCH_COVER_CENTER_Y, 0.0), rpy=(0.0, 0.0, math.pi)),
    )
    model.articulation(
        "roll_to_rear_pitch_cover",
        ArticulationType.FIXED,
        parent=roll_frame,
        child=rear_pitch_cover,
        origin=Origin(xyz=(0.0, PITCH_COVER_CENTER_Y, 0.0)),
    )
    model.articulation(
        "roll_to_pitch",
        ArticulationType.REVOLUTE,
        parent=roll_frame,
        child=pitch_cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=22.0, velocity=1.7, lower=-PITCH_LIMIT, upper=PITCH_LIMIT),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base_frame = object_model.get_part("base_frame")
    left_roll_cap = object_model.get_part("left_roll_cap")
    right_roll_cap = object_model.get_part("right_roll_cap")
    roll_frame = object_model.get_part("roll_frame")
    front_pitch_cover = object_model.get_part("front_pitch_cover")
    rear_pitch_cover = object_model.get_part("rear_pitch_cover")
    pitch_cradle = object_model.get_part("pitch_cradle")
    base_plate = base_frame.get_visual("base_plate")
    left_pedestal = base_frame.get_visual("left_pedestal")
    right_pedestal = base_frame.get_visual("right_pedestal")
    left_roll_cap_shell = left_roll_cap.get_visual("left_roll_cap_shell")
    right_roll_cap_shell = right_roll_cap.get_visual("right_roll_cap_shell")
    roll_yoke = roll_frame.get_visual("roll_yoke")
    left_roll_journal = roll_frame.get_visual("left_roll_journal")
    right_roll_journal = roll_frame.get_visual("right_roll_journal")
    front_pitch_cover_shell = front_pitch_cover.get_visual("front_pitch_cover_shell")
    rear_pitch_cover_shell = rear_pitch_cover.get_visual("rear_pitch_cover_shell")
    cradle_body = pitch_cradle.get_visual("cradle_body")
    front_pitch_journal = pitch_cradle.get_visual("front_pitch_journal")
    rear_pitch_journal = pitch_cradle.get_visual("rear_pitch_journal")

    roll_joint = object_model.get_articulation("base_to_roll")
    pitch_joint = object_model.get_articulation("roll_to_pitch")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.allow_overlap(
        front_pitch_cover,
        pitch_cradle,
        reason="front bearing cap intentionally encloses the pitch trunnion shoulder inside its relieved seat",
        elem_a=front_pitch_cover_shell,
        elem_b=front_pitch_journal,
    )
    ctx.allow_overlap(
        pitch_cradle,
        rear_pitch_cover,
        reason="rear bearing cap intentionally captures the rear clamp collar within the retained bearing stack",
        elem_a=pitch_cradle.get_visual("rear_pitch_collar"),
        elem_b=rear_pitch_cover_shell,
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check("part_count", len(object_model.parts) == 7, f"expected 7 parts, found {len(object_model.parts)}")
    ctx.check("joint_count", len(object_model.articulations) == 6, f"expected 6 articulations, found {len(object_model.articulations)}")
    ctx.check("roll_axis_is_x", tuple(float(v) for v in roll_joint.axis) == (1.0, 0.0, 0.0), f"roll axis={roll_joint.axis}")
    ctx.check("pitch_axis_is_y", tuple(float(v) for v in pitch_joint.axis) == (0.0, 1.0, 0.0), f"pitch axis={pitch_joint.axis}")

    ctx.expect_contact(base_frame, left_roll_cap, elem_a=left_pedestal, elem_b=left_roll_cap_shell, name="left_roll_cap_seated_on_left_pedestal")
    ctx.expect_contact(base_frame, right_roll_cap, elem_a=right_pedestal, elem_b=right_roll_cap_shell, name="right_roll_cap_seated_on_right_pedestal")
    ctx.expect_contact(roll_frame, front_pitch_cover, elem_a=roll_yoke, elem_b=front_pitch_cover_shell, name="front_pitch_cover_seated_on_roll_frame")
    ctx.expect_contact(roll_frame, rear_pitch_cover, elem_a=roll_yoke, elem_b=rear_pitch_cover_shell, name="rear_pitch_cover_seated_on_roll_frame")

    ctx.expect_origin_distance(roll_frame, base_frame, axes="y", max_dist=1e-6, name="roll_joint_laterally_centered_in_base")
    ctx.expect_origin_gap(roll_frame, base_frame, axis="z", min_gap=ROLL_AXIS_Z, max_gap=ROLL_AXIS_Z, name="roll_joint_height_set_above_base")
    ctx.expect_origin_distance(pitch_cradle, roll_frame, axes="xz", max_dist=1e-6, name="pitch_joint_origin_centered_in_roll_frame")
    ctx.expect_origin_gap(pitch_cradle, roll_frame, axis="y", min_gap=0.0, max_gap=0.0, name="pitch_joint_origin_depth_locked")
    ctx.expect_origin_gap(right_roll_cap, left_roll_cap, axis="x", min_gap=0.245, max_gap=0.247, name="roll_cap_spacing")
    ctx.expect_origin_gap(rear_pitch_cover, front_pitch_cover, axis="y", min_gap=0.133, max_gap=0.135, name="pitch_cover_spacing")

    ctx.expect_gap(roll_frame, base_frame, axis="z", min_gap=0.086, positive_elem=roll_yoke, negative_elem=base_plate, name="roll_yoke_clear_of_base_plate")
    ctx.expect_overlap(roll_frame, base_frame, axes="yz", min_overlap=0.036, elem_a=left_roll_journal, elem_b=left_pedestal, name="left_roll_journal_coaxial_with_left_pedestal")
    ctx.expect_overlap(roll_frame, base_frame, axes="yz", min_overlap=0.036, elem_a=right_roll_journal, elem_b=right_pedestal, name="right_roll_journal_coaxial_with_right_pedestal")
    ctx.expect_within(roll_frame, base_frame, axes="yz", margin=0.026, inner_elem=left_roll_journal, outer_elem=left_pedestal, name="left_roll_journal_nested_in_left_pedestal_bore")
    ctx.expect_within(roll_frame, base_frame, axes="yz", margin=0.026, inner_elem=right_roll_journal, outer_elem=right_pedestal, name="right_roll_journal_nested_in_right_pedestal_bore")
    ctx.expect_gap(left_roll_cap, roll_frame, axis="x", min_gap=0.0, max_gap=0.001, positive_elem=left_roll_cap_shell, negative_elem=roll_frame.get_visual("left_roll_collar"), name="left_roll_collar_stops_against_left_cap")
    ctx.expect_gap(roll_frame, right_roll_cap, axis="x", min_gap=0.0, max_gap=0.001, positive_elem=roll_frame.get_visual("right_roll_collar"), negative_elem=right_roll_cap_shell, name="right_roll_collar_stops_against_right_cap")

    ctx.expect_overlap(pitch_cradle, front_pitch_cover, axes="xz", min_overlap=0.024, elem_a=front_pitch_journal, elem_b=front_pitch_cover_shell, name="front_pitch_journal_coaxial_with_front_cover")
    ctx.expect_overlap(pitch_cradle, rear_pitch_cover, axes="xz", min_overlap=0.024, elem_a=rear_pitch_journal, elem_b=rear_pitch_cover_shell, name="rear_pitch_journal_coaxial_with_rear_cover")
    ctx.expect_within(pitch_cradle, roll_frame, axes="xz", margin=0.020, inner_elem=front_pitch_journal, outer_elem=roll_yoke, name="front_pitch_journal_nested_in_roll_cheek_bore")
    ctx.expect_within(pitch_cradle, roll_frame, axes="xz", margin=0.020, inner_elem=rear_pitch_journal, outer_elem=roll_yoke, name="rear_pitch_journal_nested_in_roll_cheek_bore")
    ctx.expect_gap(front_pitch_cover, pitch_cradle, axis="y", max_gap=0.001, max_penetration=1e-4, positive_elem=front_pitch_cover_shell, negative_elem=pitch_cradle.get_visual("front_pitch_collar"), name="front_pitch_collar_stops_at_front_cover")
    ctx.expect_gap(pitch_cradle, rear_pitch_cover, axis="y", max_gap=0.001, max_penetration=0.0085, positive_elem=pitch_cradle.get_visual("rear_pitch_collar"), negative_elem=rear_pitch_cover_shell, name="rear_pitch_collar_stops_at_rear_cover")
    ctx.expect_within(pitch_cradle, roll_frame, axes="xz", margin=0.010, inner_elem=cradle_body, outer_elem=roll_yoke, name="cradle_body_nested_within_roll_window")
    ctx.expect_overlap(pitch_cradle, roll_frame, axes="xz", min_overlap=0.052, elem_a=cradle_body, elem_b=roll_yoke, name="cradle_body_reads_nested_inside_roll_frame")

    with ctx.pose({roll_joint: 0.54}):
        ctx.expect_gap(roll_frame, base_frame, axis="z", min_gap=0.050, positive_elem=roll_yoke, negative_elem=base_plate, name="roll_yoke_clears_base_plate_at_positive_roll")
        ctx.expect_within(pitch_cradle, roll_frame, axes="xz", margin=0.012, inner_elem=cradle_body, outer_elem=roll_yoke, name="cradle_body_within_roll_window_at_positive_roll")

    with ctx.pose({roll_joint: -0.54}):
        ctx.expect_gap(roll_frame, base_frame, axis="z", min_gap=0.050, positive_elem=roll_yoke, negative_elem=base_plate, name="roll_yoke_clears_base_plate_at_negative_roll")
        ctx.expect_within(pitch_cradle, roll_frame, axes="xz", margin=0.012, inner_elem=cradle_body, outer_elem=roll_yoke, name="cradle_body_within_roll_window_at_negative_roll")

    with ctx.pose({pitch_joint: 0.60}):
        ctx.expect_within(pitch_cradle, roll_frame, axes="xz", margin=0.014, inner_elem=cradle_body, outer_elem=roll_yoke, name="cradle_body_within_roll_window_at_positive_pitch")
        ctx.expect_overlap(pitch_cradle, front_pitch_cover, axes="xz", min_overlap=0.020, elem_a=front_pitch_journal, elem_b=front_pitch_cover_shell, name="front_pitch_journal_tracks_front_cover_at_positive_pitch")
        ctx.expect_overlap(pitch_cradle, rear_pitch_cover, axes="xz", min_overlap=0.020, elem_a=rear_pitch_journal, elem_b=rear_pitch_cover_shell, name="rear_pitch_journal_tracks_rear_cover_at_positive_pitch")

    with ctx.pose({roll_joint: -0.46, pitch_joint: -0.58}):
        ctx.expect_gap(roll_frame, base_frame, axis="z", min_gap=0.050, positive_elem=roll_yoke, negative_elem=base_plate, name="roll_yoke_clears_base_in_combined_pose")
        ctx.expect_within(pitch_cradle, roll_frame, axes="xz", margin=0.016, inner_elem=cradle_body, outer_elem=roll_yoke, name="cradle_body_within_roll_window_in_combined_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
