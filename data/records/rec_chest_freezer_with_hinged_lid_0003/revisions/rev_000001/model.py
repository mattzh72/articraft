from __future__ import annotations

import os as _bootstrap_os

_REAL_BOOTSTRAP_GETCWD = _bootstrap_os.getcwd


def _bootstrap_safe_getcwd() -> str:
    try:
        return _REAL_BOOTSTRAP_GETCWD()
    except FileNotFoundError:
        return "/"


_bootstrap_os.getcwd = _bootstrap_safe_getcwd
try:
    _bootstrap_os.chdir("/")
except FileNotFoundError:
    pass
if not _bootstrap_os.path.isabs(__file__):
    __file__ = "/" + __file__.lstrip("./")

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

OUTER_W = 1.56
OUTER_D = 0.82
BODY_H = 0.88
PLINTH_H = 0.08

SIDE_SHELL_T = 0.035
FRONT_SHELL_T = 0.040
REAR_SHELL_T = 0.035

CAVITY_W = 1.34
CAVITY_D = 0.66
LINER_T = 0.006
FLOOR_T = 0.025
FLOOR_TOP_Z = 0.15
LINER_TOP_Z = 0.82
RIM_T = 0.030
RIM_H = 0.028

OPEN_ANGLE = math.radians(67.0)
CLOSED_ANGLE = -OPEN_ANGLE

HINGE_RADIUS = 0.011
HINGE_Y = -OUTER_D / 2.0 - 0.014
HINGE_Z = BODY_H + 0.016

LID_W = OUTER_W + 0.030
LID_D = OUTER_D + 0.025
LID_T = 0.110
LID_AXIS_FROM_BOTTOM = 0.014
LID_TOP_CENTER = (0.0, LID_D / 2.0 - 0.020, LID_T / 2.0 - LID_AXIS_FROM_BOTTOM)
LID_INNER_PANEL_CENTER = (0.0, OUTER_D / 2.0, -0.002)
LID_FOAM_CORE_CENTER = (0.0, OUTER_D / 2.0, 0.021)
LID_GASKET_Z = -0.012

BODY_PIN = (CAVITY_W / 2.0 - 0.050, -CAVITY_D / 2.0 + 0.110, 0.545)
LID_PIN_CLOSED_LOCAL = (BODY_PIN[0], 0.155, -0.034)

PIN_RADIUS = 0.008
EYE_RADIUS = 0.012
PIN_LENGTH = 0.028
EYE_LENGTH = 0.020
STRUT_HOUSING_RADIUS = 0.016
STRUT_ROD_RADIUS = 0.006
STRUT_HOUSING_START = 0.055


def _rot_x(point: tuple[float, float, float], angle: float) -> tuple[float, float, float]:
    x, y, z = point
    c = math.cos(angle)
    s = math.sin(angle)
    return (x, y * c - z * s, y * s + z * c)


def _add(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _sub(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _segment_angle_and_length(vector: tuple[float, float, float]) -> tuple[float, float]:
    _, dy, dz = vector
    return math.atan2(-dy, dz), math.hypot(dy, dz)


def _strut_kinematics() -> dict[str, float | tuple[float, float, float]]:
    hinge_origin = (0.0, HINGE_Y, HINGE_Z)
    open_pin_world = _add(hinge_origin, _rot_x(LID_PIN_CLOSED_LOCAL, OPEN_ANGLE))
    closed_pin_world = _add(hinge_origin, LID_PIN_CLOSED_LOCAL)

    open_span = _sub(open_pin_world, BODY_PIN)
    closed_span = _sub(closed_pin_world, BODY_PIN)

    open_angle, open_length = _segment_angle_and_length(open_span)
    closed_angle, closed_length = _segment_angle_and_length(closed_span)

    housing_len = min(0.205, open_length - 0.140)
    return {
        "hinge_origin": hinge_origin,
        "open_pin_world": open_pin_world,
        "closed_pin_world": closed_pin_world,
        "open_strut_angle": open_angle,
        "closed_delta": closed_angle - open_angle,
        "open_length": open_length,
        "closed_length": closed_length,
        "housing_len": housing_len,
        "rod_open_len": open_length - housing_len,
        "slide_closed": closed_length - open_length,
    }


KIN = _strut_kinematics()
STRUT_OPEN_ANGLE = float(KIN["open_strut_angle"])
STRUT_CLOSED_DELTA = float(KIN["closed_delta"])
STRUT_OPEN_LEN = float(KIN["open_length"])
STRUT_CLOSED_LEN = float(KIN["closed_length"])
STRUT_HOUSING_LEN = float(KIN["housing_len"])
STRUT_ROD_OPEN_LEN = float(KIN["rod_open_len"])
STRUT_SLIDE_CLOSED = float(KIN["slide_closed"])
STRUT_UPPER_EYE_Z = STRUT_OPEN_LEN - (STRUT_HOUSING_START + STRUT_HOUSING_LEN)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_chest_freezer")

    steel = model.material("stainless_steel", rgba=(0.73, 0.75, 0.78, 1.0))
    liner_white = model.material("liner_white", rgba=(0.95, 0.96, 0.97, 1.0))
    trim_black = model.material("trim_black", rgba=(0.12, 0.12, 0.13, 1.0))
    plinth_black = model.material("plinth_black", rgba=(0.16, 0.17, 0.18, 1.0))
    vent_black = model.material("vent_black", rgba=(0.14, 0.15, 0.16, 1.0))
    insulation_gray = model.material("insulation_gray", rgba=(0.86, 0.87, 0.88, 1.0))
    strut_body = model.material("strut_body", rgba=(0.29, 0.30, 0.32, 1.0))
    strut_rod = model.material("strut_rod", rgba=(0.82, 0.83, 0.85, 1.0))

    body = model.part("body")
    shell_h = BODY_H - PLINTH_H
    shell_z = PLINTH_H + shell_h / 2.0
    liner_h = LINER_TOP_Z - FLOOR_TOP_Z
    liner_z = FLOOR_TOP_Z + liner_h / 2.0
    rim_z = BODY_H - RIM_H / 2.0

    side_insulation_t = OUTER_W / 2.0 - CAVITY_W / 2.0 - SIDE_SHELL_T - LINER_T
    front_insulation_t = OUTER_D / 2.0 - CAVITY_D / 2.0 - FRONT_SHELL_T - LINER_T
    rear_insulation_t = OUTER_D / 2.0 - CAVITY_D / 2.0 - REAR_SHELL_T - LINER_T

    body.visual(
        Box((OUTER_W, OUTER_D, PLINTH_H)),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_H / 2.0)),
        material=plinth_black,
        name="plinth",
    )
    body.visual(
        Box((OUTER_W - 0.080, OUTER_D - 0.080, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_H + 0.010)),
        material=plinth_black,
        name="base_deck",
    )
    body.visual(
        Box((SIDE_SHELL_T, OUTER_D, shell_h)),
        origin=Origin(xyz=((OUTER_W - SIDE_SHELL_T) / 2.0, 0.0, shell_z)),
        material=steel,
        name="right_shell",
    )
    body.visual(
        Box((SIDE_SHELL_T, OUTER_D, shell_h)),
        origin=Origin(xyz=(-(OUTER_W - SIDE_SHELL_T) / 2.0, 0.0, shell_z)),
        material=steel,
        name="left_shell",
    )
    body.visual(
        Box((OUTER_W - 2.0 * SIDE_SHELL_T, FRONT_SHELL_T, shell_h)),
        origin=Origin(xyz=(0.0, (OUTER_D - FRONT_SHELL_T) / 2.0, shell_z)),
        material=steel,
        name="front_shell",
    )
    body.visual(
        Box((OUTER_W - 2.0 * SIDE_SHELL_T, REAR_SHELL_T, shell_h)),
        origin=Origin(xyz=(0.0, -(OUTER_D - REAR_SHELL_T) / 2.0, shell_z)),
        material=steel,
        name="rear_shell",
    )
    body.visual(
        Box((OUTER_W - 0.180, 0.012, 0.160)),
        origin=Origin(xyz=(0.0, OUTER_D / 2.0 - 0.006, 0.205)),
        material=vent_black,
        name="front_vent",
    )

    body.visual(
        Box((CAVITY_W, CAVITY_D, FLOOR_T)),
        origin=Origin(xyz=(0.0, 0.0, FLOOR_TOP_Z - FLOOR_T / 2.0)),
        material=liner_white,
        name="liner_floor",
    )
    body.visual(
        Box((LINER_T, CAVITY_D + 2.0 * LINER_T, liner_h)),
        origin=Origin(xyz=(CAVITY_W / 2.0 + LINER_T / 2.0, 0.0, liner_z)),
        material=liner_white,
        name="liner_right",
    )
    body.visual(
        Box((LINER_T, CAVITY_D + 2.0 * LINER_T, liner_h)),
        origin=Origin(xyz=(-(CAVITY_W / 2.0 + LINER_T / 2.0), 0.0, liner_z)),
        material=liner_white,
        name="liner_left",
    )
    body.visual(
        Box((CAVITY_W, LINER_T, liner_h)),
        origin=Origin(xyz=(0.0, CAVITY_D / 2.0 + LINER_T / 2.0, liner_z)),
        material=liner_white,
        name="liner_front",
    )
    body.visual(
        Box((CAVITY_W, LINER_T, liner_h)),
        origin=Origin(xyz=(0.0, -(CAVITY_D / 2.0 + LINER_T / 2.0), liner_z)),
        material=liner_white,
        name="liner_rear",
    )

    body.visual(
        Box((side_insulation_t, CAVITY_D + 2.0 * LINER_T, liner_h)),
        origin=Origin(
            xyz=(
                CAVITY_W / 2.0 + LINER_T + side_insulation_t / 2.0,
                0.0,
                liner_z,
            )
        ),
        material=insulation_gray,
        name="right_insulation",
    )
    body.visual(
        Box((side_insulation_t, CAVITY_D + 2.0 * LINER_T, liner_h)),
        origin=Origin(
            xyz=(
                -(CAVITY_W / 2.0 + LINER_T + side_insulation_t / 2.0),
                0.0,
                liner_z,
            )
        ),
        material=insulation_gray,
        name="left_insulation",
    )
    body.visual(
        Box((CAVITY_W, front_insulation_t, liner_h)),
        origin=Origin(
            xyz=(
                0.0,
                CAVITY_D / 2.0 + LINER_T + front_insulation_t / 2.0,
                liner_z,
            )
        ),
        material=insulation_gray,
        name="front_insulation",
    )
    body.visual(
        Box((CAVITY_W, rear_insulation_t, liner_h)),
        origin=Origin(
            xyz=(
                0.0,
                -(CAVITY_D / 2.0 + LINER_T + rear_insulation_t / 2.0),
                liner_z,
            )
        ),
        material=insulation_gray,
        name="rear_insulation",
    )

    body.visual(
        Box((OUTER_W - 2.0 * SIDE_SHELL_T, FRONT_SHELL_T, RIM_H)),
        origin=Origin(xyz=(0.0, OUTER_D / 2.0 - FRONT_SHELL_T / 2.0, rim_z)),
        material=trim_black,
        name="top_front_trim",
    )
    body.visual(
        Box((OUTER_W - 2.0 * SIDE_SHELL_T, REAR_SHELL_T, RIM_H)),
        origin=Origin(xyz=(0.0, -(OUTER_D / 2.0 - REAR_SHELL_T / 2.0), rim_z)),
        material=trim_black,
        name="top_rear_trim",
    )
    body.visual(
        Box((RIM_T, OUTER_D - 0.010, RIM_H)),
        origin=Origin(xyz=(OUTER_W / 2.0 - SIDE_SHELL_T / 2.0, 0.0, rim_z)),
        material=trim_black,
        name="top_right_trim",
    )
    body.visual(
        Box((RIM_T, OUTER_D - 0.010, RIM_H)),
        origin=Origin(xyz=(-(OUTER_W / 2.0 - SIDE_SHELL_T / 2.0), 0.0, rim_z)),
        material=trim_black,
        name="top_left_trim",
    )

    body.visual(
        Box((0.240, 0.280, 0.150)),
        origin=Origin(
            xyz=(
                CAVITY_W / 2.0 - 0.120,
                -(CAVITY_D / 2.0) + 0.140,
                FLOOR_TOP_Z + 0.075,
            )
        ),
        material=liner_white,
        name="compressor_hump",
    )

    body.visual(
        Box((0.065, 0.090, 0.092)),
        origin=Origin(
            xyz=(
                BODY_PIN[0] + 0.0325,
                BODY_PIN[1],
                0.492,
            )
        ),
        material=steel,
        name="strut_base_bracket",
    )
    body.visual(
        Cylinder(radius=PIN_RADIUS, length=PIN_LENGTH),
        origin=Origin(xyz=BODY_PIN, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="strut_lower_pin",
    )

    body.visual(
        Box((OUTER_W + 0.020, 0.030, 0.016)),
        origin=Origin(xyz=(0.0, -OUTER_D / 2.0 - 0.004, BODY_H - 0.002)),
        material=steel,
        name="body_hinge_leaf",
    )
    body.visual(
        Cylinder(radius=HINGE_RADIUS, length=OUTER_W + 0.030),
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hinge_barrel",
    )

    lid = model.part("lid")
    lid.visual(
        Box((LID_W, LID_D, 0.018)),
        origin=Origin(xyz=LID_TOP_CENTER),
        material=steel,
        name="lid_top_skin",
    )
    lid.visual(
        Box((LID_W, 0.026, 0.088)),
        origin=Origin(
            xyz=(
                0.0,
                LID_D - 0.020 - 0.013,
                0.088 / 2.0 - LID_AXIS_FROM_BOTTOM,
            )
        ),
        material=steel,
        name="lid_front_skirt",
    )
    lid.visual(
        Box((0.026, LID_D - 0.110, 0.088)),
        origin=Origin(
            xyz=(
                (LID_W - 0.026) / 2.0,
                LID_TOP_CENTER[1] + 0.055,
                0.088 / 2.0 - LID_AXIS_FROM_BOTTOM,
            )
        ),
        material=steel,
        name="lid_right_skirt",
    )
    lid.visual(
        Box((0.026, LID_D - 0.110, 0.088)),
        origin=Origin(
            xyz=(
                -(LID_W - 0.026) / 2.0,
                LID_TOP_CENTER[1] + 0.055,
                0.088 / 2.0 - LID_AXIS_FROM_BOTTOM,
            )
        ),
        material=steel,
        name="lid_left_skirt",
    )
    lid.visual(
        Box((LID_W, 0.022, 0.030)),
        origin=Origin(xyz=(0.0, 0.110, 0.017)),
        material=steel,
        name="lid_rear_rail",
    )
    lid.visual(
        Box((CAVITY_W + 0.100, CAVITY_D + 0.080, 0.012)),
        origin=Origin(xyz=LID_INNER_PANEL_CENTER),
        material=liner_white,
        name="lid_inner_panel",
    )
    lid.visual(
        Box((CAVITY_W + 0.060, CAVITY_D + 0.040, 0.050)),
        origin=Origin(xyz=LID_FOAM_CORE_CENTER),
        material=insulation_gray,
        name="lid_foam_core",
    )
    lid.visual(
        Box((CAVITY_W + 0.030, 0.020, 0.008)),
        origin=Origin(
            xyz=(
                0.0,
                CAVITY_D / 2.0 + 0.030,
                LID_GASKET_Z,
            )
        ),
        material=trim_black,
        name="gasket_front",
    )
    lid.visual(
        Box((CAVITY_W + 0.030, 0.020, 0.008)),
        origin=Origin(
            xyz=(
                0.0,
                0.050,
                LID_GASKET_Z,
            )
        ),
        material=trim_black,
        name="gasket_rear",
    )
    lid.visual(
        Box((0.020, CAVITY_D + 0.060, 0.008)),
        origin=Origin(
            xyz=(
                CAVITY_W / 2.0 + 0.025,
                OUTER_D / 2.0,
                LID_GASKET_Z,
            )
        ),
        material=trim_black,
        name="gasket_right",
    )
    lid.visual(
        Box((0.020, CAVITY_D + 0.060, 0.008)),
        origin=Origin(
            xyz=(
                -(CAVITY_W / 2.0 + 0.025),
                OUTER_D / 2.0,
                LID_GASKET_Z,
            )
        ),
        material=trim_black,
        name="gasket_left",
    )
    lid.visual(
        Box((0.062, 0.100, 0.018)),
        origin=Origin(
            xyz=(
                LID_PIN_CLOSED_LOCAL[0] + 0.020,
                LID_PIN_CLOSED_LOCAL[1],
                0.004,
            )
        ),
        material=steel,
        name="strut_lid_bracket",
    )
    lid.visual(
        Box((0.012, 0.026, 0.050)),
        origin=Origin(
            xyz=(
                LID_PIN_CLOSED_LOCAL[0] - 0.019,
                LID_PIN_CLOSED_LOCAL[1],
                -0.010,
            )
        ),
        material=steel,
        name="strut_lid_tab_left",
    )
    lid.visual(
        Box((0.012, 0.026, 0.050)),
        origin=Origin(
            xyz=(
                LID_PIN_CLOSED_LOCAL[0] + 0.019,
                LID_PIN_CLOSED_LOCAL[1],
                -0.010,
            )
        ),
        material=steel,
        name="strut_lid_tab_right",
    )
    lid.visual(
        Cylinder(radius=PIN_RADIUS, length=PIN_LENGTH),
        origin=Origin(
            xyz=LID_PIN_CLOSED_LOCAL,
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel,
        name="strut_upper_pin",
    )
    lid.visual(
        Box((0.620, 0.040, 0.018)),
        origin=Origin(
            xyz=(
                0.0,
                LID_D - 0.060,
                0.030,
            )
        ),
        material=trim_black,
        name="front_pull_bar",
    )
    lid.visual(
        Box((0.040, 0.030, 0.040)),
        origin=Origin(xyz=(-0.240, LID_D - 0.070, 0.030)),
        material=steel,
        name="pull_left_post",
    )
    lid.visual(
        Box((0.040, 0.030, 0.040)),
        origin=Origin(xyz=(0.240, LID_D - 0.070, 0.030)),
        material=steel,
        name="pull_right_post",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z), rpy=(OPEN_ANGLE, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=260.0,
            velocity=1.2,
            lower=CLOSED_ANGLE,
            upper=0.10,
        ),
    )

    strut_tube_part = model.part("strut_tube")
    strut_tube_part.visual(
        Cylinder(radius=EYE_RADIUS, length=EYE_LENGTH),
        origin=Origin(
            xyz=(0.0, PIN_RADIUS + EYE_RADIUS, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=strut_body,
        name="lower_eye",
    )
    strut_tube_part.visual(
        Box((0.014, 0.024, 0.064)),
        origin=Origin(xyz=(0.0, 0.012, 0.042)),
        material=strut_body,
        name="lower_eye_bridge",
    )
    strut_tube_part.visual(
        Cylinder(radius=STRUT_HOUSING_RADIUS, length=STRUT_HOUSING_LEN),
        origin=Origin(xyz=(0.0, 0.0, STRUT_HOUSING_START + STRUT_HOUSING_LEN / 2.0)),
        material=strut_body,
        name="housing_shell",
    )
    strut_tube_part.visual(
        Cylinder(radius=STRUT_HOUSING_RADIUS + 0.002, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, STRUT_HOUSING_START + STRUT_HOUSING_LEN - 0.008)),
        material=strut_body,
        name="seal_cap",
    )
    strut_tube_part.visual(
        Cylinder(radius=0.019, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, STRUT_HOUSING_START + 0.004)),
        material=strut_body,
        name="housing_collar",
    )
    model.articulation(
        "body_to_strut_tube",
        ArticulationType.REVOLUTE,
        parent=body,
        child=strut_tube_part,
        origin=Origin(xyz=BODY_PIN, rpy=(STRUT_OPEN_ANGLE, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.5,
            lower=STRUT_CLOSED_DELTA - 0.03,
            upper=0.08,
        ),
    )

    strut_rod_part = model.part("strut_rod")
    strut_rod_part.visual(
        Cylinder(radius=STRUT_ROD_RADIUS, length=STRUT_UPPER_EYE_Z - 0.016),
        origin=Origin(xyz=(0.0, 0.0, (STRUT_UPPER_EYE_Z - 0.016) / 2.0)),
        material=strut_rod,
        name="rod_shaft",
    )
    strut_rod_part.visual(
        Box((0.016, 0.020, 0.024)),
        origin=Origin(
            xyz=(0.0, 0.010, STRUT_UPPER_EYE_Z - 0.024),
        ),
        material=strut_rod,
        name="upper_eye_bridge",
    )
    strut_rod_part.visual(
        Cylinder(radius=EYE_RADIUS, length=EYE_LENGTH),
        origin=Origin(
            xyz=(0.0, PIN_RADIUS + EYE_RADIUS, STRUT_UPPER_EYE_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=strut_rod,
        name="upper_eye",
    )
    model.articulation(
        "strut_extension",
        ArticulationType.PRISMATIC,
        parent=strut_tube_part,
        child=strut_rod_part,
        origin=Origin(xyz=(0.0, 0.0, STRUT_HOUSING_START + STRUT_HOUSING_LEN)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=50.0,
            velocity=0.40,
            lower=STRUT_SLIDE_CLOSED - 0.010,
            upper=0.030,
        ),
    )

    return model


def _aabb_z_span(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float] | None:
    if aabb is None:
        return None
    return (aabb[0][2], aabb[1][2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    strut_tube = object_model.get_part("strut_tube")
    strut_rod = object_model.get_part("strut_rod")

    lid_hinge = object_model.get_articulation("lid_hinge")
    strut_pivot = object_model.get_articulation("body_to_strut_tube")
    strut_extension = object_model.get_articulation("strut_extension")

    hinge_barrel = body.get_visual("hinge_barrel")
    top_front_trim = body.get_visual("top_front_trim")
    liner_floor = body.get_visual("liner_floor")
    lower_pin = body.get_visual("strut_lower_pin")
    lid_top_skin = lid.get_visual("lid_top_skin")
    lid_inner_panel = lid.get_visual("lid_inner_panel")
    lid_rear_rail = lid.get_visual("lid_rear_rail")
    gasket_front = lid.get_visual("gasket_front")
    upper_pin = lid.get_visual("strut_upper_pin")
    housing_shell = strut_tube.get_visual("housing_shell")
    lower_eye = strut_tube.get_visual("lower_eye")
    seal_cap = strut_tube.get_visual("seal_cap")
    rod_shaft = strut_rod.get_visual("rod_shaft")
    upper_eye = strut_rod.get_visual("upper_eye")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.allow_overlap(
        strut_tube,
        strut_rod,
        reason="The gas strut rod telescopes inside a hollow cylinder in reality; the housing is approximated with solid visuals.",
    )
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem=lid.get_visual("lid_front_skirt"),
        negative_elem=top_front_trim,
        min_gap=0.65,
        name="lid_is_held_open_above_the_cabinet",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="x",
        elem_a=lid_rear_rail,
        elem_b=hinge_barrel,
        min_overlap=1.52,
        name="piano_hinge_runs_full_rear_width",
    )
    ctx.expect_contact(
        strut_tube,
        body,
        contact_tol=5e-06,
        elem_a=lower_eye,
        elem_b=lower_pin,
        name="gas_strut_lower_eye_mounts_to_body_pin",
    )
    ctx.expect_contact(
        strut_rod,
        lid,
        contact_tol=5e-06,
        elem_a=upper_eye,
        elem_b=upper_pin,
        name="gas_strut_upper_eye_mounts_to_lid_pin",
    )
    ctx.expect_contact(
        strut_tube,
        strut_rod,
        elem_a=seal_cap,
        elem_b=rod_shaft,
        name="gas_strut_rod_emerges_from_housing",
    )
    ctx.expect_gap(
        body,
        body,
        axis="z",
        positive_elem=top_front_trim,
        negative_elem=liner_floor,
        min_gap=0.64,
        name="freezer_has_a_deep_storage_cavity",
    )

    open_lid_aabb = ctx.part_element_world_aabb(lid, elem=lid_top_skin)
    with ctx.pose({lid_hinge: CLOSED_ANGLE, strut_pivot: STRUT_CLOSED_DELTA, strut_extension: STRUT_SLIDE_CLOSED}):
        closed_lid_aabb = ctx.part_element_world_aabb(lid, elem=lid_top_skin)
        ctx.fail_if_isolated_parts(name="closed_pose_no_floating")
        ctx.fail_if_parts_overlap_in_current_pose(name="closed_pose_no_unintended_overlap")
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem=gasket_front,
            negative_elem=top_front_trim,
            max_gap=0.004,
            max_penetration=0.0,
            name="closed_lid_front_gasket_seats_on_trim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="x",
            elem_a=lid_top_skin,
            min_overlap=1.54,
            name="closed_lid_covers_cabinet_width",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="y",
            elem_a=lid_top_skin,
            min_overlap=0.80,
            name="closed_lid_covers_cabinet_depth",
        )
        ctx.expect_contact(
            strut_rod,
            lid,
            contact_tol=5e-06,
            elem_a=upper_eye,
            elem_b=upper_pin,
            name="closed_pose_strut_stays_attached_to_lid_pin",
        )
        ctx.expect_contact(
            strut_tube,
            body,
            contact_tol=5e-06,
            elem_a=lower_eye,
            elem_b=lower_pin,
            name="closed_pose_strut_stays_attached_to_body_pin",
        )

    if open_lid_aabb is None or closed_lid_aabb is None:
        ctx.fail("lid_hinge_has_measurable_motion", "Could not resolve lid top skin AABBs.")
    else:
        ctx.check(
            "lid_hinge_has_measurable_motion",
            closed_lid_aabb[1][2] < open_lid_aabb[1][2] - 0.60,
            details=f"open_max_z={open_lid_aabb[1][2]:.3f}, closed_max_z={closed_lid_aabb[1][2]:.3f}",
        )

    open_housing_aabb = ctx.part_element_world_aabb(strut_tube, elem=housing_shell)
    with ctx.pose({strut_pivot: STRUT_CLOSED_DELTA}):
        pivoted_housing_aabb = ctx.part_element_world_aabb(strut_tube, elem=housing_shell)
    if open_housing_aabb is None or pivoted_housing_aabb is None:
        ctx.fail("strut_pivot_changes_housing_orientation", "Could not resolve housing AABBs.")
    else:
        open_center_y = (open_housing_aabb[0][1] + open_housing_aabb[1][1]) / 2.0
        pivoted_center_y = (pivoted_housing_aabb[0][1] + pivoted_housing_aabb[1][1]) / 2.0
        ctx.check(
            "strut_pivot_changes_housing_orientation",
            pivoted_center_y > open_center_y + 0.010,
            details=f"open_center_y={open_center_y:.3f}, pivoted_center_y={pivoted_center_y:.3f}",
        )

    open_rod_aabb = ctx.part_element_world_aabb(strut_rod, elem=upper_eye)
    with ctx.pose({strut_extension: STRUT_SLIDE_CLOSED}):
        retracted_rod_aabb = ctx.part_element_world_aabb(strut_rod, elem=upper_eye)
    if open_rod_aabb is None or retracted_rod_aabb is None:
        ctx.fail("strut_extension_retracts_rod_eye", "Could not resolve rod eye AABBs.")
    else:
        open_center_z = (open_rod_aabb[0][2] + open_rod_aabb[1][2]) / 2.0
        retracted_center_z = (retracted_rod_aabb[0][2] + retracted_rod_aabb[1][2]) / 2.0
        ctx.check(
            "strut_extension_retracts_rod_eye",
            retracted_center_z < open_center_z - 0.10,
            details=f"open_center_z={open_center_z:.3f}, retracted_center_z={retracted_center_z:.3f}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
