from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


AXIS_Z = 0.135

BASE_START_X = 0.05
BASE_LEN = 0.26
BASE_DEPTH = 0.18
BASE_T = 0.02

LEFT_SUPPORT_X = 0.11
RIGHT_SUPPORT_X = 0.24
SUPPORT_LEN = 0.048
SUPPORT_DEPTH = 0.12

PEDESTAL_BOTTOM_Z = BASE_T - 0.001
PEDESTAL_H = 0.072
SADDLE_PAD_W = 0.030
SADDLE_PAD_BOTTOM_Z = 0.090

CHEEK_T = 0.018
CHEEK_CENTER_Y = 0.029
CHEEK_BOTTOM_Z = 0.088
CHEEK_TOP_Z = 0.152

BRIDGE_UPRIGHT_W = 0.020
BRIDGE_UPRIGHT_D = 0.050
BRIDGE_BEAM_START_X = 0.10
BRIDGE_BEAM_LEN = 0.15
BRIDGE_BEAM_DEPTH = 0.05
BRIDGE_BEAM_BOTTOM_Z = 0.161
BRIDGE_BEAM_H = 0.032
BRIDGE_BEAM_Y = -0.045

SHAFT_R = 0.014
SHAFT_LEN = 0.43
INPUT_COLLAR_R = 0.020
INPUT_COLLAR_X = 0.058
INPUT_COLLAR_LEN = 0.018
OUTPUT_SHOULDER_R = 0.0185
OUTPUT_SHOULDER_X = 0.286
OUTPUT_SHOULDER_LEN = 0.020
OUTPUT_STUB_R = 0.0115
OUTPUT_STUB_X = 0.315
OUTPUT_STUB_LEN = SHAFT_LEN - OUTPUT_STUB_X

DRIVE_TAB_X = 0.355
DRIVE_TAB_LEN = 0.045
DRIVE_TAB_W = 0.010
DRIVE_TAB_H = 0.012
DRIVE_TAB_BASE_Z = 0.003


def _box_origin_from_min(
    x0: float,
    y_center: float,
    z0: float,
    sx: float,
    sy: float,
    sz: float,
) -> Origin:
    return Origin(xyz=(x0 + sx / 2.0, y_center, z0 + sz / 2.0))


def _x_axis_cylinder_origin(
    length: float,
    x0: float,
    y: float = 0.0,
    z: float = 0.0,
) -> Origin:
    return Origin(
        xyz=(x0 + length / 2.0, y, z),
        rpy=(0.0, pi / 2.0, 0.0),
    )


def _add_saddle_visuals(model: ArticulatedObject, frame, prefix: str, center_x: float) -> None:
    saddle_pad_top_z = AXIS_Z - SHAFT_R
    saddle_pad_h = saddle_pad_top_z - SADDLE_PAD_BOTTOM_Z
    cheek_h = CHEEK_TOP_Z - CHEEK_BOTTOM_Z

    frame.visual(
        Box((SUPPORT_LEN, SUPPORT_DEPTH, PEDESTAL_H)),
        origin=_box_origin_from_min(
            center_x - SUPPORT_LEN / 2.0,
            0.0,
            PEDESTAL_BOTTOM_Z,
            SUPPORT_LEN,
            SUPPORT_DEPTH,
            PEDESTAL_H,
        ),
        material="frame_paint",
        name=f"{prefix}_pedestal",
    )
    frame.visual(
        Box((SUPPORT_LEN, SADDLE_PAD_W, saddle_pad_h)),
        origin=_box_origin_from_min(
            center_x - SUPPORT_LEN / 2.0,
            0.0,
            SADDLE_PAD_BOTTOM_Z,
            SUPPORT_LEN,
            SADDLE_PAD_W,
            saddle_pad_h,
        ),
        material="frame_paint",
        name=f"{prefix}_saddle_pad",
    )
    frame.visual(
        Box((SUPPORT_LEN, CHEEK_T, cheek_h)),
        origin=_box_origin_from_min(
            center_x - SUPPORT_LEN / 2.0,
            -CHEEK_CENTER_Y,
            CHEEK_BOTTOM_Z,
            SUPPORT_LEN,
            CHEEK_T,
            cheek_h,
        ),
        material="frame_paint",
        name=f"{prefix}_saddle_rear_cheek",
    )
    frame.visual(
        Box((SUPPORT_LEN, CHEEK_T, cheek_h)),
        origin=_box_origin_from_min(
            center_x - SUPPORT_LEN / 2.0,
            CHEEK_CENTER_Y,
            CHEEK_BOTTOM_Z,
            SUPPORT_LEN,
            CHEEK_T,
            cheek_h,
        ),
        material="frame_paint",
        name=f"{prefix}_saddle_front_cheek",
    )


def _add_bridge_upright(frame, prefix: str, center_x: float) -> None:
    upright_h = BRIDGE_BEAM_BOTTOM_Z + BRIDGE_BEAM_H - PEDESTAL_BOTTOM_Z
    frame.visual(
        Box((BRIDGE_UPRIGHT_W, BRIDGE_UPRIGHT_D, upright_h)),
        origin=_box_origin_from_min(
            center_x - BRIDGE_UPRIGHT_W / 2.0,
            BRIDGE_BEAM_Y,
            PEDESTAL_BOTTOM_Z,
            BRIDGE_UPRIGHT_W,
            BRIDGE_UPRIGHT_D,
            upright_h,
        ),
        material="frame_paint",
        name=f"{prefix}_bridge_upright",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_supported_drive_shaft")

    model.material("frame_paint", rgba=(0.24, 0.27, 0.30, 1.0))
    model.material("shaft_steel", rgba=(0.72, 0.75, 0.78, 1.0))
    model.material("oxide_steel", rgba=(0.32, 0.34, 0.36, 1.0))

    frame = model.part("bridge_frame")
    frame.visual(
        Box((BASE_LEN, BASE_DEPTH, BASE_T)),
        origin=_box_origin_from_min(
            BASE_START_X,
            0.0,
            0.0,
            BASE_LEN,
            BASE_DEPTH,
            BASE_T,
        ),
        material="frame_paint",
        name="bridge_frame_body",
    )
    frame.visual(
        Box((BRIDGE_BEAM_LEN, BRIDGE_BEAM_DEPTH, BRIDGE_BEAM_H)),
        origin=_box_origin_from_min(
            BRIDGE_BEAM_START_X,
            BRIDGE_BEAM_Y,
            BRIDGE_BEAM_BOTTOM_Z,
            BRIDGE_BEAM_LEN,
            BRIDGE_BEAM_DEPTH,
            BRIDGE_BEAM_H,
        ),
        material="frame_paint",
        name="bridge_crossbeam",
    )
    _add_saddle_visuals(model, frame, "left", LEFT_SUPPORT_X)
    _add_saddle_visuals(model, frame, "right", RIGHT_SUPPORT_X)
    _add_bridge_upright(frame, "left", LEFT_SUPPORT_X)
    _add_bridge_upright(frame, "right", RIGHT_SUPPORT_X)
    frame.inertial = Inertial.from_geometry(
        Box((BASE_LEN, BASE_DEPTH, 0.20)),
        mass=18.0,
        origin=Origin(xyz=(BASE_START_X + BASE_LEN / 2.0, 0.0, 0.10)),
    )

    shaft = model.part("drive_shaft")
    shaft.visual(
        Cylinder(radius=SHAFT_R, length=OUTPUT_STUB_X),
        origin=_x_axis_cylinder_origin(OUTPUT_STUB_X, 0.0, 0.0, 0.0),
        material="shaft_steel",
        name="shaft_body",
    )
    shaft.visual(
        Cylinder(radius=INPUT_COLLAR_R, length=INPUT_COLLAR_LEN),
        origin=_x_axis_cylinder_origin(INPUT_COLLAR_LEN, INPUT_COLLAR_X, 0.0, 0.0),
        material="shaft_steel",
        name="input_collar",
    )
    shaft.visual(
        Cylinder(radius=OUTPUT_SHOULDER_R, length=OUTPUT_SHOULDER_LEN),
        origin=_x_axis_cylinder_origin(
            OUTPUT_SHOULDER_LEN,
            OUTPUT_SHOULDER_X,
            0.0,
            0.0,
        ),
        material="shaft_steel",
        name="output_shoulder",
    )
    shaft.visual(
        Cylinder(radius=OUTPUT_STUB_R, length=OUTPUT_STUB_LEN),
        origin=_x_axis_cylinder_origin(OUTPUT_STUB_LEN, OUTPUT_STUB_X, 0.0, 0.0),
        material="shaft_steel",
        name="output_stub",
    )
    shaft.visual(
        Box((DRIVE_TAB_LEN, DRIVE_TAB_W, DRIVE_TAB_H)),
        origin=_box_origin_from_min(
            DRIVE_TAB_X,
            0.0,
            DRIVE_TAB_BASE_Z,
            DRIVE_TAB_LEN,
            DRIVE_TAB_W,
            DRIVE_TAB_H,
        ),
        material="oxide_steel",
        name="drive_tab",
    )
    shaft.inertial = Inertial.from_geometry(
        Cylinder(radius=SHAFT_R, length=SHAFT_LEN),
        mass=1.6,
        origin=Origin(
            xyz=(SHAFT_LEN / 2.0, 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
    )

    model.articulation(
        "frame_to_shaft",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=shaft,
        origin=Origin(xyz=(0.0, 0.0, AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=16.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("bridge_frame")
    shaft = object_model.get_part("drive_shaft")
    shaft_joint = object_model.get_articulation("frame_to_shaft")

    bridge_crossbeam = frame.get_visual("bridge_crossbeam")
    left_saddle_pad = frame.get_visual("left_saddle_pad")
    right_saddle_pad = frame.get_visual("right_saddle_pad")
    right_upright = frame.get_visual("right_bridge_upright")
    shaft_body = shaft.get_visual("shaft_body")
    drive_tab = shaft.get_visual("drive_tab")

    ctx.check(
        "shaft uses a continuous rotation joint about the shaft centerline",
        shaft_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 6) for v in shaft_joint.axis) == (1.0, 0.0, 0.0),
        details=(
            f"type={shaft_joint.articulation_type}, axis={shaft_joint.axis}, "
            f"origin={shaft_joint.origin}"
        ),
    )

    ctx.expect_contact(
        shaft,
        frame,
        elem_a=shaft_body,
        elem_b=left_saddle_pad,
        name="left saddle pad physically supports the shaft",
    )
    ctx.expect_contact(
        shaft,
        frame,
        elem_a=shaft_body,
        elem_b=right_saddle_pad,
        name="right saddle pad physically supports the shaft",
    )
    ctx.expect_gap(
        frame,
        shaft,
        axis="z",
        positive_elem=bridge_crossbeam,
        negative_elem=shaft_body,
        min_gap=0.005,
        name="bridge crossbeam stays above the rotating shaft",
    )
    ctx.expect_gap(
        shaft,
        frame,
        axis="x",
        positive_elem=drive_tab,
        negative_elem=right_upright,
        min_gap=0.08,
        name="output end remains visibly beyond the right support",
    )

    def _center_from_aabb(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) / 2.0 for i in range(3))

    rest_tab_center = _center_from_aabb(
        ctx.part_element_world_aabb(shaft, elem="drive_tab")
    )
    with ctx.pose({shaft_joint: pi / 2.0}):
        quarter_turn_tab_center = _center_from_aabb(
            ctx.part_element_world_aabb(shaft, elem="drive_tab")
        )

    ctx.check(
        "drive tab visibly orbits when the shaft rotates",
        rest_tab_center is not None
        and quarter_turn_tab_center is not None
        and abs(quarter_turn_tab_center[0] - rest_tab_center[0]) < 0.002
        and abs(quarter_turn_tab_center[1] - rest_tab_center[1]) > 0.006
        and abs(quarter_turn_tab_center[2] - rest_tab_center[2]) > 0.006,
        details=(
            f"rest={rest_tab_center}, quarter_turn={quarter_turn_tab_center}, "
            f"axis_z={AXIS_Z}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
