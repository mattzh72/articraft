from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose, pi

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


BASE_LEN = 0.30
BASE_WID = 0.20
BASE_THK = 0.016

PLATE_THK = 0.014
PLATE_SPAN_Y = 0.16
PLATE_HEIGHT = 0.160
LEFT_PLATE_X = -0.105
RIGHT_PLATE_X = 0.115
PLATE_Y = 0.020

INPUT_X = -0.065
INPUT_Y = 0.000
BEVEL_APEX_Z = 0.118
PEDESTAL_TOP_Z = 0.088
OUTPUT_Y = 0.055

SHAFT_R = 0.0045
VERT_SHAFT_R = 0.0045
HORIZ_BORE_R = 0.0062
COLLAR_R = 0.0105
COLLAR_T = 0.004

PINION_TEETH = 12
GEAR_TEETH = 24
SPUR_WIDTH = 0.014
SPUR_CENTER_X = 0.040

PINION_ROOT_R = 0.0135
PINION_TIP_R = 0.0170
GEAR_ROOT_R = 0.0305
GEAR_TIP_R = 0.0360

BEVEL_TOOTH_COUNT = 12
BEVEL_FACE = 0.016
BEVEL_RADII = (0.0070, 0.0095, 0.0120, 0.0145)


def _cyl_x(radius: float, length: float, *, cx: float = 0.0, cy: float = 0.0, cz: float = 0.0):
    return (
        cq.Workplane("YZ")
        .circle(radius)
        .extrude(length)
        .translate((cx - length / 2.0, cy, cz))
        .val()
    )


def _cyl_z(radius: float, length: float, *, cx: float = 0.0, cy: float = 0.0, cz: float = 0.0):
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((cx, cy, cz - length / 2.0))
        .val()
    )


def _box(sx: float, sy: float, sz: float, *, cx: float = 0.0, cy: float = 0.0, cz: float = 0.0):
    return cq.Workplane("XY").box(sx, sy, sz).translate((cx, cy, cz)).val()


def _fuse_all(shapes):
    result = shapes[0]
    for shape in shapes[1:]:
        result = result.fuse(shape)
    return result


def _spur_gear_x(
    *,
    root_radius: float,
    tip_radius: float,
    width: float,
    teeth: int,
    cx: float = 0.0,
    hub_radius: float = 0.0085,
    hub_length: float = 0.020,
):
    parts = [_cyl_x(root_radius, width, cx=cx), _cyl_x(hub_radius, hub_length, cx=cx)]
    tooth_depth = tip_radius - root_radius
    tooth_thickness = 0.0042 if teeth <= 12 else 0.0038
    for i in range(teeth):
        tooth = _box(
            width,
            tooth_depth + 0.001,
            tooth_thickness,
            cx=cx,
            cy=root_radius + tooth_depth / 2.0,
            cz=0.0,
        ).rotate((0, 0, 0), (1, 0, 0), 360.0 * i / teeth)
        parts.append(tooth)
    return _fuse_all(parts)


def _bevel_gear_z():
    step = BEVEL_FACE / len(BEVEL_RADII)
    parts = []
    for i, radius in enumerate(BEVEL_RADII):
        parts.append(_cyl_z(radius, step * 0.96, cz=0.006 + i * step + step * 0.48))
    parts.append(_cyl_z(0.008, 0.010, cz=0.020))
    tooth_depth = 0.0035
    for i in range(BEVEL_TOOTH_COUNT):
        tooth = _box(
            tooth_depth,
            0.0030,
            0.0065,
            cx=BEVEL_RADII[-2] + tooth_depth / 2.0 - 0.0012,
            cy=0.0,
            cz=0.016,
        ).rotate((0, 0, 0), (0, 0, 1), 360.0 * i / BEVEL_TOOTH_COUNT)
        parts.append(tooth)
    return _fuse_all(parts)


def _bevel_gear_x():
    step = BEVEL_FACE / len(BEVEL_RADII)
    x_start = -0.022
    parts = []
    for i, radius in enumerate(reversed(BEVEL_RADII)):
        parts.append(_cyl_x(radius, step * 0.96, cx=x_start + i * step + step * 0.48))
    parts.append(_cyl_x(0.008, 0.010, cx=-0.011))
    tooth_depth = 0.0035
    for i in range(BEVEL_TOOTH_COUNT):
        tooth = _box(
            0.0065,
            tooth_depth,
            0.0030,
            cx=-0.016,
            cy=BEVEL_RADII[-2] + tooth_depth / 2.0 - 0.0012,
            cz=0.0,
        ).rotate((0, 0, 0), (1, 0, 0), 360.0 * i / BEVEL_TOOTH_COUNT)
        parts.append(tooth)
    return _fuse_all(parts)


def _make_frame():
    frame = _fuse_all(
        [
            _box(BASE_LEN, BASE_WID, BASE_THK, cz=BASE_THK / 2.0),
            _box(
                PLATE_THK,
                PLATE_SPAN_Y,
                PLATE_HEIGHT,
                cx=LEFT_PLATE_X,
                cy=PLATE_Y,
                cz=BASE_THK + PLATE_HEIGHT / 2.0,
            ),
            _box(
                PLATE_THK,
                PLATE_SPAN_Y,
                PLATE_HEIGHT,
                cx=RIGHT_PLATE_X,
                cy=PLATE_Y,
                cz=BASE_THK + PLATE_HEIGHT / 2.0,
            ),
            _box(
                RIGHT_PLATE_X - LEFT_PLATE_X,
                0.028,
                0.016,
                cx=0.5 * (LEFT_PLATE_X + RIGHT_PLATE_X),
                cy=-0.045,
                cz=0.148,
            ),
            _box(
                RIGHT_PLATE_X - LEFT_PLATE_X,
                0.016,
                0.054,
                cx=0.5 * (LEFT_PLATE_X + RIGHT_PLATE_X),
                cy=-0.018,
                cz=0.043,
            ),
            _cyl_z(0.024, PEDESTAL_TOP_Z - BASE_THK, cx=INPUT_X, cy=INPUT_Y, cz=0.5 * (PEDESTAL_TOP_Z + BASE_THK)),
            _cyl_z(0.018, 0.010, cx=INPUT_X, cy=INPUT_Y, cz=PEDESTAL_TOP_Z - 0.005),
            _box(0.052, 0.024, 0.040, cx=-0.086, cy=-0.012, cz=0.036),
        ]
    )
    for x in (LEFT_PLATE_X, RIGHT_PLATE_X):
        frame = frame.cut(_cyl_x(HORIZ_BORE_R, PLATE_THK + 0.008, cx=x, cy=0.0, cz=BEVEL_APEX_Z))
        frame = frame.cut(_cyl_x(HORIZ_BORE_R, PLATE_THK + 0.008, cx=x, cy=OUTPUT_Y, cz=BEVEL_APEX_Z))
    return frame


def _make_input_shaft():
    pedestal_top_local = PEDESTAL_TOP_Z - BEVEL_APEX_Z
    shaft_top = 0.078
    return _fuse_all(
        [
            _cyl_z(
                VERT_SHAFT_R,
                shaft_top - pedestal_top_local,
                cz=0.5 * (shaft_top + pedestal_top_local),
            ),
            _cyl_z(COLLAR_R, COLLAR_T, cz=pedestal_top_local + COLLAR_T / 2.0),
            _bevel_gear_z(),
            _cyl_z(0.0095, 0.016, cz=0.058),
            _cyl_z(0.0065, 0.012, cz=0.072),
        ]
    )


def _shaft_outer_faces(origin_x: float):
    left_outer = (LEFT_PLATE_X - PLATE_THK / 2.0) - origin_x
    right_outer = (RIGHT_PLATE_X + PLATE_THK / 2.0) - origin_x
    return left_outer, right_outer


def _make_intermediate_shaft():
    left_outer, right_outer = _shaft_outer_faces(INPUT_X)
    shaft_min = left_outer - COLLAR_T
    shaft_max = right_outer + COLLAR_T
    pinion_center = SPUR_CENTER_X - INPUT_X
    return _fuse_all(
        [
            _cyl_x(SHAFT_R, shaft_max - shaft_min, cx=0.5 * (shaft_min + shaft_max)),
            _cyl_x(COLLAR_R, COLLAR_T, cx=left_outer - COLLAR_T / 2.0),
            _cyl_x(COLLAR_R, COLLAR_T, cx=right_outer + COLLAR_T / 2.0),
            _bevel_gear_x(),
            _spur_gear_x(
                root_radius=PINION_ROOT_R,
                tip_radius=PINION_TIP_R,
                width=SPUR_WIDTH,
                teeth=PINION_TEETH,
                cx=pinion_center,
                hub_radius=0.0085,
                hub_length=0.020,
            ),
        ]
    )


def _make_output_shaft():
    left_outer, right_outer = _shaft_outer_faces(SPUR_CENTER_X)
    shaft_min = left_outer - COLLAR_T
    shaft_max = right_outer + COLLAR_T
    return _fuse_all(
        [
            _cyl_x(SHAFT_R, shaft_max - shaft_min, cx=0.5 * (shaft_min + shaft_max)),
            _cyl_x(COLLAR_R, COLLAR_T, cx=left_outer - COLLAR_T / 2.0),
            _cyl_x(COLLAR_R, COLLAR_T, cx=right_outer + COLLAR_T / 2.0),
            _spur_gear_x(
                root_radius=GEAR_ROOT_R,
                tip_radius=GEAR_TIP_R,
                width=SPUR_WIDTH,
                teeth=GEAR_TEETH,
                cx=0.0,
                hub_radius=0.0090,
                hub_length=0.022,
            ),
        ]
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bevel_to_spur_transfer")

    frame_mat = model.material("frame_mat", rgba=(0.22, 0.23, 0.26, 1.0))
    rotor_mat = model.material("rotor_mat", rgba=(0.69, 0.70, 0.67, 1.0))
    gear_mat = model.material("gear_mat", rgba=(0.78, 0.63, 0.28, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_frame(), "frame"),
        origin=Origin(),
        material=frame_mat,
        name="frame",
    )

    input_shaft = model.part("input_shaft")
    input_shaft.visual(
        mesh_from_cadquery(_make_input_shaft(), "input_shaft"),
        origin=Origin(),
        material=gear_mat,
        name="rotor",
    )

    intermediate = model.part("intermediate_shaft")
    intermediate.visual(
        mesh_from_cadquery(_make_intermediate_shaft(), "intermediate_shaft"),
        origin=Origin(),
        material=gear_mat,
        name="rotor",
    )

    output = model.part("output_shaft")
    output.visual(
        mesh_from_cadquery(_make_output_shaft(), "output_shaft"),
        origin=Origin(),
        material=rotor_mat,
        name="rotor",
    )

    model.articulation(
        "base_to_input_shaft",
        ArticulationType.REVOLUTE,
        parent=base,
        child=input_shaft,
        origin=Origin(xyz=(INPUT_X, INPUT_Y, BEVEL_APEX_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=12.0, lower=-2.0 * pi, upper=2.0 * pi),
    )
    model.articulation(
        "base_to_intermediate_shaft",
        ArticulationType.REVOLUTE,
        parent=base,
        child=intermediate,
        origin=Origin(xyz=(INPUT_X, 0.0, BEVEL_APEX_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=12.0, lower=-2.0 * pi, upper=2.0 * pi),
    )
    model.articulation(
        "base_to_output_shaft",
        ArticulationType.REVOLUTE,
        parent=base,
        child=output,
        origin=Origin(xyz=(SPUR_CENTER_X, OUTPUT_Y, BEVEL_APEX_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=12.0, lower=-2.0 * pi, upper=2.0 * pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    input_shaft = object_model.get_part("input_shaft")
    intermediate = object_model.get_part("intermediate_shaft")
    output = object_model.get_part("output_shaft")
    input_joint = object_model.get_articulation("base_to_input_shaft")
    intermediate_joint = object_model.get_articulation("base_to_intermediate_shaft")
    output_joint = object_model.get_articulation("base_to_output_shaft")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        base,
        input_shaft,
        reason="Input shaft is visually carried inside the pedestal bearing bore; the hidden bearing fit is represented as nested support geometry.",
    )
    ctx.allow_overlap(
        base,
        intermediate,
        reason="Intermediate shaft passes through side-plate bearing bores; the shaft-support fit is modeled as an embedded supported axle.",
    )
    ctx.allow_overlap(
        base,
        output,
        reason="Output shaft passes through side-plate bearing bores; the supported axle fit is intentionally nested in the frame bearings.",
    )
    ctx.allow_overlap(
        input_shaft,
        intermediate,
        reason="The bevel transfer is represented with simplified meshing teeth, which intentionally share small penetration at the pitch contact.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(input_shaft, base, name="input_shaft_supported_by_pedestal")
    ctx.expect_contact(intermediate, base, name="intermediate_shaft_supported_by_sideplates")
    ctx.expect_contact(output, base, name="output_shaft_supported_by_sideplates")
    ctx.expect_origin_distance(
        input_shaft,
        intermediate,
        axes="xyz",
        max_dist=1e-6,
        name="bevel_stage_axes_intersect_at_joint_frame",
    )
    ctx.expect_origin_gap(
        output,
        intermediate,
        axis="y",
        min_gap=0.054,
        max_gap=0.056,
        name="spur_stage_has_parallel_shaft_spacing",
    )
    ctx.expect_overlap(
        intermediate,
        output,
        axes="xz",
        min_overlap=0.010,
        name="spur_stage_gears_align_in_xz_projection",
    )

    def _vec_close(actual, expected, tol=1e-6):
        return all(abs(a - b) <= tol for a, b in zip(actual, expected))

    ctx.check(
        "input_joint_axis_is_vertical",
        _vec_close(input_joint.axis, (0.0, 0.0, 1.0)),
        f"axis={input_joint.axis}",
    )
    ctx.check(
        "intermediate_and_output_joint_axes_are_parallel_x",
        _vec_close(intermediate_joint.axis, (1.0, 0.0, 0.0))
        and _vec_close(output_joint.axis, (1.0, 0.0, 0.0)),
        f"intermediate={intermediate_joint.axis}, output={output_joint.axis}",
    )
    ctx.check(
        "bevel_pair_shares_common_transfer_point",
        isclose(input_joint.origin.xyz[0], intermediate_joint.origin.xyz[0], abs_tol=1e-6)
        and isclose(input_joint.origin.xyz[1], intermediate_joint.origin.xyz[1], abs_tol=1e-6)
        and isclose(input_joint.origin.xyz[2], intermediate_joint.origin.xyz[2], abs_tol=1e-6),
        f"input_origin={input_joint.origin.xyz}, intermediate_origin={intermediate_joint.origin.xyz}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
