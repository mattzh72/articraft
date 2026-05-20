from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_WIDTH = 0.138
BASE_DEPTH = 0.060
BASE_CENTER_Y = -0.014
BASE_THICKNESS = 0.012

AXIS_Z = 0.060

CHEEK_THICKNESS = 0.018
CHEEK_DEPTH = 0.050
CHEEK_CENTER_Y = -0.009
CHEEK_HEIGHT = 0.040
CHEEK_SPAN = 0.102

SADDLE_WIDTH = 0.082
SADDLE_DEPTH = 0.040
SADDLE_CENTER_Y = -0.006
SADDLE_HEIGHT = 0.034
SADDLE_CUT_RADIUS = 0.021

HOUSING_RADIUS = 0.022
HOUSING_LENGTH = 0.036
HOUSING_BORE_RADIUS = 0.0135

BRIDGE_WIDTH = 0.108
BRIDGE_DEPTH = 0.032
BRIDGE_CENTER_Y = -0.002
BRIDGE_HEIGHT = 0.022
BRIDGE_CUT_RADIUS = 0.029
BRIDGE_CENTER_Z = AXIS_Z + HOUSING_RADIUS + 0.013
BRIDGE_POST_WIDTH = 0.016
BRIDGE_POST_HEIGHT = BRIDGE_CENTER_Z - BRIDGE_HEIGHT / 2.0 - (BASE_THICKNESS + CHEEK_HEIGHT)

SHAFT_RADIUS = 0.0115
SHAFT_START_Y = -0.018
SHAFT_LENGTH = 0.037

OUTPUT_FACE_RADIUS = 0.026
OUTPUT_FACE_GAP = 0.0
OUTPUT_FACE_THICKNESS = 0.007
OUTPUT_FACE_BACK_Y = HOUSING_LENGTH / 2.0 + OUTPUT_FACE_GAP


def _box(length: float, width: float, height: float, center_xyz: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(length, width, height).translate(center_xyz)


def _y_cylinder(
    radius: float,
    length: float,
    *,
    center_y: float = 0.0,
    center_z: float = 0.0,
    center_x: float = 0.0,
) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .center(center_x, center_z)
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .translate((0.0, center_y, 0.0))
    )


def _make_support_module() -> cq.Workplane:
    base = _box(BASE_WIDTH, BASE_DEPTH, BASE_THICKNESS, (0.0, BASE_CENTER_Y, BASE_THICKNESS / 2.0))

    cheek_x = CHEEK_SPAN / 2.0
    cheek_z = BASE_THICKNESS + CHEEK_HEIGHT / 2.0
    left_cheek = _box(
        CHEEK_THICKNESS,
        CHEEK_DEPTH,
        CHEEK_HEIGHT,
        (-cheek_x, CHEEK_CENTER_Y, cheek_z),
    )
    right_cheek = _box(
        CHEEK_THICKNESS,
        CHEEK_DEPTH,
        CHEEK_HEIGHT,
        (cheek_x, CHEEK_CENTER_Y, cheek_z),
    )

    saddle = _box(
        SADDLE_WIDTH,
        SADDLE_DEPTH,
        SADDLE_HEIGHT,
        (0.0, SADDLE_CENTER_Y, BASE_THICKNESS + SADDLE_HEIGHT / 2.0),
    )
    saddle_cut = _y_cylinder(
        SADDLE_CUT_RADIUS,
        SADDLE_DEPTH + 0.010,
        center_z=AXIS_Z,
    )
    saddle = saddle.cut(saddle_cut)

    housing = _y_cylinder(HOUSING_RADIUS, HOUSING_LENGTH, center_z=AXIS_Z)
    bore = _y_cylinder(HOUSING_BORE_RADIUS, HOUSING_LENGTH + 0.006, center_z=AXIS_Z)
    housing = housing.cut(bore)

    bridge = _box(BRIDGE_WIDTH, BRIDGE_DEPTH, BRIDGE_HEIGHT, (0.0, BRIDGE_CENTER_Y, BRIDGE_CENTER_Z))
    arch_cut = _y_cylinder(
        BRIDGE_CUT_RADIUS,
        BRIDGE_DEPTH + 0.010,
        center_y=BRIDGE_CENTER_Y,
        center_z=AXIS_Z + 0.003,
    )
    bridge = bridge.cut(arch_cut)

    post_z = BASE_THICKNESS + CHEEK_HEIGHT + BRIDGE_POST_HEIGHT / 2.0
    left_post = _box(
        BRIDGE_POST_WIDTH,
        BRIDGE_DEPTH,
        BRIDGE_POST_HEIGHT,
        (-0.044, BRIDGE_CENTER_Y, post_z),
    )
    right_post = _box(
        BRIDGE_POST_WIDTH,
        BRIDGE_DEPTH,
        BRIDGE_POST_HEIGHT,
        (0.044, BRIDGE_CENTER_Y, post_z),
    )

    rear_pad = _box(0.060, 0.020, 0.016, (0.0, -0.033, BASE_THICKNESS + 0.008))

    body = (
        base.union(left_cheek)
        .union(right_cheek)
        .union(saddle)
        .union(housing)
        .union(left_post)
        .union(right_post)
        .union(bridge)
        .union(rear_pad)
    )
    return body


def _make_spindle_shaft() -> cq.Workplane:
    return _y_cylinder(
        SHAFT_RADIUS,
        SHAFT_LENGTH,
        center_y=SHAFT_START_Y + SHAFT_LENGTH / 2.0,
    )


def _make_output_face() -> cq.Workplane:
    disk_center_y = OUTPUT_FACE_BACK_Y + OUTPUT_FACE_THICKNESS / 2.0
    face = _y_cylinder(
        OUTPUT_FACE_RADIUS,
        OUTPUT_FACE_THICKNESS,
        center_y=disk_center_y,
    )
    recess = _y_cylinder(
        0.008,
        0.0020,
        center_y=OUTPUT_FACE_BACK_Y + OUTPUT_FACE_THICKNESS - 0.0008,
    )
    marker = _y_cylinder(
        0.0036,
        0.0016,
        center_y=OUTPUT_FACE_BACK_Y + OUTPUT_FACE_THICKNESS + 0.0008,
        center_x=0.015,
    )
    return face.cut(recess).union(marker)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_capped_roll_module")

    dark_frame = model.material("dark_frame", rgba=(0.18, 0.20, 0.22, 1.0))
    cartridge_finish = model.material("cartridge_finish", rgba=(0.64, 0.67, 0.71, 1.0))
    output_finish = model.material("output_finish", rgba=(0.78, 0.80, 0.83, 1.0))

    saddle_body = model.part("saddle_body")
    saddle_body.visual(
        mesh_from_cadquery(_make_support_module(), "support_module"),
        material=dark_frame,
        name="support_module",
    )
    saddle_body.inertial = Inertial.from_geometry(
        Box((BASE_WIDTH, 0.064, 0.108)),
        mass=1.35,
        origin=Origin(xyz=(0.0, -0.010, 0.054)),
    )

    output_face = model.part("output_face")
    output_face.visual(
        mesh_from_cadquery(_make_spindle_shaft(), "spindle_shaft"),
        material=cartridge_finish,
        name="spindle_shaft",
    )
    output_face.visual(
        mesh_from_cadquery(_make_output_face(), "output_disk"),
        material=output_finish,
        name="output_disk",
    )
    output_face.inertial = Inertial.from_geometry(
        Box((OUTPUT_FACE_RADIUS * 2.0, SHAFT_LENGTH + OUTPUT_FACE_THICKNESS, OUTPUT_FACE_RADIUS * 2.0)),
        mass=0.18,
        origin=Origin(
            xyz=(
                0.0,
                (SHAFT_START_Y + OUTPUT_FACE_BACK_Y + OUTPUT_FACE_THICKNESS) / 2.0,
                0.0,
            )
        ),
    )

    model.articulation(
        "roll_spin",
        ArticulationType.REVOLUTE,
        parent=saddle_body,
        child=output_face,
        origin=Origin(xyz=(0.0, 0.0, AXIS_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=6.0,
            lower=-pi,
            upper=pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    saddle_body = object_model.get_part("saddle_body")
    output_face = object_model.get_part("output_face")
    roll_spin = object_model.get_articulation("roll_spin")
    support_module = saddle_body.get_visual("support_module")
    spindle_shaft = output_face.get_visual("spindle_shaft")
    output_disk = output_face.get_visual("output_disk")

    limits = roll_spin.motion_limits
    ctx.check(
        "roll joint axis follows supported spindle line",
        tuple(round(v, 6) for v in roll_spin.axis) == (0.0, 1.0, 0.0),
        details=f"axis={roll_spin.axis}",
    )
    ctx.check(
        "roll joint offers near full-turn travel",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower <= -3.0
        and limits.upper >= 3.0,
        details=f"limits={limits}",
    )

    ctx.expect_overlap(
        output_face,
        saddle_body,
        axes="xz",
        min_overlap=0.023,
        name="output stays centered within the supported roll window",
    )
    ctx.expect_overlap(
        output_face,
        saddle_body,
        axes="y",
        elem_a=spindle_shaft,
        elem_b=support_module,
        min_overlap=0.032,
        name="spindle shaft remains inserted through the cartridge depth",
    )
    ctx.expect_gap(
        output_face,
        saddle_body,
        axis="y",
        positive_elem=output_disk,
        negative_elem=support_module,
        min_gap=0.0,
        max_gap=0.0002,
        name="output face seats flush to the cartridge nose without penetration",
    )

    rest_pos = ctx.part_world_position(output_face)
    with ctx.pose({roll_spin: 1.4}):
        spun_pos = ctx.part_world_position(output_face)
        ctx.expect_overlap(
            output_face,
            saddle_body,
            axes="xz",
            min_overlap=0.023,
            name="spun output remains captured by the body support",
        )
        ctx.expect_gap(
            output_face,
            saddle_body,
            axis="y",
            positive_elem=output_disk,
            negative_elem=support_module,
            min_gap=0.0,
            max_gap=0.0002,
            name="spun output face stays seated at the cartridge nose",
        )

    same_axis_center = (
        rest_pos is not None
        and spun_pos is not None
        and abs(rest_pos[0] - spun_pos[0]) <= 1e-6
        and abs(rest_pos[1] - spun_pos[1]) <= 1e-6
        and abs(rest_pos[2] - spun_pos[2]) <= 1e-6
    )
    ctx.check(
        "output rotates about a fixed supported axis without orbital drift",
        same_axis_center,
        details=f"rest={rest_pos}, spun={spun_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
