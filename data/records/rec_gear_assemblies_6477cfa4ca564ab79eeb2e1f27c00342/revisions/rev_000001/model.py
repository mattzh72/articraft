from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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


FRAME_WIDTH = 0.120
PLATE_THICKNESS = 0.010
BASE_THICKNESS = 0.014
PLATE_HEIGHT = 0.170
FRAME_LENGTH = 0.260
SHAFT_CENTER_Z = 0.090
SHAFT_RADIUS = 0.008
BEARING_HOLE_RADIUS = 0.0088
COLLAR_RADIUS = 0.014
COLLAR_THICKNESS = 0.003
GEAR_THICKNESS = 0.012
STAGE1_Y = -0.015
STAGE2_Y = 0.015
EPS = 0.0005
BRACE_THICKNESS = 0.008
SUPPORT_BLOCK_THICKNESS = 0.012
SUPPORT_BLOCK_WIDTH = 0.026
SUPPORT_BLOCK_HEIGHT = 0.108
SUPPORT_Y_CENTER = 0.051
SUPPORT_OUTER_FACE_Y = SUPPORT_Y_CENTER + SUPPORT_BLOCK_THICKNESS / 2.0

INPUT_STAGE1_TIP_R = 0.027
INPUT_STAGE1_ROOT_R = 0.0205
MID_STAGE1_TIP_R = 0.054
MID_STAGE1_ROOT_R = 0.0470
MID_STAGE2_TIP_R = 0.024
MID_STAGE2_ROOT_R = 0.0175
OUTPUT_STAGE2_TIP_R = 0.060
OUTPUT_STAGE2_ROOT_R = 0.0535

INPUT_TO_INTERMEDIATE = INPUT_STAGE1_TIP_R + MID_STAGE1_TIP_R + 0.0012
INTERMEDIATE_TO_OUTPUT = MID_STAGE2_TIP_R + OUTPUT_STAGE2_TIP_R + 0.0012

INPUT_SHAFT_X = -INPUT_TO_INTERMEDIATE
INTERMEDIATE_SHAFT_X = 0.0
OUTPUT_SHAFT_X = INTERMEDIATE_TO_OUTPUT


def polar_xy(radius: float, angle: float) -> tuple[float, float]:
    return (radius * math.cos(angle), radius * math.sin(angle))


def build_spur_gear(
    *,
    tip_radius: float,
    root_radius: float,
    thickness: float,
    teeth: int,
    y_center: float,
    hub_radius: float,
    phase: float = 0.0,
) -> cq.Workplane:
    tooth_angle = 2.0 * math.pi / teeth
    points: list[tuple[float, float]] = []
    for tooth in range(teeth):
        a = phase + tooth * tooth_angle
        points.extend(
            [
                polar_xy(root_radius, a - 0.46 * tooth_angle),
                polar_xy(tip_radius, a - 0.19 * tooth_angle),
                polar_xy(tip_radius, a + 0.19 * tooth_angle),
                polar_xy(root_radius, a + 0.46 * tooth_angle),
            ]
        )

    gear = (
        cq.Workplane("XY")
        .polyline(points)
        .close()
        .extrude(thickness)
        .translate((0.0, 0.0, -thickness / 2.0))
    )

    hub_length = thickness
    hub = (
        cq.Workplane("XY")
        .circle(hub_radius)
        .extrude(hub_length)
        .translate((0.0, 0.0, -hub_length / 2.0))
    )
    gear = gear.union(hub)

    if root_radius > 0.026:
        hole_radius = min(root_radius * 0.22, 0.010)
        hole_pitch = (hub_radius + root_radius) * 0.55
        for idx in range(4):
            angle = phase + idx * (math.pi / 2.0)
            cutter = (
                cq.Workplane("XY")
                .circle(hole_radius)
                .extrude(hub_length + 0.002)
                .translate(
                    (
                        hole_pitch * math.cos(angle),
                        hole_pitch * math.sin(angle),
                        -(hub_length + 0.002) / 2.0,
                    )
                )
            )
            gear = gear.cut(cutter)

    return gear.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0).translate((0.0, y_center, 0.0))


def build_y_cylinder(*, radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    x_pos, y_pos, z_pos = center
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate((x_pos, y_pos, z_pos))
    )


def build_z_cylinder(*, radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    x_pos, y_pos, z_pos = center
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((x_pos, y_pos, z_pos - length / 2.0))
    )


def build_shaft_core(
    *,
    length: float,
    radius: float,
    with_input_disc: bool = False,
    with_output_disc: bool = False,
) -> cq.Workplane:
    shaft = build_y_cylinder(radius=radius, length=length, center=(0.0, 0.0, 0.0))

    left_collar = (
        build_y_cylinder(
            radius=COLLAR_RADIUS,
            length=COLLAR_THICKNESS,
            center=(0.0, -SUPPORT_OUTER_FACE_Y - COLLAR_THICKNESS / 2.0, 0.0),
        )
    )
    right_collar = (
        build_y_cylinder(
            radius=COLLAR_RADIUS,
            length=COLLAR_THICKNESS,
            center=(0.0, SUPPORT_OUTER_FACE_Y + COLLAR_THICKNESS / 2.0, 0.0),
        )
    )
    shaft = shaft.union(left_collar).union(right_collar)

    if with_input_disc:
        handwheel = build_y_cylinder(radius=0.019, length=0.008, center=(0.0, -0.078, 0.0))
        grip = build_z_cylinder(radius=0.0045, length=0.016, center=(0.014, -0.082, 0.0))
        shaft = shaft.union(handwheel).union(grip)

    if with_output_disc:
        output_flange = build_y_cylinder(radius=0.024, length=0.010, center=(0.0, 0.077, 0.0))
        output_hub = build_y_cylinder(radius=0.013, length=0.016, center=(0.0, 0.090, 0.0))
        shaft = shaft.union(output_flange).union(output_hub)

    return shaft


def build_bearing_block(x_pos: float, y_center: float) -> cq.Workplane:
    block = cq.Workplane("XY").box(
        SUPPORT_BLOCK_WIDTH,
        SUPPORT_BLOCK_THICKNESS,
        SUPPORT_BLOCK_HEIGHT,
    ).translate((x_pos, y_center, SUPPORT_BLOCK_HEIGHT / 2.0))
    bore = build_y_cylinder(
        radius=BEARING_HOLE_RADIUS,
        length=SUPPORT_BLOCK_THICKNESS + 2.0 * EPS,
        center=(x_pos, y_center, SHAFT_CENTER_Z),
    )
    return block.cut(bore)


def build_frame_shape() -> cq.Workplane:
    base = cq.Workplane("XY").box(FRAME_LENGTH, FRAME_WIDTH, BASE_THICKNESS).translate(
        (0.0, 0.0, BASE_THICKNESS / 2.0)
    )
    left_rail = cq.Workplane("XY").box(0.214, SUPPORT_BLOCK_THICKNESS, 0.024).translate(
        (0.004, -SUPPORT_Y_CENTER, BASE_THICKNESS + 0.012)
    )
    right_rail = cq.Workplane("XY").box(0.214, SUPPORT_BLOCK_THICKNESS, 0.024).translate(
        (0.004, SUPPORT_Y_CENTER, BASE_THICKNESS + 0.012)
    )

    frame = base.union(left_rail).union(right_rail)
    for x_pos in (INPUT_SHAFT_X, INTERMEDIATE_SHAFT_X, OUTPUT_SHAFT_X):
        frame = frame.union(build_bearing_block(x_pos, -SUPPORT_Y_CENTER))
        frame = frame.union(build_bearing_block(x_pos, SUPPORT_Y_CENTER))

    return frame


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_stage_spur_reduction_rig")

    model.material("frame_blue", color=(0.20, 0.28, 0.38, 1.0))
    model.material("steel_dark", color=(0.38, 0.40, 0.43, 1.0))
    model.material("steel_bright", color=(0.72, 0.74, 0.76, 1.0))
    model.material("bronze", color=(0.70, 0.58, 0.28, 1.0))
    model.material("handle_black", color=(0.08, 0.08, 0.08, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(build_frame_shape(), "frame_structure", tolerance=0.0008),
        material="frame_blue",
        name="structure",
    )

    input_shaft = model.part("input_shaft")
    input_shaft.visual(
        mesh_from_cadquery(
            build_shaft_core(length=0.182, radius=SHAFT_RADIUS, with_input_disc=True),
            "input_shaft_core",
            tolerance=0.0006,
        ),
        material="steel_bright",
        name="shaft_core",
    )
    input_shaft.visual(
        mesh_from_cadquery(
            build_spur_gear(
                tip_radius=INPUT_STAGE1_TIP_R,
                root_radius=INPUT_STAGE1_ROOT_R,
                thickness=GEAR_THICKNESS,
                teeth=16,
                y_center=STAGE1_Y,
                hub_radius=0.013,
                phase=0.0,
            ),
            "input_stage1_gear",
            tolerance=0.0004,
        ),
        material="bronze",
        name="stage1_gear",
    )

    intermediate_shaft = model.part("intermediate_shaft")
    intermediate_shaft.visual(
        mesh_from_cadquery(
            build_shaft_core(length=0.136, radius=SHAFT_RADIUS),
            "intermediate_shaft_core",
            tolerance=0.0006,
        ),
        material="steel_bright",
        name="shaft_core",
    )
    intermediate_shaft.visual(
        mesh_from_cadquery(
            build_spur_gear(
                tip_radius=MID_STAGE1_TIP_R,
                root_radius=MID_STAGE1_ROOT_R,
                thickness=GEAR_THICKNESS,
                teeth=34,
                y_center=STAGE1_Y,
                hub_radius=0.014,
                phase=math.pi / 34.0,
            ),
            "intermediate_stage1_gear",
            tolerance=0.0004,
        ),
        material="steel_dark",
        name="stage1_gear",
    )
    intermediate_shaft.visual(
        mesh_from_cadquery(
            build_spur_gear(
                tip_radius=MID_STAGE2_TIP_R,
                root_radius=MID_STAGE2_ROOT_R,
                thickness=GEAR_THICKNESS,
                teeth=14,
                y_center=STAGE2_Y,
                hub_radius=0.012,
                phase=math.pi / 28.0,
            ),
            "intermediate_stage2_gear",
            tolerance=0.0004,
        ),
        material="bronze",
        name="stage2_gear",
    )

    output_shaft = model.part("output_shaft")
    output_shaft.visual(
        mesh_from_cadquery(
            build_shaft_core(length=0.176, radius=SHAFT_RADIUS, with_output_disc=True),
            "output_shaft_core",
            tolerance=0.0006,
        ),
        material="steel_bright",
        name="shaft_core",
    )
    output_shaft.visual(
        mesh_from_cadquery(
            build_spur_gear(
                tip_radius=OUTPUT_STAGE2_TIP_R,
                root_radius=OUTPUT_STAGE2_ROOT_R,
                thickness=GEAR_THICKNESS,
                teeth=38,
                y_center=STAGE2_Y,
                hub_radius=0.014,
                phase=math.pi / 38.0,
            ),
            "output_stage2_gear",
            tolerance=0.0004,
        ),
        material="steel_dark",
        name="stage2_gear",
    )

    limits = MotionLimits(
        effort=12.0,
        velocity=8.0,
        lower=-2.0 * math.pi,
        upper=2.0 * math.pi,
    )

    model.articulation(
        "frame_to_input",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=input_shaft,
        origin=Origin(xyz=(INPUT_SHAFT_X, 0.0, SHAFT_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=limits,
    )
    model.articulation(
        "frame_to_intermediate",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=intermediate_shaft,
        origin=Origin(xyz=(INTERMEDIATE_SHAFT_X, 0.0, SHAFT_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=limits,
    )
    model.articulation(
        "frame_to_output",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=output_shaft,
        origin=Origin(xyz=(OUTPUT_SHAFT_X, 0.0, SHAFT_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=limits,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    input_shaft = object_model.get_part("input_shaft")
    intermediate_shaft = object_model.get_part("intermediate_shaft")
    output_shaft = object_model.get_part("output_shaft")

    input_joint = object_model.get_articulation("frame_to_input")
    intermediate_joint = object_model.get_articulation("frame_to_intermediate")
    output_joint = object_model.get_articulation("frame_to_output")

    input_stage1 = input_shaft.get_visual("stage1_gear")
    intermediate_stage1 = intermediate_shaft.get_visual("stage1_gear")
    intermediate_stage2 = intermediate_shaft.get_visual("stage2_gear")
    output_stage2 = output_shaft.get_visual("stage2_gear")
    frame_structure = frame.get_visual("structure")
    input_core = input_shaft.get_visual("shaft_core")
    intermediate_core = intermediate_shaft.get_visual("shaft_core")
    output_core = output_shaft.get_visual("shaft_core")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.allow_overlap(
        frame,
        input_shaft,
        elem_a=frame_structure,
        elem_b=input_core,
        reason="supported input shaft intentionally passes through open bearing blocks in the fixed frame",
    )
    ctx.allow_overlap(
        frame,
        intermediate_shaft,
        elem_a=frame_structure,
        elem_b=intermediate_core,
        reason="supported intermediate shaft intentionally passes through open bearing blocks in the fixed frame",
    )
    ctx.allow_overlap(
        frame,
        output_shaft,
        elem_a=frame_structure,
        elem_b=output_core,
        reason="supported output shaft intentionally passes through open bearing blocks in the fixed frame",
    )

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
        "expected part count",
        len(object_model.parts) == 4,
        details=f"expected 4 parts, found {len(object_model.parts)}",
    )
    ctx.check(
        "expected revolute shaft count",
        len(object_model.articulations) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in object_model.articulations),
        details="the rig should have exactly three revolute shaft articulations",
    )
    ctx.check(
        "all shaft axes are along y",
        input_joint.axis == (0.0, 1.0, 0.0)
        and intermediate_joint.axis == (0.0, 1.0, 0.0)
        and output_joint.axis == (0.0, 1.0, 0.0),
        details=(
            f"axes were {input_joint.axis}, {intermediate_joint.axis}, {output_joint.axis}; "
            "all shafts should revolve about the supported y-axis"
        ),
    )

    ctx.expect_contact(input_shaft, frame, name="input shaft supported by frame")
    ctx.expect_contact(intermediate_shaft, frame, name="intermediate shaft supported by frame")
    ctx.expect_contact(output_shaft, frame, name="output shaft supported by frame")

    ctx.expect_origin_gap(
        intermediate_shaft,
        input_shaft,
        axis="x",
        min_gap=INPUT_TO_INTERMEDIATE - 0.0005,
        max_gap=INPUT_TO_INTERMEDIATE + 0.0005,
        name="input to intermediate shaft spacing",
    )
    ctx.expect_origin_gap(
        output_shaft,
        intermediate_shaft,
        axis="x",
        min_gap=INTERMEDIATE_TO_OUTPUT - 0.0005,
        max_gap=INTERMEDIATE_TO_OUTPUT + 0.0005,
        name="intermediate to output shaft spacing",
    )

    ctx.expect_overlap(
        input_shaft,
        intermediate_shaft,
        axes="y",
        elem_a=input_stage1,
        elem_b=intermediate_stage1,
        min_overlap=GEAR_THICKNESS - 0.001,
        name="stage one gears share one axial layer",
    )
    ctx.expect_overlap(
        intermediate_shaft,
        output_shaft,
        axes="y",
        elem_a=intermediate_stage2,
        elem_b=output_stage2,
        min_overlap=GEAR_THICKNESS - 0.001,
        name="stage two gears share one axial layer",
    )
    ctx.expect_gap(
        intermediate_shaft,
        intermediate_shaft,
        axis="y",
        positive_elem=intermediate_stage2,
        negative_elem=intermediate_stage1,
        min_gap=0.017,
        max_gap=0.019,
        name="intermediate shaft gears are stacked apart",
    )
    ctx.expect_gap(
        intermediate_shaft,
        input_shaft,
        axis="x",
        positive_elem=intermediate_stage1,
        negative_elem=input_stage1,
        min_gap=0.0004,
        max_gap=0.0030,
        name="stage one gear mesh clearance",
    )
    ctx.expect_gap(
        output_shaft,
        intermediate_shaft,
        axis="x",
        positive_elem=output_stage2,
        negative_elem=intermediate_stage2,
        min_gap=0.0004,
        max_gap=0.0030,
        name="stage two gear mesh clearance",
    )

    with ctx.pose(
        {
            input_joint: 0.7,
            intermediate_joint: -1.1,
            output_joint: 0.45,
        }
    ):
        ctx.expect_contact(input_shaft, frame, name="input shaft remains supported in nonzero pose")
        ctx.expect_contact(
            intermediate_shaft,
            frame,
            name="intermediate shaft remains supported in nonzero pose",
        )
        ctx.expect_contact(output_shaft, frame, name="output shaft remains supported in nonzero pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
