from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


DESK_WIDTH = 1.60
DESK_DEPTH = 0.78
DESK_HEIGHT = 0.76
TOP_THICKNESS = 0.05
CARCASS_HEIGHT = DESK_HEIGHT - TOP_THICKNESS
OUTER_PANEL_THICKNESS = 0.025
PARTITION_THICKNESS = 0.020
KNEE_WIDTH = 0.62
DRAWER_FRONT_THICKNESS = 0.030
FRONT_FACE_X = DESK_DEPTH / 2.0
DRAWER_JOINT_X = FRONT_FACE_X - DRAWER_FRONT_THICKNESS

LEFT_PARTITION_Y = -(KNEE_WIDTH / 2.0 + PARTITION_THICKNESS / 2.0)
RIGHT_PARTITION_Y = KNEE_WIDTH / 2.0 + PARTITION_THICKNESS / 2.0
LEFT_CAVITY_CENTER_Y = (
    (-DESK_WIDTH / 2.0 + OUTER_PANEL_THICKNESS) + (LEFT_PARTITION_Y - PARTITION_THICKNESS / 2.0)
) / 2.0
RIGHT_CAVITY_CENTER_Y = (
    (RIGHT_PARTITION_Y + PARTITION_THICKNESS / 2.0) + (DESK_WIDTH / 2.0 - OUTER_PANEL_THICKNESS)
) / 2.0

LEFT_DRAWER_Z = (0.5875, 0.4175, 0.2475)
CENTER_DRAWER_Z = 0.6225
FILE_DRAWER_Z = 0.4175

EPS = 0.001


def _add_pull(
    part,
    *,
    prefix: str,
    face_thickness: float,
    z: float,
    span: float,
    y_offset: float = 0.0,
    material: str,
) -> None:
    post_x = face_thickness + 0.006
    bar_x = face_thickness + 0.018
    post_spacing = max(span - 0.030, 0.060) / 2.0
    for index, y in enumerate((y_offset - post_spacing, y_offset + post_spacing)):
        part.visual(
            Box((0.012, 0.010, 0.012)),
            origin=Origin(xyz=(post_x, y, z)),
            material=material,
            name=f"{prefix}_post_{index}",
        )
    part.visual(
        Box((0.012, span, 0.012)),
        origin=Origin(xyz=(bar_x, y_offset, z)),
        material=material,
        name=f"{prefix}_bar",
    )


def _add_drawer(
    part,
    *,
    front_width: float,
    front_height: float,
    body_width: float,
    body_height: float,
    body_length: float,
    guide_face_y: float,
    handle_spans: tuple[float, ...],
    handle_offsets: tuple[float, ...],
    front_material: str,
    body_material: str,
    hardware_material: str,
    guide_material: str,
) -> None:
    side_thickness = 0.014
    bottom_thickness = 0.010
    back_thickness = 0.014

    part.visual(
        Box((DRAWER_FRONT_THICKNESS, front_width, front_height)),
        origin=Origin(xyz=(DRAWER_FRONT_THICKNESS / 2.0, 0.0, 0.0)),
        material=front_material,
        name="front",
    )

    body_center_x = -body_length / 2.0 + EPS / 2.0
    side_y = body_width / 2.0 - side_thickness / 2.0
    back_width = body_width - 2.0 * side_thickness + 2.0 * EPS
    bottom_height = -body_height / 2.0 + bottom_thickness / 2.0

    part.visual(
        Box((body_length, side_thickness, body_height)),
        origin=Origin(xyz=(body_center_x, -side_y, 0.0)),
        material=body_material,
        name="side_0",
    )
    part.visual(
        Box((body_length, side_thickness, body_height)),
        origin=Origin(xyz=(body_center_x, side_y, 0.0)),
        material=body_material,
        name="side_1",
    )
    part.visual(
        Box((back_thickness, back_width, body_height)),
        origin=Origin(xyz=(-body_length + back_thickness / 2.0 + EPS / 2.0, 0.0, 0.0)),
        material=body_material,
        name="back",
    )
    part.visual(
        Box((body_length - back_thickness + EPS, back_width, bottom_thickness)),
        origin=Origin(
            xyz=(-(body_length - back_thickness + EPS) / 2.0 + EPS / 2.0, 0.0, bottom_height)
        ),
        material=body_material,
        name="bottom",
    )

    guide_thickness = max(guide_face_y - body_width / 2.0 + EPS, 0.012)
    guide_center_y = (guide_face_y + body_width / 2.0 - EPS) / 2.0
    guide_length = max(body_length - 0.12, 0.34)
    guide_height = min(0.026, max(0.018, body_height * 0.28))
    guide_x = -guide_length / 2.0 + EPS / 2.0

    part.visual(
        Box((guide_length, guide_thickness, guide_height)),
        origin=Origin(xyz=(guide_x, -guide_center_y, 0.0)),
        material=guide_material,
        name="guide_0",
    )
    part.visual(
        Box((guide_length, guide_thickness, guide_height)),
        origin=Origin(xyz=(guide_x, guide_center_y, 0.0)),
        material=guide_material,
        name="guide_1",
    )

    for index, (handle_span, handle_offset) in enumerate(zip(handle_spans, handle_offsets)):
        _add_pull(
            part,
            prefix=f"pull_{index}",
            face_thickness=DRAWER_FRONT_THICKNESS,
            z=0.0,
            span=handle_span,
            y_offset=handle_offset,
            material=hardware_material,
        )


def _add_runner_pair(
    part,
    *,
    prefix: str,
    outer_y: float,
    inner_y: float,
    z: float,
    material: str,
) -> None:
    runner_length = 0.62
    runner_thickness = 0.014
    runner_height = 0.020
    runner_x = 0.045
    part.visual(
        Box((runner_length, runner_thickness, runner_height)),
        origin=Origin(xyz=(runner_x, outer_y, z)),
        material=material,
        name=f"{prefix}_runner_0",
    )
    part.visual(
        Box((runner_length, runner_thickness, runner_height)),
        origin=Origin(xyz=(runner_x, inner_y, z)),
        material=material,
        name=f"{prefix}_runner_1",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="classic_office_desk")

    top_finish = model.material("top_finish", rgba=(0.43, 0.29, 0.17, 1.0))
    case_finish = model.material("case_finish", rgba=(0.35, 0.22, 0.13, 1.0))
    drawer_finish = model.material("drawer_finish", rgba=(0.47, 0.31, 0.18, 1.0))
    drawer_box_finish = model.material("drawer_box_finish", rgba=(0.71, 0.60, 0.45, 1.0))
    runner_finish = model.material("runner_finish", rgba=(0.18, 0.19, 0.21, 1.0))
    hardware_finish = model.material("hardware_finish", rgba=(0.64, 0.55, 0.37, 1.0))

    carcass = model.part("carcass")
    carcass.visual(
        Box((DESK_DEPTH, DESK_WIDTH, TOP_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, DESK_HEIGHT - TOP_THICKNESS / 2.0)),
        material=top_finish,
        name="top",
    )
    carcass.visual(
        Box((0.76, OUTER_PANEL_THICKNESS, CARCASS_HEIGHT)),
        origin=Origin(xyz=(0.01, -DESK_WIDTH / 2.0 + OUTER_PANEL_THICKNESS / 2.0, CARCASS_HEIGHT / 2.0)),
        material=case_finish,
        name="left_side",
    )
    carcass.visual(
        Box((0.76, OUTER_PANEL_THICKNESS, CARCASS_HEIGHT)),
        origin=Origin(xyz=(0.01, DESK_WIDTH / 2.0 - OUTER_PANEL_THICKNESS / 2.0, CARCASS_HEIGHT / 2.0)),
        material=case_finish,
        name="right_side",
    )
    carcass.visual(
        Box((0.72, PARTITION_THICKNESS, CARCASS_HEIGHT)),
        origin=Origin(xyz=(-0.01, LEFT_PARTITION_Y, CARCASS_HEIGHT / 2.0)),
        material=case_finish,
        name="left_partition",
    )
    carcass.visual(
        Box((0.72, PARTITION_THICKNESS, CARCASS_HEIGHT)),
        origin=Origin(xyz=(-0.01, RIGHT_PARTITION_Y, CARCASS_HEIGHT / 2.0)),
        material=case_finish,
        name="right_partition",
    )
    carcass.visual(
        Box((0.020, DESK_WIDTH - 2.0 * OUTER_PANEL_THICKNESS, CARCASS_HEIGHT)),
        origin=Origin(xyz=(-DESK_DEPTH / 2.0 + 0.010, 0.0, CARCASS_HEIGHT / 2.0)),
        material=case_finish,
        name="rear_panel",
    )
    carcass.visual(
        Box((0.68, 0.44, 0.060)),
        origin=Origin(xyz=(-0.03, LEFT_CAVITY_CENTER_Y, 0.030)),
        material=case_finish,
        name="left_base",
    )
    carcass.visual(
        Box((0.68, 0.44, 0.060)),
        origin=Origin(xyz=(-0.03, RIGHT_CAVITY_CENTER_Y, 0.030)),
        material=case_finish,
        name="right_base",
    )
    carcass.visual(
        Box((0.10, KNEE_WIDTH, 0.040)),
        origin=Origin(xyz=(-0.24, 0.0, 0.650)),
        material=case_finish,
        name="center_backer",
    )

    left_outer_runner_y = -DESK_WIDTH / 2.0 + OUTER_PANEL_THICKNESS + 0.006
    left_inner_runner_y = LEFT_PARTITION_Y - PARTITION_THICKNESS / 2.0 - 0.006
    right_inner_runner_y = RIGHT_PARTITION_Y + PARTITION_THICKNESS / 2.0 + 0.006
    right_outer_runner_y = DESK_WIDTH / 2.0 - OUTER_PANEL_THICKNESS - 0.006
    center_left_runner_y = -KNEE_WIDTH / 2.0 + 0.006
    center_right_runner_y = KNEE_WIDTH / 2.0 - 0.006

    for index, drawer_z in enumerate(LEFT_DRAWER_Z):
        _add_runner_pair(
            carcass,
            prefix=f"left_{index}",
            outer_y=left_outer_runner_y,
            inner_y=left_inner_runner_y,
            z=drawer_z,
            material=runner_finish,
        )

    _add_runner_pair(
        carcass,
        prefix="center",
        outer_y=center_left_runner_y,
        inner_y=center_right_runner_y,
        z=CENTER_DRAWER_Z,
        material=runner_finish,
    )
    _add_runner_pair(
        carcass,
        prefix="file",
        outer_y=right_inner_runner_y,
        inner_y=right_outer_runner_y,
        z=FILE_DRAWER_Z,
        material=runner_finish,
    )

    drawer_specs = (
        ("left_drawer_0", LEFT_CAVITY_CENTER_Y, LEFT_DRAWER_Z[0], 0.34, 0.405, 0.155, 0.355, 0.115, 0.63, 0.2095, (0.14,), (0.0,)),
        ("left_drawer_1", LEFT_CAVITY_CENTER_Y, LEFT_DRAWER_Z[1], 0.34, 0.405, 0.155, 0.355, 0.115, 0.63, 0.2095, (0.14,), (0.0,)),
        ("left_drawer_2", LEFT_CAVITY_CENTER_Y, LEFT_DRAWER_Z[2], 0.34, 0.405, 0.155, 0.355, 0.115, 0.63, 0.2095, (0.14,), (0.0,)),
        ("center_drawer", 0.0, CENTER_DRAWER_Z, 0.27, 0.560, 0.085, 0.520, 0.055, 0.52, 0.2970, (0.18,), (0.0,)),
        (
            "right_file_drawer",
            RIGHT_CAVITY_CENTER_Y,
            FILE_DRAWER_Z,
            0.38,
            0.405,
            0.495,
            0.355,
            0.405,
            0.65,
            0.2095,
            (0.14, 0.14),
            (-0.100, 0.100),
        ),
    )

    for (
        name,
        y,
        z,
        travel,
        front_width,
        front_height,
        body_width,
        body_height,
        body_length,
        guide_face_y,
        handle_spans,
        handle_offsets,
    ) in drawer_specs:
        drawer = model.part(name)
        _add_drawer(
            drawer,
            front_width=front_width,
            front_height=front_height,
            body_width=body_width,
            body_height=body_height,
            body_length=body_length,
            guide_face_y=guide_face_y,
            handle_spans=handle_spans,
            handle_offsets=handle_offsets,
            front_material=drawer_finish,
            body_material=drawer_box_finish,
            hardware_material=hardware_finish,
            guide_material=runner_finish,
        )
        model.articulation(
            f"carcass_to_{name}",
            ArticulationType.PRISMATIC,
            parent=carcass,
            child=drawer,
            origin=Origin(xyz=(DRAWER_JOINT_X, y, z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=80.0,
                velocity=0.35,
                lower=0.0,
                upper=travel,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")
    top_aabb = ctx.part_element_world_aabb(carcass, elem="top")
    desk_front_x = top_aabb[1][0] if top_aabb is not None else None

    drawer_names = (
        ("left_drawer_0", 0.20),
        ("left_drawer_1", 0.20),
        ("left_drawer_2", 0.20),
        ("center_drawer", 0.18),
        ("right_file_drawer", 0.22),
    )

    for drawer_name, retained_insertion in drawer_names:
        drawer = object_model.get_part(drawer_name)
        joint = object_model.get_articulation(f"carcass_to_{drawer_name}")
        limits = joint.motion_limits
        upper = limits.upper if limits is not None and limits.upper is not None else 0.0

        front_aabb = ctx.part_element_world_aabb(drawer, elem="front")
        flush_ok = (
            desk_front_x is not None
            and front_aabb is not None
            and abs(front_aabb[1][0] - desk_front_x) <= 0.002
        )
        ctx.check(
            f"{drawer_name} front sits flush when closed",
            flush_ok,
            details=f"desk_front_x={desk_front_x}, drawer_front_max_x={None if front_aabb is None else front_aabb[1][0]}",
        )

        rest_pos = ctx.part_world_position(drawer)
        with ctx.pose({joint: upper}):
            extended_pos = ctx.part_world_position(drawer)
            ctx.check(
                f"{drawer_name} extends forward",
                rest_pos is not None
                and extended_pos is not None
                and extended_pos[0] >= rest_pos[0] + upper - 0.002,
                details=f"rest={rest_pos}, extended={extended_pos}, upper={upper}",
            )

            drawer_aabb = ctx.part_world_aabb(drawer)
            retained_ok = (
                drawer_aabb is not None
                and DRAWER_JOINT_X - drawer_aabb[0][0] >= retained_insertion
            )
            ctx.check(
                f"{drawer_name} keeps runner engagement at full travel",
                retained_ok,
                details=(
                    f"joint_x={DRAWER_JOINT_X}, drawer_back_x="
                    f"{None if drawer_aabb is None else drawer_aabb[0][0]}, "
                    f"retained_min={retained_insertion}"
                ),
            )

    return ctx.report()


object_model = build_object_model()
