from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    WheelGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


CABINET_DEPTH = 0.460
CABINET_WIDTH = 0.680
CABINET_HEIGHT = 0.820
SIDE_FRAME = 0.035
BACK_WALL = 0.040
BOTTOM_FRAME = 0.050
TOP_FRAME = 0.030

CAVITY_DEPTH = 0.430
CAVITY_CENTER_X = 0.025
CAVITY_WIDTH = CABINET_WIDTH - 2.0 * SIDE_FRAME
CAVITY_HEIGHT = CABINET_HEIGHT - BOTTOM_FRAME - TOP_FRAME
CAVITY_BOTTOM_Z = BOTTOM_FRAME

DRAWER_FACE_THICKNESS = 0.018
DRAWER_FACE_WIDTH = 0.602
DRAWER_BOX_DEPTH = 0.385
DRAWER_BOX_WIDTH = 0.576
DRAWER_SIDE_WALL = 0.010
DRAWER_FRONT_WALL = 0.010
DRAWER_REAR_WALL = 0.010
DRAWER_BOTTOM_WALL = 0.010
DRAWER_TRAVEL = 0.280
DRAWER_FRONT_X = CABINET_DEPTH / 2.0 + 0.002

RUNNER_LENGTH = 0.345
RUNNER_WIDTH = 0.010
RUNNER_HEIGHT = 0.018
BODY_RUNNER_CENTER_X = -0.015
BODY_RUNNER_CENTER_Y = CAVITY_WIDTH / 2.0 - 0.004

DRAWER_SLIDE_LENGTH = 0.300
DRAWER_SLIDE_WIDTH = 0.008
DRAWER_SLIDE_HEIGHT = 0.016
DRAWER_SLIDE_CENTER_X = -0.190
DRAWER_SLIDE_CENTER_Y = DRAWER_BOX_WIDTH / 2.0 + DRAWER_SLIDE_WIDTH / 2.0

HANDLE_THICKNESS = 0.014
HANDLE_HEIGHT = 0.020
HANDLE_WIDTH_RATIO = 0.720

CASTER_AXLE_Z = -0.055
CASTER_PLATE_DEPTH = 0.060
CASTER_PLATE_WIDTH = 0.050
CASTER_PLATE_THICKNESS = 0.006
CASTER_ARM_DEPTH = 0.018
CASTER_ARM_THICKNESS = 0.006
CASTER_ARM_HEIGHT = 0.058
CASTER_ARM_CENTER_Y = 0.017
CASTER_ARM_CENTER_Z = 0.014
CASTER_BRIDGE_DEPTH = 0.022
CASTER_BRIDGE_WIDTH = 0.040
CASTER_BRIDGE_THICKNESS = 0.008
CASTER_BRIDGE_CENTER_Z = 0.043
CASTER_HOUSING_SIZE = 0.018
CASTER_HOUSING_THICKNESS = 0.008
CASTER_HOUSING_CENTER_Z = 0.050

WHEEL_RIM_RADIUS = 0.028
WHEEL_OUTER_RADIUS = 0.036
WHEEL_WIDTH = 0.024
WHEEL_AXLE_RADIUS = 0.0045
WHEEL_AXLE_LENGTH = 2.0 * (CASTER_ARM_CENTER_Y - CASTER_ARM_THICKNESS / 2.0)

CASTER_X_OFFSET = CABINET_DEPTH / 2.0 - 0.075
CASTER_Y_OFFSET = CABINET_WIDTH / 2.0 - 0.085

DRAWER_FACE_HEIGHTS = (0.182, 0.158, 0.142, 0.112, 0.112)
BOTTOM_REVEAL = 0.007
INTER_DRAWER_REVEAL = 0.005


def _drawer_specs() -> list[dict[str, float | str]]:
    specs: list[dict[str, float | str]] = []
    bottom = CAVITY_BOTTOM_Z + BOTTOM_REVEAL
    for index, face_height in enumerate(DRAWER_FACE_HEIGHTS):
        box_height = face_height - 0.030
        slide_center_z = min(box_height * 0.52, box_height - DRAWER_SLIDE_HEIGHT / 2.0 - 0.008)
        specs.append(
            {
                "name": f"drawer_{index}",
                "joint": f"body_to_drawer_{index}",
                "bottom": bottom,
                "face_height": face_height,
                "box_height": box_height,
                "slide_center_z": max(slide_center_z, DRAWER_SLIDE_HEIGHT / 2.0 + 0.010),
            }
        )
        bottom += face_height + INTER_DRAWER_REVEAL
    return specs


def _build_drawer_mesh(*, mesh_name: str, face_height: float, box_height: float, slide_center_z: float) -> object:
    tray_outer_center_x = -DRAWER_FACE_THICKNESS - DRAWER_BOX_DEPTH / 2.0
    tray = (
        cq.Workplane("XY")
        .box(DRAWER_BOX_DEPTH, DRAWER_BOX_WIDTH, box_height)
        .translate((tray_outer_center_x, 0.0, box_height / 2.0))
    )

    inner_height = box_height - DRAWER_BOTTOM_WALL + 0.012
    inner = (
        cq.Workplane("XY")
        .box(
            DRAWER_BOX_DEPTH - DRAWER_FRONT_WALL - DRAWER_REAR_WALL,
            DRAWER_BOX_WIDTH - 2.0 * DRAWER_SIDE_WALL,
            inner_height,
        )
        .translate((tray_outer_center_x, 0.0, DRAWER_BOTTOM_WALL + inner_height / 2.0))
    )
    tray = tray.cut(inner)

    face = (
        cq.Workplane("XY")
        .box(DRAWER_FACE_THICKNESS, DRAWER_FACE_WIDTH, face_height)
        .translate((-DRAWER_FACE_THICKNESS / 2.0, 0.0, face_height / 2.0))
    )

    drawer = tray.union(face)
    for side in (-1.0, 1.0):
        slide = (
            cq.Workplane("XY")
            .box(DRAWER_SLIDE_LENGTH, DRAWER_SLIDE_WIDTH, DRAWER_SLIDE_HEIGHT)
            .translate((DRAWER_SLIDE_CENTER_X, side * DRAWER_SLIDE_CENTER_Y, slide_center_z))
        )
        drawer = drawer.union(slide)

    return mesh_from_cadquery(drawer, mesh_name)


def _build_caster_bracket_mesh(mesh_name: str) -> object:
    plate_center_z = -CASTER_AXLE_Z - CASTER_PLATE_THICKNESS / 2.0
    bracket = (
        cq.Workplane("XY")
        .box(CASTER_PLATE_DEPTH, CASTER_PLATE_WIDTH, CASTER_PLATE_THICKNESS)
        .translate((0.0, 0.0, plate_center_z))
    )
    housing = (
        cq.Workplane("XY")
        .box(CASTER_HOUSING_SIZE, CASTER_HOUSING_SIZE, CASTER_HOUSING_THICKNESS)
        .translate((0.0, 0.0, CASTER_HOUSING_CENTER_Z))
    )
    bridge = (
        cq.Workplane("XY")
        .box(CASTER_BRIDGE_DEPTH, CASTER_BRIDGE_WIDTH, CASTER_BRIDGE_THICKNESS)
        .translate((0.0, 0.0, CASTER_BRIDGE_CENTER_Z))
    )
    bracket = bracket.union(housing).union(bridge)

    for side in (-1.0, 1.0):
        arm = (
            cq.Workplane("XY")
            .box(CASTER_ARM_DEPTH, CASTER_ARM_THICKNESS, CASTER_ARM_HEIGHT)
            .translate((0.0, side * CASTER_ARM_CENTER_Y, CASTER_ARM_CENTER_Z))
        )
        bracket = bracket.union(arm)

    return mesh_from_cadquery(bracket, mesh_name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rolling_tool_drawer_cabinet")

    body_red = model.material("body_red", rgba=(0.72, 0.11, 0.13, 1.0))
    matte_black = model.material("matte_black", rgba=(0.11, 0.11, 0.12, 1.0))
    wheel_steel = model.material("wheel_steel", rgba=(0.71, 0.73, 0.77, 1.0))
    tire_black = model.material("tire_black", rgba=(0.09, 0.09, 0.10, 1.0))

    body = model.part("body")
    body.visual(
        Box((BACK_WALL, CABINET_WIDTH, CABINET_HEIGHT)),
        origin=Origin(xyz=(-CABINET_DEPTH / 2.0 + BACK_WALL / 2.0, 0.0, CABINET_HEIGHT / 2.0)),
        material=body_red,
        name="back",
    )
    body.visual(
        Box((CABINET_DEPTH, SIDE_FRAME, CABINET_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                CABINET_WIDTH / 2.0 - SIDE_FRAME / 2.0,
                CABINET_HEIGHT / 2.0,
            )
        ),
        material=body_red,
        name="side_0",
    )
    body.visual(
        Box((CABINET_DEPTH, SIDE_FRAME, CABINET_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -CABINET_WIDTH / 2.0 + SIDE_FRAME / 2.0,
                CABINET_HEIGHT / 2.0,
            )
        ),
        material=body_red,
        name="side_1",
    )
    body.visual(
        Box((CABINET_DEPTH, CABINET_WIDTH, BOTTOM_FRAME)),
        origin=Origin(xyz=(0.0, 0.0, BOTTOM_FRAME / 2.0)),
        material=body_red,
        name="bottom",
    )
    body.visual(
        Box((CABINET_DEPTH, CABINET_WIDTH, TOP_FRAME)),
        origin=Origin(xyz=(0.0, 0.0, CABINET_HEIGHT - TOP_FRAME / 2.0)),
        material=body_red,
        name="top",
    )
    for drawer_index, spec in enumerate(_drawer_specs()):
        runner_z = float(spec["bottom"]) + float(spec["slide_center_z"])
        for side_index, side in enumerate((-1.0, 1.0)):
            body.visual(
                Box((RUNNER_LENGTH, RUNNER_WIDTH, RUNNER_HEIGHT)),
                origin=Origin(xyz=(BODY_RUNNER_CENTER_X, side * BODY_RUNNER_CENTER_Y, runner_z)),
                material=body_red,
                name=f"runner_{drawer_index}_{side_index}",
            )
    body.visual(
        Box((CABINET_DEPTH - 0.020, CABINET_WIDTH - 0.020, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, CABINET_HEIGHT + 0.003)),
        material=matte_black,
        name="top_mat",
    )

    for spec in _drawer_specs():
        drawer = model.part(str(spec["name"]))
        drawer.visual(
            _build_drawer_mesh(
                mesh_name=f"{spec['name']}_shell",
                face_height=float(spec["face_height"]),
                box_height=float(spec["box_height"]),
                slide_center_z=float(spec["slide_center_z"]),
            ),
            material=body_red,
            name="shell",
        )
        drawer.visual(
            Box((HANDLE_THICKNESS, DRAWER_FACE_WIDTH * HANDLE_WIDTH_RATIO, HANDLE_HEIGHT)),
            origin=Origin(
                xyz=(
                    HANDLE_THICKNESS / 2.0 - 0.001,
                    0.0,
                    float(spec["face_height"]) * 0.670,
                )
            ),
            material=matte_black,
            name="handle",
        )

        model.articulation(
            str(spec["joint"]),
            ArticulationType.PRISMATIC,
            parent=body,
            child=drawer,
            origin=Origin(xyz=(DRAWER_FRONT_X, 0.0, float(spec["bottom"]))),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=180.0,
                velocity=0.300,
                lower=0.0,
                upper=DRAWER_TRAVEL,
            ),
        )

    caster_mesh = _build_caster_bracket_mesh("caster_bracket")
    rim_mesh = mesh_from_geometry(
        WheelGeometry(radius=WHEEL_RIM_RADIUS, width=WHEEL_WIDTH),
        "caster_rim",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(outer_radius=WHEEL_OUTER_RADIUS, width=WHEEL_WIDTH, inner_radius=WHEEL_RIM_RADIUS),
        "caster_tire",
    )

    caster_positions = (
        ("front_left", CASTER_X_OFFSET, CASTER_Y_OFFSET),
        ("front_right", CASTER_X_OFFSET, -CASTER_Y_OFFSET),
        ("rear_left", -CASTER_X_OFFSET, CASTER_Y_OFFSET),
        ("rear_right", -CASTER_X_OFFSET, -CASTER_Y_OFFSET),
    )
    for prefix, pos_x, pos_y in caster_positions:
        caster = model.part(f"{prefix}_caster")
        caster.visual(caster_mesh, material=matte_black, name="fork")
        model.articulation(
            f"body_to_{prefix}_caster",
            ArticulationType.FIXED,
            parent=body,
            child=caster,
            origin=Origin(xyz=(pos_x, pos_y, CASTER_AXLE_Z)),
        )

        wheel = model.part(f"{prefix}_wheel")
        wheel.visual(
            rim_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=wheel_steel,
            name="rim",
        )
        wheel.visual(
            tire_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=tire_black,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=WHEEL_AXLE_RADIUS, length=WHEEL_AXLE_LENGTH),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=wheel_steel,
            name="axle",
        )
        model.articulation(
            f"{prefix}_caster_to_wheel",
            ArticulationType.CONTINUOUS,
            parent=caster,
            child=wheel,
            origin=Origin(),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=12.0,
                velocity=20.0,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    drawer_specs = _drawer_specs()

    for spec in drawer_specs:
        drawer = object_model.get_part(str(spec["name"]))
        joint = object_model.get_articulation(str(spec["joint"]))
        limits = joint.motion_limits
        ctx.check(
            f"{spec['name']} uses prismatic travel",
            joint.articulation_type == ArticulationType.PRISMATIC
            and limits is not None
            and limits.lower == 0.0
            and limits.upper is not None
            and limits.upper >= 0.240,
            details=f"type={joint.articulation_type}, limits={limits}",
        )
        ctx.expect_within(
            drawer,
            body,
            axes="yz",
            margin=0.020,
            name=f"{spec['name']} stays within the cabinet silhouette",
        )

    top_drawer = object_model.get_part("drawer_4")
    top_joint = object_model.get_articulation("body_to_drawer_4")
    closed_position = ctx.part_world_position(top_drawer)
    with ctx.pose({top_joint: DRAWER_TRAVEL}):
        ctx.expect_overlap(
            top_drawer,
            body,
            axes="x",
            min_overlap=0.090,
            name="top drawer retains insertion when fully extended",
        )
        extended_position = ctx.part_world_position(top_drawer)

    ctx.check(
        "top drawer extends outward",
        closed_position is not None
        and extended_position is not None
        and extended_position[0] > closed_position[0] + 0.200,
        details=f"closed={closed_position}, extended={extended_position}",
    )

    for prefix in ("front_left", "front_right", "rear_left", "rear_right"):
        caster = object_model.get_part(f"{prefix}_caster")
        caster_joint = object_model.get_articulation(f"{prefix}_caster_to_wheel")
        wheel = object_model.get_part(f"{prefix}_wheel")
        limits = caster_joint.motion_limits
        ctx.allow_overlap(
            caster,
            wheel,
            elem_a="fork",
            elem_b="axle",
            reason="The axle shaft is simplified as passing through the caster fork without modeled bearing holes.",
        )
        ctx.check(
            f"{prefix} wheel spins continuously",
            caster_joint.articulation_type == ArticulationType.CONTINUOUS
            and limits is not None
            and limits.lower is None
            and limits.upper is None
            and caster_joint.axis == (0.0, 1.0, 0.0),
            details=f"type={caster_joint.articulation_type}, axis={caster_joint.axis}, limits={limits}",
        )
        ctx.expect_origin_gap(
            body,
            wheel,
            axis="z",
            min_gap=0.045,
            name=f"{prefix} wheel sits below the cabinet body",
        )

    return ctx.report()


object_model = build_object_model()
