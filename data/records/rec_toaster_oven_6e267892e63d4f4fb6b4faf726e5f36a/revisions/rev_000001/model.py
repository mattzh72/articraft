from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

WIDTH = 0.55
DEPTH = 0.40
HEIGHT = 0.34

OUTER_WALL = 0.015
BASE_THICK = 0.018
TOP_THICK = 0.016
BACK_THICK = 0.012
FRONT_FACE_THICK = 0.014

CAVITY_WIDTH = 0.395
CAVITY_HEIGHT = 0.235
CAVITY_DEPTH = 0.35
CAVITY_CENTER_X = -0.03
CAVITY_BOTTOM_Z = 0.055
CAVITY_CENTER_Z = CAVITY_BOTTOM_Z + CAVITY_HEIGHT / 2.0
CAVITY_CENTER_Y = DEPTH / 2.0 - FRONT_FACE_THICK - CAVITY_DEPTH / 2.0

CONTROL_COLUMN_WIDTH = 0.095
CONTROL_COLUMN_CENTER_X = WIDTH / 2.0 - CONTROL_COLUMN_WIDTH / 2.0

DOOR_WIDTH = 0.39
DOOR_HEIGHT = 0.248
DOOR_THICK = 0.018
DOOR_CENTER_X = CAVITY_CENTER_X
DOOR_HINGE_Y = DEPTH / 2.0 + 0.009
DOOR_HINGE_Z = 0.048

DIAL_RADIUS = 0.022
DIAL_DEPTH = 0.026
DIAL_PANEL_Y = DEPTH / 2.0

RUNNER_WIDTH = 0.012
RUNNER_HEIGHT = 0.008
RUNNER_LENGTH = 0.30
RUNNER_Z = CAVITY_BOTTOM_Z + 0.09

RACK_LENGTH = 0.34
RACK_TRAVEL = 0.10
RACK_REAR_ORIGIN_Y = -0.160
RACK_SIDE_X = CAVITY_WIDTH / 2.0 - RUNNER_WIDTH / 2.0


def _add_rod(
    part,
    *,
    axis: str,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material: str,
    name: str,
) -> None:
    rotations = {
        "x": (0.0, pi / 2.0, 0.0),
        "y": (-pi / 2.0, 0.0, 0.0),
        "z": (0.0, 0.0, 0.0),
    }
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rotations[axis]),
        material=material,
        name=name,
    )


def _add_dial(
    model: ArticulatedObject,
    body,
    *,
    name: str,
    center_xyz: tuple[float, float, float],
    mesh_name: str,
    finish: str,
    shaft_finish: str,
) -> None:
    dial = model.part(name)
    dial.visual(
        Cylinder(radius=0.0045, length=0.012),
        origin=Origin(xyz=(0.0, 0.006, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=shaft_finish,
        name="shaft",
    )
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.044,
                DIAL_DEPTH,
                body_style="skirted",
                top_diameter=0.038,
                skirt=KnobSkirt(0.050, 0.006, flare=0.07),
                grip=KnobGrip(style="fluted", count=16, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008),
                center=False,
            ),
            mesh_name,
        ),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=finish,
        name="cap",
    )

    model.articulation(
        f"body_to_{name}",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=center_xyz),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=8.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="toaster_oven")

    body_finish = model.material("body_finish", rgba=(0.17, 0.18, 0.19, 1.0))
    trim_finish = model.material("trim_finish", rgba=(0.56, 0.58, 0.60, 1.0))
    glass_finish = model.material("glass_finish", rgba=(0.12, 0.15, 0.18, 0.55))
    interior_finish = model.material("interior_finish", rgba=(0.45, 0.46, 0.47, 1.0))
    foot_finish = model.material("foot_finish", rgba=(0.08, 0.08, 0.08, 1.0))
    knob_finish = model.material("knob_finish", rgba=(0.11, 0.11, 0.12, 1.0))
    shaft_finish = model.material("shaft_finish", rgba=(0.65, 0.66, 0.68, 1.0))
    rack_finish = model.material("rack_finish", rgba=(0.78, 0.79, 0.80, 1.0))

    body = model.part("body")
    body.visual(
        Box((WIDTH, DEPTH, BASE_THICK)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICK / 2.0)),
        material=body_finish,
        name="base_plate",
    )
    body.visual(
        Box((WIDTH, DEPTH, TOP_THICK)),
        origin=Origin(xyz=(0.0, 0.0, HEIGHT - TOP_THICK / 2.0)),
        material=body_finish,
        name="top_shell",
    )
    body.visual(
        Box((OUTER_WALL, DEPTH, HEIGHT)),
        origin=Origin(xyz=(-WIDTH / 2.0 + OUTER_WALL / 2.0, 0.0, HEIGHT / 2.0)),
        material=body_finish,
        name="left_shell",
    )
    body.visual(
        Box((OUTER_WALL, DEPTH, HEIGHT)),
        origin=Origin(xyz=(WIDTH / 2.0 - OUTER_WALL / 2.0, 0.0, HEIGHT / 2.0)),
        material=body_finish,
        name="right_shell",
    )
    body.visual(
        Box((WIDTH - 2.0 * OUTER_WALL, BACK_THICK, HEIGHT - TOP_THICK)),
        origin=Origin(
            xyz=(
                0.0,
                -DEPTH / 2.0 + BACK_THICK / 2.0,
                (HEIGHT - TOP_THICK) / 2.0,
            )
        ),
        material=body_finish,
        name="back_shell",
    )

    cavity_wall_thick = 0.008
    body.visual(
        Box((cavity_wall_thick, CAVITY_DEPTH, CAVITY_HEIGHT + 0.03)),
        origin=Origin(
            xyz=(
                CAVITY_CENTER_X - CAVITY_WIDTH / 2.0 - cavity_wall_thick / 2.0,
                CAVITY_CENTER_Y,
                CAVITY_CENTER_Z + 0.015,
            )
        ),
        material=interior_finish,
        name="left_liner",
    )
    body.visual(
        Box((cavity_wall_thick, CAVITY_DEPTH, CAVITY_HEIGHT + 0.03)),
        origin=Origin(
            xyz=(
                CAVITY_CENTER_X + CAVITY_WIDTH / 2.0 + cavity_wall_thick / 2.0,
                CAVITY_CENTER_Y,
                CAVITY_CENTER_Z + 0.015,
            )
        ),
        material=interior_finish,
        name="right_liner",
    )
    body.visual(
        Box((CAVITY_WIDTH + 2.0 * cavity_wall_thick, CAVITY_DEPTH, 0.01)),
        origin=Origin(xyz=(CAVITY_CENTER_X, CAVITY_CENTER_Y, CAVITY_BOTTOM_Z - 0.005)),
        material=interior_finish,
        name="cavity_floor",
    )

    left_jamb_width = 0.03
    body.visual(
        Box((left_jamb_width, FRONT_FACE_THICK, 0.26)),
        origin=Origin(
            xyz=(
                DOOR_CENTER_X - DOOR_WIDTH / 2.0 - left_jamb_width / 2.0,
                DEPTH / 2.0 - FRONT_FACE_THICK / 2.0,
                0.175,
            )
        ),
        material=trim_finish,
        name="left_jamb",
    )
    body.visual(
        Box((0.43, FRONT_FACE_THICK, 0.05)),
        origin=Origin(
            xyz=(
                -0.01,
                DEPTH / 2.0 - FRONT_FACE_THICK / 2.0,
                HEIGHT - 0.025,
            )
        ),
        material=trim_finish,
        name="top_header",
    )
    body.visual(
        Box((0.43, FRONT_FACE_THICK, 0.046)),
        origin=Origin(
            xyz=(
                -0.01,
                DEPTH / 2.0 - FRONT_FACE_THICK / 2.0,
                0.023,
            )
        ),
        material=trim_finish,
        name="lower_apron",
    )
    body.visual(
        Box((CONTROL_COLUMN_WIDTH, FRONT_FACE_THICK, HEIGHT - 0.04)),
        origin=Origin(
            xyz=(
                CONTROL_COLUMN_CENTER_X,
                DEPTH / 2.0 - FRONT_FACE_THICK / 2.0,
                HEIGHT / 2.0 - 0.005,
            )
        ),
        material=trim_finish,
        name="control_column",
    )

    body.visual(
        Box((RUNNER_WIDTH, RUNNER_LENGTH, RUNNER_HEIGHT)),
        origin=Origin(
            xyz=(
                CAVITY_CENTER_X - CAVITY_WIDTH / 2.0 + RUNNER_WIDTH / 2.0,
                CAVITY_CENTER_Y + 0.02,
                RUNNER_Z,
            )
        ),
        material=trim_finish,
        name="left_runner",
    )
    body.visual(
        Box((RUNNER_WIDTH, RUNNER_LENGTH, RUNNER_HEIGHT)),
        origin=Origin(
            xyz=(
                CAVITY_CENTER_X + CAVITY_WIDTH / 2.0 - RUNNER_WIDTH / 2.0,
                CAVITY_CENTER_Y + 0.02,
                RUNNER_Z,
            )
        ),
        material=trim_finish,
        name="right_runner",
    )

    heater_radius = 0.003
    _add_rod(
        body,
        axis="x",
        radius=heater_radius,
        length=CAVITY_WIDTH,
        xyz=(CAVITY_CENTER_X, CAVITY_CENTER_Y - 0.005, CAVITY_BOTTOM_Z + 0.035),
        material=trim_finish,
        name="lower_element",
    )
    _add_rod(
        body,
        axis="x",
        radius=heater_radius,
        length=CAVITY_WIDTH,
        xyz=(CAVITY_CENTER_X, CAVITY_CENTER_Y - 0.005, CAVITY_BOTTOM_Z + CAVITY_HEIGHT - 0.03),
        material=trim_finish,
        name="upper_element",
    )

    foot_size = (0.04, 0.03, 0.012)
    for index, x_pos in enumerate((-0.20, 0.20)):
        for depth_index, y_pos in enumerate((-0.14, 0.14)):
            body.visual(
                Box(foot_size),
                origin=Origin(xyz=(x_pos, y_pos, foot_size[2] / 2.0)),
                material=foot_finish,
                name=f"foot_{index}_{depth_index}",
            )

    door = model.part("door")
    frame_side = 0.03
    frame_top = 0.028
    frame_bottom = 0.032
    glass_width = DOOR_WIDTH - 2.0 * frame_side
    glass_height = DOOR_HEIGHT - frame_top - frame_bottom

    door.visual(
        Box((frame_side, DOOR_THICK, DOOR_HEIGHT)),
        origin=Origin(xyz=(-DOOR_WIDTH / 2.0 + frame_side / 2.0, 0.0, DOOR_HEIGHT / 2.0)),
        material=trim_finish,
        name="left_frame",
    )
    door.visual(
        Box((frame_side, DOOR_THICK, DOOR_HEIGHT)),
        origin=Origin(xyz=(DOOR_WIDTH / 2.0 - frame_side / 2.0, 0.0, DOOR_HEIGHT / 2.0)),
        material=trim_finish,
        name="right_frame",
    )
    door.visual(
        Box((DOOR_WIDTH - 2.0 * frame_side, DOOR_THICK, frame_bottom)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                frame_bottom / 2.0,
            )
        ),
        material=trim_finish,
        name="bottom_frame",
    )
    door.visual(
        Box((DOOR_WIDTH - 2.0 * frame_side, DOOR_THICK, frame_top)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                DOOR_HEIGHT - frame_top / 2.0,
            )
        ),
        material=trim_finish,
        name="top_frame",
    )
    door.visual(
        Box((glass_width, 0.006, glass_height)),
        origin=Origin(
            xyz=(
                0.0,
                -0.003,
                frame_bottom + glass_height / 2.0,
            )
        ),
        material=glass_finish,
        name="window",
    )
    door.visual(
        Box((0.22, 0.018, 0.016)),
        origin=Origin(
            xyz=(
                0.0,
                DOOR_THICK / 2.0 + 0.009,
                DOOR_HEIGHT - 0.036,
            )
        ),
        material=body_finish,
        name="handle",
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(DOOR_CENTER_X, DOOR_HINGE_Y, DOOR_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.35, effort=18.0, velocity=1.6),
    )

    _add_dial(
        model,
        body,
        name="dial_0",
        center_xyz=(CONTROL_COLUMN_CENTER_X, DIAL_PANEL_Y, 0.245),
        mesh_name="dial_0_cap",
        finish=knob_finish,
        shaft_finish=shaft_finish,
    )
    _add_dial(
        model,
        body,
        name="dial_1",
        center_xyz=(CONTROL_COLUMN_CENTER_X, DIAL_PANEL_Y, 0.165),
        mesh_name="dial_1_cap",
        finish=knob_finish,
        shaft_finish=shaft_finish,
    )

    rack = model.part("broil_rack")
    shoe_size = (RUNNER_WIDTH, 0.29, RUNNER_HEIGHT)
    rack.visual(
        Box(shoe_size),
        origin=Origin(xyz=(-RACK_SIDE_X, 0.145, 0.0)),
        material=rack_finish,
        name="left_shoe",
    )
    rack.visual(
        Box(shoe_size),
        origin=Origin(xyz=(RACK_SIDE_X, 0.145, 0.0)),
        material=rack_finish,
        name="right_shoe",
    )

    rail_radius = 0.003
    frame_z = 0.007
    _add_rod(
        rack,
        axis="y",
        radius=rail_radius,
        length=RACK_LENGTH,
        xyz=(-RACK_SIDE_X, RACK_LENGTH / 2.0, frame_z),
        material="rack_finish",
        name="left_rail",
    )
    _add_rod(
        rack,
        axis="y",
        radius=rail_radius,
        length=RACK_LENGTH,
        xyz=(RACK_SIDE_X, RACK_LENGTH / 2.0, frame_z),
        material="rack_finish",
        name="right_rail",
    )
    _add_rod(
        rack,
        axis="x",
        radius=rail_radius,
        length=2.0 * RACK_SIDE_X,
        xyz=(0.0, 0.0, frame_z),
        material="rack_finish",
        name="rear_bar",
    )
    _add_rod(
        rack,
        axis="x",
        radius=rail_radius,
        length=2.0 * RACK_SIDE_X,
        xyz=(0.0, RACK_LENGTH, frame_z),
        material="rack_finish",
        name="front_bar",
    )
    for index, y_pos in enumerate((0.05, 0.10, 0.15, 0.20, 0.25, 0.30)):
        _add_rod(
            rack,
            axis="x",
            radius=0.0025,
            length=2.0 * RACK_SIDE_X,
            xyz=(0.0, y_pos, frame_z + 0.0055),
            material="rack_finish",
            name=f"slat_{index}",
        )

    model.articulation(
        "body_to_broil_rack",
        ArticulationType.PRISMATIC,
        parent=body,
        child=rack,
        origin=Origin(
            xyz=(
                CAVITY_CENTER_X,
                RACK_REAR_ORIGIN_Y,
                RUNNER_Z + RUNNER_HEIGHT,
            )
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=RACK_TRAVEL, effort=12.0, velocity=0.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    rack = object_model.get_part("broil_rack")
    dial_0 = object_model.get_part("dial_0")
    dial_1 = object_model.get_part("dial_1")
    door_hinge = object_model.get_articulation("body_to_door")
    rack_slide = object_model.get_articulation("body_to_broil_rack")
    dial_joint_0 = object_model.get_articulation("body_to_dial_0")
    dial_joint_1 = object_model.get_articulation("body_to_dial_1")

    ctx.expect_gap(
        door,
        body,
        axis="y",
        positive_elem="window",
        negative_elem="top_header",
        min_gap=0.0,
        max_gap=0.03,
        name="door sits just proud of the front frame",
    )
    ctx.expect_overlap(
        door,
        body,
        axes="x",
        min_overlap=0.30,
        elem_a="window",
        elem_b="top_header",
        name="door window spans the cooking cavity width",
    )

    closed_aabb = ctx.part_element_world_aabb(door, elem="handle")
    with ctx.pose({door_hinge: 1.2}):
        opened_aabb = ctx.part_element_world_aabb(door, elem="handle")
        ctx.check(
            "door opens downward",
            closed_aabb is not None
            and opened_aabb is not None
            and opened_aabb[1][1] > closed_aabb[1][1] + 0.10
            and opened_aabb[0][2] < closed_aabb[0][2] - 0.14,
            details=f"closed={closed_aabb}, opened={opened_aabb}",
        )

    ctx.expect_gap(
        dial_0,
        body,
        axis="y",
        positive_elem="shaft",
        negative_elem="control_column",
        min_gap=0.0,
        max_gap=0.001,
        name="upper dial shaft seats on the control panel",
    )
    ctx.expect_gap(
        dial_1,
        body,
        axis="y",
        positive_elem="shaft",
        negative_elem="control_column",
        min_gap=0.0,
        max_gap=0.001,
        name="lower dial shaft seats on the control panel",
    )
    ctx.check(
        "dials use continuous rotation joints",
        dial_joint_0.motion_limits is not None
        and dial_joint_0.motion_limits.lower is None
        and dial_joint_0.motion_limits.upper is None
        and dial_joint_1.motion_limits is not None
        and dial_joint_1.motion_limits.lower is None
        and dial_joint_1.motion_limits.upper is None,
        details=f"upper={dial_joint_0.motion_limits}, lower={dial_joint_1.motion_limits}",
    )

    ctx.expect_gap(
        rack,
        body,
        axis="z",
        positive_elem="left_shoe",
        negative_elem="left_runner",
        min_gap=0.0,
        max_gap=0.001,
        name="rack left shoe rests on the left runner",
    )
    ctx.expect_gap(
        rack,
        body,
        axis="z",
        positive_elem="right_shoe",
        negative_elem="right_runner",
        min_gap=0.0,
        max_gap=0.001,
        name="rack right shoe rests on the right runner",
    )
    ctx.expect_overlap(
        rack,
        body,
        axes="y",
        elem_a="left_shoe",
        elem_b="left_runner",
        min_overlap=0.24,
        name="rack remains deeply inserted on the left runner at rest",
    )

    rack_rest = ctx.part_world_position(rack)
    with ctx.pose({door_hinge: 1.2, rack_slide: RACK_TRAVEL}):
        rack_extended = ctx.part_world_position(rack)
        ctx.expect_overlap(
            rack,
            body,
            axes="y",
            elem_a="left_shoe",
            elem_b="left_runner",
            min_overlap=0.18,
            name="rack keeps left-side runner engagement when extended",
        )
        ctx.expect_overlap(
            rack,
            body,
            axes="y",
            elem_a="right_shoe",
            elem_b="right_runner",
            min_overlap=0.18,
            name="rack keeps right-side runner engagement when extended",
        )
        ctx.check(
            "rack slides forward when extended",
            rack_rest is not None
            and rack_extended is not None
            and rack_extended[1] > rack_rest[1] + 0.08,
            details=f"rest={rack_rest}, extended={rack_extended}",
        )

    return ctx.report()


object_model = build_object_model()
