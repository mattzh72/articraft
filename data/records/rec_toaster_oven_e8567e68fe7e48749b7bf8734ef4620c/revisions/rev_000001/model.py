from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BODY_WIDTH = 0.50
BODY_DEPTH = 0.39
BODY_HEIGHT = 0.31
BODY_FRONT_Y = -BODY_DEPTH / 2.0
BODY_BACK_Y = BODY_DEPTH / 2.0

DOOR_WIDTH = 0.306
DOOR_HEIGHT = 0.184
DOOR_THICKNESS = 0.022
DOOR_CENTER_X = -0.063
DOOR_HINGE_Z = 0.054

CONTROL_FRAME_OUTER_WIDTH = 0.104
CONTROL_FRAME_OUTER_HEIGHT = 0.184
CONTROL_FRAME_CENTER_X = 0.168
CONTROL_FRAME_CENTER_Z = 0.154
CONTROL_RAIL_THICKNESS = 0.008
CONTROL_FRAME_DEPTH = 0.018
CONTROL_OPENING_WIDTH = CONTROL_FRAME_OUTER_WIDTH - 2.0 * CONTROL_RAIL_THICKNESS
CONTROL_OPENING_HEIGHT = CONTROL_FRAME_OUTER_HEIGHT - 2.0 * CONTROL_RAIL_THICKNESS
PANEL_WIDTH = 0.076
PANEL_HEIGHT = 0.156
PANEL_THICKNESS = 0.006
PANEL_CENTER_Y = BODY_FRONT_Y + 0.014


def _add_body_shell(model: ArticulatedObject, steel: Material, trim: Material) -> object:
    body = model.part("body")

    body.visual(
        Box((BODY_WIDTH, BODY_DEPTH - 0.024, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=steel,
        name="bottom_pan",
    )
    body.visual(
        Box((BODY_WIDTH, BODY_DEPTH - 0.020, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT - 0.014)),
        material=steel,
        name="top_shell",
    )
    body.visual(
        Box((0.012, BODY_DEPTH - 0.020, BODY_HEIGHT - 0.040)),
        origin=Origin(xyz=(-BODY_WIDTH / 2.0 + 0.006, 0.0, 0.155)),
        material=steel,
        name="left_wall",
    )
    body.visual(
        Box((0.012, BODY_DEPTH - 0.020, BODY_HEIGHT - 0.040)),
        origin=Origin(xyz=(BODY_WIDTH / 2.0 - 0.006, 0.0, 0.155)),
        material=steel,
        name="right_wall",
    )
    body.visual(
        Box((BODY_WIDTH - 0.024, 0.012, BODY_HEIGHT - 0.040)),
        origin=Origin(xyz=(0.0, BODY_BACK_Y - 0.016, 0.155)),
        material=steel,
        name="back_wall",
    )

    body.visual(
        Box((BODY_WIDTH, 0.020, 0.030)),
        origin=Origin(xyz=(0.0, BODY_FRONT_Y + 0.010, BODY_HEIGHT - 0.029)),
        material=steel,
        name="front_top_rail",
    )
    body.visual(
        Box((BODY_WIDTH, 0.030, 0.018)),
        origin=Origin(xyz=(0.0, BODY_FRONT_Y + 0.015, 0.045)),
        material=steel,
        name="front_bottom_rail",
    )
    body.visual(
        Box((0.030, 0.020, 0.210)),
        origin=Origin(xyz=(-0.235, BODY_FRONT_Y + 0.010, 0.158)),
        material=steel,
        name="front_left_post",
    )
    body.visual(
        Box((0.022, 0.020, 0.210)),
        origin=Origin(xyz=(0.105, BODY_FRONT_Y + 0.010, 0.158)),
        material=steel,
        name="front_center_post",
    )
    body.visual(
        Box((0.030, 0.020, 0.210)),
        origin=Origin(xyz=(0.235, BODY_FRONT_Y + 0.010, 0.158)),
        material=steel,
        name="front_right_post",
    )

    body.visual(
        Box((CONTROL_RAIL_THICKNESS, CONTROL_FRAME_DEPTH, CONTROL_FRAME_OUTER_HEIGHT)),
        origin=Origin(
            xyz=(
                CONTROL_FRAME_CENTER_X - CONTROL_FRAME_OUTER_WIDTH / 2.0 + CONTROL_RAIL_THICKNESS / 2.0,
                BODY_FRONT_Y + CONTROL_FRAME_DEPTH / 2.0,
                CONTROL_FRAME_CENTER_Z,
            )
        ),
        material=trim,
        name="panel_left_rail",
    )
    body.visual(
        Box((CONTROL_RAIL_THICKNESS, CONTROL_FRAME_DEPTH, CONTROL_FRAME_OUTER_HEIGHT)),
        origin=Origin(
            xyz=(
                CONTROL_FRAME_CENTER_X + CONTROL_FRAME_OUTER_WIDTH / 2.0 - CONTROL_RAIL_THICKNESS / 2.0,
                BODY_FRONT_Y + CONTROL_FRAME_DEPTH / 2.0,
                CONTROL_FRAME_CENTER_Z,
            )
        ),
        material=trim,
        name="panel_right_rail",
    )
    body.visual(
        Box((CONTROL_FRAME_OUTER_WIDTH, CONTROL_FRAME_DEPTH, CONTROL_RAIL_THICKNESS)),
        origin=Origin(
            xyz=(
                CONTROL_FRAME_CENTER_X,
                BODY_FRONT_Y + CONTROL_FRAME_DEPTH / 2.0,
                CONTROL_FRAME_CENTER_Z + CONTROL_FRAME_OUTER_HEIGHT / 2.0 - CONTROL_RAIL_THICKNESS / 2.0,
            )
        ),
        material=trim,
        name="panel_top_rail",
    )
    body.visual(
        Box((CONTROL_FRAME_OUTER_WIDTH, CONTROL_FRAME_DEPTH, CONTROL_RAIL_THICKNESS)),
        origin=Origin(
            xyz=(
                CONTROL_FRAME_CENTER_X,
                BODY_FRONT_Y + CONTROL_FRAME_DEPTH / 2.0,
                CONTROL_FRAME_CENTER_Z - CONTROL_FRAME_OUTER_HEIGHT / 2.0 + CONTROL_RAIL_THICKNESS / 2.0,
            )
        ),
        material=trim,
        name="panel_bottom_rail",
    )
    body.visual(
        Box((CONTROL_OPENING_WIDTH, 0.012, CONTROL_OPENING_HEIGHT)),
        origin=Origin(
            xyz=(
                CONTROL_FRAME_CENTER_X,
                BODY_FRONT_Y + 0.023,
                CONTROL_FRAME_CENTER_Z,
            )
        ),
        material=trim,
        name="panel_backer",
    )

    foot_x = BODY_WIDTH / 2.0 - 0.055
    foot_y = BODY_DEPTH / 2.0 - 0.060
    for side_x, side_y, name in (
        (-foot_x, -foot_y, "front_left_foot"),
        (foot_x, -foot_y, "front_right_foot"),
        (-foot_x, foot_y, "rear_left_foot"),
        (foot_x, foot_y, "rear_right_foot"),
    ):
        body.visual(
            Box((0.030, 0.030, 0.012)),
            origin=Origin(xyz=(side_x, side_y, 0.006)),
            material=trim,
            name=name,
        )

    body.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT / 2.0)),
    )
    return body


def _add_door(
    model: ArticulatedObject,
    body: object,
    steel: Material,
    glass: Material,
    handle: Material,
) -> object:
    door = model.part("door")
    stile_width = 0.040

    door.visual(
        Box((stile_width, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(-DOOR_WIDTH / 2.0 + stile_width / 2.0, 0.0, DOOR_HEIGHT / 2.0)),
        material=steel,
        name="left_stile",
    )
    door.visual(
        Box((stile_width, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(DOOR_WIDTH / 2.0 - stile_width / 2.0, 0.0, DOOR_HEIGHT / 2.0)),
        material=steel,
        name="right_stile",
    )
    door.visual(
        Box((DOOR_WIDTH, DOOR_THICKNESS, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=steel,
        name="bottom_rail",
    )
    door.visual(
        Box((DOOR_WIDTH, DOOR_THICKNESS, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, DOOR_HEIGHT - 0.015)),
        material=steel,
        name="top_rail",
    )
    door.visual(
        Box((0.226, 0.005, 0.108)),
        origin=Origin(xyz=(0.0, 0.003, 0.098)),
        material=glass,
        name="window_glass",
    )
    door.visual(
        Box((0.018, 0.034, 0.020)),
        origin=Origin(xyz=(-0.086, -0.022, 0.155)),
        material=handle,
        name="left_handle_post",
    )
    door.visual(
        Box((0.018, 0.034, 0.020)),
        origin=Origin(xyz=(0.086, -0.022, 0.155)),
        material=handle,
        name="right_handle_post",
    )
    door.visual(
        Box((0.190, 0.020, 0.020)),
        origin=Origin(xyz=(0.0, -0.040, 0.155)),
        material=handle,
        name="handle_bar",
    )
    door.inertial = Inertial.from_geometry(
        Box((DOOR_WIDTH, 0.060, DOOR_HEIGHT)),
        mass=1.4,
        origin=Origin(xyz=(0.0, -0.008, DOOR_HEIGHT / 2.0)),
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(DOOR_CENTER_X, BODY_FRONT_Y + 0.012, DOOR_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=0.0,
            upper=1.55,
        ),
    )
    return door


def _add_control_panel(
    model: ArticulatedObject,
    body: object,
    panel_finish: Material,
    trim: Material,
) -> object:
    panel = model.part("control_panel")
    panel.visual(
        Box((PANEL_WIDTH, PANEL_THICKNESS, PANEL_HEIGHT)),
        origin=Origin(),
        material=panel_finish,
        name="panel_face",
    )
    panel.inertial = Inertial.from_geometry(
        Box((PANEL_WIDTH, 0.018, PANEL_HEIGHT)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.006, 0.0)),
    )

    model.articulation(
        "body_to_control_panel",
        ArticulationType.FIXED,
        parent=body,
        child=panel,
        origin=Origin(xyz=(CONTROL_FRAME_CENTER_X, PANEL_CENTER_Y, CONTROL_FRAME_CENTER_Z)),
    )
    return panel


def _add_knob(
    model: ArticulatedObject,
    panel: object,
    knob_index: int,
    *,
    z_offset: float,
    knob_material: Material,
    accent: Material,
) -> object:
    knob = model.part(f"knob_{knob_index}")
    knob.visual(
        Cylinder(radius=0.022, length=0.003),
        origin=Origin(xyz=(0.0, -0.0015, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=accent,
        name="escutcheon",
    )
    knob.visual(
        Cylinder(radius=0.005, length=0.010),
        origin=Origin(xyz=(0.0, -0.005, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=accent,
        name="shaft",
    )
    knob.visual(
        Cylinder(radius=0.019, length=0.026),
        origin=Origin(xyz=(0.0, -0.023, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_material,
        name="knob_body",
    )
    knob.visual(
        Box((0.004, 0.006, 0.012)),
        origin=Origin(xyz=(0.0, -0.034, 0.012)),
        material=accent,
        name="pointer",
    )
    knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.019, length=0.039),
        mass=0.06,
        origin=Origin(xyz=(0.0, -0.020, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        f"control_panel_to_knob_{knob_index}",
        ArticulationType.REVOLUTE,
        parent=panel,
        child=knob,
        origin=Origin(xyz=(0.0, -PANEL_THICKNESS / 2.0, z_offset)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=4.0,
            lower=-2.45,
            upper=2.45,
        ),
    )
    return knob


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_toaster_oven")

    steel = model.material("steel", rgba=(0.74, 0.75, 0.77, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.14, 0.15, 0.16, 1.0))
    glass = model.material("glass", rgba=(0.18, 0.24, 0.30, 0.40))
    handle = model.material("handle", rgba=(0.10, 0.10, 0.11, 1.0))
    panel_finish = model.material("panel_finish", rgba=(0.25, 0.26, 0.28, 1.0))
    knob_material = model.material("knob_material", rgba=(0.11, 0.11, 0.12, 1.0))
    knob_accent = model.material("knob_accent", rgba=(0.82, 0.83, 0.84, 1.0))

    body = _add_body_shell(model, steel, dark_trim)
    _add_door(model, body, steel, glass, handle)
    panel = _add_control_panel(model, body, panel_finish, dark_trim)
    for knob_index, z_offset in enumerate((0.048, 0.0, -0.048), start=1):
        _add_knob(
            model,
            panel,
            knob_index,
            z_offset=z_offset,
            knob_material=knob_material,
            accent=knob_accent,
        )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    door = object_model.get_part("door")
    panel = object_model.get_part("control_panel")
    door_hinge = object_model.get_articulation("body_to_door")

    def _elem_aabb(part_name: str, elem_name: str):
        return ctx.part_element_world_aabb(part_name, elem=elem_name)

    panel_aabb = _elem_aabb("control_panel", "panel_face")
    left_rail_aabb = _elem_aabb("body", "panel_left_rail")
    right_rail_aabb = _elem_aabb("body", "panel_right_rail")
    top_rail_aabb = _elem_aabb("body", "panel_top_rail")
    bottom_rail_aabb = _elem_aabb("body", "panel_bottom_rail")

    if (
        panel_aabb is not None
        and left_rail_aabb is not None
        and right_rail_aabb is not None
        and top_rail_aabb is not None
        and bottom_rail_aabb is not None
    ):
        left_margin = panel_aabb[0][0] - left_rail_aabb[1][0]
        right_margin = right_rail_aabb[0][0] - panel_aabb[1][0]
        top_margin = top_rail_aabb[0][2] - panel_aabb[1][2]
        bottom_margin = panel_aabb[0][2] - bottom_rail_aabb[1][2]
        ctx.check(
            "control panel stays centered in its frame",
            min(left_margin, right_margin, top_margin, bottom_margin) > 0.004
            and abs(left_margin - right_margin) < 0.001
            and abs(top_margin - bottom_margin) < 0.001,
            details=(
                f"left={left_margin:.4f}, right={right_margin:.4f}, "
                f"top={top_margin:.4f}, bottom={bottom_margin:.4f}"
            ),
        )
    else:
        ctx.fail("control panel stays centered in its frame", "missing panel/frame AABB")

    ctx.expect_overlap(
        door,
        body,
        axes="xz",
        min_overlap=0.18,
        name="door covers the oven opening footprint",
    )

    door_aabb = ctx.part_world_aabb(door)
    top_rail = _elem_aabb("body", "front_top_rail")
    left_post = _elem_aabb("body", "front_left_post")
    center_post = _elem_aabb("body", "front_center_post")
    if door_aabb is not None and top_rail is not None and left_post is not None and center_post is not None:
        left_gap = door_aabb[0][0] - left_post[1][0]
        right_gap = center_post[0][0] - door_aabb[1][0]
        top_gap = top_rail[0][2] - door_aabb[1][2]
        ctx.check(
            "door fits between the front frame posts",
            left_gap > 0.002 and right_gap > 0.002 and top_gap > 0.010,
            details=f"left_gap={left_gap:.4f}, right_gap={right_gap:.4f}, top_gap={top_gap:.4f}",
        )
    else:
        ctx.fail("door fits between the front frame posts", "missing door/frame AABB")

    closed_door_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.3}):
        opened_door_aabb = ctx.part_world_aabb(door)
    if closed_door_aabb is not None and opened_door_aabb is not None:
        ctx.check(
            "door opens downward and outward",
            opened_door_aabb[0][1] < closed_door_aabb[0][1] - 0.10
            and opened_door_aabb[1][2] < closed_door_aabb[1][2] - 0.10,
            details=f"closed={closed_door_aabb}, opened={opened_door_aabb}",
        )
    else:
        ctx.fail("door opens downward and outward", "missing door AABB for pose comparison")

    for knob_index in (1, 2, 3):
        knob = object_model.get_part(f"knob_{knob_index}")
        knob_joint = object_model.get_articulation(f"control_panel_to_knob_{knob_index}")
        ctx.expect_gap(
            panel,
            knob,
            axis="y",
            positive_elem="panel_face",
            max_gap=0.001,
            max_penetration=0.0,
            name=f"knob {knob_index} is shaft-mounted against the panel",
        )

        rest_pointer_aabb = ctx.part_element_world_aabb(knob, elem="pointer")
        with ctx.pose({knob_joint: 1.2}):
            turned_pointer_aabb = ctx.part_element_world_aabb(knob, elem="pointer")
        if rest_pointer_aabb is not None and turned_pointer_aabb is not None:
            rest_center = (
                (rest_pointer_aabb[0][0] + rest_pointer_aabb[1][0]) * 0.5,
                (rest_pointer_aabb[0][2] + rest_pointer_aabb[1][2]) * 0.5,
            )
            turned_center = (
                (turned_pointer_aabb[0][0] + turned_pointer_aabb[1][0]) * 0.5,
                (turned_pointer_aabb[0][2] + turned_pointer_aabb[1][2]) * 0.5,
            )
            travel = abs(turned_center[0] - rest_center[0]) + abs(turned_center[1] - rest_center[1])
            ctx.check(
                f"knob {knob_index} visibly rotates with its joint",
                travel > 0.010,
                details=f"rest={rest_center}, turned={turned_center}, travel={travel:.4f}",
            )
        else:
            ctx.fail(f"knob {knob_index} visibly rotates with its joint", "missing pointer AABB")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
