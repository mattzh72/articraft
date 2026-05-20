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
    mesh_from_cadquery,
)

HOUSING_DEPTH = 0.370
HOUSING_WIDTH = 0.310
HOUSING_HEIGHT = 0.360
HOUSING_WALL = 0.012

OPENING_Z = 0.115
DRAWER_JOINT_X = 0.167

CHAMBER_DEPTH = 0.252
CHAMBER_WIDTH = 0.252
CHAMBER_HEIGHT = 0.138
CHAMBER_WALL = 0.004

DRAWER_DEPTH = 0.236
DRAWER_WIDTH = 0.236
DRAWER_HEIGHT = 0.126
DRAWER_WALL = 0.0045
DRAWER_FRONT_THICKNESS = 0.018

BASKET_DEPTH = 0.214
BASKET_WIDTH = 0.214
BASKET_HEIGHT = 0.094
BASKET_WALL = 0.003

DRAWER_TRAVEL = 0.118

PANEL_PITCH = 0.42
PANEL_CENTER = (0.066, 0.0, 0.311)
PANEL_SIZE = (0.172, 0.228, 0.018)


def _rot_y(local_xyz: tuple[float, float, float], angle: float) -> tuple[float, float, float]:
    x, y, z = local_xyz
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    return (x * cos_a + z * sin_a, y, -x * sin_a + z * cos_a)


def _panel_world(local_xyz: tuple[float, float, float]) -> tuple[float, float, float]:
    rx, ry, rz = _rot_y(local_xyz, PANEL_PITCH)
    return (PANEL_CENTER[0] + rx, PANEL_CENTER[1] + ry, PANEL_CENTER[2] + rz)


def _build_housing_shell() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(HOUSING_DEPTH, HOUSING_WIDTH, HOUSING_HEIGHT)
        .translate((0.0, 0.0, HOUSING_HEIGHT * 0.5))
        .edges("|Z")
        .fillet(0.042)
        .edges(">Z")
        .fillet(0.016)
    )
    inner = (
        cq.Workplane("XY")
        .box(
            HOUSING_DEPTH - 2.0 * HOUSING_WALL,
            HOUSING_WIDTH - 2.0 * HOUSING_WALL,
            HOUSING_HEIGHT - 0.028,
        )
        .translate((-0.008, 0.0, HOUSING_HEIGHT * 0.5 + 0.006))
    )
    shell = outer.cut(inner)

    opening = (
        cq.Workplane("XY")
        .box(0.090, 0.254, 0.148)
        .translate((HOUSING_DEPTH * 0.5 - 0.045, 0.0, OPENING_Z))
    )
    shell = shell.cut(opening)

    for slot_y in (-0.090, -0.060, -0.030, 0.030, 0.060, 0.090):
        vent_slot = (
            cq.Workplane("XY")
            .box(0.082, 0.007, 0.016)
            .translate((-0.118, slot_y, 0.268))
        )
        shell = shell.cut(vent_slot)

    return shell


def _build_control_panel() -> cq.Workplane:
    panel = (
        cq.Workplane("XY")
        .box(*PANEL_SIZE)
        .edges("|Z")
        .fillet(0.010)
    )

    for local_y in (-0.060, 0.060):
        dial_well = (
            cq.Workplane("XY")
            .box(0.062, 0.062, 0.010)
            .translate((0.022, local_y, 0.004))
        )
        panel = panel.cut(dial_well)

    for local_y in (-0.024, 0.024):
        pocket = (
            cq.Workplane("XY")
            .box(0.032, 0.022, 0.012)
            .translate((-0.042, local_y, 0.003))
        )
        panel = panel.cut(pocket)

    return panel


def _build_basket_floor() -> cq.Workplane:
    floor = (
        cq.Workplane("XY")
        .box(BASKET_DEPTH - 0.006, BASKET_WIDTH - 0.006, BASKET_WALL)
        .translate((-BASKET_DEPTH * 0.5, 0.0, -BASKET_HEIGHT * 0.5 + BASKET_WALL * 0.5))
    )
    for hole_x in (-0.178, -0.146, -0.114, -0.082, -0.050):
        for hole_y in (-0.072, -0.036, 0.0, 0.036, 0.072):
            hole = (
                cq.Workplane("XY")
                .box(0.012, 0.012, 0.010)
                .translate((hole_x, hole_y, -BASKET_HEIGHT * 0.5 + BASKET_WALL * 0.5))
            )
            floor = floor.cut(hole)
    return floor


def _build_basket_side(sign: float) -> cq.Workplane:
    wall = (
        cq.Workplane("XY")
        .box(BASKET_DEPTH - 0.010, BASKET_WALL, BASKET_HEIGHT - 2.0 * BASKET_WALL)
        .translate((-BASKET_DEPTH * 0.5, sign * (BASKET_WIDTH * 0.5 - BASKET_WALL * 0.5), 0.0))
    )
    for slot_x in (-0.166, -0.130, -0.094, -0.058):
        for slot_z in (-0.014, 0.012):
            slot = (
                cq.Workplane("XY")
                .box(0.020, 0.010, 0.012)
                .translate((slot_x, sign * (BASKET_WIDTH * 0.5 - BASKET_WALL * 0.5), slot_z))
            )
            wall = wall.cut(slot)
    return wall


def _build_basket_rear() -> cq.Workplane:
    wall = (
        cq.Workplane("XY")
        .box(BASKET_WALL, BASKET_WIDTH - 0.014, BASKET_HEIGHT - 2.0 * BASKET_WALL)
        .translate((-BASKET_DEPTH + BASKET_WALL * 0.5, 0.0, 0.0))
    )
    for slot_y in (-0.070, -0.035, 0.0, 0.035, 0.070):
        slot = (
            cq.Workplane("XY")
            .box(0.010, 0.018, 0.012)
            .translate((-BASKET_DEPTH + BASKET_WALL * 0.5, slot_y, -0.004))
        )
        wall = wall.cut(slot)
    return wall


def _build_basket_handle() -> cq.Workplane:
    arm_left = (
        cq.Workplane("XY")
        .box(0.044, 0.018, 0.010)
        .translate((0.018, 0.028, 0.006))
    )
    arm_right = (
        cq.Workplane("XY")
        .box(0.044, 0.018, 0.010)
        .translate((0.018, -0.028, 0.006))
    )
    body = (
        cq.Workplane("XY")
        .box(0.052, 0.084, 0.018)
        .translate((0.058, 0.0, 0.010))
    )
    button_recess = (
        cq.Workplane("XY")
        .box(0.026, 0.040, 0.014)
        .translate((0.058, 0.0, 0.012))
    )
    return arm_left.union(arm_right).union(body.cut(button_recess))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="basket_air_fryer")

    housing_plastic = model.material("housing_plastic", rgba=(0.12, 0.12, 0.13, 1.0))
    panel_black = model.material("panel_black", rgba=(0.06, 0.06, 0.07, 1.0))
    drawer_plastic = model.material("drawer_plastic", rgba=(0.15, 0.15, 0.16, 1.0))
    basket_metal = model.material("basket_metal", rgba=(0.34, 0.36, 0.38, 1.0))
    chamber_metal = model.material("chamber_metal", rgba=(0.58, 0.59, 0.60, 1.0))
    rubber_dark = model.material("rubber_dark", rgba=(0.07, 0.07, 0.08, 1.0))
    button_grey = model.material("button_grey", rgba=(0.82, 0.83, 0.84, 1.0))
    safety_red = model.material("safety_red", rgba=(0.74, 0.16, 0.12, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_build_housing_shell(), "housing_shell"),
        material=housing_plastic,
        name="body_shell",
    )
    housing.visual(
        mesh_from_cadquery(_build_control_panel(), "control_panel"),
        origin=Origin(xyz=PANEL_CENTER, rpy=(0.0, PANEL_PITCH, 0.0)),
        material=panel_black,
        name="control_panel",
    )
    for index, (foot_x, foot_y) in enumerate(
        (
            (-0.125, -0.105),
            (-0.125, 0.105),
            (0.090, -0.105),
            (0.090, 0.105),
        )
    ):
        housing.visual(
            Cylinder(radius=0.012, length=0.010),
            origin=Origin(
                xyz=(foot_x, foot_y, 0.005),
                rpy=(math.pi * 0.5, 0.0, 0.0),
            ),
            material=rubber_dark,
            name=f"foot_{index}",
        )

    chamber = model.part("chamber")
    chamber.visual(
        Box((CHAMBER_DEPTH, CHAMBER_WIDTH, CHAMBER_WALL)),
        origin=Origin(xyz=(-CHAMBER_DEPTH * 0.5, 0.0, -CHAMBER_HEIGHT * 0.5 + CHAMBER_WALL * 0.5)),
        material=chamber_metal,
        name="chamber_floor",
    )
    chamber.visual(
        Box((CHAMBER_DEPTH, CHAMBER_WALL, CHAMBER_HEIGHT)),
        origin=Origin(xyz=(-CHAMBER_DEPTH * 0.5, CHAMBER_WIDTH * 0.5 - CHAMBER_WALL * 0.5, 0.0)),
        material=chamber_metal,
        name="chamber_side_0",
    )
    chamber.visual(
        Box((CHAMBER_DEPTH, CHAMBER_WALL, CHAMBER_HEIGHT)),
        origin=Origin(xyz=(-CHAMBER_DEPTH * 0.5, -CHAMBER_WIDTH * 0.5 + CHAMBER_WALL * 0.5, 0.0)),
        material=chamber_metal,
        name="chamber_side_1",
    )
    chamber.visual(
        Box((CHAMBER_WALL, CHAMBER_WIDTH, CHAMBER_HEIGHT)),
        origin=Origin(xyz=(-CHAMBER_DEPTH + CHAMBER_WALL * 0.5, 0.0, 0.0)),
        material=chamber_metal,
        name="chamber_rear",
    )
    chamber.visual(
        Box((CHAMBER_DEPTH, CHAMBER_WIDTH, CHAMBER_WALL)),
        origin=Origin(xyz=(-CHAMBER_DEPTH * 0.5, 0.0, CHAMBER_HEIGHT * 0.5 - CHAMBER_WALL * 0.5)),
        material=chamber_metal,
        name="chamber_ceiling",
    )
    model.articulation(
        "housing_to_chamber",
        ArticulationType.FIXED,
        parent=housing,
        child=chamber,
        origin=Origin(xyz=(DRAWER_JOINT_X, 0.0, OPENING_Z)),
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((DRAWER_DEPTH, DRAWER_WIDTH, DRAWER_WALL)),
        origin=Origin(xyz=(-DRAWER_DEPTH * 0.5, 0.0, -DRAWER_HEIGHT * 0.5 + DRAWER_WALL * 0.5)),
        material=drawer_plastic,
        name="drawer_floor",
    )
    drawer.visual(
        Box((DRAWER_DEPTH, DRAWER_WALL, DRAWER_HEIGHT)),
        origin=Origin(xyz=(-DRAWER_DEPTH * 0.5, DRAWER_WIDTH * 0.5 - DRAWER_WALL * 0.5, 0.0)),
        material=drawer_plastic,
        name="drawer_side_0",
    )
    drawer.visual(
        Box((DRAWER_DEPTH, DRAWER_WALL, DRAWER_HEIGHT)),
        origin=Origin(xyz=(-DRAWER_DEPTH * 0.5, -DRAWER_WIDTH * 0.5 + DRAWER_WALL * 0.5, 0.0)),
        material=drawer_plastic,
        name="drawer_side_1",
    )
    drawer.visual(
        Box((DRAWER_WALL, DRAWER_WIDTH, DRAWER_HEIGHT)),
        origin=Origin(xyz=(-DRAWER_DEPTH + DRAWER_WALL * 0.5, 0.0, 0.0)),
        material=drawer_plastic,
        name="drawer_rear",
    )
    drawer.visual(
        Box((DRAWER_WALL, DRAWER_WIDTH, 0.070)),
        origin=Origin(xyz=(-DRAWER_WALL * 0.5, 0.0, -0.028)),
        material=drawer_plastic,
        name="drawer_front_lower",
    )
    drawer.visual(
        Box((DRAWER_WALL, 0.076, 0.048)),
        origin=Origin(xyz=(-DRAWER_WALL * 0.5, -0.090, 0.016)),
        material=drawer_plastic,
        name="drawer_front_cheek_0",
    )
    drawer.visual(
        Box((DRAWER_WALL, 0.076, 0.048)),
        origin=Origin(xyz=(-DRAWER_WALL * 0.5, 0.090, 0.016)),
        material=drawer_plastic,
        name="drawer_front_cheek_1",
    )
    drawer.visual(
        Box((DRAWER_FRONT_THICKNESS, 0.258, 0.074)),
        origin=Origin(xyz=(0.029, 0.0, -0.038)),
        material=drawer_plastic,
        name="drawer_front_lower_panel",
    )
    drawer.visual(
        Box((DRAWER_FRONT_THICKNESS, 0.258, 0.050)),
        origin=Origin(xyz=(0.029, 0.0, 0.050)),
        material=drawer_plastic,
        name="drawer_front_upper_panel",
    )
    drawer.visual(
        Box((DRAWER_FRONT_THICKNESS, 0.077, 0.048)),
        origin=Origin(xyz=(0.029, -0.0905, 0.016)),
        material=drawer_plastic,
        name="drawer_front_panel_0",
    )
    drawer.visual(
        Box((DRAWER_FRONT_THICKNESS, 0.077, 0.048)),
        origin=Origin(xyz=(0.029, 0.0905, 0.016)),
        material=drawer_plastic,
        name="drawer_front_panel_1",
    )
    drawer.visual(
        Box((0.024, 0.140, 0.036)),
        origin=Origin(xyz=(0.010, 0.0, -0.034)),
        material=drawer_plastic,
        name="drawer_front_mount_0",
    )
    drawer.visual(
        Box((0.024, 0.086, 0.024)),
        origin=Origin(xyz=(0.010, -0.0905, 0.016)),
        material=drawer_plastic,
        name="drawer_front_mount_1",
    )
    drawer.visual(
        Box((0.024, 0.086, 0.024)),
        origin=Origin(xyz=(0.010, 0.0905, 0.016)),
        material=drawer_plastic,
        name="drawer_front_mount_2",
    )
    drawer.visual(
        Box((0.030, 0.018, 0.032)),
        origin=Origin(xyz=(0.050, -0.055, -0.020)),
        material=drawer_plastic,
        name="handle_post_0",
    )
    drawer.visual(
        Box((0.030, 0.018, 0.032)),
        origin=Origin(xyz=(0.050, 0.055, -0.020)),
        material=drawer_plastic,
        name="handle_post_1",
    )
    drawer.visual(
        Box((0.034, 0.100, 0.012)),
        origin=Origin(xyz=(0.056, 0.0, -0.008)),
        material=drawer_plastic,
        name="handle_bridge",
    )
    drawer_joint = model.articulation(
        "housing_to_drawer",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=drawer,
        origin=Origin(xyz=(DRAWER_JOINT_X, 0.0, OPENING_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.35,
            lower=0.0,
            upper=DRAWER_TRAVEL,
        ),
    )

    basket = model.part("basket")
    basket.visual(
        mesh_from_cadquery(_build_basket_floor(), "basket_floor"),
        material=basket_metal,
        name="basket_floor",
    )
    basket.visual(
        mesh_from_cadquery(_build_basket_side(1.0), "basket_side_0"),
        material=basket_metal,
        name="basket_side_0",
    )
    basket.visual(
        mesh_from_cadquery(_build_basket_side(-1.0), "basket_side_1"),
        material=basket_metal,
        name="basket_side_1",
    )
    basket.visual(
        mesh_from_cadquery(_build_basket_rear(), "basket_rear"),
        material=basket_metal,
        name="basket_rear",
    )
    basket.visual(
        Box((BASKET_WALL, BASKET_WIDTH, 0.078)),
        origin=Origin(xyz=(-BASKET_WALL * 0.5, 0.0, -0.005)),
        material=basket_metal,
        name="basket_front",
    )
    for index, (foot_x, foot_y) in enumerate(
        (
            (-0.174, -0.074),
            (-0.174, 0.074),
            (-0.058, -0.074),
            (-0.058, 0.074),
        )
    ):
        basket.visual(
            Box((0.014, 0.014, 0.016)),
            origin=Origin(xyz=(foot_x, foot_y, -BASKET_HEIGHT * 0.5 - 0.008)),
            material=basket_metal,
            name=f"basket_foot_{index}",
        )
    basket.visual(
        mesh_from_cadquery(_build_basket_handle(), "basket_handle"),
        material=drawer_plastic,
        name="basket_handle",
    )
    model.articulation(
        "drawer_to_basket",
        ArticulationType.FIXED,
        parent=drawer,
        child=basket,
        origin=Origin(),
    )

    release_button = model.part("release_button")
    release_button.visual(
        Box((0.022, 0.034, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=safety_red,
        name="release_button",
    )
    model.articulation(
        "basket_to_release_button",
        ArticulationType.PRISMATIC,
        parent=basket,
        child=release_button,
        origin=Origin(xyz=(0.058, 0.0, 0.005)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=0.08,
            lower=0.0,
            upper=0.004,
        ),
    )

    for dial_name, local_y in (("temp_dial", -0.060), ("time_dial", 0.060)):
        dial = model.part(dial_name)
        dial.visual(
            Cylinder(radius=0.028, length=0.010),
            origin=Origin(xyz=(0.0, 0.0, 0.005)),
            material=button_grey,
            name="dial_skirt",
        )
        dial.visual(
            Cylinder(radius=0.022, length=0.016),
            origin=Origin(xyz=(0.0, 0.0, 0.013)),
            material=drawer_plastic,
            name="dial_cap",
        )
        dial.visual(
            Cylinder(radius=0.010, length=0.010),
            origin=Origin(xyz=(0.0, 0.0, -0.005)),
            material=drawer_plastic,
            name="dial_hub",
        )
        dial.visual(
            Box((0.014, 0.004, 0.004)),
            origin=Origin(xyz=(0.012, 0.0, 0.020)),
            material=safety_red if dial_name == "temp_dial" else panel_black,
            name="dial_indicator",
        )
        model.articulation(
            f"housing_to_{dial_name}",
            ArticulationType.CONTINUOUS,
            parent=housing,
            child=dial,
            origin=Origin(
                xyz=_panel_world((0.022, local_y, PANEL_SIZE[2] * 0.5 - 0.003)),
                rpy=(0.0, PANEL_PITCH, 0.0),
            ),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=0.12,
                velocity=6.0,
            ),
        )

    for button_name, local_y in (("preset_button_0", -0.024), ("preset_button_1", 0.024)):
        button = model.part(button_name)
        button.visual(
            Box((0.026, 0.018, 0.008)),
            origin=Origin(xyz=(0.0, 0.0, 0.004)),
            material=button_grey,
            name="button_cap",
        )
        button.visual(
            Box((0.016, 0.010, 0.006)),
            origin=Origin(xyz=(0.0, 0.0, -0.003)),
            material=button_grey,
            name="button_stem",
        )
        model.articulation(
            f"housing_to_{button_name}",
            ArticulationType.PRISMATIC,
            parent=housing,
            child=button,
            origin=Origin(
                xyz=_panel_world((-0.042, local_y, PANEL_SIZE[2] * 0.5 - 0.006)),
                rpy=(0.0, PANEL_PITCH, 0.0),
            ),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=5.0,
                velocity=0.08,
                lower=0.0,
                upper=0.004,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    chamber = object_model.get_part("chamber")
    drawer = object_model.get_part("drawer")
    basket = object_model.get_part("basket")
    release_button = object_model.get_part("release_button")
    temp_dial = object_model.get_part("temp_dial")
    time_dial = object_model.get_part("time_dial")
    preset_button_0 = object_model.get_part("preset_button_0")
    preset_button_1 = object_model.get_part("preset_button_1")

    drawer_joint = object_model.get_articulation("housing_to_drawer")
    release_joint = object_model.get_articulation("basket_to_release_button")
    temp_joint = object_model.get_articulation("housing_to_temp_dial")
    time_joint = object_model.get_articulation("housing_to_time_dial")
    preset_joint_0 = object_model.get_articulation("housing_to_preset_button_0")
    preset_joint_1 = object_model.get_articulation("housing_to_preset_button_1")

    ctx.allow_overlap(
        basket,
        release_button,
        elem_a="basket_handle",
        elem_b="release_button",
        reason="The safety latch button is intentionally seated within the basket handle recess.",
    )
    ctx.allow_overlap(
        drawer,
        housing,
        elem_a="drawer_front_mount_1",
        elem_b="body_shell",
        reason="The drawer fascia uses hidden side mounting tongues inside the housing front aperture, while the rounded housing shell remains a simplified outer cover.",
    )
    ctx.allow_overlap(
        drawer,
        housing,
        elem_a="drawer_front_mount_2",
        elem_b="body_shell",
        reason="The drawer fascia uses hidden side mounting tongues inside the housing front aperture, while the rounded housing shell remains a simplified outer cover.",
    )
    ctx.allow_overlap(
        "preset_button_0",
        housing,
        elem_a="button_stem",
        elem_b="control_panel",
        reason="The preset button plunger is intentionally guided inside the control-panel pocket.",
    )
    ctx.allow_overlap(
        "preset_button_1",
        housing,
        elem_a="button_stem",
        elem_b="control_panel",
        reason="The preset button plunger is intentionally guided inside the control-panel pocket.",
    )
    ctx.allow_overlap(
        "temp_dial",
        housing,
        elem_a="dial_hub",
        elem_b="control_panel",
        reason="The temperature dial uses a mounting hub that intentionally seats into the control panel bore.",
    )
    ctx.allow_overlap(
        "time_dial",
        housing,
        elem_a="dial_hub",
        elem_b="control_panel",
        reason="The time dial uses a mounting hub that intentionally seats into the control panel bore.",
    )

    ctx.expect_within(
        drawer,
        chamber,
        axes="yz",
        inner_elem="drawer_floor",
        margin=0.002,
        name="drawer shell stays centered in chamber opening",
    )
    ctx.expect_overlap(
        drawer,
        chamber,
        axes="x",
        elem_a="drawer_floor",
        min_overlap=0.16,
        name="closed drawer remains deeply inserted in chamber",
    )
    ctx.expect_within(
        basket,
        drawer,
        axes="yz",
        inner_elem="basket_floor",
        margin=0.003,
        name="basket shell nests inside drawer shell",
    )
    ctx.expect_overlap(
        basket,
        drawer,
        axes="x",
        elem_a="basket_floor",
        elem_b="drawer_floor",
        min_overlap=0.18,
        name="basket shell remains nested in drawer shell",
    )

    drawer_rest = ctx.part_world_position(drawer)
    with ctx.pose({drawer_joint: DRAWER_TRAVEL}):
        ctx.expect_within(
            drawer,
            chamber,
            axes="yz",
            inner_elem="drawer_floor",
            margin=0.002,
            name="extended drawer shell stays aligned with chamber",
        )
        ctx.expect_overlap(
            drawer,
            chamber,
            axes="x",
            elem_a="drawer_floor",
            min_overlap=0.040,
            name="extended drawer keeps retained insertion",
        )
        drawer_extended = ctx.part_world_position(drawer)
    ctx.check(
        "drawer extends forward",
        drawer_rest is not None
        and drawer_extended is not None
        and drawer_extended[0] > drawer_rest[0] + 0.08,
        details=f"rest={drawer_rest}, extended={drawer_extended}",
    )

    button_rest = ctx.part_world_position(release_button)
    with ctx.pose({release_joint: 0.004}):
        button_pressed = ctx.part_world_position(release_button)
    ctx.check(
        "release button presses into basket handle",
        button_rest is not None
        and button_pressed is not None
        and button_pressed[2] < button_rest[2] - 0.002,
        details=f"rest={button_rest}, pressed={button_pressed}",
    )

    def indicator_center(part_name: str) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(part_name, elem="dial_indicator")
        if aabb is None:
            return None
        min_pt, max_pt = aabb
        return (
            (float(min_pt[0]) + float(max_pt[0])) * 0.5,
            (float(min_pt[1]) + float(max_pt[1])) * 0.5,
            (float(min_pt[2]) + float(max_pt[2])) * 0.5,
        )

    temp_indicator_rest = indicator_center(temp_dial)
    with ctx.pose({temp_joint: 1.2}):
        temp_indicator_rotated = indicator_center(temp_dial)
    ctx.check(
        "temperature dial rotates continuously",
        temp_joint.motion_limits is not None
        and temp_joint.motion_limits.lower is None
        and temp_joint.motion_limits.upper is None
        and temp_indicator_rest is not None
        and temp_indicator_rotated is not None
        and abs(temp_indicator_rotated[1] - temp_indicator_rest[1]) > 0.004,
        details=f"rest={temp_indicator_rest}, rotated={temp_indicator_rotated}",
    )

    time_indicator_rest = indicator_center(time_dial)
    with ctx.pose({time_joint: -1.1}):
        time_indicator_rotated = indicator_center(time_dial)
    ctx.check(
        "time dial rotates continuously",
        time_joint.motion_limits is not None
        and time_joint.motion_limits.lower is None
        and time_joint.motion_limits.upper is None
        and time_indicator_rest is not None
        and time_indicator_rotated is not None
        and abs(time_indicator_rotated[1] - time_indicator_rest[1]) > 0.004,
        details=f"rest={time_indicator_rest}, rotated={time_indicator_rotated}",
    )

    preset_rest_0 = ctx.part_world_position(preset_button_0)
    with ctx.pose({preset_joint_0: 0.004}):
        preset_pressed_0 = ctx.part_world_position(preset_button_0)
    ctx.check(
        "preset button 0 presses inward",
        preset_rest_0 is not None
        and preset_pressed_0 is not None
        and preset_pressed_0[2] < preset_rest_0[2] - 0.001,
        details=f"rest={preset_rest_0}, pressed={preset_pressed_0}",
    )

    preset_rest_1 = ctx.part_world_position(preset_button_1)
    with ctx.pose({preset_joint_1: 0.004}):
        preset_pressed_1 = ctx.part_world_position(preset_button_1)
    ctx.check(
        "preset button 1 presses inward",
        preset_rest_1 is not None
        and preset_pressed_1 is not None
        and preset_pressed_1[2] < preset_rest_1[2] - 0.001,
        details=f"rest={preset_rest_1}, pressed={preset_pressed_1}",
    )

    ctx.check("housing exists", housing is not None, details="Expected housing root part.")

    return ctx.report()


object_model = build_object_model()
