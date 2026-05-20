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
    VentGrilleFrame,
    VentGrilleGeometry,
    VentGrilleSlats,
    VentGrilleSleeve,
    mesh_from_cadquery,
    mesh_from_geometry,
)


WIDTH = 0.34
DEPTH = 0.23
HEIGHT = 0.58
WALL = 0.0045
BOTTOM_THICKNESS = 0.010
TOP_THICKNESS = 0.012
CORNER_RADIUS = 0.016

GRILLE_OPENING_WIDTH = 0.242
GRILLE_OPENING_HEIGHT = 0.342
GRILLE_CENTER_Z = 0.225
GRILLE_PANEL_SIZE = (0.254, 0.354)

REAR_OPENING_WIDTH = 0.286
REAR_OPENING_HEIGHT = 0.448
REAR_OPENING_CENTER_Z = 0.290

DOOR_WIDTH = 0.294
DOOR_HEIGHT = 0.456
DOOR_THICKNESS = 0.008
DOOR_GAP = 0.0015
HINGE_KNUCKLE_RADIUS = 0.006
HINGE_KNUCKLE_LENGTH = 0.135

FILTER_WIDTH = 0.272
FILTER_HEIGHT = 0.428
FILTER_MAIN_DEPTH = 0.038
FILTER_TAIL_DEPTH = 0.110
FILTER_TRAVEL = 0.102

BUTTON_WIDTH = 0.044
BUTTON_HEIGHT = 0.022
BUTTON_DEPTH = 0.008
BUTTON_FACE_DEPTH = 0.0035
BUTTON_TRAVEL = 0.0045
BUTTON_SLOT_WIDTH = 0.0415
BUTTON_SLOT_HEIGHT = 0.0195
BUTTON_ROW_Z = 0.470
BUTTON_X_OFFSETS = (-0.052, 0.0, 0.052)


def _build_body_shell() -> cq.Workplane:
    outer = cq.Workplane("XY").box(WIDTH, DEPTH, HEIGHT).translate((0.0, 0.0, HEIGHT / 2.0))
    outer = outer.edges("|Z").fillet(CORNER_RADIUS)

    inner = (
        cq.Workplane("XY")
        .box(
            WIDTH - 2.0 * WALL,
            DEPTH - 2.0 * WALL,
            HEIGHT - BOTTOM_THICKNESS - TOP_THICKNESS,
        )
        .translate((0.0, 0.0, HEIGHT / 2.0 + (BOTTOM_THICKNESS - TOP_THICKNESS) / 2.0))
    )

    shell = outer.cut(inner)

    front_grille_cut = (
        cq.Workplane("XY")
        .box(GRILLE_OPENING_WIDTH, WALL * 3.0, GRILLE_OPENING_HEIGHT)
        .translate((0.0, DEPTH / 2.0 - WALL * 1.5, GRILLE_CENTER_Z))
    )
    shell = shell.cut(front_grille_cut)

    rear_filter_cut = (
        cq.Workplane("XY")
        .box(REAR_OPENING_WIDTH, WALL * 3.0, REAR_OPENING_HEIGHT)
        .translate((0.0, -DEPTH / 2.0 + WALL * 1.5, REAR_OPENING_CENTER_Z))
    )
    shell = shell.cut(rear_filter_cut)

    control_recess = (
        cq.Workplane("XY")
        .box(0.190, 0.005, 0.040)
        .translate((0.0, DEPTH / 2.0 - 0.0025, BUTTON_ROW_Z))
    )
    shell = shell.cut(control_recess)

    for button_x in BUTTON_X_OFFSETS:
        button_slot = (
            cq.Workplane("XY")
            .box(BUTTON_SLOT_WIDTH, WALL * 4.0, BUTTON_SLOT_HEIGHT)
            .translate((button_x, DEPTH / 2.0 - WALL * 1.8, BUTTON_ROW_Z))
        )
        shell = shell.cut(button_slot)

    badge_recess = (
        cq.Workplane("XY")
        .box(0.090, 0.0015, 0.010)
        .translate((0.0, DEPTH / 2.0 - 0.00075, 0.425))
    )
    shell = shell.cut(badge_recess)

    return shell


def _build_button_shape() -> cq.Workplane:
    cap = (
        cq.Workplane("XY")
        .box(BUTTON_WIDTH, BUTTON_FACE_DEPTH, BUTTON_HEIGHT)
        .edges("|Y")
        .fillet(0.0025)
        .translate((0.0, BUTTON_FACE_DEPTH / 2.0, 0.0))
    )
    stem = (
        cq.Workplane("XY")
        .box(BUTTON_SLOT_WIDTH, BUTTON_DEPTH, BUTTON_SLOT_HEIGHT)
        .translate((0.0, -BUTTON_DEPTH / 2.0, 0.0))
    )
    return cap.union(stem)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="floor_air_purifier")

    shell_white = model.material("shell_white", rgba=(0.92, 0.93, 0.91, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.62, 0.65, 0.66, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.20, 0.22, 1.0))
    filter_black = model.material("filter_black", rgba=(0.14, 0.15, 0.16, 1.0))
    filter_gray = model.material("filter_gray", rgba=(0.36, 0.39, 0.41, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shell(), "purifier_body_shell"),
        material=shell_white,
        name="shell",
    )
    body.visual(
        mesh_from_geometry(
            VentGrilleGeometry(
                GRILLE_PANEL_SIZE,
                frame=0.010,
                face_thickness=0.0035,
                slat_pitch=0.016,
                slat_width=0.0085,
                slat_angle_deg=32.0,
                slats=VentGrilleSlats(profile="airfoil", direction="down", divider_count=2, divider_width=0.004),
                frame_profile=VentGrilleFrame(style="beveled", depth=0.0012),
                sleeve=VentGrilleSleeve(style="none"),
            ),
            "purifier_front_grille",
        ),
        origin=Origin(
            xyz=(0.0, DEPTH / 2.0 - 0.0032, GRILLE_CENTER_Z),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=trim_gray,
        name="front_grille",
    )
    body.visual(
        Box((0.088, 0.003, 0.008)),
        origin=Origin(xyz=(0.0, DEPTH / 2.0 - 0.0015, 0.425)),
        material=trim_gray,
        name="badge_strip",
    )
    body.visual(
        Box((0.250, DEPTH - 0.020, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=trim_gray,
        name="base_rail",
    )

    button_carrier = model.part("button_carrier")
    button_carrier.visual(
        Box((0.186, 0.006, 0.032)),
        origin=Origin(xyz=(0.0, -0.003, 0.0)),
        material=trim_gray,
        name="carrier_plate",
    )
    for side_index, side_sign in enumerate((-1.0, 1.0)):
        button_carrier.visual(
            Box((0.014, 0.010, 0.028)),
            origin=Origin(xyz=(side_sign * 0.094, -0.001, 0.0)),
            material=trim_gray,
            name=f"carrier_mount_{side_index}",
        )
    for index, button_x in enumerate(BUTTON_X_OFFSETS):
        for side_index, side_sign in enumerate((-1.0, 1.0)):
            button_carrier.visual(
                Box((0.006, 0.012, 0.022)),
                origin=Origin(
                    xyz=(
                        button_x + side_sign * (BUTTON_SLOT_WIDTH / 2.0 + 0.003),
                        -0.006,
                        0.0,
                    )
                ),
                material=trim_gray,
                name=f"button_guide_{index}_{side_index}",
            )

    filter_door = model.part("filter_door")
    filter_door.visual(
        Box((DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(-DOOR_WIDTH / 2.0, -DOOR_THICKNESS / 2.0, 0.0)),
        material=shell_white,
        name="door_panel",
    )
    filter_door.visual(
        Cylinder(radius=HINGE_KNUCKLE_RADIUS, length=HINGE_KNUCKLE_LENGTH),
        origin=Origin(xyz=(0.0, -DOOR_THICKNESS / 2.0, 0.145)),
        material=trim_gray,
        name="hinge_knuckle_top",
    )
    filter_door.visual(
        Cylinder(radius=HINGE_KNUCKLE_RADIUS, length=HINGE_KNUCKLE_LENGTH),
        origin=Origin(xyz=(0.0, -DOOR_THICKNESS / 2.0, -0.145)),
        material=trim_gray,
        name="hinge_knuckle_bottom",
    )
    filter_door.visual(
        Box((0.040, 0.014, 0.090)),
        origin=Origin(xyz=(-DOOR_WIDTH + 0.030, -DOOR_THICKNESS - 0.007, 0.0)),
        material=trim_gray,
        name="door_handle",
    )

    filter_pack = model.part("filter_pack")
    filter_pack.visual(
        Box((FILTER_WIDTH, FILTER_MAIN_DEPTH, FILTER_HEIGHT)),
        origin=Origin(xyz=(0.0, FILTER_MAIN_DEPTH / 2.0, 0.0)),
        material=filter_gray,
        name="filter_frame",
    )
    filter_pack.visual(
        Box((FILTER_WIDTH - 0.022, FILTER_MAIN_DEPTH - 0.010, FILTER_HEIGHT - 0.024)),
        origin=Origin(xyz=(0.0, FILTER_MAIN_DEPTH / 2.0, 0.0)),
        material=filter_black,
        name="filter_media",
    )
    filter_pack.visual(
        Box((FILTER_WIDTH - 0.060, FILTER_TAIL_DEPTH, FILTER_HEIGHT - 0.120)),
        origin=Origin(xyz=(0.0, FILTER_MAIN_DEPTH + FILTER_TAIL_DEPTH / 2.0, 0.0)),
        material=charcoal,
        name="guide_tail",
    )
    filter_pack.visual(
        Box((0.085, 0.007, 0.028)),
        origin=Origin(xyz=(0.0, -0.0035, FILTER_HEIGHT / 2.0 - 0.034)),
        material=trim_gray,
        name="pull_tab",
    )

    button_mesh = mesh_from_cadquery(_build_button_shape(), "purifier_button")
    for index in range(3):
        button = model.part(f"button_{index}")
        button.visual(
            button_mesh,
            origin=Origin(),
            material=charcoal,
            name="button_cap",
        )

    model.articulation(
        "body_to_filter_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=filter_door,
        origin=Origin(
            xyz=(
                REAR_OPENING_WIDTH / 2.0 + 0.004,
                -DEPTH / 2.0 - DOOR_GAP,
                REAR_OPENING_CENTER_Z,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(110.0),
        ),
    )

    model.articulation(
        "body_to_button_carrier",
        ArticulationType.FIXED,
        parent=body,
        child=button_carrier,
        origin=Origin(xyz=(0.0, DEPTH / 2.0 - WALL, BUTTON_ROW_Z)),
    )

    model.articulation(
        "body_to_filter_pack",
        ArticulationType.PRISMATIC,
        parent=body,
        child=filter_pack,
        origin=Origin(
            xyz=(
                0.0,
                -DEPTH / 2.0 + WALL + 0.001,
                REAR_OPENING_CENTER_Z,
            )
        ),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.16,
            lower=0.0,
            upper=FILTER_TRAVEL,
        ),
    )

    for index, button_x in enumerate(BUTTON_X_OFFSETS):
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=f"button_{index}",
            origin=Origin(xyz=(button_x, DEPTH / 2.0, BUTTON_ROW_Z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=0.06,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    button_carrier = object_model.get_part("button_carrier")
    filter_door = object_model.get_part("filter_door")
    filter_pack = object_model.get_part("filter_pack")
    door_joint = object_model.get_articulation("body_to_filter_door")
    filter_joint = object_model.get_articulation("body_to_filter_pack")
    buttons = [object_model.get_part(f"button_{index}") for index in range(3)]
    button_joints = [object_model.get_articulation(f"body_to_button_{index}") for index in range(3)]

    ctx.allow_overlap(
        body,
        button_carrier,
        elem_a="shell",
        elem_b="carrier_mount_0",
        reason="The hidden button carrier is intentionally represented as clipped into simplified shell-side mounting bosses.",
    )
    ctx.allow_overlap(
        body,
        button_carrier,
        elem_a="shell",
        elem_b="carrier_mount_1",
        reason="The hidden button carrier is intentionally represented as clipped into simplified shell-side mounting bosses.",
    )

    ctx.expect_gap(
        body,
        filter_door,
        axis="y",
        positive_elem="shell",
        negative_elem="door_panel",
        min_gap=0.0005,
        max_gap=0.0035,
        name="rear door seats just behind the cabinet shell",
    )
    ctx.expect_overlap(
        filter_door,
        body,
        axes="xz",
        elem_a="door_panel",
        elem_b="shell",
        min_overlap=0.26,
        name="rear door covers the rear service opening",
    )
    ctx.expect_overlap(
        filter_pack,
        body,
        axes="xz",
        elem_a="filter_frame",
        elem_b="shell",
        min_overlap=0.24,
        name="filter pack aligns with the purifier body footprint",
    )

    closed_door_aabb = ctx.part_world_aabb(filter_door)
    with ctx.pose({door_joint: door_joint.motion_limits.upper}):
        open_door_aabb = ctx.part_world_aabb(filter_door)
    ctx.check(
        "rear door swings outward from the cabinet rear",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[0][1] < closed_door_aabb[0][1] - 0.10,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    rest_filter_pos = ctx.part_world_position(filter_pack)
    with ctx.pose({filter_joint: filter_joint.motion_limits.upper}):
        extended_filter_pos = ctx.part_world_position(filter_pack)
        ctx.expect_overlap(
            filter_pack,
            body,
            axes="y",
            elem_a="guide_tail",
            elem_b="shell",
            min_overlap=0.040,
            name="extended filter pack remains captured by the body guides",
        )
    ctx.check(
        "filter pack slides out from the rear opening",
        rest_filter_pos is not None
        and extended_filter_pos is not None
        and extended_filter_pos[1] < rest_filter_pos[1] - 0.09,
        details=f"rest={rest_filter_pos}, extended={extended_filter_pos}",
    )

    button_rest_positions = [ctx.part_world_position(button) for button in buttons]
    for index, joint in enumerate(button_joints):
        pose_map = {joint: joint.motion_limits.upper}
        with ctx.pose(pose_map):
            pressed_positions = [ctx.part_world_position(button) for button in buttons]
        moved_ok = (
            button_rest_positions[index] is not None
            and pressed_positions[index] is not None
            and pressed_positions[index][1] < button_rest_positions[index][1] - 0.0035
        )
        other_ok = True
        for other_index in range(3):
            if other_index == index:
                continue
            if button_rest_positions[other_index] is None or pressed_positions[other_index] is None:
                other_ok = False
                break
            if abs(pressed_positions[other_index][1] - button_rest_positions[other_index][1]) > 1e-6:
                other_ok = False
                break
        ctx.check(
            f"button_{index} depresses independently",
            moved_ok and other_ok,
            details=f"rest={button_rest_positions}, pressed={pressed_positions}",
        )

    return ctx.report()


object_model = build_object_model()
