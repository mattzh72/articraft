from __future__ import annotations

import os

_ORIGINAL_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _ORIGINAL_GETCWD()
    except FileNotFoundError:
        return "/"


os.getcwd = _safe_getcwd

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

BODY_WIDTH = 0.72
BODY_DEPTH = 0.48
BODY_HEIGHT = 0.93
SIDE_THICKNESS = 0.026
TOP_THICKNESS = 0.030
BOTTOM_THICKNESS = 0.040
BACK_THICKNESS = 0.018

INNER_WIDTH = BODY_WIDTH - 2.0 * SIDE_THICKNESS
FACE_THICKNESS = 0.018
FACE_INSET = 0.001
TRAY_DEPTH = 0.39
TRAY_WIDTH = 0.620
DRAWER_ORIGIN_Y = BODY_DEPTH * 0.5 - FACE_INSET - FACE_THICKNESS - TRAY_DEPTH * 0.5

BODY_RAIL_THICKNESS = 0.014
DRAWER_SLIDE_THICKNESS = 0.010
RAIL_LENGTH = 0.33
SLIDE_LENGTH = 0.31
RAIL_HEIGHT = 0.016
SLIDE_ENGAGEMENT_MIN = 0.015
BODY_RAIL_CENTER_X = INNER_WIDTH * 0.5 - BODY_RAIL_THICKNESS * 0.5
DRAWER_SLIDE_CENTER_X = TRAY_WIDTH * 0.5 + DRAWER_SLIDE_THICKNESS * 0.5

DRAWER_GAP = 0.008
TOP_MARGIN = 0.023
BOTTOM_MARGIN = 0.023
FRONT_FRAME_DEPTH = 0.020

DRAWER_LAYOUT = [
    ("shallow_drawer_1", 0.078, 0.058, 0.180),
    ("shallow_drawer_2", 0.078, 0.058, 0.180),
    ("shallow_drawer_3", 0.078, 0.058, 0.180),
    ("shallow_drawer_4", 0.078, 0.058, 0.180),
    ("shallow_drawer_5", 0.078, 0.058, 0.180),
    ("shallow_drawer_6", 0.078, 0.058, 0.180),
    ("deep_drawer_1", 0.145, 0.120, 0.240),
    ("deep_drawer_2", 0.145, 0.120, 0.240),
]


def _drawer_centers() -> list[tuple[str, float, float, float, float]]:
    current_top = BODY_HEIGHT - TOP_THICKNESS - TOP_MARGIN
    positioned: list[tuple[str, float, float, float, float]] = []
    for index, (name, face_height, tray_height, travel) in enumerate(DRAWER_LAYOUT):
        center_z = current_top - face_height * 0.5
        positioned.append((name, face_height, tray_height, travel, center_z))
        current_top -= face_height
        if index < len(DRAWER_LAYOUT) - 1:
            current_top -= DRAWER_GAP
    return positioned


def _span_from_aabb(aabb: tuple[tuple[float, float, float], tuple[float, float, float]], axis: int) -> float:
    return aabb[1][axis] - aabb[0][axis]


def _add_body_visuals(model: ArticulatedObject, body, body_material, rail_material) -> dict[str, float]:
    half_w = BODY_WIDTH * 0.5
    half_d = BODY_DEPTH * 0.5

    body.visual(
        Box((BODY_WIDTH, BODY_DEPTH, TOP_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT - TOP_THICKNESS * 0.5)),
        material=body_material,
        name="top_panel",
    )
    body.visual(
        Box((BODY_WIDTH, BODY_DEPTH, BOTTOM_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BOTTOM_THICKNESS * 0.5)),
        material=body_material,
        name="bottom_panel",
    )
    body.visual(
        Box((SIDE_THICKNESS, BODY_DEPTH, BODY_HEIGHT)),
        origin=Origin(xyz=(-half_w + SIDE_THICKNESS * 0.5, 0.0, BODY_HEIGHT * 0.5)),
        material=body_material,
        name="left_wall",
    )
    body.visual(
        Box((SIDE_THICKNESS, BODY_DEPTH, BODY_HEIGHT)),
        origin=Origin(xyz=(half_w - SIDE_THICKNESS * 0.5, 0.0, BODY_HEIGHT * 0.5)),
        material=body_material,
        name="right_wall",
    )
    body.visual(
        Box((INNER_WIDTH, BACK_THICKNESS, BODY_HEIGHT - TOP_THICKNESS - BOTTOM_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                -half_d + BACK_THICKNESS * 0.5,
                BOTTOM_THICKNESS + (BODY_HEIGHT - TOP_THICKNESS - BOTTOM_THICKNESS) * 0.5,
            )
        ),
        material=body_material,
        name="back_panel",
    )
    body.visual(
        Box((BODY_WIDTH, FRONT_FRAME_DEPTH, 0.022)),
        origin=Origin(
            xyz=(0.0, half_d - FRONT_FRAME_DEPTH * 0.5, BODY_HEIGHT - TOP_THICKNESS - 0.011)
        ),
        material=body_material,
        name="top_front_lip",
    )
    body.visual(
        Box((BODY_WIDTH, FRONT_FRAME_DEPTH, BOTTOM_MARGIN)),
        origin=Origin(
            xyz=(0.0, half_d - FRONT_FRAME_DEPTH * 0.5, BOTTOM_THICKNESS + BOTTOM_MARGIN * 0.5)
        ),
        material=body_material,
        name="bottom_front_lip",
    )

    separator_depth = FRONT_FRAME_DEPTH
    separator_y = BODY_DEPTH * 0.5 - separator_depth * 0.5
    drawer_positions = _drawer_centers()
    for index in range(len(drawer_positions) - 1):
        upper_center = drawer_positions[index][4]
        upper_height = drawer_positions[index][1]
        separator_center_z = upper_center - upper_height * 0.5 - DRAWER_GAP * 0.5
        body.visual(
            Box((INNER_WIDTH, separator_depth, DRAWER_GAP)),
            origin=Origin(xyz=(0.0, separator_y, separator_center_z)),
            material=body_material,
            name=f"front_separator_{index + 1}",
        )

    for name, _, _, _, center_z in drawer_positions:
        body.visual(
            Box((BODY_RAIL_THICKNESS, RAIL_LENGTH, RAIL_HEIGHT)),
            origin=Origin(xyz=(-BODY_RAIL_CENTER_X, DRAWER_ORIGIN_Y, center_z)),
            material=rail_material,
            name=f"left_rail_{name}",
        )
        body.visual(
            Box((BODY_RAIL_THICKNESS, RAIL_LENGTH, RAIL_HEIGHT)),
            origin=Origin(xyz=(BODY_RAIL_CENTER_X, DRAWER_ORIGIN_Y, center_z)),
            material=rail_material,
            name=f"right_rail_{name}",
        )

    body.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)),
        mass=48.0,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT * 0.5)),
    )
    return {name: center_z for name, _, _, _, center_z in drawer_positions}


def _add_drawer(
    model: ArticulatedObject,
    *,
    name: str,
    face_height: float,
    tray_height: float,
    travel: float,
    center_z: float,
    face_material,
    shell_material,
    handle_material,
    rail_material,
    parent,
) -> None:
    drawer = model.part(name)

    face_width = INNER_WIDTH - 0.012
    pocket_width = 0.360 if face_height < 0.10 else 0.410
    pocket_height = 0.030 if face_height < 0.10 else 0.044
    handle_outer_width = 0.245 if face_height < 0.10 else 0.290
    handle_radius = 0.004 if face_height < 0.10 else 0.005
    handle_post_width = 0.010
    handle_post_depth = 0.012
    pocket_back_depth = 0.004
    pocket_wall_thickness = 0.006
    pocket_wall_depth = FACE_THICKNESS - pocket_back_depth
    face_center_y = TRAY_DEPTH * 0.5 + FACE_THICKNESS * 0.5
    pocket_back_center_y = TRAY_DEPTH * 0.5 - FACE_THICKNESS + pocket_back_depth * 0.5
    pocket_wall_center_y = TRAY_DEPTH * 0.5 - FACE_THICKNESS + pocket_back_depth + pocket_wall_depth * 0.5
    handle_center_y = pocket_wall_center_y
    handle_post_center_y = pocket_back_center_y + pocket_back_depth * 0.5 + handle_post_depth * 0.5

    side_strip_width = (face_width - pocket_width) * 0.5
    top_strip_height = (face_height - pocket_height) * 0.5
    handle_bar_length = handle_outer_width - 2.0 * handle_post_width

    drawer.visual(
        Box((TRAY_WIDTH, TRAY_DEPTH, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -tray_height * 0.5 + 0.003)),
        material=shell_material,
        name="drawer_bottom",
    )
    drawer.visual(
        Box((0.008, TRAY_DEPTH, tray_height)),
        origin=Origin(xyz=(-TRAY_WIDTH * 0.5 + 0.004, 0.0, 0.0)),
        material=shell_material,
        name="left_side",
    )
    drawer.visual(
        Box((0.008, TRAY_DEPTH, tray_height)),
        origin=Origin(xyz=(TRAY_WIDTH * 0.5 - 0.004, 0.0, 0.0)),
        material=shell_material,
        name="right_side",
    )
    drawer.visual(
        Box((TRAY_WIDTH, 0.008, tray_height)),
        origin=Origin(xyz=(0.0, -TRAY_DEPTH * 0.5 + 0.004, 0.0)),
        material=shell_material,
        name="back_wall",
    )

    drawer.visual(
        Box((side_strip_width, FACE_THICKNESS, face_height)),
        origin=Origin(xyz=(-face_width * 0.5 + side_strip_width * 0.5, face_center_y, 0.0)),
        material=face_material,
        name="face_left",
    )
    drawer.visual(
        Box((side_strip_width, FACE_THICKNESS, face_height)),
        origin=Origin(xyz=(face_width * 0.5 - side_strip_width * 0.5, face_center_y, 0.0)),
        material=face_material,
        name="face_right",
    )
    drawer.visual(
        Box((pocket_width, FACE_THICKNESS, top_strip_height)),
        origin=Origin(
            xyz=(0.0, face_center_y, face_height * 0.5 - top_strip_height * 0.5)
        ),
        material=face_material,
        name="face_top",
    )
    drawer.visual(
        Box((pocket_width, FACE_THICKNESS, top_strip_height)),
        origin=Origin(
            xyz=(0.0, face_center_y, -face_height * 0.5 + top_strip_height * 0.5)
        ),
        material=face_material,
        name="face_bottom",
    )
    drawer.visual(
        Box((pocket_width, pocket_back_depth, pocket_height)),
        origin=Origin(xyz=(0.0, pocket_back_center_y, 0.0)),
        material=face_material,
        name="pocket_back",
    )
    drawer.visual(
        Box((pocket_wall_thickness, pocket_wall_depth, pocket_height)),
        origin=Origin(
            xyz=(-pocket_width * 0.5 + pocket_wall_thickness * 0.5, pocket_wall_center_y, 0.0)
        ),
        material=face_material,
        name="pocket_left_wall",
    )
    drawer.visual(
        Box((pocket_wall_thickness, pocket_wall_depth, pocket_height)),
        origin=Origin(
            xyz=(pocket_width * 0.5 - pocket_wall_thickness * 0.5, pocket_wall_center_y, 0.0)
        ),
        material=face_material,
        name="pocket_right_wall",
    )
    drawer.visual(
        Box((pocket_width, pocket_wall_depth, pocket_wall_thickness)),
        origin=Origin(
            xyz=(0.0, pocket_wall_center_y, pocket_height * 0.5 - pocket_wall_thickness * 0.5)
        ),
        material=face_material,
        name="pocket_top_wall",
    )
    drawer.visual(
        Box((pocket_width, pocket_wall_depth, pocket_wall_thickness)),
        origin=Origin(
            xyz=(0.0, pocket_wall_center_y, -pocket_height * 0.5 + pocket_wall_thickness * 0.5)
        ),
        material=face_material,
        name="pocket_bottom_wall",
    )
    drawer.visual(
        Box((handle_post_width, handle_post_depth, handle_radius * 2.0)),
        origin=Origin(
            xyz=(
                -handle_outer_width * 0.5 + handle_post_width * 0.5,
                handle_post_center_y,
                0.0,
            )
        ),
        material=handle_material,
        name="left_handle_post",
    )
    drawer.visual(
        Box((handle_post_width, handle_post_depth, handle_radius * 2.0)),
        origin=Origin(
            xyz=(
                handle_outer_width * 0.5 - handle_post_width * 0.5,
                handle_post_center_y,
                0.0,
            )
        ),
        material=handle_material,
        name="right_handle_post",
    )
    drawer.visual(
        Cylinder(radius=handle_radius, length=handle_bar_length),
        origin=Origin(xyz=(0.0, handle_center_y, 0.0), rpy=(0.0, pi * 0.5, 0.0)),
        material=handle_material,
        name="handle_bar",
    )

    drawer.visual(
        Box((DRAWER_SLIDE_THICKNESS, SLIDE_LENGTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(-DRAWER_SLIDE_CENTER_X, 0.0, 0.0)),
        material=rail_material,
        name="left_slide",
    )
    drawer.visual(
        Box((DRAWER_SLIDE_THICKNESS, SLIDE_LENGTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(DRAWER_SLIDE_CENTER_X, 0.0, 0.0)),
        material=rail_material,
        name="right_slide",
    )

    drawer.inertial = Inertial.from_geometry(
        Box((face_width, TRAY_DEPTH + FACE_THICKNESS, face_height)),
        mass=4.2 if face_height < 0.10 else 7.8,
        origin=Origin(xyz=(0.0, FACE_THICKNESS * 0.5, 0.0)),
    )

    model.articulation(
        f"body_to_{name}",
        ArticulationType.PRISMATIC,
        parent=parent,
        child=drawer,
        origin=Origin(xyz=(0.0, DRAWER_ORIGIN_Y, center_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.35,
            lower=0.0,
            upper=travel,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="workshop_tool_chest")

    body_material = model.material("body_steel", rgba=(0.16, 0.19, 0.22, 1.0))
    drawer_face_material = model.material("drawer_face", rgba=(0.23, 0.26, 0.30, 1.0))
    shell_material = model.material("drawer_shell", rgba=(0.53, 0.55, 0.58, 1.0))
    rail_material = model.material("rail_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    handle_material = model.material("handle_black", rgba=(0.08, 0.08, 0.09, 1.0))

    body = model.part("body")
    centers = _add_body_visuals(model, body, body_material, rail_material)

    for name, face_height, tray_height, travel in DRAWER_LAYOUT:
        _add_drawer(
            model,
            name=name,
            face_height=face_height,
            tray_height=tray_height,
            travel=travel,
            center_z=centers[name],
            face_material=drawer_face_material,
            shell_material=shell_material,
            handle_material=handle_material,
            rail_material=rail_material,
            parent=body,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, seed=0)
    body = object_model.get_part("body")
    drawer_specs = _drawer_centers()
    drawer_names = [name for name, _, _, _, _ in drawer_specs]
    drawers = {name: object_model.get_part(name) for name in drawer_names}
    drawer_joints = {
        name: object_model.get_articulation(f"body_to_{name}") for name in drawer_names
    }

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=64)

    body_aabb = ctx.part_world_aabb(body)
    if body_aabb is not None:
        body_width = _span_from_aabb(body_aabb, 0)
        body_depth = _span_from_aabb(body_aabb, 1)
        body_height = _span_from_aabb(body_aabb, 2)
        ctx.check(
            "body_realistic_envelope",
            0.65 <= body_width <= 0.80 and 0.44 <= body_depth <= 0.55 and 0.85 <= body_height <= 1.0,
            f"unexpected chest envelope {(body_width, body_depth, body_height)}",
        )

    ctx.check(
        "drawer_bank_counts",
        sum(name.startswith("shallow") for name in drawer_names) == 6
        and sum(name.startswith("deep") for name in drawer_names) == 2,
        f"expected 6 shallow and 2 deep drawers, got {drawer_names}",
    )

    shallow_aabb = ctx.part_world_aabb(drawers["shallow_drawer_1"])
    deep_aabb = ctx.part_world_aabb(drawers["deep_drawer_1"])
    if shallow_aabb is not None and deep_aabb is not None:
        shallow_height = _span_from_aabb(shallow_aabb, 2)
        deep_height = _span_from_aabb(deep_aabb, 2)
        ctx.check(
            "deep_drawers_taller_than_shallow",
            deep_height > shallow_height + 0.05,
            f"deep drawer height {deep_height:.3f} should exceed shallow height {shallow_height:.3f}",
        )

    for upper_name, lower_name in zip(drawer_names, drawer_names[1:]):
        ctx.expect_gap(
            drawers[upper_name],
            drawers[lower_name],
            axis="z",
            min_gap=DRAWER_GAP - 0.0005,
            max_gap=DRAWER_GAP + 0.0005,
            name=f"{upper_name}_to_{lower_name}_vertical_reveal",
        )

    for name, _, _, travel, _ in drawer_specs:
        drawer = drawers[name]
        joint = drawer_joints[name]
        left_slide = drawer.get_visual("left_slide")
        right_slide = drawer.get_visual("right_slide")
        handle_bar = drawer.get_visual("handle_bar")
        pocket_back = drawer.get_visual("pocket_back")
        left_rail = body.get_visual(f"left_rail_{name}")
        right_rail = body.get_visual(f"right_rail_{name}")

        ctx.expect_origin_distance(
            drawer,
            body,
            axes="x",
            max_dist=0.001,
            name=f"{name}_centered_on_body",
        )
        ctx.expect_origin_gap(
            drawer,
            body,
            axis="y",
            min_gap=DRAWER_ORIGIN_Y - 0.0001,
            max_gap=DRAWER_ORIGIN_Y + 0.0001,
            name=f"{name}_closed_origin_position",
        )
        ctx.expect_contact(
            body,
            drawer,
            elem_a=left_rail,
            elem_b=left_slide,
            name=f"{name}_left_slide_contact_closed",
        )
        ctx.expect_contact(
            body,
            drawer,
            elem_a=right_rail,
            elem_b=right_slide,
            name=f"{name}_right_slide_contact_closed",
        )
        ctx.expect_overlap(
            body,
            drawer,
            axes="yz",
            elem_a=left_rail,
            elem_b=left_slide,
            min_overlap=SLIDE_ENGAGEMENT_MIN,
            name=f"{name}_left_slide_engagement_closed",
        )
        ctx.expect_overlap(
            body,
            drawer,
            axes="yz",
            elem_a=right_rail,
            elem_b=right_slide,
            min_overlap=SLIDE_ENGAGEMENT_MIN,
            name=f"{name}_right_slide_engagement_closed",
        )
        ctx.expect_within(
            drawer,
            drawer,
            axes="xz",
            inner_elem=handle_bar,
            outer_elem=pocket_back,
            name=f"{name}_handle_recessed_within_pocket",
        )
        ctx.expect_gap(
            drawer,
            drawer,
            axis="y",
            positive_elem=handle_bar,
            negative_elem=pocket_back,
            min_gap=0.002,
            max_gap=0.014,
            name=f"{name}_handle_clear_of_pocket_back",
        )

        limits = joint.motion_limits
        if limits is not None and limits.upper is not None:
            with ctx.pose({joint: limits.upper}):
                ctx.expect_origin_gap(
                    drawer,
                    body,
                    axis="y",
                    min_gap=travel * 0.75,
                    name=f"{name}_opens_forward",
                )
                ctx.expect_contact(
                    body,
                    drawer,
                    elem_a=left_rail,
                    elem_b=left_slide,
                    name=f"{name}_left_slide_contact_open",
                )
                ctx.expect_contact(
                    body,
                    drawer,
                    elem_a=right_rail,
                    elem_b=right_slide,
                    name=f"{name}_right_slide_contact_open",
                )
                ctx.expect_overlap(
                    body,
                    drawer,
                    axes="yz",
                    elem_a=left_rail,
                    elem_b=left_slide,
                    min_overlap=SLIDE_ENGAGEMENT_MIN,
                    name=f"{name}_left_slide_engagement_open",
                )
                ctx.expect_overlap(
                    body,
                    drawer,
                    axes="yz",
                    elem_a=right_rail,
                    elem_b=right_slide,
                    min_overlap=SLIDE_ENGAGEMENT_MIN,
                    name=f"{name}_right_slide_engagement_open",
                )
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{name}_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{name}_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
