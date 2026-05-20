from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


BASE_X = 0.260
BASE_Y = 0.290
BASE_T = 0.008

POST_SPAN = 0.210
POST_X_POS = -0.090

SLEEVE_X = 0.030
SLEEVE_Y = 0.024
SLEEVE_H = 0.060
SLEEVE_WALL = 0.003

POST_MAST_X = 0.021
POST_MAST_Y = 0.015
POST_MAST_LEN = 0.140
POST_REST_INSERTION = 0.065
POST_TRAVEL = 0.040

HEAD_X = 0.036
HEAD_Y = 0.022
HEAD_Z = 0.024
HEAD_CENTER_Z = 0.081

HINGE_R = 0.006
HINGE_CLEARANCE = 0.0

TRAY_W = 0.305
TRAY_D = 0.242
TRAY_T = 0.005
TRAY_REST_TILT = 0.22
TRAY_PANEL_Z = 0.024
TRAY_REAR_X = 0.018
TRAY_FRONT_LIP_H = 0.016
TRAY_SIDE_RAIL_H = 0.009


def _mesh_visual(part, shape, mesh_name: str, material: str, *, visual_name: str) -> None:
    part.visual(
        mesh_from_cadquery(shape, mesh_name),
        material=material,
        name=visual_name,
    )


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _sleeve_shape() -> cq.Workplane:
    center_z = BASE_T - 0.001 + SLEEVE_H / 2.0
    side_x = SLEEVE_X - 0.012
    side_center_x = -0.003
    side_offset_y = (SLEEVE_Y - SLEEVE_WALL) / 2.0
    rear_center_x = -(SLEEVE_X - SLEEVE_WALL) / 2.0
    left_wall = _box((side_x, SLEEVE_WALL, SLEEVE_H), (side_center_x, -side_offset_y, center_z))
    right_wall = _box((side_x, SLEEVE_WALL, SLEEVE_H), (side_center_x, side_offset_y, center_z))
    rear_wall = _box((SLEEVE_WALL, SLEEVE_Y, SLEEVE_H), (rear_center_x, 0.0, center_z))
    return left_wall.union(right_wall).union(rear_wall)


def _post_mast_shape() -> cq.Workplane:
    mast_center_z = POST_MAST_LEN / 2.0 - POST_REST_INSERTION
    return _box((POST_MAST_X, POST_MAST_Y, POST_MAST_LEN), (0.0, 0.0, mast_center_z))


def _post_head_shape() -> cq.Workplane:
    head = _box((HEAD_X, HEAD_Y, HEAD_Z), (0.0, 0.0, HEAD_CENTER_Z))
    rod_relief = (
        cq.Workplane("XZ")
        .circle(HINGE_R + HINGE_CLEARANCE)
        .extrude(HEAD_Y + 0.008, both=True)
        .translate((0.0, 0.0, HEAD_CENTER_Z + 0.002))
    )
    top_slot = _box(
        (HEAD_X + 0.004, HEAD_Y + 0.004, HEAD_Z),
        (0.0, 0.0, HEAD_CENTER_Z + HEAD_Z / 2.0),
    )
    return head.cut(rod_relief).cut(top_slot)


def _tilted_box(
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    tilt: float = TRAY_REST_TILT,
) -> cq.Workplane:
    return _box(size, center).rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), tilt * 180.0 / pi)


def _hinge_rod_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(HINGE_R)
        .extrude(POST_SPAN + HEAD_Y)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -90.0)
        .translate((0.0, -HEAD_Y / 2.0, 0.0))
    )


def _tray_panel_shape() -> cq.Workplane:
    center = (
        TRAY_REAR_X + TRAY_D / 2.0,
        POST_SPAN / 2.0,
        TRAY_PANEL_Z,
    )
    return _tilted_box((TRAY_D, TRAY_W, TRAY_T), center)


def _tray_front_lip_shape() -> cq.Workplane:
    center = (
        TRAY_REAR_X + TRAY_D - 0.004,
        POST_SPAN / 2.0,
        TRAY_PANEL_Z + TRAY_FRONT_LIP_H / 2.0,
    )
    return _tilted_box((0.008, TRAY_W - 0.020, TRAY_FRONT_LIP_H), center)


def _tray_side_rail_shape(side_sign: float) -> cq.Workplane:
    y_center = POST_SPAN / 2.0 + side_sign * (TRAY_W / 2.0 - 0.003)
    center = (
        TRAY_REAR_X + TRAY_D / 2.0 + 0.010,
        y_center,
        TRAY_PANEL_Z + TRAY_SIDE_RAIL_H / 2.0,
    )
    return _tilted_box((TRAY_D - 0.030, 0.006, TRAY_SIDE_RAIL_H), center)


def _tray_rib_shape(y_center: float) -> cq.Workplane:
    center = (0.036, y_center, 0.015)
    return _tilted_box((0.050, 0.014, 0.030), center)


def _tray_hinge_arm_shape(y_center: float) -> cq.Workplane:
    center = (0.016, y_center, 0.009)
    return _tilted_box((0.042, 0.018, 0.024), center)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="laptop_stand")

    model.material("powder", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("aluminum", rgba=(0.76, 0.79, 0.82, 1.0))
    model.material("dark_aluminum", rgba=(0.55, 0.59, 0.63, 1.0))
    model.material("rubber", rgba=(0.12, 0.12, 0.13, 1.0))

    base = model.part("base")
    base_plate = _box((BASE_X, BASE_Y, BASE_T), (0.0, 0.0, BASE_T / 2.0)).edges("|Z").fillet(0.010)
    _mesh_visual(base, base_plate, "base_plate", "powder", visual_name="base_plate")
    for idx, y_pos in enumerate((-POST_SPAN / 2.0, POST_SPAN / 2.0)):
        sleeve = _sleeve_shape().translate((POST_X_POS, y_pos, 0.0))
        _mesh_visual(base, sleeve, f"base_sleeve_{idx}", "powder", visual_name=f"sleeve_{idx}")
    for idx, y_pos in enumerate((-0.085, 0.085)):
        foot = _box((0.065, 0.020, 0.003), (0.050, y_pos, 0.0015))
        _mesh_visual(base, foot, f"base_foot_{idx}", "rubber", visual_name=f"foot_{idx}")

    post_0 = model.part("post_0")
    _mesh_visual(post_0, _post_mast_shape(), "post_0_mast", "dark_aluminum", visual_name="mast")
    _mesh_visual(post_0, _post_head_shape(), "post_0_head", "aluminum", visual_name="head")

    post_1 = model.part("post_1")
    _mesh_visual(post_1, _post_mast_shape(), "post_1_mast", "dark_aluminum", visual_name="mast")
    _mesh_visual(post_1, _post_head_shape(), "post_1_head", "aluminum", visual_name="head")

    tray = model.part("tray")
    _mesh_visual(tray, _hinge_rod_shape(), "tray_hinge_rod", "dark_aluminum", visual_name="hinge_rod")
    _mesh_visual(tray, _tray_panel_shape(), "tray_panel", "aluminum", visual_name="panel")
    _mesh_visual(tray, _tray_front_lip_shape(), "tray_front_lip", "aluminum", visual_name="front_lip")
    _mesh_visual(tray, _tray_side_rail_shape(-1.0), "tray_side_rail_0", "aluminum", visual_name="side_rail_0")
    _mesh_visual(tray, _tray_side_rail_shape(1.0), "tray_side_rail_1", "aluminum", visual_name="side_rail_1")
    _mesh_visual(tray, _tray_hinge_arm_shape(0.046), "tray_hinge_arm_0", "dark_aluminum", visual_name="hinge_arm_0")
    _mesh_visual(
        tray,
        _tray_hinge_arm_shape(POST_SPAN - 0.046),
        "tray_hinge_arm_1",
        "dark_aluminum",
        visual_name="hinge_arm_1",
    )
    _mesh_visual(tray, _tray_rib_shape(0.048), "tray_rib_0", "dark_aluminum", visual_name="rib_0")
    _mesh_visual(tray, _tray_rib_shape(POST_SPAN - 0.048), "tray_rib_1", "dark_aluminum", visual_name="rib_1")

    base_to_post_0 = model.articulation(
        "post_0_lift",
        ArticulationType.PRISMATIC,
        parent=base,
        child=post_0,
        origin=Origin(xyz=(POST_X_POS, -POST_SPAN / 2.0, BASE_T + SLEEVE_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=POST_TRAVEL, effort=120.0, velocity=0.15),
    )
    base_to_post_1 = model.articulation(
        "post_1_lift",
        ArticulationType.PRISMATIC,
        parent=base,
        child=post_1,
        origin=Origin(xyz=(POST_X_POS, POST_SPAN / 2.0, BASE_T + SLEEVE_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=POST_TRAVEL, effort=120.0, velocity=0.15),
    )
    tray_tilt = model.articulation(
        "tray_tilt",
        ArticulationType.REVOLUTE,
        parent=post_0,
        child=tray,
        origin=Origin(xyz=(0.0, 0.0, HEAD_CENTER_Z + 0.002)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.18, upper=0.62, effort=40.0, velocity=1.0),
    )

    # Keep local names used in tests semantically obvious even though the graph
    # cannot encode the real synchronization linkage between the twin lifts.
    _ = (base_to_post_0, base_to_post_1, tray_tilt)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    base = object_model.get_part("base")
    post_0 = object_model.get_part("post_0")
    post_1 = object_model.get_part("post_1")
    tray = object_model.get_part("tray")

    post_0_lift = object_model.get_articulation("post_0_lift")
    post_1_lift = object_model.get_articulation("post_1_lift")
    tray_tilt = object_model.get_articulation("tray_tilt")

    for idx, post in enumerate((post_0, post_1)):
        ctx.expect_within(
            post,
            base,
            axes="y",
            inner_elem="mast",
            outer_elem=f"sleeve_{idx}",
            margin=0.0015,
            name=f"post_{idx} mast stays between sleeve cheeks",
        )
        ctx.expect_overlap(
            post,
            base,
            axes="z",
            elem_a="mast",
            elem_b=f"sleeve_{idx}",
            min_overlap=0.055,
            name=f"post_{idx} retains insertion at rest",
        )

    rest_post_0 = ctx.part_world_position(post_0)
    rest_post_1 = ctx.part_world_position(post_1)
    rest_front_lip = ctx.part_element_world_aabb(tray, elem="front_lip")

    with ctx.pose({post_0_lift: POST_TRAVEL, post_1_lift: POST_TRAVEL}):
        for idx, post in enumerate((post_0, post_1)):
            ctx.expect_within(
                post,
                base,
                axes="y",
                inner_elem="mast",
                outer_elem=f"sleeve_{idx}",
                margin=0.0015,
                name=f"post_{idx} mast stays between sleeve cheeks when raised",
            )
            ctx.expect_overlap(
                post,
                base,
                axes="z",
                elem_a="mast",
                elem_b=f"sleeve_{idx}",
                min_overlap=0.022,
                name=f"post_{idx} keeps retained insertion when raised",
            )

        high_post_0 = ctx.part_world_position(post_0)
        high_post_1 = ctx.part_world_position(post_1)
        head_0 = ctx.part_element_world_aabb(post_0, elem="head")
        head_1 = ctx.part_element_world_aabb(post_1, elem="head")

        ctx.check(
            "post_0 rises with positive lift",
            rest_post_0 is not None and high_post_0 is not None and high_post_0[2] > rest_post_0[2] + 0.030,
            details=f"rest={rest_post_0}, raised={high_post_0}",
        )
        ctx.check(
            "post_1 rises with positive lift",
            rest_post_1 is not None and high_post_1 is not None and high_post_1[2] > rest_post_1[2] + 0.030,
            details=f"rest={rest_post_1}, raised={high_post_1}",
        )
        ctx.check(
            "post heads stay level when equally raised",
            head_0 is not None
            and head_1 is not None
            and abs(head_0[1][2] - head_1[1][2]) <= 0.001
            and abs(head_0[0][0] - head_1[0][0]) <= 0.001,
            details=f"head_0={head_0}, head_1={head_1}",
        )

    with ctx.pose({tray_tilt: 0.50}):
        steep_front_lip = ctx.part_element_world_aabb(tray, elem="front_lip")
        ctx.check(
            "tray front edge rises as the tray tilts",
            rest_front_lip is not None
            and steep_front_lip is not None
            and steep_front_lip[1][2] > rest_front_lip[1][2] + 0.050,
            details=f"rest={rest_front_lip}, steep={steep_front_lip}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
