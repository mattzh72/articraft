from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

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


PEDESTAL_BASE_W = 0.46
PEDESTAL_BASE_D = 0.34
PEDESTAL_BASE_T = 0.05
PEDESTAL_COLUMN_W = 0.18
PEDESTAL_COLUMN_D = 0.14
PEDESTAL_COLUMN_H = 0.40
PEDESTAL_SHOULDER_W = 0.24
PEDESTAL_SHOULDER_D = 0.18
PEDESTAL_SHOULDER_H = 0.10
PEDESTAL_HEAD_R = 0.10
PEDESTAL_HEAD_H = 0.15
PEDESTAL_HEIGHT = (
    PEDESTAL_BASE_T + PEDESTAL_COLUMN_H + PEDESTAL_SHOULDER_H + PEDESTAL_HEAD_H
)

PLATTER_BASE_R = 0.16
PLATTER_BASE_T = 0.016
PLATTER_TOP_R = 0.145
PLATTER_TOP_T = 0.028
PLATTER_CENTER_BOSS_R = 0.07
PLATTER_CENTER_BOSS_T = 0.018
PLATTER_TOTAL_H = PLATTER_BASE_T + PLATTER_TOP_T

FRAME_BASE_W = 0.25
FRAME_BASE_D = 0.18
FRAME_BASE_T = 0.022
CHEEK_X_CENTER = 0.136
CHEEK_T = 0.028
CHEEK_D = 0.11
CHEEK_H = 0.22
CHEEK_Z0 = 0.015
TRUNNION_AXIS_Z = 0.172
CHEEK_BOSS_R = 0.043
CHEEK_BORE_R = 0.026
CHEEK_OUTER_FACE_X = CHEEK_X_CENTER + (CHEEK_T / 2.0)

SHAFT_R = 0.022
SHAFT_L = 0.326
HUB_R = 0.045
HUB_L = 0.19
COLLAR_R = 0.036
COLLAR_T = 0.010
FACEPLATE_R = 0.102
FACEPLATE_T = 0.024
FACEPLATE_CENTER_HOLE_R = 0.020
FACEPLATE_HOLE_CIRCLE_R = 0.062
FACEPLATE_MOUNT_HOLE_R = 0.0065
STOP_PAD_W = 0.048
STOP_PAD_T = 0.022
STOP_PAD_H = 0.018
STOP_PAD_Z = 0.055


def _circle_points(radius: float, count: int) -> list[tuple[float, float]]:
    return [
        (radius * cos((2.0 * pi * idx) / count), radius * sin((2.0 * pi * idx) / count))
        for idx in range(count)
    ]


def _make_pedestal_shape():
    base = (
        cq.Workplane("XY")
        .box(PEDESTAL_BASE_W, PEDESTAL_BASE_D, PEDESTAL_BASE_T)
        .translate((0.0, 0.0, PEDESTAL_BASE_T / 2.0))
        .edges("|Z")
        .fillet(0.02)
    )
    column = (
        cq.Workplane("XY")
        .box(PEDESTAL_COLUMN_W, PEDESTAL_COLUMN_D, PEDESTAL_COLUMN_H)
        .translate(
            (
                0.0,
                0.0,
                PEDESTAL_BASE_T + (PEDESTAL_COLUMN_H / 2.0),
            )
        )
        .edges("|Z")
        .fillet(0.012)
    )
    shoulder = (
        cq.Workplane("XY")
        .box(PEDESTAL_SHOULDER_W, PEDESTAL_SHOULDER_D, PEDESTAL_SHOULDER_H)
        .translate(
            (
                0.0,
                0.0,
                PEDESTAL_BASE_T
                + PEDESTAL_COLUMN_H
                + (PEDESTAL_SHOULDER_H / 2.0),
            )
        )
        .edges("|Z")
        .fillet(0.01)
    )
    head = (
        cq.Workplane("XY")
        .circle(PEDESTAL_HEAD_R)
        .extrude(PEDESTAL_HEAD_H)
        .translate(
            (
                0.0,
                0.0,
                PEDESTAL_BASE_T + PEDESTAL_COLUMN_H + PEDESTAL_SHOULDER_H,
            )
        )
    )
    return base.union(column).union(shoulder).union(head)


def _make_platter_shape():
    base_ring = cq.Workplane("XY").circle(PLATTER_BASE_R).extrude(PLATTER_BASE_T)
    table = (
        cq.Workplane("XY")
        .circle(PLATTER_TOP_R)
        .extrude(PLATTER_TOP_T)
        .translate((0.0, 0.0, PLATTER_BASE_T))
    )
    center_boss = (
        cq.Workplane("XY")
        .circle(PLATTER_CENTER_BOSS_R)
        .extrude(PLATTER_CENTER_BOSS_T)
        .translate((0.0, 0.0, PLATTER_BASE_T))
    )
    bolt_holes = (
        cq.Workplane("XY")
        .pushPoints(_circle_points(0.102, 6))
        .circle(0.008)
        .extrude(PLATTER_TOTAL_H + 0.004)
        .translate((0.0, 0.0, -0.002))
    )
    return base_ring.union(table).union(center_boss).cut(bolt_holes)


def _make_frame_base():
    return (
        cq.Workplane("XY")
        .box(FRAME_BASE_W, FRAME_BASE_D, FRAME_BASE_T)
        .translate((0.0, 0.0, FRAME_BASE_T / 2.0))
        .edges("|Z")
        .fillet(0.008)
    )


def _make_cheek(x_center: float):
    cheek_plate = cq.Workplane("XY").box(CHEEK_T, CHEEK_D, CHEEK_H).translate(
        (
            x_center,
            0.0,
            CHEEK_Z0 + (CHEEK_H / 2.0),
        )
    )
    bearing_boss = (
        cq.Workplane("YZ")
        .circle(CHEEK_BOSS_R)
        .extrude(CHEEK_T)
        .translate((x_center - (CHEEK_T / 2.0), 0.0, TRUNNION_AXIS_Z))
    )
    bore = (
        cq.Workplane("YZ")
        .circle(CHEEK_BORE_R)
        .extrude(CHEEK_T + 0.006)
        .translate((x_center - ((CHEEK_T + 0.006) / 2.0), 0.0, TRUNNION_AXIS_Z))
    )
    return cheek_plate.union(bearing_boss).cut(bore)


def _make_tilt_shaft():
    return (
        cq.Workplane("YZ")
        .circle(SHAFT_R)
        .extrude(SHAFT_L)
        .translate((-SHAFT_L / 2.0, 0.0, 0.0))
    )


def _make_trunnion_hub():
    return (
        cq.Workplane("YZ")
        .circle(HUB_R)
        .extrude(HUB_L)
        .translate((-HUB_L / 2.0, 0.0, 0.0))
    )


def _make_bearing_collars():
    left_collar = (
        cq.Workplane("YZ")
        .circle(COLLAR_R)
        .extrude(COLLAR_T)
        .translate((CHEEK_OUTER_FACE_X, 0.0, 0.0))
    )
    right_collar = (
        cq.Workplane("YZ")
        .circle(COLLAR_R)
        .extrude(COLLAR_T)
        .translate((-(CHEEK_OUTER_FACE_X + COLLAR_T), 0.0, 0.0))
    )
    return left_collar.union(right_collar)


def _make_faceplate_disk():
    disk = (
        cq.Workplane("XZ")
        .circle(FACEPLATE_R)
        .extrude(FACEPLATE_T)
        .translate((0.0, -(FACEPLATE_T / 2.0), 0.0))
    )
    center_hole = (
        cq.Workplane("XZ")
        .circle(FACEPLATE_CENTER_HOLE_R)
        .extrude(FACEPLATE_T + 0.004)
        .translate((0.0, -((FACEPLATE_T + 0.004) / 2.0), 0.0))
    )
    mounting_holes = (
        cq.Workplane("XZ")
        .pushPoints(
            [
                (FACEPLATE_HOLE_CIRCLE_R, 0.0),
                (-FACEPLATE_HOLE_CIRCLE_R, 0.0),
                (0.0, FACEPLATE_HOLE_CIRCLE_R),
                (0.0, -FACEPLATE_HOLE_CIRCLE_R),
            ]
        )
        .circle(FACEPLATE_MOUNT_HOLE_R)
        .extrude(FACEPLATE_T + 0.004)
        .translate((0.0, -((FACEPLATE_T + 0.004) / 2.0), 0.0))
    )
    return disk.cut(center_hole.union(mounting_holes))


def _make_stop_pad():
    return cq.Workplane("XY").box(STOP_PAD_W, STOP_PAD_T, STOP_PAD_H).translate(
        (
            0.0,
            -0.018,
            STOP_PAD_Z,
        )
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[idx] + maxs[idx]) / 2.0 for idx in range(3))


def _aabb_size(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple(maxs[idx] - mins[idx] for idx in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_rotary_trunnion_fixture")

    model.material("machine_base", rgba=(0.22, 0.23, 0.25, 1.0))
    model.material("bearing_steel", rgba=(0.69, 0.71, 0.74, 1.0))
    model.material("fixture_orange", rgba=(0.83, 0.37, 0.11, 1.0))
    model.material("tool_steel", rgba=(0.80, 0.82, 0.85, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        mesh_from_cadquery(_make_pedestal_shape(), "pedestal_body"),
        material="machine_base",
        name="pedestal_body",
    )

    platter = model.part("platter")
    platter.visual(
        mesh_from_cadquery(_make_platter_shape(), "platter_body"),
        material="bearing_steel",
        name="platter_body",
    )

    trunnion_frame = model.part("trunnion_frame")
    trunnion_frame.visual(
        mesh_from_cadquery(_make_frame_base(), "frame_base"),
        material="fixture_orange",
        name="frame_base",
    )
    trunnion_frame.visual(
        mesh_from_cadquery(_make_cheek(CHEEK_X_CENTER), "left_cheek"),
        material="fixture_orange",
        name="left_cheek",
    )
    trunnion_frame.visual(
        mesh_from_cadquery(_make_cheek(-CHEEK_X_CENTER), "right_cheek"),
        material="fixture_orange",
        name="right_cheek",
    )

    faceplate = model.part("faceplate")
    faceplate.visual(
        mesh_from_cadquery(_make_tilt_shaft(), "tilt_shaft"),
        material="bearing_steel",
        name="tilt_shaft",
    )
    faceplate.visual(
        mesh_from_cadquery(_make_trunnion_hub(), "trunnion_hub"),
        material="bearing_steel",
        name="trunnion_hub",
    )
    faceplate.visual(
        mesh_from_cadquery(_make_bearing_collars(), "bearing_collars"),
        material="bearing_steel",
        name="bearing_collars",
    )
    faceplate.visual(
        mesh_from_cadquery(_make_faceplate_disk(), "faceplate_disk"),
        material="tool_steel",
        name="faceplate_disk",
    )
    faceplate.visual(
        mesh_from_cadquery(_make_stop_pad(), "top_stop_pad"),
        material="tool_steel",
        name="top_stop_pad",
    )

    model.articulation(
        "pedestal_to_platter",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=platter,
        origin=Origin(xyz=(0.0, 0.0, PEDESTAL_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.2,
            lower=-pi,
            upper=pi,
        ),
    )
    model.articulation(
        "platter_to_frame",
        ArticulationType.FIXED,
        parent=platter,
        child=trunnion_frame,
        origin=Origin(xyz=(0.0, 0.0, PLATTER_TOTAL_H)),
    )
    model.articulation(
        "frame_to_faceplate",
        ArticulationType.REVOLUTE,
        parent=trunnion_frame,
        child=faceplate,
        origin=Origin(xyz=(0.0, 0.0, TRUNNION_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.4,
            lower=-1.15,
            upper=1.15,
        ),
    )

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

    pedestal = object_model.get_part("pedestal")
    platter = object_model.get_part("platter")
    trunnion_frame = object_model.get_part("trunnion_frame")
    faceplate = object_model.get_part("faceplate")

    platter_joint = object_model.get_articulation("pedestal_to_platter")
    tilt_joint = object_model.get_articulation("frame_to_faceplate")

    ctx.check(
        "fixture parts exist",
        all(part is not None for part in (pedestal, platter, trunnion_frame, faceplate)),
    )
    ctx.check(
        "primary articulations exist",
        platter_joint is not None and tilt_joint is not None,
    )

    with ctx.pose({platter_joint: 0.0, tilt_joint: 0.0}):
        ctx.expect_gap(
            platter,
            pedestal,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            name="platter seats on pedestal head",
        )
        ctx.expect_overlap(
            platter,
            pedestal,
            axes="xy",
            min_overlap=0.18,
            name="platter stays centered over pedestal",
        )
        ctx.expect_gap(
            trunnion_frame,
            platter,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            name="trunnion frame mounts flush to platter",
        )
        ctx.expect_overlap(
            trunnion_frame,
            platter,
            axes="xy",
            min_overlap=0.18,
            name="frame base sits securely on platter",
        )

        left_cheek_aabb = ctx.part_element_world_aabb(trunnion_frame, elem="left_cheek")
        right_cheek_aabb = ctx.part_element_world_aabb(trunnion_frame, elem="right_cheek")
        disk_aabb = ctx.part_element_world_aabb(faceplate, elem="faceplate_disk")
        shaft_aabb = ctx.part_element_world_aabb(faceplate, elem="tilt_shaft")

        left_inner_x = left_cheek_aabb[0][0] if left_cheek_aabb is not None else None
        right_inner_x = right_cheek_aabb[1][0] if right_cheek_aabb is not None else None
        disk_min_x = disk_aabb[0][0] if disk_aabb is not None else None
        disk_max_x = disk_aabb[1][0] if disk_aabb is not None else None
        shaft_min_x = shaft_aabb[0][0] if shaft_aabb is not None else None
        shaft_max_x = shaft_aabb[1][0] if shaft_aabb is not None else None

        ctx.check(
            "faceplate disk stays between the support cheeks",
            left_inner_x is not None
            and right_inner_x is not None
            and disk_min_x is not None
            and disk_max_x is not None
            and disk_max_x < left_inner_x - 0.010
            and disk_min_x > right_inner_x + 0.010,
            details=(
                f"left_inner_x={left_inner_x}, right_inner_x={right_inner_x}, "
                f"disk_min_x={disk_min_x}, disk_max_x={disk_max_x}"
            ),
        )
        ctx.check(
            "tilt shaft spans both bearing cheeks",
            left_inner_x is not None
            and right_inner_x is not None
            and shaft_min_x is not None
            and shaft_max_x is not None
            and shaft_min_x < right_inner_x - 0.010
            and shaft_max_x > left_inner_x + 0.010,
            details=(
                f"shaft_min_x={shaft_min_x}, shaft_max_x={shaft_max_x}, "
                f"left_inner_x={left_inner_x}, right_inner_x={right_inner_x}"
            ),
        )
        ctx.expect_contact(
            faceplate,
            trunnion_frame,
            elem_a="bearing_collars",
            name="bearing collars physically support the tilting faceplate",
        )

    left_rest_center = _aabb_center(
        ctx.part_element_world_aabb(trunnion_frame, elem="left_cheek")
    )
    with ctx.pose({platter_joint: pi / 2.0, tilt_joint: 0.0}):
        left_rotated_center = _aabb_center(
            ctx.part_element_world_aabb(trunnion_frame, elem="left_cheek")
        )
    ctx.check(
        "platter positive motion rotates the trunnion frame about vertical z",
        left_rest_center is not None
        and left_rotated_center is not None
        and left_rest_center[0] > 0.10
        and abs(left_rotated_center[0]) < 0.04
        and left_rotated_center[1] > 0.10,
        details=f"rest={left_rest_center}, rotated={left_rotated_center}",
    )

    disk_rest_size = _aabb_size(ctx.part_element_world_aabb(faceplate, elem="faceplate_disk"))
    pad_rest_center = _aabb_center(ctx.part_element_world_aabb(faceplate, elem="top_stop_pad"))
    with ctx.pose({tilt_joint: 0.9}):
        disk_tilted_size = _aabb_size(
            ctx.part_element_world_aabb(faceplate, elem="faceplate_disk")
        )
        pad_tilted_center = _aabb_center(
            ctx.part_element_world_aabb(faceplate, elem="top_stop_pad")
        )
        ctx.expect_gap(
            faceplate,
            platter,
            axis="z",
            min_gap=0.075,
            name="tilted faceplate clears the rotary platter",
        )
    ctx.check(
        "faceplate tilts on a horizontal x axis",
        disk_rest_size is not None
        and disk_tilted_size is not None
        and pad_rest_center is not None
        and pad_tilted_center is not None
        and disk_tilted_size[1] > disk_rest_size[1] + 0.10
        and disk_tilted_size[2] < disk_rest_size[2] - 0.05
        and abs(pad_tilted_center[0] - pad_rest_center[0]) < 0.01
        and pad_tilted_center[1] < pad_rest_center[1] - 0.03,
        details=(
            f"disk_rest_size={disk_rest_size}, disk_tilted_size={disk_tilted_size}, "
            f"pad_rest_center={pad_rest_center}, pad_tilted_center={pad_tilted_center}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
