from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

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
    mesh_from_cadquery,
)


BASE_DEPTH = 0.220
BASE_WIDTH = 0.300
BASE_THICK = 0.010

SUPPORT_LENGTH = 0.110
SUPPORT_WIDTH = 0.020
SUPPORT_SLOT_WIDTH = 0.010
SUPPORT_HEIGHT = 0.042
LOWER_PIVOT_X = -0.060
LOWER_PIVOT_Z = 0.032

LINK_LENGTH = 0.160
LINK_THICK = 0.008
LINK_BOSS_RADIUS = 0.018
LINK_WEB_HEIGHT = 0.024
PIN_RADIUS = 0.006
PIN_LENGTH = 0.010

TRAY_DEPTH = 0.260
TRAY_WIDTH = 0.270
TRAY_THICK = 0.008
TRAY_UNDERSIDE_Z = 0.018
TRAY_REAR_OVERHANG = 0.055
TRAY_SIDE_MARGIN = 0.018
TRAY_FRONT_STOP_THICK = 0.010
TRAY_FRONT_STOP_WIDTH = 0.220
TRAY_FRONT_STOP_HEIGHT = 0.012
TRAY_PAD_DEPTH = 0.200
TRAY_PAD_WIDTH = 0.190
TRAY_PAD_THICK = 0.0015

PIVOT_Y = 0.117
PIVOT_SPACING = 2.0 * PIVOT_Y
MOUNT_LENGTH = 0.024
MOUNT_WIDTH = 0.020
MOUNT_SLOT_WIDTH = 0.010
MOUNT_BOTTOM_Z = -0.014
MOUNT_HEIGHT = TRAY_UNDERSIDE_Z - MOUNT_BOTTOM_Z
REAR_RIB_LENGTH = 0.020
REAR_RIB_HEIGHT = 0.012

REST_LINK_PITCH = 0.90
TRAY_PITCH = 0.26
RAISED_POSE = 0.44


def _base_plate_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(BASE_DEPTH, BASE_WIDTH, BASE_THICK, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.010)
    )


def _support_shape(y_center: float) -> cq.Workplane:
    support = (
        cq.Workplane("XY")
        .box(SUPPORT_LENGTH, SUPPORT_WIDTH, SUPPORT_HEIGHT, centered=(True, True, False))
        .translate((LOWER_PIVOT_X, y_center, BASE_THICK))
    )
    slot = (
        cq.Workplane("XY")
        .box(
            SUPPORT_LENGTH + 0.006,
            SUPPORT_SLOT_WIDTH,
            SUPPORT_HEIGHT + 0.004,
            centered=(True, True, False),
        )
        .translate((LOWER_PIVOT_X, y_center, BASE_THICK - 0.002))
    )
    return support.cut(slot)


def _link_arm_shape() -> cq.Workplane:
    lower_boss = cq.Workplane("XZ").circle(LINK_BOSS_RADIUS).extrude(LINK_THICK / 2.0, both=True)
    upper_boss = (
        cq.Workplane("XZ")
        .circle(LINK_BOSS_RADIUS)
        .extrude(LINK_THICK / 2.0, both=True)
        .translate((LINK_LENGTH, 0.0, 0.0))
    )
    web = (
        cq.Workplane("XZ")
        .rect(LINK_LENGTH, LINK_WEB_HEIGHT)
        .extrude(LINK_THICK / 2.0, both=True)
        .translate((LINK_LENGTH / 2.0, 0.0, 0.0))
    )
    arm = lower_boss.union(upper_boss).union(web)
    lightening_slot = (
        cq.Workplane("XZ")
        .slot2D(LINK_LENGTH * 0.54, LINK_WEB_HEIGHT * 0.46)
        .extrude(LINK_THICK, both=True)
        .translate((LINK_LENGTH / 2.0, 0.0, 0.0))
    )
    return arm.cut(lightening_slot)


def _tray_plate_shape() -> cq.Workplane:
    center_x = (TRAY_DEPTH / 2.0) - TRAY_REAR_OVERHANG
    center_y = PIVOT_Y
    plate = (
        cq.Workplane("XY")
        .box(TRAY_DEPTH, TRAY_WIDTH, TRAY_THICK, centered=(True, True, False))
        .translate((center_x, center_y, TRAY_UNDERSIDE_Z))
        .edges("|Z")
        .fillet(0.010)
    )
    return (
        plate.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .center(0.012, 0.0)
        .rarray(0.042, 0.050, 4, 3)
        .slot2D(0.024, 0.008, angle=90)
        .cutBlind(-(TRAY_THICK + 0.003))
    )


def _tray_front_stop_shape() -> cq.Workplane:
    front_edge_x = TRAY_DEPTH - TRAY_REAR_OVERHANG
    center_y = PIVOT_Y
    return (
        cq.Workplane("XY")
        .box(
            TRAY_FRONT_STOP_THICK,
            TRAY_FRONT_STOP_WIDTH,
            TRAY_FRONT_STOP_HEIGHT,
            centered=(True, True, False),
        )
        .translate(
            (
                front_edge_x - (TRAY_FRONT_STOP_THICK / 2.0),
                center_y,
                TRAY_UNDERSIDE_Z + TRAY_THICK,
            )
        )
    )


def _tray_pad_shape() -> cq.Workplane:
    center_x = (TRAY_DEPTH / 2.0) - TRAY_REAR_OVERHANG + 0.008
    center_y = PIVOT_Y
    return (
        cq.Workplane("XY")
        .box(TRAY_PAD_DEPTH, TRAY_PAD_WIDTH, TRAY_PAD_THICK, centered=(True, True, False))
        .translate((center_x, center_y, TRAY_UNDERSIDE_Z + TRAY_THICK))
    )


def _tray_mount_shape(y_center: float) -> cq.Workplane:
    mount = (
        cq.Workplane("XY")
        .box(MOUNT_LENGTH, MOUNT_WIDTH, MOUNT_HEIGHT, centered=(True, True, False))
        .translate((0.0, y_center, MOUNT_BOTTOM_Z))
    )
    slot = (
        cq.Workplane("XY")
        .box(
            MOUNT_LENGTH + 0.004,
            MOUNT_SLOT_WIDTH,
            MOUNT_HEIGHT + 0.004,
            centered=(True, True, False),
        )
        .translate((0.0, y_center, MOUNT_BOTTOM_Z - 0.002))
    )
    return mount.cut(slot)


def _tray_rear_rib_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(REAR_RIB_LENGTH, PIVOT_SPACING, REAR_RIB_HEIGHT, centered=(True, True, False))
        .translate((-0.010, PIVOT_Y, TRAY_UNDERSIDE_Z - REAR_RIB_HEIGHT))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_laptop_stand")

    model.material("graphite", rgba=(0.18, 0.20, 0.23, 1.0))
    model.material("silver", rgba=(0.80, 0.82, 0.84, 1.0))
    model.material("black", rgba=(0.09, 0.09, 0.10, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_plate_shape(), "base_plate"),
        material="graphite",
        name="base_plate",
    )
    base.visual(
        mesh_from_cadquery(_support_shape(-PIVOT_Y), "left_support"),
        material="graphite",
        name="left_support",
    )
    base.visual(
        mesh_from_cadquery(_support_shape(PIVOT_Y), "right_support"),
        material="graphite",
        name="right_support",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_DEPTH, BASE_WIDTH, BASE_THICK + SUPPORT_HEIGHT)),
        mass=2.0,
        origin=Origin(xyz=(0.0, 0.0, (BASE_THICK + SUPPORT_HEIGHT) / 2.0)),
    )

    left_link = model.part("left_link")
    left_link.visual(
        mesh_from_cadquery(_link_arm_shape(), "left_link_arm"),
        material="black",
        name="arm",
    )
    left_link.visual(
        Cylinder(radius=PIN_RADIUS, length=PIN_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="graphite",
        name="lower_pin",
    )
    left_link.visual(
        Cylinder(radius=PIN_RADIUS, length=PIN_LENGTH),
        origin=Origin(xyz=(LINK_LENGTH, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="graphite",
        name="upper_pin",
    )
    left_link.inertial = Inertial.from_geometry(
        Box((LINK_LENGTH + 0.020, LINK_THICK, LINK_BOSS_RADIUS * 2.0)),
        mass=0.28,
        origin=Origin(xyz=(LINK_LENGTH / 2.0, 0.0, 0.0)),
    )

    right_link = model.part("right_link")
    right_link.visual(
        mesh_from_cadquery(_link_arm_shape(), "right_link_arm"),
        material="black",
        name="arm",
    )
    right_link.visual(
        Cylinder(radius=PIN_RADIUS, length=PIN_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="graphite",
        name="lower_pin",
    )
    right_link.visual(
        Cylinder(radius=PIN_RADIUS, length=PIN_LENGTH),
        origin=Origin(xyz=(LINK_LENGTH, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="graphite",
        name="upper_pin",
    )
    right_link.inertial = Inertial.from_geometry(
        Box((LINK_LENGTH + 0.020, LINK_THICK, LINK_BOSS_RADIUS * 2.0)),
        mass=0.28,
        origin=Origin(xyz=(LINK_LENGTH / 2.0, 0.0, 0.0)),
    )

    tray = model.part("tray")
    tray.visual(
        mesh_from_cadquery(_tray_plate_shape(), "tray_plate"),
        material="silver",
        name="plate",
    )
    tray.visual(
        mesh_from_cadquery(_tray_front_stop_shape(), "tray_front_stop"),
        material="silver",
        name="front_stop",
    )
    tray.visual(
        mesh_from_cadquery(_tray_pad_shape(), "tray_pad"),
        material="black",
        name="pad",
    )
    tray.visual(
        mesh_from_cadquery(_tray_mount_shape(0.0), "left_mount"),
        material="silver",
        name="left_mount",
    )
    tray.visual(
        mesh_from_cadquery(_tray_mount_shape(PIVOT_SPACING), "right_mount"),
        material="silver",
        name="right_mount",
    )
    tray.visual(
        mesh_from_cadquery(_tray_rear_rib_shape(), "tray_rear_rib"),
        material="silver",
        name="rear_rib",
    )
    tray.inertial = Inertial.from_geometry(
        Box((TRAY_DEPTH, TRAY_WIDTH, TRAY_UNDERSIDE_Z + TRAY_THICK + TRAY_FRONT_STOP_HEIGHT)),
        mass=0.95,
        origin=Origin(
            xyz=(
                (TRAY_DEPTH / 2.0) - TRAY_REAR_OVERHANG,
                PIVOT_Y,
                (TRAY_UNDERSIDE_Z + TRAY_THICK + TRAY_FRONT_STOP_HEIGHT) / 2.0,
            )
        ),
    )

    model.articulation(
        "base_to_left_link",
        ArticulationType.REVOLUTE,
        parent=base,
        child=left_link,
        origin=Origin(xyz=(LOWER_PIVOT_X, -PIVOT_Y, LOWER_PIVOT_Z), rpy=(0.0, -REST_LINK_PITCH, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=0.68),
    )
    model.articulation(
        "base_to_right_link",
        ArticulationType.REVOLUTE,
        parent=base,
        child=right_link,
        origin=Origin(xyz=(LOWER_PIVOT_X, PIVOT_Y, LOWER_PIVOT_Z), rpy=(0.0, -REST_LINK_PITCH, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=0.68),
    )
    model.articulation(
        "left_link_to_tray",
        ArticulationType.REVOLUTE,
        parent=left_link,
        child=tray,
        origin=Origin(xyz=(LINK_LENGTH, 0.0, 0.0), rpy=(0.0, REST_LINK_PITCH - TRAY_PITCH, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=1.2, lower=0.0, upper=0.68),
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

    base = object_model.get_part("base")
    tray = object_model.get_part("tray")
    left_link = object_model.get_part("left_link")
    right_link = object_model.get_part("right_link")

    left_lower = object_model.get_articulation("base_to_left_link")
    right_lower = object_model.get_articulation("base_to_right_link")
    tray_joint = object_model.get_articulation("left_link_to_tray")

    ctx.expect_gap(
        tray,
        base,
        axis="z",
        positive_elem="plate",
        negative_elem="base_plate",
        min_gap=0.095,
        name="tray clears the broad base plate at rest",
    )
    ctx.expect_within(
        tray,
        base,
        axes="y",
        inner_elem="plate",
        outer_elem="base_plate",
        margin=0.0,
        name="tray stays centered within the base width at rest",
    )
    ctx.expect_overlap(
        tray,
        base,
        axes="x",
        elem_a="plate",
        elem_b="base_plate",
        min_overlap=0.120,
        name="tray keeps meaningful front to back support over the base",
    )

    def aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) / 2.0 for i in range(3))

    rest_tray_pos = ctx.part_world_position(tray)

    with ctx.pose({left_lower: RAISED_POSE, right_lower: RAISED_POSE, tray_joint: RAISED_POSE}):
        raised_tray_pos = ctx.part_world_position(tray)
        left_upper = aabb_center(ctx.part_element_world_aabb(left_link, elem="upper_pin"))
        right_upper = aabb_center(ctx.part_element_world_aabb(right_link, elem="upper_pin"))
        right_mount = aabb_center(ctx.part_element_world_aabb(tray, elem="right_mount"))

        ctx.check(
            "tray rises when the matched links rotate",
            rest_tray_pos is not None
            and raised_tray_pos is not None
            and raised_tray_pos[2] > rest_tray_pos[2] + 0.028,
            details=f"rest={rest_tray_pos}, raised={raised_tray_pos}",
        )
        ctx.check(
            "left and right lift links stay matched",
            left_upper is not None
            and right_upper is not None
            and abs(left_upper[0] - right_upper[0]) <= 0.0015
            and abs(left_upper[2] - right_upper[2]) <= 0.0015
            and abs((right_upper[1] - left_upper[1]) - PIVOT_SPACING) <= 0.0015,
            details=f"left_upper={left_upper}, right_upper={right_upper}",
        )
        ctx.check(
            "right lift link stays aligned to the tray-side mount",
            right_upper is not None
            and right_mount is not None
            and abs(right_upper[0] - right_mount[0]) <= 0.006
            and abs(right_upper[1] - right_mount[1]) <= 0.0015
            and abs(right_upper[2] - right_mount[2]) <= 0.006,
            details=f"right_upper={right_upper}, right_mount={right_mount}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
