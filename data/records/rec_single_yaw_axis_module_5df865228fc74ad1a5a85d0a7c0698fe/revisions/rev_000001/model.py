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


FRAME_LENGTH = 0.40
FRAME_WIDTH = 0.22
RAIL_WIDTH = 0.045
FOOT_HEIGHT = 0.024
CROSS_MEMBER_THICKNESS = 0.038
CROSS_MEMBER_SPAN = 0.15
SADDLE_LENGTH = 0.18
SADDLE_WIDTH = 0.12
SADDLE_THICKNESS = 0.014

SUPPORT_BASE_LENGTH = 0.16
SUPPORT_SHOULDER_LENGTH = 0.10
SUPPORT_HEAD_LENGTH = 0.072
SUPPORT_WIDTH = 0.13
SUPPORT_SHOULDER_Z = 0.080
SUPPORT_TOP_Z = 0.112

HUB_RADIUS = 0.050
HUB_HEIGHT = 0.026
HUB_FLANGE_RADIUS = 0.070
HUB_FLANGE_HEIGHT = 0.010
JOINT_Z = SUPPORT_TOP_Z + HUB_HEIGHT + HUB_FLANGE_HEIGHT

PLATE_LENGTH = 0.19
PLATE_WIDTH = 0.14
PLATE_THICKNESS = 0.014
COLLAR_RADIUS = 0.048
COLLAR_HEIGHT = 0.012
RIB_LENGTH = 0.090
RIB_WIDTH = 0.022
RIB_HEIGHT = 0.014


def _build_ground_frame_shape() -> cq.Workplane:
    rail_offset_y = FRAME_WIDTH / 2.0 - RAIL_WIDTH / 2.0
    cross_offset_x = FRAME_LENGTH / 2.0 - CROSS_MEMBER_THICKNESS / 2.0 - 0.030

    rail = (
        cq.Workplane("XY")
        .box(FRAME_LENGTH, RAIL_WIDTH, FOOT_HEIGHT)
        .edges("|Z")
        .fillet(0.006)
    )
    left_rail = rail.translate((0.0, rail_offset_y, FOOT_HEIGHT / 2.0))
    right_rail = rail.translate((0.0, -rail_offset_y, FOOT_HEIGHT / 2.0))

    cross_member = (
        cq.Workplane("XY")
        .box(CROSS_MEMBER_THICKNESS, CROSS_MEMBER_SPAN, FOOT_HEIGHT)
        .edges("|Z")
        .fillet(0.004)
    )
    front_cross = cross_member.translate((cross_offset_x, 0.0, FOOT_HEIGHT / 2.0))
    rear_cross = cross_member.translate((-cross_offset_x, 0.0, FOOT_HEIGHT / 2.0))

    saddle = (
        cq.Workplane("XY")
        .box(SADDLE_LENGTH, SADDLE_WIDTH, SADDLE_THICKNESS)
        .edges("|Z")
        .fillet(0.003)
        .translate((0.0, 0.0, FOOT_HEIGHT + SADDLE_THICKNESS / 2.0 - 0.002))
    )

    support_mass = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-SUPPORT_BASE_LENGTH / 2.0, FOOT_HEIGHT - 0.002),
                (SUPPORT_BASE_LENGTH / 2.0, FOOT_HEIGHT - 0.002),
                (SUPPORT_SHOULDER_LENGTH / 2.0, SUPPORT_SHOULDER_Z),
                (SUPPORT_HEAD_LENGTH / 2.0, SUPPORT_TOP_Z),
                (-SUPPORT_HEAD_LENGTH / 2.0, SUPPORT_TOP_Z),
                (-SUPPORT_SHOULDER_LENGTH / 2.0, SUPPORT_SHOULDER_Z),
            ]
        )
        .close()
        .extrude(SUPPORT_WIDTH / 2.0, both=True)
    )

    hub_body = (
        cq.Workplane("XY")
        .circle(HUB_RADIUS)
        .extrude(HUB_HEIGHT)
        .translate((0.0, 0.0, SUPPORT_TOP_Z))
    )
    hub_flange = (
        cq.Workplane("XY")
        .circle(HUB_FLANGE_RADIUS)
        .extrude(HUB_FLANGE_HEIGHT)
        .translate((0.0, 0.0, SUPPORT_TOP_Z + HUB_HEIGHT))
    )

    frame = left_rail.union(right_rail)
    frame = frame.union(front_cross).union(rear_cross).union(saddle)
    frame = frame.union(support_mass).union(hub_body).union(hub_flange)
    return frame


def _build_carried_plate_shape() -> cq.Workplane:
    collar = cq.Workplane("XY").circle(COLLAR_RADIUS).extrude(COLLAR_HEIGHT)
    plate = (
        cq.Workplane("XY")
        .box(PLATE_LENGTH, PLATE_WIDTH, PLATE_THICKNESS)
        .edges("|Z")
        .fillet(0.010)
        .translate((0.0, 0.0, COLLAR_HEIGHT + PLATE_THICKNESS / 2.0))
    )
    rib = cq.Workplane("XY").box(RIB_LENGTH, RIB_WIDTH, RIB_HEIGHT)
    front_rib = rib.translate((0.0, 0.035, COLLAR_HEIGHT + RIB_HEIGHT / 2.0))
    rear_rib = rib.translate((0.0, -0.035, COLLAR_HEIGHT + RIB_HEIGHT / 2.0))

    carried_plate = collar.union(plate).union(front_rib).union(rear_rib)
    return carried_plate


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_slung_yaw_carriage")

    model.material("frame_gray", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("plate_aluminum", rgba=(0.73, 0.75, 0.78, 1.0))

    ground_frame = model.part("ground_frame")
    ground_frame.visual(
        mesh_from_cadquery(_build_ground_frame_shape(), "ground_frame"),
        material="frame_gray",
        name="frame_body",
    )

    carried_plate = model.part("carried_plate")
    carried_plate.visual(
        mesh_from_cadquery(_build_carried_plate_shape(), "carried_plate"),
        material="plate_aluminum",
        name="carrier_plate",
    )

    model.articulation(
        "frame_to_carried_plate",
        ArticulationType.REVOLUTE,
        parent=ground_frame,
        child=carried_plate,
        origin=Origin(xyz=(0.0, 0.0, JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.5,
            lower=-2.4,
            upper=2.4,
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

    ground_frame = object_model.get_part("ground_frame")
    carried_plate = object_model.get_part("carried_plate")
    yaw_joint = object_model.get_articulation("frame_to_carried_plate")

    limits = yaw_joint.motion_limits
    ctx.check(
        "yaw axis is vertical",
        tuple(round(v, 6) for v in yaw_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={yaw_joint.axis}",
    )
    ctx.check(
        "yaw joint has symmetric turning range",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower < 0.0 < limits.upper,
        details=f"limits={limits}",
    )
    ctx.expect_origin_distance(
        carried_plate,
        ground_frame,
        axes="xy",
        min_dist=0.0,
        max_dist=1e-6,
        name="carried plate stays centered over the hub axis",
    )
    ctx.expect_gap(
        carried_plate,
        ground_frame,
        axis="z",
        min_gap=0.0,
        max_gap=0.0005,
        name="carried plate seats directly on the hub",
    )
    ctx.expect_overlap(
        carried_plate,
        ground_frame,
        axes="xy",
        min_overlap=0.09,
        name="carried plate remains supported by the hub footprint",
    )

    rest_aabb = ctx.part_element_world_aabb(carried_plate, elem="carrier_plate")
    with ctx.pose({yaw_joint: pi / 2.0}):
        turned_aabb = ctx.part_element_world_aabb(carried_plate, elem="carrier_plate")

    rest_dims = (
        None
        if rest_aabb is None
        else tuple(rest_aabb[1][i] - rest_aabb[0][i] for i in range(3))
    )
    turned_dims = (
        None
        if turned_aabb is None
        else tuple(turned_aabb[1][i] - turned_aabb[0][i] for i in range(3))
    )
    ctx.check(
        "carried plate visibly yaws about the vertical hub",
        rest_dims is not None
        and turned_dims is not None
        and rest_dims[0] > rest_dims[1] + 0.02
        and turned_dims[1] > turned_dims[0] + 0.02,
        details=f"rest_dims={rest_dims}, turned_dims={turned_dims}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
