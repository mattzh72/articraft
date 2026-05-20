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

SLEEVE_HEIGHT = 0.340
FOUNDATION_RADIUS = 0.125
FOUNDATION_HEIGHT = 0.028
SLEEVE_RADIUS = 0.098
COLLAR_RADIUS = 0.112
COLLAR_HEIGHT = 0.026
BORE_RADIUS = 0.084
COLLAR_INNER_RADIUS = 0.078
BORE_DEPTH = 0.290

BOLLARD_RADIUS = 0.072
BOLLARD_LENGTH = 1.000
BOLLARD_CENTER_Z = 0.215
GUIDE_FLANGE_RADIUS = 0.088
GUIDE_FLANGE_HEIGHT = 0.024
GUIDE_FLANGE_CENTER_Z = 0.012
CROWN_RADIUS = 0.079
CROWN_HEIGHT = 0.026
CROWN_CENTER_Z = 0.723
SERVICE_BOSS_RADIUS = 0.031
SERVICE_BOSS_LENGTH = 0.016
SERVICE_BOSS_CENTER_X = 0.078
SERVICE_BOSS_Z = 0.590
BEACON_BASE_RADIUS = 0.032
BEACON_BASE_HEIGHT = 0.008
BEACON_BASE_CENTER_Z = 0.740
BEACON_LENS_RADIUS = 0.024
BEACON_LENS_HEIGHT = 0.010
BEACON_LENS_CENTER_Z = 0.749
HINGE_PAD_CENTER_X = -0.040
HINGE_PAD_CENTER_Z = 0.745
HINGE_AXIS_Z = 0.757
HINGE_LUG_RADIUS = 0.0045
HINGE_LUG_LENGTH = 0.016
HINGE_LUG_Y_OFFSET = 0.018

CAP_DISK_RADIUS = 0.024
CAP_DISK_LENGTH = 0.006
CAP_LOCK_RADIUS = 0.010
CAP_LOCK_LENGTH = 0.005

LIGHT_COVER_LENGTH = 0.090
LIGHT_COVER_WIDTH = 0.054
LIGHT_COVER_THICKNESS = 0.010
LIGHT_COVER_BASE_CENTER_X = 0.059
LIGHT_COVER_BASE_CENTER_Z = 0.012
LIGHT_COVER_ARC_RADIUS = 0.018
LIGHT_COVER_ARC_CENTER_X = 0.060
LIGHT_COVER_ARC_CENTER_Z = 0.022
LIGHT_COVER_BRIDGE_LENGTH = 0.016
LIGHT_COVER_BRIDGE_WIDTH = 0.016
LIGHT_COVER_BRIDGE_HEIGHT = 0.008
LIGHT_COVER_BRIDGE_CENTER_X = 0.008
LIGHT_COVER_BRIDGE_CENTER_Z = 0.004

SLIDE_TRAVEL = 0.180
LIGHT_COVER_OPEN = 1.150


def _sleeve_shape() -> cq.Workplane:
    sleeve = cq.Workplane("XY").circle(SLEEVE_RADIUS).extrude(SLEEVE_HEIGHT)
    foundation = cq.Workplane("XY").circle(FOUNDATION_RADIUS).extrude(FOUNDATION_HEIGHT)
    collar = (
        cq.Workplane("XY")
        .workplane(offset=SLEEVE_HEIGHT - COLLAR_HEIGHT)
        .circle(COLLAR_RADIUS)
        .extrude(COLLAR_HEIGHT)
    )
    body = sleeve.union(foundation).union(collar)

    bore = (
        cq.Workplane("XY")
        .workplane(offset=SLEEVE_HEIGHT - BORE_DEPTH)
        .circle(BORE_RADIUS)
        .extrude(BORE_DEPTH)
    )
    throat = (
        cq.Workplane("XY")
        .workplane(offset=SLEEVE_HEIGHT - COLLAR_HEIGHT)
        .circle(COLLAR_INNER_RADIUS)
        .extrude(COLLAR_HEIGHT + 0.002)
    )
    return body.cut(bore).cut(throat)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_security_post")

    model.material("powder_black", rgba=(0.18, 0.18, 0.19, 1.0))
    model.material("graphite", rgba=(0.26, 0.27, 0.29, 1.0))
    model.material("zinc", rgba=(0.73, 0.74, 0.77, 1.0))
    model.material("amber", rgba=(0.93, 0.63, 0.18, 0.88))
    model.material("smoke", rgba=(0.32, 0.24, 0.16, 0.55))

    sleeve = model.part("sleeve")
    sleeve.visual(
        mesh_from_cadquery(_sleeve_shape(), "sleeve_body"),
        material="graphite",
        name="sleeve_body",
    )

    bollard = model.part("bollard")
    bollard.visual(
        Cylinder(radius=BOLLARD_RADIUS, length=BOLLARD_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, BOLLARD_CENTER_Z)),
        material="powder_black",
        name="upper_shell",
    )
    bollard.visual(
        Cylinder(radius=GUIDE_FLANGE_RADIUS, length=GUIDE_FLANGE_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, GUIDE_FLANGE_CENTER_Z)),
        material="graphite",
        name="guide_flange",
    )
    bollard.visual(
        Cylinder(radius=CROWN_RADIUS, length=CROWN_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, CROWN_CENTER_Z)),
        material="powder_black",
        name="crown",
    )
    bollard.visual(
        Cylinder(radius=SERVICE_BOSS_RADIUS, length=SERVICE_BOSS_LENGTH),
        origin=Origin(
            xyz=(SERVICE_BOSS_CENTER_X, 0.0, SERVICE_BOSS_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material="graphite",
        name="service_boss",
    )
    bollard.visual(
        Cylinder(radius=BEACON_BASE_RADIUS, length=BEACON_BASE_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BEACON_BASE_CENTER_Z)),
        material="graphite",
        name="beacon_base",
    )
    bollard.visual(
        Cylinder(radius=BEACON_LENS_RADIUS, length=BEACON_LENS_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BEACON_LENS_CENTER_Z)),
        material="amber",
        name="beacon_lens",
    )
    bollard.visual(
        Box((0.024, 0.052, 0.018)),
        origin=Origin(xyz=(HINGE_PAD_CENTER_X, 0.0, HINGE_PAD_CENTER_Z)),
        material="graphite",
        name="hinge_pad",
    )
    bollard.visual(
        Cylinder(radius=HINGE_LUG_RADIUS, length=HINGE_LUG_LENGTH),
        origin=Origin(
            xyz=(HINGE_PAD_CENTER_X, -HINGE_LUG_Y_OFFSET, HINGE_AXIS_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material="zinc",
        name="hinge_lug_0",
    )
    bollard.visual(
        Cylinder(radius=HINGE_LUG_RADIUS, length=HINGE_LUG_LENGTH),
        origin=Origin(
            xyz=(HINGE_PAD_CENTER_X, HINGE_LUG_Y_OFFSET, HINGE_AXIS_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material="zinc",
        name="hinge_lug_1",
    )

    model.articulation(
        "sleeve_to_bollard",
        ArticulationType.PRISMATIC,
        parent=sleeve,
        child=bollard,
        origin=Origin(xyz=(0.0, 0.0, SLEEVE_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=200.0,
            velocity=0.20,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )

    service_cap = model.part("service_cap")
    service_cap.visual(
        Cylinder(radius=CAP_DISK_RADIUS, length=CAP_DISK_LENGTH),
        origin=Origin(xyz=(CAP_DISK_LENGTH / 2.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="zinc",
        name="cap_disk",
    )
    service_cap.visual(
        Cylinder(radius=CAP_LOCK_RADIUS, length=CAP_LOCK_LENGTH),
        origin=Origin(
            xyz=(CAP_DISK_LENGTH + CAP_LOCK_LENGTH / 2.0, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material="graphite",
        name="cap_lock",
    )

    model.articulation(
        "bollard_to_service_cap",
        ArticulationType.CONTINUOUS,
        parent=bollard,
        child=service_cap,
        origin=Origin(
            xyz=(SERVICE_BOSS_CENTER_X + SERVICE_BOSS_LENGTH / 2.0, 0.0, SERVICE_BOSS_Z)
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=6.0),
    )

    light_cover = model.part("light_cover")
    light_cover.visual(
        Cylinder(radius=HINGE_LUG_RADIUS, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="zinc",
        name="hinge_barrel",
    )
    light_cover.visual(
        Cylinder(radius=LIGHT_COVER_ARC_RADIUS, length=LIGHT_COVER_WIDTH),
        origin=Origin(
            xyz=(LIGHT_COVER_ARC_CENTER_X, 0.0, LIGHT_COVER_ARC_CENTER_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material="smoke",
        name="hood_arc",
    )
    light_cover.visual(
        Cylinder(radius=LIGHT_COVER_ARC_RADIUS * 0.55, length=LIGHT_COVER_WIDTH * 0.84),
        origin=Origin(
            xyz=(LIGHT_COVER_ARC_CENTER_X + 0.004, 0.0, LIGHT_COVER_ARC_CENTER_Z - 0.002),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material="amber",
        name="hood_lens",
    )
    light_cover.visual(
        Box((LIGHT_COVER_LENGTH, LIGHT_COVER_WIDTH, LIGHT_COVER_THICKNESS)),
        origin=Origin(xyz=(LIGHT_COVER_BASE_CENTER_X, 0.0, LIGHT_COVER_BASE_CENTER_Z)),
        material="smoke",
        name="hood_base",
    )
    light_cover.visual(
        Box(
            (
                LIGHT_COVER_BRIDGE_LENGTH,
                LIGHT_COVER_BRIDGE_WIDTH,
                LIGHT_COVER_BRIDGE_HEIGHT,
            )
        ),
        origin=Origin(
            xyz=(LIGHT_COVER_BRIDGE_CENTER_X, 0.0, LIGHT_COVER_BRIDGE_CENTER_Z)
        ),
        material="smoke",
        name="hood_bridge",
    )

    model.articulation(
        "bollard_to_light_cover",
        ArticulationType.REVOLUTE,
        parent=bollard,
        child=light_cover,
        origin=Origin(xyz=(HINGE_PAD_CENTER_X, 0.0, HINGE_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=LIGHT_COVER_OPEN,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    sleeve = object_model.get_part("sleeve")
    bollard = object_model.get_part("bollard")
    service_cap = object_model.get_part("service_cap")
    light_cover = object_model.get_part("light_cover")
    slide = object_model.get_articulation("sleeve_to_bollard")
    cap_joint = object_model.get_articulation("bollard_to_service_cap")
    cover_joint = object_model.get_articulation("bollard_to_light_cover")

    ctx.expect_within(
        bollard,
        sleeve,
        axes="xy",
        name="bollard stays centered in the sleeve at rest",
    )
    ctx.expect_overlap(
        bollard,
        sleeve,
        axes="z",
        min_overlap=0.280,
        name="collapsed bollard remains deeply inserted in the sleeve",
    )
    ctx.expect_contact(
        bollard,
        sleeve,
        elem_a="guide_flange",
        elem_b="sleeve_body",
        name="bollard seats on the sleeve collar at rest",
    )

    rest_position = ctx.part_world_position(bollard)
    with ctx.pose({slide: SLIDE_TRAVEL}):
        ctx.expect_within(
            bollard,
            sleeve,
            axes="xy",
            name="extended bollard stays centered in the sleeve collar",
        )
        ctx.expect_overlap(
            bollard,
            sleeve,
            axes="z",
            min_overlap=0.100,
            name="extended bollard still retains insertion in the sleeve",
        )
        extended_position = ctx.part_world_position(bollard)

    ctx.check(
        "bollard extends upward from the sleeve",
        rest_position is not None
        and extended_position is not None
        and extended_position[2] > rest_position[2] + 0.150,
        details=f"rest={rest_position}, extended={extended_position}",
    )
    with ctx.pose({cap_joint: 1.350}):
        ctx.expect_contact(
            service_cap,
            bollard,
            elem_a="cap_disk",
            elem_b="service_boss",
            name="service cap stays mounted while it rotates",
        )

    ctx.expect_gap(
        light_cover,
        bollard,
        axis="z",
        positive_elem="hood_base",
        negative_elem="beacon_lens",
        min_gap=0.004,
        max_gap=0.012,
        name="closed light cover hovers just above the warning lens",
    )
    with ctx.pose({cover_joint: LIGHT_COVER_OPEN}):
        ctx.expect_contact(
            light_cover,
            bollard,
            elem_a="hinge_barrel",
            elem_b="hinge_lug_0",
            name="warning light cover stays pinned to the crown hinge",
        )

    return ctx.report()


object_model = build_object_model()
