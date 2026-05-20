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


def _annulus(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    """A simple vertical ring, authored in meters with its bottom on z=0."""
    return cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(height)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_rotary_stage")

    cast_iron = model.material("dark_cast_iron", rgba=(0.10, 0.11, 0.12, 1.0))
    painted_metal = model.material("painted_graphite", rgba=(0.18, 0.19, 0.20, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.73, 0.70, 1.0))
    black_rubber = model.material("black_rubber_seal", rgba=(0.01, 0.01, 0.012, 1.0))
    fastener_steel = model.material("dark_fastener_steel", rgba=(0.04, 0.045, 0.05, 1.0))
    safety_yellow = model.material("machined_bracket_yellow", rgba=(0.86, 0.62, 0.10, 1.0))

    lower = model.part("lower_housing")
    lower.visual(
        Cylinder(radius=0.34, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=cast_iron,
        name="floor_flange",
    )
    lower.visual(
        Cylinder(radius=0.255, length=0.145),
        origin=Origin(xyz=(0.0, 0.0, 0.1175)),
        material=painted_metal,
        name="round_housing",
    )
    lower.visual(
        mesh_from_cadquery(_annulus(0.265, 0.070, 0.020), "top_cover_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.190)),
        material=cast_iron,
        name="top_cover_ring",
    )
    lower.visual(
        mesh_from_cadquery(_annulus(0.112, 0.050, 0.040), "bearing_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.210)),
        material=brushed_aluminum,
        name="bearing_ring",
    )
    lower.visual(
        mesh_from_cadquery(_annulus(0.112, 0.084, 0.003), "stationary_seal_groove"),
        origin=Origin(xyz=(0.0, 0.0, 0.250)),
        material=black_rubber,
        name="stationary_seal_groove",
    )
    lower.visual(
        mesh_from_cadquery(_annulus(0.258, 0.248, 0.010), "side_seal_band"),
        origin=Origin(xyz=(0.0, 0.0, 0.164)),
        material=black_rubber,
        name="side_seal_band",
    )

    for index, angle in enumerate([i * math.tau / 8.0 for i in range(8)]):
        x = 0.282 * math.cos(angle)
        y = 0.282 * math.sin(angle)
        lower.visual(
            Cylinder(radius=0.018, length=0.008),
            origin=Origin(xyz=(x, y, 0.049)),
            material=fastener_steel,
            name=f"flange_screw_{index}",
        )

    platform = model.part("platform")
    platform.visual(
        Cylinder(radius=0.220, length=0.042),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=brushed_aluminum,
        name="stage_disk",
    )
    platform.visual(
        Cylinder(radius=0.106, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=black_rubber,
        name="rotating_seal_lip",
    )
    platform.visual(
        Cylinder(radius=0.078, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
        material=brushed_aluminum,
        name="rotating_hub",
    )
    platform.visual(
        Cylinder(radius=0.042, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=fastener_steel,
        name="yaw_shaft",
    )

    for index, angle in enumerate([i * math.tau / 6.0 for i in range(6)]):
        x = 0.172 * math.cos(angle)
        y = 0.172 * math.sin(angle)
        platform.visual(
            Cylinder(radius=0.012, length=0.006),
            origin=Origin(xyz=(x, y, 0.055)),
            material=fastener_steel,
            name=f"platform_screw_{index}",
        )

    # A short fixed equipment bracket rides on the rotating table.  It is part of
    # the platform link, with its base plate bolted down and gussets tied into the
    # upright so it reads as carried tooling rather than a loose accessory.
    platform.visual(
        Box((0.160, 0.115, 0.018)),
        origin=Origin(xyz=(0.070, 0.0, 0.061)),
        material=safety_yellow,
        name="bracket_base",
    )
    platform.visual(
        Box((0.018, 0.125, 0.094)),
        origin=Origin(xyz=(0.143, 0.0, 0.117)),
        material=safety_yellow,
        name="bracket_upright",
    )
    for suffix, y in (("0", -0.045), ("1", 0.045)):
        platform.visual(
            Box((0.085, 0.010, 0.012)),
            origin=Origin(xyz=(0.105, y, 0.101), rpy=(0.0, -0.72, 0.0)),
            material=safety_yellow,
            name=f"bracket_gusset_{suffix}",
        )
    for suffix, y in (("0", -0.032), ("1", 0.032)):
        platform.visual(
            Cylinder(radius=0.010, length=0.005),
            origin=Origin(xyz=(0.052, y, 0.0725)),
            material=fastener_steel,
            name=f"bracket_base_screw_{suffix}",
        )
        platform.visual(
            Cylinder(radius=0.012, length=0.004),
            origin=Origin(xyz=(0.132, y, 0.123), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=fastener_steel,
            name=f"bracket_face_bolt_{suffix}",
        )

    model.articulation(
        "housing_to_platform",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=platform,
        origin=Origin(xyz=(0.0, 0.0, 0.258)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-math.pi,
            upper=math.pi,
            effort=80.0,
            velocity=1.5,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower = object_model.get_part("lower_housing")
    platform = object_model.get_part("platform")
    yaw = object_model.get_articulation("housing_to_platform")

    ctx.expect_gap(
        platform,
        lower,
        axis="z",
        positive_elem="stage_disk",
        negative_elem="bearing_ring",
        min_gap=0.015,
        max_gap=0.025,
        name="stage has practical bearing clearance",
    )
    ctx.expect_overlap(
        platform,
        lower,
        axes="z",
        elem_a="yaw_shaft",
        elem_b="bearing_ring",
        min_overlap=0.020,
        name="yaw shaft remains supported in bearing bore",
    )
    ctx.expect_within(
        platform,
        lower,
        axes="xy",
        inner_elem="yaw_shaft",
        outer_elem="bearing_ring",
        margin=0.001,
        name="yaw shaft is centered within the bearing ring envelope",
    )

    rest_position = ctx.part_world_position(platform)
    with ctx.pose({yaw: math.pi / 2.0}):
        turned_position = ctx.part_world_position(platform)
        ctx.expect_gap(
            platform,
            lower,
            axis="z",
            positive_elem="stage_disk",
            negative_elem="bearing_ring",
            min_gap=0.015,
            max_gap=0.025,
            name="quarter-turn platform keeps cover clearance",
        )
        ctx.expect_gap(
            platform,
            lower,
            axis="z",
            positive_elem="bracket_base",
            negative_elem="top_cover_ring",
            min_gap=0.090,
            name="carried bracket stays above stationary cover",
        )

    ctx.check(
        "platform rotates about fixed yaw origin",
        rest_position is not None
        and turned_position is not None
        and abs(rest_position[0] - turned_position[0]) < 1e-6
        and abs(rest_position[1] - turned_position[1]) < 1e-6
        and abs(rest_position[2] - turned_position[2]) < 1e-6,
        details=f"rest={rest_position}, turned={turned_position}",
    )

    return ctx.report()


object_model = build_object_model()
