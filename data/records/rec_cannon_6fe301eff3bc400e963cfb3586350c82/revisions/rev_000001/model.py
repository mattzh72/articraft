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
import math
import cadquery as cq


def build_barrel_body() -> cq.Workplane:
    bore_radius = 0.15
    pts = [
        (-1.0, bore_radius),
        (-1.0, 0.35),
        (0.5, 0.35),
        (0.5, 0.25),
        (2.8, 0.25),
        (2.8, 0.28),
        (3.0, 0.28),
        (3.0, bore_radius)
    ]
    return cq.Workplane("XZ").polyline(pts).close().revolve(360, (-1, 0, 0), (1, 0, 0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="coastal_cannon")

    barbette = model.part("barbette")
    barbette.visual(
        Cylinder(radius=2.0, length=1.0),
        origin=Origin(xyz=(0.0, 0.0, 0.5)),
        name="concrete_base",
    )

    platform = model.part("gun_platform")
    platform.visual(
        Box((3.0, 2.0, 0.2)),
        origin=Origin(xyz=(0.0, 0.0, 0.1)),
        name="platform_base",
    )
    platform.visual(
        Box((0.6, 0.3, 1.2)),
        origin=Origin(xyz=(0.0, 0.6, 0.8)),
        name="left_trunnion_mount",
    )
    platform.visual(
        Box((0.6, 0.3, 1.2)),
        origin=Origin(xyz=(0.0, -0.6, 0.8)),
        name="right_trunnion_mount",
    )

    barrel = model.part("barrel")
    barrel.visual(
        mesh_from_cadquery(build_barrel_body(), "barrel_body"),
        name="barrel_body",
    )
    barrel.visual(
        Cylinder(radius=0.15, length=1.5),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
        name="trunnion_pins",
    )

    model.articulation(
        "traverse",
        ArticulationType.CONTINUOUS,
        parent=barbette,
        child=platform,
        origin=Origin(xyz=(0.0, 0.0, 1.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1000.0, velocity=1.0),
    )

    model.articulation(
        "elevate",
        ArticulationType.REVOLUTE,
        parent=platform,
        child=barrel,
        origin=Origin(xyz=(0.0, 0.0, 1.1)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=500.0, velocity=1.0, lower=-0.1, upper=0.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    ctx.allow_overlap(
        "barrel",
        "gun_platform",
        elem_a="trunnion_pins",
        elem_b="left_trunnion_mount",
        reason="Trunnion pins are captured inside the mounts.",
    )
    ctx.allow_overlap(
        "barrel",
        "gun_platform",
        elem_a="trunnion_pins",
        elem_b="right_trunnion_mount",
        reason="Trunnion pins are captured inside the mounts.",
    )

    ctx.expect_contact(
        "gun_platform", 
        "barbette", 
        elem_a="platform_base", 
        elem_b="concrete_base", 
        name="platform sits exactly on barbette"
    )

    ctx.expect_gap(
        "gun_platform", 
        "barrel", 
        axis="y", 
        positive_elem="left_trunnion_mount", 
        negative_elem="barrel_body", 
        min_gap=0.05,
        name="barrel body clears left mount"
    )

    ctx.expect_gap(
        "barrel", 
        "gun_platform", 
        axis="y", 
        positive_elem="barrel_body", 
        negative_elem="right_trunnion_mount", 
        min_gap=0.05,
        name="barrel body clears right mount"
    )

    ctx.expect_overlap(
        "barrel", 
        "gun_platform", 
        axes="y", 
        elem_a="trunnion_pins", 
        elem_b="left_trunnion_mount", 
        min_overlap=0.2,
        name="trunnion pin retained in left mount"
    )

    ctx.expect_overlap(
        "barrel", 
        "gun_platform", 
        axes="y", 
        elem_a="trunnion_pins", 
        elem_b="right_trunnion_mount", 
        min_overlap=0.2,
        name="trunnion pin retained in right mount"
    )

    return ctx.report()


object_model = build_object_model()
