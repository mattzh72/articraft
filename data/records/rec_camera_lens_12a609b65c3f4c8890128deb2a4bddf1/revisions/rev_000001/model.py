import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def make_ribbed_ring(outer_radius, inner_radius, height, num_ribs, rib_depth, rib_width):
    ring = (
        cq.Workplane("XY")
        .cylinder(height, outer_radius)
        .cut(cq.Workplane("XY").cylinder(height, inner_radius))
    )
    cuts = (
        cq.Workplane("XY")
        .polarArray(outer_radius, 0, 360, num_ribs)
        .box(rib_depth * 2, rib_width, height * 1.1)
    )
    return ring.cut(cuts)


def make_mount():
    return (
        cq.Workplane("XY")
        .workplane(offset=0.0025)
        .cylinder(0.005, 0.028)
        .cut(cq.Workplane("XY").workplane(offset=0.0025).cylinder(0.005, 0.020))
    )


def make_barrel():
    pts = [
        (0.020, 0.005),
        (0.038, 0.005),
        (0.038, 0.030),
        (0.0368, 0.030),
        (0.0368, 0.070),
        (0.039, 0.070),
        (0.039, 0.080),
        (0.0368, 0.080),
        (0.0368, 0.110),
        (0.041, 0.110),
        (0.041, 0.140),
        (0.038, 0.140),
        (0.038, 0.135),
        (0.030, 0.135),
        (0.030, 0.030),
        (0.020, 0.030),
        (0.020, 0.005),
    ]
    profile = cq.Workplane("XZ").polyline(pts).close()
    return profile.revolve(360, (0, 0, 0), (0, 1, 0))


def make_front_glass():
    sphere = cq.Workplane("XY").workplane(offset=0.095).sphere(0.050)
    cyl = cq.Workplane("XY").workplane(offset=0.135).cylinder(0.020, 0.0301)
    return sphere.intersect(cyl)


def make_rear_glass():
    sphere = cq.Workplane("XY").workplane(offset=0.040).sphere(0.035)
    cyl = cq.Workplane("XY").workplane(offset=0.010).cylinder(0.020, 0.0201)
    return sphere.intersect(cyl)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="camera_lens")

    lens_body = model.part("lens_body")

    lens_body.visual(
        mesh_from_cadquery(make_mount(), "mount_flange"),
        name="mount_flange",
        material=Material("silver", color=(0.8, 0.8, 0.8)),
    )

    lens_body.visual(
        mesh_from_cadquery(make_barrel(), "main_barrel"),
        name="main_barrel",
        material=Material("black", color=(0.1, 0.1, 0.1)),
    )

    lens_body.visual(
        mesh_from_cadquery(make_front_glass(), "front_glass"),
        name="front_glass",
        material=Material("glass", rgba=(0.1, 0.1, 0.2, 0.3)),
    )

    lens_body.visual(
        mesh_from_cadquery(make_rear_glass(), "rear_glass"),
        name="rear_glass",
        material=Material("glass", rgba=(0.1, 0.1, 0.2, 0.3)),
    )

    zoom_ring = model.part("zoom_ring")
    zoom_ring.visual(
        mesh_from_cadquery(
            make_ribbed_ring(0.040, 0.0365, 0.0398, 72, 0.001, 0.0015),
            "zoom_ring_mesh",
        ),
        name="zoom_ring_mesh",
        material=Material("rubber", color=(0.05, 0.05, 0.05)),
    )

    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        mesh_from_cadquery(
            make_ribbed_ring(0.040, 0.0365, 0.0298, 90, 0.0008, 0.001),
            "focus_ring_mesh",
        ),
        name="focus_ring_mesh",
        material=Material("rubber", color=(0.05, 0.05, 0.05)),
    )

    model.articulation(
        "zoom_joint",
        ArticulationType.REVOLUTE,
        parent=lens_body,
        child=zoom_ring,
        origin=Origin(xyz=(0, 0, 0.050)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(lower=0.0, upper=1.57),
    )

    model.articulation(
        "focus_joint",
        ArticulationType.REVOLUTE,
        parent=lens_body,
        child=focus_ring,
        origin=Origin(xyz=(0, 0, 0.095)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(lower=-1.57, upper=1.57),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lens_body = object_model.get_part("lens_body")
    zoom_ring = object_model.get_part("zoom_ring")
    focus_ring = object_model.get_part("focus_ring")

    ctx.allow_overlap(
        zoom_ring,
        lens_body,
        elem_a="zoom_ring_mesh",
        elem_b="main_barrel",
        reason="Zoom ring is seated around the main barrel.",
    )
    ctx.allow_overlap(
        focus_ring,
        lens_body,
        elem_a="focus_ring_mesh",
        elem_b="main_barrel",
        reason="Focus ring is seated around the main barrel.",
    )

    ctx.expect_within(
        zoom_ring,
        lens_body,
        axes="xy",
        inner_elem="zoom_ring_mesh",
        outer_elem="main_barrel",
        margin=0.005,
    )
    ctx.expect_within(
        focus_ring,
        lens_body,
        axes="xy",
        inner_elem="focus_ring_mesh",
        outer_elem="main_barrel",
        margin=0.005,
    )

    ctx.expect_overlap(
        zoom_ring,
        lens_body,
        axes="z",
        elem_a="zoom_ring_mesh",
        elem_b="main_barrel",
        min_overlap=0.035,
    )
    ctx.expect_overlap(
        focus_ring,
        lens_body,
        axes="z",
        elem_a="focus_ring_mesh",
        elem_b="main_barrel",
        min_overlap=0.025,
    )

    return ctx.report()


object_model = build_object_model()
