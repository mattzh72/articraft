import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def make_barrel() -> cq.Workplane:
    # A shallow barrel for a pancake lens.
    barrel = (
        cq.Workplane("XY")
        .circle(0.028).extrude(0.005)  # Backstop
        .faces(">Z").workplane()
        .circle(0.024).extrude(0.013)  # Tracks for rings
        .faces(">Z").workplane()
        .circle(0.028).extrude(0.007)  # Frontstop
    )
    # Hollow out the center for lens elements.
    barrel = barrel.faces(">Z").hole(0.030, depth=0.025)
    return barrel


def make_aperture_ring() -> cq.Workplane:
    base = (
        cq.Workplane("XY")
        .circle(0.028)
        .circle(0.025)  # 1mm radial clearance over the 0.024 track
        .extrude(0.0048)
    )
    # Add small grip notches
    cutters = (
        cq.Workplane("XY")
        .polarArray(0.028, 0, 360, 18)
        .rect(0.002, 0.002)
        .extrude(0.0048)
    )
    return base.cut(cutters)


def make_focus_ring() -> cq.Workplane:
    base = (
        cq.Workplane("XY")
        .circle(0.029)
        .circle(0.025)  # 1mm radial clearance over the 0.024 track
        .extrude(0.0078)
    )
    # Add small grip notches
    cutters = (
        cq.Workplane("XY")
        .polarArray(0.029, 0, 360, 36)
        .rect(0.0015, 0.002)
        .extrude(0.0078)
    )
    return base.cut(cutters)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pancake_lens")

    barrel = model.part("barrel")
    barrel.visual(
        mesh_from_cadquery(make_barrel(), "barrel_mesh"),
        origin=Origin(),
        name="barrel_shell",
    )

    # Front and rear glass elements
    barrel.visual(
        Cylinder(radius=0.015, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        name="rear_glass",
    )
    barrel.visual(
        Cylinder(radius=0.015, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        name="front_glass",
    )

    aperture_ring = model.part("aperture_ring")
    aperture_ring.visual(
        mesh_from_cadquery(make_aperture_ring(), "aperture_ring_mesh"),
        origin=Origin(),
        name="aperture_ring_shell",
    )

    model.articulation(
        "barrel_to_aperture_ring",
        ArticulationType.CONTINUOUS,
        parent=barrel,
        child=aperture_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.0051)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )

    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        mesh_from_cadquery(make_focus_ring(), "focus_ring_mesh"),
        origin=Origin(),
        name="focus_ring_shell",
    )

    model.articulation(
        "barrel_to_focus_ring",
        ArticulationType.CONTINUOUS,
        parent=barrel,
        child=focus_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.0101)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    barrel = object_model.get_part("barrel")
    aperture_ring = object_model.get_part("aperture_ring")
    focus_ring = object_model.get_part("focus_ring")

    # Rings are captured with a small gap to rotate freely
    ctx.allow_isolated_part("aperture_ring", reason="Aperture ring is captured around the barrel track with a small clearance gap to rotate freely.")
    ctx.allow_isolated_part("focus_ring", reason="Focus ring is captured around the barrel track with a small clearance gap to rotate freely.")

    # The rings sit around the barrel track and should be contained within the outer bounds
    ctx.expect_within(
        aperture_ring, barrel, axes="xy", margin=0.005, name="aperture ring within barrel xy"
    )
    ctx.expect_within(
        focus_ring, barrel, axes="xy", margin=0.005, name="focus ring within barrel xy"
    )

    # They are stacked adjacently in Z
    ctx.expect_gap(
        focus_ring, aperture_ring, axis="z", min_gap=-0.001, max_gap=0.005, name="rings adjacent in z"
    )

    # Check poses
    with ctx.pose(barrel_to_focus_ring=1.0):
        ctx.expect_within(
            focus_ring, barrel, axes="xy", margin=0.005, name="focus ring rotates freely"
        )
    with ctx.pose(barrel_to_aperture_ring=1.0):
        ctx.expect_within(
            aperture_ring, barrel, axes="xy", margin=0.005, name="aperture ring rotates freely"
        )

    return ctx.report()


object_model = build_object_model()