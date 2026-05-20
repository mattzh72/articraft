from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


GUIDEWAY_LENGTH = 1.30
GUIDEWAY_TOP_Z = 0.15

CARRIAGE_FRAME_Z = 0.21
CARRIAGE_TRAVEL = 0.32

PROXIMAL_LENGTH = 0.50
DISTAL_LENGTH = 0.36


def make_guideway() -> cq.Workplane:
    body = cq.Workplane("XY").box(1.10, 0.22, 0.05).translate((0.0, 0.0, 0.045))
    body = body.union(
        cq.Workplane("XY").box(1.02, 0.14, 0.04).translate((0.0, 0.0, 0.09))
    )
    body = body.union(
        cq.Workplane("XY").box(0.98, 0.10, 0.04).translate((0.0, 0.0, 0.13))
    )

    for sx in (-1.0, 1.0):
        body = body.union(
            cq.Workplane("XY").box(0.26, 0.34, 0.02).translate((sx * 0.42, 0.0, 0.01))
        )
        body = body.union(
            cq.Workplane("XY").box(0.08, 0.18, 0.08).translate((sx * 0.53, 0.0, 0.08))
        )

    return body


def make_carriage() -> cq.Workplane:
    body = cq.Workplane("XY").box(0.24, 0.18, 0.06).translate((0.0, 0.0, -0.03))
    body = body.union(
        cq.Workplane("XY").box(0.14, 0.10, 0.10).translate((0.0, 0.0, 0.02))
    )
    body = body.union(
        cq.Workplane("XY").box(0.10, 0.08, 0.08).translate((0.06, 0.0, 0.11))
    )
    body = body.union(
        cq.Workplane("XY").box(0.02, 0.09, 0.10).translate((0.11, 0.0, 0.11))
    )
    return body


def make_proximal_link() -> cq.Workplane:
    body = cq.Workplane("XY").box(0.07, 0.05, 0.072).translate((0.035, 0.0, 0.0))
    body = body.union(
        cq.Workplane("XY").box(0.33, 0.045, 0.055).translate((0.235, 0.0, 0.0))
    )
    body = body.union(
        cq.Workplane("XY").box(0.10, 0.06, 0.07).translate((0.45, 0.0, 0.0))
    )
    return body


def make_distal_link() -> cq.Workplane:
    body = cq.Workplane("XY").box(0.06, 0.045, 0.06).translate((0.03, 0.0, 0.0))
    body = body.union(
        cq.Workplane("XY").box(0.22, 0.04, 0.05).translate((0.17, 0.0, 0.0))
    )
    body = body.union(
        cq.Workplane("XY").box(0.08, 0.05, 0.06).translate((0.32, 0.0, 0.0))
    )
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_axis")

    dark_base = model.material("dark_base", rgba=(0.20, 0.21, 0.24, 1.0))
    carriage_gray = model.material("carriage_gray", rgba=(0.58, 0.60, 0.63, 1.0))
    arm_blue = model.material("arm_blue", rgba=(0.18, 0.38, 0.66, 1.0))
    arm_orange = model.material("arm_orange", rgba=(0.88, 0.53, 0.16, 1.0))

    guideway = model.part("guideway")
    guideway.visual(
        mesh_from_cadquery(make_guideway(), "guideway_body"),
        material=dark_base,
        name="guideway_body",
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(make_carriage(), "carriage_body"),
        material=carriage_gray,
        name="carriage_body",
    )

    proximal = model.part("proximal_link")
    proximal.visual(
        mesh_from_cadquery(make_proximal_link(), "proximal_link_body"),
        material=arm_blue,
        name="proximal_link_body",
    )

    distal = model.part("distal_link")
    distal.visual(
        mesh_from_cadquery(make_distal_link(), "distal_link_body"),
        material=arm_orange,
        name="distal_link_body",
    )

    model.articulation(
        "guideway_to_carriage",
        ArticulationType.PRISMATIC,
        parent=guideway,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_FRAME_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=800.0,
            velocity=0.6,
            lower=-CARRIAGE_TRAVEL,
            upper=CARRIAGE_TRAVEL,
        ),
    )

    model.articulation(
        "carriage_to_proximal",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=proximal,
        origin=Origin(xyz=(0.12, 0.0, 0.11)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=1.6,
            lower=-0.35,
            upper=1.35,
        ),
    )

    model.articulation(
        "proximal_to_distal",
        ArticulationType.REVOLUTE,
        parent=proximal,
        child=distal,
        origin=Origin(xyz=(PROXIMAL_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=160.0,
            velocity=2.0,
            lower=-1.15,
            upper=1.85,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    guideway = object_model.get_part("guideway")
    carriage = object_model.get_part("carriage")
    proximal = object_model.get_part("proximal_link")
    distal = object_model.get_part("distal_link")

    slide = object_model.get_articulation("guideway_to_carriage")
    shoulder = object_model.get_articulation("carriage_to_proximal")
    elbow = object_model.get_articulation("proximal_to_distal")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        carriage,
        guideway,
        contact_tol=0.001,
        name="carriage runners contact guideway rail",
    )
    ctx.expect_contact(
        proximal,
        carriage,
        contact_tol=0.001,
        name="proximal hub is supported by carriage clevis",
    )
    ctx.expect_contact(
        distal,
        proximal,
        contact_tol=0.001,
        name="distal hub is supported by proximal clevis",
    )

    with ctx.pose({slide: slide.motion_limits.upper}):
        ctx.expect_origin_gap(
            carriage,
            guideway,
            axis="x",
            min_gap=0.30,
            max_gap=0.34,
            name="prismatic root travels toward positive x",
        )

    with ctx.pose({shoulder: shoulder.motion_limits.upper}):
        ctx.expect_gap(
            distal,
            guideway,
            axis="z",
            min_gap=0.28,
            name="shoulder lift raises distal link above the guideway",
        )

    with ctx.pose({shoulder: 0.55, elbow: elbow.motion_limits.upper}):
        ctx.expect_gap(
            distal,
            guideway,
            axis="z",
            min_gap=0.18,
            name="positive elbow motion folds the shorter distal link upward",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
