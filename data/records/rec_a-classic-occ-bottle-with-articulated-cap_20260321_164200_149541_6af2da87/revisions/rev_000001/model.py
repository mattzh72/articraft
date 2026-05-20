from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.

# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    AssetContext,
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


ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root

BODY_LENGTH = 0.078
BODY_WIDTH = 0.032
SHOULDER_SWELL = 0.013
BODY_HEIGHT = 0.112
BODY_WALL = 0.0018
VERTICAL_FILLET = 0.0045

NECK_RADIUS = 0.012
NECK_HEIGHT = 0.020
NECK_BEAD_RADIUS = 0.0134
NECK_BEAD_HEIGHT = 0.0024

FRAME_HEIGHT = 0.010
FRAME_OUTER_RADIUS = 0.0144
FRAME_INNER_RADIUS = 0.0118
FRAME_LATCH_WIDTH = 0.012
FRAME_LATCH_DEPTH = 0.003
FRAME_LATCH_HEIGHT = 0.003

CAP_HEIGHT = 0.025
CAP_OUTER_RADIUS = 0.0170
CAP_INNER_RADIUS = 0.0152
CAP_WALL = 0.0018

HINGE_PIN_RADIUS = 0.0026
BODY_LUG_LENGTH = 0.0065
CAP_KNUCKLE_LENGTH = 0.0100
BODY_LUG_X = 0.0088
CAP_CENTER_Y = 0.0180
CAP_CENTER_Z = 0.0060
HINGE_Y = -CAP_CENTER_Y
HINGE_Z = BODY_HEIGHT + NECK_HEIGHT + 0.0048


def _make_glass_bottle() -> cq.Workplane:
    bottle_blank = (
        cq.Workplane("XY")
        .center(-BODY_LENGTH / 2.0, 0.0)
        .vLine(BODY_WIDTH / 2.0)
        .threePointArc(
            (BODY_LENGTH / 2.0, BODY_WIDTH / 2.0 + SHOULDER_SWELL),
            (BODY_LENGTH, BODY_WIDTH / 2.0),
        )
        .vLine(-BODY_WIDTH / 2.0)
        .mirrorX()
        .extrude(BODY_HEIGHT)
        .edges("|Z")
        .fillet(VERTICAL_FILLET)
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .circle(NECK_RADIUS)
        .extrude(NECK_HEIGHT)
    )

    hollow = bottle_blank.faces(">Z").shell(BODY_WALL)
    bead = (
        cq.Workplane("XY")
        .circle(NECK_BEAD_RADIUS)
        .circle(NECK_RADIUS)
        .extrude(NECK_BEAD_HEIGHT)
        .translate((0.0, 0.0, BODY_HEIGHT + NECK_HEIGHT - 0.006))
    )
    return hollow.union(bead)


def _make_neck_frame() -> cq.Workplane:
    collar = (
        cq.Workplane("XY")
        .circle(FRAME_OUTER_RADIUS)
        .circle(FRAME_INNER_RADIUS)
        .extrude(FRAME_HEIGHT)
        .translate((0.0, 0.0, BODY_HEIGHT + NECK_HEIGHT - FRAME_HEIGHT))
    )

    front_latch = (
        cq.Workplane("XY")
        .box(FRAME_LATCH_WIDTH, FRAME_LATCH_DEPTH, FRAME_LATCH_HEIGHT)
        .translate(
            (
                0.0,
                FRAME_OUTER_RADIUS - 0.0012,
                BODY_HEIGHT + NECK_HEIGHT - 0.0034,
            )
        )
    )

    rear_bridge = (
        cq.Workplane("XY")
        .box(0.022, 0.0045, 0.0045)
        .translate((0.0, -0.0133, BODY_HEIGHT + NECK_HEIGHT - 0.0044))
    )

    support_arms = []
    hinge_lugs = []
    for lug_x in (-BODY_LUG_X, BODY_LUG_X):
        support_arms.append(
            cq.Workplane("XY")
            .box(BODY_LUG_LENGTH, 0.0044, 0.0120)
            .translate((lug_x, -0.0139, HINGE_Z - 0.0060))
        )
        hinge_lugs.append(
            cq.Workplane("YZ")
            .circle(HINGE_PIN_RADIUS)
            .extrude(BODY_LUG_LENGTH / 2.0, both=True)
            .translate((lug_x, HINGE_Y, HINGE_Z))
        )

    frame = collar.union(front_latch).union(rear_bridge)
    for arm in support_arms:
        frame = frame.union(arm)
    for lug in hinge_lugs:
        frame = frame.union(lug)
    return frame


def _make_cap() -> cq.Workplane:
    cap_outer = (
        cq.Workplane("XY")
        .circle(CAP_OUTER_RADIUS)
        .extrude(CAP_HEIGHT)
        .faces(">Z")
        .edges()
        .fillet(0.0015)
        .translate((0.0, CAP_CENTER_Y, CAP_CENTER_Z - CAP_HEIGHT / 2.0))
    )

    cap_inner = (
        cq.Workplane("XY")
        .circle(CAP_INNER_RADIUS)
        .extrude(CAP_HEIGHT - CAP_WALL)
        .translate((0.0, CAP_CENTER_Y, CAP_CENTER_Z - CAP_HEIGHT / 2.0))
    )

    shell = cap_outer.cut(cap_inner)

    knuckle = (
        cq.Workplane("YZ")
        .circle(HINGE_PIN_RADIUS)
        .extrude(CAP_KNUCKLE_LENGTH / 2.0, both=True)
    )

    web = (
        cq.Workplane("XY")
        .box(CAP_KNUCKLE_LENGTH, 0.018, 0.0055)
        .translate((0.0, 0.0082, 0.0022))
    )

    thumb_tab = (
        cq.Workplane("XY")
        .box(0.018, 0.006, 0.0038)
        .edges("|X")
        .fillet(0.0011)
        .translate(
            (
                0.0,
                CAP_CENTER_Y + CAP_OUTER_RADIUS * 0.68,
                CAP_CENTER_Z + CAP_HEIGHT * 0.18,
            )
        )
    )

    top_button = (
        cq.Workplane("XY")
        .circle(0.0065)
        .extrude(0.0014)
        .translate(
            (0.0, CAP_CENTER_Y + 0.0015, CAP_CENTER_Z + CAP_HEIGHT / 2.0 - 0.0007)
        )
    )

    return shell.union(knuckle).union(web).union(thumb_tab).union(top_button)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="classic_occ_bottle_with_articulated_cap", assets=ASSETS)

    glass = model.material("glass", rgba=(0.58, 0.79, 0.72, 0.42))
    charcoal = model.material("charcoal", rgba=(0.10, 0.11, 0.13, 1.0))

    bottle = model.part("bottle")
    bottle.visual(
        mesh_from_cadquery(_make_glass_bottle(), "bottle_glass.obj", assets=ASSETS),
        material=glass,
    )
    bottle.visual(
        mesh_from_cadquery(_make_neck_frame(), "bottle_frame.obj", assets=ASSETS),
        material=charcoal,
    )
    bottle.inertial = Inertial.from_geometry(
        Box((BODY_LENGTH, BODY_WIDTH, BODY_HEIGHT + NECK_HEIGHT)),
        mass=0.32,
        origin=Origin(xyz=(0.0, 0.0, (BODY_HEIGHT + NECK_HEIGHT) / 2.0)),
    )

    cap = model.part("cap")
    cap.visual(
        mesh_from_cadquery(_make_cap(), "cap.obj", assets=ASSETS),
        material=charcoal,
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(radius=CAP_OUTER_RADIUS, length=CAP_HEIGHT),
        mass=0.05,
        origin=Origin(xyz=(0.0, CAP_CENTER_Y, CAP_CENTER_Z)),
    )

    model.articulation(
        "cap_hinge",
        ArticulationType.REVOLUTE,
        parent="bottle",
        child="cap",
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=0.0,
            upper=2.1,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_far_from_geometry(tol=0.015)
    ctx.warn_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.warn_if_coplanar_surfaces(use="visual")
    ctx.warn_if_overlaps(
        max_pose_samples=96,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    assert BODY_HEIGHT + NECK_HEIGHT > BODY_LENGTH * 1.6, (
        "Classic OCC bottle silhouette should read as a tall body with a distinct neck."
    )
    assert BODY_WIDTH < BODY_LENGTH * 0.5, (
        "Bottle planform should remain slender rather than squat."
    )
    assert NECK_RADIUS * 2.0 < BODY_WIDTH * 0.9, (
        "Neck should read narrower than the bottle body."
    )
    assert CAP_OUTER_RADIUS > FRAME_OUTER_RADIUS, (
        "Flip cap must visibly cover the neck frame."
    )
    assert HINGE_Z > BODY_HEIGHT + NECK_HEIGHT, (
        "Cap hinge should sit above the bottle lip, not inside the shoulder."
    )

    with ctx.pose(cap_hinge=0.0):
        ctx.expect_aabb_overlap("cap", "bottle", axes="x", min_overlap=0.026)
        ctx.expect_aabb_overlap("cap", "bottle", axes="y", min_overlap=0.022)
        ctx.expect_aabb_contact("cap", "bottle")

    ctx.expect_joint_motion_axis(
        "cap_hinge",
        "cap",
        world_axis="y",
        direction="negative",
        min_delta=0.008,
    )
    ctx.expect_joint_motion_axis(
        "cap_hinge",
        "cap",
        world_axis="z",
        direction="positive",
        min_delta=0.004,
    )

    with ctx.pose(cap_hinge=2.0):
        ctx.expect_aabb_contact("cap", "bottle")
        ctx.expect_aabb_overlap("cap", "bottle", axes="x", min_overlap=0.010)

    with ctx.pose(cap_hinge=1.05):
        ctx.expect_aabb_contact("cap", "bottle")
        ctx.expect_aabb_overlap("cap", "bottle", axes="x", min_overlap=0.018)

    return ctx.report()
# >>> USER_CODE_END

object_model = build_object_model()
