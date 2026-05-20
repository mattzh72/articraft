from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 0.160
BASE_WIDTH = 0.160
BASE_THICKNESS = 0.014
BASE_CENTER_X = -0.030

UPRIGHT_THICKNESS = 0.018
UPRIGHT_WIDTH = 0.120
UPRIGHT_HEIGHT = 0.220

GUSSET_THICKNESS = 0.014
GUSSET_HEIGHT = 0.110
GUSSET_FORWARD_REACH = 0.042

GUIDE_CENTER_Z = 0.160
GUIDE_LENGTH = 0.340
GUIDE_WIDTH = 0.038
GUIDE_HEIGHT = 0.018
GUIDE_FLANGE_LENGTH = 0.018
GUIDE_FLANGE_WIDTH = 0.060
GUIDE_FLANGE_HEIGHT = 0.064

CARRIAGE_BODY_LENGTH = 0.076
CARRIAGE_BODY_WIDTH = 0.070
CARRIAGE_BODY_HEIGHT = 0.038

SHUTTLE_DECK_LENGTH = 0.110
SHUTTLE_DECK_WIDTH = 0.100
SHUTTLE_DECK_THICKNESS = 0.010
SHUTTLE_DECK_CENTER_X = 0.018

PUSHER_THICKNESS = 0.008
PUSHER_WIDTH = 0.074
PUSHER_HEIGHT = 0.034

CARRIAGE_REST_X = 0.102
CARRIAGE_TRAVEL = 0.160


def _build_support_frame() -> cq.Workplane:
    base = (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS)
        .translate((BASE_CENTER_X, 0.0, BASE_THICKNESS / 2.0))
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-0.045, -0.050),
                (-0.045, 0.050),
                (0.045, -0.050),
                (0.045, 0.050),
            ]
        )
        .hole(0.012)
    )

    upright = cq.Workplane("XY").box(
        UPRIGHT_THICKNESS,
        UPRIGHT_WIDTH,
        UPRIGHT_HEIGHT,
    ).translate(
        (
            -UPRIGHT_THICKNESS / 2.0,
            0.0,
            BASE_THICKNESS + (UPRIGHT_HEIGHT / 2.0),
        )
    )

    mount_pad = cq.Workplane("XY").box(0.010, 0.080, 0.080).translate(
        (-0.005, 0.0, GUIDE_CENTER_Z)
    )

    gusset_profile = [
        (GUSSET_FORWARD_REACH, BASE_THICKNESS),
        (-UPRIGHT_THICKNESS, BASE_THICKNESS),
        (-UPRIGHT_THICKNESS, BASE_THICKNESS + GUSSET_HEIGHT),
    ]
    gusset = (
        cq.Workplane("XZ")
        .polyline(gusset_profile)
        .close()
        .extrude(GUSSET_THICKNESS / 2.0, both=True)
    )
    left_gusset = gusset.translate(
        (0.0, (UPRIGHT_WIDTH / 2.0) - (GUSSET_THICKNESS / 2.0), 0.0)
    )
    right_gusset = gusset.translate(
        (0.0, -(UPRIGHT_WIDTH / 2.0) + (GUSSET_THICKNESS / 2.0), 0.0)
    )

    return base.union(upright).union(mount_pad).union(left_gusset).union(right_gusset)


def _build_guide_track() -> cq.Workplane:
    return cq.Workplane("XY").box(
        GUIDE_LENGTH,
        GUIDE_WIDTH,
        GUIDE_HEIGHT,
        centered=(False, True, True),
    )


def _build_guide_mount() -> cq.Workplane:
    flange = (
        cq.Workplane("XY")
        .box(
            GUIDE_FLANGE_LENGTH,
            GUIDE_FLANGE_WIDTH,
            GUIDE_FLANGE_HEIGHT,
            centered=(False, True, True),
        )
        .edges("|X")
        .fillet(0.0035)
    )
    return flange


def _build_carriage() -> cq.Workplane:
    side_shoe_length = 0.050
    side_shoe_thickness = 0.012
    side_shoe_height = 0.022

    body = (
        cq.Workplane("XY")
        .box(
            CARRIAGE_BODY_LENGTH,
            CARRIAGE_BODY_WIDTH,
            CARRIAGE_BODY_HEIGHT,
        )
        .translate(
            (
                0.0,
                0.0,
                (GUIDE_HEIGHT / 2.0) + 0.002 + (CARRIAGE_BODY_HEIGHT / 2.0),
            )
        )
        .edges("|X")
        .fillet(0.0025)
    )

    left_shoe = cq.Workplane("XY").box(
        side_shoe_length,
        side_shoe_thickness,
        side_shoe_height,
    ).translate((0.0, (GUIDE_WIDTH / 2.0) + 0.001 + (side_shoe_thickness / 2.0), 0.0))
    right_shoe = cq.Workplane("XY").box(
        side_shoe_length,
        side_shoe_thickness,
        side_shoe_height,
    ).translate((0.0, -(GUIDE_WIDTH / 2.0) - 0.001 - (side_shoe_thickness / 2.0), 0.0))

    deck = (
        cq.Workplane("XY")
        .box(
            SHUTTLE_DECK_LENGTH,
            SHUTTLE_DECK_WIDTH,
            SHUTTLE_DECK_THICKNESS,
        )
        .translate(
            (
                SHUTTLE_DECK_CENTER_X,
                0.0,
                GUIDE_HEIGHT / 2.0
                + CARRIAGE_BODY_HEIGHT
                + (SHUTTLE_DECK_THICKNESS / 2.0),
            )
        )
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([(-0.026, 0.0), (0.026, 0.0)])
        .hole(0.005)
        .edges("|Z")
        .fillet(0.002)
    )

    pusher_center_x = (
        SHUTTLE_DECK_CENTER_X
        + (SHUTTLE_DECK_LENGTH / 2.0)
        - (PUSHER_THICKNESS / 2.0)
    )
    pusher = (
        cq.Workplane("XY")
        .box(PUSHER_THICKNESS, PUSHER_WIDTH, PUSHER_HEIGHT)
        .translate(
            (
                pusher_center_x,
                0.0,
                GUIDE_HEIGHT / 2.0
                + SHUTTLE_DECK_THICKNESS
                + CARRIAGE_BODY_HEIGHT
                + (PUSHER_HEIGHT / 2.0),
            )
        )
        .edges("|X")
        .fillet(0.0015)
    )

    return body.union(left_shoe).union(right_shoe).union(deck).union(pusher)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cantilever_shuttle_module")

    model.material("support_steel", rgba=(0.24, 0.27, 0.30, 1.0))
    model.material("guide_steel", rgba=(0.68, 0.71, 0.75, 1.0))
    model.material("carriage_aluminum", rgba=(0.84, 0.86, 0.88, 1.0))

    support_frame = model.part("support_frame")
    support_frame.visual(
        mesh_from_cadquery(_build_support_frame(), "support_frame"),
        material="support_steel",
        name="support_body",
    )
    support_frame.visual(
        mesh_from_cadquery(_build_guide_mount(), "support_frame_guide_mount"),
        origin=Origin(xyz=(0.0, 0.0, GUIDE_CENTER_Z)),
        material="guide_steel",
        name="guide_mount",
    )
    support_frame.visual(
        mesh_from_cadquery(_build_guide_track(), "support_frame_guide_track"),
        origin=Origin(xyz=(0.0, 0.0, GUIDE_CENTER_Z)),
        material="guide_steel",
        name="guide_track",
    )
    support_frame.inertial = Inertial.from_geometry(
        Box((0.450, BASE_WIDTH, BASE_THICKNESS + UPRIGHT_HEIGHT)),
        mass=14.0,
        origin=Origin(
            xyz=(
                0.115,
                0.0,
                (BASE_THICKNESS + UPRIGHT_HEIGHT) / 2.0,
            )
        ),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_build_carriage(), "carriage_body"),
        material="carriage_aluminum",
        name="carriage_body",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.115, SHUTTLE_DECK_WIDTH, 0.108)),
        mass=2.1,
        origin=Origin(xyz=(0.020, 0.0, 0.041)),
    )

    model.articulation(
        "guide_to_carriage",
        ArticulationType.PRISMATIC,
        parent=support_frame,
        child=carriage,
        origin=Origin(xyz=(CARRIAGE_REST_X, 0.0, GUIDE_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.35,
            lower=0.0,
            upper=CARRIAGE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support_frame = object_model.get_part("support_frame")
    carriage = object_model.get_part("carriage")
    support_body = support_frame.get_visual("support_body")
    guide_track = support_frame.get_visual("guide_track")
    carriage_body = carriage.get_visual("carriage_body")
    slide = object_model.get_articulation("guide_to_carriage")

    ctx.allow_isolated_part(
        carriage,
        reason="The carriage rides on a hidden recirculating-bearing interface, so the exposed shell keeps a small clearance from the guide track.",
    )

    ctx.expect_gap(
        carriage,
        support_frame,
        axis="x",
        min_gap=0.010,
        positive_elem=carriage_body,
        negative_elem=support_body,
        name="carriage clears the grounded support at rest",
    )
    ctx.expect_overlap(
        carriage,
        support_frame,
        axes="y",
        min_overlap=0.030,
        elem_a=carriage_body,
        elem_b=guide_track,
        name="carriage stays laterally aligned with the guide in rest pose",
    )
    ctx.expect_overlap(
        carriage,
        support_frame,
        axes="x",
        min_overlap=0.100,
        elem_a=carriage_body,
        elem_b=guide_track,
        name="rest pose retains carriage insertion on the guide",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: CARRIAGE_TRAVEL}):
        ctx.expect_gap(
            carriage,
            support_frame,
            axis="x",
            min_gap=0.080,
            positive_elem=carriage_body,
            negative_elem=support_body,
            name="extended carriage projects outward beyond the side support",
        )
        ctx.expect_overlap(
            carriage,
            support_frame,
            axes="y",
            min_overlap=0.030,
            elem_a=carriage_body,
            elem_b=guide_track,
            name="extended carriage remains laterally aligned with the guide",
        )
        ctx.expect_overlap(
            carriage,
            support_frame,
            axes="x",
            min_overlap=0.100,
            elem_a=carriage_body,
            elem_b=guide_track,
            name="extended carriage still retains insertion on the guide",
        )
        extended_pos = ctx.part_world_position(carriage)

    ctx.check(
        "carriage translates along the guide +X axis",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.120
        and abs(extended_pos[1] - rest_pos[1]) < 1e-6
        and abs(extended_pos[2] - rest_pos[2]) < 1e-6,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
