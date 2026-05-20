from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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


SUPPORT_PLATE_Z = 0.18
BEAM_PIVOT_Z = 0.0
SLIDE_HOME_X = 0.22
RAIL_BOTTOM_Z = -0.155
SLIDE_TRAVEL = 0.55


def _make_support_shape() -> cq.Workplane:
    top_plate = cq.Workplane("XY").box(0.48, 0.24, 0.03).translate((0.0, 0.0, SUPPORT_PLATE_Z))

    left_cheek = cq.Workplane("XY").box(0.22, 0.025, 0.15).translate((0.0, 0.0775, 0.09))
    right_cheek = cq.Workplane("XY").box(0.22, 0.025, 0.15).translate((0.0, -0.0775, 0.09))
    crown_block = cq.Workplane("XY").box(0.16, 0.18, 0.04).translate((0.0, 0.0, 0.145))

    bearing_housing = cq.Workplane("XY").circle(0.05).extrude(0.11).translate((0.0, 0.0, 0.015))
    thrust_plate = cq.Workplane("XY").circle(0.07).extrude(0.015)

    left_rib = (
        cq.Workplane("XZ")
        .polyline([(-0.09, 0.015), (-0.09, 0.15), (-0.02, 0.15), (0.0, 0.04), (0.0, 0.015)])
        .close()
        .extrude(0.02)
        .translate((0.0, -0.08, 0.0))
    )
    right_rib = left_rib.mirror("XZ")

    return (
        top_plate.union(left_cheek)
        .union(right_cheek)
        .union(crown_block)
        .union(bearing_housing)
        .union(thrust_plate)
        .union(left_rib)
        .union(right_rib)
        .findSolid()
    )


def _make_beam_shape() -> cq.Workplane:
    hub = cq.Workplane("XY").circle(0.07).extrude(0.115).translate((0.0, 0.0, -0.13))
    thrust_disc = cq.Workplane("XY").circle(0.066).extrude(0.015).translate((0.0, 0.0, -0.015))

    top_flange = cq.Workplane("XY").box(0.98, 0.12, 0.02).translate((0.55, 0.0, -0.055))
    left_web = cq.Workplane("XY").box(0.98, 0.02, 0.09).translate((0.55, 0.05, -0.10))
    right_web = cq.Workplane("XY").box(0.98, 0.02, 0.09).translate((0.55, -0.05, -0.10))
    center_spine = cq.Workplane("XY").box(0.34, 0.04, 0.05).translate((0.08, 0.0, -0.09))

    rear_stub = cq.Workplane("XY").box(0.16, 0.10, 0.08).translate((-0.09, 0.0, -0.09))
    left_rail = cq.Workplane("XY").box(0.84, 0.025, 0.02).translate((0.56, 0.0475, -0.145))
    right_rail = cq.Workplane("XY").box(0.84, 0.025, 0.02).translate((0.56, -0.0475, -0.145))
    nose_cap = cq.Workplane("XY").box(0.05, 0.14, 0.13).translate((1.035, 0.0, -0.085))

    return (
        hub.union(thrust_disc)
        .union(top_flange)
        .union(left_web)
        .union(right_web)
        .union(center_spine)
        .union(rear_stub)
        .union(left_rail)
        .union(right_rail)
        .union(nose_cap)
        .findSolid()
    )


def _make_carriage_shape() -> cq.Workplane:
    head = cq.Workplane("XY").box(0.16, 0.18, 0.05).translate((0.0, 0.0, -0.025))
    head_recess = cq.Workplane("XY").box(0.14, 0.08, 0.032).translate((0.0, 0.0, -0.014))
    head = head.cut(head_recess)

    left_plate = cq.Workplane("XY").box(0.08, 0.02, 0.26).translate((0.0, 0.05, -0.16))
    right_plate = cq.Workplane("XY").box(0.08, 0.02, 0.26).translate((0.0, -0.05, -0.16))
    crosshead = cq.Workplane("XY").box(0.036, 0.14, 0.036).translate((0.0, 0.0, -0.28))
    bottom_bridge = cq.Workplane("XY").box(0.05, 0.12, 0.04).translate((0.0, 0.0, -0.31))
    drop_lug = cq.Workplane("XY").box(0.03, 0.08, 0.10).translate((0.0, 0.0, -0.36))

    eye_outer = cq.Workplane("XZ").circle(0.03).extrude(0.02).translate((0.0, -0.01, -0.42))
    eye_inner = cq.Workplane("XZ").circle(0.015).extrude(0.03).translate((0.0, -0.015, -0.42))
    lifting_eye = eye_outer.cut(eye_inner)

    return (
        head.union(left_plate)
        .union(right_plate)
        .union(crosshead)
        .union(bottom_bridge)
        .union(drop_lug)
        .union(lifting_eye)
        .findSolid()
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="underslung_radial_arm_carriage")

    support_mat = model.material("support_steel", rgba=(0.38, 0.40, 0.43, 1.0))
    beam_mat = model.material("beam_paint", rgba=(0.88, 0.68, 0.17, 1.0))
    carriage_mat = model.material("carriage_blue", rgba=(0.18, 0.30, 0.55, 1.0))

    support = model.part("top_support")
    support.visual(
        mesh_from_cadquery(_make_support_shape(), "top_support"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=support_mat,
        name="support_body",
    )

    beam = model.part("beam")
    beam.visual(
        mesh_from_cadquery(_make_beam_shape(), "beam"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=beam_mat,
        name="beam_body",
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_make_carriage_shape(), "carriage"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=carriage_mat,
        name="carriage_body",
    )

    model.articulation(
        "support_to_beam",
        ArticulationType.REVOLUTE,
        parent=support,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, BEAM_PIVOT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.2, lower=-2.35, upper=2.35),
    )

    model.articulation(
        "beam_to_carriage",
        ArticulationType.PRISMATIC,
        parent=beam,
        child=carriage,
        origin=Origin(xyz=(SLIDE_HOME_X, 0.0, RAIL_BOTTOM_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.35, lower=0.0, upper=SLIDE_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("top_support")
    beam = object_model.get_part("beam")
    carriage = object_model.get_part("carriage")
    swing = object_model.get_articulation("support_to_beam")
    slide = object_model.get_articulation("beam_to_carriage")

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

    ctx.expect_gap(
        support,
        beam,
        axis="z",
        min_gap=0.0,
        max_gap=0.0005,
        name="top support bears directly on the beam thrust disc",
    )
    ctx.expect_contact(
        support,
        beam,
        contact_tol=0.0005,
        name="beam is physically carried by the top support",
    )
    ctx.expect_gap(
        beam,
        carriage,
        axis="z",
        min_gap=0.0,
        max_gap=0.0005,
        name="carriage hangs from the beam rails",
    )
    ctx.expect_contact(
        beam,
        carriage,
        contact_tol=0.0005,
        name="carriage remains mounted to the beam",
    )

    with ctx.pose({swing: 0.0, slide: 0.0}):
        home = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            carriage,
            beam,
            axes="xy",
            min_overlap=0.08,
            name="home carriage head overlaps the beam footprint",
        )

    with ctx.pose({swing: 0.0, slide: SLIDE_TRAVEL}):
        extended = ctx.part_world_position(carriage)

    if home is None or extended is None:
        ctx.fail("carriage slide pose query", "Could not measure carriage positions for the prismatic joint.")
    else:
        ctx.check(
            "carriage slides outward along the beam",
            extended[0] > home[0] + 0.50
            and abs(extended[1] - home[1]) < 1e-5
            and abs(extended[2] - home[2]) < 1e-5,
            details=f"home={home}, extended={extended}",
        )

    with ctx.pose({swing: 0.0, slide: 0.30}):
        straight = ctx.part_world_position(carriage)
        ctx.expect_contact(
            beam,
            carriage,
            contact_tol=0.0005,
            name="mid-span carriage still contacts the beam in the straight pose",
        )

    with ctx.pose({swing: 1.0, slide: 0.30}):
        swung = ctx.part_world_position(carriage)
        ctx.expect_contact(
            beam,
            carriage,
            contact_tol=0.0005,
            name="mid-span carriage stays supported while the beam swings",
        )

    if straight is None or swung is None:
        ctx.fail("beam swing pose query", "Could not measure carriage positions for the revolute joint.")
    else:
        straight_r = math.hypot(straight[0], straight[1])
        swung_r = math.hypot(swung[0], swung[1])
        ctx.check(
            "beam revolute joint sweeps the carriage around the top support",
            swung[1] > straight[1] + 0.35
            and abs(swung_r - straight_r) < 0.02
            and abs(swung[2] - straight[2]) < 0.005,
            details=f"straight={straight}, swung={swung}, radii=({straight_r}, {swung_r})",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
