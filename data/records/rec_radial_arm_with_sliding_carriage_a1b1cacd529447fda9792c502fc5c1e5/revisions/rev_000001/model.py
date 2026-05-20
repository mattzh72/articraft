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


def _pedestal_body_shape() -> cq.Workplane:
    base = (
        cq.Workplane("XY")
        .box(0.56, 0.42, 0.04, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.014)
    )
    plinth = (
        cq.Workplane("XY")
        .box(0.24, 0.28, 0.08, centered=(True, True, False))
        .translate((0.0, 0.0, 0.04))
        .edges("|Z")
        .fillet(0.012)
    )
    column = (
        cq.Workplane("XY")
        .box(0.18, 0.22, 0.64, centered=(True, True, False))
        .translate((0.0, 0.0, 0.12))
        .edges("|Z")
        .fillet(0.02)
    )
    shoulder_block = (
        cq.Workplane("XY")
        .box(0.28, 0.32, 0.12, centered=(True, True, False))
        .translate((0.0, 0.0, 0.76))
        .edges("|Z")
        .fillet(0.014)
    )
    front_gusset = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.07, 0.0),
                (0.07, 0.0),
                (0.07, 0.18),
                (0.0, 0.28),
                (-0.07, 0.18),
            ]
        )
        .close()
        .extrude(0.18)
        .translate((0.05, -0.09, 0.60))
    )
    return base.union(plinth).union(column).union(shoulder_block).union(front_gusset)


def _turntable_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .cylinder(0.042, 0.14)
        .translate((0.0, 0.0, 0.878))
    )


def _yaw_plate_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .cylinder(0.03, 0.135)
        .translate((0.0, 0.0, 0.015))
    )


def _beam_track_shape() -> cq.Workplane:
    shoulder = cq.Workplane("XY").box(
        0.22,
        0.20,
        0.12,
        centered=(False, True, False),
    ).translate((-0.04, 0.0, 0.0))
    arm = cq.Workplane("XY").box(
        0.82,
        0.10,
        0.06,
        centered=(False, True, False),
    ).translate((0.12, 0.0, 0.02))
    top_cap = cq.Workplane("XY").box(
        0.18,
        0.16,
        0.045,
        centered=(False, True, False),
    ).translate((0.00, 0.0, 0.075))
    return shoulder.union(arm).union(top_cap)


def _beam_end_stop_shape() -> cq.Workplane:
    return cq.Workplane("XY").box(
        0.016,
        0.18,
        0.12,
        centered=(False, True, False),
    ).translate((0.94, 0.0, 0.0))


def _carriage_shape() -> cq.Workplane:
    shell = cq.Workplane("XY").box(0.20, 0.18, 0.16).cut(
        cq.Workplane("XY").box(0.24, 0.116, 0.072)
    )
    top_guide = cq.Workplane("XY").box(
        0.14,
        0.07,
        0.008,
        centered=(True, True, False),
    ).translate((0.0, 0.0, 0.03))
    crown = cq.Workplane("XY").box(
        0.12,
        0.13,
        0.03,
        centered=(True, True, False),
    ).translate((0.0, 0.0, 0.08))
    return shell.union(top_guide).union(crown)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="shop_radial_arm_carriage")

    model.material("pedestal_gray", rgba=(0.34, 0.37, 0.40, 1.0))
    model.material("beam_gray", rgba=(0.58, 0.60, 0.63, 1.0))
    model.material("carriage_orange", rgba=(0.84, 0.48, 0.20, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        mesh_from_cadquery(_pedestal_body_shape(), "pedestal_body"),
        material="pedestal_gray",
        name="body",
    )
    pedestal.visual(
        mesh_from_cadquery(_turntable_shape(), "pedestal_turntable"),
        material="beam_gray",
        name="turntable",
    )

    beam = model.part("beam")
    beam.visual(
        mesh_from_cadquery(_yaw_plate_shape(), "beam_yaw_plate"),
        material="beam_gray",
        name="yaw_plate",
    )
    beam.visual(
        mesh_from_cadquery(_beam_track_shape(), "beam_track"),
        material="beam_gray",
        name="track",
    )
    beam.visual(
        mesh_from_cadquery(_beam_end_stop_shape(), "beam_end_stop"),
        material="beam_gray",
        name="end_stop",
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_shape(), "carriage_body"),
        material="carriage_orange",
        name="body",
    )

    model.articulation(
        "pedestal_to_beam_yaw",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, 0.899)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.8,
            lower=-1.2,
            upper=1.2,
        ),
    )
    model.articulation(
        "beam_to_carriage_slide",
        ArticulationType.PRISMATIC,
        parent=beam,
        child=carriage,
        origin=Origin(xyz=(0.34, 0.0, 0.05)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.35,
            lower=0.0,
            upper=0.50,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    beam = object_model.get_part("beam")
    carriage = object_model.get_part("carriage")
    beam_yaw = object_model.get_articulation("pedestal_to_beam_yaw")
    carriage_slide = object_model.get_articulation("beam_to_carriage_slide")

    turntable = pedestal.get_visual("turntable")
    yaw_plate = beam.get_visual("yaw_plate")
    track = beam.get_visual("track")
    end_stop = beam.get_visual("end_stop")
    carriage_body = carriage.get_visual("body")

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

    ctx.check(
        "assembly_parts_present",
        pedestal is not None and beam is not None and carriage is not None,
        "Expected pedestal, beam, and carriage parts.",
    )
    ctx.expect_contact(
        beam,
        pedestal,
        elem_a=yaw_plate,
        elem_b=turntable,
        contact_tol=0.001,
        name="beam_turntable_contacts_pedestal",
    )
    ctx.expect_contact(
        carriage,
        beam,
        elem_a=carriage_body,
        elem_b=track,
        contact_tol=0.001,
        name="carriage_contacts_beam_track",
    )
    ctx.expect_overlap(
        carriage,
        beam,
        axes="yz",
        elem_a=carriage_body,
        elem_b=track,
        min_overlap=0.095,
        name="carriage_wraps_beam_section",
    )

    with ctx.pose({beam_yaw: 0.0, carriage_slide: 0.0}):
        carriage_retracted = ctx.part_world_position(carriage)
    with ctx.pose({beam_yaw: 0.0, carriage_slide: 0.50}):
        carriage_extended = ctx.part_world_position(carriage)
        ctx.expect_gap(
            beam,
            carriage,
            axis="x",
            positive_elem=end_stop,
            negative_elem=carriage_body,
            max_penetration=1e-5,
            max_gap=0.03,
            name="end_stop_captures_carriage_at_full_extension",
        )
    with ctx.pose({beam_yaw: 0.6, carriage_slide: 0.0}):
        carriage_swung = ctx.part_world_position(carriage)

    slide_ok = (
        carriage_retracted is not None
        and carriage_extended is not None
        and (carriage_extended[0] - carriage_retracted[0]) > 0.45
        and abs(carriage_extended[1] - carriage_retracted[1]) < 1e-4
    )
    ctx.check(
        "carriage_translates_along_beam_axis",
        slide_ok,
        f"Retracted={carriage_retracted}, extended={carriage_extended}",
    )

    yaw_ok = (
        carriage_retracted is not None
        and carriage_swung is not None
        and carriage_swung[1] > carriage_retracted[1] + 0.15
        and carriage_swung[0] < carriage_retracted[0] - 0.04
    )
    ctx.check(
        "beam_yaws_carriage_around_pedestal",
        yaw_ok,
        f"Straight={carriage_retracted}, yawed={carriage_swung}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
