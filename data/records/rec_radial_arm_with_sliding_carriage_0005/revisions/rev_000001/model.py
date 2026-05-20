from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="column_mounted_radial_arm", assets=ASSETS)

    dark_base = model.material("dark_base", rgba=(0.21, 0.23, 0.25, 1.0))
    steel = model.material("steel", rgba=(0.63, 0.66, 0.69, 1.0))
    painted_arm = model.material("painted_arm", rgba=(0.52, 0.56, 0.61, 1.0))
    carriage_gray = model.material("carriage_gray", rgba=(0.35, 0.37, 0.40, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.18, 0.18, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        material=dark_base,
        name="base_plate",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.18, 0.18, 0.02)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
    )

    post = model.part("post")
    post.visual(
        Cylinder(radius=0.018, length=0.32),
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        material=steel,
        name="column",
    )
    post.visual(
        Cylinder(radius=0.03, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.156)),
        material=steel,
        name="support_flange",
    )
    post.inertial = Inertial.from_geometry(
        Cylinder(radius=0.018, length=0.32),
        mass=5.0,
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
    )

    turret_shape = (
        cq.Workplane("XY")
        .circle(0.034)
        .extrude(0.04, both=True)
        .cut(cq.Workplane("XY").circle(0.02).extrude(0.05, both=True))
        .union(
            cq.Workplane("XY")
            .box(0.052, 0.044, 0.032, centered=(False, True, True))
            .translate((0.024, 0.0, 0.0))
        )
    )
    turret = model.part("turret")
    turret.visual(
        mesh_from_cadquery(turret_shape, "turret.obj", assets=ASSETS),
        material=painted_arm,
        name="turret_shell",
    )
    turret.inertial = Inertial.from_geometry(
        Box((0.09, 0.07, 0.06)),
        mass=2.6,
        origin=Origin(),
    )

    beam_shell_shape = (
        cq.Workplane("XY")
        .box(0.27, 0.048, 0.026, centered=(False, True, True))
        .union(cq.Workplane("XY").box(0.045, 0.062, 0.05, centered=(False, True, True)))
        .union(
            cq.Workplane("XY")
            .box(0.21, 0.036, 0.012, centered=(False, True, True))
            .translate((0.15, 0.0, -0.019))
        )
    )
    beam = model.part("beam")
    beam.visual(
        mesh_from_cadquery(beam_shell_shape, "beam_shell.obj", assets=ASSETS),
        material=painted_arm,
        name="beam_shell",
    )
    beam.visual(
        Box((0.21, 0.024, 0.016)),
        origin=Origin(xyz=(0.15, 0.0, -0.022)),
        material=steel,
        name="guide_rail",
    )
    beam.inertial = Inertial.from_geometry(
        Box((0.27, 0.062, 0.05)),
        mass=3.2,
        origin=Origin(xyz=(0.135, 0.0, 0.0)),
    )

    carriage_shape = cq.Workplane("XY").box(0.05, 0.056, 0.03)
    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(carriage_shape, "carriage.obj", assets=ASSETS),
        material=carriage_gray,
        name="carriage_shell",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.05, 0.06, 0.03)),
        mass=1.4,
        origin=Origin(),
    )

    head = model.part("head")
    head.visual(
        Box((0.018, 0.04, 0.018)),
        origin=Origin(xyz=(0.009, 0.0, 0.0)),
        material=painted_arm,
        name="head_mount",
    )
    head.visual(
        Box((0.006, 0.06, 0.04)),
        origin=Origin(xyz=(0.021, 0.0, -0.002)),
        material=dark_base,
        name="head_plate",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.027, 0.06, 0.04)),
        mass=0.9,
        origin=Origin(xyz=(0.0135, 0.0, -0.002)),
    )

    model.articulation(
        "base_to_post",
        ArticulationType.FIXED,
        parent=base,
        child=post,
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
    )
    model.articulation(
        "post_to_turret",
        ArticulationType.REVOLUTE,
        parent=post,
        child=turret,
        origin=Origin(xyz=(0.0, 0.0, 0.2)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.5,
            lower=-math.radians(110.0),
            upper=math.radians(110.0),
        ),
    )
    model.articulation(
        "turret_to_beam",
        ArticulationType.FIXED,
        parent=turret,
        child=beam,
        origin=Origin(xyz=(0.076, 0.0, 0.0)),
    )
    model.articulation(
        "beam_to_carriage",
        ArticulationType.PRISMATIC,
        parent=beam,
        child=carriage,
        origin=Origin(xyz=(0.075, 0.0, -0.045)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.25,
            lower=0.0,
            upper=0.14,
        ),
    )
    model.articulation(
        "carriage_to_head",
        ArticulationType.FIXED,
        parent=carriage,
        child=head,
        origin=Origin(xyz=(0.025, 0.0, -0.01)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    post = object_model.get_part("post")
    turret = object_model.get_part("turret")
    beam = object_model.get_part("beam")
    carriage = object_model.get_part("carriage")
    head = object_model.get_part("head")

    turret_joint = object_model.get_articulation("post_to_turret")
    carriage_joint = object_model.get_articulation("beam_to_carriage")

    base_plate = base.get_visual("base_plate")
    column = post.get_visual("column")
    support_flange = post.get_visual("support_flange")
    turret_shell = turret.get_visual("turret_shell")
    beam_shell = beam.get_visual("beam_shell")
    guide_rail = beam.get_visual("guide_rail")
    carriage_shell = carriage.get_visual("carriage_shell")
    head_mount = head.get_visual("head_mount")
    head_plate = head.get_visual("head_plate")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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

    ctx.expect_contact(post, base, elem_a=column, elem_b=base_plate, contact_tol=0.0015)
    ctx.expect_contact(turret, post, elem_a=turret_shell, elem_b=support_flange, contact_tol=0.0015)
    ctx.expect_contact(beam, turret, elem_a=beam_shell, elem_b=turret_shell, contact_tol=0.0015)
    ctx.expect_contact(carriage, beam, elem_a=carriage_shell, elem_b=guide_rail, contact_tol=0.0015)
    ctx.expect_contact(head, carriage, elem_a=head_mount, elem_b=carriage_shell, contact_tol=0.0015)

    ctx.expect_overlap(turret, post, axes="xy", elem_a=turret_shell, elem_b=support_flange, min_overlap=0.02)
    ctx.expect_overlap(beam, turret, axes="yz", elem_a=beam_shell, elem_b=turret_shell, min_overlap=0.02)
    ctx.expect_overlap(carriage, beam, axes="x", elem_a=carriage_shell, elem_b=guide_rail, min_overlap=0.048)
    ctx.expect_overlap(head, carriage, axes="yz", elem_a=head_mount, elem_b=carriage_shell, min_overlap=0.014)

    ctx.expect_gap(beam, head, axis="z", positive_elem=beam_shell, negative_elem=head_plate, min_gap=0.004, max_gap=0.02)

    turret_axis_ok = tuple(turret_joint.axis) == (0.0, 0.0, 1.0)
    ctx.check("turret_axis_vertical", turret_axis_ok, f"expected vertical z-axis, got {turret_joint.axis}")

    carriage_axis_ok = tuple(carriage_joint.axis) == (1.0, 0.0, 0.0)
    ctx.check("carriage_axis_along_beam", carriage_axis_ok, f"expected beam x-axis slide, got {carriage_joint.axis}")

    turret_range = turret_joint.motion_limits.upper - turret_joint.motion_limits.lower
    ctx.check(
        "turret_range_is_wide",
        turret_joint.motion_limits.lower <= -math.radians(90.0)
        and turret_joint.motion_limits.upper >= math.radians(90.0)
        and turret_range >= math.radians(180.0),
        f"turret range {math.degrees(turret_range):.1f} deg is too small",
    )
    carriage_range = carriage_joint.motion_limits.upper - carriage_joint.motion_limits.lower
    ctx.check(
        "carriage_stroke_is_140mm",
        abs(carriage_range - 0.14) <= 0.002,
        f"carriage stroke is {carriage_range:.4f} m",
    )

    with ctx.pose({turret_joint: 0.0, carriage_joint: 0.0}):
        beam_rest = ctx.part_world_position(beam)
        carriage_rest = ctx.part_world_position(carriage)
        head_rest = ctx.part_world_position(head)
        ctx.expect_contact(carriage, beam, elem_a=carriage_shell, elem_b=guide_rail, contact_tol=0.0015)
        ctx.expect_overlap(carriage, beam, axes="x", elem_a=carriage_shell, elem_b=guide_rail, min_overlap=0.048)

    with ctx.pose({turret_joint: math.radians(90.0), carriage_joint: 0.0}):
        beam_turned = ctx.part_world_position(beam)
        head_turned = ctx.part_world_position(head)
        rest_radius = math.hypot(beam_rest[0], beam_rest[1])
        turned_radius = math.hypot(beam_turned[0], beam_turned[1])
        ctx.check(
            "turret_rotation_preserves_beam_radius",
            abs(rest_radius - turned_radius) <= 0.002 and abs(beam_rest[2] - beam_turned[2]) <= 1e-6,
            f"beam radius changed from {rest_radius:.4f} to {turned_radius:.4f}",
        )
        ctx.check(
            "turret_rotation_swings_head_sideways",
            abs(head_turned[1]) > abs(head_rest[1]) + 0.1,
            f"head did not sweep sideways enough: rest={head_rest}, turned={head_turned}",
        )
        ctx.expect_contact(beam, turret, elem_a=beam_shell, elem_b=turret_shell, contact_tol=0.0015)

    with ctx.pose({turret_joint: 0.0, carriage_joint: 0.14}):
        head_out = ctx.part_world_position(head)
        carriage_out = ctx.part_world_position(carriage)
        ctx.check(
            "carriage_motion_translates_head_140mm",
            abs((head_out[0] - head_rest[0]) - 0.14) <= 0.003
            and abs(head_out[1] - head_rest[1]) <= 1e-6
            and abs(head_out[2] - head_rest[2]) <= 1e-6,
            f"head translation from rest to outboard was {head_out[0] - head_rest[0]:.4f} m",
        )
        ctx.check(
            "carriage_motion_translates_carriage_140mm",
            abs((carriage_out[0] - carriage_rest[0]) - 0.14) <= 0.003
            and abs(carriage_out[1] - carriage_rest[1]) <= 1e-6
            and abs(carriage_out[2] - carriage_rest[2]) <= 1e-6,
            f"carriage translation from rest to outboard was {carriage_out[0] - carriage_rest[0]:.4f} m",
        )
        ctx.expect_contact(carriage, beam, elem_a=carriage_shell, elem_b=guide_rail, contact_tol=0.0015)
        ctx.expect_overlap(carriage, beam, axes="x", elem_a=carriage_shell, elem_b=guide_rail, min_overlap=0.048)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
