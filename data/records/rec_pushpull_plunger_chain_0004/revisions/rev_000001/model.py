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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="plunger_chain", assets=ASSETS)

    housing_color = model.material("housing_gray", rgba=(0.35, 0.37, 0.40, 1.0))
    plunger_color = model.material("plunger_black", rgba=(0.14, 0.15, 0.16, 1.0))
    lever_color = model.material("lever_steel", rgba=(0.72, 0.74, 0.76, 1.0))

    housing = model.part("housing")
    plunger = model.part("plunger")
    lever = model.part("front_lever")

    base_length = 0.110
    base_width = 0.028
    base_thickness = 0.004
    rail_length = 0.082
    rail_thickness = 0.005
    rail_height = 0.010
    cheek_length = 0.016
    cheek_thickness = 0.005
    cheek_height = 0.024
    pivot_x = 0.094
    pivot_z = 0.014
    pin_radius = 0.003
    pin_length = base_width - 2.0 * cheek_thickness
    rail_center_y = base_width / 2.0 - rail_thickness / 2.0
    front_fork_center_x = pivot_x
    plunger_height = 0.010
    plunger_width = 0.016
    plunger_length = 0.0564

    housing_shape = (
        cq.Workplane("XY")
        .box(base_length, base_width, base_thickness)
        .translate((base_length / 2.0, 0.0, base_thickness / 2.0))
        .union(
            cq.Workplane("XY")
            .box(rail_length, rail_thickness, rail_height)
            .translate((rail_length / 2.0, rail_center_y, base_thickness + rail_height / 2.0))
        )
        .union(
            cq.Workplane("XY")
            .box(rail_length, rail_thickness, rail_height)
            .translate((rail_length / 2.0, -rail_center_y, base_thickness + rail_height / 2.0))
        )
        .union(
            cq.Workplane("XY")
            .box(cheek_length, cheek_thickness, cheek_height)
            .translate((front_fork_center_x, rail_center_y, cheek_height / 2.0))
        )
        .union(
            cq.Workplane("XY")
            .box(cheek_length, cheek_thickness, cheek_height)
            .translate((front_fork_center_x, -rail_center_y, cheek_height / 2.0))
        )
        .union(
            cq.Workplane("XZ")
            .center(pivot_x, pivot_z)
            .circle(pin_radius)
            .extrude(pin_length / 2.0, both=True)
        )
    )
    housing.visual(
        mesh_from_cadquery(housing_shape, "housing.obj", assets=ASSETS),
        name="housing_shell",
        material=housing_color,
    )

    plunger_profile = cq.Sketch().polygon(
        [
            (0.000, -plunger_height / 2.0),
            (0.000, plunger_height / 2.0),
            (0.047, plunger_height / 2.0),
            (plunger_length, 0.0018),
            (plunger_length, -0.0018),
            (0.047, -plunger_height / 2.0),
        ]
    )
    plunger_shape = (
        cq.Workplane("XZ")
        .placeSketch(plunger_profile)
        .extrude(plunger_width / 2.0, both=True)
    )
    plunger.visual(
        mesh_from_cadquery(plunger_shape, "plunger.obj", assets=ASSETS),
        name="plunger_body",
        material=plunger_color,
    )

    lever_profile = (
        cq.Sketch()
        .polygon(
            [
                (-0.0095, -0.0060),
                (-0.0040, -0.0060),
                (0.0015, -0.0015),
                (0.0120, 0.0120),
                (0.0090, 0.0280),
                (-0.0020, 0.0310),
                (-0.0060, 0.0180),
                (-0.0050, 0.0050),
                (-0.0095, 0.0010),
            ]
        )
        .circle(pin_radius, mode="s")
        .clean()
    )
    lever_shape = (
        cq.Workplane("XZ")
        .placeSketch(lever_profile)
        .extrude(0.004 / 2.0, both=True)
    )
    lever.visual(
        mesh_from_cadquery(lever_shape, "front_lever.obj", assets=ASSETS),
        name="lever_body",
        material=lever_color,
    )

    model.articulation(
        "plunger_slide",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=plunger,
        origin=Origin(xyz=(0.028, 0.0, base_thickness + plunger_height / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.15,
            lower=0.0,
            upper=0.050,
        ),
    )

    model.articulation(
        "lever_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=lever,
        origin=Origin(xyz=(pivot_x, 0.0, pivot_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=0.0,
            upper=math.pi / 4.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    housing = object_model.get_part("housing")
    plunger = object_model.get_part("plunger")
    lever = object_model.get_part("front_lever")
    plunger_slide = object_model.get_articulation("plunger_slide")
    lever_hinge = object_model.get_articulation("lever_hinge")

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

    ctx.expect_contact(plunger, housing, contact_tol=0.0005, name="plunger_supported_by_housing")
    ctx.expect_within(plunger, housing, axes="yz", margin=0.0, name="plunger_guided_in_channel")
    ctx.expect_contact(lever, housing, contact_tol=0.0005, name="lever_runs_on_transverse_pin")
    ctx.expect_overlap(lever, housing, axes="yz", min_overlap=0.004, name="lever_aligned_with_front_fork")
    ctx.expect_gap(
        lever,
        plunger,
        axis="x",
        min_gap=0.0,
        max_gap=0.0005,
        name="plunger_tip_reaches_lever_heel",
    )

    with ctx.pose({plunger_slide: 0.050}):
        ctx.expect_contact(plunger, housing, contact_tol=0.0005, name="plunger_supported_at_full_stroke")
        ctx.expect_within(plunger, housing, axes="yz", margin=0.0, name="plunger_guided_at_full_stroke")

    with ctx.pose({lever_hinge: math.pi / 4.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="lever_clearance_at_max_angle")
        ctx.expect_contact(lever, housing, contact_tol=0.0005, name="lever_stays_pinned_at_max_angle")

    with ctx.pose({lever_hinge: math.radians(24.0)}):
        ctx.expect_overlap(plunger, lever, axes="yz", min_overlap=0.003, name="plunger_and_lever_share_drive_plane")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
