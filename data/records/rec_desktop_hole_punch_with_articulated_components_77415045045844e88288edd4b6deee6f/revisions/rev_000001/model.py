from __future__ import annotations

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


BODY_L = 0.120
BODY_W = 0.055
BODY_TOP_H = 0.025
BODY_CAVITY_L = 0.072
BODY_CAVITY_W = 0.045
BODY_CAVITY_H = 0.0165

HANDLE_W = 0.064
HANDLE_TUBE_R = 0.0034
HANDLE_TUBE_L = 0.028
HANDLE_HINGE_X = -0.054
HANDLE_HINGE_Z = 0.0320

FLAP_W = 0.047
FLAP_TUBE_R = 0.0018
FLAP_TUBE_L = 0.022
FLAP_HINGE_X = -0.025
FLAP_HINGE_Z = -0.0024


def _make_body_shape() -> cq.Workplane:
    body_profile = [
        (-0.060, 0.000),
        (-0.058, 0.015),
        (-0.030, 0.0228),
        (0.010, 0.0247),
        (0.042, 0.0215),
        (0.057, 0.0172),
        (0.060, 0.0128),
        (0.056, 0.000),
    ]
    body = (
        cq.Workplane("XZ")
        .polyline(body_profile)
        .close()
        .extrude(BODY_W / 2.0, both=True)
    )

    cavity = (
        cq.Workplane("XY")
        .workplane(offset=0.0)
        .center(0.014, 0.0)
        .rect(BODY_CAVITY_L, BODY_CAVITY_W)
        .extrude(BODY_CAVITY_H)
    )
    body = body.cut(cavity)

    paper_slot = (
        cq.Workplane("XY")
        .workplane(offset=0.006)
        .center(0.052, 0.0)
        .rect(0.013, 0.041)
        .extrude(0.0045)
    )
    body = body.cut(paper_slot)

    punch_cuts = (
        cq.Workplane("XY")
        .workplane(offset=-0.001)
        .pushPoints([(0.014, -0.015), (0.014, 0.015)])
        .circle(0.0033)
        .extrude(0.032)
    )
    body = body.cut(punch_cuts)

    handle_bracket = cq.Workplane("XY").box(0.008, 0.008, 0.018)
    body = body.union(handle_bracket.translate((HANDLE_HINGE_X, -0.018, 0.0225)))
    body = body.union(handle_bracket.translate((HANDLE_HINGE_X, 0.018, 0.0225)))

    flap_lug = cq.Workplane("XY").box(0.006, 0.008, 0.0048)
    body = body.union(flap_lug.translate((FLAP_HINGE_X, -0.015, -0.0021)))
    body = body.union(flap_lug.translate((FLAP_HINGE_X, 0.015, -0.0021)))

    return body


def _make_handle_shape() -> cq.Workplane:
    handle_profile = [
        (0.000, -0.0015),
        (0.006, 0.0015),
        (0.022, 0.0078),
        (0.084, 0.0090),
        (0.106, 0.0065),
        (0.118, 0.0025),
        (0.118, -0.0055),
        (0.108, -0.0045),
        (0.088, -0.0010),
        (0.020, 0.0005),
        (0.000, 0.0005),
    ]
    lever = (
        cq.Workplane("XZ")
        .polyline(handle_profile)
        .close()
        .extrude(HANDLE_W / 2.0, both=True)
    )
    hinge_tube = (
        cq.Workplane("XZ")
        .center(0.0, 0.0)
        .circle(HANDLE_TUBE_R)
        .extrude(HANDLE_TUBE_L / 2.0, both=True)
    )
    handle = lever.union(hinge_tube)
    side_relief = cq.Workplane("XY").box(0.016, 0.017, 0.020)
    handle = handle.cut(side_relief.translate((0.002, -0.023, 0.003)))
    handle = handle.cut(side_relief.translate((0.002, 0.023, 0.003)))

    return handle


def _make_flap_shape() -> cq.Workplane:
    flap_profile = [
        (0.000, -0.0010),
        (0.008, -0.0003),
        (0.070, -0.0004),
        (0.076, -0.0010),
        (0.076, -0.0044),
        (0.068, -0.0057),
        (0.008, -0.0047),
        (0.000, -0.0025),
    ]
    panel = (
        cq.Workplane("XZ")
        .polyline(flap_profile)
        .close()
        .extrude(FLAP_W / 2.0, both=True)
    )
    hinge_tube = (
        cq.Workplane("XZ")
        .center(0.0, 0.0)
        .circle(FLAP_TUBE_R)
        .extrude(FLAP_TUBE_L / 2.0, both=True)
    )

    return panel.union(hinge_tube)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_hole_punch")

    model.material("painted_steel", rgba=(0.57, 0.60, 0.63, 1.0))
    model.material("charcoal_handle", rgba=(0.14, 0.15, 0.16, 1.0))
    model.material("dark_bin", rgba=(0.20, 0.22, 0.24, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_make_body_shape(), "body"),
        material="painted_steel",
        name="body_shell",
    )

    handle = model.part("handle")
    handle.visual(
        mesh_from_cadquery(_make_handle_shape(), "handle"),
        material="charcoal_handle",
        name="handle_shell",
    )

    bin_flap = model.part("bin_flap")
    bin_flap.visual(
        mesh_from_cadquery(_make_flap_shape(), "bin_flap"),
        material="dark_bin",
        name="flap_panel",
    )

    model.articulation(
        "handle_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(HANDLE_HINGE_X, 0.0, HANDLE_HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.10, effort=12.0, velocity=2.0),
    )
    model.articulation(
        "flap_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=bin_flap,
        origin=Origin(xyz=(FLAP_HINGE_X, 0.0, FLAP_HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=2.20, effort=4.0, velocity=2.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    handle = object_model.get_part("handle")
    bin_flap = object_model.get_part("bin_flap")
    handle_hinge = object_model.get_articulation("handle_hinge")
    flap_hinge = object_model.get_articulation("flap_hinge")

    ctx.expect_overlap(
        handle,
        body,
        axes="xy",
        min_overlap=0.046,
        name="handle covers the punch body in plan",
    )
    ctx.expect_overlap(
        bin_flap,
        body,
        axes="xy",
        min_overlap=0.036,
        name="bin flap covers the underside cavity footprint",
    )

    closed_handle_aabb = ctx.part_world_aabb(handle)
    with ctx.pose({handle_hinge: 1.10}):
        open_handle_aabb = ctx.part_world_aabb(handle)
    ctx.check(
        "handle opens upward from the rear hinge",
        closed_handle_aabb is not None
        and open_handle_aabb is not None
        and open_handle_aabb[1][2] > closed_handle_aabb[1][2] + 0.028,
        details=f"closed={closed_handle_aabb}, open={open_handle_aabb}",
    )

    closed_flap_aabb = ctx.part_world_aabb(bin_flap)
    with ctx.pose({flap_hinge: 2.20}):
        open_flap_aabb = ctx.part_world_aabb(bin_flap)
    ctx.check(
        "bin flap drops downward when opened",
        closed_flap_aabb is not None
        and open_flap_aabb is not None
        and open_flap_aabb[0][2] < closed_flap_aabb[0][2] - 0.030,
        details=f"closed={closed_flap_aabb}, open={open_flap_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
