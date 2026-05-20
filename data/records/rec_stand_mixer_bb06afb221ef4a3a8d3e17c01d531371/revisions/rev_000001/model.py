from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    mesh_from_cadquery,
)


def make_pedestal_shape() -> cq.Workplane:
    base = (
        cq.Workplane("XY")
        .ellipse(0.165, 0.115)
        .extrude(0.030)
    )

    rear_column = (
        cq.Workplane("XY")
        .workplane(offset=0.028)
        .center(-0.040, 0.0)
        .ellipse(0.075, 0.076)
        .workplane(offset=0.078)
        .center(-0.064, 0.0)
        .ellipse(0.048, 0.058)
        .workplane(offset=0.105)
        .center(-0.086, 0.0)
        .ellipse(0.028, 0.046)
        .loft(combine=True)
    )

    bowl_plate = (
        cq.Workplane("XY")
        .center(0.102, 0.0)
        .circle(0.028)
        .extrude(0.006)
        .translate((0.0, 0.0, 0.0523))
    )
    bowl_riser = (
        cq.Workplane("XY")
        .center(0.102, 0.0)
        .circle(0.028)
        .extrude(0.0583)
    )

    hinge_fin = (
        cq.Workplane("XY")
        .center(-0.094, 0.0)
        .rect(0.030, 0.070)
        .extrude(0.313)
    )

    return base.union(rear_column).union(bowl_riser).union(bowl_plate).union(hinge_fin)


def make_head_shape() -> cq.Workplane:
    shell = (
        cq.Workplane("YZ")
        .workplane(offset=0.000)
        .ellipse(0.030, 0.036)
        .workplane(offset=0.065)
        .ellipse(0.058, 0.052)
        .workplane(offset=0.100)
        .ellipse(0.075, 0.062)
        .workplane(offset=0.085)
        .ellipse(0.070, 0.060)
        .workplane(offset=0.072)
        .ellipse(0.050, 0.048)
        .loft(combine=True)
        .translate((0.006, 0.0, 0.058))
    )

    hinge_barrel = (
        cq.Workplane("XZ")
        .center(0.000, 0.000)
        .circle(0.014)
        .extrude(0.042, both=True)
    )
    rear_bridge = (
        cq.Workplane("XY")
        .center(0.008, 0.0)
        .rect(0.028, 0.046)
        .extrude(0.060)
        .translate((0.0, 0.0, -0.014))
    )

    hub_fairing = (
        cq.Workplane("XZ")
        .center(0.205, -0.020)
        .circle(0.030)
        .extrude(0.050, both=True)
    )
    hub_ring = (
        cq.Workplane("XZ")
        .center(0.205, -0.055)
        .circle(0.017)
        .circle(0.008)
        .extrude(0.024, both=True)
    )
    shaft_relief = (
        cq.Workplane("XY")
        .center(0.205, 0.0)
        .circle(0.0085)
        .extrude(-0.090)
        .translate((0.0, 0.0, -0.010))
    )

    return shell.union(hinge_barrel).union(rear_bridge).union(hub_fairing).union(hub_ring).cut(shaft_relief)


def make_bowl_shape() -> LatheGeometry:
    outer_profile = [
        (0.030, 0.000),
        (0.046, 0.008),
        (0.050, 0.016),
        (0.082, 0.045),
        (0.103, 0.078),
        (0.112, 0.092),
    ]
    inner_profile = [
        (0.000, 0.004),
        (0.022, 0.012),
        (0.071, 0.045),
        (0.094, 0.078),
        (0.102, 0.088),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=64,
        end_cap="round",
        lip_samples=10,
    )


def make_beater_shape() -> cq.Workplane:
    top_shank = cq.Workplane("XY").circle(0.0080).extrude(0.012)
    shaft = cq.Workplane("XY").circle(0.0065).extrude(-0.062)

    paddle_outline = (
        cq.Workplane("XZ")
        .center(0.0, -0.102)
        .moveTo(0.000, 0.035)
        .spline(
            [
                (0.020, 0.020),
                (0.024, -0.010),
                (0.012, -0.050),
                (0.000, -0.060),
                (-0.012, -0.050),
                (-0.024, -0.010),
                (-0.020, 0.020),
            ]
        )
        .close()
        .extrude(0.008, both=True)
    )
    window_cut = (
        cq.Workplane("XZ")
        .center(0.0, -0.103)
        .moveTo(0.000, 0.018)
        .spline(
            [
                (0.010, 0.010),
                (0.013, -0.008),
                (0.008, -0.030),
                (0.000, -0.040),
                (-0.008, -0.030),
                (-0.013, -0.008),
                (-0.010, 0.010),
            ]
        )
        .close()
        .extrude(0.012, both=True)
    )
    lower_spine = (
        cq.Workplane("XY")
        .circle(0.0045)
        .extrude(-0.136)
    )

    return top_shank.union(shaft).union(paddle_outline.cut(window_cut)).union(lower_spine)


def make_shield_shape() -> cq.Workplane:
    canopy = (
        cq.Workplane("XY")
        .center(0.103, 0.0)
        .circle(0.124)
        .circle(0.046)
        .extrude(0.018)
        .translate((0.0, 0.0, 0.010))
    )
    rear_cut = (
        cq.Workplane("XY")
        .center(-0.010, 0.0)
        .rect(0.090, 0.240)
        .extrude(0.030)
        .translate((0.0, 0.0, 0.010))
    )
    feed_slot = (
        cq.Workplane("XY")
        .center(0.188, 0.0)
        .rect(0.040, 0.036)
        .extrude(0.024)
        .translate((0.0, 0.0, 0.010))
    )
    chute = (
        cq.Workplane("XY")
        .center(0.190, 0.0)
        .rect(0.052, 0.055)
        .extrude(0.040)
        .translate((0.0, 0.0, 0.010))
    )
    sleeve = (
        cq.Workplane("XY")
        .circle(0.010)
        .circle(0.006)
        .extrude(0.022)
        .translate((0.0, 0.0, -0.011))
    )
    arm = (
        cq.Workplane("XY")
        .center(0.035, 0.0)
        .rect(0.070, 0.026)
        .extrude(0.016)
        .translate((0.0, 0.0, 0.008))
    )
    return canopy.cut(rear_cut).cut(feed_slot).union(chute).union(sleeve).union(arm)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilt_head_stand_mixer")

    body_finish = model.material("body_finish", rgba=(0.78, 0.18, 0.15, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.18, 0.18, 0.19, 1.0))
    steel = model.material("steel", rgba=(0.84, 0.85, 0.87, 1.0))
    clear_guard = model.material("clear_guard", rgba=(0.86, 0.90, 0.94, 0.40))

    pedestal = model.part("pedestal")
    pedestal.visual(
        mesh_from_cadquery(make_pedestal_shape(), "pedestal"),
        material=body_finish,
        name="body_shell",
    )

    bowl = model.part("bowl")
    bowl.visual(
        mesh_from_geometry(make_bowl_shape(), "bowl"),
        material=steel,
        name="bowl_shell",
    )
    bowl.visual(
        Cylinder(radius=0.006, length=0.020),
        origin=Origin(xyz=(-0.103, 0.0, 0.098)),
        material=steel,
        name="shield_pivot",
    )

    head = model.part("head")
    head.visual(
        mesh_from_cadquery(make_head_shape(), "head"),
        material=body_finish,
        name="head_shell",
    )

    beater = model.part("beater")
    beater.visual(
        mesh_from_cadquery(make_beater_shape(), "beater"),
        material=steel,
        name="beater_body",
    )

    shield = model.part("shield")
    shield.visual(
        mesh_from_cadquery(make_shield_shape(), "shield"),
        material=clear_guard,
        name="shield_body",
    )

    model.articulation(
        "pedestal_to_bowl",
        ArticulationType.FIXED,
        parent=pedestal,
        child=bowl,
        origin=Origin(xyz=(0.102, 0.0, 0.058)),
    )
    head_hinge = model.articulation(
        "pedestal_to_head",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=head,
        origin=Origin(xyz=(-0.102, 0.0, 0.327)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(62.0),
        ),
    )
    head_hinge.meta["qc_samples"] = [0.0, math.radians(40.0), math.radians(62.0)]

    beater_spin = model.articulation(
        "head_to_beater",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=beater,
        origin=Origin(xyz=(0.205, 0.0, -0.0293)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=20.0),
    )
    beater_spin.meta["qc_samples"] = [0.0, math.pi / 2.0]

    shield_hinge = model.articulation(
        "bowl_to_shield",
        ArticulationType.REVOLUTE,
        parent=bowl,
        child=shield,
        origin=Origin(xyz=(-0.103, 0.0, 0.098)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(105.0),
        ),
    )
    shield_hinge.meta["qc_samples"] = [0.0, math.radians(70.0), math.radians(105.0)]

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bowl = object_model.get_part("bowl")
    beater = object_model.get_part("beater")
    head = object_model.get_part("head")
    shield = object_model.get_part("shield")
    head_hinge = object_model.get_articulation("pedestal_to_head")
    shield_hinge = object_model.get_articulation("bowl_to_shield")
    upper = head_hinge.motion_limits.upper if head_hinge.motion_limits else None
    shield_upper = shield_hinge.motion_limits.upper if shield_hinge.motion_limits else None

    ctx.allow_overlap(
        bowl,
        "pedestal",
        reason="The bowl foot is intentionally simplified as nesting onto the pedestal's locator post.",
    )
    ctx.allow_overlap(
        bowl,
        shield,
        elem_a="bowl_shell",
        elem_b="shield_body",
        reason="The pouring shield is intentionally modeled as a clip-on cover that nests slightly over the bowl rim.",
    )
    ctx.allow_overlap(
        bowl,
        shield,
        elem_a="shield_pivot",
        elem_b="shield_body",
        reason="The pouring shield's hinge sleeve intentionally wraps around the bowl-mounted pivot boss.",
    )
    ctx.allow_isolated_part(
        beater,
        reason="The beater shank is intentionally modeled with a slight clearance inside the drive hub.",
    )

    ctx.expect_origin_distance(
        beater,
        bowl,
        axes="xy",
        max_dist=0.012,
        name="beater axis is centered over the bowl",
    )
    ctx.expect_within(
        beater,
        bowl,
        axes="xy",
        margin=0.0,
        name="beater stays within the bowl footprint",
    )

    rest_pos = ctx.part_world_position(beater)
    with ctx.pose({head_hinge: upper or 0.0}):
        raised_pos = ctx.part_world_position(beater)

    ctx.check(
        "tilt head lifts the beater clear of the bowl",
        rest_pos is not None
        and raised_pos is not None
        and raised_pos[2] > rest_pos[2] + 0.14,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )
    ctx.expect_contact(
        head,
        "pedestal",
        contact_tol=0.001,
        name="rear hinge housing stays seated on the pedestal support",
    )
    ctx.expect_overlap(
        shield,
        bowl,
        axes="xy",
        min_overlap=0.050,
        name="shield covers the bowl opening when closed",
    )

    closed_aabb = ctx.part_world_aabb(shield)
    with ctx.pose({shield_hinge: shield_upper or 0.0}):
        open_aabb = ctx.part_world_aabb(shield)

    def aabb_center_y(aabb):
        if aabb is None:
            return None
        lower, upper_box = aabb
        return (lower[1] + upper_box[1]) * 0.5

    closed_y = aabb_center_y(closed_aabb)
    open_y = aabb_center_y(open_aabb)
    ctx.check(
        "pouring shield swings aside from the bowl centerline",
        closed_y is not None and open_y is not None and abs(open_y - closed_y) > 0.05,
        details=f"closed_y={closed_y}, open_y={open_y}",
    )

    return ctx.report()


object_model = build_object_model()
