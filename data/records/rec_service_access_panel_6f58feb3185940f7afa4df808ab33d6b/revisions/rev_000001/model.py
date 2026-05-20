from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelFace,
    BezelGeometry,
    Box,
    Cylinder,
    ExtrudeGeometry,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_service_access_panel")

    painted_metal = model.material("warm_white_powdercoat", rgba=(0.82, 0.84, 0.82, 1.0))
    satin_edge = model.material("satin_graphite_edge", rgba=(0.18, 0.19, 0.19, 1.0))
    dark_cavity = model.material("deep_shadow_reveal", rgba=(0.025, 0.027, 0.028, 1.0))
    polymer = model.material("matte_graphite_polymer", rgba=(0.06, 0.065, 0.07, 1.0))
    elastomer = model.material("soft_black_elastomer", rgba=(0.005, 0.006, 0.006, 1.0))
    brushed_pin = model.material("brushed_stainless", rgba=(0.62, 0.64, 0.62, 1.0))

    frame = model.part("frame")

    # A single, rounded, recessed surround reads as a manufactured metal frame
    # instead of four loose bars.  Local bevel depth is rotated onto world X.
    surround = BezelGeometry(
        opening_size=(0.680, 0.570),
        outer_size=(0.820, 0.620),
        depth=0.050,
        opening_shape="rounded_rect",
        outer_shape="rounded_rect",
        opening_corner_radius=0.030,
        outer_corner_radius=0.045,
        face=BezelFace(style="radiused_step", front_lip=0.004, fillet=0.002),
    )
    frame.visual(
        mesh_from_geometry(surround, "service_surround"),
        origin=Origin(xyz=(0.020, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=painted_metal,
        name="surround",
    )

    # A narrow elastomer reveal ring sits inside the frame opening and defines
    # the crisp shadow seam around the removable service door.
    frame.visual(
        Box((0.005, 0.540, 0.010)),
        origin=Origin(xyz=(0.048, 0.000, 0.345)),
        material=elastomer,
        name="top_seal",
    )
    frame.visual(
        Box((0.005, 0.540, 0.010)),
        origin=Origin(xyz=(0.048, 0.000, -0.345)),
        material=elastomer,
        name="bottom_seal",
    )
    frame.visual(
        Box((0.005, 0.010, 0.650)),
        origin=Origin(xyz=(0.048, -0.285, 0.000)),
        material=elastomer,
        name="hinge_side_seal",
    )
    frame.visual(
        Box((0.005, 0.010, 0.650)),
        origin=Origin(xyz=(0.048, 0.285, 0.000)),
        material=elastomer,
        name="latch_side_seal",
    )

    # The fixed half of the hinge is visibly mounted to the left frame rail.
    frame.visual(
        Box((0.014, 0.034, 0.610)),
        origin=Origin(xyz=(0.040, -0.274, 0.000)),
        material=satin_edge,
        name="fixed_hinge_leaf",
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.220),
        origin=Origin(xyz=(0.055, -0.256, 0.000)),
        material=brushed_pin,
        name="fixed_hinge_knuckle",
    )

    # A latch-side keeper plate on the opposite jamb makes the closure side
    # unambiguous; the rotating cam sweeps behind this plate rather than into it.
    frame.visual(
        Box((0.010, 0.028, 0.130)),
        origin=Origin(xyz=(0.046, 0.275, 0.000)),
        material=satin_edge,
        name="strike_keeper",
    )
    frame.visual(
        Box((0.003, 0.008, 0.075)),
        origin=Origin(xyz=(0.052, 0.261, 0.000)),
        material=dark_cavity,
        name="keeper_shadow_slot",
    )

    door = model.part("door")
    door_skin_profile = rounded_rect_profile(0.646, 0.474, 0.024, corner_segments=10)
    door.visual(
        mesh_from_geometry(ExtrudeGeometry(door_skin_profile, 0.024), "door_skin"),
        origin=Origin(xyz=(-0.024, 0.247, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=painted_metal,
        name="door_skin",
    )
    # Subtle dark inlays convey a tight reveal and a recessed finger-safe edge.
    door.visual(
        Box((0.002, 0.410, 0.005)),
        origin=Origin(xyz=(-0.011, 0.247, 0.298)),
        material=dark_cavity,
        name="top_reveal_line",
    )
    door.visual(
        Box((0.002, 0.410, 0.005)),
        origin=Origin(xyz=(-0.011, 0.247, -0.298)),
        material=dark_cavity,
        name="bottom_reveal_line",
    )
    door.visual(
        Box((0.002, 0.005, 0.560)),
        origin=Origin(xyz=(-0.011, 0.036, 0.000)),
        material=dark_cavity,
        name="hinge_reveal_line",
    )
    door.visual(
        Box((0.002, 0.005, 0.560)),
        origin=Origin(xyz=(-0.011, 0.460, 0.000)),
        material=dark_cavity,
        name="latch_reveal_line",
    )
    door.visual(
        Box((0.003, 0.120, 0.025)),
        origin=Origin(xyz=(-0.013, 0.245, -0.235)),
        material=satin_edge,
        name="service_badge",
    )

    # Moving hinge leaf and alternating knuckles are attached to the door edge.
    door.visual(
        Box((0.010, 0.034, 0.200)),
        origin=Origin(xyz=(-0.010, 0.023, 0.230)),
        material=satin_edge,
        name="upper_hinge_leaf",
    )
    door.visual(
        Box((0.010, 0.034, 0.200)),
        origin=Origin(xyz=(-0.010, 0.023, -0.230)),
        material=satin_edge,
        name="lower_hinge_leaf",
    )
    door.visual(
        Cylinder(radius=0.011, length=0.170),
        origin=Origin(xyz=(0.000, 0.000, 0.230)),
        material=brushed_pin,
        name="upper_hinge_knuckle",
    )
    door.visual(
        Cylinder(radius=0.011, length=0.170),
        origin=Origin(xyz=(0.000, 0.000, -0.230)),
        material=brushed_pin,
        name="lower_hinge_knuckle",
    )

    door_hinge = model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=door,
        origin=Origin(xyz=(0.055, -0.256, 0.000)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=0.0, upper=1.75),
    )

    latch = model.part("latch")
    thumb_turn = KnobGeometry(
        0.052,
        0.018,
        body_style="faceted",
        top_diameter=0.042,
        edge_radius=0.0012,
        grip=KnobGrip(style="ribbed", count=16, depth=0.0008, width=0.0015),
        indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
        center=False,
    )
    latch.visual(
        mesh_from_geometry(thumb_turn, "thumb_turn"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polymer,
        name="thumb_turn",
    )
    latch.visual(
        Cylinder(radius=0.006, length=0.068),
        origin=Origin(xyz=(-0.034, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_pin,
        name="latch_shaft",
    )
    latch.visual(
        Box((0.012, 0.084, 0.016)),
        origin=Origin(xyz=(-0.060, 0.042, 0.000)),
        material=satin_edge,
        name="cam_bar",
    )

    model.articulation(
        "latch_turn",
        ArticulationType.REVOLUTE,
        parent=door,
        child=latch,
        origin=Origin(xyz=(-0.012, 0.438, 0.000)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=math.pi / 2.0),
    )

    # Retain handles to make static analyzers happy when names change.
    _ = door_hinge
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    door = object_model.get_part("door")
    latch = object_model.get_part("latch")
    door_hinge = object_model.get_articulation("door_hinge")
    latch_turn = object_model.get_articulation("latch_turn")

    ctx.allow_overlap(
        door,
        latch,
        elem_a="door_skin",
        elem_b="latch_shaft",
        reason="The rotary latch shaft is intentionally captured through a tight bushing in the simplified solid door skin.",
    )
    ctx.expect_within(
        latch,
        door,
        axes="yz",
        inner_elem="latch_shaft",
        outer_elem="door_skin",
        margin=0.001,
        name="latch shaft is centered through the door skin",
    )
    ctx.expect_overlap(
        latch,
        door,
        axes="x",
        elem_a="latch_shaft",
        elem_b="door_skin",
        min_overlap=0.010,
        name="latch shaft remains captured through the panel thickness",
    )

    with ctx.pose({door_hinge: 0.0, latch_turn: 0.0}):
        ctx.expect_within(
            door,
            frame,
            axes="yz",
            inner_elem="door_skin",
            outer_elem="surround",
            margin=0.020,
            name="closed service door sits inside the framed surround",
        )
        ctx.expect_overlap(
            latch,
            frame,
            axes="yz",
            elem_a="cam_bar",
            elem_b="strike_keeper",
            min_overlap=0.005,
            name="locked cam aims at the latch-side keeper",
        )
        closed_aabb = ctx.part_world_aabb(door)
        cam_closed = ctx.part_element_world_aabb(latch, elem="cam_bar")

    with ctx.pose({door_hinge: 1.35, latch_turn: 0.0}):
        open_aabb = ctx.part_world_aabb(door)

    with ctx.pose({door_hinge: 0.0, latch_turn: math.pi / 2.0}):
        cam_unlatched = ctx.part_element_world_aabb(latch, elem="cam_bar")

    ctx.check(
        "door swings outward from hinge side",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][0] > closed_aabb[1][0] + 0.15,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    def _span(aabb, index: int) -> float:
        if aabb is None:
            return 0.0
        return aabb[1][index] - aabb[0][index]

    ctx.check(
        "quarter-turn latch rotates cam clear of keeper",
        _span(cam_closed, 1) > _span(cam_closed, 2) * 3.0
        and _span(cam_unlatched, 2) > _span(cam_unlatched, 1) * 3.0,
        details=f"closed_cam={cam_closed}, unlatched_cam={cam_unlatched}",
    )

    return ctx.report()


object_model = build_object_model()
