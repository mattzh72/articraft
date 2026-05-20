from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

from sdk import (
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


RAIL_LEN = 0.52
RAIL_W = 0.09
BASE_T = 0.012
GUIDE_LEN = 0.42
GUIDE_W_BOTTOM = 0.048
GUIDE_W_TOP = 0.036
GUIDE_H = 0.016

CARR_LEN = 0.14
CARR_W = 0.112
CARR_H = 0.068
CARR_CAVITY_W = 0.058
CARR_CAVITY_H = 0.026
SIDE_POCKET_LEN = 0.088
SIDE_POCKET_DEPTH = 0.012
SIDE_POCKET_H = 0.024

PLINTH_LEN = 0.082
PLINTH_W = 0.074
PLINTH_H = 0.016
RING_R_OUTER = 0.028
RING_R_INNER = 0.018
RING_H = 0.008
SPINDLE_OPEN_R = 0.0195
SPINDLE_OPEN_DEPTH = 0.032

FLANGE_R = 0.026
FLANGE_H = 0.008
SPINDLE_BODY_R = 0.022
SPINDLE_BODY_H = 0.045
SPINDLE_CAP_R = 0.017
SPINDLE_CAP_H = 0.012
CONNECTOR_L = 0.018
CONNECTOR_W = 0.012
CONNECTOR_H = 0.018
NOSE_R = 0.015
NOSE_LEN = 0.015
TIP_R = 0.011
TIP_LEN = 0.008

SLIDE_TRAVEL = 0.16
SPINDLE_SEAT_Z = CARR_H + PLINTH_H + RING_H


def _bottom_box(length: float, width: float, height: float) -> cq.Workplane:
    return cq.Workplane("XY").box(length, width, height).translate((0.0, 0.0, height / 2.0))


def _make_base_plate() -> cq.Workplane:
    return _bottom_box(RAIL_LEN, RAIL_W, BASE_T)


def _make_guide_way() -> cq.Workplane:
    profile = (
        cq.Workplane("YZ")
        .polyline(
            [
                (-GUIDE_W_BOTTOM / 2.0, 0.0),
                (GUIDE_W_BOTTOM / 2.0, 0.0),
                (GUIDE_W_TOP / 2.0, GUIDE_H),
                (-GUIDE_W_TOP / 2.0, GUIDE_H),
            ]
        )
        .close()
    )
    return profile.extrude(GUIDE_LEN).translate((-GUIDE_LEN / 2.0, 0.0, BASE_T))


def _make_carriage_frame() -> cq.Workplane:
    frame = _bottom_box(CARR_LEN, CARR_W, CARR_H)
    frame = frame.cut(_bottom_box(CARR_LEN + 0.012, CARR_CAVITY_W, CARR_CAVITY_H))

    side_pocket_z = CARR_CAVITY_H + SIDE_POCKET_H / 2.0 + 0.004
    left_pocket = cq.Workplane("XY").box(
        SIDE_POCKET_LEN,
        SIDE_POCKET_DEPTH,
        SIDE_POCKET_H,
    ).translate((0.0, CARR_W / 2.0 - SIDE_POCKET_DEPTH / 2.0, side_pocket_z))
    right_pocket = cq.Workplane("XY").box(
        SIDE_POCKET_LEN,
        SIDE_POCKET_DEPTH,
        SIDE_POCKET_H,
    ).translate((0.0, -CARR_W / 2.0 + SIDE_POCKET_DEPTH / 2.0, side_pocket_z))
    frame = frame.cut(left_pocket).cut(right_pocket)

    plinth = _bottom_box(PLINTH_LEN, PLINTH_W, PLINTH_H).translate((0.0, 0.0, CARR_H))
    frame = frame.union(plinth)

    spindle_opening = (
        cq.Workplane("XY")
        .circle(SPINDLE_OPEN_R)
        .extrude(SPINDLE_OPEN_DEPTH)
        .translate((0.0, 0.0, CARR_H + PLINTH_H - SPINDLE_OPEN_DEPTH))
    )
    return frame.cut(spindle_opening)


def _make_bearing_ring() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(RING_R_OUTER)
        .circle(RING_R_INNER)
        .extrude(RING_H)
        .translate((0.0, 0.0, CARR_H + PLINTH_H))
    )


def _make_spindle_body() -> cq.Workplane:
    body = cq.Workplane("XY").circle(FLANGE_R).extrude(FLANGE_H)
    body = body.union(
        cq.Workplane("XY")
        .workplane(offset=FLANGE_H)
        .circle(SPINDLE_BODY_R)
        .extrude(SPINDLE_BODY_H)
    )
    body = body.union(
        cq.Workplane("XY")
        .workplane(offset=FLANGE_H + SPINDLE_BODY_H)
        .circle(SPINDLE_CAP_R)
        .extrude(SPINDLE_CAP_H)
    )
    connector = cq.Workplane("XY").box(
        CONNECTOR_L,
        CONNECTOR_W,
        CONNECTOR_H,
    ).translate(
        (
            SPINDLE_BODY_R + CONNECTOR_L / 2.0 - 0.003,
            0.0,
            FLANGE_H + 0.62 * SPINDLE_BODY_H,
        )
    )
    return body.union(connector)


def _make_spindle_nose() -> cq.Workplane:
    nose = cq.Workplane("XY").circle(NOSE_R).extrude(-NOSE_LEN)
    tip = (
        cq.Workplane("XY")
        .workplane(offset=-NOSE_LEN)
        .circle(TIP_R)
        .extrude(-TIP_LEN)
    )
    return nose.union(tip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="inspection_rail_module")

    base_dark = model.material("base_dark", color=(0.24, 0.26, 0.28))
    guide_steel = model.material("guide_steel", color=(0.56, 0.58, 0.60))
    carriage_alloy = model.material("carriage_alloy", color=(0.73, 0.75, 0.77))
    bearing_dark = model.material("bearing_dark", color=(0.18, 0.18, 0.20))
    spindle_graphite = model.material("spindle_graphite", color=(0.16, 0.17, 0.18))
    spindle_steel = model.material("spindle_steel", color=(0.68, 0.70, 0.72))

    base = model.part("base_guide")
    base.visual(
        mesh_from_cadquery(_make_base_plate(), "base_plate"),
        material=base_dark,
        name="base_plate",
    )
    base.visual(
        mesh_from_cadquery(_make_guide_way(), "guide_way"),
        material=guide_steel,
        name="guide_way",
    )
    base.inertial = Inertial.from_geometry(
        Box((RAIL_LEN, RAIL_W, BASE_T + GUIDE_H)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, (BASE_T + GUIDE_H) / 2.0)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_make_carriage_frame(), "carriage_frame"),
        material=carriage_alloy,
        name="carriage_frame",
    )
    carriage.visual(
        mesh_from_cadquery(_make_bearing_ring(), "bearing_ring"),
        material=bearing_dark,
        name="bearing_ring",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((CARR_LEN, CARR_W, SPINDLE_SEAT_Z)),
        mass=5.0,
        origin=Origin(xyz=(0.0, 0.0, SPINDLE_SEAT_Z / 2.0)),
    )

    spindle = model.part("spindle")
    spindle.visual(
        mesh_from_cadquery(_make_spindle_body(), "spindle_body"),
        material=spindle_graphite,
        name="spindle_body",
    )
    spindle.visual(
        mesh_from_cadquery(_make_spindle_nose(), "spindle_nose"),
        material=spindle_steel,
        name="spindle_nose",
    )
    spindle.inertial = Inertial.from_geometry(
        Cylinder(radius=FLANGE_R, length=FLANGE_H + SPINDLE_BODY_H + SPINDLE_CAP_H),
        mass=2.4,
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                (FLANGE_H + SPINDLE_BODY_H + SPINDLE_CAP_H) / 2.0,
            )
        ),
    )

    model.articulation(
        "rail_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, BASE_T)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.35,
            lower=-SLIDE_TRAVEL,
            upper=SLIDE_TRAVEL,
        ),
    )
    model.articulation(
        "spindle_spin",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=spindle,
        origin=Origin(xyz=(0.0, 0.0, SPINDLE_SEAT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=6.0,
            lower=-pi,
            upper=pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_guide")
    carriage = object_model.get_part("carriage")
    spindle = object_model.get_part("spindle")
    rail_slide = object_model.get_articulation("rail_slide")
    spindle_spin = object_model.get_articulation("spindle_spin")
    base_plate = base.get_visual("base_plate")
    guide_way = base.get_visual("guide_way")
    carriage_frame = carriage.get_visual("carriage_frame")
    bearing_ring = carriage.get_visual("bearing_ring")
    spindle_body = spindle.get_visual("spindle_body")
    spindle_nose = spindle.get_visual("spindle_nose")

    visuals_present = all(
        [
            base_plate is not None,
            guide_way is not None,
            carriage_frame is not None,
            bearing_ring is not None,
            spindle_body is not None,
            spindle_nose is not None,
        ]
    )

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
        "named subsystem visuals exist",
        visuals_present,
        "Expected rail, carriage, and spindle visuals were not all present.",
    )
    ctx.check(
        "carriage slides prismatically along rail axis",
        rail_slide.articulation_type == ArticulationType.PRISMATIC
        and rail_slide.axis == (1.0, 0.0, 0.0)
        and rail_slide.motion_limits is not None
        and rail_slide.motion_limits.lower is not None
        and rail_slide.motion_limits.upper is not None
        and rail_slide.motion_limits.lower < 0.0 < rail_slide.motion_limits.upper,
        "Rail carriage should be a bidirectional x-axis prismatic joint.",
    )
    ctx.check(
        "spindle rotates about local vertical axis",
        spindle_spin.articulation_type == ArticulationType.REVOLUTE
        and spindle_spin.axis == (0.0, 0.0, 1.0)
        and spindle_spin.motion_limits is not None
        and spindle_spin.motion_limits.lower is not None
        and spindle_spin.motion_limits.upper is not None
        and spindle_spin.motion_limits.lower < 0.0 < spindle_spin.motion_limits.upper,
        "Spindle should be a vertical revolute joint with travel in both directions.",
    )

    ctx.expect_origin_gap(
        carriage,
        base,
        axis="z",
        min_gap=BASE_T - 1e-6,
        max_gap=BASE_T + 1e-6,
        name="carriage origin sits on base guide deck",
    )
    ctx.expect_origin_gap(
        spindle,
        carriage,
        axis="z",
        min_gap=SPINDLE_SEAT_Z - 1e-6,
        max_gap=SPINDLE_SEAT_Z + 1e-6,
        name="spindle origin is carried by the raised carriage mount",
    )
    ctx.expect_gap(
        carriage,
        base,
        axis="z",
        positive_elem=carriage_frame,
        negative_elem=base_plate,
        max_gap=0.0,
        max_penetration=0.0,
        name="carriage shoes sit flush on base plate",
    )
    ctx.expect_contact(
        spindle,
        carriage,
        elem_a=spindle_body,
        elem_b=bearing_ring,
        name="spindle flange seats on carriage bearing ring",
    )
    ctx.expect_overlap(
        spindle,
        carriage,
        axes="xy",
        elem_a=spindle_body,
        elem_b=bearing_ring,
        min_overlap=0.05,
        name="spindle stays centered over carriage bearing support",
    )

    slide_limits = rail_slide.motion_limits
    spin_limits = spindle_spin.motion_limits

    with ctx.pose({rail_slide: slide_limits.lower}):
        low_pos = ctx.part_world_position(carriage)
        ctx.expect_gap(
            carriage,
            base,
            axis="z",
            positive_elem=carriage_frame,
            negative_elem=base_plate,
            max_gap=0.0,
            max_penetration=0.0,
            name="carriage remains supported at lower travel",
        )
        ctx.expect_overlap(
            carriage,
            base,
            axes="xy",
            min_overlap=0.08,
            name="carriage remains over the rail at lower travel",
        )

    with ctx.pose({rail_slide: slide_limits.upper}):
        high_pos = ctx.part_world_position(carriage)
        ctx.expect_gap(
            carriage,
            base,
            axis="z",
            positive_elem=carriage_frame,
            negative_elem=base_plate,
            max_gap=0.0,
            max_penetration=0.0,
            name="carriage remains supported at upper travel",
        )
        ctx.expect_overlap(
            carriage,
            base,
            axes="xy",
            min_overlap=0.08,
            name="carriage remains over the rail at upper travel",
        )

    ctx.check(
        "carriage travel span follows prismatic limits",
        low_pos is not None
        and high_pos is not None
        and abs((high_pos[0] - low_pos[0]) - (slide_limits.upper - slide_limits.lower)) < 1e-6
        and abs(high_pos[1] - low_pos[1]) < 1e-6
        and abs(high_pos[2] - low_pos[2]) < 1e-6,
        "Carriage motion should translate only along x by the commanded travel span.",
    )

    with ctx.pose({rail_slide: slide_limits.upper, spindle_spin: spin_limits.upper * 0.5}):
        ctx.expect_contact(
            spindle,
            carriage,
            elem_a=spindle_body,
            elem_b=bearing_ring,
            name="spindle remains seated while rotated",
        )
        ctx.expect_origin_distance(
            spindle,
            carriage,
            axes="xy",
            max_dist=1e-6,
            name="spindle stays centered on carriage in combined pose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
