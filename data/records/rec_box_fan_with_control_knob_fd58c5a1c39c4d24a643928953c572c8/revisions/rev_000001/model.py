from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_W = 1.05
BODY_H = 0.42
BODY_D = 0.16
RAIL = 0.055
CENTER_DIVIDER = 0.045
GUARD_T = 0.006
ROTOR_X = 0.265
ROTOR_Y = -0.035
ROTOR_R = 0.145


def _twin_guard_plate() -> cq.Workplane:
    """Thin connected face plate with two circular grille guards."""

    opening_radius = 0.185
    ring_bar = 0.008
    spoke_w = 0.007
    body = cq.Workplane("XY").rect(BODY_W, BODY_H).extrude(GUARD_T)

    # Two large round fan openings in the rectangular window-fan face.
    for cx in (-ROTOR_X, ROTOR_X):
        cutter = (
            cq.Workplane("XY")
            .center(cx, 0.0)
            .circle(opening_radius)
            .extrude(GUARD_T * 3.0)
            .translate((0.0, 0.0, -GUARD_T))
        )
        body = body.cut(cutter)

    # Connected concentric rings and radial spokes across each opening.
    for cx in (-ROTOR_X, ROTOR_X):
        for radius in (0.185, 0.130, 0.080):
            ring = (
                cq.Workplane("XY")
                .center(cx, 0.0)
                .circle(radius + ring_bar / 2.0)
                .circle(radius - ring_bar / 2.0)
                .extrude(GUARD_T)
            )
            body = body.union(ring)

        center_badge = (
            cq.Workplane("XY").center(cx, 0.0).circle(0.034).extrude(GUARD_T)
        )
        body = body.union(center_badge)

        for angle_deg in (0.0, 30.0, 60.0, 90.0, 120.0, 150.0):
            spoke = (
                cq.Workplane("XY")
                .center(cx, 0.0)
                .rect(2.0 * opening_radius, spoke_w)
                .extrude(GUARD_T)
                .rotate((cx, 0.0, 0.0), (cx, 0.0, 1.0), angle_deg)
            )
            body = body.union(spoke)

    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="window_box_fan")

    body_mat = Material("warm_white_plastic", rgba=(0.82, 0.80, 0.72, 1.0))
    grille_mat = Material("pale_grille_plastic", rgba=(0.72, 0.72, 0.66, 1.0))
    dark_mat = Material("dark_motor_plastic", rgba=(0.08, 0.085, 0.09, 1.0))
    blade_mat = Material("translucent_smoke_blades", rgba=(0.38, 0.46, 0.50, 0.82))
    knob_mat = Material("black_control_knob", rgba=(0.02, 0.02, 0.018, 1.0))
    label_mat = Material("printed_black_ticks", rgba=(0.01, 0.01, 0.01, 1.0))

    housing = model.part("housing")

    # Deep box-fan casing: four perimeter rails and a center mullion.
    housing.visual(
        Box((BODY_W, BODY_D, RAIL)),
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0 - RAIL / 2.0)),
        material=body_mat,
        name="top_rail",
    )
    housing.visual(
        Box((BODY_W, BODY_D, RAIL)),
        origin=Origin(xyz=(0.0, 0.0, -BODY_H / 2.0 + RAIL / 2.0)),
        material=body_mat,
        name="bottom_rail",
    )
    for name, sx in (("side_rail_0", -BODY_W / 2.0 + RAIL / 2.0), ("side_rail_1", BODY_W / 2.0 - RAIL / 2.0)):
        housing.visual(
            Box((RAIL, BODY_D, BODY_H)),
            origin=Origin(xyz=(sx, 0.0, 0.0)),
            material=body_mat,
            name=name,
        )
    housing.visual(
        Box((CENTER_DIVIDER, BODY_D, BODY_H)),
        origin=Origin(),
        material=body_mat,
        name="center_mullion",
    )

    guard_mesh = mesh_from_cadquery(
        _twin_guard_plate(),
        "twin_circular_guard",
        tolerance=0.0012,
        angular_tolerance=0.12,
    )
    housing.visual(
        guard_mesh,
        origin=Origin(xyz=(0.0, -BODY_D / 2.0 - 0.002, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=grille_mat,
        name="front_guard",
    )
    housing.visual(
        guard_mesh,
        origin=Origin(xyz=(0.0, BODY_D / 2.0 - GUARD_T, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=grille_mat,
        name="rear_guard",
    )

    # Rear motor pods on separate axes, visibly supported by the rear grille.
    for idx, cx in enumerate((-ROTOR_X, ROTOR_X)):
        housing.visual(
            Cylinder(radius=0.044, length=0.060),
            origin=Origin(xyz=(cx, 0.045, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_mat,
            name=f"motor_boss_{idx}",
        )

    # Rotary control collar and printed speed ticks on the top edge.
    knob_x = BODY_W / 2.0 - 0.145
    knob_y = -0.018
    collar_z = BODY_H / 2.0 + 0.003
    housing.visual(
        Cylinder(radius=0.045, length=0.006),
        origin=Origin(xyz=(knob_x, knob_y, collar_z)),
        material=dark_mat,
        name="knob_collar",
    )
    for idx, tx in enumerate((-0.038, -0.018, 0.018, 0.038)):
        housing.visual(
            Box((0.012, 0.003, 0.002)),
            origin=Origin(xyz=(knob_x + tx, knob_y - 0.052, BODY_H / 2.0 + 0.001)),
            material=label_mat,
            name=f"speed_tick_{idx}",
        )

    rotor_mesh = mesh_from_geometry(
        FanRotorGeometry(
            ROTOR_R,
            0.035,
            5,
            thickness=0.034,
            blade_pitch_deg=32.0,
            blade_sweep_deg=24.0,
            blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=16.0, camber=0.12, tip_clearance=0.004),
            hub=FanRotorHub(style="spinner"),
        ),
        "five_blade_rotor",
    )

    for idx, cx in enumerate((-ROTOR_X, ROTOR_X)):
        rotor = model.part(f"rotor_{idx}")
        rotor.visual(rotor_mesh, material=blade_mat, name="rotor_blades")
        rotor.visual(
            Cylinder(radius=0.006, length=0.050),
            origin=Origin(xyz=(0.0, 0.0, 0.025)),
            material=dark_mat,
            name="axle_shaft",
        )
        model.articulation(
            f"rotor_axle_{idx}",
            ArticulationType.CONTINUOUS,
            parent=housing,
            child=rotor,
            origin=Origin(xyz=(cx, ROTOR_Y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.25, velocity=80.0),
        )

    knob = model.part("knob")
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.058,
                0.030,
                body_style="skirted",
                top_diameter=0.044,
                edge_radius=0.0015,
                grip=KnobGrip(style="fluted", count=18, depth=0.0015),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "top_speed_knob",
        ),
        material=knob_mat,
        name="knob_cap",
    )
    model.articulation(
        "knob_axis",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=knob,
        origin=Origin(xyz=(knob_x, knob_y, BODY_H / 2.0 + 0.006)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=3.0, lower=0.0, upper=math.radians(300.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    knob = object_model.get_part("knob")
    rotor_0 = object_model.get_part("rotor_0")
    rotor_1 = object_model.get_part("rotor_1")
    axle_0 = object_model.get_articulation("rotor_axle_0")
    axle_1 = object_model.get_articulation("rotor_axle_1")
    knob_axis = object_model.get_articulation("knob_axis")

    ctx.check(
        "twin rotors have independent spin axles",
        axle_0.articulation_type == ArticulationType.CONTINUOUS
        and axle_1.articulation_type == ArticulationType.CONTINUOUS
        and axle_0.child == "rotor_0"
        and axle_1.child == "rotor_1"
        and abs(axle_0.origin.xyz[0] - axle_1.origin.xyz[0]) > 0.45,
    )
    ctx.check(
        "top knob is a limited rotary control",
        knob_axis.articulation_type == ArticulationType.REVOLUTE
        and knob_axis.motion_limits is not None
        and knob_axis.motion_limits.lower == 0.0
        and knob_axis.motion_limits.upper is not None
        and knob_axis.motion_limits.upper > math.radians(250.0),
    )

    for idx, rotor in enumerate((rotor_0, rotor_1)):
        ctx.expect_gap(
            rotor,
            housing,
            axis="y",
            positive_elem="rotor_blades",
            negative_elem="front_guard",
            min_gap=0.015,
            name=f"rotor_{idx} clears front safety grille",
        )
        ctx.expect_gap(
            housing,
            rotor,
            axis="y",
            positive_elem=f"motor_boss_{idx}",
            negative_elem="axle_shaft",
            max_gap=0.003,
            max_penetration=0.0,
            name=f"rotor_{idx} axle reaches its motor boss",
        )

    ctx.expect_gap(
        knob,
        housing,
        axis="z",
        positive_elem="knob_cap",
        negative_elem="knob_collar",
        max_gap=0.002,
        max_penetration=0.0,
        name="control knob seats on top collar",
    )

    return ctx.report()


object_model = build_object_model()
