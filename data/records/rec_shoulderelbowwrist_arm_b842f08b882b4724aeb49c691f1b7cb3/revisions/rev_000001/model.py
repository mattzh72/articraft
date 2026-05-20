from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


SHOULDER_Z = 0.36
UPPER_ELBOW_X = 0.72
UPPER_ELBOW_Z = 0.24
FOREARM_WRIST_X = 0.58
ELBOW_LIMITS = (-0.50, 0.75)
WRIST_LIMITS = (-pi, pi)


def _filleted_box(size: tuple[float, float, float], center: tuple[float, float, float], radius: float) -> cq.Workplane:
    """Rounded rectangular machined block, in meters."""
    solid = cq.Workplane("XY").box(size[0], size[1], size[2])
    if radius > 0.0:
        # Keep the design robust: if a particular tiny edge refuses to fillet,
        # fall back to the exact block instead of failing the whole artifact.
        try:
            solid = solid.edges().fillet(radius)
        except Exception:
            pass
    return solid.translate(center)


def _union(solids: list[cq.Workplane]) -> cq.Workplane:
    result = solids[0]
    for solid in solids[1:]:
        result = result.union(solid).clean()
    return result.clean()


def _upper_link_shape() -> cq.Workplane:
    """Broad hollow upper-arm frame with real webs and wall thickness."""
    # Start from one solid boxed arm and cut a through-window.  This preserves a
    # continuous manufactured wall instead of assembling visibly floating rails.
    tube = _filleted_box((0.56, 0.27, 0.205), (0.34, 0.0, 0.240), 0.012)
    window = cq.Workplane("XY").box(0.330, 0.36, 0.105).translate((0.390, 0.0, 0.240))
    tube = tube.cut(window).clean()

    solids: list[cq.Workplane] = [
        tube,
        # Outboard clevis cheeks carry the elbow covers while leaving the
        # center barrel clearance through the middle of the hinge.
        _filleted_box((0.120, 0.040, 0.185), (0.665, 0.126, 0.240), 0.010),
        _filleted_box((0.120, 0.040, 0.185), (0.665, -0.126, 0.240), 0.010),
        # Low machined rib under the top cable trough.
        _filleted_box((0.40, 0.070, 0.018), (0.385, 0.0, 0.348), 0.006),
        # Shoulder block blends the rotary column into the link root.
        _filleted_box((0.24, 0.30, 0.19), (0.090, 0.0, 0.160), 0.020),
    ]
    return _union(solids)


def _forearm_shape() -> cq.Workplane:
    """Slimmer forearm tube with visible wall thickness and end webs."""
    solids: list[cq.Workplane] = [
        _filleted_box((0.470, 0.145, 0.022), (0.315, 0.0, 0.061), 0.007),
        _filleted_box((0.470, 0.145, 0.022), (0.315, 0.0, -0.061), 0.007),
        _filleted_box((0.470, 0.024, 0.125), (0.315, 0.067, 0.0), 0.007),
        _filleted_box((0.470, 0.024, 0.125), (0.315, -0.067, 0.0), 0.007),
        _filleted_box((0.052, 0.145, 0.115), (0.091, 0.0, 0.0), 0.010),
        _filleted_box((0.040, 0.145, 0.115), (0.535, 0.0, 0.0), 0.010),
        # Raised structural land under the cable channel.
        _filleted_box((0.355, 0.055, 0.016), (0.330, 0.0, 0.080), 0.005),
    ]
    return _union(solids)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="machine_tending_arm")

    model.material("cast_iron", rgba=(0.16, 0.17, 0.18, 1.0))
    model.material("gunmetal", rgba=(0.27, 0.29, 0.31, 1.0))
    model.material("machined_aluminum", rgba=(0.62, 0.65, 0.66, 1.0))
    model.material("dark_anodized", rgba=(0.08, 0.09, 0.10, 1.0))
    model.material("bearing_steel", rgba=(0.73, 0.74, 0.72, 1.0))
    model.material("safety_yellow", rgba=(0.96, 0.70, 0.08, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.36, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material="cast_iron",
        name="floor_plinth",
    )
    base.visual(
        Cylinder(radius=0.23, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 0.205)),
        material="gunmetal",
        name="pedestal",
    )
    base.visual(
        Cylinder(radius=0.285, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.335)),
        material="bearing_steel",
        name="top_bearing_cover",
    )
    base.visual(
        Cylinder(radius=0.245, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.355)),
        material="dark_anodized",
        name="turntable_shadow_line",
    )
    for index, (x_pos, y_pos) in enumerate(
        ((0.24, 0.0), (-0.24, 0.0), (0.0, 0.24), (0.0, -0.24))
    ):
        base.visual(
            Cylinder(radius=0.022, length=0.018),
            origin=Origin(xyz=(x_pos, y_pos, 0.110)),
            material="bearing_steel",
            name=f"plinth_bolt_{index}",
        )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.250, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material="gunmetal",
        name="rotary_skirt",
    )
    upper_arm.visual(
        Cylinder(radius=0.185, length=0.230),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material="machined_aluminum",
        name="shoulder_column",
    )
    upper_arm.visual(
        mesh_from_cadquery(_upper_link_shape(), "upper_link_frame"),
        material="machined_aluminum",
        name="upper_link_frame",
    )
    upper_arm.visual(
        Box((0.375, 0.052, 0.010)),
        origin=Origin(xyz=(0.405, 0.0, 0.362)),
        material="dark_anodized",
        name="upper_cable_channel",
    )
    upper_arm.visual(
        Cylinder(radius=0.092, length=0.048),
        origin=Origin(xyz=(UPPER_ELBOW_X, 0.116, UPPER_ELBOW_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material="bearing_steel",
        name="elbow_cover_pos",
    )
    upper_arm.visual(
        Cylinder(radius=0.092, length=0.048),
        origin=Origin(xyz=(UPPER_ELBOW_X, -0.116, UPPER_ELBOW_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material="bearing_steel",
        name="elbow_cover_neg",
    )
    upper_arm.visual(
        Box((0.055, 0.015, 0.210)),
        origin=Origin(xyz=(0.655, 0.151, UPPER_ELBOW_Z)),
        material="safety_yellow",
        name="elbow_stop_pos",
    )
    upper_arm.visual(
        Box((0.055, 0.015, 0.210)),
        origin=Origin(xyz=(0.655, -0.151, UPPER_ELBOW_Z)),
        material="safety_yellow",
        name="elbow_stop_neg",
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.074, length=0.184),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="bearing_steel",
        name="elbow_barrel",
    )
    forearm.visual(
        mesh_from_cadquery(_forearm_shape(), "forearm_shell"),
        material="machined_aluminum",
        name="forearm_shell",
    )
    forearm.visual(
        Box((0.330, 0.040, 0.010)),
        origin=Origin(xyz=(0.335, 0.0, 0.093)),
        material="dark_anodized",
        name="forearm_cable_channel",
    )
    forearm.visual(
        Cylinder(radius=0.092, length=0.085),
        origin=Origin(xyz=(0.5375, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="bearing_steel",
        name="wrist_bearing",
    )

    wrist = model.part("wrist")
    wrist.visual(
        Cylinder(radius=0.070, length=0.155),
        origin=Origin(xyz=(0.0775, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="dark_anodized",
        name="wrist_nose",
    )
    wrist.visual(
        Cylinder(radius=0.092, length=0.020),
        origin=Origin(xyz=(0.150, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="bearing_steel",
        name="plate_hub",
    )
    wrist.visual(
        Box((0.040, 0.220, 0.220)),
        origin=Origin(xyz=(0.1775, 0.0, 0.0)),
        material="machined_aluminum",
        name="square_plate",
    )
    for index, (y_pos, z_pos) in enumerate(
        ((0.075, 0.075), (-0.075, 0.075), (0.075, -0.075), (-0.075, -0.075))
    ):
        wrist.visual(
            Cylinder(radius=0.012, length=0.010),
            origin=Origin(xyz=(0.2025, y_pos, z_pos), rpy=(0.0, pi / 2.0, 0.0)),
            material="bearing_steel",
            name=f"plate_bolt_{index}",
        )

    model.articulation(
        "shoulder_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-2.70, upper=2.70, effort=260.0, velocity=1.2),
    )
    model.articulation(
        "elbow_hinge",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(UPPER_ELBOW_X, 0.0, UPPER_ELBOW_Z)),
        # With the forearm extending along local +X, positive motion raises it.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=ELBOW_LIMITS[0], upper=ELBOW_LIMITS[1], effort=120.0, velocity=1.4),
    )
    model.articulation(
        "wrist_roll",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist,
        origin=Origin(xyz=(FOREARM_WRIST_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=WRIST_LIMITS[0], upper=WRIST_LIMITS[1], effort=35.0, velocity=3.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist = object_model.get_part("wrist")
    elbow = object_model.get_articulation("elbow_hinge")
    wrist_roll = object_model.get_articulation("wrist_roll")

    ctx.expect_contact(
        upper_arm,
        base,
        elem_a="rotary_skirt",
        elem_b="top_bearing_cover",
        name="shoulder skirt is seated on base bearing",
    )
    ctx.expect_contact(
        upper_arm,
        forearm,
        elem_a="elbow_cover_pos",
        elem_b="elbow_barrel",
        name="positive elbow cover bears on center barrel",
    )
    ctx.expect_contact(
        forearm,
        upper_arm,
        elem_a="elbow_barrel",
        elem_b="elbow_cover_neg",
        name="negative elbow cover bears on center barrel",
    )
    ctx.expect_contact(
        wrist,
        forearm,
        elem_a="wrist_nose",
        elem_b="wrist_bearing",
        name="wrist nose seats against forearm bearing",
    )

    for q, label in ((ELBOW_LIMITS[0], "lower"), (ELBOW_LIMITS[1], "upper")):
        with ctx.pose({elbow: q}):
            ctx.expect_gap(
                forearm,
                upper_arm,
                axis="x",
                positive_elem="forearm_shell",
                negative_elem="upper_link_frame",
                min_gap=0.005,
                name=f"forearm shell stays forward of upper link at {label} elbow limit",
            )
            wrist_pos = ctx.part_world_position(wrist)
            clear_radially = wrist_pos is not None and (wrist_pos[0] ** 2 + wrist_pos[1] ** 2) ** 0.5 > 0.48
            clear_above = wrist_pos is not None and wrist_pos[2] > SHOULDER_Z + 0.04
            ctx.check(
                f"wrist package clears pedestal at {label} elbow limit",
                clear_radially or clear_above,
                details=f"wrist_pos={wrist_pos}",
            )

    rest_wrist_pos = ctx.part_world_position(wrist)
    with ctx.pose({wrist_roll: pi / 2.0}):
        rolled_wrist_pos = ctx.part_world_position(wrist)
        ctx.expect_contact(
            wrist,
            forearm,
            elem_a="wrist_nose",
            elem_b="wrist_bearing",
            name="rolled wrist nose stays captured on bearing face",
        )
    ctx.check(
        "wrist roll pivots without translating the nose",
        rest_wrist_pos is not None
        and rolled_wrist_pos is not None
        and max(abs(a - b) for a, b in zip(rest_wrist_pos, rolled_wrist_pos)) < 1e-6,
        details=f"rest={rest_wrist_pos}, rolled={rolled_wrist_pos}",
    )

    return ctx.report()


object_model = build_object_model()
