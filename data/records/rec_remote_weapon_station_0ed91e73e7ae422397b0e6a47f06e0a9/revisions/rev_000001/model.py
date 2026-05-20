from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TrunnionYokeGeometry,
    mesh_from_geometry,
)


OLIVE = Material("mat_olive_drab", rgba=(0.20, 0.27, 0.17, 1.0))
DARK_OLIVE = Material("mat_dark_olive", rgba=(0.11, 0.15, 0.10, 1.0))
GUNMETAL = Material("mat_gunmetal", rgba=(0.06, 0.065, 0.06, 1.0))
BLACK = Material("mat_black", rgba=(0.01, 0.012, 0.012, 1.0))
GLASS = Material("mat_blue_black_glass", rgba=(0.03, 0.08, 0.10, 0.85))
BRASS = Material("mat_brass", rgba=(0.72, 0.55, 0.22, 1.0))
RUBBER = Material("mat_rubber", rgba=(0.02, 0.02, 0.018, 1.0))


def _armor_plate_mesh(name: str):
    """Thin sloped applique armor plate, local X forward, local Z up, thickness on Y."""
    profile_xz = [
        (-0.24, -0.18),
        (0.46, -0.16),
        (0.51, 0.12),
        (0.28, 0.25),
        (-0.20, 0.20),
    ]
    geom = ExtrudeGeometry(profile_xz, 0.026, center=True)
    # ExtrudeGeometry makes thickness along local Z; rotate so the profile's
    # vertical coordinate becomes local Z and thickness becomes local Y.
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="shielded_remote_weapon_station")

    # Fixed pedestal: a heavy base, column, and azimuth bearing.
    pedestal = model.part("pedestal")
    pedestal.visual(
        Box((0.86, 0.78, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=DARK_OLIVE,
        name="floor_plinth",
    )
    pedestal.visual(
        Cylinder(radius=0.18, length=0.48),
        origin=Origin(xyz=(0.0, 0.0, 0.32)),
        material=OLIVE,
        name="pedestal_column",
    )
    pedestal.visual(
        Cylinder(radius=0.32, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.59)),
        material=GUNMETAL,
        name="top_bearing",
    )

    # Rotating azimuth stage: turntable, support yoke, ammo box, and optic yoke.
    azimuth_stage = model.part("azimuth_stage")
    azimuth_stage.visual(
        Cylinder(radius=0.39, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=GUNMETAL,
        name="turntable",
    )
    azimuth_stage.visual(
        Box((0.74, 0.54, 0.08)),
        origin=Origin(xyz=(0.03, 0.0, 0.16)),
        material=OLIVE,
        name="turret_deck",
    )
    main_yoke = TrunnionYokeGeometry(
        (0.72, 0.18, 0.55),
        span_width=0.48,
        trunnion_diameter=0.096,
        trunnion_center_z=0.35,
        base_thickness=0.060,
        corner_radius=0.012,
        center=False,
    )
    azimuth_stage.visual(
        mesh_from_geometry(main_yoke, "elevation_yoke_mesh"),
        origin=Origin(xyz=(0.0, 0.0, 0.20), rpy=(0.0, 0.0, math.pi / 2.0)),
        material=OLIVE,
        name="elevation_yoke",
    )
    azimuth_stage.visual(
        Box((0.28, 0.18, 0.24)),
        origin=Origin(xyz=(-0.18, -0.50, 0.32)),
        material=DARK_OLIVE,
        name="ammo_box",
    )
    azimuth_stage.visual(
        Box((0.23, 0.18, 0.045)),
        origin=Origin(xyz=(-0.18, -0.34, 0.222)),
        material=OLIVE,
        name="ammo_shelf",
    )
    azimuth_stage.visual(
        Box((0.08, 0.24, 0.08)),
        origin=Origin(xyz=(-0.05, -0.35, 0.46)),
        material=BRASS,
        name="feed_chute",
    )
    azimuth_stage.visual(
        Box((0.18, 0.25, 0.045)),
        origin=Origin(xyz=(0.16, 0.42, 0.48)),
        material=OLIVE,
        name="optic_arm",
    )
    optic_yoke_geom = TrunnionYokeGeometry(
        (0.20, 0.13, 0.20),
        span_width=0.106,
        trunnion_diameter=0.046,
        trunnion_center_z=0.10,
        base_thickness=0.035,
        corner_radius=0.006,
        center=False,
    )
    azimuth_stage.visual(
        mesh_from_geometry(optic_yoke_geom, "optic_yoke_mesh"),
        origin=Origin(xyz=(0.16, 0.62, 0.45), rpy=(0.0, 0.0, math.pi / 2.0)),
        material=OLIVE,
        name="optic_yoke",
    )

    # Elevating weapon cradle.  Its part frame is exactly on the trunnion axis.
    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.048, length=0.70),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=GUNMETAL,
        name="elevation_trunnion",
    )
    cradle.visual(
        Box((0.42, 0.16, 0.16)),
        origin=Origin(xyz=(0.19, 0.0, 0.0)),
        material=GUNMETAL,
        name="receiver",
    )
    cradle.visual(
        Cylinder(radius=0.040, length=0.82),
        origin=Origin(xyz=(0.75, 0.0, 0.015), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=GUNMETAL,
        name="barrel",
    )
    cradle.visual(
        Cylinder(radius=0.055, length=0.11),
        origin=Origin(xyz=(1.20, 0.0, 0.015), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=BLACK,
        name="muzzle_brake",
    )
    cradle.visual(
        Box((0.18, 0.018, 0.070)),
        origin=Origin(xyz=(1.20, 0.0, 0.015)),
        material=BLACK,
        name="muzzle_slot",
    )
    cradle.visual(
        _armor_plate_mesh("armor_plate_mesh_0"),
        origin=Origin(xyz=(0.14, 0.185, 0.01)),
        material=OLIVE,
        name="armor_plate_0",
    )
    cradle.visual(
        _armor_plate_mesh("armor_plate_mesh_1"),
        origin=Origin(xyz=(0.14, -0.185, 0.01)),
        material=OLIVE,
        name="armor_plate_1",
    )
    cradle.visual(
        Box((0.40, 0.40, 0.035)),
        origin=Origin(xyz=(0.17, 0.0, 0.17)),
        material=OLIVE,
        name="top_bridge",
    )
    cradle.visual(
        Box((0.34, 0.40, 0.030)),
        origin=Origin(xyz=(0.13, 0.0, -0.135)),
        material=OLIVE,
        name="lower_bridge",
    )
    cradle.visual(
        Box((0.30, 0.06, 0.030)),
        origin=Origin(xyz=(0.28, -0.205, 0.04)),
        material=BRASS,
        name="belt_receiver",
    )

    # Compact optic head captured by a side yoke.  Its local frame is the tilt axis.
    optic_head = model.part("optic_head")
    optic_head.visual(
        Cylinder(radius=0.026, length=0.18),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=GUNMETAL,
        name="optic_trunnion",
    )
    optic_head.visual(
        Box((0.145, 0.074, 0.092)),
        origin=Origin(xyz=(0.025, 0.0, 0.0)),
        material=BLACK,
        name="sensor_body",
    )
    optic_head.visual(
        Cylinder(radius=0.030, length=0.026),
        origin=Origin(xyz=(0.110, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=GLASS,
        name="front_lens",
    )
    optic_head.visual(
        Cylinder(radius=0.020, length=0.026),
        origin=Origin(xyz=(-0.060, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=RUBBER,
        name="rear_cap",
    )
    optic_head.visual(
        Box((0.050, 0.035, 0.016)),
        origin=Origin(xyz=(0.005, 0.0, 0.054)),
        material=DARK_OLIVE,
        name="top_clip",
    )

    model.articulation(
        "azimuth",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=azimuth_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.62)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=200.0, velocity=1.2),
    )
    model.articulation(
        "elevation",
        ArticulationType.REVOLUTE,
        parent=azimuth_stage,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.55)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=100.0, velocity=0.8, lower=-0.25, upper=0.55),
    )
    model.articulation(
        "optic_tilt",
        ArticulationType.REVOLUTE,
        parent=azimuth_stage,
        child=optic_head,
        origin=Origin(xyz=(0.16, 0.62, 0.55)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.6, lower=-0.50, upper=0.70),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    azimuth_stage = object_model.get_part("azimuth_stage")
    cradle = object_model.get_part("cradle")
    optic_head = object_model.get_part("optic_head")
    azimuth = object_model.get_articulation("azimuth")
    elevation = object_model.get_articulation("elevation")
    optic_tilt = object_model.get_articulation("optic_tilt")

    ctx.allow_overlap(
        azimuth_stage,
        cradle,
        elem_a="elevation_yoke",
        elem_b="elevation_trunnion",
        reason="The elevation shaft is intentionally modeled as a captured trunnion running through the yoke bore proxy.",
    )
    ctx.allow_overlap(
        azimuth_stage,
        optic_head,
        elem_a="optic_yoke",
        elem_b="optic_trunnion",
        reason="The compact sensor head is intentionally clipped into the side yoke by its tilt trunnion.",
    )

    ctx.check(
        "azimuth joint is vertical and continuous",
        azimuth.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 3) for v in azimuth.axis) == (0.0, 0.0, 1.0),
        details=f"type={azimuth.articulation_type}, axis={azimuth.axis}",
    )
    ctx.check(
        "elevation and optic axes are horizontal",
        elevation.articulation_type == ArticulationType.REVOLUTE
        and optic_tilt.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 3) for v in elevation.axis) == (0.0, -1.0, 0.0)
        and tuple(round(v, 3) for v in optic_tilt.axis) == (0.0, -1.0, 0.0),
        details=f"elevation={elevation.axis}, optic={optic_tilt.axis}",
    )

    ctx.expect_gap(
        azimuth_stage,
        pedestal,
        axis="z",
        positive_elem="turntable",
        negative_elem="top_bearing",
        max_gap=0.001,
        max_penetration=0.001,
        name="turntable sits on pedestal bearing",
    )
    ctx.expect_within(
        cradle,
        azimuth_stage,
        axes="xz",
        inner_elem="elevation_trunnion",
        outer_elem="elevation_yoke",
        margin=0.010,
        name="weapon trunnion is captured by elevation yoke",
    )
    ctx.expect_overlap(
        cradle,
        azimuth_stage,
        axes="y",
        elem_a="elevation_trunnion",
        elem_b="elevation_yoke",
        min_overlap=0.55,
        name="weapon trunnion spans the support cheeks",
    )
    ctx.expect_within(
        optic_head,
        azimuth_stage,
        axes="xz",
        inner_elem="optic_trunnion",
        outer_elem="optic_yoke",
        margin=0.006,
        name="optic trunnion sits in side yoke",
    )
    ctx.expect_overlap(
        optic_head,
        azimuth_stage,
        axes="y",
        elem_a="optic_trunnion",
        elem_b="optic_yoke",
        min_overlap=0.15,
        name="optic trunnion remains clipped through yoke cheeks",
    )

    rest_barrel_aabb = ctx.part_element_world_aabb(cradle, elem="barrel")
    rest_lens_aabb = ctx.part_element_world_aabb(optic_head, elem="front_lens")
    with ctx.pose({elevation: 0.45, optic_tilt: 0.55}):
        raised_barrel_aabb = ctx.part_element_world_aabb(cradle, elem="barrel")
        tilted_lens_aabb = ctx.part_element_world_aabb(optic_head, elem="front_lens")
        ctx.expect_within(
            optic_head,
            azimuth_stage,
            axes="xz",
            inner_elem="optic_trunnion",
            outer_elem="optic_yoke",
            margin=0.006,
            name="optic stays clipped while tilted",
        )
        ctx.expect_overlap(
            optic_head,
            azimuth_stage,
            axes="y",
            elem_a="optic_trunnion",
            elem_b="optic_yoke",
            min_overlap=0.15,
            name="optic trunnion still spans yoke at tilt",
        )

    ctx.check(
        "positive elevation raises barrel",
        rest_barrel_aabb is not None
        and raised_barrel_aabb is not None
        and raised_barrel_aabb[1][2] > rest_barrel_aabb[1][2] + 0.08,
        details=f"rest={rest_barrel_aabb}, raised={raised_barrel_aabb}",
    )
    ctx.check(
        "positive optic tilt raises lens",
        rest_lens_aabb is not None
        and tilted_lens_aabb is not None
        and tilted_lens_aabb[1][2] > rest_lens_aabb[1][2] + 0.025,
        details=f"rest={rest_lens_aabb}, tilted={tilted_lens_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
