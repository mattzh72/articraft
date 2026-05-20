from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BlowerWheelGeometry,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)
import cadquery as cq


def _polygon_slab(
    points: list[tuple[float, float]],
    z_min: float,
    thickness: float,
) -> cq.Workplane:
    return cq.Workplane("XY").polyline(points).close().extrude(thickness).translate((0, 0, z_min))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="residential_tower_fan")

    matte_black = model.material("matte_black", rgba=(0.02, 0.022, 0.024, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.10, 0.105, 0.11, 1.0))
    warm_gray = model.material("warm_gray", rgba=(0.72, 0.70, 0.66, 1.0))
    grille_gray = model.material("grille_gray", rgba=(0.18, 0.19, 0.20, 1.0))
    satin_silver = model.material("satin_silver", rgba=(0.58, 0.60, 0.62, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.215, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=dark_gray,
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=0.095, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.057)),
        material=matte_black,
        name="turntable",
    )
    base.visual(
        Cylinder(radius=0.052, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.072)),
        material=satin_silver,
        name="oscillation_boss",
    )

    housing = model.part("housing")
    cap_profile = [
        (-0.110, -0.080),
        (-0.090, 0.060),
        (-0.045, 0.085),
        (0.045, 0.085),
        (0.090, 0.060),
        (0.110, -0.080),
    ]
    housing.visual(
        mesh_from_cadquery(_polygon_slab(cap_profile, 0.000, 0.065), "bottom_housing_cap"),
        origin=Origin(),
        material=warm_gray,
        name="bottom_cap",
    )
    housing.visual(
        mesh_from_cadquery(_polygon_slab(cap_profile, 0.950, 0.060), "top_housing_cap"),
        origin=Origin(),
        material=warm_gray,
        name="top_cap",
    )

    def add_panel(
        name: str,
        p0: tuple[float, float],
        p1: tuple[float, float],
        thickness: float,
        z_min: float,
        z_max: float,
        material,
    ) -> None:
        x0, y0 = p0
        x1, y1 = p1
        length = math.hypot(x1 - x0, y1 - y0)
        yaw = math.atan2(y1 - y0, x1 - x0)
        housing.visual(
            Box((length, thickness, z_max - z_min)),
            origin=Origin(
                xyz=((x0 + x1) * 0.5, (y0 + y1) * 0.5, (z_min + z_max) * 0.5),
                rpy=(0.0, 0.0, yaw),
            ),
            material=material,
            name=name,
        )

    for index, (p0, p1) in enumerate(zip(cap_profile[:-1], cap_profile[1:])):
        add_panel(f"side_facet_{index}", p0, p1, 0.010, 0.045, 0.955, warm_gray)

    housing.visual(
        Box((0.018, 0.014, 0.880)),
        origin=Origin(xyz=(-0.101, -0.083, 0.500)),
        material=warm_gray,
        name="front_post_0",
    )
    housing.visual(
        Box((0.018, 0.014, 0.880)),
        origin=Origin(xyz=(0.101, -0.083, 0.500)),
        material=warm_gray,
        name="front_post_1",
    )
    housing.visual(
        Box((0.196, 0.008, 0.014)),
        origin=Origin(xyz=(0.0, -0.093, 0.145)),
        material=grille_gray,
        name="lower_grille_rail",
    )
    housing.visual(
        Box((0.196, 0.008, 0.014)),
        origin=Origin(xyz=(0.0, -0.093, 0.885)),
        material=grille_gray,
        name="upper_grille_rail",
    )
    housing.visual(
        Box((0.008, 0.008, 0.760)),
        origin=Origin(xyz=(0.0, -0.093, 0.515)),
        material=grille_gray,
        name="center_mullion",
    )
    for i in range(19):
        housing.visual(
            Box((0.196, 0.006, 0.006)),
            origin=Origin(xyz=(0.0, -0.093, 0.180 + i * 0.036)),
            material=grille_gray,
            name=f"grille_bar_{i}",
        )
    housing.visual(
        Cylinder(radius=0.029, length=0.008),
        origin=Origin(xyz=(-0.045, -0.004, 1.012)),
        material=warm_gray,
        name="speed_boss",
    )
    housing.visual(
        Cylinder(radius=0.029, length=0.008),
        origin=Origin(xyz=(0.045, -0.004, 1.012)),
        material=warm_gray,
        name="timer_boss",
    )

    fan_drum = model.part("fan_drum")
    fan_drum.visual(
        mesh_from_geometry(
            BlowerWheelGeometry(
                0.055,
                0.027,
                0.735,
                32,
                blade_thickness=0.0024,
                blade_sweep_deg=30.0,
                backplate=True,
                shroud=True,
            ),
            "crossflow_fan_drum",
        ),
        origin=Origin(),
        material=grille_gray,
        name="drum_cage",
    )
    fan_drum.visual(
        Cylinder(radius=0.006, length=0.880),
        origin=Origin(),
        material=satin_silver,
        name="axle",
    )
    fan_drum.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.440)),
        material=satin_silver,
        name="top_journal",
    )
    fan_drum.visual(
        Cylinder(radius=0.031, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.360)),
        material=satin_silver,
        name="lower_hub",
    )
    fan_drum.visual(
        Cylinder(radius=0.031, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.360)),
        material=satin_silver,
        name="upper_hub",
    )

    knob_geom = KnobGeometry(
        0.052,
        0.034,
        body_style="skirted",
        top_diameter=0.044,
        skirt=KnobSkirt(0.060, 0.006, flare=0.05, chamfer=0.001),
        grip=KnobGrip(style="fluted", count=18, depth=0.0012),
        indicator=KnobIndicator(style="line", mode="raised", depth=0.0008, angle_deg=0.0),
        center=False,
    )

    speed_knob = model.part("speed_knob")
    speed_knob.visual(
        mesh_from_geometry(knob_geom, "speed_knob"),
        origin=Origin(),
        material=matte_black,
        name="knob_cap",
    )

    timer_knob = model.part("timer_knob")
    timer_knob.visual(
        mesh_from_geometry(knob_geom, "timer_knob"),
        origin=Origin(),
        material=matte_black,
        name="knob_cap",
    )

    model.articulation(
        "base_to_housing",
        ArticulationType.REVOLUTE,
        parent=base,
        child=housing,
        origin=Origin(xyz=(0.0, 0.0, 0.087)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.8, lower=-0.75, upper=0.75),
    )
    model.articulation(
        "housing_to_drum",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=fan_drum,
        origin=Origin(xyz=(0.0, 0.0, 0.505)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=45.0),
    )
    model.articulation(
        "housing_to_speed_knob",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=speed_knob,
        origin=Origin(xyz=(-0.045, -0.004, 1.016)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.3, velocity=8.0),
    )
    model.articulation(
        "housing_to_timer_knob",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=timer_knob,
        origin=Origin(xyz=(0.045, -0.004, 1.016)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.3, velocity=8.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    housing = object_model.get_part("housing")
    fan_drum = object_model.get_part("fan_drum")
    speed_knob = object_model.get_part("speed_knob")
    timer_knob = object_model.get_part("timer_knob")
    oscillation = object_model.get_articulation("base_to_housing")
    drum_spin = object_model.get_articulation("housing_to_drum")
    speed_spin = object_model.get_articulation("housing_to_speed_knob")
    timer_spin = object_model.get_articulation("housing_to_timer_knob")

    ctx.check(
        "tower fan has four user mechanisms",
        len(object_model.articulations) == 4,
        details=f"articulations={len(object_model.articulations)}",
    )
    ctx.check(
        "drum and knobs are continuous rotary controls",
        all(j.articulation_type == ArticulationType.CONTINUOUS for j in (drum_spin, speed_spin, timer_spin)),
    )
    ctx.check(
        "oscillation is a limited vertical revolute joint",
        oscillation.articulation_type == ArticulationType.REVOLUTE
        and oscillation.axis == (0.0, 0.0, 1.0)
        and oscillation.motion_limits is not None
        and oscillation.motion_limits.lower < 0.0
        and oscillation.motion_limits.upper > 0.0,
    )

    ctx.expect_gap(
        housing,
        base,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        name="housing sits on the turntable without sinking into it",
    )
    ctx.expect_overlap(
        housing,
        base,
        axes="xy",
        min_overlap=0.10,
        name="tower footprint is centered on the circular base",
    )
    ctx.expect_within(
        fan_drum,
        housing,
        axes="xy",
        margin=0.0,
        name="internal fan drum is inside the faceted housing footprint",
    )
    ctx.expect_gap(
        speed_knob,
        housing,
        axis="z",
        max_gap=0.0015,
        max_penetration=0.0,
        name="speed knob is seated on the top panel",
    )
    ctx.expect_gap(
        timer_knob,
        housing,
        axis="z",
        max_gap=0.0015,
        max_penetration=0.0,
        name="timer knob is seated on the top panel",
    )

    with ctx.pose({oscillation: 0.55, drum_spin: 1.6, speed_spin: 2.2, timer_spin: -1.8}):
        ctx.expect_within(
            fan_drum,
            housing,
            axes="xy",
            margin=0.0,
            name="spinning drum remains captured during oscillation",
        )

    return ctx.report()


object_model = build_object_model()
