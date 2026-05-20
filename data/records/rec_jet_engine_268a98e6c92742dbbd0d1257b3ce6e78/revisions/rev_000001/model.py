from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


FRONT_X = -1.05
FAN_X = -0.72
REAR_X = 1.20


def _smoothstep(t: float) -> float:
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


def _inner_radius(x: float) -> float:
    """Approximate inlet/exhaust duct radius along the engine axis."""
    t = _smoothstep((x - FRONT_X) / (REAR_X - FRONT_X))
    return 0.505 - 0.145 * t


def _nacelle_shell() -> cq.Workplane:
    outer: list[tuple[float, float]] = []
    for i in range(34):
        t = i / 33.0
        x = -1.00 + (REAR_X + 0.02 + 1.00) * t
        aft_taper = _smoothstep((x + 0.18) / 1.45)
        shoulder = 0.075 * math.exp(-((x + 0.58) / 0.42) ** 2)
        r = 0.625 + shoulder - 0.185 * aft_taper
        outer.append((x, r))

    inner: list[tuple[float, float]] = []
    for i in range(34):
        t = i / 33.0
        x = REAR_X - 0.06 - (REAR_X - 0.06 + 1.00) * t
        inner.append((x, _inner_radius(x)))

    # The final points roll the inner duct out into a broad rounded intake lip.
    lip = [(-1.060, 0.520), (-1.095, 0.585), (-1.060, 0.635)]
    profile = outer + inner + lip
    return (
        cq.Workplane("XY")
        .polyline(profile)
        .close()
        .revolve(360.0, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    )


def _intake_liner() -> cq.Workplane:
    outer: list[tuple[float, float]] = []
    inner: list[tuple[float, float]] = []
    x0, x1 = -1.02, 0.96
    for i in range(28):
        t = i / 27.0
        x = x0 + (x1 - x0) * t
        outer.append((x, _inner_radius(x) + 0.003))
    for i in range(28):
        t = i / 27.0
        x = x1 - (x1 - x0) * t
        inner.append((x, _inner_radius(x) - 0.012))
    return (
        cq.Workplane("XY")
        .polyline(outer + inner)
        .close()
        .revolve(360.0, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    )


def _spinner() -> cq.Workplane:
    profile = [
        (-0.245, 0.000),
        (-0.230, 0.030),
        (-0.185, 0.075),
        (-0.110, 0.112),
        (-0.020, 0.132),
        (0.055, 0.135),
        (0.058, 0.000),
    ]
    return (
        cq.Workplane("XY")
        .polyline(profile)
        .close()
        .revolve(360.0, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    )


def _exhaust_cone() -> cq.Workplane:
    profile = [
        (0.70, 0.000),
        (0.70, 0.185),
        (0.82, 0.175),
        (1.02, 0.120),
        (1.18, 0.040),
        (1.22, 0.000),
    ]
    return (
        cq.Workplane("XY")
        .polyline(profile)
        .close()
        .revolve(360.0, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    )


def _radial_strut_set(
    *,
    x: float,
    inner_radius: float,
    outer_radius: float,
    chord: float,
    thickness: float,
    angles_deg: tuple[float, ...],
) -> cq.Workplane:
    radial_len = outer_radius - inner_radius
    radial_mid = (outer_radius + inner_radius) / 2.0
    combined: cq.Workplane | None = None
    for angle in angles_deg:
        strut = (
            cq.Workplane("XY")
            .box(chord, radial_len, thickness)
            .translate((x, radial_mid, 0.0))
            .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), angle)
        )
        combined = strut if combined is None else combined.union(strut)
    assert combined is not None
    return combined


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="regional_airliner_turbofan")

    white = model.material("warm_white_cowl", rgba=(0.86, 0.88, 0.86, 1.0))
    dark = model.material("dark_graphite", rgba=(0.035, 0.038, 0.042, 1.0))
    titanium = model.material("brushed_titanium", rgba=(0.46, 0.48, 0.50, 1.0))
    spinner_mat = model.material("polished_spinner", rgba=(0.72, 0.75, 0.76, 1.0))

    nacelle = model.part("nacelle")
    nacelle.visual(
        mesh_from_cadquery(_nacelle_shell(), "cowling", tolerance=0.003, angular_tolerance=0.08),
        material=white,
        name="cowling",
    )
    nacelle.visual(
        mesh_from_cadquery(_intake_liner(), "intake_liner", tolerance=0.003, angular_tolerance=0.08),
        material=dark,
        name="intake_liner",
    )
    nacelle.visual(
        Cylinder(radius=0.110, length=0.120),
        origin=Origin(xyz=(-0.520, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=titanium,
        name="bearing_boss",
    )
    nacelle.visual(
        mesh_from_cadquery(
            _radial_strut_set(
                x=-0.515,
                inner_radius=0.103,
                outer_radius=0.500,
                chord=0.100,
                thickness=0.034,
                angles_deg=(0.0, 90.0, 180.0, 270.0),
            ),
            "inlet_struts",
            tolerance=0.002,
            angular_tolerance=0.08,
        ),
        material=dark,
        name="inlet_struts",
    )
    nacelle.visual(
        mesh_from_cadquery(_exhaust_cone(), "exhaust_cone", tolerance=0.0025, angular_tolerance=0.08),
        material=dark,
        name="exhaust_cone",
    )
    nacelle.visual(
        mesh_from_cadquery(
            _radial_strut_set(
                x=0.800,
                inner_radius=0.170,
                outer_radius=0.410,
                chord=0.085,
                thickness=0.028,
                angles_deg=(45.0, 135.0, 225.0, 315.0),
            ),
            "tail_struts",
            tolerance=0.002,
            angular_tolerance=0.08,
        ),
        material=dark,
        name="tail_struts",
    )

    fan_rotor = model.part("fan_rotor")
    fan_rotor.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                outer_radius=0.455,
                hub_radius=0.130,
                blade_count=18,
                thickness=0.090,
                blade_pitch_deg=34.0,
                blade_sweep_deg=31.0,
                blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=15.0, camber=0.12, tip_clearance=0.012),
                hub=FanRotorHub(style="flat", rear_collar_height=0.018, rear_collar_radius=0.118),
            ),
            "fan_blades",
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=titanium,
        name="fan_blades",
    )
    fan_rotor.visual(
        mesh_from_cadquery(_spinner(), "spinner", tolerance=0.002, angular_tolerance=0.08),
        material=spinner_mat,
        name="spinner",
    )
    fan_rotor.visual(
        Cylinder(radius=0.095, length=0.100),
        origin=Origin(xyz=(0.090, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=titanium,
        name="rear_collar",
    )

    model.articulation(
        "fan_spin",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=fan_rotor,
        origin=Origin(xyz=(FAN_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=220.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    nacelle = object_model.get_part("nacelle")
    fan_rotor = object_model.get_part("fan_rotor")
    spin = object_model.get_articulation("fan_spin")

    ctx.check(
        "fan rotor uses continuous longitudinal spin",
        spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (1.0, 0.0, 0.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )
    ctx.expect_within(
        fan_rotor,
        nacelle,
        axes="yz",
        inner_elem="fan_blades",
        outer_elem="cowling",
        margin=0.0,
        name="fan disk fits inside nacelle envelope",
    )
    ctx.expect_contact(
        fan_rotor,
        nacelle,
        elem_a="rear_collar",
        elem_b="bearing_boss",
        contact_tol=0.003,
        name="rotor collar remains seated on center bearing",
    )

    rest_pos = ctx.part_world_position(fan_rotor)
    with ctx.pose({spin: math.pi * 1.25}):
        ctx.expect_within(
            fan_rotor,
            nacelle,
            axes="yz",
            inner_elem="fan_blades",
            outer_elem="cowling",
            margin=0.0,
            name="rotated fan stays centered within nacelle",
        )
        ctx.expect_contact(
            fan_rotor,
            nacelle,
            elem_a="rear_collar",
            elem_b="bearing_boss",
            contact_tol=0.003,
            name="rotated collar remains clipped to bearing",
        )
        rotated_pos = ctx.part_world_position(fan_rotor)

    ctx.check(
        "continuous spin preserves rotor center",
        rest_pos is not None
        and rotated_pos is not None
        and max(abs(a - b) for a, b in zip(rest_pos, rotated_pos)) < 1.0e-6,
        details=f"rest={rest_pos}, rotated={rotated_pos}",
    )

    return ctx.report()


object_model = build_object_model()
