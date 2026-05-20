from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


ENGINE_RPY = (0.0, math.pi / 2.0, 0.0)
ENGINE_CENTER_Z = 0.42
SLEEVE_TRAVEL = 0.14


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _shell_mesh(
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
    name: str,
    *,
    start_cap: str = "flat",
    end_cap: str = "flat",
):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=72,
            start_cap=start_cap,
            end_cap=end_cap,
            lip_samples=10,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="turbofan_on_maintenance_stand")

    nacelle_white = model.material("nacelle_white", rgba=(0.83, 0.85, 0.88, 1.0))
    sleeve_white = model.material("sleeve_white", rgba=(0.77, 0.79, 0.82, 1.0))
    stand_grey = model.material("stand_grey", rgba=(0.28, 0.30, 0.33, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.12, 0.12, 0.13, 1.0))
    bright_metal = model.material("bright_metal", rgba=(0.70, 0.72, 0.75, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.31, 0.33, 0.36, 1.0))

    stand = model.part("stand")
    stand.visual(
        Box((0.90, 0.06, 0.04)),
        origin=Origin(xyz=(0.0, -0.19, 0.02)),
        material=stand_grey,
        name="runner_0",
    )
    stand.visual(
        Box((0.90, 0.06, 0.04)),
        origin=Origin(xyz=(0.0, 0.19, 0.02)),
        material=stand_grey,
        name="runner_1",
    )
    stand.visual(
        Box((0.08, 0.44, 0.04)),
        origin=Origin(xyz=(-0.27, 0.0, 0.02)),
        material=stand_grey,
    )
    stand.visual(
        Box((0.08, 0.44, 0.04)),
        origin=Origin(xyz=(0.27, 0.0, 0.02)),
        material=stand_grey,
    )
    stand.visual(
        Box((0.12, 0.29, 0.02)),
        origin=Origin(xyz=(-0.18, 0.0, 0.05)),
        material=stand_grey,
        name="cradle_bridge",
    )
    stand.visual(
        Box((0.08, 0.04, 0.05)),
        origin=Origin(xyz=(-0.18, -0.145, 0.065)),
        material=stand_grey,
    )
    stand.visual(
        Box((0.08, 0.04, 0.05)),
        origin=Origin(xyz=(-0.18, 0.145, 0.065)),
        material=stand_grey,
    )
    stand.visual(
        Cylinder(radius=0.008, length=0.10),
        origin=Origin(xyz=(-0.18, -0.145, 0.094968), rpy=ENGINE_RPY),
        material=rubber_black,
        name="support_rail_0",
    )
    stand.visual(
        Cylinder(radius=0.008, length=0.10),
        origin=Origin(xyz=(-0.18, 0.145, 0.094968), rpy=ENGINE_RPY),
        material=rubber_black,
        name="support_rail_1",
    )

    stand.inertial = Inertial.from_geometry(
        Box((0.92, 0.46, 0.24)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
    )

    nacelle = model.part("nacelle")
    nacelle.visual(
        _shell_mesh(
            [
                (0.305, -0.60),
                (0.332, -0.55),
                (0.350, -0.40),
                (0.348, -0.22),
                (0.338, -0.05),
                (0.330, 0.08),
                (0.324, 0.18),
            ],
            [
                (0.270, -0.62),
                (0.294, -0.56),
                (0.302, -0.40),
                (0.300, -0.22),
                (0.292, -0.05),
                (0.284, 0.08),
                (0.278, 0.18),
            ],
            "front_shell",
            start_cap="round",
            end_cap="flat",
        ),
        origin=Origin(rpy=ENGINE_RPY),
        material=nacelle_white,
        name="front_shell",
    )
    nacelle.visual(
        _shell_mesh(
            [
                (0.324, 0.18),
                (0.326, 0.20),
                (0.324, 0.22),
                (0.320, 0.24),
            ],
            [
                (0.278, 0.18),
                (0.280, 0.20),
                (0.280, 0.22),
                (0.286, 0.24),
            ],
            "guide_shell",
        ),
        origin=Origin(rpy=ENGINE_RPY),
        material=dark_metal,
        name="guide_shell",
    )
    nacelle.visual(
        _shell_mesh(
            [
                (0.320, 0.24),
                (0.304, 0.36),
                (0.266, 0.50),
                (0.224, 0.62),
                (0.198, 0.70),
            ],
            [
                (0.286, 0.24),
                (0.258, 0.38),
                (0.214, 0.54),
                (0.168, 0.66),
                (0.140, 0.76),
            ],
            "nozzle_shell",
        ),
        origin=Origin(rpy=ENGINE_RPY),
        material=dark_metal,
        name="nozzle_shell",
    )
    nacelle.visual(
        Cylinder(radius=0.026, length=0.99),
        origin=Origin(xyz=(0.154, 0.0, 0.0), rpy=ENGINE_RPY),
        material=dark_metal,
        name="center_spine",
    )
    nacelle.visual(
        Cylinder(radius=0.010, length=0.60),
        origin=Origin(xyz=(0.34, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="support_vane_y",
    )
    nacelle.visual(
        Cylinder(radius=0.010, length=0.60),
        origin=Origin(xyz=(0.34, 0.0, 0.0)),
        material=dark_metal,
        name="support_vane_z",
    )
    nacelle.inertial = Inertial.from_geometry(
        Box((1.34, 0.72, 0.72)),
        mass=28.0,
        origin=Origin(xyz=(0.03, 0.0, 0.0)),
    )

    fan = model.part("fan")
    fan.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                0.285,
                0.092,
                18,
                thickness=0.070,
                blade_pitch_deg=34.0,
                blade_sweep_deg=26.0,
                blade=FanRotorBlade(
                    shape="scimitar",
                    tip_pitch_deg=16.0,
                    camber=0.18,
                    tip_clearance=0.010,
                ),
                hub=FanRotorHub(
                    style="spinner",
                    rear_collar_height=0.030,
                    rear_collar_radius=0.084,
                ),
            ),
            "fan_stage",
        ),
        origin=Origin(rpy=ENGINE_RPY),
        material=bright_metal,
        name="fan_stage",
    )
    fan.inertial = Inertial.from_geometry(
        Cylinder(radius=0.285, length=0.08),
        mass=3.2,
    )

    sleeve = model.part("sleeve")
    sleeve.visual(
        _shell_mesh(
            [
                (0.362, -0.08),
                (0.364, 0.00),
                (0.358, 0.16),
                (0.346, 0.26),
            ],
            [
                (0.344, -0.08),
                (0.346, 0.00),
                (0.340, 0.16),
                (0.330, 0.26),
            ],
            "sleeve_shell",
        ),
        origin=Origin(rpy=ENGINE_RPY),
        material=sleeve_white,
        name="sleeve_shell",
    )
    sleeve.inertial = Inertial.from_geometry(
        Cylinder(radius=0.36, length=0.36),
        mass=6.5,
    )

    model.articulation(
        "stand_to_nacelle",
        ArticulationType.FIXED,
        parent=stand,
        child=nacelle,
        origin=Origin(xyz=(0.0, 0.0, ENGINE_CENTER_Z)),
    )
    model.articulation(
        "nacelle_to_fan",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=fan,
        origin=Origin(xyz=(-0.39, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=35.0),
    )
    model.articulation(
        "nacelle_to_sleeve",
        ArticulationType.PRISMATIC,
        parent=nacelle,
        child=sleeve,
        origin=Origin(xyz=(0.09, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.35,
            lower=0.0,
            upper=SLEEVE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stand = object_model.get_part("stand")
    nacelle = object_model.get_part("nacelle")
    fan = object_model.get_part("fan")
    sleeve = object_model.get_part("sleeve")

    fan_spin = object_model.get_articulation("nacelle_to_fan")
    sleeve_slide = object_model.get_articulation("nacelle_to_sleeve")
    sleeve_limits = sleeve_slide.motion_limits

    ctx.allow_isolated_part(
        sleeve,
        reason="The thrust-reverser sleeve is intentionally modeled as a clearanced translating shell that rides on hidden internal guides rather than on visible external contact pads.",
    )

    ctx.expect_origin_distance(
        fan,
        nacelle,
        axes="yz",
        max_dist=0.001,
        name="fan stays on nacelle centerline",
    )
    ctx.expect_within(
        fan,
        nacelle,
        axes="yz",
        inner_elem="fan_stage",
        outer_elem="front_shell",
        margin=0.070,
        name="fan stays inside intake envelope",
    )
    ctx.expect_origin_distance(
        sleeve,
        nacelle,
        axes="yz",
        max_dist=0.001,
        name="sleeve stays concentric with nacelle body",
    )
    ctx.expect_overlap(
        sleeve,
        nacelle,
        axes="x",
        elem_a="sleeve_shell",
        elem_b="guide_shell",
        min_overlap=0.05,
        name="sleeve remains engaged with guide shell at rest",
    )

    stand_rail_aabb = ctx.part_element_world_aabb(stand, elem="support_rail_0")
    fan_pos = ctx.part_world_position(fan)
    ctx.check(
        "fan stage sits ahead of support cradle",
        stand_rail_aabb is not None
        and fan_pos is not None
        and fan_pos[0] < stand_rail_aabb[0][0] - 0.14,
        details=f"fan_pos={fan_pos}, stand_rail_aabb={stand_rail_aabb}",
    )

    with ctx.pose({fan_spin: 1.1}):
        ctx.expect_origin_distance(
            fan,
            nacelle,
            axes="yz",
            max_dist=0.001,
            name="fan spin keeps rotor centered",
        )

    if sleeve_limits is not None and sleeve_limits.upper is not None:
        rest_pos = ctx.part_world_position(sleeve)
        with ctx.pose({sleeve_slide: sleeve_limits.upper}):
            ctx.expect_origin_distance(
                sleeve,
                nacelle,
                axes="yz",
                max_dist=0.001,
                name="extended sleeve stays concentric",
            )
            ctx.expect_overlap(
                sleeve,
                nacelle,
                axes="x",
                elem_a="sleeve_shell",
                elem_b="guide_shell",
                min_overlap=0.04,
                name="extended sleeve retains guide engagement",
            )
            extended_pos = ctx.part_world_position(sleeve)
        ctx.check(
            "sleeve slides aft along engine axis",
            rest_pos is not None
            and extended_pos is not None
            and extended_pos[0] > rest_pos[0] + 0.10,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    return ctx.report()


object_model = build_object_model()
