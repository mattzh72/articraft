from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    ExtrudeWithHolesGeometry,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


PETAL_COUNT = 12
FRONT_X = -1.18
REAR_RING_X = 1.08
HINGE_X = 1.120
HINGE_RADIUS = 0.322
PETAL_LENGTH = 0.48
PETAL_LIMIT_LOWER = -0.14
PETAL_LIMIT_UPPER = 0.34


def _petal_panel_geometry(layer_offset: float) -> MeshGeometry:
    """A tapered, thin, swept nozzle flap in the petal's local hinge frame.

    Local +X is downstream, local +Y is tangent to the exhaust ring, and local
    +Z is radially outward.  The panel converges inward toward the exhaust, and
    alternating layer offsets make neighboring panels read as overlapped
    shingles without requiring broad interpenetration.
    """

    root_x = 0.012
    tip_x = PETAL_LENGTH
    root_width = 0.138
    tip_width = 0.182
    thickness = 0.009
    tip_center_z = -0.058 + layer_offset
    root_center_z = 0.0

    sections = [
        (root_x, root_width, root_center_z),
        (tip_x, tip_width, tip_center_z),
    ]
    vertices: list[tuple[float, float, float]] = []
    for x, width, zc in sections:
        for y, z in (
            (-width / 2.0, zc - thickness / 2.0),
            (width / 2.0, zc - thickness / 2.0),
            (width / 2.0, zc + thickness / 2.0),
            (-width / 2.0, zc + thickness / 2.0),
        ):
            vertices.append((x, y, z))

    faces = [
        # root and tip faces
        (0, 1, 2),
        (0, 2, 3),
        (4, 6, 5),
        (4, 7, 6),
        # lower, upper, and side skins
        (0, 4, 5),
        (0, 5, 1),
        (3, 2, 6),
        (3, 6, 7),
        (0, 3, 7),
        (0, 7, 4),
        (1, 5, 6),
        (1, 6, 2),
    ]

    geom = MeshGeometry()
    geom.vertices = vertices
    geom.faces = faces
    return geom


def _petal_frame(theta: float) -> Origin:
    """Place a petal hinge frame around the circular exhaust ring."""

    return Origin(
        xyz=(HINGE_X, HINGE_RADIUS * math.cos(theta), HINGE_RADIUS * math.sin(theta)),
        # local +X downstream, local +Z radially outward, local +Y tangential
        rpy=(theta - math.pi / 2.0, 0.0, 0.0),
    )


def _circle_profile(radius: float, segments: int = 96) -> list[tuple[float, float]]:
    return [
        (radius * math.cos(2.0 * math.pi * i / segments), radius * math.sin(2.0 * math.pi * i / segments))
        for i in range(segments)
    ]


def _annular_sector_hole(
    inner_radius: float,
    outer_radius: float,
    center_angle: float,
    width_angle: float,
    *,
    samples: int = 5,
) -> list[tuple[float, float]]:
    """A rounded wedge opening for the one-piece front stator/bearing web."""

    a0 = center_angle - width_angle / 2.0
    a1 = center_angle + width_angle / 2.0
    pts: list[tuple[float, float]] = []
    for j in range(samples):
        a = a0 + (a1 - a0) * j / (samples - 1)
        pts.append((outer_radius * math.cos(a), outer_radius * math.sin(a)))
    for j in range(samples):
        a = a1 - (a1 - a0) * j / (samples - 1)
        pts.append((inner_radius * math.cos(a), inner_radius * math.sin(a)))
    return pts


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fighter_jet_engine")

    titanium = model.material("brushed_titanium", rgba=(0.48, 0.50, 0.52, 1.0))
    dark_metal = model.material("dark_burnt_metal", rgba=(0.09, 0.095, 0.10, 1.0))
    fan_metal = model.material("compressor_blade_steel", rgba=(0.70, 0.72, 0.74, 1.0))
    hot_nozzle = model.material("heat_stained_nozzle", rgba=(0.38, 0.31, 0.25, 1.0))
    hinge_metal = model.material("dark_hinge_metal", rgba=(0.18, 0.17, 0.16, 1.0))

    core = model.part("core")

    # Thin-walled lathed nacelle/core casing.  It stays open at both ends so the
    # fan and exhaust read as cavities rather than capped cylinders.
    outer_profile = [
        (0.430, FRONT_X),
        (0.445, -1.05),
        (0.390, -0.42),
        (0.365, 0.45),
        (0.345, REAR_RING_X),
    ]
    inner_profile = [
        (0.335, FRONT_X),
        (0.320, -1.02),
        (0.292, -0.35),
        (0.270, 0.55),
        (0.245, REAR_RING_X),
    ]
    core.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                outer_profile,
                inner_profile,
                segments=96,
                start_cap="flat",
                end_cap="flat",
            ),
            "core_shell",
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=titanium,
        name="core_shell",
    )

    core.visual(
        mesh_from_geometry(TorusGeometry(radius=0.383, tube=0.030, radial_segments=18, tubular_segments=96), "front_lip"),
        origin=Origin(xyz=(FRONT_X, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=titanium,
        name="front_lip",
    )
    core.visual(
        mesh_from_geometry(TorusGeometry(radius=0.305, tube=0.030, radial_segments=16, tubular_segments=96), "exhaust_ring"),
        origin=Origin(xyz=(REAR_RING_X, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="exhaust_ring",
    )
    core.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(
                _circle_profile(0.326, 96),
                [
                    _annular_sector_hole(
                        inner_radius=0.082,
                        outer_radius=0.292,
                        center_angle=2.0 * math.pi * j / 8.0 + math.pi / 8.0,
                        width_angle=0.46,
                    )
                    for j in range(8)
                ],
                0.026,
                center=True,
            ),
            "front_stator",
        ),
        origin=Origin(xyz=(-1.018, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="front_stator",
    )

    fan = model.part("fan")
    fan.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                outer_radius=0.305,
                hub_radius=0.072,
                blade_count=13,
                thickness=0.075,
                blade_pitch_deg=36.0,
                blade_sweep_deg=33.0,
                blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=18.0, camber=0.14, tip_clearance=0.006),
                hub=FanRotorHub(style="spinner", bore_diameter=0.028),
            ),
            "front_fan",
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=fan_metal,
        name="front_fan",
    )
    fan.visual(
        Cylinder(radius=0.060, length=0.032),
        origin=Origin(xyz=(0.058, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=fan_metal,
        name="rear_bearing_collar",
    )
    model.articulation(
        "fan_spin",
        ArticulationType.CONTINUOUS,
        parent=core,
        child=fan,
        origin=Origin(xyz=(-1.105, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=160.0),
    )

    for i in range(PETAL_COUNT):
        theta = 2.0 * math.pi * i / PETAL_COUNT
        petal = model.part(f"petal_{i}")
        layer = 0.014 if i % 2 == 0 else -0.014
        petal.visual(
            mesh_from_geometry(_petal_panel_geometry(layer), f"petal_panel_{i}"),
            material=hot_nozzle,
            name="panel",
        )
        petal.visual(
            Cylinder(radius=0.017, length=0.074),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hinge_metal,
            name="hinge_knuckle",
        )
        model.articulation(
            f"petal_{i}_hinge",
            ArticulationType.REVOLUTE,
            parent=core,
            child=petal,
            origin=_petal_frame(theta),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=18.0,
                velocity=1.5,
                lower=PETAL_LIMIT_LOWER,
                upper=PETAL_LIMIT_UPPER,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    core = object_model.get_part("core")
    fan = object_model.get_part("fan")
    fan_spin = object_model.get_articulation("fan_spin")

    ctx.check(
        "fan has continuous centerline joint",
        fan_spin is not None and fan_spin.articulation_type == ArticulationType.CONTINUOUS and tuple(fan_spin.axis) == (1.0, 0.0, 0.0),
        details=f"joint={fan_spin!r}",
    )
    fan_pos = ctx.part_world_position(fan)
    ctx.check(
        "fan origin lies on engine axis",
        fan_pos is not None and abs(fan_pos[1]) < 1e-6 and abs(fan_pos[2]) < 1e-6,
        details=f"fan_pos={fan_pos!r}",
    )
    ctx.expect_within(
        fan,
        core,
        axes="yz",
        margin=0.018,
        inner_elem="front_fan",
        outer_elem="front_lip",
        name="fan fits inside front inlet lip",
    )
    ctx.expect_gap(
        core,
        fan,
        axis="x",
        positive_elem="front_stator",
        negative_elem="rear_bearing_collar",
        max_gap=0.004,
        max_penetration=0.001,
        name="fan bearing collar seats against stator web",
    )

    for i in range(PETAL_COUNT):
        petal = object_model.get_part(f"petal_{i}")
        hinge = object_model.get_articulation(f"petal_{i}_hinge")
        ctx.check(
            f"petal_{i} has rear revolute hinge",
            hinge is not None
            and hinge.articulation_type == ArticulationType.REVOLUTE
            and hinge.motion_limits is not None
            and hinge.motion_limits.lower == PETAL_LIMIT_LOWER
            and hinge.motion_limits.upper == PETAL_LIMIT_UPPER,
            details=f"hinge={hinge!r}",
        )
        ctx.allow_overlap(
            core,
            petal,
            elem_a="exhaust_ring",
            elem_b="hinge_knuckle",
            reason="Each nozzle petal hinge knuckle is intentionally captured in the exhaust-ring hinge band.",
        )
        ctx.expect_gap(
            petal,
            core,
            axis="x",
            positive_elem="hinge_knuckle",
            negative_elem="exhaust_ring",
            max_gap=0.006,
            max_penetration=0.010,
            name=f"petal_{i} hinge sits on exhaust ring",
        )

    petal_0 = object_model.get_part("petal_0")
    hinge_0 = object_model.get_articulation("petal_0_hinge")
    rest_aabb = ctx.part_world_aabb(petal_0)
    with ctx.pose({hinge_0: PETAL_LIMIT_UPPER}):
        open_aabb = ctx.part_world_aabb(petal_0)
    ctx.check(
        "nozzle petal upper limit opens outward",
        rest_aabb is not None and open_aabb is not None and open_aabb[1][1] > rest_aabb[1][1] + 0.035,
        details=f"rest={rest_aabb!r}, open={open_aabb!r}",
    )

    return ctx.report()


object_model = build_object_model()
