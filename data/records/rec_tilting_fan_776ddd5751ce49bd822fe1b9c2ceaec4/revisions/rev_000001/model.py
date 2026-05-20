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
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tall_tilting_desk_fan")

    satin_white = model.material("satin_white", rgba=(0.88, 0.88, 0.84, 1.0))
    warm_gray = model.material("warm_gray", rgba=(0.42, 0.43, 0.42, 1.0))
    dark_wire = model.material("dark_wire", rgba=(0.07, 0.08, 0.08, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    blade_blue = model.material("smoky_blue_blade", rgba=(0.28, 0.50, 0.72, 0.88))

    # Root: a tall, narrow stand with a small yoke at the top.  The long stem and
    # small head proportions are intentionally closer to a slender desk/pedestal
    # fan than to a squat tabletop fan.
    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.165, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=satin_white,
        name="base_disc",
    )
    stand.visual(
        mesh_from_geometry(TorusGeometry(0.145, 0.006, radial_segments=16, tubular_segments=64), "rubber_foot_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=black_rubber,
        name="rubber_foot_ring",
    )
    stand.visual(
        Cylinder(radius=0.018, length=0.78),
        origin=Origin(xyz=(0.0, 0.0, 0.425)),
        material=satin_white,
        name="slender_stem",
    )
    stand.visual(
        Cylinder(radius=0.027, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.810)),
        material=warm_gray,
        name="height_collar",
    )
    stand.visual(
        Box((0.425, 0.038, 0.038)),
        origin=Origin(xyz=(0.0, 0.0, 0.810)),
        material=satin_white,
        name="yoke_bridge",
    )
    for suffix, x in (("0", 0.205), ("1", -0.205)):
        stand.visual(
            Box((0.026, 0.044, 0.270)),
            origin=Origin(xyz=(x, 0.0, 0.945)),
            material=satin_white,
            name=f"side_bracket_{suffix}",
        )
        stand.visual(
            Cylinder(radius=0.034, length=0.034),
            origin=Origin(xyz=(x, 0.0, 1.055), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=warm_gray,
            name=f"pivot_bushing_{suffix}",
        )

    # Tilting head: the local origin is the trunnion / tilt axis.  The guard is
    # a shallow wire cage with concentric rings, radial spokes, and longitudinal
    # side rods surrounding a comparatively small rotor.
    head = model.part("head")
    ring_meshes = {
        "outer": mesh_from_geometry(TorusGeometry(0.166, 0.0055, radial_segments=16, tubular_segments=72), "outer_guard_ring"),
        "middle": mesh_from_geometry(TorusGeometry(0.112, 0.0028, radial_segments=12, tubular_segments=64), "middle_guard_ring"),
        "inner": mesh_from_geometry(TorusGeometry(0.060, 0.0026, radial_segments=12, tubular_segments=48), "inner_guard_ring"),
    }
    for y, face in ((-0.064, "front"), (0.064, "rear")):
        for ring, mesh in ring_meshes.items():
            head.visual(
                mesh,
                origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
                material=dark_wire,
                name=f"{face}_{ring}_ring",
            )

    head.visual(
        Cylinder(radius=0.026, length=0.010),
        origin=Origin(xyz=(0.0, -0.064, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_wire,
        name="front_center_cap",
    )
    head.visual(
        Cylinder(radius=0.054, length=0.108),
        origin=Origin(xyz=(0.0, 0.110, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_white,
        name="motor_shell",
    )
    head.visual(
        Cylinder(radius=0.027, length=0.064),
        origin=Origin(xyz=(0.0, 0.024, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=warm_gray,
        name="axle_nose",
    )

    # Side trunnions visually continue the tilt axis into the bracket bushings.
    for suffix, x in (("0", 0.1765), ("1", -0.1765)):
        head.visual(
            Cylinder(radius=0.019, length=0.023),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=warm_gray,
            name=f"trunnion_{suffix}",
        )

    # Radial guard spokes in the front and rear planes.
    for face, y, start_r, spoke_count in (("front", -0.064, 0.022, 12), ("rear", 0.064, 0.050, 10)):
        end_r = 0.166
        length = end_r - start_r + 0.010
        mid_r = (end_r + start_r) / 2.0
        for i in range(spoke_count):
            theta = 2.0 * math.pi * i / spoke_count
            head.visual(
                Cylinder(radius=0.0022, length=length),
                origin=Origin(
                    xyz=(mid_r * math.cos(theta), y, mid_r * math.sin(theta)),
                    rpy=(0.0, math.pi / 2.0 - theta, 0.0),
                ),
                material=dark_wire,
                name=f"{face}_spoke_{i}",
            )

    # Longitudinal rods join the front and rear rings into a real cage.
    for i in range(12):
        theta = 2.0 * math.pi * i / 12
        head.visual(
            Cylinder(radius=0.0026, length=0.128),
            origin=Origin(
                xyz=(0.166 * math.cos(theta), 0.0, 0.166 * math.sin(theta)),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_wire,
            name=f"cage_rod_{i}",
        )

    rotor = model.part("rotor")
    rotor.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                0.124,
                0.032,
                5,
                thickness=0.022,
                blade_pitch_deg=31.0,
                blade_sweep_deg=28.0,
                blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=13.0, camber=0.18, tip_clearance=0.004),
                hub=FanRotorHub(style="spinner", rear_collar_height=0.006, rear_collar_radius=0.026),
            ),
            "rotor_blades",
        ),
        # FanRotorGeometry spins about local Z; rotate that axis onto the fan's
        # local +Y axle.
        origin=Origin(xyz=(0.0, -0.019, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=blade_blue,
        name="rotor_blades",
    )

    model.articulation(
        "stand_to_head",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 1.055)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=1.2, lower=-0.42, upper=0.42),
    )
    model.articulation(
        "head_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=rotor,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=70.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    head = object_model.get_part("head")
    rotor = object_model.get_part("rotor")
    tilt = object_model.get_articulation("stand_to_head")
    spin = object_model.get_articulation("head_to_rotor")

    ctx.check(
        "head tilt is horizontal",
        tuple(round(v, 3) for v in tilt.axis) == (1.0, 0.0, 0.0),
        details=f"tilt axis={tilt.axis}",
    )
    ctx.check(
        "rotor spin is continuous",
        spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"spin type={spin.articulation_type}",
    )
    ctx.expect_within(
        rotor,
        head,
        axes="xz",
        inner_elem="rotor_blades",
        outer_elem="front_outer_ring",
        margin=0.010,
        name="rotor fits inside guard diameter",
    )
    ctx.expect_gap(
        stand,
        head,
        axis="x",
        positive_elem="pivot_bushing_0",
        negative_elem="trunnion_0",
        min_gap=0.0,
        max_gap=0.006,
        name="side bracket closely captures trunnion",
    )

    closed_cap = ctx.part_element_world_aabb(head, elem="front_center_cap")
    with ctx.pose({tilt: tilt.motion_limits.upper}):
        tilted_cap = ctx.part_element_world_aabb(head, elem="front_center_cap")
    closed_z = (closed_cap[0][2] + closed_cap[1][2]) / 2.0 if closed_cap else None
    tilted_z = (tilted_cap[0][2] + tilted_cap[1][2]) / 2.0 if tilted_cap else None
    ctx.check(
        "head visibly tilts about side brackets",
        closed_z is not None and tilted_z is not None and abs(tilted_z - closed_z) > 0.018,
        details=f"front center cap z at rest={closed_z}, at tilt={tilted_z}",
    )

    return ctx.report()


object_model = build_object_model()
