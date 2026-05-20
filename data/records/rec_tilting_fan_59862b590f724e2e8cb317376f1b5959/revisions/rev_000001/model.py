from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    superellipse_profile,
)


def _torus_mesh(radius: float, tube: float, name: str):
    """Build a guard ring in the local YZ plane, ready for a fan head."""
    ring = TorusGeometry(radius, tube, radial_segments=18, tubular_segments=80)
    # TorusGeometry is centered in the XY plane around local Z; a +90 deg pitch
    # makes the ring's normal local +X, matching the fan axle direction.
    ring.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(ring, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desk_fan_tilting_head")

    dark_plastic = model.material("matte_dark_plastic", rgba=(0.045, 0.050, 0.055, 1.0))
    grey = model.material("warm_grey_plastic", rgba=(0.55, 0.58, 0.58, 1.0))
    pale_grey = model.material("pale_guard_wire", rgba=(0.82, 0.85, 0.84, 1.0))
    brushed = model.material("brushed_metal", rgba=(0.68, 0.68, 0.63, 1.0))
    translucent_blade = model.material("smoky_translucent_blade", rgba=(0.42, 0.62, 0.78, 0.62))

    stand = model.part("stand")
    base_profile = superellipse_profile(0.70, 0.58, exponent=2.7, segments=80)
    stand.visual(
        mesh_from_geometry(
            ExtrudeGeometry(base_profile, 0.045, center=True),
            "base_slab",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=dark_plastic,
        name="base_slab",
    )
    stand.visual(
        Cylinder(radius=0.045, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=grey,
        name="central_pedestal",
    )
    stand.visual(
        Box((0.200, 0.180, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=grey,
        name="yoke_saddle",
    )

    # Two side brackets straddle the guarded fan head and put the tilt axis
    # through the fan axle, not high above it.  Their bases penetrate the broad
    # low support so the assembly reads as one molded stand.
    for i, y in enumerate((0.275, -0.275)):
        stand.visual(
            Box((0.075, 0.036, 0.285)),
            origin=Origin(xyz=(0.0, y, 0.1825)),
            material=grey,
            name=f"side_bracket_{i}",
        )
        stand.visual(
            Cylinder(radius=0.033, length=0.052),
            origin=Origin(xyz=(0.0, 0.259 if y > 0.0 else -0.259, 0.320), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=brushed,
            name=f"pivot_boss_{i}",
        )

    head = model.part("head")
    # Rear motor pod and axle boss: compact and low on the support, like a real
    # desktop circulator rather than a tall pedestal fan.
    head.visual(
        Cylinder(radius=0.078, length=0.120),
        origin=Origin(xyz=(-0.083, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="motor_pod",
    )
    head.visual(
        Cylinder(radius=0.0125, length=0.112),
        origin=Origin(xyz=(-0.012, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed,
        name="axle",
    )

    # Guard cage: two circular wire grilles with radial spokes and side rails.
    front_x = 0.056
    rear_x = -0.056
    for x, prefix in ((front_x, "front"), (rear_x, "rear")):
        for radius, label, tube in (
            (0.204, "outer", 0.0060),
            (0.147, "middle", 0.0036),
            (0.088, "inner", 0.0032),
        ):
            head.visual(
                _torus_mesh(radius, tube, f"{prefix}_{label}_guard_ring"),
                origin=Origin(xyz=(x, 0.0, 0.0)),
                material=pale_grey,
                name=f"{prefix}_{label}_guard_ring",
            )
        for spoke in range(12):
            theta = spoke * math.pi / 12.0
            head.visual(
                Cylinder(radius=0.0024, length=0.398),
                origin=Origin(xyz=(x, 0.0, 0.0), rpy=(-theta, 0.0, 0.0)),
                material=pale_grey,
                name=f"{prefix}_spoke_{spoke}",
            )

    for i, (y, z) in enumerate(((0.0, 0.204), (0.0, -0.204), (0.204, 0.0), (-0.204, 0.0))):
        head.visual(
            Cylinder(radius=0.0045, length=0.112),
            origin=Origin(xyz=(0.0, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=pale_grey,
            name=f"cage_side_rail_{i}",
        )

    head.visual(
        Cylinder(radius=0.038, length=0.018),
        origin=Origin(xyz=(0.063, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grey,
        name="front_medallion",
    )
    for i, y in enumerate((0.229, -0.229)):
        head.visual(
            Cylinder(radius=0.023, length=0.047),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=brushed,
            name=f"side_pin_{i}",
        )

    rotor = model.part("rotor")
    fan_rotor = FanRotorGeometry(
        outer_radius=0.152,
        hub_radius=0.038,
        blade_count=5,
        thickness=0.026,
        blade_pitch_deg=32.0,
        blade_sweep_deg=27.0,
        blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=15.0, camber=0.14, tip_clearance=0.010),
        hub=FanRotorHub(style="spinner", rear_collar_height=0.010, rear_collar_radius=0.026, bore_diameter=0.024),
    )
    rotor.visual(
        mesh_from_geometry(fan_rotor, "five_blade_rotor"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=translucent_blade,
        name="five_blade_rotor",
    )

    model.articulation(
        "stand_to_head",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.320)),
        # Positive tilt lifts the front grille upward.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=-0.38, upper=0.55),
    )
    model.articulation(
        "head_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=rotor,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=80.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    head = object_model.get_part("head")
    rotor = object_model.get_part("rotor")
    tilt = object_model.get_articulation("stand_to_head")
    spin = object_model.get_articulation("head_to_rotor")

    for i in range(2):
        ctx.allow_overlap(
            stand,
            head,
            elem_a=f"pivot_boss_{i}",
            elem_b=f"side_pin_{i}",
            reason="The tilting axle pin is intentionally captured inside the side bracket pivot boss.",
        )
        ctx.expect_overlap(
            stand,
            head,
            axes="y",
            elem_a=f"pivot_boss_{i}",
            elem_b=f"side_pin_{i}",
            min_overlap=0.010,
            name=f"pivot pin {i} is retained through the bracket boss",
        )
        ctx.expect_overlap(
            stand,
            head,
            axes="xz",
            elem_a=f"pivot_boss_{i}",
            elem_b=f"side_pin_{i}",
            min_overlap=0.030,
            name=f"pivot pin {i} is coaxial with the bracket boss",
        )

    ctx.allow_overlap(
        head,
        rotor,
        elem_a="axle",
        elem_b="five_blade_rotor",
        reason="The motor shaft is intentionally seated through the rotor hub bore with a slight captured fit.",
    )
    ctx.expect_overlap(
        head,
        rotor,
        axes="x",
        elem_a="axle",
        elem_b="five_blade_rotor",
        min_overlap=0.020,
        name="rotor hub remains on the axle shaft",
    )
    ctx.expect_within(
        head,
        rotor,
        axes="yz",
        inner_elem="axle",
        outer_elem="five_blade_rotor",
        margin=0.002,
        name="axle is centered within the rotor hub envelope",
    )

    ctx.check(
        "fan has stand head and rotor parts",
        {part.name for part in object_model.parts} == {"stand", "head", "rotor"},
    )
    ctx.check("head tilt is revolute", tilt.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("rotor spin is continuous", spin.articulation_type == ArticulationType.CONTINUOUS)
    ctx.check("tilt axis crosses the side brackets", abs(tilt.axis[1]) > 0.99)
    ctx.check("rotor spin axis follows the fan axle", abs(spin.axis[0]) > 0.99)

    limits = tilt.motion_limits
    ctx.check(
        "tilt has realistic limited range",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and -0.6 < limits.lower < 0.0
        and 0.35 < limits.upper < 0.8,
    )

    base_box = ctx.part_element_world_aabb(stand, elem="base_slab")
    if base_box is not None:
        bmin, bmax = base_box
        ctx.check(
            "base is low and wide",
            (bmax[0] - bmin[0]) > 0.55 and (bmax[1] - bmin[1]) > 0.36 and (bmax[2] - bmin[2]) < 0.060,
            details=f"base bounds={base_box}",
        )

    rotor_pos = ctx.part_world_position(rotor)
    ctx.check(
        "rotating mass stays close to the mounting plane",
        rotor_pos is not None and 0.25 < rotor_pos[2] < 0.40,
        details=f"rotor position={rotor_pos}",
    )

    rest_front = ctx.part_element_world_aabb(head, elem="front_outer_guard_ring")
    with ctx.pose({tilt: limits.upper if limits is not None and limits.upper is not None else 0.5}):
        tilted_front = ctx.part_element_world_aabb(head, elem="front_outer_guard_ring")
    if rest_front is not None and tilted_front is not None:
        rest_z = 0.5 * (rest_front[0][2] + rest_front[1][2])
        tilted_z = 0.5 * (tilted_front[0][2] + tilted_front[1][2])
        ctx.check(
            "positive tilt lifts the fan face",
            tilted_z > rest_z + 0.020,
            details=f"rest_z={rest_z:.3f}, tilted_z={tilted_z:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
