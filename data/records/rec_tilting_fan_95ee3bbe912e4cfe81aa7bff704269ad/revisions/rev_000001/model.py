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
    Sphere,
    TorusGeometry,
    mesh_from_geometry,
    superellipse_profile,
    TestContext,
    TestReport,
)


PI = math.pi


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_tilting_desktop_fan")

    matte_white = model.material("matte_white", rgba=(0.86, 0.86, 0.82, 1.0))
    warm_grey = model.material("warm_grey", rgba=(0.48, 0.50, 0.50, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.05, 0.055, 0.06, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.66, 0.68, 0.68, 1.0))
    translucent_blue = model.material("translucent_blue", rgba=(0.36, 0.70, 0.95, 0.62))

    # Root link: a low oval desktop foot plus the fixed fork/yoke that carries
    # the side tilt pivots.  Dimensions are deliberately apartment/desktop scale.
    base = model.part("base")
    base.visual(
        mesh_from_geometry(
            ExtrudeGeometry(superellipse_profile(0.340, 0.178, exponent=3.0, segments=72), 0.028),
            "oval_base",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=matte_white,
        name="oval_base",
    )
    base.visual(
        Cylinder(radius=0.030, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.066)),
        material=matte_white,
        name="short_neck",
    )
    base.visual(
        Cylinder(radius=0.013, length=0.320),
        origin=Origin(xyz=(0.0, 0.0, 0.075), rpy=(0.0, PI / 2.0, 0.0)),
        material=matte_white,
        name="fork_crossbar",
    )
    for i, x in enumerate((-0.160, 0.160)):
        base.visual(
            Cylinder(radius=0.010, length=0.125),
            origin=Origin(xyz=(x, 0.0, 0.1375)),
            material=matte_white,
            name=f"side_arm_{i}",
        )
        base.visual(
            Cylinder(radius=0.024, length=0.032),
            origin=Origin(xyz=(x, 0.0, 0.224), rpy=(0.0, PI / 2.0, 0.0)),
            material=brushed_steel,
            name=f"pivot_socket_{i}",
        )
        base.visual(
            Cylinder(radius=0.030, length=0.007),
            origin=Origin(
                xyz=(x + (-0.018 if x < 0.0 else 0.018), 0.0, 0.224),
                rpy=(0.0, PI / 2.0, 0.0),
            ),
            material=dark_rubber,
            name=f"friction_knob_{i}",
        )
    for i, (x, y) in enumerate(((-0.105, -0.056), (0.105, -0.056), (-0.105, 0.056), (0.105, 0.056))):
        base.visual(
            Cylinder(radius=0.014, length=0.006),
            origin=Origin(xyz=(x, y, 0.002)),
            material=dark_rubber,
            name=f"rubber_foot_{i}",
        )

    # Tilting child link.  Its local Z is the fan shaft/spin axis; the tilt
    # articulation below rotates about local X through the side trunnions.
    head = model.part("fan_head")

    outer_ring_mesh = mesh_from_geometry(TorusGeometry(0.132, 0.0055, radial_segments=18, tubular_segments=72), "guard_outer_ring")
    mid_ring_mesh = mesh_from_geometry(TorusGeometry(0.092, 0.0032, radial_segments=14, tubular_segments=64), "guard_mid_ring")
    inner_ring_mesh = mesh_from_geometry(TorusGeometry(0.054, 0.0028, radial_segments=12, tubular_segments=56), "guard_inner_ring")
    for z, face, outer_name, mid_name, inner_name, hub_name in (
        (-0.041, "front", "front_outer_ring", "front_mid_ring", "front_inner_ring", "front_guard_hub"),
        (0.041, "rear", "rear_outer_ring", "rear_mid_ring", "rear_inner_ring", "rear_guard_hub"),
    ):
        head.visual(outer_ring_mesh, origin=Origin(xyz=(0.0, 0.0, z)), material=matte_white, name=outer_name)
        head.visual(mid_ring_mesh, origin=Origin(xyz=(0.0, 0.0, z)), material=matte_white, name=mid_name)
        head.visual(inner_ring_mesh, origin=Origin(xyz=(0.0, 0.0, z)), material=matte_white, name=inner_name)
        head.visual(
            Cylinder(radius=0.022, length=0.007),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=matte_white,
            name=hub_name,
        )
        for spoke in range(12):
            angle = spoke * (2.0 * PI / 12.0)
            radius_inner = 0.020
            length = 0.110
            mid = radius_inner + length / 2.0
            head.visual(
                Cylinder(radius=0.0022, length=length),
                origin=Origin(
                    xyz=(mid * math.cos(angle), mid * math.sin(angle), z),
                    rpy=(0.0, PI / 2.0, angle),
                ),
                material=matte_white,
                name=f"{face}_spoke_{spoke}",
            )

    for rib in range(8):
        angle = rib * (2.0 * PI / 8.0) + PI / 8.0
        head.visual(
            Cylinder(radius=0.0028, length=0.082),
            origin=Origin(xyz=(0.132 * math.cos(angle), 0.132 * math.sin(angle), 0.0)),
            material=matte_white,
            name=f"rim_standoff_{rib}",
        )

    # Motor and trunnions are fixed to the guard, proving the tilt axis is
    # carried by real side pivots rather than a floating fan disk.
    head.visual(
        Cylinder(radius=0.045, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.074)),
        material=warm_grey,
        name="motor_can",
    )
    head.visual(
        Cylinder(radius=0.006, length=0.104),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=brushed_steel,
        name="motor_shaft",
    )
    head.visual(
        Sphere(radius=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.107)),
        material=warm_grey,
        name="motor_cap",
    )
    for i, x in enumerate((-0.160, 0.160)):
        head.visual(
            Box((0.020, 0.026, 0.087)),
            origin=Origin(xyz=(x * 0.82, 0.0, 0.0)),
            material=matte_white,
            name=f"side_bridge_{i}",
        )
        head.visual(
            Cylinder(radius=0.017, length=0.048),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, PI / 2.0, 0.0)),
            material=brushed_steel,
            name=f"pivot_pin_{i}",
        )
        head.visual(
            Box((0.024, 0.038, 0.018)),
            origin=Origin(xyz=(x * 0.80, 0.0, 0.0)),
            material=matte_white,
            name=f"side_boss_{i}",
        )

    # Spinning child link.  The rotor geometry is centered on local Z and is
    # mounted with a continuous joint at the same explicit shaft/hub axis.
    rotor = model.part("rotor")
    rotor.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                0.104,
                0.024,
                5,
                thickness=0.016,
                blade_pitch_deg=31.0,
                blade_sweep_deg=24.0,
                blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=13.0, camber=0.14, tip_clearance=0.003),
                hub=FanRotorHub(style="spinner", rear_collar_height=0.010, rear_collar_radius=0.018, bore_diameter=0.006),
            ),
            "scimitar_rotor",
        ),
        origin=Origin(),
        material=translucent_blue,
        name="rotor_blades",
    )

    tilt = model.articulation(
        "base_to_fan_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.224), rpy=(-PI / 2.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.1, lower=-1.20, upper=1.20),
    )
    tilt.meta["qc_samples"] = [-1.20, 0.0, 1.20]

    model.articulation(
        "fan_head_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=rotor,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.15, velocity=60.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    head = object_model.get_part("fan_head")
    rotor = object_model.get_part("rotor")
    tilt = object_model.get_articulation("base_to_fan_head")
    spin = object_model.get_articulation("fan_head_to_rotor")

    # The side trunnions are intentionally captured in the yoke sockets.  Keep
    # the allowance local to the named bearing interfaces and prove centering.
    for i in (0, 1):
        ctx.allow_overlap(
            base,
            head,
            elem_a=f"pivot_socket_{i}",
            elem_b=f"pivot_pin_{i}",
            reason="The tilt trunnion is intentionally seated inside the yoke socket bearing.",
        )
        ctx.expect_within(
            head,
            base,
            axes="yz",
            inner_elem=f"pivot_pin_{i}",
            outer_elem=f"pivot_socket_{i}",
            margin=0.004,
            name=f"pivot pin {i} is coaxially captured",
        )
        ctx.expect_overlap(
            head,
            base,
            axes="x",
            elem_a=f"pivot_pin_{i}",
            elem_b=f"pivot_socket_{i}",
            min_overlap=0.018,
            name=f"pivot pin {i} has retained insertion",
        )
        ctx.allow_overlap(
            base,
            head,
            elem_a=f"friction_knob_{i}",
            elem_b=f"pivot_pin_{i}",
            reason="The side friction knob is seated over the end of the tilt pin as a captured fastener.",
        )
        ctx.expect_within(
            head,
            base,
            axes="yz",
            inner_elem=f"pivot_pin_{i}",
            outer_elem=f"friction_knob_{i}",
            margin=0.002,
            name=f"pivot pin {i} is centered in friction knob",
        )
        ctx.expect_overlap(
            head,
            base,
            axes="x",
            elem_a=f"pivot_pin_{i}",
            elem_b=f"friction_knob_{i}",
            min_overlap=0.004,
            name=f"friction knob {i} traps pin end",
        )

    ctx.allow_overlap(
        head,
        rotor,
        elem_a="motor_shaft",
        elem_b="rotor_blades",
        reason="The rotor hub is intentionally captured on the central motor shaft.",
    )
    ctx.expect_within(
        head,
        rotor,
        axes="xz",
        inner_elem="motor_shaft",
        outer_elem="rotor_blades",
        margin=0.002,
        name="shaft runs through rotor hub",
    )
    ctx.expect_overlap(
        head,
        rotor,
        axes="y",
        elem_a="motor_shaft",
        elem_b="rotor_blades",
        min_overlap=0.010,
        name="rotor hub remains on shaft",
    )

    ctx.expect_within(
        rotor,
        head,
        axes="xz",
        inner_elem="rotor_blades",
        outer_elem="front_outer_ring",
        margin=0.006,
        name="rotor disk stays inside guard diameter",
    )
    ctx.expect_gap(
        rotor,
        head,
        axis="y",
        positive_elem="rotor_blades",
        negative_elem="front_outer_ring",
        min_gap=0.012,
        name="front grille clears rotor sweep",
    )
    ctx.expect_gap(
        head,
        rotor,
        axis="y",
        positive_elem="rear_outer_ring",
        negative_elem="rotor_blades",
        min_gap=0.012,
        name="rear grille clears rotor sweep",
    )

    rest_spin_axis = spin.axis == (0.0, 0.0, 1.0)
    ctx.check("rotor spin uses explicit shaft axis", rest_spin_axis, details=f"axis={spin.axis!r}")

    with ctx.pose({tilt: -1.15}):
        ctx.expect_gap(
            head,
            base,
            axis="z",
            positive_elem="motor_can",
            negative_elem="oval_base",
            min_gap=0.040,
            name="stowed motor clears compact base",
        )
        stowed_aabb = ctx.part_world_aabb(head)
    with ctx.pose({tilt: 1.15}):
        raised_aabb = ctx.part_world_aabb(head)
    ctx.check(
        "tilt range lowers head for compact stow",
        stowed_aabb is not None
        and raised_aabb is not None
        and stowed_aabb[1][2] < raised_aabb[1][2] - 0.040,
        details=f"stowed={stowed_aabb}, raised={raised_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
