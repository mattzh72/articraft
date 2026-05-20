from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _cylinder_origin_between(
    start: tuple[float, float, float], end: tuple[float, float, float]
) -> tuple[Origin, float]:
    """Return an Origin and length for a cylinder whose local +Z spans start->end."""
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    return (
        Origin(
            xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        length,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_horizontal_axis_wind_turbine")

    painted_steel = Material("painted_steel", rgba=(0.18, 0.20, 0.22, 1.0))
    black_rubber = Material("black_rubber", rgba=(0.025, 0.025, 0.025, 1.0))
    bearing_metal = Material("bearing_metal", rgba=(0.55, 0.57, 0.58, 1.0))
    nacelle_white = Material("nacelle_white", rgba=(0.88, 0.90, 0.88, 1.0))
    blade_white = Material("blade_white", rgba=(0.94, 0.95, 0.92, 1.0))
    tail_yellow = Material("tail_yellow", rgba=(0.95, 0.72, 0.14, 1.0))

    tower = model.part("tower")
    tower.visual(
        Box((0.95, 0.65, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=black_rubber,
        name="wide_base",
    )
    tower.visual(
        Cylinder(radius=0.055, length=0.36),
        origin=Origin(xyz=(0.0, 0.0, 0.22)),
        material=painted_steel,
        name="short_mast",
    )
    tower.visual(
        Cylinder(radius=0.108, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.410)),
        material=bearing_metal,
        name="yaw_lower_bearing",
    )

    brace_feet = (
        (0.35, 0.22, 0.045),
        (0.35, -0.22, 0.045),
        (-0.35, 0.22, 0.045),
        (-0.35, -0.22, 0.045),
    )
    for index, foot in enumerate(brace_feet):
        origin, length = _cylinder_origin_between(foot, (0.0, 0.0, 0.335))
        tower.visual(
            Cylinder(radius=0.014, length=length),
            origin=origin,
            material=painted_steel,
            name=f"brace_{index}",
        )

    nacelle = model.part("nacelle")
    nacelle_body = CapsuleGeometry(radius=0.075, length=0.28)
    nacelle.visual(
        mesh_from_geometry(nacelle_body, "nacelle_body"),
        origin=Origin(xyz=(0.04, 0.0, 0.085), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=nacelle_white,
        name="nacelle_body",
    )
    nacelle.visual(
        Cylinder(radius=0.060, length=0.060),
        origin=Origin(xyz=(0.285, 0.0, 0.085), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bearing_metal,
        name="bearing_collar",
    )
    nacelle.visual(
        Cylinder(radius=0.102, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=bearing_metal,
        name="yaw_upper_bearing",
    )
    nacelle.visual(
        Cylinder(radius=0.012, length=0.34),
        origin=Origin(xyz=(-0.34, 0.0, 0.085), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=painted_steel,
        name="tail_boom",
    )
    nacelle.visual(
        Box((0.070, 0.008, 0.170)),
        origin=Origin(xyz=(-0.515, 0.0, 0.120)),
        material=tail_yellow,
        name="tail_vane",
    )

    rotor = model.part("rotor")
    rotor_mesh = FanRotorGeometry(
        outer_radius=0.260,
        hub_radius=0.055,
        blade_count=3,
        thickness=0.028,
        blade_pitch_deg=18.0,
        blade_sweep_deg=8.0,
        blade_root_chord=0.060,
        blade_tip_chord=0.020,
        blade=FanRotorBlade(shape="narrow", tip_pitch_deg=4.0, camber=0.07),
        hub=FanRotorHub(
            style="spinner",
            rear_collar_height=0.020,
            rear_collar_radius=0.040,
            bore_diameter=0.018,
        ),
    )
    rotor.visual(
        mesh_from_geometry(rotor_mesh, "three_blade_rotor"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blade_white,
        name="three_blade_rotor",
    )
    rotor.visual(
        Cylinder(radius=0.024, length=0.100),
        origin=Origin(xyz=(-0.030, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=painted_steel,
        name="shaft",
    )

    model.articulation(
        "tower_to_nacelle",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=nacelle,
        origin=Origin(xyz=(0.0, 0.0, 0.420)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.6),
    )
    model.articulation(
        "nacelle_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=rotor,
        origin=Origin(xyz=(0.360, 0.0, 0.085)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    nacelle = object_model.get_part("nacelle")
    rotor = object_model.get_part("rotor")
    yaw = object_model.get_articulation("tower_to_nacelle")
    spin = object_model.get_articulation("nacelle_to_rotor")

    ctx.allow_overlap(
        nacelle,
        rotor,
        elem_a="bearing_collar",
        elem_b="shaft",
        reason="The rotating main shaft is intentionally captured inside the stationary nose bearing collar.",
    )
    ctx.expect_within(
        rotor,
        nacelle,
        axes="yz",
        inner_elem="shaft",
        outer_elem="bearing_collar",
        margin=0.0,
        name="shaft is centered inside bearing collar",
    )
    ctx.expect_overlap(
        rotor,
        nacelle,
        axes="x",
        elem_a="shaft",
        elem_b="bearing_collar",
        min_overlap=0.025,
        name="shaft remains inserted in bearing collar",
    )

    ctx.check(
        "nacelle yaw is continuous about vertical tower axis",
        yaw.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 6) for v in yaw.axis) == (0.0, 0.0, 1.0),
        details=f"type={yaw.articulation_type}, axis={yaw.axis}",
    )
    ctx.check(
        "rotor spins continuously about horizontal shaft",
        spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 6) for v in spin.axis) == (1.0, 0.0, 0.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )

    tower_aabb = ctx.part_world_aabb(tower)
    rotor_pos = ctx.part_world_position(rotor)
    if tower_aabb is not None and rotor_pos is not None:
        base_width = tower_aabb[1][0] - tower_aabb[0][0]
        ctx.check(
            "rotating mass is kept low over a wide base",
            rotor_pos[2] < 0.60 * base_width,
            details=f"hub_height={rotor_pos[2]:.3f}, base_width={base_width:.3f}",
        )
    else:
        ctx.fail("rotating mass is kept low over a wide base", "missing pose or base bounds")

    rest_pos = ctx.part_world_position(rotor)
    with ctx.pose({yaw: math.pi / 2.0}):
        yawed_pos = ctx.part_world_position(rotor)
    ctx.check(
        "yaw sweeps the nacelle around the tower",
        rest_pos is not None
        and yawed_pos is not None
        and yawed_pos[1] > rest_pos[1] + 0.25
        and abs(yawed_pos[2] - rest_pos[2]) < 0.002,
        details=f"rest={rest_pos}, yawed={yawed_pos}",
    )

    with ctx.pose({spin: math.pi / 3.0}):
        spun_pos = ctx.part_world_position(rotor)
    ctx.check(
        "rotor spin keeps hub on the main shaft",
        rest_pos is not None
        and spun_pos is not None
        and abs(spun_pos[0] - rest_pos[0]) < 0.001
        and abs(spun_pos[1] - rest_pos[1]) < 0.001
        and abs(spun_pos[2] - rest_pos[2]) < 0.001,
        details=f"rest={rest_pos}, spun={spun_pos}",
    )

    return ctx.report()


object_model = build_object_model()
