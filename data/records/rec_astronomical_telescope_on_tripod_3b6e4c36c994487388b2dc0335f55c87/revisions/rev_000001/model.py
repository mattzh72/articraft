from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    mesh_from_geometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def _cylinder_between(part, start, end, radius, *, material, name):
    """Add a cylinder whose local Z axis runs from start to end."""
    sx, sy, sz = start
    ex, ey, ez = end
    vx, vy, vz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(vx * vx + vy * vy + vz * vz)
    horizontal = math.sqrt(vx * vx + vy * vy)
    yaw = math.atan2(vy, vx)
    pitch = math.atan2(horizontal, vz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="child_tabletop_refractor")

    blue = model.material("toy_blue", rgba=(0.05, 0.23, 0.78, 1.0))
    yellow = model.material("safety_yellow", rgba=(1.0, 0.75, 0.10, 1.0))
    red = model.material("soft_red", rgba=(0.92, 0.12, 0.10, 1.0))
    black = model.material("matte_black", rgba=(0.015, 0.015, 0.018, 1.0))
    dark = model.material("dark_plastic", rgba=(0.05, 0.055, 0.06, 1.0))
    rubber = model.material("rubber_feet", rgba=(0.025, 0.025, 0.025, 1.0))
    glass = model.material("pale_lens", rgba=(0.55, 0.85, 1.0, 0.48))

    tripod = model.part("tripod")

    # Tabletop tripod: short central post, three splayed plastic legs, and a
    # broad fixed half of the azimuth bearing at adult-hand height for a child.
    tripod.visual(
        Cylinder(radius=0.018, length=0.58),
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
        material=dark,
        name="center_post",
    )
    tripod.visual(
        Cylinder(radius=0.041, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.465)),
        material=yellow,
        name="leg_collar",
    )
    tripod.visual(
        Cylinder(radius=0.060, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.600)),
        material=dark,
        name="azimuth_bearing_base",
    )
    tripod.visual(
        Box((0.135, 0.135, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.515), rpy=(0.0, 0.0, math.radians(45.0))),
        material=yellow,
        name="spread_limit_plate",
    )

    leg_top_radius = 0.032
    foot_radius = 0.285
    for i in range(3):
        angle = math.radians(90.0 + i * 120.0)
        start = (leg_top_radius * math.cos(angle), leg_top_radius * math.sin(angle), 0.458)
        end = (foot_radius * math.cos(angle), foot_radius * math.sin(angle), 0.035)
        _cylinder_between(
            tripod,
            start,
            end,
            0.012,
            material=blue,
            name=f"leg_{i}",
        )
        tripod.visual(
            Sphere(radius=0.025),
            origin=Origin(xyz=end),
            material=rubber,
            name=f"foot_{i}",
        )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.055, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=yellow,
        name="azimuth_bearing_cap",
    )
    head.visual(
        Cylinder(radius=0.030, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=yellow,
        name="head_pedestal",
    )
    head.visual(
        Box((0.110, 0.128, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=yellow,
        name="fork_bridge",
    )
    for i, y in enumerate((-0.058, 0.058)):
        head.visual(
            Box((0.092, 0.014, 0.160)),
            origin=Origin(xyz=(0.0, y, 0.160)),
            material=yellow,
            name=f"fork_cheek_{i}",
        )
        head.visual(
            Cylinder(radius=0.020, length=0.004),
            origin=Origin(xyz=(0.0, y * 0.98, 0.150), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=red,
            name=f"pivot_washer_{i}",
        )

    tube = model.part("tube")
    main_tube_shell = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(0.034, -0.220), (0.034, 0.220)],
            [(0.030, -0.214), (0.030, 0.214)],
            segments=64,
            start_cap="flat",
            end_cap="flat",
        ),
        "main_tube_shell",
    )
    dew_shield_shell = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(0.042, -0.045), (0.042, 0.045)],
            [(0.033, -0.040), (0.035, 0.040)],
            segments=64,
            start_cap="flat",
            end_cap="flat",
        ),
        "dew_shield_shell",
    )
    tube.visual(
        main_tube_shell,
        origin=Origin(xyz=(0.070, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blue,
        name="main_tube",
    )
    tube.visual(
        dew_shield_shell,
        origin=Origin(xyz=(0.325, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=red,
        name="dew_shield",
    )
    tube.visual(
        Cylinder(radius=0.036, length=0.008),
        origin=Origin(xyz=(0.367, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="objective_lens",
    )
    tube.visual(
        Cylinder(radius=0.035, length=0.026),
        origin=Origin(xyz=(-0.154, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="focuser_collar",
    )
    tube.visual(
        Cylinder(radius=0.022, length=0.080),
        origin=Origin(xyz=(-0.190, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="focuser_drawtube",
    )
    _cylinder_between(
        tube,
        (-0.220, 0.0, 0.010),
        (-0.260, 0.0, 0.060),
        0.016,
        material=black,
        name="diagonal_body",
    )
    _cylinder_between(
        tube,
        (-0.257, 0.0, 0.056),
        (-0.292, 0.0, 0.096),
        0.013,
        material=black,
        name="eyepiece",
    )
    tube.visual(
        Cylinder(radius=0.014, length=0.102),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="trunnion_pin",
    )
    tube.visual(
        Cylinder(radius=0.015, length=0.255),
        origin=Origin(xyz=(0.065, 0.0, 0.067), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=yellow,
        name="finder_scope",
    )
    for x in (-0.040, 0.150):
        tube.visual(
            Box((0.020, 0.018, 0.034)),
            origin=Origin(xyz=(x, 0.0, 0.049)),
            material=yellow,
            name=f"finder_bracket_{0 if x < 0 else 1}",
        )

    model.articulation(
        "azimuth",
        ArticulationType.CONTINUOUS,
        parent=tripod,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.620)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=1.0),
    )
    model.articulation(
        "altitude",
        ArticulationType.REVOLUTE,
        parent=head,
        child=tube,
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.2, lower=-0.35, upper=0.90),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tripod = object_model.get_part("tripod")
    head = object_model.get_part("head")
    tube = object_model.get_part("tube")
    azimuth = object_model.get_articulation("azimuth")
    altitude = object_model.get_articulation("altitude")

    ctx.expect_gap(
        head,
        tripod,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="azimuth bearing cap sits on tripod base",
    )
    ctx.expect_contact(
        tube,
        head,
        elem_a="trunnion_pin",
        elem_b="fork_cheek_0",
        contact_tol=0.001,
        name="altitude trunnion is captured by one fork cheek",
    )
    ctx.expect_contact(
        tube,
        head,
        elem_a="trunnion_pin",
        elem_b="fork_cheek_1",
        contact_tol=0.001,
        name="altitude trunnion is captured by opposite fork cheek",
    )

    def elem_center_z(part, elem):
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    def elem_center_xy(part, elem):
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        return (0.5 * (aabb[0][0] + aabb[1][0]), 0.5 * (aabb[0][1] + aabb[1][1]))

    rest_front_z = elem_center_z(tube, "objective_lens")
    with ctx.pose({altitude: 0.75}):
        raised_front_z = elem_center_z(tube, "objective_lens")
    ctx.check(
        "positive altitude tilt raises objective",
        rest_front_z is not None and raised_front_z is not None and raised_front_z > rest_front_z + 0.20,
        details=f"rest_z={rest_front_z}, raised_z={raised_front_z}",
    )

    rest_front_xy = elem_center_xy(tube, "objective_lens")
    with ctx.pose({azimuth: math.pi / 2.0}):
        turned_front_xy = elem_center_xy(tube, "objective_lens")
    ctx.check(
        "azimuth bearing turns telescope around vertical axis",
        rest_front_xy is not None
        and turned_front_xy is not None
        and abs(turned_front_xy[1]) > abs(rest_front_xy[1]) + 0.25
        and abs(turned_front_xy[0]) < abs(rest_front_xy[0]) - 0.20,
        details=f"rest_xy={rest_front_xy}, turned_xy={turned_front_xy}",
    )

    return ctx.report()


object_model = build_object_model()
