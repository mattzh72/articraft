from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


STEEL = Material("galvanized_steel", color=(0.46, 0.48, 0.48, 1.0))
YELLOW = Material("safety_yellow", color=(1.0, 0.78, 0.08, 1.0))
RUBBER = Material("black_rubber", color=(0.02, 0.02, 0.018, 1.0))


def _cylinder_between(part, p0, p1, radius: float, material, name: str) -> None:
    """Add a cylinder whose local +Z axis runs from p0 to p1."""
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 1e-9:
        return
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    center = (
        (p0[0] + p1[0]) * 0.5,
        (p0[1] + p1[1]) * 0.5,
        (p0[2] + p1[2]) * 0.5,
    )
    part.visual(
        Cylinder(radius, length),
        origin=Origin(xyz=center, rpy=(0.0, pitch, yaw)),
        material=material,
        name=name,
    )


def _hoop_arc_points(z: float, *, segments: int = 44) -> list[tuple[float, float, float]]:
    """C-shaped cage hoop in plan view, open at the ladder front."""
    center_y = -0.35
    radius = 0.55
    start = math.radians(147.0)
    stop = math.radians(393.0)
    return [
        (
            radius * math.cos(start + (stop - start) * i / segments),
            center_y + radius * math.sin(start + (stop - start) * i / segments),
            z,
        )
        for i in range(segments + 1)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="safety_cage_vertical_access_ladder")

    base = model.part("ladder_base")

    # Fixed ladder section: two round rail stiles, welded rungs, foot plates,
    # guide shoes for the fly section, and the safety cage.
    rail_x = 0.25
    base_height = 3.0
    rail_radius = 0.025
    rung_radius = 0.018
    for side, x in enumerate((-rail_x, rail_x)):
        _cylinder_between(
            base,
            (x, 0.0, 0.0),
            (x, 0.0, base_height),
            rail_radius,
            STEEL,
            f"base_stile_{side}",
        )
        base.visual(
            Box((0.14, 0.12, 0.018)),
            origin=Origin(xyz=(x, 0.0, 0.009)),
            material=RUBBER,
            name=f"foot_plate_{side}",
        )

    z = 0.25
    rung_index = 0
    while z < base_height - 0.12:
        _cylinder_between(
            base,
            (-rail_x, 0.0, z),
            (rail_x, 0.0, z),
            rung_radius,
            STEEL,
            f"base_rung_{rung_index}",
        )
        z += 0.30
        rung_index += 1

    # Non-moving guide shoes and small welded bridges show how the fly section
    # is retained while sliding upward on a vertical prismatic joint.
    for level, z in enumerate((0.95, 2.35)):
        for side, x in enumerate((-0.18, 0.18)):
            base.visual(
                Box((0.080, 0.030, 0.140)),
                origin=Origin(xyz=(x, 0.068, z)),
                material=STEEL,
                name=f"guide_shoe_{level}_{side}",
            )
            _cylinder_between(
                base,
                ((-rail_x if x < 0 else rail_x), 0.0, z),
                (x, 0.068, z),
                0.012,
                STEEL,
                f"guide_bridge_{level}_{side}",
            )

    hoop_zs = (0.95, 1.38, 1.81, 2.24, 2.67)
    for index, hoop_z in enumerate(hoop_zs):
        hoop = tube_from_spline_points(
            _hoop_arc_points(hoop_z),
            radius=0.018,
            samples_per_segment=2,
            radial_segments=16,
            cap_ends=True,
        )
        base.visual(
            mesh_from_geometry(hoop, f"safety_hoop_{index}"),
            material=YELLOW,
            name=f"safety_hoop_{index}",
        )

        left_end = _hoop_arc_points(hoop_z, segments=1)[0]
        right_end = _hoop_arc_points(hoop_z, segments=1)[1]
        _cylinder_between(
            base,
            (-rail_x, 0.0, hoop_z),
            left_end,
            0.014,
            YELLOW,
            f"standoff_{index}_0",
        )
        _cylinder_between(
            base,
            (rail_x, 0.0, hoop_z),
            right_end,
            0.014,
            YELLOW,
            f"standoff_{index}_1",
        )

    # Longitudinal straps tie the hoops together like a real cage, rather than
    # leaving separate rings floating in space.
    for strap_index, angle_deg in enumerate((205.0, 270.0, 335.0)):
        theta = math.radians(angle_deg)
        x = 0.55 * math.cos(theta)
        y = -0.35 + 0.55 * math.sin(theta)
        _cylinder_between(
            base,
            (x, y, hoop_zs[0]),
            (x, y, hoop_zs[-1]),
            0.012,
            YELLOW,
            f"cage_strap_{strap_index}",
        )

    fly = model.part("fly_section")
    fly_rail_x = 0.18
    fly_y = 0.105
    fly_bottom = -0.80
    fly_top = 1.90
    for side, x in enumerate((-fly_rail_x, fly_rail_x)):
        _cylinder_between(
            fly,
            (x, fly_y, fly_bottom),
            (x, fly_y, fly_top),
            0.022,
            STEEL,
            f"fly_stile_{side}",
        )

    z = -0.55
    rung_index = 0
    while z < fly_top - 0.10:
        _cylinder_between(
            fly,
            (-fly_rail_x, fly_y, z),
            (fly_rail_x, fly_y, z),
            0.016,
            STEEL,
            f"fly_rung_{rung_index}",
        )
        z += 0.30
        rung_index += 1

    # Small stop bar and colored upper caps make the moving fly section read as
    # a distinct sliding ladder segment.
    _cylinder_between(
        fly,
        (-fly_rail_x, fly_y, fly_top),
        (fly_rail_x, fly_y, fly_top),
        0.018,
        YELLOW,
        "fly_stop_bar",
    )
    for side, x in enumerate((-fly_rail_x, fly_rail_x)):
        fly.visual(
            Cylinder(0.026, 0.035),
            origin=Origin(xyz=(x, fly_y, fly_top + 0.0175)),
            material=YELLOW,
            name=f"fly_cap_{side}",
        )

    model.articulation(
        "fly_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=fly,
        origin=Origin(xyz=(0.0, 0.0, 1.10)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.25, lower=0.0, upper=1.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("ladder_base")
    fly = object_model.get_part("fly_section")
    slide = object_model.get_articulation("fly_slide")

    ctx.check(
        "fly slide is vertical prismatic",
        slide.articulation_type == ArticulationType.PRISMATIC and slide.axis == (0.0, 0.0, 1.0),
        details=f"type={slide.articulation_type}, axis={slide.axis}",
    )

    with ctx.pose({slide: 0.0}):
        ctx.expect_overlap(
            fly,
            base,
            axes="z",
            min_overlap=1.6,
            name="collapsed fly section remains engaged with fixed ladder",
        )
        collapsed_pos = ctx.part_world_position(fly)

    with ctx.pose({slide: 1.0}):
        ctx.expect_overlap(
            fly,
            base,
            axes="z",
            min_overlap=1.2,
            name="extended fly section retains guide overlap",
        )
        extended_pos = ctx.part_world_position(fly)

    ctx.check(
        "fly section extends upward",
        collapsed_pos is not None
        and extended_pos is not None
        and extended_pos[2] > collapsed_pos[2] + 0.95,
        details=f"collapsed={collapsed_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
