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
)


def _cylinder_between(part, p0, p1, radius: float, *, material: Material, name: str) -> None:
    """Add a cylinder whose local +Z axis is aligned between two model-space points."""
    x0, y0, z0 = p0
    x1, y1, z1 = p1
    dx, dy, dz = x1 - x0, y1 - y0, z1 - z0
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError(f"zero-length cylinder {name}")
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((x0 + x1) * 0.5, (y0 + y1) * 0.5, (z0 + z1) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_service_traditional_windmill")

    galvanized = model.material("galvanized_steel", rgba=(0.58, 0.62, 0.60, 1.0))
    dark_steel = model.material("dark_worn_steel", rgba=(0.12, 0.13, 0.13, 1.0))
    red_oxide = model.material("red_oxide_primer", rgba=(0.55, 0.12, 0.08, 1.0))
    bearing_bronze = model.material("oiled_bronze_bearing", rgba=(0.68, 0.43, 0.16, 1.0))
    service_yellow = model.material("service_yellow", rgba=(0.96, 0.72, 0.14, 1.0))
    weathered_tail = model.material("weathered_tail_paint", rgba=(0.92, 0.90, 0.80, 1.0))

    # Root field tower: broad stance, welded diagonals, top deck, and a turntable seat.
    tower = model.part("tower")
    base_half = 1.25
    top_half = 0.34
    tower_height = 7.05

    def leg_point(sign_x: int, sign_y: int, z: float) -> tuple[float, float, float]:
        t = z / tower_height
        half = base_half + (top_half - base_half) * t
        return (sign_x * half, sign_y * half, z)

    for sx in (-1, 1):
        for sy in (-1, 1):
            tower.visual(
                Box((0.46, 0.46, 0.07)),
                origin=Origin(xyz=(sx * base_half, sy * base_half, 0.035)),
                material=dark_steel,
                name=f"foot_pad_{sx}_{sy}",
            )
            _cylinder_between(
                tower,
                (sx * base_half, sy * base_half, 0.055),
                leg_point(sx, sy, 6.95),
                0.065,
                material=galvanized,
                name=f"tower_leg_{sx}_{sy}",
            )

    levels = [1.55, 3.05, 4.60, 6.10]
    for i, z0 in enumerate(levels):
        z1 = min(z0 + 1.15, 6.85)
        corners0 = {
            (sx, sy): leg_point(sx, sy, z0)
            for sx in (-1, 1)
            for sy in (-1, 1)
        }
        corners1 = {
            (sx, sy): leg_point(sx, sy, z1)
            for sx in (-1, 1)
            for sy in (-1, 1)
        }
        # perimeter rails at each service level
        for sy in (-1, 1):
            _cylinder_between(
                tower, corners0[(-1, sy)], corners0[(1, sy)], 0.025,
                material=dark_steel, name=f"face_rail_x_{i}_{sy}",
            )
        for sx in (-1, 1):
            _cylinder_between(
                tower, corners0[(sx, -1)], corners0[(sx, 1)], 0.025,
                material=dark_steel, name=f"face_rail_y_{i}_{sx}",
            )
        # X bracing on all four faces; the ends bury into the legs like welded lugs.
        for sy in (-1, 1):
            _cylinder_between(
                tower, corners0[(-1, sy)], corners1[(1, sy)], 0.022,
                material=galvanized, name=f"x_brace_front_a_{i}_{sy}",
            )
            _cylinder_between(
                tower, corners0[(1, sy)], corners1[(-1, sy)], 0.022,
                material=galvanized, name=f"x_brace_front_b_{i}_{sy}",
            )
        for sx in (-1, 1):
            _cylinder_between(
                tower, corners0[(sx, -1)], corners1[(sx, 1)], 0.022,
                material=galvanized, name=f"x_brace_side_a_{i}_{sx}",
            )
            _cylinder_between(
                tower, corners0[(sx, 1)], corners1[(sx, -1)], 0.022,
                material=galvanized, name=f"x_brace_side_b_{i}_{sx}",
            )

    tower.visual(
        Box((1.02, 1.02, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 6.88)),
        material=dark_steel,
        name="service_deck",
    )
    tower.visual(
        Cylinder(radius=0.46, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 6.99)),
        material=bearing_bronze,
        name="tower_turntable",
    )

    # Yawing cap/nacelle with an exposed bearing sleeve, service deck, and tail boom.
    cap = model.part("cap")
    cap.visual(
        Cylinder(radius=0.38, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=bearing_bronze,
        name="yaw_bearing_plate",
    )
    cap.visual(
        Cylinder(radius=0.16, length=0.52),
        origin=Origin(xyz=(0.0, 0.0, 0.26)),
        material=dark_steel,
        name="yaw_stem",
    )
    # The gearbox case is split around the shaft opening so the visible rotor
    # shaft only passes through the bronze bearing seats, not a solid box proxy.
    cap.visual(
        Box((0.194, 0.90, 0.48)),
        origin=Origin(xyz=(0.275, -0.30, 0.57)),
        material=red_oxide,
        name="gearbox",
    )
    cap.visual(
        Box((0.17, 0.90, 0.48)),
        origin=Origin(xyz=(-0.275, -0.30, 0.57)),
        material=red_oxide,
        name="gearbox_cheek",
    )
    cap.visual(
        Box((0.72, 0.90, 0.11)),
        origin=Origin(xyz=(0.0, -0.30, 0.385)),
        material=red_oxide,
        name="gearbox_lower_case",
    )
    cap.visual(
        Box((0.72, 0.90, 0.11)),
        origin=Origin(xyz=(0.0, -0.30, 0.755)),
        material=red_oxide,
        name="gearbox_upper_case",
    )
    cap.visual(
        Box((0.80, 0.98, 0.08)),
        origin=Origin(xyz=(0.0, -0.30, 0.85)),
        material=galvanized,
        name="cap_roof",
    )
    _cylinder_between(
        cap,
        (0.0, -1.08, 0.60),
        (0.0, -0.54, 0.60),
        0.21,
        material=bearing_bronze,
        name="main_bearing_sleeve",
    )
    _cylinder_between(
        cap,
        (0.0, -1.16, 0.60),
        (0.0, -1.04, 0.60),
        0.30,
        material=dark_steel,
        name="front_bearing_flange",
    )
    cap.visual(
        Box((0.92, 0.38, 0.06)),
        origin=Origin(xyz=(0.0, 0.27, 0.36)),
        material=dark_steel,
        name="maintenance_deck",
    )
    cap.visual(
        Box((0.24, 0.12, 0.24)),
        origin=Origin(xyz=(0.0, 0.12, 0.62)),
        material=dark_steel,
        name="tail_boom_socket",
    )
    for x in (-0.40, 0.40):
        _cylinder_between(
            cap, (x, 0.08, 0.37), (x, 0.45, 0.37), 0.020,
            material=service_yellow, name=f"deck_side_rail_{x}",
        )
        _cylinder_between(
            cap, (x, 0.08, 0.37), (x, 0.08, 0.72), 0.018,
            material=service_yellow, name=f"deck_post_front_{x}",
        )
        _cylinder_between(
            cap, (x, 0.45, 0.37), (x, 0.45, 0.72), 0.018,
            material=service_yellow, name=f"deck_post_rear_{x}",
        )
    _cylinder_between(
        cap, (-0.40, 0.45, 0.72), (0.40, 0.45, 0.72), 0.020,
        material=service_yellow, name="deck_rear_handrail",
    )
    _cylinder_between(
        cap, (0.0, 0.12, 0.62), (0.0, 1.96, 0.80), 0.045,
        material=galvanized, name="tail_boom",
    )
    _cylinder_between(
        cap, (-0.28, 0.05, 0.48), (0.0, 1.78, 0.58), 0.026,
        material=dark_steel, name="tail_brace_0",
    )
    _cylinder_between(
        cap, (0.28, 0.05, 0.48), (0.0, 1.78, 0.58), 0.026,
        material=dark_steel, name="tail_brace_1",
    )
    cap.visual(
        Box((0.72, 0.06, 0.62)),
        origin=Origin(xyz=(0.0, 1.96, 0.80)),
        material=weathered_tail,
        name="tail_vane",
    )
    _cylinder_between(
        cap, (0.425, -0.56, 0.40), (0.425, -0.56, 0.80), 0.026,
        material=dark_steel,
        name="hatch_pin",
    )
    cap.visual(
        Box((0.12, 0.12, 0.05)),
        origin=Origin(xyz=(0.39, -0.56, 0.385)),
        material=dark_steel,
        name="lower_hinge_lug",
    )
    cap.visual(
        Box((0.12, 0.12, 0.05)),
        origin=Origin(xyz=(0.39, -0.56, 0.815)),
        material=dark_steel,
        name="upper_hinge_lug",
    )

    # Hinged service hatch; it is a separate wear/access part on real hinges.
    side_hatch = model.part("side_hatch")
    side_hatch.visual(
        Box((0.040, 0.41, 0.32)),
        origin=Origin(xyz=(-0.025, 0.245, 0.18)),
        material=service_yellow,
        name="hatch_panel",
    )
    side_hatch.visual(
        Box((0.022, 0.070, 0.26)),
        origin=Origin(xyz=(-0.040, 0.035, 0.18)),
        material=dark_steel,
        name="hinge_leaf",
    )
    side_hatch.visual(
        Cylinder(radius=0.034, length=0.38),
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        material=dark_steel,
        name="hinge_barrel",
    )
    side_hatch.visual(
        Box((0.020, 0.10, 0.08)),
        origin=Origin(xyz=(0.000, 0.34, 0.18)),
        material=dark_steel,
        name="pull_lug",
    )

    # Rotor assembly: a captured shaft, chunky hub, circular rim, spokes, louvered blades.
    rotor = model.part("rotor")
    _cylinder_between(
        rotor,
        (0.0, -0.32, 0.0),
        (0.0, 0.42, 0.0),
        0.085,
        material=dark_steel,
        name="rotor_shaft",
    )
    _cylinder_between(
        rotor,
        (0.0, -0.60, 0.0),
        (0.0, -0.30, 0.0),
        0.20,
        material=bearing_bronze,
        name="rotor_hub",
    )
    _cylinder_between(
        rotor,
        (0.0, -0.75, 0.0),
        (0.0, -0.58, 0.0),
        0.13,
        material=dark_steel,
        name="front_locknut",
    )
    blade_count = 12
    rim_radius = 1.55
    inner_ring = 0.78
    for i in range(blade_count):
        theta = 2.0 * math.pi * i / blade_count
        next_theta = 2.0 * math.pi * (i + 1) / blade_count
        radial = (math.cos(theta), math.sin(theta))
        next_radial = (math.cos(next_theta), math.sin(next_theta))

        _cylinder_between(
            rotor,
            (0.12 * radial[0], -0.36, 0.12 * radial[1]),
            (rim_radius * radial[0], -0.36, rim_radius * radial[1]),
            0.026,
            material=galvanized,
            name=f"blade_spoke_{i}",
        )
        _cylinder_between(
            rotor,
            (rim_radius * radial[0], -0.36, rim_radius * radial[1]),
            (rim_radius * next_radial[0], -0.36, rim_radius * next_radial[1]),
            0.026,
            material=dark_steel,
            name=f"rim_segment_{i}",
        )
        _cylinder_between(
            rotor,
            (inner_ring * radial[0], -0.36, inner_ring * radial[1]),
            (inner_ring * next_radial[0], -0.36, inner_ring * next_radial[1]),
            0.018,
            material=dark_steel,
            name=f"inner_lattice_{i}",
        )
        rotor.visual(
            Box((0.70, 0.036, 0.20)),
            origin=Origin(
                xyz=(1.10 * radial[0], -0.40, 1.10 * radial[1]),
                rpy=(0.0, -theta, 0.0),
            ),
            material=galvanized,
            name=f"blade_panel_{i}",
        )
        rotor.visual(
            Box((0.18, 0.050, 0.26)),
            origin=Origin(
                xyz=(rim_radius * radial[0], -0.40, rim_radius * radial[1]),
                rpy=(0.0, -theta, 0.0),
            ),
            material=red_oxide,
            name=f"tip_wear_pad_{i}",
        )

    model.articulation(
        "tower_to_cap",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 7.05)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=-2.4, upper=2.4),
    )
    model.articulation(
        "cap_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=cap,
        child=rotor,
        origin=Origin(xyz=(0.0, -0.95, 0.60)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=12.0),
    )
    model.articulation(
        "cap_to_side_hatch",
        ArticulationType.REVOLUTE,
        parent=cap,
        child=side_hatch,
        origin=Origin(xyz=(0.425, -0.56, 0.42)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=0.0, upper=1.25),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    cap = object_model.get_part("cap")
    rotor = object_model.get_part("rotor")
    side_hatch = object_model.get_part("side_hatch")
    cap_yaw = object_model.get_articulation("tower_to_cap")
    rotor_spin = object_model.get_articulation("cap_to_rotor")
    hatch_hinge = object_model.get_articulation("cap_to_side_hatch")

    ctx.allow_overlap(
        cap,
        rotor,
        elem_a="main_bearing_sleeve",
        elem_b="rotor_shaft",
        reason="The rotor shaft is intentionally captured inside the replaceable bearing sleeve.",
    )
    ctx.allow_overlap(
        cap,
        rotor,
        elem_a="front_bearing_flange",
        elem_b="rotor_shaft",
        reason="The front bearing flange is a bolted wear ring around the same captured rotor shaft.",
    )
    ctx.allow_overlap(
        cap,
        side_hatch,
        elem_a="hatch_pin",
        elem_b="hinge_barrel",
        reason="The service hatch barrel rotates around a captured steel hinge pin.",
    )

    ctx.expect_contact(
        tower,
        cap,
        elem_a="tower_turntable",
        elem_b="yaw_bearing_plate",
        contact_tol=0.002,
        name="cap sits on tower bearing seat",
    )
    ctx.expect_within(
        rotor,
        cap,
        axes="xz",
        inner_elem="rotor_shaft",
        outer_elem="main_bearing_sleeve",
        margin=0.002,
        name="rotor shaft centered in bearing sleeve",
    )
    ctx.expect_overlap(
        rotor,
        cap,
        axes="y",
        elem_a="rotor_shaft",
        elem_b="main_bearing_sleeve",
        min_overlap=0.36,
        name="rotor shaft retained through bearing sleeve",
    )
    ctx.expect_overlap(
        rotor,
        cap,
        axes="y",
        elem_a="rotor_shaft",
        elem_b="front_bearing_flange",
        min_overlap=0.08,
        name="front flange captures shaft nose",
    )
    ctx.expect_gap(
        side_hatch,
        cap,
        axis="x",
        positive_elem="hatch_panel",
        negative_elem="gearbox",
        min_gap=0.006,
        max_gap=0.026,
        name="closed hatch is proud of gearbox side",
    )
    ctx.expect_overlap(
        side_hatch,
        cap,
        axes="z",
        elem_a="hinge_barrel",
        elem_b="hatch_pin",
        min_overlap=0.35,
        name="hatch barrel length captures hinge pin",
    )

    with ctx.pose({rotor_spin: math.pi * 0.5, cap_yaw: 0.55}):
        ctx.expect_within(
            rotor,
            cap,
            axes="xz",
            inner_elem="rotor_shaft",
            outer_elem="main_bearing_sleeve",
            margin=0.002,
            name="shaft remains seated after rotor and cap motion",
        )

    closed_aabb = ctx.part_world_aabb(side_hatch)
    with ctx.pose({hatch_hinge: 1.0}):
        open_aabb = ctx.part_world_aabb(side_hatch)
    ctx.check(
        "maintenance hatch opens outward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][0] > closed_aabb[1][0] + 0.10,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    blade_panels = [v for v in rotor.visuals if v.name and v.name.startswith("blade_panel_")]
    ctx.check(
        "rotor has visible blade lattice",
        len(blade_panels) >= 12,
        details=f"blade_panel_count={len(blade_panels)}",
    )

    return ctx.report()


object_model = build_object_model()
