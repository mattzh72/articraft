from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_side_loft,
)


def _circle_profile(
    radius: float,
    *,
    segments: int = 48,
    center: tuple[float, float] = (0.0, 0.0),
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * i / segments),
            cy + radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _chainring_tooth_profile(
    *,
    teeth: int = 48,
    root_radius: float = 0.098,
    tip_radius: float = 0.106,
) -> list[tuple[float, float]]:
    profile: list[tuple[float, float]] = []
    pitch = 2.0 * math.pi / teeth
    for tooth in range(teeth):
        base = tooth * pitch
        for frac, radius in (
            (0.00, root_radius),
            (0.22, tip_radius),
            (0.62, tip_radius),
            (0.84, root_radius),
        ):
            a = base + frac * pitch
            profile.append((radius * math.cos(a), radius * math.sin(a)))
    return profile


def _square_taper_geometry(x0: float, x1: float, half0: float, half1: float) -> MeshGeometry:
    geom = MeshGeometry()
    corners = [
        (x0, -half0, -half0),
        (x0, half0, -half0),
        (x0, half0, half0),
        (x0, -half0, half0),
        (x1, -half1, -half1),
        (x1, half1, -half1),
        (x1, half1, half1),
        (x1, -half1, half1),
    ]
    for x, y, z in corners:
        geom.add_vertex(x, y, z)
    for a, b, c, d in (
        (0, 1, 2, 3),
        (4, 7, 6, 5),
        (0, 4, 5, 1),
        (1, 5, 6, 2),
        (2, 6, 7, 3),
        (3, 7, 4, 0),
    ):
        geom.add_face(a, b, c)
        geom.add_face(a, c, d)
    return geom


def _crank_arm_geometry(name: str) -> object:
    sections = [
        (0.014, -0.014, 0.014, 0.018),
        (0.040, -0.012, 0.012, 0.016),
        (0.095, -0.0095, 0.0095, 0.014),
        (0.150, -0.008, 0.008, 0.012),
        (0.160, -0.010, 0.010, 0.014),
    ]
    return mesh_from_geometry(
        superellipse_side_loft(sections, exponents=2.7, segments=32),
        name,
    )


def _annular_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    thickness: float,
    segments: int = 72,
) -> object:
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(outer_radius, segments=segments),
            [_circle_profile(inner_radius, segments=segments)],
            thickness,
            center=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="track_fixie_crankset")

    polished = model.material("polished_aluminum", rgba=(0.78, 0.76, 0.70, 1.0))
    dark_shell = model.material("dark_anodized_shell", rgba=(0.035, 0.035, 0.035, 1.0))
    black_ring = model.material("black_chainring", rgba=(0.01, 0.012, 0.014, 1.0))
    steel = model.material("brushed_steel", rgba=(0.56, 0.57, 0.54, 1.0))
    pedal_black = model.material("black_pedal_cage", rgba=(0.02, 0.02, 0.018, 1.0))

    shell = model.part("shell")
    shell.visual(
        _annular_mesh(
            "shell_tube",
            outer_radius=0.034,
            inner_radius=0.0185,
            thickness=0.074,
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_shell,
        name="shell_tube",
    )
    for side, x in (("right", 0.041), ("left", -0.041)):
        shell.visual(
            _annular_mesh(
                f"{side}_bearing_cup",
                outer_radius=0.038,
                inner_radius=0.018,
                thickness=0.012,
            ),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"{side}_bearing_cup",
        )

    crankset = model.part("crankset")
    crankset.visual(
        Cylinder(radius=0.0115, length=0.190),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="spindle",
    )
    crankset.visual(
        mesh_from_geometry(_square_taper_geometry(0.042, 0.083, 0.0115, 0.0080), "right_square_taper"),
        material=steel,
        name="right_square_taper",
    )
    crankset.visual(
        mesh_from_geometry(_square_taper_geometry(-0.042, -0.083, 0.0115, 0.0080), "left_square_taper"),
        material=steel,
        name="left_square_taper",
    )

    crankset.visual(
        _crank_arm_geometry("right_crank_arm"),
        origin=Origin(xyz=(0.095, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=polished,
        name="right_crank_arm",
    )
    crankset.visual(
        _crank_arm_geometry("left_crank_arm"),
        origin=Origin(xyz=(-0.095, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished,
        name="left_crank_arm",
    )

    for side, x, z_tip in (("right", 0.095, -0.170), ("left", -0.095, 0.170)):
        crankset.visual(
            Cylinder(radius=0.026, length=0.019),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=polished,
            name=f"{side}_hub_boss",
        )
        crankset.visual(
            Cylinder(radius=0.018, length=0.019),
            origin=Origin(xyz=(x, 0.0, z_tip), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=polished,
            name=f"{side}_pedal_eye",
        )
        crankset.visual(
            Cylinder(radius=0.010, length=0.005),
            origin=Origin(xyz=(x + (0.0115 if x > 0 else -0.0115), 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"{side}_crank_bolt",
        )
        crankset.visual(
            Cylinder(radius=0.0042, length=0.006),
            origin=Origin(xyz=(x + (0.0135 if x > 0 else -0.0135), 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_shell,
            name=f"{side}_hex_socket",
        )

    chainring = ExtrudeWithHolesGeometry(
        _chainring_tooth_profile(teeth=48),
        [_circle_profile(0.062, segments=72)]
        + [
            _circle_profile(
                0.0048,
                segments=20,
                center=(0.072 * math.cos(2.0 * math.pi * i / 5.0), 0.072 * math.sin(2.0 * math.pi * i / 5.0)),
            )
            for i in range(5)
        ],
        0.004,
        center=True,
    )
    crankset.visual(
        mesh_from_geometry(chainring, "chainring_48t"),
        origin=Origin(xyz=(0.074, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_ring,
        name="chainring_48t",
    )

    bolt_radius = 0.072
    for i in range(5):
        theta = 2.0 * math.pi * i / 5.0
        y = bolt_radius * math.sin(theta)
        z = -bolt_radius * math.cos(theta)
        radial_mid = 0.050
        crankset.visual(
            Box((0.008, 0.010, 0.060)),
            origin=Origin(
                xyz=(0.087, radial_mid * math.sin(theta), -radial_mid * math.cos(theta)),
                rpy=(math.pi + theta, 0.0, 0.0),
            ),
            material=polished,
            name=f"spider_arm_{i}",
        )
        crankset.visual(
            Cylinder(radius=0.009, length=0.009),
            origin=Origin(xyz=(0.087, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=polished,
            name=f"spider_boss_{i}",
        )
        crankset.visual(
            Cylinder(radius=0.0065, length=0.008),
            origin=Origin(xyz=(0.079, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"chainring_bolt_head_{i}",
        )
        crankset.visual(
            Cylinder(radius=0.0030, length=0.024),
            origin=Origin(xyz=(0.084, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"chainring_bolt_shank_{i}",
        )

    model.articulation(
        "shell_to_crankset",
        ArticulationType.CONTINUOUS,
        parent=shell,
        child=crankset,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=18.0),
    )

    for side, sign, z_tip in (("right", 1.0, -0.170), ("left", -1.0, 0.170)):
        pedal = model.part(f"{side}_pedal")
        pedal.visual(
            Cylinder(radius=0.0045, length=0.090),
            origin=Origin(xyz=(sign * 0.045, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name="pedal_axle",
        )
        for rail_x in (0.030, 0.090):
            pedal.visual(
                Box((0.008, 0.106, 0.010)),
                origin=Origin(xyz=(sign * rail_x, 0.0, 0.0)),
                material=pedal_black,
                name=f"side_rail_{rail_x:.3f}".replace(".", "_"),
            )
        for y in (-0.051, 0.051):
            pedal.visual(
                Box((0.072, 0.007, 0.022)),
                origin=Origin(xyz=(sign * 0.060, y, 0.0)),
                material=pedal_black,
                name=f"end_plate_{'front' if y > 0 else 'rear'}",
            )
            for n, tx in enumerate((0.028, 0.044, 0.060, 0.076, 0.092)):
                pedal.visual(
                    Box((0.007, 0.004, 0.008)),
                    origin=Origin(xyz=(sign * tx, y, 0.014)),
                    material=steel,
                    name=f"tooth_{'front' if y > 0 else 'rear'}_{n}",
                )
        pedal.visual(
            Box((0.026, 0.020, 0.014)),
            origin=Origin(xyz=(sign * 0.018, 0.0, 0.0)),
            material=steel,
            name="threaded_block",
        )

        model.articulation(
            f"crankset_to_{side}_pedal",
            ArticulationType.CONTINUOUS,
            parent=crankset,
            child=pedal,
            origin=Origin(xyz=(sign * 0.1045, 0.0, z_tip)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=25.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shell = object_model.get_part("shell")
    crankset = object_model.get_part("crankset")
    right_pedal = object_model.get_part("right_pedal")
    left_pedal = object_model.get_part("left_pedal")
    axle = object_model.get_articulation("shell_to_crankset")
    right_pedal_joint = object_model.get_articulation("crankset_to_right_pedal")
    left_pedal_joint = object_model.get_articulation("crankset_to_left_pedal")

    for cup_name in ("shell_tube", "right_bearing_cup", "left_bearing_cup"):
        ctx.allow_overlap(
            crankset,
            shell,
            elem_a="spindle",
            elem_b=cup_name,
            reason=(
                "The bottom-bracket spindle is intentionally captured through the "
                "annular shell and bearing-cup bore; the allowance is scoped to "
                "that hidden axle-through-bore interface."
            ),
        )
    for taper_name, cup_name in (
        ("right_square_taper", "right_bearing_cup"),
        ("left_square_taper", "left_bearing_cup"),
    ):
        ctx.allow_overlap(
            crankset,
            shell,
            elem_a=taper_name,
            elem_b=cup_name,
            reason=(
                "The square-taper end is intentionally shown passing through "
                "the bearing-cup bore before the crank is clamped onto it."
            ),
        )

    ctx.check(
        "three continuous spin axes",
        axle.articulation_type == ArticulationType.CONTINUOUS
        and right_pedal_joint.articulation_type == ArticulationType.CONTINUOUS
        and left_pedal_joint.articulation_type == ArticulationType.CONTINUOUS,
        details="The bottom-bracket axle and both rat-trap pedals should spin continuously.",
    )
    ctx.expect_within(
        crankset,
        shell,
        axes="yz",
        inner_elem="spindle",
        outer_elem="shell_tube",
        margin=0.0,
        name="spindle is centered through bottom bracket shell",
    )
    ctx.expect_overlap(
        crankset,
        shell,
        axes="x",
        elem_a="spindle",
        elem_b="shell_tube",
        min_overlap=0.065,
        name="spindle runs through shell length",
    )
    for cup_name in ("right_bearing_cup", "left_bearing_cup"):
        ctx.expect_within(
            crankset,
            shell,
            axes="yz",
            inner_elem="spindle",
            outer_elem=cup_name,
            margin=0.0,
            name=f"spindle is centered through {cup_name}",
        )
        ctx.expect_overlap(
            crankset,
            shell,
            axes="x",
            elem_a="spindle",
            elem_b=cup_name,
            min_overlap=0.010,
            name=f"spindle passes through {cup_name}",
        )
    for taper_name, cup_name in (
        ("right_square_taper", "right_bearing_cup"),
        ("left_square_taper", "left_bearing_cup"),
    ):
        ctx.expect_within(
            crankset,
            shell,
            axes="yz",
            inner_elem=taper_name,
            outer_elem=cup_name,
            margin=0.0,
            name=f"{taper_name} stays inside {cup_name} bore",
        )
        ctx.expect_overlap(
            crankset,
            shell,
            axes="x",
            elem_a=taper_name,
            elem_b=cup_name,
            min_overlap=0.004,
            name=f"{taper_name} is retained in {cup_name}",
        )
    ctx.expect_contact(
        right_pedal,
        crankset,
        elem_a="pedal_axle",
        elem_b="right_pedal_eye",
        contact_tol=0.001,
        name="right pedal axle seats on crank tip",
    )
    ctx.expect_contact(
        left_pedal,
        crankset,
        elem_a="pedal_axle",
        elem_b="left_pedal_eye",
        contact_tol=0.001,
        name="left pedal axle seats on crank tip",
    )
    with ctx.pose({axle: math.pi / 2.0, right_pedal_joint: math.pi / 3.0, left_pedal_joint: -math.pi / 4.0}):
        ctx.expect_origin_distance(
            right_pedal,
            left_pedal,
            axes="yz",
            min_dist=0.30,
            name="opposed crank arms stay 180 degrees apart while rotating",
        )

    return ctx.report()


object_model = build_object_model()
