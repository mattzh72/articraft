from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(radius: float, segments: int = 48) -> list[tuple[float, float]]:
    return [
        (radius * cos(2.0 * pi * i / segments), radius * sin(2.0 * pi * i / segments))
        for i in range(segments)
    ]


def _toothed_profile(
    root_radius: float,
    tip_radius: float,
    *,
    teeth: int = 28,
    samples_per_tooth: int = 4,
) -> list[tuple[float, float]]:
    """A blunt BMX-style chainring outline with squared-off tooth tips."""
    points: list[tuple[float, float]] = []
    total = teeth * samples_per_tooth
    for i in range(total):
        tooth_phase = (i % samples_per_tooth) / samples_per_tooth
        radius = tip_radius if 0.18 <= tooth_phase <= 0.42 else root_radius
        angle = 2.0 * pi * i / total
        points.append((radius * cos(angle), radius * sin(angle)))
    return points


def _annular_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    width: float,
    segments: int = 64,
):
    geom = ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius, segments),
        [_circle_profile(inner_radius, segments)],
        height=width,
        center=True,
    ).rotate_y(pi / 2.0)
    return mesh_from_geometry(geom, name)


def _chainring_mesh():
    bolt_circle = 0.045
    holes = [_circle_profile(0.024, 48)]
    for i in range(5):
        angle = 2.0 * pi * i / 5.0
        holes.append(
            [
                (0.006 * cos(t) + bolt_circle * cos(angle), 0.006 * sin(t) + bolt_circle * sin(angle))
                for t in (2.0 * pi * j / 20.0 for j in range(20))
            ]
        )

    geom = ExtrudeWithHolesGeometry(
        _toothed_profile(0.087, 0.097, teeth=28, samples_per_tooth=6),
        holes,
        height=0.009,
        center=True,
    ).rotate_y(pi / 2.0)
    return mesh_from_geometry(geom, "toothed_chainring")


def _add_spider_spokes(part, material: str) -> None:
    """Five broad stamped arms tying the thick chainring to the crank boss."""
    for i in range(5):
        angle = 2.0 * pi * i / 5.0 + pi / 10.0
        part.visual(
            Box((0.012, 0.014, 0.060)),
            origin=Origin(
                xyz=(0.120, 0.057 * sin(angle), 0.057 * cos(angle)),
                rpy=(-angle, 0.0, 0.0),
            ),
            material=material,
            name=f"spider_spoke_{i}",
        )


def _add_platform_pedal(part, *, side_sign: float, body_material: str, steel_material: str) -> None:
    """Compact BMX platform pedal, with its body outboard of the crank arm."""
    axle_center = side_sign * 0.071
    body_center = side_sign * 0.082
    half_body_width = 0.040

    part.visual(
        Cylinder(radius=0.006, length=0.092),
        origin=Origin(xyz=(axle_center, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel_material,
        name="pedal_axle",
    )
    part.visual(
        Box((0.090, 0.010, 0.018)),
        origin=Origin(xyz=(body_center, 0.046, 0.0)),
        material=body_material,
        name="cage_rail_0",
    )
    part.visual(
        Box((0.090, 0.010, 0.018)),
        origin=Origin(xyz=(body_center, -0.046, 0.0)),
        material=body_material,
        name="cage_rail_1",
    )
    part.visual(
        Box((0.010, 0.102, 0.018)),
        origin=Origin(xyz=(body_center + side_sign * half_body_width, 0.0, 0.0)),
        material=body_material,
        name="cage_end_0",
    )
    part.visual(
        Box((0.010, 0.102, 0.018)),
        origin=Origin(xyz=(body_center - side_sign * half_body_width, 0.0, 0.0)),
        material=body_material,
        name="cage_end_1",
    )
    part.visual(
        Box((0.030, 0.030, 0.024)),
        origin=Origin(xyz=(body_center, 0.0, 0.0)),
        material=body_material,
        name="center_hub",
    )

    pin_index = 0
    for y in (-0.046, 0.046):
        for x_offset in (-0.026, 0.026):
            for z in (-0.0115, 0.0115):
                part.visual(
                    Cylinder(radius=0.0026, length=0.006),
                    origin=Origin(
                        xyz=(body_center + side_sign * x_offset, y, z),
                        rpy=(0.0, 0.0, 0.0),
                    ),
                    material=steel_material,
                    name=f"grip_pin_{pin_index}",
                )
                pin_index += 1


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bmx_freestyle_crankset")

    model.material("polished_steel", rgba=(0.72, 0.73, 0.70, 1.0))
    model.material("dark_steel", rgba=(0.10, 0.105, 0.11, 1.0))
    model.material("black_powdercoat", rgba=(0.015, 0.016, 0.018, 1.0))
    model.material("bearing_black", rgba=(0.025, 0.026, 0.028, 1.0))
    model.material("chainring_steel", rgba=(0.54, 0.55, 0.52, 1.0))

    bottom_bracket = model.part("bottom_bracket")
    bottom_bracket.visual(
        _annular_mesh(
            "bottom_bracket_sleeve",
            outer_radius=0.044,
            inner_radius=0.024,
            width=0.132,
            segments=72,
        ),
        material="dark_steel",
        name="hollow_sleeve",
    )
    cup_mesh = _annular_mesh(
        "bottom_bracket_cup",
        outer_radius=0.052,
        inner_radius=0.023,
        width=0.018,
        segments=72,
    )
    bottom_bracket.visual(
        cup_mesh,
        origin=Origin(xyz=(0.072, 0.0, 0.0)),
        material="bearing_black",
        name="cup_0",
    )
    bottom_bracket.visual(
        cup_mesh,
        origin=Origin(xyz=(-0.072, 0.0, 0.0)),
        material="bearing_black",
        name="cup_1",
    )

    crankset = model.part("crankset")
    crankset.visual(
        Cylinder(radius=0.017, length=0.395),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material="polished_steel",
        name="wide_spindle",
    )
    for x, radial_sign, boss_name, arm_name, eye_name, web_name in (
        (
            0.190,
            -1.0,
            "chainring_boss",
            "chainring_arm",
            "chainring_pedal_eye",
            "chainring_root_web",
        ),
        (
            -0.190,
            1.0,
            "opposite_boss",
            "opposite_arm",
            "opposite_pedal_eye",
            "opposite_root_web",
        ),
    ):
        crankset.visual(
            Cylinder(radius=0.034, length=0.050),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material="black_powdercoat",
            name=boss_name,
        )
        crankset.visual(
            Box((0.042, 0.036, 0.140)),
            origin=Origin(xyz=(x, 0.0, radial_sign * 0.073)),
            material="black_powdercoat",
            name=arm_name,
        )
        crankset.visual(
            Cylinder(radius=0.024, length=0.050),
            origin=Origin(xyz=(x, 0.0, radial_sign * 0.157), rpy=(0.0, pi / 2.0, 0.0)),
            material="black_powdercoat",
            name=eye_name,
        )
        crankset.visual(
            Box((0.046, 0.044, 0.028)),
            origin=Origin(xyz=(x, 0.0, radial_sign * 0.014)),
            material="black_powdercoat",
            name=web_name,
        )

    crankset.visual(
        _chainring_mesh(),
        origin=Origin(xyz=(0.120, 0.0, 0.0)),
        material="chainring_steel",
        name="thick_chainring",
    )
    crankset.visual(
        Cylinder(radius=0.036, length=0.020),
        origin=Origin(xyz=(0.120, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="chainring_steel",
        name="chainring_hub",
    )
    _add_spider_spokes(crankset, "chainring_steel")
    for i in range(5):
        angle = 2.0 * pi * i / 5.0
        crankset.visual(
            Cylinder(radius=0.005, length=0.006),
            origin=Origin(
                xyz=(0.127, 0.045 * sin(angle), 0.045 * cos(angle)),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material="dark_steel",
            name=f"chainring_bolt_{i}",
        )

    chainring_pedal = model.part("chainring_pedal")
    _add_platform_pedal(
        chainring_pedal,
        side_sign=1.0,
        body_material="black_powdercoat",
        steel_material="polished_steel",
    )

    opposite_pedal = model.part("opposite_pedal")
    _add_platform_pedal(
        opposite_pedal,
        side_sign=-1.0,
        body_material="black_powdercoat",
        steel_material="polished_steel",
    )

    model.articulation(
        "crankset_spin",
        ArticulationType.CONTINUOUS,
        parent=bottom_bracket,
        child=crankset,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=18.0),
    )
    model.articulation(
        "chainring_pedal_spin",
        ArticulationType.CONTINUOUS,
        parent=crankset,
        child=chainring_pedal,
        origin=Origin(xyz=(0.190, 0.0, -0.157)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=30.0),
    )
    model.articulation(
        "opposite_pedal_spin",
        ArticulationType.CONTINUOUS,
        parent=crankset,
        child=opposite_pedal,
        origin=Origin(xyz=(-0.190, 0.0, 0.157)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=30.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bottom_bracket = object_model.get_part("bottom_bracket")
    crankset = object_model.get_part("crankset")
    chainring_pedal = object_model.get_part("chainring_pedal")
    opposite_pedal = object_model.get_part("opposite_pedal")
    crank_spin = object_model.get_articulation("crankset_spin")
    chainring_pedal_spin = object_model.get_articulation("chainring_pedal_spin")
    opposite_pedal_spin = object_model.get_articulation("opposite_pedal_spin")

    ctx.check(
        "spindle and pedals use continuous rotation",
        all(
            joint.articulation_type == ArticulationType.CONTINUOUS
            for joint in (crank_spin, chainring_pedal_spin, opposite_pedal_spin)
        ),
    )

    ctx.allow_overlap(
        bottom_bracket,
        crankset,
        elem_a="hollow_sleeve",
        elem_b="wide_spindle",
        reason=(
            "The wide spindle is intentionally captured through the bottom-bracket bearing bore; "
            "the hollow sleeve mesh is used as the stationary bearing housing."
        ),
    )
    for cup_name in ("cup_0", "cup_1"):
        ctx.allow_overlap(
            bottom_bracket,
            crankset,
            elem_a=cup_name,
            elem_b="wide_spindle",
            reason=(
                "The spindle is intentionally shown running through the bearing cup bore; "
                "the small mesh intersection represents the captured bearing interface."
            ),
        )
        ctx.expect_overlap(
            crankset,
            bottom_bracket,
            axes="x",
            elem_a="wide_spindle",
            elem_b=cup_name,
            min_overlap=0.015,
            name=f"spindle passes through {cup_name}",
        )
    ctx.expect_within(
        crankset,
        bottom_bracket,
        axes="yz",
        inner_elem="wide_spindle",
        outer_elem="hollow_sleeve",
        margin=0.002,
        name="spindle is centered through bottom bracket",
    )
    ctx.expect_overlap(
        crankset,
        bottom_bracket,
        axes="x",
        elem_a="wide_spindle",
        elem_b="hollow_sleeve",
        min_overlap=0.120,
        name="spindle passes through bearing sleeve",
    )

    ctx.expect_gap(
        chainring_pedal,
        crankset,
        axis="x",
        positive_elem="pedal_axle",
        negative_elem="chainring_pedal_eye",
        min_gap=0.0,
        max_gap=0.004,
        name="chainring pedal axle seats at crank eye",
    )
    ctx.expect_overlap(
        chainring_pedal,
        crankset,
        axes="yz",
        elem_a="pedal_axle",
        elem_b="chainring_pedal_eye",
        min_overlap=0.008,
        name="chainring pedal axle aligns with eye",
    )
    ctx.expect_gap(
        crankset,
        opposite_pedal,
        axis="x",
        positive_elem="opposite_pedal_eye",
        negative_elem="pedal_axle",
        min_gap=0.0,
        max_gap=0.004,
        name="opposite pedal axle seats at crank eye",
    )
    ctx.expect_overlap(
        opposite_pedal,
        crankset,
        axes="yz",
        elem_a="pedal_axle",
        elem_b="opposite_pedal_eye",
        min_overlap=0.008,
        name="opposite pedal axle aligns with eye",
    )

    def _aabb_center(aabb, axis_index: int) -> float | None:
        if aabb is None:
            return None
        return 0.5 * (aabb[0][axis_index] + aabb[1][axis_index])

    eye_rest = ctx.part_element_world_aabb(crankset, elem="chainring_pedal_eye")
    with ctx.pose({crank_spin: pi / 2.0}):
        eye_rotated = ctx.part_element_world_aabb(crankset, elem="chainring_pedal_eye")
    rest_y = _aabb_center(eye_rest, 1)
    rest_z = _aabb_center(eye_rest, 2)
    rotated_y = _aabb_center(eye_rotated, 1)
    rotated_z = _aabb_center(eye_rotated, 2)
    ctx.check(
        "crankset spins around wide spindle",
        rest_y is not None
        and rest_z is not None
        and rotated_y is not None
        and rotated_z is not None
        and abs(rotated_y - rest_y) > 0.12
        and abs(rotated_z - rest_z) > 0.12,
        details=f"rest_yz=({rest_y}, {rest_z}), rotated_yz=({rotated_y}, {rotated_z})",
    )

    rail_rest = ctx.part_element_world_aabb(chainring_pedal, elem="cage_rail_0")
    with ctx.pose({chainring_pedal_spin: pi / 2.0}):
        rail_rotated = ctx.part_element_world_aabb(chainring_pedal, elem="cage_rail_0")
    rest_pedal_z = _aabb_center(rail_rest, 2)
    rotated_pedal_z = _aabb_center(rail_rotated, 2)
    ctx.check(
        "platform pedal spins on its axle",
        rest_pedal_z is not None
        and rotated_pedal_z is not None
        and abs(rotated_pedal_z - rest_pedal_z) > 0.035,
        details=f"rest_z={rest_pedal_z}, rotated_z={rotated_pedal_z}",
    )

    return ctx.report()


object_model = build_object_model()
