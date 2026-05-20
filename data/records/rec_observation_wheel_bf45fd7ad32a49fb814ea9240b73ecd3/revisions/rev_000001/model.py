from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


AXLE_Z = 2.58
RIM_RADIUS = 1.35
PIVOT_RADIUS = 1.46
CABIN_COUNT = 8


def _mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, name)


def _tube_between(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    segments: int = 16,
) -> MeshGeometry:
    return tube_from_spline_points(
        [a, b],
        radius=radius,
        samples_per_segment=2,
        radial_segments=segments,
        cap_ends=True,
    )


def _rim_structure_mesh() -> MeshGeometry:
    """Double-sided observation-wheel rim with spokes and bracket bridges."""
    geom = MeshGeometry()

    for y in (-0.12, 0.12):
        geom.merge(
            TorusGeometry(
                radius=RIM_RADIUS,
                tube=0.045,
                radial_segments=18,
                tubular_segments=96,
            )
            .rotate_x(math.pi / 2.0)
            .translate(0.0, y, 0.0)
        )
        for index in range(16):
            angle = 2.0 * math.pi * index / 16.0
            endpoint = (
                math.cos(angle) * (RIM_RADIUS - 0.01),
                y,
                math.sin(angle) * (RIM_RADIUS - 0.01),
            )
            geom.merge(_tube_between((0.0, y, 0.0), endpoint, radius=0.015, segments=12))

    for index in range(CABIN_COUNT):
        angle = 2.0 * math.pi * index / CABIN_COUNT
        root = (
            math.cos(angle) * (RIM_RADIUS - 0.03),
            0.0,
            math.sin(angle) * (RIM_RADIUS - 0.03),
        )
        for y0, y1 in ((-0.12, -0.08), (0.08, 0.12)):
            geom.merge(
                _tube_between(
                    (root[0], y0, root[2]),
                    (root[0], y1, root[2]),
                    radius=0.022,
                    segments=12,
                )
            )

    geom.merge(CylinderGeometry(radius=0.16, height=0.34, radial_segments=32).rotate_x(math.pi / 2.0))
    geom.merge(CylinderGeometry(radius=0.07, height=0.72, radial_segments=28).rotate_x(math.pi / 2.0))
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_urban_observation_wheel")

    painted_steel = model.material("painted_steel", rgba=(0.86, 0.88, 0.86, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.16, 0.17, 0.18, 1.0))
    concrete = model.material("warm_concrete", rgba=(0.63, 0.61, 0.56, 1.0))
    bearing_blue = model.material("bearing_blue", rgba=(0.08, 0.20, 0.34, 1.0))
    cabin_red = model.material("cabin_red", rgba=(0.75, 0.16, 0.10, 1.0))
    cabin_yellow = model.material("cabin_yellow", rgba=(0.95, 0.66, 0.12, 1.0))
    cabin_teal = model.material("cabin_teal", rgba=(0.12, 0.56, 0.58, 1.0))
    glass = model.material("blue_tinted_glass", rgba=(0.35, 0.70, 0.90, 0.65))

    base = model.part("base")
    base.visual(Box((1.85, 1.08, 0.12)), origin=Origin(xyz=(0.0, 0.0, 0.06)), material=concrete, name="plinth")
    base.visual(Box((1.20, 0.52, 0.10)), origin=Origin(xyz=(0.0, 0.0, 0.17)), material=dark_steel, name="foot_plate")
    for y, suffix in ((0.42, "pos"), (-0.42, "neg")):
        support_geom = MeshGeometry()
        support_geom.merge(_tube_between((-0.72, y, 0.14), (0.0, y, AXLE_Z), radius=0.045))
        support_geom.merge(_tube_between((0.72, y, 0.14), (0.0, y, AXLE_Z), radius=0.045))
        support_geom.merge(_tube_between((-0.56, y, 0.70), (0.56, y, 0.70), radius=0.030))
        support_geom.merge(_tube_between((-0.30, y, 1.40), (0.30, y, 1.40), radius=0.028))
        base.visual(_mesh(f"side_support_{suffix}", support_geom), material=dark_steel, name=f"side_support_{suffix}")
        if suffix == "pos":
            base.visual(
                Cylinder(radius=0.135, length=0.12),
                origin=Origin(xyz=(0.0, y, AXLE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=bearing_blue,
                name="bearing_pos",
            )
        else:
            base.visual(
                Cylinder(radius=0.135, length=0.12),
                origin=Origin(xyz=(0.0, y, AXLE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=bearing_blue,
                name="bearing_neg",
            )
        base.visual(
            Box((0.30, 0.11, 0.22)),
            origin=Origin(xyz=(0.0, y, AXLE_Z - 0.02)),
            material=bearing_blue,
            name=f"bearing_block_{suffix}",
        )
    base.visual(
        _mesh(
            "base_cross_tie",
            _tube_between((-0.72, -0.42, 0.14), (-0.72, 0.42, 0.14), radius=0.035),
        ),
        material=dark_steel,
        name="rear_cross_tie",
    )
    base.visual(
        _mesh(
            "front_cross_tie",
            _tube_between((0.72, -0.42, 0.14), (0.72, 0.42, 0.14), radius=0.035),
        ),
        material=dark_steel,
        name="front_cross_tie",
    )
    base.inertial = Inertial.from_geometry(Box((1.85, 1.08, AXLE_Z + 0.25)), mass=2200.0, origin=Origin(xyz=(0.0, 0.0, 1.20)))

    wheel = model.part("wheel")
    wheel.visual(_mesh("wheel_rim_spokes", _rim_structure_mesh()), material=painted_steel, name="rim_spokes")
    wheel.visual(
        Cylinder(radius=0.075, length=0.72),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="axle",
    )
    wheel.visual(
        Cylinder(radius=0.18, length=0.28),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bearing_blue,
        name="hub",
    )
    for index in range(CABIN_COUNT):
        angle = 2.0 * math.pi * index / CABIN_COUNT
        radial = (math.cos(angle), 0.0, math.sin(angle))
        pivot = (radial[0] * PIVOT_RADIUS, 0.0, radial[2] * PIVOT_RADIUS)
        bridge_center = (
            pivot[0] - radial[0] * 0.085,
            0.0,
            pivot[2] - radial[2] * 0.085,
        )
        for bridge_y, bridge_suffix in ((0.1575, "pos"), (-0.1575, "neg")):
            wheel.visual(
                Box((0.11, 0.115, 0.050)),
                origin=Origin(xyz=(bridge_center[0], bridge_y, bridge_center[2]), rpy=(0.0, -angle, 0.0)),
                material=painted_steel,
                name=f"bracket_{index}_bridge_{bridge_suffix}",
            )
        for y, suffix in ((0.215, "pos"), (-0.215, "neg")):
            cheek_center = (pivot[0], y, pivot[2])
            if index == 0 and suffix == "pos":
                wheel.visual(
                    Box((0.095, 0.040, 0.125)),
                    origin=Origin(xyz=cheek_center, rpy=(0.0, -angle, 0.0)),
                    material=painted_steel,
                    name="bracket_0_cheek_pos",
                )
            elif index == 0 and suffix == "neg":
                wheel.visual(
                    Box((0.095, 0.040, 0.125)),
                    origin=Origin(xyz=cheek_center, rpy=(0.0, -angle, 0.0)),
                    material=painted_steel,
                    name="bracket_0_cheek_neg",
                )
            else:
                wheel.visual(
                    Box((0.095, 0.040, 0.125)),
                    origin=Origin(xyz=cheek_center, rpy=(0.0, -angle, 0.0)),
                    material=painted_steel,
                    name=f"bracket_{index}_cheek_{suffix}",
                )
    wheel.inertial = Inertial.from_geometry(Cylinder(radius=RIM_RADIUS, length=0.72), mass=900.0, origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)))

    model.articulation(
        "wheel_rotation",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, AXLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4500.0, velocity=0.65),
    )

    cabin_materials = (cabin_red, cabin_yellow, cabin_teal)
    for index in range(CABIN_COUNT):
        cabin = model.part(f"cabin_{index}")
        cabin.visual(
            Cylinder(radius=0.022, length=0.39),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name="pivot_pin",
        )
        cabin.visual(Box((0.035, 0.018, 0.54)), origin=Origin(xyz=(0.0, 0.040, -0.27)), material=dark_steel, name="hanger_pos")
        cabin.visual(Box((0.035, 0.018, 0.54)), origin=Origin(xyz=(0.0, -0.040, -0.27)), material=dark_steel, name="hanger_neg")
        cabin.visual(
            Box((0.36, 0.10, 0.34)),
            origin=Origin(xyz=(0.0, 0.0, -0.68)),
            material=cabin_materials[index % len(cabin_materials)],
            name="square_cabin",
        )
        cabin.visual(
            Box((0.24, 0.012, 0.16)),
            origin=Origin(xyz=(0.0, 0.056, -0.68)),
            material=glass,
            name="front_window",
        )
        cabin.visual(
            Box((0.24, 0.012, 0.16)),
            origin=Origin(xyz=(0.0, -0.056, -0.68)),
            material=glass,
            name="rear_window",
        )
        cabin.visual(
            Box((0.018, 0.07, 0.14)),
            origin=Origin(xyz=(0.185, 0.0, -0.68)),
            material=glass,
            name="side_window_pos",
        )
        cabin.visual(
            Box((0.018, 0.07, 0.14)),
            origin=Origin(xyz=(-0.185, 0.0, -0.68)),
            material=glass,
            name="side_window_neg",
        )
        cabin.visual(Box((0.40, 0.12, 0.035)), origin=Origin(xyz=(0.0, 0.0, -0.8675)), material=dark_steel, name="floor_skid")
        cabin.inertial = Inertial.from_geometry(Box((0.40, 0.14, 0.90)), mass=120.0, origin=Origin(xyz=(0.0, 0.0, -0.45)))

        angle = 2.0 * math.pi * index / CABIN_COUNT
        model.articulation(
            f"cabin_{index}_pivot",
            ArticulationType.REVOLUTE,
            parent=wheel,
            child=cabin,
            origin=Origin(xyz=(math.cos(angle) * PIVOT_RADIUS, 0.0, math.sin(angle) * PIVOT_RADIUS)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=80.0, velocity=1.8, lower=-1.35, upper=1.35),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wheel = object_model.get_part("wheel")
    base = object_model.get_part("base")
    wheel_joint = object_model.get_articulation("wheel_rotation")

    ctx.check(
        "wheel uses continuous horizontal axle",
        wheel_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(wheel_joint.axis) == (0.0, 1.0, 0.0),
        details=f"type={wheel_joint.articulation_type}, axis={wheel_joint.axis}",
    )
    ctx.expect_gap(
        base,
        wheel,
        axis="y",
        positive_elem="bearing_pos",
        negative_elem="axle",
        max_gap=0.003,
        max_penetration=0.0,
        name="axle reaches positive bearing face",
    )
    ctx.expect_gap(
        wheel,
        base,
        axis="y",
        positive_elem="axle",
        negative_elem="bearing_neg",
        max_gap=0.003,
        max_penetration=0.0,
        name="axle reaches negative bearing face",
    )

    cabin_joints = [object_model.get_articulation(f"cabin_{index}_pivot") for index in range(CABIN_COUNT)]
    ctx.check(
        "all square cabins have hanger pivots",
        all(j.articulation_type == ArticulationType.REVOLUTE and tuple(j.axis) == (0.0, 1.0, 0.0) for j in cabin_joints),
        details=[(j.name, j.articulation_type, j.axis) for j in cabin_joints],
    )
    ctx.expect_gap(
        wheel,
        "cabin_0",
        axis="y",
        positive_elem="bracket_0_cheek_pos",
        negative_elem="pivot_pin",
        max_gap=0.003,
        max_penetration=0.0,
        name="cabin pin sits in positive fork cheek",
    )
    ctx.expect_gap(
        "cabin_0",
        wheel,
        axis="y",
        positive_elem="pivot_pin",
        negative_elem="bracket_0_cheek_neg",
        max_gap=0.003,
        max_penetration=0.0,
        name="cabin pin sits in negative fork cheek",
    )

    rest_pos = ctx.part_world_position("cabin_0")
    with ctx.pose({wheel_joint: math.pi / 2.0}):
        turned_pos = ctx.part_world_position("cabin_0")
    ctx.check(
        "wheel rotation moves rim cabin around axle",
        rest_pos is not None
        and turned_pos is not None
        and abs(rest_pos[0] - turned_pos[0]) > 0.50
        and abs(rest_pos[2] - turned_pos[2]) > 0.50,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


object_model = build_object_model()
