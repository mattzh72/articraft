from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
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


def _annular_cylinder_x(
    *,
    inner_radius: float,
    outer_radius: float,
    length: float,
    segments: int = 72,
) -> MeshGeometry:
    """Build a true hollow tube along the local X axis."""

    geom = MeshGeometry()
    x0 = -length / 2.0
    x1 = length / 2.0
    outer_0: list[int] = []
    outer_1: list[int] = []
    inner_0: list[int] = []
    inner_1: list[int] = []

    for i in range(segments):
        angle = 2.0 * math.pi * i / segments
        c = math.cos(angle)
        s = math.sin(angle)
        outer_0.append(geom.add_vertex(x0, outer_radius * c, outer_radius * s))
        outer_1.append(geom.add_vertex(x1, outer_radius * c, outer_radius * s))
        inner_0.append(geom.add_vertex(x0, inner_radius * c, inner_radius * s))
        inner_1.append(geom.add_vertex(x1, inner_radius * c, inner_radius * s))

    for i in range(segments):
        j = (i + 1) % segments
        # Outer wall.
        geom.add_face(outer_0[i], outer_1[i], outer_1[j])
        geom.add_face(outer_0[i], outer_1[j], outer_0[j])
        # Inner bore wall, wound inward.
        geom.add_face(inner_0[i], inner_1[j], inner_1[i])
        geom.add_face(inner_0[i], inner_0[j], inner_1[j])
        # Left and right annular end faces.
        geom.add_face(outer_0[i], outer_0[j], inner_0[j])
        geom.add_face(outer_0[i], inner_0[j], inner_0[i])
        geom.add_face(outer_1[i], inner_1[j], outer_1[j])
        geom.add_face(outer_1[i], inner_1[i], inner_1[j])

    return geom


def _track_chainring_mesh() -> MeshGeometry:
    teeth = 48
    samples = teeth * 5
    root_radius = 0.101
    tooth_height = 0.006
    outer_profile: list[tuple[float, float]] = []

    for i in range(samples):
        angle = 2.0 * math.pi * i / samples
        phase = (i * teeth / samples) % 1.0
        tooth = max(0.0, 1.0 - abs(phase - 0.50) / 0.24)
        radius = root_radius + tooth_height * tooth
        outer_profile.append((radius * math.cos(angle), radius * math.sin(angle)))

    inner_hole = _circle_profile(0.056, segments=96)
    bolt_circle = 0.074
    bolt_holes = [
        _circle_profile(
            0.0042,
            segments=24,
            center=(
                bolt_circle * math.cos(-math.pi / 2.0 + 2.0 * math.pi * i / 5.0),
                bolt_circle * math.sin(-math.pi / 2.0 + 2.0 * math.pi * i / 5.0),
            ),
        )
        for i in range(5)
    ]
    return ExtrudeWithHolesGeometry(
        outer_profile,
        [inner_hole, *bolt_holes],
        0.004,
        center=True,
    )


def _crank_arm_mesh() -> MeshGeometry:
    """Flattened, tapered forged arm profile, extruded through its thickness."""

    length = 0.170
    root_width = 0.037
    tip_width = 0.022
    side_samples = 20
    cap_samples = 12
    profile: list[tuple[float, float]] = []

    for i in range(side_samples + 1):
        s = i / side_samples
        x = s * length
        width = tip_width + (root_width - tip_width) * (1.0 - s) ** 0.55
        profile.append((x, width / 2.0))

    tip_radius = tip_width / 2.0
    for i in range(1, cap_samples + 1):
        a = math.pi / 2.0 - math.pi * i / cap_samples
        profile.append((length + tip_radius * math.cos(a), tip_radius * math.sin(a)))

    for i in range(side_samples, -1, -1):
        s = i / side_samples
        x = s * length
        width = tip_width + (root_width - tip_width) * (1.0 - s) ** 0.55
        profile.append((x, -width / 2.0))

    root_radius = root_width / 2.0
    for i in range(1, cap_samples + 1):
        a = -math.pi / 2.0 - math.pi * i / cap_samples
        profile.append((root_radius * math.cos(a), root_radius * math.sin(a)))

    return ExtrudeGeometry(profile, 0.014, center=True)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fixie_single_speed_crankset")

    alloy = model.material("brushed_forged_alloy", rgba=(0.72, 0.72, 0.68, 1.0))
    dark_alloy = model.material("black_anodized_alloy", rgba=(0.02, 0.022, 0.024, 1.0))
    steel = model.material("polished_steel", rgba=(0.78, 0.76, 0.70, 1.0))
    black = model.material("matte_black", rgba=(0.01, 0.011, 0.012, 1.0))

    bb_shell = model.part("bb_shell")
    bb_shell.visual(
        mesh_from_geometry(
            _annular_cylinder_x(inner_radius=0.017, outer_radius=0.0235, length=0.076),
            "pressfit_bb_shell",
        ),
        material=black,
        name="pressfit_shell",
    )
    bearing_ring = _annular_cylinder_x(inner_radius=0.0132, outer_radius=0.0208, length=0.006)
    bb_shell.visual(
        mesh_from_geometry(bearing_ring.clone().translate(0.041, 0.0, 0.0), "drive_bearing_ring"),
        material=steel,
        name="drive_bearing_ring",
    )
    bb_shell.visual(
        mesh_from_geometry(bearing_ring.clone().translate(-0.041, 0.0, 0.0), "nondrive_bearing_ring"),
        material=steel,
        name="nondrive_bearing_ring",
    )

    crankset = model.part("crankset")
    crankset.visual(
        Cylinder(radius=0.0115, length=0.178),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="spindle",
    )
    crankset.visual(
        Cylinder(radius=0.026, length=0.020),
        origin=Origin(xyz=(0.052, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=alloy,
        name="right_spindle_boss",
    )
    crankset.visual(
        Cylinder(radius=0.026, length=0.020),
        origin=Origin(xyz=(-0.052, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=alloy,
        name="left_spindle_boss",
    )

    arm_mesh = _crank_arm_mesh()
    crankset.visual(
        mesh_from_geometry(arm_mesh.clone(), "right_crank_arm"),
        origin=Origin(xyz=(0.065, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=alloy,
        name="right_crank_arm",
    )
    crankset.visual(
        mesh_from_geometry(arm_mesh.clone(), "left_crank_arm"),
        origin=Origin(xyz=(-0.065, 0.0, 0.0), rpy=(0.0, -math.pi / 2.0, 0.0)),
        material=alloy,
        name="left_crank_arm",
    )

    crankset.visual(
        mesh_from_geometry(_track_chainring_mesh(), "track_chainring"),
        origin=Origin(xyz=(0.0465, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_alloy,
        name="track_chainring",
    )

    bolt_circle = 0.074
    for i in range(5):
        angle = -math.pi / 2.0 + 2.0 * math.pi * i / 5.0
        y = bolt_circle * math.cos(angle)
        z = bolt_circle * math.sin(angle)
        crankset.visual(
            Box((0.012, 0.010, 0.052)),
            origin=Origin(
                xyz=(0.057, 0.0495 * math.cos(angle), 0.0495 * math.sin(angle)),
                rpy=(angle - math.pi / 2.0, 0.0, 0.0),
            ),
            material=alloy,
            name=f"spider_spoke_{i}",
        )
        crankset.visual(
            Cylinder(radius=0.0072, length=0.006),
            origin=Origin(xyz=(0.053, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"chainring_bolt_{i}",
        )

    for side, x, z, stub_x in (
        ("right", 0.065, -0.170, 0.083),
        ("left", -0.065, 0.170, -0.083),
    ):
        crankset.visual(
            Cylinder(radius=0.019, length=0.020),
            origin=Origin(xyz=(x, 0.0, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=alloy,
            name=f"{side}_pedal_eye",
        )
        crankset.visual(
            Cylinder(radius=0.006, length=0.016),
            origin=Origin(xyz=(stub_x, 0.0, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"{side}_stub_axle",
        )

    for side, direction, z in (("right", 1.0, -0.170), ("left", -1.0, 0.170)):
        pedal = model.part(f"{side}_pedal")
        pedal.visual(
            Box((0.080, 0.095, 0.016)),
            origin=Origin(xyz=(direction * 0.040, 0.0, 0.0)),
            material=black,
            name="platform_body",
        )
        pedal.visual(
            Cylinder(radius=0.013, length=0.078),
            origin=Origin(xyz=(direction * 0.039, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_alloy,
            name="axle_hub",
        )
        for row, y in enumerate((-0.032, 0.032)):
            for col, x_offset in enumerate((0.014, 0.034, 0.054, 0.074)):
                pedal.visual(
                    Cylinder(radius=0.0022, length=0.006),
                    origin=Origin(xyz=(direction * x_offset, y, 0.011)),
                    material=steel,
                    name=f"grip_pin_{row}_{col}",
                )

    model.articulation(
        "spindle_spin",
        ArticulationType.CONTINUOUS,
        parent=bb_shell,
        child=crankset,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=14.0),
    )
    model.articulation(
        "right_pedal_spin",
        ArticulationType.CONTINUOUS,
        parent=crankset,
        child="right_pedal",
        origin=Origin(xyz=(0.091, 0.0, -0.170)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=18.0),
    )
    model.articulation(
        "left_pedal_spin",
        ArticulationType.CONTINUOUS,
        parent=crankset,
        child="left_pedal",
        origin=Origin(xyz=(-0.091, 0.0, 0.170)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bb_shell = object_model.get_part("bb_shell")
    crankset = object_model.get_part("crankset")
    right_pedal = object_model.get_part("right_pedal")
    spindle_spin = object_model.get_articulation("spindle_spin")
    right_pedal_spin = object_model.get_articulation("right_pedal_spin")

    ctx.expect_within(
        crankset,
        bb_shell,
        axes="yz",
        inner_elem="spindle",
        outer_elem="pressfit_shell",
        margin=0.0,
        name="spindle is centered inside the press-fit shell bore envelope",
    )
    ctx.expect_overlap(
        crankset,
        bb_shell,
        axes="x",
        elem_a="spindle",
        elem_b="pressfit_shell",
        min_overlap=0.070,
        name="spindle passes through the bottom-bracket shell",
    )

    rest_pos = ctx.part_world_position(right_pedal)
    with ctx.pose({spindle_spin: math.pi / 2.0}):
        turned_pos = ctx.part_world_position(right_pedal)
    ctx.check(
        "crank arms rotate about the spindle axis",
        rest_pos is not None
        and turned_pos is not None
        and turned_pos[1] > rest_pos[1] + 0.12
        and abs(turned_pos[2]) < 0.03,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    rest_aabb = ctx.part_element_world_aabb(right_pedal, elem="platform_body")
    with ctx.pose({right_pedal_spin: math.pi / 2.0}):
        spun_aabb = ctx.part_element_world_aabb(right_pedal, elem="platform_body")
    if rest_aabb is not None and spun_aabb is not None:
        rest_dy = rest_aabb[1][1] - rest_aabb[0][1]
        rest_dz = rest_aabb[1][2] - rest_aabb[0][2]
        spun_dy = spun_aabb[1][1] - spun_aabb[0][1]
        spun_dz = spun_aabb[1][2] - spun_aabb[0][2]
        pedal_spins = rest_dy > 0.08 and rest_dz < 0.025 and spun_dy < 0.030 and spun_dz > 0.08
    else:
        pedal_spins = False
    ctx.check(
        "platform pedal spins on its stub axle",
        pedal_spins,
        details=f"rest_aabb={rest_aabb}, spun_aabb={spun_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
