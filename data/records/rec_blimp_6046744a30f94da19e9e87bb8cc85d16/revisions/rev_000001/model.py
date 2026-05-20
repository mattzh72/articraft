from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    ConeGeometry,
    CylinderGeometry,
    ExtrudeGeometry,
    FanRotorGeometry,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _merge_geometries(*geometries: MeshGeometry) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _cylinder_between(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    radius: float,
    radial_segments: int = 18,
) -> MeshGeometry:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    geometry = CylinderGeometry(radius=radius, height=length, radial_segments=radial_segments)

    if length > 1e-9:
        vx = dx / length
        vy = dy / length
        vz = dz / length
        axis = (-vy, vx, 0.0)
        axis_norm = math.sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2])
        if axis_norm > 1e-9:
            rotation_axis = (axis[0] / axis_norm, axis[1] / axis_norm, axis[2] / axis_norm)
            geometry.rotate(rotation_axis, math.atan2(axis_norm, vz))
        elif vz < 0.0:
            geometry.rotate((1.0, 0.0, 0.0), math.pi)

    geometry.translate(
        0.5 * (start[0] + end[0]),
        0.5 * (start[1] + end[1]),
        0.5 * (start[2] + end[2]),
    )
    return geometry


def _build_horizontal_surface_mesh(
    profile: list[tuple[float, float]],
    *,
    thickness: float,
) -> MeshGeometry:
    return ExtrudeGeometry(profile, thickness, cap=True, center=True, closed=True)


def _build_vertical_surface_mesh(
    profile_xz: list[tuple[float, float]],
    *,
    thickness: float,
) -> MeshGeometry:
    geometry = ExtrudeGeometry(
        [(x_pos, -z_pos) for x_pos, z_pos in profile_xz],
        thickness,
        cap=True,
        center=True,
        closed=True,
    )
    geometry.rotate_x(-math.pi / 2.0)
    return geometry


def _build_envelope_mesh() -> MeshGeometry:
    envelope = LatheGeometry(
        [
            (0.00, -14.00),
            (0.26, -13.72),
            (0.80, -13.20),
            (1.58, -12.05),
            (2.42, -10.20),
            (3.00, -7.20),
            (3.18, -2.60),
            (3.12, 2.80),
            (2.82, 7.80),
            (2.02, 11.20),
            (1.10, 13.15),
            (0.40, 13.82),
            (0.00, 14.00),
        ],
        segments=96,
    )
    envelope.rotate_y(math.pi / 2.0)
    return envelope


def _build_top_fin_mesh() -> MeshGeometry:
    return _build_vertical_surface_mesh(
        [
            (-11.05, 0.18),
            (-12.10, 2.62),
            (-13.30, 2.62),
            (-13.56, 0.18),
        ],
        thickness=0.20,
    )


def _build_ventral_fin_mesh() -> MeshGeometry:
    return _build_vertical_surface_mesh(
        [
            (-11.00, -0.12),
            (-11.86, -1.82),
            (-12.70, -1.82),
            (-12.96, -0.12),
        ],
        thickness=0.14,
    )


def _build_stabilizer_root_mesh(side_sign: float) -> MeshGeometry:
    profile = [
        (-11.10, side_sign * 0.46),
        (-12.18, side_sign * 3.12),
        (-13.20, side_sign * 3.12),
        (-13.28, side_sign * 0.46),
    ]
    if side_sign < 0.0:
        profile.reverse()
    return _build_horizontal_surface_mesh(profile, thickness=0.18)


def _build_elevator_mesh(side_sign: float) -> MeshGeometry:
    profile = [
        (0.020, side_sign * 0.00),
        (-0.200, side_sign * 2.40),
        (-0.76, side_sign * 2.40),
        (-0.86, side_sign * 0.00),
    ]
    if side_sign < 0.0:
        profile.reverse()
    return _build_horizontal_surface_mesh(profile, thickness=0.12)


def _build_rudder_mesh() -> MeshGeometry:
    return _build_vertical_surface_mesh(
        [
            (0.012, 0.00),
            (-0.018, 2.02),
            (-0.60, 2.02),
            (-0.78, 1.02),
            (-0.86, 0.00),
        ],
        thickness=0.12,
    )


def _build_gondola_mesh() -> MeshGeometry:
    cabin_block = BoxGeometry((2.70, 1.18, 1.10)).translate(0.00, 0.0, -1.10)
    roof_block = BoxGeometry((1.90, 0.96, 0.26)).translate(0.10, 0.0, -0.54)
    nose = ConeGeometry(radius=0.46, height=1.00, radial_segments=32, closed=True)
    nose.rotate_y(-math.pi / 2.0)
    nose.translate(1.58, 0.0, -0.98)
    lower_keel = BoxGeometry((2.10, 0.42, 0.22)).translate(-0.10, 0.0, -1.74)
    clamp_fairing = BoxGeometry((0.72, 0.22, 0.12)).translate(0.00, 0.0, -0.06)
    left_outrigger = BoxGeometry((0.70, 0.16, 0.16)).translate(-0.52, 0.62, -0.70)
    right_outrigger = BoxGeometry((0.70, 0.16, 0.16)).translate(-0.52, -0.62, -0.70)

    central_post = _cylinder_between((0.0, 0.0, -0.12), (0.0, 0.0, -0.50), radius=0.08)
    front_hanger = _cylinder_between((0.0, 0.0, -0.12), (1.08, 0.0, -0.48), radius=0.05)
    rear_hanger = _cylinder_between((0.0, 0.0, -0.12), (-0.92, 0.0, -0.46), radius=0.05)

    return _merge_geometries(
        cabin_block,
        roof_block,
        nose,
        lower_keel,
        clamp_fairing,
        left_outrigger,
        right_outrigger,
        central_post,
        front_hanger,
        rear_hanger,
    )


def _build_nacelle_mesh(side_sign: float) -> MeshGeometry:
    nacelle_body = LatheGeometry(
        [
            (0.00, -1.88),
            (0.12, -1.74),
            (0.20, -1.56),
            (0.29, -1.18),
            (0.32, -0.76),
            (0.27, -0.36),
            (0.18, -0.12),
        ],
        segments=72,
    )
    nacelle_body.rotate_y(math.pi / 2.0)
    return nacelle_body


def _build_propeller_mesh() -> MeshGeometry:
    blades = FanRotorGeometry(
        outer_radius=0.82,
        hub_radius=0.12,
        blade_count=3,
        thickness=0.07,
        blade_pitch_deg=30.0,
        blade_sweep_deg=14.0,
        blade_root_chord=0.34,
        blade_tip_chord=0.18,
        center=True,
    )
    blades.rotate_y(math.pi / 2.0)

    spinner = LatheGeometry(
        [
            (0.00, -0.16),
            (0.05, -0.12),
            (0.10, -0.04),
            (0.12, 0.06),
            (0.07, 0.12),
            (0.00, 0.14),
        ],
        segments=48,
    )
    spinner.rotate_y(math.pi / 2.0)

    hub = CylinderGeometry(radius=0.12, height=0.16, radial_segments=28)
    hub.rotate_y(math.pi / 2.0)
    hub.translate(-0.02, 0.0, 0.0)

    return _merge_geometries(blades, spinner, hub)


def _build_axle_fairing_mesh() -> MeshGeometry:
    fairing = LatheGeometry(
        [
            (0.17, -0.20),
            (0.13, -0.12),
            (0.07, -0.02),
            (0.02, 0.06),
            (0.00, 0.12),
        ],
        segments=40,
    )
    fairing.rotate_y(math.pi / 2.0)
    return fairing


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_blimp")

    envelope_skin = model.material("envelope_skin", rgba=(0.86, 0.88, 0.90, 1.0))
    fin_fabric = model.material("fin_fabric", rgba=(0.70, 0.73, 0.77, 1.0))
    structure = model.material("structure", rgba=(0.33, 0.35, 0.38, 1.0))
    gondola_paint = model.material("gondola_paint", rgba=(0.23, 0.26, 0.30, 1.0))
    window_glass = model.material("window_glass", rgba=(0.22, 0.34, 0.42, 0.48))
    engine_paint = model.material("engine_paint", rgba=(0.42, 0.45, 0.49, 1.0))
    prop_finish = model.material("prop_finish", rgba=(0.13, 0.14, 0.15, 1.0))
    hull = model.part("hull")
    hull.visual(
        mesh_from_geometry(_build_envelope_mesh(), "blimp_envelope"),
        material=envelope_skin,
        name="envelope",
    )
    hull.visual(
        Box((0.90, 0.34, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, -3.09)),
        material=structure,
        name="keel_saddle",
    )
    hull.visual(
        mesh_from_geometry(_build_top_fin_mesh(), "blimp_top_fin"),
        material=fin_fabric,
        name="top_fin",
    )
    hull.visual(
        mesh_from_geometry(_build_ventral_fin_mesh(), "blimp_ventral_fin"),
        material=fin_fabric,
        name="ventral_fin",
    )
    hull.visual(
        mesh_from_geometry(_build_stabilizer_root_mesh(1.0), "blimp_left_stabilizer"),
        material=fin_fabric,
        name="left_stabilizer",
    )
    hull.visual(
        mesh_from_geometry(_build_stabilizer_root_mesh(-1.0), "blimp_right_stabilizer"),
        material=fin_fabric,
        name="right_stabilizer",
    )
    left_hinge = CylinderGeometry(radius=0.03, height=2.42, radial_segments=22)
    left_hinge.rotate_x(math.pi / 2.0)
    left_hinge.translate(-13.25, 1.93, 0.0)
    hull.visual(
        mesh_from_geometry(left_hinge, "left_elevator_hinge"),
        material=structure,
        name="left_hinge",
    )
    right_hinge = CylinderGeometry(radius=0.03, height=2.42, radial_segments=22)
    right_hinge.rotate_x(math.pi / 2.0)
    right_hinge.translate(-13.25, -1.93, 0.0)
    hull.visual(
        mesh_from_geometry(right_hinge, "right_elevator_hinge"),
        material=structure,
        name="right_hinge",
    )
    rudder_hinge = CylinderGeometry(radius=0.02, height=2.02, radial_segments=20)
    rudder_hinge.translate(-13.55, 0.0, 1.43)
    hull.visual(
        mesh_from_geometry(rudder_hinge, "rudder_hinge"),
        material=structure,
        name="rudder_hinge",
    )

    gondola = model.part("gondola")
    gondola.visual(
        mesh_from_geometry(_build_gondola_mesh(), "blimp_gondola"),
        material=gondola_paint,
        name="cabin",
    )
    gondola.visual(
        Box((0.42, 0.14, 0.28)),
        origin=Origin(xyz=(1.34, 0.0, -0.96)),
        material=window_glass,
        name="front_window",
    )
    for index, x_pos in enumerate((0.86, 0.10, -0.62)):
        gondola.visual(
            Box((0.24, 0.08, 0.22)),
            origin=Origin(xyz=(x_pos, 0.57, -1.00)),
            material=window_glass,
            name=f"left_window_{index}",
        )
        gondola.visual(
            Box((0.24, 0.08, 0.22)),
            origin=Origin(xyz=(x_pos, -0.57, -1.00)),
            material=window_glass,
            name=f"right_window_{index}",
        )

    left_nacelle = model.part("left_nacelle")
    left_nacelle.visual(
        mesh_from_geometry(_build_nacelle_mesh(1.0), "left_nacelle_mesh"),
        material=engine_paint,
        name="pod",
    )
    left_nacelle.visual(
        mesh_from_geometry(_build_axle_fairing_mesh(), "left_nacelle_axle_fairing"),
        material=structure,
        name="axle_fairing",
    )
    left_nacelle.visual(
        Box((0.62, 0.16, 0.16)),
        origin=Origin(xyz=(-0.45, -0.72, 0.10)),
        material=structure,
        name="mount_pad",
    )
    left_nacelle.visual(
        mesh_from_geometry(
            _cylinder_between((-0.72, -0.72, 0.10), (-0.80, 0.0, 0.22), radius=0.045),
            "left_nacelle_rear_strut",
        ),
        material=structure,
        name="rear_strut",
    )
    left_nacelle.visual(
        mesh_from_geometry(
            _cylinder_between((-0.56, -0.72, 0.02), (-0.90, 0.0, -0.14), radius=0.032),
            "left_nacelle_lower_brace",
        ),
        material=structure,
        name="lower_brace",
    )

    right_nacelle = model.part("right_nacelle")
    right_nacelle.visual(
        mesh_from_geometry(_build_nacelle_mesh(-1.0), "right_nacelle_mesh"),
        material=engine_paint,
        name="pod",
    )
    right_nacelle.visual(
        mesh_from_geometry(_build_axle_fairing_mesh(), "right_nacelle_axle_fairing"),
        material=structure,
        name="axle_fairing",
    )
    right_nacelle.visual(
        Box((0.62, 0.16, 0.16)),
        origin=Origin(xyz=(-0.45, 0.72, 0.10)),
        material=structure,
        name="mount_pad",
    )
    right_nacelle.visual(
        mesh_from_geometry(
            _cylinder_between((-0.72, 0.72, 0.10), (-0.80, 0.0, 0.22), radius=0.045),
            "right_nacelle_rear_strut",
        ),
        material=structure,
        name="rear_strut",
    )
    right_nacelle.visual(
        mesh_from_geometry(
            _cylinder_between((-0.56, 0.72, 0.02), (-0.90, 0.0, -0.14), radius=0.032),
            "right_nacelle_lower_brace",
        ),
        material=structure,
        name="lower_brace",
    )

    propeller_mesh = mesh_from_geometry(_build_propeller_mesh(), "blimp_propeller")
    propeller_shaft = CylinderGeometry(radius=0.05, height=0.20, radial_segments=24)
    propeller_shaft.rotate_y(math.pi / 2.0)
    propeller_shaft.translate(-0.08, 0.0, 0.0)
    propeller_shaft_mesh = mesh_from_geometry(propeller_shaft, "blimp_propeller_shaft")

    left_propeller = model.part("left_propeller")
    left_propeller.visual(propeller_mesh, material=prop_finish, name="prop")
    left_propeller.visual(propeller_shaft_mesh, material=structure, name="shaft")

    right_propeller = model.part("right_propeller")
    right_propeller.visual(propeller_mesh, material=prop_finish, name="prop")
    right_propeller.visual(propeller_shaft_mesh, material=structure, name="shaft")

    rudder = model.part("rudder")
    rudder.visual(
        mesh_from_geometry(_build_rudder_mesh(), "blimp_rudder"),
        material=fin_fabric,
        name="panel",
    )

    left_elevator = model.part("left_elevator")
    left_elevator.visual(
        mesh_from_geometry(_build_elevator_mesh(1.0), "blimp_left_elevator"),
        material=fin_fabric,
        name="panel",
    )

    right_elevator = model.part("right_elevator")
    right_elevator.visual(
        mesh_from_geometry(_build_elevator_mesh(-1.0), "blimp_right_elevator"),
        material=fin_fabric,
        name="panel",
    )

    model.articulation(
        "hull_to_gondola",
        ArticulationType.FIXED,
        parent=hull,
        child=gondola,
        origin=Origin(xyz=(0.35, 0.0, -3.18)),
    )
    model.articulation(
        "gondola_to_left_nacelle",
        ArticulationType.FIXED,
        parent=gondola,
        child=left_nacelle,
        origin=Origin(xyz=(-0.30, 1.50, -0.88)),
    )
    model.articulation(
        "gondola_to_right_nacelle",
        ArticulationType.FIXED,
        parent=gondola,
        child=right_nacelle,
        origin=Origin(xyz=(-0.30, -1.50, -0.88)),
    )
    model.articulation(
        "left_nacelle_to_left_propeller",
        ArticulationType.CONTINUOUS,
        parent=left_nacelle,
        child=left_propeller,
        origin=Origin(xyz=(0.02, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=60.0),
    )
    model.articulation(
        "right_nacelle_to_right_propeller",
        ArticulationType.CONTINUOUS,
        parent=right_nacelle,
        child=right_propeller,
        origin=Origin(xyz=(0.02, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=60.0),
    )
    model.articulation(
        "hull_to_rudder",
        ArticulationType.REVOLUTE,
        parent=hull,
        child=rudder,
        origin=Origin(xyz=(-13.58, 0.0, 0.42)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.2, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "hull_to_left_elevator",
        ArticulationType.REVOLUTE,
        parent=hull,
        child=left_elevator,
        origin=Origin(xyz=(-13.30, 0.72, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.2, lower=-0.42, upper=0.42),
    )
    model.articulation(
        "hull_to_right_elevator",
        ArticulationType.REVOLUTE,
        parent=hull,
        child=right_elevator,
        origin=Origin(xyz=(-13.30, -0.72, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.2, lower=-0.42, upper=0.42),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    hull = object_model.get_part("hull")
    gondola = object_model.get_part("gondola")
    left_nacelle = object_model.get_part("left_nacelle")
    right_nacelle = object_model.get_part("right_nacelle")
    rudder = object_model.get_part("rudder")
    left_elevator = object_model.get_part("left_elevator")
    right_elevator = object_model.get_part("right_elevator")

    left_propeller_joint = object_model.get_articulation("left_nacelle_to_left_propeller")
    right_propeller_joint = object_model.get_articulation("right_nacelle_to_right_propeller")
    rudder_joint = object_model.get_articulation("hull_to_rudder")
    left_elevator_joint = object_model.get_articulation("hull_to_left_elevator")
    right_elevator_joint = object_model.get_articulation("hull_to_right_elevator")

    ctx.expect_origin_gap(hull, gondola, axis="z", min_gap=3.0, name="gondola hangs below the envelope")
    ctx.expect_origin_gap(left_nacelle, gondola, axis="y", min_gap=1.2, name="left nacelle sits outboard of the gondola")
    ctx.expect_origin_gap(gondola, right_nacelle, axis="y", min_gap=1.2, name="right nacelle sits outboard of the gondola")
    ctx.allow_overlap(
        hull,
        rudder,
        elem_a="rudder_hinge",
        elem_b="panel",
        reason="The rudder hinge pin is intentionally modeled as nesting into the rudder leading edge.",
    )
    ctx.allow_overlap(
        left_nacelle,
        "left_propeller",
        elem_a="axle_fairing",
        elem_b="prop",
        reason="The propeller hub is intentionally seated into the nacelle nose bearing fairing.",
    )
    ctx.allow_overlap(
        left_nacelle,
        "left_propeller",
        elem_a="axle_fairing",
        elem_b="shaft",
        reason="The propeller shaft is intentionally seated into the nacelle nose bearing fairing.",
    )
    ctx.allow_overlap(
        right_nacelle,
        "right_propeller",
        elem_a="axle_fairing",
        elem_b="prop",
        reason="The propeller hub is intentionally seated into the nacelle nose bearing fairing.",
    )
    ctx.allow_overlap(
        right_nacelle,
        "right_propeller",
        elem_a="axle_fairing",
        elem_b="shaft",
        reason="The propeller shaft is intentionally seated into the nacelle nose bearing fairing.",
    )

    for joint_name, joint in (
        ("left propeller uses a continuous axle", left_propeller_joint),
        ("right propeller uses a continuous axle", right_propeller_joint),
    ):
        limits = joint.motion_limits
        ctx.check(
            joint_name,
            joint.articulation_type == ArticulationType.CONTINUOUS
            and limits is not None
            and limits.lower is None
            and limits.upper is None,
            details=f"type={joint.articulation_type}, limits={limits}",
        )

    rudder_limits = rudder_joint.motion_limits
    if rudder_limits is not None and rudder_limits.upper is not None:
        rudder_rest = ctx.part_element_world_aabb(rudder, elem="panel")
        with ctx.pose({rudder_joint: rudder_limits.upper}):
            rudder_turned = ctx.part_element_world_aabb(rudder, elem="panel")
        ctx.check(
            "rudder deflects laterally",
            rudder_rest is not None
            and rudder_turned is not None
            and rudder_turned[1][1] > rudder_rest[1][1] + 0.16,
            details=f"rest={rudder_rest}, turned={rudder_turned}",
        )

    left_limits = left_elevator_joint.motion_limits
    if left_limits is not None and left_limits.upper is not None:
        left_rest = ctx.part_element_world_aabb(left_elevator, elem="panel")
        with ctx.pose({left_elevator_joint: left_limits.upper}):
            left_up = ctx.part_element_world_aabb(left_elevator, elem="panel")
        ctx.check(
            "left elevator raises at positive deflection",
            left_rest is not None and left_up is not None and left_up[1][2] > left_rest[1][2] + 0.10,
            details=f"rest={left_rest}, raised={left_up}",
        )

    right_limits = right_elevator_joint.motion_limits
    if right_limits is not None and right_limits.upper is not None:
        right_rest = ctx.part_element_world_aabb(right_elevator, elem="panel")
        with ctx.pose({right_elevator_joint: right_limits.upper}):
            right_up = ctx.part_element_world_aabb(right_elevator, elem="panel")
        ctx.check(
            "right elevator raises at positive deflection",
            right_rest is not None and right_up is not None and right_up[1][2] > right_rest[1][2] + 0.10,
            details=f"rest={right_rest}, raised={right_up}",
        )

    return ctx.report()


object_model = build_object_model()
