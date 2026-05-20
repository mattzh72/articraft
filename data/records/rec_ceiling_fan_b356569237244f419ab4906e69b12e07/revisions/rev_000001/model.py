from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _curved_plate_mesh(
    sections: list[tuple[float, float, float]],
    *,
    z_center: float,
    thickness: float,
) -> MeshGeometry:
    """Closed tapered plate in the XY plane, extruded in Z."""
    geom = MeshGeometry()
    top_left: list[int] = []
    top_right: list[int] = []
    bot_left: list[int] = []
    bot_right: list[int] = []
    z_top = z_center + thickness / 2.0
    z_bot = z_center - thickness / 2.0

    for x, y, half_width in sections:
        top_left.append(geom.add_vertex(x, y - half_width, z_top))
        top_right.append(geom.add_vertex(x, y + half_width, z_top))
        bot_left.append(geom.add_vertex(x, y - half_width, z_bot))
        bot_right.append(geom.add_vertex(x, y + half_width, z_bot))

    for i in range(len(sections) - 1):
        # top and bottom skins
        geom.add_face(top_left[i], top_left[i + 1], top_right[i + 1])
        geom.add_face(top_left[i], top_right[i + 1], top_right[i])
        geom.add_face(bot_left[i], bot_right[i + 1], bot_left[i + 1])
        geom.add_face(bot_left[i], bot_right[i], bot_right[i + 1])

        # long edges
        geom.add_face(top_left[i], bot_left[i + 1], top_left[i + 1])
        geom.add_face(top_left[i], bot_left[i], bot_left[i + 1])
        geom.add_face(top_right[i], top_right[i + 1], bot_right[i + 1])
        geom.add_face(top_right[i], bot_right[i + 1], bot_right[i])

    # root cap
    geom.add_face(top_left[0], top_right[0], bot_right[0])
    geom.add_face(top_left[0], bot_right[0], bot_left[0])
    # tip cap
    geom.add_face(top_left[-1], bot_right[-1], top_right[-1])
    geom.add_face(top_left[-1], bot_left[-1], bot_right[-1])
    return geom


def _ceiling_fan_blade_mesh() -> MeshGeometry:
    """A swept, slightly cambered composite paddle blade."""
    geom = MeshGeometry()
    span_steps = 13
    chord_steps = 8
    root_radius = 0.285
    length = 0.430
    thickness = 0.010
    base_z = -0.042

    top: list[list[int]] = []
    bottom: list[list[int]] = []
    for i in range(span_steps):
        t = i / (span_steps - 1)
        x = root_radius + length * t
        # Forward-swept centerline gives the blade its curved planform.
        center_y = 0.018 * t + 0.052 * math.sin(t * math.pi * 0.82)
        chord = 0.105 + 0.060 * math.sin(t * math.pi * 0.72) - 0.010 * t
        pitch = math.radians(8.0 - 4.0 * t)
        row_top: list[int] = []
        row_bottom: list[int] = []
        for j in range(chord_steps):
            u = j / (chord_steps - 1)
            offset = (u - 0.5) * chord
            # Tiny airfoil-like crown and a twist across the chord.
            camber = 0.006 * math.sin(math.pi * u) * (1.0 - 0.25 * t)
            twist = math.sin(pitch) * offset
            z_top = base_z + camber + twist + thickness / 2.0
            z_bot = z_top - thickness
            y = center_y + offset
            row_top.append(geom.add_vertex(x, y, z_top))
            row_bottom.append(geom.add_vertex(x, y, z_bot))
        top.append(row_top)
        bottom.append(row_bottom)

    for i in range(span_steps - 1):
        for j in range(chord_steps - 1):
            geom.add_face(top[i][j], top[i + 1][j], top[i + 1][j + 1])
            geom.add_face(top[i][j], top[i + 1][j + 1], top[i][j + 1])
            geom.add_face(bottom[i][j], bottom[i + 1][j + 1], bottom[i + 1][j])
            geom.add_face(bottom[i][j], bottom[i][j + 1], bottom[i + 1][j + 1])

    # Leading/trailing edges.
    for i in range(span_steps - 1):
        for j in (0, chord_steps - 1):
            geom.add_face(top[i][j], bottom[i + 1][j], top[i + 1][j])
            geom.add_face(top[i][j], bottom[i][j], bottom[i + 1][j])

    # Root and tip caps.
    for j in range(chord_steps - 1):
        geom.add_face(top[0][j], top[0][j + 1], bottom[0][j + 1])
        geom.add_face(top[0][j], bottom[0][j + 1], bottom[0][j])
        geom.add_face(top[-1][j], bottom[-1][j + 1], top[-1][j + 1])
        geom.add_face(top[-1][j], bottom[-1][j], bottom[-1][j + 1])

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dc_motor_ceiling_fan")

    matte_white = model.material("matte_white", rgba=(0.88, 0.86, 0.80, 1.0))
    warm_metal = model.material("brushed_warm_metal", rgba=(0.58, 0.54, 0.47, 1.0))
    dark_metal = model.material("dark_cast_metal", rgba=(0.06, 0.065, 0.07, 1.0))
    charcoal_composite = model.material("charcoal_composite", rgba=(0.11, 0.12, 0.12, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    screw_metal = model.material("screw_heads", rgba=(0.78, 0.74, 0.66, 1.0))

    body = model.part("body")

    motor_shell = LatheGeometry(
        [
            (0.0, -0.034),
            (0.095, -0.034),
            (0.145, -0.022),
            (0.162, 0.010),
            (0.153, 0.052),
            (0.112, 0.078),
            (0.040, 0.090),
            (0.0, 0.090),
        ],
        segments=72,
        closed=True,
    )
    body.visual(
        mesh_from_geometry(motor_shell, "motor_shell"),
        material=matte_white,
        name="motor_shell",
    )
    body.visual(
        mesh_from_geometry(TorusGeometry(0.154, 0.0032, radial_segments=12, tubular_segments=72), "housing_seam"),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=warm_metal,
        name="housing_seam",
    )
    body.visual(
        Cylinder(radius=0.040, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.118)),
        material=matte_white,
        name="top_collar",
    )
    body.visual(
        Cylinder(radius=0.046, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.037)),
        material=black_rubber,
        name="lower_bearing",
    )

    rod_bottom = (0.0, 0.0, 0.145)
    rod_top = (-0.180, 0.0, 0.905)
    rod_vec = tuple(rod_top[i] - rod_bottom[i] for i in range(3))
    rod_len = math.sqrt(sum(component * component for component in rod_vec))
    rod_mid = tuple((rod_bottom[i] + rod_top[i]) / 2.0 for i in range(3))
    rod_pitch = math.atan2(rod_vec[0], rod_vec[2])
    body.visual(
        Cylinder(radius=0.018, length=rod_len),
        origin=Origin(xyz=rod_mid, rpy=(0.0, rod_pitch, 0.0)),
        material=warm_metal,
        name="angled_downrod",
    )
    body.visual(
        Sphere(radius=0.038),
        origin=Origin(xyz=rod_bottom),
        material=warm_metal,
        name="lower_ball",
    )

    canopy = LatheGeometry(
        [
            (0.0, -0.020),
            (0.062, -0.020),
            (0.092, -0.006),
            (0.084, 0.026),
            (0.058, 0.044),
            (0.0, 0.044),
        ],
        segments=64,
        closed=True,
    )
    body.visual(
        mesh_from_geometry(canopy, "ceiling_canopy"),
        origin=Origin(xyz=rod_top),
        material=matte_white,
        name="ceiling_canopy",
    )
    body.visual(
        Sphere(radius=0.031),
        origin=Origin(xyz=rod_top),
        material=warm_metal,
        name="canopy_ball",
    )

    # Toggle switch socket: a small raised boss on the round motor body.
    body.visual(
        Box((0.030, 0.050, 0.046)),
        origin=Origin(xyz=(0.160, 0.0, 0.014)),
        material=matte_white,
        name="switch_boss",
    )
    body.visual(
        Box((0.003, 0.034, 0.030)),
        origin=Origin(xyz=(0.1765, 0.0, 0.014)),
        material=black_rubber,
        name="switch_slot",
    )

    blade_assembly = model.part("blade_assembly")
    blade_mesh = mesh_from_geometry(_ceiling_fan_blade_mesh(), "curved_composite_blade")
    iron_mesh = mesh_from_geometry(
        _curved_plate_mesh(
            [
                (0.128, 0.000, 0.023),
                (0.185, 0.010, 0.027),
                (0.245, 0.026, 0.040),
                (0.322, 0.045, 0.056),
            ],
            z_center=-0.034,
            thickness=0.016,
        ),
        "blade_iron_bracket",
    )

    blade_assembly.visual(
        Cylinder(radius=0.152, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, -0.016)),
        material=dark_metal,
        name="rotor_ring",
    )
    blade_assembly.visual(
        Cylinder(radius=0.062, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=dark_metal,
        name="central_hub",
    )
    blade_assembly.visual(
        Cylinder(radius=0.020, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, -0.0055)),
        material=warm_metal,
        name="axle_stub",
    )

    bolt_positions = [
        (0.160, -0.014, -0.023),
        (0.160, 0.018, -0.023),
        (0.292, 0.012, -0.029),
        (0.305, 0.067, -0.029),
    ]
    for index in range(5):
        yaw = index * 2.0 * math.pi / 5.0
        rot = Origin(rpy=(0.0, 0.0, yaw))
        blade_assembly.visual(
            iron_mesh,
            origin=rot,
            material=dark_metal,
            name=f"blade_iron_{index}",
        )
        blade_assembly.visual(
            blade_mesh,
            origin=rot,
            material=charcoal_composite,
            name=f"blade_{index}",
        )
        for bolt_index, (x, y, z) in enumerate(bolt_positions):
            blade_assembly.visual(
                Cylinder(radius=0.008, length=0.006),
                origin=Origin(xyz=(x, y, z), rpy=(0.0, 0.0, yaw)),
                material=screw_metal,
                name=f"bolt_{index}_{bolt_index}",
            )

    toggle = model.part("toggle")
    toggle.visual(
        Cylinder(radius=0.008, length=0.026),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=warm_metal,
        name="pivot_disk",
    )
    lever_end = (0.052, 0.0, -0.042)
    lever_len = math.sqrt(lever_end[0] ** 2 + lever_end[2] ** 2)
    lever_pitch = math.atan2(lever_end[0], lever_end[2])
    toggle.visual(
        Cylinder(radius=0.0055, length=lever_len),
        origin=Origin(
            xyz=(lever_end[0] / 2.0, 0.0, lever_end[2] / 2.0),
            rpy=(0.0, lever_pitch, 0.0),
        ),
        material=warm_metal,
        name="toggle_stem",
    )
    toggle.visual(
        Sphere(radius=0.011),
        origin=Origin(xyz=lever_end),
        material=black_rubber,
        name="toggle_tip",
    )

    model.articulation(
        "central_axle",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=blade_assembly,
        origin=Origin(xyz=(0.0, 0.0, -0.060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.5, velocity=55.0),
    )
    model.articulation(
        "direction_toggle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=toggle,
        origin=Origin(xyz=(0.183, 0.0, 0.014)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=3.0, lower=-0.45, upper=0.45),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    rotor = object_model.get_part("blade_assembly")
    toggle = object_model.get_part("toggle")
    axle = object_model.get_articulation("central_axle")
    switch = object_model.get_articulation("direction_toggle")

    ctx.check(
        "blade assembly uses continuous central axle",
        axle.articulation_type == ArticulationType.CONTINUOUS and tuple(axle.axis) == (0.0, 0.0, 1.0),
        details=f"type={axle.articulation_type}, axis={axle.axis}",
    )
    ctx.check(
        "reversing switch is a limited revolute toggle",
        switch.articulation_type == ArticulationType.REVOLUTE
        and switch.motion_limits is not None
        and switch.motion_limits.lower < 0.0
        and switch.motion_limits.upper > 0.0,
        details=f"type={switch.articulation_type}, limits={switch.motion_limits}",
    )
    ctx.check(
        "five separate curved blades are present",
        len([visual.name for visual in rotor.visuals if visual.name and visual.name.startswith("blade_") and not visual.name.startswith("blade_iron")])
        == 5,
    )
    ctx.check(
        "five blade iron brackets are present",
        len([visual.name for visual in rotor.visuals if visual.name and visual.name.startswith("blade_iron_")]) == 5,
    )

    ctx.expect_gap(
        body,
        rotor,
        axis="z",
        positive_elem="motor_shell",
        negative_elem="rotor_ring",
        min_gap=0.015,
        max_gap=0.050,
        name="rotating ring is just below motor housing",
    )
    ctx.expect_overlap(
        rotor,
        body,
        axes="xy",
        elem_a="rotor_ring",
        elem_b="motor_shell",
        min_overlap=0.12,
        name="rotor ring is concentric with housing rim",
    )
    ctx.expect_contact(
        toggle,
        body,
        elem_a="pivot_disk",
        elem_b="switch_boss",
        contact_tol=0.003,
        name="toggle pivot is seated on housing boss",
    )

    lower_aabb = None
    upper_aabb = None
    with ctx.pose({switch: -0.45}):
        lower_aabb = ctx.part_element_world_aabb(toggle, elem="toggle_tip")
    with ctx.pose({switch: 0.45}):
        upper_aabb = ctx.part_element_world_aabb(toggle, elem="toggle_tip")
    if lower_aabb is not None and upper_aabb is not None:
        lower_z = (lower_aabb[0][2] + lower_aabb[1][2]) / 2.0
        upper_z = (upper_aabb[0][2] + upper_aabb[1][2]) / 2.0
        ctx.check(
            "toggle tip flips between reverse positions",
            abs(upper_z - lower_z) > 0.025,
            details=f"lower_z={lower_z:.3f}, upper_z={upper_z:.3f}",
        )
    else:
        ctx.fail("toggle tip flips between reverse positions", "toggle tip AABB was unavailable")

    return ctx.report()


object_model = build_object_model()
