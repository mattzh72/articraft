from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BlowerWheelGeometry,
    Box,
    Cylinder,
    ExtrudeGeometry,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MeshGeometry,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    superellipse_profile,
)


def _tapered_rear_shell_geometry() -> MeshGeometry:
    """Open rear/side skin for the fan column, leaving the front for the grille."""
    geom = MeshGeometry()
    z_levels = [0.055, 0.18, 0.36, 0.58, 0.80, 0.975]
    # Angles wrap from the right-front edge, around the rear, to the left-front edge.
    theta_values = [math.radians(a) for a in (-50, -25, 0, 35, 70, 105, 140, 175, 205, 230)]
    rings: list[list[int]] = []
    for z in z_levels:
        t = (z - z_levels[0]) / (z_levels[-1] - z_levels[0])
        # Gently taper from the heavier motor end to a slimmer top.
        width = 0.190 - 0.035 * t
        depth = 0.145 - 0.022 * t
        ring: list[int] = []
        for theta in theta_values:
            x = 0.5 * width * math.cos(theta)
            y = 0.5 * depth * math.sin(theta)
            ring.append(geom.add_vertex(x, y, z))
        rings.append(ring)

    for lower, upper in zip(rings, rings[1:]):
        for i in range(len(theta_values) - 1):
            a, b = lower[i], lower[i + 1]
            c, d = upper[i], upper[i + 1]
            geom.add_face(a, c, b)
            geom.add_face(b, c, d)

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="home_tower_fan")

    matte_black = model.material("matte_black", rgba=(0.015, 0.016, 0.018, 1.0))
    warm_gray = model.material("warm_gray_plastic", rgba=(0.72, 0.70, 0.66, 1.0))
    dark_gray = model.material("dark_gray_plastic", rgba=(0.11, 0.115, 0.12, 1.0))
    satin_silver = model.material("satin_silver", rgba=(0.58, 0.60, 0.62, 1.0))
    black_mesh = model.material("black_recessed_mesh", rgba=(0.005, 0.006, 0.007, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.185, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_gray,
        name="round_foot",
    )
    base.visual(
        Cylinder(radius=0.132, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0775)),
        material=warm_gray,
        name="motor_housing",
    )
    base.visual(
        Cylinder(radius=0.068, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.1175)),
        material=dark_gray,
        name="pivot_socket",
    )

    column = model.part("column")
    column.visual(
        Cylinder(radius=0.066, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=dark_gray,
        name="bottom_collar",
    )
    column.visual(
        mesh_from_geometry(
            ExtrudeGeometry.from_z0(superellipse_profile(0.195, 0.145, 2.7, segments=56), 0.052),
            "bottom_fascia",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=warm_gray,
        name="bottom_fascia",
    )
    column.visual(
        mesh_from_geometry(_tapered_rear_shell_geometry(), "tapered_rear_shell"),
        material=warm_gray,
        name="tapered_shell",
    )
    # Side rails and cross bars tie the grille into the tapered shell and give the
    # front a molded appliance-frame look.
    for index, x in enumerate((-0.078, 0.078)):
        column.visual(
            Box((0.024, 0.026, 0.825)),
            origin=Origin(xyz=(x, -0.063, 0.515)),
            material=warm_gray,
            name=f"side_rail_{index}",
        )
    column.visual(
        Box((0.150, 0.026, 0.026)),
        origin=Origin(xyz=(0.0, -0.063, 0.925)),
        material=warm_gray,
        name="upper_grille_rail",
    )
    column.visual(
        Box((0.156, 0.026, 0.030)),
        origin=Origin(xyz=(0.0, -0.063, 0.105)),
        material=warm_gray,
        name="lower_grille_rail",
    )
    column.visual(
        mesh_from_geometry(
            SlotPatternPanelGeometry(
                (0.132, 0.770),
                0.006,
                slot_size=(0.070, 0.006),
                pitch=(0.014, 0.086),
                frame=0.011,
                corner_radius=0.012,
                slot_angle_deg=88.0,
                stagger=True,
            ),
            "front_grille",
        ),
        # Local panel Z is the thickness direction; rotate it so the plate lies in XZ.
        origin=Origin(xyz=(0.0, -0.073, 0.515), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_mesh,
        name="front_grille",
    )
    column.visual(
        mesh_from_geometry(
            ExtrudeGeometry.from_z0(superellipse_profile(0.160, 0.118, 2.8, segments=56), 0.035),
            "top_control_cluster",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.972)),
        material=warm_gray,
        name="top_cap",
    )
    column.visual(
        Box((0.092, 0.060, 0.006)),
        origin=Origin(xyz=(0.0, -0.010, 1.010)),
        material=dark_gray,
        name="control_plate",
    )
    for index, z in enumerate((0.117, 0.913)):
        column.visual(
            mesh_from_geometry(TorusGeometry(0.0105, 0.0035, radial_segments=24, tubular_segments=12), f"bearing_ring_{index}"),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=matte_black,
            name=f"bearing_ring_{index}",
        )
        column.visual(
            Box((0.006, 0.050, 0.007)),
            origin=Origin(xyz=(0.0, -0.038, z)),
            material=matte_black,
            name=f"bearing_spoke_{index}",
        )

    rotor = model.part("blower_rotor")
    rotor.visual(
        mesh_from_geometry(
            BlowerWheelGeometry(
                outer_radius=0.046,
                inner_radius=0.023,
                width=0.735,
                blade_count=28,
                blade_thickness=0.0026,
                blade_sweep_deg=28.0,
                backplate=True,
                shroud=True,
            ),
            "blower_wheel",
        ),
        origin=Origin(),
        material=satin_silver,
        name="blower_wheel",
    )
    rotor.visual(
        Cylinder(radius=0.007, length=0.810),
        origin=Origin(),
        material=matte_black,
        name="rotor_shaft",
    )
    rotor.visual(
        Cylinder(radius=0.026, length=0.030),
        origin=Origin(),
        material=satin_silver,
        name="rotor_hub",
    )

    speed_knob = model.part("speed_knob")
    speed_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.044,
                0.026,
                body_style="domed",
                base_diameter=0.048,
                top_diameter=0.038,
                edge_radius=0.0012,
                grip=KnobGrip(style="ribbed", count=18, depth=0.0009, width=0.0012),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "speed_knob",
        ),
        origin=Origin(),
        material=dark_gray,
        name="knob_cap",
    )

    model.articulation(
        "base_to_column",
        ArticulationType.REVOLUTE,
        parent=base,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, 0.130)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.2, lower=-0.85, upper=0.85),
    )
    model.articulation(
        "column_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=column,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.520)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=45.0),
    )
    model.articulation(
        "column_to_speed_knob",
        ArticulationType.CONTINUOUS,
        parent=column,
        child=speed_knob,
        origin=Origin(xyz=(0.0, -0.010, 1.013)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.15, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    column = object_model.get_part("column")
    rotor = object_model.get_part("blower_rotor")
    knob = object_model.get_part("speed_knob")
    oscillation = object_model.get_articulation("base_to_column")
    spin = object_model.get_articulation("column_to_rotor")
    knob_spin = object_model.get_articulation("column_to_speed_knob")

    for bearing_name in ("bearing_ring_0", "bearing_ring_1"):
        ctx.allow_overlap(
            rotor,
            column,
            elem_a="rotor_shaft",
            elem_b=bearing_name,
            reason="The blower shaft is intentionally captured inside a molded bearing ring so the rotor is mechanically supported.",
        )
        ctx.expect_contact(
            rotor,
            column,
            elem_a="rotor_shaft",
            elem_b=bearing_name,
            contact_tol=0.001,
            name=f"rotor shaft is carried by {bearing_name}",
        )

    ctx.check(
        "column oscillates on limited vertical revolute joint",
        oscillation.articulation_type == ArticulationType.REVOLUTE
        and oscillation.axis == (0.0, 0.0, 1.0)
        and oscillation.motion_limits is not None
        and oscillation.motion_limits.lower < -0.5
        and oscillation.motion_limits.upper > 0.5,
        details=str(oscillation),
    )
    ctx.check(
        "blower rotor and speed knob use continuous vertical spin",
        spin.articulation_type == ArticulationType.CONTINUOUS
        and knob_spin.articulation_type == ArticulationType.CONTINUOUS
        and spin.axis == (0.0, 0.0, 1.0)
        and knob_spin.axis == (0.0, 0.0, 1.0),
        details=f"rotor={spin.articulation_type}/{spin.axis}, knob={knob_spin.articulation_type}/{knob_spin.axis}",
    )

    ctx.expect_gap(
        column,
        base,
        axis="z",
        positive_elem="bottom_collar",
        negative_elem="pivot_socket",
        max_gap=0.001,
        max_penetration=0.0,
        name="rotating collar sits on base socket",
    )
    ctx.expect_gap(
        knob,
        column,
        axis="z",
        positive_elem="knob_cap",
        negative_elem="control_plate",
        max_gap=0.0015,
        max_penetration=0.00001,
        name="speed knob is seated on top controls",
    )
    ctx.expect_gap(
        rotor,
        column,
        axis="y",
        positive_elem="blower_wheel",
        negative_elem="front_grille",
        min_gap=0.010,
        max_gap=0.040,
        name="internal blower sits behind front grille",
    )
    ctx.expect_overlap(
        rotor,
        column,
        axes="xz",
        elem_a="blower_wheel",
        elem_b="front_grille",
        min_overlap=0.080,
        name="blower wheel sits in the grille opening",
    )
    ctx.expect_overlap(
        rotor,
        column,
        axes="z",
        elem_a="blower_wheel",
        elem_b="front_grille",
        min_overlap=0.65,
        name="blower wheel spans the tall vent opening",
    )

    def _aabb_center_x(aabb):
        return 0.5 * (aabb[0][0] + aabb[1][0])

    rest = ctx.part_element_world_aabb(column, elem="front_grille")
    with ctx.pose({oscillation: 0.65}):
        swept = ctx.part_element_world_aabb(column, elem="front_grille")
    ctx.check(
        "positive oscillation visibly swings the front grille sideways",
        rest is not None and swept is not None and _aabb_center_x(swept) > _aabb_center_x(rest) + 0.025,
        details=f"rest={rest}, swept={swept}",
    )

    return ctx.report()


object_model = build_object_model()
