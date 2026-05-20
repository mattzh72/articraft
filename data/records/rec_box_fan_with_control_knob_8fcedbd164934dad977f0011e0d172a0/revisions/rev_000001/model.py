from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    ExtrudeWithHolesGeometry,
    tube_from_spline_points,
)


def _circle_points(radius: float, segments: int = 72) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * i / segments),
            radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _radial_origin(angle: float, radius: float, y: float) -> Origin:
    """Origin for a cylinder whose local Z axis points radially in the XZ plane."""
    return Origin(
        xyz=(radius * math.cos(angle), y, radius * math.sin(angle)),
        rpy=(0.0, math.pi / 2.0 - angle, 0.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vintage_box_fan")

    ivory = Material("aged_ivory_plastic", rgba=(0.78, 0.70, 0.55, 1.0))
    rib_shadow = Material("rib_shadow", rgba=(0.55, 0.49, 0.38, 1.0))
    chrome = Material("polished_chrome", rgba=(0.86, 0.86, 0.82, 1.0))
    dark_chrome = Material("darkened_chrome", rgba=(0.40, 0.41, 0.40, 1.0))
    blade_mat = Material("warm_gray_blades", rgba=(0.62, 0.62, 0.58, 1.0))
    bakelite = Material("dark_brown_bakelite", rgba=(0.16, 0.075, 0.035, 1.0))
    black = Material("black_control_marks", rgba=(0.02, 0.018, 0.015, 1.0))

    housing = model.part("housing")

    # A continuous rounded front plastic bezel with a real circular opening.
    front_frame = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.58, 0.58, 0.045, corner_segments=10),
        [_circle_points(0.235, segments=96)],
        0.038,
        cap=True,
        center=True,
    )
    housing.visual(
        mesh_from_geometry(front_frame, "front_frame"),
        origin=Origin(xyz=(0.0, 0.086, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=ivory,
        name="front_frame",
    )

    # Box-fan side depth and rear frame members.
    housing.visual(
        Box((0.060, 0.175, 0.545)),
        origin=Origin(xyz=(-0.260, -0.006, 0.0)),
        material=ivory,
        name="side_wall_0",
    )
    housing.visual(
        Box((0.060, 0.175, 0.545)),
        origin=Origin(xyz=(0.260, -0.006, 0.0)),
        material=ivory,
        name="side_wall_1",
    )
    housing.visual(
        Box((0.550, 0.175, 0.060)),
        origin=Origin(xyz=(0.0, -0.006, 0.260)),
        material=ivory,
        name="top_wall",
    )
    housing.visual(
        Box((0.550, 0.175, 0.060)),
        origin=Origin(xyz=(0.0, -0.006, -0.260)),
        material=ivory,
        name="bottom_wall",
    )

    # Raised ribs and darker grooves on the front plastic housing.
    for idx, z in enumerate((-0.278, -0.264, -0.250, 0.250, 0.264, 0.278)):
        housing.visual(
            Box((0.455, 0.010, 0.006)),
            origin=Origin(xyz=(0.0, 0.109, z)),
            material=rib_shadow if idx in (1, 4) else ivory,
            name=f"horizontal_rib_{idx}",
        )
    for idx, x in enumerate((-0.278, -0.264, -0.250, 0.250, 0.264, 0.278)):
        housing.visual(
            Box((0.006, 0.010, 0.430)),
            origin=Origin(xyz=(x, 0.109, 0.0)),
            material=rib_shadow if idx in (1, 4) else ivory,
            name=f"vertical_rib_{idx}",
        )

    # Small screw bosses in the molded corners.
    for idx, (x, z) in enumerate(
        ((-0.235, -0.235), (-0.235, 0.235), (0.235, -0.235), (0.235, 0.235))
    ):
        housing.visual(
            Cylinder(radius=0.015, length=0.006),
            origin=Origin(xyz=(x, 0.111, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_chrome,
            name=f"corner_screw_{idx}",
        )

    # Two squat molded feet are slightly keyed into the lower housing rail.
    for idx, x in enumerate((-0.170, 0.170)):
        housing.visual(
            Box((0.135, 0.155, 0.040)),
            origin=Origin(xyz=(x, -0.010, -0.302)),
            material=ivory,
            name=f"foot_{idx}",
        )

    # Arched carry handle molded into the top, overlapping the top rail at its feet.
    handle_geom = tube_from_spline_points(
        [
            (-0.135, 0.000, 0.292),
            (-0.090, 0.000, 0.333),
            (0.000, 0.000, 0.348),
            (0.090, 0.000, 0.333),
            (0.135, 0.000, 0.292),
        ],
        radius=0.012,
        samples_per_segment=14,
        radial_segments=18,
        cap_ends=True,
    )
    housing.visual(
        mesh_from_geometry(handle_geom, "carry_handle"),
        material=ivory,
        name="carry_handle",
    )

    # Rear motor cowling and four pressed-metal support struts.
    housing.visual(
        Cylinder(radius=0.092, length=0.082),
        origin=Origin(xyz=(0.0, -0.048, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=ivory,
        name="motor_cowl",
    )
    housing.visual(
        Cylinder(radius=0.060, length=0.010),
        origin=Origin(xyz=(0.0, -0.004, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_chrome,
        name="front_bearing",
    )
    for idx, angle in enumerate((math.pi / 4.0, 3.0 * math.pi / 4.0, 5.0 * math.pi / 4.0, 7.0 * math.pi / 4.0)):
        mid = (0.080 + 0.325) / 2.0
        housing.visual(
            Cylinder(radius=0.0075, length=0.325 - 0.080),
            origin=_radial_origin(angle, mid, -0.018),
            material=ivory,
            name=f"motor_strut_{idx}",
        )

    # Chromed wire front grille: concentric wire rings, radial spokes, and short
    # stand-offs that visibly fasten it to the plastic frame.
    grille_y = 0.119
    grille_rings = (
        ("grille_ring_0", 0.245),
        ("grille_ring_1", 0.212),
        ("grille_ring_2", 0.178),
        ("grille_ring_3", 0.144),
        ("grille_ring_4", 0.110),
        ("grille_ring_5", 0.076),
        ("grille_ring_6", 0.044),
    )
    for ring_name, radius in grille_rings:
        housing.visual(
            mesh_from_geometry(
                TorusGeometry(radius=radius, tube=0.0024, radial_segments=18, tubular_segments=96),
                ring_name,
            ),
            origin=Origin(xyz=(0.0, grille_y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=chrome,
            name=ring_name,
        )
    for idx in range(16):
        angle = 2.0 * math.pi * idx / 16.0
        mid = (0.028 + 0.245) / 2.0
        housing.visual(
            Cylinder(radius=0.0022, length=0.245 - 0.028),
            origin=_radial_origin(angle, mid, grille_y),
            material=chrome,
            name=f"grille_spoke_{idx}",
        )
    for idx, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        housing.visual(
            Cylinder(radius=0.0055, length=0.033),
            origin=Origin(
                xyz=(0.246 * math.cos(angle), 0.104, 0.246 * math.sin(angle)),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=chrome,
            name=f"grille_standoff_{idx}",
        )
    housing.visual(
        Cylinder(radius=0.030, length=0.008),
        origin=Origin(xyz=(0.0, 0.124, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_chrome,
        name="center_badge",
    )

    # Control collar and simple speed index marks on the top rail.
    knob_x = 0.205
    housing.visual(
        Cylinder(radius=0.035, length=0.014),
        origin=Origin(xyz=(knob_x, 0.000, 0.297)),
        material=ivory,
        name="knob_collar",
    )
    for idx, angle in enumerate((-0.9, -0.3, 0.3, 0.9)):
        housing.visual(
            Box((0.004, 0.020, 0.001)),
            origin=Origin(
                xyz=(knob_x + 0.058 * math.sin(angle), 0.033 * math.cos(angle), 0.2905),
                rpy=(0.0, 0.0, -angle),
            ),
            material=black,
            name=f"speed_mark_{idx}",
        )

    propeller = model.part("propeller")
    propeller.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                0.205,
                0.052,
                4,
                thickness=0.030,
                blade_pitch_deg=32.0,
                blade_sweep_deg=18.0,
                blade=FanRotorBlade(shape="broad", tip_pitch_deg=16.0, camber=0.11, tip_clearance=0.004),
                hub=FanRotorHub(style="domed", rear_collar_height=0.016, rear_collar_radius=0.034, bore_diameter=0.010),
                center=True,
            ),
            "propeller_blades",
        ),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=blade_mat,
        name="propeller_blades",
    )
    propeller.visual(
        Cylinder(radius=0.014, length=0.0062),
        origin=Origin(xyz=(0.0, -0.0181, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_chrome,
        name="motor_shaft",
    )

    model.articulation(
        "propeller_spin",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=propeller,
        origin=Origin(xyz=(0.0, 0.022, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=65.0),
    )

    speed_knob = model.part("speed_knob")
    speed_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.074,
                0.044,
                body_style="skirted",
                top_diameter=0.058,
                base_diameter=0.078,
                crown_radius=0.004,
                edge_radius=0.002,
                skirt=KnobSkirt(0.086, 0.010, flare=0.10, chamfer=0.0015),
                grip=KnobGrip(style="ribbed", count=24, depth=0.0016, width=0.0025),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008),
                bore=KnobBore(style="d_shaft", diameter=0.008, flat_depth=0.0014),
                center=False,
            ),
            "bakelite_knob",
        ),
        material=bakelite,
        name="bakelite_knob",
    )

    model.articulation(
        "speed_select",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=speed_knob,
        origin=Origin(xyz=(knob_x, 0.000, 0.304)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=0.0, upper=4.712),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    propeller = object_model.get_part("propeller")
    speed_knob = object_model.get_part("speed_knob")
    propeller_spin = object_model.get_articulation("propeller_spin")
    speed_select = object_model.get_articulation("speed_select")

    ctx.check(
        "propeller has continuous central spin",
        propeller_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={propeller_spin.articulation_type}",
    )
    ctx.check(
        "speed knob has limited rotary travel",
        speed_select.motion_limits is not None
        and speed_select.motion_limits.lower == 0.0
        and speed_select.motion_limits.upper is not None
        and speed_select.motion_limits.upper > 4.0,
        details=f"limits={speed_select.motion_limits}",
    )

    ctx.expect_within(
        propeller,
        housing,
        axes="xz",
        inner_elem="propeller_blades",
        outer_elem="grille_ring_0",
        margin=0.0,
        name="four blade propeller fits within chrome grille",
    )
    ctx.expect_gap(
        housing,
        propeller,
        axis="y",
        positive_elem="grille_ring_0",
        negative_elem="propeller_blades",
        min_gap=0.035,
        name="front grille stands in front of propeller",
    )
    ctx.expect_gap(
        speed_knob,
        housing,
        axis="z",
        positive_elem="bakelite_knob",
        negative_elem="knob_collar",
        min_gap=0.0,
        max_gap=0.002,
        max_penetration=0.0,
        name="bakelite speed knob sits on top collar",
    )

    with ctx.pose({propeller_spin: math.pi / 2.0, speed_select: 2.5}):
        ctx.expect_within(
            propeller,
            housing,
            axes="xz",
            inner_elem="propeller_blades",
            outer_elem="grille_ring_0",
            margin=0.0,
            name="spun propeller remains within grille",
        )

    return ctx.report()


object_model = build_object_model()
