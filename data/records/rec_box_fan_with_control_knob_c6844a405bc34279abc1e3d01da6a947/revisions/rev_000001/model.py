from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _circle_profile(radius: float, segments: int = 72) -> list[tuple[float, float]]:
    return [
        (radius * math.cos(2.0 * math.pi * i / segments), radius * math.sin(2.0 * math.pi * i / segments))
        for i in range(segments)
    ]


def _cylinder_origin_for_xz_segment(
    start_radius: float,
    end_radius: float,
    angle: float,
    *,
    y: float = 0.0,
) -> Origin:
    """Origin for a cylinder whose local Z axis spans a radial segment in the XZ plane."""
    mid_radius = 0.5 * (start_radius + end_radius)
    return Origin(
        xyz=(mid_radius * math.cos(angle), y, mid_radius * math.sin(angle)),
        rpy=(0.0, math.pi / 2.0 - angle, 0.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="square_box_fan")

    warm_white = Material("warm_white_plastic", rgba=(0.84, 0.82, 0.74, 1.0))
    dark_plastic = Material("dark_grey_plastic", rgba=(0.04, 0.045, 0.05, 1.0))
    black_wire = Material("black_powder_coated_wire", rgba=(0.015, 0.017, 0.018, 1.0))
    blade_plastic = Material("smoky_blue_blade_plastic", rgba=(0.38, 0.53, 0.66, 0.82))
    label_white = Material("white_speed_marks", rgba=(0.93, 0.92, 0.86, 1.0))

    housing = model.part("housing")

    outer = 0.50
    border = 0.050
    depth = 0.120
    half_outer = outer / 2.0
    side_center = half_outer - border / 2.0

    # Flat square molded housing: four deep side rails plus a continuous front face
    # with a large circular fan opening.
    housing.visual(
        Box((border, depth, outer)),
        origin=Origin(xyz=(-side_center, 0.0, 0.0)),
        material=warm_white,
        name="side_rail_0",
    )
    housing.visual(
        Box((border, depth, outer)),
        origin=Origin(xyz=(side_center, 0.0, 0.0)),
        material=warm_white,
        name="side_rail_1",
    )
    housing.visual(
        Box((outer, depth, border)),
        origin=Origin(xyz=(0.0, 0.0, side_center)),
        material=warm_white,
        name="top_rail",
    )
    housing.visual(
        Box((outer, depth, border)),
        origin=Origin(xyz=(0.0, 0.0, -side_center)),
        material=warm_white,
        name="bottom_rail",
    )

    front_face = ExtrudeWithHolesGeometry(
        rounded_rect_profile(outer, outer, radius=0.028, corner_segments=10),
        [_circle_profile(0.205, 96)],
        0.012,
        center=False,
    )
    housing.visual(
        mesh_from_geometry(front_face, "front_face"),
        origin=Origin(xyz=(0.0, -0.060, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=warm_white,
        name="front_face",
    )

    # Rounded circular lip around the blade opening.
    housing.visual(
        mesh_from_geometry(TorusGeometry(0.208, 0.007, radial_segments=14, tubular_segments=96), "front_lip"),
        origin=Origin(xyz=(0.0, -0.057, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=warm_white,
        name="front_lip",
    )

    # Rear motor pod, bearing shaft, and four molded struts supporting the center.
    housing.visual(
        Cylinder(0.058, 0.042),
        origin=Origin(xyz=(0.0, 0.035, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="motor_boss",
    )
    housing.visual(
        Cylinder(0.006, 0.092),
        origin=Origin(xyz=(0.0, -0.004, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="axle_pin",
    )
    for i, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        housing.visual(
            Cylinder(0.008, 0.173),
            origin=_cylinder_origin_for_xz_segment(0.052, 0.225, angle, y=0.034),
            material=dark_plastic,
            name=f"motor_strut_{i}",
        )

    # Small speed scale printed around the side-mounted rotary knob.
    knob_center = (half_outer, 0.0, 0.145)
    for i, angle in enumerate((-0.95, -0.30, 0.35, 1.00)):
        mark_y = knob_center[1] + 0.040 * math.cos(angle)
        mark_z = knob_center[2] + 0.040 * math.sin(angle)
        housing.visual(
            Cylinder(0.0032, 0.002),
            origin=Origin(xyz=(half_outer + 0.001, mark_y, mark_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=label_white,
            name=f"speed_mark_{i}",
        )

    # Front wire guard: separate fixed grille, held to the front face by four tabs.
    grille = model.part("front_grille")
    for idx, radius in enumerate((0.045, 0.090, 0.135, 0.180, 0.218)):
        grille.visual(
            mesh_from_geometry(
                TorusGeometry(radius, 0.0025 if idx < 4 else 0.0032, radial_segments=10, tubular_segments=96),
                f"grille_ring_{idx}",
            ),
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=black_wire,
            name=f"ring_{idx}",
        )

    for i in range(16):
        angle = 2.0 * math.pi * i / 16
        grille.visual(
            Cylinder(0.0021, 0.405),
            origin=_cylinder_origin_for_xz_segment(0.020, 0.425, angle),
            material=black_wire,
            name=f"radial_wire_{i}",
        )

    for tab_name, angle in zip(
        ("mount_tab_0", "mount_tab_1", "mount_tab_2", "mount_tab_3"),
        (0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0),
    ):
        grille.visual(
            Cylinder(0.004, 0.032),
            origin=_cylinder_origin_for_xz_segment(0.218, 0.250, angle),
            material=black_wire,
            name=tab_name,
        )

    grille.visual(
        Cylinder(0.032, 0.006),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black_wire,
        name="center_badge",
    )

    model.articulation(
        "housing_to_grille",
        ArticulationType.FIXED,
        parent=housing,
        child=grille,
        origin=Origin(xyz=(0.0, -0.064, 0.0)),
    )

    # Multi-blade rotor on the central continuous axle.
    rotor = model.part("rotor")
    rotor_mesh = FanRotorGeometry(
        0.178,
        0.044,
        5,
        thickness=0.030,
        blade_pitch_deg=31.0,
        blade_sweep_deg=24.0,
        blade=FanRotorBlade(shape="broad", tip_pitch_deg=13.0, camber=0.12, tip_clearance=0.006),
        hub=FanRotorHub(style="capped", rear_collar_height=0.006, rear_collar_radius=0.030, bore_diameter=0.012),
    )
    rotor.visual(
        mesh_from_geometry(rotor_mesh, "fan_rotor"),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=blade_plastic,
        name="fan_rotor",
    )
    model.articulation(
        "fan_axle",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=rotor,
        origin=Origin(xyz=(0.0, -0.015, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=45.0),
    )

    # Side rotary speed knob with ribbed grip and a raised pointer indicator.
    speed_knob = model.part("speed_knob")
    knob_mesh = KnobGeometry(
        0.045,
        0.024,
        body_style="cylindrical",
        edge_radius=0.0012,
        grip=KnobGrip(style="ribbed", count=18, depth=0.0011, width=0.0016),
        indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
        center=False,
    )
    speed_knob.visual(
        mesh_from_geometry(knob_mesh, "speed_knob"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="knob_cap",
    )
    model.articulation(
        "speed_selector",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=speed_knob,
        origin=Origin(xyz=knob_center),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=4.0, lower=0.0, upper=4.70),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    grille = object_model.get_part("front_grille")
    rotor = object_model.get_part("rotor")
    speed_knob = object_model.get_part("speed_knob")
    fan_axle = object_model.get_articulation("fan_axle")
    speed_selector = object_model.get_articulation("speed_selector")

    ctx.allow_overlap(
        housing,
        rotor,
        elem_a="axle_pin",
        elem_b="fan_rotor",
        reason="The central metal axle is intentionally captured inside the simplified rotor hub bore.",
    )

    ctx.check(
        "rotor has continuous central axle",
        fan_axle.articulation_type == ArticulationType.CONTINUOUS,
        details=f"fan_axle type={fan_axle.articulation_type}",
    )
    ctx.check(
        "speed knob is limited rotary control",
        speed_selector.articulation_type == ArticulationType.REVOLUTE
        and speed_selector.motion_limits is not None
        and speed_selector.motion_limits.upper is not None
        and speed_selector.motion_limits.upper > 4.0,
        details=f"speed_selector={speed_selector}",
    )

    ctx.expect_within(
        rotor,
        housing,
        axes="xz",
        margin=0.002,
        elem_a="fan_rotor",
        elem_b="front_face",
        name="fan rotor fits inside square housing opening",
    )
    ctx.expect_overlap(
        housing,
        rotor,
        axes="y",
        min_overlap=0.020,
        elem_a="axle_pin",
        elem_b="fan_rotor",
        name="axle pin remains captured through rotor hub",
    )
    ctx.expect_gap(
        rotor,
        grille,
        axis="y",
        min_gap=0.020,
        max_gap=0.050,
        positive_elem="fan_rotor",
        negative_elem="center_badge",
        name="front grille stands in front of spinning rotor",
    )
    ctx.expect_gap(
        housing,
        grille,
        axis="y",
        max_gap=0.002,
        max_penetration=0.0005,
        positive_elem="front_face",
        negative_elem="mount_tab_0",
        name="grille mounting tabs touch front face",
    )
    ctx.expect_gap(
        speed_knob,
        housing,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="knob_cap",
        negative_elem="side_rail_1",
        name="rotary speed knob is seated on side panel",
    )

    rest_pos = ctx.part_world_position(rotor)
    with ctx.pose({fan_axle: 1.25, speed_selector: 2.2}):
        moved_pos = ctx.part_world_position(rotor)
        ctx.expect_within(
            rotor,
            housing,
            axes="xz",
            margin=0.002,
            elem_a="fan_rotor",
            elem_b="front_face",
            name="spinning rotor remains centered in opening",
        )
    ctx.check(
        "fan axle spin keeps rotor on bearing center",
        rest_pos is not None and moved_pos is not None and abs(rest_pos[1] - moved_pos[1]) < 1e-6,
        details=f"rest={rest_pos}, moved={moved_pos}",
    )

    return ctx.report()


object_model = build_object_model()
