from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireGroove,
    TireSidewall,
    TireShoulder,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    superellipse_side_loft,
    tube_from_spline_points,
)


WHEEL_RADIUS = 0.180
TIRE_WIDTH = 0.056


def _curved_fender_geometry(
    *,
    width: float,
    inner_radius: float,
    outer_radius: float,
    center_y: float,
    center_z: float,
    start_angle: float,
    end_angle: float,
    segments: int = 28,
) -> MeshGeometry:
    """Build a solid thin curved fender plate extruded across local X."""
    geom = MeshGeometry()
    angles = [
        start_angle + (end_angle - start_angle) * i / segments
        for i in range(segments + 1)
    ]
    xs = (-width * 0.5, width * 0.5)
    radii = (inner_radius, outer_radius)

    def vid(i: int, side: int, radial: int) -> int:
        return (i * 4) + (side * 2) + radial

    for angle in angles:
        for x in xs:
            for radius in radii:
                geom.add_vertex(
                    x,
                    center_y + radius * sin(angle),
                    center_z + radius * cos(angle),
                )

    def quad(a: int, b: int, c: int, d: int) -> None:
        geom.add_face(a, b, c)
        geom.add_face(a, c, d)

    for i in range(segments):
        # Outer and inner curved skins.
        quad(vid(i, 0, 1), vid(i + 1, 0, 1), vid(i + 1, 1, 1), vid(i, 1, 1))
        quad(vid(i, 1, 0), vid(i + 1, 1, 0), vid(i + 1, 0, 0), vid(i, 0, 0))
        # Left and right side walls.
        quad(vid(i, 0, 0), vid(i + 1, 0, 0), vid(i + 1, 0, 1), vid(i, 0, 1))
        quad(vid(i, 1, 1), vid(i + 1, 1, 1), vid(i + 1, 1, 0), vid(i, 1, 0))

    # End caps.
    quad(vid(0, 0, 0), vid(0, 0, 1), vid(0, 1, 1), vid(0, 1, 0))
    quad(
        vid(segments, 1, 0),
        vid(segments, 1, 1),
        vid(segments, 0, 1),
        vid(segments, 0, 0),
    )
    return geom


def _add_scooter_wheel(part, prefix: str, *, rim_material, tire_material) -> None:
    part.visual(
        mesh_from_geometry(
            WheelGeometry(
                0.128,
                0.042,
                rim=WheelRim(
                    inner_radius=0.086,
                    flange_height=0.008,
                    flange_thickness=0.0035,
                    bead_seat_depth=0.003,
                ),
                hub=WheelHub(
                    radius=0.030,
                    width=0.047,
                    cap_style="domed",
                    bolt_pattern=BoltPattern(
                        count=6,
                        circle_diameter=0.036,
                        hole_diameter=0.0035,
                    ),
                ),
                face=WheelFace(dish_depth=0.005, front_inset=0.002, rear_inset=0.002),
                spokes=WheelSpokes(
                    style="straight",
                    count=8,
                    thickness=0.0035,
                    window_radius=0.010,
                ),
                bore=WheelBore(style="round", diameter=0.014),
            ),
            f"{prefix}_rim",
        ),
        material=rim_material,
        name="rim",
    )
    part.visual(
        mesh_from_geometry(
            TireGeometry(
                WHEEL_RADIUS,
                TIRE_WIDTH,
                inner_radius=0.132,
                carcass=None,
                tread=TireTread(style="circumferential", depth=0.004, count=5),
                grooves=(
                    TireGroove(center_offset=0.0, width=0.004, depth=0.002),
                    TireGroove(center_offset=-0.014, width=0.003, depth=0.0015),
                    TireGroove(center_offset=0.014, width=0.003, depth=0.0015),
                ),
                sidewall=TireSidewall(style="rounded", bulge=0.045),
                shoulder=TireShoulder(width=0.006, radius=0.003),
            ),
            f"{prefix}_tire",
        ),
        material=tire_material,
        name="tire",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="big_wheel_commuter_kick_scooter")

    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.68, 0.70, 0.72, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.10, 0.11, 0.12, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.035, 0.035, 0.038, 1.0))
    tire_rubber = model.material("tire_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    rim_silver = model.material("rim_silver", rgba=(0.82, 0.84, 0.86, 1.0))
    satin_red = model.material("satin_red", rgba=(0.70, 0.05, 0.035, 1.0))
    spring_metal = model.material("spring_metal", rgba=(0.86, 0.87, 0.86, 1.0))

    deck = model.part("deck")
    deck.visual(
        mesh_from_geometry(
            superellipse_side_loft(
                [
                    (-0.405, 0.218, 0.268, 0.155),
                    (-0.300, 0.212, 0.274, 0.190),
                    (-0.050, 0.210, 0.277, 0.198),
                    (0.235, 0.213, 0.276, 0.190),
                    (0.445, 0.220, 0.272, 0.160),
                ],
                exponents=3.2,
                segments=56,
            ),
            "long_deck_shell",
        ),
        material=brushed_aluminum,
        name="deck_shell",
    )
    deck.visual(
        Box((0.145, 0.670, 0.006)),
        origin=Origin(xyz=(0.0, 0.000, 0.279)),
        material=black_plastic,
        name="grip_tape",
    )
    deck.visual(
        Box((0.014, 0.770, 0.024)),
        origin=Origin(xyz=(0.095, 0.020, 0.246)),
        material=dark_metal,
        name="edge_rail_0",
    )
    deck.visual(
        Box((0.014, 0.770, 0.024)),
        origin=Origin(xyz=(-0.095, 0.020, 0.246)),
        material=dark_metal,
        name="edge_rail_1",
    )
    # Rear dropout plates support the rear wheel without intersecting the tire.
    deck.visual(
        Box((0.020, 0.240, 0.072)),
        origin=Origin(xyz=(0.056, -0.505, 0.205)),
        material=dark_metal,
        name="rear_axle_plate_0",
    )
    deck.visual(
        Box((0.020, 0.240, 0.072)),
        origin=Origin(xyz=(-0.056, -0.505, 0.205)),
        material=dark_metal,
        name="rear_axle_plate_1",
    )
    # Steering-head side lugs leave a clear center gap for the rotating column.
    deck.visual(
        Box((0.026, 0.105, 0.105)),
        origin=Origin(xyz=(0.037, 0.468, 0.287)),
        material=dark_metal,
        name="head_lug_0",
    )
    deck.visual(
        Box((0.026, 0.105, 0.105)),
        origin=Origin(xyz=(-0.037, 0.468, 0.287)),
        material=dark_metal,
        name="head_lug_1",
    )
    # Rear fender-brake hinge lugs.
    deck.visual(
        Box((0.024, 0.038, 0.056)),
        origin=Origin(xyz=(0.082, -0.430, 0.320)),
        material=dark_metal,
        name="brake_hinge_lug_0",
    )
    deck.visual(
        Box((0.024, 0.038, 0.056)),
        origin=Origin(xyz=(-0.082, -0.430, 0.320)),
        material=dark_metal,
        name="brake_hinge_lug_1",
    )
    deck.visual(
        Box((0.026, 0.070, 0.048)),
        origin=Origin(xyz=(0.075, -0.395, 0.288)),
        material=dark_metal,
        name="brake_lug_strut_0",
    )
    deck.visual(
        Box((0.026, 0.070, 0.048)),
        origin=Origin(xyz=(-0.075, -0.395, 0.288)),
        material=dark_metal,
        name="brake_lug_strut_1",
    )
    deck.visual(
        Cylinder(radius=0.006, length=0.150),
        origin=Origin(xyz=(0.0, -0.600, 0.180), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
        name="rear_axle",
    )
    deck.visual(
        Cylinder(radius=0.025, length=0.006),
        origin=Origin(xyz=(0.0395, -0.600, 0.180), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
        name="rear_axle_washer_0",
    )
    deck.visual(
        Cylinder(radius=0.025, length=0.006),
        origin=Origin(xyz=(-0.031, -0.600, 0.180), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
        name="rear_axle_washer_1",
    )
    # Side kickstand clevis under the deck.
    deck.visual(
        Box((0.040, 0.090, 0.012)),
        origin=Origin(xyz=(-0.106, -0.100, 0.218)),
        material=dark_metal,
        name="kickstand_mount",
    )
    deck.visual(
        Box((0.034, 0.012, 0.032)),
        origin=Origin(xyz=(-0.106, -0.130, 0.198)),
        material=dark_metal,
        name="kickstand_cheek_0",
    )
    deck.visual(
        Box((0.034, 0.012, 0.032)),
        origin=Origin(xyz=(-0.106, -0.070, 0.198)),
        material=dark_metal,
        name="kickstand_cheek_1",
    )

    front_assembly = model.part("front_assembly")
    front_assembly.visual(
        Cylinder(radius=0.024, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=dark_metal,
        name="steer_bearing",
    )
    front_assembly.visual(
        Cylinder(radius=0.016, length=0.730),
        origin=Origin(xyz=(0.0, 0.0, 0.390)),
        material=brushed_aluminum,
        name="steering_column",
    )
    front_assembly.visual(
        Cylinder(radius=0.018, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.765)),
        material=dark_metal,
        name="handlebar_clamp",
    )
    front_assembly.visual(
        Cylinder(radius=0.013, length=0.560),
        origin=Origin(xyz=(0.0, 0.0, 0.790), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
        name="handlebar",
    )
    front_assembly.visual(
        Cylinder(radius=0.019, length=0.120),
        origin=Origin(xyz=(0.315, 0.0, 0.790), rpy=(0.0, pi / 2.0, 0.0)),
        material=black_plastic,
        name="grip_0",
    )
    front_assembly.visual(
        Cylinder(radius=0.019, length=0.120),
        origin=Origin(xyz=(-0.315, 0.0, 0.790), rpy=(0.0, pi / 2.0, 0.0)),
        material=black_plastic,
        name="grip_1",
    )
    front_assembly.visual(
        Box((0.118, 0.045, 0.028)),
        origin=Origin(xyz=(0.0, 0.035, 0.055)),
        material=dark_metal,
        name="fork_crown",
    )
    for side, x, fork_leg_name, slider_name, lower_name, cap_name in (
        ("left", -0.052, "left_fork_leg", "left_suspension_slider", "left_lower_fork", "left_axle_cap"),
        ("right", 0.052, "right_fork_leg", "right_suspension_slider", "right_lower_fork", "right_axle_cap"),
    ):
        front_assembly.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    [
                        (x, 0.055, 0.050),
                        (x, 0.112, -0.065),
                        (x, 0.205, -0.125),
                    ],
                    radius=0.010,
                    samples_per_segment=12,
                    radial_segments=18,
                    cap_ends=True,
                ),
                f"{side}_front_fork_leg",
            ),
            material=dark_metal,
            name=fork_leg_name,
        )
        front_assembly.visual(
            Cylinder(radius=0.014, length=0.092),
            origin=Origin(xyz=(x, 0.108, -0.055)),
            material=spring_metal,
            name=slider_name,
        )
        front_assembly.visual(
            Cylinder(radius=0.016, length=0.062),
            origin=Origin(xyz=(x, 0.174, -0.106)),
            material=satin_red,
            name=lower_name,
        )
        front_assembly.visual(
            Cylinder(radius=0.010, length=0.026),
            origin=Origin(xyz=(x, 0.205, -0.125), rpy=(0.0, pi / 2.0, 0.0)),
            material=dark_metal,
            name=cap_name,
        )
    front_assembly.visual(
        Cylinder(radius=0.025, length=0.006),
        origin=Origin(xyz=(0.0395, 0.205, -0.125), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
        name="front_axle_washer_0",
    )
    front_assembly.visual(
        Cylinder(radius=0.025, length=0.010),
        origin=Origin(xyz=(-0.033, 0.205, -0.125), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
        name="front_axle_washer_1",
    )

    front_wheel = model.part("front_wheel")
    _add_scooter_wheel(front_wheel, "front_wheel", rim_material=rim_silver, tire_material=tire_rubber)

    rear_wheel = model.part("rear_wheel")
    _add_scooter_wheel(rear_wheel, "rear_wheel", rim_material=rim_silver, tire_material=tire_rubber)

    rear_brake = model.part("rear_fender_brake")
    rear_brake.visual(
        Cylinder(radius=0.012, length=0.140),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
        name="hinge_barrel",
    )
    rear_brake.visual(
        Box((0.105, 0.080, 0.035)),
        origin=Origin(xyz=(0.0, 0.010, 0.000)),
        material=satin_red,
        name="hinge_tang",
    )
    rear_brake.visual(
        mesh_from_geometry(
            _curved_fender_geometry(
                width=0.120,
                inner_radius=0.190,
                outer_radius=0.212,
                center_y=-0.170,
                center_z=-0.140,
                start_angle=0.84,
                end_angle=-1.25,
                segments=32,
            ),
            "rear_fender_brake_shell",
        ),
        material=satin_red,
        name="fender_shell",
    )
    kickstand = model.part("kickstand")
    kickstand.visual(
        Cylinder(radius=0.011, length=0.048),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="hinge_barrel",
    )
    kickstand.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.0, 0.0, 0.0),
                    (-0.060, -0.018, -0.095),
                    (-0.118, -0.030, -0.205),
                ],
                radius=0.008,
                samples_per_segment=14,
                radial_segments=16,
                cap_ends=True,
            ),
            "kickstand_leg_tube",
        ),
        material=dark_metal,
        name="stand_leg",
    )
    kickstand.visual(
        Box((0.070, 0.030, 0.012)),
        origin=Origin(xyz=(-0.145, -0.034, -0.195)),
        material=black_plastic,
        name="foot_pad",
    )

    model.articulation(
        "steering_yaw",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=front_assembly,
        origin=Origin(xyz=(0.0, 0.500, 0.300)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=3.0, lower=-0.65, upper=0.65),
    )
    model.articulation(
        "front_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=front_assembly,
        child=front_wheel,
        origin=Origin(xyz=(0.0, 0.205, -0.125)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=28.0),
    )
    model.articulation(
        "rear_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=rear_wheel,
        origin=Origin(xyz=(0.0, -0.600, 0.180)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=28.0),
    )
    model.articulation(
        "rear_brake_hinge",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=rear_brake,
        origin=Origin(xyz=(0.0, -0.430, 0.320)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=0.0, upper=0.90),
    )
    model.articulation(
        "kickstand_hinge",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=kickstand,
        origin=Origin(xyz=(-0.106, -0.100, 0.198)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.5, lower=0.0, upper=1.45),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    front = object_model.get_part("front_assembly")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")
    rear_brake = object_model.get_part("rear_fender_brake")
    kickstand = object_model.get_part("kickstand")
    steer = object_model.get_articulation("steering_yaw")
    brake_hinge = object_model.get_articulation("rear_brake_hinge")
    stand_hinge = object_model.get_articulation("kickstand_hinge")

    ctx.expect_gap(
        front_wheel,
        front,
        axis="x",
        positive_elem="tire",
        negative_elem="left_fork_leg",
        min_gap=0.006,
        max_gap=0.040,
        name="front tire clears left fork leg",
    )
    ctx.expect_gap(
        front,
        front_wheel,
        axis="x",
        positive_elem="right_fork_leg",
        negative_elem="tire",
        min_gap=0.006,
        max_gap=0.040,
        name="front tire clears right fork leg",
    )
    ctx.expect_overlap(
        front,
        front_wheel,
        axes="yz",
        elem_a="left_fork_leg",
        elem_b="tire",
        min_overlap=0.050,
        name="left fork leg straddles front wheel",
    )
    ctx.expect_overlap(
        front,
        front_wheel,
        axes="yz",
        elem_a="right_fork_leg",
        elem_b="tire",
        min_overlap=0.050,
        name="right fork leg straddles front wheel",
    )
    ctx.expect_overlap(
        rear_brake,
        rear_wheel,
        axes="xy",
        elem_a="fender_shell",
        elem_b="tire",
        min_overlap=0.050,
        name="rear fender brake covers wheel tread",
    )
    ctx.expect_gap(
        deck,
        kickstand,
        axis="x",
        positive_elem="deck_shell",
        negative_elem="foot_pad",
        min_gap=0.050,
        name="deployed kickstand foot sits outboard of deck",
    )

    rest_front_pos = ctx.part_world_position(front_wheel)
    with ctx.pose({steer: 0.55}):
        yawed_front_pos = ctx.part_world_position(front_wheel)
        ctx.expect_overlap(
            front,
            front_wheel,
            axes="yz",
            elem_a="left_fork_leg",
            elem_b="tire",
            min_overlap=0.030,
            name="yawed fork still carries front wheel",
        )
    ctx.check(
        "front wheel is carried by steering yaw",
        rest_front_pos is not None
        and yawed_front_pos is not None
        and abs(yawed_front_pos[0] - rest_front_pos[0]) > 0.055,
        details=f"rest={rest_front_pos}, yawed={yawed_front_pos}",
    )

    closed_brake_aabb = ctx.part_world_aabb(rear_brake)
    with ctx.pose({brake_hinge: 0.85}):
        raised_brake_aabb = ctx.part_world_aabb(rear_brake)
    ctx.check(
        "rear fender brake rotates upward",
        closed_brake_aabb is not None
        and raised_brake_aabb is not None
        and raised_brake_aabb[1][2] > closed_brake_aabb[1][2] + 0.070,
        details=f"closed={closed_brake_aabb}, raised={raised_brake_aabb}",
    )

    deployed_foot = ctx.part_element_world_aabb(kickstand, elem="foot_pad")
    with ctx.pose({stand_hinge: 1.45}):
        stowed_foot = ctx.part_element_world_aabb(kickstand, elem="foot_pad")
    ctx.check(
        "kickstand folds upward from underside",
        deployed_foot is not None
        and stowed_foot is not None
        and stowed_foot[0][2] > deployed_foot[0][2] + 0.130,
        details=f"deployed={deployed_foot}, stowed={stowed_foot}",
    )

    return ctx.report()


object_model = build_object_model()
