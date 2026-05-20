from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _mat(model: ArticulatedObject, name: str, rgba: tuple[float, float, float, float]) -> Material:
    return model.material(name, rgba=rgba)


def _tube_mesh(points, radius: float, name: str, *, radial_segments: int = 18):
    return mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=14,
            radial_segments=radial_segments,
            cap_ends=True,
        ),
        name,
    )


def _curved_fender_mesh(
    name: str,
    *,
    center_x: float,
    center_z: float,
    inner_radius: float,
    outer_radius: float,
    width: float,
    start_deg: float,
    end_deg: float,
    segments: int = 18,
):
    """A one-piece curved strip, wide across Y and arched in the local XZ plane."""
    geom = MeshGeometry()
    angles = [
        math.radians(start_deg + (end_deg - start_deg) * i / segments)
        for i in range(segments + 1)
    ]
    # Four vertices per arc station: outer/inner on the left and right edges.
    for a in angles:
        ca = math.cos(a)
        sa = math.sin(a)
        for y in (-width / 2.0, width / 2.0):
            geom.add_vertex(center_x + outer_radius * ca, y, center_z + outer_radius * sa)
            geom.add_vertex(center_x + inner_radius * ca, y, center_z + inner_radius * sa)

    for i in range(segments):
        b = i * 4
        n = (i + 1) * 4
        # left side wall
        geom.add_face(b + 0, n + 0, n + 1)
        geom.add_face(b + 0, n + 1, b + 1)
        # right side wall
        geom.add_face(b + 2, n + 3, n + 2)
        geom.add_face(b + 2, b + 3, n + 3)
        # outer curved skin
        geom.add_face(b + 0, b + 2, n + 2)
        geom.add_face(b + 0, n + 2, n + 0)
        # inner curved skin
        geom.add_face(b + 1, n + 1, n + 3)
        geom.add_face(b + 1, n + 3, b + 3)

    # End caps.
    geom.add_face(0, 1, 3)
    geom.add_face(0, 3, 2)
    last = segments * 4
    geom.add_face(last + 0, last + 3, last + 1)
    geom.add_face(last + 0, last + 2, last + 3)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="urban_kick_scooter")

    aluminum = _mat(model, "brushed_aluminum", (0.72, 0.74, 0.72, 1.0))
    dark_aluminum = _mat(model, "dark_anodized_aluminum", (0.08, 0.09, 0.10, 1.0))
    blue = _mat(model, "deep_blue_powdercoat", (0.04, 0.14, 0.55, 1.0))
    black = _mat(model, "matte_black", (0.005, 0.005, 0.006, 1.0))
    rubber = _mat(model, "black_rubber", (0.015, 0.014, 0.012, 1.0))
    grip = _mat(model, "sandpaper_grip", (0.025, 0.025, 0.025, 1.0))
    red = _mat(model, "red_reflector", (0.85, 0.04, 0.02, 1.0))
    amber = _mat(model, "amber_reflector", (1.0, 0.48, 0.04, 1.0))

    # Main frame/deck: full-size narrow adult urban scooter proportions.
    deck = model.part("deck")
    deck_profile = rounded_rect_profile(0.58, 0.13, 0.045, corner_segments=10)
    deck.visual(
        mesh_from_geometry(ExtrudeGeometry(deck_profile, 0.035, center=True), "deck_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.135)),
        material=blue,
        name="deck_shell",
    )
    deck.visual(
        mesh_from_geometry(
            ExtrudeGeometry(rounded_rect_profile(0.50, 0.100, 0.026, corner_segments=8), 0.004, center=True),
            "grip_tape",
        ),
        origin=Origin(xyz=(-0.015, 0.0, 0.155)),
        material=grip,
        name="grip_tape",
    )
    for i, x in enumerate((-0.20, -0.10, 0.0, 0.10, 0.20)):
        deck.visual(
            Box((0.004, 0.092, 0.0025)),
            origin=Origin(xyz=(x, 0.0, 0.1580)),
            material=dark_aluminum,
            name=f"grip_groove_{i}",
        )
    deck.visual(
        Box((0.55, 0.010, 0.014)),
        origin=Origin(xyz=(-0.005, 0.069, 0.136)),
        material=aluminum,
        name="side_rail_0",
    )
    deck.visual(
        Box((0.55, 0.010, 0.014)),
        origin=Origin(xyz=(-0.005, -0.069, 0.136)),
        material=aluminum,
        name="side_rail_1",
    )

    # Fixed rear dropouts, axle, and brake hinge mounts are part of the frame.
    rear_axle = (-0.390, 0.0, 0.090)
    for side, y in enumerate((0.056, -0.056)):
        deck.visual(
            _tube_mesh(
                [(-0.280, y, 0.130), (-0.330, y, 0.112), (rear_axle[0], y, rear_axle[2])],
                0.0075,
                f"rear_stay_{side}",
            ),
            material=aluminum,
            name=f"rear_stay_{side}",
        )
    deck.visual(
        Cylinder(radius=0.0095, length=0.128),
        origin=Origin(xyz=rear_axle, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_aluminum,
        name="rear_axle",
    )
    for side, y in enumerate((0.052, -0.052)):
        deck.visual(
            _tube_mesh(
                [(-0.250, y, 0.150), (-0.300, y, 0.166), (-0.330, y, 0.176)],
                0.006,
                f"brake_strut_{side}",
            ),
            material=blue,
            name=f"brake_strut_{side}",
        )
    for side, y in enumerate((0.052, -0.052)):
        deck.visual(
            Box((0.032, 0.012, 0.036)),
            origin=Origin(xyz=(-0.330, y, 0.176)),
            material=blue,
            name=f"brake_tab_{side}",
        )
    deck.visual(
        Box((0.010, 0.070, 0.016)),
        origin=Origin(xyz=(-0.294, 0.0, 0.141)),
        material=red,
        name="rear_reflector",
    )

    # Front head support: low fixed bearing and gussets, clear of rotating fork.
    steer_pivot = (0.300, 0.0, 0.205)
    deck.visual(
        Box((0.070, 0.095, 0.030)),
        origin=Origin(xyz=(0.265, 0.0, 0.167)),
        material=blue,
        name="head_base",
    )
    deck.visual(
        Cylinder(radius=0.016, length=0.030),
        origin=Origin(xyz=steer_pivot),
        material=dark_aluminum,
        name="head_bushing",
    )
    deck.visual(
        Box((0.040, 0.044, 0.016)),
        origin=Origin(xyz=(0.292, 0.0, 0.185)),
        material=dark_aluminum,
        name="head_socket_bridge",
    )
    deck.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [(0.240, 0.052, 0.150), (0.275, 0.052, 0.180), (0.298, 0.052, 0.205)],
                radius=0.007,
                samples_per_segment=12,
                radial_segments=16,
                cap_ends=True,
            ),
            "head_gusset_0",
        ),
        material=aluminum,
        name="head_gusset_0",
    )
    deck.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [(0.240, -0.052, 0.150), (0.275, -0.052, 0.180), (0.298, -0.052, 0.205)],
                radius=0.007,
                samples_per_segment=12,
                radial_segments=16,
                cap_ends=True,
            ),
            "head_gusset_1",
        ),
        material=aluminum,
        name="head_gusset_1",
    )
    deck.visual(
        Box((0.040, 0.090, 0.016)),
        origin=Origin(xyz=(0.320, 0.0, 0.152)),
        material=amber,
        name="front_reflector",
    )

    # Steered assembly: stem, fork, handlebar, clamp, and front axle rotate together.
    steering = model.part("steering")
    column_top = (-0.100, 0.0, 0.860)
    steering.visual(
        _tube_mesh([(0.0, 0.0, 0.000), (-0.032, 0.0, 0.280), (-0.070, 0.0, 0.590), column_top], 0.0145, "steering_column"),
        material=aluminum,
        name="steering_column",
    )
    steering.visual(
        _tube_mesh([(0.0, 0.0, 0.000), (0.030, 0.0, -0.004), (0.058, 0.0, -0.005)], 0.012, "lower_steerer"),
        material=aluminum,
        name="lower_steerer",
    )
    steering.visual(
        Box((0.060, 0.088, 0.020)),
        origin=Origin(xyz=(0.070, 0.0, -0.005)),
        material=blue,
        name="fork_crown",
    )
    front_axle_rel = (0.120, 0.0, -0.115)
    for side, y in enumerate((0.038, -0.038)):
        steering.visual(
            _tube_mesh(
                [(0.065, y, -0.013), (0.088, y, -0.058), (front_axle_rel[0], y, front_axle_rel[2])],
                0.007,
                f"fork_blade_{side}",
            ),
            material=aluminum,
            name=f"fork_blade_{side}",
        )
    steering.visual(
        Cylinder(radius=0.0095, length=0.102),
        origin=Origin(xyz=front_axle_rel, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_aluminum,
        name="front_axle",
    )
    for idx, (x, z) in enumerate(((-0.035, 0.315), (-0.064, 0.565))):
        steering.visual(
            Box((0.048, 0.052, 0.034)),
            origin=Origin(xyz=(x, 0.0, z), rpy=(0.0, -0.11, 0.0)),
            material=dark_aluminum,
            name=f"stem_clamp_{idx}",
        )
        steering.visual(
            Box((0.010, 0.060, 0.020)),
            origin=Origin(xyz=(x + 0.018, -0.052, z)),
            material=black,
            name=f"clamp_lever_{idx}",
        )
    steering.visual(
        Cylinder(radius=0.012, length=0.440),
        origin=Origin(xyz=column_top, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="handlebar",
    )
    steering.visual(
        Box((0.050, 0.052, 0.045)),
        origin=Origin(xyz=column_top),
        material=dark_aluminum,
        name="bar_clamp",
    )
    for side, y in enumerate((0.242, -0.242)):
        steering.visual(
            Cylinder(radius=0.017, length=0.112),
            origin=Origin(xyz=(column_top[0], y, column_top[2]), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name=f"grip_{side}",
        )
    steering.visual(
        Box((0.018, 0.004, 0.065)),
        origin=Origin(xyz=(column_top[0] + 0.012, -0.040, column_top[2] - 0.030), rpy=(0.45, 0.0, 0.0)),
        material=black,
        name="bell_lever",
    )

    steer_axis = (-0.116, 0.0, 0.993)
    model.articulation(
        "deck_to_steering",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=steering,
        origin=Origin(xyz=steer_pivot),
        axis=steer_axis,
        motion_limits=MotionLimits(effort=18.0, velocity=3.5, lower=-0.70, upper=0.70),
    )

    # Foot brake/fender: a real hinged rear fender that presses toward the tire.
    brake = model.part("brake")
    brake.visual(
        Cylinder(radius=0.007, length=0.094),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_aluminum,
        name="brake_barrel",
    )
    brake.visual(
        _curved_fender_mesh(
            "brake_fender",
            center_x=-0.060,
            center_z=-0.086,
            inner_radius=0.105,
            outer_radius=0.113,
            width=0.074,
            start_deg=55.0,
            end_deg=155.0,
            segments=20,
        ),
        material=blue,
        name="brake_fender",
    )
    brake.visual(
        Box((0.090, 0.054, 0.006)),
        origin=Origin(xyz=(-0.070, 0.0, 0.010), rpy=(0.0, -0.35, 0.0)),
        material=black,
        name="brake_pad",
    )
    brake_joint = model.articulation(
        "deck_to_brake",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=brake,
        origin=Origin(xyz=(-0.330, 0.0, 0.176)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-0.28, upper=0.04),
    )

    # Urban wheels: small PU tires with distinct hubs, carried on continuous rolling joints.
    tire_geom = TireGeometry(
        0.090,
        0.032,
        inner_radius=0.062,
        carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.035),
        grooves=(
            TireGroove(center_offset=-0.006, width=0.0025, depth=0.0015),
            TireGroove(center_offset=0.006, width=0.0025, depth=0.0015),
        ),
        sidewall=TireSidewall(style="rounded", bulge=0.04),
        shoulder=TireShoulder(width=0.004, radius=0.0025),
    )
    rim_geom = WheelGeometry(
        0.062,
        0.034,
        rim=WheelRim(inner_radius=0.034, flange_height=0.004, flange_thickness=0.0025, bead_seat_depth=0.002),
        hub=WheelHub(
            radius=0.020,
            width=0.030,
            cap_style="domed",
            bolt_pattern=BoltPattern(count=5, circle_diameter=0.030, hole_diameter=0.003),
        ),
        face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
        spokes=WheelSpokes(style="split_y", count=5, thickness=0.0028, window_radius=0.006),
        bore=WheelBore(style="round", diameter=0.018),
    )

    rear_wheel = model.part("rear_wheel")
    rear_wheel.visual(
        mesh_from_geometry(tire_geom, "rear_tire"),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=rubber,
        name="tire",
    )
    rear_wheel.visual(
        mesh_from_geometry(rim_geom, "rear_rim"),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=aluminum,
        name="rim",
    )
    model.articulation(
        "deck_to_rear_wheel",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=rear_wheel,
        origin=Origin(xyz=rear_axle),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=40.0),
    )

    front_wheel = model.part("front_wheel")
    front_wheel.visual(
        mesh_from_geometry(tire_geom, "front_tire"),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=rubber,
        name="tire",
    )
    front_wheel.visual(
        mesh_from_geometry(rim_geom, "front_rim"),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=aluminum,
        name="rim",
    )
    model.articulation(
        "steering_to_front_wheel",
        ArticulationType.CONTINUOUS,
        parent=steering,
        child=front_wheel,
        origin=Origin(xyz=front_axle_rel),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=40.0),
    )

    # Keep a reference in metadata for tests that need the lower limit value.
    brake_joint.meta["rest_note"] = "negative motion presses the fender toward the rear tire"
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    steering = object_model.get_part("steering")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")
    brake = object_model.get_part("brake")
    steer_joint = object_model.get_articulation("deck_to_steering")
    front_roll = object_model.get_articulation("steering_to_front_wheel")
    rear_roll = object_model.get_articulation("deck_to_rear_wheel")
    brake_joint = object_model.get_articulation("deck_to_brake")

    ctx.allow_overlap(
        deck,
        rear_wheel,
        elem_a="rear_axle",
        elem_b="rim",
        reason="The rear axle is intentionally captured inside the wheel bearing/bore proxy.",
    )
    ctx.expect_overlap(
        deck,
        rear_wheel,
        axes="y",
        elem_a="rear_axle",
        elem_b="rim",
        min_overlap=0.030,
        name="rear axle spans wheel hub",
    )
    ctx.allow_overlap(
        deck,
        steering,
        elem_a="head_bushing",
        elem_b="lower_steerer",
        reason="The lower steerer is intentionally seated inside a simplified headset bushing.",
    )
    ctx.expect_overlap(
        deck,
        steering,
        axes="z",
        elem_a="head_bushing",
        elem_b="lower_steerer",
        min_overlap=0.010,
        name="lower steerer remains seated in headset",
    )
    ctx.allow_overlap(
        deck,
        steering,
        elem_a="head_bushing",
        elem_b="steering_column",
        reason="The steering column passes through the same simplified headset bushing.",
    )
    ctx.expect_overlap(
        deck,
        steering,
        axes="z",
        elem_a="head_bushing",
        elem_b="steering_column",
        min_overlap=0.010,
        name="steering column passes through headset",
    )
    ctx.allow_overlap(
        front_wheel,
        steering,
        elem_a="rim",
        elem_b="front_axle",
        reason="The front axle is intentionally captured inside the wheel bearing/bore proxy.",
    )
    ctx.expect_overlap(
        front_wheel,
        steering,
        axes="y",
        elem_a="rim",
        elem_b="front_axle",
        min_overlap=0.030,
        name="front axle spans wheel hub",
    )

    deck_aabb = ctx.part_world_aabb(deck)
    if deck_aabb is not None:
        (mn, mx) = deck_aabb
        deck_len = mx[0] - mn[0]
        deck_width = mx[1] - mn[1]
        ctx.check(
            "deck is long and narrow",
            deck_len > 0.55 and deck_width < 0.16 and deck_len / deck_width > 3.7,
            details=f"deck length={deck_len:.3f}, width={deck_width:.3f}",
        )

    ctx.check(
        "rolling wheels use continuous joints",
        front_roll.articulation_type == ArticulationType.CONTINUOUS
        and rear_roll.articulation_type == ArticulationType.CONTINUOUS,
    )
    ctx.check(
        "steering has practical stop angle",
        steer_joint.motion_limits is not None
        and steer_joint.motion_limits.lower <= -0.65
        and steer_joint.motion_limits.upper >= 0.65,
    )

    ctx.expect_overlap(
        brake,
        rear_wheel,
        axes="x",
        min_overlap=0.08,
        name="brake fender covers rear tire",
    )
    ctx.expect_gap(
        front_wheel,
        deck,
        axis="x",
        min_gap=0.02,
        negative_elem="deck_shell",
        name="front wheel sits ahead of deck",
    )
    ctx.expect_gap(
        deck,
        rear_wheel,
        axis="x",
        min_gap=0.0,
        positive_elem="deck_shell",
        name="deck clears rear wheel fore-aft",
    )

    rest_front = ctx.part_world_position(front_wheel)
    with ctx.pose({steer_joint: 0.50}):
        steered_front = ctx.part_world_position(front_wheel)
    ctx.check(
        "steering turns front wheel laterally",
        rest_front is not None
        and steered_front is not None
        and abs(steered_front[1] - rest_front[1]) > 0.045,
        details=f"rest={rest_front}, steered={steered_front}",
    )

    rest_brake_aabb = ctx.part_world_aabb(brake)
    with ctx.pose({brake_joint: -0.20}):
        pressed_brake_aabb = ctx.part_world_aabb(brake)
    ctx.check(
        "brake fender moves downward when pressed",
        rest_brake_aabb is not None
        and pressed_brake_aabb is not None
        and pressed_brake_aabb[0][2] < rest_brake_aabb[0][2] - 0.005,
        details=f"rest={rest_brake_aabb}, pressed={pressed_brake_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
