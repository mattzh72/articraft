from __future__ import annotations

import math

import cadquery as cq
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
    TireGeometry,
    TireGroove,
    TireSidewall,
    TireShoulder,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    rounded_rect_profile,
)


CYL_TO_Y = (-(math.pi / 2.0), 0.0, 0.0)


def _origin_xyz(x: float, y: float, z: float, rpy=(0.0, 0.0, 0.0)) -> Origin:
    return Origin(xyz=(x, y, z), rpy=rpy)


def _rounded_deck_mesh(length: float, width: float, thickness: float, name: str):
    profile = rounded_rect_profile(length, width, radius=min(width * 0.22, 0.028), corner_segments=10)
    return mesh_from_geometry(ExtrudeGeometry(profile, thickness, center=True), name)


def _curved_fender_mesh(
    *,
    center_x: float,
    center_z: float,
    inner_radius: float,
    thickness: float,
    width: float,
    start_deg: float,
    end_deg: float,
    segments: int,
    name: str,
):
    """Thin arched brake fender, centered on local Y and curved in local XZ."""
    geom = MeshGeometry()
    angles = [
        math.radians(start_deg + (end_deg - start_deg) * i / segments)
        for i in range(segments + 1)
    ]
    for theta in angles:
        row = []
        for radius in (inner_radius, inner_radius + thickness):
            x = center_x + radius * math.cos(theta)
            z = center_z + radius * math.sin(theta)
            for y in (-width / 2.0, width / 2.0):
                row.append(geom.add_vertex(x, y, z))
    for i in range(segments):
        a = i * 4
        b = (i + 1) * 4
        # inner curved surface
        geom.add_face(a + 0, b + 0, b + 1)
        geom.add_face(a + 0, b + 1, a + 1)
        # outer curved surface
        geom.add_face(a + 2, a + 3, b + 3)
        geom.add_face(a + 2, b + 3, b + 2)
        # side walls
        geom.add_face(a + 0, a + 2, b + 2)
        geom.add_face(a + 0, b + 2, b + 0)
        geom.add_face(a + 1, b + 1, b + 3)
        geom.add_face(a + 1, b + 3, a + 3)
    # End caps.
    first = 0
    last = segments * 4
    for base in (first, last):
        geom.add_face(base + 0, base + 1, base + 3)
        geom.add_face(base + 0, base + 3, base + 2)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_city_kick_scooter")

    aluminum = Material("brushed_aluminum", rgba=(0.72, 0.74, 0.72, 1.0))
    dark_metal = Material("dark_anodized_metal", rgba=(0.10, 0.11, 0.12, 1.0))
    black_rubber = Material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    grip_black = Material("textured_black_grip", rgba=(0.025, 0.025, 0.022, 1.0))
    hinge_pin = Material("polished_hinge_pin", rgba=(0.88, 0.84, 0.76, 1.0))
    accent = Material("red_safety_accent", rgba=(0.80, 0.08, 0.04, 1.0))

    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.052,
            0.032,
            rim=WheelRim(inner_radius=0.036, flange_height=0.004, flange_thickness=0.0025),
            hub=WheelHub(
                radius=0.018,
                width=0.030,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=5, circle_diameter=0.025, hole_diameter=0.003),
            ),
            face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="split_y", count=5, thickness=0.0025, window_radius=0.007),
            bore=WheelBore(style="round", diameter=0.008),
        ),
        "shared_scooter_wheel_rim",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.078,
            0.036,
            inner_radius=0.052,
            grooves=(
                TireGroove(center_offset=-0.010, width=0.003, depth=0.002),
                TireGroove(center_offset=0.010, width=0.003, depth=0.002),
            ),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
            shoulder=TireShoulder(width=0.004, radius=0.0025),
        ),
        "shared_scooter_tire",
    )

    rear_deck = model.part("rear_deck")
    rear_deck.visual(
        _rounded_deck_mesh(0.350, 0.130, 0.035, "rear_deck_shell"),
        origin=_origin_xyz(-0.182, 0.0, -0.0340),
        material=aluminum,
        name="rear_deck_shell",
    )
    rear_deck.visual(
        Box((0.300, 0.105, 0.006)),
        origin=_origin_xyz(-0.195, 0.0, -0.0135),
        material=grip_black,
        name="rear_grip_pad",
    )
    # Rear half of the center folding hinge: two side knuckles and leaves.
    rear_deck.visual(
        Cylinder(radius=0.014, length=0.040),
        origin=_origin_xyz(0.0, -0.044, 0.0, CYL_TO_Y),
        material=hinge_pin,
        name="deck_hinge_barrel_0",
    )
    rear_deck.visual(
        Box((0.074, 0.034, 0.007)),
        origin=_origin_xyz(-0.038, -0.044, -0.014),
        material=dark_metal,
        name="deck_hinge_leaf_0",
    )
    rear_deck.visual(
        Cylinder(radius=0.014, length=0.040),
        origin=_origin_xyz(0.0, 0.044, 0.0, CYL_TO_Y),
        material=hinge_pin,
        name="deck_hinge_barrel_1",
    )
    rear_deck.visual(
        Box((0.074, 0.034, 0.007)),
        origin=_origin_xyz(-0.038, 0.044, -0.014),
        material=dark_metal,
        name="deck_hinge_leaf_1",
    )
    # Rear fork and dropout plates carry the rear wheel without intersecting it.
    rear_deck.visual(
        Box((0.210, 0.012, 0.018)),
        origin=_origin_xyz(-0.355, -0.040, -0.052),
        material=dark_metal,
        name="rear_fork_stay_0",
    )
    rear_deck.visual(
        Box((0.032, 0.012, 0.072)),
        origin=_origin_xyz(-0.455, -0.040, -0.066),
        material=dark_metal,
        name="rear_dropout_0",
    )
    rear_deck.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=_origin_xyz(-0.405, -0.042, 0.030, CYL_TO_Y),
        material=hinge_pin,
        name="brake_hinge_ear_0",
    )
    rear_deck.visual(
        Box((0.060, 0.014, 0.050)),
        origin=_origin_xyz(-0.383, -0.042, 0.003),
        material=dark_metal,
        name="brake_mount_strut_0",
    )
    rear_deck.visual(
        Box((0.210, 0.012, 0.018)),
        origin=_origin_xyz(-0.355, 0.040, -0.052),
        material=dark_metal,
        name="rear_fork_stay_1",
    )
    rear_deck.visual(
        Box((0.032, 0.012, 0.072)),
        origin=_origin_xyz(-0.455, 0.040, -0.066),
        material=dark_metal,
        name="rear_dropout_1",
    )
    rear_deck.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=_origin_xyz(-0.405, 0.042, 0.030, CYL_TO_Y),
        material=hinge_pin,
        name="brake_hinge_ear_1",
    )
    rear_deck.visual(
        Box((0.060, 0.014, 0.050)),
        origin=_origin_xyz(-0.383, 0.042, 0.003),
        material=dark_metal,
        name="brake_mount_strut_1",
    )
    rear_deck.visual(
        Box((0.090, 0.115, 0.015)),
        origin=_origin_xyz(-0.315, 0.0, -0.006),
        material=dark_metal,
        name="rear_fork_bridge",
    )

    front_deck = model.part("front_deck")
    front_deck.visual(
        _rounded_deck_mesh(0.350, 0.130, 0.035, "front_deck_shell"),
        origin=_origin_xyz(0.182, 0.0, -0.0340),
        material=aluminum,
        name="front_deck_shell",
    )
    front_deck.visual(
        Box((0.300, 0.105, 0.006)),
        origin=_origin_xyz(0.195, 0.0, -0.0135),
        material=grip_black,
        name="front_grip_pad",
    )
    front_deck.visual(
        Cylinder(radius=0.014, length=0.048),
        origin=_origin_xyz(0.0, 0.0, 0.0, CYL_TO_Y),
        material=hinge_pin,
        name="deck_hinge_barrel",
    )
    front_deck.visual(
        Box((0.074, 0.044, 0.007)),
        origin=_origin_xyz(0.038, 0.0, -0.014),
        material=dark_metal,
        name="deck_hinge_leaf",
    )
    # Head tube, clevis ears, and fork crown tied into the front deck.
    front_deck.visual(
        Cylinder(radius=0.022, length=0.075),
        origin=_origin_xyz(0.340, 0.0, 0.015),
        material=dark_metal,
        name="neck_socket",
    )
    front_deck.visual(
        Box((0.130, 0.102, 0.024)),
        origin=_origin_xyz(0.385, 0.0, 0.052),
        material=dark_metal,
        name="front_fork_crown",
    )
    front_deck.visual(
        Box((0.025, 0.012, 0.172)),
        origin=_origin_xyz(0.440, -0.040, -0.030),
        material=dark_metal,
        name="front_fork_leg_0",
    )
    front_deck.visual(
        Box((0.034, 0.012, 0.058)),
        origin=_origin_xyz(0.440, -0.040, -0.066),
        material=dark_metal,
        name="front_dropout_0",
    )
    front_deck.visual(
        Cylinder(radius=0.018, length=0.020),
        origin=_origin_xyz(0.340, -0.040, 0.080, CYL_TO_Y),
        material=hinge_pin,
        name="stem_hinge_ear_0",
    )
    front_deck.visual(
        Box((0.025, 0.012, 0.172)),
        origin=_origin_xyz(0.440, 0.040, -0.030),
        material=dark_metal,
        name="front_fork_leg_1",
    )
    front_deck.visual(
        Box((0.034, 0.012, 0.058)),
        origin=_origin_xyz(0.440, 0.040, -0.066),
        material=dark_metal,
        name="front_dropout_1",
    )
    front_deck.visual(
        Cylinder(radius=0.018, length=0.020),
        origin=_origin_xyz(0.340, 0.040, 0.080, CYL_TO_Y),
        material=hinge_pin,
        name="stem_hinge_ear_1",
    )

    stem = model.part("stem")
    stem.visual(
        Cylinder(radius=0.018, length=0.036),
        origin=_origin_xyz(0.0, 0.0, 0.0, CYL_TO_Y),
        material=hinge_pin,
        name="stem_hinge_barrel",
    )
    stem.visual(
        Cylinder(radius=0.017, length=0.620),
        origin=_origin_xyz(0.0, 0.0, 0.335),
        material=aluminum,
        name="upright_tube",
    )
    stem.visual(
        Cylinder(radius=0.024, length=0.060),
        origin=_origin_xyz(0.0, 0.0, 0.045),
        material=dark_metal,
        name="lower_clamp_collar",
    )
    stem.visual(
        Cylinder(radius=0.018, length=0.320),
        origin=_origin_xyz(0.0, 0.0, 0.660, CYL_TO_Y),
        material=aluminum,
        name="handlebar_bar",
    )
    for i, y in enumerate((-0.205, 0.205)):
        stem.visual(
            Cylinder(radius=0.016, length=0.110),
            origin=_origin_xyz(0.0, y, 0.660, CYL_TO_Y),
            material=black_rubber,
            name=f"handle_grip_{i}",
        )
    stem.visual(
        Box((0.052, 0.045, 0.030)),
        origin=_origin_xyz(0.0, 0.0, 0.622),
        material=dark_metal,
        name="handlebar_clamp",
    )

    rear_wheel = model.part("rear_wheel")
    rear_wheel.visual(
        tire_mesh,
        origin=_origin_xyz(0.0, 0.0, 0.0, rpy=(0.0, 0.0, math.pi / 2.0)),
        material=black_rubber,
        name="tire",
    )
    rear_wheel.visual(
        wheel_mesh,
        origin=_origin_xyz(0.0, 0.0, 0.0, rpy=(0.0, 0.0, math.pi / 2.0)),
        material=aluminum,
        name="rim",
    )
    rear_wheel.visual(
        Cylinder(radius=0.014, length=0.022),
        origin=_origin_xyz(0.0, -0.023, 0.0, CYL_TO_Y),
        material=dark_metal,
        name="side_hub_0",
    )
    rear_wheel.visual(
        Cylinder(radius=0.014, length=0.022),
        origin=_origin_xyz(0.0, 0.023, 0.0, CYL_TO_Y),
        material=dark_metal,
        name="side_hub_1",
    )

    front_wheel = model.part("front_wheel")
    front_wheel.visual(
        tire_mesh,
        origin=_origin_xyz(0.0, 0.0, 0.0, rpy=(0.0, 0.0, math.pi / 2.0)),
        material=black_rubber,
        name="tire",
    )
    front_wheel.visual(
        wheel_mesh,
        origin=_origin_xyz(0.0, 0.0, 0.0, rpy=(0.0, 0.0, math.pi / 2.0)),
        material=aluminum,
        name="rim",
    )
    front_wheel.visual(
        Cylinder(radius=0.014, length=0.022),
        origin=_origin_xyz(0.0, -0.023, 0.0, CYL_TO_Y),
        material=dark_metal,
        name="side_hub_0",
    )
    front_wheel.visual(
        Cylinder(radius=0.014, length=0.022),
        origin=_origin_xyz(0.0, 0.023, 0.0, CYL_TO_Y),
        material=dark_metal,
        name="side_hub_1",
    )

    fender_brake = model.part("fender_brake")
    fender_brake.visual(
        Cylinder(radius=0.012, length=0.066),
        origin=_origin_xyz(0.0, 0.0, 0.0, CYL_TO_Y),
        material=hinge_pin,
        name="brake_hinge_barrel",
    )
    fender_brake.visual(
        Box((0.016, 0.052, 0.018)),
        origin=_origin_xyz(0.003, 0.0, -0.008),
        material=accent,
        name="brake_front_tab",
    )
    for i, theta_deg in enumerate((55.0, 72.0, 90.0, 108.0, 125.0, 142.0, 154.0)):
        theta = math.radians(theta_deg)
        x = -0.050 + 0.098 * math.cos(theta)
        z = -0.096 + 0.098 * math.sin(theta)
        pitch = math.radians(90.0 - theta_deg)
        fender_brake.visual(
            Box((0.050, 0.062, 0.010)),
            origin=_origin_xyz(x, 0.0, z, rpy=(0.0, pitch, 0.0)),
            material=accent,
            name=f"fender_shell_{i}",
        )
    fender_brake.visual(
        Box((0.100, 0.066, 0.007)),
        origin=_origin_xyz(-0.082, 0.0, 0.006),
        material=grip_black,
        name="brake_foot_pad",
    )

    deck_hinge = model.articulation(
        "deck_hinge",
        ArticulationType.REVOLUTE,
        parent=rear_deck,
        child=front_deck,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.5, lower=0.0, upper=2.35),
    )
    model.articulation(
        "stem_hinge",
        ArticulationType.REVOLUTE,
        parent=front_deck,
        child=stem,
        origin=_origin_xyz(0.340, 0.0, 0.080),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.8, lower=0.0, upper=1.55),
    )
    model.articulation(
        "rear_brake_hinge",
        ArticulationType.REVOLUTE,
        parent=rear_deck,
        child=fender_brake,
        origin=_origin_xyz(-0.405, 0.0, 0.030),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=9.0, velocity=2.5, lower=-0.35, upper=0.90),
    )
    model.articulation(
        "rear_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=rear_deck,
        child=rear_wheel,
        origin=_origin_xyz(-0.455, 0.0, -0.066),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=20.0),
    )
    model.articulation(
        "front_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=front_deck,
        child=front_wheel,
        origin=_origin_xyz(0.440, 0.0, -0.066),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=20.0),
    )

    # Keep the center hinge object alive for sanity while the model is built.
    assert deck_hinge.name == "deck_hinge"
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    rear_deck = object_model.get_part("rear_deck")
    front_deck = object_model.get_part("front_deck")
    stem = object_model.get_part("stem")
    rear_wheel = object_model.get_part("rear_wheel")
    front_wheel = object_model.get_part("front_wheel")
    fender = object_model.get_part("fender_brake")

    deck_hinge = object_model.get_articulation("deck_hinge")
    stem_hinge = object_model.get_articulation("stem_hinge")
    brake_hinge = object_model.get_articulation("rear_brake_hinge")
    rear_spin = object_model.get_articulation("rear_wheel_spin")
    front_spin = object_model.get_articulation("front_wheel_spin")

    def aabb_center(box):
        if box is None:
            return None
        lo, hi = box
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    ctx.check(
        "both wheels use continuous spin joints",
        rear_spin.articulation_type == ArticulationType.CONTINUOUS
        and front_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"rear={rear_spin.articulation_type}, front={front_spin.articulation_type}",
    )
    ctx.check(
        "three folding mechanisms are revolute",
        deck_hinge.articulation_type == ArticulationType.REVOLUTE
        and stem_hinge.articulation_type == ArticulationType.REVOLUTE
        and brake_hinge.articulation_type == ArticulationType.REVOLUTE,
        details=(
            f"deck={deck_hinge.articulation_type}, "
            f"stem={stem_hinge.articulation_type}, brake={brake_hinge.articulation_type}"
        ),
    )

    ctx.expect_contact(
        front_deck,
        rear_deck,
        elem_a="deck_hinge_barrel",
        elem_b="deck_hinge_barrel_1",
        contact_tol=0.003,
        name="center hinge knuckles meet",
    )
    ctx.expect_contact(
        front_wheel,
        front_deck,
        elem_a="side_hub_1",
        elem_b="front_dropout_1",
        contact_tol=0.004,
        name="front wheel is captured between fork dropouts",
    )
    ctx.expect_contact(
        rear_wheel,
        rear_deck,
        elem_a="side_hub_1",
        elem_b="rear_dropout_1",
        contact_tol=0.004,
        name="rear wheel is captured between fork dropouts",
    )

    front_rest = ctx.part_world_position(front_wheel)
    with ctx.pose({deck_hinge: deck_hinge.motion_limits.upper}):
        front_folded = ctx.part_world_position(front_wheel)
    ctx.check(
        "center deck hinge folds the front assembly upward and rearward",
        front_rest is not None
        and front_folded is not None
        and front_folded[2] > front_rest[2] + 0.20
        and front_folded[0] < front_rest[0] - 0.45,
        details=f"rest={front_rest}, folded={front_folded}",
    )

    handle_rest = aabb_center(ctx.part_element_world_aabb(stem, elem="handlebar_bar"))
    with ctx.pose({stem_hinge: stem_hinge.motion_limits.upper}):
        handle_folded = aabb_center(ctx.part_element_world_aabb(stem, elem="handlebar_bar"))
    ctx.check(
        "upper neck clamp hinge folds the handlebar stem down over the deck",
        handle_rest is not None
        and handle_folded is not None
        and handle_folded[0] < handle_rest[0] - 0.45
        and handle_folded[2] < handle_rest[2] - 0.35,
        details=f"rest={handle_rest}, folded={handle_folded}",
    )

    pad_rest = aabb_center(ctx.part_element_world_aabb(fender, elem="brake_foot_pad"))
    with ctx.pose({brake_hinge: brake_hinge.motion_limits.upper}):
        pad_folded = aabb_center(ctx.part_element_world_aabb(fender, elem="brake_foot_pad"))
    ctx.check(
        "rear fender brake folds upward on its hinge",
        pad_rest is not None and pad_folded is not None and pad_folded[2] > pad_rest[2] + 0.04,
        details=f"rest={pad_rest}, folded={pad_folded}",
    )

    rear_pos = ctx.part_world_position(rear_wheel)
    front_pos = ctx.part_world_position(front_wheel)
    with ctx.pose({rear_spin: math.pi, front_spin: math.pi}):
        rear_spun = ctx.part_world_position(rear_wheel)
        front_spun = ctx.part_world_position(front_wheel)
    ctx.check(
        "wheel spin changes orientation without translating axles",
        rear_pos is not None
        and front_pos is not None
        and rear_spun is not None
        and front_spun is not None
        and abs(rear_pos[0] - rear_spun[0]) < 1e-6
        and abs(rear_pos[2] - rear_spun[2]) < 1e-6
        and abs(front_pos[0] - front_spun[0]) < 1e-6
        and abs(front_pos[2] - front_spun[2]) < 1e-6,
        details=f"rear={rear_pos}->{rear_spun}, front={front_pos}->{front_spun}",
    )

    return ctx.report()


object_model = build_object_model()
