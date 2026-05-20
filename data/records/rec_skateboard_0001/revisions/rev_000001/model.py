from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
from pathlib import Path

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    LoftGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
MESH_DIR = ASSETS.mesh_dir

DECK_LENGTH = 0.80
DECK_WIDTH = 0.205
DECK_THICKNESS = 0.012
GRIP_THICKNESS = 0.0016
TRUCK_Y = 0.215
WHEEL_RADIUS = 0.027
WHEEL_WIDTH = 0.026


def _make_material(name: str, rgba: tuple[float, float, float, float]) -> Material:
    try:
        return Material(name=name, color=rgba)
    except TypeError:
        return Material(name=name, rgba=rgba)


MAPLE = _make_material("maple_veneer", (0.72, 0.57, 0.37, 1.0))
GRIP = _make_material("grip_tape", (0.08, 0.08, 0.09, 1.0))
TRUCK_METAL = _make_material("cast_aluminum", (0.71, 0.73, 0.76, 1.0))
STEEL = _make_material("steel_hardware", (0.47, 0.49, 0.52, 1.0))
BUSHING = _make_material("urethane_bushing", (0.84, 0.70, 0.34, 1.0))
WHEEL = _make_material("wheel_urethane", (0.94, 0.93, 0.87, 0.98))
HUB = _make_material("wheel_hub", (0.16, 0.16, 0.17, 1.0))


def _normalize(axis: tuple[float, float, float]) -> tuple[float, float, float]:
    length = math.sqrt(sum(value * value for value in axis))
    return tuple(value / length for value in axis)


def _deck_section_loop(
    y: float,
    width: float,
    thickness: float,
    center_lift: float,
    camber: float,
    underside_crown: float,
    samples: int = 15,
) -> list[tuple[float, float, float]]:
    top: list[tuple[float, float, float]] = []
    bottom: list[tuple[float, float, float]] = []
    half_width = width * 0.5
    for index in range(samples):
        t = index / (samples - 1)
        u = -1.0 + 2.0 * t
        x = half_width * u
        edge = abs(u) ** 1.55
        top_z = center_lift + thickness * 0.5 + camber * edge
        bottom_z = center_lift - thickness * 0.5 - underside_crown * edge
        top.append((x, top_z, y))
        bottom.append((x, bottom_z, y))
    return top + list(reversed(bottom))


def _deck_stations() -> list[tuple[float, float, float]]:
    return [
        (-0.400, 0.052, 0.055),
        (-0.360, 0.094, 0.042),
        (-0.318, 0.140, 0.026),
        (-0.255, 0.183, 0.011),
        (-0.190, 0.198, 0.004),
        (-0.110, 0.205, 0.001),
        (0.000, 0.205, 0.000),
        (0.110, 0.205, 0.001),
        (0.190, 0.198, 0.004),
        (0.255, 0.183, 0.011),
        (0.318, 0.140, 0.026),
        (0.360, 0.094, 0.042),
        (0.400, 0.052, 0.055),
    ]


def _build_deck_meshes() -> tuple[object, object]:
    MESH_DIR.mkdir(parents=True, exist_ok=True)

    deck_profiles: list[list[tuple[float, float, float]]] = []
    grip_profiles: list[list[tuple[float, float, float]]] = []
    for y, width, lift in _deck_stations():
        camber = 0.0030 + 0.0016 * (1.0 - min(abs(y) / (DECK_LENGTH * 0.5), 1.0))
        deck_profiles.append(
            _deck_section_loop(
                y=y,
                width=width,
                thickness=DECK_THICKNESS,
                center_lift=lift,
                camber=camber,
                underside_crown=0.0012,
            )
        )
        grip_profiles.append(
            _deck_section_loop(
                y=y,
                width=max(width - 0.016, 0.036),
                thickness=GRIP_THICKNESS,
                center_lift=lift + DECK_THICKNESS * 0.5 + 0.00025 + GRIP_THICKNESS * 0.5,
                camber=camber * 0.96,
                underside_crown=0.00015,
            )
        )

    deck_mesh = mesh_from_geometry(
        LoftGeometry(deck_profiles, cap=True, closed=True).rotate_x(math.pi * 0.5),
        MESH_DIR / "skateboard_deck.obj",
    )
    grip_mesh = mesh_from_geometry(
        LoftGeometry(grip_profiles, cap=True, closed=True).rotate_x(math.pi * 0.5),
        MESH_DIR / "skateboard_grip.obj",
    )
    return deck_mesh, grip_mesh


def _build_wheel_mesh() -> object:
    MESH_DIR.mkdir(parents=True, exist_ok=True)
    profile = [
        (0.0, -WHEEL_WIDTH * 0.5),
        (0.017, -WHEEL_WIDTH * 0.5),
        (0.022, -0.010),
        (WHEEL_RADIUS, -0.007),
        (WHEEL_RADIUS, 0.007),
        (0.022, 0.010),
        (0.017, WHEEL_WIDTH * 0.5),
        (0.0, WHEEL_WIDTH * 0.5),
    ]
    return mesh_from_geometry(LatheGeometry(profile, segments=48), MESH_DIR / "street_wheel.obj")


def _add_mount_hardware(part, top_z: float) -> None:
    for y0 in (-TRUCK_Y, TRUCK_Y):
        for x0 in (-0.021, 0.021):
            for dy in (-0.018, 0.018):
                part.visual(
                    Cylinder(radius=0.0031, length=0.0014),
                    origin=Origin(xyz=(x0, y0 + dy, top_z)),
                    material=STEEL,
                )


def _add_baseplate_visuals(part) -> None:
    part.visual(
        Box((0.078, 0.056, 0.005)),
        origin=Origin(xyz=(0.0, 0.0, -0.0025)),
        material=TRUCK_METAL,
    )
    part.visual(
        Box((0.050, 0.032, 0.007)),
        origin=Origin(xyz=(0.0, 0.0, -0.0075)),
        material=TRUCK_METAL,
    )
    part.visual(
        Cylinder(radius=0.014, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.010), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=BUSHING,
    )
    part.visual(
        Cylinder(radius=0.010, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, -0.0055), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=STEEL,
    )
    part.visual(
        Cylinder(radius=0.0055, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.016), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=STEEL,
    )
    for x0 in (-0.021, 0.021):
        for y0 in (-0.018, 0.018):
            part.visual(
                Cylinder(radius=0.0028, length=0.0022),
                origin=Origin(xyz=(x0, y0, -0.0052)),
                material=STEEL,
            )


def _add_hanger_visuals(part) -> None:
    part.visual(
        Cylinder(radius=0.0095, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, -0.001)),
        material=STEEL,
    )
    part.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.007)),
        material=TRUCK_METAL,
    )
    part.visual(
        Box((0.032, 0.022, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
        material=TRUCK_METAL,
    )
    part.visual(
        Box((0.094, 0.024, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.031)),
        material=TRUCK_METAL,
    )
    part.visual(
        Cylinder(radius=0.006, length=0.188),
        origin=Origin(xyz=(0.0, 0.0, -0.031), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=STEEL,
    )


def _add_wheel_visuals(part, wheel_mesh: object) -> None:
    part.visual(
        wheel_mesh,
        origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
        material=WHEEL,
    )
    part.visual(
        Cylinder(radius=0.011, length=0.020),
        origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
        material=HUB,
    )
    for x0 in (-0.0115, 0.0115):
        part.visual(
            Cylinder(radius=0.008, length=0.003),
            origin=Origin(xyz=(x0, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
            material=STEEL,
        )


def build_object_model() -> ArticulatedObject:
    deck_mesh, grip_mesh = _build_deck_meshes()
    wheel_mesh = _build_wheel_mesh()

    model = ArticulatedObject(name="street_skateboard", assets=ASSETS)

    deck = model.part("deck")
    deck.visual(deck_mesh, material=MAPLE)
    deck.inertial = Inertial.from_geometry(Box((DECK_LENGTH, DECK_WIDTH, 0.020)), mass=1.35)

    grip_tape = model.part("grip_tape")
    grip_tape.visual(grip_mesh, material=GRIP)
    _add_mount_hardware(grip_tape, top_z=0.0073)
    grip_tape.inertial = Inertial.from_geometry(
        Box((0.78, 0.19, 0.004)),
        mass=0.10,
        origin=Origin(xyz=(0.0, 0.0, 0.0073)),
    )

    front_baseplate = model.part("front_baseplate")
    _add_baseplate_visuals(front_baseplate)
    front_baseplate.inertial = Inertial.from_geometry(
        Box((0.078, 0.056, 0.028)),
        mass=0.20,
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
    )

    rear_baseplate = model.part("rear_baseplate")
    _add_baseplate_visuals(rear_baseplate)
    rear_baseplate.inertial = Inertial.from_geometry(
        Box((0.078, 0.056, 0.028)),
        mass=0.20,
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
    )

    front_hanger = model.part("front_hanger")
    _add_hanger_visuals(front_hanger)
    front_hanger.inertial = Inertial.from_geometry(
        Box((0.188, 0.028, 0.040)),
        mass=0.27,
        origin=Origin(xyz=(0.0, 0.0, -0.024)),
    )

    rear_hanger = model.part("rear_hanger")
    _add_hanger_visuals(rear_hanger)
    rear_hanger.inertial = Inertial.from_geometry(
        Box((0.188, 0.028, 0.040)),
        mass=0.27,
        origin=Origin(xyz=(0.0, 0.0, -0.024)),
    )

    for wheel_name in (
        "front_left_wheel",
        "front_right_wheel",
        "rear_left_wheel",
        "rear_right_wheel",
    ):
        wheel = model.part(wheel_name)
        _add_wheel_visuals(wheel, wheel_mesh)
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
            mass=0.20,
            origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
        )

    model.articulation(
        "deck_to_grip",
        ArticulationType.FIXED,
        parent="deck",
        child="grip_tape",
        origin=Origin(),
    )
    model.articulation(
        "deck_to_front_baseplate",
        ArticulationType.FIXED,
        parent="deck",
        child="front_baseplate",
        origin=Origin(xyz=(0.0, TRUCK_Y, -0.0068)),
    )
    model.articulation(
        "deck_to_rear_baseplate",
        ArticulationType.FIXED,
        parent="deck",
        child="rear_baseplate",
        origin=Origin(xyz=(0.0, -TRUCK_Y, -0.0068)),
    )

    steer_axis = _normalize((0.28, 0.0, 0.96))
    model.articulation(
        "front_truck_steer",
        ArticulationType.REVOLUTE,
        parent="front_baseplate",
        child="front_hanger",
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        axis=steer_axis,
        motion_limits=MotionLimits(effort=24.0, velocity=6.0, lower=-0.38, upper=0.38),
    )
    model.articulation(
        "rear_truck_steer",
        ArticulationType.REVOLUTE,
        parent="rear_baseplate",
        child="rear_hanger",
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        axis=steer_axis,
        motion_limits=MotionLimits(effort=24.0, velocity=6.0, lower=-0.38, upper=0.38),
    )

    wheel_positions = {
        "front_left_wheel_spin": ("front_hanger", "front_left_wheel", 0.103),
        "front_right_wheel_spin": ("front_hanger", "front_right_wheel", -0.103),
        "rear_left_wheel_spin": ("rear_hanger", "rear_left_wheel", 0.103),
        "rear_right_wheel_spin": ("rear_hanger", "rear_right_wheel", -0.103),
    }
    for joint_name, (parent, child, x_pos) in wheel_positions.items():
        model.articulation(
            joint_name,
            ArticulationType.CONTINUOUS,
            parent=parent,
            child=child,
            origin=Origin(xyz=(x_pos, 0.0, -0.031)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=40.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")

    ctx.allow_overlap(
        "deck",
        "grip_tape",
        reason="The grip sheet is bonded to the deck and conservative collision hulls can touch through the laminate.",
    )
    ctx.allow_overlap(
        "deck",
        "front_baseplate",
        reason="Truck baseplates sit flush against the curved deck underside.",
    )
    ctx.allow_overlap(
        "deck",
        "rear_baseplate",
        reason="Truck baseplates sit flush against the curved deck underside.",
    )
    ctx.allow_overlap(
        "front_baseplate",
        "front_hanger",
        reason="Bushings and the hanger pivot interleave around the kingpin.",
    )
    ctx.allow_overlap(
        "rear_baseplate",
        "rear_hanger",
        reason="Bushings and the hanger pivot interleave around the kingpin.",
    )
    ctx.allow_overlap(
        "front_hanger",
        "front_left_wheel",
        reason="The axle stub sits inside the wheel bearing seat and the wheel mesh is authored as a solid urethane shell.",
    )
    ctx.allow_overlap(
        "front_hanger",
        "front_right_wheel",
        reason="The axle stub sits inside the wheel bearing seat and the wheel mesh is authored as a solid urethane shell.",
    )
    ctx.allow_overlap(
        "rear_hanger",
        "rear_left_wheel",
        reason="The axle stub sits inside the wheel bearing seat and the wheel mesh is authored as a solid urethane shell.",
    )
    ctx.allow_overlap(
        "rear_hanger",
        "rear_right_wheel",
        reason="The axle stub sits inside the wheel bearing seat and the wheel mesh is authored as a solid urethane shell.",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=192, overlap_tol=0.004, overlap_volume_tol=0.0)

    ctx.expect_aabb_overlap("grip_tape", "deck", axes="xy", min_overlap=0.12)
    ctx.expect_aabb_overlap("front_baseplate", "deck", axes="xy", min_overlap=0.03)
    ctx.expect_aabb_overlap("rear_baseplate", "deck", axes="xy", min_overlap=0.03)
    ctx.expect_origin_distance("front_left_wheel", "front_baseplate", axes="xy", max_dist=0.13)
    ctx.expect_origin_distance("front_right_wheel", "front_baseplate", axes="xy", max_dist=0.13)
    ctx.expect_origin_distance("rear_left_wheel", "rear_baseplate", axes="xy", max_dist=0.13)
    ctx.expect_origin_distance("rear_right_wheel", "rear_baseplate", axes="xy", max_dist=0.13)
    ctx.expect_aabb_gap("deck", "front_left_wheel", axis="z", max_gap=0.05, max_penetration=0.0)
    ctx.expect_aabb_gap("deck", "front_right_wheel", axis="z", max_gap=0.05, max_penetration=0.0)
    ctx.expect_aabb_gap("deck", "rear_left_wheel", axis="z", max_gap=0.05, max_penetration=0.0)
    ctx.expect_aabb_gap("deck", "rear_right_wheel", axis="z", max_gap=0.05, max_penetration=0.0)
    ctx.expect_joint_motion_axis(
        "front_truck_steer",
        "front_left_wheel",
        world_axis="y",
        direction="positive",
        min_delta=0.015,
    )
    ctx.expect_joint_motion_axis(
        "front_truck_steer",
        "front_right_wheel",
        world_axis="y",
        direction="negative",
        min_delta=0.015,
    )
    ctx.expect_joint_motion_axis(
        "rear_truck_steer",
        "rear_left_wheel",
        world_axis="y",
        direction="positive",
        min_delta=0.015,
    )
    ctx.expect_joint_motion_axis(
        "rear_truck_steer",
        "rear_right_wheel",
        world_axis="y",
        direction="negative",
        min_delta=0.015,
    )

    deck_rest = ctx.part_world_position("deck")
    grip_rest = ctx.part_world_position("grip_tape")
    front_baseplate_rest = ctx.part_world_position("front_baseplate")
    rear_baseplate_rest = ctx.part_world_position("rear_baseplate")
    front_hanger_rest = ctx.part_world_position("front_hanger")
    rear_hanger_rest = ctx.part_world_position("rear_hanger")
    front_left_rest = ctx.part_world_position("front_left_wheel")
    front_right_rest = ctx.part_world_position("front_right_wheel")
    rear_left_rest = ctx.part_world_position("rear_left_wheel")
    rear_right_rest = ctx.part_world_position("rear_right_wheel")

    assert math.dist(deck_rest, grip_rest) < 1e-9
    assert abs(front_baseplate_rest[0]) < 1e-6 and abs(rear_baseplate_rest[0]) < 1e-6
    assert abs(front_baseplate_rest[1] - TRUCK_Y) < 0.003
    assert abs(rear_baseplate_rest[1] + TRUCK_Y) < 0.003
    assert front_baseplate_rest[2] < deck_rest[2] - 0.004
    assert rear_baseplate_rest[2] < deck_rest[2] - 0.004
    assert front_hanger_rest[2] < front_baseplate_rest[2]
    assert rear_hanger_rest[2] < rear_baseplate_rest[2]
    assert front_left_rest[0] > 0.09 and front_right_rest[0] < -0.09
    assert rear_left_rest[0] > 0.09 and rear_right_rest[0] < -0.09
    assert front_left_rest[1] > 0.18 and rear_left_rest[1] < -0.18
    assert front_left_rest[1] - rear_left_rest[1] > 0.40
    assert -0.060 < front_left_rest[2] < -0.040

    with ctx.pose(front_left_wheel_spin=1.35, rear_right_wheel_spin=2.15):
        spun_front_left = ctx.part_world_position("front_left_wheel")
        spun_rear_right = ctx.part_world_position("rear_right_wheel")
        assert math.dist(front_left_rest, spun_front_left) < 1e-9
        assert math.dist(rear_right_rest, spun_rear_right) < 1e-9

    with ctx.pose(front_truck_steer=0.30, rear_truck_steer=0.30):
        front_left_turn = ctx.part_world_position("front_left_wheel")
        front_right_turn = ctx.part_world_position("front_right_wheel")
        rear_left_turn = ctx.part_world_position("rear_left_wheel")
        rear_right_turn = ctx.part_world_position("rear_right_wheel")
        assert front_left_turn[1] > front_left_rest[1] + 0.018
        assert front_right_turn[1] < front_right_rest[1] - 0.018
        assert rear_left_turn[1] > rear_left_rest[1] + 0.018
        assert rear_right_turn[1] < rear_right_rest[1] - 0.018
        ctx.expect_origin_distance("front_left_wheel", "front_baseplate", axes="xy", max_dist=0.13)
        ctx.expect_origin_distance("rear_right_wheel", "rear_baseplate", axes="xy", max_dist=0.13)
        ctx.expect_aabb_gap("deck", "front_left_wheel", axis="z", max_gap=0.06, max_penetration=0.0)
        ctx.expect_aabb_gap("deck", "rear_right_wheel", axis="z", max_gap=0.06, max_penetration=0.0)

    with ctx.pose(front_truck_steer=-0.30, rear_truck_steer=-0.30):
        front_left_turn = ctx.part_world_position("front_left_wheel")
        front_right_turn = ctx.part_world_position("front_right_wheel")
        rear_left_turn = ctx.part_world_position("rear_left_wheel")
        rear_right_turn = ctx.part_world_position("rear_right_wheel")
        assert front_left_turn[1] < front_left_rest[1] - 0.018
        assert front_right_turn[1] > front_right_rest[1] + 0.018
        assert rear_left_turn[1] < rear_left_rest[1] - 0.018
        assert rear_right_turn[1] > rear_right_rest[1] + 0.018
        ctx.expect_aabb_gap("deck", "front_right_wheel", axis="z", max_gap=0.06, max_penetration=0.0)
        ctx.expect_aabb_gap("deck", "rear_left_wheel", axis="z", max_gap=0.06, max_penetration=0.0)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
