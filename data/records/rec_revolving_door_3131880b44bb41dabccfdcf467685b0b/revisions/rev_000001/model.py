from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


AIRLOCK_LENGTH = 4.90
AIRLOCK_WIDTH = 2.10
AIRLOCK_HEIGHT = 2.55
FLOOR_THICKNESS = 0.06
CEILING_THICKNESS = 0.12
SIDE_WALL_THICKNESS = 0.08
SIDE_WALL_HEIGHT = AIRLOCK_HEIGHT - FLOOR_THICKNESS - CEILING_THICKNESS
DRUM_CENTERS_X = (-1.20, 1.20)
DRUM_TRACK_RADIUS = 0.80
POST_RADIUS = 0.09
POST_HEIGHT = 2.12
WING_LENGTH = 0.62
WING_THICKNESS = 0.045
WING_HEIGHT = 2.02
WING_BASE_Z = 0.10
PORTAL_HEADER_Z = 2.30
PORTAL_HEADER_THICKNESS = 0.14
DRUM_SHELL_OUTER_RADIUS = 0.86
DRUM_SHELL_INNER_RADIUS = 0.84
DRUM_SHELL_HEIGHT = AIRLOCK_HEIGHT - CEILING_THICKNESS - FLOOR_THICKNESS


def _add_rotor_visuals(part, *, glass_material, metal_material) -> None:
    part.visual(
        Cylinder(radius=0.055, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=metal_material,
        name="bottom_bearing",
    )
    part.visual(
        Cylinder(radius=POST_RADIUS, length=POST_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, 0.04 + POST_HEIGHT / 2.0)),
        material=metal_material,
        name="central_post",
    )
    part.visual(
        Cylinder(radius=0.13, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=metal_material,
        name="lower_hub",
    )
    part.visual(
        Cylinder(radius=0.13, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 2.10)),
        material=metal_material,
        name="upper_hub",
    )

    radial_center = POST_RADIUS + WING_LENGTH / 2.0
    wing_center_z = WING_BASE_Z + WING_HEIGHT / 2.0
    wing_specs = (
        ("wing_pos_x", Box((WING_LENGTH, WING_THICKNESS, WING_HEIGHT)), (radial_center, 0.0, wing_center_z)),
        ("wing_neg_x", Box((WING_LENGTH, WING_THICKNESS, WING_HEIGHT)), (-radial_center, 0.0, wing_center_z)),
        ("wing_pos_y", Box((WING_THICKNESS, WING_LENGTH, WING_HEIGHT)), (0.0, radial_center, wing_center_z)),
        ("wing_neg_y", Box((WING_THICKNESS, WING_LENGTH, WING_HEIGHT)), (0.0, -radial_center, wing_center_z)),
    )
    for name, geometry, xyz in wing_specs:
        part.visual(
            geometry,
            origin=Origin(xyz=xyz),
            material=glass_material,
            name=name,
        )

    rail_specs = (
        ("outer_rail_pos_x", Box((0.035, 0.085, WING_HEIGHT)), (POST_RADIUS + WING_LENGTH + 0.0175, 0.0, wing_center_z)),
        ("outer_rail_neg_x", Box((0.035, 0.085, WING_HEIGHT)), (-POST_RADIUS - WING_LENGTH - 0.0175, 0.0, wing_center_z)),
        ("outer_rail_pos_y", Box((0.085, 0.035, WING_HEIGHT)), (0.0, POST_RADIUS + WING_LENGTH + 0.0175, wing_center_z)),
        ("outer_rail_neg_y", Box((0.085, 0.035, WING_HEIGHT)), (0.0, -POST_RADIUS - WING_LENGTH - 0.0175, wing_center_z)),
    )
    for name, geometry, xyz in rail_specs:
        part.visual(
            geometry,
            origin=Origin(xyz=xyz),
            material=metal_material,
            name=name,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_stage_revolving_airlock")

    frame_metal = model.material("frame_metal", rgba=(0.23, 0.25, 0.28, 1.0))
    cladding = model.material("cladding", rgba=(0.50, 0.52, 0.55, 1.0))
    rotor_metal = model.material("rotor_metal", rgba=(0.72, 0.74, 0.77, 1.0))
    security_glass = model.material("security_glass", rgba=(0.60, 0.76, 0.84, 0.35))
    floor_finish = model.material("floor_finish", rgba=(0.18, 0.20, 0.21, 1.0))

    drum_shell_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (DRUM_SHELL_OUTER_RADIUS, 0.0),
                (DRUM_SHELL_OUTER_RADIUS, DRUM_SHELL_HEIGHT),
            ],
            [
                (DRUM_SHELL_INNER_RADIUS, 0.02),
                (DRUM_SHELL_INNER_RADIUS, DRUM_SHELL_HEIGHT - 0.02),
            ],
            segments=64,
        ),
        "drum_shell",
    )

    frame = model.part("airlock_frame")
    frame.visual(
        Box((AIRLOCK_LENGTH, AIRLOCK_WIDTH, FLOOR_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, FLOOR_THICKNESS / 2.0)),
        material=floor_finish,
        name="floor_deck",
    )
    frame.visual(
        Box((AIRLOCK_LENGTH, AIRLOCK_WIDTH, CEILING_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, AIRLOCK_HEIGHT - CEILING_THICKNESS / 2.0)),
        material=frame_metal,
        name="ceiling_canopy",
    )
    frame.visual(
        Box((AIRLOCK_LENGTH, SIDE_WALL_THICKNESS, SIDE_WALL_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -AIRLOCK_WIDTH / 2.0 + SIDE_WALL_THICKNESS / 2.0,
                FLOOR_THICKNESS + SIDE_WALL_HEIGHT / 2.0,
            )
        ),
        material=cladding,
        name="left_side_wall",
    )
    frame.visual(
        Box((AIRLOCK_LENGTH, SIDE_WALL_THICKNESS, SIDE_WALL_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                AIRLOCK_WIDTH / 2.0 - SIDE_WALL_THICKNESS / 2.0,
                FLOOR_THICKNESS + SIDE_WALL_HEIGHT / 2.0,
            )
        ),
        material=cladding,
        name="right_side_wall",
    )

    for x, name in ((-2.35, "entry_header"), (0.0, "interstage_header"), (2.35, "exit_header")):
        frame.visual(
            Box((0.14, AIRLOCK_WIDTH - 2.0 * SIDE_WALL_THICKNESS + 0.02, PORTAL_HEADER_THICKNESS)),
            origin=Origin(xyz=(x, 0.0, PORTAL_HEADER_Z)),
            material=frame_metal,
            name=name,
        )

    for x, prefix in ((-2.35, "entry"), (0.0, "interstage"), (2.35, "exit")):
        for y, side in ((-0.82, "left"), (0.82, "right")):
            frame.visual(
                Box((0.14, 0.30, SIDE_WALL_HEIGHT)),
                origin=Origin(
                    xyz=(x, y, FLOOR_THICKNESS + SIDE_WALL_HEIGHT / 2.0),
                ),
                material=frame_metal,
                name=f"{prefix}_{side}_jamb",
            )

    for x, name in zip(DRUM_CENTERS_X, ("front", "rear")):
        frame.visual(
            drum_shell_mesh,
            origin=Origin(xyz=(x, 0.0, FLOOR_THICKNESS)),
            material=security_glass,
            name=f"{name}_drum_shell",
        )
        frame.visual(
            Cylinder(radius=DRUM_TRACK_RADIUS, length=FLOOR_THICKNESS),
            origin=Origin(xyz=(x, 0.0, FLOOR_THICKNESS / 2.0)),
            material=frame_metal,
            name=f"{name}_floor_ring",
        )
        frame.visual(
            Cylinder(radius=DRUM_TRACK_RADIUS, length=0.04),
            origin=Origin(xyz=(x, 0.0, AIRLOCK_HEIGHT - CEILING_THICKNESS + 0.02)),
            material=frame_metal,
            name=f"{name}_ceiling_ring",
        )
        frame.visual(
            Box((1.16, 1.52, 0.18)),
            origin=Origin(xyz=(x, 0.0, AIRLOCK_HEIGHT - 0.01)),
            material=frame_metal,
            name=f"{name}_motor_housing",
        )

    frame.inertial = Inertial.from_geometry(
        Box((AIRLOCK_LENGTH, AIRLOCK_WIDTH, AIRLOCK_HEIGHT)),
        mass=1200.0,
        origin=Origin(xyz=(0.0, 0.0, AIRLOCK_HEIGHT / 2.0)),
    )

    front_rotor = model.part("front_drum_rotor")
    _add_rotor_visuals(
        front_rotor,
        glass_material=security_glass,
        metal_material=rotor_metal,
    )
    front_rotor.inertial = Inertial.from_geometry(
        Box((1.50, 1.50, 2.14)),
        mass=180.0,
        origin=Origin(xyz=(0.0, 0.0, 1.07)),
    )

    rear_rotor = model.part("rear_drum_rotor")
    _add_rotor_visuals(
        rear_rotor,
        glass_material=security_glass,
        metal_material=rotor_metal,
    )
    rear_rotor.inertial = Inertial.from_geometry(
        Box((1.50, 1.50, 2.14)),
        mass=180.0,
        origin=Origin(xyz=(0.0, 0.0, 1.07)),
    )

    model.articulation(
        "frame_to_front_drum",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=front_rotor,
        origin=Origin(xyz=(DRUM_CENTERS_X[0], 0.0, FLOOR_THICKNESS)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=1.5),
    )
    model.articulation(
        "frame_to_rear_drum",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_rotor,
        origin=Origin(xyz=(DRUM_CENTERS_X[1], 0.0, FLOOR_THICKNESS)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=1.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    frame = object_model.get_part("airlock_frame")
    front_rotor = object_model.get_part("front_drum_rotor")
    rear_rotor = object_model.get_part("rear_drum_rotor")
    front_joint = object_model.get_articulation("frame_to_front_drum")
    rear_joint = object_model.get_articulation("frame_to_rear_drum")

    def elem_center(part, elem_name: str) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem_name)
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    for joint, name in ((front_joint, "front"), (rear_joint, "rear")):
        limits = joint.motion_limits
        ctx.check(
            f"{name} drum uses continuous rotation",
            joint.joint_type == ArticulationType.CONTINUOUS
            and limits is not None
            and limits.lower is None
            and limits.upper is None
            and joint.axis == (0.0, 0.0, 1.0),
            details=f"type={joint.joint_type}, axis={joint.axis}, limits={limits}",
        )

    ctx.expect_gap(
        frame,
        front_rotor,
        axis="y",
        positive_elem="right_side_wall",
        negative_elem="wing_pos_y",
        min_gap=0.18,
        name="front drum clears right enclosure wall",
    )
    ctx.expect_gap(
        front_rotor,
        frame,
        axis="y",
        positive_elem="wing_neg_y",
        negative_elem="left_side_wall",
        min_gap=0.18,
        name="front drum clears left enclosure wall",
    )
    ctx.expect_gap(
        frame,
        rear_rotor,
        axis="y",
        positive_elem="right_side_wall",
        negative_elem="wing_pos_y",
        min_gap=0.18,
        name="rear drum clears right enclosure wall",
    )
    ctx.expect_gap(
        rear_rotor,
        frame,
        axis="y",
        positive_elem="wing_neg_y",
        negative_elem="left_side_wall",
        min_gap=0.18,
        name="rear drum clears left enclosure wall",
    )
    ctx.expect_gap(
        rear_rotor,
        front_rotor,
        axis="x",
        positive_elem="wing_neg_x",
        negative_elem="wing_pos_x",
        min_gap=0.75,
        name="two revolving drums remain distinct stages in sequence",
    )

    front_rest = elem_center(front_rotor, "wing_pos_x")
    rear_rest = elem_center(rear_rotor, "wing_pos_x")

    with ctx.pose({front_joint: math.pi / 2.0}):
        front_turned = elem_center(front_rotor, "wing_pos_x")
        rear_static_during_front_turn = elem_center(rear_rotor, "wing_pos_x")
    ctx.check(
        "front rotor turns about its own post without moving the rear rotor",
        front_rest is not None
        and rear_rest is not None
        and front_turned is not None
        and rear_static_during_front_turn is not None
        and front_rest[0] > DRUM_CENTERS_X[0] + 0.35
        and abs(front_rest[1]) < 0.03
        and abs(front_turned[0] - DRUM_CENTERS_X[0]) < 0.03
        and front_turned[1] > 0.35
        and abs(rear_static_during_front_turn[0] - rear_rest[0]) < 0.01
        and abs(rear_static_during_front_turn[1] - rear_rest[1]) < 0.01,
        details=(
            f"front_rest={front_rest}, front_turned={front_turned}, "
            f"rear_rest={rear_rest}, rear_static={rear_static_during_front_turn}"
        ),
    )

    with ctx.pose({rear_joint: -math.pi / 2.0}):
        rear_turned = elem_center(rear_rotor, "wing_pos_x")
        front_static_during_rear_turn = elem_center(front_rotor, "wing_pos_x")
    ctx.check(
        "rear rotor turns independently in the opposite direction when commanded",
        rear_rest is not None
        and front_rest is not None
        and rear_turned is not None
        and front_static_during_rear_turn is not None
        and rear_rest[0] > DRUM_CENTERS_X[1] + 0.35
        and abs(rear_rest[1]) < 0.03
        and abs(rear_turned[0] - DRUM_CENTERS_X[1]) < 0.03
        and rear_turned[1] < -0.35
        and abs(front_static_during_rear_turn[0] - front_rest[0]) < 0.01
        and abs(front_static_during_rear_turn[1] - front_rest[1]) < 0.01,
        details=(
            f"rear_rest={rear_rest}, rear_turned={rear_turned}, "
            f"front_rest={front_rest}, front_static={front_static_during_rear_turn}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
