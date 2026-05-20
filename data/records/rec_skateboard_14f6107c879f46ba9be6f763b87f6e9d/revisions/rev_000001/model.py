from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


DECK_LENGTH = 0.81
DECK_WIDTH = 0.205
DECK_THICKNESS = 0.011
TRUCK_SPACING = 0.365
WHEEL_RADIUS = 0.03
WHEEL_WIDTH = 0.036
AXLE_SPAN = 0.205


def _translate_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _deck_section(x_pos: float, width: float, thickness: float, z_center: float) -> list[tuple[float, float, float]]:
    radius = min(0.018, width * 0.16, thickness * 0.46)
    return [(x_pos, y, z_center + z) for y, z in rounded_rect_profile(width, thickness, radius, corner_segments=8)]


def _deck_mesh():
    return section_loft(
        [
            _deck_section(-0.5 * DECK_LENGTH, 0.082, DECK_THICKNESS, 0.033),
            _deck_section(-0.34, 0.155, DECK_THICKNESS, 0.022),
            _deck_section(-0.5 * TRUCK_SPACING, 0.198, DECK_THICKNESS, 0.006),
            _deck_section(0.0, DECK_WIDTH, DECK_THICKNESS, 0.0),
            _deck_section(0.5 * TRUCK_SPACING, 0.198, DECK_THICKNESS, 0.006),
            _deck_section(0.34, 0.155, DECK_THICKNESS, 0.022),
            _deck_section(0.5 * DECK_LENGTH, 0.082, DECK_THICKNESS, 0.033),
        ]
    )


def _truck_baseplate_mesh():
    outer = rounded_rect_profile(0.086, 0.056, 0.012, corner_segments=8)
    hole = rounded_rect_profile(0.006, 0.006, 0.003, corner_segments=8)
    holes = [
        rounded_rect_profile(0.024, 0.016, 0.006, corner_segments=8),
        _translate_profile(hole, dx=-0.024, dy=-0.015),
        _translate_profile(hole, dx=-0.024, dy=0.015),
        _translate_profile(hole, dx=0.024, dy=-0.015),
        _translate_profile(hole, dx=0.024, dy=0.015),
    ]
    return ExtrudeWithHolesGeometry(outer, holes, height=0.004, center=True)


def _merge_meshes(*geometries) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _hanger_mesh():
    axle_z = -0.037
    center_drop = -0.025
    brace_y = 0.056
    return _merge_meshes(
        CylinderGeometry(radius=0.009, height=0.014, radial_segments=18).translate(0.0, 0.0, -0.007),
        tube_from_spline_points(
            [(0.0, 0.0, 0.0), (0.0, 0.0, -0.010), (0.0, 0.0, center_drop)],
            radius=0.0072,
            samples_per_segment=10,
            radial_segments=14,
        ),
        tube_from_spline_points(
            [(0.0, 0.0, center_drop), (0.0, brace_y * 0.48, -0.030), (0.0, brace_y, axle_z)],
            radius=0.006,
            samples_per_segment=10,
            radial_segments=14,
        ),
        tube_from_spline_points(
            [(0.0, 0.0, center_drop), (0.0, -brace_y * 0.48, -0.030), (0.0, -brace_y, axle_z)],
            radius=0.006,
            samples_per_segment=10,
            radial_segments=14,
        ),
        CylinderGeometry(radius=0.011, height=0.003, radial_segments=18).rotate_x(-pi / 2.0).translate(0.0, 0.0915, axle_z),
        CylinderGeometry(radius=0.011, height=0.003, radial_segments=18).rotate_x(-pi / 2.0).translate(0.0, -0.0915, axle_z),
        CylinderGeometry(radius=0.008, height=0.040, radial_segments=18).rotate_x(-pi / 2.0).translate(0.0, 0.052, axle_z),
        CylinderGeometry(radius=0.008, height=0.040, radial_segments=18).rotate_x(-pi / 2.0).translate(0.0, -0.052, axle_z),
        CylinderGeometry(radius=0.0045, height=AXLE_SPAN, radial_segments=18).rotate_x(-pi / 2.0).translate(0.0, 0.0, axle_z),
    )


def _wheel_mesh():
    half_width = WHEEL_WIDTH * 0.5
    outer_profile = [
        (0.020, -half_width),
        (0.025, -half_width * 0.98),
        (0.028, -half_width * 0.75),
        (0.0295, -half_width * 0.38),
        (WHEEL_RADIUS, -half_width * 0.12),
        (WHEEL_RADIUS, half_width * 0.12),
        (0.0295, half_width * 0.38),
        (0.028, half_width * 0.75),
        (0.025, half_width * 0.98),
        (0.020, half_width),
    ]
    inner_profile = [
        (0.0072, -half_width),
        (0.0086, -half_width * 0.74),
        (0.0118, -half_width * 0.42),
        (0.0135, 0.0),
        (0.0118, half_width * 0.42),
        (0.0086, half_width * 0.74),
        (0.0072, half_width),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=48,
        start_cap="flat",
        end_cap="flat",
    ).rotate_x(-pi / 2.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_frame_skateboard")

    maple = model.material("maple", rgba=(0.72, 0.57, 0.36, 1.0))
    grip = model.material("grip", rgba=(0.10, 0.10, 0.11, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.24, 0.25, 0.28, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.68, 0.70, 0.73, 1.0))
    wheel_urethane = model.material("wheel_urethane", rgba=(0.93, 0.92, 0.83, 1.0))

    deck = model.part("deck")
    deck.visual(mesh_from_geometry(_deck_mesh(), "skateboard_deck"), material=maple, name="deck_body")
    deck.visual(
        Box((0.56, 0.168, 0.0015)),
        origin=Origin(xyz=(0.0, 0.0, DECK_THICKNESS * 0.34)),
        material=grip,
        name="grip_strip",
    )
    baseplate_mesh = mesh_from_geometry(_truck_baseplate_mesh(), "truck_baseplate")
    for name, x_pos in (("front", 0.5 * TRUCK_SPACING), ("rear", -0.5 * TRUCK_SPACING)):
        deck.visual(
            baseplate_mesh,
            origin=Origin(xyz=(x_pos, 0.0, -0.0015)),
            material=dark_steel,
            name=f"{name}_baseplate",
        )
        deck.visual(
            Cylinder(radius=0.0105, length=0.014),
            origin=Origin(xyz=(x_pos, 0.0, -0.0085)),
            material=dark_steel,
            name=f"{name}_pivot_cup",
        )
    deck.inertial = Inertial.from_geometry(
        Box((DECK_LENGTH, DECK_WIDTH, 0.072)),
        mass=1.9,
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
    )

    hanger_mesh = mesh_from_geometry(_hanger_mesh(), "truck_hanger")
    wheel_mesh = mesh_from_geometry(_wheel_mesh(), "skate_wheel")

    for truck_name in ("front_truck", "rear_truck"):
        truck = model.part(truck_name)
        truck.visual(hanger_mesh, material=brushed_steel, name="hanger_frame")
        truck.inertial = Inertial.from_geometry(
            Box((0.06, AXLE_SPAN, 0.05)),
            mass=0.58,
            origin=Origin(xyz=(0.0, 0.0, -0.026)),
        )

    for wheel_name in (
        "front_left_wheel",
        "front_right_wheel",
        "rear_left_wheel",
        "rear_right_wheel",
    ):
        wheel = model.part(wheel_name)
        wheel.visual(wheel_mesh, material=wheel_urethane, name="wheel_shell")
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
            mass=0.12,
            origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        )

    kingpin_x = 0.39
    kingpin_z = (1.0 - kingpin_x**2) ** 0.5
    front_axis = (-kingpin_x, 0.0, kingpin_z)
    rear_axis = (kingpin_x, 0.0, kingpin_z)

    model.articulation(
        "deck_to_front_truck",
        ArticulationType.REVOLUTE,
        parent=deck,
        child="front_truck",
        origin=Origin(xyz=(0.5 * TRUCK_SPACING, 0.0, -0.015)),
        axis=front_axis,
        motion_limits=MotionLimits(effort=9.0, velocity=2.5, lower=-0.14, upper=0.14),
    )
    model.articulation(
        "deck_to_rear_truck",
        ArticulationType.REVOLUTE,
        parent=deck,
        child="rear_truck",
        origin=Origin(xyz=(-0.5 * TRUCK_SPACING, 0.0, -0.015)),
        axis=rear_axis,
        motion_limits=MotionLimits(effort=9.0, velocity=2.5, lower=-0.14, upper=0.14),
    )

    axle_y = 0.111
    for parent_name, prefix in (("front_truck", "front"), ("rear_truck", "rear")):
        for side_name, side_sign in (("left", 1.0), ("right", -1.0)):
            model.articulation(
                f"{prefix}_{side_name}_wheel_spin",
                ArticulationType.CONTINUOUS,
                parent=parent_name,
                child=f"{prefix}_{side_name}_wheel",
                origin=Origin(xyz=(0.0, side_sign * axle_y, -0.037)),
                axis=(0.0, 1.0, 0.0),
                motion_limits=MotionLimits(effort=3.5, velocity=40.0),
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
    deck = object_model.get_part("deck")
    front_truck = object_model.get_part("front_truck")
    rear_truck = object_model.get_part("rear_truck")
    front_steer = object_model.get_articulation("deck_to_front_truck")
    rear_steer = object_model.get_articulation("deck_to_rear_truck")

    wheel_mounts = (
        ("front_left_wheel", front_truck, "front_left_wheel_spin"),
        ("front_right_wheel", front_truck, "front_right_wheel_spin"),
        ("rear_left_wheel", rear_truck, "rear_left_wheel_spin"),
        ("rear_right_wheel", rear_truck, "rear_right_wheel_spin"),
    )

    ctx.expect_contact(front_truck, deck, name="front truck seats against the deck")
    ctx.expect_contact(rear_truck, deck, name="rear truck seats against the deck")

    for wheel_name, truck, spin_name in wheel_mounts:
        wheel = object_model.get_part(wheel_name)
        spin_joint = object_model.get_articulation(spin_name)
        ctx.expect_contact(wheel, truck, name=f"{wheel_name} is carried on its axle")
        ctx.expect_gap(
            deck,
            wheel,
            axis="z",
            max_gap=0.012,
            max_penetration=0.0,
            name=f"{wheel_name} clears the deck at rest",
        )
        rest_pos = ctx.part_world_position(wheel)
        with ctx.pose({spin_joint: 1.2}):
            spun_pos = ctx.part_world_position(wheel)
        ctx.check(
            f"{wheel_name} spins about a fixed axle center",
            rest_pos is not None
            and spun_pos is not None
            and max(abs(a - b) for a, b in zip(rest_pos, spun_pos)) < 1e-6,
            details=f"rest={rest_pos}, spun={spun_pos}",
        )

    front_left_rest = ctx.part_world_position(object_model.get_part("front_left_wheel"))
    front_right_rest = ctx.part_world_position(object_model.get_part("front_right_wheel"))
    rear_left_rest = ctx.part_world_position(object_model.get_part("rear_left_wheel"))
    rear_right_rest = ctx.part_world_position(object_model.get_part("rear_right_wheel"))

    with ctx.pose({front_steer: 0.12}):
        ctx.expect_gap(
            deck,
            "front_left_wheel",
            axis="z",
            max_gap=0.02,
            max_penetration=0.0,
            name="front left wheel keeps deck clearance while steering",
        )
        ctx.expect_gap(
            deck,
            "front_right_wheel",
            axis="z",
            max_gap=0.01,
            max_penetration=0.0,
            name="front right wheel keeps deck clearance while steering",
        )
        front_left_steered = ctx.part_world_position("front_left_wheel")
        front_right_steered = ctx.part_world_position("front_right_wheel")

    ctx.check(
        "front truck steering reorients the axle",
        front_left_rest is not None
        and front_right_rest is not None
        and front_left_steered is not None
        and front_right_steered is not None
        and front_left_steered[0] < front_left_rest[0] - 0.008
        and front_right_steered[0] > front_right_rest[0] + 0.008,
        details=(
            f"rest_left={front_left_rest}, steered_left={front_left_steered}, "
            f"rest_right={front_right_rest}, steered_right={front_right_steered}"
        ),
    )

    with ctx.pose({rear_steer: 0.12}):
        ctx.expect_gap(
            deck,
            "rear_left_wheel",
            axis="z",
            max_gap=0.01,
            max_penetration=0.0,
            name="rear left wheel keeps deck clearance while steering",
        )
        ctx.expect_gap(
            deck,
            "rear_right_wheel",
            axis="z",
            max_gap=0.02,
            max_penetration=0.0,
            name="rear right wheel keeps deck clearance while steering",
        )
        rear_left_steered = ctx.part_world_position("rear_left_wheel")
        rear_right_steered = ctx.part_world_position("rear_right_wheel")

    ctx.check(
        "rear truck steering reorients the axle",
        rear_left_rest is not None
        and rear_right_rest is not None
        and rear_left_steered is not None
        and rear_right_steered is not None
        and rear_left_steered[0] < rear_left_rest[0] - 0.008
        and rear_right_steered[0] > rear_right_rest[0] + 0.008,
        details=(
            f"rest_left={rear_left_rest}, steered_left={rear_left_steered}, "
            f"rest_right={rear_right_rest}, steered_right={rear_right_steered}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
