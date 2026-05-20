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
    rounded_rect_profile,
    section_loft,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _norm(vec: tuple[float, float, float]) -> tuple[float, float, float]:
    mag = math.sqrt(sum(component * component for component in vec))
    return (vec[0] / mag, vec[1] / mag, vec[2] / mag)


def _scale(vec: tuple[float, float, float], scalar: float) -> tuple[float, float, float]:
    return (vec[0] * scalar, vec[1] * scalar, vec[2] * scalar)


def _add(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _axis_origin(
    center: tuple[float, float, float],
    direction: tuple[float, float, float],
) -> Origin:
    return Origin(
        xyz=center,
        rpy=(0.0, math.atan2(direction[0], direction[2]), 0.0),
    )


def _deck_section(
    x_pos: float,
    width: float,
    thickness: float,
    z_center: float,
) -> list[tuple[float, float, float]]:
    radius = min(0.028, width * 0.22, thickness * 0.48)
    return [
        (x_pos, y_pos, z_center + z_pos)
        for y_pos, z_pos in rounded_rect_profile(
            width,
            thickness,
            radius,
            corner_segments=8,
        )
    ]


def _build_deck_mesh():
    return section_loft(
        [
            _deck_section(-0.390, 0.110, 0.014, 0.024),
            _deck_section(-0.325, 0.172, 0.014, 0.012),
            _deck_section(-0.235, 0.215, 0.014, 0.003),
            _deck_section(-0.080, 0.234, 0.014, 0.000),
            _deck_section(0.080, 0.234, 0.014, 0.000),
            _deck_section(0.235, 0.215, 0.014, 0.003),
            _deck_section(0.325, 0.176, 0.014, 0.014),
            _deck_section(0.390, 0.104, 0.014, 0.028),
        ]
    )


def _build_wheel_mesh(name: str, radius: float, width: float):
    half_width = width * 0.5
    outer_profile = [
        (radius * 0.82, -half_width),
        (radius * 0.95, -half_width * 0.90),
        (radius, -half_width * 0.42),
        (radius, half_width * 0.42),
        (radius * 0.95, half_width * 0.90),
        (radius * 0.82, half_width),
    ]
    inner_profile = [
        (0.0058, -half_width * 0.88),
        (radius * 0.33, -half_width * 0.80),
        (radius * 0.46, -half_width * 0.30),
        (radius * 0.46, half_width * 0.30),
        (radius * 0.33, half_width * 0.80),
        (0.0058, half_width * 0.88),
    ]
    return _save_mesh(
        name,
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=56,
            start_cap="flat",
            end_cap="flat",
        ).rotate_x(math.pi / 2.0),
    )


def _build_ring_mesh(name: str, outer_radius: float, inner_radius: float, width: float):
    half_width = width * 0.5
    return _save_mesh(
        name,
        LatheGeometry.from_shell_profiles(
            [
                (outer_radius, -half_width),
                (outer_radius, half_width),
            ],
            [
                (inner_radius, -half_width),
                (inner_radius, half_width),
            ],
            segments=40,
            start_cap="flat",
            end_cap="flat",
        ).rotate_x(math.pi / 2.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_legacy_skateboard")

    deck_wood = model.material("deck_wood", rgba=(0.55, 0.40, 0.24, 1.0))
    reinforcement_steel = model.material("reinforcement_steel", rgba=(0.35, 0.36, 0.39, 1.0))
    cast_aluminum = model.material("cast_aluminum", rgba=(0.70, 0.72, 0.74, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.21, 1.0))
    bushing_amber = model.material("bushing_amber", rgba=(0.72, 0.55, 0.22, 1.0))
    hatch_steel = model.material("hatch_steel", rgba=(0.61, 0.63, 0.66, 1.0))
    seal_black = model.material("seal_black", rgba=(0.08, 0.08, 0.09, 1.0))
    wheel_urethane = model.material("wheel_urethane", rgba=(0.88, 0.74, 0.41, 1.0))

    wheel_radius = 0.031
    wheel_width = 0.042
    wheel_mesh = _build_wheel_mesh("skateboard_wheel_shell", wheel_radius, wheel_width)
    bearing_ring_mesh = _build_ring_mesh("skateboard_bearing_ring", 0.0105, 0.0044, 0.008)

    deck = model.part("deck")
    deck.visual(_save_mesh("legacy_deck_shell", _build_deck_mesh()), material=deck_wood, name="deck_shell")
    deck.visual(
        Box((0.114, 0.086, 0.008)),
        origin=Origin(xyz=(0.185, 0.0, -0.005)),
        material=reinforcement_steel,
        name="front_doubler",
    )
    deck.visual(
        Box((0.114, 0.086, 0.008)),
        origin=Origin(xyz=(-0.185, 0.0, -0.005)),
        material=reinforcement_steel,
        name="rear_doubler",
    )
    deck.inertial = Inertial.from_geometry(
        Box((0.80, 0.24, 0.06)),
        mass=3.4,
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
    )

    def add_service_hatch(name: str, x_pos: float) -> None:
        hatch = model.part(name)
        hatch.visual(
            Box((0.092, 0.050, 0.003)),
            origin=Origin(xyz=(0.0, 0.0, 0.0015)),
            material=hatch_steel,
            name="plate",
        )
        hatch.visual(
            Box((0.074, 0.032, 0.0016)),
            origin=Origin(xyz=(0.0, 0.0, 0.0008)),
            material=seal_black,
            name="seal",
        )
        for bolt_x in (-0.030, 0.030):
            for bolt_y in (-0.016, 0.016):
                hatch.visual(
                    Cylinder(radius=0.0028, length=0.0032),
                    origin=Origin(xyz=(bolt_x, bolt_y, 0.0024)),
                    material=dark_steel,
                )
        hatch.inertial = Inertial.from_geometry(
            Box((0.092, 0.050, 0.005)),
            mass=0.08,
            origin=Origin(xyz=(0.0, 0.0, 0.0025)),
        )
        model.articulation(
            f"{name}_mount",
            ArticulationType.FIXED,
            parent=deck,
            child=hatch,
            origin=Origin(xyz=(x_pos, 0.0, 0.007)),
        )

    add_service_hatch("front_service_hatch", 0.185)
    add_service_hatch("rear_service_hatch", -0.185)

    def add_truck(prefix: str, x_pos: float, axis_x: float) -> None:
        steer_axis = _norm((axis_x, 0.0, -1.0))
        pivot_local = (0.0, 0.0, -0.022)
        axle_local = (0.0, 0.0, -0.056)

        adapter = model.part(f"{prefix}_adapter")
        adapter.visual(
            Box((0.086, 0.066, 0.004)),
            origin=Origin(xyz=(0.0, 0.0, -0.002)),
            material=reinforcement_steel,
            name="adapter_plate",
        )
        adapter.visual(
            Box((0.014, 0.020, 0.008)),
            origin=Origin(xyz=(0.022, 0.0, -0.008)),
            material=reinforcement_steel,
            name="adapter_block",
        )
        adapter.visual(
            Box((0.018, 0.052, 0.008)),
            origin=Origin(xyz=(0.024, 0.0, -0.008)),
            material=reinforcement_steel,
        )
        adapter.visual(
            Box((0.018, 0.052, 0.008)),
            origin=Origin(xyz=(-0.024, 0.0, -0.008)),
            material=reinforcement_steel,
        )
        for bolt_x in (-0.028, 0.028):
            for bolt_y in (-0.020, 0.020):
                adapter.visual(
                    Cylinder(radius=0.0032, length=0.004),
                    origin=Origin(xyz=(bolt_x, bolt_y, -0.0018)),
                    material=dark_steel,
                )
        adapter.inertial = Inertial.from_geometry(
            Box((0.086, 0.066, 0.016)),
            mass=0.22,
            origin=Origin(xyz=(0.0, 0.0, -0.008)),
        )

        baseplate = model.part(f"{prefix}_baseplate")
        baseplate.visual(
            Box((0.062, 0.052, 0.006)),
            origin=Origin(xyz=(0.0, 0.0, -0.003)),
            material=cast_aluminum,
            name="mounting_plate",
        )
        baseplate.visual(
            Box((0.020, 0.044, 0.012)),
            origin=Origin(xyz=(0.0, 0.0, -0.012)),
            material=cast_aluminum,
            name="pivot_tower",
        )
        baseplate.visual(
            Box((0.032, 0.008, 0.010)),
            origin=Origin(xyz=(0.0, 0.017, -0.017)),
            material=cast_aluminum,
            name="hanger_saddle",
        )
        baseplate.visual(
            Box((0.032, 0.008, 0.010)),
            origin=Origin(xyz=(0.0, -0.017, -0.017)),
            material=cast_aluminum,
        )
        baseplate.visual(
            Box((0.016, 0.034, 0.012)),
            origin=Origin(xyz=(0.015, 0.0, -0.013)),
            material=cast_aluminum,
        )
        baseplate.visual(
            Box((0.016, 0.034, 0.012)),
            origin=Origin(xyz=(-0.015, 0.0, -0.013)),
            material=cast_aluminum,
        )
        baseplate.visual(
            Cylinder(radius=0.005, length=0.028),
            origin=_axis_origin(_add(pivot_local, _scale(steer_axis, 0.000)), steer_axis),
            material=dark_steel,
            name="kingpin_shaft",
        )
        baseplate.visual(
            Cylinder(radius=0.013, length=0.0025),
            origin=_axis_origin(_add(pivot_local, _scale(steer_axis, -0.0125)), steer_axis),
            material=dark_steel,
            name="upper_cup_washer",
        )
        baseplate.visual(
            Cylinder(radius=0.010, length=0.010),
            origin=_axis_origin(_add(pivot_local, _scale(steer_axis, -0.0065)), steer_axis),
            material=bushing_amber,
            name="upper_bushing",
        )
        baseplate.visual(
            Cylinder(radius=0.010, length=0.010),
            origin=_axis_origin(_add(pivot_local, _scale(steer_axis, 0.006)), steer_axis),
            material=bushing_amber,
            name="lower_bushing",
        )
        baseplate.inertial = Inertial.from_geometry(
            Box((0.068, 0.056, 0.032)),
            mass=0.28,
            origin=Origin(xyz=(0.0, 0.0, -0.012)),
        )

        hanger = model.part(f"{prefix}_hanger")
        hanger.visual(
            Box((0.024, 0.008, 0.008)),
            origin=Origin(xyz=(0.0, 0.017, -0.004)),
            material=cast_aluminum,
            name="top_pad",
        )
        hanger.visual(
            Box((0.024, 0.008, 0.008)),
            origin=Origin(xyz=(0.0, -0.017, -0.004)),
            material=cast_aluminum,
        )
        hanger.visual(
            Box((0.018, 0.008, 0.014)),
            origin=Origin(xyz=(0.0, 0.017, -0.015)),
            material=cast_aluminum,
            name="upper_neck",
        )
        hanger.visual(
            Box((0.018, 0.008, 0.014)),
            origin=Origin(xyz=(0.0, -0.017, -0.015)),
            material=cast_aluminum,
        )
        hanger.visual(
            Box((0.018, 0.010, 0.016)),
            origin=Origin(xyz=(0.0, 0.015, -0.030)),
            material=cast_aluminum,
            name="lower_neck",
        )
        hanger.visual(
            Box((0.018, 0.010, 0.016)),
            origin=Origin(xyz=(0.0, -0.015, -0.030)),
            material=cast_aluminum,
        )
        hanger.visual(
            Box((0.022, 0.094, 0.014)),
            origin=Origin(xyz=(0.0, 0.0, -0.044)),
            material=cast_aluminum,
            name="center_bar",
        )
        hanger.visual(
            Box((0.016, 0.076, 0.012)),
            origin=Origin(xyz=(0.0, 0.061, -0.050)),
            material=cast_aluminum,
        )
        hanger.visual(
            Box((0.016, 0.076, 0.012)),
            origin=Origin(xyz=(0.0, -0.061, -0.050)),
            material=cast_aluminum,
        )
        hanger.visual(
            Cylinder(radius=0.0085, length=0.020),
            origin=Origin(xyz=(0.0, 0.101, axle_local[2]), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=cast_aluminum,
            name="left_collar",
        )
        hanger.visual(
            Cylinder(radius=0.0085, length=0.020),
            origin=Origin(xyz=(0.0, -0.101, axle_local[2]), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=cast_aluminum,
            name="right_collar",
        )
        hanger.visual(
            Cylinder(radius=0.0035, length=0.286),
            origin=Origin(xyz=axle_local, rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name="axle_rod",
        )
        hanger.inertial = Inertial.from_geometry(
            Box((0.030, 0.288, 0.042)),
            mass=0.42,
            origin=Origin(xyz=(0.0, 0.0, -0.042)),
        )

        for side_name, wheel_y in (("left", 0.132), ("right", -0.132)):
            wheel = model.part(f"{prefix}_{side_name}_wheel")
            wheel.visual(
                wheel_mesh,
                material=wheel_urethane,
                name="wheel_shell",
            )
            inner_center = -0.017 if side_name == "left" else 0.017
            wheel.visual(
                bearing_ring_mesh,
                origin=Origin(xyz=(0.0, inner_center, 0.0)),
                material=cast_aluminum,
                name="bearing_ring",
            )
            wheel.inertial = Inertial.from_geometry(
                Cylinder(radius=wheel_radius, length=wheel_width),
                mass=0.14,
                origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            )
            model.articulation(
                f"{prefix}_{side_name}_wheel_spin",
                ArticulationType.CONTINUOUS,
                parent=hanger,
                child=wheel,
                origin=Origin(xyz=(0.0, wheel_y, axle_local[2])),
                axis=(0.0, 1.0, 0.0),
                motion_limits=MotionLimits(effort=8.0, velocity=32.0),
            )

        model.articulation(
            f"{prefix}_adapter_mount",
            ArticulationType.FIXED,
            parent=deck,
            child=adapter,
            origin=Origin(xyz=(x_pos, 0.0, -0.009)),
        )
        model.articulation(
            f"{prefix}_baseplate_mount",
            ArticulationType.FIXED,
            parent=adapter,
            child=baseplate,
            origin=Origin(xyz=(0.0, 0.0, -0.012)),
        )
        model.articulation(
            f"{prefix}_truck_steer",
            ArticulationType.REVOLUTE,
            parent=baseplate,
            child=hanger,
            origin=Origin(xyz=pivot_local),
            axis=steer_axis,
            motion_limits=MotionLimits(
                effort=18.0,
                velocity=2.0,
                lower=-0.38,
                upper=0.38,
            ),
        )

    add_truck("front", 0.185, 0.38)
    add_truck("rear", -0.185, -0.38)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    front_hatch = object_model.get_part("front_service_hatch")
    rear_hatch = object_model.get_part("rear_service_hatch")
    front_adapter = object_model.get_part("front_adapter")
    rear_adapter = object_model.get_part("rear_adapter")
    front_baseplate = object_model.get_part("front_baseplate")
    rear_baseplate = object_model.get_part("rear_baseplate")
    front_hanger = object_model.get_part("front_hanger")
    rear_hanger = object_model.get_part("rear_hanger")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")

    front_steer = object_model.get_articulation("front_truck_steer")
    rear_steer = object_model.get_articulation("rear_truck_steer")
    wheel_spins = [
        object_model.get_articulation("front_left_wheel_spin"),
        object_model.get_articulation("front_right_wheel_spin"),
        object_model.get_articulation("rear_left_wheel_spin"),
        object_model.get_articulation("rear_right_wheel_spin"),
    ]

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(front_hatch, deck, name="front_service_hatch_is_seated")
    ctx.expect_contact(rear_hatch, deck, name="rear_service_hatch_is_seated")
    ctx.expect_contact(front_adapter, deck, name="front_adapter_contacts_deck")
    ctx.expect_contact(rear_adapter, deck, name="rear_adapter_contacts_deck")
    ctx.expect_contact(front_baseplate, front_adapter, name="front_baseplate_bolts_to_adapter")
    ctx.expect_contact(rear_baseplate, rear_adapter, name="rear_baseplate_bolts_to_adapter")
    ctx.expect_contact(front_hanger, front_baseplate, name="front_hanger_supported_at_bushings")
    ctx.expect_contact(rear_hanger, rear_baseplate, name="rear_hanger_supported_at_bushings")

    ctx.expect_origin_distance(
        front_left_wheel,
        front_right_wheel,
        axes="y",
        min_dist=0.24,
        max_dist=0.28,
        name="front_track_width_is_realistic",
    )
    ctx.expect_origin_distance(
        rear_left_wheel,
        rear_right_wheel,
        axes="y",
        min_dist=0.24,
        max_dist=0.28,
        name="rear_track_width_is_realistic",
    )

    ctx.check(
        "wheel_spin_axes_follow_axles",
        all(
            joint.articulation_type == ArticulationType.CONTINUOUS
            and joint.axis == (0.0, 1.0, 0.0)
            for joint in wheel_spins
        ),
        "All wheel spin joints should be continuous and aligned to the truck axle direction.",
    )
    ctx.check(
        "steer_axes_are_mirrored_kingpins",
        front_steer.axis[2] < -0.8
        and rear_steer.axis[2] < -0.8
        and front_steer.axis[0] > 0.2
        and rear_steer.axis[0] < -0.2,
        "Front and rear steer joints should use diagonally tilted kingpin axes with mirrored fore-aft lean.",
    )

    with ctx.pose({front_steer: 0.30, rear_steer: 0.30}):
        ctx.expect_gap(
            deck,
            front_left_wheel,
            axis="z",
            min_gap=0.010,
            name="front_left_wheel_clears_deck_when_steered",
        )
        ctx.expect_gap(
            deck,
            rear_left_wheel,
            axis="z",
            min_gap=0.010,
            name="rear_left_wheel_clears_deck_when_steered",
        )
        steered_front_left = ctx.part_world_position(front_left_wheel)
        steered_front_right = ctx.part_world_position(front_right_wheel)
        steered_rear_left = ctx.part_world_position(rear_left_wheel)
        steered_rear_right = ctx.part_world_position(rear_right_wheel)

    with ctx.pose({front_steer: 0.0, rear_steer: 0.0}):
        neutral_front_left = ctx.part_world_position(front_left_wheel)
        neutral_front_right = ctx.part_world_position(front_right_wheel)
        neutral_rear_left = ctx.part_world_position(rear_left_wheel)
        neutral_rear_right = ctx.part_world_position(rear_right_wheel)

    ctx.check(
        "front_truck_steer_advances_left_wheel_and_retracts_right",
        steered_front_left is not None
        and neutral_front_left is not None
        and steered_front_right is not None
        and neutral_front_right is not None
        and steered_front_left[0] > neutral_front_left[0] + 0.010
        and steered_front_right[0] < neutral_front_right[0] - 0.010,
        "Positive front steer should swing the left wheel forward and the right wheel rearward.",
    )
    ctx.check(
        "rear_truck_steer_advances_left_wheel_and_retracts_right",
        steered_rear_left is not None
        and neutral_rear_left is not None
        and steered_rear_right is not None
        and neutral_rear_right is not None
        and steered_rear_left[0] > neutral_rear_left[0] + 0.010
        and steered_rear_right[0] < neutral_rear_right[0] - 0.010,
        "Positive rear steer should swing the left wheel forward and the right wheel rearward.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
