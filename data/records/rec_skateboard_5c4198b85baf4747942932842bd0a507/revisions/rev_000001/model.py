from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    LoftSection,
    MotionLimits,
    Origin,
    SectionLoftSpec,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)


def _unit(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    length = sqrt(sum(component * component for component in vector))
    return tuple(component / length for component in vector)


def _deck_section(
    x_pos: float,
    *,
    half_width: float,
    lift: float,
    thickness: float,
    concave: float,
) -> list[tuple[float, float, float]]:
    top_edge_z = lift + thickness * 0.45
    top_mid_z = lift + thickness * 0.40 - concave * 0.55
    top_center_z = lift + thickness * 0.38 - concave
    bottom_edge_z = lift - thickness * 0.30
    bottom_mid_z = lift - thickness * 0.46
    bottom_center_z = lift - thickness * 0.56
    return [
        (x_pos, -half_width, bottom_edge_z),
        (x_pos, -half_width * 0.82, top_edge_z - 0.001),
        (x_pos, -half_width * 0.42, top_mid_z),
        (x_pos, 0.0, top_center_z),
        (x_pos, half_width * 0.42, top_mid_z),
        (x_pos, half_width * 0.82, top_edge_z - 0.001),
        (x_pos, half_width, bottom_edge_z),
        (x_pos, half_width * 0.74, bottom_mid_z),
        (x_pos, 0.0, bottom_center_z),
        (x_pos, -half_width * 0.74, bottom_mid_z),
    ]


def _deck_mesh(name: str, *, inset: float = 0.0, top_bias: float = 0.0):
    sections = (
        (-0.405, 0.046, 0.054),
        (-0.335, 0.094, 0.020),
        (-0.225, 0.103, 0.000),
        (-0.080, 0.105, -0.002),
        (0.080, 0.105, -0.002),
        (0.225, 0.103, 0.000),
        (0.335, 0.094, 0.020),
        (0.405, 0.046, 0.054),
    )
    loft_sections = []
    for x_pos, half_width, lift in sections:
        loft_sections.append(
            LoftSection(
                tuple(
                    _deck_section(
                        x_pos,
                        half_width=max(0.008, half_width - inset),
                        lift=lift + top_bias,
                        thickness=0.012,
                        concave=0.0024,
                    )
                )
            )
        )
    return mesh_from_geometry(
        section_loft(SectionLoftSpec(sections=tuple(loft_sections), cap=True, solid=True)),
        name,
    )


def _wheel_mesh(name: str, *, radius: float, width: float):
    half_width = width * 0.5
    outer_profile = [
        (radius * 0.18, -half_width * 0.80),
        (radius * 0.60, -half_width * 0.80),
        (radius * 0.86, -half_width * 0.60),
        (radius, -half_width * 0.18),
        (radius, half_width * 0.18),
        (radius * 0.86, half_width * 0.60),
        (radius * 0.60, half_width * 0.80),
        (radius * 0.18, half_width * 0.80),
    ]
    inner_profile = [
        (0.0068, -half_width * 0.72),
        (0.0095, -half_width * 0.28),
        (0.0095, half_width * 0.28),
        (0.0068, half_width * 0.72),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=56,
            start_cap="flat",
            end_cap="flat",
        ).rotate_x(pi / 2.0),
        name,
    )


def _add_truck(
    model: ArticulatedObject,
    *,
    deck,
    prefix: str,
    x_pos: float,
    nose_sign: float,
    truck_metal,
    hardware,
    bushing,
    wheel_material,
) -> None:
    upper = model.part(f"{prefix}_upper_body")
    upper.visual(
        Box((0.090, 0.055, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=truck_metal,
        name="baseplate",
    )
    upper.visual(
        Box((0.046, 0.040, 0.020)),
        origin=Origin(xyz=(nose_sign * 0.011, 0.0, 0.002)),
        material=truck_metal,
        name="base_wedge",
    )
    upper.visual(
        Cylinder(radius=0.015, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.007)),
        material=bushing,
        name="lower_bushing",
    )
    upper.visual(
        Box((0.030, 0.018, 0.016)),
        origin=Origin(xyz=(nose_sign * 0.030, 0.0, -0.007)),
        material=truck_metal,
        name="pivot_cup_body",
    )
    for x_bolt in (-0.024, 0.024):
        for y_bolt in (-0.016, 0.016):
            upper.visual(
                Cylinder(radius=0.0032, length=0.008),
                origin=Origin(xyz=(x_bolt, y_bolt, 0.014)),
                material=hardware,
            )
    upper.inertial = Inertial.from_geometry(
        Box((0.090, 0.055, 0.030)),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    fork = model.part(f"{prefix}_fork")
    fork.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
        material=bushing,
        name="pivot_puck",
    )
    fork.visual(
        Box((0.046, 0.036, 0.020)),
        origin=Origin(xyz=(nose_sign * 0.016, 0.0, -0.029)),
        material=truck_metal,
        name="neck",
    )
    fork.visual(
        Box((0.018, 0.028, 0.026)),
        origin=Origin(xyz=(nose_sign * 0.027, -0.023, -0.052)),
        material=truck_metal,
        name="left_stanchion",
    )
    fork.visual(
        Box((0.018, 0.028, 0.026)),
        origin=Origin(xyz=(nose_sign * 0.027, 0.023, -0.052)),
        material=truck_metal,
        name="right_stanchion",
    )
    fork.visual(
        Box((0.030, 0.098, 0.015)),
        origin=Origin(xyz=(nose_sign * 0.010, 0.0, -0.064)),
        material=truck_metal,
        name="hanger",
    )
    fork.inertial = Inertial.from_geometry(
        Box((0.060, 0.110, 0.074)),
        mass=0.26,
        origin=Origin(xyz=(nose_sign * 0.010, 0.0, -0.050)),
    )

    axle = model.part(f"{prefix}_axle")
    axle.visual(
        Cylinder(radius=0.0045, length=0.205),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="axle_rod",
    )
    axle.visual(
        Box((0.022, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.0105)),
        material=truck_metal,
        name="axle_clamp",
    )
    for y_spacer in (-0.0624, 0.0624):
        axle.visual(
            Cylinder(radius=0.010, length=0.010),
            origin=Origin(xyz=(0.0, y_spacer, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=hardware,
        )
    for y_nut in (-0.098, 0.098):
        axle.visual(
            Cylinder(radius=0.007, length=0.008),
            origin=Origin(xyz=(0.0, y_nut, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=hardware,
        )
    axle.inertial = Inertial.from_geometry(
        Box((0.022, 0.205, 0.020)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
    )

    for side_name, y_pos in (("left", 0.081), ("right", -0.081)):
        wheel = model.part(f"{prefix}_{side_name}_wheel")
        wheel.visual(
            _wheel_mesh(f"{prefix}_{side_name}_wheel_shell", radius=0.0275, width=0.034),
            material=wheel_material,
            name="wheel_shell",
        )
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=0.0275, length=0.034),
            mass=0.18,
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        )
        model.articulation(
            f"{prefix}_{side_name}_wheel_spin",
            ArticulationType.CONTINUOUS,
            parent=axle,
            child=wheel,
            origin=Origin(xyz=(0.0, y_pos, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=35.0),
        )

    model.articulation(
        f"deck_to_{prefix}_upper",
        ArticulationType.FIXED,
        parent=deck,
        child=upper,
        origin=Origin(xyz=(x_pos, 0.0, -0.025)),
    )
    model.articulation(
        f"{prefix}_truck_steer",
        ArticulationType.REVOLUTE,
        parent=upper,
        child=fork,
        origin=Origin(),
        axis=_unit((-nose_sign * 0.62, 0.0, 0.78)),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-0.40, upper=0.40),
    )
    model.articulation(
        f"{prefix}_fork_to_axle",
        ArticulationType.FIXED,
        parent=fork,
        child=axle,
        origin=Origin(xyz=(nose_sign * 0.010, 0.0, -0.040)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="standard_skateboard")

    maple = model.material("maple", rgba=(0.73, 0.56, 0.34, 1.0))
    grip = model.material("grip", rgba=(0.10, 0.10, 0.11, 1.0))
    truck_metal = model.material("truck_metal", rgba=(0.72, 0.74, 0.76, 1.0))
    hardware = model.material("hardware", rgba=(0.31, 0.33, 0.36, 1.0))
    bushing = model.material("bushing", rgba=(0.88, 0.26, 0.24, 1.0))
    wheel_material = model.material("wheel_material", rgba=(0.94, 0.93, 0.88, 1.0))

    deck = model.part("deck")
    deck.visual(_deck_mesh("skateboard_deck_shell"), material=maple, name="deck_shell")
    deck.visual(
        _deck_mesh("skateboard_grip_tape", inset=0.006, top_bias=0.0012),
        material=grip,
        name="grip_tape",
    )
    deck.inertial = Inertial.from_geometry(
        Box((0.81, 0.21, 0.03)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    _add_truck(
        model,
        deck=deck,
        prefix="front",
        x_pos=0.225,
        nose_sign=1.0,
        truck_metal=truck_metal,
        hardware=hardware,
        bushing=bushing,
        wheel_material=wheel_material,
    )
    _add_truck(
        model,
        deck=deck,
        prefix="rear",
        x_pos=-0.225,
        nose_sign=-1.0,
        truck_metal=truck_metal,
        hardware=hardware,
        bushing=bushing,
        wheel_material=wheel_material,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    front_upper = object_model.get_part("front_upper_body")
    rear_upper = object_model.get_part("rear_upper_body")
    front_axle = object_model.get_part("front_axle")
    rear_axle = object_model.get_part("rear_axle")
    front_fork = object_model.get_part("front_fork")
    rear_fork = object_model.get_part("rear_fork")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")
    front_steer = object_model.get_articulation("front_truck_steer")
    rear_steer = object_model.get_articulation("rear_truck_steer")

    ctx.expect_gap(
        deck,
        front_upper,
        axis="z",
        max_gap=0.0025,
        max_penetration=0.0,
        positive_elem="deck_shell",
        negative_elem="baseplate",
        name="front truck upper body seats directly beneath deck",
    )
    ctx.expect_gap(
        deck,
        rear_upper,
        axis="z",
        max_gap=0.0025,
        max_penetration=0.0,
        positive_elem="deck_shell",
        negative_elem="baseplate",
        name="rear truck upper body seats directly beneath deck",
    )
    ctx.expect_contact(
        front_axle,
        front_fork,
        elem_a="axle_clamp",
        elem_b="hanger",
        name="front axle module bolts to fork hanger",
    )
    ctx.expect_contact(
        rear_axle,
        rear_fork,
        elem_a="axle_clamp",
        elem_b="hanger",
        name="rear axle module bolts to fork hanger",
    )
    ctx.expect_origin_gap(
        front_left_wheel,
        front_axle,
        axis="y",
        min_gap=0.07,
        name="front left wheel sits on positive axle end",
    )
    ctx.expect_origin_gap(
        front_axle,
        front_right_wheel,
        axis="y",
        min_gap=0.07,
        name="front right wheel sits on negative axle end",
    )
    ctx.expect_origin_gap(
        rear_left_wheel,
        rear_axle,
        axis="y",
        min_gap=0.07,
        name="rear left wheel sits on positive axle end",
    )
    ctx.expect_origin_gap(
        rear_axle,
        rear_right_wheel,
        axis="y",
        min_gap=0.07,
        name="rear right wheel sits on negative axle end",
    )

    with ctx.pose({front_steer: 0.30, rear_steer: -0.30}):
        ctx.expect_gap(
            deck,
            front_left_wheel,
            axis="z",
            min_gap=0.010,
            name="front left wheel clears deck while steered",
        )
        ctx.expect_gap(
            deck,
            front_right_wheel,
            axis="z",
            min_gap=0.010,
            name="front right wheel clears deck while steered",
        )
        ctx.expect_gap(
            deck,
            rear_left_wheel,
            axis="z",
            min_gap=0.010,
            name="rear left wheel clears deck while steered",
        )
        ctx.expect_gap(
            deck,
            rear_right_wheel,
            axis="z",
            min_gap=0.010,
            name="rear right wheel clears deck while steered",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
