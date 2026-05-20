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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    wire_from_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _wheel_spin_origin() -> Origin:
    return Origin(rpy=(pi / 2.0, 0.0, 0.0))


def _build_handle_mesh(handle_span: float, handle_height: float, tube_radius: float):
    return wire_from_points(
        [
            (0.0, -handle_span * 0.5, 0.0),
            (0.0, -handle_span * 0.5, handle_height),
            (0.0, handle_span * 0.5, handle_height),
            (0.0, handle_span * 0.5, 0.0),
        ],
        radius=tube_radius,
        radial_segments=18,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.055,
        corner_segments=10,
    )


def _add_caster_fork_visuals(
    part,
    *,
    trail: float,
    axle_drop: float,
    wheel_width: float,
    arm_clearance: float,
    side_plate_thickness: float,
    caster_metal,
) -> None:
    yoke_outer = wheel_width + 2.0 * (arm_clearance + side_plate_thickness)
    arm_y = wheel_width * 0.5 + arm_clearance + side_plate_thickness * 0.5

    part.visual(
        Cylinder(radius=0.024, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=caster_metal,
        name="swivel_housing",
    )
    part.visual(
        Cylinder(radius=0.013, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, -0.034)),
        material=caster_metal,
        name="kingpin_stem",
    )
    part.visual(
        Box((0.018, yoke_outer, 0.016)),
        origin=Origin(xyz=(-trail * 0.20, 0.0, -0.046)),
        material=caster_metal,
        name="fork_crown_block",
    )
    for side_sign, suffix in ((-1.0, "left"), (1.0, "right")):
        part.visual(
            Box((trail + 0.020, side_plate_thickness, 0.018)),
            origin=Origin(
                xyz=(
                    -trail * 0.5,
                    side_sign * arm_y,
                    -0.053,
                )
            ),
            material=caster_metal,
            name=f"fork_rail_{suffix}",
        )
        part.visual(
            Box((0.010, side_plate_thickness, axle_drop - 0.058)),
            origin=Origin(
                xyz=(
                    -trail,
                    side_sign * arm_y,
                    -(0.058 + axle_drop) * 0.5,
                )
            ),
            material=caster_metal,
            name=f"fork_arm_{suffix}",
        )
        part.visual(
            Box((0.014, 0.019, 0.022)),
            origin=Origin(
                xyz=(
                    -trail,
                    side_sign * arm_y,
                    -axle_drop,
                )
            ),
            material=caster_metal,
            name=f"axle_tab_{suffix}",
        )


def _add_wheel_visuals(part, *, wheel_radius: float, wheel_width: float, rubber, hub_metal) -> None:
    spin_origin = _wheel_spin_origin()
    part.visual(
        Cylinder(radius=wheel_radius, length=wheel_width),
        origin=spin_origin,
        material=rubber,
        name="tire",
    )
    part.visual(
        Cylinder(radius=wheel_radius * 0.62, length=wheel_width * 0.72),
        origin=spin_origin,
        material=hub_metal,
        name="hub",
    )
    part.visual(
        Cylinder(radius=wheel_radius * 0.23, length=wheel_width * 0.30),
        origin=spin_origin,
        material=hub_metal,
        name="bearing_core",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flatbed_platform_cart")

    deck_length = 0.86
    deck_width = 0.54
    deck_thickness = 0.036
    deck_corner_radius = 0.026
    tread_thickness = 0.004

    handle_span = deck_width - 0.10
    handle_height = 0.84
    handle_tube_radius = 0.015

    caster_pivot_x = deck_length * 0.5 - 0.100
    caster_pivot_y = deck_width * 0.5 - 0.085
    caster_trail = 0.045
    caster_axle_drop = 0.108
    wheel_radius = 0.055
    wheel_width = 0.032
    fork_side_plate_thickness = 0.007
    fork_arm_clearance = 0.006

    platform_blue = model.material("platform_blue", rgba=(0.18, 0.35, 0.60, 1.0))
    tread_black = model.material("tread_black", rgba=(0.12, 0.13, 0.14, 1.0))
    steel_gray = model.material("steel_gray", rgba=(0.56, 0.58, 0.62, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.24, 0.25, 0.27, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.08, 0.08, 0.08, 1.0))

    deck = model.part("deck")
    deck.inertial = Inertial.from_geometry(
        Box((deck_length, deck_width, 0.14)),
        mass=19.0,
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
    )

    deck_shell = _save_mesh(
        "cart_deck_shell",
        ExtrudeGeometry(
            rounded_rect_profile(deck_length, deck_width, deck_corner_radius),
            deck_thickness,
            center=True,
        ),
    )
    deck_tread = _save_mesh(
        "cart_deck_tread",
        ExtrudeGeometry(
            rounded_rect_profile(deck_length - 0.070, deck_width - 0.070, deck_corner_radius * 0.7),
            tread_thickness,
            center=True,
        ),
    )
    deck.visual(deck_shell, material=platform_blue, name="deck_shell")
    deck.visual(
        deck_tread,
        origin=Origin(xyz=(0.0, 0.0, deck_thickness * 0.5 + tread_thickness * 0.5)),
        material=tread_black,
        name="deck_surface",
    )
    deck.visual(
        Box((deck_length - 0.24, 0.050, 0.028)),
        origin=Origin(xyz=(0.0, -0.145, -0.032)),
        material=dark_steel,
        name="underside_rail_left",
    )
    deck.visual(
        Box((deck_length - 0.24, 0.050, 0.028)),
        origin=Origin(xyz=(0.0, 0.145, -0.032)),
        material=dark_steel,
        name="underside_rail_right",
    )
    deck.visual(
        Box((0.080, deck_width - 0.14, 0.028)),
        origin=Origin(xyz=(0.16, 0.0, -0.032)),
        material=dark_steel,
        name="underside_cross_rail",
    )
    for x_sign, y_sign, name_suffix in (
        (1.0, 1.0, "front_left"),
        (1.0, -1.0, "front_right"),
        (-1.0, 1.0, "rear_left"),
        (-1.0, -1.0, "rear_right"),
    ):
        deck.visual(
            Box((0.090, 0.070, 0.006)),
            origin=Origin(
                xyz=(
                    x_sign * caster_pivot_x,
                    y_sign * caster_pivot_y,
                    -deck_thickness * 0.5 - 0.003,
                )
            ),
            material=steel_gray,
            name=f"caster_plate_{name_suffix}",
        )

    hinge_x = -deck_length * 0.5 + 0.008
    hinge_z = 0.045
    bracket_y = handle_span * 0.5 + 0.022
    bracket_height = hinge_z - deck_thickness * 0.5 + 0.028
    bracket_z = deck_thickness * 0.5 + bracket_height * 0.5
    for y_sign, suffix in ((-1.0, "left"), (1.0, "right")):
        deck.visual(
            Box((0.024, 0.022, bracket_height)),
            origin=Origin(
                xyz=(
                    hinge_x + 0.027,
                    y_sign * bracket_y,
                    bracket_z,
                )
            ),
            material=steel_gray,
            name=f"handle_bracket_{suffix}",
        )

    handle = model.part("handle")
    handle.inertial = Inertial.from_geometry(
        Box((0.060, handle_span, handle_height)),
        mass=3.8,
        origin=Origin(xyz=(0.0, 0.0, handle_height * 0.5)),
    )
    handle.visual(
        _save_mesh(
            "cart_handle_frame",
            _build_handle_mesh(handle_span, handle_height, handle_tube_radius),
        ),
        material=steel_gray,
        name="handle_frame",
    )
    handle.visual(
        Box((0.012, handle_span - 0.010, 0.042)),
        origin=Origin(xyz=(0.0, 0.0, 0.360)),
        material=steel_gray,
        name="handle_cross_brace",
    )
    handle.visual(
        Cylinder(radius=0.019, length=handle_span - 0.030),
        origin=Origin(xyz=(0.0, 0.0, handle_height), rpy=(pi / 2.0, 0.0, 0.0)),
        material=tread_black,
        name="handle_grip",
    )
    for y_sign, suffix in ((-1.0, "left"), (1.0, "right")):
        handle.visual(
            Box((0.015, 0.020, 0.030)),
            origin=Origin(
                xyz=(
                    0.0075,
                    y_sign * (handle_span * 0.5 + 0.006),
                    0.015,
                )
            ),
            material=steel_gray,
            name=f"hinge_tab_{suffix}",
        )

    model.articulation(
        "deck_to_handle",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=handle,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=0.0, upper=1.50),
    )

    caster_specs = [
        ("front_left", caster_pivot_x, caster_pivot_y),
        ("front_right", caster_pivot_x, -caster_pivot_y),
        ("rear_left", -caster_pivot_x, caster_pivot_y),
        ("rear_right", -caster_pivot_x, -caster_pivot_y),
    ]

    for name, pivot_x, pivot_y in caster_specs:
        fork = model.part(f"{name}_caster")
        fork.inertial = Inertial.from_geometry(
            Box((caster_trail + 0.060, 0.065, 0.14)),
            mass=0.9,
            origin=Origin(xyz=(-caster_trail * 0.5, 0.0, -0.075)),
        )
        _add_caster_fork_visuals(
            fork,
            trail=caster_trail,
            axle_drop=caster_axle_drop,
            wheel_width=wheel_width,
            arm_clearance=fork_arm_clearance,
            side_plate_thickness=fork_side_plate_thickness,
            caster_metal=dark_steel,
        )

        wheel = model.part(f"{name}_wheel")
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=wheel_radius, length=wheel_width),
            mass=0.75,
            origin=_wheel_spin_origin(),
        )
        _add_wheel_visuals(
            wheel,
            wheel_radius=wheel_radius,
            wheel_width=wheel_width,
            rubber=wheel_rubber,
            hub_metal=steel_gray,
        )

        model.articulation(
            f"{name}_swivel",
            ArticulationType.CONTINUOUS,
            parent=deck,
            child=fork,
            origin=Origin(xyz=(pivot_x, pivot_y, -deck_thickness * 0.5 - 0.006)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=8.0, velocity=8.0),
        )
        model.articulation(
            f"{name}_wheel_spin",
            ArticulationType.CONTINUOUS,
            parent=fork,
            child=wheel,
            origin=Origin(xyz=(-caster_trail, 0.0, -caster_axle_drop)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=6.0, velocity=18.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck")
    handle = object_model.get_part("handle")
    handle_hinge = object_model.get_articulation("deck_to_handle")

    swivel_joint_names = (
        "front_left_swivel",
        "front_right_swivel",
        "rear_left_swivel",
        "rear_right_swivel",
    )
    wheel_joint_names = (
        "front_left_wheel_spin",
        "front_right_wheel_spin",
        "rear_left_wheel_spin",
        "rear_right_wheel_spin",
    )

    ctx.check(
        "handle hinge axis runs along deck width",
        tuple(round(v, 6) for v in handle_hinge.axis) == (0.0, 1.0, 0.0),
        details=f"axis={handle_hinge.axis}",
    )
    ctx.check(
        "all caster swivels rotate on vertical pivots",
        all(tuple(round(v, 6) for v in object_model.get_articulation(name).axis) == (0.0, 0.0, 1.0) for name in swivel_joint_names),
        details=str({name: object_model.get_articulation(name).axis for name in swivel_joint_names}),
    )
    ctx.check(
        "all wheel spins rotate on lateral axles",
        all(tuple(round(v, 6) for v in object_model.get_articulation(name).axis) == (0.0, 1.0, 0.0) for name in wheel_joint_names),
        details=str({name: object_model.get_articulation(name).axis for name in wheel_joint_names}),
    )

    for wheel_name in (
        "front_left_wheel",
        "front_right_wheel",
        "rear_left_wheel",
        "rear_right_wheel",
    ):
        wheel = object_model.get_part(wheel_name)
        ctx.expect_gap(
            deck,
            wheel,
            axis="z",
            min_gap=0.025,
            name=f"{wheel_name} hangs below the deck",
        )

    with ctx.pose({handle_hinge: handle_hinge.motion_limits.upper}):
        ctx.expect_gap(
            handle,
            deck,
            axis="z",
            positive_elem="handle_grip",
            negative_elem="deck_surface",
            min_gap=0.010,
            max_gap=0.070,
            name="folded handle grip clears the deck surface",
        )
        ctx.expect_overlap(
            handle,
            deck,
            axes="xy",
            elem_a="handle_grip",
            elem_b="deck_shell",
            min_overlap=0.020,
            name="folded handle stays over the platform",
        )

    front_left_swivel = object_model.get_articulation("front_left_swivel")
    front_left_wheel = object_model.get_part("front_left_wheel")
    rest_pos = ctx.part_world_position(front_left_wheel)
    with ctx.pose({front_left_swivel: pi / 2.0}):
        turned_pos = ctx.part_world_position(front_left_wheel)

    ctx.check(
        "front left caster swings sideways when the swivel turns",
        rest_pos is not None
        and turned_pos is not None
        and abs(turned_pos[2] - rest_pos[2]) < 1e-6
        and turned_pos[1] < rest_pos[1] - 0.030
        and turned_pos[0] > rest_pos[0] + 0.030,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
