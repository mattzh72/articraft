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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    wire_from_points,
)


def _build_push_handle_mesh(
    *,
    foot_run: float,
    handle_width: float,
    height: float,
    tube_radius: float,
):
    return mesh_from_geometry(
        wire_from_points(
            [
                (foot_run, -handle_width * 0.5, tube_radius),
                (0.0, -handle_width * 0.5, tube_radius),
                (0.0, -handle_width * 0.5, height),
                (0.0, handle_width * 0.5, height),
                (0.0, handle_width * 0.5, tube_radius),
                (foot_run, handle_width * 0.5, tube_radius),
            ],
            radius=tube_radius,
            radial_segments=18,
            cap_ends=True,
            corner_mode="fillet",
            corner_radius=0.045,
            corner_segments=10,
        ),
        "warehouse_cart_push_handle",
    )

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="warehouse_platform_cart")

    deck_blue = model.material("deck_blue", rgba=(0.14, 0.31, 0.63, 1.0))
    deck_black = model.material("deck_black", rgba=(0.09, 0.09, 0.10, 1.0))
    steel = model.material("steel", rgba=(0.73, 0.75, 0.78, 1.0))
    caster_steel = model.material("caster_steel", rgba=(0.40, 0.43, 0.47, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.08, 0.08, 0.08, 1.0))

    deck_length = 0.90
    deck_width = 0.60
    deck_body_thickness = 0.030
    deck_mat_thickness = 0.005
    deck_body_bottom_z = 0.155
    deck_body_top_z = deck_body_bottom_z + deck_body_thickness
    deck_top_z = deck_body_top_z + deck_mat_thickness

    mount_pad_size = (0.09, 0.07, 0.008)
    caster_mount_z = deck_body_bottom_z - mount_pad_size[2]
    caster_x = deck_length * 0.5 - 0.10
    caster_y = deck_width * 0.5 - 0.10

    wheel_radius = 0.060
    wheel_tread_width = 0.024
    fork_trail = 0.034
    axle_drop = caster_mount_z - wheel_radius
    fork_leg_y = 0.015

    deck = model.part("deck")
    deck.visual(
        Box((deck_length, deck_width, deck_body_thickness)),
        origin=Origin(xyz=(0.0, 0.0, deck_body_bottom_z + deck_body_thickness * 0.5)),
        material=deck_blue,
        name="deck_body",
    )
    deck.visual(
        Box((deck_length - 0.04, deck_width - 0.04, deck_mat_thickness)),
        origin=Origin(xyz=(0.0, 0.0, deck_body_top_z + deck_mat_thickness * 0.5)),
        material=deck_black,
        name="deck_mat",
    )
    for mount_x in (-caster_x, caster_x):
        for mount_y in (-caster_y, caster_y):
            deck.visual(
                Box(mount_pad_size),
                origin=Origin(
                    xyz=(
                        mount_x,
                        mount_y,
                        deck_body_bottom_z - mount_pad_size[2] * 0.5,
                    )
                ),
                material=caster_steel,
                name=f"mount_{'rear' if mount_x < 0.0 else 'front'}_{'left' if mount_y > 0.0 else 'right'}",
            )
    deck.inertial = Inertial.from_geometry(
        Box((deck_length, deck_width, deck_top_z)),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, deck_top_z * 0.5)),
    )

    handle = model.part("push_handle")
    handle_tube_radius = 0.016
    handle.visual(
        _build_push_handle_mesh(
            foot_run=0.11,
            handle_width=0.44,
            height=0.80,
            tube_radius=handle_tube_radius,
        ),
        material=steel,
        name="handle_tube",
    )
    for side_y, side_name in ((-0.22, "right"), (0.22, "left")):
        handle.visual(
            Box((0.11, 0.05, 0.006)),
            origin=Origin(xyz=(0.055, side_y, 0.003)),
            material=caster_steel,
            name=f"{side_name}_foot_plate",
        )
    handle.inertial = Inertial.from_geometry(
        Box((0.13, 0.48, 0.82)),
        mass=4.5,
        origin=Origin(xyz=(0.045, 0.0, 0.41)),
    )
    model.articulation(
        "deck_to_handle",
        ArticulationType.FIXED,
        parent=deck,
        child=handle,
        origin=Origin(xyz=(-deck_length * 0.5 + 0.02, 0.0, deck_top_z)),
    )

    def add_caster(base_name: str, mount_x: float, mount_y: float) -> None:
        fork = model.part(f"{base_name}_fork")
        fork.visual(
            Box((0.078, 0.058, 0.006)),
            origin=Origin(xyz=(0.0, 0.0, -0.003)),
            material=caster_steel,
            name="swivel_plate",
        )
        fork.visual(
            Cylinder(radius=0.011, length=0.020),
            origin=Origin(xyz=(0.0, 0.0, -0.013)),
            material=steel,
            name="swivel_race",
        )
        fork.visual(
            Box((0.038, 0.074, 0.010)),
            origin=Origin(xyz=(0.018, 0.0, -0.016)),
            material=caster_steel,
            name="fork_bridge",
        )
        fork.visual(
            Box((0.008, 0.006, 0.076)),
            origin=Origin(xyz=(0.036, -fork_leg_y, -0.059)),
            material=caster_steel,
            name="right_leg",
        )
        fork.visual(
            Box((0.008, 0.006, 0.076)),
            origin=Origin(xyz=(0.036, fork_leg_y, -0.059)),
            material=caster_steel,
            name="left_leg",
        )
        fork.inertial = Inertial.from_geometry(
            Box((0.09, 0.06, 0.11)),
            mass=1.4,
            origin=Origin(xyz=(0.010, 0.0, -0.052)),
        )

        wheel = model.part(f"{base_name}_wheel")
        wheel.visual(
            Cylinder(radius=wheel_radius, length=wheel_tread_width),
            origin=Origin(rpy=(-pi * 0.5, 0.0, 0.0)),
            material=wheel_rubber,
            name="tread",
        )
        wheel.visual(
            Cylinder(radius=0.014, length=wheel_tread_width),
            origin=Origin(rpy=(-pi * 0.5, 0.0, 0.0)),
            material=steel,
            name="hub",
        )
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=wheel_radius, length=wheel_tread_width),
            mass=1.1,
            origin=Origin(rpy=(-pi * 0.5, 0.0, 0.0)),
        )

        model.articulation(
            f"{base_name}_swivel",
            ArticulationType.CONTINUOUS,
            parent=deck,
            child=fork,
            origin=Origin(xyz=(mount_x, mount_y, caster_mount_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=12.0, velocity=6.0),
        )
        model.articulation(
            f"{base_name}_spin",
            ArticulationType.CONTINUOUS,
            parent=fork,
            child=wheel,
            origin=Origin(xyz=(fork_trail, 0.0, -axle_drop)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=18.0),
        )

    add_caster("front_left", caster_x, caster_y)
    add_caster("front_right", caster_x, -caster_y)
    add_caster("rear_left", -caster_x, caster_y)
    add_caster("rear_right", -caster_x, -caster_y)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    handle = object_model.get_part("push_handle")
    caster_names = ("front_left", "front_right", "rear_left", "rear_right")
    forks = {name: object_model.get_part(f"{name}_fork") for name in caster_names}
    wheels = {name: object_model.get_part(f"{name}_wheel") for name in caster_names}
    swivels = {name: object_model.get_articulation(f"{name}_swivel") for name in caster_names}
    spins = {name: object_model.get_articulation(f"{name}_spin") for name in caster_names}

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
    for name in caster_names:
        ctx.allow_overlap(
            forks[name],
            wheels[name],
            elem_a="right_leg",
            elem_b="tread",
            reason="Simplified rectangular caster fork side plate stays near-tangent to the tire envelope through swivel poses.",
        )
        ctx.allow_overlap(
            forks[name],
            wheels[name],
            elem_a="left_leg",
            elem_b="tread",
            reason="Simplified rectangular caster fork side plate stays near-tangent to the tire envelope through swivel poses.",
        )
        ctx.allow_overlap(
            forks[name],
            wheels[name],
            elem_a="right_leg",
            elem_b="hub",
            reason="Simplified hub cylinder slightly intrudes into the stamped fork-side envelope while representing a real axle-side hub seat.",
        )
        ctx.allow_overlap(
            forks[name],
            wheels[name],
            elem_a="left_leg",
            elem_b="hub",
            reason="Simplified hub cylinder slightly intrudes into the stamped fork-side envelope while representing a real axle-side hub seat.",
        )
    ctx.fail_if_parts_overlap_in_current_pose()

    deck_aabb = ctx.part_world_aabb(deck)
    handle_aabb = ctx.part_world_aabb(handle)
    assert deck_aabb is not None
    assert handle_aabb is not None

    deck_length = deck_aabb[1][0] - deck_aabb[0][0]
    deck_width = deck_aabb[1][1] - deck_aabb[0][1]
    deck_top = deck_aabb[1][2]
    handle_height = handle_aabb[1][2] - deck_top
    ctx.check(
        "deck_length_realistic",
        0.85 <= deck_length <= 0.95,
        f"Expected a warehouse cart deck around 0.9 m long, got {deck_length:.3f} m.",
    )
    ctx.check(
        "deck_width_realistic",
        0.55 <= deck_width <= 0.65,
        f"Expected a warehouse cart deck around 0.6 m wide, got {deck_width:.3f} m.",
    )
    ctx.check(
        "handle_height_realistic",
        0.70 <= handle_height <= 0.85,
        f"Expected handle rise above deck in the 0.70-0.85 m range, got {handle_height:.3f} m.",
    )
    ctx.check(
        "handle_on_short_end",
        handle_aabb[1][0] < deck_aabb[0][0] + 0.18,
        "Push handle should rise from one short end of the platform.",
    )

    ctx.expect_contact(handle, deck, name="handle_contacts_deck")
    ctx.expect_overlap(handle, deck, axes="xy", min_overlap=0.04, name="handle_mounts_over_deck")

    for name in caster_names:
        fork = forks[name]
        wheel = wheels[name]
        swivel = swivels[name]
        spin = spins[name]

        ctx.check(
            f"{swivel.name}_axis_vertical",
            tuple(swivel.axis) == (0.0, 0.0, 1.0),
            f"{swivel.name} should swivel around the vertical z axis, got {swivel.axis}.",
        )
        ctx.check(
            f"{spin.name}_axis_axle",
            tuple(spin.axis) == (0.0, 1.0, 0.0),
            f"{spin.name} should spin around the wheel axle on y, got {spin.axis}.",
        )
        ctx.check(
            f"{swivel.name}_continuous_joint",
            swivel.articulation_type == ArticulationType.CONTINUOUS,
            f"{swivel.name} should be a continuous swivel joint.",
        )
        ctx.check(
            f"{spin.name}_continuous_joint",
            spin.articulation_type == ArticulationType.CONTINUOUS,
            f"{spin.name} should be a continuous wheel-spin joint.",
        )

        ctx.expect_contact(fork, deck, name=f"{name}_fork_contacts_deck")
        ctx.expect_overlap(fork, deck, axes="xy", min_overlap=0.05, name=f"{name}_fork_under_mount")
        ctx.expect_contact(wheel, fork, name=f"{name}_wheel_contacts_fork")
        ctx.expect_gap(
            deck,
            wheel,
            axis="z",
            min_gap=0.012,
            max_gap=0.040,
            name=f"{name}_wheel_clears_deck",
        )

        wheel_aabb = ctx.part_world_aabb(wheel)
        assert wheel_aabb is not None
        ctx.check(
            f"{name}_wheel_ground_height",
            abs(wheel_aabb[0][2]) <= 0.002,
            f"{name} wheel should sit on the ground plane near z=0, got bottom z={wheel_aabb[0][2]:.4f}.",
        )

    with ctx.pose(
        {
            swivels["front_left"]: 1.35,
            swivels["front_right"]: -0.95,
            swivels["rear_left"]: 2.10,
            swivels["rear_right"]: -1.55,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="posed_casters_no_overlap")
        ctx.fail_if_isolated_parts(name="posed_casters_no_floating")
        for name in caster_names:
            ctx.expect_contact(forks[name], deck, name=f"{name}_posed_fork_contacts_deck")
            ctx.expect_contact(wheels[name], forks[name], name=f"{name}_posed_wheel_contacts_fork")
            wheel_aabb = ctx.part_world_aabb(wheels[name])
            assert wheel_aabb is not None
            ctx.check(
                f"{name}_posed_ground_height",
                abs(wheel_aabb[0][2]) <= 0.002,
                f"{name} wheel bottom should stay on the ground plane in a turned pose, got z={wheel_aabb[0][2]:.4f}.",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
