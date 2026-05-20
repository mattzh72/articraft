from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flatbed_platform_cart")

    blue = model.material("blue_powder_coat", rgba=(0.05, 0.22, 0.62, 1.0))
    black = model.material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    dark = model.material("dark_tread_mat", rgba=(0.025, 0.028, 0.026, 1.0))
    steel = model.material("zinc_plated_steel", rgba=(0.72, 0.72, 0.68, 1.0))
    yellow = model.material("yellow_brake_pedal", rgba=(0.95, 0.72, 0.08, 1.0))

    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.075,
            0.045,
            inner_radius=0.050,
            tread=TireTread(style="circumferential", depth=0.0035, count=3, land_ratio=0.58),
            grooves=(TireGroove(center_offset=0.0, width=0.006, depth=0.002),),
            sidewall=TireSidewall(style="rounded", bulge=0.05),
            shoulder=TireShoulder(width=0.006, radius=0.003),
        ),
        "caster_rubber_tire",
    )
    rim_mesh = mesh_from_geometry(
        WheelGeometry(
            0.051,
            0.036,
            rim=WheelRim(inner_radius=0.029, flange_height=0.004, flange_thickness=0.003),
            hub=WheelHub(
                radius=0.018,
                width=0.030,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=4, circle_diameter=0.026, hole_diameter=0.003),
            ),
            face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="straight", count=5, thickness=0.003, window_radius=0.006),
            bore=WheelBore(style="round", diameter=0.010),
        ),
        "caster_steel_rim",
    )

    deck = model.part("deck")
    deck.visual(Box((1.00, 0.58, 0.060)), origin=Origin(xyz=(0.0, 0.0, 0.290)), material=blue, name="deck_pan")
    deck.visual(Box((0.94, 0.50, 0.010)), origin=Origin(xyz=(0.0, 0.0, 0.325)), material=dark, name="non_skid_mat")
    deck.visual(Box((1.04, 0.035, 0.055)), origin=Origin(xyz=(0.0, 0.3075, 0.292)), material=black, name="side_bumper_0")
    deck.visual(Box((1.04, 0.035, 0.055)), origin=Origin(xyz=(0.0, -0.3075, 0.292)), material=black, name="side_bumper_1")
    deck.visual(Box((0.035, 0.58, 0.055)), origin=Origin(xyz=(0.5175, 0.0, 0.292)), material=black, name="end_bumper_0")
    deck.visual(Box((0.035, 0.58, 0.055)), origin=Origin(xyz=(-0.5175, 0.0, 0.292)), material=black, name="end_bumper_1")

    for ix, x in enumerate((-0.39, 0.39)):
        for iy, y in enumerate((-0.22, 0.22)):
            name_suffix = ix * 2 + iy
            deck.visual(
                Box((0.145, 0.120, 0.012)),
                origin=Origin(xyz=(x, y, 0.254)),
                material=steel,
                name=f"caster_plate_{name_suffix}",
            )

    # The handle is a fixed welded part of the deck assembly: two uprights
    # touch the rear deck edge and are joined by a horizontal grip.
    deck.visual(Cylinder(radius=0.018, length=0.56), origin=Origin(xyz=(-0.47, -0.20, 0.600)), material=blue, name="handle_post_0")
    deck.visual(Cylinder(radius=0.018, length=0.56), origin=Origin(xyz=(-0.47, 0.20, 0.600)), material=blue, name="handle_post_1")
    deck.visual(
        Cylinder(radius=0.019, length=0.40),
        origin=Origin(xyz=(-0.47, 0.0, 0.880), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=blue,
        name="handle_grip",
    )
    deck.visual(Box((0.080, 0.070, 0.018)), origin=Origin(xyz=(-0.47, -0.20, 0.334)), material=blue, name="handle_socket_0")
    deck.visual(Box((0.080, 0.070, 0.018)), origin=Origin(xyz=(-0.47, 0.20, 0.334)), material=blue, name="handle_socket_1")

    def add_caster(index: int, x: float, y: float, *, brake_mount: bool = False):
        caster = model.part(f"caster_{index}")
        caster.visual(Cylinder(radius=0.016, length=0.070), origin=Origin(xyz=(0.0, 0.0, -0.035)), material=steel, name="swivel_stem")
        caster.visual(Cylinder(radius=0.043, length=0.016), origin=Origin(xyz=(0.0, 0.0, -0.076)), material=steel, name="swivel_race")
        caster.visual(Box((0.115, 0.045, 0.018)), origin=Origin(xyz=(0.0, 0.0, -0.062)), material=steel, name="fork_bridge")
        caster.visual(Box((0.012, 0.045, 0.148)), origin=Origin(xyz=(-0.036, 0.0, -0.145)), material=steel, name="fork_cheek_0")
        caster.visual(Box((0.012, 0.045, 0.148)), origin=Origin(xyz=(0.036, 0.0, -0.145)), material=steel, name="fork_cheek_1")
        caster.visual(
            Cylinder(radius=0.009, length=0.096),
            origin=Origin(xyz=(0.0, 0.0, -0.155), rpy=(0.0, pi / 2.0, 0.0)),
            material=steel,
            name="wheel_axle",
        )
        if brake_mount:
            caster.visual(Box((0.120, 0.045, 0.016)), origin=Origin(xyz=(0.0, -0.035, -0.066)), material=steel, name="brake_hanger")
            caster.visual(Box((0.016, 0.060, 0.030)), origin=Origin(xyz=(-0.0555, -0.055, -0.086)), material=steel, name="brake_lug_0")
            caster.visual(Box((0.016, 0.060, 0.030)), origin=Origin(xyz=(0.0555, -0.055, -0.086)), material=steel, name="brake_lug_1")
            caster.visual(
                Cylinder(radius=0.004, length=0.135),
                origin=Origin(xyz=(0.0, -0.055, -0.086), rpy=(0.0, pi / 2.0, 0.0)),
                material=steel,
                name="brake_pin",
            )

        model.articulation(
            f"deck_to_caster_{index}",
            ArticulationType.CONTINUOUS,
            parent=deck,
            child=caster,
            origin=Origin(xyz=(x, y, 0.248)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=8.0, velocity=6.0),
        )

        wheel = model.part(f"wheel_{index}")
        wheel.visual(tire_mesh, material=black, name="tire")
        wheel.visual(rim_mesh, material=steel, name="rim")
        model.articulation(
            f"caster_{index}_to_wheel_{index}",
            ArticulationType.CONTINUOUS,
            parent=caster,
            child=wheel,
            origin=Origin(xyz=(0.0, 0.0, -0.155)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=20.0),
        )
        return caster, wheel

    caster_positions = [(-0.39, -0.22), (-0.39, 0.22), (0.39, -0.22), (0.39, 0.22)]
    for i, (x, y) in enumerate(caster_positions):
        add_caster(i, x, y, brake_mount=(i == 0))

    brake_pedal = model.part("brake_pedal")
    brake_pedal.visual(
        Cylinder(radius=0.007, length=0.095),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=yellow,
        name="hinge_sleeve",
    )
    brake_pedal.visual(Box((0.035, 0.100, 0.012)), origin=Origin(xyz=(0.0, -0.050, -0.006)), material=yellow, name="pedal_arm")
    brake_pedal.visual(Box((0.085, 0.060, 0.014)), origin=Origin(xyz=(0.0, -0.115, -0.008)), material=yellow, name="foot_pad")
    brake_pedal.visual(Box((0.070, 0.006, 0.008)), origin=Origin(xyz=(0.0, -0.143, -0.001)), material=black, name="pad_toe_grip")
    model.articulation(
        "caster_0_to_brake_pedal",
        ArticulationType.REVOLUTE,
        parent="caster_0",
        child=brake_pedal,
        origin=Origin(xyz=(0.0, -0.055, -0.086)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.5, lower=0.0, upper=0.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck")
    brake = object_model.get_part("brake_pedal")
    brake_joint = object_model.get_articulation("caster_0_to_brake_pedal")

    ctx.check(
        "four caster assemblies and wheels",
        all(object_model.get_part(f"caster_{i}") is not None and object_model.get_part(f"wheel_{i}") is not None for i in range(4)),
        "Expected four caster yokes and four separate wheel parts.",
    )

    for i in range(4):
        caster = object_model.get_part(f"caster_{i}")
        wheel = object_model.get_part(f"wheel_{i}")
        swivel = object_model.get_articulation(f"deck_to_caster_{i}")
        spin = object_model.get_articulation(f"caster_{i}_to_wheel_{i}")
        ctx.check(f"caster_{i} swivels vertically", swivel.articulation_type == ArticulationType.CONTINUOUS and tuple(swivel.axis) == (0.0, 0.0, 1.0))
        ctx.check(f"wheel_{i} spins on axle", spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (1.0, 0.0, 0.0))
        ctx.expect_gap(deck, caster, axis="z", max_gap=0.002, max_penetration=0.001, name=f"caster_{i} stem seats under deck plate")
        ctx.expect_within(wheel, caster, axes="x", margin=0.002, name=f"wheel_{i} sits between fork cheeks")
        ctx.allow_overlap(
            caster,
            wheel,
            elem_a="wheel_axle",
            elem_b="rim",
            reason="The visible axle is intentionally captured through the rim bore so the wheel reads as spinning on a real pin.",
        )
        ctx.expect_overlap(
            caster,
            wheel,
            axes="x",
            min_overlap=0.035,
            elem_a="wheel_axle",
            elem_b="rim",
            name=f"wheel_{i} axle passes through rim",
        )
        ctx.expect_within(
            caster,
            wheel,
            axes="yz",
            margin=0.012,
            elem_a="wheel_axle",
            elem_b="rim",
            name=f"wheel_{i} axle is centered in rim bore",
        )

    ctx.check(
        "brake pedal has hinged travel",
        brake_joint.articulation_type == ArticulationType.REVOLUTE
        and brake_joint.motion_limits.lower == 0.0
        and brake_joint.motion_limits.upper >= 0.70,
        "Brake pedal should rotate from a raised rest toward the tire.",
    )
    ctx.allow_overlap(
        brake,
        "caster_0",
        elem_a="hinge_sleeve",
        elem_b="brake_pin",
        reason="The brake hinge pin is intentionally nested inside the pedal sleeve to keep the pedal clipped to the corner hinge.",
    )
    ctx.expect_overlap(
        brake,
        "caster_0",
        axes="x",
        min_overlap=0.085,
        elem_a="hinge_sleeve",
        elem_b="brake_pin",
        name="brake sleeve rides on hinge pin",
    )
    ctx.expect_contact(
        brake,
        "caster_0",
        elem_a="hinge_sleeve",
        elem_b="brake_lug_0",
        contact_tol=0.001,
        name="brake sleeve is clipped against hinge lug",
    )

    with ctx.pose({brake_joint: 0.75}):
        ctx.expect_contact(
            brake,
            "wheel_0",
            elem_a="pedal_arm",
            elem_b="tire",
            contact_tol=0.002,
            name="pressed brake pedal rotates down into rear wheel",
        )

    return ctx.report()


object_model = build_object_model()
