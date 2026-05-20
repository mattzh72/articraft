from __future__ import annotations

import math

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
    model = ArticulatedObject(name="warehouse_platform_cart")

    steel_blue = model.material("powder_coated_blue", rgba=(0.05, 0.17, 0.34, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    galvanized = model.material("galvanized_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    worn_steel = model.material("worn_steel", rgba=(0.42, 0.43, 0.42, 1.0))
    safety_yellow = model.material("worn_safety_yellow", rgba=(0.95, 0.70, 0.10, 1.0))

    deck = model.part("deck")
    deck.visual(
        Box((1.10, 0.62, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.235)),
        material=steel_blue,
        name="flat_deck",
    )
    deck.visual(
        Box((0.94, 0.46, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.269)),
        material=dark_rubber,
        name="ribbed_deck_mat",
    )
    for i, y in enumerate((-0.18, 0.0, 0.18)):
        deck.visual(
            Box((0.90, 0.012, 0.010)),
            origin=Origin(xyz=(0.0, y, 0.276)),
            material=worn_steel,
            name=f"deck_tread_rib_{i}",
        )

    mount_positions = {
        "front_mount_0": (0.43, -0.245, 0.1875),
        "front_mount_1": (0.43, 0.245, 0.1875),
        "rear_mount_0": (-0.43, -0.245, 0.1875),
        "rear_mount_1": (-0.43, 0.245, 0.1875),
    }
    for name, xyz in mount_positions.items():
        deck.visual(
            Box((0.22, 0.18, 0.036)),
            origin=Origin(xyz=xyz),
            material=galvanized,
            name=name,
        )
    deck.visual(
        Box((1.00, 0.035, 0.045)),
        origin=Origin(xyz=(0.0, -0.327, 0.207)),
        material=steel_blue,
        name="side_rail_0",
    )
    deck.visual(
        Box((1.00, 0.035, 0.045)),
        origin=Origin(xyz=(0.0, 0.327, 0.207)),
        material=steel_blue,
        name="side_rail_1",
    )
    deck.visual(
        Box((0.035, 0.58, 0.045)),
        origin=Origin(xyz=(0.557, 0.0, 0.207)),
        material=steel_blue,
        name="front_nose_rail",
    )

    # A simple tubular push handle welded into socket collars at one short end.
    for i, y in enumerate((-0.235, 0.235)):
        deck.visual(
            Cylinder(radius=0.034, length=0.065),
            origin=Origin(xyz=(-0.500, y, 0.292)),
            material=galvanized,
            name=f"handle_socket_{i}",
        )
        deck.visual(
            Cylinder(radius=0.017, length=0.735),
            origin=Origin(xyz=(-0.500, y, 0.615)),
            material=safety_yellow,
            name=f"handle_upright_{i}",
        )
    deck.visual(
        Cylinder(radius=0.017, length=0.505),
        origin=Origin(xyz=(-0.500, 0.0, 0.980), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=safety_yellow,
        name="handle_grip",
    )
    deck.visual(
        Cylinder(radius=0.014, length=0.470),
        origin=Origin(xyz=(-0.500, 0.0, 0.430), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=safety_yellow,
        name="handle_crossbar",
    )

    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.052,
            0.042,
            rim=WheelRim(
                inner_radius=0.034,
                flange_height=0.005,
                flange_thickness=0.003,
                bead_seat_depth=0.003,
            ),
            hub=WheelHub(
                radius=0.020,
                width=0.036,
                cap_style="domed",
                bolt_pattern=BoltPattern(
                    count=4,
                    circle_diameter=0.026,
                    hole_diameter=0.0035,
                ),
            ),
            face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="straight", count=6, thickness=0.0035, window_radius=0.007),
            bore=WheelBore(style="round", diameter=0.020),
        ),
        "caster_wheel_rim",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.075,
            0.052,
            inner_radius=0.050,
            tread=TireTread(style="block", depth=0.004, count=18, land_ratio=0.62),
            grooves=(TireGroove(center_offset=0.0, width=0.006, depth=0.0025),),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
            shoulder=TireShoulder(width=0.006, radius=0.003),
        ),
        "caster_rubber_tire",
    )

    def add_fork_visuals(fork, *, caster_trail: float) -> None:
        axle_x = -caster_trail
        fork.visual(
            Cylinder(radius=0.055, length=0.018),
            origin=Origin(xyz=(0.0, 0.0, -0.009)),
            material=galvanized,
            name="swivel_plate",
        )
        fork.visual(
            Cylinder(radius=0.018, length=0.012),
            origin=Origin(xyz=(0.0, 0.0, -0.006)),
            material=galvanized,
            name="swivel_stem",
        )
        fork.visual(
            Box((0.110 + caster_trail, 0.096, 0.014)),
            origin=Origin(xyz=(-caster_trail / 2.0, 0.0, -0.012)),
            material=galvanized,
            name="fork_crown",
        )
        for i, y in enumerate((-0.041, 0.041)):
            fork.visual(
                Box((0.078, 0.014, 0.122)),
                origin=Origin(xyz=(axle_x, y, -0.079)),
                material=galvanized,
                name=f"fork_cheek_{i}",
            )
        fork.visual(
            Cylinder(radius=0.007, length=0.102),
            origin=Origin(xyz=(axle_x, 0.0, -0.095), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=worn_steel,
            name="axle_pin",
        )

    def add_wheel_visuals(wheel) -> None:
        wheel.visual(wheel_mesh, material=worn_steel, name="metal_rim")
        wheel.visual(tire_mesh, material=dark_rubber, name="rubber_tire")

    caster_specs = (
        ("front_fork_0", "front_wheel_0", (0.43, -0.245, 0.1695), 0.035, True),
        ("front_fork_1", "front_wheel_1", (0.43, 0.245, 0.1695), 0.035, True),
        ("rear_fork_0", "rear_wheel_0", (-0.43, -0.245, 0.1695), 0.0, False),
        ("rear_fork_1", "rear_wheel_1", (-0.43, 0.245, 0.1695), 0.0, False),
    )

    for fork_name, wheel_name, xyz, trail, swivels in caster_specs:
        fork = model.part(fork_name)
        add_fork_visuals(fork, caster_trail=trail)
        wheel = model.part(wheel_name)
        add_wheel_visuals(wheel)

        if swivels:
            model.articulation(
                f"deck_to_{fork_name}",
                ArticulationType.REVOLUTE,
                parent=deck,
                child=fork,
                origin=Origin(xyz=xyz),
                axis=(0.0, 0.0, 1.0),
                motion_limits=MotionLimits(effort=12.0, velocity=5.0, lower=-math.pi, upper=math.pi),
            )
        else:
            model.articulation(
                f"deck_to_{fork_name}",
                ArticulationType.FIXED,
                parent=deck,
                child=fork,
                origin=Origin(xyz=xyz),
            )

        model.articulation(
            f"{fork_name}_to_{wheel_name}",
            ArticulationType.CONTINUOUS,
            parent=fork,
            child=wheel,
            origin=Origin(xyz=(-trail, 0.0, -0.095), rpy=(0.0, 0.0, math.pi / 2.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=30.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck")
    front_forks = [object_model.get_part("front_fork_0"), object_model.get_part("front_fork_1")]
    rear_forks = [object_model.get_part("rear_fork_0"), object_model.get_part("rear_fork_1")]
    wheels = [
        object_model.get_part("front_wheel_0"),
        object_model.get_part("front_wheel_1"),
        object_model.get_part("rear_wheel_0"),
        object_model.get_part("rear_wheel_1"),
    ]
    front_swivels = [
        object_model.get_articulation("deck_to_front_fork_0"),
        object_model.get_articulation("deck_to_front_fork_1"),
    ]
    wheel_spins = [
        object_model.get_articulation("front_fork_0_to_front_wheel_0"),
        object_model.get_articulation("front_fork_1_to_front_wheel_1"),
        object_model.get_articulation("rear_fork_0_to_rear_wheel_0"),
        object_model.get_articulation("rear_fork_1_to_rear_wheel_1"),
    ]

    ctx.check(
        "four caster wheels spin continuously",
        all(j.articulation_type == ArticulationType.CONTINUOUS for j in wheel_spins),
        details=f"wheel joint types={[j.articulation_type for j in wheel_spins]}",
    )
    ctx.check(
        "front caster forks have vertical swivel axes",
        all(
            j.articulation_type == ArticulationType.REVOLUTE
            and tuple(round(v, 6) for v in j.axis) == (0.0, 0.0, 1.0)
            for j in front_swivels
        ),
        details=f"front swivel axes={[j.axis for j in front_swivels]}",
    )

    for i, fork in enumerate(front_forks):
        ctx.expect_contact(
            fork,
            deck,
            elem_a="swivel_plate",
            elem_b=f"front_mount_{i}",
            contact_tol=0.002,
            name=f"front caster {i} is seated under its corner mount",
        )
    for i, fork in enumerate(rear_forks):
        ctx.expect_contact(
            fork,
            deck,
            elem_a="swivel_plate",
            elem_b=f"rear_mount_{i}",
            contact_tol=0.002,
            name=f"rear caster {i} is seated under its corner mount",
        )

    rest = ctx.part_world_position(wheels[0])
    with ctx.pose({"deck_to_front_fork_0": 0.75}):
        turned = ctx.part_world_position(wheels[0])
    ctx.check(
        "front caster swivel moves the trailed wheel about the vertical kingpin",
        rest is not None
        and turned is not None
        and abs(turned[2] - rest[2]) < 0.002
        and math.hypot(turned[0] - rest[0], turned[1] - rest[1]) > 0.015,
        details=f"rest={rest}, turned={turned}",
    )

    return ctx.report()


object_model = build_object_model()
