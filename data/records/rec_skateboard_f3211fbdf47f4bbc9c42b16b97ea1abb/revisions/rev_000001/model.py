from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireSidewall,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    superellipse_side_loft,
)


DECK_LENGTH = 0.82
DECK_WIDTH = 0.215
TRUCK_X = 0.255
WHEEL_RADIUS = 0.028
WHEEL_WIDTH = 0.034


def _add_deck_bolt_pattern(deck, *, x: float, y_span: float, material) -> None:
    """Small countersunk bolt heads visible through the grip tape."""
    for dx in (-0.020, 0.020):
        for y in (-y_span * 0.5, y_span * 0.5):
            deck.visual(
                Cylinder(radius=0.0020, length=0.0180),
                origin=Origin(xyz=(x + dx, y, 0.0200)),
                material=material,
                name=f"bolt_shank_{'front' if x > 0 else 'rear'}_{dx:+.3f}_{y:+.3f}",
            )
            deck.visual(
                Cylinder(radius=0.0042, length=0.0020),
                origin=Origin(xyz=(x + dx, y, 0.0284)),
                material=material,
                name=f"bolt_{'front' if x > 0 else 'rear'}_{dx:+.3f}_{y:+.3f}",
            )


def _add_truck_visuals(
    truck,
    *,
    hanger_width: float,
    axle_length: float,
    plate_width: float,
    metal,
    dark_metal,
    rubber,
) -> None:
    """Build a standard skateboard truck around a local vertical kingpin."""
    truck.visual(
        Box((0.082, plate_width, 0.007)),
        origin=Origin(xyz=(0.0, 0.0, -0.0035)),
        material=metal,
        name="baseplate",
    )
    truck.visual(
        Box((0.050, max(0.035, plate_width * 0.55), 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.0115)),
        material=dark_metal,
        name="kingpin_seat",
    )
    truck.visual(
        Cylinder(radius=0.0055, length=0.058),
        origin=Origin(xyz=(0.0, 0.0, -0.029)),
        material=dark_metal,
        name="kingpin",
    )
    truck.visual(
        Cylinder(radius=0.0155, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=rubber,
        name="upper_bushing",
    )
    truck.visual(
        Cylinder(radius=0.0165, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.038)),
        material=rubber,
        name="lower_bushing",
    )
    truck.visual(
        Box((0.064, 0.058, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, -0.057)),
        material=metal,
        name="hanger_block",
    )
    truck.visual(
        Cylinder(radius=0.0135, length=hanger_width),
        origin=Origin(xyz=(0.0, 0.0, -0.061), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="hanger_bar",
    )
    truck.visual(
        Cylinder(radius=0.0054, length=axle_length),
        origin=Origin(xyz=(0.0, 0.0, -0.061), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="axle",
    )
    for y in (-(axle_length * 0.5 + 0.003), axle_length * 0.5 + 0.003):
        truck.visual(
            Cylinder(radius=0.0085, length=0.008),
            origin=Origin(xyz=(0.0, y, -0.061), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name=f"axle_nut_{'pos' if y > 0 else 'neg'}",
        )
    truck.visual(
        Box((0.048, 0.036, 0.018)),
        origin=Origin(xyz=(-0.032, 0.0, -0.047)),
        material=metal,
        name="pivot_nose",
    )


def _add_wheel_visuals(wheel, *, tire_mesh, core_mesh, urethane, bearing_metal) -> None:
    # WheelGeometry/TireGeometry spin about their local X axis; the visual
    # rotation aligns that axis to the skateboard axle (local Y of the wheel part).
    axle_aligned = Origin(rpy=(0.0, 0.0, pi / 2.0))
    wheel.visual(
        tire_mesh,
        origin=axle_aligned,
        material=urethane,
        name="urethane",
    )
    wheel.visual(
        core_mesh,
        origin=axle_aligned,
        material=bearing_metal,
        name="bearing_core",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="skateboard_standard_trucks")

    maple = model.material("maple_sidewall", rgba=(0.78, 0.58, 0.34, 1.0))
    grip = model.material("black_grip_tape", rgba=(0.015, 0.015, 0.014, 1.0))
    truck_metal = model.material("brushed_aluminum", rgba=(0.62, 0.63, 0.61, 1.0))
    dark_metal = model.material("dark_axle_steel", rgba=(0.13, 0.14, 0.15, 1.0))
    bushing = model.material("black_bushing_rubber", rgba=(0.03, 0.03, 0.035, 1.0))
    urethane = model.material("warm_white_urethane", rgba=(0.86, 0.82, 0.68, 1.0))
    bearing_metal = model.material("bearing_silver", rgba=(0.72, 0.74, 0.75, 1.0))

    deck = model.part("deck")
    deck_sections = [
        (-0.410, 0.041, 0.050, 0.018),
        (-0.365, 0.032, 0.142, 0.016),
        (-0.285, 0.023, 0.202, 0.014),
        (-0.150, 0.020, DECK_WIDTH, 0.012),
        (0.000, 0.020, DECK_WIDTH, 0.012),
        (0.150, 0.020, DECK_WIDTH, 0.012),
        (0.285, 0.023, 0.202, 0.014),
        (0.365, 0.032, 0.142, 0.016),
        (0.410, 0.041, 0.050, 0.018),
    ]
    deck.visual(
        mesh_from_geometry(
            superellipse_side_loft(deck_sections, exponents=2.45, segments=64),
            "skateboard_kicktail_deck",
        ),
        material=maple,
        name="curved_deck",
    )
    deck.visual(
        Box((0.610, 0.166, 0.0020)),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=grip,
        name="grip_tape",
    )
    deck.visual(
        Box((0.078, 0.072, 0.012)),
        origin=Origin(xyz=(TRUCK_X, 0.0, 0.0180)),
        material=grip,
        name="front_support",
    )
    deck.visual(
        Box((0.092, 0.138, 0.012)),
        origin=Origin(xyz=(-TRUCK_X, 0.0, 0.0180)),
        material=grip,
        name="rear_support",
    )
    _add_deck_bolt_pattern(deck, x=TRUCK_X, y_span=0.044, material=dark_metal)
    _add_deck_bolt_pattern(deck, x=-TRUCK_X, y_span=0.094, material=dark_metal)
    deck.inertial = Inertial.from_geometry(
        Box((DECK_LENGTH, DECK_WIDTH, 0.022)),
        mass=1.25,
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
    )

    tire_mesh = mesh_from_geometry(
        TireGeometry(
            WHEEL_RADIUS,
            WHEEL_WIDTH,
            inner_radius=0.0145,
            sidewall=TireSidewall(style="rounded", bulge=0.08),
        ),
        "skateboard_wheel_urethane",
    )
    core_mesh = mesh_from_geometry(
        WheelGeometry(
            0.0143,
            0.030,
            rim=WheelRim(inner_radius=0.0105, flange_height=0.0020, flange_thickness=0.0015),
            hub=WheelHub(
                radius=0.0075,
                width=0.032,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=5, circle_diameter=0.010, hole_diameter=0.0012),
            ),
            face=WheelFace(dish_depth=0.0015, front_inset=0.001, rear_inset=0.001),
            spokes=WheelSpokes(style="straight", count=5, thickness=0.0012, window_radius=0.0025),
            bore=WheelBore(style="round", diameter=0.010),
        ),
        "skateboard_wheel_bearing_core",
    )

    front_truck = model.part("front_truck")
    _add_truck_visuals(
        front_truck,
        hanger_width=0.166,
        axle_length=0.292,
        plate_width=0.068,
        metal=truck_metal,
        dark_metal=dark_metal,
        rubber=bushing,
    )
    front_truck.inertial = Inertial.from_geometry(
        Box((0.10, 0.18, 0.065)),
        mass=0.36,
        origin=Origin(xyz=(0.0, 0.0, -0.042)),
    )

    rear_truck = model.part("rear_truck")
    _add_truck_visuals(
        rear_truck,
        hanger_width=0.218,
        axle_length=0.352,
        plate_width=0.128,
        metal=truck_metal,
        dark_metal=dark_metal,
        rubber=bushing,
    )
    rear_truck.inertial = Inertial.from_geometry(
        Box((0.11, 0.24, 0.065)),
        mass=0.44,
        origin=Origin(xyz=(0.0, 0.0, -0.042)),
    )

    wheel_specs = [
        ("front_wheel_0", front_truck, "front_truck_to_wheel_0", 0.128),
        ("front_wheel_1", front_truck, "front_truck_to_wheel_1", -0.128),
        ("rear_wheel_0", rear_truck, "rear_truck_to_wheel_0", 0.158),
        ("rear_wheel_1", rear_truck, "rear_truck_to_wheel_1", -0.158),
    ]
    wheels = {}
    for wheel_name, _parent, _joint_name, _y in wheel_specs:
        wheel = model.part(wheel_name)
        _add_wheel_visuals(
            wheel,
            tire_mesh=tire_mesh,
            core_mesh=core_mesh,
            urethane=urethane,
            bearing_metal=bearing_metal,
        )
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
            mass=0.090,
            origin=Origin(rpy=(0.0, 0.0, pi / 2.0)),
        )
        wheels[wheel_name] = wheel

    model.articulation(
        "front_kingpin",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=front_truck,
        origin=Origin(xyz=(TRUCK_X, 0.0, 0.0120)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=-0.45, upper=0.45),
    )
    model.articulation(
        "rear_kingpin",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=rear_truck,
        origin=Origin(xyz=(-TRUCK_X, 0.0, 0.0120)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.5, velocity=2.5, lower=-0.42, upper=0.42),
    )

    for wheel_name, truck, joint_name, y in wheel_specs:
        model.articulation(
            joint_name,
            ArticulationType.CONTINUOUS,
            parent=truck,
            child=wheels[wheel_name],
            origin=Origin(xyz=(0.0, y, -0.061)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.8, velocity=40.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck")
    front_truck = object_model.get_part("front_truck")
    rear_truck = object_model.get_part("rear_truck")
    front_kingpin = object_model.get_articulation("front_kingpin")
    rear_kingpin = object_model.get_articulation("rear_kingpin")

    for truck_name, wheel_name in (
        ("front_truck", "front_wheel_0"),
        ("front_truck", "front_wheel_1"),
        ("rear_truck", "rear_wheel_0"),
        ("rear_truck", "rear_wheel_1"),
    ):
        truck = object_model.get_part(truck_name)
        wheel = object_model.get_part(wheel_name)
        ctx.allow_overlap(
            truck,
            wheel,
            elem_a="axle",
            elem_b="bearing_core",
            reason="The metal axle is intentionally captured through the wheel bearing bore with a tiny interference fit.",
        )
        ctx.expect_within(
            truck,
            wheel,
            axes="xz",
            inner_elem="axle",
            outer_elem="bearing_core",
            margin=0.001,
            name=f"{wheel_name}_axle_centered_in_bearing",
        )
        ctx.expect_overlap(
            truck,
            wheel,
            axes="y",
            elem_a="axle",
            elem_b="bearing_core",
            min_overlap=0.025,
            name=f"{wheel_name}_bearing_retained_on_axle",
        )

    front_support_aabb = ctx.part_element_world_aabb(deck, elem="front_support")
    rear_support_aabb = ctx.part_element_world_aabb(deck, elem="rear_support")
    if front_support_aabb is not None and rear_support_aabb is not None:
        front_width = float(front_support_aabb[1][1] - front_support_aabb[0][1])
        rear_width = float(rear_support_aabb[1][1] - rear_support_aabb[0][1])
        ctx.check(
            "rear_support_broader_than_front",
            rear_width > front_width * 1.6,
            details=f"front_width={front_width:.4f}, rear_width={rear_width:.4f}",
        )
    else:
        ctx.fail("support_widths_measurable", "Expected both support-pad AABBs.")

    for joint_name in (
        "front_truck_to_wheel_0",
        "front_truck_to_wheel_1",
        "rear_truck_to_wheel_0",
        "rear_truck_to_wheel_1",
    ):
        joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name}_continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"{joint_name} should be a continuous spin joint.",
        )

    ctx.expect_gap(
        deck,
        front_truck,
        axis="z",
        positive_elem="front_support",
        negative_elem="baseplate",
        max_gap=0.001,
        max_penetration=0.0005,
        name="front_baseplate_seated_under_support",
    )
    ctx.expect_gap(
        deck,
        rear_truck,
        axis="z",
        positive_elem="rear_support",
        negative_elem="baseplate",
        max_gap=0.001,
        max_penetration=0.0005,
        name="rear_baseplate_seated_under_support",
    )

    front_wheel = object_model.get_part("front_wheel_0")
    rest_pos = ctx.part_world_position(front_wheel)
    with ctx.pose({front_kingpin: 0.30, rear_kingpin: -0.25}):
        steered_pos = ctx.part_world_position(front_wheel)
    ctx.check(
        "front_truck_steers_wheels",
        rest_pos is not None
        and steered_pos is not None
        and abs(steered_pos[0] - rest_pos[0]) > 0.020,
        details=f"rest={rest_pos}, steered={steered_pos}",
    )

    return ctx.report()


object_model = build_object_model()
