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
    TireCarcass,
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
    section_loft,
)


DECK_LENGTH = 0.80
DECK_WIDTH = 0.210
DECK_THICKNESS = 0.012
TRUCK_X = 0.255
WHEEL_Y = 0.185
AXLE_Z = -0.071
WHEEL_RADIUS = 0.032
WHEEL_WIDTH = 0.034


def _deck_geometry():
    """A thin concave deck with rounded, upturned nose and tail."""

    half_length = DECK_LENGTH / 2.0
    half_width = DECK_WIDTH / 2.0
    rounded_center = half_length - half_width
    kick_start = 0.285
    kick_height = 0.036
    concave_depth = 0.0045
    y_samples = [-1.0, -0.72, -0.42, -0.18, 0.0, 0.18, 0.42, 0.72, 1.0]
    x_samples = [-0.392, -0.365, -0.325, -0.285, -0.190, 0.0, 0.190, 0.285, 0.325, 0.365, 0.392]

    sections = []
    for x in x_samples:
        ax = abs(x)
        if ax <= rounded_center:
            section_half_width = half_width
        else:
            nose_dx = min(half_width, ax - rounded_center)
            section_half_width = max(0.028, math.sqrt(max(0.0, half_width * half_width - nose_dx * nose_dx)))

        kick_t = 0.0
        if ax > kick_start:
            kick_t = (ax - kick_start) / (half_length - kick_start)
        kick = kick_height * kick_t * kick_t * (3.0 - 2.0 * kick_t)

        top = []
        bottom = []
        for yn in y_samples:
            y = yn * section_half_width
            rail_lift = concave_depth * abs(yn) ** 1.8
            top.append((x, y, kick + DECK_THICKNESS / 2.0 + rail_lift))
            bottom.append((x, y, kick - DECK_THICKNESS / 2.0 + rail_lift))

        sections.append(top + list(reversed(bottom)))

    return section_loft(sections, cap=True, solid=True, repair="mesh")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wide_stance_skateboard")

    grip = model.material("black_grip", rgba=(0.018, 0.018, 0.016, 1.0))
    aluminum = model.material("cast_aluminum", rgba=(0.70, 0.72, 0.72, 1.0))
    axle_steel = model.material("polished_steel", rgba=(0.86, 0.86, 0.82, 1.0))
    bolt_black = model.material("blackened_bolts", rgba=(0.03, 0.03, 0.035, 1.0))
    bushing = model.material("orange_bushings", rgba=(0.95, 0.32, 0.07, 1.0))
    tire_mat = model.material("translucent_urethane", rgba=(0.12, 0.55, 0.86, 1.0))
    core_mat = model.material("white_wheel_core", rgba=(0.94, 0.92, 0.82, 1.0))

    deck = model.part("deck")
    deck.visual(
        mesh_from_geometry(_deck_geometry(), "concave_kicktail_deck"),
        material=grip,
        name="deck_shell",
    )

    for truck_name, x in (("front", TRUCK_X), ("rear", -TRUCK_X)):
        deck.visual(
            Box((0.092, 0.070, 0.008)),
            origin=Origin(xyz=(x, 0.0, -0.010)),
            material=aluminum,
            name=f"{truck_name}_baseplate",
        )
        for bx in (-0.026, 0.026):
            for by in (-0.022, 0.022):
                deck.visual(
                    Cylinder(radius=0.0045, length=0.003),
                    origin=Origin(xyz=(x + bx, by, 0.0072)),
                    material=bolt_black,
                    name=f"{truck_name}_bolt_{'p' if bx > 0 else 'n'}_{'p' if by > 0 else 'n'}",
                )

    wheel_core = mesh_from_geometry(
        WheelGeometry(
            0.021,
            WHEEL_WIDTH,
            rim=WheelRim(inner_radius=0.013, flange_height=0.0025, flange_thickness=0.002),
            hub=WheelHub(
                radius=0.010,
                width=0.024,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=5, circle_diameter=0.015, hole_diameter=0.002),
            ),
            face=WheelFace(dish_depth=0.0025, front_inset=0.0015, rear_inset=0.0015),
            spokes=WheelSpokes(style="straight", count=5, thickness=0.0018, window_radius=0.004),
            bore=WheelBore(style="round", diameter=0.010),
        ),
        "skateboard_wheel_core",
    )
    tire = mesh_from_geometry(
        TireGeometry(
            WHEEL_RADIUS,
            WHEEL_WIDTH,
            inner_radius=0.019,
            carcass=TireCarcass(belt_width_ratio=0.74, sidewall_bulge=0.05),
            tread=TireTread(style="ribbed", depth=0.0012, count=24, land_ratio=0.68),
            grooves=(TireGroove(center_offset=0.0, width=0.0025, depth=0.0010),),
            sidewall=TireSidewall(style="rounded", bulge=0.045),
            shoulder=TireShoulder(width=0.003, radius=0.002),
        ),
        "skateboard_urethane_tire",
    )

    for truck_name, x in (("front", TRUCK_X), ("rear", -TRUCK_X)):
        truck = model.part(f"{truck_name}_truck")
        truck.visual(
            Cylinder(radius=0.015, length=0.004),
            origin=Origin(xyz=(0.0, 0.0, -0.002)),
            material=axle_steel,
            name="top_washer",
        )
        truck.visual(
            Cylinder(radius=0.012, length=0.030),
            origin=Origin(xyz=(0.0, 0.0, -0.019)),
            material=bushing,
            name="kingpin_bushing",
        )
        truck.visual(
            Box((0.058, 0.168, 0.030)),
            origin=Origin(xyz=(0.0, 0.0, -0.052)),
            material=aluminum,
            name="hanger_body",
        )
        truck.visual(
            Cylinder(radius=0.0042, length=0.392),
            origin=Origin(xyz=(0.0, 0.0, AXLE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=axle_steel,
            name="axle",
        )
        truck.visual(
            Box((0.034, 0.036, 0.034)),
            origin=Origin(xyz=(0.0, 0.0, -0.037), rpy=(0.0, 0.0, math.pi / 4.0)),
            material=aluminum,
            name="pivot_boss",
        )

        model.articulation(
            f"{truck_name}_truck_steer",
            ArticulationType.REVOLUTE,
            parent=deck,
            child=truck,
            origin=Origin(xyz=(x, 0.0, -0.014)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-0.48, upper=0.48),
        )

        for i, y in enumerate((WHEEL_Y, -WHEEL_Y)):
            wheel = model.part(f"{truck_name}_wheel_{i}")
            wheel.visual(
                tire,
                origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
                material=tire_mat,
                name="urethane_tire",
            )
            wheel.visual(
                wheel_core,
                origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
                material=core_mat,
                name="plastic_core",
            )
            model.articulation(
                f"{truck_name}_wheel_{i}_spin",
                ArticulationType.CONTINUOUS,
                parent=truck,
                child=wheel,
                origin=Origin(xyz=(0.0, y, AXLE_Z)),
                axis=(0.0, 1.0, 0.0),
                motion_limits=MotionLimits(effort=2.0, velocity=70.0),
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck")
    steering_joints = [
        object_model.get_articulation("front_truck_steer"),
        object_model.get_articulation("rear_truck_steer"),
    ]
    spin_joints = [
        object_model.get_articulation("front_wheel_0_spin"),
        object_model.get_articulation("front_wheel_1_spin"),
        object_model.get_articulation("rear_wheel_0_spin"),
        object_model.get_articulation("rear_wheel_1_spin"),
    ]

    ctx.check(
        "truck steering joints are revolute kingpins",
        all(joint.articulation_type == ArticulationType.REVOLUTE for joint in steering_joints),
        details=f"joint types={[joint.articulation_type for joint in steering_joints]}",
    )
    ctx.check(
        "wheel spin joints are continuous",
        all(joint.articulation_type == ArticulationType.CONTINUOUS for joint in spin_joints),
        details=f"joint types={[joint.articulation_type for joint in spin_joints]}",
    )

    for truck_name in ("front", "rear"):
        truck = object_model.get_part(f"{truck_name}_truck")
        ctx.expect_gap(
            deck,
            truck,
            axis="z",
            max_gap=0.001,
            max_penetration=0.001,
            positive_elem=f"{truck_name}_baseplate",
            negative_elem="top_washer",
            name=f"{truck_name} truck is seated under its baseplate",
        )

        for wheel_i in (0, 1):
            wheel = object_model.get_part(f"{truck_name}_wheel_{wheel_i}")
            ctx.allow_overlap(
                truck,
                wheel,
                elem_a="axle",
                elem_b="plastic_core",
                reason="The steel axle is intentionally captured through the wheel hub proxy so the continuous spin joint is visibly supported.",
            )
            ctx.expect_within(
                truck,
                wheel,
                axes="xz",
                inner_elem="axle",
                outer_elem="plastic_core",
                margin=0.002,
                name=f"{truck_name} wheel {wheel_i} axle is centered in the hub",
            )
            ctx.expect_overlap(
                truck,
                wheel,
                axes="y",
                elem_a="axle",
                elem_b="plastic_core",
                min_overlap=0.020,
                name=f"{truck_name} wheel {wheel_i} remains captured on the axle",
            )

        ctx.expect_origin_distance(
            f"{truck_name}_wheel_0",
            f"{truck_name}_wheel_1",
            axes="y",
            min_dist=0.35,
            name=f"{truck_name} truck has a wide wheel stance",
        )

    return ctx.report()


object_model = build_object_model()
