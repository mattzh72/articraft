from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireSidewall,
    TireShoulder,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _capsule_prism(length: float, width: float, thickness: float) -> cq.Workplane:
    """Rounded skateboard-deck planform extruded upward from z=0."""
    radius = width / 2.0
    shape = (
        cq.Workplane("XY")
        .moveTo(-length / 2.0 + radius, -width / 2.0)
        .lineTo(length / 2.0 - radius, -width / 2.0)
        .threePointArc((length / 2.0, 0.0), (length / 2.0 - radius, width / 2.0))
        .lineTo(-length / 2.0 + radius, width / 2.0)
        .threePointArc((-length / 2.0, 0.0), (-length / 2.0 + radius, -width / 2.0))
        .close()
        .extrude(thickness)
    )
    return shape


def _bearing_core(width: float, outer_radius: float, bore_radius: float) -> cq.Workplane:
    """Annular wheel core centered on local X, matching the wheel spin axis."""
    def circle(radius: float, segments: int = 48) -> list[tuple[float, float]]:
        return [
            (radius * math.cos(2.0 * math.pi * i / segments), radius * math.sin(2.0 * math.pi * i / segments))
            for i in range(segments)
        ]

    core = ExtrudeWithHolesGeometry(circle(outer_radius), [circle(bore_radius)], width, center=True)
    return core.rotate_y(math.pi / 2.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_wide_skateboard")

    maple = model.material("warm_maple", rgba=(0.72, 0.47, 0.25, 1.0))
    grip = model.material("black_grip", rgba=(0.015, 0.014, 0.013, 1.0))
    brushed = model.material("brushed_aluminum", rgba=(0.62, 0.64, 0.64, 1.0))
    dark_metal = model.material("dark_axle_steel", rgba=(0.08, 0.085, 0.09, 1.0))
    rubber = model.material("red_bushings", rgba=(0.55, 0.04, 0.03, 1.0))
    urethane = model.material("warm_urethane", rgba=(0.98, 0.86, 0.50, 1.0))

    deck_length = 0.82
    deck_width = 0.22
    deck_thickness = 0.012
    deck_bottom_z = 0.078
    deck_top_z = deck_bottom_z + deck_thickness
    grip_thickness = 0.0012
    baseplate_thickness = 0.006
    baseplate_bottom_z = deck_bottom_z - baseplate_thickness
    truck_x = 0.275
    axle_z_local = -0.036
    wheel_radius = 0.034
    wheel_width = 0.040
    wheel_center_y = 0.157

    deck = model.part("deck")
    deck.visual(
        mesh_from_cadquery(_capsule_prism(deck_length, deck_width, deck_thickness), "deck_shell"),
        origin=Origin(xyz=(0.0, 0.0, deck_bottom_z)),
        material=maple,
        name="deck_shell",
    )
    deck.visual(
        mesh_from_cadquery(_capsule_prism(0.69, 0.188, grip_thickness), "grip_tape"),
        origin=Origin(xyz=(0.0, 0.0, deck_top_z)),
        material=grip,
        name="grip_tape",
    )

    for x in (-truck_x, truck_x):
        deck.visual(
            Box((0.105, 0.070, baseplate_thickness)),
            origin=Origin(xyz=(x, 0.0, deck_bottom_z - baseplate_thickness / 2.0)),
            material=brushed,
            name=f"{'rear' if x < 0 else 'front'}_baseplate",
        )
        for dx in (-0.030, 0.030):
            for dy in (-0.022, 0.022):
                deck.visual(
                    Cylinder(radius=0.0042, length=0.0022),
                    origin=Origin(xyz=(x + dx, dy, deck_top_z + grip_thickness + 0.0011)),
                    material=dark_metal,
                    name=f"{'rear' if x < 0 else 'front'}_bolt_{dx}_{dy}",
                )

    wheel_core = _bearing_core(wheel_width, outer_radius=0.020, bore_radius=0.0048)
    wheel_outer = TireGeometry(
        wheel_radius,
        0.038,
        inner_radius=0.019,
        carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.03),
        sidewall=TireSidewall(style="rounded", bulge=0.035),
        shoulder=TireShoulder(width=0.0035, radius=0.0025),
    )

    def make_truck(name: str, x: float) -> object:
        truck = model.part(name)
        truck.visual(
            Cylinder(radius=0.005, length=0.036),
            origin=Origin(xyz=(0.0, 0.0, -0.018)),
            material=dark_metal,
            name="kingpin",
        )
        truck.visual(
            Cylinder(radius=0.017, length=0.012),
            origin=Origin(xyz=(0.0, 0.0, -0.006)),
            material=rubber,
            name="upper_bushing",
        )
        truck.visual(
            Cylinder(radius=0.015, length=0.012),
            origin=Origin(xyz=(0.0, 0.0, -0.020)),
            material=rubber,
            name="lower_bushing",
        )
        truck.visual(
            Box((0.050, 0.040, 0.026)),
            origin=Origin(xyz=(0.0, 0.0, -0.025)),
            material=brushed,
            name="neck",
        )
        truck.visual(
            Box((0.082, 0.122, 0.020)),
            origin=Origin(xyz=(0.0, 0.0, -0.037)),
            material=brushed,
            name="hanger",
        )
        truck.visual(
            Cylinder(radius=0.00485, length=0.361),
            origin=Origin(xyz=(0.0, 0.0, axle_z_local), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name="axle",
        )
        for side, y_sign in (("wheel_0", -1.0), ("wheel_1", 1.0)):
            y = y_sign * wheel_center_y
            inner_center = y_sign * (wheel_center_y - wheel_width / 2.0 - 0.0038)
            outer_center = y_sign * (wheel_center_y + wheel_width / 2.0 + 0.005)
            truck.visual(
                Cylinder(radius=0.012, length=0.006),
                origin=Origin(xyz=(0.0, inner_center, axle_z_local), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=dark_metal,
                name=f"{side}_washer",
            )
            truck.visual(
                Cylinder(radius=0.0085, length=0.008),
                origin=Origin(xyz=(0.0, outer_center, axle_z_local), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=dark_metal,
                name=f"{side}_nut",
            )

        model.articulation(
            f"deck_to_{name}",
            ArticulationType.REVOLUTE,
            parent=deck,
            child=truck,
            origin=Origin(xyz=(x, 0.0, baseplate_bottom_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-0.45, upper=0.45),
        )
        return truck

    front_truck = make_truck("front_truck", truck_x)
    rear_truck = make_truck("rear_truck", -truck_x)

    for truck_name, truck in (("front", front_truck), ("rear", rear_truck)):
        for index, y_sign in enumerate((-1.0, 1.0)):
            wheel = model.part(f"{truck_name}_wheel_{index}")
            wheel.visual(
                mesh_from_geometry(wheel_outer, f"{truck_name}_wheel_{index}_urethane"),
                material=urethane,
                name="urethane_tire",
            )
            wheel.visual(
                mesh_from_geometry(wheel_core, f"{truck_name}_wheel_{index}_bearing_core"),
                material=brushed,
                name="bearing_core",
            )
            model.articulation(
                f"{truck_name}_wheel_{index}_spin",
                ArticulationType.CONTINUOUS,
                parent=truck,
                child=wheel,
                origin=Origin(
                    xyz=(0.0, y_sign * wheel_center_y, axle_z_local),
                    rpy=(0.0, 0.0, math.pi / 2.0),
                ),
                axis=(1.0, 0.0, 0.0),
                motion_limits=MotionLimits(effort=2.0, velocity=40.0),
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    front_truck = object_model.get_part("front_truck")
    rear_truck = object_model.get_part("rear_truck")
    front_steer = object_model.get_articulation("deck_to_front_truck")
    rear_steer = object_model.get_articulation("deck_to_rear_truck")

    for truck in (front_truck, rear_truck):
        ctx.expect_gap(
            deck,
            truck,
            axis="z",
            max_gap=0.0015,
            max_penetration=0.0,
            name=f"{truck.name} kingpin stack seats under baseplate",
        )

    for truck_name, truck in (("front", front_truck), ("rear", rear_truck)):
        for index in (0, 1):
            wheel = object_model.get_part(f"{truck_name}_wheel_{index}")
            ctx.allow_overlap(
                truck,
                wheel,
                elem_a="axle",
                elem_b="bearing_core",
                reason="The steel axle is intentionally captured through the wheel bearing core proxy.",
            )
            ctx.expect_overlap(
                truck,
                wheel,
                axes="y",
                elem_a="axle",
                elem_b="bearing_core",
                min_overlap=0.035,
                name=f"{wheel.name} axle passes through bearing width",
            )
            ctx.expect_overlap(
                truck,
                wheel,
                axes="xz",
                elem_a="axle",
                elem_b="bearing_core",
                min_overlap=0.008,
                name=f"{wheel.name} bearing is coaxial with axle",
            )

    ctx.check(
        "two steerable truck kingpins",
        front_steer.articulation_type == ArticulationType.REVOLUTE
        and rear_steer.articulation_type == ArticulationType.REVOLUTE
        and front_steer.motion_limits is not None
        and front_steer.motion_limits.lower <= -0.4
        and front_steer.motion_limits.upper >= 0.4
        and rear_steer.motion_limits is not None
        and rear_steer.motion_limits.lower <= -0.4
        and rear_steer.motion_limits.upper >= 0.4,
        details="front and rear truck hangers should steer about limited revolute kingpin axes",
    )

    wheel_joints = [
        object_model.get_articulation(name)
        for name in (
            "front_wheel_0_spin",
            "front_wheel_1_spin",
            "rear_wheel_0_spin",
            "rear_wheel_1_spin",
        )
    ]
    ctx.check(
        "four continuous wheel spin joints",
        all(j.articulation_type == ArticulationType.CONTINUOUS and j.axis == (1.0, 0.0, 0.0) for j in wheel_joints),
        details="each wheel should spin freely around its axle-aligned local X axis",
    )

    deck_aabb = ctx.part_world_aabb(deck)
    wheel_aabbs = [
        ctx.part_world_aabb(object_model.get_part(name))
        for name in ("front_wheel_0", "front_wheel_1", "rear_wheel_0", "rear_wheel_1")
    ]
    if deck_aabb is not None and all(box is not None for box in wheel_aabbs):
        deck_min, deck_max = deck_aabb
        deck_width = deck_max[1] - deck_min[1]
        track_width = max(box[1][1] for box in wheel_aabbs if box is not None) - min(
            box[0][1] for box in wheel_aabbs if box is not None
        )
        lowest_wheel = min(box[0][2] for box in wheel_aabbs if box is not None)
        ctx.check(
            "wide low stance",
            track_width > deck_width + 0.09 and deck_max[2] < 0.105 and lowest_wheel < 0.006,
            details=f"track_width={track_width:.3f}, deck_width={deck_width:.3f}, deck_top={deck_max[2]:.3f}",
        )
    else:
        ctx.fail("wide low stance", "could not measure deck and wheel bounds")

    rest_pos = ctx.part_world_position(object_model.get_part("front_wheel_1"))
    with ctx.pose({front_steer: 0.40}):
        turned_pos = ctx.part_world_position(object_model.get_part("front_wheel_1"))
    ctx.check(
        "front truck steers wheels around kingpin",
        rest_pos is not None
        and turned_pos is not None
        and abs(turned_pos[0] - rest_pos[0]) > 0.020
        and abs(turned_pos[1] - rest_pos[1]) > 0.004,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


object_model = build_object_model()
