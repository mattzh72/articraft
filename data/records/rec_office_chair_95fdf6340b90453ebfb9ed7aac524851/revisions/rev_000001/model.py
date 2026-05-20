from __future__ import annotations

import math

import cadquery as cq
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
    TireShoulder,
    TireSidewall,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    """A softly radiused cushion/back pad, authored in meters."""
    return cq.Workplane("XY").box(*size).edges().fillet(radius)


def _tube(outer_radius: float, inner_radius: float, length: float) -> cq.Workplane:
    """An open annular tube extruded upward from local z=0."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="five_star_office_chair")

    black_plastic = model.material("black_plastic", rgba=(0.015, 0.014, 0.013, 1.0))
    charcoal = model.material("charcoal_fabric", rgba=(0.08, 0.085, 0.09, 1.0))
    seam = model.material("dark_seam", rgba=(0.025, 0.025, 0.028, 1.0))
    steel = model.material("brushed_steel", rgba=(0.70, 0.72, 0.70, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.10, 0.105, 0.11, 1.0))
    rubber = model.material("rubber_tread", rgba=(0.008, 0.008, 0.008, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_tube(0.105, 0.036, 0.075), "center_hub"),
        origin=Origin(xyz=(0.0, 0.0, 0.0425)),
        material=black_plastic,
        name="center_hub",
    )
    base.visual(
        mesh_from_cadquery(_tube(0.041, 0.025, 0.270), "gas_sleeve"),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=dark_steel,
        name="gas_sleeve",
    )

    leg_length = 0.440
    leg_center_radius = 0.255
    wheel_radius = 0.045
    wheel_width = 0.038
    wheel_center_radius = 0.555
    wheel_center_z = wheel_radius
    caster_wheel_names = (
        "caster_wheel_0",
        "caster_wheel_1",
        "caster_wheel_2",
        "caster_wheel_3",
        "caster_wheel_4",
    )
    caster_axle_names = (
        "caster_axle_0",
        "caster_axle_1",
        "caster_axle_2",
        "caster_axle_3",
        "caster_axle_4",
    )

    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.031,
            wheel_width,
            rim=WheelRim(
                inner_radius=0.020,
                flange_height=0.003,
                flange_thickness=0.002,
                bead_seat_depth=0.002,
            ),
            hub=WheelHub(
                radius=0.014,
                width=0.030,
                cap_style="domed",
                bolt_pattern=BoltPattern(
                    count=5,
                    circle_diameter=0.018,
                    hole_diameter=0.0025,
                ),
            ),
            face=WheelFace(dish_depth=0.0025, front_inset=0.0015, rear_inset=0.0015),
            spokes=WheelSpokes(style="split_y", count=5, thickness=0.0018, window_radius=0.004),
            bore=WheelBore(style="round", diameter=0.020),
        ),
        "caster_rim",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            wheel_radius,
            wheel_width + 0.004,
            inner_radius=0.027,
            sidewall=TireSidewall(style="rounded", bulge=0.05),
            shoulder=TireShoulder(width=0.004, radius=0.002),
        ),
        "caster_tire",
    )

    for i in range(5):
        theta = 2.0 * math.pi * i / 5.0
        radial = (math.cos(theta), math.sin(theta))
        tangent = (-math.sin(theta), math.cos(theta))
        yaw_radial = theta
        yaw_tangent = theta + math.pi / 2.0

        base.visual(
            Box((leg_length, 0.058, 0.042)),
            origin=Origin(
                xyz=(radial[0] * leg_center_radius, radial[1] * leg_center_radius, 0.078),
                rpy=(0.0, 0.0, yaw_radial),
            ),
            material=black_plastic,
            name=f"star_leg_{i}",
        )
        base.visual(
            Cylinder(radius=0.030, length=0.030),
            origin=Origin(
                xyz=(radial[0] * 0.455, radial[1] * 0.455, 0.078),
            ),
            material=black_plastic,
            name=f"leg_boss_{i}",
        )
        base.visual(
            Box((0.088, 0.090, 0.026)),
            origin=Origin(
                xyz=(radial[0] * 0.500, radial[1] * 0.500, 0.112),
                rpy=(0.0, 0.0, yaw_radial),
            ),
            material=black_plastic,
            name=f"caster_bridge_{i}",
        )
        for side, offset in enumerate((-0.032, 0.032)):
            base.visual(
                Box((0.052, 0.010, 0.076)),
                origin=Origin(
                    xyz=(
                        radial[0] * wheel_center_radius + tangent[0] * offset,
                        radial[1] * wheel_center_radius + tangent[1] * offset,
                        0.062,
                    ),
                    rpy=(0.0, 0.0, yaw_radial),
                ),
                material=dark_steel,
                name=f"caster_fork_{i}_{side}",
            )
        base.visual(
            Cylinder(radius=0.010, length=0.075),
            origin=Origin(
                xyz=(
                    radial[0] * wheel_center_radius,
                    radial[1] * wheel_center_radius,
                    wheel_center_z,
                ),
                rpy=(0.0, math.pi / 2.0, yaw_tangent),
            ),
            material=steel,
            name=caster_axle_names[i],
        )

        wheel = model.part(caster_wheel_names[i])
        wheel.visual(tire_mesh, material=rubber, name="tire")
        wheel.visual(wheel_mesh, material=dark_steel, name="rim")
        model.articulation(
            f"base_to_caster_wheel_{i}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=wheel,
            origin=Origin(
                xyz=(
                    radial[0] * wheel_center_radius,
                    radial[1] * wheel_center_radius,
                    wheel_center_z,
                ),
                rpy=(0.0, 0.0, yaw_tangent),
            ),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=20.0),
        )

    seat = model.part("seat_frame")
    seat.visual(
        Cylinder(radius=0.025, length=0.300),
        origin=Origin(xyz=(0.0, 0.0, -0.080)),
        material=steel,
        name="gas_piston",
    )
    seat.visual(
        Box((0.360, 0.300, 0.032)),
        origin=Origin(xyz=(0.0, -0.015, 0.066)),
        material=dark_steel,
        name="under_plate",
    )
    seat.visual(
        mesh_from_cadquery(_rounded_box((0.520, 0.470, 0.082), 0.026), "seat_cushion"),
        origin=Origin(xyz=(0.0, 0.020, 0.120)),
        material=charcoal,
        name="seat_cushion",
    )
    seat.visual(
        Box((0.500, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, 0.252, 0.151)),
        material=seam,
        name="front_seam",
    )
    seat.visual(
        Box((0.040, 0.160, 0.034)),
        origin=Origin(xyz=(-0.150, -0.185, 0.078)),
        material=dark_steel,
        name="rear_arm_0",
    )
    seat.visual(
        Box((0.040, 0.160, 0.034)),
        origin=Origin(xyz=(0.150, -0.185, 0.078)),
        material=dark_steel,
        name="rear_arm_1",
    )
    seat.visual(
        Box((0.030, 0.058, 0.076)),
        origin=Origin(xyz=(-0.215, -0.255, 0.110)),
        material=dark_steel,
        name="rear_bracket_0",
    )
    seat.visual(
        Box((0.060, 0.044, 0.020)),
        origin=Origin(xyz=(-0.185, -0.242, 0.072)),
        material=dark_steel,
        name="bracket_connector_0",
    )
    seat.visual(
        Box((0.030, 0.058, 0.076)),
        origin=Origin(xyz=(0.215, -0.255, 0.110)),
        material=dark_steel,
        name="rear_bracket_1",
    )
    seat.visual(
        Box((0.060, 0.044, 0.020)),
        origin=Origin(xyz=(0.185, -0.242, 0.072)),
        material=dark_steel,
        name="bracket_connector_1",
    )

    model.articulation(
        "gas_column_to_seat",
        ArticulationType.PRISMATIC,
        parent=base,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.340)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=600.0, velocity=0.18, lower=0.0, upper=0.120),
    )

    back = model.part("backrest")
    back.visual(
        Cylinder(radius=0.016, length=0.400),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="hinge_barrel",
    )
    for side, x in enumerate((-0.155, 0.155)):
        back.visual(
            Box((0.034, 0.034, 0.355)),
            origin=Origin(xyz=(x, -0.040, 0.175), rpy=(0.18, 0.0, 0.0)),
            material=dark_steel,
            name=f"back_post_{side}",
        )
    back.visual(
        mesh_from_cadquery(_rounded_box((0.455, 0.062, 0.520), 0.024), "back_cushion"),
        origin=Origin(xyz=(0.0, -0.120, 0.395), rpy=(0.18, 0.0, 0.0)),
        material=charcoal,
        name="back_cushion",
    )
    model.articulation(
        "seat_to_backrest",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=back,
        origin=Origin(xyz=(0.0, -0.255, 0.110)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2, lower=0.0, upper=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    seat = object_model.get_part("seat_frame")
    back = object_model.get_part("backrest")
    gas = object_model.get_articulation("gas_column_to_seat")
    recline = object_model.get_articulation("seat_to_backrest")

    ctx.check(
        "five caster wheels are articulated",
        len([p for p in object_model.parts if p.name.startswith("caster_wheel_")]) == 5,
        details="The chair should have one rotating wheel at each point of the five-star base.",
    )
    caster_wheel_names = (
        "caster_wheel_0",
        "caster_wheel_1",
        "caster_wheel_2",
        "caster_wheel_3",
        "caster_wheel_4",
    )
    caster_axle_names = (
        "caster_axle_0",
        "caster_axle_1",
        "caster_axle_2",
        "caster_axle_3",
        "caster_axle_4",
    )
    for index, (wheel_name, axle_name) in enumerate(zip(caster_wheel_names, caster_axle_names)):
        wheel = object_model.get_part(wheel_name)
        ctx.allow_overlap(
            base,
            wheel,
            elem_a=axle_name,
            elem_b="rim",
            reason="The caster wheel rim is intentionally captured on its axle for continuous rotation.",
        )
        ctx.expect_overlap(
            base,
            wheel,
            axes="xyz",
            elem_a=axle_name,
            elem_b="rim",
            min_overlap=0.010,
            name=f"caster axle {index} passes through its rim",
        )
    ctx.allow_overlap(
        base,
        seat,
        elem_a="gas_sleeve",
        elem_b="gas_piston",
        reason="The gas piston is intentionally retained inside the column sleeve for the prismatic height slide.",
    )
    ctx.expect_within(
        seat,
        base,
        axes="xy",
        inner_elem="gas_piston",
        outer_elem="gas_sleeve",
        margin=0.001,
        name="gas piston is centered in the sleeve",
    )
    ctx.expect_overlap(
        seat,
        base,
        axes="z",
        elem_a="gas_piston",
        elem_b="gas_sleeve",
        min_overlap=0.130,
        name="seat piston remains inserted at low height",
    )
    ctx.expect_contact(
        seat,
        back,
        elem_a="rear_bracket_0",
        elem_b="hinge_barrel",
        contact_tol=0.002,
        name="one rear bracket touches the back hinge barrel",
    )
    ctx.expect_contact(
        seat,
        back,
        elem_a="rear_bracket_1",
        elem_b="hinge_barrel",
        contact_tol=0.002,
        name="opposite rear bracket touches the back hinge barrel",
    )

    low_pos = ctx.part_world_position(seat)
    with ctx.pose({gas: 0.120}):
        ctx.expect_within(
            seat,
            base,
            axes="xy",
            inner_elem="gas_piston",
            outer_elem="gas_sleeve",
            margin=0.001,
            name="raised seat remains coaxial with gas column",
        )
        ctx.expect_overlap(
            seat,
            base,
            axes="z",
            elem_a="gas_piston",
            elem_b="gas_sleeve",
            min_overlap=0.070,
            name="raised seat retains gas-column insertion",
        )
        high_pos = ctx.part_world_position(seat)

    ctx.check(
        "height adjustment moves the seat upward",
        low_pos is not None and high_pos is not None and high_pos[2] > low_pos[2] + 0.10,
        details=f"low={low_pos}, high={high_pos}",
    )

    rest_back_aabb = ctx.part_element_world_aabb(back, elem="back_cushion")
    with ctx.pose({recline: 0.45}):
        reclined_back_aabb = ctx.part_element_world_aabb(back, elem="back_cushion")

    def _aabb_center_y(aabb):
        if aabb is None:
            return None
        return 0.5 * (aabb[0][1] + aabb[1][1])

    rest_y = _aabb_center_y(rest_back_aabb)
    reclined_y = _aabb_center_y(reclined_back_aabb)
    ctx.check(
        "backrest reclines rearward",
        rest_y is not None and reclined_y is not None and reclined_y < rest_y - 0.10,
        details=f"rest_y={rest_y}, reclined_y={reclined_y}",
    )

    return ctx.report()


object_model = build_object_model()
