from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
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
    tube_from_spline_points,
)


def _tube(points, radius: float, name: str):
    return mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=18,
            radial_segments=20,
            cap_ends=True,
        ),
        name,
    )


def _cyl_x(length: float, radius: float) -> tuple[Cylinder, Origin]:
    return Cylinder(radius=radius, length=length), Origin(rpy=(0.0, pi / 2.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cylinder_bottle_hand_truck")

    red = Material("red_powder_coat", rgba=(0.82, 0.06, 0.035, 1.0))
    black = Material("black_rubber", rgba=(0.01, 0.01, 0.009, 1.0))
    dark = Material("dark_hardware", rgba=(0.04, 0.045, 0.045, 1.0))
    steel = Material("brushed_steel", rgba=(0.58, 0.60, 0.59, 1.0))
    plate = Material("worn_steel_plate", rgba=(0.34, 0.36, 0.34, 1.0))
    bottle_green = Material("green_cylinder", rgba=(0.03, 0.34, 0.18, 1.0))
    brass = Material("brass_valve", rgba=(0.80, 0.58, 0.24, 1.0))

    frame = model.part("frame")

    # Welded tubular side rails with rear handle sweeps.
    for idx, x in enumerate((-0.23, 0.23)):
        rail = _tube(
            [
                (x, 0.17, 0.055),
                (x, 0.085, 0.19),
                (x, 0.050, 0.48),
                (x, 0.045, 0.82),
                (x, 0.070, 1.06),
                (x, -0.055, 1.25),
                (x, -0.170, 1.36),
            ],
            0.016,
            f"side_rail_{idx}",
        )
        frame.visual(rail, material=red, name=f"side_rail_{idx}")

    cyl, x_axis = _cyl_x(0.52, 0.014)
    for name, y, z, radius in (
        ("lower_crossbar", 0.018, 0.255, 0.014),
        ("mid_crossbar", 0.012, 0.610, 0.012),
        ("shoulder_crossbar", 0.066, 1.055, 0.014),
        ("rear_handle", -0.170, 1.360, 0.017),
    ):
        geometry, axis_origin = _cyl_x(0.52, radius)
        frame.visual(
            geometry,
            origin=Origin(xyz=(0.0, y, z), rpy=axis_origin.rpy),
            material=red,
            name=name,
        )
    for idx, x in enumerate((-0.23, 0.23)):
        lower_mount = Cylinder(radius=0.010, length=0.060)
        frame.visual(
            lower_mount,
            origin=Origin(xyz=(x, 0.048, 0.255), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=red,
            name=f"lower_crossbar_mount_{idx}",
        )
        mid_mount = Cylinder(radius=0.009, length=0.040)
        frame.visual(
            mid_mount,
            origin=Origin(xyz=(x, 0.032, 0.610), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=red,
            name=f"mid_crossbar_mount_{idx}",
        )

    # Curved cradle straps sit just behind the cylinder and weld into the rails.
    for idx, z in enumerate((0.42, 0.72, 0.96)):
        cradle = _tube(
            [
                (-0.235, 0.060, z),
                (-0.145, 0.036, z - 0.010),
                (0.000, 0.026, z - 0.015),
                (0.145, 0.036, z - 0.010),
                (0.235, 0.060, z),
            ],
            0.011,
            f"curved_cradle_{idx}",
        )
        frame.visual(cradle, material=red, name=f"curved_cradle_{idx}")

    # Lower nose plate and retaining lip support the bottle base.
    frame.visual(
        Box((0.48, 0.32, 0.035)),
        origin=Origin(xyz=(0.0, 0.205, 0.0375)),
        material=plate,
        name="nose_plate",
    )
    frame.visual(
        Box((0.46, 0.020, 0.055)),
        origin=Origin(xyz=(0.0, 0.365, 0.075)),
        material=plate,
        name="front_lip",
    )

    # Rear axle and small shoulder hinge pins welded to the frame.
    axle, axle_origin = _cyl_x(0.72, 0.017)
    frame.visual(
        axle,
        origin=Origin(xyz=(0.0, -0.100, 0.180), rpy=axle_origin.rpy),
        material=dark,
        name="wheel_axle",
    )
    for idx, x in enumerate((-0.23, 0.23)):
        frame.visual(
            Cylinder(radius=0.013, length=0.175),
            origin=Origin(xyz=(x, -0.015, 0.180), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=dark,
            name=f"axle_mount_{idx}",
        )
    for idx, x in enumerate((-0.300, 0.300)):
        pin, pin_origin = _cyl_x(0.080, 0.018)
        frame.visual(
            pin,
            origin=Origin(xyz=(x, 0.070, 1.055), rpy=pin_origin.rpy),
            material=steel,
            name=f"pivot_pin_{idx}",
        )

    # Fixed load cylinder, included so the cradle, nose plate, and hoop read at scale.
    cylinder = model.part("cylinder")
    cylinder.visual(
        Cylinder(radius=0.155, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=dark,
        name="foot_ring",
    )
    body_profile = [
        (0.0, 0.034),
        (0.116, 0.034),
        (0.145, 0.065),
        (0.145, 0.670),
        (0.136, 0.730),
        (0.090, 0.795),
        (0.050, 0.835),
        (0.039, 0.890),
        (0.039, 0.930),
        (0.0, 0.930),
    ]
    tank_mesh = mesh_from_geometry(
        LatheGeometry(body_profile, segments=64, closed=True), "cylinder_body"
    )
    cylinder.visual(tank_mesh, material=bottle_green, name="cylinder_body")
    cylinder.visual(
        Cylinder(radius=0.030, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.965)),
        material=brass,
        name="valve_neck",
    )
    cylinder.visual(
        Cylinder(radius=0.055, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 1.020), rpy=(0.0, pi / 2.0, 0.0)),
        material=brass,
        name="valve_knob",
    )
    model.articulation(
        "frame_to_cylinder",
        ArticulationType.FIXED,
        parent=frame,
        child=cylinder,
        origin=Origin(xyz=(0.0, 0.190, 0.055)),
    )

    # Retaining hoop: child frame is the shoulder hinge line.  The sleeves remain
    # clipped over the fixed side pivot pins while the U tube swings over the load.
    hoop = model.part("retaining_hoop")
    hoop_tube = _tube(
        [
            (-0.300, 0.032, -0.024),
            (-0.275, 0.120, -0.120),
            (-0.250, 0.245, -0.315),
            (-0.165, 0.335, -0.430),
            (0.000, 0.365, -0.470),
            (0.165, 0.335, -0.430),
            (0.250, 0.245, -0.315),
            (0.275, 0.120, -0.120),
            (0.300, 0.032, -0.024),
        ],
        0.014,
        "front_hoop_tube",
    )
    hoop.visual(hoop_tube, material=red, name="front_hoop_tube")
    for idx, x in enumerate((-0.300, 0.300)):
        sleeve, sleeve_origin = _cyl_x(0.055, 0.026)
        hoop.visual(
            sleeve,
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=sleeve_origin.rpy),
            material=steel,
            name=f"pivot_sleeve_{idx}",
        )
        hoop.visual(
            Sphere(radius=0.018),
            origin=Origin(xyz=(x, 0.034, -0.020)),
            material=red,
            name=f"sleeve_weld_{idx}",
        )
    model.articulation(
        "frame_to_hoop",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=hoop,
        origin=Origin(xyz=(0.0, 0.070, 1.055)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.8, lower=0.0, upper=1.35),
    )

    # Large rear wheels with pneumatic tires and visible metal hubs.
    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.118,
            0.070,
            rim=WheelRim(
                inner_radius=0.078,
                flange_height=0.009,
                flange_thickness=0.004,
                bead_seat_depth=0.004,
            ),
            hub=WheelHub(
                radius=0.038,
                width=0.054,
                cap_style="domed",
                bolt_pattern=BoltPattern(
                    count=5,
                    circle_diameter=0.052,
                    hole_diameter=0.006,
                ),
            ),
            face=WheelFace(dish_depth=0.010, front_inset=0.003, rear_inset=0.003),
            spokes=WheelSpokes(style="split_y", count=5, thickness=0.005, window_radius=0.014),
            bore=WheelBore(style="round", diameter=0.034),
        ),
        "wheel_hub",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.180,
            0.078,
            inner_radius=0.118,
            carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.06),
            tread=TireTread(style="block", depth=0.010, count=22, land_ratio=0.56),
            grooves=(TireGroove(center_offset=0.0, width=0.010, depth=0.004),),
            sidewall=TireSidewall(style="rounded", bulge=0.06),
            shoulder=TireShoulder(width=0.010, radius=0.004),
        ),
        "wheel_tire",
    )
    for idx, x in enumerate((-0.320, 0.320)):
        wheel = model.part(f"wheel_{idx}")
        wheel.visual(tire_mesh, material=black, name="tire")
        wheel.visual(wheel_mesh, material=steel, name="hub")
        model.articulation(
            f"frame_to_wheel_{idx}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=wheel,
            origin=Origin(xyz=(x, -0.100, 0.180)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=25.0, velocity=18.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    cylinder = object_model.get_part("cylinder")
    hoop = object_model.get_part("retaining_hoop")
    hoop_joint = object_model.get_articulation("frame_to_hoop")

    # The visible hinge sleeves intentionally capture the side pins; this local
    # interpenetration represents a retained pin-in-sleeve fit rather than a free
    # floating hoop.
    for idx in (0, 1):
        ctx.allow_overlap(
            frame,
            hoop,
            elem_a=f"pivot_pin_{idx}",
            elem_b=f"pivot_sleeve_{idx}",
            reason="The hoop sleeve is intentionally clipped over the side pivot pin.",
        )
        ctx.expect_overlap(
            frame,
            hoop,
            axes="xyz",
            elem_a=f"pivot_pin_{idx}",
            elem_b=f"pivot_sleeve_{idx}",
            min_overlap=0.010,
            name=f"pivot {idx} sleeve captures pin",
        )

    ctx.expect_gap(
        cylinder,
        frame,
        axis="z",
        positive_elem="foot_ring",
        negative_elem="nose_plate",
        max_gap=0.002,
        max_penetration=0.0005,
        name="cylinder base rests on nose plate",
    )
    ctx.expect_overlap(
        cylinder,
        frame,
        axes="xy",
        elem_a="foot_ring",
        elem_b="nose_plate",
        min_overlap=0.12,
        name="nose plate supports cylinder footprint",
    )

    closed_aabb = ctx.part_element_world_aabb(hoop, elem="front_hoop_tube")
    with ctx.pose({hoop_joint: 1.20}):
        raised_aabb = ctx.part_element_world_aabb(hoop, elem="front_hoop_tube")
    ctx.check(
        "retaining hoop swings upward over load",
        closed_aabb is not None
        and raised_aabb is not None
        and float(raised_aabb[1][2]) > float(closed_aabb[1][2]) + 0.15,
        details=f"closed={closed_aabb}, raised={raised_aabb}",
    )

    for idx in (0, 1):
        wheel = object_model.get_part(f"wheel_{idx}")
        ctx.allow_overlap(
            frame,
            wheel,
            elem_a="wheel_axle",
            elem_b="hub",
            reason="The rear axle is intentionally represented as passing through the wheel hub bore.",
        )
        ctx.expect_overlap(
            frame,
            wheel,
            axes="xyz",
            elem_a="wheel_axle",
            elem_b="hub",
            min_overlap=0.025,
            name=f"wheel {idx} hub is captured on axle",
        )
        joint = object_model.get_articulation(f"frame_to_wheel_{idx}")
        ctx.check(
            f"wheel {idx} rotates on rear axle",
            joint.articulation_type == ArticulationType.CONTINUOUS
            and tuple(joint.axis) == (1.0, 0.0, 0.0),
            details=f"type={joint.articulation_type}, axis={joint.axis}",
        )

    return ctx.report()


object_model = build_object_model()
