from __future__ import annotations

from math import cos, pi, sin

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
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
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _rounded_box_mesh(size: tuple[float, float, float], radius: float, name: str):
    shape = cq.Workplane("XY").box(*size).edges().fillet(radius)
    return mesh_from_cadquery(shape, name, tolerance=0.001, angular_tolerance=0.08)


def _radial_origin(
    radius: float,
    theta: float,
    z: float,
    *,
    yaw_offset: float = 0.0,
    pitch: float = 0.0,
) -> Origin:
    return Origin(
        xyz=(radius * cos(theta), radius * sin(theta), z),
        rpy=(0.0, pitch, theta + yaw_offset),
    )


def _add_radial_box(
    part,
    size: tuple[float, float, float],
    radius: float,
    theta: float,
    z: float,
    material: Material,
    name: str,
) -> None:
    part.visual(
        Box(size),
        origin=_radial_origin(radius, theta, z),
        material=material,
        name=name,
    )


def _add_horizontal_cylinder(
    part,
    *,
    radius: float,
    length: float,
    center_radius: float,
    theta: float,
    z: float,
    material: Material,
    name: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=_radial_origin(center_radius, theta, z, pitch=pi / 2.0),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="five_star_office_chair")

    black_plastic = model.material("black_plastic", rgba=(0.025, 0.026, 0.028, 1.0))
    charcoal_fabric = model.material("charcoal_fabric", rgba=(0.075, 0.080, 0.086, 1.0))
    dark_seam = model.material("dark_seam", rgba=(0.012, 0.012, 0.014, 1.0))
    chrome = model.material("brushed_chrome", rgba=(0.72, 0.74, 0.76, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.12, 0.125, 0.13, 1.0))
    rubber = model.material("soft_rubber", rgba=(0.018, 0.018, 0.017, 1.0))
    grey_rim = model.material("caster_grey_rim", rgba=(0.45, 0.47, 0.49, 1.0))

    frame = model.part("chair_frame")

    seat_mesh = _rounded_box_mesh((0.54, 0.56, 0.075), 0.030, "seat_cushion")
    back_mesh = _rounded_box_mesh((0.075, 0.50, 0.62), 0.025, "back_cushion")

    frame.visual(
        seat_mesh,
        origin=Origin(xyz=(0.035, 0.0, 0.497)),
        material=charcoal_fabric,
        name="seat_cushion",
    )
    frame.visual(
        Box((0.47, 0.43, 0.040)),
        origin=Origin(xyz=(0.005, 0.0, 0.437)),
        material=dark_metal,
        name="underseat_plate",
    )
    frame.visual(
        Box((0.36, 0.36, 0.022)),
        origin=Origin(xyz=(0.005, 0.0, 0.470)),
        material=black_plastic,
        name="seat_pan",
    )

    # Subtle raised seams on the upholstered seat make the cushion read as soft.
    frame.visual(
        Box((0.006, 0.500, 0.006)),
        origin=Origin(xyz=(-0.085, 0.0, 0.532)),
        material=dark_seam,
        name="seat_rear_seam",
    )
    frame.visual(
        Box((0.006, 0.440, 0.005)),
        origin=Origin(xyz=(0.145, 0.0, 0.532)),
        material=dark_seam,
        name="seat_front_seam",
    )

    # Central gas-lift column, shroud, hub, and five-star base.
    frame.visual(
        Cylinder(radius=0.034, length=0.350),
        origin=Origin(xyz=(0.0, 0.0, 0.285)),
        material=chrome,
        name="gas_column",
    )
    frame.visual(
        Cylinder(radius=0.055, length=0.205),
        origin=Origin(xyz=(0.0, 0.0, 0.220)),
        material=black_plastic,
        name="gas_shroud",
    )
    frame.visual(
        Cylinder(radius=0.088, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.113)),
        material=black_plastic,
        name="base_hub",
    )
    frame.visual(
        Cylinder(radius=0.060, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.157)),
        material=chrome,
        name="hub_cap",
    )

    wheel = WheelGeometry(
        0.030,
        0.034,
        rim=WheelRim(inner_radius=0.018, flange_height=0.003, flange_thickness=0.002),
        hub=WheelHub(radius=0.013, width=0.030, cap_style="domed"),
        face=WheelFace(dish_depth=0.002, front_inset=0.001, rear_inset=0.001),
        spokes=WheelSpokes(style="straight", count=5, thickness=0.0018, window_radius=0.004),
        bore=WheelBore(style="round", diameter=0.016),
    )
    tire = TireGeometry(
        0.045,
        0.042,
        inner_radius=0.030,
        tread=TireTread(style="ribbed", depth=0.0025, count=18, land_ratio=0.58),
        sidewall=TireSidewall(style="rounded", bulge=0.035),
        shoulder=TireShoulder(width=0.004, radius=0.002),
    )
    wheel_mesh = mesh_from_geometry(wheel, "caster_wheel_rim")
    tire_mesh = mesh_from_geometry(tire, "caster_tire")

    caster_radius = 0.455
    caster_z = 0.045
    for i in range(5):
        theta = pi / 2.0 + 2.0 * pi * i / 5.0
        tangent_yaw = theta + pi / 2.0
        _add_radial_box(
            frame,
            (0.405, 0.072, 0.040),
            0.255,
            theta,
            0.116,
            black_plastic,
            f"base_spoke_{i}",
        )
        _add_horizontal_cylinder(
            frame,
            radius=0.010,
            length=0.330,
            center_radius=0.270,
            theta=theta,
            z=0.142,
            material=chrome,
            name=f"spoke_chrome_strip_{i}",
        )
        frame.visual(
            Cylinder(radius=0.010, length=0.050),
            origin=Origin(
                xyz=(0.390 * cos(theta), 0.390 * sin(theta), 0.098),
                rpy=(0.0, 0.0, 0.0),
            ),
            material=dark_metal,
            name=f"caster_stem_{i}",
        )
        def caster_origin(local_x: float, local_y: float, z: float) -> Origin:
            return Origin(
                xyz=(
                    caster_radius * cos(theta)
                    + local_x * cos(tangent_yaw)
                    - local_y * sin(tangent_yaw),
                    caster_radius * sin(theta)
                    + local_x * sin(tangent_yaw)
                    + local_y * cos(tangent_yaw),
                    z,
                ),
                rpy=(0.0, 0.0, tangent_yaw),
            )

        frame.visual(
            Box((0.010, 0.030, 0.085)),
            origin=caster_origin(0.034, 0.0, caster_z + 0.030),
            material=dark_metal,
            name=f"caster_fork_outer_{i}",
        )
        frame.visual(
            Box((0.010, 0.030, 0.085)),
            origin=caster_origin(-0.034, 0.0, caster_z + 0.030),
            material=dark_metal,
            name=f"caster_fork_inner_{i}",
        )
        frame.visual(
            Box((0.086, 0.032, 0.018)),
            origin=caster_origin(0.0, 0.0, caster_z + 0.060),
            material=dark_metal,
            name=f"caster_fork_bridge_{i}",
        )
        frame.visual(
            Cylinder(radius=0.012, length=0.052),
            origin=Origin(
                xyz=(caster_radius * cos(theta), caster_radius * sin(theta), caster_z + 0.083),
                rpy=(0.0, 0.0, 0.0),
            ),
            material=dark_metal,
            name=f"caster_fork_stem_{i}",
        )
        frame.visual(
            Cylinder(radius=0.008, length=0.086),
            origin=Origin(
                xyz=(caster_radius * cos(theta), caster_radius * sin(theta), caster_z),
                rpy=(0.0, pi / 2.0, tangent_yaw),
            ),
            material=chrome,
            name=f"caster_axle_{i}",
        )

        caster = model.part(f"caster_{i}")
        caster.visual(tire_mesh, material=rubber, name="tire")
        caster.visual(wheel_mesh, material=grey_rim, name="rim")
        model.articulation(
            f"caster_{i}_spin",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=caster,
            origin=Origin(
                xyz=(caster_radius * cos(theta), caster_radius * sin(theta), caster_z),
                rpy=(0.0, 0.0, tangent_yaw),
            ),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.8, velocity=30.0),
        )

    # Fixed armrests bolted to the seat-side rails.
    for side, y in (("left", 0.330), ("right", -0.330)):
        post_y = 0.300 if y > 0 else -0.300
        rail_y = 0.255 if y > 0 else -0.255
        frame.visual(
            Box((0.045, 0.050, 0.200)),
            origin=Origin(xyz=(0.185, post_y, 0.590)),
            material=dark_metal,
            name=f"{side}_front_arm_post",
        )
        frame.visual(
            Box((0.045, 0.050, 0.200)),
            origin=Origin(xyz=(-0.130, post_y, 0.590)),
            material=dark_metal,
            name=f"{side}_rear_arm_post",
        )
        frame.visual(
            Box((0.410, 0.078, 0.045)),
            origin=Origin(xyz=(0.025, y, 0.700)),
            material=black_plastic,
            name=f"{side}_arm_pad",
        )
        frame.visual(
            Box((0.400, 0.080, 0.026)),
            origin=Origin(xyz=(0.020, rail_y, 0.492)),
            material=dark_metal,
            name=f"{side}_seat_side_rail",
        )

    # Rear hinge clevis and transverse pin for the reclining backrest.
    frame.visual(
        Cylinder(radius=0.009, length=0.560),
        origin=Origin(xyz=(-0.285, 0.0, 0.515), rpy=(pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="back_hinge_pin",
    )
    frame.visual(
        Box((0.052, 0.032, 0.072)),
        origin=Origin(xyz=(-0.285, 0.245, 0.515)),
        material=dark_metal,
        name="left_back_clevis",
    )
    frame.visual(
        Box((0.052, 0.032, 0.072)),
        origin=Origin(xyz=(-0.285, -0.245, 0.515)),
        material=dark_metal,
        name="right_back_clevis",
    )
    frame.visual(
        Box((0.090, 0.540, 0.025)),
        origin=Origin(xyz=(-0.262, 0.0, 0.480)),
        material=dark_metal,
        name="rear_seat_crossbar",
    )

    backrest = model.part("backrest")
    backrest.visual(
        Cylinder(radius=0.021, length=0.420),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="back_hinge_barrel",
    )
    backrest.visual(
        Box((0.030, 0.034, 0.050)),
        origin=Origin(xyz=(-0.015, 0.205, 0.040)),
        material=dark_metal,
        name="left_hinge_lug",
    )
    backrest.visual(
        Box((0.030, 0.034, 0.050)),
        origin=Origin(xyz=(-0.015, -0.205, 0.040)),
        material=dark_metal,
        name="right_hinge_lug",
    )
    backrest.visual(
        Cylinder(radius=0.014, length=0.500),
        origin=Origin(xyz=(-0.065, 0.205, 0.306), rpy=(0.0, -0.18, 0.0)),
        material=dark_metal,
        name="left_back_upright",
    )
    backrest.visual(
        Cylinder(radius=0.014, length=0.500),
        origin=Origin(xyz=(-0.065, -0.205, 0.306), rpy=(0.0, -0.18, 0.0)),
        material=dark_metal,
        name="right_back_upright",
    )
    backrest.visual(
        back_mesh,
        origin=Origin(xyz=(-0.095, 0.0, 0.405), rpy=(0.0, -0.18, 0.0)),
        material=charcoal_fabric,
        name="back_cushion",
    )
    backrest.visual(
        Box((0.020, 0.440, 0.006)),
        origin=Origin(xyz=(-0.132, 0.0, 0.405), rpy=(0.0, -0.18, 0.0)),
        material=dark_seam,
        name="back_lumbar_seam",
    )
    backrest.visual(
        Box((0.050, 0.460, 0.035)),
        origin=Origin(xyz=(-0.135, 0.0, 0.700), rpy=(0.0, -0.18, 0.0)),
        material=black_plastic,
        name="back_top_cap",
    )

    model.articulation(
        "back_recline",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=backrest,
        origin=Origin(xyz=(-0.285, 0.0, 0.515)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.5, lower=0.0, upper=0.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("chair_frame")
    backrest = object_model.get_part("backrest")
    recline = object_model.get_articulation("back_recline")

    ctx.allow_overlap(
        frame,
        backrest,
        elem_a="back_hinge_pin",
        elem_b="back_hinge_barrel",
        reason="The fixed transverse pin is intentionally captured inside the reclining backrest barrel.",
    )
    ctx.expect_within(
        frame,
        backrest,
        axes="xz",
        inner_elem="back_hinge_pin",
        outer_elem="back_hinge_barrel",
        margin=0.002,
        name="back hinge pin is coaxial within the barrel",
    )
    ctx.expect_overlap(
        frame,
        backrest,
        axes="y",
        elem_a="back_hinge_pin",
        elem_b="back_hinge_barrel",
        min_overlap=0.38,
        name="back hinge pin spans the barrel",
    )

    caster_joints = [object_model.get_articulation(f"caster_{i}_spin") for i in range(5)]
    for i in range(5):
        caster = object_model.get_part(f"caster_{i}")
        ctx.allow_overlap(
            frame,
            caster,
            elem_a=f"caster_axle_{i}",
            elem_b="rim",
            reason="The caster wheel is intentionally captured by its fixed axle through the rim bore.",
        )
        ctx.expect_overlap(
            frame,
            caster,
            axes="xyz",
            elem_a=f"caster_axle_{i}",
            elem_b="rim",
            min_overlap=0.014,
            name=f"caster {i} axle is retained in the rim",
        )
    ctx.check(
        "five continuous caster spin joints",
        len(caster_joints) == 5
        and all(j.articulation_type == ArticulationType.CONTINUOUS for j in caster_joints),
        details=str([j.articulation_type for j in caster_joints]),
    )
    ctx.check(
        "backrest recline range is about twenty degrees",
        recline.motion_limits is not None
        and recline.motion_limits.lower == 0.0
        and 0.33 <= float(recline.motion_limits.upper) <= 0.37,
        details=str(recline.motion_limits),
    )

    at_rest = ctx.part_element_world_aabb(backrest, elem="back_cushion")
    with ctx.pose({recline: 0.35}):
        reclined = ctx.part_element_world_aabb(backrest, elem="back_cushion")
    ctx.check(
        "positive recline moves the backrest rearward",
        at_rest is not None
        and reclined is not None
        and reclined[0][0] < at_rest[0][0] - 0.035,
        details=f"rest={at_rest}, reclined={reclined}",
    )

    return ctx.report()


object_model = build_object_model()
