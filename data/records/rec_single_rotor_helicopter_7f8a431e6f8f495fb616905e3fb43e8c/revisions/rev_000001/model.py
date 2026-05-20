from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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
    rounded_rect_profile,
    section_loft,
    sweep_profile_along_spline,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _yz_section(
    x: float,
    *,
    width: float,
    height: float,
    radius: float,
    z_center: float,
    corner_segments: int = 8,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z_center + z)
        for z, y in rounded_rect_profile(
            height,
            width,
            radius,
            corner_segments=corner_segments,
        )
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_helicopter")

    body_paint = model.material("body_paint", rgba=(0.34, 0.40, 0.28, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.55, 0.73, 0.82, 0.34))
    dark_metal = model.material("dark_metal", rgba=(0.16, 0.17, 0.18, 1.0))
    skid_metal = model.material("skid_metal", rgba=(0.67, 0.69, 0.72, 1.0))
    rotor_black = model.material("rotor_black", rgba=(0.10, 0.10, 0.11, 1.0))
    hub_gray = model.material("hub_gray", rgba=(0.42, 0.44, 0.46, 1.0))
    accent_red = model.material("accent_red", rgba=(0.75, 0.12, 0.10, 1.0))

    airframe = model.part("airframe")
    airframe.inertial = Inertial.from_geometry(
        Box((8.2, 3.2, 3.0)),
        mass=980.0,
        origin=Origin(xyz=(-2.0, 0.0, 0.85)),
    )

    fuselage_sections = [
        _yz_section(1.55, width=0.16, height=0.20, radius=0.05, z_center=0.80),
        _yz_section(1.18, width=0.92, height=1.00, radius=0.18, z_center=0.94),
        _yz_section(0.55, width=1.42, height=1.42, radius=0.24, z_center=0.96),
        _yz_section(0.00, width=1.56, height=1.56, radius=0.24, z_center=1.02),
        _yz_section(-0.82, width=1.30, height=1.28, radius=0.22, z_center=1.00),
        _yz_section(-1.58, width=0.72, height=0.78, radius=0.15, z_center=1.02),
    ]
    airframe.visual(
        _save_mesh("fuselage_shell", section_loft(fuselage_sections)),
        material=body_paint,
        name="fuselage_shell",
    )

    canopy_sections = [
        _yz_section(1.20, width=0.86, height=0.92, radius=0.16, z_center=0.98),
        _yz_section(0.76, width=1.16, height=1.14, radius=0.18, z_center=1.06),
        _yz_section(0.18, width=1.06, height=1.02, radius=0.18, z_center=1.13),
    ]
    airframe.visual(
        _save_mesh("canopy_glazing", section_loft(canopy_sections)),
        material=glass_tint,
        name="canopy_glazing",
    )

    airframe.visual(
        Box((2.50, 0.98, 0.38)),
        origin=Origin(xyz=(-0.18, 0.0, 0.42)),
        material=body_paint,
        name="belly_fairing",
    )
    airframe.visual(
        Box((0.72, 0.84, 0.34)),
        origin=Origin(xyz=(1.04, 0.0, 0.46)),
        material=body_paint,
        name="nose_chin",
    )
    airframe.visual(
        Box((1.18, 0.82, 0.26)),
        origin=Origin(xyz=(-0.36, 0.0, 1.67)),
        material=body_paint,
        name="engine_housing",
    )
    airframe.visual(
        Box((0.58, 0.38, 0.18)),
        origin=Origin(xyz=(0.02, 0.0, 1.91)),
        material=dark_metal,
        name="mast_saddle",
    )
    airframe.visual(
        Cylinder(radius=0.13, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 2.07)),
        material=dark_metal,
        name="mast_pylon",
    )
    airframe.visual(
        Box((0.58, 0.56, 0.06)),
        origin=Origin(xyz=(-0.55, 0.0, 1.83)),
        material=dark_metal,
        name="service_deck",
    )
    airframe.visual(
        Box((0.78, 0.06, 0.72)),
        origin=Origin(xyz=(-0.95, 0.775, 0.96)),
        material=dark_metal,
        name="baggage_frame",
    )
    airframe.visual(
        Box((0.78, 0.16, 0.72)),
        origin=Origin(xyz=(-0.95, 0.69, 0.96)),
        material=dark_metal,
        name="baggage_backer",
    )
    for z in (-0.24, 0.0, 0.24):
        airframe.visual(
            Cylinder(radius=0.012, length=0.09),
            origin=Origin(xyz=(-0.58, 0.795, 0.96 + z)),
            material=hub_gray,
            name=f"frame_hinge_{int((z + 0.24) * 100):02d}",
        )

    tail_boom = sweep_profile_along_spline(
        [
            (-1.54, 0.0, 1.02),
            (-2.40, 0.0, 1.07),
            (-3.65, 0.0, 1.13),
            (-4.95, 0.0, 1.20),
            (-5.92, 0.0, 1.27),
        ],
        profile=rounded_rect_profile(0.30, 0.22, radius=0.055, corner_segments=8),
        samples_per_segment=18,
        cap_profile=True,
    )
    airframe.visual(
        _save_mesh("tail_boom", tail_boom),
        material=body_paint,
        name="tail_boom",
    )
    airframe.visual(
        Box((0.60, 0.08, 0.76)),
        origin=Origin(xyz=(-5.58, 0.0, 1.64)),
        material=body_paint,
        name="vertical_fin",
    )
    airframe.visual(
        Box((0.34, 0.22, 0.22)),
        origin=Origin(xyz=(-5.10, 0.0, 1.31)),
        material=body_paint,
        name="tail_root_fairing",
    )
    airframe.visual(
        Box((0.72, 1.18, 0.05)),
        origin=Origin(xyz=(-4.78, 0.0, 1.07)),
        material=body_paint,
        name="tailplane",
    )
    airframe.visual(
        Cylinder(radius=0.06, length=0.75),
        origin=Origin(xyz=(-6.295, 0.0, 1.32), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="tail_pylon",
    )
    airframe.visual(
        Box((0.12, 0.12, 0.16)),
        origin=Origin(xyz=(-6.61, 0.06, 1.32)),
        material=dark_metal,
        name="tail_gearbox",
    )

    left_skid = tube_from_spline_points(
        [
            (1.15, 0.95, -0.64),
            (0.94, 0.95, -0.74),
            (0.40, 0.95, -0.77),
            (-0.72, 0.95, -0.77),
            (-1.52, 0.95, -0.74),
            (-1.80, 0.95, -0.65),
        ],
        radius=0.055,
        samples_per_segment=14,
        radial_segments=16,
        cap_ends=True,
    )
    right_skid = tube_from_spline_points(
        [
            (1.15, -0.95, -0.64),
            (0.94, -0.95, -0.74),
            (0.40, -0.95, -0.77),
            (-0.72, -0.95, -0.77),
            (-1.52, -0.95, -0.74),
            (-1.80, -0.95, -0.65),
        ],
        radius=0.055,
        samples_per_segment=14,
        radial_segments=16,
        cap_ends=True,
    )
    front_cross = tube_from_spline_points(
        [
            (0.60, 0.95, -0.72),
            (0.60, 0.80, -0.56),
            (0.60, 0.40, -0.12),
            (0.60, 0.0, 0.25),
            (0.60, -0.40, -0.12),
            (0.60, -0.80, -0.56),
            (0.60, -0.95, -0.72),
        ],
        radius=0.040,
        samples_per_segment=14,
        radial_segments=14,
        cap_ends=True,
    )
    rear_cross = tube_from_spline_points(
        [
            (-0.72, 0.95, -0.72),
            (-0.72, 0.80, -0.57),
            (-0.72, 0.40, -0.14),
            (-0.72, 0.0, 0.22),
            (-0.72, -0.40, -0.14),
            (-0.72, -0.80, -0.57),
            (-0.72, -0.95, -0.72),
        ],
        radius=0.040,
        samples_per_segment=14,
        radial_segments=14,
        cap_ends=True,
    )
    airframe.visual(_save_mesh("left_skid", left_skid), material=skid_metal, name="left_skid")
    airframe.visual(_save_mesh("right_skid", right_skid), material=skid_metal, name="right_skid")
    airframe.visual(
        _save_mesh("front_cross_tube", front_cross),
        material=skid_metal,
        name="front_cross_tube",
    )
    airframe.visual(
        _save_mesh("rear_cross_tube", rear_cross),
        material=skid_metal,
        name="rear_cross_tube",
    )

    baggage_door = model.part("baggage_door")
    baggage_door.inertial = Inertial.from_geometry(
        Box((0.78, 0.06, 0.74)),
        mass=18.0,
        origin=Origin(xyz=(-0.39, 0.015, 0.0)),
    )
    baggage_door.visual(
        Box((0.76, 0.03, 0.70)),
        origin=Origin(xyz=(-0.38, 0.015, 0.0)),
        material=body_paint,
        name="door_panel",
    )
    for z in (-0.24, 0.0, 0.24):
        baggage_door.visual(
            Cylinder(radius=0.012, length=0.09),
            origin=Origin(xyz=(-0.01, 0.012, z)),
            material=hub_gray,
            name=f"door_hinge_{int((z + 0.24) * 100):02d}",
        )
    baggage_door.visual(
        Box((0.07, 0.06, 0.02)),
        origin=Origin(xyz=(-0.58, 0.040, 0.0)),
        material=accent_red,
        name="door_handle",
    )

    service_hatch = model.part("service_hatch")
    service_hatch.inertial = Inertial.from_geometry(
        Box((0.58, 0.58, 0.06)),
        mass=7.5,
        origin=Origin(xyz=(-0.29, 0.0, 0.015)),
    )
    service_hatch.visual(
        Box((0.56, 0.54, 0.03)),
        origin=Origin(xyz=(-0.28, 0.0, 0.015)),
        material=body_paint,
        name="hatch_panel",
    )
    service_hatch.visual(
        Box((0.10, 0.10, 0.025)),
        origin=Origin(xyz=(-0.44, 0.0, 0.038)),
        material=accent_red,
        name="hatch_handle",
    )

    main_rotor = model.part("main_rotor")
    main_rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.18, length=0.16),
        mass=55.0,
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
    )
    main_rotor.visual(
        Cylinder(radius=0.16, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=hub_gray,
        name="main_hub",
    )
    main_rotor.visual(
        Cylinder(radius=0.055, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
        material=hub_gray,
        name="mast_cap",
    )
    for prefix, sx, sy in (
        ("x_pos", 1.0, 0.0),
        ("x_neg", -1.0, 0.0),
        ("y_pos", 0.0, 1.0),
        ("y_neg", 0.0, -1.0),
    ):
        yaw = 0.0 if abs(sx) > 0.0 else math.pi / 2.0
        sign = sx if abs(sx) > 0.0 else sy
        main_rotor.visual(
            Box((2.10, 0.22, 0.05)),
            origin=Origin(
                xyz=(sign * 1.05 if abs(sx) > 0.0 else 0.0, sign * 1.05 if abs(sy) > 0.0 else 0.0, 0.09),
                rpy=(0.0, 0.0, yaw),
            ),
            material=rotor_black,
            name=f"{prefix}_blade_root",
        )
        main_rotor.visual(
            Box((2.65, 0.16, 0.035)),
            origin=Origin(
                xyz=(sign * 3.42 if abs(sx) > 0.0 else 0.0, sign * 3.42 if abs(sy) > 0.0 else 0.0, 0.08),
                rpy=(0.0, 0.0, yaw),
            ),
            material=rotor_black,
            name=f"{prefix}_blade_tip",
        )

    tail_rotor = model.part("tail_rotor")
    tail_rotor.inertial = Inertial.from_geometry(
        Box((0.92, 0.12, 0.92)),
        mass=9.0,
    )
    tail_rotor.visual(
        Cylinder(radius=0.06, length=0.10),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hub_gray,
        name="tail_hub",
    )
    tail_rotor.visual(
        Box((0.78, 0.05, 0.12)),
        origin=Origin(xyz=(0.39, 0.0, 0.0)),
        material=rotor_black,
        name="tail_blade_x_pos",
    )
    tail_rotor.visual(
        Box((0.78, 0.05, 0.12)),
        origin=Origin(xyz=(-0.39, 0.0, 0.0)),
        material=rotor_black,
        name="tail_blade_x_neg",
    )
    tail_rotor.visual(
        Box((0.12, 0.05, 0.78)),
        origin=Origin(xyz=(0.0, 0.0, 0.39)),
        material=rotor_black,
        name="tail_blade_z_pos",
    )
    tail_rotor.visual(
        Box((0.12, 0.05, 0.78)),
        origin=Origin(xyz=(0.0, 0.0, -0.39)),
        material=rotor_black,
        name="tail_blade_z_neg",
    )

    model.articulation(
        "airframe_to_baggage_door",
        ArticulationType.REVOLUTE,
        parent=airframe,
        child=baggage_door,
        origin=Origin(xyz=(-0.57, 0.807, 0.96)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.6,
            lower=0.0,
            upper=math.radians(92.0),
        ),
    )
    model.articulation(
        "airframe_to_service_hatch",
        ArticulationType.REVOLUTE,
        parent=airframe,
        child=service_hatch,
        origin=Origin(xyz=(-0.27, 0.0, 1.862)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )
    model.articulation(
        "mast_to_main_rotor",
        ArticulationType.CONTINUOUS,
        parent=airframe,
        child=main_rotor,
        origin=Origin(xyz=(0.0, 0.0, 2.14)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=35.0),
    )
    model.articulation(
        "tail_to_tail_rotor",
        ArticulationType.CONTINUOUS,
        parent=airframe,
        child=tail_rotor,
        origin=Origin(xyz=(-6.73, 0.17, 1.32)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=55.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    airframe = object_model.get_part("airframe")
    baggage_door = object_model.get_part("baggage_door")
    service_hatch = object_model.get_part("service_hatch")

    door_joint = object_model.get_articulation("airframe_to_baggage_door")
    hatch_joint = object_model.get_articulation("airframe_to_service_hatch")
    main_rotor_joint = object_model.get_articulation("mast_to_main_rotor")
    tail_rotor_joint = object_model.get_articulation("tail_to_tail_rotor")

    ctx.check(
        "main rotor uses vertical mast axis",
        main_rotor_joint.articulation_type == ArticulationType.CONTINUOUS
        and main_rotor_joint.axis == (0.0, 0.0, 1.0),
        details=f"type={main_rotor_joint.articulation_type}, axis={main_rotor_joint.axis}",
    )
    ctx.check(
        "tail rotor uses transverse tail axis",
        tail_rotor_joint.articulation_type == ArticulationType.CONTINUOUS
        and tail_rotor_joint.axis == (0.0, 1.0, 0.0),
        details=f"type={tail_rotor_joint.articulation_type}, axis={tail_rotor_joint.axis}",
    )
    ctx.check(
        "baggage door hinges on vertical axis",
        abs(door_joint.axis[2]) > 0.99 and abs(door_joint.axis[0]) < 1e-6 and abs(door_joint.axis[1]) < 1e-6,
        details=f"axis={door_joint.axis}",
    )
    ctx.check(
        "service hatch hinges on transverse roof axis",
        hatch_joint.axis == (0.0, 1.0, 0.0),
        details=f"axis={hatch_joint.axis}",
    )

    with ctx.pose({door_joint: 0.0, hatch_joint: 0.0}):
        ctx.expect_gap(
            baggage_door,
            airframe,
            axis="y",
            max_gap=0.008,
            max_penetration=0.0,
            positive_elem="door_panel",
            negative_elem="baggage_frame",
            name="baggage door sits flush to aft side frame",
        )
        ctx.expect_overlap(
            baggage_door,
            airframe,
            axes="xz",
            min_overlap=0.58,
            elem_a="door_panel",
            elem_b="baggage_frame",
            name="baggage door covers the baggage opening",
        )
        ctx.expect_gap(
            service_hatch,
            airframe,
            axis="z",
            max_gap=0.008,
            max_penetration=0.0,
            positive_elem="hatch_panel",
            negative_elem="service_deck",
            name="service hatch rests on the upper deck",
        )
        ctx.expect_overlap(
            service_hatch,
            airframe,
            axes="xy",
            min_overlap=0.48,
            elem_a="hatch_panel",
            elem_b="service_deck",
            name="service hatch covers the service deck",
        )
        closed_door_aabb = ctx.part_element_world_aabb(baggage_door, elem="door_panel")
        closed_hatch_aabb = ctx.part_element_world_aabb(service_hatch, elem="hatch_panel")

    with ctx.pose(
        {
            door_joint: math.radians(75.0),
            hatch_joint: math.radians(70.0),
        }
    ):
        open_door_aabb = ctx.part_element_world_aabb(baggage_door, elem="door_panel")
        open_hatch_aabb = ctx.part_element_world_aabb(service_hatch, elem="hatch_panel")

    ctx.check(
        "baggage door swings outward from the fuselage side",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][1] > closed_door_aabb[1][1] + 0.28,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )
    ctx.check(
        "service hatch lifts above the mast deck",
        closed_hatch_aabb is not None
        and open_hatch_aabb is not None
        and open_hatch_aabb[1][2] > closed_hatch_aabb[1][2] + 0.16,
        details=f"closed={closed_hatch_aabb}, open={open_hatch_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
