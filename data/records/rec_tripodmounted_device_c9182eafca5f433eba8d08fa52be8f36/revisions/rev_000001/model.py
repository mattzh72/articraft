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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tripod_pan_tilt_device")

    tripod_black = model.material("tripod_black", rgba=(0.12, 0.12, 0.13, 1.0))
    dark_anthracite = model.material("dark_anthracite", rgba=(0.20, 0.21, 0.23, 1.0))
    cast_gray = model.material("cast_gray", rgba=(0.45, 0.47, 0.50, 1.0))
    rubber = model.material("rubber", rgba=(0.09, 0.09, 0.10, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.72, 0.74, 0.77, 1.0))

    leg_splay = math.radians(24.0)
    hinge_radius = 0.106

    def leg_frame_xyz(phi: float, x_local: float, y_local: float, z_local: float) -> tuple[float, float, float]:
        radial = math.cos(leg_splay) * x_local - math.sin(leg_splay) * z_local
        return (
            hinge_radius * math.cos(phi) + radial * math.cos(phi) - y_local * math.sin(phi),
            hinge_radius * math.sin(phi) + radial * math.sin(phi) + y_local * math.cos(phi),
            0.918 + math.sin(leg_splay) * x_local + math.cos(leg_splay) * z_local,
        )

    def crown_hinge_xyz(phi: float, x_local: float, y_local: float, z_local: float) -> tuple[float, float, float]:
        return (
            hinge_radius * math.cos(phi) + x_local * math.cos(phi) - y_local * math.sin(phi),
            hinge_radius * math.sin(phi) + x_local * math.sin(phi) + y_local * math.cos(phi),
            0.918 + z_local,
        )

    crown = model.part("crown")
    crown.visual(
        Cylinder(radius=0.080, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.900)),
        material=dark_anthracite,
        name="crown_hub",
    )
    crown.visual(
        Cylinder(radius=0.100, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.945)),
        material=cast_gray,
        name="top_plate",
    )
    crown.visual(
        Cylinder(radius=0.055, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.970)),
        material=cast_gray,
        name="pan_bearing_collar",
    )

    for idx, phi in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0), start=1):
        crown.visual(
            Box((0.070, 0.072, 0.060)),
            origin=Origin(
                xyz=leg_frame_xyz(phi, -0.050, 0.0, 0.005),
                rpy=(0.0, -leg_splay, phi),
            ),
            material=dark_anthracite,
            name=f"leg_arm_{idx}",
        )
        crown.visual(
            Box((0.044, 0.086, 0.018)),
            origin=Origin(
                xyz=leg_frame_xyz(phi, 0.000, 0.0, 0.033),
                rpy=(0.0, -leg_splay, phi),
            ),
            material=cast_gray,
            name=f"leg_yoke_bridge_{idx}",
        )
        for lug_side, y_local in (("outer", 0.034), ("inner", -0.034)):
            crown.visual(
                Box((0.030, 0.014, 0.058)),
                origin=Origin(
                    xyz=leg_frame_xyz(phi, 0.034, y_local, 0.0),
                    rpy=(0.0, -leg_splay, phi),
                ),
                material=cast_gray,
                name=f"leg_hinge_lug_{idx}_{lug_side}",
            )
    crown.inertial = Inertial.from_geometry(
        Box((0.28, 0.28, 0.16)),
        mass=2.6,
        origin=Origin(xyz=(0.0, 0.0, 0.900)),
    )

    def add_leg(part_name: str, material_name: str) -> None:
        leg = model.part(part_name)
        leg.visual(
            Cylinder(radius=0.015, length=0.052),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=material_name,
            name="hinge_knuckle",
        )
        leg.visual(
            Box((0.050, 0.030, 0.090)),
            origin=Origin(xyz=(0.032, 0.0, -0.050)),
            material=material_name,
            name="upper_block",
        )
        for rail_index, y_off in enumerate((-0.015, 0.015), start=1):
            leg.visual(
                Box((0.026, 0.012, 0.830)),
                origin=Origin(xyz=(0.028, y_off, -0.500)),
                material=tripod_black,
                name=f"rail_{rail_index}",
            )
        leg.visual(
            Box((0.074, 0.062, 0.040)),
            origin=Origin(xyz=(0.0, 0.0, -0.905)),
            material=material_name,
            name="foot_block",
        )
        leg.visual(
            Box((0.095, 0.058, 0.022)),
            origin=Origin(xyz=(0.0, 0.0, -0.932), rpy=(0.0, math.radians(24.0), 0.0)),
            material=rubber,
            name="foot_pad",
        )
        leg.inertial = Inertial.from_geometry(
            Box((0.11, 0.07, 0.98)),
            mass=1.0,
            origin=Origin(xyz=(0.0, 0.0, -0.490)),
        )

    add_leg("leg_front", dark_anthracite)
    add_leg("leg_left", dark_anthracite)
    add_leg("leg_right", dark_anthracite)

    leg_limit_inward = math.radians(-20.0)
    leg_limit_outward = math.radians(14.0)
    for part_name, phi in (
        ("leg_front", 0.0),
        ("leg_left", 2.0 * math.pi / 3.0),
        ("leg_right", 4.0 * math.pi / 3.0),
    ):
        model.articulation(
            f"crown_to_{part_name}",
            ArticulationType.REVOLUTE,
            parent=crown,
            child=part_name,
            origin=Origin(
                xyz=(hinge_radius * math.cos(phi), hinge_radius * math.sin(phi), 0.918),
                rpy=(0.0, -leg_splay, phi),
            ),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=40.0,
                velocity=1.5,
                lower=leg_limit_inward,
                upper=leg_limit_outward,
            ),
        )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.085, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=dark_anthracite,
        name="turntable_base",
    )
    pan_head.visual(
        Cylinder(radius=0.060, length=0.102),
        origin=Origin(xyz=(0.0, 0.0, 0.079)),
        material=cast_gray,
        name="pan_spindle_housing",
    )
    pan_head.visual(
        Box((0.100, 0.126, 0.074)),
        origin=Origin(xyz=(0.0, 0.0, 0.128)),
        material=cast_gray,
        name="pan_neck",
    )
    pan_head.visual(
        Box((0.142, 0.182, 0.078)),
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        material=cast_gray,
        name="yoke_bridge",
    )
    pan_head.visual(
        Box((0.092, 0.162, 0.126)),
        origin=Origin(xyz=(-0.042, 0.0, 0.220)),
        material=dark_anthracite,
        name="rear_brace",
    )
    for side_name, y_center in (("left", 0.116), ("right", -0.116)):
        pan_head.visual(
            Box((0.118, 0.050, 0.214)),
            origin=Origin(xyz=(0.0, y_center, 0.300)),
            material=cast_gray,
            name=f"{side_name}_side_support",
        )
        pan_head.visual(
            Cylinder(radius=0.046, length=0.040),
            origin=Origin(
                xyz=(0.0, 0.136 if side_name == "left" else -0.136, 0.336),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_anthracite,
            name=f"{side_name}_bearing_housing",
        )
    pan_head.inertial = Inertial.from_geometry(
        Box((0.20, 0.32, 0.42)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.210)),
    )

    device_stage = model.part("device_stage")
    device_stage.visual(
        Cylinder(radius=0.026, length=0.182),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_anthracite,
        name="tilt_trunnion",
    )
    device_stage.visual(
        Box((0.112, 0.112, 0.100)),
        origin=Origin(xyz=(0.024, 0.0, 0.000)),
        material=dark_anthracite,
        name="tilt_core",
    )
    device_stage.visual(
        Box((0.106, 0.082, 0.152)),
        origin=Origin(xyz=(0.042, 0.0, 0.076)),
        material=cast_gray,
        name="tilt_upright",
    )
    device_stage.visual(
        Box((0.240, 0.120, 0.014)),
        origin=Origin(xyz=(0.084, 0.0, 0.154)),
        material=satin_metal,
        name="mount_plate",
    )
    device_stage.visual(
        Box((0.108, 0.084, 0.020)),
        origin=Origin(xyz=(0.064, 0.0, 0.171)),
        material=dark_anthracite,
        name="plate_clamp",
    )
    device_stage.visual(
        Box((0.190, 0.110, 0.120)),
        origin=Origin(xyz=(0.084, 0.0, 0.231)),
        material=tripod_black,
        name="device_body",
    )
    device_stage.visual(
        Cylinder(radius=0.034, length=0.060),
        origin=Origin(xyz=(0.205, 0.0, 0.231), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_anthracite,
        name="lens_barrel",
    )
    device_stage.inertial = Inertial.from_geometry(
        Box((0.28, 0.18, 0.30)),
        mass=1.5,
        origin=Origin(xyz=(0.085, 0.0, 0.155)),
    )

    model.articulation(
        "crown_to_pan_head",
        ArticulationType.CONTINUOUS,
        parent=crown,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 0.985)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.5),
    )
    model.articulation(
        "pan_head_to_device_stage",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=device_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.336)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=math.radians(-45.0),
            upper=math.radians(60.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    crown = object_model.get_part("crown")
    pan_head = object_model.get_part("pan_head")
    device_stage = object_model.get_part("device_stage")
    pan_joint = object_model.get_articulation("crown_to_pan_head")
    tilt_joint = object_model.get_articulation("pan_head_to_device_stage")

    ctx.expect_overlap(
        pan_head,
        crown,
        axes="xy",
        min_overlap=0.080,
        name="pan head is centered over the tripod crown",
    )
    ctx.expect_gap(
        device_stage,
        pan_head,
        axis="z",
        positive_elem="mount_plate",
        negative_elem="yoke_bridge",
        min_gap=0.250,
        max_gap=0.300,
        name="device plate clears the yoke bridge",
    )

    def elem_center(part_name: str, elem_name: str) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(part_name, elem=elem_name)
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    foot_mins = []
    for part_name in ("leg_front", "leg_left", "leg_right"):
        aabb = ctx.part_element_world_aabb(part_name, elem="foot_pad")
        if aabb is not None:
            foot_mins.append(aabb[0][2])
    ctx.check(
        "all tripod feet sit on nearly the same ground plane",
        len(foot_mins) == 3 and (max(foot_mins) - min(foot_mins)) <= 0.012 and min(foot_mins) >= -0.002,
        details=f"foot_z_mins={foot_mins}",
    )

    rest_lens = elem_center("device_stage", "lens_barrel")
    with ctx.pose({tilt_joint: math.radians(35.0)}):
        tilted_lens = elem_center("device_stage", "lens_barrel")
    ctx.check(
        "positive tilt raises the device nose",
        rest_lens is not None and tilted_lens is not None and tilted_lens[2] > rest_lens[2] + 0.07,
        details=f"rest_lens={rest_lens}, tilted_lens={tilted_lens}",
    )

    with ctx.pose({pan_joint: math.pi / 2.0}):
        panned_lens = elem_center("device_stage", "lens_barrel")
    ctx.check(
        "pan stage rotates the device about vertical",
        rest_lens is not None
        and panned_lens is not None
        and abs(panned_lens[1]) > abs(rest_lens[1]) + 0.15
        and abs(panned_lens[0]) < abs(rest_lens[0]) * 0.4,
        details=f"rest_lens={rest_lens}, panned_lens={panned_lens}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
