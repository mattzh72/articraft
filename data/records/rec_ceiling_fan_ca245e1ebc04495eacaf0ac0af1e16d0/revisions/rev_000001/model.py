from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    MeshGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


BLADE_COUNT = 5
BLADE_ROOT_RADIUS = 0.27
BLADE_TIP_RADIUS = 0.76
BLADE_PITCH = math.radians(9.0)


def _blade_profile() -> list[tuple[float, float]]:
    """Planform for one gently tapered residential ceiling-fan blade."""
    root_w = 0.105
    mid_w = 0.145
    tip_w = 0.125
    tip_cap = tip_w * 0.50
    tip_center_x = BLADE_TIP_RADIUS - tip_cap

    points: list[tuple[float, float]] = [
        (BLADE_ROOT_RADIUS, -root_w * 0.50),
        (0.43, -mid_w * 0.50),
        (tip_center_x, -tip_w * 0.50),
    ]
    for i in range(1, 9):
        theta = -math.pi / 2.0 + i * math.pi / 8.0
        points.append(
            (
                tip_center_x + tip_cap * math.cos(theta),
                tip_cap * math.sin(theta),
            )
        )
    points.extend(
        [
            (0.43, mid_w * 0.50),
            (BLADE_ROOT_RADIUS, root_w * 0.50),
        ]
    )
    return points


def _blade_mesh():
    return mesh_from_geometry(ExtrudeGeometry(_blade_profile(), 0.014), "wood_blade")


def _rotor_spider_mesh():
    """Hidden five-spoke rotor carrier that mechanically links the hub ring to the bearing."""
    geom = MeshGeometry()
    geom.merge(CylinderGeometry(radius=0.055, height=0.050, radial_segments=36).translate(0.0, 0.0, 0.002))
    for index in range(BLADE_COUNT):
        angle = index * math.tau / BLADE_COUNT
        geom.merge(BoxGeometry((0.295, 0.024, 0.018)).translate(0.1475, 0.0, 0.002).rotate_z(angle))
    return mesh_from_geometry(geom, "rotor_spider")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="five_blade_ceiling_fan")

    brushed_nickel = model.material("brushed_nickel", rgba=(0.62, 0.60, 0.56, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.12, 0.12, 0.12, 1.0))
    warm_wood = model.material("warm_wood", rgba=(0.53, 0.31, 0.13, 1.0))
    frosted_glass = model.material("frosted_glass", rgba=(0.95, 0.89, 0.68, 0.58))
    ceiling_white = model.material("ceiling_white", rgba=(0.92, 0.91, 0.88, 1.0))

    mount = model.part("mount")
    mount.visual(
        Box((0.34, 0.34, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.844)),
        material=ceiling_white,
        name="ceiling_plate",
    )
    mount.visual(
        Cylinder(radius=0.120, length=0.085),
        origin=Origin(xyz=(0.0, 0.0, 0.796)),
        material=brushed_nickel,
        name="ceiling_canopy",
    )
    mount.visual(
        Cylinder(radius=0.024, length=0.610),
        origin=Origin(xyz=(0.0, 0.0, 0.455)),
        material=dark_metal,
        name="downrod",
    )
    mount.visual(
        Cylinder(radius=0.056, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, 0.158)),
        material=brushed_nickel,
        name="upper_collar",
    )
    mount.visual(
        Cylinder(radius=0.152, length=0.215),
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
        material=brushed_nickel,
        name="motor_housing",
    )
    mount.visual(
        Cylinder(radius=0.171, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.121)),
        material=brushed_nickel,
        name="top_flange",
    )
    mount.visual(
        Cylinder(radius=0.165, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, -0.121)),
        material=brushed_nickel,
        name="lower_flange",
    )
    mount.visual(
        Cylinder(radius=0.033, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, -0.168)),
        material=dark_metal,
        name="light_stem",
    )
    mount.visual(
        Cylinder(radius=0.108, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, -0.206)),
        material=brushed_nickel,
        name="globe_ring",
    )
    mount.visual(
        Sphere(radius=0.116),
        origin=Origin(xyz=(0.0, 0.0, -0.294)),
        material=frosted_glass,
        name="glass_globe",
    )

    blade_mesh = _blade_mesh()
    rotor = model.part("rotor")
    rotor.visual(
        _rotor_spider_mesh(),
        material=dark_metal,
        name="rotor_spider",
    )
    rotor.visual(
        mesh_from_geometry(TorusGeometry(radius=0.187, tube=0.015, radial_segments=18, tubular_segments=64), "blade_hub_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=dark_metal,
        name="hub_ring",
    )

    for index in range(BLADE_COUNT):
        angle = index * math.tau / BLADE_COUNT
        rotor.visual(
            Box((0.160, 0.038, 0.022)),
            origin=Origin(
                xyz=(0.245 * math.cos(angle), 0.245 * math.sin(angle), 0.002),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark_metal,
            name=f"blade_arm_{index}",
        )
        rotor.visual(
            blade_mesh,
            origin=Origin(rpy=(BLADE_PITCH, 0.0, angle)),
            material=warm_wood,
            name=f"blade_{index}",
        )

    model.articulation(
        "central_spin",
        ArticulationType.CONTINUOUS,
        parent=mount,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=12.0),
        motion_properties=MotionProperties(damping=0.01, friction=0.002),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    mount = object_model.get_part("mount")
    rotor = object_model.get_part("rotor")
    spin = object_model.get_articulation("central_spin")

    ctx.allow_overlap(
        mount,
        rotor,
        elem_a="motor_housing",
        elem_b="rotor_spider",
        reason="The rotor spider represents the hidden rotating motor core captured inside the simplified solid motor housing.",
    )
    ctx.expect_overlap(
        mount,
        rotor,
        axes="xy",
        elem_a="motor_housing",
        elem_b="rotor_spider",
        min_overlap=0.040,
        name="hidden rotor core sits inside the motor housing footprint",
    )
    ctx.expect_overlap(
        mount,
        rotor,
        axes="z",
        elem_a="motor_housing",
        elem_b="rotor_spider",
        min_overlap=0.030,
        name="hidden rotor core is captured through the motor housing height",
    )

    ctx.check("five_blades_modeled", len([v for v in rotor.visuals if v.name.startswith("blade_") and v.name[6:].isdigit()]) == 5)
    ctx.check(
        "central_joint_is_continuous",
        spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={spin.articulation_type!r}",
    )

    globe_aabb = ctx.part_element_world_aabb(mount, elem="glass_globe")
    motor_aabb = ctx.part_element_world_aabb(mount, elem="motor_housing")
    ctx.check(
        "globe_light_below_motor",
        globe_aabb is not None
        and motor_aabb is not None
        and float(globe_aabb[1][2]) < float(motor_aabb[0][2]) - 0.020,
        details=f"globe={globe_aabb}, motor={motor_aabb}",
    )

    blade0_rest = ctx.part_element_world_aabb(rotor, elem="blade_0")
    with ctx.pose({spin: math.pi / 2.0}):
        blade0_quarter = ctx.part_element_world_aabb(rotor, elem="blade_0")
    if blade0_rest is not None and blade0_quarter is not None:
        rest_center = (
            0.5 * (float(blade0_rest[0][0]) + float(blade0_rest[1][0])),
            0.5 * (float(blade0_rest[0][1]) + float(blade0_rest[1][1])),
        )
        quarter_center = (
            0.5 * (float(blade0_quarter[0][0]) + float(blade0_quarter[1][0])),
            0.5 * (float(blade0_quarter[0][1]) + float(blade0_quarter[1][1])),
        )
        ctx.check(
            "blade_sweeps_about_downrod",
            rest_center[0] > 0.45 and abs(rest_center[1]) < 0.10 and quarter_center[1] > 0.45,
            details=f"rest={rest_center}, quarter_turn={quarter_center}",
        )
    else:
        ctx.fail("blade_sweeps_about_downrod", "Could not measure blade_0 before and after rotation.")

    return ctx.report()


object_model = build_object_model()
