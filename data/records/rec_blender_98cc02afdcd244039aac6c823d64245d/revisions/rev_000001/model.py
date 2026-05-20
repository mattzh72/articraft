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
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="immersion_blender")

    graphite = model.material("graphite", rgba=(0.18, 0.19, 0.20, 1.0))
    soft_black = model.material("soft_black", rgba=(0.10, 0.10, 0.11, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.77, 0.79, 0.81, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.66, 0.68, 0.71, 1.0))
    accent_gray = model.material("accent_gray", rgba=(0.35, 0.37, 0.40, 1.0))

    motor_body = model.part("motor_body")
    motor_body.visual(
        Cylinder(radius=0.026, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=graphite,
        name="body_neck",
    )
    motor_body.visual(
        Cylinder(radius=0.030, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=accent_gray,
        name="body_coupler",
    )
    motor_body.visual(
        Cylinder(radius=0.033, length=0.170),
        origin=Origin(xyz=(0.0, 0.0, 0.135)),
        material=graphite,
        name="main_housing",
    )
    motor_body.visual(
        Cylinder(radius=0.034, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.221)),
        material=accent_gray,
        name="upper_band",
    )
    motor_body.visual(
        Sphere(radius=0.033),
        origin=Origin(xyz=(0.0, 0.0, 0.250)),
        material=graphite,
        name="top_cap",
    )
    motor_body.visual(
        Box((0.024, 0.008, 0.084)),
        origin=Origin(xyz=(0.0, 0.028, 0.160)),
        material=soft_black,
        name="switch_pad",
    )
    motor_body.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(0.0, 0.034, 0.188), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=soft_black,
        name="power_button",
    )
    motor_body.visual(
        Cylinder(radius=0.007, length=0.005),
        origin=Origin(xyz=(0.0, 0.0335, 0.144), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=soft_black,
        name="turbo_button",
    )
    motor_body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.034, length=0.283),
        mass=0.95,
        origin=Origin(xyz=(0.0, 0.0, 0.1415)),
    )

    shaft = model.part("blending_shaft")
    shaft.visual(
        Cylinder(radius=0.028, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=accent_gray,
        name="shaft_collar",
    )
    shaft.visual(
        Cylinder(radius=0.030, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=soft_black,
        name="bayonet_grip",
    )
    shaft.visual(
        Box((0.014, 0.008, 0.006)),
        origin=Origin(xyz=(0.031, 0.0, -0.012)),
        material=soft_black,
        name="bayonet_tab_right",
    )
    shaft.visual(
        Box((0.014, 0.008, 0.006)),
        origin=Origin(xyz=(-0.031, 0.0, -0.012)),
        material=soft_black,
        name="bayonet_tab_left",
    )
    shaft.visual(
        Cylinder(radius=0.011, length=0.160),
        origin=Origin(xyz=(0.0, 0.0, -0.100)),
        material=satin_steel,
        name="shaft_tube",
    )
    shaft.visual(
        Cylinder(radius=0.016, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.188)),
        material=brushed_steel,
        name="guard_neck",
    )
    guard_ring_mesh = _save_mesh(
        "guard_ring",
        LatheGeometry.from_shell_profiles(
            [
                (0.0160, -0.196),
                (0.0185, -0.203),
                (0.0225, -0.214),
                (0.0245, -0.225),
                (0.0240, -0.231),
                (0.0225, -0.234),
            ],
            [
                (0.0115, -0.196),
                (0.0135, -0.203),
                (0.0175, -0.214),
                (0.0200, -0.225),
                (0.0195, -0.231),
                (0.0165, -0.234),
            ],
            segments=56,
            start_cap="flat",
            end_cap="flat",
        ),
    )
    shaft.visual(
        guard_ring_mesh,
        material=brushed_steel,
        name="guard_ring",
    )
    shaft.visual(
        Cylinder(radius=0.0035, length=0.021),
        origin=Origin(xyz=(0.0, 0.0, -0.2065)),
        material=brushed_steel,
        name="drive_spindle",
    )
    shaft.inertial = Inertial.from_geometry(
        Cylinder(radius=0.030, length=0.236),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.0, -0.118)),
    )

    blade = model.part("blade_assembly")
    blade.visual(
        Cylinder(radius=0.006, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=brushed_steel,
        name="blade_hub",
    )
    blade.visual(
        Box((0.024, 0.004, 0.0012)),
        origin=Origin(xyz=(0.0, 0.0, -0.004), rpy=(0.0, 0.28, 0.0)),
        material=satin_steel,
        name="blade_fin_a",
    )
    blade.visual(
        Box((0.024, 0.004, 0.0012)),
        origin=Origin(xyz=(0.0, 0.0, -0.004), rpy=(0.0, -0.28, math.pi / 2.0)),
        material=satin_steel,
        name="blade_fin_b",
    )
    blade.inertial = Inertial.from_geometry(
        Cylinder(radius=0.013, length=0.010),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
    )

    model.articulation(
        "body_to_shaft_lock",
        ArticulationType.REVOLUTE,
        parent=motor_body,
        child=shaft,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=3.0,
            lower=0.0,
            upper=math.pi / 2.0,
        ),
    )
    model.articulation(
        "shaft_to_blade_spin",
        ArticulationType.CONTINUOUS,
        parent=shaft,
        child=blade,
        origin=Origin(xyz=(0.0, 0.0, -0.217)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=60.0,
        ),
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
    motor_body = object_model.get_part("motor_body")
    shaft = object_model.get_part("blending_shaft")
    blade = object_model.get_part("blade_assembly")
    shaft_lock = object_model.get_articulation("body_to_shaft_lock")
    blade_spin = object_model.get_articulation("shaft_to_blade_spin")

    ctx.expect_gap(
        motor_body,
        shaft,
        axis="z",
        positive_elem="body_coupler",
        negative_elem="shaft_collar",
        max_gap=0.0005,
        max_penetration=0.0,
        name="bayonet collar seats flush against motor coupler",
    )
    ctx.expect_overlap(
        motor_body,
        shaft,
        axes="xy",
        elem_a="body_coupler",
        elem_b="shaft_collar",
        min_overlap=0.050,
        name="shaft collar stays coaxial with motor coupler",
    )
    ctx.expect_gap(
        shaft,
        blade,
        axis="z",
        positive_elem="drive_spindle",
        negative_elem="blade_hub",
        max_gap=0.0005,
        max_penetration=0.0,
        name="blade hub is mounted directly below the drive spindle",
    )
    ctx.expect_within(
        blade,
        shaft,
        axes="xy",
        inner_elem="blade_fin_a",
        outer_elem="guard_ring",
        margin=0.0,
        name="blade fin remains inside the guard ring footprint",
    )
    ctx.expect_within(
        blade,
        shaft,
        axes="xy",
        inner_elem="blade_fin_b",
        outer_elem="guard_ring",
        margin=0.0,
        name="cross blade remains inside the guard ring footprint",
    )

    rest_blade_pos = ctx.part_world_position(blade)
    with ctx.pose({shaft_lock: math.pi / 2.0}):
        ctx.expect_gap(
            motor_body,
            shaft,
            axis="z",
            positive_elem="body_coupler",
            negative_elem="shaft_collar",
            max_gap=0.0005,
            max_penetration=0.0,
            name="quarter-turn lock keeps the shaft seated",
        )
        locked_blade_pos = ctx.part_world_position(blade)
    ctx.check(
        "quarter-turn bayonet joint is limited to ninety degrees",
        shaft_lock.motion_limits is not None
        and shaft_lock.motion_limits.lower is not None
        and shaft_lock.motion_limits.upper is not None
        and abs(shaft_lock.motion_limits.lower) < 1e-9
        and abs(shaft_lock.motion_limits.upper - (math.pi / 2.0)) < 1e-9,
        details=f"limits={shaft_lock.motion_limits}",
    )
    ctx.check(
        "twisting the shaft does not translate the blade assembly",
        rest_blade_pos is not None
        and locked_blade_pos is not None
        and max(abs(a - b) for a, b in zip(rest_blade_pos, locked_blade_pos)) < 1e-6,
        details=f"rest={rest_blade_pos}, locked={locked_blade_pos}",
    )

    blade_rest_pos = ctx.part_world_position(blade)
    with ctx.pose({blade_spin: 1.7}):
        blade_spun_pos = ctx.part_world_position(blade)
    ctx.check(
        "blade assembly spins continuously about its own axis without shifting",
        blade_spin.articulation_type == ArticulationType.CONTINUOUS
        and blade_rest_pos is not None
        and blade_spun_pos is not None
        and max(abs(a - b) for a, b in zip(blade_rest_pos, blade_spun_pos)) < 1e-6,
        details=f"type={blade_spin.articulation_type}, rest={blade_rest_pos}, spun={blade_spun_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
