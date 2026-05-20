from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _rotated_lathe_mesh(profile, name: str, *, segments: int = 72):
    """Build a lathed mesh whose revolution axis is the engine X centerline."""
    geom = LatheGeometry(profile, segments=segments, closed=True)
    geom.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _rotated_shell_mesh(outer_profile, inner_profile, name: str, *, segments: int = 96):
    """Build a hollow thin-wall nacelle shell open on the longitudinal axis."""
    geom = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=segments,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )
    geom.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _rotated_torus_mesh(radius: float, tube: float, name: str):
    geom = TorusGeometry(radius, tube, radial_segments=28, tubular_segments=96)
    geom.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="turbofan_nacelle_display")

    nacelle_paint = model.material("warm_white_composite", rgba=(0.86, 0.86, 0.80, 1.0))
    intake_metal = model.material("polished_intake_lip", rgba=(0.78, 0.80, 0.82, 1.0))
    dark_metal = model.material("dark_exhaust_metal", rgba=(0.08, 0.085, 0.09, 1.0))
    fan_metal = model.material("brushed_fan_metal", rgba=(0.55, 0.58, 0.60, 1.0))
    spinner_metal = model.material("satin_spinner", rgba=(0.70, 0.72, 0.74, 1.0))
    stand_black = model.material("display_stand_black", rgba=(0.015, 0.016, 0.018, 1.0))

    nacelle = model.part("nacelle")

    # A real nacelle is a thin hollow duct rather than a solid cylinder.  These
    # profiles are radius-vs-X before rotating the lathe axis onto +X.
    outer_profile = [
        (0.355, -0.640),
        (0.395, -0.570),
        (0.420, -0.420),
        (0.415, -0.080),
        (0.390, 0.290),
        (0.315, 0.640),
    ]
    inner_profile = [
        (0.285, -0.640),
        (0.305, -0.500),
        (0.300, -0.160),
        (0.265, 0.250),
        (0.205, 0.640),
    ]
    nacelle.visual(
        _rotated_shell_mesh(outer_profile, inner_profile, "nacelle_shell"),
        origin=Origin(),
        material=nacelle_paint,
        name="nacelle_shell",
    )
    nacelle.visual(
        _rotated_torus_mesh(0.320, 0.038, "front_lip"),
        origin=Origin(xyz=(-0.640, 0.0, 0.0)),
        material=intake_metal,
        name="front_lip",
    )
    nacelle.visual(
        _rotated_torus_mesh(0.250, 0.026, "rear_lip"),
        origin=Origin(xyz=(0.640, 0.0, 0.0)),
        material=dark_metal,
        name="rear_lip",
    )

    # Static rear centerbody/cone and four simple struts keep the exhaust cone
    # visibly mounted on the same centerline as the fan.
    exhaust_profile = [
        (0.0, 0.135),
        (0.082, 0.185),
        (0.118, 0.320),
        (0.112, 0.470),
        (0.060, 0.625),
        (0.0, 0.710),
    ]
    nacelle.visual(
        _rotated_lathe_mesh(exhaust_profile, "exhaust_cone", segments=80),
        origin=Origin(),
        material=dark_metal,
        name="exhaust_cone",
    )
    nacelle.visual(
        Box((0.170, 0.016, 0.130)),
        origin=Origin(xyz=(0.440, 0.0, 0.178)),
        material=dark_metal,
        name="upper_strut",
    )
    nacelle.visual(
        Box((0.170, 0.016, 0.130)),
        origin=Origin(xyz=(0.440, 0.0, -0.178)),
        material=dark_metal,
        name="lower_strut",
    )
    nacelle.visual(
        Box((0.170, 0.130, 0.016)),
        origin=Origin(xyz=(0.440, 0.178, 0.0)),
        material=dark_metal,
        name="side_strut_0",
    )
    nacelle.visual(
        Box((0.170, 0.130, 0.016)),
        origin=Origin(xyz=(0.440, -0.178, 0.0)),
        material=dark_metal,
        name="side_strut_1",
    )
    nacelle.visual(
        Cylinder(radius=0.036, length=0.190),
        origin=Origin(xyz=(-0.210, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="front_bearing",
    )
    nacelle.visual(
        Box((0.065, 0.012, 0.320)),
        origin=Origin(xyz=(-0.145, 0.0, 0.195)),
        material=dark_metal,
        name="upper_vane",
    )
    nacelle.visual(
        Box((0.065, 0.012, 0.320)),
        origin=Origin(xyz=(-0.145, 0.0, -0.195)),
        material=dark_metal,
        name="lower_vane",
    )
    nacelle.visual(
        Box((0.065, 0.320, 0.012)),
        origin=Origin(xyz=(-0.145, 0.195, 0.0)),
        material=dark_metal,
        name="side_vane_0",
    )
    nacelle.visual(
        Box((0.065, 0.320, 0.012)),
        origin=Origin(xyz=(-0.145, -0.195, 0.0)),
        material=dark_metal,
        name="side_vane_1",
    )

    # The short museum-style display stand is welded into the underside of the
    # static nacelle assembly so that the root part is one supported structure.
    nacelle.visual(
        Box((0.720, 0.440, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, -0.840)),
        material=stand_black,
        name="base_plate",
    )
    nacelle.visual(
        Cylinder(radius=0.045, length=0.390),
        origin=Origin(xyz=(0.0, 0.0, -0.625)),
        material=stand_black,
        name="pedestal_post",
    )
    nacelle.visual(
        Box((0.210, 0.100, 0.090)),
        origin=Origin(xyz=(0.0, 0.0, -0.425)),
        material=stand_black,
        name="saddle_block",
    )

    fan_rotor = model.part("fan_rotor")
    fan_geometry = FanRotorGeometry(
        outer_radius=0.262,
        hub_radius=0.095,
        blade_count=18,
        thickness=0.070,
        blade_pitch_deg=34.0,
        blade_sweep_deg=31.0,
        blade=FanRotorBlade(
            shape="scimitar",
            tip_pitch_deg=15.0,
            camber=0.14,
            tip_clearance=0.006,
        ),
        hub=FanRotorHub(style="capped", bore_diameter=0.020),
        center=True,
    )
    fan_rotor.visual(
        mesh_from_geometry(fan_geometry, "fan_stage"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=fan_metal,
        name="fan_stage",
    )
    fan_rotor.visual(
        Cylinder(radius=0.022, length=0.260),
        origin=Origin(xyz=(0.090, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="shaft_stub",
    )
    spinner_profile = [
        (0.0, -0.305),
        (0.045, -0.265),
        (0.095, -0.165),
        (0.122, -0.040),
        (0.108, 0.000),
        (0.0, 0.000),
    ]
    fan_rotor.visual(
        _rotated_lathe_mesh(spinner_profile, "spinner", segments=80),
        origin=Origin(),
        material=spinner_metal,
        name="spinner",
    )

    model.articulation(
        "nacelle_to_fan",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=fan_rotor,
        origin=Origin(xyz=(-0.335, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=80.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    nacelle = object_model.get_part("nacelle")
    fan_rotor = object_model.get_part("fan_rotor")
    fan_joint = object_model.get_articulation("nacelle_to_fan")

    ctx.expect_origin_distance(
        fan_rotor,
        nacelle,
        axes="yz",
        max_dist=0.001,
        name="fan rotation axis lies on nacelle centerline",
    )
    ctx.expect_within(
        fan_rotor,
        nacelle,
        axes="yz",
        inner_elem="fan_stage",
        outer_elem="nacelle_shell",
        margin=0.0,
        name="front fan stage fits inside hollow intake",
    )
    ctx.allow_overlap(
        nacelle,
        fan_rotor,
        elem_a="front_bearing",
        elem_b="shaft_stub",
        reason=(
            "The rotating low-pressure shaft is intentionally captured inside "
            "the stationary bearing sleeve."
        ),
    )
    ctx.expect_overlap(
        nacelle,
        fan_rotor,
        axes="x",
        elem_a="front_bearing",
        elem_b="shaft_stub",
        min_overlap=0.100,
        name="rotor shaft remains inserted in the front bearing",
    )
    ctx.expect_within(
        fan_rotor,
        nacelle,
        axes="yz",
        inner_elem="shaft_stub",
        outer_elem="front_bearing",
        margin=0.0,
        name="rotor shaft is centered inside the front bearing",
    )

    rest_position = ctx.part_world_position(fan_rotor)
    with ctx.pose({fan_joint: math.pi / 2.0}):
        spun_position = ctx.part_world_position(fan_rotor)
        ctx.expect_within(
            fan_rotor,
            nacelle,
            axes="yz",
            inner_elem="fan_stage",
            outer_elem="nacelle_shell",
            margin=0.0,
            name="spinning fan remains inside nacelle",
        )
    ctx.check(
        "continuous fan spin keeps rotor on its longitudinal bearing",
        rest_position is not None
        and spun_position is not None
        and max(abs(rest_position[i] - spun_position[i]) for i in range(3)) < 1e-6,
        details=f"rest={rest_position}, spun={spun_position}",
    )

    return ctx.report()


object_model = build_object_model()
