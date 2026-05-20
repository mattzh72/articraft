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
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _engine_axis_visual() -> Origin:
    """Rotate local-Z mesh/primitives so their spin/lathe axis lies on +X."""

    return Origin(rpy=(0.0, math.pi / 2.0, 0.0))


def _axis_lathe(profile: list[tuple[float, float]], *, segments: int = 96) -> MeshGeometry:
    geom = LatheGeometry(profile, segments=segments, closed=True)
    geom.rotate_y(math.pi / 2.0)
    return geom


def _shell_lathe(
    outer: list[tuple[float, float]],
    inner: list[tuple[float, float]],
    *,
    segments: int = 128,
    start_cap: str = "round",
    end_cap: str = "flat",
) -> MeshGeometry:
    geom = LatheGeometry.from_shell_profiles(
        outer,
        inner,
        segments=segments,
        start_cap=start_cap,
        end_cap=end_cap,
        lip_samples=10,
    )
    geom.rotate_y(math.pi / 2.0)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_turbojet_module")

    brushed_steel = model.material("brushed_steel", rgba=(0.58, 0.61, 0.62, 1.0))
    dark_liner = model.material("sooty_nozzle_liner", rgba=(0.035, 0.038, 0.040, 1.0))
    warm_core = model.material("warm_titanium_core", rgba=(0.43, 0.40, 0.35, 1.0))
    strut_metal = model.material("stator_strut_metal", rgba=(0.50, 0.53, 0.55, 1.0))
    blade_metal = model.material("compressor_blade_metal", rgba=(0.72, 0.74, 0.76, 1.0))
    hub_metal = model.material("polished_spinner", rgba=(0.82, 0.83, 0.80, 1.0))

    casing = model.part("casing")

    casing_shell = _shell_lathe(
        outer=[
            (0.168, -0.395),
            (0.188, -0.382),
            (0.184, -0.356),
            (0.163, -0.325),
            (0.158, -0.055),
            (0.154, 0.225),
            (0.140, 0.330),
            (0.108, 0.450),
        ],
        inner=[
            (0.132, -0.390),
            (0.123, -0.333),
            (0.116, -0.060),
            (0.109, 0.225),
            (0.094, 0.330),
            (0.079, 0.440),
        ],
        start_cap="round",
        end_cap="flat",
    )
    casing.visual(
        mesh_from_geometry(casing_shell, "casing_shell"),
        material=brushed_steel,
        name="casing_shell",
    )

    intake_lip = TorusGeometry(radius=0.160, tube=0.026, radial_segments=24, tubular_segments=96)
    intake_lip.rotate_y(math.pi / 2.0)
    intake_lip.translate(-0.382, 0.0, 0.0)
    casing.visual(
        mesh_from_geometry(intake_lip, "intake_lip"),
        material=brushed_steel,
        name="intake_lip",
    )

    nozzle_liner = _shell_lathe(
        outer=[
            (0.102, 0.252),
            (0.096, 0.330),
            (0.079, 0.444),
        ],
        inner=[
            (0.085, 0.260),
            (0.079, 0.335),
            (0.064, 0.438),
        ],
        segments=96,
        start_cap="flat",
        end_cap="flat",
    )
    casing.visual(
        mesh_from_geometry(nozzle_liner, "nozzle_liner"),
        material=dark_liner,
        name="nozzle_liner",
    )

    core_fairing = _axis_lathe(
        [
            (0.000, -0.205),
            (0.032, -0.182),
            (0.060, -0.132),
            (0.071, -0.030),
            (0.071, 0.235),
            (0.060, 0.320),
            (0.026, 0.385),
            (0.000, 0.405),
        ],
        segments=96,
    )
    casing.visual(
        mesh_from_geometry(core_fairing, "core_body"),
        material=warm_core,
        name="core_body",
    )
    casing.visual(
        Cylinder(radius=0.020, length=0.040),
        origin=Origin(xyz=(-0.205, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_metal,
        name="front_bearing",
    )

    # Six front stator struts physically bridge the center core to the inner casing.
    strut_radius = 0.098
    for index in range(6):
        theta = index * math.tau / 6.0
        casing.visual(
            Box((0.118, 0.086, 0.012)),
            origin=Origin(
                xyz=(-0.178, strut_radius * math.cos(theta), strut_radius * math.sin(theta)),
                rpy=(theta, 0.0, 0.0),
            ),
            material=strut_metal,
            name=f"strut_{index}",
        )

    fan = model.part("fan")
    rotor = FanRotorGeometry(
        0.108,
        0.032,
        13,
        thickness=0.030,
        blade_pitch_deg=34.0,
        blade_sweep_deg=30.0,
        blade=FanRotorBlade(
            shape="scimitar",
            tip_pitch_deg=14.0,
            camber=0.18,
            tip_clearance=0.004,
        ),
        hub=FanRotorHub(
            style="spinner",
            rear_collar_height=0.010,
            rear_collar_radius=0.025,
            bore_diameter=0.007,
        ),
    )
    fan.visual(
        mesh_from_geometry(rotor, "compressor_rotor"),
        origin=_engine_axis_visual(),
        material=blade_metal,
        name="blades",
    )
    spinner_cap = _axis_lathe(
        [
            (0.000, -0.021),
            (0.019, -0.015),
            (0.030, -0.003),
            (0.025, 0.014),
            (0.000, 0.026),
        ],
        segments=72,
    )
    fan.visual(
        mesh_from_geometry(spinner_cap, "spinner_cap"),
        material=hub_metal,
        name="spinner_cap",
    )
    fan.visual(
        Cylinder(radius=0.007, length=0.100),
        origin=Origin(xyz=(0.060, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_metal,
        name="shaft",
    )

    model.articulation(
        "fan_spin",
        ArticulationType.CONTINUOUS,
        parent=casing,
        child=fan,
        origin=Origin(xyz=(-0.300, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=80.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    casing = object_model.get_part("casing")
    fan = object_model.get_part("fan")
    spin = object_model.get_articulation("fan_spin")

    ctx.allow_overlap(
        casing,
        fan,
        elem_a="front_bearing",
        elem_b="shaft",
        reason="The fan shaft is intentionally captured inside the front bearing boss.",
    )
    ctx.allow_overlap(
        casing,
        fan,
        elem_a="core_body",
        elem_b="shaft",
        reason="The rear end of the fan shaft intentionally enters the core nose bearing socket.",
    )
    ctx.expect_within(
        fan,
        casing,
        axes="yz",
        inner_elem="shaft",
        outer_elem="front_bearing",
        margin=0.0,
        name="shaft is centered inside front bearing",
    )
    ctx.expect_overlap(
        casing,
        fan,
        axes="x",
        elem_a="front_bearing",
        elem_b="shaft",
        min_overlap=0.020,
        name="shaft remains seated in bearing",
    )
    ctx.expect_overlap(
        casing,
        fan,
        axes="x",
        elem_a="core_body",
        elem_b="shaft",
        min_overlap=0.008,
        name="shaft enters core nose socket",
    )

    ctx.check(
        "fan uses continuous spin joint",
        spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={spin.articulation_type}",
    )
    ctx.check(
        "fan spins on longitudinal axis",
        tuple(round(v, 6) for v in spin.axis) == (1.0, 0.0, 0.0),
        details=f"axis={spin.axis}",
    )
    ctx.expect_within(
        fan,
        casing,
        axes="yz",
        margin=0.002,
        name="fan fits inside intake casing diameter",
    )
    ctx.expect_gap(
        fan,
        casing,
        axis="x",
        min_gap=0.030,
        name="fan sits behind the rounded intake lip",
        positive_elem="blades",
        negative_elem="intake_lip",
    )

    with ctx.pose({spin: math.pi / 2.0}):
        ctx.expect_within(
            fan,
            casing,
            axes="yz",
            margin=0.002,
            name="spinning fan remains inside casing",
        )

    return ctx.report()


object_model = build_object_model()
