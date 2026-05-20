from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


AXIS_HEIGHT = 0.75


def _engine_axis_origin(x: float = 0.0) -> Origin:
    """Orient local-Z revolve/rotor meshes onto the engine's world X axis."""
    return Origin(xyz=(x, 0.0, AXIS_HEIGHT), rpy=(0.0, math.pi / 2.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="turbofan_nacelle_display")

    nacelle_mat = model.material("warm_light_gray", rgba=(0.78, 0.80, 0.78, 1.0))
    dark_mat = model.material("matte_black", rgba=(0.015, 0.016, 0.018, 1.0))
    metal_mat = model.material("brushed_titanium", rgba=(0.55, 0.57, 0.58, 1.0))
    blade_mat = model.material("dark_fan_metal", rgba=(0.10, 0.105, 0.11, 1.0))
    stand_mat = model.material("satin_display_black", rgba=(0.02, 0.022, 0.024, 1.0))

    body = model.part("display_nacelle")

    # A real nacelle is a thin hollow duct, not a capped cylinder.  The profile
    # is revolved around local Z and then rotated so the engine centerline is X.
    nacelle_shell = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.365, -0.82),
            (0.435, -0.70),
            (0.445, -0.28),
            (0.415, 0.32),
            (0.335, 0.72),
            (0.300, 0.86),
        ],
        inner_profile=[
            (0.295, -0.80),
            (0.325, -0.62),
            (0.332, -0.18),
            (0.300, 0.34),
            (0.235, 0.76),
            (0.215, 0.84),
        ],
        segments=96,
        start_cap="round",
        end_cap="round",
        lip_samples=8,
    )
    body.visual(
        mesh_from_geometry(nacelle_shell, "nacelle_shell"),
        origin=_engine_axis_origin(),
        material=nacelle_mat,
        name="nacelle_shell",
    )

    # Dark inlet and exhaust ring accents, each slightly inside the duct wall.
    front_lip = LatheGeometry.from_shell_profiles(
        outer_profile=[(0.318, -0.790), (0.330, -0.710)],
        inner_profile=[(0.292, -0.790), (0.300, -0.710)],
        segments=96,
        start_cap="flat",
        end_cap="flat",
        lip_samples=4,
    )
    body.visual(
        mesh_from_geometry(front_lip, "front_lip"),
        origin=_engine_axis_origin(),
        material=dark_mat,
        name="front_lip",
    )
    rear_lip = LatheGeometry.from_shell_profiles(
        outer_profile=[(0.238, 0.710), (0.248, 0.835)],
        inner_profile=[(0.205, 0.710), (0.214, 0.835)],
        segments=96,
        start_cap="flat",
        end_cap="flat",
        lip_samples=4,
    )
    body.visual(
        mesh_from_geometry(rear_lip, "rear_lip"),
        origin=_engine_axis_origin(),
        material=dark_mat,
        name="rear_lip",
    )

    # Front bearing sleeve behind the fan: a stationary annular proxy with
    # small, intentional shaft capture.  Four vanes tie it back to the nacelle.
    bearing_sleeve = LatheGeometry.from_shell_profiles(
        outer_profile=[(0.066, -0.125), (0.066, 0.125)],
        inner_profile=[(0.038, -0.125), (0.038, 0.125)],
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=4,
    )
    body.visual(
        mesh_from_geometry(bearing_sleeve, "bearing_sleeve"),
        origin=_engine_axis_origin(-0.250),
        material=metal_mat,
        name="bearing_sleeve",
    )
    for idx, (y, z, sy, sz) in enumerate(
        [
            (0.0, AXIS_HEIGHT + 0.208, 0.030, 0.310),
            (0.0, AXIS_HEIGHT - 0.208, 0.030, 0.310),
            (0.208, AXIS_HEIGHT, 0.310, 0.030),
            (-0.208, AXIS_HEIGHT, 0.310, 0.030),
        ]
    ):
        body.visual(
            Box((0.055, sy, sz)),
            origin=Origin(xyz=(-0.250, y, z)),
            material=metal_mat,
            name=f"bearing_vane_{idx}",
        )

    # Rear exhaust plug/cone, held on-axis by four visible outlet struts.
    exhaust_cone = LatheGeometry(
        [
            (0.015, 0.245),
            (0.080, 0.160),
            (0.155, -0.030),
            (0.155, -0.195),
            (0.020, -0.245),
        ],
        segments=72,
        closed=True,
    )
    body.visual(
        mesh_from_geometry(exhaust_cone, "exhaust_cone"),
        origin=_engine_axis_origin(0.575),
        material=metal_mat,
        name="exhaust_cone",
    )
    for idx, (y, z, sy, sz) in enumerate(
        [
            (0.0, AXIS_HEIGHT + 0.230, 0.035, 0.180),
            (0.0, AXIS_HEIGHT - 0.230, 0.035, 0.180),
            (0.230, AXIS_HEIGHT, 0.180, 0.035),
            (-0.230, AXIS_HEIGHT, 0.180, 0.035),
        ]
    ):
        body.visual(
            Box((0.085, sy, sz)),
            origin=Origin(xyz=(0.505, y, z)),
            material=metal_mat,
            name=f"exhaust_strut_{idx}",
        )

    # Short museum-style display stand with a pylon that actually penetrates the
    # underside of the nacelle shell, making the static assembly one supported
    # object.
    body.visual(
        Box((0.90, 0.46, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=stand_mat,
        name="stand_base",
    )
    body.visual(
        Box((0.230, 0.135, 0.315)),
        origin=Origin(xyz=(0.0, 0.0, 0.2175)),
        material=stand_mat,
        name="stand_pylon",
    )
    body.visual(
        Box((0.340, 0.205, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.362)),
        material=stand_mat,
        name="stand_saddle",
    )

    rotor = model.part("fan_rotor")
    fan_stage = FanRotorGeometry(
        outer_radius=0.282,
        hub_radius=0.088,
        blade_count=18,
        thickness=0.075,
        blade_pitch_deg=34.0,
        blade_sweep_deg=31.0,
        blade=FanRotorBlade(
            shape="scimitar",
            tip_pitch_deg=14.0,
            camber=0.18,
            tip_clearance=0.010,
        ),
        hub=FanRotorHub(style="domed", rear_collar_height=0.030, rear_collar_radius=0.070),
        center=True,
    )
    rotor.visual(
        mesh_from_geometry(fan_stage, "fan_stage"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blade_mat,
        name="fan_stage",
    )

    spinner = LatheGeometry(
        [
            (0.000, -0.225),
            (0.035, -0.190),
            (0.075, -0.105),
            (0.092, -0.025),
            (0.090, 0.055),
            (0.000, 0.058),
        ],
        segments=72,
        closed=True,
    )
    rotor.visual(
        mesh_from_geometry(spinner, "spinner"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_mat,
        name="spinner",
    )
    rotor.visual(
        # The shaft is intentionally slightly larger than the stationary
        # bearing bore so the compiled geometry has a real captured support path.
        mesh_from_geometry(
            LatheGeometry(
                [(0.0, -0.270), (0.043, -0.270), (0.043, 0.270), (0.0, 0.270)],
                segments=48,
                closed=True,
            ),
            "rotor_shaft",
        ),
        origin=Origin(xyz=(0.180, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_mat,
        name="rotor_shaft",
    )

    model.articulation(
        "fan_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=rotor,
        origin=Origin(xyz=(-0.535, 0.0, AXIS_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=120.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("display_nacelle")
    rotor = object_model.get_part("fan_rotor")
    spin = object_model.get_articulation("fan_spin")

    ctx.allow_overlap(
        body,
        rotor,
        elem_a="bearing_sleeve",
        elem_b="rotor_shaft",
        reason="The fan shaft is intentionally represented as captured inside the stationary bearing sleeve.",
    )
    ctx.expect_within(
        rotor,
        body,
        axes="yz",
        inner_elem="rotor_shaft",
        outer_elem="bearing_sleeve",
        margin=0.002,
        name="shaft is centered in bearing sleeve",
    )
    ctx.expect_overlap(
        rotor,
        body,
        axes="x",
        elem_a="rotor_shaft",
        elem_b="bearing_sleeve",
        min_overlap=0.200,
        name="shaft remains inserted through bearing",
    )
    ctx.check(
        "fan uses continuous centerline joint",
        spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (1.0, 0.0, 0.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )
    ctx.expect_within(
        rotor,
        body,
        axes="yz",
        inner_elem="fan_stage",
        outer_elem="nacelle_shell",
        margin=0.002,
        name="fan stage fits inside nacelle bore",
    )
    ctx.expect_overlap(
        rotor,
        body,
        axes="x",
        elem_a="fan_stage",
        elem_b="nacelle_shell",
        min_overlap=0.060,
        name="fan stage is set back inside intake",
    )

    rest_position = ctx.part_world_position(rotor)
    with ctx.pose({spin: math.pi / 2.0}):
        spun_position = ctx.part_world_position(rotor)
        ctx.expect_within(
            rotor,
            body,
            axes="yz",
            inner_elem="fan_stage",
            outer_elem="nacelle_shell",
            margin=0.002,
            name="spinning fan remains coaxial in duct",
        )
    ctx.check(
        "spin changes attitude without translating rotor",
        rest_position is not None
        and spun_position is not None
        and all(abs(a - b) < 1e-6 for a, b in zip(rest_position, spun_position)),
        details=f"rest={rest_position}, spun={spun_position}",
    )

    return ctx.report()


object_model = build_object_model()
