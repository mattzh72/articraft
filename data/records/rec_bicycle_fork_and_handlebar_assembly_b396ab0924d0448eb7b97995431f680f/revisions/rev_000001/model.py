from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_profile,
    superellipse_side_loft,
    tube_from_spline_points,
)


STEER_Z = 0.96


def _lz(world_z: float) -> float:
    """World Z to the steering assembly's local Z frame."""
    return world_z - STEER_Z


def _elliptic_loop(cx: float, cy: float, z: float, width: float, depth: float):
    return [
        (cx + x, cy + y, _lz(z))
        for x, y in superellipse_profile(width, depth, exponent=2.7, segments=40)
    ]


def _blade_mesh(y: float) -> LoftGeometry:
    # Straight gravel fork blade: broad fore-aft aero section, thin laterally,
    # tapering from a chunky crown socket to a small dropout end.
    return LoftGeometry(
        [
            _elliptic_loop(0.035, y, 0.060, 0.022, 0.013),
            _elliptic_loop(0.035, y, 0.300, 0.029, 0.015),
            _elliptic_loop(0.034, y, 0.540, 0.037, 0.017),
            _elliptic_loop(0.032, y, 0.675, 0.049, 0.023),
        ],
        cap=True,
        closed=True,
    )


def _lathe_tube(outer_radius: float, inner_radius: float, length: float, *, segments: int = 56):
    half = 0.5 * length
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, -half), (outer_radius, half)],
        [(inner_radius, -half), (inner_radius, half)],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )


def _solid_frustum(z0: float, z1: float, r0: float, r1: float):
    return LatheGeometry(
        [(0.0, _lz(z0)), (r0, _lz(z0)), (r1, _lz(z1)), (0.0, _lz(z1))],
        segments=72,
        closed=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gravel_bike_carbon_fork_cockpit")

    carbon = model.material("matte_carbon", rgba=(0.015, 0.017, 0.018, 1.0))
    carbon_sheen = model.material("carbon_edge_sheen", rgba=(0.030, 0.034, 0.038, 1.0))
    anodized = model.material("anodized_black_stem", rgba=(0.005, 0.006, 0.007, 1.0))
    headset_metal = model.material("dark_headset_metal", rgba=(0.11, 0.115, 0.12, 1.0))
    bolt_steel = model.material("brushed_steel_bolts", rgba=(0.55, 0.55, 0.52, 1.0))

    headset = model.part("headset")
    headset_shell = LatheGeometry.from_shell_profiles(
        [
            (0.034, 0.775),
            (0.045, 0.795),
            (0.045, 0.835),
            (0.033, 0.860),
            (0.033, 1.060),
            (0.043, 1.090),
            (0.043, 1.120),
            (0.034, 1.135),
        ],
        [(0.024, 0.775), (0.024, 1.135)],
        segments=72,
        start_cap="round",
        end_cap="round",
        lip_samples=8,
    )
    headset.visual(
        mesh_from_geometry(headset_shell, "headset_shell"),
        material=headset_metal,
        name="headset_shell",
    )

    fork = model.part("fork")
    steerer = _solid_frustum(0.675, 1.365, 0.022, 0.0145)
    fork.visual(mesh_from_geometry(steerer, "steerer"), material=carbon_sheen, name="steerer")

    crown_sections = [
        (-0.108, _lz(0.615), _lz(0.705), 0.073),
        (-0.072, _lz(0.625), _lz(0.728), 0.105),
        (0.000, _lz(0.642), _lz(0.758), 0.125),
        (0.072, _lz(0.625), _lz(0.728), 0.105),
        (0.108, _lz(0.615), _lz(0.705), 0.073),
    ]
    crown = superellipse_side_loft(crown_sections, exponents=2.9, segments=60, cap=True)
    fork.visual(mesh_from_geometry(crown, "bladed_crown"), material=carbon, name="bladed_crown")

    fork.visual(mesh_from_geometry(_blade_mesh(-0.075), "blade_0"), material=carbon, name="blade_0")
    fork.visual(mesh_from_geometry(_blade_mesh(0.075), "blade_1"), material=carbon, name="blade_1")
    # Small bonded metal dropout pads make the blade bottoms read as a real fork.
    for idx, y in enumerate((-0.075, 0.075)):
        fork.visual(
            Box((0.038, 0.018, 0.036)),
            origin=Origin(xyz=(0.039, y, _lz(0.045))),
            material=carbon_sheen,
            name=f"dropout_{idx}",
        )

    stem = model.part("stem")
    rear_collar = _lathe_tube(0.035, 0.0165, 0.090)
    rear_collar.translate(0.0, 0.0, _lz(1.185))
    stem.visual(
        mesh_from_geometry(rear_collar, "rear_collar"),
        material=anodized,
        name="rear_collar",
    )
    stem.visual(
        Box((0.118, 0.032, 0.030)),
        origin=Origin(xyz=(0.092, 0.0, _lz(1.228)), rpy=(0.0, -0.20, 0.0)),
        material=anodized,
        name="stem_body",
    )
    front_collar = _lathe_tube(0.038, 0.0115, 0.076)
    front_collar.rotate_x(math.pi / 2.0)
    front_collar.translate(0.165, 0.0, _lz(1.275))
    stem.visual(
        mesh_from_geometry(front_collar, "front_collar"),
        material=anodized,
        name="front_collar",
    )
    stem.visual(
        Box((0.016, 0.102, 0.082)),
        origin=Origin(xyz=(0.205, 0.0, _lz(1.275))),
        material=anodized,
        name="faceplate",
    )
    # Four real faceplate bolts, seated in the ahead-stem clamp face.
    for yi, y in enumerate((-0.036, 0.036)):
        for zi, z in enumerate((1.248, 1.302)):
            stem.visual(
                Cylinder(radius=0.0065, length=0.0075),
                origin=Origin(xyz=(0.215, y, _lz(z)), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=bolt_steel,
                name=f"bolt_{yi}_{zi}",
            )
    stem.visual(
        Box((0.006, 0.006, 0.068)),
        origin=Origin(xyz=(0.214, 0.0, _lz(1.275))),
        material=headset_metal,
        name="pinch_slot",
    )

    handlebar = model.part("handlebar")
    bar_points = [
        (0.335, -0.340, _lz(1.030)),
        (0.326, -0.328, _lz(1.075)),
        (0.300, -0.300, _lz(1.150)),
        (0.225, -0.265, _lz(1.245)),
        (0.165, -0.155, _lz(1.275)),
        (0.165, 0.000, _lz(1.275)),
        (0.165, 0.155, _lz(1.275)),
        (0.225, 0.265, _lz(1.245)),
        (0.300, 0.300, _lz(1.150)),
        (0.326, 0.328, _lz(1.075)),
        (0.335, 0.340, _lz(1.030)),
    ]
    bar_tube = tube_from_spline_points(
        bar_points,
        radius=0.012,
        samples_per_segment=12,
        radial_segments=24,
        cap_ends=True,
        up_hint=(0.0, 0.0, 1.0),
    )
    handlebar.visual(
        mesh_from_geometry(bar_tube, "bar_tube"),
        material=carbon_sheen,
        name="bar_tube",
    )

    steer = model.articulation(
        "steering",
        ArticulationType.REVOLUTE,
        parent=headset,
        child=fork,
        origin=Origin(xyz=(0.0, 0.0, STEER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=3.5, lower=-0.78, upper=0.78),
    )
    model.articulation(
        "fork_to_stem",
        ArticulationType.FIXED,
        parent=fork,
        child=stem,
        origin=Origin(),
    )
    model.articulation(
        "stem_to_handlebar",
        ArticulationType.FIXED,
        parent=stem,
        child=handlebar,
        origin=Origin(),
    )
    steer.meta["description"] = "Threadless steerer rotates inside the headset; stem and flared bar follow the fork."
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fork = object_model.get_part("fork")
    stem = object_model.get_part("stem")
    handlebar = object_model.get_part("handlebar")
    headset = object_model.get_part("headset")
    steering = object_model.get_articulation("steering")

    ctx.expect_within(
        fork,
        headset,
        axes="xy",
        inner_elem="steerer",
        outer_elem="headset_shell",
        margin=0.004,
        name="tapered steerer runs through headset bore",
    )
    ctx.expect_overlap(
        fork,
        headset,
        axes="z",
        elem_a="steerer",
        elem_b="headset_shell",
        min_overlap=0.30,
        name="headset surrounds a long steerer section",
    )
    ctx.allow_overlap(
        stem,
        fork,
        elem_a="rear_collar",
        elem_b="steerer",
        reason="The threadless steerer is intentionally shown captured by the stem's hollow clamp collar.",
    )
    ctx.expect_overlap(
        stem,
        fork,
        axes="z",
        elem_a="rear_collar",
        elem_b="steerer",
        min_overlap=0.07,
        name="ahead stem collar clamps the threadless steerer",
    )
    ctx.allow_overlap(
        stem,
        handlebar,
        elem_a="front_collar",
        elem_b="bar_tube",
        reason="The handlebar tube is intentionally shown compressed inside the stem's center clamp collar.",
    )
    ctx.expect_overlap(
        stem,
        handlebar,
        axes="y",
        elem_a="front_collar",
        elem_b="bar_tube",
        min_overlap=0.06,
        name="center stem collar spans the handlebar clamp zone",
    )

    def _elem_center(part, elem):
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    with ctx.pose({steering: 0.0}):
        center_rest = _elem_center(handlebar, "bar_tube")
    with ctx.pose({steering: 0.65}):
        center_turned = _elem_center(handlebar, "bar_tube")

    ctx.check(
        "handlebar follows steering rotation",
        center_rest is not None
        and center_turned is not None
        and abs(center_turned[1] - center_rest[1]) > 0.08
        and abs(center_turned[2] - center_rest[2]) < 0.02,
        details=f"rest={center_rest}, turned={center_turned}",
    )

    return ctx.report()


object_model = build_object_model()
