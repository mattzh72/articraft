from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _helix_points(
    *,
    center: tuple[float, float],
    radius: float,
    z_min: float,
    z_max: float,
    turns: int,
    samples_per_turn: int,
) -> list[tuple[float, float, float]]:
    cx, cy = center
    total_samples = turns * samples_per_turn
    points: list[tuple[float, float, float]] = []
    for i in range(total_samples + 1):
        t = i / total_samples
        angle = t * turns * 2.0 * math.pi
        points.append(
            (
                cx + radius * math.cos(angle),
                cy + radius * math.sin(angle),
                z_min + (z_max - z_min) * t,
            )
        )
    return points


def _aabb_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_prep_faucet")

    chrome = model.material("chrome", rgba=(0.78, 0.80, 0.82, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.35, 0.37, 0.39, 1.0))
    matte_black = model.material("matte_black", rgba=(0.12, 0.12, 0.13, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.032, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=chrome,
        name="deck_flange",
    )

    body = model.part("body")
    body.visual(
        Cylinder(radius=0.026, length=0.056),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=chrome,
        name="body_skirt",
    )
    body.visual(
        Cylinder(radius=0.017, length=0.108),
        origin=Origin(xyz=(0.0, 0.0, 0.054)),
        material=chrome,
        name="swivel_column",
    )
    body.visual(
        Cylinder(radius=0.0105, length=0.305),
        origin=Origin(xyz=(0.0, 0.0, 0.235)),
        material=chrome,
        name="riser_tube",
    )
    body.visual(
        Cylinder(radius=0.020, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=chrome,
        name="hub_cap",
    )

    arch_points = [
        (0.0, 0.0, 0.380),
        (0.008, 0.0, 0.430),
        (0.040, 0.0, 0.485),
        (0.088, 0.0, 0.520),
    ]
    arch_mesh = mesh_from_geometry(
        tube_from_spline_points(
            arch_points,
            radius=0.0105,
            samples_per_segment=18,
            radial_segments=24,
        ),
        "arch_tube",
    )
    body.visual(arch_mesh, material=chrome, name="arch_tube")

    top_feed_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.088, 0.0, 0.520),
                (0.091, 0.0, 0.510),
                (0.094, 0.0, 0.500),
            ],
            radius=0.007,
            samples_per_segment=12,
            radial_segments=18,
        ),
        "top_feed",
    )
    body.visual(top_feed_mesh, material=matte_black, name="top_feed")

    spring_mesh = mesh_from_geometry(
        tube_from_spline_points(
            _helix_points(
                center=(0.090, 0.0),
                radius=0.018,
                z_min=0.274,
                z_max=0.500,
                turns=16,
                samples_per_turn=18,
            ),
            radius=0.0024,
            samples_per_segment=2,
            radial_segments=16,
            up_hint=(1.0, 0.0, 0.0),
        ),
        "neck_spring",
    )
    body.visual(spring_mesh, material=chrome, name="neck_spring")

    body.visual(
        Cylinder(radius=0.024, length=0.018),
        origin=Origin(xyz=(0.090, 0.0, 0.509)),
        material=chrome,
        name="top_collar",
    )
    body.visual(
        Cylinder(radius=0.0055, length=0.020),
        origin=Origin(xyz=(0.090, 0.018, 0.274)),
        material=chrome,
        name="bottom_socket_0",
    )
    body.visual(
        Cylinder(radius=0.0055, length=0.020),
        origin=Origin(xyz=(0.090, -0.018, 0.274)),
        material=chrome,
        name="bottom_socket_1",
    )
    body.visual(
        Cylinder(radius=0.0055, length=0.020),
        origin=Origin(xyz=(0.108, 0.0, 0.274)),
        material=chrome,
        name="bottom_socket_2",
    )
    body.visual(
        Cylinder(radius=0.003, length=0.064),
        origin=Origin(xyz=(0.090, 0.018, 0.240)),
        material=chrome,
        name="cradle_rod_0",
    )
    body.visual(
        Cylinder(radius=0.003, length=0.064),
        origin=Origin(xyz=(0.090, -0.018, 0.240)),
        material=chrome,
        name="cradle_rod_1",
    )

    lever = model.part("lever")
    lever.visual(
        Cylinder(radius=0.0065, length=0.014),
        origin=Origin(xyz=(0.0, -0.007, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="lever_pivot",
    )
    lever_arm = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.0, -0.007, 0.0),
                (0.018, -0.004, 0.014),
                (0.038, -0.001, 0.033),
                (0.054, 0.0, 0.048),
            ],
            radius=0.0042,
            samples_per_segment=16,
            radial_segments=18,
        ),
        "lever_arm",
    )
    lever.visual(lever_arm, material=chrome, name="lever_arm")
    lever.visual(
        Cylinder(radius=0.008, length=0.026),
        origin=Origin(
            xyz=(0.056, 0.0, 0.050),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_metal,
        name="lever_pad",
    )

    wand = model.part("wand")
    hose_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.0, 0.0, 0.220),
                (0.0, 0.0, 0.165),
                (0.0, 0.0, 0.105),
                (0.002, 0.0, 0.040),
                (0.004, 0.0, -0.020),
            ],
            radius=0.0055,
            samples_per_segment=16,
            radial_segments=16,
        ),
        "wand_hose",
    )
    wand.visual(hose_mesh, material=matte_black, name="wand_hose")
    wand.visual(
        Cylinder(radius=0.0095, length=0.028),
        origin=Origin(xyz=(0.004, 0.0, -0.024)),
        material=dark_metal,
        name="hose_ferrule",
    )
    wand.visual(
        Cylinder(radius=0.0150, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=chrome,
        name="dock_collar",
    )
    wand.visual(
        Cylinder(radius=0.0125, length=0.125),
        origin=Origin(xyz=(0.008, 0.0, -0.088)),
        material=chrome,
        name="wand_barrel",
    )
    wand.visual(
        Cylinder(radius=0.0145, length=0.074),
        origin=Origin(xyz=(0.008, 0.0, -0.112)),
        material=matte_black,
        name="wand_grip",
    )
    wand.visual(
        Cylinder(radius=0.0155, length=0.030),
        origin=Origin(xyz=(0.010, 0.0, -0.162)),
        material=chrome,
        name="wand_head",
    )

    model.articulation(
        "base_to_body",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=6.0),
    )

    model.articulation(
        "body_to_lever",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lever,
        origin=Origin(xyz=(0.0, -0.023, 0.060)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=2.0,
            lower=-0.65,
            upper=0.55,
        ),
    )

    model.articulation(
        "body_to_wand",
        ArticulationType.PRISMATIC,
        parent=body,
        child=wand,
        origin=Origin(xyz=(0.090, 0.0, 0.255)),
        axis=(0.20, 0.0, -0.98),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.30,
            lower=0.0,
            upper=0.180,
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
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    base = object_model.get_part("base")
    body = object_model.get_part("body")
    lever = object_model.get_part("lever")
    wand = object_model.get_part("wand")

    spin = object_model.get_articulation("base_to_body")
    lever_joint = object_model.get_articulation("body_to_lever")
    wand_slide = object_model.get_articulation("body_to_wand")

    ctx.expect_gap(
        body,
        base,
        axis="z",
        positive_elem="body_skirt",
        negative_elem="deck_flange",
        max_gap=0.004,
        max_penetration=1e-6,
        name="body skirt seats cleanly on the deck flange",
    )

    spring_rest = _aabb_center(ctx.part_element_world_aabb(body, elem="neck_spring"))
    with ctx.pose({spin: math.pi / 2.0}):
        spring_turned = _aabb_center(ctx.part_element_world_aabb(body, elem="neck_spring"))
    ctx.check(
        "body can swivel around the base axis",
        spring_rest is not None
        and spring_turned is not None
        and spring_rest[0] > 0.07
        and abs(spring_rest[1]) < 0.03
        and spring_turned[1] > 0.07
        and abs(spring_turned[0]) < 0.03,
        details=f"rest={spring_rest}, turned={spring_turned}",
    )

    lever_rest = _aabb_center(ctx.part_element_world_aabb(lever, elem="lever_pad"))
    lever_upper = lever_joint.motion_limits.upper if lever_joint.motion_limits is not None else 0.45
    with ctx.pose({lever_joint: lever_upper}):
        lever_lifted = _aabb_center(ctx.part_element_world_aabb(lever, elem="lever_pad"))
    ctx.check(
        "side lever lifts on its short pivot",
        lever_rest is not None
        and lever_lifted is not None
        and lever_lifted[2] > lever_rest[2] + 0.015,
        details=f"rest={lever_rest}, lifted={lever_lifted}",
    )

    wand_rest = ctx.part_world_position(wand)
    wand_upper = wand_slide.motion_limits.upper if wand_slide.motion_limits is not None else 0.18
    with ctx.pose({wand_slide: wand_upper}):
        wand_extended = ctx.part_world_position(wand)
    ctx.check(
        "spray wand pulls down and out from the spring support",
        wand_rest is not None
        and wand_extended is not None
        and wand_extended[0] > wand_rest[0] + 0.025
        and wand_extended[2] < wand_rest[2] - 0.14,
        details=f"rest={wand_rest}, extended={wand_extended}",
    )

    return ctx.report()


object_model = build_object_model()
