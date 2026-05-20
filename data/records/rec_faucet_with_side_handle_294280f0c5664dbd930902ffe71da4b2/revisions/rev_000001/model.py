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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mixing_faucet")

    polished_chrome = model.material("polished_chrome", rgba=(0.83, 0.85, 0.88, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.67, 0.69, 0.72, 1.0))

    body = model.part("body")
    body.visual(
        Cylinder(radius=0.036, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=brushed_steel,
        name="deck_flange",
    )
    body.visual(
        Cylinder(radius=0.027, length=0.074),
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
        material=polished_chrome,
        name="body_barrel",
    )
    body.visual(
        Cylinder(radius=0.023, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.091)),
        material=polished_chrome,
        name="top_cap",
    )
    body.visual(
        Cylinder(radius=0.015, length=0.026),
        origin=Origin(
            xyz=(0.022, 0.0, 0.064),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=polished_chrome,
        name="spout_collar",
    )
    spout_path = [
        (0.012, 0.0, 0.060),
        (0.040, 0.0, 0.095),
        (0.118, 0.0, 0.108),
        (0.178, 0.0, 0.104),
        (0.210, 0.0, 0.086),
    ]
    body.visual(
        _save_mesh(
            "faucet_spout",
            tube_from_spline_points(
                spout_path,
                radius=0.0105,
                samples_per_segment=18,
                radial_segments=20,
                cap_ends=True,
            ),
        ),
        material=polished_chrome,
        name="spout_tube",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(
            xyz=(0.208, 0.0, 0.084),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=polished_chrome,
        name="spout_aerator",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.26, 0.09, 0.12)),
        mass=2.8,
        origin=Origin(xyz=(0.10, 0.0, 0.05)),
    )

    pivot_base = model.part("pivot_base")
    pivot_base.visual(
        Cylinder(radius=0.016, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=polished_chrome,
        name="pivot_pedestal",
    )
    pivot_base.visual(
        Box((0.010, 0.006, 0.024)),
        origin=Origin(xyz=(0.0, 0.014, 0.012)),
        material=polished_chrome,
        name="left_ear",
    )
    pivot_base.visual(
        Box((0.010, 0.006, 0.024)),
        origin=Origin(xyz=(0.0, -0.014, 0.012)),
        material=polished_chrome,
        name="right_ear",
    )
    pivot_base.inertial = Inertial.from_geometry(
        Box((0.036, 0.036, 0.028)),
        mass=0.20,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.0055, length=0.022),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished_chrome,
        name="trunnion",
    )
    handle.visual(
        Sphere(radius=0.0075),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=polished_chrome,
        name="handle_hub",
    )
    handle.visual(
        _save_mesh(
            "faucet_handle_lever",
            tube_from_spline_points(
                [
                    (0.000, 0.0, 0.006),
                    (-0.014, 0.0, 0.028),
                    (-0.034, 0.0, 0.060),
                    (-0.050, 0.0, 0.084),
                ],
                radius=0.0055,
                samples_per_segment=18,
                radial_segments=18,
                cap_ends=True,
            ),
        ),
        material=polished_chrome,
        name="handle_lever",
    )
    handle.visual(
        Sphere(radius=0.014),
        origin=Origin(xyz=(-0.054, 0.0, 0.086)),
        material=polished_chrome,
        name="handle_ball",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.090, 0.030, 0.110)),
        mass=0.28,
        origin=Origin(xyz=(-0.028, 0.0, 0.050)),
    )

    model.articulation(
        "flow_swing",
        ArticulationType.REVOLUTE,
        parent=body,
        child=pivot_base,
        origin=Origin(xyz=(-0.004, 0.0, 0.098)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.5,
            lower=-0.70,
            upper=0.70,
        ),
    )
    model.articulation(
        "temperature_tilt",
        ArticulationType.REVOLUTE,
        parent=pivot_base,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=-0.55,
            upper=0.65,
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

    body = object_model.get_part("body")
    pivot_base = object_model.get_part("pivot_base")
    handle = object_model.get_part("handle")
    flow_swing = object_model.get_articulation("flow_swing")
    temperature_tilt = object_model.get_articulation("temperature_tilt")

    ctx.expect_contact(
        pivot_base,
        body,
        elem_a="pivot_pedestal",
        elem_b="top_cap",
        contact_tol=0.0005,
        name="swivel pedestal seats on top cap",
    )
    ctx.expect_contact(
        handle,
        pivot_base,
        contact_tol=0.0005,
        name="handle trunnion is supported by the yoke",
    )

    spout_aabb = ctx.part_element_world_aabb(body, elem="spout_aerator")
    barrel_aabb = ctx.part_element_world_aabb(body, elem="body_barrel")
    spout_center = _aabb_center(spout_aabb)
    barrel_center = _aabb_center(barrel_aabb)
    ctx.check(
        "spout projects forward from the round body",
        spout_center is not None
        and barrel_center is not None
        and spout_center[0] > barrel_center[0] + 0.17
        and abs(spout_center[1] - barrel_center[1]) < 0.005,
        details=f"spout_center={spout_center}, barrel_center={barrel_center}",
    )

    rest_ball_aabb = ctx.part_element_world_aabb(handle, elem="handle_ball")
    rest_ball_center = _aabb_center(rest_ball_aabb)
    with ctx.pose({flow_swing: 0.55}):
        swung_ball_center = _aabb_center(ctx.part_element_world_aabb(handle, elem="handle_ball"))
    with ctx.pose({temperature_tilt: 0.40}):
        tilted_ball_center = _aabb_center(ctx.part_element_world_aabb(handle, elem="handle_ball"))

    ctx.check(
        "flow joint swings the handle around the vertical axis",
        rest_ball_center is not None
        and swung_ball_center is not None
        and abs(swung_ball_center[1] - rest_ball_center[1]) > 0.025
        and abs(swung_ball_center[2] - rest_ball_center[2]) < 0.003,
        details=f"rest={rest_ball_center}, swung={swung_ball_center}",
    )
    ctx.check(
        "temperature joint tilts the handle about a horizontal axis",
        rest_ball_center is not None
        and tilted_ball_center is not None
        and abs(tilted_ball_center[1] - rest_ball_center[1]) < 0.003
        and abs(tilted_ball_center[0] - rest_ball_center[0]) > 0.010
        and abs(tilted_ball_center[2] - rest_ball_center[2]) > 0.008,
        details=f"rest={rest_ball_center}, tilted={tilted_ball_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
