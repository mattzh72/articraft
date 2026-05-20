from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _annular_cylinder(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    """A centered cylindrical ring with a clear middle opening."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
        .translate((0.0, 0.0, -height / 2.0))
    )


def _upper_hemisphere_shell(radius: float, thickness: float) -> cq.Workplane:
    """A thin hollow dome whose equator lies on z=0 and whose crown rises in +Z."""
    outer = cq.Workplane("XY").sphere(radius)
    inner = cq.Workplane("XY").sphere(radius - thickness)
    upper_clip = cq.Workplane("XY").box(
        radius * 2.4,
        radius * 2.4,
        radius * 1.25,
    ).translate((0.0, 0.0, radius * 0.625))
    return outer.cut(inner).intersect(upper_clip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_column_cctv")

    warm_white = model.material("warm_white", rgba=(0.90, 0.88, 0.82, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.48, 0.50, 0.50, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.03, 0.035, 0.04, 1.0))
    smoked_dome = model.material("smoked_polycarbonate", rgba=(0.07, 0.09, 0.10, 0.38))
    lens_glass = model.material("blue_black_glass", rgba=(0.02, 0.03, 0.055, 1.0))

    pedestal = model.part("pedestal")

    # Stepped round pedestal base with a visible decorative dark inlay.
    pedestal.visual(
        Cylinder(radius=0.160, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=warm_white,
        name="base_foot",
    )
    pedestal.visual(
        Cylinder(radius=0.137, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=warm_white,
        name="base_step",
    )
    pedestal.visual(
        Cylinder(radius=0.103, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        material=warm_white,
        name="base_plinth",
    )
    pedestal.visual(
        mesh_from_cadquery(_annular_cylinder(0.126, 0.108, 0.003), "base_inlay"),
        origin=Origin(xyz=(0.0, 0.0, 0.039)),
        material=dark_rubber,
        name="base_inlay",
    )

    # Short column and a compact bracket carrying the dome housing.
    pedestal.visual(
        Cylinder(radius=0.028, length=0.230),
        origin=Origin(xyz=(0.0, 0.0, 0.164)),
        material=satin_metal,
        name="column_post",
    )
    pedestal.visual(
        Cylinder(radius=0.043, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.287)),
        material=warm_white,
        name="post_collar",
    )
    pedestal.visual(
        Box((0.155, 0.036, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.301)),
        material=warm_white,
        name="bracket_crossbar",
    )
    pedestal.visual(
        Box((0.026, 0.054, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.295)),
        material=warm_white,
        name="bracket_web",
    )

    pedestal.visual(
        mesh_from_cadquery(_annular_cylinder(0.120, 0.040, 0.020), "housing_collar"),
        origin=Origin(xyz=(0.0, 0.0, 0.315)),
        material=warm_white,
        name="housing_collar",
    )
    pedestal.visual(
        mesh_from_cadquery(_annular_cylinder(0.062, 0.047, 0.008), "outer_bearing"),
        origin=Origin(xyz=(0.0, 0.0, 0.326)),
        material=satin_metal,
        name="outer_bearing",
    )
    pedestal.visual(
        mesh_from_cadquery(_upper_hemisphere_shell(0.105, 0.004), "dome_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.320)),
        material=smoked_dome,
        name="dome_shell",
    )

    pan_carriage = model.part("pan_carriage")
    pan_carriage.visual(
        Cylinder(radius=0.041, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
        material=satin_metal,
        name="pan_rotor",
    )
    pan_carriage.visual(
        Cylinder(radius=0.016, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=dark_rubber,
        name="bearing_spindle",
    )
    pan_carriage.visual(
        Box((0.112, 0.024, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=satin_metal,
        name="yoke_bridge",
    )
    for idx, x in enumerate((-0.048, 0.048)):
        pan_carriage.visual(
            Box((0.014, 0.026, 0.070)),
            origin=Origin(xyz=(x, 0.0, 0.039)),
            material=satin_metal,
            name=f"yoke_arm_{idx}",
        )
        pan_carriage.visual(
            Cylinder(radius=0.012, length=0.004),
            origin=Origin(xyz=(x * 0.86, 0.0, 0.047), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_rubber,
            name=f"tilt_bushing_{idx}",
        )

    camera_module = model.part("camera_module")
    camera_module.visual(
        Sphere(radius=0.029),
        origin=Origin(xyz=(0.0, -0.008, -0.002)),
        material=dark_rubber,
        name="camera_body",
    )
    camera_module.visual(
        Cylinder(radius=0.016, length=0.050),
        origin=Origin(xyz=(0.0, -0.047, -0.002), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_rubber,
        name="lens_barrel",
    )
    camera_module.visual(
        Cylinder(radius=0.013, length=0.004),
        origin=Origin(xyz=(0.0, -0.074, -0.002), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=lens_glass,
        name="front_glass",
    )
    camera_module.visual(
        Cylinder(radius=0.006, length=0.082),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="tilt_axle",
    )

    model.articulation(
        "pan_bearing",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=pan_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.326)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=5.0),
    )
    model.articulation(
        "tilt_axis",
        ArticulationType.REVOLUTE,
        parent=pan_carriage,
        child=camera_module,
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.7, velocity=2.0, lower=-0.75, upper=0.75),
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

    pedestal = object_model.get_part("pedestal")
    pan_carriage = object_model.get_part("pan_carriage")
    camera_module = object_model.get_part("camera_module")
    pan = object_model.get_articulation("pan_bearing")
    tilt = object_model.get_articulation("tilt_axis")

    ctx.allow_overlap(
        pedestal,
        pan_carriage,
        elem_a="housing_collar",
        elem_b="pan_rotor",
        reason="The pan rotor is intentionally captured in the collar bearing with a tiny hidden radial interference.",
    )

    ctx.check(
        "continuous pan bearing",
        pan.articulation_type == ArticulationType.CONTINUOUS,
        details=f"pan type is {pan.articulation_type}",
    )
    ctx.check(
        "bounded tilt joint",
        tilt.articulation_type == ArticulationType.REVOLUTE
        and tilt.motion_limits is not None
        and tilt.motion_limits.lower is not None
        and tilt.motion_limits.upper is not None
        and tilt.motion_limits.lower < -0.5
        and tilt.motion_limits.upper > 0.5,
        details=f"tilt type={tilt.articulation_type}, limits={tilt.motion_limits}",
    )
    ctx.expect_within(
        pan_carriage,
        pedestal,
        axes="xy",
        inner_elem="pan_rotor",
        outer_elem="dome_shell",
        margin=0.0,
        name="pan rotor sits under the dome footprint",
    )
    ctx.expect_overlap(
        pan_carriage,
        pedestal,
        axes="z",
        elem_a="pan_rotor",
        elem_b="housing_collar",
        min_overlap=0.003,
        name="pan rotor is retained by the collar bearing",
    )
    ctx.expect_within(
        camera_module,
        pedestal,
        axes="xy",
        inner_elem="lens_barrel",
        outer_elem="dome_shell",
        margin=0.0,
        name="tilting lens stays inside the dome footprint",
    )

    with ctx.pose({pan: math.pi / 2.0}):
        ctx.expect_within(
            camera_module,
            pedestal,
            axes="xy",
            inner_elem="lens_barrel",
            outer_elem="dome_shell",
            margin=0.0,
            name="panned lens remains within the dome footprint",
        )

    lowered_aabb = None
    raised_aabb = None
    with ctx.pose({tilt: 0.65}):
        lowered_aabb = ctx.part_element_world_aabb(camera_module, elem="front_glass")
    with ctx.pose({tilt: -0.65}):
        raised_aabb = ctx.part_element_world_aabb(camera_module, elem="front_glass")
    lowered_center_z = (lowered_aabb[0][2] + lowered_aabb[1][2]) / 2.0 if lowered_aabb else None
    raised_center_z = (raised_aabb[0][2] + raised_aabb[1][2]) / 2.0 if raised_aabb else None
    ctx.check(
        "tilt changes lens elevation",
        lowered_center_z is not None
        and raised_center_z is not None
        and lowered_center_z < raised_center_z - 0.04,
        details=f"lowered_z={lowered_center_z}, raised_z={raised_center_z}",
    )

    return ctx.report()


object_model = build_object_model()
