from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _make_material(name: str, rgba: tuple[float, float, float, float]) -> Material:
    try:
        return Material(name=name, color=rgba)
    except TypeError:
        try:
            return Material(name=name, rgba=rgba)
        except TypeError:
            return Material(name, rgba)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_vise", assets=ASSETS)

    cast_iron_blue = _make_material("cast_iron_blue", (0.23, 0.30, 0.37, 1.0))
    machined_steel = _make_material("machined_steel", (0.72, 0.74, 0.76, 1.0))
    hardened_jaw = _make_material("hardened_jaw", (0.32, 0.34, 0.36, 1.0))
    oxide_black = _make_material("oxide_black", (0.09, 0.09, 0.10, 1.0))
    if hasattr(model, "materials"):
        model.materials.extend([cast_iron_blue, machined_steel, hardened_jaw, oxide_black])

    base_plate_profile = rounded_rect_profile(
        width=0.26,
        height=0.16,
        radius=0.024,
        corner_segments=10,
    )
    base_plate_mesh = mesh_from_geometry(
        ExtrudeGeometry.centered(base_plate_profile, height=0.018),
        ASSETS.mesh_path("base_plate.obj"),
    )

    base = model.part("base")
    base.visual(
        base_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=cast_iron_blue,
    )
    base.visual(
        Cylinder(radius=0.060, length=0.014),
        origin=Origin(xyz=(-0.020, 0.0, 0.027)),
        material=cast_iron_blue,
    )
    base.visual(
        Box((0.110, 0.100, 0.040)),
        origin=Origin(xyz=(-0.030, 0.0, 0.040)),
        material=cast_iron_blue,
    )
    for x, y in ((0.098, 0.055), (0.098, -0.055), (-0.112, 0.055), (-0.112, -0.055)):
        base.visual(
            Cylinder(radius=0.017, length=0.018),
            origin=Origin(xyz=(x, y, 0.009)),
            material=cast_iron_blue,
        )
    base.inertial = Inertial.from_geometry(
        Box((0.260, 0.160, 0.060)),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
    )

    body = model.part("body")
    body.visual(
        Box((0.065, 0.065, 0.016)),
        origin=Origin(xyz=(0.000, 0.0, 0.008)),
        material=cast_iron_blue,
    )
    body.visual(
        Box((0.090, 0.060, 0.040)),
        origin=Origin(xyz=(-0.005, 0.0, 0.020)),
        material=cast_iron_blue,
    )
    body.visual(
        Box((0.180, 0.070, 0.030)),
        origin=Origin(xyz=(0.040, 0.0, 0.045)),
        material=cast_iron_blue,
    )
    body.visual(
        Box((0.180, 0.012, 0.030)),
        origin=Origin(xyz=(0.040, 0.029, 0.015)),
        material=cast_iron_blue,
    )
    body.visual(
        Box((0.180, 0.012, 0.030)),
        origin=Origin(xyz=(0.040, -0.029, 0.015)),
        material=cast_iron_blue,
    )
    body.visual(
        Box((0.028, 0.078, 0.090)),
        origin=Origin(xyz=(-0.064, 0.0, 0.025)),
        material=cast_iron_blue,
    )
    body.visual(
        Box((0.065, 0.080, 0.015)),
        origin=Origin(xyz=(-0.026, 0.0, 0.0675)),
        material=machined_steel,
    )
    body.visual(
        Box((0.006, 0.068, 0.038)),
        origin=Origin(xyz=(-0.047, 0.0, 0.022)),
        material=hardened_jaw,
    )
    body.visual(
        Cylinder(radius=0.026, length=0.020),
        origin=Origin(
            xyz=(0.128, 0.0, -0.005),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=machined_steel,
    )
    body.visual(
        Box((0.072, 0.012, 0.018)),
        origin=Origin(
            xyz=(0.095, 0.031, 0.004),
            rpy=(0.0, -0.58, 0.0),
        ),
        material=cast_iron_blue,
    )
    body.visual(
        Box((0.072, 0.012, 0.018)),
        origin=Origin(
            xyz=(0.095, -0.031, 0.004),
            rpy=(0.0, -0.58, 0.0),
        ),
        material=cast_iron_blue,
    )
    body.visual(
        Box((0.078, 0.012, 0.036)),
        origin=Origin(
            xyz=(-0.020, 0.039, 0.013),
            rpy=(0.0, 0.58, 0.0),
        ),
        material=cast_iron_blue,
    )
    body.visual(
        Box((0.078, 0.012, 0.036)),
        origin=Origin(
            xyz=(-0.020, -0.039, 0.013),
            rpy=(0.0, 0.58, 0.0),
        ),
        material=cast_iron_blue,
    )
    body.inertial = Inertial.from_geometry(
        Box((0.220, 0.090, 0.110)),
        mass=9.0,
        origin=Origin(xyz=(0.015, 0.0, 0.035)),
    )

    sliding_jaw = model.part("sliding_jaw")
    sliding_jaw.visual(
        Box((0.058, 0.010, 0.060)),
        origin=Origin(xyz=(-0.014, 0.041, 0.018)),
        material=cast_iron_blue,
    )
    sliding_jaw.visual(
        Box((0.058, 0.010, 0.060)),
        origin=Origin(xyz=(-0.014, -0.041, 0.018)),
        material=cast_iron_blue,
    )
    sliding_jaw.visual(
        Box((0.058, 0.094, 0.012)),
        origin=Origin(xyz=(-0.014, 0.0, 0.054)),
        material=cast_iron_blue,
    )
    sliding_jaw.visual(
        Box((0.030, 0.082, 0.090)),
        origin=Origin(xyz=(0.025, 0.0, 0.013)),
        material=cast_iron_blue,
    )
    sliding_jaw.visual(
        Box((0.036, 0.044, 0.032)),
        origin=Origin(xyz=(0.018, 0.0, -0.016)),
        material=cast_iron_blue,
    )
    sliding_jaw.visual(
        Box((0.024, 0.070, 0.012)),
        origin=Origin(xyz=(0.018, 0.0, 0.059)),
        material=machined_steel,
    )
    sliding_jaw.visual(
        Box((0.006, 0.068, 0.038)),
        origin=Origin(xyz=(0.013, 0.0, 0.010)),
        material=hardened_jaw,
    )
    sliding_jaw.inertial = Inertial.from_geometry(
        Box((0.090, 0.090, 0.100)),
        mass=5.0,
        origin=Origin(xyz=(0.010, 0.0, 0.018)),
    )

    screw = model.part("screw")
    screw.visual(
        Cylinder(radius=0.0105, length=0.235),
        origin=Origin(
            xyz=(-0.1175, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=machined_steel,
    )
    for i in range(13):
        x = -0.200 + i * 0.015
        screw.visual(
            Cylinder(radius=0.0120, length=0.003),
            origin=Origin(
                xyz=(x, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=machined_steel,
        )
    screw.visual(
        Cylinder(radius=0.018, length=0.004),
        origin=Origin(
            xyz=(-0.004, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=machined_steel,
    )
    screw.visual(
        Cylinder(radius=0.026, length=0.018),
        origin=Origin(
            xyz=(0.009, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=machined_steel,
    )
    screw.visual(
        Cylinder(radius=0.020, length=0.026),
        origin=Origin(
            xyz=(0.027, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=oxide_black,
    )
    screw.visual(
        Cylinder(radius=0.0045, length=0.170),
        origin=Origin(
            xyz=(0.027, 0.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=oxide_black,
    )
    screw.visual(
        Sphere(radius=0.010),
        origin=Origin(xyz=(0.027, 0.085, 0.0)),
        material=oxide_black,
    )
    screw.visual(
        Sphere(radius=0.010),
        origin=Origin(xyz=(0.027, -0.085, 0.0)),
        material=oxide_black,
    )
    screw.inertial = Inertial.from_geometry(
        Cylinder(radius=0.030, length=0.300),
        mass=1.8,
        origin=Origin(
            xyz=(-0.090, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
    )

    model.articulation(
        "base_to_body",
        ArticulationType.FIXED,
        parent="base",
        child="body",
        origin=Origin(xyz=(-0.040, 0.0, 0.060)),
    )
    model.articulation(
        "jaw_slide",
        ArticulationType.PRISMATIC,
        parent="body",
        child="sliding_jaw",
        origin=Origin(xyz=(0.138, 0.0, 0.030)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6000.0,
            velocity=0.15,
            lower=0.0,
            upper=0.110,
        ),
    )
    model.articulation(
        "handle_spin",
        ArticulationType.CONTINUOUS,
        parent="sliding_jaw",
        child="screw",
        origin=Origin(xyz=(0.018, 0.0, -0.016)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=8.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    def _aabb_span(aabb: object, axis: str) -> float:
        idx = "xyz".index(axis)
        lo = getattr(aabb, f"min_{axis}", None)
        hi = getattr(aabb, f"max_{axis}", None)
        if lo is not None and hi is not None:
            return float(hi) - float(lo)
        for lo_name, hi_name in (
            ("minimum", "maximum"),
            ("min_corner", "max_corner"),
            ("mins", "maxs"),
            ("lower", "upper"),
            ("min", "max"),
        ):
            lo_val = getattr(aabb, lo_name, None)
            hi_val = getattr(aabb, hi_name, None)
            if lo_val is not None and hi_val is not None:
                return float(hi_val[idx]) - float(lo_val[idx])
        if isinstance(aabb, (tuple, list)) and len(aabb) == 2:
            return float(aabb[1][idx]) - float(aabb[0][idx])
        raise AssertionError(f"Unsupported AABB object: {aabb!r}")

    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")

    ctx.allow_overlap(
        "body",
        "screw",
        reason="lead screw passes through the body thrust collar and internal nut cavity",
    )
    ctx.allow_overlap(
        "sliding_jaw",
        "screw",
        reason="lead screw is retained inside the movable jaw nut housing",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=192,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
    )

    ctx.expect_aabb_overlap("body", "base", axes="xy", min_overlap=0.050)
    ctx.expect_aabb_overlap("sliding_jaw", "body", axes="xy", min_overlap=0.030)
    ctx.expect_aabb_overlap("screw", "sliding_jaw", axes="xy", min_overlap=0.025)
    ctx.expect_joint_motion_axis(
        "jaw_slide",
        "sliding_jaw",
        world_axis="x",
        direction="positive",
        min_delta=0.080,
    )

    closed_jaw = tuple(ctx.part_world_position("sliding_jaw"))
    closed_screw = tuple(ctx.part_world_position("screw"))
    with ctx.pose(jaw_slide=0.100):
        open_jaw = tuple(ctx.part_world_position("sliding_jaw"))
        open_screw = tuple(ctx.part_world_position("screw"))
        ctx.expect_aabb_overlap("screw", "body", axes="xy", min_overlap=0.015)
    jaw_dx = open_jaw[0] - closed_jaw[0]
    screw_dx = open_screw[0] - closed_screw[0]
    if jaw_dx < 0.095:
        raise AssertionError("Movable jaw should open by roughly 100 mm along +X.")
    if abs(open_jaw[1] - closed_jaw[1]) > 1e-6 or abs(open_jaw[2] - closed_jaw[2]) > 1e-6:
        raise AssertionError("Movable jaw should remain laterally aligned while sliding.")
    if abs(screw_dx - jaw_dx) > 1e-4:
        raise AssertionError("Lead screw head should translate with the movable jaw.")

    flat_handle = ctx.part_world_aabb("screw")
    if _aabb_span(flat_handle, "y") < 0.150:
        raise AssertionError("Handle should read as a broad crossbar in the default pose.")
    if _aabb_span(flat_handle, "z") > 0.090:
        raise AssertionError("Horizontal handle pose should stay relatively low in Z.")
    with ctx.pose(handle_spin=math.pi / 2.0):
        vertical_handle = ctx.part_world_aabb("screw")
        ctx.expect_aabb_overlap("screw", "sliding_jaw", axes="xy", min_overlap=0.020)
    if _aabb_span(vertical_handle, "z") < 0.150:
        raise AssertionError("Quarter-turn handle pose should stand tall in Z.")
    if _aabb_span(vertical_handle, "y") > 0.090:
        raise AssertionError("Quarter-turn handle pose should no longer read as wide in Y.")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
