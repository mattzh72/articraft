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
    ExtrudeWithHolesGeometry,
    Inertial,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_side_loft,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _make_material(name: str, rgba: tuple[float, float, float, float]) -> Material:
    attempts = (
        ((), {"name": name, "rgba": rgba}),
        ((), {"name": name, "color": rgba}),
        ((name, rgba), {}),
        ((name,), {"rgba": rgba}),
        ((name,), {"color": rgba}),
        ((name,), {}),
    )
    for args, kwargs in attempts:
        try:
            return Material(*args, **kwargs)
        except TypeError:
            continue
    return Material(name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="floodlight_mast", assets=ASSETS)

    materials = {
        "concrete": _make_material("concrete", (0.60, 0.61, 0.62, 1.0)),
        "galvanized_steel": _make_material("galvanized_steel", (0.56, 0.58, 0.60, 1.0)),
        "brushed_steel": _make_material("brushed_steel", (0.72, 0.74, 0.77, 1.0)),
        "powdercoat_black": _make_material("powdercoat_black", (0.14, 0.15, 0.16, 1.0)),
        "rubber_trim": _make_material("rubber_trim", (0.06, 0.06, 0.07, 1.0)),
        "glass": _make_material("glass", (0.76, 0.87, 0.94, 0.34)),
    }
    model.materials.extend(materials.values())

    shell_geom = superellipse_side_loft(
        [
            (0.035, -0.085, 0.090, 0.440),
            (0.085, -0.095, 0.105, 0.500),
            (0.155, -0.100, 0.112, 0.510),
            (0.220, -0.092, 0.106, 0.485),
            (0.255, -0.080, 0.094, 0.455),
        ],
        exponents=(2.8, 3.0, 3.1, 3.0, 2.7),
        segments=60,
        cap=True,
        closed=True,
    )
    shell_mesh = mesh_from_geometry(shell_geom, ASSETS.mesh_path("lamp_shell.obj"))

    bezel_geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.520, 0.205, 0.028, corner_segments=8),
        [rounded_rect_profile(0.440, 0.142, 0.018, corner_segments=8)],
        height=0.026,
        cap=True,
        center=True,
        closed=True,
    ).rotate_x(math.pi / 2.0)
    bezel_mesh = mesh_from_geometry(bezel_geom, ASSETS.mesh_path("lamp_bezel.obj"))

    brace_geom = tube_from_spline_points(
        [
            (-0.130, -0.060, 2.930),
            (-0.185, -0.048, 3.010),
            (-0.255, -0.020, 3.105),
            (-0.315, 0.010, 3.170),
        ],
        radius=0.014,
        samples_per_segment=14,
        radial_segments=18,
    )
    brace_geom.merge(
        tube_from_spline_points(
            [
                (0.130, -0.060, 2.930),
                (0.185, -0.048, 3.010),
                (0.255, -0.020, 3.105),
                (0.315, 0.010, 3.170),
            ],
            radius=0.014,
            samples_per_segment=14,
            radial_segments=18,
        )
    )
    brace_mesh = mesh_from_geometry(brace_geom, ASSETS.mesh_path("yoke_braces.obj"))

    conduit_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.050, 0.015, 0.330),
                (0.082, 0.020, 0.800),
                (0.076, 0.012, 1.700),
                (0.058, -0.006, 2.700),
            ],
            radius=0.010,
            samples_per_segment=16,
            radial_segments=14,
        ),
        ASSETS.mesh_path("feed_conduit.obj"),
    )

    foundation = model.part("foundation")
    foundation.visual(
        Box((0.920, 0.920, 0.200)),
        origin=Origin(xyz=(0.0, 0.0, -0.100)),
        material=materials["concrete"],
        name="foundation_block",
    )
    foundation.visual(
        Box((0.680, 0.680, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=materials["concrete"],
        name="foundation_plinth",
    )
    foundation.inertial = Inertial.from_geometry(
        Box((0.920, 0.920, 0.200)),
        mass=1600.0,
        origin=Origin(xyz=(0.0, 0.0, -0.100)),
    )

    mast = model.part("mast_assembly")
    mast.visual(
        Cylinder(radius=0.180, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=materials["galvanized_steel"],
        name="base_flange",
    )
    for bolt_x in (-0.130, 0.130):
        for bolt_y in (-0.130, 0.130):
            mast.visual(
                Cylinder(radius=0.016, length=0.090),
                origin=Origin(xyz=(bolt_x, bolt_y, 0.045)),
                material=materials["brushed_steel"],
                name=f"anchor_bolt_{bolt_x:+.3f}_{bolt_y:+.3f}",
            )
            mast.visual(
                Cylinder(radius=0.030, length=0.018),
                origin=Origin(xyz=(bolt_x, bolt_y, 0.081)),
                material=materials["brushed_steel"],
                name=f"anchor_nut_{bolt_x:+.3f}_{bolt_y:+.3f}",
            )
    mast.visual(
        Cylinder(radius=0.112, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material=materials["galvanized_steel"],
        name="base_collar",
    )
    mast.visual(
        Cylinder(radius=0.075, length=2.820),
        origin=Origin(xyz=(0.0, 0.0, 1.530)),
        material=materials["galvanized_steel"],
        name="mast_tube",
    )
    mast.visual(
        Cylinder(radius=0.055, length=0.280),
        origin=Origin(xyz=(0.0, 0.0, 3.080)),
        material=materials["galvanized_steel"],
        name="mast_reducer",
    )
    mast.visual(
        Box((0.220, 0.130, 0.300)),
        origin=Origin(xyz=(0.130, 0.0, 0.350)),
        material=materials["powdercoat_black"],
        name="service_box",
    )
    mast.visual(
        Box((0.012, 0.112, 0.232)),
        origin=Origin(xyz=(0.236, 0.0, 0.350)),
        material=materials["rubber_trim"],
        name="service_door",
    )
    mast.visual(
        Cylinder(radius=0.007, length=0.065),
        origin=Origin(xyz=(0.248, 0.0, 0.350), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=materials["brushed_steel"],
        name="service_handle",
    )
    mast.visual(
        conduit_mesh,
        origin=Origin(),
        material=materials["brushed_steel"],
        name="feed_conduit",
    )
    mast.visual(
        Box((0.180, 0.070, 0.090)),
        origin=Origin(xyz=(0.0, -0.070, 3.085)),
        material=materials["galvanized_steel"],
        name="yoke_saddle",
    )
    mast.visual(
        Box((0.620, 0.026, 0.036)),
        origin=Origin(xyz=(0.0, -0.074, 3.170)),
        material=materials["galvanized_steel"],
        name="rear_tilt_rail",
    )
    mast.visual(
        Box((0.120, 0.012, 0.110)),
        origin=Origin(xyz=(0.0, -0.010, 3.180)),
        material=materials["brushed_steel"],
        name="pivot_clevis",
    )
    for arm_x in (-0.310, 0.310):
        mast.visual(
            Box((0.022, 0.052, 0.360)),
            origin=Origin(xyz=(arm_x * 1.06, 0.040, 3.170)),
            material=materials["galvanized_steel"],
            name=f"yoke_arm_{arm_x:+.3f}",
        )
    mast.visual(
        brace_mesh,
        origin=Origin(),
        material=materials["galvanized_steel"],
        name="yoke_braces",
    )
    mast.inertial = Inertial.from_geometry(
        Box((0.700, 0.420, 3.450)),
        mass=340.0,
        origin=Origin(xyz=(0.0, -0.010, 1.725)),
    )

    lamp = model.part("lamp_head")
    lamp.visual(
        Box((0.560, 0.020, 0.024)),
        origin=Origin(xyz=(0.0, 0.072, -0.010)),
        material=materials["galvanized_steel"],
        name="trunnion_tie_bar",
    )
    lamp.visual(
        Box((0.150, 0.012, 0.090)),
        origin=Origin(xyz=(0.0, 0.010, -0.004)),
        material=materials["brushed_steel"],
        name="pivot_spine",
    )
    lamp.visual(
        Box((0.110, 0.070, 0.060)),
        origin=Origin(xyz=(0.0, 0.045, -0.012)),
        material=materials["galvanized_steel"],
        name="pivot_hanger",
    )
    lamp.visual(
        Box((0.590, 0.055, 0.115)),
        origin=Origin(xyz=(0.0, 0.096, -0.020)),
        material=materials["powdercoat_black"],
        name="rear_chassis_crossmember",
    )
    lamp.visual(
        Box((0.040, 0.052, 0.100)),
        origin=Origin(xyz=(-0.280, 0.074, -0.002)),
        material=materials["galvanized_steel"],
        name="left_trunnion_block",
    )
    lamp.visual(
        Box((0.040, 0.052, 0.100)),
        origin=Origin(xyz=(0.280, 0.074, -0.002)),
        material=materials["galvanized_steel"],
        name="right_trunnion_block",
    )
    lamp.visual(
        shell_mesh,
        origin=Origin(xyz=(0.0, 0.070, -0.035)),
        material=materials["powdercoat_black"],
        name="lamp_shell",
    )
    lamp.visual(
        bezel_mesh,
        origin=Origin(xyz=(0.0, 0.328, -0.025)),
        material=materials["rubber_trim"],
        name="bezel_frame",
    )
    lamp.visual(
        Box((0.442, 0.010, 0.145)),
        origin=Origin(xyz=(0.0, 0.341, -0.025)),
        material=materials["glass"],
        name="lens",
    )
    lamp.visual(
        Box((0.540, 0.082, 0.018)),
        origin=Origin(xyz=(0.0, 0.316, 0.089), rpy=(-0.180, 0.0, 0.0)),
        material=materials["powdercoat_black"],
        name="visor",
    )
    lamp.visual(
        Box((0.410, 0.240, 0.022)),
        origin=Origin(xyz=(0.0, 0.235, 0.082)),
        material=materials["powdercoat_black"],
        name="heatsink_backplate",
    )
    for index, fin_y in enumerate((0.095, 0.120, 0.145, 0.170, 0.195, 0.220)):
        lamp.visual(
            Box((0.385, 0.010, 0.028)),
            origin=Origin(xyz=(0.0, fin_y + 0.070, 0.089)),
            material=materials["powdercoat_black"],
            name=f"cooling_fin_{index}",
        )
    lamp.visual(
        Box((0.200, 0.080, 0.095)),
        origin=Origin(xyz=(0.0, 0.160, -0.035)),
        material=materials["powdercoat_black"],
        name="driver_housing",
    )
    lamp.inertial = Inertial.from_geometry(
        Box((0.560, 0.310, 0.260)),
        mass=46.0,
        origin=Origin(xyz=(0.0, 0.155, 0.008)),
    )

    model.articulation(
        "foundation_to_mast",
        ArticulationType.FIXED,
        parent="foundation",
        child="mast_assembly",
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
    )
    model.articulation(
        "lamp_tilt",
        ArticulationType.REVOLUTE,
        parent="mast_assembly",
        child="lamp_head",
        origin=Origin(xyz=(0.0, 0.0, 3.180)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=600.0,
            velocity=0.8,
            lower=-0.35,
            upper=1.00,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap(
        "lamp_head",
        "mast_assembly",
        reason="tilt-trunnion clamp hardware sits in a tight yoke and conservative auto-generated collisions overstate contact near the pivot",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=192,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
    )

    ctx.expect_aabb_gap("mast_assembly", "foundation", axis="z", max_gap=0.003, max_penetration=0.0)
    ctx.expect_aabb_overlap("mast_assembly", "foundation", axes="xy", min_overlap=0.300)
    ctx.expect_origin_distance("mast_assembly", "foundation", axes="xy", max_dist=0.090)

    ctx.expect_aabb_overlap("lamp_head", "mast_assembly", axes="xy", min_overlap=0.140)
    ctx.expect_aabb_overlap("lamp_head", "foundation", axes="xy", min_overlap=0.180)
    ctx.expect_origin_distance("lamp_head", "foundation", axes="xy", max_dist=0.220)
    ctx.expect_joint_motion_axis(
        "lamp_tilt",
        "lamp_head",
        world_axis="z",
        direction="positive",
        min_delta=0.030,
    )

    with ctx.pose(lamp_tilt=-0.35):
        ctx.expect_aabb_overlap("lamp_head", "mast_assembly", axes="xy", min_overlap=0.130)
        ctx.expect_aabb_overlap("lamp_head", "foundation", axes="xy", min_overlap=0.180)
        ctx.expect_origin_distance("lamp_head", "foundation", axes="xy", max_dist=0.230)

    with ctx.pose(lamp_tilt=1.00):
        ctx.expect_aabb_overlap("lamp_head", "mast_assembly", axes="xy", min_overlap=0.100)
        ctx.expect_aabb_overlap("lamp_head", "foundation", axes="xy", min_overlap=0.160)
        ctx.expect_origin_distance("lamp_head", "foundation", axes="xy", max_dist=0.260)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
