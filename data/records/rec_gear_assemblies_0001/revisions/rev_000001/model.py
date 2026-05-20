from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
MESH_DIR = ASSETS.ensure_mesh_dir()


# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import Cylinder

BED_THICKNESS = 0.014
PAD_HEIGHT = 0.006
SUPPORT_HEIGHT = 0.040
SUPPORT_MOUNT_Z = BED_THICKNESS + PAD_HEIGHT
JOURNAL_INSERTION = 0.006

INPUT_X = -0.078
COMPOUND_X = 0.0
OUTPUT_X = 0.078

INPUT_STAGE_RATIO = 20.0 / 32.0
OUTPUT_STAGE_RATIO = (20.0 / 32.0) * (16.0 / 36.0)
SPIN_LIMITS = MotionLimits(effort=18.0, velocity=25.0)


def _polar_xy(radius: float, angle_rad: float) -> tuple[float, float]:
    return (radius * math.cos(angle_rad), radius * math.sin(angle_rad))


def _gear_outline_points(
    tooth_count: int,
    root_radius: float,
    tip_radius: float,
) -> list[tuple[float, float]]:
    pts: list[tuple[float, float]] = []
    step = 2.0 * math.pi / tooth_count
    for tooth_index in range(tooth_count):
        center = tooth_index * step
        pts.extend(
            [
                _polar_xy(root_radius, center - 0.50 * step),
                _polar_xy(root_radius * 0.995, center - 0.27 * step),
                _polar_xy(tip_radius, center - 0.14 * step),
                _polar_xy(tip_radius, center + 0.14 * step),
                _polar_xy(root_radius * 0.995, center + 0.27 * step),
            ]
        )
    return pts


def _spur_gear_solid(
    tooth_count: int,
    root_radius: float,
    tip_radius: float,
    thickness: float,
    hub_radius: float,
    hub_height: float,
    bore_radius: float,
    lightening_radius: float,
    lightening_hole_radius: float,
    lightening_hole_count: int,
) -> cq.Workplane:
    gear = (
        cq.Workplane("XY")
        .polyline(_gear_outline_points(tooth_count, root_radius, tip_radius))
        .close()
        .extrude(thickness)
    )
    gear = gear.union(cq.Workplane("XY").circle(hub_radius).extrude(thickness + hub_height))
    cut_height = thickness + hub_height + 0.006
    for hole_index in range(lightening_hole_count):
        angle = 2.0 * math.pi * hole_index / lightening_hole_count
        hx, hy = _polar_xy(lightening_radius, angle)
        gear = gear.cut(
            cq.Workplane("XY").center(hx, hy).circle(lightening_hole_radius).extrude(cut_height)
        )
    gear = gear.cut(cq.Workplane("XY").circle(bore_radius).extrude(cut_height))
    return gear


def _handwheel_solid() -> cq.Workplane:
    thickness = 0.010
    wheel = cq.Workplane("XY").circle(0.036).extrude(thickness)
    wheel = wheel.cut(cq.Workplane("XY").circle(0.029).extrude(thickness))
    wheel = wheel.union(cq.Workplane("XY").circle(0.011).extrude(thickness))
    for angle_deg in (0.0, 120.0, 240.0):
        spoke = (
            cq.Workplane("XY")
            .center(0.0205, 0.0)
            .rect(0.019, 0.005)
            .extrude(thickness)
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
        )
        wheel = wheel.union(spoke)
    return wheel.translate((0.0, 0.0, 0.072))


def _output_flange_solid() -> cq.Workplane:
    flange = cq.Workplane("XY").circle(0.022).extrude(0.008)
    flange = flange.cut(cq.Workplane("XY").circle(0.0065).extrude(0.012))
    for hole_index in range(4):
        angle = math.radians(45.0 + 90.0 * hole_index)
        hx, hy = _polar_xy(0.0155, angle)
        flange = flange.cut(cq.Workplane("XY").center(hx, hy).circle(0.003).extrude(0.012))
    return flange.translate((0.0, 0.0, 0.058))


def _bed_solid() -> cq.Workplane:
    bed = (
        cq.Workplane("XY")
        .box(0.276, 0.148, BED_THICKNESS)
        .translate((0.0, 0.0, BED_THICKNESS / 2.0))
    )
    bed = bed.union(
        cq.Workplane("XY")
        .box(0.192, 0.028, PAD_HEIGHT)
        .translate((0.0, 0.0, BED_THICKNESS + PAD_HEIGHT / 2.0))
    )
    bed = bed.union(
        cq.Workplane("XY")
        .box(0.214, 0.012, PAD_HEIGHT)
        .translate((0.0, 0.048, BED_THICKNESS + PAD_HEIGHT / 2.0))
    )
    bed = bed.union(
        cq.Workplane("XY")
        .box(0.214, 0.012, PAD_HEIGHT)
        .translate((0.0, -0.048, BED_THICKNESS + PAD_HEIGHT / 2.0))
    )
    for x_pos, width in ((INPUT_X, 0.052), (COMPOUND_X, 0.060), (OUTPUT_X, 0.052)):
        bed = bed.union(
            cq.Workplane("XY")
            .box(width, 0.050, PAD_HEIGHT)
            .translate((x_pos, 0.0, BED_THICKNESS + PAD_HEIGHT / 2.0))
        )
    for x_hole in (-0.105, 0.105):
        for y_hole in (-0.050, 0.050):
            bed = bed.cut(cq.Workplane("XY").center(x_hole, y_hole).circle(0.006).extrude(0.040))
    return bed


def _support_body_solid(width: float, depth: float) -> cq.Workplane:
    flange = cq.Workplane("XY").box(width, depth, 0.008).translate((0.0, 0.0, 0.004))
    grounded_core = (
        cq.Workplane("XY").box(width * 0.34, depth * 0.44, 0.020).translate((0.0, 0.0, 0.010))
    )
    profile = [
        (-0.42 * width, 0.0),
        (-0.48 * width, 0.010),
        (-0.24 * width, 0.010),
        (-0.18 * width, 0.029),
        (-0.12 * width, 0.036),
        (0.12 * width, 0.036),
        (0.18 * width, 0.029),
        (0.24 * width, 0.010),
        (0.48 * width, 0.010),
        (0.42 * width, 0.0),
    ]
    pedestal = (
        cq.Workplane("XZ")
        .polyline(profile)
        .close()
        .extrude(depth * 0.74)
        .translate((0.0, -0.37 * depth, 0.0))
    )
    top_cap = cq.Workplane("XY").box(width * 0.44, depth * 0.50, 0.006).translate((0.0, 0.0, 0.033))
    return flange.union(grounded_core).union(pedestal).union(top_cap)


def _input_gear_solid() -> cq.Workplane:
    assembly = (
        cq.Workplane("XY")
        .circle(0.0068)
        .extrude(0.094 + JOURNAL_INSERTION)
        .translate((0.0, 0.0, -JOURNAL_INSERTION))
    )
    assembly = assembly.union(
        cq.Workplane("XY").circle(0.011).extrude(0.006).translate((0.0, 0.0, 0.002))
    )
    assembly = assembly.union(
        _spur_gear_solid(
            tooth_count=20,
            root_radius=0.0270,
            tip_radius=0.0315,
            thickness=0.012,
            hub_radius=0.014,
            hub_height=0.008,
            bore_radius=0.0055,
            lightening_radius=0.017,
            lightening_hole_radius=0.0048,
            lightening_hole_count=4,
        ).translate((0.0, 0.0, 0.012))
    )
    assembly = assembly.union(
        cq.Workplane("XY").circle(0.009).extrude(0.004).translate((0.0, 0.0, 0.088))
    )
    return assembly


def _compound_cluster_solid() -> cq.Workplane:
    cluster = (
        cq.Workplane("XY")
        .circle(0.0068)
        .extrude(0.068 + JOURNAL_INSERTION)
        .translate((0.0, 0.0, -JOURNAL_INSERTION))
    )
    cluster = cluster.union(
        _spur_gear_solid(
            tooth_count=32,
            root_radius=0.0450,
            tip_radius=0.0495,
            thickness=0.012,
            hub_radius=0.016,
            hub_height=0.008,
            bore_radius=0.0056,
            lightening_radius=0.024,
            lightening_hole_radius=0.0062,
            lightening_hole_count=5,
        ).translate((0.0, 0.0, 0.010))
    )
    cluster = cluster.union(
        _spur_gear_solid(
            tooth_count=16,
            root_radius=0.0210,
            tip_radius=0.0255,
            thickness=0.010,
            hub_radius=0.012,
            hub_height=0.006,
            bore_radius=0.0056,
            lightening_radius=0.0125,
            lightening_hole_radius=0.0032,
            lightening_hole_count=3,
        ).translate((0.0, 0.0, 0.032))
    )
    cluster = cluster.union(
        cq.Workplane("XY").circle(0.012).extrude(0.008).translate((0.0, 0.0, 0.022))
    )
    cluster = cluster.union(
        cq.Workplane("XY").circle(0.009).extrude(0.005).translate((0.0, 0.0, 0.058))
    )
    return cluster


def _output_gear_solid() -> cq.Workplane:
    assembly = (
        cq.Workplane("XY")
        .circle(0.0070)
        .extrude(0.082 + JOURNAL_INSERTION)
        .translate((0.0, 0.0, -JOURNAL_INSERTION))
    )
    assembly = assembly.union(
        cq.Workplane("XY").circle(0.0115).extrude(0.006).translate((0.0, 0.0, 0.002))
    )
    assembly = assembly.union(
        _spur_gear_solid(
            tooth_count=36,
            root_radius=0.0510,
            tip_radius=0.0555,
            thickness=0.010,
            hub_radius=0.017,
            hub_height=0.008,
            bore_radius=0.0058,
            lightening_radius=0.029,
            lightening_hole_radius=0.0063,
            lightening_hole_count=6,
        ).translate((0.0, 0.0, 0.032))
    )
    return assembly


def _synchronized_pose(input_angle: float) -> dict[str, float]:
    return {
        "input_spin": input_angle,
        "compound_spin": -INPUT_STAGE_RATIO * input_angle,
        "output_spin": OUTPUT_STAGE_RATIO * input_angle,
    }


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_compound_gear_train", assets=ASSETS)

    model.material("painted_steel", rgba=(0.24, 0.28, 0.32, 1.0))
    model.material("machined_steel", rgba=(0.66, 0.68, 0.71, 1.0))
    model.material("dark_oxide", rgba=(0.14, 0.15, 0.17, 1.0))
    model.material("bronze", rgba=(0.73, 0.55, 0.24, 1.0))
    model.material("signal_red", rgba=(0.68, 0.15, 0.13, 1.0))

    bed = model.part("bed")
    bed.visual(
        mesh_from_cadquery(_bed_solid(), "gear_train_bed.obj", assets=ASSETS),
        material="painted_steel",
    )
    for x_hole in (-0.105, 0.105):
        for y_hole in (-0.050, 0.050):
            bed.visual(
                Cylinder(radius=0.005, length=0.004),
                origin=Origin(xyz=(x_hole, y_hole, BED_THICKNESS + 0.002)),
                material="dark_oxide",
            )
    bed.inertial = Inertial.from_geometry(
        Box((0.276, 0.148, 0.026)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
    )

    input_support = model.part("input_support")
    input_support.visual(
        mesh_from_cadquery(
            _support_body_solid(width=0.050, depth=0.044),
            "input_support.obj",
            assets=ASSETS,
        ),
        material="painted_steel",
    )
    input_support.visual(
        Cylinder(radius=0.0095, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.037)),
        material="bronze",
    )
    for x_bolt in (-0.014, 0.014):
        input_support.visual(
            Cylinder(radius=0.0036, length=0.003),
            origin=Origin(xyz=(x_bolt, 0.0, 0.0095)),
            material="dark_oxide",
        )
    input_support.inertial = Inertial.from_geometry(
        Box((0.050, 0.044, SUPPORT_HEIGHT)),
        mass=0.95,
        origin=Origin(xyz=(0.0, 0.0, SUPPORT_HEIGHT / 2.0)),
    )

    compound_support = model.part("compound_support")
    compound_support.visual(
        mesh_from_cadquery(
            _support_body_solid(width=0.058, depth=0.048),
            "compound_support.obj",
            assets=ASSETS,
        ),
        material="painted_steel",
    )
    compound_support.visual(
        Cylinder(radius=0.0105, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.037)),
        material="bronze",
    )
    for x_bolt in (-0.016, 0.016):
        compound_support.visual(
            Cylinder(radius=0.0038, length=0.003),
            origin=Origin(xyz=(x_bolt, 0.0, 0.0095)),
            material="dark_oxide",
        )
    compound_support.inertial = Inertial.from_geometry(
        Box((0.058, 0.048, SUPPORT_HEIGHT)),
        mass=1.15,
        origin=Origin(xyz=(0.0, 0.0, SUPPORT_HEIGHT / 2.0)),
    )

    output_support = model.part("output_support")
    output_support.visual(
        mesh_from_cadquery(
            _support_body_solid(width=0.052, depth=0.046),
            "output_support.obj",
            assets=ASSETS,
        ),
        material="painted_steel",
    )
    output_support.visual(
        Cylinder(radius=0.0100, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.037)),
        material="bronze",
    )
    for x_bolt in (-0.014, 0.014):
        output_support.visual(
            Cylinder(radius=0.0036, length=0.003),
            origin=Origin(xyz=(x_bolt, 0.0, 0.0095)),
            material="dark_oxide",
        )
    output_support.inertial = Inertial.from_geometry(
        Box((0.052, 0.046, SUPPORT_HEIGHT)),
        mass=1.0,
        origin=Origin(xyz=(0.0, 0.0, SUPPORT_HEIGHT / 2.0)),
    )

    input_gear = model.part("input_gear")
    input_gear.visual(
        mesh_from_cadquery(_input_gear_solid(), "input_gear.obj", assets=ASSETS),
        material="machined_steel",
    )
    input_gear.visual(
        mesh_from_cadquery(_handwheel_solid(), "input_handwheel.obj", assets=ASSETS),
        material="signal_red",
    )
    input_gear.inertial = Inertial.from_geometry(
        Box((0.074, 0.074, 0.094 + JOURNAL_INSERTION)),
        mass=1.05,
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
    )

    compound_cluster = model.part("compound_cluster")
    compound_cluster.visual(
        mesh_from_cadquery(
            _compound_cluster_solid(),
            "compound_cluster.obj",
            assets=ASSETS,
        ),
        material="machined_steel",
    )
    compound_cluster.inertial = Inertial.from_geometry(
        Box((0.108, 0.108, 0.068 + JOURNAL_INSERTION)),
        mass=1.55,
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
    )

    output_gear = model.part("output_gear")
    output_gear.visual(
        mesh_from_cadquery(_output_gear_solid(), "output_gear.obj", assets=ASSETS),
        material="machined_steel",
    )
    output_gear.visual(
        mesh_from_cadquery(_output_flange_solid(), "output_flange.obj", assets=ASSETS),
        material="dark_oxide",
    )
    output_gear.inertial = Inertial.from_geometry(
        Box((0.116, 0.116, 0.082 + JOURNAL_INSERTION)),
        mass=1.45,
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
    )

    model.articulation(
        "bed_to_input_support",
        ArticulationType.FIXED,
        parent="bed",
        child="input_support",
        origin=Origin(xyz=(INPUT_X, 0.0, SUPPORT_MOUNT_Z)),
    )
    model.articulation(
        "bed_to_compound_support",
        ArticulationType.FIXED,
        parent="bed",
        child="compound_support",
        origin=Origin(xyz=(COMPOUND_X, 0.0, SUPPORT_MOUNT_Z)),
    )
    model.articulation(
        "bed_to_output_support",
        ArticulationType.FIXED,
        parent="bed",
        child="output_support",
        origin=Origin(xyz=(OUTPUT_X, 0.0, SUPPORT_MOUNT_Z)),
    )

    model.articulation(
        "input_spin",
        ArticulationType.CONTINUOUS,
        parent="input_support",
        child="input_gear",
        origin=Origin(xyz=(0.0, 0.0, SUPPORT_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=SPIN_LIMITS,
    )
    model.articulation(
        "compound_spin",
        ArticulationType.CONTINUOUS,
        parent="compound_support",
        child="compound_cluster",
        origin=Origin(xyz=(0.0, 0.0, SUPPORT_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=SPIN_LIMITS,
    )
    model.articulation(
        "output_spin",
        ArticulationType.CONTINUOUS,
        parent="output_support",
        child="output_gear",
        origin=Origin(xyz=(0.0, 0.0, SUPPORT_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=SPIN_LIMITS,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap(
        "input_gear",
        "compound_cluster",
        reason="first-stage spur gear teeth intentionally intermesh",
    )
    ctx.allow_overlap(
        "input_support",
        "input_gear",
        reason="input shaft journal is intentionally seated inside the support bearing",
    )
    ctx.allow_overlap(
        "compound_cluster",
        "output_gear",
        reason="second-stage spur gear teeth intentionally intermesh",
    )
    ctx.allow_overlap(
        "compound_support",
        "compound_cluster",
        reason="compound shaft journal is intentionally seated inside the support bearing",
    )
    ctx.allow_overlap(
        "output_support",
        "output_gear",
        reason="output shaft journal is intentionally seated inside the support bearing",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=96,
        overlap_tol=0.002,
        overlap_volume_tol=0.0,
    )

    ctx.expect_aabb_gap("input_support", "bed", axis="z", max_gap=0.001, max_penetration=0.001)
    ctx.expect_aabb_gap("compound_support", "bed", axis="z", max_gap=0.001, max_penetration=0.001)
    ctx.expect_aabb_gap("output_support", "bed", axis="z", max_gap=0.001, max_penetration=0.001)

    ctx.expect_origin_distance("input_gear", "input_support", axes="xy", max_dist=0.002)
    ctx.expect_origin_distance("compound_cluster", "compound_support", axes="xy", max_dist=0.002)
    ctx.expect_origin_distance("output_gear", "output_support", axes="xy", max_dist=0.002)

    ctx.expect_aabb_gap("input_gear", "input_support", axis="z", max_gap=0.0015, max_penetration=JOURNAL_INSERTION + 0.0005)
    ctx.expect_aabb_gap("compound_cluster", "compound_support", axis="z", max_gap=0.0015, max_penetration=JOURNAL_INSERTION + 0.0005)
    ctx.expect_aabb_gap("output_gear", "output_support", axis="z", max_gap=0.0015, max_penetration=JOURNAL_INSERTION + 0.0005)

    ctx.expect_aabb_overlap("input_gear", "compound_cluster", axes="xy", min_overlap=0.0015)
    ctx.expect_aabb_overlap("compound_cluster", "output_gear", axes="xy", min_overlap=0.0025)

    for input_angle in (-1.2, -0.45, 0.9, 1.7):
        with ctx.pose(_synchronized_pose(input_angle)):
            ctx.expect_origin_distance("input_gear", "input_support", axes="xy", max_dist=0.002)
            ctx.expect_origin_distance("compound_cluster", "compound_support", axes="xy", max_dist=0.002)
            ctx.expect_origin_distance("output_gear", "output_support", axes="xy", max_dist=0.002)
            ctx.expect_aabb_gap("input_gear", "input_support", axis="z", max_gap=0.0015, max_penetration=JOURNAL_INSERTION + 0.0005)
            ctx.expect_aabb_gap("compound_cluster", "compound_support", axis="z", max_gap=0.0015, max_penetration=JOURNAL_INSERTION + 0.0005)
            ctx.expect_aabb_gap("output_gear", "output_support", axis="z", max_gap=0.0015, max_penetration=JOURNAL_INSERTION + 0.0005)
            ctx.expect_aabb_overlap("input_gear", "compound_cluster", axes="xy", min_overlap=0.0015)
            ctx.expect_aabb_overlap("compound_cluster", "output_gear", axes="xy", min_overlap=0.0025)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
