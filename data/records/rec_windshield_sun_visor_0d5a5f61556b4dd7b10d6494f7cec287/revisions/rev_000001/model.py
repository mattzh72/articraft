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
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _rounded_box(size: tuple[float, float, float], radius: float, center: tuple[float, float, float]):
    """CadQuery rounded-rectangle slab, authored in meters."""
    sx, sy, sz = size
    cx, cy, cz = center
    return (
        cq.Workplane("XY")
        .box(sx, sy, sz)
        .edges("|X")
        .fillet(radius)
        .translate((cx, cy, cz))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_windshield_sun_visor")

    headliner = model.material("woven_headliner", rgba=(0.69, 0.65, 0.57, 1.0))
    warm_polymer = model.material("warm_grey_polymer", rgba=(0.54, 0.53, 0.49, 1.0))
    fabric = model.material("soft_twill_fabric", rgba=(0.62, 0.60, 0.55, 1.0))
    seam = model.material("subtle_shadow_seam", rgba=(0.34, 0.34, 0.32, 1.0))
    elastomer = model.material("matte_elastomer", rgba=(0.08, 0.085, 0.08, 1.0))
    painted_metal = model.material("satin_painted_metal", rgba=(0.18, 0.19, 0.19, 1.0))

    roof_header = model.part("roof_header")
    roof_header.visual(
        Box((0.18, 0.72, 0.025)),
        origin=Origin(xyz=(-0.035, 0.30, 0.035)),
        material=headliner,
        name="headliner_panel",
    )
    roof_header.visual(
        Box((0.088, 0.080, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=warm_polymer,
        name="mount_plinth",
    )
    roof_header.visual(
        Cylinder(radius=0.027, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=warm_polymer,
        name="pivot_socket",
    )
    roof_header.visual(
        Cylinder(radius=0.034, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.0155)),
        material=painted_metal,
        name="trim_ring",
    )
    roof_header.visual(
        Box((0.060, 0.056, 0.010)),
        origin=Origin(xyz=(0.0, 0.545, 0.018)),
        material=warm_polymer,
        name="retainer_base",
    )
    roof_header.visual(
        Box((0.009, 0.040, 0.035)),
        origin=Origin(xyz=(-0.018, 0.545, -0.002)),
        material=warm_polymer,
        name="retainer_fork_0",
    )
    roof_header.visual(
        Box((0.009, 0.040, 0.035)),
        origin=Origin(xyz=(0.018, 0.545, -0.002)),
        material=warm_polymer,
        name="retainer_fork_1",
    )
    roof_header.visual(
        Box((0.042, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, 0.522, -0.018)),
        material=elastomer,
        name="retainer_pad",
    )

    hinge_arm = model.part("hinge_arm")
    hinge_arm.visual(
        Cylinder(radius=0.0135, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
        material=painted_metal,
        name="vertical_spindle",
    )
    hinge_arm.visual(
        Cylinder(radius=0.022, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.017)),
        material=painted_metal,
        name="pivot_cap",
    )
    hinge_arm.visual(
        Cylinder(radius=0.0075, length=0.032),
        origin=Origin(xyz=(0.0, 0.026, -0.020)),
        material=painted_metal,
        name="drop_post",
    )
    hinge_arm.visual(
        Box((0.018, 0.050, 0.014)),
        origin=Origin(xyz=(0.0, 0.045, -0.035)),
        material=painted_metal,
        name="elbow_block",
    )
    hinge_arm.visual(
        Cylinder(radius=0.0055, length=0.600),
        origin=Origin(xyz=(0.0, 0.330, -0.035), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=painted_metal,
        name="hinge_rod",
    )
    hinge_arm.visual(
        Cylinder(radius=0.0105, length=0.030),
        origin=Origin(xyz=(0.0, 0.030, -0.035), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=painted_metal,
        name="hinge_collar_0",
    )
    hinge_arm.visual(
        Cylinder(radius=0.0105, length=0.020),
        origin=Origin(xyz=(0.0, 0.610, -0.035), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=painted_metal,
        name="hinge_collar_1",
    )

    visor_panel = model.part("visor_panel")
    visor_panel.visual(
        mesh_from_cadquery(
            _rounded_box((0.020, 0.525, 0.205), 0.026, (0.0, 0.285, -0.112)),
            "visor_elastomer_edge",
            tolerance=0.0007,
            angular_tolerance=0.05,
        ),
        material=elastomer,
        name="edge_band",
    )
    visor_panel.visual(
        mesh_from_cadquery(
            _rounded_box((0.017, 0.492, 0.184), 0.021, (-0.0008, 0.286, -0.111)),
            "visor_fabric_core",
            tolerance=0.0007,
            angular_tolerance=0.05,
        ),
        material=fabric,
        name="fabric_core",
    )
    visor_panel.visual(
        Box((0.003, 0.392, 0.0032)),
        origin=Origin(xyz=(-0.0102, 0.288, -0.038)),
        material=seam,
        name="upper_seam",
    )
    visor_panel.visual(
        Box((0.003, 0.392, 0.0032)),
        origin=Origin(xyz=(-0.0102, 0.288, -0.184)),
        material=seam,
        name="lower_seam",
    )
    visor_panel.visual(
        Box((0.003, 0.0032, 0.130)),
        origin=Origin(xyz=(-0.0102, 0.083, -0.111)),
        material=seam,
        name="side_seam_0",
    )
    visor_panel.visual(
        Box((0.003, 0.0032, 0.130)),
        origin=Origin(xyz=(-0.0102, 0.493, -0.111)),
        material=seam,
        name="side_seam_1",
    )
    for index, y_center, length in (
        (0, 0.105, 0.105),
        (1, 0.290, 0.120),
        (2, 0.485, 0.075),
    ):
        visor_panel.visual(
            Cylinder(radius=0.014, length=length),
            origin=Origin(xyz=(0.0, y_center, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=warm_polymer,
            name=f"hinge_barrel_{index}",
        )
        visor_panel.visual(
            Box((0.014, length, 0.012)),
            origin=Origin(xyz=(0.0, y_center, -0.017)),
            material=warm_polymer,
            name=f"barrel_web_{index}",
        )

    model.articulation(
        "roof_swing",
        ArticulationType.REVOLUTE,
        parent=roof_header,
        child=hinge_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.4, lower=0.0, upper=1.45),
    )
    model.articulation(
        "panel_pivot",
        ArticulationType.REVOLUTE,
        parent=hinge_arm,
        child=visor_panel,
        origin=Origin(xyz=(0.0, 0.040, -0.035)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.8, lower=-0.25, upper=1.45),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    roof_header = object_model.get_part("roof_header")
    hinge_arm = object_model.get_part("hinge_arm")
    visor_panel = object_model.get_part("visor_panel")
    roof_swing = object_model.get_articulation("roof_swing")
    panel_pivot = object_model.get_articulation("panel_pivot")

    ctx.allow_overlap(
        roof_header,
        hinge_arm,
        elem_a="pivot_socket",
        elem_b="vertical_spindle",
        reason="The spindle is intentionally captured inside the roof pivot socket proxy.",
    )
    ctx.expect_within(
        hinge_arm,
        roof_header,
        axes="xy",
        inner_elem="vertical_spindle",
        outer_elem="pivot_socket",
        margin=0.002,
        name="spindle is centered inside roof socket",
    )
    ctx.expect_overlap(
        hinge_arm,
        roof_header,
        axes="z",
        elem_a="vertical_spindle",
        elem_b="pivot_socket",
        min_overlap=0.018,
        name="spindle remains captured by roof socket",
    )

    for index, min_overlap in ((0, 0.075), (1, 0.090), (2, 0.055)):
        barrel = f"hinge_barrel_{index}"
        ctx.allow_overlap(
            hinge_arm,
            visor_panel,
            elem_a="hinge_rod",
            elem_b=barrel,
            reason="The visible hinge rod is intentionally represented as passing through a polymer barrel.",
        )
        ctx.expect_within(
            hinge_arm,
            visor_panel,
            axes="xz",
            inner_elem="hinge_rod",
            outer_elem=barrel,
            margin=0.001,
            name=f"hinge rod is concentric in barrel {index}",
        )
        ctx.expect_overlap(
            hinge_arm,
            visor_panel,
            axes="y",
            elem_a="hinge_rod",
            elem_b=barrel,
            min_overlap=min_overlap,
            name=f"hinge rod spans barrel {index}",
        )

    ctx.check(
        "two visible revolute pivots",
        len(object_model.articulations) == 2
        and roof_swing.articulation_type == ArticulationType.REVOLUTE
        and panel_pivot.articulation_type == ArticulationType.REVOLUTE,
        details=f"articulations={[a.name for a in object_model.articulations]}",
    )

    deployed_aabb = ctx.part_world_aabb(visor_panel)
    with ctx.pose({panel_pivot: 1.20}):
        folded_aabb = ctx.part_world_aabb(visor_panel)
    ctx.check(
        "panel folds up about the hinge rod",
        deployed_aabb is not None
        and folded_aabb is not None
        and folded_aabb[0][2] > deployed_aabb[0][2] + 0.045,
        details=f"deployed={deployed_aabb}, folded={folded_aabb}",
    )

    rest_swing_aabb = ctx.part_world_aabb(visor_panel)
    with ctx.pose({roof_swing: 1.05}):
        swung_aabb = ctx.part_world_aabb(visor_panel)
    rest_center_x = None if rest_swing_aabb is None else 0.5 * (rest_swing_aabb[0][0] + rest_swing_aabb[1][0])
    swung_center_x = None if swung_aabb is None else 0.5 * (swung_aabb[0][0] + swung_aabb[1][0])
    ctx.check(
        "roof swing pivot carries visor sideways",
        rest_center_x is not None and swung_center_x is not None and swung_center_x < rest_center_x - 0.15,
        details=f"rest={rest_swing_aabb}, swung={swung_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
