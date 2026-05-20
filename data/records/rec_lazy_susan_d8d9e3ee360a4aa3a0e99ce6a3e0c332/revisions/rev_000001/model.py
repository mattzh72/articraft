from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dining_table_lazy_susan")

    warm_walnut = model.material("warm_walnut", rgba=(0.48, 0.25, 0.11, 1.0))
    dark_walnut = model.material("dark_walnut", rgba=(0.23, 0.11, 0.045, 1.0))
    endgrain = model.material("endgrain", rgba=(0.57, 0.33, 0.16, 1.0))
    satin_brass = model.material("satin_brass", rgba=(0.92, 0.70, 0.32, 1.0))
    shadow_black = model.material("shadow_gap", rgba=(0.025, 0.018, 0.012, 1.0))

    table = model.part("table_base")

    tabletop_profile = [
        (0.000, -0.030),
        (0.635, -0.030),
        (0.690, -0.022),
        (0.720, -0.006),
        (0.720, 0.006),
        (0.690, 0.024),
        (0.635, 0.030),
        (0.000, 0.030),
    ]
    table.visual(
        mesh_from_geometry(LatheGeometry(tabletop_profile, segments=96), "broad_bullnose_tabletop"),
        origin=Origin(xyz=(0.0, 0.0, 0.732)),
        material=warm_walnut,
        name="tabletop",
    )
    table.visual(
        mesh_from_geometry(TorusGeometry(0.665, 0.006, radial_segments=12, tubular_segments=96), "outer_inlay_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.764)),
        material=dark_walnut,
        name="outer_inlay",
    )
    table.visual(
        mesh_from_geometry(TorusGeometry(0.440, 0.004, radial_segments=10, tubular_segments=96), "inner_inlay_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.765)),
        material=dark_walnut,
        name="inner_inlay",
    )

    pedestal_profile = [
        (0.000, 0.000),
        (0.120, 0.000),
        (0.145, 0.025),
        (0.138, 0.060),
        (0.095, 0.085),
        (0.078, 0.180),
        (0.060, 0.270),
        (0.083, 0.360),
        (0.095, 0.500),
        (0.138, 0.545),
        (0.150, 0.590),
        (0.128, 0.620),
        (0.000, 0.620),
    ]
    table.visual(
        mesh_from_geometry(LatheGeometry(pedestal_profile, segments=80), "turned_pedestal"),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=warm_walnut,
        name="pedestal",
    )
    table.visual(
        Cylinder(radius=0.250, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.686)),
        material=dark_walnut,
        name="apron_plate",
    )
    table.visual(
        Cylinder(radius=0.190, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.768)),
        material=satin_brass,
        name="bearing_race",
    )

    for i in range(4):
        angle = i * math.pi / 2.0
        cx = math.cos(angle) * 0.290
        cy = math.sin(angle) * 0.290
        tx = math.cos(angle) * 0.570
        ty = math.sin(angle) * 0.570
        table.visual(
            Box((0.600, 0.115, 0.070)),
            origin=Origin(xyz=(cx, cy, 0.055), rpy=(0.0, 0.0, angle)),
            material=dark_walnut,
            name=f"foot_{i}",
        )
        table.visual(
            Cylinder(radius=0.070, length=0.040),
            origin=Origin(xyz=(tx, ty, 0.020)),
            material=endgrain,
            name=f"rounded_foot_tip_{i}",
        )
        table.visual(
            Box((0.380, 0.040, 0.045)),
            origin=Origin(xyz=(math.cos(angle) * 0.245, math.sin(angle) * 0.245, 0.205), rpy=(0.0, 0.0, angle)),
            material=warm_walnut,
            name=f"knee_brace_{i}",
        )

    tray = model.part("serving_tray")
    tray_profile = [
        (0.000, 0.006),
        (0.315, 0.006),
        (0.380, 0.011),
        (0.405, 0.024),
        (0.398, 0.044),
        (0.368, 0.058),
        (0.000, 0.058),
    ]
    tray.visual(
        mesh_from_geometry(LatheGeometry(tray_profile, segments=96), "raised_lazy_susan_tray"),
        material=warm_walnut,
        name="tray_body",
    )
    tray.visual(
        Cylinder(radius=0.155, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=satin_brass,
        name="lower_bearing",
    )
    tray.visual(
        mesh_from_geometry(TorusGeometry(0.355, 0.006, radial_segments=12, tubular_segments=96), "tray_recessed_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.059)),
        material=dark_walnut,
        name="tray_inlay_ring",
    )
    for i, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        tray.visual(
            Box((0.300, 0.014, 0.003)),
            origin=Origin(xyz=(math.cos(angle) * 0.185, math.sin(angle) * 0.185, 0.059), rpy=(0.0, 0.0, angle)),
            material=dark_walnut,
            name=f"radial_inlay_{i}",
        )
    tray.visual(
        Cylinder(radius=0.030, length=0.006),
        origin=Origin(xyz=(0.275, 0.0, 0.063)),
        material=satin_brass,
        name="brass_index",
    )
    tray.visual(
        Cylinder(radius=0.052, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.064)),
        material=satin_brass,
        name="center_cap",
    )

    model.articulation(
        "tray_swivel",
        ArticulationType.CONTINUOUS,
        parent=table,
        child=tray,
        origin=Origin(xyz=(0.0, 0.0, 0.774)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.5, velocity=1.4),
        motion_properties=MotionProperties(damping=0.08, friction=0.03),
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

    table = object_model.get_part("table_base")
    tray = object_model.get_part("serving_tray")
    swivel = object_model.get_articulation("tray_swivel")

    ctx.check(
        "lazy Susan uses a vertical continuous swivel",
        swivel.articulation_type == ArticulationType.CONTINUOUS and tuple(swivel.axis) == (0.0, 0.0, 1.0),
        details=f"type={swivel.articulation_type}, axis={swivel.axis}",
    )
    ctx.expect_within(
        tray,
        table,
        axes="xy",
        inner_elem="tray_body",
        outer_elem="tabletop",
        margin=0.0,
        name="serving tray is centered within the tabletop",
    )
    ctx.expect_overlap(
        tray,
        table,
        axes="xy",
        elem_a="lower_bearing",
        elem_b="bearing_race",
        min_overlap=0.28,
        name="lazy Susan bearing races are concentric",
    )
    ctx.expect_gap(
        tray,
        table,
        axis="z",
        positive_elem="lower_bearing",
        negative_elem="bearing_race",
        min_gap=0.0,
        max_gap=0.0005,
        name="lazy Susan lower race seats on the fixed bearing",
    )

    rest_aabb = ctx.part_element_world_aabb(tray, elem="brass_index")
    with ctx.pose({swivel: math.pi / 2.0}):
        turned_aabb = ctx.part_element_world_aabb(tray, elem="brass_index")
        ctx.expect_gap(
            tray,
            table,
            axis="z",
            positive_elem="lower_bearing",
            negative_elem="bearing_race",
            min_gap=0.0,
            max_gap=0.0005,
            name="bearing seating is preserved while rotating",
        )

    def _center_xy(aabb):
        if aabb is None:
            return None
        low, high = aabb
        return ((low[0] + high[0]) * 0.5, (low[1] + high[1]) * 0.5)

    rest_xy = _center_xy(rest_aabb)
    turned_xy = _center_xy(turned_aabb)
    ctx.check(
        "off-center tray detail rotates about the table center",
        rest_xy is not None
        and turned_xy is not None
        and rest_xy[0] > 0.22
        and abs(rest_xy[1]) < 0.04
        and abs(turned_xy[0]) < 0.04
        and turned_xy[1] > 0.22,
        details=f"rest_xy={rest_xy}, turned_xy={turned_xy}",
    )

    return ctx.report()


object_model = build_object_model()
