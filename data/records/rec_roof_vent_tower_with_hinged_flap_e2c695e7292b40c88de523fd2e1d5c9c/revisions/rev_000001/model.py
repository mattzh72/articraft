from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    VentGrilleFrame,
    VentGrilleGeometry,
    VentGrilleMounts,
    VentGrilleSlats,
    VentGrilleSleeve,
    mesh_from_cadquery,
    mesh_from_geometry,
)


GALVANIZED = Material("weathered_galvanized_steel", color=(0.58, 0.64, 0.66, 1.0))
DARK_GALVANIZED = Material("darkened_galvanized_edges", color=(0.34, 0.39, 0.40, 1.0))
ROOF_MEMBRANE = Material("charcoal_roof_membrane", color=(0.06, 0.065, 0.07, 1.0))
ROOF_SEAM = Material("slightly_raised_roof_seams", color=(0.10, 0.105, 0.11, 1.0))
BLACK_RUBBER = Material("black_epdm_gasket", color=(0.01, 0.01, 0.012, 1.0))
SHADOW = Material("dark_airway_shadow", color=(0.0, 0.0, 0.0, 1.0))
STAINLESS = Material("stainless_fasteners", color=(0.74, 0.75, 0.73, 1.0))


def _box(part, name, size, xyz, material, rpy=(0.0, 0.0, 0.0)):
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _cyl(part, name, radius, length, xyz, material, rpy=(0.0, 0.0, 0.0)):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _tapered_rect_tube(
    bottom_size: tuple[float, float],
    top_size: tuple[float, float],
    z0: float,
    height: float,
    wall: float,
):
    outer = (
        cq.Workplane("XY", origin=(0.0, 0.0, z0))
        .rect(bottom_size[0], bottom_size[1])
        .workplane(offset=height)
        .rect(top_size[0], top_size[1])
        .loft(combine=True)
    )
    inner = (
        cq.Workplane("XY", origin=(0.0, 0.0, z0 - 0.025))
        .rect(bottom_size[0] - (2.0 * wall), bottom_size[1] - (2.0 * wall))
        .workplane(offset=height + 0.050)
        .rect(top_size[0] - (2.0 * wall), top_size[1] - (2.0 * wall))
        .loft(combine=True)
    )
    return outer.cut(inner)


def _build_tower_body():
    body = _tapered_rect_tube((0.52, 0.39), (0.42, 0.31), 0.14, 0.72, 0.028)
    outlet_cutter = cq.Workplane("XY").box(0.34, 0.31, 0.36).translate((0.255, 0.0, 0.58))
    return body.cut(outlet_cutter)


def _build_rain_cap():
    sloped_hood = (
        cq.Workplane("XY", origin=(0.0, 0.0, 0.825))
        .rect(0.64, 0.50)
        .workplane(offset=0.125)
        .rect(0.39, 0.29)
        .loft(combine=True)
    )
    cap_band = cq.Workplane("XY").box(0.50, 0.38, 0.055).translate((0.0, 0.0, 0.835))
    drip_front = cq.Workplane("XY").box(0.66, 0.018, 0.035).translate((0.0, 0.259, 0.815))
    drip_back = cq.Workplane("XY").box(0.66, 0.018, 0.035).translate((0.0, -0.259, 0.815))
    drip_side_a = cq.Workplane("XY").box(0.018, 0.50, 0.035).translate((0.339, 0.0, 0.815))
    drip_side_b = cq.Workplane("XY").box(0.018, 0.50, 0.035).translate((-0.339, 0.0, 0.815))
    return sloped_hood.union(cap_band).union(drip_front).union(drip_back).union(drip_side_a).union(drip_side_b)


def _build_outlet_frame():
    outer = cq.Workplane("XY").box(0.112, 0.392, 0.380).translate((0.265, 0.0, 0.560))
    inner = cq.Workplane("XY").box(0.130, 0.292, 0.280).translate((0.275, 0.0, 0.560))
    bevel = cq.Workplane("XY").box(0.020, 0.335, 0.315).translate((0.325, 0.0, 0.560))
    return outer.cut(inner).union(bevel.cut(inner))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="roof_vent_tower")

    tower = model.part("tower")

    # Roof patch and architectural integration: a low-slope membrane roof with
    # raised seams, a flashing skirt, counterflashing, curb, sealant, and screws.
    _box(tower, "roof_membrane", (1.40, 0.95, 0.055), (0.0, 0.0, 0.0275), ROOF_MEMBRANE)
    for idx, y in enumerate((-0.36, -0.18, 0.0, 0.18, 0.36)):
        _box(tower, f"roof_seam_{idx}", (1.34, 0.012, 0.007), (0.0, y, 0.061), ROOF_SEAM)
    for idx, x in enumerate((-0.52, 0.52)):
        _box(tower, f"lap_joint_{idx}", (0.014, 0.90, 0.006), (x, 0.0, 0.062), ROOF_SEAM)

    _box(tower, "base_flashing", (0.90, 0.67, 0.018), (0.0, 0.0, 0.064), GALVANIZED)
    _box(tower, "curb_block", (0.58, 0.44, 0.125), (0.0, 0.0, 0.122), DARK_GALVANIZED)
    _box(tower, "front_counterflashing", (0.62, 0.035, 0.070), (0.0, 0.237, 0.118), GALVANIZED)
    _box(tower, "rear_counterflashing", (0.62, 0.035, 0.070), (0.0, -0.237, 0.118), GALVANIZED)
    _box(tower, "side_counterflashing_0", (0.035, 0.44, 0.070), (0.317, 0.0, 0.118), GALVANIZED)
    _box(tower, "side_counterflashing_1", (0.035, 0.44, 0.070), (-0.317, 0.0, 0.118), GALVANIZED)

    _cyl(tower, "front_sealant_bead", 0.006, 0.66, (0.0, 0.324, 0.074), BLACK_RUBBER, rpy=(0.0, math.pi / 2.0, 0.0))
    _cyl(tower, "rear_sealant_bead", 0.006, 0.66, (0.0, -0.324, 0.074), BLACK_RUBBER, rpy=(0.0, math.pi / 2.0, 0.0))
    _cyl(tower, "side_sealant_bead_0", 0.006, 0.45, (0.444, 0.0, 0.074), BLACK_RUBBER, rpy=(-math.pi / 2.0, 0.0, 0.0))
    _cyl(tower, "side_sealant_bead_1", 0.006, 0.45, (-0.444, 0.0, 0.074), BLACK_RUBBER, rpy=(-math.pi / 2.0, 0.0, 0.0))

    screw_points = [
        (-0.36, -0.25),
        (0.0, -0.25),
        (0.36, -0.25),
        (-0.36, 0.25),
        (0.0, 0.25),
        (0.36, 0.25),
        (-0.31, 0.0),
        (0.31, 0.0),
    ]
    for idx, (x, y) in enumerate(screw_points):
        _cyl(tower, f"flashing_screw_{idx}", 0.011, 0.005, (x, y, 0.0738), STAINLESS)

    tower.visual(
        mesh_from_cadquery(_build_tower_body(), "tapered_weatherproof_body", tolerance=0.0015),
        material=GALVANIZED,
        name="body_shell",
    )
    tower.visual(
        mesh_from_cadquery(_build_rain_cap(), "overhanging_rain_cap", tolerance=0.0015),
        material=GALVANIZED,
        name="rain_cap",
    )
    tower.visual(
        mesh_from_cadquery(_build_outlet_frame(), "flanged_outlet_frame", tolerance=0.001),
        material=DARK_GALVANIZED,
        name="outlet_frame",
    )
    _box(tower, "outlet_dark", (0.010, 0.270, 0.300), (0.214, 0.0, 0.580), SHADOW)

    grille = VentGrilleGeometry(
        (0.260, 0.205),
        frame=0.012,
        face_thickness=0.004,
        duct_depth=0.020,
        duct_wall=0.003,
        slat_pitch=0.024,
        slat_width=0.010,
        slat_angle_deg=34.0,
        slats=VentGrilleSlats(profile="boxed", direction="down", divider_count=1, divider_width=0.004),
        frame_profile=VentGrilleFrame(style="beveled", depth=0.0015),
        mounts=VentGrilleMounts(style="holes", inset=0.016, hole_diameter=0.004),
        sleeve=VentGrilleSleeve(style="short", depth=0.020, wall=0.003),
        center=False,
    )
    tower.visual(
        mesh_from_geometry(grille, "side_louver_0"),
        origin=Origin(xyz=(0.0, 0.178, 0.535), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=DARK_GALVANIZED,
        name="side_louver_0",
    )
    tower.visual(
        mesh_from_geometry(grille, "side_louver_1"),
        origin=Origin(xyz=(0.0, -0.178, 0.535), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=DARK_GALVANIZED,
        name="side_louver_1",
    )

    # Hinge brackets are fixed to the tower; the flap carries the rolled hinge
    # barrel between them, so the visible pivot line is physically supported.
    _box(tower, "hinge_leaf", (0.040, 0.430, 0.050), (0.296, 0.0, 0.775), DARK_GALVANIZED)
    _box(tower, "hinge_bracket_0", (0.076, 0.030, 0.070), (0.310, 0.211, 0.775), DARK_GALVANIZED)
    _box(tower, "hinge_bracket_1", (0.076, 0.030, 0.070), (0.310, -0.211, 0.775), DARK_GALVANIZED)
    _cyl(tower, "hinge_screw_0", 0.007, 0.004, (0.304, 0.120, 0.787), STAINLESS, rpy=(0.0, math.pi / 2.0, 0.0))
    _cyl(tower, "hinge_screw_1", 0.007, 0.004, (0.304, -0.120, 0.787), STAINLESS, rpy=(0.0, math.pi / 2.0, 0.0))

    flap = model.part("flap")
    _cyl(flap, "hinge_roll", 0.014, 0.350, (0.0, 0.0, 0.0), GALVANIZED, rpy=(-math.pi / 2.0, 0.0, 0.0))
    _box(flap, "flap_panel", (0.024, 0.360, 0.400), (0.026, 0.0, -0.205), GALVANIZED)
    _box(flap, "top_hem", (0.032, 0.365, 0.026), (0.019, 0.0, -0.019), DARK_GALVANIZED)
    _box(flap, "lower_hem", (0.038, 0.365, 0.030), (0.033, 0.0, -0.405), DARK_GALVANIZED)
    _box(flap, "side_hem_0", (0.034, 0.024, 0.360), (0.032, 0.178, -0.215), DARK_GALVANIZED)
    _box(flap, "side_hem_1", (0.034, 0.024, 0.360), (0.032, -0.178, -0.215), DARK_GALVANIZED)
    _box(flap, "diagonal_stiffener_0", (0.012, 0.300, 0.020), (0.044, 0.0, -0.235), DARK_GALVANIZED, rpy=(0.0, 0.0, 0.42))
    _box(flap, "diagonal_stiffener_1", (0.012, 0.300, 0.020), (0.045, 0.0, -0.285), DARK_GALVANIZED, rpy=(0.0, 0.0, -0.42))
    _box(flap, "gasket_top", (0.010, 0.300, 0.018), (0.011, 0.0, -0.055), BLACK_RUBBER)
    _box(flap, "gasket_bottom", (0.010, 0.300, 0.018), (0.011, 0.0, -0.355), BLACK_RUBBER)
    _box(flap, "gasket_side_0", (0.010, 0.018, 0.300), (0.011, 0.147, -0.205), BLACK_RUBBER)
    _box(flap, "gasket_side_1", (0.010, 0.018, 0.300), (0.011, -0.147, -0.205), BLACK_RUBBER)
    _cyl(flap, "drip_lip", 0.007, 0.355, (0.048, 0.0, -0.422), DARK_GALVANIZED, rpy=(-math.pi / 2.0, 0.0, 0.0))

    model.articulation(
        "tower_to_flap",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=flap,
        origin=Origin(xyz=(0.330, 0.0, 0.775)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.8, lower=0.0, upper=1.15),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    flap = object_model.get_part("flap")
    hinge = object_model.get_articulation("tower_to_flap")

    with ctx.pose({hinge: 0.0}):
        ctx.expect_overlap(
            flap,
            tower,
            axes="yz",
            elem_a="flap_panel",
            elem_b="outlet_dark",
            min_overlap=0.24,
            name="closed flap covers the outlet throat",
        )
        ctx.expect_gap(
            flap,
            tower,
            axis="x",
            positive_elem="lower_hem",
            negative_elem="outlet_frame",
            min_gap=0.003,
            max_gap=0.020,
            name="closed flap sits just proud of the outlet frame",
        )

    with ctx.pose({hinge: 1.0}):
        ctx.expect_gap(
            flap,
            tower,
            axis="x",
            positive_elem="lower_hem",
            negative_elem="outlet_frame",
            min_gap=0.22,
            name="opened flap swings outward from the tower",
        )

    return ctx.report()


object_model = build_object_model()
