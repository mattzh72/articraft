from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BlowerWheelGeometry,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    VentGrilleGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _rounded_rect_sketch(width: float, depth: float, radius: float) -> cq.Sketch:
    return cq.Sketch().rect(width, depth).vertices().fillet(radius).reset()


def _lofted_shell_section(
    sections: list[tuple[float, float, float, float]],
) -> cq.Workplane:
    sketches = [
        _rounded_rect_sketch(width, depth, radius).moved(cq.Location(cq.Vector(0.0, 0.0, z)))
        for z, width, depth, radius in sections
    ]
    return cq.Workplane("XY").placeSketch(*sketches).loft(combine=True)


def _build_tower_shell() -> cq.Workplane:
    outer = _lofted_shell_section(
        [
            (0.026, 0.172, 0.112, 0.028),
            (0.300, 0.180, 0.122, 0.032),
            (0.620, 0.176, 0.118, 0.030),
            (0.892, 0.162, 0.102, 0.026),
        ]
    )
    inner = _lofted_shell_section(
        [
            (0.036, 0.156, 0.096, 0.022),
            (0.300, 0.164, 0.106, 0.026),
            (0.620, 0.160, 0.102, 0.024),
            (0.850, 0.144, 0.086, 0.020),
        ]
    )

    front_opening = cq.Workplane("XY").box(0.124, 0.160, 0.730).translate((0.0, 0.032, 0.470))
    dial_recess = cq.Workplane("XY").circle(0.050).extrude(0.020).translate((0.0, 0.0, 0.872))

    collar = (
        cq.Workplane("XY")
        .circle(0.058)
        .extrude(0.030)
    )

    tower_shell = outer.cut(inner).cut(front_opening).cut(dial_recess).union(collar)
    return tower_shell


def _build_dial_mesh() -> cq.Workplane:
    dial = (
        cq.Workplane("XY")
        .circle(0.044)
        .extrude(0.010)
        .faces(">Z")
        .circle(0.039)
        .extrude(0.012)
    )
    pointer = cq.Workplane("XY").box(0.010, 0.020, 0.004).translate((0.024, 0.0, 0.020))
    return dial.union(pointer)


def _build_dial_well() -> cq.Workplane:
    return cq.Workplane("XY").circle(0.056).circle(0.047).extrude(0.012)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_tower_fan")

    base_black = model.material("base_black", rgba=(0.13, 0.14, 0.15, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.06, 0.06, 0.07, 1.0))
    tower_white = model.material("tower_white", rgba=(0.90, 0.91, 0.92, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.32, 0.34, 0.36, 1.0))
    dial_black = model.material("dial_black", rgba=(0.10, 0.11, 0.12, 1.0))
    rotor_dark = model.material("rotor_dark", rgba=(0.18, 0.19, 0.20, 1.0))
    rotor_steel = model.material("rotor_steel", rgba=(0.55, 0.58, 0.61, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.175, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=base_black,
        name="base_disk",
    )
    base.visual(
        Cylinder(radius=0.148, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=rubber_black,
        name="foot_ring",
    )
    base.visual(
        Cylinder(radius=0.102, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=trim_gray,
        name="top_trim",
    )
    base.visual(
        Cylinder(radius=0.048, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=trim_gray,
        name="pivot_stem",
    )

    tower = model.part("tower")
    tower.visual(
        mesh_from_cadquery(_build_tower_shell(), "tower_shell"),
        material=tower_white,
        name="shell",
    )
    tower.visual(
        mesh_from_geometry(
            VentGrilleGeometry(
                (0.126, 0.768),
                frame=0.010,
                face_thickness=0.004,
                duct_depth=0.020,
                duct_wall=0.003,
                slat_pitch=0.016,
                slat_width=0.0075,
                slat_angle_deg=34.0,
                corner_radius=0.010,
            ),
            "tower_grille",
        ),
        origin=Origin(xyz=(0.0, 0.058, 0.476), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=trim_gray,
        name="grille",
    )
    tower.visual(
        Cylinder(radius=0.056, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=trim_gray,
        name="tower_collar",
    )
    tower.visual(
        mesh_from_cadquery(_build_dial_well(), "dial_well"),
        origin=Origin(xyz=(0.0, 0.0, 0.886)),
        material=trim_gray,
        name="dial_well",
    )
    tower.visual(
        Cylinder(radius=0.009, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.881)),
        material=trim_gray,
        name="dial_bushing",
    )
    tower.visual(
        Box((0.030, 0.008, 0.010)),
        origin=Origin(xyz=(0.0, -0.042, 0.874)),
        material=trim_gray,
        name="rear_handle",
    )
    tower.visual(
        Cylinder(radius=0.014, length=0.068),
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        material=trim_gray,
        name="bottom_bearing",
    )
    tower.visual(
        Cylinder(radius=0.014, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.840)),
        material=trim_gray,
        name="top_bearing",
    )

    rotor = model.part("rotor")
    rotor.visual(
        mesh_from_geometry(
            BlowerWheelGeometry(
                outer_radius=0.045,
                inner_radius=0.018,
                width=0.690,
                blade_count=40,
                blade_thickness=0.0024,
                blade_sweep_deg=36.0,
                backplate=True,
                shroud=True,
            ),
            "tower_rotor",
        ),
        material=rotor_dark,
        name="wheel",
    )
    rotor.visual(
        Cylinder(radius=0.008, length=0.740),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=rotor_steel,
        name="shaft",
    )
    rotor.visual(
        Cylinder(radius=0.015, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.350)),
        material=rotor_steel,
        name="top_cap",
    )
    rotor.visual(
        Cylinder(radius=0.015, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.350)),
        material=rotor_steel,
        name="bottom_cap",
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_cadquery(_build_dial_mesh(), "selector_dial"),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=dial_black,
        name="dial_body",
    )
    dial.visual(
        Cylinder(radius=0.006, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=rotor_steel,
        name="dial_shaft",
    )

    model.articulation(
        "tower_oscillation",
        ArticulationType.REVOLUTE,
        parent=base,
        child=tower,
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.7,
            lower=-0.85,
            upper=0.85,
        ),
    )
    model.articulation(
        "rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.460)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=30.0,
        ),
    )
    model.articulation(
        "dial_turn",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=dial,
        origin=Origin(xyz=(0.0, 0.0, 0.896)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=6.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    tower = object_model.get_part("tower")
    rotor = object_model.get_part("rotor")
    dial = object_model.get_part("dial")
    tower_oscillation = object_model.get_articulation("tower_oscillation")

    ctx.expect_gap(
        tower,
        base,
        axis="z",
        positive_elem="tower_collar",
        negative_elem="pivot_stem",
        max_gap=0.002,
        max_penetration=0.0,
        name="tower collar sits on pivot stem",
    )
    ctx.expect_within(
        rotor,
        tower,
        axes="xy",
        inner_elem="wheel",
        outer_elem="shell",
        margin=0.004,
        name="blower wheel stays inside the tower footprint",
    )
    ctx.expect_overlap(
        rotor,
        tower,
        axes="z",
        elem_a="wheel",
        elem_b="shell",
        min_overlap=0.62,
        name="blower wheel spans most of the vented tower height",
    )
    ctx.expect_origin_distance(
        dial,
        tower,
        axes="xy",
        max_dist=0.001,
        name="selector dial stays centered on the tower top",
    )

    rest_aabb = ctx.part_element_world_aabb(tower, elem="grille")
    with ctx.pose({tower_oscillation: tower_oscillation.motion_limits.upper}):
        swung_aabb = ctx.part_element_world_aabb(tower, elem="grille")

    rest_x = None if rest_aabb is None else (rest_aabb[0][0] + rest_aabb[1][0]) * 0.5
    swung_x = None if swung_aabb is None else (swung_aabb[0][0] + swung_aabb[1][0]) * 0.5

    ctx.check(
        "tower oscillation swings the upper body sideways",
        rest_x is not None and swung_x is not None and abs(swung_x - rest_x) > 0.03,
        details=f"rest_x={rest_x}, swung_x={swung_x}",
    )

    return ctx.report()


object_model = build_object_model()
