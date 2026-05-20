from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dj_monitor_wedge_speaker")

    powder_coat = Material("powder_coat", rgba=(0.10, 0.105, 0.11, 1.0))
    cabinet_black = Material("cabinet_black", rgba=(0.025, 0.026, 0.028, 1.0))
    grille_black = Material("grille_black", rgba=(0.005, 0.006, 0.007, 1.0))
    rubber = Material("rubber", rgba=(0.012, 0.012, 0.012, 1.0))
    dark_foam = Material("dark_foam", rgba=(0.018, 0.018, 0.020, 1.0))
    steel = Material("brushed_steel", rgba=(0.55, 0.57, 0.58, 1.0))
    label_blue = Material("label_blue", rgba=(0.05, 0.16, 0.45, 1.0))

    pivot_z = 0.32

    bracket = model.part("tilt_bracket")
    bracket.visual(
        Box((0.50, 0.82, 0.032)),
        origin=Origin(xyz=(0.03, 0.0, 0.016)),
        material=powder_coat,
        name="base_plate",
    )
    bracket.visual(
        Box((0.20, 0.052, 0.34)),
        origin=Origin(xyz=(0.0, 0.38, 0.186)),
        material=powder_coat,
        name="side_cheek_0",
    )
    bracket.visual(
        Box((0.20, 0.052, 0.34)),
        origin=Origin(xyz=(0.0, -0.38, 0.186)),
        material=powder_coat,
        name="side_cheek_1",
    )
    bracket.visual(
        Box((0.065, 0.70, 0.060)),
        origin=Origin(xyz=(-0.12, 0.0, 0.055)),
        material=powder_coat,
        name="rear_bridge",
    )
    bracket.visual(
        Box((0.055, 0.70, 0.050)),
        origin=Origin(xyz=(0.205, 0.0, 0.050)),
        material=powder_coat,
        name="front_lip",
    )
    bracket.visual(
        Cylinder(radius=0.050, length=0.010),
        origin=Origin(xyz=(0.0, 0.411, pivot_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_boss_0",
    )
    bracket.visual(
        Cylinder(radius=0.050, length=0.010),
        origin=Origin(xyz=(0.0, -0.411, pivot_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_boss_1",
    )
    for i, (x, y) in enumerate(((-0.17, -0.33), (-0.17, 0.33), (0.22, -0.33), (0.22, 0.33))):
        bracket.visual(
            Cylinder(radius=0.032, length=0.012),
            origin=Origin(xyz=(x, y, 0.006)),
            material=rubber,
            name=f"rubber_foot_{i}",
        )

    cabinet = model.part("speaker_cabinet")
    cabinet.visual(
        Box((0.32, 0.62, 0.42)),
        origin=Origin(xyz=(0.08, 0.0, 0.03)),
        material=cabinet_black,
        name="cabinet_shell",
    )
    cabinet.visual(
        mesh_from_geometry(
            PerforatedPanelGeometry(
                (0.54, 0.33),
                0.006,
                hole_diameter=0.010,
                pitch=(0.018, 0.018),
                frame=0.018,
                corner_radius=0.010,
                stagger=True,
            ),
            "perforated_grille",
        ),
        origin=Origin(xyz=(0.244, 0.0, 0.035), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grille_black,
        name="front_grille",
    )
    cabinet.visual(
        Cylinder(radius=0.125, length=0.010),
        origin=Origin(xyz=(0.237, 0.0, -0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_foam,
        name="woofer_shadow",
    )
    cabinet.visual(
        Cylinder(radius=0.047, length=0.010),
        origin=Origin(xyz=(0.238, 0.0, 0.165), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_foam,
        name="tweeter_shadow",
    )
    cabinet.visual(
        Cylinder(radius=0.022, length=0.88),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_pin",
    )
    for i, y in enumerate((0.455, -0.455)):
        cabinet.visual(
            Cylinder(radius=0.046, length=0.030),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name=f"clamp_knob_{i}",
        )
    for i, y in enumerate((0.323, -0.323)):
        cabinet.visual(
            Box((0.090, 0.028, 0.075)),
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=steel,
            name=f"trunnion_plate_{i}",
        )
    for i, (y, z) in enumerate(((-0.245, 0.215), (0.245, 0.215), (-0.245, -0.155), (0.245, -0.155))):
        cabinet.visual(
            Box((0.030, 0.070, 0.070)),
            origin=Origin(xyz=(0.242, y, z)),
            material=rubber,
            name=f"corner_guard_{i}",
        )
    cabinet.visual(
        Box((0.012, 0.20, 0.045)),
        origin=Origin(xyz=(-0.084, 0.0, 0.205)),
        material=rubber,
        name="rear_handle_grip",
    )
    cabinet.visual(
        Box((0.018, 0.035, 0.090)),
        origin=Origin(xyz=(-0.084, 0.090, 0.170)),
        material=rubber,
        name="rear_handle_post_0",
    )
    cabinet.visual(
        Box((0.018, 0.035, 0.090)),
        origin=Origin(xyz=(-0.084, -0.090, 0.170)),
        material=rubber,
        name="rear_handle_post_1",
    )
    cabinet.visual(
        Box((0.010, 0.12, 0.055)),
        origin=Origin(xyz=(-0.082, 0.0, -0.105)),
        material=label_blue,
        name="rear_input_plate",
    )

    model.articulation(
        "bracket_to_cabinet",
        ArticulationType.REVOLUTE,
        parent=bracket,
        child=cabinet,
        origin=Origin(xyz=(0.0, 0.0, pivot_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.2, lower=-0.35, upper=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bracket = object_model.get_part("tilt_bracket")
    cabinet = object_model.get_part("speaker_cabinet")
    hinge = object_model.get_articulation("bracket_to_cabinet")

    ctx.allow_overlap(
        cabinet,
        bracket,
        elem_a="pivot_pin",
        elem_b="side_cheek_0",
        reason="The metal trunnion pin is intentionally captured through the bracket cheek bore proxy.",
    )
    ctx.allow_overlap(
        cabinet,
        bracket,
        elem_a="pivot_pin",
        elem_b="side_cheek_1",
        reason="The metal trunnion pin is intentionally captured through the bracket cheek bore proxy.",
    )
    ctx.allow_overlap(
        cabinet,
        bracket,
        elem_a="pivot_pin",
        elem_b="pivot_boss_0",
        reason="The bracket bushing is represented as a solid collar around the captured tilt pin.",
    )
    ctx.allow_overlap(
        cabinet,
        bracket,
        elem_a="pivot_pin",
        elem_b="pivot_boss_1",
        reason="The bracket bushing is represented as a solid collar around the captured tilt pin.",
    )
    ctx.expect_overlap(
        cabinet,
        bracket,
        axes="y",
        elem_a="pivot_pin",
        elem_b="side_cheek_0",
        min_overlap=0.030,
        name="pin passes through upper cheek",
    )
    ctx.expect_overlap(
        cabinet,
        bracket,
        axes="y",
        elem_a="pivot_pin",
        elem_b="side_cheek_1",
        min_overlap=0.030,
        name="pin passes through lower cheek",
    )
    ctx.expect_within(
        cabinet,
        bracket,
        axes="xz",
        inner_elem="pivot_pin",
        outer_elem="side_cheek_0",
        margin=0.001,
        name="pin centered in upper cheek bore",
    )
    ctx.expect_within(
        cabinet,
        bracket,
        axes="xz",
        inner_elem="pivot_pin",
        outer_elem="side_cheek_1",
        margin=0.001,
        name="pin centered in lower cheek bore",
    )
    ctx.expect_within(
        cabinet,
        bracket,
        axes="xz",
        inner_elem="pivot_pin",
        outer_elem="pivot_boss_0",
        margin=0.001,
        name="pin centered in first bushing",
    )
    ctx.expect_within(
        cabinet,
        bracket,
        axes="xz",
        inner_elem="pivot_pin",
        outer_elem="pivot_boss_1",
        margin=0.001,
        name="pin centered in second bushing",
    )

    axis = getattr(hinge, "axis", None)
    ctx.check(
        "tilt hinge uses a horizontal axis",
        axis is not None and abs(axis[1]) > 0.99 and abs(axis[0]) < 0.01 and abs(axis[2]) < 0.01,
        details=f"axis={axis}",
    )

    rest_aabb = ctx.part_element_world_aabb(cabinet, elem="front_grille")
    with ctx.pose({hinge: 0.35}):
        raised_aabb = ctx.part_element_world_aabb(cabinet, elem="front_grille")

    def _center_z(aabb):
        return (aabb[0][2] + aabb[1][2]) * 0.5 if aabb is not None else None

    rest_z = _center_z(rest_aabb)
    raised_z = _center_z(raised_aabb)
    ctx.check(
        "positive tilt raises speaker face",
        rest_z is not None and raised_z is not None and raised_z > rest_z + 0.035,
        details=f"rest_z={rest_z}, raised_z={raised_z}",
    )

    return ctx.report()


object_model = build_object_model()
