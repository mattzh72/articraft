from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    boolean_difference,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _merge(geometries: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _ring_band(
    *,
    outer_radius: float,
    inner_radius: float,
    height: float,
    radial_segments: int = 96,
) -> MeshGeometry:
    """A centered annular band with a real through bore."""
    outer = CylinderGeometry(
        radius=outer_radius,
        height=height,
        radial_segments=radial_segments,
    )
    cutter = CylinderGeometry(
        radius=inner_radius,
        height=height + 0.006,
        radial_segments=radial_segments,
    )
    return boolean_difference(outer, cutter)


def _stage_mesh(
    *,
    outer_radius: float,
    inner_radius: float,
    body_height: float,
    rim_tube: float,
    marker_width: float,
) -> MeshGeometry:
    """Rotary level with a through-hole, raised rims, and one orientation lug."""
    body = _ring_band(
        outer_radius=outer_radius,
        inner_radius=inner_radius,
        height=body_height,
    )
    top_rim = TorusGeometry(
        radius=outer_radius - rim_tube,
        tube=rim_tube,
        radial_segments=18,
        tubular_segments=112,
    ).translate(0.0, 0.0, body_height / 2.0 - rim_tube * 0.20)
    inner_bead = TorusGeometry(
        radius=inner_radius + rim_tube * 0.85,
        tube=rim_tube * 0.55,
        radial_segments=14,
        tubular_segments=80,
    ).translate(0.0, 0.0, body_height / 2.0 - rim_tube * 0.25)

    # A shallow radial index lug makes each independent rotation visibly legible.
    lug = CylinderGeometry(
        radius=marker_width,
        height=0.018,
        radial_segments=4,
    ).scale(1.0, 0.42, 1.0)
    lug.translate(outer_radius * 0.70, 0.0, body_height / 2.0 + 0.002)

    return _merge([body, top_rim, inner_bead, lug])


def _housing_mesh() -> MeshGeometry:
    """Wide grounded saddle body: rounded plinth plus raised side saddles."""
    base = ExtrudeGeometry.from_z0(
        rounded_rect_profile(1.22, 0.72, 0.08, corner_segments=10),
        0.150,
    )
    left_saddle = ExtrudeGeometry.from_z0(
        rounded_rect_profile(0.18, 0.58, 0.035, corner_segments=8),
        0.070,
    ).translate(-0.330, 0.0, 0.148)
    right_saddle = ExtrudeGeometry.from_z0(
        rounded_rect_profile(0.18, 0.58, 0.035, corner_segments=8),
        0.070,
    ).translate(0.330, 0.0, 0.148)
    front_foot = ExtrudeGeometry.from_z0(
        rounded_rect_profile(0.92, 0.075, 0.025, corner_segments=6),
        0.030,
    ).translate(0.0, 0.385, 0.0)
    rear_foot = ExtrudeGeometry.from_z0(
        rounded_rect_profile(0.92, 0.075, 0.025, corner_segments=6),
        0.030,
    ).translate(0.0, -0.385, 0.0)
    return _merge([base, left_saddle, right_saddle, front_foot, rear_foot])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="saddle_body_coaxial_stack")

    housing_paint = model.material("housing_paint", rgba=(0.13, 0.15, 0.17, 1.0))
    bearing_black = model.material("bearing_black", rgba=(0.02, 0.023, 0.025, 1.0))
    lower_blue = model.material("lower_blue", rgba=(0.08, 0.30, 0.55, 1.0))
    middle_steel = model.material("middle_steel", rgba=(0.62, 0.65, 0.68, 1.0))
    top_bronze = model.material("top_bronze", rgba=(0.76, 0.55, 0.28, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_geometry(_housing_mesh(), "saddle_housing"),
        material=housing_paint,
        name="saddle_body",
    )
    housing.visual(
        Cylinder(radius=0.055, length=0.363),
        origin=Origin(xyz=(0.0, 0.0, 0.3315)),
        material=bearing_black,
        name="fixed_spindle",
    )
    housing.visual(
        Cylinder(radius=0.185, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.219)),
        material=bearing_black,
        name="lower_bearing_race",
    )
    housing.visual(
        Cylinder(radius=0.142, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.329)),
        material=bearing_black,
        name="middle_bearing_race",
    )
    housing.visual(
        Cylinder(radius=0.107, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.421)),
        material=bearing_black,
        name="upper_bearing_race",
    )
    housing.visual(
        Cylinder(radius=0.084, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.5012)),
        material=bearing_black,
        name="retainer_cap",
    )

    lower_stage = model.part("lower_stage")
    lower_stage.visual(
        mesh_from_geometry(
            _stage_mesh(
                outer_radius=0.420,
                inner_radius=0.074,
                body_height=0.084,
                rim_tube=0.010,
                marker_width=0.038,
            ),
            "lower_stage_shell",
        ),
        material=lower_blue,
        name="lower_rotor",
    )

    middle_stage = model.part("middle_stage")
    middle_stage.visual(
        mesh_from_geometry(
            _stage_mesh(
                outer_radius=0.300,
                inner_radius=0.070,
                body_height=0.068,
                rim_tube=0.008,
                marker_width=0.030,
            ),
            "middle_stage_shell",
        ),
        material=middle_steel,
        name="middle_rotor",
    )

    top_stage = model.part("top_stage")
    top_stage.visual(
        mesh_from_geometry(
            _stage_mesh(
                outer_radius=0.205,
                inner_radius=0.068,
                body_height=0.058,
                rim_tube=0.0065,
                marker_width=0.024,
            ),
            "top_stage_shell",
        ),
        material=top_bronze,
        name="top_rotor",
    )

    model.articulation(
        "housing_to_lower_stage",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=lower_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.270)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=85.0, velocity=1.2, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "housing_to_middle_stage",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=middle_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.369)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=55.0, velocity=1.5, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "housing_to_top_stage",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=top_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.455)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.8, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    lower_stage = object_model.get_part("lower_stage")
    middle_stage = object_model.get_part("middle_stage")
    top_stage = object_model.get_part("top_stage")
    lower_joint = object_model.get_articulation("housing_to_lower_stage")
    middle_joint = object_model.get_articulation("housing_to_middle_stage")
    top_joint = object_model.get_articulation("housing_to_top_stage")

    ctx.expect_gap(
        lower_stage,
        housing,
        axis="z",
        max_gap=0.001,
        max_penetration=0.00001,
        positive_elem="lower_rotor",
        negative_elem="lower_bearing_race",
        name="lower stage rests on lower bearing",
    )
    ctx.expect_gap(
        middle_stage,
        housing,
        axis="z",
        max_gap=0.001,
        max_penetration=0.00001,
        positive_elem="middle_rotor",
        negative_elem="middle_bearing_race",
        name="middle stage rests on middle bearing",
    )
    ctx.expect_gap(
        top_stage,
        housing,
        axis="z",
        max_gap=0.001,
        max_penetration=0.00001,
        positive_elem="top_rotor",
        negative_elem="upper_bearing_race",
        name="top stage rests on upper bearing",
    )
    for stage, elem in (
        (lower_stage, "lower_rotor"),
        (middle_stage, "middle_rotor"),
        (top_stage, "top_rotor"),
    ):
        ctx.expect_overlap(
            stage,
            housing,
            axes="xy",
            min_overlap=0.090,
            elem_a=elem,
            elem_b="fixed_spindle",
            name=f"{elem} remains coaxial with spindle",
        )

    lower_rest = ctx.part_world_position(lower_stage)
    middle_rest = ctx.part_world_position(middle_stage)
    top_rest = ctx.part_world_position(top_stage)
    with ctx.pose({lower_joint: 0.65, middle_joint: -0.90, top_joint: 1.20}):
        lower_moved = ctx.part_world_position(lower_stage)
        middle_moved = ctx.part_world_position(middle_stage)
        top_moved = ctx.part_world_position(top_stage)
        ctx.expect_gap(
            middle_stage,
            lower_stage,
            axis="z",
            min_gap=0.006,
            name="rotated middle stage stays separated from lower stage",
        )
        ctx.expect_gap(
            top_stage,
            middle_stage,
            axis="z",
            min_gap=0.004,
            name="rotated top stage stays separated from middle stage",
        )
    ctx.check(
        "independent revolute origins stay on shared vertical line",
        all(
            pos is not None and abs(pos[0]) < 1e-6 and abs(pos[1]) < 1e-6
            for pos in (
                lower_rest,
                middle_rest,
                top_rest,
                lower_moved,
                middle_moved,
                top_moved,
            )
        ),
    )

    return ctx.report()


object_model = build_object_model()
