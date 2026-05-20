from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="record_turntable")

    walnut = model.material("walnut", rgba=(0.41, 0.28, 0.19, 1.0))
    charcoal = model.material("charcoal", rgba=(0.14, 0.15, 0.16, 1.0))
    black = model.material("black", rgba=(0.07, 0.07, 0.08, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.20, 0.21, 0.23, 1.0))
    aluminum = model.material("aluminum", rgba=(0.78, 0.79, 0.81, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))

    top_deck_mesh = _mesh(
        "turntable_top_deck",
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(0.46, 0.35, 0.022, corner_segments=10),
            0.032,
            cap=True,
            closed=True,
        ),
    )
    lower_body_mesh = _mesh(
        "turntable_lower_body",
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(0.43, 0.32, 0.018, corner_segments=8),
            0.018,
            cap=True,
            closed=True,
        ),
    )
    tonearm_tube_mesh = _mesh(
        "tonearm_tube",
        tube_from_spline_points(
            [
                (0.0, 0.0, 0.012),
                (0.028, 0.0, 0.012),
                (0.095, 0.003, 0.011),
                (0.155, 0.007, 0.009),
                (0.188, 0.011, 0.006),
            ],
            radius=0.0055,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        ),
    )

    plinth = model.part("plinth")
    plinth.visual(
        lower_body_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=walnut,
        name="lower_body",
    )
    plinth.visual(
        top_deck_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=charcoal,
        name="top_deck",
    )
    for index, (x_pos, y_pos) in enumerate(
        [
            (-0.175, -0.125),
            (-0.175, 0.125),
            (0.175, -0.125),
            (0.175, 0.125),
        ]
    ):
        plinth.visual(
            Cylinder(radius=0.016, length=0.010),
            origin=Origin(xyz=(x_pos, y_pos, 0.005)),
            material=rubber,
            name=f"foot_{index}",
        )
    plinth.visual(
        Cylinder(radius=0.040, length=0.026),
        origin=Origin(xyz=(-0.030, 0.0, 0.027)),
        material=dark_metal,
        name="bearing_housing",
    )
    plinth.visual(
        Cylinder(radius=0.021, length=0.024),
        origin=Origin(xyz=(0.155, 0.087, 0.052)),
        material=dark_metal,
        name="tonearm_base",
    )
    plinth.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(0.175, -0.118, 0.043)),
        material=aluminum,
        name="speed_knob",
    )
    plinth.inertial = Inertial.from_geometry(
        Box((0.46, 0.35, 0.040)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )

    platter = model.part("platter")
    platter.visual(
        Cylinder(radius=0.148, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=dark_metal,
        name="platter_body",
    )
    platter.visual(
        Cylinder(radius=0.028, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=dark_metal,
        name="support_hub",
    )
    platter.visual(
        Cylinder(radius=0.145, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=black,
        name="record_mat",
    )
    platter.visual(
        Cylinder(radius=0.004, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=aluminum,
        name="spindle",
    )
    platter.inertial = Inertial.from_geometry(
        Cylinder(radius=0.148, length=0.018),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    tonearm = model.part("tonearm")
    tonearm.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=dark_metal,
        name="pivot_collar",
    )
    tonearm.visual(
        Cylinder(radius=0.007, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=dark_metal,
        name="bearing_cap",
    )
    tonearm.visual(
        tonearm_tube_mesh,
        material=aluminum,
        name="arm_tube",
    )
    tonearm.visual(
        Box((0.034, 0.016, 0.006)),
        origin=Origin(xyz=(0.203, 0.012, 0.005)),
        material=charcoal,
        name="headshell",
    )
    tonearm.visual(
        Box((0.013, 0.011, 0.006)),
        origin=Origin(xyz=(0.221, 0.013, 0.002)),
        material=black,
        name="cartridge",
    )
    tonearm.visual(
        Cylinder(radius=0.0045, length=0.034),
        origin=Origin(xyz=(-0.017, 0.0, 0.013), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="counterweight_stub",
    )
    tonearm.visual(
        Cylinder(radius=0.011, length=0.022),
        origin=Origin(xyz=(-0.045, 0.0, 0.013), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="counterweight",
    )
    tonearm.inertial = Inertial.from_geometry(
        Box((0.28, 0.05, 0.05)),
        mass=0.35,
        origin=Origin(xyz=(0.080, 0.0, 0.014)),
    )

    model.articulation(
        "platter_spin",
        ArticulationType.CONTINUOUS,
        parent=plinth,
        child=platter,
        origin=Origin(xyz=(-0.030, 0.0, 0.040)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=12.0,
        ),
    )
    model.articulation(
        "tonearm_swing",
        ArticulationType.REVOLUTE,
        parent=plinth,
        child=tonearm,
        origin=Origin(xyz=(0.155, 0.087, 0.064), rpy=(0.0, 0.0, -1.18)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    plinth = object_model.get_part("plinth")
    platter = object_model.get_part("platter")
    tonearm = object_model.get_part("tonearm")
    platter_spin = object_model.get_articulation("platter_spin")
    tonearm_swing = object_model.get_articulation("tonearm_swing")

    ctx.check(
        "platter uses continuous vertical spin",
        platter_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(platter_spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={platter_spin.articulation_type}, axis={platter_spin.axis}",
    )
    ctx.check(
        "tonearm swings on a vertical pivot",
        tonearm_swing.articulation_type == ArticulationType.REVOLUTE
        and tuple(tonearm_swing.axis) == (0.0, 0.0, -1.0),
        details=f"type={tonearm_swing.articulation_type}, axis={tonearm_swing.axis}",
    )

    ctx.expect_gap(
        platter,
        plinth,
        axis="z",
        positive_elem="platter_body",
        negative_elem="top_deck",
        min_gap=0.001,
        max_gap=0.003,
        name="platter rides just above the top deck",
    )
    ctx.expect_overlap(
        platter,
        plinth,
        axes="xy",
        elem_a="platter_body",
        elem_b="top_deck",
        min_overlap=0.28,
        name="platter stays fully supported inside the plinth footprint",
    )
    ctx.expect_gap(
        tonearm,
        platter,
        axis="x",
        positive_elem="pivot_collar",
        negative_elem="platter_body",
        min_gap=0.015,
        name="tonearm base sits off to the side of the platter",
    )

    def _center_of_aabb(aabb):
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3)) if aabb else None

    rest_headshell = ctx.part_element_world_aabb(tonearm, elem="headshell")
    platter_body = ctx.part_element_world_aabb(platter, elem="platter_body")
    with ctx.pose({tonearm_swing: 1.20}):
        swung_headshell = ctx.part_element_world_aabb(tonearm, elem="headshell")

    rest_center = _center_of_aabb(rest_headshell)
    swung_center = _center_of_aabb(swung_headshell)
    platter_center = _center_of_aabb(platter_body)
    rest_distance = None
    swung_distance = None
    if rest_center and swung_center and platter_center:
        rest_distance = math.hypot(
            rest_center[0] - platter_center[0],
            rest_center[1] - platter_center[1],
        )
        swung_distance = math.hypot(
            swung_center[0] - platter_center[0],
            swung_center[1] - platter_center[1],
        )

    ctx.check(
        "tonearm can swing inward across the record area",
        rest_center is not None
        and swung_center is not None
        and rest_distance is not None
        and swung_distance is not None
        and swung_center[0] < rest_center[0] - 0.10
        and swung_distance < rest_distance - 0.08,
        details=(
            f"rest_center={rest_center}, swung_center={swung_center}, "
            f"rest_distance={rest_distance}, swung_distance={swung_distance}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
