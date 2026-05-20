from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
)


SHAFT_Z = 0.50
PILLOW_X = (-0.28, 0.32)


def _circle_profile(
    radius: float,
    *,
    x: float = 0.0,
    y: float = 0.0,
    segments: int = 48,
) -> list[tuple[float, float]]:
    profile = superellipse_profile(
        radius * 2.0,
        radius * 2.0,
        exponent=2.0,
        segments=segments,
    )
    return [(px + x, py + y) for px, py in profile]


def _pillow_housing_mesh(name: str):
    """Upright bearing housing with a real through-bore along the shaft axis."""
    outer = rounded_rect_profile(0.58, 0.34, 0.075, corner_segments=10)
    bore = _circle_profile(0.076, segments=56)
    geom = ExtrudeWithHolesGeometry(outer, [bore], height=0.20, center=True)
    return mesh_from_geometry(geom.rotate_y(pi / 2.0), name)


def _annulus_mesh(name: str, outer_radius: float, inner_radius: float, width: float):
    geom = ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius, segments=64),
        [_circle_profile(inner_radius, segments=48)],
        height=width,
        center=True,
    )
    return mesh_from_geometry(geom.rotate_y(pi / 2.0), name)


def _flange_mesh(name: str):
    work_holes = [_circle_profile(0.043, segments=40)]
    for index in range(6):
        angle = 2.0 * pi * index / 6.0
        # Profile X maps to world -Z after the 90 degree Y rotation; profile Y
        # maps to world Y.  This keeps the hole pattern on the Y/Z face.
        work_holes.append(
            _circle_profile(
                0.015,
                x=-sin(angle) * 0.155,
                y=cos(angle) * 0.155,
                segments=24,
            )
        )
    geom = ExtrudeWithHolesGeometry(
        _circle_profile(0.245, segments=96),
        work_holes,
        height=0.075,
        center=True,
    )
    return mesh_from_geometry(geom.rotate_y(pi / 2.0), name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="horizontal_spin_fixture")

    painted_steel = model.material("painted_steel", rgba=(0.16, 0.19, 0.21, 1.0))
    dark_casting = model.material("dark_casting", rgba=(0.08, 0.09, 0.10, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.62, 0.64, 0.66, 1.0))
    bearing_bronze = model.material("bearing_bronze", rgba=(0.72, 0.55, 0.28, 1.0))
    black_oxide = model.material("black_oxide", rgba=(0.02, 0.02, 0.025, 1.0))
    face_blue = model.material("face_blue", rgba=(0.10, 0.22, 0.36, 1.0))

    flange_mesh = _flange_mesh("workholding_flange")

    frame = model.part("frame")
    frame.visual(
        Box((1.36, 0.54, 0.08)),
        origin=Origin(xyz=(0.02, 0.0, 0.04)),
        material=painted_steel,
        name="base_plate",
    )
    frame.visual(
        Box((1.44, 0.065, 0.055)),
        origin=Origin(xyz=(0.02, -0.205, 0.105)),
        material=dark_casting,
        name="rail_0",
    )
    frame.visual(
        Box((1.44, 0.065, 0.055)),
        origin=Origin(xyz=(0.02, 0.205, 0.105)),
        material=dark_casting,
        name="rail_1",
    )
    for cross_i, x in enumerate((-0.49, 0.02, 0.53)):
        frame.visual(
            Box((0.075, 0.46, 0.050)),
            origin=Origin(xyz=(x, 0.0, 0.125)),
            material=dark_casting,
            name=f"crossmember_{cross_i}",
        )

    for foot_i, (x, y) in enumerate(
        ((-0.58, -0.205), (-0.58, 0.205), (0.62, -0.205), (0.62, 0.205))
    ):
        frame.visual(
            Box((0.21, 0.12, 0.040)),
            origin=Origin(xyz=(x, y, 0.118)),
            material=painted_steel,
            name=f"foot_{foot_i}",
        )
        frame.visual(
            Cylinder(radius=0.014, length=0.018),
            origin=Origin(xyz=(x, y, 0.147)),
            material=black_oxide,
            name=f"anchor_bolt_{foot_i}",
        )

    frame.visual(
        Box((0.24, 0.32, 0.12)),
        origin=Origin(xyz=(PILLOW_X[0], 0.0, SHAFT_Z + 0.13)),
        material=dark_casting,
        name="pillow_0_housing",
    )
    frame.visual(
        Box((0.24, 0.32, 0.12)),
        origin=Origin(xyz=(PILLOW_X[1], 0.0, SHAFT_Z + 0.13)),
        material=dark_casting,
        name="pillow_1_housing",
    )

    for idx, x in enumerate(PILLOW_X):
        frame.visual(
            Box((0.32, 0.43, 0.060)),
            origin=Origin(xyz=(x, 0.0, 0.112)),
            material=painted_steel,
            name=f"pillow_{idx}_foot",
        )
        frame.visual(
            Box((0.21, 0.23, 0.230)),
            origin=Origin(xyz=(x, 0.0, 0.245)),
            material=dark_casting,
            name=f"pillow_{idx}_pedestal",
        )
        frame.visual(
            Box((0.24, 0.32, 0.12)),
            origin=Origin(xyz=(x, 0.0, SHAFT_Z - 0.13)),
            material=dark_casting,
            name=f"pillow_{idx}_lower_saddle",
        )
        for side_i, y in enumerate((-0.147, 0.147)):
            frame.visual(
                Box((0.24, 0.055, 0.26)),
                origin=Origin(xyz=(x, y, SHAFT_Z)),
                material=dark_casting,
                name=f"pillow_{idx}_side_cheek_{side_i}",
            )
        for face_i, sx in enumerate((-0.124, 0.124)):
            frame.visual(
                Box((0.014, 0.13, 0.026)),
                origin=Origin(xyz=(x + sx, 0.0, SHAFT_Z + 0.048)),
                material=bearing_bronze,
                name=f"pillow_{idx}_upper_bearing_{face_i}",
            )
            frame.visual(
                Box((0.014, 0.13, 0.026)),
                origin=Origin(xyz=(x + sx, 0.0, SHAFT_Z - 0.048)),
                material=bearing_bronze,
                name=f"pillow_{idx}_lower_bearing_{face_i}",
            )
            for side_i, y in enumerate((-0.080, 0.080)):
                frame.visual(
                    Box((0.014, 0.090, 0.13)),
                    origin=Origin(xyz=(x + sx, y, SHAFT_Z)),
                    material=bearing_bronze,
                    name=f"pillow_{idx}_side_bearing_{face_i}_{side_i}",
                )
        frame.visual(
            Box((0.24, 0.31, 0.052)),
            origin=Origin(xyz=(x, 0.0, SHAFT_Z + 0.216)),
            material=dark_casting,
            name=f"pillow_{idx}_cap",
        )
        for bolt_i, y in enumerate((-0.105, 0.105)):
            frame.visual(
                Cylinder(radius=0.015, length=0.030),
                origin=Origin(xyz=(x - 0.055, y, SHAFT_Z + 0.257)),
                material=black_oxide,
                name=f"pillow_{idx}_cap_bolt_{bolt_i}",
            )
            frame.visual(
                Cylinder(radius=0.015, length=0.030),
                origin=Origin(xyz=(x + 0.055, y, SHAFT_Z + 0.257)),
                material=black_oxide,
                name=f"pillow_{idx}_cap_bolt_{bolt_i + 2}",
            )
        frame.visual(
            Box((0.006, 0.30, 0.006)),
            origin=Origin(xyz=(x - 0.123, 0.0, SHAFT_Z + 0.070)),
            material=black_oxide,
            name=f"pillow_{idx}_split_line",
        )

    frame.inertial = Inertial.from_geometry(
        Box((1.45, 0.55, 0.82)),
        mass=85.0,
        origin=Origin(xyz=(0.02, 0.0, 0.41)),
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=0.035, length=1.22),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=machined_steel,
        name="shaft",
    )
    spindle.visual(
        flange_mesh,
        origin=Origin(xyz=(-0.50, 0.0, 0.0)),
        material=face_blue,
        name="flange",
    )
    spindle.visual(
        Cylinder(radius=0.078, length=0.074),
        origin=Origin(xyz=(-0.450, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=machined_steel,
        name="inboard_hub",
    )
    spindle.visual(
        Cylinder(radius=0.067, length=0.064),
        origin=Origin(xyz=(-0.553, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=machined_steel,
        name="outboard_hub",
    )
    spindle.visual(
        Box((0.075, 0.022, 0.012)),
        origin=Origin(xyz=(-0.455, 0.0, 0.081)),
        material=black_oxide,
        name="hub_key",
    )
    spindle.visual(
        Cylinder(radius=0.054, length=0.040),
        origin=Origin(xyz=(0.525, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=machined_steel,
        name="end_collar",
    )
    spindle.visual(
        Box((0.060, 0.018, 0.010)),
        origin=Origin(xyz=(0.525, 0.0, 0.059)),
        material=black_oxide,
        name="collar_key",
    )
    for index in range(6):
        angle = 2.0 * pi * index / 6.0
        spindle.visual(
            Cylinder(radius=0.013, length=0.018),
            origin=Origin(
                xyz=(-0.539, cos(angle) * 0.155, sin(angle) * 0.155),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=black_oxide,
            name=f"flange_bolt_{index}",
        )
    spindle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.245, length=1.22),
        mass=18.0,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    model.articulation(
        "shaft_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=spindle,
        origin=Origin(xyz=(0.0, 0.0, SHAFT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=18.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    spindle = object_model.get_part("spindle")
    spin = object_model.get_articulation("shaft_spin")

    ctx.check(
        "single continuous shaft joint",
        len(object_model.articulations) == 1
        and spin.child == "spindle"
        and spin.parent == "frame"
        and tuple(spin.axis) == (1.0, 0.0, 0.0),
        details=f"articulations={object_model.articulations}",
    )
    ctx.expect_origin_gap(
        spindle,
        frame,
        axis="z",
        min_gap=SHAFT_Z - 0.001,
        max_gap=SHAFT_Z + 0.001,
        name="shaft axis height is carried above the base",
    )
    ctx.expect_overlap(
        spindle,
        frame,
        axes="x",
        elem_a="shaft",
        elem_b="pillow_0_housing",
        min_overlap=0.18,
        name="shaft is supported through pillow block 0",
    )
    ctx.expect_overlap(
        spindle,
        frame,
        axes="x",
        elem_a="shaft",
        elem_b="pillow_1_housing",
        min_overlap=0.18,
        name="shaft is supported through pillow block 1",
    )
    ctx.expect_gap(
        frame,
        spindle,
        axis="x",
        positive_elem="pillow_0_housing",
        negative_elem="flange",
        min_gap=0.035,
        name="flange clears the nearest pillow block",
    )
    ctx.expect_gap(
        spindle,
        frame,
        axis="z",
        positive_elem="flange",
        negative_elem="rail_0",
        min_gap=0.10,
        name="flange sweep clears the bed rail below",
    )

    flange_aabb = ctx.part_element_world_aabb(spindle, elem="flange")
    if flange_aabb is not None:
        flange_min, flange_max = flange_aabb
        ctx.check(
            "flange is a large workholding face",
            (flange_max[1] - flange_min[1]) > 0.45
            and (flange_max[2] - flange_min[2]) > 0.45,
            details=f"flange_aabb={flange_aabb}",
        )

    rest_pos = ctx.part_world_position(spindle)
    with ctx.pose({spin: pi / 2.0}):
        quarter_turn_pos = ctx.part_world_position(spindle)
        ctx.expect_gap(
            frame,
            spindle,
            axis="x",
            positive_elem="pillow_0_housing",
            negative_elem="flange",
            min_gap=0.035,
            name="flange still clears pillow block after rotation",
        )
    ctx.check(
        "spin keeps shaft origin fixed",
        rest_pos is not None
        and quarter_turn_pos is not None
        and all(abs(a - b) < 1e-7 for a, b in zip(rest_pos, quarter_turn_pos)),
        details=f"rest={rest_pos}, quarter_turn={quarter_turn_pos}",
    )

    return ctx.report()


object_model = build_object_model()
