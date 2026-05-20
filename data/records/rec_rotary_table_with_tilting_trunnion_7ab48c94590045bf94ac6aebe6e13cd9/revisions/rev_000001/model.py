from __future__ import annotations

from math import cos, pi, sin

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
import cadquery as cq


AXIS_X = 0.240
AXIS_Z = 0.215


def _box_at(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cylinder_z(radius: float, length: float, center_z: float) -> cq.Workplane:
    return cq.Workplane("XY").cylinder(length, radius).translate((0.0, 0.0, center_z))


def _cylinder_x(radius: float, length: float, center_x: float = 0.0) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .cylinder(length, radius)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate((center_x, 0.0, 0.0))
    )


def _cylinder_y(
    radius: float,
    length: float,
    center: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .cylinder(length, radius)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate(center)
    )


def _ring_x(outer: float, inner: float, length: float, center_x: float = 0.0) -> cq.Workplane:
    ring = _cylinder_x(outer, length, center_x)
    return ring.cut(_cylinder_x(inner, length + 0.004, center_x))


def _ring_y(
    outer: float,
    inner: float,
    length: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    ring = _cylinder_y(outer, length, center)
    return ring.cut(_cylinder_y(inner, length + 0.004, center))


def _turntable_geometry() -> cq.Workplane:
    table = _cylinder_z(0.170, 0.040, 0.020)
    center_bore = _cylinder_z(0.049, 0.070, 0.020)
    return table.cut(center_bore)


def _cradle_frame_geometry() -> cq.Workplane:
    pad = _box_at((0.265, 0.240, 0.040), (0.195, 0.0, 0.058))
    plate_pos = _box_at((0.175, 0.026, 0.290), (AXIS_X, 0.132, AXIS_Z))
    plate_neg = _box_at((0.175, 0.026, 0.290), (AXIS_X, -0.132, AXIS_Z))
    rear_web = _box_at((0.050, 0.230, 0.155), (0.135, 0.0, 0.145))
    lower_bridge = _box_at((0.120, 0.285, 0.034), (0.220, 0.0, 0.085))
    top_bridge = _box_at((0.140, 0.295, 0.026), (0.232, 0.0, 0.355))
    rib_pos = _box_at((0.225, 0.026, 0.045), (0.170, 0.077, 0.073))
    rib_neg = _box_at((0.225, 0.026, 0.045), (0.170, -0.077, 0.073))

    frame = (
        pad.union(plate_pos)
        .union(plate_neg)
        .union(rear_web)
        .union(lower_bridge)
        .union(top_bridge)
        .union(rib_pos)
        .union(rib_neg)
    )
    trunnion_bore = _cylinder_y(0.034, 0.360, (AXIS_X, 0.0, AXIS_Z))
    return frame.cut(trunnion_bore)


def _face_disk_geometry() -> cq.Workplane:
    disk = _cylinder_x(0.104, 0.032, 0.0)
    clamp_ring = _ring_x(0.095, 0.069, 0.010, 0.020)
    front_boss = _cylinder_x(0.036, 0.056, 0.004)
    rear_boss = _cylinder_x(0.045, 0.020, -0.025)
    nose_bore = _cylinder_x(0.016, 0.065, 0.010)
    return disk.union(clamp_ring).union(front_boss).union(rear_boss).cut(nose_bore)


def _trunnion_geometry() -> cq.Workplane:
    shaft = _cylinder_y(0.0252, 0.330)
    collar_pos = _cylinder_y(0.026, 0.018, (0.0, 0.104, 0.0))
    collar_neg = _cylinder_y(0.026, 0.018, (0.0, -0.104, 0.0))
    cap_pos = _cylinder_y(0.020, 0.014, (0.0, 0.166, 0.0))
    cap_neg = _cylinder_y(0.020, 0.014, (0.0, -0.166, 0.0))
    stop_dog = _box_at((0.024, 0.020, 0.074), (0.0, 0.174, 0.052))
    return shaft.union(collar_pos).union(collar_neg).union(cap_pos).union(cap_neg).union(stop_dog)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_cradle_indexer")

    cast_iron = model.material("dark_cast_iron", rgba=(0.08, 0.085, 0.09, 1.0))
    blue_paint = model.material("machined_blue_paint", rgba=(0.08, 0.18, 0.28, 1.0))
    ground_steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    black_oxide = model.material("black_oxide", rgba=(0.015, 0.014, 0.013, 1.0))
    brass = model.material("worn_brass", rgba=(0.83, 0.62, 0.25, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.225, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=cast_iron,
        name="base_foot",
    )
    base.visual(
        mesh_from_cadquery(
            _cylinder_z(0.176, 0.018, 0.063).cut(_cylinder_z(0.118, 0.024, 0.063)),
            "bearing_race",
            tolerance=0.0007,
        ),
        material=ground_steel,
        name="bearing_race",
    )
    base.visual(
        Cylinder(radius=0.034, length=0.104),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material=ground_steel,
        name="center_hub",
    )
    for idx, angle in enumerate((0.0, pi / 2.0, pi, 3.0 * pi / 2.0)):
        base.visual(
            Box((0.040, 0.009, 0.006)),
            origin=Origin(
                xyz=(0.190 * cos(angle), 0.190 * sin(angle), 0.055),
                rpy=(0.0, 0.0, angle),
            ),
            material=brass,
            name=f"index_mark_{idx}",
        )

    lower_stage = model.part("lower_stage")
    lower_stage.visual(
        mesh_from_cadquery(_turntable_geometry(), "turntable_table", tolerance=0.0008),
        material=ground_steel,
        name="turntable_table",
    )
    lower_stage.visual(
        mesh_from_cadquery(_cradle_frame_geometry(), "cradle_frame", tolerance=0.0008),
        material=blue_paint,
        name="cradle_frame",
    )
    lower_stage.visual(
        mesh_from_cadquery(
            _ring_y(0.050, 0.025, 0.014, (AXIS_X, 0.151, AXIS_Z)),
            "bearing_cover_pos",
            tolerance=0.0006,
        ),
        material=ground_steel,
        name="bearing_cover_pos",
    )
    lower_stage.visual(
        mesh_from_cadquery(
            _ring_y(0.050, 0.025, 0.014, (AXIS_X, -0.151, AXIS_Z)),
            "bearing_cover_neg",
            tolerance=0.0006,
        ),
        material=ground_steel,
        name="bearing_cover_neg",
    )
    for name, x_off, z_off in (
        ("tilt_stop_0", -0.048, 0.088),
        ("tilt_stop_1", 0.052, -0.084),
    ):
        lower_stage.visual(
            Box((0.034, 0.018, 0.024)),
            origin=Origin(xyz=(AXIS_X + x_off, 0.150, AXIS_Z + z_off)),
            material=black_oxide,
            name=name,
        )

    faceplate = model.part("faceplate")
    faceplate.visual(
        mesh_from_cadquery(_face_disk_geometry(), "face_disk", tolerance=0.0007),
        material=ground_steel,
        name="face_disk",
    )
    faceplate.visual(
        mesh_from_cadquery(_trunnion_geometry(), "trunnion", tolerance=0.0007),
        material=ground_steel,
        name="trunnion",
    )
    for idx, angle in enumerate(tuple(i * pi / 3.0 for i in range(6))):
        faceplate.visual(
            Cylinder(radius=0.0055, length=0.008),
            origin=Origin(
                xyz=(0.0275, 0.082 * cos(angle), 0.082 * sin(angle)),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=black_oxide,
            name=f"clamp_bolt_{idx}",
        )

    model.articulation(
        "base_to_lower_stage",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.072)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2, lower=-pi, upper=pi),
    )
    model.articulation(
        "lower_stage_to_faceplate",
        ArticulationType.REVOLUTE,
        parent=lower_stage,
        child=faceplate,
        origin=Origin(xyz=(AXIS_X, 0.0, AXIS_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=0.8, lower=-0.55, upper=0.55),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lower_stage = object_model.get_part("lower_stage")
    faceplate = object_model.get_part("faceplate")
    yaw = object_model.get_articulation("base_to_lower_stage")
    tilt = object_model.get_articulation("lower_stage_to_faceplate")

    def _coord(point, index: int) -> float:
        try:
            return float(point[index])
        except TypeError:
            return float((point.x, point.y, point.z)[index])

    def _span(aabb, index: int) -> float:
        return _coord(aabb[1], index) - _coord(aabb[0], index)

    def _center(aabb, index: int) -> float:
        return 0.5 * (_coord(aabb[0], index) + _coord(aabb[1], index))

    ctx.expect_gap(
        lower_stage,
        base,
        axis="z",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem="turntable_table",
        negative_elem="bearing_race",
        name="rotary table clears fixed bearing race",
    )
    ctx.expect_gap(
        lower_stage,
        faceplate,
        axis="y",
        min_gap=0.030,
        positive_elem="bearing_cover_pos",
        negative_elem="face_disk",
        name="positive cheek has faceplate side clearance",
    )
    ctx.expect_gap(
        faceplate,
        lower_stage,
        axis="y",
        min_gap=0.030,
        positive_elem="face_disk",
        negative_elem="bearing_cover_neg",
        name="negative cheek has faceplate side clearance",
    )
    ctx.allow_overlap(
        faceplate,
        lower_stage,
        elem_a="trunnion",
        elem_b="bearing_cover_pos",
        reason="The trunnion journal is intentionally seated in the positive-side bearing cover with a tiny captured-shaft interference proxy.",
    )
    ctx.expect_overlap(
        faceplate,
        lower_stage,
        axes="y",
        min_overlap=0.010,
        elem_a="trunnion",
        elem_b="bearing_cover_pos",
        name="positive bearing cover retains the trunnion journal",
    )
    ctx.allow_overlap(
        faceplate,
        lower_stage,
        elem_a="trunnion",
        elem_b="bearing_cover_neg",
        reason="The trunnion journal is intentionally seated in the negative-side bearing cover with a tiny captured-shaft interference proxy.",
    )
    ctx.expect_overlap(
        faceplate,
        lower_stage,
        axes="y",
        min_overlap=0.010,
        elem_a="trunnion",
        elem_b="bearing_cover_neg",
        name="negative bearing cover retains the trunnion journal",
    )
    for q in (-0.55, 0.0, 0.55):
        with ctx.pose({tilt: q}):
            ctx.expect_gap(
                faceplate,
                lower_stage,
                axis="z",
                min_gap=0.025,
                positive_elem="face_disk",
                negative_elem="turntable_table",
                name=f"tilted faceplate clears lower table at {q:+.2f} rad",
            )

    rest_cradle = ctx.part_element_world_aabb(lower_stage, elem="cradle_frame")
    with ctx.pose({yaw: pi / 2.0}):
        indexed_cradle = ctx.part_element_world_aabb(lower_stage, elem="cradle_frame")
    ctx.check(
        "lower stage rotation sweeps the offset cradle",
        rest_cradle is not None
        and indexed_cradle is not None
        and _center(rest_cradle, 0) > 0.12
        and _center(indexed_cradle, 1) > 0.12,
        details=f"rest={rest_cradle}, indexed={indexed_cradle}",
    )

    rest_face = ctx.part_element_world_aabb(faceplate, elem="face_disk")
    with ctx.pose({tilt: 0.55}):
        tilted_face = ctx.part_element_world_aabb(faceplate, elem="face_disk")
    ctx.check(
        "faceplate tilts about the trunnion axis",
        rest_face is not None
        and tilted_face is not None
        and _span(tilted_face, 0) > _span(rest_face, 0) + 0.045,
        details=f"rest={rest_face}, tilted={tilted_face}",
    )

    return ctx.report()


object_model = build_object_model()
