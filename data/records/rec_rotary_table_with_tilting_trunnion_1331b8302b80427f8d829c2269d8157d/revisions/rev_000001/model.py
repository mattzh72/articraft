from __future__ import annotations

from math import pi

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
    mesh_from_cadquery,
)
import cadquery as cq


def _box(size: tuple[float, float, float], center: tuple[float, float, float]):
    return cq.Workplane("XY").box(*size).translate(center)


def _cyl_z(radius: float, length: float, center: tuple[float, float, float]):
    return cq.Workplane("XY").cylinder(length, radius).translate(center)


def _cyl_y(radius: float, length: float, center: tuple[float, float, float]):
    return (
        cq.Workplane("XY")
        .cylinder(length, radius)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate(center)
    )


def _union_all(*solids):
    result = solids[0]
    for solid in solids[1:]:
        result = result.union(solid)
    return result


def _make_foot_shape():
    base = _box((0.72, 0.52, 0.08), (0.0, 0.0, 0.04)).edges("|Z").fillet(0.035)
    pedestal = _cyl_z(0.115, 0.38, (0.0, 0.0, 0.27))
    top_bearing = _cyl_z(0.205, 0.08, (0.0, 0.0, 0.50))
    rib_x = _box((0.42, 0.055, 0.31), (0.0, 0.0, 0.255))
    rib_y = _box((0.055, 0.32, 0.31), (0.0, 0.0, 0.255))
    return _union_all(base, pedestal, top_bearing, rib_x, rib_y).clean()


def _make_lower_platform_shape():
    turntable = _cyl_z(0.290, 0.070, (0.0, 0.0, 0.035))
    deck = _box((0.58, 0.46, 0.060), (0.0, 0.0, 0.100))
    raised_race = _cyl_z(0.220, 0.035, (0.0, 0.0, 0.147))
    support_0 = _box((0.155, 0.080, 0.340), (0.0, 0.245, 0.300))
    support_1 = _box((0.155, 0.080, 0.340), (0.0, -0.245, 0.300))
    boss_0 = _cyl_y(0.090, 0.105, (0.0, 0.245, 0.350))
    boss_1 = _cyl_y(0.090, 0.105, (0.0, -0.245, 0.350))

    body = _union_all(turntable, deck, raised_race, support_0, support_1, boss_0, boss_1)
    bearing_clearance = _cyl_y(0.062, 0.72, (0.0, 0.0, 0.350))
    return body.cut(bearing_clearance).clean()


def _make_faceplate_shape():
    disk = _cyl_y(0.170, 0.055, (0.0, 0.0, 0.0))
    hub = _cyl_y(0.075, 0.100, (0.0, 0.0, 0.0))
    shaft = _cyl_y(0.045, 0.385, (0.0, 0.0, 0.0))
    cap_0 = _cyl_y(0.067, 0.025, (0.0, 0.1800, 0.0))
    cap_1 = _cyl_y(0.067, 0.025, (0.0, -0.1800, 0.0))
    body = _union_all(disk, hub, shaft, cap_0, cap_1)

    bolt_holes = []
    for x, z in ((0.085, 0.085), (-0.085, 0.085), (-0.085, -0.085), (0.085, -0.085)):
        bolt_holes.append(_cyl_y(0.014, 0.14, (x, 0.0, z)))

    slot_holes = []
    # Three asymmetric workholding slots make rotation of the compact faceplate visible.
    for x, z, sx, sz in ((0.0, 0.125, 0.032, 0.050), (-0.115, -0.030, 0.050, 0.032), (0.110, -0.055, 0.050, 0.032)):
        slot_holes.append(_box((sx, 0.14, sz), (x, 0.0, z)))

    cutters = _union_all(*(bolt_holes + slot_holes))
    return body.cut(cutters).clean()


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_trunnion_positioner")

    painted_casting = Material("painted_casting", rgba=(0.16, 0.19, 0.21, 1.0))
    platform_blue = Material("blue_positioner_casting", rgba=(0.05, 0.18, 0.32, 1.0))
    machined_steel = Material("machined_steel", rgba=(0.56, 0.58, 0.57, 1.0))
    black_oxide = Material("black_oxide", rgba=(0.02, 0.022, 0.024, 1.0))
    brass = Material("warm_bearing_bronze", rgba=(0.70, 0.47, 0.18, 1.0))
    caution_yellow = Material("caution_yellow", rgba=(1.0, 0.72, 0.08, 1.0))

    foot = model.part("foot")
    foot.visual(
        mesh_from_cadquery(_make_foot_shape(), "fixed_pedestal_foot", tolerance=0.0015),
        material=painted_casting,
        name="pedestal_casting",
    )
    for idx, (x, y) in enumerate(((-0.285, -0.185), (-0.285, 0.185), (0.285, -0.185), (0.285, 0.185))):
        foot.visual(
            Cylinder(radius=0.033, length=0.020),
            origin=Origin(xyz=(x, y, 0.087)),
            material=black_oxide,
            name=f"anchor_bolt_{idx}",
        )
    foot.visual(
        Cylinder(radius=0.198, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.535)),
        material=brass,
        name="fixed_bearing_race",
    )

    lower_platform = model.part("lower_platform")
    lower_platform.visual(
        Cylinder(radius=0.290, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=platform_blue,
        name="turntable_disk",
    )
    lower_platform.visual(
        Box((0.58, 0.46, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.099)),
        material=platform_blue,
        name="deck_plate",
    )
    lower_platform.visual(
        Cylinder(radius=0.220, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.146)),
        material=platform_blue,
        name="upper_slew_ring",
    )
    for support_name, boss_name, y in (
        ("trunnion_support_0", "bearing_boss_0", -0.245),
        ("trunnion_support_1", "bearing_boss_1", 0.245),
    ):
        lower_platform.visual(
            Box((0.155, 0.080, 0.340)),
            origin=Origin(xyz=(0.0, y, 0.298)),
            material=platform_blue,
            name=support_name,
        )
        lower_platform.visual(
            Cylinder(radius=0.090, length=0.105),
            origin=Origin(xyz=(0.0, y, 0.350), rpy=(pi / 2.0, 0.0, 0.0)),
            material=machined_steel,
            name=boss_name,
        )
    lower_platform.visual(
        Cylinder(radius=0.215, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=machined_steel,
        name="rotating_bearing_race",
    )

    faceplate = model.part("faceplate")
    faceplate.visual(
        mesh_from_cadquery(_make_faceplate_shape(), "compact_trunnion_faceplate", tolerance=0.0012),
        material=machined_steel,
        name="faceplate_disk",
    )
    faceplate.visual(
        Box((0.038, 0.016, 0.030)),
        origin=Origin(xyz=(0.100, -0.035, 0.058)),
        material=caution_yellow,
        name="index_lug",
    )

    model.articulation(
        "foot_to_lower_platform",
        ArticulationType.REVOLUTE,
        parent=foot,
        child=lower_platform,
        origin=Origin(xyz=(0.0, 0.0, 0.540)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=1.2, lower=-pi, upper=pi),
    )
    model.articulation(
        "platform_to_faceplate",
        ArticulationType.REVOLUTE,
        parent=lower_platform,
        child=faceplate,
        origin=Origin(xyz=(0.0, 0.0, 0.350)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=140.0, velocity=1.0, lower=-pi, upper=pi),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    foot = object_model.get_part("foot")
    lower_platform = object_model.get_part("lower_platform")
    faceplate = object_model.get_part("faceplate")
    yaw = object_model.get_articulation("foot_to_lower_platform")
    tilt = object_model.get_articulation("platform_to_faceplate")

    ctx.check(
        "two revolute axes",
        yaw.axis == (0.0, 0.0, 1.0) and tilt.axis == (0.0, 1.0, 0.0),
        details=f"yaw axis={yaw.axis}, faceplate axis={tilt.axis}",
    )
    ctx.expect_gap(
        lower_platform,
        foot,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem="turntable_disk",
        negative_elem="pedestal_casting",
        name="rotating platform sits on fixed bearing",
    )
    ctx.expect_within(
        faceplate,
        lower_platform,
        axes="x",
        inner_elem="faceplate_disk",
        outer_elem="deck_plate",
        margin=0.040,
        name="faceplate axis is centered between supports",
    )
    ctx.expect_overlap(
        faceplate,
        lower_platform,
        axes="y",
        min_overlap=0.35,
        elem_a="faceplate_disk",
        elem_b="deck_plate",
        name="trunnion shaft spans both support bearings",
    )
    ctx.expect_contact(
        faceplate,
        lower_platform,
        elem_a="faceplate_disk",
        elem_b="bearing_boss_1",
        contact_tol=0.002,
        name="faceplate journals seat against trunnion bearings",
    )

    rest_lug = ctx.part_element_world_aabb(faceplate, elem="index_lug")
    with ctx.pose({tilt: pi / 2.0}):
        turned_lug = ctx.part_element_world_aabb(faceplate, elem="index_lug")
    ctx.check(
        "faceplate rotation moves index lug",
        rest_lug is not None
        and turned_lug is not None
        and abs(((rest_lug[0][2] + rest_lug[1][2]) * 0.5) - ((turned_lug[0][2] + turned_lug[1][2]) * 0.5)) > 0.040,
        details=f"rest_lug={rest_lug}, turned_lug={turned_lug}",
    )

    return ctx.report()


object_model = build_object_model()
