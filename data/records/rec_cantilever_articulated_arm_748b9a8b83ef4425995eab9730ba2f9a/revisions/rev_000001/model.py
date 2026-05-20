from __future__ import annotations

from math import atan2, pi, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


SHOULDER_Z = 0.68
FIRST_LEN = 0.75
SECOND_LEN = 0.45


def _cyl_x(radius: float, length: float) -> Cylinder:
    return Cylinder(radius=radius, length=length)


def _origin_x(x: float, y: float, z: float) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(0.0, pi / 2.0, 0.0))


def _origin_y(x: float, y: float, z: float) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(pi / 2.0, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_frame_cantilever_arm")

    model.material("dark_powder_coat", rgba=(0.05, 0.055, 0.06, 1.0))
    model.material("blue_anodized", rgba=(0.08, 0.22, 0.55, 1.0))
    model.material("brushed_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    model.material("black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.18, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material="dark_powder_coat",
        name="floor_plate",
    )
    pedestal.visual(
        Cylinder(radius=0.065, length=0.56),
        origin=Origin(xyz=(0.0, 0.0, 0.29)),
        material="dark_powder_coat",
        name="column",
    )
    pedestal.visual(
        Box((0.24, 0.22, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.58)),
        material="dark_powder_coat",
        name="top_saddle",
    )
    for y in (-0.095, 0.095):
        pedestal.visual(
            Box((0.17, 0.03, 0.18)),
            origin=Origin(xyz=(0.0, y, SHOULDER_Z)),
            material="dark_powder_coat",
            name=f"shoulder_cheek_{'neg' if y < 0.0 else 'pos'}",
        )
        pedestal.visual(
            Cylinder(radius=0.072, length=0.026),
            origin=_origin_y(0.0, y, SHOULDER_Z),
            material="brushed_steel",
            name=f"shoulder_bearing_{'neg' if y < 0.0 else 'pos'}",
        )
    for x in (-0.115, 0.115):
        for y in (-0.115, 0.115):
            pedestal.visual(
                Cylinder(radius=0.012, length=0.012),
                origin=Origin(xyz=(x, y, 0.046)),
                material="brushed_steel",
                name=f"base_bolt_{'n' if x < 0 else 'p'}_{'n' if y < 0 else 'p'}",
            )

    first_link = model.part("first_link")
    first_link.visual(
        Cylinder(radius=0.055, length=0.160),
        origin=_origin_y(0.0, 0.0, 0.0),
        material="brushed_steel",
        name="shoulder_hub",
    )
    for y in (-0.056, 0.056):
        first_link.visual(
            Cylinder(radius=0.055, length=0.020),
            origin=_origin_y(FIRST_LEN, y, 0.0),
            material="brushed_steel",
            name=f"elbow_outer_hub_{'neg' if y < 0.0 else 'pos'}",
        )
    rail_len = 0.66
    rail_center = 0.39
    for y in (-0.055, 0.055):
        side = "neg" if y < 0.0 else "pos"
        for z, rail in ((0.055, "upper"), (-0.055, "lower")):
            first_link.visual(
                Box((rail_len, 0.018, 0.026)),
                origin=Origin(xyz=(rail_center, y, z)),
                material="blue_anodized",
                name=f"{rail}_rail_{side}",
            )
        for x, post in ((0.075, "shoulder"), (0.705, "elbow")):
            first_link.visual(
                Box((0.055, 0.018, 0.135)),
                origin=Origin(xyz=(x, y, 0.0)),
                material="blue_anodized",
                name=f"{post}_post_{side}",
            )
        diag_dx = 0.56
        diag_dz = 0.11
        diag_len = sqrt(diag_dx * diag_dx + diag_dz * diag_dz)
        diag_angle = atan2(diag_dz, diag_dx)
        first_link.visual(
            Box((diag_len, 0.016, 0.018)),
            origin=Origin(xyz=(0.40, y, 0.0), rpy=(0.0, -diag_angle, 0.0)),
            material="blue_anodized",
            name=f"rising_brace_{side}",
        )
        first_link.visual(
            Box((diag_len, 0.016, 0.018)),
            origin=Origin(xyz=(0.40, y, 0.0), rpy=(0.0, diag_angle, 0.0)),
            material="blue_anodized",
            name=f"falling_brace_{side}",
        )

    second_link = model.part("second_link")
    second_link.visual(
        Cylinder(radius=0.045, length=0.092),
        origin=_origin_y(0.0, 0.0, 0.0),
        material="brushed_steel",
        name="elbow_inner_hub",
    )
    second_link.visual(
        Cylinder(radius=0.045, length=0.050),
        origin=_origin_x(SECOND_LEN - 0.025, 0.0, 0.0),
        material="brushed_steel",
        name="wrist_bearing",
    )
    short_rail_len = 0.34
    short_rail_center = 0.225
    for y in (-0.026, 0.026):
        side = "neg" if y < 0.0 else "pos"
        for z, rail in ((0.040, "upper"), (-0.040, "lower")):
            second_link.visual(
                Box((short_rail_len, 0.016, 0.022)),
                origin=Origin(xyz=(short_rail_center, y, z)),
                material="blue_anodized",
                name=f"{rail}_rail_{side}",
            )
        for x, post in ((0.055, "elbow"), (0.390, "wrist")):
            second_link.visual(
                Box((0.045, 0.016, 0.100)),
                origin=Origin(xyz=(x, y, 0.0)),
                material="blue_anodized",
                name=f"{post}_post_{side}",
            )
        diag_dx = 0.28
        diag_dz = 0.08
        diag_len = sqrt(diag_dx * diag_dx + diag_dz * diag_dz)
        diag_angle = atan2(diag_dz, diag_dx)
        second_link.visual(
            Box((diag_len, 0.014, 0.016)),
            origin=Origin(xyz=(0.225, y, 0.0), rpy=(0.0, -diag_angle, 0.0)),
            material="blue_anodized",
            name=f"web_brace_{side}",
        )

    wrist_plate = model.part("wrist_plate")
    wrist_plate.visual(
        Cylinder(radius=0.040, length=0.050),
        origin=_origin_x(0.025, 0.0, 0.0),
        material="brushed_steel",
        name="wrist_hub",
    )
    wrist_plate.visual(
        Box((0.035, 0.20, 0.14)),
        origin=Origin(xyz=(0.060, 0.0, 0.0)),
        material="dark_powder_coat",
        name="mounting_plate",
    )
    for y in (-0.065, 0.065):
        for z in (-0.040, 0.040):
            wrist_plate.visual(
                Cylinder(radius=0.009, length=0.008),
                origin=_origin_x(0.0805, y, z),
                material="black_rubber",
                name=f"face_bolt_{'n' if y < 0 else 'p'}_{'n' if z < 0 else 'p'}",
            )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=first_link,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2, lower=-0.55, upper=1.15),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=first_link,
        child=second_link,
        origin=Origin(xyz=(FIRST_LEN, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=1.4, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "wrist",
        ArticulationType.REVOLUTE,
        parent=second_link,
        child=wrist_plate,
        origin=Origin(xyz=(SECOND_LEN, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=-pi / 2.0, upper=pi / 2.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")
    wrist = object_model.get_articulation("wrist")
    first_link = object_model.get_part("first_link")
    second_link = object_model.get_part("second_link")
    wrist_plate = object_model.get_part("wrist_plate")
    pedestal = object_model.get_part("pedestal")

    ctx.check(
        "three revolute joints",
        all(j.articulation_type == ArticulationType.REVOLUTE for j in (shoulder, elbow, wrist)),
        details=f"types={[j.articulation_type for j in (shoulder, elbow, wrist)]}",
    )
    ctx.check(
        "shoulder and elbow share vertical-plane axis",
        shoulder.axis == (0.0, -1.0, 0.0) and elbow.axis == (0.0, -1.0, 0.0),
        details=f"shoulder={shoulder.axis}, elbow={elbow.axis}",
    )
    ctx.check(
        "wrist rolls at arm tip",
        wrist.axis == (1.0, 0.0, 0.0),
        details=f"wrist axis={wrist.axis}",
    )
    ctx.expect_origin_gap(
        second_link,
        first_link,
        axis="x",
        min_gap=0.70,
        max_gap=0.80,
        name="first link has bench-arm length",
    )
    ctx.expect_origin_gap(
        wrist_plate,
        second_link,
        axis="x",
        min_gap=0.42,
        max_gap=0.48,
        name="second link is shorter than first",
    )
    ctx.expect_origin_gap(
        first_link,
        pedestal,
        axis="z",
        min_gap=0.62,
        max_gap=0.72,
        name="shoulder is raised on pedestal",
    )

    rest_elbow = ctx.part_world_position(second_link)
    with ctx.pose({shoulder: 0.55}):
        raised_elbow = ctx.part_world_position(second_link)
    ctx.check(
        "shoulder raises elbow in vertical plane",
        rest_elbow is not None
        and raised_elbow is not None
        and raised_elbow[2] > rest_elbow[2] + 0.25
        and abs(raised_elbow[1] - rest_elbow[1]) < 1e-6,
        details=f"rest={rest_elbow}, raised={raised_elbow}",
    )

    rest_wrist = ctx.part_world_position(wrist_plate)
    with ctx.pose({elbow: 0.75}):
        bent_wrist = ctx.part_world_position(wrist_plate)
    ctx.check(
        "elbow bends forearm in same vertical plane",
        rest_wrist is not None
        and bent_wrist is not None
        and bent_wrist[2] > rest_wrist[2] + 0.25
        and abs(bent_wrist[1] - rest_wrist[1]) < 1e-6,
        details=f"rest={rest_wrist}, bent={bent_wrist}",
    )

    return ctx.report()


object_model = build_object_model()
