from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


L_PROXIMAL = 0.155
L_MIDDLE = 0.125
L_DISTAL = 0.105


def _barrel_origin(x: float, y: float = 0.0, z: float = 0.0) -> Origin:
    """Cylinder visual transform for a hinge barrel whose axis runs along Y."""
    return Origin(xyz=(x, y, z), rpy=(math.pi / 2.0, 0.0, 0.0))


def _rounded_plate_mesh(length: float, width: float, thickness: float, name: str):
    profile = rounded_rect_profile(
        length,
        width,
        min(width * 0.48, length * 0.20),
        corner_segments=10,
    )
    return mesh_from_geometry(ExtrudeGeometry(profile, thickness, center=True), name)


def _add_rounded_plate(part, *, name: str, length: float, width: float, thickness: float, center_x: float, material) -> None:
    part.visual(
        _rounded_plate_mesh(length, width, thickness, name),
        origin=Origin(xyz=(center_x, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_phalanx_chain")

    satin = model.material("satin_titanium", rgba=(0.62, 0.64, 0.66, 1.0))
    dark = model.material("dark_bushings", rgba=(0.08, 0.085, 0.09, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))

    root = model.part("root_clevis")
    root.visual(
        Box((0.075, 0.068, 0.008)),
        origin=Origin(xyz=(-0.035, 0.0, -0.018)),
        material=satin,
        name="mounting_plate",
    )
    root.visual(
        Box((0.018, 0.056, 0.030)),
        origin=Origin(xyz=(-0.029, 0.0, 0.0)),
        material=satin,
        name="rear_bridge",
    )
    for sign, suffix in ((1.0, "0"), (-1.0, "1")):
        root.visual(
            Box((0.052, 0.007, 0.030)),
            origin=Origin(xyz=(0.0, sign * 0.020, 0.0)),
            material=satin,
            name=f"clevis_cheek_{suffix}",
        )
        root.visual(
            Cylinder(radius=0.011, length=0.007),
            origin=_barrel_origin(0.0, sign * 0.020, 0.0),
            material=dark,
            name=f"cheek_boss_{suffix}",
        )

    proximal = model.part("proximal_phalanx")
    proximal.visual(
        Cylinder(radius=0.010, length=0.033),
        origin=_barrel_origin(0.0),
        material=dark,
        name="proximal_barrel",
    )
    _add_rounded_plate(
        proximal,
        name="proximal_spine",
        length=0.130,
        width=0.024,
        thickness=0.008,
        center_x=0.072,
        material=satin,
    )
    proximal.visual(
        Box((0.032, 0.042, 0.008)),
        origin=Origin(xyz=(L_PROXIMAL - 0.026, 0.0, 0.0)),
        material=satin,
        name="distal_yoke_bridge",
    )
    for sign, suffix in ((1.0, "0"), (-1.0, "1")):
        proximal.visual(
            Box((0.032, 0.006, 0.021)),
            origin=Origin(xyz=(L_PROXIMAL, sign * 0.015, 0.0)),
            material=satin,
            name=f"distal_yoke_cheek_{suffix}",
        )
        proximal.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=_barrel_origin(L_PROXIMAL, sign * 0.015, 0.0),
            material=dark,
            name=f"distal_boss_{suffix}",
        )

    middle = model.part("middle_phalanx")
    middle.visual(
        Cylinder(radius=0.0085, length=0.024),
        origin=_barrel_origin(0.0),
        material=dark,
        name="middle_barrel",
    )
    _add_rounded_plate(
        middle,
        name="middle_spine",
        length=0.101,
        width=0.020,
        thickness=0.007,
        center_x=0.057,
        material=satin,
    )
    middle.visual(
        Box((0.028, 0.036, 0.007)),
        origin=Origin(xyz=(L_MIDDLE - 0.023, 0.0, 0.0)),
        material=satin,
        name="distal_yoke_bridge",
    )
    for sign, suffix in ((1.0, "0"), (-1.0, "1")):
        middle.visual(
            Box((0.028, 0.0055, 0.018)),
            origin=Origin(xyz=(L_MIDDLE, sign * 0.01225, 0.0)),
            material=satin,
            name=f"distal_yoke_cheek_{suffix}",
        )
        middle.visual(
            Cylinder(radius=0.0088, length=0.0055),
            origin=_barrel_origin(L_MIDDLE, sign * 0.01225, 0.0),
            material=dark,
            name=f"distal_boss_{suffix}",
        )

    distal = model.part("distal_phalanx")
    distal.visual(
        Cylinder(radius=0.0078, length=0.019),
        origin=_barrel_origin(0.0),
        material=dark,
        name="distal_barrel",
    )
    _add_rounded_plate(
        distal,
        name="distal_spine",
        length=0.078,
        width=0.017,
        thickness=0.0065,
        center_x=0.046,
        material=satin,
    )
    _add_rounded_plate(
        distal,
        name="end_pad",
        length=0.034,
        width=0.016,
        thickness=0.008,
        center_x=0.094,
        material=rubber,
    )

    model.articulation(
        "root_knuckle",
        ArticulationType.REVOLUTE,
        parent=root,
        child=proximal,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.5, velocity=4.0, lower=0.0, upper=1.25),
    )
    model.articulation(
        "proximal_knuckle",
        ArticulationType.REVOLUTE,
        parent=proximal,
        child=middle,
        origin=Origin(xyz=(L_PROXIMAL, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=0.0, upper=1.35),
    )
    model.articulation(
        "middle_knuckle",
        ArticulationType.REVOLUTE,
        parent=middle,
        child=distal,
        origin=Origin(xyz=(L_MIDDLE, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.6, velocity=4.0, lower=0.0, upper=1.20),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    root = object_model.get_part("root_clevis")
    proximal = object_model.get_part("proximal_phalanx")
    middle = object_model.get_part("middle_phalanx")
    distal = object_model.get_part("distal_phalanx")
    root_joint = object_model.get_articulation("root_knuckle")
    proximal_joint = object_model.get_articulation("proximal_knuckle")
    middle_joint = object_model.get_articulation("middle_knuckle")

    joints = (root_joint, proximal_joint, middle_joint)
    ctx.check(
        "three revolute knuckles",
        len(object_model.articulations) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints),
        details=f"articulations={[j.name for j in object_model.articulations]}",
    )

    with ctx.pose({root_joint: 0.0, proximal_joint: 0.0, middle_joint: 0.0}):
        ctx.expect_overlap(
            proximal,
            root,
            axes="yz",
            min_overlap=0.018,
            elem_a="proximal_barrel",
            name="root clevis surrounds proximal barrel",
        )
        ctx.expect_origin_gap(
            middle,
            proximal,
            axis="x",
            min_gap=L_PROXIMAL - 0.001,
            max_gap=L_PROXIMAL + 0.001,
            name="proximal link spans first phalanx length",
        )
        ctx.expect_origin_gap(
            distal,
            middle,
            axis="x",
            min_gap=L_MIDDLE - 0.001,
            max_gap=L_MIDDLE + 0.001,
            name="middle link spans second phalanx length",
        )
        ctx.expect_overlap(
            distal,
            middle,
            axes="yz",
            min_overlap=0.012,
            elem_a="distal_barrel",
            name="distal barrel is captured in middle yoke",
        )
        rest_tip = ctx.part_element_world_aabb(distal, elem="end_pad")

    with ctx.pose({root_joint: 0.65, proximal_joint: 0.75, middle_joint: 0.60}):
        folded_tip = ctx.part_element_world_aabb(distal, elem="end_pad")

    ctx.check(
        "chain bends upward in one plane",
        rest_tip is not None
        and folded_tip is not None
        and folded_tip[0][2] > rest_tip[0][2] + 0.10
        and abs(folded_tip[0][1] - rest_tip[0][1]) < 0.002,
        details=f"rest_tip={rest_tip}, folded_tip={folded_tip}",
    )

    return ctx.report()


object_model = build_object_model()
