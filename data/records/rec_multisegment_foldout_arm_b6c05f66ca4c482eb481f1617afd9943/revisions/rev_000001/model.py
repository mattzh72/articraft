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
)


LINK_LENGTH = 0.18
LINK_WIDTH = 0.036
LINK_HEIGHT = 0.032
PIN_RADIUS = 0.026
INNER_PIN_LENGTH = 0.042
OUTER_PIN_LENGTH = 0.020
OUTER_KNUCKLE_Y = INNER_PIN_LENGTH / 2.0 + OUTER_PIN_LENGTH / 2.0
FIRST_PIVOT = (-0.075, 0.0, 0.140)


def _pin_origin(x: float, y: float = 0.0, z: float = 0.0) -> Origin:
    """Cylinder helper: SDK cylinders run along local Z; these hinge pins run along Y."""
    return Origin(xyz=(x, y, z), rpy=(pi / 2.0, 0.0, 0.0))


def _add_boxed_link(part, *, body: Material, pin: Material, accent: Material) -> None:
    """A short rectangular link with a center hinge barrel and a forked distal end."""
    part.visual(
        Cylinder(radius=PIN_RADIUS, length=INNER_PIN_LENGTH),
        origin=_pin_origin(0.0),
        material=pin,
        name="proximal_barrel",
    )
    part.visual(
        Box((0.116, LINK_WIDTH, LINK_HEIGHT)),
        origin=Origin(xyz=(0.074, 0.0, 0.0)),
        material=body,
        name="boxed_tube",
    )
    part.visual(
        Box((0.024, 0.095, LINK_HEIGHT)),
        origin=Origin(xyz=(LINK_LENGTH - 0.050, 0.0, 0.0)),
        material=body,
        name="fork_bridge",
    )
    for side, y in (("upper", OUTER_KNUCKLE_Y), ("lower", -OUTER_KNUCKLE_Y)):
        part.visual(
            Box((0.052, 0.018, LINK_HEIGHT)),
            origin=Origin(xyz=(LINK_LENGTH - 0.020, y, 0.0)),
            material=body,
            name=f"{side}_fork_cheek",
        )
        part.visual(
            Cylinder(radius=PIN_RADIUS, length=OUTER_PIN_LENGTH),
            origin=_pin_origin(LINK_LENGTH, y, 0.0),
            material=pin,
            name=f"{side}_fork_barrel",
        )
    part.visual(
        Box((0.085, 0.038, 0.004)),
        origin=Origin(xyz=(0.079, 0.0, LINK_HEIGHT / 2.0 + 0.002)),
        material=accent,
        name="top_inset",
    )
    part.visual(
        Box((0.085, 0.004, 0.020)),
        origin=Origin(xyz=(0.079, LINK_WIDTH / 2.0 + 0.002, 0.0)),
        material=accent,
        name="side_inset",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="four_segment_fold_out_arm")

    base_mat = model.material("matte_black_base", rgba=(0.045, 0.047, 0.050, 1.0))
    body_mat = model.material("powder_coated_link", rgba=(0.16, 0.23, 0.30, 1.0))
    pin_mat = model.material("dark_bushed_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    accent_mat = model.material("recessed_black_panels", rgba=(0.015, 0.016, 0.017, 1.0))
    bracket_mat = model.material("satin_platform", rgba=(0.62, 0.64, 0.62, 1.0))
    screw_mat = model.material("black_screw_heads", rgba=(0.005, 0.005, 0.004, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.260, 0.180, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=base_mat,
        name="base_plate",
    )
    for sx in (-0.085, 0.085):
        for sy in (-0.060, 0.060):
            base.visual(
                Cylinder(radius=0.012, length=0.006),
                origin=Origin(xyz=(sx, sy, 0.021)),
                material=screw_mat,
                name=f"mount_screw_{sx:+.2f}_{sy:+.2f}",
            )
    for side, y in (("upper", OUTER_KNUCKLE_Y), ("lower", -OUTER_KNUCKLE_Y)):
        base.visual(
            Box((0.055, 0.018, 0.112)),
            origin=Origin(xyz=(FIRST_PIVOT[0], y, 0.074)),
            material=base_mat,
            name=f"{side}_stanchion",
        )
        base.visual(
            Box((0.052, 0.018, 0.036)),
            origin=Origin(xyz=(FIRST_PIVOT[0], y, FIRST_PIVOT[2])),
            material=base_mat,
            name=f"{side}_clevis_cheek",
        )
        base.visual(
            Cylinder(radius=PIN_RADIUS, length=OUTER_PIN_LENGTH),
            origin=_pin_origin(FIRST_PIVOT[0], y, FIRST_PIVOT[2]),
            material=pin_mat,
            name=f"{side}_clevis_barrel",
        )

    links = []
    for index in range(4):
        link = model.part(f"link_{index}")
        _add_boxed_link(link, body=body_mat, pin=pin_mat, accent=accent_mat)
        links.append(link)

    platform = model.part("platform")
    platform.visual(
        Cylinder(radius=PIN_RADIUS, length=INNER_PIN_LENGTH),
        origin=_pin_origin(0.0),
        material=pin_mat,
        name="proximal_barrel",
    )
    platform.visual(
        Box((0.070, 0.034, 0.030)),
        origin=Origin(xyz=(0.035, 0.0, 0.0)),
        material=bracket_mat,
        name="neck_block",
    )
    platform.visual(
        Box((0.025, 0.034, 0.045)),
        origin=Origin(xyz=(0.070, 0.0, 0.008)),
        material=bracket_mat,
        name="riser_web",
    )
    platform.visual(
        Box((0.100, 0.080, 0.012)),
        origin=Origin(xyz=(0.090, 0.0, 0.030)),
        material=bracket_mat,
        name="mounting_plate",
    )
    platform.visual(
        Box((0.012, 0.080, 0.030)),
        origin=Origin(xyz=(0.140, 0.0, 0.016)),
        material=bracket_mat,
        name="front_lip",
    )
    for sx in (0.065, 0.115):
        for sy in (-0.025, 0.025):
            platform.visual(
                Cylinder(radius=0.006, length=0.006),
                origin=Origin(xyz=(sx, sy, 0.039)),
                material=screw_mat,
                name=f"plate_screw_{sx:.2f}_{sy:+.2f}",
            )

    limit = MotionLimits(effort=20.0, velocity=2.0, lower=-2.35, upper=2.35)
    model.articulation(
        "base_to_link_0",
        ArticulationType.REVOLUTE,
        parent=base,
        child=links[0],
        origin=Origin(xyz=FIRST_PIVOT),
        axis=(0.0, 1.0, 0.0),
        motion_limits=limit,
    )
    for index in range(3):
        model.articulation(
            f"link_{index}_to_link_{index + 1}",
            ArticulationType.REVOLUTE,
            parent=links[index],
            child=links[index + 1],
            origin=Origin(xyz=(LINK_LENGTH, 0.0, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=limit,
        )
    model.articulation(
        "link_3_to_platform",
        ArticulationType.FIXED,
        parent=links[3],
        child=platform,
        origin=Origin(xyz=(LINK_LENGTH, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    joints = [
        object_model.get_articulation("base_to_link_0"),
        object_model.get_articulation("link_0_to_link_1"),
        object_model.get_articulation("link_1_to_link_2"),
        object_model.get_articulation("link_2_to_link_3"),
    ]

    ctx.check(
        "four boxed links are present",
        all(object_model.get_part(f"link_{i}") is not None for i in range(4)),
        details="expected four serial link parts named link_0 through link_3",
    )
    ctx.check(
        "four revolute hinges fold in the same vertical plane",
        len(joints) == 4 and all(tuple(j.axis) == (0.0, 1.0, 0.0) for j in joints),
        details=f"joint axes={[j.axis for j in joints]}",
    )

    base = object_model.get_part("base")
    platform = object_model.get_part("platform")
    first_link = object_model.get_part("link_0")
    last_link = object_model.get_part("link_3")

    ctx.expect_gap(
        first_link,
        base,
        axis="z",
        min_gap=0.010,
        positive_elem="boxed_tube",
        negative_elem="base_plate",
        name="first link is carried above the base plate",
    )
    ctx.expect_origin_gap(
        platform,
        base,
        axis="x",
        min_gap=0.62,
        name="straight pose reaches outward from the base",
    )
    ctx.expect_origin_gap(
        platform,
        first_link,
        axis="z",
        max_gap=0.003,
        min_gap=-0.003,
        name="straight pose keeps hinges level",
    )

    folded_pose = {
        joints[0]: -1.57,
        joints[1]: 2.20,
        joints[2]: -2.20,
        joints[3]: 2.20,
    }
    rest_platform = ctx.part_world_position(platform)
    rest_last = ctx.part_world_position(last_link)
    with ctx.pose(folded_pose):
        folded_platform = ctx.part_world_position(platform)
        folded_last = ctx.part_world_position(last_link)

    ctx.check(
        "folded pose packs the end bracket near the base",
        rest_platform is not None
        and folded_platform is not None
        and folded_platform[0] < rest_platform[0] - 0.42,
        details=f"rest={rest_platform}, folded={folded_platform}",
    )
    ctx.check(
        "serial joints arc in the fold plane",
        rest_last is not None
        and folded_last is not None
        and abs(folded_last[1] - rest_last[1]) < 0.003
        and abs(folded_last[2] - rest_last[2]) > 0.05,
        details=f"rest_last={rest_last}, folded_last={folded_last}",
    )

    return ctx.report()


object_model = build_object_model()
