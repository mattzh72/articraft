from __future__ import annotations

from math import pi

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


LINK_LENGTHS = (0.34, 0.29, 0.36, 0.31, 0.38)
BEAM_WIDTH = 0.040
BEAM_DEPTH = 0.044
HINGE_RADIUS = 0.024
HINGE_AXIS_RPY = (-pi / 2.0, 0.0, 0.0)


def _add_outer_hinge(part, x: float, metal, dark_metal) -> None:
    """Parent-side fork: two outer hinge barrels tied into the box link."""
    part.visual(
        Box((0.040, 0.128, 0.052)),
        origin=Origin(xyz=(x - 0.080, 0.0, 0.0)),
        material=metal,
        name="distal_cross_cap",
    )
    for y, cheek_name, barrel_name in (
        (-0.048, "distal_cheek_0", "outer_barrel_0"),
        (0.048, "distal_cheek_1", "outer_barrel_1"),
    ):
        part.visual(
            Box((0.086, 0.033, 0.052)),
            origin=Origin(xyz=(x - 0.040, y, 0.0)),
            material=metal,
            name=cheek_name,
        )
        part.visual(
            Cylinder(radius=HINGE_RADIUS, length=0.031),
            origin=Origin(xyz=(x, y, 0.0), rpy=HINGE_AXIS_RPY),
            material=dark_metal,
            name=barrel_name,
        )
    part.visual(
        Cylinder(radius=0.010, length=0.142),
        origin=Origin(xyz=(x, 0.0, 0.0), rpy=HINGE_AXIS_RPY),
        material=dark_metal,
        name="hinge_pin",
    )


def _add_center_hinge(part, metal, dark_metal) -> None:
    """Child-side center knuckle and tongue, clear of the parent fork cheeks."""
    part.visual(
        Cylinder(radius=HINGE_RADIUS, length=0.046),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=HINGE_AXIS_RPY),
        material=dark_metal,
        name="center_barrel",
    )
    part.visual(
        Box((0.086, 0.046, 0.036)),
        origin=Origin(xyz=(0.061, 0.0, 0.0)),
        material=metal,
        name="proximal_tongue",
    )


def _add_link_body(part, length: float, metal, accent, *, proximal: bool, distal: bool) -> None:
    start = 0.052 if proximal else 0.000
    end = length - (0.082 if distal else 0.025)
    body_len = max(0.050, end - start)
    body_center = start + body_len / 2.0
    part.visual(
        Box((body_len, BEAM_WIDTH, BEAM_DEPTH)),
        origin=Origin(xyz=(body_center, 0.0, 0.0)),
        material=metal,
        name="web_box",
    )
    # Narrow cover plates make the otherwise plain rectangular link read as a
    # formed mechanical bar rather than a single placeholder block.
    part.visual(
        Box((body_len * 0.82, BEAM_WIDTH + 0.010, 0.006)),
        origin=Origin(xyz=(body_center, 0.0, BEAM_DEPTH / 2.0 + 0.003)),
        material=accent,
        name="top_plate",
    )
    part.visual(
        Box((body_len * 0.82, BEAM_WIDTH + 0.010, 0.006)),
        origin=Origin(xyz=(body_center, 0.0, -BEAM_DEPTH / 2.0 - 0.003)),
        material=accent,
        name="bottom_plate",
    )
    if distal:
        _add_outer_hinge(part, length, metal, accent)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="five_link_serial_linkage")

    satin_steel = model.material("satin_steel", rgba=(0.56, 0.61, 0.64, 1.0))
    dark_steel = model.material("dark_hinge_steel", rgba=(0.18, 0.20, 0.22, 1.0))
    blued_plate = model.material("blued_cap_plate", rgba=(0.16, 0.25, 0.34, 1.0))
    black_rubber = model.material("matte_black_pad", rgba=(0.03, 0.03, 0.028, 1.0))

    root = model.part("root_link")
    root.visual(
        Box((0.34, 0.22, 0.040)),
        origin=Origin(xyz=(-0.050, 0.0, -0.120)),
        material=dark_steel,
        name="root_foot",
    )
    root.visual(
        Box((0.096, 0.090, 0.150)),
        origin=Origin(xyz=(0.015, 0.0, -0.050)),
        material=satin_steel,
        name="root_pedestal",
    )
    root.visual(
        Box((0.120, 0.142, 0.050)),
        origin=Origin(xyz=(0.045, 0.0, -0.002)),
        material=satin_steel,
        name="root_saddle",
    )
    for y, name in ((-0.062, "root_rib_0"), (0.062, "root_rib_1")):
        root.visual(
            Box((0.142, 0.014, 0.112)),
            origin=Origin(xyz=(0.055, y, -0.055)),
            material=blued_plate,
            name=name,
        )
    _add_link_body(root, LINK_LENGTHS[0], satin_steel, blued_plate, proximal=False, distal=True)

    previous = root
    for index, length in enumerate(LINK_LENGTHS[1:], start=1):
        part = model.part(f"link_{index}")
        _add_center_hinge(part, satin_steel, blued_plate)
        _add_link_body(
            part,
            length,
            satin_steel,
            blued_plate,
            proximal=True,
            distal=index < len(LINK_LENGTHS) - 1,
        )
        if index == len(LINK_LENGTHS) - 1:
            part.visual(
                Box((0.096, 0.052, 0.052)),
                origin=Origin(xyz=(length - 0.030, 0.0, 0.0)),
                material=satin_steel,
                name="pad_neck",
            )
            part.visual(
                Box((0.028, 0.116, 0.116)),
                origin=Origin(xyz=(length + 0.030, 0.0, 0.0)),
                material=black_rubber,
                name="square_pad",
            )

        parent_length = LINK_LENGTHS[index - 1]
        model.articulation(
            f"joint_{index - 1}",
            ArticulationType.REVOLUTE,
            parent=previous,
            child=part,
            origin=Origin(xyz=(parent_length, 0.0, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=-0.55, upper=0.55),
        )
        previous = part

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    joints = [object_model.get_articulation(f"joint_{i}") for i in range(4)]
    links = [object_model.get_part("root_link")] + [
        object_model.get_part(f"link_{i}") for i in range(1, 5)
    ]

    ctx.check(
        "four revolute hinge joints",
        len(joints) == 4 and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints),
        details=f"joints={[j.name for j in joints]}",
    )
    ctx.check(
        "parallel planar hinge axes",
        all(tuple(j.axis or ()) == (0.0, 1.0, 0.0) for j in joints),
        details=f"axes={[j.axis for j in joints]}",
    )
    ctx.check(
        "alternating link rhythm",
        LINK_LENGTHS[0] > LINK_LENGTHS[1] < LINK_LENGTHS[2] > LINK_LENGTHS[3] < LINK_LENGTHS[4],
        details=f"lengths={LINK_LENGTHS}",
    )

    for index, joint in enumerate(joints):
        parent = links[index]
        child = links[index + 1]
        ctx.allow_overlap(
            parent,
            child,
            elem_a="hinge_pin",
            elem_b="center_barrel",
            reason="The visible hinge pin is intentionally captured through the child knuckle bore represented by a solid barrel proxy.",
        )
        ctx.expect_overlap(
            parent,
            child,
            axes="y",
            min_overlap=0.040,
            elem_a="hinge_pin",
            elem_b="center_barrel",
            name=f"joint_{index}_pin_captures_center_knuckle",
        )
        ctx.expect_overlap(
            parent,
            child,
            axes="xz",
            min_overlap=0.020,
            elem_a="outer_barrel_0",
            elem_b="center_barrel",
            name=f"joint_{index}_coaxial_barrels_share_xz",
        )
        ctx.expect_gap(
            parent,
            child,
            axis="y",
            min_gap=0.004,
            positive_elem="outer_barrel_1",
            negative_elem="center_barrel",
            name=f"joint_{index}_barrel_knuckle_clearance",
        )

    pad = object_model.get_part("link_4")
    rest_pad = ctx.part_element_world_aabb(pad, elem="square_pad")
    with ctx.pose({joints[0]: -0.45, joints[1]: 0.35, joints[2]: -0.30, joints[3]: 0.40}):
        bent_pad = ctx.part_element_world_aabb(pad, elem="square_pad")
    ctx.check(
        "articulated chain moves end pad",
        rest_pad is not None
        and bent_pad is not None
        and abs(bent_pad[0][2] - rest_pad[0][2]) > 0.030,
        details=f"rest={rest_pad}, bent={bent_pad}",
    )

    return ctx.report()


object_model = build_object_model()
