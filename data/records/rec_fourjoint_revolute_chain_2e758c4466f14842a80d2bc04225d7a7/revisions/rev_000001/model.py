from __future__ import annotations

import math

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


LINK_LENGTHS = (0.30, 0.24, 0.32, 0.25, 0.29)
LINK_WIDTH = 0.045
LINK_HEIGHT = 0.024
LOW_LINK_Z = -0.025
HIGH_LINK_Z = 0.025
LINK_Z_LEVELS = (LOW_LINK_Z, HIGH_LINK_Z, LOW_LINK_Z, HIGH_LINK_Z, LOW_LINK_Z)

PIN_RADIUS = 0.018
PIN_EMBED = 0.001
PIN_LENGTH = abs(HIGH_LINK_Z - LOW_LINK_Z) - LINK_HEIGHT + PIN_EMBED


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="five_link_serial_linkage")

    model.material("blue_anodized", rgba=(0.12, 0.32, 0.72, 1.0))
    model.material("silver_anodized", rgba=(0.72, 0.74, 0.76, 1.0))
    model.material("dark_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    model.material("base_cast", rgba=(0.18, 0.19, 0.20, 1.0))
    model.material("rubber_pad", rgba=(0.02, 0.02, 0.018, 1.0))

    # The part frame for every link sits on its proximal revolute axis.  The
    # rectangular bar runs in local +X to the next joint.  Alternate bars are
    # raised/lowered slightly, so neighboring bars can sweep about parallel Z
    # axes without reading as a solid stack of identical coplanar blocks.
    link_0 = model.part("link_0")
    link_0.visual(
        Box((LINK_LENGTHS[0], LINK_WIDTH, LINK_HEIGHT)),
        origin=Origin(xyz=(LINK_LENGTHS[0] / 2.0, 0.0, LINK_Z_LEVELS[0])),
        material="blue_anodized",
        name="bar",
    )
    # Fixed root foot, post, and first link are one grounded root assembly.
    link_0.visual(
        Box((0.22, 0.16, 0.018)),
        origin=Origin(xyz=(-0.055, 0.0, -0.086)),
        material="base_cast",
        name="root_foot",
    )
    link_0.visual(
        Box((0.060, 0.060, 0.040)),
        origin=Origin(xyz=(0.025, 0.0, -0.057)),
        material="base_cast",
        name="root_post",
    )

    links = [link_0]
    for index in range(1, 5):
        link = model.part(f"link_{index}")
        link.visual(
            Box((LINK_LENGTHS[index], LINK_WIDTH, LINK_HEIGHT)),
            origin=Origin(xyz=(LINK_LENGTHS[index] / 2.0, 0.0, LINK_Z_LEVELS[index])),
            material="silver_anodized" if index % 2 else "blue_anodized",
            name="bar",
        )

        # A short vertical spacer bridges the height gap at the proximal pivot.
        # It touches the parent bar and embeds only into this link's own bar.
        child_level = LINK_Z_LEVELS[index]
        parent_level = LINK_Z_LEVELS[index - 1]
        pin_center_z = PIN_EMBED / 2.0 if child_level > parent_level else -PIN_EMBED / 2.0
        link.visual(
            Cylinder(radius=PIN_RADIUS, length=PIN_LENGTH),
            origin=Origin(xyz=(0.0, 0.0, pin_center_z)),
            material="dark_steel",
            name="spacer",
        )

        links.append(link)

    # The small square pad is fixed to the distal end of the final link.
    links[-1].visual(
        Box((0.075, 0.075, 0.022)),
        origin=Origin(xyz=(LINK_LENGTHS[-1] + 0.0375, 0.0, LINK_Z_LEVELS[-1])),
        material="rubber_pad",
        name="end_pad",
    )

    for index in range(4):
        model.articulation(
            f"link_{index}_to_link_{index + 1}",
            ArticulationType.REVOLUTE,
            parent=links[index],
            child=links[index + 1],
            origin=Origin(xyz=(LINK_LENGTHS[index], 0.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                lower=-1.25,
                upper=1.25,
                effort=8.0,
                velocity=2.0,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    links = [object_model.get_part(f"link_{index}") for index in range(5)]
    joints = [
        object_model.get_articulation(f"link_{index}_to_link_{index + 1}")
        for index in range(4)
    ]

    ctx.check("five box links", len(links) == 5)
    ctx.check("four revolute joints", len(joints) == 4)
    ctx.check(
        "alternating link lengths",
        LINK_LENGTHS[0] != LINK_LENGTHS[1]
        and LINK_LENGTHS[1] != LINK_LENGTHS[2]
        and LINK_LENGTHS[2] != LINK_LENGTHS[3]
        and LINK_LENGTHS[3] != LINK_LENGTHS[4],
        details=f"lengths={LINK_LENGTHS}",
    )
    ctx.check(
        "parallel vertical joint axes",
        all(tuple(joint.axis or ()) == (0.0, 0.0, 1.0) for joint in joints),
    )

    with ctx.pose({joint: 0.0 for joint in joints}):
        for index in range(1, 5):
            ctx.expect_contact(
                links[index],
                links[index - 1],
                elem_a="spacer",
                elem_b="bar",
                contact_tol=0.0015,
                name=f"joint_{index}_spacer_touches_parent",
            )

    link_2_origin_rest = ctx.part_world_position(links[2])
    with ctx.pose({joints[0]: math.radians(35.0)}):
        link_2_origin_swept = ctx.part_world_position(links[2])
    ctx.check(
        "first joint sweeps chain in plane",
        link_2_origin_rest is not None
        and link_2_origin_swept is not None
        and link_2_origin_swept[1] > link_2_origin_rest[1] + 0.05
        and abs(link_2_origin_swept[2] - link_2_origin_rest[2]) < 1e-6,
        details=f"rest={link_2_origin_rest}, swept={link_2_origin_swept}",
    )

    return ctx.report()


object_model = build_object_model()
