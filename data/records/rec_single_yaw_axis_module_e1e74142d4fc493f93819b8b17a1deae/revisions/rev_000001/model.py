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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_wafer_yaw_module")

    dark_cast = model.material("dark_cast_aluminum", rgba=(0.10, 0.11, 0.12, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.015, 0.018, 1.0))
    amber_mark = model.material("amber_index_mark", rgba=(1.0, 0.55, 0.08, 1.0))
    screw_dark = model.material("dark_screw_heads", rgba=(0.025, 0.026, 0.028, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.245, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=dark_cast,
        name="base_disk",
    )
    base.visual(
        Cylinder(radius=0.222, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        material=black_rubber,
        name="base_top_gasket",
    )
    base.visual(
        Cylinder(radius=0.118, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.037)),
        material=dark_cast,
        name="raised_plinth",
    )
    base.visual(
        Cylinder(radius=0.055, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0645)),
        material=satin_steel,
        name="bearing_body",
    )

    for i, (x, y) in enumerate(
        ((0.172, 0.0), (-0.172, 0.0), (0.0, 0.172), (0.0, -0.172))
    ):
        base.visual(
            Cylinder(radius=0.015, length=0.005),
            origin=Origin(xyz=(x, y, 0.0345)),
            material=screw_dark,
            name=f"mount_screw_{i}",
        )

    upper_deck = model.part("upper_deck")
    upper_deck.visual(
        Cylinder(radius=0.190, length=0.018),
        # The child frame is on the lower running face at the centerline.
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=satin_steel,
        name="deck_disk",
    )
    upper_deck.visual(
        Cylinder(radius=0.074, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=satin_steel,
        name="deck_center_boss",
    )
    upper_deck.visual(
        Box(size=(0.120, 0.018, 0.004)),
        origin=Origin(xyz=(0.080, 0.0, 0.020)),
        material=amber_mark,
        name="radial_index_mark",
    )
    for i, (x, y) in enumerate(
        ((0.112, 0.112), (-0.112, 0.112), (-0.112, -0.112), (0.112, -0.112))
    ):
        upper_deck.visual(
            Cylinder(radius=0.012, length=0.004),
            origin=Origin(xyz=(x, y, 0.020)),
            material=screw_dark,
            name=f"deck_screw_{i}",
        )

    model.articulation(
        "yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_deck,
        origin=Origin(xyz=(0.0, 0.0, 0.087)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=2.5,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    upper_deck = object_model.get_part("upper_deck")
    yaw = object_model.get_articulation("yaw")

    ctx.check(
        "one vertical yaw joint",
        len(object_model.articulations) == 1
        and yaw.articulation_type == ArticulationType.REVOLUTE
        and tuple(yaw.axis) == (0.0, 0.0, 1.0),
        details=f"articulations={object_model.articulations}",
    )
    ctx.expect_gap(
        upper_deck,
        base,
        axis="z",
        positive_elem="deck_disk",
        negative_elem="bearing_body",
        max_gap=0.001,
        max_penetration=0.0,
        name="deck rides on bearing top",
    )
    ctx.expect_gap(
        upper_deck,
        base,
        axis="z",
        positive_elem="deck_disk",
        negative_elem="raised_plinth",
        min_gap=0.040,
        max_gap=0.052,
        name="bearing body remains visible below deck",
    )
    ctx.expect_overlap(
        upper_deck,
        base,
        axes="xy",
        elem_a="deck_disk",
        elem_b="bearing_body",
        min_overlap=0.10,
        name="deck centered over bearing",
    )

    rest_pos = ctx.part_world_position(upper_deck)
    with ctx.pose({yaw: math.pi / 2.0}):
        turned_pos = ctx.part_world_position(upper_deck)
        ctx.expect_gap(
            upper_deck,
            base,
            axis="z",
            positive_elem="deck_disk",
            negative_elem="bearing_body",
            max_gap=0.001,
            max_penetration=0.0,
            name="yaw rotation keeps deck seated",
        )
    ctx.check(
        "yaw rotates in place",
        rest_pos is not None
        and turned_pos is not None
        and abs(rest_pos[0] - turned_pos[0]) < 1e-6
        and abs(rest_pos[1] - turned_pos[1]) < 1e-6,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


object_model = build_object_model()
