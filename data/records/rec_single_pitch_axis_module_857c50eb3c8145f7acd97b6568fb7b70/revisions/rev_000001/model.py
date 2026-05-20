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
    TrunnionYokeGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_pitch_trunnion")

    cast = model.material("dark_cast_iron", color=(0.08, 0.09, 0.10, 1.0))
    column_mat = model.material("blackened_column", color=(0.02, 0.025, 0.03, 1.0))
    yoke_mat = model.material("painted_yoke", color=(0.18, 0.22, 0.27, 1.0))
    steel = model.material("brushed_steel", color=(0.68, 0.70, 0.72, 1.0))
    bronze = model.material("bearing_bronze", color=(0.72, 0.48, 0.22, 1.0))
    head_mat = model.material("moving_head_orange", color=(0.88, 0.38, 0.10, 1.0))
    face_mat = model.material("dark_front_face", color=(0.03, 0.035, 0.04, 1.0))
    lens_mat = model.material("smoked_lens", color=(0.01, 0.015, 0.018, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Box((0.62, 0.44, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=cast,
        name="foot_plate",
    )
    pedestal.visual(
        Cylinder(radius=0.065, length=0.520),
        origin=Origin(xyz=(0.0, 0.0, 0.335)),
        material=column_mat,
        name="short_column",
    )
    pedestal.visual(
        Cylinder(radius=0.092, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.092)),
        material=cast,
        name="lower_collar",
    )
    pedestal.visual(
        Cylinder(radius=0.086, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.585)),
        material=cast,
        name="upper_collar",
    )
    pedestal.visual(
        Box((0.30, 0.24, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.595)),
        material=cast,
        name="top_saddle",
    )

    # The yoke is a single clevis/bearing-housing member with a real central
    # opening and through bores for the pitch trunnion.
    yoke_bottom_z = 0.610
    trunnion_center_z = 0.225
    yoke = TrunnionYokeGeometry(
        (0.56, 0.28, 0.34),
        span_width=0.34,
        trunnion_diameter=0.120,
        trunnion_center_z=trunnion_center_z,
        base_thickness=0.055,
        corner_radius=0.018,
        center=False,
    )
    pedestal.visual(
        mesh_from_geometry(yoke, "trunnion_yoke"),
        origin=Origin(xyz=(0.0, 0.0, yoke_bottom_z)),
        material=yoke_mat,
        name="yoke",
    )
    joint_z = yoke_bottom_z + trunnion_center_z
    for liner_index, (x, y) in enumerate(
        (
            (-0.205, -0.060),
            (-0.205, 0.060),
            (0.205, -0.060),
            (0.205, 0.060),
        )
    ):
        pedestal.visual(
            Box((0.050, 0.030, 0.040)),
            origin=Origin(xyz=(x, y, joint_z)),
            material=bronze,
            name=f"bearing_liner_{liner_index}",
        )

    for bolt_index, (x, y) in enumerate(
        ((-0.245, -0.155), (-0.245, 0.155), (0.245, -0.155), (0.245, 0.155))
    ):
        pedestal.visual(
            Cylinder(radius=0.024, length=0.014),
            origin=Origin(xyz=(x, y, 0.084)),
            material=steel,
            name=f"foot_bolt_{bolt_index}",
        )

    head = model.part("head")
    # The moving rigid member is authored around its trunnion axis; the child
    # part frame is exactly on that pitch axis.
    head.visual(
        Cylinder(radius=0.045, length=0.600),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="trunnion_pin",
    )
    head.visual(
        Cylinder(radius=0.060, length=0.310),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=head_mat,
        name="center_barrel",
    )
    head.visual(
        Box((0.280, 0.180, 0.160)),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=head_mat,
        name="head_block",
    )
    head.visual(
        Box((0.230, 0.020, 0.125)),
        origin=Origin(xyz=(0.0, 0.096, -0.030)),
        material=face_mat,
        name="front_plate",
    )
    head.visual(
        Cylinder(radius=0.056, length=0.012),
        origin=Origin(xyz=(0.0, 0.112, -0.030), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=lens_mat,
        name="front_round_cap",
    )
    for retainer_index, x in enumerate((-0.295, 0.295)):
        head.visual(
            Cylinder(radius=0.063, length=0.020),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"side_retainer_{retainer_index}",
        )

    model.articulation(
        "pitch_trunnion",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, joint_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2, lower=-0.65, upper=0.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    head = object_model.get_part("head")
    pitch = object_model.get_articulation("pitch_trunnion")

    ctx.check(
        "single horizontal pitch joint",
        pitch.axis == (1.0, 0.0, 0.0)
        and pitch.motion_limits.lower < 0.0
        and pitch.motion_limits.upper > 0.0,
        details=f"axis={pitch.axis}, limits={pitch.motion_limits}",
    )
    ctx.expect_overlap(
        head,
        pedestal,
        axes="x",
        elem_a="trunnion_pin",
        elem_b="yoke",
        min_overlap=0.45,
        name="trunnion pin spans both yoke cheeks",
    )
    ctx.expect_gap(
        head,
        pedestal,
        axis="z",
        positive_elem="head_block",
        negative_elem="top_saddle",
        min_gap=0.08,
        name="moving head clears pedestal saddle",
    )

    neutral_aabb = ctx.part_element_world_aabb(head, elem="front_round_cap")
    with ctx.pose({pitch: pitch.motion_limits.upper}):
        up_aabb = ctx.part_element_world_aabb(head, elem="front_round_cap")
        ctx.expect_gap(
            head,
            pedestal,
            axis="z",
            positive_elem="head_block",
            negative_elem="top_saddle",
            min_gap=0.025,
            name="head clears saddle at upper tilt stop",
        )
    with ctx.pose({pitch: pitch.motion_limits.lower}):
        down_aabb = ctx.part_element_world_aabb(head, elem="front_round_cap")
        ctx.expect_gap(
            head,
            pedestal,
            axis="z",
            positive_elem="head_block",
            negative_elem="top_saddle",
            min_gap=0.025,
            name="head clears saddle at lower tilt stop",
        )

    if neutral_aabb is not None and up_aabb is not None and down_aabb is not None:
        neutral_z = (neutral_aabb[0][2] + neutral_aabb[1][2]) * 0.5
        up_z = (up_aabb[0][2] + up_aabb[1][2]) * 0.5
        down_z = (down_aabb[0][2] + down_aabb[1][2]) * 0.5
        ctx.check(
            "front cap pitches about trunnion axis",
            up_z > neutral_z + 0.050 and down_z < neutral_z - 0.050,
            details=f"neutral_z={neutral_z}, up_z={up_z}, down_z={down_z}",
        )

    return ctx.report()


object_model = build_object_model()
