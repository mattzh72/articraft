from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


SHOULDER_Z = 0.100
FIRST_LINK_LENGTH = 0.460
SECOND_LINK_LENGTH = 0.300


def _circle_profile(cx: float, cy: float, radius: float, segments: int = 40):
    return [
        (
            cx + radius * cos(2.0 * pi * i / segments),
            cy + radius * sin(2.0 * pi * i / segments),
        )
        for i in range(segments)
    ]


def _capsule_profile(length: float, radius: float, segments: int = 28):
    """2-D dogbone/capsule outline in the local XY plane."""

    pts = [(0.0, -radius), (length, -radius)]
    for i in range(1, segments + 1):
        a = -pi / 2.0 + pi * i / segments
        pts.append((length + radius * cos(a), radius * sin(a)))
    pts.append((0.0, radius))
    for i in range(1, segments):
        a = pi / 2.0 + pi * i / segments
        pts.append((radius * cos(a), radius * sin(a)))
    return pts


def _link_plate_mesh(
    length: float,
    radius: float,
    thickness: float,
    bore_radius: float,
    distal_bore_radius: float,
    name: str,
):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _capsule_profile(length, radius),
            [
                _circle_profile(0.0, 0.0, bore_radius),
                _circle_profile(length, 0.0, distal_bore_radius),
            ],
            thickness,
            center=True,
        ),
        name,
    )


def _washer_mesh(outer_radius: float, inner_radius: float, thickness: float, name: str):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(0.0, 0.0, outer_radius),
            [_circle_profile(0.0, 0.0, inner_radius)],
            thickness,
            center=True,
        ),
        name,
    )


def _yoke_plate_mesh(name: str):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(0.130, 0.105, 0.014, corner_segments=8),
            [_circle_profile(0.0, 0.0, 0.0175)],
            0.012,
            center=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_service_arm")

    model.material("painted_steel", rgba=(0.18, 0.20, 0.22, 1.0))
    model.material("blackened_steel", rgba=(0.05, 0.055, 0.060, 1.0))
    model.material("brushed_aluminum", rgba=(0.62, 0.65, 0.67, 1.0))
    model.material("brushed_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    model.material("zinc_bolt", rgba=(0.80, 0.78, 0.70, 1.0))
    model.material("rubber", rgba=(0.015, 0.015, 0.014, 1.0))

    base = model.part("base_bracket")
    base.visual(
        Box((0.220, 0.155, 0.016)),
        origin=Origin(xyz=(0.000, 0.000, 0.008)),
        material="painted_steel",
        name="ground_plate",
    )
    base.visual(
        Box((0.024, 0.112, 0.116)),
        origin=Origin(xyz=(-0.077, 0.000, 0.066)),
        material="painted_steel",
        name="rear_web",
    )
    base.visual(
        _yoke_plate_mesh("lower_yoke_mesh"),
        origin=Origin(xyz=(0.000, 0.000, SHOULDER_Z - 0.030)),
        material="painted_steel",
        name="lower_yoke",
    )
    base.visual(
        _yoke_plate_mesh("upper_yoke_mesh"),
        origin=Origin(xyz=(0.000, 0.000, SHOULDER_Z + 0.030)),
        material="painted_steel",
        name="upper_yoke",
    )
    base.visual(
        Cylinder(radius=0.0095, length=0.080),
        origin=Origin(xyz=(0.000, 0.000, SHOULDER_Z + 0.012)),
        material="zinc_bolt",
        name="shoulder_shank",
    )
    base.visual(
        Cylinder(radius=0.025, length=0.006),
        origin=Origin(xyz=(0.000, 0.000, SHOULDER_Z + 0.039)),
        material="zinc_bolt",
        name="shoulder_head",
    )
    base.visual(
        Cylinder(radius=0.021, length=0.006),
        origin=Origin(xyz=(0.000, 0.000, SHOULDER_Z - 0.039)),
        material="zinc_bolt",
        name="shoulder_nut",
    )
    for i, (x_pos, y_pos) in enumerate(
        ((-0.078, -0.052), (-0.078, 0.052), (0.078, -0.052), (0.078, 0.052))
    ):
        base.visual(
            Cylinder(radius=0.010, length=0.004),
            origin=Origin(xyz=(x_pos, y_pos, 0.018)),
            material="zinc_bolt",
            name=f"base_bolt_{i}",
        )

    first = model.part("first_link")
    first.visual(
        _link_plate_mesh(
            FIRST_LINK_LENGTH,
            radius=0.044,
            thickness=0.020,
            bore_radius=0.017,
            distal_bore_radius=0.016,
            name="first_link_plate_mesh",
        ),
        material="brushed_aluminum",
        name="link_plate",
    )
    for i, y_pos in enumerate((-0.032, 0.032)):
        first.visual(
            Box((FIRST_LINK_LENGTH - 0.110, 0.008, 0.006)),
            origin=Origin(xyz=(FIRST_LINK_LENGTH / 2.0, y_pos, 0.012)),
            material="brushed_steel",
            name=f"edge_flange_{i}",
        )
    first.visual(
        _washer_mesh(0.027, 0.017, 0.006, "shoulder_collar_mesh"),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material="brushed_steel",
        name="shoulder_collar",
    )
    first.visual(
        _washer_mesh(0.027, 0.016, 0.006, "elbow_collar_mesh"),
        origin=Origin(xyz=(FIRST_LINK_LENGTH, 0.0, 0.013)),
        material="brushed_steel",
        name="elbow_collar",
    )
    first.visual(
        _washer_mesh(0.020, 0.013, 0.006, "elbow_nut_mesh"),
        origin=Origin(xyz=(FIRST_LINK_LENGTH, 0.0, -0.013)),
        material="zinc_bolt",
        name="elbow_nut",
    )

    second = model.part("second_link")
    second.visual(
        _link_plate_mesh(
            SECOND_LINK_LENGTH,
            radius=0.031,
            thickness=0.014,
            bore_radius=0.013,
            distal_bore_radius=0.010,
            name="second_link_plate_mesh",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material="brushed_aluminum",
        name="link_plate",
    )
    for i, y_pos in enumerate((-0.022, 0.022)):
        second.visual(
            Box((SECOND_LINK_LENGTH - 0.080, 0.006, 0.005)),
            origin=Origin(xyz=(SECOND_LINK_LENGTH / 2.0, y_pos, 0.0395)),
            material="brushed_steel",
            name=f"edge_flange_{i}",
        )
    second.visual(
        _washer_mesh(0.022, 0.013, 0.006, "elbow_top_washer_mesh"),
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
        material="zinc_bolt",
        name="elbow_head",
    )
    second.visual(
        Cylinder(radius=0.009, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material="zinc_bolt",
        name="elbow_shank",
    )
    second.visual(
        Box((0.034, 0.058, 0.020)),
        origin=Origin(xyz=(SECOND_LINK_LENGTH + 0.016, 0.0, 0.032)),
        material="brushed_steel",
        name="end_clevis",
    )

    pad = model.part("end_pad")
    pad.visual(
        Box((0.018, 0.060, 0.030)),
        origin=Origin(xyz=(0.009, 0.0, 0.0)),
        material="blackened_steel",
        name="pad_saddle",
    )
    pad.visual(
        Box((0.030, 0.074, 0.044)),
        origin=Origin(xyz=(0.033, 0.0, 0.0)),
        material="rubber",
        name="rubber_face",
    )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=base,
        child=first,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-1.10, upper=1.10, effort=40.0, velocity=1.6),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=first,
        child=second,
        origin=Origin(xyz=(FIRST_LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-2.20, upper=2.20, effort=22.0, velocity=1.8),
    )
    model.articulation(
        "pad_mount",
        ArticulationType.FIXED,
        parent=second,
        child=pad,
        origin=Origin(xyz=(SECOND_LINK_LENGTH + 0.033, 0.0, 0.032)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_bracket")
    first = object_model.get_part("first_link")
    second = object_model.get_part("second_link")
    pad = object_model.get_part("end_pad")
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")

    ctx.allow_overlap(
        base,
        first,
        elem_a="shoulder_shank",
        elem_b="link_plate",
        reason=(
            "The shoulder bolt shank is intentionally captured through the "
            "link bore to ground the first link in the base yoke."
        ),
    )
    ctx.allow_overlap(
        base,
        first,
        elem_a="shoulder_shank",
        elem_b="shoulder_collar",
        reason=(
            "The visible shoulder collar is a simplified bearing washer seated "
            "around the same captured shoulder-bolt shank."
        ),
    )
    ctx.allow_overlap(
        second,
        first,
        elem_a="elbow_shank",
        elem_b="link_plate",
        reason=(
            "The elbow shoulder bolt is intentionally represented as a captured "
            "pin through the first-link eye while carrying the raised distal link."
        ),
    )
    ctx.allow_overlap(
        second,
        first,
        elem_a="elbow_shank",
        elem_b="elbow_collar",
        reason=(
            "The machined elbow collar is intentionally modeled around the "
            "captured shoulder-bolt shank."
        ),
    )
    ctx.allow_overlap(
        second,
        first,
        elem_a="elbow_shank",
        elem_b="elbow_nut",
        reason=(
            "The underside elbow nut is shown as part of the same captured "
            "shoulder bolt stack."
        ),
    )

    ctx.expect_origin_distance(
        first,
        second,
        axes="xy",
        min_dist=FIRST_LINK_LENGTH - 0.002,
        max_dist=FIRST_LINK_LENGTH + 0.002,
        name="first link sets the longer reach span",
    )
    ctx.expect_gap(
        first,
        base,
        axis="z",
        positive_elem="link_plate",
        negative_elem="lower_yoke",
        min_gap=0.010,
        max_gap=0.020,
        name="shoulder link clears the lower yoke plate",
    )
    ctx.expect_gap(
        base,
        first,
        axis="z",
        positive_elem="upper_yoke",
        negative_elem="link_plate",
        min_gap=0.010,
        max_gap=0.020,
        name="shoulder link clears the upper yoke plate",
    )
    ctx.expect_gap(
        second,
        first,
        axis="z",
        positive_elem="link_plate",
        negative_elem="link_plate",
        min_gap=0.010,
        max_gap=0.025,
        name="elbow stack keeps neighboring links separated",
    )
    ctx.expect_gap(
        pad,
        second,
        axis="x",
        positive_elem="pad_saddle",
        negative_elem="end_clevis",
        max_gap=0.002,
        max_penetration=0.00001,
        name="end pad saddle seats on the distal clevis face",
    )
    ctx.expect_within(
        base,
        first,
        axes="xy",
        inner_elem="shoulder_shank",
        outer_elem="link_plate",
        margin=0.001,
        name="shoulder bolt is centered inside the first-link eye",
    )
    ctx.expect_overlap(
        base,
        first,
        axes="z",
        elem_a="shoulder_shank",
        elem_b="link_plate",
        min_overlap=0.015,
        name="shoulder bolt remains captured through the link thickness",
    )
    ctx.expect_overlap(
        base,
        first,
        axes="z",
        elem_a="shoulder_shank",
        elem_b="shoulder_collar",
        min_overlap=0.004,
        name="shoulder collar is retained by the bolt shank",
    )
    ctx.expect_within(
        second,
        first,
        axes="xy",
        inner_elem="elbow_shank",
        outer_elem="link_plate",
        margin=0.001,
        name="elbow bolt is centered inside the first-link distal eye",
    )
    ctx.expect_overlap(
        second,
        first,
        axes="z",
        elem_a="elbow_shank",
        elem_b="link_plate",
        min_overlap=0.015,
        name="elbow bolt remains captured through the first-link eye",
    )
    ctx.expect_overlap(
        second,
        first,
        axes="z",
        elem_a="elbow_shank",
        elem_b="elbow_collar",
        min_overlap=0.004,
        name="elbow collar is retained by the bolt shank",
    )

    rest_elbow = ctx.part_world_position(second)
    with ctx.pose({shoulder: 1.0, elbow: -1.4}):
        swept_elbow = ctx.part_world_position(second)
        ctx.expect_gap(
            second,
            first,
            axis="z",
            positive_elem="link_plate",
            negative_elem="link_plate",
            min_gap=0.010,
            max_gap=0.025,
            name="folded pose keeps the offset elbow stack clear",
        )

    ctx.check(
        "shoulder rotation sweeps the elbow in the service plane",
        rest_elbow is not None
        and swept_elbow is not None
        and swept_elbow[1] > rest_elbow[1] + 0.30,
        details=f"rest_elbow={rest_elbow}, swept_elbow={swept_elbow}",
    )

    return ctx.report()


object_model = build_object_model()
