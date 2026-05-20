from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    KnobGeometry,
    KnobGrip,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
)


def _save_mesh(geometry: MeshGeometry, name: str):
    return mesh_from_geometry(geometry, name)


def _ring_band(
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    radial_segments: int = 64,
) -> MeshGeometry:
    outer = CylinderGeometry(radius=outer_radius, height=length, radial_segments=radial_segments)
    inner = CylinderGeometry(radius=inner_radius, height=length + 0.004, radial_segments=radial_segments)
    return boolean_difference(outer, inner)


def _objective_lens_mesh() -> MeshGeometry:
    # A shallow concave, dark glass meniscus set behind the retaining rim.
    return LatheGeometry(
        [
            (0.000, -0.0020),
            (0.014, -0.0022),
            (0.027, -0.0011),
            (0.032, 0.0000),
            (0.030, 0.0018),
            (0.015, 0.0026),
            (0.000, 0.0022),
        ],
        segments=64,
    )


def _eyepiece_lens_mesh() -> MeshGeometry:
    return LatheGeometry(
        [
            (0.000, -0.0012),
            (0.010, -0.0014),
            (0.018, -0.0004),
            (0.020, 0.0008),
            (0.012, 0.0018),
            (0.000, 0.0016),
        ],
        segments=48,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="marine_waterproof_binoculars")

    navy_rubber = model.material("navy_rubber", rgba=(0.015, 0.045, 0.075, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.006, 0.007, 0.008, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.13, 0.15, 0.16, 1.0))
    dark_glass = model.material("dark_green_glass", rgba=(0.02, 0.15, 0.16, 0.72))
    label_white = model.material("white_markings", rgba=(0.82, 0.86, 0.82, 1.0))

    model.meta["description"] = (
        "Waterproof marine binoculars: two fixed heavy objective housings on a rigid "
        "central bridge, a large continuous focus wheel, and a continuous diopter ring."
    )

    body = model.part("body")

    sep = 0.068
    for index, y in enumerate((-sep, sep)):
        body.visual(
            Cylinder(radius=0.033, length=0.168),
            origin=Origin(xyz=(0.000, y, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=navy_rubber,
            name=f"barrel_{index}",
        )
        body.visual(
            Cylinder(radius=0.047, length=0.072),
            origin=Origin(xyz=(0.073, y, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=navy_rubber,
            name=f"objective_housing_{index}",
        )
        body.visual(
            Cylinder(radius=0.049, length=0.010),
            origin=Origin(xyz=(0.043, y, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black_rubber,
            name=f"front_shoulder_{index}",
        )
        body.visual(
            _save_mesh(
                _ring_band(outer_radius=0.051, inner_radius=0.032, length=0.018),
                f"objective_retainer_{index}",
            ),
            origin=Origin(xyz=(0.118, y, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black_rubber,
            name=f"objective_retainer_{index}",
        )
        body.visual(
            _save_mesh(_objective_lens_mesh(), f"objective_lens_{index}"),
            origin=Origin(xyz=(0.118, y, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_glass,
            name=f"objective_lens_{index}",
        )
        body.visual(
            Cylinder(radius=0.021, length=0.062),
            origin=Origin(xyz=(-0.090, y, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black_rubber,
            name="diopter_sleeve" if index == 0 else f"eyepiece_sleeve_{index}",
        )
        body.visual(
            Cylinder(radius=0.030, length=0.020),
            origin=Origin(xyz=(-0.128, y, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black_rubber,
            name=f"eyecup_{index}",
        )
        body.visual(
            _save_mesh(_eyepiece_lens_mesh(), f"eyepiece_lens_{index}"),
            origin=Origin(xyz=(-0.139, y, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_glass,
            name=f"eyepiece_lens_{index}",
        )

    # The fixed bridge is intentionally a continuous, bulky casting that overlaps both
    # barrels, rather than an interpupillary hinge.
    body.visual(
        Box((0.138, 0.080, 0.036)),
        origin=Origin(xyz=(0.006, 0.000, 0.004)),
        material=satin_metal,
        name="bridge_block",
    )
    body.visual(
        Box((0.092, 0.056, 0.030)),
        origin=Origin(xyz=(-0.030, 0.000, 0.037)),
        material=navy_rubber,
        name="top_bridge",
    )
    body.visual(
        Box((0.052, 0.044, 0.024)),
        origin=Origin(xyz=(-0.036, 0.000, 0.055)),
        material=navy_rubber,
        name="focus_pedestal",
    )
    body.visual(
        Box((0.046, 0.096, 0.012)),
        origin=Origin(xyz=(-0.040, 0.000, 0.056)),
        material=satin_metal,
        name="focus_saddle",
    )
    body.visual(
        Box((0.033, 0.012, 0.055)),
        origin=Origin(xyz=(-0.040, -0.041, 0.076)),
        material=satin_metal,
        name="focus_cheek_0",
    )
    body.visual(
        Box((0.033, 0.012, 0.055)),
        origin=Origin(xyz=(-0.040, 0.041, 0.076)),
        material=satin_metal,
        name="focus_cheek_1",
    )
    for index, y in enumerate((-0.034, 0.034)):
        body.visual(
            _save_mesh(
                _ring_band(outer_radius=0.012, inner_radius=0.007, length=0.012, radial_segments=36),
                f"focus_bearing_{index}",
            ),
            origin=Origin(xyz=(-0.040, y, 0.095), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=satin_metal,
            name=f"focus_bearing_{index}",
        )

    body.visual(
        Box((0.042, 0.006, 0.010)),
        origin=Origin(xyz=(0.016, 0.000, 0.054)),
        material=label_white,
        name="marine_mark",
    )

    focus_wheel = model.part("focus_wheel")
    focus_wheel.visual(
        _save_mesh(
            KnobGeometry(
                0.054,
                0.044,
                body_style="cylindrical",
                edge_radius=0.0015,
                grip=KnobGrip(style="fluted", count=30, depth=0.0022),
            ),
            "focus_wheel_mesh",
        ),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_rubber,
        name="fluted_wheel",
    )
    focus_wheel.visual(
        Cylinder(radius=0.006, length=0.086),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="focus_axle",
    )

    diopter_ring = model.part("diopter_ring")
    diopter_ring.visual(
        _save_mesh(_ring_band(outer_radius=0.031, inner_radius=0.0225, length=0.024), "diopter_ring_shell"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_rubber,
        name="ring_shell",
    )
    for index in range(24):
        angle = (math.tau * index) / 24.0
        diopter_ring.visual(
            Cylinder(radius=0.0017, length=0.026),
            origin=Origin(
                xyz=(0.0, 0.031 * math.cos(angle), 0.031 * math.sin(angle)),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=black_rubber,
            name=f"grip_rib_{index:02d}",
        )
    diopter_ring.visual(
        Box((0.002, 0.020, 0.004)),
        origin=Origin(xyz=(0.013, 0.000, 0.032)),
        material=label_white,
        name="index_mark",
    )

    model.articulation(
        "body_to_focus_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=focus_wheel,
        origin=Origin(xyz=(-0.040, 0.000, 0.095)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=8.0),
    )
    model.articulation(
        "body_to_diopter_ring",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=diopter_ring,
        origin=Origin(xyz=(-0.098, -sep, 0.000)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.45, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    focus = object_model.get_part("focus_wheel")
    diopter = object_model.get_part("diopter_ring")
    focus_joint = object_model.get_articulation("body_to_focus_wheel")
    diopter_joint = object_model.get_articulation("body_to_diopter_ring")

    objective_visuals = [v for v in body.visuals if v.name and v.name.startswith("objective_housing_")]
    ctx.check(
        "two fixed objective housings",
        len(objective_visuals) == 2 and {focus_joint.child, diopter_joint.child} == {"focus_wheel", "diopter_ring"},
        details=f"objective_count={len(objective_visuals)}",
    )
    ctx.check(
        "focus wheel rotates continuously",
        focus_joint.articulation_type == ArticulationType.CONTINUOUS
        and focus_joint.motion_limits is not None
        and focus_joint.motion_limits.lower is None
        and focus_joint.motion_limits.upper is None,
        details=str(focus_joint),
    )
    ctx.check(
        "diopter ring rotates continuously",
        diopter_joint.articulation_type == ArticulationType.CONTINUOUS
        and diopter_joint.motion_limits is not None
        and diopter_joint.motion_limits.lower is None
        and diopter_joint.motion_limits.upper is None,
        details=str(diopter_joint),
    )

    ctx.expect_within(
        focus,
        body,
        axes="xz",
        inner_elem="focus_axle",
        outer_elem="focus_bearing_0",
        margin=0.0,
        name="focus axle fits first bearing",
    )
    ctx.expect_within(
        focus,
        body,
        axes="xz",
        inner_elem="focus_axle",
        outer_elem="focus_bearing_1",
        margin=0.0,
        name="focus axle fits second bearing",
    )
    ctx.expect_overlap(
        focus,
        body,
        axes="y",
        elem_a="focus_axle",
        elem_b="focus_bearing_0",
        min_overlap=0.008,
        name="focus axle passes through first bearing",
    )
    ctx.expect_overlap(
        focus,
        body,
        axes="y",
        elem_a="focus_axle",
        elem_b="focus_bearing_1",
        min_overlap=0.008,
        name="focus axle passes through second bearing",
    )
    for cheek_name in ("focus_cheek_0", "focus_cheek_1"):
        ctx.allow_overlap(
            body,
            focus,
            elem_a=cheek_name,
            elem_b="focus_axle",
            reason=(
                "The continuous focus wheel shaft is intentionally captured through "
                "a simplified solid side cheek/bearing boss."
            ),
        )
        ctx.expect_within(
            focus,
            body,
            axes="xz",
            inner_elem="focus_axle",
            outer_elem=cheek_name,
            margin=0.0,
            name=f"focus axle is centered in {cheek_name}",
        )
    ctx.expect_within(
        body,
        diopter,
        axes="yz",
        inner_elem="diopter_sleeve",
        outer_elem="ring_shell",
        margin=0.0,
        name="diopter ring surrounds eyepiece sleeve",
    )
    ctx.expect_overlap(
        diopter,
        body,
        axes="x",
        elem_a="ring_shell",
        elem_b="diopter_sleeve",
        min_overlap=0.020,
        name="diopter ring is retained on eyepiece",
    )

    with ctx.pose({focus_joint: math.tau * 1.25, diopter_joint: -math.tau * 0.5}):
        ctx.expect_within(
            focus,
            body,
            axes="xz",
            inner_elem="focus_axle",
            outer_elem="focus_bearing_0",
            margin=0.0,
            name="rotated focus axle remains in bearing",
        )
        ctx.expect_overlap(
            diopter,
            body,
            axes="x",
            elem_a="ring_shell",
            elem_b="diopter_sleeve",
            min_overlap=0.020,
            name="rotated diopter ring remains on sleeve",
        )

    return ctx.report()


object_model = build_object_model()
