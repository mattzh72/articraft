from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _tube_mesh(name: str, outer_radius: float, inner_radius: float, length: float):
    """CadQuery annular tube aligned to local +Z and centered at local z=0."""
    tube = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((0.0, 0.0, -0.5 * length))
    )
    return mesh_from_cadquery(tube, name, tolerance=0.0008, angular_tolerance=0.08)


def _rounded_projector_body():
    return (
        cq.Workplane("XY")
        .box(0.260, 0.180, 0.100)
        .edges("|Z")
        .fillet(0.018)
        .edges(">Z")
        .fillet(0.006)
    )


def _radial_origin(radius: float, z: float, angle: float, *, local_y: float = 0.0) -> Origin:
    ca = math.cos(angle)
    sa = math.sin(angle)
    # local +X is radial outward and local +Y is the tangent direction.
    x = radius * ca - local_y * sa
    y = radius * sa + local_y * ca
    return Origin(xyz=(x, y, z), rpy=(0.0, 0.0, angle))


def _add_leg_geometry(part, *, tube: Material, rubber: Material, metal: Material) -> None:
    start = (0.004, 0.0, -0.004)
    end = (0.420, 0.0, -0.630)
    dx = end[0] - start[0]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dz * dz)
    tilt = math.atan2(dx, dz)

    part.visual(
        Cylinder(radius=0.020, length=0.056),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="hinge_knuckle",
    )
    part.visual(
        Cylinder(radius=0.014, length=length),
        origin=Origin(
            xyz=((start[0] + end[0]) * 0.5, 0.0, (start[2] + end[2]) * 0.5),
            rpy=(0.0, tilt, 0.0),
        ),
        material=tube,
        name="leg_tube",
    )
    part.visual(
        Box((0.140, 0.060, 0.035)),
        origin=Origin(xyz=(end[0], 0.0, -0.6375)),
        material=rubber,
        name="foot_pad",
    )


def _add_crown_hinge(crown, *, angle: float, metal: Material) -> None:
    # A crown-side yoke: radial bridge plus two cheeks.  The open center between
    # the cheeks is where each leg's hinge knuckle sits.
    for side, local_y in (("a", -0.036), ("b", 0.036)):
        crown.visual(
            Box((0.070, 0.016, 0.060)),
            origin=_radial_origin(0.094, 0.000, angle, local_y=local_y),
            material=metal,
            name=f"hinge_bridge_{int(round(math.degrees(angle))) % 360}_{side}",
        )
    for side, local_y in (("a", -0.036), ("b", 0.036)):
        crown.visual(
            Box((0.038, 0.016, 0.065)),
            origin=_radial_origin(0.148, -0.018, angle, local_y=local_y),
            material=metal,
            name=f"hinge_cheek_{int(round(math.degrees(angle))) % 360}_{side}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tripod_projector_stand")

    satin_black = model.material("satin_black", rgba=(0.02, 0.025, 0.028, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.07, 0.075, 0.080, 1.0))
    graphite = model.material("graphite", rgba=(0.18, 0.19, 0.20, 1.0))
    brushed_metal = model.material("brushed_metal", rgba=(0.58, 0.60, 0.62, 1.0))
    rubber = model.material("soft_rubber", rgba=(0.025, 0.025, 0.025, 1.0))
    lens_glass = model.material("blue_lens_glass", rgba=(0.18, 0.34, 0.48, 0.55))
    projector_white = model.material("projector_warm_white", rgba=(0.86, 0.85, 0.80, 1.0))
    vent_black = model.material("vent_black", rgba=(0.01, 0.01, 0.012, 1.0))

    crown = model.part("crown")
    crown.visual(
        _tube_mesh("crown_ring", 0.088, 0.033, 0.060),
        material=brushed_metal,
        name="crown_ring",
    )
    crown.visual(
        _tube_mesh("column_sleeve", 0.040, 0.026, 0.220),
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        material=brushed_metal,
        name="column_sleeve",
    )
    crown.visual(
        _tube_mesh("sleeve_lip", 0.045, 0.026, 0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.225)),
        material=graphite,
        name="sleeve_lip",
    )
    for theta in (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0):
        _add_crown_hinge(crown, angle=theta, metal=brushed_metal)

    # Three folding tripod legs, each in a yawed local hinge frame.
    hinge_radius = 0.148
    hinge_z = -0.018
    for index, theta in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        leg = model.part(f"leg_{index}")
        _add_leg_geometry(leg, tube=satin_black, rubber=rubber, metal=brushed_metal)
        model.articulation(
            f"crown_to_leg_{index}",
            ArticulationType.REVOLUTE,
            parent=crown,
            child=leg,
            origin=_radial_origin(hinge_radius, hinge_z, theta),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=-1.05, upper=0.22),
        )

    center_column = model.part("center_column")
    center_column.visual(
        Cylinder(radius=0.020, length=0.900),
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        material=satin_black,
        name="mast",
    )
    center_column.visual(
        Cylinder(radius=0.047, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=graphite,
        name="lift_collar",
    )
    center_column.visual(
        Cylinder(radius=0.041, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.5825)),
        material=graphite,
        name="top_cap",
    )
    center_column.visual(
        Box((0.055, 0.018, 0.030)),
        origin=Origin(xyz=(0.0, -0.026, 0.400)),
        material=brushed_metal,
        name="height_scale_tab",
    )
    model.articulation(
        "crown_to_center_column",
        ArticulationType.PRISMATIC,
        parent=crown,
        child=center_column,
        origin=Origin(xyz=(0.0, 0.0, 0.220)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.18, lower=0.0, upper=0.250),
    )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.045, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=graphite,
        name="pan_bearing",
    )
    pan_head.visual(
        Box((0.045, 0.045, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=graphite,
        name="neck_block",
    )
    pan_head.visual(
        Box((0.085, 0.190, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=brushed_metal,
        name="yoke_crossbar",
    )
    for side_name, y in (("a", -0.100), ("b", 0.100)):
        pan_head.visual(
            Box((0.048, 0.014, 0.090)),
            origin=Origin(xyz=(0.0, y, 0.120)),
            material=brushed_metal,
            name=f"yoke_cheek_{side_name}",
        )
    model.articulation(
        "center_column_to_pan_head",
        ArticulationType.REVOLUTE,
        parent=center_column,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 0.595)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.3, lower=-math.pi, upper=math.pi),
    )

    tilt_plate = model.part("tilt_plate")
    tilt_plate.visual(
        Cylinder(radius=0.018, length=0.186),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed_metal,
        name="tilt_barrel",
    )
    tilt_plate.visual(
        Box((0.090, 0.120, 0.014)),
        origin=Origin(xyz=(0.045, 0.0, 0.0)),
        material=brushed_metal,
        name="hinge_tongue",
    )
    tilt_plate.visual(
        Box((0.280, 0.220, 0.018)),
        origin=Origin(xyz=(0.200, 0.0, 0.000)),
        material=graphite,
        name="support_plate",
    )
    tilt_plate.visual(
        mesh_from_cadquery(_rounded_projector_body(), "projector_body", tolerance=0.0008),
        origin=Origin(xyz=(0.200, 0.0, 0.059)),
        material=projector_white,
        name="projector_body",
    )
    tilt_plate.visual(
        Cylinder(radius=0.037, length=0.050),
        origin=Origin(xyz=(0.350, 0.0, 0.060), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="lens_barrel",
    )
    tilt_plate.visual(
        Cylinder(radius=0.030, length=0.006),
        origin=Origin(xyz=(0.378, 0.0, 0.060), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_glass,
        name="front_glass",
    )
    for slot_index, y in enumerate((-0.045, -0.025, -0.005, 0.015, 0.035)):
        tilt_plate.visual(
            Box((0.080, 0.006, 0.003)),
            origin=Origin(xyz=(0.175, y, 0.1105)),
            material=vent_black,
            name=f"top_vent_{slot_index}",
        )
    tilt_plate.visual(
        Box((0.008, 0.060, 0.028)),
        origin=Origin(xyz=(0.068, -0.094, 0.045)),
        material=vent_black,
        name="side_vent",
    )
    model.articulation(
        "pan_head_to_tilt_plate",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=tilt_plate,
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=-0.35, upper=0.45),
    )

    side_knob = model.part("side_knob")
    side_knob.visual(
        Cylinder(radius=0.007, length=0.030),
        origin=Origin(xyz=(0.0, 0.015, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed_metal,
        name="short_shaft",
    )
    side_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.052,
                0.032,
                body_style="lobed",
                grip=KnobGrip(style="ribbed", count=18, depth=0.0011),
                indicator=KnobIndicator(style="dot", mode="raised", angle_deg=0.0),
            ),
            "tilt_adjustment_knob",
        ),
        origin=Origin(xyz=(0.0, 0.044, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="knob_cap",
    )
    model.articulation(
        "pan_head_to_side_knob",
        ArticulationType.CONTINUOUS,
        parent=pan_head,
        child=side_knob,
        origin=Origin(xyz=(0.0, 0.107, 0.120)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    crown = object_model.get_part("crown")
    center_column = object_model.get_part("center_column")
    pan_head = object_model.get_part("pan_head")
    tilt_plate = object_model.get_part("tilt_plate")
    side_knob = object_model.get_part("side_knob")

    column_slide = object_model.get_articulation("crown_to_center_column")
    tilt_joint = object_model.get_articulation("pan_head_to_tilt_plate")

    ctx.check(
        "three folding legs",
        all(object_model.get_part(f"leg_{i}") is not None for i in range(3))
        and all(object_model.get_articulation(f"crown_to_leg_{i}") is not None for i in range(3)),
        "Expected three leg parts with crown hinge joints.",
    )

    ctx.expect_within(
        center_column,
        crown,
        axes="xy",
        inner_elem="mast",
        outer_elem="column_sleeve",
        margin=0.002,
        name="mast centered in sleeve",
    )
    ctx.expect_overlap(
        center_column,
        crown,
        axes="z",
        elem_a="mast",
        elem_b="column_sleeve",
        min_overlap=0.120,
        name="mast retained in sleeve at rest",
    )
    ctx.expect_contact(
        pan_head,
        center_column,
        elem_a="pan_bearing",
        elem_b="top_cap",
        contact_tol=0.002,
        name="pan head seated on column cap",
    )
    ctx.expect_overlap(
        tilt_plate,
        pan_head,
        axes="y",
        elem_a="tilt_barrel",
        elem_b="yoke_crossbar",
        min_overlap=0.10,
        name="tilt barrel spans the yoke",
    )
    ctx.expect_contact(
        side_knob,
        pan_head,
        elem_a="short_shaft",
        elem_b="yoke_cheek_b",
        contact_tol=0.002,
        name="side knob shaft seats against yoke cheek",
    )

    rest_column_position = ctx.part_world_position(center_column)
    with ctx.pose({column_slide: 0.250}):
        ctx.expect_within(
            center_column,
            crown,
            axes="xy",
            inner_elem="mast",
            outer_elem="column_sleeve",
            margin=0.002,
            name="extended mast remains centered",
        )
        ctx.expect_overlap(
            center_column,
            crown,
            axes="z",
            elem_a="mast",
            elem_b="column_sleeve",
            min_overlap=0.055,
            name="extended mast remains inserted",
        )
        extended_column_position = ctx.part_world_position(center_column)
    ctx.check(
        "center column slides upward",
        rest_column_position is not None
        and extended_column_position is not None
        and extended_column_position[2] > rest_column_position[2] + 0.20,
        details=f"rest={rest_column_position}, extended={extended_column_position}",
    )

    rest_tilt_aabb = ctx.part_world_aabb(tilt_plate)
    with ctx.pose({tilt_joint: 0.40}):
        tilted_aabb = ctx.part_world_aabb(tilt_plate)
    ctx.check(
        "positive tilt raises projector",
        rest_tilt_aabb is not None
        and tilted_aabb is not None
        and float(tilted_aabb[1][2]) > float(rest_tilt_aabb[1][2]) + 0.025,
        details=f"rest={rest_tilt_aabb}, tilted={tilted_aabb}",
    )

    ctx.check(
        "continuous side adjustment knob present",
        object_model.get_articulation("pan_head_to_side_knob") is not None and side_knob is not None,
        "Expected side knob on a continuous horizontal shaft.",
    )

    return ctx.report()


object_model = build_object_model()
