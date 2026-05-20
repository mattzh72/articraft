from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


CAST_IRON = Material("blued_cast_iron", rgba=(0.10, 0.13, 0.15, 1.0))
SATIN_STEEL = Material("satin_ground_steel", rgba=(0.68, 0.70, 0.68, 1.0))
DARK_STEEL = Material("black_oxide_steel", rgba=(0.015, 0.017, 0.018, 1.0))
OILED_WAY = Material("oiled_machined_way", rgba=(0.42, 0.46, 0.44, 1.0))
BRASS = Material("brass_gib", rgba=(0.72, 0.55, 0.24, 1.0))
RUBBER = Material("dark_rubber", rgba=(0.02, 0.018, 0.015, 1.0))


def _add_box(part, name: str, size, xyz, material) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _add_cylinder(part, name: str, radius: float, length: float, xyz, material, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _dovetail_prism(*, length: float, bottom_width: float, top_width: float, height: float, axis: str) -> MeshGeometry:
    """Small trapezoid rail, bottom on z=0, extruded along X or Y."""
    geom = MeshGeometry()
    if axis == "x":
        ends = (-length / 2.0, length / 2.0)
        pts = []
        for x in ends:
            pts.extend(
                [
                    (x, -bottom_width / 2.0, 0.0),
                    (x, bottom_width / 2.0, 0.0),
                    (x, top_width / 2.0, height),
                    (x, -top_width / 2.0, height),
                ]
            )
    elif axis == "y":
        ends = (-length / 2.0, length / 2.0)
        pts = []
        for y in ends:
            pts.extend(
                [
                    (-bottom_width / 2.0, y, 0.0),
                    (bottom_width / 2.0, y, 0.0),
                    (top_width / 2.0, y, height),
                    (-top_width / 2.0, y, height),
                ]
            )
    else:
        raise ValueError("axis must be 'x' or 'y'")

    for p in pts:
        geom.add_vertex(*p)

    # end caps and four side faces
    for tri in (
        (0, 1, 2),
        (0, 2, 3),
        (4, 6, 5),
        (4, 7, 6),
        (0, 4, 5),
        (0, 5, 1),
        (1, 5, 6),
        (1, 6, 2),
        (2, 6, 7),
        (2, 7, 3),
        (3, 7, 4),
        (3, 4, 0),
    ):
        geom.add_face(*tri)
    return geom


def _add_cap_screw(part, prefix: str, x: float, y: float, top_z: float, *, radius: float = 0.006) -> None:
    head_h = radius * 0.55
    _add_cylinder(
        part,
        f"{prefix}_head",
        radius=radius,
        length=head_h,
        xyz=(x, y, top_z + head_h / 2.0 - 0.0006),
        material=DARK_STEEL,
    )
    _add_cylinder(
        part,
        f"{prefix}_socket",
        radius=radius * 0.42,
        length=0.0014,
        xyz=(x, y, top_z + head_h - 0.0002),
        material=CAST_IRON,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="microscope_translation_table")

    base = model.part("base")
    _add_box(base, "base_plate", (0.360, 0.250, 0.035), (0.0, 0.0, 0.0175), CAST_IRON)

    # Ground X ways: long, broad, and visibly different from the upper Y guide set.
    x_way_mesh = _dovetail_prism(length=0.315, bottom_width=0.030, top_width=0.020, height=0.020, axis="x")
    for i, y in enumerate((-0.077, 0.077)):
        base.visual(
            mesh_from_geometry(x_way_mesh.copy(), f"x_way_mesh_{i}"),
            origin=Origin(xyz=(0.0, y, 0.0345)),
            material=OILED_WAY,
            name=f"x_way_{i}",
        )
    for i, x in enumerate((-0.118, 0.118)):
        for j, y in enumerate((-0.077, 0.077)):
            _add_cap_screw(base, f"x_way_screw_{i}_{j}", x, y, 0.055, radius=0.005)

    for side, x in (("neg", -0.179), ("pos", 0.179)):
        _add_box(base, f"x_stop_{side}", (0.014, 0.072, 0.031), (x, 0.0, 0.0505), DARK_STEEL)
        bumper_x = x + (0.0082 if side == "neg" else -0.0082)
        _add_box(base, f"x_bumper_{side}", (0.003, 0.046, 0.018), (bumper_x, 0.0, 0.058), RUBBER)
        _add_cap_screw(base, f"x_stop_screw_{side}", x, 0.0, 0.066, radius=0.0045)

    lower_slide = model.part("lower_slide")
    _add_box(lower_slide, "lower_body", (0.260, 0.220, 0.030), (0.0, 0.0, 0.015), CAST_IRON)
    _add_box(lower_slide, "front_gib", (0.236, 0.010, 0.014), (0.0, -0.110, 0.011), BRASS)
    _add_box(lower_slide, "rear_gib", (0.236, 0.010, 0.014), (0.0, 0.110, 0.011), BRASS)
    for i, y in enumerate((-0.077, 0.077)):
        _add_box(lower_slide, f"x_bearing_pad_{i}", (0.228, 0.026, 0.008), (0.0, y, 0.0005), SATIN_STEEL)
    for i, x in enumerate((-0.098, 0.0, 0.098)):
        _add_cylinder(lower_slide, f"gib_pin_{i}", 0.0032, 0.012, (x, -0.116, 0.014), DARK_STEEL, rpy=(-math.pi / 2.0, 0.0, 0.0))
        _add_cylinder(lower_slide, f"gib_lock_{i}", 0.0032, 0.012, (x, 0.116, 0.014), DARK_STEEL, rpy=(-math.pi / 2.0, 0.0, 0.0))

    # Upper guide ways run along Y, perpendicular to the lower slide ways.
    y_way_mesh = _dovetail_prism(length=0.184, bottom_width=0.024, top_width=0.016, height=0.010, axis="y")
    for i, x in enumerate((-0.050, 0.050)):
        lower_slide.visual(
            mesh_from_geometry(y_way_mesh.copy(), f"y_way_mesh_{i}"),
            origin=Origin(xyz=(x, 0.0, 0.0295)),
            material=SATIN_STEEL,
            name=f"y_way_{i}",
        )
    for side, y in (("neg", -0.116), ("pos", 0.116)):
        _add_box(lower_slide, f"y_stop_{side}", (0.068, 0.012, 0.021), (0.0, y, 0.0405), DARK_STEEL)
        bumper_y = y + (0.0072 if side == "neg" else -0.0072)
        _add_box(lower_slide, f"y_bumper_{side}", (0.045, 0.003, 0.014), (0.0, bumper_y, 0.042), RUBBER)
        _add_cap_screw(lower_slide, f"y_stop_screw_{side}", 0.0, y, 0.051, radius=0.004)

    upper_saddle = model.part("upper_saddle")
    _add_box(upper_saddle, "saddle_body", (0.132, 0.160, 0.028), (0.0, 0.0, 0.014), CAST_IRON)
    _add_box(upper_saddle, "left_gib", (0.010, 0.144, 0.013), (-0.066, 0.0, 0.0105), BRASS)
    _add_box(upper_saddle, "right_gib", (0.010, 0.144, 0.013), (0.066, 0.0, 0.0105), BRASS)
    for i, x in enumerate((-0.050, 0.050)):
        _add_box(upper_saddle, f"y_bearing_pad_{i}", (0.020, 0.138, 0.008), (x, 0.0, 0.0005), SATIN_STEEL)

    top_plate_geom = PerforatedPanelGeometry(
        (0.198, 0.178),
        0.008,
        hole_diameter=0.011,
        pitch=(0.026, 0.026),
        frame=0.018,
        corner_radius=0.006,
        stagger=True,
    )
    upper_saddle.visual(
        mesh_from_geometry(top_plate_geom, "perforated_top_plate_mesh"),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=SATIN_STEEL,
        name="perforated_plate",
    )
    _add_box(upper_saddle, "top_rib_x", (0.162, 0.020, 0.010), (0.0, 0.0, 0.029), DARK_STEEL)
    _add_box(upper_saddle, "top_rib_y", (0.020, 0.142, 0.010), (0.0, 0.0, 0.029), DARK_STEEL)
    for i, x in enumerate((-0.075, 0.075)):
        for j, y in enumerate((-0.062, 0.062)):
            _add_cap_screw(upper_saddle, f"top_screw_{i}_{j}", x, y, 0.036, radius=0.0048)

    model.articulation(
        "x_axis",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lower_slide,
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.08, lower=-0.035, upper=0.035),
    )
    model.articulation(
        "y_axis",
        ArticulationType.PRISMATIC,
        parent=lower_slide,
        child=upper_saddle,
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.08, lower=-0.030, upper=0.030),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lower = object_model.get_part("lower_slide")
    upper = object_model.get_part("upper_saddle")
    x_axis = object_model.get_articulation("x_axis")
    y_axis = object_model.get_articulation("y_axis")

    ctx.check(
        "orthogonal prismatic axes",
        x_axis.articulation_type == ArticulationType.PRISMATIC
        and y_axis.articulation_type == ArticulationType.PRISMATIC
        and tuple(x_axis.axis) == (1.0, 0.0, 0.0)
        and tuple(y_axis.axis) == (0.0, 1.0, 0.0),
        details=f"x={x_axis.articulation_type}/{x_axis.axis}, y={y_axis.articulation_type}/{y_axis.axis}",
    )

    ctx.expect_gap(
        lower,
        base,
        axis="z",
        positive_elem="x_bearing_pad_0",
        negative_elem="x_way_0",
        min_gap=0.0,
        max_gap=0.0005,
        name="lower slide rides on X ways",
    )
    ctx.expect_overlap(
        lower,
        base,
        axes="xy",
        elem_a="x_bearing_pad_0",
        elem_b="x_way_0",
        min_overlap=0.020,
        name="lower bearing pad covers X way",
    )
    ctx.expect_gap(
        upper,
        lower,
        axis="z",
        positive_elem="y_bearing_pad_0",
        negative_elem="y_way_0",
        min_gap=0.0,
        max_gap=0.0005,
        name="upper saddle rides on Y ways",
    )
    ctx.expect_overlap(
        upper,
        lower,
        axes="xy",
        elem_a="y_bearing_pad_0",
        elem_b="y_way_0",
        min_overlap=0.016,
        name="upper bearing pad covers Y way",
    )

    rest_lower = ctx.part_world_position(lower)
    rest_upper = ctx.part_world_position(upper)
    with ctx.pose({x_axis: 0.035, y_axis: 0.030}):
        moved_lower = ctx.part_world_position(lower)
        moved_upper = ctx.part_world_position(upper)
        ctx.expect_gap(
            base,
            lower,
            axis="x",
            positive_elem="x_stop_pos",
            negative_elem="lower_body",
            min_gap=0.003,
            name="positive X stop remains clear at travel limit",
        )
        ctx.expect_gap(
            lower,
            upper,
            axis="y",
            positive_elem="y_stop_pos",
            negative_elem="saddle_body",
            min_gap=0.0,
            max_gap=0.001,
            name="positive Y stop catches saddle at travel limit",
        )
    with ctx.pose({x_axis: -0.035, y_axis: -0.030}):
        ctx.expect_gap(
            lower,
            base,
            axis="x",
            positive_elem="lower_body",
            negative_elem="x_stop_neg",
            min_gap=0.003,
            name="negative X stop remains clear at travel limit",
        )
        ctx.expect_gap(
            upper,
            lower,
            axis="y",
            positive_elem="saddle_body",
            negative_elem="y_stop_neg",
            min_gap=0.0,
            max_gap=0.001,
            name="negative Y stop catches saddle at travel limit",
        )

    ctx.check(
        "slides move in requested positive directions",
        rest_lower is not None
        and moved_lower is not None
        and rest_upper is not None
        and moved_upper is not None
        and moved_lower[0] > rest_lower[0] + 0.025
        and moved_upper[1] > rest_upper[1] + 0.025,
        details=f"rest_lower={rest_lower}, moved_lower={moved_lower}, rest_upper={rest_upper}, moved_upper={moved_upper}",
    )

    return ctx.report()


object_model = build_object_model()
