from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


BODY_WIDTH = 0.36
BODY_DEPTH = 0.33
SIDE_WALL = 0.018
BODY_TOP = 0.080
GRILL_CENTER_Y = -0.005
HINGE_Y = 0.145
HINGE_Z = 0.105


def _merge_box(
    geom: MeshGeometry,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
) -> None:
    geom.merge(BoxGeometry(size).translate(*center))


def _build_base_shell() -> MeshGeometry:
    geom = MeshGeometry()

    wall_center_z = BODY_TOP * 0.5
    inner_depth = 0.250
    shell_span_x = BODY_WIDTH - SIDE_WALL * 2.0

    _merge_box(geom, (SIDE_WALL, BODY_DEPTH, BODY_TOP), (-0.171, 0.0, wall_center_z))
    _merge_box(geom, (SIDE_WALL, BODY_DEPTH, BODY_TOP), (0.171, 0.0, wall_center_z))
    _merge_box(geom, (shell_span_x, 0.018, BODY_TOP), (0.0, 0.156, wall_center_z))

    _merge_box(geom, (0.308, 0.014, 0.022), (0.0, -0.158, 0.067))
    _merge_box(geom, (0.034, 0.014, 0.024), (-0.146, -0.158, 0.012))
    _merge_box(geom, (0.034, 0.014, 0.024), (0.146, -0.158, 0.012))

    _merge_box(geom, (0.022, inner_depth, 0.010), (-0.132, -0.005, 0.029))
    _merge_box(geom, (0.022, inner_depth, 0.010), (0.132, -0.005, 0.029))
    _merge_box(geom, (0.300, 0.110, 0.012), (0.0, 0.070, 0.020))

    _merge_box(geom, (0.018, 0.250, 0.010), (-0.150, GRILL_CENTER_Y, 0.067))
    _merge_box(geom, (0.018, 0.250, 0.010), (0.150, GRILL_CENTER_Y, 0.067))
    _merge_box(geom, (0.300, 0.018, 0.010), (0.0, 0.120, 0.067))
    _merge_box(geom, (0.300, 0.018, 0.010), (0.0, -0.130, 0.067))

    _merge_box(geom, (0.040, 0.034, 0.020), (-0.110, 0.150, 0.089))
    _merge_box(geom, (0.040, 0.034, 0.020), (0.110, 0.150, 0.089))

    return geom


def _build_base_feet() -> MeshGeometry:
    geom = MeshGeometry()
    foot_size = (0.046, 0.038, 0.008)
    for x_pos in (-0.120, 0.120):
        for y_pos in (-0.105, 0.105):
            _merge_box(geom, foot_size, (x_pos, y_pos, 0.004))
    return geom


def _build_grill(
    *,
    width: float,
    depth: float,
    plate_center: tuple[float, float, float],
    plate_thickness: float,
    ridge_count: int,
    ridge_height: float,
    ridge_width: float,
    ridge_embed: float,
    downward: bool,
) -> MeshGeometry:
    geom = MeshGeometry()
    cx, cy, cz = plate_center
    _merge_box(geom, (width, depth, plate_thickness), plate_center)

    span = width * 0.78
    ridge_depth = depth * 0.96
    if ridge_count == 1:
        x_positions = [cx]
    else:
        x_positions = [
            cx - span * 0.5 + span * index / (ridge_count - 1)
            for index in range(ridge_count)
        ]

    for x_pos in x_positions:
        ridge_center_z = (
            cz - plate_thickness * 0.5 - ridge_height * 0.5 + ridge_embed
            if downward
            else cz + plate_thickness * 0.5 + ridge_height * 0.5 - ridge_embed
        )
        _merge_box(
            geom,
            (ridge_width, ridge_depth, ridge_height),
            (x_pos, cy, ridge_center_z),
        )

    return geom


def _build_upper_shell() -> MeshGeometry:
    geom = MeshGeometry()

    _merge_box(geom, (0.336, 0.258, 0.030), (0.0, -0.129, 0.045))
    _merge_box(geom, (0.016, 0.250, 0.048), (-0.160, -0.125, 0.014))
    _merge_box(geom, (0.016, 0.250, 0.048), (0.160, -0.125, 0.014))
    _merge_box(geom, (0.304, 0.020, 0.050), (0.0, -0.260, 0.016))
    _merge_box(geom, (0.336, 0.028, 0.040), (0.0, -0.012, 0.024))
    return geom


def _build_upper_handle() -> MeshGeometry:
    geom = tube_from_spline_points(
        [
            (-0.105, -0.283, 0.002),
            (-0.105, -0.312, 0.030),
            (0.105, -0.312, 0.030),
            (0.105, -0.283, 0.002),
        ],
        radius=0.008,
        samples_per_segment=18,
        radial_segments=20,
        cap_ends=True,
        up_hint=(0.0, 0.0, 1.0),
    )
    _merge_box(geom, (0.016, 0.022, 0.018), (-0.105, -0.276, 0.006))
    _merge_box(geom, (0.016, 0.022, 0.018), (0.105, -0.276, 0.006))
    return geom


def _build_drawer() -> MeshGeometry:
    geom = MeshGeometry()
    _merge_box(geom, (0.244, 0.165, 0.004), (0.0, 0.0825, 0.002))
    _merge_box(geom, (0.006, 0.153, 0.020), (-0.119, 0.079, 0.010))
    _merge_box(geom, (0.006, 0.153, 0.020), (0.119, 0.079, 0.010))
    _merge_box(geom, (0.238, 0.006, 0.020), (0.0, 0.162, 0.010))
    _merge_box(geom, (0.248, 0.012, 0.028), (0.0, -0.006, 0.014))
    _merge_box(geom, (0.082, 0.012, 0.008), (0.0, -0.015, 0.017))
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cafe_panini_press")

    body_dark = model.material("body_dark", rgba=(0.18, 0.19, 0.20, 1.0))
    cast_iron = model.material("cast_iron", rgba=(0.23, 0.24, 0.24, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    knob_black = model.material("knob_black", rgba=(0.08, 0.08, 0.09, 1.0))
    handle_black = model.material("handle_black", rgba=(0.10, 0.10, 0.11, 1.0))

    base = model.part("base")
    base.visual(
        Box((SIDE_WALL, BODY_DEPTH, BODY_TOP)),
        origin=Origin(xyz=(-0.171, 0.0, BODY_TOP * 0.5)),
        material=body_dark,
        name="left_wall",
    )
    base.visual(
        Box((SIDE_WALL, BODY_DEPTH, BODY_TOP)),
        origin=Origin(xyz=(0.171, 0.0, BODY_TOP * 0.5)),
        material=body_dark,
        name="right_wall",
    )
    base.visual(
        Box((BODY_WIDTH - SIDE_WALL * 2.0, 0.018, BODY_TOP)),
        origin=Origin(xyz=(0.0, 0.156, BODY_TOP * 0.5)),
        material=body_dark,
        name="rear_wall",
    )
    base.visual(
        Box((0.324, 0.014, 0.022)),
        origin=Origin(xyz=(0.0, -0.158, 0.067)),
        material=body_dark,
        name="front_upper_fascia",
    )
    base.visual(
        Box((0.034, 0.014, 0.024)),
        origin=Origin(xyz=(-0.146, -0.158, 0.012)),
        material=body_dark,
        name="front_left_cheek",
    )
    base.visual(
        Box((0.034, 0.014, 0.024)),
        origin=Origin(xyz=(0.146, -0.158, 0.012)),
        material=body_dark,
        name="front_right_cheek",
    )
    base.visual(
        Box((0.040, 0.250, 0.010)),
        origin=Origin(xyz=(-0.142, -0.005, 0.029)),
        material=body_dark,
        name="left_drawer_guide",
    )
    base.visual(
        Box((0.040, 0.250, 0.010)),
        origin=Origin(xyz=(0.142, -0.005, 0.029)),
        material=body_dark,
        name="right_drawer_guide",
    )
    base.visual(
        Box((0.300, 0.110, 0.012)),
        origin=Origin(xyz=(0.0, 0.070, 0.020)),
        material=body_dark,
        name="rear_floor",
    )
    base.visual(
        Box((0.024, 0.250, 0.010)),
        origin=Origin(xyz=(-0.150, GRILL_CENTER_Y, 0.067)),
        material=body_dark,
        name="left_grill_ledge",
    )
    base.visual(
        Box((0.024, 0.250, 0.010)),
        origin=Origin(xyz=(0.150, GRILL_CENTER_Y, 0.067)),
        material=body_dark,
        name="right_grill_ledge",
    )
    base.visual(
        Box((0.324, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, 0.120, 0.067)),
        material=body_dark,
        name="rear_grill_ledge",
    )
    base.visual(
        Box((0.324, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, -0.130, 0.067)),
        material=body_dark,
        name="front_grill_ledge",
    )
    base.visual(
        Box((0.040, 0.034, 0.020)),
        origin=Origin(xyz=(-0.110, 0.150, 0.089)),
        material=body_dark,
        name="left_hinge_support",
    )
    base.visual(
        Box((0.040, 0.034, 0.020)),
        origin=Origin(xyz=(0.110, 0.150, 0.089)),
        material=body_dark,
        name="right_hinge_support",
    )
    for foot_index, (foot_x, foot_y) in enumerate(
        ((-0.150, -0.140), (0.150, -0.140), (-0.150, 0.145), (0.150, 0.145))
    ):
        base.visual(
            Box((0.040, 0.030, 0.010)),
            origin=Origin(xyz=(foot_x, foot_y, 0.005)),
            material=knob_black,
            name=f"foot_{foot_index}",
        )
    base.visual(
        Box((0.300, 0.250, 0.010)),
        origin=Origin(xyz=(0.0, GRILL_CENTER_Y, 0.073)),
        material=cast_iron,
        name="lower_grill",
    )
    for ridge_index, ridge_x in enumerate((-0.117, -0.078, -0.039, 0.0, 0.039, 0.078, 0.117)):
        base.visual(
            Box((0.020, 0.240, 0.008)),
            origin=Origin(xyz=(ridge_x, GRILL_CENTER_Y, 0.080)),
            material=cast_iron,
            name=f"lower_ridge_{ridge_index}",
        )
    base.visual(
        Cylinder(radius=0.018, length=0.008),
        origin=Origin(xyz=(0.184, 0.040, 0.052), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=body_dark,
        name="control_boss",
    )

    upper_platen = model.part("upper_platen")
    upper_platen.visual(
        mesh_from_geometry(_build_upper_shell(), "panini_press_upper_shell"),
        material=body_dark,
        name="upper_shell",
    )
    upper_platen.visual(
        Box((0.320, 0.244, 0.010)),
        origin=Origin(xyz=(0.0, -0.137, -0.004)),
        material=cast_iron,
        name="upper_grill",
    )
    for ridge_index, ridge_x in enumerate((-0.114, -0.076, -0.038, 0.0, 0.038, 0.076, 0.114)):
        upper_platen.visual(
            Box((0.019, 0.228, 0.010)),
            origin=Origin(xyz=(ridge_x, -0.137, -0.012)),
            material=cast_iron,
            name=f"upper_ridge_{ridge_index}",
        )
    upper_platen.visual(
        Cylinder(radius=0.008, length=0.250),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="hinge_barrel",
    )
    upper_platen.visual(
        mesh_from_geometry(_build_upper_handle(), "panini_press_handle"),
        material=handle_black,
        name="handle",
    )

    grease_drawer = model.part("grease_drawer")
    grease_drawer.visual(
        mesh_from_geometry(_build_drawer(), "panini_press_drawer"),
        material=satin_steel,
        name="drawer_tray",
    )

    browning_knob = model.part("browning_knob")
    browning_knob.visual(
        Cylinder(radius=0.020, length=0.020),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_black,
        name="knob_body",
    )
    browning_knob.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(xyz=(0.023, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_black,
        name="knob_cap",
    )
    browning_knob.visual(
        Box((0.004, 0.008, 0.008)),
        origin=Origin(xyz=(0.025, 0.010, 0.0)),
        material=satin_steel,
        name="knob_indicator",
    )

    model.articulation(
        "platen_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_platen,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=1.2,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=grease_drawer,
        origin=Origin(xyz=(0.0, -0.153, 0.011)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.18,
            lower=0.0,
            upper=0.085,
        ),
    )
    model.articulation(
        "knob_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=browning_knob,
        origin=Origin(xyz=(0.188, 0.040, 0.052)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=8.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    upper_platen = object_model.get_part("upper_platen")
    grease_drawer = object_model.get_part("grease_drawer")
    browning_knob = object_model.get_part("browning_knob")

    platen_hinge = object_model.get_articulation("platen_hinge")
    drawer_slide = object_model.get_articulation("drawer_slide")
    knob_spin = object_model.get_articulation("knob_spin")

    ctx.expect_overlap(
        upper_platen,
        base,
        axes="xy",
        elem_a="upper_grill",
        elem_b="lower_grill",
        min_overlap=0.20,
        name="upper and lower grill plates align in plan",
    )
    ctx.expect_gap(
        upper_platen,
        base,
        axis="z",
        positive_elem="upper_grill",
        negative_elem="lower_grill",
        min_gap=0.002,
        max_gap=0.020,
        name="closed platens sit just above each other",
    )
    ctx.expect_contact(
        browning_knob,
        base,
        elem_a="knob_body",
        elem_b="control_boss",
        name="browning knob mounts on the right side wall",
    )
    ctx.expect_within(
        grease_drawer,
        base,
        axes="x",
        margin=0.010,
        name="grease drawer stays laterally centered under the grill",
    )
    ctx.check(
        "browning control uses continuous rotation",
        knob_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint type={knob_spin.articulation_type}",
    )

    closed_upper_aabb = ctx.part_world_aabb(upper_platen)
    rest_drawer_pos = ctx.part_world_position(grease_drawer)
    hinge_upper = platen_hinge.motion_limits.upper if platen_hinge.motion_limits is not None else None
    drawer_upper = drawer_slide.motion_limits.upper if drawer_slide.motion_limits is not None else None

    with ctx.pose({platen_hinge: hinge_upper if hinge_upper is not None else 1.0}):
        open_upper_aabb = ctx.part_world_aabb(upper_platen)

    with ctx.pose({drawer_slide: drawer_upper if drawer_upper is not None else 0.085}):
        extended_drawer_pos = ctx.part_world_position(grease_drawer)
        ctx.expect_overlap(
            grease_drawer,
            base,
            axes="y",
            min_overlap=0.070,
            name="grease drawer remains retained when extended",
        )
        ctx.expect_within(
            grease_drawer,
            base,
            axes="x",
            margin=0.012,
            name="grease drawer remains guided at full extension",
        )

    closed_max_z = closed_upper_aabb[1][2] if closed_upper_aabb is not None else None
    open_max_z = open_upper_aabb[1][2] if open_upper_aabb is not None else None
    ctx.check(
        "upper platen opens upward",
        (
            closed_max_z is not None
            and open_max_z is not None
            and open_max_z > closed_max_z + 0.10
        ),
        details=f"closed_max_z={closed_max_z}, open_max_z={open_max_z}",
    )

    rest_drawer_y = rest_drawer_pos[1] if rest_drawer_pos is not None else None
    extended_drawer_y = extended_drawer_pos[1] if extended_drawer_pos is not None else None
    ctx.check(
        "grease drawer slides out from the front",
        (
            rest_drawer_y is not None
            and extended_drawer_y is not None
            and extended_drawer_y < rest_drawer_y - 0.05
        ),
        details=f"rest_drawer_y={rest_drawer_y}, extended_drawer_y={extended_drawer_y}",
    )

    return ctx.report()


object_model = build_object_model()
