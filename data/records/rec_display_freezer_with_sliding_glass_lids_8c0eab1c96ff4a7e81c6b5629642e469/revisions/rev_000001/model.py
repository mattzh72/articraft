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
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _curved_lid_glass(
    *,
    length: float,
    depth: float,
    rise: float,
    edge_height: float = 0.018,
    thickness: float = 0.006,
    x_segments: int = 3,
    y_segments: int = 28,
) -> MeshGeometry:
    """Thin closed barrel-vault mesh for one sliding glass freezer lid."""
    geom = MeshGeometry()

    def z_profile(y: float) -> float:
        # Parabolic vault: low at the guide-track edges, highest at mid-depth.
        t = max(-1.0, min(1.0, 2.0 * y / depth))
        return edge_height + rise * (1.0 - t * t)

    xs = [-length / 2.0 + length * i / x_segments for i in range(x_segments + 1)]
    ys = [-depth / 2.0 + depth * j / y_segments for j in range(y_segments + 1)]
    top: list[list[int]] = []
    bottom: list[list[int]] = []
    for x in xs:
        top_row: list[int] = []
        bottom_row: list[int] = []
        for y in ys:
            z = z_profile(y)
            top_row.append(geom.add_vertex(x, y, z + thickness / 2.0))
            bottom_row.append(geom.add_vertex(x, y, z - thickness / 2.0))
        top.append(top_row)
        bottom.append(bottom_row)

    for i in range(x_segments):
        for j in range(y_segments):
            # Top skin.
            geom.add_face(top[i][j], top[i + 1][j], top[i + 1][j + 1])
            geom.add_face(top[i][j], top[i + 1][j + 1], top[i][j + 1])
            # Bottom skin, reversed.
            geom.add_face(bottom[i][j], bottom[i][j + 1], bottom[i + 1][j + 1])
            geom.add_face(bottom[i][j], bottom[i + 1][j + 1], bottom[i + 1][j])

    # Long edge faces.
    for i in range(x_segments):
        for j in (0, y_segments):
            geom.add_face(bottom[i][j], bottom[i + 1][j], top[i + 1][j])
            geom.add_face(bottom[i][j], top[i + 1][j], top[i][j])

    # End faces.
    for i in (0, x_segments):
        for j in range(y_segments):
            geom.add_face(bottom[i][j], top[i][j], top[i][j + 1])
            geom.add_face(bottom[i][j], top[i][j + 1], bottom[i][j + 1])

    return geom


def _add_sliding_lid(
    model: ArticulatedObject,
    *,
    part_name: str,
    glass_mesh_name: str,
    length: float,
    depth: float,
    rise: float,
    handle_x_sign: float,
    glass: Material,
    aluminum: Material,
    gasket: Material,
):
    lid = model.part(part_name)
    lid.visual(
        mesh_from_geometry(
            _curved_lid_glass(length=length, depth=depth, rise=rise),
            glass_mesh_name,
        ),
        material=glass,
        name="curved_glass",
    )
    runner_h = 0.026
    runner_w = 0.024
    lid.visual(
        Box((length, runner_w, runner_h)),
        origin=Origin(xyz=(0.0, -depth / 2.0, runner_h / 2.0)),
        material=aluminum,
        name="front_runner",
    )
    lid.visual(
        Box((length, runner_w, runner_h)),
        origin=Origin(xyz=(0.0, depth / 2.0, runner_h / 2.0)),
        material=aluminum,
        name="rear_runner",
    )
    # End hoops read as the thicker aluminum caps that tie both runners to the
    # curved pane.  They intentionally intersect the glass edge within the part.
    lid.visual(
        Box((0.028, depth + 0.030, 0.060)),
        origin=Origin(xyz=(-length / 2.0, 0.0, 0.060)),
        material=aluminum,
        name="end_frame_0",
    )
    lid.visual(
        Box((0.028, depth + 0.030, 0.060)),
        origin=Origin(xyz=(length / 2.0, 0.0, 0.060)),
        material=aluminum,
        name="end_frame_1",
    )
    lid.visual(
        Box((0.035, min(0.38, depth * 0.62), 0.042)),
        origin=Origin(xyz=(handle_x_sign * (length / 2.0 - 0.022), 0.0, 0.086)),
        material=gasket,
        name="pull_grip",
    )
    lid.visual(
        Box((0.018, depth + 0.035, 0.030)),
        origin=Origin(xyz=(handle_x_sign * (length / 2.0 - 0.006), 0.0, 0.055)),
        material=gasket,
        name="edge_gasket",
    )
    return lid


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="curved_lid_display_freezer")

    white = model.material("warm_white_powder_coat", rgba=(0.92, 0.94, 0.90, 1.0))
    liner = model.material("pale_blue_liner", rgba=(0.78, 0.90, 0.95, 1.0))
    aluminum = model.material("brushed_aluminum", rgba=(0.70, 0.72, 0.70, 1.0))
    black = model.material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    dark = model.material("dark_control_glass", rgba=(0.04, 0.05, 0.055, 1.0))
    green = model.material("green_display", rgba=(0.10, 0.85, 0.38, 1.0))
    amber = model.material("amber_indicator", rgba=(1.0, 0.58, 0.10, 1.0))
    glass = model.material("slightly_blue_glass", rgba=(0.55, 0.82, 1.0, 0.38))

    tub = model.part("insulated_tub")

    # Real chest-display-freezer proportions: a waist-high insulated tub with an
    # open refrigerated well and a raised metal frame around the top.
    tub.visual(
        Box((1.62, 0.74, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=black,
        name="recessed_plinth",
    )
    tub.visual(
        Box((1.70, 0.82, 0.100)),
        origin=Origin(xyz=(0.0, 0.0, 0.130)),
        material=white,
        name="insulated_bottom",
    )
    tub.visual(
        Box((1.70, 0.070, 0.630)),
        origin=Origin(xyz=(0.0, -0.375, 0.495)),
        material=white,
        name="front_wall",
    )
    tub.visual(
        Box((1.70, 0.070, 0.630)),
        origin=Origin(xyz=(0.0, 0.375, 0.495)),
        material=white,
        name="rear_wall",
    )
    tub.visual(
        Box((0.070, 0.82, 0.630)),
        origin=Origin(xyz=(-0.815, 0.0, 0.495)),
        material=white,
        name="side_wall_0",
    )
    tub.visual(
        Box((0.070, 0.82, 0.630)),
        origin=Origin(xyz=(0.815, 0.0, 0.495)),
        material=white,
        name="side_wall_1",
    )
    tub.visual(
        Box((1.50, 0.67, 0.015)),
        origin=Origin(xyz=(0.0, 0.0, 0.190)),
        material=liner,
        name="well_floor_liner",
    )
    tub.visual(
        Box((1.50, 0.014, 0.560)),
        origin=Origin(xyz=(0.0, -0.333, 0.475)),
        material=liner,
        name="front_liner",
    )
    tub.visual(
        Box((1.50, 0.014, 0.560)),
        origin=Origin(xyz=(0.0, 0.333, 0.475)),
        material=liner,
        name="rear_liner",
    )
    tub.visual(
        Box((0.014, 0.62, 0.560)),
        origin=Origin(xyz=(-0.742, 0.0, 0.475)),
        material=liner,
        name="side_liner_0",
    )
    tub.visual(
        Box((0.014, 0.62, 0.560)),
        origin=Origin(xyz=(0.742, 0.0, 0.475)),
        material=liner,
        name="side_liner_1",
    )

    # Aluminum top frame and guide rails for the two sliding barrel-vault lids.
    tub.visual(
        Box((1.74, 0.090, 0.065)),
        origin=Origin(xyz=(0.0, -0.365, 0.842)),
        material=aluminum,
        name="front_top_frame",
    )
    tub.visual(
        Box((1.74, 0.090, 0.065)),
        origin=Origin(xyz=(0.0, 0.365, 0.842)),
        material=aluminum,
        name="rear_top_frame",
    )
    tub.visual(
        Box((0.090, 0.82, 0.065)),
        origin=Origin(xyz=(-0.825, 0.0, 0.842)),
        material=aluminum,
        name="end_top_frame_0",
    )
    tub.visual(
        Box((0.090, 0.82, 0.065)),
        origin=Origin(xyz=(0.825, 0.0, 0.842)),
        material=aluminum,
        name="end_top_frame_1",
    )
    tub.visual(
        Box((1.60, 0.025, 0.016)),
        origin=Origin(xyz=(0.0, -0.337, 0.876)),
        material=black,
        name="front_lower_track",
    )
    tub.visual(
        Box((1.60, 0.025, 0.016)),
        origin=Origin(xyz=(0.0, 0.337, 0.876)),
        material=black,
        name="rear_lower_track",
    )
    tub.visual(
        Box((1.60, 0.030, 0.026)),
        origin=Origin(xyz=(0.0, -0.390, 0.886)),
        material=aluminum,
        name="front_upper_rail",
    )
    tub.visual(
        Box((1.60, 0.030, 0.026)),
        origin=Origin(xyz=(0.0, 0.390, 0.886)),
        material=aluminum,
        name="rear_upper_rail",
    )
    tub.visual(
        Box((1.60, 0.024, 0.010)),
        origin=Origin(xyz=(0.0, -0.390, 0.904)),
        material=black,
        name="front_upper_track",
    )
    tub.visual(
        Box((1.60, 0.024, 0.010)),
        origin=Origin(xyz=(0.0, 0.390, 0.904)),
        material=black,
        name="rear_upper_track",
    )

    # Front control section: fixed fascia details, with the hinged service trim
    # panel mounted below it.
    tub.visual(
        Box((0.74, 0.018, 0.140)),
        origin=Origin(xyz=(0.0, -0.419, 0.655)),
        material=dark,
        name="control_fascia",
    )
    tub.visual(
        Box((0.150, 0.006, 0.045)),
        origin=Origin(xyz=(-0.205, -0.431, 0.668)),
        material=green,
        name="temperature_display",
    )
    tub.visual(
        Box((0.035, 0.006, 0.035)),
        origin=Origin(xyz=(0.055, -0.431, 0.668)),
        material=amber,
        name="power_indicator",
    )
    tub.visual(
        Box((0.055, 0.006, 0.028)),
        origin=Origin(xyz=(0.155, -0.431, 0.668)),
        material=black,
        name="label_plate_0",
    )
    tub.visual(
        Box((0.055, 0.006, 0.028)),
        origin=Origin(xyz=(0.235, -0.431, 0.668)),
        material=black,
        name="label_plate_1",
    )
    tub.visual(
        Cylinder(radius=0.010, length=0.660),
        origin=Origin(xyz=(0.0, -0.433, 0.306), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="trim_hinge_pin",
    )
    tub.visual(
        Box((0.680, 0.014, 0.020)),
        origin=Origin(xyz=(0.0, -0.417, 0.300)),
        material=aluminum,
        name="hinge_leaf",
    )

    lower_lid = _add_sliding_lid(
        model,
        part_name="front_lid",
        glass_mesh_name="front_lid_curved_glass",
        length=0.82,
        depth=0.674,
        rise=0.105,
        handle_x_sign=1.0,
        glass=glass,
        aluminum=aluminum,
        gasket=black,
    )
    upper_lid = _add_sliding_lid(
        model,
        part_name="rear_lid",
        glass_mesh_name="rear_lid_curved_glass",
        length=0.82,
        depth=0.780,
        rise=0.092,
        handle_x_sign=-1.0,
        glass=glass,
        aluminum=aluminum,
        gasket=black,
    )
    # The rear/upper glass panel is carried on deeper raised shoes so it can
    # slide over the lower panel without the curved panes intersecting.
    upper_lid.visual(
        Box((0.82, 0.024, 0.086)),
        origin=Origin(xyz=(0.0, -0.390, -0.043)),
        material=aluminum,
        name="front_track_shoe",
    )
    upper_lid.visual(
        Box((0.82, 0.024, 0.086)),
        origin=Origin(xyz=(0.0, 0.390, -0.043)),
        material=aluminum,
        name="rear_track_shoe",
    )
    trim = model.part("trim_panel")
    trim.visual(
        Box((0.620, 0.028, 0.255)),
        origin=Origin(xyz=(0.0, -0.014, 0.128)),
        material=white,
        name="panel_face",
    )
    trim.visual(
        Box((0.620, 0.026, 0.026)),
        origin=Origin(xyz=(0.0, -0.027, 0.013)),
        material=aluminum,
        name="lower_hinge_lip",
    )
    trim.visual(
        Box((0.500, 0.006, 0.090)),
        origin=Origin(xyz=(0.0, -0.031, 0.132)),
        material=dark,
        name="vent_recess",
    )
    for idx, z in enumerate((0.105, 0.132, 0.159)):
        trim.visual(
            Box((0.450, 0.006, 0.008)),
            origin=Origin(xyz=(0.0, -0.035, z)),
            material=aluminum,
            name=f"vent_slat_{idx}",
        )

    model.articulation(
        "front_lid_slide",
        ArticulationType.PRISMATIC,
        parent=tub,
        child=lower_lid,
        origin=Origin(xyz=(-0.430, 0.0, 0.884)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.35, lower=0.0, upper=0.36),
    )
    model.articulation(
        "rear_lid_slide",
        ArticulationType.PRISMATIC,
        parent=tub,
        child=upper_lid,
        origin=Origin(xyz=(0.430, 0.0, 0.995)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.35, lower=0.0, upper=0.36),
    )
    model.articulation(
        "trim_panel_hinge",
        ArticulationType.REVOLUTE,
        parent=tub,
        child=trim,
        origin=Origin(xyz=(0.0, -0.410, 0.315)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=0.0, upper=1.30),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tub = object_model.get_part("insulated_tub")
    front_lid = object_model.get_part("front_lid")
    rear_lid = object_model.get_part("rear_lid")
    trim = object_model.get_part("trim_panel")
    front_slide = object_model.get_articulation("front_lid_slide")
    rear_slide = object_model.get_articulation("rear_lid_slide")
    trim_hinge = object_model.get_articulation("trim_panel_hinge")

    ctx.expect_contact(
        front_lid,
        tub,
        elem_a="front_runner",
        elem_b="front_lower_track",
        name="front sliding lid rests in the lower front guide track",
    )
    ctx.expect_contact(
        front_lid,
        tub,
        elem_a="rear_runner",
        elem_b="rear_lower_track",
        name="front sliding lid rests in the lower rear guide track",
    )
    ctx.expect_contact(
        rear_lid,
        tub,
        elem_a="front_track_shoe",
        elem_b="front_upper_track",
        name="rear sliding lid rests in the upper front guide track",
    )
    ctx.expect_contact(
        rear_lid,
        tub,
        elem_a="rear_track_shoe",
        elem_b="rear_upper_track",
        name="rear sliding lid rests in the upper rear guide track",
    )
    ctx.expect_gap(
        tub,
        trim,
        axis="y",
        positive_elem="front_wall",
        negative_elem="panel_face",
        max_gap=0.001,
        max_penetration=0.000001,
        name="trim access panel closes flush against the front wall",
    )

    front_rest = ctx.part_world_position(front_lid)
    rear_rest = ctx.part_world_position(rear_lid)
    with ctx.pose({front_slide: 0.36, rear_slide: 0.36}):
        ctx.expect_gap(
            rear_lid,
            front_lid,
            axis="z",
            positive_elem="curved_glass",
            negative_elem="curved_glass",
            max_penetration=0.000001,
            name="upper curved glass clears the lower sliding glass at full travel",
        )
        front_open = ctx.part_world_position(front_lid)
        rear_open = ctx.part_world_position(rear_lid)
    ctx.check(
        "curved glass lids slide in opposite directions along their tracks",
        front_rest is not None
        and rear_rest is not None
        and front_open is not None
        and rear_open is not None
        and front_open[0] > front_rest[0] + 0.32
        and rear_open[0] < rear_rest[0] - 0.32,
        details=f"front_rest={front_rest}, front_open={front_open}, rear_rest={rear_rest}, rear_open={rear_open}",
    )

    closed_aabb = ctx.part_world_aabb(trim)
    with ctx.pose({trim_hinge: 1.20}):
        folded_aabb = ctx.part_world_aabb(trim)
    ctx.check(
        "lower trim access panel folds outward and downward on its hinge",
        closed_aabb is not None
        and folded_aabb is not None
        and folded_aabb[0][1] < closed_aabb[0][1] - 0.12
        and folded_aabb[1][2] < closed_aabb[1][2] - 0.10,
        details=f"closed_aabb={closed_aabb}, folded_aabb={folded_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
