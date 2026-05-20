from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _triangular_prism(dx: float, dz: float, thickness: float) -> MeshGeometry:
    """Right triangular gusset plate in the local XZ plane, extruded along Y."""
    geom = MeshGeometry()
    y0 = -thickness * 0.5
    y1 = thickness * 0.5
    verts = [
        (0.0, y0, 0.0),
        (dx, y0, 0.0),
        (0.0, y0, dz),
        (0.0, y1, 0.0),
        (dx, y1, 0.0),
        (0.0, y1, dz),
    ]
    ids = [geom.add_vertex(*v) for v in verts]
    # two triangular faces
    geom.add_face(ids[0], ids[1], ids[2])
    geom.add_face(ids[3], ids[5], ids[4])
    # rectangular edge faces
    geom.add_face(ids[0], ids[3], ids[4])
    geom.add_face(ids[0], ids[4], ids[1])
    geom.add_face(ids[1], ids[4], ids[5])
    geom.add_face(ids[1], ids[5], ids[2])
    geom.add_face(ids[2], ids[5], ids[3])
    geom.add_face(ids[2], ids[3], ids[0])
    return geom


def _add_bolt_on_top(part, *, name: str, x: float, y: float, z: float, material) -> None:
    part.visual(
        Cylinder(radius=0.006, length=0.004),
        origin=Origin(xyz=(x, y, z)),
        material=material,
        name=name,
    )


def _add_bolt_on_front(part, *, name: str, x: float, y: float, z: float, material) -> None:
    part.visual(
        Cylinder(radius=0.005, length=0.005),
        origin=Origin(xyz=(x, y, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _add_bolt_on_side(part, *, name: str, x: float, y: float, z: float, material) -> None:
    part.visual(
        Cylinder(radius=0.0045, length=0.005),
        origin=Origin(xyz=(x, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="orthogonal_xz_stage")

    cast_iron = model.material("cast_iron", rgba=(0.16, 0.17, 0.18, 1.0))
    black_oxide = model.material("black_oxide", rgba=(0.025, 0.027, 0.030, 1.0))
    way_steel = model.material("ground_steel", rgba=(0.72, 0.73, 0.70, 1.0))
    machined_aluminum = model.material("machined_aluminum", rgba=(0.58, 0.61, 0.63, 1.0))
    cover_blue = model.material("cover_blue", rgba=(0.10, 0.14, 0.18, 1.0))
    brass = model.material("brass_bronze", rgba=(0.68, 0.48, 0.22, 1.0))
    rubber = model.material("black_rubber", rgba=(0.02, 0.022, 0.020, 1.0))
    warning_red = model.material("stop_red", rgba=(0.70, 0.05, 0.035, 1.0))

    gusset_right_mesh = mesh_from_geometry(_triangular_prism(0.090, 0.170, 0.014), "right_column_gusset")
    gusset_left_mesh = mesh_from_geometry(_triangular_prism(-0.090, 0.170, 0.014), "left_column_gusset")

    base_frame = model.part("base_frame")
    base_frame.visual(
        Box((0.920, 0.420, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=cast_iron,
        name="base_plate",
    )
    base_frame.visual(
        Box((0.920, 0.030, 0.110)),
        origin=Origin(xyz=(0.0, -0.205, 0.055)),
        material=cast_iron,
        name="front_rail",
    )
    base_frame.visual(
        Box((0.920, 0.030, 0.110)),
        origin=Origin(xyz=(0.0, 0.205, 0.055)),
        material=cast_iron,
        name="rear_rail",
    )
    for idx, x in enumerate((-0.445, 0.445)):
        base_frame.visual(
            Box((0.030, 0.420, 0.100)),
            origin=Origin(xyz=(x, 0.0, 0.050)),
            material=cast_iron,
            name=f"end_crossmember_{idx}",
        )
    base_frame.visual(
        Box((0.820, 0.050, 0.052)),
        origin=Origin(xyz=(0.0, 0.0, 0.061)),
        material=cast_iron,
        name="center_web",
    )

    # Horizontal X guide system: two exposed round ways on machined support pads.
    base_frame.visual(
        Box((0.830, 0.038, 0.033)),
        origin=Origin(xyz=(0.0, -0.130, 0.0515)),
        material=cast_iron,
        name="x_way_riser_0",
    )
    base_frame.visual(
        Box((0.830, 0.038, 0.036)),
        origin=Origin(xyz=(0.0, -0.130, 0.086)),
        material=cast_iron,
        name="x_way_pad_0",
    )
    base_frame.visual(
        Box((0.830, 0.022, 0.012)),
        origin=Origin(xyz=(0.0, -0.130, 0.106)),
        material=way_steel,
        name="x_way_saddle_0",
    )
    base_frame.visual(
        Cylinder(radius=0.011, length=0.800),
        origin=Origin(xyz=(0.0, -0.130, 0.119), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=way_steel,
        name="x_round_way_0",
    )
    base_frame.visual(
        Box((0.830, 0.038, 0.033)),
        origin=Origin(xyz=(0.0, 0.130, 0.0515)),
        material=cast_iron,
        name="x_way_riser_1",
    )
    base_frame.visual(
        Box((0.830, 0.038, 0.036)),
        origin=Origin(xyz=(0.0, 0.130, 0.086)),
        material=cast_iron,
        name="x_way_pad_1",
    )
    base_frame.visual(
        Box((0.830, 0.022, 0.012)),
        origin=Origin(xyz=(0.0, 0.130, 0.106)),
        material=way_steel,
        name="x_way_saddle_1",
    )
    base_frame.visual(
        Cylinder(radius=0.011, length=0.800),
        origin=Origin(xyz=(0.0, 0.130, 0.119), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=way_steel,
        name="x_round_way_1",
    )

    # X drive screw and bearing blocks.
    base_frame.visual(
        Cylinder(radius=0.007, length=0.760),
        origin=Origin(xyz=(0.0, 0.0, 0.135), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=way_steel,
        name="x_leadscrew",
    )
    for idx, x in enumerate((-0.405, 0.405)):
        base_frame.visual(
            Box((0.060, 0.075, 0.070)),
            origin=Origin(xyz=(x, 0.0, 0.135)),
            material=black_oxide,
            name=f"x_end_bearing_{idx}",
        )
        base_frame.visual(
            Cylinder(radius=0.015, length=0.012),
            origin=Origin(xyz=(x + (0.037 if x < 0 else -0.037), 0.0, 0.135), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=way_steel,
            name=f"x_bearing_cap_{idx}",
        )
    for idx, x in enumerate((-0.335, 0.335)):
        base_frame.visual(
            Box((0.030, 0.070, 0.054)),
            origin=Origin(xyz=(x, -0.168, 0.121)),
            material=warning_red,
            name=f"x_hard_stop_{idx}",
        )
        base_frame.visual(
            Cylinder(radius=0.006, length=0.025),
            origin=Origin(xyz=(x, -0.140, 0.129), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=way_steel,
            name=f"x_stop_screw_{idx}",
        )

    # Scale strip and tick marks make the front edge read as a machined study fixture.
    base_frame.visual(
        Box((0.650, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, -0.222, 0.115)),
        material=way_steel,
        name="x_scale_strip",
    )
    for idx, x in enumerate((-0.300, -0.240, -0.180, -0.120, -0.060, 0.0, 0.060, 0.120, 0.180, 0.240, 0.300)):
        base_frame.visual(
            Box((0.004, 0.006, 0.016 if idx % 2 == 0 else 0.010)),
            origin=Origin(xyz=(x, -0.225, 0.120)),
            material=black_oxide,
            name=f"x_scale_tick_{idx}",
        )

    for idx, (x, y) in enumerate(
        [
            (-0.360, -0.175),
            (-0.360, 0.175),
            (0.360, -0.175),
            (0.360, 0.175),
            (-0.120, -0.175),
            (-0.120, 0.175),
            (0.120, -0.175),
            (0.120, 0.175),
        ]
    ):
        _add_bolt_on_top(base_frame, name=f"base_bolt_{idx}", x=x, y=y, z=0.037, material=way_steel)

    x_carriage = model.part("x_carriage")
    x_carriage.visual(
        Box((0.340, 0.300, 0.034)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=machined_aluminum,
        name="x_carriage_plate",
    )
    for idx, x in enumerate((-0.110, 0.110)):
        for jdx, y in enumerate((-0.130, 0.130)):
            x_carriage.visual(
                Box((0.082, 0.050, 0.036)),
                origin=Origin(xyz=(x, y, -0.018)),
                material=black_oxide,
                name=f"x_bearing_truck_{idx}_{jdx}",
            )
            x_carriage.visual(
                Box((0.010, 0.052, 0.026)),
                origin=Origin(xyz=(x - 0.047, y, -0.020)),
                material=rubber,
                name=f"x_wiper_outer_{idx}_{jdx}",
            )
            x_carriage.visual(
                Box((0.010, 0.052, 0.026)),
                origin=Origin(xyz=(x + 0.047, y, -0.020)),
                material=rubber,
                name=f"x_wiper_inner_{idx}_{jdx}",
            )
    x_carriage.visual(
        Box((0.100, 0.080, 0.038)),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=brass,
        name="x_drive_nut_block",
    )
    x_carriage.visual(
        Box((0.030, 0.060, 0.040)),
        origin=Origin(xyz=(0.158, -0.170, -0.010)),
        material=warning_red,
        name="x_stop_dog",
    )
    x_carriage.visual(
        Box((0.210, 0.225, 0.062)),
        origin=Origin(xyz=(0.0, 0.055, 0.048)),
        material=cast_iron,
        name="column_foot",
    )
    x_carriage.visual(
        Box((0.130, 0.105, 0.560)),
        origin=Origin(xyz=(0.0, 0.055, 0.350)),
        material=cast_iron,
        name="column_body",
    )
    x_carriage.visual(
        Box((0.160, 0.130, 0.036)),
        origin=Origin(xyz=(0.0, 0.055, 0.642)),
        material=cast_iron,
        name="column_cap",
    )

    # Column gusset plates, real triangular webs instead of decorative blocks.
    for idx, y in enumerate((0.010, 0.100)):
        x_carriage.visual(
            gusset_right_mesh,
            origin=Origin(xyz=(0.055, y, 0.006)),
            material=cast_iron,
            name=f"right_gusset_{idx}",
        )
        x_carriage.visual(
            gusset_left_mesh,
            origin=Origin(xyz=(-0.055, y, 0.006)),
            material=cast_iron,
            name=f"left_gusset_{idx}",
        )

    # Separate Z guide hardware fixed to the X carriage / column.
    x_carriage.visual(
        Box((0.028, 0.014, 0.505)),
        origin=Origin(xyz=(-0.043, 0.000, 0.360)),
        material=black_oxide,
        name="z_rail_backer_0",
    )
    x_carriage.visual(
        Box((0.016, 0.010, 0.490)),
        origin=Origin(xyz=(-0.043, -0.009, 0.360)),
        material=way_steel,
        name="z_linear_rail_0",
    )
    x_carriage.visual(
        Box((0.028, 0.014, 0.505)),
        origin=Origin(xyz=(0.043, 0.000, 0.360)),
        material=black_oxide,
        name="z_rail_backer_1",
    )
    x_carriage.visual(
        Box((0.016, 0.010, 0.490)),
        origin=Origin(xyz=(0.043, -0.009, 0.360)),
        material=way_steel,
        name="z_linear_rail_1",
    )
    for idx, z in enumerate((0.110, 0.610)):
        x_carriage.visual(
            Box((0.150, 0.045, 0.036)),
            origin=Origin(xyz=(0.0, 0.000, z)),
            material=black_oxide,
            name=f"z_end_bracket_{idx}",
        )
        x_carriage.visual(
            Cylinder(radius=0.018, length=0.018),
            origin=Origin(xyz=(0.0, -0.018, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=way_steel,
            name=f"z_bearing_boss_{idx}",
        )
    x_carriage.visual(
        Cylinder(radius=0.006, length=0.520),
        origin=Origin(xyz=(0.0, -0.016, 0.360)),
        material=way_steel,
        name="z_leadscrew",
    )
    x_carriage.visual(
        Box((0.012, 0.008, 0.430)),
        origin=Origin(xyz=(-0.071, 0.0065, 0.360)),
        material=way_steel,
        name="z_scale_strip",
    )
    for idx, z in enumerate((0.180, 0.230, 0.280, 0.330, 0.380, 0.430, 0.480, 0.530)):
        x_carriage.visual(
            Box((0.018 if idx % 2 == 0 else 0.012, 0.006, 0.003)),
            origin=Origin(xyz=(-0.080, 0.000, z)),
            material=black_oxide,
            name=f"z_scale_tick_{idx}",
        )
    for idx, (x, y) in enumerate(
        [(-0.075, -0.050), (0.075, -0.050), (-0.075, 0.150), (0.075, 0.150)]
    ):
        _add_bolt_on_top(x_carriage, name=f"column_foot_bolt_{idx}", x=x, y=y, z=0.081, material=way_steel)

    z_carriage = model.part("z_carriage")
    z_carriage.visual(
        Box((0.190, 0.024, 0.190)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=machined_aluminum,
        name="z_carriage_plate",
    )
    z_carriage.visual(
        Box((0.145, 0.018, 0.115)),
        origin=Origin(xyz=(0.0, -0.020, 0.0)),
        material=way_steel,
        name="tool_face_plate",
    )
    for idx, x in enumerate((-0.043, 0.043)):
        for jdx, z in enumerate((-0.055, 0.055)):
            z_carriage.visual(
                Box((0.048, 0.024, 0.052)),
                origin=Origin(xyz=(x, 0.0235, z)),
                material=black_oxide,
                name=f"z_bearing_shoe_{idx}_{jdx}",
            )
            z_carriage.visual(
                Box((0.054, 0.017, 0.010)),
                origin=Origin(xyz=(x, 0.012, z + 0.033)),
                material=rubber,
                name=f"z_wiper_top_{idx}_{jdx}",
            )
            z_carriage.visual(
                Box((0.054, 0.017, 0.010)),
                origin=Origin(xyz=(x, 0.012, z - 0.033)),
                material=rubber,
                name=f"z_wiper_bottom_{idx}_{jdx}",
            )
    z_carriage.visual(
        Box((0.065, 0.020, 0.052)),
        origin=Origin(xyz=(0.0, 0.009, 0.0)),
        material=brass,
        name="z_drive_nut_block",
    )
    z_carriage.visual(
        Box((0.030, 0.045, 0.028)),
        origin=Origin(xyz=(0.095, 0.000, 0.088)),
        material=warning_red,
        name="z_upper_stop_dog",
    )
    z_carriage.visual(
        Box((0.030, 0.045, 0.028)),
        origin=Origin(xyz=(0.095, 0.000, -0.088)),
        material=warning_red,
        name="z_lower_stop_dog",
    )
    bolt_index = 0
    for x in (-0.050, 0.050):
        for z in (-0.035, 0.035):
            _add_bolt_on_front(
                z_carriage,
                name=f"tool_bolt_{bolt_index}",
                x=x,
                y=-0.0315,
                z=z,
                material=black_oxide,
            )
            bolt_index += 1

    x_cover = model.part("x_cover")
    x_cover.visual(
        Box((0.620, 0.016, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=cover_blue,
        name="cover_panel",
    )
    x_cover.visual(
        Box((0.620, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, 0.003, 0.026)),
        material=black_oxide,
        name="upper_return_lip",
    )
    x_cover.visual(
        Box((0.620, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, 0.003, -0.026)),
        material=black_oxide,
        name="lower_return_lip",
    )
    bolt_index = 0
    for x in (-0.270, -0.090, 0.090, 0.270):
        for z in (-0.020, 0.020):
            _add_bolt_on_front(
                x_cover,
                name=f"cover_screw_{bolt_index}",
                x=x,
                y=-0.0105,
                z=z,
                material=way_steel,
            )
            bolt_index += 1

    z_cover = model.part("z_cover")
    z_cover.visual(
        Box((0.012, 0.060, 0.370)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=cover_blue,
        name="cover_plate",
    )
    z_cover.visual(
        Box((0.006, 0.060, 0.370)),
        origin=Origin(xyz=(0.004, 0.0, 0.0)),
        material=black_oxide,
        name="folded_edge",
    )
    for idx, z in enumerate((-0.145, -0.050, 0.050, 0.145)):
        for jdx, y in enumerate((-0.020, 0.020)):
            _add_bolt_on_side(
                z_cover,
                name=f"z_cover_screw_{idx}_{jdx}",
                x=0.0085,
                y=y,
                z=z,
                material=way_steel,
            )

    model.articulation(
        "x_slide",
        ArticulationType.PRISMATIC,
        parent=base_frame,
        child=x_carriage,
        origin=Origin(xyz=(-0.140, 0.0, 0.166)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=450.0, velocity=0.35, lower=0.0, upper=0.300),
    )
    model.articulation(
        "z_lift",
        ArticulationType.PRISMATIC,
        parent=x_carriage,
        child=z_carriage,
        origin=Origin(xyz=(0.0, -0.0495, 0.250)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=320.0, velocity=0.22, lower=0.0, upper=0.250),
    )
    model.articulation(
        "x_cover_mount",
        ArticulationType.FIXED,
        parent=base_frame,
        child=x_cover,
        origin=Origin(xyz=(0.0, -0.228, 0.080)),
    )
    model.articulation(
        "z_cover_mount",
        ArticulationType.FIXED,
        parent=x_carriage,
        child=z_cover,
        origin=Origin(xyz=(0.071, 0.055, 0.360)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    x_carriage = object_model.get_part("x_carriage")
    z_carriage = object_model.get_part("z_carriage")
    base_frame = object_model.get_part("base_frame")
    x_cover = object_model.get_part("x_cover")
    z_cover = object_model.get_part("z_cover")
    x_slide = object_model.get_articulation("x_slide")
    z_lift = object_model.get_articulation("z_lift")

    x_rest = ctx.part_world_position(x_carriage)
    z_rest = ctx.part_world_position(z_carriage)
    with ctx.pose({x_slide: 0.300, z_lift: 0.250}):
        x_extended = ctx.part_world_position(x_carriage)
        z_extended = ctx.part_world_position(z_carriage)

    ctx.check(
        "x stage translates along X",
        x_rest is not None and x_extended is not None and x_extended[0] > x_rest[0] + 0.290,
        details=f"rest={x_rest}, extended={x_extended}",
    )
    ctx.check(
        "z lift translates upward",
        z_rest is not None and z_extended is not None and z_extended[2] > z_rest[2] + 0.240,
        details=f"rest={z_rest}, extended={z_extended}",
    )
    ctx.expect_overlap(
        x_carriage,
        base_frame,
        axes="x",
        elem_a="x_carriage_plate",
        elem_b="x_round_way_0",
        min_overlap=0.25,
        name="x carriage remains over the X ways",
    )
    with ctx.pose({x_slide: 0.300}):
        ctx.expect_overlap(
            x_carriage,
            base_frame,
            axes="x",
            elem_a="x_carriage_plate",
            elem_b="x_round_way_1",
            min_overlap=0.25,
            name="x carriage stays retained at full travel",
        )
    ctx.expect_overlap(
        z_carriage,
        x_carriage,
        axes="z",
        elem_a="z_carriage_plate",
        elem_b="z_linear_rail_0",
        min_overlap=0.16,
        name="z carriage overlaps vertical guide span",
    )
    with ctx.pose({z_lift: 0.250}):
        ctx.expect_overlap(
            z_carriage,
            x_carriage,
            axes="z",
            elem_a="z_carriage_plate",
            elem_b="z_linear_rail_1",
            min_overlap=0.16,
            name="z lift remains on guide rails at top travel",
        )
    ctx.expect_gap(
        base_frame,
        x_cover,
        axis="y",
        positive_elem="front_rail",
        negative_elem="cover_panel",
        max_gap=0.002,
        max_penetration=0.0,
        name="removable X cover seats on front rail",
    )
    ctx.expect_gap(
        z_cover,
        x_carriage,
        axis="x",
        positive_elem="cover_plate",
        negative_elem="column_body",
        max_gap=0.002,
        max_penetration=0.00001,
        name="removable Z cover seats on column side",
    )

    return ctx.report()


object_model = build_object_model()
