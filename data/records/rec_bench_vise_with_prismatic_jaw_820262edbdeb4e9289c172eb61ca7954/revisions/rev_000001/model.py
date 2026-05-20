from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _make_dovetail_rail_mesh(name: str, *, side_sign: float):
    x0 = -0.070
    x1 = 0.148
    z_base = 0.042
    z_top = 0.052
    lower_outer = side_sign * 0.062
    lower_inner = side_sign * 0.044
    upper_inner = side_sign * 0.050
    upper_outer = side_sign * 0.056
    profile = [
        (lower_outer, z_base),
        (lower_inner, z_base),
        (upper_inner, z_top),
        (upper_outer, z_top),
    ]
    if side_sign < 0.0:
        profile = list(reversed(profile))

    rail = MeshGeometry()
    start = [rail.add_vertex(x0, y, z) for y, z in profile]
    end = [rail.add_vertex(x1, y, z) for y, z in profile]

    _add_quad(rail, start[0], start[1], start[2], start[3])
    _add_quad(rail, end[3], end[2], end[1], end[0])

    for index in range(4):
        next_index = (index + 1) % 4
        _add_quad(
            rail,
            start[index],
            end[index],
            end[next_index],
            start[next_index],
        )
    return mesh_from_geometry(rail, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compound_angle_vise")

    cast_iron = model.material("cast_iron", rgba=(0.23, 0.28, 0.33, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.73, 0.75, 0.78, 1.0))
    jaw_steel = model.material("jaw_steel", rgba=(0.60, 0.63, 0.67, 1.0))
    oxide_black = model.material("oxide_black", rgba=(0.12, 0.12, 0.13, 1.0))

    left_rail_mesh = _make_dovetail_rail_mesh("vise_left_dovetail", side_sign=1.0)
    right_rail_mesh = _make_dovetail_rail_mesh("vise_right_dovetail", side_sign=-1.0)

    base_plate = model.part("base_plate")
    base_plate.visual(
        Box((0.320, 0.180, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=cast_iron,
        name="plate",
    )
    base_plate.visual(
        Cylinder(radius=0.078, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=machined_steel,
        name="swivel_pad",
    )
    base_plate.inertial = Inertial.from_geometry(
        Box((0.320, 0.180, 0.024)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
    )

    swivel_table = model.part("swivel_table")
    swivel_table.visual(
        Cylinder(radius=0.076, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=cast_iron,
        name="turntable",
    )
    swivel_table.visual(
        Box((0.220, 0.160, 0.016)),
        origin=Origin(xyz=(0.020, 0.0, 0.022)),
        material=cast_iron,
        name="saddle",
    )
    swivel_table.visual(
        Box((0.052, 0.014, 0.058)),
        origin=Origin(xyz=(-0.010, -0.063, 0.051)),
        material=cast_iron,
        name="left_cheek",
    )
    swivel_table.visual(
        Box((0.052, 0.014, 0.058)),
        origin=Origin(xyz=(-0.010, 0.063, 0.051)),
        material=cast_iron,
        name="right_cheek",
    )
    swivel_table.inertial = Inertial.from_geometry(
        Box((0.220, 0.160, 0.080)),
        mass=4.2,
        origin=Origin(xyz=(0.010, 0.0, 0.040)),
    )

    tilt_stage = model.part("tilt_stage")
    tilt_stage.visual(
        Cylinder(radius=0.018, length=0.112),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machined_steel,
        name="trunnion",
    )
    tilt_stage.visual(
        Box((0.072, 0.100, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=cast_iron,
        name="pivot_block",
    )
    tilt_stage.visual(
        Box((0.260, 0.120, 0.012)),
        origin=Origin(xyz=(0.045, 0.0, 0.036)),
        material=cast_iron,
        name="deck",
    )
    tilt_stage.visual(
        Box((0.230, 0.084, 0.010)),
        origin=Origin(xyz=(0.045, 0.0, 0.047)),
        material=machined_steel,
        name="guide_land",
    )
    tilt_stage.visual(left_rail_mesh, material=machined_steel, name="left_dovetail")
    tilt_stage.visual(right_rail_mesh, material=machined_steel, name="right_dovetail")
    tilt_stage.visual(
        Box((0.016, 0.018, 0.032)),
        origin=Origin(xyz=(0.150, -0.031, 0.058)),
        material=cast_iron,
        name="left_screw_pillar",
    )
    tilt_stage.visual(
        Box((0.016, 0.018, 0.032)),
        origin=Origin(xyz=(0.150, 0.031, 0.058)),
        material=cast_iron,
        name="right_screw_pillar",
    )
    tilt_stage.visual(
        Box((0.020, 0.082, 0.010)),
        origin=Origin(xyz=(0.150, 0.0, 0.079)),
        material=cast_iron,
        name="screw_bridge",
    )
    tilt_stage.inertial = Inertial.from_geometry(
        Box((0.270, 0.130, 0.090)),
        mass=5.8,
        origin=Origin(xyz=(0.045, 0.0, 0.045)),
    )

    fixed_jaw = model.part("fixed_jaw")
    fixed_jaw.visual(
        Box((0.050, 0.084, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=machined_steel,
        name="jaw_base",
    )
    fixed_jaw.visual(
        Box((0.036, 0.132, 0.048)),
        origin=Origin(xyz=(0.002, 0.0, 0.038)),
        material=cast_iron,
        name="jaw_block",
    )
    fixed_jaw.visual(
        Box((0.004, 0.124, 0.040)),
        origin=Origin(xyz=(0.022, 0.0, 0.038)),
        material=jaw_steel,
        name="clamp_face",
    )
    fixed_jaw.visual(
        Box((0.024, 0.094, 0.026)),
        origin=Origin(xyz=(-0.018, 0.0, 0.021)),
        material=cast_iron,
        name="rear_rib",
    )
    fixed_jaw.visual(
        Box((0.028, 0.100, 0.010)),
        origin=Origin(xyz=(0.000, 0.0, 0.062)),
        material=machined_steel,
        name="top_cap",
    )
    fixed_jaw.inertial = Inertial.from_geometry(
        Box((0.056, 0.132, 0.072)),
        mass=2.7,
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
    )

    moving_jaw = model.part("moving_jaw")
    moving_jaw.visual(
        Box((0.044, 0.078, 0.016)),
        origin=Origin(xyz=(0.024, 0.0, 0.008)),
        material=machined_steel,
        name="carriage",
    )
    moving_jaw.visual(
        Box((0.034, 0.132, 0.050)),
        origin=Origin(xyz=(0.019, 0.0, 0.041)),
        material=cast_iron,
        name="jaw_block",
    )
    moving_jaw.visual(
        Box((0.004, 0.124, 0.040)),
        origin=Origin(xyz=(0.002, 0.0, 0.038)),
        material=jaw_steel,
        name="clamp_face",
    )
    moving_jaw.visual(
        Box((0.026, 0.098, 0.010)),
        origin=Origin(xyz=(0.019, 0.0, 0.066)),
        material=machined_steel,
        name="top_cap",
    )
    moving_jaw.visual(
        Cylinder(radius=0.0075, length=0.122),
        origin=Origin(xyz=(0.107, 0.0, 0.012), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=jaw_steel,
        name="leadscrew",
    )
    moving_jaw.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(xyz=(0.175, 0.0, 0.012), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=jaw_steel,
        name="handle_hub",
    )
    moving_jaw.visual(
        Cylinder(radius=0.004, length=0.094),
        origin=Origin(xyz=(0.175, 0.0, 0.012), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=oxide_black,
        name="tommy_bar",
    )
    moving_jaw.inertial = Inertial.from_geometry(
        Box((0.190, 0.132, 0.078)),
        mass=2.4,
        origin=Origin(xyz=(0.090, 0.0, 0.033)),
    )

    model.articulation(
        "swivel_rotation",
        ArticulationType.CONTINUOUS,
        parent=base_plate,
        child=swivel_table,
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=50.0, velocity=1.5),
    )
    model.articulation(
        "tilt_adjustment",
        ArticulationType.REVOLUTE,
        parent=swivel_table,
        child=tilt_stage,
        origin=Origin(xyz=(-0.010, 0.0, 0.052)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.8,
            lower=0.0,
            upper=math.radians(45.0),
        ),
    )
    model.articulation(
        "rear_jaw_mount",
        ArticulationType.FIXED,
        parent=tilt_stage,
        child=fixed_jaw,
        origin=Origin(xyz=(-0.052, 0.0, 0.052)),
    )
    model.articulation(
        "front_jaw_slide",
        ArticulationType.PRISMATIC,
        parent=tilt_stage,
        child=moving_jaw,
        origin=Origin(xyz=(0.008, 0.0, 0.052)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.10,
            lower=0.0,
            upper=0.080,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_plate = object_model.get_part("base_plate")
    swivel_table = object_model.get_part("swivel_table")
    tilt_stage = object_model.get_part("tilt_stage")
    fixed_jaw = object_model.get_part("fixed_jaw")
    moving_jaw = object_model.get_part("moving_jaw")

    swivel_rotation = object_model.get_articulation("swivel_rotation")
    tilt_adjustment = object_model.get_articulation("tilt_adjustment")
    front_jaw_slide = object_model.get_articulation("front_jaw_slide")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "swivel_axis_is_vertical",
        tuple(float(value) for value in swivel_rotation.axis) == (0.0, 0.0, 1.0),
        f"Unexpected swivel axis: {swivel_rotation.axis}",
    )
    ctx.check(
        "tilt_axis_is_crosswise",
        tuple(float(value) for value in tilt_adjustment.axis)
        in ((0.0, 1.0, 0.0), (0.0, -1.0, 0.0)),
        f"Unexpected tilt axis: {tilt_adjustment.axis}",
    )
    ctx.check(
        "jaw_slide_axis_is_longitudinal",
        tuple(float(value) for value in front_jaw_slide.axis) == (1.0, 0.0, 0.0),
        f"Unexpected slide axis: {front_jaw_slide.axis}",
    )

    ctx.expect_contact(
        swivel_table,
        base_plate,
        elem_a="turntable",
        elem_b="swivel_pad",
        name="swivel_table_seats_on_base",
    )
    ctx.expect_contact(tilt_stage, swivel_table, name="tilt_stage_supported_in_cheeks")
    ctx.expect_contact(fixed_jaw, tilt_stage, name="fixed_jaw_mounted_to_stage")
    ctx.expect_contact(moving_jaw, tilt_stage, name="moving_jaw_supported_on_guides")
    ctx.expect_gap(
        moving_jaw,
        fixed_jaw,
        axis="x",
        positive_elem="clamp_face",
        negative_elem="clamp_face",
        min_gap=0.035,
        max_gap=0.037,
        name="rest_jaw_gap",
    )
    ctx.expect_overlap(
        moving_jaw,
        fixed_jaw,
        axes="yz",
        elem_a="clamp_face",
        elem_b="clamp_face",
        min_overlap=0.030,
        name="jaw_faces_stay_aligned",
    )

    moving_rest = ctx.part_world_position(moving_jaw)
    fixed_rest = ctx.part_world_position(fixed_jaw)
    assert moving_rest is not None
    assert fixed_rest is not None
    jaw_origin_spacing = moving_rest[0] - fixed_rest[0]

    with ctx.pose({front_jaw_slide: 0.080}):
        ctx.expect_contact(moving_jaw, tilt_stage, name="moving_jaw_supported_when_open")
        ctx.expect_gap(
            moving_jaw,
            fixed_jaw,
            axis="x",
            positive_elem="clamp_face",
            negative_elem="clamp_face",
            min_gap=0.115,
            max_gap=0.117,
            name="open_jaw_gap",
        )

    with ctx.pose({tilt_adjustment: math.radians(45.0)}):
        fixed_tilted = ctx.part_world_position(fixed_jaw)
        moving_tilted = ctx.part_world_position(moving_jaw)
        assert fixed_tilted is not None
        assert moving_tilted is not None
        dx = moving_tilted[0] - fixed_tilted[0]
        dz = moving_tilted[2] - fixed_tilted[2]
        expected_component = jaw_origin_spacing / math.sqrt(2.0)
        ctx.check(
            "tilt_stage_angles_jaw_pair",
            abs(dx - expected_component) < 0.008 and abs(dz - expected_component) < 0.008,
            (
                "Jaw-origin separation should rotate into equal x/z components at 45 deg tilt: "
                f"rest_spacing={jaw_origin_spacing}, dx={dx}, dz={dz}, "
                f"expected_component={expected_component}"
            ),
        )
        ctx.check(
            "tilt_preserves_jaw_lateral_alignment",
            abs(moving_tilted[1] - fixed_tilted[1]) < 1e-6,
            f"Jaw pair should remain laterally aligned: fixed={fixed_tilted}, moving={moving_tilted}",
        )
        ctx.expect_contact(tilt_stage, swivel_table, name="tilt_stage_remains_supported_when_tilted")

    with ctx.pose({swivel_rotation: math.pi / 2.0}):
        fixed_swiveled = ctx.part_world_position(fixed_jaw)
        assert fixed_swiveled is not None
        ctx.check(
            "swivel_changes_plan_view_orientation",
            abs(fixed_swiveled[0] - fixed_rest[0]) > 0.020
            and abs(fixed_swiveled[1] - fixed_rest[1]) > 0.020,
            f"Fixed jaw did not move around the swivel axis: rest={fixed_rest}, swivel={fixed_swiveled}",
        )
        ctx.check(
            "swivel_preserves_height",
            abs(fixed_swiveled[2] - fixed_rest[2]) < 1e-6,
            f"Swivel should not change height: rest={fixed_rest}, swivel={fixed_swiveled}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
