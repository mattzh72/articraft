from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_RAIL_TOP_Z = 0.119
BEAM_FRONT_CONTACT_Y = -0.072
BEAM_GUIDE_CENTER_Z = 0.60
BEAM_SLIDE_LIMIT = 0.32
CARRIAGE_SLIDE_LIMIT = 0.40


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _combine(*shapes: cq.Workplane) -> cq.Workplane:
    result = shapes[0]
    for shape in shapes[1:]:
        result = result.union(shape)
    return result


def _base_shape() -> cq.Workplane:
    side_beam_left = _box((0.10, 1.00, 0.10), (-0.52, 0.0, 0.05))
    side_beam_right = _box((0.10, 1.00, 0.10), (0.52, 0.0, 0.05))
    cross_front = _box((1.04, 0.08, 0.08), (0.0, -0.44, 0.04))
    cross_mid = _box((1.04, 0.07, 0.07), (0.0, 0.0, 0.035))
    cross_back = _box((1.04, 0.08, 0.08), (0.0, 0.44, 0.04))
    left_rail = _box((0.045, 0.92, 0.020), (-0.52, 0.0, 0.109))
    right_rail = _box((0.045, 0.92, 0.020), (0.52, 0.0, 0.109))
    return _combine(
        side_beam_left,
        side_beam_right,
        cross_front,
        cross_mid,
        cross_back,
        left_rail,
        right_rail,
    )


def _beam_shape() -> cq.Workplane:
    left_truck = _box((0.12, 0.18, 0.050), (-0.52, 0.0, 0.144))
    right_truck = _box((0.12, 0.18, 0.050), (0.52, 0.0, 0.144))
    left_cheek = _box((0.03, 0.16, 0.54), (-0.52, 0.0, 0.39))
    right_cheek = _box((0.03, 0.16, 0.54), (0.52, 0.0, 0.39))
    beam_body = _box((1.08, 0.12, 0.14), (0.0, 0.0, 0.60))
    front_mount_plate = _box((0.92, 0.024, 0.12), (0.0, -0.072, 0.60))
    upper_guide = _box((0.92, 0.012, 0.020), (0.0, -0.084, 0.64))
    lower_guide = _box((0.92, 0.012, 0.020), (0.0, -0.084, 0.56))
    return _combine(
        left_truck,
        right_truck,
        left_cheek,
        right_cheek,
        beam_body,
        front_mount_plate,
        upper_guide,
        lower_guide,
    )


def _carriage_shape() -> cq.Workplane:
    upper_slider = _box((0.13, 0.012, 0.035), (0.0, -0.024, 0.040))
    lower_slider = _box((0.13, 0.012, 0.035), (0.0, -0.024, -0.040))
    bridge_plate = _box((0.18, 0.024, 0.18), (0.0, -0.042, 0.0))
    tool_block = _box((0.10, 0.070, 0.12), (0.0, -0.089, 0.0))
    cable_block = _box((0.06, 0.030, 0.050), (0.0, -0.050, 0.105))
    tool_nose = cq.Workplane("XZ").circle(0.026).extrude(-0.080).translate((0.0, -0.105, 0.0))
    return _combine(
        upper_slider,
        lower_slider,
        bridge_plate,
        tool_block,
        cable_block,
        tool_nose,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wide_gantry_positioning_module")

    frame_gray = model.material("frame_gray", rgba=(0.22, 0.24, 0.27, 1.0))
    beam_silver = model.material("beam_silver", rgba=(0.70, 0.73, 0.76, 1.0))
    carriage_black = model.material("carriage_black", rgba=(0.16, 0.17, 0.19, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_shape(), "base_shell"),
        material=frame_gray,
        name="base_shell",
    )
    base.inertial = Inertial.from_geometry(
        Box((1.14, 1.00, 0.12)),
        mass=34.0,
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
    )

    beam = model.part("moving_beam")
    beam.visual(
        mesh_from_cadquery(_beam_shape(), "moving_beam_shell"),
        material=beam_silver,
        name="moving_beam_shell",
    )
    beam.inertial = Inertial.from_geometry(
        Box((1.08, 0.18, 0.66)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.33)),
    )

    carriage = model.part("tool_carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_shape(), "tool_carriage_shell"),
        material=carriage_black,
        name="tool_carriage_shell",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.18, 0.17, 0.23)),
        mass=4.5,
        origin=Origin(xyz=(0.0, -0.055, 0.0)),
    )

    model.articulation(
        "base_to_beam_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=450.0,
            velocity=0.60,
            lower=-BEAM_SLIDE_LIMIT,
            upper=BEAM_SLIDE_LIMIT,
        ),
    )

    model.articulation(
        "beam_to_carriage_slide",
        ArticulationType.PRISMATIC,
        parent=beam,
        child=carriage,
        origin=Origin(xyz=(0.0, BEAM_FRONT_CONTACT_Y, BEAM_GUIDE_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.80,
            lower=-CARRIAGE_SLIDE_LIMIT,
            upper=CARRIAGE_SLIDE_LIMIT,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    beam = object_model.get_part("moving_beam")
    carriage = object_model.get_part("tool_carriage")
    beam_slide = object_model.get_articulation("base_to_beam_slide")
    carriage_slide = object_model.get_articulation("beam_to_carriage_slide")

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
        "gantry_joint_types",
        beam_slide.articulation_type == ArticulationType.PRISMATIC
        and carriage_slide.articulation_type == ArticulationType.PRISMATIC,
        "Both primary mechanisms should be prismatic joints.",
    )
    ctx.check(
        "gantry_joint_axes",
        tuple(beam_slide.axis) == (0.0, 1.0, 0.0)
        and tuple(carriage_slide.axis) == (1.0, 0.0, 0.0),
        "The beam should slide along world Y and the carriage should slide along beam X.",
    )

    ctx.expect_contact(base, beam, name="beam_trucks_contact_base_rails")
    ctx.expect_contact(beam, carriage, name="carriage_contacts_beam_guides")

    beam_rest = ctx.part_world_position(beam)
    with ctx.pose({beam_slide: 0.20}):
        beam_shifted = ctx.part_world_position(beam)
    beam_moves_correctly = (
        beam_rest is not None
        and beam_shifted is not None
        and abs((beam_shifted[1] - beam_rest[1]) - 0.20) < 1e-4
        and abs(beam_shifted[0] - beam_rest[0]) < 1e-6
        and abs(beam_shifted[2] - beam_rest[2]) < 1e-6
    )
    ctx.check(
        "beam_translation_follows_long_rails",
        beam_moves_correctly,
        "The moving beam should translate only along the long base rails.",
    )

    with ctx.pose({beam_slide: 0.10, carriage_slide: 0.0}):
        carriage_centered = ctx.part_world_position(carriage)
    with ctx.pose({beam_slide: 0.10, carriage_slide: 0.25}):
        carriage_shifted = ctx.part_world_position(carriage)
    carriage_moves_correctly = (
        carriage_centered is not None
        and carriage_shifted is not None
        and abs((carriage_shifted[0] - carriage_centered[0]) - 0.25) < 1e-4
        and abs(carriage_shifted[1] - carriage_centered[1]) < 1e-6
        and abs(carriage_shifted[2] - carriage_centered[2]) < 1e-6
    )
    ctx.check(
        "carriage_translation_follows_cross_beam",
        carriage_moves_correctly,
        "The tool carriage should translate only across the moving beam.",
    )

    with ctx.pose({beam_slide: BEAM_SLIDE_LIMIT, carriage_slide: CARRIAGE_SLIDE_LIMIT}):
        ctx.expect_contact(base, beam, name="beam_supported_at_forward_limit")
        ctx.expect_contact(beam, carriage, name="carriage_supported_at_right_limit")
        ctx.expect_within(carriage, beam, axes="x", margin=0.0, name="carriage_stays_between_beam_cheeks")

    with ctx.pose({beam_slide: -BEAM_SLIDE_LIMIT, carriage_slide: -CARRIAGE_SLIDE_LIMIT}):
        ctx.expect_contact(base, beam, name="beam_supported_at_rear_limit")
        ctx.expect_contact(beam, carriage, name="carriage_supported_at_left_limit")
        ctx.expect_within(carriage, beam, axes="x", margin=0.0, name="carriage_stays_between_beam_cheeks_left")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
