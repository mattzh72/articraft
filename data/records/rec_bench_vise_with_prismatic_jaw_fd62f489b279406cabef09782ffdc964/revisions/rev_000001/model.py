from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    section_loft,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _loop_at_x(x_pos: float, yz_points: list[tuple[float, float]]) -> list[tuple[float, float, float]]:
    return [(x_pos, y_pos, z_pos) for y_pos, z_pos in yz_points]


def _dovetail_rail_mesh(*, x_start: float, x_end: float, z_base: float) -> object:
    cross_section = [
        (-0.025, z_base),
        (-0.013, z_base + 0.010),
        (0.013, z_base + 0.010),
        (0.025, z_base),
    ]
    return _mesh(
        "vise_dovetail_rail",
        section_loft(
            [
                _loop_at_x(x_start, cross_section),
                _loop_at_x(x_end, cross_section),
            ]
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="toolmakers_precision_vise")

    ground_steel = model.material("ground_steel", rgba=(0.73, 0.75, 0.78, 1.0))
    hardened_steel = model.material("hardened_steel", rgba=(0.53, 0.56, 0.60, 1.0))
    blued_steel = model.material("blued_steel", rgba=(0.20, 0.23, 0.27, 1.0))
    dark_oxide = model.material("dark_oxide", rgba=(0.11, 0.12, 0.14, 1.0))

    swivel_base = model.part("swivel_base")
    swivel_base.visual(
        Box((0.150, 0.105, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=blued_steel,
        name="base_plate",
    )
    swivel_base.visual(
        Box((0.090, 0.014, 0.020)),
        origin=Origin(xyz=(0.0, 0.034, 0.020)),
        material=blued_steel,
        name="left_socket_arm",
    )
    swivel_base.visual(
        Box((0.090, 0.014, 0.020)),
        origin=Origin(xyz=(0.0, -0.034, 0.020)),
        material=blued_steel,
        name="right_socket_arm",
    )
    swivel_base.visual(
        Cylinder(radius=0.020, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=dark_oxide,
        name="socket_floor",
    )
    swivel_base.visual(
        _mesh(
            "vise_swivel_socket_ring",
            TorusGeometry(radius=0.032, tube=0.006, radial_segments=20, tubular_segments=40).translate(
                0.0,
                0.0,
                0.030,
            ),
        ),
        material=hardened_steel,
        name="socket_ring",
    )
    swivel_base.inertial = Inertial.from_geometry(
        Box((0.150, 0.105, 0.040)),
        mass=3.6,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )

    vise_body = model.part("vise_body")
    vise_body.visual(
        Sphere(radius=0.024),
        material=ground_steel,
        name="ball_joint",
    )
    vise_body.visual(
        Box((0.230, 0.090, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=ground_steel,
        name="main_base_block",
    )
    vise_body.visual(
        _dovetail_rail_mesh(x_start=-0.058, x_end=0.084, z_base=0.019),
        material=hardened_steel,
        name="guide_rail",
    )
    vise_body.visual(
        Box((0.040, 0.090, 0.016)),
        origin=Origin(xyz=(-0.103, 0.0, 0.027)),
        material=ground_steel,
        name="fixed_jaw_buttress",
    )
    vise_body.visual(
        Box((0.034, 0.090, 0.038)),
        origin=Origin(xyz=(-0.090, 0.0, 0.038)),
        material=ground_steel,
        name="fixed_jaw_block",
    )
    vise_body.visual(
        Box((0.004, 0.072, 0.026)),
        origin=Origin(xyz=(-0.071, 0.0, 0.042)),
        material=hardened_steel,
        name="fixed_jaw_face",
    )
    vise_body.visual(
        Box((0.036, 0.062, 0.028)),
        origin=Origin(xyz=(0.095, 0.0, 0.033)),
        material=ground_steel,
        name="screw_bearing_block",
    )
    vise_body.visual(
        Cylinder(radius=0.006, length=0.176),
        origin=Origin(xyz=(0.058, 0.0, 0.046), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_oxide,
        name="leadscrew_core",
    )
    for index in range(17):
        vise_body.visual(
            Cylinder(radius=0.0068, length=0.0018),
            origin=Origin(
                xyz=(0.020 + index * 0.0052, 0.0, 0.046),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=blued_steel,
            name=f"thread_ridge_{index:02d}",
        )
    vise_body.visual(
        Cylinder(radius=0.010, length=0.024),
        origin=Origin(xyz=(0.114, 0.0, 0.046), rpy=(0.0, pi / 2.0, 0.0)),
        material=ground_steel,
        name="handle_boss",
    )
    vise_body.visual(
        Cylinder(radius=0.004, length=0.072),
        origin=Origin(xyz=(0.132, 0.0, 0.046), rpy=(pi / 2.0, 0.0, 0.0)),
        material=blued_steel,
        name="t_handle_bar",
    )
    vise_body.visual(
        Sphere(radius=0.007),
        origin=Origin(xyz=(0.132, 0.036, 0.046)),
        material=ground_steel,
        name="handle_left_knob",
    )
    vise_body.visual(
        Sphere(radius=0.007),
        origin=Origin(xyz=(0.132, -0.036, 0.046)),
        material=ground_steel,
        name="handle_right_knob",
    )
    vise_body.inertial = Inertial.from_geometry(
        Box((0.230, 0.090, 0.090)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
    )

    moving_jaw = model.part("moving_jaw")
    moving_jaw.visual(
        Box((0.046, 0.068, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=hardened_steel,
        name="carriage_pad",
    )
    moving_jaw.visual(
        Box((0.042, 0.020, 0.030)),
        origin=Origin(xyz=(0.0, 0.024, 0.015)),
        material=ground_steel,
        name="left_carriage_cheek",
    )
    moving_jaw.visual(
        Box((0.042, 0.020, 0.030)),
        origin=Origin(xyz=(0.0, -0.024, 0.015)),
        material=ground_steel,
        name="right_carriage_cheek",
    )
    moving_jaw.visual(
        Box((0.036, 0.082, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material=ground_steel,
        name="jaw_block",
    )
    moving_jaw.visual(
        Box((0.006, 0.066, 0.024)),
        origin=Origin(xyz=(-0.0185, 0.0, 0.040)),
        material=hardened_steel,
        name="moving_jaw_face",
    )
    moving_jaw.inertial = Inertial.from_geometry(
        Box((0.046, 0.082, 0.054)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
    )

    model.articulation(
        "swivel_base_joint",
        ArticulationType.REVOLUTE,
        parent=swivel_base,
        child=vise_body,
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.2,
            lower=-0.9,
            upper=0.9,
        ),
    )
    model.articulation(
        "jaw_slide",
        ArticulationType.PRISMATIC,
        parent=vise_body,
        child=moving_jaw,
        origin=Origin(xyz=(-0.028, 0.0, 0.029)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.03,
            lower=0.0,
            upper=0.070,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    swivel_base = object_model.get_part("swivel_base")
    vise_body = object_model.get_part("vise_body")
    moving_jaw = object_model.get_part("moving_jaw")
    swivel_joint = object_model.get_articulation("swivel_base_joint")
    jaw_slide = object_model.get_articulation("jaw_slide")

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

    ctx.expect_gap(
        vise_body,
        swivel_base,
        axis="z",
        positive_elem="main_base_block",
        negative_elem="socket_ring",
        max_gap=0.0012,
        max_penetration=0.0004,
        name="vise_body_seats_on_socket_ring",
    )
    ctx.expect_overlap(
        vise_body,
        swivel_base,
        axes="xy",
        elem_a="main_base_block",
        elem_b="socket_ring",
        min_overlap=0.060,
        name="socket_ring_underlaps_body",
    )
    ctx.expect_gap(
        moving_jaw,
        vise_body,
        axis="z",
        positive_elem="carriage_pad",
        negative_elem="guide_rail",
        max_gap=0.0010,
        max_penetration=0.0004,
        name="moving_jaw_runs_on_dovetail_top",
    )
    ctx.expect_overlap(
        moving_jaw,
        vise_body,
        axes="xy",
        elem_a="carriage_pad",
        elem_b="guide_rail",
        min_overlap=0.020,
        name="carriage_pad_overlaps_guide_rail",
    )
    ctx.expect_gap(
        moving_jaw,
        vise_body,
        axis="x",
        positive_elem="moving_jaw_face",
        negative_elem="fixed_jaw_face",
        min_gap=0.016,
        max_gap=0.024,
        name="jaw_opening_rest_gap",
    )

    rest_jaw_pos = ctx.part_world_position(moving_jaw)
    assert rest_jaw_pos is not None
    with ctx.pose({jaw_slide: 0.070}):
        open_jaw_pos = ctx.part_world_position(moving_jaw)
        assert open_jaw_pos is not None
        assert open_jaw_pos[0] > rest_jaw_pos[0] + 0.060
        ctx.expect_gap(
            moving_jaw,
            vise_body,
            axis="x",
            positive_elem="moving_jaw_face",
            negative_elem="fixed_jaw_face",
            min_gap=0.086,
            max_gap=0.094,
            name="jaw_opens_with_leadscrew_travel",
        )
        ctx.expect_overlap(
            moving_jaw,
            vise_body,
            axes="xy",
            elem_a="carriage_pad",
            elem_b="guide_rail",
            min_overlap=0.020,
            name="moving_jaw_stays_guided_when_open",
        )

    with ctx.pose({swivel_joint: 0.60}):
        swiveled_jaw_pos = ctx.part_world_position(moving_jaw)
        assert swiveled_jaw_pos is not None
        assert abs(swiveled_jaw_pos[1]) > 0.012
        ctx.expect_gap(
            vise_body,
            swivel_base,
            axis="z",
            positive_elem="main_base_block",
            negative_elem="socket_ring",
            max_gap=0.0012,
            max_penetration=0.0004,
            name="swivel_base_still_supports_body_while_rotated",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
