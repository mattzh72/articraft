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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _panel_mesh(name: str, *, width: float, length: float, thickness: float, radius: float):
    return mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(width, length, radius), thickness),
        name,
    )


def _add_caster_assembly(
    model: ArticulatedObject,
    chassis,
    *,
    prefix: str,
    x: float,
    y: float,
    z: float,
    steel,
    zinc,
    rubber,
) -> None:
    swivel = model.part(f"{prefix}_caster_swivel")
    swivel.visual(
        Box((0.090, 0.060, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=steel,
        name="top_plate",
    )
    swivel.visual(
        Cylinder(radius=0.012, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, -0.013)),
        material=zinc,
        name="swivel_stem",
    )
    swivel.visual(
        Box((0.050, 0.020, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=zinc,
        name="fork_crown",
    )
    swivel.visual(
        Box((0.006, 0.020, 0.064)),
        origin=Origin(xyz=(-0.020, 0.0, -0.057)),
        material=zinc,
        name="left_fork_leg",
    )
    swivel.visual(
        Box((0.006, 0.020, 0.064)),
        origin=Origin(xyz=(0.020, 0.0, -0.057)),
        material=zinc,
        name="right_fork_leg",
    )
    swivel.visual(
        Cylinder(radius=0.004, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, -0.070), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="axle",
    )
    swivel.inertial = Inertial.from_geometry(
        Box((0.090, 0.060, 0.078)),
        mass=0.75,
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
    )

    wheel = model.part(f"{prefix}_caster_wheel")
    wheel.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.028, tube=0.009, radial_segments=16, tubular_segments=28).rotate_y(
                math.pi / 2.0
            ),
            f"{prefix}_caster_tire",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=rubber,
        name="tire",
    )
    wheel.visual(
        Cylinder(radius=0.020, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hub",
    )
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.037, length=0.033),
        mass=0.45,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        f"{prefix}_caster_swivel_joint",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=swivel,
        origin=Origin(xyz=(x, y, z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=6.0),
    )
    model.articulation(
        f"{prefix}_caster_spin_joint",
        ArticulationType.CONTINUOUS,
        parent=swivel,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, -0.070)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=18.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="platform_cart")

    deck_blue = model.material("deck_blue", rgba=(0.16, 0.38, 0.63, 1.0))
    steel = model.material("steel", rgba=(0.69, 0.72, 0.75, 1.0))
    zinc = model.material("zinc", rgba=(0.53, 0.56, 0.60, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))
    grip_black = model.material("grip_black", rgba=(0.14, 0.14, 0.15, 1.0))

    chassis = model.part("chassis")
    chassis.visual(
        _panel_mesh(
            "platform_cart_main_deck",
            width=0.720,
            length=0.440,
            thickness=0.026,
            radius=0.038,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.113)),
        material=deck_blue,
        name="main_deck",
    )
    chassis.visual(
        Box((0.030, 0.500, 0.012)),
        origin=Origin(xyz=(-0.245, -0.020, 0.094)),
        material=zinc,
        name="left_frame_rail",
    )
    chassis.visual(
        Box((0.030, 0.500, 0.012)),
        origin=Origin(xyz=(0.245, -0.020, 0.094)),
        material=zinc,
        name="right_frame_rail",
    )
    chassis.visual(
        Box((0.360, 0.050, 0.012)),
        origin=Origin(xyz=(0.0, -0.255, 0.094)),
        material=zinc,
        name="rear_frame_crossmember",
    )
    chassis.visual(
        Box((0.028, 0.320, 0.012)),
        origin=Origin(xyz=(-0.140, 0.140, 0.094)),
        material=zinc,
        name="left_guide_rail",
    )
    chassis.visual(
        Box((0.028, 0.320, 0.012)),
        origin=Origin(xyz=(0.140, 0.140, 0.094)),
        material=zinc,
        name="right_guide_rail",
    )
    for prefix, px, py in (
        ("rear_left", -0.290, -0.240),
        ("rear_right", 0.290, -0.240),
        ("front_left", -0.290, 0.240),
        ("front_right", 0.290, 0.240),
    ):
        chassis.visual(
            Box((0.090, 0.060, 0.008)),
            origin=Origin(xyz=(px, py, 0.096)),
            material=steel,
            name=f"{prefix}_pad",
        )
    chassis.visual(
        Box((0.050, 0.110, 0.036)),
        origin=Origin(xyz=(-0.155, -0.280, 0.112)),
        material=zinc,
        name="left_hinge_arm",
    )
    chassis.visual(
        Box((0.050, 0.110, 0.036)),
        origin=Origin(xyz=(0.155, -0.280, 0.112)),
        material=zinc,
        name="right_hinge_arm",
    )
    chassis.visual(
        Box((0.520, 0.050, 0.012)),
        origin=Origin(xyz=(0.0, -0.245, 0.094)),
        material=zinc,
        name="rear_frame_crossmember",
    )
    chassis.inertial = Inertial.from_geometry(
        Box((0.720, 0.440, 0.128)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.064)),
    )

    front_extension = model.part("front_extension")
    front_extension.visual(
        Box((0.420, 0.280, 0.014)),
        origin=Origin(xyz=(0.0, 0.100, -0.024)),
        material=deck_blue,
        name="extension_panel",
    )
    front_extension.visual(
        Box((0.340, 0.024, 0.022)),
        origin=Origin(xyz=(0.0, 0.232, -0.017)),
        material=steel,
        name="pull_lip",
    )
    for side_name, sign in (("left", -1.0), ("right", 1.0)):
        rail_x = 0.140 * sign
        front_extension.visual(
            Box((0.040, 0.300, 0.008)),
            origin=Origin(xyz=(rail_x, 0.020, -0.015)),
            material=zinc,
            name=f"{side_name}_runner_lower",
        )
        front_extension.visual(
            Box((0.040, 0.300, 0.008)),
            origin=Origin(xyz=(rail_x, 0.020, -0.007)),
            material=zinc,
            name=f"{side_name}_runner_keeper",
        )
        front_extension.visual(
            Box((0.006, 0.300, 0.026)),
            origin=Origin(xyz=(rail_x + sign * 0.017, 0.020, -0.011)),
            material=zinc,
            name=f"{side_name}_runner_outer",
        )
        front_extension.visual(
            Box((0.006, 0.300, 0.026)),
            origin=Origin(xyz=(rail_x - sign * 0.017, 0.020, -0.011)),
            material=zinc,
            name=f"{side_name}_runner_inner",
        )
    front_extension.inertial = Inertial.from_geometry(
        Box((0.420, 0.300, 0.032)),
        mass=4.5,
        origin=Origin(xyz=(0.0, 0.100, -0.004)),
    )

    rear_handle = model.part("rear_handle")
    rear_handle.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (-0.190, 0.0, 0.000),
                    (-0.190, 0.0, 0.760),
                    (0.000, 0.0, 0.810),
                    (0.190, 0.0, 0.760),
                    (0.190, 0.0, 0.000),
                ],
                radius=0.012,
                samples_per_segment=18,
                radial_segments=18,
            ),
            "platform_cart_handle_tube",
        ),
        material=steel,
        name="handle_tube",
    )
    rear_handle.visual(
        Cylinder(radius=0.012, length=0.400),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=zinc,
        name="pivot_tube",
    )
    rear_handle.visual(
        Cylinder(radius=0.016, length=0.120),
        origin=Origin(xyz=(-0.105, 0.0, 0.805), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grip_black,
        name="left_grip",
    )
    rear_handle.visual(
        Cylinder(radius=0.016, length=0.120),
        origin=Origin(xyz=(0.105, 0.0, 0.805), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grip_black,
        name="right_grip",
    )
    rear_handle.inertial = Inertial.from_geometry(
        Box((0.440, 0.040, 0.840)),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.0, 0.420)),
    )

    model.articulation(
        "rear_handle_fold",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=rear_handle,
        origin=Origin(xyz=(0.0, -0.340, 0.130)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=24.0,
            velocity=1.8,
            lower=0.0,
            upper=1.55,
        ),
    )
    model.articulation(
        "front_extension_slide",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=front_extension,
        origin=Origin(xyz=(0.0, 0.080, 0.094)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.25,
            lower=0.0,
            upper=0.220,
        ),
    )

    for prefix, px, py in (
        ("rear_left", -0.290, -0.240),
        ("rear_right", 0.290, -0.240),
        ("front_left", -0.290, 0.240),
        ("front_right", 0.290, 0.240),
    ):
        _add_caster_assembly(
            model,
            chassis,
            prefix=prefix,
            x=px,
            y=py,
            z=0.092,
            steel=steel,
            zinc=zinc,
            rubber=rubber,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    chassis = object_model.get_part("chassis")
    front_extension = object_model.get_part("front_extension")
    rear_handle = object_model.get_part("rear_handle")
    front_extension_slide = object_model.get_articulation("front_extension_slide")
    rear_handle_fold = object_model.get_articulation("rear_handle_fold")

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
    ctx.allow_overlap(
        chassis,
        rear_handle,
        elem_a="left_hinge_arm",
        elem_b="pivot_tube",
        reason="Left hinge bracket wraps the rear handle pivot tube.",
    )
    ctx.allow_overlap(
        chassis,
        rear_handle,
        elem_a="right_hinge_arm",
        elem_b="pivot_tube",
        reason="Right hinge bracket wraps the rear handle pivot tube.",
    )
    for prefix in ("rear_left", "rear_right", "front_left", "front_right"):
        ctx.allow_overlap(
            object_model.get_part(f"{prefix}_caster_swivel"),
            object_model.get_part(f"{prefix}_caster_wheel"),
            elem_a="axle",
            elem_b="hub",
            reason="Caster axle passes through the wheel hub by design.",
        )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "rear_handle_fold_axis",
        rear_handle_fold.axis == (-1.0, 0.0, 0.0),
        f"Expected rear handle hinge axis (-1, 0, 0), got {rear_handle_fold.axis}",
    )
    ctx.check(
        "front_extension_slide_axis",
        front_extension_slide.axis == (0.0, 1.0, 0.0),
        f"Expected extension slide axis (0, 1, 0), got {front_extension_slide.axis}",
    )

    ctx.expect_contact(rear_handle, chassis, elem_a="pivot_tube", elem_b="left_hinge_arm")
    ctx.expect_gap(
        chassis,
        front_extension,
        axis="z",
        positive_elem="left_guide_rail",
        negative_elem="left_runner_lower",
        min_gap=0.004,
        max_gap=0.006,
    )
    ctx.expect_gap(
        chassis,
        front_extension,
        axis="z",
        positive_elem="right_guide_rail",
        negative_elem="right_runner_lower",
        min_gap=0.004,
        max_gap=0.006,
    )
    ctx.expect_gap(
        chassis,
        front_extension,
        axis="z",
        positive_elem="left_guide_rail",
        negative_elem="left_runner_keeper",
        max_penetration=0.0035,
    )
    ctx.expect_gap(
        chassis,
        front_extension,
        axis="z",
        positive_elem="right_guide_rail",
        negative_elem="right_runner_keeper",
        max_penetration=0.0035,
    )
    ctx.expect_overlap(
        front_extension,
        chassis,
        elem_a="extension_panel",
        elem_b="main_deck",
        axes="xy",
        min_overlap=0.18,
    )
    ctx.expect_within(front_extension, chassis, axes="x", margin=0.03)

    for prefix in ("rear_left", "rear_right", "front_left", "front_right"):
        caster = object_model.get_part(f"{prefix}_caster_swivel")
        wheel = object_model.get_part(f"{prefix}_caster_wheel")
        swivel_joint = object_model.get_articulation(f"{prefix}_caster_swivel_joint")
        spin_joint = object_model.get_articulation(f"{prefix}_caster_spin_joint")

        ctx.check(
            f"{prefix}_caster_swivel_axis",
            swivel_joint.axis == (0.0, 0.0, 1.0),
            f"Expected swivel axis (0, 0, 1), got {swivel_joint.axis}",
        )
        ctx.check(
            f"{prefix}_caster_spin_axis",
            spin_joint.axis == (1.0, 0.0, 0.0),
            f"Expected wheel spin axis (1, 0, 0), got {spin_joint.axis}",
        )
        ctx.expect_contact(caster, chassis, elem_a="top_plate", elem_b=f"{prefix}_pad")
        ctx.expect_overlap(caster, wheel, elem_a="axle", elem_b="hub", axes="xz", min_overlap=0.007)

    extension_rest = ctx.part_world_position(front_extension)
    assert extension_rest is not None
    with ctx.pose({front_extension_slide: 0.220}):
        extension_extended = ctx.part_world_position(front_extension)
        assert extension_extended is not None
        assert extension_extended[1] > extension_rest[1] + 0.20
        assert abs(extension_extended[2] - extension_rest[2]) < 1e-6
        ctx.expect_gap(
            chassis,
            front_extension,
            axis="z",
            positive_elem="left_guide_rail",
            negative_elem="left_runner_lower",
            min_gap=0.004,
            max_gap=0.006,
        )
        ctx.expect_gap(
            chassis,
            front_extension,
            axis="z",
            positive_elem="right_guide_rail",
            negative_elem="right_runner_keeper",
            max_penetration=0.0035,
        )
        ctx.expect_overlap(
            front_extension,
            chassis,
            elem_a="left_runner_lower",
            elem_b="left_guide_rail",
            axes="y",
            min_overlap=0.10,
        )

    handle_rest = ctx.part_world_aabb(rear_handle)
    assert handle_rest is not None
    with ctx.pose({rear_handle_fold: 1.50}):
        handle_folded = ctx.part_world_aabb(rear_handle)
        assert handle_folded is not None
        assert handle_folded[1][2] < handle_rest[1][2] - 0.55
        assert handle_folded[1][1] > handle_rest[1][1] + 0.45
        ctx.expect_overlap(rear_handle, chassis, axes="xy", min_overlap=0.28)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
