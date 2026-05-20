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
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_box_cutter")

    shell_body = model.material("shell_body", rgba=(0.77, 0.80, 0.82, 1.0))
    shell_rubber = model.material("shell_rubber", rgba=(0.20, 0.21, 0.23, 1.0))
    carrier_plastic = model.material("carrier_plastic", rgba=(0.87, 0.57, 0.14, 1.0))
    steel = model.material("steel", rgba=(0.76, 0.78, 0.80, 1.0))

    def utility_blade_mesh():
        profile = [
            (-0.006, 0.0000),
            (0.011, 0.0000),
            (0.022, 0.0023),
            (0.031, 0.0048),
            (0.028, 0.0065),
            (0.011, 0.0054),
            (-0.006, 0.0054),
        ]
        return mesh_from_geometry(ExtrudeGeometry(profile, 0.0009), "utility_blade")

    blade_mesh = utility_blade_mesh()

    handle_shell = model.part("handle_shell")
    handle_shell.visual(
        Box((0.112, 0.040, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=shell_body,
        name="bottom_floor",
    )
    handle_shell.visual(
        Box((0.110, 0.005, 0.014)),
        origin=Origin(xyz=(-0.001, 0.0165, 0.011)),
        material=shell_body,
        name="left_wall",
    )
    handle_shell.visual(
        Box((0.110, 0.005, 0.014)),
        origin=Origin(xyz=(-0.001, -0.0165, 0.011)),
        material=shell_body,
        name="right_wall",
    )
    handle_shell.visual(
        Box((0.026, 0.030, 0.010)),
        origin=Origin(xyz=(-0.043, 0.0, 0.013)),
        material=shell_body,
        name="rear_cap",
    )
    handle_shell.visual(
        Cylinder(radius=0.009, length=0.028),
        origin=Origin(xyz=(-0.056, 0.0, 0.009), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=shell_body,
        name="rear_bumper",
    )
    handle_shell.visual(
        Box((0.080, 0.007, 0.004)),
        origin=Origin(xyz=(-0.007, 0.0115, 0.016)),
        material=shell_body,
        name="left_top_rail",
    )
    handle_shell.visual(
        Box((0.080, 0.007, 0.004)),
        origin=Origin(xyz=(-0.007, -0.0115, 0.016)),
        material=shell_body,
        name="right_top_rail",
    )
    handle_shell.visual(
        Box((0.022, 0.004, 0.009)),
        origin=Origin(xyz=(0.042, 0.010, 0.0095)),
        material=steel,
        name="left_nose_guide",
    )
    handle_shell.visual(
        Box((0.022, 0.004, 0.009)),
        origin=Origin(xyz=(0.042, -0.010, 0.0095)),
        material=steel,
        name="right_nose_guide",
    )
    handle_shell.visual(
        Cylinder(radius=0.0035, length=0.007),
        origin=Origin(xyz=(0.054, 0.0155, 0.0155), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_lug_left",
    )
    handle_shell.visual(
        Cylinder(radius=0.0035, length=0.007),
        origin=Origin(xyz=(0.054, -0.0155, 0.0155), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_lug_right",
    )
    handle_shell.visual(
        Box((0.060, 0.0015, 0.010)),
        origin=Origin(xyz=(-0.006, 0.0196, 0.010)),
        material=shell_rubber,
        name="left_grip_pad",
    )
    handle_shell.visual(
        Box((0.060, 0.0015, 0.010)),
        origin=Origin(xyz=(-0.006, -0.0196, 0.010)),
        material=shell_rubber,
        name="right_grip_pad",
    )
    handle_shell.inertial = Inertial.from_geometry(
        Box((0.125, 0.042, 0.020)),
        mass=0.34,
        origin=Origin(xyz=(-0.002, 0.0, 0.010)),
    )

    blade_carrier = model.part("blade_carrier")
    blade_carrier.visual(
        Box((0.040, 0.014, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=carrier_plastic,
        name="carrier_block",
    )
    blade_carrier.visual(
        Box((0.018, 0.010, 0.006)),
        origin=Origin(xyz=(-0.003, 0.0, 0.013)),
        material=carrier_plastic,
        name="thumb_slider",
    )
    blade_carrier.visual(
        Box((0.010, 0.012, 0.003)),
        origin=Origin(xyz=(0.013, 0.0, 0.0085)),
        material=steel,
        name="blade_clamp",
    )
    for ridge_x in (-0.008, -0.003, 0.002):
        blade_carrier.visual(
            Box((0.002, 0.0102, 0.0012)),
            origin=Origin(xyz=(ridge_x, 0.0, 0.0166)),
            material=shell_rubber,
            name=f"thumb_ridge_{int((ridge_x + 0.01) * 1000)}",
        )
    blade_carrier.visual(
        blade_mesh,
        origin=Origin(xyz=(0.022, 0.0, 0.0042), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="blade",
    )
    blade_carrier.inertial = Inertial.from_geometry(
        Box((0.060, 0.015, 0.018)),
        mass=0.06,
        origin=Origin(xyz=(0.010, 0.0, 0.009)),
    )

    guard_cover = model.part("guard_cover")
    guard_cover.visual(
        Cylinder(radius=0.0032, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="guard_barrel",
    )
    guard_cover.visual(
        Box((0.018, 0.014, 0.002)),
        origin=Origin(xyz=(0.011, 0.0, -0.003)),
        material=shell_body,
        name="guard_plate",
    )
    guard_cover.visual(
        Box((0.002, 0.014, 0.007)),
        origin=Origin(xyz=(0.020, 0.0, -0.0065)),
        material=shell_body,
        name="guard_lip",
    )
    guard_cover.visual(
        Box((0.012, 0.004, 0.003)),
        origin=Origin(xyz=(0.009, 0.0, -0.0015)),
        material=shell_rubber,
        name="guard_rib",
    )
    guard_cover.inertial = Inertial.from_geometry(
        Box((0.024, 0.024, 0.012)),
        mass=0.03,
        origin=Origin(xyz=(0.010, 0.0, -0.004)),
    )

    model.articulation(
        "carrier_slide",
        ArticulationType.PRISMATIC,
        parent=handle_shell,
        child=blade_carrier,
        origin=Origin(xyz=(-0.004, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.15, lower=0.0, upper=0.024),
    )
    model.articulation(
        "nose_guard_hinge",
        ArticulationType.REVOLUTE,
        parent=handle_shell,
        child=guard_cover,
        origin=Origin(xyz=(0.054, 0.0, 0.0155)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=0.0,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    handle_shell = object_model.get_part("handle_shell")
    blade_carrier = object_model.get_part("blade_carrier")
    guard_cover = object_model.get_part("guard_cover")
    carrier_slide = object_model.get_articulation("carrier_slide")
    nose_guard_hinge = object_model.get_articulation("nose_guard_hinge")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "carrier_slide_axis",
        tuple(carrier_slide.axis) == (1.0, 0.0, 0.0),
        f"unexpected slide axis: {carrier_slide.axis}",
    )
    ctx.check(
        "guard_hinge_axis",
        tuple(nose_guard_hinge.axis) == (0.0, -1.0, 0.0),
        f"unexpected guard hinge axis: {nose_guard_hinge.axis}",
    )

    ctx.expect_contact(
        blade_carrier,
        handle_shell,
        elem_a="carrier_block",
        elem_b="bottom_floor",
        contact_tol=5e-5,
        name="carrier_supported_on_shell_floor",
    )
    ctx.expect_contact(
        guard_cover,
        handle_shell,
        elem_a="guard_barrel",
        elem_b="hinge_lug_left",
        contact_tol=5e-5,
        name="guard_barrel_clipped_to_left_hinge_lug",
    )
    ctx.expect_contact(
        guard_cover,
        handle_shell,
        elem_a="guard_barrel",
        elem_b="hinge_lug_right",
        contact_tol=5e-5,
        name="guard_barrel_clipped_to_right_hinge_lug",
    )
    ctx.expect_gap(
        guard_cover,
        blade_carrier,
        axis="z",
        positive_elem="guard_plate",
        negative_elem="blade",
        min_gap=0.0004,
        max_gap=0.004,
        name="closed_guard_plate_clears_retracted_blade",
    )
    ctx.expect_within(
        blade_carrier,
        handle_shell,
        axes="y",
        margin=0.001,
        name="carrier_stays_centered_in_handle_width",
    )

    carrier_rest = ctx.part_world_position(blade_carrier)
    assert carrier_rest is not None
    with ctx.pose({carrier_slide: 0.024}):
        carrier_extended = ctx.part_world_position(blade_carrier)
        assert carrier_extended is not None
        assert carrier_extended[0] > carrier_rest[0] + 0.020
        ctx.expect_contact(
            blade_carrier,
            handle_shell,
            elem_a="carrier_block",
            elem_b="bottom_floor",
            contact_tol=5e-5,
            name="carrier_remains_supported_when_extended",
        )
        ctx.expect_within(
            blade_carrier,
            handle_shell,
            axes="y",
            margin=0.001,
            name="extended_carrier_remains_centered_in_handle_width",
        )

    guard_closed_aabb = ctx.part_world_aabb(guard_cover)
    assert guard_closed_aabb is not None
    with ctx.pose({nose_guard_hinge: 1.25}):
        guard_open_aabb = ctx.part_world_aabb(guard_cover)
        assert guard_open_aabb is not None
        assert guard_open_aabb[1][2] > guard_closed_aabb[1][2] + 0.010
        ctx.expect_contact(
            guard_cover,
            handle_shell,
            elem_a="guard_barrel",
            elem_b="hinge_lug_left",
            contact_tol=5e-5,
            name="guard_remains_attached_at_open_pose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
