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


def _extrude_xz_profile(
    profile: list[tuple[float, float]],
    thickness: float,
    name: str,
):
    geom = ExtrudeGeometry(profile, thickness)
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="snap_off_utility_knife")

    body_yellow = model.material("body_yellow", rgba=(0.91, 0.73, 0.14, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.16, 0.17, 0.18, 1.0))
    grip_black = model.material("grip_black", rgba=(0.08, 0.08, 0.09, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.74, 0.77, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.82, 0.83, 0.84, 1.0))

    handle = model.part("handle")

    side_profile = [
        (-0.079, 0.002),
        (-0.072, 0.000),
        (-0.024, 0.000),
        (0.056, 0.001),
        (0.076, 0.004),
        (0.082, 0.007),
        (0.076, 0.0155),
        (0.058, 0.0190),
        (0.012, 0.0190),
        (-0.042, 0.0185),
        (-0.070, 0.0172),
        (-0.079, 0.0130),
    ]
    side_shell_mesh = _extrude_xz_profile(side_profile, 0.003, "knife_handle_side_shell")

    handle.visual(
        side_shell_mesh,
        origin=Origin(xyz=(0.0, 0.0155, 0.0)),
        material=body_yellow,
        name="left_shell",
    )
    handle.visual(
        side_shell_mesh,
        origin=Origin(xyz=(0.0, -0.0155, 0.0)),
        material=body_yellow,
        name="right_shell",
    )
    handle.visual(
        Box((0.146, 0.028, 0.004)),
        origin=Origin(xyz=(0.002, 0.0, 0.002)),
        material=body_yellow,
        name="tray",
    )
    handle.visual(
        Box((0.118, 0.0115, 0.004)),
        origin=Origin(xyz=(0.004, 0.01125, 0.016)),
        material=body_yellow,
        name="left_top_rail",
    )
    handle.visual(
        Box((0.118, 0.0115, 0.004)),
        origin=Origin(xyz=(0.004, -0.01125, 0.016)),
        material=body_yellow,
        name="right_top_rail",
    )
    handle.visual(
        Box((0.010, 0.024, 0.010)),
        origin=Origin(xyz=(-0.074, 0.0, 0.007)),
        material=body_yellow,
        name="rear_core",
    )
    handle.visual(
        Cylinder(radius=0.0035, length=0.008),
        origin=Origin(xyz=(-0.079, 0.009, 0.015), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_grey,
        name="left_hinge_knuckle",
    )
    handle.visual(
        Cylinder(radius=0.0035, length=0.008),
        origin=Origin(xyz=(-0.079, -0.009, 0.015), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_grey,
        name="right_hinge_knuckle",
    )
    handle.visual(
        Box((0.018, 0.022, 0.004)),
        origin=Origin(xyz=(0.073, 0.0, 0.002)),
        material=steel,
        name="nose_floor",
    )
    handle.visual(
        Box((0.018, 0.008, 0.010)),
        origin=Origin(xyz=(0.073, 0.010, 0.009)),
        material=steel,
        name="left_nose_cheek",
    )
    handle.visual(
        Box((0.018, 0.008, 0.010)),
        origin=Origin(xyz=(0.073, -0.010, 0.009)),
        material=steel,
        name="right_nose_cheek",
    )
    handle.visual(
        Box((0.054, 0.0008, 0.011)),
        origin=Origin(xyz=(-0.012, 0.0174, 0.0085)),
        material=grip_black,
        name="left_grip_pad",
    )
    handle.visual(
        Box((0.054, 0.0008, 0.011)),
        origin=Origin(xyz=(-0.012, -0.0174, 0.0085)),
        material=grip_black,
        name="right_grip_pad",
    )
    handle.visual(
        Box((0.030, 0.0008, 0.007)),
        origin=Origin(xyz=(0.040, 0.0174, 0.0105)),
        material=grip_black,
        name="left_front_grip",
    )
    handle.visual(
        Box((0.030, 0.0008, 0.007)),
        origin=Origin(xyz=(0.040, -0.0174, 0.0105)),
        material=grip_black,
        name="right_front_grip",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.165, 0.034, 0.020)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    blade_carrier = model.part("blade_carrier")
    blade_profile = [
        (0.070, 0.0015),
        (0.106, 0.0015),
        (0.115, 0.0068),
        (0.101, 0.0145),
        (0.070, 0.0145),
    ]
    blade_mesh = _extrude_xz_profile(blade_profile, 0.0012, "utility_blade_profile")

    blade_carrier.visual(
        Box((0.100, 0.012, 0.006)),
        origin=Origin(xyz=(0.050, 0.0, 0.003)),
        material=steel,
        name="carrier_body",
    )
    blade_carrier.visual(
        Box((0.060, 0.010, 0.003)),
        origin=Origin(xyz=(0.045, 0.0, 0.0075)),
        material=steel,
        name="carrier_clamp",
    )
    blade_carrier.visual(
        blade_mesh,
        material=blade_steel,
        name="blade",
    )
    blade_carrier.visual(
        Box((0.0004, 0.0013, 0.0115)),
        origin=Origin(xyz=(0.079, 0.0, 0.0085)),
        material=dark_grey,
        name="segment_line_1",
    )
    blade_carrier.visual(
        Box((0.0004, 0.0013, 0.0115)),
        origin=Origin(xyz=(0.091, 0.0, 0.0085)),
        material=dark_grey,
        name="segment_line_2",
    )
    blade_carrier.visual(
        Box((0.0004, 0.0013, 0.0095)),
        origin=Origin(xyz=(0.102, 0.0, 0.0090)),
        material=dark_grey,
        name="segment_line_3",
    )
    blade_carrier.inertial = Inertial.from_geometry(
        Box((0.116, 0.014, 0.015)),
        mass=0.05,
        origin=Origin(xyz=(0.058, 0.0, 0.0075)),
    )

    slider = model.part("slider")
    slider.visual(
        Box((0.013, 0.005, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=grip_black,
        name="slider_stem",
    )
    slider.visual(
        Box((0.021, 0.010, 0.006)),
        origin=Origin(xyz=(0.0015, 0.0, 0.011)),
        material=grip_black,
        name="thumb_button",
    )
    slider.visual(
        Box((0.002, 0.010, 0.001)),
        origin=Origin(xyz=(-0.0050, 0.0, 0.0145)),
        material=dark_grey,
        name="thumb_ridge_left",
    )
    slider.visual(
        Box((0.002, 0.010, 0.001)),
        origin=Origin(xyz=(0.0015, 0.0, 0.0145)),
        material=dark_grey,
        name="thumb_ridge_mid",
    )
    slider.visual(
        Box((0.002, 0.010, 0.001)),
        origin=Origin(xyz=(0.0080, 0.0, 0.0145)),
        material=dark_grey,
        name="thumb_ridge_right",
    )
    slider.inertial = Inertial.from_geometry(
        Box((0.021, 0.010, 0.015)),
        mass=0.01,
        origin=Origin(xyz=(0.0015, 0.0, 0.0075)),
    )

    rear_cap = model.part("rear_cap")
    rear_cap.visual(
        Box((0.014, 0.028, 0.014)),
        origin=Origin(xyz=(-0.009, 0.0, -0.006)),
        material=dark_grey,
        name="cap_body",
    )
    rear_cap.visual(
        Box((0.010, 0.020, 0.003)),
        origin=Origin(xyz=(-0.011, 0.0, -0.0095)),
        material=steel,
        name="breaker_insert",
    )
    rear_cap.visual(
        Cylinder(radius=0.0035, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_grey,
        name="cap_hinge_knuckle",
    )
    rear_cap.inertial = Inertial.from_geometry(
        Box((0.014, 0.028, 0.014)),
        mass=0.015,
        origin=Origin(xyz=(-0.009, 0.0, -0.006)),
    )

    model.articulation(
        "handle_to_blade_carrier",
        ArticulationType.PRISMATIC,
        parent=handle,
        child=blade_carrier,
        origin=Origin(xyz=(-0.020, 0.0, 0.004)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.20,
            lower=0.0,
            upper=0.036,
        ),
    )
    model.articulation(
        "blade_carrier_to_slider",
        ArticulationType.FIXED,
        parent=blade_carrier,
        child=slider,
        origin=Origin(xyz=(0.052, 0.0, 0.006)),
    )
    model.articulation(
        "handle_to_rear_cap",
        ArticulationType.REVOLUTE,
        parent=handle,
        child=rear_cap,
        origin=Origin(xyz=(-0.079, 0.0, 0.015)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=0.0,
            upper=1.20,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    handle = object_model.get_part("handle")
    blade_carrier = object_model.get_part("blade_carrier")
    slider = object_model.get_part("slider")
    rear_cap = object_model.get_part("rear_cap")

    carrier_slide = object_model.get_articulation("handle_to_blade_carrier")
    cap_hinge = object_model.get_articulation("handle_to_rear_cap")

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

    ctx.expect_contact(
        blade_carrier,
        handle,
        elem_a="carrier_body",
        elem_b="tray",
        name="carrier_body_rides_on_tray",
    )
    ctx.expect_within(
        blade_carrier,
        handle,
        axes="yz",
        inner_elem="carrier_body",
        margin=0.0,
        name="carrier_body_stays_in_channel",
    )
    ctx.expect_contact(
        slider,
        blade_carrier,
        elem_a="slider_stem",
        elem_b="carrier_body",
        name="slider_stem_contacts_carrier",
    )
    ctx.expect_contact(
        rear_cap,
        handle,
        elem_a="cap_hinge_knuckle",
        elem_b="left_hinge_knuckle",
        name="rear_cap_clipped_left",
    )
    ctx.expect_contact(
        rear_cap,
        handle,
        elem_a="cap_hinge_knuckle",
        elem_b="right_hinge_knuckle",
        name="rear_cap_clipped_right",
    )

    carrier_rest = ctx.part_world_position(blade_carrier)
    slider_rest = ctx.part_world_position(slider)
    cap_rest_aabb = ctx.part_world_aabb(rear_cap)

    ctx.check(
        "primary_axes_are_correct",
        carrier_slide.axis == (1.0, 0.0, 0.0) and cap_hinge.axis == (0.0, 1.0, 0.0),
        details=f"carrier axis={carrier_slide.axis}, cap axis={cap_hinge.axis}",
    )

    assert carrier_rest is not None
    assert slider_rest is not None
    assert cap_rest_aabb is not None

    with ctx.pose({carrier_slide: 0.030}):
        carrier_open = ctx.part_world_position(blade_carrier)
        slider_open = ctx.part_world_position(slider)
        assert carrier_open is not None
        assert slider_open is not None

        carrier_dx = carrier_open[0] - carrier_rest[0]
        slider_dx = slider_open[0] - slider_rest[0]
        ctx.check(
            "blade_carrier_slides_forward",
            carrier_dx > 0.029,
            details=f"carrier moved {carrier_dx:.6f} m along x",
        )
        ctx.check(
            "top_slider_tracks_carrier",
            abs(slider_dx - carrier_dx) < 1e-6,
            details=f"slider moved {slider_dx:.6f} m while carrier moved {carrier_dx:.6f} m",
        )
        ctx.expect_contact(
            blade_carrier,
            handle,
            elem_a="carrier_body",
            elem_b="tray",
            name="carrier_stays_supported_when_extended",
        )
        ctx.expect_within(
            blade_carrier,
            handle,
            axes="yz",
            inner_elem="carrier_body",
            margin=0.0,
            name="carrier_body_remains_centered_when_extended",
        )

    with ctx.pose({cap_hinge: 1.10}):
        cap_open_aabb = ctx.part_world_aabb(rear_cap)
        assert cap_open_aabb is not None
        ctx.check(
            "rear_cap_opens_on_hinge",
            cap_open_aabb[1][2] > cap_rest_aabb[1][2] + 0.010,
            details=(
                f"rear cap max z changed from {cap_rest_aabb[1][2]:.6f} "
                f"to {cap_open_aabb[1][2]:.6f}"
            ),
        )
        ctx.expect_contact(
            rear_cap,
            handle,
            elem_a="cap_hinge_knuckle",
            elem_b="left_hinge_knuckle",
            name="rear_cap_stays_attached_left_when_open",
        )
        ctx.expect_contact(
            rear_cap,
            handle,
            elem_a="cap_hinge_knuckle",
            elem_b="right_hinge_knuckle",
            name="rear_cap_stays_attached_right_when_open",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
