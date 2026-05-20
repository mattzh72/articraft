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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(
    radius: float,
    *,
    center: tuple[float, float],
    segments: int = 24,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * index / segments),
            cy + radius * math.sin(2.0 * math.pi * index / segments),
        )
        for index in range(segments)
    ]


def _extrude_xz_profile(
    outer_profile: list[tuple[float, float]],
    *,
    thickness: float,
    name: str,
    hole_profiles: tuple[list[tuple[float, float]], ...] = (),
):
    if hole_profiles:
        geom = ExtrudeWithHolesGeometry(
            outer_profile,
            hole_profiles,
            thickness,
            cap=True,
            center=True,
            closed=True,
        )
    else:
        geom = ExtrudeGeometry(
            outer_profile,
            thickness,
            cap=True,
            center=True,
            closed=True,
        )
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _handle_side_profile() -> list[tuple[float, float]]:
    return [
        (-0.082, 0.0030),
        (-0.076, 0.0215),
        (-0.056, 0.0248),
        (-0.010, 0.0260),
        (0.038, 0.0260),
        (0.063, 0.0220),
        (0.078, 0.0170),
        (0.082, 0.0145),
        (0.079, 0.0095),
        (0.050, 0.0060),
        (-0.052, 0.0028),
        (-0.074, 0.0026),
    ]


def _blade_mesh(name: str):
    blade_profile = [
        (-0.006, 0.0018),
        (-0.006, 0.0122),
        (0.018, 0.0122),
        (0.036, 0.0066),
        (0.050, 0.0066),
        (0.042, 0.0018),
    ]
    geom = ExtrudeGeometry(
        blade_profile,
        0.0008,
        cap=True,
        center=True,
        closed=True,
    )
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retractable_utility_knife")

    shell_metal = model.material("shell_metal", rgba=(0.70, 0.72, 0.75, 1.0))
    dark_polymer = model.material("dark_polymer", rgba=(0.14, 0.14, 0.15, 1.0))
    wheel_black = model.material("wheel_black", rgba=(0.10, 0.10, 0.10, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.86, 0.87, 0.89, 1.0))

    body_length = 0.164
    body_width = 0.016
    body_height = 0.026
    shell_half_width = body_width * 0.5
    side_plate_thickness = 0.0014
    wheel_radius = 0.0066
    wheel_length = 0.0040

    side_profile = _handle_side_profile()
    wheel_opening = _circle_profile(0.0049, center=(0.004, 0.0182), segments=28)
    left_plate_mesh = _extrude_xz_profile(
        side_profile,
        thickness=side_plate_thickness,
        name="knife_left_plate",
    )
    right_plate_mesh = _extrude_xz_profile(
        side_profile,
        thickness=side_plate_thickness,
        name="knife_right_plate",
        hole_profiles=(wheel_opening,),
    )
    blade_mesh = _blade_mesh("knife_blade")

    handle_shell = model.part("handle_shell")
    handle_shell.visual(
        left_plate_mesh,
        origin=Origin(xyz=(0.0, -(shell_half_width - side_plate_thickness * 0.5), 0.0)),
        material=shell_metal,
        name="left_plate",
    )
    handle_shell.visual(
        right_plate_mesh,
        origin=Origin(xyz=(0.0, shell_half_width - side_plate_thickness * 0.5, 0.0)),
        material=shell_metal,
        name="right_plate",
    )
    handle_shell.visual(
        Box((0.142, body_width, 0.0050)),
        origin=Origin(xyz=(0.001, 0.0, 0.0025)),
        material=shell_metal,
        name="bottom_spine",
    )
    handle_shell.visual(
        Box((0.012, body_width, 0.019)),
        origin=Origin(xyz=(-0.076, 0.0, 0.0125)),
        material=shell_metal,
        name="rear_cap",
    )
    handle_shell.visual(
        Box((0.045, body_width, 0.0030)),
        origin=Origin(xyz=(-0.046, 0.0, 0.0238)),
        material=shell_metal,
        name="top_rear_bridge",
    )
    handle_shell.visual(
        Box((0.024, body_width, 0.0030)),
        origin=Origin(xyz=(0.066, 0.0, 0.0228)),
        material=shell_metal,
        name="top_front_bridge",
    )
    handle_shell.visual(
        Box((0.017, body_width, 0.0050)),
        origin=Origin(xyz=(0.069, 0.0, 0.0025)),
        material=shell_metal,
        name="nose_bridge",
    )
    handle_shell.inertial = Inertial.from_geometry(
        Box((body_length, body_width, body_height)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, body_height * 0.5)),
    )

    blade_carrier = model.part("blade_carrier")
    blade_carrier.visual(
        Box((0.052, 0.0108, 0.0100)),
        origin=Origin(xyz=(0.010, 0.0, 0.0100)),
        material=dark_polymer,
        name="carrier_body",
    )
    blade_carrier.visual(
        Box((0.018, 0.0100, 0.0100)),
        origin=Origin(xyz=(0.035, 0.0, 0.0100)),
        material=dark_polymer,
        name="carrier_shoe",
    )
    blade_carrier.visual(
        Box((0.019, 0.0038, 0.0100)),
        origin=Origin(xyz=(0.000, 0.0, 0.0170)),
        material=dark_polymer,
        name="slider_stem",
    )
    blade_carrier.visual(
        Box((0.022, 0.0086, 0.0052)),
        origin=Origin(xyz=(0.000, 0.0, 0.0244)),
        material=dark_polymer,
        name="thumb_slider",
    )
    blade_carrier.visual(
        blade_mesh,
        origin=Origin(xyz=(0.030, 0.0, 0.0)),
        material=blade_steel,
        name="blade",
    )
    blade_carrier.inertial = Inertial.from_geometry(
        Box((0.086, 0.011, 0.029)),
        mass=0.05,
        origin=Origin(xyz=(0.020, 0.0, 0.0145)),
    )

    lock_wheel = model.part("lock_wheel")
    lock_wheel.visual(
        Cylinder(radius=wheel_radius, length=wheel_length),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wheel_black,
        name="wheel_disk",
    )
    lock_wheel.visual(
        Cylinder(radius=0.0032, length=0.0020),
        origin=Origin(xyz=(0.0, 0.0015, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_polymer,
        name="wheel_hub",
    )
    lock_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=wheel_length),
        mass=0.015,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "carrier_slide",
        ArticulationType.PRISMATIC,
        parent=handle_shell,
        child=blade_carrier,
        origin=Origin(xyz=(0.012, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=0.18,
            lower=-0.012,
            upper=0.018,
        ),
    )
    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=handle_shell,
        child=lock_wheel,
        origin=Origin(
            xyz=(0.004, shell_half_width + wheel_length * 0.5, 0.0182),
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    handle_shell = object_model.get_part("handle_shell")
    blade_carrier = object_model.get_part("blade_carrier")
    lock_wheel = object_model.get_part("lock_wheel")
    carrier_slide = object_model.get_articulation("carrier_slide")
    wheel_spin = object_model.get_articulation("wheel_spin")

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
        "carrier articulation axis",
        tuple(carrier_slide.axis) == (1.0, 0.0, 0.0),
        f"expected prismatic axis along handle x, got {carrier_slide.axis}",
    )
    ctx.check(
        "wheel articulation axis",
        tuple(wheel_spin.axis) == (0.0, 1.0, 0.0),
        f"expected wheel axis along side-to-side y, got {wheel_spin.axis}",
    )

    ctx.expect_contact(
        blade_carrier,
        handle_shell,
        elem_a="carrier_body",
        name="carrier body rides on shell channel",
    )
    ctx.expect_contact(
        lock_wheel,
        handle_shell,
        elem_a="wheel_disk",
        elem_b="right_plate",
        name="lock wheel mounted on shell side wall",
    )
    ctx.expect_within(
        blade_carrier,
        handle_shell,
        axes="yz",
        inner_elem="carrier_body",
        margin=0.0,
        name="carrier body stays clipped inside channel footprint",
    )
    ctx.expect_within(
        blade_carrier,
        handle_shell,
        axes="yz",
        inner_elem="carrier_shoe",
        margin=0.0,
        name="carrier shoe stays within shell guide envelope",
    )

    shell_aabb = ctx.part_world_aabb(handle_shell)
    blade_aabb = ctx.part_element_world_aabb(blade_carrier, elem="blade")
    assert shell_aabb is not None
    assert blade_aabb is not None
    ctx.check(
        "blade reads as exposed at rest",
        blade_aabb[1][0] > shell_aabb[1][0] + 0.006,
        f"blade tip x={blade_aabb[1][0]:.4f} should project past shell nose x={shell_aabb[1][0]:.4f}",
    )

    carrier_rest = ctx.part_world_position(blade_carrier)
    wheel_rest = ctx.part_world_position(lock_wheel)
    assert carrier_rest is not None
    assert wheel_rest is not None

    with ctx.pose({carrier_slide: carrier_slide.motion_limits.upper}):
        carrier_extended = ctx.part_world_position(blade_carrier)
        extended_blade = ctx.part_element_world_aabb(blade_carrier, elem="blade")
        assert carrier_extended is not None
        assert extended_blade is not None
        ctx.check(
            "carrier extends forward",
            carrier_extended[0] > carrier_rest[0] + 0.015,
            f"carrier x moved from {carrier_rest[0]:.4f} to {carrier_extended[0]:.4f}",
        )
        ctx.expect_contact(
            blade_carrier,
            handle_shell,
            elem_a="carrier_body",
            name="carrier remains supported when extended",
        )
        ctx.expect_within(
            blade_carrier,
            handle_shell,
            axes="yz",
            inner_elem="carrier_body",
            margin=0.0,
            name="extended carrier stays laterally captured",
        )
        ctx.check(
            "blade extends further at upper travel",
            extended_blade[1][0] > shell_aabb[1][0] + 0.020,
            f"extended blade tip x={extended_blade[1][0]:.4f} should clearly project beyond shell nose x={shell_aabb[1][0]:.4f}",
        )

    with ctx.pose({carrier_slide: carrier_slide.motion_limits.lower}):
        ctx.expect_contact(
            blade_carrier,
            handle_shell,
            elem_a="carrier_body",
            name="carrier remains supported when retracted",
        )
        ctx.expect_within(
            blade_carrier,
            handle_shell,
            axes="yz",
            inner_elem="carrier_body",
            margin=0.0,
            name="retracted carrier stays clipped in shell",
        )

    with ctx.pose({wheel_spin: 1.8}):
        wheel_rotated = ctx.part_world_position(lock_wheel)
        assert wheel_rotated is not None
        ctx.expect_contact(
            lock_wheel,
            handle_shell,
            elem_a="wheel_disk",
            elem_b="right_plate",
            name="wheel stays seated on shell while rotating",
        )
        ctx.check(
            "wheel rotation does not translate wheel center",
            abs(wheel_rotated[0] - wheel_rest[0]) < 1e-6
            and abs(wheel_rotated[1] - wheel_rest[1]) < 1e-6
            and abs(wheel_rotated[2] - wheel_rest[2]) < 1e-6,
            f"wheel center moved from {wheel_rest} to {wheel_rotated}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
