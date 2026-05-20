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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flat_top_patio_grill")

    body_paint = model.material("body_paint", rgba=(0.13, 0.13, 0.14, 1.0))
    stainless = model.material("stainless", rgba=(0.74, 0.75, 0.77, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.10, 0.10, 0.10, 1.0))
    caster_metal = model.material("caster_metal", rgba=(0.68, 0.69, 0.71, 1.0))

    body_width = 1.00
    body_depth = 0.58
    body_height = 0.16
    body_top = body_height / 2.0

    plate_width = 0.90
    plate_depth = 0.50
    plate_thickness = 0.02

    leg_size = 0.06
    leg_height = 0.58
    leg_mount_z = -body_height / 2.0

    caster_stem_radius = 0.012
    caster_stem_length = 0.04
    wheel_radius = 0.032
    wheel_width = 0.024

    cart_body = model.part("cart_body")
    cart_body.visual(
        Box((body_width, body_depth, body_height)),
        material=body_paint,
        name="body_shell",
    )
    cart_body.visual(
        Box((0.18, 0.36, 0.018)),
        origin=Origin(xyz=(-0.59, 0.0, body_top + 0.009)),
        material=stainless,
        name="left_side_shelf",
    )
    cart_body.visual(
        Box((0.18, 0.36, 0.018)),
        origin=Origin(xyz=(0.59, 0.0, body_top + 0.009)),
        material=stainless,
        name="right_side_shelf",
    )

    cooking_plate = model.part("cooking_plate")
    cooking_plate.visual(
        Box((plate_width, plate_depth, plate_thickness)),
        origin=Origin(xyz=(0.0, 0.0, plate_thickness / 2.0)),
        material=stainless,
        name="griddle_plate",
    )
    model.articulation(
        "body_to_plate",
        ArticulationType.FIXED,
        parent=cart_body,
        child=cooking_plate,
        origin=Origin(xyz=(0.0, 0.0, body_top)),
    )

    cover = model.part("cover")
    cover.visual(
        Box((0.98, 0.56, 0.012)),
        origin=Origin(xyz=(0.0, 0.28, 0.186)),
        material=body_paint,
        name="cover_top",
    )
    cover.visual(
        Box((0.98, 0.012, 0.18)),
        origin=Origin(xyz=(0.0, 0.006, 0.09)),
        material=body_paint,
        name="cover_rear_panel",
    )
    cover.visual(
        Box((0.98, 0.012, 0.18)),
        origin=Origin(xyz=(0.0, 0.554, 0.09)),
        material=body_paint,
        name="cover_front_panel",
    )
    cover.visual(
        Box((0.012, 0.56, 0.18)),
        origin=Origin(xyz=(-0.484, 0.28, 0.09)),
        material=body_paint,
        name="cover_left_panel",
    )
    cover.visual(
        Box((0.012, 0.56, 0.18)),
        origin=Origin(xyz=(0.484, 0.28, 0.09)),
        material=body_paint,
        name="cover_right_panel",
    )
    cover.visual(
        Box((0.03, 0.014, 0.05)),
        origin=Origin(xyz=(-0.23, 0.567, 0.105)),
        material=stainless,
        name="cover_handle_left_standoff",
    )
    cover.visual(
        Box((0.03, 0.014, 0.05)),
        origin=Origin(xyz=(0.23, 0.567, 0.105)),
        material=stainless,
        name="cover_handle_right_standoff",
    )
    cover.visual(
        Cylinder(radius=0.01, length=0.62),
        origin=Origin(xyz=(0.0, 0.579, 0.13), rpy=(0.0, pi / 2.0, 0.0)),
        material=stainless,
        name="cover_handle_bar",
    )
    model.articulation(
        "body_to_cover",
        ArticulationType.REVOLUTE,
        parent=cart_body,
        child=cover,
        origin=Origin(xyz=(0.0, -0.284, body_top)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=0.0,
            upper=1.35,
        ),
    )

    leg_positions = {
        "front_left": (-0.42, 0.23),
        "front_right": (0.42, 0.23),
        "rear_left": (-0.42, -0.23),
        "rear_right": (0.42, -0.23),
    }

    for corner, (x_pos, y_pos) in leg_positions.items():
        leg = model.part(f"{corner}_leg")
        leg.visual(
            Box((leg_size, leg_size, leg_height)),
            origin=Origin(xyz=(0.0, 0.0, -leg_height / 2.0)),
            material=body_paint,
            name="leg_post",
        )
        model.articulation(
            f"body_to_{corner}_leg",
            ArticulationType.FIXED,
            parent=cart_body,
            child=leg,
            origin=Origin(xyz=(x_pos, y_pos, leg_mount_z)),
        )

        caster_fork = model.part(f"{corner}_caster_fork")
        caster_fork.visual(
            Cylinder(radius=caster_stem_radius, length=caster_stem_length),
            origin=Origin(xyz=(0.0, 0.0, -caster_stem_length / 2.0)),
            material=caster_metal,
            name="caster_stem",
        )
        caster_fork.visual(
            Box((0.05, 0.036, 0.018)),
            origin=Origin(xyz=(0.0, 0.0, -0.034)),
            material=caster_metal,
            name="caster_crown",
        )
        caster_fork.visual(
            Box((0.008, 0.024, 0.068)),
            origin=Origin(xyz=(-0.016, 0.0, -0.075)),
            material=caster_metal,
            name="caster_left_yoke",
        )
        caster_fork.visual(
            Box((0.008, 0.024, 0.068)),
            origin=Origin(xyz=(0.016, 0.0, -0.075)),
            material=caster_metal,
            name="caster_right_yoke",
        )
        model.articulation(
            f"{corner}_caster_swivel",
            ArticulationType.CONTINUOUS,
            parent=leg,
            child=caster_fork,
            origin=Origin(xyz=(0.0, 0.0, -leg_height)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=3.0, velocity=8.0),
        )

        caster_wheel = model.part(f"{corner}_caster_wheel")
        caster_wheel.visual(
            Cylinder(radius=wheel_radius, length=wheel_width),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=wheel_rubber,
            name="wheel_tread",
        )
        model.articulation(
            f"{corner}_wheel_spin",
            ArticulationType.CONTINUOUS,
            parent=caster_fork,
            child=caster_wheel,
            origin=Origin(xyz=(0.0, 0.0, -0.075)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=20.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cart_body = object_model.get_part("cart_body")
    cooking_plate = object_model.get_part("cooking_plate")
    cover = object_model.get_part("cover")
    cover_hinge = object_model.get_articulation("body_to_cover")
    cover_limits = cover_hinge.motion_limits
    corners = ("front_left", "front_right", "rear_left", "rear_right")

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

    def part_exists(name: str) -> bool:
        try:
            object_model.get_part(name)
        except Exception:
            return False
        return True

    def articulation_exists(name: str) -> bool:
        try:
            object_model.get_articulation(name)
        except Exception:
            return False
        return True

    expected_parts = ["cart_body", "cooking_plate", "cover"]
    expected_joints = ["body_to_plate", "body_to_cover"]
    for corner in corners:
        expected_parts.extend(
            [
                f"{corner}_leg",
                f"{corner}_caster_fork",
                f"{corner}_caster_wheel",
            ]
        )
        expected_joints.extend(
            [
                f"body_to_{corner}_leg",
                f"{corner}_caster_swivel",
                f"{corner}_wheel_spin",
            ]
        )

    for name in expected_parts:
        ctx.check(f"part present: {name}", part_exists(name), f"missing part {name}")
    for name in expected_joints:
        ctx.check(
            f"articulation present: {name}",
            articulation_exists(name),
            f"missing articulation {name}",
        )

    ctx.expect_contact(cooking_plate, cart_body, name="cooking plate seats on cart body")
    ctx.expect_overlap(
        cover,
        cooking_plate,
        axes="xy",
        min_overlap=0.45,
        name="cover spans the cooking plate footprint",
    )
    with ctx.pose({cover_hinge: 0.0}):
        ctx.expect_contact(cover, cart_body, name="cover rests on cart body when closed")
    with ctx.pose({cover_hinge: 1.2}):
        ctx.expect_gap(
            cover,
            cooking_plate,
            axis="z",
            min_gap=0.12,
            positive_elem="cover_front_panel",
            negative_elem="griddle_plate",
            name="open cover front clears the cooking plate",
        )

    ctx.check(
        "cover hinge axis is rear horizontal",
        tuple(cover_hinge.axis) == (1.0, 0.0, 0.0),
        f"expected hinge axis (1, 0, 0), got {cover_hinge.axis}",
    )
    ctx.check(
        "cover hinge limits open upward",
        cover_limits is not None
        and cover_limits.lower == 0.0
        and cover_limits.upper is not None
        and 1.2 <= cover_limits.upper <= 1.5,
        f"unexpected cover motion limits {cover_limits}",
    )

    for corner in corners:
        leg = object_model.get_part(f"{corner}_leg")
        caster_fork = object_model.get_part(f"{corner}_caster_fork")
        caster_wheel = object_model.get_part(f"{corner}_caster_wheel")
        caster_swivel = object_model.get_articulation(f"{corner}_caster_swivel")
        wheel_spin = object_model.get_articulation(f"{corner}_wheel_spin")

        ctx.expect_contact(leg, cart_body, name=f"{corner} leg is mounted to the cart body")
        ctx.expect_contact(
            caster_fork,
            leg,
            name=f"{corner} caster fork is mounted below its leg",
        )
        ctx.expect_contact(
            caster_wheel,
            caster_fork,
            name=f"{corner} caster wheel is supported by its fork",
        )
        ctx.expect_within(
            caster_wheel,
            caster_fork,
            axes="x",
            margin=0.0,
            name=f"{corner} caster wheel stays between the fork cheeks",
        )
        ctx.check(
            f"{corner} caster swivel axis is vertical",
            tuple(caster_swivel.axis) == (0.0, 0.0, 1.0),
            f"expected swivel axis (0, 0, 1), got {caster_swivel.axis}",
        )
        ctx.check(
            f"{corner} caster wheel spins on a horizontal axle",
            tuple(wheel_spin.axis) == (1.0, 0.0, 0.0),
            f"expected wheel axis (1, 0, 0), got {wheel_spin.axis}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
