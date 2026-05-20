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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _translate_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _add_side_antenna(part, *, side_sign: float, material) -> None:
    part.visual(
        Cylinder(radius=0.0065, length=0.004),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=material,
        name="pivot_barrel",
    )
    part.visual(
        Box((0.011, 0.014, 0.028)),
        origin=Origin(xyz=(side_sign * 0.012, 0.0, 0.014)),
        material=material,
        name="hinge_neck",
    )
    part.visual(
        Box((0.014, 0.022, 0.046)),
        origin=Origin(xyz=(side_sign * 0.014, 0.0, 0.039)),
        material=material,
        name="paddle_root",
    )
    part.visual(
        Box((0.008, 0.022, 0.102)),
        origin=Origin(xyz=(side_sign * 0.015, 0.0, 0.113)),
        material=material,
        name="paddle",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_mesh_router")

    shell_white = model.material("shell_white", rgba=(0.93, 0.94, 0.95, 1.0))
    shell_gray = model.material("shell_gray", rgba=(0.82, 0.84, 0.86, 1.0))
    charcoal = model.material("charcoal", rgba=(0.20, 0.22, 0.24, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))
    status_blue = model.material("status_blue", rgba=(0.55, 0.86, 1.0, 0.95))

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((0.248, 0.172, 0.032)),
        mass=0.65,
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
    )

    frame_outer = rounded_rect_profile(0.244, 0.168, 0.012, corner_segments=10)
    frame_inner = rounded_rect_profile(0.208, 0.132, 0.010, corner_segments=10)
    body.visual(
        _mesh(
            "router_bottom_frame",
            ExtrudeWithHolesGeometry(frame_outer, [frame_inner], height=0.012, center=True),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=shell_gray,
        name="bottom_frame",
    )

    vent_slot = rounded_rect_profile(0.010, 0.098, 0.0032, corner_segments=8)
    vent_holes = [
        _translate_profile(vent_slot, dx=x_pos, dy=0.0)
        for x_pos in (-0.072, -0.048, -0.024, 0.0, 0.024, 0.048, 0.072)
    ]
    top_outer = rounded_rect_profile(0.238, 0.162, 0.014, corner_segments=10)
    body.visual(
        _mesh(
            "router_top_shell",
            ExtrudeWithHolesGeometry(top_outer, vent_holes, height=0.016, center=True),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=shell_white,
        name="top_shell",
    )

    for index, (x_pos, y_pos) in enumerate(
        (
            (-0.094, -0.062),
            (0.094, -0.062),
            (-0.094, 0.062),
            (0.094, 0.062),
        ),
        start=1,
    ):
        body.visual(
            Box((0.016, 0.016, 0.003)),
            origin=Origin(xyz=(x_pos, y_pos, 0.0015)),
            material=rubber,
            name=f"foot_{index}",
        )

    body.visual(
        Box((0.034, 0.004, 0.0025)),
        origin=Origin(xyz=(0.0, -0.080, 0.018)),
        material=status_blue,
        name="status_led",
    )

    for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
        body.visual(
            Box((0.012, 0.036, 0.016)),
            origin=Origin(xyz=(side_sign * 0.125, 0.040, 0.020)),
            material=charcoal,
            name=f"{side_name}_pod_mount",
        )
        body.visual(
            Box((0.006, 0.036, 0.022)),
            origin=Origin(xyz=(side_sign * 0.128, 0.040, 0.020)),
            material=charcoal,
            name=f"{side_name}_pod",
        )
        body.visual(
            Cylinder(radius=0.0065, length=0.008),
            origin=Origin(
                xyz=(side_sign * 0.131, 0.046, 0.020),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material=charcoal,
            name=f"{side_name}_front_knuckle",
        )
        body.visual(
            Cylinder(radius=0.0065, length=0.008),
            origin=Origin(
                xyz=(side_sign * 0.131, 0.034, 0.020),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material=charcoal,
            name=f"{side_name}_rear_knuckle",
        )

    left_antenna = model.part("left_antenna")
    left_antenna.inertial = Inertial.from_geometry(
        Box((0.034, 0.024, 0.160)),
        mass=0.05,
        origin=Origin(xyz=(-0.015, 0.0, 0.080)),
    )
    _add_side_antenna(left_antenna, side_sign=-1.0, material=charcoal)

    right_antenna = model.part("right_antenna")
    right_antenna.inertial = Inertial.from_geometry(
        Box((0.034, 0.024, 0.160)),
        mass=0.05,
        origin=Origin(xyz=(0.015, 0.0, 0.080)),
    )
    _add_side_antenna(right_antenna, side_sign=1.0, material=charcoal)

    model.articulation(
        "left_antenna_fold",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_antenna,
        origin=Origin(xyz=(-0.131, 0.040, 0.020)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=-0.20, upper=1.25),
    )
    model.articulation(
        "right_antenna_fold",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_antenna,
        origin=Origin(xyz=(0.131, 0.040, 0.020)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=-0.20, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    left_antenna = object_model.get_part("left_antenna")
    right_antenna = object_model.get_part("right_antenna")
    left_fold = object_model.get_articulation("left_antenna_fold")
    right_fold = object_model.get_articulation("right_antenna_fold")

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
        "left hinge axis follows case depth",
        left_fold.axis == (0.0, -1.0, 0.0),
        details=f"unexpected left hinge axis: {left_fold.axis}",
    )
    ctx.check(
        "right hinge axis follows case depth",
        right_fold.axis == (0.0, 1.0, 0.0),
        details=f"unexpected right hinge axis: {right_fold.axis}",
    )

    with ctx.pose({left_fold: 0.0, right_fold: 0.0}):
        ctx.expect_contact(
            body,
            left_antenna,
            elem_a="left_front_knuckle",
            elem_b="pivot_barrel",
            name="left pivot barrel contacts front knuckle upright",
        )
        ctx.expect_contact(
            body,
            left_antenna,
            elem_a="left_rear_knuckle",
            elem_b="pivot_barrel",
            name="left pivot barrel contacts rear knuckle upright",
        )
        ctx.expect_contact(
            body,
            right_antenna,
            elem_a="right_front_knuckle",
            elem_b="pivot_barrel",
            name="right pivot barrel contacts front knuckle upright",
        )
        ctx.expect_contact(
            body,
            right_antenna,
            elem_a="right_rear_knuckle",
            elem_b="pivot_barrel",
            name="right pivot barrel contacts rear knuckle upright",
        )
        ctx.expect_gap(
            body,
            left_antenna,
            axis="x",
            positive_elem="top_shell",
            negative_elem="paddle",
            min_gap=0.010,
            name="left paddle clears case upright",
        )
        ctx.expect_gap(
            right_antenna,
            body,
            axis="x",
            positive_elem="paddle",
            negative_elem="top_shell",
            min_gap=0.010,
            name="right paddle clears case upright",
        )

    with ctx.pose({left_fold: 1.15, right_fold: 1.15}):
        ctx.expect_contact(
            body,
            left_antenna,
            elem_a="left_front_knuckle",
            elem_b="pivot_barrel",
            name="left pivot barrel stays clipped at front knuckle",
        )
        ctx.expect_contact(
            body,
            left_antenna,
            elem_a="left_rear_knuckle",
            elem_b="pivot_barrel",
            name="left pivot barrel stays clipped at rear knuckle",
        )
        ctx.expect_contact(
            body,
            right_antenna,
            elem_a="right_front_knuckle",
            elem_b="pivot_barrel",
            name="right pivot barrel stays clipped at front knuckle",
        )
        ctx.expect_contact(
            body,
            right_antenna,
            elem_a="right_rear_knuckle",
            elem_b="pivot_barrel",
            name="right pivot barrel stays clipped at rear knuckle",
        )
        ctx.expect_gap(
            body,
            left_antenna,
            axis="x",
            positive_elem="top_shell",
            negative_elem="paddle",
            min_gap=0.040,
            name="left paddle clears case folded out",
        )
        ctx.expect_gap(
            right_antenna,
            body,
            axis="x",
            positive_elem="paddle",
            negative_elem="top_shell",
            min_gap=0.040,
            name="right paddle clears case folded out",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
