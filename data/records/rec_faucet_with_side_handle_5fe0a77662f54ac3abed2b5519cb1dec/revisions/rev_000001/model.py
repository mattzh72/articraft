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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

def _add_handle_visuals(part, material, accent_material, *, side_sign: float) -> None:
    part.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=material,
        name="hub_cap",
    )
    part.visual(
        Cylinder(radius=0.008, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=material,
        name="stem",
    )
    part.visual(
        Box((0.026, 0.016, 0.014)),
        origin=Origin(xyz=(side_sign * 0.012, -0.010, 0.030)),
        material=material,
        name="neck",
    )
    part.visual(
        Box((0.056, 0.034, 0.008)),
        origin=Origin(xyz=(side_sign * 0.030, -0.024, 0.040)),
        material=material,
        name="paddle",
    )
    part.visual(
        Box((0.030, 0.016, 0.002)),
        origin=Origin(xyz=(side_sign * 0.030, -0.024, 0.045)),
        material=accent_material,
        name="paddle_inset",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="deck_mount_bathtub_faucet")

    chrome = model.material("chrome", rgba=(0.82, 0.84, 0.87, 1.0))
    satin_shadow = model.material("satin_shadow", rgba=(0.40, 0.42, 0.46, 1.0))

    deck_base = model.part("deck_base")
    deck_base.visual(
        Box((0.200, 0.140, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=chrome,
        name="base_plate_center",
    )
    deck_base.visual(
        Cylinder(radius=0.070, length=0.010),
        origin=Origin(xyz=(-0.100, 0.0, 0.005)),
        material=chrome,
        name="base_plate_left_end",
    )
    deck_base.visual(
        Cylinder(radius=0.070, length=0.010),
        origin=Origin(xyz=(0.100, 0.0, 0.005)),
        material=chrome,
        name="base_plate_right_end",
    )

    deck_base.visual(
        Cylinder(radius=0.030, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=chrome,
        name="spout_flange",
    )
    deck_base.visual(
        Cylinder(radius=0.026, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=chrome,
        name="spout_collar",
    )

    for boss_name, x_pos in (("left_boss", -0.110), ("right_boss", 0.110)):
        deck_base.visual(
            Cylinder(radius=0.028, length=0.006),
            origin=Origin(xyz=(x_pos, 0.0, 0.013)),
            material=chrome,
            name=f"{boss_name}_flange",
        )
        deck_base.visual(
            Cylinder(radius=0.024, length=0.016),
            origin=Origin(xyz=(x_pos, 0.0, 0.018)),
            material=chrome,
            name=boss_name,
        )

    deck_base.visual(
        Cylinder(radius=0.020, length=0.118),
        origin=Origin(xyz=(0.0, 0.004, 0.087)),
        material=chrome,
        name="spout_column",
    )
    deck_base.visual(
        Box((0.040, 0.034, 0.024)),
        origin=Origin(xyz=(0.0, 0.012, 0.157)),
        material=chrome,
        name="spout_shoulder",
    )
    deck_base.visual(
        Cylinder(radius=0.024, length=0.052),
        origin=Origin(xyz=(0.0, 0.020, 0.179), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="spout_body",
    )
    deck_base.visual(
        Box((0.034, 0.004, 0.008)),
        origin=Origin(xyz=(0.0, 0.046, 0.180)),
        material=satin_shadow,
        name="outlet_shadow",
    )
    deck_base.inertial = Inertial.from_geometry(
        Box((0.340, 0.140, 0.220)),
        mass=6.5,
        origin=Origin(xyz=(0.0, 0.015, 0.110)),
    )

    left_handle = model.part("left_handle")
    _add_handle_visuals(left_handle, chrome, satin_shadow, side_sign=-1.0)
    left_handle.inertial = Inertial.from_geometry(
        Box((0.080, 0.060, 0.050)),
        mass=0.35,
        origin=Origin(xyz=(-0.010, -0.014, 0.025)),
    )

    right_handle = model.part("right_handle")
    _add_handle_visuals(right_handle, chrome, satin_shadow, side_sign=1.0)
    right_handle.inertial = Inertial.from_geometry(
        Box((0.080, 0.060, 0.050)),
        mass=0.35,
        origin=Origin(xyz=(0.010, -0.014, 0.025)),
    )

    model.articulation(
        "left_handle_turn",
        ArticulationType.REVOLUTE,
        parent=deck_base,
        child=left_handle,
        origin=Origin(xyz=(-0.110, 0.0, 0.026)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=-0.85,
            upper=0.85,
        ),
    )
    model.articulation(
        "right_handle_turn",
        ArticulationType.REVOLUTE,
        parent=deck_base,
        child=right_handle,
        origin=Origin(xyz=(0.110, 0.0, 0.026)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=-0.85,
            upper=0.85,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck_base = object_model.get_part("deck_base")
    left_handle = object_model.get_part("left_handle")
    right_handle = object_model.get_part("right_handle")
    left_joint = object_model.get_articulation("left_handle_turn")
    right_joint = object_model.get_articulation("right_handle_turn")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(left_handle, deck_base)
    ctx.expect_contact(right_handle, deck_base)
    ctx.expect_overlap(left_handle, deck_base, axes="xy", min_overlap=0.032)
    ctx.expect_overlap(right_handle, deck_base, axes="xy", min_overlap=0.032)
    ctx.expect_origin_distance(
        left_handle,
        right_handle,
        axes="x",
        min_dist=0.210,
        max_dist=0.230,
    )

    ctx.check(
        "left_handle_axis_is_vertical",
        left_joint.axis == (0.0, 0.0, 1.0),
        details=f"left handle axis was {left_joint.axis!r}",
    )
    ctx.check(
        "right_handle_axis_is_vertical",
        right_joint.axis == (0.0, 0.0, 1.0),
        details=f"right handle axis was {right_joint.axis!r}",
    )

    left_limits = left_joint.motion_limits
    right_limits = right_joint.motion_limits
    assert left_limits is not None
    assert right_limits is not None
    ctx.check(
        "handle_motion_limits_are_quarter_turn_like",
        (
            left_limits.lower is not None
            and left_limits.upper is not None
            and right_limits.lower is not None
            and right_limits.upper is not None
            and math.isclose(left_limits.lower, -0.85, abs_tol=1e-9)
            and math.isclose(left_limits.upper, 0.85, abs_tol=1e-9)
            and math.isclose(right_limits.lower, -0.85, abs_tol=1e-9)
            and math.isclose(right_limits.upper, 0.85, abs_tol=1e-9)
        ),
        details="handle limits should be symmetric quarter-turn style travel",
    )

    left_rest = ctx.part_element_world_aabb(left_handle, elem="paddle")
    right_rest = ctx.part_element_world_aabb(right_handle, elem="paddle")
    assert left_rest is not None
    assert right_rest is not None
    left_rest_x = 0.5 * (left_rest[0][0] + left_rest[1][0])
    right_rest_x = 0.5 * (right_rest[0][0] + right_rest[1][0])

    with ctx.pose({left_joint: 0.60, right_joint: -0.60}):
        ctx.expect_contact(left_handle, deck_base)
        ctx.expect_contact(right_handle, deck_base)
        left_open = ctx.part_element_world_aabb(left_handle, elem="paddle")
        right_open = ctx.part_element_world_aabb(right_handle, elem="paddle")
        assert left_open is not None
        assert right_open is not None
        left_open_x = 0.5 * (left_open[0][0] + left_open[1][0])
        right_open_x = 0.5 * (right_open[0][0] + right_open[1][0])
        assert abs(left_open_x) < abs(left_rest_x) - 0.008
        assert abs(right_open_x) < abs(right_rest_x) - 0.008

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
