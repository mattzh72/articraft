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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="home_automation_hub_controller")

    body_white = model.material("body_white", rgba=(0.93, 0.94, 0.95, 1.0))
    ring_graphite = model.material("ring_graphite", rgba=(0.26, 0.28, 0.31, 1.0))
    touch_glass = model.material("touch_glass", rgba=(0.64, 0.73, 0.78, 0.80))
    accent_blue = model.material("accent_blue", rgba=(0.28, 0.66, 0.90, 1.0))
    underside_gray = model.material("underside_gray", rgba=(0.68, 0.70, 0.73, 1.0))
    guide_dark = model.material("guide_dark", rgba=(0.35, 0.37, 0.39, 1.0))

    body = model.part("hub_body")

    lower_shell_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            superellipse_profile(0.102, 0.102, exponent=2.0, segments=72),
            [rounded_rect_profile(0.052, 0.036, radius=0.006, corner_segments=8)],
            0.010,
            cap=True,
            center=True,
            closed=True,
        ),
        "hub_lower_shell",
    )
    body.visual(
        lower_shell_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=body_white,
        name="lower_shell",
    )
    body.visual(
        Cylinder(radius=0.040, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=body_white,
        name="central_cap",
    )
    body.visual(
        Cylinder(radius=0.028, length=0.0016),
        origin=Origin(xyz=(0.0, 0.0, 0.0188)),
        material=touch_glass,
        name="touch_surface",
    )
    body.visual(
        Box((0.052, 0.032, 0.001)),
        origin=Origin(xyz=(0.0, 0.0, 0.0075)),
        material=guide_dark,
        name="battery_track_span",
    )
    body.visual(
        Box((0.048, 0.002, 0.0024)),
        origin=Origin(xyz=(0.0, 0.016, 0.0052)),
        material=guide_dark,
        name="battery_guide_left",
    )
    body.visual(
        Box((0.048, 0.002, 0.0024)),
        origin=Origin(xyz=(0.0, -0.016, 0.0052)),
        material=guide_dark,
        name="battery_guide_right",
    )
    body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.051, length=0.020),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    selection_ring = model.part("selection_ring")
    ring_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            superellipse_profile(0.118, 0.118, exponent=2.0, segments=88),
            [superellipse_profile(0.086, 0.086, exponent=2.0, segments=72)],
            0.0095,
            cap=True,
            center=True,
            closed=True,
        ),
        "selection_ring_shell",
    )
    ring_ridge_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            superellipse_profile(0.120, 0.120, exponent=2.0, segments=88),
            [superellipse_profile(0.094, 0.094, exponent=2.0, segments=72)],
            0.0026,
            cap=True,
            center=True,
            closed=True,
        ),
        "selection_ring_ridge",
    )
    selection_ring.visual(
        ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.01475)),
        material=ring_graphite,
        name="ring_shell",
    )
    selection_ring.visual(
        ring_ridge_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0182)),
        material=guide_dark,
        name="ring_ridge",
    )
    selection_ring.visual(
        Box((0.008, 0.0035, 0.0012)),
        origin=Origin(xyz=(0.048, 0.0, 0.0193)),
        material=accent_blue,
        name="selection_tick",
    )
    selection_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.060, length=0.011),
        mass=0.11,
        origin=Origin(xyz=(0.0, 0.0, 0.01475)),
    )

    battery_door = model.part("battery_door")
    battery_door.visual(
        Box((0.048, 0.030, 0.0028)),
        origin=Origin(xyz=(0.0, 0.0, 0.0014)),
        material=underside_gray,
        name="door_panel",
    )
    battery_door.visual(
        Box((0.010, 0.014, 0.0012)),
        origin=Origin(xyz=(0.017, 0.0, 0.0034)),
        material=guide_dark,
        name="door_thumb_ridge",
    )
    battery_door.visual(
        Box((0.044, 0.0015, 0.0016)),
        origin=Origin(xyz=(0.0, 0.0145, 0.0036)),
        material=guide_dark,
        name="runner_left",
    )
    battery_door.visual(
        Box((0.044, 0.0015, 0.0016)),
        origin=Origin(xyz=(0.0, -0.0145, 0.0036)),
        material=guide_dark,
        name="runner_right",
    )
    battery_door.inertial = Inertial.from_geometry(
        Box((0.048, 0.032, 0.0052)),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.0, 0.0026)),
    )

    model.articulation(
        "body_to_selection_ring",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selection_ring,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=8.0,
        ),
    )
    model.articulation(
        "body_to_battery_door",
        ArticulationType.PRISMATIC,
        parent=body,
        child=battery_door,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.06,
            lower=0.0,
            upper=0.020,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    body = object_model.get_part("hub_body")
    ring = object_model.get_part("selection_ring")
    door = object_model.get_part("battery_door")
    ring_joint = object_model.get_articulation("body_to_selection_ring")
    door_joint = object_model.get_articulation("body_to_battery_door")

    ctx.expect_origin_distance(
        ring,
        body,
        axes="xy",
        min_dist=0.0,
        max_dist=1e-6,
        name="selection ring stays concentric with the hub body",
    )

    with ctx.pose({door_joint: 0.0}):
        ctx.expect_within(
            door,
            body,
            axes="y",
            inner_elem="door_panel",
            outer_elem="battery_track_span",
            margin=0.0015,
            name="battery door stays laterally captured in the track at rest",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="x",
            elem_a="door_panel",
            elem_b="battery_track_span",
            min_overlap=0.045,
            name="battery door fully covers the bay when closed",
        )
        ctx.expect_gap(
            body,
            door,
            axis="z",
            positive_elem="battery_track_span",
            negative_elem="door_panel",
            min_gap=0.003,
            max_gap=0.006,
            name="battery door sits below the cavity roof",
        )

    rest_tick_aabb = None
    quarter_turn_tick_aabb = None
    rest_door_position = None
    open_door_position = None

    with ctx.pose({ring_joint: 0.0}):
        rest_tick_aabb = ctx.part_element_world_aabb(ring, elem="selection_tick")
        rest_door_position = ctx.part_world_position(door)

    with ctx.pose({ring_joint: math.pi / 2.0}):
        quarter_turn_tick_aabb = ctx.part_element_world_aabb(ring, elem="selection_tick")

    with ctx.pose({door_joint: 0.020}):
        open_door_position = ctx.part_world_position(door)
        ctx.expect_within(
            door,
            body,
            axes="y",
            inner_elem="door_panel",
            outer_elem="battery_track_span",
            margin=0.0015,
            name="battery door remains guided laterally when open",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="x",
            elem_a="door_panel",
            elem_b="battery_track_span",
            min_overlap=0.028,
            name="battery door keeps retained insertion at full travel",
        )

    def _aabb_center_x(aabb):
        return 0.5 * (aabb[0][0] + aabb[1][0]) if aabb is not None else None

    def _aabb_center_y(aabb):
        return 0.5 * (aabb[0][1] + aabb[1][1]) if aabb is not None else None

    rest_tick_x = _aabb_center_x(rest_tick_aabb)
    rest_tick_y = _aabb_center_y(rest_tick_aabb)
    quarter_tick_x = _aabb_center_x(quarter_turn_tick_aabb)
    quarter_tick_y = _aabb_center_y(quarter_turn_tick_aabb)

    ctx.check(
        "selection tick moves around the vertical axis",
        rest_tick_x is not None
        and rest_tick_y is not None
        and quarter_tick_x is not None
        and quarter_tick_y is not None
        and rest_tick_x > 0.040
        and abs(rest_tick_y) < 0.004
        and quarter_tick_y > 0.040
        and abs(quarter_tick_x) < 0.004,
        details=(
            f"rest_tick=({rest_tick_x}, {rest_tick_y}), "
            f"quarter_turn_tick=({quarter_tick_x}, {quarter_tick_y})"
        ),
    )
    ctx.check(
        "battery door slides outward along +X",
        rest_door_position is not None
        and open_door_position is not None
        and open_door_position[0] > rest_door_position[0] + 0.015,
        details=f"rest={rest_door_position}, open={open_door_position}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
