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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wheelie_bin")

    body_color = model.material("body_plastic", rgba=(0.16, 0.29, 0.18, 1.0))
    lid_color = model.material("lid_plastic", rgba=(0.15, 0.27, 0.16, 1.0))
    metal = model.material("axle_metal", rgba=(0.45, 0.47, 0.50, 1.0))
    tire_color = model.material("tire_rubber", rgba=(0.08, 0.08, 0.08, 1.0))

    body_width = 0.43
    body_depth = 0.35
    wall = 0.012
    body_height = 0.50
    floor_thickness = 0.020
    corner_radius = 0.030

    outer_profile = rounded_rect_profile(body_width, body_depth, corner_radius)
    inner_profile = rounded_rect_profile(
        body_width - 2.0 * wall,
        body_depth - 2.0 * wall,
        max(corner_radius - wall * 0.6, 0.012),
    )
    shell_geom = ExtrudeWithHolesGeometry(
        outer_profile,
        [inner_profile],
        body_height,
        cap=False,
        center=False,
    )
    shell_mesh = mesh_from_geometry(shell_geom, "bin_body_shell")

    body = model.part("body")
    body.visual(shell_mesh, material=body_color, name="body_shell")
    body.visual(
        Box((body_width - 0.035, body_depth - 0.035, floor_thickness)),
        origin=Origin(xyz=(0.0, 0.0, floor_thickness * 0.5)),
        material=body_color,
        name="body_floor",
    )
    body.visual(
        Box((0.11, 0.10, 0.045)),
        origin=Origin(xyz=(0.0, -body_depth * 0.5 + 0.020, body_height - 0.050)),
        material=body_color,
        name="rear_handle_block",
    )
    body.visual(
        Box((0.012, 0.026, 0.038)),
        origin=Origin(
            xyz=(-(body_width * 0.5 + 0.014), -body_depth * 0.5 - 0.012, body_height - 0.014)
        ),
        material=body_color,
        name="left_hinge_tab",
    )
    body.visual(
        Box((0.020, 0.024, 0.022)),
        origin=Origin(
            xyz=(-(body_width * 0.5 + 0.005), -body_depth * 0.5 - 0.010, body_height - 0.020)
        ),
        material=body_color,
        name="left_hinge_bracket",
    )
    body.visual(
        Box((0.012, 0.026, 0.038)),
        origin=Origin(
            xyz=((body_width * 0.5 + 0.014), -body_depth * 0.5 - 0.012, body_height - 0.014)
        ),
        material=body_color,
        name="right_hinge_tab",
    )
    body.visual(
        Box((0.020, 0.024, 0.022)),
        origin=Origin(
            xyz=((body_width * 0.5 + 0.005), -body_depth * 0.5 - 0.010, body_height - 0.020)
        ),
        material=body_color,
        name="right_hinge_bracket",
    )
    body.visual(
        Box((body_width + 0.030, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, -body_depth * 0.5 - 0.009, body_height - 0.020)),
        material=body_color,
        name="rear_hinge_rib",
    )
    body.visual(
        Cylinder(radius=0.010, length=body_width + 0.012),
        origin=Origin(xyz=(0.0, -body_depth * 0.5 - 0.008, 0.105), rpy=(0.0, pi * 0.5, 0.0)),
        material=metal,
        name="rear_axle_tube",
    )
    body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_height + floor_thickness)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, (body_height + floor_thickness) * 0.5)),
    )

    lid_overhang = 0.006
    lid_depth = body_depth + 0.012
    lid_thickness = 0.018
    lid_rear_setback = 0.008
    lid_panel_center_y = lid_rear_setback + lid_depth * 0.5

    lid = model.part("lid")
    lid.visual(
        Box((body_width + lid_overhang, lid_depth, lid_thickness)),
        origin=Origin(xyz=(0.0, lid_panel_center_y, -0.004)),
        material=lid_color,
        name="lid_panel",
    )
    lid.visual(
        Box((body_width + 0.012, 0.020, 0.022)),
        origin=Origin(xyz=(0.0, lid_rear_setback + lid_depth - 0.010, -0.002)),
        material=lid_color,
        name="lid_front_lip",
    )
    lid.visual(
        Cylinder(radius=0.013, length=body_width - 0.060),
        origin=Origin(rpy=(0.0, pi * 0.5, 0.0)),
        material=lid_color,
        name="lid_hinge_barrel",
    )
    lid.inertial = Inertial.from_geometry(
        Box((body_width + lid_overhang, lid_depth, lid_thickness)),
        mass=1.2,
        origin=Origin(xyz=(0.0, lid_panel_center_y, -0.004)),
    )

    wheel_radius = 0.102
    wheel_width = 0.036
    wheel_center_x = body_width * 0.5 + wheel_width * 0.5 + 0.006
    wheel_center_y = -body_depth * 0.5 - 0.002
    wheel_center_z = wheel_radius

    left_wheel = model.part("left_wheel")
    left_wheel.visual(
        Cylinder(radius=wheel_radius, length=wheel_width),
        origin=Origin(rpy=(0.0, pi * 0.5, 0.0)),
        material=tire_color,
        name="left_tire",
    )
    left_wheel.visual(
        Cylinder(radius=0.050, length=wheel_width + 0.004),
        origin=Origin(rpy=(0.0, pi * 0.5, 0.0)),
        material=metal,
        name="left_hub",
    )
    left_wheel.inertial = Inertial.from_geometry(
        Box((wheel_width, wheel_radius * 2.0, wheel_radius * 2.0)),
        mass=1.4,
    )

    right_wheel = model.part("right_wheel")
    right_wheel.visual(
        Cylinder(radius=wheel_radius, length=wheel_width),
        origin=Origin(rpy=(0.0, pi * 0.5, 0.0)),
        material=tire_color,
        name="right_tire",
    )
    right_wheel.visual(
        Cylinder(radius=0.050, length=wheel_width + 0.004),
        origin=Origin(rpy=(0.0, pi * 0.5, 0.0)),
        material=metal,
        name="right_hub",
    )
    right_wheel.inertial = Inertial.from_geometry(
        Box((wheel_width, wheel_radius * 2.0, wheel_radius * 2.0)),
        mass=1.4,
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, -body_depth * 0.5 - 0.013, body_height + 0.013)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.0,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "body_to_left_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=left_wheel,
        origin=Origin(xyz=(-wheel_center_x, wheel_center_y, wheel_center_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=18.0,
        ),
    )
    model.articulation(
        "body_to_right_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=right_wheel,
        origin=Origin(xyz=(wheel_center_x, wheel_center_y, wheel_center_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=18.0,
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
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    lid_hinge = object_model.get_articulation("body_to_lid")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    left_spin = object_model.get_articulation("body_to_left_wheel")
    right_spin = object_model.get_articulation("body_to_right_wheel")

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="body_shell",
            max_gap=0.015,
            max_penetration=0.0,
            name="closed lid sits neatly on the body rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.28,
            name="closed lid covers the opening footprint",
        )

    closed_front = None
    open_front = None
    with ctx.pose({lid_hinge: 0.0}):
        closed_front = ctx.part_element_world_aabb(lid, elem="lid_front_lip")
    with ctx.pose({lid_hinge: 1.1}):
        open_front = ctx.part_element_world_aabb(lid, elem="lid_front_lip")

    ctx.check(
        "lid front edge lifts when opened",
        closed_front is not None
        and open_front is not None
        and open_front[1][2] > closed_front[1][2] + 0.12,
        details=f"closed_front={closed_front}, open_front={open_front}",
    )

    ctx.expect_origin_distance(
        left_wheel,
        right_wheel,
        axes="x",
        min_dist=0.46,
        max_dist=0.52,
        name="rear wheels sit wide on the axle",
    )

    left_rest = None
    right_rest = None
    left_spun = None
    right_spun = None
    with ctx.pose({left_spin: 0.0, right_spin: 0.0}):
        left_rest = ctx.part_world_position(left_wheel)
        right_rest = ctx.part_world_position(right_wheel)
    with ctx.pose({left_spin: pi * 0.5, right_spin: pi * 0.5}):
        left_spun = ctx.part_world_position(left_wheel)
        right_spun = ctx.part_world_position(right_wheel)

    ctx.check(
        "wheel joints spin around fixed axle centers",
        left_rest is not None
        and right_rest is not None
        and left_spun is not None
        and right_spun is not None
        and max(abs(a - b) for a, b in zip(left_rest, left_spun)) < 1e-9
        and max(abs(a - b) for a, b in zip(right_rest, right_spun)) < 1e-9,
        details=(
            f"left_rest={left_rest}, left_spun={left_spun}, "
            f"right_rest={right_rest}, right_spun={right_spun}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
