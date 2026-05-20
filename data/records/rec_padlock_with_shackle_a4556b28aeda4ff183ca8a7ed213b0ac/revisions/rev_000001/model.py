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
    rounded_rect_profile,
    tube_from_spline_points,
)


def _rounded_cover_mesh(name: str, *, width: float, height: float, thickness: float, radius: float):
    return mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(width, height, radius, corner_segments=8),
            thickness,
            cap=True,
            center=True,
            closed=True,
        ).rotate_x(math.pi / 2.0),
        name,
    )


def _rounded_side_cap_mesh(name: str, *, width: float, height: float, thickness: float, radius: float):
    return mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(width, height, radius, corner_segments=8),
            thickness,
            cap=True,
            center=True,
            closed=True,
        )
        .rotate_x(-math.pi / 2.0)
        .rotate_z(math.pi / 2.0),
        name,
    )


def _shackle_arch_mesh(name: str, *, span: float, rise: float, rod_radius: float):
    arch = tube_from_spline_points(
        [
            (0.0, 0.0, 0.012),
            (0.0025, 0.0, rise * 0.82),
            (span * 0.34, 0.0, rise),
            (span * 0.66, 0.0, rise),
            (span - 0.0025, 0.0, rise * 0.82),
            (span, 0.0, 0.012),
        ],
        radius=rod_radius,
        samples_per_segment=16,
        radial_segments=20,
        cap_ends=True,
        up_hint=(0.0, 1.0, 0.0),
    )
    return mesh_from_geometry(arch, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="luggage_padlock")

    body_width = 0.032
    body_depth = 0.015
    body_height = 0.026
    cover_thickness = 0.0016
    side_thickness = 0.0018
    floor_thickness = 0.0030
    top_rail_depth = 0.0024
    top_rail_thickness = 0.0022
    inner_depth = body_depth - 2.0 * cover_thickness

    slot_center_z = 0.015
    slot_height = 0.0078
    slot_width_y = 0.0068

    shackle_span = 0.022
    shackle_rod_radius = 0.0022
    shackle_leg_length = 0.016
    shackle_leg_center_z = 0.004
    shackle_rise = 0.024

    button_cap_thickness = 0.0028
    button_cap_width = 0.0062
    button_cap_height = 0.0072
    button_stem_length = 0.0050
    button_stem_width = 0.0056
    button_stem_height = 0.0062
    button_clip_thickness = 0.0026
    button_clip_height = 0.0080
    button_travel = 0.0010

    body_brass = model.material("body_brass", rgba=(0.77, 0.63, 0.29, 1.0))
    body_shadow = model.material("body_shadow", rgba=(0.33, 0.27, 0.15, 1.0))
    shackle_steel = model.material("shackle_steel", rgba=(0.86, 0.88, 0.90, 1.0))
    shackle_dark = model.material("shackle_dark", rgba=(0.53, 0.55, 0.58, 1.0))
    button_red = model.material("button_red", rgba=(0.69, 0.13, 0.12, 1.0))
    keyway_steel = model.material("keyway_steel", rgba=(0.70, 0.72, 0.74, 1.0))

    cover_mesh = _rounded_cover_mesh(
        "padlock_cover",
        width=body_width,
        height=body_height,
        thickness=cover_thickness,
        radius=0.0038,
    )
    button_cap_mesh = _rounded_side_cap_mesh(
        "padlock_button_cap",
        width=button_cap_width,
        height=button_cap_height,
        thickness=button_cap_thickness,
        radius=0.0016,
    )
    shackle_arch_mesh = _shackle_arch_mesh(
        "padlock_shackle_arch",
        span=shackle_span,
        rise=shackle_rise,
        rod_radius=shackle_rod_radius,
    )

    body = model.part("body")
    body.visual(
        cover_mesh,
        origin=Origin(xyz=(0.0, -body_depth / 2.0 + cover_thickness / 2.0, body_height / 2.0)),
        material=body_brass,
        name="front_cover",
    )
    body.visual(
        cover_mesh,
        origin=Origin(xyz=(0.0, body_depth / 2.0 - cover_thickness / 2.0, body_height / 2.0)),
        material=body_brass,
        name="back_cover",
    )
    body.visual(
        Box((body_width - 2.0 * side_thickness, inner_depth, floor_thickness)),
        origin=Origin(xyz=(0.0, 0.0, floor_thickness / 2.0)),
        material=body_shadow,
        name="floor_plate",
    )
    body.visual(
        Box((side_thickness, inner_depth, body_height - floor_thickness)),
        origin=Origin(
            xyz=(
                -body_width / 2.0 + side_thickness / 2.0,
                0.0,
                floor_thickness + (body_height - floor_thickness) / 2.0,
            )
        ),
        material=body_brass,
        name="left_wall",
    )

    slot_bottom = slot_center_z - slot_height / 2.0
    slot_top = slot_center_z + slot_height / 2.0
    slot_front_back_depth = (inner_depth - slot_width_y) / 2.0

    body.visual(
        Box((side_thickness, inner_depth, slot_bottom - floor_thickness)),
        origin=Origin(
            xyz=(
                body_width / 2.0 - side_thickness / 2.0,
                0.0,
                floor_thickness + (slot_bottom - floor_thickness) / 2.0,
            )
        ),
        material=body_brass,
        name="right_wall_lower",
    )
    body.visual(
        Box((side_thickness, inner_depth, body_height - slot_top)),
        origin=Origin(
            xyz=(
                body_width / 2.0 - side_thickness / 2.0,
                0.0,
                slot_top + (body_height - slot_top) / 2.0,
            )
        ),
        material=body_brass,
        name="right_wall_upper",
    )
    body.visual(
        Box((side_thickness, slot_front_back_depth, slot_height)),
        origin=Origin(
            xyz=(
                body_width / 2.0 - side_thickness / 2.0,
                -slot_width_y / 2.0 - slot_front_back_depth / 2.0,
                slot_center_z,
            )
        ),
        material=body_brass,
        name="right_wall_front_jamb",
    )
    body.visual(
        Box((side_thickness, slot_front_back_depth, slot_height)),
        origin=Origin(
            xyz=(
                body_width / 2.0 - side_thickness / 2.0,
                slot_width_y / 2.0 + slot_front_back_depth / 2.0,
                slot_center_z,
            )
        ),
        material=body_brass,
        name="right_wall_rear_jamb",
    )

    body.visual(
        Box((body_width - 2.0 * side_thickness, top_rail_depth, top_rail_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                -inner_depth / 2.0 + top_rail_depth / 2.0,
                body_height - top_rail_thickness / 2.0,
            )
        ),
        material=body_brass,
        name="top_front_rail",
    )
    body.visual(
        Box((body_width - 2.0 * side_thickness, top_rail_depth, top_rail_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                inner_depth / 2.0 - top_rail_depth / 2.0,
                body_height - top_rail_thickness / 2.0,
            )
        ),
        material=body_brass,
        name="top_back_rail",
    )

    stop_size_x = 0.006
    stop_height = 0.002
    stop_center_z = body_height - 0.005
    body.visual(
        Box((stop_size_x, inner_depth, stop_height)),
        origin=Origin(
            xyz=(-shackle_span / 2.0, 0.0, stop_center_z),
        ),
        material=body_shadow,
        name="left_stop",
    )
    body.visual(
        Box((stop_size_x, inner_depth, stop_height)),
        origin=Origin(
            xyz=(shackle_span / 2.0, 0.0, stop_center_z),
        ),
        material=body_shadow,
        name="right_stop",
    )

    body.visual(
        Box((0.010, 0.0012, 0.012)),
        origin=Origin(xyz=(0.0, -body_depth / 2.0 - 0.0006, 0.0105)),
        material=keyway_steel,
        name="escutcheon",
    )
    body.visual(
        Cylinder(radius=0.0043, length=0.0014),
        origin=Origin(
            xyz=(0.0, -body_depth / 2.0 - 0.0007, 0.0105),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=keyway_steel,
        name="key_cylinder",
    )
    body.visual(
        Box((0.0013, 0.0016, 0.0046)),
        origin=Origin(xyz=(0.0, -body_depth / 2.0 - 0.0008, 0.0088)),
        material=body_shadow,
        name="key_slot_blade",
    )
    body.visual(
        Box((0.0038, 0.0016, 0.0012)),
        origin=Origin(xyz=(0.00125, -body_depth / 2.0 - 0.0008, 0.0071)),
        material=body_shadow,
        name="key_slot_ward",
    )
    body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_height)),
        mass=0.13,
        origin=Origin(xyz=(0.0, 0.0, body_height / 2.0)),
    )

    shackle = model.part("shackle")
    shackle.visual(
        Cylinder(radius=shackle_rod_radius, length=shackle_leg_length),
        origin=Origin(xyz=(0.0, 0.0, shackle_leg_center_z)),
        material=shackle_steel,
        name="captive_leg",
    )
    shackle.visual(
        Cylinder(radius=shackle_rod_radius, length=shackle_leg_length),
        origin=Origin(xyz=(shackle_span, 0.0, shackle_leg_center_z)),
        material=shackle_steel,
        name="free_leg",
    )
    shackle.visual(
        shackle_arch_mesh,
        material=shackle_steel,
        name="arch",
    )
    shackle.visual(
        Cylinder(radius=shackle_rod_radius * 1.05, length=0.0038),
        origin=Origin(xyz=(shackle_span, 0.0, -0.0021)),
        material=shackle_dark,
        name="free_tip",
    )
    shackle.inertial = Inertial.from_geometry(
        Box((shackle_span + 0.006, 0.008, shackle_rise + 0.006)),
        mass=0.035,
        origin=Origin(xyz=(shackle_span / 2.0, 0.0, 0.013)),
    )

    release_button = model.part("release_button")
    release_button.visual(
        button_cap_mesh,
        material=button_red,
        name="button_cap",
    )
    release_button.visual(
        Box((button_stem_length, button_stem_width, button_stem_height)),
        origin=Origin(xyz=(-(button_cap_thickness + button_stem_length) / 2.0, 0.0, 0.0)),
        material=button_red,
        name="button_stem",
    )
    release_button.visual(
        Box((button_clip_thickness, inner_depth, button_clip_height)),
        origin=Origin(
            xyz=(
                -button_cap_thickness / 2.0 - button_stem_length - button_clip_thickness / 2.0,
                0.0,
                0.0,
            )
        ),
        material=keyway_steel,
        name="clip_plate",
    )
    release_button.inertial = Inertial.from_geometry(
        Box((0.012, inner_depth, button_clip_height)),
        mass=0.008,
        origin=Origin(xyz=(-0.004, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_shackle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=shackle,
        origin=Origin(xyz=(-shackle_span / 2.0, 0.0, body_height)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,
            upper=1.25,
        ),
    )
    model.articulation(
        "body_to_release_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=release_button,
        origin=Origin(
            xyz=(
                body_width / 2.0 + button_cap_thickness / 2.0,
                0.0,
                slot_center_z,
            )
        ),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.04,
            lower=0.0,
            upper=button_travel,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    shackle = object_model.get_part("shackle")
    release_button = object_model.get_part("release_button")
    shackle_hinge = object_model.get_articulation("body_to_shackle")
    button_slide = object_model.get_articulation("body_to_release_button")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(shackle, body, elem_a="captive_leg", elem_b="left_stop")
    ctx.expect_contact(shackle, body, elem_a="free_leg", elem_b="right_stop")
    ctx.expect_contact(release_button, body, elem_a="clip_plate", elem_b="front_cover")
    ctx.expect_contact(release_button, body, elem_a="clip_plate", elem_b="back_cover")
    ctx.expect_within(release_button, body, axes="yz", margin=0.0)

    button_rest = ctx.part_world_position(release_button)
    assert button_rest is not None
    with ctx.pose({button_slide: 0.0009}):
        button_pressed = ctx.part_world_position(release_button)
        assert button_pressed is not None
        assert button_pressed[0] < button_rest[0] - 0.0007
        ctx.expect_within(release_button, body, axes="yz", margin=0.0)
        ctx.expect_contact(release_button, body, elem_a="clip_plate", elem_b="front_cover")

    with ctx.pose({shackle_hinge: 1.10}):
        ctx.expect_gap(shackle, body, axis="z", positive_elem="free_leg", min_gap=0.004)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
