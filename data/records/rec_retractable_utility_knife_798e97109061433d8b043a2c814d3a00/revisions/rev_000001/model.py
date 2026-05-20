from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _build_blade_mesh():
    blade_profile = [
        (-0.018, 0.0015),
        (0.020, 0.0015),
        (0.047, 0.0120),
        (0.020, 0.0140),
        (-0.010, 0.0140),
        (-0.018, 0.0070),
    ]
    blade_geom = ExtrudeGeometry(blade_profile, 0.0010, cap=True, center=True)
    blade_geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(blade_geom, "utility_knife_blade")


def _build_slider_pad_mesh():
    slider_geom = ExtrudeGeometry(
        rounded_rect_profile(0.018, 0.0080, 0.0022),
        0.005,
        cap=True,
        center=True,
    )
    return mesh_from_geometry(slider_geom, "utility_knife_slider_pad")


def _build_side_shell_mesh():
    side_profile = [
        (-0.082, 0.003),
        (-0.078, 0.024),
        (-0.060, 0.0285),
        (-0.018, 0.030),
        (0.040, 0.0295),
        (0.062, 0.028),
        (0.078, 0.024),
        (0.086, 0.019),
        (0.083, 0.009),
        (0.070, 0.003),
    ]
    side_geom = ExtrudeGeometry(side_profile, 0.003, cap=True, center=True)
    side_geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(side_geom, "utility_knife_side_shell")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="snap_lock_utility_knife")

    body_yellow = model.material("body_yellow", rgba=(0.88, 0.76, 0.13, 1.0))
    charcoal = model.material("charcoal", rgba=(0.15, 0.16, 0.18, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.09, 0.09, 0.10, 1.0))
    steel = model.material("steel", rgba=(0.80, 0.82, 0.84, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.42, 0.45, 0.48, 1.0))

    blade_mesh = _build_blade_mesh()
    slider_pad_mesh = _build_slider_pad_mesh()
    side_shell_mesh = _build_side_shell_mesh()

    handle = model.part("handle_body")
    handle.visual(
        Box((0.150, 0.018, 0.005)),
        origin=Origin(xyz=(-0.002, 0.0, 0.0025)),
        material=body_yellow,
        name="shell_floor",
    )
    handle.visual(
        side_shell_mesh,
        origin=Origin(xyz=(0.000, -0.0075, 0.0)),
        material=body_yellow,
        name="shell_left",
    )
    handle.visual(
        side_shell_mesh,
        origin=Origin(xyz=(0.000, 0.0075, 0.0)),
        material=body_yellow,
        name="shell_right",
    )
    handle.visual(
        Box((0.034, 0.018, 0.006)),
        origin=Origin(xyz=(-0.068, 0.0, 0.027)),
        material=charcoal,
        name="rear_cap",
    )
    handle.visual(
        Box((0.114, 0.005, 0.004)),
        origin=Origin(xyz=(0.000, -0.0065, 0.028)),
        material=dark_steel,
        name="spine_left",
    )
    handle.visual(
        Box((0.114, 0.005, 0.004)),
        origin=Origin(xyz=(0.000, 0.0065, 0.028)),
        material=dark_steel,
        name="spine_right",
    )
    handle.visual(
        Box((0.026, 0.018, 0.008)),
        origin=Origin(xyz=(0.077, 0.0, 0.026)),
        material=dark_steel,
        name="front_nose_upper",
    )
    handle.visual(
        Box((0.030, 0.018, 0.008)),
        origin=Origin(xyz=(0.075, 0.0, 0.004)),
        material=dark_steel,
        name="front_nose_lower",
    )
    handle.visual(
        Box((0.082, 0.0018, 0.013)),
        origin=Origin(xyz=(-0.004, -0.0091, 0.014)),
        material=rubber_black,
        name="left_grip_pad",
    )
    handle.visual(
        Box((0.082, 0.0018, 0.013)),
        origin=Origin(xyz=(-0.004, 0.0091, 0.014)),
        material=rubber_black,
        name="right_grip_pad",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.176, 0.020, 0.032)),
        mass=0.34,
        origin=Origin(xyz=(0.000, 0.0, 0.016)),
    )

    blade_carriage = model.part("blade_carriage")
    blade_carriage.visual(
        Box((0.060, 0.008, 0.010)),
        origin=Origin(xyz=(0.012, 0.0, -0.010)),
        material=charcoal,
        name="carriage_body",
    )
    blade_carriage.visual(
        Box((0.022, 0.008, 0.012)),
        origin=Origin(xyz=(0.032, 0.0, -0.009)),
        material=dark_steel,
        name="blade_clamp",
    )
    blade_carriage.visual(
        Box((0.014, 0.004, 0.016)),
        origin=Origin(xyz=(0.000, 0.0, -0.008)),
        material=dark_steel,
        name="carriage_stem",
    )
    blade_carriage.visual(
        blade_mesh,
        origin=Origin(xyz=(0.059, 0.0, -0.0185)),
        material=steel,
        name="blade",
    )
    blade_carriage.inertial = Inertial.from_geometry(
        Box((0.090, 0.010, 0.028)),
        mass=0.06,
        origin=Origin(xyz=(0.020, 0.0, -0.007)),
    )

    thumb_slider = model.part("thumb_slider")
    thumb_slider.visual(
        slider_pad_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0025)),
        material=charcoal,
        name="slider_pad",
    )
    thumb_slider.visual(
        Box((0.010, 0.0016, 0.001)),
        origin=Origin(xyz=(-0.004, 0.0, 0.0055)),
        material=rubber_black,
        name="slider_rib_rear",
    )
    thumb_slider.visual(
        Box((0.010, 0.0016, 0.001)),
        origin=Origin(xyz=(0.004, 0.0, 0.0055)),
        material=rubber_black,
        name="slider_rib_front",
    )
    thumb_slider.inertial = Inertial.from_geometry(
        Box((0.018, 0.008, 0.006)),
        mass=0.01,
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
    )

    model.articulation(
        "handle_to_blade_carriage",
        ArticulationType.PRISMATIC,
        parent=handle,
        child=blade_carriage,
        origin=Origin(xyz=(-0.020, 0.0, 0.026)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.12,
            lower=0.0,
            upper=0.040,
        ),
    )
    model.articulation(
        "carriage_to_thumb_slider",
        ArticulationType.FIXED,
        parent=blade_carriage,
        child=thumb_slider,
        origin=Origin(),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    handle = object_model.get_part("handle_body")
    blade_carriage = object_model.get_part("blade_carriage")
    thumb_slider = object_model.get_part("thumb_slider")
    slide_joint = object_model.get_articulation("handle_to_blade_carriage")

    open_q = 0.0
    if slide_joint.motion_limits is not None and slide_joint.motion_limits.upper is not None:
        open_q = slide_joint.motion_limits.upper

    with ctx.pose({slide_joint: 0.0}):
        closed_handle_aabb = ctx.part_world_aabb(handle)
        closed_blade_aabb = ctx.part_element_world_aabb(blade_carriage, elem="blade")
        closed_slider_aabb = ctx.part_element_world_aabb(thumb_slider, elem="slider_pad")
        closed_carriage_pos = ctx.part_world_position(blade_carriage)
        closed_slider_pos = ctx.part_world_position(thumb_slider)
        rear_cap_aabb = ctx.part_element_world_aabb(handle, elem="rear_cap")
        front_nose_aabb = ctx.part_element_world_aabb(handle, elem="front_nose_upper")

    with ctx.pose({slide_joint: open_q}):
        ctx.expect_gap(
            handle,
            blade_carriage,
            axis="z",
            positive_elem="front_nose_upper",
            negative_elem="blade",
            min_gap=0.0005,
            name="blade clears the upper nose guide when extended",
        )
        ctx.expect_gap(
            blade_carriage,
            handle,
            axis="z",
            positive_elem="blade",
            negative_elem="front_nose_lower",
            min_gap=0.0005,
            name="blade clears the lower nose guide when extended",
        )
        open_handle_aabb = ctx.part_world_aabb(handle)
        open_blade_aabb = ctx.part_element_world_aabb(blade_carriage, elem="blade")
        open_slider_aabb = ctx.part_element_world_aabb(thumb_slider, elem="slider_pad")
        open_carriage_pos = ctx.part_world_position(blade_carriage)
        open_slider_pos = ctx.part_world_position(thumb_slider)

    ctx.check(
        "blade carriage extends toward the nose",
        (
            closed_carriage_pos is not None
            and open_carriage_pos is not None
            and open_carriage_pos[0] > closed_carriage_pos[0] + 0.030
        ),
        details=f"closed={closed_carriage_pos}, open={open_carriage_pos}",
    )
    ctx.check(
        "thumb slider translates with the carriage",
        (
            closed_carriage_pos is not None
            and open_carriage_pos is not None
            and closed_slider_pos is not None
            and open_slider_pos is not None
            and abs(
                (open_slider_pos[0] - closed_slider_pos[0])
                - (open_carriage_pos[0] - closed_carriage_pos[0])
            )
            <= 0.0005
        ),
        details=(
            f"closed carriage={closed_carriage_pos}, open carriage={open_carriage_pos}, "
            f"closed slider={closed_slider_pos}, open slider={open_slider_pos}"
        ),
    )
    ctx.check(
        "closed blade stays nearly flush with the front opening",
        (
            closed_handle_aabb is not None
            and closed_blade_aabb is not None
            and closed_blade_aabb[1][0] <= closed_handle_aabb[1][0] + 0.002
        ),
        details=f"handle={closed_handle_aabb}, blade={closed_blade_aabb}",
    )
    ctx.check(
        "open blade projects clearly beyond the handle",
        (
            open_handle_aabb is not None
            and open_blade_aabb is not None
            and open_blade_aabb[1][0] >= open_handle_aabb[1][0] + 0.020
        ),
        details=f"handle={open_handle_aabb}, blade={open_blade_aabb}",
    )
    ctx.check(
        "thumb slider remains between the slot stops",
        (
            rear_cap_aabb is not None
            and front_nose_aabb is not None
            and closed_slider_aabb is not None
            and open_slider_aabb is not None
            and closed_slider_aabb[0][0] >= rear_cap_aabb[1][0] - 0.001
            and closed_slider_aabb[1][0] <= front_nose_aabb[0][0] + 0.001
            and open_slider_aabb[0][0] >= rear_cap_aabb[1][0] - 0.001
            and open_slider_aabb[1][0] <= front_nose_aabb[0][0] + 0.001
        ),
        details=(
            f"rear_cap={rear_cap_aabb}, front_nose={front_nose_aabb}, "
            f"closed_slider={closed_slider_aabb}, open_slider={open_slider_aabb}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
