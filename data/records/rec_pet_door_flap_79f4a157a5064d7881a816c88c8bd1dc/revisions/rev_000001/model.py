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


def _ring_mesh(
    name: str,
    *,
    outer_width: float,
    outer_height: float,
    inner_width: float,
    inner_height: float,
    depth: float,
    outer_radius: float,
    inner_radius: float,
):
    geometry = ExtrudeWithHolesGeometry(
        rounded_rect_profile(outer_width, outer_height, outer_radius, corner_segments=8),
        [rounded_rect_profile(inner_width, inner_height, inner_radius, corner_segments=8)],
        height=depth,
        center=True,
    ).rotate_x(pi / 2.0)
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pet_flap")

    frame_plastic = model.material("frame_plastic", rgba=(0.93, 0.93, 0.91, 1.0))
    trim_plastic = model.material("trim_plastic", rgba=(0.87, 0.87, 0.84, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.23, 0.24, 0.26, 1.0))
    smoked_clear = model.material("smoked_clear", rgba=(0.38, 0.46, 0.52, 0.35))
    magnet_dark = model.material("magnet_dark", rgba=(0.16, 0.17, 0.18, 1.0))

    outer_width = 0.320
    outer_height = 0.400
    body_depth = 0.022
    trim_depth = 0.007
    opening_width = 0.242
    opening_height = 0.300
    flap_width = 0.234
    flap_height = 0.296
    hinge_z = opening_height * 0.5

    frame = model.part("frame")
    frame.visual(
        Box((outer_width, body_depth, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.175)),
        material=frame_plastic,
        name="frame_header",
    )
    frame.visual(
        Box((outer_width, body_depth, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, -0.175)),
        material=trim_plastic,
        name="frame_sill",
    )
    frame.visual(
        Box((0.039, body_depth, 0.320)),
        origin=Origin(xyz=(-0.1405, 0.0, 0.0)),
        material=frame_plastic,
        name="frame_left_jamb",
    )
    frame.visual(
        Box((0.039, body_depth, 0.320)),
        origin=Origin(xyz=(0.1405, 0.0, 0.0)),
        material=frame_plastic,
        name="frame_right_jamb",
    )
    frame.visual(
        Box((0.356, trim_depth, 0.052)),
        origin=Origin(xyz=(0.0, 0.0145, 0.176)),
        material=trim_plastic,
        name="front_header_trim",
    )
    frame.visual(
        Box((0.356, trim_depth, 0.052)),
        origin=Origin(xyz=(0.0, 0.0145, -0.176)),
        material=trim_plastic,
        name="front_sill_trim",
    )
    frame.visual(
        Box((0.044, trim_depth, 0.330)),
        origin=Origin(xyz=(-0.156, 0.0145, 0.0)),
        material=trim_plastic,
        name="front_left_trim",
    )
    frame.visual(
        Box((0.044, trim_depth, 0.330)),
        origin=Origin(xyz=(0.156, 0.0145, 0.0)),
        material=trim_plastic,
        name="front_right_trim",
    )
    frame.visual(
        Box((0.356, trim_depth, 0.052)),
        origin=Origin(xyz=(0.0, -0.0145, 0.176)),
        material=trim_plastic,
        name="rear_header_trim",
    )
    frame.visual(
        Box((0.356, trim_depth, 0.052)),
        origin=Origin(xyz=(0.0, -0.0145, -0.176)),
        material=trim_plastic,
        name="rear_sill_trim",
    )
    frame.visual(
        Box((0.044, trim_depth, 0.330)),
        origin=Origin(xyz=(-0.156, -0.0145, 0.0)),
        material=trim_plastic,
        name="rear_left_trim",
    )
    frame.visual(
        Box((0.044, trim_depth, 0.330)),
        origin=Origin(xyz=(0.156, -0.0145, 0.0)),
        material=trim_plastic,
        name="rear_right_trim",
    )
    frame.visual(
        Box((0.210, 0.012, 0.016)),
        origin=Origin(xyz=(0.0, 0.004, hinge_z + 0.027)),
        material=dark_trim,
        name="hinge_cover",
    )

    guide_x = outer_width * 0.5 + 0.012
    guide_z = 0.020
    frame.visual(
        Box((0.030, 0.004, 0.118)),
        origin=Origin(xyz=(guide_x, 0.013, guide_z)),
        material=dark_trim,
        name="lock_guide_back",
    )
    frame.visual(
        Box((0.004, 0.011, 0.118)),
        origin=Origin(xyz=(guide_x - 0.013, 0.0165, guide_z)),
        material=dark_trim,
        name="lock_guide_inner_rail",
    )
    frame.visual(
        Box((0.004, 0.011, 0.118)),
        origin=Origin(xyz=(guide_x + 0.013, 0.0165, guide_z)),
        material=dark_trim,
        name="lock_guide_outer_rail",
    )
    frame.visual(
        Box((0.030, 0.011, 0.007)),
        origin=Origin(xyz=(guide_x, 0.0165, guide_z + 0.0555)),
        material=dark_trim,
        name="lock_guide_top",
    )
    frame.visual(
        Box((0.030, 0.011, 0.007)),
        origin=Origin(xyz=(guide_x, 0.0165, guide_z - 0.0555)),
        material=dark_trim,
        name="lock_guide_bottom",
    )
    frame.visual(
        Box((0.026, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, 0.010, -0.146)),
        material=magnet_dark,
        name="frame_catch",
    )
    frame.inertial = Inertial.from_geometry(
        Box((0.370, 0.040, 0.440)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    flap = model.part("flap")
    flap.visual(
        Box((flap_width, 0.008, 0.022)),
        origin=Origin(xyz=(0.0, 0.001, -0.011)),
        material=frame_plastic,
        name="flap_top_rail",
    )
    flap.visual(
        Box((0.021, 0.008, 0.260)),
        origin=Origin(xyz=(-0.1065, 0.001, -0.152)),
        material=frame_plastic,
        name="flap_left_stile",
    )
    flap.visual(
        Box((0.021, 0.008, 0.260)),
        origin=Origin(xyz=(0.1065, 0.001, -0.152)),
        material=frame_plastic,
        name="flap_right_stile",
    )
    flap.visual(
        Box((flap_width, 0.006, 0.014)),
        origin=Origin(xyz=(0.0, -0.004, -0.289)),
        material=frame_plastic,
        name="flap_bottom_rail",
    )
    flap.visual(
        Box((0.192, 0.003, 0.256)),
        origin=Origin(xyz=(0.0, 0.001, -0.150)),
        material=smoked_clear,
        name="flap_pane",
    )
    flap.visual(
        Cylinder(radius=0.0055, length=0.190),
        origin=Origin(xyz=(0.0, 0.008, -0.006), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_trim,
        name="hinge_bar",
    )
    flap.visual(
        Box((0.072, 0.014, 0.014)),
        origin=Origin(xyz=(0.0, 0.002, -0.270)),
        material=dark_trim,
        name="flap_pull_lip",
    )
    flap.visual(
        Box((0.024, 0.008, 0.010)),
        origin=Origin(xyz=(0.0, 0.003, -0.291)),
        material=magnet_dark,
        name="flap_catch",
    )
    flap.inertial = Inertial.from_geometry(
        Box((flap_width, 0.015, flap_height)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.004, -flap_height * 0.5)),
    )

    lock_slider = model.part("lock_slider")
    lock_slider.visual(
        Box((0.020, 0.006, 0.046)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_trim,
        name="slider_body",
    )
    lock_slider.visual(
        Box((0.026, 0.010, 0.016)),
        origin=Origin(xyz=(0.0, 0.004, 0.012)),
        material=dark_trim,
        name="slider_thumb",
    )
    lock_slider.visual(
        Box((0.006, 0.004, 0.020)),
        origin=Origin(xyz=(-0.011, -0.002, -0.008)),
        material=magnet_dark,
        name="slider_bolt",
    )
    lock_slider.inertial = Inertial.from_geometry(
        Box((0.028, 0.012, 0.050)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.002, 0.0)),
    )

    model.articulation(
        "frame_to_flap",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=flap,
        origin=Origin(xyz=(0.0, 0.0, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=1.25,
        ),
    )
    model.articulation(
        "frame_to_lock_slider",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=lock_slider,
        origin=Origin(xyz=(guide_x, 0.019, 0.045)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.08,
            lower=0.0,
            upper=0.045,
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

    frame = object_model.get_part("frame")
    flap = object_model.get_part("flap")
    lock_slider = object_model.get_part("lock_slider")
    flap_hinge = object_model.get_articulation("frame_to_flap")
    slider_joint = object_model.get_articulation("frame_to_lock_slider")

    ctx.expect_contact(
        frame,
        flap,
        elem_a="frame_catch",
        elem_b="flap_catch",
        contact_tol=0.0015,
        name="magnetic lower catch seats when flap is closed",
    )
    ctx.expect_within(
        lock_slider,
        frame,
        axes="xz",
        inner_elem="slider_body",
        outer_elem="lock_guide_back",
        margin=0.001,
        name="lock slider starts inside its guide track",
    )

    rest_lip_aabb = ctx.part_element_world_aabb(flap, elem="flap_pull_lip")
    with ctx.pose({flap_hinge: flap_hinge.motion_limits.upper}):
        open_lip_aabb = ctx.part_element_world_aabb(flap, elem="flap_pull_lip")
    rest_lip_center_y = None if rest_lip_aabb is None else (rest_lip_aabb[0][1] + rest_lip_aabb[1][1]) * 0.5
    open_lip_center_y = None if open_lip_aabb is None else (open_lip_aabb[0][1] + open_lip_aabb[1][1]) * 0.5
    ctx.check(
        "flap opens outward from the top hinge",
        rest_lip_center_y is not None
        and open_lip_center_y is not None
        and open_lip_center_y > rest_lip_center_y + 0.16,
        details=f"rest_y={rest_lip_center_y}, open_y={open_lip_center_y}",
    )

    rest_slider_pos = ctx.part_world_position(lock_slider)
    with ctx.pose({slider_joint: slider_joint.motion_limits.upper}):
        ctx.expect_within(
            lock_slider,
            frame,
            axes="xz",
            inner_elem="slider_body",
            outer_elem="lock_guide_back",
            margin=0.001,
            name="lock slider remains captured at full travel",
        )
        locked_slider_pos = ctx.part_world_position(lock_slider)
    ctx.check(
        "lock slider moves downward in its side slot",
        rest_slider_pos is not None
        and locked_slider_pos is not None
        and locked_slider_pos[2] < rest_slider_pos[2] - 0.030,
        details=f"rest={rest_slider_pos}, locked={locked_slider_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
