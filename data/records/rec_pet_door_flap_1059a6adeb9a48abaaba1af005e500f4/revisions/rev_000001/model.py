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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _arched_profile(
    width: float,
    height: float,
    *,
    bottom: float = 0.0,
    segments: int = 24,
) -> list[tuple[float, float]]:
    half_width = width * 0.5
    radius = half_width
    straight_top = bottom + height - radius
    points = [
        (-half_width, bottom),
        (half_width, bottom),
        (half_width, straight_top),
    ]
    for index in range(1, segments):
        angle = math.pi * (1.0 - index / segments)
        points.append((radius * math.cos(angle), straight_top + radius * math.sin(angle)))
    points.append((-half_width, straight_top))
    return points


def _arch_panel_mesh(
    *,
    width: float,
    height: float,
    thickness: float,
    bottom: float,
    name: str,
):
    geom = ExtrudeGeometry(
        _arched_profile(width, height, bottom=bottom),
        thickness,
        center=True,
    )
    geom.rotate_x(-math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _arch_ring_mesh(
    *,
    outer_width: float,
    outer_height: float,
    outer_bottom: float,
    inner_width: float,
    inner_height: float,
    inner_bottom: float,
    thickness: float,
    name: str,
):
    geom = ExtrudeWithHolesGeometry(
        _arched_profile(outer_width, outer_height, bottom=outer_bottom),
        [_arched_profile(inner_width, inner_height, bottom=inner_bottom)],
        thickness,
        center=True,
    )
    geom.rotate_x(-math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rounded_top_pet_door")

    frame_white = model.material("frame_white", rgba=(0.93, 0.93, 0.91, 1.0))
    inner_trim = model.material("inner_trim", rgba=(0.83, 0.83, 0.80, 1.0))
    smoked_clear = model.material("smoked_clear", rgba=(0.72, 0.86, 0.96, 0.34))
    dark_hardware = model.material("dark_hardware", rgba=(0.18, 0.19, 0.21, 1.0))
    slider_gray = model.material("slider_gray", rgba=(0.25, 0.27, 0.30, 1.0))
    seal_black = model.material("seal_black", rgba=(0.10, 0.10, 0.11, 1.0))

    outer_width = 0.360
    outer_height = 0.460
    opening_width = 0.258
    opening_height = 0.352
    opening_bottom = 0.018

    frame_depth = 0.040
    bezel_depth = 0.006
    tunnel_depth = frame_depth - (2.0 * bezel_depth)
    tunnel_outer_width = 0.286
    tunnel_outer_height = 0.390
    tunnel_outer_bottom = 0.012

    guide_width = 0.024
    guide_height = 0.080
    guide_bottom = 0.050
    guide_center_z = guide_bottom + guide_height * 0.5
    guide_center_y = 0.025
    lip_depth = 0.003
    stop_depth = 0.006
    guide_x = 0.151
    slider_travel = 0.046

    frame = model.part("frame")
    frame.visual(
        _arch_ring_mesh(
            outer_width=outer_width,
            outer_height=outer_height,
            outer_bottom=0.0,
            inner_width=opening_width,
            inner_height=opening_height,
            inner_bottom=opening_bottom,
            thickness=bezel_depth,
            name="front_bezel_ring",
        ),
        origin=Origin(
            xyz=(0.0, (tunnel_depth * 0.5) + (bezel_depth * 0.5), outer_height)
        ),
        material=frame_white,
        name="front_bezel",
    )
    frame.visual(
        _arch_ring_mesh(
            outer_width=outer_width,
            outer_height=outer_height,
            outer_bottom=0.0,
            inner_width=opening_width,
            inner_height=opening_height,
            inner_bottom=opening_bottom,
            thickness=bezel_depth,
            name="rear_bezel_ring",
        ),
        origin=Origin(
            xyz=(0.0, -((tunnel_depth * 0.5) + (bezel_depth * 0.5)), outer_height)
        ),
        material=frame_white,
        name="rear_bezel",
    )
    frame.visual(
        _arch_ring_mesh(
            outer_width=tunnel_outer_width,
            outer_height=tunnel_outer_height,
            outer_bottom=tunnel_outer_bottom,
            inner_width=opening_width,
            inner_height=opening_height,
            inner_bottom=opening_bottom,
            thickness=tunnel_depth,
            name="tunnel_liner_ring",
        ),
        origin=Origin(xyz=(0.0, 0.0, tunnel_outer_bottom + tunnel_outer_height)),
        material=inner_trim,
        name="tunnel_liner",
    )

    for side_sign, prefix in ((-1.0, "left"), (1.0, "right")):
        center_x = side_sign * guide_x
        frame.visual(
            Box((guide_width, 0.006, guide_height)),
            origin=Origin(xyz=(center_x, 0.020, guide_center_z)),
            material=frame_white,
            name=f"{prefix}_guide_back",
        )
        for offset in (-0.0085, 0.0085):
            frame.visual(
                Box((0.007, lip_depth, guide_height)),
                origin=Origin(xyz=(center_x + offset, 0.0285, guide_center_z)),
                material=frame_white,
                name=f"{prefix}_guide_lip_{'outer' if offset * side_sign > 0.0 else 'inner'}",
            )
        frame.visual(
            Box((guide_width, stop_depth, 0.006)),
            origin=Origin(xyz=(center_x, 0.0260, guide_bottom + 0.003)),
            material=frame_white,
            name=f"{prefix}_guide_lower_stop",
        )
        frame.visual(
            Box((guide_width, stop_depth, 0.006)),
            origin=Origin(xyz=(center_x, 0.0260, guide_bottom + guide_height - 0.003)),
            material=frame_white,
            name=f"{prefix}_guide_upper_stop",
        )

    frame.inertial = Inertial.from_geometry(
        Box((outer_width, frame_depth, outer_height)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, outer_height * 0.5)),
    )

    flap = model.part("flap")
    flap.visual(
        _arch_panel_mesh(
            width=0.244,
            height=0.332,
            thickness=0.004,
            bottom=0.0,
            name="clear_flap_panel",
        ),
        origin=Origin(xyz=(0.0, 0.009, -0.010)),
        material=smoked_clear,
        name="clear_panel",
    )
    flap.visual(
        Box((0.252, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, 0.004, -0.008)),
        material=dark_hardware,
        name="hinge_header",
    )
    flap.visual(
        Box((0.224, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, 0.010, -0.327)),
        material=seal_black,
        name="bottom_seal",
    )
    flap.inertial = Inertial.from_geometry(
        Box((0.252, 0.012, 0.345)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.003, -0.172)),
    )

    hinge_axis_height = 0.356
    model.articulation(
        "frame_to_flap",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=flap,
        origin=Origin(xyz=(0.0, 0.003, hinge_axis_height)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=-0.95,
            upper=0.95,
        ),
    )

    def _add_slider(name: str, side_sign: float) -> None:
        slider = model.part(name)
        slider.visual(
            Box((0.018, 0.004, 0.022)),
            material=slider_gray,
            name="inner_catch",
        )
        slider.visual(
            Box((0.008, 0.004, 0.018)),
            origin=Origin(xyz=(0.0, 0.004, 0.0)),
            material=slider_gray,
            name="neck",
        )
        slider.visual(
            Box((0.018, 0.006, 0.016)),
            origin=Origin(xyz=(0.0, 0.009, 0.0)),
            material=dark_hardware,
            name="thumb_tab",
        )
        slider.visual(
            Box((0.010, 0.008, 0.010)),
            origin=Origin(xyz=(side_sign * 0.012, 0.000, -0.004)),
            material=dark_hardware,
            name="latch_tooth",
        )
        slider.inertial = Inertial.from_geometry(
            Box((0.026, 0.016, 0.024)),
            mass=0.05,
            origin=Origin(xyz=(0.0, 0.005, 0.0)),
        )
        model.articulation(
            f"frame_to_{name}",
            ArticulationType.PRISMATIC,
            parent=frame,
            child=slider,
            origin=Origin(
                xyz=(
                    side_sign * guide_x,
                    guide_center_y,
                    guide_bottom + 0.006 + 0.011,
                )
            ),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=1.2,
                velocity=0.08,
                lower=0.0,
                upper=slider_travel,
            ),
        )

    _add_slider("left_lock_slider", -1.0)
    _add_slider("right_lock_slider", 1.0)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    frame = object_model.get_part("frame")
    flap = object_model.get_part("flap")
    left_lock_slider = object_model.get_part("left_lock_slider")
    right_lock_slider = object_model.get_part("right_lock_slider")
    flap_hinge = object_model.get_articulation("frame_to_flap")
    left_slider_joint = object_model.get_articulation("frame_to_left_lock_slider")
    right_slider_joint = object_model.get_articulation("frame_to_right_lock_slider")

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
        "flap_joint_axis_is_horizontal",
        tuple(flap_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"axis={flap_hinge.axis}",
    )
    ctx.check(
        "left_slider_axis_is_vertical",
        tuple(left_slider_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={left_slider_joint.axis}",
    )
    ctx.check(
        "right_slider_axis_is_vertical",
        tuple(right_slider_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={right_slider_joint.axis}",
    )

    with ctx.pose({flap_hinge: 0.0}):
        ctx.expect_contact(flap, frame)
        ctx.expect_overlap(flap, frame, axes="x", min_overlap=0.20)

    ctx.expect_contact(left_lock_slider, frame)
    ctx.expect_contact(right_lock_slider, frame)

    left_rest = ctx.part_world_position(left_lock_slider)
    right_rest = ctx.part_world_position(right_lock_slider)
    assert left_rest is not None
    assert right_rest is not None

    with ctx.pose({left_slider_joint: 0.046}):
        ctx.expect_contact(left_lock_slider, frame)
        left_raised = ctx.part_world_position(left_lock_slider)
        assert left_raised is not None
        assert left_raised[2] > left_rest[2] + 0.040

    with ctx.pose({right_slider_joint: 0.046}):
        ctx.expect_contact(right_lock_slider, frame)
        right_raised = ctx.part_world_position(right_lock_slider)
        assert right_raised is not None
        assert right_raised[2] > right_rest[2] + 0.040

    flap_closed_aabb = ctx.part_world_aabb(flap)
    assert flap_closed_aabb is not None
    with ctx.pose({flap_hinge: 0.75}):
        flap_open_aabb = ctx.part_world_aabb(flap)
        assert flap_open_aabb is not None
        assert flap_open_aabb[1][1] > flap_closed_aabb[1][1] + 0.12

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
