from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    sweep_profile_along_spline,
    tube_from_spline_points,
)


def _ellipse_loop(
    x: float,
    *,
    y_radius: float,
    z_radius: float,
    z_center: float,
    segments: int = 56,
) -> list[tuple[float, float, float]]:
    return [
        (
            x,
            y_radius * math.cos(2.0 * math.pi * i / segments),
            z_center + z_radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _oval_disc(normal_sign: float, *, y_radius: float, z_radius: float, x: float):
    """A very thin oval disc in the local YZ plane."""
    return section_loft(
        [
            _ellipse_loop(x - normal_sign * 0.0012, y_radius=y_radius, z_radius=z_radius, z_center=-0.060),
            _ellipse_loop(x + normal_sign * 0.0012, y_radius=y_radius, z_radius=z_radius, z_center=-0.060),
        ]
    )


def _cup_shell_mesh():
    """Smooth over-ear cup shell, local origin at the folding hinge axis."""
    return section_loft(
        [
            _ellipse_loop(-0.024, y_radius=0.032, z_radius=0.046, z_center=-0.073),
            _ellipse_loop(-0.014, y_radius=0.039, z_radius=0.056, z_center=-0.073),
            _ellipse_loop(0.008, y_radius=0.043, z_radius=0.060, z_center=-0.073),
            _ellipse_loop(0.023, y_radius=0.035, z_radius=0.050, z_center=-0.073),
        ]
    )


def _oval_cushion_mesh(inner_sign: float):
    points = [
        (
            inner_sign * 0.025,
            0.034 * math.cos(2.0 * math.pi * i / 48),
            -0.060 + 0.047 * math.sin(2.0 * math.pi * i / 48),
        )
        for i in range(48)
    ]
    return tube_from_spline_points(
        points,
        radius=0.0065,
        closed_spline=True,
        samples_per_segment=3,
        radial_segments=16,
    )


def _slider_slot_mesh():
    geom = ExtrudeGeometry(
        rounded_rect_profile(0.046, 0.013, 0.006, corner_segments=8),
        0.0022,
        center=True,
    )
    # Extrusion local Z becomes cup local +X, and the rounded rectangle lies in YZ.
    return geom.rotate_y(math.pi / 2.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="noise_canceling_headphones")

    matte_black = model.material("matte_black", rgba=(0.015, 0.016, 0.018, 1.0))
    satin_graphite = model.material("satin_graphite", rgba=(0.12, 0.13, 0.15, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.035, 0.036, 0.038, 1.0))
    soft_leather = model.material("soft_leather", rgba=(0.025, 0.025, 0.026, 1.0))
    dark_mesh = model.material("dark_mesh", rgba=(0.01, 0.012, 0.014, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.30, 0.31, 0.33, 1.0))
    control_gray = model.material("control_gray", rgba=(0.46, 0.48, 0.50, 1.0))

    headband = model.part("headband")
    band_path = [
        (-0.108, 0.0, 0.162),
        (-0.086, 0.0, 0.202),
        (-0.035, 0.0, 0.247),
        (0.0, 0.0, 0.255),
        (0.035, 0.0, 0.247),
        (0.086, 0.0, 0.202),
        (0.108, 0.0, 0.162),
    ]
    headband.visual(
        mesh_from_geometry(
            sweep_profile_along_spline(
                band_path,
                profile=rounded_rect_profile(0.016, 0.010, 0.004, corner_segments=6),
                samples_per_segment=12,
                cap_profile=True,
            ),
            "smooth_headband",
        ),
        material=matte_black,
        name="outer_band",
    )
    pad_path = [(x, y, z - 0.006) for x, y, z in band_path[1:-1]]
    headband.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                pad_path,
                radius=0.006,
                samples_per_segment=12,
                radial_segments=16,
            ),
            "headband_pad",
        ),
        material=dark_rubber,
        name="underside_pad",
    )

    for side_name, side in (("left", -1.0), ("right", 1.0)):
        # The yoke is part of the fixed headband assembly.  Fork arms and
        # washers visibly support the folding hinge barrel on each cup.
        x = side * 0.108
        headband.visual(
            Cylinder(radius=0.010, length=0.018),
            origin=Origin(xyz=(x, 0.0, 0.154), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=gunmetal,
            name=f"{side_name}_band_socket",
        )
        for y, tag in ((-0.024, "front"), (0.024, "rear")):
            washer_name = {
                ("left", "front"): "left_front_washer",
                ("left", "rear"): "left_rear_washer",
                ("right", "front"): "right_front_washer",
                ("right", "rear"): "right_rear_washer",
            }[(side_name, tag)]
            arm = tube_from_spline_points(
                [
                    (side * 0.101, 0.0, 0.154),
                    (side * 0.107, y * 0.78, 0.154),
                    (side * 0.108, y * 1.24, 0.135),
                ],
                radius=0.0042,
                samples_per_segment=8,
                radial_segments=12,
            )
            headband.visual(
                mesh_from_geometry(arm, f"{side_name}_{tag}_yoke_arm"),
                material=gunmetal,
                name=f"{side_name}_{tag}_yoke_arm",
            )
            headband.visual(
                Cylinder(radius=0.012, length=0.008),
                origin=Origin(xyz=(x, y, 0.135), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=gunmetal,
                name=washer_name,
            )

    def build_earcup(name: str, side: float):
        cup = model.part(name)
        inner_sign = -side
        outer_sign = side
        cup.visual(
            mesh_from_geometry(_cup_shell_mesh(), f"{name}_smooth_shell"),
            material=satin_graphite,
            name="smooth_shell",
        )
        cup.visual(
            mesh_from_geometry(_oval_cushion_mesh(inner_sign), f"{name}_ear_cushion"),
            material=soft_leather,
            name="ear_cushion",
        )
        cup.visual(
            mesh_from_geometry(
                _oval_disc(inner_sign, y_radius=0.028, z_radius=0.039, x=inner_sign * 0.027),
                f"{name}_speaker_fabric",
            ),
            material=dark_mesh,
            name="speaker_fabric",
        )
        cup.visual(
            Cylinder(radius=0.0068, length=0.040),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=gunmetal,
            name="hinge_barrel",
        )
        cup.visual(
            Box((0.018, 0.026, 0.030)),
            origin=Origin(xyz=(0.0, 0.0, -0.014)),
            material=satin_graphite,
            name="hinge_bridge",
        )
        # Small feed-forward microphone detail for the noise cancelling shell.
        cup.visual(
            Cylinder(radius=0.004, length=0.002),
            origin=Origin(
                xyz=(outer_sign * 0.0235, -0.026, -0.033),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=dark_mesh,
            name="anc_mic_port",
        )
        return cup

    left_earcup = build_earcup("left_earcup", -1.0)
    right_earcup = build_earcup("right_earcup", 1.0)

    right_earcup.visual(
        mesh_from_geometry(_slider_slot_mesh(), "right_slider_guide_slot"),
        origin=Origin(xyz=(0.0235, -0.023, -0.060)),
        material=dark_mesh,
        name="slider_slot",
    )

    control_slider = model.part("control_slider")
    control_slider.visual(
        Box((0.008, 0.018, 0.014)),
        origin=Origin(xyz=(0.004, 0.0, 0.0)),
        material=control_gray,
        name="thumb_cap",
    )
    control_slider.visual(
        Box((0.004, 0.008, 0.020)),
        origin=Origin(xyz=(-0.002, 0.0, 0.0)),
        material=dark_rubber,
        name="guide_stem",
    )

    model.articulation(
        "left_yoke_hinge",
        ArticulationType.REVOLUTE,
        parent=headband,
        child=left_earcup,
        origin=Origin(xyz=(-0.108, 0.0, 0.135)),
        # Positive motion folds the left cup inward toward the headband center.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.45),
    )
    model.articulation(
        "right_yoke_hinge",
        ArticulationType.REVOLUTE,
        parent=headband,
        child=right_earcup,
        origin=Origin(xyz=(0.108, 0.0, 0.135)),
        # Positive motion folds the right cup inward toward the headband center.
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.45),
    )
    model.articulation(
        "slider_travel",
        ArticulationType.PRISMATIC,
        parent=right_earcup,
        child=control_slider,
        origin=Origin(xyz=(0.028, -0.023, -0.060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=0.05, lower=-0.010, upper=0.010),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    left = object_model.get_part("left_earcup")
    right = object_model.get_part("right_earcup")
    slider = object_model.get_part("control_slider")
    left_hinge = object_model.get_articulation("left_yoke_hinge")
    right_hinge = object_model.get_articulation("right_yoke_hinge")
    slider_joint = object_model.get_articulation("slider_travel")

    ctx.expect_contact(
        "headband",
        left,
        elem_a="left_front_washer",
        elem_b="hinge_barrel",
        contact_tol=0.001,
        name="left yoke washer captures hinge barrel",
    )
    ctx.expect_contact(
        "headband",
        right,
        elem_a="right_front_washer",
        elem_b="hinge_barrel",
        contact_tol=0.001,
        name="right yoke washer captures hinge barrel",
    )
    ctx.expect_within(
        slider,
        right,
        axes="yz",
        inner_elem="guide_stem",
        outer_elem="slider_slot",
        margin=0.002,
        name="slider stem sits inside guide slot footprint",
    )

    rest_left_aabb = ctx.part_world_aabb(left)
    rest_right_aabb = ctx.part_world_aabb(right)
    with ctx.pose({left_hinge: 1.20, right_hinge: 1.20, slider_joint: 0.010}):
        folded_left_aabb = ctx.part_world_aabb(left)
        folded_right_aabb = ctx.part_world_aabb(right)
        moved_slider = ctx.part_world_position(slider)
        ctx.expect_within(
            slider,
            right,
            axes="yz",
            inner_elem="guide_stem",
            outer_elem="slider_slot",
            margin=0.002,
            name="extended slider remains in short guide slot",
        )

    def mid_x(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return 0.5 * (lo[0] + hi[0])

    rest_slider = ctx.part_world_position(slider)
    ctx.check(
        "earcups fold inward on yoke hinges",
        rest_left_aabb is not None
        and folded_left_aabb is not None
        and rest_right_aabb is not None
        and folded_right_aabb is not None
        and mid_x(folded_left_aabb) > mid_x(rest_left_aabb) + 0.018
        and mid_x(folded_right_aabb) < mid_x(rest_right_aabb) - 0.018,
        details=f"left rest/fold={rest_left_aabb}/{folded_left_aabb}, right rest/fold={rest_right_aabb}/{folded_right_aabb}",
    )
    ctx.check(
        "control slider moves upward prismatically",
        rest_slider is not None and moved_slider is not None and moved_slider[2] > rest_slider[2] + 0.008,
        details=f"rest={rest_slider}, moved={moved_slider}",
    )

    return ctx.report()


object_model = build_object_model()
