from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _rounded_rect_prism(width_y: float, height_z: float, depth_x: float, radius: float):
    """Rounded-rectangle prism with its back face on local x=0."""
    geom = ExtrudeGeometry.from_z0(
        rounded_rect_profile(width_y, height_z, radius, corner_segments=8),
        depth_x,
        cap=True,
        closed=True,
    )
    # ExtrudeGeometry starts as (profile X, profile Y, depth Z).  Rotate into
    # the appliance frame so depth is +X, width is Y, and height is Z.
    geom.rotate_y(math.pi / 2.0).rotate_x(math.pi / 2.0)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="household_rice_cooker")

    warm_white = model.material("warm_white_plastic", rgba=(0.88, 0.86, 0.80, 1.0))
    gray = model.material("soft_gray_plastic", rgba=(0.46, 0.46, 0.44, 1.0))
    dark = model.material("dark_control_plastic", rgba=(0.08, 0.085, 0.09, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    metal = model.material("brushed_inner_metal", rgba=(0.72, 0.70, 0.66, 1.0))
    button_blue = model.material("blue_mode_button", rgba=(0.13, 0.25, 0.62, 1.0))
    button_red = model.material("red_mode_button", rgba=(0.70, 0.12, 0.08, 1.0))

    body = model.part("body")

    body_shell = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.126, 0.025),
            (0.160, 0.036),
            (0.176, 0.070),
            (0.180, 0.215),
            (0.174, 0.255),
            (0.168, 0.263),
        ],
        inner_profile=[
            (0.092, 0.040),
            (0.124, 0.050),
            (0.148, 0.076),
            (0.151, 0.230),
            (0.142, 0.255),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    body.visual(
        mesh_from_geometry(body_shell, "body_shell"),
        material=warm_white,
        name="body_shell",
    )

    body.visual(
        Cylinder(radius=0.137, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=warm_white,
        name="base_plate",
    )

    pot_rim = LatheGeometry.from_shell_profiles(
        outer_profile=[(0.143, 0.238), (0.146, 0.248), (0.144, 0.258)],
        inner_profile=[(0.130, 0.236), (0.134, 0.248), (0.132, 0.258)],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    body.visual(
        mesh_from_geometry(pot_rim, "inner_pot_rim"),
        material=metal,
        name="inner_pot_rim",
    )
    body.visual(
        Cylinder(radius=0.166, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.2615)),
        material=metal,
        name="rim_seat",
    )

    for index, (x, y) in enumerate(
        ((0.096, 0.096), (0.096, -0.096), (-0.096, 0.096), (-0.096, -0.096))
    ):
        body.visual(
            Cylinder(radius=0.019, length=0.025),
            origin=Origin(xyz=(x, y, 0.0125)),
            material=rubber,
            name=f"foot_{index}",
        )

    body.visual(
        mesh_from_geometry(_rounded_rect_prism(0.124, 0.075, 0.026, 0.015), "control_cluster"),
        origin=Origin(xyz=(0.168, 0.0, 0.108)),
        material=dark,
        name="control_cluster",
    )
    body.visual(
        mesh_from_geometry(_rounded_rect_prism(0.092, 0.038, 0.014, 0.013), "release_socket"),
        origin=Origin(xyz=(0.170, 0.0, 0.232)),
        material=gray,
        name="release_socket",
    )
    body.visual(
        Box((0.0025, 0.030, 0.006)),
        origin=Origin(xyz=(0.1945, -0.028, 0.146)),
        material=button_blue,
        name="mode_indicator_0",
    )
    body.visual(
        Box((0.0025, 0.030, 0.006)),
        origin=Origin(xyz=(0.1945, 0.028, 0.146)),
        material=button_red,
        name="mode_indicator_1",
    )

    # Three-part rear hinge: two fixed body knuckles and one moving lid knuckle.
    for y in (-0.075, 0.075):
        body.visual(
            Cylinder(radius=0.012, length=0.040),
            origin=Origin(xyz=(-0.191, y, 0.266), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=gray,
            name=f"hinge_knuckle_{0 if y < 0 else 1}",
        )
        body.visual(
            Box((0.040, 0.034, 0.020)),
            origin=Origin(xyz=(-0.175, y, 0.256)),
            material=gray,
            name=f"hinge_base_{0 if y < 0 else 1}",
        )

    lid = model.part("lid")
    lid_shell = LatheGeometry(
        [
            (0.000, 0.004),
            (0.142, 0.004),
            (0.188, 0.010),
            (0.197, 0.020),
            (0.191, 0.035),
            (0.157, 0.053),
            (0.080, 0.064),
            (0.000, 0.064),
        ],
        segments=72,
        closed=True,
    )
    lid.visual(
        mesh_from_geometry(lid_shell, "lid_shell"),
        origin=Origin(xyz=(0.191, 0.0, 0.0)),
        material=warm_white,
        name="lid_shell",
    )
    inner_lid_frame = LatheGeometry.from_shell_profiles(
        outer_profile=[(0.150, 0.000), (0.158, 0.003), (0.158, 0.008)],
        inner_profile=[(0.104, 0.000), (0.118, 0.003), (0.118, 0.008)],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    lid.visual(
        mesh_from_geometry(inner_lid_frame, "inner_lid_frame"),
        origin=Origin(xyz=(0.191, 0.0, -0.003)),
        material=metal,
        name="inner_lid_frame",
    )
    lid.visual(
        Cylinder(radius=0.0105, length=0.056),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gray,
        name="hinge_barrel",
    )
    lid.visual(
        Box((0.060, 0.050, 0.010)),
        origin=Origin(xyz=(0.030, 0.0, 0.004)),
        material=gray,
        name="hinge_tab",
    )

    release_button = model.part("release_button")
    release_button.visual(
        mesh_from_geometry(
            _rounded_rect_prism(0.074, 0.026, 0.014, 0.010),
            "release_button_cap",
        ),
        material=gray,
        name="release_button_cap",
    )

    mode_button_0 = model.part("mode_button_0")
    mode_button_0.visual(
        mesh_from_geometry(_rounded_rect_prism(0.036, 0.024, 0.010, 0.007), "mode_button_0_cap"),
        material=button_blue,
        name="mode_button_cap",
    )

    mode_button_1 = model.part("mode_button_1")
    mode_button_1.visual(
        mesh_from_geometry(_rounded_rect_prism(0.036, 0.024, 0.010, 0.007), "mode_button_1_cap"),
        material=button_red,
        name="mode_button_cap",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-0.191, 0.0, 0.266)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.25),
    )
    model.articulation(
        "body_to_release_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=release_button,
        origin=Origin(xyz=(0.184, 0.0, 0.232)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.08, lower=0.0, upper=0.008),
    )
    model.articulation(
        "body_to_mode_button_0",
        ArticulationType.PRISMATIC,
        parent=body,
        child=mode_button_0,
        origin=Origin(xyz=(0.194, -0.028, 0.108)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=0.06, lower=0.0, upper=0.006),
    )
    model.articulation(
        "body_to_mode_button_1",
        ArticulationType.PRISMATIC,
        parent=body,
        child=mode_button_1,
        origin=Origin(xyz=(0.194, 0.028, 0.108)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=0.06, lower=0.0, upper=0.006),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    release_button = object_model.get_part("release_button")
    mode_button_0 = object_model.get_part("mode_button_0")
    mode_button_1 = object_model.get_part("mode_button_1")
    lid_hinge = object_model.get_articulation("body_to_lid")
    release_slide = object_model.get_articulation("body_to_release_button")
    mode_slide_0 = object_model.get_articulation("body_to_mode_button_0")
    mode_slide_1 = object_model.get_articulation("body_to_mode_button_1")

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="inner_lid_frame",
            negative_elem="body_shell",
            max_gap=0.012,
            max_penetration=0.0,
            name="closed lid sits just above the body rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_shell",
            elem_b="body_shell",
            min_overlap=0.25,
            name="lid footprint covers the cylindrical body",
        )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: 1.0}):
        opened_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid hinge opens upward from the rear rim",
        closed_lid_aabb is not None
        and opened_lid_aabb is not None
        and opened_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.08,
        details=f"closed={closed_lid_aabb}, opened={opened_lid_aabb}",
    )

    ctx.expect_contact(
        release_button,
        body,
        elem_a="release_button_cap",
        elem_b="release_socket",
        contact_tol=0.0002,
        name="release button is seated in its front socket",
    )
    ctx.allow_overlap(
        release_button,
        body,
        elem_a="release_button_cap",
        elem_b="release_socket",
        reason="At full travel the latch button intentionally slides into the simplified socket pocket.",
    )
    rest_release = ctx.part_world_position(release_button)
    with ctx.pose({release_slide: 0.008}):
        pressed_release = ctx.part_world_position(release_button)
        ctx.expect_within(
            release_button,
            body,
            axes="yz",
            inner_elem="release_button_cap",
            outer_elem="release_socket",
            margin=0.003,
            name="pressed release button stays guided by its socket",
        )
        ctx.expect_overlap(
            release_button,
            body,
            axes="x",
            elem_a="release_button_cap",
            elem_b="release_socket",
            min_overlap=0.006,
            name="pressed release button remains inserted in its socket",
        )
    ctx.check(
        "release button slides inward to unlatch",
        rest_release is not None
        and pressed_release is not None
        and pressed_release[0] < rest_release[0] - 0.006,
        details=f"rest={rest_release}, pressed={pressed_release}",
    )

    for button, slide, name in (
        (mode_button_0, mode_slide_0, "mode button 0"),
        (mode_button_1, mode_slide_1, "mode button 1"),
    ):
        ctx.allow_overlap(
            button,
            body,
            elem_a="mode_button_cap",
            elem_b="control_cluster",
            reason=f"{name} intentionally presses into the simplified control-cluster pocket.",
        )
        ctx.expect_contact(
            button,
            body,
            elem_a="mode_button_cap",
            elem_b="control_cluster",
            contact_tol=0.0002,
            name=f"{name} is mounted proud of the control cluster",
        )
        rest = ctx.part_world_position(button)
        with ctx.pose({slide: 0.006}):
            pressed = ctx.part_world_position(button)
            ctx.expect_within(
                button,
                body,
                axes="yz",
                inner_elem="mode_button_cap",
                outer_elem="control_cluster",
                margin=0.002,
                name=f"{name} stays guided by the control cluster",
            )
            ctx.expect_overlap(
                button,
                body,
                axes="x",
                elem_a="mode_button_cap",
                elem_b="control_cluster",
                min_overlap=0.004,
                name=f"{name} remains inserted when pressed",
            )
        ctx.check(
            f"{name} pushes inward",
            rest is not None and pressed is not None and pressed[0] < rest[0] - 0.004,
            details=f"rest={rest}, pressed={pressed}",
        )

    return ctx.report()


object_model = build_object_model()
