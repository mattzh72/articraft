from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ExtrudeWithHolesGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _add_quad(mesh: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    mesh.add_face(a, b, c)
    mesh.add_face(a, c, d)


def _canopy_shell_geometry() -> MeshGeometry:
    """Thin stainless trapezoid shell with open intake and chimney throat."""

    mesh = MeshGeometry()
    z_bottom = 0.060
    z_top = 0.320
    bottom_width = 0.920
    bottom_depth = 0.540
    top_width = 0.380
    top_depth = 0.260
    top_y = 0.155
    wall = 0.018

    def loop(width: float, depth: float, y_center: float, z: float) -> list[int]:
        points = (
            (-width / 2.0, y_center - depth / 2.0, z),
            (width / 2.0, y_center - depth / 2.0, z),
            (width / 2.0, y_center + depth / 2.0, z),
            (-width / 2.0, y_center + depth / 2.0, z),
        )
        return [mesh.add_vertex(*point) for point in points]

    outer_bottom = loop(bottom_width, bottom_depth, 0.0, z_bottom)
    outer_top = loop(top_width, top_depth, top_y, z_top)
    inner_bottom = loop(bottom_width - 2.0 * wall, bottom_depth - 2.0 * wall, 0.0, z_bottom)
    inner_top = loop(top_width - 2.0 * wall, top_depth - 2.0 * wall, top_y, z_top)

    for i in range(4):
        j = (i + 1) % 4
        _add_quad(mesh, outer_bottom[i], outer_bottom[j], outer_top[j], outer_top[i])
        _add_quad(mesh, inner_bottom[j], inner_bottom[i], inner_top[i], inner_top[j])
        _add_quad(mesh, outer_bottom[j], outer_bottom[i], inner_bottom[i], inner_bottom[j])
        _add_quad(mesh, outer_top[i], outer_top[j], inner_top[j], inner_top[i])

    return mesh


def _offset_profile(profile: list[tuple[float, float]], dx: float, dy: float) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _control_panel_geometry() -> MeshGeometry:
    outer = rounded_rect_profile(0.580, 0.058, 0.006, corner_segments=6)
    button_hole = rounded_rect_profile(0.052, 0.026, 0.004, corner_segments=5)
    holes = [_offset_profile(button_hole, x, 0.0) for x in (-0.180, -0.090, 0.0, 0.090, 0.180)]
    return ExtrudeWithHolesGeometry(outer, holes, 0.006, cap=True, center=True, closed=True)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chimney_range_hood")

    stainless = model.material("brushed_stainless", rgba=(0.70, 0.71, 0.69, 1.0))
    dark_metal = model.material("dark_anodized_metal", rgba=(0.03, 0.035, 0.04, 1.0))
    button_plastic = model.material("satin_button_plastic", rgba=(0.18, 0.19, 0.20, 1.0))
    shadow = model.material("deep_filter_shadow", rgba=(0.01, 0.012, 0.014, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_geometry(_canopy_shell_geometry(), "lower_canopy_shell"),
        material=stainless,
        name="lower_canopy_shell",
    )
    housing.visual(
        mesh_from_geometry(_control_panel_geometry(), "control_panel"),
        origin=Origin(xyz=(0.0, -0.285, 0.100), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="control_panel",
    )
    housing.visual(
        Box((0.600, 0.110, 0.012)),
        origin=Origin(xyz=(0.0, -0.235, 0.132)),
        material=stainless,
        name="panel_top_flange",
    )
    housing.visual(
        mesh_from_geometry(
            SlotPatternPanelGeometry(
                (0.68, 0.34),
                0.004,
                slot_size=(0.050, 0.009),
                pitch=(0.070, 0.030),
                frame=0.018,
                corner_radius=0.004,
                slot_angle_deg=0.0,
                stagger=True,
            ),
            "underside_filter",
        ),
        origin=Origin(xyz=(0.0, -0.010, 0.058)),
        material=shadow,
        name="underside_filter",
    )
    housing.visual(
        Box((0.96, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, -0.174, 0.063)),
        material=stainless,
        name="front_filter_rail",
    )
    housing.visual(
        Box((0.96, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, 0.154, 0.063)),
        material=stainless,
        name="rear_filter_rail",
    )
    housing.visual(
        Box((0.340, 0.012, 0.880)),
        origin=Origin(xyz=(0.0, 0.039, 0.760)),
        material=stainless,
        name="chimney_front",
    )
    housing.visual(
        Box((0.012, 0.220, 0.880)),
        origin=Origin(xyz=(-0.176, 0.150, 0.760)),
        material=stainless,
        name="chimney_side_0",
    )
    housing.visual(
        Box((0.012, 0.220, 0.880)),
        origin=Origin(xyz=(0.176, 0.150, 0.760)),
        material=stainless,
        name="chimney_side_1",
    )
    housing.visual(
        Box((0.340, 0.010, 0.880)),
        origin=Origin(xyz=(0.0, 0.265, 0.760)),
        material=stainless,
        name="chimney_rear_flange",
    )
    housing.visual(
        Box((0.014, 0.004, 0.860)),
        origin=Origin(xyz=(0.0, 0.032, 0.760)),
        material=dark_metal,
        name="chimney_center_seam",
    )

    button_xs = (-0.180, -0.090, 0.0, 0.090, 0.180)
    for index, x_pos in enumerate(button_xs):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.042, 0.016, 0.018)),
            material=button_plastic,
            name="button_cap",
        )
        model.articulation(
            f"housing_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=housing,
            child=button,
            origin=Origin(xyz=(x_pos, -0.296, 0.100)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=0.12, lower=0.0, upper=0.008),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    buttons = [object_model.get_part(f"button_{index}") for index in range(5)]
    joints = [object_model.get_articulation(f"housing_to_button_{index}") for index in range(5)]

    ctx.check("five separate front buttons", len(buttons) == 5 and len(joints) == 5)
    for index, (button, joint) in enumerate(zip(buttons, joints)):
        limits = joint.motion_limits
        ctx.check(
            f"button_{index} is a short prismatic plunger",
            joint.articulation_type == ArticulationType.PRISMATIC
            and joint.axis == (0.0, 1.0, 0.0)
            and limits is not None
            and limits.lower == 0.0
            and 0.006 <= limits.upper <= 0.010,
            details=f"type={joint.articulation_type}, axis={joint.axis}, limits={limits}",
        )
        ctx.expect_within(
            button,
            housing,
            axes="xz",
            inner_elem="button_cap",
            outer_elem="control_panel",
            margin=0.002,
            name=f"button_{index} sits in control panel opening",
        )
        ctx.expect_gap(
            housing,
            button,
            axis="y",
            positive_elem="control_panel",
            negative_elem="button_cap",
            min_gap=-0.001,
            max_gap=0.002,
            name=f"button_{index} rests flush to the panel face",
        )

        rest_position = ctx.part_world_position(button)
        with ctx.pose({joint: limits.upper}):
            pressed_position = ctx.part_world_position(button)
            ctx.expect_within(
                button,
                housing,
                axes="xz",
                inner_elem="button_cap",
                outer_elem="control_panel",
                margin=0.002,
                name=f"button_{index} stays aligned while pressed",
            )
        ctx.check(
            f"button_{index} travels inward",
            rest_position is not None
            and pressed_position is not None
            and pressed_position[1] > rest_position[1] + 0.006,
            details=f"rest={rest_position}, pressed={pressed_position}",
        )

    return ctx.report()


object_model = build_object_model()
