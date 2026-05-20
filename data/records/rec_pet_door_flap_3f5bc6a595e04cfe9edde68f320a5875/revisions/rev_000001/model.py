from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="insulated_double_flap_pet_door")

    frame_color = model.material("frame_plastic", rgba=(0.16, 0.18, 0.20, 1.0))
    liner_color = model.material("liner_plastic", rgba=(0.24, 0.25, 0.27, 1.0))
    flap_frame_color = model.material("flap_frame", rgba=(0.12, 0.13, 0.14, 1.0))
    flap_panel_color = model.material("flap_panel", rgba=(0.72, 0.82, 0.90, 0.50))

    outer_width = 0.300
    outer_height = 0.430
    opening_width = 0.220
    opening_height = 0.310
    total_depth = 0.100
    bezel_depth = 0.020
    tunnel_depth = total_depth - 2.0 * bezel_depth
    liner_thickness = 0.008

    side_border = (outer_width - opening_width) / 2.0
    top_bottom_border = (outer_height - opening_height) / 2.0

    half_outer_w = outer_width / 2.0
    half_outer_h = outer_height / 2.0
    half_open_w = opening_width / 2.0
    half_open_h = opening_height / 2.0
    half_depth = total_depth / 2.0

    exterior_y = -half_depth + bezel_depth / 2.0
    interior_y = half_depth - bezel_depth / 2.0

    frame = model.part("frame")

    for side_name, y_center in (("exterior", exterior_y), ("interior", interior_y)):
        frame.visual(
            Box((side_border, bezel_depth, outer_height)),
            origin=Origin(xyz=(-(half_open_w + side_border / 2.0), y_center, 0.0)),
            material=frame_color,
            name=f"{side_name}_left_jamb",
        )
        frame.visual(
            Box((side_border, bezel_depth, outer_height)),
            origin=Origin(xyz=(half_open_w + side_border / 2.0, y_center, 0.0)),
            material=frame_color,
            name=f"{side_name}_right_jamb",
        )
        frame.visual(
            Box((outer_width, bezel_depth, top_bottom_border)),
            origin=Origin(xyz=(0.0, y_center, half_open_h + top_bottom_border / 2.0)),
            material=frame_color,
            name=f"{side_name}_header",
        )
        frame.visual(
            Box((outer_width, bezel_depth, top_bottom_border)),
            origin=Origin(xyz=(0.0, y_center, -(half_open_h + top_bottom_border / 2.0))),
            material=frame_color,
            name=f"{side_name}_sill",
        )

    frame.visual(
        Box((liner_thickness, tunnel_depth, opening_height)),
        origin=Origin(xyz=(-(half_open_w + liner_thickness / 2.0), 0.0, 0.0)),
        material=liner_color,
        name="tunnel_left_liner",
    )
    frame.visual(
        Box((liner_thickness, tunnel_depth, opening_height)),
        origin=Origin(xyz=(half_open_w + liner_thickness / 2.0, 0.0, 0.0)),
        material=liner_color,
        name="tunnel_right_liner",
    )
    frame.visual(
        Box((opening_width, tunnel_depth, liner_thickness)),
        origin=Origin(xyz=(0.0, 0.0, half_open_h + liner_thickness / 2.0)),
        material=liner_color,
        name="tunnel_header_liner",
    )
    frame.visual(
        Box((opening_width, tunnel_depth, liner_thickness)),
        origin=Origin(xyz=(0.0, 0.0, -(half_open_h + liner_thickness / 2.0))),
        material=liner_color,
        name="tunnel_sill_liner",
    )
    frame.visual(
        Box((opening_width, bezel_depth, 0.010)),
        origin=Origin(xyz=(0.0, -0.020, 0.160)),
        material=frame_color,
        name="outer_hinge_lintel",
    )
    frame.visual(
        Box((opening_width, bezel_depth, 0.010)),
        origin=Origin(xyz=(0.0, 0.020, 0.160)),
        material=frame_color,
        name="inner_hinge_lintel",
    )

    flap_width = 0.212
    flap_height = 0.304
    flap_thickness = 0.012
    top_cap_height = 0.022
    bottom_rail_height = 0.018
    side_rail_width = 0.018
    glazing_width = flap_width - 2.0 * side_rail_width
    glazing_height = flap_height - top_cap_height - bottom_rail_height
    hinge_barrel_radius = 0.004
    hinge_barrel_length = flap_width - 0.020
    hinge_z = 0.151

    def add_flap(
        part_name: str,
        y_center: float,
        *,
        top_name: str,
        left_name: str,
        right_name: str,
        bottom_name: str,
        panel_name: str,
        hinge_name: str,
    ) -> None:
        flap = model.part(part_name)
        flap.visual(
            Box((flap_width, flap_thickness, top_cap_height)),
            origin=Origin(xyz=(0.0, y_center, -top_cap_height / 2.0)),
            material=flap_frame_color,
            name=top_name,
        )
        flap.visual(
            Box((side_rail_width, flap_thickness, glazing_height)),
            origin=Origin(
                xyz=(
                    -(flap_width / 2.0 - side_rail_width / 2.0),
                    y_center,
                    -(top_cap_height + glazing_height / 2.0),
                )
            ),
            material=flap_frame_color,
            name=left_name,
        )
        flap.visual(
            Box((side_rail_width, flap_thickness, glazing_height)),
            origin=Origin(
                xyz=(
                    flap_width / 2.0 - side_rail_width / 2.0,
                    y_center,
                    -(top_cap_height + glazing_height / 2.0),
                )
            ),
            material=flap_frame_color,
            name=right_name,
        )
        flap.visual(
            Box((flap_width, flap_thickness, bottom_rail_height)),
            origin=Origin(xyz=(0.0, y_center, -(flap_height - bottom_rail_height / 2.0))),
            material=flap_frame_color,
            name=bottom_name,
        )
        flap.visual(
            Box((glazing_width, 0.006, glazing_height)),
            origin=Origin(xyz=(0.0, y_center, -(top_cap_height + glazing_height / 2.0))),
            material=flap_panel_color,
            name=panel_name,
        )
        flap.visual(
            Cylinder(radius=hinge_barrel_radius, length=hinge_barrel_length),
            origin=Origin(xyz=(0.0, y_center, 0.0), rpy=(0.0, 1.5707963267948966, 0.0)),
            material=flap_frame_color,
            name=hinge_name,
        )

    add_flap(
        "outer_flap",
        y_center=-0.020,
        top_name="outer_flap_top_rail",
        left_name="outer_flap_left_rail",
        right_name="outer_flap_right_rail",
        bottom_name="outer_flap_bottom_rail",
        panel_name="outer_flap_panel",
        hinge_name="outer_flap_hinge_barrel",
    )
    add_flap(
        "inner_flap",
        y_center=0.020,
        top_name="inner_flap_top_rail",
        left_name="inner_flap_left_rail",
        right_name="inner_flap_right_rail",
        bottom_name="inner_flap_bottom_rail",
        panel_name="inner_flap_panel",
        hinge_name="inner_flap_hinge_barrel",
    )

    model.articulation(
        "frame_to_outer_flap",
        ArticulationType.REVOLUTE,
        parent=frame,
        child="outer_flap",
        origin=Origin(xyz=(0.0, -0.020, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.5,
            lower=-1.25,
            upper=1.25,
        ),
    )
    model.articulation(
        "frame_to_inner_flap",
        ArticulationType.REVOLUTE,
        parent=frame,
        child="inner_flap",
        origin=Origin(xyz=(0.0, 0.020, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.5,
            lower=-1.25,
            upper=1.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    outer_flap = object_model.get_part("outer_flap")
    inner_flap = object_model.get_part("inner_flap")
    outer_hinge = object_model.get_articulation("frame_to_outer_flap")
    inner_hinge = object_model.get_articulation("frame_to_inner_flap")

    def elem_center_y(part, elem: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        return 0.5 * (aabb[0][1] + aabb[1][1])

    ctx.check(
        "hinges are horizontal and mounted near the top of the frame opening",
        outer_hinge.axis == (1.0, 0.0, 0.0)
        and inner_hinge.axis == (1.0, 0.0, 0.0)
        and 0.145 <= outer_hinge.origin.xyz[2] <= 0.155
        and 0.145 <= inner_hinge.origin.xyz[2] <= 0.155,
        details=(
            f"outer axis={outer_hinge.axis}, outer origin={outer_hinge.origin.xyz}, "
            f"inner axis={inner_hinge.axis}, inner origin={inner_hinge.origin.xyz}"
        ),
    )
    ctx.check(
        "outer and inner hinges sit on separate depth planes",
        outer_hinge.origin.xyz[1] < -0.015 and inner_hinge.origin.xyz[1] > 0.015,
        details=f"outer y={outer_hinge.origin.xyz[1]}, inner y={inner_hinge.origin.xyz[1]}",
    )
    ctx.expect_origin_gap(
        inner_flap,
        outer_flap,
        axis="y",
        min_gap=0.035,
        name="closed flaps are depth-separated inside the insulated tunnel",
    )

    with ctx.pose({outer_hinge: 0.0, inner_hinge: 0.0}):
        ctx.expect_gap(
            frame,
            outer_flap,
            axis="z",
            min_gap=0.003,
            max_gap=0.008,
            positive_elem="exterior_header",
            negative_elem="outer_flap_top_rail",
            name="outer flap hangs just below the exterior header",
        )
        ctx.expect_gap(
            outer_flap,
            frame,
            axis="z",
            min_gap=0.0,
            max_gap=0.003,
            positive_elem="outer_flap_bottom_rail",
            negative_elem="exterior_sill",
            name="outer flap closes just above the exterior sill",
        )
        ctx.expect_gap(
            frame,
            inner_flap,
            axis="z",
            min_gap=0.003,
            max_gap=0.008,
            positive_elem="interior_header",
            negative_elem="inner_flap_top_rail",
            name="inner flap hangs just below the interior header",
        )
        ctx.expect_gap(
            inner_flap,
            frame,
            axis="z",
            min_gap=0.0,
            max_gap=0.003,
            positive_elem="inner_flap_bottom_rail",
            negative_elem="interior_sill",
            name="inner flap closes just above the interior sill",
        )

    outer_rest_y = elem_center_y(outer_flap, "outer_flap_panel")
    inner_rest_y = elem_center_y(inner_flap, "inner_flap_panel")

    with ctx.pose({outer_hinge: -0.70}):
        outer_open_y = elem_center_y(outer_flap, "outer_flap_panel")
        inner_still_y = elem_center_y(inner_flap, "inner_flap_panel")

    ctx.check(
        "outer flap rotates independently on its own hinge",
        outer_rest_y is not None
        and outer_open_y is not None
        and inner_rest_y is not None
        and inner_still_y is not None
        and outer_open_y < outer_rest_y - 0.050
        and abs(inner_still_y - inner_rest_y) < 1e-6,
        details=(
            f"outer rest y={outer_rest_y}, outer open y={outer_open_y}, "
            f"inner rest y={inner_rest_y}, inner during outer motion y={inner_still_y}"
        ),
    )

    with ctx.pose({inner_hinge: 0.70}):
        inner_open_y = elem_center_y(inner_flap, "inner_flap_panel")
        outer_still_y = elem_center_y(outer_flap, "outer_flap_panel")

    ctx.check(
        "inner flap rotates independently on its own hinge",
        inner_rest_y is not None
        and inner_open_y is not None
        and outer_rest_y is not None
        and outer_still_y is not None
        and inner_open_y > inner_rest_y + 0.050
        and abs(outer_still_y - outer_rest_y) < 1e-6,
        details=(
            f"inner rest y={inner_rest_y}, inner open y={inner_open_y}, "
            f"outer rest y={outer_rest_y}, outer during inner motion y={outer_still_y}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
