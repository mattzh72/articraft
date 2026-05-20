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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sample_catmull_rom_spline_2d,
)


def _circle_profile(radius: float, *, segments: int = 32) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _arc_points(
    radius: float,
    start_angle: float,
    end_angle: float,
    *,
    segments: int,
) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(start_angle + (end_angle - start_angle) * (index / segments)),
            radius * math.sin(start_angle + (end_angle - start_angle) * (index / segments)),
        )
        for index in range(segments + 1)
    ]


def _transform_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
    angle: float = 0.0,
) -> list[tuple[float, float]]:
    c = math.cos(angle)
    s = math.sin(angle)
    return [(c * x - s * y + dx, s * x + c * y + dy) for x, y in profile]


def _spinner_outline() -> list[tuple[float, float]]:
    hub_radius = 0.018
    attach_angle = math.radians(48.0)
    attach_x = hub_radius * math.cos(attach_angle)
    attach_y = hub_radius * math.sin(attach_angle)

    right_outer_top_to_bottom = sample_catmull_rom_spline_2d(
        [
            (attach_x, attach_y),
            (0.018, 0.017),
            (0.028, 0.018),
            (0.036, 0.015),
            (0.042, 0.008),
            (0.043, -0.001),
            (0.040, -0.010),
            (0.034, -0.017),
            (0.026, -0.020),
            (0.018, -0.019),
            (attach_x, -attach_y),
        ],
        samples_per_segment=10,
        closed=False,
    )
    left_outer_bottom_to_top = [(-x, -y) for x, y in right_outer_top_to_bottom]
    left_outer_top_to_bottom = list(reversed(left_outer_bottom_to_top))
    right_outer_bottom_to_top = list(reversed(right_outer_top_to_bottom))

    top_arc = _arc_points(
        hub_radius,
        attach_angle,
        math.pi - attach_angle,
        segments=18,
    )
    bottom_arc = _arc_points(
        hub_radius,
        math.pi + attach_angle,
        (2.0 * math.pi) - attach_angle,
        segments=18,
    )

    return (
        top_arc[:-1]
        + left_outer_top_to_bottom[:-1]
        + bottom_arc[:-1]
        + right_outer_bottom_to_top[:-1]
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bi_wing_fidget_spinner")

    body_metal = model.material("body_metal", rgba=(0.18, 0.20, 0.23, 1.0))
    cap_black = model.material("cap_black", rgba=(0.09, 0.09, 0.10, 1.0))
    axle_steel = model.material("axle_steel", rgba=(0.43, 0.45, 0.48, 1.0))
    weight_metal = model.material("weight_metal", rgba=(0.78, 0.79, 0.81, 1.0))

    body_total_thickness = 0.0072
    core_thickness = 0.0044
    skin_thickness = 0.0014
    insert_thickness = 0.0010
    axial_clearance = 0.00055

    cap_radius = 0.015
    cap_thickness = 0.0024
    axle_radius = 0.0052
    bore_radius = 0.0063

    cap_inner_gap = body_total_thickness + (2.0 * axial_clearance)
    cap_center_z = (cap_inner_gap * 0.5) + (cap_thickness * 0.5)
    axle_length = cap_inner_gap + (2.0 * cap_thickness)

    pocket_angle = -0.28
    pocket_center = (0.0285, -0.0040)
    outer_pocket = rounded_rect_profile(0.018, 0.011, 0.0036, corner_segments=8)
    inner_weight = rounded_rect_profile(0.0155, 0.0088, 0.0028, corner_segments=8)

    body_outline = _spinner_outline()
    bore_profile = _circle_profile(bore_radius, segments=36)
    right_pocket = _transform_profile(
        outer_pocket,
        dx=pocket_center[0],
        dy=pocket_center[1],
        angle=pocket_angle,
    )
    left_pocket = _transform_profile(
        outer_pocket,
        dx=-pocket_center[0],
        dy=-pocket_center[1],
        angle=pocket_angle,
    )

    core_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            body_outline,
            [bore_profile],
            core_thickness,
            center=True,
        ),
        "spinner_core",
    )
    skin_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            body_outline,
            [bore_profile, right_pocket, left_pocket],
            skin_thickness,
            center=True,
        ),
        "spinner_skin",
    )
    weight_mesh = mesh_from_geometry(
        ExtrudeGeometry(inner_weight, insert_thickness, center=True),
        "spinner_weight",
    )

    cap_stack = model.part("cap_stack")
    cap_stack.visual(
        Cylinder(radius=axle_radius, length=axle_length),
        material=axle_steel,
        name="axle",
    )
    cap_stack.visual(
        Cylinder(radius=cap_radius, length=cap_thickness),
        origin=Origin(xyz=(0.0, 0.0, cap_center_z)),
        material=cap_black,
        name="top_cap",
    )
    cap_stack.visual(
        Cylinder(radius=cap_radius, length=cap_thickness),
        origin=Origin(xyz=(0.0, 0.0, -cap_center_z)),
        material=cap_black,
        name="bottom_cap",
    )
    cap_stack.inertial = Inertial.from_geometry(
        Cylinder(radius=cap_radius, length=axle_length),
        mass=0.03,
    )

    wing_assembly = model.part("wing_assembly")
    wing_assembly.visual(
        core_mesh,
        material=body_metal,
        name="core",
    )
    wing_assembly.visual(
        skin_mesh,
        origin=Origin(xyz=(0.0, 0.0, (core_thickness + skin_thickness) * 0.5)),
        material=body_metal,
        name="top_skin",
    )
    wing_assembly.visual(
        skin_mesh,
        origin=Origin(xyz=(0.0, 0.0, -((core_thickness + skin_thickness) * 0.5))),
        material=body_metal,
        name="bottom_skin",
    )

    top_weight_z = (core_thickness * 0.5) + (insert_thickness * 0.5)
    bottom_weight_z = -top_weight_z
    for index, (x_pos, y_pos) in enumerate(
        (
            (pocket_center[0], pocket_center[1]),
            (-pocket_center[0], -pocket_center[1]),
        )
    ):
        wing_assembly.visual(
            weight_mesh,
            origin=Origin(xyz=(x_pos, y_pos, top_weight_z), rpy=(0.0, 0.0, pocket_angle)),
            material=weight_metal,
            name=f"top_weight_{index}",
        )
        wing_assembly.visual(
            weight_mesh,
            origin=Origin(xyz=(x_pos, y_pos, bottom_weight_z), rpy=(0.0, 0.0, pocket_angle)),
            material=weight_metal,
            name=f"bottom_weight_{index}",
        )
    wing_assembly.inertial = Inertial.from_geometry(
        Box((0.086, 0.040, body_total_thickness)),
        mass=0.085,
    )

    model.articulation(
        "hub_spin",
        ArticulationType.CONTINUOUS,
        parent=cap_stack,
        child=wing_assembly,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.25, velocity=45.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cap_stack = object_model.get_part("cap_stack")
    wing_assembly = object_model.get_part("wing_assembly")
    hub_spin = object_model.get_articulation("hub_spin")

    ctx.expect_origin_distance(
        wing_assembly,
        cap_stack,
        axes="xy",
        max_dist=1e-6,
        name="wing assembly remains centered on the hub axis",
    )
    ctx.expect_gap(
        cap_stack,
        wing_assembly,
        axis="z",
        positive_elem="top_cap",
        negative_elem="top_skin",
        min_gap=0.0003,
        max_gap=0.0010,
        name="top cap plate clears the spinning wing body",
    )
    ctx.expect_gap(
        wing_assembly,
        cap_stack,
        axis="z",
        positive_elem="bottom_skin",
        negative_elem="bottom_cap",
        min_gap=0.0003,
        max_gap=0.0010,
        name="bottom cap plate clears the spinning wing body",
    )

    rest_aabb = ctx.part_world_aabb(wing_assembly)
    with ctx.pose({hub_spin: math.pi * 0.5}):
        turned_aabb = ctx.part_world_aabb(wing_assembly)
        ctx.expect_gap(
            cap_stack,
            wing_assembly,
            axis="z",
            positive_elem="top_cap",
            negative_elem="top_skin",
            min_gap=0.0003,
            max_gap=0.0010,
            name="top cap clearance holds through quarter-turn spin",
        )
        ctx.expect_gap(
            wing_assembly,
            cap_stack,
            axis="z",
            positive_elem="bottom_skin",
            negative_elem="bottom_cap",
            min_gap=0.0003,
            max_gap=0.0010,
            name="bottom cap clearance holds through quarter-turn spin",
        )

    rest_dx = None if rest_aabb is None else rest_aabb[1][0] - rest_aabb[0][0]
    rest_dy = None if rest_aabb is None else rest_aabb[1][1] - rest_aabb[0][1]
    turned_dx = None if turned_aabb is None else turned_aabb[1][0] - turned_aabb[0][0]
    turned_dy = None if turned_aabb is None else turned_aabb[1][1] - turned_aabb[0][1]
    ctx.check(
        "quarter turn swaps the spinner planform span",
        rest_dx is not None
        and rest_dy is not None
        and turned_dx is not None
        and turned_dy is not None
        and rest_dx > rest_dy + 0.025
        and turned_dy > turned_dx + 0.025,
        details=(
            f"rest_dx={rest_dx}, rest_dy={rest_dy}, "
            f"turned_dx={turned_dx}, turned_dy={turned_dy}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
