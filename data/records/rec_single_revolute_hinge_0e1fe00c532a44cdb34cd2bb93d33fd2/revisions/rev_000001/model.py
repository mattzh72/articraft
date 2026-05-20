from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(radius: float, segments: int = 48) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * i / segments),
            radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _tab_profile(
    *,
    lug_radius: float,
    strap_half_width: float,
    total_length: float,
    arc_segments: int = 18,
) -> list[tuple[float, float]]:
    """Plan shape of the moving tab in local (Y, Z), later extruded along X."""

    tangent_y = math.sqrt(lug_radius * lug_radius - strap_half_width * strap_half_width)
    end_center_y = total_length - strap_half_width
    alpha = math.asin(strap_half_width / lug_radius)

    pts: list[tuple[float, float]] = []
    pts.append((tangent_y, -strap_half_width))
    pts.append((end_center_y, -strap_half_width))

    # Rounded working end of the strap.
    for i in range(1, arc_segments + 1):
        phi = -math.pi / 2.0 + math.pi * i / arc_segments
        pts.append(
            (
                end_center_y + strap_half_width * math.cos(phi),
                strap_half_width * math.sin(phi),
            )
        )

    pts.append((tangent_y, strap_half_width))

    # The large rear arc forms the circular eye around the hinge pin.
    rear_segments = 30
    for i in range(1, rear_segments + 1):
        theta = alpha + (2.0 * math.pi - 2.0 * alpha) * i / rear_segments
        pts.append((lug_radius * math.cos(theta), lug_radius * math.sin(theta)))

    return pts


def _extrude_yz_profile_along_x(
    outer_profile: list[tuple[float, float]],
    hole_profiles: list[list[tuple[float, float]]],
    thickness: float,
):
    """Create an extruded mesh whose profile coordinates are semantic (Y, Z)."""

    geom = ExtrudeWithHolesGeometry(
        outer_profile,
        hole_profiles,
        thickness,
        center=True,
        cap=True,
        closed=True,
    )
    # ExtrudeWithHolesGeometry emits profile-X/profile-Y/extrusion-Z.  Remap
    # those axes to assembly X/Y/Z so the plate thickness lies along the pin.
    geom.vertices = [(z, x, y) for (x, y, z) in geom.vertices]
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fork_and_tab_hinge")

    steel = model.material("brushed_steel", rgba=(0.58, 0.60, 0.61, 1.0))
    dark_steel = model.material("dark_bore_shadow", rgba=(0.08, 0.09, 0.10, 1.0))
    bronze = model.material("bronze_spacer", rgba=(0.78, 0.54, 0.23, 1.0))
    blue = model.material("blue_tab", rgba=(0.12, 0.28, 0.68, 1.0))

    # Assembly frame: X is the pin axis across the fork cheeks, Y is the
    # nominal tab direction at q=0, and Z is upward from the mounting base.
    base_size = (0.110, 0.076, 0.012)
    cheek_thickness = 0.029
    gap_width = 0.032
    cheek_y = 0.062
    cheek_height = 0.070
    hinge_z = 0.052
    cheek_center_x = gap_width / 2.0 + cheek_thickness / 2.0

    fork = model.part("fork")
    fork.visual(
        Box(base_size),
        origin=Origin(xyz=(0.0, -0.002, base_size[2] / 2.0)),
        material=steel,
        name="base_plate",
    )
    fork.visual(
        Box((cheek_thickness, cheek_y, cheek_height)),
        origin=Origin(
            xyz=(cheek_center_x, 0.0, base_size[2] + cheek_height / 2.0)
        ),
        material=steel,
        name="cheek_0",
    )
    fork.visual(
        Cylinder(radius=0.013, length=0.004),
        origin=Origin(
            xyz=(gap_width / 2.0 + 0.001, 0.0, hinge_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=dark_steel,
        name="bore_shadow_0",
    )
    fork.visual(
        Box((cheek_thickness, cheek_y, cheek_height)),
        origin=Origin(
            xyz=(-cheek_center_x, 0.0, base_size[2] + cheek_height / 2.0)
        ),
        material=steel,
        name="cheek_1",
    )
    fork.visual(
        Cylinder(radius=0.013, length=0.004),
        origin=Origin(
            xyz=(-(gap_width / 2.0 + 0.001), 0.0, hinge_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=dark_steel,
        name="bore_shadow_1",
    )

    fork.visual(
        Box((0.090, 0.008, 0.050)),
        origin=Origin(xyz=(0.0, -0.035, base_size[2] + 0.025)),
        material=steel,
        name="rear_bridge",
    )
    fork.visual(
        Cylinder(radius=0.0055, length=0.118),
        origin=Origin(xyz=(0.0, 0.0, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="pin_shaft",
    )
    for idx, sx in enumerate((1.0, -1.0)):
        fork.visual(
            Cylinder(radius=0.0085, length=0.006),
            origin=Origin(
                xyz=(sx * 0.062, 0.0, hinge_z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=dark_steel,
            name=f"pin_head_{idx}",
        )
        fork.visual(
            Cylinder(radius=0.0105, length=0.005),
            origin=Origin(
                xyz=(sx * 0.0122, 0.0, hinge_z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=bronze,
            name=f"spacer_{idx}",
        )

    tab_thickness = 0.018
    tab = model.part("tab")
    tab_mesh = _extrude_yz_profile_along_x(
        _tab_profile(lug_radius=0.022, strap_half_width=0.012, total_length=0.132),
        [_circle_profile(0.0074, segments=48)],
        tab_thickness,
    )
    tab.visual(
        mesh_from_geometry(tab_mesh, "moving_tab"),
        origin=Origin(),
        material=blue,
        name="tab_plate",
    )
    tab.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blue,
        name="tab_barrel_0",
    )
    tab.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(xyz=(-0.006, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blue,
        name="tab_barrel_1",
    )

    model.articulation(
        "fork_to_tab",
        ArticulationType.REVOLUTE,
        parent=fork,
        child=tab,
        origin=Origin(xyz=(0.0, 0.0, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.65),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fork = object_model.get_part("fork")
    tab = object_model.get_part("tab")
    hinge = object_model.get_articulation("fork_to_tab")

    ctx.allow_overlap(
        fork,
        tab,
        elem_a="pin_shaft",
        elem_b="tab_plate",
        reason="The through-pin is intentionally represented as captured inside the tab's simplified bore.",
    )
    ctx.allow_overlap(
        fork,
        tab,
        elem_a="pin_shaft",
        elem_b="tab_barrel_0",
        reason="The through-pin is intentionally nested inside the moving tab barrel segment.",
    )
    ctx.allow_overlap(
        fork,
        tab,
        elem_a="pin_shaft",
        elem_b="tab_barrel_1",
        reason="The through-pin is intentionally nested inside the moving tab barrel segment.",
    )

    ctx.check(
        "single supported revolute tab",
        hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(hinge.axis) == (1.0, 0.0, 0.0),
        details=f"type={hinge.articulation_type}, axis={hinge.axis}",
    )
    ctx.expect_gap(
        fork,
        tab,
        axis="x",
        positive_elem="cheek_0",
        negative_elem="tab_plate",
        min_gap=0.004,
        name="tab clears one fork cheek",
    )
    ctx.expect_gap(
        tab,
        fork,
        axis="x",
        positive_elem="tab_plate",
        negative_elem="cheek_1",
        min_gap=0.004,
        name="tab clears other fork cheek",
    )
    ctx.expect_overlap(
        tab,
        fork,
        axes="x",
        elem_a="tab_plate",
        elem_b="pin_shaft",
        min_overlap=0.016,
        name="through pin spans the moving tab",
    )
    ctx.expect_within(
        fork,
        tab,
        axes="yz",
        inner_elem="pin_shaft",
        outer_elem="tab_plate",
        margin=0.0,
        name="pin is captured inside the tab bore area",
    )

    rest_aabb = ctx.part_element_world_aabb(tab, elem="tab_plate")
    with ctx.pose({hinge: 1.20}):
        open_aabb = ctx.part_element_world_aabb(tab, elem="tab_plate")
        ctx.expect_gap(
            fork,
            tab,
            axis="x",
            positive_elem="cheek_0",
            negative_elem="tab_plate",
            min_gap=0.004,
            name="opened tab remains between cheek_0 and pin",
        )

    ctx.check(
        "tab swings upward about the pin",
        rest_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > rest_aabb[1][2] + 0.060,
        details=f"rest_aabb={rest_aabb}, open_aabb={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
