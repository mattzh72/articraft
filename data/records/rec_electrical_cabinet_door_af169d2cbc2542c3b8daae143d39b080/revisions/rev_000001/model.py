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
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _aabb_center(aabb):
    if aabb is None:
        return None
    return tuple((aabb[0][index] + aabb[1][index]) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flameproof_junction_box")

    cast_alloy = model.material("cast_alloy", rgba=(0.50, 0.52, 0.54, 1.0))
    cover_alloy = model.material("cover_alloy", rgba=(0.58, 0.60, 0.62, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.20, 0.21, 0.23, 1.0))

    body_w = 0.360
    body_d = 0.240
    body_h = 0.120
    wall_t = 0.014
    floor_t = 0.016
    flange_t = 0.012
    flange_land = 0.012
    outer_r = 0.028
    inner_r = 0.014
    cover_overhang = 0.012
    cover_t = 0.018
    hinge_axis_y = -(body_d * 0.5 + cover_overhang)
    hinge_axis_z = body_h
    cover_panel_start_y = 0.014
    cover_panel_depth = body_d + 2.0 * cover_overhang - cover_panel_start_y
    cover_panel_center_y = cover_panel_start_y + cover_panel_depth * 0.5
    cover_w = body_w + 2.0 * cover_overhang
    cover_r = outer_r + 0.006
    ring_boss_y = cover_panel_start_y + cover_panel_depth - 0.030
    hinge_radius = 0.012

    body_outer = rounded_rect_profile(body_w, body_d, outer_r, corner_segments=8)
    body_inner = rounded_rect_profile(
        body_w - 2.0 * wall_t,
        body_d - 2.0 * wall_t,
        inner_r,
        corner_segments=8,
    )
    body_flange_inner = rounded_rect_profile(
        body_w - 2.0 * (wall_t + flange_land),
        body_d - 2.0 * (wall_t + flange_land),
        0.008,
        corner_segments=8,
    )
    cover_outer = rounded_rect_profile(cover_w, cover_panel_depth, cover_r, corner_segments=8)
    cover_rib_outer = rounded_rect_profile(
        cover_w - 0.042,
        cover_panel_depth - 0.040,
        cover_r - 0.010,
        corner_segments=8,
    )
    cover_rib_inner = rounded_rect_profile(
        cover_w - 0.080,
        cover_panel_depth - 0.078,
        max(cover_r - 0.022, 0.010),
        corner_segments=8,
    )

    body = model.part("body")
    body.visual(
        _mesh(
            "body_wall_ring",
            ExtrudeWithHolesGeometry(body_outer, [body_inner], body_h, center=True),
        ),
        origin=Origin(xyz=(0.0, 0.0, body_h * 0.5)),
        material=cast_alloy,
        name="wall_shell",
    )
    body.visual(
        _mesh("body_floor", ExtrudeGeometry(body_outer, floor_t, center=True)),
        origin=Origin(xyz=(0.0, 0.0, floor_t * 0.5)),
        material=cast_alloy,
        name="body_floor",
    )
    body.visual(
        _mesh(
            "body_top_flange",
            ExtrudeWithHolesGeometry(body_outer, [body_flange_inner], flange_t, center=True),
        ),
        origin=Origin(xyz=(0.0, 0.0, body_h - flange_t * 0.5)),
        material=cast_alloy,
        name="top_flange",
    )
    body.visual(
        Box((0.070, 0.020, 0.050)),
        origin=Origin(xyz=(0.0, body_d * 0.5 + 0.006, body_h - 0.025)),
        material=cast_alloy,
        name="front_receiver_pad",
    )
    for side_name, x_pos in (("left", -(body_w * 0.5 + 0.008)), ("right", body_w * 0.5 + 0.008)):
        body.visual(
            Cylinder(radius=0.028, length=0.024),
            origin=Origin(xyz=(x_pos, 0.0, 0.062), rpy=(0.0, math.pi * 0.5, 0.0)),
            material=cast_alloy,
            name=f"{side_name}_conduit_boss",
        )
    for index, x_pos in enumerate((-0.135, 0.0, 0.135)):
        body.visual(
            Cylinder(radius=hinge_radius, length=0.060),
            origin=Origin(
                xyz=(x_pos, hinge_axis_y, hinge_axis_z + 0.006),
                rpy=(0.0, math.pi * 0.5, 0.0),
            ),
            material=dark_steel,
            name=f"body_hinge_knuckle_{index}",
        )
        body.visual(
            Box((0.060, 0.012, 0.012)),
            origin=Origin(xyz=(x_pos, hinge_axis_y + 0.008, hinge_axis_z + 0.006)),
            material=cast_alloy,
            name=f"body_hinge_pad_{index}",
        )
    body.inertial = Inertial.from_geometry(
        Box((body_w + 0.060, body_d + 0.024, body_h)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, body_h * 0.5)),
    )

    cover = model.part("cover")
    cover.visual(
        _mesh("cover_plate", ExtrudeGeometry(cover_outer, cover_t, center=True)),
        origin=Origin(xyz=(0.0, cover_panel_center_y, cover_t * 0.5)),
        material=cover_alloy,
        name="cover_plate",
    )
    cover.visual(
        _mesh(
            "cover_perimeter_rib",
            ExtrudeWithHolesGeometry(cover_rib_outer, [cover_rib_inner], 0.006, center=True),
        ),
        origin=Origin(xyz=(0.0, cover_panel_center_y, cover_t + 0.003)),
        material=cover_alloy,
        name="cover_rib",
    )
    cover.visual(
        Box((0.180, 0.110, 0.008)),
        origin=Origin(xyz=(0.0, cover_panel_center_y, cover_t + 0.004)),
        material=cover_alloy,
        name="center_stiffener",
    )
    cover.visual(
        Box((0.090, 0.045, 0.008)),
        origin=Origin(xyz=(0.0, ring_boss_y - 0.008, cover_t + 0.004)),
        material=cover_alloy,
        name="bolt_pad",
    )
    cover.visual(
        Cylinder(radius=0.022, length=0.010),
        origin=Origin(xyz=(0.0, ring_boss_y, cover_t + 0.005)),
        material=dark_steel,
        name="ring_boss",
    )
    for index, x_pos in enumerate((-0.0675, 0.0675)):
        cover.visual(
            Cylinder(radius=hinge_radius, length=0.075),
            origin=Origin(xyz=(x_pos, 0.0, 0.006), rpy=(0.0, math.pi * 0.5, 0.0)),
            material=dark_steel,
            name=f"cover_hinge_knuckle_{index}",
        )
        cover.visual(
            Box((0.075, 0.012, 0.012)),
            origin=Origin(xyz=(x_pos, 0.018, 0.006)),
            material=cover_alloy,
            name=f"cover_hinge_pad_{index}",
        )
    cover.inertial = Inertial.from_geometry(
        Box((cover_w, cover_panel_depth, cover_t + 0.014)),
        mass=5.0,
        origin=Origin(xyz=(0.0, cover_panel_center_y, 0.016)),
    )

    bolt_ring = model.part("bolt_ring")
    bolt_ring.visual(
        Cylinder(radius=0.016, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=dark_steel,
        name="retainer_washer",
    )
    bolt_ring.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=dark_steel,
        name="ring_collar",
    )
    bolt_ring.visual(
        _mesh(
            "captive_bolt_ring",
            TorusGeometry(radius=0.018, tube=0.003, radial_segments=18, tubular_segments=40),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=dark_steel,
        name="ring_loop",
    )
    bolt_ring.visual(
        Box((0.018, 0.006, 0.008)),
        origin=Origin(xyz=(0.009, 0.0, 0.013)),
        material=dark_steel,
        name="ring_spoke",
    )
    bolt_ring.visual(
        Box((0.012, 0.007, 0.004)),
        origin=Origin(xyz=(0.025, 0.0, 0.017)),
        material=dark_steel,
        name="thumb_tab",
    )
    bolt_ring.inertial = Inertial.from_geometry(
        Box((0.052, 0.040, 0.022)),
        mass=0.3,
        origin=Origin(xyz=(0.010, 0.0, 0.011)),
    )

    model.articulation(
        "body_to_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cover,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.0,
            lower=0.0,
            upper=math.radians(115.0),
        ),
    )
    model.articulation(
        "cover_to_bolt_ring",
        ArticulationType.REVOLUTE,
        parent=cover,
        child=bolt_ring,
        origin=Origin(xyz=(0.0, ring_boss_y, cover_t + 0.010)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=4.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    cover = object_model.get_part("cover")
    bolt_ring = object_model.get_part("bolt_ring")
    cover_hinge = object_model.get_articulation("body_to_cover")
    ring_joint = object_model.get_articulation("cover_to_bolt_ring")

    ctx.expect_gap(
        cover,
        body,
        axis="z",
        positive_elem="cover_plate",
        negative_elem="top_flange",
        min_gap=0.0,
        max_gap=0.002,
        name="cover seats tightly on the flamepath flange",
    )
    ctx.expect_overlap(
        cover,
        body,
        axes="xy",
        elem_a="cover_plate",
        elem_b="top_flange",
        min_overlap=0.220,
        name="cover spans the body opening",
    )
    ctx.expect_gap(
        bolt_ring,
        cover,
        axis="z",
        positive_elem="retainer_washer",
        negative_elem="ring_boss",
        min_gap=0.0,
        max_gap=0.002,
        name="captive bolt ring is seated on its boss",
    )

    closed_boss = ctx.part_element_world_aabb(cover, elem="ring_boss")
    with ctx.pose({cover_hinge: math.radians(80.0)}):
        open_boss = ctx.part_element_world_aabb(cover, elem="ring_boss")
    closed_boss_center = _aabb_center(closed_boss)
    open_boss_center = _aabb_center(open_boss)
    ctx.check(
        "hinged cover lifts from the free edge",
        closed_boss_center is not None
        and open_boss_center is not None
        and open_boss_center[2] > closed_boss_center[2] + 0.10,
        details=f"closed={closed_boss_center}, open={open_boss_center}",
    )

    rest_tab = ctx.part_element_world_aabb(bolt_ring, elem="thumb_tab")
    with ctx.pose({ring_joint: math.pi * 0.5}):
        turned_tab = ctx.part_element_world_aabb(bolt_ring, elem="thumb_tab")
    rest_tab_center = _aabb_center(rest_tab)
    turned_tab_center = _aabb_center(turned_tab)
    ctx.check(
        "bolt ring rotates on its captive spindle",
        rest_tab_center is not None
        and turned_tab_center is not None
        and abs(turned_tab_center[1] - rest_tab_center[1]) > 0.015,
        details=f"rest={rest_tab_center}, turned={turned_tab_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
