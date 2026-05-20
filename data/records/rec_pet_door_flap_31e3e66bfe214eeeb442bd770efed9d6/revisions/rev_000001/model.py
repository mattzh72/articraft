from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _shift_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _managed_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pet_door_with_trim_cover")

    frame_plastic = model.material("frame_plastic", rgba=(0.90, 0.91, 0.89, 1.0))
    frame_shadow = model.material("frame_shadow", rgba=(0.74, 0.76, 0.74, 1.0))
    flap_tint = model.material("flap_tint", rgba=(0.35, 0.39, 0.44, 0.62))
    handle_dark = model.material("handle_dark", rgba=(0.26, 0.27, 0.28, 1.0))

    outer_width = 0.300
    outer_height = 0.402
    outer_corner = 0.022

    opening_width = 0.212
    opening_height = 0.304
    opening_corner = 0.016

    tunnel_outer_width = 0.238
    tunnel_outer_height = 0.330
    tunnel_corner = 0.018

    inner_width = 0.286
    inner_height = 0.386
    inner_corner = 0.020

    outer_bezel_thickness = 0.010
    tunnel_depth = 0.045
    inner_frame_thickness = 0.008
    frame_depth = outer_bezel_thickness + tunnel_depth + inner_frame_thickness

    flap_width = 0.204
    flap_height = 0.294
    flap_corner = 0.012
    flap_thickness = 0.004
    flap_drop = 0.003
    flap_plane_z = 0.018
    flap_axis_z = flap_plane_z + flap_thickness * 0.5
    flap_hinge_y = opening_height * 0.5

    trim_cover_width = 0.274
    trim_cover_height = 0.372
    trim_cover_corner = 0.018
    trim_cover_thickness = 0.006
    trim_opening_width = 0.218
    trim_opening_height = 0.312
    trim_cover_plane_z = frame_depth + trim_cover_thickness * 0.5

    outer_frame = model.part("outer_frame")
    outer_frame.inertial = Inertial.from_geometry(
        Box((outer_width, outer_height, frame_depth)),
        mass=1.25,
        origin=Origin(xyz=(0.0, 0.0, frame_depth * 0.5)),
    )

    outer_bezel = ExtrudeWithHolesGeometry(
        rounded_rect_profile(outer_width, outer_height, outer_corner, corner_segments=8),
        [rounded_rect_profile(opening_width, opening_height, opening_corner, corner_segments=8)],
        height=outer_bezel_thickness,
        center=True,
    ).translate(0.0, 0.0, outer_bezel_thickness * 0.5)
    outer_frame.visual(
        _managed_mesh("pet_door_outer_bezel", outer_bezel),
        material=frame_plastic,
        name="elem_outer_bezel",
    )

    tunnel = ExtrudeWithHolesGeometry(
        rounded_rect_profile(tunnel_outer_width, tunnel_outer_height, tunnel_corner, corner_segments=8),
        [rounded_rect_profile(opening_width, opening_height, opening_corner, corner_segments=8)],
        height=tunnel_depth,
        center=True,
    ).translate(0.0, 0.0, outer_bezel_thickness + tunnel_depth * 0.5)
    outer_frame.visual(
        _managed_mesh("pet_door_tunnel", tunnel),
        material=frame_shadow,
        name="elem_tunnel",
    )

    inner_frame = ExtrudeWithHolesGeometry(
        rounded_rect_profile(inner_width, inner_height, inner_corner, corner_segments=8),
        [rounded_rect_profile(opening_width, opening_height, opening_corner, corner_segments=8)],
        height=inner_frame_thickness,
        center=True,
    ).translate(0.0, 0.0, outer_bezel_thickness + tunnel_depth + inner_frame_thickness * 0.5)
    outer_frame.visual(
        _managed_mesh("pet_door_inner_frame", inner_frame),
        material=frame_plastic,
        name="elem_inner_frame_plate",
    )
    outer_frame.visual(
        Box((0.120, 0.006, 0.006)),
        origin=Origin(xyz=(0.0, opening_height * 0.5 + 0.001, flap_plane_z)),
        material=frame_shadow,
        name="elem_flap_hinge_lug",
    )

    flap = model.part("flap")
    flap.inertial = Inertial.from_geometry(
        Box((flap_width, flap_height, flap_thickness)),
        mass=0.26,
        origin=Origin(xyz=(0.0, -(flap_height * 0.5 + flap_drop), 0.0)),
    )

    flap_profile = _shift_profile(
        rounded_rect_profile(flap_width, flap_height, flap_corner, corner_segments=8),
        dy=-(flap_height * 0.5 + flap_drop),
    )
    flap_panel = ExtrudeGeometry(
        flap_profile,
        flap_thickness,
        cap=True,
        center=True,
    )
    flap.visual(
        _managed_mesh("pet_door_flap_panel", flap_panel),
        origin=Origin(xyz=(0.0, 0.0, -flap_thickness * 0.5)),
        material=flap_tint,
        name="elem_flap_panel",
    )
    flap.visual(
        Box((0.118, 0.013, 0.004)),
        origin=Origin(xyz=(0.0, -0.0085, -0.003)),
        material=handle_dark,
        name="elem_flap_hinge_rail",
    )
    flap.visual(
        Box((0.090, 0.016, 0.008)),
        origin=Origin(xyz=(0.0, -flap_height + 0.020 - flap_drop, -flap_thickness * 0.5)),
        material=handle_dark,
        name="elem_flap_pull",
    )

    trim_cover = model.part("trim_cover")
    trim_cover.inertial = Inertial.from_geometry(
        Box((trim_cover_width, trim_cover_height, trim_cover_thickness)),
        mass=0.34,
        origin=Origin(xyz=(trim_cover_width * 0.5, 0.0, 0.0)),
    )

    trim_cover_profile = _shift_profile(
        rounded_rect_profile(trim_cover_width, trim_cover_height, trim_cover_corner, corner_segments=8),
        dx=trim_cover_width * 0.5,
    )
    trim_hole_profile = _shift_profile(
        rounded_rect_profile(trim_opening_width, trim_opening_height, 0.014, corner_segments=8),
        dx=trim_cover_width * 0.5,
    )
    trim_cover_mesh = ExtrudeWithHolesGeometry(
        trim_cover_profile,
        [trim_hole_profile],
        height=trim_cover_thickness,
        center=True,
    )
    trim_cover.visual(
        _managed_mesh("pet_door_trim_cover", trim_cover_mesh),
        material=frame_plastic,
        name="elem_trim_cover",
    )
    trim_cover.visual(
        Box((0.014, 0.338, 0.010)),
        origin=Origin(xyz=(0.007, 0.0, 0.002)),
        material=frame_shadow,
        name="elem_trim_hinge_stile",
    )
    trim_cover.visual(
        Box((0.014, 0.072, 0.014)),
        origin=Origin(xyz=(trim_cover_width + 0.003, 0.0, 0.004)),
        material=handle_dark,
        name="elem_trim_pull",
    )

    model.articulation(
        "flap_hinge",
        ArticulationType.REVOLUTE,
        parent=outer_frame,
        child=flap,
        origin=Origin(xyz=(0.0, flap_hinge_y, flap_axis_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=-1.20,
            upper=1.20,
        ),
    )

    model.articulation(
        "trim_cover_hinge",
        ArticulationType.REVOLUTE,
        parent=outer_frame,
        child=trim_cover,
        origin=Origin(xyz=(-trim_cover_width * 0.5, 0.0, trim_cover_plane_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=1.75,
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

    outer_frame = object_model.get_part("outer_frame")
    flap = object_model.get_part("flap")
    trim_cover = object_model.get_part("trim_cover")

    flap_hinge = object_model.get_articulation("flap_hinge")
    trim_cover_hinge = object_model.get_articulation("trim_cover_hinge")

    inner_frame_plate = outer_frame.get_visual("elem_inner_frame_plate")
    flap_panel = flap.get_visual("elem_flap_panel")
    trim_cover_panel = trim_cover.get_visual("elem_trim_cover")

    def _center_z(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return (lower[2] + upper[2]) * 0.5

    flap_limits = flap_hinge.motion_limits
    trim_limits = trim_cover_hinge.motion_limits

    ctx.check(
        "flap hinge uses horizontal top axis",
        flap_hinge.axis == (-1.0, 0.0, 0.0)
        and flap_limits is not None
        and flap_limits.lower is not None
        and flap_limits.upper is not None
        and flap_limits.lower < 0.0 < flap_limits.upper,
        details=f"axis={flap_hinge.axis}, limits={flap_limits}",
    )
    ctx.check(
        "trim cover hinge uses vertical side axis",
        trim_cover_hinge.axis == (0.0, -1.0, 0.0)
        and trim_limits is not None
        and trim_limits.lower == 0.0
        and trim_limits.upper is not None
        and trim_limits.upper >= 1.4,
        details=f"axis={trim_cover_hinge.axis}, limits={trim_limits}",
    )

    with ctx.pose({trim_cover_hinge: 0.0}):
        ctx.expect_gap(
            trim_cover,
            outer_frame,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=trim_cover_panel,
            negative_elem=inner_frame_plate,
            name="trim cover closes nearly flush to the inner frame",
        )
        ctx.expect_overlap(
            trim_cover,
            outer_frame,
            axes="xy",
            min_overlap=0.22,
            elem_a=trim_cover_panel,
            elem_b=inner_frame_plate,
            name="trim cover aligns over the interior frame opening",
        )

        flap_closed_aabb = ctx.part_element_world_aabb(flap, elem=flap_panel)
        trim_closed_aabb = ctx.part_element_world_aabb(trim_cover, elem=trim_cover_panel)

    with ctx.pose({flap_hinge: 0.90}):
        flap_open_inward_aabb = ctx.part_element_world_aabb(flap, elem=flap_panel)

    with ctx.pose({flap_hinge: -0.90}):
        flap_open_outward_aabb = ctx.part_element_world_aabb(flap, elem=flap_panel)

    with ctx.pose({trim_cover_hinge: 1.20}):
        trim_open_aabb = ctx.part_element_world_aabb(trim_cover, elem=trim_cover_panel)

    flap_closed_z = _center_z(flap_closed_aabb)
    flap_inward_z = _center_z(flap_open_inward_aabb)
    flap_outward_z = _center_z(flap_open_outward_aabb)
    trim_closed_z = _center_z(trim_closed_aabb)
    trim_open_z = _center_z(trim_open_aabb)

    ctx.check(
        "positive flap rotation swings the panel inward",
        flap_closed_z is not None
        and flap_inward_z is not None
        and flap_inward_z > flap_closed_z + 0.05,
        details=f"closed_z={flap_closed_z}, inward_z={flap_inward_z}",
    )
    ctx.check(
        "negative flap rotation swings the panel outward",
        flap_closed_z is not None
        and flap_outward_z is not None
        and flap_outward_z < flap_closed_z - 0.05,
        details=f"closed_z={flap_closed_z}, outward_z={flap_outward_z}",
    )
    ctx.check(
        "trim cover opens away from the inner frame",
        trim_closed_z is not None
        and trim_open_z is not None
        and trim_open_z > trim_closed_z + 0.05,
        details=f"closed_z={trim_closed_z}, open_z={trim_open_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
