from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(filename: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def _circle_profile(radius: float, *, segments: int = 72) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _annulus_mesh(
    filename: str,
    *,
    outer_radius: float,
    inner_radius: float,
    thickness: float,
    segments: int = 72,
):
    geom = ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius, segments=segments),
        [_circle_profile(inner_radius, segments=segments)],
        thickness,
        cap=True,
        center=True,
        closed=True,
    )
    geom.rotate_y(math.pi / 2.0)
    return _save_mesh(filename, geom)


def _tube_shell_mesh(
    filename: str,
    *,
    outer_radius: float,
    inner_radius: float,
    height: float,
    radial_segments: int = 56,
):
    outer = CylinderGeometry(radius=outer_radius, height=height, radial_segments=radial_segments)
    inner = CylinderGeometry(
        radius=inner_radius,
        height=height + 0.004,
        radial_segments=radial_segments,
    )
    shell = boolean_difference(outer, inner).translate(0.0, 0.0, height * 0.5)
    return _save_mesh(filename, shell)


def _merged_mesh(filename: str, geometries: list):
    merged = geometries[0].copy()
    for geometry in geometries[1:]:
        merged.merge(geometry)
    return _save_mesh(filename, merged)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_ring_light", assets=ASSETS)

    matte_black = model.material("matte_black", rgba=(0.10, 0.11, 0.12, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.08, 0.08, 0.09, 1.0))
    satin_graphite = model.material("satin_graphite", rgba=(0.30, 0.33, 0.36, 1.0))
    powder_white = model.material("powder_white", rgba=(0.93, 0.94, 0.95, 1.0))
    diffuser_white = model.material("diffuser_white", rgba=(0.97, 0.97, 0.96, 1.0))

    sleeve_mesh = _tube_shell_mesh(
        "ring_light_mast_sleeve.obj",
        outer_radius=0.0195,
        inner_radius=0.0155,
        height=0.220,
    )
    ring_body_mesh = _annulus_mesh(
        "ring_light_head_body.obj",
        outer_radius=0.125,
        inner_radius=0.088,
        thickness=0.028,
    )
    diffuser_mesh = _annulus_mesh(
        "ring_light_diffuser.obj",
        outer_radius=0.123,
        inner_radius=0.091,
        thickness=0.006,
    )

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.095, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=matte_black,
        name="weighted_base",
    )
    base.visual(
        Cylinder(radius=0.078, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=rubber_black,
        name="base_foot_pad",
    )
    base.visual(
        Cylinder(radius=0.028, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        material=satin_graphite,
        name="mast_collar",
    )
    base.visual(
        sleeve_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=satin_graphite,
        name="mast_sleeve",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.095, length=0.034),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
    )

    mast_slider = model.part("mast_slider")
    mast_slider.visual(
        Cylinder(radius=0.013, length=0.240),
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        material=satin_graphite,
        name="slider_tube",
    )
    mast_slider.visual(
        Box((0.140, 0.020, 0.020)),
        origin=Origin(xyz=(0.070, 0.0, 0.240)),
        material=matte_black,
        name="forward_beam",
    )
    mast_slider.visual(
        Box((0.020, 0.130, 0.020)),
        origin=Origin(xyz=(0.140, 0.065, 0.240)),
        material=matte_black,
        name="left_yoke_connector",
    )
    mast_slider.visual(
        Box((0.020, 0.130, 0.020)),
        origin=Origin(xyz=(0.140, -0.065, 0.240)),
        material=matte_black,
        name="right_yoke_connector",
    )
    mast_slider.visual(
        Box((0.020, 0.020, 0.130)),
        origin=Origin(xyz=(0.140, 0.130, 0.315)),
        material=matte_black,
        name="left_yoke_arm",
    )
    mast_slider.visual(
        Box((0.020, 0.020, 0.130)),
        origin=Origin(xyz=(0.140, -0.130, 0.315)),
        material=matte_black,
        name="right_yoke_arm",
    )
    mast_slider.visual(
        Box((0.020, 0.020, 0.020)),
        origin=Origin(xyz=(0.140, 0.130, 0.390)),
        material=matte_black,
        name="left_yoke_tip",
    )
    mast_slider.visual(
        Box((0.020, 0.020, 0.020)),
        origin=Origin(xyz=(0.140, -0.130, 0.390)),
        material=matte_black,
        name="right_yoke_tip",
    )
    mast_slider.inertial = Inertial.from_geometry(
        Box((0.180, 0.300, 0.410)),
        mass=0.65,
        origin=Origin(xyz=(0.070, 0.0, 0.205)),
    )

    head = model.part("head")
    head.visual(
        ring_body_mesh,
        material=powder_white,
        name="ring_body",
    )
    head.visual(
        diffuser_mesh,
        origin=Origin(xyz=(0.011, 0.0, 0.0)),
        material=diffuser_white,
        name="diffuser_ring",
    )
    head.visual(
        Cylinder(radius=0.006, length=0.276),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="pivot_shaft",
    )
    head.visual(
        Cylinder(radius=0.018, length=0.060),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="hub_barrel",
    )
    head.visual(
        Box((0.018, 0.020, 0.072)),
        origin=Origin(xyz=(0.0, 0.0, 0.054)),
        material=matte_black,
        name="upper_spoke",
    )
    head.visual(
        Box((0.018, 0.020, 0.072)),
        origin=Origin(xyz=(0.0, 0.0, -0.054)),
        material=matte_black,
        name="lower_spoke",
    )
    head.inertial = Inertial.from_geometry(
        Cylinder(radius=0.125, length=0.034),
        mass=0.9,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "mast_extension",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mast_slider,
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.20,
            lower=0.0,
            upper=0.150,
        ),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=mast_slider,
        child=head,
        origin=Origin(xyz=(0.140, 0.0, 0.390)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=-math.pi / 4.0,
            upper=math.pi / 4.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    mast_slider = object_model.get_part("mast_slider")
    head = object_model.get_part("head")

    mast_extension = object_model.get_articulation("mast_extension")
    head_tilt = object_model.get_articulation("head_tilt")

    mast_sleeve = base.get_visual("mast_sleeve")
    slider_tube = mast_slider.get_visual("slider_tube")
    forward_beam = mast_slider.get_visual("forward_beam")
    left_yoke_connector = mast_slider.get_visual("left_yoke_connector")
    right_yoke_connector = mast_slider.get_visual("right_yoke_connector")
    left_yoke_arm = mast_slider.get_visual("left_yoke_arm")
    right_yoke_arm = mast_slider.get_visual("right_yoke_arm")
    left_yoke_tip = mast_slider.get_visual("left_yoke_tip")
    right_yoke_tip = mast_slider.get_visual("right_yoke_tip")
    ring_body = head.get_visual("ring_body")
    pivot_shaft = head.get_visual("pivot_shaft")
    hub_barrel = head.get_visual("hub_barrel")
    upper_spoke = head.get_visual("upper_spoke")
    lower_spoke = head.get_visual("lower_spoke")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
    ctx.allow_overlap(
        head,
        mast_slider,
        elem_a=pivot_shaft,
        elem_b=left_yoke_tip,
        reason="Left hinge pin runs inside the left yoke tip as a pivot bushing.",
    )
    ctx.allow_overlap(
        head,
        mast_slider,
        elem_a=pivot_shaft,
        elem_b=right_yoke_tip,
        reason="Right hinge pin runs inside the right yoke tip as a pivot bushing.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=32)

    ctx.expect_origin_distance(
        mast_slider,
        base,
        axes="xy",
        max_dist=0.04,
        name="mast_slider_stays_near_base_centerline",
    )
    ctx.expect_within(
        mast_slider,
        base,
        axes="xy",
        inner_elem=slider_tube,
        outer_elem=mast_sleeve,
        margin=0.004,
        name="slider_tube_stays_within_sleeve_projection",
    )
    ctx.expect_overlap(
        mast_slider,
        base,
        axes="xy",
        elem_a=slider_tube,
        elem_b=mast_sleeve,
        min_overlap=0.024,
        name="slider_and_sleeve_share_telescoping_footprint",
    )
    ctx.expect_origin_gap(
        head,
        base,
        axis="z",
        min_gap=0.418,
        max_gap=0.426,
        name="head_sits_above_weighted_base",
    )
    ctx.expect_overlap(
        head,
        mast_slider,
        axes="xz",
        elem_a=pivot_shaft,
        elem_b=left_yoke_tip,
        min_overlap=0.010,
        name="left_yoke_tip_aligned_with_pivot_axis",
    )
    ctx.expect_overlap(
        head,
        mast_slider,
        axes="xz",
        elem_a=pivot_shaft,
        elem_b=right_yoke_tip,
        min_overlap=0.010,
        name="right_yoke_tip_aligned_with_pivot_axis",
    )
    ctx.expect_contact(
        mast_slider,
        mast_slider,
        elem_a=slider_tube,
        elem_b=forward_beam,
        name="forward_beam_attaches_to_slider_tube",
    )
    ctx.expect_contact(
        mast_slider,
        mast_slider,
        elem_a=forward_beam,
        elem_b=left_yoke_connector,
        name="left_yoke_connector_attaches_to_forward_beam",
    )
    ctx.expect_contact(
        mast_slider,
        mast_slider,
        elem_a=forward_beam,
        elem_b=right_yoke_connector,
        name="right_yoke_connector_attaches_to_forward_beam",
    )
    ctx.expect_contact(
        mast_slider,
        mast_slider,
        elem_a=left_yoke_connector,
        elem_b=left_yoke_arm,
        name="left_yoke_arm_attaches_to_left_connector",
    )
    ctx.expect_contact(
        mast_slider,
        mast_slider,
        elem_a=right_yoke_connector,
        elem_b=right_yoke_arm,
        name="right_yoke_arm_attaches_to_right_connector",
    )
    ctx.expect_contact(
        mast_slider,
        mast_slider,
        elem_a=left_yoke_arm,
        elem_b=left_yoke_tip,
        name="left_yoke_tip_attaches_to_left_yoke_arm",
    )
    ctx.expect_contact(
        mast_slider,
        mast_slider,
        elem_a=right_yoke_arm,
        elem_b=right_yoke_tip,
        name="right_yoke_tip_attaches_to_right_yoke_arm",
    )
    ctx.expect_contact(
        head,
        head,
        elem_a=hub_barrel,
        elem_b=upper_spoke,
        name="upper_spoke_attaches_to_hub_barrel",
    )
    ctx.expect_contact(
        head,
        head,
        elem_a=hub_barrel,
        elem_b=lower_spoke,
        name="lower_spoke_attaches_to_hub_barrel",
    )
    ctx.expect_contact(
        head,
        head,
        elem_a=hub_barrel,
        elem_b=pivot_shaft,
        name="pivot_shaft_passes_through_hub_barrel",
    )

    rest_head_pos = ctx.part_world_position(head)
    rest_ring_aabb = ctx.part_element_world_aabb(head, elem=ring_body)

    with ctx.pose({mast_extension: 0.150}):
        extended_head_pos = ctx.part_world_position(head)
        ctx.expect_origin_distance(
            mast_slider,
            base,
            axes="xy",
            max_dist=0.04,
            name="extended_mast_stays_near_centerline",
        )
        ctx.expect_within(
            mast_slider,
            base,
            axes="xy",
            inner_elem=slider_tube,
            outer_elem=mast_sleeve,
            margin=0.004,
            name="extended_slider_stays_within_sleeve_projection",
        )
        ctx.expect_origin_gap(
            head,
            base,
            axis="z",
            min_gap=0.568,
            max_gap=0.576,
            name="extended_head_raises_above_base",
        )

    if rest_head_pos is not None and extended_head_pos is not None:
        dz = extended_head_pos[2] - rest_head_pos[2]
        dx = abs(extended_head_pos[0] - rest_head_pos[0])
        dy = abs(extended_head_pos[1] - rest_head_pos[1])
        ctx.check(
            "mast_extension_travels_full_150mm",
            abs(dz - 0.150) <= 0.002 and dx <= 1e-6 and dy <= 1e-6,
            details=(
                f"expected 0.150 m vertical travel with no lateral drift; "
                f"got dz={dz:.4f}, dx={dx:.6f}, dy={dy:.6f}"
            ),
        )
    else:
        ctx.fail("mast_extension_travels_full_150mm", "Could not measure head positions.")

    with ctx.pose({head_tilt: math.pi / 4.0}):
        pos_tilt_ring_aabb = ctx.part_element_world_aabb(head, elem=ring_body)
        ctx.expect_overlap(
            head,
            mast_slider,
            elem_a=pivot_shaft,
            elem_b=left_yoke_tip,
            axes="xz",
            min_overlap=0.008,
            name="positive_tilt_pivot_stays_in_left_yoke_tip",
        )
        ctx.expect_overlap(
            head,
            mast_slider,
            elem_a=pivot_shaft,
            elem_b=right_yoke_tip,
            axes="xz",
            min_overlap=0.008,
            name="positive_tilt_pivot_stays_in_right_yoke_tip",
        )

    with ctx.pose({head_tilt: -math.pi / 4.0}):
        neg_tilt_ring_aabb = ctx.part_element_world_aabb(head, elem=ring_body)
        ctx.expect_overlap(
            head,
            mast_slider,
            elem_a=pivot_shaft,
            elem_b=left_yoke_tip,
            axes="xz",
            min_overlap=0.008,
            name="negative_tilt_pivot_stays_in_left_yoke_tip",
        )
        ctx.expect_overlap(
            head,
            mast_slider,
            elem_a=pivot_shaft,
            elem_b=right_yoke_tip,
            axes="xz",
            min_overlap=0.008,
            name="negative_tilt_pivot_stays_in_right_yoke_tip",
        )

    if (
        rest_ring_aabb is not None
        and pos_tilt_ring_aabb is not None
        and neg_tilt_ring_aabb is not None
    ):
        rest_dx = rest_ring_aabb[1][0] - rest_ring_aabb[0][0]
        rest_dy = rest_ring_aabb[1][1] - rest_ring_aabb[0][1]
        pos_dx = pos_tilt_ring_aabb[1][0] - pos_tilt_ring_aabb[0][0]
        pos_dy = pos_tilt_ring_aabb[1][1] - pos_tilt_ring_aabb[0][1]
        neg_dx = neg_tilt_ring_aabb[1][0] - neg_tilt_ring_aabb[0][0]
        neg_dy = neg_tilt_ring_aabb[1][1] - neg_tilt_ring_aabb[0][1]
        ctx.check(
            "head_tilt_uses_horizontal_axis",
            (
                pos_dx >= rest_dx + 0.12
                and neg_dx >= rest_dx + 0.12
                and abs(pos_dy - rest_dy) <= 0.004
                and abs(neg_dy - rest_dy) <= 0.004
            ),
            details=(
                f"expected tilt to increase x-depth while keeping y-span nearly fixed; "
                f"rest_dx={rest_dx:.4f}, pos_dx={pos_dx:.4f}, neg_dx={neg_dx:.4f}, "
                f"rest_dy={rest_dy:.4f}, pos_dy={pos_dy:.4f}, neg_dy={neg_dy:.4f}"
            ),
        )
    else:
        ctx.fail("head_tilt_uses_horizontal_axis", "Could not measure ring-body AABBs.")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
