from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_side_loft,
    tube_from_spline_points,
)


def _material(model: ArticulatedObject, name: str, rgba: tuple[float, float, float, float]) -> Material:
    return model.material(name, rgba=rgba)


def _flat_beater_geometry():
    """A connected wire-like paddle beater in the hub frame."""
    outer = tube_from_spline_points(
        [
            (-0.052, 0.0, -0.030),
            (-0.064, 0.0, -0.074),
            (-0.055, 0.0, -0.118),
            (0.0, 0.0, -0.146),
            (0.055, 0.0, -0.118),
            (0.064, 0.0, -0.074),
            (0.052, 0.0, -0.030),
            (0.0, 0.0, -0.018),
        ],
        radius=0.0042,
        samples_per_segment=10,
        closed_spline=True,
        radial_segments=14,
        cap_ends=False,
        up_hint=(0.0, 1.0, 0.0),
    )
    center_rib = tube_from_spline_points(
        [(0.0, 0.0, -0.012), (0.0, 0.0, -0.072), (0.0, 0.0, -0.143)],
        radius=0.0032,
        samples_per_segment=12,
        radial_segments=12,
        cap_ends=True,
        up_hint=(0.0, 1.0, 0.0),
    )
    cross_rib = tube_from_spline_points(
        [(-0.045, 0.0, -0.084), (0.0, 0.0, -0.070), (0.045, 0.0, -0.084)],
        radius=0.0030,
        samples_per_segment=12,
        radial_segments=12,
        cap_ends=True,
        up_hint=(0.0, 1.0, 0.0),
    )
    return outer.merge(center_rib).merge(cross_rib)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stand_mixer_blending")

    enamel = _material(model, "gloss_deep_red_enamel", (0.62, 0.035, 0.028, 1.0))
    enamel_dark = _material(model, "shadowed_red_recess", (0.36, 0.018, 0.018, 1.0))
    stainless = _material(model, "brushed_stainless_steel", (0.78, 0.78, 0.74, 1.0))
    dark = _material(model, "matte_black_rubber", (0.012, 0.012, 0.014, 1.0))
    polished = _material(model, "polished_hub_metal", (0.88, 0.86, 0.80, 1.0))

    base = model.part("base")
    platform = ExtrudeGeometry.from_z0(
        rounded_rect_profile(0.430, 0.330, 0.055, corner_segments=10),
        0.075,
    ).translate(0.0, -0.035, 0.0)
    base.visual(mesh_from_geometry(platform, "heavy_rounded_base"), material=enamel, name="base_shell")

    base.visual(
        Cylinder(radius=0.126, length=0.018),
        origin=Origin(xyz=(0.0, -0.100, 0.084)),
        material=enamel_dark,
        name="bowl_lock_plate",
    )
    base.visual(
        Cylinder(radius=0.101, length=0.004),
        origin=Origin(xyz=(0.0, -0.100, 0.095)),
        material=dark,
        name="twist_slot_ring",
    )
    base.visual(
        Box((0.112, 0.014, 0.004)),
        origin=Origin(xyz=(0.0, -0.040, 0.098)),
        material=dark,
        name="front_lock_slot",
    )
    base.visual(
        Box((0.112, 0.014, 0.004)),
        origin=Origin(xyz=(0.0, -0.160, 0.098)),
        material=dark,
        name="rear_lock_slot",
    )
    column = ExtrudeGeometry.from_z0(
        rounded_rect_profile(0.182, 0.116, 0.032, corner_segments=10),
        0.192,
    ).translate(0.0, 0.094, 0.068)
    base.visual(mesh_from_geometry(column, "rear_column"), material=enamel, name="rear_column")
    base.visual(
        Box((0.225, 0.036, 0.024)),
        origin=Origin(xyz=(0.0, 0.135, 0.260)),
        material=enamel,
        name="pivot_bridge",
    )
    for x, name in ((-0.112, "hinge_cheek_0"), (0.112, "hinge_cheek_1")):
        base.visual(
            Box((0.044, 0.076, 0.146)),
            origin=Origin(xyz=(x, 0.094, 0.323)),
            material=enamel,
            name=name,
        )
    for x, y, name in (
        (-0.170, -0.170, "foot_0"),
        (0.170, -0.170, "foot_1"),
        (-0.160, 0.115, "foot_2"),
        (0.160, 0.115, "foot_3"),
    ):
        base.visual(
            Cylinder(radius=0.032, length=0.012),
            origin=Origin(xyz=(x, y, -0.006)),
            material=dark,
            name=name,
        )

    bowl = model.part("bowl")
    bowl_shell = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.066, 0.000),
            (0.078, 0.016),
            (0.092, 0.050),
            (0.114, 0.148),
            (0.124, 0.180),
        ],
        inner_profile=[
            (0.044, 0.012),
            (0.066, 0.028),
            (0.085, 0.058),
            (0.106, 0.147),
            (0.114, 0.170),
        ],
        segments=80,
        start_cap="flat",
        end_cap="round",
        lip_samples=10,
    )
    bowl.visual(mesh_from_geometry(bowl_shell, "open_cylindrical_bowl"), material=stainless, name="bowl_shell")
    for x, name in ((0.095, "lock_tab_0"), (-0.095, "lock_tab_1")):
        bowl.visual(
            Box((0.044, 0.030, 0.014)),
            origin=Origin(xyz=(x, 0.0, 0.010)),
            material=polished,
            name=name,
        )

    head = model.part("mixing_head")
    head_shell = superellipse_side_loft(
        [
            (0.030, -0.030, 0.050, 0.118),
            (-0.035, -0.055, 0.086, 0.155),
            (-0.145, -0.061, 0.078, 0.236),
            (-0.238, -0.040, 0.050, 0.148),
        ],
        exponents=2.7,
        segments=72,
    )
    head.visual(mesh_from_geometry(head_shell, "tilting_head_shell"), material=enamel, name="head_shell")
    head.visual(
        Cylinder(radius=0.025, length=0.150),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished,
        name="hinge_bushing",
    )
    head.visual(
        Cylinder(radius=0.045, length=0.018),
        origin=Origin(xyz=(0.0, -0.244, 0.006), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished,
        name="front_attachment_cap",
    )
    head.visual(
        Box((0.150, 0.008, 0.014)),
        origin=Origin(xyz=(0.0, -0.145, 0.084)),
        material=polished,
        name="trim_band",
    )
    head.visual(
        Box((0.010, 0.060, 0.038)),
        origin=Origin(xyz=(0.112, -0.115, 0.030)),
        material=dark,
        name="speed_slot_plate",
    )

    speed_lever = model.part("speed_lever")
    speed_lever.visual(
        Cylinder(radius=0.006, length=0.018),
        origin=Origin(xyz=(0.009, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished,
        name="lever_stem",
    )
    speed_lever.visual(
        Box((0.014, 0.032, 0.020)),
        origin=Origin(xyz=(0.025, 0.0, 0.0)),
        material=dark,
        name="lever_paddle",
    )

    beater_hub = model.part("beater_hub")
    beater_hub.visual(
        Cylinder(radius=0.034, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=polished,
        name="hub_coupler",
    )
    beater_hub.visual(
        Cylinder(radius=0.010, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, -0.046)),
        material=polished,
        name="drive_shaft",
    )
    beater_hub.visual(
        mesh_from_geometry(_flat_beater_geometry(), "flat_beater_wire"),
        material=polished,
        name="flat_beater",
    )

    model.articulation(
        "bowl_twist_lock",
        ArticulationType.REVOLUTE,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(0.0, -0.100, 0.093)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.0, lower=-0.26, upper=0.26),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(0.0, 0.094, 0.334)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.7, lower=0.0, upper=0.72),
    )
    model.articulation(
        "speed_slider",
        ArticulationType.PRISMATIC,
        parent=head,
        child=speed_lever,
        origin=Origin(xyz=(0.116, -0.115, 0.030)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=0.08, lower=0.0, upper=0.045),
    )
    model.articulation(
        "beater_spin",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=beater_hub,
        origin=Origin(xyz=(0.0, -0.194, -0.066)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    bowl = object_model.get_part("bowl")
    head = object_model.get_part("mixing_head")
    beater = object_model.get_part("beater_hub")
    speed = object_model.get_part("speed_lever")
    bowl_lock = object_model.get_articulation("bowl_twist_lock")
    head_tilt = object_model.get_articulation("head_tilt")
    spin = object_model.get_articulation("beater_spin")
    slider = object_model.get_articulation("speed_slider")

    ctx.allow_overlap(
        head,
        beater,
        elem_a="head_shell",
        elem_b="hub_coupler",
        reason="The metal drive hub is intentionally seated slightly into the head socket.",
    )
    ctx.allow_overlap(
        head,
        speed,
        elem_a="speed_slot_plate",
        elem_b="lever_stem",
        reason="The speed lever stem intentionally passes into the side control slot.",
    )
    ctx.expect_contact(
        head,
        speed,
        elem_a="speed_slot_plate",
        elem_b="lever_stem",
        contact_tol=0.003,
        name="speed lever stem is captured in the side slot",
    )
    ctx.expect_contact(
        bowl,
        base,
        elem_a="bowl_shell",
        elem_b="bowl_lock_plate",
        contact_tol=0.002,
        name="bowl foot seats on twist lock plate",
    )
    ctx.expect_within(
        beater,
        bowl,
        axes="xy",
        inner_elem="flat_beater",
        outer_elem="bowl_shell",
        margin=0.0,
        name="flat beater stays inside the cylindrical bowl footprint",
    )
    ctx.expect_gap(
        beater,
        base,
        axis="z",
        positive_elem="flat_beater",
        negative_elem="bowl_lock_plate",
        min_gap=0.018,
        name="beater clears the bowl floor and lock plate",
    )
    ctx.expect_contact(
        head,
        beater,
        elem_a="head_shell",
        elem_b="hub_coupler",
        contact_tol=0.003,
        name="rotating hub is seated under the tilting head",
    )

    rest_hub_pos = ctx.part_world_position(beater)
    with ctx.pose({head_tilt: 0.72}):
        raised_hub_pos = ctx.part_world_position(beater)
    ctx.check(
        "head tilt lifts the beater clear of the bowl",
        rest_hub_pos is not None
        and raised_hub_pos is not None
        and raised_hub_pos[2] > rest_hub_pos[2] + 0.075,
        details=f"rest={rest_hub_pos}, raised={raised_hub_pos}",
    )

    def _aabb_center_xy(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return ((lo[0] + hi[0]) * 0.5, (lo[1] + hi[1]) * 0.5)

    tab_rest = _aabb_center_xy(ctx.part_element_world_aabb(bowl, elem="lock_tab_0"))
    with ctx.pose({bowl_lock: 0.26}):
        tab_twisted = _aabb_center_xy(ctx.part_element_world_aabb(bowl, elem="lock_tab_0"))
    ctx.check(
        "bowl twist lock rotates the locking tabs on the base",
        tab_rest is not None
        and tab_twisted is not None
        and abs(tab_twisted[1] - tab_rest[1]) > 0.020,
        details=f"rest={tab_rest}, twisted={tab_twisted}",
    )

    rest_beater_aabb = ctx.part_element_world_aabb(beater, elem="flat_beater")
    with ctx.pose({spin: math.pi / 2.0}):
        spun_beater_aabb = ctx.part_element_world_aabb(beater, elem="flat_beater")
    ctx.check(
        "continuous hub spin carries the flat beater around the shaft",
        rest_beater_aabb is not None
        and spun_beater_aabb is not None
        and (rest_beater_aabb[1][0] - rest_beater_aabb[0][0])
        > (rest_beater_aabb[1][1] - rest_beater_aabb[0][1]) * 3.0
        and (spun_beater_aabb[1][1] - spun_beater_aabb[0][1])
        > (spun_beater_aabb[1][0] - spun_beater_aabb[0][0]) * 3.0,
        details=f"rest={rest_beater_aabb}, spun={spun_beater_aabb}",
    )

    lever_rest = ctx.part_world_position(speed)
    with ctx.pose({slider: 0.045}):
        lever_high = ctx.part_world_position(speed)
    ctx.check(
        "side speed lever slides through its control slot",
        lever_rest is not None and lever_high is not None and lever_high[1] > lever_rest[1] + 0.035,
        details=f"rest={lever_rest}, high={lever_high}",
    )

    return ctx.report()


object_model = build_object_model()
