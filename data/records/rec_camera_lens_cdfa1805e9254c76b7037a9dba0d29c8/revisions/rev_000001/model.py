from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _lathed_shell(
    name: str,
    *,
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
    segments: int = 72,
):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=segments,
            start_cap="flat",
            end_cap="flat",
        ),
        name,
    )


def _ring_mesh(name: str, *, inner_radius: float, outer_radius: float, length: float):
    half = 0.5 * length
    bevel = min(0.002, length * 0.12, (outer_radius - inner_radius) * 0.35)
    return _lathed_shell(
        name,
        outer_profile=[
            (outer_radius - bevel, -half),
            (outer_radius, -half + bevel),
            (outer_radius, half - bevel),
            (outer_radius - bevel, half),
        ],
        inner_profile=[
            (inner_radius, -half),
            (inner_radius, half),
        ],
    )


def _add_ribs(
    part,
    *,
    count: int,
    radius: float,
    radial_depth: float,
    tangential_width: float,
    length: float,
    z_center: float,
    material,
    name_prefix: str,
) -> None:
    for index in range(count):
        angle = (2.0 * math.pi * index) / count
        part.visual(
            Box((radial_depth, tangential_width, length)),
            origin=Origin(
                xyz=(
                    radius * math.cos(angle),
                    radius * math.sin(angle),
                    z_center,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            material=material,
            name=f"{name_prefix}_{index:02d}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="standard_zoom_lens")

    alloy = model.material("satin_alloy", rgba=(0.48, 0.50, 0.52, 1.0))
    dark_alloy = model.material("dark_alloy", rgba=(0.10, 0.11, 0.12, 1.0))
    rubber = model.material("ribbed_rubber", rgba=(0.015, 0.016, 0.018, 1.0))
    matte_black = model.material("matte_black", rgba=(0.003, 0.003, 0.004, 1.0))
    glass = model.material("coated_glass", rgba=(0.24, 0.42, 0.52, 0.45))
    marking_white = model.material("engraved_white", rgba=(0.92, 0.93, 0.86, 1.0))
    marking_orange = model.material("index_orange", rgba=(1.0, 0.35, 0.08, 1.0))

    barrel_shell = _lathed_shell(
        "alloy_barrel_shell",
        outer_profile=[
            (0.037, -0.004),
            (0.041, 0.002),
            (0.041, 0.011),
            (0.037, 0.017),
            (0.035, 0.026),
            (0.034, 0.087),
            (0.035, 0.100),
            (0.033, 0.108),
        ],
        inner_profile=[
            (0.025, -0.004),
            (0.026, 0.012),
            (0.0315, 0.030),
            (0.0315, 0.108),
        ],
        segments=96,
    )
    rear_mount_lip = _ring_mesh(
        "rear_mount_lip",
        inner_radius=0.026,
        outer_radius=0.043,
        length=0.006,
    )
    zoom_front_race = _ring_mesh(
        "zoom_front_race",
        inner_radius=0.032,
        outer_radius=0.036,
        length=0.004,
    )
    zoom_rear_race = _ring_mesh(
        "zoom_rear_race",
        inner_radius=0.032,
        outer_radius=0.036,
        length=0.004,
    )
    front_guide = _ring_mesh(
        "front_guide_sleeve",
        inner_radius=0.0315,
        outer_radius=0.035,
        length=0.010,
    )
    internal_baffle = _ring_mesh(
        "internal_baffle_ring",
        inner_radius=0.020,
        outer_radius=0.032,
        length=0.003,
    )

    zoom_shell = _ring_mesh(
        "zoom_rubber_shell",
        inner_radius=0.036,
        outer_radius=0.0415,
        length=0.043,
    )
    inner_shell = _lathed_shell(
        "inner_barrel_shell",
        outer_profile=[
            (0.0315, -0.030),
            (0.0315, -0.002),
            (0.0300, 0.006),
            (0.0300, 0.058),
            (0.0335, 0.062),
            (0.0335, 0.070),
        ],
        inner_profile=[
            (0.0240, -0.030),
            (0.0240, 0.058),
            (0.0255, 0.070),
        ],
        segments=96,
    )
    focus_bearing_rear = _ring_mesh(
        "focus_bearing_rear",
        inner_radius=0.024,
        outer_radius=0.032,
        length=0.003,
    )
    focus_bearing_front = _ring_mesh(
        "focus_bearing_front",
        inner_radius=0.024,
        outer_radius=0.032,
        length=0.003,
    )
    focus_shell = _ring_mesh(
        "focus_rubber_shell",
        inner_radius=0.032,
        outer_radius=0.0370,
        length=0.027,
    )

    barrel = model.part("barrel")
    barrel.visual(barrel_shell, material=alloy, name="alloy_barrel")
    barrel.visual(
        rear_mount_lip,
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
        material=dark_alloy,
        name="metal_mount_lip",
    )
    for index in range(3):
        angle = (2.0 * math.pi * index) / 3.0 + math.radians(22)
        barrel.visual(
            Box((0.009, 0.018, 0.004)),
            origin=Origin(
                xyz=(0.040 * math.cos(angle), 0.040 * math.sin(angle), -0.004),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark_alloy,
            name=f"bayonet_lug_{index}",
        )
    barrel.visual(
        zoom_rear_race,
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=dark_alloy,
        name="zoom_rear_race",
    )
    barrel.visual(
        zoom_front_race,
        origin=Origin(xyz=(0.0, 0.0, 0.076)),
        material=dark_alloy,
        name="zoom_front_race",
    )
    barrel.visual(
        front_guide,
        origin=Origin(xyz=(0.0, 0.0, 0.101)),
        material=dark_alloy,
        name="front_guide",
    )
    for offset, z_pos in enumerate((0.021, 0.030, 0.088)):
        barrel.visual(
            internal_baffle,
            origin=Origin(xyz=(0.0, 0.0, z_pos)),
            material=matte_black,
            name=f"internal_baffle_{offset}",
        )
    barrel.visual(
        Box((0.004, 0.0014, 0.017)),
        origin=Origin(xyz=(0.0, -0.0345, 0.085)),
        material=marking_white,
        name="zoom_scale_mark",
    )
    for index, z_pos in enumerate((0.039, 0.050, 0.061, 0.072)):
        barrel.visual(
            Box((0.003, 0.0015, 0.006)),
            origin=Origin(xyz=(0.0, -0.0348, z_pos)),
            material=marking_white,
            name=f"focal_tick_{index}",
        )

    zoom_ring = model.part("zoom_ring")
    zoom_ring.visual(zoom_shell, material=rubber, name="zoom_sleeve")
    _add_ribs(
        zoom_ring,
        count=40,
        radius=0.0420,
        radial_depth=0.0040,
        tangential_width=0.0026,
        length=0.038,
        z_center=0.0,
        material=rubber,
        name_prefix="zoom_rib",
    )
    zoom_ring.visual(
        Box((0.004, 0.0015, 0.021)),
        origin=Origin(xyz=(0.0, -0.0432, 0.0)),
        material=marking_orange,
        name="zoom_index_line",
    )

    inner_barrel = model.part("inner_barrel")
    inner_barrel.visual(inner_shell, material=dark_alloy, name="inner_shell")
    inner_barrel.visual(
        focus_bearing_rear,
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=dark_alloy,
        name="focus_rear_race",
    )
    inner_barrel.visual(
        focus_bearing_front,
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        material=dark_alloy,
        name="focus_front_race",
    )
    inner_barrel.visual(
        Cylinder(radius=0.023, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.064)),
        material=glass,
        name="front_glass",
    )
    inner_barrel.visual(
        _ring_mesh("front_retainer", inner_radius=0.023, outer_radius=0.0335, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.066)),
        material=matte_black,
        name="front_retainer",
    )

    focus_ring = model.part("focus_ring")
    focus_ring.visual(focus_shell, material=rubber, name="focus_sleeve")
    _add_ribs(
        focus_ring,
        count=34,
        radius=0.0375,
        radial_depth=0.0035,
        tangential_width=0.0022,
        length=0.023,
        z_center=0.0,
        material=rubber,
        name_prefix="focus_rib",
    )
    focus_ring.visual(
        Box((0.0032, 0.0020, 0.017)),
        origin=Origin(xyz=(0.0, -0.0372, 0.0)),
        material=marking_white,
        name="focus_index_line",
    )

    zoom_rotation = model.articulation(
        "zoom_rotation",
        ArticulationType.REVOLUTE,
        parent=barrel,
        child=zoom_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.1,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(80.0),
        ),
    )
    model.articulation(
        "zoom_extension",
        ArticulationType.PRISMATIC,
        parent=barrel,
        child=inner_barrel,
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=0.05,
            lower=0.0,
            upper=0.025,
        ),
        meta={
            "driven_by": zoom_rotation.name,
            "cam_ratio_m_per_rad": 0.025 / math.radians(80.0),
            "note": "Cross-domain screw-cam coupling: zoom ring rotation drives axial extension.",
        },
    )
    model.articulation(
        "focus_rotation",
        ArticulationType.REVOLUTE,
        parent=inner_barrel,
        child=focus_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.7,
            velocity=2.5,
            lower=math.radians(-95.0),
            upper=math.radians(95.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barrel = object_model.get_part("barrel")
    zoom_ring = object_model.get_part("zoom_ring")
    inner_barrel = object_model.get_part("inner_barrel")
    focus_ring = object_model.get_part("focus_ring")
    zoom = object_model.get_articulation("zoom_rotation")
    zoom_extension = object_model.get_articulation("zoom_extension")
    focus = object_model.get_articulation("focus_rotation")

    ctx.allow_overlap(
        barrel,
        inner_barrel,
        elem_a="alloy_barrel",
        elem_b="inner_shell",
        reason=(
            "The telescoping inner barrel is intentionally represented as a "
            "captured member inside the alloy sleeve proxy."
        ),
    )

    ctx.check(
        "zoom ring is revolute",
        zoom.articulation_type == ArticulationType.REVOLUTE and zoom.axis == (0.0, 0.0, 1.0),
        details=f"type={zoom.articulation_type}, axis={zoom.axis}",
    )
    ctx.check(
        "inner barrel slide is zoom driven",
        zoom_extension.articulation_type == ArticulationType.PRISMATIC
        and zoom_extension.meta.get("driven_by") == "zoom_rotation",
        details=f"type={zoom_extension.articulation_type}, meta={zoom_extension.meta}",
    )
    ctx.check(
        "focus ring is separate revolute",
        focus.articulation_type == ArticulationType.REVOLUTE
        and focus.parent == "inner_barrel"
        and focus.child == "focus_ring",
        details=f"type={focus.articulation_type}, parent={focus.parent}, child={focus.child}",
    )

    ctx.expect_contact(
        zoom_ring,
        barrel,
        elem_a="zoom_sleeve",
        elem_b="zoom_rear_race",
        contact_tol=0.0006,
        name="zoom ring rides on rear race",
    )
    ctx.expect_contact(
        focus_ring,
        inner_barrel,
        elem_a="focus_sleeve",
        elem_b="focus_front_race",
        contact_tol=0.0006,
        name="focus ring rides on front race",
    )
    ctx.expect_within(
        inner_barrel,
        barrel,
        axes="xy",
        inner_elem="inner_shell",
        outer_elem="alloy_barrel",
        margin=0.001,
        name="inner barrel is centered in alloy barrel",
    )
    ctx.expect_overlap(
        inner_barrel,
        barrel,
        axes="z",
        elem_a="inner_shell",
        elem_b="alloy_barrel",
        min_overlap=0.025,
        name="inner barrel is retained when collapsed",
    )

    rest_pos = ctx.part_world_position(inner_barrel)
    with ctx.pose({zoom: math.radians(80.0), zoom_extension: 0.025}):
        extended_pos = ctx.part_world_position(inner_barrel)
        ctx.expect_overlap(
            inner_barrel,
            barrel,
            axes="z",
            elem_a="inner_shell",
            elem_b="alloy_barrel",
            min_overlap=0.015,
            name="inner barrel remains retained when extended",
        )
    ctx.check(
        "zoom cam setting extends inner barrel",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[2] > rest_pos[2] + 0.020,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    before_focus = ctx.part_world_position(inner_barrel)
    with ctx.pose({focus: math.radians(60.0)}):
        after_focus = ctx.part_world_position(inner_barrel)
    ctx.check(
        "focus rotation does not drive extension",
        before_focus is not None
        and after_focus is not None
        and abs(after_focus[2] - before_focus[2]) < 1e-6,
        details=f"before={before_focus}, after={after_focus}",
    )

    return ctx.report()


object_model = build_object_model()
