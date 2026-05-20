from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
from pathlib import Path

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

HERE = Path(__file__).resolve().parent


def _add_radial_ribs(
    part,
    *,
    prefix: str,
    core_radius: float,
    z_min: float,
    z_max: float,
    rib_depth: float,
    rib_width: float,
    count: int,
    material: Material,
    phase: float = 0.0,
    embed: float = 0.001,
) -> None:
    rib_length = z_max - z_min
    rib_mid = 0.5 * (z_min + z_max)
    radial_center = core_radius + 0.5 * rib_depth - embed
    for idx in range(count):
        angle = phase + (2.0 * math.pi * idx / count)
        part.visual(
            Box((rib_depth, rib_width, rib_length)),
            origin=Origin(
                xyz=(
                    radial_center * math.cos(angle),
                    radial_center * math.sin(angle),
                    rib_mid,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            material=material,
            name=f"{prefix}_rib_{idx:02d}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="professional_camera_lens")

    matte_black = Material(name="matte_black", rgba=(0.08, 0.08, 0.09, 1.0))
    satin_black = Material(name="satin_black", rgba=(0.14, 0.14, 0.15, 1.0))
    rubber_black = Material(name="rubber_black", rgba=(0.05, 0.05, 0.05, 1.0))
    brushed_steel = Material(name="brushed_steel", rgba=(0.58, 0.59, 0.61, 1.0))
    anodized_gold = Material(name="anodized_gold", rgba=(0.62, 0.54, 0.23, 1.0))
    engraving_white = Material(name="engraving_white", rgba=(0.92, 0.92, 0.90, 1.0))
    alignment_red = Material(name="alignment_red", rgba=(0.74, 0.08, 0.08, 1.0))
    coated_glass = Material(name="coated_glass", rgba=(0.12, 0.19, 0.22, 0.38))
    warm_glass = Material(name="warm_glass", rgba=(0.25, 0.20, 0.08, 0.24))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.034, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=brushed_steel,
        name="mount_flange",
    )
    base.visual(
        Cylinder(radius=0.036, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=rubber_black,
        name="mount_gasket",
    )
    base.visual(
        Cylinder(radius=0.040, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=matte_black,
        name="rear_barrel",
    )
    base.visual(
        Box((0.020, 0.004, 0.018)),
        origin=Origin(xyz=(0.038, 0.0, 0.040)),
        material=satin_black,
        name="switch_plate",
    )
    base.visual(
        Box((0.009, 0.005, 0.007)),
        origin=Origin(xyz=(0.043, 0.0, 0.043)),
        material=brushed_steel,
        name="switch_slider",
    )
    base.visual(
        Cylinder(radius=0.046, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=satin_black,
        name="mid_barrel",
    )
    base.visual(
        Box((0.003, 0.006, 0.010)),
        origin=Origin(xyz=(0.0445, -0.010, 0.062)),
        material=engraving_white,
        name="distance_mark_1",
    )
    base.visual(
        Box((0.003, 0.004, 0.010)),
        origin=Origin(xyz=(0.0445, -0.003, 0.062)),
        material=engraving_white,
        name="distance_mark_2",
    )
    base.visual(
        Cylinder(radius=0.047, length=0.017),
        origin=Origin(xyz=(0.0, 0.0, 0.0855)),
        material=matte_black,
        name="branding_band",
    )
    base.visual(
        Cylinder(radius=0.0476, length=0.0012),
        origin=Origin(xyz=(0.0, 0.0, 0.0925)),
        material=anodized_gold,
        name="accent_ring",
    )
    base.visual(
        Cylinder(radius=0.050, length=0.029),
        origin=Origin(xyz=(0.0, 0.0, 0.1085)),
        material=satin_black,
        name="front_shoulder",
    )
    base.visual(
        Cylinder(radius=0.0508, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.122)),
        material=matte_black,
        name="front_trim",
    )
    base.visual(
        Box((0.003, 0.003, 0.003)),
        origin=Origin(xyz=(0.0505, 0.0, 0.097)),
        material=alignment_red,
        name="mount_alignment_dot",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0508, length=0.123),
        mass=1.05,
        origin=Origin(xyz=(0.0, 0.0, 0.0615)),
    )

    zoom_ring = model.part("zoom_ring")
    zoom_ring.visual(
        Cylinder(radius=0.048, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=matte_black,
        name="zoom_rear_shell",
    )
    zoom_ring.visual(
        Cylinder(radius=0.050, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=rubber_black,
        name="zoom_grip_core",
    )
    zoom_ring.visual(
        Cylinder(radius=0.047, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=satin_black,
        name="zoom_front_shell",
    )
    _add_radial_ribs(
        zoom_ring,
        prefix="zoom",
        core_radius=0.050,
        z_min=0.009,
        z_max=0.031,
        rib_depth=0.004,
        rib_width=0.0045,
        count=30,
        material=rubber_black,
    )
    zoom_ring.visual(
        Box((0.004, 0.011, 0.006)),
        origin=Origin(xyz=(0.051, 0.0, 0.031)),
        material=engraving_white,
        name="zoom_index_marker",
    )
    zoom_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.052, length=0.038),
        mass=0.24,
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
    )

    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        Cylinder(radius=0.047, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=matte_black,
        name="focus_rear_shell",
    )
    focus_ring.visual(
        Cylinder(radius=0.049, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=rubber_black,
        name="focus_grip_core",
    )
    focus_ring.visual(
        Cylinder(radius=0.046, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        material=satin_black,
        name="front_filter_ring",
    )
    focus_ring.visual(
        Cylinder(radius=0.048, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=matte_black,
        name="front_bezel",
    )
    _add_radial_ribs(
        focus_ring,
        prefix="focus",
        core_radius=0.049,
        z_min=0.008,
        z_max=0.027,
        rib_depth=0.0035,
        rib_width=0.0032,
        count=36,
        material=rubber_black,
        phase=math.pi / 36.0,
    )
    focus_ring.visual(
        Box((0.006, 0.012, 0.006)),
        origin=Origin(xyz=(0.0505, 0.0, 0.018)),
        material=satin_black,
        name="focus_throw_tab",
    )
    focus_ring.visual(
        Cylinder(radius=0.043, length=0.009),
        origin=Origin(xyz=(0.0, 0.0, 0.0345)),
        material=coated_glass,
        name="front_element",
    )
    focus_ring.visual(
        Cylinder(radius=0.037, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=warm_glass,
        name="coating_reflection",
    )
    focus_ring.visual(
        Cylinder(radius=0.031, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=coated_glass,
        name="rear_visible_element",
    )
    focus_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0505, length=0.042),
        mass=0.31,
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
    )

    model.articulation(
        "base_to_zoom_ring",
        ArticulationType.REVOLUTE,
        parent="base",
        child="zoom_ring",
        origin=Origin(xyz=(0.0, 0.0, 0.123)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=4.0,
            lower=-1.35,
            upper=1.35,
        ),
    )
    model.articulation(
        "zoom_ring_to_focus_ring",
        ArticulationType.REVOLUTE,
        parent="zoom_ring",
        child="focus_ring",
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=5.0,
            lower=-2.2,
            upper=2.2,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=160,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
    )

    def check_lens_stack() -> None:
        ctx.expect_aabb_overlap("zoom_ring", "base", axes="xy", min_overlap=0.085)
        ctx.expect_aabb_overlap("focus_ring", "zoom_ring", axes="xy", min_overlap=0.084)
        ctx.expect_aabb_overlap("focus_ring", "base", axes="xy", min_overlap=0.080)
        ctx.expect_origin_distance("zoom_ring", "base", axes="xy", max_dist=0.002)
        ctx.expect_origin_distance("focus_ring", "zoom_ring", axes="xy", max_dist=0.002)
        ctx.expect_origin_distance("focus_ring", "base", axes="xy", max_dist=0.002)
        ctx.expect_aabb_gap("zoom_ring", "base", axis="z", max_gap=0.003, max_penetration=0.0)
        ctx.expect_aabb_gap("focus_ring", "zoom_ring", axis="z", max_gap=0.003, max_penetration=0.0)
        ctx.expect_aabb_gap("focus_ring", "base", axis="z", max_gap=0.045, max_penetration=0.0)

    check_lens_stack()

    with ctx.pose(base_to_zoom_ring=-1.35, zoom_ring_to_focus_ring=-2.2):
        check_lens_stack()

    with ctx.pose(base_to_zoom_ring=1.35, zoom_ring_to_focus_ring=2.2):
        check_lens_stack()

    with ctx.pose(base_to_zoom_ring=0.65, zoom_ring_to_focus_ring=-0.9):
        check_lens_stack()

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
