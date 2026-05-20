from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_mast", assets=ASSETS)

    dark_base = model.material("dark_base", rgba=(0.16, 0.18, 0.20, 1.0))
    aluminum = model.material("aluminum", rgba=(0.76, 0.79, 0.82, 1.0))
    black_anodized = model.material("black_anodized", rgba=(0.10, 0.11, 0.13, 1.0))

    foot_radius = 0.075
    foot_thickness = 0.018

    base_outer_r = 0.036
    base_inner_r = 0.029
    base_height = 0.220
    base_guide_height = 0.028
    base_guide_pad_width = 0.008

    stage1_outer_r = 0.028
    stage1_inner_r = 0.023
    stage1_length = 0.310
    stage1_travel = 0.150
    stage1_rest_z = foot_thickness + 0.030

    stage1_guide_outer_r = stage1_inner_r
    stage1_guide_inner_r = 0.021
    stage1_guide_height = 0.025
    stage1_guide_z = 0.245
    stage1_guide_pad_width = 0.007

    stage2_outer_r = 0.021
    stage2_inner_r = 0.016
    stage2_length = 0.300
    stage2_travel = 0.150
    stage2_rest_z = 0.045

    pan_hub_radius = 0.020
    pan_hub_height = 0.018
    pan_plate_size = (0.080, 0.055, 0.006)
    sensor_pad_size = (0.032, 0.022, 0.005)

    def tube_mesh(filename: str, outer_r: float, inner_r: float, length: float):
        outer = cq.Workplane("XY").circle(outer_r).extrude(length)
        inner = cq.Workplane("XY").circle(inner_r).extrude(length)
        return mesh_from_cadquery(outer.cut(inner), filename, assets=ASSETS)

    base = model.part("base")
    base.visual(
        Cylinder(radius=foot_radius, length=foot_thickness),
        origin=Origin(xyz=(0.0, 0.0, foot_thickness / 2.0)),
        material=dark_base,
        name="foot",
    )
    base.visual(
        tube_mesh("base_sleeve.obj", base_outer_r, base_inner_r, base_height),
        origin=Origin(xyz=(0.0, 0.0, foot_thickness)),
        material=dark_base,
        name="sleeve",
    )
    base_pad_thickness = base_inner_r - stage1_outer_r
    base_pad_center = stage1_outer_r + base_pad_thickness / 2.0
    base.visual(
        Box((base_pad_thickness, base_guide_pad_width, base_guide_height)),
        origin=Origin(
            xyz=(base_pad_center, 0.0, foot_thickness + base_height - base_guide_height / 2.0)
        ),
        material=black_anodized,
        name="guide_pad_xp",
    )
    base.visual(
        Box((base_pad_thickness, base_guide_pad_width, base_guide_height)),
        origin=Origin(
            xyz=(-base_pad_center, 0.0, foot_thickness + base_height - base_guide_height / 2.0)
        ),
        material=black_anodized,
        name="guide_pad_xn",
    )
    base.visual(
        Box((base_guide_pad_width, base_pad_thickness, base_guide_height)),
        origin=Origin(
            xyz=(0.0, base_pad_center, foot_thickness + base_height - base_guide_height / 2.0)
        ),
        material=black_anodized,
        name="guide_pad_yp",
    )
    base.visual(
        Box((base_guide_pad_width, base_pad_thickness, base_guide_height)),
        origin=Origin(
            xyz=(0.0, -base_pad_center, foot_thickness + base_height - base_guide_height / 2.0)
        ),
        material=black_anodized,
        name="guide_pad_yn",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=foot_radius, length=foot_thickness),
        mass=3.8,
        origin=Origin(xyz=(0.0, 0.0, foot_thickness / 2.0)),
    )

    stage1 = model.part("stage1")
    stage1.visual(
        tube_mesh("stage1_tube.obj", stage1_outer_r, stage1_inner_r, stage1_length),
        material=aluminum,
        name="tube",
    )
    stage1_pad_thickness = stage1_inner_r - stage2_outer_r
    stage1_pad_center = stage2_outer_r + stage1_pad_thickness / 2.0
    stage1.visual(
        Box((stage1_pad_thickness, stage1_guide_pad_width, stage1_guide_height)),
        origin=Origin(xyz=(stage1_pad_center, 0.0, stage1_guide_z + stage1_guide_height / 2.0)),
        material=black_anodized,
        name="guide_pad_xp",
    )
    stage1.visual(
        Box((stage1_pad_thickness, stage1_guide_pad_width, stage1_guide_height)),
        origin=Origin(xyz=(-stage1_pad_center, 0.0, stage1_guide_z + stage1_guide_height / 2.0)),
        material=black_anodized,
        name="guide_pad_xn",
    )
    stage1.visual(
        Box((stage1_guide_pad_width, stage1_pad_thickness, stage1_guide_height)),
        origin=Origin(xyz=(0.0, stage1_pad_center, stage1_guide_z + stage1_guide_height / 2.0)),
        material=black_anodized,
        name="guide_pad_yp",
    )
    stage1.visual(
        Box((stage1_guide_pad_width, stage1_pad_thickness, stage1_guide_height)),
        origin=Origin(xyz=(0.0, -stage1_pad_center, stage1_guide_z + stage1_guide_height / 2.0)),
        material=black_anodized,
        name="guide_pad_yn",
    )
    stage1.inertial = Inertial.from_geometry(
        Cylinder(radius=stage1_outer_r, length=stage1_length),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, stage1_length / 2.0)),
    )

    stage2 = model.part("stage2")
    stage2.visual(
        tube_mesh("stage2_tube.obj", stage2_outer_r, stage2_inner_r, stage2_length),
        material=aluminum,
        name="tube",
    )
    stage2.inertial = Inertial.from_geometry(
        Cylinder(radius=stage2_outer_r, length=stage2_length),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, stage2_length / 2.0)),
    )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=pan_hub_radius, length=pan_hub_height),
        origin=Origin(xyz=(0.0, 0.0, pan_hub_height / 2.0)),
        material=black_anodized,
        name="hub",
    )
    pan_head.visual(
        Box(pan_plate_size),
        origin=Origin(xyz=(0.0, 0.0, pan_hub_height + pan_plate_size[2] / 2.0)),
        material=black_anodized,
        name="sensor_plate",
    )
    pan_head.visual(
        Box(sensor_pad_size),
        origin=Origin(
            xyz=(
                0.012,
                0.0,
                pan_hub_height + pan_plate_size[2] + sensor_pad_size[2] / 2.0,
            )
        ),
        material=aluminum,
        name="sensor_pad",
    )
    pan_head.inertial = Inertial.from_geometry(
        Box((pan_plate_size[0], pan_plate_size[1], pan_hub_height + pan_plate_size[2])),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, (pan_hub_height + pan_plate_size[2]) / 2.0)),
    )

    model.articulation(
        "base_to_stage1",
        ArticulationType.PRISMATIC,
        parent=base,
        child=stage1,
        origin=Origin(xyz=(0.0, 0.0, stage1_rest_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.25,
            lower=0.0,
            upper=stage1_travel,
        ),
    )
    model.articulation(
        "stage1_to_stage2",
        ArticulationType.PRISMATIC,
        parent=stage1,
        child=stage2,
        origin=Origin(xyz=(0.0, 0.0, stage2_rest_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.25,
            lower=0.0,
            upper=stage2_travel,
        ),
    )
    model.articulation(
        "stage2_to_pan_head",
        ArticulationType.REVOLUTE,
        parent=stage2,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, stage2_length)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=2.5,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    stage1 = object_model.get_part("stage1")
    stage2 = object_model.get_part("stage2")
    pan_head = object_model.get_part("pan_head")

    base_to_stage1 = object_model.get_articulation("base_to_stage1")
    stage1_to_stage2 = object_model.get_articulation("stage1_to_stage2")
    stage2_to_pan_head = object_model.get_articulation("stage2_to_pan_head")

    base_sleeve = base.get_visual("sleeve")
    base_guide = base.get_visual("guide_pad_xp")
    stage1_tube = stage1.get_visual("tube")
    stage1_guide = stage1.get_visual("guide_pad_xp")
    stage2_tube = stage2.get_visual("tube")
    pan_hub = pan_head.get_visual("hub")
    sensor_plate = pan_head.get_visual("sensor_plate")

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
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_origin_distance(stage1, base, axes="xy", max_dist=1e-6)
    ctx.expect_origin_distance(stage2, stage1, axes="xy", max_dist=1e-6)
    ctx.expect_origin_distance(pan_head, stage2, axes="xy", max_dist=1e-6)

    ctx.expect_within(stage1, base, axes="xy", inner_elem=stage1_tube, outer_elem=base_sleeve, margin=0.010)
    ctx.expect_within(stage2, stage1, axes="xy", inner_elem=stage2_tube, outer_elem=stage1_tube, margin=0.008)
    ctx.expect_contact(stage1, base, elem_a=stage1_tube, elem_b=base_guide, contact_tol=1e-6)
    ctx.expect_contact(stage2, stage1, elem_a=stage2_tube, elem_b=stage1_guide, contact_tol=1e-6)
    ctx.expect_gap(pan_head, stage2, axis="z", positive_elem=pan_hub, negative_elem=stage2_tube, max_gap=1e-6, max_penetration=0.0)
    ctx.expect_overlap(pan_head, stage2, axes="xy", elem_a=pan_hub, elem_b=stage2_tube, min_overlap=0.040)
    ctx.expect_overlap(pan_head, stage2, axes="xy", elem_a=sensor_plate, elem_b=stage2_tube, min_overlap=0.030)

    rest_stage1_pos = ctx.part_world_position(stage1)
    with ctx.pose({base_to_stage1: 0.150}):
        raised_stage1_pos = ctx.part_world_position(stage1)
        ctx.expect_contact(stage1, base, elem_a=stage1_tube, elem_b=base_guide, contact_tol=1e-6)
        ctx.expect_within(stage1, base, axes="xy", inner_elem=stage1_tube, outer_elem=base_sleeve, margin=0.010)
    ctx.check(
        "stage1_prismatic_travel_is_150mm",
        rest_stage1_pos is not None
        and raised_stage1_pos is not None
        and abs((raised_stage1_pos[2] - rest_stage1_pos[2]) - 0.150) < 1e-6,
        details=f"rest={rest_stage1_pos}, raised={raised_stage1_pos}",
    )

    rest_stage2_pos = ctx.part_world_position(stage2)
    with ctx.pose({stage1_to_stage2: 0.150}):
        raised_stage2_pos = ctx.part_world_position(stage2)
        ctx.expect_contact(stage2, stage1, elem_a=stage2_tube, elem_b=stage1_guide, contact_tol=1e-6)
        ctx.expect_within(stage2, stage1, axes="xy", inner_elem=stage2_tube, outer_elem=stage1_tube, margin=0.008)
    ctx.check(
        "stage2_prismatic_travel_is_150mm",
        rest_stage2_pos is not None
        and raised_stage2_pos is not None
        and abs((raised_stage2_pos[2] - rest_stage2_pos[2]) - 0.150) < 1e-6,
        details=f"rest={rest_stage2_pos}, raised={raised_stage2_pos}",
    )

    with ctx.pose({base_to_stage1: 0.150, stage1_to_stage2: 0.150}):
        ctx.expect_gap(pan_head, base, axis="z", min_gap=0.440, positive_elem=pan_hub, negative_elem=base_sleeve)
        ctx.expect_contact(stage1, base, elem_a=stage1_tube, elem_b=base_guide, contact_tol=1e-6)
        ctx.expect_contact(stage2, stage1, elem_a=stage2_tube, elem_b=stage1_guide, contact_tol=1e-6)

    with ctx.pose({stage2_to_pan_head: 0.0}):
        pan_rest_aabb = ctx.part_world_aabb(pan_head)
    with ctx.pose({stage2_to_pan_head: math.pi / 2.0}):
        pan_quarter_aabb = ctx.part_world_aabb(pan_head)
        ctx.expect_gap(pan_head, stage2, axis="z", positive_elem=pan_hub, negative_elem=stage2_tube, max_gap=1e-6, max_penetration=0.0)
    with ctx.pose({stage2_to_pan_head: -math.pi}):
        ctx.expect_overlap(pan_head, stage2, axes="xy", elem_a=pan_hub, elem_b=stage2_tube, min_overlap=0.040)

    if pan_rest_aabb is not None and pan_quarter_aabb is not None:
        rest_dx = pan_rest_aabb[1][0] - pan_rest_aabb[0][0]
        rest_dy = pan_rest_aabb[1][1] - pan_rest_aabb[0][1]
        quarter_dx = pan_quarter_aabb[1][0] - pan_quarter_aabb[0][0]
        quarter_dy = pan_quarter_aabb[1][1] - pan_quarter_aabb[0][1]
        ctx.check(
            "pan_head_rotates_about_vertical_axis",
            rest_dx > rest_dy and quarter_dy > quarter_dx,
            details=(
                f"rest_spans=({rest_dx:.6f}, {rest_dy:.6f}), "
                f"quarter_spans=({quarter_dx:.6f}, {quarter_dy:.6f})"
            ),
        )
    else:
        ctx.fail("pan_head_rotates_about_vertical_axis", "Pan head AABB unavailable during rotation check.")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
