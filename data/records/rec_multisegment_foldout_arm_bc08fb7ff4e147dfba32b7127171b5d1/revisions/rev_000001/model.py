from __future__ import annotations

import math

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


LINK_SPECS = (
    # name, length in the motion plane, vertical step, y offset, y width, plate thickness, boss radius
    ("root_rocker", 0.34, 0.090, 0.0975, 0.065, 0.040, 0.060),
    ("middle_link", 0.285, -0.110, 0.0375, 0.055, 0.034, 0.052),
    ("outer_crank", 0.240, 0.060, -0.0125, 0.045, 0.030, 0.044),
    ("tip_link", 0.185, -0.040, -0.0540, 0.038, 0.026, 0.038),
)


def _add_segment(
    part,
    *,
    length: float,
    step_z: float,
    y_offset: float,
    y_width: float,
    plate_thickness: float,
    boss_radius: float,
    material,
    rib_material,
) -> None:
    """Build one non-repeating rocker link from overlapping, supported features."""
    span = math.hypot(length, step_z)
    pitch = math.atan2(-step_z, length)
    mid = (0.5 * length, y_offset, 0.5 * step_z)
    cyl_rpy = (math.pi / 2.0, 0.0, 0.0)

    part.visual(
        Box((span, y_width, plate_thickness)),
        origin=Origin(xyz=mid, rpy=(0.0, pitch, 0.0)),
        material=material,
        name="web",
    )
    part.visual(
        Cylinder(radius=boss_radius, length=y_width),
        origin=Origin(xyz=(0.0, y_offset, 0.0), rpy=cyl_rpy),
        material=material,
        name="prox_boss",
    )
    part.visual(
        Cylinder(radius=boss_radius * 0.92, length=y_width),
        origin=Origin(xyz=(length, y_offset, step_z), rpy=cyl_rpy),
        material=material,
        name="dist_boss",
    )

    # A raised rib makes each plate read as a fabricated stepped arm instead of a
    # plain repeated bar.  It is slightly sunk into the web to remain connected.
    rib_thickness = plate_thickness * 0.46
    normal_x = math.sin(pitch)
    normal_z = math.cos(pitch)
    rib_offset = 0.5 * plate_thickness + 0.5 * rib_thickness - 0.003
    part.visual(
        Box((span * 0.64, y_width * 0.46, rib_thickness)),
        origin=Origin(
            xyz=(
                mid[0] + normal_x * rib_offset,
                y_offset,
                mid[2] + normal_z * rib_offset,
            ),
            rpy=(0.0, pitch, 0.0),
        ),
        material=rib_material,
        name="raised_rib",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="offset_rocker_foldout_arm")

    dark_steel = model.material("dark_blued_steel", rgba=(0.08, 0.095, 0.105, 1.0))
    gunmetal = model.material("gunmetal_links", rgba=(0.28, 0.31, 0.33, 1.0))
    warm_steel = model.material("brushed_warm_steel", rgba=(0.58, 0.55, 0.49, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    brass = model.material("oiled_bronze_bushings", rgba=(0.72, 0.49, 0.20, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.44, 0.30, 0.040)),
        origin=Origin(xyz=(0.02, 0.0, 0.020)),
        material=dark_steel,
        name="ground_foot",
    )
    base.visual(
        Box((0.18, 0.18, 0.145)),
        origin=Origin(xyz=(-0.025, 0.0, 0.112)),
        material=dark_steel,
        name="stepped_pedestal",
    )
    base.visual(
        Box((0.080, 0.120, 0.115)),
        origin=Origin(xyz=(0.0, 0.0, 0.198)),
        material=dark_steel,
        name="upright_lug",
    )
    base.visual(
        Cylinder(radius=0.060, length=0.130),
        origin=Origin(xyz=(0.0, 0.0, 0.250), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="base_hinge",
    )
    for idx, (x, y) in enumerate(((-0.14, -0.10), (-0.14, 0.10), (0.18, -0.10), (0.18, 0.10))):
        base.visual(
            Box((0.070, 0.050, 0.014)),
            origin=Origin(xyz=(x, y, 0.004)),
            material=black_rubber,
            name=f"rubber_pad_{idx}",
        )

    link_materials = (gunmetal, warm_steel, gunmetal, warm_steel)
    rib_materials = (warm_steel, gunmetal, warm_steel, gunmetal)
    parts = {}
    for spec, material, rib_material in zip(LINK_SPECS, link_materials, rib_materials):
        name, length, step_z, y_offset, y_width, plate_thickness, boss_radius = spec
        link = model.part(name)
        _add_segment(
            link,
            length=length,
            step_z=step_z,
            y_offset=y_offset,
            y_width=y_width,
            plate_thickness=plate_thickness,
            boss_radius=boss_radius,
            material=material,
            rib_material=rib_material,
        )
        parts[name] = link

    # The small output shoe is built into the final link and aligned with the
    # fourth joint plane; it gives the arm a functional fold-out endpoint.
    parts["tip_link"].visual(
        Box((0.070, 0.055, 0.032)),
        origin=Origin(xyz=(0.220, -0.054, -0.045), rpy=(0.0, -0.18, 0.0)),
        material=dark_steel,
        name="output_shoe",
    )

    model.articulation(
        "base_to_root",
        ArticulationType.REVOLUTE,
        parent=base,
        child=parts["root_rocker"],
        origin=Origin(xyz=(0.0, 0.0, 0.250)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=2.0, lower=-0.75, upper=1.35),
    )
    model.articulation(
        "root_to_middle",
        ArticulationType.REVOLUTE,
        parent=parts["root_rocker"],
        child=parts["middle_link"],
        origin=Origin(xyz=(LINK_SPECS[0][1], 0.0, LINK_SPECS[0][2])),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=32.0, velocity=2.4, lower=-1.45, upper=1.20),
    )
    model.articulation(
        "middle_to_outer",
        ArticulationType.REVOLUTE,
        parent=parts["middle_link"],
        child=parts["outer_crank"],
        origin=Origin(xyz=(LINK_SPECS[1][1], 0.0, LINK_SPECS[1][2])),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=24.0, velocity=2.8, lower=-1.30, upper=1.30),
    )
    model.articulation(
        "outer_to_tip",
        ArticulationType.REVOLUTE,
        parent=parts["outer_crank"],
        child=parts["tip_link"],
        origin=Origin(xyz=(LINK_SPECS[2][1], 0.0, LINK_SPECS[2][2])),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=16.0, velocity=3.0, lower=-1.10, upper=1.45),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    joints = object_model.articulations
    ctx.check(
        "serial chain has four revolute joints",
        len(joints) == 4 and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints),
        details=f"joints={[j.name for j in joints]}",
    )
    ctx.check(
        "all joint axes share one rocker plane",
        all(tuple(round(v, 6) for v in j.axis) == (0.0, 1.0, 0.0) for j in joints),
        details=f"axes={[(j.name, j.axis) for j in joints]}",
    )

    width_sequence = [spec[4] for spec in LINK_SPECS]
    step_sequence = [spec[2] for spec in LINK_SPECS]
    ctx.check(
        "link widths and steps vary toward the tip",
        len(set(width_sequence)) == 4 and any(s > 0 for s in step_sequence) and any(s < 0 for s in step_sequence),
        details=f"widths={width_sequence}, steps={step_sequence}",
    )

    base = object_model.get_part("base")
    root = object_model.get_part("root_rocker")
    middle = object_model.get_part("middle_link")
    outer = object_model.get_part("outer_crank")
    tip = object_model.get_part("tip_link")

    ctx.expect_overlap(root, base, axes="xz", elem_a="prox_boss", elem_b="base_hinge", min_overlap=0.040)
    ctx.expect_contact(
        root,
        base,
        elem_a="prox_boss",
        elem_b="base_hinge",
        name="root rocker bears on base hinge cheek",
    )

    ctx.expect_overlap(root, middle, axes="xz", elem_a="dist_boss", elem_b="prox_boss", min_overlap=0.035)
    ctx.expect_contact(
        root,
        middle,
        elem_a="dist_boss",
        elem_b="prox_boss",
        name="first offset hinge cheeks touch",
    )

    ctx.expect_overlap(middle, outer, axes="xz", elem_a="dist_boss", elem_b="prox_boss", min_overlap=0.030)
    ctx.expect_contact(
        middle,
        outer,
        elem_a="dist_boss",
        elem_b="prox_boss",
        name="second offset hinge cheeks touch",
    )

    ctx.expect_overlap(outer, tip, axes="xz", elem_a="dist_boss", elem_b="prox_boss", min_overlap=0.025)
    ctx.expect_contact(
        outer,
        tip,
        elem_a="dist_boss",
        elem_b="prox_boss",
        name="tip offset hinge cheeks touch",
    )

    rest_tip = ctx.part_world_position(tip)
    with ctx.pose({"base_to_root": 0.50, "root_to_middle": -0.65, "middle_to_outer": 0.55, "outer_to_tip": -0.45}):
        moved_tip = ctx.part_world_position(tip)
    ctx.check(
        "compound revolute pose moves the output link in the xz plane",
        rest_tip is not None
        and moved_tip is not None
        and abs(moved_tip[1] - rest_tip[1]) < 1.0e-6
        and (abs(moved_tip[0] - rest_tip[0]) + abs(moved_tip[2] - rest_tip[2])) > 0.050,
        details=f"rest={rest_tip}, moved={moved_tip}",
    )

    return ctx.report()


object_model = build_object_model()
