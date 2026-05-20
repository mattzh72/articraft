from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import os

os.chdir("/")
if not os.path.isabs(__file__):
    __file__ = f"/{os.path.basename(__file__)}"

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

BASE_HEIGHT = 0.22
PEDESTAL_HEIGHT = 0.12
LOWER_MAST_RADIUS = 0.06
LOWER_MAST_HEIGHT = 1.95
HINGE_X = 0.18
HINGE_Z = BASE_HEIGHT + PEDESTAL_HEIGHT + LOWER_MAST_HEIGHT

UPPER_MAST_RADIUS = 0.05
UPPER_MAST_HEIGHT = 1.34
UPPER_HEAD_Z = 1.67
HEAD_PIVOT_X = 0.12

HEAD_SPECS = (
    ("left", -0.30),
    ("center", 0.0),
    ("right", 0.30),
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hinged_mast_floodlight_pole")

    concrete = model.material("concrete", rgba=(0.63, 0.64, 0.66, 1.0))
    galvanized = model.material("galvanized_steel", rgba=(0.58, 0.60, 0.62, 1.0))
    dark = model.material("dark_housing", rgba=(0.17, 0.18, 0.19, 1.0))
    black = model.material("bracket_black", rgba=(0.10, 0.10, 0.11, 1.0))
    lens_frame = model.material("lens_frame", rgba=(0.22, 0.23, 0.24, 1.0))
    glass = model.material("lens_glass", rgba=(0.82, 0.88, 0.94, 0.42))

    base = model.part("base_assembly")
    base.visual(
        Box((0.72, 0.52, BASE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT * 0.5)),
        material=concrete,
        name="foundation",
    )
    base.visual(
        Box((0.18, 0.18, PEDESTAL_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT + PEDESTAL_HEIGHT * 0.5)),
        material=galvanized,
        name="anchor_pedestal",
    )
    base.visual(
        Cylinder(radius=LOWER_MAST_RADIUS, length=LOWER_MAST_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT + PEDESTAL_HEIGHT + LOWER_MAST_HEIGHT * 0.5)),
        material=galvanized,
        name="lower_mast",
    )
    base.visual(
        Cylinder(radius=0.085, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, HINGE_Z - 0.07)),
        material=galvanized,
        name="lower_mast_collar",
    )
    base.visual(
        Box((0.03, 0.13, 0.08)),
        origin=Origin(xyz=(HINGE_X - 0.0775, 0.0, HINGE_Z - 0.07)),
        material=black,
        name="hinge_spine",
    )
    base.visual(
        Box((0.17, 0.008, 0.16)),
        origin=Origin(xyz=(HINGE_X - 0.04, 0.055, HINGE_Z)),
        material=black,
        name="hinge_pos_cheek",
    )
    base.visual(
        Box((0.17, 0.008, 0.16)),
        origin=Origin(xyz=(HINGE_X - 0.04, -0.055, HINGE_Z)),
        material=black,
        name="hinge_neg_cheek",
    )
    base.visual(
        Box((0.085, 0.13, 0.035)),
        origin=Origin(xyz=(HINGE_X - 0.0975, 0.0, HINGE_Z - 0.0625)),
        material=black,
        name="hinge_saddle",
    )

    upper = model.part("upper_mast")
    upper.visual(
        Box((0.05, 0.086, 0.10)),
        origin=Origin(xyz=(0.01, 0.0, 0.05)),
        material=black,
        name="hinge_block",
    )
    upper.visual(
        Cylinder(radius=0.04, length=0.008),
        origin=Origin(xyz=(0.0, 0.047, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=black,
        name="hinge_pos_trunnion",
    )
    upper.visual(
        Cylinder(radius=0.04, length=0.008),
        origin=Origin(xyz=(0.0, -0.047, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=black,
        name="hinge_neg_trunnion",
    )
    upper.visual(
        Box((0.14, 0.08, 0.18)),
        origin=Origin(xyz=(0.08, 0.0, 0.16)),
        material=black,
        name="mast_transition",
    )
    upper.visual(
        Cylinder(radius=UPPER_MAST_RADIUS, length=UPPER_MAST_HEIGHT),
        origin=Origin(xyz=(0.12, 0.0, 0.24 + UPPER_MAST_HEIGHT * 0.5)),
        material=galvanized,
        name="upper_mast_column",
    )
    upper.visual(
        Box((0.16, 0.16, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, UPPER_HEAD_Z)),
        material=black,
        name="top_hub",
    )
    upper.visual(
        Box((0.06, 0.84, 0.06)),
        origin=Origin(xyz=(0.03, 0.0, UPPER_HEAD_Z)),
        material=black,
        name="array_crossarm",
    )
    upper.visual(
        Box((0.05, 0.24, 0.05)),
        origin=Origin(xyz=(0.07, 0.0, UPPER_HEAD_Z - 0.045)),
        material=black,
        name="array_spine",
    )

    for head_name, y_pos in HEAD_SPECS:
        upper.visual(
            Box((0.09, 0.006, 0.10)),
            origin=Origin(xyz=(0.095, y_pos + 0.054, UPPER_HEAD_Z)),
            material=black,
            name=f"head_{head_name}_pos_cheek",
        )
        upper.visual(
            Box((0.09, 0.006, 0.10)),
            origin=Origin(xyz=(0.095, y_pos - 0.054, UPPER_HEAD_Z)),
            material=black,
            name=f"head_{head_name}_neg_cheek",
        )
        upper.visual(
            Box((0.09, 0.028, 0.04)),
            origin=Origin(xyz=(0.03, y_pos, UPPER_HEAD_Z - 0.01)),
            material=black,
            name=f"head_{head_name}_arm",
        )
        upper.visual(
            Box((0.03, 0.114, 0.014)),
            origin=Origin(xyz=(0.03, y_pos, UPPER_HEAD_Z + 0.035)),
            material=black,
            name=f"head_{head_name}_bridge",
        )

        head = model.part(f"head_{head_name}")
        head.visual(
            Box((0.145, 0.096, 0.09)),
            origin=Origin(xyz=(0.1275, 0.0, -0.01)),
            material=dark,
            name="housing",
        )
        head.visual(
            Box((0.055, 0.088, 0.074)),
            origin=Origin(xyz=(0.0275, 0.0, -0.012)),
            material=dark,
            name="rear_can",
        )
        head.visual(
            Box((0.018, 0.104, 0.092)),
            origin=Origin(xyz=(0.209, 0.0, -0.01)),
            material=lens_frame,
            name="bezel",
        )
        head.visual(
            Box((0.004, 0.090, 0.074)),
            origin=Origin(xyz=(0.220, 0.0, -0.01)),
            material=glass,
            name="lens",
        )
        head.visual(
            Box((0.036, 0.110, 0.012)),
            origin=Origin(xyz=(0.190, 0.0, 0.041)),
            material=lens_frame,
            name="visor",
        )
        for fin_index, x_pos in enumerate((0.004, 0.014, 0.024), start=1):
            head.visual(
                Box((0.006, 0.082, 0.078)),
                origin=Origin(xyz=(x_pos, 0.0, -0.012)),
                material=lens_frame,
                name=f"cooling_fin_{fin_index}",
            )
        head.visual(
            Cylinder(radius=0.020, length=0.008),
            origin=Origin(xyz=(0.0, 0.047, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
            material=lens_frame,
            name="pos_trunnion",
        )
        head.visual(
            Cylinder(radius=0.020, length=0.008),
            origin=Origin(xyz=(0.0, -0.047, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
            material=lens_frame,
            name="neg_trunnion",
        )

        model.articulation(
            f"head_{head_name}_tilt",
            ArticulationType.REVOLUTE,
            parent=upper,
            child=head,
            origin=Origin(xyz=(HEAD_PIVOT_X, y_pos, UPPER_HEAD_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=10.0,
                velocity=1.2,
                lower=-0.45,
                upper=0.45,
            ),
        )

    model.articulation(
        "mast_lowering_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=150.0,
            velocity=0.5,
            lower=-1.10,
            upper=0.10,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root="/")
    base = object_model.get_part("base_assembly")
    upper = object_model.get_part("upper_mast")
    heads = {
        name: object_model.get_part(f"head_{name}")
        for name, _ in HEAD_SPECS
    }

    mast_hinge = object_model.get_articulation("mast_lowering_hinge")
    head_tilts = {
        name: object_model.get_articulation(f"head_{name}_tilt")
        for name, _ in HEAD_SPECS
    }

    hinge_pos_cheek = base.get_visual("hinge_pos_cheek")
    hinge_neg_cheek = base.get_visual("hinge_neg_cheek")
    hinge_spine = base.get_visual("hinge_spine")
    hinge_pos_trunnion = upper.get_visual("hinge_pos_trunnion")
    hinge_neg_trunnion = upper.get_visual("hinge_neg_trunnion")
    hinge_block = upper.get_visual("hinge_block")
    upper_column = upper.get_visual("upper_mast_column")
    lower_collar = base.get_visual("lower_mast_collar")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts(max_pose_samples=16)
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=64)
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=64, ignore_adjacent=True, ignore_fixed=True)

    def axis_is_y(articulation) -> bool:
        return tuple(round(value, 6) for value in articulation.axis) == (0.0, 1.0, 0.0)

    ctx.check(
        "mast_lowering_hinge_axis",
        axis_is_y(mast_hinge),
        details=f"expected mast hinge axis (0, 1, 0), got {mast_hinge.axis}",
    )
    for head_name, articulation in head_tilts.items():
        ctx.check(
            f"{head_name}_tilt_axis",
            axis_is_y(articulation),
            details=f"expected {head_name} tilt axis (0, 1, 0), got {articulation.axis}",
        )

    ctx.expect_contact(upper, base, elem_a=hinge_pos_trunnion, elem_b=hinge_pos_cheek, name="mast_pos_pin_contact")
    ctx.expect_contact(upper, base, elem_a=hinge_neg_trunnion, elem_b=hinge_neg_cheek, name="mast_neg_pin_contact")
    ctx.expect_within(
        upper,
        base,
        axes="xz",
        inner_elem=hinge_pos_trunnion,
        outer_elem=hinge_pos_cheek,
        margin=0.001,
        name="mast_pos_pin_within_cheek",
    )
    ctx.expect_within(
        upper,
        base,
        axes="xz",
        inner_elem=hinge_neg_trunnion,
        outer_elem=hinge_neg_cheek,
        margin=0.001,
        name="mast_neg_pin_within_cheek",
    )
    ctx.expect_gap(
        upper,
        base,
        axis="x",
        positive_elem=hinge_block,
        negative_elem=hinge_spine,
        min_gap=0.03,
        max_gap=0.07,
        name="mast_hinge_block_clears_spine",
    )
    ctx.expect_gap(
        upper,
        base,
        axis="z",
        positive_elem=upper_column,
        negative_elem=lower_collar,
        min_gap=0.22,
        max_gap=0.26,
        name="upper_tube_stands_above_lower_collar",
    )

    head_visuals = {}
    for head_name, _ in HEAD_SPECS:
        head = heads[head_name]
        head_visuals[head_name] = {
            "pos_cheek": upper.get_visual(f"head_{head_name}_pos_cheek"),
            "neg_cheek": upper.get_visual(f"head_{head_name}_neg_cheek"),
            "arm": upper.get_visual(f"head_{head_name}_arm"),
            "bridge": upper.get_visual(f"head_{head_name}_bridge"),
            "pos_trunnion": head.get_visual("pos_trunnion"),
            "neg_trunnion": head.get_visual("neg_trunnion"),
            "rear_can": head.get_visual("rear_can"),
            "visor": head.get_visual("visor"),
        }

        ctx.expect_contact(
            head,
            upper,
            elem_a=head_visuals[head_name]["pos_trunnion"],
            elem_b=head_visuals[head_name]["pos_cheek"],
            name=f"{head_name}_pos_trunnion_contact",
        )
        ctx.expect_contact(
            head,
            upper,
            elem_a=head_visuals[head_name]["neg_trunnion"],
            elem_b=head_visuals[head_name]["neg_cheek"],
            name=f"{head_name}_neg_trunnion_contact",
        )
        ctx.expect_within(
            head,
            upper,
            axes="xz",
            inner_elem=head_visuals[head_name]["pos_trunnion"],
            outer_elem=head_visuals[head_name]["pos_cheek"],
            name=f"{head_name}_pos_trunnion_within_bracket",
        )
        ctx.expect_within(
            head,
            upper,
            axes="xz",
            inner_elem=head_visuals[head_name]["neg_trunnion"],
            outer_elem=head_visuals[head_name]["neg_cheek"],
            name=f"{head_name}_neg_trunnion_within_bracket",
        )
        ctx.expect_gap(
            upper,
            head,
            axis="z",
            positive_elem=head_visuals[head_name]["bridge"],
            negative_elem=head_visuals[head_name]["rear_can"],
            min_gap=0.002,
            max_gap=0.010,
            name=f"{head_name}_bridge_above_rear_can",
        )
        ctx.expect_gap(
            head,
            upper,
            axis="x",
            positive_elem=head_visuals[head_name]["rear_can"],
            negative_elem=head_visuals[head_name]["arm"],
            min_gap=0.035,
            max_gap=0.055,
            name=f"{head_name}_head_clears_mount_arm",
        )

    ctx.expect_gap(heads["center"], base, axis="z", min_gap=1.40, name="head_array_height")
    ctx.expect_gap(heads["center"], heads["left"], axis="y", min_gap=0.18, name="left_to_center_head_spacing")
    ctx.expect_gap(heads["right"], heads["center"], axis="y", min_gap=0.18, name="center_to_right_head_spacing")

    mast_limits = mast_hinge.motion_limits
    if mast_limits is not None and mast_limits.lower is not None and mast_limits.upper is not None:
        for pose_name, pose_value in (("lower", mast_limits.lower), ("upper", mast_limits.upper)):
            with ctx.pose({mast_hinge: pose_value}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"mast_{pose_name}_no_overlap")
                ctx.fail_if_isolated_parts(name=f"mast_{pose_name}_no_floating")
                ctx.expect_contact(
                    upper,
                    base,
                    elem_a=hinge_pos_trunnion,
                    elem_b=hinge_pos_cheek,
                    name=f"mast_{pose_name}_pos_pin_contact",
                )
                ctx.expect_contact(
                    upper,
                    base,
                    elem_a=hinge_neg_trunnion,
                    elem_b=hinge_neg_cheek,
                    name=f"mast_{pose_name}_neg_pin_contact",
                )
                ctx.expect_gap(
                    heads["center"],
                    base,
                    axis="z",
                    min_gap=0.18,
                    name=f"mast_{pose_name}_head_clear_of_base",
                )

    for head_name, articulation in head_tilts.items():
        limits = articulation.motion_limits
        if limits is None or limits.lower is None or limits.upper is None:
            continue
        for pose_name, pose_value in (("lower", limits.lower), ("upper", limits.upper)):
            with ctx.pose({articulation: pose_value}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{head_name}_{pose_name}_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{head_name}_{pose_name}_no_floating")
                ctx.expect_contact(
                    heads[head_name],
                    upper,
                    elem_a=head_visuals[head_name]["pos_trunnion"],
                    elem_b=head_visuals[head_name]["pos_cheek"],
                    name=f"{head_name}_{pose_name}_pos_pin_contact",
                )
                ctx.expect_contact(
                    heads[head_name],
                    upper,
                    elem_a=head_visuals[head_name]["neg_trunnion"],
                    elem_b=head_visuals[head_name]["neg_cheek"],
                    name=f"{head_name}_{pose_name}_neg_pin_contact",
                )

    with ctx.pose(
        {
            head_tilts["left"]: 0.45,
            head_tilts["center"]: -0.40,
            head_tilts["right"]: 0.35,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="mixed_head_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="mixed_head_pose_no_floating")
        ctx.expect_gap(
            heads["center"],
            heads["left"],
            axis="y",
            min_gap=0.12,
            name="mixed_pose_left_center_spacing",
        )
        ctx.expect_gap(
            heads["right"],
            heads["center"],
            axis="y",
            min_gap=0.12,
            name="mixed_pose_center_right_spacing",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
