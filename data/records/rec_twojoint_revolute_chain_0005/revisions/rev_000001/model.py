from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
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
)

ASSETS = AssetContext.from_script(__file__)

SHOULDER_Z = 0.050
PIN_RADIUS = 0.004

ROOT_BASE_LEN = 0.050
ROOT_BASE_W = 0.040
ROOT_BASE_H = 0.014
ROOT_POST_LEN = 0.014
ROOT_POST_W = 0.020
ROOT_POST_H = 0.032
ROOT_EAR_LEN = 0.026
ROOT_EAR_T = 0.006
ROOT_EAR_H = 0.022
ROOT_CLEVIS_GAP = 0.010

PRIMARY_THICK = ROOT_CLEVIS_GAP
PRIMARY_OUTER_R = 0.010
PRIMARY_BAR_H = 0.010
PRIMARY_SPAN = 0.078
PRIMARY_FORK_EAR_T = 0.003
PRIMARY_FORK_GAP = 0.006
PRIMARY_FORK_LEN = 0.020

SECONDARY_THICK = PRIMARY_FORK_GAP
SECONDARY_OUTER_R = 0.009
SECONDARY_BAR_H = 0.009
SECONDARY_BAR_LEN = 0.120
TAB_LEN = 0.028
TAB_H = 0.020

FOLDED_SHOULDER = -1.55
FOLDED_ELBOW = -2.55


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_linkage", assets=ASSETS)

    steel = model.material("steel", rgba=(0.58, 0.60, 0.63, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.34, 0.36, 0.39, 1.0))
    anodized = model.material("anodized", rgba=(0.47, 0.50, 0.55, 1.0))
    tab_finish = model.material("tab_finish", rgba=(0.72, 0.56, 0.28, 1.0))

    root = model.part("root_clevis")
    root.visual(
        Box((ROOT_BASE_LEN, ROOT_BASE_W, ROOT_BASE_H)),
        origin=Origin(xyz=(-0.026, 0.0, ROOT_BASE_H / 2.0)),
        material=dark_steel,
        name="base_block",
    )
    root.visual(
        Box((ROOT_POST_LEN, ROOT_POST_W, ROOT_POST_H)),
        origin=Origin(xyz=(-0.026, 0.0, ROOT_BASE_H + ROOT_POST_H / 2.0)),
        material=dark_steel,
        name="support_post",
    )
    root.visual(
        Box((ROOT_EAR_LEN, ROOT_EAR_T, ROOT_EAR_H)),
        origin=Origin(
            xyz=(-0.009, ROOT_CLEVIS_GAP / 2.0 + ROOT_EAR_T / 2.0, SHOULDER_Z),
        ),
        material=dark_steel,
        name="left_cheek",
    )
    root.visual(
        Box((ROOT_EAR_LEN, ROOT_EAR_T, ROOT_EAR_H)),
        origin=Origin(
            xyz=(-0.009, -(ROOT_CLEVIS_GAP / 2.0 + ROOT_EAR_T / 2.0), SHOULDER_Z),
        ),
        material=dark_steel,
        name="right_cheek",
    )
    root.inertial = Inertial.from_geometry(
        Box((ROOT_BASE_LEN, ROOT_BASE_W, SHOULDER_Z + ROOT_EAR_H / 2.0)),
        mass=0.85,
        origin=Origin(xyz=(-0.020, 0.0, (SHOULDER_Z + ROOT_EAR_H / 2.0) / 2.0)),
    )

    primary = model.part("primary_link")
    primary.visual(
        Cylinder(radius=PRIMARY_OUTER_R, length=PRIMARY_THICK),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(1.57079632679, 0.0, 0.0)),
        material=steel,
        name="shoulder_lug",
    )
    primary.visual(
        Box((PRIMARY_SPAN - PRIMARY_OUTER_R - 0.010, PRIMARY_THICK, PRIMARY_BAR_H)),
        origin=Origin(xyz=((PRIMARY_OUTER_R + PRIMARY_SPAN - 0.010) / 2.0, 0.0, 0.0)),
        material=steel,
        name="main_bar",
    )
    primary.visual(
        Box((PRIMARY_FORK_LEN, PRIMARY_FORK_EAR_T, 0.014)),
        origin=Origin(
            xyz=(PRIMARY_SPAN, PRIMARY_FORK_GAP / 2.0 + PRIMARY_FORK_EAR_T / 2.0, 0.0),
        ),
        material=steel,
        name="left_fork",
    )
    primary.visual(
        Box((PRIMARY_FORK_LEN, PRIMARY_FORK_EAR_T, 0.014)),
        origin=Origin(
            xyz=(PRIMARY_SPAN, -(PRIMARY_FORK_GAP / 2.0 + PRIMARY_FORK_EAR_T / 2.0), 0.0),
        ),
        material=steel,
        name="right_fork",
    )
    primary.inertial = Inertial.from_geometry(
        Box((PRIMARY_SPAN + 0.012, PRIMARY_THICK, 0.016)),
        mass=0.22,
        origin=Origin(xyz=((PRIMARY_SPAN + 0.012) / 2.0, 0.0, 0.0)),
    )

    secondary = model.part("secondary_link")
    secondary.visual(
        Cylinder(radius=SECONDARY_OUTER_R, length=SECONDARY_THICK),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(1.57079632679, 0.0, 0.0)),
        material=anodized,
        name="elbow_lug",
    )
    secondary.visual(
        Box((SECONDARY_BAR_LEN - SECONDARY_OUTER_R, SECONDARY_THICK, SECONDARY_BAR_H)),
        origin=Origin(xyz=((SECONDARY_OUTER_R + SECONDARY_BAR_LEN) / 2.0, 0.0, 0.0)),
        material=anodized,
        name="secondary_bar",
    )
    secondary.visual(
        Box((TAB_LEN, SECONDARY_THICK, TAB_H)),
        origin=Origin(xyz=(SECONDARY_BAR_LEN + TAB_LEN / 2.0, 0.0, 0.0)),
        material=tab_finish,
        name="end_tab",
    )
    secondary.inertial = Inertial.from_geometry(
        Box((SECONDARY_BAR_LEN + TAB_LEN, SECONDARY_THICK, TAB_H)),
        mass=0.26,
        origin=Origin(xyz=((SECONDARY_BAR_LEN + TAB_LEN) / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "shoulder_joint",
        ArticulationType.REVOLUTE,
        parent=root,
        child=primary,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=FOLDED_SHOULDER, upper=0.4),
    )
    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent=primary,
        child=secondary,
        origin=Origin(xyz=(PRIMARY_SPAN, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=3.0, lower=-2.6, upper=0.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    root = object_model.get_part("root_clevis")
    primary = object_model.get_part("primary_link")
    secondary = object_model.get_part("secondary_link")
    shoulder = object_model.get_articulation("shoulder_joint")
    elbow = object_model.get_articulation("elbow_joint")
    root_left = root.get_visual("left_cheek")
    root_right = root.get_visual("right_cheek")
    root_base = root.get_visual("base_block")
    primary_lug = primary.get_visual("shoulder_lug")
    primary_bar = primary.get_visual("main_bar")
    primary_left_fork = primary.get_visual("left_fork")
    primary_right_fork = primary.get_visual("right_fork")
    secondary_lug = secondary.get_visual("elbow_lug")
    secondary_bar = secondary.get_visual("secondary_bar")
    end_tab = secondary.get_visual("end_tab")

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
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=24, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_overlap(
        primary,
        root,
        axes="xz",
        min_overlap=0.0135,
        elem_a=primary_lug,
        elem_b=root_left,
        name="primary_lug_aligned_with_left_cheek",
    )
    ctx.expect_overlap(
        primary,
        root,
        axes="xz",
        min_overlap=0.0135,
        elem_a=primary_lug,
        elem_b=root_right,
        name="primary_lug_aligned_with_right_cheek",
    )
    ctx.expect_contact(
        primary,
        root,
        elem_a=primary_lug,
        elem_b=root_left,
        name="left_cheek_contact",
    )
    ctx.expect_contact(
        primary,
        root,
        elem_a=primary_lug,
        elem_b=root_right,
        name="right_cheek_contact",
    )
    ctx.expect_overlap(
        secondary,
        primary,
        axes="xz",
        min_overlap=0.0135,
        elem_a=secondary_lug,
        elem_b=primary_left_fork,
        name="secondary_lug_aligned_with_left_fork",
    )
    ctx.expect_overlap(
        secondary,
        primary,
        axes="xz",
        min_overlap=0.0135,
        elem_a=secondary_lug,
        elem_b=primary_right_fork,
        name="secondary_lug_aligned_with_right_fork",
    )
    ctx.expect_contact(
        secondary,
        primary,
        elem_a=secondary_lug,
        elem_b=primary_left_fork,
        name="left_fork_contact",
    )
    ctx.expect_contact(
        secondary,
        primary,
        elem_a=secondary_lug,
        elem_b=primary_right_fork,
        name="right_fork_contact",
    )
    ctx.check(
        "parallel_joint_axes",
        tuple(round(v, 6) for v in shoulder.axis) == (0.0, 1.0, 0.0)
        and tuple(round(v, 6) for v in elbow.axis) == (0.0, 1.0, 0.0),
        details=f"shoulder axis={shoulder.axis}, elbow axis={elbow.axis}",
    )

    with ctx.pose({shoulder: 0.0, elbow: 0.0}):
        ctx.expect_origin_gap(
            secondary,
            primary,
            axis="x",
            min_gap=PRIMARY_SPAN - 0.001,
            max_gap=PRIMARY_SPAN + 0.001,
            name="extended_elbow_axis_offset",
        )
        tab_aabb = ctx.part_element_world_aabb(secondary, elem=end_tab)
        ctx.check(
            "extended_tab_reaches_far_forward",
            tab_aabb is not None and tab_aabb[1][0] > 0.21,
            details=f"end tab aabb in extended pose: {tab_aabb}",
        )
        ctx.expect_contact(
            primary,
            root,
            elem_a=primary_lug,
            elem_b=root_left,
            name="left_cheek_contact_extended",
        )
        ctx.expect_contact(
            primary,
            root,
            elem_a=primary_lug,
            elem_b=root_right,
            name="right_cheek_contact_extended",
        )

    with ctx.pose({shoulder: FOLDED_SHOULDER, elbow: FOLDED_ELBOW}):
        root_aabb = ctx.part_world_aabb(root)
        tab_aabb = ctx.part_element_world_aabb(secondary, elem=end_tab)
        ctx.check(
            "folded_tab_returns_toward_base",
            root_aabb is not None and tab_aabb is not None and tab_aabb[0][0] < root_aabb[1][0] + 0.02,
            details=f"root aabb={root_aabb}, end tab aabb={tab_aabb}",
        )
        ctx.check(
            "folded_tab_stays_close_to_base_height",
            root_aabb is not None and tab_aabb is not None and tab_aabb[0][2] < root_aabb[1][2] + 0.03,
            details=f"root aabb={root_aabb}, end tab aabb={tab_aabb}",
        )
        ctx.expect_contact(
            primary,
            root,
            elem_a=primary_lug,
            elem_b=root_left,
            name="left_cheek_contact_folded",
        )
        ctx.expect_contact(
            secondary,
            primary,
            elem_a=secondary_lug,
            elem_b=primary_left_fork,
            name="left_fork_contact_folded",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
