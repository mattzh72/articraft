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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


ROOT_AXIS_Z = 0.420
BEAM_BOTTOM_Z = 0.472
BEAM_THICKNESS = 0.070
FOOT_THICKNESS = 0.025


def add_box(
    part,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material,
    *,
    name: str | None = None,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def add_y_cylinder(
    part,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material,
    *,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_mounted_revolute_chain")

    support_mat = model.material("support_gray", rgba=(0.22, 0.24, 0.27, 1.0))
    clevis_mat = model.material("clevis_steel", rgba=(0.42, 0.45, 0.48, 1.0))
    link1_mat = model.material("link_one_steel", rgba=(0.63, 0.66, 0.70, 1.0))
    link2_mat = model.material("link_two_steel", rgba=(0.53, 0.57, 0.62, 1.0))
    tab_mat = model.material("tab_dark", rgba=(0.28, 0.30, 0.34, 1.0))

    bridge_support = model.part("bridge_support")
    leg_height = BEAM_BOTTOM_Z - FOOT_THICKNESS
    leg_center_z = FOOT_THICKNESS + leg_height / 2.0
    beam_center_z = BEAM_BOTTOM_Z + BEAM_THICKNESS / 2.0
    for side, x_sign in (("left", -1.0), ("right", 1.0)):
        x = x_sign * 0.210
        add_box(
            bridge_support,
            (0.160, 0.220, FOOT_THICKNESS),
            (x, 0.0, FOOT_THICKNESS / 2.0),
            support_mat,
            name=f"{side}_foot",
        )
        add_box(
            bridge_support,
            (0.060, 0.140, leg_height),
            (x, 0.0, leg_center_z),
            support_mat,
            name=f"{side}_leg",
        )
    add_box(
        bridge_support,
        (0.580, 0.140, BEAM_THICKNESS),
        (0.0, 0.0, beam_center_z),
        support_mat,
        name="top_beam",
    )

    root_clevis = model.part("root_clevis")
    add_box(
        root_clevis,
        (0.072, 0.032, 0.008),
        (0.0, 0.0, 0.048),
        clevis_mat,
        name="mount_pad",
    )
    add_box(
        root_clevis,
        (0.032, 0.024, 0.030),
        (0.0, 0.0, 0.029),
        clevis_mat,
        name="neck",
    )
    for side, y in (("left", -0.0195), ("right", 0.0195)):
        add_box(
            root_clevis,
            (0.026, 0.013, 0.026),
            (0.0, y, 0.013),
            clevis_mat,
            name=f"{side}_cheek",
        )
        add_y_cylinder(
            root_clevis,
            radius=0.018,
            length=0.013,
            xyz=(0.0, y, 0.0),
            material=clevis_mat,
            name=f"{side}_barrel",
        )
    for side, y in (("left", -0.0125), ("right", 0.0125)):
        add_box(
            root_clevis,
            (0.022, 0.003, 0.018),
            (0.0, y, 0.031),
            clevis_mat,
            name=f"{side}_web",
        )

    link_1 = model.part("link_1")
    add_y_cylinder(
        link_1,
        radius=0.018,
        length=0.026,
        xyz=(0.0, 0.0, 0.0),
        material=link1_mat,
        name="proximal_barrel",
    )
    add_box(
        link_1,
        (0.032, 0.026, 0.076),
        (0.0, 0.0, -0.055),
        link1_mat,
        name="upper_body",
    )
    add_box(
        link_1,
        (0.026, 0.022, 0.060),
        (0.0, 0.0, -0.122),
        link1_mat,
        name="lower_body",
    )
    for side, y in (("left", -0.016), ("right", 0.016)):
        add_box(
            link_1,
            (0.020, 0.010, 0.036),
            (0.0, y, -0.162),
            link1_mat,
            name=f"{side}_distal_cheek",
        )
        add_y_cylinder(
            link_1,
            radius=0.015,
            length=0.010,
            xyz=(0.0, y, -0.180),
            material=link1_mat,
            name=f"{side}_distal_barrel",
        )

    link_2 = model.part("link_2")
    add_y_cylinder(
        link_2,
        radius=0.015,
        length=0.022,
        xyz=(0.0, 0.0, 0.0),
        material=link2_mat,
        name="proximal_barrel",
    )
    add_box(
        link_2,
        (0.026, 0.022, 0.064),
        (0.0, 0.0, -0.047),
        link2_mat,
        name="upper_body",
    )
    add_box(
        link_2,
        (0.018, 0.016, 0.012),
        (0.0, 0.0, -0.017),
        link2_mat,
        name="joint_collar",
    )
    add_box(
        link_2,
        (0.020, 0.018, 0.052),
        (0.0, 0.0, -0.104),
        link2_mat,
        name="lower_body",
    )
    for side, y in (("left", -0.0135), ("right", 0.0135)):
        add_box(
            link_2,
            (0.016, 0.009, 0.030),
            (0.0, y, -0.135),
            link2_mat,
            name=f"{side}_distal_cheek",
        )
        add_y_cylinder(
            link_2,
            radius=0.012,
            length=0.009,
            xyz=(0.0, y, -0.150),
            material=link2_mat,
            name=f"{side}_distal_barrel",
        )

    end_tab = model.part("end_tab")
    add_y_cylinder(
        end_tab,
        radius=0.012,
        length=0.018,
        xyz=(0.0, 0.0, 0.0),
        material=tab_mat,
        name="proximal_barrel",
    )
    add_box(
        end_tab,
        (0.018, 0.018, 0.040),
        (0.0, 0.0, -0.032),
        tab_mat,
        name="neck",
    )
    add_box(
        end_tab,
        (0.014, 0.014, 0.014),
        (0.0, 0.0, -0.015),
        tab_mat,
        name="root_collar",
    )
    add_box(
        end_tab,
        (0.014, 0.014, 0.032),
        (0.0, 0.0, -0.067),
        tab_mat,
        name="tab_body",
    )
    add_box(
        end_tab,
        (0.020, 0.014, 0.012),
        (0.0, 0.0, -0.089),
        tab_mat,
        name="tip_pad",
    )
    add_y_cylinder(
        end_tab,
        radius=0.007,
        length=0.014,
        xyz=(0.0, 0.0, -0.095),
        material=tab_mat,
        name="tip_round",
    )

    model.articulation(
        "support_to_root_clevis",
        ArticulationType.FIXED,
        parent=bridge_support,
        child=root_clevis,
        origin=Origin(xyz=(0.0, 0.0, ROOT_AXIS_Z)),
    )
    model.articulation(
        "root_to_link_1",
        ArticulationType.REVOLUTE,
        parent=root_clevis,
        child=link_1,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.0,
            lower=-1.20,
            upper=1.20,
        ),
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(0.0, 0.0, -0.180)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=2.2,
            lower=-1.05,
            upper=1.05,
        ),
    )
    model.articulation(
        "link_2_to_end_tab",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=end_tab,
        origin=Origin(xyz=(0.0, 0.0, -0.150)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.5,
            lower=-0.95,
            upper=0.95,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bridge_support = object_model.get_part("bridge_support")
    root_clevis = object_model.get_part("root_clevis")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    end_tab = object_model.get_part("end_tab")
    joint_1 = object_model.get_articulation("root_to_link_1")
    joint_2 = object_model.get_articulation("link_1_to_link_2")
    joint_3 = object_model.get_articulation("link_2_to_end_tab")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
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

    ctx.check(
        "three_revolute_joints_in_series",
        all(
            joint.articulation_type == ArticulationType.REVOLUTE
            and joint.axis == (0.0, -1.0, 0.0)
            for joint in (joint_1, joint_2, joint_3)
        ),
        details=(
            f"joint types/axes: "
            f"{[(joint.name, joint.articulation_type, joint.axis) for joint in (joint_1, joint_2, joint_3)]}"
        ),
    )

    ctx.expect_contact(bridge_support, root_clevis, contact_tol=1e-6, name="bridge_carries_root_clevis")
    ctx.expect_contact(root_clevis, link_1, contact_tol=1e-6, name="root_joint_has_physical_contact")
    ctx.expect_contact(link_1, link_2, contact_tol=1e-6, name="mid_joint_has_physical_contact")
    ctx.expect_contact(link_2, end_tab, contact_tol=1e-6, name="distal_joint_has_physical_contact")

    def span(part) -> tuple[float, float, float]:
        aabb = ctx.part_world_aabb(part)
        assert aabb is not None
        lower, upper = aabb
        return (
            upper[0] - lower[0],
            upper[1] - lower[1],
            upper[2] - lower[2],
        )

    link_1_span = span(link_1)
    link_2_span = span(link_2)
    end_tab_span = span(end_tab)
    ctx.check(
        "tapered_sections_toward_end_tab",
        (
            link_1_span[0] > link_2_span[0] > end_tab_span[0]
            and link_1_span[1] > link_2_span[1] > end_tab_span[1]
            and link_1_span[2] > link_2_span[2] > end_tab_span[2]
        ),
        details=(
            f"link_1 span={link_1_span}, "
            f"link_2 span={link_2_span}, "
            f"end_tab span={end_tab_span}"
        ),
    )

    rest_pos = ctx.part_world_position(end_tab)
    with ctx.pose({joint_1: 0.55, joint_2: 0.35, joint_3: 0.20}):
        swung_pos = ctx.part_world_position(end_tab)
    ctx.check(
        "positive_joint_motion_swings_chain_forward",
        (
            rest_pos is not None
            and swung_pos is not None
            and swung_pos[0] > rest_pos[0] + 0.05
            and swung_pos[2] > rest_pos[2] + 0.04
        ),
        details=f"rest end_tab origin={rest_pos}, swung end_tab origin={swung_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
