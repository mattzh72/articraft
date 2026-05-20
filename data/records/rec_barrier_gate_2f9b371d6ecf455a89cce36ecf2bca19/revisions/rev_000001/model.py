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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="railway_level_crossing_gate")

    concrete = model.material("concrete", rgba=(0.63, 0.64, 0.66, 1.0))
    mast_grey = model.material("mast_grey", rgba=(0.58, 0.60, 0.62, 1.0))
    housing_grey = model.material("housing_grey", rgba=(0.33, 0.35, 0.38, 1.0))
    warning_white = model.material("warning_white", rgba=(0.96, 0.97, 0.98, 1.0))
    warning_red = model.material("warning_red", rgba=(0.79, 0.09, 0.10, 1.0))
    sign_yellow = model.material("sign_yellow", rgba=(0.94, 0.76, 0.18, 1.0))
    frame_black = model.material("frame_black", rgba=(0.12, 0.12, 0.13, 1.0))
    lens_red = model.material("lens_red", rgba=(0.74, 0.08, 0.08, 1.0))

    support = model.part("support")
    support.visual(
        Box((0.72, 0.72, 0.22)),
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        material=concrete,
        name="foundation",
    )
    support.visual(
        Box((0.18, 0.18, 3.00)),
        origin=Origin(xyz=(0.0, 0.0, 1.72)),
        material=mast_grey,
        name="signal_post",
    )
    support.visual(
        Box((0.34, 0.28, 0.26)),
        origin=Origin(xyz=(0.0, 0.0, 3.09)),
        material=housing_grey,
        name="gearbox_housing",
    )
    support.visual(
        Box((0.10, 0.02, 0.20)),
        origin=Origin(xyz=(0.0, 0.08, 3.32)),
        material=housing_grey,
        name="hinge_cheek_front",
    )
    support.visual(
        Box((0.10, 0.02, 0.20)),
        origin=Origin(xyz=(0.0, -0.08, 3.32)),
        material=housing_grey,
        name="hinge_cheek_back",
    )
    support.visual(
        Box((0.46, 0.05, 0.05)),
        origin=Origin(xyz=(0.0, 0.115, 2.18)),
        material=housing_grey,
        name="signal_bracket",
    )
    for side, offset_x in (("left", -0.14), ("right", 0.14)):
        support.visual(
            Cylinder(radius=0.10, length=0.16),
            origin=Origin(
                xyz=(offset_x, 0.20, 2.18),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=frame_black,
            name=f"{side}_signal_head",
        )
        support.visual(
            Cylinder(radius=0.070, length=0.02),
            origin=Origin(
                xyz=(offset_x, 0.29, 2.18),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=lens_red,
            name=f"{side}_signal_lens",
        )
    support.inertial = Inertial.from_geometry(
        Box((0.72, 0.72, 3.42)),
        mass=420.0,
        origin=Origin(xyz=(0.0, 0.0, 1.71)),
    )

    boom = model.part("boom")
    boom.visual(
        Cylinder(radius=0.07, length=0.14),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=housing_grey,
        name="pivot_barrel",
    )
    boom.visual(
        Box((4.60, 0.10, 0.12)),
        origin=Origin(xyz=(2.30, 0.0, 0.0)),
        material=warning_white,
        name="main_arm",
    )
    boom.visual(
        Box((0.62, 0.12, 0.14)),
        origin=Origin(xyz=(-0.31, 0.0, 0.0)),
        material=warning_white,
        name="rear_stub",
    )
    boom.visual(
        Box((0.34, 0.30, 0.34)),
        origin=Origin(xyz=(-0.46, 0.0, -0.24)),
        material=housing_grey,
        name="counterweight_block",
    )
    for stripe_index, stripe_x in enumerate((0.48, 0.90, 1.32, 1.74, 2.16, 2.58, 3.00, 3.42, 3.84, 4.24)):
        boom.visual(
            Box((0.20, 0.014, 0.030)),
            origin=Origin(
                xyz=(stripe_x, 0.048, 0.0),
                rpy=(0.0, math.radians(30.0), 0.0),
            ),
            material=warning_red,
            name=f"stripe_front_{stripe_index}",
        )
        boom.visual(
            Box((0.20, 0.014, 0.030)),
            origin=Origin(
                xyz=(stripe_x, -0.048, 0.0),
                rpy=(0.0, math.radians(-30.0), 0.0),
            ),
            material=warning_red,
            name=f"stripe_back_{stripe_index}",
        )
    boom.visual(
        Box((0.18, 0.08, 0.04)),
        origin=Origin(xyz=(2.65, 0.0, -0.08)),
        material=housing_grey,
        name="panel_hanger_block",
    )
    boom.visual(
        Box((0.05, 0.010, 0.14)),
        origin=Origin(xyz=(2.65, 0.037, -0.13)),
        material=housing_grey,
        name="panel_hanger_front",
    )
    boom.visual(
        Box((0.05, 0.010, 0.14)),
        origin=Origin(xyz=(2.65, -0.037, -0.13)),
        material=housing_grey,
        name="panel_hanger_back",
    )
    boom.visual(
        Box((0.12, 0.10, 0.06)),
        origin=Origin(xyz=(4.58, 0.0, 0.0)),
        material=warning_red,
        name="tip_marker",
    )
    boom.inertial = Inertial.from_geometry(
        Box((5.22, 0.30, 0.68)),
        mass=78.0,
        origin=Origin(xyz=(2.05, 0.0, -0.08)),
    )

    warning_panel = model.part("warning_panel")
    warning_panel.visual(
        Cylinder(radius=0.016, length=0.064),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=housing_grey,
        name="panel_pivot_barrel",
    )
    warning_panel.visual(
        Box((0.14, 0.04, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, -0.04)),
        material=housing_grey,
        name="top_connector",
    )
    warning_panel.visual(
        Box((0.08, 0.03, 0.22)),
        origin=Origin(xyz=(0.0, 0.0, -0.11)),
        material=housing_grey,
        name="center_hanger_web",
    )
    warning_panel.visual(
        Box((0.03, 0.02, 0.14)),
        origin=Origin(xyz=(-0.26, 0.0, -0.10)),
        material=housing_grey,
        name="left_hanger_strap",
    )
    warning_panel.visual(
        Box((0.03, 0.02, 0.14)),
        origin=Origin(xyz=(0.26, 0.0, -0.10)),
        material=housing_grey,
        name="right_hanger_strap",
    )
    warning_panel.visual(
        Box((0.90, 0.04, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, -0.19)),
        material=frame_black,
        name="top_rail",
    )
    warning_panel.visual(
        Box((0.90, 0.04, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, -0.67)),
        material=frame_black,
        name="bottom_rail",
    )
    warning_panel.visual(
        Box((0.05, 0.04, 0.54)),
        origin=Origin(xyz=(-0.425, 0.0, -0.43)),
        material=frame_black,
        name="left_rail",
    )
    warning_panel.visual(
        Box((0.05, 0.04, 0.54)),
        origin=Origin(xyz=(0.425, 0.0, -0.43)),
        material=frame_black,
        name="right_rail",
    )
    warning_panel.visual(
        Box((0.80, 0.014, 0.44)),
        origin=Origin(xyz=(0.0, 0.0, -0.43)),
        material=sign_yellow,
        name="sign_face",
    )
    warning_panel.visual(
        Box((0.80, 0.016, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, -0.225)),
        material=warning_red,
        name="sign_border_top",
    )
    warning_panel.visual(
        Box((0.80, 0.016, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, -0.635)),
        material=warning_red,
        name="sign_border_bottom",
    )
    warning_panel.visual(
        Box((0.05, 0.016, 0.44)),
        origin=Origin(xyz=(-0.375, 0.0, -0.43)),
        material=warning_red,
        name="sign_border_left",
    )
    warning_panel.visual(
        Box((0.05, 0.016, 0.44)),
        origin=Origin(xyz=(0.375, 0.0, -0.43)),
        material=warning_red,
        name="sign_border_right",
    )
    warning_panel.inertial = Inertial.from_geometry(
        Box((0.90, 0.05, 0.70)),
        mass=22.0,
        origin=Origin(xyz=(0.0, 0.0, -0.35)),
    )

    model.articulation(
        "support_to_boom",
        ArticulationType.REVOLUTE,
        parent=support,
        child=boom,
        origin=Origin(xyz=(0.0, 0.0, 3.33)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.6,
            lower=0.0,
            upper=1.30,
        ),
    )
    model.articulation(
        "boom_to_panel",
        ArticulationType.REVOLUTE,
        parent=boom,
        child=warning_panel,
        origin=Origin(xyz=(2.65, 0.0, -0.18)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.0,
            lower=-1.25,
            upper=1.25,
        ),
    )

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    return tuple((aabb[0][index] + aabb[1][index]) * 0.5 for index in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    boom = object_model.get_part("boom")
    panel = object_model.get_part("warning_panel")
    boom_joint = object_model.get_articulation("support_to_boom")
    panel_joint = object_model.get_articulation("boom_to_panel")

    with ctx.pose({boom_joint: 0.0, panel_joint: 0.0}):
        ctx.expect_gap(
            boom,
            panel,
            axis="z",
            positive_elem="main_arm",
            negative_elem="sign_face",
            min_gap=0.20,
            max_gap=0.40,
            name="warning panel hangs below the boom arm",
        )
        ctx.expect_overlap(
            boom,
            panel,
            axes="x",
            elem_a="main_arm",
            elem_b="sign_face",
            min_overlap=0.78,
            name="warning panel sits under the boom mid-span",
        )
        ctx.expect_overlap(
            boom,
            panel,
            axes="y",
            elem_a="main_arm",
            elem_b="sign_face",
            min_overlap=0.012,
            name="warning panel remains centered under the arm thickness",
        )

    with ctx.pose({boom_joint: 0.0}):
        rest_tip = ctx.part_element_world_aabb(boom, elem="tip_marker")
    with ctx.pose({boom_joint: 1.20}):
        raised_tip = ctx.part_element_world_aabb(boom, elem="tip_marker")
    ctx.check(
        "boom arm lifts upward from the closed position",
        rest_tip is not None
        and raised_tip is not None
        and raised_tip[0][2] > rest_tip[1][2] + 2.5,
        details=f"rest_tip={rest_tip}, raised_tip={raised_tip}",
    )

    with ctx.pose({boom_joint: 0.0, panel_joint: 0.0}):
        panel_rest = ctx.part_element_world_aabb(panel, elem="sign_face")
    with ctx.pose({boom_joint: 0.0, panel_joint: 0.90}):
        panel_swung = ctx.part_element_world_aabb(panel, elem="sign_face")
    panel_rest_center = _aabb_center(panel_rest)
    panel_swung_center = _aabb_center(panel_swung)
    ctx.check(
        "warning panel can swing on its transverse hinge",
        panel_rest_center is not None
        and panel_swung_center is not None
        and abs(panel_swung_center[0] - panel_rest_center[0]) > 0.18
        and panel_swung_center[2] > panel_rest_center[2] + 0.10,
        details=f"rest_center={panel_rest_center}, swung_center={panel_swung_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
