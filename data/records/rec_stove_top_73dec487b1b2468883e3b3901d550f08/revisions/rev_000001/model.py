from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

BODY_W = 0.54
BODY_D = 0.31
BODY_H = 0.076
FOOT_H = 0.006
CORNER_R = 0.024

CONTROL_PANEL_T = 0.003
CONTROL_PANEL_W = 0.38
CONTROL_PANEL_H = 0.044
CONTROL_PANEL_Z = FOOT_H + 0.030

BURNER_X = 0.135
BURNER_Y = 0.055
TRAY_R = 0.084
TRAY_T = 0.002
PLATE_R = 0.072
PLATE_T = 0.008

HINGE_R = 0.0045
HINGE_Y = BODY_D / 2.0 - 0.002
HINGE_Z = FOOT_H + BODY_H + HINGE_R + 0.001

LID_W = BODY_W - 0.020
LID_D = BODY_D - 0.018
LID_T = 0.016
LID_BOTTOM_CLEARANCE = 0.006

KNOB_Z = FOOT_H + 0.031
KNOB_X = 0.112


def _rounded_panel_mesh(width: float, depth: float, thickness: float, radius: float, name: str):
    return mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(width, depth, radius),
            thickness,
            cap=True,
            closed=True,
        ),
        name,
    )


def _knob_mesh(name: str):
    return mesh_from_geometry(
        KnobGeometry(
            0.042,
            0.022,
            body_style="skirted",
            top_diameter=0.034,
            skirt=KnobSkirt(0.054, 0.006, flare=0.06),
            grip=KnobGrip(style="fluted", count=16, depth=0.0011),
            indicator=KnobIndicator(
                style="line",
                mode="engraved",
                depth=0.0008,
                angle_deg=90.0,
            ),
            center=False,
        ),
        name,
    )


def _add_knob(model: ArticulatedObject, body, index: int, x_pos: float, knob_finish) -> None:
    knob = model.part(f"knob_{index}")
    knob.visual(
        Cylinder(radius=0.004, length=0.007),
        origin=Origin(xyz=(0.0, 0.0, 0.0035)),
        material=knob_finish,
        name="shaft",
    )
    knob.visual(
        _knob_mesh(f"knob_{index}_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.0035)),
        material=knob_finish,
        name="knob_shell",
    )

    model.articulation(
        f"body_to_knob_{index}",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=knob,
        origin=Origin(
            xyz=(x_pos, -BODY_D / 2.0 - CONTROL_PANEL_T, KNOB_Z),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=10.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_two_burner_hotplate")

    enamel = model.material("enamel", rgba=(0.93, 0.93, 0.90, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.22, 0.22, 0.23, 1.0))
    plate_finish = model.material("plate_finish", rgba=(0.14, 0.14, 0.15, 1.0))
    knob_finish = model.material("knob_finish", rgba=(0.10, 0.10, 0.11, 1.0))
    foot_finish = model.material("foot_finish", rgba=(0.12, 0.12, 0.12, 1.0))

    body = model.part("body")
    body.visual(
        _rounded_panel_mesh(BODY_W, BODY_D, BODY_H, CORNER_R, "body_shell"),
        origin=Origin(xyz=(0.0, 0.0, FOOT_H)),
        material=enamel,
        name="body_shell",
    )
    body.visual(
        Box((CONTROL_PANEL_W, CONTROL_PANEL_T, CONTROL_PANEL_H)),
        origin=Origin(
            xyz=(
                0.0,
                -BODY_D / 2.0 - CONTROL_PANEL_T / 2.0,
                CONTROL_PANEL_Z,
            )
        ),
        material=dark_trim,
        name="control_panel",
    )

    foot_positions = (
        (-BODY_W / 2.0 + 0.060, -BODY_D / 2.0 + 0.055),
        (BODY_W / 2.0 - 0.060, -BODY_D / 2.0 + 0.055),
        (-BODY_W / 2.0 + 0.060, BODY_D / 2.0 - 0.055),
        (BODY_W / 2.0 - 0.060, BODY_D / 2.0 - 0.055),
    )
    for index, (x_pos, y_pos) in enumerate(foot_positions):
        body.visual(
            Box((0.038, 0.028, FOOT_H)),
            origin=Origin(xyz=(x_pos, y_pos, FOOT_H / 2.0)),
            material=foot_finish,
            name=f"foot_{index}",
        )

    burner_positions = ((-BURNER_X, BURNER_Y), (BURNER_X, BURNER_Y))
    for index, (x_pos, y_pos) in enumerate(burner_positions):
        body.visual(
            Cylinder(radius=TRAY_R, length=TRAY_T),
            origin=Origin(xyz=(x_pos, y_pos, FOOT_H + BODY_H + TRAY_T / 2.0)),
            material=dark_trim,
            name=f"burner_tray_{index}",
        )
        body.visual(
            Cylinder(radius=PLATE_R, length=PLATE_T),
            origin=Origin(
                xyz=(
                    x_pos,
                    y_pos,
                    FOOT_H + BODY_H + TRAY_T + PLATE_T / 2.0,
                )
            ),
            material=plate_finish,
            name=f"burner_plate_{index}",
        )

    for name, x_pos, length in (
        ("hinge_knuckle_0", -0.151, 0.122),
        ("hinge_knuckle_1", 0.151, 0.122),
    ):
        body.visual(
            Box((length + 0.010, 0.012, 0.008)),
            origin=Origin(
                xyz=(
                    x_pos,
                    BODY_D / 2.0 - 0.003,
                    FOOT_H + BODY_H + 0.0005,
                )
            ),
            material=dark_trim,
            name=f"{name}_leaf",
        )
        body.visual(
            Cylinder(radius=HINGE_R, length=length),
            origin=Origin(
                xyz=(x_pos, HINGE_Y, HINGE_Z),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=dark_trim,
            name=name,
        )

    lid = model.part("lid")
    lid.visual(
        _rounded_panel_mesh(LID_W, LID_D, LID_T, CORNER_R * 0.9, "lid_panel"),
        origin=Origin(
            xyz=(
                0.0,
                -LID_D / 2.0,
                LID_BOTTOM_CLEARANCE,
            )
        ),
        material=enamel,
        name="lid_panel",
    )
    lid.visual(
        Box((LID_W - 0.070, 0.012, 0.014)),
        origin=Origin(
            xyz=(
                0.0,
                -LID_D + 0.006,
                LID_BOTTOM_CLEARANCE + 0.007,
            )
        ),
        material=enamel,
        name="front_lip",
    )
    lid.visual(
        Box((0.190, 0.012, 0.009)),
        origin=Origin(
            xyz=(
                0.0,
                -0.004,
                0.0045,
            )
        ),
        material=dark_trim,
        name="hinge_leaf",
    )
    lid.visual(
        Cylinder(radius=HINGE_R, length=0.180),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=dark_trim,
        name="hinge_knuckle",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=0.0,
            upper=1.75,
        ),
    )

    _add_knob(model, body, 0, -KNOB_X, knob_finish)
    _add_knob(model, body, 1, KNOB_X, knob_finish)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    knob_0 = object_model.get_part("knob_0")
    knob_1 = object_model.get_part("knob_1")
    lid_hinge = object_model.get_articulation("body_to_lid")
    knob_joint_0 = object_model.get_articulation("body_to_knob_0")
    knob_joint_1 = object_model.get_articulation("body_to_knob_1")

    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_panel",
        elem_b="body_shell",
        min_overlap=0.24,
        name="closed_lid_covers_body_plan",
    )
    for index in range(2):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_panel",
            negative_elem=f"burner_plate_{index}",
            min_gap=0.001,
            max_gap=0.010,
            name=f"closed_lid_clears_burner_{index}",
        )

    for knob in (knob_0, knob_1):
        ctx.expect_gap(
            body,
            knob,
            axis="y",
            positive_elem="control_panel",
            negative_elem="shaft",
            max_gap=0.001,
            max_penetration=0.0,
            name=f"{knob.name}_shaft_meets_front_panel",
        )

    for joint in (knob_joint_0, knob_joint_1):
        limits = joint.motion_limits
        joint_type = getattr(joint.articulation_type, "name", str(joint.articulation_type))
        ctx.check(
            f"{joint.name}_is_continuous",
            joint_type == "CONTINUOUS" and limits is not None and limits.lower is None and limits.upper is None,
            details=f"type={joint_type}, limits={limits!r}",
        )

    closed_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper}):
        open_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
        ctx.check(
            "lid_opens_upward",
            closed_aabb is not None
            and open_aabb is not None
            and open_aabb[1][2] > closed_aabb[1][2] + 0.14,
            details=f"closed={closed_aabb}, open={open_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
