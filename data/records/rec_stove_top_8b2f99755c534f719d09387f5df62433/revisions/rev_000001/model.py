from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BODY_W = 0.29
BODY_D = 0.51
BODY_H = 0.046
TOP_T = 0.004
TOP_Z = BODY_H + TOP_T / 2.0

BURNER_ZONE_X = -0.035
BURNER_ZONE_Y = -0.005
BURNER_ZONE_W = 0.186
BURNER_ZONE_D = 0.370
BURNER_ZONE_T = 0.004
BURNER_ZONE_Z = 0.050

LID_W = 0.182
LID_D = 0.350
LID_T = 0.004
LID_AXIS_X = BURNER_ZONE_X
LID_AXIS_Y = 0.178
LID_AXIS_Z = 0.075

KNOB_X = 0.086
KNOB_YS = (-0.175, -0.105)
KNOB_COLLAR_R = 0.015
KNOB_COLLAR_H = 0.004
KNOB_COLLAR_Z = 0.052


def _add_burner_stack(
    body,
    *,
    label: str,
    center_x: float,
    center_y: float,
    base_radius: float,
    spreader_radius: float,
    cap_radius: float,
) -> None:
    body.visual(
        Cylinder(radius=base_radius, length=0.004),
        origin=Origin(xyz=(center_x, center_y, 0.054)),
        material="burner_black",
        name=f"{label}_base",
    )
    body.visual(
        Cylinder(radius=spreader_radius, length=0.004),
        origin=Origin(xyz=(center_x, center_y, 0.058)),
        material="burner_black",
        name=f"{label}_spreader",
    )
    body.visual(
        Cylinder(radius=cap_radius, length=0.010),
        origin=Origin(xyz=(center_x, center_y, 0.065)),
        material="burner_black",
        name=f"{label}_cap",
    )


def _add_grate(body) -> None:
    rail_h = 0.007
    rail_z = 0.068
    foot_h = 0.014
    foot_z = 0.059
    outer_w = 0.162
    outer_d = 0.338
    rail_t = 0.010
    half_w = outer_w / 2.0 - rail_t / 2.0
    half_d = outer_d / 2.0 - rail_t / 2.0

    body.visual(
        Box((rail_t, outer_d, rail_h)),
        origin=Origin(xyz=(BURNER_ZONE_X - half_w, BURNER_ZONE_Y, rail_z)),
        material="cast_iron",
        name="grate_side_0",
    )
    body.visual(
        Box((rail_t, outer_d, rail_h)),
        origin=Origin(xyz=(BURNER_ZONE_X + half_w, BURNER_ZONE_Y, rail_z)),
        material="cast_iron",
        name="grate_side_1",
    )
    body.visual(
        Box((outer_w, rail_t, rail_h)),
        origin=Origin(xyz=(BURNER_ZONE_X, BURNER_ZONE_Y - half_d, rail_z)),
        material="cast_iron",
        name="grate_end_front",
    )
    body.visual(
        Box((outer_w, rail_t, rail_h)),
        origin=Origin(xyz=(BURNER_ZONE_X, BURNER_ZONE_Y + half_d, rail_z)),
        material="cast_iron",
        name="grate_end_rear",
    )
    body.visual(
        Box((0.146, rail_t, rail_h)),
        origin=Origin(xyz=(BURNER_ZONE_X, -0.118, rail_z)),
        material="cast_iron",
        name="grate_bridge_front",
    )
    body.visual(
        Box((0.146, rail_t, rail_h)),
        origin=Origin(xyz=(BURNER_ZONE_X, 0.102, rail_z)),
        material="cast_iron",
        name="grate_bridge_rear",
    )
    body.visual(
        Box((rail_t, 0.250, rail_h)),
        origin=Origin(xyz=(BURNER_ZONE_X, BURNER_ZONE_Y, rail_z)),
        material="cast_iron",
        name="grate_spine",
    )

    foot_positions = (
        (BURNER_ZONE_X - half_w, BURNER_ZONE_Y - half_d),
        (BURNER_ZONE_X + half_w, BURNER_ZONE_Y - half_d),
        (BURNER_ZONE_X - half_w, BURNER_ZONE_Y + half_d),
        (BURNER_ZONE_X + half_w, BURNER_ZONE_Y + half_d),
    )
    for index, (x_pos, y_pos) in enumerate(foot_positions):
        body.visual(
            Box((0.014, 0.014, foot_h)),
            origin=Origin(xyz=(x_pos, y_pos, foot_z)),
            material="cast_iron",
            name=f"grate_foot_{index}",
        )


def _build_knob_part(model: ArticulatedObject, part_name: str) -> object:
    knob = model.part(part_name)
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.038,
                0.020,
                body_style="tapered",
                top_diameter=0.031,
                base_diameter=0.038,
                crown_radius=0.003,
                edge_radius=0.0015,
                side_draft_deg=6.0,
                center=False,
            ),
            f"{part_name}_shell",
        ),
        material="knob_black",
        name="knob_shell",
    )
    knob.visual(
        Cylinder(radius=0.024, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material="knob_black",
        name="skirt_ring",
    )
    knob.visual(
        Box((0.004, 0.015, 0.0015)),
        origin=Origin(xyz=(0.0, -0.0075, 0.02075)),
        material="knob_mark",
        name="pointer",
    )
    return knob


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_burner_domino_gas_stovetop")

    model.material("stainless", rgba=(0.74, 0.76, 0.78, 1.0))
    model.material("enamel_black", rgba=(0.13, 0.13, 0.14, 1.0))
    model.material("burner_black", rgba=(0.10, 0.10, 0.10, 1.0))
    model.material("cast_iron", rgba=(0.18, 0.18, 0.19, 1.0))
    model.material("glass", rgba=(0.24, 0.32, 0.37, 0.38))
    model.material("trim_metal", rgba=(0.62, 0.64, 0.67, 1.0))
    model.material("knob_black", rgba=(0.09, 0.09, 0.10, 1.0))
    model.material("knob_mark", rgba=(0.88, 0.89, 0.90, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_W, BODY_D, BODY_H)),
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0)),
        material="stainless",
        name="shell",
    )
    body.visual(
        Box((BODY_W, BODY_D, TOP_T)),
        origin=Origin(xyz=(0.0, 0.0, TOP_Z)),
        material="stainless",
        name="top_plate",
    )
    body.visual(
        Box((BURNER_ZONE_W, BURNER_ZONE_D, BURNER_ZONE_T)),
        origin=Origin(xyz=(BURNER_ZONE_X, BURNER_ZONE_Y, BURNER_ZONE_Z)),
        material="enamel_black",
        name="burner_zone",
    )
    body.visual(
        Box((LID_W + 0.004, 0.014, 0.020)),
        origin=Origin(xyz=(LID_AXIS_X, 0.171, 0.060)),
        material="trim_metal",
        name="hinge_support",
    )

    body.visual(
        Cylinder(radius=KNOB_COLLAR_R, length=KNOB_COLLAR_H),
        origin=Origin(xyz=(KNOB_X, KNOB_YS[0], KNOB_COLLAR_Z)),
        material="stainless",
        name="knob_collar_0",
    )
    body.visual(
        Cylinder(radius=KNOB_COLLAR_R, length=KNOB_COLLAR_H),
        origin=Origin(xyz=(KNOB_X, KNOB_YS[1], KNOB_COLLAR_Z)),
        material="stainless",
        name="knob_collar_1",
    )

    _add_burner_stack(
        body,
        label="front_burner",
        center_x=BURNER_ZONE_X,
        center_y=-0.118,
        base_radius=0.048,
        spreader_radius=0.040,
        cap_radius=0.023,
    )
    _add_burner_stack(
        body,
        label="rear_burner",
        center_x=BURNER_ZONE_X,
        center_y=0.102,
        base_radius=0.060,
        spreader_radius=0.050,
        cap_radius=0.027,
    )
    _add_grate(body)

    lid = model.part("lid")
    lid.visual(
        Cylinder(radius=0.005, length=LID_W),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 1.5707963267948966, 0.0)),
        material="trim_metal",
        name="hinge_barrel",
    )
    lid.visual(
        Box((LID_W, 0.026, 0.002)),
        origin=Origin(xyz=(0.0, -0.013, 0.000)),
        material="trim_metal",
        name="rear_leaf",
    )
    lid.visual(
        Box((LID_W - 0.008, LID_D, LID_T)),
        origin=Origin(xyz=(0.0, -LID_D / 2.0, LID_T / 2.0)),
        material="glass",
        name="glass_panel",
    )
    lid.visual(
        Box((0.120, 0.014, 0.008)),
        origin=Origin(xyz=(0.0, -LID_D + 0.007, 0.006)),
        material="trim_metal",
        name="front_trim",
    )

    knob_0 = _build_knob_part(model, "knob_0")
    knob_1 = _build_knob_part(model, "knob_1")

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(LID_AXIS_X, LID_AXIS_Y, LID_AXIS_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.8, lower=0.0, upper=1.38),
    )
    model.articulation(
        "knob_0_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=knob_0,
        origin=Origin(xyz=(KNOB_X, KNOB_YS[0], KNOB_COLLAR_Z + KNOB_COLLAR_H / 2.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=8.0),
    )
    model.articulation(
        "knob_1_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=knob_1,
        origin=Origin(xyz=(KNOB_X, KNOB_YS[1], KNOB_COLLAR_Z + KNOB_COLLAR_H / 2.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    knob_0 = object_model.get_part("knob_0")
    knob_1 = object_model.get_part("knob_1")
    lid_hinge = object_model.get_articulation("lid_hinge")
    knob_0_spin = object_model.get_articulation("knob_0_spin")
    knob_1_spin = object_model.get_articulation("knob_1_spin")

    ctx.check(
        "lid uses rear revolute hinge",
        lid_hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(lid_hinge.axis) == (-1.0, 0.0, 0.0),
        details=f"type={lid_hinge.articulation_type!r}, axis={lid_hinge.axis!r}",
    )
    ctx.check(
        "front knobs are continuous",
        knob_0_spin.articulation_type == ArticulationType.CONTINUOUS
        and knob_1_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(knob_0_spin.axis) == (0.0, 0.0, 1.0)
        and tuple(knob_1_spin.axis) == (0.0, 0.0, 1.0),
        details=(
            f"knob_0={knob_0_spin.articulation_type!r}/{knob_0_spin.axis!r}, "
            f"knob_1={knob_1_spin.articulation_type!r}/{knob_1_spin.axis!r}"
        ),
    )

    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="glass_panel",
        elem_b="burner_zone",
        min_overlap=0.15,
        name="lid covers the burner zone",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="glass_panel",
        negative_elem="grate_bridge_rear",
        min_gap=0.002,
        max_gap=0.006,
        name="closed lid clears the grate",
    )
    ctx.expect_gap(
        knob_0,
        body,
        axis="z",
        positive_elem="knob_shell",
        negative_elem="knob_collar_0",
        min_gap=0.0,
        max_gap=0.001,
        name="front knob seats on its collar",
    )
    ctx.expect_gap(
        knob_1,
        body,
        axis="z",
        positive_elem="knob_shell",
        negative_elem="knob_collar_1",
        min_gap=0.0,
        max_gap=0.001,
        name="rear knob seats on its collar",
    )

    closed_aabb = ctx.part_element_world_aabb(lid, elem="front_trim")
    with ctx.pose({lid_hinge: 1.25}):
        open_aabb = ctx.part_element_world_aabb(lid, elem="front_trim")
    ctx.check(
        "lid front edge lifts when opened",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.16,
        details=f"closed={closed_aabb!r}, open={open_aabb!r}",
    )

    return ctx.report()


object_model = build_object_model()
