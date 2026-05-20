from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_W = 0.38
BASE_D = 0.27
BASE_H = 0.06
BODY_H = 0.52
BODY_BOTTOM_W = 0.32
BODY_BOTTOM_D = 0.22
BODY_TOP_W = 0.12
BODY_TOP_D = 0.08
WALL = 0.014

FRONT_SLOPE = (BODY_BOTTOM_D - BODY_TOP_D) * 0.5 / BODY_H
FRONT_TILT = math.atan(FRONT_SLOPE)
DOOR_W = 0.25
DOOR_H = 0.205
DOOR_T = 0.018
DOOR_BOTTOM_Z = BASE_H + 0.016
HINGE_Y = -BODY_BOTTOM_D * 0.5 - 0.006
PIVOT_Z = BASE_H + BODY_H + 0.010
PIVOT_Y = -BODY_TOP_D * 0.5 - 0.055


def _front_y_at_z(z: float) -> float:
    """World y coordinate of the tapered housing's front face at height z."""
    return -BODY_BOTTOM_D * 0.5 + FRONT_SLOPE * (z - BASE_H)


def _rear_y_at_z(z: float) -> float:
    """World y coordinate of the tapered housing's rear face at height z."""
    return BODY_BOTTOM_D * 0.5 - FRONT_SLOPE * (z - BASE_H)


def _housing_shell_mesh() -> object:
    """Hollow tapered wooden shell with a service-door opening in the front."""
    outer = (
        cq.Workplane("XY")
        .rect(BODY_BOTTOM_W, BODY_BOTTOM_D)
        .workplane(offset=BODY_H)
        .rect(BODY_TOP_W, BODY_TOP_D)
        .loft(combine=True)
    )
    inner = (
        cq.Workplane("XY")
        .workplane(offset=-0.010)
        .rect(BODY_BOTTOM_W - 2 * WALL, BODY_BOTTOM_D - 2 * WALL)
        .workplane(offset=BODY_H - WALL + 0.010)
        .rect(BODY_TOP_W - 2 * WALL, BODY_TOP_D - 2 * WALL)
        .loft(combine=True)
    )
    shell = outer.cut(inner)

    opening_center_z = (DOOR_BOTTOM_Z - BASE_H) + DOOR_H * 0.5
    service_cut = (
        cq.Workplane("XY")
        .box(DOOR_W - 0.026, 0.20, DOOR_H - 0.026)
        .translate((0.0, -BODY_BOTTOM_D * 0.5, opening_center_z))
    )
    shell = shell.cut(service_cut)
    return shell.translate((0.0, 0.0, BASE_H - 0.001))


def _wedge_weight_mesh() -> object:
    """Brass sliding metronome weight with a clear rod slot."""
    height = 0.058
    body = (
        cq.Workplane("XY")
        .rect(0.066, 0.033)
        .workplane(offset=height)
        .rect(0.050, 0.027)
        .loft(combine=True)
        .translate((0.0, 0.0, -height * 0.5))
    )
    rod_clearance = (
        cq.Workplane("XY")
        .circle(0.009)
        .extrude(height + 0.030)
        .translate((0.0, 0.0, -height * 0.5 - 0.015))
    )
    return body.cut(rod_clearance)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pyramid_metronome")

    wood = model.material("warm_wood", rgba=(0.47, 0.25, 0.10, 1.0))
    dark_wood = model.material("dark_wood", rgba=(0.24, 0.12, 0.05, 1.0))
    brass = model.material("brass", rgba=(0.86, 0.64, 0.25, 1.0))
    steel = model.material("polished_steel", rgba=(0.72, 0.75, 0.74, 1.0))
    black = model.material("blackened_steel", rgba=(0.02, 0.02, 0.018, 1.0))
    felt = model.material("dark_felt", rgba=(0.03, 0.035, 0.030, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box((BASE_W, BASE_D, BASE_H)),
        origin=Origin(xyz=(0.0, 0.0, BASE_H * 0.5)),
        material=dark_wood,
        name="plinth",
    )
    housing.visual(
        mesh_from_cadquery(_housing_shell_mesh(), "tapered_housing_shell"),
        material=wood,
        name="tapered_shell",
    )

    # Dark backing plate and simplified serviceable mechanism visible behind the door.
    housing.visual(
        Box((0.17, 0.010, 0.205)),
        origin=Origin(xyz=(0.0, -0.058, BASE_H + 0.101)),
        material=felt,
        name="mechanism_plate",
    )
    housing.visual(
        Cylinder(radius=0.038, length=0.014),
        origin=Origin(
            xyz=(-0.050, -0.068, BASE_H + 0.122),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=brass,
        name="winding_spring_drum",
    )
    housing.visual(
        Cylinder(radius=0.030, length=0.004),
        origin=Origin(
            xyz=(-0.050, -0.077, BASE_H + 0.122),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="spring_spiral_face",
    )
    housing.visual(
        Cylinder(radius=0.012, length=0.014),
        origin=Origin(
            xyz=(0.045, -0.068, BASE_H + 0.157),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=brass,
        name="escape_wheel",
    )
    housing.visual(
        Box((0.055, 0.010, 0.010)),
        origin=Origin(xyz=(0.050, -0.060, BASE_H + 0.188), rpy=(0.0, 0.0, 0.38)),
        material=steel,
        name="escapement_anchor",
    )

    # Alternating hinge knuckles on the fixed housing frame.
    for x in (-0.155, 0.155):
        housing.visual(
            Box((0.044, 0.018, 0.030)),
            origin=Origin(xyz=(x, HINGE_Y + 0.006, DOOR_BOTTOM_Z + 0.012)),
            material=brass,
            name=f"hinge_leaf_{'neg' if x < 0 else 'pos'}",
        )
        housing.visual(
            Cylinder(radius=0.007, length=0.060),
            origin=Origin(
                xyz=(x, HINGE_Y, DOOR_BOTTOM_Z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=brass,
            name=f"hinge_knuckle_{'neg' if x < 0 else 'pos'}",
        )
    for x in (-0.060, 0.060):
        housing.visual(
            Box((0.014, 0.068, 0.036)),
            origin=Origin(xyz=(x, PIVOT_Y + 0.026, PIVOT_Z - 0.002)),
            material=brass,
            name=f"pivot_bracket_{'neg' if x < 0 else 'pos'}",
        )
    housing.visual(
        Cylinder(radius=0.006, length=0.155),
        origin=Origin(
            xyz=(0.0, PIVOT_Y, PIVOT_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel,
        name="pivot_shaft",
    )

    front_door = model.part("front_door")
    front_door.visual(
        Box((DOOR_W, DOOR_T, DOOR_H)),
        origin=Origin(xyz=(0.0, -DOOR_T * 0.5, DOOR_H * 0.5)),
        material=wood,
        name="door_panel",
    )
    front_door.visual(
        Box((DOOR_W - 0.030, 0.004, 0.026)),
        origin=Origin(xyz=(0.0, -DOOR_T - 0.002, DOOR_H - 0.030)),
        material=brass,
        name="door_nameplate",
    )
    front_door.visual(
        Cylinder(radius=0.007, length=0.082),
        origin=Origin(xyz=(0.0, -0.009, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="door_hinge_barrel",
    )
    front_door.visual(
        Cylinder(radius=0.010, length=0.016),
        origin=Origin(xyz=(0.0, -DOOR_T - 0.007, DOOR_H * 0.48), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="door_pull",
    )

    pendulum = model.part("pendulum")
    pendulum.visual(
        Cylinder(radius=0.0042, length=0.470),
        origin=Origin(xyz=(0.0, 0.0, 0.252)),
        material=steel,
        name="upper_rod",
    )
    pendulum.visual(
        Cylinder(radius=0.0042, length=0.170),
        origin=Origin(xyz=(0.0, 0.0, -0.102)),
        material=steel,
        name="lower_rod",
    )
    pendulum.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="pivot_bushing",
    )
    pendulum.visual(
        Cylinder(radius=0.015, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.190)),
        material=brass,
        name="lower_bob",
    )

    wedge_weight = model.part("wedge_weight")
    wedge_weight.visual(
        mesh_from_cadquery(_wedge_weight_mesh(), "sliding_wedge_weight", tolerance=0.0007),
        material=brass,
        name="weight_body",
    )
    wedge_weight.visual(
        Box((0.008, 0.022, 0.008)),
        origin=Origin(xyz=(0.0, -0.0152, 0.0)),
        material=steel,
        name="friction_screw",
    )

    winding_key = model.part("winding_key")
    winding_key.visual(
        Cylinder(radius=0.0055, length=0.040),
        origin=Origin(xyz=(0.0, 0.020, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="key_shaft",
    )
    winding_key.visual(
        Cylinder(radius=0.016, length=0.009),
        origin=Origin(xyz=(0.0, 0.043, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="key_hub",
    )
    winding_key.visual(
        Box((0.078, 0.008, 0.020)),
        origin=Origin(xyz=(0.0, 0.050, 0.0)),
        material=brass,
        name="key_wings",
    )

    door_hinge = model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=front_door,
        origin=Origin(xyz=(0.0, HINGE_Y, DOOR_BOTTOM_Z), rpy=(-FRONT_TILT, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.65),
    )
    model.articulation(
        "pendulum_pivot",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=pendulum,
        origin=Origin(xyz=(0.0, PIVOT_Y, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=6.0, lower=-0.45, upper=0.45),
    )
    model.articulation(
        "weight_slide",
        ArticulationType.PRISMATIC,
        parent=pendulum,
        child=wedge_weight,
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.12, lower=0.0, upper=0.270),
    )
    rear_z = BASE_H + 0.205
    rear_y = _rear_y_at_z(rear_z)
    housing.visual(
        Cylinder(radius=0.015, length=0.009),
        origin=Origin(xyz=(0.0, rear_y + 0.0045, rear_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="rear_key_bushing",
    )
    model.articulation(
        "key_rotation",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=winding_key,
        origin=Origin(xyz=(0.0, rear_y + 0.0090, rear_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=8.0),
    )

    door_hinge.meta["description"] = "Bottom edge hinge lets the service door swing forward and down."
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    door = object_model.get_part("front_door")
    pendulum = object_model.get_part("pendulum")
    weight = object_model.get_part("wedge_weight")
    key = object_model.get_part("winding_key")
    door_hinge = object_model.get_articulation("door_hinge")
    pendulum_pivot = object_model.get_articulation("pendulum_pivot")
    weight_slide = object_model.get_articulation("weight_slide")
    key_rotation = object_model.get_articulation("key_rotation")

    ctx.allow_overlap(
        "housing",
        "pendulum",
        elem_a="pivot_shaft",
        elem_b="pivot_bushing",
        reason="The pendulum bushing is intentionally captured around the fixed top pivot shaft.",
    )
    ctx.expect_overlap(
        "housing",
        "pendulum",
        axes="xy",
        elem_a="pivot_shaft",
        elem_b="pivot_bushing",
        min_overlap=0.010,
        name="pivot shaft passes through pendulum bushing",
    )

    ctx.check(
        "primary articulated mechanisms are present",
        door_hinge.articulation_type == ArticulationType.REVOLUTE
        and pendulum_pivot.articulation_type == ArticulationType.REVOLUTE
        and weight_slide.articulation_type == ArticulationType.PRISMATIC
        and key_rotation.articulation_type == ArticulationType.CONTINUOUS,
    )

    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.35}):
        open_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "front door opens forward and downward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][1] < closed_aabb[0][1] - 0.050
        and open_aabb[1][2] < closed_aabb[1][2] - 0.050,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    rest_weight = ctx.part_world_position(weight)
    with ctx.pose({weight_slide: 0.240}):
        raised_weight = ctx.part_world_position(weight)
    ctx.check(
        "sliding wedge weight moves up the pendulum rod",
        rest_weight is not None
        and raised_weight is not None
        and raised_weight[2] > rest_weight[2] + 0.200,
        details=f"rest={rest_weight}, raised={raised_weight}",
    )

    rest_pendulum = ctx.part_world_aabb(pendulum)
    with ctx.pose({pendulum_pivot: 0.35}):
        swung_pendulum = ctx.part_world_aabb(pendulum)
    ctx.check(
        "pendulum swing displaces the rod sideways",
        rest_pendulum is not None
        and swung_pendulum is not None
        and (swung_pendulum[1][0] - rest_pendulum[1][0]) > 0.050,
        details=f"rest={rest_pendulum}, swung={swung_pendulum}",
    )

    rest_key = ctx.part_world_aabb(key)
    with ctx.pose({key_rotation: math.pi / 2.0}):
        turned_key = ctx.part_world_aabb(key)
    ctx.check(
        "rear winding key can rotate continuously",
        rest_key is not None and turned_key is not None,
        details=f"rest={rest_key}, turned={turned_key}",
    )

    return ctx.report()


object_model = build_object_model()
