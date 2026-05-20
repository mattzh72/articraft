from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    VentGrilleGeometry,
    VentGrilleSlats,
    VentGrilleSleeve,
    mesh_from_cadquery,
    mesh_from_geometry,
)

BODY_W = 0.380
BODY_D = 0.300
BODY_H = 0.690
SHELL_T = 0.004
BOTTOM_T = 0.016
TOP_T = 0.042

CONTROL_W = 0.224
CONTROL_H = 0.094
CONTROL_Z0 = 0.554
CONTROL_POCKET_D = 0.010
CONTROL_PANEL_T = 0.002

REAR_OPEN_W = 0.308
REAR_OPEN_H = 0.532
REAR_OPEN_Z0 = 0.092

DOOR_W = REAR_OPEN_W + 0.026
DOOR_H = REAR_OPEN_H + 0.026
DOOR_T = 0.014

FILTER_W = REAR_OPEN_W - 0.022
FILTER_H = REAR_OPEN_H - 0.026
FILTER_D = 0.216
FILTER_TRAVEL = 0.158


def _housing_shell() -> cq.Workplane:
    outer = cq.Workplane("XY").box(BODY_W, BODY_D, BODY_H, centered=(True, True, False))

    cavity = (
        cq.Workplane("XY")
        .box(
            BODY_W - 2.0 * SHELL_T,
            BODY_D - 2.0 * SHELL_T,
            BODY_H - BOTTOM_T - TOP_T,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, BOTTOM_T))
    )

    rear_opening = (
        cq.Workplane("XY")
        .box(REAR_OPEN_W, SHELL_T * 6.0, REAR_OPEN_H, centered=(True, True, False))
        .translate((0.0, -BODY_D / 2.0, REAR_OPEN_Z0))
    )

    control_pocket = (
        cq.Workplane("XY")
        .box(CONTROL_W, CONTROL_POCKET_D * 1.8, CONTROL_H, centered=(True, True, False))
        .translate((0.0, BODY_D / 2.0 - CONTROL_POCKET_D * 0.5, CONTROL_Z0))
    )

    return outer.cut(cavity).cut(rear_opening).cut(control_pocket)


def _door_panel() -> cq.Workplane:
    panel = cq.Workplane("XY").box(DOOR_W, DOOR_T, DOOR_H, centered=(False, False, True)).translate(
        (-DOOR_W, -0.015, 0.0)
    )
    inner_relief = (
        cq.Workplane("XY")
        .box(DOOR_W - 0.050, DOOR_T * 0.60, DOOR_H - 0.090, centered=(False, False, True))
        .translate((-(DOOR_W - 0.025), -0.010, 0.0))
    )
    return panel.cut(inner_relief)


def _button_cap(width: float, height: float, depth: float) -> cq.Workplane:
    return cq.Workplane("XY").box(width, depth, height, centered=(True, False, True))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="room_air_purifier")

    shell_white = model.material("shell_white", rgba=(0.95, 0.96, 0.97, 1.0))
    trim_grey = model.material("trim_grey", rgba=(0.76, 0.79, 0.82, 1.0))
    charcoal = model.material("charcoal", rgba=(0.15, 0.16, 0.17, 1.0))
    button_black = model.material("button_black", rgba=(0.10, 0.11, 0.12, 1.0))
    filter_frame = model.material("filter_frame", rgba=(0.17, 0.18, 0.19, 1.0))
    filter_media = model.material("filter_media", rgba=(0.83, 0.88, 0.79, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_housing_shell(), "purifier_housing"),
        material=shell_white,
        name="shell",
    )
    housing.visual(
        Box((CONTROL_W - 0.010, CONTROL_PANEL_T, CONTROL_H - 0.010)),
        origin=Origin(
            xyz=(
                0.0,
                BODY_D / 2.0 - CONTROL_POCKET_D + CONTROL_PANEL_T / 2.0 - 0.0004,
                CONTROL_Z0 + CONTROL_H / 2.0,
            )
        ),
        material=charcoal,
        name="control_panel",
    )
    housing.visual(
        mesh_from_geometry(
            SlotPatternPanelGeometry(
                (0.270, 0.352),
                0.003,
                slot_size=(0.060, 0.0045),
                pitch=(0.011, 0.073),
                frame=0.010,
                corner_radius=0.010,
                slot_angle_deg=89.0,
                center=False,
            ),
            "purifier_front_intake",
        ),
        origin=Origin(
            xyz=(0.0, BODY_D / 2.0 - 0.0010, 0.188),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=trim_grey,
        name="front_intake",
    )
    housing.visual(
        mesh_from_geometry(
            VentGrilleGeometry(
                (0.228, 0.120),
                frame=0.012,
                face_thickness=0.004,
                duct_depth=0.014,
                duct_wall=0.003,
                slat_pitch=0.016,
                slat_width=0.008,
                slat_angle_deg=30.0,
                corner_radius=0.010,
                slats=VentGrilleSlats(profile="flat", direction="down", divider_count=1, divider_width=0.004),
                sleeve=VentGrilleSleeve(style="none"),
                center=False,
            ),
            "purifier_top_vent",
        ),
        origin=Origin(xyz=(0.0, 0.012, BODY_H - 0.0012)),
        material=trim_grey,
        name="top_vent",
    )
    housing.visual(
        Cylinder(radius=0.040, length=0.003),
        origin=Origin(
            xyz=(-0.054, BODY_D / 2.0 - CONTROL_POCKET_D + 0.0030, CONTROL_Z0 + 0.055),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=trim_grey,
        name="dial_bezel",
    )
    housing.visual(
        Box((0.008, 0.182, REAR_OPEN_H - 0.050)),
        origin=Origin(
            xyz=(
                REAR_OPEN_W / 2.0 + 0.002,
                -0.055,
                REAR_OPEN_Z0 + REAR_OPEN_H / 2.0,
            )
        ),
        material=trim_grey,
        name="guide_0",
    )
    housing.visual(
        Box((0.008, 0.182, REAR_OPEN_H - 0.050)),
        origin=Origin(
            xyz=(
                -(REAR_OPEN_W / 2.0 + 0.002),
                -0.055,
                REAR_OPEN_Z0 + REAR_OPEN_H / 2.0,
            )
        ),
        material=trim_grey,
        name="guide_1",
    )

    door = model.part("rear_door")
    door.visual(
        mesh_from_cadquery(_door_panel(), "purifier_rear_door"),
        material=shell_white,
        name="door_panel",
    )
    door.visual(
        Box((0.088, 0.010, 0.034)),
        origin=Origin(xyz=(-DOOR_W + 0.070, -0.015, 0.0)),
        material=trim_grey,
        name="pull",
    )

    filter_pack = model.part("filter_pack")
    filter_pack.visual(
        Box((FILTER_W, FILTER_D, FILTER_H)),
        origin=Origin(xyz=(0.0, FILTER_D / 2.0, 0.0)),
        material=filter_frame,
        name="frame",
    )
    filter_pack.visual(
        Box((FILTER_W - 0.030, FILTER_D - 0.012, FILTER_H - 0.030)),
        origin=Origin(xyz=(0.0, 0.006 + (FILTER_D - 0.012) / 2.0, 0.0)),
        material=filter_media,
        name="media",
    )
    filter_pack.visual(
        Box((0.010, 0.150, FILTER_H - 0.040)),
        origin=Origin(xyz=(FILTER_W / 2.0 + 0.004, 0.075, 0.0)),
        material=filter_frame,
        name="shoe_0",
    )
    filter_pack.visual(
        Box((0.010, 0.150, FILTER_H - 0.040)),
        origin=Origin(xyz=(-(FILTER_W / 2.0 + 0.004), 0.075, 0.0)),
        material=filter_frame,
        name="shoe_1",
    )
    filter_pack.visual(
        Box((0.062, 0.004, 0.024)),
        origin=Origin(xyz=(0.0, -0.002, FILTER_H / 2.0 - 0.054)),
        material=button_black,
        name="tab",
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.062,
                0.028,
                body_style="skirted",
                top_diameter=0.051,
                skirt=KnobSkirt(0.072, 0.005, flare=0.06),
                grip=KnobGrip(style="fluted", count=20, depth=0.0013),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007, angle_deg=0.0),
                center=False,
            ),
            "purifier_dial",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=button_black,
        name="dial_cap",
    )

    button_mesh = mesh_from_cadquery(_button_cap(0.034, 0.017, 0.010), "purifier_button_cap")
    button_locations = (
        ("button_0", 0.034, CONTROL_Z0 + 0.067),
        ("button_1", 0.076, CONTROL_Z0 + 0.067),
        ("button_2", 0.034, CONTROL_Z0 + 0.033),
        ("button_3", 0.076, CONTROL_Z0 + 0.033),
    )
    for name, x_pos, z_pos in button_locations:
        button = model.part(name)
        button.visual(button_mesh, material=button_black, name="cap")
        model.articulation(
            f"housing_to_{name}",
            ArticulationType.PRISMATIC,
            parent=housing,
            child=button,
            origin=Origin(xyz=(x_pos, BODY_D / 2.0 - CONTROL_POCKET_D + CONTROL_PANEL_T - 0.0004, z_pos)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=0.08, lower=0.0, upper=0.0045),
        )

    model.articulation(
        "housing_to_rear_door",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(
            xyz=(
                REAR_OPEN_W / 2.0 + 0.010,
                -BODY_D / 2.0 + 0.001,
                REAR_OPEN_Z0 + REAR_OPEN_H / 2.0,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(80.0),
        ),
    )

    model.articulation(
        "housing_to_filter_pack",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=filter_pack,
        origin=Origin(
            xyz=(
                0.0,
                -BODY_D / 2.0 + SHELL_T + 0.006,
                REAR_OPEN_Z0 + REAR_OPEN_H / 2.0,
            )
        ),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=28.0,
            velocity=0.22,
            lower=0.0,
            upper=FILTER_TRAVEL,
        ),
    )

    model.articulation(
        "housing_to_dial",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=dial,
        origin=Origin(
            xyz=(-0.054, BODY_D / 2.0 - CONTROL_POCKET_D + CONTROL_PANEL_T, CONTROL_Z0 + 0.055)
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    door = object_model.get_part("rear_door")
    filter_pack = object_model.get_part("filter_pack")
    dial = object_model.get_part("dial")
    dial_joint = object_model.get_articulation("housing_to_dial")
    door_joint = object_model.get_articulation("housing_to_rear_door")
    filter_joint = object_model.get_articulation("housing_to_filter_pack")

    button_parts = [object_model.get_part(f"button_{index}") for index in range(4)]
    button_joints = [object_model.get_articulation(f"housing_to_button_{index}") for index in range(4)]

    ctx.check(
        "dial_joint_is_continuous",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={dial_joint.articulation_type!r}",
    )
    ctx.check(
        "rear_door_is_revolute",
        door_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={door_joint.articulation_type!r}",
    )
    ctx.check(
        "filter_pack_is_prismatic",
        filter_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={filter_joint.articulation_type!r}",
    )
    for index, joint in enumerate(button_joints):
        ctx.check(
            f"button_{index}_is_prismatic",
            joint.articulation_type == ArticulationType.PRISMATIC,
            details=f"type={joint.articulation_type!r}",
        )

    with ctx.pose({door_joint: 0.0, filter_joint: 0.0}):
        ctx.expect_overlap(
            door,
            housing,
            axes="xz",
            min_overlap=0.285,
            name="rear door covers opening footprint",
        )
        ctx.expect_gap(
            housing,
            door,
            axis="y",
            min_gap=0.0,
            max_gap=0.020,
            name="rear door sits just behind the cabinet shell",
        )
        ctx.expect_within(
            filter_pack,
            housing,
            axes="xz",
            margin=0.0,
            name="filter pack stays aligned with housing footprint",
        )
        ctx.expect_overlap(
            filter_pack,
            housing,
            axes="y",
            min_overlap=0.180,
            name="filter pack remains deeply inserted at rest",
        )

    door_rest_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_joint: math.radians(70.0)}):
        door_open_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "rear door swings outward",
        door_rest_aabb is not None
        and door_open_aabb is not None
        and ((door_open_aabb[0][1] + door_open_aabb[1][1]) * 0.5) < ((door_rest_aabb[0][1] + door_rest_aabb[1][1]) * 0.5) - 0.050,
        details=f"rest_aabb={door_rest_aabb}, open_aabb={door_open_aabb}",
    )

    filter_rest = ctx.part_world_position(filter_pack)
    with ctx.pose({door_joint: math.radians(70.0), filter_joint: FILTER_TRAVEL}):
        ctx.expect_within(
            filter_pack,
            housing,
            axes="xz",
            margin=0.0,
            name="extended filter pack remains registered to the rear opening",
        )
        ctx.expect_overlap(
            filter_pack,
            housing,
            axes="y",
            min_overlap=0.050,
            name="extended filter pack keeps retained insertion",
        )
        filter_extended = ctx.part_world_position(filter_pack)
    ctx.check(
        "filter pack slides out toward the rear",
        filter_rest is not None
        and filter_extended is not None
        and filter_extended[1] < filter_rest[1] - 0.120,
        details=f"rest={filter_rest}, extended={filter_extended}",
    )

    dial_rest = ctx.part_world_position(dial)
    with ctx.pose({dial_joint: math.pi / 2.0}):
        dial_turn = ctx.part_world_position(dial)
    ctx.check(
        "dial rotates in place",
        dial_rest is not None
        and dial_turn is not None
        and max(abs(dial_turn[i] - dial_rest[i]) for i in range(3)) < 1e-6,
        details=f"rest={dial_rest}, turned={dial_turn}",
    )

    button_rest_positions = [ctx.part_world_position(button) for button in button_parts]
    for index, (button, joint, rest_pos) in enumerate(zip(button_parts, button_joints, button_rest_positions)):
        upper = joint.motion_limits.upper if joint.motion_limits is not None else None
        with ctx.pose({joint: upper if upper is not None else 0.0}):
            moved_pos = ctx.part_world_position(button)
            other_static = True
            for other_index, other_button in enumerate(button_parts):
                if other_index == index:
                    continue
                other_pose = ctx.part_world_position(other_button)
                other_rest = button_rest_positions[other_index]
                other_static = (
                    other_static
                    and other_pose is not None
                    and other_rest is not None
                    and max(abs(other_pose[axis] - other_rest[axis]) for axis in range(3)) < 1e-6
                )
        ctx.check(
            f"button_{index}_depresses_inward",
            rest_pos is not None and moved_pos is not None and moved_pos[1] < rest_pos[1] - 0.0030,
            details=f"rest={rest_pos}, moved={moved_pos}",
        )
        ctx.check(
            f"button_{index}_moves_independently",
            other_static,
            details=f"button_{index} changed but neighboring buttons also shifted",
        )

    return ctx.report()


object_model = build_object_model()
