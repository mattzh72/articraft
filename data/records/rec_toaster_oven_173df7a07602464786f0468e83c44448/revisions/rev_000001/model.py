from __future__ import annotations

from math import pi

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
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BODY_D = 0.43
BODY_W = 0.56
BODY_H = 0.36
WALL = 0.012
FRONT_T = 0.010
OPEN_H = 0.215
OPEN_CY = -0.07
OPEN_BOTTOM = 0.080
DOOR_W = 0.384
DOOR_H = 0.235
DOOR_T = 0.028
DOOR_FRAME = 0.028
CONTROL_W = 0.148
CONTROL_CY = 0.200
DIVIDER_CY = 0.120
FRONT_FACE_X = BODY_D / 2.0 - FRONT_T / 2.0
RACK_TRAVEL = 0.160
TRAY_TRAVEL = 0.145


def add_box(part, size, xyz, *, material, name):
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def add_cylinder(part, radius, length, xyz, rpy, *, material, name):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="digital_toaster_oven")

    steel = model.material("steel", rgba=(0.71, 0.73, 0.75, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.27, 0.28, 0.30, 1.0))
    glass = model.material("glass", rgba=(0.18, 0.22, 0.26, 0.32))
    plastic = model.material("plastic", rgba=(0.10, 0.10, 0.11, 1.0))
    display = model.material("display", rgba=(0.05, 0.08, 0.10, 1.0))
    trim = model.material("trim", rgba=(0.82, 0.82, 0.80, 1.0))

    body = model.part("body")
    add_box(
        body,
        (BODY_D, BODY_W, WALL),
        (0.0, 0.0, WALL / 2.0),
        material=steel,
        name="bottom_shell",
    )
    add_box(
        body,
        (BODY_D, WALL, BODY_H),
        (0.0, -BODY_W / 2.0 + WALL / 2.0, BODY_H / 2.0),
        material=steel,
        name="left_wall",
    )
    add_box(
        body,
        (BODY_D, WALL, BODY_H),
        (0.0, BODY_W / 2.0 - WALL / 2.0, BODY_H / 2.0),
        material=steel,
        name="right_wall",
    )
    add_box(
        body,
        (WALL, BODY_W - 2.0 * WALL, BODY_H),
        (-BODY_D / 2.0 + WALL / 2.0, 0.0, BODY_H / 2.0),
        material=steel,
        name="back_wall",
    )
    add_box(
        body,
        (BODY_D, BODY_W, WALL),
        (0.0, 0.0, BODY_H - WALL / 2.0),
        material=steel,
        name="top_shell",
    )
    add_box(
        body,
        (BODY_D, WALL, BODY_H - WALL),
        (0.0, DIVIDER_CY, (BODY_H - WALL) / 2.0),
        material=dark_steel,
        name="divider",
    )
    add_box(
        body,
        (FRONT_T, 0.040, 0.265),
        (FRONT_FACE_X, OPEN_CY - DOOR_W / 2.0 + 0.020, 0.142),
        material=steel,
        name="left_jamb",
    )
    add_box(
        body,
        (FRONT_T, 0.040, 0.265),
        (FRONT_FACE_X, OPEN_CY + DOOR_W / 2.0 - 0.020, 0.142),
        material=steel,
        name="right_jamb",
    )
    add_box(
        body,
        (FRONT_T, DOOR_W + 0.020, 0.048),
        (FRONT_FACE_X, OPEN_CY, 0.024),
        material=steel,
        name="lower_sill",
    )
    add_box(
        body,
        (FRONT_T, DOOR_W + 0.020, BODY_H - (OPEN_BOTTOM + OPEN_H)),
        (FRONT_FACE_X, OPEN_CY, OPEN_BOTTOM + OPEN_H + (BODY_H - (OPEN_BOTTOM + OPEN_H)) / 2.0),
        material=steel,
        name="upper_bezel",
    )
    add_box(
        body,
        (FRONT_T, CONTROL_W, BODY_H),
        (FRONT_FACE_X, CONTROL_CY, BODY_H / 2.0),
        material=dark_steel,
        name="control_face",
    )
    add_box(
        body,
        (0.004, 0.094, 0.030),
        (FRONT_FACE_X + 0.007, CONTROL_CY, 0.295),
        material=display,
        name="display_window",
    )
    add_box(
        body,
        (BODY_D - 0.050, DOOR_W + 0.024, 0.008),
        (0.006, OPEN_CY, 0.078),
        material=dark_steel,
        name="cavity_floor",
    )
    for idx, hinge_y in enumerate((OPEN_CY - 0.145, OPEN_CY + 0.145)):
        add_box(
            body,
            (0.010, 0.016, 0.052),
            (BODY_D / 2.0 - 0.009, hinge_y, 0.074),
            material=dark_steel,
            name=f"hinge_bracket_{idx}",
        )
    for idx, guide_y in enumerate((OPEN_CY - 0.152, OPEN_CY + 0.152)):
        add_box(
            body,
            (0.318, 0.014, 0.010),
            (-0.044, guide_y, 0.139),
            material=trim,
            name=f"rack_guide_{idx}",
        )
    for idx, guide_y in enumerate((OPEN_CY - 0.160, OPEN_CY + 0.160)):
        add_box(
            body,
            (0.326, 0.012, 0.010),
            (-0.041, guide_y, 0.047),
            material=trim,
            name=f"tray_guide_{idx}",
        )

    door = model.part("door")
    door_x = DOOR_T / 2.0 + 0.004
    add_box(
        door,
        (DOOR_T, DOOR_W, DOOR_FRAME),
        (door_x, 0.0, DOOR_FRAME / 2.0),
        material=steel,
        name="bottom_frame",
    )
    add_box(
        door,
        (DOOR_T, DOOR_W, DOOR_FRAME),
        (door_x, 0.0, DOOR_H - DOOR_FRAME / 2.0),
        material=steel,
        name="top_frame",
    )
    add_box(
        door,
        (DOOR_T, DOOR_FRAME, DOOR_H),
        (door_x, -DOOR_W / 2.0 + DOOR_FRAME / 2.0, DOOR_H / 2.0),
        material=steel,
        name="side_frame_0",
    )
    add_box(
        door,
        (DOOR_T, DOOR_FRAME, DOOR_H),
        (door_x, DOOR_W / 2.0 - DOOR_FRAME / 2.0, DOOR_H / 2.0),
        material=steel,
        name="side_frame_1",
    )
    add_box(
        door,
        (0.008, DOOR_W - 2.0 * DOOR_FRAME + 0.004, DOOR_H - 2.0 * DOOR_FRAME + 0.004),
        (door_x - 0.004, 0.0, DOOR_H / 2.0),
        material=glass,
        name="glass",
    )
    add_cylinder(
        door,
        radius=0.011,
        length=DOOR_W - 0.104,
        xyz=(DOOR_T + 0.030, 0.0, DOOR_H * 0.77),
        rpy=(pi / 2.0, 0.0, 0.0),
        material=trim,
        name="handle",
    )
    for idx, sign in enumerate((-1.0, 1.0)):
        add_box(
            door,
            (0.030, 0.016, 0.022),
            (DOOR_T + 0.014, sign * (DOOR_W * 0.31), DOOR_H * 0.77),
            material=steel,
            name=f"standoff_{idx}",
        )
        add_box(
            door,
            (0.030, 0.014, 0.018),
            (DOOR_T * 0.75, sign * (DOOR_W * 0.31), DOOR_H * 0.77),
            material=steel,
            name=f"mount_{idx}",
        )
        add_box(
            door,
            (0.008, 0.016, 0.020),
            (0.000, sign * 0.145, 0.010),
            material=dark_steel,
            name=f"hinge_ear_{idx}",
        )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(BODY_D / 2.0, OPEN_CY, OPEN_BOTTOM)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.2, lower=0.0, upper=1.35),
    )

    knob_geometry = KnobGeometry(
        0.064,
        0.035,
        body_style="skirted",
        top_diameter=0.052,
        skirt=KnobSkirt(0.076, 0.006, flare=0.05),
        grip=KnobGrip(style="fluted", count=16, depth=0.0014),
        indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
        center=False,
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_geometry(knob_geometry, "control_dial"),
        origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=plastic,
        name="knob",
    )
    add_cylinder(
        dial,
        radius=0.014,
        length=0.006,
        xyz=(0.003, 0.0, 0.0),
        rpy=(0.0, pi / 2.0, 0.0),
        material=trim,
        name="hub",
    )

    model.articulation(
        "dial_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(BODY_D / 2.0, CONTROL_CY, 0.188)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )

    rack = model.part("rack")
    for idx, runner_y in enumerate((-0.152, 0.152)):
        add_box(
            rack,
            (0.340, 0.010, 0.008),
            (-0.170, runner_y, 0.0),
            material=trim,
            name=f"runner_{idx}",
        )
    add_box(
        rack,
        (0.012, 0.314, 0.008),
        (-0.006, 0.0, 0.0),
        material=trim,
        name="front_bar",
    )
    add_box(
        rack,
        (0.012, 0.314, 0.008),
        (-0.334, 0.0, 0.0),
        material=trim,
        name="rear_bar",
    )
    for idx, slat_y in enumerate((-0.100, -0.050, 0.0, 0.050, 0.100)):
        add_box(
            rack,
            (0.320, 0.006, 0.006),
            (-0.170, slat_y, 0.001),
            material=trim,
            name=f"slat_{idx}",
        )

    model.articulation(
        "rack_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=rack,
        origin=Origin(xyz=(0.185, OPEN_CY, 0.148)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.18,
            lower=0.0,
            upper=RACK_TRAVEL,
        ),
    )

    tray = model.part("crumb_tray")
    add_box(
        tray,
        (0.330, 0.320, 0.006),
        (-0.155, 0.0, 0.0),
        material=dark_steel,
        name="tray_floor",
    )
    add_box(
        tray,
        (0.330, 0.008, 0.018),
        (-0.155, -0.156, 0.009),
        material=dark_steel,
        name="tray_side_0",
    )
    add_box(
        tray,
        (0.330, 0.008, 0.018),
        (-0.155, 0.156, 0.009),
        material=dark_steel,
        name="tray_side_1",
    )
    add_box(
        tray,
        (0.010, 0.320, 0.018),
        (-0.315, 0.0, 0.009),
        material=dark_steel,
        name="tray_back",
    )
    add_box(
        tray,
        (0.020, 0.334, 0.024),
        (0.0, 0.0, 0.012),
        material=steel,
        name="tray_front",
    )
    add_cylinder(
        tray,
        radius=0.010,
        length=0.130,
        xyz=(0.020, 0.0, 0.014),
        rpy=(pi / 2.0, 0.0, 0.0),
        material=trim,
        name="tray_pull",
    )

    model.articulation(
        "tray_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=(0.194, OPEN_CY, 0.054)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.16,
            lower=0.0,
            upper=TRAY_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    dial = object_model.get_part("dial")
    rack = object_model.get_part("rack")
    tray = object_model.get_part("crumb_tray")
    door_hinge = object_model.get_articulation("door_hinge")
    dial_spin = object_model.get_articulation("dial_spin")
    rack_slide = object_model.get_articulation("rack_slide")
    tray_slide = object_model.get_articulation("tray_slide")

    door_limits = door_hinge.motion_limits
    dial_limits = dial_spin.motion_limits
    rack_limits = rack_slide.motion_limits
    tray_limits = tray_slide.motion_limits
    ctx.check(
        "door hinge uses downward-opening limits",
        door_limits is not None
        and door_limits.lower == 0.0
        and door_limits.upper is not None
        and door_limits.upper >= 1.2
        and tuple(door_hinge.axis) == (0.0, 1.0, 0.0),
        details=f"axis={door_hinge.axis}, limits={door_limits}",
    )
    ctx.check(
        "dial articulation is continuous",
        dial_limits is not None and dial_limits.lower is None and dial_limits.upper is None,
        details=f"limits={dial_limits}",
    )
    ctx.check(
        "rack slides straight forward",
        rack_limits is not None
        and rack_limits.lower == 0.0
        and rack_limits.upper is not None
        and rack_limits.upper >= 0.15
        and tuple(rack_slide.axis) == (1.0, 0.0, 0.0),
        details=f"axis={rack_slide.axis}, limits={rack_limits}",
    )
    ctx.check(
        "crumb tray slides straight forward",
        tray_limits is not None
        and tray_limits.lower == 0.0
        and tray_limits.upper is not None
        and tray_limits.upper >= 0.12
        and tuple(tray_slide.axis) == (1.0, 0.0, 0.0),
        details=f"axis={tray_slide.axis}, limits={tray_limits}",
    )

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(
            door,
            body,
            axis="x",
            max_penetration=0.0005,
            max_gap=0.040,
            positive_elem="glass",
            name="door sits just in front of the oven face",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="yz",
            min_overlap=0.20,
            name="door covers the front oven opening",
        )
        ctx.expect_contact(
            dial,
            body,
            elem_a="hub",
            elem_b="control_face",
            contact_tol=0.001,
            name="dial hub seats against the control panel",
        )
        ctx.expect_within(
            rack,
            body,
            axes="yz",
            margin=0.0,
            name="rack stays inside the oven cavity envelope",
        )
        ctx.expect_within(
            tray,
            body,
            axes="yz",
            margin=0.0,
            name="crumb tray stays within the lower body envelope",
        )

    closed_door_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.20}):
        open_door_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door swings outward and downward",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][0] > closed_door_aabb[1][0] + 0.11
        and open_door_aabb[1][2] < closed_door_aabb[1][2] - 0.05,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    rack_rest_pos = ctx.part_world_position(rack)
    with ctx.pose({rack_slide: RACK_TRAVEL}):
        rack_extended_pos = ctx.part_world_position(rack)
        ctx.expect_overlap(
            rack,
            body,
            axes="x",
            min_overlap=0.18,
            name="rack remains retained in the oven at full extension",
        )
    ctx.check(
        "rack extends toward the user",
        rack_rest_pos is not None
        and rack_extended_pos is not None
        and rack_extended_pos[0] > rack_rest_pos[0] + 0.12,
        details=f"rest={rack_rest_pos}, extended={rack_extended_pos}",
    )

    tray_rest_pos = ctx.part_world_position(tray)
    with ctx.pose({tray_slide: TRAY_TRAVEL}):
        tray_extended_pos = ctx.part_world_position(tray)
        ctx.expect_overlap(
            tray,
            body,
            axes="x",
            min_overlap=0.18,
            name="crumb tray remains retained on its guides",
        )
    ctx.check(
        "crumb tray extends forward",
        tray_rest_pos is not None
        and tray_extended_pos is not None
        and tray_extended_pos[0] > tray_rest_pos[0] + 0.10,
        details=f"rest={tray_rest_pos}, extended={tray_extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
