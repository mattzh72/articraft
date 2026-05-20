from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    mesh_from_cadquery,
)


BODY_W = 0.500
BODY_D = 0.380
BODY_H = 0.150

SCANNER_OPEN_W = 0.404
SCANNER_OPEN_D = 0.272
SCANNER_OPEN_Y = -0.012
SCANNER_BAY_DEPTH = 0.032

MAIN_TRAY_W = 0.408
MAIN_TRAY_D = 0.285
MAIN_TRAY_H = 0.050

PHOTO_TRAY_W = 0.212
PHOTO_TRAY_D = 0.232
PHOTO_TRAY_H = 0.022

LID_W = 0.470
LID_D = 0.326
LID_H = 0.026

WALL = 0.0035
PANEL_TILT = math.radians(65.0)


def _body_shape() -> cq.Workplane:
    def plate(size_x: float, size_y: float, size_z: float, xyz: tuple[float, float, float]) -> cq.Workplane:
        return (
            cq.Workplane("XY")
            .box(size_x, size_y, size_z, centered=(True, True, False))
            .translate(xyz)
        )

    body = plate(BODY_W, BODY_D, 0.010, (0.0, 0.0, 0.0))
    for member in (
        plate(0.028, BODY_D, 0.130, (-(BODY_W * 0.5) + 0.014, 0.0, 0.010)),
        plate(0.028, BODY_D, 0.130, ((BODY_W * 0.5) - 0.014, 0.0, 0.010)),
        plate(BODY_W, 0.034, 0.130, (0.0, -(BODY_D * 0.5) + 0.017, 0.010)),
        plate(BODY_W, 0.058, 0.010, (0.0, 0.161, BODY_H - 0.010)),
        plate(BODY_W, 0.036, 0.010, (0.0, -(BODY_D * 0.5) + 0.018, BODY_H - 0.010)),
        plate(0.044, 0.286, 0.010, (-(SCANNER_OPEN_W * 0.5) - 0.022, SCANNER_OPEN_Y, BODY_H - 0.010)),
        plate(0.044, 0.286, 0.010, ((SCANNER_OPEN_W * 0.5) + 0.022, SCANNER_OPEN_Y, BODY_H - 0.010)),
        plate(0.006, 0.282, 0.022, (-(SCANNER_OPEN_W * 0.5) - 0.003, SCANNER_OPEN_Y, BODY_H - SCANNER_BAY_DEPTH)),
        plate(0.006, 0.282, 0.022, ((SCANNER_OPEN_W * 0.5) + 0.003, SCANNER_OPEN_Y, BODY_H - SCANNER_BAY_DEPTH)),
        plate(SCANNER_OPEN_W + 0.012, 0.006, 0.022, (0.0, SCANNER_OPEN_Y + (SCANNER_OPEN_D * 0.5) - 0.003, BODY_H - SCANNER_BAY_DEPTH)),
        plate(SCANNER_OPEN_W + 0.012, 0.006, 0.022, (0.0, SCANNER_OPEN_Y - (SCANNER_OPEN_D * 0.5) + 0.003, BODY_H - SCANNER_BAY_DEPTH)),
        plate(0.462, 0.018, 0.016, (0.0, (BODY_D * 0.5) - 0.009, 0.010)),
        plate(0.034, 0.018, 0.114, (-(BODY_W * 0.5) + 0.017, (BODY_D * 0.5) - 0.009, 0.026)),
        plate(0.034, 0.018, 0.114, ((BODY_W * 0.5) - 0.017, (BODY_D * 0.5) - 0.009, 0.026)),
        plate(0.462, 0.018, 0.006, (0.0, (BODY_D * 0.5) - 0.009, 0.078)),
        plate(0.119, 0.018, 0.024, (-0.1715, (BODY_D * 0.5) - 0.009, 0.084)),
        plate(0.119, 0.018, 0.024, (0.1715, (BODY_D * 0.5) - 0.009, 0.084)),
        plate(0.100, 0.018, 0.018, (-0.181, (BODY_D * 0.5) - 0.009, 0.108)),
        plate(0.100, 0.018, 0.018, (0.181, (BODY_D * 0.5) - 0.009, 0.108)),
        plate(0.462, 0.018, 0.012, (0.0, (BODY_D * 0.5) - 0.009, 0.126)),
    ):
        body = body.union(member)

    return body


def _lid_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(LID_W, LID_D, LID_H, centered=(True, False, False))
        .edges("|Z")
        .fillet(0.014)
        .edges(">Z")
        .fillet(0.004)
    )
    inner = (
        cq.Workplane("XY")
        .box(
            LID_W - 2.0 * WALL,
            LID_D - 2.0 * WALL,
            LID_H - WALL,
            centered=(True, False, False),
        )
        .translate((0.0, WALL, 0.0))
    )
    front_grip = (
        cq.Workplane("XY")
        .box(0.160, 0.010, 0.010, centered=(True, False, False))
        .translate((0.0, LID_D - 0.010, 0.0))
        .edges("|Z")
        .fillet(0.003)
    )
    return outer.cut(inner).union(front_grip)


def _tray_shape(*, width: float, depth: float, height: float, name: str) -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(width, depth, height, centered=(True, False, False))
        .edges("|Z")
        .fillet(min(0.006, height * 0.16))
    )
    inner = (
        cq.Workplane("XY")
        .box(
            width - 2.0 * WALL,
            depth - 2.0 * WALL,
            height - WALL,
            centered=(True, False, False),
        )
        .translate((0.0, WALL, WALL))
    )
    left_runner = (
        cq.Workplane("XY")
        .box(0.008, depth * 0.82, 0.008, centered=(True, False, False))
        .translate((-(width * 0.5 - 0.004), depth * 0.06, 0.010))
    )
    right_runner = (
        cq.Workplane("XY")
        .box(0.008, depth * 0.82, 0.008, centered=(True, False, False))
        .translate(((width * 0.5 - 0.004), depth * 0.06, 0.010))
    )
    front_lip = (
        cq.Workplane("XY")
        .box(width, 0.012, height + 0.008, centered=(True, False, False))
        .translate((0.0, depth - 0.012, 0.0))
        .edges("|Z")
        .fillet(min(0.005, height * 0.18))
    )
    shape = outer.cut(inner).union(left_runner).union(right_runner).union(front_lip)

    return shape


def _aabb_z_bounds(aabb) -> tuple[float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return float(mins[2]), float(maxs[2])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="photo_printer")

    shell_white = model.material("shell_white", rgba=(0.955, 0.955, 0.955, 1.0))
    trim_grey = model.material("trim_grey", rgba=(0.780, 0.790, 0.810, 1.0))
    scanner_glass = model.material("scanner_glass", rgba=(0.420, 0.530, 0.620, 0.35))
    tray_grey = model.material("tray_grey", rgba=(0.880, 0.890, 0.900, 1.0))
    hinge_dark = model.material("hinge_dark", rgba=(0.310, 0.330, 0.350, 1.0))
    control_dark = model.material("control_dark", rgba=(0.170, 0.180, 0.190, 1.0))
    control_cap = model.material("control_cap", rgba=(0.245, 0.255, 0.270, 1.0))

    body = model.part("body")
    body.visual(mesh_from_cadquery(_body_shape(), "printer_body"), material=shell_white, name="body_shell")
    body.visual(
        Box((SCANNER_OPEN_W + 0.014, SCANNER_OPEN_D + 0.014, 0.004)),
        origin=Origin(xyz=(0.0, SCANNER_OPEN_Y, BODY_H - SCANNER_BAY_DEPTH)),
        material=trim_grey,
        name="scanner_bed",
    )
    body.visual(
        Box((SCANNER_OPEN_W - 0.022, SCANNER_OPEN_D - 0.022, 0.002)),
        origin=Origin(xyz=(0.0, SCANNER_OPEN_Y, BODY_H - SCANNER_BAY_DEPTH + 0.0035)),
        material=scanner_glass,
        name="scanner_glass",
    )
    body.visual(
        Box((0.010, 0.260, 0.004)),
        origin=Origin(xyz=(-0.223, 0.048, 0.031)),
        material=trim_grey,
        name="main_rail_0",
    )
    body.visual(
        Box((0.010, 0.260, 0.004)),
        origin=Origin(xyz=(0.223, 0.048, 0.031)),
        material=trim_grey,
        name="main_rail_1",
    )
    body.visual(
        Box((0.010, 0.198, 0.004)),
        origin=Origin(xyz=(-0.117, 0.079, 0.091)),
        material=trim_grey,
        name="photo_rail_0",
    )
    body.visual(
        Box((0.010, 0.198, 0.004)),
        origin=Origin(xyz=(0.117, 0.079, 0.091)),
        material=trim_grey,
        name="photo_rail_1",
    )
    body.visual(
        Cylinder(radius=0.004, length=0.252),
        origin=Origin(xyz=(0.0, 0.187, 0.112), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_dark,
        name="door_mount",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_D, BODY_H)),
        mass=7.4,
        origin=Origin(xyz=(0.0, 0.0, BODY_H * 0.5)),
    )

    control_panel = model.part("control_panel")
    control_panel.visual(
        Box((0.164, 0.054, 0.010)),
        material=trim_grey,
        name="control_panel",
    )
    control_panel.visual(
        Box((0.168, 0.058, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.007)),
        material=shell_white,
        name="control_surround",
    )
    control_panel.visual(
        Box((0.150, 0.040, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, -0.013)),
        material=trim_grey,
        name="control_mount",
    )
    control_panel.inertial = Inertial.from_geometry(
        Box((0.168, 0.058, 0.026)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
    )
    model.articulation(
        "body_to_control_panel",
        ArticulationType.FIXED,
        parent=body,
        child=control_panel,
        origin=Origin(xyz=(0.145, 0.190, 0.135), rpy=(-PANEL_TILT, 0.0, 0.0)),
    )

    lid = model.part("lid")
    lid.visual(mesh_from_cadquery(_lid_shape(), "scanner_lid"), material=shell_white, name="lid_shell")
    lid.inertial = Inertial.from_geometry(
        Box((LID_W, LID_D, LID_H)),
        mass=1.2,
        origin=Origin(xyz=(0.0, LID_D * 0.5, LID_H * 0.5)),
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, -(BODY_D * 0.5) + 0.010, BODY_H)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(68.0),
        ),
    )

    main_tray = model.part("main_tray")
    main_tray.visual(
        mesh_from_cadquery(_tray_shape(width=MAIN_TRAY_W, depth=MAIN_TRAY_D, height=MAIN_TRAY_H, name="main"), "main_tray"),
        material=tray_grey,
        name="tray_shell",
    )
    main_tray.inertial = Inertial.from_geometry(
        Box((MAIN_TRAY_W, MAIN_TRAY_D, MAIN_TRAY_H)),
        mass=0.8,
        origin=Origin(xyz=(0.0, MAIN_TRAY_D * 0.5, MAIN_TRAY_H * 0.5)),
    )
    model.articulation(
        "body_to_main_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=main_tray,
        origin=Origin(xyz=(0.0, -0.091, 0.024)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.18,
            lower=0.0,
            upper=0.132,
        ),
    )

    photo_tray = model.part("photo_tray")
    photo_tray.visual(
        mesh_from_cadquery(_tray_shape(width=PHOTO_TRAY_W, depth=PHOTO_TRAY_D, height=PHOTO_TRAY_H, name="photo"), "photo_tray"),
        material=tray_grey,
        name="tray_shell",
    )
    photo_tray.inertial = Inertial.from_geometry(
        Box((PHOTO_TRAY_W, PHOTO_TRAY_D, PHOTO_TRAY_H)),
        mass=0.32,
        origin=Origin(xyz=(0.0, PHOTO_TRAY_D * 0.5, PHOTO_TRAY_H * 0.5)),
    )
    model.articulation(
        "body_to_photo_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=photo_tray,
        origin=Origin(xyz=(0.0, -0.045, 0.084)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.14,
            lower=0.0,
            upper=0.090,
        ),
    )

    output_door = model.part("output_door")
    output_door.visual(
        Box((0.252, 0.020, 0.006)),
        origin=Origin(xyz=(0.0, -0.010, 0.003)),
        material=tray_grey,
        name="door_panel",
    )
    output_door.visual(
        Cylinder(radius=0.004, length=0.252),
        origin=Origin(xyz=(0.0, -0.016, 0.003), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_dark,
        name="door_hinge",
    )
    output_door.inertial = Inertial.from_geometry(
        Box((0.252, 0.020, 0.008)),
        mass=0.16,
        origin=Origin(xyz=(0.0, -0.010, 0.003)),
    )
    model.articulation(
        "body_to_output_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=output_door,
        origin=Origin(xyz=(0.0, (BODY_D * 0.5) + 0.019, 0.109)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.2,
            lower=0.0,
            upper=math.radians(88.0),
        ),
    )

    knob = model.part("knob")
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.026,
                0.015,
                body_style="skirted",
                top_diameter=0.020,
                skirt=KnobSkirt(0.032, 0.004, flare=0.06),
                grip=KnobGrip(style="fluted", count=16, depth=0.0008),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0006, angle_deg=0.0),
                center=False,
            ),
            "printer_control_knob",
        ),
        material=control_dark,
        name="knob_shell",
    )
    knob.inertial = Inertial.from_geometry(
        Box((0.032, 0.032, 0.016)),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
    )
    model.articulation(
        "body_to_knob",
        ArticulationType.CONTINUOUS,
        parent=control_panel,
        child=knob,
        origin=Origin(xyz=(-0.060, 0.000, 0.006)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.35,
            velocity=8.0,
        ),
    )

    for index, x_pos in enumerate((0.108, 0.138, 0.168)):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.008, 0.008, 0.004)),
            origin=Origin(xyz=(0.0, 0.0, 0.002)),
            material=control_dark,
            name="button_stem",
        )
        button.visual(
            Box((0.015, 0.015, 0.004)),
            origin=Origin(xyz=(0.0, 0.0, 0.006)),
            material=control_cap,
            name="button_cap",
        )
        button.inertial = Inertial.from_geometry(
            Box((0.015, 0.015, 0.010)),
            mass=0.018,
            origin=Origin(xyz=(0.0, 0.0, 0.005)),
        )
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=control_panel,
            child=button,
            origin=Origin(xyz=(x_pos - 0.112, 0.000, 0.005)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=0.08,
                lower=0.0,
                upper=0.0022,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    control_panel = object_model.get_part("control_panel")
    lid = object_model.get_part("lid")
    main_tray = object_model.get_part("main_tray")
    photo_tray = object_model.get_part("photo_tray")
    output_door = object_model.get_part("output_door")
    knob = object_model.get_part("knob")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")
    button_2 = object_model.get_part("button_2")

    lid_joint = object_model.get_articulation("body_to_lid")
    main_tray_joint = object_model.get_articulation("body_to_main_tray")
    photo_tray_joint = object_model.get_articulation("body_to_photo_tray")
    door_joint = object_model.get_articulation("body_to_output_door")
    knob_joint = object_model.get_articulation("body_to_knob")
    button_joints = [
        object_model.get_articulation("body_to_button_0"),
        object_model.get_articulation("body_to_button_1"),
        object_model.get_articulation("body_to_button_2"),
    ]

    ctx.allow_overlap(
        body,
        main_tray,
        elem_a="body_shell",
        elem_b="tray_shell",
        reason="The main tray cavity is represented by a simplified outer housing proxy while the visible tray remains nested in the printer body.",
    )
    ctx.allow_overlap(
        body,
        photo_tray,
        elem_a="body_shell",
        elem_b="tray_shell",
        reason="The upper photo tray is intentionally shown nested into the simplified front housing proxy.",
    )
    ctx.allow_overlap(
        body,
        output_door,
        elem_a="door_mount",
        elem_b="door_hinge",
        reason="The output flap uses a simplified shared hinge barrel at the front lip.",
    )
    ctx.allow_overlap(
        body,
        control_panel,
        elem_a="body_shell",
        elem_b="control_mount",
        reason="The concealed control pod mount is simplified as a support block embedded into the front housing shell.",
    )
    ctx.allow_overlap(
        body,
        control_panel,
        elem_a="body_shell",
        elem_b="control_surround",
        reason="The front fascia is simplified and the control surround is intentionally nested slightly into the housing shell.",
    )
    ctx.allow_overlap(
        body,
        control_panel,
        elem_a="body_shell",
        elem_b="control_panel",
        reason="The sloped control pod face is intentionally embedded into the simplified front body shell.",
    )
    ctx.allow_overlap(
        body,
        control_panel,
        elem_a="door_mount",
        elem_b="control_surround",
        reason="The front hinge boss is simplified as packaging directly beneath the control surround.",
    )
    ctx.allow_overlap(
        body,
        control_panel,
        elem_a="door_mount",
        elem_b="control_mount",
        reason="The hinge boss and concealed control-pod mount share the same simplified front-fascia packaging volume.",
    )
    ctx.allow_overlap(
        control_panel,
        photo_tray,
        elem_a="control_mount",
        elem_b="tray_shell",
        reason="The hidden control-pod support is simplified inside the fascia above the nested photo tray path.",
    )
    ctx.allow_overlap(
        control_panel,
        output_door,
        elem_a="control_surround",
        elem_b="door_hinge",
        reason="The compact front fascia uses a simplified hinge package under the control surround.",
    )
    ctx.allow_overlap(
        control_panel,
        output_door,
        elem_a="control_panel",
        elem_b="door_hinge",
        reason="The narrow output-door hinge is simplified as tucking directly beneath the control pod face.",
    )
    ctx.allow_overlap(
        control_panel,
        output_door,
        elem_a="control_panel",
        elem_b="door_panel",
        reason="The compact front lip is simplified so the closed output flap nests partly under the control pod face.",
    )
    ctx.allow_overlap(
        control_panel,
        output_door,
        elem_a="control_surround",
        elem_b="door_panel",
        reason="The closed output flap tucks under the simplified control surround at the front lip.",
    )
    for button_name in ("button_0", "button_1", "button_2"):
        ctx.allow_overlap(
            button_name,
            control_panel,
            elem_a="button_stem",
            elem_b="control_panel",
            reason="Each button stem is intentionally shown passing through the control panel face.",
        )

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_shell",
        negative_elem="body_shell",
        max_gap=0.004,
        max_penetration=0.0005,
        name="lid seats on the printer top deck",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_shell",
        elem_b="body_shell",
        min_overlap=0.220,
        name="lid broadly covers the scanner bay",
    )

    lid_closed_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    main_tray_rest = ctx.part_world_position(main_tray)
    photo_tray_rest = ctx.part_world_position(photo_tray)
    door_closed_aabb = ctx.part_element_world_aabb(output_door, elem="door_panel")

    lid_limits = lid_joint.motion_limits
    if lid_limits is not None and lid_limits.upper is not None:
        with ctx.pose({lid_joint: lid_limits.upper}):
            lid_open_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
        closed_z = _aabb_z_bounds(lid_closed_aabb)
        open_z = _aabb_z_bounds(lid_open_aabb)
        ctx.check(
            "lid opens upward from the rear hinge",
            closed_z is not None and open_z is not None and open_z[1] > closed_z[1] + 0.110,
            details=f"closed={closed_z}, open={open_z}",
        )

    main_limits = main_tray_joint.motion_limits
    if main_limits is not None and main_limits.upper is not None:
        with ctx.pose({main_tray_joint: main_limits.upper}):
            main_tray_open = ctx.part_world_position(main_tray)
            main_tray_open_aabb = ctx.part_element_world_aabb(main_tray, elem="tray_shell")
        ctx.check(
            "main tray slides out the front",
            main_tray_rest is not None
            and main_tray_open is not None
            and main_tray_open[1] > main_tray_rest[1] + 0.100,
            details=f"rest={main_tray_rest}, open={main_tray_open}",
        )
        ctx.check(
            "main tray stays retained on its rails",
            main_tray_open_aabb is not None and float(main_tray_open_aabb[0][1]) < BODY_D * 0.5 - 0.020,
            details=f"open_aabb={main_tray_open_aabb}",
        )

    photo_limits = photo_tray_joint.motion_limits
    if photo_limits is not None and photo_limits.upper is not None:
        with ctx.pose({photo_tray_joint: photo_limits.upper}):
            photo_tray_open = ctx.part_world_position(photo_tray)
            photo_tray_open_aabb = ctx.part_element_world_aabb(photo_tray, elem="tray_shell")
        ctx.check(
            "photo tray slides out independently above the main tray",
            photo_tray_rest is not None
            and photo_tray_open is not None
            and photo_tray_open[1] > photo_tray_rest[1] + 0.060
            and photo_tray_open[2] > 0.070,
            details=f"rest={photo_tray_rest}, open={photo_tray_open}",
        )
        ctx.check(
            "photo tray stays retained on its upper rails",
            photo_tray_open_aabb is not None and float(photo_tray_open_aabb[0][1]) < BODY_D * 0.5 - 0.030,
            details=f"open_aabb={photo_tray_open_aabb}",
        )

    door_limits = door_joint.motion_limits
    if door_limits is not None and door_limits.upper is not None:
        with ctx.pose({door_joint: door_limits.upper}):
            door_open_aabb = ctx.part_element_world_aabb(output_door, elem="door_panel")
        ctx.check(
            "output door rotates downward",
            door_closed_aabb is not None
            and door_open_aabb is not None
            and float(door_open_aabb[0][1]) > float(door_closed_aabb[0][1]) + 0.010
            and float(door_open_aabb[0][2]) < float(door_closed_aabb[0][2]) - 0.010,
            details=f"closed={door_closed_aabb}, open={door_open_aabb}",
        )

    knob_limits = knob_joint.motion_limits
    ctx.check(
        "control knob uses continuous rotation",
        knob_limits is not None and knob_limits.lower is None and knob_limits.upper is None,
        details=f"limits={knob_limits}",
    )
    ctx.expect_overlap(
        knob,
        control_panel,
        axes="x",
        elem_a="knob_shell",
        elem_b="control_panel",
        min_overlap=0.018,
        name="control knob sits over the front panel area",
    )

    for button_part, button_joint in zip((button_0, button_1, button_2), button_joints):
        button_rest = ctx.part_world_position(button_part)
        button_limits = button_joint.motion_limits
        button_open = None
        if button_limits is not None and button_limits.upper is not None:
            with ctx.pose({button_joint: button_limits.upper}):
                button_open = ctx.part_world_position(button_part)
        ctx.check(
            f"{button_part.name} presses inward along the control panel",
            button_rest is not None
            and button_open is not None
            and button_open[1] < button_rest[1] - 0.001
            and button_open[2] < button_rest[2] - 0.0004,
            details=f"rest={button_rest}, pressed={button_open}",
        )

    return ctx.report()


object_model = build_object_model()
