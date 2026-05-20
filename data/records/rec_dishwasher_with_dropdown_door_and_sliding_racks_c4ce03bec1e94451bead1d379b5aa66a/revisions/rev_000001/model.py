from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_W = 0.598
BODY_D = 0.570
BODY_H = 0.820
WALL = 0.018
DOOR_W = 0.594
DOOR_H = 0.745
DOOR_T = 0.035
HINGE_Z = 0.036
ROD = 0.006
JOIN = 0.002


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _union_box(
    shape: cq.Workplane | None,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
) -> cq.Workplane:
    piece = _box(size, center)
    return piece if shape is None else shape.union(piece)


def _make_tub_shape() -> cq.Workplane:
    cavity_w = BODY_W - 2.0 * WALL
    inner_height = BODY_H - WALL - 0.020
    cavity_depth = BODY_D - WALL + 0.020
    cavity_center_y = (BODY_D - WALL - 0.020) / 2.0
    cavity_center_z = 0.020 + inner_height / 2.0

    tub = _box((BODY_W, BODY_D, BODY_H), (0.0, BODY_D / 2.0, BODY_H / 2.0))
    cavity = _box((cavity_w, cavity_depth, inner_height), (0.0, cavity_center_y, cavity_center_z))
    tub = tub.cut(cavity)

    # Lower guide tracks.
    track_y_len = 0.445
    for sign in (-1.0, 1.0):
        tub = tub.union(
            _box(
                (0.016, track_y_len, 0.020),
                (sign * 0.237, 0.285, 0.030),
            )
        )

    # Upper rack side rails.
    for sign in (-1.0, 1.0):
        tub = tub.union(
            _box(
                (0.022, 0.432, 0.010),
                (sign * 0.272, 0.285, 0.467),
            )
        )

    # Third tray rails.
    for sign in (-1.0, 1.0):
        tub = tub.union(
            _box(
                (0.020, 0.422, 0.010),
                (sign * 0.273, 0.280, 0.695),
            )
        )

    # Interior sump/filter block to ground the lower wash arm hub.
    tub = tub.union(_box((0.115, 0.095, 0.056), (0.0, 0.345, 0.046)))
    tub = tub.union(_box((0.070, 0.055, 0.050), (0.0, 0.430, 0.043)))

    # Slight front rim around the opening.
    tub = tub.union(_box((BODY_W, 0.010, 0.030), (0.0, 0.005, BODY_H - 0.015)))
    return tub


def _make_door_shape(button_slots: list[tuple[float, float]]) -> cq.Workplane:
    door = _box((DOOR_W, DOOR_T, DOOR_H), (0.0, -DOOR_T / 2.0, DOOR_H / 2.0))

    liner_cut = _box((DOOR_W - 0.090, DOOR_T + 0.004, DOOR_H - 0.120), (0.0, -0.009, DOOR_H / 2.0 - 0.010))
    door = door.cut(liner_cut)

    # Underside finger pull recessed into the top back edge.
    handle_cut = _box((0.180, 0.018, 0.022), (0.0, -0.003, DOOR_H - 0.020))
    door = door.cut(handle_cut)

    # Top-edge button pockets.
    for x, width in button_slots:
        door = door.cut(_box((width + 0.006, 0.018, 0.018), (x, -DOOR_T / 2.0, DOOR_H - 0.003)))

    return door


def _make_rack_shape(
    *,
    width: float,
    depth: float,
    floor_z: float,
    top_z: float,
    front_offset: float,
    runner_out: float = 0.0,
    runner_z: float = 0.0,
    tine_rows: tuple[tuple[float, float, int], ...] = (),
    extra_dividers: tuple[float, ...] = (),
) -> cq.Workplane:
    y0 = front_offset
    y1 = front_offset + depth
    ym = (y0 + y1) / 2.0
    shape: cq.Workplane | None = None

    # Bottom perimeter.
    shape = _union_box(shape, (width + ROD, ROD, ROD), (0.0, y0, floor_z))
    shape = _union_box(shape, (width + ROD, ROD, ROD), (0.0, y1, floor_z))
    shape = _union_box(shape, (ROD, depth + ROD, ROD), (-width / 2.0, ym, floor_z))
    shape = _union_box(shape, (ROD, depth + ROD, ROD), (width / 2.0, ym, floor_z))

    # Top perimeter and side walls.
    shape = _union_box(shape, (width - 0.004, ROD, ROD), (0.0, y1, top_z))
    shape = _union_box(shape, (ROD, depth - 0.012, ROD), (-width / 2.0, ym + 0.010, top_z))
    shape = _union_box(shape, (ROD, depth - 0.012, ROD), (width / 2.0, ym + 0.010, top_z))
    for x in (-width / 2.0, width / 2.0):
        for y in (y0, y1):
            shape = _union_box(shape, (ROD, ROD, top_z - floor_z + ROD), (x, y, (top_z + floor_z) / 2.0))

    # Bottom cross members.
    for step in range(1, 5):
        y = y0 + depth * step / 5.0
        shape = _union_box(shape, (width - 0.008, ROD, ROD), (0.0, y, floor_z))

    # Front handle rail.
    handle_z = floor_z + 0.68 * (top_z - floor_z)
    shape = _union_box(shape, (width * 0.72, ROD, ROD), (0.0, y0 - 0.024, handle_z))
    for x in (-width * 0.34, width * 0.34):
        shape = _union_box(shape, (ROD, 0.024 + JOIN, top_z - handle_z + 0.014), (x, y0 - 0.012, (top_z + handle_z - 0.006) / 2.0))

    # Plate rows / cutlery dividers.
    for row_y, tine_h, count in tine_rows:
        usable = width - 0.060
        for i in range(count):
            x = -usable / 2.0 + usable * i / max(1, count - 1)
            shape = _union_box(shape, (ROD, ROD, tine_h + JOIN), (x, y0 + row_y, floor_z + tine_h / 2.0))

    for y in extra_dividers:
        shape = _union_box(shape, (width - 0.028, ROD, ROD), (0.0, y0 + y, top_z - 0.012))

    # Side runners for upper guides and third tray rails.
    if runner_out > 0.0:
        runner_x = width / 2.0 + runner_out
        for sign in (-1.0, 1.0):
            shape = _union_box(shape, (ROD, depth + 0.004, ROD), (sign * runner_x, ym, runner_z))
            for y in (y0 + 0.040, y1 - 0.040):
                shape = _union_box(shape, (runner_out * 2.0 + ROD, ROD, ROD), (sign * (width / 2.0 + runner_out / 2.0), y, runner_z))

    return shape.combine()


def _make_lower_wash_arm_shape() -> cq.Workplane:
    arm = cq.Workplane("XY").circle(0.018).extrude(0.016).translate((0.0, 0.0, -0.008))
    arm = arm.union(_box((0.250, 0.020, 0.010), (0.0, 0.0, -0.001)))
    arm = arm.union(_box((0.075, 0.038, 0.008), (-0.088, 0.0, -0.001)))
    arm = arm.union(_box((0.075, 0.038, 0.008), (0.088, 0.0, -0.001)))
    return arm.combine()


def _make_upper_wash_arm_shape() -> cq.Workplane:
    arm = cq.Workplane("XY").circle(0.014).extrude(0.014).translate((0.0, 0.0, -0.007))
    arm = arm.union(_box((0.215, 0.018, 0.008), (0.0, 0.0, -0.001)))
    arm = arm.union(_box((0.060, 0.032, 0.007), (-0.072, 0.0, -0.001)))
    arm = arm.union(_box((0.060, 0.032, 0.007), (0.072, 0.0, -0.001)))
    arm = arm.union(_box((0.018, 0.018, 0.032), (0.0, 0.0, 0.012)))
    return arm.combine()


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="panel_ready_dishwasher")

    panel = model.material("panel", rgba=(0.82, 0.82, 0.80, 1.0))
    stainless = model.material("stainless", rgba=(0.73, 0.75, 0.77, 1.0))
    rack_gray = model.material("rack_gray", rgba=(0.74, 0.76, 0.78, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.16, 0.17, 0.18, 1.0))
    arm_gray = model.material("arm_gray", rgba=(0.55, 0.58, 0.62, 1.0))
    start_green = model.material("start_green", rgba=(0.26, 0.46, 0.27, 1.0))

    button_slots = [
        (-0.098, 0.022),
        (-0.064, 0.022),
        (-0.030, 0.022),
        (0.016, 0.019),
        (0.050, 0.019),
        (0.108, 0.028),
    ]

    tub = model.part("tub")
    tub.visual(mesh_from_cadquery(_make_tub_shape(), "tub_shell"), material=stainless, name="tub_shell")

    door = model.part("door")
    door.visual(mesh_from_cadquery(_make_door_shape(button_slots), "door_shell"), material=panel, name="door_shell")

    lower_rack = model.part("lower_rack")
    lower_rack.visual(
        mesh_from_cadquery(
            _make_rack_shape(
                width=0.490,
                depth=0.470,
                floor_z=0.0,
                top_z=0.158,
                front_offset=0.032,
                tine_rows=((0.145, 0.095, 7), (0.310, 0.105, 8)),
            ),
            "lower_rack",
        ),
        material=rack_gray,
        name="lower_rack_wire",
    )
    for x in (-0.237, 0.237):
        for y in (0.070, 0.430):
            lower_rack.visual(
                Box((0.016, 0.014, 0.010)),
                origin=Origin(xyz=(x, y, -0.010)),
                material=dark_trim,
                name=f"wheel_{'n' if y < 0.20 else 'f'}_{'l' if x < 0.0 else 'r'}",
            )

    upper_rack = model.part("upper_rack")
    upper_rack.visual(
        mesh_from_cadquery(
            _make_rack_shape(
                width=0.464,
                depth=0.446,
                floor_z=-0.074,
                top_z=0.048,
                front_offset=0.028,
                runner_out=0.031,
                runner_z=0.005,
                tine_rows=((0.170, 0.062, 6), (0.320, 0.072, 6)),
                extra_dividers=(0.210,),
            ),
            "upper_rack",
        ),
        material=rack_gray,
        name="upper_rack_wire",
    )
    upper_rack.visual(
        Box((0.024, 0.040, 0.020)),
        origin=Origin(xyz=(0.0, 0.282, -0.014)),
        material=dark_trim,
        name="arm_mount",
    )

    cutlery_tray = model.part("cutlery_tray")
    cutlery_tray.visual(
        mesh_from_cadquery(
            _make_rack_shape(
                width=0.500,
                depth=0.420,
                floor_z=-0.014,
                top_z=0.028,
                front_offset=0.024,
                runner_out=0.014,
                runner_z=0.005,
                tine_rows=((0.110, 0.028, 8), (0.250, 0.028, 8)),
                extra_dividers=(0.125, 0.255),
            ),
            "cutlery_tray",
        ),
        material=rack_gray,
        name="cutlery_tray_wire",
    )

    lower_wash_arm = model.part("lower_wash_arm")
    lower_wash_arm.visual(
        mesh_from_cadquery(_make_lower_wash_arm_shape(), "lower_wash_arm"),
        material=arm_gray,
        name="lower_wash_arm_body",
    )

    upper_wash_arm = model.part("upper_wash_arm")
    upper_wash_arm.visual(
        mesh_from_cadquery(_make_upper_wash_arm_shape(), "upper_wash_arm"),
        material=arm_gray,
        name="upper_wash_arm_body",
    )

    # Concealed top-edge controls.
    button_specs = [
        ("program_button_0", 0.022, dark_trim, -0.098),
        ("program_button_1", 0.022, dark_trim, -0.064),
        ("program_button_2", 0.022, dark_trim, -0.030),
        ("cancel_button", 0.019, dark_trim, 0.016),
        ("start_button", 0.019, start_green, 0.050),
        ("latch_button", 0.028, dark_trim, 0.108),
    ]

    for name, width, material, _ in button_specs:
        button = model.part(name)
        button.visual(
            Box((width, 0.014, 0.004)),
            origin=Origin(xyz=(0.0, 0.0, -0.001)),
            material=material,
            name="cap",
        )
        button.visual(
            Box((max(0.010, width - 0.006), 0.010, 0.009)),
            origin=Origin(xyz=(0.0, 0.0, -0.0075)),
            material=material,
            name="stem",
        )

    door_hinge = model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=tub,
        child=door,
        origin=Origin(xyz=(0.0, 0.0, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.8,
            lower=0.0,
            upper=math.radians(96.0),
        ),
    )

    lower_rack_slide = model.articulation(
        "lower_rack_slide",
        ArticulationType.PRISMATIC,
        parent=tub,
        child=lower_rack,
        origin=Origin(xyz=(0.0, 0.042, 0.055)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.35, lower=0.0, upper=0.285),
    )

    upper_rack_slide = model.articulation(
        "upper_rack_slide",
        ArticulationType.PRISMATIC,
        parent=tub,
        child=upper_rack,
        origin=Origin(xyz=(0.0, 0.044, 0.470)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.30, lower=0.0, upper=0.265),
    )

    cutlery_tray_slide = model.articulation(
        "cutlery_tray_slide",
        ArticulationType.PRISMATIC,
        parent=tub,
        child=cutlery_tray,
        origin=Origin(xyz=(0.0, 0.045, 0.698)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.26, lower=0.0, upper=0.225),
    )

    model.articulation(
        "lower_wash_arm_spin",
        ArticulationType.CONTINUOUS,
        parent=tub,
        child=lower_wash_arm,
        origin=Origin(xyz=(0.0, 0.345, 0.070)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=20.0),
    )

    model.articulation(
        "upper_wash_arm_spin",
        ArticulationType.CONTINUOUS,
        parent=upper_rack,
        child=upper_wash_arm,
        origin=Origin(xyz=(0.0, 0.282, -0.052)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=20.0),
    )

    for name, width, _material, x in button_specs:
        upper = 0.0035 if name != "latch_button" else 0.004
        model.articulation(
            f"{name}_press",
            ArticulationType.PRISMATIC,
            parent=door,
            child=name,
            origin=Origin(xyz=(x, -DOOR_T / 2.0, DOOR_H)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=8.0, velocity=0.08, lower=0.0, upper=upper),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tub = object_model.get_part("tub")
    door = object_model.get_part("door")
    lower_rack = object_model.get_part("lower_rack")
    upper_rack = object_model.get_part("upper_rack")
    cutlery_tray = object_model.get_part("cutlery_tray")
    upper_wash_arm = object_model.get_part("upper_wash_arm")
    lower_wash_arm = object_model.get_part("lower_wash_arm")

    door_hinge = object_model.get_articulation("door_hinge")
    lower_rack_slide = object_model.get_articulation("lower_rack_slide")
    upper_rack_slide = object_model.get_articulation("upper_rack_slide")
    cutlery_tray_slide = object_model.get_articulation("cutlery_tray_slide")
    start_button_press = object_model.get_articulation("start_button_press")
    latch_button_press = object_model.get_articulation("latch_button_press")
    start_button = object_model.get_part("start_button")
    latch_button = object_model.get_part("latch_button")

    for button_name in (
        "program_button_0",
        "program_button_1",
        "program_button_2",
        "cancel_button",
        "start_button",
        "latch_button",
    ):
        ctx.allow_overlap(
            button_name,
            door,
            elem_a="stem",
            elem_b="door_shell",
            reason="The button stem is intentionally represented as nested into the concealed top-edge control pocket.",
        )

    for rack_name in ("lower_rack", "upper_rack", "cutlery_tray"):
        ctx.allow_overlap(
            rack_name,
            tub,
            reason="The rack-to-guide engagement is simplified as a retained sliding fit within the tub guide structure.",
        )

    ctx.allow_overlap(
        upper_wash_arm,
        upper_rack,
        elem_a="upper_wash_arm_body",
        elem_b="arm_mount",
        reason="The upper spray arm hub is intentionally captured by the central rack mount proxy.",
    )
    ctx.allow_overlap(
        lower_wash_arm,
        tub,
        elem_a="lower_wash_arm_body",
        elem_b="tub_shell",
        reason="The lower spray arm hub is intentionally seated into the sump pedestal proxy.",
    )

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(
            tub,
            door,
            axis="y",
            min_gap=0.0,
            max_gap=0.003,
            name="closed door sits just in front of the tub opening",
        )
        ctx.expect_overlap(
            door,
            tub,
            axes="xz",
            min_overlap=0.50,
            name="door covers the dishwasher opening footprint",
        )

    closed_aabb = ctx.part_element_world_aabb(door, elem="door_shell")
    with ctx.pose({door_hinge: door_hinge.motion_limits.upper}):
        open_aabb = ctx.part_element_world_aabb(door, elem="door_shell")
        ctx.check(
            "door opens downward on the lower hinge",
            closed_aabb is not None
            and open_aabb is not None
            and open_aabb[0][1] < closed_aabb[0][1] - 0.20
            and open_aabb[1][2] < closed_aabb[1][2] - 0.15,
            details=f"closed={closed_aabb}, open={open_aabb}",
        )

    for rack, joint, min_travel, label in (
        (lower_rack, lower_rack_slide, 0.22, "lower rack"),
        (upper_rack, upper_rack_slide, 0.20, "upper rack"),
        (cutlery_tray, cutlery_tray_slide, 0.16, "cutlery tray"),
    ):
        rest = ctx.part_world_position(rack)
        with ctx.pose({joint: joint.motion_limits.upper}):
            extended = ctx.part_world_position(rack)
        ctx.check(
            f"{label} slides outward",
            rest is not None and extended is not None and extended[1] < rest[1] - min_travel,
            details=f"rest={rest}, extended={extended}",
        )

    ctx.expect_overlap(
        lower_wash_arm,
        tub,
        axes="xy",
        min_overlap=0.03,
        name="lower wash arm remains centered over the lower sump zone",
    )
    ctx.expect_overlap(
        upper_wash_arm,
        upper_rack,
        axes="xy",
        min_overlap=0.03,
        name="upper wash arm stays under the upper rack footprint",
    )

    for button, joint, label in (
        (start_button, start_button_press, "start button"),
        (latch_button, latch_button_press, "latch button"),
    ):
        rest = ctx.part_world_position(button)
        with ctx.pose({joint: joint.motion_limits.upper}):
            pressed = ctx.part_world_position(button)
        ctx.check(
            f"{label} presses into the top control edge",
            rest is not None and pressed is not None and pressed[2] < rest[2] - 0.002,
            details=f"rest={rest}, pressed={pressed}",
        )

    return ctx.report()


object_model = build_object_model()
