from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


WASHER_WIDTH = 0.598
WASHER_DEPTH = 0.570
WASHER_HEIGHT = 0.820


def wp_box(
    sx: float,
    sy: float,
    sz: float,
    *,
    translate: tuple[float, float, float] = (0.0, 0.0, 0.0),
    centered: tuple[bool, bool, bool] = (True, True, True),
) -> cq.Workplane:
    return cq.Workplane("XY").box(sx, sy, sz, centered=centered).translate(translate)


def build_cabinet_shell() -> cq.Workplane:
    side_wall = 0.016
    side_depth = 0.520
    side_front = -0.238
    back_wall = 0.014

    shell = wp_box(
        side_wall,
        side_depth,
        WASHER_HEIGHT,
        translate=(-WASHER_WIDTH / 2.0 + side_wall / 2.0, side_front + side_depth / 2.0, 0.0),
        centered=(True, True, False),
    )
    shell = shell.union(
        wp_box(
            side_wall,
            side_depth,
            WASHER_HEIGHT,
            translate=(WASHER_WIDTH / 2.0 - side_wall / 2.0, side_front + side_depth / 2.0, 0.0),
            centered=(True, True, False),
        )
    )
    shell = shell.union(
        wp_box(
            WASHER_WIDTH - 2.0 * side_wall,
            back_wall,
            WASHER_HEIGHT,
            translate=(0.0, WASHER_DEPTH / 2.0 - back_wall / 2.0, 0.0),
            centered=(True, True, False),
        )
    )
    shell = shell.union(
        wp_box(
            WASHER_WIDTH - 2.0 * side_wall,
            WASHER_DEPTH - 0.110,
            0.060,
            translate=(0.0, -0.180 + (WASHER_DEPTH - 0.110) / 2.0, 0.0),
            centered=(True, True, False),
        )
    )
    shell = shell.union(
        wp_box(
            WASHER_WIDTH - 2.0 * side_wall,
            0.060,
            0.045,
            translate=(0.0, -0.195, WASHER_HEIGHT - 0.045),
            centered=(True, True, False),
        )
    )
    shell = shell.union(
        wp_box(
            WASHER_WIDTH - 2.0 * side_wall,
            0.040,
            0.030,
            translate=(0.0, WASHER_DEPTH / 2.0 - 0.020, WASHER_HEIGHT - 0.030),
            centered=(True, True, False),
        )
    )
    return shell


def build_tub_shell() -> cq.Workplane:
    tub_width = 0.548
    tub_depth = 0.505
    tub_height = 0.700
    wall = 0.006
    back_wall = 0.008
    top_wall = 0.010
    bottom_wall = 0.012

    tub = wp_box(tub_width, tub_depth, tub_height, centered=(True, False, False))
    tub = tub.cut(
        wp_box(
            tub_width - 2.0 * wall,
            tub_depth - back_wall + 0.002,
            tub_height - bottom_wall - top_wall,
            translate=(0.0, -back_wall / 2.0 - 0.001, bottom_wall),
            centered=(True, True, False),
        )
    )
    return tub


def build_door_shell() -> cq.Workplane:
    door_width = 0.594
    door_height = 0.720
    door_thickness = 0.042
    frame = 0.028
    front_skin = 0.006
    liner_skin = 0.004

    door = wp_box(door_width, door_thickness, door_height, centered=(True, False, False))
    door = door.cut(
        wp_box(
            door_width - 2.0 * frame,
            door_thickness - front_skin - liner_skin,
            door_height - 2.0 * frame,
            translate=(0.0, front_skin, frame),
            centered=(True, False, False),
        )
    )

    handle_width = door_width - 0.100
    handle_depth = 0.016
    handle_height = 0.036
    handle_z = door_height - 0.125

    door = door.cut(
        wp_box(
            handle_width,
            handle_depth,
            handle_height,
            translate=(0.0, 0.0, handle_z),
            centered=(True, False, False),
        )
    )
    door = door.union(
        wp_box(
            handle_width,
            0.008,
            0.016,
            translate=(0.0, 0.004, handle_z + 0.010),
            centered=(True, False, False),
        )
    )
    return door


def union_all(shapes: list[cq.Workplane]) -> cq.Workplane:
    result = shapes[0]
    for shape in shapes[1:]:
        result = result.union(shape)
    return result


def rod_x(length: float, thickness: float, center: tuple[float, float, float]) -> cq.Workplane:
    return wp_box(length, thickness, thickness, translate=center)


def rod_y(length: float, thickness: float, center: tuple[float, float, float]) -> cq.Workplane:
    return wp_box(thickness, length, thickness, translate=center)


def rod_z(length: float, thickness: float, center: tuple[float, float, float]) -> cq.Workplane:
    return wp_box(thickness, thickness, length, translate=center)


def build_rack_mesh(
    width: float,
    depth: float,
    height: float,
    *,
    rod: float,
    floor_count: int,
    rail_count: int,
    divider_count: int,
) -> cq.Workplane:
    wire = rod * 0.68
    shapes = [
        rod_x(width, rod, (0.0, rod / 2.0, rod / 2.0)),
        rod_x(width, rod, (0.0, depth - rod / 2.0, rod / 2.0)),
        rod_y(depth, rod, (-width / 2.0 + rod / 2.0, depth / 2.0, rod / 2.0)),
        rod_y(depth, rod, (width / 2.0 - rod / 2.0, depth / 2.0, rod / 2.0)),
        rod_x(width, rod, (0.0, rod / 2.0, height - rod / 2.0)),
        rod_x(width, rod, (0.0, depth - rod / 2.0, height - rod / 2.0)),
        rod_y(depth, rod, (-width / 2.0 + rod / 2.0, depth / 2.0, height - rod / 2.0)),
        rod_y(depth, rod, (width / 2.0 - rod / 2.0, depth / 2.0, height - rod / 2.0)),
    ]

    for x_sign in (-1.0, 1.0):
        for y_center in (rod / 2.0, depth - rod / 2.0):
            shapes.append(
                rod_z(
                    height,
                    rod,
                    (x_sign * (width / 2.0 - rod / 2.0), y_center, height / 2.0),
                )
            )

    for index in range(floor_count):
        y_pos = 0.055 + index * (depth - 0.110) / max(floor_count - 1, 1)
        shapes.append(rod_x(width - 2.5 * rod, wire, (0.0, y_pos, rod * 1.25)))

    for index in range(rail_count):
        x_pos = -width * 0.34 + index * (width * 0.68) / max(rail_count - 1, 1)
        shapes.append(rod_y(depth - 0.035, wire, (x_pos, depth / 2.0 + 0.010, rod * 1.2)))

    for index in range(divider_count):
        x_pos = -width * 0.28 + index * (width * 0.56) / max(divider_count - 1, 1)
        shapes.append(rod_z(height * 0.58, wire, (x_pos, depth * 0.62, height * 0.29)))

    shapes.append(rod_x(width * 0.56, rod, (0.0, rod / 2.0, height * 0.78)))
    return union_all(shapes)


def build_cutlery_tray_mesh(width: float, depth: float, height: float, *, rod: float) -> cq.Workplane:
    wire = rod * 0.70
    shapes = [
        rod_x(width, rod, (0.0, rod / 2.0, rod / 2.0)),
        rod_x(width, rod, (0.0, depth - rod / 2.0, rod / 2.0)),
        rod_y(depth, rod, (-width / 2.0 + rod / 2.0, depth / 2.0, rod / 2.0)),
        rod_y(depth, rod, (width / 2.0 - rod / 2.0, depth / 2.0, rod / 2.0)),
        rod_x(width, rod, (0.0, rod / 2.0, height - rod / 2.0)),
        rod_x(width, rod, (0.0, depth - rod / 2.0, height - rod / 2.0)),
    ]

    for x_sign in (-1.0, 1.0):
        for y_center in (rod / 2.0, depth - rod / 2.0):
            shapes.append(
                rod_z(
                    height,
                    rod,
                    (x_sign * (width / 2.0 - rod / 2.0), y_center, height / 2.0),
                )
            )

    for index in range(8):
        y_pos = 0.040 + index * (depth - 0.080) / 7.0
        shapes.append(rod_x(width - 2.5 * rod, wire, (0.0, y_pos, rod * 1.1)))

    for index in range(3):
        x_pos = -width * 0.25 + index * (width * 0.25)
        shapes.append(rod_y(depth - 0.035, rod, (x_pos, depth / 2.0 + 0.010, height * 0.48)))

    shapes.append(rod_x(width * 0.46, rod, (0.0, rod / 2.0, height * 0.72)))
    return union_all(shapes)


def add_rod(
    part,
    axis: str,
    length: float,
    thickness: float,
    center: tuple[float, float, float],
    material,
    name: str,
) -> None:
    if axis == "x":
        geometry = Box((length, thickness, thickness))
    elif axis == "y":
        geometry = Box((thickness, length, thickness))
    else:
        geometry = Box((thickness, thickness, length))
    part.visual(geometry, origin=Origin(xyz=center), material=material, name=name)


def add_wire_rack(
    part,
    *,
    width: float,
    depth: float,
    height: float,
    rod: float,
    floor_count: int,
    rail_count: int,
    divider_count: int,
    material,
    prefix: str,
) -> None:
    wire = rod * 0.72
    add_rod(part, "x", width, rod, (0.0, rod / 2.0, rod / 2.0), material, f"{prefix}_front_base")
    add_rod(part, "x", width, rod, (0.0, depth - rod / 2.0, rod / 2.0), material, f"{prefix}_rear_base")
    add_rod(
        part,
        "y",
        depth + rod * 0.4,
        rod,
        (-width / 2.0 + rod / 2.0, depth / 2.0, rod / 2.0),
        material,
        f"{prefix}_left_base",
    )
    add_rod(
        part,
        "y",
        depth + rod * 0.4,
        rod,
        (width / 2.0 - rod / 2.0, depth / 2.0, rod / 2.0),
        material,
        f"{prefix}_right_base",
    )
    add_rod(part, "x", width, rod, (0.0, rod / 2.0, height - rod / 2.0), material, f"{prefix}_front_top")
    add_rod(part, "x", width, rod, (0.0, depth - rod / 2.0, height - rod / 2.0), material, f"{prefix}_rear_top")
    add_rod(
        part,
        "y",
        depth + rod * 0.4,
        rod,
        (-width / 2.0 + rod / 2.0, depth / 2.0, height - rod / 2.0),
        material,
        f"{prefix}_left_top",
    )
    add_rod(
        part,
        "y",
        depth + rod * 0.4,
        rod,
        (width / 2.0 - rod / 2.0, depth / 2.0, height - rod / 2.0),
        material,
        f"{prefix}_right_top",
    )

    for x_sign, x_name in ((-1.0, "left"), (1.0, "right")):
        for y_center, y_name in ((rod / 2.0, "front"), (depth - rod / 2.0, "rear")):
            add_rod(
                part,
                "z",
                height + rod * 0.3,
                rod,
                (x_sign * (width / 2.0 - rod / 2.0), y_center, height / 2.0),
                material,
                f"{prefix}_{x_name}_{y_name}_post",
            )

    for index in range(floor_count):
        y_pos = 0.055 + index * (depth - 0.110) / max(floor_count - 1, 1)
        add_rod(
            part,
            "x",
            width - 2.0 * rod,
            wire,
            (0.0, y_pos, rod * 1.35),
            material,
            f"{prefix}_floor_{index}",
        )

    for index in range(rail_count):
        x_pos = -width * 0.34 + index * (width * 0.68) / max(rail_count - 1, 1)
        add_rod(
            part,
            "y",
            depth - 0.015,
            wire,
            (x_pos, depth / 2.0, rod * 1.30),
            material,
            f"{prefix}_rail_{index}",
        )

    for index in range(divider_count):
        x_pos = -width * 0.28 + index * (width * 0.56) / max(divider_count - 1, 1)
        add_rod(
            part,
            "z",
            height * 0.60,
            wire,
            (x_pos, depth * 0.62, height * 0.30),
            material,
            f"{prefix}_divider_{index}",
        )


def add_cutlery_tray(
    part,
    *,
    width: float,
    depth: float,
    height: float,
    rod: float,
    material,
    prefix: str,
) -> None:
    wire = rod * 0.72
    add_rod(part, "x", width, rod, (0.0, rod / 2.0, rod / 2.0), material, f"{prefix}_front")
    add_rod(part, "x", width, rod, (0.0, depth - rod / 2.0, rod / 2.0), material, f"{prefix}_rear")
    add_rod(part, "y", depth + rod * 0.4, rod, (-width / 2.0 + rod / 2.0, depth / 2.0, rod / 2.0), material, f"{prefix}_left")
    add_rod(part, "y", depth + rod * 0.4, rod, (width / 2.0 - rod / 2.0, depth / 2.0, rod / 2.0), material, f"{prefix}_right")
    add_rod(part, "x", width, rod, (0.0, rod / 2.0, height - rod / 2.0), material, f"{prefix}_front_top")
    add_rod(part, "x", width, rod, (0.0, depth - rod / 2.0, height - rod / 2.0), material, f"{prefix}_rear_top")
    for x_sign, x_name in ((-1.0, "left"), (1.0, "right")):
        for y_center, y_name in ((rod / 2.0, "front"), (depth - rod / 2.0, "rear")):
            add_rod(
                part,
                "z",
                height + rod * 0.3,
                rod,
                (x_sign * (width / 2.0 - rod / 2.0), y_center, height / 2.0),
                material,
                f"{prefix}_{x_name}_{y_name}_post",
            )
    for index in range(8):
        y_pos = 0.040 + index * (depth - 0.080) / 7.0
        add_rod(part, "x", width - 2.0 * rod, wire, (0.0, y_pos, rod * 1.15), material, f"{prefix}_floor_{index}")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="modern_dishwasher")

    stainless = model.material("stainless", rgba=(0.74, 0.75, 0.77, 1.0))
    graphite = model.material("graphite", rgba=(0.16, 0.17, 0.19, 1.0))
    tub_gray = model.material("tub_gray", rgba=(0.70, 0.72, 0.75, 1.0))
    strip_black = model.material("strip_black", rgba=(0.08, 0.09, 0.10, 1.0))
    control_black = model.material("control_black", rgba=(0.12, 0.12, 0.13, 1.0))
    detergent_gray = model.material("detergent_gray", rgba=(0.78, 0.79, 0.81, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(build_cabinet_shell(), "cabinet_shell"),
        material=graphite,
        name="cabinet_shell",
    )

    tub = model.part("tub")
    tub.visual(
        Box((0.548, 0.505, 0.012)),
        origin=Origin(xyz=(0.0, 0.2525, 0.006)),
        material=tub_gray,
        name="tub_floor",
    )
    tub.visual(
        Box((0.548, 0.008, 0.700)),
        origin=Origin(xyz=(0.0, 0.501, 0.350)),
        material=tub_gray,
        name="tub_back",
    )
    tub.visual(
        Box((0.008, 0.505, 0.700)),
        origin=Origin(xyz=(-0.270, 0.2525, 0.350)),
        material=tub_gray,
        name="tub_side_0",
    )
    tub.visual(
        Box((0.008, 0.505, 0.700)),
        origin=Origin(xyz=(0.270, 0.2525, 0.350)),
        material=tub_gray,
        name="tub_side_1",
    )
    tub.visual(
        Box((0.548, 0.030, 0.014)),
        origin=Origin(xyz=(0.0, 0.015, 0.693)),
        material=tub_gray,
        name="tub_front_lip",
    )
    tub.visual(
        Box((0.548, 0.020, 0.012)),
        origin=Origin(xyz=(0.0, 0.491, 0.694)),
        material=tub_gray,
        name="tub_rear_lip",
    )
    for side in (-1.0, 1.0):
        tub.visual(
            Box((0.032, 0.440, 0.014)),
            origin=Origin(xyz=(side * 0.250, 0.250, 0.125)),
            material=graphite,
            name=f"lower_rail_{0 if side < 0 else 1}",
        )
        tub.visual(
            Box((0.030, 0.425, 0.012)),
            origin=Origin(xyz=(side * 0.251, 0.245, 0.395)),
            material=graphite,
            name=f"upper_rail_{0 if side < 0 else 1}",
        )
        tub.visual(
            Box((0.026, 0.455, 0.010)),
            origin=Origin(xyz=(side * 0.253, 0.250, 0.612)),
            material=graphite,
            name=f"tray_rail_{0 if side < 0 else 1}",
        )
    tub.visual(
        Box((0.060, 0.090, 0.006)),
        origin=Origin(xyz=(-0.185, 0.220, -0.003)),
        material=graphite,
        name="mount_pad_0",
    )
    tub.visual(
        Box((0.060, 0.090, 0.006)),
        origin=Origin(xyz=(0.185, 0.220, -0.003)),
        material=graphite,
        name="mount_pad_1",
    )

    model.articulation(
        "cabinet_to_tub",
        ArticulationType.FIXED,
        parent=cabinet,
        child=tub,
        origin=Origin(xyz=(0.0, -0.235, 0.066)),
    )

    door = model.part("door")
    door.visual(
        mesh_from_cadquery(build_door_shell(), "door_shell"),
        material=stainless,
        name="door_shell",
    )
    door.visual(
        Box((0.500, 0.003, 0.052)),
        origin=Origin(xyz=(0.0, -0.0015, 0.682)),
        material=strip_black,
        name="status_strip",
    )
    door.visual(
        Box((0.118, 0.010, 0.090)),
        origin=Origin(xyz=(0.118, 0.036, 0.360)),
        material=detergent_gray,
        name="detergent_well",
    )
    for idx, x_pos in enumerate((-0.287, 0.287)):
        door.visual(
            Box((0.024, 0.008, 0.024)),
            origin=Origin(xyz=(x_pos, 0.045, 0.012)),
            material=graphite,
            name=f"hinge_tab_{idx}",
        )

    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(0.0, -WASHER_DEPTH / 2.0 - 0.002, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(100.0),
        ),
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.022, length=0.015),
        origin=Origin(xyz=(0.0, -0.0075, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=control_black,
        name="dial_body",
    )
    dial.visual(
        Box((0.004, 0.002, 0.012)),
        origin=Origin(xyz=(0.0, -0.015, 0.014)),
        material=detergent_gray,
        name="dial_marker",
    )
    model.articulation(
        "door_to_dial",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=dial,
        origin=Origin(xyz=(-0.150, -0.003, 0.682)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )

    for idx, x_pos in enumerate((0.048, 0.086, 0.124)):
        button = model.part(f"button_{idx}")
        button.visual(
            Box((0.026, 0.006, 0.012)),
            origin=Origin(xyz=(0.0, -0.003, 0.0)),
            material=control_black,
            name="button_cap",
        )
        button.visual(
            Box((0.018, 0.004, 0.008)),
            origin=Origin(xyz=(0.0, 0.001, 0.0)),
            material=strip_black,
            name="button_stem",
        )
        model.articulation(
            f"door_to_button_{idx}",
            ArticulationType.PRISMATIC,
            parent=door,
            child=button,
            origin=Origin(xyz=(x_pos, -0.003, 0.682)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.5,
                velocity=0.03,
                lower=0.0,
                upper=0.003,
            ),
        )

    detergent_cover = model.part("detergent_cover")
    detergent_cover.visual(
        Box((0.106, 0.004, 0.076)),
        origin=Origin(xyz=(0.0, 0.002, -0.038)),
        material=detergent_gray,
        name="cover_panel",
    )
    detergent_cover.visual(
        Box((0.040, 0.008, 0.012)),
        origin=Origin(xyz=(0.0, 0.006, -0.072)),
        material=control_black,
        name="cover_lip",
    )
    model.articulation(
        "door_to_detergent_cover",
        ArticulationType.REVOLUTE,
        parent=door,
        child=detergent_cover,
        origin=Origin(xyz=(0.118, 0.038, 0.402)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(110.0),
        ),
    )

    lower_rack = model.part("lower_rack")
    add_wire_rack(
        lower_rack,
        width=0.478,
        depth=0.470,
        height=0.125,
        rod=0.006,
        floor_count=6,
        rail_count=4,
        divider_count=0,
        material=tub_gray,
        prefix="lower",
    )
    lower_rack.visual(
        Box((0.010, 0.032, 0.050)),
        origin=Origin(xyz=(-0.229, 0.210, 0.025)),
        material=graphite,
        name="guide_0",
    )
    lower_rack.visual(
        Box((0.010, 0.032, 0.050)),
        origin=Origin(xyz=(0.229, 0.210, 0.025)),
        material=graphite,
        name="guide_1",
    )
    model.articulation(
        "tub_to_lower_rack",
        ArticulationType.PRISMATIC,
        parent=tub,
        child=lower_rack,
        origin=Origin(xyz=(0.0, 0.010, 0.082)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=0.22,
            lower=0.0,
            upper=0.285,
        ),
    )

    upper_rack = model.part("upper_rack")
    add_wire_rack(
        upper_rack,
        width=0.468,
        depth=0.455,
        height=0.102,
        rod=0.0055,
        floor_count=5,
        rail_count=4,
        divider_count=0,
        material=tub_gray,
        prefix="upper",
    )
    upper_rack.visual(
        Box((0.012, 0.030, 0.050)),
        origin=Origin(xyz=(-0.230, 0.205, 0.025)),
        material=graphite,
        name="guide_0",
    )
    upper_rack.visual(
        Box((0.012, 0.030, 0.050)),
        origin=Origin(xyz=(0.230, 0.205, 0.025)),
        material=graphite,
        name="guide_1",
    )
    upper_rack.visual(
        Cylinder(radius=0.007, length=0.036),
        origin=Origin(xyz=(0.0, 0.232, 0.018)),
        material=graphite,
        name="spray_manifold",
    )
    model.articulation(
        "tub_to_upper_rack",
        ArticulationType.PRISMATIC,
        parent=tub,
        child=upper_rack,
        origin=Origin(xyz=(0.0, 0.016, 0.346)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.22,
            lower=0.0,
            upper=0.255,
        ),
    )

    cutlery_tray = model.part("cutlery_tray")
    add_cutlery_tray(
        cutlery_tray,
        width=0.460,
        depth=0.435,
        height=0.048,
        rod=0.005,
        material=tub_gray,
        prefix="tray",
    )
    cutlery_tray.visual(
        Box((0.010, 0.028, 0.028)),
        origin=Origin(xyz=(-0.235, 0.198, 0.014)),
        material=graphite,
        name="guide_0",
    )
    cutlery_tray.visual(
        Box((0.010, 0.028, 0.028)),
        origin=Origin(xyz=(0.235, 0.198, 0.014)),
        material=graphite,
        name="guide_1",
    )
    model.articulation(
        "tub_to_cutlery_tray",
        ArticulationType.PRISMATIC,
        parent=tub,
        child=cutlery_tray,
        origin=Origin(xyz=(0.0, 0.020, 0.588)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.20,
            lower=0.0,
            upper=0.220,
        ),
    )

    lower_spray_arm = model.part("lower_spray_arm")
    lower_spray_arm.visual(
        Cylinder(radius=0.016, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=graphite,
        name="hub",
    )
    lower_spray_arm.visual(
        Box((0.310, 0.018, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=graphite,
        name="arm_bar",
    )
    lower_spray_arm.visual(
        Box((0.065, 0.016, 0.008)),
        origin=Origin(xyz=(-0.120, 0.030, 0.004), rpy=(0.0, 0.0, 0.55)),
        material=graphite,
        name="arm_tip_0",
    )
    lower_spray_arm.visual(
        Box((0.060, 0.016, 0.008)),
        origin=Origin(xyz=(0.125, -0.028, 0.004), rpy=(0.0, 0.0, -0.50)),
        material=graphite,
        name="arm_tip_1",
    )
    lower_spray_arm.visual(
        Cylinder(radius=0.004, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=graphite,
        name="spindle",
    )
    model.articulation(
        "tub_to_lower_spray_arm",
        ArticulationType.CONTINUOUS,
        parent=tub,
        child=lower_spray_arm,
        origin=Origin(xyz=(0.0, 0.255, 0.034)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=12.0),
    )

    upper_spray_arm = model.part("upper_spray_arm")
    upper_spray_arm.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=graphite,
        name="hub",
    )
    upper_spray_arm.visual(
        Box((0.255, 0.016, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=graphite,
        name="arm_bar",
    )
    upper_spray_arm.visual(
        Box((0.055, 0.014, 0.008)),
        origin=Origin(xyz=(-0.095, -0.022, 0.004), rpy=(0.0, 0.0, -0.45)),
        material=graphite,
        name="arm_tip_0",
    )
    upper_spray_arm.visual(
        Box((0.055, 0.014, 0.008)),
        origin=Origin(xyz=(0.098, 0.024, 0.004), rpy=(0.0, 0.0, 0.45)),
        material=graphite,
        name="arm_tip_1",
    )
    upper_spray_arm.visual(
        Cylinder(radius=0.004, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=graphite,
        name="spindle",
    )
    model.articulation(
        "tub_to_upper_spray_arm",
        ArticulationType.CONTINUOUS,
        parent=tub,
        child=upper_spray_arm,
        origin=Origin(xyz=(0.0, 0.248, 0.382)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    tub = object_model.get_part("tub")
    door = object_model.get_part("door")
    door_hinge = object_model.get_articulation("cabinet_to_door")
    detergent_cover = object_model.get_part("detergent_cover")
    detergent_hinge = object_model.get_articulation("door_to_detergent_cover")
    lower_rack = object_model.get_part("lower_rack")
    upper_rack = object_model.get_part("upper_rack")
    cutlery_tray = object_model.get_part("cutlery_tray")
    lower_rack_joint = object_model.get_articulation("tub_to_lower_rack")
    upper_rack_joint = object_model.get_articulation("tub_to_upper_rack")
    tray_joint = object_model.get_articulation("tub_to_cutlery_tray")

    ctx.allow_overlap(
        cabinet,
        tub,
        elem_a="cabinet_shell",
        elem_b="mount_pad_0",
        reason="The tub's support pad is modeled as bearing on the cabinet base, and the cabinet shell mesh resolves that seating plane as a slight intersection.",
    )
    ctx.allow_overlap(
        cabinet,
        tub,
        elem_a="cabinet_shell",
        elem_b="mount_pad_1",
        reason="The tub's support pad is modeled as bearing on the cabinet base, and the cabinet shell mesh resolves that seating plane as a slight intersection.",
    )
    ctx.allow_overlap(
        upper_rack,
        "upper_spray_arm",
        elem_a="spray_manifold",
        elem_b="hub",
        reason="The upper spray arm is represented with a simplified hub nested over the rack-mounted feed manifold, so the bearing fit is modeled as a slight solid overlap.",
    )

    ctx.expect_gap(
        tub,
        door,
        axis="y",
        positive_elem="tub_front_lip",
        negative_elem="door_shell",
        min_gap=0.005,
        max_gap=0.018,
        name="closed door sits just in front of the tub opening",
    )
    ctx.expect_overlap(
        door,
        cabinet,
        axes="x",
        elem_a="door_shell",
        elem_b="cabinet_shell",
        min_overlap=0.56,
        name="door spans the full built-in opening width",
    )

    closed_aabb = ctx.part_element_world_aabb(door, elem="door_shell")
    with ctx.pose({door_hinge: math.radians(92.0)}):
        open_aabb = ctx.part_element_world_aabb(door, elem="door_shell")

    ctx.check(
        "door rotates down into an open loading shelf",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][1] < closed_aabb[0][1] - 0.45
        and open_aabb[1][2] < closed_aabb[1][2] - 0.55,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    dial_joint = object_model.get_articulation("door_to_dial")
    ctx.check(
        "front dial is authored as a continuous control",
        dial_joint.motion_limits is not None
        and dial_joint.motion_limits.lower is None
        and dial_joint.motion_limits.upper is None,
        details=str(dial_joint.motion_limits),
    )
    for spray_joint_name in ("tub_to_lower_spray_arm", "tub_to_upper_spray_arm"):
        spray_joint = object_model.get_articulation(spray_joint_name)
        ctx.check(
            f"{spray_joint_name} is authored as continuous rotation",
            spray_joint.motion_limits is not None
            and spray_joint.motion_limits.lower is None
            and spray_joint.motion_limits.upper is None,
            details=str(spray_joint.motion_limits),
        )

    for idx in range(3):
        button = object_model.get_part(f"button_{idx}")
        joint = object_model.get_articulation(f"door_to_button_{idx}")
        rest_pos = ctx.part_world_position(button)
        with ctx.pose({joint: 0.003}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"button_{idx} presses inward independently",
            rest_pos is not None and pressed_pos is not None and pressed_pos[1] > rest_pos[1] + 0.0015,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    closed_cover_aabb = ctx.part_element_world_aabb(detergent_cover, elem="cover_panel")
    with ctx.pose({detergent_hinge: math.radians(95.0)}):
        open_cover_aabb = ctx.part_element_world_aabb(detergent_cover, elem="cover_panel")
    ctx.check(
        "detergent cover opens away from the liner",
        closed_cover_aabb is not None
        and open_cover_aabb is not None
        and open_cover_aabb[1][1] > closed_cover_aabb[1][1] + 0.045,
        details=f"closed={closed_cover_aabb}, open={open_cover_aabb}",
    )

    for part_name, rack, joint, min_retained in (
        ("lower rack", lower_rack, lower_rack_joint, 0.17),
        ("upper rack", upper_rack, upper_rack_joint, 0.16),
        ("cutlery tray", cutlery_tray, tray_joint, 0.14),
    ):
        rest_pos = ctx.part_world_position(rack)
        ctx.expect_within(
            rack,
            tub,
            axes="x",
            margin=0.010,
            name=f"{part_name} stays centered between tub walls",
        )
        with ctx.pose({joint: joint.motion_limits.upper}):
            extended_pos = ctx.part_world_position(rack)
            ctx.expect_overlap(
                rack,
                tub,
                axes="y",
                min_overlap=min_retained,
                name=f"{part_name} remains retained on its guides at full extension",
            )
        ctx.check(
            f"{part_name} extends outward from the tub",
            rest_pos is not None and extended_pos is not None and extended_pos[1] < rest_pos[1] - 0.10,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    return ctx.report()


object_model = build_object_model()
