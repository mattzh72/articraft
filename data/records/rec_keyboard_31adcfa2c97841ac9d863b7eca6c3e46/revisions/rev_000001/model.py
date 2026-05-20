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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


TRAY_W = 0.320
TRAY_D = 0.150
BASE_H = 0.014
TRAY_TOP_Z = BASE_H
SWITCH_TOP_Z = TRAY_TOP_Z + 0.004
KEY_TRAVEL = 0.004


def _add_switch_socket(tray, x: float, y: float, row: int, col: int, material) -> None:
    """Four low rails leave a central slot for the moving key stem."""
    rail_h = 0.0045
    z = TRAY_TOP_Z + rail_h / 2.0 - 0.0005
    outer_x = 0.018
    outer_y = 0.016
    rail_t = 0.003
    # Keep the tested home-row socket name literal so exact-geometry checks have
    # a stable visual contract.
    is_sample_socket = row == 1 and col == 4
    tray.visual(
        Box((outer_x, rail_t, rail_h)),
        origin=Origin(xyz=(x, y + outer_y / 2.0 - rail_t / 2.0, z)),
        material=material,
        name="switch_1_4_rear" if is_sample_socket else f"switch_{row}_{col}_rear",
    )
    tray.visual(
        Box((outer_x, rail_t, rail_h)),
        origin=Origin(xyz=(x, y - outer_y / 2.0 + rail_t / 2.0, z)),
        material=material,
        name=f"switch_{row}_{col}_front",
    )
    tray.visual(
        Box((rail_t, outer_y, rail_h)),
        origin=Origin(xyz=(x + outer_x / 2.0 - rail_t / 2.0, y, z)),
        material=material,
        name=f"switch_{row}_{col}_side_0",
    )
    tray.visual(
        Box((rail_t, outer_y, rail_h)),
        origin=Origin(xyz=(x - outer_x / 2.0 + rail_t / 2.0, y, z)),
        material=material,
        name=f"switch_{row}_{col}_side_1",
    )


def _add_key(
    model: ArticulatedObject,
    tray,
    *,
    row: int,
    col: int,
    x: float,
    y: float,
    width: float,
    depth: float,
    key_material,
    top_material,
    legend_material,
):
    key = model.part(f"key_{row}_{col}")
    key.visual(
        Cylinder(radius=0.003, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=top_material,
        name="stem",
    )
    key.visual(
        Box((width, depth, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=key_material,
        name="skirt",
    )
    key.visual(
        Box((max(width - 0.003, 0.010), max(depth - 0.003, 0.010), 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=top_material,
        name="top_step",
    )
    key.visual(
        Box((min(width * 0.36, 0.007), 0.0014, 0.0005)),
        origin=Origin(xyz=(0.0, -depth * 0.18, 0.01625)),
        material=legend_material,
        name="legend",
    )
    model.articulation(
        f"tray_to_key_{row}_{col}",
        ArticulationType.PRISMATIC,
        parent=tray,
        child=key,
        origin=Origin(xyz=(x, y, SWITCH_TOP_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=KEY_TRAVEL, effort=8.0, velocity=0.18),
    )
    return key


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_trackpoint_keyboard")

    tray_mat = model.material("matte_graphite", rgba=(0.035, 0.038, 0.042, 1.0))
    lip_mat = model.material("soft_black", rgba=(0.008, 0.009, 0.011, 1.0))
    socket_mat = model.material("switch_black", rgba=(0.012, 0.013, 0.015, 1.0))
    key_mat = model.material("charcoal_key_sides", rgba=(0.055, 0.058, 0.064, 1.0))
    key_top_mat = model.material("satin_key_tops", rgba=(0.090, 0.094, 0.104, 1.0))
    legend_mat = model.material("muted_legends", rgba=(0.65, 0.68, 0.70, 1.0))
    red_rubber = model.material("trackpoint_red", rgba=(0.76, 0.035, 0.025, 1.0))
    rubber_black = model.material("black_rubber", rgba=(0.004, 0.004, 0.004, 1.0))
    wheel_mat = model.material("knurled_dark_metal", rgba=(0.12, 0.125, 0.13, 1.0))

    tray = model.part("tray")
    tray_shape = cq.Workplane("XY").box(TRAY_W, TRAY_D, BASE_H).edges("|Z").fillet(0.010)
    tray.visual(
        mesh_from_cadquery(tray_shape, "rounded_keyboard_tray", tolerance=0.0008),
        origin=Origin(xyz=(0.0, 0.0, BASE_H / 2.0)),
        material=tray_mat,
        name="rounded_tray",
    )

    # Low perimeter lip makes the body read as a shallow compact tray.
    lip_h = 0.006
    lip_z = TRAY_TOP_Z + lip_h / 2.0 - 0.0004
    tray.visual(Box((TRAY_W - 0.030, 0.006, lip_h)), origin=Origin(xyz=(0.0, -0.071, lip_z)), material=lip_mat, name="front_lip")
    tray.visual(Box((TRAY_W - 0.030, 0.006, lip_h)), origin=Origin(xyz=(0.0, 0.071, lip_z)), material=lip_mat, name="rear_lip")
    tray.visual(Box((0.006, TRAY_D - 0.026, lip_h)), origin=Origin(xyz=(-0.154, 0.0, lip_z)), material=lip_mat, name="side_lip_0")
    tray.visual(Box((0.006, TRAY_D - 0.026, lip_h)), origin=Origin(xyz=(0.154, 0.0, lip_z)), material=lip_mat, name="side_lip_1")

    # Raised pod for the corner media wheel.
    tray.visual(
        Box((0.052, 0.036, 0.016)),
        origin=Origin(xyz=(0.132, 0.054, TRAY_TOP_Z + 0.008)),
        material=lip_mat,
        name="wheel_pod",
    )
    tray.visual(
        Cylinder(radius=0.0175, length=0.004),
        origin=Origin(xyz=(0.132, 0.054, TRAY_TOP_Z + 0.014)),
        material=tray_mat,
        name="wheel_socket",
    )

    col_xs = [(col - 4.5) * 0.026 for col in range(10)]
    row_ys = [0.030, 0.007, -0.016, -0.039]
    for row, y in enumerate(row_ys):
        for col, x in enumerate(col_xs):
            _add_switch_socket(tray, x, y, row, col, socket_mat)
            _add_key(
                model,
                tray,
                row=row,
                col=col,
                x=x,
                y=y,
                width=0.019,
                depth=0.017,
                key_material=key_mat,
                top_material=key_top_mat,
                legend_material=legend_mat,
            )

    # Compliant pointing stick between the two central home-row keys.
    tray.visual(
        Cylinder(radius=0.0045, length=0.003),
        origin=Origin(xyz=(0.0, row_ys[1], TRAY_TOP_Z + 0.0015)),
        material=rubber_black,
        name="trackpoint_socket",
    )
    nub_stem = model.part("nub_stem")
    nub_stem.visual(
        Cylinder(radius=0.0018, length=0.011),
        origin=Origin(xyz=(0.0, 0.0, 0.0055)),
        material=rubber_black,
        name="flex_stem",
    )
    model.articulation(
        "tray_to_nub_stem",
        ArticulationType.REVOLUTE,
        parent=tray,
        child=nub_stem,
        origin=Origin(xyz=(0.0, row_ys[1], TRAY_TOP_Z + 0.003)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.18, upper=0.18, effort=0.18, velocity=1.5),
    )

    trackpoint_nub = model.part("trackpoint_nub")
    trackpoint_nub.visual(
        Cylinder(radius=0.0030, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=red_rubber,
        name="nub_body",
    )
    trackpoint_nub.visual(
        Sphere(radius=0.0030),
        origin=Origin(xyz=(0.0, 0.0, 0.0034)),
        material=red_rubber,
        name="rounded_tip",
    )
    model.articulation(
        "nub_stem_to_trackpoint_nub",
        ArticulationType.REVOLUTE,
        parent=nub_stem,
        child=trackpoint_nub,
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.16, upper=0.16, effort=0.12, velocity=1.5),
    )

    # Rotating wheel sitting on the top-edge pod.
    media_wheel = model.part("media_wheel")
    media_wheel.visual(
        Cylinder(radius=0.013, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=wheel_mat,
        name="wheel_disk",
    )
    media_wheel.visual(
        Cylinder(radius=0.006, length=0.011),
        origin=Origin(xyz=(0.0, 0.0, 0.0055)),
        material=tray_mat,
        name="center_hub",
    )
    for i in range(16):
        angle = i * math.tau / 16.0
        media_wheel.visual(
            Box((0.0020, 0.0042, 0.0105)),
            origin=Origin(
                xyz=(math.cos(angle) * 0.0138, math.sin(angle) * 0.0138, 0.0052),
                rpy=(0.0, 0.0, angle),
            ),
            material=wheel_mat,
            name=f"grip_{i}",
        )
    model.articulation(
        "tray_to_media_wheel",
        ArticulationType.CONTINUOUS,
        parent=tray,
        child=media_wheel,
        origin=Origin(xyz=(0.132, 0.054, TRAY_TOP_Z + 0.016)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.25, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tray = object_model.get_part("tray")
    key = object_model.get_part("key_1_4")
    key_joint = object_model.get_articulation("tray_to_key_1_4")
    wheel = object_model.get_part("media_wheel")
    wheel_joint = object_model.get_articulation("tray_to_media_wheel")
    stem = object_model.get_part("nub_stem")
    nub = object_model.get_part("trackpoint_nub")
    stem_joint = object_model.get_articulation("tray_to_nub_stem")
    nub_joint = object_model.get_articulation("nub_stem_to_trackpoint_nub")

    ctx.expect_contact(
        key,
        tray,
        elem_a="stem",
        elem_b="rounded_tray",
        contact_tol=0.0008,
        name="sample key stem is seated at its switch socket",
    )

    rest_key_pos = ctx.part_world_position(key)
    with ctx.pose({key_joint: KEY_TRAVEL}):
        pressed_key_pos = ctx.part_world_position(key)
    ctx.check(
        "sample key plunges downward",
        rest_key_pos is not None
        and pressed_key_pos is not None
        and pressed_key_pos[2] < rest_key_pos[2] - 0.003,
        details=f"rest={rest_key_pos}, pressed={pressed_key_pos}",
    )

    rest_nub_aabb = ctx.part_world_aabb(nub)
    with ctx.pose({stem_joint: 0.16, nub_joint: -0.12}):
        tilted_nub_aabb = ctx.part_world_aabb(nub)
    ctx.check(
        "trackpoint nub tilts on compliant mount",
        rest_nub_aabb is not None
        and tilted_nub_aabb is not None
        and abs(((tilted_nub_aabb[0][1] + tilted_nub_aabb[1][1]) * 0.5) - ((rest_nub_aabb[0][1] + rest_nub_aabb[1][1]) * 0.5)) > 0.001,
        details=f"rest={rest_nub_aabb}, tilted={tilted_nub_aabb}",
    )

    rest_wheel_pos = ctx.part_world_position(wheel)
    with ctx.pose({wheel_joint: math.pi * 0.75}):
        turned_wheel_pos = ctx.part_world_position(wheel)
    ctx.check(
        "media wheel spins about a fixed corner axis",
        rest_wheel_pos is not None
        and turned_wheel_pos is not None
        and max(abs(rest_wheel_pos[i] - turned_wheel_pos[i]) for i in range(3)) < 1e-6,
        details=f"rest={rest_wheel_pos}, turned={turned_wheel_pos}",
    )
    ctx.expect_contact(wheel, tray, elem_a="wheel_disk", elem_b="wheel_socket", contact_tol=0.0008, name="wheel sits on pod socket")
    ctx.expect_contact(stem, tray, elem_a="flex_stem", elem_b="trackpoint_socket", contact_tol=0.0008, name="trackpoint stem emerges from socket")

    return ctx.report()


object_model = build_object_model()
