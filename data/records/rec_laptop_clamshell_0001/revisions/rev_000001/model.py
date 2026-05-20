from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
MESH_DIR = ASSETS.ensure_mesh_dir()


# >>> USER_CODE_START
IDENTITY = Origin(xyz=(0.0, 0.0, 0.0))


def _add_key_row(
    keyboard: cq.Workplane,
    *,
    y: float,
    widths: list[float],
    depth: float,
    key_height: float,
    z_center: float,
    gap: float,
) -> cq.Workplane:
    total_width = sum(widths) + gap * (len(widths) - 1)
    x_cursor = -0.5 * total_width
    for width in widths:
        key = (
            cq.Workplane("XY")
            .box(width, depth, key_height)
            .translate((x_cursor + 0.5 * width, y, z_center))
        )
        keyboard = keyboard.union(key)
        x_cursor += width + gap
    return keyboard


def _build_base_shell(
    *,
    width: float,
    depth: float,
    thickness: float,
    corner_radius: float,
) -> cq.Workplane:
    speaker_points = [
        (-0.122, 0.012),
        (-0.122, 0.028),
        (-0.122, 0.044),
        (-0.122, 0.060),
        (-0.122, 0.076),
        (0.122, 0.012),
        (0.122, 0.028),
        (0.122, 0.044),
        (0.122, 0.060),
        (0.122, 0.076),
    ]

    shell = cq.Workplane("XY").box(width, depth, thickness)
    shell = shell.edges("|Z").fillet(corner_radius)
    shell = (
        shell.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .center(0.0, 0.028)
        .rect(width - 0.074, 0.120)
        .cutBlind(-0.0008)
    )
    shell = (
        shell.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .center(0.0, -0.046)
        .rect(0.118, 0.082)
        .cutBlind(-0.00045)
    )
    shell = (
        shell.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(speaker_points)
        .slot2D(0.015, 0.0016, 90)
        .cutBlind(-0.0006)
    )
    return shell.translate((0.0, 0.0, 0.5 * thickness))


def _build_keyboard() -> cq.Workplane:
    plate_center_z = 0.01455
    key_center_z = 0.01525
    key_gap = 0.0025

    keyboard = cq.Workplane("XY").box(0.250, 0.118, 0.0007).translate((0.0, 0.028, plate_center_z))

    row_specs = [
        (0.079, [0.015] * 12, 0.008),
        (0.059, [0.015] * 13 + [0.024], 0.012),
        (0.039, [0.021] + [0.015] * 11 + [0.024], 0.013),
        (0.019, [0.024] + [0.015] * 10 + [0.027], 0.013),
        (-0.001, [0.028] + [0.015] * 10 + [0.020], 0.013),
        (-0.023, [0.019, 0.019, 0.019, 0.105, 0.019, 0.019, 0.019], 0.013),
    ]

    for y, widths, depth in row_specs:
        keyboard = _add_key_row(
            keyboard,
            y=y,
            widths=widths,
            depth=depth,
            key_height=0.0011,
            z_center=key_center_z,
            gap=key_gap,
        )

    return keyboard


def _build_lid_outer(
    *,
    width: float,
    depth: float,
    thickness: float,
    corner_radius: float,
) -> cq.Workplane:
    lid = cq.Workplane("XY").box(width, depth, thickness)
    lid = lid.edges("|Z").fillet(corner_radius)
    return lid.translate((0.0, -0.5 * depth, 0.5 * thickness))


def _build_lid_bezel(
    *,
    width: float,
    depth: float,
    screen_width: float,
    screen_depth: float,
    display_center_y: float,
) -> cq.Workplane:
    bezel_outer = (
        cq.Workplane("XY")
        .box(width - 0.016, depth - 0.020, 0.0018)
        .translate((0.0, display_center_y, 0.0013))
    )
    bezel_inner = (
        cq.Workplane("XY")
        .box(screen_width, screen_depth, 0.0032)
        .translate((0.0, display_center_y, 0.0016))
    )
    webcam_bar = cq.Workplane("XY").box(0.008, 0.0024, 0.00035).translate((0.0, -0.012, 0.00205))
    return bezel_outer.cut(bezel_inner).union(webcam_bar)


def build_object_model() -> ArticulatedObject:
    width = 0.320
    base_depth = 0.225
    lid_depth = 0.218
    base_thickness = 0.016
    lid_thickness = 0.0065
    corner_radius = 0.0105
    hinge_y = 0.106
    hinge_z = 0.0182
    display_center_y = -(0.5 * lid_depth) + 0.006
    screen_width = 0.286
    screen_depth = 0.180

    model = ArticulatedObject(name="refined_laptop", assets=ASSETS)
    model.material("chassis", rgba=(0.58, 0.59, 0.62, 1.0))
    model.material("keys", rgba=(0.09, 0.09, 0.10, 1.0))
    model.material("bezel", rgba=(0.05, 0.05, 0.06, 1.0))
    model.material("screen", rgba=(0.03, 0.05, 0.06, 1.0))
    model.material("trackpad", rgba=(0.25, 0.26, 0.28, 1.0))
    model.material("polymer", rgba=(0.18, 0.19, 0.20, 1.0))
    model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(
            _build_base_shell(
                width=width,
                depth=base_depth,
                thickness=base_thickness,
                corner_radius=corner_radius,
            ),
            "base_shell.obj",
            assets=ASSETS,
        ),
        origin=IDENTITY,
        material="chassis",
    )
    base.visual(
        mesh_from_cadquery(_build_keyboard(), "keyboard.obj", assets=ASSETS),
        origin=IDENTITY,
        material="keys",
    )
    base.visual(
        Box((0.110, 0.075, 0.0008)),
        origin=Origin(xyz=(0.0, -0.046, 0.01545)),
        material="trackpad",
    )
    base.visual(
        Box((0.096, 0.012, 0.0010)),
        origin=Origin(xyz=(0.0, -0.086, 0.00035)),
        material="rubber",
    )
    base.visual(
        Box((0.096, 0.012, 0.0010)),
        origin=Origin(xyz=(0.0, 0.084, 0.00035)),
        material="rubber",
    )
    base.inertial = Inertial.from_geometry(
        Box((width, base_depth, base_thickness)),
        mass=1.35,
        origin=Origin(xyz=(0.0, 0.0, 0.5 * base_thickness)),
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(
            _build_lid_outer(
                width=width,
                depth=lid_depth,
                thickness=lid_thickness,
                corner_radius=corner_radius,
            ),
            "lid_outer.obj",
            assets=ASSETS,
        ),
        origin=IDENTITY,
        material="chassis",
    )
    lid.visual(
        mesh_from_cadquery(
            _build_lid_bezel(
                width=width,
                depth=lid_depth,
                screen_width=screen_width,
                screen_depth=screen_depth,
                display_center_y=display_center_y,
            ),
            "lid_bezel.obj",
            assets=ASSETS,
        ),
        origin=IDENTITY,
        material="bezel",
    )
    lid.visual(
        Box((screen_width, screen_depth, 0.0008)),
        origin=Origin(xyz=(0.0, display_center_y, 0.0010)),
        material="screen",
    )
    lid.visual(
        Box((0.212, 0.0075, 0.00025)),
        origin=Origin(xyz=(0.0, -0.018, lid_thickness - 0.000125)),
        material="polymer",
    )
    lid.inertial = Inertial.from_geometry(
        Box((width, lid_depth, lid_thickness)),
        mass=0.75,
        origin=Origin(xyz=(0.0, -0.5 * lid_depth, 0.5 * lid_thickness)),
    )

    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=1.9,
            effort=4.0,
            velocity=2.5,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=96,
        overlap_tol=0.0015,
        overlap_volume_tol=0.0,
    )

    ctx.expect_aabb_overlap("lid", "base", axes="xy", min_overlap=0.18)
    ctx.expect_origin_gap("lid", "base", axis="z", min_gap=0.0)
    ctx.expect_aabb_gap("lid", "base", axis="z", max_gap=0.008, max_penetration=0.0)
    ctx.expect_joint_motion_axis(
        "base_to_lid",
        "lid",
        world_axis="z",
        direction="positive",
        min_delta=0.07,
    )

    with ctx.pose(base_to_lid=0.95):
        ctx.expect_aabb_overlap("lid", "base", axes="xy", min_overlap=0.09)
        ctx.expect_origin_gap("lid", "base", axis="z", min_gap=0.0)
        ctx.expect_aabb_gap("lid", "base", axis="z", max_gap=0.010, max_penetration=0.0)

    with ctx.pose(base_to_lid=1.55):
        ctx.expect_aabb_overlap("lid", "base", axes="xy", min_overlap=0.003)
        ctx.expect_origin_gap("lid", "base", axis="z", min_gap=0.0)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
