from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_side_loft,
)

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.

# >>> USER_CODE_START
ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root

HALF_BARREL_X = 0.043
HINGE_OPEN_LIMIT = math.radians(18.0)
HINGE_CLOSE_LIMIT = -math.radians(16.0)


def _make_material(name: str, rgba: tuple[float, float, float, float]) -> Material:
    for kwargs in (
        {"name": name, "rgba": rgba},
        {"name": name, "color": rgba},
    ):
        try:
            return Material(**kwargs)
        except TypeError:
            pass
    try:
        return Material(name, rgba)
    except TypeError:
        return Material(name=name)


ARMORED_BLACK = _make_material("armored_black", (0.13, 0.14, 0.15, 1.0))
GRAPHITE_METAL = _make_material("graphite_metal", (0.24, 0.25, 0.27, 1.0))
BRUSHED_STEEL = _make_material("brushed_steel", (0.56, 0.58, 0.61, 1.0))
TEXTURED_RUBBER = _make_material("textured_rubber", (0.08, 0.08, 0.09, 1.0))
OPTIC_GLASS = _make_material("optic_glass", (0.16, 0.22, 0.28, 0.45))


def _y_axis_origin(xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=(-math.pi / 2.0, 0.0, 0.0))


def _x_axis_origin(xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0))


def _shell_mesh(filename: str, x_center: float):
    sections = [
        (-0.048, -0.013, 0.013, 0.028),
        (-0.036, -0.015, 0.017, 0.034),
        (-0.012, -0.018, 0.023, 0.044),
        (0.012, -0.017, 0.024, 0.047),
        (0.032, -0.015, 0.021, 0.045),
    ]
    shell = superellipse_side_loft(
        sections,
        exponents=[2.15, 2.35, 2.8, 2.85, 2.55],
        segments=56,
    )
    shell.translate(x_center, 0.006, 0.0)
    return mesh_from_geometry(shell, ASSETS.mesh_path(filename))


def _add_barrel_half(part, sign: float, shell_mesh, include_hinge_core: bool) -> None:
    barrel_x = sign * HALF_BARREL_X
    part.visual(shell_mesh, origin=Origin(), material=ARMORED_BLACK)

    part.visual(
        Box((0.032, 0.024, 0.018)),
        origin=Origin(xyz=(sign * 0.017, -0.002, 0.003)),
        material=GRAPHITE_METAL,
    )
    part.visual(
        Box((0.020, 0.028, 0.010)),
        origin=Origin(xyz=(sign * 0.030, -0.003, 0.022)),
        material=GRAPHITE_METAL,
    )
    part.visual(
        Box((0.018, 0.036, 0.008)),
        origin=Origin(xyz=(barrel_x, -0.004, 0.023)),
        material=GRAPHITE_METAL,
    )

    part.visual(
        Cylinder(radius=0.0225, length=0.062),
        origin=_y_axis_origin((barrel_x, 0.057, 0.0)),
        material=ARMORED_BLACK,
    )
    part.visual(
        Cylinder(radius=0.0248, length=0.006),
        origin=_y_axis_origin((barrel_x, 0.090, 0.0)),
        material=BRUSHED_STEEL,
    )
    for rib_y in (0.028, 0.037, 0.046, 0.055):
        part.visual(
            Cylinder(radius=0.0242, length=0.003),
            origin=_y_axis_origin((barrel_x, rib_y, 0.0)),
            material=TEXTURED_RUBBER,
        )

    part.visual(
        Cylinder(radius=0.0205, length=0.002),
        origin=_y_axis_origin((barrel_x, 0.092, 0.0)),
        material=OPTIC_GLASS,
    )

    part.visual(
        Cylinder(radius=0.0175, length=0.020),
        origin=_y_axis_origin((barrel_x, -0.056, 0.0)),
        material=GRAPHITE_METAL,
    )
    part.visual(
        Cylinder(radius=0.0187, length=0.004),
        origin=_y_axis_origin((barrel_x, -0.064, 0.0)),
        material=BRUSHED_STEEL,
    )
    part.visual(
        Cylinder(radius=0.0198, length=0.016),
        origin=_y_axis_origin((barrel_x, -0.075, 0.0)),
        material=TEXTURED_RUBBER,
    )
    part.visual(
        Cylinder(radius=0.0132, length=0.002),
        origin=_y_axis_origin((barrel_x, -0.082, 0.0)),
        material=OPTIC_GLASS,
    )

    part.visual(
        Box((0.006, 0.058, 0.030)),
        origin=Origin(xyz=(sign * 0.060, 0.004, 0.001)),
        material=TEXTURED_RUBBER,
    )
    for rib_y in (-0.020, -0.012, -0.004, 0.004, 0.012, 0.020):
        part.visual(
            Box((0.008, 0.004, 0.032)),
            origin=Origin(xyz=(sign * 0.060, rib_y, 0.001)),
            material=TEXTURED_RUBBER,
        )

    if include_hinge_core:
        part.visual(
            Cylinder(radius=0.006, length=0.050),
            origin=Origin(),
            material=BRUSHED_STEEL,
        )
        for cap_z in (-0.026, 0.026):
            part.visual(
                Cylinder(radius=0.009, length=0.004),
                origin=Origin(xyz=(0.0, 0.0, cap_z)),
                material=GRAPHITE_METAL,
            )
        part.visual(
            Box((0.020, 0.022, 0.018)),
            origin=Origin(xyz=(-0.004, -0.013, 0.014)),
            material=GRAPHITE_METAL,
        )
        part.visual(
            Cylinder(radius=0.010, length=0.026),
            origin=_x_axis_origin((0.0, -0.018, 0.017)),
            material=GRAPHITE_METAL,
        )
        for wheel_x in (-0.009, -0.003, 0.003, 0.009):
            part.visual(
                Cylinder(radius=0.0115, length=0.002),
                origin=_x_axis_origin((wheel_x, -0.018, 0.017)),
                material=TEXTURED_RUBBER,
            )
    else:
        part.visual(
            Cylinder(radius=0.0092, length=0.046),
            origin=Origin(),
            material=GRAPHITE_METAL,
        )
        for flange_z in (-0.024, 0.024):
            part.visual(
                Cylinder(radius=0.011, length=0.004),
                origin=Origin(xyz=(0.0, 0.0, flange_z)),
                material=BRUSHED_STEEL,
            )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="binoculars", assets=ASSETS)

    left_body = model.part("left_body")
    left_shell = _shell_mesh("left_body_shell.obj", -HALF_BARREL_X)
    _add_barrel_half(left_body, -1.0, left_shell, include_hinge_core=True)
    left_body.inertial = Inertial.from_geometry(
        Box((0.090, 0.165, 0.060)),
        mass=0.42,
        origin=Origin(xyz=(-0.024, 0.004, 0.001)),
    )

    right_body = model.part("right_body")
    right_shell = _shell_mesh("right_body_shell.obj", HALF_BARREL_X)
    _add_barrel_half(right_body, 1.0, right_shell, include_hinge_core=False)
    right_body.inertial = Inertial.from_geometry(
        Box((0.090, 0.165, 0.060)),
        mass=0.36,
        origin=Origin(xyz=(0.024, 0.004, 0.001)),
    )

    model.articulation(
        "center_hinge",
        ArticulationType.REVOLUTE,
        parent="left_body",
        child="right_body",
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.2,
            lower=HINGE_CLOSE_LIMIT,
            upper=HINGE_OPEN_LIMIT,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=160,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_overlap("left_body", "right_body", axes="xy", min_overlap=0.014)
    ctx.expect_joint_motion_axis(
        "center_hinge",
        "right_body",
        world_axis="y",
        direction="positive",
        min_delta=0.02,
    )

    with ctx.pose(center_hinge=math.radians(8.0)):
        ctx.expect_aabb_overlap("left_body", "right_body", axes="xy", min_overlap=0.012)

    with ctx.pose(center_hinge=HINGE_OPEN_LIMIT):
        ctx.expect_aabb_overlap("left_body", "right_body", axes="xy", min_overlap=0.010)

    with ctx.pose(center_hinge=HINGE_CLOSE_LIMIT):
        ctx.expect_aabb_overlap("left_body", "right_body", axes="xy", min_overlap=0.016)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
