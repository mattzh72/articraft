from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


UPPER_REST_PITCH = 1.15


def _yz_section(
    x_pos: float,
    width: float,
    height: float,
    radius: float,
    *,
    z_center: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [(x_pos, y, z + z_center) for y, z in rounded_rect_profile(width, height, radius)]


def _body_shell_mesh():
    return section_loft(
        [
            _yz_section(-0.12, 0.10, 0.18, 0.020, z_center=0.18),
            _yz_section(-0.02, 0.15, 0.25, 0.030, z_center=0.22),
            _yz_section(0.10, 0.14, 0.22, 0.028, z_center=0.25),
            _yz_section(0.18, 0.10, 0.16, 0.020, z_center=0.23),
        ]
    )


def _nozzle_shell_mesh():
    return section_loft(
        [
            _yz_section(-0.01, 0.08, 0.06, 0.012, z_center=-0.050),
            _yz_section(0.02, 0.24, 0.036, 0.012, z_center=-0.060),
            _yz_section(0.08, 0.30, 0.032, 0.012, z_center=-0.062),
            _yz_section(0.12, 0.27, 0.020, 0.008, z_center=-0.060),
        ]
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="articulated_vacuum_cleaner")

    graphite = model.material("graphite", rgba=(0.18, 0.19, 0.20, 1.0))
    charcoal = model.material("charcoal", rgba=(0.10, 0.11, 0.12, 1.0))
    steel = model.material("steel", rgba=(0.68, 0.71, 0.75, 1.0))
    clear_bin = model.material("clear_bin", rgba=(0.70, 0.82, 0.92, 0.35))
    accent = model.material("accent", rgba=(0.78, 0.18, 0.10, 1.0))

    main_body = model.part("main_body")
    main_body.visual(
        mesh_from_geometry(_body_shell_mesh(), "vacuum_body_shell"),
        material=graphite,
        name="body_shell",
    )
    main_body.visual(
        Box((0.18, 0.10, 0.12)),
        origin=Origin(xyz=(-0.04, 0.0, 0.08)),
        material=charcoal,
        name="battery_pack",
    )
    main_body.visual(
        Cylinder(radius=0.055, length=0.22),
        origin=Origin(xyz=(0.05, 0.0, 0.18), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=clear_bin,
        name="dust_bin",
    )
    main_body.visual(
        Box((0.14, 0.05, 0.12)),
        origin=Origin(xyz=(-0.01, 0.0, 0.30), rpy=(0.0, -0.35, 0.0)),
        material=graphite,
        name="handle_bridge",
    )
    main_body.visual(
        Box((0.12, 0.045, 0.13)),
        origin=Origin(xyz=(-0.08, 0.0, 0.39), rpy=(0.0, -0.55, 0.0)),
        material=graphite,
        name="grip",
    )
    main_body.visual(
        Box((0.034, 0.088, 0.070)),
        origin=Origin(xyz=(0.036, 0.0, 0.080)),
        material=accent,
        name="socket_back",
    )
    main_body.visual(
        Box((0.042, 0.024, 0.070)),
        origin=Origin(xyz=(0.058, 0.037, 0.080)),
        material=accent,
        name="socket_cheek_left",
    )
    main_body.visual(
        Box((0.042, 0.024, 0.070)),
        origin=Origin(xyz=(0.058, -0.037, 0.080)),
        material=accent,
        name="socket_cheek_right",
    )
    main_body.inertial = Inertial.from_geometry(
        Box((0.34, 0.16, 0.54)),
        mass=3.4,
        origin=Origin(xyz=(0.00, 0.0, 0.22)),
    )

    upper_wand = model.part("upper_wand")
    upper_wand.visual(
        Box((0.29, 0.038, 0.032)),
        origin=Origin(xyz=(0.145, 0.0, 0.0)),
        material=steel,
        name="upper_tube",
    )
    upper_wand.visual(
        Cylinder(radius=0.022, length=0.050),
        origin=Origin(xyz=(0.00, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="upper_elbow_housing",
    )
    upper_wand.visual(
        Box((0.040, 0.055, 0.048)),
        origin=Origin(xyz=(0.290, 0.0, 0.0)),
        material=charcoal,
        name="upper_distal_cuff",
    )
    upper_wand.inertial = Inertial.from_geometry(
        Box((0.32, 0.06, 0.06)),
        mass=0.9,
        origin=Origin(xyz=(0.160, 0.0, 0.0)),
    )

    lower_wand = model.part("lower_wand")
    lower_wand.visual(
        Box((0.27, 0.042, 0.034)),
        origin=Origin(xyz=(0.155, 0.0, 0.0)),
        material=steel,
        name="lower_tube",
    )
    lower_wand.visual(
        Box((0.040, 0.055, 0.050)),
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
        material=charcoal,
        name="lower_elbow_housing",
    )
    lower_wand.visual(
        Box((0.050, 0.030, 0.038)),
        origin=Origin(xyz=(0.252, 0.0, 0.028)),
        material=charcoal,
        name="nozzle_coupler",
    )
    lower_wand.visual(
        Box((0.024, 0.016, 0.050)),
        origin=Origin(xyz=(0.298, 0.022, 0.0)),
        material=charcoal,
        name="nozzle_yoke_left",
    )
    lower_wand.visual(
        Box((0.024, 0.016, 0.050)),
        origin=Origin(xyz=(0.298, -0.022, 0.0)),
        material=charcoal,
        name="nozzle_yoke_right",
    )
    lower_wand.inertial = Inertial.from_geometry(
        Box((0.31, 0.06, 0.06)),
        mass=0.8,
        origin=Origin(xyz=(0.155, 0.0, 0.0)),
    )

    floor_nozzle = model.part("floor_nozzle")
    floor_nozzle.visual(
        mesh_from_geometry(_nozzle_shell_mesh(), "vacuum_floor_nozzle"),
        material=charcoal,
        name="nozzle_shell",
    )
    floor_nozzle.visual(
        Cylinder(radius=0.017, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="hinge_barrel",
    )
    floor_nozzle.visual(
        Box((0.040, 0.028, 0.032)),
        origin=Origin(xyz=(-0.020, 0.0, -0.012)),
        material=graphite,
        name="neck_stem",
    )
    floor_nozzle.visual(
        Box((0.060, 0.036, 0.016)),
        origin=Origin(xyz=(-0.020, 0.0, -0.024)),
        material=graphite,
        name="neck_bridge",
    )
    floor_nozzle.visual(
        Box((0.025, 0.27, 0.012)),
        origin=Origin(xyz=(0.11, 0.0, -0.048)),
        material=accent,
        name="front_bumper",
    )
    floor_nozzle.visual(
        Box((0.080, 0.18, 0.010)),
        origin=Origin(xyz=(0.06, 0.0, -0.074)),
        material=graphite,
        name="intake_strip",
    )
    floor_nozzle.inertial = Inertial.from_geometry(
        Box((0.14, 0.30, 0.10)),
        mass=1.2,
        origin=Origin(xyz=(0.05, 0.0, -0.05)),
    )

    model.articulation(
        "body_to_upper_wand",
        ArticulationType.REVOLUTE,
        parent=main_body,
        child=upper_wand,
        origin=Origin(xyz=(0.095, 0.0, 0.090), rpy=(0.0, UPPER_REST_PITCH, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.6,
            lower=-0.50,
            upper=0.75,
        ),
    )

    model.articulation(
        "upper_to_lower_wand",
        ArticulationType.REVOLUTE,
        parent=upper_wand,
        child=lower_wand,
        origin=Origin(xyz=(0.31, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.8,
            lower=-0.65,
            upper=0.95,
        ),
    )

    model.articulation(
        "lower_wand_to_nozzle",
        ArticulationType.REVOLUTE,
        parent=lower_wand,
        child=floor_nozzle,
        origin=Origin(xyz=(0.31, 0.0, 0.0), rpy=(0.0, -UPPER_REST_PITCH, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=-0.45,
            upper=0.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    main_body = object_model.get_part("main_body")
    lower_wand = object_model.get_part("lower_wand")
    floor_nozzle = object_model.get_part("floor_nozzle")
    body_elbow = object_model.get_articulation("body_to_upper_wand")
    mid_elbow = object_model.get_articulation("upper_to_lower_wand")
    nozzle_hinge = object_model.get_articulation("lower_wand_to_nozzle")

    body_pos = ctx.part_world_position(main_body)
    nozzle_rest = ctx.part_world_position(floor_nozzle)
    lower_rest = ctx.part_world_position(lower_wand)
    ctx.check(
        "wand and nozzle hang below the hand unit at rest",
        body_pos is not None
        and lower_rest is not None
        and nozzle_rest is not None
        and lower_rest[2] < body_pos[2] - 0.15
        and nozzle_rest[2] < lower_rest[2] - 0.20,
        details=f"body={body_pos}, lower={lower_rest}, nozzle={nozzle_rest}",
    )

    with ctx.pose({mid_elbow: 0.75}):
        mid_fold_nozzle = ctx.part_world_position(floor_nozzle)
    ctx.check(
        "mid wand elbow folds the nozzle back toward the body",
        nozzle_rest is not None
        and mid_fold_nozzle is not None
        and mid_fold_nozzle[0] < nozzle_rest[0] - 0.15,
        details=f"rest={nozzle_rest}, folded={mid_fold_nozzle}",
    )

    with ctx.pose({body_elbow: 0.50}):
        upper_fold_nozzle = ctx.part_world_position(floor_nozzle)
    ctx.check(
        "upper elbow also repositions the wand chain",
        nozzle_rest is not None
        and upper_fold_nozzle is not None
        and upper_fold_nozzle[0] < nozzle_rest[0] - 0.15,
        details=f"rest={nozzle_rest}, posed={upper_fold_nozzle}",
    )

    with ctx.pose({nozzle_hinge: 0.35}):
        bumper_pitch_a = ctx.part_element_world_aabb(floor_nozzle, elem="front_bumper")
    with ctx.pose({nozzle_hinge: -0.35}):
        bumper_pitch_b = ctx.part_element_world_aabb(floor_nozzle, elem="front_bumper")
    ctx.check(
        "floor nozzle pitches through a useful cleaning arc",
        bumper_pitch_a is not None
        and bumper_pitch_b is not None
        and abs(bumper_pitch_a[1][2] - bumper_pitch_b[1][2]) > 0.05,
        details=f"pitch_a={bumper_pitch_a}, pitch_b={bumper_pitch_b}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
