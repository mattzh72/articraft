from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root

BARREL_OUTER_RADIUS = 0.0105
BARREL_INNER_RADIUS = 0.0089
BARREL_REAR_OPENING_Z = 0.0
BARREL_INNER_FRONT_Z = 0.110
BARREL_TIP_END_Z = 0.1365

PLUNGER_STOPPER_CENTER_Z = 0.040
PLUNGER_STOPPER_LENGTH = 0.009
PLUNGER_THUMB_CENTER_Z = -0.076
PLUNGER_THUMB_LENGTH = 0.0036

PLUNGER_LOWER_LIMIT = -0.028
PLUNGER_UPPER_LIMIT = 0.032


def _material(name: str, rgba: tuple[float, float, float, float]) -> Material:
    try:
        return Material(name=name, color=rgba)
    except TypeError:
        return Material(name=name, rgba=rgba)


def _mesh_path(name: str):
    try:
        return ASSETS.mesh_path(name)
    except AttributeError:
        return ASSETS.mesh_dir / name


def _build_barrel_shell_mesh():
    profile = [
        (0.0130, 0.0000),
        (0.0130, 0.0040),
        (BARREL_OUTER_RADIUS, 0.0060),
        (BARREL_OUTER_RADIUS, 0.1120),
        (0.0064, 0.1180),
        (0.0026, 0.1310),
        (0.0012, 0.1360),
        (0.0009, BARREL_TIP_END_Z),
        (0.0003, BARREL_TIP_END_Z),
        (0.0008, 0.1290),
        (0.0014, 0.1260),
        (0.0022, 0.1180),
        (BARREL_INNER_RADIUS, BARREL_INNER_FRONT_Z),
        (BARREL_INNER_RADIUS, BARREL_REAR_OPENING_Z),
    ]
    geom = LatheGeometry(profile, segments=72)
    return mesh_from_geometry(geom, _mesh_path("syringe_barrel_shell.obj"))


def _build_finger_grip_mesh():
    profile = rounded_rect_profile(0.033, 0.012, radius=0.0032, corner_segments=10)
    geom = ExtrudeGeometry(profile, height=0.0036, cap=True, center=True, closed=True)
    return mesh_from_geometry(geom, _mesh_path("syringe_finger_grip.obj"))


def _add_barrel_markings(part, material: Material) -> None:
    mark_zs = [
        0.020,
        0.026,
        0.032,
        0.038,
        0.044,
        0.050,
        0.056,
        0.062,
        0.068,
        0.074,
        0.080,
        0.086,
        0.092,
    ]
    for index, z in enumerate(mark_zs, start=1):
        width = 0.0090 if index in {2, 5, 8, 11, 13} else 0.0056
        part.visual(
            Box((width, 0.00045, 0.0007)),
            origin=Origin(xyz=(0.0, BARREL_OUTER_RADIUS - 0.00022, z)),
            material=material,
            name=f"graduation_{index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="medical_syringe", assets=ASSETS)

    clear_polycarbonate = _material("clear_polycarbonate", (0.88, 0.94, 0.98, 0.28))
    measurement_ink = _material("measurement_ink", (0.18, 0.21, 0.24, 0.92))
    white_polypropylene = _material("white_polypropylene", (0.95, 0.96, 0.98, 1.0))
    black_rubber = _material("black_rubber", (0.08, 0.09, 0.10, 1.0))
    blue_tint = _material("blue_hub_tint", (0.42, 0.71, 0.94, 0.75))
    brushed_steel = _material("brushed_steel", (0.76, 0.78, 0.82, 1.0))
    model.materials.extend(
        [
            clear_polycarbonate,
            measurement_ink,
            white_polypropylene,
            black_rubber,
            blue_tint,
            brushed_steel,
        ]
    )

    barrel_shell = _build_barrel_shell_mesh()
    finger_grip = _build_finger_grip_mesh()

    barrel = model.part("barrel")
    barrel.visual(barrel_shell, material=clear_polycarbonate, name="barrel_shell")
    barrel.visual(
        finger_grip,
        origin=Origin(xyz=(0.022, 0.0, 0.0032)),
        material=clear_polycarbonate,
        name="right_finger_grip",
    )
    barrel.visual(
        finger_grip,
        origin=Origin(xyz=(-0.022, 0.0, 0.0032)),
        material=clear_polycarbonate,
        name="left_finger_grip",
    )
    barrel.visual(
        Cylinder(radius=0.0057, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.116)),
        material=blue_tint,
        name="luer_collar",
    )
    barrel.visual(
        Cylinder(radius=0.00045, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.148)),
        material=brushed_steel,
        name="needle",
    )
    _add_barrel_markings(barrel, measurement_ink)
    barrel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0135, length=0.155),
        mass=0.040,
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
    )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.0033, length=0.108),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material=white_polypropylene,
        name="plunger_rod",
    )
    plunger.visual(
        Box((0.0075, 0.0026, 0.058)),
        origin=Origin(xyz=(0.0, 0.0, -0.039)),
        material=white_polypropylene,
        name="rod_rib_x",
    )
    plunger.visual(
        Box((0.0026, 0.0075, 0.058)),
        origin=Origin(xyz=(0.0, 0.0, -0.039)),
        material=white_polypropylene,
        name="rod_rib_y",
    )
    plunger.visual(
        Cylinder(radius=0.0175, length=PLUNGER_THUMB_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, PLUNGER_THUMB_CENTER_Z)),
        material=white_polypropylene,
        name="thumb_pad",
    )
    plunger.visual(
        Cylinder(radius=0.0135, length=0.0032),
        origin=Origin(xyz=(0.0, 0.0, PLUNGER_THUMB_CENTER_Z + 0.0030)),
        material=white_polypropylene,
        name="thumb_pad_step",
    )
    plunger.visual(
        Cylinder(radius=0.0049, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        material=white_polypropylene,
        name="stopper_boss",
    )
    plunger.visual(
        Cylinder(radius=0.00825, length=PLUNGER_STOPPER_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, PLUNGER_STOPPER_CENTER_Z)),
        material=black_rubber,
        name="main_stopper",
    )
    plunger.visual(
        Cylinder(radius=0.0087, length=0.0014),
        origin=Origin(xyz=(0.0, 0.0, 0.0362)),
        material=black_rubber,
        name="stopper_lip_rear",
    )
    plunger.visual(
        Cylinder(radius=0.0087, length=0.0014),
        origin=Origin(xyz=(0.0, 0.0, 0.0438)),
        material=black_rubber,
        name="stopper_lip_front",
    )
    plunger.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0175, length=0.122),
        mass=0.018,
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
    )

    model.articulation(
        "plunger_slide",
        ArticulationType.PRISMATIC,
        parent="barrel",
        child="plunger",
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.20,
            lower=PLUNGER_LOWER_LIMIT,
            upper=PLUNGER_UPPER_LIMIT,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap(
        "barrel",
        "plunger",
        reason="The rubber stopper intentionally rides inside the hollow barrel, and generated collision hulls can conservatively fill the transparent shell volume.",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=192, overlap_tol=0.0035, overlap_volume_tol=0.0)

    ctx.expect_origin_distance("plunger", "barrel", axes="xy", max_dist=0.0025)
    ctx.expect_aabb_overlap("plunger", "barrel", axes="xy", min_overlap=0.022)
    ctx.expect_joint_motion_axis(
        "plunger_slide",
        "plunger",
        world_axis="z",
        direction="negative",
        min_delta=0.020,
    )

    rest_plunger_z = ctx.part_world_position("plunger")[2]

    with ctx.pose(plunger_slide=PLUNGER_LOWER_LIMIT):
        ctx.expect_origin_distance("plunger", "barrel", axes="xy", max_dist=0.0025)
        ctx.expect_aabb_overlap("plunger", "barrel", axes="xy", min_overlap=0.022)
        pressed_plunger_z = ctx.part_world_position("plunger")[2]

    with ctx.pose(plunger_slide=PLUNGER_UPPER_LIMIT):
        ctx.expect_origin_distance("plunger", "barrel", axes="xy", max_dist=0.0025)
        ctx.expect_aabb_overlap("plunger", "barrel", axes="xy", min_overlap=0.022)
        retracted_plunger_z = ctx.part_world_position("plunger")[2]

    if not (retracted_plunger_z < rest_plunger_z < pressed_plunger_z):
        raise AssertionError(
            "Plunger frame should move rearward when retracted and forward when depressed."
        )

    if pressed_plunger_z - retracted_plunger_z < 0.055:
        raise AssertionError("Plunger stroke should cover a realistic syringe travel distance.")

    pressed_stopper_front = (
        pressed_plunger_z + PLUNGER_STOPPER_CENTER_Z + (PLUNGER_STOPPER_LENGTH * 0.5)
    )
    retracted_stopper_rear = (
        retracted_plunger_z + PLUNGER_STOPPER_CENTER_Z - (PLUNGER_STOPPER_LENGTH * 0.5)
    )
    if pressed_stopper_front >= BARREL_INNER_FRONT_Z:
        raise AssertionError(
            "The stopper should not run past the front taper of the barrel when fully depressed."
        )
    if retracted_stopper_rear <= BARREL_REAR_OPENING_Z:
        raise AssertionError(
            "The stopper should remain captured inside the rear opening when fully retracted."
        )

    pressed_thumb_front = pressed_plunger_z + PLUNGER_THUMB_CENTER_Z + (PLUNGER_THUMB_LENGTH * 0.5)
    if pressed_thumb_front >= -0.010:
        raise AssertionError(
            "The thumb pad should remain clearly behind the finger grips through the full stroke."
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
