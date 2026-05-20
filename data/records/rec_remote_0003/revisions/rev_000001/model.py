from __future__ import annotations

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
    MotionLimits,
    Origin,
    SectionLoftSpec,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    rounded_rect_profile,
    section_loft,
    sweep_profile_along_spline,
)

ASSETS = AssetContext.from_script(__file__)
FACE_Z = 0.0152


def _rounded_rect_loop(width: float, depth: float, radius: float, z: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in rounded_rect_profile(width, depth, radius, corner_segments=10)]


def _save_mesh(geometry, name: str):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _make_shell_mesh(name: str, sections: list[tuple[float, float, float, float]]):
    loft_spec = SectionLoftSpec(
        sections=tuple(_rounded_rect_loop(width, depth, radius, z) for width, depth, radius, z in sections),
        cap=True,
        solid=True,
    )
    return _save_mesh(repair_loft(section_loft(loft_spec)), name)


def _fixed_mount(
    model: ArticulatedObject,
    name: str,
    parent: str,
    child: str,
    xyz: tuple[float, float, float],
) -> None:
    model.articulation(
        name,
        ArticulationType.FIXED,
        parent=parent,
        child=child,
        origin=Origin(xyz=xyz),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_gaming_controller", assets=ASSETS)

    face_grey = model.material("face_grey", rgba=(0.86, 0.87, 0.89, 1.0))
    back_grey = model.material("back_grey", rgba=(0.75, 0.76, 0.78, 1.0))
    trim_grey = model.material("trim_grey", rgba=(0.67, 0.68, 0.70, 1.0))
    charcoal = model.material("charcoal", rgba=(0.19, 0.20, 0.22, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.09, 0.09, 0.10, 1.0))

    top_shell_mesh = _make_shell_mesh(
        "controller_top_shell.obj",
        [
            (0.154, 0.098, 0.022, -0.0006),
            (0.151, 0.095, 0.021, 0.0032),
            (0.147, 0.091, 0.019, 0.0075),
            (0.141, 0.086, 0.017, 0.0122),
            (0.136, 0.082, 0.015, 0.0156),
        ],
    )
    back_shell_mesh = _make_shell_mesh(
        "controller_back_shell.obj",
        [
            (0.152, 0.097, 0.022, 0.0008),
            (0.149, 0.095, 0.021, -0.0046),
            (0.145, 0.092, 0.019, -0.0108),
            (0.139, 0.086, 0.017, -0.0172),
            (0.131, 0.079, 0.014, -0.0218),
        ],
    )
    bumper_mesh = _save_mesh(
        ExtrudeGeometry(
            rounded_rect_profile(0.034, 0.014, radius=0.0045, corner_segments=8),
            height=0.009,
            cap=True,
            center=True,
            closed=True,
        ),
        "controller_bumper.obj",
    )
    trigger_mesh = _save_mesh(
        sweep_profile_along_spline(
            [
                (0.0, 0.0, 0.0),
                (0.0, 0.010, -0.002),
                (0.0, 0.018, -0.008),
                (0.0, 0.022, -0.017),
            ],
            profile=rounded_rect_profile(0.029, 0.008, radius=0.0025, corner_segments=6),
            samples_per_segment=16,
            cap_profile=True,
            up_hint=(1.0, 0.0, 0.0),
        ),
        "controller_trigger.obj",
    )

    housing = model.part("housing")
    housing.visual(top_shell_mesh, material=face_grey)
    housing.visual(back_shell_mesh, material=back_grey)
    housing.inertial = Inertial.from_geometry(
        Box((0.154, 0.098, 0.038)),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
    )

    dpad = model.part("dpad")
    dpad.visual(Box((0.024, 0.009, 0.0032)), origin=Origin(xyz=(0.0, 0.0, 0.0009)), material=charcoal)
    dpad.visual(Box((0.009, 0.024, 0.0032)), origin=Origin(xyz=(0.0, 0.0, 0.0009)), material=charcoal)
    dpad.visual(Cylinder(radius=0.0053, length=0.0024), origin=Origin(xyz=(0.0, 0.0, 0.0014)), material=charcoal)
    dpad.inertial = Inertial.from_geometry(
        Box((0.024, 0.024, 0.0032)),
        mass=0.012,
        origin=Origin(xyz=(0.0, 0.0, 0.0009)),
    )

    for name, x, y in (
        ("face_button_n", 0.050, 0.0195),
        ("face_button_e", 0.061, 0.0085),
        ("face_button_s", 0.050, -0.0025),
        ("face_button_w", 0.039, 0.0085),
    ):
        button = model.part(name)
        button.visual(
            Cylinder(radius=0.0054, length=0.0030),
            origin=Origin(xyz=(0.0, 0.0, 0.0010)),
            material=charcoal,
        )
        button.inertial = Inertial.from_geometry(
            Cylinder(radius=0.0054, length=0.0030),
            mass=0.004,
            origin=Origin(xyz=(0.0, 0.0, 0.0010)),
        )
        _fixed_mount(model, f"housing_to_{name}", "housing", name, (x, y, FACE_Z))

    for side, x in (("left", -0.018), ("right", 0.018)):
        collar = model.part(f"{side}_stick_collar")
        collar.visual(
            Cylinder(radius=0.0103, length=0.0028),
            origin=Origin(xyz=(0.0, 0.0, 0.0007)),
            material=charcoal,
        )
        collar.visual(
            Cylinder(radius=0.0078, length=0.0016),
            origin=Origin(xyz=(0.0, 0.0, 0.0012)),
            material=trim_grey,
        )
        collar.inertial = Inertial.from_geometry(
            Cylinder(radius=0.0103, length=0.0028),
            mass=0.006,
            origin=Origin(xyz=(0.0, 0.0, 0.0007)),
        )
        _fixed_mount(model, f"housing_to_{side}_stick_collar", "housing", f"{side}_stick_collar", (x, -0.008, FACE_Z + 0.0002))

        pitch = model.part(f"{side}_stick_pitch")
        pitch.visual(Box((0.0082, 0.0018, 0.0016)), origin=Origin(xyz=(0.0, 0.0, -0.0011)), material=trim_grey)
        pitch.visual(Box((0.0014, 0.0066, 0.0042)), origin=Origin(xyz=(-0.0048, 0.0, 0.0002)), material=trim_grey)
        pitch.visual(Box((0.0014, 0.0066, 0.0042)), origin=Origin(xyz=(0.0048, 0.0, 0.0002)), material=trim_grey)
        pitch.inertial = Inertial.from_geometry(
            Box((0.0105, 0.0070, 0.0042)),
            mass=0.005,
            origin=Origin(xyz=(0.0, 0.0, 0.0002)),
        )

        roll = model.part(f"{side}_stick_roll")
        roll.visual(Cylinder(radius=0.0036, length=0.0060), origin=Origin(xyz=(0.0, 0.0, 0.0032)), material=trim_grey)
        roll.visual(Cylinder(radius=0.0090, length=0.0040), origin=Origin(xyz=(0.0, 0.0, 0.0082)), material=rubber_black)
        roll.visual(Cylinder(radius=0.0104, length=0.0012), origin=Origin(xyz=(0.0, 0.0, 0.0106)), material=rubber_black)
        roll.inertial = Inertial.from_geometry(
            Cylinder(radius=0.0090, length=0.0100),
            mass=0.009,
            origin=Origin(xyz=(0.0, 0.0, 0.0065)),
        )

        model.articulation(
            f"{side}_stick_pitch_joint",
            ArticulationType.REVOLUTE,
            parent="housing",
            child=f"{side}_stick_pitch",
            origin=Origin(xyz=(x, -0.008, FACE_Z + 0.0020)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.8, velocity=4.0, lower=-0.34, upper=0.34),
        )
        model.articulation(
            f"{side}_stick_roll_joint",
            ArticulationType.REVOLUTE,
            parent=f"{side}_stick_pitch",
            child=f"{side}_stick_roll",
            origin=Origin(),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.8, velocity=4.0, lower=-0.34, upper=0.34),
        )

    left_bumper = model.part("left_bumper")
    left_bumper.visual(bumper_mesh, origin=Origin(xyz=(0.0, -0.0105, -0.0036)), material=trim_grey)
    left_bumper.inertial = Inertial.from_geometry(
        Box((0.034, 0.014, 0.009)),
        mass=0.010,
        origin=Origin(xyz=(0.0, -0.0105, -0.0036)),
    )
    right_bumper = model.part("right_bumper")
    right_bumper.visual(bumper_mesh, origin=Origin(xyz=(0.0, -0.0105, -0.0036)), material=trim_grey)
    right_bumper.inertial = Inertial.from_geometry(
        Box((0.034, 0.014, 0.009)),
        mass=0.010,
        origin=Origin(xyz=(0.0, -0.0105, -0.0036)),
    )

    left_trigger = model.part("left_trigger")
    left_trigger.visual(trigger_mesh, material=charcoal)
    left_trigger.inertial = Inertial.from_geometry(
        Box((0.029, 0.024, 0.018)),
        mass=0.012,
        origin=Origin(xyz=(0.0, 0.010, -0.008)),
    )
    right_trigger = model.part("right_trigger")
    right_trigger.visual(trigger_mesh, material=charcoal)
    right_trigger.inertial = Inertial.from_geometry(
        Box((0.029, 0.024, 0.018)),
        mass=0.012,
        origin=Origin(xyz=(0.0, 0.010, -0.008)),
    )

    model.articulation(
        "left_bumper_hinge",
        ArticulationType.REVOLUTE,
        parent="housing",
        child="left_bumper",
        origin=Origin(xyz=(-0.039, 0.040, FACE_Z + 0.0045)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=3.5, lower=0.0, upper=0.42),
    )
    model.articulation(
        "right_bumper_hinge",
        ArticulationType.REVOLUTE,
        parent="housing",
        child="right_bumper",
        origin=Origin(xyz=(0.039, 0.040, FACE_Z + 0.0045)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=3.5, lower=0.0, upper=0.42),
    )
    model.articulation(
        "left_trigger_hinge",
        ArticulationType.REVOLUTE,
        parent="housing",
        child="left_trigger",
        origin=Origin(xyz=(-0.039, 0.048, FACE_Z + 0.0026)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=3.5, lower=0.0, upper=0.68),
    )
    model.articulation(
        "right_trigger_hinge",
        ArticulationType.REVOLUTE,
        parent="housing",
        child="right_trigger",
        origin=Origin(xyz=(0.039, 0.048, FACE_Z + 0.0026)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=3.5, lower=0.0, upper=0.68),
    )

    _fixed_mount(model, "housing_to_dpad", "housing", "dpad", (-0.050, 0.0085, FACE_Z + 0.0002))
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root, geometry_source="collision")
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_part_geometry_disconnected(use="visual")
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.allow_overlap("housing", "left_bumper", reason="bumper root nests slightly into the top shell at the hinge line")
    ctx.allow_overlap("housing", "right_bumper", reason="bumper root nests slightly into the top shell at the hinge line")
    ctx.allow_overlap("housing", "left_trigger", reason="trigger root seats into the rear shoulder cut line")
    ctx.allow_overlap("housing", "right_trigger", reason="trigger root seats into the rear shoulder cut line")
    ctx.allow_overlap("left_bumper", "left_trigger", reason="shoulder bumper and trigger use tight nested envelopes near the shared left shoulder")
    ctx.allow_overlap("right_bumper", "right_trigger", reason="shoulder bumper and trigger use tight nested envelopes near the shared right shoulder")
    ctx.check_articulation_overlaps(max_pose_samples=128, overlap_tol=0.003, overlap_volume_tol=0.0)
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_overlaps(max_pose_samples=128, overlap_tol=0.003, overlap_volume_tol=0.0, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_aabb_overlap("dpad", "housing", axes="xy", min_overlap=0.016)
    ctx.expect_aabb_gap("dpad", "housing", axis="z", max_gap=0.0015, max_penetration=0.0012)

    for name in ("face_button_n", "face_button_e", "face_button_s", "face_button_w"):
        ctx.expect_aabb_overlap(name, "housing", axes="xy", min_overlap=0.007)
        ctx.expect_aabb_gap(name, "housing", axis="z", max_gap=0.0015, max_penetration=0.0010)

    ctx.expect_origin_distance("face_button_n", "face_button_s", axes="x", max_dist=0.001)
    ctx.expect_origin_distance("face_button_e", "face_button_w", axes="y", max_dist=0.001)

    for side in ("left", "right"):
        ctx.expect_aabb_overlap(f"{side}_stick_collar", "housing", axes="xy", min_overlap=0.014)
        ctx.expect_aabb_gap(f"{side}_stick_collar", "housing", axis="z", max_gap=0.0015, max_penetration=0.0012)
        ctx.expect_origin_distance(f"{side}_stick_collar", f"{side}_stick_pitch", axes="xy", max_dist=0.001)
        ctx.expect_joint_motion_axis(f"{side}_stick_pitch_joint", f"{side}_stick_roll", world_axis="y", direction="positive", min_delta=0.0012)
        ctx.expect_joint_motion_axis(f"{side}_stick_roll_joint", f"{side}_stick_roll", world_axis="x", direction="positive", min_delta=0.0012)

    ctx.expect_joint_motion_axis("left_bumper_hinge", "left_bumper", world_axis="z", direction="negative", min_delta=0.0025)
    ctx.expect_joint_motion_axis("right_bumper_hinge", "right_bumper", world_axis="z", direction="negative", min_delta=0.0025)
    ctx.expect_joint_motion_axis("left_trigger_hinge", "left_trigger", world_axis="z", direction="negative", min_delta=0.0040)
    ctx.expect_joint_motion_axis("right_trigger_hinge", "right_trigger", world_axis="z", direction="negative", min_delta=0.0040)

    dpad_pos = ctx.part_world_position("dpad")
    left_stick_pos = ctx.part_world_position("left_stick_roll")
    right_stick_pos = ctx.part_world_position("right_stick_roll")
    button_n_pos = ctx.part_world_position("face_button_n")
    button_e_pos = ctx.part_world_position("face_button_e")
    button_s_pos = ctx.part_world_position("face_button_s")
    button_w_pos = ctx.part_world_position("face_button_w")
    left_bumper_pos = ctx.part_world_position("left_bumper")
    left_trigger_pos = ctx.part_world_position("left_trigger")

    cluster_center_x = 0.25 * (button_n_pos[0] + button_e_pos[0] + button_s_pos[0] + button_w_pos[0])
    cluster_center_y = 0.25 * (button_n_pos[1] + button_e_pos[1] + button_s_pos[1] + button_w_pos[1])
    if abs((button_e_pos[0] - button_w_pos[0]) - 0.022) > 0.0015:
        raise AssertionError("Face button east-west spacing drifted from the intended compact diamond cluster.")
    if abs((button_n_pos[1] - button_s_pos[1]) - 0.022) > 0.0015:
        raise AssertionError("Face button north-south spacing drifted from the intended compact diamond cluster.")
    if abs(cluster_center_x - 0.05) > 0.0015 or abs(cluster_center_y - 0.0085) > 0.0015:
        raise AssertionError("Face button cluster center moved off the right-hand control field.")
    if not (dpad_pos[0] < left_stick_pos[0] < right_stick_pos[0] < cluster_center_x):
        raise AssertionError("Controls no longer read left-to-right as D-pad, twin thumbsticks, then face buttons.")
    if abs(0.5 * (left_stick_pos[0] + right_stick_pos[0])) > 0.001:
        raise AssertionError("Twin thumbsticks should stay centered on the controller face.")
    if abs(left_stick_pos[1] - dpad_pos[1]) > 0.02 or abs(right_stick_pos[1] - cluster_center_y) > 0.02:
        raise AssertionError("Thumbsticks should remain in the central control band between D-pad and face buttons.")
    if not (left_trigger_pos[1] > left_bumper_pos[1] and left_trigger_pos[2] < left_bumper_pos[2]):
        raise AssertionError("Trigger should sit behind and below the bumper along the top rear shoulder.")

    with ctx.pose(left_stick_pitch_joint=0.28, left_stick_roll_joint=0.24):
        ctx.expect_aabb_overlap("left_stick_roll", "left_stick_collar", axes="xy", min_overlap=0.008)
    with ctx.pose(right_stick_pitch_joint=-0.28, right_stick_roll_joint=-0.24):
        ctx.expect_aabb_overlap("right_stick_roll", "right_stick_collar", axes="xy", min_overlap=0.008)
    with ctx.pose(left_bumper_hinge=0.35, right_bumper_hinge=0.35):
        ctx.expect_aabb_overlap("left_bumper", "housing", axes="x", min_overlap=0.020)
        ctx.expect_aabb_overlap("right_bumper", "housing", axes="x", min_overlap=0.020)
    with ctx.pose(left_trigger_hinge=0.55, right_trigger_hinge=0.55):
        ctx.expect_aabb_overlap("left_trigger", "housing", axes="x", min_overlap=0.0065)
        ctx.expect_aabb_overlap("right_trigger", "housing", axes="x", min_overlap=0.0065)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
