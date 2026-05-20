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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _x_cylinder_origin(xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0))


def _y_cylinder_origin(xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ceiling_exam_lamp")

    warm_white = model.material("warm_white", rgba=(0.91, 0.92, 0.90, 1.0))
    satin_white = model.material("satin_white", rgba=(0.96, 0.96, 0.94, 1.0))
    joint_gray = model.material("joint_gray", rgba=(0.56, 0.58, 0.61, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.24, 0.25, 0.27, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.93, 0.96, 0.99, 0.45))

    upper_arm_length = 0.54
    forearm_length = 0.46

    ceiling_mount = model.part("ceiling_mount")
    ceiling_mount.visual(
        Cylinder(radius=0.125, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=warm_white,
        name="ceiling_plate",
    )
    ceiling_mount.visual(
        Cylinder(radius=0.038, length=0.054),
        origin=Origin(xyz=(0.0, 0.0, -0.043)),
        material=warm_white,
        name="drop_neck",
    )
    ceiling_mount.visual(
        Box((0.050, 0.082, 0.018)),
        origin=Origin(xyz=(-0.026, 0.0, -0.077)),
        material=joint_gray,
        name="shoulder_bridge",
    )
    ceiling_mount.visual(
        Box((0.030, 0.012, 0.056)),
        origin=Origin(xyz=(-0.032, 0.029, -0.102)),
        material=joint_gray,
        name="left_shoulder_cheek",
    )
    ceiling_mount.visual(
        Box((0.030, 0.012, 0.056)),
        origin=Origin(xyz=(-0.032, -0.029, -0.102)),
        material=joint_gray,
        name="right_shoulder_cheek",
    )
    ceiling_mount.visual(
        Cylinder(radius=0.018, length=0.082),
        origin=_y_cylinder_origin((-0.018, 0.0, -0.120)),
        material=joint_gray,
        name="shoulder_barrel",
    )
    ceiling_mount.inertial = Inertial.from_geometry(
        Box((0.26, 0.26, 0.13)),
        mass=6.5,
        origin=Origin(xyz=(0.0, 0.0, -0.065)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Box((0.048, 0.072, 0.032)),
        origin=Origin(xyz=(0.024, 0.0, 0.0)),
        material=joint_gray,
        name="shoulder_knuckle",
    )
    upper_arm.visual(
        Box((0.480, 0.050, 0.036)),
        origin=Origin(xyz=(0.272, 0.0, 0.0)),
        material=warm_white,
        name="upper_arm_body",
    )
    upper_arm.visual(
        Box((0.340, 0.030, 0.018)),
        origin=Origin(xyz=(0.242, 0.0, 0.020)),
        material=satin_white,
        name="upper_arm_cover",
    )
    upper_arm.visual(
        Cylinder(radius=0.018, length=0.090),
        origin=_y_cylinder_origin((upper_arm_length - 0.018, 0.0, 0.0)),
        material=joint_gray,
        name="elbow_barrel",
    )
    upper_arm.visual(
        Box((0.062, 0.076, 0.028)),
        origin=Origin(xyz=(upper_arm_length - 0.032, 0.0, 0.0)),
        material=joint_gray,
        name="elbow_housing",
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((upper_arm_length, 0.09, 0.07)),
        mass=4.2,
        origin=Origin(xyz=(upper_arm_length * 0.5, 0.0, 0.0)),
    )

    forearm = model.part("forearm")
    forearm.visual(
        Box((0.046, 0.064, 0.030)),
        origin=Origin(xyz=(0.023, 0.0, 0.0)),
        material=joint_gray,
        name="elbow_knuckle",
    )
    forearm.visual(
        Box((0.400, 0.046, 0.032)),
        origin=Origin(xyz=(0.230, 0.0, 0.0)),
        material=warm_white,
        name="forearm_body",
    )
    forearm.visual(
        Box((0.290, 0.028, 0.016)),
        origin=Origin(xyz=(0.218, 0.0, 0.018)),
        material=satin_white,
        name="forearm_cover",
    )
    forearm.visual(
        Box((0.060, 0.074, 0.040)),
        origin=Origin(xyz=(forearm_length - 0.040, 0.0, 0.0)),
        material=joint_gray,
        name="wrist_housing",
    )
    forearm.visual(
        Cylinder(radius=0.020, length=0.050),
        origin=Origin(xyz=(forearm_length - 0.020, 0.0, 0.0)),
        material=joint_gray,
        name="wrist_barrel",
    )
    forearm.inertial = Inertial.from_geometry(
        Box((forearm_length, 0.08, 0.07)),
        mass=3.6,
        origin=Origin(xyz=(forearm_length * 0.5, 0.0, 0.0)),
    )

    pan_yoke = model.part("pan_yoke")
    pan_yoke.visual(
        Box((0.032, 0.072, 0.030)),
        origin=Origin(xyz=(0.016, 0.0, -0.002)),
        material=joint_gray,
        name="pan_knuckle",
    )
    pan_yoke.visual(
        Box((0.020, 0.040, 0.070)),
        origin=Origin(xyz=(0.010, 0.0, -0.035)),
        material=joint_gray,
        name="drop_stem",
    )
    pan_yoke.visual(
        Box((0.070, 0.205, 0.014)),
        origin=Origin(xyz=(0.035, 0.0, -0.020)),
        material=joint_gray,
        name="yoke_bridge",
    )
    pan_yoke.visual(
        Box((0.018, 0.014, 0.090)),
        origin=Origin(xyz=(0.060, 0.095, -0.065)),
        material=joint_gray,
        name="left_yoke_arm",
    )
    pan_yoke.visual(
        Box((0.018, 0.014, 0.090)),
        origin=Origin(xyz=(0.060, -0.095, -0.065)),
        material=joint_gray,
        name="right_yoke_arm",
    )
    pan_yoke.inertial = Inertial.from_geometry(
        Box((0.10, 0.22, 0.13)),
        mass=1.8,
        origin=Origin(xyz=(0.045, 0.0, -0.055)),
    )

    lamp_head = model.part("lamp_head")
    lamp_head.visual(
        Cylinder(radius=0.080, length=0.060),
        origin=_x_cylinder_origin((0.0, 0.0, 0.0)),
        material=warm_white,
        name="head_shell",
    )
    lamp_head.visual(
        Cylinder(radius=0.086, length=0.012),
        origin=_x_cylinder_origin((0.029, 0.0, 0.0)),
        material=satin_white,
        name="front_bezel",
    )
    lamp_head.visual(
        Cylinder(radius=0.072, length=0.006),
        origin=_x_cylinder_origin((0.032, 0.0, 0.0)),
        material=lens_glass,
        name="front_lens",
    )
    lamp_head.visual(
        Cylinder(radius=0.056, length=0.024),
        origin=_x_cylinder_origin((-0.028, 0.0, 0.0)),
        material=joint_gray,
        name="rear_housing",
    )
    lamp_head.visual(
        Cylinder(radius=0.014, length=0.028),
        origin=_y_cylinder_origin((0.0, 0.074, 0.0)),
        material=joint_gray,
        name="left_trunnion",
    )
    lamp_head.visual(
        Cylinder(radius=0.014, length=0.028),
        origin=_y_cylinder_origin((0.0, -0.074, 0.0)),
        material=joint_gray,
        name="right_trunnion",
    )
    lamp_head.visual(
        Cylinder(radius=0.010, length=0.048),
        origin=_x_cylinder_origin((0.056, 0.0, 0.0)),
        material=dark_trim,
        name="exam_handle",
    )
    lamp_head.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=_x_cylinder_origin((0.034, 0.0, 0.0)),
        material=dark_trim,
        name="handle_mount",
    )
    lamp_head.inertial = Inertial.from_geometry(
        Box((0.15, 0.18, 0.18)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    model.articulation(
        "shoulder_pitch",
        ArticulationType.REVOLUTE,
        parent=ceiling_mount,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, -0.120)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=24.0,
            velocity=1.4,
            lower=-1.05,
            upper=0.80,
        ),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(upper_arm_length, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.6,
            lower=-1.20,
            upper=1.10,
        ),
    )
    model.articulation(
        "head_pan",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=pan_yoke,
        origin=Origin(xyz=(forearm_length, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=-1.60,
            upper=1.60,
        ),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=pan_yoke,
        child=lamp_head,
        origin=Origin(xyz=(0.060, 0.0, -0.110)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=-0.95,
            upper=0.85,
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

    ceiling_mount = object_model.get_part("ceiling_mount")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    pan_yoke = object_model.get_part("pan_yoke")
    lamp_head = object_model.get_part("lamp_head")

    shoulder_pitch = object_model.get_articulation("shoulder_pitch")
    elbow_pitch = object_model.get_articulation("elbow_pitch")
    head_pan = object_model.get_articulation("head_pan")
    head_tilt = object_model.get_articulation("head_tilt")

    ctx.expect_contact(
        ceiling_mount,
        upper_arm,
        elem_a="shoulder_barrel",
        elem_b="shoulder_knuckle",
        name="upper arm seats against the shoulder barrel",
    )
    ctx.expect_contact(
        upper_arm,
        forearm,
        elem_a="elbow_barrel",
        elem_b="elbow_knuckle",
        name="forearm seats against the elbow barrel",
    )
    ctx.expect_contact(
        forearm,
        pan_yoke,
        elem_a="wrist_barrel",
        elem_b="pan_knuckle",
        name="yoke mounts at the pan joint housing",
    )
    ctx.expect_contact(
        pan_yoke,
        lamp_head,
        elem_a="left_yoke_arm",
        elem_b="left_trunnion",
        name="left trunnion is supported by the yoke",
    )
    ctx.expect_contact(
        pan_yoke,
        lamp_head,
        elem_a="right_yoke_arm",
        elem_b="right_trunnion",
        name="right trunnion is supported by the yoke",
    )

    ctx.expect_origin_gap(
        ceiling_mount,
        lamp_head,
        axis="z",
        min_gap=0.18,
        name="lamp head hangs below the ceiling plate",
    )
    ctx.expect_origin_gap(
        lamp_head,
        ceiling_mount,
        axis="x",
        min_gap=0.95,
        name="lamp head reaches out from the ceiling mount",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    rest_head_pos = ctx.part_world_position(lamp_head)
    with ctx.pose({shoulder_pitch: 0.55}):
        raised_by_shoulder = ctx.part_world_position(lamp_head)
    ctx.check(
        "shoulder pitch raises the lamp head",
        rest_head_pos is not None
        and raised_by_shoulder is not None
        and raised_by_shoulder[2] > rest_head_pos[2] + 0.15,
        details=f"rest={rest_head_pos}, raised={raised_by_shoulder}",
    )

    with ctx.pose({elbow_pitch: 0.70}):
        raised_by_elbow = ctx.part_world_position(lamp_head)
    ctx.check(
        "elbow pitch lifts the distal assembly",
        rest_head_pos is not None
        and raised_by_elbow is not None
        and raised_by_elbow[2] > rest_head_pos[2] + 0.08,
        details=f"rest={rest_head_pos}, raised={raised_by_elbow}",
    )

    with ctx.pose({head_pan: 0.80}):
        panned_head_pos = ctx.part_world_position(lamp_head)
    ctx.check(
        "pan joint swings the head sideways",
        rest_head_pos is not None
        and panned_head_pos is not None
        and panned_head_pos[1] > rest_head_pos[1] + 0.03,
        details=f"rest={rest_head_pos}, panned={panned_head_pos}",
    )

    rest_lens_center = _aabb_center(ctx.part_element_world_aabb(lamp_head, elem="front_lens"))
    with ctx.pose({head_tilt: 0.55}):
        tilted_lens_center = _aabb_center(ctx.part_element_world_aabb(lamp_head, elem="front_lens"))
    ctx.check(
        "tilt joint aims the lamp downward",
        rest_lens_center is not None
        and tilted_lens_center is not None
        and tilted_lens_center[2] < rest_lens_center[2] - 0.012,
        details=f"rest={rest_lens_center}, tilted={tilted_lens_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
