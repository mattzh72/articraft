from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


PITCH_AXIS_X = 0.072
PITCH_AXIS_Z = 0.048
ROLL_AXIS_X = 0.060


def _centered_cylinder(axis: str, length: float, radius: float) -> cq.Workplane:
    plane = {"x": "YZ", "y": "XZ", "z": "XY"}[axis]
    return cq.Workplane(plane).circle(radius).extrude(length / 2.0, both=True)


def _make_housing_shape() -> cq.Workplane:
    lower_shell = (
        cq.Workplane("XY")
        .box(0.220, 0.168, 0.058)
        .translate((0.0, 0.0, -0.049))
        .edges("|Z")
        .fillet(0.018)
    )
    upper_shoulder = (
        cq.Workplane("XY")
        .box(0.176, 0.122, 0.028)
        .translate((0.0, 0.0, -0.016))
        .edges("|Z")
        .fillet(0.015)
    )
    bearing_seat = _centered_cylinder("z", 0.016, 0.086).translate((0.0, 0.0, -0.008))
    front_pad = (
        cq.Workplane("XY")
        .box(0.078, 0.090, 0.020)
        .translate((0.042, 0.0, -0.030))
        .edges("|Z")
        .fillet(0.012)
    )
    return lower_shell.union(upper_shoulder).union(bearing_seat).union(front_pad)


def _make_yaw_stage_shape() -> cq.Workplane:
    rotating_platter = _centered_cylinder("z", 0.020, 0.078).translate((0.0, 0.0, 0.010))
    rear_tower = (
        cq.Workplane("XY")
        .box(0.072, 0.052, 0.036)
        .translate((0.010, 0.0, 0.038))
        .edges("|Z")
        .fillet(0.010)
    )
    lower_bridge = (
        cq.Workplane("XY")
        .box(0.028, 0.074, 0.010)
        .translate((0.057, 0.0, 0.023))
        .edges("|Z")
        .fillet(0.004)
    )
    ear_blocks = cq.Workplane("XY")
    for y_center in (0.043, -0.043):
        ear = (
            cq.Workplane("XY")
            .box(0.042, 0.012, 0.038)
            .translate((0.064, y_center, 0.047))
            .edges("|Z")
            .fillet(0.005)
        )
        ear_blocks = ear_blocks.union(ear)
    return rotating_platter.union(rear_tower).union(lower_bridge).union(ear_blocks)


def _make_pitch_support_shape() -> cq.Workplane:
    cross_tube = _centered_cylinder("y", 0.054, 0.013)
    left_trunnion = _centered_cylinder("y", 0.010, 0.018).translate((0.0, 0.032, 0.0))
    right_trunnion = _centered_cylinder("y", 0.010, 0.018).translate((0.0, -0.032, 0.0))

    support = cross_tube.union(left_trunnion).union(right_trunnion)
    for y_center in (0.027, -0.027):
        arm = (
            cq.Workplane("XY")
            .box(0.040, 0.012, 0.046)
            .translate((0.020, y_center, 0.0))
            .edges("|Z")
            .fillet(0.005)
        )
        support = support.union(arm)

    bearing_ring = _centered_cylinder("x", 0.014, 0.026).translate((0.047, 0.0, 0.0))
    bearing_bore = _centered_cylinder("x", 0.018, 0.020).translate((0.047, 0.0, 0.0))
    upper_saddle = (
        cq.Workplane("XY")
        .box(0.022, 0.028, 0.010)
        .translate((0.029, 0.0, 0.021))
        .edges("|Z")
        .fillet(0.003)
    )
    lower_saddle = (
        cq.Workplane("XY")
        .box(0.022, 0.032, 0.010)
        .translate((0.029, 0.0, -0.021))
        .edges("|Z")
        .fillet(0.003)
    )
    return (
        support.union(bearing_ring.cut(bearing_bore))
        .union(upper_saddle)
        .union(lower_saddle)
    )


def _make_roll_cartridge_shape() -> cq.Workplane:
    rear_hub = _centered_cylinder("x", 0.022, 0.018).translate((-0.005, 0.0, 0.0))
    rear_flange = _centered_cylinder("x", 0.012, 0.024)
    main_body = _centered_cylinder("x", 0.050, 0.019).translate((0.031, 0.0, 0.0))
    front_flange = _centered_cylinder("x", 0.010, 0.027).translate((0.061, 0.0, 0.0))
    nose = _centered_cylinder("x", 0.018, 0.013).translate((0.075, 0.0, 0.0))
    return rear_hub.union(rear_flange).union(main_body).union(front_flange).union(nose)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="saddle_body_multi_axis_wrist")

    model.material("housing_paint", rgba=(0.23, 0.24, 0.26, 1.0))
    model.material("stage_graphite", rgba=(0.17, 0.18, 0.20, 1.0))
    model.material("machined_alloy", rgba=(0.70, 0.73, 0.76, 1.0))
    model.material("cartridge_finish", rgba=(0.77, 0.79, 0.81, 1.0))
    model.material("sensor_black", rgba=(0.09, 0.09, 0.10, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_make_housing_shape(), "wrist_housing"),
        material="housing_paint",
        name="housing_shell",
    )

    yaw_stage = model.part("yaw_stage")
    yaw_stage.visual(
        mesh_from_cadquery(_make_yaw_stage_shape(), "wrist_yaw_stage"),
        material="stage_graphite",
        name="yaw_stage_shell",
    )

    pitch_support = model.part("pitch_support")
    pitch_support.visual(
        mesh_from_cadquery(_make_pitch_support_shape(), "wrist_pitch_support"),
        material="machined_alloy",
        name="pitch_support_shell",
    )

    roll_cartridge = model.part("roll_cartridge")
    roll_cartridge.visual(
        mesh_from_cadquery(_make_roll_cartridge_shape(), "wrist_roll_cartridge"),
        material="cartridge_finish",
        name="roll_shell",
    )
    roll_cartridge.visual(
        Box((0.016, 0.014, 0.010)),
        origin=Origin(xyz=(0.028, 0.0, 0.022)),
        material="sensor_black",
        name="encoder_pod",
    )

    model.articulation(
        "base_yaw",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=yaw_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=2.0, lower=-2.35, upper=2.35),
    )
    model.articulation(
        "wrist_pitch",
        ArticulationType.REVOLUTE,
        parent=yaw_stage,
        child=pitch_support,
        origin=Origin(xyz=(PITCH_AXIS_X, 0.0, PITCH_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=24.0, velocity=2.5, lower=-1.10, upper=1.10),
    )
    model.articulation(
        "wrist_roll",
        ArticulationType.REVOLUTE,
        parent=pitch_support,
        child=roll_cartridge,
        origin=Origin(xyz=(ROLL_AXIS_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=16.0, velocity=4.0, lower=-3.05, upper=3.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    yaw_stage = object_model.get_part("yaw_stage")
    pitch_support = object_model.get_part("pitch_support")
    roll_cartridge = object_model.get_part("roll_cartridge")

    yaw_joint = object_model.get_articulation("base_yaw")
    pitch_joint = object_model.get_articulation("wrist_pitch")
    roll_joint = object_model.get_articulation("wrist_roll")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "serial_joint_axes_match_prompt",
        yaw_joint.axis == (0.0, 0.0, 1.0)
        and pitch_joint.axis == (0.0, -1.0, 0.0)
        and roll_joint.axis == (1.0, 0.0, 0.0),
        details=(
            f"got yaw={yaw_joint.axis}, pitch={pitch_joint.axis}, roll={roll_joint.axis}; "
            "expected vertical, crosswise horizontal, then longitudinal axes"
        ),
    )

    with ctx.pose(base_yaw=0.0, wrist_pitch=0.0, wrist_roll=0.0):
        ctx.expect_contact(
            yaw_stage,
            housing,
            name="yaw stage seats on grounded housing",
        )
        ctx.expect_overlap(
            yaw_stage,
            housing,
            axes="xy",
            min_overlap=0.120,
            name="yaw bearing footprint overlaps housing seat",
        )
        ctx.expect_contact(
            pitch_support,
            yaw_stage,
            name="pitch support trunnions contact yaw cradle",
        )
        ctx.expect_origin_gap(
            pitch_support,
            yaw_stage,
            axis="x",
            min_gap=0.068,
            max_gap=0.076,
            name="pitch axis sits forward of yaw axis",
        )
        ctx.expect_origin_gap(
            pitch_support,
            yaw_stage,
            axis="z",
            min_gap=0.044,
            max_gap=0.052,
            name="pitch axis sits above yaw platter",
        )
        ctx.expect_contact(
            roll_cartridge,
            pitch_support,
            name="roll cartridge seats against pitch frame",
        )
        ctx.expect_origin_gap(
            roll_cartridge,
            pitch_support,
            axis="x",
            min_gap=0.056,
            max_gap=0.064,
            name="roll axis is nested forward of pitch axis",
        )
        ctx.expect_within(
            roll_cartridge,
            pitch_support,
            axes="yz",
            margin=0.002,
            name="roll cartridge stays nested inside pitch frame envelope",
        )

        housing_box = ctx.part_world_aabb(housing)
        yaw_box = ctx.part_world_aabb(yaw_stage)
        pitch_box = ctx.part_world_aabb(pitch_support)
        roll_box = ctx.part_world_aabb(roll_cartridge)
        taper_ok = False
        taper_details = "missing AABB data"
        if all(box is not None for box in (housing_box, yaw_box, pitch_box, roll_box)):
            housing_width = housing_box[1][1] - housing_box[0][1]
            yaw_width = yaw_box[1][1] - yaw_box[0][1]
            pitch_width = pitch_box[1][1] - pitch_box[0][1]
            roll_width = roll_box[1][1] - roll_box[0][1]
            taper_ok = housing_width > yaw_width > pitch_width > roll_width
            taper_details = (
                f"widths y: housing={housing_width:.3f}, yaw={yaw_width:.3f}, "
                f"pitch={pitch_width:.3f}, roll={roll_width:.3f}"
            )
        ctx.check("assembly tapers toward the output", taper_ok, taper_details)

        rest_output_pos = ctx.part_world_position(roll_cartridge)
        yawed_output_pos = None
        pitched_output_pos = None
        if rest_output_pos is not None:
            with ctx.pose(base_yaw=0.60, wrist_pitch=0.0, wrist_roll=0.0):
                yawed_output_pos = ctx.part_world_position(roll_cartridge)
            with ctx.pose(base_yaw=0.0, wrist_pitch=0.60, wrist_roll=0.0):
                pitched_output_pos = ctx.part_world_position(roll_cartridge)

        yaw_ok = yawed_output_pos is not None and yawed_output_pos[1] > 0.060
        ctx.check(
            "positive yaw swings output toward positive y",
            yaw_ok,
            details=(
                f"rest={rest_output_pos}, yawed={yawed_output_pos}; "
                "expected positive yaw to rotate the forward output toward +Y"
            ),
        )

        pitch_ok = (
            rest_output_pos is not None
            and pitched_output_pos is not None
            and pitched_output_pos[2] > rest_output_pos[2] + 0.020
        )
        ctx.check(
            "positive pitch lifts the output",
            pitch_ok,
            details=(
                f"rest={rest_output_pos}, pitched={pitched_output_pos}; "
                "expected positive pitch to raise the forward roll cartridge"
            ),
        )

        def elem_center(aabb):
            if aabb is None:
                return None
            return tuple((lo + hi) / 2.0 for lo, hi in zip(aabb[0], aabb[1]))

        encoder_rest = elem_center(ctx.part_element_world_aabb(roll_cartridge, elem="encoder_pod"))
        with ctx.pose(base_yaw=0.0, wrist_pitch=0.0, wrist_roll=1.0):
            encoder_rolled = elem_center(ctx.part_element_world_aabb(roll_cartridge, elem="encoder_pod"))
        roll_ok = (
            encoder_rest is not None
            and encoder_rolled is not None
            and encoder_rolled[1] < -0.010
            and encoder_rolled[2] < encoder_rest[2]
        )
        ctx.check(
            "positive roll turns the top pod toward negative y",
            roll_ok,
            details=(
                f"rest pod={encoder_rest}, rolled pod={encoder_rolled}; "
                "expected +X roll to carry the top-mounted pod down toward -Y"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
