from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os

_REAL_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _REAL_GETCWD()
    except FileNotFoundError:
        fallback_dir = __file__.rsplit("/", 1)[0] if "/" in __file__ else "/"
        try:
            os.chdir(fallback_dir or "/")
        except FileNotFoundError:
            os.chdir("/")
        return _REAL_GETCWD()


os.getcwd = _safe_getcwd
_safe_getcwd()

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

FRAME_LENGTH = 0.28
FRAME_WIDTH = 0.20
FRAME_THICKNESS = 0.022
RAIL_WIDTH = 0.026
CROSSMEMBER_WIDTH = 0.028
HINGE_Z = 0.078
POD_AXLE_ORIGIN = (0.056, 0.0, 0.150)
CRUISE_TILT = -0.75

CORNERS = {
    "front_left": ((0.260, 0.180, HINGE_Z), (FRAME_LENGTH / 2.0, FRAME_WIDTH / 2.0, 0.0)),
    "rear_left": ((-0.260, 0.180, HINGE_Z), (-FRAME_LENGTH / 2.0, FRAME_WIDTH / 2.0, 0.0)),
    "front_right": ((0.260, -0.180, HINGE_Z), (FRAME_LENGTH / 2.0, -FRAME_WIDTH / 2.0, 0.0)),
    "rear_right": ((-0.260, -0.180, HINGE_Z), (-FRAME_LENGTH / 2.0, -FRAME_WIDTH / 2.0, 0.0)),
}


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilt_rotor_quadrotor")

    carbon = model.material("carbon", rgba=(0.12, 0.13, 0.15, 1.0))
    graphite = model.material("graphite", rgba=(0.20, 0.22, 0.24, 1.0))
    aluminum = model.material("aluminum", rgba=(0.66, 0.69, 0.73, 1.0))
    rotor_black = model.material("rotor_black", rgba=(0.06, 0.06, 0.07, 1.0))
    accent = model.material("accent_blue", rgba=(0.24, 0.42, 0.62, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((FRAME_LENGTH, RAIL_WIDTH, FRAME_THICKNESS)),
        origin=Origin(xyz=(0.0, FRAME_WIDTH / 2.0, FRAME_THICKNESS / 2.0)),
        material=carbon,
        name="left_rail",
    )
    frame.visual(
        Box((FRAME_LENGTH, RAIL_WIDTH, FRAME_THICKNESS)),
        origin=Origin(xyz=(0.0, -FRAME_WIDTH / 2.0, FRAME_THICKNESS / 2.0)),
        material=carbon,
        name="right_rail",
    )
    frame.visual(
        Box((CROSSMEMBER_WIDTH, FRAME_WIDTH, FRAME_THICKNESS)),
        origin=Origin(xyz=(0.130, 0.0, FRAME_THICKNESS / 2.0)),
        material=carbon,
        name="front_crossmember",
    )
    frame.visual(
        Box((CROSSMEMBER_WIDTH, FRAME_WIDTH, FRAME_THICKNESS)),
        origin=Origin(xyz=(-0.130, 0.0, FRAME_THICKNESS / 2.0)),
        material=carbon,
        name="rear_crossmember",
    )

    for corner_name, (hinge_xyz, frame_corner) in CORNERS.items():
        hx, hy, hz = hinge_xyz
        cx, cy, _ = frame_corner
        arm_dx = hx - cx
        arm_dy = hy - cy
        arm_span = math.hypot(arm_dx, arm_dy)
        unit_x = arm_dx / arm_span
        unit_y = arm_dy / arm_span
        arm_tip_x = hx - unit_x * 0.032
        arm_tip_y = hy - unit_y * 0.032
        arm_length = math.hypot(arm_tip_x - cx, arm_tip_y - cy)
        arm_angle = math.atan2(arm_dy, arm_dx)
        arm_mid = ((arm_tip_x + cx) / 2.0, (arm_tip_y + cy) / 2.0, FRAME_THICKNESS / 2.0)
        lateral_x = -unit_y
        lateral_y = unit_x

        frame.visual(
            Box((arm_length, 0.024, 0.022)),
            origin=Origin(xyz=arm_mid, rpy=(0.0, 0.0, arm_angle)),
            material=carbon,
            name=f"{corner_name}_arm",
        )
        frame.visual(
            Box((0.028, 0.036, 0.060)),
            origin=Origin(
                xyz=(hx - unit_x * 0.018, hy - unit_y * 0.018, 0.040),
                rpy=(0.0, 0.0, arm_angle),
            ),
            material=graphite,
            name=f"{corner_name}_riser",
        )
        frame.visual(
            Box((0.026, 0.030, 0.010)),
            origin=Origin(
                xyz=(hx - unit_x * 0.014, hy - unit_y * 0.014, hz - 0.020),
                rpy=(0.0, 0.0, arm_angle),
            ),
            material=graphite,
            name=f"{corner_name}_hinge_mount",
        )
        frame.visual(
            Cylinder(radius=0.0045, length=0.024),
            origin=Origin(xyz=(hx, hy, hz), rpy=(math.pi / 2.0, 0.0, arm_angle)),
            material=aluminum,
            name=f"{corner_name}_hinge_axle",
        )

    frame.inertial = Inertial.from_geometry(
        Box((0.60, 0.42, 0.18)),
        mass=2.1,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
    )

    for corner_name, (hinge_xyz, _) in CORNERS.items():
        hx, hy, _ = hinge_xyz
        arm_angle = math.atan2(hy, hx)
        pod = model.part(f"{corner_name}_pod")
        pod.visual(
            Box((0.020, 0.028, 0.010)),
            origin=Origin(xyz=(0.010, 0.0, -0.010)),
            material=aluminum,
            name="hinge_web",
        )
        pod.visual(
            Box((0.018, 0.004, 0.024)),
            origin=Origin(xyz=(0.009, 0.014, 0.0)),
            material=aluminum,
            name="hinge_ear_pos",
        )
        pod.visual(
            Box((0.018, 0.004, 0.024)),
            origin=Origin(xyz=(0.009, -0.014, 0.0)),
            material=aluminum,
            name="hinge_ear_neg",
        )
        pod.visual(
            Box((0.042, 0.022, 0.018)),
            origin=Origin(xyz=(0.034, 0.0, 0.003)),
            material=accent,
            name="support_boom",
        )
        pod.visual(
            Box((0.038, 0.032, 0.024)),
            origin=Origin(xyz=(0.052, 0.0, 0.018)),
            material=accent,
            name="pod_fairing",
        )
        pod.visual(
            Cylinder(radius=0.011, length=0.112),
            origin=Origin(xyz=(0.056, 0.0, 0.080)),
            material=accent,
            name="mast",
        )
        pod.visual(
            Cylinder(radius=0.025, length=0.036),
            origin=Origin(xyz=(0.056, 0.0, 0.118)),
            material=graphite,
            name="motor_can",
        )
        pod.visual(
            Cylinder(radius=0.013, length=0.010),
            origin=Origin(xyz=(0.056, 0.0, 0.141)),
            material=aluminum,
            name="motor_cap",
        )
        pod.visual(
            Cylinder(radius=0.011, length=0.004),
            origin=Origin(xyz=(0.056, 0.0, POD_AXLE_ORIGIN[2] - 0.002)),
            material=aluminum,
            name="axle_seat",
        )
        pod.inertial = Inertial.from_geometry(
            Box((0.115, 0.070, 0.170)),
            mass=0.22,
            origin=Origin(xyz=(0.056, 0.0, 0.082)),
        )

        prop = model.part(f"{corner_name}_prop")
        prop.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, 0.003)),
            material=aluminum,
            name="hub",
        )
        prop.visual(
            Box((0.122, 0.018, 0.003)),
            origin=Origin(xyz=(0.070, 0.0, 0.0075)),
            material=rotor_black,
            name="blade_pos",
        )
        prop.visual(
            Box((0.122, 0.018, 0.003)),
            origin=Origin(xyz=(-0.070, 0.0, 0.0075)),
            material=rotor_black,
            name="blade_neg",
        )
        prop.inertial = Inertial.from_geometry(
            Box((0.270, 0.028, 0.012)),
            mass=0.035,
            origin=Origin(xyz=(0.0, 0.0, 0.006)),
        )

        model.articulation(
            f"frame_to_{corner_name}_pod",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=pod,
            origin=Origin(xyz=hinge_xyz, rpy=(0.0, 0.0, arm_angle)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=2.5,
                lower=-0.80,
                upper=0.15,
            ),
        )
        model.articulation(
            f"{corner_name}_pod_to_prop",
            ArticulationType.CONTINUOUS,
            parent=pod,
            child=prop,
            origin=Origin(xyz=POD_AXLE_ORIGIN),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=1.0, velocity=55.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    left_rail = frame.get_visual("left_rail")
    right_rail = frame.get_visual("right_rail")
    front_crossmember = frame.get_visual("front_crossmember")
    rear_crossmember = frame.get_visual("rear_crossmember")
    pods = {corner: object_model.get_part(f"{corner}_pod") for corner in CORNERS}
    props = {corner: object_model.get_part(f"{corner}_prop") for corner in CORNERS}
    tilt_joints = {
        corner: object_model.get_articulation(f"frame_to_{corner}_pod") for corner in CORNERS
    }
    prop_joints = {
        corner: object_model.get_articulation(f"{corner}_pod_to_prop") for corner in CORNERS
    }

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    ctx.fail_if_isolated_parts()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_gap(
        frame,
        frame,
        axis="y",
        min_gap=0.17,
        positive_elem=left_rail,
        negative_elem=right_rail,
        name="frame has a wide open center between side rails",
    )
    ctx.expect_gap(
        frame,
        frame,
        axis="x",
        min_gap=0.22,
        positive_elem=front_crossmember,
        negative_elem=rear_crossmember,
        name="frame has a rectangular opening between front and rear crossmembers",
    )

    hover_pose = {joint: 0.0 for joint in tilt_joints.values()}
    for corner in CORNERS:
        pod = pods[corner]
        prop = props[corner]
        hinge_web = pod.get_visual("hinge_web")
        hinge_ear_pos = pod.get_visual("hinge_ear_pos")
        hinge_ear_neg = pod.get_visual("hinge_ear_neg")
        axle_seat = pod.get_visual("axle_seat")
        hub = prop.get_visual("hub")
        hinge_axle = frame.get_visual(f"{corner}_hinge_axle")
        hinge_mount = frame.get_visual(f"{corner}_hinge_mount")

        ctx.allow_overlap(
            frame,
            pod,
            elem_a=hinge_axle,
            elem_b=hinge_ear_pos,
            reason=f"{corner} tilt hinge axle intentionally passes through the upper hinge ear as a real pinned joint",
        )
        ctx.allow_overlap(
            frame,
            pod,
            elem_a=hinge_axle,
            elem_b=hinge_ear_neg,
            reason=f"{corner} tilt hinge axle intentionally passes through the lower hinge ear as a real pinned joint",
        )

        ctx.expect_origin_distance(
            pod,
            frame,
            axes="xy",
            min_dist=0.22,
            max_dist=0.36,
            name=f"{corner} pod sits out near the end of its arm",
        )
        ctx.expect_gap(
            pod,
            frame,
            axis="z",
            positive_elem=hinge_web,
            negative_elem=hinge_mount,
            max_gap=0.006,
            max_penetration=0.0,
            name=f"{corner} hinge web sits just above its frame mount block",
        )
        ctx.expect_contact(
            pod,
            frame,
            elem_a=hinge_ear_pos,
            elem_b=hinge_axle,
            name=f"{corner} upper hinge ear is pinned on the frame axle",
        )
        ctx.expect_contact(
            pod,
            frame,
            elem_a=hinge_ear_neg,
            elem_b=hinge_axle,
            name=f"{corner} lower hinge ear is pinned on the frame axle",
        )
        ctx.expect_contact(
            prop,
            pod,
            elem_a=hub,
            elem_b=axle_seat,
            name=f"{corner} propeller hub seats on the pod spindle cap",
        )
        ctx.expect_gap(
            prop,
            pod,
            axis="z",
            positive_elem=hub,
            negative_elem=axle_seat,
            max_gap=0.0005,
            max_penetration=0.0,
            name=f"{corner} propeller hub sits flush on its spindle cap",
        )
        ctx.expect_overlap(
            prop,
            pod,
            axes="xy",
            elem_a=hub,
            elem_b=axle_seat,
            min_overlap=0.018,
            name=f"{corner} propeller hub stays centered over the spindle cap",
        )

    with ctx.pose(hover_pose):
        for corner in CORNERS:
            if "front" in corner:
                ctx.expect_origin_gap(
                    props[corner],
                    frame,
                    axis="x",
                    min_gap=0.24,
                    name=f"{corner} propeller sits forward of the rectangular frame",
                )
            else:
                ctx.expect_origin_gap(
                    frame,
                    props[corner],
                    axis="x",
                    min_gap=0.24,
                    name=f"{corner} propeller sits aft of the rectangular frame",
                )
            if "left" in corner:
                ctx.expect_origin_gap(
                    props[corner],
                    frame,
                    axis="y",
                    min_gap=0.15,
                    name=f"{corner} propeller sits outboard on the left side",
                )
            else:
                ctx.expect_origin_gap(
                    frame,
                    props[corner],
                    axis="y",
                    min_gap=0.15,
                    name=f"{corner} propeller sits outboard on the right side",
                )
            ctx.expect_gap(
                props[corner],
                frame,
                axis="z",
                min_gap=0.12,
                name=f"{corner} hover propeller clears the frame vertically",
            )

    spin_pose = {
        prop_joints["front_left"]: 0.0,
        prop_joints["rear_left"]: 0.9,
        prop_joints["front_right"]: 1.8,
        prop_joints["rear_right"]: 2.7,
    }
    with ctx.pose(spin_pose):
        for corner in CORNERS:
            ctx.expect_contact(
                props[corner],
                pods[corner],
                elem_a=props[corner].get_visual("hub"),
                elem_b=pods[corner].get_visual("axle_seat"),
                name=f"{corner} spinning propeller remains captured by the pod spindle cap",
            )
            ctx.expect_gap(
                props[corner],
                frame,
                axis="z",
                min_gap=0.12,
                name=f"{corner} spinning propeller still clears the frame in hover",
            )

    cruise_pose = {joint: CRUISE_TILT for joint in tilt_joints.values()}
    cruise_pose.update(spin_pose)
    with ctx.pose(cruise_pose):
        for corner in CORNERS:
            ctx.expect_contact(
                props[corner],
                pods[corner],
                elem_a=props[corner].get_visual("hub"),
                elem_b=pods[corner].get_visual("axle_seat"),
                name=f"{corner} tilted propeller remains seated on the pod spindle cap",
            )
        ctx.fail_if_parts_overlap_in_current_pose(
            name="all tilted propellers clear the frame in cruise pose"
        )
        ctx.fail_if_isolated_parts(name="all tilted pods and props remain mounted in cruise pose")

    for corner, joint in tilt_joints.items():
        limits = joint.motion_limits
        if limits is None or limits.lower is None or limits.upper is None:
            continue
        with ctx.pose({joint: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(
                name=f"{corner} tilt lower limit stays collision free"
            )
            ctx.fail_if_isolated_parts(name=f"{corner} tilt lower limit stays mounted")
        with ctx.pose({joint: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(
                name=f"{corner} tilt upper limit stays collision free"
            )
            ctx.fail_if_isolated_parts(name=f"{corner} tilt upper limit stays mounted")
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
