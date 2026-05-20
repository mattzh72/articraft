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
    DomeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BODY_RADIUS = 0.045
LOWER_SKIRT_HEIGHT = 0.026
SHOULDER_RADIUS = 0.038
SHOULDER_HEIGHT = 0.012
COLLAR_RADIUS = 0.023
COLLAR_HEIGHT = 0.012
BODY_BOTTOM_Z = -0.018
BODY_TOP_Z = 0.020

CAP_RADIUS = 0.032
CAP_THICKNESS = 0.012
CAP_CENTER_Z = 0.0265
CAP_CROWN_RADIUS = 0.029
CAP_CROWN_Z_SCALE = 0.22
CAP_STEM_RADIUS = 0.012
CAP_STEM_LENGTH = 0.024
CAP_STEM_CENTER_Z = 0.010

DOOR_PANEL_SIZE = (0.038, 0.024, 0.002)
DOOR_PULL_SIZE = (0.007, 0.015, 0.003)
DOOR_JOINT_Z = BODY_BOTTOM_Z
DOOR_TRAVEL = 0.018


def _build_cap_crown_mesh():
    crown = DomeGeometry(
        radius=CAP_CROWN_RADIUS,
        radial_segments=36,
        height_segments=16,
        closed=True,
    )
    crown.scale(1.0, 1.0, CAP_CROWN_Z_SCALE)
    crown.translate(0.0, 0.0, CAP_CENTER_Z + (CAP_THICKNESS * 0.5))
    return mesh_from_geometry(crown, "space_mouse_cap_crown")


def _build_body_shell_mesh():
    shell = LatheGeometry.from_shell_profiles(
        [
            (0.044, BODY_BOTTOM_Z),
            (0.045, -0.010),
            (0.045, -0.004),
            (0.042, 0.008),
            (0.037, 0.014),
            (0.026, 0.018),
            (COLLAR_RADIUS, BODY_TOP_Z),
        ],
        [
            (0.028, BODY_BOTTOM_Z),
            (0.029, -0.010),
            (0.028, -0.004),
            (0.024, 0.008),
            (0.020, 0.014),
            (0.018, 0.018),
            (0.016, BODY_TOP_Z),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    return mesh_from_geometry(shell, "space_mouse_body_shell")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="space_mouse_controller")

    body_finish = model.material("body_finish", rgba=(0.18, 0.19, 0.21, 1.0))
    track_finish = model.material("track_finish", rgba=(0.10, 0.11, 0.12, 1.0))
    cap_finish = model.material("cap_finish", rgba=(0.68, 0.70, 0.74, 1.0))
    collar_finish = model.material("collar_finish", rgba=(0.14, 0.15, 0.16, 1.0))
    door_finish = model.material("door_finish", rgba=(0.13, 0.13, 0.14, 1.0))

    body = model.part("body")
    body.visual(
        _build_body_shell_mesh(),
        material=body_finish,
        name="body_shell",
    )
    body.visual(
        Box((0.064, 0.004, 0.002)),
        origin=Origin(xyz=(0.004, 0.015, -0.017)),
        material=track_finish,
        name="left_track",
    )
    body.visual(
        Box((0.064, 0.004, 0.002)),
        origin=Origin(xyz=(0.004, -0.015, -0.017)),
        material=track_finish,
        name="right_track",
    )
    body.visual(
        Box((0.006, 0.034, 0.002)),
        origin=Origin(xyz=(-0.022, 0.0, -0.017)),
        material=track_finish,
        name="rear_stop",
    )
    body.visual(
        Cylinder(radius=0.005, length=0.006),
        origin=Origin(xyz=(0.0, -0.016, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=collar_finish,
        name="left_bearing",
    )
    body.visual(
        Box((0.032, 0.006, 0.006)),
        origin=Origin(xyz=(0.011, -0.016, 0.0)),
        material=collar_finish,
        name="left_bearing_brace",
    )
    body.visual(
        Cylinder(radius=0.005, length=0.006),
        origin=Origin(xyz=(0.0, 0.016, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=collar_finish,
        name="right_bearing",
    )
    body.visual(
        Box((0.032, 0.006, 0.006)),
        origin=Origin(xyz=(0.011, 0.016, 0.0)),
        material=collar_finish,
        name="right_bearing_brace",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.090, 0.090, 0.040)),
        mass=0.82,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    cap = model.part("cap")
    cap.visual(
        Cylinder(radius=CAP_RADIUS, length=CAP_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, CAP_CENTER_Z)),
        material=cap_finish,
        name="cap_shell",
    )
    cap.visual(
        _build_cap_crown_mesh(),
        material=cap_finish,
        name="cap_crown",
    )
    cap.visual(
        Cylinder(radius=CAP_STEM_RADIUS, length=CAP_STEM_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, CAP_STEM_CENTER_Z)),
        material=collar_finish,
        name="cap_stem",
    )
    cap.visual(
        Cylinder(radius=0.014, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=collar_finish,
        name="gimbal_hub",
    )
    cap.visual(
        Cylinder(radius=0.0045, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=collar_finish,
        name="tilt_axle",
    )
    cap.inertial = Inertial.from_geometry(
        Box((0.068, 0.068, 0.040)),
        mass=0.16,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )

    battery_door = model.part("battery_door")
    battery_door.visual(
        Box(DOOR_PANEL_SIZE),
        origin=Origin(xyz=(0.0, 0.0, -0.001)),
        material=door_finish,
        name="door_panel",
    )
    battery_door.visual(
        Box(DOOR_PULL_SIZE),
        origin=Origin(xyz=(0.0195, 0.0, -0.0025)),
        material=door_finish,
        name="pull_tab",
    )
    battery_door.inertial = Inertial.from_geometry(
        Box((0.046, 0.028, 0.005)),
        mass=0.04,
        origin=Origin(xyz=(0.002, 0.0, -0.0015)),
    )

    model.articulation(
        "body_to_cap_tilt",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cap,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=-0.16,
            upper=0.16,
        ),
    )
    model.articulation(
        "body_to_battery_door",
        ArticulationType.PRISMATIC,
        parent=body,
        child=battery_door,
        origin=Origin(xyz=(0.0, 0.0, DOOR_JOINT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.06,
            lower=0.0,
            upper=DOOR_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    cap = object_model.get_part("cap")
    battery_door = object_model.get_part("battery_door")
    cap_tilt = object_model.get_articulation("body_to_cap_tilt")
    door_slide = object_model.get_articulation("body_to_battery_door")

    ctx.expect_gap(
        cap,
        body,
        axis="z",
        min_gap=0.0004,
        max_gap=0.003,
        positive_elem="cap_shell",
        negative_elem="body_shell",
        name="cap sits just above the collar",
    )
    ctx.expect_overlap(
        cap,
        body,
        axes="xy",
        min_overlap=0.045,
        elem_a="cap_shell",
        elem_b="body_shell",
        name="cap is centered over the body",
    )
    ctx.expect_gap(
        body,
        battery_door,
        axis="z",
        max_gap=0.0005,
        max_penetration=1e-5,
        positive_elem="rear_stop",
        negative_elem="door_panel",
        name="battery door nests against the underside track plane",
    )

    rest_cap_aabb = ctx.part_element_world_aabb(cap, elem="cap_shell")
    with ctx.pose({cap_tilt: cap_tilt.motion_limits.upper}):
        tilted_cap_aabb = ctx.part_element_world_aabb(cap, elem="cap_shell")
        ctx.expect_contact(
            cap,
            body,
            elem_a="cap_shell",
            elem_b="body_shell",
            contact_tol=1e-4,
            name="cap reaches a clean travel stop against the body rim",
        )
        ctx.check(
            "cap tilts about its horizontal hinge",
            rest_cap_aabb is not None
            and tilted_cap_aabb is not None
            and tilted_cap_aabb[0][2] < rest_cap_aabb[0][2] - 0.002,
            details=f"rest={rest_cap_aabb}, tilted={tilted_cap_aabb}",
        )

    rest_door_position = ctx.part_world_position(battery_door)
    with ctx.pose({door_slide: door_slide.motion_limits.upper}):
        extended_door_position = ctx.part_world_position(battery_door)
        ctx.expect_overlap(
            battery_door,
            body,
            axes="x",
            min_overlap=0.030,
            elem_a="door_panel",
            elem_b="left_track",
            name="battery door remains retained under the guide track",
        )
        ctx.check(
            "battery door slides outward",
            rest_door_position is not None
            and extended_door_position is not None
            and extended_door_position[0] > rest_door_position[0] + 0.012,
            details=f"rest={rest_door_position}, extended={extended_door_position}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
