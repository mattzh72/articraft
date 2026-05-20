from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


CUBE_SIZE = 0.065
HALF = CUBE_SIZE / 2.0


def _rounded_cube_mesh():
    return mesh_from_cadquery(
        cq.Workplane("XY")
        .box(CUBE_SIZE, CUBE_SIZE, CUBE_SIZE)
        .edges()
        .fillet(0.006),
        "rounded_cube",
        tolerance=0.00045,
        angular_tolerance=0.08,
    )


def _lever_paddle_mesh():
    return mesh_from_cadquery(
        cq.Workplane("XY")
        .box(0.006, 0.018, 0.032)
        .edges()
        .fillet(0.0014)
        .translate((-0.0002, 0.0, 0.018)),
        "toggle_paddle",
        tolerance=0.00025,
        angular_tolerance=0.08,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fidget_cube")

    model.material("soft_white", rgba=(0.88, 0.90, 0.86, 1.0))
    model.material("charcoal_rubber", rgba=(0.035, 0.038, 0.045, 1.0))
    model.material("dark_socket", rgba=(0.015, 0.017, 0.020, 1.0))
    model.material("hinge_pin", rgba=(0.13, 0.14, 0.15, 1.0))
    model.material("roller_blue", rgba=(0.08, 0.35, 0.95, 1.0))
    model.material("roller_mark", rgba=(1.0, 0.95, 0.35, 1.0))
    model.material("dial_teal", rgba=(0.02, 0.62, 0.68, 1.0))
    model.material("dial_mark", rgba=(0.88, 0.98, 1.0, 1.0))
    lever_materials = (
        model.material("lever_red", rgba=(0.88, 0.12, 0.10, 1.0)),
        model.material("lever_yellow", rgba=(0.95, 0.72, 0.12, 1.0)),
        model.material("lever_green", rgba=(0.18, 0.70, 0.25, 1.0)),
        model.material("lever_purple", rgba=(0.48, 0.22, 0.82, 1.0)),
    )

    cube = model.part("cube")
    cube.visual(
        _rounded_cube_mesh(),
        material="soft_white",
        name="rounded_cube",
    )

    panel_t = 0.0012
    panel_visible_t = 0.0010
    panel_w = 0.044
    panel_h = 0.048
    lug_t = 0.0038
    lug_w = 0.0042
    lug_h = 0.009
    barrel_len = 0.026
    lug_clearance = 0.00015
    hinge_z = -0.013

    # Dark inset-looking pads and static yoke cheeks on the four vertical faces.
    side_specs = (
        ("+x", 1, "x", 0.0),
        ("-x", -1, "x", math.pi),
        ("+y", 1, "y", math.pi / 2.0),
        ("-y", -1, "y", -math.pi / 2.0),
    )
    for i, (_label, sign, face_axis, _yaw) in enumerate(side_specs):
        if face_axis == "x":
            x = sign * (HALF + panel_t / 2.0 - (panel_t - panel_visible_t))
            cube.visual(
                Box((panel_t, panel_w, panel_h)),
                origin=Origin(xyz=(x, 0.0, 0.0)),
                material="charcoal_rubber",
                name=f"side_panel_{i}",
            )
            lug_x = sign * (HALF + panel_visible_t + lug_t / 2.0 - 0.0002)
            for j, y in enumerate(
                (
                    -(barrel_len / 2.0 + lug_w / 2.0 + lug_clearance),
                    +(barrel_len / 2.0 + lug_w / 2.0 + lug_clearance),
                )
            ):
                cube.visual(
                    Box((lug_t, lug_w, lug_h)),
                    origin=Origin(xyz=(lug_x, y, hinge_z)),
                    material="dark_socket",
                    name=f"hinge_cheek_{i}_{j}",
                )
        else:
            y = sign * (HALF + panel_t / 2.0 - (panel_t - panel_visible_t))
            cube.visual(
                Box((panel_w, panel_t, panel_h)),
                origin=Origin(xyz=(0.0, y, 0.0)),
                material="charcoal_rubber",
                name=f"side_panel_{i}",
            )
            lug_y = sign * (HALF + panel_visible_t + lug_t / 2.0 - 0.0002)
            for j, x in enumerate(
                (
                    -(barrel_len / 2.0 + lug_w / 2.0 + lug_clearance),
                    +(barrel_len / 2.0 + lug_w / 2.0 + lug_clearance),
                )
            ):
                cube.visual(
                    Box((lug_w, lug_t, lug_h)),
                    origin=Origin(xyz=(x, lug_y, hinge_z)),
                    material="dark_socket",
                    name=f"hinge_cheek_{i}_{j}",
                )

    cube.visual(
        mesh_from_geometry(TorusGeometry(0.013, 0.0026, radial_segments=40, tubular_segments=14), "top_socket_ring"),
        origin=Origin(xyz=(0.0, 0.0, HALF + 0.0024)),
        material="dark_socket",
        name="top_socket_ring",
    )
    cube.visual(
        mesh_from_geometry(TorusGeometry(0.021, 0.0024, radial_segments=44, tubular_segments=12), "bottom_dial_ring"),
        origin=Origin(xyz=(0.0, 0.0, -HALF - 0.0022)),
        material="dark_socket",
        name="bottom_dial_ring",
    )

    paddle_mesh = _lever_paddle_mesh()
    barrel_r = 0.0034
    hinge_normal = HALF + panel_visible_t + lug_t + lug_clearance + barrel_r
    for i, (_label, sign, face_axis, yaw) in enumerate(side_specs):
        toggle = model.part(f"toggle_{i}")
        toggle.visual(
            paddle_mesh,
            material=lever_materials[i],
            name="paddle",
        )
        toggle.visual(
            Cylinder(radius=barrel_r, length=barrel_len),
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material="hinge_pin",
            name="hinge_barrel",
        )

        if face_axis == "x":
            joint_xyz = (sign * hinge_normal, 0.0, hinge_z)
        else:
            joint_xyz = (0.0, sign * hinge_normal, hinge_z)
        model.articulation(
            f"cube_to_toggle_{i}",
            ArticulationType.REVOLUTE,
            parent=cube,
            child=toggle,
            origin=Origin(xyz=joint_xyz, rpy=(0.0, 0.0, yaw)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=5.0, lower=0.0, upper=0.72),
        )

    ball_r = 0.011
    roller_ball = model.part("roller_ball")
    roller_ball.visual(
        Sphere(ball_r),
        material="roller_blue",
        name="ball",
    )
    roller_ball.visual(
        Sphere(0.0023),
        origin=Origin(xyz=(0.0, 0.0, ball_r + 0.0019)),
        material="roller_mark",
        name="click_mark",
    )
    model.articulation(
        "cube_to_roller_ball",
        ArticulationType.REVOLUTE,
        parent=cube,
        child=roller_ball,
        origin=Origin(xyz=(0.0, 0.0, HALF + ball_r + 0.00035)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=8.0, lower=-1.2, upper=1.2),
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.035,
                0.006,
                body_style="cylindrical",
                edge_radius=0.0009,
                center=True,
            ),
            "smooth_dial_cap",
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.0036)),
        material="dial_teal",
        name="dial_cap",
    )
    dial.visual(
        Sphere(0.0024),
        origin=Origin(xyz=(0.0075, 0.0, -0.0082)),
        material="dial_mark",
        name="finger_mark",
    )
    model.articulation(
        "cube_to_dial",
        ArticulationType.REVOLUTE,
        parent=cube,
        child=dial,
        origin=Origin(xyz=(0.0, 0.0, -HALF)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=12.0, lower=-2.0 * math.pi, upper=2.0 * math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cube = object_model.get_part("cube")
    roller_ball = object_model.get_part("roller_ball")
    dial = object_model.get_part("dial")
    toggle_parts = [object_model.get_part(f"toggle_{i}") for i in range(4)]
    toggle_joints = [object_model.get_articulation(f"cube_to_toggle_{i}") for i in range(4)]
    ball_joint = object_model.get_articulation("cube_to_roller_ball")
    dial_joint = object_model.get_articulation("cube_to_dial")

    ctx.check("four side toggle levers", len(toggle_parts) == 4 and len(toggle_joints) == 4)
    ctx.expect_gap(
        roller_ball,
        cube,
        axis="z",
        min_gap=0.0,
        max_gap=0.002,
        positive_elem="ball",
        negative_elem="rounded_cube",
        name="roller ball clears top shell",
    )
    ctx.expect_gap(
        cube,
        dial,
        axis="z",
        min_gap=0.0,
        max_gap=0.002,
        positive_elem="rounded_cube",
        negative_elem="dial_cap",
        name="dial clears bottom shell",
    )

    axis_index = {"x": 0, "y": 1, "z": 2}

    def aabb_center(aabb, axis: str) -> float:
        idx = axis_index[axis]
        return (aabb[0][idx] + aabb[1][idx]) / 2.0

    # In every side orientation, the toggle's free paddle swings farther away
    # from the cube when the revolute hinge is actuated.
    toggle_motion_specs = (
        ("x", +1),
        ("x", -1),
        ("y", +1),
        ("y", -1),
    )
    for i, (axis, sign) in enumerate(toggle_motion_specs):
        rest = ctx.part_element_world_aabb(toggle_parts[i], elem="paddle")
        with ctx.pose({toggle_joints[i]: 0.65}):
            moved = ctx.part_element_world_aabb(toggle_parts[i], elem="paddle")
        idx = axis_index[axis]
        if sign > 0:
            ok = rest is not None and moved is not None and moved[1][idx] > rest[1][idx] + 0.004
        else:
            ok = rest is not None and moved is not None and moved[0][idx] < rest[0][idx] - 0.004
        ctx.check(f"toggle_{i} swings outward", ok, details=f"rest={rest}, moved={moved}")

    rest_mark = ctx.part_element_world_aabb(roller_ball, elem="click_mark")
    with ctx.pose({ball_joint: 0.85}):
        rolled_mark = ctx.part_element_world_aabb(roller_ball, elem="click_mark")
    ctx.check(
        "roller mark follows ball rotation",
        rest_mark is not None
        and rolled_mark is not None
        and abs(aabb_center(rolled_mark, "y") - aabb_center(rest_mark, "y")) > 0.005,
        details=f"rest={rest_mark}, rolled={rolled_mark}",
    )

    rest_finger = ctx.part_element_world_aabb(dial, elem="finger_mark")
    with ctx.pose({dial_joint: math.pi / 2.0}):
        spun_finger = ctx.part_element_world_aabb(dial, elem="finger_mark")
    ctx.check(
        "dial mark orbits on spin axis",
        rest_finger is not None
        and spun_finger is not None
        and abs(aabb_center(spun_finger, "y") - aabb_center(rest_finger, "y")) > 0.005,
        details=f"rest={rest_finger}, spun={spun_finger}",
    )

    return ctx.report()


object_model = build_object_model()
